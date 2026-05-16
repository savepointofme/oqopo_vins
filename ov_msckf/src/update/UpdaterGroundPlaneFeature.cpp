/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "UpdaterGroundPlaneFeature.h"

#include <Eigen/Dense>
#include <algorithm>

#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

using namespace ov_msckf;

UpdaterGroundPlaneFeature::UpdaterGroundPlaneFeature(double sigma_pixel,
                                                     int max_features,
                                                     double center_frac,
                                                     double min_cos_tilt,
                                                     double max_residual_px,
                                                     double min_lambda,
                                                     double max_lambda)
    : sigma_pixel_(sigma_pixel),
      max_features_(max_features),
      center_frac_(center_frac),
      min_cos_tilt_(min_cos_tilt),
      max_residual_px_(max_residual_px),
      min_lambda_(min_lambda),
      max_lambda_(max_lambda) {}

void UpdaterGroundPlaneFeature::reset() {
  last_ = LastUpdate();
  stats_ = Stats();
}

namespace {

// Convenience: extract pose/extrinsics for a given clone timestamp and cam id.
// Returns false if the clone is missing.
bool get_camera_pose(const std::shared_ptr<ov_msckf::State> &state,
                     double t_clone, size_t cam_id,
                     Eigen::Matrix3d &R_GtoC,
                     Eigen::Vector3d &p_CinG) {
  auto it = state->_clones_IMU.find(t_clone);
  if (it == state->_clones_IMU.end()) {
    return false;
  }
  Eigen::Matrix3d R_GtoI = it->second->Rot();
  Eigen::Vector3d p_IinG = it->second->pos();
  auto calib_it = state->_calib_IMUtoCAM.find(cam_id);
  if (calib_it == state->_calib_IMUtoCAM.end()) {
    return false;
  }
  Eigen::Matrix3d R_ItoC = calib_it->second->Rot();
  Eigen::Vector3d p_IinC = calib_it->second->pos();
  R_GtoC = R_ItoC * R_GtoI;
  // p_CinG = p_IinG + R_GtoI^T * p_CinI, with p_CinI = -R_ItoC^T * p_IinC.
  p_CinG = p_IinG - R_GtoI.transpose() * R_ItoC.transpose() * p_IinC;
  return true;
}

// Look up the (u,v) pixel of a feature at the given timestamp; returns false
// if no observation at that time / cam.  cam_id_out receives the matching cam.
bool find_uv_at(const std::shared_ptr<ov_core::Feature> &feat, double t_query,
                Eigen::Vector2d &uv_out, size_t &cam_id_out) {
  for (auto const &kv : feat->timestamps) {
    size_t cam = kv.first;
    auto const &ts = kv.second;
    auto const &uvs = feat->uvs.at(cam);
    for (size_t k = 0; k < ts.size(); k++) {
      if (std::fabs(ts[k] - t_query) < 1e-6) {
        uv_out << uvs[k](0), uvs[k](1);
        cam_id_out = cam;
        return true;
      }
    }
  }
  return false;
}

} // namespace

bool UpdaterGroundPlaneFeature::try_update(
    std::shared_ptr<ov_msckf::State> state,
    std::shared_ptr<ov_core::FeatureDatabase> features,
    double t_state, double z_ground) {

  using namespace ov_type;

  stats_.n_called++;
  if (stats_.first_t < 0) stats_.first_t = t_state;
  stats_.last_t = t_state;
  last_ = LastUpdate();
  last_.t = t_state;

  // ---- guard: enough clones ----
  if (state->_clones_IMU.size() < 2) {
    stats_.n_skipped_no_clones++;
    last_.decision = "SKIP_NO_CLONES";
    return false;
  }

  // ---- guard: tilt ----
  double cos_tilt = state->_imu->Rot()(2, 2);
  if (std::fabs(cos_tilt) < min_cos_tilt_) {
    stats_.n_skipped_tilt++;
    last_.decision = "SKIP_TILT";
    return false;
  }

  // ---- choose anchor (oldest clone) and current clone ----
  double t_anchor = state->_clones_IMU.begin()->first;
  if (std::fabs(t_anchor - t_state) < 1e-6) {
    stats_.n_skipped_no_clones++;
    last_.decision = "SKIP_ANCHOR=CURRENT";
    return false;
  }

  // ---- find features observed at both anchor AND current ----
  auto feats_curr = features->features_containing(t_state, false, true);
  if (feats_curr.empty()) {
    stats_.n_skipped_no_features++;
    last_.decision = "SKIP_NO_FEATURES";
    return false;
  }

  // ---- build measurement system ----
  std::shared_ptr<PoseJPL> curr_clone = state->_clones_IMU.at(t_state);
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(curr_clone->q());
  Hx_order.push_back(curr_clone->p());

  std::vector<Eigen::Matrix<double, 2, 6>> H_blocks;
  std::vector<Eigen::Vector2d> res_blocks;

  int n_candidates = 0;
  int n_pass_gate = 0;
  double sum_abs_res = 0.0;
  double max_abs_res = 0.0;

  for (const auto &feat : feats_curr) {
    // -- find anchor & current pixels --
    Eigen::Vector2d uv_anchor, uv_curr;
    size_t cam_id_anchor = 0, cam_id_curr = 0;
    if (!find_uv_at(feat, t_anchor, uv_anchor, cam_id_anchor)) continue;
    if (!find_uv_at(feat, t_state, uv_curr, cam_id_curr)) continue;
    if (cam_id_anchor != cam_id_curr) continue; // single-cam only for v0
    size_t cam_id = cam_id_curr;
    n_candidates++;

    auto cam_it = state->_cam_intrinsics_cameras.find(cam_id);
    if (cam_it == state->_cam_intrinsics_cameras.end()) continue;
    auto cam = cam_it->second;

    // -- center-region gate (anchor pixel) --
    int W = cam->w();
    int H = cam->h();
    double border_x = 0.5 * (1.0 - center_frac_) * W;
    double border_y = 0.5 * (1.0 - center_frac_) * H;
    if (uv_anchor(0) < border_x || uv_anchor(0) > W - border_x ||
        uv_anchor(1) < border_y || uv_anchor(1) > H - border_y) {
      continue;
    }

    // -- compute world ray from anchor pose --
    Eigen::Matrix3d R_GtoCa;
    Eigen::Vector3d p_CainG;
    if (!get_camera_pose(state, t_anchor, cam_id, R_GtoCa, p_CainG)) continue;

    Eigen::Vector2d uv_n = cam->undistort_d(uv_anchor);
    Eigen::Vector3d u_C(uv_n(0), uv_n(1), 1.0);
    Eigen::Vector3d u_G = R_GtoCa.transpose() * u_C; // ray in world frame
    // Normalise direction so |u_G| ~ 1 (numerical hygiene).
    u_G.normalize();

    // -- ground-plane intersection: z = z_ground --
    if (std::fabs(u_G.z()) < 0.2) continue;          // ray near horizontal
    double lambda = (z_ground - p_CainG.z()) / u_G.z();
    if (lambda < min_lambda_ || lambda > max_lambda_) continue;
    Eigen::Vector3d p_FinG = p_CainG + lambda * u_G;

    // -- predict pixel at current clone --
    Eigen::Matrix3d R_GtoCc;
    Eigen::Vector3d p_CcinG;
    if (!get_camera_pose(state, t_state, cam_id, R_GtoCc, p_CcinG)) continue;
    Eigen::Vector3d p_FinCc = R_GtoCc * (p_FinG - p_CcinG);
    if (p_FinCc.z() < 0.1) continue; // behind / on top of camera

    Eigen::Vector2d uv_pred_n(p_FinCc.x() / p_FinCc.z(), p_FinCc.y() / p_FinCc.z());
    Eigen::Vector2d uv_pred = cam->distort_d(uv_pred_n);

    Eigen::Vector2d res = uv_curr - uv_pred;
    double r_norm = res.norm();
    if (r_norm > max_residual_px_) continue;
    n_pass_gate++;

    // -- Jacobian: dh/d(curr_clone_q,p) --
    // d(uv) / d(uv_n) — from camera intrinsics
    Eigen::MatrixXd H_dz_dzn, H_dz_dzeta;
    cam->compute_distort_jacobian(uv_pred_n, H_dz_dzn, H_dz_dzeta);

    // d(uv_n) / d(p_FinCc)
    double zc = p_FinCc.z();
    double zc2 = zc * zc;
    Eigen::Matrix<double, 2, 3> H_dzn_dpfc;
    H_dzn_dpfc << 1.0 / zc, 0.0, -p_FinCc.x() / zc2,
                  0.0, 1.0 / zc, -p_FinCc.y() / zc2;

    // d(p_FinCc) / d(clone_pose) — current clone is the only state we update
    // p_FinCc = R_ItoC * R_GtoIc * (p_FinG - p_IcinG) + p_IinC
    // ∂p_FinCc / ∂θ_clone = R_ItoC * skew(p_FinIc)
    // ∂p_FinCc / ∂p_clone = -R_ItoC * R_GtoIc
    auto calib = state->_calib_IMUtoCAM.at(cam_id);
    Eigen::Matrix3d R_ItoC = calib->Rot();
    Eigen::Matrix3d R_GtoIc = curr_clone->Rot();
    Eigen::Vector3d p_IcinG = curr_clone->pos();
    Eigen::Vector3d p_FinIc = R_GtoIc * (p_FinG - p_IcinG);

    Eigen::Matrix<double, 3, 6> H_pfc_clone;
    H_pfc_clone.block<3, 3>(0, 0) = R_ItoC * ov_core::skew_x(p_FinIc);
    H_pfc_clone.block<3, 3>(0, 3) = -R_ItoC * R_GtoIc;

    Eigen::Matrix<double, 2, 6> H_block =
        H_dz_dzn * H_dzn_dpfc * H_pfc_clone;
    H_blocks.push_back(H_block);
    res_blocks.push_back(res);

    sum_abs_res += r_norm;
    if (r_norm > max_abs_res) max_abs_res = r_norm;

    if ((int)H_blocks.size() >= max_features_) break;
  }

  last_.n_candidates = n_candidates;
  last_.n_passed_gate = n_pass_gate;
  last_.n_used = (int)H_blocks.size();

  if (H_blocks.empty()) {
    stats_.n_skipped_no_features++;
    last_.decision = "SKIP_NO_PASS";
    PRINT_DEBUG(YELLOW "[GPLANE-FEAT] t=%.3f cand=%d gate=%d used=0 — skip\n" RESET,
                t_state, n_candidates, n_pass_gate);
    return false;
  }

  // -- stack H and residuals --
  int m = (int)H_blocks.size() * 2;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m, 6);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m);
  for (size_t i = 0; i < H_blocks.size(); i++) {
    H.block(2 * i, 0, 2, 6) = H_blocks[i];
    res.segment(2 * i, 2) = res_blocks[i];
  }
  Eigen::MatrixXd R = (sigma_pixel_ * sigma_pixel_) *
                      Eigen::MatrixXd::Identity(m, m);

  // -- snapshot mean to measure actual delta --
  Eigen::Vector3d p_pre = state->_imu->pos();

  ov_msckf::StateHelper::EKFUpdate(state, Hx_order, H, res, R);

  Eigen::Vector3d p_post = state->_imu->pos();
  Eigen::Vector3d dp = p_post - p_pre;
  double dxy_norm = std::sqrt(dp.x() * dp.x() + dp.y() * dp.y());

  last_.mean_residual_px = sum_abs_res / std::max(1, (int)H_blocks.size());
  last_.max_residual_px = max_abs_res;
  last_.dxy_norm = dxy_norm;
  last_.dz_after = dp.z();
  last_.decision = "ACCEPT";

  stats_.n_accepted_updates++;
  stats_.n_features_used_total += H_blocks.size();
  stats_.sum_residual_px += sum_abs_res;
  stats_.sum_dxy_norm += dxy_norm;
  stats_.sum_dz_after += std::fabs(dp.z());

  PRINT_INFO(CYAN "[GPLANE-FEAT] t=%.3f cand=%d gate=%d used=%d "
             "|res|_mu=%.2fpx max=%.2fpx |dxy|=%.3fm dz=%+.3fm\n" RESET,
             t_state, n_candidates, n_pass_gate, (int)H_blocks.size(),
             last_.mean_residual_px, last_.max_residual_px, dxy_norm, dp.z());

  if (t_state - stats_.last_summary_t > 30.0) {
    stats_.last_summary_t = t_state;
    size_t na = std::max((size_t)1, stats_.n_accepted_updates);
    PRINT_INFO(CYAN "[GPLANE-FEAT-STAT] t=%.1f called=%zu acc=%zu skip_tilt=%zu "
               "skip_noclone=%zu skip_nofeat=%zu feats/upd=%.1f |res|_mu=%.2fpx "
               "|dxy|_mu=%.3fm |dz|_mu=%.3fm\n" RESET,
               t_state, stats_.n_called, stats_.n_accepted_updates,
               stats_.n_skipped_tilt, stats_.n_skipped_no_clones,
               stats_.n_skipped_no_features,
               (double)stats_.n_features_used_total / na,
               stats_.sum_residual_px /
                   std::max((size_t)1, stats_.n_features_used_total),
               stats_.sum_dxy_norm / na, stats_.sum_dz_after / na);
  }

  return true;
}

void UpdaterGroundPlaneFeature::print_summary() const {
  size_t na = std::max((size_t)1, stats_.n_accepted_updates);
  PRINT_INFO(GREEN "[GPLANE-FEAT-FINAL] ===== Ground-Plane Feature Summary =====\n" RESET);
  PRINT_INFO(GREEN "[GPLANE-FEAT-FINAL] called=%zu acc=%zu skip_tilt=%zu "
             "skip_noclone=%zu skip_nofeat=%zu\n" RESET,
             stats_.n_called, stats_.n_accepted_updates,
             stats_.n_skipped_tilt, stats_.n_skipped_no_clones,
             stats_.n_skipped_no_features);
  PRINT_INFO(GREEN "[GPLANE-FEAT-FINAL] features/update=%.2f  |res|_mu=%.2fpx  "
             "|dxy|_mu=%.3fm  |dz|_mu=%.3fm\n" RESET,
             (double)stats_.n_features_used_total / na,
             stats_.sum_residual_px /
                 std::max((size_t)1, stats_.n_features_used_total),
             stats_.sum_dxy_norm / na, stats_.sum_dz_after / na);
  PRINT_INFO(GREEN "[GPLANE-FEAT-FINAL] ============================================\n" RESET);
}
