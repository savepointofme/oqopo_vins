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

#include "UpdaterGroundPlaneFeatureV1.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

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

// ---------------------------------------------------------------------------
// OpenVINS JPL convention used throughout:
//   R_GtoI(perturbed) = (I - skew(δθ)) * R_GtoI(estimate)
//   p(perturbed)      = p(estimate) + δp
// Resulting derivative rules:
//   ∂(R    * v) / ∂δθ = +skew(R   * v)         (for fixed v in G)
//   ∂(R^T  * w) / ∂δθ = -R^T * skew(w)         (for fixed w in I/C)
//   ∂p / ∂δp = I_3
// State order in Hx_order: [anchor_q, anchor_p, current_q, current_p]
// Column layout:           [   0..2 ,   3..5  ,   6..8   ,    9..11 ]
// ---------------------------------------------------------------------------

UpdaterGroundPlaneFeatureV1::UpdaterGroundPlaneFeatureV1(
    Mode mode, double sigma_pixel, int max_features, double center_frac,
    double min_cos_tilt, double max_residual_px, double min_lambda,
    double max_lambda, double fd_step_rot, double fd_step_pos,
    double fd_rel_tol_rot, double fd_rel_tol_pos, double fd_max_abs_rel_tol,
    bool exclude_used_from_msckf, int dump_first_n)
    : mode_(mode),
      sigma_pixel_(sigma_pixel),
      max_features_(max_features),
      center_frac_(center_frac),
      min_cos_tilt_(min_cos_tilt),
      max_residual_px_(max_residual_px),
      min_lambda_(min_lambda),
      max_lambda_(max_lambda),
      fd_step_rot_(fd_step_rot),
      fd_step_pos_(fd_step_pos),
      fd_rel_tol_rot_(fd_rel_tol_rot),
      fd_rel_tol_pos_(fd_rel_tol_pos),
      fd_max_abs_rel_tol_(fd_max_abs_rel_tol),
      exclude_used_from_msckf_(exclude_used_from_msckf),
      dump_remaining_(dump_first_n) {}

void UpdaterGroundPlaneFeatureV1::reset() {
  last_ = LastUpdate();
  stats_ = Stats();
  last_used_ids_.clear();
}

namespace {

// Find first observation of feat at timestamp t_query.
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

// ---- Forward model -------------------------------------------------------
//
// Inputs: anchor pose (R_GtoIa, p_IainG), current pose (R_GtoIc, p_IcinG),
//         camera extrinsics (R_ItoC, p_IinC), anchor pixel uv_a, ground
//         plane z = z_ground, and the camera intrinsics for distortion.
//
// Outputs: predicted current pixel uv_pred, plus intermediates needed for
//          the analytic Jacobian (u_C, u_G_raw, lambda, p_FinG, p_FinCc).
//
// Returns false if the geometry is degenerate (ray horizontal, depth
// invalid, etc.).
struct ForwardOut {
  Eigen::Vector3d u_C;       // (u_n, v_n, 1) — normalised, undistorted anchor ray (camera frame)
  Eigen::Vector3d u_G_raw;   // R_GtoIa^T * R_ItoC^T * u_C, NOT normalised
  double lambda;
  Eigen::Vector3d p_FinG;
  Eigen::Vector3d p_FinIc;   // R_GtoIc * (p_FinG - p_IcinG)
  Eigen::Vector3d p_FinCc;   // R_ItoC * p_FinIc + p_IinC
  Eigen::Vector2d uv_n_pred; // p_FinCc.head(2) / p_FinCc.z()
  Eigen::Vector2d uv_pred;   // after distort_d
};

bool forward(const Eigen::Matrix3d &R_GtoIa, const Eigen::Vector3d &p_IainG,
             const Eigen::Matrix3d &R_GtoIc, const Eigen::Vector3d &p_IcinG,
             const Eigen::Matrix3d &R_ItoC,  const Eigen::Vector3d &p_IinC,
             const Eigen::Vector2d &uv_anchor_undistorted,
             double z_ground, double min_lambda, double max_lambda,
             const std::shared_ptr<ov_core::CamBase> &cam,
             ForwardOut &out) {
  out.u_C << uv_anchor_undistorted(0), uv_anchor_undistorted(1), 1.0;
  out.u_G_raw = R_GtoIa.transpose() * R_ItoC.transpose() * out.u_C;
  if (std::fabs(out.u_G_raw.z()) < 0.2) return false;

  // p_CainG = p_IainG - R_GtoIa^T * R_ItoC^T * p_IinC
  Eigen::Vector3d p_CainG =
      p_IainG - R_GtoIa.transpose() * R_ItoC.transpose() * p_IinC;
  out.lambda = (z_ground - p_CainG.z()) / out.u_G_raw.z();
  if (out.lambda < min_lambda || out.lambda > max_lambda) return false;

  out.p_FinG  = p_CainG + out.lambda * out.u_G_raw;
  out.p_FinIc = R_GtoIc * (out.p_FinG - p_IcinG);
  out.p_FinCc = R_ItoC * out.p_FinIc + p_IinC;
  if (out.p_FinCc.z() < 0.1) return false;

  out.uv_n_pred << out.p_FinCc.x() / out.p_FinCc.z(),
                   out.p_FinCc.y() / out.p_FinCc.z();
  out.uv_pred = cam->distort_d(out.uv_n_pred);
  return true;
}

// ---- Analytic Jacobian ---------------------------------------------------
//
// Returns 2 x 12 matrix dh/d[δθ_a, δp_a, δθ_c, δp_c].
//
//   ∂h / ∂uv_pred_n = H_distort        (2x2, from cam->compute_distort_jacobian)
//   ∂uv_pred_n / ∂p_FinCc = H_proj     (2x3)
//   ∂p_FinCc / ∂x = R_ItoC * ∂p_FinIc/∂x  (where x is each clone variable)
//
// For each variable we work out ∂p_FinIc/∂x. p_FinIc = R_GtoIc * (p_FinG - p_IcinG).
//
//   ∂p_FinIc / ∂δθ_a = R_GtoIc * ∂p_FinG/∂δθ_a
//   ∂p_FinIc / ∂δp_a = R_GtoIc * ∂p_FinG/∂δp_a
//   ∂p_FinIc / ∂δθ_c = +skew(p_FinIc)         (∂(R*v)/∂δθ for v fixed, OV convention)
//   ∂p_FinIc / ∂δp_c = -R_GtoIc                (since p_IcinG → p_IcinG + δp_c)
//
// p_FinG = p_CainG + λ * u_G_raw.
//   ∂p_FinG / ∂δθ_a = ∂p_CainG/∂δθ_a + (∂λ/∂δθ_a) * u_G_raw + λ * ∂u_G_raw/∂δθ_a
//   ∂p_FinG / ∂δp_a = ∂p_CainG/∂δp_a + (∂λ/∂δp_a) * u_G_raw
//
// Building blocks (OV JPL convention):
//   ∂u_G_raw / ∂δθ_a = -R_GtoIa^T * skew(R_ItoC^T * u_C)
//   ∂p_CainG / ∂δθ_a = +R_GtoIa^T * skew(R_ItoC^T * p_IinC)
//   ∂p_CainG / ∂δp_a = I_3
//   ∂λ / ∂δθ_a = ( -∂p_CainG.z()/∂δθ_a  -  λ * ∂u_G_raw.z()/∂δθ_a ) / u_G_raw.z()
//   ∂λ / ∂δp_a = -e3^T / u_G_raw.z()
//
Eigen::Matrix<double, 2, 12> analytic_jacobian(
    const Eigen::Matrix3d &R_GtoIa, const Eigen::Vector3d &p_IinC,
    const Eigen::Matrix3d &R_GtoIc, const Eigen::Matrix3d &R_ItoC,
    const ForwardOut &fwd,
    const std::shared_ptr<ov_core::CamBase> &cam) {
  using Eigen::Vector3d;
  using Eigen::Matrix3d;
  using Eigen::Matrix;

  // 1) ∂h / ∂uv_pred_n  -- 2x2
  Eigen::MatrixXd Hd_zn, Hd_zeta; // 2x2 and 2xN (distortion params, unused)
  cam->compute_distort_jacobian(fwd.uv_n_pred, Hd_zn, Hd_zeta);
  Matrix<double, 2, 2> H_dist = Hd_zn.block<2, 2>(0, 0);

  // 2) ∂uv_pred_n / ∂p_FinCc -- 2x3
  double zc = fwd.p_FinCc.z();
  double zc2 = zc * zc;
  Matrix<double, 2, 3> H_proj;
  H_proj << 1.0 / zc, 0.0, -fwd.p_FinCc.x() / zc2,
            0.0, 1.0 / zc, -fwd.p_FinCc.y() / zc2;

  Matrix<double, 2, 3> H_uv_pFinCc = H_dist * H_proj;

  // 3) Building blocks for anchor variables
  Matrix3d S_RIC_uC   = ov_core::skew_x(R_ItoC.transpose() * fwd.u_C);
  Matrix3d S_RIC_pIC  = ov_core::skew_x(R_ItoC.transpose() * p_IinC);
  Matrix3d dpCa_dtha  =  R_GtoIa.transpose() * S_RIC_pIC;        // ∂p_CainG / ∂δθ_a
  Matrix3d duG_dtha   = -R_GtoIa.transpose() * S_RIC_uC;         // ∂u_G_raw / ∂δθ_a
  Eigen::RowVector3d dlam_dtha =
      ( -dpCa_dtha.row(2) - fwd.lambda * duG_dtha.row(2) ) / fwd.u_G_raw.z();
  Eigen::RowVector3d dlam_dpa  =
      Eigen::RowVector3d(0.0, 0.0, -1.0) / fwd.u_G_raw.z();

  // ∂p_FinG / ∂δθ_a = ∂p_CainG/∂δθ_a + u_G * dlam_dtha + λ * ∂u_G/∂δθ_a
  Matrix3d dpFG_dtha =
      dpCa_dtha + fwd.u_G_raw * dlam_dtha + fwd.lambda * duG_dtha;
  // ∂p_FinG / ∂δp_a = I + u_G * dlam_dpa
  Matrix3d dpFG_dpa = Matrix3d::Identity() + fwd.u_G_raw * dlam_dpa;

  // 4) ∂p_FinIc / ∂(anchor vars)  =  R_GtoIc * ∂p_FinG/∂(anchor vars)
  Matrix3d dpFIc_dtha = R_GtoIc * dpFG_dtha;
  Matrix3d dpFIc_dpa  = R_GtoIc * dpFG_dpa;

  // 5) ∂p_FinIc / ∂(current vars)
  Matrix3d dpFIc_dthc =  ov_core::skew_x(fwd.p_FinIc); // OV convention
  Matrix3d dpFIc_dpc  = -R_GtoIc;

  // 6) Assemble H (2x12)
  Matrix<double, 2, 12> H;
  H.block<2, 3>(0, 0) = H_uv_pFinCc * R_ItoC * dpFIc_dtha;   // δθ_a
  H.block<2, 3>(0, 3) = H_uv_pFinCc * R_ItoC * dpFIc_dpa;    // δp_a
  H.block<2, 3>(0, 6) = H_uv_pFinCc * R_ItoC * dpFIc_dthc;   // δθ_c
  H.block<2, 3>(0, 9) = H_uv_pFinCc * R_ItoC * dpFIc_dpc;    // δp_c

  // Convention: residual r = z - h(x).  The H above is ∂h/∂x; EKFUpdate
  // expects H s.t. r ≈ H * δx, i.e. ∂(h(x_true) - h(x_est))/∂δx.  With
  // x_true = x_est ⊕ δx and our +δp convention, h grows with +δx, so for a
  // SMALL perturbation that DECREASES h relative to the current estimate
  // the residual increases.  EKFUpdate uses (z - h), so the sign here is
  // already correct.  (Mirror v0 behaviour.)
  return H;
}

// ---- Finite-difference Jacobian -----------------------------------------
//
// Builds a 2x12 H by perturbing each of the 12 state variables in turn
// (3 anchor θ axes, 3 anchor p axes, 3 current θ axes, 3 current p axes)
// and re-evaluating the forward model.
//
// Rotation perturbation uses the OV JPL convention:
//   R(δθ) = (I - skew(δθ)) * R         (small-angle approx; matches the
//                                       analytic derivation above)
bool fd_jacobian(const Eigen::Matrix3d &R_GtoIa, const Eigen::Vector3d &p_IainG,
                 const Eigen::Matrix3d &R_GtoIc, const Eigen::Vector3d &p_IcinG,
                 const Eigen::Matrix3d &R_ItoC,  const Eigen::Vector3d &p_IinC,
                 const Eigen::Vector2d &uv_anchor_undistorted,
                 double z_ground, double min_lambda, double max_lambda,
                 const std::shared_ptr<ov_core::CamBase> &cam,
                 const ForwardOut &fwd0,
                 double h_rot, double h_pos,
                 Eigen::Matrix<double, 2, 12> &H_fd) {
  H_fd.setZero();

  auto eval = [&](const Eigen::Matrix3d &Ra, const Eigen::Vector3d &pa,
                  const Eigen::Matrix3d &Rc, const Eigen::Vector3d &pc,
                  Eigen::Vector2d &uv_out) -> bool {
    ForwardOut tmp;
    if (!forward(Ra, pa, Rc, pc, R_ItoC, p_IinC, uv_anchor_undistorted,
                 z_ground, min_lambda, max_lambda, cam, tmp))
      return false;
    uv_out = tmp.uv_pred;
    return true;
  };

  // Reference (unperturbed) prediction
  Eigen::Vector2d uv0 = fwd0.uv_pred;

  // Block 0: δθ_a  (rotation perturb on R_GtoIa)
  for (int k = 0; k < 3; k++) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero(); e(k) = h_rot;
    Eigen::Matrix3d Ra_p = (Eigen::Matrix3d::Identity() - ov_core::skew_x(e)) * R_GtoIa;
    Eigen::Matrix3d Ra_m = (Eigen::Matrix3d::Identity() + ov_core::skew_x(e)) * R_GtoIa;
    Eigen::Vector2d uv_p, uv_m;
    if (!eval(Ra_p, p_IainG, R_GtoIc, p_IcinG, uv_p)) return false;
    if (!eval(Ra_m, p_IainG, R_GtoIc, p_IcinG, uv_m)) return false;
    H_fd.col(k) = (uv_p - uv_m) / (2.0 * h_rot);
  }
  // Block 1: δp_a
  for (int k = 0; k < 3; k++) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero(); e(k) = h_pos;
    Eigen::Vector2d uv_p, uv_m;
    if (!eval(R_GtoIa, p_IainG + e, R_GtoIc, p_IcinG, uv_p)) return false;
    if (!eval(R_GtoIa, p_IainG - e, R_GtoIc, p_IcinG, uv_m)) return false;
    H_fd.col(3 + k) = (uv_p - uv_m) / (2.0 * h_pos);
  }
  // Block 2: δθ_c
  for (int k = 0; k < 3; k++) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero(); e(k) = h_rot;
    Eigen::Matrix3d Rc_p = (Eigen::Matrix3d::Identity() - ov_core::skew_x(e)) * R_GtoIc;
    Eigen::Matrix3d Rc_m = (Eigen::Matrix3d::Identity() + ov_core::skew_x(e)) * R_GtoIc;
    Eigen::Vector2d uv_p, uv_m;
    if (!eval(R_GtoIa, p_IainG, Rc_p, p_IcinG, uv_p)) return false;
    if (!eval(R_GtoIa, p_IainG, Rc_m, p_IcinG, uv_m)) return false;
    H_fd.col(6 + k) = (uv_p - uv_m) / (2.0 * h_rot);
  }
  // Block 3: δp_c
  for (int k = 0; k < 3; k++) {
    Eigen::Vector3d e = Eigen::Vector3d::Zero(); e(k) = h_pos;
    Eigen::Vector2d uv_p, uv_m;
    if (!eval(R_GtoIa, p_IainG, R_GtoIc, p_IcinG + e, uv_p)) return false;
    if (!eval(R_GtoIa, p_IainG, R_GtoIc, p_IcinG - e, uv_m)) return false;
    H_fd.col(9 + k) = (uv_p - uv_m) / (2.0 * h_pos);
  }

  (void)uv0; // (kept for potential one-sided FD)
  return true;
}

// Print a 2x12 matrix with column labels matching the v1 Hx_order.
void print_2x12(const char *label, const Eigen::Matrix<double, 2, 12> &M) {
  PRINT_INFO(CYAN "[V1-DUMP] %s\n" RESET, label);
  PRINT_INFO(CYAN "[V1-DUMP]           "
             "%10s %10s %10s | %10s %10s %10s | %10s %10s %10s | %10s %10s %10s\n" RESET,
             "tha_x", "tha_y", "tha_z",
             "pa_x", "pa_y", "pa_z",
             "thc_x", "thc_y", "thc_z",
             "pc_x", "pc_y", "pc_z");
  for (int r = 0; r < 2; r++) {
    PRINT_INFO(CYAN "[V1-DUMP]  row%d: "
               "%+10.3e %+10.3e %+10.3e | %+10.3e %+10.3e %+10.3e | "
               "%+10.3e %+10.3e %+10.3e | %+10.3e %+10.3e %+10.3e\n" RESET,
               r, M(r,0), M(r,1), M(r,2),
                  M(r,3), M(r,4), M(r,5),
                  M(r,6), M(r,7), M(r,8),
                  M(r,9), M(r,10), M(r,11));
  }
}

// b=0 (tha) and b=2 (thc) are rotation blocks; b=1 (pa) and b=3 (pc) are
// position blocks.  A block passes when BOTH (rel_err < per-block rel_tol)
// AND (max_abs_err / max(an, fn) < max_abs_rel_tol).
void compare_jacobian(const Eigen::Matrix<double, 2, 12> &Ha,
                      const Eigen::Matrix<double, 2, 12> &Hf,
                      double rel_tol_rot, double rel_tol_pos,
                      double max_abs_rel_tol,
                      UpdaterGroundPlaneFeatureV1::FDCheck &out) {
  out.valid = true;
  for (int b = 0; b < 4; b++) {
    Eigen::Matrix<double, 2, 3> A = Ha.block<2, 3>(0, b * 3);
    Eigen::Matrix<double, 2, 3> F = Hf.block<2, 3>(0, b * 3);
    double an = A.norm(), fn = F.norm();
    double maxabs = (A - F).cwiseAbs().maxCoeff();
    double denom = std::max({an, fn, 1e-9});
    double rel = (A - F).norm() / denom;
    double maxabs_rel = maxabs / denom;
    bool is_rotation = (b == 0 || b == 2);
    double rel_tol = is_rotation ? rel_tol_rot : rel_tol_pos;
    out.analytic_norm[b] = an;
    out.numeric_norm[b] = fn;
    out.max_abs_err[b] = maxabs;
    out.rel_err[b] = rel;
    out.passed[b] = (rel < rel_tol) && (maxabs_rel < max_abs_rel_tol);
  }
  out.passed_all = out.passed[0] && out.passed[1] && out.passed[2] && out.passed[3];
}

} // namespace

// ---------------------------------------------------------------------------

bool UpdaterGroundPlaneFeatureV1::try_update(
    std::shared_ptr<ov_msckf::State> state,
    std::shared_ptr<ov_core::FeatureDatabase> features,
    double t_state, double z_ground) {

  using namespace ov_type;

  stats_.n_called++;
  if (stats_.first_t < 0) stats_.first_t = t_state;
  stats_.last_t = t_state;
  last_ = LastUpdate();
  last_.t = t_state;
  last_used_ids_.clear();

  if (state->_clones_IMU.size() < 2) {
    stats_.n_skipped_no_clones++;
    last_.decision = "SKIP_NO_CLONES";
    return false;
  }
  double cos_tilt = state->_imu->Rot()(2, 2);
  if (std::fabs(cos_tilt) < min_cos_tilt_) {
    stats_.n_skipped_tilt++; last_.decision = "SKIP_TILT"; return false;
  }

  double t_anchor = state->_clones_IMU.begin()->first;
  if (std::fabs(t_anchor - t_state) < 1e-6) {
    stats_.n_skipped_no_clones++;
    last_.decision = "SKIP_ANCHOR=CURRENT";
    return false;
  }

  std::shared_ptr<PoseJPL> anchor_clone = state->_clones_IMU.at(t_anchor);
  std::shared_ptr<PoseJPL> curr_clone   = state->_clones_IMU.at(t_state);
  Eigen::Matrix3d R_GtoIa = anchor_clone->Rot();
  Eigen::Vector3d p_IainG = anchor_clone->pos();
  Eigen::Matrix3d R_GtoIc = curr_clone->Rot();
  Eigen::Vector3d p_IcinG = curr_clone->pos();

  auto feats_curr = features->features_containing(t_state, false, true);
  if (feats_curr.empty()) {
    stats_.n_skipped_no_features++;
    last_.decision = "SKIP_NO_FEATURES";
    return false;
  }

  std::vector<std::shared_ptr<Type>> Hx_order = {
      anchor_clone->q(), anchor_clone->p(),
      curr_clone->q(),   curr_clone->p()
  };

  std::vector<Eigen::Matrix<double, 2, 12>> H_blocks;
  std::vector<Eigen::Vector2d> res_blocks;
  std::vector<size_t> used_ids;

  int n_candidates = 0, n_pass_gate = 0;
  double sum_abs_res = 0.0, max_abs_res = 0.0;
  FDCheck last_fd;

  for (const auto &feat : feats_curr) {
    Eigen::Vector2d uv_anchor, uv_curr;
    size_t cam_a = 0, cam_c = 0;
    if (!find_uv_at(feat, t_anchor, uv_anchor, cam_a)) continue;
    if (!find_uv_at(feat, t_state, uv_curr, cam_c))   continue;
    if (cam_a != cam_c) continue;
    size_t cam_id = cam_c;
    n_candidates++;

    auto cam_it = state->_cam_intrinsics_cameras.find(cam_id);
    if (cam_it == state->_cam_intrinsics_cameras.end()) continue;
    auto cam = cam_it->second;
    auto calib_it = state->_calib_IMUtoCAM.find(cam_id);
    if (calib_it == state->_calib_IMUtoCAM.end()) continue;
    Eigen::Matrix3d R_ItoC = calib_it->second->Rot();
    Eigen::Vector3d p_IinC = calib_it->second->pos();

    // -- center-region gate (anchor pixel) --
    int W = cam->w(), H = cam->h();
    double bx = 0.5 * (1.0 - center_frac_) * W;
    double by = 0.5 * (1.0 - center_frac_) * H;
    if (uv_anchor(0) < bx || uv_anchor(0) > W - bx ||
        uv_anchor(1) < by || uv_anchor(1) > H - by) continue;

    Eigen::Vector2d uv_a_n = cam->undistort_d(uv_anchor);

    ForwardOut fwd;
    if (!forward(R_GtoIa, p_IainG, R_GtoIc, p_IcinG, R_ItoC, p_IinC,
                 uv_a_n, z_ground, min_lambda_, max_lambda_, cam, fwd)) {
      continue;
    }

    Eigen::Vector2d res = uv_curr - fwd.uv_pred;
    double r_norm = res.norm();
    if (r_norm > max_residual_px_) continue;
    n_pass_gate++;

    // analytic Jacobian
    Eigen::Matrix<double, 2, 12> Ha =
        analytic_jacobian(R_GtoIa, p_IinC, R_GtoIc, R_ItoC, fwd, cam);

    // FD check (always in DRY_RUN; first-feature-of-each-update in UPDATE)
    bool do_fd = (mode_ == Mode::DRY_RUN) || (H_blocks.empty());
    if (do_fd) {
      Eigen::Matrix<double, 2, 12> Hf;
      bool fd_ok = fd_jacobian(R_GtoIa, p_IainG, R_GtoIc, p_IcinG, R_ItoC,
                               p_IinC, uv_a_n, z_ground, min_lambda_,
                               max_lambda_, cam, fwd,
                               fd_step_rot_, fd_step_pos_, Hf);
      if (fd_ok) {
        FDCheck fd; compare_jacobian(Ha, Hf,
                                     fd_rel_tol_rot_, fd_rel_tol_pos_,
                                     fd_max_abs_rel_tol_, fd);
        last_fd = fd;
        stats_.n_fd_checks++;
        if (fd.passed_all) stats_.n_fd_pass++;
        else               stats_.n_fd_fail++;
        // per-block stats
        for (int b = 0; b < 4; b++) {
          if (fd.passed[b]) stats_.n_pass_block[b]++;
          else              stats_.n_fail_block[b]++;
          stats_.sum_rel_err_block[b] += fd.rel_err[b];
          if (fd.rel_err[b] > stats_.max_rel_err_block[b])
            stats_.max_rel_err_block[b] = fd.rel_err[b];
          if (fd.max_abs_err[b] > stats_.max_abs_err_block[b])
            stats_.max_abs_err_block[b] = fd.max_abs_err[b];
          stats_.rel_err_history[b].push_back(fd.rel_err[b]);
        }

        PRINT_INFO(
            CYAN "[GPLANE-V1-FD] t=%.3f feat=%zu "
            "blk_theta_a:[an=%.4g num=%.4g maxabs=%.4g rel=%.3g %s] "
            "blk_p_a:[an=%.4g num=%.4g maxabs=%.4g rel=%.3g %s] "
            "blk_theta_c:[an=%.4g num=%.4g maxabs=%.4g rel=%.3g %s] "
            "blk_p_c:[an=%.4g num=%.4g maxabs=%.4g rel=%.3g %s]\n" RESET,
            t_state, feat->featid,
            fd.analytic_norm[0], fd.numeric_norm[0], fd.max_abs_err[0], fd.rel_err[0],
            fd.passed[0] ? "PASS" : "FAIL",
            fd.analytic_norm[1], fd.numeric_norm[1], fd.max_abs_err[1], fd.rel_err[1],
            fd.passed[1] ? "PASS" : "FAIL",
            fd.analytic_norm[2], fd.numeric_norm[2], fd.max_abs_err[2], fd.rel_err[2],
            fd.passed[2] ? "PASS" : "FAIL",
            fd.analytic_norm[3], fd.numeric_norm[3], fd.max_abs_err[3], fd.rel_err[3],
            fd.passed[3] ? "PASS" : "FAIL");

        // Verbose dump for the first N features (debug aid).
        if (dump_remaining_ > 0) {
          PRINT_INFO(CYAN "[V1-DUMP] ===== feature dump (remaining=%d) =====\n" RESET,
                     dump_remaining_);
          PRINT_INFO(CYAN "[V1-DUMP] t=%.6f feat_id=%zu cam_id=%zu\n" RESET,
                     t_state, feat->featid, cam_id);
          PRINT_INFO(CYAN "[V1-DUMP] anchor:  t=%.6f  R_GtoIa row0=(%.4f %.4f %.4f) row1=(%.4f %.4f %.4f) row2=(%.4f %.4f %.4f)\n" RESET,
                     t_anchor,
                     R_GtoIa(0,0), R_GtoIa(0,1), R_GtoIa(0,2),
                     R_GtoIa(1,0), R_GtoIa(1,1), R_GtoIa(1,2),
                     R_GtoIa(2,0), R_GtoIa(2,1), R_GtoIa(2,2));
          PRINT_INFO(CYAN "[V1-DUMP] anchor:  p_IainG=(%.6f, %.6f, %.6f)\n" RESET,
                     p_IainG(0), p_IainG(1), p_IainG(2));
          PRINT_INFO(CYAN "[V1-DUMP] current: t=%.6f  R_GtoIc row0=(%.4f %.4f %.4f) row1=(%.4f %.4f %.4f) row2=(%.4f %.4f %.4f)\n" RESET,
                     t_state,
                     R_GtoIc(0,0), R_GtoIc(0,1), R_GtoIc(0,2),
                     R_GtoIc(1,0), R_GtoIc(1,1), R_GtoIc(1,2),
                     R_GtoIc(2,0), R_GtoIc(2,1), R_GtoIc(2,2));
          PRINT_INFO(CYAN "[V1-DUMP] current: p_IcinG=(%.6f, %.6f, %.6f)\n" RESET,
                     p_IcinG(0), p_IcinG(1), p_IcinG(2));
          PRINT_INFO(CYAN "[V1-DUMP] calib: R_ItoC row0=(%.4f %.4f %.4f) row1=(%.4f %.4f %.4f) row2=(%.4f %.4f %.4f)  p_IinC=(%.4f, %.4f, %.4f)\n" RESET,
                     R_ItoC(0,0), R_ItoC(0,1), R_ItoC(0,2),
                     R_ItoC(1,0), R_ItoC(1,1), R_ItoC(1,2),
                     R_ItoC(2,0), R_ItoC(2,1), R_ItoC(2,2),
                     p_IinC(0), p_IinC(1), p_IinC(2));
          PRINT_INFO(CYAN "[V1-DUMP] z_ground=%.6f\n" RESET, z_ground);
          PRINT_INFO(CYAN "[V1-DUMP] anchor_pixel=(%.4f, %.4f)  current_pixel=(%.4f, %.4f)\n" RESET,
                     uv_anchor(0), uv_anchor(1), uv_curr(0), uv_curr(1));
          PRINT_INFO(CYAN "[V1-DUMP] u_C=(%.6f, %.6f, %.6f)  (undistorted anchor ray in cam frame)\n" RESET,
                     fwd.u_C(0), fwd.u_C(1), fwd.u_C(2));
          PRINT_INFO(CYAN "[V1-DUMP] u_G_raw=(%.6f, %.6f, %.6f)  (NOT normalised)\n" RESET,
                     fwd.u_G_raw(0), fwd.u_G_raw(1), fwd.u_G_raw(2));
          PRINT_INFO(CYAN "[V1-DUMP] lambda=%.6f  p_FinG=(%.6f, %.6f, %.6f)\n" RESET,
                     fwd.lambda, fwd.p_FinG(0), fwd.p_FinG(1), fwd.p_FinG(2));
          PRINT_INFO(CYAN "[V1-DUMP] p_FinIc=(%.6f, %.6f, %.6f)  p_FinCc=(%.6f, %.6f, %.6f)\n" RESET,
                     fwd.p_FinIc(0), fwd.p_FinIc(1), fwd.p_FinIc(2),
                     fwd.p_FinCc(0), fwd.p_FinCc(1), fwd.p_FinCc(2));
          PRINT_INFO(CYAN "[V1-DUMP] uv_n_pred=(%.6f, %.6f)  uv_pred=(%.4f, %.4f)  residual=(%.4f, %.4f)\n" RESET,
                     fwd.uv_n_pred(0), fwd.uv_n_pred(1),
                     fwd.uv_pred(0), fwd.uv_pred(1),
                     res(0), res(1));

          // J_proj = H_dist * H_proj (2x3), the camera-frame -> pixel Jac
          Eigen::MatrixXd Hd_zn, Hd_zeta;
          cam->compute_distort_jacobian(fwd.uv_n_pred, Hd_zn, Hd_zeta);
          Eigen::Matrix<double, 2, 2> H_dist = Hd_zn.block<2, 2>(0, 0);
          double zc = fwd.p_FinCc.z(), zc2 = zc * zc;
          Eigen::Matrix<double, 2, 3> H_proj;
          H_proj << 1.0 / zc, 0.0, -fwd.p_FinCc.x() / zc2,
                    0.0, 1.0 / zc, -fwd.p_FinCc.y() / zc2;
          Eigen::Matrix<double, 2, 3> J_proj = H_dist * H_proj;
          PRINT_INFO(CYAN "[V1-DUMP] H_dist:\n  row0=(%.6f, %.6f)  row1=(%.6f, %.6f)\n" RESET,
                     H_dist(0,0), H_dist(0,1), H_dist(1,0), H_dist(1,1));
          PRINT_INFO(CYAN "[V1-DUMP] H_proj (2x3, d(uv_n)/d(p_FinCc)):\n  row0=(%.6f, %.6f, %.6f)  row1=(%.6f, %.6f, %.6f)\n" RESET,
                     H_proj(0,0), H_proj(0,1), H_proj(0,2),
                     H_proj(1,0), H_proj(1,1), H_proj(1,2));
          PRINT_INFO(CYAN "[V1-DUMP] J_proj = H_dist * H_proj (2x3):\n  row0=(%.4f, %.4f, %.4f)  row1=(%.4f, %.4f, %.4f)\n" RESET,
                     J_proj(0,0), J_proj(0,1), J_proj(0,2),
                     J_proj(1,0), J_proj(1,1), J_proj(1,2));

          print_2x12("H_analytic (2x12):", Ha);
          char hdr[160];
          std::snprintf(hdr, sizeof(hdr),
                        "H_numeric (h_rot=%.0e, h_pos=%.0e):",
                        fd_step_rot_, fd_step_pos_);
          print_2x12(hdr, Hf);
          Eigen::Matrix<double, 2, 12> Hdiff = Ha - Hf;
          print_2x12("H_diff = H_analytic - H_numeric:", Hdiff);

          // Rotation step-size sweep
          PRINT_INFO(CYAN "[V1-DUMP] rotation step-size sweep (h_pos=%.0e fixed):\n" RESET, fd_step_pos_);
          for (double h : {1e-4, 1e-5, 1e-6}) {
            Eigen::Matrix<double, 2, 12> Hf_sw;
            if (fd_jacobian(R_GtoIa, p_IainG, R_GtoIc, p_IcinG, R_ItoC,
                            p_IinC, uv_a_n, z_ground, min_lambda_,
                            max_lambda_, cam, fwd,
                            h, fd_step_pos_, Hf_sw)) {
              FDCheck fd_sw;
              compare_jacobian(Ha, Hf_sw,
                               fd_rel_tol_rot_, fd_rel_tol_pos_,
                               fd_max_abs_rel_tol_, fd_sw);
              PRINT_INFO(CYAN "[V1-DUMP]   h_rot=%.0e  blk_tha:[an=%.4g num=%.4g maxabs=%.3g rel=%.3g %s]  "
                         "blk_thc:[an=%.4g num=%.4g maxabs=%.3g rel=%.3g %s]\n" RESET,
                         h,
                         fd_sw.analytic_norm[0], fd_sw.numeric_norm[0],
                         fd_sw.max_abs_err[0], fd_sw.rel_err[0],
                         fd_sw.passed[0] ? "PASS" : "FAIL",
                         fd_sw.analytic_norm[2], fd_sw.numeric_norm[2],
                         fd_sw.max_abs_err[2], fd_sw.rel_err[2],
                         fd_sw.passed[2] ? "PASS" : "FAIL");
            } else {
              PRINT_INFO(CYAN "[V1-DUMP]   h_rot=%.0e  FD degenerate\n" RESET, h);
            }
          }
          // Position step-size sweep
          PRINT_INFO(CYAN "[V1-DUMP] position step-size sweep (h_rot=%.0e fixed):\n" RESET, fd_step_rot_);
          for (double h : {1e-3, 1e-4, 1e-5, 1e-6}) {
            Eigen::Matrix<double, 2, 12> Hf_sw;
            if (fd_jacobian(R_GtoIa, p_IainG, R_GtoIc, p_IcinG, R_ItoC,
                            p_IinC, uv_a_n, z_ground, min_lambda_,
                            max_lambda_, cam, fwd,
                            fd_step_rot_, h, Hf_sw)) {
              FDCheck fd_sw;
              compare_jacobian(Ha, Hf_sw,
                               fd_rel_tol_rot_, fd_rel_tol_pos_,
                               fd_max_abs_rel_tol_, fd_sw);
              PRINT_INFO(CYAN "[V1-DUMP]   h_pos=%.0e  blk_pa:[an=%.4g num=%.4g maxabs=%.3g rel=%.3g %s]  "
                         "blk_pc:[an=%.4g num=%.4g maxabs=%.3g rel=%.3g %s]\n" RESET,
                         h,
                         fd_sw.analytic_norm[1], fd_sw.numeric_norm[1],
                         fd_sw.max_abs_err[1], fd_sw.rel_err[1],
                         fd_sw.passed[1] ? "PASS" : "FAIL",
                         fd_sw.analytic_norm[3], fd_sw.numeric_norm[3],
                         fd_sw.max_abs_err[3], fd_sw.rel_err[3],
                         fd_sw.passed[3] ? "PASS" : "FAIL");
            } else {
              PRINT_INFO(CYAN "[V1-DUMP]   h_pos=%.0e  FD degenerate\n" RESET, h);
            }
          }
          PRINT_INFO(CYAN "[V1-DUMP] ===== end of feature dump =====\n" RESET);
          dump_remaining_--;
        }

        // In UPDATE mode: refuse to use a feature whose FD check failed.
        if (mode_ == Mode::UPDATE && !fd.passed_all) continue;
      } else {
        // FD became degenerate after perturbation -> skip this feature
        continue;
      }
    }

    H_blocks.push_back(Ha);
    res_blocks.push_back(res);
    used_ids.push_back(feat->featid);
    sum_abs_res += r_norm;
    if (r_norm > max_abs_res) max_abs_res = r_norm;
    if ((int)H_blocks.size() >= max_features_) break;
  }

  last_.n_candidates = n_candidates;
  last_.n_passed_gate = n_pass_gate;
  last_.n_used = (int)H_blocks.size();
  last_.fd = last_fd;

  if (H_blocks.empty()) {
    stats_.n_skipped_no_features++;
    last_.decision = "SKIP_NO_PASS";
    PRINT_DEBUG(YELLOW "[GPLANE-V1] t=%.3f cand=%d gate=%d used=0 (mode=%s) -- skip\n" RESET,
                t_state, n_candidates, n_pass_gate,
                mode_ == Mode::DRY_RUN ? "DRY_RUN" : "UPDATE");
    return false;
  }

  // DRY_RUN: do NOT touch state / P; just log.
  if (mode_ == Mode::DRY_RUN) {
    last_.decision = "DRY_RUN_OK";
    last_.mean_residual_px = sum_abs_res / (double)H_blocks.size();
    last_.max_residual_px = max_abs_res;
    PRINT_INFO(CYAN "[GPLANE-V1-DRY] t=%.3f cand=%d gate=%d feats=%d "
               "|res|_mu=%.2fpx max=%.2fpx fd_pass=%zu fd_fail=%zu\n" RESET,
               t_state, n_candidates, n_pass_gate, (int)H_blocks.size(),
               last_.mean_residual_px, last_.max_residual_px,
               stats_.n_fd_pass, stats_.n_fd_fail);
    return true;
  }

  // UPDATE: stack and EKFUpdate
  int m = (int)H_blocks.size() * 2;
  Eigen::MatrixXd Hbig = Eigen::MatrixXd::Zero(m, 12);
  Eigen::VectorXd res  = Eigen::VectorXd::Zero(m);
  for (size_t i = 0; i < H_blocks.size(); i++) {
    Hbig.block(2 * i, 0, 2, 12) = H_blocks[i];
    res.segment(2 * i, 2)       = res_blocks[i];
  }
  Eigen::MatrixXd R =
      (sigma_pixel_ * sigma_pixel_) * Eigen::MatrixXd::Identity(m, m);

  Eigen::Vector3d p_pre = state->_imu->pos();
  ov_msckf::StateHelper::EKFUpdate(state, Hx_order, Hbig, res, R);
  Eigen::Vector3d dp = state->_imu->pos() - p_pre;

  for (auto id : used_ids) last_used_ids_.insert(id);

  last_.mean_residual_px = sum_abs_res / (double)H_blocks.size();
  last_.max_residual_px = max_abs_res;
  last_.dxy_norm = std::sqrt(dp.x() * dp.x() + dp.y() * dp.y());
  last_.dz_after = dp.z();
  last_.decision = "ACCEPT";

  stats_.n_accepted_updates++;
  stats_.n_features_used_total += H_blocks.size();
  stats_.sum_residual_px += sum_abs_res;
  stats_.sum_dxy_norm += last_.dxy_norm;
  stats_.sum_dz_after += std::fabs(dp.z());
  if (max_abs_res > stats_.max_residual_px) stats_.max_residual_px = max_abs_res;
  if (last_.dxy_norm > stats_.max_dxy_norm) stats_.max_dxy_norm = last_.dxy_norm;
  if (std::fabs(dp.z()) > stats_.max_dz_after) stats_.max_dz_after = std::fabs(dp.z());

  PRINT_INFO(CYAN "[GPLANE-V1] t=%.3f cand=%d gate=%d used=%d "
             "|res|_mu=%.2fpx max=%.2fpx |dxy|=%.3fm dz=%+.3fm\n" RESET,
             t_state, n_candidates, n_pass_gate, (int)H_blocks.size(),
             last_.mean_residual_px, last_.max_residual_px,
             last_.dxy_norm, dp.z());
  return true;
}

void UpdaterGroundPlaneFeatureV1::print_summary() const {
  size_t na = std::max((size_t)1, stats_.n_accepted_updates);
  PRINT_INFO(GREEN "[GPLANE-V1-FINAL] ===== Stage B v1 Summary =====\n" RESET);
  PRINT_INFO(GREEN "[GPLANE-V1-FINAL] mode=%s  h_rot=%.0e  h_pos=%.0e  "
             "rel_tol_rot=%.0e  rel_tol_pos=%.0e  max_abs_rel_tol=%.0e\n" RESET,
             mode_ == Mode::DRY_RUN ? "DRY_RUN" : "UPDATE",
             fd_step_rot_, fd_step_pos_,
             fd_rel_tol_rot_, fd_rel_tol_pos_, fd_max_abs_rel_tol_);
  PRINT_INFO(GREEN "[GPLANE-V1-FINAL] called=%zu acc=%zu skip_tilt=%zu "
             "skip_noclone=%zu skip_nofeat=%zu\n" RESET,
             stats_.n_called, stats_.n_accepted_updates,
             stats_.n_skipped_tilt, stats_.n_skipped_no_clones,
             stats_.n_skipped_no_features);
  PRINT_INFO(GREEN "[GPLANE-V1-FINAL] fd_checks=%zu  all4_pass=%zu  any_fail=%zu  "
             "feats/upd=%.2f  |res|_mu=%.2fpx  |res|_max=%.2fpx  "
             "|dxy|_mu=%.3fm  |dxy|_max=%.3fm  |dz|_mu=%.3fm  |dz|_max=%.3fm\n" RESET,
             stats_.n_fd_checks, stats_.n_fd_pass, stats_.n_fd_fail,
             (double)stats_.n_features_used_total / na,
             stats_.sum_residual_px /
                 std::max((size_t)1, stats_.n_features_used_total),
             stats_.max_residual_px,
             stats_.sum_dxy_norm / na, stats_.max_dxy_norm,
             stats_.sum_dz_after / na, stats_.max_dz_after);

  const char *names[4] = {"tha", "pa ", "thc", "pc "};
  for (int b = 0; b < 4; b++) {
    size_t n = stats_.n_pass_block[b] + stats_.n_fail_block[b];
    if (n == 0) continue;
    double mean = stats_.sum_rel_err_block[b] / (double)n;
    // p95 (modify a local copy so the function stays const for clarity)
    std::vector<double> v = stats_.rel_err_history[b];
    double p95 = 0.0;
    if (!v.empty()) {
      std::sort(v.begin(), v.end());
      size_t idx = (size_t)((v.size() - 1) * 0.95);
      p95 = v[idx];
    }
    double pass_rate = 100.0 * (double)stats_.n_pass_block[b] / (double)n;
    PRINT_INFO(GREEN "[GPLANE-V1-FINAL]   blk_%s: pass=%zu/%zu (%.1f%%)  "
               "rel_err mean=%.3e  p95=%.3e  max=%.3e  maxabs_seen=%.3e\n" RESET,
               names[b], stats_.n_pass_block[b], n, pass_rate,
               mean, p95, stats_.max_rel_err_block[b],
               stats_.max_abs_err_block[b]);
  }
  PRINT_INFO(GREEN "[GPLANE-V1-FINAL] =================================\n" RESET);
}
