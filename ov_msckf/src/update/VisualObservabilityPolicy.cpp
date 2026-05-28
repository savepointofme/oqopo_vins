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

#include "VisualObservabilityPolicy.h"

#include "state/State.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/print.h"

using namespace ov_type;
using namespace ov_msckf;

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

VisualObservabilityPolicy::Mode VisualObservabilityPolicy::mode_from_string(const std::string &s) {
  if (s == "global_yaw_oc_fej_prechi2")
    return Mode::GLOBAL_YAW_OC_FEJ_PRECHI2;
  if (s == "visual_4d_oc_fej_prechi2")
    return Mode::VISUAL_4D_OC_FEJ_PRECHI2;
  return Mode::DISABLED;
}

bool VisualObservabilityPolicy::is_prechi2_mode_string(const std::string &s) {
  return s == "global_yaw_oc_fej_prechi2" || s == "visual_4d_oc_fej_prechi2";
}

std::vector<int> VisualObservabilityPolicy::build_H_id(
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order) {
  std::vector<int> H_id;
  H_id.reserve(H_order.size());
  int cur = 0;
  for (const auto &var : H_order) {
    H_id.push_back(cur);
    cur += var->size();
  }
  return H_id;
}

// ---------------------------------------------------------------------------
// Gauge construction helpers
// ---------------------------------------------------------------------------

// Fill 3 entries of `n` at `col` with the orientation yaw gauge: R_GtoI * e3.
// In JPL convention R_GtoI rotates from global to IMU body frame.
// The yaw axis in the body frame is R_GtoI * [0,0,1]^T.
static void fill_rot_yaw(Eigen::VectorXd &n, int col,
                          const Eigen::Matrix3d &R_GtoI) {
  if (col < 0 || col + 3 > n.rows())
    return;
  n.segment(col, 3) = R_GtoI * Eigen::Vector3d::UnitZ();
}

// Fill 3 entries with the position yaw gauge: e3 x p = [-p_y, p_x, 0].
static void fill_pos_yaw(Eigen::VectorXd &n, int col,
                          const Eigen::Vector3d &p) {
  if (col < 0 || col + 3 > n.rows())
    return;
  n.segment(col, 3) = Eigen::Vector3d::UnitZ().cross(p);
}

// ---------------------------------------------------------------------------
// build_yaw_gauge
// ---------------------------------------------------------------------------
// The 1-D global-yaw unobservable gauge for visual measurements.
// For each state block in H_order we fill the direction in gauge-space:
//
//   Clone/IMU orientation  →  R_GtoI_fej * e3   (3 entries)
//   Clone/IMU position     →  e3 × p_IinG_fej   (3 entries)
//   Clone/IMU velocity     →  e3 × v_IinG_fej   (3 entries)
//   GLOBAL_3D landmark     →  e3 × p_FinG_fej   (3 entries)
//   Everything else        →  0
//
// Reference: Huang 2011 "Observability-based Rules for Designing Consistent
// EKF SLAM Estimators".  The gauge here corresponds to N_q and N_p in that
// paper, assembled into a single flat vector matching H_order's column layout.
Eigen::VectorXd VisualObservabilityPolicy::build_yaw_gauge(
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
    const std::vector<int> &H_id, int H_cols,
    std::shared_ptr<State> state, bool use_fej) const {

  Eigen::VectorXd n = Eigen::VectorXd::Zero(H_cols);

  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    const int col = H_id[i];

    // --- IMU full block (size = 15: q(3) p(3) v(3) bg(3) ba(3)) ---
    if (var == state->_imu) {
      Eigen::Matrix3d R = use_fej ? state->_imu->Rot_fej() : state->_imu->Rot();
      Eigen::Vector3d p = use_fej ? state->_imu->pos_fej() : state->_imu->pos();
      Eigen::Vector3d v = use_fej ? state->_imu->vel_fej() : state->_imu->vel();
      fill_rot_yaw(n, col, R);
      fill_pos_yaw(n, col + 3, p);
      fill_pos_yaw(n, col + 6, v);
      // bg, ba: zero gauge (cols 9–14 already zero)
      continue;
    }
    if (var == state->_imu->q()) {
      Eigen::Matrix3d R = use_fej ? state->_imu->Rot_fej() : state->_imu->Rot();
      fill_rot_yaw(n, col, R);
      continue;
    }
    if (var == state->_imu->p()) {
      Eigen::Vector3d p = use_fej ? state->_imu->pos_fej() : state->_imu->pos();
      fill_pos_yaw(n, col, p);
      continue;
    }
    if (var == state->_imu->v()) {
      Eigen::Vector3d v = use_fej ? state->_imu->vel_fej() : state->_imu->vel();
      fill_pos_yaw(n, col, v);
      continue;
    }
    // bg / ba: zero gauge
    if (var == state->_imu->bg() || var == state->_imu->ba())
      continue;

    // --- Clone poses (PoseJPL, size=6: q(3) p(3)) ---
    bool matched_clone = false;
    for (const auto &kv : state->_clones_IMU) {
      const auto &pose = kv.second;
      if (var == pose) {
        Eigen::Matrix3d R = use_fej ? pose->Rot_fej() : pose->Rot();
        Eigen::Vector3d p = use_fej ? pose->pos_fej() : pose->pos();
        fill_rot_yaw(n, col, R);
        fill_pos_yaw(n, col + 3, p);
        matched_clone = true;
        break;
      }
      if (var == pose->q()) {
        Eigen::Matrix3d R = use_fej ? pose->Rot_fej() : pose->Rot();
        fill_rot_yaw(n, col, R);
        matched_clone = true;
        break;
      }
      if (var == pose->p()) {
        Eigen::Vector3d p = use_fej ? pose->pos_fej() : pose->pos();
        fill_pos_yaw(n, col, p);
        matched_clone = true;
        break;
      }
    }
    if (matched_clone)
      continue;

    // --- GLOBAL_3D SLAM landmarks ---
    auto lm = std::dynamic_pointer_cast<ov_type::Landmark>(var);
    if (lm && lm->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D &&
        lm->size() == 3) {
      Eigen::Vector3d p_f = lm->get_xyz(use_fej);
      fill_pos_yaw(n, col, p_f);
      continue;
    }

    // Camera extrinsics / intrinsics / time-offset: zero gauge.
    // (Formally there is a yaw component for extrinsics when online calib is
    //  active, but its derivation is non-trivial and we follow the same
    //  conservative choice as the existing StateHelper OC code.)
  }

  return n;
}

// ---------------------------------------------------------------------------
// build_4d_gauge
// ---------------------------------------------------------------------------
// Assembles a (H_cols x 4) matrix whose columns span the 4-D visual
// unobservable subspace:
//
//   col 0: global yaw      (same as build_yaw_gauge)
//   col 1: global x shift  (all position / landmark blocks get e1)
//   col 2: global y shift  (all position / landmark blocks get e2)
//   col 3: global z shift  (all position / landmark blocks get e3)
//
// For anchored landmarks the translation gauge is zero (since the residual
// p_FinCi = R_GtoCi*(p_FinG - p_CiinG) is invariant to a global shift that
// moves both the feature and all camera origins by the same delta).
Eigen::MatrixXd VisualObservabilityPolicy::build_4d_gauge(
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
    const std::vector<int> &H_id, int H_cols,
    std::shared_ptr<State> state, bool use_fej) const {

  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(H_cols, 4);

  // Column 0: yaw
  N.col(0) = build_yaw_gauge(H_order, H_id, H_cols, state, use_fej);

  // Columns 1-3: global translation
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    const int col = H_id[i];

    auto fill_trans = [&](int base) {
      // base: absolute row index of the start of the 3-D position sub-block
      if (base < 0 || base + 3 > H_cols)
        return;
      N(base, 1) = 1.0;
      N(base + 1, 2) = 1.0;
      N(base + 2, 3) = 1.0;
    };

    if (var == state->_imu) {
      fill_trans(col + 3); // p starts at offset 3 inside IMU block
      continue;
    }
    if (var == state->_imu->p()) {
      fill_trans(col);
      continue;
    }
    // q, v, bg, ba: no position contribution for translation gauge
    if (var == state->_imu->q() || var == state->_imu->v() ||
        var == state->_imu->bg() || var == state->_imu->ba())
      continue;

    bool matched_clone = false;
    for (const auto &kv : state->_clones_IMU) {
      const auto &pose = kv.second;
      if (var == pose) {
        fill_trans(col + 3); // p starts at offset 3 inside PoseJPL
        matched_clone = true;
        break;
      }
      if (var == pose->p()) {
        fill_trans(col);
        matched_clone = true;
        break;
      }
      if (var == pose->q()) {
        matched_clone = true; // rotation: no translation gauge
        break;
      }
    }
    if (matched_clone)
      continue;

    // GLOBAL_3D landmarks: all three translation directions
    auto lm = std::dynamic_pointer_cast<ov_type::Landmark>(var);
    if (lm && lm->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D &&
        lm->size() == 3) {
      fill_trans(col);
      continue;
    }
    // Anchored landmarks and calibration: zero gauge for translation columns
  }

  return N;
}

// ---------------------------------------------------------------------------
// apply
// ---------------------------------------------------------------------------
Eigen::MatrixXd VisualObservabilityPolicy::apply(
    const Eigen::MatrixXd &H,
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
    const std::vector<int> &H_id,
    std::shared_ptr<State> state,
    Diag *diag_out) const {

  if (mode_ == Mode::DISABLED || H.rows() == 0 || H.cols() == 0) {
    if (diag_out)
      *diag_out = Diag{};
    return H;
  }

  const bool use_fej = true; // always FEJ for these new modes

  // ---- Build unobservable gauge matrix N ----
  Eigen::MatrixXd N;
  if (mode_ == Mode::GLOBAL_YAW_OC_FEJ_PRECHI2) {
    N = build_yaw_gauge(H_order, H_id, H.cols(), state, use_fej);
  } else {
    N = build_4d_gauge(H_order, H_id, H.cols(), state, use_fej);
  }

  // ---- Diagnostics: H*N before projection ----
  double norm_H = H.norm();
  Eigen::MatrixXd HN_before = H * N;
  double norm_HN_before = HN_before.norm();

  // ---- Orthogonalise N to get Q via thin SVD ----
  // SVD is more numerically robust than QR when N columns are nearly collinear.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(N, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto &sv = svd.singularValues();

  // Rank threshold: singular values smaller than 1e-8 * max are treated as 0.
  const double sv_thresh = 1e-8 * (sv.size() > 0 ? sv(0) : 1.0);
  int rank_N = 0;
  for (int k = 0; k < sv.size(); k++)
    if (sv(k) > sv_thresh)
      rank_N++;

  double condition_N = 0.0;
  if (rank_N > 0) {
    double sv_min_nonzero = sv(rank_N - 1);
    condition_N = (sv_min_nonzero > 1e-15) ? sv(0) / sv_min_nonzero : 1e15;
  }

  if (rank_N == 0) {
    // Gauge is entirely zero — skip projection
    if (diag_out) {
      diag_out->projection_applied = false;
      diag_out->used_fej_basis = use_fej;
      diag_out->rank_N = 0;
      diag_out->condition_N = 0.0;
      diag_out->norm_HN_before = norm_HN_before;
      diag_out->norm_HN_after = norm_HN_before;
      diag_out->rel_norm_HN_before = (norm_H > 1e-12) ? norm_HN_before / norm_H : 0.0;
      diag_out->rel_norm_HN_after = diag_out->rel_norm_HN_before;
    }
    return H;
  }

  // Q: first rank_N left-singular vectors (orthonormal basis of col-space of N)
  Eigen::MatrixXd Q = svd.matrixU().leftCols(rank_N);

  // ---- Project ----
  // H_oc = H - H * Q * Q^T
  Eigen::MatrixXd H_oc = H - H * (Q * Q.transpose());

  // ---- Diagnostics: H_oc * N after projection ----
  if (diag_out) {
    Eigen::MatrixXd HN_after = H_oc * N;
    double norm_HN_after = HN_after.norm();
    diag_out->projection_applied = true;
    diag_out->used_fej_basis = use_fej;
    diag_out->rank_N = rank_N;
    diag_out->condition_N = condition_N;
    diag_out->norm_HN_before = norm_HN_before;
    diag_out->norm_HN_after = norm_HN_after;
    diag_out->rel_norm_HN_before = (norm_H > 1e-12) ? norm_HN_before / norm_H : 0.0;
    diag_out->rel_norm_HN_after = (norm_H > 1e-12) ? norm_HN_after / norm_H : 0.0;
  }

  return H_oc;
}
