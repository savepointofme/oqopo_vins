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

#include "StateHelper.h"

#include "state/Propagator.h"
#include "state/State.h"

#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "utils/colors.h"
#include "utils/print.h"

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

StateHelper::YawDxProjectionDiag last_yaw_dx_projection_diag;

Eigen::Matrix3d yaw_projection_matrix(const Eigen::Matrix3d &R_GtoI, double scale) {
  scale = std::max(0.0, std::min(1.0, scale));
  Eigen::Vector3d g_local = R_GtoI * Eigen::Vector3d::UnitZ();
  const double n = g_local.norm();
  if (n < 1e-12)
    return Eigen::Matrix3d::Identity();
  g_local /= n;
  return Eigen::Matrix3d::Identity() - (1.0 - scale) * (g_local * g_local.transpose());
}

void apply_yaw_projection_to_orientation_columns(Eigen::MatrixXd &H_eff, int col, int var_size,
                                                 const Eigen::Matrix3d &R_GtoI, double scale) {
  if (col < 0 || col + 3 > H_eff.cols() || var_size < 3)
    return;
  const Eigen::Matrix3d Cq = yaw_projection_matrix(R_GtoI, scale);
  H_eff.block(0, col, H_eff.rows(), 3) =
      (H_eff.block(0, col, H_eff.rows(), 3) * Cq).eval();
}

Eigen::MatrixXd project_per_block_yaw_from_H(std::shared_ptr<State> state,
                                             const std::vector<std::shared_ptr<Type>> &H_order,
                                             const std::vector<int> &H_id, const Eigen::MatrixXd &H,
                                             double scale) {
  scale = std::max(0.0, std::min(1.0, scale));
  if (scale >= 1.0 - 1e-12)
    return H;

  Eigen::MatrixXd H_eff = H;
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    const int col = H_id[i];

    if (var == state->_imu || var == state->_imu->pose() || var == state->_imu->q()) {
      apply_yaw_projection_to_orientation_columns(H_eff, col, var->size(), state->_imu->Rot(), scale);
      continue;
    }

    for (const auto &clone : state->_clones_IMU) {
      const auto &pose = clone.second;
      if (var == pose || var == pose->q()) {
        apply_yaw_projection_to_orientation_columns(H_eff, col, var->size(), pose->Rot(), scale);
        break;
      }
    }
  }
  return H_eff;
}

Eigen::Vector3d yaw_position_dir(const Eigen::Vector3d &p) {
  return Eigen::Vector3d::UnitZ().cross(p);
}

void fill_pose_yaw_gauge(Eigen::VectorXd &n, int col, int size, const Eigen::Matrix3d &R_GtoI,
                         const Eigen::Vector3d &p_IinG, bool has_velocity,
                         const Eigen::Vector3d &v_IinG,
                         const Eigen::Vector3d &position_reference = Eigen::Vector3d::Zero()) {
  if (col < 0 || col + size > n.rows())
    return;
  if (size >= 3) {
    n.block(col, 0, 3, 1) = R_GtoI * Eigen::Vector3d::UnitZ();
  }
  if (size >= 6) {
    n.block(col + 3, 0, 3, 1) = yaw_position_dir(p_IinG - position_reference);
  }
  if (has_velocity && size >= 9) {
    n.block(col + 6, 0, 3, 1) = yaw_position_dir(v_IinG);
  }
}

void fill_landmark_yaw_gauge(Eigen::VectorXd &n, int col, const std::shared_ptr<Landmark> &lm,
                             bool use_fej = false,
                             const Eigen::Vector3d &position_reference = Eigen::Vector3d::Zero()) {
  if (lm == nullptr || col < 0 || col + lm->size() > n.rows())
    return;
  if (LandmarkRepresentation::is_relative_representation(lm->_feat_representation))
    return;
  if (lm->_feat_representation != LandmarkRepresentation::GLOBAL_3D || lm->size() != 3)
    return;
  n.block(col, 0, 3, 1) =
      yaw_position_dir(lm->get_xyz(use_fej) - position_reference);
}

Eigen::Vector3d yaw_gauge_position_reference(std::shared_ptr<State> state,
                                             bool use_fej) {
  if (state == nullptr)
    return Eigen::Vector3d::Zero();
  if (!state->_clones_IMU.empty() && state->_clones_IMU.begin()->second != nullptr) {
    const auto &anchor = state->_clones_IMU.begin()->second;
    return use_fej ? anchor->pos_fej() : anchor->pos();
  }
  if (state->_imu != nullptr)
    return use_fej ? state->_imu->pos_fej() : state->_imu->pos();
  return Eigen::Vector3d::Zero();
}

Eigen::VectorXd build_global_yaw_gauge_small(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
                                             const std::vector<int> &H_id, int H_cols,
                                             bool use_fej = false,
                                             bool center_positions = false) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(H_cols);
  const Eigen::Vector3d position_reference = center_positions
                                                 ? yaw_gauge_position_reference(state, use_fej)
                                                 : Eigen::Vector3d::Zero();
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    const int col = H_id[i];

    if (var == state->_imu) {
      fill_pose_yaw_gauge(n, col, var->size(),
                          use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                          use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                          true,
                          use_fej ? state->_imu->vel_fej() : state->_imu->vel(),
                          position_reference);
      continue;
    }
    if (var == state->_imu->q()) {
      fill_pose_yaw_gauge(n, col, var->size(),
                          use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                          use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                          false, Eigen::Vector3d::Zero(), position_reference);
      continue;
    }
    if (var == state->_imu->p()) {
      n.block(col, 0, 3, 1) = yaw_position_dir(
          (use_fej ? state->_imu->pos_fej() : state->_imu->pos()) -
          position_reference);
      continue;
    }
    if (var == state->_imu->v()) {
      n.block(col, 0, 3, 1) = yaw_position_dir(use_fej ? state->_imu->vel_fej() : state->_imu->vel());
      continue;
    }
    if (var == state->_imu->bg() || var == state->_imu->ba()) {
      continue;
    }

    bool matched_clone = false;
    for (const auto &clone : state->_clones_IMU) {
      const auto &pose = clone.second;
      if (var == pose) {
        fill_pose_yaw_gauge(n, col, var->size(),
                            use_fej ? pose->Rot_fej() : pose->Rot(),
                            use_fej ? pose->pos_fej() : pose->pos(),
                            false, Eigen::Vector3d::Zero(), position_reference);
        matched_clone = true;
        break;
      }
      if (var == pose->q()) {
        fill_pose_yaw_gauge(n, col, var->size(),
                            use_fej ? pose->Rot_fej() : pose->Rot(),
                            use_fej ? pose->pos_fej() : pose->pos(),
                            false, Eigen::Vector3d::Zero());
        matched_clone = true;
        break;
      }
      if (var == pose->p()) {
        n.block(col, 0, 3, 1) = yaw_position_dir(
            (use_fej ? pose->pos_fej() : pose->pos()) - position_reference);
        matched_clone = true;
        break;
      }
    }
    if (matched_clone)
      continue;

    fill_landmark_yaw_gauge(n, col, std::dynamic_pointer_cast<Landmark>(var),
                            use_fej, position_reference);
  }
  return n;
}

void fill_translation_gauge(Eigen::MatrixXd &N, int col) {
  if (col < 0 || col + 3 > N.rows() || N.cols() < 4)
    return;
  N.block<3, 3>(col, 1) = Eigen::Matrix3d::Identity();
}

Eigen::MatrixXd build_global_4d_gauge_small(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const std::vector<int> &H_id, int H_cols, bool use_fej) {
  Eigen::MatrixXd N = Eigen::MatrixXd::Zero(H_cols, 4);
  N.col(0) = build_global_yaw_gauge_small(
      state, H_order, H_id, H_cols, use_fej, false);

  for (size_t i = 0; i < H_order.size(); ++i) {
    const auto &var = H_order[i];
    const int col = H_id[i];
    if (var == state->_imu) {
      fill_translation_gauge(N, col + 3);
      continue;
    }
    if (var == state->_imu->p()) {
      fill_translation_gauge(N, col);
      continue;
    }
    if (var == state->_imu->q() || var == state->_imu->v() ||
        var == state->_imu->bg() || var == state->_imu->ba())
      continue;

    bool matched_clone = false;
    for (const auto &clone : state->_clones_IMU) {
      const auto &pose = clone.second;
      if (var == pose) {
        fill_translation_gauge(N, col + 3);
        matched_clone = true;
        break;
      }
      if (var == pose->p()) {
        fill_translation_gauge(N, col);
        matched_clone = true;
        break;
      }
      if (var == pose->q()) {
        matched_clone = true;
        break;
      }
    }
    if (matched_clone)
      continue;

    const auto landmark = std::dynamic_pointer_cast<Landmark>(var);
    if (landmark != nullptr &&
        landmark->_feat_representation == LandmarkRepresentation::GLOBAL_3D &&
        landmark->size() == 3)
      fill_translation_gauge(N, col);
  }
  return N;
}

// Once-per-process flags for gauge-coverage diagnostic print (one per source).
bool g_gauge_full_diag_printed = false;
bool g_gauge_full_fej_diag_printed = false;

// Build the global yaw gauge over the FULL state vector.
// Unlike build_global_yaw_gauge_small (which only covers H_order variables), this version
// covers every variable in the filter so the yaw gauge direction is consistent.
// in the complete N-dimensional state space.
//
// N is passed in (= state->_Cov.rows()) since _Cov is private and this function
// lives in the anonymous namespace.  The caller (a StateHelper static member) can
// access it.
//
// Iteration order: IMU, all clones, all SLAM features.
// Camera extrinsics / intrinsics / time-offset: gauge = 0 (relative/scalar quantities).
//
// On the first call a summary is printed via PRINT_INFO to confirm coverage and flag any
// composite-variable duplication.
Eigen::VectorXd build_global_yaw_gauge_full(std::shared_ptr<State> state, int N,
                                            bool use_fej = false,
                                            bool center_positions = false) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(N);
  const Eigen::Vector3d position_reference = center_positions
                                                 ? yaw_gauge_position_reference(state, use_fej)
                                                 : Eigen::Vector3d::Zero();

  bool imu_covered  = false;
  int  n_clones     = 0;
  int  n_slam       = 0;
  int  n_slam_skip  = 0;   // relative/anchored landmarks (gauge not defined)

  // IMU state (q + p + v + bg + ba).
  if (state->_imu != nullptr) {
    fill_pose_yaw_gauge(n, state->_imu->id(), state->_imu->size(),
                        use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                        use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                        true,
                        use_fej ? state->_imu->vel_fej() : state->_imu->vel(),
                        position_reference);
    imu_covered = true;
  }

  // All camera clones (PoseJPL, 6-DOF: q + p).
  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &pose = clone_pair.second;
    if (pose == nullptr) continue;
    fill_pose_yaw_gauge(n, pose->id(), pose->size(),
                        use_fej ? pose->Rot_fej() : pose->Rot(),
                        use_fej ? pose->pos_fej() : pose->pos(),
                        false, Eigen::Vector3d::Zero(), position_reference);
    n_clones++;
  }

  // SLAM features.
  for (const auto &feat_pair : state->_features_SLAM) {
    const auto &lm = feat_pair.second;
    if (lm == nullptr) continue;
    if (LandmarkRepresentation::is_relative_representation(lm->_feat_representation)) {
      n_slam_skip++;
    } else {
      fill_landmark_yaw_gauge(n, lm->id(), lm, use_fej, position_reference);
      n_slam++;
    }
  }

  // Camera extrinsics (relative sensor transform), intrinsics, time offset:
  // gauge contribution = 0.  These are already zeroed in n; nothing to do.
  // Calibration scalars (IMU intrinsics): gauge = 0 by the same reasoning.

  // ---- one-time diagnostic summary ----------------------------------------
  bool &printed_flag = use_fej ? g_gauge_full_fej_diag_printed : g_gauge_full_diag_printed;
  if (!printed_flag) {
    printed_flag = true;

    // Count nonzero entries in n to confirm actual fill.
    int n_nonzero = 0;
    for (int i = 0; i < N; i++)
      if (n(i) != 0.0) n_nonzero++;

    // Count extrinsic / intrinsic / time-offset DOFs (zero gauge).
    int n_zero_calib_dof = 0;
    for (const auto &kv : state->_calib_IMUtoCAM)
      if (kv.second) n_zero_calib_dof += kv.second->size();
    for (const auto &kv : state->_cam_intrinsics)
      if (kv.second) n_zero_calib_dof += kv.second->size();
    if (state->_calib_dt_CAMtoIMU)
      n_zero_calib_dof += state->_calib_dt_CAMtoIMU->size();
    int n_slam_skip_dof = 0;
    for (const auto &fp : state->_features_SLAM)
      if (fp.second && LandmarkRepresentation::is_relative_representation(
                           fp.second->_feat_representation))
        n_slam_skip_dof += fp.second->size();

    PRINT_INFO(GREEN
               "[YAW-GAUGE] gauge_full first-call summary (source=%s):\n"
               "  cov_dim N         = %d\n"
               "  q_full nonzero    = %d\n"
               "  IMU covered       = %s  (id=%d size=%d)\n"
               "  clone count       = %d\n"
               "  SLAM global       = %d (filled)\n"
               "  SLAM relative     = %d (skipped, gauge=0, %d DOF)\n"
               "  cam_extr/intr/dt  = 0 (gauge=0, %d DOF)\n"
               "  q_full norm       = %.6f\n"
               "  position anchor   = [%.3f %.3f %.3f] (%s)\n"
               RESET,
               use_fej ? "fej" : "current",
               N, n_nonzero,
               imu_covered ? "YES" : "NO",
               state->_imu ? state->_imu->id() : -1,
               state->_imu ? state->_imu->size() : 0,
               n_clones,
               n_slam, n_slam_skip, n_slam_skip_dof,
               n_zero_calib_dof,
               n.norm(),
               position_reference.x(), position_reference.y(),
               position_reference.z(), center_positions ? "centered" : "global-origin");
  }

  return n;
}

Eigen::MatrixXd project_global_yaw_from_H(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
                                          const std::vector<int> &H_id, const Eigen::MatrixXd &H, double alpha,
                                          bool use_fej = false,
                                          bool center_positions = false) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  if (alpha <= 1e-12)
    return H;
  Eigen::VectorXd n = build_global_yaw_gauge_small(
      state, H_order, H_id, H.cols(), use_fej, center_positions);
  const double n2 = n.squaredNorm();
  if (n2 < 1e-12)
    return H;
  Eigen::VectorXd Hn = H * n;
  return H - (alpha / n2) * Hn * n.transpose();
}

Eigen::MatrixXd project_global_4d_from_H(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const std::vector<int> &H_id, const Eigen::MatrixXd &H, double alpha,
    bool use_fej = false) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  if (alpha <= 1e-12)
    return H;
  const Eigen::MatrixXd N = build_global_4d_gauge_small(
      state, H_order, H_id, H.cols(), use_fej);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      N, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0 || singular_values(0) <= 1e-12)
    return H;
  const double threshold = 1e-8 * singular_values(0);
  int rank = 0;
  while (rank < singular_values.size() && singular_values(rank) > threshold)
    ++rank;
  if (rank == 0)
    return H;
  const Eigen::MatrixXd Q = svd.matrixU().leftCols(rank);
  return H - alpha * (H * Q) * Q.transpose();
}

double yaw_dx_component_deg(std::shared_ptr<State> state, const Eigen::VectorXd &dx) {
  const auto &q_var = state->_imu->q();
  if (q_var == nullptr || q_var->id() < 0 || q_var->id() + 3 > dx.rows())
    return 0.0;
  Eigen::Vector3d g_local = state->_imu->Rot() * Eigen::Vector3d::UnitZ();
  const double n = g_local.norm();
  if (n < 1e-12)
    return 0.0;
  g_local /= n;
  constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
  return g_local.dot(dx.segment(q_var->id(), 3)) * rad_to_deg;
}

constexpr double kScaleResetEpsilon = 1e-12;
constexpr double kScaleResetStorageTolerance = 1e-12;
constexpr double kScaleResetQuaternionTolerance = 1e-6;
constexpr double kScaleResetSymmetryTolerance = 1e-10;
constexpr double kScaleResetPsdRoundoffFactor = 64.0;

Eigen::Matrix3d scale_reset_skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d out;
  out << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return out;
}

bool scale_reset_matrix_near(const Eigen::MatrixXd &a,
                             const Eigen::MatrixXd &b) {
  if (a.rows() != b.rows() || a.cols() != b.cols() || !a.allFinite() ||
      !b.allFinite())
    return false;
  const double scale =
      std::max(1.0, std::max(a.cwiseAbs().maxCoeff(), b.cwiseAbs().maxCoeff()));
  return (a - b).cwiseAbs().maxCoeff() <=
         kScaleResetStorageTolerance * scale;
}

bool scale_reset_valid_quaternion(const Eigen::Vector4d &q) {
  return q.allFinite() &&
         std::abs(q.squaredNorm() - 1.0) <=
             kScaleResetQuaternionTolerance;
}

bool scale_reset_pose_consistent(const std::shared_ptr<PoseJPL> &pose,
                                 std::string &reason) {
  if (pose == nullptr) {
    reason = "null pose";
    return false;
  }
  if (pose->size() != 6 || pose->value().rows() != 7 ||
      pose->value().cols() != 1 || pose->fej().rows() != 7 ||
      pose->fej().cols() != 1 || !pose->value().allFinite() ||
      !pose->fej().allFinite()) {
    reason = "invalid pose storage";
    return false;
  }
  if (!scale_reset_valid_quaternion(pose->quat()) ||
      !scale_reset_valid_quaternion(pose->quat_fej())) {
    reason = "non-unit pose quaternion";
    return false;
  }
  if (!scale_reset_matrix_near(pose->value().block(0, 0, 4, 1),
                               pose->q()->value()) ||
      !scale_reset_matrix_near(pose->value().block(4, 0, 3, 1),
                               pose->p()->value()) ||
      !scale_reset_matrix_near(pose->fej().block(0, 0, 4, 1),
                               pose->q()->fej()) ||
      !scale_reset_matrix_near(pose->fej().block(4, 0, 3, 1),
                               pose->p()->fej())) {
    reason = "pose composite/substate mismatch";
    return false;
  }
  return pose->Rot().allFinite() && pose->Rot_fej().allFinite();
}

bool scale_reset_imu_consistent(const std::shared_ptr<IMU> &imu,
                                std::string &reason) {
  if (imu == nullptr) {
    reason = "null IMU";
    return false;
  }
  if (imu->size() != 15 || imu->value().rows() != 16 ||
      imu->value().cols() != 1 || imu->fej().rows() != 16 ||
      imu->fej().cols() != 1 || !imu->value().allFinite() ||
      !imu->fej().allFinite()) {
    reason = "invalid IMU storage";
    return false;
  }
  if (!scale_reset_pose_consistent(imu->pose(), reason))
    return false;
  const bool value_synced =
      scale_reset_matrix_near(imu->value().block(0, 0, 7, 1),
                              imu->pose()->value()) &&
      scale_reset_matrix_near(imu->value().block(7, 0, 3, 1),
                              imu->v()->value()) &&
      scale_reset_matrix_near(imu->value().block(10, 0, 3, 1),
                              imu->bg()->value()) &&
      scale_reset_matrix_near(imu->value().block(13, 0, 3, 1),
                              imu->ba()->value());
  const bool fej_synced =
      scale_reset_matrix_near(imu->fej().block(0, 0, 7, 1),
                              imu->pose()->fej()) &&
      scale_reset_matrix_near(imu->fej().block(7, 0, 3, 1),
                              imu->v()->fej()) &&
      scale_reset_matrix_near(imu->fej().block(10, 0, 3, 1),
                              imu->bg()->fej()) &&
      scale_reset_matrix_near(imu->fej().block(13, 0, 3, 1),
                              imu->ba()->fej());
  if (!value_synced || !fej_synced) {
    reason = "IMU composite/substate mismatch";
    return false;
  }
  return true;
}

struct ScaleResetCameraKinematics {
  Eigen::Vector3d lever_in_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d lever_in_global = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_in_global = Eigen::Vector3d::Zero();
  Eigen::Matrix3d dcenter_dpose_theta = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dcenter_dextrinsic_theta = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d dcenter_dextrinsic_position = Eigen::Matrix3d::Zero();
};

bool scale_reset_camera_kinematics(
    const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
    const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC,
    ScaleResetCameraKinematics &out) {
  if (!R_GtoI.allFinite() || !p_IinG.allFinite() ||
      !R_ItoC.allFinite() || !p_IinC.allFinite())
    return false;
  const Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
  const Eigen::Matrix3d R_CtoI = R_ItoC.transpose();
  out.lever_in_imu = -R_CtoI * p_IinC;
  out.lever_in_global = R_ItoG * out.lever_in_imu;
  out.center_in_global = p_IinG + out.lever_in_global;
  out.dcenter_dpose_theta =
      -R_ItoG * scale_reset_skew(out.lever_in_imu);
  out.dcenter_dextrinsic_theta =
      R_ItoG * R_CtoI * scale_reset_skew(p_IinC);
  out.dcenter_dextrinsic_position = -R_ItoG * R_CtoI;
  return out.lever_in_imu.allFinite() && out.lever_in_global.allFinite() &&
         out.center_in_global.allFinite() &&
         out.dcenter_dpose_theta.allFinite() &&
         out.dcenter_dextrinsic_theta.allFinite() &&
         out.dcenter_dextrinsic_position.allFinite();
}

bool scale_reset_inverse_depth_xyz(const Eigen::Vector3d &inverse,
                                   Eigen::Vector3d &xyz,
                                   Eigen::Matrix3d *dxyz_dinverse) {
  const double theta = inverse(0);
  const double phi = inverse(1);
  const double rho = inverse(2);
  if (!inverse.allFinite() || rho <= kScaleResetEpsilon)
    return false;
  const double radius = 1.0 / rho;
  const double st = std::sin(theta);
  const double ct = std::cos(theta);
  const double sp = std::sin(phi);
  const double cp = std::cos(phi);
  xyz << radius * ct * sp, radius * st * sp, radius * cp;
  if (!xyz.allFinite())
    return false;
  if (dxyz_dinverse != nullptr) {
    dxyz_dinverse->col(0) << -radius * st * sp, radius * ct * sp, 0.0;
    dxyz_dinverse->col(1) << radius * ct * cp, radius * st * cp,
        -radius * sp;
    dxyz_dinverse->col(2) = -xyz / rho;
    if (!dxyz_dinverse->allFinite())
      return false;
  }
  return true;
}

bool scale_reset_transform_global_inverse_depth(
    const Eigen::Vector3d &inverse, const Eigen::Vector3d &pivot,
    double scale, Eigen::Vector3d &inverse_new,
    Eigen::Matrix3d *dinverse_new_dinverse,
    Eigen::Matrix3d *dinverse_new_dpivot) {
  Eigen::Vector3d xyz;
  Eigen::Matrix3d dxyz_dinverse;
  if (!scale_reset_inverse_depth_xyz(inverse, xyz, &dxyz_dinverse))
    return false;
  const Eigen::Vector3d xyz_new = pivot + scale * (xyz - pivot);
  const double radius_new = xyz_new.norm();
  if (!xyz_new.allFinite() || !std::isfinite(radius_new) ||
      radius_new <= kScaleResetEpsilon)
    return false;

  constexpr double pi = 3.14159265358979323846;
  double theta_new = std::atan2(xyz_new.y(), xyz_new.x());
  theta_new += 2.0 * pi *
               std::round((inverse(0) - theta_new) / (2.0 * pi));
  const double cos_phi =
      std::max(-1.0, std::min(1.0, xyz_new.z() / radius_new));
  inverse_new << theta_new, std::acos(cos_phi), 1.0 / radius_new;

  Eigen::Matrix3d dxyz_new_dinverse_new;
  Eigen::Vector3d xyz_roundtrip;
  if (!scale_reset_inverse_depth_xyz(inverse_new, xyz_roundtrip,
                                     &dxyz_new_dinverse_new))
    return false;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      dxyz_new_dinverse_new, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d singular_values = svd.singularValues();
  if (!singular_values.allFinite() || singular_values.maxCoeff() <= 0.0 ||
      singular_values.minCoeff() <=
          kScaleResetEpsilon * singular_values.maxCoeff())
    return false;
  const Eigen::Matrix3d dinverse_new_dxyz =
      svd.solve(Eigen::Matrix3d::Identity());
  if (dinverse_new_dinverse != nullptr)
    *dinverse_new_dinverse =
        dinverse_new_dxyz * (scale * dxyz_dinverse);
  if (dinverse_new_dpivot != nullptr)
    *dinverse_new_dpivot = (1.0 - scale) * dinverse_new_dxyz;
  return inverse_new.allFinite() && dinverse_new_dxyz.allFinite() &&
         (dinverse_new_dinverse == nullptr ||
          dinverse_new_dinverse->allFinite()) &&
         (dinverse_new_dpivot == nullptr ||
          dinverse_new_dpivot->allFinite());
}

bool gauge_reset_transform_global_inverse_depth(
    const Eigen::Vector3d &inverse, const Eigen::Matrix3d &rotation,
    const Eigen::Vector3d &translation, Eigen::Vector3d &inverse_new,
    Eigen::Matrix3d *dinverse_new_dinverse) {
  Eigen::Vector3d xyz;
  Eigen::Matrix3d dxyz_dinverse;
  if (!scale_reset_inverse_depth_xyz(inverse, xyz, &dxyz_dinverse))
    return false;
  const Eigen::Vector3d xyz_new = rotation * xyz + translation;
  const double radius_new = xyz_new.norm();
  if (!xyz_new.allFinite() || !std::isfinite(radius_new) ||
      radius_new <= kScaleResetEpsilon)
    return false;

  constexpr double pi = 3.14159265358979323846;
  double theta_new = std::atan2(xyz_new.y(), xyz_new.x());
  theta_new += 2.0 * pi *
               std::round((inverse(0) - theta_new) / (2.0 * pi));
  const double cos_phi =
      std::max(-1.0, std::min(1.0, xyz_new.z() / radius_new));
  inverse_new << theta_new, std::acos(cos_phi), 1.0 / radius_new;

  if (dinverse_new_dinverse != nullptr) {
    Eigen::Matrix3d dxyz_new_dinverse_new;
    Eigen::Vector3d xyz_roundtrip;
    if (!scale_reset_inverse_depth_xyz(inverse_new, xyz_roundtrip,
                                       &dxyz_new_dinverse_new))
      return false;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        dxyz_new_dinverse_new, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d singular_values = svd.singularValues();
    if (!singular_values.allFinite() || singular_values.maxCoeff() <= 0.0 ||
        singular_values.minCoeff() <=
            kScaleResetEpsilon * singular_values.maxCoeff())
      return false;
    const Eigen::Matrix3d dinverse_new_dxyz =
        svd.solve(Eigen::Matrix3d::Identity());
    *dinverse_new_dinverse =
        dinverse_new_dxyz * rotation * dxyz_dinverse;
  }
  return inverse_new.allFinite() &&
         (dinverse_new_dinverse == nullptr ||
          dinverse_new_dinverse->allFinite());
}

struct ScaleResetCovarianceHealth {
  bool valid = false;
  double symmetry_error = std::numeric_limits<double>::infinity();
  double min_diagonal = -std::numeric_limits<double>::infinity();
  double min_eigenvalue = -std::numeric_limits<double>::infinity();
  double psd_tolerance = 0.0;
  std::string reason;
};

ScaleResetCovarianceHealth
scale_reset_covariance_health(const Eigen::MatrixXd &covariance) {
  ScaleResetCovarianceHealth out;
  if (covariance.rows() <= 0 || covariance.rows() != covariance.cols()) {
    out.reason = "covariance is not a non-empty square matrix";
    return out;
  }
  if (!covariance.allFinite()) {
    out.reason = "covariance contains non-finite values";
    return out;
  }
  const double covariance_scale = std::max(1.0, covariance.norm());
  out.symmetry_error = (covariance - covariance.transpose()).norm();
  if (out.symmetry_error >
      kScaleResetSymmetryTolerance * covariance_scale) {
    out.reason = "covariance is not symmetric";
    return out;
  }
  const Eigen::MatrixXd symmetric =
      0.5 * (covariance + covariance.transpose());
  const double spectral_upper_bound = std::max(
      1.0, symmetric.cwiseAbs().rowwise().sum().maxCoeff());
  out.psd_tolerance =
      kScaleResetPsdRoundoffFactor * std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max<Eigen::Index>(1, covariance.rows())) *
      spectral_upper_bound;
  out.min_diagonal = covariance.diagonal().minCoeff();
  if (out.min_diagonal < -out.psd_tolerance) {
    out.reason = "covariance has a negative diagonal";
    return out;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(
      symmetric, Eigen::EigenvaluesOnly);
  if (eigen_solver.info() != Eigen::Success ||
      !eigen_solver.eigenvalues().allFinite()) {
    out.reason = "covariance eigendecomposition failed";
    return out;
  }
  out.min_eigenvalue = eigen_solver.eigenvalues().minCoeff();
  if (out.min_eigenvalue < -out.psd_tolerance) {
    out.reason = "covariance is not positive semidefinite";
    return out;
  }
  out.valid = true;
  return out;
}

struct ScaleResetPendingValue {
  std::shared_ptr<Type> type;
  Eigen::MatrixXd value_before;
  Eigen::MatrixXd fej_before;
  Eigen::MatrixXd value_after;
  Eigen::MatrixXd fej_after;
};

} // namespace

StateHelper::Sim3ScaleResetResult StateHelper::apply_sim3_scale_reset(
    std::shared_ptr<State> state, double scale, Propagator *propagator,
    std::size_t camera_id) {
  Sim3ScaleResetResult result;
  result.scale = scale;
  result.camera_id = camera_id;
  auto reject = [&result](const std::string &reason) {
    result.failure_reason = reason;
    return result;
  };

  if (state == nullptr)
    return reject("state is null");
  if (!std::isfinite(scale) || scale <= 0.0)
    return reject("scale must be finite and strictly positive");

  std::lock_guard<std::mutex> lock(state->_mutex_state);
  if (state->_options.num_cameras != 1)
    return reject("scale reset supports exactly one camera");
  if (camera_id != 0)
    return reject("scale reset supports only camera_id 0");
  if (state->_calib_IMUtoCAM.size() != 1 ||
      state->_calib_IMUtoCAM.count(0) != 1)
    return reject("single-camera calibration map is inconsistent");
  const int covariance_size = static_cast<int>(state->_Cov.rows());
  if (covariance_size <= 0 || state->_Cov.cols() != covariance_size)
    return reject("state covariance has invalid dimensions");

  std::unordered_set<const Type *> variable_pointers;
  int expected_id = 0;
  for (const auto &variable : state->_variables) {
    if (variable == nullptr)
      return reject("state variable list contains a null entry");
    if (!variable_pointers.insert(variable.get()).second)
      return reject("state variable list contains a duplicate entry");
    if (variable->size() <= 0 || variable->id() != expected_id ||
        variable->id() + variable->size() > covariance_size)
      return reject("state variables are not contiguous with covariance");
    if (variable->value().size() <= 0 || variable->fej().size() <= 0 ||
        !variable->value().allFinite() || !variable->fej().allFinite())
      return reject("state variable contains invalid nominal or FEJ data");
    expected_id += variable->size();
  }
  if (expected_id != covariance_size)
    return reject("state variable dimensions do not match covariance");
  if (state->_imu == nullptr ||
      variable_pointers.count(state->_imu.get()) != 1)
    return reject("active IMU is missing from the state variable list");

  std::string consistency_reason;
  if (!scale_reset_imu_consistent(state->_imu, consistency_reason))
    return reject("active IMU is inconsistent: " + consistency_reason);

  std::unordered_set<const Type *> calibration_pointers;
  for (const auto &calibration_pair : state->_calib_IMUtoCAM) {
    const auto &calibration = calibration_pair.second;
    if (!scale_reset_pose_consistent(calibration, consistency_reason))
      return reject("camera extrinsic is inconsistent: " +
                    consistency_reason);
    if (!calibration_pointers.insert(calibration.get()).second)
      return reject("camera extrinsic map contains a duplicate pose");
    if (calibration->id() >= 0 &&
        variable_pointers.count(calibration.get()) != 1)
      return reject("estimated camera extrinsic is absent from state ordering");
  }
  for (const auto &intrinsic_pair : state->_cam_intrinsics) {
    const auto &intrinsic = intrinsic_pair.second;
    if (intrinsic == nullptr || !intrinsic->value().allFinite() ||
        !intrinsic->fej().allFinite())
      return reject("camera intrinsics contain invalid data");
  }

  std::unordered_set<const Type *> clone_pointers;
  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &clone = clone_pair.second;
    if (!std::isfinite(clone_pair.first) ||
        !scale_reset_pose_consistent(clone, consistency_reason))
      return reject("IMU clone is inconsistent: " + consistency_reason);
    if (!clone_pointers.insert(clone.get()).second)
      return reject("clone map contains a duplicate pose");
    if (variable_pointers.count(clone.get()) != 1)
      return reject("IMU clone is absent from state ordering");
  }

  std::unordered_set<const Type *> landmark_pointers;
  for (const auto &feature_pair : state->_features_SLAM) {
    const auto &landmark = feature_pair.second;
    if (landmark == nullptr)
      return reject("SLAM landmark map contains a null entry");
    if (!landmark_pointers.insert(landmark.get()).second)
      return reject("SLAM landmark map contains a duplicate entry");
    if (variable_pointers.count(landmark.get()) != 1)
      return reject("SLAM landmark is absent from state ordering");
    if (!landmark->value().allFinite() || !landmark->fej().allFinite())
      return reject("SLAM landmark contains non-finite nominal or FEJ data");

    const auto representation = landmark->_feat_representation;
    const bool three_parameter =
        representation == LandmarkRepresentation::GLOBAL_3D ||
        representation == LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH ||
        representation == LandmarkRepresentation::ANCHORED_3D ||
        representation == LandmarkRepresentation::ANCHORED_FULL_INVERSE_DEPTH ||
        representation == LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH;
    const bool single_parameter =
        representation == LandmarkRepresentation::ANCHORED_INVERSE_DEPTH_SINGLE;
    if ((!three_parameter && !single_parameter) ||
        (three_parameter && landmark->size() != 3) ||
        (single_parameter && landmark->size() != 1) ||
        landmark->value().rows() != landmark->size() ||
        landmark->fej().rows() != landmark->size())
      return reject("SLAM landmark representation and storage disagree");

    if (LandmarkRepresentation::is_relative_representation(representation)) {
      if (landmark->_anchor_cam_id < 0 ||
          state->_calib_IMUtoCAM.count(
              static_cast<std::size_t>(landmark->_anchor_cam_id)) != 1 ||
          !std::isfinite(landmark->_anchor_clone_timestamp) ||
          state->_clones_IMU.count(landmark->_anchor_clone_timestamp) != 1)
        return reject("anchored SLAM landmark has an invalid anchor");
    }
    if (single_parameter &&
        (!landmark->uv_norm_zero.allFinite() ||
         !landmark->uv_norm_zero_fej.allFinite()))
      return reject("single inverse-depth landmark has an invalid bearing");
  }
  for (const auto &variable : state->_variables) {
    const auto landmark = std::dynamic_pointer_cast<Landmark>(variable);
    if (landmark != nullptr && landmark_pointers.count(landmark.get()) != 1)
      return reject("state ordering contains an unregistered SLAM landmark");
  }

  const auto prior_health = scale_reset_covariance_health(state->_Cov);
  result.covariance_min_eigenvalue = prior_health.min_eigenvalue;
  result.covariance_psd_projection_limit = prior_health.psd_tolerance;
  result.covariance_psd_projection_magnitude =
      std::max(0.0, -prior_health.min_eigenvalue);
  if (!prior_health.valid)
    return reject("prior " + prior_health.reason);

  const auto calibration_it = state->_calib_IMUtoCAM.find(camera_id);
  if (calibration_it == state->_calib_IMUtoCAM.end() ||
      calibration_it->second == nullptr)
    return reject("pivot camera calibration is unavailable");
  const auto &calibration = calibration_it->second;

  ScaleResetCameraKinematics pivot;
  ScaleResetCameraKinematics pivot_fej;
  if (!scale_reset_camera_kinematics(
          state->_imu->Rot(), state->_imu->pos(), calibration->Rot(),
          calibration->pos(), pivot) ||
      !scale_reset_camera_kinematics(
          state->_imu->Rot_fej(), state->_imu->pos_fej(),
          calibration->Rot_fej(), calibration->pos_fej(), pivot_fej))
    return reject("failed to form current camera pivot");
  result.pivot_camera_center = pivot.center_in_global;

  if (scale == 1.0) {
    result.success = true;
    result.failure_reason.clear();
    return result;
  }
  if (propagator == nullptr)
    return reject("non-identity reset requires a propagator cache handle");

  const int imu_q_id = state->_imu->q()->id();
  const int imu_p_id = state->_imu->p()->id();
  const int imu_v_id = state->_imu->v()->id();
  const bool estimated_extrinsic = calibration->id() >= 0;
  if (estimated_extrinsic &&
      variable_pointers.count(calibration.get()) != 1)
    return reject("pivot camera extrinsic covariance block is unavailable");

  Eigen::MatrixXd reset_jacobian =
      Eigen::MatrixXd::Identity(covariance_size, covariance_size);
  std::vector<ScaleResetPendingValue> pending;
  pending.reserve(1 + state->_clones_IMU.size() +
                  state->_features_SLAM.size());

  ScaleResetPendingValue imu_pending;
  imu_pending.type = state->_imu;
  imu_pending.value_before = state->_imu->value();
  imu_pending.fej_before = state->_imu->fej();
  imu_pending.value_after = imu_pending.value_before;
  imu_pending.fej_after = imu_pending.fej_before;
  imu_pending.value_after.block(7, 0, 3, 1) *= scale;
  imu_pending.fej_after.block(7, 0, 3, 1) *= scale;
  pending.push_back(std::move(imu_pending));
  reset_jacobian.block(imu_v_id, 0, 3, covariance_size).setZero();
  reset_jacobian.block<3, 3>(imu_v_id, imu_v_id) =
      scale * Eigen::Matrix3d::Identity();

  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &clone = clone_pair.second;
    ScaleResetCameraKinematics clone_camera;
    ScaleResetCameraKinematics clone_camera_fej;
    if (!scale_reset_camera_kinematics(
            clone->Rot(), clone->pos(), calibration->Rot(),
            calibration->pos(), clone_camera) ||
        !scale_reset_camera_kinematics(
            clone->Rot_fej(), clone->pos_fej(), calibration->Rot_fej(),
            calibration->pos_fej(), clone_camera_fej))
      return reject("failed to form a clone camera center");

    ScaleResetPendingValue clone_pending;
    clone_pending.type = clone;
    clone_pending.value_before = clone->value();
    clone_pending.fej_before = clone->fej();
    clone_pending.value_after = clone_pending.value_before;
    clone_pending.fej_after = clone_pending.fej_before;
    clone_pending.value_after.block(4, 0, 3, 1) =
        pivot.center_in_global +
        scale * (clone_camera.center_in_global - pivot.center_in_global) -
        clone_camera.lever_in_global;
    clone_pending.fej_after.block(4, 0, 3, 1) =
        pivot_fej.center_in_global +
        scale * (clone_camera_fej.center_in_global -
                 pivot_fej.center_in_global) -
        clone_camera_fej.lever_in_global;
    if (!clone_pending.value_after.allFinite() ||
        !clone_pending.fej_after.allFinite())
      return reject("clone scale transform produced non-finite data");
    pending.push_back(std::move(clone_pending));

    const int clone_q_id = clone->q()->id();
    const int clone_p_id = clone->p()->id();
    reset_jacobian.block(clone_p_id, 0, 3, covariance_size).setZero();
    reset_jacobian.block<3, 3>(clone_p_id, clone_p_id) =
        scale * Eigen::Matrix3d::Identity();
    reset_jacobian.block<3, 3>(clone_p_id, imu_p_id) =
        (1.0 - scale) * Eigen::Matrix3d::Identity();
    reset_jacobian.block<3, 3>(clone_p_id, clone_q_id) =
        (scale - 1.0) * clone_camera.dcenter_dpose_theta;
    reset_jacobian.block<3, 3>(clone_p_id, imu_q_id) =
        (1.0 - scale) * pivot.dcenter_dpose_theta;
    if (estimated_extrinsic) {
      reset_jacobian.block<3, 3>(clone_p_id, calibration->q()->id()) =
          (scale - 1.0) *
          (clone_camera.dcenter_dextrinsic_theta -
           pivot.dcenter_dextrinsic_theta);
      reset_jacobian.block<3, 3>(clone_p_id, calibration->p()->id()) =
          (scale - 1.0) *
          (clone_camera.dcenter_dextrinsic_position -
           pivot.dcenter_dextrinsic_position);
    }
  }

  auto add_pivot_jacobian = [&](int row,
                                const Eigen::Matrix3d &drow_dcenter) {
    reset_jacobian.block<3, 3>(row, imu_p_id) += drow_dcenter;
    reset_jacobian.block<3, 3>(row, imu_q_id) +=
        drow_dcenter * pivot.dcenter_dpose_theta;
    if (estimated_extrinsic) {
      reset_jacobian.block<3, 3>(row, calibration->q()->id()) +=
          drow_dcenter * pivot.dcenter_dextrinsic_theta;
      reset_jacobian.block<3, 3>(row, calibration->p()->id()) +=
          drow_dcenter * pivot.dcenter_dextrinsic_position;
    }
  };

  for (const auto &feature_pair : state->_features_SLAM) {
    const auto &landmark = feature_pair.second;
    const auto representation = landmark->_feat_representation;
    ScaleResetPendingValue landmark_pending;
    landmark_pending.type = landmark;
    landmark_pending.value_before = landmark->value();
    landmark_pending.fej_before = landmark->fej();
    landmark_pending.value_after = landmark_pending.value_before;
    landmark_pending.fej_after = landmark_pending.fej_before;

    const int landmark_id = landmark->id();
    reset_jacobian.block(landmark_id, 0, landmark->size(),
                         covariance_size)
        .setZero();

    if (representation == LandmarkRepresentation::GLOBAL_3D) {
      landmark_pending.value_after =
          pivot.center_in_global +
          scale * (landmark->value() - pivot.center_in_global);
      landmark_pending.fej_after =
          pivot_fej.center_in_global +
          scale * (landmark->fej() - pivot_fej.center_in_global);
      reset_jacobian.block<3, 3>(landmark_id, landmark_id) =
          scale * Eigen::Matrix3d::Identity();
      add_pivot_jacobian(
          landmark_id,
          (1.0 - scale) * Eigen::Matrix3d::Identity());
    } else if (representation ==
               LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH) {
      Eigen::Vector3d inverse_new;
      Eigen::Vector3d inverse_fej_new;
      Eigen::Matrix3d dinverse_new_dinverse;
      Eigen::Matrix3d dinverse_new_dpivot;
      if (!scale_reset_transform_global_inverse_depth(
              landmark->value(), pivot.center_in_global, scale, inverse_new,
              &dinverse_new_dinverse, &dinverse_new_dpivot) ||
          !scale_reset_transform_global_inverse_depth(
              landmark->fej(), pivot_fej.center_in_global, scale,
              inverse_fej_new, nullptr, nullptr))
        return reject("global inverse-depth landmark reset is singular");
      landmark_pending.value_after = inverse_new;
      landmark_pending.fej_after = inverse_fej_new;
      reset_jacobian.block<3, 3>(landmark_id, landmark_id) =
          dinverse_new_dinverse;
      add_pivot_jacobian(landmark_id, dinverse_new_dpivot);
    } else if (representation == LandmarkRepresentation::ANCHORED_3D) {
      landmark_pending.value_after *= scale;
      landmark_pending.fej_after *= scale;
      reset_jacobian.block<3, 3>(landmark_id, landmark_id) =
          scale * Eigen::Matrix3d::Identity();
    } else if (
        representation == LandmarkRepresentation::ANCHORED_FULL_INVERSE_DEPTH ||
        representation == LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH) {
      if (landmark->value()(2) <= kScaleResetEpsilon ||
          landmark->fej()(2) <= kScaleResetEpsilon)
        return reject("anchored inverse-depth landmark has non-positive depth");
      landmark_pending.value_after(2) /= scale;
      landmark_pending.fej_after(2) /= scale;
      reset_jacobian.block<3, 3>(landmark_id, landmark_id) =
          Eigen::Vector3d(1.0, 1.0, 1.0 / scale).asDiagonal();
    } else if (representation ==
               LandmarkRepresentation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      if (landmark->value()(0) <= kScaleResetEpsilon ||
          landmark->fej()(0) <= kScaleResetEpsilon)
        return reject("single inverse-depth landmark has non-positive depth");
      landmark_pending.value_after(0) /= scale;
      landmark_pending.fej_after(0) /= scale;
      reset_jacobian(landmark_id, landmark_id) = 1.0 / scale;
    } else {
      return reject("unsupported SLAM landmark representation");
    }

    if (!landmark_pending.value_after.allFinite() ||
        !landmark_pending.fej_after.allFinite())
      return reject("landmark scale transform produced non-finite data");
    pending.push_back(std::move(landmark_pending));
  }

  if (!reset_jacobian.allFinite())
    return reject("scale-reset Jacobian contains non-finite values");

  Eigen::MatrixXd covariance_candidate =
      (reset_jacobian * state->_Cov * reset_jacobian.transpose()).eval();
  covariance_candidate =
      0.5 * (covariance_candidate + covariance_candidate.transpose());
  auto candidate_health =
      scale_reset_covariance_health(covariance_candidate);
  result.covariance_min_eigenvalue = candidate_health.min_eigenvalue;
  result.covariance_psd_projection_limit = candidate_health.psd_tolerance;
  result.covariance_psd_projection_magnitude =
      std::max(0.0, -candidate_health.min_eigenvalue);
  if (!candidate_health.valid)
    return reject("candidate " + candidate_health.reason);

  // Remove only roundoff-sized negative eigenvalues. A material negative mode
  // was rejected above, while this projection makes the committed matrix
  // numerically PSD rather than merely PSD within tolerance.
  if (candidate_health.min_eigenvalue < 0.0) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(
        covariance_candidate);
    if (eigen_solver.info() != Eigen::Success ||
        !eigen_solver.eigenvalues().allFinite() ||
        !eigen_solver.eigenvectors().allFinite())
      return reject("candidate covariance PSD stabilization failed");
    Eigen::VectorXd eigenvalues = eigen_solver.eigenvalues().cwiseMax(0.0);
    covariance_candidate =
        (eigen_solver.eigenvectors() * eigenvalues.asDiagonal() *
         eigen_solver.eigenvectors().transpose())
            .eval();
    covariance_candidate =
        0.5 * (covariance_candidate + covariance_candidate.transpose());
    result.covariance_psd_projected = true;
    candidate_health = scale_reset_covariance_health(covariance_candidate);
    if (!candidate_health.valid)
      return reject("stabilized candidate " + candidate_health.reason);
  }

  for (const auto &entry : pending) {
    if (entry.type == nullptr || !entry.value_after.allFinite() ||
        !entry.fej_after.allFinite() ||
        entry.value_after.rows() != entry.value_before.rows() ||
        entry.value_after.cols() != entry.value_before.cols() ||
        entry.fej_after.rows() != entry.fej_before.rows() ||
        entry.fej_after.cols() != entry.fej_before.cols())
      return reject("pending scale-reset state is invalid");
  }

  const Eigen::MatrixXd covariance_before = state->_Cov;
  const Eigen::Vector4d imu_quaternion_before = state->_imu->quat();
  const Eigen::Vector4d imu_quaternion_fej_before = state->_imu->quat_fej();
  const Eigen::Vector3d imu_position_before = state->_imu->pos();
  const Eigen::Vector3d imu_position_fej_before = state->_imu->pos_fej();
  const Eigen::Vector3d gyro_bias_before = state->_imu->bias_g();
  const Eigen::Vector3d gyro_bias_fej_before = state->_imu->bias_g_fej();
  const Eigen::Vector3d accel_bias_before = state->_imu->bias_a();
  const Eigen::Vector3d accel_bias_fej_before = state->_imu->bias_a_fej();
  const Eigen::MatrixXd calibration_value_before = calibration->value();
  const Eigen::MatrixXd calibration_fej_before = calibration->fej();

  auto rollback = [&]() {
    for (auto it = pending.rbegin(); it != pending.rend(); ++it) {
      it->type->set_value(it->value_before);
      it->type->set_fej(it->fej_before);
    }
    state->_Cov = covariance_before;
  };

  try {
    for (const auto &entry : pending) {
      entry.type->set_value(entry.value_after);
      entry.type->set_fej(entry.fej_after);
    }
    state->_Cov = covariance_candidate;
  } catch (const std::exception &error) {
    rollback();
    return reject(std::string("scale-reset commit threw: ") + error.what());
  } catch (...) {
    rollback();
    return reject("scale-reset commit threw an unknown exception");
  }

  bool post_state_valid = true;
  for (const auto &entry : pending) {
    post_state_valid =
        post_state_valid &&
        scale_reset_matrix_near(entry.type->value(), entry.value_after) &&
        scale_reset_matrix_near(entry.type->fej(), entry.fej_after);
  }
  post_state_valid =
      post_state_valid &&
      scale_reset_matrix_near(state->_imu->quat(), imu_quaternion_before) &&
      scale_reset_matrix_near(state->_imu->quat_fej(),
                              imu_quaternion_fej_before) &&
      scale_reset_matrix_near(state->_imu->pos(), imu_position_before) &&
      scale_reset_matrix_near(state->_imu->pos_fej(),
                              imu_position_fej_before) &&
      scale_reset_matrix_near(state->_imu->bias_g(), gyro_bias_before) &&
      scale_reset_matrix_near(state->_imu->bias_g_fej(),
                              gyro_bias_fej_before) &&
      scale_reset_matrix_near(state->_imu->bias_a(), accel_bias_before) &&
      scale_reset_matrix_near(state->_imu->bias_a_fej(),
                              accel_bias_fej_before) &&
      scale_reset_matrix_near(calibration->value(),
                              calibration_value_before) &&
      scale_reset_matrix_near(calibration->fej(), calibration_fej_before);
  const auto post_health = scale_reset_covariance_health(state->_Cov);
  if (!post_state_valid || !post_health.valid) {
    rollback();
    return reject(!post_state_valid
                      ? "post-reset state consistency check failed"
                      : "post-reset " + post_health.reason);
  }

  propagator->invalidate_cache();
  result.success = true;
  result.state_changed = true;
  result.propagator_cache_invalidated = true;
  result.covariance_min_eigenvalue = post_health.min_eigenvalue;
  result.failure_reason.clear();
  return result;
}

StateHelper::GlobalYawTranslationResetResult
StateHelper::apply_global_yaw_translation_reset(
    std::shared_ptr<State> state, double yaw_delta_rad,
    const Eigen::Vector3d &target_imu_position, Propagator *propagator) {
  GlobalYawTranslationResetResult result;
  result.yaw_delta_rad = yaw_delta_rad;
  auto reject = [&result](const std::string &reason) {
    result.failure_reason = reason;
    return result;
  };
  if (state == nullptr)
    return reject("state is null");
  if (!std::isfinite(yaw_delta_rad) || !target_imu_position.allFinite())
    return reject("yaw delta and target position must be finite");

  std::lock_guard<std::mutex> lock(state->_mutex_state);
  if (state->_imu == nullptr)
    return reject("active IMU is unavailable");
  const int covariance_size = static_cast<int>(state->_Cov.rows());
  if (covariance_size <= 0 || state->_Cov.cols() != covariance_size)
    return reject("state covariance has invalid dimensions");

  std::unordered_set<const Type *> variable_pointers;
  int expected_id = 0;
  for (const auto &variable : state->_variables) {
    if (variable == nullptr || variable->id() != expected_id ||
        variable->size() <= 0 ||
        variable->id() + variable->size() > covariance_size ||
        !variable->value().allFinite() || !variable->fej().allFinite())
      return reject("state variables are invalid or non-contiguous");
    if (!variable_pointers.insert(variable.get()).second)
      return reject("state variable list contains a duplicate entry");
    expected_id += variable->size();
  }
  if (expected_id != covariance_size ||
      variable_pointers.count(state->_imu.get()) != 1)
    return reject("state ordering does not match covariance");

  std::string consistency_reason;
  if (!scale_reset_imu_consistent(state->_imu, consistency_reason))
    return reject("active IMU is inconsistent: " + consistency_reason);
  for (const auto &clone_pair : state->_clones_IMU) {
    if (!std::isfinite(clone_pair.first) ||
        !scale_reset_pose_consistent(clone_pair.second, consistency_reason) ||
        variable_pointers.count(clone_pair.second.get()) != 1)
      return reject("IMU clone is invalid or absent from state ordering");
  }
  for (const auto &feature_pair : state->_features_SLAM) {
    const auto &landmark = feature_pair.second;
    if (landmark == nullptr ||
        variable_pointers.count(landmark.get()) != 1 ||
        !landmark->value().allFinite() || !landmark->fej().allFinite())
      return reject("SLAM landmark is invalid or absent from state ordering");
    if (LandmarkRepresentation::is_relative_representation(
            landmark->_feat_representation) &&
        (landmark->_anchor_cam_id < 0 ||
         !std::isfinite(landmark->_anchor_clone_timestamp) ||
         state->_clones_IMU.count(landmark->_anchor_clone_timestamp) != 1))
      return reject("anchored SLAM landmark has an invalid anchor");
  }

  const auto prior_health = scale_reset_covariance_health(state->_Cov);
  result.covariance_min_eigenvalue = prior_health.min_eigenvalue;
  result.covariance_psd_projection_limit = prior_health.psd_tolerance;
  if (!prior_health.valid)
    return reject("prior " + prior_health.reason);
  if (propagator == nullptr)
    return reject("gauge reset requires a propagator cache handle");

  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(yaw_delta_rad, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d translation =
      target_imu_position - rotation * state->_imu->pos();
  result.translation = translation;
  const bool identity =
      std::fabs(yaw_delta_rad) <= 1.0e-15 && translation.norm() <= 1.0e-15;
  if (identity) {
    result.success = true;
    result.failure_reason.clear();
    return result;
  }

  Eigen::MatrixXd reset_jacobian =
      Eigen::MatrixXd::Identity(covariance_size, covariance_size);
  std::vector<ScaleResetPendingValue> pending;
  pending.reserve(1 + state->_clones_IMU.size() +
                  state->_features_SLAM.size());

  auto transform_pose = [&](const Eigen::MatrixXd &value,
                            Eigen::MatrixXd &transformed) {
    if (value.rows() != 7 || value.cols() != 1)
      return false;
    transformed = value;
    const Eigen::Matrix3d R_GtoI =
        ov_core::quat_2_Rot(value.block<4, 1>(0, 0));
    transformed.block<4, 1>(0, 0) =
        ov_core::rot_2_quat(R_GtoI * rotation.transpose());
    transformed.block<3, 1>(4, 0) =
        rotation * value.block<3, 1>(4, 0) + translation;
    return transformed.allFinite();
  };

  ScaleResetPendingValue imu_pending;
  imu_pending.type = state->_imu;
  imu_pending.value_before = state->_imu->value();
  imu_pending.fej_before = state->_imu->fej();
  imu_pending.value_after = imu_pending.value_before;
  imu_pending.fej_after = imu_pending.fej_before;
  {
    Eigen::MatrixXd pose_after;
    Eigen::MatrixXd pose_fej_after;
    if (!transform_pose(imu_pending.value_before.block(0, 0, 7, 1),
                        pose_after) ||
        !transform_pose(imu_pending.fej_before.block(0, 0, 7, 1),
                        pose_fej_after))
      return reject("active IMU pose transform failed");
    imu_pending.value_after.block(0, 0, 7, 1) = pose_after;
    imu_pending.fej_after.block(0, 0, 7, 1) = pose_fej_after;
  }
  imu_pending.value_after.block<3, 1>(7, 0) =
      rotation * imu_pending.value_before.block<3, 1>(7, 0);
  imu_pending.fej_after.block<3, 1>(7, 0) =
      rotation * imu_pending.fej_before.block<3, 1>(7, 0);
  pending.push_back(std::move(imu_pending));
  reset_jacobian.block<3, 3>(state->_imu->p()->id(),
                             state->_imu->p()->id()) = rotation;
  reset_jacobian.block<3, 3>(state->_imu->v()->id(),
                             state->_imu->v()->id()) = rotation;

  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &clone = clone_pair.second;
    ScaleResetPendingValue clone_pending;
    clone_pending.type = clone;
    clone_pending.value_before = clone->value();
    clone_pending.fej_before = clone->fej();
    if (!transform_pose(clone_pending.value_before,
                        clone_pending.value_after) ||
        !transform_pose(clone_pending.fej_before,
                        clone_pending.fej_after))
      return reject("clone gauge transform failed");
    pending.push_back(std::move(clone_pending));
    reset_jacobian.block<3, 3>(clone->p()->id(), clone->p()->id()) =
        rotation;
  }

  for (const auto &feature_pair : state->_features_SLAM) {
    const auto &landmark = feature_pair.second;
    const auto representation = landmark->_feat_representation;
    if (LandmarkRepresentation::is_relative_representation(representation))
      continue;
    ScaleResetPendingValue landmark_pending;
    landmark_pending.type = landmark;
    landmark_pending.value_before = landmark->value();
    landmark_pending.fej_before = landmark->fej();
    landmark_pending.value_after = landmark_pending.value_before;
    landmark_pending.fej_after = landmark_pending.fej_before;
    if (representation == LandmarkRepresentation::GLOBAL_3D) {
      landmark_pending.value_after =
          rotation * landmark_pending.value_before + translation;
      landmark_pending.fej_after =
          rotation * landmark_pending.fej_before + translation;
      reset_jacobian.block<3, 3>(landmark->id(), landmark->id()) =
          rotation;
    } else if (representation ==
               LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH) {
      Eigen::Matrix3d representation_jacobian;
      Eigen::Vector3d value_after;
      Eigen::Vector3d fej_after;
      if (!gauge_reset_transform_global_inverse_depth(
              landmark_pending.value_before, rotation, translation,
              value_after, &representation_jacobian) ||
          !gauge_reset_transform_global_inverse_depth(
              landmark_pending.fej_before, rotation, translation, fej_after,
              nullptr))
        return reject("global inverse-depth gauge transform failed");
      landmark_pending.value_after = value_after;
      landmark_pending.fej_after = fej_after;
      reset_jacobian.block<3, 3>(landmark->id(), landmark->id()) =
          representation_jacobian;
    } else {
      return reject("unsupported global SLAM landmark representation");
    }
    pending.push_back(std::move(landmark_pending));
  }

  if (!reset_jacobian.allFinite())
    return reject("gauge-reset Jacobian contains non-finite values");
  Eigen::MatrixXd covariance_candidate =
      (reset_jacobian * state->_Cov * reset_jacobian.transpose()).eval();
  covariance_candidate =
      0.5 * (covariance_candidate + covariance_candidate.transpose());
  auto candidate_health =
      scale_reset_covariance_health(covariance_candidate);
  result.covariance_min_eigenvalue = candidate_health.min_eigenvalue;
  result.covariance_psd_projection_limit = candidate_health.psd_tolerance;
  result.covariance_psd_projection_magnitude =
      std::max(0.0, -candidate_health.min_eigenvalue);
  if (!candidate_health.valid)
    return reject("candidate " + candidate_health.reason);
  if (candidate_health.min_eigenvalue < 0.0) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(
        covariance_candidate);
    if (eigen_solver.info() != Eigen::Success ||
        !eigen_solver.eigenvalues().allFinite() ||
        !eigen_solver.eigenvectors().allFinite())
      return reject("candidate covariance PSD stabilization failed");
    covariance_candidate =
        (eigen_solver.eigenvectors() *
         eigen_solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
         eigen_solver.eigenvectors().transpose())
            .eval();
    covariance_candidate =
        0.5 * (covariance_candidate + covariance_candidate.transpose());
    result.covariance_psd_projected = true;
    candidate_health = scale_reset_covariance_health(covariance_candidate);
    if (!candidate_health.valid)
      return reject("stabilized candidate " + candidate_health.reason);
  }

  const Eigen::MatrixXd covariance_before = state->_Cov;
  auto rollback = [&]() {
    for (auto it = pending.rbegin(); it != pending.rend(); ++it) {
      it->type->set_value(it->value_before);
      it->type->set_fej(it->fej_before);
    }
    state->_Cov = covariance_before;
  };
  try {
    for (const auto &entry : pending) {
      entry.type->set_value(entry.value_after);
      entry.type->set_fej(entry.fej_after);
    }
    state->_Cov = covariance_candidate;
  } catch (const std::exception &error) {
    rollback();
    return reject(std::string("gauge-reset commit threw: ") + error.what());
  } catch (...) {
    rollback();
    return reject("gauge-reset commit threw an unknown exception");
  }

  const auto post_health = scale_reset_covariance_health(state->_Cov);
  if (!post_health.valid ||
      (state->_imu->pos() - target_imu_position).norm() > 1.0e-8 ||
      !state->_imu->bias_g().isApprox(
          pending.front().value_before.block<3, 1>(10, 0), 1.0e-12) ||
      !state->_imu->bias_a().isApprox(
          pending.front().value_before.block<3, 1>(13, 0), 1.0e-12)) {
    rollback();
    return reject(!post_health.valid
                      ? "post-reset " + post_health.reason
                      : "post-reset state invariant failed");
  }

  propagator->invalidate_cache();
  result.success = true;
  result.state_changed = true;
  result.propagator_cache_invalidated = true;
  result.covariance_min_eigenvalue = post_health.min_eigenvalue;
  result.failure_reason.clear();
  return result;
}

StateHelper::GlobalYawTranslationResetResult
StateHelper::apply_global_yaw_scale_translation_reset(
    std::shared_ptr<State> state, double scale, double yaw_delta_rad,
    const Eigen::Vector3d &target_imu_position, Propagator *propagator,
    std::size_t camera_id) {
  GlobalYawTranslationResetResult result;
  result.scale = scale;
  result.yaw_delta_rad = yaw_delta_rad;
  auto reject = [&result](const std::string &reason) {
    result.failure_reason = reason;
    return result;
  };
  if (state == nullptr || propagator == nullptr)
    return reject("similarity reset requires state and propagator");
  if (!std::isfinite(scale) || scale <= 0.0 ||
      !std::isfinite(yaw_delta_rad) || !target_imu_position.allFinite())
    return reject("similarity parameters must be finite and scale positive");

  struct SavedValue {
    std::shared_ptr<Type> type;
    Eigen::MatrixXd value;
    Eigen::MatrixXd fej;
  };
  std::vector<SavedValue> saved;
  Eigen::MatrixXd covariance_before;
  {
    std::lock_guard<std::mutex> lock(state->_mutex_state);
    saved.reserve(state->_variables.size());
    for (const auto &variable : state->_variables) {
      if (variable == nullptr)
        return reject("similarity reset found a null state variable");
      saved.push_back({variable, variable->value(), variable->fej()});
    }
    covariance_before = state->_Cov;
  }

  auto restore = [&]() {
    std::lock_guard<std::mutex> lock(state->_mutex_state);
    for (const auto &entry : saved) {
      entry.type->set_value(entry.value);
      entry.type->set_fej(entry.fej);
    }
    state->_Cov = covariance_before;
    propagator->invalidate_cache();
  };

  const Sim3ScaleResetResult scale_result =
      apply_sim3_scale_reset(state, scale, propagator, camera_id);
  if (!scale_result.success)
    return reject("metric scale stage rejected: " +
                  scale_result.failure_reason);

  const GlobalYawTranslationResetResult gauge_result =
      apply_global_yaw_translation_reset(state, yaw_delta_rad,
                                         target_imu_position, propagator);
  if (!gauge_result.success) {
    restore();
    result.propagator_cache_invalidated = true;
    return reject("yaw/translation stage rejected after rolled-back scale: " +
                  gauge_result.failure_reason);
  }

  result.success = true;
  result.state_changed = scale_result.state_changed || gauge_result.state_changed;
  result.propagator_cache_invalidated =
      scale_result.propagator_cache_invalidated ||
      gauge_result.propagator_cache_invalidated;
  result.translation = gauge_result.translation;
  result.covariance_min_eigenvalue =
      gauge_result.covariance_min_eigenvalue;
  result.covariance_psd_projection_limit =
      std::max(scale_result.covariance_psd_projection_limit,
               gauge_result.covariance_psd_projection_limit);
  result.covariance_psd_projection_magnitude =
      std::max(scale_result.covariance_psd_projection_magnitude,
               gauge_result.covariance_psd_projection_magnitude);
  result.covariance_psd_projected =
      scale_result.covariance_psd_projected ||
      gauge_result.covariance_psd_projected;
  result.failure_reason.clear();
  return result;
}

Eigen::MatrixXd StateHelper::project_visual_measurement_jacobian(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    VisualYawUpdateMode visual_yaw_update_mode,
    double visual_yaw_update_scale,
    double visual_global_yaw_oc_alpha) {
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var ? meas_var->size() : 0;
  }
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION)
    return project_global_yaw_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, false);
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION)
    return project_global_yaw_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, true);
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_CENTERED_PROJECTION)
    return project_global_yaw_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, false, true);
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION)
    return project_global_4d_from_H(state, H_order, H_id, H,
                                    visual_global_yaw_oc_alpha, false);
  if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE)
    return project_per_block_yaw_from_H(state, H_order, H_id, H,
                                        visual_yaw_update_scale);
  return H;
}

void StateHelper::reset_last_yaw_dx_projection_diag() {
  last_yaw_dx_projection_diag = YawDxProjectionDiag();
}

StateHelper::YawDxProjectionDiag StateHelper::get_last_yaw_dx_projection_diag() {
  return last_yaw_dx_projection_diag;
}

void StateHelper::EKFPropagation(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &order_NEW,
                                 const std::vector<std::shared_ptr<Type>> &order_OLD, const Eigen::MatrixXd &Phi,
                                 const Eigen::MatrixXd &Q) {

  // We need at least one old and new variable
  if (order_NEW.empty() || order_OLD.empty()) {
    PRINT_ERROR(RED "StateHelper::EKFPropagation() - Called with empty variable arrays!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Loop through our Phi order and ensure that they are continuous in memory
  int size_order_NEW = order_NEW.at(0)->size();
  for (size_t i = 0; i < order_NEW.size() - 1; i++) {
    if (order_NEW.at(i)->id() + order_NEW.at(i)->size() != order_NEW.at(i + 1)->id()) {
      PRINT_ERROR(RED "StateHelper::EKFPropagation() - Called with non-contiguous state elements!\n" RESET);
      PRINT_ERROR(
          RED "StateHelper::EKFPropagation() - This code only support a state transition which is in the same order as the state\n" RESET);
      std::exit(EXIT_FAILURE);
    }
    size_order_NEW += order_NEW.at(i + 1)->size();
  }

  // Size of the old phi matrix
  int size_order_OLD = order_OLD.at(0)->size();
  for (size_t i = 0; i < order_OLD.size() - 1; i++) {
    size_order_OLD += order_OLD.at(i + 1)->size();
  }

  // Assert that we have correct sizes
  assert(size_order_NEW == Phi.rows());
  assert(size_order_OLD == Phi.cols());
  assert(size_order_NEW == Q.cols());
  assert(size_order_NEW == Q.rows());

  // Get the location in small phi for each measuring variable
  int current_it = 0;
  std::vector<int> Phi_id;
  for (const auto &var : order_OLD) {
    Phi_id.push_back(current_it);
    current_it += var->size();
  }

  // Loop through all our old states and get the state transition times it
  // Cov_PhiT = [ Pxx ] [ Phi' ]'
  Eigen::MatrixXd Cov_PhiT = Eigen::MatrixXd::Zero(state->_Cov.rows(), Phi.rows());
  for (size_t i = 0; i < order_OLD.size(); i++) {
    std::shared_ptr<Type> var = order_OLD.at(i);
    Cov_PhiT.noalias() +=
        state->_Cov.block(0, var->id(), state->_Cov.rows(), var->size()) * Phi.block(0, Phi_id[i], Phi.rows(), var->size()).transpose();
  }

  // Get Phi_NEW*Covariance*Phi_NEW^t + Q
  Eigen::MatrixXd Phi_Cov_PhiT = Q.selfadjointView<Eigen::Upper>();
  for (size_t i = 0; i < order_OLD.size(); i++) {
    std::shared_ptr<Type> var = order_OLD.at(i);
    Phi_Cov_PhiT.noalias() += Phi.block(0, Phi_id[i], Phi.rows(), var->size()) * Cov_PhiT.block(var->id(), 0, var->size(), Phi.rows());
  }

  // We are good to go!
  int start_id = order_NEW.at(0)->id();
  int phi_size = Phi.rows();
  int total_size = state->_Cov.rows();
  state->_Cov.block(start_id, 0, phi_size, total_size) = Cov_PhiT.transpose();
  state->_Cov.block(0, start_id, total_size, phi_size) = Cov_PhiT;
  state->_Cov.block(start_id, start_id, phi_size, phi_size) = Phi_Cov_PhiT;

  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  Eigen::VectorXd diags = state->_Cov.diagonal();
  bool found_neg = false;
  for (int i = 0; i < diags.rows(); i++) {
    if (diags(i) < 0.0) {
      PRINT_WARNING(RED "StateHelper::EKFPropagation() - diagonal at %d is %.2f\n" RESET, i, diags(i));
      found_neg = true;
    }
  }
  if (found_neg) {
    std::exit(EXIT_FAILURE);
  }
}

void StateHelper::EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order, const Eigen::MatrixXd &H,
                            const Eigen::VectorXd &res, const Eigen::MatrixXd &R, VisualYawUpdateMode visual_yaw_update_mode,
                            double visual_yaw_update_scale, double visual_global_yaw_oc_alpha,
                            double visual_bgz_update_scale, double visual_imu_yaw_gain_scale) {

  //==========================================================
  //==========================================================
  // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());
  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }
  Eigen::MatrixXd H_eff = H;
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, /*use_fej=*/false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, /*use_fej=*/true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_YAW_OC_CENTERED_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H,
                                      visual_global_yaw_oc_alpha,
                                      /*use_fej=*/false,
                                      /*center_positions=*/true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION) {
    H_eff = project_global_4d_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale);
  }

  //==========================================================
  //==========================================================
  // For each active variable find its M = P*H^T
  for (const auto &var : state->_variables) {
    // Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order[i];
      M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                       H_eff.block(0, H_id[i], H_eff.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  //==========================================================
  //==========================================================
  // Get covariance of the involved terms
  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

  // Residual covariance S = H*Cov*H' + R
  Eigen::MatrixXd S(R.rows(), R.rows());
  S.triangularView<Eigen::Upper>() = H_eff * P_small * H_eff.transpose();
  S.triangularView<Eigen::Upper>() += R;
  // Eigen::MatrixXd S = H * P_small * H.transpose() + R;

  if (!H_eff.allFinite() || !M_a.allFinite() || !P_small.allFinite() ||
      !R.allFinite() || !res.allFinite() || !state->_Cov.allFinite()) {
    PRINT_WARNING(RED "StateHelper::EKFUpdate() - rejected non-finite update input\n" RESET);
    return;
  }

  // Invert S.
  Eigen::MatrixXd K(M_a.rows(), R.rows());
  {
    Eigen::MatrixXd S_full = S.selfadjointView<Eigen::Upper>();
    Eigen::LLT<Eigen::MatrixXd> llt(S_full);
    if (llt.info() != Eigen::Success) {
      PRINT_WARNING(RED "StateHelper::EKFUpdate() - rejected update with non-SPD residual covariance\n" RESET);
      return;
    }
    Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
    llt.solveInPlace(Sinv);
    K = M_a * Sinv.selfadjointView<Eigen::Upper>();
  }

  // Save the unmodified gain so diagnostics can distinguish the update requested
  // by the visual residual from the diagnostic gain intervention below.
  const Eigen::MatrixXd K_before_gain_scaling = K;

  // Scale bg_z row of K for visual bg_z ablation (default scale=1.0 = no change)
  bool bgz_row_scaled = false;
  if (visual_bgz_update_scale < 1.0 - 1e-12 &&
      state->_imu != nullptr && state->_imu->bg() != nullptr) {
    int bgz_row = state->_imu->bg()->id() + 2;
    if (bgz_row >= 0 && bgz_row < (int)K.rows()) {
      K.row(bgz_row) *= visual_bgz_update_scale;
      bgz_row_scaled = true;
    }
  }

  // Diagnostic-only suppression of the current IMU global-heading correction.
  // Projecting H orientation columns is insufficient because P cross-covariance
  // can still put yaw into the current IMU gain. Project the current IMU
  // orientation rows of K directly, while leaving roll/pitch and all non-yaw
  // state corrections available to the measurement.
  bool imu_yaw_gain_scaled = false;
  visual_imu_yaw_gain_scale =
      std::max(0.0, std::min(1.0, visual_imu_yaw_gain_scale));
  if (visual_imu_yaw_gain_scale < 1.0 - 1e-12 && state->_imu != nullptr &&
      state->_imu->q() != nullptr) {
    const int q_row = state->_imu->q()->id();
    if (q_row >= 0 && q_row + 3 <= K.rows()) {
      const Eigen::Matrix3d Cq =
          yaw_projection_matrix(state->_imu->Rot(), visual_imu_yaw_gain_scale);
      K.block(q_row, 0, 3, K.cols()) =
          (Cq * K.block(q_row, 0, 3, K.cols())).eval();
      imu_yaw_gain_scaled = true;
    }
  }

  Eigen::MatrixXd P_candidate = state->_Cov;
  if (bgz_row_scaled || imu_yaw_gain_scaled) {
    // K no longer equals M_a S^{-1}, so the standard form P -= K M_a^T is invalid
    // (non-symmetric, goes non-PSD).  Use the gain-agnostic consistent form
    //   P+ = P - K' M_a^T - M_a K'^T + K' S K'^T
    // which reduces to the standard form when K' = K, and keeps bg_z a proper
    // "considered" state: its variance is not reduced by the visual update.
    Eigen::MatrixXd KM = K * M_a.transpose();
    Eigen::MatrixXd Sfull = S.selfadjointView<Eigen::Upper>();
    Eigen::MatrixXd KSK = K * Sfull * K.transpose();
    P_candidate += KSK - KM - KM.transpose();
    P_candidate = (0.5 * (P_candidate + P_candidate.transpose())).eval();
  } else {
    // Covariance update (standard for all modes; K is unmodified so K = M_a S^{-1} holds).
    P_candidate.triangularView<Eigen::Upper>() -= K * M_a.transpose();
    P_candidate = P_candidate.selfadjointView<Eigen::Upper>();
  }

  // Treat the covariance update as a transaction. A numerically unhealthy
  // visual update is skipped instead of killing the whole estimator.
  Eigen::VectorXd diags = P_candidate.diagonal();
  bool found_bad = !P_candidate.allFinite();
  const double diag_scale = diags.rows() > 0 ? std::max(1.0, diags.cwiseAbs().maxCoeff()) : 1.0;
  const double diag_tol = 1e-12 * diag_scale;
  for (int i = 0; i < diags.rows(); i++) {
    if (!std::isfinite(diags(i)) || diags(i) < -diag_tol) {
      PRINT_WARNING(RED "StateHelper::EKFUpdate() - diagonal at %d is %.2f\n" RESET, i, diags(i));
      found_bad = true;
    }
  }
  if (found_bad) {
    return;
  }
  for (int i = 0; i < diags.rows(); i++) {
    if (P_candidate(i, i) < 0.0) {
      P_candidate(i, i) = 0.0;
    }
  }
  state->_Cov = P_candidate;

  // Calculate our delta and update all our active states
  const Eigen::VectorXd dx_before_gain_scaling = K_before_gain_scaling * res;
  Eigen::VectorXd dx = K * res;
  last_yaw_dx_projection_diag.valid = true;
  last_yaw_dx_projection_diag.mode = visual_yaw_update_mode;
  last_yaw_dx_projection_diag.dx_yaw_before_projection_deg =
      yaw_dx_component_deg(state, dx_before_gain_scaling);
  last_yaw_dx_projection_diag.dx_yaw_after_projection_deg = yaw_dx_component_deg(state, dx);

  for (size_t i = 0; i < state->_variables.size(); i++) {
    state->_variables.at(i)->update(dx.block(state->_variables.at(i)->id(), 0, state->_variables.at(i)->size(), 1));
  }

  // If we are doing online intrinsic calibration we should update our camera objects
  // NOTE: is this the best place to put this update logic??? probably..
  if (state->_options.do_calib_camera_intrinsics) {
    for (auto const &calib : state->_cam_intrinsics) {
      state->_cam_intrinsics_cameras.at(calib.first)->set_value(calib.second->value());
    }
  }
}

StateHelper::UpdateDiagnostics StateHelper::compute_update_diagnostics(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &res,
    const Eigen::MatrixXd &R,
    VisualYawUpdateMode visual_yaw_update_mode,
    double visual_yaw_update_scale,
    double visual_global_yaw_oc_alpha,
    double visual_bgz_update_scale) {

  UpdateDiagnostics out;
  out.rows = (int)H.rows();
  out.cols = (int)H.cols();
  if (state == nullptr || res.rows() != R.rows() || H.rows() != res.rows())
    return out;

  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var ? meas_var->size() : 0;
  }

  Eigen::MatrixXd H_eff = H;
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_YAW_OC_CENTERED_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H,
                                      visual_global_yaw_oc_alpha, false, true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION) {
    H_eff = project_global_4d_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale);
  }

  out.valid = true;
  out.residual_norm = res.norm();
  out.H_norm = H_eff.norm();
  double H_white_sq = 0.0;
  for (int r = 0; r < R.rows() && r < H_eff.rows(); r++) {
    const double var = R(r, r);
    if (std::isfinite(var) && var > 1e-18)
      H_white_sq += H_eff.row(r).squaredNorm() / var;
  }
  out.whitened_H_norm = std::sqrt(std::max(0.0, H_white_sq));

  double H_yaw_sq = 0.0;
  double H_bgz_sq = 0.0;
  double H_pos_sq = 0.0;
  double H_landmark_sq = 0.0;
  double H_other_sq = 0.0;
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    if (!var)
      continue;
    const int col = H_id[i];
    const int sz = var->size();
    if (col < 0 || col + sz > H_eff.cols())
      continue;
    const bool is_landmark = (std::dynamic_pointer_cast<Landmark>(var) != nullptr);
    const bool is_bg = (state->_imu != nullptr && state->_imu->bg() != nullptr && var == state->_imu->bg());
    if (is_landmark) {
      H_landmark_sq += H_eff.block(0, col, H_eff.rows(), sz).squaredNorm();
    } else if (is_bg && sz >= 3) {
      H_bgz_sq += H_eff.col(col + 2).squaredNorm();
      H_other_sq += H_eff.block(0, col, H_eff.rows(), sz).squaredNorm();
    } else if (sz >= 6) {
      H_yaw_sq += H_eff.col(col + 2).squaredNorm();
      H_pos_sq += H_eff.block(0, col + 3, H_eff.rows(), 3).squaredNorm();
    } else {
      H_other_sq += H_eff.block(0, col, H_eff.rows(), sz).squaredNorm();
    }
  }
  out.H_yaw_col_norm = std::sqrt(std::max(0.0, H_yaw_sq));
  out.H_bgz_col_norm = std::sqrt(std::max(0.0, H_bgz_sq));
  out.H_pos_col_norm = std::sqrt(std::max(0.0, H_pos_sq));
  out.H_landmark_norm = std::sqrt(std::max(0.0, H_landmark_sq));
  out.H_other_norm = std::sqrt(std::max(0.0, H_other_sq));

  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());
  for (const auto &var : state->_variables) {
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order[i];
      M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                       H_eff.block(0, H_id[i], H_eff.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd HPH(R.rows(), R.rows());
  HPH.triangularView<Eigen::Upper>() = H_eff * P_small * H_eff.transpose();
  Eigen::MatrixXd S = HPH.selfadjointView<Eigen::Upper>();
  S += R;
  out.HPH_trace = HPH.diagonal().sum();
  out.R_trace = R.trace();
  out.HPH_over_R = (std::fabs(out.R_trace) > 1e-18) ? out.HPH_trace / out.R_trace
                                                    : std::numeric_limits<double>::quiet_NaN();

  Eigen::MatrixXd S_full = S.selfadjointView<Eigen::Upper>();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(S_full);
  if (es.info() == Eigen::Success && es.eigenvalues().rows() > 0) {
    out.S_min_eig = es.eigenvalues().minCoeff();
    out.S_max_eig = es.eigenvalues().maxCoeff();
    out.S_cond = (out.S_min_eig > 1e-18) ? out.S_max_eig / out.S_min_eig
                                         : std::numeric_limits<double>::infinity();
  }

  Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
  S_full.llt().solveInPlace(Sinv);
  Eigen::MatrixXd K = M_a * Sinv.selfadjointView<Eigen::Upper>();
  if (visual_bgz_update_scale < 1.0 - 1e-12 &&
      state->_imu != nullptr && state->_imu->bg() != nullptr) {
    const int bgz_row = state->_imu->bg()->id() + 2;
    if (bgz_row >= 0 && bgz_row < (int)K.rows())
      K.row(bgz_row) *= visual_bgz_update_scale;
  }

  Eigen::VectorXd dx = K * res;

  out.dx_norm = dx.norm();
  out.dx_yaw_deg = yaw_dx_component_deg(state, dx);

  const auto &q_var = (state->_imu != nullptr) ? state->_imu->q() : nullptr;
  const auto &p_var = (state->_imu != nullptr) ? state->_imu->p() : nullptr;
  const auto &bg_var = (state->_imu != nullptr) ? state->_imu->bg() : nullptr;
  constexpr double rad_to_deg = 180.0 / 3.14159265358979323846;
  Eigen::Vector3d g_local = Eigen::Vector3d::UnitZ();
  if (state->_imu != nullptr) {
    g_local = state->_imu->Rot() * Eigen::Vector3d::UnitZ();
    const double gn = g_local.norm();
    if (gn > 1e-12)
      g_local /= gn;
  }

  if (q_var != nullptr && q_var->id() >= 0 && q_var->id() + 3 <= K.rows()) {
    Eigen::RowVectorXd K_yaw = g_local.transpose() * K.block(q_var->id(), 0, 3, K.cols());
    out.K_yaw_row_norm = K_yaw.norm() * rad_to_deg;
    Eigen::Matrix3d Pqq = state->_Cov.block(q_var->id(), q_var->id(), 3, 3);
    out.P_yaw_var = g_local.dot(Pqq * g_local);
  }
  if (bg_var != nullptr && bg_var->id() >= 0 && bg_var->id() + 3 <= K.rows()) {
    const int bgz = bg_var->id() + 2;
    out.dx_bgz = dx(bgz);
    out.K_bgz_row_norm = K.row(bgz).norm();
    out.P_bgz_var = state->_Cov(bgz, bgz);
    if (q_var != nullptr && q_var->id() >= 0 && q_var->id() + 3 <= state->_Cov.rows()) {
      const double cov = (g_local.transpose() * state->_Cov.block(q_var->id(), bgz, 3, 1))(0, 0);
      const double den = std::sqrt(std::max(0.0, out.P_yaw_var * out.P_bgz_var));
      out.corr_yaw_bgz = den > 1e-18 ? cov / den : std::numeric_limits<double>::quiet_NaN();
    }
  }
  if (p_var != nullptr && p_var->id() >= 0 && p_var->id() + 3 <= K.rows()) {
    out.dx_pos_norm = dx.segment(p_var->id(), 3).norm();
    out.K_pos_row_norm = K.block(p_var->id(), 0, 3, K.cols()).norm();
    out.P_pos_trace = state->_Cov.block(p_var->id(), p_var->id(), 3, 3).trace();
    if (q_var != nullptr && q_var->id() >= 0 && q_var->id() + 3 <= state->_Cov.rows()) {
      const double den_x = std::sqrt(std::max(0.0, out.P_yaw_var * state->_Cov(p_var->id() + 0, p_var->id() + 0)));
      const double den_y = std::sqrt(std::max(0.0, out.P_yaw_var * state->_Cov(p_var->id() + 1, p_var->id() + 1)));
      const double cov_x = (g_local.transpose() * state->_Cov.block(q_var->id(), p_var->id() + 0, 3, 1))(0, 0);
      const double cov_y = (g_local.transpose() * state->_Cov.block(q_var->id(), p_var->id() + 1, 3, 1))(0, 0);
      out.corr_yaw_px = den_x > 1e-18 ? cov_x / den_x : std::numeric_limits<double>::quiet_NaN();
      out.corr_yaw_py = den_y > 1e-18 ? cov_y / den_y : std::numeric_limits<double>::quiet_NaN();
    }
  }

  double lm_dx_sq = 0.0;
  double lm_K_sq = 0.0;
  double pose_lm_cov_sq = 0.0;
  for (const auto &var : H_order) {
    auto lm = std::dynamic_pointer_cast<Landmark>(var);
    if (!lm || lm->id() < 0 || lm->id() + lm->size() > K.rows())
      continue;
    lm_dx_sq += dx.segment(lm->id(), lm->size()).squaredNorm();
    lm_K_sq += K.block(lm->id(), 0, lm->size(), K.cols()).squaredNorm();
    if (q_var != nullptr && q_var->id() >= 0 && q_var->id() + 3 <= state->_Cov.rows())
      pose_lm_cov_sq += state->_Cov.block(q_var->id(), lm->id(), 3, lm->size()).squaredNorm();
    if (p_var != nullptr && p_var->id() >= 0 && p_var->id() + 3 <= state->_Cov.rows())
      pose_lm_cov_sq += state->_Cov.block(p_var->id(), lm->id(), 3, lm->size()).squaredNorm();
  }
  out.dx_landmark_norm = std::sqrt(std::max(0.0, lm_dx_sq));
  out.K_landmark_row_norm = std::sqrt(std::max(0.0, lm_K_sq));
  out.pose_landmark_cov_norm = std::sqrt(std::max(0.0, pose_lm_cov_sq));
  return out;
}

Eigen::VectorXd StateHelper::compute_update_dx(std::shared_ptr<State> state,
                                               const std::vector<std::shared_ptr<Type>> &H_order,
                                               const Eigen::MatrixXd &H,
                                               const Eigen::VectorXd &res,
                                               const Eigen::MatrixXd &R,
                                               VisualYawUpdateMode visual_yaw_update_mode,
                                               double visual_yaw_update_scale,
                                               double visual_global_yaw_oc_alpha,
                                               double visual_bgz_update_scale) {
  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());

  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }

  Eigen::MatrixXd H_eff = H;
  if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, visual_global_yaw_oc_alpha, true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_YAW_OC_CENTERED_PROJECTION) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H,
                                      visual_global_yaw_oc_alpha, false, true);
  } else if (visual_yaw_update_mode ==
             VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION) {
    H_eff = project_global_4d_from_H(state, H_order, H_id, H,
                                     visual_global_yaw_oc_alpha, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale);
  }

  for (const auto &var : state->_variables) {
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order[i];
      M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                       H_eff.block(0, H_id[i], H_eff.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S(R.rows(), R.rows());
  S.triangularView<Eigen::Upper>() = H_eff * P_small * H_eff.transpose();
  S.triangularView<Eigen::Upper>() += R;
  Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
  S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);
  Eigen::MatrixXd K = M_a * Sinv.selfadjointView<Eigen::Upper>();

  if (visual_bgz_update_scale < 1.0 - 1e-12 &&
      state->_imu != nullptr && state->_imu->bg() != nullptr) {
    int bgz_row = state->_imu->bg()->id() + 2;
    if (bgz_row >= 0 && bgz_row < (int)K.rows())
      K.row(bgz_row) *= visual_bgz_update_scale;
  }

  Eigen::VectorXd dx = K * res;
  return dx;
}

double StateHelper::yaw_delta_from_full_dx_deg(std::shared_ptr<State> state, const Eigen::VectorXd &dx) {
  return yaw_dx_component_deg(state, dx);
}

void StateHelper::set_initial_covariance(std::shared_ptr<State> state, const Eigen::MatrixXd &covariance,
                                         const std::vector<std::shared_ptr<ov_type::Type>> &order) {

  // We need to loop through each element and overwrite the current covariance values
  // For example consider the following:
  // x = [ ori pos ] -> insert into -> x = [ ori bias pos ]
  // P = [ P_oo P_op ] -> P = [ P_oo  0   P_op ]
  //     [ P_po P_pp ]        [  0    P*    0  ]
  //                          [ P_po  0   P_pp ]
  // The key assumption here is that the covariance is block diagonal (cross-terms zero with P* can be dense)
  // This is normally the care on startup (for example between calibration and the initial state

  // For each variable, lets copy over all other variable cross terms
  // Note: this copies over itself to when i_index=k_index
  int i_index = 0;
  for (size_t i = 0; i < order.size(); i++) {
    int k_index = 0;
    for (size_t k = 0; k < order.size(); k++) {
      state->_Cov.block(order[i]->id(), order[k]->id(), order[i]->size(), order[k]->size()) =
          covariance.block(i_index, k_index, order[i]->size(), order[k]->size());
      k_index += order[k]->size();
    }
    i_index += order[i]->size();
  }
  state->_Cov = state->_Cov.selfadjointView<Eigen::Upper>();
}

bool StateHelper::inflate_initial_imu_subspace_covariance(
    Eigen::MatrixXd &joint_covariance, double orientation_inflation,
    double velocity_inflation, double gyro_bias_inflation,
    double accel_bias_inflation, std::string *failure_reason) {
  const auto fail = [&](const std::string &reason) {
    if (failure_reason != nullptr)
      *failure_reason = reason;
    return false;
  };
  if (joint_covariance.rows() < 15 ||
      joint_covariance.cols() != joint_covariance.rows() ||
      !joint_covariance.allFinite())
    return fail("joint_covariance_dimension_or_finiteness");
  const std::array<double, 4> inflation = {
      orientation_inflation, velocity_inflation, gyro_bias_inflation,
      accel_bias_inflation};
  for (double value : inflation) {
    if (!std::isfinite(value) || value <= 0.0)
      return fail("covariance_inflation_not_positive_finite");
  }

  const Eigen::MatrixXd symmetric =
      0.5 * (joint_covariance + joint_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> before_eigen(symmetric);
  if (before_eigen.info() != Eigen::Success ||
      before_eigen.eigenvalues().minCoeff() <= 0.0)
    return fail("joint_covariance_not_positive_definite_before_inflation");

  joint_covariance = symmetric;
  joint_covariance.block<3, 3>(0, 0) *= orientation_inflation;
  joint_covariance.block<3, 3>(6, 6) *= velocity_inflation;
  joint_covariance.block<3, 3>(9, 9) *= gyro_bias_inflation;
  joint_covariance.block<3, 3>(12, 12) *= accel_bias_inflation;
  joint_covariance =
      0.5 * (joint_covariance + joint_covariance.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> after_eigen(
      joint_covariance);
  if (after_eigen.info() != Eigen::Success ||
      after_eigen.eigenvalues().minCoeff() <= 0.0)
    return fail("joint_covariance_not_positive_definite_after_inflation");
  if (failure_reason != nullptr)
    failure_reason->clear();
  return true;
}

bool StateHelper::initialize_with_pose_history(
    std::shared_ptr<State> state, double active_timestamp,
    const Eigen::Matrix<double, 16, 1> &imu_state,
    const std::vector<double> &clone_timestamps,
    const std::vector<Eigen::Matrix<double, 7, 1>> &clone_states,
    const std::vector<size_t> &landmark_feature_ids,
    const std::vector<Eigen::Vector3d> &landmark_positions,
    const Eigen::MatrixXd &joint_covariance, std::string *failure_reason) {
  const auto fail = [&](const std::string &reason) {
    if (failure_reason != nullptr)
      *failure_reason = reason;
    return false;
  };
  if (state == nullptr || state->_imu == nullptr)
    return fail("missing_state_or_imu");
  if (!std::isfinite(active_timestamp) || !imu_state.allFinite())
    return fail("nonfinite_active_state");
  if (std::fabs(imu_state.head<4>().norm() - 1.0) > 1.0e-6)
    return fail("active_quaternion_not_unit");
  if (clone_timestamps.size() != clone_states.size())
    return fail("clone_timestamp_state_size_mismatch");
  if (landmark_feature_ids.size() != landmark_positions.size())
    return fail("landmark_id_state_size_mismatch");
  if (clone_timestamps.size() >
      static_cast<size_t>(std::max(0, state->_options.max_clone_size)))
    return fail("clone_capacity_exceeded");
  if (landmark_feature_ids.size() >
      static_cast<size_t>(std::max(0, state->_options.max_slam_features)))
    return fail("landmark_capacity_exceeded");
  if (!state->_clones_IMU.empty() || !state->_features_SLAM.empty())
    return fail("state_history_not_empty");
  double previous_timestamp = -std::numeric_limits<double>::infinity();
  for (size_t index = 0; index < clone_timestamps.size(); ++index) {
    if (!std::isfinite(clone_timestamps[index]) ||
        clone_timestamps[index] <= previous_timestamp + 1.0e-9 ||
        clone_timestamps[index] >= active_timestamp - 1.0e-9)
      return fail("clone_timestamps_not_strictly_historical");
    if (!clone_states[index].allFinite() ||
        std::fabs(clone_states[index].head<4>().norm() - 1.0) > 1.0e-6)
      return fail("invalid_clone_state");
    previous_timestamp = clone_timestamps[index];
  }
  std::unordered_set<size_t> unique_landmark_ids;
  for (size_t index = 0; index < landmark_feature_ids.size(); ++index) {
    if (!landmark_positions[index].allFinite())
      return fail("invalid_landmark_state");
    if (!unique_landmark_ids.insert(landmark_feature_ids[index]).second)
      return fail("duplicate_landmark_feature_id");
  }
  const Eigen::Index expected_covariance_dimension =
      15 + 6 * static_cast<Eigen::Index>(clone_states.size()) +
      3 * static_cast<Eigen::Index>(landmark_positions.size());
  if (joint_covariance.rows() != expected_covariance_dimension ||
      joint_covariance.cols() != expected_covariance_dimension ||
      !joint_covariance.allFinite())
    return fail("joint_covariance_dimension_or_finiteness");
  const Eigen::MatrixXd symmetric_covariance =
      0.5 * (joint_covariance + joint_covariance.transpose());
  const double covariance_scale =
      std::max(1.0, symmetric_covariance.cwiseAbs().maxCoeff());
  if ((joint_covariance - joint_covariance.transpose()).norm() >
      1.0e-9 * covariance_scale)
    return fail("joint_covariance_not_symmetric");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covariance_eigen(
      symmetric_covariance);
  if (covariance_eigen.info() != Eigen::Success ||
      covariance_eigen.eigenvalues().minCoeff() <= 0.0)
    return fail("joint_covariance_not_positive_definite");

  const Eigen::Index old_covariance_dimension = state->_Cov.rows();
  const Eigen::Index new_covariance_dimension =
      old_covariance_dimension +
      6 * static_cast<Eigen::Index>(clone_states.size()) +
      3 * static_cast<Eigen::Index>(landmark_positions.size());
  Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Zero(
      new_covariance_dimension, new_covariance_dimension);
  new_covariance.topLeftCorner(old_covariance_dimension,
                               old_covariance_dimension) = state->_Cov;
  const Eigen::Index imu_offset = state->_imu->id();
  if (imu_offset < 0 || imu_offset + state->_imu->size() >
                            old_covariance_dimension)
    return fail("active_imu_covariance_index_invalid");
  new_covariance.block(imu_offset, 0, state->_imu->size(),
                       new_covariance_dimension)
      .setZero();
  new_covariance.block(0, imu_offset, new_covariance_dimension,
                       state->_imu->size())
      .setZero();

  std::vector<std::shared_ptr<ov_type::Type>> new_variables =
      state->_variables;
  std::map<double, std::shared_ptr<ov_type::PoseJPL>> new_clones;
  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>
      new_landmarks;
  std::vector<std::shared_ptr<ov_type::Type>> covariance_order;
  covariance_order.reserve(1 + clone_states.size() + landmark_positions.size());
  covariance_order.push_back(state->_imu);
  for (size_t index = 0; index < clone_states.size(); ++index) {
    auto clone = std::make_shared<ov_type::PoseJPL>();
    clone->set_value(clone_states[index]);
    clone->set_fej(clone_states[index]);
    clone->set_local_id(static_cast<int>(old_covariance_dimension + 6 * index));
    new_variables.push_back(clone);
    new_clones.emplace(clone_timestamps[index], clone);
    covariance_order.push_back(clone);
  }
  const Eigen::Index landmark_covariance_offset =
      old_covariance_dimension +
      6 * static_cast<Eigen::Index>(clone_states.size());
  for (size_t index = 0; index < landmark_positions.size(); ++index) {
    auto landmark = std::make_shared<ov_type::Landmark>(3);
    landmark->_featid = landmark_feature_ids[index];
    landmark->_unique_camera_id = 0;
    landmark->_feat_representation =
        ov_type::LandmarkRepresentation::Representation::GLOBAL_3D;
    landmark->set_from_xyz(landmark_positions[index], false);
    landmark->set_from_xyz(landmark_positions[index], true);
    landmark->set_local_id(
        static_cast<int>(landmark_covariance_offset + 3 * index));
    new_variables.push_back(landmark);
    new_landmarks.emplace(landmark_feature_ids[index], landmark);
    covariance_order.push_back(landmark);
  }

  Eigen::Index source_row = 0;
  for (const auto &row_variable : covariance_order) {
    Eigen::Index source_col = 0;
    for (const auto &col_variable : covariance_order) {
      new_covariance.block(row_variable->id(), col_variable->id(),
                           row_variable->size(), col_variable->size()) =
          symmetric_covariance.block(source_row, source_col,
                                     row_variable->size(),
                                     col_variable->size());
      source_col += col_variable->size();
    }
    source_row += row_variable->size();
  }
  new_covariance =
      0.5 * (new_covariance + new_covariance.transpose());

  state->_imu->set_value(imu_state);
  state->_imu->set_fej(imu_state);
  state->_variables = std::move(new_variables);
  state->_clones_IMU = std::move(new_clones);
  state->_features_SLAM = std::move(new_landmarks);
  state->_Cov = std::move(new_covariance);
  state->_timestamp = active_timestamp;
  if (failure_reason != nullptr)
    failure_reason->clear();
  return true;
}

void StateHelper::inject_pz_noise(std::shared_ptr<State> state, double noise) {
  int global_pz = state->_imu->id() + 5; // q(3) + p_z(idx=2)
  state->_Cov(global_pz, global_pz) += noise;
}

// =============================================================================
// josephCovUpdate — pure-math Joseph-form covariance update
// Ported from PX4-Autopilot ekf_helper.cpp:measurementUpdate lines 1127-1171
// Commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909, BSD-3 License
// Copyright (c) 2012-2025 PX4 Development Team
// =============================================================================
void StateHelper::josephCovUpdate(Eigen::MatrixXd &P,
                                   const Eigen::VectorXd &K,
                                   const Eigen::VectorXd &H,
                                   double R) {
  const int N = (int)P.rows();
  assert(P.cols() == N);
  assert((int)K.rows() == N);
  assert((int)H.rows() == N);

  // Step 1 (PX4 "conventional update"): P = (I - K * H^T) * P
  //   PH = P * H  (N×1)
  //   P(i,j) -= K(i) * PH(j)  for all i,j
  // Matrix form: P -= K * (P*H)^T = K * H^T * P
  // For masked K entries (K(i)=0), that row of P is unchanged in this step.
  Eigen::VectorXd PH = P * H;
  P.noalias() -= K * PH.transpose();

  // Step 2 (PX4 "stabilized update"): enforces Joseph form and symmetry
  //   PH = P_step1 * H  (recomputed)
  //   P(i,j) = P(i,j) - PH(i)*K(j) + K(i)*R*K(j)  for j<=i
  //   P(j,i) = P(i,j)
  // Net result: P = P_step1*(I - H*K^T) + R*K*K^T
  //           = (I-K*H^T)*P*(I-K*H^T)^T + R*K*K^T  (Joseph form)
  PH = P * H;
  for (int i = 0; i < N; i++) {
    for (int j = 0; j <= i; j++) {
      const double v = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
      P(i, j) = v;
      P(j, i) = v;
    }
  }
}

// =============================================================================
// EKFUpdateJoseph — State-aware Joseph update with pre-computed masked gain
// Ported from PX4-Autopilot ekf_helper.cpp:measurementUpdate + fuseHaglRng
// Commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909, BSD-3 License
// Copyright (c) 2012-2025 PX4 Development Team
// =============================================================================
void StateHelper::EKFUpdateJoseph(std::shared_ptr<State> state,
                                   const Eigen::VectorXd &K_full,
                                   const Eigen::VectorXd &H_full,
                                   double R, double res) {
  JosephUpdateHealth health;
  if (!EKFUpdateJosephChecked(state, K_full, H_full, R, res, false, &health)) {
    PRINT_ERROR(RED "StateHelper::EKFUpdateJoseph() - rejected numerically invalid Joseph update "
                    "(finite=%d sym=%.3e min_diag=%.3e)\n" RESET,
                health.finite ? 1 : 0, health.symmetry_error, health.min_diagonal);
  }
}

bool StateHelper::EKFUpdateJosephChecked(std::shared_ptr<State> state,
                                          const Eigen::VectorXd &K_full,
                                          const Eigen::VectorXd &H_full,
                                          double R, double res,
                                          bool check_psd,
                                          JosephUpdateHealth *health) {
  const int N = (int)state->_Cov.rows();
  assert((int)K_full.rows() == N);
  assert((int)H_full.rows() == N);

  JosephUpdateHealth out;
  out.psd_checked = check_psd;
  if (!std::isfinite(R) || R <= 0.0 || !std::isfinite(res) ||
      !K_full.allFinite() || !H_full.allFinite() || !state->_Cov.allFinite()) {
    if (health) *health = out;
    return false;
  }

  // Form the candidate transactionally: a failed health check leaves both P
  // and the nominal state untouched.
  Eigen::MatrixXd P_candidate = state->_Cov;
  josephCovUpdate(P_candidate, K_full, H_full, R);
  P_candidate = 0.5 * (P_candidate + P_candidate.transpose());

  out.finite = P_candidate.allFinite();
  out.symmetry_error = out.finite
                           ? (P_candidate - P_candidate.transpose()).norm()
                           : std::numeric_limits<double>::infinity();
  out.symmetric = out.finite && out.symmetry_error <= 1e-10;
  out.min_diagonal = out.finite ? P_candidate.diagonal().minCoeff()
                                : -std::numeric_limits<double>::infinity();
  const double diag_scale = out.finite
                                ? std::max(1.0, P_candidate.diagonal().cwiseAbs().maxCoeff())
                                : 1.0;
  const double diag_tol = 1e-12 * diag_scale;
  out.nonnegative_diagonal = out.finite && out.min_diagonal >= -diag_tol;

  // Eliminate only roundoff-sized negative diagonals. Material negative values
  // reject the complete update below.
  if (out.nonnegative_diagonal) {
    for (int i = 0; i < N; i++) {
      if (P_candidate(i, i) < 0.0) P_candidate(i, i) = 0.0;
    }
    out.min_diagonal = P_candidate.diagonal().minCoeff();
  }

  out.psd = !check_psd;
  if (out.finite && out.symmetric && out.nonnegative_diagonal && check_psd) {
    const double psd_tol = 1e-10 * diag_scale;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(P_candidate);
    if (ldlt.info() == Eigen::Success && ldlt.vectorD().allFinite()) {
      out.min_ldlt_diagonal = ldlt.vectorD().minCoeff();
      out.psd = out.min_ldlt_diagonal >= -psd_tol;
    } else {
      // LDLT (no definiteness pivoting) returns a non-finite D on a *singular*
      // but valid PSD covariance — routine in VIO (rank-deficient / near-
      // unobservable directions, freshly initialized landmarks). A Joseph
      // update of a PSD prior is provably PSD, so falling through to "not PSD"
      // here false-rejects healthy GPS-Z updates. Fall back to the true minimum
      // eigenvalue (always well defined for a symmetric matrix) instead.
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_candidate, Eigen::EigenvaluesOnly);
      if (es.info() == Eigen::Success && es.eigenvalues().allFinite()) {
        out.min_ldlt_diagonal = es.eigenvalues().minCoeff();
        out.psd = out.min_ldlt_diagonal >= -psd_tol;
      } else {
        out.psd = false;  // genuinely unfactorable / non-symmetric-spectrum -> reject
      }
    }
  }

  const Eigen::VectorXd dx = K_full * res;
  const bool valid = out.finite && out.symmetric && out.nonnegative_diagonal &&
                     out.psd && dx.allFinite();
  if (!valid) {
    if (health) *health = out;
    return false;
  }

  state->_Cov = std::move(P_candidate);

  // Covariance update — Joseph form (K_full may have zeroed entries for masking)
  // Covariance candidate was committed above after all requested checks.

  // State correction: dx = K_full * res
  // Variables with all-zero K block are skipped (masked out).
  for (const auto &var : state->_variables) {
    const int id = var->id(), sz = var->size();
    if (id < 0 || id + sz > N) continue;
    if (dx.segment(id, sz).norm() < 1e-18) continue;
    var->update(dx.segment(id, sz));
  }
  if (health) *health = out;
  return true;
}

// =============================================================================
// computeLearUnderweightGain — NASA/Lear scalar measurement underweighting
// Navigation Filter Best Practices, NTRS 20180003657 Eq. 4.36/4.38;
// NTRS 20250002787 Eq. 5.36.  See StateHelper.h for the derivation.
// =============================================================================
Eigen::VectorXd StateHelper::computeLearUnderweightGain(
    const Eigen::VectorXd &gain_numerator, double hph, double R, double beta,
    double &R_eff_out, double &W_U_out) {
  const double b = std::max(0.0, beta);
  const double q = std::max(0.0, hph);
  // Eq. 4.38 effective innovation variance and Eq. 4.36 additive residual noise.
  W_U_out = (1.0 + b) * q + R;
  R_eff_out = R + b * q;
  if (!std::isfinite(W_U_out) || W_U_out <= 1e-18) {
    // Degenerate prior — return a zero gain so the caller applies no update.
    return Eigen::VectorXd::Zero(gain_numerator.size());
  }
  return gain_numerator / W_U_out;
}

// =============================================================================
// EKFUpdateJosephMasked — convenience wrapper
// Ported from PX4-Autopilot fuseHaglRng K-masking pattern
// Commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909, BSD-3 License
// Copyright (c) 2012-2025 PX4 Development Team
// =============================================================================
void StateHelper::EKFUpdateJosephMasked(std::shared_ptr<State> state,
                                         const std::vector<std::shared_ptr<Type>> &H_order,
                                         const Eigen::MatrixXd &H,
                                         const Eigen::VectorXd &res,
                                         const Eigen::MatrixXd &R,
                                         std::shared_ptr<Type> active_var,
                                         int active_dof) {
  assert(H.rows() == 1 && res.rows() == 1 && R.rows() == 1 && R.cols() == 1);

  const int N = (int)state->_Cov.rows();
  const int active_global_idx = active_var->id() + active_dof;
  assert(active_global_idx >= 0 && active_global_idx < N);

  // --- Expand compressed H to global H_full (N×1) ---
  Eigen::VectorXd H_full = Eigen::VectorXd::Zero(N);
  {
    int col = 0;
    for (const auto &var : H_order) {
      for (int k = 0; k < var->size(); k++)
        H_full(var->id() + k) = H(0, col + k);
      col += var->size();
    }
  }

  // --- Compute optimal K = P * H_full / S ---
  Eigen::VectorXd M = state->_Cov * H_full;          // N×1 gain numerator
  const double S = H_full.dot(M) + R(0, 0);           // innovation variance
  if (S < 1e-18) return;                               // degenerate: skip
  Eigen::VectorXd K_full = M / S;

  // --- Mask K: zero all entries except active_global_idx (PX4 fuseHaglRng pattern) ---
  const double k_active = K_full(active_global_idx);
  K_full.setZero();
  K_full(active_global_idx) = k_active;

  // --- Joseph-form update with masked K ---
  EKFUpdateJoseph(state, K_full, H_full, R(0, 0), res(0));
}

Eigen::MatrixXd StateHelper::get_marginal_covariance(std::shared_ptr<State> state,
                                                     const std::vector<std::shared_ptr<Type>> &small_variables) {

  // Calculate the marginal covariance size we need to make our matrix
  int cov_size = 0;
  for (size_t i = 0; i < small_variables.size(); i++) {
    cov_size += small_variables[i]->size();
  }

  // Construct our return covariance
  Eigen::MatrixXd Small_cov = Eigen::MatrixXd::Zero(cov_size, cov_size);

  // For each variable, lets copy over all other variable cross terms
  // Note: this copies over itself to when i_index=k_index
  int i_index = 0;
  for (size_t i = 0; i < small_variables.size(); i++) {
    int k_index = 0;
    for (size_t k = 0; k < small_variables.size(); k++) {
      Small_cov.block(i_index, k_index, small_variables[i]->size(), small_variables[k]->size()) =
          state->_Cov.block(small_variables[i]->id(), small_variables[k]->id(), small_variables[i]->size(), small_variables[k]->size());
      k_index += small_variables[k]->size();
    }
    i_index += small_variables[i]->size();
  }

  // Return the covariance
  // Small_cov = 0.5*(Small_cov+Small_cov.transpose());
  return Small_cov;
}

Eigen::MatrixXd StateHelper::get_full_covariance(std::shared_ptr<State> state) {

  // Size of the covariance is the active
  int cov_size = (int)state->_Cov.rows();

  // Construct our return covariance
  Eigen::MatrixXd full_cov = Eigen::MatrixXd::Zero(cov_size, cov_size);

  // Copy in the active state elements
  full_cov.block(0, 0, state->_Cov.rows(), state->_Cov.rows()) = state->_Cov;

  // Return the covariance
  return full_cov;
}

void StateHelper::marginalize(std::shared_ptr<State> state, std::shared_ptr<Type> marg) {

  // Check if the current state has the element we want to marginalize
  if (std::find(state->_variables.begin(), state->_variables.end(), marg) == state->_variables.end()) {
    PRINT_ERROR(RED "StateHelper::marginalize() - Called on variable that is not in the state\n" RESET);
    PRINT_ERROR(RED "StateHelper::marginalize() - Marginalization, does NOT work on sub-variables yet...\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Generic covariance has this form for x_1, x_m, x_2. If we want to remove x_m:
  //
  //  P_(x_1,x_1) P(x_1,x_m) P(x_1,x_2)
  //  P_(x_m,x_1) P(x_m,x_m) P(x_m,x_2)
  //  P_(x_2,x_1) P(x_2,x_m) P(x_2,x_2)
  //
  //  to
  //
  //  P_(x_1,x_1) P(x_1,x_2)
  //  P_(x_2,x_1) P(x_2,x_2)
  //
  // i.e. x_1 goes from 0 to marg_id, x_2 goes from marg_id+marg_size to Cov.rows() in the original covariance

  int marg_size = marg->size();
  int marg_id = marg->id();
  int x2_size = (int)state->_Cov.rows() - marg_id - marg_size;

  Eigen::MatrixXd Cov_new(state->_Cov.rows() - marg_size, state->_Cov.rows() - marg_size);

  // P_(x_1,x_1)
  Cov_new.block(0, 0, marg_id, marg_id) = state->_Cov.block(0, 0, marg_id, marg_id);

  // P_(x_1,x_2)
  Cov_new.block(0, marg_id, marg_id, x2_size) = state->_Cov.block(0, marg_id + marg_size, marg_id, x2_size);

  // P_(x_2,x_1)
  Cov_new.block(marg_id, 0, x2_size, marg_id) = Cov_new.block(0, marg_id, marg_id, x2_size).transpose();

  // P(x_2,x_2)
  Cov_new.block(marg_id, marg_id, x2_size, x2_size) = state->_Cov.block(marg_id + marg_size, marg_id + marg_size, x2_size, x2_size);

  // Now set new covariance
  // state->_Cov.resize(Cov_new.rows(),Cov_new.cols());
  state->_Cov = Cov_new;
  // state->Cov() = 0.5*(Cov_new+Cov_new.transpose());
  assert(state->_Cov.rows() == Cov_new.rows());

  // Now we keep the remaining variables and update their ordering
  // Note: DOES NOT SUPPORT MARGINALIZING SUBVARIABLES YET!!!!!!!
  std::vector<std::shared_ptr<Type>> remaining_variables;
  for (size_t i = 0; i < state->_variables.size(); i++) {
    // Only keep non-marginal states
    if (state->_variables.at(i) != marg) {
      if (state->_variables.at(i)->id() > marg_id) {
        // If the variable is "beyond" the marginal one in ordering, need to "move it forward"
        state->_variables.at(i)->set_local_id(state->_variables.at(i)->id() - marg_size);
      }
      remaining_variables.push_back(state->_variables.at(i));
    }
  }

  // Delete the old state variable to free up its memory
  // NOTE: we don't need to do this any more since our variable is a shared ptr
  // NOTE: thus this is automatically managed, but this allows outside references to keep the old variable
  // delete marg;
  marg->set_local_id(-1);

  // Now set variables as the remaining ones
  state->_variables = remaining_variables;
}

std::shared_ptr<Type> StateHelper::clone(std::shared_ptr<State> state, std::shared_ptr<Type> variable_to_clone) {

  // Get total size of new cloned variables, and the old covariance size
  int total_size = variable_to_clone->size();
  int old_size = (int)state->_Cov.rows();
  int new_loc = (int)state->_Cov.rows();

  // Resize both our covariance to the new size
  state->_Cov.conservativeResizeLike(Eigen::MatrixXd::Zero(old_size + total_size, old_size + total_size));

  // What is the new state, and variable we inserted
  const std::vector<std::shared_ptr<Type>> new_variables = state->_variables;
  std::shared_ptr<Type> new_clone = nullptr;

  // Loop through all variables, and find the variable that we are going to clone
  for (size_t k = 0; k < state->_variables.size(); k++) {

    // Skip this if it is not the same
    // First check if the top level variable is the same, then check the sub-variables
    std::shared_ptr<Type> type_check = state->_variables.at(k)->check_if_subvariable(variable_to_clone);
    if (state->_variables.at(k) == variable_to_clone) {
      type_check = state->_variables.at(k);
    } else if (type_check != variable_to_clone) {
      continue;
    }

    // So we will clone this one
    int old_loc = type_check->id();

    // Copy the covariance elements
    state->_Cov.block(new_loc, new_loc, total_size, total_size) = state->_Cov.block(old_loc, old_loc, total_size, total_size);
    state->_Cov.block(0, new_loc, old_size, total_size) = state->_Cov.block(0, old_loc, old_size, total_size);
    state->_Cov.block(new_loc, 0, total_size, old_size) = state->_Cov.block(old_loc, 0, total_size, old_size);

    // Create clone from the type being cloned
    new_clone = type_check->clone();
    new_clone->set_local_id(new_loc);
    break;
  }

  // Check if the current state has this variable
  if (new_clone == nullptr) {
    PRINT_ERROR(RED "StateHelper::clone() - Called on variable is not in the state\n" RESET);
    PRINT_ERROR(RED "StateHelper::clone() - Ensure that the variable specified is a variable, or sub-variable..\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Add to variable list and return
  state->_variables.push_back(new_clone);
  return new_clone;
}

bool StateHelper::initialize(std::shared_ptr<State> state, std::shared_ptr<Type> new_variable,
                             const std::vector<std::shared_ptr<Type>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L,
                             Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult,
                             VisualYawUpdateMode visual_yaw_update_mode, double visual_yaw_update_scale,
                             double visual_global_yaw_oc_alpha, VisualOcFn oc_fn,
                             double visual_imu_yaw_gain_scale) {

  // Check that this new variable is not already initialized
  if (std::find(state->_variables.begin(), state->_variables.end(), new_variable) != state->_variables.end()) {
    PRINT_ERROR("StateHelper::initialize_invertible() - Called on variable that is already in the state\n");
    PRINT_ERROR("StateHelper::initialize_invertible() - Found this variable at %d in covariance\n", new_variable->id());
    std::exit(EXIT_FAILURE);
  }

  // Check that we have isotropic noise (i.e. is diagonal and all the same value)
  // TODO: can we simplify this so it doesn't take as much time?
  assert(R.rows() == R.cols());
  assert(R.rows() > 0);
  for (int r = 0; r < R.rows(); r++) {
    for (int c = 0; c < R.cols(); c++) {
      if (r == c && R(0, 0) != R(r, c)) {
        PRINT_ERROR(RED "StateHelper::initialize() - Your noise is not isotropic!\n" RESET);
        PRINT_ERROR(RED "StateHelper::initialize() - Found a value of %.2f verses value of %.2f\n" RESET, R(r, c), R(0, 0));
        std::exit(EXIT_FAILURE);
      } else if (r != c && R(r, c) != 0.0) {
        PRINT_ERROR(RED "StateHelper::initialize() - Your noise is not diagonal!\n" RESET);
        PRINT_ERROR(RED "StateHelper::initialize() - Found a value of %.2f at row %d and column %d\n" RESET, R(r, c), r, c);
        std::exit(EXIT_FAILURE);
      }
    }
  }

  //==========================================================
  //==========================================================
  // First we perform QR givens to seperate the system
  // The top will be a system that depends on the new state, while the bottom does not
  size_t new_var_size = new_variable->size();
  assert((int)new_var_size == H_L.cols());

  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < H_L.cols(); ++n) {
    for (int m = (int)H_L.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(H_L(m - 1, n), H_L(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (H_L.block(m - 1, n, 2, H_L.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (H_R.block(m - 1, 0, 2, H_R.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // Separate into initializing and updating portions
  // 1. Invertible initializing system
  Eigen::MatrixXd Hxinit = H_R.block(0, 0, new_var_size, H_R.cols());
  Eigen::MatrixXd H_finit = H_L.block(0, 0, new_var_size, new_var_size);
  Eigen::VectorXd resinit = res.block(0, 0, new_var_size, 1);
  Eigen::MatrixXd Rinit = R.block(0, 0, new_var_size, new_var_size);

  // 2. Nullspace projected updating system
  Eigen::MatrixXd Hup = H_R.block(new_var_size, 0, H_R.rows() - new_var_size, H_R.cols());
  Eigen::VectorXd resup = res.block(new_var_size, 0, res.rows() - new_var_size, 1);
  Eigen::MatrixXd Rup = R.block(new_var_size, new_var_size, R.rows() - new_var_size, R.rows() - new_var_size);

  //==========================================================
  //==========================================================

  // Optionally apply pre-chi2 OC projection to the updating (Hup) portion.
  // When oc_fn is provided (new pre-chi2 modes), we project Hup before the
  // Mahalanobis gate so the gate operates on the OC-consistent Jacobian.
  // In that case EKFUpdate is called with ORIGINAL mode (no re-application).
  Eigen::MatrixXd Hup_for_gate = Hup;
  if (oc_fn && Hup.rows() > 0) {
    int cur = 0;
    std::vector<int> H_id_up;
    for (const auto &meas_var : H_order) {
      H_id_up.push_back(cur);
      cur += meas_var->size();
    }
    Hup_for_gate = oc_fn(Hup, H_order, H_id_up);
  }

  // Do mahalanobis distance testing
  Eigen::MatrixXd P_up = get_marginal_covariance(state, H_order);
  assert(Rup.rows() == Hup_for_gate.rows());
  assert(Hup_for_gate.cols() == P_up.cols());
  Eigen::MatrixXd S = Hup_for_gate * P_up * Hup_for_gate.transpose() + Rup;
  double chi2 = resup.dot(S.llt().solve(resup));

  // Get what our threshold should be
  boost::math::chi_squared chi_squared_dist(res.rows());
  double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
  if (chi2 > chi_2_mult * chi2_check) {
    return false;
  }

  //==========================================================
  //==========================================================
  // Finally, initialize it in our state
  StateHelper::initialize_invertible(state, new_variable, H_order, Hxinit, H_finit, Rinit, resinit);

  // Update with updating portion.
  // If oc_fn was used, Hup_for_gate already carries the OC projection and
  // we call EKFUpdate with ORIGINAL (no second application).
  // Otherwise fall back to the legacy visual_yaw_update_mode path.
  if (Hup.rows() > 0) {
    if (oc_fn) {
      StateHelper::EKFUpdate(state, H_order, Hup_for_gate, resup, Rup,
                             VisualYawUpdateMode::ORIGINAL, 1.0, 0.0, 1.0,
                             visual_imu_yaw_gain_scale);
    } else {
      StateHelper::EKFUpdate(state, H_order, Hup, resup, Rup, visual_yaw_update_mode,
                             visual_yaw_update_scale, visual_global_yaw_oc_alpha, 1.0,
                             visual_imu_yaw_gain_scale);
    }
  }
  return true;
}

void StateHelper::initialize_invertible(std::shared_ptr<State> state, std::shared_ptr<Type> new_variable,
                                        const std::vector<std::shared_ptr<Type>> &H_order, const Eigen::MatrixXd &H_R,
                                        const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res) {

  // Check that this new variable is not already initialized
  if (std::find(state->_variables.begin(), state->_variables.end(), new_variable) != state->_variables.end()) {
    PRINT_ERROR("StateHelper::initialize_invertible() - Called on variable that is already in the state\n");
    PRINT_ERROR("StateHelper::initialize_invertible() - Found this variable at %d in covariance\n", new_variable->id());
    std::exit(EXIT_FAILURE);
  }

  // Check that we have isotropic noise (i.e. is diagonal and all the same value)
  // TODO: can we simplify this so it doesn't take as much time?
  assert(R.rows() == R.cols());
  assert(R.rows() > 0);
  for (int r = 0; r < R.rows(); r++) {
    for (int c = 0; c < R.cols(); c++) {
      if (r == c && R(0, 0) != R(r, c)) {
        PRINT_ERROR(RED "StateHelper::initialize_invertible() - Your noise is not isotropic!\n" RESET);
        PRINT_ERROR(RED "StateHelper::initialize_invertible() - Found a value of %.2f verses value of %.2f\n" RESET, R(r, c), R(0, 0));
        std::exit(EXIT_FAILURE);
      } else if (r != c && R(r, c) != 0.0) {
        PRINT_ERROR(RED "StateHelper::initialize_invertible() - Your noise is not diagonal!\n" RESET);
        PRINT_ERROR(RED "StateHelper::initialize_invertible() - Found a value of %.2f at row %d and column %d\n" RESET, R(r, c), r, c);
        std::exit(EXIT_FAILURE);
      }
    }
  }

  //==========================================================
  //==========================================================
  // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
  assert(res.rows() == R.rows());
  assert(H_L.rows() == res.rows());
  assert(H_L.rows() == H_R.rows());
  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }

  //==========================================================
  //==========================================================
  // For each active variable find its M = P*H^T
  for (const auto &var : state->_variables) {
    // Sum up effect of each subjacobian= K_i= \sum_m (P_im Hm^T)
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      std::shared_ptr<Type> meas_var = H_order.at(i);
      M_i += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
             H_R.block(0, H_id[i], H_R.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  //==========================================================
  //==========================================================
  // Get covariance of this small jacobian
  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

  // M = H_R*Cov*H_R' + R
  Eigen::MatrixXd M(H_R.rows(), H_R.rows());
  M.triangularView<Eigen::Upper>() = H_R * P_small * H_R.transpose();
  M.triangularView<Eigen::Upper>() += R;

  // Covariance of the variable/landmark that will be initialized
  assert(H_L.rows() == H_L.cols());
  assert(H_L.rows() == new_variable->size());
  Eigen::MatrixXd H_Linv = H_L.inverse();
  Eigen::MatrixXd P_LL = H_Linv * M.selfadjointView<Eigen::Upper>() * H_Linv.transpose();

  // Augment the covariance matrix
  size_t oldSize = state->_Cov.rows();
  state->_Cov.conservativeResizeLike(Eigen::MatrixXd::Zero(oldSize + new_variable->size(), oldSize + new_variable->size()));
  state->_Cov.block(0, oldSize, oldSize, new_variable->size()).noalias() = -M_a * H_Linv.transpose();
  state->_Cov.block(oldSize, 0, new_variable->size(), oldSize) = state->_Cov.block(0, oldSize, oldSize, new_variable->size()).transpose();
  state->_Cov.block(oldSize, oldSize, new_variable->size(), new_variable->size()) = P_LL;

  // Update the variable that will be initialized (invertible systems can only update the new variable).
  // However this update should be almost zero if we already used a conditional Gauss-Newton to solve for the initial estimate
  new_variable->update(H_Linv * res);

  // Now collect results, and add it to the state variables
  new_variable->set_local_id(oldSize);
  state->_variables.push_back(new_variable);

  // std::stringstream ss;
  // ss << new_variable->id() <<  " init dx = " << (H_Linv * res).transpose() << std::endl;
  // PRINT_DEBUG(ss.str().c_str());
}

void StateHelper::augment_clone(std::shared_ptr<State> state, Eigen::Matrix<double, 3, 1> last_w) {

  // We can't insert a clone that occured at the same timestamp!
  if (state->_clones_IMU.find(state->_timestamp) != state->_clones_IMU.end()) {
    PRINT_ERROR(RED "TRIED TO INSERT A CLONE AT THE SAME TIME AS AN EXISTING CLONE, EXITING!#!@#!@#\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our cloner and add it to our vector of types
  // NOTE: this will clone the clone pose to the END of the covariance...
  std::shared_ptr<Type> posetemp = StateHelper::clone(state, state->_imu->pose());

  // Cast to a JPL pose type, check if valid
  std::shared_ptr<PoseJPL> pose = std::dynamic_pointer_cast<PoseJPL>(posetemp);
  if (pose == nullptr) {
    PRINT_ERROR(RED "INVALID OBJECT RETURNED FROM STATEHELPER CLONE, EXITING!#!@#!@#\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Append the new clone to our clone vector
  state->_clones_IMU[state->_timestamp] = pose;

  // If we are doing time calibration, then our clones are a function of the time offset
  // Logic is based on Mingyang Li and Anastasios I. Mourikis paper:
  // http://journals.sagepub.com/doi/pdf/10.1177/0278364913515286
  if (state->_options.do_calib_camera_timeoffset) {
    // Jacobian to augment by
    Eigen::Matrix<double, 6, 1> dnc_dt = Eigen::MatrixXd::Zero(6, 1);
    dnc_dt.block(0, 0, 3, 1) = last_w;
    dnc_dt.block(3, 0, 3, 1) = state->_imu->vel();
    // Augment covariance with time offset Jacobian
    // TODO: replace this with a call to the EKFPropagate function instead....
    state->_Cov.block(0, pose->id(), state->_Cov.rows(), 6) +=
        state->_Cov.block(0, state->_calib_dt_CAMtoIMU->id(), state->_Cov.rows(), 1) * dnc_dt.transpose();
    state->_Cov.block(pose->id(), 0, 6, state->_Cov.rows()) +=
        dnc_dt * state->_Cov.block(state->_calib_dt_CAMtoIMU->id(), 0, 1, state->_Cov.rows());
  }
}

void StateHelper::marginalize_old_clone(std::shared_ptr<State> state) {
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    double marginal_time = state->margtimestep();
    // Lock the mutex to avoid deleting any elements from _clones_IMU while accessing it from other threads
    std::lock_guard<std::mutex> lock(state->_mutex_state);
    assert(marginal_time != INFINITY);
    StateHelper::marginalize(state, state->_clones_IMU.at(marginal_time));
    // Note that the marginalizer should have already deleted the clone
    // Thus we just need to remove the pointer to it from our state
    state->_clones_IMU.erase(marginal_time);
  }
}

void StateHelper::marginalize_slam(std::shared_ptr<State> state) {
  // Remove SLAM features that have their marginalization flag set
  // We also check that we do not remove any aruoctag landmarks
  int ct_marginalized = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((*it0).second->should_marg && (int)(*it0).first > 4 * state->_options.max_aruco_features) {
      StateHelper::marginalize(state, (*it0).second);
      it0 = state->_features_SLAM.erase(it0);
      ct_marginalized++;
    } else {
      it0++;
    }
  }
}
