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

#include "state/State.h"

#include "types/Landmark.h"
#include "utils/colors.h"
#include "utils/print.h"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>

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
                                             double scale, bool current_only) {
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

    if (current_only)
      continue;

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
                         const Eigen::Vector3d &p_IinG, bool has_velocity, const Eigen::Vector3d &v_IinG) {
  if (col < 0 || col + size > n.rows())
    return;
  if (size >= 3) {
    n.block(col, 0, 3, 1) = R_GtoI * Eigen::Vector3d::UnitZ();
  }
  if (size >= 6) {
    n.block(col + 3, 0, 3, 1) = yaw_position_dir(p_IinG);
  }
  if (has_velocity && size >= 9) {
    n.block(col + 6, 0, 3, 1) = yaw_position_dir(v_IinG);
  }
}

void fill_landmark_yaw_gauge(Eigen::VectorXd &n, int col, const std::shared_ptr<Landmark> &lm,
                             bool use_fej = false) {
  if (lm == nullptr || col < 0 || col + lm->size() > n.rows())
    return;
  if (LandmarkRepresentation::is_relative_representation(lm->_feat_representation))
    return;
  if (lm->_feat_representation != LandmarkRepresentation::GLOBAL_3D || lm->size() != 3)
    return;
  n.block(col, 0, 3, 1) = yaw_position_dir(lm->get_xyz(use_fej));
}

Eigen::VectorXd build_global_yaw_gauge_small(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
                                             const std::vector<int> &H_id, int H_cols,
                                             bool use_fej = false) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(H_cols);
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    const int col = H_id[i];

    if (var == state->_imu) {
      fill_pose_yaw_gauge(n, col, var->size(),
                          use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                          use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                          true,
                          use_fej ? state->_imu->vel_fej() : state->_imu->vel());
      continue;
    }
    if (var == state->_imu->q()) {
      fill_pose_yaw_gauge(n, col, var->size(),
                          use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                          use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                          false, Eigen::Vector3d::Zero());
      continue;
    }
    if (var == state->_imu->p()) {
      n.block(col, 0, 3, 1) = yaw_position_dir(use_fej ? state->_imu->pos_fej() : state->_imu->pos());
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
                            false, Eigen::Vector3d::Zero());
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
        n.block(col, 0, 3, 1) = yaw_position_dir(use_fej ? pose->pos_fej() : pose->pos());
        matched_clone = true;
        break;
      }
    }
    if (matched_clone)
      continue;

    fill_landmark_yaw_gauge(n, col, std::dynamic_pointer_cast<Landmark>(var), use_fej);
  }
  return n;
}

// Once-per-process flags for gauge-coverage diagnostic print (one per source).
bool g_gauge_full_diag_printed = false;
bool g_gauge_full_fej_diag_printed = false;

// Build the global yaw gauge over the FULL state vector.
// Unlike build_global_yaw_gauge_small (which only covers H_order variables), this version
// covers every variable in the filter so that the Schmidt subspace is properly defined
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
                                            bool use_fej = false) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(N);

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
                        use_fej ? state->_imu->vel_fej() : state->_imu->vel());
    imu_covered = true;
  }

  // All camera clones (PoseJPL, 6-DOF: q + p).
  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &pose = clone_pair.second;
    if (pose == nullptr) continue;
    fill_pose_yaw_gauge(n, pose->id(), pose->size(),
                        use_fej ? pose->Rot_fej() : pose->Rot(),
                        use_fej ? pose->pos_fej() : pose->pos(),
                        false, Eigen::Vector3d::Zero());
    n_clones++;
  }

  // SLAM features.
  for (const auto &feat_pair : state->_features_SLAM) {
    const auto &lm = feat_pair.second;
    if (lm == nullptr) continue;
    if (LandmarkRepresentation::is_relative_representation(lm->_feat_representation)) {
      n_slam_skip++;
    } else {
      fill_landmark_yaw_gauge(n, lm->id(), lm, use_fej);
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
               "[SCHMIDT-YAW] gauge_full first-call summary (source=%s):\n"
               "  cov_dim N         = %d\n"
               "  q_full nonzero    = %d\n"
               "  IMU covered       = %s  (id=%d size=%d)\n"
               "  clone count       = %d\n"
               "  SLAM global       = %d (filled)\n"
               "  SLAM relative     = %d (skipped, gauge=0, %d DOF)\n"
               "  cam_extr/intr/dt  = 0 (gauge=0, %d DOF)\n"
               "  q_full norm       = %.6f\n"
               RESET,
               use_fej ? "fej" : "current",
               N, n_nonzero,
               imu_covered ? "YES" : "NO",
               state->_imu ? state->_imu->id() : -1,
               state->_imu ? state->_imu->size() : 0,
               n_clones,
               n_slam, n_slam_skip, n_slam_skip_dof,
               n_zero_calib_dof,
               n.norm());
  }

  return n;
}

// Mixed gauge: IMU from current state, clones and landmarks from FEJ.
// Used as a diagnostic probe to separate IMU-FEJ vs clone-FEJ contributions.
Eigen::VectorXd build_global_yaw_gauge_mixed(std::shared_ptr<State> state, int N) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(N);
  if (state->_imu != nullptr) {
    fill_pose_yaw_gauge(n, state->_imu->id(), state->_imu->size(),
                        state->_imu->Rot(), state->_imu->pos(),   // IMU: current
                        true, state->_imu->vel());
  }
  for (const auto &clone_pair : state->_clones_IMU) {
    const auto &pose = clone_pair.second;
    if (pose == nullptr) continue;
    fill_pose_yaw_gauge(n, pose->id(), pose->size(),
                        pose->Rot_fej(), pose->pos_fej(),          // clones: FEJ
                        false, Eigen::Vector3d::Zero());
  }
  for (const auto &feat_pair : state->_features_SLAM) {
    const auto &lm = feat_pair.second;
    if (lm == nullptr) continue;
    if (!LandmarkRepresentation::is_relative_representation(lm->_feat_representation))
      fill_landmark_yaw_gauge(n, lm->id(), lm, true);             // landmarks: FEJ
  }
  return n;
}

Eigen::MatrixXd project_global_yaw_from_H(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order,
                                          const std::vector<int> &H_id, const Eigen::MatrixXd &H, double alpha,
                                          bool use_fej = false) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  if (alpha <= 1e-12)
    return H;
  Eigen::VectorXd n = build_global_yaw_gauge_small(state, H_order, H_id, H.cols(), use_fej);
  const double n2 = n.squaredNorm();
  if (n2 < 1e-12)
    return H;
  Eigen::VectorXd Hn = H * n;
  return H - (alpha / n2) * Hn * n.transpose();
}

void remove_yaw_dx_component(Eigen::VectorXd &dx, const std::shared_ptr<Type> &q_var,
                             const Eigen::Matrix3d &R_GtoI) {
  if (q_var == nullptr || q_var->id() < 0 || q_var->id() + 3 > dx.rows())
    return;
  Eigen::Vector3d g_local = R_GtoI * Eigen::Vector3d::UnitZ();
  const double n = g_local.norm();
  if (n < 1e-12)
    return;
  g_local /= n;
  const Eigen::Matrix3d P_no_yaw = Eigen::Matrix3d::Identity() - g_local * g_local.transpose();
  dx.segment(q_var->id(), 3) = (P_no_yaw * dx.segment(q_var->id(), 3)).eval();
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

void zero_visual_yaw_dx(std::shared_ptr<State> state, Eigen::VectorXd &dx, bool protect_bg_z) {
  remove_yaw_dx_component(dx, state->_imu->q(), state->_imu->Rot());

  for (const auto &clone : state->_clones_IMU) {
    const auto &pose = clone.second;
    remove_yaw_dx_component(dx, pose->q(), pose->Rot());
  }

  if (protect_bg_z && state->_imu->bg() != nullptr) {
    const int bg_id = state->_imu->bg()->id();
    if (bg_id >= 0 && bg_id + 2 < dx.rows())
      dx(bg_id + 2) = 0.0;
  }
}

// ---- Schmidt yaw diagnostics CSV ----
std::ofstream g_schmidt_yaw_diag_csv;
bool g_schmidt_yaw_diag_header_written = false;

void write_schmidt_yaw_diag_header() {
  g_schmidt_yaw_diag_csv
      << "timestamp,update_type,mode,gauge_source,H_rows,H_cols,N_cols,rank_Q,norm_Q,norm_H,condition_N,"
      << "norm_HQ,rel_norm_HQ,normal_dx_s_coeff_before,schmidt_dx_s_coeff_after,"
      << "norm_dx_normal,norm_dx_schmidt,norm_delta_dx,"
      << "Pss_norm_before,Pss_norm_after,Pss_norm_after_new_q,Pss_change_norm,Pas_change_norm,"
      << "yaw_before_update,yaw_after_update,delta_yaw_update,"
      << "bg_z_before,bg_z_after,"
      << "q_energy_imu_ori,q_energy_imu_pos,q_energy_imu_vel,"
      << "q_energy_clone_ori,q_energy_clone_pos,q_energy_slam,q_energy_bias_calib,"
      << "neg_diag_clamp_count,min_cov_diag_before_clamp,min_cov_diag_after_update,"
      << "projection_applied,schmidt_applied,skipped_reason,"
      << "rel_norm_Hq_alt,q_alt_dot_dx_eff,angle_q_alt_deg,"
      << "angle_q_cur_fej_deg,angle_q_cur_mix_deg,angle_q_fej_mix_deg,"
      << "q_mix_rel_norm_Hq,q_mix_dot_dx_eff,"
      << "q_mix_energy_imu_ori,q_mix_energy_imu_pos,q_mix_energy_imu_vel,"
      << "q_mix_energy_clone_ori,q_mix_energy_clone_pos,"
      << "q_mix_energy_slam,q_mix_energy_bias_calib\n";
  g_schmidt_yaw_diag_csv.flush();
  g_schmidt_yaw_diag_header_written = true;
}

void flush_schmidt_yaw_diag(const StateHelper::SchmidtYawDiag &d) {
  if (!g_schmidt_yaw_diag_csv.is_open())
    return;
  if (!g_schmidt_yaw_diag_header_written)
    write_schmidt_yaw_diag_header();
  g_schmidt_yaw_diag_csv
      << std::fixed << std::setprecision(9) << d.timestamp << ","
      << d.update_type << ","
      << d.mode << ","
      << d.gauge_source << ","
      << d.H_rows << "," << d.H_cols << "," << d.N_cols << ","
      << d.rank_Q << ","
      << std::setprecision(6) << d.norm_Q << ","
      << d.norm_H << ","
      << d.condition_N << ","
      << d.norm_HQ << ","
      << d.rel_norm_HQ << ","
      << d.normal_dx_s_coeff_before << ","
      << d.schmidt_dx_s_coeff_after << ","
      << d.norm_dx_normal << ","
      << d.norm_dx_schmidt << ","
      << d.norm_delta_dx << ","
      << d.Pss_norm_before << ","
      << d.Pss_norm_after << ","
      << d.Pss_norm_after_new_q << ","
      << d.Pss_change_norm << ","
      << d.Pas_change_norm << ","
      << d.yaw_before_update << ","
      << d.yaw_after_update << ","
      << d.delta_yaw_update << ","
      << d.bg_z_before << ","
      << d.bg_z_after << ","
      << d.q_energy_imu_ori << ","
      << d.q_energy_imu_pos << ","
      << d.q_energy_imu_vel << ","
      << d.q_energy_clone_ori << ","
      << d.q_energy_clone_pos << ","
      << d.q_energy_slam << ","
      << d.q_energy_bias_calib << ","
      << d.neg_diag_clamp_count << ","
      << d.min_cov_diag_before_clamp << ","
      << d.min_cov_diag_after_update << ","
      << (int)d.projection_applied << ","
      << (int)d.schmidt_applied << ","
      << d.skipped_reason << ","
      << d.rel_norm_Hq_alt << ","
      << d.q_alt_dot_dx_eff << ","
      << d.angle_q_alt_deg << ","
      << d.angle_q_cur_fej_deg << ","
      << d.angle_q_cur_mix_deg << ","
      << d.angle_q_fej_mix_deg << ","
      << d.q_mix_rel_norm_Hq << ","
      << d.q_mix_dot_dx_eff << ","
      << d.q_mix_energy_imu_ori << ","
      << d.q_mix_energy_imu_pos << ","
      << d.q_mix_energy_imu_vel << ","
      << d.q_mix_energy_clone_ori << ","
      << d.q_mix_energy_clone_pos << ","
      << d.q_mix_energy_slam << ","
      << d.q_mix_energy_bias_calib << "\n";
  g_schmidt_yaw_diag_csv.flush();
}

double extract_imu_yaw_rad(std::shared_ptr<State> state) {
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
  return std::atan2(R_ItoG(1, 0), R_ItoG(0, 0));
}

} // namespace

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

// ── VINS numeric nullspace (Route 4) static config — must be before EKFUpdate ─
static StateHelper::VinsNullspaceConfig g_vins_nullspace_cfg;

void StateHelper::EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>> &H_order, const Eigen::MatrixXd &H,
                            const Eigen::VectorXd &res, const Eigen::MatrixXd &R, VisualYawUpdateMode visual_yaw_update_mode,
                            double visual_yaw_update_scale, double visual_global_yaw_oc_alpha,
                            double visual_bgz_update_scale) {

  // Dispatch: mode B_current — Schmidt, gauge from current state
  if (visual_yaw_update_mode == VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_CURRENT_GAUGE) {
    SchmidtYawDiag diag;
    diag.timestamp = state->_timestamp;
    diag.mode = "visual_yaw_schmidt_current_gauge";
    EKFUpdateSchmidtYawCurrentGauge(state, H_order, H, res, R, "visual", &diag, false, visual_bgz_update_scale);
    flush_schmidt_yaw_diag(diag);
    return;
  }
  // Dispatch: mode B_FEJ — same Schmidt math, gauge from FEJ state
  if (visual_yaw_update_mode == VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_FEJ_GAUGE) {
    SchmidtYawDiag diag;
    diag.timestamp = state->_timestamp;
    diag.mode = "visual_yaw_schmidt_fej_gauge";
    EKFUpdateSchmidtYawCurrentGauge(state, H_order, H, res, R, "visual", &diag, true, visual_bgz_update_scale);
    flush_schmidt_yaw_diag(diag);
    return;
  }
  // Dispatch: mode C — current-yaw-gauge H-space projection, then standard EKF.
  // NOTE: mode C does NOT write to schmidt_yaw_update_diag.csv.  It projects H
  // before calling EKFUpdate(ORIGINAL), so the Schmidt gain-projection path (mode B)
  // is never entered and no SchmidtYawDiag is emitted.  If mode C diagnostics are
  // needed later, add a separate EKFUpdateYawGaugeHProjectionCurrentWithDiag() that
  // logs to either a dedicated CSV or a shared visual-yaw-gauge diagnostic CSV.
  if (visual_yaw_update_mode == VisualYawUpdateMode::VISUAL_YAW_H_PROJECTION_CURRENT) {
    EKFUpdateYawGaugeHProjectionCurrent(state, H_order, H, res, R);
    return;
  }
  // Dispatch: mode D — Schmidt + pre-update guard (R-inflate or reject suspicious updates)
  if (visual_yaw_update_mode == VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_GUARDED) {
    EKFUpdateSchmidtGuarded(state, H_order, H, res, R, "visual");
    return;
  }

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
  // DSO_INCREMENT_ORTHO (enum 11): v1 K-projection retired; mode removed from official dispatch.
  // String "dso_increment_ortho" now redirects to GLOBAL_YAW_OC_FEJ_PROJECTION in VioManager.
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, 1.0, /*use_fej=*/false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, 0.0, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE ||
             visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale,
                                         visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE);
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

  // Invert S: Cholesky for all modes.
  // VINS_NUMERIC_NULLSPACE (enum 12) is retired — fly3 smoke showed n_zeroed=0 at
  // eig_thresh=1e-6, meaning EVD adds no nullspace maintenance vs Cholesky.
  Eigen::MatrixXd K(M_a.rows(), R.rows());
  if (visual_yaw_update_mode == VisualYawUpdateMode::VINS_NUMERIC_NULLSPACE) {
    static bool g_vins_warned = false;
    if (!g_vins_warned) {
      PRINT_WARNING(RED "[VINS-NULLSPACE] mode is retired (smoke n_zeroed=0 at 1e-6); "
                        "falling through to Cholesky.\n" RESET);
      g_vins_warned = true;
    }
  }
  {
    Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
    S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);
    K = M_a * Sinv.selfadjointView<Eigen::Upper>();
  }

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

  if (bgz_row_scaled) {
    // K no longer equals M_a S^{-1}, so the standard form P -= K M_a^T is invalid
    // (non-symmetric, goes non-PSD).  Use the gain-agnostic consistent form
    //   P+ = P - K' M_a^T - M_a K'^T + K' S K'^T
    // which reduces to the standard form when K' = K, and keeps bg_z a proper
    // "considered" state: its variance is not reduced by the visual update.
    Eigen::MatrixXd KM = K * M_a.transpose();
    Eigen::MatrixXd Sfull = S.selfadjointView<Eigen::Upper>();
    Eigen::MatrixXd KSK = K * Sfull * K.transpose();
    state->_Cov += KSK - KM - KM.transpose();
    state->_Cov = (0.5 * (state->_Cov + state->_Cov.transpose())).eval();
  } else {
    // Covariance update (standard for all modes; K is unmodified so K = M_a S^{-1} holds).
    state->_Cov.triangularView<Eigen::Upper>() -= K * M_a.transpose();
    state->_Cov = state->_Cov.selfadjointView<Eigen::Upper>();
  }

  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  Eigen::VectorXd diags = state->_Cov.diagonal();
  bool found_neg = false;
  for (int i = 0; i < diags.rows(); i++) {
    if (diags(i) < 0.0) {
      PRINT_WARNING(RED "StateHelper::EKFUpdate() - diagonal at %d is %.2f\n" RESET, i, diags(i));
      found_neg = true;
    }
  }
  if (found_neg) {
    std::exit(EXIT_FAILURE);
  }

  // Calculate our delta and update all our active states
  Eigen::VectorXd dx = K * res;
  last_yaw_dx_projection_diag.valid = true;
  last_yaw_dx_projection_diag.mode = visual_yaw_update_mode;
  last_yaw_dx_projection_diag.dx_yaw_before_projection_deg = yaw_dx_component_deg(state, dx);
  if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    zero_visual_yaw_dx(state, dx, true);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    zero_visual_yaw_dx(state, dx, false);
  }
  last_yaw_dx_projection_diag.dx_yaw_after_projection_deg = yaw_dx_component_deg(state, dx);

  // CONSTRAINED_YAW_NULLSPACE: post-update rank-1 constraint nᵀδx=0 (Simon & Chia 2002).
  // Runs AFTER the standard EKF P update (P is P_new here), modifying both dx and P.
  // dx_c = dx - K_c(nᵀdx),  P_c = P - P·n·nᵀ·P / (nᵀPn)
  if (visual_yaw_update_mode == VisualYawUpdateMode::CONSTRAINED_YAW_NULLSPACE) {
    const int N = (int)state->_Cov.rows();
    Eigen::VectorXd n = build_global_yaw_gauge_full(state, N, /*use_fej=*/true);
    const double n_norm = n.norm();

    static int g_con_n = 0;
    ++g_con_n;
    bool do_diag = (g_con_n <= 10 || g_con_n % 500 == 0);

    const bool cov_finite = state->_Cov.allFinite();
    const bool dx_finite  = dx.allFinite();

    // Compute P_n = nᵀ P n (scalar gauge variance)
    double P_n = 0.0;
    Eigen::VectorXd Pn;
    if (n_norm > 1e-10 && cov_finite) {
      Pn = state->_Cov * n;
      P_n = n.dot(Pn);
    }

    bool can_constrain = (n_norm > 1e-10) && cov_finite && dx_finite && (P_n > 1e-15);
    if (!can_constrain) {
      PRINT_WARNING(YELLOW "[CNSTR-YAW #%d] SKIPPED: n_norm=%.3e P_n=%.3e "
                           "cov_finite=%s dx_finite=%s\n" RESET,
                   g_con_n, n_norm, P_n,
                   cov_finite ? "OK" : "NaN/Inf", dx_finite ? "OK" : "NaN/Inf");
    } else {
      // Collect cross-covariance diagnostics before projection
      int pz_idx = -1, px_idx = -1, py_idx = -1, qz_idx = -1;
      if (state->_imu) {
        if (state->_imu->p()) {
          px_idx = state->_imu->p()->id() + 0;
          py_idx = state->_imu->p()->id() + 1;
          pz_idx = state->_imu->p()->id() + 2;
        }
        if (state->_imu->q())
          qz_idx = state->_imu->q()->id() + 2;
      }

      const double nTdx_before = (n_norm > 1e-30) ? n.dot(dx) / n_norm : 0.0;
      const double min_diag_before = state->_Cov.diagonal().minCoeff();
      double P_zx_b = 0, P_zy_b = 0, P_zt_b = 0;
      if (do_diag && pz_idx >= 0) {
        if (px_idx >= 0) P_zx_b = state->_Cov(pz_idx, px_idx);
        if (py_idx >= 0) P_zy_b = state->_Cov(pz_idx, py_idx);
        if (qz_idx >= 0) P_zt_b = state->_Cov(pz_idx, qz_idx);
      }

      // Constraint gain K_c = P·n / P_n  (N-vector)
      Eigen::VectorXd K_c = Pn / P_n;

      // Apply to dx
      const double nTdx = n.dot(dx);
      const Eigen::VectorXd dx_correction = K_c * nTdx;
      const double correction_norm = dx_correction.norm();
      dx -= dx_correction;

      // Apply rank-1 deflation to P: P_c = P - P·n·nᵀ·P / P_n = P - Pn·Pnᵀ / P_n
      state->_Cov.noalias() -= (Pn * Pn.transpose()) / P_n;
      state->_Cov = 0.5 * (state->_Cov + state->_Cov.transpose());

      const double nTdx_after   = (n_norm > 1e-30) ? n.dot(dx) / n_norm : 0.0;
      const double min_diag_after = state->_Cov.diagonal().minCoeff();
      const double sym_err_after  = (state->_Cov - state->_Cov.transpose()).norm();

      if (do_diag) {
        double P_zx_a = 0, P_zy_a = 0, P_zt_a = 0;
        if (pz_idx >= 0) {
          if (px_idx >= 0) P_zx_a = state->_Cov(pz_idx, px_idx);
          if (py_idx >= 0) P_zy_a = state->_Cov(pz_idx, py_idx);
          if (qz_idx >= 0) P_zt_a = state->_Cov(pz_idx, qz_idx);
        }
        PRINT_INFO(YELLOW "[CNSTR-YAW #%d] nᵀdx/||n||: %.3e → %.3e  "
                          "corr_norm=%.3e  P_n=%.3e\n" RESET,
                   g_con_n, nTdx_before, nTdx_after, correction_norm, P_n);
        PRINT_INFO(YELLOW "[CNSTR-YAW #%d] min_diag: %.3e → %.3e  sym_err=%.3e\n" RESET,
                   g_con_n, min_diag_before, min_diag_after, sym_err_after);
        if (pz_idx >= 0)
          PRINT_INFO(YELLOW "[CNSTR-YAW #%d] P_zx: %.3e→%.3e  P_zy: %.3e→%.3e  "
                            "P_zt: %.3e→%.3e\n" RESET,
                     g_con_n, P_zx_b, P_zx_a, P_zy_b, P_zy_a, P_zt_b, P_zt_a);
        if (g_con_n <= 2) {
          Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(state->_Cov, Eigen::EigenvaluesOnly);
          PRINT_INFO(YELLOW "[CNSTR-YAW #%d] min_ev=%.3e\n" RESET,
                     g_con_n, saes.eigenvalues().minCoeff());
        }
      }
    }
  }

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
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, 1.0, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, 0.0, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE ||
             visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale,
                                         visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE);
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
  if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    zero_visual_yaw_dx(state, dx, true);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    zero_visual_yaw_dx(state, dx, false);
  }

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
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    H_eff = project_global_yaw_from_H(state, H_order, H_id, H, 1.0, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, 0.0, false);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::PER_BLOCK_SCALE ||
             visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE) {
    H_eff = project_per_block_yaw_from_H(state, H_order, H_id, H, visual_yaw_update_scale,
                                         visual_yaw_update_mode == VisualYawUpdateMode::CURRENT_ONLY_SCALE);
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
  if (visual_yaw_update_mode == VisualYawUpdateMode::HARD_GYRO_YAW) {
    zero_visual_yaw_dx(state, dx, true);
  } else if (visual_yaw_update_mode == VisualYawUpdateMode::A_STRICT_YAW_DX0) {
    zero_visual_yaw_dx(state, dx, false);
  }
  return dx;
}

double StateHelper::yaw_delta_from_full_dx_deg(std::shared_ptr<State> state, const Eigen::VectorXd &dx) {
  return yaw_dx_component_deg(state, dx);
}

void StateHelper::EKFUpdateSchmidt(std::shared_ptr<State> state,
                                   const std::vector<std::shared_ptr<Type>> &H_order,
                                   const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
                                   const Eigen::MatrixXd &R) {

  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());

  // [中文] Bar-Shalom "consider filter" / sub-optimal Schmidt (consistent form).
  // 流程:
  //   1. 和标准 EKF 一样计算 full-state Kalman gain (保证 cov 更新 PSD safe).
  //   2. cov 按标准 EKF 更新: P <- P - K * S * K^T.
  //   3. mean 只更新 active (H_order) 变量; 其他 ("nuisance") 的 mean 保持不变.
  // 这样协方差是全状态协同更新 (不会违反 Schur-PSD), 但 bias/attitude 等易被
  // cross-cov 错误拉偏的状态的 mean 被保护住了.

  // ---- M_a = P * H^T (按标准 EKFUpdate 的方式计算, active 列由 H_order 决定) ----
  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(state->_Cov.rows(), res.rows());
  int cur_it = 0;
  std::vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(cur_it);
    cur_it += meas_var->size();
  }
  for (const auto &var : state->_variables) {
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      auto meas_var = H_order[i];
      M_i.noalias() += state->_Cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) *
                       H.block(0, H_id[i], H.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  // S = H P_SS H^T + R (P_SS 是 active marginal)
  Eigen::MatrixXd P_SS = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S = H * P_SS * H.transpose() + R;
  Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(R.rows(), R.rows());
  S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);

  // K_full = M_a * Sinv  (N_state x m)
  Eigen::MatrixXd K_full = M_a * Sinv.selfadjointView<Eigen::Upper>();

  // Cov 更新 (standard): P -= K * M_a^T
  state->_Cov.triangularView<Eigen::Upper>() -= K_full * M_a.transpose();
  state->_Cov = state->_Cov.selfadjointView<Eigen::Upper>();

  // Mean 更新: 只对 active 变量 (H_order) 应用 dx = K * res, 其他全部保留
  Eigen::VectorXd dx = K_full * res;
  std::set<int> active_ids;
  for (const auto &v : H_order)
    active_ids.insert(v->id());
  for (const auto &var : state->_variables) {
    if (active_ids.count(var->id()) == 0)
      continue;
    var->update(dx.block(var->id(), 0, var->size(), 1));
  }

  // Robustness check
  Eigen::VectorXd diags = state->_Cov.diagonal();
  bool found_neg = false;
  for (int i = 0; i < diags.rows(); i++) {
    if (diags(i) < 0.0) {
      PRINT_WARNING(RED "EKFUpdateSchmidt() - diag at %d = %.2f\n" RESET, i, diags(i));
      found_neg = true;
    }
  }
  if (found_neg) {
    std::exit(EXIT_FAILURE);
  }
}

void StateHelper::open_schmidt_yaw_diag_csv(const std::string &path) {
  if (g_schmidt_yaw_diag_csv.is_open())
    g_schmidt_yaw_diag_csv.close();
  g_schmidt_yaw_diag_header_written = false;
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  g_schmidt_yaw_diag_csv.open(path, std::ios::out | std::ios::trunc);
  if (!g_schmidt_yaw_diag_csv.is_open()) {
    PRINT_WARNING(YELLOW "[SCHMIDT-YAW] failed to open diag CSV: %s\n" RESET, path.c_str());
    return;
  }
  PRINT_INFO(GREEN "[SCHMIDT-YAW] writing diag CSV: %s\n" RESET, path.c_str());
}

void StateHelper::EKFUpdateSchmidtYawCurrentGauge(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &res,
    const Eigen::MatrixXd &R,
    const std::string &update_type,
    SchmidtYawDiag *diag_out,
    bool use_fej,
    double bgz_scale) {

  // =========================================================================
  // Full-state Schmidt / consider-state Kalman update.
  //
  // The protected subspace is a single direction q (the global yaw gauge)
  // in the FULL N_state-dimensional state space — not just the H_order-local
  // subspace.  Every other direction is "active" and receives corrections
  // including through cross-covariance from non-H_order variables.
  //
  // Key identities used (q = q_full, unit norm):
  //   K_eff = (I - q q^T) K_std        [standard gain with yaw direction removed]
  //   q^T K_eff = 0                    [by construction]
  //   dx_eff = K_eff r                 [q^T dx_eff = 0 ✓]
  //   P_plus = P - K_eff M_a^T - M_a K_eff^T + K_eff S K_eff^T  [Joseph form]
  //   q^T P_plus q = q^T P q           [Pss unchanged ✓]
  //   P_as changes                     [cross-covariance updated ✓]
  // =========================================================================

  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());

  const int m   = (int)H.rows();
  const int n_H = (int)H.cols();
  const int N   = (int)state->_Cov.rows();  // full state dimension

  // -- H_id: column offsets of H_order variables in H -----------------------
  int cur_it = 0;
  std::vector<int> H_id;
  H_id.reserve(H_order.size());
  for (const auto &v : H_order) { H_id.push_back(cur_it); cur_it += v->size(); }
  assert(cur_it == n_H);

  // -- Pre-update diagnostics ------------------------------------------------
  double yaw_before  = extract_imu_yaw_rad(state);
  double bg_z_before = (state->_imu->bg() != nullptr) ? state->_imu->bg()->value()(2) : 0.0;

  if (diag_out) {
    diag_out->H_rows          = m;
    diag_out->H_cols          = n_H;
    diag_out->N_cols          = 1;
    diag_out->yaw_before_update = yaw_before * (180.0 / M_PI);
    diag_out->bg_z_before     = bg_z_before;
  }

  // -- Build FULL-STATE global yaw gauge Q_full (N × 1) ----------------------
  // Covers ALL variables (IMU, all clones, all SLAM features, etc.).
  // Non-H_order variables contribute to the gauge but their H_full columns are
  // zero, so they don't affect the innovation; they DO receive corrections through
  // cross-covariance (the active subspace includes them).
  if (diag_out)
    diag_out->gauge_source = use_fej ? "fej" : "current";

  Eigen::VectorXd Q_full = build_global_yaw_gauge_full(state, N, use_fej);
  double norm_Q = Q_full.norm();

  if (norm_Q < 1e-8) {
    PRINT_WARNING(YELLOW "[SCHMIDT-YAW] norm_Q_full=%.2e < 1e-8, falling back to standard EKFUpdate\n" RESET, norm_Q);
    if (diag_out) { diag_out->skipped_reason = "norm_Q_full_below_threshold"; }
    EKFUpdate(state, H_order, H, res, R, VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
    return;
  }

  Eigen::VectorXd q = Q_full / norm_Q;   // N × 1, unit full-state gauge

  // -- H restricted to H_order columns: H_q = H * q[H_order section] --------
  // This equals H_full * q (non-H_order H_full columns are zero).
  Eigen::VectorXd q_Horder(n_H);
  for (size_t k = 0; k < H_order.size(); k++)
    q_Horder.segment(H_id[k], H_order[k]->size()) =
        q.segment(H_order[k]->id(), H_order[k]->size());
  Eigen::VectorXd HQ       = H * q_Horder;   // m × 1
  double norm_HQ           = HQ.norm();
  double norm_H            = H.norm();
  double rel_norm_HQ       = (norm_H > 1e-12) ? norm_HQ / norm_H : 0.0;

  // -- q-energy decomposition (fraction of ||q||^2 in each state block) ---------
  // NOTE: q mixes rad (orientation), m (position), m/s (velocity).
  // Euclidean fractions are reported as-is; the user interprets physical meaning.
  double q_energy_imu_ori = 0, q_energy_imu_pos = 0, q_energy_imu_vel = 0;
  double q_energy_clone_ori = 0, q_energy_clone_pos = 0, q_energy_slam = 0;
  {
    if (state->_imu != nullptr) {
      int id = state->_imu->id();
      q_energy_imu_ori = q.segment(id, 3).squaredNorm();
      q_energy_imu_pos = q.segment(id+3, 3).squaredNorm();
      q_energy_imu_vel = q.segment(id+6, 3).squaredNorm();
      // bg/ba at id+9 and id+12 are zero by gauge construction
    }
    for (const auto &clone_pair : state->_clones_IMU) {
      const auto &pose = clone_pair.second;
      if (!pose || pose->id() < 0) continue;
      int id = pose->id();
      q_energy_clone_ori += q.segment(id, 3).squaredNorm();
      q_energy_clone_pos += q.segment(id+3, 3).squaredNorm();
    }
    for (const auto &feat_pair : state->_features_SLAM) {
      const auto &lm = feat_pair.second;
      if (!lm || lm->id() < 0 || lm->id() + lm->size() > N) continue;
      q_energy_slam += q.segment(lm->id(), lm->size()).squaredNorm();
    }
  }
  // q is unit-norm so sum should = 1; remainder = bias + calibration
  double q_energy_bias_calib = 1.0 - q_energy_imu_ori - q_energy_imu_pos - q_energy_imu_vel
                               - q_energy_clone_ori - q_energy_clone_pos - q_energy_slam;
  q_energy_bias_calib = std::max(0.0, q_energy_bias_calib);  // guard numerical round-off

  if (diag_out) {
    diag_out->rank_Q             = 1;
    diag_out->norm_Q             = 1.0;   // normalized by construction
    diag_out->norm_H             = norm_H;
    diag_out->condition_N        = 1.0;
    diag_out->norm_HQ            = norm_HQ;
    diag_out->rel_norm_HQ        = rel_norm_HQ;
    diag_out->q_energy_imu_ori   = q_energy_imu_ori;
    diag_out->q_energy_imu_pos   = q_energy_imu_pos;
    diag_out->q_energy_imu_vel   = q_energy_imu_vel;
    diag_out->q_energy_clone_ori = q_energy_clone_ori;
    diag_out->q_energy_clone_pos = q_energy_clone_pos;
    diag_out->q_energy_slam      = q_energy_slam;
    diag_out->q_energy_bias_calib= q_energy_bias_calib;
  }

  // -- M_a = P * H_full^T  (same loop as standard EKFUpdate) ----------------
  // M_a[var] = sum_k P(var, H_order[k]) * H[:,k_cols]^T
  Eigen::MatrixXd M_a = Eigen::MatrixXd::Zero(N, m);
  for (const auto &var : state->_variables) {
    Eigen::MatrixXd M_i = Eigen::MatrixXd::Zero(var->size(), m);
    for (size_t k = 0; k < H_order.size(); k++) {
      M_i.noalias() +=
          state->_Cov.block(var->id(), H_order[k]->id(), var->size(), H_order[k]->size()) *
          H.block(0, H_id[k], m, H_order[k]->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), m) = M_i;
  }

  // -- S = H P_H H^T + R (innovation covariance; identical to standard) ------
  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S(m, m);
  S.triangularView<Eigen::Upper>() = H * P_small * H.transpose();
  S.triangularView<Eigen::Upper>() += R;
  Eigen::MatrixXd Sinv = Eigen::MatrixXd::Identity(m, m);
  S.selfadjointView<Eigen::Upper>().llt().solveInPlace(Sinv);

  // -- Standard Kalman gain K_std = M_a * S^{-1}  (N × m) -------------------
  Eigen::MatrixXd K_std = M_a * Sinv.selfadjointView<Eigen::Upper>();

  // -- Schmidt gain K_eff = (I - q q^T) K_std --------------------------------
  // Subtracts the yaw-gauge component from the standard gain.
  // q^T K_eff = 0 by construction → q^T dx_eff = 0.
  Eigen::RowVectorXd qT_Kstd = q.transpose() * K_std;   // 1 × m
  Eigen::MatrixXd K_eff = K_std;
  K_eff.noalias() -= q * qT_Kstd;   // N × m

  // Scale bg_z row (visual bg_z ablation; bgz_scale=1.0 = no change)
  if (bgz_scale < 1.0 - 1e-12 &&
      state->_imu != nullptr && state->_imu->bg() != nullptr) {
    int bgz_row = state->_imu->bg()->id() + 2;
    if (bgz_row >= 0 && bgz_row < N)
      K_eff.row(bgz_row) *= bgz_scale;
  }

  // -- Diagnostics: normal EKF dx_s coefficient (what standard EKF would give)
  Eigen::VectorXd dx_normal   = K_std * res;
  double normal_dx_s_coeff    = q.dot(dx_normal);   // nonzero in general

  // -- Schmidt mean update ---------------------------------------------------
  Eigen::VectorXd dx_eff      = K_eff * res;         // = dx_normal - q*(q.dot(dx_normal))
  double schmidt_dx_s_coeff   = q.dot(dx_eff);       // should be ≈ 0

  double norm_dx_normal  = dx_normal.norm();
  double norm_dx_schmidt = dx_eff.norm();
  double norm_delta_dx   = (dx_normal - dx_eff).norm();

  if (!dx_eff.allFinite()) {
    PRINT_WARNING(YELLOW "[SCHMIDT-YAW] dx_eff contains NaN/Inf, falling back to standard EKFUpdate\n" RESET);
    if (diag_out) { diag_out->skipped_reason = "dx_eff_nan_inf"; }
    EKFUpdate(state, H_order, H, res, R, VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
    return;
  }

  // -- Three-gauge comparison block (before state mutation) -----------------
  // q_cur = full-state current gauge
  // q_fej = full-state FEJ gauge
  // q_mix = IMU-current + clone-FEJ + landmark-FEJ (isolates IMU-FEJ effect)
  if (diag_out) {
    Eigen::VectorXd Q_cur = build_global_yaw_gauge_full(state, N, false);
    Eigen::VectorXd Q_fej = build_global_yaw_gauge_full(state, N, true);
    Eigen::VectorXd Q_mix = build_global_yaw_gauge_mixed(state, N);

    double n_cur = Q_cur.norm(), n_fej = Q_fej.norm(), n_mix = Q_mix.norm();
    if (n_cur < 1e-8 || n_fej < 1e-8) goto skip_three_gauge;
    {
      Eigen::VectorXd q_cur = Q_cur / n_cur;
      Eigen::VectorXd q_fej = Q_fej / n_fej;

      auto angle_deg_fn = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
        return std::acos(std::max(-1.0, std::min(1.0, a.dot(b)))) * (180.0 / M_PI);
      };
      auto rel_norm_Hq_fn = [&](const Eigen::VectorXd &qn) {
        Eigen::VectorXd qH(n_H);
        for (size_t k = 0; k < H_order.size(); k++)
          qH.segment(H_id[k], H_order[k]->size()) =
              qn.segment(H_order[k]->id(), H_order[k]->size());
        return (norm_H > 1e-12) ? (H * qH).norm() / norm_H : 0.0;
      };

      // backward-compat alt-gauge fields (alt = opposite of used gauge)
      const Eigen::VectorXd &q_alt = use_fej ? q_cur : q_fej;
      diag_out->angle_q_alt_deg  = angle_deg_fn(q, q_alt);
      diag_out->q_alt_dot_dx_eff = q_alt.dot(dx_eff);
      diag_out->rel_norm_Hq_alt  = rel_norm_Hq_fn(q_alt);

      // explicit current-vs-FEJ angle (same as angle_q_alt_deg in magnitude,
      // but always labeled as current-vs-fej regardless of mode)
      diag_out->angle_q_cur_fej_deg = angle_deg_fn(q_cur, q_fej);

      if (n_mix > 1e-8) {
        Eigen::VectorXd q_mix = Q_mix / n_mix;
        diag_out->angle_q_cur_mix_deg = angle_deg_fn(q_cur, q_mix);
        diag_out->angle_q_fej_mix_deg = angle_deg_fn(q_fej, q_mix);
        diag_out->q_mix_dot_dx_eff    = q_mix.dot(dx_eff);
        diag_out->q_mix_rel_norm_Hq   = rel_norm_Hq_fn(q_mix);

        // q_mix energy decomposition
        double &ei = diag_out->q_mix_energy_imu_ori,
               &ep = diag_out->q_mix_energy_imu_pos,
               &ev = diag_out->q_mix_energy_imu_vel,
               &co = diag_out->q_mix_energy_clone_ori,
               &cp = diag_out->q_mix_energy_clone_pos,
               &sl = diag_out->q_mix_energy_slam,
               &bc = diag_out->q_mix_energy_bias_calib;
        ei = ep = ev = co = cp = sl = bc = 0.0;
        if (state->_imu) {
          int id = state->_imu->id();
          ei = q_mix.segment(id,   3).squaredNorm();
          ep = q_mix.segment(id+3, 3).squaredNorm();
          ev = q_mix.segment(id+6, 3).squaredNorm();
        }
        for (const auto &pr : state->_clones_IMU) {
          const auto &pose = pr.second;
          if (!pose || pose->id() < 0) continue;
          co += q_mix.segment(pose->id(),   3).squaredNorm();
          cp += q_mix.segment(pose->id()+3, 3).squaredNorm();
        }
        for (const auto &pr : state->_features_SLAM) {
          const auto &lm = pr.second;
          if (!lm || lm->id() < 0 || lm->id() + lm->size() > N) continue;
          sl += q_mix.segment(lm->id(), lm->size()).squaredNorm();
        }
        bc = std::max(0.0, 1.0 - ei - ep - ev - co - cp - sl);
      }
    }
    skip_three_gauge:;
  }

  // -- Pss and Pas diagnostics BEFORE update ---------------------------------
  // Pss = q^T P q  (scalar — full-state yaw-gauge variance)
  // Pas = (I - qq^T) P q  (N×1 — cross-covariance between active and Schmidt subspaces)
  Eigen::VectorXd Pq_before = state->_Cov.selfadjointView<Eigen::Upper>() * q;
  double Pss_before          = q.dot(Pq_before);
  Eigen::VectorXd Pas_before_vec = Pq_before - q * Pss_before;

  // -- Apply dx_eff to ALL state variables -----------------------------------
  // Non-H_order variables receive nonzero corrections through cross-covariance.
  for (const auto &var : state->_variables) {
    var->update(dx_eff.segment(var->id(), var->size()));
  }

  // -- Covariance update: Joseph form ----------------------------------------
  // P_plus = P - K_eff M_a^T - M_a K_eff^T + K_eff S K_eff^T
  //
  // Proof that Pss is unchanged:
  //   q^T K_eff = 0 → q^T (K_eff M_a^T) = 0,  (M_a K_eff^T) q = 0,
  //               q^T (K_eff S K_eff^T) q = 0
  //   → q^T P_plus q = q^T P q = Pss_before ✓
  Eigen::MatrixXd KM  = K_eff * M_a.transpose();   // N × N
  Eigen::MatrixXd KSK = K_eff * S.selfadjointView<Eigen::Upper>() * K_eff.transpose();  // N × N

  state->_Cov -= KM + KM.transpose() - KSK;
  state->_Cov = 0.5 * (state->_Cov + state->_Cov.transpose());

  // -- Covariance health: count negative diagonals, clamp explicitly -----------
  int neg_clamp_count = 0;
  double min_diag_before_clamp = state->_Cov.diagonal().minCoeff();
  {
    Eigen::VectorXd diags = state->_Cov.diagonal();
    for (int i = 0; i < diags.rows(); i++) {
      if (diags(i) < 0.0) {
        neg_clamp_count++;
        if (state->_Cov(i, i) < 0.0) state->_Cov(i, i) = 1e-12;
      }
    }
  }
  if (neg_clamp_count > 0) {
    PRINT_WARNING(YELLOW "[SCHMIDT-YAW] HEALTH WARNING: %d negative covariance diagonal(s) clamped "
                         "(min_before=%.4e). This must be exactly 0 in a valid run.\n" RESET,
                  neg_clamp_count, min_diag_before_clamp);
  }
  double min_diag_after_update = state->_Cov.diagonal().minCoeff();

  // -- Pss and Pas AFTER update (using same q as before) ----------------------
  Eigen::VectorXd Pq_after = state->_Cov.selfadjointView<Eigen::Upper>() * q;
  double Pss_after          = q.dot(Pq_after);                    // q_old^T P_plus q_old
  Eigen::VectorXd Pas_after_vec = Pq_after - q * Pss_after;
  double Pas_change_norm    = (Pas_after_vec - Pas_before_vec).norm();
  double Pss_change_norm    = std::abs(Pss_after - Pss_before);

  // -- Pss with NEW gauge direction (q rebuilt from updated state) -------------
  // After applying dx_eff the IMU orientation has changed, so the yaw-gauge
  // direction q_new differs from q.  q_new^T P_plus q_new measures whether the
  // updated covariance still has low variance in the NEW gauge direction.
  Eigen::VectorXd Q_full_new = build_global_yaw_gauge_full(state, N, use_fej);
  double norm_Q_new = Q_full_new.norm();
  double Pss_after_new_q = 0.0;
  if (norm_Q_new > 1e-8) {
    Eigen::VectorXd q_new = Q_full_new / norm_Q_new;
    Eigen::VectorXd Pq_new = state->_Cov.selfadjointView<Eigen::Upper>() * q_new;
    Pss_after_new_q = q_new.dot(Pq_new);
  }

  if (Pss_change_norm > 1e-6 * std::max(1.0, std::abs(Pss_before))) {
    PRINT_WARNING(YELLOW "[SCHMIDT-YAW] Pss_change=%.3e (before=%.6f after=%.6f) — Schmidt property violated\n" RESET,
                  Pss_change_norm, Pss_before, Pss_after);
  }

  // -- Post-update diagnostics -----------------------------------------------
  double yaw_after  = extract_imu_yaw_rad(state);
  double bg_z_after = (state->_imu->bg() != nullptr) ? state->_imu->bg()->value()(2) : 0.0;

  if (diag_out) {
    diag_out->normal_dx_s_coeff_before  = normal_dx_s_coeff;
    diag_out->schmidt_dx_s_coeff_after  = schmidt_dx_s_coeff;
    diag_out->norm_dx_normal            = norm_dx_normal;
    diag_out->norm_dx_schmidt           = norm_dx_schmidt;
    diag_out->norm_delta_dx             = norm_delta_dx;
    diag_out->Pss_norm_before           = Pss_before;
    diag_out->Pss_norm_after            = Pss_after;
    diag_out->Pss_norm_after_new_q      = Pss_after_new_q;
    diag_out->Pss_change_norm           = Pss_change_norm;
    diag_out->Pas_change_norm           = Pas_change_norm;
    diag_out->yaw_before_update         = yaw_before * (180.0 / M_PI);
    diag_out->yaw_after_update          = yaw_after  * (180.0 / M_PI);
    diag_out->delta_yaw_update          = (yaw_after - yaw_before) * (180.0 / M_PI);
    diag_out->bg_z_before               = bg_z_before;
    diag_out->bg_z_after                = bg_z_after;
    diag_out->neg_diag_clamp_count      = neg_clamp_count;
    diag_out->min_cov_diag_before_clamp = min_diag_before_clamp;
    diag_out->min_cov_diag_after_update = min_diag_after_update;
    diag_out->projection_applied        = true;
    diag_out->schmidt_applied           = true;
    diag_out->update_type               = update_type;
  }

  // Camera intrinsic calibration sync (mirrors EKFUpdate)
  if (state->_options.do_calib_camera_intrinsics) {
    for (auto const &calib : state->_cam_intrinsics) {
      state->_cam_intrinsics_cameras.at(calib.first)->set_value(calib.second->value());
    }
  }
}

void StateHelper::EKFUpdateYawGaugeHProjectionCurrent(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &res,
    const Eigen::MatrixXd &R) {

  // =========================================================================
  // H-space current-yaw-gauge projection update (mode C).
  //
  // Builds the current-state yaw gauge q restricted to the H_order subspace,
  // then projects: H_eff = H - (H*q_H)*q_H^T so that H_eff*q_H = 0.
  // A standard EKFUpdate (ORIGINAL mode) is then applied with H_eff.
  //
  // Unlike mode B (K-space projection), the measurement Jacobian itself has
  // no yaw component; the visual residual cannot pull the yaw direction at all.
  // S is computed from H_eff, so the innovation covariance also excludes the
  // gauge direction — which is the key difference from mode B where S includes
  // the full H and the residual can still redirect yaw info into other subspaces.
  // =========================================================================

  const int N = (int)state->_Cov.rows();

  // Build H_id
  int cur_it = 0;
  std::vector<int> H_id;
  H_id.reserve(H_order.size());
  for (const auto &v : H_order) { H_id.push_back(cur_it); cur_it += v->size(); }
  const int n_H = cur_it;

  // Full-state gauge → restrict to H_order section
  Eigen::VectorXd Q_full = build_global_yaw_gauge_full(state, N);
  double norm_Q_full = Q_full.norm();

  if (norm_Q_full < 1e-8) {
    PRINT_WARNING(YELLOW "[YAW-H-PROJ] norm_Q_full=%.2e < 1e-8, using standard EKFUpdate\n" RESET, norm_Q_full);
    EKFUpdate(state, H_order, H, res, R, VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
    return;
  }

  // q restricted to H_order columns (same as H_full * q_full since non-H_order H cols = 0)
  Eigen::VectorXd q_H(n_H);
  q_H.setZero();
  for (size_t k = 0; k < H_order.size(); k++) {
    int id_k = H_order[k]->id();
    int sz_k = H_order[k]->size();
    if (id_k >= 0 && id_k + sz_k <= N)
      q_H.segment(H_id[k], sz_k) = Q_full.segment(id_k, sz_k);
  }
  double norm_qH = q_H.norm();

  if (norm_qH < 1e-8) {
    // Gauge has no component in the H_order subspace → no projection possible
    PRINT_WARNING(YELLOW "[YAW-H-PROJ] norm_qH=%.2e in H_order space, fallback\n" RESET, norm_qH);
    EKFUpdate(state, H_order, H, res, R, VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
    return;
  }
  q_H /= norm_qH;   // unit vector in H_order space

  // Project H: H_eff = H - (H*q_H)*q_H^T  →  H_eff * q_H = 0
  Eigen::VectorXd Hq      = H * q_H;   // m×1
  Eigen::MatrixXd H_eff   = H;
  H_eff.noalias()         -= Hq * q_H.transpose();

  // Standard EKF with projected H (ORIGINAL to avoid re-application)
  EKFUpdate(state, H_order, H_eff, res, R, VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
}

void StateHelper::set_vins_nullspace_config(const VinsNullspaceConfig &cfg) {
  g_vins_nullspace_cfg = cfg;
}

// ── Guarded Schmidt (mode D) static state ─────────────────────────────────────
static StateHelper::SchmidtGuardConfig g_guard_cfg;
static std::ofstream g_guard_log_csv;
static bool g_guard_log_header = false;
static std::deque<double> g_guard_burst_times;  // timestamps of recent suspicious updates

void StateHelper::set_schmidt_guard_config(const SchmidtGuardConfig &cfg) {
  g_guard_cfg = cfg;
}

void StateHelper::open_schmidt_guard_log(const std::string &path) {
  if (g_guard_log_csv.is_open()) g_guard_log_csv.close();
  g_guard_log_csv.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!g_guard_log_csv.is_open()) return;
  g_guard_log_csv << "timestamp,update_type,decision,reason,gauge_frac,"
                  << "norm_dx_normal,norm_delta_dx,Pas_est,rel_norm_HQ,"
                  << "H_rows,H_cols,R_scale_applied\n";
  g_guard_log_csv.flush();
  g_guard_log_header = true;
}

void StateHelper::EKFUpdateSchmidtGuarded(
    std::shared_ptr<State> state,
    const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
    const Eigen::MatrixXd &H,
    const Eigen::VectorXd &res,
    const Eigen::MatrixXd &R,
    const std::string &update_type) {

  const int N = (int)state->_Cov.rows();
  const double t_now = state->_timestamp;

  // Build full-state gauge vector and compute pre-update gauge fraction
  Eigen::VectorXd Q_full = build_global_yaw_gauge_full(state, N);
  const double norm_Q = Q_full.norm();
  if (norm_Q < 1e-8) {
    // Degenerate: fall through to normal Schmidt
    SchmidtYawDiag diag; diag.timestamp = t_now; diag.update_type = update_type;
    EKFUpdateSchmidtYawCurrentGauge(state, H_order, H, res, R, update_type, &diag);
    flush_schmidt_yaw_diag(diag);
    return;
  }
  const Eigen::VectorXd q = Q_full / norm_Q;

  // Restrict q to H_order subspace
  int cur_it = 0; std::vector<int> H_id;
  for (const auto &v : H_order) { H_id.push_back(cur_it); cur_it += v->size(); }
  const int n_H = cur_it;
  Eigen::VectorXd q_H(n_H); q_H.setZero();
  for (size_t k = 0; k < H_order.size(); k++) {
    int id_k = H_order[k]->id(), sz_k = H_order[k]->size();
    if (id_k >= 0 && id_k + sz_k <= N)
      q_H.segment(H_id[k], sz_k) = q.segment(id_k, sz_k);
  }
  const double norm_q_H = q_H.norm();
  const double norm_H   = H.norm();

  // M_a = P * H_H^T for computing K and dx_normal (abbreviated pre-computation)
  // We compute a simplified dx_normal = K_std * res to get the gauge coefficient
  // without running the full EKF (avoids duplicate covariance updates).
  // Note: This is a READ-ONLY probe — state is not modified.
  Eigen::MatrixXd M_a(N, H.rows()); M_a.setZero();
  for (size_t i = 0; i < H_order.size(); ++i) {
    int xi = H_order[i]->id(), si = H_order[i]->size();
    if (xi < 0) continue;
    for (size_t j = 0; j < H_order.size(); ++j) {
      int xj = H_order[j]->id(), sj = H_order[j]->size();
      if (xj < 0) continue;
      M_a.block(xi, 0, si, H.rows()) +=
          state->_Cov.block(xi, xj, si, sj) * H.block(0, H_id[j], H.rows(), sj).transpose();
    }
  }
  Eigen::MatrixXd S = H * M_a.block(0, 0, N, H.rows());
  // Only the H_order×H_order block is needed; compute S properly
  Eigen::MatrixXd S_sub(H.rows(), H.rows()); S_sub.setZero();
  for (size_t i = 0; i < H_order.size(); ++i) {
    int xi = H_order[i]->id(), si = H_order[i]->size();
    if (xi < 0) continue;
    S_sub += H.block(0, H_id[i], H.rows(), si) * M_a.block(xi, 0, si, H.rows());
  }
  S_sub += R;

  // dx_normal probe (H_order subspace only, same as the real EKFUpdate)
  Eigen::MatrixXd K_H(n_H, H.rows()); K_H.setZero();
  for (size_t i = 0; i < H_order.size(); ++i) {
    int xi = H_order[i]->id(), si = H_order[i]->size();
    if (xi < 0) continue;
    K_H.block(H_id[i], 0, si, H.rows()) = M_a.block(xi, 0, si, H.rows());
  }
  Eigen::VectorXd dx_H = K_H * S_sub.ldlt().solve(res);  // dx in H_order space

  const double norm_dx_n   = dx_H.norm();
  const double coeff        = (norm_q_H > 1e-8) ? q_H.dot(dx_H) : 0.0;
  const double gauge_frac   = (norm_dx_n > 1e-9) ? std::fabs(coeff) / norm_dx_n : 0.0;
  const double norm_ddx     = std::fabs(coeff) * norm_q_H;  // approx norm(coeff*q_H)
  const double rel_HQ       = (norm_H > 1e-12 && norm_q_H > 1e-8)
                              ? (H * q_H / norm_q_H).norm() / norm_H : 0.0;

  // ── Guard decision ──────────────────────────────────────────────────────
  const auto &gc = g_guard_cfg;

  // Purge stale burst timestamps
  while (!g_guard_burst_times.empty() && t_now - g_guard_burst_times.front() > gc.burst_window_s)
    g_guard_burst_times.pop_front();

  const bool mild_suspicious = (gauge_frac > gc.gauge_frac_mild && norm_ddx > gc.norm_dx_mild);
  const bool severe_suspicious = (gauge_frac > gc.gauge_frac_severe && norm_ddx > gc.norm_dx_severe);
  const bool burst_active = (int)g_guard_burst_times.size() >= gc.burst_count;

  std::string decision = "accept";
  std::string reason   = "";
  double r_scale = 1.0;

  if (severe_suspicious || (mild_suspicious && burst_active)) {
    if (gc.reject_on_severe) {
      decision = "reject"; reason = severe_suspicious ? "severe" : "burst";
    } else {
      decision = "inflate_R_severe"; reason = severe_suspicious ? "severe" : "burst";
      r_scale = gc.r_scale_severe;
    }
    g_guard_burst_times.push_back(t_now);
  } else if (mild_suspicious) {
    decision = "inflate_R_mild"; reason = "mild";
    r_scale = gc.r_scale_mild;
    g_guard_burst_times.push_back(t_now);
  }

  // Log guard decision
  if (g_guard_log_csv.is_open()) {
    g_guard_log_csv << std::fixed << std::setprecision(9)
      << t_now << "," << update_type << "," << decision << "," << reason << ","
      << std::setprecision(6) << gauge_frac << "," << norm_dx_n << ","
      << norm_ddx << ",0.0," << rel_HQ << ","
      << H.rows() << "," << n_H << "," << r_scale << "\n";
    g_guard_log_csv.flush();
  }

  if (decision == "reject") {
    PRINT_DEBUG(YELLOW "[GUARD] t=%.3f  %s  REJECTED  gauge_frac=%.3f  norm_ddx=%.3f  reason=%s\n" RESET,
                t_now, update_type.c_str(), gauge_frac, norm_ddx, reason.c_str());
    return;
  }

  // Apply (possibly R-inflated) Schmidt update
  Eigen::MatrixXd R_eff = (r_scale > 1.0 + 1e-9) ? R * r_scale : R;
  if (r_scale > 1.0 + 1e-9) {
    PRINT_DEBUG(YELLOW "[GUARD] t=%.3f  %s  R*=%.0f  gauge_frac=%.3f  norm_ddx=%.3f  reason=%s\n" RESET,
                t_now, update_type.c_str(), r_scale, gauge_frac, norm_ddx, reason.c_str());
  }

  SchmidtYawDiag diag; diag.timestamp = t_now; diag.update_type = update_type;
  EKFUpdateSchmidtYawCurrentGauge(state, H_order, H, res, R_eff, update_type, &diag);
  flush_schmidt_yaw_diag(diag);
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

void StateHelper::inject_pz_noise(std::shared_ptr<State> state, double noise) {
  int global_pz = state->_imu->id() + 5; // q(3) + p_z(idx=2)
  state->_Cov(global_pz, global_pz) += noise;
}

void StateHelper::inject_h_offset_noise(std::shared_ptr<State> state, double noise) {
  if (!state->_h_offset || state->_h_offset->id() < 0) return;
  state->_Cov(state->_h_offset->id(), state->_h_offset->id()) += noise;
}

void StateHelper::inject_flex_yaw_noise(std::shared_ptr<State> state,
                                        double noise) {
  if (!state->_flex_yaw || state->_flex_yaw->id() < 0 ||
      !std::isfinite(noise) || noise <= 0.0)
    return;
  state->_Cov(state->_flex_yaw->id(), state->_flex_yaw->id()) += noise;
}

void StateHelper::ekf_update_zonly(std::shared_ptr<State> state, double R) {
  int pz_global = state->_imu->p()->id() + 2; // p_z = 3rd element of IMU position
  double P_pz = state->_Cov(pz_global, pz_global);
  double S = P_pz + R;
  state->_Cov(pz_global, pz_global) -= (P_pz * P_pz) / S;
}

void StateHelper::EKFUpdateZOnly(std::shared_ptr<State> state,
                                 const std::vector<std::shared_ptr<Type>> &H_order,
                                 const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
                                 const Eigen::MatrixXd &R) {

  // Save pre-update IMU position (to extract p_z correction later)
  Eigen::Vector3d p_pre = state->_imu->pos();

  // Save pre-update values for all state variables
  std::vector<Eigen::VectorXd> pre_vals;
  for (const auto &var : state->_variables) {
    pre_vals.push_back(var->value());
  }

  // Full standard EKF update (consistent covariance + state)
  EKFUpdate(state, H_order, H, res, R);

  // Record post-update p_z
  double p_z_new = state->_imu->pos()(2);

  // Revert ALL state variables to pre-update values
  for (size_t i = 0; i < state->_variables.size(); i++) {
    state->_variables[i]->set_value(pre_vals[i]);
  }

  // Apply only the p_z correction (covariance remains fully updated)
  Eigen::Vector3d p_final = p_pre;
  p_final(2) = p_z_new;
  state->_imu->p()->set_value(p_final);
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
                             double visual_global_yaw_oc_alpha, VisualOcFn oc_fn) {

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
                             VisualYawUpdateMode::ORIGINAL, 1.0, 0.0);
    } else {
      StateHelper::EKFUpdate(state, H_order, Hup, resup, Rup, visual_yaw_update_mode,
                             visual_yaw_update_scale, visual_global_yaw_oc_alpha);
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

  if (state->_options.use_flex_yaw_state) {
    std::shared_ptr<Type> flextemp =
        StateHelper::clone(state, state->_flex_yaw);
    std::shared_ptr<Vec> flex = std::dynamic_pointer_cast<Vec>(flextemp);
    if (flex == nullptr) {
      PRINT_ERROR(RED "INVALID FLEX OBJECT RETURNED FROM STATEHELPER CLONE\n" RESET);
      std::exit(EXIT_FAILURE);
    }
    state->_clones_flex_yaw[state->_timestamp] = flex;
  }

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
    auto flex_it = state->_clones_flex_yaw.find(marginal_time);
    if (flex_it != state->_clones_flex_yaw.end()) {
      StateHelper::marginalize(state, flex_it->second);
      state->_clones_flex_yaw.erase(flex_it);
    }
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
