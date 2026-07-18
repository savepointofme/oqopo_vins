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

#include "VioManager.h"

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "types/LandmarkRepresentation.h"
#include "utils/print.h"

#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"

#include <Eigen/Eigenvalues>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

void VioManager::initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate) {

  // Initialize the system
  state->_imu->set_value(imustate.block(1, 0, 16, 1));
  state->_imu->set_fej(imustate.block(1, 0, 16, 1));

  // Fix the global yaw and position gauge freedoms
  // TODO: Why does this break out simulation consistency metrics?
  std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu};
  Eigen::MatrixXd Cov = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(state->_imu->size(), state->_imu->size());
  Cov.block(0, 0, 3, 3) = std::pow(0.017, 2) * Eigen::Matrix3d::Identity(); // q
  Cov.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity();  // p
  Cov.block(6, 6, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();  // v (static)
  StateHelper::set_initial_covariance(state, Cov, order);

  // Set the state time
  state->_timestamp = imustate(0, 0);
  startup_time = imustate(0, 0);
  is_initialized_vio = true;

  // Cleanup any features older then the initialization time
  trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
  }

  // Print what we init'ed with
  PRINT_DEBUG(GREEN "[INIT]: INITIALIZED FROM GROUNDTRUTH FILE!!!!!\n" RESET);
  PRINT_DEBUG(GREEN "[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET, state->_imu->quat()(0), state->_imu->quat()(1),
              state->_imu->quat()(2), state->_imu->quat()(3));
  PRINT_DEBUG(GREEN "[INIT]: bias gyro = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_g()(0), state->_imu->bias_g()(1),
              state->_imu->bias_g()(2));
  PRINT_DEBUG(GREEN "[INIT]: velocity = %.4f, %.4f, %.4f\n" RESET, state->_imu->vel()(0), state->_imu->vel()(1), state->_imu->vel()(2));
  PRINT_DEBUG(GREEN "[INIT]: bias accel = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_a()(0), state->_imu->bias_a()(1),
              state->_imu->bias_a()(2));
  PRINT_DEBUG(GREEN "[INIT]: position = %.4f, %.4f, %.4f\n" RESET, state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));
}

void VioManager::initialize_with_fc_state(const FCInitState &fc,
                                          double sigma_att_rad,
                                          double sigma_vel,
                                          double sigma_pos,
                                          double sigma_bg,
                                          double sigma_ba,
                                          double camera_timestamp) {

  if (is_initialized_vio) {
    PRINT_WARNING(YELLOW "[FC-INIT]: requested but VIO is already initialized; ignoring\n" RESET);
    return;
  }

  Eigen::Matrix<double, 16, 1> imu_state;
  imu_state.setZero();
  imu_state.block(0, 0, 4, 1) = fc.q_GtoI;
  imu_state.block(4, 0, 3, 1) = fc.p_IinG;
  imu_state.block(7, 0, 3, 1) = fc.v_IinG;
  imu_state.block(10, 0, 3, 1) = fc.bg;
  imu_state.block(13, 0, 3, 1) = fc.ba;
  state->_imu->set_value(imu_state);
  state->_imu->set_fej(imu_state);

  std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu};
  Eigen::MatrixXd Cov = Eigen::MatrixXd::Zero(state->_imu->size(), state->_imu->size());
  Cov.block(0, 0, 3, 3) = std::pow(sigma_att_rad, 2) * Eigen::Matrix3d::Identity();
  Cov.block(3, 3, 3, 3) = std::pow(sigma_pos, 2) * Eigen::Matrix3d::Identity();
  Cov.block(6, 6, 3, 3) = std::pow(sigma_vel, 2) * Eigen::Matrix3d::Identity();
  Cov.block(9, 9, 3, 3) = std::pow(sigma_bg, 2) * Eigen::Matrix3d::Identity();
  Cov.block(12, 12, 3, 3) = std::pow(sigma_ba, 2) * Eigen::Matrix3d::Identity();
  StateHelper::set_initial_covariance(state, Cov, order);

  // The ros-free runner trims IMU samples before --start-time. The first
  // camera after trimming is usually a few tens of ms after the requested
  // start time, while the FC state row is exactly at --start-time. The runner
  // uses this first camera only to seed the EKF and then feeds the next camera
  // normally, so the first propagated interval has real IMU coverage.
  double init_timestamp = camera_timestamp;
  state->_timestamp = init_timestamp;
  startup_time = init_timestamp;
  is_initialized_vio = true;
  thread_init_success = true;
  thread_init_running = false;
  has_moved_since_zupt = (state->_imu->vel().norm() > params.zupt_max_velocity);

  trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
  trackFEATS->set_num_features(std::floor((double)params.num_pts / (double)params.state_options.num_cameras));
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
  }
  camera_queue_init.clear();
  propagator->invalidate_cache();

  PRINT_INFO(GREEN "[FC-INIT]: FC-assisted VIO initialization, no continuous GPS/FC fusion\n" RESET);
  PRINT_INFO(GREEN "[FC-INIT]: DynamicInitializer bypassed: yes\n" RESET);
  PRINT_INFO(GREEN "[FC-INIT]: EKF init timestamp %.6f, FC state timestamp %.6f, camera timestamp %.6f, dt_fc_to_camera %.6f\n" RESET,
             init_timestamp, fc.timestamp, camera_timestamp, fc.timestamp - camera_timestamp);
  PRINT_INFO(GREEN "[FC-INIT]: q_GtoI = %.6f, %.6f, %.6f, %.6f\n" RESET,
             state->_imu->quat()(0), state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3));
  PRINT_INFO(GREEN "[FC-INIT]: p_IinG = %.3f, %.3f, %.3f\n" RESET,
             state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));
  PRINT_INFO(GREEN "[FC-INIT]: v_IinG = %.3f, %.3f, %.3f | speed = %.3f m/s\n" RESET,
             state->_imu->vel()(0), state->_imu->vel()(1), state->_imu->vel()(2), state->_imu->vel().norm());
  PRINT_INFO(GREEN "[FC-INIT]: bg = %.5f, %.5f, %.5f | ba = %.5f, %.5f, %.5f\n" RESET,
             state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));
  PRINT_INFO(GREEN "[FC-INIT]: P diag attitude=%.6g velocity=%.6g position=%.6g bg=%.6g ba=%.6g\n" RESET,
             std::pow(sigma_att_rad, 2), std::pow(sigma_vel, 2), std::pow(sigma_pos, 2),
             std::pow(sigma_bg, 2), std::pow(sigma_ba, 2));
}

void VioManager::initialize_with_online_alignment(
    const OnlineAlignmentResult &result) {
  if (is_initialized_vio) {
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN] release requested after initialization; ignoring\n" RESET);
    return;
  }
  if (!result.diagnostics.quality_passed ||
      result.diagnostics.future_data_used || !result.q_GtoI.allFinite() ||
      !result.p_IinG.allFinite() || !result.v_IinG.allFinite() ||
      !result.bg.allFinite() || !result.ba.allFinite() ||
      !result.covariance.allFinite()) {
    PRINT_ERROR(RED "[ONLINE-ALIGN] invalid or non-causal release rejected\n" RESET);
    return;
  }
  OnlineAlignmentResult applied_result = result;
  Eigen::Matrix<double, 15, 15> release_covariance =
      online_alignment_release_covariance_override_enabled_
          ? online_alignment_release_covariance_override_
          : result.covariance;
  const auto covariance_std = [](const Eigen::Matrix<double, 15, 15> &P) {
    return P.diagonal().cwiseMax(0.0).cwiseSqrt();
  };
  applied_result.diagnostics.handoff_raw_covariance_std =
      covariance_std(result.covariance);

  const bool verified_formal_candidate_release =
      result.diagnostics.formal_candidate_holdout_passed &&
      !result.diagnostics.formal_refinement_release;
  const bool apply_native_handoff_inflation =
      result.diagnostics.direct_sliding_state_release &&
      !online_alignment_release_covariance_override_enabled_;
  if (apply_native_handoff_inflation) {
    const double orientation_inflation =
        params.init_options.init_dyn_inflation_orientation;
    const double velocity_inflation =
        params.init_options.init_dyn_inflation_velocity;
    const double gyro_bias_inflation =
        params.init_options.init_dyn_inflation_bias_gyro;
    const double accel_bias_inflation =
        params.init_options.init_dyn_inflation_bias_accel;
    std::string inflation_failure;
    Eigen::MatrixXd active_covariance = release_covariance;
    if (!StateHelper::inflate_initial_imu_subspace_covariance(
            active_covariance, orientation_inflation, velocity_inflation,
            gyro_bias_inflation, accel_bias_inflation,
            &inflation_failure)) {
      PRINT_ERROR(RED
                  "[ONLINE-ALIGN] terminal covariance handoff inflation "
                  "rejected: %s\n"
                  RESET,
                  inflation_failure.c_str());
      return;
    }
    release_covariance = active_covariance.block<15, 15>(0, 0);
    applied_result.diagnostics.handoff_covariance_inflation_applied = true;
    applied_result.diagnostics.handoff_covariance_model =
        "openvins_dynamic_initializer_terminal_state";
    applied_result.diagnostics.handoff_covariance_inflation = {
        orientation_inflation, velocity_inflation, gyro_bias_inflation,
        accel_bias_inflation};
    PRINT_INFO(
        GREEN
        "[ONLINE-ALIGN] direct terminal-state handoff with upstream covariance "
        "inflation [att %.1f vel %.1f bg %.1f ba %.1f]\n"
        RESET,
        orientation_inflation, velocity_inflation, gyro_bias_inflation,
        accel_bias_inflation);
  } else if (verified_formal_candidate_release &&
             !online_alignment_release_covariance_override_enabled_) {
    // The formal joint graph already returns the Schur marginal of its
    // terminal q/p/v/bg/ba state. Applying the generic dynamic-initializer
    // block inflation here would replace that graph posterior with a second,
    // unrelated uncertainty model (in particular velocity x100). Preserve the
    // estimator-derived marginal and expose that choice in the run receipt.
    applied_result.diagnostics.handoff_covariance_model =
        "formal_joint_terminal_schur_marginal";
    applied_result.diagnostics.handoff_covariance_inflation_applied = false;
    applied_result.diagnostics.handoff_covariance_inflation = {1.0, 1.0, 1.0,
                                                               1.0};
    PRINT_INFO(
        GREEN
        "[ONLINE-ALIGN] formal terminal-state handoff preserves joint-graph "
        "Schur marginal covariance\n"
        RESET);
  } else if (online_alignment_release_covariance_override_enabled_) {
    applied_result.diagnostics.handoff_covariance_model =
        "diagnostic_cli_diagonal_no_history";
  }
  applied_result.covariance = release_covariance;
  applied_result.initial_clones.clear();
  applied_result.initial_landmarks.clear();
  applied_result.startup_consumed_feature_ids.clear();
  applied_result.initial_joint_covariance.resize(0, 0);
  applied_result.diagnostics.initial_history_clone_count = 0;
  applied_result.diagnostics.initial_persistent_landmark_count = 0;
  applied_result.diagnostics.initial_joint_covariance_dimension = 0;
  applied_result.diagnostics.initial_history_covariance_recovered = false;
  applied_result.diagnostics.handoff_applied_covariance_std =
      covariance_std(release_covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eig(
      release_covariance);
  if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0.0) {
    PRINT_ERROR(RED "[ONLINE-ALIGN] non-positive initial covariance rejected\n" RESET);
    return;
  }

  Eigen::Matrix<double, 16, 1> imu_state;
  const Eigen::Vector3d estimator_position =
      online_alignment_local_estimator_origin_enabled_
          ? Eigen::Vector3d::Zero()
          : result.p_IinG;
  imu_state << result.q_GtoI, estimator_position, result.v_IinG, result.bg,
      result.ba;
  state->_imu->set_value(imu_state);
  state->_imu->set_fej(imu_state);
  std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu};
  StateHelper::set_initial_covariance(state, release_covariance, order);
  state->_timestamp = result.timestamp;

  startup_time = result.timestamp;
  // Match the standard OpenVINS dynamic-initializer boundary: the graph is an
  // initializer, not an already-running EKF. Only its terminal IMU state and
  // 15-D marginal enter the filter; normal OpenVINS propagation rebuilds clone
  // history from subsequent causal camera frames.
  trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
  trackFEATS->set_num_features(std::floor(
      static_cast<double>(params.num_pts) /
      static_cast<double>(params.state_options.num_cameras)));
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup_measurements(
        state->_timestamp);
  }
  camera_queue_init.clear();
  propagator->invalidate_cache();
  has_moved_since_zupt = state->_imu->vel().norm() > params.zupt_max_velocity;
  thread_init_success = true;
  thread_init_running = false;
  is_initialized_vio = true;
  online_alignment_result_ = applied_result;
  online_alignment_result_valid_ = true;

  PRINT_INFO(GREEN
             "[ONLINE-ALIGN] released terminal q/p/v/bg/ba only; graph clones, "
             "landmarks, and joint history covariance were not injected\n"
             RESET);

  if (online_alignment_local_estimator_origin_enabled_) {
    PRINT_INFO(GREEN
               "[ONLINE-ALIGN][LOCAL-W0] estimator p_W0=[0 0 0], "
               "fixed output p_W0inGnav=[%.6f %.6f %.6f]; q/v/bg/ba and "
               "15x15 covariance unchanged\n" RESET,
               result.p_IinG.x(), result.p_IinG.y(), result.p_IinG.z());
  }

  if (post_alignment_camera_extrinsic_rotation_enabled_) {
    const auto calibration_it = state->_calib_IMUtoCAM.find(0);
    if (calibration_it == state->_calib_IMUtoCAM.end() ||
        calibration_it->second == nullptr) {
      PRINT_ERROR(RED "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] camera 0 calibration missing\n" RESET);
    } else {
      const Eigen::Matrix3d R_old = calibration_it->second->Rot();
      const Eigen::Matrix3d R_new =
          ov_core::exp_so3(post_alignment_camera_extrinsic_left_rotvec_rad_) *
          R_old;
      Eigen::Matrix<double, 7, 1> value = calibration_it->second->value();
      value.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_new);
      calibration_it->second->set_value(value);
      calibration_it->second->set_fej(value);
      propagator->invalidate_cache();
      PRINT_INFO(CYAN "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] post-P4 only; "
                      "left rotvec_deg=[%+.6f %+.6f %+.6f], norm=%.6fdeg\n" RESET,
                 post_alignment_camera_extrinsic_left_rotvec_rad_.x() * 180.0 / M_PI,
                 post_alignment_camera_extrinsic_left_rotvec_rad_.y() * 180.0 / M_PI,
                 post_alignment_camera_extrinsic_left_rotvec_rad_.z() * 180.0 / M_PI,
                 post_alignment_camera_extrinsic_left_rotvec_rad_.norm() * 180.0 / M_PI);
      PRINT_INFO(CYAN "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] old_R_ItoC=[%.9f %.9f %.9f; %.9f %.9f %.9f; %.9f %.9f %.9f]\n" RESET,
                 R_old(0, 0), R_old(0, 1), R_old(0, 2),
                 R_old(1, 0), R_old(1, 1), R_old(1, 2),
                 R_old(2, 0), R_old(2, 1), R_old(2, 2));
      PRINT_INFO(CYAN "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] new_R_ItoC=[%.9f %.9f %.9f; %.9f %.9f %.9f; %.9f %.9f %.9f]\n" RESET,
                 R_new(0, 0), R_new(0, 1), R_new(0, 2),
                 R_new(1, 0), R_new(1, 1), R_new(1, 2),
                 R_new(2, 0), R_new(2, 1), R_new(2, 2));
    }
  }

  if (online_alignment_release_covariance_override_enabled_) {
    const Eigen::Matrix<double, 15, 1> source_diag =
        result.covariance.diagonal();
    const Eigen::Matrix<double, 15, 1> applied_diag =
        release_covariance.diagonal();
    PRINT_INFO(CYAN "[ONLINE-ALIGN][COV-ABLATION] P4 nominal state and release timestamp unchanged; "
                    "replaced only initial covariance\n" RESET);
    PRINT_INFO(CYAN "[ONLINE-ALIGN][COV-ABLATION] source std=[att %.6g pos %.6g vel %.6g bg %.6g ba %.6g], "
                    "applied std=[att %.6g pos %.6g vel %.6g bg %.6g ba %.6g]\n" RESET,
               std::sqrt(source_diag.segment<3>(0).mean()),
               std::sqrt(source_diag.segment<3>(3).mean()),
               std::sqrt(source_diag.segment<3>(6).mean()),
               std::sqrt(source_diag.segment<3>(9).mean()),
               std::sqrt(source_diag.segment<3>(12).mean()),
               std::sqrt(applied_diag.segment<3>(0).mean()),
               std::sqrt(applied_diag.segment<3>(3).mean()),
               std::sqrt(applied_diag.segment<3>(6).mean()),
               std::sqrt(applied_diag.segment<3>(9).mean()),
               std::sqrt(applied_diag.segment<3>(12).mean()));
  }

  PRINT_INFO(GREEN "[ONLINE-ALIGN] RELEASED causal state at t=%.6f, solve_t=%.6f, window=[%.6f, %.6f]\n" RESET,
             result.timestamp, result.diagnostics.solve_time,
             result.diagnostics.window_start, result.diagnostics.init_time);
  PRINT_INFO(GREEN "[ONLINE-ALIGN] locked dt_att=%+.6fs dt_nav=%+.6fs mount_residual=%.3fdeg rate_rms=%.4frad/s visual_imu=%.3fdeg\n" RESET,
             result.fc_attitude_to_board_time_offset_s,
             result.fc_navigation_to_board_time_offset_s,
             result.diagnostics.mount_residual_deg,
             result.diagnostics.rate_residual_rms_rad_s,
             result.diagnostics.visual_imu_rotation_residual_deg);
  PRINT_INFO(GREEN "[ONLINE-ALIGN] release closed-loop updates=%d visual=%d correction=[%.3fdeg %.3fm %.3fm/s %.5frad/s %.4fm/s2], Joseph/reset=%s/%s; FC yaw gauge factors=%d\n" RESET,
             result.diagnostics.candidate_closed_loop_update_count,
             result.diagnostics.candidate_closed_loop_visual_update_count,
             result.diagnostics.candidate_closed_loop_attitude_correction_deg,
             result.diagnostics.candidate_closed_loop_position_correction_m,
             result.diagnostics.candidate_closed_loop_velocity_correction_mps,
             result.diagnostics.candidate_closed_loop_gyro_bias_correction_rad_s,
             result.diagnostics.candidate_closed_loop_accel_bias_correction_mps2,
             result.diagnostics.candidate_covariance_joseph_update_applied ? "yes" : "no",
             result.diagnostics.candidate_covariance_error_reset_applied ? "yes" : "no",
             result.diagnostics.fc_attitude_gauge_factor_count);
  PRINT_INFO(GREEN "[ONLINE-ALIGN] bg=[%.5f %.5f %.5f] ba=[%.5f %.5f %.5f], no continuous FC/GPS fusion\n" RESET,
             result.bg.x(), result.bg.y(), result.bg.z(), result.ba.x(),
              result.ba.y(), result.ba.z());
}

bool VioManager::process_online_alignment_shadow_window(
    const OnlineAlignmentResult &window_result) {
  if (!is_initialized_vio || state == nullptr || state->_imu == nullptr ||
      propagator == nullptr || online_alignment_initializer_ == nullptr ||
      (!online_alignment_initializer_->sliding_window_shadow_only() &&
       !online_alignment_initializer_->upstream_dynamic_init_fc_gauge()) ||
      online_alignment_result_valid_)
    return false;

  const bool low_dimensional_target =
      online_alignment_initializer_->upstream_dynamic_init_fc_gauge();
  const bool window_quality =
      (low_dimensional_target || window_result.diagnostics.solver_converged) &&
      window_result.diagnostics.navigation_quality_passed &&
      !window_result.diagnostics.future_data_used &&
      window_result.q_GtoI.allFinite() &&
      window_result.p_IinG.allFinite() &&
      window_result.covariance.allFinite();
  if (!window_quality) {
    online_alignment_gauge_aligner_.reset();
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] t=%.6f rejected by target quality; time window reset\n" RESET,
                  window_result.timestamp);
    return false;
  }
  if (!std::isfinite(window_result.timestamp)) {
    online_alignment_gauge_aligner_.reset();
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] target timestamp is non-finite\n" RESET);
    return false;
  }

  OnlineAlignmentLocalStateSnapshot current_snapshot;
  current_snapshot.timestamp = state->_timestamp;
  current_snapshot.q_GtoI = state->_imu->quat();
  current_snapshot.p_IinG = state->_imu->pos();
  current_snapshot.v_IinG = state->_imu->vel();
  if (online_alignment_local_state_history_.empty() ||
      current_snapshot.timestamp >
          online_alignment_local_state_history_.back().timestamp + 1.0e-9) {
    online_alignment_local_state_history_.push_back(current_snapshot);
  } else if (std::fabs(current_snapshot.timestamp -
                       online_alignment_local_state_history_.back().timestamp) <=
             1.0e-9) {
    online_alignment_local_state_history_.back() = current_snapshot;
  } else {
    online_alignment_local_state_history_.clear();
    online_alignment_local_state_history_.push_back(current_snapshot);
    online_alignment_gauge_aligner_.reset();
  }
  const double history_keep_after =
      state->_timestamp - online_alignment_gauge_aligner_.config().window_duration_s -
      online_alignment_gauge_aligner_.config().maximum_observation_gap_s - 1.0;
  while (online_alignment_local_state_history_.size() > 2 &&
         online_alignment_local_state_history_[1].timestamp <
             history_keep_after)
    online_alignment_local_state_history_.pop_front();

  const Eigen::MatrixXd local_covariance =
      StateHelper::get_marginal_covariance(state, {state->_imu});
  if (local_covariance.rows() != 15 || local_covariance.cols() != 15 ||
      !local_covariance.allFinite() ||
      (local_covariance.diagonal().array() < -1.0e-10).any()) {
    online_alignment_gauge_aligner_.reset();
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] local VIO covariance is invalid; time window reset\n" RESET);
    return false;
  }
  const Eigen::MatrixXd symmetric_covariance =
      0.5 * (local_covariance + local_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> covariance_solver(
      symmetric_covariance, Eigen::EigenvaluesOnly);
  if (covariance_solver.info() != Eigen::Success ||
      covariance_solver.eigenvalues().minCoeff() < -1.0e-8) {
    online_alignment_gauge_aligner_.reset();
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] local VIO covariance is not positive semidefinite; time window reset\n" RESET);
    return false;
  }

  Eigen::Vector4d provisional_q = state->_imu->quat();
  Eigen::Vector3d provisional_p = state->_imu->pos();
  Eigen::Vector3d provisional_v = state->_imu->vel();
  bool local_state_available =
      std::fabs(window_result.timestamp - state->_timestamp) <= 1.0e-9;
  if (window_result.timestamp > state->_timestamp + 1.0e-9) {
    Eigen::Matrix<double, 13, 1> propagated_state;
    Eigen::Matrix<double, 12, 12> propagated_covariance;
    if (!propagator->fast_state_propagate(
            state, window_result.timestamp, propagated_state,
            propagated_covariance)) {
      online_alignment_gauge_aligner_.reset();
      PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] no causal provisional propagation to t=%.6f\n" RESET,
                    window_result.timestamp);
      return false;
    }
    provisional_q = propagated_state.head<4>();
    provisional_p = propagated_state.segment<3>(4);
    provisional_v = propagated_state.segment<3>(7);
    local_state_available = true;
  } else if (window_result.timestamp < state->_timestamp - 1.0e-9) {
    for (std::size_t index = 0;
         index < online_alignment_local_state_history_.size(); ++index) {
      const auto &snapshot = online_alignment_local_state_history_[index];
      if (std::fabs(snapshot.timestamp - window_result.timestamp) <= 1.0e-9) {
        provisional_q = snapshot.q_GtoI;
        provisional_p = snapshot.p_IinG;
        provisional_v = snapshot.v_IinG;
        local_state_available = true;
        break;
      }
      if (index == 0 || snapshot.timestamp < window_result.timestamp)
        continue;
      const auto &before = online_alignment_local_state_history_[index - 1];
      if (before.timestamp > window_result.timestamp)
        break;
      const double span = snapshot.timestamp - before.timestamp;
      if (!(span > 0.0))
        break;
      const double alpha =
          (window_result.timestamp - before.timestamp) / span;
      Eigen::Quaterniond q_before(ov_core::quat_2_Rot(before.q_GtoI));
      Eigen::Quaterniond q_after(ov_core::quat_2_Rot(snapshot.q_GtoI));
      if (q_before.dot(q_after) < 0.0)
        q_after.coeffs() *= -1.0;
      provisional_q = ov_core::rot_2_quat(
          q_before.slerp(alpha, q_after).normalized().toRotationMatrix());
      provisional_p =
          (1.0 - alpha) * before.p_IinG + alpha * snapshot.p_IinG;
      provisional_v =
          (1.0 - alpha) * before.v_IinG + alpha * snapshot.v_IinG;
      local_state_available = true;
      break;
    }
  }
  if (!local_state_available) {
    PRINT_INFO(CYAN "[ONLINE-ALIGN][GAUGE-WINDOW] waiting for local VIO history bracket at t=%.6f (history %.6f..%.6f, active %.6f)\n" RESET,
               window_result.timestamp,
               online_alignment_local_state_history_.empty()
                   ? -1.0
                   : online_alignment_local_state_history_.front().timestamp,
               online_alignment_local_state_history_.empty()
                   ? -1.0
                   : online_alignment_local_state_history_.back().timestamp,
               state->_timestamp);
    return false;
  }

  const Eigen::Matrix3d provisional_R_ItoG =
      ov_core::quat_2_Rot(provisional_q).transpose();
  const Eigen::Matrix3d target_R_ItoG =
      ov_core::quat_2_Rot(window_result.q_GtoI).transpose();
  const Eigen::Matrix3d gauge_rotation_observation =
      target_R_ItoG * provisional_R_ItoG.transpose();
  const double gauge_yaw =
      std::atan2(gauge_rotation_observation(1, 0),
                 gauge_rotation_observation(0, 0));
  const double target_yaw_variance = std::max(
      0.0, window_result.covariance.block<3, 3>(0, 0)
               .diagonal()
               .maxCoeff());
  const double local_yaw_variance = std::max(
      0.0, local_covariance.block<3, 3>(0, 0).diagonal().maxCoeff());
  OnlineVioFcGaugeObservation observation;
  observation.timestamp = window_result.timestamp;
  observation.yaw_rad = gauge_yaw;
  observation.yaw_sigma_deg =
      std::sqrt(target_yaw_variance + local_yaw_variance) * 180.0 / M_PI;
  observation.target_position = window_result.p_IinG;
  observation.target_velocity = window_result.v_IinG;
  observation.local_position = provisional_p;
  observation.local_velocity = provisional_v;
  std::string push_reason;
  if (!online_alignment_gauge_aligner_.push(observation, &push_reason)) {
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE-WINDOW] t=%.6f observation rejected: %s\n" RESET,
                  window_result.timestamp, push_reason.c_str());
    return false;
  }
  ++online_alignment_gauge_window_count_;
  const OnlineVioFcGaugeEstimate estimate =
      online_alignment_gauge_aligner_.estimate();

  PRINT_INFO(CYAN "[ONLINE-ALIGN][GAUGE-WINDOW] t=%.6f n=%d duration=%.3f/%.3fs gap=%.3fs yaw=%+.3fdeg yaw_mad=%.3fdeg unit_vfit_diag=%.3fm/s translation_rmse_diag=%.3fm decision=%s\n" RESET,
             window_result.timestamp, estimate.observation_count,
             estimate.covered_duration_s,
             online_alignment_gauge_aligner_.config().window_duration_s,
             estimate.maximum_observation_gap_s,
             estimate.yaw_rad * 180.0 / M_PI, estimate.yaw_mad_deg,
             estimate.unit_scale_velocity_rmse_mps,
             estimate.translation_rmse_m, estimate.reason.c_str());
  if (!estimate.ready)
    return false;
  if (online_alignment_options_.sliding_window_diagnostic_never_anchor) {
    PRINT_INFO(CYAN "[ONLINE-ALIGN][DIAGNOSTIC] gauge is ready but the "
                    "anchor is intentionally disabled; provisional VIO "
                    "continues unchanged\n" RESET);
    return false;
  }

  OnlineAlignmentResult release = window_result;
  // The similarity was estimated from a historical causal window, but it is a
  // global coordinate transform. Apply it atomically to the current active
  // state instead of pretending that the active state still lives at the last
  // FC sample time.
  release.timestamp = state->_timestamp;
  release.released_to_openvins = true;
  release.readiness = AlignmentReadiness::NAVIGATION_READY;
  release.diagnostics.readiness = release.readiness;
  release.diagnostics.quality_passed = true;
  release.diagnostics.decision_time =
      window_result.diagnostics.solve_time;
  release.diagnostics.readiness_reason =
      "upstream_dynamic_init_then_causal_vio_fc_yaw_translation_window_scale_deferred_to_agl_output";
  release.diagnostics.gauge_window_observation_count =
      online_alignment_gauge_window_count_;
  release.diagnostics.gauge_stability_required_s =
      online_alignment_gauge_aligner_.config().window_duration_s;
  release.diagnostics.gauge_stable_duration_s =
      estimate.covered_duration_s;
  release.diagnostics.gauge_yaw_delta_from_previous_deg =
      estimate.yaw_mad_deg;
  release.diagnostics.gauge_translation_delta_from_previous_m =
      estimate.translation_rmse_m;
  release.diagnostics.gauge_yaw_consistency_limit_deg =
      online_alignment_gauge_aligner_.config().maximum_yaw_mad_deg;
  release.diagnostics.gauge_translation_consistency_limit_m =
      std::numeric_limits<double>::infinity();
  release.diagnostics.gauge_metric_scale = 1.0;
  release.diagnostics.gauge_metric_scale_observed =
      estimate.observed_metric_scale;
  release.diagnostics.gauge_metric_scale_sigma =
      estimate.observed_metric_scale_sigma;
  release.diagnostics.gauge_velocity_fit_rmse_mps =
      estimate.unit_scale_velocity_rmse_mps;
  release.diagnostics.gauge_velocity_excitation_mps = 0.0;
  release.diagnostics.gauge_yaw_reset_deg =
      estimate.yaw_rad * 180.0 / M_PI;
  release.diagnostics.gauge_translation_G = estimate.translation_G;
  release.diagnostics.gauge_window_quality_passed = true;
  release.diagnostics.gauge_consistency_passed = true;
  if (!anchor_with_online_alignment(release))
    return false;
  if (!online_alignment_initializer_->commit_shadow_gauge_release(release)) {
    online_alignment_result_valid_ = false;
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] initializer refused an already-applied formal shadow release\n" RESET);
    return false;
  }
  return true;
}

bool VioManager::anchor_with_online_alignment(
    const OnlineAlignmentResult &result) {
  if (!is_initialized_vio || state == nullptr || state->_imu == nullptr) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] provisional VIO is unavailable\n" RESET);
    return false;
  }
  if (online_alignment_result_valid_) {
    PRINT_WARNING(YELLOW "[ONLINE-ALIGN][GAUGE] duplicate anchor ignored\n" RESET);
    return false;
  }
  if (!result.released_to_openvins || !result.diagnostics.quality_passed ||
      result.diagnostics.future_data_used || !result.q_GtoI.allFinite() ||
      !result.p_IinG.allFinite()) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] invalid or non-causal anchor rejected\n" RESET);
    return false;
  }
  if (!std::isfinite(result.timestamp) ||
      result.timestamp < state->_timestamp - 1.0e-6) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] anchor precedes active state: state=%.9f result=%.9f\n" RESET,
                state->_timestamp, result.timestamp);
    return false;
  }

  Eigen::Vector4d provisional_q_at_anchor = state->_imu->quat();
  Eigen::Vector3d provisional_p_at_anchor = state->_imu->pos();
  if (result.timestamp > state->_timestamp + 1.0e-9) {
    Eigen::Matrix<double, 13, 1> propagated_state;
    Eigen::Matrix<double, 12, 12> propagated_covariance;
    if (propagator == nullptr ||
        !propagator->fast_state_propagate(
            state, result.timestamp, propagated_state,
            propagated_covariance)) {
      PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] provisional VIO cannot be propagated to anchor t=%.9f from state t=%.9f\n" RESET,
                  result.timestamp, state->_timestamp);
      return false;
    }
    provisional_q_at_anchor = propagated_state.head<4>();
    provisional_p_at_anchor = propagated_state.segment<3>(4);
  }

  const Eigen::Matrix3d R_ItoG_current =
      ov_core::quat_2_Rot(provisional_q_at_anchor).transpose();
  const Eigen::Matrix3d R_ItoG_target =
      ov_core::quat_2_Rot(result.q_GtoI).transpose();
  const double yaw_current =
      std::atan2(R_ItoG_current(1, 0), R_ItoG_current(0, 0));
  const double yaw_target =
      std::atan2(R_ItoG_target(1, 0), R_ItoG_target(0, 0));
  double yaw_delta =
      std::atan2(std::sin(yaw_target - yaw_current),
                 std::cos(yaw_target - yaw_current));
  double metric_scale = 1.0;
  Eigen::Vector3d gauge_translation = Eigen::Vector3d::Zero();
  const bool supervised_gauge =
      result.diagnostics.gauge_consistency_passed &&
      std::isfinite(result.diagnostics.gauge_metric_scale) &&
      result.diagnostics.gauge_metric_scale > 0.0 &&
      std::isfinite(result.diagnostics.gauge_yaw_reset_deg) &&
      result.diagnostics.gauge_translation_G.allFinite();
  if (supervised_gauge) {
    metric_scale = 1.0;
    yaw_delta = result.diagnostics.gauge_yaw_reset_deg * M_PI / 180.0;
    gauge_translation = result.diagnostics.gauge_translation_G;
  }
  const Eigen::Matrix3d gauge_rotation =
      Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  if (!supervised_gauge) {
    gauge_translation =
        result.p_IinG - gauge_rotation * provisional_p_at_anchor;
  }
  const Eigen::Vector3d active_state_target_position =
      metric_scale * gauge_rotation * state->_imu->pos() +
      gauge_translation;

  const Eigen::Vector3d bg_before = state->_imu->bias_g();
  const Eigen::Vector3d ba_before = state->_imu->bias_a();
  const std::size_t clone_count_before = state->_clones_IMU.size();
  const std::size_t landmark_count_before = state->_features_SLAM.size();
  const auto reset =
      StateHelper::apply_global_yaw_scale_translation_reset(
          state, metric_scale, yaw_delta, active_state_target_position,
          propagator.get(), 0);
  if (!reset.success) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] atomic reset rejected: %s\n" RESET,
                reset.failure_reason.c_str());
    return false;
  }
  if (!state->_imu->bias_g().isApprox(bg_before, 1.0e-12) ||
      !state->_imu->bias_a().isApprox(ba_before, 1.0e-12) ||
      state->_clones_IMU.size() != clone_count_before ||
      state->_features_SLAM.size() != landmark_count_before) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][GAUGE] post-reset lifecycle invariant failed\n" RESET);
    return false;
  }

  online_alignment_result_ = result;
  online_alignment_result_.timestamp = state->_timestamp;
  online_alignment_result_.q_GtoI = state->_imu->quat();
  online_alignment_result_.p_IinG = state->_imu->pos();
  online_alignment_result_.v_IinG = state->_imu->vel();
  online_alignment_result_.bg = state->_imu->bias_g();
  online_alignment_result_.ba = state->_imu->bias_a();
  online_alignment_result_.covariance =
      StateHelper::get_marginal_covariance(state, {state->_imu});
  online_alignment_result_valid_ = true;
  startup_time = state->_timestamp;

  PRINT_INFO(GREEN "[ONLINE-ALIGN][GAUGE] RELEASED at state_t=%.6f anchor_t=%.6f scale=%.6f yaw_reset=%+.3fdeg "
                   "translation=[%+.3f %+.3f %+.3f], preserved bg/ba, "
                   "clones=%zu landmarks=%zu\n" RESET,
             state->_timestamp, result.timestamp,
             metric_scale,
             yaw_delta * 180.0 / M_PI,
             reset.translation.x(), reset.translation.y(),
             reset.translation.z(), clone_count_before,
             landmark_count_before);
  return true;
}

bool VioManager::set_online_alignment_release_covariance_override(
    double sigma_att_rad, double sigma_pos, double sigma_vel,
    double sigma_bg, double sigma_ba) {
  const std::array<double, 5> sigmas = {
      sigma_att_rad, sigma_pos, sigma_vel, sigma_bg, sigma_ba};
  for (double sigma : sigmas) {
    if (!std::isfinite(sigma) || sigma <= 0.0) {
      PRINT_ERROR(RED "[ONLINE-ALIGN][COV-ABLATION] all covariance sigmas must be finite and positive\n" RESET);
      return false;
    }
  }
  online_alignment_release_covariance_override_.setZero();
  online_alignment_release_covariance_override_.block<3, 3>(0, 0) =
      std::pow(sigma_att_rad, 2) * Eigen::Matrix3d::Identity();
  online_alignment_release_covariance_override_.block<3, 3>(3, 3) =
      std::pow(sigma_pos, 2) * Eigen::Matrix3d::Identity();
  online_alignment_release_covariance_override_.block<3, 3>(6, 6) =
      std::pow(sigma_vel, 2) * Eigen::Matrix3d::Identity();
  online_alignment_release_covariance_override_.block<3, 3>(9, 9) =
      std::pow(sigma_bg, 2) * Eigen::Matrix3d::Identity();
  online_alignment_release_covariance_override_.block<3, 3>(12, 12) =
      std::pow(sigma_ba, 2) * Eigen::Matrix3d::Identity();
  online_alignment_release_covariance_override_enabled_ = true;
  return true;
}

bool VioManager::configure_post_alignment_camera_extrinsic_rotation(
    const Eigen::Vector3d &left_rotvec_rad) {
  if (!left_rotvec_rad.allFinite() ||
      left_rotvec_rad.norm() > 10.0 * M_PI / 180.0) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] rotation must be finite and <=10deg\n" RESET);
    return false;
  }
  if (is_initialized_vio) {
    PRINT_ERROR(RED "[ONLINE-ALIGN][CAM-EXTRINSIC-ABLATION] must be configured before initialization\n" RESET);
    return false;
  }
  post_alignment_camera_extrinsic_rotation_enabled_ =
      left_rotvec_rad.norm() > 1e-12;
  post_alignment_camera_extrinsic_left_rotvec_rad_ = left_rotvec_rad;
  return true;
}

// =============================================================================
// [中文] try_to_initialize
//  初始化全过程:
//   1. 若初始化线程正在执行, 仅把当前相机时间戳记入 camera_queue_init 后返回 false
//   2. 若线程已成功过, 返回 true
//   3. 否则启动一个新线程, 在里面调用 initializer->initialize:
//        - 有 ZUPT -> wait_for_jerk = false (可在静止时立即初始化)
//        - 无 ZUPT -> wait_for_jerk = true  (必须观测到加速度阶跃)
//   4. 若初始化成功:
//        - 将初始协方差写入 State
//        - 把初始化过程中积压的相机时间戳 (camera_queue_init) 重放,
//          通过 propagate_and_clone + marginalize_old_clone 把状态推到最新时刻
//   5. 函数返回的是 thread_init_success 的"上一轮"结果, 真正的就绪判断靠下次调用
// =============================================================================
bool VioManager::try_to_initialize(const ov_core::CameraData &message) {

  // Directly return if the initialization thread is running
  // Note that we lock on the queue since we could have finished an update
  // And are using this queue to propagate the state forward. We should wait in this case
  if (thread_init_running) {
    std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
    camera_queue_init.push_back(message.timestamp);
    return false;
  }

  // If the thread was a success, then return success!
  if (thread_init_success) {
    return true;
  }

  // Run the initialization in a second thread so it can go as slow as it desires
  thread_init_running = true;
  std::thread thread([&] {
    // Returns from our initializer
    double timestamp;
    Eigen::MatrixXd covariance;
    std::vector<std::shared_ptr<ov_type::Type>> order;
    auto init_rT1 = boost::posix_time::microsec_clock::local_time();

    // Try to initialize the system
    // We will wait for a jerk if we do not have the zero velocity update enabled
    // Otherwise we can initialize right away as the zero velocity will handle the stationary case
    bool wait_for_jerk = (updaterZUPT == nullptr);
    bool success = initializer->initialize(timestamp, covariance, order, state->_imu, wait_for_jerk);

    // If we have initialized successfully we will set the covariance and state elements as needed
    // TODO: set the clones and SLAM features here so we can start updating right away...
    if (success) {

      // Set our covariance (state should already be set in the initializer)
      StateHelper::set_initial_covariance(state, covariance, order);

      // Set the state time
      state->_timestamp = timestamp;
      startup_time = timestamp;

      // Cleanup any features older than the initialization time
      // Also increase the number of features to the desired amount during estimation
      // NOTE: we will split the total number of features over all cameras uniformly
      trackFEATS->get_feature_database()->cleanup_measurements(state->_timestamp);
      trackFEATS->set_num_features(std::floor((double)params.num_pts / (double)params.state_options.num_cameras));
      if (trackARUCO != nullptr) {
        trackARUCO->get_feature_database()->cleanup_measurements(state->_timestamp);
      }

      // If we are moving then don't do zero velocity update4
      if (state->_imu->vel().norm() > params.zupt_max_velocity) {
        has_moved_since_zupt = true;
      }

      // Else we are good to go, print out our stats
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_INFO(GREEN "[init]: successful initialization in %.4f seconds\n" RESET, (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      PRINT_INFO(GREEN "[init]: orientation = %.4f, %.4f, %.4f, %.4f\n" RESET, state->_imu->quat()(0), state->_imu->quat()(1),
                 state->_imu->quat()(2), state->_imu->quat()(3));
      PRINT_INFO(GREEN "[init]: bias gyro = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_g()(0), state->_imu->bias_g()(1),
                 state->_imu->bias_g()(2));
      PRINT_INFO(GREEN "[init]: velocity = %.4f, %.4f, %.4f\n" RESET, state->_imu->vel()(0), state->_imu->vel()(1), state->_imu->vel()(2));
      PRINT_INFO(GREEN "[init]: bias accel = %.4f, %.4f, %.4f\n" RESET, state->_imu->bias_a()(0), state->_imu->bias_a()(1),
                 state->_imu->bias_a()(2));
      PRINT_INFO(GREEN "[init]: position = %.4f, %.4f, %.4f\n" RESET, state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));

      // Remove any camera times that are order then the initialized time
      // This can happen if the initialization has taken a while to perform
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      std::vector<double> camera_timestamps_to_init;
      for (size_t i = 0; i < camera_queue_init.size(); i++) {
        if (camera_queue_init.at(i) > timestamp) {
          camera_timestamps_to_init.push_back(camera_queue_init.at(i));
        }
      }

      // Now we have initialized we will propagate the state to the current timestep
      // In general this should be ok as long as the initialization didn't take too long to perform
      // Propagating over multiple seconds will become an issue if the initial biases are bad
      size_t clone_rate = (size_t)((double)camera_timestamps_to_init.size() / (double)params.state_options.max_clone_size) + 1;
      for (size_t i = 0; i < camera_timestamps_to_init.size(); i += clone_rate) {
        propagator->propagate_and_clone(state, camera_timestamps_to_init.at(i));
        StateHelper::marginalize_old_clone(state);
      }
      PRINT_DEBUG(YELLOW "[init]: moved the state forward %.2f seconds\n" RESET, state->_timestamp - timestamp);
      thread_init_success = true;
      camera_queue_init.clear();

    } else {
      auto init_rT2 = boost::posix_time::microsec_clock::local_time();
      PRINT_DEBUG(YELLOW "[init]: failed initialization in %.4f seconds\n" RESET, (init_rT2 - init_rT1).total_microseconds() * 1e-6);
      thread_init_success = false;
      std::lock_guard<std::mutex> lck(camera_queue_init_mtx);
      camera_queue_init.clear();
    }

    // Finally, mark that the thread has finished running
    thread_init_running = false;
  });

  // If we are single threaded, then run single threaded
  // Otherwise detach this thread so it runs in the background!
  if (!params.use_multi_threading_subs) {
    thread.join();
  } else {
    thread.detach();
  }
  return false;
}

void VioManager::retriangulate_active_tracks(const ov_core::CameraData &message) {

  // Start timing
  boost::posix_time::ptime retri_rT1, retri_rT2, retri_rT3;
  retri_rT1 = boost::posix_time::microsec_clock::local_time();

  // Clear old active track data
  assert(state->_clones_IMU.find(message.timestamp) != state->_clones_IMU.end());
  active_tracks_time = message.timestamp;
  active_image = cv::Mat();
  trackFEATS->display_active(active_image, 255, 255, 255, 255, 255, 255, " ");
  if (!active_image.empty()) {
    active_image = active_image(cv::Rect(0, 0, message.images.at(0).cols, message.images.at(0).rows));
  }
  active_tracks_posinG.clear();
  active_tracks_uvd.clear();

  // Current active tracks in our frontend
  // TODO: should probably assert here that these are at the message time...
  auto last_obs = trackFEATS->get_last_obs();
  auto last_ids = trackFEATS->get_last_ids();

  // New set of linear systems that only contain the latest track info
  std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A_new;
  std::map<size_t, Eigen::Vector3d> active_feat_linsys_b_new;
  std::map<size_t, int> active_feat_linsys_count_new;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG_new;

  // Append our new observations for each camera
  std::map<size_t, cv::Point2f> feat_uvs_in_cam0;
  for (auto const &cam_id : message.sensor_ids) {

    // IMU historical clone
    Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(active_tracks_time)->Rot();
    Eigen::Vector3d p_IinG = state->_clones_IMU.at(active_tracks_time)->pos();

    // Calibration for this cam_id
    Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(cam_id)->Rot();
    Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(cam_id)->pos();

    // Convert current CAMERA position relative to global
    Eigen::Matrix3d R_GtoCi = R_ItoC * R_GtoI;
    Eigen::Vector3d p_CiinG = p_IinG - R_GtoCi.transpose() * p_IinC;

    // Loop through each measurement
    assert(last_obs.find(cam_id) != last_obs.end());
    assert(last_ids.find(cam_id) != last_ids.end());
    for (size_t i = 0; i < last_obs.at(cam_id).size(); i++) {

      // Record this feature uv if is seen from cam0
      size_t featid = last_ids.at(cam_id).at(i);
      cv::Point2f pt_d = last_obs.at(cam_id).at(i).pt;
      if (cam_id == 0) {
        feat_uvs_in_cam0[featid] = pt_d;
      }

      // Skip this feature if it is a SLAM feature (the state estimate takes priority)
      if (state->_features_SLAM.find(featid) != state->_features_SLAM.end()) {
        continue;
      }

      // Get the UV coordinate normal
      cv::Point2f pt_n = state->_cam_intrinsics_cameras.at(cam_id)->undistort_cv(pt_d);
      Eigen::Matrix<double, 3, 1> b_i;
      b_i << pt_n.x, pt_n.y, 1;
      b_i = R_GtoCi.transpose() * b_i;
      b_i = b_i / b_i.norm();
      Eigen::Matrix3d Bperp = skew_x(b_i);

      // Append to our linear system
      Eigen::Matrix3d Ai = Bperp.transpose() * Bperp;
      Eigen::Vector3d bi = Ai * p_CiinG;
      if (active_feat_linsys_A.find(featid) == active_feat_linsys_A.end()) {
        active_feat_linsys_A_new.insert({featid, Ai});
        active_feat_linsys_b_new.insert({featid, bi});
        active_feat_linsys_count_new.insert({featid, 1});
      } else {
        active_feat_linsys_A_new[featid] = Ai + active_feat_linsys_A[featid];
        active_feat_linsys_b_new[featid] = bi + active_feat_linsys_b[featid];
        active_feat_linsys_count_new[featid] = 1 + active_feat_linsys_count[featid];
      }

      // For this feature, recover its 3d position if we have enough observations!
      if (active_feat_linsys_count_new.at(featid) > 3) {

        // Recover feature estimate
        Eigen::Matrix3d A = active_feat_linsys_A_new[featid];
        Eigen::Vector3d b = active_feat_linsys_b_new[featid];
        Eigen::MatrixXd p_FinG = A.colPivHouseholderQr().solve(b);
        Eigen::MatrixXd p_FinCi = R_GtoCi * (p_FinG - p_CiinG);

        // Check A and p_FinCi
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
        Eigen::MatrixXd singularValues;
        singularValues.resize(svd.singularValues().rows(), 1);
        singularValues = svd.singularValues();
        double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);

        // If we have a bad condition number, or it is too close
        // Then set the flag for bad (i.e. set z-axis to nan)
        if (std::abs(condA) <= params.featinit_options.max_cond_number && p_FinCi(2, 0) >= params.featinit_options.min_dist &&
            p_FinCi(2, 0) <= params.featinit_options.max_dist && !std::isnan(p_FinCi.norm())) {
          active_tracks_posinG_new[featid] = p_FinG;
        }
      }
    }
  }
  size_t total_triangulated = active_tracks_posinG.size();

  // Update active set of linear systems
  active_feat_linsys_A = active_feat_linsys_A_new;
  active_feat_linsys_b = active_feat_linsys_b_new;
  active_feat_linsys_count = active_feat_linsys_count_new;
  active_tracks_posinG = active_tracks_posinG_new;
  retri_rT2 = boost::posix_time::microsec_clock::local_time();

  // Return if no features
  if (active_tracks_posinG.empty() && state->_features_SLAM.empty())
    return;

  // Append our SLAM features we have
  for (const auto &feat : state->_features_SLAM) {
    Eigen::Vector3d p_FinG = feat.second->get_xyz(false);
    if (LandmarkRepresentation::is_relative_representation(feat.second->_feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(feat.second->_anchor_cam_id != -1);
      // Get calibration for our anchor camera
      Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(feat.second->_anchor_cam_id)->Rot();
      Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(feat.second->_anchor_cam_id)->pos();
      // Anchor pose orientation and position
      Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(feat.second->_anchor_clone_timestamp)->Rot();
      Eigen::Vector3d p_IinG = state->_clones_IMU.at(feat.second->_anchor_clone_timestamp)->pos();
      // Feature in the global frame
      p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (feat.second->get_xyz(false) - p_IinC) + p_IinG;
    }
    active_tracks_posinG[feat.second->_featid] = p_FinG;
  }

  // Calibration of the first camera (cam0)
  std::shared_ptr<Vec> distortion = state->_cam_intrinsics.at(0);
  std::shared_ptr<PoseJPL> calibration = state->_calib_IMUtoCAM.at(0);
  Eigen::Matrix<double, 3, 3> R_ItoC = calibration->Rot();
  Eigen::Matrix<double, 3, 1> p_IinC = calibration->pos();

  // Get current IMU clone state
  std::shared_ptr<PoseJPL> clone_Ii = state->_clones_IMU.at(active_tracks_time);
  Eigen::Matrix3d R_GtoIi = clone_Ii->Rot();
  Eigen::Vector3d p_IiinG = clone_Ii->pos();

  // 4. Next we can update our variable with the global position
  //    We also will project the features into the current frame
  for (const auto &feat : active_tracks_posinG) {

    // For now skip features not seen from current frame
    // TODO: should we publish other features not tracked in cam0??
    if (feat_uvs_in_cam0.find(feat.first) == feat_uvs_in_cam0.end())
      continue;

    // Calculate the depth of the feature in the current frame
    // Project SLAM feature and non-cam0 features into the current frame of reference
    Eigen::Vector3d p_FinIi = R_GtoIi * (feat.second - p_IiinG);
    Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
    double depth = p_FinCi(2);
    Eigen::Vector2d uv_dist;
    if (feat_uvs_in_cam0.find(feat.first) != feat_uvs_in_cam0.end()) {
      uv_dist << (double)feat_uvs_in_cam0.at(feat.first).x, (double)feat_uvs_in_cam0.at(feat.first).y;
    } else {
      Eigen::Vector2d uv_norm;
      uv_norm << p_FinCi(0) / depth, p_FinCi(1) / depth;
      uv_dist = state->_cam_intrinsics_cameras.at(0)->distort_d(uv_norm);
    }

    // Skip if not valid (i.e. negative depth, or outside of image)
    if (depth < 0.1) {
      continue;
    }

    // Skip if not valid (i.e. negative depth, or outside of image)
    int width = state->_cam_intrinsics_cameras.at(0)->w();
    int height = state->_cam_intrinsics_cameras.at(0)->h();
    if (uv_dist(0) < 0 || (int)uv_dist(0) >= width || uv_dist(1) < 0 || (int)uv_dist(1) >= height) {
      // PRINT_DEBUG("feat %zu -> depth = %.2f | u_d = %.2f | v_d = %.2f\n",(*it2)->featid,depth,uv_dist(0),uv_dist(1));
      continue;
    }

    // Finally construct the uv and depth
    Eigen::Vector3d uvd;
    uvd << uv_dist, depth;
    active_tracks_uvd.insert({feat.first, uvd});
  }
  retri_rT3 = boost::posix_time::microsec_clock::local_time();

  // Timing information
  PRINT_ALL(CYAN "[RETRI-TIME]: %.4f seconds for triangulation (%zu tri of %zu active)\n" RESET,
            (retri_rT2 - retri_rT1).total_microseconds() * 1e-6, total_triangulated, active_feat_linsys_A.size());
  PRINT_ALL(CYAN "[RETRI-TIME]: %.4f seconds for re-projection into current\n" RESET, (retri_rT3 - retri_rT2).total_microseconds() * 1e-6);
  PRINT_ALL(CYAN "[RETRI-TIME]: %.4f seconds total\n" RESET, (retri_rT3 - retri_rT1).total_microseconds() * 1e-6);
}

cv::Mat VioManager::get_historical_viz_image() {

  // Return if not ready yet
  if (state == nullptr || trackFEATS == nullptr)
    return cv::Mat();

  // Build an id-list of what features we should highlight (i.e. SLAM)
  std::vector<size_t> highlighted_ids;
  for (const auto &feat : state->_features_SLAM) {
    highlighted_ids.push_back(feat.first);
  }

  // Text we will overlay if needed
  std::string overlay = (did_zupt_update) ? "zvupt" : "";
  overlay = (!is_initialized_vio) ? "init" : overlay;

  // Get the current active tracks
  cv::Mat img_history;
  trackFEATS->display_history(img_history, 255, 255, 0, 255, 255, 255, highlighted_ids, overlay);
  if (trackARUCO != nullptr) {
    trackARUCO->display_history(img_history, 0, 255, 255, 255, 255, 255, highlighted_ids, overlay);
    // trackARUCO->display_active(img_history, 0, 255, 255, 255, 255, 255, overlay);
  }

  // Finally return the image
  return img_history;
}

std::vector<Eigen::Vector3d> VioManager::get_features_SLAM() {
  std::vector<Eigen::Vector3d> slam_feats;
  for (auto &f : state->_features_SLAM) {
    if ((int)f.first <= 4 * state->_options.max_aruco_features)
      continue;
    if (ov_type::LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(f.second->_anchor_cam_id != -1);
      // Get calibration for our anchor camera
      Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
      Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
      // Anchor pose orientation and position
      Eigen::Matrix<double, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
      Eigen::Matrix<double, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
      // Feature in the global frame
      slam_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
    } else {
      slam_feats.push_back(f.second->get_xyz(false));
    }
  }
  return slam_feats;
}

int VioManager::get_feature_database_size() {
  if (trackFEATS == nullptr || trackFEATS->get_feature_database() == nullptr)
    return 0;
  return (int)trackFEATS->get_feature_database()->size();
}

std::vector<Eigen::Vector3d> VioManager::get_features_ARUCO() {
  std::vector<Eigen::Vector3d> aruco_feats;
  for (auto &f : state->_features_SLAM) {
    if ((int)f.first > 4 * state->_options.max_aruco_features)
      continue;
    if (ov_type::LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(f.second->_anchor_cam_id != -1);
      // Get calibration for our anchor camera
      Eigen::Matrix<double, 3, 3> R_ItoC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->Rot();
      Eigen::Matrix<double, 3, 1> p_IinC = state->_calib_IMUtoCAM.at(f.second->_anchor_cam_id)->pos();
      // Anchor pose orientation and position
      Eigen::Matrix<double, 3, 3> R_GtoI = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->Rot();
      Eigen::Matrix<double, 3, 1> p_IinG = state->_clones_IMU.at(f.second->_anchor_clone_timestamp)->pos();
      // Feature in the global frame
      aruco_feats.push_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
    } else {
      aruco_feats.push_back(f.second->get_xyz(false));
    }
  }
  return aruco_feats;
}
