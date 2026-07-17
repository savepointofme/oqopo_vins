#include "core/OnlineAlignmentInitializer.h"

#include "utils/quat_ops.h"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

namespace {

using ov_msckf::BoardImuSample;
using ov_msckf::AlignmentCloneState;
using ov_msckf::AlignmentPhase;
using ov_msckf::AlignmentReadiness;
using ov_msckf::AlignmentReleasePolicy;
using ov_msckf::CandidateGroupGate;
using ov_msckf::CandidateStateGroup;
using ov_msckf::FCNavigationSample;
using ov_msckf::EstimateSourceStatus;
using ov_msckf::FactorContribution;
using ov_msckf::OnlineAlignmentInitializer;
using ov_msckf::OnlineAlignmentOptions;
using ov_msckf::OnlineAlignmentResult;
using ov_msckf::StereoAlignmentFrame;
using ov_msckf::StereoAlignmentObservation;

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

bool contains(const std::string &value, const std::string &needle) {
  return value.find(needle) != std::string::npos;
}

const FactorContribution *find_factor(
    const ov_msckf::OnlineAlignmentDiagnostics &diagnostics,
    const std::string &family) {
  for (const auto &factor : diagnostics.factor_contributions)
    if (factor.family == family)
      return &factor;
  return nullptr;
}

OnlineAlignmentOptions make_options() {
  OnlineAlignmentOptions options;
  // Keep the broad synthetic suite bounded. The formal runner explicitly
  // overrides this to 36 and exercises every frame selected by its 8 s
  // time-window policy.
  options.max_keyframes = 10;
  options.reference_window_duration_s = 5.0;
  options.candidate_window_durations_s = {5.0, 8.0, 12.0};
  options.camera_to_imu_time_offset_s = 0.013;
  options.min_angular_excitation_rad_s = 0.01;
  options.min_second_axis_ratio = 0.005;
  options.max_rate_residual_rms_rad_s = 0.04;
  options.max_angular_rate_rad_s = 0.5;
  options.max_mount_residual_deg = 20.0;
  options.max_attitude_sigma_deg = 20.0;
  options.max_velocity_sigma_mps = 20.0;
  options.max_position_sigma_m = 20.0;
  options.max_gyro_bias_sigma_rad_s = 0.2;
  options.max_accel_bias_sigma_mps2 = 2.0;
  options.max_mount_sigma_deg = 20.0;
  options.navigation_max_attitude_sigma_deg = 20.0;
  options.navigation_max_position_sigma_m = 20.0;
  options.navigation_max_velocity_sigma_mps = 20.0;
  options.navigation_max_gyro_bias_sigma_rad_s = 0.2;
  options.navigation_max_accel_bias_sigma_mps2 = 2.0;
  options.navigation_max_reprojection_rmse_px = 5.0;
  options.navigation_max_reprojection_p95_px = 10.0;
  options.candidate_max_fc_imu_rotation_residual_deg = 30.0;
  options.candidate_reject_fc_imu_rotation_residual_deg = 60.0;
  options.candidate_max_relative_position_residual_m = 30.0;
  options.candidate_reject_relative_position_residual_m = 60.0;
  options.candidate_max_relative_velocity_residual_mps = 15.0;
  options.candidate_reject_relative_velocity_residual_mps = 30.0;
  options.candidate_max_visual_compensated_p95_px = 2000.0;
  options.candidate_reject_visual_compensated_p95_px = 4000.0;
  // The synthetic scene deliberately uses sparse, fast image motion so this
  // test exercises lifecycle and causal validation instead of production
  // pixel gates. Production defaults remain unchanged.
  // The production candidate now validates a full FC sample holdout derived
  // from the selected startup window. This synthetic scene has deliberately
  // sparse/fast image motion, so its late reprojection tail is not a control
  // threshold for the sliding-window lifecycle tests.
  options.candidate_max_visual_reprojection_p95_px = 2000.0;
  options.candidate_reject_visual_reprojection_p95_px = 4000.0;
  options.candidate_max_visual_reprojection_trend_pxps = 1000.0;
  options.candidate_max_closed_loop_attitude_correction_deg = 30.0;
  options.candidate_max_closed_loop_position_correction_m = 30.0;
  options.candidate_max_closed_loop_velocity_correction_mps = 15.0;
  options.candidate_max_closed_loop_gyro_bias_correction_rad_s = 0.2;
  options.candidate_max_closed_loop_accel_bias_correction_mps2 = 2.0;
  const auto configure_gate =
      [&](CandidateStateGroup group, const Eigen::Vector3d &max_abs_error,
          const Eigen::Vector3d &max_std,
          const Eigen::Vector3d &max_feedback_step,
          const Eigen::Vector3d &max_cumulative_feedback) {
        CandidateGroupGate &gate =
            options.candidate_filter_config.group_gates
                [static_cast<std::size_t>(group)];
        gate.configured = true;
        gate.derive_depth_from_sliding_window = true;
        gate.allow_initial_feedback_before_derived_history =
            group == CandidateStateGroup::ATTITUDE ||
            group == CandidateStateGroup::GYRO_BIAS ||
            group == CandidateStateGroup::ACCEL_BIAS;
        gate.require_initial_graph_support = true;
        gate.history_length_updates = 0;
        gate.min_supported_updates = 0;
        gate.required_post_feedback_stable_updates = 0;
        gate.max_abs_error = max_abs_error;
        gate.max_std = max_std;
        gate.max_std_step = max_std;
        gate.max_error_peak_to_peak = 2.0 * max_abs_error;
        gate.max_std_peak_to_peak = max_std;
        gate.max_feedback_step = max_feedback_step;
        gate.max_cumulative_feedback = max_cumulative_feedback;
      };
  const double max_attitude_error_rad = 30.0 * kPi / 180.0;
  configure_gate(CandidateStateGroup::ATTITUDE,
                 Eigen::Vector3d::Constant(max_attitude_error_rad),
                 Eigen::Vector3d::Constant(20.0 * kPi / 180.0),
                 Eigen::Vector3d::Constant(max_attitude_error_rad),
                 Eigen::Vector3d::Constant(max_attitude_error_rad));
  configure_gate(CandidateStateGroup::POSITION,
                 Eigen::Vector3d::Constant(30.0),
                 Eigen::Vector3d::Constant(20.0),
                 Eigen::Vector3d::Constant(30.0),
                 Eigen::Vector3d::Constant(30.0));
  configure_gate(CandidateStateGroup::VELOCITY,
                 Eigen::Vector3d::Constant(15.0),
                 Eigen::Vector3d::Constant(20.0),
                 Eigen::Vector3d::Constant(15.0),
                 Eigen::Vector3d::Constant(15.0));
  // Repeated FC position/velocity updates provide their own direct support.
  // q/bg/ba still require independent support from the initial joint graph.
  options.candidate_filter_config
      .group_gates[static_cast<std::size_t>(CandidateStateGroup::POSITION)]
      .require_initial_graph_support = false;
  options.candidate_filter_config
      .group_gates[static_cast<std::size_t>(CandidateStateGroup::VELOCITY)]
      .require_initial_graph_support = false;
  configure_gate(CandidateStateGroup::GYRO_BIAS,
                 Eigen::Vector3d::Constant(0.2),
                 Eigen::Vector3d::Constant(0.2),
                 Eigen::Vector3d::Constant(0.2),
                 Eigen::Vector3d::Constant(0.2));
  configure_gate(CandidateStateGroup::ACCEL_BIAS,
                 Eigen::Vector3d::Constant(2.0),
                 Eigen::Vector3d::Constant(2.0),
                 Eigen::Vector3d::Constant(2.0),
                 Eigen::Vector3d::Constant(2.0));
  options.min_information_eigenvalue = 1e-12;
  options.max_information_condition = 1e15;
  options.solver_max_time_s = 5.0;
  for (int camera = 0; camera < 2; ++camera) {
    options.camera_intrinsics[camera] << 400.0, 400.0, 640.0, 360.0,
        0.0, 0.0, 0.0, 0.0;
  }
  options.p_IinC[1] = Eigen::Vector3d(-0.12, 0.0, 0.0);
  options.provenance.fc_stream = "synthetic_fc_navigation";
  options.provenance.imu_stream = "synthetic_board_imu";
  options.provenance.stereo_stream = "synthetic_stereo_tracks";
  options.provenance.camera_imu_calibration = "locked_synthetic_T_C_I";
  options.provenance.fc_axis_mapping = "declared_identity_for_test";
  return options;
}

Eigen::Vector3d omega_F(double t) {
  return Eigen::Vector3d(0.08 + 0.12 * std::sin(0.9 * t),
                         0.04 + 0.10 * std::cos(0.7 * t),
                         0.15 + 0.08 * std::sin(0.4 * t));
}

Eigen::Vector3d velocity_G(double t) {
  return Eigen::Vector3d(5.0 + 0.5 * std::sin(0.3 * t),
                         1.0 + 0.2 * std::cos(0.5 * t),
                         0.1 * std::sin(0.6 * t));
}

Eigen::Vector3d acceleration_G(double t) {
  return Eigen::Vector3d(0.15 * std::cos(0.3 * t),
                         -0.1 * std::sin(0.5 * t),
                         0.06 * std::cos(0.6 * t));
}

Eigen::Vector3d position_G(double t) {
  return Eigen::Vector3d(5.0 * t - (0.5 / 0.3) * std::cos(0.3 * t) + 0.5 / 0.3,
                         t + (0.2 / 0.5) * std::sin(0.5 * t),
                         (0.1 / 0.6) * (1.0 - std::cos(0.6 * t)));
}

Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w) {
  const double angle = w.norm();
  if (angle < 1e-12)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, w / angle).toRotationMatrix();
}

struct SyntheticMotion {
  double dt = 0.001;
  std::vector<Eigen::Matrix3d> R_GtoF;

  SyntheticMotion() {
    R_GtoF.push_back(Eigen::Matrix3d::Identity());
    // Cover both the solve window and the bounded post-candidate validation
    // interval. Clamping the synthetic FC attitude while continuing to feed a
    // non-zero IMU rate creates an artificial validation failure.
    for (int i = 1; i <= 30000; ++i) {
      const double t = (i - 0.5) * dt;
      R_GtoF.push_back(exp_so3(-omega_F(t) * dt) * R_GtoF.back());
    }
  }

  Eigen::Matrix3d rotation(double t) const {
    t = std::max(0.0, std::min(t, dt * (R_GtoF.size() - 1)));
    const double index = t / dt;
    const size_t lo = static_cast<size_t>(std::floor(index));
    const size_t hi = std::min(lo + 1, R_GtoF.size() - 1);
    const double alpha = index - lo;
    Eigen::Quaterniond q0(R_GtoF[lo]);
    Eigen::Quaterniond q1(R_GtoF[hi]);
    return q0.slerp(alpha, q1).normalized().toRotationMatrix();
  }
};

StereoAlignmentFrame make_stereo_frame(double camera_time,
                                       const SyntheticMotion &motion,
                                       const Eigen::Matrix3d &R_FtoI,
                                       double camera_to_imu_offset,
                                       double clock_offset, int frame_index,
                                       bool include_right = false) {
  const double board_time = camera_time + camera_to_imu_offset;
  const double fc_time = board_time - clock_offset;
  const Eigen::Matrix3d R_GtoI = R_FtoI * motion.rotation(fc_time);
  const Eigen::Vector3d p = position_G(fc_time);
  StereoAlignmentFrame frame;
  // Stereo transport timestamps remain camera-clock timestamps. The locked
  // Camera-to-IMU offset is applied exactly once by the initializer.
  frame.left_timestamp = camera_time;
  frame.right_timestamp = camera_time;
  frame.tracking_valid = true;
  frame.stereo_valid = true;
  size_t id = 0;
  for (int z_index = 0; z_index < 4; ++z_index) {
    for (int y_index = -4; y_index <= 4; ++y_index) {
      for (int x_index = -6; x_index <= 6; ++x_index, ++id) {
        const Eigen::Vector3d point_G(2.0 * x_index + 12.0,
                                      1.5 * y_index + 4.0,
                                      24.0 + 8.0 * z_index);
        const Eigen::Vector3d point_I = R_GtoI * (point_G - p);
        if (point_I.z() < 4.0)
          continue;
        const Eigen::Vector3d point_right =
            point_I - Eigen::Vector3d(0.12, 0.0, 0.0);
        StereoAlignmentObservation obs;
        obs.feature_id = id;
        obs.normalized_left = point_I.head<2>() / point_I.z();
        obs.normalized_right = point_right.head<2>() / point_right.z();
        obs.raw_left = 400.0 * obs.normalized_left + Eigen::Vector2d(640, 360);
        obs.raw_right = 400.0 * obs.normalized_right + Eigen::Vector2d(640, 360);
        obs.depth_m = point_I.z();
        obs.stereo_ray_residual_m = 0.0;
        obs.track_length = frame_index + 1;
        obs.left_valid = true;
        obs.right_valid = include_right;
        frame.observations.push_back(obs);
      }
    }
  }
  return frame;
}

void test_rejections_and_causal_recovery() {
  OnlineAlignmentOptions options = make_options();
  const SyntheticMotion motion;
  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(3.0 * kPi / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(-2.0 * kPi / 180.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  const Eigen::Vector3d bg(0.012, -0.008, 0.006);
  const Eigen::Vector3d ba(0.12, -0.08, 0.05);
  const double offset = 0.024;
  options.fc_board_calibration_locked = true;
  options.R_FtoI_declared = R_mount;
  options.fc_attitude_to_board_time_offset_s = offset;
  options.fc_navigation_to_board_time_offset_s = 0.0;

  OnlineAlignmentInitializer initializer(options);
  FCNavigationSample invalid_fc;
  invalid_fc.timestamp = 0.0;
  invalid_fc.navigation_frame = "wrong";
  invalid_fc.body_frame = "FC_body";
  invalid_fc.position_valid = invalid_fc.velocity_valid =
      invalid_fc.attitude_valid = invalid_fc.status_valid = true;
  require(!initializer.feed_fc_navigation(invalid_fc),
          "frame mismatch must be rejected");
  require(initializer.last_rejection() == "fc_navigation_frame_mismatch",
          "frame rejection must be named");
  require(initializer.phase() == AlignmentPhase::FATAL_CONFIGURATION_ERROR,
          "frame-contract mismatch must enter a fatal state");
  initializer.reset();

  bool early_checked = false;
  int frame_index = 0;
  for (int tick = 0; tick <= 1240; ++tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      const double fc_time = board_time;
      FCNavigationSample fc;
      fc.timestamp = fc_time;
      fc.position_G = position_G(fc_time);
      fc.velocity_G = velocity_G(fc_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(fc_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "valid FC feed");
    }

    const double fc_time_for_imu = std::max(0.0, board_time - offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity = R_mount * omega_F(fc_time_for_imu) + bg;
    if (tick == 1150)
      imu.angular_velocity += Eigen::Vector3d(0.6, 0.0, 0.0);
    const Eigen::Matrix3d R_GtoI =
        R_mount * motion.rotation(fc_time_for_imu);
    imu.linear_acceleration =
        R_GtoI * (acceleration_G(fc_time_for_imu) + options.gravity_G) + ba;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "valid IMU feed");

    if (tick % 20 == 0 && board_time >= 0.1) {
      const double camera_time =
          board_time - options.camera_to_imu_time_offset_s;
      StereoAlignmentFrame frame = make_stereo_frame(
          camera_time, motion, R_mount,
          options.camera_to_imu_time_offset_s, offset, frame_index++);
      require(initializer.feed_stereo(frame), "valid stereo feed");
    }

    if (!early_checked && board_time >= 1.0) {
      OnlineAlignmentResult early;
      require(!initializer.try_initialize(board_time, early),
              "initializer must not release before the full causal window");
      require(!initializer.alignment_window_closed(),
              "an unusable early window must keep sliding instead of closing");
      early_checked = true;
    }
  }

  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result) &&
              initializer.candidate_active(),
          "covered causal window must create a validation candidate: " +
              initializer.last_rejection());
  const auto &candidate = initializer.current_candidate();
  require(!candidate.result.released_to_openvins &&
              !candidate.selected_frame_timestamps.empty() &&
              !candidate.selected_feature_ids.empty() &&
              candidate.optimized_landmarks_G.empty() &&
              candidate.solve_window_end > candidate.solve_window_start,
          "landmark-free AlignmentCandidate must retain its bounded solve "
          "snapshot and provenance");
  const int solves_before_duplicate_try =
      initializer.last_diagnostics().nonlinear_solve_attempt_count;
  OnlineAlignmentResult pending_duplicate;
  require(!initializer.try_initialize(6.2, pending_duplicate) &&
              initializer.last_diagnostics().nonlinear_solve_attempt_count ==
                  solves_before_duplicate_try,
          "candidate validation calls must not rerun Ceres on the same frame");
  bool released = false;
  const double candidate_board_time =
      candidate.result.timestamp + options.camera_to_imu_time_offset_s;
  for (int tick = 1241; tick <= 5400 && !released; ++tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = board_time;
      fc.position_G = position_G(board_time);
      fc.velocity_G = velocity_G(board_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(board_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "validation FC feed");
    }
    const double fc_time_for_imu = std::max(0.0, board_time - offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity = R_mount * omega_F(fc_time_for_imu) + bg;
    const Eigen::Matrix3d R_GtoI =
        R_mount * motion.rotation(fc_time_for_imu);
    imu.linear_acceleration =
        R_GtoI * (acceleration_G(fc_time_for_imu) + options.gravity_G) + ba;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "validation IMU feed");
    if (tick % 20 == 0) {
      const double camera_time =
          board_time - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, R_mount,
                  options.camera_to_imu_time_offset_s, offset, frame_index++)),
              "validation visual feed");
      released = initializer.try_initialize(camera_time, result);
    }
  }
  if (!released) {
    const auto &diagnostics = initializer.last_diagnostics();
    std::cerr << "candidate diagnostics rotation="
              << diagnostics.candidate_fc_imu_rotation_residual_deg
              << " position="
              << diagnostics.candidate_relative_position_residual_m
              << " velocity="
              << diagnostics.candidate_relative_velocity_residual_mps
              << " visual="
              << diagnostics.candidate_visual_compensated_p95_px
              << " reprojection="
              << diagnostics.candidate_visual_reprojection_p95_px
              << " reprojection_trend="
              << diagnostics.candidate_visual_reprojection_trend_pxps
              << " solves=" << diagnostics.nonlinear_solve_attempt_count
              << " candidates=" << diagnostics.candidate_created_count
              << " rejected=" << diagnostics.candidate_rejected_count
              << " fc_consumed="
              << diagnostics.candidate_consumed_fc_event_count
              << " fc_accepted="
              << diagnostics.candidate_accepted_fc_event_count
              << " fc_rejected="
              << diagnostics.candidate_rejected_fc_event_count
              << " imu_steps="
              << diagnostics.candidate_propagated_imu_step_count
              << " feedback_tier=" << diagnostics.candidate_feedback_tier
              << " dx=" << diagnostics.candidate_persistent_error.transpose()
              << " std=" << diagnostics.candidate_persistent_std.transpose()
              << std::endl;
    for (const auto &receipt : initializer.attempt_receipts())
      std::cerr << "  receipt outcome=" << receipt.outcome
                << " gate=" << receipt.failed_gate
                << " next=" << receipt.next_eligible_condition << std::endl;
  }
  require(released, "bounded future validation must release stable candidate: " +
                        initializer.last_rejection());
  require(result.readiness == AlignmentReadiness::FULL_ALIGNMENT_READY &&
              result.released_to_openvins,
          "excited joint solve must atomically release full alignment");
  require(!result.diagnostics.future_data_used, "future_data_used must be false");
  require(std::fabs(result.fc_attitude_to_board_time_offset_s - offset) <=
              1.0e-12 &&
              std::fabs(result.fc_navigation_to_board_time_offset_s) <=
                  1.0e-12 &&
              std::fabs(result.fc_to_board_time_offset_s) <= 1.0e-12,
          "locked attitude and navigation time contracts");
  const double mount_error =
      Eigen::AngleAxisd(result.R_FtoI_nominal * R_mount.transpose()).angle() *
      180.0 / kPi;
  require(mount_error < 1.0e-9, "locked FC-to-board rotation: " +
                                 std::to_string(mount_error) + " deg");
  require((result.bg - bg).norm() < 0.015, "gyro bias recovery");
  require((result.ba - ba).norm() < 0.35,
          "accelerometer bias recovery error=" +
              std::to_string((result.ba - ba).norm()));
  require(!result.diagnostics.rejected_intervals.empty(),
          "high-angular-rate interval must be rejected and recorded");
  require(result.diagnostics.stereo_depths > 0,
          "monocular multi-frame epipolar pairs must be accepted");
  require(std::fabs(result.diagnostics.selected_window_duration_s - 5.0) <
              1e-9 &&
              result.diagnostics.keyframes <= options.max_keyframes,
          "the shortest configured usable window must respect graph caps");
  require(!initializer.attempt_receipts().empty() &&
              initializer.attempt_receipts().back().outcome ==
                  "validated_release",
          "attempt receipt must end in the applied validated release");
  require(std::isfinite(
              result.diagnostics.visual_statistics.reprojection_rmse_px) &&
              std::isfinite(
                  result.diagnostics.visual_statistics.reprojection_p95_px) &&
              std::isfinite(result.diagnostics
                                .candidate_covariance_normalized_residual),
          "joint graph and bounded validation must report monocular "
          "reprojection and covariance-normalized residual evidence");
  require(result.diagnostics.candidate_closed_loop_correction_applied &&
              result.diagnostics.candidate_closed_loop_update_count > 0 &&
              result.diagnostics.candidate_consumed_fc_event_count > 0 &&
              result.diagnostics.candidate_accepted_fc_event_count > 0 &&
              result.diagnostics.candidate_propagated_imu_step_count > 0 &&
              result.diagnostics.candidate_duplicate_fc_event_count == 0 &&
              result.diagnostics.candidate_duplicate_imu_step_count == 0 &&
              result.diagnostics.candidate_attitude_feedback_allowed &&
              result.diagnostics.candidate_feedback_tier != "PV_ONLY" &&
              result.diagnostics.candidate_covariance_joseph_update_applied &&
              result.diagnostics.candidate_covariance_error_reset_applied &&
              result.diagnostics.candidate_persistent_error.allFinite() &&
              result.diagnostics.candidate_persistent_std.allFinite() &&
              (result.diagnostics.candidate_covariance_health == "healthy" ||
               result.diagnostics.candidate_covariance_health ==
                   "roundoff_repaired"),
          "release must be supported by persistent q/p/v convergence, "
          "feedback/reset evidence, and monotonic FC/IMU cursors");
  require(std::isfinite(
              result.diagnostics.candidate_post_correction_attitude_residual_deg) &&
              std::isfinite(
                  result.diagnostics.candidate_post_correction_position_residual_m) &&
              std::isfinite(
                  result.diagnostics.candidate_post_correction_velocity_residual_mps) &&
              result.diagnostics.candidate_post_correction_position_residual_m <=
                  result.diagnostics.candidate_pre_correction_position_residual_m +
                      1.0e-9 &&
              result.diagnostics.candidate_post_correction_velocity_residual_mps <=
                  result.diagnostics.candidate_pre_correction_velocity_residual_mps +
                      1.0e-9,
          "same-time closed-loop feedback must reduce or preserve the FC "
          "position/velocity residuals before release");
  require(result.diagnostics.fc_attitude_gauge_factor_enabled &&
              result.diagnostics.fc_attitude_gauge_factor_count == 1 &&
              std::isfinite(
                  result.diagnostics.fc_attitude_gauge_sigma_deg) &&
              result.diagnostics.fc_attitude_gauge_sigma_deg > 0.0 &&
              !result.diagnostics.candidate_fc_attitude_evaluation_only &&
              std::isfinite(
                  result.diagnostics.candidate_fc_imu_yaw_residual_deg),
          "FC yaw must provide graph gauge and causal holdout support");
  require(result.timestamp > initializer.current_candidate().result.timestamp &&
              result.timestamp + options.camera_to_imu_time_offset_s >
                  candidate_board_time &&
              result.diagnostics.candidate_release_propagation_duration_s >
                  0.0,
          "release must use a causally advanced state after supported "
          "closed-loop feedback, without a fixed-duration success gate");
  require(result.diagnostics.candidate_refinement_count <= 1,
          "candidate refinement must remain zero or one per startup");
  for (const std::string &family : {"fc_pose_velocity_attitude",
                                    "imu_preintegration",
                                    "visual_reprojection"}) {
    const FactorContribution *factor = find_factor(result.diagnostics, family);
    require(factor != nullptr && factor->residual_blocks > 0 &&
                factor->residual_dimension > 0 &&
                std::isfinite(factor->residual_rms) &&
                factor->jacobian_frobenius > 1e-8,
            family + " must have real residual and Jacobian influence");
  }
  require(result.diagnostics.state_observability.size() == 6,
          "all six released state groups must have observability records");
  for (const auto &state : result.diagnostics.state_observability) {
    if (state.state == "startup_misalignment") {
      require(state.estimate_status ==
                  EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION,
              "startup_misalignment must retain external calibration");
    } else {
      require(state.observable && !state.prior_only,
              state.state + " must be observable and not prior-only");
    }
    if (state.state == "attitude")
      require(state.fc_residual_count > 0,
              "FC yaw gauge must support final attitude through the joint "
              "IMU-chain information");
  }
  bool saw_failed = false;
  bool saw_retry_collecting = false;
  bool saw_aligning = false;
  bool saw_validating = false;
  bool saw_ready = false;
  for (const auto &transition : result.diagnostics.state_transitions) {
    saw_failed = saw_failed || transition.to == AlignmentPhase::FAILED_WAIT_RETRY;
    saw_retry_collecting = saw_retry_collecting ||
                           (transition.from == AlignmentPhase::FAILED_WAIT_RETRY &&
                            transition.to == AlignmentPhase::COLLECTING);
    saw_aligning = saw_aligning || transition.to == AlignmentPhase::ALIGNING;
    saw_validating = saw_validating || transition.to == AlignmentPhase::VALIDATING;
    saw_ready = saw_ready ||
                transition.to == AlignmentPhase::FULL_ALIGNMENT_READY;
  }
  require(saw_failed && saw_retry_collecting && saw_aligning && saw_validating &&
              saw_ready,
          "failed window must retry through ALIGNING/VALIDATING to full alignment");
  require(result.covariance.allFinite(), "covariance must be finite");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> eig(
      result.covariance);
  require(eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() > 0.0,
          "covariance must be positive definite");
  require(!initializer.try_initialize(6.2, result),
          "atomic initializer must release at most once");
  require(initializer.last_rejection() == "alignment_already_released",
          "second release must be named");
  require(initializer.last_diagnostics().nonlinear_solve_attempt_count >= 1 &&
              initializer.last_diagnostics().successful_release_count == 1 &&
              initializer.last_diagnostics().post_release_try_count == 1,
          "one-shot counters must distinguish solve, release, and rejected post-release call");
}

void test_formal_candidate_holdout_and_single_refinement_release() {
  OnlineAlignmentOptions options = make_options();
  options.formal_causal_lifecycle = true;
  options.candidate_window_durations_s = {5.0};
  options.reference_window_duration_s = 5.0;
  options.candidate_short_validation_duration_s = 2.0;
  options.sliding_window_min_advance_s = 0.5;
  options.max_initial_clones = 0;
  options.max_initial_slam_features = 0;
  const SyntheticMotion motion;
  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(3.0 * kPi / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(-2.0 * kPi / 180.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  const Eigen::Vector3d bg(0.012, -0.008, 0.006);
  const Eigen::Vector3d ba(0.12, -0.08, 0.05);
  const double attitude_offset = 0.024;
  options.fc_board_calibration_locked = true;
  options.R_FtoI_declared = R_mount;
  options.fc_attitude_to_board_time_offset_s = attitude_offset;
  options.fc_navigation_to_board_time_offset_s = 0.0;

  OnlineAlignmentInitializer initializer(options);
  int frame_index = 0;
  const auto feed_tick = [&](int tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = board_time;
      fc.position_G = position_G(board_time);
      fc.velocity_G = velocity_G(board_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(board_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "formal FC feed");
    }
    const double fc_attitude_time =
        std::max(0.0, board_time - attitude_offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity =
        R_mount * omega_F(fc_attitude_time) + bg;
    const Eigen::Matrix3d R_GtoI =
        R_mount * motion.rotation(fc_attitude_time);
    imu.linear_acceleration =
        R_GtoI * (acceleration_G(fc_attitude_time) + options.gravity_G) + ba;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "formal IMU feed");
    if (tick % 20 == 0 && board_time >= 0.1) {
      const double camera_time =
          board_time - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, R_mount,
                  options.camera_to_imu_time_offset_s, attitude_offset,
                  frame_index++)),
              "formal visual feed");
    }
  };

  for (int tick = 0; tick <= 1240; ++tick)
    feed_tick(tick);
  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result) &&
              initializer.candidate_active(),
          "formal finite graph must freeze a candidate before release");
  const double candidate_camera_time =
      initializer.current_candidate().result.timestamp;
  const int solves_at_candidate =
      initializer.last_diagnostics().nonlinear_solve_attempt_count;
  bool released = false;
  for (int tick = 1241; tick <= 2200 && !released; ++tick) {
    feed_tick(tick);
    if (tick % 20 == 0) {
      const double camera_time =
          0.005 * tick - options.camera_to_imu_time_offset_s;
      released = initializer.try_initialize(camera_time, result);
    }
  }

  require(released && result.released_to_openvins,
          "passed causal holdout must release the single refined graph");
  require(result.diagnostics.formal_candidate_holdout_passed &&
              result.diagnostics.formal_candidate_holdout_duration_s +
                      1.0e-9 >=
                  options.candidate_short_validation_duration_s &&
              result.diagnostics.formal_refinement_release &&
              result.diagnostics.candidate_refinement_count == 1,
          "formal receipt must prove two-second holdout and exactly one "
          "refinement: passed=" +
              std::to_string(
                  result.diagnostics.formal_candidate_holdout_passed) +
              ", duration=" +
              std::to_string(
                  result.diagnostics.formal_candidate_holdout_duration_s) +
              ", release=" +
              std::to_string(result.diagnostics.formal_refinement_release) +
              ", refinements=" +
              std::to_string(
                  result.diagnostics.candidate_refinement_count));
  require(result.diagnostics.nonlinear_solve_attempt_count ==
                  solves_at_candidate + 1 &&
              !result.diagnostics.candidate_closed_loop_correction_applied &&
              result.diagnostics.candidate_closed_loop_update_count == 0,
          "holdout must not perform sequential FC feedback or extra solves");
  require(result.timestamp > candidate_camera_time + 1.5 &&
              result.diagnostics.decision_time >= result.timestamp,
          "release must move to the current causal camera horizon");
  require(result.diagnostics.shared_window_bias_model &&
              result.diagnostics.visual_statistics
                      .triangulated_landmark_count == 0 &&
              result.diagnostics.visual_pairing_policy ==
                  "one_maximum_time_baseline_pair_per_feature_no_pixel_reuse" &&
              result.diagnostics.fc_terminal_increment_max_correlation > 0.0,
          "formal graph topology and correlated FC model must be observable in diagnostics");
  require(!initializer.attempt_receipts().empty() &&
              initializer.attempt_receipts().back().outcome ==
                  "formal_refined_release",
          "final receipt must identify the one refined atomic release");
}

void feed_minimal_nonvisual_streams(OnlineAlignmentInitializer &initializer,
                                    const OnlineAlignmentOptions &options,
                                    bool include_stereo) {
  const SyntheticMotion motion;
  for (int tick = 0; tick <= 1240; ++tick) {
    const double t = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = t;
      fc.position_G = position_G(t);
      fc.velocity_G = velocity_G(t);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(t));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "negative-control FC feed");
    }
    BoardImuSample imu;
    imu.timestamp = t;
    imu.angular_velocity = omega_F(t);
    imu.linear_acceleration =
        motion.rotation(t) * (acceleration_G(t) + options.gravity_G);
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "negative-control IMU feed");
    if (include_stereo && tick % 20 == 0 && t >= 0.1) {
      const double camera_time = t - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, Eigen::Matrix3d::Identity(),
                  options.camera_to_imu_time_offset_s, 0.0, tick / 20)),
              "negative-control stereo feed");
    }
  }
}

bool validate_identity_candidate(OnlineAlignmentInitializer &initializer,
                                  const OnlineAlignmentOptions &options,
                                  OnlineAlignmentResult &result,
                                  double transient_fc_flex_deg = 0.0) {
  const SyntheticMotion motion;
  if (!initializer.candidate_active()) {
    require(!initializer.try_initialize(6.2, result),
            "joint solve must wait for candidate validation");
  }
  for (int tick = 1241; tick <= 5400; ++tick) {
    const double t = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = t;
      fc.position_G = position_G(t);
      fc.velocity_G = velocity_G(t);
      const Eigen::Matrix3d transient_flex =
          Eigen::AngleAxisd(transient_fc_flex_deg * kPi / 180.0,
                            Eigen::Vector3d::UnitX())
              .toRotationMatrix();
      fc.q_GtoF = ov_core::rot_2_quat(transient_flex * motion.rotation(t));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "candidate validation FC");
    }
    BoardImuSample imu;
    imu.timestamp = t;
    imu.angular_velocity = omega_F(t);
    imu.linear_acceleration =
        motion.rotation(t) * (acceleration_G(t) + options.gravity_G);
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "candidate validation IMU");
    if (tick % 20 == 0) {
      const double camera_time = t - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, Eigen::Matrix3d::Identity(),
                  options.camera_to_imu_time_offset_s, 0.0, tick / 20)),
              "candidate validation visual");
      if (initializer.try_initialize(camera_time, result))
        return true;
    }
  }
  return false;
}

void test_turn_flex_roll_pitch_is_diagnostic_while_yaw_is_control() {
  OnlineAlignmentOptions options = make_options();
  // These tight gates prove that a large roll/pitch-only flex remains a
  // diagnostic while the yaw holdout still controls navigation release.
  options.candidate_max_fc_imu_rotation_residual_deg = 0.05;
  options.candidate_reject_fc_imu_rotation_residual_deg = 0.10;
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);
  OnlineAlignmentResult result;
  require(validate_identity_candidate(initializer, options, result, 12.0),
          "transient turn flex must not reject an otherwise valid navigation "
          "initialization");
  require(result.diagnostics.candidate_fc_imu_rotation_residual_deg > 5.0 &&
              result.diagnostics.candidate_fc_imu_yaw_residual_deg <= 0.05 &&
              result.diagnostics.fc_attitude_gauge_factor_count == 1 &&
              !result.diagnostics.candidate_fc_attitude_evaluation_only,
          "roll/pitch flex must remain visible without defeating the active "
          "FC yaw graph and holdout contract");
}

void test_pre_candidate_turn_flex_does_not_bias_initial_graph() {
  OnlineAlignmentOptions options = make_options();
  const SyntheticMotion motion;
  const double flex_rad = 20.0 * kPi / 180.0;
  OnlineAlignmentInitializer initializer(options);
  int frame_index = 0;
  for (int tick = 0; tick <= 1240; ++tick) {
    const double t = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = t;
      fc.position_G = position_G(t);
      fc.velocity_G = velocity_G(t);
      const Eigen::Matrix3d flex =
          (t >= 1.5 && t <= 3.5)
              ? Eigen::AngleAxisd(flex_rad, Eigen::Vector3d::UnitX())
                    .toRotationMatrix()
              : Eigen::Matrix3d::Identity();
      fc.q_GtoF = ov_core::rot_2_quat(flex * motion.rotation(t));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc),
              "pre-candidate flex FC feed");
    }
    BoardImuSample imu;
    imu.timestamp = t;
    imu.angular_velocity = omega_F(t);
    imu.linear_acceleration =
        motion.rotation(t) * (acceleration_G(t) + options.gravity_G);
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "pre-candidate flex IMU feed");
    if (tick % 20 == 0 && t >= 0.1) {
      const double camera_time = t - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, Eigen::Matrix3d::Identity(),
                  options.camera_to_imu_time_offset_s, 0.0, frame_index++)),
              "pre-candidate flex visual feed");
    }
  }

  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result) &&
              initializer.candidate_active(),
          "pre-candidate flex stream must form a validation candidate: " +
              initializer.last_rejection());
  const auto &candidate = initializer.current_candidate().result;
  const Eigen::Matrix3d expected_R_GtoI =
      options.R_FtoI_declared *
      motion.rotation(candidate.timestamp + options.camera_to_imu_time_offset_s);
  const Eigen::Matrix3d candidate_R_GtoI =
      ov_core::quat_2_Rot(candidate.q_GtoI);
  const double attitude_error_deg =
      Eigen::AngleAxisd(candidate_R_GtoI * expected_R_GtoI.transpose()).angle() *
      180.0 / kPi;
  require(attitude_error_deg < 3.0,
          "turn flex before candidate formation must not bias initial graph "
          "attitude: " + std::to_string(attitude_error_deg) + " deg");
}

void test_candidate_deadline_refines_only_on_new_window() {
  OnlineAlignmentOptions options = make_options();
  options.release_policy = AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  CandidateGroupGate &position_gate =
      options.candidate_filter_config.group_gates[static_cast<std::size_t>(
          CandidateStateGroup::POSITION)];
  position_gate.max_std = Eigen::Vector3d::Constant(1.0e-12);
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);

  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result) &&
              initializer.candidate_active(),
          "deadline test must start from a solved candidate");
  const SyntheticMotion motion;
  int frame_index = 63;
  bool refinement_observed = false;
  bool candidate_ready_after_refinement = false;
  for (int tick = 1241;
       tick <= 2400 && !candidate_ready_after_refinement; ++tick) {
    const double t = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = t;
      fc.position_G = position_G(t);
      fc.velocity_G = velocity_G(t);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(t));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc),
              "deadline refinement FC feed");
    }
    BoardImuSample imu;
    imu.timestamp = t;
    imu.angular_velocity = omega_F(t);
    imu.linear_acceleration =
        motion.rotation(t) * (acceleration_G(t) + options.gravity_G);
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu),
            "deadline refinement IMU feed");
    if (tick % 20 != 0)
      continue;
    const double camera_time = t - options.camera_to_imu_time_offset_s;
    require(initializer.feed_stereo(make_stereo_frame(
                camera_time, motion, Eigen::Matrix3d::Identity(),
                options.camera_to_imu_time_offset_s, 0.0, frame_index++)),
            "deadline refinement visual feed");
    require(!initializer.try_initialize(camera_time, result),
            "impossible position covariance gate must not release");
    const auto &receipts = initializer.attempt_receipts();
    for (const auto &receipt : receipts) {
      if (receipt.trigger == "single_candidate_refinement")
        refinement_observed = true;
      if (refinement_observed && receipt.outcome == "candidate_ready")
        candidate_ready_after_refinement = true;
    }
  }

  const auto &receipts = initializer.attempt_receipts();
  std::string receipt_summary;
  for (const auto &receipt : receipts)
    receipt_summary += " [trigger=" + receipt.trigger +
                       " outcome=" + receipt.outcome +
                       " gate=" + receipt.failed_gate + "]";
  std::vector<std::size_t> refinement_indices;
  for (std::size_t index = 0; index < receipts.size(); ++index) {
    if (receipts[index].trigger == "single_candidate_refinement")
      refinement_indices.push_back(index);
  }
  require(refinement_observed && candidate_ready_after_refinement &&
              refinement_indices.size() == 1 &&
              refinement_indices.front() > 0,
          "candidate deadline must attempt exactly one bounded refinement and "
          "remain open for a later advanced candidate:" + receipt_summary);
  const std::size_t refinement_index = refinement_indices.front();
  const auto &original = receipts[refinement_index - 1];
  const auto &refinement = receipts[refinement_index];
  require(original.outcome == "candidate_rejected" &&
              contains(original.failed_gate, "candidate_validation_deadline") &&
              refinement.window_fingerprint != original.window_fingerprint &&
              refinement.window_end_timestamp >=
                  original.window_end_timestamp +
                      options.sliding_window_min_advance_s - 1.0e-9 &&
              refinement.warm_start_used,
          "refinement must use a unique newly advanced window and warm start");
  if (refinement.outcome != "candidate_ready") {
    require(refinement.outcome == "rejected_window" &&
                contains(refinement.failed_gate,
                         "navigation_state_quality"),
            "a quality-rejected bounded refinement must name the failed "
            "navigation gate");
    bool later_candidate_ready = false;
    for (std::size_t index = refinement_index + 1; index < receipts.size();
         ++index)
      later_candidate_ready =
          later_candidate_ready || receipts[index].outcome == "candidate_ready";
    require(later_candidate_ready,
            "a rejected bounded refinement must not prevent a later normal "
            "advanced window from creating a candidate");
  }
  require(initializer.last_diagnostics().candidate_refinement_count == 1 &&
              initializer.last_diagnostics().successful_release_count == 0 &&
              !initializer.alignment_window_closed(),
          "refinement is startup-wide bounded and cannot release a failed gate");
}

void test_no_visual_fail_closed() {
  OnlineAlignmentOptions options = make_options();
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, false);
  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result),
          "no-visual control must not release");
  require(initializer.phase() == AlignmentPhase::FAILED_WAIT_RETRY &&
              contains(initializer.last_rejection(), "no_visual"),
          "no-visual control must fail closed with a named gate");
}

void test_practical_navigation_start_with_weak_alignment() {
  OnlineAlignmentOptions options = make_options();
  options.min_angular_excitation_rad_s = 1.0;
  options.release_policy = AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);
  OnlineAlignmentResult result;
  const bool navigation_released =
      validate_identity_candidate(initializer, options, result);
  if (!navigation_released) {
    const auto &diagnostics = initializer.last_diagnostics();
    std::cerr << "navigation gates:";
    for (const auto &gate : diagnostics.navigation_failed_gates)
      std::cerr << " " << gate;
    std::cerr << " pre_p="
              << diagnostics.candidate_pre_correction_position_residual_m
              << " post_p="
              << diagnostics.candidate_relative_position_residual_m
              << " pre_v="
              << diagnostics.candidate_pre_correction_velocity_residual_mps
              << " post_v="
              << diagnostics.candidate_relative_velocity_residual_mps
              << " dq="
              << diagnostics.candidate_closed_loop_attitude_correction_deg
              << " dp="
              << diagnostics.candidate_closed_loop_position_correction_m
              << " dv="
              << diagnostics.candidate_closed_loop_velocity_correction_mps
              << " dbg="
              << diagnostics.candidate_closed_loop_gyro_bias_correction_rad_s
              << " dba="
              << diagnostics.candidate_closed_loop_accel_bias_correction_mps2
              << " candidates=" << diagnostics.candidate_created_count
              << " rejected=" << diagnostics.candidate_rejected_count
              << std::endl;
    for (const auto &receipt : initializer.attempt_receipts())
      std::cerr << "  practical receipt outcome=" << receipt.outcome
                << " gate=" << receipt.failed_gate
                << " next=" << receipt.next_eligible_condition << std::endl;
  }
  require(navigation_released,
          "practical policy must allow navigation start under weak excitation: " +
              initializer.last_rejection());
  require(result.readiness == AlignmentReadiness::FULL_ALIGNMENT_READY &&
              result.released_to_openvins,
          "low startup excitation must not downgrade accepted external calibration");
  require(result.diagnostics.alignment_window_closed &&
              result.diagnostics.successful_release_count == 1 &&
              result.diagnostics.nonlinear_solve_attempt_count >= 1 &&
              result.diagnostics.post_release_try_count == 0,
          "practical release must atomically close the finite startup window");
  require(!result.diagnostics.startup_mount_fixed_to_prior &&
              !result.diagnostics.startup_time_offset_fixed_to_prior,
          "external calibration must not be mislabeled as a startup prior");
  require(contains(
              ov_msckf::estimate_source_status_name(
                  result.diagnostics.startup_misalignment_status),
              "fixed_external_calibration"),
          "startup mounting must retain external-calibration lineage");
  const int solves_after_release =
      result.diagnostics.nonlinear_solve_attempt_count;
  OnlineAlignmentResult duplicate;
  require(!initializer.try_initialize(6.2, duplicate),
          "navigation release must be atomic and final");
  require(initializer.phase() == AlignmentPhase::FULL_ALIGNMENT_READY,
          "externally calibrated run must remain fully ready");
  const size_t fc_size = initializer.fc_buffer_size();
  const size_t imu_size = initializer.imu_buffer_size();
  const size_t visual_size = initializer.stereo_buffer_size();
  FCNavigationSample late_fc;
  late_fc.timestamp = 6.3;
  late_fc.position_G = position_G(late_fc.timestamp);
  late_fc.velocity_G = velocity_G(late_fc.timestamp);
  late_fc.q_GtoF = ov_core::rot_2_quat(SyntheticMotion().rotation(late_fc.timestamp));
  late_fc.navigation_frame = "G_nav";
  late_fc.body_frame = "FC_body";
  late_fc.position_valid = late_fc.velocity_valid = late_fc.attitude_valid =
      late_fc.status_valid = true;
  BoardImuSample late_imu;
  late_imu.timestamp = 6.3;
  late_imu.angular_velocity = omega_F(late_imu.timestamp);
  late_imu.linear_acceleration = SyntheticMotion().rotation(late_imu.timestamp) *
                                 (acceleration_G(late_imu.timestamp) +
                                  options.gravity_G);
  late_imu.frame = "board_imu";
  late_imu.status_valid = true;
  const StereoAlignmentFrame late_visual = make_stereo_frame(
      6.3, SyntheticMotion(), Eigen::Matrix3d::Identity(),
      options.camera_to_imu_time_offset_s, 0.0, 630);
  require(!initializer.feed_fc_navigation(late_fc) &&
              !initializer.feed_board_imu(late_imu) &&
              !initializer.feed_stereo(late_visual),
          "released initializer must reject later FC, IMU, and visual samples");
  require(initializer.fc_buffer_size() == fc_size &&
              initializer.imu_buffer_size() == imu_size &&
              initializer.stereo_buffer_size() == visual_size,
          "released initializer buffers must remain frozen");
  require(initializer.last_diagnostics().nonlinear_solve_attempt_count ==
              solves_after_release &&
              initializer.last_diagnostics().successful_release_count == 1,
          "post-release traffic must not re-enter the nonlinear solver");
}

void test_missing_full_flight_calibration_fails_closed() {
  OnlineAlignmentOptions options = make_options();
  options.fc_board_calibration_locked = false;
  OnlineAlignmentInitializer initializer(options);
  OnlineAlignmentResult result;
  require(!initializer.try_initialize(6.2, result),
          "missing full-flight calibration must fail closed");
  require(initializer.phase() == AlignmentPhase::FATAL_CONFIGURATION_ERROR &&
              contains(initializer.last_rejection(),
                       "locked_full_flight_fc_board_calibration_required"),
          "missing calibration must have a fatal named rejection");
}

void test_collection_persists_beyond_sixty_seconds_with_bounded_buffers() {
  OnlineAlignmentOptions options = make_options();
  options.navigation_max_collection_s = 1.0;
  OnlineAlignmentInitializer initializer(options);
  OnlineAlignmentResult result;
  for (int tick = 0; tick <= 6500; ++tick) {
    const double t = 0.01 * tick;
    BoardImuSample imu;
    imu.timestamp = t;
    imu.angular_velocity.setZero();
    imu.linear_acceleration = options.gravity_G;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "persistent IMU feed");
    if (tick % 100 == 0)
      require(!initializer.try_initialize(t, result),
              "no-visual stream must not release");
  }
  require(!initializer.alignment_window_closed(),
          "elapsed wall/sensor time must never close persistent collection");
  require(initializer.imu_buffer_size() < 1500,
          "persistent collection must prune to the maximum candidate window");
}

void test_strict_policy_accepts_external_mount_calibration() {
  OnlineAlignmentOptions options = make_options();
  options.min_angular_excitation_rad_s = 1.0;
  options.release_policy = AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT;
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);
  OnlineAlignmentResult result;
  require(validate_identity_candidate(initializer, options, result),
          "strict policy must accept independently calibrated mount/time");
  require(result.readiness == AlignmentReadiness::FULL_ALIGNMENT_READY,
          "strict policy must release only the fully observed navigation state");
}

void test_no_visual_navigation_control() {
  OnlineAlignmentOptions options = make_options();
  OnlineAlignmentInitializer visual_initializer(options);
  feed_minimal_nonvisual_streams(visual_initializer, options, true);
  OnlineAlignmentResult visual_result;
  require(validate_identity_candidate(visual_initializer, options,
                                      visual_result),
          "same-time visual baseline must release");

  options.visual_factors_enabled = false;
  options.navigation_allow_without_visual = true;
  options.release_policy = AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);
  OnlineAlignmentResult result;
  require(validate_identity_candidate(initializer, options, result),
          "explicit no-vision control may release navigation-only: " +
              initializer.last_rejection());
  require(result.readiness == AlignmentReadiness::NAVIGATION_READY,
          "no-vision control must never claim full visual joint alignment");
  const FactorContribution *visual =
      find_factor(result.diagnostics, "visual_reprojection");
  require(visual != nullptr && visual->residual_blocks == 0 &&
              visual->jacobian_frobenius == 0.0,
          "no-vision control must contain zero visual residual/Jacobian contribution");
  const double state_difference =
      (result.q_GtoI - visual_result.q_GtoI).norm() +
      (result.p_IinG - visual_result.p_IinG).norm() +
      (result.v_IinG - visual_result.v_IinG).norm() +
      (result.bg - visual_result.bg).norm() +
      (result.ba - visual_result.ba).norm();
  const double covariance_difference =
      (result.covariance - visual_result.covariance).norm();
  require(state_difference > 1e-6 || covariance_difference > 1e-6 ||
              result.readiness != visual_result.readiness,
          "same-time vision switch must change state, covariance, or readiness");
}

void test_perturbed_visual_tracks_change_decision() {
  OnlineAlignmentOptions options = make_options();
  options.visual_perturbation_px = 35.0;
  options.visual_perturbation_fraction = 0.5;
  OnlineAlignmentInitializer initializer(options);
  feed_minimal_nonvisual_streams(initializer, options, true);
  OnlineAlignmentResult result;
  const bool released = initializer.try_initialize(6.2, result);
  require(!released || result.readiness != AlignmentReadiness::FULL_ALIGNMENT_READY,
          "severely perturbed tracks must not pass full alignment");
  require(contains(initializer.last_rejection(), "visual") ||
              initializer.last_diagnostics()
                      .visual_statistics.reprojection_rmse_px >
                  options.navigation_max_reprojection_rmse_px,
          "perturbed visual control must raise a visual residual or quality gate");
  if (!initializer.candidate_active()) {
    const int solve_count =
        initializer.last_diagnostics().nonlinear_solve_attempt_count;
    require(!initializer.try_initialize(6.2, result) &&
                initializer.last_diagnostics().nonlinear_solve_attempt_count ==
                    solve_count,
            "duplicate fingerprint must not enter Ceres twice");
  }
}

void test_stale_visual_tail_is_pruned() {
  OnlineAlignmentOptions options = make_options();
  options.candidate_window_durations_s = {3.0};
  options.reference_window_duration_s = 3.0;
  options.buffer_margin_s = 0.25;
  options.max_time_offset_s = 0.10;
  require(options.minimum_solve_interval_s >= 0.75 &&
              options.minimum_new_selected_frames_for_resolve >= 3,
          "default solve eligibility must not permit per-frame Ceres retries");
  require(options.alignment_frame_maximum_interval_s < options.max_stereo_gap_s,
          "alignment tail selection needs sampling headroom below the visual gap gate");

  OnlineAlignmentInitializer initializer(options);
  const SyntheticMotion motion;
  StereoAlignmentFrame frame = make_stereo_frame(
      1.0, motion, Eigen::Matrix3d::Identity(),
      options.camera_to_imu_time_offset_s, 0.0, 0);
  require(initializer.feed_stereo(frame) && initializer.stereo_buffer_size() == 1,
          "first valid alignment frame must enter the bounded visual buffer");

  BoardImuSample imu;
  imu.timestamp = 20.0;
  imu.angular_velocity = Eigen::Vector3d::Zero();
  imu.linear_acceleration = options.gravity_G;
  imu.frame = "board_imu";
  imu.status_valid = true;
  require(initializer.feed_board_imu(imu), "valid late IMU sample");
  require(initializer.stereo_buffer_size() == 0,
          "all visual frames outside the maximum solve horizon must be removed");
}

void test_r1_shadow_rebuilds_advancing_fixed_time_windows() {
  OnlineAlignmentOptions options = make_options();
  options.sliding_window_shadow_only = true;
  options.sliding_window_duration_s = 5.0;
  options.sliding_window_min_advance_s = 0.5;
  options.candidate_window_durations_s = {5.0};
  options.alignment_frame_minimum_interval_s = 0.05;
  options.alignment_target_compensated_parallax_px = 0.0;
  const SyntheticMotion motion;
  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(3.0 * kPi / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(-2.0 * kPi / 180.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  const Eigen::Vector3d bg(0.012, -0.008, 0.006);
  const Eigen::Vector3d ba(0.12, -0.08, 0.05);
  const double offset = 0.024;
  options.R_FtoI_declared = R_mount;
  options.fc_attitude_to_board_time_offset_s = offset;
  options.fc_navigation_to_board_time_offset_s = 0.0;

  OnlineAlignmentInitializer initializer(options);
  int frame_index = 0;
  int returned_shadow_window_count = 0;
  for (int tick = 0; tick <= 1800; ++tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = board_time;
      fc.position_G = position_G(board_time);
      fc.velocity_G = velocity_G(board_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(board_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "R1 FC feed");
    }
    const double fc_time_for_imu = std::max(0.0, board_time - offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity = R_mount * omega_F(fc_time_for_imu) + bg;
    const Eigen::Matrix3d R_GtoI =
        R_mount * motion.rotation(fc_time_for_imu);
    imu.linear_acceleration =
        R_GtoI * (acceleration_G(fc_time_for_imu) + options.gravity_G) + ba;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "R1 IMU feed");

    if (tick % 20 == 0 && board_time >= 0.1) {
      const double camera_time =
          board_time - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, R_mount,
                  options.camera_to_imu_time_offset_s, offset, frame_index++)),
              "R1 visual feed");
      OnlineAlignmentResult shadow;
      if (initializer.try_initialize(camera_time, shadow)) {
        ++returned_shadow_window_count;
        require(!shadow.released_to_openvins &&
                    shadow.readiness == AlignmentReadiness::NOT_READY &&
                    shadow.diagnostics.solver_converged,
                "R1 must expose solved windows without formally releasing them");
        const FactorContribution *fc_factor =
            find_factor(shadow.diagnostics, "fc_pose_velocity_attitude");
        const auto &receipt = initializer.attempt_receipts().back();
        require(
            fc_factor != nullptr &&
                fc_factor->residual_blocks ==
                    2 * receipt.selected_keyframe_count &&
                shadow.diagnostics.fc_attitude_gauge_factor_count == 1 &&
                shadow.diagnostics.fc_position_velocity_factor_count ==
                    receipt.selected_keyframe_count &&
                std::fabs(shadow.diagnostics
                              .fc_position_velocity_time_weight_sum -
                          1.0) < 1.0e-12 &&
                shadow.diagnostics.fc_position_velocity_weight_model ==
                    "terminal_absolute_plus_density_invariant_increments" &&
                std::fabs(
                    shadow.diagnostics.fc_attitude_gauge_anchor_timestamp_s -
                    receipt.selected_frame_timestamps.back()) < 1.0e-9 &&
                std::isfinite(
                    shadow.diagnostics.fc_terminal_max_normalized_residual) &&
                shadow.diagnostics.fc_terminal_max_normalized_residual <=
                    options.navigation_max_fc_residual_rms,
            "each R1 graph must preserve a full terminal FC boundary and use "
            "density-invariant FC trajectory increments");
      }
    }
  }

  const auto &windows = initializer.attempt_receipts();
  require(windows.size() >= 3,
          "R1 must invoke Ceres on multiple advanced windows");
  require(returned_shadow_window_count >= 3,
          "R1 must return each solved window to the external gauge supervisor");
  require(!initializer.candidate_active() &&
              !initializer.navigation_released() &&
              !initializer.alignment_window_closed(),
          "R1 must bypass candidate release and keep alignment active");
  double previous_begin = -1.0;
  double previous_end = -1.0;
  std::vector<double> previous_selected;
  int expected_window = 1;
  int expected_invocation = 1;
  bool observed_warm_start = false;
  bool observed_more_than_legacy_visual_cap = false;
  bool observed_bias_trend_diagnostic = false;
  for (const auto &window : windows) {
    require(window.window_id == expected_window &&
                window.optimizer_invocation_index == expected_invocation &&
                window.optimizer_invocation_count == 1,
            "R1 windows must contain one standard dynamic graph solve");
    require(window.shared_window_bias_model &&
                !window.staged_solver_enabled,
            "R1 receipts must prove one shared bg/ba pair and one joint solve");
    require(std::fabs((window.window_end_timestamp -
                       window.window_begin_timestamp) -
                      options.sliding_window_duration_s) < 1.0e-6,
            "R1 window duration must remain fixed in time");
    require(previous_end < 0.0 ||
                window.window_end_timestamp > previous_end + 0.49,
            "R1 window end must advance by the time trigger");
    require(previous_begin < 0.0 ||
                window.window_begin_timestamp > previous_begin + 0.49,
            "R1 window begin must advance with the end");
    require(!window.selected_frame_timestamps.empty() &&
                window.selected_frame_timestamps.front() >=
                    window.window_begin_timestamp - 1.0e-9 &&
                window.selected_frame_timestamps.back() <=
                    window.window_end_timestamp + 1.0e-9,
            "R1 selected keyframes must lie inside the current window");
    if (!previous_selected.empty())
      require(window.selected_frame_timestamps != previous_selected,
              "R1 must reselect keyframes for each advanced window");
    require(window.q_GtoI.allFinite() && window.p_IinG.allFinite() &&
                window.v_IinG.allFinite() && window.bg.allFinite() &&
                window.ba.allFinite(),
            "R1 must record a finite q/p/v/bg/ba estimate per window");
    observed_warm_start = observed_warm_start || window.warm_start_used;
    observed_more_than_legacy_visual_cap =
        observed_more_than_legacy_visual_cap || window.visual_frame_count > 36;
    observed_bias_trend_diagnostic =
        observed_bias_trend_diagnostic ||
        (window.bias_trend_sample_count >= 3 &&
         window.bias_trend_span_s > 0.0 &&
         std::isfinite(window.bias_trend_normalized_max_sigma));
    previous_begin = window.window_begin_timestamp;
    previous_end = window.window_end_timestamp;
    previous_selected = window.selected_frame_timestamps;
    ++expected_window;
    expected_invocation += 1;
  }
  require(observed_warm_start,
          "R1 later windows must warm start from the previous solution");
  require(observed_more_than_legacy_visual_cap,
          "R1 time window must retain all selected visual measurements instead "
          "of truncating the raw buffer at the legacy 36-frame cap");
  require(observed_bias_trend_diagnostic,
          "R1 must compute a causal time-domain shared-bias trend");
  require(initializer.last_diagnostics().nonlinear_solve_attempt_count ==
              static_cast<int>(windows.size()),
           "R1 diagnostics must expose every nonlinear invocation");
}

void test_direct_sliding_release_uses_overlap_time_not_event_count() {
  OnlineAlignmentOptions options = make_options();
  options.sliding_window_shadow_only = true;
  options.sliding_window_direct_state_release = true;
  options.sliding_window_duration_s = 5.0;
  options.sliding_window_min_advance_s = 0.5;
  options.sliding_window_overlap_stability_duration_s = 1.0;
  // Keep the trend numerically non-limiting on this exact synthetic motion,
  // while still requiring it to cover the same finite sensor-time interval.
  // Release must intersect the two parallel clocks instead of waiting another
  // complete duration after the trend itself becomes valid.
  options.sliding_window_bias_trend_max_normalized_sigma = 100.0;
  options.sliding_window_min_overlap_states = 3;
  options.candidate_window_durations_s = {5.0};
  options.alignment_frame_minimum_interval_s = 0.05;
  options.alignment_target_compensated_parallax_px = 0.0;
  const SyntheticMotion motion;
  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(3.0 * kPi / 180.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix() *
      Eigen::AngleAxisd(-2.0 * kPi / 180.0, Eigen::Vector3d::UnitY())
          .toRotationMatrix();
  const Eigen::Vector3d bg(0.012, -0.008, 0.006);
  const Eigen::Vector3d ba(0.12, -0.08, 0.05);
  const double offset = 0.024;
  options.R_FtoI_declared = R_mount;
  options.fc_attitude_to_board_time_offset_s = offset;
  options.fc_navigation_to_board_time_offset_s = 0.0;

  OnlineAlignmentInitializer initializer(options);
  int frame_index = 0;
  OnlineAlignmentResult released;
  bool release_observed = false;
  for (int tick = 0; tick <= 1600 && !release_observed; ++tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = board_time;
      fc.position_G = position_G(board_time);
      fc.velocity_G = velocity_G(board_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(board_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc), "direct-release FC feed");
    }
    const double fc_time_for_imu = std::max(0.0, board_time - offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity = R_mount * omega_F(fc_time_for_imu) + bg;
    const Eigen::Matrix3d R_GtoI =
        R_mount * motion.rotation(fc_time_for_imu);
    imu.linear_acceleration =
        R_GtoI * (acceleration_G(fc_time_for_imu) + options.gravity_G) + ba;
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu), "direct-release IMU feed");

    if (tick % 20 == 0 && board_time >= 0.1) {
      const double camera_time =
          board_time - options.camera_to_imu_time_offset_s;
      require(initializer.feed_stereo(make_stereo_frame(
                  camera_time, motion, R_mount,
                  options.camera_to_imu_time_offset_s, offset, frame_index++)),
              "direct-release visual feed");
      OnlineAlignmentResult result;
      if (initializer.try_initialize(camera_time, result) &&
          result.released_to_openvins) {
        released = result;
        release_observed = true;
      }
    }
  }

  require(release_observed, "direct sliding release must occur on stable data");
  require(initializer.navigation_released() &&
              initializer.alignment_window_closed(),
          "direct sliding release must atomically close the startup window");
  require(released.diagnostics.direct_sliding_state_release &&
              released.diagnostics.sliding_overlap_consistency_passed,
          "direct release must carry same-timestamp overlap evidence");
  require(released.diagnostics.sliding_overlap_state_count >=
              options.sliding_window_min_overlap_states,
          "direct release must compare enough overlapping graph states");
  require(released.diagnostics.sliding_overlap_stable_duration_s + 1.0e-9 >=
              options.sliding_window_overlap_stability_duration_s,
          "direct release must use elapsed sensor time, not an event quota");
  require(released.diagnostics.sliding_bias_trend_passed &&
              released.diagnostics.sliding_bias_trend_span_s + 1.0e-9 >=
                  options.sliding_window_overlap_stability_duration_s &&
              released.diagnostics.sliding_overlap_stable_duration_s <
                  2.0 * options.sliding_window_overlap_stability_duration_s,
          "overlap and bias stability must be parallel time evidence, not two "
          "serial copies of the same maturation duration");
  require(released.initial_clones.size() >= 4,
          "direct release must carry a mature historical pose window");
  require(released.initial_landmarks.empty(),
          "landmark-free graph must not transfer nuisance visual states");
  require(released.initial_joint_covariance.rows() ==
              15 + 6 * static_cast<Eigen::Index>(
                           released.initial_clones.size()) +
                  3 * static_cast<Eigen::Index>(
                          released.initial_landmarks.size()) &&
              released.initial_joint_covariance.cols() ==
                  released.initial_joint_covariance.rows() &&
              released.initial_joint_covariance.allFinite(),
          "direct release must carry the complete active/clone covariance");
  double previous_clone_time = -1.0;
  for (const AlignmentCloneState &clone : released.initial_clones) {
    require(clone.timestamp > previous_clone_time &&
                clone.timestamp < released.timestamp &&
                clone.q_GtoI.allFinite() && clone.p_IinG.allFinite(),
            "released clone history must be finite, ordered, and historical");
    previous_clone_time = clone.timestamp;
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> joint_eigen(
      released.initial_joint_covariance);
  require(joint_eigen.info() == Eigen::Success &&
              joint_eigen.eigenvalues().minCoeff() > 0.0,
          "released active/clone covariance must be positive definite");
  require(!initializer.attempt_receipts().empty() &&
              initializer.attempt_receipts().back().outcome ==
                  "direct_state_release",
          "the final receipt must identify the direct-state handoff");
}

void test_upstream_dynamic_mode_builds_only_causal_fc_gauge_targets() {
  OnlineAlignmentOptions options = make_options();
  const SyntheticMotion motion;
  const Eigen::Matrix3d R_mount =
      Eigen::AngleAxisd(4.0 * kPi / 180.0, Eigen::Vector3d::UnitX())
          .toRotationMatrix();
  const Eigen::Vector3d gyro_bias(0.01, -0.006, 0.004);
  const double attitude_offset = 0.024;
  options.upstream_dynamic_init_fc_gauge = true;
  options.sliding_window_duration_s = 2.0;
  options.sliding_window_gauge_stability_duration_s = 2.0;
  options.fc_board_calibration_locked = true;
  options.R_FtoI_declared = R_mount;
  options.fc_attitude_to_board_time_offset_s = attitude_offset;
  options.fc_navigation_to_board_time_offset_s = 0.0;
  options.max_rate_residual_rms_rad_s = 0.05;

  OnlineAlignmentInitializer initializer(options);
  for (int tick = 0; tick <= 600; ++tick) {
    const double board_time = 0.005 * tick;
    if (tick % 4 == 0) {
      FCNavigationSample fc;
      fc.timestamp = board_time;
      fc.position_G = position_G(board_time);
      fc.velocity_G = velocity_G(board_time);
      fc.q_GtoF = ov_core::rot_2_quat(motion.rotation(board_time));
      fc.navigation_frame = "G_nav";
      fc.body_frame = "FC_body";
      fc.position_valid = fc.velocity_valid = fc.attitude_valid =
          fc.status_valid = true;
      require(initializer.feed_fc_navigation(fc),
              "FC gauge target test rejected valid FC data");
    }
    const double fc_attitude_time =
        std::max(0.0, board_time - attitude_offset);
    BoardImuSample imu;
    imu.timestamp = board_time;
    imu.angular_velocity =
        R_mount * omega_F(fc_attitude_time) + gyro_bias;
    imu.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81);
    imu.frame = "board_imu";
    imu.status_valid = true;
    require(initializer.feed_board_imu(imu),
            "FC gauge target test rejected valid board IMU data");
  }

  OnlineAlignmentResult early;
  require(!initializer.make_fc_gauge_target(1.0, early),
          "FC gauge target ignored the configured time-window duration");
  OnlineAlignmentResult target;
  const double requested_camera_time =
      3.1 - options.camera_to_imu_time_offset_s;
  require(initializer.make_fc_gauge_target(
              requested_camera_time, target),
          "complete causal FC/IMU window did not produce a gauge target");
  const double board_time =
      target.timestamp + options.camera_to_imu_time_offset_s;
  require(target.timestamp < requested_camera_time &&
              std::fabs(board_time - 3.0) < 1.0e-9,
          "FC gauge target did not retreat to the newest jointly supported causal time");
  const double attitude_time = board_time - attitude_offset;
  const Eigen::Matrix3d expected_R_GtoI =
      R_mount * motion.rotation(attitude_time);
  const Eigen::AngleAxisd target_attitude_error(
      ov_core::quat_2_Rot(target.q_GtoI) * expected_R_GtoI.transpose());
  require(std::fabs(target_attitude_error.angle()) < 1.0e-4,
          "FC gauge target attitude does not use locked mounting and causal time");
  require(target.p_IinG.isApprox(position_G(board_time), 1.0e-6),
          "FC gauge target position is incorrect");
  require(target.v_IinG.isApprox(velocity_G(board_time), 1.0e-6),
          "FC gauge target velocity is incorrect");
  require(target.diagnostics.navigation_quality_passed &&
              !target.diagnostics.solver_converged &&
              target.diagnostics.nonlinear_solve_attempt_count == 0,
          "low-dimensional target path falsely reported a nonlinear solve");
}

} // namespace

int main() {
  test_rejections_and_causal_recovery();
  test_formal_candidate_holdout_and_single_refinement_release();
  test_no_visual_fail_closed();
  test_practical_navigation_start_with_weak_alignment();
  test_missing_full_flight_calibration_fails_closed();
  test_collection_persists_beyond_sixty_seconds_with_bounded_buffers();
  test_strict_policy_accepts_external_mount_calibration();
  test_turn_flex_roll_pitch_is_diagnostic_while_yaw_is_control();
  test_pre_candidate_turn_flex_does_not_bias_initial_graph();
  test_candidate_deadline_refines_only_on_new_window();
  test_no_visual_navigation_control();
  test_perturbed_visual_tracks_change_decision();
  test_stale_visual_tail_is_pruned();
  test_r1_shadow_rebuilds_advancing_fixed_time_windows();
  test_direct_sliding_release_uses_overlap_time_not_event_count();
  test_upstream_dynamic_mode_builds_only_causal_fc_gauge_targets();
  std::cout << "online alignment initializer tests passed" << std::endl;
  return EXIT_SUCCESS;
}
