/* Fully causal FC + board-IMU + monocular joint alignment supervisor. */

#ifndef OV_MSCKF_ONLINE_ALIGNMENT_INITIALIZER_H
#define OV_MSCKF_ONLINE_ALIGNMENT_INITIALIZER_H

#include "core/AlignmentFrameSelector.h"
#include "core/OnlineAlignmentCandidateFilter.h"
#include "core/VisualCadencePlanner.h"

#include <Eigen/Dense>
#include <array>
#include <deque>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace ov_msckf {

struct FCNavigationSample {
  double timestamp = -1.0;
  Eigen::Vector3d position_G = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_G = Eigen::Vector3d::Zero();
  /// Passive JPL quaternion, global navigation frame to FC body frame.
  Eigen::Vector4d q_GtoF = (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  std::string navigation_frame;
  std::string body_frame;
  bool position_valid = false;
  bool velocity_valid = false;
  bool attitude_valid = false;
  bool status_valid = false;
};

struct BoardImuSample {
  double timestamp = -1.0;
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  std::string frame;
  bool status_valid = false;
  bool gyro_saturated = false;
  bool accel_saturated = false;
};

struct StereoAlignmentObservation {
  size_t feature_id = 0;
  Eigen::Vector2d raw_left = Eigen::Vector2d::Zero();
  Eigen::Vector2d raw_right = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d normalized_left = Eigen::Vector2d::Zero();
  Eigen::Vector2d normalized_right = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double depth_m = std::numeric_limits<double>::quiet_NaN();
  double stereo_ray_residual_m = std::numeric_limits<double>::quiet_NaN();
  int track_length = 0;
  bool left_valid = false;
  bool right_valid = false;
};

struct StereoAlignmentFrame {
  double left_timestamp = -1.0;
  double right_timestamp = -1.0;
  std::vector<StereoAlignmentObservation> observations;
  bool tracking_valid = false;
  bool stereo_valid = false;
};

enum class AlignmentPhase {
  WAIT_INPUTS,
  COLLECTING,
  ALIGNING,
  VALIDATING,
  CANDIDATE_VALIDATING,
  CANDIDATE_REFINING,
  NAVIGATION_READY,
  FULL_ALIGNMENT_READY,
  FAILED_WAIT_RETRY,
  CLOSED_AFTER_RELEASE,
  FATAL_CONFIGURATION_ERROR
};

const char *alignment_phase_name(AlignmentPhase phase);

enum class AlignmentReadiness {
  NOT_READY,
  NAVIGATION_READY,
  FULL_ALIGNMENT_READY
};

const char *alignment_readiness_name(AlignmentReadiness readiness);

enum class AlignmentReleasePolicy {
  PRACTICAL_NAVIGATION_START,
  STRICT_FULL_ALIGNMENT
};

const char *alignment_release_policy_name(AlignmentReleasePolicy policy);

enum class EstimateSourceStatus {
  ESTIMATED_CURRENT_DATA,
  WEAKLY_OBSERVABLE,
  FIXED_TO_PRIOR,
  FIXED_EXTERNAL_CALIBRATION,
  UNOBSERVABLE
};

const char *estimate_source_status_name(EstimateSourceStatus status);

struct AlignmentTransition {
  AlignmentPhase from = AlignmentPhase::WAIT_INPUTS;
  AlignmentPhase to = AlignmentPhase::WAIT_INPUTS;
  double stream_time = -1.0;
  std::string reason;
};

struct RejectedAlignmentInterval {
  double start_time = -1.0;
  double end_time = -1.0;
  std::string reason;
  double peak_angular_rate_rad_s = 0.0;
};

struct FactorContribution {
  std::string family;
  int residual_blocks = 0;
  int residual_dimension = 0;
  double residual_rms = std::numeric_limits<double>::infinity();
  double residual_p95 = std::numeric_limits<double>::infinity();
  double residual_max_abs = std::numeric_limits<double>::infinity();
  double jacobian_frobenius = 0.0;
  std::map<std::string, double> state_jacobian_frobenius;
};

struct StateObservability {
  std::string state;
  Eigen::Vector3d covariance_std = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  double information_min_eigenvalue = 0.0;
  double information_max_eigenvalue = 0.0;
  double information_condition = std::numeric_limits<double>::infinity();
  /// Marginal information contributed by current IMU/visual/FC data after
  /// removing explicit priors. This is the release quantity; inverse total
  /// covariance alone can make a prior-dominated bias look observable.
  double data_information_min_eigenvalue = 0.0;
  double data_information_max_eigenvalue = 0.0;
  double data_information_condition = std::numeric_limits<double>::infinity();
  /// Physical tangent-unit scale used to form z = dx / scale. Release rank
  /// gates use H_z = scale^2 H_x, so radians, metres, m/s and bias units are
  /// never compared through one dimensional threshold.
  double information_normalization_scale = 1.0;
  double normalized_data_information_min_eigenvalue = 0.0;
  double normalized_data_information_max_eigenvalue = 0.0;
  double normalized_data_information_condition =
      std::numeric_limits<double>::infinity();
  int prior_residual_count = 0;
  int imu_residual_count = 0;
  int visual_residual_count = 0;
  int fc_residual_count = 0;
  double prior_jacobian_norm = 0.0;
  double imu_jacobian_norm = 0.0;
  double visual_jacobian_norm = 0.0;
  double fc_jacobian_norm = 0.0;
  bool observable = false;
  bool prior_only = false;
  EstimateSourceStatus estimate_status = EstimateSourceStatus::UNOBSERVABLE;
  std::string reason;
};

struct VisualAlignmentStatistics {
  int tracked_feature_count = 0;
  int multi_frame_track_count = 0;
  int triangulated_landmark_count = 0;
  int rejected_landmark_count = 0;
  int visual_factor_count = 0;
  double reprojection_rmse_px = std::numeric_limits<double>::infinity();
  double reprojection_p95_px = std::numeric_limits<double>::infinity();
  double schur_complement_information_trace = 0.0;
  std::map<std::string, double> information_contribution_by_state;
};

struct SensorProvenance {
  std::string fc_stream;
  std::string imu_stream;
  std::string stereo_stream;
  std::string camera_imu_calibration;
  std::string fc_axis_mapping;
  bool gps_used = false;
  bool post_alignment_used = false;
  bool manual_mounting_compensation_used = false;
};

struct OnlineAlignmentOptions {
  /// Formal production lifecycle: one finite joint candidate window, a later
  /// causal holdout measured in sensor time, at most one newly advanced joint
  /// refinement, then one atomic terminal-state release.
  bool formal_causal_lifecycle = false;
  /// Legacy comparison only. Initialize local VIO upstream, then estimate a
  /// low-dimensional FC yaw/translation gauge. The formal runner must keep
  /// this false whenever formal_causal_lifecycle is true.
  bool upstream_dynamic_init_fc_gauge = false;
  /// Diagnostic ablation: replace the solved release attitude with the
  /// synchronized FC attitude composed with the accepted FC-to-board mount.
  /// The graph, window, p/v/bg/ba solution, and covariance remain unchanged.
  bool diagnostic_release_attitude_from_fc = false;
  /// Root-cause ablation only. Retain the joint graph attitude while replacing
  /// p/v with the synchronized FC board-state observation and bg/ba with zero.
  /// Production must never enable this mixed-source release contract.
  bool diagnostic_release_graph_attitude_fc_pv_zero_bias = false;
  /// Anchor the released terminal attitude to the synchronized FC attitude
  /// composed with the accepted full-flight FC-to-board calibration. Earlier
  /// FC attitudes constrain only relative increments, so keyframe density does
  /// not multiply absolute gauge information.
  bool fc_attitude_gauge_factor_enabled = true;
  /// Rebuild a bounded joint problem on every advanced fixed-time window and
  /// retain each solution as shadow evidence. Candidate validation and formal
  /// OpenVINS injection are bypassed only while this is true.
  bool sliding_window_shadow_only = false;
  /// Production P4 lifecycle: keep OpenVINS uninitialized while repeated
  /// joint windows are solved, then inject one coherent q/p/v/bg/ba state and
  /// covariance after time-based overlap consistency is established.
  bool sliding_window_direct_state_release = false;
  /// Keep every advanced window as a complete joint q/p/v/bg/ba solve, but
  /// defer release-only Jacobian/Schur/covariance certification until the
  /// rolling solutions have been physically stable for the configured
  /// sensor-time interval. This changes runtime scheduling, not estimation
  /// topology or release gates.
  bool sliding_window_deferred_release_certification = false;
  /// Production OpenVINS injects only the terminal IMU state. Do not recover
  /// clone/landmark covariance blocks that the handoff will discard.
  bool sliding_window_terminal_state_covariance_only = false;
  /// Diagnostic isolation only: keep solving the same sliding-window graphs
  /// and keep the provisional VIO running, but never apply the final gauge
  /// anchor. This is not a production release policy.
  bool sliding_window_diagnostic_never_anchor = false;
  double sliding_window_duration_s = 8.0;
  double sliding_window_min_advance_s = 0.5;
  /// Formal gauge release requires this much continuous sensor-time agreement
  /// between independently advanced fixed-time window solutions. It is not an
  /// update-count quota.
  double sliding_window_gauge_stability_duration_s = 2.0;
  double sliding_window_overlap_stability_duration_s = 2.0;
  /// A shared bias is converged only when its causal cross-window linear trend,
  /// projected over one complete solve window, is no larger than this many
  /// posterior standard deviations. This complements pairwise overlap checks,
  /// which cannot reject a smooth one-directional drift.
  double sliding_window_bias_trend_max_normalized_sigma = 1.0;
  int sliding_window_min_overlap_states = 4;
  /// Statistical agreement is measured against the combined 15-state graph
  /// covariance. These absolute limits are only fail-closed physical guards.
  double sliding_window_overlap_max_normalized_sigma = 3.0;
  double sliding_window_max_overlap_attitude_deg = 5.0;
  double sliding_window_max_overlap_position_m = 5.0;
  double sliding_window_max_overlap_velocity_mps = 3.0;
  double sliding_window_max_overlap_gyro_bias_rad_s = 0.03;
  double sliding_window_max_overlap_accel_bias_mps2 = 1.0;
  /// Upstream-style reference window followed by progressively longer joint
  /// alignment windows. The shortest usable window is solved first.
  double reference_window_duration_s = 2.0;
  std::vector<double> candidate_window_durations_s = {3.0, 5.0, 8.0, 12.0};
  double window_duration_s = 5.0;
  double buffer_margin_s = 0.75;
  int min_fc_samples = 12;
  int min_imu_samples = 100;
  int min_stereo_frames = 6;
  int min_feature_tracks = 20;
  int min_stereo_depths = 12;
  int min_keyframes = 5;
  /// Computational guard only. Formal P4 uses every frame selected by the
  /// time-window selector up to this bound; it must not thin a normal 8 s
  /// window to an arbitrary fixed measurement count.
  int max_keyframes = 36;
  int min_visual_residual_blocks = 30;
  int max_visual_features = 80;
  /// Bound persistent landmarks transferred with the batch posterior. Only
  /// landmarks observed at the release keyframe are eligible.
  int max_initial_slam_features = 50;
  /// Number of terminal historical poses transferred to the EKF. This is a
  /// backend state-capacity bound, not a limit on P4 graph measurements.
  int max_initial_clones = 10;
  double min_monocular_parallax_deg = 0.5;
  double min_monocular_baseline_m = 0.20;
  double max_fc_gap_s = 0.35;
  double max_imu_gap_s = 0.03;
  double max_stereo_gap_s = 0.75;
  double max_stereo_sync_gap_s = 0.02;
  double max_time_offset_s = 0.12;
  double time_offset_step_s = 0.002;
  double max_time_offset_sigma_s = 0.05;
  double camera_to_imu_time_offset_s = 0.0;
  /// Accepted offline full-flight FC-attitude/board-gyro calibration. P4 must
  /// never replace these values with an estimate from its startup window.
  bool fc_board_calibration_locked = true;
  double fc_attitude_to_board_time_offset_s = 0.0;
  /// Position/velocity time mapping is a separate declaration. It must not be
  /// inferred from gyro correlation.
  double fc_navigation_to_board_time_offset_s = 0.0;
  double fc_attitude_to_board_time_offset_sigma_s = 0.0;
  double fc_board_mount_sigma_deg = 5.0;
  double max_angular_rate_rad_s = 3.5;
  double min_angular_excitation_rad_s = 0.08;
  double min_second_axis_ratio = 0.02;
  double max_mount_residual_deg = 15.0;
  double max_rate_residual_rms_rad_s = 0.20;
  double max_gyro_bias_norm_rad_s = 0.20;
  double max_accel_bias_norm_mps2 = 2.0;
  double max_attitude_sigma_deg = 8.0;
  double max_position_sigma_m = 10.0;
  double max_velocity_sigma_mps = 5.0;
  double max_gyro_bias_sigma_rad_s = 0.10;
  double max_accel_bias_sigma_mps2 = 1.0;
  double max_mount_sigma_deg = 8.0;
  double navigation_max_attitude_sigma_deg = 5.0;
  double navigation_max_position_sigma_m = 5.0;
  double navigation_max_velocity_sigma_mps = 2.0;
  double navigation_max_gyro_bias_sigma_rad_s = 0.10;
  double navigation_max_accel_bias_sigma_mps2 = 2.0;
  double navigation_max_reprojection_rmse_px = 2.5;
  double navigation_max_reprojection_p95_px = 4.0;
  double navigation_max_imu_residual_rms = 3.0;
  double navigation_max_fc_residual_rms = 3.0;
  /// Deprecated compatibility field. It is intentionally ignored: total
  /// collection time never closes a live sliding-window initialization.
  double navigation_max_collection_s = std::numeric_limits<double>::infinity();
  double navigation_target_latency_s = 12.0;
  double full_alignment_observation_s = 60.0;
  double weak_mount_prior_sigma_deg = 15.0;
  double weak_time_offset_prior_s = 0.0;
  double weak_time_offset_prior_sigma_s = 1.0;
  /// Allow NAVIGATION_READY when startup FC-to-board orientation/time are
  /// fixed to declared priors. Keep false unless those priors have an
  /// independently accepted runtime lineage.
  bool navigation_allow_startup_prior_release = false;
  /// Dimensionless minimum eigenvalue of H_z after each 3-D state group is
  /// scaled by its configured maximum admissible one-sigma uncertainty.
  double min_information_eigenvalue = 1e-8;
  double max_information_condition = 1e12;
  double fc_attitude_sigma_deg = 2.0;
  double fc_position_sigma_m = 2.0;
  double fc_velocity_sigma_mps = 0.75;
  /// Fraction of terminal FC PVA error covariance assigned to independent
  /// Phi=I process increments across the complete physical window.
  double fc_process_variance_fraction = 0.25;
  double visual_pixel_sigma = 1.5;
  double gyro_bias_prior_sigma_rad_s = 0.10;
  double accel_bias_prior_sigma_mps2 = 1.0;
  double mount_prior_sigma_deg = 10.0;
  double imu_sigma_w = 1.6968e-4;
  double imu_sigma_wb = 1.9393e-5;
  double imu_sigma_a = 2.0e-3;
  double imu_sigma_ab = 3.0e-3;
  int solver_max_iterations = 30;
  double solver_max_time_s = 2.0;
  double minimum_solve_interval_s = 0.75;
  int minimum_new_selected_frames_for_resolve = 3;
  int maximum_selected_alignment_frames = 36;
  double alignment_frame_minimum_interval_s = 0.10;
  double alignment_frame_maximum_interval_s = 0.60;
  double alignment_target_compensated_parallax_px = 3.0;
  /// A solved window is checked only with later causal data. This is the
  /// required evidence duration; the bounded deadline additionally allows one
  /// maximum FC/visual sampling gap. It is a sensor-time contract, not a fixed
  /// update quota or a success-by-age rule.
  double candidate_short_validation_duration_s = 2.0;
  CandidateFilterConfig candidate_filter_config;
  double candidate_max_fc_imu_rotation_residual_deg = 5.0;
  double candidate_reject_fc_imu_rotation_residual_deg = 10.0;
  double candidate_max_relative_position_residual_m = 5.0;
  double candidate_reject_relative_position_residual_m = 10.0;
  double candidate_max_relative_velocity_residual_mps = 3.0;
  double candidate_reject_relative_velocity_residual_mps = 6.0;
  double candidate_max_visual_compensated_p95_px = 20.0;
  double candidate_reject_visual_compensated_p95_px = 40.0;
  double candidate_max_visual_reprojection_p95_px = 8.0;
  double candidate_reject_visual_reprojection_p95_px = 16.0;
  double candidate_max_visual_reprojection_trend_pxps = 3.0;
  /// Total correction limits over the repeated closed-loop window. They are
  /// safety bounds, not convergence thresholds.
  double candidate_max_closed_loop_attitude_correction_deg = 3.0;
  double candidate_max_closed_loop_position_correction_m = 5.0;
  double candidate_max_closed_loop_velocity_correction_mps = 3.0;
  double candidate_max_closed_loop_gyro_bias_correction_rad_s = 0.02;
  double candidate_max_closed_loop_accel_bias_correction_mps2 = 0.50;
  /// Future-frame landmarks were optimized in the same batch and their
  /// covariance is not carried in the released 15-state. Keep them as an
  /// independent reprojection validation gate; feeding them again would double
  /// count correlated information. The joint solve remains fully visual.
  int candidate_max_closed_loop_visual_updates = 0;
  /// Permit at most one startup-wide refinement. The refinement must use a
  /// newly advanced current window and may never rerun the candidate window.
  bool candidate_refinement_enabled = true;
  bool visual_factors_enabled = true;
  bool navigation_allow_without_visual = false;
  double visual_perturbation_px = 0.0;
  double visual_perturbation_fraction = 0.0;
  AlignmentReleasePolicy release_policy =
      AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  Eigen::Vector3d gravity_G = Eigen::Vector3d(0.0, 0.0, 9.81);
  Eigen::Matrix3d R_FtoI_declared = Eigen::Matrix3d::Identity();
  /// FC-to-IMU axis map used only to start the provisional local VIO.  This is
  /// intentionally separate from the externally calibrated P4 graph mount:
  /// the provisional estimator must reproduce the frozen one-row baseline,
  /// while P4 estimates the later global yaw+translation gauge independently.
  Eigen::Matrix3d R_FtoI_provisional_seed = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinF = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector4d, 2> q_ItoC = {
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished(),
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished()};
  std::array<Eigen::Vector3d, 2> p_IinC = {
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  std::array<Eigen::Matrix<double, 8, 1>, 2> camera_intrinsics = {
      Eigen::Matrix<double, 8, 1>::Zero(),
      Eigen::Matrix<double, 8, 1>::Zero()};
  std::array<bool, 2> camera_fisheye = {false, false};
  std::string expected_navigation_frame = "G_nav";
  std::string expected_fc_body_frame = "FC_body";
  std::string expected_board_imu_frame = "board_imu";
  SensorProvenance provenance;
};

struct OnlineAlignmentDiagnostics {
  double solve_time = -1.0;
  double collection_start_time = -1.0;
  double window_start = -1.0;
  double init_time = -1.0;
  double decision_time = -1.0;
  double navigation_ready_time = -1.0;
  double full_alignment_ready_time = -1.0;
  double first_openvins_output_time = -1.0;
  double initialization_duration_s = 0.0;
  int fc_samples = 0;
  int imu_samples = 0;
  int stereo_frames = 0;
  int keyframes = 0;
  int feature_tracks = 0;
  int stereo_depths = 0;
  double max_fc_gap_s = std::numeric_limits<double>::infinity();
  double max_imu_gap_s = std::numeric_limits<double>::infinity();
  double max_stereo_gap_s = std::numeric_limits<double>::infinity();
  double angular_excitation_rad_s = 0.0;
  double second_axis_ratio = 0.0;
  double rate_residual_rms_rad_s = std::numeric_limits<double>::infinity();
  double visual_imu_rotation_residual_deg = std::numeric_limits<double>::infinity();
  double estimated_fc_to_board_time_offset_s = 0.0;
  double time_offset_sigma_s = std::numeric_limits<double>::infinity();
  double mount_residual_deg = std::numeric_limits<double>::infinity();
  double initial_cost = std::numeric_limits<double>::infinity();
  double final_cost = std::numeric_limits<double>::infinity();
  int solver_iterations = 0;
  bool shared_window_bias_model = false;
  bool staged_solver_enabled = false;
  bool stage1_solution_usable = false;
  double stage1_initial_cost = std::numeric_limits<double>::infinity();
  double stage1_final_cost = std::numeric_limits<double>::infinity();
  int stage1_solver_iterations = 0;
  double stage1_solve_wall_time_s = 0.0;
  double final_stage_solve_wall_time_s = 0.0;
  int retry_count = 0;
  /// Number of actual entries into the nonlinear Ceres joint solve.
  int nonlinear_solve_attempt_count = 0;
  /// Atomic OpenVINS releases. This is constrained to zero or one.
  int successful_release_count = 0;
  /// Defensive count of direct API calls made after a successful release.
  int post_release_try_count = 0;
  int solve_eligibility_skip_count = 0;
  int duplicate_window_skip_count = 0;
  int candidate_created_count = 0;
  int candidate_rejected_count = 0;
  int candidate_refinement_count = 0;
  int gauge_window_observation_count = 0;
  double gauge_stability_required_s = 0.0;
  double gauge_stable_duration_s = 0.0;
  double gauge_yaw_delta_from_previous_deg =
      std::numeric_limits<double>::infinity();
  double gauge_translation_delta_from_previous_m =
      std::numeric_limits<double>::infinity();
  double gauge_yaw_consistency_limit_deg =
      std::numeric_limits<double>::infinity();
  double gauge_translation_consistency_limit_m =
      std::numeric_limits<double>::infinity();
  double gauge_metric_scale = std::numeric_limits<double>::quiet_NaN();
  double gauge_metric_scale_observed =
      std::numeric_limits<double>::quiet_NaN();
  double gauge_metric_scale_sigma =
      std::numeric_limits<double>::infinity();
  double gauge_velocity_fit_rmse_mps =
      std::numeric_limits<double>::infinity();
  double gauge_velocity_excitation_mps = 0.0;
  double gauge_yaw_reset_deg = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d gauge_translation_G = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  bool gauge_window_quality_passed = false;
  bool gauge_consistency_passed = false;
  int sliding_overlap_state_count = 0;
  double sliding_overlap_attitude_max_deg =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_position_max_m =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_velocity_max_mps =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_gyro_bias_max_rad_s =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_accel_bias_max_mps2 =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_normalized_max_sigma =
      std::numeric_limits<double>::infinity();
  double sliding_overlap_stable_duration_s = 0.0;
  double sliding_overlap_stability_required_s = 0.0;
  bool sliding_overlap_consistency_passed = false;
  int sliding_bias_trend_sample_count = 0;
  double sliding_bias_trend_span_s = 0.0;
  Eigen::Vector3d sliding_bg_trend_projected_change_rad_s =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d sliding_ba_trend_projected_change_mps2 =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  double sliding_bias_trend_normalized_max_sigma =
      std::numeric_limits<double>::infinity();
  bool sliding_bias_trend_passed = false;
  bool direct_sliding_state_release = false;
  int initial_history_clone_count = 0;
  int initial_persistent_landmark_count = 0;
  int initial_joint_covariance_dimension = 0;
  bool initial_history_covariance_recovered = false;
  bool handoff_covariance_inflation_applied = false;
  std::string handoff_covariance_model = "none";
  std::array<double, 4> handoff_covariance_inflation = {1.0, 1.0, 1.0,
                                                        1.0};
  Eigen::Matrix<double, 15, 1> handoff_raw_covariance_std =
      Eigen::Matrix<double, 15, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 15, 1> handoff_applied_covariance_std =
      Eigen::Matrix<double, 15, 1>::Constant(
          std::numeric_limits<double>::quiet_NaN());
  int candidate_validation_frames = 0;
  double selected_window_duration_s = 0.0;
  double maximum_buffer_window_s = 0.0;
  double candidate_validation_duration_s = 0.0;
  double candidate_fc_imu_rotation_residual_deg =
      std::numeric_limits<double>::infinity();
  double candidate_fc_imu_yaw_residual_deg =
      std::numeric_limits<double>::infinity();
  double candidate_relative_position_residual_m =
      std::numeric_limits<double>::infinity();
  double candidate_relative_velocity_residual_mps =
      std::numeric_limits<double>::infinity();
  double candidate_nis_gate_3d = std::numeric_limits<double>::quiet_NaN();
  double candidate_last_position_nis = std::numeric_limits<double>::quiet_NaN();
  double candidate_last_velocity_nis = std::numeric_limits<double>::quiet_NaN();
  double candidate_max_position_nis = std::numeric_limits<double>::quiet_NaN();
  double candidate_max_velocity_nis = std::numeric_limits<double>::quiet_NaN();
  double candidate_visual_compensated_p95_px =
      std::numeric_limits<double>::infinity();
  double candidate_visual_reprojection_p50_px =
      std::numeric_limits<double>::infinity();
  double candidate_visual_reprojection_p95_px =
      std::numeric_limits<double>::infinity();
  double candidate_visual_reprojection_trend_pxps =
      std::numeric_limits<double>::infinity();
  double candidate_covariance_normalized_residual =
      std::numeric_limits<double>::infinity();
  double candidate_release_propagation_duration_s = 0.0;
  double candidate_pre_correction_position_residual_m =
      std::numeric_limits<double>::infinity();
  double candidate_pre_correction_velocity_residual_mps =
      std::numeric_limits<double>::infinity();
  double candidate_pre_correction_visual_reprojection_p95_px =
      std::numeric_limits<double>::infinity();
  /// Residuals evaluated against the same FC sample and board time after the
  /// release feedback/reset has been applied. These are comparable with the
  /// three pre-correction residuals above.
  double candidate_post_correction_attitude_residual_deg =
      std::numeric_limits<double>::infinity();
  double candidate_post_correction_position_residual_m =
      std::numeric_limits<double>::infinity();
  double candidate_post_correction_velocity_residual_mps =
      std::numeric_limits<double>::infinity();
  double candidate_closed_loop_attitude_correction_deg = 0.0;
  double candidate_closed_loop_position_correction_m = 0.0;
  double candidate_closed_loop_velocity_correction_mps = 0.0;
  double candidate_closed_loop_gyro_bias_correction_rad_s = 0.0;
  double candidate_closed_loop_accel_bias_correction_mps2 = 0.0;
  int candidate_closed_loop_update_count = 0;
  int candidate_closed_loop_measurement_update_count = 0;
  int candidate_closed_loop_rejected_measurement_count = 0;
  int candidate_closed_loop_visual_update_count = 0;
  bool candidate_closed_loop_correction_applied = false;
  bool candidate_closed_loop_refinement_applied = false;
  bool candidate_covariance_joseph_update_applied = false;
  bool candidate_covariance_error_reset_applied = false;
  bool candidate_attitude_feedback_allowed = false;
  bool candidate_gyro_bias_feedback_allowed = false;
  bool candidate_accel_bias_feedback_allowed = false;
  double candidate_attitude_stable_duration_s = 0.0;
  double candidate_gyro_bias_stable_duration_s = 0.0;
  double candidate_accel_bias_stable_duration_s = 0.0;
  std::string candidate_feedback_tier = "PV_ONLY";
  std::string candidate_feedback_state_mask = "p,v";
  /// False only when a robust FC attitude gauge factor participates in the
  /// joint solve and the held-out FC attitude residual is a release gate.
  bool candidate_fc_attitude_evaluation_only = true;
  bool fc_attitude_gauge_factor_enabled = false;
  int fc_attitude_gauge_factor_count = 0;
  /// FC navigation is a temporally correlated filter output. The released
  /// terminal p/v receives one full absolute factor; earlier rows constrain
  /// density-invariant increments only.
  int fc_position_velocity_factor_count = 0;
  double fc_position_velocity_time_weight_sum = 0.0;
  std::string fc_position_velocity_weight_model = "none";
  std::string fc_error_state_transition_model = "none";
  double fc_process_variance_fraction = 0.0;
  double fc_terminal_increment_max_correlation = 0.0;
  std::string visual_pairing_policy = "none";
  int visual_unique_measurement_count = 0;
  int formal_candidate_holdout_fc_samples = 0;
  int formal_candidate_holdout_imu_samples = 0;
  int formal_candidate_holdout_visual_frames = 0;
  double formal_candidate_holdout_duration_s = 0.0;
  bool formal_candidate_holdout_passed = false;
  bool formal_refinement_release = false;
  double fc_terminal_attitude_residual_deg =
      std::numeric_limits<double>::infinity();
  double fc_terminal_position_residual_m =
      std::numeric_limits<double>::infinity();
  double fc_terminal_velocity_residual_mps =
      std::numeric_limits<double>::infinity();
  double fc_terminal_max_normalized_residual =
      std::numeric_limits<double>::infinity();
  double fc_attitude_gauge_anchor_timestamp_s =
      std::numeric_limits<double>::quiet_NaN();
  double fc_attitude_gauge_anchor_rate_rad_s =
      std::numeric_limits<double>::quiet_NaN();
  double fc_attitude_gauge_anchor_rate_residual_rad_s =
      std::numeric_limits<double>::quiet_NaN();
  double fc_attitude_gauge_sigma_deg =
      std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 15, 1> candidate_persistent_error =
      Eigen::Matrix<double, 15, 1>::Zero();
  Eigen::Matrix<double, 15, 1> candidate_persistent_std =
      Eigen::Matrix<double, 15, 1>::Constant(
          std::numeric_limits<double>::infinity());
  int candidate_consumed_fc_event_count = 0;
  int candidate_accepted_fc_event_count = 0;
  int candidate_rejected_fc_event_count = 0;
  int candidate_propagated_imu_step_count = 0;
  int candidate_duplicate_fc_event_count = 0;
  int candidate_duplicate_imu_step_count = 0;
  /// Per-group convergence evidence in q,p,v,bg,ba order.
  std::array<int, 5> candidate_group_supported_update_counts{};
  /// The support/history minimum required by the active candidate gate.
  std::array<int, 5> candidate_required_group_support_update_counts{};
  std::array<int, 5> candidate_required_group_history_lengths{};
  /// Effective valid FC measurement rows in the selected solve window. The
  /// production gate depth is derived from these counts, not from a fixed
  /// event quota or a keyframe cap.
  std::array<int, 5> candidate_window_effective_measurement_counts{};
  std::string candidate_gate_depth_source =
      "explicit_gate_configuration";
  std::array<int, 5> candidate_group_feedback_counts{};
  std::array<int, 5> candidate_group_post_feedback_stable_update_counts{};
  std::array<bool, 5> candidate_group_gate_passed{};
  std::string candidate_covariance_health = "healthy";
  std::string window_fingerprint;
  std::vector<double> assessed_window_durations_s;
  bool alignment_window_closed = false;
  double alignment_window_close_time = -1.0;
  bool solver_converged = false;
  bool future_data_used = false;
  bool post_alignment_used = false;
  bool quality_passed = false;
  bool navigation_quality_passed = false;
  bool full_alignment_quality_passed = false;
  bool old_initializer_fallback_used = false;
  bool startup_mount_fixed_to_prior = false;
  bool startup_time_offset_fixed_to_prior = false;
  AlignmentReadiness readiness = AlignmentReadiness::NOT_READY;
  AlignmentReleasePolicy release_policy =
      AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  EstimateSourceStatus time_offset_status =
      EstimateSourceStatus::UNOBSERVABLE;
  EstimateSourceStatus startup_misalignment_status =
      EstimateSourceStatus::UNOBSERVABLE;
  std::string readiness_reason;
  std::vector<std::string> failed_gates;
  std::vector<std::string> navigation_failed_gates;
  std::vector<std::string> full_alignment_failed_gates;
  std::vector<std::string> estimated_state_list;
  std::vector<std::string> weak_state_list;
  std::vector<std::string> fixed_state_list;
  std::vector<std::string> unobservable_state_list;
  std::vector<FactorContribution> factor_contributions;
  std::vector<StateObservability> state_observability;
  std::vector<RejectedAlignmentInterval> rejected_intervals;
  std::vector<AlignmentTransition> state_transitions;
  VisualAlignmentStatistics visual_statistics;
  SensorProvenance provenance;
};

struct AlignmentCloneState {
  double timestamp = -1.0;
  Eigen::Vector4d q_GtoI =
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
};

struct AlignmentLandmarkState {
  size_t feature_id = 0;
  Eigen::Vector3d p_FinG = Eigen::Vector3d::Zero();
};

struct AlignmentResult {
  double timestamp = -1.0;
  Eigen::Vector4d q_GtoI = (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Identity();
  /// Joint tangent covariance in the exact order
  /// [active IMU(q,p,v,bg,ba), historical clone_0(q,p), ...,
  /// persistent landmark_0(xyz), ...].
  /// Historical clones are strictly increasing and exclude the active-state
  /// timestamp. An empty matrix preserves the legacy terminal-only contract.
  std::vector<AlignmentCloneState> initial_clones;
  std::vector<AlignmentLandmarkState> initial_landmarks;
  /// Exact tracker IDs whose startup observations entered P4 visual factors.
  /// Their pre-release tracks must be removed after posterior transfer to
  /// prevent reuse; all other KLT histories remain available to MSCKF.
  std::vector<size_t> startup_consumed_feature_ids;
  Eigen::MatrixXd initial_joint_covariance;
  Eigen::Matrix3d R_FtoI_nominal = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_mount_residual = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d mount_covariance = Eigen::Matrix3d::Identity();
  double fc_to_board_time_offset_s = 0.0;
  double fc_attitude_to_board_time_offset_s = 0.0;
  double fc_navigation_to_board_time_offset_s = 0.0;
  double camera_to_imu_time_offset_s = 0.0;
  AlignmentReadiness readiness = AlignmentReadiness::NOT_READY;
  AlignmentReleasePolicy release_policy =
      AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
  bool released_to_openvins = false;
  OnlineAlignmentDiagnostics diagnostics;
};

struct AlignmentCandidate {
  AlignmentResult result;
  double solve_window_start = -1.0;
  double solve_window_end = -1.0;
  std::vector<double> selected_frame_timestamps;
  std::vector<size_t> selected_feature_ids;
  /// Optimized nuisance landmarks retained only for the bounded pre-release
  /// monocular reprojection check. They are discarded when P4 closes.
  std::map<size_t, Eigen::Vector3d> optimized_landmarks_G;
  std::vector<FactorContribution> factor_contributions;
  std::vector<StateObservability> state_observability;
  std::vector<std::string> prior_dominated_states;
  double solve_wall_time_s = 0.0;
  SensorProvenance provenance;
};

struct SlidingWindowStateEstimate {
  double timestamp = -1.0;
  Eigen::Vector4d q_GtoI =
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

struct AlignmentAttemptReceipt {
  int window_id = 0;
  int optimizer_invocation_index = 0;
  double attempt_timestamp = -1.0;
  std::string trigger;
  std::string window_fingerprint;
  double window_duration_s = 0.0;
  double window_begin_timestamp = -1.0;
  double window_end_timestamp = -1.0;
  int raw_fc_count = 0;
  int valid_fc_count = 0;
  int imu_count = 0;
  int visual_frame_count = 0;
  int selected_keyframe_count = 0;
  std::vector<double> selected_frame_timestamps;
  int selected_tracks = 0;
  int selected_landmarks = 0;
  int selected_visual_pairs = 0;
  int factor_count = 0;
  double solve_wall_time_s = 0.0;
  int optimizer_invocation_count = 0;
  bool shared_window_bias_model = false;
  bool staged_solver_enabled = false;
  bool stage1_solution_usable = false;
  double stage1_initial_cost = std::numeric_limits<double>::infinity();
  double stage1_final_cost = std::numeric_limits<double>::infinity();
  int stage1_solver_iterations = 0;
  double stage1_solve_wall_time_s = 0.0;
  double final_stage_solve_wall_time_s = 0.0;
  double initial_cost = std::numeric_limits<double>::infinity();
  double final_cost = std::numeric_limits<double>::infinity();
  Eigen::Vector4d q_GtoI =
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_prior_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba_prior_center = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 1> state_std =
      Eigen::Matrix<double, 15, 1>::Constant(
          std::numeric_limits<double>::infinity());
  double imu_residual_rms = std::numeric_limits<double>::infinity();
  double fc_residual_rms = std::numeric_limits<double>::infinity();
  double visual_reprojection_rmse_px =
      std::numeric_limits<double>::infinity();
  double visual_reprojection_p95_px =
      std::numeric_limits<double>::infinity();
  double joint_normalized_cost = std::numeric_limits<double>::infinity();
  double angular_excitation_rad_s = 0.0;
  double second_axis_ratio = 0.0;
  double previous_attitude_delta_deg =
      std::numeric_limits<double>::quiet_NaN();
  double previous_position_delta_m =
      std::numeric_limits<double>::quiet_NaN();
  double previous_velocity_delta_mps =
      std::numeric_limits<double>::quiet_NaN();
  double previous_gyro_bias_delta_rad_s =
      std::numeric_limits<double>::quiet_NaN();
  double previous_accel_bias_delta_mps2 =
      std::numeric_limits<double>::quiet_NaN();
  int overlap_state_count = 0;
  double overlap_attitude_max_deg =
      std::numeric_limits<double>::infinity();
  double overlap_position_max_m =
      std::numeric_limits<double>::infinity();
  double overlap_velocity_max_mps =
      std::numeric_limits<double>::infinity();
  double overlap_gyro_bias_max_rad_s =
      std::numeric_limits<double>::infinity();
  double overlap_accel_bias_max_mps2 =
      std::numeric_limits<double>::infinity();
  double overlap_normalized_max_sigma =
      std::numeric_limits<double>::infinity();
  double overlap_stable_duration_s = 0.0;
  bool overlap_consistency_passed = false;
  int bias_trend_sample_count = 0;
  double bias_trend_span_s = 0.0;
  Eigen::Vector3d bg_trend_projected_change_rad_s =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d ba_trend_projected_change_mps2 =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  double bias_trend_normalized_max_sigma =
      std::numeric_limits<double>::infinity();
  bool bias_trend_passed = false;
  bool warm_start_used = false;
  std::string outcome;
  std::string failed_gate;
  std::string next_eligible_condition;
};

using OnlineAlignmentResult = AlignmentResult;

struct ProvisionalNavigationOutput {
  double timestamp = -1.0;
  Eigen::Vector4d q_GtoI =
      (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 15, 15> covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 1.0e4;
  bool valid = false;
  bool provisional = true;
  bool official_openvins_state = false;
  std::string navigation_frame = "G_nav";
  std::string source;
  std::string joint_initialization_pending_reason;
};

class OnlineAlignmentInitializer {
public:
  explicit OnlineAlignmentInitializer(const OnlineAlignmentOptions &options =
                                          OnlineAlignmentOptions());

  bool feed_fc_navigation(const FCNavigationSample &sample);
  bool feed_board_imu(const BoardImuSample &sample);
  bool feed_stereo(const StereoAlignmentFrame &frame);
  bool make_fc_gauge_target(double camera_timestamp, AlignmentResult &result);
  bool try_initialize(double now, AlignmentResult &result);
  bool provisional_navigation(double now, ProvisionalNavigationOutput &output) const;
  bool commit_shadow_gauge_release(const AlignmentResult &result);
  void reset();

  AlignmentPhase phase() const { return phase_; }
  const OnlineAlignmentDiagnostics &last_diagnostics() const { return last_diagnostics_; }
  const OnlineAlignmentDiagnostics &latest_evidence_diagnostics() const {
    return latest_evidence_diagnostics_valid_ ? latest_evidence_diagnostics_
                                              : last_diagnostics_;
  }
  const std::string &last_rejection() const { return last_rejection_; }
  size_t fc_buffer_size() const { return fc_buffer_.size(); }
  size_t imu_buffer_size() const { return imu_buffer_.size(); }
  size_t stereo_buffer_size() const { return stereo_buffer_.size(); }
  bool navigation_released() const { return navigation_released_; }
  bool full_alignment_recorded() const { return full_alignment_recorded_; }
  bool alignment_window_closed() const { return alignment_window_closed_; }
  bool candidate_active() const { return candidate_active_; }
  bool sliding_window_shadow_only() const {
    return options_.sliding_window_shadow_only;
  }
  bool upstream_dynamic_init_fc_gauge() const {
    return options_.upstream_dynamic_init_fc_gauge;
  }
  size_t sliding_window_solve_count() const { return attempt_receipts_.size(); }
  bool fatal_configuration_error() const { return fatal_configuration_error_; }
  const AlignmentCandidate &current_candidate() const { return candidate_record_; }
  const std::deque<AlignmentAttemptReceipt> &attempt_receipts() const {
    return attempt_receipts_;
  }

private:
  OnlineAlignmentOptions options_;
  std::deque<FCNavigationSample> fc_buffer_;
  std::deque<BoardImuSample> imu_buffer_;
  std::deque<StereoAlignmentFrame> stereo_buffer_;
  OnlineAlignmentDiagnostics last_diagnostics_;
  std::string last_rejection_;
  AlignmentPhase phase_ = AlignmentPhase::WAIT_INPUTS;
  std::vector<AlignmentTransition> transitions_;
  std::vector<RejectedAlignmentInterval> rejected_intervals_;
  int retry_count_ = 0;
  int nonlinear_solve_attempt_count_ = 0;
  int successful_release_count_ = 0;
  int post_release_try_count_ = 0;
  bool navigation_released_ = false;
  bool full_alignment_recorded_ = false;
  bool alignment_window_closed_ = false;
  double alignment_window_close_time_ = -1.0;
  double collection_start_time_ = -1.0;
  double navigation_ready_time_ = -1.0;
  double full_alignment_ready_time_ = -1.0;
  double maximum_window_duration_s_ = 12.0;
  double last_solve_stream_time_ = -1.0;
  int selected_frames_at_last_solve_ = 0;
  int selected_frame_serial_ = 0;
  int selected_frame_serial_at_last_solve_ = 0;
  int solve_eligibility_skip_count_ = 0;
  int duplicate_window_skip_count_ = 0;
  std::string last_window_fingerprint_;
  AlignmentFrameSelector frame_selector_;
  VisualFrameSnapshot previous_tracking_snapshot_;
  bool previous_tracking_snapshot_valid_ = false;

  bool candidate_active_ = false;
  bool formal_refinement_pending_ = false;
  double formal_candidate_camera_time_ = -1.0;
  int formal_candidate_holdout_fc_samples_ = 0;
  int formal_candidate_holdout_imu_samples_ = 0;
  int formal_candidate_holdout_visual_frames_ = 0;
  double formal_candidate_holdout_duration_s_ = 0.0;
  OnlineAlignmentCandidateFilter candidate_filter_;
  AlignmentResult candidate_result_;
  double candidate_created_time_ = -1.0;
  VisualFrameSnapshot candidate_reference_snapshot_;
  /// Every post-candidate tracking snapshot is retained until its timestamp
  /// is covered by the persistent candidate filter. Validation must not
  /// depend on the sparse nonlinear-solve frame selector.
  std::deque<VisualFrameSnapshot> candidate_visual_snapshots_;
  int candidate_validation_frames_ = 0;
  double candidate_visual_p95_max_ = 0.0;
  double candidate_last_validation_time_ = -1.0;
  std::vector<double> candidate_validation_times_;
  std::vector<double> candidate_rotation_residuals_deg_;
  std::vector<double> candidate_position_residuals_m_;
  std::vector<double> candidate_velocity_residuals_mps_;
  std::vector<double> candidate_visual_reprojection_residuals_px_;
  std::vector<double> candidate_visual_frame_p95_px_;
  double last_candidate_rotation_residual_deg_ =
      std::numeric_limits<double>::infinity();
  double last_candidate_position_residual_m_ =
      std::numeric_limits<double>::infinity();
  double last_candidate_velocity_residual_mps_ =
      std::numeric_limits<double>::infinity();
  double last_candidate_visual_p95_px_ =
      std::numeric_limits<double>::infinity();
  double candidate_latest_visual_compensated_p95_px_ =
      std::numeric_limits<double>::infinity();
  double candidate_latest_visual_reprojection_p95_px_ =
      std::numeric_limits<double>::infinity();
  double candidate_latest_visual_reprojection_p50_px_ =
      std::numeric_limits<double>::infinity();
  int candidate_created_count_ = 0;
  int candidate_rejected_count_ = 0;
  int candidate_refinement_count_ = 0;
  bool force_refinement_solve_ = false;
  double refinement_pre_position_residual_m_ =
      std::numeric_limits<double>::infinity();
  double refinement_pre_velocity_residual_mps_ =
      std::numeric_limits<double>::infinity();
  double refinement_pre_visual_reprojection_p95_px_ =
      std::numeric_limits<double>::infinity();
  bool fatal_configuration_error_ = false;
  AlignmentCandidate candidate_record_;
  std::deque<AlignmentAttemptReceipt> attempt_receipts_;
  std::array<int, 5> candidate_gate_depth_counts_{};
  std::string candidate_gate_depth_source_ =
      "explicit_gate_configuration";
  std::vector<SlidingWindowStateEstimate> previous_window_states_;
  Eigen::Matrix<double, 15, 15> previous_window_covariance_ =
      Eigen::Matrix<double, 15, 15>::Zero();
  bool previous_window_covariance_valid_ = false;
  AlignmentResult latest_shadow_window_result_;
  bool latest_shadow_window_result_valid_ = false;
  OnlineAlignmentDiagnostics latest_evidence_diagnostics_;
  bool latest_evidence_diagnostics_valid_ = false;
  double last_sliding_window_end_time_ = -1.0;
  int sliding_window_id_ = 0;
  double sliding_overlap_stable_since_ = -1.0;
  double sliding_overlap_last_window_end_ = -1.0;
  double sliding_prevalidation_stable_since_ = -1.0;
  double sliding_prevalidation_last_window_end_ = -1.0;

  void prune(double newest_timestamp);
  void transition(AlignmentPhase next, double stream_time,
                  const std::string &reason);
  void fail_retry(double stream_time, const std::string &reason);
  void copy_runtime_counters(OnlineAlignmentDiagnostics &diagnostics) const;
  bool validate_candidate(double now, AlignmentResult &result);
  bool validate_formal_candidate(double now, AlignmentResult &result);
  bool advance_candidate_filter(OnlineAlignmentCandidateFilter &filter,
                                double target_board_time,
                                std::string &reason) const;
  void reject_candidate(double now, const std::string &reason,
                        bool request_refinement);
  void mark_fatal(double stream_time, const std::string &reason);
  bool interpolate_previous_window_state(
      double timestamp, SlidingWindowStateEstimate &state) const;
  VisualFrameSnapshot make_visual_snapshot(const StereoAlignmentFrame &frame) const;
  Eigen::Matrix3d relative_camera_rotation(double previous_camera_time,
                                           double current_camera_time) const;
  bool validate_fc(const FCNavigationSample &sample, std::string &reason) const;
  bool validate_imu(const BoardImuSample &sample, std::string &reason) const;
  bool validate_stereo(const StereoAlignmentFrame &frame, std::string &reason) const;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ONLINE_ALIGNMENT_INITIALIZER_H
