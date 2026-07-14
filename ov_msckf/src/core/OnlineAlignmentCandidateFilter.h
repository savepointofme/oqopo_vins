/* Persistent error-state filter used by online alignment candidates. */

#ifndef OV_MSCKF_ONLINE_ALIGNMENT_CANDIDATE_FILTER_H
#define OV_MSCKF_ONLINE_ALIGNMENT_CANDIDATE_FILTER_H

#include <Eigen/Dense>

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <string>

namespace ov_msckf {

using CandidateErrorState = Eigen::Matrix<double, 15, 1>;
using CandidateCovariance = Eigen::Matrix<double, 15, 15>;

enum class CandidateStateGroup : std::size_t {
  ATTITUDE = 0,
  POSITION,
  VELOCITY,
  GYRO_BIAS,
  ACCEL_BIAS,
  COUNT
};

enum class CandidateMeasurementStatus {
  NOT_PROVIDED,
  ACCEPTED,
  NIS_REJECTED,
  INVALID,
  NUMERICAL_FAILURE
};

enum class CandidateCovarianceHealth {
  HEALTHY,
  ROUNDOFF_REPAIRED,
  NON_FINITE,
  MATERIAL_NEGATIVE_EIGENVALUE,
  EIGENSOLVER_FAILURE
};

enum class CandidateFeedbackTier {
  PV_ONLY,
  ATTITUDE,
  ATTITUDE_GYRO_BIAS,
  ATTITUDE_ACCEL_BIAS,
  FULL
};

struct CandidateNominalState {
  /// Passive global-to-IMU rotation. The local attitude error is defined by
  /// R_true = Exp(-dtheta) * R_nominal.
  Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
};

/// One already time-ordered board-IMU interval. Buffering, interpolation, and
/// cursor ownership remain with OnlineAlignmentInitializer.
struct CandidateImuStep {
  double start_time = -1.0;
  double end_time = -1.0;
  Eigen::Vector3d angular_velocity_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_end = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration_end = Eigen::Vector3d::Zero();
};

struct CandidateFilterNoise {
  double sigma_w = 1.6968e-4;
  double sigma_wb = 1.9393e-5;
  double sigma_a = 2.0e-3;
  double sigma_ab = 3.0e-3;
};

/// A gate is deliberately disabled until its unit-bearing thresholds have
/// been supplied by the caller. Infinite individual limits are neutral, not
/// inferred convergence thresholds.
struct CandidateGroupGate {
  bool configured = false;
  /// Production gates set this flag and leave the two count fields unset.
  /// OnlineAlignmentInitializer resolves them from the selected sliding
  /// window's valid FC measurements before constructing the filter. Tests may
  /// leave it false and provide explicit counts.
  bool derive_depth_from_sliding_window = false;
  /// A group may receive its first correction from the first accepted
  /// post-candidate FC update when its current data-gated error and covariance
  /// pass. It must still pass the full derived history before release. This
  /// separates causal feedback from the amount of subsequent evidence needed
  /// to certify that feedback.
  bool allow_initial_feedback_before_derived_history = false;
  bool require_initial_graph_support = true;
  /// A positive duration uses a causal time window for convergence history.
  /// The number of retained/required measurements is then determined by the
  /// actual valid FC update stream, not by a fixed event quota.
  double history_duration_s = 0.0;
  std::size_t history_length_updates = 0;
  int min_supported_updates = 0;
  int required_post_feedback_stable_updates = 0;
  Eigen::Vector3d max_abs_error = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_std = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_std_step = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_error_peak_to_peak = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_std_peak_to_peak = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_feedback_step = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d max_cumulative_feedback = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
};

struct CandidateFilterConfig {
  Eigen::Vector3d gravity_G = Eigen::Vector3d(0.0, 0.0, 9.81);
  CandidateFilterNoise noise;
  double position_sigma_m = 2.0;
  double velocity_sigma_mps = 0.75;
  /// 99% chi-square threshold for a three-dimensional innovation.
  double nis_gate_3d = 11.345;
  /// Only eigenvalues inside this scale-relative numerical band may be
  /// repaired. A more negative eigenvalue is a hard covariance failure.
  double covariance_roundoff_relative_tolerance = 1.0e-10;
  double time_tolerance_s = 1.0e-9;
  std::array<CandidateGroupGate,
             static_cast<std::size_t>(CandidateStateGroup::COUNT)>
      group_gates;
};

struct CandidateConvergenceSample {
  double board_time = -1.0;
  Eigen::Vector3d error = Eigen::Vector3d::Zero();
  Eigen::Vector3d std = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
};

struct CandidateGroupRuntime {
  bool initial_graph_data_supported = false;
  int supported_update_count = 0;
  int consecutive_stable_updates = 0;
  int post_first_feedback_stable_updates = 0;
  int feedback_count = 0;
  bool converged = false;
  bool last_gate_passed = false;
  bool last_feedback_clipped = false;
  int clipped_feedback_count = 0;
  Eigen::Vector3d cumulative_feedback = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_feedback = Eigen::Vector3d::Zero();
  std::deque<CandidateConvergenceSample> history;
};

struct CandidateFilterRuntime {
  bool active = false;
  double board_time = -1.0;
  CandidateNominalState nominal;
  CandidateNominalState initial_nominal;
  CandidateErrorState dx = CandidateErrorState::Zero();
  CandidateCovariance P = CandidateCovariance::Identity();
  CandidateCovariance initial_P = CandidateCovariance::Identity();
  std::array<CandidateGroupRuntime,
             static_cast<std::size_t>(CandidateStateGroup::COUNT)>
      groups;

  /// Cursor snapshots are verification metadata. The owner still selects and
  /// supplies each FC/IMU event.
  double last_imu_step_end_time = -1.0;
  double last_fc_source_timestamp = -1.0;
  double last_fc_board_time = -1.0;
  int duplicate_imu_step_count = 0;
  int duplicate_fc_event_count = 0;
  int propagated_imu_step_count = 0;
  int consumed_fc_event_count = 0;
  int accepted_fc_event_count = 0;
  int rejected_fc_event_count = 0;
  int accepted_position_count = 0;
  int rejected_position_count = 0;
  int accepted_velocity_count = 0;
  int rejected_velocity_count = 0;
  double last_position_nis = std::numeric_limits<double>::quiet_NaN();
  double last_velocity_nis = std::numeric_limits<double>::quiet_NaN();
  double max_position_nis = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_nis = std::numeric_limits<double>::quiet_NaN();
  CandidateCovarianceHealth covariance_health =
      CandidateCovarianceHealth::HEALTHY;
  int covariance_roundoff_repair_count = 0;
  bool safety_violation = false;
};

struct CandidatePositionVelocityMeasurement {
  /// Monotonic raw FC timestamp used only for duplicate protection.
  double source_timestamp = -1.0;
  double board_time = -1.0;
  Eigen::Vector3d position_G = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_G = Eigen::Vector3d::Zero();
  bool position_valid = true;
  bool velocity_valid = true;
  /// Negative values select CandidateFilterConfig defaults.
  double position_sigma_m = -1.0;
  double velocity_sigma_mps = -1.0;
};

struct CandidateFeedbackResult {
  CandidateStateGroup group = CandidateStateGroup::POSITION;
  bool applied = false;
  bool clipped = false;
  bool safety_violation = false;
  Eigen::Vector3d requested = Eigen::Vector3d::Zero();
  Eigen::Vector3d applied_correction = Eigen::Vector3d::Zero();
};

struct CandidatePositionVelocityUpdateResult {
  bool committed = false;
  bool duplicate = false;
  CandidateMeasurementStatus position_status =
      CandidateMeasurementStatus::NOT_PROVIDED;
  CandidateMeasurementStatus velocity_status =
      CandidateMeasurementStatus::NOT_PROVIDED;
  double position_nis = std::numeric_limits<double>::quiet_NaN();
  double velocity_nis = std::numeric_limits<double>::quiet_NaN();
  std::array<bool,
             static_cast<std::size_t>(CandidateStateGroup::COUNT)>
      group_supported{};
  std::array<CandidateFeedbackResult,
             static_cast<std::size_t>(CandidateStateGroup::COUNT)>
      feedback;
  std::string reason;
};

class OnlineAlignmentCandidateFilter {
public:
  explicit OnlineAlignmentCandidateFilter(
      const CandidateFilterConfig &config = CandidateFilterConfig());

  bool initialize(
      double board_time, const CandidateNominalState &nominal,
      const CandidateCovariance &covariance,
      const std::array<bool,
                       static_cast<std::size_t>(CandidateStateGroup::COUNT)>
          &initial_graph_data_support,
      const CandidateErrorState &initial_error = CandidateErrorState::Zero(),
      std::string *reason = nullptr);
  void reset();

  /// Propagates exactly one caller-supplied IMU interval. Repeating an already
  /// committed step is an idempotent no-op.
  bool propagate(const CandidateImuStep &step, std::string *reason = nullptr);

  /// Applies position and velocity updates atomically. NIS rejection is a
  /// committed/consumed outcome; numerical failure rolls the entire event
  /// back and leaves the FC cursor unchanged.
  CandidatePositionVelocityUpdateResult updatePositionVelocity(
      const CandidatePositionVelocityMeasurement &measurement);

  /// Applies the currently retained error of one group. This is public so the
  /// supervisor can explicitly trigger feedback, while the standard p/v and
  /// gated q/bg/ba paths call it internally.
  CandidateFeedbackResult feedbackGroup(CandidateStateGroup group);

  const CandidateFilterRuntime &runtime() const { return runtime_; }
  const CandidateFilterConfig &config() const { return config_; }

  bool groupReady(CandidateStateGroup group) const;
  bool navigationReady() const;
  bool fullAlignmentReady() const;
  CandidateFeedbackTier feedbackTier() const;

  /// Builds a conservative covariance for a practical release that retains an
  /// untrusted bias at its initial nominal value.
  CandidateCovariance releaseCovariance(bool trust_gyro_bias,
                                        bool trust_accel_bias,
                                        CandidateCovarianceHealth *health =
                                            nullptr) const;

private:
  CandidateFilterConfig config_;
  CandidateFilterRuntime runtime_;

  bool propagateRuntime(CandidateFilterRuntime &runtime,
                        const CandidateImuStep &step,
                        std::string *reason) const;
  CandidateMeasurementStatus updateMeasurement(
      CandidateFilterRuntime &runtime, const Eigen::Vector3d &measurement,
      int state_offset, double sigma, double &nis,
      std::array<bool,
                 static_cast<std::size_t>(CandidateStateGroup::COUNT)>
          &group_supported,
      std::string &reason) const;
  void updateGroupHistories(
      CandidateFilterRuntime &runtime, double board_time,
      const std::array<bool,
                       static_cast<std::size_t>(CandidateStateGroup::COUNT)>
          &group_supported,
      bool accepted_event) const;
  bool gatePassed(const CandidateFilterRuntime &runtime,
                  CandidateStateGroup group) const;
  CandidateFeedbackResult feedbackGroup(CandidateFilterRuntime &runtime,
                                        CandidateStateGroup group) const;
  CandidateCovarianceHealth stabilizeCovariance(
      CandidateCovariance &covariance) const;
};

const char *candidate_state_group_name(CandidateStateGroup group);
const char *candidate_feedback_tier_name(CandidateFeedbackTier tier);

} // namespace ov_msckf

#endif // OV_MSCKF_ONLINE_ALIGNMENT_CANDIDATE_FILTER_H
