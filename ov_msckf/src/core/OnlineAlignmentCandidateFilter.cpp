#include "OnlineAlignmentCandidateFilter.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ov_msckf {
namespace {

constexpr std::size_t kGroupCount =
    static_cast<std::size_t>(CandidateStateGroup::COUNT);

std::size_t group_index(CandidateStateGroup group) {
  return static_cast<std::size_t>(group);
}

int group_offset(CandidateStateGroup group) {
  switch (group) {
  case CandidateStateGroup::ATTITUDE:
    return 0;
  case CandidateStateGroup::POSITION:
    return 3;
  case CandidateStateGroup::VELOCITY:
    return 6;
  case CandidateStateGroup::GYRO_BIAS:
    return 9;
  case CandidateStateGroup::ACCEL_BIAS:
    return 12;
  case CandidateStateGroup::COUNT:
    break;
  }
  return -1;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &value) {
  Eigen::Matrix3d out;
  out << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return out;
}

Eigen::Matrix3d so3_exp(const Eigen::Vector3d &value) {
  const double angle = value.norm();
  if (angle < 1.0e-12)
    return Eigen::Matrix3d::Identity() + skew(value);
  return Eigen::AngleAxisd(angle, value / angle).toRotationMatrix();
}

Eigen::Vector3d so3_log(const Eigen::Matrix3d &rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1.0e-12)
    return Eigen::Vector3d::Zero();
  return angle_axis.axis() * angle_axis.angle();
}

bool nominal_finite(const CandidateNominalState &state) {
  return state.R_GtoI.allFinite() && state.p_IinG.allFinite() &&
         state.v_IinG.allFinite() && state.bg.allFinite() &&
         state.ba.allFinite();
}

bool rotation_valid(const Eigen::Matrix3d &rotation) {
  return rotation.allFinite() &&
         std::fabs(rotation.determinant() - 1.0) <= 1.0e-6 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                 .norm() <=
             1.0e-6;
}

bool limits_valid(const Eigen::Vector3d &limits) {
  for (int axis = 0; axis < 3; ++axis) {
    const double limit = limits(axis);
    if (std::isnan(limit) || limit < 0.0)
      return false;
  }
  return true;
}

bool gate_valid(const CandidateGroupGate &gate) {
  if (!gate.configured)
    return true;
  const bool depth_valid =
      gate.derive_depth_from_sliding_window ||
      (gate.history_duration_s > 0.0 &&
       std::isfinite(gate.history_duration_s)) ||
      (gate.history_length_updates > 0 && gate.min_supported_updates > 0);
  return depth_valid &&
         gate.required_post_feedback_stable_updates >= 0 &&
         limits_valid(gate.max_abs_error) && limits_valid(gate.max_std) &&
         limits_valid(gate.max_std_step) &&
         limits_valid(gate.max_error_peak_to_peak) &&
         limits_valid(gate.max_std_peak_to_peak) &&
         limits_valid(gate.max_feedback_step) &&
         limits_valid(gate.max_cumulative_feedback);
}

bool config_valid(const CandidateFilterConfig &config) {
  if (!config.gravity_G.allFinite() ||
      !(config.position_sigma_m > 0.0) ||
      !(config.velocity_sigma_mps > 0.0) || !(config.nis_gate_3d > 0.0) ||
      !std::isfinite(config.position_sigma_m) ||
      !std::isfinite(config.velocity_sigma_mps) ||
      !std::isfinite(config.nis_gate_3d) ||
      !(config.covariance_roundoff_relative_tolerance >= 0.0) ||
      !std::isfinite(config.covariance_roundoff_relative_tolerance) ||
      !(config.time_tolerance_s >= 0.0) ||
      !std::isfinite(config.time_tolerance_s))
    return false;
  const auto noise_valid = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (!noise_valid(config.noise.sigma_w) ||
      !noise_valid(config.noise.sigma_wb) ||
      !noise_valid(config.noise.sigma_a) ||
      !noise_valid(config.noise.sigma_ab))
    return false;
  if (!std::all_of(config.group_gates.begin(), config.group_gates.end(),
                   gate_valid))
    return false;
  for (const CandidateStateGroup group :
       {CandidateStateGroup::ATTITUDE, CandidateStateGroup::GYRO_BIAS,
        CandidateStateGroup::ACCEL_BIAS}) {
    const CandidateGroupGate &gate = config.group_gates[group_index(group)];
    if (gate.configured && gate.required_post_feedback_stable_updates <= 0)
      return false;
  }
  return true;
}

void inject_nominal(CandidateNominalState &state,
                    const CandidateErrorState &correction) {
  state.R_GtoI = so3_exp(-correction.segment<3>(0)) * state.R_GtoI;
  state.p_IinG += correction.segment<3>(3);
  state.v_IinG += correction.segment<3>(6);
  state.bg += correction.segment<3>(9);
  state.ba += correction.segment<3>(12);
}

CandidateNominalState propagate_nominal_midpoint(
    const CandidateNominalState &start, const CandidateImuStep &step,
    const Eigen::Vector3d &gravity_G) {
  CandidateNominalState end = start;
  const double dt = step.end_time - step.start_time;
  const Eigen::Vector3d omega_mid =
      0.5 * (step.angular_velocity_start + step.angular_velocity_end) -
      start.bg;
  const Eigen::Vector3d acceleration_mid =
      0.5 * (step.linear_acceleration_start +
             step.linear_acceleration_end) -
      start.ba;
  const Eigen::Matrix3d R_mid = so3_exp(-0.5 * omega_mid * dt) *
                                start.R_GtoI;
  const Eigen::Vector3d acceleration_G =
      R_mid.transpose() * acceleration_mid - gravity_G;
  end.p_IinG = start.p_IinG + start.v_IinG * dt +
               0.5 * acceleration_G * dt * dt;
  end.v_IinG = start.v_IinG + acceleration_G * dt;
  end.R_GtoI = so3_exp(-omega_mid * dt) * start.R_GtoI;
  return end;
}

Eigen::Vector3d nominal_group(const CandidateNominalState &state,
                              int offset) {
  switch (offset) {
  case 3:
    return state.p_IinG;
  case 6:
    return state.v_IinG;
  case 9:
    return state.bg;
  case 12:
    return state.ba;
  default:
    return Eigen::Vector3d::Zero();
  }
}

bool componentwise_within(const Eigen::Vector3d &value,
                          const Eigen::Vector3d &limit) {
  return (value.cwiseAbs().array() <= limit.array()).all();
}

Eigen::Vector3d peak_to_peak_error(
    const std::deque<CandidateConvergenceSample> &history, bool use_std) {
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  for (const auto &sample : history) {
    const Eigen::Vector3d &value = use_std ? sample.std : sample.error;
    minimum = minimum.cwiseMin(value);
    maximum = maximum.cwiseMax(value);
  }
  return maximum - minimum;
}

Eigen::Vector3d clamp_correction(const Eigen::Vector3d &requested,
                                 const Eigen::Vector3d &limit,
                                 bool &clipped) {
  Eigen::Vector3d applied = requested;
  clipped = false;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::isfinite(limit(axis)) &&
        std::fabs(applied(axis)) > limit(axis)) {
      applied(axis) = std::copysign(limit(axis), applied(axis));
      clipped = true;
    }
  }
  return applied;
}

} // namespace

const char *candidate_state_group_name(CandidateStateGroup group) {
  switch (group) {
  case CandidateStateGroup::ATTITUDE:
    return "attitude";
  case CandidateStateGroup::POSITION:
    return "position";
  case CandidateStateGroup::VELOCITY:
    return "velocity";
  case CandidateStateGroup::GYRO_BIAS:
    return "gyro_bias";
  case CandidateStateGroup::ACCEL_BIAS:
    return "accel_bias";
  case CandidateStateGroup::COUNT:
    break;
  }
  return "unknown";
}

const char *candidate_feedback_tier_name(CandidateFeedbackTier tier) {
  switch (tier) {
  case CandidateFeedbackTier::PV_ONLY:
    return "PV_ONLY";
  case CandidateFeedbackTier::ATTITUDE:
    return "ATTITUDE";
  case CandidateFeedbackTier::ATTITUDE_GYRO_BIAS:
    return "ATTITUDE_BG";
  case CandidateFeedbackTier::ATTITUDE_ACCEL_BIAS:
    return "ATTITUDE_BA";
  case CandidateFeedbackTier::FULL:
    return "FULL";
  }
  return "UNKNOWN";
}

OnlineAlignmentCandidateFilter::OnlineAlignmentCandidateFilter(
    const CandidateFilterConfig &config)
    : config_(config) {}

bool OnlineAlignmentCandidateFilter::initialize(
    double board_time, const CandidateNominalState &nominal,
    const CandidateCovariance &covariance,
    const std::array<bool, kGroupCount> &initial_graph_data_support,
    const CandidateErrorState &initial_error, std::string *reason) {
  const auto fail = [&](const std::string &message) {
    if (reason)
      *reason = message;
    return false;
  };
  if (!config_valid(config_))
    return fail("candidate_filter_config_invalid");
  if (!std::isfinite(board_time) || !nominal_finite(nominal) ||
      !rotation_valid(nominal.R_GtoI) || !initial_error.allFinite() ||
      !covariance.allFinite())
    return fail("candidate_filter_initial_state_invalid");

  CandidateFilterRuntime initialized;
  initialized.active = true;
  initialized.board_time = board_time;
  initialized.last_imu_step_end_time = board_time;
  initialized.last_fc_source_timestamp =
      -std::numeric_limits<double>::infinity();
  initialized.last_fc_board_time =
      -std::numeric_limits<double>::infinity();
  initialized.nominal = nominal;
  initialized.initial_nominal = nominal;
  initialized.dx = initial_error;
  initialized.P = covariance;
  const CandidateCovarianceHealth health =
      stabilizeCovariance(initialized.P);
  initialized.covariance_health = health;
  if (health == CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
    ++initialized.covariance_roundoff_repair_count;
  if (health != CandidateCovarianceHealth::HEALTHY &&
      health != CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
    return fail("candidate_filter_initial_covariance_invalid");
  initialized.initial_P = initialized.P;
  for (std::size_t index = 0; index < kGroupCount; ++index)
    initialized.groups[index].initial_graph_data_supported =
        initial_graph_data_support[index];
  runtime_ = std::move(initialized);
  if (reason)
    reason->clear();
  return true;
}

void OnlineAlignmentCandidateFilter::reset() {
  runtime_ = CandidateFilterRuntime();
}

bool OnlineAlignmentCandidateFilter::propagate(const CandidateImuStep &step,
                                               std::string *reason) {
  const auto fail = [&](const std::string &message) {
    if (reason)
      *reason = message;
    return false;
  };
  if (!runtime_.active)
    return fail("candidate_filter_inactive");
  if (!std::isfinite(step.start_time) || !std::isfinite(step.end_time))
    return fail("candidate_imu_step_time_invalid");
  if (step.end_time <= runtime_.board_time + config_.time_tolerance_s) {
    ++runtime_.duplicate_imu_step_count;
    if (reason)
      reason->clear();
    return true;
  }
  if (std::fabs(step.start_time - runtime_.board_time) >
      config_.time_tolerance_s)
    return fail("candidate_imu_step_not_contiguous");

  CandidateFilterRuntime trial = runtime_;
  std::string local_reason;
  if (!propagateRuntime(trial, step, &local_reason))
    return fail(local_reason);
  runtime_ = std::move(trial);
  if (reason)
    reason->clear();
  return true;
}

bool OnlineAlignmentCandidateFilter::propagateRuntime(
    CandidateFilterRuntime &runtime, const CandidateImuStep &step,
    std::string *reason) const {
  const auto fail = [&](const std::string &message) {
    if (reason)
      *reason = message;
    return false;
  };
  const double dt = step.end_time - step.start_time;
  if (!(dt > 0.0) ||
      !step.angular_velocity_start.allFinite() ||
      !step.angular_velocity_end.allFinite() ||
      !step.linear_acceleration_start.allFinite() ||
      !step.linear_acceleration_end.allFinite())
    return fail("candidate_imu_step_invalid");

  const CandidateNominalState nominal_start = runtime.nominal;
  const CandidateNominalState nominal_end = propagate_nominal_midpoint(
      nominal_start, step, config_.gravity_G);
  if (!nominal_finite(nominal_end))
    return fail("candidate_nominal_propagation_non_finite");

  CandidateCovariance transition = CandidateCovariance::Zero();
  for (int axis = 0; axis < 15; ++axis) {
    const double epsilon = axis < 3 || axis >= 9 ? 1.0e-6 : 1.0e-4;
    CandidateErrorState perturbation = CandidateErrorState::Zero();
    perturbation(axis) = epsilon;
    CandidateNominalState perturbed_start = nominal_start;
    inject_nominal(perturbed_start, perturbation);
    const CandidateNominalState perturbed_end = propagate_nominal_midpoint(
        perturbed_start, step, config_.gravity_G);
    CandidateErrorState final_error = CandidateErrorState::Zero();
    final_error.segment<3>(0) =
        -so3_log(perturbed_end.R_GtoI * nominal_end.R_GtoI.transpose());
    final_error.segment<3>(3) =
        perturbed_end.p_IinG - nominal_end.p_IinG;
    final_error.segment<3>(6) =
        perturbed_end.v_IinG - nominal_end.v_IinG;
    final_error.segment<3>(9) = perturbed_end.bg - nominal_end.bg;
    final_error.segment<3>(12) = perturbed_end.ba - nominal_end.ba;
    transition.col(axis) = final_error / epsilon;
  }
  if (!transition.allFinite())
    return fail("candidate_error_transition_non_finite");

  CandidateErrorState propagated_error = transition * runtime.dx;
  if (!propagated_error.allFinite())
    return fail("candidate_error_state_propagation_non_finite");

  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  CandidateCovariance process = CandidateCovariance::Zero();
  const double gyro_variance = config_.noise.sigma_w * config_.noise.sigma_w;
  const double accel_variance = config_.noise.sigma_a * config_.noise.sigma_a;
  process.block<3, 3>(0, 0).diagonal().array() = gyro_variance * dt;
  process.block<3, 3>(3, 3).diagonal().array() =
      accel_variance * dt3 / 3.0;
  process.block<3, 3>(6, 6).diagonal().array() = accel_variance * dt;
  process.block<3, 3>(3, 6).diagonal().array() =
      accel_variance * dt2 / 2.0;
  process.block<3, 3>(6, 3) = process.block<3, 3>(3, 6).transpose();
  process.block<3, 3>(9, 9).diagonal().array() =
      config_.noise.sigma_wb * config_.noise.sigma_wb * dt;
  process.block<3, 3>(12, 12).diagonal().array() =
      config_.noise.sigma_ab * config_.noise.sigma_ab * dt;

  CandidateCovariance propagated_covariance =
      transition * runtime.P * transition.transpose() + process;
  const CandidateCovarianceHealth health =
      stabilizeCovariance(propagated_covariance);
  runtime.covariance_health = health;
  if (health == CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
    ++runtime.covariance_roundoff_repair_count;
  if (health != CandidateCovarianceHealth::HEALTHY &&
      health != CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
    return fail("candidate_covariance_propagation_invalid");

  runtime.nominal = nominal_end;
  runtime.dx = propagated_error;
  runtime.P = propagated_covariance;
  runtime.board_time = step.end_time;
  runtime.last_imu_step_end_time = step.end_time;
  ++runtime.propagated_imu_step_count;
  if (reason)
    reason->clear();
  return true;
}

CandidatePositionVelocityUpdateResult
OnlineAlignmentCandidateFilter::updatePositionVelocity(
    const CandidatePositionVelocityMeasurement &measurement) {
  CandidatePositionVelocityUpdateResult result;
  for (std::size_t index = 0; index < kGroupCount; ++index)
    result.feedback[index].group =
        static_cast<CandidateStateGroup>(index);
  if (!runtime_.active) {
    result.reason = "candidate_filter_inactive";
    return result;
  }
  if (!std::isfinite(measurement.source_timestamp) ||
      !std::isfinite(measurement.board_time)) {
    result.reason = "candidate_fc_event_time_invalid";
    return result;
  }
  if (measurement.source_timestamp <=
      runtime_.last_fc_source_timestamp + config_.time_tolerance_s) {
    ++runtime_.duplicate_fc_event_count;
    result.duplicate = true;
    return result;
  }
  if (std::fabs(measurement.board_time - runtime_.board_time) >
      config_.time_tolerance_s) {
    result.reason = "candidate_fc_event_requires_prior_propagation";
    return result;
  }

  CandidateFilterRuntime trial = runtime_;
  std::string update_reason;
  if (measurement.position_valid) {
    const double sigma = measurement.position_sigma_m < 0.0
                             ? config_.position_sigma_m
                             : measurement.position_sigma_m;
    result.position_status = updateMeasurement(
        trial, measurement.position_G, 3, sigma, result.position_nis,
        result.group_supported, update_reason);
    if (result.position_status ==
        CandidateMeasurementStatus::NUMERICAL_FAILURE) {
      result.reason = update_reason;
      return result;
    }
  }
  if (measurement.velocity_valid) {
    const double sigma = measurement.velocity_sigma_mps < 0.0
                             ? config_.velocity_sigma_mps
                             : measurement.velocity_sigma_mps;
    result.velocity_status = updateMeasurement(
        trial, measurement.velocity_G, 6, sigma, result.velocity_nis,
        result.group_supported, update_reason);
    if (result.velocity_status ==
        CandidateMeasurementStatus::NUMERICAL_FAILURE) {
      result.reason = update_reason;
      return result;
    }
  }

  if (std::isfinite(result.position_nis)) {
    trial.max_position_nis =
        std::isfinite(trial.max_position_nis)
            ? std::max(trial.max_position_nis, result.position_nis)
            : result.position_nis;
  }
  if (std::isfinite(result.velocity_nis)) {
    trial.max_velocity_nis =
        std::isfinite(trial.max_velocity_nis)
            ? std::max(trial.max_velocity_nis, result.velocity_nis)
            : result.velocity_nis;
  }

  const bool position_accepted =
      result.position_status == CandidateMeasurementStatus::ACCEPTED;
  const bool velocity_accepted =
      result.velocity_status == CandidateMeasurementStatus::ACCEPTED;
  const bool accepted_event = position_accepted || velocity_accepted;
  updateGroupHistories(trial, measurement.board_time,
                       result.group_supported, accepted_event);

  if (accepted_event) {
    result.feedback[group_index(CandidateStateGroup::POSITION)] =
        feedbackGroup(trial, CandidateStateGroup::POSITION);
    result.feedback[group_index(CandidateStateGroup::VELOCITY)] =
        feedbackGroup(trial, CandidateStateGroup::VELOCITY);
    for (const CandidateStateGroup group :
         {CandidateStateGroup::ATTITUDE, CandidateStateGroup::GYRO_BIAS,
          CandidateStateGroup::ACCEL_BIAS}) {
      const std::size_t index = group_index(group);
      const CandidateGroupRuntime &group_runtime = trial.groups[index];
      const bool first_feedback = group_runtime.feedback_count == 0;
      // q/bg/ba receive one data-gated injection first. Later supported
      // updates are reserved for post-feedback validation. Re-injecting on
      // the same update that completes the hold would make the group ready
      // before the newest correction has ever been propagated or observed.
      if (group_runtime.last_gate_passed && first_feedback)
        result.feedback[index] = feedbackGroup(trial, group);
    }
    for (const CandidateFeedbackResult &feedback : result.feedback) {
      if (feedback.safety_violation && !feedback.applied) {
        result.reason = "candidate_feedback_numerical_failure";
        return result;
      }
    }
  }

  trial.last_fc_source_timestamp = measurement.source_timestamp;
  trial.last_fc_board_time = measurement.board_time;
  trial.last_position_nis = result.position_nis;
  trial.last_velocity_nis = result.velocity_nis;
  ++trial.consumed_fc_event_count;
  if (position_accepted)
    ++trial.accepted_position_count;
  else if (result.position_status == CandidateMeasurementStatus::NIS_REJECTED ||
           result.position_status == CandidateMeasurementStatus::INVALID)
    ++trial.rejected_position_count;
  if (velocity_accepted)
    ++trial.accepted_velocity_count;
  else if (result.velocity_status == CandidateMeasurementStatus::NIS_REJECTED ||
           result.velocity_status == CandidateMeasurementStatus::INVALID)
    ++trial.rejected_velocity_count;
  if (accepted_event) {
    ++trial.accepted_fc_event_count;
  } else {
    ++trial.rejected_fc_event_count;
  }

  runtime_ = std::move(trial);
  result.committed = true;
  if (runtime_.safety_violation)
    result.reason = "candidate_feedback_safety_violation";
  return result;
}

CandidateMeasurementStatus OnlineAlignmentCandidateFilter::updateMeasurement(
    CandidateFilterRuntime &runtime, const Eigen::Vector3d &measurement,
    int state_offset, double sigma, double &nis,
    std::array<bool, kGroupCount> &group_supported,
    std::string &reason) const {
  if (!measurement.allFinite() || !(sigma > 0.0) || !std::isfinite(sigma)) {
    reason = "candidate_measurement_invalid";
    return CandidateMeasurementStatus::INVALID;
  }
  Eigen::Matrix<double, 3, 15> H =
      Eigen::Matrix<double, 3, 15>::Zero();
  H.block<3, 3>(0, state_offset).setIdentity();
  const Eigen::Vector3d innovation =
      measurement - nominal_group(runtime.nominal, state_offset) -
      H * runtime.dx;
  const Eigen::Matrix3d measurement_covariance =
      Eigen::Matrix3d::Identity() * sigma * sigma;
  const Eigen::Matrix3d innovation_covariance =
      H * runtime.P * H.transpose() + measurement_covariance;
  const Eigen::LDLT<Eigen::Matrix3d> ldlt(innovation_covariance);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    reason = "candidate_innovation_covariance_invalid";
    return CandidateMeasurementStatus::NUMERICAL_FAILURE;
  }
  nis = innovation.dot(ldlt.solve(innovation));
  if (!std::isfinite(nis)) {
    reason = "candidate_nis_non_finite";
    return CandidateMeasurementStatus::NUMERICAL_FAILURE;
  }
  if (nis > config_.nis_gate_3d)
    return CandidateMeasurementStatus::NIS_REJECTED;

  const CandidateCovariance prior_covariance = runtime.P;
  const Eigen::Matrix<double, 15, 3> gain =
      runtime.P * H.transpose() *
      ldlt.solve(Eigen::Matrix3d::Identity());
  const CandidateErrorState posterior_error =
      runtime.dx + gain * innovation;
  const CandidateCovariance identity = CandidateCovariance::Identity();
  const CandidateCovariance correction = identity - gain * H;
  CandidateCovariance posterior_covariance =
      correction * runtime.P * correction.transpose() +
      gain * measurement_covariance * gain.transpose();
  const CandidateCovarianceHealth health =
      stabilizeCovariance(posterior_covariance);
  runtime.covariance_health = health;
  if (health == CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
    ++runtime.covariance_roundoff_repair_count;
  if (health != CandidateCovarianceHealth::HEALTHY &&
      health != CandidateCovarianceHealth::ROUNDOFF_REPAIRED) {
    reason = "candidate_measurement_covariance_invalid";
    return CandidateMeasurementStatus::NUMERICAL_FAILURE;
  }
  if (!posterior_error.allFinite()) {
    reason = "candidate_measurement_error_state_non_finite";
    return CandidateMeasurementStatus::NUMERICAL_FAILURE;
  }

  for (std::size_t index = 0; index < kGroupCount; ++index) {
    const int offset =
        group_offset(static_cast<CandidateStateGroup>(index));
    const double prior_trace =
        prior_covariance.block<3, 3>(offset, offset).trace();
    const double posterior_trace =
        posterior_covariance.block<3, 3>(offset, offset).trace();
    const double trace_scale =
        std::max({std::fabs(prior_trace), std::fabs(posterior_trace),
                  std::numeric_limits<double>::min()});
    const bool information_gain =
        prior_trace - posterior_trace >
        64.0 * std::numeric_limits<double>::epsilon() * trace_scale;
    group_supported[index] = group_supported[index] || information_gain;
  }

  runtime.dx = posterior_error;
  runtime.P = posterior_covariance;
  reason.clear();
  return CandidateMeasurementStatus::ACCEPTED;
}

void OnlineAlignmentCandidateFilter::updateGroupHistories(
    CandidateFilterRuntime &runtime, double board_time,
    const std::array<bool, kGroupCount> &group_supported,
    bool accepted_event) const {
  for (std::size_t index = 0; index < kGroupCount; ++index) {
    CandidateGroupRuntime &group_runtime = runtime.groups[index];
    group_runtime.last_feedback_clipped = false;
    if (!accepted_event || !group_supported[index]) {
      group_runtime.history.clear();
      group_runtime.consecutive_stable_updates = 0;
      group_runtime.post_first_feedback_stable_updates = 0;
      group_runtime.converged = false;
      group_runtime.last_gate_passed = false;
      continue;
    }
    const CandidateStateGroup group =
        static_cast<CandidateStateGroup>(index);
    const int offset = group_offset(group);
    CandidateConvergenceSample sample;
    sample.board_time = board_time;
    sample.error = runtime.dx.segment<3>(offset);
    sample.std = runtime.P.block<3, 3>(offset, offset)
                     .diagonal()
                     .cwiseMax(0.0)
                     .cwiseSqrt();
    group_runtime.history.push_back(sample);
    const CandidateGroupGate &gate = config_.group_gates[index];
    if (gate.configured && gate.history_duration_s > 0.0 &&
        std::isfinite(gate.history_duration_s)) {
      while (group_runtime.history.size() > 1 &&
             group_runtime.history.back().board_time -
                     group_runtime.history.front().board_time >
                 gate.history_duration_s)
        group_runtime.history.pop_front();
    } else {
      const std::size_t retained =
          gate.configured ? std::max<std::size_t>(1, gate.history_length_updates)
                          : 1;
      while (group_runtime.history.size() > retained)
        group_runtime.history.pop_front();
    }
    ++group_runtime.supported_update_count;

    const bool passed = gatePassed(runtime, group);
    group_runtime.last_gate_passed = passed;
    if (passed) {
      ++group_runtime.consecutive_stable_updates;
      if (group_runtime.feedback_count > 0)
        ++group_runtime.post_first_feedback_stable_updates;
      group_runtime.converged =
          group_runtime.feedback_count == 0 ||
          group_runtime.post_first_feedback_stable_updates >=
              gate.required_post_feedback_stable_updates;
    } else {
      group_runtime.consecutive_stable_updates = 0;
      group_runtime.post_first_feedback_stable_updates = 0;
      group_runtime.converged = false;
    }
  }
}

bool OnlineAlignmentCandidateFilter::gatePassed(
    const CandidateFilterRuntime &runtime, CandidateStateGroup group) const {
  const std::size_t index = group_index(group);
  if (index >= kGroupCount || runtime.safety_violation)
    return false;
  const CandidateGroupGate &gate = config_.group_gates[index];
  const CandidateGroupRuntime &group_runtime = runtime.groups[index];
  if (!gate.configured || !gate_valid(gate) ||
      group_runtime.last_feedback_clipped)
    return false;
  // The selected sliding-window duration is a post-feedback validation
  // history, not a prerequisite for the first causal correction.  This
  // decision is intentionally independent of the pre-resolution flag.
  const bool early_feedback =
      gate.allow_initial_feedback_before_derived_history &&
      group_runtime.feedback_count == 0;
  if (!early_feedback) {
    if (gate.history_duration_s > 0.0 &&
        std::isfinite(gate.history_duration_s)) {
      if (group_runtime.history.size() < 2 ||
          group_runtime.history.back().board_time -
                  group_runtime.history.front().board_time +
              config_.time_tolerance_s <
                  gate.history_duration_s)
        return false;
    } else if (group_runtime.supported_update_count <
                   gate.min_supported_updates ||
               group_runtime.history.size() < gate.history_length_updates) {
      return false;
    }
  }
  if (gate.require_initial_graph_support &&
      !group_runtime.initial_graph_data_supported)
    return false;

  const CandidateConvergenceSample &current = group_runtime.history.back();
  if (!current.error.allFinite() || !current.std.allFinite() ||
      !componentwise_within(current.error, gate.max_abs_error) ||
      !componentwise_within(current.std, gate.max_std))
    return false;
  if (group_runtime.history.size() >= 2) {
    const Eigen::Vector3d std_step =
        current.std - group_runtime.history[group_runtime.history.size() - 2].std;
    if (!componentwise_within(std_step, gate.max_std_step))
      return false;
  }
  if (!componentwise_within(peak_to_peak_error(group_runtime.history, false),
                            gate.max_error_peak_to_peak) ||
      !componentwise_within(peak_to_peak_error(group_runtime.history, true),
                            gate.max_std_peak_to_peak))
    return false;
  return true;
}

CandidateFeedbackResult OnlineAlignmentCandidateFilter::feedbackGroup(
    CandidateStateGroup group) {
  if (!runtime_.active) {
    CandidateFeedbackResult result;
    result.group = group;
    result.safety_violation = true;
    return result;
  }
  return feedbackGroup(runtime_, group);
}

CandidateFeedbackResult OnlineAlignmentCandidateFilter::feedbackGroup(
    CandidateFilterRuntime &runtime, CandidateStateGroup group) const {
  CandidateFeedbackResult result;
  result.group = group;
  const std::size_t index = group_index(group);
  const int offset = group_offset(group);
  if (index >= kGroupCount || offset < 0 || !runtime.active) {
    result.safety_violation = true;
    return result;
  }

  CandidateFilterRuntime trial = runtime;
  CandidateGroupRuntime &group_runtime = trial.groups[index];
  const CandidateGroupGate &gate = config_.group_gates[index];
  result.requested = trial.dx.segment<3>(offset);
  if (!result.requested.allFinite()) {
    result.safety_violation = true;
    return result;
  }
  result.applied_correction = clamp_correction(
      result.requested, gate.max_feedback_step, result.clipped);
  CandidateErrorState applied = CandidateErrorState::Zero();
  applied.segment<3>(offset) = result.applied_correction;

  if (group == CandidateStateGroup::ATTITUDE) {
    trial.nominal.R_GtoI =
        so3_exp(-result.applied_correction) * trial.nominal.R_GtoI;
    CandidateCovariance reset = CandidateCovariance::Identity();
    reset.block<3, 3>(0, 0) -=
        0.5 * skew(result.applied_correction);
    trial.dx = reset * (trial.dx - applied);
    trial.P = reset * trial.P * reset.transpose();
    const CandidateCovarianceHealth health = stabilizeCovariance(trial.P);
    trial.covariance_health = health;
    if (health == CandidateCovarianceHealth::ROUNDOFF_REPAIRED)
      ++trial.covariance_roundoff_repair_count;
    if (health != CandidateCovarianceHealth::HEALTHY &&
        health != CandidateCovarianceHealth::ROUNDOFF_REPAIRED) {
      result.safety_violation = true;
      return result;
    }
  } else {
    inject_nominal(trial.nominal, applied);
    trial.dx -= applied;
  }
  if (!nominal_finite(trial.nominal) || !trial.dx.allFinite()) {
    result.safety_violation = true;
    return result;
  }

  const Eigen::Vector3d cumulative =
      group_runtime.cumulative_feedback + result.applied_correction;
  const bool cumulative_violation =
      !componentwise_within(cumulative, gate.max_cumulative_feedback);
  group_runtime.cumulative_feedback = cumulative;
  group_runtime.last_feedback = result.applied_correction;
  group_runtime.last_feedback_clipped = result.clipped;
  if (result.clipped)
    ++group_runtime.clipped_feedback_count;
  ++group_runtime.feedback_count;
  if (group == CandidateStateGroup::ATTITUDE ||
      group == CandidateStateGroup::GYRO_BIAS ||
      group == CandidateStateGroup::ACCEL_BIAS) {
    group_runtime.converged = false;
    group_runtime.consecutive_stable_updates = 0;
    group_runtime.post_first_feedback_stable_updates = 0;
    group_runtime.last_gate_passed = false;
  }
  if (result.clipped) {
    group_runtime.last_gate_passed = false;
    group_runtime.consecutive_stable_updates = 0;
    group_runtime.post_first_feedback_stable_updates = 0;
  }
  trial.safety_violation = trial.safety_violation || cumulative_violation;
  result.safety_violation = trial.safety_violation;
  result.applied = true;
  runtime = std::move(trial);
  return result;
}

bool OnlineAlignmentCandidateFilter::groupReady(
    CandidateStateGroup group) const {
  const std::size_t index = group_index(group);
  if (!runtime_.active || index >= kGroupCount || runtime_.safety_violation)
    return false;
  const CandidateGroupGate &gate = config_.group_gates[index];
  const CandidateGroupRuntime &group_runtime = runtime_.groups[index];
  if (!gate.configured || !group_runtime.last_gate_passed ||
      group_runtime.last_feedback_clipped || !group_runtime.converged)
    return false;
  if (group == CandidateStateGroup::POSITION ||
      group == CandidateStateGroup::VELOCITY)
    if (gate.history_duration_s > 0.0 &&
        std::isfinite(gate.history_duration_s))
      return group_runtime.history.size() >= 2 &&
             group_runtime.history.back().board_time -
                     group_runtime.history.front().board_time +
                 config_.time_tolerance_s >=
                 gate.history_duration_s;
    else
      return group_runtime.supported_update_count >=
             gate.min_supported_updates;
  return group_runtime.feedback_count > 0 &&
         group_runtime.post_first_feedback_stable_updates >=
             gate.required_post_feedback_stable_updates;
}

bool OnlineAlignmentCandidateFilter::navigationReady() const {
  return groupReady(CandidateStateGroup::ATTITUDE) &&
         groupReady(CandidateStateGroup::POSITION) &&
         groupReady(CandidateStateGroup::VELOCITY);
}

bool OnlineAlignmentCandidateFilter::fullAlignmentReady() const {
  return navigationReady() && groupReady(CandidateStateGroup::GYRO_BIAS) &&
         groupReady(CandidateStateGroup::ACCEL_BIAS);
}

CandidateFeedbackTier OnlineAlignmentCandidateFilter::feedbackTier() const {
  const bool attitude = groupReady(CandidateStateGroup::ATTITUDE);
  const bool gyro_bias = groupReady(CandidateStateGroup::GYRO_BIAS);
  const bool accel_bias = groupReady(CandidateStateGroup::ACCEL_BIAS);
  if (!attitude)
    return CandidateFeedbackTier::PV_ONLY;
  if (gyro_bias && accel_bias)
    return CandidateFeedbackTier::FULL;
  if (gyro_bias)
    return CandidateFeedbackTier::ATTITUDE_GYRO_BIAS;
  if (accel_bias)
    return CandidateFeedbackTier::ATTITUDE_ACCEL_BIAS;
  return CandidateFeedbackTier::ATTITUDE;
}

CandidateCovariance OnlineAlignmentCandidateFilter::releaseCovariance(
    bool trust_gyro_bias, bool trust_accel_bias,
    CandidateCovarianceHealth *health) const {
  CandidateCovariance released = runtime_.P;
  const auto retain_prior = [&](int offset) {
    for (int axis = 0; axis < 3; ++axis) {
      released.row(offset + axis).setZero();
      released.col(offset + axis).setZero();
    }
    const Eigen::Vector3d retained_error =
        runtime_.dx.segment<3>(offset) +
        nominal_group(runtime_.nominal, offset) -
        nominal_group(runtime_.initial_nominal, offset);
    released.block<3, 3>(offset, offset) =
        runtime_.initial_P.block<3, 3>(offset, offset) +
        retained_error * retained_error.transpose();
  };
  if (!trust_gyro_bias)
    retain_prior(9);
  if (!trust_accel_bias)
    retain_prior(12);
  const CandidateCovarianceHealth covariance_health =
      stabilizeCovariance(released);
  if (health)
    *health = covariance_health;
  return released;
}

CandidateCovarianceHealth
OnlineAlignmentCandidateFilter::stabilizeCovariance(
    CandidateCovariance &covariance) const {
  covariance = 0.5 * (covariance + covariance.transpose());
  if (!covariance.allFinite())
    return CandidateCovarianceHealth::NON_FINITE;
  const Eigen::SelfAdjointEigenSolver<CandidateCovariance> eig(covariance);
  if (eig.info() != Eigen::Success)
    return CandidateCovarianceHealth::EIGENSOLVER_FAILURE;
  const double scale =
      std::max(eig.eigenvalues().cwiseAbs().maxCoeff(),
               std::numeric_limits<double>::min());
  const double minimum = eig.eigenvalues().minCoeff();
  const double repair_band =
      config_.covariance_roundoff_relative_tolerance * scale;
  if (minimum < -repair_band)
    return CandidateCovarianceHealth::MATERIAL_NEGATIVE_EIGENVALUE;
  if (minimum < 0.0) {
    const double numerical_margin =
        64.0 * std::numeric_limits<double>::epsilon() * scale;
    covariance.diagonal().array() += -minimum + numerical_margin;
    covariance = 0.5 * (covariance + covariance.transpose());
    return CandidateCovarianceHealth::ROUNDOFF_REPAIRED;
  }
  return CandidateCovarianceHealth::HEALTHY;
}

} // namespace ov_msckf
