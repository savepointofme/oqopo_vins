#include "core/OnlineAlignmentCandidateFilter.h"

#include <Eigen/Eigenvalues>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

namespace {

using ov_msckf::CandidateCovariance;
using ov_msckf::CandidateCovarianceHealth;
using ov_msckf::CandidateErrorState;
using ov_msckf::CandidateFeedbackTier;
using ov_msckf::CandidateFilterConfig;
using ov_msckf::CandidateFilterRuntime;
using ov_msckf::CandidateGroupGate;
using ov_msckf::CandidateImuStep;
using ov_msckf::CandidateMeasurementStatus;
using ov_msckf::CandidateNominalState;
using ov_msckf::CandidatePositionVelocityMeasurement;
using ov_msckf::CandidateStateGroup;
using ov_msckf::OnlineAlignmentCandidateFilter;

constexpr std::size_t kGroupCount =
    static_cast<std::size_t>(CandidateStateGroup::COUNT);

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

std::size_t groupIndex(CandidateStateGroup group) {
  return static_cast<std::size_t>(group);
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(),
      0.0;
  return result;
}

Eigen::Matrix3d expSO3(const Eigen::Vector3d &rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-14)
    return Eigen::Matrix3d::Identity() + skew(rotation_vector);
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

std::array<bool, kGroupCount> allGraphSupported() {
  std::array<bool, kGroupCount> support{};
  support.fill(true);
  return support;
}

CandidateCovariance diagonalCovariance(double value = 1.0) {
  return value * CandidateCovariance::Identity();
}

CandidateCovariance coupledCovariance() {
  CandidateCovariance P = CandidateCovariance::Identity();
  for (int axis = 0; axis < 3; ++axis) {
    P(axis, 3 + axis) = P(3 + axis, axis) = 0.20;
    P(9 + axis, 3 + axis) = P(3 + axis, 9 + axis) = 0.15;
    P(12 + axis, 6 + axis) = P(6 + axis, 12 + axis) = 0.15;
  }
  return P;
}

CandidateGroupGate permissiveGate(std::size_t history_length = 1,
                                  int min_supported_updates = 1,
                                  int post_feedback_updates = 0) {
  CandidateGroupGate gate;
  gate.configured = true;
  gate.require_initial_graph_support = true;
  gate.history_length_updates = history_length;
  gate.min_supported_updates = min_supported_updates;
  gate.required_post_feedback_stable_updates = post_feedback_updates;
  gate.max_abs_error = Eigen::Vector3d::Constant(1.0e6);
  gate.max_std = Eigen::Vector3d::Constant(1.0e6);
  gate.max_std_step =
      history_length >= 2
          ? Eigen::Vector3d::Constant(1.0e6)
          : Eigen::Vector3d::Constant(
                std::numeric_limits<double>::infinity());
  gate.max_error_peak_to_peak = Eigen::Vector3d::Constant(1.0e6);
  gate.max_std_peak_to_peak = Eigen::Vector3d::Constant(1.0e6);
  gate.max_feedback_step = Eigen::Vector3d::Constant(1.0e6);
  gate.max_cumulative_feedback = Eigen::Vector3d::Constant(1.0e6);
  return gate;
}

void requireFiniteSymmetricPsd(const CandidateCovariance &P,
                               const std::string &context) {
  require(P.allFinite(), context + ": covariance must be finite");
  require((P - P.transpose()).norm() < 1.0e-10,
          context + ": covariance must be symmetric");
  Eigen::SelfAdjointEigenSolver<CandidateCovariance> eig(P);
  require(eig.info() == Eigen::Success,
          context + ": covariance eigensolver must succeed");
  require(eig.eigenvalues().minCoeff() >= -1.0e-10,
          context + ": covariance must be PSD");
}

void requireSameNominal(const CandidateNominalState &lhs,
                        const CandidateNominalState &rhs,
                        const std::string &context) {
  require((lhs.R_GtoI - rhs.R_GtoI).norm() < 1.0e-14,
          context + ": attitude changed");
  require((lhs.p_IinG - rhs.p_IinG).norm() < 1.0e-14,
          context + ": position changed");
  require((lhs.v_IinG - rhs.v_IinG).norm() < 1.0e-14,
          context + ": velocity changed");
  require((lhs.bg - rhs.bg).norm() < 1.0e-14,
          context + ": gyro bias changed");
  require((lhs.ba - rhs.ba).norm() < 1.0e-14,
          context + ": accelerometer bias changed");
}

void testAttitudeInjectionSignsAndCovarianceReset() {
  for (const double sign : {-1.0, 1.0}) {
    CandidateFilterConfig config;
    OnlineAlignmentCandidateFilter filter(config);
    CandidateNominalState nominal;
    CandidateErrorState dx = CandidateErrorState::Zero();
    const Eigen::Vector3d correction =
        sign * Eigen::Vector3d(0.021, -0.014, 0.009);
    dx.segment<3>(0) = correction;
    dx.segment<3>(3) = Eigen::Vector3d(0.3, -0.2, 0.1);

    CandidateCovariance P = 0.2 * CandidateCovariance::Identity();
    P.block<3, 3>(0, 3) = 0.01 * Eigen::Matrix3d::Identity();
    P.block<3, 3>(3, 0) = P.block<3, 3>(0, 3).transpose();
    const CandidateCovariance P_before = P;

    std::string reason;
    require(filter.initialize(4.0, nominal, P, allGraphSupported(), dx,
                              &reason),
            "attitude reset initialization: " + reason);
    const auto feedback = filter.feedbackGroup(CandidateStateGroup::ATTITUDE);
    require(feedback.applied, "explicit attitude feedback must be applied");
    require((feedback.applied_correction - correction).norm() < 1.0e-14,
            "attitude feedback must preserve correction sign");

    const CandidateFilterRuntime &runtime = filter.runtime();
    const Eigen::Matrix3d expected_rotation = expSO3(-correction);
    require((runtime.nominal.R_GtoI - expected_rotation).norm() < 1.0e-12,
            "passive-JPL attitude injection has the wrong sign");
    require(runtime.dx.segment<3>(0).norm() < 1.0e-13,
            "applied attitude error must be removed from persistent dx");
    require((runtime.dx.segment<3>(3) - dx.segment<3>(3)).norm() < 1.0e-13,
            "attitude reset must not erase additive position error");

    CandidateCovariance G = CandidateCovariance::Identity();
    G.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() - 0.5 * skew(correction);
    const CandidateCovariance expected_P = G * P_before * G.transpose();
    require((runtime.P - expected_P).norm() < 1.0e-11,
            "attitude feedback must apply the full covariance reset, including cross-covariance");
    requireFiniteSymmetricPsd(runtime.P, "attitude covariance reset");
  }
}

void testFullGainEstimationIsIndependentFromFeedbackPermission() {
  CandidateFilterConfig config;
  config.position_sigma_m = 1.0;
  config.nis_gate_3d = 1.0e9;
  // q/bg/ba gates intentionally remain unconfigured: they may be estimated
  // through cross-covariance, but they are not permitted to feed back.
  OnlineAlignmentCandidateFilter filter(config);

  CandidateCovariance P = CandidateCovariance::Identity();
  P(0, 3) = P(3, 0) = 0.40;
  P(9, 3) = P(3, 9) = 0.30;
  P(12, 3) = P(3, 12) = -0.20;
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), P,
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "full-gain initialization: " + reason);

  CandidatePositionVelocityMeasurement measurement;
  measurement.source_timestamp = 10.0;
  measurement.board_time = 0.0;
  measurement.position_G = Eigen::Vector3d(1.0, 0.0, 0.0);
  measurement.velocity_valid = false;
  measurement.position_sigma_m = 1.0;
  const auto update = filter.updatePositionVelocity(measurement);
  require(update.committed &&
              update.position_status == CandidateMeasurementStatus::ACCEPTED,
          "position event must be accepted for full-gain test: " +
              update.reason);

  const CandidateFilterRuntime &runtime = filter.runtime();
  require(std::fabs(runtime.dx(0)) > 1.0e-4,
          "attitude error must accumulate through the full Kalman gain");
  require(std::fabs(runtime.dx(9)) > 1.0e-4,
          "gyro-bias error must accumulate through the full Kalman gain");
  require(std::fabs(runtime.dx(12)) > 1.0e-4,
          "accelerometer-bias error must accumulate through the full Kalman gain");
  require((runtime.nominal.R_GtoI - Eigen::Matrix3d::Identity()).norm() <
              1.0e-14,
          "unconfigured attitude gate must prevent attitude feedback");
  require(runtime.nominal.bg.norm() < 1.0e-14,
          "unconfigured gyro-bias gate must prevent bias feedback");
  require(runtime.nominal.ba.norm() < 1.0e-14,
          "unconfigured accelerometer-bias gate must prevent bias feedback");
  require(!update.feedback[groupIndex(CandidateStateGroup::ATTITUDE)].applied &&
              !update.feedback[groupIndex(CandidateStateGroup::GYRO_BIAS)]
                   .applied &&
              !update.feedback[groupIndex(CandidateStateGroup::ACCEL_BIAS)]
                   .applied,
          "feedback permission must not be confused with Kalman estimation");
}

void testPersistentErrorIsSubtractedFromInnovation() {
  CandidateFilterConfig config;
  config.position_sigma_m = 1.0;
  config.nis_gate_3d = 1.0e9;
  config.group_gates[groupIndex(CandidateStateGroup::POSITION)] =
      permissiveGate();
  config.group_gates[groupIndex(CandidateStateGroup::POSITION)]
      .max_feedback_step = Eigen::Vector3d::Constant(0.10);
  config.group_gates[groupIndex(CandidateStateGroup::POSITION)]
      .max_cumulative_feedback = Eigen::Vector3d::Constant(10.0);

  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), diagonalCovariance(),
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "persistent-dx initialization: " + reason);

  CandidatePositionVelocityMeasurement first;
  first.source_timestamp = 1.0;
  first.board_time = 0.0;
  first.position_G = Eigen::Vector3d(1.0, 0.0, 0.0);
  first.velocity_valid = false;
  first.position_sigma_m = 1.0;
  const auto first_result = filter.updatePositionVelocity(first);
  require(first_result.committed &&
              first_result.position_status == CandidateMeasurementStatus::ACCEPTED,
          "first persistent-dx measurement must be accepted");
  require(first_result.feedback[groupIndex(CandidateStateGroup::POSITION)]
              .applied,
          "position update must enter the standard feedback path");
  require(first_result.feedback[groupIndex(CandidateStateGroup::POSITION)]
              .clipped,
          "test requires a deliberately retained position error");

  const double represented_position_before =
      filter.runtime().nominal.p_IinG.x() + filter.runtime().dx(3);
  require(std::fabs(filter.runtime().dx(3)) > 1.0e-6,
          "clipped feedback must retain unapplied error in dx");

  CandidatePositionVelocityMeasurement second = first;
  second.source_timestamp = 2.0;
  second.board_time = 0.0;
  second.position_G.x() = represented_position_before;
  const auto second_result = filter.updatePositionVelocity(second);
  require(second_result.committed &&
              second_result.position_status == CandidateMeasurementStatus::ACCEPTED,
          "second persistent-dx measurement must be accepted");
  const double represented_position_after =
      filter.runtime().nominal.p_IinG.x() + filter.runtime().dx(3);
  require(std::fabs(represented_position_after - represented_position_before) <
              1.0e-11,
          "innovation must use z-h(nominal)-H*dx and not estimate retained error twice");
}

void testFeedbackReducesRepresentedMeasurementResidual() {
  CandidateFilterConfig config;
  config.position_sigma_m = 1.0;
  config.nis_gate_3d = 1.0e9;
  config.group_gates[groupIndex(CandidateStateGroup::POSITION)] =
      permissiveGate();
  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(),
                            diagonalCovariance(), allGraphSupported(),
                            CandidateErrorState::Zero(), &reason),
          "residual feedback initialization: " + reason);

  CandidatePositionVelocityMeasurement measurement;
  measurement.source_timestamp = 1.0;
  measurement.board_time = 0.0;
  measurement.position_G = Eigen::Vector3d(1.0, 0.0, 0.0);
  measurement.velocity_valid = false;
  measurement.position_sigma_m = 1.0;
  const double residual_before =
      (measurement.position_G - filter.runtime().nominal.p_IinG -
       filter.runtime().dx.segment<3>(3))
          .norm();
  const auto result = filter.updatePositionVelocity(measurement);
  require(result.committed &&
              result.position_status == CandidateMeasurementStatus::ACCEPTED,
          "residual feedback measurement must be accepted");
  require(result.feedback[groupIndex(CandidateStateGroup::POSITION)].applied,
          "position correction must be applied before residual comparison");
  const double residual_after =
      (measurement.position_G - filter.runtime().nominal.p_IinG -
       filter.runtime().dx.segment<3>(3))
          .norm();
  require(residual_after < residual_before,
          "feedback must reduce the represented measurement residual");
}

void testNisGateRejectsMeasurementWithoutStateMutation() {
  CandidateFilterConfig config;
  config.position_sigma_m = 0.1;
  config.nis_gate_3d = 11.345;
  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(),
                            diagonalCovariance(), allGraphSupported(),
                            CandidateErrorState::Zero(), &reason),
          "NIS rejection initialization: " + reason);

  const CandidateFilterRuntime before = filter.runtime();
  CandidatePositionVelocityMeasurement measurement;
  measurement.source_timestamp = 1.0;
  measurement.board_time = 0.0;
  measurement.position_G = Eigen::Vector3d(10.0, -8.0, 6.0);
  measurement.velocity_valid = false;
  measurement.position_sigma_m = 0.1;
  const auto result = filter.updatePositionVelocity(measurement);

  require(result.committed &&
              result.position_status == CandidateMeasurementStatus::NIS_REJECTED,
          "large FC position innovation must be NIS rejected");
  require(result.position_nis > config.nis_gate_3d,
          "rejected position event must expose the NIS that crossed the gate");
  require(std::isfinite(filter.runtime().max_position_nis) &&
              filter.runtime().max_position_nis == result.position_nis,
          "NIS diagnostics must retain the maximum position NIS");
  require(filter.runtime().accepted_position_count == 0 &&
              filter.runtime().rejected_position_count == 1 &&
              filter.runtime().accepted_fc_event_count == 0 &&
              filter.runtime().rejected_fc_event_count == 1,
          "NIS rejection must remain auditable in accepted/rejected counters");
  requireSameNominal(filter.runtime().nominal, before.nominal,
                     "NIS rejection");
  require((filter.runtime().dx - before.dx).norm() < 1.0e-14 &&
              (filter.runtime().P - before.P).norm() < 1.0e-14,
          "NIS rejection must not mutate the state or covariance");
}

void testDuplicateEventsAreIdempotentAndCursorOwned() {
  CandidateFilterConfig config;
  config.nis_gate_3d = 1.0e9;
  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), diagonalCovariance(),
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "cursor initialization: " + reason);

  CandidateImuStep step;
  step.start_time = 0.0;
  step.end_time = 0.1;
  step.linear_acceleration_start = config.gravity_G;
  step.linear_acceleration_end = config.gravity_G;
  require(filter.propagate(step, &reason), "first IMU step: " + reason);
  const CandidateFilterRuntime after_imu = filter.runtime();
  (void)filter.propagate(step, &reason);
  const CandidateFilterRuntime &after_duplicate_imu = filter.runtime();
  require(after_duplicate_imu.propagated_imu_step_count ==
              after_imu.propagated_imu_step_count &&
              after_duplicate_imu.duplicate_imu_step_count ==
                  after_imu.duplicate_imu_step_count + 1,
          "duplicate IMU step must not advance the external cursor");
  requireSameNominal(after_duplicate_imu.nominal, after_imu.nominal,
                     "duplicate IMU step");
  require((after_duplicate_imu.dx - after_imu.dx).norm() < 1.0e-14 &&
              (after_duplicate_imu.P - after_imu.P).norm() < 1.0e-14,
          "duplicate IMU step must be a state/covariance no-op");

  CandidatePositionVelocityMeasurement measurement;
  measurement.source_timestamp = 7.0;
  measurement.board_time = 0.1;
  measurement.position_G = filter.runtime().nominal.p_IinG;
  measurement.velocity_G = filter.runtime().nominal.v_IinG;
  const auto accepted = filter.updatePositionVelocity(measurement);
  require(accepted.committed && !accepted.duplicate,
          "first FC snapshot must be consumed atomically");
  const CandidateFilterRuntime after_fc = filter.runtime();
  const auto duplicate = filter.updatePositionVelocity(measurement);
  require(duplicate.duplicate && !duplicate.committed,
          "same source timestamp must be identified as a duplicate snapshot");
  const CandidateFilterRuntime &after_duplicate_fc = filter.runtime();
  require(after_duplicate_fc.last_fc_source_timestamp ==
                  after_fc.last_fc_source_timestamp &&
              after_duplicate_fc.consumed_fc_event_count ==
                  after_fc.consumed_fc_event_count &&
              after_duplicate_fc.duplicate_fc_event_count ==
                  after_fc.duplicate_fc_event_count + 1,
          "duplicate FC event must not advance the consumed cursor");
  requireSameNominal(after_duplicate_fc.nominal, after_fc.nominal,
                     "duplicate FC event");
  require((after_duplicate_fc.dx - after_fc.dx).norm() < 1.0e-14 &&
              (after_duplicate_fc.P - after_fc.P).norm() < 1.0e-14,
          "duplicate FC event must preserve the committed atomic snapshot");
}

void testBiasFeedbackChangesNextImuPropagation() {
  const auto run_pair = [](CandidateStateGroup group,
                           const Eigen::Vector3d &bias_error) {
    CandidateFilterConfig config;
    CandidateErrorState dx = CandidateErrorState::Zero();
    if (group == CandidateStateGroup::GYRO_BIAS)
      dx.segment<3>(9) = bias_error;
    else
      dx.segment<3>(12) = bias_error;

    OnlineAlignmentCandidateFilter feedback_filter(config);
    OnlineAlignmentCandidateFilter retained_filter(config);
    std::string reason;
    require(feedback_filter.initialize(0.0, CandidateNominalState(),
                                       diagonalCovariance(),
                                       allGraphSupported(), dx, &reason),
            "bias feedback initialization: " + reason);
    require(retained_filter.initialize(0.0, CandidateNominalState(),
                                       diagonalCovariance(),
                                       allGraphSupported(), dx, &reason),
            "retained bias initialization: " + reason);
    const auto feedback = feedback_filter.feedbackGroup(group);
    require(feedback.applied, "explicit bias feedback must be applied");

    CandidateImuStep step;
    step.start_time = 0.0;
    step.end_time = 1.0;
    step.angular_velocity_start = Eigen::Vector3d(0.0, 0.0, 0.4);
    step.angular_velocity_end = step.angular_velocity_start;
    step.linear_acceleration_start =
        config.gravity_G + Eigen::Vector3d(1.0, 0.0, 0.0);
    step.linear_acceleration_end = step.linear_acceleration_start;
    require(feedback_filter.propagate(step, &reason),
            "feedback bias propagation: " + reason);
    require(retained_filter.propagate(step, &reason),
            "retained bias propagation: " + reason);

    if (group == CandidateStateGroup::GYRO_BIAS) {
      require((feedback_filter.runtime().nominal.bg - bias_error).norm() <
                  1.0e-14,
              "gyro-bias feedback must change nominal compensation");
      require((feedback_filter.runtime().nominal.R_GtoI -
               retained_filter.runtime().nominal.R_GtoI)
                      .norm() >
                  1.0e-3,
              "gyro-bias feedback must change the next attitude propagation");
    } else {
      require((feedback_filter.runtime().nominal.ba - bias_error).norm() <
                  1.0e-14,
              "accelerometer-bias feedback must change nominal compensation");
      require((feedback_filter.runtime().nominal.v_IinG -
               retained_filter.runtime().nominal.v_IinG)
                      .norm() >
                  1.0e-3,
              "accelerometer-bias feedback must change the next velocity propagation");
    }
  };

  run_pair(CandidateStateGroup::GYRO_BIAS,
           Eigen::Vector3d(0.0, 0.0, 0.08));
  run_pair(CandidateStateGroup::ACCEL_BIAS,
           Eigen::Vector3d(0.25, 0.0, 0.0));
}

CandidatePositionVelocityMeasurement makeCoupledMeasurement(int serial) {
  CandidatePositionVelocityMeasurement measurement;
  measurement.source_timestamp = static_cast<double>(serial);
  measurement.board_time = 0.0;
  measurement.position_G =
      Eigen::Vector3d(0.20 * serial, -0.10 * serial, 0.05 * serial);
  measurement.velocity_G =
      Eigen::Vector3d(0.04 * serial, 0.03 * serial, -0.02 * serial);
  measurement.position_sigma_m = 1.0;
  measurement.velocity_sigma_mps = 1.0;
  return measurement;
}

void testGroupHistoryIsRetainedAndPostFeedbackValidated() {
  CandidateFilterConfig config;
  config.nis_gate_3d = 1.0e9;
  config.group_gates[groupIndex(CandidateStateGroup::ATTITUDE)] =
      permissiveGate(2, 2, 2);

  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), coupledCovariance(),
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "group-history initialization: " + reason);

  const auto first = filter.updatePositionVelocity(makeCoupledMeasurement(1));
  require(first.committed, "first convergence update must commit");
  const auto &group_after_first =
      filter.runtime().groups[groupIndex(CandidateStateGroup::ATTITUDE)];
  require(group_after_first.supported_update_count == 1 &&
              group_after_first.feedback_count == 0 &&
              group_after_first.history.size() == 1,
          "attitude gate must wait for its configured history/support length");

  const auto second = filter.updatePositionVelocity(makeCoupledMeasurement(2));
  require(second.committed, "second convergence update must commit");
  const auto &group_after_feedback =
      filter.runtime().groups[groupIndex(CandidateStateGroup::ATTITUDE)];
  require(group_after_feedback.feedback_count == 1,
          "attitude group must feed back after the first stable gate window");
  require(group_after_feedback.history.size() == 2 &&
              group_after_feedback.supported_update_count == 2,
          "feedback must retain rolling history and cumulative support count");
  require(!group_after_feedback.converged,
          "feedback must clear only the convergence latch");

  const auto third = filter.updatePositionVelocity(makeCoupledMeasurement(3));
  require(third.committed, "third convergence update must commit");
  require(filter.runtime()
              .groups[groupIndex(CandidateStateGroup::ATTITUDE)]
              .feedback_count == 1,
          "one post-feedback update must not bypass a two-update rearm gate");

  const auto fourth = filter.updatePositionVelocity(makeCoupledMeasurement(4));
  require(fourth.committed, "fourth convergence update must commit");
  const auto &group_after_hold =
      filter.runtime().groups[groupIndex(CandidateStateGroup::ATTITUDE)];
  require(group_after_hold.feedback_count == 1,
          "the update completing the hold must not inject a second unvalidated correction");
  require(group_after_hold.supported_update_count == 4 &&
              group_after_hold.history.size() == 2 &&
              filter.groupReady(CandidateStateGroup::ATTITUDE),
          "post-feedback validation must preserve cumulative counts and establish readiness");

  CandidatePositionVelocityMeasurement rejected = makeCoupledMeasurement(5);
  rejected.position_valid = false;
  rejected.velocity_valid = false;
  const auto rejected_result = filter.updatePositionVelocity(rejected);
  require(rejected_result.committed,
          "an invalid measurement event must still advance the causal cursor");
  const auto &group_after_reject =
      filter.runtime().groups[groupIndex(CandidateStateGroup::ATTITUDE)];
  require(group_after_reject.history.empty() &&
              !filter.groupReady(CandidateStateGroup::ATTITUDE),
          "rejected or unsupported updates must break the consecutive convergence window");
}

void testBiasFeedbackLeavesPostValidationBudget() {
  CandidateFilterConfig config;
  config.nis_gate_3d = 1.0e9;
  config.group_gates[groupIndex(CandidateStateGroup::GYRO_BIAS)] =
      permissiveGate(8, 8, 2);
  config.group_gates[groupIndex(CandidateStateGroup::ACCEL_BIAS)] =
      permissiveGate(8, 8, 2);

  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), coupledCovariance(),
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "bias post-validation budget initialization: " + reason);
  for (int serial = 1; serial <= 10; ++serial) {
    const auto update =
        filter.updatePositionVelocity(makeCoupledMeasurement(serial));
    require(update.committed &&
                update.position_status == CandidateMeasurementStatus::ACCEPTED &&
                update.velocity_status == CandidateMeasurementStatus::ACCEPTED,
            "bias post-validation budget measurement must be accepted");
  }
  const auto &runtime = filter.runtime();
  require(runtime.groups[groupIndex(CandidateStateGroup::GYRO_BIAS)]
                  .feedback_count > 0 &&
              runtime.groups[groupIndex(CandidateStateGroup::ACCEL_BIAS)]
                  .feedback_count > 0,
          "bias groups must receive feedback before the final two events");
  require(runtime.groups[groupIndex(CandidateStateGroup::GYRO_BIAS)]
                  .post_first_feedback_stable_updates >= 2 &&
              runtime.groups[groupIndex(CandidateStateGroup::ACCEL_BIAS)]
                  .post_first_feedback_stable_updates >= 2,
          "bias groups must retain two post-feedback validation events");
  require(filter.groupReady(CandidateStateGroup::GYRO_BIAS) &&
              filter.groupReady(CandidateStateGroup::ACCEL_BIAS),
          "bias groups must become ready when the causal budget is sufficient");
}

CandidateFilterConfig tierConfig(bool attitude, bool gyro_bias,
                                 bool accel_bias) {
  CandidateFilterConfig config;
  config.nis_gate_3d = 1.0e9;
  config.position_sigma_m = 1.0;
  config.velocity_sigma_mps = 1.0;
  config.group_gates[groupIndex(CandidateStateGroup::POSITION)] =
      permissiveGate();
  config.group_gates[groupIndex(CandidateStateGroup::VELOCITY)] =
      permissiveGate();
  if (attitude)
    config.group_gates[groupIndex(CandidateStateGroup::ATTITUDE)] =
        permissiveGate(1, 1, 1);
  if (gyro_bias)
    config.group_gates[groupIndex(CandidateStateGroup::GYRO_BIAS)] =
        permissiveGate(1, 1, 1);
  if (accel_bias)
    config.group_gates[groupIndex(CandidateStateGroup::ACCEL_BIAS)] =
        permissiveGate(1, 1, 1);
  return config;
}

OnlineAlignmentCandidateFilter makeTierFilter(bool attitude, bool gyro_bias,
                                               bool accel_bias) {
  OnlineAlignmentCandidateFilter filter(
      tierConfig(attitude, gyro_bias, accel_bias));
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), coupledCovariance(),
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "tier initialization: " + reason);
  const auto update = filter.updatePositionVelocity(makeCoupledMeasurement(1));
  require(update.committed &&
              update.position_status == CandidateMeasurementStatus::ACCEPTED &&
              update.velocity_status == CandidateMeasurementStatus::ACCEPTED,
          "tier measurement must be accepted: " + update.reason);
  const auto validation =
      filter.updatePositionVelocity(makeCoupledMeasurement(2));
  require(validation.committed &&
              validation.position_status ==
                  CandidateMeasurementStatus::ACCEPTED &&
              validation.velocity_status ==
                  CandidateMeasurementStatus::ACCEPTED,
          "tier post-feedback measurement must be accepted: " +
              validation.reason);
  return filter;
}

void testReleaseTiersCannotPromotePvOnly() {
  OnlineAlignmentCandidateFilter pv_only =
      makeTierFilter(false, false, false);
  require(pv_only.feedbackTier() == CandidateFeedbackTier::PV_ONLY,
          "p/v feedback alone must remain PV_ONLY");
  require(!pv_only.navigationReady() && !pv_only.fullAlignmentReady(),
          "PV_ONLY must never produce a normal navigation release");

  OnlineAlignmentCandidateFilter gyro_partial =
      makeTierFilter(true, true, false);
  require(gyro_partial.feedbackTier() ==
              CandidateFeedbackTier::ATTITUDE_GYRO_BIAS,
          "trusted attitude+bg with retained ba must report gyro-bias partial tier");
  require(gyro_partial.navigationReady(),
          "q/p/v closed loop may produce practical navigation readiness");
  require(!gyro_partial.fullAlignmentReady(),
          "bias-partial tier must not claim full alignment readiness");

  OnlineAlignmentCandidateFilter accel_partial =
      makeTierFilter(true, false, true);
  require(accel_partial.feedbackTier() ==
              CandidateFeedbackTier::ATTITUDE_ACCEL_BIAS,
          "trusted attitude+ba with retained bg must report accel-bias partial tier");
  require(accel_partial.navigationReady() &&
              !accel_partial.fullAlignmentReady(),
          "accelerometer-bias partial tier must remain practical-only");

  OnlineAlignmentCandidateFilter full = makeTierFilter(true, true, true);
  require(full.feedbackTier() == CandidateFeedbackTier::FULL,
          "all five trusted groups must report FULL tier");
  require(full.navigationReady() && full.fullAlignmentReady(),
          "FULL tier must satisfy both readiness levels");
}

void testRetainedBiasReleaseCovarianceIsConservative() {
  CandidateFilterConfig config;
  OnlineAlignmentCandidateFilter filter(config);
  CandidateErrorState dx = CandidateErrorState::Zero();
  dx.segment<3>(9) = Eigen::Vector3d(0.02, -0.01, 0.03);
  dx.segment<3>(12) = Eigen::Vector3d(0.30, -0.20, 0.10);
  CandidateCovariance initial_P = 0.4 * CandidateCovariance::Identity();
  initial_P.block<3, 3>(0, 9) = 0.02 * Eigen::Matrix3d::Identity();
  initial_P.block<3, 3>(9, 0) = initial_P.block<3, 3>(0, 9).transpose();
  initial_P.block<3, 3>(3, 12) = 0.03 * Eigen::Matrix3d::Identity();
  initial_P.block<3, 3>(12, 3) = initial_P.block<3, 3>(3, 12).transpose();
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), initial_P,
                            allGraphSupported(), dx, &reason),
          "release covariance initialization: " + reason);
  require(filter.feedbackGroup(CandidateStateGroup::GYRO_BIAS).applied &&
              filter.feedbackGroup(CandidateStateGroup::ACCEL_BIAS).applied,
          "retained-prior test must include biases that were previously fed back");

  CandidateCovarianceHealth health = CandidateCovarianceHealth::NON_FINITE;
  const CandidateCovariance release =
      filter.releaseCovariance(false, false, &health);
  const Eigen::Matrix3d expected_bg =
      initial_P.block<3, 3>(9, 9) +
      dx.segment<3>(9) * dx.segment<3>(9).transpose();
  const Eigen::Matrix3d expected_ba =
      initial_P.block<3, 3>(12, 12) +
      dx.segment<3>(12) * dx.segment<3>(12).transpose();
  require((release.block<3, 3>(9, 9) - expected_bg).norm() < 1.0e-12 &&
              (release.block<3, 3>(12, 12) - expected_ba).norm() <
                  1.0e-12,
          "retained bias covariance must include initial uncertainty plus dx*dx' MSE");
  require(release.block<9, 3>(0, 9).norm() < 1.0e-14 &&
              release.block<3, 9>(9, 0).norm() < 1.0e-14 &&
              release.block<9, 3>(0, 12).norm() < 1.0e-14 &&
              release.block<3, 9>(12, 0).norm() < 1.0e-14,
          "retained biases must be decorrelated from released navigation states");
  require(health == CandidateCovarianceHealth::HEALTHY ||
              health == CandidateCovarianceHealth::ROUNDOFF_REPAIRED,
          "conservative release covariance must remain numerically healthy");
  requireFiniteSymmetricPsd(release, "retained-bias release covariance");
}

void testCovarianceHealthHasNoFixedEigenvalueFloor() {
  CandidateFilterConfig config;
  OnlineAlignmentCandidateFilter filter(config);
  CandidateCovariance singular = CandidateCovariance::Identity();
  singular(14, 14) = 0.0;
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(), singular,
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "singular PSD covariance must be accepted without a floor: " + reason);
  require(std::fabs(filter.runtime().P(14, 14)) < 1.0e-15,
          "valid zero covariance eigenvalue must not be raised to a fixed floor");
  require(filter.runtime().covariance_health ==
              CandidateCovarianceHealth::HEALTHY,
          "exact PSD covariance should be healthy");

  CandidateCovariance roundoff = CandidateCovariance::Identity();
  roundoff(14, 14) = -5.0e-12;
  require(filter.initialize(0.0, CandidateNominalState(), roundoff,
                            allGraphSupported(), CandidateErrorState::Zero(),
                            &reason),
          "roundoff-scale negative eigenvalue should be repairable: " + reason);
  require(filter.runtime().covariance_health ==
              CandidateCovarianceHealth::ROUNDOFF_REPAIRED,
          "roundoff repair must be reported explicitly");
  require(std::fabs(filter.runtime().P(14, 14)) < 1.0e-13,
          "roundoff repair must project to zero, not a fixed positive floor");
  requireFiniteSymmetricPsd(filter.runtime().P, "roundoff covariance repair");

  CandidateCovariance invalid = CandidateCovariance::Identity();
  invalid(14, 14) = -1.0e-3;
  require(!filter.initialize(0.0, CandidateNominalState(), invalid,
                             allGraphSupported(), CandidateErrorState::Zero(),
                             &reason),
          "material negative eigenvalue must fail rather than be floored");

  CandidateCovariance tiny_scale =
      1.0e-12 * CandidateCovariance::Identity();
  tiny_scale(14, 14) = -5.0e-13;
  require(!filter.initialize(0.0, CandidateNominalState(), tiny_scale,
                             allGraphSupported(), CandidateErrorState::Zero(),
                             &reason),
          "covariance repair tolerance must scale with P, not a fixed unit matrix");
}

void testInvalidSigmaAndPostFeedbackContractFailClosed() {
  CandidateFilterConfig config;
  OnlineAlignmentCandidateFilter filter(config);
  std::string reason;
  require(filter.initialize(0.0, CandidateNominalState(),
                            diagonalCovariance(), allGraphSupported(),
                            CandidateErrorState::Zero(), &reason),
          "invalid-sigma test initialization: " + reason);

  CandidatePositionVelocityMeasurement zero_sigma;
  zero_sigma.source_timestamp = 1.0;
  zero_sigma.board_time = 0.0;
  zero_sigma.position_sigma_m = 0.0;
  zero_sigma.velocity_valid = false;
  const auto zero_result = filter.updatePositionVelocity(zero_sigma);
  require(zero_result.committed &&
              zero_result.position_status ==
                  CandidateMeasurementStatus::INVALID,
          "zero measurement sigma must be consumed as invalid, not replaced by a default");

  CandidatePositionVelocityMeasurement nan_sigma = zero_sigma;
  nan_sigma.source_timestamp = 2.0;
  nan_sigma.position_sigma_m = std::numeric_limits<double>::quiet_NaN();
  const auto nan_result = filter.updatePositionVelocity(nan_sigma);
  require(nan_result.committed &&
              nan_result.position_status ==
                  CandidateMeasurementStatus::INVALID &&
              filter.runtime().rejected_position_count == 2,
          "non-finite measurement sigma must fail closed and remain auditable");

  CandidateFilterConfig unsafe_config;
  unsafe_config.group_gates[groupIndex(CandidateStateGroup::ATTITUDE)] =
      permissiveGate(1, 1, 0);
  OnlineAlignmentCandidateFilter unsafe_filter(unsafe_config);
  require(!unsafe_filter.initialize(0.0, CandidateNominalState(),
                                    diagonalCovariance(), allGraphSupported(),
                                    CandidateErrorState::Zero(), &reason),
          "attitude feedback must require a post-feedback accepted update");
}

} // namespace

int main() {
  testAttitudeInjectionSignsAndCovarianceReset();
  testFullGainEstimationIsIndependentFromFeedbackPermission();
  testPersistentErrorIsSubtractedFromInnovation();
  testFeedbackReducesRepresentedMeasurementResidual();
  testNisGateRejectsMeasurementWithoutStateMutation();
  testDuplicateEventsAreIdempotentAndCursorOwned();
  testBiasFeedbackChangesNextImuPropagation();
  testGroupHistoryIsRetainedAndPostFeedbackValidated();
  testBiasFeedbackLeavesPostValidationBudget();
  testReleaseTiersCannotPromotePvOnly();
  testRetainedBiasReleaseCovarianceIsConservative();
  testCovarianceHealthHasNoFixedEigenvalueFloor();
  testInvalidSigmaAndPostFeedbackContractFailClosed();
  std::cout << "online alignment candidate filter tests passed" << std::endl;
  return EXIT_SUCCESS;
}
