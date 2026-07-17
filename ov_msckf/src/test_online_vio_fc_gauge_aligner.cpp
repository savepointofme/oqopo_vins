#include "core/OnlineVioFcGaugeAligner.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

ov_msckf::OnlineVioFcGaugeObservation make_observation(
    double timestamp, double yaw_rad, const Eigen::Vector3d &translation,
    double similarity_scale = 1.0) {
  ov_msckf::OnlineVioFcGaugeObservation observation;
  observation.timestamp = timestamp;
  observation.yaw_rad = yaw_rad;
  observation.yaw_sigma_deg = 0.2;
  observation.local_position = Eigen::Vector3d(
      3.0 * timestamp, std::sin(0.2 * timestamp), 0.1 * timestamp);
  observation.local_velocity =
      Eigen::Vector3d(15.0, 1.5 * std::cos(0.2 * timestamp), 0.5);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  observation.target_position =
      similarity_scale * rotation * observation.local_position + translation;
  observation.target_velocity =
      similarity_scale * rotation * observation.local_velocity;
  return observation;
}

ov_msckf::OnlineVioFcGaugeConfig make_config() {
  ov_msckf::OnlineVioFcGaugeConfig config;
  config.window_duration_s = 4.0;
  config.maximum_observation_gap_s = 0.6;
  config.maximum_yaw_mad_deg = 1.0;
  return config;
}

void feed_window(ov_msckf::OnlineVioFcGaugeAligner &aligner,
                 double yaw_rad, const Eigen::Vector3d &translation,
                 double similarity_scale = 1.0) {
  for (int index = 0; index <= 8; ++index) {
    require(aligner.push(make_observation(
                0.5 * index, yaw_rad, translation, similarity_scale)),
            "valid observation rejected");
  }
}

void test_time_window_and_transform() {
  ov_msckf::OnlineVioFcGaugeAligner aligner(make_config());
  const double yaw = 12.0 * kPi / 180.0;
  const Eigen::Vector3d translation(5.0, -3.0, 2.0);
  for (int index = 0; index < 8; ++index) {
    auto observation = make_observation(0.5 * index, yaw, translation);
    observation.yaw_rad = yaw;
    require(aligner.push(observation),
            "valid observation rejected before full duration");
  }
  require(!aligner.estimate().ready,
          "window released before configured sensor-time duration");
  auto terminal = make_observation(4.0, yaw, translation);
  terminal.yaw_rad = yaw;
  require(aligner.push(terminal),
          "terminal observation rejected");
  const auto estimate = aligner.estimate();
  require(estimate.ready, "complete metric window did not release");
  require(std::fabs(estimate.yaw_rad - yaw) < 1.0e-9,
          "attitude-gauge yaw estimate is incorrect");
  require(estimate.translation_G.isApprox(translation, 1.0e-9),
          "translation estimate is incorrect");
  require(std::fabs(estimate.observed_metric_scale - 1.0) < 1.0e-9,
          "metric scale health estimate is incorrect");
}

void test_circular_yaw_and_reverse_turn() {
  ov_msckf::OnlineVioFcGaugeAligner aligner(make_config());
  const Eigen::Vector3d translation(1.0, 2.0, -1.0);
  for (int index = 0; index <= 8; ++index) {
    const double yaw_deg = index % 2 == 0 ? 179.8 : -179.8;
    require(aligner.push(make_observation(
                0.5 * index, yaw_deg * kPi / 180.0, translation)),
            "circular yaw observation rejected");
  }
  const auto circular = aligner.estimate();
  require(circular.ready, "circular yaw window was not accepted");
  require(std::fabs(std::fabs(circular.yaw_rad) - kPi) < 0.01,
          "circular yaw median crossed the wrap boundary incorrectly");

  aligner.reset();
  feed_window(aligner, -14.0 * kPi / 180.0, translation);
  const auto reverse = aligner.estimate();
  require(reverse.ready && reverse.yaw_rad < 0.0,
          "opposite turn/yaw sign was not handled symmetrically");
}

void test_duplicate_gap_and_scale_deferral() {
  ov_msckf::OnlineVioFcGaugeAligner aligner(make_config());
  const Eigen::Vector3d translation(0.5, -0.5, 0.0);
  require(aligner.push(make_observation(0.0, 0.1, translation)),
          "first observation rejected");
  std::string reason;
  require(!aligner.push(make_observation(0.0, 0.1, translation), &reason) &&
              reason == "observation_non_monotonic_or_duplicate",
          "duplicate timestamp was not rejected");
  require(aligner.push(make_observation(2.0, 0.1, translation)),
          "post-gap observation rejected");
  require(aligner.observations().size() == 1,
          "time discontinuity did not reset the window");

  aligner.reset();
  feed_window(aligner, 0.1, translation, 1.25);
  const auto estimate = aligner.estimate();
  require(estimate.ready,
          "scale ambiguity incorrectly blocked yaw/translation release");
  require(std::fabs(estimate.observed_metric_scale - 1.0) < 1.0e-12,
          "P4 inferred scale instead of deferring it to AGL");
  const auto terminal = aligner.observations().back();
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(estimate.yaw_rad, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  require((rotation * terminal.local_position + estimate.translation_G)
              .isApprox(terminal.target_position, 1.0e-9),
          "terminal translation anchor is incorrect");
  require(estimate.translation_rmse_m > 0.1 &&
              estimate.unit_scale_velocity_rmse_mps > 1.0,
          "deferred scale mismatch was not retained as a diagnostic");
}

} // namespace

int main() {
  test_time_window_and_transform();
  test_circular_yaw_and_reverse_turn();
  test_duplicate_gap_and_scale_deferral();
  std::cout << "online VIO-FC gauge aligner tests passed" << std::endl;
  return EXIT_SUCCESS;
}
