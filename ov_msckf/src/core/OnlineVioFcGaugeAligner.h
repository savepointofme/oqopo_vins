#ifndef OV_MSCKF_ONLINE_VIO_FC_GAUGE_ALIGNER_H
#define OV_MSCKF_ONLINE_VIO_FC_GAUGE_ALIGNER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>
#include <vector>

namespace ov_msckf {

struct OnlineVioFcGaugeConfig {
  double window_duration_s = 8.0;
  double maximum_observation_gap_s = 0.75;
  double maximum_yaw_mad_deg = 5.0;
};

struct OnlineVioFcGaugeObservation {
  double timestamp = -1.0;
  double yaw_rad = 0.0;
  double yaw_sigma_deg = std::numeric_limits<double>::infinity();
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_velocity = Eigen::Vector3d::Zero();
};

struct OnlineVioFcGaugeEstimate {
  bool ready = false;
  std::string reason = "no_observations";
  double window_start = -1.0;
  double window_end = -1.0;
  double covered_duration_s = 0.0;
  double maximum_observation_gap_s =
      std::numeric_limits<double>::infinity();
  int observation_count = 0;
  double yaw_rad = 0.0;
  double yaw_mad_deg = std::numeric_limits<double>::infinity();
  Eigen::Vector3d translation_G = Eigen::Vector3d::Zero();
  double translation_rmse_m = std::numeric_limits<double>::infinity();
  double observed_metric_scale = std::numeric_limits<double>::quiet_NaN();
  double observed_metric_scale_sigma =
      std::numeric_limits<double>::infinity();
  double unit_scale_velocity_rmse_mps =
      std::numeric_limits<double>::infinity();
};

/// Robustly estimates the one-time yaw/translation gauge between a local VIO
/// and synchronized FC navigation states. The window is defined only by sensor
/// time. Metric scale is deliberately not inferred from FC horizontal
/// velocity; the independent AGL output-scale stage owns that one-dimensional
/// correction.
class OnlineVioFcGaugeAligner {
public:
  explicit OnlineVioFcGaugeAligner(
      const OnlineVioFcGaugeConfig &config = OnlineVioFcGaugeConfig())
      : config_(config) {
    config_.window_duration_s = std::max(0.1, config_.window_duration_s);
    config_.maximum_observation_gap_s =
        std::max(1.0e-3, config_.maximum_observation_gap_s);
    config_.maximum_yaw_mad_deg =
        std::max(0.0, config_.maximum_yaw_mad_deg);
  }

  void reset() { observations_.clear(); }

  const OnlineVioFcGaugeConfig &config() const { return config_; }

  const std::deque<OnlineVioFcGaugeObservation> &observations() const {
    return observations_;
  }

  bool push(const OnlineVioFcGaugeObservation &observation,
            std::string *reason = nullptr) {
    const bool finite = std::isfinite(observation.timestamp) &&
                        std::isfinite(observation.yaw_rad) &&
                        std::isfinite(observation.yaw_sigma_deg) &&
                        observation.target_position.allFinite() &&
                        observation.target_velocity.allFinite() &&
                        observation.local_position.allFinite() &&
                        observation.local_velocity.allFinite();
    if (!finite) {
      set_reason(reason, "observation_non_finite");
      return false;
    }
    if (!observations_.empty() &&
        observation.timestamp <= observations_.back().timestamp) {
      set_reason(reason, "observation_non_monotonic_or_duplicate");
      return false;
    }
    if (!observations_.empty() &&
        observation.timestamp - observations_.back().timestamp >
            config_.maximum_observation_gap_s) {
      observations_.clear();
    }
    observations_.push_back(observation);
    const double keep_from =
        observation.timestamp - config_.window_duration_s;
    while (observations_.size() > 2 &&
           observations_[1].timestamp <= keep_from)
      observations_.pop_front();
    set_reason(reason, "accepted");
    return true;
  }

  OnlineVioFcGaugeEstimate estimate() const {
    OnlineVioFcGaugeEstimate estimate;
    estimate.observation_count = static_cast<int>(observations_.size());
    if (observations_.size() < 2) {
      estimate.reason = "waiting_for_time_window";
      return estimate;
    }
    estimate.window_start = observations_.front().timestamp;
    estimate.window_end = observations_.back().timestamp;
    estimate.covered_duration_s =
        estimate.window_end - estimate.window_start;
    estimate.maximum_observation_gap_s = 0.0;
    for (std::size_t index = 1; index < observations_.size(); ++index) {
      estimate.maximum_observation_gap_s =
          std::max(estimate.maximum_observation_gap_s,
                   observations_[index].timestamp -
                       observations_[index - 1].timestamp);
    }
    if (estimate.maximum_observation_gap_s >
        config_.maximum_observation_gap_s) {
      estimate.reason = "observation_gap";
      return estimate;
    }
    if (estimate.covered_duration_s + 1.0e-9 <
        config_.window_duration_s) {
      estimate.reason = "waiting_for_time_window";
      return estimate;
    }

    std::vector<double> yaw_observations;
    yaw_observations.reserve(observations_.size());
    for (const auto &observation : observations_)
      yaw_observations.push_back(observation.yaw_rad);
    const double yaw_reference = yaw_observations.back();
    std::vector<double> unwrapped_yaws;
    unwrapped_yaws.reserve(yaw_observations.size());
    for (const double yaw : yaw_observations)
      unwrapped_yaws.push_back(yaw_reference + wrap(yaw - yaw_reference));
    estimate.yaw_rad = wrap(median(unwrapped_yaws));
    std::vector<double> yaw_absolute_residuals;
    yaw_absolute_residuals.reserve(yaw_observations.size());
    for (const double yaw : yaw_observations) {
      yaw_absolute_residuals.push_back(
          std::fabs(wrap(yaw - estimate.yaw_rad)));
    }
    estimate.yaw_mad_deg =
        1.4826 * median(yaw_absolute_residuals) * 180.0 / pi();
    estimate.observed_metric_scale = 1.0;
    estimate.observed_metric_scale_sigma = 0.0;

    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(estimate.yaw_rad, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const auto &terminal = observations_.back();
    estimate.translation_G =
        terminal.target_position - rotation * terminal.local_position;

    double translation_squared_error = 0.0;
    double unit_scale_velocity_squared_error = 0.0;
    for (const auto &observation : observations_) {
      const Eigen::Vector3d position_residual =
          observation.target_position -
          (rotation * observation.local_position + estimate.translation_G);
      translation_squared_error += position_residual.squaredNorm();
      const Eigen::Vector2d rotated_velocity =
          (rotation * observation.local_velocity).head<2>();
      const Eigen::Vector2d target_velocity =
          observation.target_velocity.head<2>();
      unit_scale_velocity_squared_error +=
          (target_velocity - rotated_velocity).squaredNorm();
    }
    const double count = static_cast<double>(observations_.size());
    estimate.translation_rmse_m =
        std::sqrt(translation_squared_error / count);
    estimate.unit_scale_velocity_rmse_mps =
        std::sqrt(unit_scale_velocity_squared_error / count);

    if (!std::isfinite(estimate.yaw_mad_deg) ||
        estimate.yaw_mad_deg > config_.maximum_yaw_mad_deg) {
      estimate.reason = "yaw_window_inconsistent";
      return estimate;
    }
    estimate.ready = true;
    estimate.reason = "time_window_yaw_translation_ready_scale_deferred_to_agl";
    return estimate;
  }

private:
  static double pi() { return 3.14159265358979323846; }

  static double wrap(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static double median(std::vector<double> values) {
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 != 0)
      return upper;
    const double lower =
        *std::max_element(values.begin(), values.begin() + middle);
    return 0.5 * (lower + upper);
  }

  static void set_reason(std::string *reason, const std::string &value) {
    if (reason != nullptr)
      *reason = value;
  }

  OnlineVioFcGaugeConfig config_;
  std::deque<OnlineVioFcGaugeObservation> observations_;
};

} // namespace ov_msckf

#endif
