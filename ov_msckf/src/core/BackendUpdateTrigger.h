/* Accumulated-information trigger between the last backend frame and now. */

#ifndef OV_MSCKF_BACKEND_UPDATE_TRIGGER_H
#define OV_MSCKF_BACKEND_UPDATE_TRIGGER_H

#include "core/VisualCadencePlanner.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace ov_msckf {

struct BackendUpdateTriggerConfig {
  double minimum_compensated_parallax_px = 5.0;
  double maximum_compensated_p95_px = 24.0;
  double minimum_survival_ratio = 0.45;
  int minimum_common_tracks = 20;
  double minimum_information_proxy = 20.0;
  double high_imu_excitation_rad_s = 0.35;
  double maximum_latency_s = 0.50;
};

struct BackendUpdateTriggerInput {
  double timestamp = -1.0;
  double last_backend_timestamp = -1.0;
  VisualMotionMetrics motion_from_last_backend;
  bool initialized = false;
  bool safety_forced = false;
  bool visual_health_bad = false;
  bool long_track_ending = false;
  double imu_excitation_rad_s = 0.0;
};

struct BackendUpdateDecision {
  bool trigger = false;
  bool information_trigger = false;
  bool latency_fallback = false;
  bool safety_trigger = false;
  double accumulated_information = 0.0;
  std::string reason = "not_initialized";
};

class BackendUpdateTrigger {
public:
  explicit BackendUpdateTrigger(
      const BackendUpdateTriggerConfig &config = BackendUpdateTriggerConfig())
      : config_(config) {}

  BackendUpdateDecision evaluate(const BackendUpdateTriggerInput &input) const {
    BackendUpdateDecision out;
    if (!input.initialized)
      return out;
    if (input.last_backend_timestamp < 0.0) {
      out.trigger = true;
      out.safety_trigger = true;
      out.reason = "first_backend_frame";
      return out;
    }
    const double elapsed = input.timestamp - input.last_backend_timestamp;
    if (!(elapsed >= 0.0) || !std::isfinite(elapsed)) {
      out.reason = "timestamp_invalid";
      return out;
    }
    if (input.safety_forced || input.visual_health_bad ||
        input.long_track_ending ||
        input.imu_excitation_rad_s >= config_.high_imu_excitation_rad_s) {
      out.trigger = true;
      out.safety_trigger = true;
      out.reason = input.visual_health_bad
                       ? "visual_health"
                       : input.long_track_ending
                             ? "long_track_ending"
                             : input.imu_excitation_rad_s >=
                                       config_.high_imu_excitation_rad_s
                                   ? "imu_excitation"
                                   : "safety_state";
      return out;
    }
    const auto &motion = input.motion_from_last_backend;
    out.accumulated_information =
        motion.compensated_median_px * std::max(0.0, motion.survival_ratio) *
        std::log1p(static_cast<double>(std::max(0, motion.common_tracks)));
    const bool information_ready =
        motion.valid &&
        motion.common_tracks >= config_.minimum_common_tracks &&
        motion.survival_ratio >= config_.minimum_survival_ratio &&
        motion.compensated_median_px >=
            config_.minimum_compensated_parallax_px;
    const bool information_proxy_ready =
        motion.valid &&
        out.accumulated_information >= config_.minimum_information_proxy;
    const bool feature_loss_risk =
        motion.valid &&
        (motion.common_tracks < config_.minimum_common_tracks ||
         motion.survival_ratio < config_.minimum_survival_ratio);
    const bool motion_limit =
        motion.valid &&
        motion.compensated_p95_px >= config_.maximum_compensated_p95_px;
    if (information_ready || information_proxy_ready || motion_limit ||
        feature_loss_risk) {
      out.trigger = true;
      out.information_trigger = true;
      out.reason = motion_limit
                       ? "motion_limit"
                       : feature_loss_risk
                             ? "feature_loss_risk"
                             : information_ready ? "accumulated_parallax"
                                                 : "visual_information_proxy";
      return out;
    }
    if (elapsed >= config_.maximum_latency_s) {
      out.trigger = true;
      out.latency_fallback = true;
      out.reason = "maximum_latency_fallback";
      return out;
    }
    out.reason = "information_accumulating";
    return out;
  }

private:
  BackendUpdateTriggerConfig config_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_BACKEND_UPDATE_TRIGGER_H
