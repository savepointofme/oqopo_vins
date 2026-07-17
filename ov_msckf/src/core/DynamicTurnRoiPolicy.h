#ifndef OV_MSCKF_DYNAMIC_TURN_ROI_POLICY_H
#define OV_MSCKF_DYNAMIC_TURN_ROI_POLICY_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace ov_msckf {

enum class TurnDirection { RIGHT = -1, STRAIGHT = 0, LEFT = 1 };
enum class PreferredImageSide { LEFT = -1, FULL = 0, RIGHT = 1 };

inline const char *turn_direction_name(TurnDirection direction) {
  switch (direction) {
  case TurnDirection::RIGHT:
    return "RIGHT";
  case TurnDirection::LEFT:
    return "LEFT";
  case TurnDirection::STRAIGHT:
    return "STRAIGHT";
  }
  return "STRAIGHT";
}

inline const char *preferred_image_side_name(PreferredImageSide side) {
  switch (side) {
  case PreferredImageSide::LEFT:
    return "LEFT";
  case PreferredImageSide::RIGHT:
    return "RIGHT";
  case PreferredImageSide::FULL:
    return "FULL";
  }
  return "FULL";
}

struct DynamicTurnRoiConfig {
  // The input is board-IMU angular velocity projected onto global +Z using
  // this OpenVINS passive-JPL convention. Flight replay verifies positive for
  // the fly3 right turn and negative for the fly1 left turn.
  double lowpass_time_constant_s = 0.25;
  double enter_yaw_rate_radps = 0.08;
  double exit_yaw_rate_radps = 0.04;
  double full_strength_yaw_rate_radps = 0.20;
  double entry_confirmation_s = 0.20;
  double recovery_confirmation_s = 0.75;
  double maximum_signal_age_s = 0.35;

  // At full strength, at most 80% of a bounded update may come from the
  // preferred half. The other half is never disabled.
  double maximum_preferred_fraction = 0.80;

  // Detection-only masking removes at most the outer 35% of the harmful side.
  // Existing KLT tracks remain valid across the complete hardware mask.
  double maximum_detection_exclusion_fraction = 0.35;
};

struct DynamicTurnRoiDecision {
  double timestamp = -1.0;
  bool signal_valid = false;
  double raw_yaw_rate_radps = std::numeric_limits<double>::quiet_NaN();
  double filtered_yaw_rate_radps = 0.0;
  TurnDirection direction = TurnDirection::STRAIGHT;
  PreferredImageSide preferred_side = PreferredImageSide::FULL;
  double strength = 0.0;
  double preferred_fraction = 0.50;
  double detection_exclusion_fraction = 0.0;
  std::string reason = "not_initialized";
};

struct DynamicTurnRoiQuota {
  int preferred = 0;
  int other = 0;
};

class DynamicTurnRoiPolicy {
public:
  explicit DynamicTurnRoiPolicy(
      const DynamicTurnRoiConfig &config = DynamicTurnRoiConfig())
      : config_(sanitized(config)) {}

  void reset() {
    initialized_ = false;
    filtered_yaw_rate_radps_ = 0.0;
    last_timestamp_ = -1.0;
    last_valid_signal_timestamp_ = -1.0;
    active_direction_ = TurnDirection::STRAIGHT;
    candidate_direction_ = TurnDirection::STRAIGHT;
    candidate_since_ = -1.0;
    decision_ = DynamicTurnRoiDecision();
  }

  DynamicTurnRoiDecision update(double timestamp, double yaw_rate_radps,
                                bool signal_valid) {
    if (!std::isfinite(timestamp) || timestamp < 0.0 ||
        (last_timestamp_ >= 0.0 && timestamp <= last_timestamp_)) {
      decision_.reason = "non_monotonic_timestamp";
      return decision_;
    }

    const double dt = last_timestamp_ >= 0.0 ? timestamp - last_timestamp_ : 0.0;
    last_timestamp_ = timestamp;
    const bool finite_signal = signal_valid && std::isfinite(yaw_rate_radps);
    if (finite_signal) {
      if (!initialized_) {
        filtered_yaw_rate_radps_ = yaw_rate_radps;
        initialized_ = true;
      } else {
        const double alpha =
            1.0 - std::exp(-dt / config_.lowpass_time_constant_s);
        filtered_yaw_rate_radps_ +=
            std::max(0.0, std::min(1.0, alpha)) *
            (yaw_rate_radps - filtered_yaw_rate_radps_);
      }
      last_valid_signal_timestamp_ = timestamp;
    }

    const bool signal_fresh =
        initialized_ && last_valid_signal_timestamp_ >= 0.0 &&
        timestamp - last_valid_signal_timestamp_ <= config_.maximum_signal_age_s;
    const double abs_rate = std::fabs(filtered_yaw_rate_radps_);
    TurnDirection raw_direction = TurnDirection::STRAIGHT;
    if (signal_fresh && abs_rate >= config_.enter_yaw_rate_radps) {
      raw_direction = filtered_yaw_rate_radps_ > 0.0 ? TurnDirection::RIGHT
                                                     : TurnDirection::LEFT;
    } else if (signal_fresh && active_direction_ != TurnDirection::STRAIGHT &&
               abs_rate > config_.exit_yaw_rate_radps) {
      raw_direction = active_direction_;
    }

    if (raw_direction != candidate_direction_) {
      candidate_direction_ = raw_direction;
      candidate_since_ = timestamp;
    }
    const double confirmation =
        candidate_direction_ == TurnDirection::STRAIGHT
            ? config_.recovery_confirmation_s
            : config_.entry_confirmation_s;
    if (candidate_direction_ != active_direction_ && candidate_since_ >= 0.0 &&
        timestamp - candidate_since_ >= confirmation) {
      active_direction_ = candidate_direction_;
    }

    double strength = 0.0;
    if (active_direction_ != TurnDirection::STRAIGHT && signal_fresh) {
      const double denom = std::max(
          1e-9, config_.full_strength_yaw_rate_radps -
                    config_.exit_yaw_rate_radps);
      const double u = std::max(
          0.0, std::min(1.0, (abs_rate - config_.exit_yaw_rate_radps) / denom));
      strength = u * u * (3.0 - 2.0 * u);
    }

    decision_.timestamp = timestamp;
    decision_.signal_valid = signal_fresh;
    decision_.raw_yaw_rate_radps =
        finite_signal ? yaw_rate_radps
                      : std::numeric_limits<double>::quiet_NaN();
    decision_.filtered_yaw_rate_radps = filtered_yaw_rate_radps_;
    decision_.direction = active_direction_;
    decision_.preferred_side =
        active_direction_ == TurnDirection::LEFT
            ? PreferredImageSide::RIGHT
            : (active_direction_ == TurnDirection::RIGHT
                   ? PreferredImageSide::LEFT
                   : PreferredImageSide::FULL);
    decision_.strength = strength;
    decision_.preferred_fraction =
        0.50 + strength * (config_.maximum_preferred_fraction - 0.50);
    decision_.detection_exclusion_fraction =
        strength * config_.maximum_detection_exclusion_fraction;
    if (!signal_fresh) {
      decision_.reason = "yaw_rate_unavailable";
    } else if (active_direction_ == TurnDirection::STRAIGHT &&
               candidate_direction_ != TurnDirection::STRAIGHT) {
      decision_.reason = "turn_entry_confirmation";
    } else if (active_direction_ != TurnDirection::STRAIGHT &&
               candidate_direction_ == TurnDirection::STRAIGHT) {
      decision_.reason = "turn_recovery_confirmation";
    } else if (active_direction_ == TurnDirection::STRAIGHT) {
      decision_.reason = "straight";
    } else {
      decision_.reason = "turn_active";
    }
    return decision_;
  }

  const DynamicTurnRoiDecision &decision() const { return decision_; }

  static DynamicTurnRoiQuota quota(int capacity, int preferred_available,
                                   int other_available,
                                   double preferred_fraction) {
    DynamicTurnRoiQuota out;
    capacity = std::max(0, std::min(capacity,
                                    preferred_available + other_available));
    preferred_available = std::max(0, preferred_available);
    other_available = std::max(0, other_available);
    preferred_fraction =
        std::max(0.5, std::min(1.0, preferred_fraction));
    const int preferred_target = static_cast<int>(
        std::lround(static_cast<double>(capacity) * preferred_fraction));
    out.preferred = std::min(preferred_available, preferred_target);
    out.other = std::min(other_available, capacity - out.preferred);
    int unfilled = capacity - out.preferred - out.other;
    const int preferred_spare = preferred_available - out.preferred;
    const int add_preferred = std::min(unfilled, preferred_spare);
    out.preferred += add_preferred;
    unfilled -= add_preferred;
    out.other += std::min(unfilled, other_available - out.other);
    return out;
  }

private:
  static DynamicTurnRoiConfig sanitized(DynamicTurnRoiConfig config) {
    config.lowpass_time_constant_s =
        std::max(1e-3, config.lowpass_time_constant_s);
    config.enter_yaw_rate_radps = std::max(0.0, config.enter_yaw_rate_radps);
    config.exit_yaw_rate_radps = std::max(
        0.0, std::min(config.exit_yaw_rate_radps, config.enter_yaw_rate_radps));
    config.full_strength_yaw_rate_radps = std::max(
        config.enter_yaw_rate_radps + 1e-6,
        config.full_strength_yaw_rate_radps);
    config.entry_confirmation_s = std::max(0.0, config.entry_confirmation_s);
    config.recovery_confirmation_s =
        std::max(0.0, config.recovery_confirmation_s);
    config.maximum_signal_age_s = std::max(0.0, config.maximum_signal_age_s);
    config.maximum_preferred_fraction = std::max(
        0.50, std::min(0.95, config.maximum_preferred_fraction));
    config.maximum_detection_exclusion_fraction =
        std::max(0.0, std::min(0.49,
                              config.maximum_detection_exclusion_fraction));
    return config;
  }

  DynamicTurnRoiConfig config_;
  bool initialized_ = false;
  double filtered_yaw_rate_radps_ = 0.0;
  double last_timestamp_ = -1.0;
  double last_valid_signal_timestamp_ = -1.0;
  TurnDirection active_direction_ = TurnDirection::STRAIGHT;
  TurnDirection candidate_direction_ = TurnDirection::STRAIGHT;
  double candidate_since_ = -1.0;
  DynamicTurnRoiDecision decision_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_DYNAMIC_TURN_ROI_POLICY_H
