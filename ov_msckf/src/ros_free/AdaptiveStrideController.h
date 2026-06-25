/*
 * OpenVINS ROS-free adaptive camera stride controller.
 *
 * This controller is deliberately estimator-agnostic and causal.  It never
 * reads a flight identifier, a dataset name, future GPS, or an offline error.
 * The runner supplies only quantities available at the current camera time.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace ov_msckf {

struct AdaptiveStrideConfig {
  // Height bands (V2, 2026-06-22).  V1 placed the stride-3->4 boundary at
  // 260 m, which mapped fly4's ~300 m cruise to base stride 4.  The
  // leave-one-flight-out audit (train on fly1/fly2/fly3, hold fly4) recovers a
  // 3->4 boundary near 349 m and a 4->5 boundary near 449 m, so ~300 m belongs
  // to stride 3 and stride 4 is the higher 350..440 m interpolation band.
  // Unified causal rule: ~200 m -> stride 2, ~300 m -> stride 3, ~500 m ->
  // stride 5.  Bands are data-derived from pooled height quantiles only; no
  // flight identifier is read.
  std::array<double, 4> height_up_m{{100.0, 225.0, 350.0, 440.0}};
  std::array<double, 4> height_hysteresis_m{{15.0, 20.0, 25.0, 25.0}};
  double height_tau_s = 2.0;
  double speed_tau_s = 1.0;
  double gyro_tau_s = 0.25;
  double speed_reference_mps = 40.0;
  double speed_height_correction_m_per_mps = 2.0;
  double max_speed_height_correction_m = 40.0;

  // Cross-flight pooled quantiles: |roll| q95=6.38 deg; gyro q90=0.160,
  // q95=0.207 rad/s.  Exit gates are lower to provide hysteresis.
  double turn_enter_gyro_radps = 0.16;
  double turn_exit_gyro_radps = 0.10;
  double turn_severe_gyro_radps = 0.24;
  double turn_enter_roll_deg = 6.0;
  double turn_exit_roll_deg = 4.0;
  double turn_severe_roll_deg = 9.0;
  double pitch_guard_deg = 25.0;
  double pitch_severe_deg = 32.0;
  double pitch_exit_deg = 20.0;
  double turn_quiet_recovery_s = 4.0;

  // A 400-point front-end has a pooled lower tail near 260 tracks; fly3 uses
  // 700 points, so the fractional term preserves the same meaning without a
  // flight-specific table.
  int feature_low_floor = 260;
  int feature_recover_floor = 290;
  int feature_severe_floor = 160;
  double feature_low_fraction = 0.55;
  double feature_recover_fraction = 0.60;
  double feature_severe_fraction = 0.35;
  double feature_recovery_s = 2.0;

  double down_hold_s = 0.50;
  double up_hold_s = 3.0;
};

struct AdaptiveStrideInput {
  double timestamp = 0.0;
  bool initialized = false;
  double relative_height_m = std::numeric_limits<double>::quiet_NaN();
  double horizontal_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double roll_deg = std::numeric_limits<double>::quiet_NaN();
  double pitch_deg = std::numeric_limits<double>::quiet_NaN();
  double gyro_norm_radps = std::numeric_limits<double>::quiet_NaN();
  int active_msckf_features = -1;
  int active_slam_features = -1;
  int configured_feature_count = 0;
};

struct AdaptiveStrideDecision {
  int recommended_stride = 1;
  int base_stride = 1;
  bool changed = false;
  bool reason_turn = false;
  bool reason_feature_low = false;
  bool reason_height_valid = false;
  bool severe_turn = false;
  bool severe_feature_low = false;
  double filtered_height_m = std::numeric_limits<double>::quiet_NaN();
  double filtered_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double effective_height_m = std::numeric_limits<double>::quiet_NaN();
  double filtered_gyro_radps = std::numeric_limits<double>::quiet_NaN();
  double hold_timer_s = 0.0;
  std::string hysteresis_state = "uninitialized";
  std::string switch_reason = "none";
};

class AdaptiveStrideController {
public:
  explicit AdaptiveStrideController(const AdaptiveStrideConfig &config = AdaptiveStrideConfig())
      : config_(config) {}

  AdaptiveStrideDecision update(const AdaptiveStrideInput &in) {
    AdaptiveStrideDecision out;
    const bool first = !have_time_;
    double dt = first ? 0.0 : std::max(0.0, in.timestamp - last_time_);
    last_time_ = in.timestamp;
    have_time_ = true;

    update_lowpass(in.relative_height_m, dt, config_.height_tau_s, filtered_height_m_);
    update_lowpass(in.horizontal_speed_mps, dt, config_.speed_tau_s, filtered_speed_mps_);
    update_lowpass(in.gyro_norm_radps, dt, config_.gyro_tau_s, filtered_gyro_radps_);

    out.filtered_height_m = filtered_height_m_;
    out.filtered_speed_mps = filtered_speed_mps_;
    out.filtered_gyro_radps = filtered_gyro_radps_;
    out.reason_height_valid = std::isfinite(filtered_height_m_);

    double effective_height = filtered_height_m_;
    if (std::isfinite(effective_height) && std::isfinite(filtered_speed_mps_)) {
      double correction = config_.speed_height_correction_m_per_mps *
                          (filtered_speed_mps_ - config_.speed_reference_mps);
      correction = std::max(-config_.max_speed_height_correction_m,
                            std::min(config_.max_speed_height_correction_m, correction));
      effective_height -= correction;
    }
    out.effective_height_m = effective_height;
    update_height_band(effective_height);
    out.base_stride = height_base_stride_;

    const double abs_roll = std::isfinite(in.roll_deg) ? std::fabs(in.roll_deg) : 0.0;
    const double abs_pitch = std::isfinite(in.pitch_deg) ? std::fabs(in.pitch_deg) : 0.0;
    const double raw_gyro = std::isfinite(in.gyro_norm_radps) ? in.gyro_norm_radps : 0.0;
    const double filt_gyro = std::isfinite(filtered_gyro_radps_) ? filtered_gyro_radps_ : raw_gyro;
    const bool turn_enter = raw_gyro >= config_.turn_enter_gyro_radps ||
                            filt_gyro >= config_.turn_enter_gyro_radps ||
                            abs_roll >= config_.turn_enter_roll_deg ||
                            abs_pitch >= config_.pitch_guard_deg;
    const bool turn_severe = raw_gyro >= config_.turn_severe_gyro_radps ||
                             abs_roll >= config_.turn_severe_roll_deg ||
                             abs_pitch >= config_.pitch_severe_deg;
    const bool turn_quiet = filt_gyro <= config_.turn_exit_gyro_radps &&
                            abs_roll <= config_.turn_exit_roll_deg &&
                            abs_pitch <= config_.pitch_exit_deg;
    if (turn_enter) {
      turn_active_ = true;
      if (turn_severe)
        turn_severe_active_ = true;
      turn_quiet_since_ = std::numeric_limits<double>::quiet_NaN();
    } else if (turn_active_) {
      if (turn_quiet) {
        if (!std::isfinite(turn_quiet_since_))
          turn_quiet_since_ = in.timestamp;
        if (in.timestamp - turn_quiet_since_ >= config_.turn_quiet_recovery_s) {
          turn_active_ = false;
          turn_severe_active_ = false;
        }
      } else {
        turn_quiet_since_ = std::numeric_limits<double>::quiet_NaN();
      }
    }

    const int configured = std::max(1, in.configured_feature_count);
    const int low_threshold = std::max(config_.feature_low_floor,
        static_cast<int>(std::floor(config_.feature_low_fraction * configured)));
    const int recover_threshold = std::max(config_.feature_recover_floor,
        static_cast<int>(std::floor(config_.feature_recover_fraction * configured)));
    const int severe_threshold = std::max(config_.feature_severe_floor,
        static_cast<int>(std::floor(config_.feature_severe_fraction * configured)));
    // The tracker/MSCKF pool is compared with num_pts.  SLAM is logged
    // separately because adding its capped 0..50 state count would move the
    // threshold between initialization and cruise without any real front-end
    // recovery.  If no tracker count is available, fall back to SLAM count.
    const int active_features = in.active_msckf_features >= 0
                                    ? in.active_msckf_features
                                    : std::max(0, in.active_slam_features);
    const bool feature_available = in.active_msckf_features >= 0 || in.active_slam_features >= 0;
    const bool feature_low_now = feature_available && active_features < low_threshold;
    const bool feature_severe_now = feature_available && active_features < severe_threshold;
    if (feature_low_now) {
      feature_low_active_ = true;
      if (feature_severe_now)
        feature_severe_active_ = true;
      feature_recovered_since_ = std::numeric_limits<double>::quiet_NaN();
    } else if (feature_low_active_) {
      if (feature_available && active_features >= recover_threshold) {
        if (!std::isfinite(feature_recovered_since_))
          feature_recovered_since_ = in.timestamp;
        if (in.timestamp - feature_recovered_since_ >= config_.feature_recovery_s) {
          feature_low_active_ = false;
          feature_severe_active_ = false;
        }
      } else {
        feature_recovered_since_ = std::numeric_limits<double>::quiet_NaN();
      }
    }

    int desired = in.initialized && out.reason_height_valid ? height_base_stride_ : 1;
    if (turn_active_)
      desired = std::min(desired, turn_severe_active_ ? 1 : 2);
    if (feature_low_active_)
      desired = std::min(desired, feature_severe_active_ ? 1 : 2);
    desired = std::max(1, std::min(5, desired));

    if (!have_switch_time_) {
      last_switch_time_ = in.timestamp;
      have_switch_time_ = true;
    }
    const double since_switch = std::max(0.0, in.timestamp - last_switch_time_);
    const double required_hold = desired < recommended_stride_ ? config_.down_hold_s : config_.up_hold_s;
    if (desired != recommended_stride_ && since_switch + 1e-12 >= required_hold) {
      const bool moving_up = desired > recommended_stride_;
      recommended_stride_ += moving_up ? 1 : -1;
      recommended_stride_ = std::max(1, std::min(5, recommended_stride_));
      last_switch_time_ = in.timestamp;
      out.changed = true;
      if (feature_low_active_)
        out.switch_reason = feature_severe_active_ ? "feature_severe" : "feature_low";
      else if (turn_active_)
        out.switch_reason = turn_severe_active_ ? "turn_severe" : "turn";
      else
        out.switch_reason = moving_up ? "height_up" : "height_down";
    }

    out.recommended_stride = recommended_stride_;
    out.reason_turn = turn_active_;
    out.reason_feature_low = feature_low_active_;
    out.severe_turn = turn_severe_active_;
    out.severe_feature_low = feature_severe_active_;
    const double active_hold = desired < recommended_stride_ ? config_.down_hold_s : config_.up_hold_s;
    out.hold_timer_s = std::max(0.0, active_hold - std::max(0.0, in.timestamp - last_switch_time_));
    if (!in.initialized)
      out.hysteresis_state = "not_initialized";
    else if (feature_low_active_)
      out.hysteresis_state = "feature_low";
    else if (turn_active_)
      out.hysteresis_state = "turn_hold";
    else if (desired > recommended_stride_)
      out.hysteresis_state = "height_recovery_wait";
    else if (desired < recommended_stride_)
      out.hysteresis_state = "down_hold";
    else
      out.hysteresis_state = "stable";
    return out;
  }

  int recommended_stride() const { return recommended_stride_; }
  const AdaptiveStrideConfig &config() const { return config_; }

private:
  static void update_lowpass(double value, double dt, double tau, double &state) {
    if (!std::isfinite(value))
      return;
    if (!std::isfinite(state) || dt <= 0.0 || tau <= 0.0) {
      state = value;
      return;
    }
    const double alpha = 1.0 - std::exp(-dt / tau);
    state += alpha * (value - state);
  }

  void update_height_band(double height_m) {
    if (!std::isfinite(height_m)) {
      height_base_stride_ = 1;
      return;
    }
    while (height_base_stride_ < 5) {
      const int boundary = height_base_stride_ - 1;
      if (height_m >= config_.height_up_m[boundary] + config_.height_hysteresis_m[boundary])
        height_base_stride_++;
      else
        break;
    }
    while (height_base_stride_ > 1) {
      const int boundary = height_base_stride_ - 2;
      if (height_m < config_.height_up_m[boundary] - config_.height_hysteresis_m[boundary])
        height_base_stride_--;
      else
        break;
    }
  }

  AdaptiveStrideConfig config_;
  bool have_time_ = false;
  bool have_switch_time_ = false;
  double last_time_ = 0.0;
  double last_switch_time_ = 0.0;
  double filtered_height_m_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_speed_mps_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_gyro_radps_ = std::numeric_limits<double>::quiet_NaN();
  double turn_quiet_since_ = std::numeric_limits<double>::quiet_NaN();
  double feature_recovered_since_ = std::numeric_limits<double>::quiet_NaN();
  int recommended_stride_ = 1;
  int height_base_stride_ = 1;
  bool turn_active_ = false;
  bool turn_severe_active_ = false;
  bool feature_low_active_ = false;
  bool feature_severe_active_ = false;
};

} // namespace ov_msckf
