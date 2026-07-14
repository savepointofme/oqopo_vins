/*
 * Causal P5 tracking/backend cadence state machine.
 *
 * The policy consumes only present/past estimator and sensor signals. It owns
 * exactly one policy state and publishes tracking/backend cadence atomically.
 */

#pragma once

#include "core/VisualCadencePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace ov_msckf {

enum class AdaptivePolicyState {
  HIGH_ALTITUDE_CRUISE,
  NORMAL_CRUISE,
  DESCENT_SAFETY,
  LOW_ALTITUDE_SAFETY,
  TURN_SAFETY,
  VISUAL_DEGRADED,
};

inline const char *adaptive_policy_state_name(AdaptivePolicyState state) {
  switch (state) {
  case AdaptivePolicyState::HIGH_ALTITUDE_CRUISE:
    return "HIGH_ALTITUDE_CRUISE";
  case AdaptivePolicyState::NORMAL_CRUISE:
    return "NORMAL_CRUISE";
  case AdaptivePolicyState::DESCENT_SAFETY:
    return "DESCENT_SAFETY";
  case AdaptivePolicyState::LOW_ALTITUDE_SAFETY:
    return "LOW_ALTITUDE_SAFETY";
  case AdaptivePolicyState::TURN_SAFETY:
    return "TURN_SAFETY";
  case AdaptivePolicyState::VISUAL_DEGRADED:
    return "VISUAL_DEGRADED";
  }
  return "UNKNOWN";
}

struct AdaptiveStrideConfig {
  int stride_min = 1;
  int stride_max = 12;

  int high_tracking_stride = 4;
  int high_backend_stride = 12;
  int normal_tracking_stride = 2;
  int normal_backend_stride = 6;
  int descent_tracking_stride = 2;
  int descent_backend_stride = 4;
  int low50_tracking_stride = 2;
  int low50_backend_stride = 4;
  int low30_tracking_stride = 1;
  int low30_backend_stride = 2;
  int low20_tracking_stride = 1;
  int low20_backend_stride = 1;
  int turn_tracking_stride = 1;
  int turn_backend_stride = 2;
  int degraded_tracking_stride = 1;
  int degraded_backend_stride = 1;

  double high_altitude_enter_m = 120.0;
  double high_altitude_exit_m = 100.0;
  double low_altitude_enter_m = 50.0;
  double low_altitude_exit_m = 55.0;
  double low30_enter_m = 30.0;
  double low30_exit_m = 33.0;
  double low20_enter_m = 20.0;
  double low20_exit_m = 22.0;
  double descent_enter_mps = -1.5;
  double descent_exit_mps = -0.2;

  double turn_enter_gyro_radps = 0.20;
  double turn_exit_gyro_radps = 0.12;
  double turn_severe_gyro_radps = 0.35;
  double turn_enter_roll_deg = 10.0;
  double turn_exit_roll_deg = 6.0;
  double turn_severe_roll_deg = 18.0;
  double turn_enter_pitch_deg = 25.0;
  double turn_exit_pitch_deg = 20.0;
  double turn_severe_pitch_deg = 32.0;

  int feature_degraded_count = 210;
  int feature_emergency_count = 120;
  int feature_recover_count = 260;
  double track_age_degraded_frames = 2.0;
  // P95 thresholds are normalized to one received raw-camera interval. They
  // are therefore independent of the previously commanded stride.
  double parallax_high_px = 6.0;
  double parallax_emergency_px = 10.0;
  double parallax_recover_px = 4.0;
  double parallax_stale_s = 1.0;
  double residual_p95_degraded_px = 8.0;
  double residual_p95_recover_px = 4.0;
  double backend_accept_ratio_degraded = 0.08;
  double backend_accept_ratio_recover = 0.25;
  int backend_ratio_min_input = 12;
  double backend_accept_ratio_tau_s = 2.0;
  double backend_starvation_s = 1.5;
  double position_jump_m = 25.0;
  double velocity_jump_mps = 10.0;
  double attitude_jump_deg = 20.0;

  double predicted_overlap_low_enter = 0.90;
  double predicted_overlap_low_exit = 0.93;
  double predicted_overlap_high_enter = 0.95;
  double horizontal_tan_half_fov = 0.8355;
  double predicted_p95_scale = 1.5;

  double height_tau_s = 1.0;
  double vertical_speed_tau_s = 0.5;
  double gyro_tau_s = 0.20;
  double parallax_rate_tau_s = 0.35;
  double visual_entry_confirm_s = 1.0;
  double low_entry_confirm_s = 0.20;
  double turn_entry_confirm_s = 0.50;
  double descent_entry_confirm_s = 1.50;
  double visual_recovery_s = 5.0;
  double low_recovery_s = 3.0;
  double turn_recovery_s = 5.0;
  double descent_recovery_s = 5.0;
  double high_cruise_entry_s = 6.0;
  double low_band_recovery_s = 2.0;
  double unnecessary_reversal_window_s = 10.0;
  double target_compensated_parallax_min_px = 2.0;
  double target_compensated_parallax_max_px = 8.0;
  double maximum_tracking_interval_s = 0.40;
  double minimum_track_survival_ratio = 0.55;
};

struct AdaptiveStrideInput {
  double timestamp = 0.0;
  bool initialized = false;
  double relative_height_m = std::numeric_limits<double>::quiet_NaN();
  double horizontal_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double vertical_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double roll_deg = std::numeric_limits<double>::quiet_NaN();
  double pitch_deg = std::numeric_limits<double>::quiet_NaN();
  double gyro_norm_radps = std::numeric_limits<double>::quiet_NaN();
  double median_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double p95_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double raw_median_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double raw_p95_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double rotation_median_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double rotation_p95_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double rotation_max_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double track_survival_ratio = std::numeric_limits<double>::quiet_NaN();
  int common_track_count = -1;
  double ground_footprint_polygon_overlap =
      std::numeric_limits<double>::quiet_NaN();
  std::map<int, double> ground_footprint_overlap_by_stride;
  double parallax_measurement_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double parallax_measurement_dt_s = std::numeric_limits<double>::quiet_NaN();
  double actual_received_camera_dt_s = std::numeric_limits<double>::quiet_NaN();
  double actual_tracking_frame_interval_s = std::numeric_limits<double>::quiet_NaN();
  double actual_backend_frame_interval_s = std::numeric_limits<double>::quiet_NaN();
  int active_msckf_features = -1;
  int active_slam_features = -1;
  int configured_feature_count = 0;
  double median_track_age_frames = std::numeric_limits<double>::quiet_NaN();
  double visual_residual_rmse_px = std::numeric_limits<double>::quiet_NaN();
  double visual_residual_p95_px = std::numeric_limits<double>::quiet_NaN();
  int msckf_input_count = -1;
  int msckf_accepted_count = -1;
  int msckf_rejected_count = -1;
  double time_since_accepted_backend_update_s = std::numeric_limits<double>::quiet_NaN();
  bool covariance_all_finite = true;
  int covariance_negative_diagonal_count = 0;
  double position_jump_m = 0.0;
  double velocity_jump_mps = 0.0;
  double attitude_jump_deg = 0.0;
};

struct AdaptiveStrideDecision {
  int recommended_stride = 1; // compatibility alias for backend_update_stride
  int base_stride = 1;
  int tracking_stride = 1;
  int backend_update_stride = 1;
  bool changed = false;
  bool policy_state_changed = false;
  bool tracking_stride_changed = false;
  bool backend_stride_changed = false;
  bool emergency_downshift = false;
  bool normal_recovery = false;
  bool unnecessary_reversal = false;
  bool minimum_dwell_violation = false;
  bool same_state_target_rewrite = false;

  AdaptivePolicyState state = AdaptivePolicyState::NORMAL_CRUISE;
  AdaptivePolicyState previous_state = AdaptivePolicyState::NORMAL_CRUISE;
  std::string policy_state = "NORMAL_CRUISE";
  std::string previous_policy_state = "NORMAL_CRUISE";
  std::string main_trigger = "not_initialized";
  std::string transition_reason = "none";
  std::string switch_reason = "none";
  std::string hysteresis_state = "NORMAL_CRUISE";

  bool reason_turn = false;
  bool reason_feature_low = false;
  bool reason_parallax_low = false;
  bool reason_parallax_high = false;
  bool reason_descent = false;
  bool reason_low_height = false;
  bool reason_height_valid = false;
  bool severe_turn = false;
  bool severe_feature_low = false;
  bool severe_parallax_high = false;
  bool severe_visual_stale = false;
  bool severe_descent = false;
  bool force_fullrate_tracking = false;

  double filtered_height_m = std::numeric_limits<double>::quiet_NaN();
  double filtered_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double filtered_vertical_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_p95_px = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_rate_pxps = std::numeric_limits<double>::quiet_NaN();
  double normalized_parallax_px = std::numeric_limits<double>::quiet_NaN();
  double parallax_measurement_age_s = std::numeric_limits<double>::quiet_NaN();
  double effective_height_m = std::numeric_limits<double>::quiet_NaN();
  double filtered_gyro_radps = std::numeric_limits<double>::quiet_NaN();
  double predicted_overlap = std::numeric_limits<double>::quiet_NaN();
  double predicted_median_displacement_px = std::numeric_limits<double>::quiet_NaN();
  double predicted_p95_displacement_px = std::numeric_limits<double>::quiet_NaN();
  int parallax_target_stride = 1;
  int tracking_safety_cap = 1;
  int backend_latency_cap = 1;
  std::string cadence_planner_reason = "bootstrap_dense";
  double hold_timer_s = 0.0;
  double state_dwell_s = 0.0;
  double transition_guard_s = 0.0;
  int low_altitude_band = 0;
};

class AdaptiveStrideController {
public:
  explicit AdaptiveStrideController(
      const AdaptiveStrideConfig &config = AdaptiveStrideConfig())
      : config_(config), cadence_planner_(make_planner_config(config)) {}

  AdaptiveStrideDecision update(const AdaptiveStrideInput &in) {
    AdaptiveStrideDecision out;
    const double dt = have_time_ ? std::max(0.0, in.timestamp - last_time_) : 0.0;
    last_time_ = in.timestamp;
    have_time_ = true;

    update_lowpass(in.relative_height_m, dt, config_.height_tau_s,
                   filtered_height_m_);
    update_lowpass(in.vertical_speed_mps, dt, config_.vertical_speed_tau_s,
                   filtered_vertical_speed_mps_);
    update_lowpass(in.gyro_norm_radps, dt, config_.gyro_tau_s,
                   filtered_gyro_radps_);

    const bool fresh_flow =
        std::isfinite(in.median_parallax_px) &&
        std::isfinite(in.parallax_measurement_dt_s) &&
        in.parallax_measurement_dt_s > 1.0e-6;
    if (fresh_flow) {
      const double measurement_time =
          std::isfinite(in.parallax_measurement_timestamp_s)
              ? in.parallax_measurement_timestamp_s
              : in.timestamp;
      const double filter_dt = have_flow_
                                   ? std::max(0.0, measurement_time - last_flow_time_)
                                   : 0.0;
      update_lowpass(in.median_parallax_px, filter_dt,
                     config_.parallax_rate_tau_s, filtered_parallax_px_);
      const double p95 = std::isfinite(in.p95_parallax_px)
                             ? in.p95_parallax_px
                             : in.median_parallax_px * config_.predicted_p95_scale;
      update_lowpass(p95, filter_dt, config_.parallax_rate_tau_s,
                     filtered_parallax_p95_px_);
      update_lowpass(in.median_parallax_px / in.parallax_measurement_dt_s,
                     filter_dt, config_.parallax_rate_tau_s,
                     filtered_parallax_rate_pxps_);
      update_lowpass(p95 / in.parallax_measurement_dt_s, filter_dt,
                     config_.parallax_rate_tau_s,
                     filtered_parallax_p95_rate_pxps_);
      last_flow_time_ = measurement_time;
      have_flow_ = true;
    }

    const double raw_dt =
        std::isfinite(in.actual_received_camera_dt_s) &&
                in.actual_received_camera_dt_s > 1.0e-6
            ? in.actual_received_camera_dt_s
            : 1.0 / 30.0;

    if (!in.initialized) {
      was_initialized_ = false;
      reset_candidate(in.timestamp);
      VisualCadencePlannerInput cadence_input;
      cadence_input.timestamp = in.timestamp;
      cadence_input.motion_measurement_timestamp_s =
          in.parallax_measurement_timestamp_s;
      cadence_input.raw_camera_dt_s = raw_dt;
      cadence_input.safety_stride_cap = config_.stride_max;
      cadence_input.motion.dt_s = in.parallax_measurement_dt_s;
      cadence_input.motion.common_tracks =
          in.common_track_count >= 0 ? in.common_track_count
                                     : std::max(0, in.active_msckf_features);
      cadence_input.motion.survival_ratio =
          std::isfinite(in.track_survival_ratio) ? in.track_survival_ratio : 1.0;
      cadence_input.motion.raw_median_px =
          std::isfinite(in.raw_median_parallax_px)
              ? in.raw_median_parallax_px
              : in.median_parallax_px;
      cadence_input.motion.raw_p95_px =
          std::isfinite(in.raw_p95_parallax_px) ? in.raw_p95_parallax_px
                                               : in.p95_parallax_px;
      cadence_input.motion.rotation_median_px =
          std::isfinite(in.rotation_median_parallax_px)
              ? in.rotation_median_parallax_px
              : 0.0;
      cadence_input.motion.rotation_p95_px =
          std::isfinite(in.rotation_p95_parallax_px)
              ? in.rotation_p95_parallax_px
              : 0.0;
      cadence_input.motion.rotation_max_px =
          std::isfinite(in.rotation_max_parallax_px)
              ? in.rotation_max_parallax_px
              : cadence_input.motion.rotation_p95_px;
      cadence_input.motion.compensated_median_px = in.median_parallax_px;
      cadence_input.motion.compensated_p95_px = in.p95_parallax_px;
      cadence_input.motion.valid = fresh_flow &&
                                    cadence_input.motion.common_tracks >= 4;
      const VisualCadenceDecision cadence = cadence_planner_.update(cadence_input);
      tracking_stride_ = cadence.stride;
      backend_stride_ = tracking_stride_;
      fill_output(in, out);
      out.main_trigger = "not_initialized";
      out.tracking_safety_cap = config_.stride_max;
      out.backend_latency_cap = config_.stride_max;
      out.cadence_planner_reason = cadence.reason;
      return out;
    }
    if (!was_initialized_) {
      was_initialized_ = true;
      state_ = AdaptivePolicyState::NORMAL_CRUISE;
      state_enter_time_ = in.timestamp;
      tracking_stride_ = clamp(config_.normal_tracking_stride);
      backend_stride_ = clamp(config_.normal_backend_stride);
      reset_candidate(in.timestamp);
      low_band_ = 0;
    }

    const double safety_height = filtered_height(in.relative_height_m);
    predicted_overlap_ = in.ground_footprint_polygon_overlap;

    const double abs_roll = std::isfinite(in.roll_deg) ? std::fabs(in.roll_deg) : 0.0;
    const double abs_pitch = std::isfinite(in.pitch_deg) ? std::fabs(in.pitch_deg) : 0.0;
    const double gyro = std::isfinite(filtered_gyro_radps_)
                            ? filtered_gyro_radps_
                            : (std::isfinite(in.gyro_norm_radps)
                                   ? in.gyro_norm_radps
                                   : 0.0);
    const bool turn_enter = gyro >= config_.turn_enter_gyro_radps ||
                            abs_roll >= config_.turn_enter_roll_deg ||
                            abs_pitch >= config_.turn_enter_pitch_deg;
    const bool turn_severe =
        (std::isfinite(in.gyro_norm_radps) &&
         in.gyro_norm_radps >= config_.turn_severe_gyro_radps) ||
        abs_roll >= config_.turn_severe_roll_deg ||
        abs_pitch >= config_.turn_severe_pitch_deg;
    const bool turn_recovered = gyro <= config_.turn_exit_gyro_radps &&
                                abs_roll <= config_.turn_exit_roll_deg &&
                                abs_pitch <= config_.turn_exit_pitch_deg;
    const bool descending = std::isfinite(filtered_vertical_speed_mps_) &&
                            filtered_vertical_speed_mps_ <=
                                config_.descent_enter_mps;
    const bool descent_recovered =
        std::isfinite(filtered_vertical_speed_mps_) &&
        filtered_vertical_speed_mps_ >= config_.descent_exit_mps;

    const int active_features = in.active_msckf_features >= 0
                                    ? in.active_msckf_features
                                    : in.active_slam_features;
    const bool visual_stale =
        have_flow_ && in.timestamp - last_flow_time_ > config_.parallax_stale_s;
    const double accept_ratio =
        in.msckf_input_count >= config_.backend_ratio_min_input
            ? static_cast<double>(std::max(0, in.msckf_accepted_count)) /
                  static_cast<double>(std::max(1, in.msckf_input_count))
            : std::numeric_limits<double>::quiet_NaN();
    update_lowpass(accept_ratio, dt, config_.backend_accept_ratio_tau_s,
                   filtered_accept_ratio_);
    const bool backend_starved =
        std::isfinite(in.time_since_accepted_backend_update_s) &&
        in.time_since_accepted_backend_update_s > config_.backend_starvation_s;
    const bool state_jump = in.position_jump_m > config_.position_jump_m ||
                            in.velocity_jump_mps > config_.velocity_jump_mps ||
                            in.attitude_jump_deg > config_.attitude_jump_deg;
    const bool covariance_bad = !in.covariance_all_finite ||
                                in.covariance_negative_diagonal_count > 0;
    const bool severe_feature = active_features >= 0 &&
                                active_features < config_.feature_emergency_count;
    const int feature_threshold =
        state_ == AdaptivePolicyState::VISUAL_DEGRADED
            ? config_.feature_recover_count
            : config_.feature_degraded_count;
    const bool low_feature = active_features >= 0 &&
                             active_features < feature_threshold;
    const double normalized_p95_raw =
        std::isfinite(filtered_parallax_p95_rate_pxps_)
            ? filtered_parallax_p95_rate_pxps_ * raw_dt
            : std::numeric_limits<double>::quiet_NaN();
    const bool severe_parallax =
        std::isfinite(normalized_p95_raw) &&
        normalized_p95_raw > config_.parallax_emergency_px;
    const bool high_parallax =
        std::isfinite(normalized_p95_raw) &&
        normalized_p95_raw >
            (state_ == AdaptivePolicyState::VISUAL_DEGRADED
                 ? config_.parallax_recover_px
                 : config_.parallax_high_px);
    const bool residual_bad =
        std::isfinite(in.visual_residual_p95_px) &&
        in.visual_residual_p95_px >
            (state_ == AdaptivePolicyState::VISUAL_DEGRADED
                 ? config_.residual_p95_recover_px
                 : config_.residual_p95_degraded_px);
    const bool accept_bad = std::isfinite(filtered_accept_ratio_) &&
                            filtered_accept_ratio_ <
                                (state_ == AdaptivePolicyState::VISUAL_DEGRADED
                                     ? config_.backend_accept_ratio_recover
                                     : config_.backend_accept_ratio_degraded);
    const bool track_age_bad =
        std::isfinite(in.median_track_age_frames) &&
        in.median_track_age_frames < config_.track_age_degraded_frames &&
        low_feature;
    const bool visual_bad = visual_stale || severe_feature || low_feature ||
                            severe_parallax || high_parallax || residual_bad ||
                            accept_bad || backend_starved || covariance_bad ||
                            state_jump || track_age_bad;
    const bool hard_visual_emergency =
        visual_stale || covariance_bad || state_jump;

    const bool low_altitude =
        (std::isfinite(safety_height) &&
         safety_height < config_.low_altitude_enter_m) ||
        (std::isfinite(predicted_overlap_) &&
         predicted_overlap_ < config_.predicted_overlap_low_enter);
    const bool low_recovered =
        (!std::isfinite(safety_height) ||
         safety_height > config_.low_altitude_exit_m) &&
        (!std::isfinite(predicted_overlap_) ||
         predicted_overlap_ > config_.predicted_overlap_low_exit);
    const bool already_high =
        state_ == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE;
    const bool high_cruise =
        std::isfinite(safety_height) &&
        safety_height >= (already_high ? config_.high_altitude_exit_m
                                       : config_.high_altitude_enter_m) &&
        (!std::isfinite(predicted_overlap_) ||
         predicted_overlap_ >=
             (already_high ? config_.predicted_overlap_low_exit
                           : config_.predicted_overlap_high_enter)) &&
        turn_recovered && descent_recovered && !visual_bad;

    AdaptivePolicyState raw_state = AdaptivePolicyState::NORMAL_CRUISE;
    std::string raw_trigger = "normal_flight";
    if (visual_bad) {
      raw_state = AdaptivePolicyState::VISUAL_DEGRADED;
      raw_trigger = visual_trigger(visual_stale, severe_feature, severe_parallax,
                                   covariance_bad, state_jump, residual_bad,
                                   accept_bad, backend_starved, low_feature);
    } else if (low_altitude) {
      raw_state = AdaptivePolicyState::LOW_ALTITUDE_SAFETY;
      raw_trigger = std::isfinite(safety_height) &&
                            safety_height < config_.low_altitude_enter_m
                        ? "low_altitude"
                        : "predicted_overlap_low";
    } else if (turn_enter ||
               (state_ == AdaptivePolicyState::TURN_SAFETY && !turn_recovered)) {
      raw_state = AdaptivePolicyState::TURN_SAFETY;
      raw_trigger = turn_severe ? "severe_turn" : "turn";
    } else if (descending ||
               (state_ == AdaptivePolicyState::DESCENT_SAFETY &&
                !descent_recovered)) {
      raw_state = AdaptivePolicyState::DESCENT_SAFETY;
      raw_trigger = "descent";
    } else if (high_cruise) {
      raw_state = AdaptivePolicyState::HIGH_ALTITUDE_CRUISE;
      raw_trigger = "stable_high_altitude";
    }

    if (raw_state != candidate_state_ || raw_trigger != candidate_trigger_) {
      candidate_state_ = raw_state;
      candidate_trigger_ = raw_trigger;
      candidate_since_ = in.timestamp;
    }
    const double guard_elapsed = std::max(0.0, in.timestamp - candidate_since_);
    const bool safety_direction = priority(raw_state) > priority(state_);
    const double required = safety_direction
                                ? entry_confirm(raw_state, hard_visual_emergency,
                                                turn_severe, safety_height)
                                : recovery_confirm(state_, raw_state);
    const bool state_exit_valid =
        (state_ != AdaptivePolicyState::VISUAL_DEGRADED || !visual_bad) &&
        (state_ != AdaptivePolicyState::LOW_ALTITUDE_SAFETY || low_recovered) &&
        (state_ != AdaptivePolicyState::TURN_SAFETY || turn_recovered) &&
        (state_ != AdaptivePolicyState::DESCENT_SAFETY || descent_recovered);

    const AdaptivePolicyState previous_state = state_;
    const int previous_tracking = tracking_stride_;
    const int previous_backend = backend_stride_;
    bool state_changed = false;
    bool band_changed = false;
    if (raw_state != state_ && guard_elapsed + 1.0e-12 >= required &&
        (safety_direction || state_exit_valid)) {
      state_ = raw_state;
      state_enter_time_ = in.timestamp;
      state_changed = true;
      candidate_since_ = in.timestamp;
    }

    if (state_ == AdaptivePolicyState::LOW_ALTITUDE_SAFETY) {
      band_changed = update_low_band(safety_height, in.timestamp);
    } else {
      low_band_ = 0;
      low_band_candidate_ = 0;
      low_band_candidate_since_ = in.timestamp;
    }
    int tracking_cap = config_.stride_max;
    int backend_cap = config_.stride_max;
    assign_safety_caps(state_, tracking_cap, backend_cap);
    VisualCadencePlannerInput cadence_input;
    cadence_input.timestamp = in.timestamp;
    cadence_input.motion_measurement_timestamp_s =
        in.parallax_measurement_timestamp_s;
    cadence_input.raw_camera_dt_s = raw_dt;
    cadence_input.safety_stride_cap = tracking_cap;
    cadence_input.force_dense = state_ == AdaptivePolicyState::VISUAL_DEGRADED;
    cadence_input.ground_footprint_overlap =
        in.ground_footprint_polygon_overlap;
    cadence_input.ground_footprint_overlap_by_stride =
        in.ground_footprint_overlap_by_stride;
    cadence_input.motion.dt_s = in.parallax_measurement_dt_s;
    cadence_input.motion.common_tracks =
        in.common_track_count >= 0 ? in.common_track_count : active_features;
    cadence_input.motion.previous_tracks = std::max(1, in.configured_feature_count);
    cadence_input.motion.current_tracks = std::max(0, active_features);
    cadence_input.motion.survival_ratio =
        std::isfinite(in.track_survival_ratio)
            ? in.track_survival_ratio
            : static_cast<double>(std::max(0, active_features)) /
                  static_cast<double>(std::max(1, in.configured_feature_count));
    cadence_input.motion.raw_median_px =
        std::isfinite(in.raw_median_parallax_px)
            ? in.raw_median_parallax_px
            : in.median_parallax_px;
    cadence_input.motion.raw_p95_px =
        std::isfinite(in.raw_p95_parallax_px) ? in.raw_p95_parallax_px
                                             : in.p95_parallax_px;
    cadence_input.motion.rotation_median_px =
        std::isfinite(in.rotation_median_parallax_px)
            ? in.rotation_median_parallax_px
            : 0.0;
    cadence_input.motion.rotation_p95_px =
        std::isfinite(in.rotation_p95_parallax_px)
            ? in.rotation_p95_parallax_px
            : 0.0;
    cadence_input.motion.rotation_max_px =
        std::isfinite(in.rotation_max_parallax_px)
            ? in.rotation_max_parallax_px
            : cadence_input.motion.rotation_p95_px;
    cadence_input.motion.compensated_median_px = in.median_parallax_px;
    cadence_input.motion.compensated_p95_px = in.p95_parallax_px;
    cadence_input.motion.valid = fresh_flow &&
                                  cadence_input.motion.common_tracks >= 4;
    const VisualCadenceDecision cadence = cadence_planner_.update(cadence_input);
    tracking_stride_ = std::min(clamp(tracking_cap), clamp(cadence.stride));
    backend_stride_ = clamp(std::max(tracking_stride_, backend_cap));

    out.previous_state = previous_state;
    out.state = state_;
    out.previous_policy_state = adaptive_policy_state_name(previous_state);
    out.policy_state = adaptive_policy_state_name(state_);
    out.policy_state_changed = state_changed;
    // Registered low-altitude sub-band changes are explicit policy events,
    // not an unregistered same-state rewrite. This metric is reserved for a
    // controller defect and must remain zero in the current implementation.
    out.same_state_target_rewrite = false;
    out.tracking_stride_changed = tracking_stride_ != previous_tracking;
    out.backend_stride_changed = backend_stride_ != previous_backend;
    out.changed = out.tracking_stride_changed || out.backend_stride_changed;
    out.normal_recovery = state_changed && priority(state_) < priority(previous_state);
    out.emergency_downshift =
        state_changed && safety_direction &&
        (tracking_stride_ < previous_tracking || backend_stride_ < previous_backend);
    const int direction = backend_stride_ > previous_backend
                              ? 1
                              : (backend_stride_ < previous_backend ? -1 : 0);
    const bool external_change = state_changed || band_changed;
    out.unnecessary_reversal =
        direction != 0 && last_stride_direction_ != 0 &&
        direction != last_stride_direction_ && !external_change &&
        in.timestamp - last_stride_change_time_ <=
            config_.unnecessary_reversal_window_s;
    if (direction != 0) {
      last_stride_direction_ = direction;
      last_stride_change_time_ = in.timestamp;
    }
    out.minimum_dwell_violation = false;
    out.main_trigger = state_changed ? candidate_trigger_ : raw_trigger;
    out.transition_reason =
        state_changed
            ? std::string(adaptive_policy_state_name(previous_state)) + "->" +
                  adaptive_policy_state_name(state_) + ":" + candidate_trigger_
            : (band_changed ? "low_altitude_band_change" : "none");
    out.switch_reason = out.transition_reason;
    out.transition_guard_s = guard_elapsed;
    out.state_dwell_s = std::max(0.0, in.timestamp - state_enter_time_);
    out.hold_timer_s = std::max(0.0, required - guard_elapsed);

    fill_output(in, out);
    out.tracking_safety_cap = tracking_cap;
    out.backend_latency_cap = backend_cap;
    out.cadence_planner_reason = cadence.reason;
    out.reason_turn = turn_enter || state_ == AdaptivePolicyState::TURN_SAFETY;
    out.reason_feature_low = low_feature;
    out.reason_parallax_high = high_parallax;
    out.reason_descent = descending || state_ == AdaptivePolicyState::DESCENT_SAFETY;
    out.reason_low_height = low_altitude || state_ == AdaptivePolicyState::LOW_ALTITUDE_SAFETY;
    out.severe_turn = turn_severe;
    out.severe_feature_low = severe_feature;
    out.severe_parallax_high = severe_parallax;
    out.severe_visual_stale = visual_stale;
    out.force_fullrate_tracking = tracking_stride_ == 1;
    return out;
  }

  int recommended_stride() const { return backend_stride_; }
  int tracking_stride() const { return tracking_stride_; }
  int backend_update_stride() const { return backend_stride_; }
  AdaptivePolicyState policy_state() const { return state_; }
  const AdaptiveStrideConfig &config() const { return config_; }

private:
  static void update_lowpass(double value, double dt, double tau,
                             double &state) {
    if (!std::isfinite(value))
      return;
    if (!std::isfinite(state) || dt <= 0.0 || tau <= 0.0) {
      state = value;
      return;
    }
    state += (1.0 - std::exp(-dt / tau)) * (value - state);
  }

  int clamp(int value) const {
    return std::max(config_.stride_min,
                    std::min(config_.stride_max, value));
  }

  static int priority(AdaptivePolicyState state) {
    switch (state) {
    case AdaptivePolicyState::VISUAL_DEGRADED:
      return 6;
    case AdaptivePolicyState::LOW_ALTITUDE_SAFETY:
      return 5;
    case AdaptivePolicyState::TURN_SAFETY:
      return 4;
    case AdaptivePolicyState::DESCENT_SAFETY:
      return 3;
    case AdaptivePolicyState::NORMAL_CRUISE:
      return 2;
    case AdaptivePolicyState::HIGH_ALTITUDE_CRUISE:
      return 1;
    }
    return 0;
  }

  double entry_confirm(AdaptivePolicyState state, bool visual_emergency,
                       bool severe_turn, double height) const {
    if (state == AdaptivePolicyState::VISUAL_DEGRADED)
      return visual_emergency ? 0.0 : config_.visual_entry_confirm_s;
    if (state == AdaptivePolicyState::LOW_ALTITUDE_SAFETY)
      return std::isfinite(height) && height < config_.low30_enter_m
                 ? 0.0
                 : config_.low_entry_confirm_s;
    if (state == AdaptivePolicyState::TURN_SAFETY)
      return severe_turn ? 0.0 : config_.turn_entry_confirm_s;
    if (state == AdaptivePolicyState::DESCENT_SAFETY)
      return config_.descent_entry_confirm_s;
    return 0.25;
  }

  double recovery_confirm(AdaptivePolicyState current,
                          AdaptivePolicyState target) const {
    if (current == AdaptivePolicyState::VISUAL_DEGRADED)
      return config_.visual_recovery_s;
    if (current == AdaptivePolicyState::LOW_ALTITUDE_SAFETY)
      return config_.low_recovery_s;
    if (current == AdaptivePolicyState::TURN_SAFETY)
      return config_.turn_recovery_s;
    if (current == AdaptivePolicyState::DESCENT_SAFETY)
      return config_.descent_recovery_s;
    if (target == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE)
      return config_.high_cruise_entry_s;
    return 1.0;
  }

  double filtered_height(double raw_height) const {
    if (std::isfinite(raw_height) && std::isfinite(filtered_height_m_))
      return std::min(raw_height, filtered_height_m_);
    return std::isfinite(raw_height) ? raw_height : filtered_height_m_;
  }

  bool update_low_band(double height, double now) {
    int desired = 1;
    if (std::isfinite(height)) {
      if (height < config_.low20_enter_m)
        desired = 3;
      else if (height < config_.low30_enter_m)
        desired = 2;
    }
    if (low_band_ == 3 && std::isfinite(height) &&
        height <= config_.low20_exit_m)
      desired = 3;
    if (low_band_ >= 2 && std::isfinite(height) &&
        height <= config_.low30_exit_m)
      desired = std::max(desired, 2);
    if (desired != low_band_candidate_) {
      low_band_candidate_ = desired;
      low_band_candidate_since_ = now;
    }
    const bool safer = desired > low_band_;
    const double required = safer ? 0.0 : config_.low_band_recovery_s;
    if (desired != low_band_ &&
        now - low_band_candidate_since_ + 1.0e-12 >= required) {
      low_band_ = desired;
      return true;
    }
    return false;
  }

  void assign_safety_caps(AdaptivePolicyState state, int &tracking_cap,
                          int &backend_cap) const {
    switch (state) {
    case AdaptivePolicyState::HIGH_ALTITUDE_CRUISE:
      tracking_cap = config_.stride_max;
      backend_cap = config_.high_backend_stride;
      break;
    case AdaptivePolicyState::NORMAL_CRUISE:
      tracking_cap = std::max(config_.normal_tracking_stride, 8);
      backend_cap = std::max(config_.normal_backend_stride, 8);
      break;
    case AdaptivePolicyState::DESCENT_SAFETY:
      tracking_cap = config_.descent_tracking_stride;
      backend_cap = config_.descent_backend_stride;
      break;
    case AdaptivePolicyState::LOW_ALTITUDE_SAFETY:
      if (low_band_ >= 3) {
        tracking_cap = config_.low20_tracking_stride;
        backend_cap = config_.low20_backend_stride;
      } else if (low_band_ == 2) {
        tracking_cap = config_.low30_tracking_stride;
        backend_cap = config_.low30_backend_stride;
      } else {
        tracking_cap = config_.low50_tracking_stride;
        backend_cap = config_.low50_backend_stride;
      }
      break;
    case AdaptivePolicyState::TURN_SAFETY:
      tracking_cap = config_.turn_tracking_stride;
      backend_cap = config_.turn_backend_stride;
      break;
    case AdaptivePolicyState::VISUAL_DEGRADED:
      tracking_cap = config_.degraded_tracking_stride;
      backend_cap = config_.degraded_backend_stride;
      break;
    }
  }

  static VisualCadencePlannerConfig
  make_planner_config(const AdaptiveStrideConfig &config) {
    VisualCadencePlannerConfig planner;
    planner.target_compensated_parallax_min_px =
        config.target_compensated_parallax_min_px;
    planner.target_compensated_parallax_max_px =
        config.target_compensated_parallax_max_px;
    planner.max_compensated_p95_px = config.parallax_emergency_px * 1.8;
    planner.max_total_p95_px = 35.0;
    planner.max_rotation_p95_px = 28.0;
    planner.minimum_survival_ratio = config.minimum_track_survival_ratio;
    planner.maximum_tracking_interval_s = config.maximum_tracking_interval_s;
    planner.minimum_ground_overlap = config.predicted_overlap_low_enter;
    return planner;
  }

  void fill_output(const AdaptiveStrideInput &in,
                   AdaptiveStrideDecision &out) const {
    out.state = state_;
    out.policy_state = adaptive_policy_state_name(state_);
    out.hysteresis_state = out.policy_state;
    out.tracking_stride = tracking_stride_;
    out.backend_update_stride = backend_stride_;
    out.recommended_stride = backend_stride_;
    out.base_stride = backend_stride_;
    out.filtered_height_m = filtered_height_m_;
    out.filtered_vertical_speed_mps = filtered_vertical_speed_mps_;
    out.filtered_gyro_radps = filtered_gyro_radps_;
    out.filtered_parallax_px = filtered_parallax_px_;
    out.filtered_parallax_p95_px = filtered_parallax_p95_px_;
    out.filtered_parallax_rate_pxps = filtered_parallax_rate_pxps_;
    out.effective_height_m = filtered_height(in.relative_height_m);
    out.reason_height_valid = std::isfinite(out.effective_height_m);
    out.parallax_measurement_age_s =
        have_flow_ ? std::max(0.0, in.timestamp - last_flow_time_)
                   : std::numeric_limits<double>::quiet_NaN();
    const double raw_dt =
        std::isfinite(in.actual_received_camera_dt_s) &&
                in.actual_received_camera_dt_s > 1.0e-6
            ? in.actual_received_camera_dt_s
            : 1.0 / 30.0;
    if (std::isfinite(filtered_parallax_rate_pxps_)) {
      out.normalized_parallax_px = filtered_parallax_rate_pxps_ * raw_dt;
      out.predicted_median_displacement_px =
          filtered_parallax_rate_pxps_ * raw_dt * backend_stride_;
    }
    if (std::isfinite(filtered_parallax_p95_rate_pxps_))
      out.predicted_p95_displacement_px =
          filtered_parallax_p95_rate_pxps_ * raw_dt * backend_stride_;
    out.predicted_overlap = predicted_overlap_;
    out.parallax_target_stride = backend_stride_;
    out.low_altitude_band = low_band_;
  }

  static std::string visual_trigger(bool stale, bool feature, bool parallax,
                                    bool covariance, bool jump,
                                    bool residual, bool accept,
                                    bool starvation, bool low_feature) {
    if (covariance)
      return "covariance_invalid";
    if (jump)
      return "state_jump";
    if (stale)
      return "visual_stale";
    if (feature)
      return "feature_emergency";
    if (parallax)
      return "parallax_emergency";
    if (residual)
      return "visual_residual_high";
    if (accept)
      return "msckf_acceptance_low";
    if (starvation)
      return "backend_update_starved";
    if (low_feature)
      return "feature_low";
    return "visual_degraded";
  }

  void reset_candidate(double now) {
    candidate_state_ = state_;
    candidate_trigger_ = "not_initialized";
    candidate_since_ = now;
  }

  AdaptiveStrideConfig config_;
  VisualCadencePlanner cadence_planner_;
  bool have_time_ = false;
  bool was_initialized_ = false;
  double last_time_ = 0.0;
  bool have_flow_ = false;
  double last_flow_time_ = 0.0;
  double filtered_height_m_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_vertical_speed_mps_ =
      std::numeric_limits<double>::quiet_NaN();
  double filtered_gyro_radps_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_px_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_p95_px_ = std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_rate_pxps_ =
      std::numeric_limits<double>::quiet_NaN();
  double filtered_parallax_p95_rate_pxps_ =
      std::numeric_limits<double>::quiet_NaN();
  double filtered_accept_ratio_ = std::numeric_limits<double>::quiet_NaN();
  double predicted_overlap_ = std::numeric_limits<double>::quiet_NaN();

  AdaptivePolicyState state_ = AdaptivePolicyState::NORMAL_CRUISE;
  AdaptivePolicyState candidate_state_ = AdaptivePolicyState::NORMAL_CRUISE;
  std::string candidate_trigger_ = "not_initialized";
  double candidate_since_ = 0.0;
  double state_enter_time_ = 0.0;
  int tracking_stride_ = 1;
  int backend_stride_ = 1;
  int low_band_ = 0;
  int low_band_candidate_ = 0;
  double low_band_candidate_since_ = 0.0;
  int last_stride_direction_ = 0;
  double last_stride_change_time_ =
      -std::numeric_limits<double>::infinity();
};

} // namespace ov_msckf
