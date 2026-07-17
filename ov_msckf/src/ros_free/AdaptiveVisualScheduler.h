/* Continuous P5 tracking scheduler; no flight-state-to-stride table. */

#ifndef OV_MSCKF_ADAPTIVE_VISUAL_SCHEDULER_H
#define OV_MSCKF_ADAPTIVE_VISUAL_SCHEDULER_H

#include "core/VisualCadencePlanner.h"

#include <cmath>
#include <limits>
#include <string>

namespace ov_msckf {

enum class AdaptiveVisualMode {
  BOOTSTRAP_DENSE = 0,
  GEOMETRY_CONTROLLED = 1,
  TRACKING_PROTECTION = 2,
  ESTIMATOR_PROTECTION = 3
};

inline const char *adaptive_visual_mode_name(AdaptiveVisualMode mode) {
  switch (mode) {
  case AdaptiveVisualMode::BOOTSTRAP_DENSE:
    return "BOOTSTRAP_DENSE";
  case AdaptiveVisualMode::GEOMETRY_CONTROLLED:
    return "GEOMETRY_CONTROLLED";
  case AdaptiveVisualMode::TRACKING_PROTECTION:
    return "TRACKING_PROTECTION";
  case AdaptiveVisualMode::ESTIMATOR_PROTECTION:
    return "ESTIMATOR_PROTECTION";
  }
  return "UNKNOWN";
}

struct AdaptiveVisualSchedulerConfig {
  ContinuousTrackingPlannerConfig tracking;
  int minimum_active_features = 50;
  double maximum_visual_residual_p95_px = 12.0;
  double maximum_backend_starvation_s = 1.0;
  double maximum_visual_position_correction_m = 5.0;
  double maximum_visual_velocity_correction_mps = 3.0;
  double maximum_visual_attitude_correction_deg = 12.0;
};

struct AdaptiveVisualSchedulerInput {
  double timestamp = -1.0;
  bool initialized = false;
  double raw_camera_dt_s = 0.0;
  double actual_tracking_interval_s = 0.0;
  double motion_measurement_timestamp_s = -1.0;
  VisualMotionMetrics motion;
  GroundFootprintPredictionInput ground_footprint;
  int active_feature_count = 0;
  double visual_residual_p95_px =
      std::numeric_limits<double>::quiet_NaN();
  double time_since_accepted_backend_update_s = 0.0;
  bool covariance_all_finite = true;
  int covariance_negative_diagonal_count = 0;
  bool visual_state_correction_valid = false;
  double visual_position_correction_m = 0.0;
  double visual_velocity_correction_mps = 0.0;
  double visual_attitude_correction_deg = 0.0;
};

struct AdaptiveVisualSchedulerDecision {
  int tracking_gap = 1;
  int instantaneous_safe_gap = 1;
  AdaptiveVisualMode mode = AdaptiveVisualMode::BOOTSTRAP_DENSE;
  std::string mode_name = "BOOTSTRAP_DENSE";
  std::string reason = "bootstrap_dense";
  bool changed = false;
  bool immediate_contraction = false;
  bool expansion_confirmed = false;
  bool new_motion_measurement = false;
  bool estimator_protection = false;
  bool tracking_protection = false;
  double motion_age_s = std::numeric_limits<double>::infinity();
  ContinuousTrackingDecision tracking;
};

class AdaptiveVisualScheduler {
public:
  explicit AdaptiveVisualScheduler(
      const AdaptiveVisualSchedulerConfig &config =
          AdaptiveVisualSchedulerConfig())
      : config_(config), tracking_planner_(config.tracking) {}

  AdaptiveVisualSchedulerDecision
  update(const AdaptiveVisualSchedulerInput &input) {
    AdaptiveVisualSchedulerDecision out;
    const bool estimator_protection =
        input.initialized &&
        (!input.covariance_all_finite ||
         input.covariance_negative_diagonal_count > 0 ||
         (input.visual_state_correction_valid &&
          (input.visual_position_correction_m >
               config_.maximum_visual_position_correction_m ||
           input.visual_velocity_correction_mps >
               config_.maximum_visual_velocity_correction_mps ||
           input.visual_attitude_correction_deg >
               config_.maximum_visual_attitude_correction_deg)) ||
         (std::isfinite(input.visual_residual_p95_px) &&
          input.visual_residual_p95_px >
              config_.maximum_visual_residual_p95_px) ||
         input.time_since_accepted_backend_update_s >
             config_.maximum_backend_starvation_s);
    const bool tracking_protection =
        input.motion.valid &&
        (input.active_feature_count < config_.minimum_active_features ||
         input.motion.effective_track_count <
             config_.tracking.minimum_effective_tracks ||
         input.motion.grid_occupancy_ratio <
             config_.tracking.minimum_grid_occupancy ||
         input.motion.grid_entropy < config_.tracking.minimum_grid_entropy ||
         input.motion.border_track_ratio >
             config_.tracking.maximum_border_ratio);

    ContinuousTrackingPlannerInput tracking_input;
    tracking_input.timestamp = input.timestamp;
    tracking_input.motion_measurement_timestamp_s =
        input.motion_measurement_timestamp_s;
    tracking_input.raw_camera_dt_s = input.raw_camera_dt_s;
    tracking_input.motion = input.motion;
    tracking_input.ground_footprint = input.ground_footprint;
    tracking_input.force_dense = estimator_protection || tracking_protection;
    out.tracking = tracking_planner_.update(tracking_input);
    out.tracking_gap = out.tracking.tracking_gap;
    out.instantaneous_safe_gap = out.tracking.instantaneous_safe_gap;
    out.changed = out.tracking.changed;
    out.immediate_contraction = out.tracking.immediate_contraction;
    out.expansion_confirmed = out.tracking.expansion_confirmed;
    out.new_motion_measurement = out.tracking.new_motion_measurement;
    out.motion_age_s = out.tracking.motion_age_s;
    out.estimator_protection = estimator_protection;
    out.tracking_protection = tracking_protection;
    if (estimator_protection) {
      out.mode = AdaptiveVisualMode::ESTIMATOR_PROTECTION;
      out.reason = "estimator_protection";
    } else if (tracking_protection) {
      out.mode = AdaptiveVisualMode::TRACKING_PROTECTION;
      out.reason = "tracking_protection";
    } else if (!input.motion.valid) {
      out.mode = AdaptiveVisualMode::BOOTSTRAP_DENSE;
      out.reason = out.tracking.reason;
    } else {
      out.mode = AdaptiveVisualMode::GEOMETRY_CONTROLLED;
      out.reason = out.tracking.reason;
    }
    out.mode_name = adaptive_visual_mode_name(out.mode);
    return out;
  }

  int tracking_gap() const { return tracking_planner_.tracking_gap(); }

private:
  AdaptiveVisualSchedulerConfig config_;
  ContinuousTrackingPlanner tracking_planner_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ADAPTIVE_VISUAL_SCHEDULER_H
