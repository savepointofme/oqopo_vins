/* P5 backend scheduler driven by direct geometry since the last actual clone. */

#ifndef OV_MSCKF_BACKEND_UPDATE_TRIGGER_H
#define OV_MSCKF_BACKEND_UPDATE_TRIGGER_H

#include "VisualCadencePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>

namespace ov_msckf {

struct AdaptiveBackendSchedulerConfig {
  int minimum_common_tracks = 20;
  double minimum_effective_tracks = 15.0;
  double minimum_survival_ratio = 0.45;
  double minimum_grid_occupancy = 0.25;
  double minimum_grid_entropy = 0.35;
  double maximum_border_ratio = 0.75;
  double termination_border_margin_fraction = 0.05;
  // Backend geometry should span more baseline than one nominal KLT step.
  double minimum_compensated_median_px = 10.0;
  double maximum_compensated_p95_ucb_px = 30.0;
  double minimum_information_score = 8.0;
  double pure_rotation_minimum_rotation_px = 5.0;
  double pure_rotation_maximum_compensated_px = 1.5;
  double pure_rotation_maximum_ratio = 0.25;
  // Preserve real-time support in the bounded clone window. This duration is
  // divided by the configured clone capacity; it is not a frame stride.
  double target_clone_window_duration_s = 4.0;
  double maximum_clone_latency_s = 0.50;
};

struct AdaptiveBackendSchedulerInput {
  bool initialized = false;
  double timestamp = -1.0;
  double last_actual_clone_timestamp = -1.0;
  int clone_capacity = 1;
  VisualMotionMetrics motion_from_last_clone;
  bool feature_termination_imminent = false;
};

struct AdaptiveBackendDecision {
  bool trigger = false;
  bool bootstrap_trigger = false;
  bool information_trigger = false;
  bool latency_trigger = false;
  bool termination_trigger = false;
  bool pure_rotation = false;
  bool visual_geometry_valid = false;
  bool forced_without_visual_information = false;
  double elapsed_since_clone_s = 0.0;
  double minimum_temporal_separation_s = 0.0;
  bool temporal_support_ready = false;
  double information_score = 0.0;
  double translation_baseline_px = 0.0;
  double translation_p95_ucb_px = 0.0;
  double effective_tracks = 0.0;
  double survival_ratio = 0.0;
  double grid_occupancy = 0.0;
  std::string reason = "not_initialized";
};

inline bool backend_reference_commit_allowed(bool trigger,
                                             bool snapshot_valid,
                                             bool clone_created) {
  return trigger && snapshot_valid && clone_created;
}

inline bool backend_feature_termination_imminent(
    const VisualFrameSnapshot &reference,
    const VisualFrameSnapshot &current, int image_width, int image_height,
    const AdaptiveBackendSchedulerConfig &config) {
  if (image_width <= 0 || image_height <= 0)
    return false;
  std::unordered_set<size_t> reference_ids;
  for (const VisualTrackPoint &track : reference.tracks)
    if (track.valid)
      reference_ids.insert(track.feature_id);
  if (reference_ids.empty())
    return false;

  const double margin_x =
      config.termination_border_margin_fraction * image_width;
  const double margin_y =
      config.termination_border_margin_fraction * image_height;
  int current_count = 0;
  int common_count = 0;
  int border_count = 0;
  for (const VisualTrackPoint &track : current.tracks) {
    if (!track.valid)
      continue;
    ++current_count;
    common_count += reference_ids.count(track.feature_id) > 0 ? 1 : 0;
    if (track.raw.allFinite() &&
        (track.raw.x() <= margin_x ||
         track.raw.x() >= image_width - margin_x ||
         track.raw.y() <= margin_y ||
         track.raw.y() >= image_height - margin_y))
      ++border_count;
  }
  const double survival_ratio =
      static_cast<double>(common_count) /
      static_cast<double>(reference_ids.size());
  const double border_ratio =
      current_count > 0
          ? static_cast<double>(border_count) / current_count
          : 1.0;
  return current_count < config.minimum_common_tracks ||
         common_count < config.minimum_common_tracks ||
         survival_ratio < config.minimum_survival_ratio ||
         border_ratio > config.maximum_border_ratio;
}

class AdaptiveBackendScheduler {
public:
  explicit AdaptiveBackendScheduler(
      const AdaptiveBackendSchedulerConfig &config =
          AdaptiveBackendSchedulerConfig())
      : config_(config) {}

  const AdaptiveBackendSchedulerConfig &config() const { return config_; }

  AdaptiveBackendDecision
  evaluate(const AdaptiveBackendSchedulerInput &input) const {
    AdaptiveBackendDecision out;
    if (!input.initialized || !std::isfinite(input.timestamp))
      return out;

    if (!(input.last_actual_clone_timestamp >= 0.0)) {
      out.trigger = true;
      out.bootstrap_trigger = true;
      out.reason = "bootstrap_actual_clone_reference";
      return out;
    }
    out.elapsed_since_clone_s =
        std::max(0.0, input.timestamp - input.last_actual_clone_timestamp);
    out.minimum_temporal_separation_s =
        config_.target_clone_window_duration_s /
        static_cast<double>(std::max(1, input.clone_capacity));
    out.temporal_support_ready =
        out.elapsed_since_clone_s + 1e-9 >=
        out.minimum_temporal_separation_s;
    const VisualMotionMetrics &motion = input.motion_from_last_clone;
    out.visual_geometry_valid = motion.valid;

    if (motion.valid) {
      out.translation_baseline_px = motion.compensated_median_px;
      out.translation_p95_ucb_px = motion.compensated_p95_ucb_px;
      out.effective_tracks = motion.effective_track_count;
      out.survival_ratio = motion.survival_ratio;
      out.grid_occupancy = motion.grid_occupancy_ratio;
      const double uncertainty =
          std::max(0.25, motion.compensated_sigma_px);
      const double signal_to_noise =
          motion.compensated_median_px * motion.compensated_median_px /
          (uncertainty * uncertainty);
      const double bounded_signal = signal_to_noise / (1.0 + signal_to_noise);
      out.information_score =
          motion.effective_track_count * motion.grid_occupancy_ratio *
          motion.grid_entropy * motion.survival_ratio * bounded_signal;
      out.pure_rotation =
          motion.rotation_median_px >=
              config_.pure_rotation_minimum_rotation_px &&
          motion.compensated_median_px <=
              config_.pure_rotation_maximum_compensated_px &&
          motion.compensated_median_px <=
              config_.pure_rotation_maximum_ratio *
                  std::max(1e-6, motion.rotation_median_px);
    }

    const bool termination_risk =
        input.feature_termination_imminent ||
        (motion.valid &&
         (motion.common_tracks < config_.minimum_common_tracks ||
          motion.survival_ratio < config_.minimum_survival_ratio ||
          motion.border_track_ratio > config_.maximum_border_ratio));
    // A weak/terminating track set is useful only after the clone window has
    // accumulated real temporal support. Triggering immediately on every
    // low-track frame collapses the clone span to the camera period and feeds
    // repeated poorly-conditioned updates to the backend.
    if (termination_risk && out.temporal_support_ready) {
      out.trigger = true;
      out.termination_trigger = true;
      out.forced_without_visual_information = !motion.valid;
      out.reason = "feature_termination_risk";
      return out;
    }

    if (out.elapsed_since_clone_s >= config_.maximum_clone_latency_s) {
      out.trigger = true;
      out.latency_trigger = true;
      out.forced_without_visual_information =
          !motion.valid || out.pure_rotation;
      out.reason = out.pure_rotation ? "latency_during_pure_rotation"
                                     : "maximum_clone_latency";
      return out;
    }

    if (termination_risk) {
      out.reason = "accumulate_termination_temporal_support";
      return out;
    }

    if (!motion.valid) {
      out.reason = "await_direct_geometry";
      return out;
    }
    if (out.pure_rotation) {
      out.reason = "accumulate_through_pure_rotation";
      return out;
    }

    const bool geometry_healthy =
        motion.common_tracks >= config_.minimum_common_tracks &&
        motion.effective_track_count >= config_.minimum_effective_tracks &&
        motion.survival_ratio >= config_.minimum_survival_ratio &&
        motion.grid_occupancy_ratio >= config_.minimum_grid_occupancy &&
        motion.grid_entropy >= config_.minimum_grid_entropy &&
        motion.border_track_ratio <= config_.maximum_border_ratio &&
        motion.compensated_p95_ucb_px <=
            config_.maximum_compensated_p95_ucb_px;
    const bool baseline_ready =
        motion.compensated_median_px >=
        config_.minimum_compensated_median_px;
    if (geometry_healthy && baseline_ready && out.temporal_support_ready &&
        out.information_score >= config_.minimum_information_score) {
      out.trigger = true;
      out.information_trigger = true;
      out.reason = "translation_information_ready";
      return out;
    }

    if (geometry_healthy && baseline_ready && !out.temporal_support_ready)
      out.reason = "accumulate_temporal_window_support";
    else
      out.reason = geometry_healthy ? "accumulate_translation_baseline"
                                    : "accumulate_visual_support";
    return out;
  }

private:
  AdaptiveBackendSchedulerConfig config_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_BACKEND_UPDATE_TRIGGER_H
