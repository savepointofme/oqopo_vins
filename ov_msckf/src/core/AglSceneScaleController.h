/* Causal policy for estimator-consistent AGL scene-scale resets. */

#ifndef OV_MSCKF_AGL_SCENE_SCALE_CONTROLLER_H
#define OV_MSCKF_AGL_SCENE_SCALE_CONTROLLER_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ov_msckf {

/**
 * Physical and temporal gates for AGL scene-scale proposals.
 *
 * This policy has no estimator-state or GPS dependency. The caller supplies a
 * robust map height from visual geometry and a causal AGL value represented at
 * the same sample timestamp.
 */
struct AglSceneScaleControllerConfig {
  /// Required oldest-to-newest support in the retained sample window [s].
  double window_duration_s = 3.0;
  /// Largest permitted gap between adjacent accepted samples [s].
  double maximum_sample_gap_s = 0.75;
  /// Largest permitted age of the paired map-height/AGL sample [s].
  double maximum_sample_age_s = 0.25;
  /// Maximum MAD of log(agl_m / map_height_m) [natural-log scale].
  double maximum_log_ratio_mad = 0.02;
  /// Minimum symmetric multiplicative scale departure from one [fraction].
  double minimum_scale_deviation_fraction = 0.05;
  /// Minimum elapsed time after a successful reset before another proposal [s].
  double cooldown_duration_s = 5.0;
  /// Closed physical interval for one incremental scene-scale reset [ratio].
  double minimum_incremental_scale = 0.80;
  double maximum_incremental_scale = 1.25;
  /// Closed physical interval for the product of successful resets [ratio].
  double minimum_cumulative_scale = 0.50;
  double maximum_cumulative_scale = 2.00;
};

/**
 * One causal, time-paired height sample.
 *
 * `sample_timestamp_s` is shared by `map_height_m` and `agl_m`; it is not an
 * independently timestamped VIO vertical displacement. `causal_timestamp_s`
 * is the current processing time and must never precede the sample time.
 */
struct AglSceneScaleInput {
  double causal_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double sample_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double map_height_m = std::numeric_limits<double>::quiet_NaN();
  double agl_m = std::numeric_limits<double>::quiet_NaN();
  bool map_height_valid = false;
  bool agl_valid = false;
};

struct AglSceneScaleDecision {
  bool window_started = false;
  bool ready = false;
  bool reset_feedback_pending = false;
  double proposed_scale = std::numeric_limits<double>::quiet_NaN();
  double candidate_scale = std::numeric_limits<double>::quiet_NaN();
  double cumulative_scale = 1.0;
  double proposed_cumulative_scale =
      std::numeric_limits<double>::quiet_NaN();
  double coverage_s = 0.0;
  double max_gap_s = 0.0;
  double median_ratio = std::numeric_limits<double>::quiet_NaN();
  /// Median absolute deviation of log(agl_m / map_height_m).
  double mad = std::numeric_limits<double>::quiet_NaN();
  /// Multiplicative form of `mad`, e.g. 0.02 is approximately two percent.
  double mad_fraction = std::numeric_limits<double>::quiet_NaN();
  double scale_deviation_fraction =
      std::numeric_limits<double>::quiet_NaN();
  double sample_age_s = std::numeric_limits<double>::quiet_NaN();
  double cooldown_remaining_s = 0.0;
  std::string reason = "not_evaluated";
};

/** Result returned by the atomic StateHelper reset call. */
struct AglSceneScaleResetFeedback {
  bool success = false;
  double applied_scale = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Header-only policy layer for deciding when a Sim(3) scale reset is safe.
 *
 * It neither estimates visual ground geometry nor changes estimator state.
 * A successful proposal must be passed to StateHelper by the caller, followed
 * by exactly one `handle_reset_feedback()` call.
 */
class AglSceneScaleController {
public:
  explicit AglSceneScaleController(
      const AglSceneScaleControllerConfig &config =
          AglSceneScaleControllerConfig())
      : config_(config) {
    validate_config(config_);
  }

  AglSceneScaleDecision update(const AglSceneScaleInput &input) {
    if (!std::isfinite(input.causal_timestamp_s)) {
      auto out = diagnostics(last_causal_timestamp_s_);
      out.reason = "causal_timestamp_invalid";
      return out;
    }
    if (std::isfinite(last_causal_timestamp_s_) &&
        input.causal_timestamp_s <=
            last_causal_timestamp_s_ + kTimeToleranceS) {
      auto out = diagnostics(last_causal_timestamp_s_);
      out.reason = "causal_timestamp_non_monotonic";
      return out;
    }
    last_causal_timestamp_s_ = input.causal_timestamp_s;

    if (proposal_pending_) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.reason = "reset_feedback_pending";
      return out;
    }

    prune_window(input.causal_timestamp_s);

    if (!std::isfinite(input.sample_timestamp_s)) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.reason = "sample_timestamp_invalid";
      return out;
    }
    if (input.sample_timestamp_s >
        input.causal_timestamp_s + kTimeToleranceS) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.reason = "sample_timestamp_in_future";
      return out;
    }

    const double sample_age_s =
        std::max(0.0, input.causal_timestamp_s - input.sample_timestamp_s);
    if (sample_age_s > config_.maximum_sample_age_s + kTimeToleranceS) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "sample_stale";
      return out;
    }
    if (std::isfinite(last_sample_timestamp_s_) &&
        input.sample_timestamp_s <=
            last_sample_timestamp_s_ + kTimeToleranceS) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "sample_timestamp_non_monotonic";
      return out;
    }
    if (!input.map_height_valid || !std::isfinite(input.map_height_m)) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "map_height_invalid";
      return out;
    }
    if (input.map_height_m <= 0.0) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "map_height_non_positive";
      return out;
    }
    if (!input.agl_valid || !std::isfinite(input.agl_m)) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "agl_invalid";
      return out;
    }
    if (input.agl_m <= 0.0) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "agl_non_positive";
      return out;
    }

    const double ratio = input.agl_m / input.map_height_m;
    if (!std::isfinite(ratio) || ratio <= 0.0) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "ratio_invalid";
      return out;
    }
    const double log_ratio = std::log(ratio);
    if (!std::isfinite(log_ratio)) {
      auto out = diagnostics(input.causal_timestamp_s);
      out.sample_age_s = sample_age_s;
      out.reason = "ratio_invalid";
      return out;
    }

    window_.push_back({input.sample_timestamp_s, log_ratio});
    last_sample_timestamp_s_ = input.sample_timestamp_s;
    prune_window(input.causal_timestamp_s);

    auto out = diagnostics(input.causal_timestamp_s);
    out.sample_age_s = sample_age_s;
    if (out.cooldown_remaining_s > kTimeToleranceS) {
      out.reason = "cooldown_active";
      return out;
    }
    // Two endpoints are necessary to define elapsed coverage. There is no
    // sample-count release threshold; data rate determines the natural count.
    if (window_.size() < 2) {
      out.reason = "window_started";
      return out;
    }
    if (out.coverage_s + kTimeToleranceS < config_.window_duration_s) {
      out.reason = "window_coverage_insufficient";
      return out;
    }
    if (out.max_gap_s > config_.maximum_sample_gap_s + kTimeToleranceS) {
      out.reason = "window_gap_too_large";
      return out;
    }
    if (!std::isfinite(out.mad) ||
        out.mad > config_.maximum_log_ratio_mad + kScaleTolerance) {
      out.reason = "ratio_unstable";
      return out;
    }
    if (!std::isfinite(out.scale_deviation_fraction) ||
        out.scale_deviation_fraction + kScaleTolerance <
            config_.minimum_scale_deviation_fraction) {
      out.reason = "scale_deviation_below_threshold";
      return out;
    }

    // Safety bounds are fail-closed. In particular, an out-of-range scale is
    // never clipped into an apparently valid reset.
    if (outside_closed_interval(out.candidate_scale,
                                config_.minimum_incremental_scale,
                                config_.maximum_incremental_scale)) {
      out.reason = "incremental_scale_out_of_bounds";
      return out;
    }
    const double proposed_cumulative =
        cumulative_scale_ * out.candidate_scale;
    out.proposed_cumulative_scale = proposed_cumulative;
    if (!std::isfinite(proposed_cumulative) ||
        outside_closed_interval(proposed_cumulative,
                                config_.minimum_cumulative_scale,
                                config_.maximum_cumulative_scale)) {
      out.reason = "cumulative_scale_out_of_bounds";
      return out;
    }

    out.ready = true;
    out.proposed_scale = out.candidate_scale;
    out.reset_feedback_pending = true;
    out.reason = "scale_ready";
    proposal_pending_ = true;
    pending_scale_ = out.proposed_scale;
    pending_cumulative_scale_ = proposed_cumulative;
    pending_causal_timestamp_s_ = input.causal_timestamp_s;
    return out;
  }

  AglSceneScaleDecision handle_reset_feedback(
      const AglSceneScaleResetFeedback &feedback) {
    if (!proposal_pending_) {
      auto out = diagnostics(last_causal_timestamp_s_);
      out.reason = "reset_feedback_without_proposal";
      return out;
    }

    const double attempted_scale = pending_scale_;
    const double attempted_cumulative = pending_cumulative_scale_;
    const double proposal_timestamp_s = pending_causal_timestamp_s_;

    if (!feedback.success) {
      clear_pending_proposal();
      auto out = diagnostics(last_causal_timestamp_s_);
      out.proposed_scale = attempted_scale;
      out.proposed_cumulative_scale = attempted_cumulative;
      out.reason = "reset_failed";
      return out;
    }
    if (!std::isfinite(feedback.applied_scale) ||
        feedback.applied_scale <= 0.0) {
      clear_pending_proposal();
      auto out = diagnostics(last_causal_timestamp_s_);
      out.proposed_scale = attempted_scale;
      out.proposed_cumulative_scale = attempted_cumulative;
      out.reason = "reset_feedback_scale_invalid";
      return out;
    }
    if (!nearly_equal(feedback.applied_scale, attempted_scale)) {
      clear_pending_proposal();
      auto out = diagnostics(last_causal_timestamp_s_);
      out.proposed_scale = attempted_scale;
      out.proposed_cumulative_scale = attempted_cumulative;
      out.reason = "reset_feedback_scale_mismatch";
      return out;
    }

    const double applied_cumulative =
        cumulative_scale_ * feedback.applied_scale;
    if (!std::isfinite(applied_cumulative) ||
        outside_closed_interval(applied_cumulative,
                                config_.minimum_cumulative_scale,
                                config_.maximum_cumulative_scale)) {
      clear_pending_proposal();
      auto out = diagnostics(last_causal_timestamp_s_);
      out.proposed_scale = attempted_scale;
      out.proposed_cumulative_scale = attempted_cumulative;
      out.reason = "reset_feedback_cumulative_out_of_bounds";
      return out;
    }

    cumulative_scale_ = applied_cumulative;
    last_success_timestamp_s_ = proposal_timestamp_s;
    window_.clear();
    clear_pending_proposal();

    auto out = diagnostics(last_causal_timestamp_s_);
    out.proposed_scale = feedback.applied_scale;
    out.proposed_cumulative_scale = cumulative_scale_;
    out.reason = "reset_applied";
    return out;
  }

private:
  struct Sample {
    double timestamp_s = 0.0;
    double log_ratio = 0.0;
  };

  static constexpr double kTimeToleranceS = 1.0e-9;
  static constexpr double kScaleTolerance = 1.0e-12;
  static constexpr double kFeedbackRelativeTolerance = 1.0e-9;

  static void validate_config(const AglSceneScaleControllerConfig &config) {
    const bool finite =
        std::isfinite(config.window_duration_s) &&
        std::isfinite(config.maximum_sample_gap_s) &&
        std::isfinite(config.maximum_sample_age_s) &&
        std::isfinite(config.maximum_log_ratio_mad) &&
        std::isfinite(config.minimum_scale_deviation_fraction) &&
        std::isfinite(config.cooldown_duration_s) &&
        std::isfinite(config.minimum_incremental_scale) &&
        std::isfinite(config.maximum_incremental_scale) &&
        std::isfinite(config.minimum_cumulative_scale) &&
        std::isfinite(config.maximum_cumulative_scale);
    const bool physical =
        config.window_duration_s > 0.0 &&
        config.maximum_sample_gap_s > 0.0 &&
        config.maximum_sample_age_s >= 0.0 &&
        config.maximum_log_ratio_mad >= 0.0 &&
        config.minimum_scale_deviation_fraction >= 0.0 &&
        config.cooldown_duration_s >= 0.0 &&
        config.minimum_incremental_scale > 0.0 &&
        config.minimum_incremental_scale <= 1.0 &&
        config.maximum_incremental_scale >= 1.0 &&
        config.minimum_incremental_scale <=
            config.maximum_incremental_scale &&
        config.minimum_cumulative_scale > 0.0 &&
        config.minimum_cumulative_scale <= 1.0 &&
        config.maximum_cumulative_scale >= 1.0 &&
        config.minimum_cumulative_scale <= config.maximum_cumulative_scale;
    if (!finite || !physical)
      throw std::invalid_argument("invalid AGL scene-scale controller config");
  }

  static double median(std::vector<double> values) {
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 1)
      return values[middle];
    return 0.5 * (values[middle - 1] + values[middle]);
  }

  static bool outside_closed_interval(double value, double lower,
                                      double upper) {
    if (!std::isfinite(value))
      return true;
    const double tolerance =
        kScaleTolerance *
        std::max({1.0, std::fabs(value), std::fabs(lower), std::fabs(upper)});
    return value < lower - tolerance || value > upper + tolerance;
  }

  static bool nearly_equal(double lhs, double rhs) {
    return std::fabs(lhs - rhs) <=
           kFeedbackRelativeTolerance *
               std::max({1.0, std::fabs(lhs), std::fabs(rhs)});
  }

  void prune_window(double causal_timestamp_s) {
    const double cutoff_s =
        causal_timestamp_s - config_.window_duration_s;
    // Retain the final sample at or before the left boundary. This endpoint is
    // needed to prove real-time coverage; a large boundary gap is still
    // rejected by maximum_sample_gap_s.
    while (window_.size() >= 2 &&
           window_[1].timestamp_s <= cutoff_s + kTimeToleranceS)
      window_.pop_front();
  }

  AglSceneScaleDecision diagnostics(double causal_timestamp_s) const {
    AglSceneScaleDecision out;
    out.window_started = !window_.empty();
    out.reset_feedback_pending = proposal_pending_;
    out.cumulative_scale = cumulative_scale_;

    if (std::isfinite(causal_timestamp_s) &&
        std::isfinite(last_success_timestamp_s_)) {
      const double elapsed_s =
          causal_timestamp_s - last_success_timestamp_s_;
      out.cooldown_remaining_s =
          std::max(0.0, config_.cooldown_duration_s - elapsed_s);
    }
    if (window_.empty())
      return out;

    out.coverage_s =
        std::max(0.0, window_.back().timestamp_s -
                          window_.front().timestamp_s);
    std::vector<double> log_ratios;
    log_ratios.reserve(window_.size());
    for (std::size_t index = 0; index < window_.size(); ++index) {
      log_ratios.push_back(window_[index].log_ratio);
      if (index > 0) {
        out.max_gap_s =
            std::max(out.max_gap_s,
                     window_[index].timestamp_s -
                         window_[index - 1].timestamp_s);
      }
    }

    const double median_log_ratio = median(log_ratios);
    out.median_ratio = std::exp(median_log_ratio);
    out.candidate_scale = out.median_ratio;
    std::vector<double> absolute_deviations;
    absolute_deviations.reserve(log_ratios.size());
    for (double value : log_ratios)
      absolute_deviations.push_back(std::fabs(value - median_log_ratio));
    out.mad = median(std::move(absolute_deviations));
    out.mad_fraction = std::expm1(out.mad);
    out.scale_deviation_fraction =
        std::expm1(std::fabs(median_log_ratio));
    return out;
  }

  void clear_pending_proposal() {
    proposal_pending_ = false;
    pending_scale_ = std::numeric_limits<double>::quiet_NaN();
    pending_cumulative_scale_ =
        std::numeric_limits<double>::quiet_NaN();
    pending_causal_timestamp_s_ =
        std::numeric_limits<double>::quiet_NaN();
  }

  AglSceneScaleControllerConfig config_;
  std::deque<Sample> window_;
  double cumulative_scale_ = 1.0;
  double last_causal_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_sample_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_success_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  bool proposal_pending_ = false;
  double pending_scale_ = std::numeric_limits<double>::quiet_NaN();
  double pending_cumulative_scale_ =
      std::numeric_limits<double>::quiet_NaN();
  double pending_causal_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
};

} // namespace ov_msckf

#endif // OV_MSCKF_AGL_SCENE_SCALE_CONTROLLER_H
