/* Causal visual-motion statistics and continuous-horizon tracking planner. */

#ifndef OV_MSCKF_VISUAL_CADENCE_PLANNER_H
#define OV_MSCKF_VISUAL_CADENCE_PLANNER_H

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace ov_msckf {

struct VisualTrackPoint {
  size_t feature_id = 0;
  Eigen::Vector2d raw = Eigen::Vector2d::Zero();
  Eigen::Vector2d normalized = Eigen::Vector2d::Zero();
  int track_age = 0;
  bool valid = false;
};

struct VisualFrameSnapshot {
  double timestamp = -1.0;
  std::vector<VisualTrackPoint> tracks;
};

struct VisualMotionMetrics {
  double dt_s = 0.0;
  int previous_tracks = 0;
  int current_tracks = 0;
  int common_tracks = 0;
  double effective_track_count = 0.0;
  double survival_ratio = 0.0;
  double median_track_age = 0.0;
  double grid_occupancy_ratio = 0.0;
  double grid_entropy = 0.0;
  double border_track_ratio = 0.0;
  double raw_median_px = 0.0;
  double raw_p75_px = 0.0;
  double raw_p90_px = 0.0;
  double raw_p95_px = 0.0;
  double rotation_median_px = 0.0;
  double rotation_p95_px = 0.0;
  double rotation_max_px = 0.0;
  double compensated_median_px = 0.0;
  double compensated_p75_px = 0.0;
  double compensated_p90_px = 0.0;
  double compensated_p95_px = 0.0;
  double compensated_mad_px = 0.0;
  double compensated_sigma_px = 0.0;
  double compensated_p95_ucb_px = 0.0;
  bool valid = false;
};

struct GroundFootprintPredictionInput {
  Eigen::Matrix3d R_GtoC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_CinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_G = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 4> corner_rays_C;
  double ground_height_G = 0.0;
  bool valid = false;
};

inline double visual_polygon_signed_area(
    const std::vector<Eigen::Vector2d> &polygon) {
  if (polygon.size() < 3)
    return 0.0;
  double area = 0.0;
  for (size_t index = 0; index < polygon.size(); ++index) {
    const Eigen::Vector2d &a = polygon[index];
    const Eigen::Vector2d &b = polygon[(index + 1) % polygon.size()];
    area += a.x() * b.y() - a.y() * b.x();
  }
  return 0.5 * area;
}

inline Eigen::Vector2d visual_line_intersection(
    const Eigen::Vector2d &p, const Eigen::Vector2d &q,
    const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
  const Eigen::Vector2d r = q - p;
  const Eigen::Vector2d s = b - a;
  const double denominator = r.x() * s.y() - r.y() * s.x();
  if (std::fabs(denominator) < 1e-12)
    return q;
  const Eigen::Vector2d ap = a - p;
  const double fraction =
      (ap.x() * s.y() - ap.y() * s.x()) / denominator;
  return p + fraction * r;
}

inline std::vector<Eigen::Vector2d> visual_clip_convex_polygon(
    std::vector<Eigen::Vector2d> subject,
    std::vector<Eigen::Vector2d> clip) {
  if (visual_polygon_signed_area(clip) < 0.0)
    std::reverse(clip.begin(), clip.end());
  for (size_t edge = 0; edge < clip.size() && !subject.empty(); ++edge) {
    const Eigen::Vector2d a = clip[edge];
    const Eigen::Vector2d b = clip[(edge + 1) % clip.size()];
    const auto inside = [&](const Eigen::Vector2d &point) {
      const Eigen::Vector2d e = b - a;
      const Eigen::Vector2d d = point - a;
      return e.x() * d.y() - e.y() * d.x() >= -1e-9;
    };
    std::vector<Eigen::Vector2d> output;
    Eigen::Vector2d previous = subject.back();
    bool previous_inside = inside(previous);
    for (const Eigen::Vector2d &current : subject) {
      const bool current_inside = inside(current);
      if (current_inside != previous_inside)
        output.push_back(
            visual_line_intersection(previous, current, a, b));
      if (current_inside)
        output.push_back(current);
      previous = current;
      previous_inside = current_inside;
    }
    subject.swap(output);
  }
  return subject;
}

/// Causal ground-footprint overlap. Height only enters through this geometry;
/// it never maps directly to a fixed tracking or backend cadence.
inline double ground_footprint_overlap(
    const GroundFootprintPredictionInput &input, double horizon_s) {
  if (!input.valid || !input.R_GtoC.allFinite() ||
      !input.p_CinG.allFinite() || !input.velocity_G.allFinite() ||
      !(horizon_s >= 0.0) || !std::isfinite(horizon_s))
    return std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector2d> footprint;
  footprint.reserve(4);
  for (const Eigen::Vector3d &corner_ray_C : input.corner_rays_C) {
    const Eigen::Vector3d direction_G =
        input.R_GtoC.transpose() * corner_ray_C;
    if (!direction_G.allFinite() || std::fabs(direction_G.z()) < 1e-8)
      return std::numeric_limits<double>::quiet_NaN();
    const double scale =
        (input.ground_height_G - input.p_CinG.z()) / direction_G.z();
    if (!(scale > 0.0) || !std::isfinite(scale))
      return std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d point_G = input.p_CinG + scale * direction_G;
    footprint.emplace_back(point_G.x(), point_G.y());
  }
  if (visual_polygon_signed_area(footprint) < 0.0)
    std::reverse(footprint.begin(), footprint.end());
  const double area = std::fabs(visual_polygon_signed_area(footprint));
  if (!(area > 1e-6) || !std::isfinite(area))
    return std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector2d> predicted = footprint;
  const Eigen::Vector2d shift = input.velocity_G.head<2>() * horizon_s;
  for (Eigen::Vector2d &point : predicted)
    point += shift;
  const std::vector<Eigen::Vector2d> intersection =
      visual_clip_convex_polygon(footprint, predicted);
  const double intersection_area =
      intersection.size() >= 3
          ? std::fabs(visual_polygon_signed_area(intersection))
          : 0.0;
  const double union_area = 2.0 * area - intersection_area;
  return union_area > 1e-9 ? intersection_area / union_area
                           : std::numeric_limits<double>::quiet_NaN();
}

inline double visual_percentile(std::vector<double> values, double quantile) {
  if (values.empty())
    return 0.0;
  quantile = std::max(0.0, std::min(1.0, quantile));
  const double index = quantile * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(index));
  const size_t upper = static_cast<size_t>(std::ceil(index));
  std::nth_element(values.begin(), values.begin() + lower, values.end());
  const double lo = values[lower];
  if (upper == lower)
    return lo;
  std::nth_element(values.begin(), values.begin() + upper, values.end());
  return lo + (index - static_cast<double>(lower)) * (values[upper] - lo);
}

inline double visual_effective_sample_count(const std::vector<double> &weights) {
  double sum = 0.0;
  double squared_sum = 0.0;
  for (double weight : weights) {
    if (!(weight > 0.0) || !std::isfinite(weight))
      continue;
    sum += weight;
    squared_sum += weight * weight;
  }
  return squared_sum > 1e-12 ? sum * sum / squared_sum : 0.0;
}

inline void finalize_visual_motion_metrics(
    VisualMotionMetrics &out, const std::vector<double> &raw_motion,
    const std::vector<double> &rotation_motion,
    const std::vector<double> &compensated_motion,
    const std::vector<double> &ages, const std::vector<double> &weights,
    const std::vector<Eigen::Vector2d> &current_pixels, int image_width = 0,
    int image_height = 0) {
  out.common_tracks = static_cast<int>(compensated_motion.size());
  const int denominator = std::max(1, std::min(out.previous_tracks,
                                               out.current_tracks));
  out.survival_ratio =
      static_cast<double>(out.common_tracks) / static_cast<double>(denominator);
  out.effective_track_count = visual_effective_sample_count(weights);
  out.median_track_age = visual_percentile(ages, 0.5);
  out.raw_median_px = visual_percentile(raw_motion, 0.5);
  out.raw_p75_px = visual_percentile(raw_motion, 0.75);
  out.raw_p90_px = visual_percentile(raw_motion, 0.90);
  out.raw_p95_px = visual_percentile(raw_motion, 0.95);
  out.rotation_median_px = visual_percentile(rotation_motion, 0.5);
  out.rotation_p95_px = visual_percentile(rotation_motion, 0.95);
  out.rotation_max_px = visual_percentile(rotation_motion, 1.0);
  out.compensated_median_px = visual_percentile(compensated_motion, 0.5);
  out.compensated_p75_px = visual_percentile(compensated_motion, 0.75);
  out.compensated_p90_px = visual_percentile(compensated_motion, 0.90);
  out.compensated_p95_px = visual_percentile(compensated_motion, 0.95);

  std::vector<double> absolute_deviation;
  absolute_deviation.reserve(compensated_motion.size());
  for (double value : compensated_motion)
    absolute_deviation.push_back(std::fabs(value - out.compensated_median_px));
  out.compensated_mad_px = visual_percentile(absolute_deviation, 0.5);
  out.compensated_sigma_px = 1.4826 * out.compensated_mad_px;
  const double sample_scale =
      std::sqrt(std::max(1.0, out.effective_track_count));
  out.compensated_p95_ucb_px =
      out.compensated_p95_px + 1.96 * out.compensated_sigma_px / sample_scale;

  if (image_width > 0 && image_height > 0 && !current_pixels.empty()) {
    constexpr int grid_columns = 4;
    constexpr int grid_rows = 3;
    constexpr int grid_cells = grid_columns * grid_rows;
    std::array<double, grid_cells> cell_weights{};
    double total_weight = 0.0;
    int border_count = 0;
    for (size_t index = 0; index < current_pixels.size(); ++index) {
      const Eigen::Vector2d &pixel = current_pixels[index];
      const double nx = pixel.x() / static_cast<double>(image_width);
      const double ny = pixel.y() / static_cast<double>(image_height);
      const int column = std::max(
          0, std::min(grid_columns - 1, static_cast<int>(nx * grid_columns)));
      const int row = std::max(
          0, std::min(grid_rows - 1, static_cast<int>(ny * grid_rows)));
      const double weight = index < weights.size() ? weights[index] : 1.0;
      cell_weights[row * grid_columns + column] += weight;
      total_weight += weight;
      const double border_distance =
          std::min(std::min(nx, 1.0 - nx), std::min(ny, 1.0 - ny));
      if (border_distance < 0.08)
        ++border_count;
    }
    int occupied = 0;
    double entropy = 0.0;
    for (double weight : cell_weights) {
      if (weight <= 0.0)
        continue;
      ++occupied;
      const double probability = weight / std::max(1e-12, total_weight);
      entropy -= probability * std::log(probability);
    }
    out.grid_occupancy_ratio =
        static_cast<double>(occupied) / static_cast<double>(grid_cells);
    out.grid_entropy = entropy / std::log(static_cast<double>(grid_cells));
    out.border_track_ratio = static_cast<double>(border_count) /
                             static_cast<double>(current_pixels.size());
  } else {
    // Geometry consumers that do not own image dimensions must not invent a
    // spatial failure. The runner and VioManager always provide dimensions.
    out.grid_occupancy_ratio = 1.0;
    out.grid_entropy = 1.0;
    out.border_track_ratio = 0.0;
  }
  out.valid = out.common_tracks >= 4 && out.dt_s > 0.0 &&
              std::isfinite(out.dt_s);
}

/// R_Ccurrent_Cprevious rotates a previous-camera bearing into current-camera
/// coordinates. fx/fy convert normalized displacement to pixels.
inline VisualMotionMetrics compute_visual_motion_metrics(
    const VisualFrameSnapshot &previous, const VisualFrameSnapshot &current,
    const Eigen::Matrix3d &R_Ccurrent_Cprevious, double fx, double fy,
    int image_width = 0, int image_height = 0) {
  VisualMotionMetrics out;
  out.dt_s = current.timestamp - previous.timestamp;
  out.previous_tracks = static_cast<int>(previous.tracks.size());
  out.current_tracks = static_cast<int>(current.tracks.size());
  if (!(out.dt_s > 0.0) || !std::isfinite(out.dt_s) || !(fx > 0.0) ||
      !(fy > 0.0) || !R_Ccurrent_Cprevious.allFinite())
    return out;

  std::map<size_t, const VisualTrackPoint *> prior;
  for (const auto &track : previous.tracks)
    if (track.valid && track.raw.allFinite() && track.normalized.allFinite())
      prior[track.feature_id] = &track;

  std::vector<double> raw_motion;
  std::vector<double> rotation_motion;
  std::vector<double> compensated_motion;
  std::vector<double> ages;
  std::vector<double> weights;
  std::vector<Eigen::Vector2d> current_pixels;
  for (const auto &track : current.tracks) {
    if (!track.valid || !track.raw.allFinite() || !track.normalized.allFinite())
      continue;
    const auto found = prior.find(track.feature_id);
    if (found == prior.end())
      continue;
    const VisualTrackPoint &old = *found->second;
    Eigen::Vector3d old_ray(old.normalized.x(), old.normalized.y(), 1.0);
    const Eigen::Vector3d predicted = R_Ccurrent_Cprevious * old_ray;
    if (!(predicted.z() > 1e-8) || !predicted.allFinite())
      continue;
    const Eigen::Vector2d predicted_norm = predicted.head<2>() / predicted.z();
    const Eigen::Vector2d raw_delta = track.raw - old.raw;
    const Eigen::Vector2d rotation_delta(
        fx * (predicted_norm.x() - old.normalized.x()),
        fy * (predicted_norm.y() - old.normalized.y()));
    const Eigen::Vector2d compensated_delta(
        fx * (track.normalized.x() - predicted_norm.x()),
        fy * (track.normalized.y() - predicted_norm.y()));
    raw_motion.push_back(raw_delta.norm());
    rotation_motion.push_back(rotation_delta.norm());
    compensated_motion.push_back(compensated_delta.norm());
    ages.push_back(static_cast<double>(track.track_age));
    double border_weight = 1.0;
    if (image_width > 0 && image_height > 0) {
      const double nx = track.raw.x() / static_cast<double>(image_width);
      const double ny = track.raw.y() / static_cast<double>(image_height);
      const double border_distance =
          std::min(std::min(nx, 1.0 - nx), std::min(ny, 1.0 - ny));
      border_weight = std::max(0.20, std::min(1.0, border_distance / 0.08));
    }
    const double age_weight =
        std::max(0.20, std::min(1.0, static_cast<double>(track.track_age) / 5.0));
    weights.push_back(age_weight * border_weight);
    current_pixels.push_back(track.raw);
  }
  finalize_visual_motion_metrics(out, raw_motion, rotation_motion,
                                 compensated_motion, ages, weights,
                                 current_pixels, image_width, image_height);
  return out;
}

struct ContinuousTrackingPlannerConfig {
  int minimum_gap = 1;
  int maximum_gap = 12;
  // Use a conservative fraction of the complete continuous safety horizon.
  // The horizon already combines KLT displacement, rotation, survival,
  // interval and ground-footprint overlap constraints, so no fixed pixel
  // target or discrete cadence table is needed.
  double safe_horizon_utilization = 0.50;
  double maximum_compensated_p95_ucb_px = 18.0;
  double maximum_total_p95_px = 35.0;
  double maximum_rotation_p95_px = 28.0;
  double maximum_rotation_edge_px = 42.0;
  double minimum_survival_ratio = 0.55;
  double minimum_effective_tracks = 20.0;
  double minimum_grid_occupancy = 0.25;
  double minimum_grid_entropy = 0.35;
  double maximum_border_ratio = 0.75;
  double maximum_tracking_interval_s = 0.40;
  double minimum_ground_footprint_overlap = 0.85;
  double motion_stale_timeout_s = 0.80;
  double upshift_confirmation_duration_s = 0.50;
};

struct ContinuousTrackingPlannerInput {
  double timestamp = -1.0;
  double motion_measurement_timestamp_s = -1.0;
  double raw_camera_dt_s = 0.0;
  VisualMotionMetrics motion;
  GroundFootprintPredictionInput ground_footprint;
  bool force_dense = false;
};

struct ContinuousTrackingDecision {
  int tracking_gap = 1;
  int instantaneous_safe_gap = 1;
  double target_horizon_s = 0.0;
  double safe_horizon_s = 0.0;
  double interval_horizon_s = 0.0;
  double compensated_horizon_s = 0.0;
  double raw_horizon_s = 0.0;
  double rotation_horizon_s = 0.0;
  double survival_horizon_s = 0.0;
  double ground_overlap_horizon_s =
      std::numeric_limits<double>::infinity();
  double predicted_ground_overlap =
      std::numeric_limits<double>::quiet_NaN();
  double predicted_compensated_median_px = 0.0;
  double predicted_compensated_p95_ucb_px = 0.0;
  double predicted_total_p95_px = 0.0;
  double predicted_rotation_p95_px = 0.0;
  double motion_age_s = std::numeric_limits<double>::infinity();
  bool changed = false;
  bool immediate_contraction = false;
  bool expansion_confirmed = false;
  bool new_motion_measurement = false;
  bool ground_geometry_valid = false;
  std::string reason = "bootstrap_dense";
};

class ContinuousTrackingPlanner {
public:
  explicit ContinuousTrackingPlanner(
      const ContinuousTrackingPlannerConfig &config =
          ContinuousTrackingPlannerConfig())
      : config_(config) {
    config_.minimum_gap = std::max(1, config_.minimum_gap);
    config_.maximum_gap =
        std::max(config_.minimum_gap, config_.maximum_gap);
    config_.safe_horizon_utilization =
        std::max(0.05, std::min(1.0, config_.safe_horizon_utilization));
    current_gap_ = config_.minimum_gap;
  }

  ContinuousTrackingDecision update(
      const ContinuousTrackingPlannerInput &input) {
    ContinuousTrackingDecision out;
    const double measurement_time =
        std::isfinite(input.motion_measurement_timestamp_s)
            ? input.motion_measurement_timestamp_s
            : std::numeric_limits<double>::quiet_NaN();
    out.new_motion_measurement =
        std::isfinite(measurement_time) &&
        (!std::isfinite(last_motion_timestamp_s_) ||
         measurement_time > last_motion_timestamp_s_ + 1e-9);
    if (out.new_motion_measurement)
      last_motion_timestamp_s_ = measurement_time;
    if (std::isfinite(last_motion_timestamp_s_) &&
        std::isfinite(input.timestamp))
      out.motion_age_s =
          std::max(0.0, input.timestamp - last_motion_timestamp_s_);

    const bool stale = out.motion_age_s > config_.motion_stale_timeout_s;
    const bool poor_spatial_support =
        input.motion.valid &&
        (input.motion.effective_track_count < config_.minimum_effective_tracks ||
         input.motion.grid_occupancy_ratio < config_.minimum_grid_occupancy ||
         input.motion.grid_entropy < config_.minimum_grid_entropy ||
         input.motion.border_track_ratio > config_.maximum_border_ratio);
    if (input.force_dense || stale || poor_spatial_support ||
        !input.motion.valid || !(input.raw_camera_dt_s > 0.0)) {
      const char *reason = input.force_dense
                               ? "estimator_protection"
                               : stale ? "motion_stale"
                               : poor_spatial_support
                                     ? "insufficient_spatial_support"
                                     : "bootstrap_dense";
      apply_gap(config_.minimum_gap, reason, out, true);
      return out;
    }

    if (!out.new_motion_measurement) {
      out.tracking_gap = current_gap_;
      out.instantaneous_safe_gap = current_gap_;
      out.reason = "hold_until_new_motion_measurement";
      return out;
    }

    const double measured_dt = std::max(1e-6, input.motion.dt_s);
    const auto horizon_for_limit = [measured_dt](double limit,
                                                  double measurement) {
      if (!(measurement > 1e-9) || !std::isfinite(measurement))
        return std::numeric_limits<double>::infinity();
      return limit * measured_dt / measurement;
    };
    out.interval_horizon_s = config_.maximum_tracking_interval_s;
    out.compensated_horizon_s = horizon_for_limit(
        config_.maximum_compensated_p95_ucb_px,
        input.motion.compensated_p95_ucb_px);
    out.raw_horizon_s = horizon_for_limit(config_.maximum_total_p95_px,
                                          input.motion.raw_p95_px);
    const double rotation_p95_horizon = horizon_for_limit(
        config_.maximum_rotation_p95_px, input.motion.rotation_p95_px);
    const double rotation_edge_horizon = horizon_for_limit(
        config_.maximum_rotation_edge_px, input.motion.rotation_max_px);
    out.rotation_horizon_s =
        std::min(rotation_p95_horizon, rotation_edge_horizon);
    if (input.motion.survival_ratio >= 0.999999) {
      out.survival_horizon_s = std::numeric_limits<double>::infinity();
    } else {
      const double survival =
          std::max(1e-4, std::min(0.999999, input.motion.survival_ratio));
      const double hazard = -std::log(survival) / measured_dt;
      out.survival_horizon_s =
          -std::log(config_.minimum_survival_ratio) / std::max(1e-9, hazard);
    }
    if (input.ground_footprint.valid) {
      const double maximum_geometry_horizon =
          std::min(config_.maximum_tracking_interval_s,
                   input.raw_camera_dt_s * config_.maximum_gap);
      const double overlap_at_minimum = ground_footprint_overlap(
          input.ground_footprint, input.raw_camera_dt_s);
      const double overlap_at_maximum = ground_footprint_overlap(
          input.ground_footprint, maximum_geometry_horizon);
      if (std::isfinite(overlap_at_minimum) &&
          std::isfinite(overlap_at_maximum)) {
        out.ground_geometry_valid = true;
        if (overlap_at_maximum >=
            config_.minimum_ground_footprint_overlap) {
          out.ground_overlap_horizon_s = maximum_geometry_horizon;
        } else if (overlap_at_minimum <
                   config_.minimum_ground_footprint_overlap) {
          out.ground_overlap_horizon_s = input.raw_camera_dt_s;
        } else {
          double lower = input.raw_camera_dt_s;
          double upper = maximum_geometry_horizon;
          for (int iteration = 0; iteration < 32; ++iteration) {
            const double middle = 0.5 * (lower + upper);
            const double overlap =
                ground_footprint_overlap(input.ground_footprint, middle);
            if (std::isfinite(overlap) &&
                overlap >= config_.minimum_ground_footprint_overlap)
              lower = middle;
            else
              upper = middle;
          }
          out.ground_overlap_horizon_s = lower;
        }
      }
    }
    out.safe_horizon_s =
        std::min(std::min(out.interval_horizon_s, out.compensated_horizon_s),
                 std::min(std::min(out.raw_horizon_s, out.rotation_horizon_s),
                          std::min(out.survival_horizon_s,
                                   out.ground_overlap_horizon_s)));
    out.target_horizon_s =
        config_.safe_horizon_utilization * out.safe_horizon_s;
    const double requested_horizon =
        std::max(input.raw_camera_dt_s,
                 std::min(out.target_horizon_s, out.safe_horizon_s));
    int desired_gap = static_cast<int>(
        std::floor(requested_horizon / input.raw_camera_dt_s + 1e-9));
    desired_gap = std::max(config_.minimum_gap,
                           std::min(config_.maximum_gap, desired_gap));
    out.instantaneous_safe_gap = desired_gap;
    const double prediction_horizon = input.raw_camera_dt_s * desired_gap;
    const double prediction_scale = prediction_horizon / measured_dt;
    out.predicted_compensated_median_px =
        input.motion.compensated_median_px * prediction_scale;
    out.predicted_compensated_p95_ucb_px =
        input.motion.compensated_p95_ucb_px * prediction_scale;
    out.predicted_total_p95_px = input.motion.raw_p95_px * prediction_scale;
    out.predicted_rotation_p95_px =
        input.motion.rotation_p95_px * prediction_scale;
    if (out.ground_geometry_valid)
      out.predicted_ground_overlap = ground_footprint_overlap(
          input.ground_footprint, prediction_horizon);

    if (desired_gap < current_gap_) {
      pending_gap_ = -1;
      pending_since_s_ = std::numeric_limits<double>::quiet_NaN();
      apply_gap(desired_gap, "continuous_safety_contraction", out, true);
      out.immediate_contraction = true;
      return out;
    }
    if (desired_gap == current_gap_) {
      pending_gap_ = -1;
      pending_since_s_ = std::numeric_limits<double>::quiet_NaN();
      out.tracking_gap = current_gap_;
      out.reason = "continuous_horizon_hold";
      return out;
    }

    if (pending_gap_ <= current_gap_ || !std::isfinite(pending_since_s_)) {
      pending_gap_ = desired_gap;
      pending_since_s_ = measurement_time;
    } else {
      // Confirm a continuously safe lower bound. Small rate fluctuations may
      // change the rounded integer, but they must not restart a count-like
      // confirmation sequence on every KLT pair.
      pending_gap_ = std::min(pending_gap_, desired_gap);
    }
    if (measurement_time - pending_since_s_ >=
        config_.upshift_confirmation_duration_s) {
      apply_gap(pending_gap_, "continuous_horizon_expansion_confirmed", out,
                false);
      out.expansion_confirmed = true;
      pending_gap_ = -1;
      pending_since_s_ = std::numeric_limits<double>::quiet_NaN();
    } else {
      out.tracking_gap = current_gap_;
      out.reason = "continuous_horizon_expansion_pending";
    }
    return out;
  }

  int tracking_gap() const { return current_gap_; }

private:
  void apply_gap(int gap, const std::string &reason,
                 ContinuousTrackingDecision &out, bool contraction) {
    gap = std::max(config_.minimum_gap, std::min(config_.maximum_gap, gap));
    out.changed = gap != current_gap_;
    out.immediate_contraction = contraction && gap < current_gap_;
    current_gap_ = gap;
    out.tracking_gap = current_gap_;
    out.instantaneous_safe_gap = gap;
    out.reason = reason;
  }

  ContinuousTrackingPlannerConfig config_;
  int current_gap_ = 1;
  int pending_gap_ = -1;
  double pending_since_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_motion_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
};

} // namespace ov_msckf

#endif // OV_MSCKF_VISUAL_CADENCE_PLANNER_H
