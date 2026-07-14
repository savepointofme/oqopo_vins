/* Shared causal visual-motion measurement and target-parallax cadence planner. */

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
  double survival_ratio = 0.0;
  double median_track_age = 0.0;
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
  bool valid = false;
};

struct GroundFootprintOverlapInput {
  Eigen::Matrix3d R_GtoC = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_CinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_G = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d, 4> corner_rays_C;
  double ground_height_G = 0.0;
  double prediction_horizon_s = 0.0;
};

inline double polygon_signed_area(const std::vector<Eigen::Vector2d> &polygon) {
  double area = 0.0;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const Eigen::Vector2d &a = polygon[i];
    const Eigen::Vector2d &b = polygon[(i + 1) % polygon.size()];
    area += a.x() * b.y() - a.y() * b.x();
  }
  return 0.5 * area;
}

inline Eigen::Vector2d line_intersection(const Eigen::Vector2d &p,
                                         const Eigen::Vector2d &q,
                                         const Eigen::Vector2d &a,
                                         const Eigen::Vector2d &b) {
  const Eigen::Vector2d r = q - p;
  const Eigen::Vector2d s = b - a;
  const double denominator = r.x() * s.y() - r.y() * s.x();
  if (std::fabs(denominator) < 1e-12)
    return q;
  const Eigen::Vector2d ap = a - p;
  const double t = (ap.x() * s.y() - ap.y() * s.x()) / denominator;
  return p + t * r;
}

inline std::vector<Eigen::Vector2d>
clip_convex_polygon(std::vector<Eigen::Vector2d> subject,
                    std::vector<Eigen::Vector2d> clip) {
  if (polygon_signed_area(clip) < 0.0)
    std::reverse(clip.begin(), clip.end());
  for (size_t edge = 0; edge < clip.size() && !subject.empty(); ++edge) {
    const Eigen::Vector2d a = clip[edge];
    const Eigen::Vector2d b = clip[(edge + 1) % clip.size()];
    auto inside = [&](const Eigen::Vector2d &point) {
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
        output.push_back(line_intersection(previous, current, a, b));
      if (current_inside)
        output.push_back(current);
      previous = current;
      previous_inside = current_inside;
    }
    subject.swap(output);
  }
  return subject;
}

/// Projects the calibrated image corners onto the declared horizontal ground
/// plane and computes polygon intersection-over-union after the predicted
/// horizontal translation. Invalid/upward-looking geometry returns NaN.
inline double ground_footprint_polygon_overlap(
    const GroundFootprintOverlapInput &input) {
  if (!input.R_GtoC.allFinite() || !input.p_CinG.allFinite() ||
      !input.velocity_G.allFinite() || !(input.prediction_horizon_s >= 0.0))
    return std::numeric_limits<double>::quiet_NaN();
  std::vector<Eigen::Vector2d> footprint;
  footprint.reserve(4);
  for (const Eigen::Vector3d &corner_ray_C : input.corner_rays_C) {
    const Eigen::Vector3d direction_G = input.R_GtoC.transpose() * corner_ray_C;
    if (!direction_G.allFinite() || std::fabs(direction_G.z()) < 1e-8)
      return std::numeric_limits<double>::quiet_NaN();
    const double scale =
        (input.ground_height_G - input.p_CinG.z()) / direction_G.z();
    if (!(scale > 0.0) || !std::isfinite(scale))
      return std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d point_G = input.p_CinG + scale * direction_G;
    footprint.emplace_back(point_G.x(), point_G.y());
  }
  if (polygon_signed_area(footprint) < 0.0)
    std::reverse(footprint.begin(), footprint.end());
  const double area = std::fabs(polygon_signed_area(footprint));
  if (!(area > 1e-9))
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector2d shift = input.velocity_G.head<2>() *
                                input.prediction_horizon_s;
  std::vector<Eigen::Vector2d> predicted = footprint;
  for (Eigen::Vector2d &point : predicted)
    point += shift;
  const std::vector<Eigen::Vector2d> intersection =
      clip_convex_polygon(footprint, predicted);
  const double intersection_area =
      intersection.size() >= 3
          ? std::fabs(polygon_signed_area(intersection))
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

/// R_Ccurrent_Cprevious rotates a previous-camera bearing into the current
/// camera coordinates. fx/fy convert normalized displacement to pixels.
inline VisualMotionMetrics compute_visual_motion_metrics(
    const VisualFrameSnapshot &previous, const VisualFrameSnapshot &current,
    const Eigen::Matrix3d &R_Ccurrent_Cprevious, double fx, double fy) {
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
  for (const auto &track : current.tracks) {
    if (!track.valid || !track.raw.allFinite() || !track.normalized.allFinite())
      continue;
    const auto found = prior.find(track.feature_id);
    if (found == prior.end())
      continue;
    const VisualTrackPoint &old = *found->second;
    Eigen::Vector3d old_ray(old.normalized.x(), old.normalized.y(), 1.0);
    Eigen::Vector3d predicted = R_Ccurrent_Cprevious * old_ray;
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
  }

  out.common_tracks = static_cast<int>(raw_motion.size());
  const int denominator = std::max(1, std::min(out.previous_tracks, out.current_tracks));
  out.survival_ratio = static_cast<double>(out.common_tracks) /
                       static_cast<double>(denominator);
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
  out.valid = out.common_tracks >= 4;
  return out;
}

struct VisualCadencePlannerConfig {
  std::vector<int> allowed_strides = {1, 2, 4, 6, 8, 12};
  double target_compensated_parallax_min_px = 2.0;
  double target_compensated_parallax_max_px = 8.0;
  double max_compensated_p95_px = 18.0;
  double max_total_p95_px = 35.0;
  double max_rotation_p95_px = 28.0;
  double minimum_survival_ratio = 0.55;
  double maximum_tracking_interval_s = 0.40;
  /// Median leave-one-flight-out recommendation from the registered legacy
  /// 4x8 rotation-aware overlap evidence. Pixel-flow targets remain separate
  /// because that legacy evidence did not log common-ID compensated flow.
  double minimum_ground_overlap = 0.90;
  int upshift_confirmation_count = 4;
  double upshift_confirmation_duration_s = 0.50;
  double target_downshift_hysteresis_ratio = 1.25;
};

struct VisualCadencePlannerInput {
  double timestamp = -1.0;
  /// Timestamp of the KLT pair that produced `motion`. It can remain older
  /// than `timestamp` on raw frames where tracking was intentionally skipped.
  double motion_measurement_timestamp_s = -1.0;
  double raw_camera_dt_s = 0.0;
  VisualMotionMetrics motion;
  double ground_footprint_overlap = std::numeric_limits<double>::quiet_NaN();
  std::map<int, double> ground_footprint_overlap_by_stride;
  int safety_stride_cap = 12;
  bool force_dense = false;
};

struct VisualCadenceDecision {
  int stride = 1;
  int unconstrained_stride = 1;
  int safety_stride_cap = 1;
  double predicted_compensated_median_px = 0.0;
  double predicted_compensated_p95_px = 0.0;
  double predicted_total_p95_px = 0.0;
  double predicted_rotation_p95_px = 0.0;
  bool immediate_downshift = false;
  bool upshift_confirmed = false;
  std::string reason = "bootstrap_dense";
};

class VisualCadencePlanner {
public:
  explicit VisualCadencePlanner(
      const VisualCadencePlannerConfig &config = VisualCadencePlannerConfig())
      : config_(config) {
    std::sort(config_.allowed_strides.begin(), config_.allowed_strides.end());
    config_.allowed_strides.erase(
        std::unique(config_.allowed_strides.begin(), config_.allowed_strides.end()),
        config_.allowed_strides.end());
    if (config_.allowed_strides.empty())
      config_.allowed_strides.push_back(1);
    current_stride_ = config_.allowed_strides.front();
  }

  VisualCadenceDecision update(const VisualCadencePlannerInput &input) {
    VisualCadenceDecision out;
    const int cap = bounded_stride(input.safety_stride_cap);
    out.safety_stride_cap = cap;
    const double measurement_time =
        std::isfinite(input.motion_measurement_timestamp_s)
            ? input.motion_measurement_timestamp_s
            : (input.motion.valid ? input.timestamp
                                  : std::numeric_limits<double>::quiet_NaN());
    const bool new_motion_measurement =
        std::isfinite(measurement_time) &&
        (!std::isfinite(last_motion_measurement_timestamp_s_) ||
         measurement_time > last_motion_measurement_timestamp_s_ + 1.0e-9);
    if (new_motion_measurement)
      last_motion_measurement_timestamp_s_ = measurement_time;

    if (input.force_dense) {
      set_stride(config_.allowed_strides.front(), out, "insufficient_visual_motion");
      return out;
    }
    if (!input.motion.valid || !(input.raw_camera_dt_s > 0.0)) {
      if (std::isfinite(last_motion_measurement_timestamp_s_) &&
          !new_motion_measurement) {
        out.stride = std::min(current_stride_, cap);
        out.unconstrained_stride = current_stride_;
        out.reason = "hold_until_new_motion_measurement";
        return out;
      }
      set_stride(config_.allowed_strides.front(), out,
                 "insufficient_visual_motion");
      return out;
    }

    const double measured_dt = std::max(1e-6, input.motion.dt_s);
    int desired = config_.allowed_strides.front();
    int largest_safe = config_.allowed_strides.front();
    std::string reason = "target_parallax_dense";
    for (int stride : config_.allowed_strides) {
      if (stride > cap)
        break;
      const double horizon = input.raw_camera_dt_s * stride;
      const double scale = horizon / measured_dt;
      const double compensated_median = input.motion.compensated_median_px * scale;
      const double compensated_p95 = input.motion.compensated_p95_px * scale;
      const double total_p95 = input.motion.raw_p95_px * scale;
      const double rotation_p95 = input.motion.rotation_p95_px * scale;
      const double rotation_edge_max = input.motion.rotation_max_px * scale;
      const auto overlap_found =
          input.ground_footprint_overlap_by_stride.find(stride);
      const double overlap =
          overlap_found != input.ground_footprint_overlap_by_stride.end()
              ? overlap_found->second
              : input.ground_footprint_overlap;
      const bool safe =
          horizon <= config_.maximum_tracking_interval_s &&
          compensated_p95 <= config_.max_compensated_p95_px &&
          total_p95 <= config_.max_total_p95_px &&
          rotation_p95 <= config_.max_rotation_p95_px &&
          rotation_edge_max <= 1.5 * config_.max_rotation_p95_px &&
          input.motion.survival_ratio >= config_.minimum_survival_ratio &&
          (!std::isfinite(overlap) ||
           overlap >= config_.minimum_ground_overlap);
      if (!safe)
        break;
      largest_safe = stride;
      if (compensated_median <= config_.target_compensated_parallax_max_px) {
        desired = stride;
        reason = compensated_median < config_.target_compensated_parallax_min_px
                     ? "below_target_parallax"
                     : "inside_target_parallax";
      }
    }
    if (desired < current_stride_ && current_stride_ <= largest_safe) {
      const double current_scale =
          input.raw_camera_dt_s * current_stride_ / measured_dt;
      const double current_compensated_median =
          input.motion.compensated_median_px * current_scale;
      if (current_compensated_median <=
          config_.target_compensated_parallax_max_px *
              config_.target_downshift_hysteresis_ratio) {
        desired = current_stride_;
        reason = "target_parallax_downshift_deadband";
      }
    }
    out.unconstrained_stride = desired;
    const double scale = input.raw_camera_dt_s * desired / measured_dt;
    out.predicted_compensated_median_px =
        input.motion.compensated_median_px * scale;
    out.predicted_compensated_p95_px = input.motion.compensated_p95_px * scale;
    out.predicted_total_p95_px = input.motion.raw_p95_px * scale;
    out.predicted_rotation_p95_px = input.motion.rotation_p95_px * scale;

    if (desired < current_stride_) {
      current_stride_ = desired;
      pending_upshift_stride_ = desired;
      upshift_count_ = 0;
      pending_upshift_since_s_ = measurement_time;
      out.immediate_downshift = true;
      reason = "safety_or_motion_downshift:" + reason;
    } else if (desired > current_stride_) {
      if (new_motion_measurement && pending_upshift_stride_ != desired) {
        pending_upshift_stride_ = desired;
        upshift_count_ = 1;
        pending_upshift_since_s_ = measurement_time;
      } else if (new_motion_measurement) {
        ++upshift_count_;
      }
      const double stable_duration =
          std::isfinite(measurement_time) &&
                  std::isfinite(pending_upshift_since_s_)
              ? measurement_time - pending_upshift_since_s_
              : 0.0;
      if (new_motion_measurement &&
          upshift_count_ >= std::max(1, config_.upshift_confirmation_count) &&
          stable_duration + 1.0e-12 >=
              config_.upshift_confirmation_duration_s) {
        current_stride_ = desired;
        upshift_count_ = 0;
        out.upshift_confirmed = true;
      }
    } else if (new_motion_measurement) {
      pending_upshift_stride_ = desired;
      upshift_count_ = 0;
      pending_upshift_since_s_ = measurement_time;
    }
    out.stride = current_stride_;
    out.reason = reason;
    return out;
  }

  int current_stride() const { return current_stride_; }

private:
  int bounded_stride(int requested) const {
    int bounded = config_.allowed_strides.front();
    for (int stride : config_.allowed_strides)
      if (stride <= requested)
        bounded = stride;
    return bounded;
  }

  void set_stride(int stride, VisualCadenceDecision &out,
                  const std::string &reason) {
    out.immediate_downshift = stride < current_stride_;
    current_stride_ = stride;
    pending_upshift_stride_ = stride;
    upshift_count_ = 0;
    pending_upshift_since_s_ =
        std::numeric_limits<double>::quiet_NaN();
    out.stride = stride;
    out.unconstrained_stride = stride;
    out.reason = reason;
  }

  VisualCadencePlannerConfig config_;
  int current_stride_ = 1;
  int pending_upshift_stride_ = 1;
  int upshift_count_ = 0;
  double pending_upshift_since_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_motion_measurement_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
};

} // namespace ov_msckf

#endif // OV_MSCKF_VISUAL_CADENCE_PLANNER_H
