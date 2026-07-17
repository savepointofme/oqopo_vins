#ifndef OV_MSCKF_AGL_SCENE_SCALE_GROUND_ESTIMATOR_H
#define OV_MSCKF_AGL_SCENE_SCALE_GROUND_ESTIMATOR_H

#include <Eigen/Core>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace ov_msckf {

struct AglGroundFeature {
  size_t feature_id = 0;
  Eigen::Vector3d p_FinG = Eigen::Vector3d::Zero();
  Eigen::Vector2d uv = Eigen::Vector2d::Zero();
  int observation_count = 0;
  double first_observation_time = -1.0;
  double last_observation_time = -1.0;
  std::vector<double> supporting_clone_times;
};

struct AglGroundEstimatorConfig {
  // These are geometric support requirements, not release counters. Every
  // accepted plane must be constrained by several independent image regions
  // and clone times in the current visual window.
  int minimum_candidate_features = 18;
  int minimum_plane_inliers = 12;
  int minimum_feature_observations = 4;
  int minimum_occupied_image_cells = 5;
  int image_grid_columns = 4;
  int image_grid_rows = 3;
  int minimum_distinct_clone_times = 3;
  double minimum_track_span_s = 0.15;
  double minimum_u_span_fraction = 0.30;
  double minimum_v_span_fraction = 0.15;

  double minimum_positive_depth_m = 0.10;
  double minimum_map_height_m = 2.0;
  double maximum_map_height_m = 1000.0;
  double maximum_plane_tilt_deg = 20.0;

  // Initial log-height mode: ground points have a common gravity-axis height
  // even when their perspective depths differ strongly. A 35% half-width is
  // deliberately broad; the following robust plane fit supplies the actual
  // residual gate.
  double log_height_mode_half_width = std::log(1.35);
  double huber_scale_multiplier = 1.5;
  double inlier_mad_multiplier = 3.5;
  double minimum_inlier_threshold_m = 0.20;
  double maximum_relative_plane_residual = 0.04;
  int irls_iterations = 8;
};

struct AglGroundEstimate {
  double timestamp = -1.0;
  bool valid = false;
  std::string reason = "not_evaluated";
  double map_height_m = std::numeric_limits<double>::quiet_NaN();
  double map_height_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double plane_tilt_deg = std::numeric_limits<double>::quiet_NaN();
  double plane_residual_mad_m = std::numeric_limits<double>::quiet_NaN();
  double plane_residual_rmse_m = std::numeric_limits<double>::quiet_NaN();
  double inlier_ratio = 0.0;
  double u_span_fraction = 0.0;
  double v_span_fraction = 0.0;
  int input_feature_count = 0;
  int candidate_feature_count = 0;
  int inlier_feature_count = 0;
  int occupied_image_cells = 0;
  int distinct_clone_times = 0;
  Eigen::Vector3d ground_normal_G = Eigen::Vector3d::UnitZ();
};

class AglSceneScaleGroundEstimator {
public:
  explicit AglSceneScaleGroundEstimator(
      const AglGroundEstimatorConfig &config = AglGroundEstimatorConfig())
      : config_(sanitize(config)) {}

  AglGroundEstimate estimate(double timestamp,
                             const Eigen::Vector3d &camera_center_G,
                             const Eigen::Matrix3d &R_GtoC,
                             const Eigen::Vector3d &gravity_up_G,
                             int image_width, int image_height,
                             const std::vector<AglGroundFeature> &features) const {
    AglGroundEstimate out;
    out.timestamp = timestamp;
    out.input_feature_count = static_cast<int>(features.size());
    if (!std::isfinite(timestamp) || !camera_center_G.allFinite() ||
        !R_GtoC.allFinite() || !gravity_up_G.allFinite() ||
        gravity_up_G.norm() < 1e-9 || image_width <= 1 || image_height <= 1) {
      out.reason = "invalid_input";
      return out;
    }

    const Eigen::Vector3d up = gravity_up_G.normalized();
    Eigen::Vector3d reference = std::fabs(up.z()) < 0.9
                                    ? Eigen::Vector3d::UnitZ()
                                    : Eigen::Vector3d::UnitX();
    const Eigen::Vector3d tangent_x = up.cross(reference).normalized();
    const Eigen::Vector3d tangent_y = up.cross(tangent_x).normalized();

    struct Candidate {
      const AglGroundFeature *feature = nullptr;
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double height = 0.0;
      double log_height = 0.0;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(features.size());
    for (const auto &feature : features) {
      if (!feature.p_FinG.allFinite() || !feature.uv.allFinite() ||
          feature.observation_count < config_.minimum_feature_observations ||
          !std::isfinite(feature.first_observation_time) ||
          !std::isfinite(feature.last_observation_time) ||
          feature.last_observation_time - feature.first_observation_time <
              config_.minimum_track_span_s ||
          feature.uv.x() < 0.0 || feature.uv.x() >= image_width ||
          feature.uv.y() < 0.0 || feature.uv.y() >= image_height) {
        continue;
      }
      const Eigen::Vector3d delta = feature.p_FinG - camera_center_G;
      const double depth = (R_GtoC * delta).z();
      const double height = -up.dot(delta);
      if (!std::isfinite(depth) || depth < config_.minimum_positive_depth_m ||
          !std::isfinite(height) || height < config_.minimum_map_height_m ||
          height > config_.maximum_map_height_m) {
        continue;
      }
      Candidate candidate;
      candidate.feature = &feature;
      candidate.x = tangent_x.dot(delta);
      candidate.y = tangent_y.dot(delta);
      candidate.z = up.dot(delta);
      candidate.height = height;
      candidate.log_height = std::log(height);
      candidates.push_back(candidate);
    }
    out.candidate_feature_count = static_cast<int>(candidates.size());
    if (out.candidate_feature_count < config_.minimum_candidate_features) {
      out.reason = "insufficient_candidates";
      return out;
    }

    // Find the densest gravity-height mode. Support is capped so one very long
    // track cannot dominate the spatial evidence. Equal-density modes select
    // the farther surface, which is the ground rather than nearby vegetation.
    size_t best_center = 0;
    double best_score = -1.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
      double score = 0.0;
      for (const auto &candidate : candidates) {
        if (std::fabs(candidate.log_height - candidates[i].log_height) <=
            config_.log_height_mode_half_width) {
          score += std::sqrt(static_cast<double>(std::min(
              12, std::max(1, candidate.feature->observation_count))));
        }
      }
      if (score > best_score + 1e-9 ||
          (std::fabs(score - best_score) <= 1e-9 &&
           candidates[i].height > candidates[best_center].height)) {
        best_score = score;
        best_center = i;
      }
    }

    std::vector<const Candidate *> mode;
    for (const auto &candidate : candidates) {
      if (std::fabs(candidate.log_height -
                    candidates[best_center].log_height) <=
          config_.log_height_mode_half_width) {
        mode.push_back(&candidate);
      }
    }
    if (static_cast<int>(mode.size()) < config_.minimum_plane_inliers) {
      out.reason = "insufficient_height_mode";
      return out;
    }

    Eigen::Vector3d beta = Eigen::Vector3d::Zero();
    std::vector<double> weights(mode.size(), 1.0);
    std::vector<double> residuals(mode.size(), 0.0);
    for (int iteration = 0; iteration < config_.irls_iterations; ++iteration) {
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      for (size_t i = 0; i < mode.size(); ++i) {
        const Eigen::Vector3d row(mode[i]->x, mode[i]->y, 1.0);
        normal.noalias() += weights[i] * row * row.transpose();
        rhs.noalias() += weights[i] * row * mode[i]->z;
      }
      Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomposition(
          normal);
      if (decomposition.rank() < 3) {
        out.reason = "plane_rank_deficient";
        return out;
      }
      beta = decomposition.solve(rhs);
      if (!beta.allFinite()) {
        out.reason = "plane_nonfinite";
        return out;
      }
      for (size_t i = 0; i < mode.size(); ++i) {
        residuals[i] =
            mode[i]->z - (beta.x() * mode[i]->x +
                          beta.y() * mode[i]->y + beta.z());
      }
      const double center = median(residuals);
      std::vector<double> absolute_deviation;
      absolute_deviation.reserve(residuals.size());
      for (double residual : residuals)
        absolute_deviation.push_back(std::fabs(residual - center));
      const double sigma = std::max(
          1e-6, 1.4826 * median(absolute_deviation));
      const double huber = config_.huber_scale_multiplier * sigma;
      for (size_t i = 0; i < residuals.size(); ++i) {
        const double magnitude = std::fabs(residuals[i] - center);
        weights[i] = magnitude <= huber ? 1.0 : huber / magnitude;
      }
    }

    for (size_t i = 0; i < mode.size(); ++i) {
      residuals[i] =
          mode[i]->z - (beta.x() * mode[i]->x + beta.y() * mode[i]->y +
                        beta.z());
    }
    const double residual_center = median(residuals);
    std::vector<double> absolute_deviation;
    absolute_deviation.reserve(residuals.size());
    for (double residual : residuals)
      absolute_deviation.push_back(std::fabs(residual - residual_center));
    const double residual_mad = 1.4826 * median(absolute_deviation);
    const double inlier_threshold = std::max(
        config_.minimum_inlier_threshold_m,
        config_.inlier_mad_multiplier * std::max(1e-6, residual_mad));

    std::vector<const Candidate *> inliers;
    for (size_t i = 0; i < mode.size(); ++i) {
      if (std::fabs(residuals[i] - residual_center) <= inlier_threshold)
        inliers.push_back(mode[i]);
    }
    out.inlier_feature_count = static_cast<int>(inliers.size());
    out.inlier_ratio = static_cast<double>(inliers.size()) /
                       static_cast<double>(candidates.size());
    if (out.inlier_feature_count < config_.minimum_plane_inliers) {
      out.reason = "insufficient_plane_inliers";
      return out;
    }

    // Final unweighted fit on the robust inlier set.
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (const Candidate *candidate : inliers) {
      const Eigen::Vector3d row(candidate->x, candidate->y, 1.0);
      normal.noalias() += row * row.transpose();
      rhs.noalias() += row * candidate->z;
    }
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> decomposition(
        normal);
    if (decomposition.rank() < 3) {
      out.reason = "final_plane_rank_deficient";
      return out;
    }
    beta = decomposition.solve(rhs);
    out.map_height_m = -beta.z();
    out.plane_tilt_deg =
        std::atan(std::hypot(beta.x(), beta.y())) * 180.0 / M_PI;
    if (!std::isfinite(out.map_height_m) ||
        out.map_height_m < config_.minimum_map_height_m ||
        out.map_height_m > config_.maximum_map_height_m) {
      out.reason = "map_height_out_of_range";
      return out;
    }
    if (!std::isfinite(out.plane_tilt_deg) ||
        out.plane_tilt_deg > config_.maximum_plane_tilt_deg) {
      out.reason = "plane_tilt_too_large";
      return out;
    }

    double sum_squared = 0.0;
    std::vector<double> final_absolute;
    final_absolute.reserve(inliers.size());
    std::set<int> image_cells;
    std::set<long long> clone_time_bins;
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();
    for (const Candidate *candidate : inliers) {
      const double residual =
          candidate->z - (beta.x() * candidate->x +
                          beta.y() * candidate->y + beta.z());
      sum_squared += residual * residual;
      final_absolute.push_back(std::fabs(residual));
      const double u = candidate->feature->uv.x();
      const double v = candidate->feature->uv.y();
      min_u = std::min(min_u, u);
      max_u = std::max(max_u, u);
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
      const int column = std::min(
          config_.image_grid_columns - 1,
          std::max(0, static_cast<int>(u / image_width *
                                       config_.image_grid_columns)));
      const int row = std::min(
          config_.image_grid_rows - 1,
          std::max(0, static_cast<int>(v / image_height *
                                       config_.image_grid_rows)));
      image_cells.insert(row * config_.image_grid_columns + column);
      for (double clone_time : candidate->feature->supporting_clone_times) {
        if (std::isfinite(clone_time))
          clone_time_bins.insert(static_cast<long long>(
              std::llround(clone_time * 1000.0)));
      }
    }
    out.plane_residual_rmse_m =
        std::sqrt(sum_squared / static_cast<double>(inliers.size()));
    out.plane_residual_mad_m = 1.4826 * median(final_absolute);
    out.map_height_sigma_m = std::max(
        out.plane_residual_mad_m /
            std::sqrt(static_cast<double>(inliers.size())),
        1e-6);
    out.occupied_image_cells = static_cast<int>(image_cells.size());
    out.distinct_clone_times = static_cast<int>(clone_time_bins.size());
    out.u_span_fraction = (max_u - min_u) / static_cast<double>(image_width);
    out.v_span_fraction = (max_v - min_v) / static_cast<double>(image_height);

    if (out.occupied_image_cells < config_.minimum_occupied_image_cells ||
        out.u_span_fraction < config_.minimum_u_span_fraction ||
        out.v_span_fraction < config_.minimum_v_span_fraction) {
      out.reason = "insufficient_image_coverage";
      return out;
    }
    if (out.distinct_clone_times < config_.minimum_distinct_clone_times) {
      out.reason = "insufficient_clone_support";
      return out;
    }
    if (!std::isfinite(out.plane_residual_rmse_m) ||
        out.plane_residual_rmse_m >
            config_.maximum_relative_plane_residual * out.map_height_m) {
      out.reason = "plane_residual_too_large";
      return out;
    }

    out.ground_normal_G =
        (up - beta.x() * tangent_x - beta.y() * tangent_y).normalized();
    out.valid = true;
    out.reason = "accepted";
    return out;
  }

private:
  static AglGroundEstimatorConfig sanitize(AglGroundEstimatorConfig config) {
    config.minimum_candidate_features =
        std::max(3, config.minimum_candidate_features);
    config.minimum_plane_inliers = std::max(3, config.minimum_plane_inliers);
    config.minimum_feature_observations =
        std::max(2, config.minimum_feature_observations);
    config.minimum_occupied_image_cells =
        std::max(1, config.minimum_occupied_image_cells);
    config.image_grid_columns = std::max(1, config.image_grid_columns);
    config.image_grid_rows = std::max(1, config.image_grid_rows);
    config.minimum_distinct_clone_times =
        std::max(2, config.minimum_distinct_clone_times);
    config.minimum_track_span_s = std::max(0.0, config.minimum_track_span_s);
    config.minimum_u_span_fraction =
        std::max(0.0, std::min(1.0, config.minimum_u_span_fraction));
    config.minimum_v_span_fraction =
        std::max(0.0, std::min(1.0, config.minimum_v_span_fraction));
    config.minimum_positive_depth_m =
        std::max(0.0, config.minimum_positive_depth_m);
    config.minimum_map_height_m = std::max(1e-3, config.minimum_map_height_m);
    config.maximum_map_height_m =
        std::max(config.minimum_map_height_m, config.maximum_map_height_m);
    config.maximum_plane_tilt_deg =
        std::max(0.0, std::min(80.0, config.maximum_plane_tilt_deg));
    config.log_height_mode_half_width =
        std::max(1e-4, config.log_height_mode_half_width);
    config.huber_scale_multiplier =
        std::max(0.1, config.huber_scale_multiplier);
    config.inlier_mad_multiplier =
        std::max(1.0, config.inlier_mad_multiplier);
    config.minimum_inlier_threshold_m =
        std::max(1e-4, config.minimum_inlier_threshold_m);
    config.maximum_relative_plane_residual =
        std::max(1e-4, config.maximum_relative_plane_residual);
    config.irls_iterations = std::max(1, config.irls_iterations);
    return config;
  }

  static double median(std::vector<double> values) {
    if (values.empty())
      return std::numeric_limits<double>::quiet_NaN();
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    const double upper = values[middle];
    if (values.size() % 2 == 1)
      return upper;
    std::nth_element(values.begin(), values.begin() + middle - 1,
                     values.begin() + middle);
    return 0.5 * (upper + values[middle - 1]);
  }

  AglGroundEstimatorConfig config_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_AGL_SCENE_SCALE_GROUND_ESTIMATOR_H
