#include "core/AglSceneScaleGroundEstimator.h"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using ov_msckf::AglGroundEstimate;
using ov_msckf::AglGroundEstimatorConfig;
using ov_msckf::AglGroundFeature;
using ov_msckf::AglSceneScaleGroundEstimator;

namespace {

int checks = 0;
int failures = 0;

void expect(bool condition, const std::string &message) {
  ++checks;
  if (!condition) {
    ++failures;
    std::cerr << "FAIL: " << message << "\n";
  }
}

std::vector<AglGroundFeature> make_scene(double map_height,
                                         double slope_x = 0.0,
                                         bool collapsed_image = false) {
  std::vector<AglGroundFeature> features;
  size_t id = 1;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 8; ++column) {
      AglGroundFeature feature;
      feature.feature_id = id++;
      const double x = -45.0 + 12.0 * column;
      const double y = -30.0 + 12.0 * row;
      const double deterministic_noise =
          0.08 * std::sin(0.7 * column + 1.3 * row);
      feature.p_FinG = Eigen::Vector3d(
          x, y, -map_height + slope_x * x + deterministic_noise);
      feature.uv = collapsed_image
                       ? Eigen::Vector2d(320.0 + column, 240.0 + row)
                       : Eigen::Vector2d(35.0 + 80.0 * column,
                                         55.0 + 65.0 * row);
      feature.observation_count = 7;
      feature.first_observation_time = 10.0;
      feature.last_observation_time = 10.7;
      feature.supporting_clone_times = {10.0, 10.2, 10.4, 10.6};
      features.push_back(feature);
    }
  }

  // A coherent but nearer vegetation/roof cluster. It must not replace the
  // farther, spatially supported ground mode.
  for (int i = 0; i < 15; ++i) {
    AglGroundFeature clutter;
    clutter.feature_id = id++;
    clutter.p_FinG = Eigen::Vector3d(-25.0 + 3.0 * i,
                                     -10.0 + 1.2 * i,
                                     -0.42 * map_height + 0.3 * (i % 3));
    clutter.uv = Eigen::Vector2d(120.0 + 10.0 * i, 180.0 + 2.0 * i);
    clutter.observation_count = 5;
    clutter.first_observation_time = 10.1;
    clutter.last_observation_time = 10.5;
    clutter.supporting_clone_times = {10.1, 10.3, 10.5};
    features.push_back(clutter);
  }

  // Isolated triangulation outliers.
  for (int i = 0; i < 6; ++i) {
    AglGroundFeature outlier;
    outlier.feature_id = id++;
    outlier.p_FinG = Eigen::Vector3d(-20.0 + i * 8.0, 5.0,
                                     -map_height * (1.15 + 0.08 * i));
    outlier.uv = Eigen::Vector2d(70.0 + 90.0 * i, 390.0);
    outlier.observation_count = 5;
    outlier.first_observation_time = 10.0;
    outlier.last_observation_time = 10.5;
    outlier.supporting_clone_times = {10.0, 10.25, 10.5};
    features.push_back(outlier);
  }
  return features;
}

Eigen::Matrix3d downward_camera() {
  return Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

void test_known_scene_height_and_clutter_rejection() {
  AglSceneScaleGroundEstimator estimator;
  const AglGroundEstimate estimate = estimator.estimate(
      10.7, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, make_scene(100.0));
  expect(estimate.valid, "ground plane should be accepted: " + estimate.reason);
  expect(std::fabs(estimate.map_height_m - 100.0) < 0.20,
         "known 100 m map height should be recovered");
  expect(estimate.inlier_feature_count >= 40,
         "ground mode should retain broad ground support");
  expect(estimate.inlier_feature_count < estimate.candidate_feature_count,
         "near clutter/outliers should not all become plane inliers");
  expect(estimate.occupied_image_cells >= 5,
         "accepted plane should cover multiple image cells");
  expect(estimate.distinct_clone_times >= 3,
         "accepted plane should have multi-clone support");
}

void test_scale_observability_ratio() {
  AglSceneScaleGroundEstimator estimator;
  const AglGroundEstimate estimate = estimator.estimate(
      12.0, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, make_scene(80.0));
  expect(estimate.valid, "scaled scene should remain geometrically valid");
  const double scale = 100.0 / estimate.map_height_m;
  expect(std::fabs(scale - 1.25) < 0.005,
         "AGL/map-height ratio should recover the known scene scale");
}

void test_gravity_constrained_slope() {
  AglSceneScaleGroundEstimator estimator;
  const double slope = std::tan(8.0 * M_PI / 180.0);
  const AglGroundEstimate estimate = estimator.estimate(
      14.0, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, make_scene(90.0, slope));
  expect(estimate.valid, "8 degree ground plane should pass tilt contract");
  expect(std::fabs(estimate.map_height_m - 90.0) < 0.25,
         "sloped plane height at camera XY should be recovered");
  expect(std::fabs(estimate.plane_tilt_deg - 8.0) < 0.2,
         "reported plane tilt should match the synthetic plane");
}

void test_fail_closed_on_spatial_or_temporal_degeneracy() {
  AglSceneScaleGroundEstimator estimator;
  const AglGroundEstimate collapsed = estimator.estimate(
      16.0, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, make_scene(100.0, 0.0, true));
  expect(!collapsed.valid,
         "spatially collapsed features must not define scene scale");
  expect(collapsed.reason == "insufficient_image_coverage",
         "collapsed support should fail the image coverage contract");

  auto features = make_scene(100.0);
  for (auto &feature : features)
    feature.supporting_clone_times = {10.0, 10.0, 10.0};
  const AglGroundEstimate single_clone = estimator.estimate(
      16.0, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, features);
  expect(!single_clone.valid,
         "single-clone support must not define scene scale");
  expect(single_clone.reason == "insufficient_clone_support",
         "single-clone evidence should fail the clone support contract");
}

void test_fail_closed_on_excessive_tilt() {
  AglSceneScaleGroundEstimator estimator;
  const double slope = std::tan(28.0 * M_PI / 180.0);
  const AglGroundEstimate estimate = estimator.estimate(
      18.0, Eigen::Vector3d::Zero(), downward_camera(),
      Eigen::Vector3d::UnitZ(), 640, 480, make_scene(100.0, slope));
  expect(!estimate.valid, "28 degree plane must exceed gravity tilt limit");
  expect(estimate.reason == "plane_tilt_too_large",
         "excessive tilt should be diagnosed explicitly");
}

} // namespace

int main() {
  test_known_scene_height_and_clutter_rejection();
  test_scale_observability_ratio();
  test_gravity_constrained_slope();
  test_fail_closed_on_spatial_or_temporal_degeneracy();
  test_fail_closed_on_excessive_tilt();
  std::cout << "AGL scene-scale ground estimator: " << checks
            << " checks passed=" << (checks - failures)
            << " failed=" << failures << "\n";
  return failures == 0 ? 0 : 1;
}
