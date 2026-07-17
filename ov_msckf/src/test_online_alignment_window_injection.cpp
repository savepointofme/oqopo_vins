#include "feat/FeatureDatabase.h"
#include "state/State.h"
#include "state/StateHelper.h"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace ov_msckf;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

Eigen::MatrixXd make_positive_covariance(Eigen::Index dimension) {
  Eigen::MatrixXd lower = Eigen::MatrixXd::Zero(dimension, dimension);
  for (Eigen::Index row = 0; row < dimension; ++row) {
    lower(row, row) = 0.02 + 0.0005 * static_cast<double>(row);
    for (Eigen::Index col = 0; col < row; ++col)
      lower(row, col) =
          1.0e-4 * static_cast<double>(1 + (row + 2 * col) % 7);
  }
  return lower * lower.transpose();
}

void test_terminal_covariance_handoff_inflation() {
  const Eigen::MatrixXd source = make_positive_covariance(33);
  Eigen::MatrixXd inflated = source;
  std::string failure;
  require(StateHelper::inflate_initial_imu_subspace_covariance(
              inflated, 10.0, 100.0, 10.0, 100.0, &failure),
          "valid joint covariance handoff inflation: " + failure);

  Eigen::MatrixXd expected = source;
  expected.block<3, 3>(0, 0) *= 10.0;
  expected.block<3, 3>(6, 6) *= 100.0;
  expected.block<3, 3>(9, 9) *= 10.0;
  expected.block<3, 3>(12, 12) *= 100.0;
  require((inflated - expected).norm() < 1.0e-12,
          "handoff inflation must match upstream diagonal-block semantics");
  require((inflated.block(0, 15, 15, 18) -
           source.block(0, 15, 15, 18))
              .norm() < 1.0e-12,
          "handoff inflation must not scale active-history cross blocks");
  require((inflated.block(15, 15, 18, 18) -
           source.block(15, 15, 18, 18))
              .norm() < 1.0e-12,
          "clone and landmark marginal covariance must retain unit scale");
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(inflated);
  require(eigen.info() == Eigen::Success &&
              eigen.eigenvalues().minCoeff() > 0.0,
          "handoff inflation must preserve positive definiteness");

  Eigen::MatrixXd invalid = source;
  require(!StateHelper::inflate_initial_imu_subspace_covariance(
              invalid, 10.0, 0.0, 10.0, 100.0, &failure) &&
              failure == "covariance_inflation_not_positive_finite" &&
              (invalid - source).norm() < 1.0e-12,
          "invalid inflation must fail without mutating covariance");
}

void test_atomic_window_injection() {
  StateOptions options;
  options.max_clone_size = 5;
  auto state = std::make_shared<State>(options);

  Eigen::Matrix<double, 16, 1> imu_state;
  imu_state << 0.01, -0.02, 0.03, 0.9992997548, 10.0, -4.0, 2.0,
      18.0, 1.0, -0.3, 0.005, -0.004, 0.003, 0.1, -0.2, 0.05;
  imu_state.head<4>().normalize();
  std::vector<double> timestamps = {8.0, 9.0};
  std::vector<Eigen::Matrix<double, 7, 1>> clones(2);
  clones[0] << 0.0, 0.0, 0.0, 1.0, 7.0, -4.2, 2.1;
  clones[1] << 0.005, -0.002, 0.001, 0.999985, 8.5, -4.1, 2.05;
  clones[1].head<4>().normalize();
  const std::vector<size_t> landmark_ids = {101, 205};
  const std::vector<Eigen::Vector3d> landmark_positions = {
      Eigen::Vector3d(30.0, 5.0, 2.0), Eigen::Vector3d(42.0, -8.0, 4.0)};
  const Eigen::MatrixXd covariance = make_positive_covariance(33);

  std::string failure;
  require(StateHelper::initialize_with_pose_history(
              state, 10.0, imu_state, timestamps, clones, landmark_ids,
              landmark_positions, covariance, &failure),
          "valid joint state/history injection: " + failure);
  require(state->_timestamp == 10.0 && state->_clones_IMU.size() == 2 &&
              state->_features_SLAM.size() == 2,
          "active timestamp, clones, and landmarks must be committed together");
  require((state->_imu->value() - imu_state).norm() < 1.0e-12 &&
              (state->_imu->fej() - imu_state).norm() < 1.0e-12,
          "active nominal and FEJ must match the released state");

  std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu};
  size_t clone_index = 0;
  for (const auto &entry : state->_clones_IMU) {
    require(entry.first == timestamps[clone_index] &&
                (entry.second->value() - clones[clone_index]).norm() < 1.0e-12 &&
                (entry.second->fej() - clones[clone_index]).norm() < 1.0e-12,
            "clone nominal/FEJ ordering must match the release contract");
    order.push_back(entry.second);
    ++clone_index;
  }
  for (size_t index = 0; index < landmark_ids.size(); ++index) {
    const auto landmark = state->_features_SLAM.at(landmark_ids[index]);
    require(landmark->_featid == landmark_ids[index] &&
                landmark->_unique_camera_id == 0 &&
                (landmark->get_xyz(false) - landmark_positions[index]).norm() <
                    1.0e-12 &&
                (landmark->get_xyz(true) - landmark_positions[index]).norm() <
                    1.0e-12,
            "persistent landmark nominal/FEJ must match the graph posterior");
    order.push_back(landmark);
  }
  const Eigen::MatrixXd recovered =
      StateHelper::get_marginal_covariance(state, order);
  require((recovered - covariance).norm() < 1.0e-12,
          "active-clone and clone-clone covariance blocks must be exact");
}

void test_failed_validation_is_non_mutating() {
  StateOptions options;
  options.max_clone_size = 5;
  auto state = std::make_shared<State>(options);
  const int covariance_size_before = state->max_covariance_size();
  const Eigen::MatrixXd imu_before = state->_imu->value();
  Eigen::Matrix<double, 16, 1> imu_state =
      Eigen::Matrix<double, 16, 1>::Zero();
  imu_state(3) = 1.0;
  std::vector<double> timestamps = {9.0, 9.0};
  std::vector<Eigen::Matrix<double, 7, 1>> clones(
      2, (Eigen::Matrix<double, 7, 1>() << 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0)
             .finished());
  std::string failure;
  require(!StateHelper::initialize_with_pose_history(
              state, 10.0, imu_state, timestamps, clones,
              {}, {}, make_positive_covariance(27), &failure) &&
              failure == "clone_timestamps_not_strictly_historical",
          "duplicate clone times must fail closed");
  require(state->_timestamp == -1.0 && state->_clones_IMU.empty() &&
              state->max_covariance_size() == covariance_size_before &&
              (state->_imu->value() - imu_before).norm() < 1.0e-12,
          "failed validation must not partially mutate the filter state");
}

void test_exact_startup_feature_consumption() {
  ov_core::FeatureDatabase database;
  for (size_t feature_id : {11U, 22U, 33U}) {
    database.update_feature(feature_id, 8.0, 0, 10.0F, 20.0F, 0.1F,
                            0.2F);
    database.update_feature(feature_id, 9.0, 0, 11.0F, 21.0F, 0.11F,
                            0.21F);
  }
  require(database.remove_features({11U, 33U, 999U}) == 2,
          "only exact P4-consumed feature IDs must be removed");
  require(database.size() == 1 && database.get_feature(22U) != nullptr &&
              database.get_feature(11U) == nullptr &&
              database.get_feature(33U) == nullptr,
          "independent KLT histories must survive startup consumption");
}

} // namespace

int main() {
  test_terminal_covariance_handoff_inflation();
  test_atomic_window_injection();
  test_failed_validation_is_non_mutating();
  test_exact_startup_feature_consumption();
  std::cout << "online alignment window injection tests passed" << std::endl;
  return EXIT_SUCCESS;
}
