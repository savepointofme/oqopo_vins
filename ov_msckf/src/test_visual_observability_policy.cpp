#include <Eigen/Dense>

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/VisualObservabilityPolicy.h"

using namespace ov_msckf;
using namespace ov_type;

namespace {

Eigen::Matrix<double, 16, 1> make_imu_state(const Eigen::Vector3d &position) {
  Eigen::Matrix<double, 16, 1> value;
  value.setZero();
  value(3) = 1.0;
  value.segment<3>(4) = position;
  value.segment<3>(7) = Eigen::Vector3d(31.0, -7.0, 2.0);
  return value;
}

Eigen::MatrixXd deterministic_jacobian(int rows, int cols) {
  Eigen::MatrixXd H(rows, cols);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      H(row, col) = std::sin(0.31 * (row + 1) * (col + 2)) +
                    0.07 * (row - col);
    }
  }
  return H;
}

} // namespace

int main() {
  VioManagerOptions options;
  options.state_options.num_cameras = 0;
  options.state_options.max_clone_size = 0;
  options.state_options.max_slam_features = 0;
  auto state = std::make_shared<State>(options.state_options);

  const Eigen::Vector3d initial_position(120.0, -45.0, 8.0);
  const Eigen::Vector3d origin_shift(5000.0, -7000.0, 300.0);
  state->_imu->set_value(make_imu_state(initial_position));
  state->_imu->set_fej(make_imu_state(initial_position));

  std::vector<std::shared_ptr<Type>> order{state->_imu};
  const std::vector<int> ids = VisualObservabilityPolicy::build_H_id(order);
  const Eigen::MatrixXd H = deterministic_jacobian(8, state->_imu->size());

  VisualObservabilityPolicy full_policy(
      VisualObservabilityPolicy::Mode::VISUAL_4D_OC_FEJ_PRECHI2);
  VisualObservabilityPolicy::Diag initial_diag;
  const Eigen::MatrixXd initial_projection =
      full_policy.apply(H, order, ids, state, &initial_diag);

  state->_imu->set_value(make_imu_state(initial_position + origin_shift));
  state->_imu->set_fej(make_imu_state(initial_position + origin_shift));
  VisualObservabilityPolicy::Diag shifted_diag;
  const Eigen::MatrixXd shifted_projection =
      full_policy.apply(H, order, ids, state, &shifted_diag);

  const double full_difference =
      (initial_projection - shifted_projection).norm();
  if (initial_diag.rank_N != 4 || shifted_diag.rank_N != 4 ||
      full_difference > 1.0e-10) {
    std::cerr << "4-DoF projection is not origin invariant: rank="
              << initial_diag.rank_N << "/" << shifted_diag.rank_N
              << " difference=" << full_difference << "\n";
    return 1;
  }

  state->_imu->set_value(make_imu_state(initial_position));
  state->_imu->set_fej(make_imu_state(initial_position));
  VisualObservabilityPolicy yaw_policy(
      VisualObservabilityPolicy::Mode::GLOBAL_YAW_OC_FEJ_PRECHI2);
  const Eigen::MatrixXd yaw_initial = yaw_policy.apply(H, order, ids, state);
  state->_imu->set_value(make_imu_state(initial_position + origin_shift));
  state->_imu->set_fej(make_imu_state(initial_position + origin_shift));
  const Eigen::MatrixXd yaw_shifted = yaw_policy.apply(H, order, ids, state);
  const double yaw_difference = (yaw_initial - yaw_shifted).norm();
  if (yaw_difference < 1.0e-4) {
    std::cerr << "test is not sensitive to the one-dimensional origin defect: "
              << yaw_difference << "\n";
    return 1;
  }

  state->_imu->set_value(make_imu_state(initial_position));
  state->_imu->set_fej(make_imu_state(initial_position));
  const Eigen::MatrixXd post_initial =
      StateHelper::project_visual_measurement_jacobian(
          state, order, H,
          StateHelper::VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION,
          1.0, 1.0);
  state->_imu->set_value(make_imu_state(initial_position + origin_shift));
  state->_imu->set_fej(make_imu_state(initial_position + origin_shift));
  const Eigen::MatrixXd post_shifted =
      StateHelper::project_visual_measurement_jacobian(
          state, order, H,
          StateHelper::VisualYawUpdateMode::GLOBAL_4DOF_OC_PROJECTION,
          1.0, 1.0);
  const double post_difference = (post_initial - post_shifted).norm();
  if (post_difference > 1.0e-10) {
    std::cerr << "post-chi2 4-DoF projection is not origin invariant: "
              << post_difference << "\n";
    return 1;
  }

  std::cout << "visual observability policy tests passed\n";
  return 0;
}
