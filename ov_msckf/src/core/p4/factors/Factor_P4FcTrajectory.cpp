#include "core/p4/factors/Factor_P4FcTrajectory.h"

#include <cmath>
#include <stdexcept>

namespace ov_msckf {
namespace p4 {

Factor_P4FcTrajectory::Factor_P4FcTrajectory(
    const std::vector<double> &timestamps,
    const std::vector<Eigen::Vector3d> &target_positions_G,
    const std::vector<Eigen::Vector3d> &target_velocities_G,
    const Eigen::Matrix<double, 6, 6> &terminal_covariance,
    double process_variance_fraction)
    : timestamps_(timestamps), target_positions_G_(target_positions_G),
      target_velocities_G_(target_velocities_G) {
  const std::size_t count = timestamps_.size();
  if (count < 2 || target_positions_G_.size() != count ||
      target_velocities_G_.size() != count)
    throw std::invalid_argument("P4 FC PV trajectory dimensions are inconsistent");
  for (std::size_t index = 1; index < count; ++index)
    if (!(timestamps_[index] > timestamps_[index - 1]))
      throw std::invalid_argument("P4 FC PV timestamps must increase");

  absolute_covariance_ = buildAbsoluteMeasurementCovariance(
      timestamps_, terminal_covariance, process_variance_fraction);
  transform_ = buildTerminalIncrementTransform(count);
  residual_covariance_ =
      transform_ * absolute_covariance_ * transform_.transpose();
  residual_covariance_ =
      0.5 * (residual_covariance_ + residual_covariance_.transpose());
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(residual_covariance_);
  if (covariance_llt.info() != Eigen::Success)
    throw std::invalid_argument(
        "P4 FC PV residual covariance is not positive definite");
  whitener_ = covariance_llt.matrixL().solve(
      Eigen::MatrixXd::Identity(residual_covariance_.rows(),
                                residual_covariance_.cols()));
  if (!whitener_.allFinite())
    throw std::invalid_argument("P4 FC PV residual whitener is non-finite");

  set_num_residuals(static_cast<int>(6 * count));
  for (std::size_t index = 0; index < count; ++index) {
    mutable_parameter_block_sizes()->push_back(3); // p_IinG
    mutable_parameter_block_sizes()->push_back(3); // v_IinG
  }
}

Eigen::MatrixXd Factor_P4FcTrajectory::buildAbsoluteMeasurementCovariance(
    const std::vector<double> &timestamps,
    const Eigen::Matrix<double, 6, 6> &terminal_covariance,
    double process_variance_fraction) {
  if (timestamps.size() < 2 || !terminal_covariance.allFinite() ||
      !(process_variance_fraction > 0.0) ||
      !(process_variance_fraction < 1.0))
    throw std::invalid_argument("invalid P4 FC PV covariance model");
  const double span = timestamps.back() - timestamps.front();
  if (!(span > 0.0))
    throw std::invalid_argument("P4 FC PV covariance window has zero duration");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> terminal_eigen(
      0.5 * (terminal_covariance + terminal_covariance.transpose()));
  if (terminal_eigen.info() != Eigen::Success ||
      terminal_eigen.eigenvalues().minCoeff() <= 0.0)
    throw std::invalid_argument(
        "P4 FC PV terminal covariance must be positive definite");

  const std::size_t count = timestamps.size();
  std::vector<Eigen::Matrix<double, 6, 6>> prefix_covariance(count);
  prefix_covariance[0] =
      (1.0 - process_variance_fraction) * terminal_covariance;
  for (std::size_t index = 1; index < count; ++index) {
    const double dt = timestamps[index] - timestamps[index - 1];
    if (!(dt > 0.0))
      throw std::invalid_argument("P4 FC PV timestamps must increase");
    const Eigen::Matrix<double, 6, 6> process_covariance =
        process_variance_fraction * (dt / span) * terminal_covariance;
    prefix_covariance[index] =
        prefix_covariance[index - 1] + process_covariance;
  }

  Eigen::MatrixXd covariance =
      Eigen::MatrixXd::Zero(6 * count, 6 * count);
  for (std::size_t row = 0; row < count; ++row) {
    for (std::size_t column = 0; column < count; ++column) {
      covariance.block<6, 6>(6 * row, 6 * column) =
          prefix_covariance[std::min(row, column)];
    }
  }
  return 0.5 * (covariance + covariance.transpose());
}

Eigen::MatrixXd
Factor_P4FcTrajectory::buildTerminalIncrementTransform(std::size_t count) {
  if (count < 2)
    throw std::invalid_argument("P4 FC PV transform needs at least two states");
  Eigen::MatrixXd transform =
      Eigen::MatrixXd::Zero(6 * count, 6 * count);
  transform.block(0, 6 * (count - 1), 6, 6).setIdentity();
  for (std::size_t index = 1; index < count; ++index) {
    const Eigen::Index row = static_cast<Eigen::Index>(6 * index);
    transform.block(row, 6 * (index - 1), 6, 6) =
        -Eigen::Matrix<double, 6, 6>::Identity();
    transform.block(row, 6 * index, 6, 6) =
        Eigen::Matrix<double, 6, 6>::Identity();
  }
  return transform;
}

Eigen::VectorXd Factor_P4FcTrajectory::evaluateRaw(
    double const *const *parameters) const {
  const std::size_t count = timestamps_.size();
  Eigen::VectorXd raw = Eigen::VectorXd::Zero(6 * count);
  const auto position = [&](std::size_t index) {
    return Eigen::Map<const Eigen::Vector3d>(parameters[2 * index]);
  };
  const auto velocity = [&](std::size_t index) {
    return Eigen::Map<const Eigen::Vector3d>(parameters[2 * index + 1]);
  };

  const std::size_t terminal = count - 1;
  raw.segment<3>(0) =
      position(terminal) - target_positions_G_[terminal];
  raw.segment<3>(3) =
      velocity(terminal) - target_velocities_G_[terminal];
  for (std::size_t index = 1; index < count; ++index) {
    const Eigen::Index offset = static_cast<Eigen::Index>(6 * index);
    raw.segment<3>(offset) =
        (position(index) - position(index - 1)) -
        (target_positions_G_[index] - target_positions_G_[index - 1]);
    raw.segment<3>(offset + 3) =
        (velocity(index) - velocity(index - 1)) -
        (target_velocities_G_[index] - target_velocities_G_[index - 1]);
  }
  return raw;
}

bool Factor_P4FcTrajectory::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  const std::size_t count = timestamps_.size();
  const Eigen::VectorXd whitened = whitener_ * evaluateRaw(parameters);
  Eigen::Map<Eigen::VectorXd>(residuals, whitened.size()) = whitened;
  if (jacobians == nullptr)
    return whitened.allFinite();

  for (std::size_t state_index = 0; state_index < count; ++state_index) {
    for (int component = 0; component < 2; ++component) {
      const std::size_t block = 2 * state_index + component;
      if (jacobians[block] == nullptr)
        continue;
      Eigen::MatrixXd raw_jacobian = Eigen::MatrixXd::Zero(6 * count, 3);
      const Eigen::Index component_offset = component == 0 ? 0 : 3;
      if (state_index == count - 1)
        raw_jacobian.block<3, 3>(component_offset, 0).setIdentity();
      if (state_index > 0) {
        const Eigen::Index row =
            static_cast<Eigen::Index>(6 * state_index + component_offset);
        raw_jacobian.block<3, 3>(row, 0).setIdentity();
      }
      if (state_index + 1 < count) {
        const Eigen::Index row = static_cast<Eigen::Index>(
            6 * (state_index + 1) + component_offset);
        raw_jacobian.block<3, 3>(row, 0) = -Eigen::Matrix3d::Identity();
      }
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
          output(jacobians[block], 6 * count, 3);
      output = whitener_ * raw_jacobian;
    }
  }
  return whitened.allFinite();
}

} // namespace p4
} // namespace ov_msckf
