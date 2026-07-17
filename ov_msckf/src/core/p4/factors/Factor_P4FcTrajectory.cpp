#include "core/p4/factors/Factor_P4FcTrajectory.h"

#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ov_msckf {
namespace p4 {

namespace {
constexpr double kFiniteDifferenceStep = 1.0e-7;

Eigen::Vector4d normalizedJpl(const double *data) {
  Eigen::Vector4d quaternion = Eigen::Map<const Eigen::Vector4d>(data);
  const double norm = quaternion.norm();
  if (!(norm > 1.0e-12) || !std::isfinite(norm))
    return (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  return quaternion / norm;
}
} // namespace

Factor_P4FcTrajectory::Factor_P4FcTrajectory(
    const std::vector<double> &timestamps,
    const std::vector<Eigen::Matrix3d> &target_R_GtoI,
    const std::vector<Eigen::Vector3d> &target_positions_G,
    const std::vector<Eigen::Vector3d> &target_velocities_G,
    const Eigen::Matrix<double, 9, 9> &terminal_covariance,
    double process_variance_fraction)
    : timestamps_(timestamps), target_R_GtoI_(target_R_GtoI),
      target_positions_G_(target_positions_G),
      target_velocities_G_(target_velocities_G) {
  const std::size_t count = timestamps_.size();
  if (count < 2 || target_R_GtoI_.size() != count ||
      target_positions_G_.size() != count ||
      target_velocities_G_.size() != count)
    throw std::invalid_argument("P4 FC trajectory dimensions are inconsistent");
  for (std::size_t index = 1; index < count; ++index)
    if (!(timestamps_[index] > timestamps_[index - 1]))
      throw std::invalid_argument("P4 FC trajectory timestamps must increase");

  absolute_covariance_ = buildAbsoluteMeasurementCovariance(
      timestamps_, terminal_covariance, process_variance_fraction);
  transform_ = buildTerminalIncrementTransform(count);
  residual_covariance_ =
      transform_ * absolute_covariance_ * transform_.transpose();
  residual_covariance_ =
      0.5 * (residual_covariance_ + residual_covariance_.transpose());
  Eigen::LLT<Eigen::MatrixXd> covariance_llt(residual_covariance_);
  if (covariance_llt.info() != Eigen::Success)
    throw std::invalid_argument("P4 FC residual covariance is not positive definite");
  whitener_ = covariance_llt.matrixL().solve(
      Eigen::MatrixXd::Identity(residual_covariance_.rows(),
                                residual_covariance_.cols()));
  if (!whitener_.allFinite())
    throw std::invalid_argument("P4 FC residual whitener is non-finite");

  set_num_residuals(static_cast<int>(9 * count));
  for (std::size_t index = 0; index < count; ++index) {
    mutable_parameter_block_sizes()->push_back(4); // q_GtoI
    mutable_parameter_block_sizes()->push_back(3); // p_IinG
    mutable_parameter_block_sizes()->push_back(3); // v_IinG
  }
}

Eigen::MatrixXd Factor_P4FcTrajectory::buildAbsoluteMeasurementCovariance(
    const std::vector<double> &timestamps,
    const Eigen::Matrix<double, 9, 9> &terminal_covariance,
    double process_variance_fraction) {
  if (timestamps.size() < 2 || !terminal_covariance.allFinite() ||
      !(process_variance_fraction > 0.0) ||
      !(process_variance_fraction < 1.0))
    throw std::invalid_argument("invalid P4 FC covariance model");
  const double span = timestamps.back() - timestamps.front();
  if (!(span > 0.0))
    throw std::invalid_argument("P4 FC covariance window has zero duration");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> terminal_eigen(
      0.5 * (terminal_covariance + terminal_covariance.transpose()));
  if (terminal_eigen.info() != Eigen::Success ||
      terminal_eigen.eigenvalues().minCoeff() <= 0.0)
    throw std::invalid_argument("P4 FC terminal covariance must be positive definite");

  const std::size_t count = timestamps.size();
  std::vector<Eigen::Matrix<double, 9, 9>> prefix_covariance(count);
  prefix_covariance[0] =
      (1.0 - process_variance_fraction) * terminal_covariance;
  for (std::size_t index = 1; index < count; ++index) {
    const double dt = timestamps[index] - timestamps[index - 1];
    if (!(dt > 0.0))
      throw std::invalid_argument("P4 FC covariance timestamps must increase");
    const Eigen::Matrix<double, 9, 9> process_covariance =
        process_variance_fraction * (dt / span) * terminal_covariance;
    prefix_covariance[index] =
        prefix_covariance[index - 1] + process_covariance;
  }

  Eigen::MatrixXd covariance =
      Eigen::MatrixXd::Zero(9 * count, 9 * count);
  for (std::size_t row = 0; row < count; ++row) {
    for (std::size_t column = 0; column < count; ++column) {
      covariance.block<9, 9>(9 * row, 9 * column) =
          prefix_covariance[std::min(row, column)];
    }
  }
  covariance = 0.5 * (covariance + covariance.transpose());
  return covariance;
}

Eigen::MatrixXd
Factor_P4FcTrajectory::buildTerminalIncrementTransform(std::size_t count) {
  if (count < 2)
    throw std::invalid_argument("P4 FC transform needs at least two states");
  Eigen::MatrixXd transform =
      Eigen::MatrixXd::Zero(9 * count, 9 * count);
  transform.block(0, 9 * (count - 1), 9, 9).setIdentity();
  for (std::size_t index = 1; index < count; ++index) {
    const Eigen::Index row = static_cast<Eigen::Index>(9 * index);
    transform.block(row, 9 * (index - 1), 9, 9) =
        -Eigen::Matrix<double, 9, 9>::Identity();
    transform.block(row, 9 * index, 9, 9) =
        Eigen::Matrix<double, 9, 9>::Identity();
  }
  return transform;
}

Eigen::Vector3d Factor_P4FcTrajectory::so3Log(
    const Eigen::Matrix3d &rotation) {
  Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1.0e-12)
    return Eigen::Vector3d::Zero();
  return angle_axis.axis() * angle_axis.angle();
}

Eigen::Vector4d Factor_P4FcTrajectory::plusJpl(
    const Eigen::Vector4d &quaternion, const Eigen::Vector3d &delta) {
  const double theta = delta.norm();
  Eigen::Vector4d delta_quaternion;
  if (theta < 1.0e-10) {
    delta_quaternion << 0.5 * delta, 1.0;
  } else {
    delta_quaternion.head<3>() =
        delta / theta * std::sin(0.5 * theta);
    delta_quaternion(3) = std::cos(0.5 * theta);
  }
  delta_quaternion.normalize();
  return ov_core::quat_multiply(delta_quaternion, quaternion).normalized();
}

Eigen::VectorXd Factor_P4FcTrajectory::evaluateRaw(
    double const *const *parameters) const {
  const std::size_t count = timestamps_.size();
  Eigen::VectorXd raw = Eigen::VectorXd::Zero(9 * count);
  const auto rotation = [&](std::size_t index) {
    return ov_core::quat_2_Rot(normalizedJpl(parameters[3 * index]));
  };
  const auto position = [&](std::size_t index) {
    return Eigen::Map<const Eigen::Vector3d>(parameters[3 * index + 1]);
  };
  const auto velocity = [&](std::size_t index) {
    return Eigen::Map<const Eigen::Vector3d>(parameters[3 * index + 2]);
  };

  const std::size_t terminal = count - 1;
  raw.segment<3>(0) =
      so3Log(rotation(terminal) * target_R_GtoI_[terminal].transpose());
  raw.segment<3>(3) =
      position(terminal) - target_positions_G_[terminal];
  raw.segment<3>(6) =
      velocity(terminal) - target_velocities_G_[terminal];
  for (std::size_t index = 1; index < count; ++index) {
    const Eigen::Index offset = static_cast<Eigen::Index>(9 * index);
    const Eigen::Matrix3d estimated_relative =
        rotation(index) * rotation(index - 1).transpose();
    const Eigen::Matrix3d target_relative =
        target_R_GtoI_[index] * target_R_GtoI_[index - 1].transpose();
    raw.segment<3>(offset) =
        so3Log(estimated_relative * target_relative.transpose());
    raw.segment<3>(offset + 3) =
        (position(index) - position(index - 1)) -
        (target_positions_G_[index] - target_positions_G_[index - 1]);
    raw.segment<3>(offset + 6) =
        (velocity(index) - velocity(index - 1)) -
        (target_velocities_G_[index] - target_velocities_G_[index - 1]);
  }
  return raw;
}

bool Factor_P4FcTrajectory::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  const std::size_t count = timestamps_.size();
  const Eigen::VectorXd raw = evaluateRaw(parameters);
  const Eigen::VectorXd whitened = whitener_ * raw;
  Eigen::Map<Eigen::VectorXd> residual_map(residuals, whitened.size());
  residual_map = whitened;
  if (jacobians == nullptr)
    return whitened.allFinite();

  for (std::size_t state_index = 0; state_index < count; ++state_index) {
    const std::size_t q_block = 3 * state_index;
    if (jacobians[q_block] != nullptr) {
      Eigen::MatrixXd raw_jacobian = Eigen::MatrixXd::Zero(9 * count, 3);
      const Eigen::Vector4d quaternion =
          normalizedJpl(parameters[q_block]);
      std::vector<const double *> perturbed_parameters(3 * count);
      for (std::size_t block = 0; block < 3 * count; ++block)
        perturbed_parameters[block] = parameters[block];
      for (int axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta(axis) = kFiniteDifferenceStep;
        const Eigen::Vector4d plus = plusJpl(quaternion, delta);
        const Eigen::Vector4d minus = plusJpl(quaternion, -delta);
        perturbed_parameters[q_block] = plus.data();
        const Eigen::VectorXd plus_residual =
            evaluateRaw(perturbed_parameters.data());
        perturbed_parameters[q_block] = minus.data();
        const Eigen::VectorXd minus_residual =
            evaluateRaw(perturbed_parameters.data());
        perturbed_parameters[q_block] = parameters[q_block];
        raw_jacobian.col(axis) =
            (plus_residual - minus_residual) /
            (2.0 * kFiniteDifferenceStep);
      }
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor>>
          output(jacobians[q_block], 9 * count, 4);
      output.setZero();
      output.leftCols<3>() = whitener_ * raw_jacobian;
    }

    for (int component = 0; component < 2; ++component) {
      const std::size_t block = 3 * state_index + 1 + component;
      if (jacobians[block] == nullptr)
        continue;
      Eigen::MatrixXd raw_jacobian = Eigen::MatrixXd::Zero(9 * count, 3);
      const Eigen::Index component_offset = component == 0 ? 3 : 6;
      if (state_index == count - 1)
        raw_jacobian.block<3, 3>(component_offset, 0).setIdentity();
      if (state_index > 0) {
        const Eigen::Index row =
            static_cast<Eigen::Index>(9 * state_index + component_offset);
        raw_jacobian.block<3, 3>(row, 0).setIdentity();
      }
      if (state_index + 1 < count) {
        const Eigen::Index row = static_cast<Eigen::Index>(
            9 * (state_index + 1) + component_offset);
        raw_jacobian.block<3, 3>(row, 0) = -Eigen::Matrix3d::Identity();
      }
      Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
          output(jacobians[block], 9 * count, 3);
      output = whitener_ * raw_jacobian;
    }
  }
  return whitened.allFinite();
}

} // namespace p4
} // namespace ov_msckf
