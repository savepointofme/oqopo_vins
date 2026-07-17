#include "core/p4/factors/Factor_P4Epipolar.h"

#include "utils/quat_ops.h"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace ov_msckf {
namespace p4 {

namespace {
constexpr double kDerivativeStep = 1.0e-7;

Eigen::Vector4d normalizedJpl(const double *data) {
  Eigen::Vector4d quaternion = Eigen::Map<const Eigen::Vector4d>(data);
  const double norm = quaternion.norm();
  if (!(norm > 1.0e-12) || !std::isfinite(norm))
    return (Eigen::Vector4d() << 0.0, 0.0, 0.0, 1.0).finished();
  return quaternion / norm;
}
} // namespace

Factor_P4Epipolar::Factor_P4Epipolar(
    const Eigen::Vector2d &normalized_previous,
    const Eigen::Vector2d &normalized_current,
    const Eigen::Matrix3d &R_ItoC, const Eigen::Vector3d &p_IinC,
    double normalized_measurement_sigma)
    : previous_bearing_C_(normalized_previous.x(), normalized_previous.y(),
                          1.0),
      current_bearing_C_(normalized_current.x(), normalized_current.y(), 1.0),
      R_ItoC_(R_ItoC), p_IinC_(p_IinC),
      normalized_sigma_(normalized_measurement_sigma) {
  if (!previous_bearing_C_.allFinite() ||
      !current_bearing_C_.allFinite() || !R_ItoC_.allFinite() ||
      !p_IinC_.allFinite() || !(normalized_sigma_ > 0.0) ||
      !std::isfinite(normalized_sigma_))
    throw std::invalid_argument("invalid P4 epipolar factor input");
  // Keep normalized-image homogeneous coordinates in the canonical
  // [u, v, 1] scale.  The Sampson denominator is not invariant to arbitrary
  // independent rescaling of the two homogeneous points.
  set_num_residuals(1);
  mutable_parameter_block_sizes()->push_back(4);
  mutable_parameter_block_sizes()->push_back(3);
  mutable_parameter_block_sizes()->push_back(4);
  mutable_parameter_block_sizes()->push_back(3);
}

Eigen::Vector4d Factor_P4Epipolar::plusJpl(
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

double Factor_P4Epipolar::evaluateResidual(
    double const *const *parameters) const {
  const Eigen::Matrix3d R_GtoI_previous =
      ov_core::quat_2_Rot(normalizedJpl(parameters[0]));
  const Eigen::Matrix3d R_GtoI_current =
      ov_core::quat_2_Rot(normalizedJpl(parameters[2]));
  const Eigen::Vector3d p_Iprevious_inG =
      Eigen::Map<const Eigen::Vector3d>(parameters[1]);
  const Eigen::Vector3d p_Icurrent_inG =
      Eigen::Map<const Eigen::Vector3d>(parameters[3]);

  const Eigen::Matrix3d R_GtoC_previous =
      R_ItoC_ * R_GtoI_previous;
  const Eigen::Matrix3d R_GtoC_current =
      R_ItoC_ * R_GtoI_current;
  const Eigen::Vector3d p_Cprevious_inG =
      p_Iprevious_inG -
      R_GtoI_previous.transpose() * R_ItoC_.transpose() * p_IinC_;
  const Eigen::Vector3d p_Ccurrent_inG =
      p_Icurrent_inG -
      R_GtoI_current.transpose() * R_ItoC_.transpose() * p_IinC_;
  const Eigen::Matrix3d R_Cprevious_to_Ccurrent =
      R_GtoC_current * R_GtoC_previous.transpose();
  const Eigen::Vector3d p_Cprevious_in_Ccurrent =
      R_GtoC_current * (p_Cprevious_inG - p_Ccurrent_inG);
  const Eigen::Matrix3d essential =
      ov_core::skew_x(p_Cprevious_in_Ccurrent) *
      R_Cprevious_to_Ccurrent;

  const Eigen::Vector3d line_current = essential * previous_bearing_C_;
  const Eigen::Vector3d line_previous =
      essential.transpose() * current_bearing_C_;
  const double numerator =
      current_bearing_C_.dot(line_current);
  const double denominator = std::sqrt(
      line_current.head<2>().squaredNorm() +
      line_previous.head<2>().squaredNorm() + 1.0e-12);
  return numerator / (normalized_sigma_ * denominator);
}

bool Factor_P4Epipolar::Evaluate(double const *const *parameters,
                                 double *residuals,
                                 double **jacobians) const {
  residuals[0] = evaluateResidual(parameters);
  if (!std::isfinite(residuals[0]))
    return false;
  if (jacobians == nullptr)
    return true;

  std::vector<const double *> perturbed(4);
  for (std::size_t block = 0; block < perturbed.size(); ++block)
    perturbed[block] = parameters[block];
  for (int block : {0, 2}) {
    if (jacobians[block] == nullptr)
      continue;
    Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> output(
        jacobians[block]);
    output.setZero();
    const Eigen::Vector4d quaternion = normalizedJpl(parameters[block]);
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(axis) = kDerivativeStep;
      const Eigen::Vector4d plus = plusJpl(quaternion, delta);
      const Eigen::Vector4d minus = plusJpl(quaternion, -delta);
      perturbed[block] = plus.data();
      const double plus_residual = evaluateResidual(perturbed.data());
      perturbed[block] = minus.data();
      const double minus_residual = evaluateResidual(perturbed.data());
      perturbed[block] = parameters[block];
      output(axis) =
          (plus_residual - minus_residual) / (2.0 * kDerivativeStep);
    }
  }
  for (int block : {1, 3}) {
    if (jacobians[block] == nullptr)
      continue;
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> output(
        jacobians[block]);
    const Eigen::Vector3d position =
        Eigen::Map<const Eigen::Vector3d>(parameters[block]);
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d plus = position;
      Eigen::Vector3d minus = position;
      plus(axis) += kDerivativeStep;
      minus(axis) -= kDerivativeStep;
      perturbed[block] = plus.data();
      const double plus_residual = evaluateResidual(perturbed.data());
      perturbed[block] = minus.data();
      const double minus_residual = evaluateResidual(perturbed.data());
      perturbed[block] = parameters[block];
      output(axis) =
          (plus_residual - minus_residual) / (2.0 * kDerivativeStep);
    }
  }
  return true;
}

} // namespace p4
} // namespace ov_msckf
