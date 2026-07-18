#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "core/p4/factors/Factor_P4Epipolar.h"
#include "core/p4/factors/Factor_P4FcTrajectory.h"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

Eigen::Vector4d plus_jpl(const double *quaternion, int axis, double step) {
  ov_init::State_JPLQuatLocal manifold;
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  delta(axis) = step;
  Eigen::Vector4d result;
  require(manifold.Plus(quaternion, delta.data(), result.data()),
          "JPL manifold perturbation");
  return result;
}

void test_standard_cpi_keeps_bias_endpoints_independent() {
  double dt = 0.02;
  Eigen::Vector3d gravity(0.0, 0.0, 9.81);
  Eigen::Vector3d alpha = Eigen::Vector3d::Zero();
  Eigen::Vector3d beta = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_delta(0.0, 0.0, 0.0, 1.0);
  Eigen::Vector3d ba_linearization = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg_linearization = Eigen::Vector3d::Zero();
  Eigen::Matrix3d J_q = 0.01 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d J_beta = 0.02 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d J_alpha = 0.03 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d H_beta = 0.04 * Eigen::Matrix3d::Identity();
  Eigen::Matrix3d H_alpha = 0.05 * Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 15, 15> covariance =
      0.1 * Eigen::Matrix<double, 15, 15>::Identity();

  ov_init::Factor_ImuCPIv1 factor(
      dt, gravity, alpha, beta, q_delta, ba_linearization,
      bg_linearization, J_q, J_beta, J_alpha, H_beta, H_alpha,
      covariance);

  std::array<double, 4> q0{0.0, 0.0, 0.0, 1.0};
  std::array<double, 4> q1{0.001, -0.002, 0.003, 0.999993};
  std::array<double, 3> bg0{0.01, -0.02, 0.005};
  std::array<double, 3> bg1{0.012, -0.017, 0.004};
  std::array<double, 3> ba0{0.10, -0.05, 0.03};
  std::array<double, 3> ba1{0.08, -0.03, 0.06};
  std::array<double, 3> v0{1.0, 0.2, -0.1};
  std::array<double, 3> v1{1.1, 0.15, -0.25};
  std::array<double, 3> p0{2.0, -1.0, 0.5};
  std::array<double, 3> p1{2.02, -0.996, 0.496};
  const double *parameters[10] = {
      q0.data(), bg0.data(), v0.data(), ba0.data(), p0.data(),
      q1.data(), bg1.data(), v1.data(), ba1.data(), p1.data()};
  std::array<double, 15> residual{};
  require(factor.Evaluate(parameters, residual.data(), nullptr),
          "standard CPI evaluation");

  constexpr std::array<int, 10> block_sizes = {4, 3, 3, 3, 3,
                                                4, 3, 3, 3, 3};
  std::array<std::vector<double>, 10> storage;
  std::array<double *, 10> jacobians{};
  for (std::size_t index = 0; index < storage.size(); ++index) {
    storage[index].resize(15 * block_sizes[index]);
    jacobians[index] = storage[index].data();
  }
  require(factor.Evaluate(parameters, residual.data(), jacobians.data()),
          "standard CPI Jacobian evaluation");
  const Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
      J_bg0(storage[1].data());
  const Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
      J_bg1(storage[6].data());
  const Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
      J_ba0(storage[3].data());
  const Eigen::Map<const Eigen::Matrix<double, 15, 3, Eigen::RowMajor>>
      J_ba1(storage[8].data());
  require(J_bg0.norm() > 0.0 && J_bg1.norm() > 0.0 &&
              J_ba0.norm() > 0.0 && J_ba1.norm() > 0.0,
          "both CPI bias endpoints must remain active parameter blocks");
  require((J_bg0 - J_bg1).norm() > 1.0e-8 &&
              (J_ba0 - J_ba1).norm() > 1.0e-8,
          "CPI endpoint bias Jacobians must not be tied or summed");
}

double correlated_cost(const std::vector<double> &timestamps,
                       const Eigen::Matrix<double, 6, 1> &origin,
                       const Eigen::Matrix<double, 6, 1> &rate,
                       const Eigen::Matrix<double, 6, 6> &terminal) {
  const Eigen::MatrixXd absolute =
      ov_msckf::p4::Factor_P4FcTrajectory::
          buildAbsoluteMeasurementCovariance(timestamps, terminal, 0.25);
  const Eigen::MatrixXd transform =
      ov_msckf::p4::Factor_P4FcTrajectory::
          buildTerminalIncrementTransform(timestamps.size());
  Eigen::VectorXd absolute_error(6 * timestamps.size());
  for (std::size_t index = 0; index < timestamps.size(); ++index)
    absolute_error.segment<6>(6 * index) =
        origin + timestamps[index] * rate;
  const Eigen::VectorXd residual = transform * absolute_error;
  const Eigen::MatrixXd residual_covariance =
      transform * absolute * transform.transpose();
  return residual.dot(residual_covariance.ldlt().solve(residual));
}

void test_fc_covariance_is_correlated_and_density_invariant() {
  Eigen::Matrix<double, 6, 6> terminal =
      Eigen::Matrix<double, 6, 6>::Zero();
  terminal.diagonal() << 4.0, 5.0, 6.0, 0.25, 0.36, 0.49;
  const std::vector<double> coarse{0.0, 1.0, 2.0};
  const std::vector<double> fine{0.0, 0.5, 1.0, 1.5, 2.0};
  const Eigen::MatrixXd absolute =
      ov_msckf::p4::Factor_P4FcTrajectory::
          buildAbsoluteMeasurementCovariance(coarse, terminal, 0.25);
  require((absolute.bottomRightCorner<6, 6>() - terminal).norm() < 1.0e-12,
          "declared terminal FC covariance must be preserved exactly");
  const Eigen::MatrixXd transform =
      ov_msckf::p4::Factor_P4FcTrajectory::
          buildTerminalIncrementTransform(coarse.size());
  const Eigen::MatrixXd residual_covariance =
      transform * absolute * transform.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(
      residual_covariance);
  require(eigen.info() == Eigen::Success &&
              eigen.eigenvalues().minCoeff() > 0.0,
          "dense FC terminal/increment covariance must be positive definite");
  require(residual_covariance.block<6, 6>(0, 12).norm() > 1.0e-8,
          "terminal and increment FC residuals must retain correlation");

  Eigen::Matrix<double, 6, 1> origin;
  origin << 0.3, -0.2, 0.1, 0.05, -0.02, 0.01;
  Eigen::Matrix<double, 6, 1> rate;
  rate << 0.1, -0.05, 0.03, 0.02, -0.01, 0.005;
  const double coarse_cost = correlated_cost(coarse, origin, rate, terminal);
  const double fine_cost = correlated_cost(fine, origin, rate, terminal);
  require(std::fabs(coarse_cost - fine_cost) < 1.0e-10,
          "linear physical FC error path must have keyframe-density invariant likelihood");

  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  for (double timestamp : coarse) {
    positions.emplace_back(timestamp, 0.2 * timestamp, -0.1 * timestamp);
    velocities.emplace_back(1.0, 0.2, -0.1);
  }
  ov_msckf::p4::Factor_P4FcTrajectory factor(
      coarse, positions, velocities, terminal, 0.25);
  std::vector<std::array<double, 3>> estimated_positions(coarse.size());
  std::vector<std::array<double, 3>> estimated_velocities(coarse.size());
  std::vector<const double *> parameters;
  for (std::size_t index = 0; index < coarse.size(); ++index) {
    const Eigen::Vector3d p = positions[index] +
                              Eigen::Vector3d(0.01, -0.02, 0.015);
    const Eigen::Vector3d v = velocities[index] +
                              Eigen::Vector3d(-0.01, 0.02, 0.005);
    std::copy(p.data(), p.data() + 3, estimated_positions[index].begin());
    std::copy(v.data(), v.data() + 3, estimated_velocities[index].begin());
    parameters.push_back(estimated_positions[index].data());
    parameters.push_back(estimated_velocities[index].data());
  }
  const int residual_dimension = factor.num_residuals();
  std::vector<double> residual(residual_dimension);
  std::vector<std::vector<double>> jacobian_storage(parameters.size());
  std::vector<double *> jacobians(parameters.size());
  for (std::size_t block = 0; block < parameters.size(); ++block) {
    jacobian_storage[block].resize(residual_dimension * 3);
    jacobians[block] = jacobian_storage[block].data();
  }
  require(factor.Evaluate(parameters.data(), residual.data(),
                          jacobians.data()),
          "FC trajectory Jacobian evaluation");
  constexpr double step = 1.0e-7;
  for (std::size_t block = 0; block < parameters.size(); ++block) {
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                   Eigen::RowMajor>>
        analytic(jacobian_storage[block].data(), residual_dimension, 3);
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<const double *> perturbed = parameters;
      Eigen::Vector3d x_plus, x_minus;
      const Eigen::Vector3d value =
          Eigen::Map<const Eigen::Vector3d>(parameters[block]);
      x_plus = value;
      x_minus = value;
      x_plus(axis) += step;
      x_minus(axis) -= step;
      perturbed[block] = x_plus.data();
      std::vector<double> plus_residual(residual_dimension);
      std::vector<double> minus_residual(residual_dimension);
      require(factor.Evaluate(perturbed.data(), plus_residual.data(), nullptr),
              "FC positive finite difference");
      perturbed[block] = x_minus.data();
      require(factor.Evaluate(perturbed.data(), minus_residual.data(), nullptr),
              "FC negative finite difference");
      const Eigen::VectorXd numeric =
          (Eigen::Map<const Eigen::VectorXd>(plus_residual.data(),
                                             residual_dimension) -
           Eigen::Map<const Eigen::VectorXd>(minus_residual.data(),
                                             residual_dimension)) /
          (2.0 * step);
      require((analytic.col(axis) - numeric).norm() < 2.0e-5,
              "FC p/v Jacobian must match finite difference");
    }
  }
}

void test_epipolar_factor_is_landmark_free_and_geometric() {
  const Eigen::Vector3d point_G(3.0, 1.0, 5.0);
  const Eigen::Vector2d previous(point_G.x() / point_G.z(),
                                 point_G.y() / point_G.z());
  const Eigen::Vector3d point_current =
      point_G - Eigen::Vector3d(1.0, 0.0, 0.0);
  const Eigen::Vector2d current(point_current.x() / point_current.z(),
                                point_current.y() / point_current.z());
  ov_msckf::p4::Factor_P4Epipolar factor(
      previous, current, Eigen::Matrix3d::Identity(),
      Eigen::Vector3d::Zero(), 1.0e-3);
  std::array<double, 4> q0{0.0, 0.0, 0.0, 1.0};
  std::array<double, 4> q1{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> p0{0.0, 0.0, 0.0};
  std::array<double, 3> p1{1.0, 0.0, 0.0};
  const double *parameters[4] = {q0.data(), p0.data(), q1.data(), p1.data()};
  double residual = 0.0;
  require(factor.Evaluate(parameters, &residual, nullptr) &&
              std::fabs(residual) < 1.0e-10,
          "exact two-view geometry must satisfy the epipolar factor");

  ov_msckf::p4::Factor_P4Epipolar perturbed(
      previous, current + Eigen::Vector2d(0.0, 0.02),
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), 1.0e-3);
  require(perturbed.Evaluate(parameters, &residual, nullptr) &&
              std::fabs(residual) > 1.0,
          "off-epipolar observation must produce a nonzero residual");

  std::array<std::vector<double>, 4> jacobian_storage;
  std::array<double *, 4> jacobians{};
  constexpr std::array<int, 4> block_sizes{4, 3, 4, 3};
  for (std::size_t block = 0; block < jacobian_storage.size(); ++block) {
    jacobian_storage[block].resize(block_sizes[block]);
    jacobians[block] = jacobian_storage[block].data();
  }
  require(perturbed.Evaluate(parameters, &residual, jacobians.data()),
          "epipolar Jacobian evaluation");
  constexpr double step = 1.0e-7;
  for (int block = 0; block < 4; ++block) {
    const bool quaternion_block = block == 0 || block == 2;
    for (int axis = 0; axis < 3; ++axis) {
      std::array<const double *, 4> finite_parameters{
          parameters[0], parameters[1], parameters[2], parameters[3]};
      Eigen::Vector4d q_plus, q_minus;
      Eigen::Vector3d x_plus, x_minus;
      if (quaternion_block) {
        q_plus = plus_jpl(parameters[block], axis, step);
        q_minus = plus_jpl(parameters[block], axis, -step);
        finite_parameters[block] = q_plus.data();
      } else {
        const Eigen::Vector3d value =
            Eigen::Map<const Eigen::Vector3d>(parameters[block]);
        x_plus = value;
        x_minus = value;
        x_plus(axis) += step;
        x_minus(axis) -= step;
        finite_parameters[block] = x_plus.data();
      }
      double plus_residual = 0.0;
      double minus_residual = 0.0;
      require(perturbed.Evaluate(finite_parameters.data(), &plus_residual,
                                 nullptr),
              "epipolar positive finite difference");
      finite_parameters[block] =
          quaternion_block ? q_minus.data() : x_minus.data();
      require(perturbed.Evaluate(finite_parameters.data(), &minus_residual,
                                 nullptr),
              "epipolar negative finite difference");
      const double numeric =
          (plus_residual - minus_residual) / (2.0 * step);
      require(std::fabs(jacobian_storage[block][axis] - numeric) < 1.0e-6,
              "epipolar q/p Jacobian must match JPL finite difference");
    }
    if (quaternion_block)
      require(std::fabs(jacobian_storage[block][3]) < 1.0e-14,
              "epipolar quaternion ambient scalar column must be zero");
  }
}

} // namespace

int main() {
  test_standard_cpi_keeps_bias_endpoints_independent();
  test_fc_covariance_is_correlated_and_density_invariant();
  test_epipolar_factor_is_landmark_free_and_geometric();
  std::cout << "formal P4 factor tests passed" << std::endl;
  return EXIT_SUCCESS;
}
