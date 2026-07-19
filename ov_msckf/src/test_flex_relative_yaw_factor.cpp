#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterFlexRelativeYaw.h"
#include "utils/quat_ops.h"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

using ov_msckf::FlexRelativeYawParameters;
using ov_msckf::State;
using ov_msckf::StateHelper;
using ov_msckf::StateOptions;
using ov_msckf::UpdaterFlexRelativeYaw;

namespace {

Eigen::Matrix3d rpy(double roll_deg, double pitch_deg, double yaw_deg) {
  const double d = M_PI / 180.0;
  return (Eigen::AngleAxisd(yaw_deg * d, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch_deg * d, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll_deg * d, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

void expect_near(double left, double right, double tolerance) {
  if (std::fabs(left - right) > tolerance)
    std::cerr << "expect_near failed left=" << left << " right=" << right
              << " tol=" << tolerance << '\n';
  assert(std::fabs(left - right) <= tolerance);
}

void set_imu_attitude(const std::shared_ptr<State> &state,
                      const Eigen::Matrix3d &R_GtoI) {
  Eigen::VectorXd value = state->_imu->value();
  value.head<4>() = ov_core::rot_2_quat(R_GtoI);
  state->_imu->set_value(value);
}

} // namespace

int main() {
  const Eigen::Matrix3d M0 = rpy(-178.8, -1.2, 87.0);
  const Eigen::Matrix3d B0 = rpy(-8.0, 7.0, 23.0);
  const Eigen::Matrix3d B1 = rpy(5.0, -12.0, 61.0);
  const Eigen::Matrix3d dB = B1.transpose() * B0;

  // Rigid non-commuting three-axis motion must close exactly.
  const Eigen::Matrix3d I0 = B0 * M0.transpose();
  const Eigen::Matrix3d I1 = B1 * M0.transpose();
  expect_near(UpdaterFlexRelativeYaw::prediction(
                  I0.transpose(), I1.transpose(), 0.0, 0.0, dB, M0),
              0.0, 1e-10);

  // FC absolute global yaw is absent from the delta by construction.
  const Eigen::Matrix3d global = rpy(11.0, -4.0, 30.0);
  const Eigen::Matrix3d dB_global =
      (global * B1).transpose() * (global * B0);
  assert((dB_global - dB).norm() < 1e-12);

  // Quaternion sign changes do not alter the reconstructed endpoint rotation.
  Eigen::Quaterniond q(B1);
  Eigen::Quaterniond q_neg(-q.coeffs());
  assert((q.normalized().toRotationMatrix() -
          q_neg.normalized().toRotationMatrix())
             .norm() < 1e-12);

  // A flex level change belongs to f_c-f_a. Supplying both endpoint levels
  // cancels it; reversing the change cancels the accumulated level again.
  const double f1 = 0.6 * M_PI / 180.0;
  const Eigen::Matrix3d M1 =
      UpdaterFlexRelativeYaw::mount_from_flex(M0, f1);
  const Eigen::Matrix3d I1_flex = B1 * M1.transpose();
  const double unmodelled = UpdaterFlexRelativeYaw::prediction(
      I0.transpose(), I1_flex.transpose(), 0.0, 0.0, dB, M0);
  assert(std::fabs(unmodelled) > 0.4 * M_PI / 180.0);
  expect_near(UpdaterFlexRelativeYaw::prediction(
                  I0.transpose(), I1_flex.transpose(), 0.0, f1, dB, M0),
              0.0, 1e-10);
  const Eigen::Matrix3d B2 = rpy(-3.0, 9.0, 88.0);
  const Eigen::Matrix3d dB2 = B2.transpose() * B1;
  const Eigen::Matrix3d I2 = B2 * M0.transpose();
  expect_near(UpdaterFlexRelativeYaw::prediction(
                  I1_flex.transpose(), I2.transpose(), f1, 0.0, dB2, M0),
              0.0, 1e-10);

  // Jacobian uses the repository's left JPL error convention. Verify against
  // an independent one-sided perturbation for all eight columns.
  const auto H = UpdaterFlexRelativeYaw::numeric_jacobian(
      I0.transpose(), I1_flex.transpose(), 0.0, 0.0, dB, M0);
  const double h0 = UpdaterFlexRelativeYaw::prediction(
      I0.transpose(), I1_flex.transpose(), 0.0, 0.0, dB, M0);
  const double eps = 2e-7;
  for (int column = 0; column < 8; ++column) {
    Eigen::Matrix3d Ra = I0.transpose();
    Eigen::Matrix3d Rc = I1_flex.transpose();
    double fa = 0.0, fc = 0.0;
    if (column < 3) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(column) = eps;
      Ra = ov_core::exp_so3(-delta) * Ra;
    } else if (column < 6) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(column - 3) = eps;
      Rc = ov_core::exp_so3(-delta) * Rc;
    } else if (column == 6) {
      fa += eps;
    } else {
      fc += eps;
    }
    const double hp = UpdaterFlexRelativeYaw::prediction(
        Ra, Rc, fa, fc, dB, M0);
    expect_near(H(column), (hp - h0) / eps, 2e-5);
  }

  // Actual EKF update: residual and covariance must decrease while preserving
  // finite, symmetric, positive-semidefinite covariance.
  StateOptions options;
  options.use_flex_yaw_state = true;
  auto state = std::make_shared<State>(options);
  set_imu_attitude(state, I0.transpose());
  state->_timestamp = 10.0;
  StateHelper::augment_clone(state, Eigen::Vector3d::Zero());
  set_imu_attitude(state, I1_flex.transpose());
  state->_timestamp = 10.4;
  StateHelper::augment_clone(state, Eigen::Vector3d::Zero());

  FlexRelativeYawParameters parameters;
  parameters.measurement_sigma_rad = 0.35 * M_PI / 180.0;
  parameters.nis_limit = 100.0;
  UpdaterFlexRelativeYaw updater(parameters);
  updater.reset_nominal_mount(M0);
  const double before = std::fabs(UpdaterFlexRelativeYaw::prediction(
      state->_clones_IMU.at(10.0)->Rot(),
      state->_clones_IMU.at(10.4)->Rot(),
      state->_clones_flex_yaw.at(10.0)->value()(0),
      state->_clones_flex_yaw.at(10.4)->value()(0), dB, M0));
  const Eigen::Vector3d position_before = state->_imu->pos();
  const Eigen::Vector3d velocity_before = state->_imu->vel();
  const Eigen::Vector3d gyro_bias_before = state->_imu->bias_g();
  const Eigen::Vector3d accel_bias_before = state->_imu->bias_a();
  const auto diagnostics = updater.update(state, 10.0, 10.4, dB);
  assert(diagnostics.accepted);
  const double after = std::fabs(UpdaterFlexRelativeYaw::prediction(
      state->_clones_IMU.at(10.0)->Rot(),
      state->_clones_IMU.at(10.4)->Rot(),
      state->_clones_flex_yaw.at(10.0)->value()(0),
      state->_clones_flex_yaw.at(10.4)->value()(0), dB, M0));
  assert(after < before);
  assert((state->_imu->pos() - position_before).norm() == 0.0);
  assert((state->_imu->vel() - velocity_before).norm() == 0.0);
  assert((state->_imu->bias_g() - gyro_bias_before).norm() == 0.0);
  assert((state->_imu->bias_a() - accel_bias_before).norm() == 0.0);
  assert(diagnostics.position_delta_norm_m == 0.0);
  assert(diagnostics.velocity_delta_norm_mps == 0.0);
  assert(diagnostics.gyro_bias_delta_norm_radps == 0.0);
  assert(diagnostics.accel_bias_delta_norm_mps2 == 0.0);
  const Eigen::MatrixXd P = StateHelper::get_full_covariance(state);
  assert(P.allFinite());
  assert((P - P.transpose()).norm() < 1e-10);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(P);
  assert(solver.eigenvalues().minCoeff() > -1e-12);

  std::cout << "Flex relative-yaw factor tests passed\n";
  return 0;
}
