#include "update/UpdaterFlexRelativeYaw.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/quat_ops.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ov_msckf {
namespace {

Eigen::Matrix3d project_so3(const Eigen::Matrix3d &matrix) {
  const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
  correction(2, 2) =
      (svd.matrixU() * svd.matrixV().transpose()).determinant();
  return svd.matrixU() * correction * svd.matrixV().transpose();
}

double yaw_from_R_GtoI(const Eigen::Matrix3d &R_GtoI) {
  const Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
  return std::atan2(R_ItoG(1, 0), R_ItoG(0, 0));
}

double wrap_rad(double value) {
  return std::atan2(std::sin(value), std::cos(value));
}

} // namespace

UpdaterFlexRelativeYaw::UpdaterFlexRelativeYaw(
    const FlexRelativeYawParameters &parameters)
    : parameters_(parameters) {
  if (!(parameters_.measurement_sigma_rad > 0.0) ||
      !(parameters_.nis_limit > 0.0) ||
      !(parameters_.finite_difference_step > 0.0))
    throw std::invalid_argument("invalid flex relative-yaw parameters");
}

void UpdaterFlexRelativeYaw::reset_nominal_mount(
    const Eigen::Matrix3d &nominal_R_I_from_B) {
  if (!nominal_R_I_from_B.allFinite())
    throw std::invalid_argument("non-finite nominal flex mount");
  nominal_R_I_from_B_ = project_so3(nominal_R_I_from_B);
  accepted_count_ = 0;
  initialized_ = true;
}

Eigen::Matrix3d UpdaterFlexRelativeYaw::mount_from_flex(
    const Eigen::Matrix3d &nominal_R_I_from_B, double flex_yaw_rad) {
  return project_so3(
      nominal_R_I_from_B *
      ov_core::exp_so3(Eigen::Vector3d(0.0, 0.0, flex_yaw_rad)));
}

double UpdaterFlexRelativeYaw::prediction(
    const Eigen::Matrix3d &anchor_R_GtoI,
    const Eigen::Matrix3d &current_R_GtoI, double anchor_flex_yaw_rad,
    double current_flex_yaw_rad,
    const Eigen::Matrix3d &fc_delta_R_current_to_previous,
    const Eigen::Matrix3d &nominal_R_I_from_B) {
  const Eigen::Matrix3d mount_anchor =
      mount_from_flex(nominal_R_I_from_B, anchor_flex_yaw_rad);
  const Eigen::Matrix3d mount_current =
      mount_from_flex(nominal_R_I_from_B, current_flex_yaw_rad);
  const Eigen::Matrix3d observed =
      project_so3(current_R_GtoI * anchor_R_GtoI.transpose());
  const Eigen::Matrix3d predicted = project_so3(
      mount_current * fc_delta_R_current_to_previous *
      mount_anchor.transpose());
  const Eigen::Vector3d mismatch_log =
      ov_core::log_so3(project_so3(observed * predicted.transpose()));
  return (mount_current.transpose() * mismatch_log).z();
}

Eigen::Matrix<double, 1, 8> UpdaterFlexRelativeYaw::numeric_jacobian(
    const Eigen::Matrix3d &anchor_R_GtoI,
    const Eigen::Matrix3d &current_R_GtoI, double anchor_flex_yaw_rad,
    double current_flex_yaw_rad,
    const Eigen::Matrix3d &fc_delta_R_current_to_previous,
    const Eigen::Matrix3d &nominal_R_I_from_B, double step) {
  if (!(step > 0.0))
    throw std::invalid_argument("finite-difference step must be positive");
  Eigen::Matrix<double, 1, 8> H;
  for (int column = 0; column < 8; ++column) {
    Eigen::Matrix3d Ra_plus = anchor_R_GtoI;
    Eigen::Matrix3d Ra_minus = anchor_R_GtoI;
    Eigen::Matrix3d Rc_plus = current_R_GtoI;
    Eigen::Matrix3d Rc_minus = current_R_GtoI;
    double fa_plus = anchor_flex_yaw_rad;
    double fa_minus = anchor_flex_yaw_rad;
    double fc_plus = current_flex_yaw_rad;
    double fc_minus = current_flex_yaw_rad;
    if (column < 3) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(column) = step;
      Ra_plus = ov_core::exp_so3(-delta) * anchor_R_GtoI;
      Ra_minus = ov_core::exp_so3(delta) * anchor_R_GtoI;
    } else if (column < 6) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta(column - 3) = step;
      Rc_plus = ov_core::exp_so3(-delta) * current_R_GtoI;
      Rc_minus = ov_core::exp_so3(delta) * current_R_GtoI;
    } else if (column == 6) {
      fa_plus += step;
      fa_minus -= step;
    } else {
      fc_plus += step;
      fc_minus -= step;
    }
    const double plus = prediction(
        Ra_plus, Rc_plus, fa_plus, fc_plus,
        fc_delta_R_current_to_previous, nominal_R_I_from_B);
    const double minus = prediction(
        Ra_minus, Rc_minus, fa_minus, fc_minus,
        fc_delta_R_current_to_previous, nominal_R_I_from_B);
    H(column) = wrap_rad(plus - minus) / (2.0 * step);
  }
  return H;
}

FlexRelativeYawDiagnostics UpdaterFlexRelativeYaw::update(
    const std::shared_ptr<State> &state, double anchor_timestamp_s,
    double current_timestamp_s,
    const Eigen::Matrix3d &fc_delta_R_current_to_previous) {
  FlexRelativeYawDiagnostics diagnostics;
  diagnostics.anchor_timestamp_s = anchor_timestamp_s;
  diagnostics.current_timestamp_s = current_timestamp_s;
  if (!initialized_) {
    diagnostics.decision = "reject_not_initialized";
    return diagnostics;
  }
  if (!state || !state->_options.use_flex_yaw_state ||
      !fc_delta_R_current_to_previous.allFinite() ||
      !(current_timestamp_s > anchor_timestamp_s)) {
    diagnostics.decision = "reject_invalid_input";
    return diagnostics;
  }
  const auto pose_anchor_it = state->_clones_IMU.find(anchor_timestamp_s);
  const auto pose_current_it = state->_clones_IMU.find(current_timestamp_s);
  const auto flex_anchor_it =
      state->_clones_flex_yaw.find(anchor_timestamp_s);
  const auto flex_current_it =
      state->_clones_flex_yaw.find(current_timestamp_s);
  if (pose_anchor_it == state->_clones_IMU.end() ||
      pose_current_it == state->_clones_IMU.end() ||
      flex_anchor_it == state->_clones_flex_yaw.end() ||
      flex_current_it == state->_clones_flex_yaw.end()) {
    diagnostics.decision = "reject_missing_clone";
    return diagnostics;
  }

  const auto &pose_anchor = pose_anchor_it->second;
  const auto &pose_current = pose_current_it->second;
  const auto &flex_anchor = flex_anchor_it->second;
  const auto &flex_current = flex_current_it->second;
  const double fa = flex_anchor->value()(0);
  const double fc = flex_current->value()(0);
  diagnostics.flex_anchor_before_rad = fa;
  diagnostics.flex_current_before_rad = fc;
  diagnostics.prediction_rad = prediction(
      pose_anchor->Rot(), pose_current->Rot(), fa, fc,
      fc_delta_R_current_to_previous, nominal_R_I_from_B_);
  diagnostics.residual_rad = -diagnostics.prediction_rad;

  const Eigen::Matrix<double, 1, 8> H = numeric_jacobian(
      pose_anchor->Rot(), pose_current->Rot(), fa, fc,
      fc_delta_R_current_to_previous, nominal_R_I_from_B_,
      parameters_.finite_difference_step);
  if (!H.allFinite() || !std::isfinite(diagnostics.residual_rad)) {
    diagnostics.decision = "reject_nonfinite_model";
    return diagnostics;
  }

  std::vector<std::shared_ptr<ov_type::Type>> order{
      pose_anchor->q(), pose_current->q(), flex_anchor, flex_current};
  const Eigen::MatrixXd P_small =
      StateHelper::get_marginal_covariance(state, order);
  const double measurement_variance =
      parameters_.measurement_sigma_rad * parameters_.measurement_sigma_rad;
  diagnostics.innovation_variance =
      (H * P_small * H.transpose())(0, 0) + measurement_variance;
  if (!std::isfinite(diagnostics.innovation_variance) ||
      diagnostics.innovation_variance <= 0.0) {
    diagnostics.decision = "reject_invalid_innovation";
    return diagnostics;
  }
  diagnostics.nis = diagnostics.residual_rad * diagnostics.residual_rad /
                    diagnostics.innovation_variance;
  if (diagnostics.nis > parameters_.nis_limit) {
    diagnostics.decision = "reject_nis";
    return diagnostics;
  }

  const double yaw_before = yaw_from_R_GtoI(state->_imu->Rot());
  Eigen::VectorXd residual(1);
  residual(0) = diagnostics.residual_rad;
  const Eigen::MatrixXd P_full = StateHelper::get_full_covariance(state);
  Eigen::VectorXd H_full = Eigen::VectorXd::Zero(P_full.rows());
  int local_column = 0;
  for (const auto &variable : order) {
    H_full.segment(variable->id(), variable->size()) =
        H.block(0, local_column, 1, variable->size()).transpose();
    local_column += variable->size();
  }
  const Eigen::VectorXd gain_numerator = P_full * H_full;
  const double innovation_variance =
      H_full.dot(gain_numerator) + measurement_variance;
  Eigen::VectorXd gain = gain_numerator / innovation_variance;

  // A one-dimensional relative-yaw/flex factor may correct orientations and
  // flex states, but must not inject an instantaneous translation, velocity,
  // bias, landmark, or calibration correction through incidental cross-
  // covariance. Keep the statistically optimal rows for every active pose
  // orientation and flex level, zero all other rows, then use the matched
  // Joseph covariance update for the masked gain.
  Eigen::VectorXd masked_gain = Eigen::VectorXd::Zero(gain.rows());
  masked_gain.segment(state->_imu->q()->id(), 3) =
      gain.segment(state->_imu->q()->id(), 3);
  for (const auto &entry : state->_clones_IMU) {
    masked_gain.segment(entry.second->q()->id(), 3) =
        gain.segment(entry.second->q()->id(), 3);
  }
  masked_gain(state->_flex_yaw->id()) = gain(state->_flex_yaw->id());
  for (const auto &entry : state->_clones_flex_yaw)
    masked_gain(entry.second->id()) = gain(entry.second->id());

  const Eigen::Vector3d position_before = state->_imu->pos();
  const Eigen::Vector3d velocity_before = state->_imu->vel();
  const Eigen::Vector3d gyro_bias_before = state->_imu->bias_g();
  const Eigen::Vector3d accel_bias_before = state->_imu->bias_a();
  StateHelper::JosephUpdateHealth health;
  const bool check_psd = accepted_count_ == 0 || accepted_count_ % 100 == 0;
  const bool applied = StateHelper::EKFUpdateJosephChecked(
      state, masked_gain, H_full, measurement_variance,
      diagnostics.residual_rad, check_psd, &health);
  if (!applied) {
    diagnostics.decision = "reject_numerical";
    return diagnostics;
  }
  ++accepted_count_;
  const double yaw_after = yaw_from_R_GtoI(state->_imu->Rot());
  diagnostics.flex_current_after_rad = flex_current->value()(0);
  diagnostics.posterior_prediction_rad = prediction(
      pose_anchor->Rot(), pose_current->Rot(), flex_anchor->value()(0),
      flex_current->value()(0), fc_delta_R_current_to_previous,
      nominal_R_I_from_B_);
  diagnostics.state_yaw_delta_rad = wrap_rad(yaw_after - yaw_before);
  diagnostics.position_delta_norm_m =
      (state->_imu->pos() - position_before).norm();
  diagnostics.velocity_delta_norm_mps =
      (state->_imu->vel() - velocity_before).norm();
  diagnostics.gyro_bias_delta_norm_radps =
      (state->_imu->bias_g() - gyro_bias_before).norm();
  diagnostics.accel_bias_delta_norm_mps2 =
      (state->_imu->bias_a() - accel_bias_before).norm();
  diagnostics.accepted = true;
  diagnostics.decision = "accepted";
  return diagnostics;
}

} // namespace ov_msckf
