/* Experimental flex-aware FC relative-yaw EKF factor (default off). */
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <memory>

namespace ov_msckf {

class State;

struct FlexRelativeYawParameters {
  double measurement_sigma_rad = 0.35 * M_PI / 180.0;
  double nis_limit = 9.0;
  double finite_difference_step = 1e-7;
};

struct FlexRelativeYawDiagnostics {
  double anchor_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double current_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  bool accepted = false;
  const char *decision = "not_run";
  double prediction_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_rad = std::numeric_limits<double>::quiet_NaN();
  double innovation_variance = std::numeric_limits<double>::quiet_NaN();
  double nis = std::numeric_limits<double>::quiet_NaN();
  double flex_anchor_before_rad = std::numeric_limits<double>::quiet_NaN();
  double flex_current_before_rad = std::numeric_limits<double>::quiet_NaN();
  double flex_current_after_rad = std::numeric_limits<double>::quiet_NaN();
  double posterior_prediction_rad = std::numeric_limits<double>::quiet_NaN();
  double state_yaw_delta_rad = std::numeric_limits<double>::quiet_NaN();
  double position_delta_norm_m = std::numeric_limits<double>::quiet_NaN();
  double velocity_delta_norm_mps = std::numeric_limits<double>::quiet_NaN();
  double gyro_bias_delta_norm_radps = std::numeric_limits<double>::quiet_NaN();
  double accel_bias_delta_norm_mps2 = std::numeric_limits<double>::quiet_NaN();
};

class UpdaterFlexRelativeYaw {
public:
  explicit UpdaterFlexRelativeYaw(
      const FlexRelativeYawParameters &parameters = {});

  void reset_nominal_mount(const Eigen::Matrix3d &nominal_R_I_from_B);
  bool initialized() const { return initialized_; }

  FlexRelativeYawDiagnostics update(
      const std::shared_ptr<State> &state, double anchor_timestamp_s,
      double current_timestamp_s,
      const Eigen::Matrix3d &fc_delta_R_current_to_previous);

  static Eigen::Matrix3d mount_from_flex(
      const Eigen::Matrix3d &nominal_R_I_from_B, double flex_yaw_rad);

  static double prediction(
      const Eigen::Matrix3d &anchor_R_GtoI,
      const Eigen::Matrix3d &current_R_GtoI, double anchor_flex_yaw_rad,
      double current_flex_yaw_rad,
      const Eigen::Matrix3d &fc_delta_R_current_to_previous,
      const Eigen::Matrix3d &nominal_R_I_from_B);

  static Eigen::Matrix<double, 1, 8> numeric_jacobian(
      const Eigen::Matrix3d &anchor_R_GtoI,
      const Eigen::Matrix3d &current_R_GtoI, double anchor_flex_yaw_rad,
      double current_flex_yaw_rad,
      const Eigen::Matrix3d &fc_delta_R_current_to_previous,
      const Eigen::Matrix3d &nominal_R_I_from_B, double step = 1e-7);

private:
  FlexRelativeYawParameters parameters_;
  bool initialized_ = false;
  size_t accepted_count_ = 0;
  Eigen::Matrix3d nominal_R_I_from_B_ = Eigen::Matrix3d::Identity();
};

} // namespace ov_msckf
