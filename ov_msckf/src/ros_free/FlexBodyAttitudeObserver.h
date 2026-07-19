/*
 * Output-only continuous yaw-flex observer for the ROS-free runner.
 * This class never owns or modifies an OpenVINS state.
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ov_msckf {

struct FlexBodyAttitudeParameters {
  double tau_s = 12.0;
  double huber_delta_rad = 1.5 * M_PI / 180.0;
  double maximum_rate_rad_s = 0.25 * M_PI / 180.0;
  double maximum_absolute_flex_rad = 6.0 * M_PI / 180.0;
  double maximum_filter_timestep_s = 0.45;
};

struct FlexBodyAttitudeUpdate {
  double timestamp_s = std::numeric_limits<double>::quiet_NaN();
  bool measurement_valid = false;
  double observed_increment_rad = 0.0;
  double observed_flex_yaw_rad = 0.0;
  double filtered_flex_yaw_rad = 0.0;
  double applied_step_rad = 0.0;
  double applied_rate_rad_s = 0.0;
};

struct FlexBodyAttitudeRelease {
  double timestamp_s = std::numeric_limits<double>::quiet_NaN();
  bool release_allowed = false;
  double output_flex_yaw_rad = 0.0;
  double applied_step_rad = 0.0;
  double applied_rate_rad_s = 0.0;
};

class FlexBodyAttitudeObserver {
public:
  explicit FlexBodyAttitudeObserver(
      const FlexBodyAttitudeParameters &parameters = {})
      : parameters_(parameters) {
    if (!(parameters_.tau_s > 0.0) ||
        !(parameters_.maximum_filter_timestep_s > 0.0))
      throw std::invalid_argument("invalid yaw-flex observer parameters");
  }

  FlexBodyAttitudeUpdate reset_at_initialization(
      double timestamp_s, const Eigen::Matrix3d &R_ItoG,
      const Eigen::Matrix3d &R_BtoG) {
    initialized_ = true;
    initialization_timestamp_s_ = timestamp_s;
    nominal_R_I_from_B_ = project_so3(R_ItoG.transpose() * R_BtoG);
    last_d455_R_ItoG_ = project_so3(R_ItoG);
    last_timestamp_s_ = timestamp_s;
    observed_flex_yaw_rad_ = 0.0;
    filtered_flex_yaw_rad_ = 0.0;
    output_flex_yaw_rad_ = 0.0;
    last_output_timestamp_s_ = timestamp_s;
    last_update_ = {};
    last_update_.timestamp_s = timestamp_s;
    return last_update_;
  }

  FlexBodyAttitudeRelease release_output(double timestamp_s,
                                         bool release_allowed = true) {
    require_initialized();
    FlexBodyAttitudeRelease release;
    release.timestamp_s = timestamp_s;
    release.release_allowed = release_allowed;
    const double dt = timestamp_s - last_output_timestamp_s_;
    if (!std::isfinite(dt) || dt <= 0.0) {
      release.output_flex_yaw_rad = output_flex_yaw_rad_;
      return release;
    }
    if (release_allowed) {
      const double innovation =
          wrap_rad(filtered_flex_yaw_rad_ - output_flex_yaw_rad_);
      const double maximum_step = parameters_.maximum_rate_rad_s * dt;
      const double step =
          std::max(-maximum_step, std::min(maximum_step, innovation));
      output_flex_yaw_rad_ += step;
      release.applied_step_rad = step;
      release.applied_rate_rad_s = step / dt;
    }
    last_output_timestamp_s_ = timestamp_s;
    release.output_flex_yaw_rad = output_flex_yaw_rad_;
    return release;
  }

  FlexBodyAttitudeUpdate update(
      double timestamp_s, const Eigen::Matrix3d &R_ItoG,
      const Eigen::Matrix3d &fc_delta_R_current_to_previous,
      bool measurement_valid = true) {
    if (!initialized_)
      throw std::logic_error("yaw-flex observer must be reset first");
    const double dt = timestamp_s - last_timestamp_s_;
    const bool valid = measurement_valid && std::isfinite(dt) && dt > 0.0 &&
                       R_ItoG.allFinite() &&
                       fc_delta_R_current_to_previous.allFinite();
    if (!valid) {
      last_update_ = {};
      last_update_.timestamp_s = timestamp_s;
      last_update_.observed_flex_yaw_rad = observed_flex_yaw_rad_;
      last_update_.filtered_flex_yaw_rad = filtered_flex_yaw_rad_;
      return last_update_;
    }

    // Both inputs are current-to-previous relative rotations.  Conjugate the
    // FC increment into the D455 IMU frame, form the SO(3) mismatch, and only
    // then project its Log vector onto the frozen aircraft-body yaw axis.
    // A constant global attitude or mount therefore cannot accumulate as
    // flex, and a reversed relative rotation subtracts the prior increment.
    const Eigen::Matrix3d d455_delta_R_current_to_previous =
        project_so3(R_ItoG.transpose() * last_d455_R_ItoG_);
    const Eigen::Matrix3d predicted_fc_delta_in_imu = project_so3(
        nominal_R_I_from_B_ * fc_delta_R_current_to_previous *
        nominal_R_I_from_B_.transpose());
    const Eigen::Matrix3d mismatch = project_so3(
        d455_delta_R_current_to_previous *
        predicted_fc_delta_in_imu.transpose());
    const Eigen::AngleAxisd mismatch_angle_axis(mismatch);
    const Eigen::Vector3d mismatch_log_imu =
        mismatch_angle_axis.angle() * mismatch_angle_axis.axis();
    // The relative matrices above are current-to-previous.  Negate the Log
    // projection so a positive D455-minus-FC forward yaw change produces a
    // positive flex increment, matching the output convention D455-flex.
    const double observed_increment =
        -(nominal_R_I_from_B_.transpose() * mismatch_log_imu).z();
    observed_flex_yaw_rad_ += observed_increment;

    const double filter_dt =
        std::min(dt, parameters_.maximum_filter_timestep_s);
    const double alpha = 1.0 - std::exp(-filter_dt / parameters_.tau_s);
    const double innovation =
        wrap_rad(observed_flex_yaw_rad_ - filtered_flex_yaw_rad_);
    const double robust_innovation =
        std::max(-parameters_.huber_delta_rad,
                 std::min(parameters_.huber_delta_rad, innovation));
    double step = alpha * robust_innovation;
    const double maximum_step = parameters_.maximum_rate_rad_s * filter_dt;
    step = std::max(-maximum_step, std::min(maximum_step, step));
    const double updated = std::max(
        -parameters_.maximum_absolute_flex_rad,
        std::min(parameters_.maximum_absolute_flex_rad,
                 filtered_flex_yaw_rad_ + step));
    step = updated - filtered_flex_yaw_rad_;
    filtered_flex_yaw_rad_ = updated;

    last_d455_R_ItoG_ = project_so3(R_ItoG);
    last_timestamp_s_ = timestamp_s;
    last_update_.timestamp_s = timestamp_s;
    last_update_.measurement_valid = true;
    last_update_.observed_increment_rad = observed_increment;
    last_update_.observed_flex_yaw_rad = observed_flex_yaw_rad_;
    last_update_.filtered_flex_yaw_rad = filtered_flex_yaw_rad_;
    last_update_.applied_step_rad = step;
    last_update_.applied_rate_rad_s = step / filter_dt;
    return last_update_;
  }

  Eigen::Matrix3d nominal_body_attitude(
      const Eigen::Matrix3d &R_ItoG) const {
    require_initialized();
    return project_so3(R_ItoG * nominal_R_I_from_B_);
  }

  Eigen::Matrix3d aircraft_body_attitude(
      const Eigen::Matrix3d &R_ItoG) const {
    const Eigen::Matrix3d nominal = nominal_body_attitude(R_ItoG);
    const Eigen::Vector3d rpy = rpy_from_R_to_G(nominal);
    return R_from_rpy(rpy.x(), rpy.y(),
                      wrap_rad(rpy.z() - output_flex_yaw_rad_));
  }

  bool initialized() const { return initialized_; }
  double observed_flex_yaw_rad() const { return observed_flex_yaw_rad_; }
  double filtered_flex_yaw_rad() const { return filtered_flex_yaw_rad_; }
  double output_flex_yaw_rad() const { return output_flex_yaw_rad_; }
  const Eigen::Matrix3d &nominal_R_I_from_B() const {
    return nominal_R_I_from_B_;
  }

  static double wrap_rad(double angle) {
    while (angle > M_PI)
      angle -= 2.0 * M_PI;
    while (angle < -M_PI)
      angle += 2.0 * M_PI;
    return angle;
  }

  static double yaw_from_R_to_G(const Eigen::Matrix3d &R_to_G) {
    return std::atan2(R_to_G(1, 0), R_to_G(0, 0));
  }

private:
  static Eigen::Matrix3d project_so3(const Eigen::Matrix3d &matrix) {
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) =
        (svd.matrixU() * svd.matrixV().transpose()).determinant();
    return svd.matrixU() * correction * svd.matrixV().transpose();
  }

  static Eigen::Vector3d rpy_from_R_to_G(const Eigen::Matrix3d &matrix) {
    const double pitch = std::asin(
        std::max(-1.0, std::min(1.0, -matrix(2, 0))));
    double roll = 0.0;
    double yaw = 0.0;
    if (std::fabs(std::cos(pitch)) > 1e-9) {
      roll = std::atan2(matrix(2, 1), matrix(2, 2));
      yaw = std::atan2(matrix(1, 0), matrix(0, 0));
    } else {
      roll = std::atan2(-matrix(1, 2), matrix(1, 1));
      yaw = std::atan2(-matrix(0, 1), matrix(1, 1));
    }
    return {roll, pitch, yaw};
  }

  static Eigen::Matrix3d R_from_rpy(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
  }

  void require_initialized() const {
    if (!initialized_)
      throw std::logic_error("yaw-flex observer is not initialized");
  }

  FlexBodyAttitudeParameters parameters_;
  bool initialized_ = false;
  double initialization_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double last_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_output_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d nominal_R_I_from_B_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d last_d455_R_ItoG_ = Eigen::Matrix3d::Identity();
  double observed_flex_yaw_rad_ = 0.0;
  double filtered_flex_yaw_rad_ = 0.0;
  double output_flex_yaw_rad_ = 0.0;
  FlexBodyAttitudeUpdate last_update_;
};

} // namespace ov_msckf
