#include "ros_free/FlexBodyAttitudeObserver.h"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

using ov_msckf::FlexBodyAttitudeObserver;
using ov_msckf::FlexBodyAttitudeParameters;

namespace {

Eigen::Matrix3d rpy(double roll_deg, double pitch_deg, double yaw_deg) {
  const double scale = M_PI / 180.0;
  return (Eigen::AngleAxisd(yaw_deg * scale, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch_deg * scale, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll_deg * scale, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

void expect_near(double left, double right, double tolerance = 1e-10) {
  if (std::fabs(left - right) > tolerance)
    std::cerr << "expect_near failed: left=" << left << " right=" << right
              << " tolerance=" << tolerance << '\n';
  assert(std::fabs(left - right) <= tolerance);
}

} // namespace

int main() {
  const Eigen::Matrix3d mount = rpy(-178.8, -1.2, 87.0);
  {
    const Eigen::Matrix3d body = rpy(-2.0, 3.0, 12.0);
    const Eigen::Matrix3d d455 = body * mount.transpose();
    FlexBodyAttitudeObserver observer;
    const auto reset = observer.reset_at_initialization(10.0, d455, body);
    expect_near(reset.observed_flex_yaw_rad, 0.0);
    expect_near(reset.filtered_flex_yaw_rad, 0.0);
    expect_near(observer.output_flex_yaw_rad(), 0.0);
    assert((d455 * observer.nominal_R_I_from_B() - body).norm() < 1e-10);
  }

  {
    FlexBodyAttitudeObserver observer;
    Eigen::Matrix3d previous_body = rpy(-4.0, 0.0, 0.0);
    observer.reset_at_initialization(
        0.0, previous_body * mount.transpose(), previous_body);
    for (int yaw = 5; yaw <= 360; yaw += 5) {
      const Eigen::Matrix3d body =
          rpy(-4.0, 8.0 * std::sin(yaw * M_PI / 180.0), yaw);
      const Eigen::Matrix3d d455 = body * mount.transpose();
      observer.update(0.2 * yaw / 5.0, d455,
                      body.transpose() * previous_body);
      previous_body = body;
    }
    expect_near(observer.observed_flex_yaw_rad(), 0.0, 1e-9);
  }

  {
    // A fixed global left rotation must not alter the relative SO(3) flex
    // observation, including for non-commuting roll/pitch/yaw motion.
    const Eigen::Matrix3d global = rpy(13.0, -7.0, 31.0);
    const Eigen::Matrix3d body0 = rpy(-8.0, 5.0, 22.0);
    const Eigen::Matrix3d body1 = rpy(6.0, -11.0, 57.0);
    const Eigen::Matrix3d flex =
        Eigen::AngleAxisd(0.7 * M_PI / 180.0, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    FlexBodyAttitudeObserver plain;
    plain.reset_at_initialization(0.0, body0 * mount.transpose(), body0);
    plain.update(0.2, body1 * flex.transpose() * mount.transpose(),
                 body1.transpose() * body0);
    FlexBodyAttitudeObserver rotated;
    rotated.reset_at_initialization(
        0.0, global * body0 * mount.transpose(), global * body0);
    rotated.update(0.2, global * body1 * flex.transpose() * mount.transpose(),
                   body1.transpose() * body0);
    expect_near(rotated.observed_flex_yaw_rad(),
                plain.observed_flex_yaw_rad(), 1e-10);
  }

  {
    FlexBodyAttitudeParameters parameters;
    FlexBodyAttitudeObserver observer(parameters);
    observer.reset_at_initialization(0.0, Eigen::Matrix3d::Identity(),
                                     Eigen::Matrix3d::Identity());
    const auto sparse = observer.update(
        1.2, rpy(0.0, 0.0, 3.0), Eigen::Matrix3d::Identity());
    expect_near(sparse.observed_flex_yaw_rad, 3.0 * M_PI / 180.0);
    assert(std::fabs(sparse.applied_step_rad) <=
           parameters.maximum_rate_rad_s *
                   parameters.maximum_filter_timestep_s +
               1e-12);
    const double held = observer.filtered_flex_yaw_rad();
    observer.update(1.4, rpy(0.0, 0.0, 3.0),
                    Eigen::Matrix3d::Identity(), false);
    expect_near(observer.filtered_flex_yaw_rad(), held);

    observer.release_output(1.4, false);
    double previous_output = observer.output_flex_yaw_rad();
    for (int index = 0; index < 30; ++index) {
      const auto release = observer.release_output(1.4 + 0.02 * (index + 1));
      assert(std::fabs(release.applied_rate_rad_s) <=
             parameters.maximum_rate_rad_s + 1e-12);
      assert(std::fabs(observer.output_flex_yaw_rad() - previous_output) <=
             parameters.maximum_rate_rad_s * 0.02 + 1e-12);
      previous_output = observer.output_flex_yaw_rad();
    }
    const double output_before_hold = observer.output_flex_yaw_rad();
    observer.release_output(2.2, false);
    expect_near(observer.output_flex_yaw_rad(), output_before_hold);
  }

  {
    FlexBodyAttitudeObserver observer;
    Eigen::Matrix3d previous_body = Eigen::Matrix3d::Identity();
    observer.reset_at_initialization(
        0.0, previous_body * mount.transpose(), previous_body);
    const double flex_deg[] = {0.8, 1.4, 0.5, 0.0};
    for (int index = 0; index < 4; ++index) {
      const Eigen::Matrix3d body =
          rpy(2.0 * index, -index, 30.0 * (index + 1));
      const Eigen::Matrix3d flex =
          Eigen::AngleAxisd(flex_deg[index] * M_PI / 180.0,
                            Eigen::Vector3d::UnitZ())
              .toRotationMatrix();
      const Eigen::Matrix3d d455 =
          body * flex.transpose() * mount.transpose();
      observer.update(0.2 * (index + 1), d455,
                      body.transpose() * previous_body);
      previous_body = body;
    }
    expect_near(observer.observed_flex_yaw_rad(), 0.0, 1e-4);
  }

  std::cout << "FlexBodyAttitudeObserver tests passed\n";
  return 0;
}
