/* Correlated FC position/velocity trajectory factor for formal P4. */

#ifndef OV_MSCKF_P4_FACTOR_FC_TRAJECTORY_H
#define OV_MSCKF_P4_FACTOR_FC_TRAJECTORY_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

#include <vector>

namespace ov_msckf {
namespace p4 {

/**
 * The FC navigation error state is e=[dp, dv] with Phi=I. Its first-row
 * covariance is (1-process_fraction)*P_terminal and independent process
 * increments integrate to process_fraction*P_terminal over the complete
 * physical window.  Consequently the terminal covariance is exactly
 * P_terminal.
 *
 * Residual order is [terminal absolute PV, chronological PV increments].
 * The residual is whitened by the full transformed joint covariance.  In
 * particular the terminal residual and every increment are correlated; they
 * are never installed as independent Ceres residual blocks. FC attitude is
 * intentionally absent: formal P4 installs one terminal yaw gauge, while CPI
 * and image reprojection estimate roll/pitch and relative attitude.
 */
class Factor_P4FcTrajectory final : public ceres::CostFunction {
public:
  Factor_P4FcTrajectory(
      const std::vector<double> &timestamps,
      const std::vector<Eigen::Vector3d> &target_positions_G,
      const std::vector<Eigen::Vector3d> &target_velocities_G,
      const Eigen::Matrix<double, 6, 6> &terminal_covariance,
      double process_variance_fraction);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

  const Eigen::MatrixXd &absolute_measurement_covariance() const {
    return absolute_covariance_;
  }
  const Eigen::MatrixXd &residual_covariance() const {
    return residual_covariance_;
  }
  const Eigen::MatrixXd &residual_transform() const { return transform_; }

  static Eigen::MatrixXd buildAbsoluteMeasurementCovariance(
      const std::vector<double> &timestamps,
      const Eigen::Matrix<double, 6, 6> &terminal_covariance,
      double process_variance_fraction);
  static Eigen::MatrixXd buildTerminalIncrementTransform(std::size_t count);

private:
  Eigen::VectorXd evaluateRaw(double const *const *parameters) const;

  std::vector<double> timestamps_;
  std::vector<Eigen::Vector3d> target_positions_G_;
  std::vector<Eigen::Vector3d> target_velocities_G_;
  Eigen::MatrixXd absolute_covariance_;
  Eigen::MatrixXd transform_;
  Eigen::MatrixXd residual_covariance_;
  Eigen::MatrixXd whitener_;
};

} // namespace p4
} // namespace ov_msckf

#endif // OV_MSCKF_P4_FACTOR_FC_TRAJECTORY_H
