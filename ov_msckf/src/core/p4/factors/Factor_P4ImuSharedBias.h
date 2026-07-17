/*
 * OpenVINS P4 formal initializer.
 *
 * This adapter preserves the complete OpenVINS 15-D CPI likelihood while a
 * finite P4 window owns one shared gyroscope bias and one shared accelerometer
 * bias.  It is intentionally not a reduced 9-D reimplementation: the two bias
 * endpoint parameter blocks of Factor_ImuCPIv1 are tied to the same variables
 * and their Jacobians are summed.
 */

#ifndef OV_MSCKF_P4_FACTOR_IMU_SHARED_BIAS_H
#define OV_MSCKF_P4_FACTOR_IMU_SHARED_BIAS_H

#include "ceres/Factor_ImuCPIv1.h"

#include <ceres/ceres.h>

namespace ov_msckf {
namespace p4 {

class Factor_P4ImuSharedBias final : public ceres::CostFunction {
public:
  Factor_P4ImuSharedBias(
      double delta_time, Eigen::Vector3d &gravity,
      Eigen::Vector3d &alpha, Eigen::Vector3d &beta,
      Eigen::Vector4d &q_k_to_k1, Eigen::Vector3d &ba_linearization,
      Eigen::Vector3d &bg_linearization, Eigen::Matrix3d &J_q,
      Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha,
      Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha,
      Eigen::Matrix<double, 15, 15> &covariance);

  /**
   * Parameter order:
   *   q_k, shared_bg, v_k, shared_ba, p_k, q_k1, v_k1, p_k1.
   */
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

private:
  ov_init::Factor_ImuCPIv1 base_factor_;
};

} // namespace p4
} // namespace ov_msckf

#endif // OV_MSCKF_P4_FACTOR_IMU_SHARED_BIAS_H
