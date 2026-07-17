/* Landmark-free monocular Sampson factor for formal P4. */

#ifndef OV_MSCKF_P4_FACTOR_EPIPOLAR_H
#define OV_MSCKF_P4_FACTOR_EPIPOLAR_H

#include <Eigen/Dense>
#include <ceres/ceres.h>

namespace ov_msckf {
namespace p4 {

class Factor_P4Epipolar final : public ceres::CostFunction {
public:
  Factor_P4Epipolar(const Eigen::Vector2d &normalized_previous,
                    const Eigen::Vector2d &normalized_current,
                    const Eigen::Matrix3d &R_ItoC,
                    const Eigen::Vector3d &p_IinC,
                    double normalized_measurement_sigma);

  /** Parameter order: q_GtoI_previous, p_Iprevious_inG,
   *                   q_GtoI_current,  p_Icurrent_inG. */
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const override;

private:
  double evaluateResidual(double const *const *parameters) const;
  static Eigen::Vector4d plusJpl(const Eigen::Vector4d &quaternion,
                                const Eigen::Vector3d &delta);

  Eigen::Vector3d previous_bearing_C_;
  Eigen::Vector3d current_bearing_C_;
  Eigen::Matrix3d R_ItoC_;
  Eigen::Vector3d p_IinC_;
  double normalized_sigma_ = 1.0;
};

} // namespace p4
} // namespace ov_msckf

#endif // OV_MSCKF_P4_FACTOR_EPIPOLAR_H
