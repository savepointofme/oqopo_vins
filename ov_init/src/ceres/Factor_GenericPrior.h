/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_INIT_CERES_GENERICPRIOR_H
#define OV_INIT_CERES_GENERICPRIOR_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief Factor for generic state priors for specific types.
 *
 * This is a general factor which handles state priors which have non-zero linear errors.
 * In general a unitary factor will have zero error when it is created, thus this extra term can be ignored.
 * But if performing marginalization, this can be non-zero. See the following paper Section 3.2 Eq. 25-35
 * https://journals.sagepub.com/doi/full/10.1177/0278364919835021
 *
 * [中文] Factor_GenericPrior — 通用的 "带非零线性项" 状态先验因子
 *
 *   场景 1 (本仓用到的): DynamicInitializer 初始化时, 为了锁定 4 自由度不可观 (yaw + p_IiinG),
 *     对第 1 帧的 (q_yaw, p, bg, ba) 提供一个信息矩阵非常大的先验, 让 Ceres 不会让它们随意漂。
 *     此时 prior_grad = 0, 因为"在 x_lin 处刚好残差为 0"。
 *
 *   场景 2 (通用): 边缘化后会得到 (A, b), A^T A 是 prior information, A^T b 是 prior gradient。
 *     此时 prior_grad 不为 0 (因为 x_lin 不是最优点了), 代入 cost 公式 ||A(x-x_lin)+b||^2 依然正确。
 *
 *   x_type 选项及含义 (会影响 Plus 操作):
 *     - "quat"     : 4维 JPL 四元数, 误差 3 维, 左乘扰动 (同 State_JPLQuatLocal)。
 *     - "quat_yaw" : 4维 JPL 四元数但只对 yaw 线性化 (用于锁 yaw 不可观)。
 *     - "vec3"     : 3维普通向量 (位置/速度/偏置等), 纯加法。
 *     - "vec8"     : 8维 (相机内参 fx,fy,cx,cy,k1,k2,p1,p2/k3,k4)。
 *
 * We have the following minimization problem:
 * @f[
 * \textrm{argmin} ||A * (x - x_{lin}) + b||^2
 * @f]
 *
 *
 * In general we have the following after marginalization:
 * - @f$(A^T*A) = Inf_{prior} @f$ (the prior information)
 * - @f$A^T*b = grad_{prior} @f$ (the prior gradient)
 *
 * For example, consider we have the following system were we wish to remove the xm states.
 * This is the problem of state marginalization.
 * @f[
 * [ Arr Arm ] [ xr ] = [ - gr ]
 * @f]
 * @f[
 * [ Amr Amm ] [ xm ] = [ - gm ]
 * @f]
 *
 * We wish to marginalize the xm states which are correlated with the other states @f$ xr @f$.
 * The Jacobian (and thus information matrix A) is computed at the current best guess @f$ x_{lin} @f$.
 * We can define the following optimal subcost form which only involves the @f$ xr @f$ states as:
 * @f[
 * cost^2 = (xr - xr_{lin})^T*(A^T*A)*(xr - xr_{lin}) + b^T*A*(xr - xr_{lin}) + b^b
 * @f]
 *
 * where we have:
 * @f[
 * A = sqrt(Arr - Arm*Amm^{-1}*Amr)
 * @f]
 * @f[
 * b = A^-1 * (gr - Arm*Amm^{-1}*gm)
 * @f]
 *
 */
class Factor_GenericPrior : public ceres::CostFunction {
public:
  /// State estimates at the time of marginalization to linearize the problem
  Eigen::MatrixXd x_lin;

  /// State type for each variable in x_lin. Can be [quat, quat_yaw, vec3, vec8]
  std::vector<std::string> x_type;

  /// The square-root of the information s.t. sqrtI^T * sqrtI = marginal information
  Eigen::MatrixXd sqrtI;

  /// Constant term inside the cost s.t. sqrtI^T * b = marginal gradient (can be zero)
  Eigen::MatrixXd b;

  /**
   * @brief Default constructor
   */
  Factor_GenericPrior(const Eigen::MatrixXd &x_lin_, const std::vector<std::string> &x_type_, const Eigen::MatrixXd &prior_Info,
                      const Eigen::MatrixXd &prior_grad);

  virtual ~Factor_GenericPrior() {}

  /**
   * @brief Error residual and Jacobian calculation
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_GENERICPRIOR_H