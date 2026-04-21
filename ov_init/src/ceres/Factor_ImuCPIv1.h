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

#ifndef OV_INIT_CERES_IMUCPIV1_H
#define OV_INIT_CERES_IMUCPIV1_H

#include <ceres/ceres.h>

namespace ov_init {

/**
 * @brief Factor for IMU continuous preintegration version 1
 *
 * [中文] Factor_ImuCPIv1 — 连续预积分 Model 1 的 Ceres 因子 (Eckenhoff IJRR)
 *
 *   15 维残差 r:
 *     r_q  (0..2)  = 2 * vec( q_breve^{-1} ⊗ q_GtoI_{k+1} ⊗ q_GtoI_k^{-1} )        // 姿态残差
 *     r_p  (3..5)  = R_GtoI_k * (p_Ik+1 - p_Ik - v_Ik * DT + 0.5 * g * DT^2) - alpha
 *     r_v  (6..8)  = R_GtoI_k * (v_Ik+1 - v_Ik + g * DT) - beta
 *     r_bg (9..11) = bg_{k+1} - bg_k
 *     r_ba (12..14)= ba_{k+1} - ba_k
 *
 *   bias 校正 (若运行时 bg/ba 与 setLinearizationPoints 时不同):
 *     alpha' = alpha + J_a * dbg + H_a * dba
 *     beta'  = beta  + J_b * dbg + H_b * dba
 *     q_corr ≈ exp(J_q * dbg) ⊗ q_breve    // 一阶修正
 *
 *   参数块顺序 (parameters):
 *     [0] q_GtoIk, [1] bg_k,   [2] v_Ik,  [3] ba_k,   [4] p_Ik
 *     [5] q_GtoIk+1,[6] bg_k+1,[7] v_Ik+1,[8] ba_k+1, [9] p_Ik+1
 *     (注意顺序是 q-bg-v-ba-p, 不是 q-p-v-bg-ba, 对应旧 MSCKF 传统)
 *
 *   sqrtI_save: 预积分 15x15 协方差 P_meas 的 Cholesky 平方根 (sqrtI^T sqrtI = P^{-1}),
 *              乘进残差 r 让其服从单位方差, Ceres 内部再做 cost = 0.5 ||r||^2。
 */
class Factor_ImuCPIv1 : public ceres::CostFunction {
public:
  // Preintegrated measurements and time interval
  Eigen::Vector3d alpha;
  Eigen::Vector3d beta;
  Eigen::Vector4d q_breve;
  double dt;

  // Preintegration linearization points
  Eigen::Vector3d b_w_lin_save;
  Eigen::Vector3d b_a_lin_save;

  // Prinetegrated bias jacobians
  Eigen::Matrix3d J_q; // J_q - orientation wrt bias w
  Eigen::Matrix3d J_a; // J_a - position wrt bias w
  Eigen::Matrix3d J_b; // J_b - velocity wrt bias w
  Eigen::Matrix3d H_a; // H_a - position wrt bias a
  Eigen::Matrix3d H_b; // H_b - velocity wrt bias a

  // Sqrt of the preintegration information
  Eigen::Matrix<double, 15, 15> sqrtI_save;

  // Gravity
  Eigen::Vector3d grav_save;

  /**
   * @brief Default constructor
   */
  Factor_ImuCPIv1(double deltatime, Eigen::Vector3d &grav, Eigen::Vector3d &alpha, Eigen::Vector3d &beta, Eigen::Vector4d &q_KtoK1,
                  Eigen::Vector3d &ba_lin, Eigen::Vector3d &bg_lin, Eigen::Matrix3d &J_q, Eigen::Matrix3d &J_beta, Eigen::Matrix3d &J_alpha,
                  Eigen::Matrix3d &H_beta, Eigen::Matrix3d &H_alpha, Eigen::Matrix<double, 15, 15> &covariance);

  virtual ~Factor_ImuCPIv1() {}

  /**
   * @brief Error residual and Jacobian calculation
   *
   * This computes the error between the integrated preintegrated measurement
   * and the current state estimate. This also takes into account the
   * bias linearization point changes.
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_IMUCPIV1_H