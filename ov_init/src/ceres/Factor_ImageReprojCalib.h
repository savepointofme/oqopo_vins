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

#ifndef OV_INIT_CERES_IMAGEREPROJCALIB_H
#define OV_INIT_CERES_IMAGEREPROJCALIB_H

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <deque>
#include <iostream>
#include <map>

#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "utils/quat_ops.h"

namespace ov_init {

/**
 * @brief Factor of feature bearing observation (raw) with calibration
 *
 * [中文] Factor_ImageReprojCalib — 带在线标定的像素重投影因子 (Ceres CostFunction)
 *
 *   残差: r = sqrtQ * (uv_meas - project_distort(q_ItoC, p_IinC, q_GtoIi, p_IiinG, p_FinG, intrinsics))
 *
 *   参数块顺序 (evaluate 时 parameters 的顺序):
 *     [0] q_GtoIi    (JPL 四元数, 4-dim, 用 State_JPLQuatLocal 做 local param)
 *     [1] p_IiinG    (3-dim)
 *     [2] p_FinG     (3-dim, 特征在全局系的位置)
 *     [3] q_ItoC     (JPL 四元数, 4-dim, 相机-IMU 外参旋转)
 *     [4] p_IinC     (3-dim, 相机-IMU 外参平移, 注意方向是 IMU 在相机下)
 *     [5] intrinsics (8-dim: fx,fy,cx,cy 加 4 个畸变系数)
 *
 *   雅可比链:
 *     d(r)/d(pose)   : 通过投影 dpi/dpc 乘上 d(p_FinC)/d(pose)
 *     d(r)/d(p_FinG) : 投影 dpi/dpc 乘上 R_ItoC * R_GtoIi
 *     d(r)/d(calib)  : 外参部分类似, 内参直接对 fx/fy/cx/cy/distortion 求偏导
 *
 *   gate: 给每个残差一个 "开关" (0/1), 用于在 Ceres 运行中跳过坏观测而不重新构造 Problem。
 *
 *   is_fisheye: 决定内部用 CamRadtan 还是 CamEqui 做去/加畸变, 二者的雅可比公式不同。
 *
 *   sqrtQ: 测量噪声协方差的 Cholesky 因子 (1/pix_sigma * I 2x2), 用来把像素观测归一化成单位方差。
 */
class Factor_ImageReprojCalib : public ceres::CostFunction {
public:
  // Measurement observation of the feature (raw pixel coordinates)
  Eigen::Vector2d uv_meas;

  // Measurement noise
  double pix_sigma = 1.0;
  Eigen::Matrix<double, 2, 2> sqrtQ;

  // If distortion model is fisheye or radtan
  bool is_fisheye = false;

  // If value of 1 then this residual adds to the problem, otherwise if zero it is "gated"
  double gate = 1.0;

  /**
   * @brief Default constructor
   * @param uv_meas_ Raw pixel uv measurement of a environmental feature
   * @param pix_sigma_ Raw pixel measurement uncertainty (typically 1)
   * @param is_fisheye_ If this raw pixel camera uses fisheye distortion
   */
  Factor_ImageReprojCalib(const Eigen::Vector2d &uv_meas_, double pix_sigma_, bool is_fisheye_);

  virtual ~Factor_ImageReprojCalib() {}

  /**
   * @brief Error residual and Jacobian calculation
   *
   * This computes the Jacobians and residual of the feature projection model.
   * This is a function of the observing pose, feature in global, and calibration parameters.
   * The normalized pixel coordinates are found and then distorted using the camera distortion model.
   * See the @ref update-feat page for more details.
   */
  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override;
};

} // namespace ov_init

#endif // OV_INIT_CERES_IMAGEREPROJCALIB_H