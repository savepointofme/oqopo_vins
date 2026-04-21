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

#ifndef OV_INIT_DYNAMICINITIALIZER_H
#define OV_INIT_DYNAMICINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
class PoseJPL;
class Landmark;
class Vec;
} // namespace ov_type

namespace ov_init {

/**
 * @brief Initializer for a dynamic visual-inertial system.
 *
 * This implementation that will try to recover the initial conditions of the system.
 * Additionally, we will try to recover the covariance of the system.
 * To initialize with arbitrary motion:
 * 1. Preintegrate our system to get the relative rotation change (biases assumed known)
 * 2. Construct linear system with features to recover velocity (solve with |g| constraint)
 * 3. Perform a large MLE with all calibration and recover the covariance.
 *
 * [中文] DynamicInitializer — 带运动的动态初始化 (系统并非静止)
 *
 *   论文: Dong-Si & Mourikis 2012 (IROS) + Eckenhoff et al. 预积分扩展。
 *   适用场景: 无人机手投 / 背包启动 / 车辆起步等 "启动时就在运动" 的情况。
 *
 *   6 个 stage (见 .cpp 文件的 "==== " 分割线):
 *     Stage 1 — 窗口准备: 根据 init_window_time 切一段数据, 拷贝 features 防止异步改写。
 *     Stage 2 — CPI 预积分: 对每个相机时刻做两段预积分
 *                  (a) I0 → Ii (用于线性系统中累积位移/速度)
 *                  (b) Ii → Ii+1 (用于后续 MLE IMU 因子)
 *     Stage 3 — 构造线性系统: Eq.(14), 状态 [features, velocity, gravity],
 *                  用特征视差约束求解, 未知数为像素反推的位置残差。
 *     Stage 4 — |g| 约束求解: 对重力 3D 向量加 ‖g‖ = 9.81 硬约束,
 *                  化成 6 次多项式特征值问题, 取最小实特征值 → 求出 gravity & velocity。
 *     Stage 5 — 坐标对齐: 通过 Gram-Schmidt 构造 R_GtoI0 使 G 系 z 轴对齐重力,
 *                  所有特征、速度、位置均旋转到 G 系。
 *     Stage 6 — Ceres MLE: 构造 Problem, 加入 IMU CPI factor + 每个特征 ImageReprojCalib factor
 *                  + 第一位姿 GenericPrior (锁定 4 自由度不可观), 优化 → 恢复协方差。
 *
 *   输出: 最新 IMU 状态 + 所有位姿 clones + SLAM 特征 + 协方差矩阵 (按 order 排列)。
 *
 *   失败分支: 特征数不够 / CPI 积分缺 IMU / 多项式无实根 / 重力不收敛 / Ceres 不 converge.
 *
 * Method is based on this work (see this [tech report](https://pgeneva.com/downloads/reports/tr_init.pdf) for a high level walk through):
 *
 * > Dong-Si, Tue-Cuong, and Anastasios I. Mourikis.
 * > "Estimator initialization in vision-aided inertial navigation with unknown camera-IMU calibration."
 * > 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE, 2012.
 *
 * - https://ieeexplore.ieee.org/abstract/document/6386235
 * - https://tdongsi.github.io/download/pubs/2011_VIO_Init_TR.pdf
 * - https://pgeneva.com/downloads/reports/tr_init.pdf
 *
 */
class DynamicInitializer {
public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   * @param imu_data_ Shared pointer to our IMU vector of historical information
   */
  explicit DynamicInitializer(const InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db,
                              std::shared_ptr<std::vector<ov_core::ImuData>> imu_data_)
      : params(params_), _db(db), imu_data(imu_data_) {}

  /**
   * @brief Try to get the initialized system
   *
   * @param[out] timestamp Timestamp we have initialized the state at (last imu state)
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param _imu Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
   * @param _clones_IMU Map between imaging times and clone poses (q_GtoIi, p_IiinG)
   * @param _features_SLAM Our current set of SLAM features (3d positions)
   * @return True if we have successfully initialized our system
   */
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> &_imu, std::map<double, std::shared_ptr<ov_type::PoseJPL>> &_clones_IMU,
                  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> &_features_SLAM);

private:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;
};

} // namespace ov_init

#endif // OV_INIT_DYNAMICINITIALIZER_H
