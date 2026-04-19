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

// =============================================================================
// [中文注释] InertialInitializer.h
// -----------------------------------------------------------------------------
// 视觉惯性系统初始化器的总入口。内部聚合两种策略:
//   - StaticInitializer  : 假设系统静止, 用 IMU 方差判断、用均值给出初始姿态与 bias
//   - DynamicInitializer : 动态初始化, 先解相机-IMU 旋转,
//                          再解线性系统 (速度/重力/特征), 最后一次优化 (Ceres)
//
// initialize() 流程:
//   1. 若 wait_for_jerk && 未探测到加速度阶跃 -> 等待
//   2. 先试 init_dynamic->initialize(...)
//   3. 失败且有已知相机外参, 再试 init_static->initialize(...)
//   4. 返回 timestamp / covariance / order / IMU 初值
// =============================================================================

#ifndef OV_INIT_INERTIALINITIALIZER_H
#define OV_INIT_INERTIALINITIALIZER_H

#include "init/InertialInitializerOptions.h"

namespace ov_core {
class FeatureDatabase;
struct ImuData;
} // namespace ov_core
namespace ov_type {
class Type;
class IMU;
} // namespace ov_type

namespace ov_init {

class StaticInitializer;
class DynamicInitializer;

/**
 * @brief Initializer for visual-inertial system.
 *
 * This will try to do both dynamic and state initialization of the state.
 * The user can request to wait for a jump in our IMU readings (i.e. device is picked up) or to initialize as soon as possible.
 * For state initialization, the user needs to specify the calibration beforehand, otherwise dynamic is always used.
 * The logic is as follows:
 * 1. Try to perform dynamic initialization of state elements.
 * 2. If this fails and we have calibration then we can try to do static initialization
 * 3. If the unit is stationary and we are waiting for a jerk, just return, otherwise initialize the state!
 *
 * The dynamic system is based on an implementation and extension of the work [Estimator initialization in vision-aided inertial navigation
 * with unknown camera-IMU calibration](https://ieeexplore.ieee.org/document/6386235) @cite Dong2012IROS which solves the initialization
 * problem by first creating a linear system for recovering the camera to IMU rotation, then for velocity, gravity, and feature positions,
 * and finally a full optimization to allow for covariance recovery.
 * Another paper which might be of interest to the reader is [An Analytical Solution to the IMU Initialization
 * Problem for Visual-Inertial Systems](https://ieeexplore.ieee.org/abstract/document/9462400) which has some detailed
 * experiments on scale recovery and the accelerometer bias.
 */
// [中文] 对外统一的初始化入口。VioManager::try_to_initialize 只持有本类指针,
//        具体采用静态或动态由本类内部自适应选择。
class InertialInitializer {

public:
  /**
   * @brief Default constructor
   * @param params_ Parameters loaded from either ROS or CMDLINE
   * @param db Feature tracker database with all features in it
   */
  explicit InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before
   *
   * [中文] 向 imu_data 追加新的 IMU 并清理 oldest_time 之前的数据。
   *        注意这里的 imu_data 和 Propagator::imu_data 是两个独立的副本。
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1);

  /**
   * @brief Try to get the initialized system
   *
   *
   * @m_class{m-note m-warning}
   *
   * @par Processing Cost
   * This is a serial process that can take on orders of seconds to complete.
   * If you are a real-time application then you will likely want to call this from
   * a async thread which allows for this to process in the background.
   * The features used are cloned from the feature database thus should be thread-safe
   * to continue to append new feature tracks to the database.
   *
   * @param[out] timestamp Timestamp we have initialized the state at
   * @param[out] covariance Calculated covariance of the returned state
   * @param[out] order Order of the covariance matrix
   * @param[out] t_imu Our imu type (need to have correct ids)
   * @param wait_for_jerk If true we will wait for a "jerk"
   * @return True if we have successfully initialized our system
   */
  // [中文] 罕次初始化入口；是一个串行、耗时可能到秒级的运算,
  //   实际调用方 (VioManager::try_to_initialize) 会把它放在独立线程中执行,
  //   所以这里的所有操作已经用 shared_ptr / 数据库拷贝保证线程安全。
  bool initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                  std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk = true);

protected:
  /// Initialization parameters
  InertialInitializerOptions params;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our history of IMU messages (time, angular, linear)
  std::shared_ptr<std::vector<ov_core::ImuData>> imu_data;

  /// Static initialization helper class
  std::shared_ptr<StaticInitializer> init_static;

  /// Dynamic initialization helper class
  std::shared_ptr<DynamicInitializer> init_dynamic;
};

} // namespace ov_init

#endif // OV_INIT_INERTIALINITIALIZER_H
