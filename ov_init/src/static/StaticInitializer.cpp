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

#include "StaticInitializer.h"

#include "utils/helper.h"

#include "feat/FeatureHelper.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

// =============================================================================
// [中文] StaticInitializer::initialize — 静态初始化核心流程
// -----------------------------------------------------------------------------
// 基本思路:
//   系统刚启动时如果 IMU 处于静止状态, 加速度计读数的方向 ≈ 反重力方向 (a ≈ -R·g),
//   角速度读数 ≈ 陀螺 bias (w ≈ b_g)。据此可以直接求出:
//     - 初始姿态 R_GtoI   (使 z 轴对准 -g)
//     - 陀螺 bias b_g     (= w_avg)
//     - 加速度 bias b_a   (= a_avg - R·g)
//
// 实现步骤:
//   1) 把 IMU 缓冲切成 \"晚\" window_1to0 (最近半窗) 和 \"早\" window_2to1 (前半窗) 两段
//   2) 对每段分别算加速度方差:
//        - 晚段方差 a_var_1to0 > 阈值 → \"动了\" (wait_for_jerk 模式下所需的阶跃信号)
//        - 早段方差 a_var_2to1 < 阈值 → \"早段是静止的\" (我们要用早段估姿态 / bias)
//   3) 用早段 IMU 均值求姿态、重力、bias
//   4) 填 16 维 imu_state 和 15×15 初始协方差 (q/p/v 各对角块)
//
// wait_for_jerk 的作用:
//   - true  : 必须早段静止 + 晚段有运动 (即一次 jerk) 才启动; 避免系统还没来得及估 bias 就动了
//   - false : 早段或晚段静止就足够; 仅在已有 ZUPT 约束可防 yaw/position 漂走时才推荐
// =============================================================================
bool StaticInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<Type>> &order,
                                   std::shared_ptr<IMU> t_imu, bool wait_for_jerk) {

  // Return if we don't have any measurements
  // [中文] 连两条 IMU 都没有, 无法算任何统计量, 直接失败。
  if (imu_data->size() < 2) {
    return false;
  }

  // Newest and oldest imu timestamp
  // [中文] 取 IMU 缓冲的首尾时间戳, 判断是否凑够 init_window_time 时长。
  double newesttime = imu_data->at(imu_data->size() - 1).timestamp;
  double oldesttime = imu_data->at(0).timestamp;

  // Return if we don't have enough for two windows
  // [中文] 时间跨度不足 init_window_time (默认 1~2 秒), 无法切成前后两段, 失败。
  if (newesttime - oldesttime < params.init_window_time) {
    PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // First lets collect a window of IMU readings from the newest measurement to the oldest
  // [中文] 把 IMU 切两段: window_1to0 = 最近半窗 (晚), window_2to1 = 前半窗 (早)。
  //        时间区间使用半开半闭 (a, b] 风格, 防止边界样本被重复计入两段。
  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : *imu_data) {
    // [中文] 晚段: (newest - 0.5*T,  newest]
    if (data.timestamp > newesttime - 0.5 * params.init_window_time && data.timestamp <= newesttime - 0.0 * params.init_window_time) {
      window_1to0.push_back(data);
    }
    // [中文] 早段: (newest - T, newest - 0.5*T]
    if (data.timestamp > newesttime - 1.0 * params.init_window_time && data.timestamp <= newesttime - 0.5 * params.init_window_time) {
      window_2to1.push_back(data);
    }
  }

  // Return if both of these failed
  // [中文] 任一段样本 < 2 条, 无法估方差, 失败。
  if (window_1to0.size() < 2 || window_2to1.size() < 2) {
    PRINT_INFO(YELLOW "[init-s]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // Calculate the sample variance for the newest window from 1 to 0
  // [中文] 对晚段 (1to0): 求加速度均值, 再求样本标准差 a_var_1to0。
  //   数学上: σ = sqrt( (1/(N-1)) · Σ‖a_i - ā‖² ), 这里把三轴方差和开根作为 \"激励强度\"。
  Eigen::Vector3d a_avg_1to0 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_1to0) {
    a_avg_1to0 += data.am;
  }
  a_avg_1to0 /= (int)window_1to0.size();
  double a_var_1to0 = 0;
  for (const ImuData &data : window_1to0) {
    a_var_1to0 += (data.am - a_avg_1to0).dot(data.am - a_avg_1to0);
  }
  a_var_1to0 = std::sqrt(a_var_1to0 / ((int)window_1to0.size() - 1));

  // Calculate the sample variance for the second newest window from 2 to 1
  // [中文] 对早段 (2to1): 同时计算加速度和角速度均值 (稍后早段均值要用于估重力/bias)。
  Eigen::Vector3d a_avg_2to1 = Eigen::Vector3d::Zero();
  Eigen::Vector3d w_avg_2to1 = Eigen::Vector3d::Zero();
  for (const ImuData &data : window_2to1) {
    a_avg_2to1 += data.am;
    w_avg_2to1 += data.wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  PRINT_DEBUG(YELLOW "[init-s]: IMU excitation stats: %.3f,%.3f\n" RESET, a_var_2to1, a_var_1to0);

  // If it is below the threshold and we want to wait till we detect a jerk
  // [中文] wait_for_jerk=true 时, 晚段方差必须 > 阈值才视为 \"检测到 jerk\",
  //        否则说明系统一直静止, 需要继续等。
  if (a_var_1to0 < params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: no IMU excitation, below threshold %.3f < %.3f\n" RESET, a_var_1to0, params.init_imu_thresh);
    return false;
  }

  // We should also check that the old state was below the threshold!
  // This is the case when we have started up moving, and thus we need to wait for a period of stationary motion
  // [中文] wait_for_jerk=true 时, 早段方差必须 < 阈值 (= 早段确实是静止的), 否则
  //        没办法把早段 IMU 均值当作 \"纯重力\" 来估姿态, 需要等到出现一段平稳期。
  if (a_var_2to1 > params.init_imu_thresh && wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: to much IMU excitation, above threshold %.3f > %.3f\n" RESET, a_var_2to1, params.init_imu_thresh);
    return false;
  }

  // If it is above the threshold and we are not waiting for a jerk
  // Then we are not stationary (i.e. moving) so we should wait till we are
  // [中文] wait_for_jerk=false 时, 要求前后段都静止 (即 var 都小), 否则初值会把运动带入 bias 估计。
  if ((a_var_1to0 > params.init_imu_thresh || a_var_2to1 > params.init_imu_thresh) && !wait_for_jerk) {
    PRINT_INFO(YELLOW "[init-s]: to much IMU excitation, above threshold %.3f,%.3f > %.3f\n" RESET, a_var_2to1, a_var_1to0,
               params.init_imu_thresh);
    return false;
  }

  // Get rotation with z axis aligned with -g (z_in_G=0,0,1)
  // [中文] 构造初始姿态 R_GtoI:
  //        在 IMU 系下, 静止时 ā ≈ -R_GtoI · g (重力作用于加速度计, 方向反过来)。
  //        做法: 把 ā 方向作为 IMU 系的 z 轴, 用 Gram-Schmidt 补出 x, y 正交基, 构成 R。
  //        这样构出的 R_GtoI 使 IMU 系的 z 轴对准 -g, 即 roll/pitch 正确, 但 yaw 不确定 (4-DOF 不可观)。
  Eigen::Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();
  Eigen::Matrix3d Ro;
  InitializerHelper::gram_schmidt(z_axis, Ro);
  Eigen::Vector4d q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  // [中文] 静止时角速度均值 ≈ 陀螺 bias (因为真实旋转 ω ≈ 0);
  //        加速度 bias = a_avg - 投影到 IMU 系的重力。
  Eigen::Vector3d gravity_inG;
  gravity_inG << 0.0, 0.0, params.gravity_mag;
  Eigen::Vector3d bg = w_avg_2to1;
  Eigen::Vector3d ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * gravity_inG;

  // Set our state variables
  // [中文] 写 16 维 IMU 状态向量: [q(4), p(3), v(3), b_g(3), b_a(3)]; 这里只填 q/b_g/b_a, p=v=0。
  //        set_fej(...) 同时把 \"第一估计线性化点\" 固定到当前值 — FEJ 的基础, 见 docs-cn/math_foundations.md §6。
  timestamp = window_2to1.at(window_2to1.size() - 1).timestamp;
  Eigen::VectorXd imu_state = Eigen::VectorXd::Zero(16);
  imu_state.block(0, 0, 4, 1) = q_GtoI;
  imu_state.block(10, 0, 3, 1) = bg;
  imu_state.block(13, 0, 3, 1) = ba;
  assert(t_imu != nullptr);
  t_imu->set_value(imu_state);
  t_imu->set_fej(imu_state);

  // Create base covariance and its covariance ordering
  // [中文] 初始协方差: 只含一个 15 维 IMU 误差块 (δθ, δp, δv, δb_g, δb_a 各 3 维),
  //        对角分块如下 (单位均为 SI):
  //            δθ ~ 0.02 rad      roll/pitch 从加速度直接求出, 较准; yaw 不可观本应更大但放小以靠 FEJ 锁定
  //            δp ~ 0.05 m        位置任选原点, 但留一点不确定性
  //            δv ~ 0.01 m/s      静止初始化速度本应为 0, 给一点余量便于激活
  //            δb_g / δb_a        保持默认 0.02 (没有更具体的先验)
  order.clear();
  order.push_back(t_imu);
  covariance = std::pow(0.02, 2) * Eigen::MatrixXd::Identity(t_imu->size(), t_imu->size());
  covariance.block(0, 0, 3, 3) = std::pow(0.02, 2) * Eigen::Matrix3d::Identity(); // q
  covariance.block(3, 3, 3, 3) = std::pow(0.05, 2) * Eigen::Matrix3d::Identity(); // p
  covariance.block(6, 6, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity(); // v (static)

  // A VIO system has 4dof unobservable directions which can be arbitrarily picked.
  // This means that on startup, we can fix the yaw and position to be 100 percent known.
  // TODO: why can't we set these to zero and get a good NEES realworld result?
  // Thus, after determining the global to current IMU orientation after initialization, we can propagate the global error
  // into the new IMU pose. In this case the position is directly equivalent, but the orientation needs to be propagated.
  // We propagate the global orientation into the current local IMU frame
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  // [中文] VIO 系统有 4-DOF 不可观方向 (3D 全局位置 + yaw 绕重力轴), 理论上可以任意选取,
  //        在启动时完全可以把 yaw/position 协方差设成 \"近乎 0\", 再通过雅可比变换传播到 IMU 系下。
  //        代码里给的注释就是这个思路; 但目前默认保留较大协方差, 留出数值余量与 FEJ 兼容。
  //        (TODO 是原作者在问: 为何实际跑 NEES 时这个修改不一定好, 和 FEJ 约束的实现细节相关。)
  // Eigen::Matrix3d R_GtoI = quat_2_Rot(q_GtoI);
  // covariance(2, 2) = std::pow(1e-4, 2);
  // covariance.block(0, 0, 3, 3) = R_GtoI * covariance.block(0, 0, 3, 3) * R_GtoI.transpose();
  // covariance.block(3, 3, 3, 3) = std::pow(1e-3, 2) * Eigen::Matrix3d::Identity();

  // Return :D
  return true;
}
