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

#include "InertialInitializer.h"

#ifndef __ANDROID__
#include "dynamic/DynamicInitializer.h"
#endif
#include "static/StaticInitializer.h"

#include "feat/FeatureHelper.h"
#include "types/Type.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_init;

// [中文] 构造函数:
//   - 保存配置 params / 特征数据库 _db;
//   - 新建共享的 IMU 缓冲 imu_data (静态和动态两个子初始化器共用同一份数据);
//   - 同时创建静态/动态初始化器实例, 后续根据运动状态在 initialize() 里二选一。
//   Android 平台不编 Ceres, 故动态分支用宏开关屏蔽。
InertialInitializer::InertialInitializer(InertialInitializerOptions &params_, std::shared_ptr<ov_core::FeatureDatabase> db)
    : params(params_), _db(db) {

  // Vector of our IMU data
  // [中文] IMU 样本缓冲: shared_ptr 便于子初始化器间共享而无需拷贝。
  imu_data = std::make_shared<std::vector<ov_core::ImuData>>();

  // Create initializers
  // [中文] 静态初始化器总是创建 (即使 Android 也能用, 只靠 IMU 均值/方差, 无需 Ceres)。
  init_static = std::make_shared<StaticInitializer>(params, _db, imu_data);
#ifndef __ANDROID__
  // [中文] 动态初始化器依赖 Ceres, 只在非 Android 平台启用。
  init_dynamic = std::make_shared<DynamicInitializer>(params, _db, imu_data);
#else
  init_dynamic = nullptr;
#endif
}

// [中文] feed_imu: 追加一条 IMU 观测到缓冲, 并可选地按 oldest_time 丢弃过旧数据。
//   oldest_time = -1 表示不丢弃; 否则保留 [oldest_time, +inf) 的所有样本。
//   注意: 本函数不做时间戳排序 (默认调用方已按时间序推入), 若外部乱序请启用下方 std::sort。
void InertialInitializer::feed_imu(const ov_core::ImuData &message, double oldest_time) {

  // Append it to our vector
  // [中文] 追加到缓冲末尾 (默认调用方已按时间序推送, 故不需要每次排序)。
  imu_data->emplace_back(message);

  // Sort our imu data (handles any out of order measurements)
  // [中文] 若外部 IMU 源可能乱序, 解开下面两行即可启用按时间排序 (O(N log N))。
  // std::sort(imu_data->begin(), imu_data->end(), [](const IMUDATA i, const IMUDATA j) {
  //    return i.timestamp < j.timestamp;
  //});

  // Loop through and delete imu messages that are older than our requested time
  // [中文] 清理早于 oldest_time 的 IMU, 防止缓冲无限增长。
  //   这里用 vector::erase 线性擦除, 是因为初始化窗口只有几秒, 数据量有限。
  // std::cout << "INIT: imu_data.size() " << imu_data->size() << std::endl;
  if (oldest_time != -1) {
    auto it0 = imu_data->begin();
    while (it0 != imu_data->end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data->erase(it0);
      } else {
        it0++;
      }
    }
  }
}

// =============================================================================
// [中文] initialize
//  根据最近一段时间的视差 (disparity) 判断系统是静止还是运动, 选择对应的子初始化器:
//    Step 1  扫描 FeatureDatabase 求最新相机帧时间; 据此确定初始化窗口
//    Step 2  清理窗口之外的特征与 IMU
//    Step 3  用 FeatureHelper::compute_disparity 分别求前半 / 后半窗口的平均视差:
//              前小 + 后小 → 一直静止 (is_still)
//              前小 + 后大 → 出现阶跃 (has_jerk)
//              其他         → 一直运动中
//    Step 4  依据 wait_for_jerk 选走哪条分支:
//              (has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk) → 静态初始化
//              否则 (允许动态且非全程静止)                                 → DynamicInitializer
//    Step 5  子初始化器输出 IMU 类型、状态协方差、变量顺序; 调用方写回 VioManager。
// =============================================================================
bool InertialInitializer::initialize(double &timestamp, Eigen::MatrixXd &covariance, std::vector<std::shared_ptr<ov_type::Type>> &order,
                                     std::shared_ptr<ov_type::IMU> t_imu, bool wait_for_jerk) {

  // Get the newest and oldest timestamps we will try to initialize between!
  // [中文] 三重循环遍历 { feat_id -> Feature }, Feature::timestamps = { cam_id -> vector<double> },
  //   取最大值即为 \"当前存在于数据库中最新的相机帧时间\"。
  double newest_cam_time = -1;
  for (auto const &feat : _db->get_internal_data()) {
    for (auto const &camtimepair : feat.second->timestamps) {
      for (auto const &time : camtimepair.second) {
        newest_cam_time = std::max(newest_cam_time, time);
      }
    }
  }
  // [中文] 初始化窗口 = [newest - init_window_time - 0.10, newest]; 多减 0.10s 是为了给
  //   IMU-相机时间偏移 calib_camimu_dt 留一点缓冲, 免得把对齐需要的数据裁掉。
  double oldest_time = newest_cam_time - params.init_window_time - 0.10;
  if (newest_cam_time < 0 || oldest_time < 0) {
    return false; // [中文] 特征库还没有任何帧, 或窗口尚未填满, 直接返回。
  }

  // Remove all measurements that are older then our initialization window
  // Then we will try to use all features that are in the feature database!
  // [中文] 把早于窗口的特征观测从数据库清掉, 下面静/动态两个子初始化器都基于窗口内数据工作。
  _db->cleanup_measurements(oldest_time);
  // [中文] IMU 缓冲也对齐到窗口 (再往前预留 calib_camimu_dt 以应对时间偏移)。
  auto it_imu = imu_data->begin();
  while (it_imu != imu_data->end() && it_imu->timestamp < oldest_time + params.calib_camimu_dt) {
    it_imu = imu_data->erase(it_imu);
  }

  // Compute the disparity of the system at the current timestep
  // If disparity is zero or negative we will always use the static initializer
  // [中文] 视差 (disparity) = 同一特征在相邻帧上的像素位移均值, 用来判断是否在运动:
  //   - 大 → 相机在运动 (或特征很近) → 有视差
  //   - 小 → 相机几乎不动 → 系统静止
  //   init_max_disparity <= 0 时禁用视差判据, 强制走静态初始化。
  bool disparity_detected_moving_1to0 = false; // [中文] 窗口前半段是否有运动
  bool disparity_detected_moving_2to1 = false; // [中文] 窗口后半段是否有运动
  if (params.init_max_disparity > 0) {

    // Get the disparity statistics from this image to the previous
    // Only compute the disparity for the oldest half of the initialization period
    // [中文] 以窗口正中心把窗口劈成早 / 晚两段分别求平均视差,
    //   用于区分 \"一直静止\" vs \"静→动 (jerk)\" vs \"一直运动\" 三种场景。
    double newest_time_allowed = newest_cam_time - 0.5 * params.init_window_time;
    int num_features0 = 0;
    int num_features1 = 0;
    double avg_disp0, avg_disp1;
    double var_disp0, var_disp1;
    // [中文] compute_disparity(db, avg, var, n, t_end [, t_start]) 返回 (t_start..t_end] 区间的视差。
    FeatureHelper::compute_disparity(_db, avg_disp0, var_disp0, num_features0, newest_time_allowed);
    FeatureHelper::compute_disparity(_db, avg_disp1, var_disp1, num_features1, newest_cam_time, newest_time_allowed);

    // Return if we can't compute the disparity
    // [中文] 样本不足 15 个特征就放弃 (avg_disp 统计量噪声太大, 否则容易误判)。
    int feat_thresh = 15;
    if (num_features0 < feat_thresh || num_features1 < feat_thresh) {
      PRINT_WARNING(YELLOW "[init]: not enough feats to compute disp: %d,%d < %d\n" RESET, num_features0, num_features1, feat_thresh);
      return false;
    }

    // Check if it passed our check!
    // [中文] 阈值比较: 均值视差 > init_max_disparity (单位像素, 典型 1~10) → 判为 \"运动\"。
    PRINT_INFO(YELLOW "[init]: disparity is %.3f,%.3f (%.2f thresh)\n" RESET, avg_disp0, avg_disp1, params.init_max_disparity);
    disparity_detected_moving_1to0 = (avg_disp0 > params.init_max_disparity);
    disparity_detected_moving_2to1 = (avg_disp1 > params.init_max_disparity);
  }

  // Use our static initializer!
  // CASE1: if our disparity says we were static in last window and have moved in the newest, we have a jerk
  // CASE2: if both disparities are below the threshold, then the platform has been stationary during both periods
  // [中文] 决策树 (综合视差 + wait_for_jerk 开关):
  //   CASE1 (jerk)  : 前半静止、后半运动 → 系统被 \"举起\" 或 \"踢一脚\" → 静态初始化最可靠
  //                   (此时前半段的数据天然已静止, 可以直接估 bias/重力)
  //   CASE2 (still) : 前后都静止 → 若允许不等 jerk (!wait_for_jerk, 需要 ZUPT 支持) 也可直接初始化
  //   其它 (moving) : 一直在动 → 只能走 DynamicInitializer (线性系统 + Ceres BA)
  bool has_jerk = (!disparity_detected_moving_1to0 && disparity_detected_moving_2to1);
  bool is_still = (!disparity_detected_moving_1to0 && !disparity_detected_moving_2to1);
  // [中文] 进入静态分支还要满足 init_imu_thresh > 0; 该阈值用于 StaticInitializer 内部\n
  //   再次校验加速度方差, 防止 \"视差算出是静止但 IMU 实际在震\" 的误触发。
  if (((has_jerk && wait_for_jerk) || (is_still && !wait_for_jerk)) && params.init_imu_thresh > 0.0) {
    PRINT_DEBUG(GREEN "[init]: USING STATIC INITIALIZER METHOD!\n" RESET);
    return init_static->initialize(timestamp, covariance, order, t_imu, wait_for_jerk);
  } else if (params.init_dyn_use && !is_still) {
    // [中文] 动态分支: 配置里允许动态初始化且不是全程静止。
#ifndef __ANDROID__
    PRINT_DEBUG(GREEN "[init]: USING DYNAMIC INITIALIZER METHOD!\n" RESET);
    // [中文] DynamicInitializer 额外会输出 clone 姿态与 SLAM 特征 (可被 VioManager 采纳, 省一轮扩维)。
    std::map<double, std::shared_ptr<ov_type::PoseJPL>> _clones_IMU;
    std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>> _features_SLAM;
    if (init_dynamic) {
      return init_dynamic->initialize(timestamp, covariance, order, t_imu, _clones_IMU, _features_SLAM);
    }
#else
    PRINT_ERROR(RED "[init]: DYNAMIC INITIALIZER not available on Android (Ceres Solver not included)\n" RESET);
#endif
  } else {
    // [中文] 失败: 既不满足静态条件也没有启用动态, 打印具体失败原因方便调参。
    std::string msg = (has_jerk) ? "" : "no accel jerk detected";
    msg += (has_jerk || is_still) ? "" : ", ";
    msg += (is_still) ? "" : "platform moving too much";
    PRINT_INFO(YELLOW "[init]: failed static init: %s\n" RESET, msg.c_str());
  }
  return false;
}
