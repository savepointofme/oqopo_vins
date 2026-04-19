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
// [中文注释] VioManager.h
// -----------------------------------------------------------------------------
// OpenVINS 系统的**总调度**类。
//
// 背景:
//   OpenVINS 采用 MSCKF (Multi-State Constraint Kalman Filter) 架构,
//   将一段时间窗口内的 IMU 位姿克隆维护在状态中,
//   通过左零空间投影消掉特征位置维度, 既保留多帧约束又避免了状态臆肿。
//
// VioManager 职责:
//   - 持有系统的所有主要模块:
//       state (状态) / propagator (IMU 传播) /
//       trackFEATS / trackARUCO (视觉前端) /
//       initializer (初始化) /
//       updaterMSCKF / updaterSLAM / updaterZUPT (三类更新器)
//   - 对外提供 3 个主要入口:
//       feed_measurement_imu       -> IMU 数据入口
//       feed_measurement_camera    -> 相机帧入口
//       feed_measurement_simulation-> 仿真模式下的合成特征入口
//   - 内部流程: track_image_and_update -> do_feature_propagate_update
//     (见 docs-cn/vio_manager.md 中的流程图)
//
// 主要成员设计触观:
//   所有组件用 std::shared_ptr 持有, 确保生命周期与 VioManager 一致,
//   线程安全性主要由 ROS 层的预队列和内部 std::atomic<bool> 实现。
// =============================================================================

#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H

#include <Eigen/StdVector>
#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include "VioManagerOptions.h"

namespace ov_core {
struct ImuData;
struct CameraData;
class TrackBase;
class FeatureInitializer;
} // namespace ov_core
namespace ov_init {
class InertialInitializer;
} // namespace ov_init

namespace ov_msckf {

class State;
class StateHelper;
class UpdaterMSCKF;
class UpdaterSLAM;
class UpdaterZeroVelocity;
class Propagator;

/**
 * @brief Core class that manages the entire system
 *
 * This class contains the state and other algorithms needed for the MSCKF to work.
 * We feed in measurements into this class and send them to their respective algorithms.
 * If we have measurements to propagate or update with, this class will call on our state to do that.
 *
 * [中文] VIO 系统的**总调度类**。
 *  - 外部通过 feed_measurement_* 将 IMU / 图像 / 仿真特征喂入这个类;
 *  - 类内部决定当前应该调用 Propagator / TrackBase / Initializer / Updater* 中的哪个;
 *  - State 是系统唯一的**共享可变状态**, 其他模块都在这个类里取用/修改。
 */
class VioManager {

public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ Parameters loaded from either ROS or CMDLINE
   *
   * [中文] 构造函数。根据 params_ 依次创建与启用:
   *   - state + propagator
   *   - trackFEATS (TrackKLT / TrackDescriptor / TrackSIM)
   *   - trackARUCO (可选, 开启后与 trackFEATS 并存)
   *   - initializer (含静态 + 动态两种)
   *   - updaterMSCKF / updaterSLAM / updaterZUPT
   *   - 没有在这里初始化 State, 真正的初值设在 try_to_initialize 成功时写入。
   */
  VioManager(VioManagerOptions &params_);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   *
   * [中文] IMU 入口。内部会将测量同时发给
   *   - propagator (用于之后的状态传播)
   *   - initializer (如果尚未初始化)
   *   - updaterZUPT (开启零速时)
   * 并传入一个 oldest_time, 由各个模块自行裁剪过旧的 IMU。
   */
  void feed_measurement_imu(const ov_core::ImuData &message);

  /**
   * @brief Feed function for camera measurements
   * @param message Contains our timestamp, images, and camera ids
   *
   * [中文] 真实相机帧入口, 直接转发给 track_image_and_update。
   */
  void feed_measurement_camera(const ov_core::CameraData &message) { track_image_and_update(message); }

  /**
   * @brief Feed function for a synchronized simulated cameras
   * @param timestamp Time that this image was collected
   * @param camids Camera ids that we have simulated measurements for
   * @param feats Raw uv simulated measurements
   *
   * [中文] 仿真模式下的合成特征入口。feats 已经是 (id, uv) 话题,
   *        会绕过 TrackKLT 直接写入 TrackSIM 的 FeatureDatabase。
   */
  void feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                   const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /**
   * @brief Given a state, this will initialize our IMU state.
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   *
   * [中文] 绕过 InertialInitializer, 直接用真值初始化 IMU 状态;
   *        主要用于仿真或调试, 生产环境用不到。
   */
  void initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate);

  /// If we are initialized or not
  /// [中文] 判断系统是否已初始化: 必须既完成初始化又至少做过一次更新。
  bool initialized() { return is_initialized_vio && timelastupdate != -1; }

  /// Timestamp that the system was initialized at
  double initialized_time() { return startup_time; }

  /// Accessor for current system parameters
  VioManagerOptions get_params() { return params; }

  /// Accessor to get the current state
  std::shared_ptr<State> get_state() { return state; }

  /// Accessor to get the current propagator
  std::shared_ptr<Propagator> get_propagator() { return propagator; }

  /// Get a nice visualization image of what tracks we have
  cv::Mat get_historical_viz_image();

  /// Returns 3d SLAM features in the global frame
  std::vector<Eigen::Vector3d> get_features_SLAM();

  /// Returns 3d ARUCO features in the global frame
  std::vector<Eigen::Vector3d> get_features_ARUCO();

  /// Returns 3d features used in the last update in global frame
  std::vector<Eigen::Vector3d> get_good_features_MSCKF() { return good_features_MSCKF; }

  /// Return the image used when projecting the active tracks
  void get_active_image(double &timestamp, cv::Mat &image) {
    timestamp = active_tracks_time;
    image = active_image;
  }

  /// Returns active tracked features in the current frame
  void get_active_tracks(double &timestamp, std::unordered_map<size_t, Eigen::Vector3d> &feat_posinG,
                         std::unordered_map<size_t, Eigen::Vector3d> &feat_tracks_uvd) {
    timestamp = active_tracks_time;
    feat_posinG = active_tracks_posinG;
    feat_tracks_uvd = active_tracks_uvd;
  }

protected:
  /**
   * @brief Given a new set of camera images, this will track them.
   *
   * If we are having stereo tracking, we should call stereo tracking functions.
   * Otherwise we will try to track on each of the images passed.
   *
   * @param message Contains our timestamp, images, and camera ids
   *
   * [中文] 相机入口的**总分流器**:
   *   1. 可选对图像 cv::pyrDown 下采样
   *   2. 调用 trackFEATS->feed_new_camera / trackARUCO->feed_new_camera 做前端跟踪
   *   3. 若已初始化:
   *      - 尝试 ZUPT 零速更新 (updaterZUPT->try_update), 命中则跳过传播直接返回
   *      - 否则进入 do_feature_propagate_update
   *   4. 若尚未初始化: 调用 try_to_initialize, 成功后才真正开始滤波
   */
  void track_image_and_update(const ov_core::CameraData &message);

  /**
   * @brief This will do the propagation and feature updates to the state
   * @param message Contains our timestamp, images, and camera ids
   *
   * [中文] 单帧的完整滤波流程 (见 docs-cn/diagrams/03_vio_manager_flow.png):
   *   Step 1. propagator->propagate_and_clone: IMU 预测 + 创建新克隆
   *   Step 2. 从 FeatureDatabase 拉取并分类特征:
   *            feats_lost / feats_marg / feats_maxtracks
   *            feats_slam_UPDATE (已存在的 SLAM)
   *            feats_slam_DELAYED (新 SLAM)
   *   Step 3. StateHelper::marginalize_slam + updaterMSCKF->update +
   *            updaterSLAM->update + updaterSLAM->delayed_init
   *   Step 4. retriangulate_active_tracks + marginalize_old_clone (维持滑窗大小)
   */
  void do_feature_propagate_update(const ov_core::CameraData &message);

  /**
   * @brief This function will try to initialize the state.
   *
   * This should call on our initializer and try to init the state.
   * In the future we should call the structure-from-motion code from here.
   * This function could also be repurposed to re-initialize the system after failure.
   *
   * @param message Contains our timestamp, images, and camera ids
   * @return True if we have successfully initialized
   *
   * [中文] 尝试初始化. 内部可能将 initializer->initialize 异步地放到另一个线程,
   *        通过 thread_init_running / thread_init_success 两个原子变量交换状态。
   *        成功后会根据 initializer 返回的 (timestamp, cov, order, t_imu) 构建初始
   *        State, 并把初始化期间积累的相机时间戳重放交给 track_image_and_update。
   */
  bool try_to_initialize(const ov_core::CameraData &message);

  /**
   * @brief This function will will re-triangulate all features in the current frame
   *
   * For all features that are currently being tracked by the system, this will re-triangulate them.
   * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
   * This will try to triangulate *all* points, not just ones that have been used in the update.
   *
   * @param message Contains our timestamp, images, and camera ids
   *
   * [中文] 对当前帧内**所有在跟踪的特征**重新三角化, 即使它们没被用于更新。
   *        输出保存在 active_tracks_posinG / active_tracks_uvd, 供回环检测,
   *        交互点云可视化等下游模块使用。内部维护 (A, b, count) 让三角化
   *        返回的 p_FinG 在帧间滤波 (累积线性系统)。
   */
  void retriangulate_active_tracks(const ov_core::CameraData &message);

  /// Manager parameters
  /// [中文] 启动时整体加载的配置 (时间窗、最大克隆数、各种阈值等)
  VioManagerOptions params;

  /// Our master state object :D
  /// [中文] 状态对象, 包含 IMU / 克隆位姿 / SLAM 特征 / 标定参数 + 形式完全协方差 P
  std::shared_ptr<State> state;

  /// Propagator of our state
  /// [中文] IMU 预测器
  std::shared_ptr<Propagator> propagator;

  /// Our sparse feature tracker (klt or descriptor)
  /// [中文] 稀疏前端 (params.use_klt ? TrackKLT : TrackDescriptor)
  std::shared_ptr<ov_core::TrackBase> trackFEATS;

  /// Our aruoc tracker
  /// [中文] Aruco 标码前端 (可选), 与 trackFEATS 并存而非互斥
  std::shared_ptr<ov_core::TrackBase> trackARUCO;

  /// State initializer
  /// [中文] 初始化器, 内部同时持有静态和动态两种
  std::shared_ptr<ov_init::InertialInitializer> initializer;

  /// Boolean if we are initialized or not
  /// [中文] 初始化完成标记, 影响 track_image_and_update 的分支逻辑
  bool is_initialized_vio = false;

  /// Our MSCKF feature updater
  /// [中文] 短轨特征更新器 (零空间投影 + 卡方检验 + QR 压缩 + EKF)
  std::shared_ptr<UpdaterMSCKF> updaterMSCKF;

  /// Our SLAM/ARUCO feature updater
  /// [中文] 长轨 / Aruco 特征更新器, 包括 delayed_init 和 update
  std::shared_ptr<UpdaterSLAM> updaterSLAM;

  /// Our zero velocity tracker
  /// [中文] 零速更新器, 仅在静止时触发
  std::shared_ptr<UpdaterZeroVelocity> updaterZUPT;

  /// This is the queue of measurement times that have come in since we starting doing initialization
  /// After we initialize, we will want to prop & update to the latest timestamp quickly
  /// [中文] 初始化期间稯积的相机帧时间戳队列; 初始化成功后会快速重放这些时刻
  /// 以把状态推到最新。队列访问受 camera_queue_init_mtx 保护 (与异步初始化线程并发)。
  std::vector<double> camera_queue_init;
  std::mutex camera_queue_init_mtx;

  // Timing statistic file and variables
  std::ofstream of_statistics;
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // Track how much distance we have traveled
  double timelastupdate = -1;
  double distance = 0;

  // Startup time of the filter
  double startup_time = -1;

  // Threads and their atomics
  // [中文] 异步初始化的状态标志:
  //   thread_init_running = 是否正在初始化 (防止重入)
  //   thread_init_success = 初始化线程已经写完结果, 主线程可以读取
  std::atomic<bool> thread_init_running, thread_init_success;

  // If we did a zero velocity update
  bool did_zupt_update = false;
  bool has_moved_since_zupt = false;

  // Good features that where used in the last update (used in visualization)
  std::vector<Eigen::Vector3d> good_features_MSCKF;

  // Re-triangulated features 3d positions seen from the current frame (used in visualization)
  // For each feature we have a linear system A * p_FinG = b we create and increment their costs
  double active_tracks_time = -1;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
  cv::Mat active_image;
  std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A;
  std::map<size_t, Eigen::Vector3d> active_feat_linsys_b;
  std::map<size_t, int> active_feat_linsys_count;
};

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGER_H
