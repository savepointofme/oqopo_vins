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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <deque>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "FCInitLoader.h"
#include "VioManagerOptions.h"

namespace ov_core {
struct ImuData;
struct CameraData;
struct TrackerWarpVizPacket;
class TrackBase;
class FeatureInitializer;
} // namespace ov_core
namespace ov_init {
class InertialInitializer;
} // namespace ov_init
namespace ov_type {
class PoseJPL;
} // namespace ov_type

namespace ov_msckf {

class State;
class StateHelper;
class UpdaterMSCKF;
class UpdaterSLAM;
class UpdaterZeroVelocity;
class UpdaterGroundPlaneRange;
class UpdaterGroundPlaneFeature;
class UpdaterGroundPlaneFeatureV1;
class Propagator;
class VisualResidualDiag;

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

  ~VioManager();

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

  /// Initialize from a one-shot flight-controller state. This bypasses the
  /// visual DynamicInitializer and does not enable continuous FC/GPS fusion.
  void initialize_with_fc_state(const FCInitState &fc,
                                double sigma_att_rad,
                                double sigma_vel,
                                double sigma_pos,
                                double sigma_bg,
                                double sigma_ba,
                                double camera_timestamp);

  /**
   * @brief Feed a scalar GPS altitude (world Z) measurement, performs a 1D EKF
   *        update on the IMU position z component.
   *
   * 用途: mono VIO 下 accelerometer bias 不可观测导致 z 漂移. 把 GPS 高度当
   * 单维观测量锁住 z 轴, 间接稳住 ba_y/ba_z 的估计.
   *
   * @param timestamp  Time (sec) of the GPS altitude sample (sensor-clock).
   * @param altitude_z Altitude in VIO world frame (meters, ENU-up compatible).
   *                   Caller must align to VIO origin (subtract init altitude).
   * @param sigma      Measurement stddev (meters), e.g. 1.0-5.0 for RTK/GNSS.
   */
  void feed_measurement_gps_altitude(double timestamp, double altitude_z, double sigma,
                                     double chi2_gate = 10000.0,
                                     bool also_update_vz = false);

  /// Feed GPS altitude with VioManager-managed relative reference.
  /// On first call, captures gps_ref and vio_ref, then uses:
  ///   res = (altitude_z - gps_ref) - (p_z - vio_ref)
  /// Caller must pass raw GPS ENU z (without pre-subtracting a reference).
  void feed_measurement_gps_altitude_relative(double timestamp, double altitude_z, double sigma,
                                               double chi2_gate = 10000.0,
                                               bool also_update_vz = false);

  /// Periodic absolute/anchor pose repair. Intended for sparse reference-image
  /// matches; the ros-free runner can simulate this with GPS ENU position and
  /// course yaw. Roll/pitch are intentionally not accepted unless a real
  /// external attitude source is supplied by a future caller.
  struct PoseAnchorLastUpdate {
    double t = -1.0;
    double pos_residual_norm = std::numeric_limits<double>::quiet_NaN();
    double yaw_residual_deg = std::numeric_limits<double>::quiet_NaN();
    double nis = std::numeric_limits<double>::quiet_NaN();
    double gate_ratio = std::numeric_limits<double>::quiet_NaN();
    double predicted_pos_correction_norm = std::numeric_limits<double>::quiet_NaN();
    double predicted_yaw_correction_deg = std::numeric_limits<double>::quiet_NaN();
    bool yaw_used = false;
    bool accepted = false;
    std::string decision = "NONE";
  };
  bool feed_measurement_pose_anchor(double timestamp,
                                    const Eigen::Vector3d &p_IinG_meas,
                                    double yaw_meas_rad,
                                    bool yaw_valid,
                                    double pos_sigma,
                                    double yaw_sigma_rad,
                                    double gate_sigma,
                                    double max_pos_correction,
                                    double max_yaw_correction_rad);
  /// Trusted sparse global repair. This is used when an external reference-map
  /// match is considered authoritative enough that a large residual means VIO
  /// is lost, not that the reference should be rejected. Applies one global
  /// yaw+translation reset to IMU pose, clones, and global landmarks; optional
  /// velocity/bias values come from the FCInitState-style anchor.
  bool apply_trusted_pose_anchor_reset(const FCInitState &fc,
                                       double camera_timestamp,
                                       bool yaw_valid,
                                       bool velocity_valid,
                                       bool reset_biases);
  const PoseAnchorLastUpdate &get_last_pose_anchor_update() const { return pose_anchor_last_; }

  /// Set min P_zz floor to prevent covariance collapse (0 = disabled).
  /// Floor is applied BEFORE the EKF update so K_pz stays non-degenerate,
  /// and again AFTER the update so the next call also benefits.
  void set_gps_alt_min_pzz(double v) { gps_alt_min_pzz_ = v; }
  /// Delay GPS altitude / ground-plane updates by this many seconds after
  /// VIO init.  This prevents the z_ground bootstrap from anchoring to
  /// transient (un-settled) monocular scale.
  void set_gps_alt_min_t_after_init(double v) { gps_alt_min_t_after_init_ = v; }

  /// Enable PX4-style masked Joseph update for GPS-Z / scalar height aiding.
  /// Ported from PX4-Autopilot fuseHaglRng, commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909.
  /// Only p_z state DOF and p_z column/row of P are updated; all others unchanged.
  void set_gps_alt_joseph_update(bool v) { gps_alt_joseph_update_ = v; }

  /// Set innovation rejection gate (meters). Updates with |residual| > threshold
  /// are skipped entirely (no EKF update). Default = 1e9 (off).
  void set_gps_alt_max_res_gate(double v) { gps_alt_max_res_gate_ = v; }

  /// Set cross-covariance guard thresholds (0 = disabled).
  /// Reject GPS update if single-update XY position correction norm exceeds this (meters).
  void set_gps_alt_guard_dxy_max(double v) { gps_alt_guard_dxy_max_ = v; }
  /// Reject if norm(K_xy) / abs(K_pz) exceeds this (dimensionless gain ratio).
  void set_gps_alt_guard_kxy_ratio_max(double v) { gps_alt_guard_kxy_ratio_max_ = v; }
  /// Reject if bias correction norm (acc or gyro) exceeds this.
  void set_gps_alt_guard_dbias_max(double v) { gps_alt_guard_dbias_max_ = v; }

  /// Configure the staged GPS-Z coupled-update experiment.
  /// Modes: guarded, full, bounded, nasa_lean.
  /// All bounds are disabled when <= 0. Attitude is in radians.
  bool configure_gps_alt_coupled_update(const std::string &mode,
                                         double residual_soft_limit,
                                         double max_delta_xy,
                                         double max_delta_z,
                                         double max_delta_velocity,
                                         double max_delta_attitude,
                                         double max_delta_accel_bias,
                                         double max_delta_gyro_bias,
                                         double covariance_psd_check_interval);

  /// Configure NASA measurement underweighting for NASA_LEAN coupled mode.
  /// @param beta        Lear coefficient beta (>=0). 0 disables (reduces to standard).
  /// @param q_threshold Orion-style trigger: underweight only when H'PH > q_threshold (m^2).
  void set_gps_alt_nasa_underweight(double beta, double q_threshold) {
    gps_alt_nasa_beta_ = std::max(0.0, beta);
    gps_alt_nasa_q_threshold_ = std::max(0.0, q_threshold);
  }

  /// Detailed per-GPS-Z coupled-gain CSV, restricted by the mechanism window.
  void set_gps_alt_coupled_diag_path(const std::string &path);

  /// GPS XY is diagnostic reference only; it is never added to the EKF H/residual.
  void set_gps_alt_xy_diagnostic_reference(double timestamp,
                                            const Eigen::Vector2d &xy,
                                            bool valid = true);

  /// Visual yaw update control. A disabled yaw update is implemented inside
  /// EKFUpdate by scaling Kalman-gain orientation rows, not by restoring yaw.
  void set_enable_vio_yaw_update(bool v);
  void set_vio_yaw_update_mode(const std::string &mode);
  void set_vio_yaw_update_scale(double scale);
  void set_vio_global_yaw_oc_alpha(double alpha);
  void set_visual_bgz_update_scale(double scale);

  /// Open/replace the per-visual-update yaw diagnostic CSV.
  void set_vio_yaw_update_diag_path(const std::string &path);

  /// Open/replace the visual observability diagnostic CSV.
  void set_visual_obs_diag_path(const std::string &path);

  /// Mechanism-level local diagnostics. These only write rows inside [t0, t1].
  void set_mechanism_diag_window(double t0, double t1);
  void set_visual_flow_curl_diag_path(const std::string &path);
  void set_yaw_update_mechanism_diag_path(const std::string &path);
  void set_slam_feature_yaw_contrib_diag_path(const std::string &path);
  void set_visual_residual_diag_paths(const std::string &feature_path,
                                      const std::string &summary_path);
  void configure_slam_yaw_contrib_cap(bool enabled, double t0, double t1,
                                      double roll_deg, double yawrate_degps,
                                      int topk, double ratio,
                                      const std::string &mode);
  void set_slam_yaw_contrib_cap_diag_path(const std::string &path);
  void configure_slam_info_reduction(bool enabled, double low_ratio,
                                     double high_ratio, double alpha_max);
  void set_slam_info_reduction_diag_path(const std::string &path);
  void set_slam_ekf_leverage_diag_path(const std::string &path);
  void set_slam_stacked_ekf_diag_path(const std::string &path);
  void set_slam_landmark_metadata_diag_path(const std::string &path);
  void configure_slam_geometry_lifecycle_refresh(bool enabled,
                                                 double min_depth_current,
                                                 double min_age_since_added,
                                                 double min_pose_landmark_cov_norm,
                                                 int min_regular_features_after_refresh,
                                                 bool require_anchor_change);
  void set_slam_geometry_lifecycle_refresh_diag_path(const std::string &path);
  void set_reference_course_yaw(double timestamp, double yaw_deg, bool valid);
  void set_slam_update_freeze_window(double t0, double t1) {
    slam_freeze_t0_ = t0;
    slam_freeze_t1_ = t1;
  }

  /// Per-call diagnostic snapshot populated by every GPS altitude update.
  struct GpsAltLastUpdate {
    double t = -1.0;
    double gps_z = 0.0;    ///< GPS altitude fed (after reference subtraction by caller)
    double vio_z = 0.0;    ///< VIO p_z BEFORE update
    double residual = 0.0; ///< raw residual (GPS - VIO) before rejection gate
    double pzz = 0.0;      ///< P_zz BEFORE EKF update (after floor injection)
    double kpz = 0.0;      ///< Kalman gain for p_z
    double kpx = 0.0;      ///< Kalman gain for p_x
    double kpy = 0.0;      ///< Kalman gain for p_y
    double kxy_norm = 0.0; ///< norm(K_xy) = sqrt(K_px^2 + K_py^2)
    double dx = 0.0;       ///< predicted dx correction (m)
    double dy = 0.0;       ///< predicted dy correction (m)
    double dz = 0.0;       ///< predicted dz correction (m)
    double dtheta_norm = 0.0; ///< predicted orientation correction norm
    double dba_norm = 0.0;    ///< predicted accel bias correction norm
    double dbg_norm = 0.0;    ///< predicted gyro bias correction norm
    double chi2 = 0.0;
    double innovation_variance = 0.0;   ///< S = H'PH + R (standard innovation variance)
    double gain_scale = 1.0;            ///< BOUNDED-mode uniform gain scale alpha (1.0 otherwise)
    double nasa_beta = 0.0;             ///< NASA_LEAN underweight coefficient actually applied
    double r_effective = 0.0;           ///< measurement noise fed to Joseph (R_eff = R + beta*H'PH for NASA)
    bool large_coupled_correction = false;
    bool covariance_psd_checked = false;
    bool covariance_psd = false;
    bool clipped = false;  ///< always false (rejection gate replaces clip)
    std::string decision = "NONE"; ///< APPLY / REJECT_BY_DXY / REJECT_BY_GAIN_RATIO / REJECT_BY_BIAS
  };
  const GpsAltLastUpdate &get_last_gps_alt_update() const { return gps_alt_last_; }

  /// Last-frame MSCKF update statistics (chi2 rejected, accepted, etc.)
  struct MsckfLastStats {
    int n_features_in = 0;
    int n_tri_failed = 0;
    int n_chi2_rejected = 0;
    int n_accepted = 0;
    double chi2_sum_rej = 0.0;
    double chi2_max_rej = 0.0;
    double chi2_sum_acc = 0.0;
    double chi2_max_acc = 0.0;
    double chi2_threshold_sum_acc = 0.0;
    double chi2_threshold_sum_rej = 0.0;
    double chi2_threshold_max_acc = 0.0;
    double chi2_threshold_max_rej = 0.0;
    int track_len_sum_acc = 0;
    int track_len_max_acc = 0;
    int track_len_sum_rej = 0;
    int track_len_max_rej = 0;
    int uv_count_acc = 0;
    int uv_count_rej = 0;
    double uv_sum_u_acc = 0.0;
    double uv_sum_v_acc = 0.0;
    double uv_sum_u_rej = 0.0;
    double uv_sum_v_rej = 0.0;
    // Triangulation rejection breakdown (flat copy from FeatureInitializer::TriBatchStats)
    int    tri_cond_bad = 0;
    int    tri_depth_near = 0;
    int    tri_depth_far = 0;
    int    tri_nan = 0;
    int    tri_gn_depth_near = 0;
    int    tri_gn_depth_far = 0;
    int    tri_gn_baseline_ratio = 0;
    int    tri_gn_nan = 0;
    int    tri_n_geom = 0;
    double tri_cond_sum = 0, tri_cond_max = 0;
    double tri_depth_sum = 0, tri_depth_max = 0;
    double tri_base_sum  = 0, tri_base_max  = 0;
    double tri_ratio_sum = 0, tri_ratio_max = 0;
  };
  MsckfLastStats get_last_msckf_stats() const;

  struct VisualUpdateCounters {
    size_t msckf_update_count = 0;
    size_t regular_slam_update_count = 0;
    size_t delayed_slam_update_count = 0;
    size_t tracker_call_count = 0;
    size_t feature_database_insert_count = 0;
    size_t visual_update_frame_count = 0;
    size_t visual_update_eligible_count = 0;
    size_t visual_update_skipped_count = 0;
    size_t msckf_attempt_count = 0;
    size_t msckf_accept_count = 0;
    size_t msckf_features_attempted = 0;
    size_t msckf_features_accepted = 0;
    size_t regular_slam_attempt_count = 0;
    size_t regular_slam_accept_count = 0;
    size_t regular_slam_features_attempted = 0;
    size_t regular_slam_features_accepted = 0;
    size_t delayed_slam_attempt_count = 0;
    size_t delayed_slam_accept_count = 0;
    size_t delayed_slam_features_attempted = 0;
    size_t delayed_slam_features_accepted = 0;
    size_t tracks_retained_final = 0;
    size_t tracks_retained_max = 0;
    size_t tracks_consumed_count = 0;
    size_t tracks_dropped_without_update = 0;
    size_t visual_update_adaptive_enabled = 0;
    size_t visual_update_adaptive_keyframe_count = 0;
    size_t visual_update_adaptive_skip_count = 0;
    size_t visual_update_adaptive_flow_trigger_count = 0;
    size_t visual_update_adaptive_maxdt_trigger_count = 0;
    size_t visual_update_adaptive_track_trigger_count = 0;
    size_t visual_update_adaptive_first_trigger_count = 0;
    size_t visual_update_adaptive_min_dt_block_count = 0;
    size_t visual_update_adaptive_drop_current_observations_count = 0;
    double visual_update_adaptive_last_dt_s = 0.0;
    double visual_update_adaptive_last_frame_flow_px = 0.0;
    double visual_update_adaptive_last_accum_flow_px = 0.0;
    int visual_update_adaptive_last_track_count = -1;
  };
  const VisualUpdateCounters &get_visual_update_counters() const { return visual_update_counters_; }

  /// Reset GPS altitude bootstrap state (for all modes). Call between runs.
  void reset_gps_altitude_bootstrap() {
    gps_alt_bootstrapped_ = false;
    gps_alt_z_ground_ = 0.0;
    gps_alt_rel_bootstrapped_ = false;
    gps_alt_rel_gps_ref_ = 0.0;
    gps_alt_rel_vio_ref_ = 0.0;
  }

  /// Print final GPS altitude fusion statistics summary.
  void print_gps_alt_final_summary();

  /// Feed a scalar height measurement that bootstraps the local ground plane.
  /// GPS altitude uses h(x)=p_z. Rangefinder/LiDAR uses h(x)=(p_z-z_ground)/cos_tilt.
  bool feed_measurement_gps_ground_plane(double timestamp, double height,
                                          double sigma = 0.3,
                                          bool rangefinder = false);

  /// Access the ground-plane range updater for diagnostics.
  std::shared_ptr<UpdaterGroundPlaneRange> get_updater_gplane_range() { return updaterGPlaneRange; }

  /// Enable Stage B (ground-plane feature update).  z_ground is taken from the
  /// Stage A bootstrap (must be enabled and bootstrapped for Stage B to fire).
  void enable_gplane_feature(double sigma_pixel = 3.0, int max_features = 5,
                             double center_frac = 0.8, double min_cos_tilt = 0.85,
                             double max_residual_px = 5.0);

  /// Access the ground-plane feature updater for diagnostics.
  std::shared_ptr<UpdaterGroundPlaneFeature> get_updater_gplane_feature() { return updaterGPlaneFeature; }

  /// Enable Stage B v1 (two-clone H, anchor pose is a state, not fixed).
  /// dry_run=true builds analytic + finite-difference Jacobian and logs the
  /// comparison without ever calling EKFUpdate.  Activation policy mirrors
  /// Stage A's: this fires only after z_ground is bootstrapped and the
  /// Stage-A min-t-after-init delay has elapsed (so callers don't need to
  /// gate themselves).
  void enable_gplane_feature_v1(bool dry_run,
                                double sigma_pixel = 50.0,        // E1 default (was 3.0)
                                int max_features = 2,             // E1 default (was 5)
                                double center_frac = 0.8,
                                double min_cos_tilt = 0.85,
                                double max_residual_px = 5.0,
                                double fd_step_rot = 1e-4,
                                double fd_step_pos = 1e-2,
                                double fd_rel_tol_rot = 1e-3,
                                double fd_rel_tol_pos = 3e-3,
                                double fd_max_abs_rel_tol = 1e-2,
                                bool exclude_used_from_msckf = false, // E1 default (was true)
                                int dump_first_n = 0);

  /// Access the v1 updater for diagnostics.
  std::shared_ptr<UpdaterGroundPlaneFeatureV1> get_updater_gplane_feature_v1() { return updaterGPlaneFeatureV1; }

  /// If we are initialized or not
  /// [中文] 判断系统是否已初始化: 必须既完成初始化又至少做过一次更新。
  bool initialized() { return is_initialized_vio && timelastupdate != -1; }

  /// Timestamp that the system was initialized at
  double initialized_time() { return startup_time; }

  /// Accessor for current system parameters
  VioManagerOptions get_params() { return params; }

  /// Read-only real-time IMU filter diagnostics.
  const ImuFilterStats &get_imu_filter_stats() const { return imu_filter.stats(); }

  /// Accessor to get the current state
  std::shared_ptr<State> get_state() { return state; }

  /// Accessor to get the current propagator
  std::shared_ptr<Propagator> get_propagator() { return propagator; }

  /// Forward the latest per-camera warp visualization packet from the tracker.
  bool get_warp_viz_packet(size_t cam_id, ov_core::TrackerWarpVizPacket &packet) {
    return trackFEATS->get_warp_viz_packet(cam_id, packet);
  }

  /// Get a nice visualization image of what tracks we have
  cv::Mat get_historical_viz_image();

  /// Returns 3d SLAM features in the global frame
  std::vector<Eigen::Vector3d> get_features_SLAM();

  /// Returns 3d ARUCO features in the global frame
  std::vector<Eigen::Vector3d> get_features_ARUCO();

  /// Returns 3d features used in the last update in global frame
  std::vector<Eigen::Vector3d> get_good_features_MSCKF() { return good_features_MSCKF; }

  /// Number of feature tracks currently stored in the frontend database.
  int get_feature_database_size();

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

  /// Wrap one visual update with yaw before/after logging.
  void apply_visual_update_with_yaw_diag(const std::string &update_type,
                                         const std::function<void()> &update_fn,
                                         int num_features,
                                         double chi2,
                                         int accepted);

  /// Current yaw from IMU orientation, in degrees.
  double current_imu_yaw_deg() const;

  /// Restore yaw in the IMU mean while preserving current roll/pitch.
  void restore_imu_yaw_deg(double yaw_deg);

  /// Snapshot of all pose yaws that visual updates are allowed to touch.
  struct PoseYawSnapshot {
    double imu_yaw_deg = 0.0;
    std::vector<std::pair<double, double>> clone_yaw_deg;
  };

  /// Current yaw from a cloned pose orientation, in degrees.
  double pose_yaw_deg(const std::shared_ptr<ov_type::PoseJPL> &pose) const;

  /// Restore yaw in a cloned pose while preserving current roll/pitch.
  void restore_pose_yaw_deg(const std::shared_ptr<ov_type::PoseJPL> &pose, double yaw_deg);

  /// Save/restore current IMU + all clone yaws for no-yaw visual A/B runs.
  PoseYawSnapshot snapshot_vio_pose_yaws() const;
  void restore_vio_pose_yaws(const PoseYawSnapshot &snapshot);

  /// Write one row to vio_yaw_update_diag_path, if enabled.
  void log_vio_yaw_update(double timestamp, const std::string &update_type,
                          double yaw_before_deg, double yaw_after_deg,
                          double delta_yaw_deg, double bg_z,
                          int num_features, double chi2, int accepted, int rejected,
                          int tracking_feature_count);

  /// Write one row to visual_observability_diag CSV, if enabled.
  void log_visual_obs_diag(double timestamp, const std::string &update_type,
                           int n_in, int n_accepted, int n_rejected,
                           double norm_HN_before, double norm_HN_after,
                           double rel_HN_before, double rel_HN_after,
                           int rank_N, double condition_N,
                           bool projection_applied, bool used_fej,
                           double yaw_delta_deg,
                           double chi2_before, double chi2_after);

  struct VisualFlowSummary {
    bool valid = false;
    double timestamp = 0.0;
    size_t cam_id = 0;
    int image_width = 0;
    int image_height = 0;
    int num_tracks = 0;
    int num_features_total = 0;
    int num_klt_attempted = 0;
    int num_klt_newly_detected = 0;
    bool warp_active = false;
    double mean_u = 0.0, mean_v = 0.0, std_u = 0.0, std_v = 0.0;
    int count_left = 0, count_right = 0, count_top = 0, count_bottom = 0;
    int count_q1 = 0, count_q2 = 0, count_q3 = 0, count_q4 = 0;
    double mean_du = 0.0, mean_dv = 0.0, std_du = 0.0, std_dv = 0.0;
    double mean_flow_mag = 0.0, p95_flow_mag = 0.0;
    double flow_curl_proxy = 0.0, flow_radial_proxy = 0.0, flow_tangential_proxy = 0.0;
    double left_right_flow_asymmetry = 0.0, top_bottom_flow_asymmetry = 0.0;
    double mean_track_age = 0.0, median_track_age = 0.0;
    int new_track_count = 0, lost_track_count = 0;
  };

  bool in_mechanism_diag_window(double timestamp) const;
  bool in_slam_update_freeze_window(double timestamp) const;
  void collect_visual_flow_curl_diag(const ov_core::CameraData &message);
  VisualFlowSummary summarize_tracker_packet(const ov_core::TrackerWarpVizPacket &pkt);
  void log_visual_flow_curl_diag(double timestamp);
  void log_yaw_update_mechanism(double timestamp, const std::string &update_type,
                                double state_yaw_before_deg, double state_yaw_after_deg,
                                double vio_course_yaw_before_deg, double vio_course_yaw_after_deg,
                                double delta_yaw_deg, int num_features, double chi2,
                                int accepted, int rejected, int tracking_feature_count);

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

  /// Single processing point shared by propagation, initialization, and ZUPT.
  ImuFilter imu_filter;

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

  /// Ground-plane pseudo-rangefinder updater
  /// [中文] 地面平面伪测距更新器
  std::shared_ptr<UpdaterGroundPlaneRange> updaterGPlaneRange;

  /// Stage B — ground-plane feature updater (optional, off by default)
  std::shared_ptr<UpdaterGroundPlaneFeature> updaterGPlaneFeature;

  /// Stage B v1 — two-clone H, dry-run-first updater (optional, off by default)
  std::shared_ptr<UpdaterGroundPlaneFeatureV1> updaterGPlaneFeatureV1;

  /// This is the queue of measurement times that have come in since we starting doing initialization
  /// After we initialize, we will want to prop & update to the latest timestamp quickly
  /// [中文] 初始化期间稯积的相机帧时间戳队列; 初始化成功后会快速重放这些时刻
  /// 以把状态推到最新。队列访问受 camera_queue_init_mtx 保护 (与异步初始化线程并发)。
  std::vector<double> camera_queue_init;
  std::mutex camera_queue_init_mtx;

  // Timing statistic file and variables
  std::ofstream of_statistics;
  std::ofstream of_vio_yaw_update_diag;
  double vio_yaw_update_diag_cumsum_deg = 0.0;
  VisualUpdateCounters visual_update_counters_;
  std::ofstream of_visual_obs_diag;
  double visual_obs_diag_cumsum_yaw_deg = 0.0;
  std::ofstream of_visual_flow_curl_diag;
  std::ofstream of_yaw_update_mechanism_diag;
  std::ofstream of_gps_alt_coupled_diag;
  std::shared_ptr<VisualResidualDiag> visual_residual_diag_;
  double mechanism_diag_t0_ = -std::numeric_limits<double>::infinity();
  double mechanism_diag_t1_ = std::numeric_limits<double>::infinity();
  double reference_course_time_ = -1.0;
  double reference_course_yaw_deg_ = 0.0;
  double reference_course_yaw_rate_degps_ = std::numeric_limits<double>::quiet_NaN();
  bool reference_course_valid_ = false;
  double slam_freeze_t0_ = -1.0;
  double slam_freeze_t1_ = -1.0;
  std::unordered_map<size_t, VisualFlowSummary> latest_visual_flow_by_cam_;
  std::unordered_map<size_t, std::unordered_map<size_t, int>> flow_track_age_by_cam_;
  std::unordered_map<size_t, std::unordered_set<size_t>> flow_prev_ids_by_cam_;

  // Visual update guard: skip window, reject-file, guard log
  double visual_skip_t0_ = -1.0;   // --visual-update-skip-window start
  double visual_skip_t1_ = -1.0;   // --visual-update-skip-window end
  std::vector<std::pair<double,double>> visual_reject_intervals_;  // from --visual-update-reject-topn-file
  std::ofstream of_visual_guard_log_;
  bool visual_guard_log_header_written_ = false;
  // Burst state for guarded mode (populated in commit 2)
  std::deque<double> guard_burst_times_;

public:
  void set_visual_skip_window(double t0, double t1) { visual_skip_t0_ = t0; visual_skip_t1_ = t1; }
  void set_visual_update_stride(int stride) { visual_update_stride_ = std::max(1, stride); }
  int get_visual_update_stride() const { return visual_update_stride_; }
  void configure_visual_update_adaptive(bool enabled,
                                        double target_flow_px,
                                        double min_dt,
                                        double max_dt,
                                        int min_tracks);
  void open_visual_guard_log(const std::string &path);
  void load_visual_reject_file(const std::string &path);

private:
  int visual_update_stride_ = 1;
  bool visual_update_adaptive_enable_ = false;
  double visual_update_adaptive_target_flow_px_ = 6.0;
  double visual_update_adaptive_min_dt_ = 0.06;
  double visual_update_adaptive_max_dt_ = 0.18;
  int visual_update_adaptive_min_tracks_ = 240;
  double visual_update_adaptive_last_update_time_ = -1.0;
  double visual_update_adaptive_accum_flow_px_ = 0.0;

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

  // [C-mode] GPS altitude / range bootstrap: z_ground estimated once from first measurement
  bool gps_alt_bootstrapped_ = false;
  double gps_alt_z_ground_ = 0.0;

  // [Relative mode] GPS altitude relative reference bootstrap
  bool gps_alt_rel_bootstrapped_ = false;
  double gps_alt_rel_gps_ref_ = 0.0;
  double gps_alt_rel_vio_ref_ = 0.0;

  // [P_zz floor] Minimum P_zz to prevent Kalman gain collapse (0 = disabled)
  double gps_alt_min_pzz_ = 0.0;
  double gps_alt_min_t_after_init_ = 0.0; // delay z_ground bootstrap (s)

  // [Innovation gate] Max |residual| before rejecting update entirely (1e9 = disabled)
  double gps_alt_max_res_gate_ = 1e9;

  // [Cross-covariance guard] Reject update if predicted XY correction norm > this (m). 0=off.
  double gps_alt_guard_dxy_max_ = 0.0;
  // [Cross-covariance guard] Reject if norm(K_xy)/abs(K_pz) > this. 0=off.
  double gps_alt_guard_kxy_ratio_max_ = 0.0;
  // [Cross-covariance guard] Reject if bias correction norm > this. 0=off.
  double gps_alt_guard_dbias_max_ = 0.0;

  enum class GpsAltCoupledMode { GUARDED, FULL, BOUNDED, NASA_LEAN };
  GpsAltCoupledMode gps_alt_coupled_mode_ = GpsAltCoupledMode::GUARDED;
  // NASA measurement underweighting (NASA_LEAN mode only).
  double gps_alt_nasa_beta_ = 0.0;        // Lear coefficient beta (>=0; 0 disables)
  double gps_alt_nasa_q_threshold_ = 0.0; // Orion-style trigger on H'PH (m^2)
  double gps_alt_residual_soft_limit_ = 0.0;
  double gps_alt_max_delta_xy_ = 0.0;
  double gps_alt_max_delta_z_ = 0.0;
  double gps_alt_max_delta_velocity_ = 0.0;
  double gps_alt_max_delta_attitude_ = 0.0;
  double gps_alt_max_delta_accel_bias_ = 0.0;
  double gps_alt_max_delta_gyro_bias_ = 0.0;
  double gps_alt_covariance_psd_check_interval_ = 1.0;
  double gps_alt_last_covariance_psd_check_time_ = -1.0;
  double gps_alt_diag_xy_time_ = -1.0;
  Eigen::Vector2d gps_alt_diag_xy_ = Eigen::Vector2d::Zero();
  bool gps_alt_diag_xy_valid_ = false;

  // [Joseph masked] PX4-style masked Joseph update: only p_z state DOF and p_z row/col updated.
  // Ported from PX4-Autopilot fuseHaglRng, commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909.
  bool gps_alt_joseph_update_ = false;

public:
  /// Per-evaluation statistics for GPS altitude fusion diagnostics.
  struct GpsAltStats {
    size_t n_called = 0;
    size_t n_skipped = 0;      // time-alignment skip (|dt| > 0.2s)
    size_t n_rejected = 0;     // residual gate rejection
    size_t n_rejected_dxy = 0; // guard: XY correction too large
    size_t n_rejected_kxy = 0; // guard: K_xy/K_pz ratio too large
    size_t n_rejected_bias = 0;// guard: bias correction too large
    size_t n_large_coupled = 0;// large correction observed (not necessarily rejected)
    size_t n_bounded = 0;      // Test 2 update used gain scaling
    size_t n_nasa_underweight = 0; // NASA/Lear underweighting was applied (beta>0 & q>thresh)
    size_t n_rejected_numerical = 0; // transactional Joseph health failure
    size_t n_accepted = 0;     // EKF update applied
    // --- GPS-Z fusion watchdog bookkeeping (interface for a future forced-aiding
    //     / recovery stage; this round only records, it does not act). ---
    double last_measurement_time = -1.0;        // last healthy GPS-Z call reached the update path
    double last_successful_fusion_time = -1.0;  // last GPS-Z update actually applied
    size_t consecutive_failures = 0;            // failed/rejected GPS-Z since last success
    double longest_no_fusion_duration = 0.0;    // max gap between successful fusions (s)
    double sum_K_pz = 0.0;
    double sum_K_xy_norm = 0.0;
    double sum_dxy_norm = 0.0;
    double sum_dtheta_norm = 0.0;
    double sum_dba_norm = 0.0;
    double sum_dbg_norm = 0.0;
    double sum_P_zz = 0.0;
    double sum_abs_res = 0.0;
    double sum_abs_dpz = 0.0;
    double last_summary_time = -1.0;
    double first_eval_time = -1.0;
    double last_eval_time = -1.0;
  };
  const GpsAltStats &get_gps_alt_stats() const { return gps_alt_stats_; }

private:
  GpsAltStats gps_alt_stats_;
  GpsAltLastUpdate gps_alt_last_;
  PoseAnchorLastUpdate pose_anchor_last_;

  // [Gyro-aided KLT] Per-camera last image timestamp used for inter-frame
  // gyro integration. Initialised to -1 (no previous image) so the first
  // frame falls back to the identity rotation.
  std::unordered_map<size_t, double> last_track_image_time_;

  // [Ground-parallel warp] Stores the PREVIOUS frame's R_GtoC per camera.
  // R_comp = R_GtoC_curr * R_GtoC_prev^T (frame-to-frame) is passed to TrackKLT.
  // Updated to R_GtoC_curr each frame after the warp is computed.
  bool gravity_warp_ref_set_ = false;
  std::unordered_map<size_t, Eigen::Matrix3d> gravity_warp_R_ref_;

  /// [Landing watch] rolling (t, n_slam) for drop-rate detection
  std::deque<std::pair<double, int>> slam_count_history_;
  /// [Landing gate] latched once altitude drops below threshold (with hysteresis)
  bool landing_update_halted_ = false;
  /// [Landing gate] one-shot to avoid log spam
  bool landing_halt_msg_emitted_ = false;

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
