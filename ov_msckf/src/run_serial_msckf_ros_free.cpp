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

// ROS-free offline runner for OpenVINS. Feeds IMU / stereo-or-mono camera data
// from an ASL/EuRoC-style dataset folder into VioManager and optionally renders
// a OpenCV-based live dashboard comparing against GT / GPS.
//
// [中文] ROS-free 离线回放主入口。遵循 OpenVINS 官方 ROS-free 指南
// (https://docs.openvins.com/gs-installing-free.html): 调用方自行构造
// VioManagerOptions 并按时间戳顺序把 IMU 和相机数据喂给 VioManager。

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/StateHelper.h"
#include "update/UpdaterGroundPlaneRange.h"
#include "update/UpdaterGroundPlaneFeature.h"
#include "update/UpdaterGroundPlaneFeatureV1.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "ros_free/DatasetReaderEuroc.h"
#include "ros_free/AdaptiveStrideController.h"
#include "ros_free/DiagLogger.h"
#include "ros_free/DiagMetrics.h"
#include "ros_free/DiagPrinter.h"
#include "ros_free/TrajectoryAligner.h"
#include "ros_free/VizDashboard.h"
#include "utils/quat_ops.h"

using namespace ov_msckf;
namespace fs = boost::filesystem;

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct Args {
  std::string config_path;
  std::string dataset_dir;
  std::string gps_path;
  std::string gt_path;
  std::string output_path = "traj_ros_free.txt";
  std::string init_from_fc_path;
  std::string video_path;
  std::string video_cam_path;    // [中文] 仅相机 + 光流轨迹视频 (cam0/cam1 并排, 带 TrackBase 历史线)
  int video_fps = 20;
  double align_seconds = 8.0;    // [中文] 初始化完成后收集多少秒数据再做 SE3 对齐
  double start_time = 0.0;       // [中文] 跳过前 N 秒数据 (相对 bag 第一条 IMU), 用于复现 ROS bag_start 行为
  double until_time = std::numeric_limits<double>::infinity(); // stop processing when cam timestamp exceeds this
  bool stereo = false;           // [中文] 使用 cam1 配对
  bool gps_alt_update = false;   // [中文] 使用 GPS 高度作为 VIO EKF 1D 观测 (锁 z 漂移)
  double gps_alt_sigma = 2.0;    // [中文] GPS 高度观测噪声 stddev (meters)
  double gps_alt_chi2 = 10000.0; // [中文] GPS 高度 chi^2 门 (默认宽松不拒绝, 调小可防大残差污染 ba)
  bool gps_alt_schmidt = false;  // [中文] 使用 Schmidt consider-filter 避免 ba/bg 被 z 残差污染
  bool gps_alt_also_vz = false;  // [中文] active 集合加入 v(), 让 v_z 随 z 一起被 update
  bool gps_alt_range_mode = false; // [中文] C-mode: range model h=(p_z-z_ground)/r22, z_ground 首帧 bootstrap
  bool gps_alt_relative = false;  // [中文] 相对高度模型: res=(gps_now-gps_ref)-(p_z-vio_ref), VioManager 管 bootstrap
  double gps_alt_min_pzz = 0.0;          // [中文] P_zz 地板: 防止 K_pz 坍缩 (0=禁用, 建议 0.005-0.02)
  double gps_alt_min_t_after_init = 0.0; // delay Stage A z_ground bootstrap (s)
  double gps_alt_max_res = 1e9;          // [中文] 创新拒绝门: |residual| > 此值则跳过更新 (1e9=禁用, 建议 30)
  double gps_alt_guard_dxy = 0.0;        // [中文] 交叉协方差 guard: |dxy|_pred > 此值则拒 (0=禁用, 建议 0.5)
  double gps_alt_guard_kxy_ratio = 0.0;  // [中文] 交叉协方差 guard: |K_xy|/|K_pz| > 此值则拒 (0=禁用, 建议 0.3)
  double gps_alt_guard_dbias = 0.0;      // [中文] 交叉协方差 guard: |dbias|_pred > 此值则拒 (0=禁用)
  std::string gps_alt_coupled_mode = "legacy_guarded"; // Test0/1/2
  std::string gps_alt_coupled_diag_path;
  double gps_alt_residual_soft_limit = 0.0;
  double gps_alt_max_delta_xy = 0.0;
  double gps_alt_max_delta_z = 0.0;
  double gps_alt_max_delta_velocity = 0.0;
  double gps_alt_max_delta_attitude_deg = 0.0;
  double gps_alt_max_delta_accel_bias = 0.0;
  double gps_alt_max_delta_gyro_bias = 0.0;
  double gps_alt_cov_psd_check_interval = 1.0;
  double gps_alt_nasa_beta = 0.0;         // NASA/Lear underweight coefficient beta (>=0; 0 disables)
  double gps_alt_nasa_q_threshold = 0.0;  // NASA/Lear Orion trigger on H'PH (m^2)
  bool gps_alt_arch_g = false;           // Architecture G: h_offset Vec(1) augmented state
  bool gps_alt_zonly = false;            // legacy Z-only: full cov update, state p_z only
  bool gps_alt_joseph = false;           // PX4-style masked Joseph update (Gate 2)
  bool gps_alt_ground_plane = false;     // [中文] 地面平面伪测距模式: 将 GPS 高度转换为斜距观测
  // -- Stage B (ground-plane feature update) --
  // Shared defaults between v0 and v1 — chosen to match the empirically-best
  // v0 R5b config (sigma=50, K=2) and v1 E1 config (sigma=50, K=2, excl=false).
  bool gplane_feat_enable = false;        // turn on Stage B
  double gplane_feat_sigma_px = 50.0;     // E1/R5b default (was 3.0)
  int gplane_feat_max_features = 2;       // E1/R5b default (was 5)
  double gplane_feat_center_frac = 0.8;   // restrict anchor pixel to central fraction
  double gplane_feat_min_cos_tilt = 0.85; // skip when drone tilted
  double gplane_feat_max_res_px = 5.0;    // reject feature if predicted residual > this
  // -- Stage B v1 (two-clone H + dry-run/FD) --
  bool gplane_feat_v1_enable = false;     // turn on v1 (mutually exclusive with v0 in practice)
  bool gplane_feat_v1_dry_run = true;     // default: dry-run, no EKF update
  bool gplane_feat_v1_update = false;     // when --gplane-feat-v1-update is passed, overrides dry-run
  double gplane_feat_v1_fd_step_rot = 1e-4;     // tuned for camera quantization floor
  double gplane_feat_v1_fd_step_pos = 1e-2;     // tuned for camera quantization floor
  double gplane_feat_v1_fd_rel_tol_rot = 1e-3;  // rotation blocks (tha, thc)
  double gplane_feat_v1_fd_rel_tol_pos = 3e-3;  // position blocks (pa, pc)
  double gplane_feat_v1_fd_max_abs_rel_tol = 1e-2; // |max(A-F)|/||A,F||
  bool gplane_feat_exclude_used_from_msckf = false;  // E1 default (was true)
  int gplane_feat_v1_fd_dump = 0;        // dump full matrices for first N features
  double gps_cutoff_time = -1.0;         // [中文] Hold-out 评估: 超过 t_cam > cutoff 后不再 feed GPS, 看 VIO 裸跑
  double gps_feed_every = 1.0;           // [中文] GPS 喂入比例 1.0=全部, 0.2=每 5 个采样用 1 个 (验证降采样)
  // [中文] CLI 覆盖 yaml 里的 gps_time_offset; NaN = 不覆盖, 用 yaml/默认值
  double gps_time_offset_cli = std::numeric_limits<double>::quiet_NaN();
  double init_att_sigma_deg = 5.0;
  double init_vel_sigma = 5.0;
  double init_pos_sigma = 100.0;
  double init_bg_sigma = 0.05;
  double init_ba_sigma = 1.0;
  bool show = true;              // [中文] 显示窗口
  std::string dash_title = "OpenVINS ROS-free Dashboard";
  int dash_every = 1;            // [中文] 每 N 帧刷新仪表板
  int cam_subsample = 1;         // --camera-frame-stride/--cam-subsample N: feed only 1 of every N camera frames to VIO
  bool adaptive_stride_shadow = false; // calculate/log policy but preserve fixed input stride
  bool adaptive_stride = false;        // actively apply the causal 1..5 policy
  std::string adaptive_stride_log_path;
  bool camera_frame_adaptive = false; // --camera-frame-adaptive: online geometry-gated input-frame sampler
  double camera_frame_adaptive_target_ratio = 0.01656506713377612;
  double camera_frame_adaptive_min_dt = 0.11;
  double camera_frame_adaptive_max_dt = 0.18;
  double camera_frame_adaptive_min_depth = 0.0;
  int camera_frame_adaptive_min_tracks = 240;
  int visual_update_stride = 1;  // --visual-update-stride N: track every fed frame, update visual EKF every Nth frame
  bool visual_update_adaptive = false; // --visual-update-adaptive: full-rate KLT, adaptive visual keyframe observations
  double visual_update_adaptive_target_flow_px = 6.0;
  double visual_update_adaptive_min_dt = 0.06;
  double visual_update_adaptive_max_dt = 0.18;
  int visual_update_adaptive_min_tracks = 240;
  std::string camera_stride_audit_path; // --camera-stride-audit: one-row CSV proving input stride
  bool use_ground_parallel_warp = false; // --use-ground-parallel-warp: warp prev image by gravity rotation
  bool viz_fast = false;         // [中文] 快速仪表板模式 (轨迹/曲线抽帧, 特征限 200)
  bool verbose_timing = false;
  bool no_vio_yaw_update = false; // alias for --vio-yaw-update-scale 0.0
  std::string vio_yaw_update_mode;
  double vio_yaw_update_scale = std::numeric_limits<double>::quiet_NaN();
  double vio_global_yaw_oc_alpha = std::numeric_limits<double>::quiet_NaN();
  double vio_yaw_control_start_after_init = 0.0; // seconds; 0 = apply requested yaw control immediately
  double visual_bgz_update_scale = 1.0;          // --visual-bgz-update-scale: scale bg_z row of visual K
  double curl_correction_rate_degps = 0.0;       // --curl-correction-rate: camera-fixed curl bias correction (deg/s)
  // Timed B→A staged switch (D variants)
  std::string vio_yaw_switch_mode;              // --vio-yaw-switch-mode: switch to this mode at vio_yaw_switch_time
  double vio_yaw_switch_time = -1.0;            // --vio-yaw-switch-time: absolute timestamp (s) for switch
  double vio_yaw_switch_alpha = std::numeric_limits<double>::quiet_NaN(); // --vio-yaw-switch-alpha
  std::string vio_yaw_diag_path;           // per-visual-update yaw CSV
  std::string visual_obs_diag_path;        // VisualObservabilityPolicy per-update CSV
  std::string visual_flow_curl_diag_path;  // mechanism-level tracker flow/curl CSV
  std::string yaw_update_mechanism_diag_path; // reference-course yaw update mechanism CSV
  std::string imu_propagation_yaw_diag_path;  // propagation yaw-rate CSV
  std::string slam_feature_yaw_contrib_diag_path; // per-feature SLAM residual/yaw contribution CSV
  std::string visual_feature_residual_diag_path; // per-feature MSCKF/SLAM residual gate CSV
  std::string visual_residual_frame_summary_path; // per-update residual summary CSV
  bool slam_yaw_contrib_cap_enable = false; // roll-gated frame-local top-k SLAM yaw contribution cap
  double slam_yaw_contrib_cap_t0 = -std::numeric_limits<double>::infinity();
  double slam_yaw_contrib_cap_t1 = std::numeric_limits<double>::infinity();
  double slam_yaw_contrib_cap_roll_deg = 10.0;
  double slam_yaw_contrib_cap_yawrate_degps = 3.0;
  int slam_yaw_contrib_cap_topk = 5;
  double slam_yaw_contrib_cap_ratio = 0.5;
  std::string slam_yaw_contrib_cap_mode = "soft_scale";
  std::string slam_yaw_contrib_cap_diag_path;
  bool slam_info_reduction_enable = false;
  double slam_info_reduction_low_ratio = 0.2;
  double slam_info_reduction_high_ratio = 0.5;
  double slam_info_reduction_alpha_max = 4.0;
  std::string slam_info_reduction_diag_path;
  std::string slam_ekf_leverage_diag_path;
  std::string slam_stacked_ekf_diag_path;
  std::string slam_landmark_metadata_diag_path;
  bool slam_geometry_lifecycle_refresh_enable = false;
  double slam_geometry_refresh_min_depth = 400.0;
  double slam_geometry_refresh_min_age = 3.0;
  double slam_geometry_refresh_min_pose_lm_cov = 0.0;
  int slam_geometry_refresh_min_regular_features = 0;
  bool slam_geometry_refresh_require_anchor_change = false;
  std::string slam_geometry_refresh_diag_path;
  std::string state_safety_diag_path;     // --state-safety-diag: read-only state/covariance health CSV
  int state_safety_eig_every = 0;         // compute full covariance min eigen every N fed camera frames (0=off)
  double mech_diag_t0 = -std::numeric_limits<double>::infinity();
  double mech_diag_t1 = std::numeric_limits<double>::infinity();
  double slam_freeze_t0 = -1.0;          // --slam-update-freeze-window start
  double slam_freeze_t1 = -1.0;          // --slam-update-freeze-window end
  std::string schmidt_yaw_diag_path;       // Schmidt yaw update diagnostics CSV
  // Atomic experiment selection for explicit OC / FEJ combinations.
  std::string vio_consistency_mode;
  // High-level gauge-mode alias: expands to vio_yaw_update_mode + alpha
  std::string vio_yaw_gauge_mode;
  // Visual update guard (commit 1)
  double visual_skip_t0 = -1.0;           // --visual-update-skip-window start
  double visual_skip_t1 = -1.0;           // --visual-update-skip-window end
  std::string visual_guard_log_path;       // --visual-update-guard-log
  std::string visual_reject_file_path;     // --visual-update-reject-topn-file
  // Guarded-B thresholds (commit 2)
  StateHelper::SchmidtGuardConfig guard_cfg;  // defaults already set in struct
  std::string guard_diag_log_path;        // --visual-guard-diag-log
  // VINS-inspired numerical nullspace (Route 4 candidate)
  StateHelper::VinsNullspaceConfig vins_cfg;  // defaults in struct: eig_thresh=1e-6
  // Diagnostic overrides
  double cam_toff_override = std::numeric_limits<double>::quiet_NaN(); // override timeshift_cam_imu; disables online calib
  int diag_chi2_trigger = 5;    // trigger detailed diagnostics when chi2_rej >= this in one frame
  int diag_window = 15;         // print detailed diagnostics for this many frames after trigger
  // Diagnostic printing and logging (all off by default)
  bool diag_print = false;       // --diag-print: print compact [DIAG] blocks
  int  diag_print_every = 30;    // --diag-print-every N: print every N camera frames
  std::string run_name;          // --run-name: label written into CSV/event logs
  std::string diag_csv_path;     // --diag-csv: per-frame diagnostic CSV (post-flight plotting)
  std::string diag_events_path;  // --diag-events: human-readable event log
};

void print_help() {
  std::cout << "Usage: run_serial_msckf_ros_free --config <estimator_config.yaml> --dataset <MAV0_DIR> [options]\n"
               "\n"
               "Required:\n"
               "  --config PATH         Path to OpenVINS estimator YAML\n"
               "  --dataset DIR         ASL/EuRoC mav0 directory containing imu0/ cam0/ [cam1/]\n"
               "\n"
               "Optional:\n"
               "  --stereo              Use cam1 in addition to cam0 (must be in config as well)\n"
               "  --gps PATH            CSV: ts_ns, x, y, z  or  ts_ns, lat, lon, alt (WGS84)\n"
               "  --gps-alt-update      Feed GPS altitude (z) as 1D EKF update (mono rescue)\n"
               "  --gps-alt-sigma SIG   GPS altitude noise stddev meters (default 2.0)\n"
               "  --gps-alt-min-pzz V   P_zz floor to prevent K_pz collapse (0=off, try 0.01)\n"
               "  --gps-alt-min-t-after-init S   delay Stage A bootstrap by S sec after VIO init (default 0)\n"
               "  --gps-alt-max-res M   Innovation rejection gate (m): skip updates where |res| > M (1e9=off, try 30)\n"
               "  --gps-alt-guard-dxy M Reject update if predicted |dXY| > M (0=off, try 0.5)\n"
               "  --gps-alt-guard-kxy R Reject if |K_xy|/|K_pz| > R (0=off, try 0.3)\n"
               "  --gps-alt-guard-dbias V Reject if predicted |dbias| > V (0=off)\n"
               "  --height-mode MODE     High-level GPS-Z fusion mode: classic|nasa-lear|standard|bounded\n"
               "                         classic = legacy_guarded GPS-Z guard path used by current baseline\n"
               "  --gps-alt-coupled-mode MODE  Low-level alias: legacy_guarded, full|standard, bounded|scaled_joseph, nasa_lear\n"
               "  --gps-alt-coupled-diag PATH  per-update full-gain/covariance diagnostic CSV\n"
               "  --gps-alt-residual-soft-limit M  uniform residual-based gain bound (0=off)\n"
               "  --gps-alt-max-delta-xy M      bounded-mode single-update XY norm limit\n"
               "  --gps-alt-max-delta-z M       bounded-mode single-update |dz| limit\n"
               "  --gps-alt-max-delta-velocity MPS  bounded-mode velocity increment norm limit\n"
               "  --gps-alt-max-delta-attitude-deg DEG  bounded-mode attitude error norm limit\n"
               "  --gps-alt-max-delta-accel-bias V  bounded-mode accel-bias increment norm limit\n"
               "  --gps-alt-max-delta-gyro-bias V   bounded-mode gyro-bias increment norm limit\n"
               "  --gps-alt-cov-psd-check-interval S full LDLT check interval; <=0 checks every update\n"
               "  --gps-alt-nasa-beta B         NASA/Lear underweight coefficient (nasa_lear mode; 0=off)\n"
               "  --gps-alt-nasa-q-threshold Q  NASA/Lear Orion trigger on H'PH in m^2 (nasa_lear mode)\n"
               "  --gps-alt-arch-g      Architecture G: augment state with h_offset Vec(1) bias.\n"
               "                            Measurement: GPS_z = p_z + h_offset. Standard EKF update.\n"
               "                            Requires --gps-alt-update. Mutually exclusive with --gps-alt-zonly.\n"
               "  --gps-alt-zonly       Legacy Z-only: only p_z state correction; full cov updated.\n"
               "  --gps-alt-joseph-update  PX4-style masked Joseph update (Gate 2):\n"
               "                            only p_z state DOF and p_z row/col of P updated.\n"
               "                            All guard flags still apply. Takes priority over --gps-alt-zonly.\n"
               "  --gps-alt-ground-plane   Pseudo-rangefinder mode: treat GPS altitude as\n"
               "                            slant range to a local flat ground plane.\n"
               "                            Bootstraps z_ground on first GPS sample.\n"
               "  --gplane-feat            Stage B: ground-plane feature update (requires --gps-alt-ground-plane)\n"
               "  --gplane-feat-sigma-px V    pixel noise (default 3.0)\n"
               "  --gplane-feat-max N         max features per update (default 5)\n"
               "  --gplane-feat-center-frac V central image fraction to keep (default 0.8)\n"
               "  --gplane-feat-min-cos-tilt V skip when |cos_tilt|<V (default 0.85)\n"
               "  --gplane-feat-max-res-px V  reject feature if predicted residual>V (default 5.0)\n"
               "  --gplane-feat-v1          Stage B v1 (two-clone H, dry-run by default)\n"
               "  --gplane-feat-v1-update   Run v1 in UPDATE mode (refuses features whose FD check fails)\n"
               "  --gplane-feat-v1-fd-step-rot V  rotation FD step (default 1e-4)\n"
               "  --gplane-feat-v1-fd-step-pos V  position FD step (default 1e-2)\n"
               "  --gplane-feat-v1-fd-rel-tol-rot V   rotation rel_err pass threshold (default 1e-3)\n"
               "  --gplane-feat-v1-fd-rel-tol-pos V   position rel_err pass threshold (default 3e-3)\n"
               "  --gplane-feat-v1-fd-max-abs-rel-tol V  max-abs per-entry rel threshold (default 1e-2)\n"
               "  --gplane-feat-exclude-used-from-msckf {0|1}  default 1 in v1 UPDATE\n"
               "  --gplane-feat-v1-fd-dump N    dump full H_analytic/H_numeric/H_diff matrices\n"
               "                                + intermediate geometry + step-size sweep for first N features\n"
               "  --gps-cutoff-time T   Stop feeding GPS after t_cam > T (hold-out test)\n"
               "  --gps-feed-every F    Feed 1/F of GPS samples (e.g. 0.2 = 1-in-5, validation set)\n"
               "  --gps-time-offset SEC Override yaml gps_time_offset (sec). Adds to gps timestamps.\n"
               "                         For jc82 18r.bag: ~+36.19s (physics) or +39s (empirical opt)\n"
               "  --init-from-fc PATH   Init-only FC-assisted VIO state CSV; bypass DynamicInitializer.\n"
               "                        Requires --start-time 930 for this D455 flight-test.\n"
               "  --init-vel-sigma MPS  Initial velocity stddev (default 5.0; test 2/5/10)\n"
               "  --init-att-sigma-deg DEG  Initial attitude stddev (default 5.0)\n"
               "  --init-pos-sigma M    Initial position stddev (default 100.0)\n"
               "  --init-bg-sigma RPS   Initial gyro-bias stddev (default 0.05)\n"
               "  --init-ba-sigma MPS2  Initial accel-bias stddev (default 1.0)\n"
               "  --gt PATH             ASL 17-col ground truth CSV\n"
               "  --output PATH         Output TUM trajectory (default: traj_ros_free.txt)\n"
               "  --video PATH          Record dashboard to MP4\n"
               "  --video-cam PATH      Record camera-only (cam0/cam1 w/ optical-flow tracks) to MP4\n"
               "  --video-fps N         Video FPS (default 20)\n"
               "  --align-seconds X     Seconds of data to collect before SE3 align (default 8)\n"
               "  --no-display          Do not create a window (useful headless)\n"
               "  --dash-title TITLE    Window title (default: 'OpenVINS ROS-free Dashboard')\n"
               "  --dash-every N        Refresh dashboard every N camera frames (default 1)\n"
               "  --camera-frame-stride N  Feed only 1 of every N camera frames to VIO before tracker (default 1)\n"
               "  --adaptive-stride-shadow  Compute/log the causal height+turn policy; keep fixed camera stride\n"
               "  --adaptive-stride         Apply the same causal policy to camera input (requires fixed stride 1)\n"
               "  --adaptive-stride-log PATH  Per-raw-frame policy CSV (default: OUTPUT.adaptive_stride.csv)\n"
               "  --camera-frame-adaptive  Online geometry-gated input camera sampling (default off)\n"
               "  --camera-frame-adaptive-target-ratio R  target translation/depth ratio (default 0.016565)\n"
               "  --camera-frame-adaptive-min-dt S        full-rate dead-zone below desired dt (default 0.11)\n"
               "  --camera-frame-adaptive-max-dt S        maximum accepted-frame spacing (default 0.18)\n"
               "  --camera-frame-adaptive-min-depth M     full-rate dead-zone below median SLAM depth (default 0)\n"
               "  --camera-frame-adaptive-min-tracks N    immediate feed if last tracks fall below N (default 240)\n"
               "  --visual-update-stride N Track every fed frame, but run MSCKF/SLAM visual updates every Nth frame (default 1)\n"
               "  --visual-update-adaptive Full-rate tracking with adaptive visual keyframe observations\n"
               "  --visual-update-adaptive-target-flow PX  accumulated mean optical flow trigger (default 6)\n"
               "  --visual-update-adaptive-min-dt S        minimum visual keyframe spacing (default 0.06)\n"
               "  --visual-update-adaptive-max-dt S        maximum visual keyframe spacing (default 0.18)\n"
               "  --visual-update-adaptive-min-tracks N    immediate update if tracks fall below N (default 240)\n"
               "  --cam-subsample N     Legacy alias for --camera-frame-stride\n"
               "  --camera-stride-audit PATH  Write one-row CSV proving camera stride counts/timestamps\n"
               "  --use-ground-parallel-warp  Warp previous image by gravity rotation before KLT (removes rotational component)\n"
               "  --viz-fast            Fast dashboard mode: decimate curves/trajectories,\n"
               "                        cap SLAM/MSCKF features at 200, skip expensive resize.\n"
               "                        Keeps all panels. Reduces render time ~5-10x.\n"
               "  --verbose             Print per-frame timing\n"
               "  --yaw-mode MODE       High-level visual consistency mode: baseline|fej|oc|oc-fej|none\n"
               "                         baseline = FEJ Jacobians + current-gauge OC, preserving current best recipe\n"
               "                         oc-fej = FEJ Jacobians + FEJ-gauge OC projection\n"
               "  --no-vio-yaw-update   Alias for --vio-yaw-update-scale 0.0\n"
               "  --vio-yaw-update-mode M   original|per_block_scale|global_yaw_oc_projection|current_only_scale|hard_gyro_yaw|a_strict_yaw_dx0\n"
               "                            visual_yaw_schmidt_current_gauge|visual_yaw_schmidt_fej_gauge\n"
               "  --vio-yaw-update-scale S  Scale visual yaw correction for *_scale modes (1=orig, 0=off)\n"
               "  --vio-global-yaw-oc-alpha A  H-projection alpha for global_yaw_oc_projection (0=orig, 1=full)\n"
               "  --vio-vins-nullspace-eig-thresh T  Eigenvalue threshold for vins_numeric_nullspace (relative, default 1e-6)\n"
               "  --vio-yaw-control-start-after-init S  Delay requested visual yaw control until S seconds after init\n"
               "  --visual-bgz-update-scale S  Scale bg_z row of visual K_eff (1.0=normal, 0.0=freeze bg_z from visual)\n"
               "  --curl-correction-rate R     Camera-fixed optical-axis curl correction (deg/s, + = CCW in image)\n"
               "  --vio-yaw-switch-mode M   After --vio-yaw-switch-time, switch to this mode (staged B→A)\n"
               "  --vio-yaw-switch-time T   Absolute timestamp (s) to switch yaw mode (D1–D4 variants)\n"
               "  --vio-yaw-switch-alpha A  Alpha for the switched-to mode (e.g. 1.0 for global_yaw_oc_projection)\n"
               "  --vio-yaw-diag PATH   Write per-MSCKF/SLAM yaw update CSV\n"
               "  --visual-obs-diag PATH  Write VisualObservabilityPolicy diagnostics CSV\n"
               "  --mech-diag-t0 T      Mechanism diagnostic window start time (camera seconds)\n"
               "  --mech-diag-t1 T      Mechanism diagnostic window end time (camera seconds)\n"
               "  --visual-flow-curl-diag PATH  Write per-frame feature spatial/flow/curl CSV\n"
               "  --yaw-update-mechanism-diag PATH  Write yaw update vs reference-course CSV\n"
               "  --imu-propagation-yaw-diag PATH   Write IMU propagation yaw CSV\n"
               "  --slam-feature-yaw-contrib-diag PATH  Write per-feature SLAM residual/yaw contribution CSV\n"
               "  --visual-feature-residual-diag PATH  Write per-feature MSCKF/SLAM residual chi2 CSV\n"
               "  --visual-residual-frame-summary PATH Write per-update visual residual summary CSV\n"
               "  --slam-yaw-contrib-cap-enable  Enable roll-gated frame-local top-k SLAM yaw contribution cap\n"
               "  --slam-yaw-contrib-cap-t0 T    Cap window start time (camera seconds)\n"
               "  --slam-yaw-contrib-cap-t1 T    Cap window end time (camera seconds)\n"
               "  --slam-yaw-contrib-cap-roll-deg DEG  Apply only when |roll| <= DEG\n"
               "  --slam-yaw-contrib-cap-yawrate-degps DEG_S  Apply only when |reference course yaw rate| <= DEG_S\n"
               "  --slam-yaw-contrib-cap-topk N  Cap top-N same-sign per-feature yaw contributors\n"
                "  --slam-yaw-contrib-cap-ratio R Soft cap information weight ratio (0<R<1)\n"
                "  --slam-yaw-contrib-cap-mode M  Cap mode (default soft_scale)\n"
                "  --slam-yaw-contrib-cap-diag PATH  Write per-SLAM-update cap diagnostic CSV\n"
                "  --slam-info-reduction-enable  Enable SLAM-only accepted residual information reduction\n"
                "  --slam-info-reduction-low-ratio R  chi2_ratio where alpha starts increasing (default 0.2)\n"
                "  --slam-info-reduction-high-ratio R chi2_ratio where alpha reaches max (default 0.5)\n"
                "  --slam-info-reduction-alpha-max A  Maximum covariance inflation alpha (default 4.0)\n"
                "  --slam-info-reduction-diag PATH  Write per-feature SLAM information reduction CSV\n"
                "  --slam-ekf-leverage-diag PATH  Write per-feature regular SLAM EKF leverage/anchor diagnostic CSV\n"
                "  --slam-stacked-ekf-diag PATH  Write per-frame stacked SLAM EKF H/K/P/projection diagnostic CSV\n"
                "  --slam-landmark-metadata-diag PATH  Write accepted SLAM landmark lifecycle/geometry diagnostic CSV\n"
                "  --slam-geometry-lifecycle-refresh  Refresh old far-depth regular SLAM landmarks before update (causal test)\n"
                "  --slam-geometry-refresh-min-depth D  Depth trigger in meters (default 400)\n"
                "  --slam-geometry-refresh-min-age S    Lifetime trigger in seconds (default 3)\n"
                "  --slam-geometry-refresh-min-pose-lm-cov C  Pose-landmark covariance trigger (default 0=off)\n"
                "  --slam-geometry-refresh-min-regular-features N  Keep at least N regular SLAM features after refresh (default 0=off)\n"
                "  --slam-geometry-refresh-require-anchor-change  Also require landmark has anchor-changed\n"
                "  --slam-geometry-refresh-diag PATH   Write geometry lifecycle refresh decision CSV\n"
                "  --state-safety-diag PATH  Write read-only per-frame state/covariance health CSV\n"
                "  --state-safety-eig-every N  Also compute full covariance min eigen every N fed camera frames (0=off)\n"
                "  --slam-update-freeze-window T0 T1 Skip only SLAM update/delayed init in [T0,T1]\n"
               "  --vio-consistency-mode M  Low-level alias for --yaw-mode:\n"
               "    baseline  use_fej=true, current-gauge global-yaw OC alpha=1 (current best recipe)\n"
               "    oc        use_fej=false, current-gauge global-yaw OC alpha=1\n"
               "    fej       use_fej=true, original visual update (no OC)\n"
               "    none      use_fej=false, original visual update (no OC)\n"
               "    oc-fej    use_fej=true, FEJ-gauge global-yaw OC alpha=1\n"
               "    oc-current-fej  use_fej=true, current-gauge global-yaw OC alpha=1\n"
               "  --visual-update-skip-window T0 T1  Skip visual EKF updates in [T0,T1] seconds (ablation)\n"
               "  --visual-update-guard-log PATH      Write per-update guard decision CSV\n"
               "  --visual-update-reject-topn-file PATH  CSV of t or t0,t1 intervals to reject\n"
               "  Modes for --vio-yaw-update-mode (new pre-chi2 OC modes):\n"
               "    global_yaw_oc_fej_prechi2  1-D FEJ yaw OC applied before chi2 gating\n"
               "    visual_4d_oc_fej_prechi2   4-D FEJ (yaw+xyz) OC applied before chi2 gating\n"
               "\n"
               "  --vio-yaw-gauge-mode M   High-level alias (expands to --vio-yaw-update-mode + alpha):\n"
               "    baseline / r1            global_yaw_oc_projection + alpha=1.0  [R1: 216.27m]\n"
               "    original                 original (no gauge protection)\n"
               "    oc_legacy                global_yaw_oc_projection + alpha=1.0  (same as baseline)\n"
               "    oc_prechi2               global_yaw_oc_fej_prechi2  (1-D FEJ, pre-chi2)\n"
               "    oc_4d                    visual_4d_oc_fej_prechi2   (4-D FEJ, pre-chi2)\n"
               "    schmidt                  visual_yaw_schmidt_current_gauge\n"
               "    schmidt_fej              visual_yaw_schmidt_fej_gauge\n"
               "    h_proj                   visual_yaw_h_projection_current  (hard H-space null-space)\n"
               "    no_yaw                   hard_gyro_yaw  (zero visual yaw entirely)\n"
               "    oc_legacy_fej            global_yaw_oc_fej_projection + alpha=1.0  (FEJ-gauge M0, post-chi2)\n"
               "    msckf2                   global_yaw_oc_fej_prechi2  (Li&Mourikis IJRR 2013: FEJ+OC+pre-chi2)\n"
               "    msckf2_pure              original  (pure Li&Mourikis 2013: FEJ only, no OC)\n"
               "    constrained_yaw_nullspace  constrained_yaw_nullspace (rank-1 post-update nᵀδx=0 constraint)\n"
               "    dso                      RETIRED (v1 K-projection failed); redirects to global_yaw_oc_fej_projection\n"
               "    vins_nullspace           RETIRED (S-EVD v1 failed); redirects to original\n"
               "    Note: --vio-yaw-update-mode overrides --vio-yaw-gauge-mode if both specified.\n"
               "\n"
               "\n"
               "Diagnostic logging (all off by default; independent of --verbose):\n"
               "  --diag-print          Print compact [DIAG] block every N frames\n"
               "  --diag-print-every N  Frames between [DIAG] prints (default 30)\n"
               "  --run-name NAME       Label written into CSV and event log\n"
               "  --diag-csv PATH       Per-frame diagnostic CSV for post-flight plotting\n"
               "  --diag-events PATH    Append-only human-readable event log\n";
}

bool parse_args(int argc, char **argv, Args &a) {
  for (int i = 1; i < argc; ++i) {
    std::string s = argv[i];
    auto next = [&](const char *name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(EXIT_FAILURE);
      }
      return argv[++i];
    };
    if (s == "--config") a.config_path = next("--config");
    else if (s == "--dataset") a.dataset_dir = next("--dataset");
    else if (s == "--gps") a.gps_path = next("--gps");
    else if (s == "--gt") a.gt_path = next("--gt");
    else if (s == "--output") a.output_path = next("--output");
    else if (s == "--video") a.video_path = next("--video");
    else if (s == "--video-cam") a.video_cam_path = next("--video-cam");
    else if (s == "--video-fps") a.video_fps = std::atoi(next("--video-fps").c_str());
    else if (s == "--align-seconds") a.align_seconds = std::atof(next("--align-seconds").c_str());
    else if (s == "--start-time") a.start_time = std::atof(next("--start-time").c_str());
    else if (s == "--until-time") a.until_time = std::atof(next("--until-time").c_str());
    else if (s == "--stereo") a.stereo = true;
    else if (s == "--gps-alt-update") a.gps_alt_update = true;
    else if (s == "--gps-alt-sigma") a.gps_alt_sigma = std::atof(next("--gps-alt-sigma").c_str());
    else if (s == "--gps-alt-chi2") a.gps_alt_chi2 = std::atof(next("--gps-alt-chi2").c_str());
    else if (s == "--gps-alt-schmidt") a.gps_alt_schmidt = true;
    else if (s == "--gps-alt-also-vz") a.gps_alt_also_vz = true;
    else if (s == "--gps-alt-range-mode") a.gps_alt_range_mode = true;
    else if (s == "--gps-alt-relative") a.gps_alt_relative = true;
    else if (s == "--gps-alt-min-pzz") a.gps_alt_min_pzz = std::atof(next("--gps-alt-min-pzz").c_str());
    else if (s == "--gps-alt-min-t-after-init") a.gps_alt_min_t_after_init = std::atof(next("--gps-alt-min-t-after-init").c_str());
    else if (s == "--gps-alt-max-res") a.gps_alt_max_res = std::atof(next("--gps-alt-max-res").c_str());
    else if (s == "--gps-alt-guard-dxy") a.gps_alt_guard_dxy = std::atof(next("--gps-alt-guard-dxy").c_str());
    else if (s == "--gps-alt-guard-kxy") a.gps_alt_guard_kxy_ratio = std::atof(next("--gps-alt-guard-kxy").c_str());
    else if (s == "--gps-alt-guard-dbias") a.gps_alt_guard_dbias = std::atof(next("--gps-alt-guard-dbias").c_str());
    else if (s == "--gps-alt-coupled-mode" || s == "--gps-alt-mode" || s == "--height-mode")
      a.gps_alt_coupled_mode = next(s.c_str());
    else if (s == "--gps-alt-coupled-diag") a.gps_alt_coupled_diag_path = next("--gps-alt-coupled-diag");
    else if (s == "--gps-alt-residual-soft-limit") a.gps_alt_residual_soft_limit = std::atof(next("--gps-alt-residual-soft-limit").c_str());
    else if (s == "--gps-alt-max-delta-xy") a.gps_alt_max_delta_xy = std::atof(next("--gps-alt-max-delta-xy").c_str());
    else if (s == "--gps-alt-max-delta-z") a.gps_alt_max_delta_z = std::atof(next("--gps-alt-max-delta-z").c_str());
    else if (s == "--gps-alt-max-delta-velocity") a.gps_alt_max_delta_velocity = std::atof(next("--gps-alt-max-delta-velocity").c_str());
    else if (s == "--gps-alt-max-delta-attitude-deg") a.gps_alt_max_delta_attitude_deg = std::atof(next("--gps-alt-max-delta-attitude-deg").c_str());
    else if (s == "--gps-alt-max-delta-accel-bias") a.gps_alt_max_delta_accel_bias = std::atof(next("--gps-alt-max-delta-accel-bias").c_str());
    else if (s == "--gps-alt-max-delta-gyro-bias") a.gps_alt_max_delta_gyro_bias = std::atof(next("--gps-alt-max-delta-gyro-bias").c_str());
    else if (s == "--gps-alt-cov-psd-check-interval") a.gps_alt_cov_psd_check_interval = std::atof(next("--gps-alt-cov-psd-check-interval").c_str());
    else if (s == "--gps-alt-nasa-beta") a.gps_alt_nasa_beta = std::atof(next("--gps-alt-nasa-beta").c_str());
    else if (s == "--gps-alt-nasa-q-threshold") a.gps_alt_nasa_q_threshold = std::atof(next("--gps-alt-nasa-q-threshold").c_str());
    else if (s == "--gps-alt-arch-g") a.gps_alt_arch_g = true;
    else if (s == "--gps-alt-zonly") a.gps_alt_zonly = true;
    else if (s == "--gps-alt-joseph-update") a.gps_alt_joseph = true;
    else if (s == "--gps-alt-ground-plane") a.gps_alt_ground_plane = true;
    else if (s == "--gplane-feat") a.gplane_feat_enable = true;
    else if (s == "--gplane-feat-sigma-px") a.gplane_feat_sigma_px = std::atof(next("--gplane-feat-sigma-px").c_str());
    else if (s == "--gplane-feat-max") a.gplane_feat_max_features = std::atoi(next("--gplane-feat-max").c_str());
    else if (s == "--gplane-feat-center-frac") a.gplane_feat_center_frac = std::atof(next("--gplane-feat-center-frac").c_str());
    else if (s == "--gplane-feat-min-cos-tilt") a.gplane_feat_min_cos_tilt = std::atof(next("--gplane-feat-min-cos-tilt").c_str());
    else if (s == "--gplane-feat-max-res-px") a.gplane_feat_max_res_px = std::atof(next("--gplane-feat-max-res-px").c_str());
    else if (s == "--gplane-feat-v1") a.gplane_feat_v1_enable = true;
    else if (s == "--gplane-feat-v1-update") { a.gplane_feat_v1_enable = true; a.gplane_feat_v1_update = true; }
    else if (s == "--gplane-feat-v1-fd-step-rot") a.gplane_feat_v1_fd_step_rot = std::atof(next("--gplane-feat-v1-fd-step-rot").c_str());
    else if (s == "--gplane-feat-v1-fd-step-pos") a.gplane_feat_v1_fd_step_pos = std::atof(next("--gplane-feat-v1-fd-step-pos").c_str());
    else if (s == "--gplane-feat-v1-fd-rel-tol-rot") a.gplane_feat_v1_fd_rel_tol_rot = std::atof(next("--gplane-feat-v1-fd-rel-tol-rot").c_str());
    else if (s == "--gplane-feat-v1-fd-rel-tol-pos") a.gplane_feat_v1_fd_rel_tol_pos = std::atof(next("--gplane-feat-v1-fd-rel-tol-pos").c_str());
    else if (s == "--gplane-feat-v1-fd-max-abs-rel-tol") a.gplane_feat_v1_fd_max_abs_rel_tol = std::atof(next("--gplane-feat-v1-fd-max-abs-rel-tol").c_str());
    else if (s == "--gplane-feat-exclude-used-from-msckf") a.gplane_feat_exclude_used_from_msckf = (std::atoi(next("--gplane-feat-exclude-used-from-msckf").c_str()) != 0);
    else if (s == "--gplane-feat-v1-fd-dump") a.gplane_feat_v1_fd_dump = std::atoi(next("--gplane-feat-v1-fd-dump").c_str());
    else if (s == "--gps-cutoff-time") a.gps_cutoff_time = std::atof(next("--gps-cutoff-time").c_str());
    else if (s == "--gps-feed-every") a.gps_feed_every = std::atof(next("--gps-feed-every").c_str());
    else if (s == "--gps-time-offset") a.gps_time_offset_cli = std::atof(next("--gps-time-offset").c_str());
    else if (s == "--init-from-fc" || s == "--init-state-csv") a.init_from_fc_path = next("--init-from-fc");
    else if (s == "--init-vel-sigma") a.init_vel_sigma = std::atof(next("--init-vel-sigma").c_str());
    else if (s == "--init-att-sigma-deg") a.init_att_sigma_deg = std::atof(next("--init-att-sigma-deg").c_str());
    else if (s == "--init-pos-sigma") a.init_pos_sigma = std::atof(next("--init-pos-sigma").c_str());
    else if (s == "--init-bg-sigma") a.init_bg_sigma = std::atof(next("--init-bg-sigma").c_str());
    else if (s == "--init-ba-sigma") a.init_ba_sigma = std::atof(next("--init-ba-sigma").c_str());
    else if (s == "--no-display") a.show = false;
    else if (s == "--dash-title") a.dash_title = next("--dash-title");
    else if (s == "--dash-every") a.dash_every = std::atoi(next("--dash-every").c_str());
    else if (s == "--cam-subsample" || s == "--camera-frame-stride")
      a.cam_subsample = std::max(1, std::atoi(next(s.c_str()).c_str()));
    else if (s == "--adaptive-stride-shadow")
      a.adaptive_stride_shadow = true;
    else if (s == "--adaptive-stride")
      a.adaptive_stride = true;
    else if (s == "--adaptive-stride-log")
      a.adaptive_stride_log_path = next("--adaptive-stride-log");
    else if (s == "--camera-frame-adaptive")
      a.camera_frame_adaptive = true;
    else if (s == "--camera-frame-adaptive-target-ratio")
      a.camera_frame_adaptive_target_ratio = std::atof(next("--camera-frame-adaptive-target-ratio").c_str());
    else if (s == "--camera-frame-adaptive-min-dt")
      a.camera_frame_adaptive_min_dt = std::atof(next("--camera-frame-adaptive-min-dt").c_str());
    else if (s == "--camera-frame-adaptive-max-dt")
      a.camera_frame_adaptive_max_dt = std::atof(next("--camera-frame-adaptive-max-dt").c_str());
    else if (s == "--camera-frame-adaptive-min-depth")
      a.camera_frame_adaptive_min_depth = std::atof(next("--camera-frame-adaptive-min-depth").c_str());
    else if (s == "--camera-frame-adaptive-min-tracks")
      a.camera_frame_adaptive_min_tracks = std::atoi(next("--camera-frame-adaptive-min-tracks").c_str());
    else if (s == "--visual-update-stride")
      a.visual_update_stride = std::max(1, std::atoi(next("--visual-update-stride").c_str()));
    else if (s == "--visual-update-adaptive")
      a.visual_update_adaptive = true;
    else if (s == "--visual-update-adaptive-target-flow")
      a.visual_update_adaptive_target_flow_px = std::atof(next("--visual-update-adaptive-target-flow").c_str());
    else if (s == "--visual-update-adaptive-min-dt")
      a.visual_update_adaptive_min_dt = std::atof(next("--visual-update-adaptive-min-dt").c_str());
    else if (s == "--visual-update-adaptive-max-dt")
      a.visual_update_adaptive_max_dt = std::atof(next("--visual-update-adaptive-max-dt").c_str());
    else if (s == "--visual-update-adaptive-min-tracks")
      a.visual_update_adaptive_min_tracks = std::atoi(next("--visual-update-adaptive-min-tracks").c_str());
    else if (s == "--camera-stride-audit" || s == "--stride-audit")
      a.camera_stride_audit_path = next(s.c_str());
    else if (s == "--use-ground-parallel-warp") a.use_ground_parallel_warp = true;
    else if (s == "--viz-fast") a.viz_fast = true;
    else if (s == "--verbose") a.verbose_timing = true;
    else if (s == "--no-vio-yaw-update") a.no_vio_yaw_update = true;
    else if (s == "--vio-yaw-update-mode") a.vio_yaw_update_mode = next("--vio-yaw-update-mode");
    else if (s == "--vio-yaw-update-scale") a.vio_yaw_update_scale = std::atof(next("--vio-yaw-update-scale").c_str());
    else if (s == "--vio-global-yaw-oc-alpha") a.vio_global_yaw_oc_alpha = std::atof(next("--vio-global-yaw-oc-alpha").c_str());
    else if (s == "--vio-global-yaw-schmidt-alpha") a.vio_global_yaw_oc_alpha = std::atof(next("--vio-global-yaw-schmidt-alpha").c_str());
    else if (s == "--vio-vins-nullspace-eig-thresh") a.vins_cfg.eig_thresh = std::atof(next("--vio-vins-nullspace-eig-thresh").c_str());
    else if (s == "--vio-yaw-control-start-after-init") a.vio_yaw_control_start_after_init = std::atof(next("--vio-yaw-control-start-after-init").c_str());
    else if (s == "--visual-bgz-update-scale") a.visual_bgz_update_scale = std::atof(next("--visual-bgz-update-scale").c_str());
    else if (s == "--curl-correction-rate") a.curl_correction_rate_degps = std::atof(next("--curl-correction-rate").c_str());
    else if (s == "--vio-yaw-switch-mode") a.vio_yaw_switch_mode = next("--vio-yaw-switch-mode");
    else if (s == "--vio-yaw-switch-time") a.vio_yaw_switch_time = std::atof(next("--vio-yaw-switch-time").c_str());
    else if (s == "--vio-yaw-switch-alpha") a.vio_yaw_switch_alpha = std::atof(next("--vio-yaw-switch-alpha").c_str());
    else if (s == "--vio-consistency-mode" || s == "--yaw-mode") a.vio_consistency_mode = next(s.c_str());
    else if (s == "--vio-yaw-gauge-mode") a.vio_yaw_gauge_mode = next("--vio-yaw-gauge-mode");
    else if (s == "--vio-yaw-diag") a.vio_yaw_diag_path = next("--vio-yaw-diag");
    else if (s == "--visual-obs-diag") a.visual_obs_diag_path = next("--visual-obs-diag");
    else if (s == "--mech-diag-t0") a.mech_diag_t0 = std::atof(next("--mech-diag-t0").c_str());
    else if (s == "--mech-diag-t1") a.mech_diag_t1 = std::atof(next("--mech-diag-t1").c_str());
    else if (s == "--visual-flow-curl-diag") a.visual_flow_curl_diag_path = next("--visual-flow-curl-diag");
    else if (s == "--yaw-update-mechanism-diag") a.yaw_update_mechanism_diag_path = next("--yaw-update-mechanism-diag");
    else if (s == "--imu-propagation-yaw-diag") a.imu_propagation_yaw_diag_path = next("--imu-propagation-yaw-diag");
    else if (s == "--slam-feature-yaw-contrib-diag") a.slam_feature_yaw_contrib_diag_path = next("--slam-feature-yaw-contrib-diag");
    else if (s == "--visual-feature-residual-diag") a.visual_feature_residual_diag_path = next("--visual-feature-residual-diag");
    else if (s == "--visual-residual-frame-summary") a.visual_residual_frame_summary_path = next("--visual-residual-frame-summary");
    else if (s == "--slam-yaw-contrib-cap-enable") a.slam_yaw_contrib_cap_enable = true;
    else if (s == "--slam-yaw-contrib-cap-t0") a.slam_yaw_contrib_cap_t0 = std::atof(next("--slam-yaw-contrib-cap-t0").c_str());
    else if (s == "--slam-yaw-contrib-cap-t1") a.slam_yaw_contrib_cap_t1 = std::atof(next("--slam-yaw-contrib-cap-t1").c_str());
    else if (s == "--slam-yaw-contrib-cap-roll-deg") a.slam_yaw_contrib_cap_roll_deg = std::atof(next("--slam-yaw-contrib-cap-roll-deg").c_str());
    else if (s == "--slam-yaw-contrib-cap-yawrate-degps") a.slam_yaw_contrib_cap_yawrate_degps = std::atof(next("--slam-yaw-contrib-cap-yawrate-degps").c_str());
    else if (s == "--slam-yaw-contrib-cap-topk") a.slam_yaw_contrib_cap_topk = std::atoi(next("--slam-yaw-contrib-cap-topk").c_str());
    else if (s == "--slam-yaw-contrib-cap-ratio") a.slam_yaw_contrib_cap_ratio = std::atof(next("--slam-yaw-contrib-cap-ratio").c_str());
    else if (s == "--slam-yaw-contrib-cap-mode") a.slam_yaw_contrib_cap_mode = next("--slam-yaw-contrib-cap-mode");
    else if (s == "--slam-yaw-contrib-cap-diag") a.slam_yaw_contrib_cap_diag_path = next("--slam-yaw-contrib-cap-diag");
    else if (s == "--slam-info-reduction-enable") a.slam_info_reduction_enable = true;
    else if (s == "--slam-info-reduction-low-ratio") a.slam_info_reduction_low_ratio = std::atof(next("--slam-info-reduction-low-ratio").c_str());
    else if (s == "--slam-info-reduction-high-ratio") a.slam_info_reduction_high_ratio = std::atof(next("--slam-info-reduction-high-ratio").c_str());
    else if (s == "--slam-info-reduction-alpha-max") a.slam_info_reduction_alpha_max = std::atof(next("--slam-info-reduction-alpha-max").c_str());
    else if (s == "--slam-info-reduction-diag") a.slam_info_reduction_diag_path = next("--slam-info-reduction-diag");
    else if (s == "--slam-ekf-leverage-diag") a.slam_ekf_leverage_diag_path = next("--slam-ekf-leverage-diag");
    else if (s == "--slam-stacked-ekf-diag") a.slam_stacked_ekf_diag_path = next("--slam-stacked-ekf-diag");
    else if (s == "--slam-landmark-metadata-diag") a.slam_landmark_metadata_diag_path = next("--slam-landmark-metadata-diag");
    else if (s == "--slam-geometry-lifecycle-refresh") a.slam_geometry_lifecycle_refresh_enable = true;
    else if (s == "--slam-geometry-refresh-min-depth") a.slam_geometry_refresh_min_depth = std::atof(next("--slam-geometry-refresh-min-depth").c_str());
    else if (s == "--slam-geometry-refresh-min-age") a.slam_geometry_refresh_min_age = std::atof(next("--slam-geometry-refresh-min-age").c_str());
    else if (s == "--slam-geometry-refresh-min-pose-lm-cov") a.slam_geometry_refresh_min_pose_lm_cov = std::atof(next("--slam-geometry-refresh-min-pose-lm-cov").c_str());
    else if (s == "--slam-geometry-refresh-min-regular-features") a.slam_geometry_refresh_min_regular_features = std::atoi(next("--slam-geometry-refresh-min-regular-features").c_str());
    else if (s == "--slam-geometry-refresh-require-anchor-change") a.slam_geometry_refresh_require_anchor_change = true;
    else if (s == "--slam-geometry-refresh-diag") a.slam_geometry_refresh_diag_path = next("--slam-geometry-refresh-diag");
    else if (s == "--state-safety-diag") a.state_safety_diag_path = next("--state-safety-diag");
    else if (s == "--state-safety-eig-every") a.state_safety_eig_every = std::atoi(next("--state-safety-eig-every").c_str());
    else if (s == "--slam-update-freeze-window") {
      a.slam_freeze_t0 = std::atof(next("--slam-update-freeze-window T0").c_str());
      a.slam_freeze_t1 = std::atof(next("--slam-update-freeze-window T1").c_str());
    }
    else if (s == "--schmidt-yaw-diag") a.schmidt_yaw_diag_path = next("--schmidt-yaw-diag");
    else if (s == "--visual-update-skip-window") {
      a.visual_skip_t0 = std::atof(next("--visual-update-skip-window T0").c_str());
      a.visual_skip_t1 = std::atof(next("--visual-update-skip-window T1").c_str());
    }
    else if (s == "--visual-update-guard-log") a.visual_guard_log_path = next("--visual-update-guard-log");
    else if (s == "--visual-update-reject-topn-file") a.visual_reject_file_path = next("--visual-update-reject-topn-file");
    // Guarded-B thresholds (commit 2)
    else if (s == "--visual-guard-gauge-frac") {
      a.guard_cfg.gauge_frac_mild   = std::atof(next("--visual-guard-gauge-frac").c_str());
      a.guard_cfg.gauge_frac_severe = a.guard_cfg.gauge_frac_mild + 0.10;
    }
    else if (s == "--visual-guard-norm-dx") {
      a.guard_cfg.norm_dx_mild   = std::atof(next("--visual-guard-norm-dx").c_str());
      a.guard_cfg.norm_dx_severe = a.guard_cfg.norm_dx_mild * 1.5;
    }
    else if (s == "--visual-guard-pas")           a.guard_cfg.pas_mild          = std::atof(next("--visual-guard-pas").c_str());
    else if (s == "--visual-guard-r-scale-mild")  a.guard_cfg.r_scale_mild      = std::atof(next("--visual-guard-r-scale-mild").c_str());
    else if (s == "--visual-guard-r-scale-severe")a.guard_cfg.r_scale_severe    = std::atof(next("--visual-guard-r-scale-severe").c_str());
    else if (s == "--visual-guard-burst-count")   a.guard_cfg.burst_count       = std::atoi(next("--visual-guard-burst-count").c_str());
    else if (s == "--visual-guard-burst-window")  a.guard_cfg.burst_window_s    = std::atof(next("--visual-guard-burst-window").c_str());
    else if (s == "--visual-guard-reject-severe") a.guard_cfg.reject_on_severe  = true;
    else if (s == "--visual-guard-diag-log")      a.guard_diag_log_path         = next("--visual-guard-diag-log");
    else if (s == "--cam-toff") a.cam_toff_override = std::atof(next("--cam-toff").c_str());
    else if (s == "--diag-chi2-trigger") a.diag_chi2_trigger = std::atoi(next("--diag-chi2-trigger").c_str());
    else if (s == "--diag-window") a.diag_window = std::atoi(next("--diag-window").c_str());
    else if (s == "--diag-print") a.diag_print = true;
    else if (s == "--diag-print-every") a.diag_print_every = std::atoi(next("--diag-print-every").c_str());
    else if (s == "--run-name") a.run_name = next("--run-name");
    else if (s == "--diag-csv") a.diag_csv_path = next("--diag-csv");
    else if (s == "--diag-events") a.diag_events_path = next("--diag-events");
    else if (s == "-h" || s == "--help") { print_help(); return false; }
    else {
      std::cerr << "unknown arg: " << s << "\n";
      print_help();
      return false;
    }
  }
  if (a.config_path.empty() || a.dataset_dir.empty()) {
    print_help();
    return false;
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, on_sigint);

  Args args;
  if (!parse_args(argc, argv, args))
    return EXIT_FAILURE;
  if (args.adaptive_stride && args.adaptive_stride_shadow) {
    PRINT_ERROR(RED "[adaptive-stride] active and shadow modes are mutually exclusive\n" RESET);
    return EXIT_FAILURE;
  }
  if ((args.adaptive_stride || args.adaptive_stride_shadow) && args.camera_frame_adaptive) {
    PRINT_ERROR(RED "[adaptive-stride] cannot be combined with --camera-frame-adaptive\n" RESET);
    return EXIT_FAILURE;
  }
  if (args.adaptive_stride && args.cam_subsample != 1) {
    PRINT_ERROR(RED "[adaptive-stride] active mode requires --camera-frame-stride 1; "
                    "shadow mode is the fixed-stride comparator\n" RESET);
    return EXIT_FAILURE;
  }
  if ((args.adaptive_stride || args.adaptive_stride_shadow) && args.adaptive_stride_log_path.empty())
    args.adaptive_stride_log_path = args.output_path + ".adaptive_stride.csv";

  // -------------------- load config --------------------
  auto parser = std::make_shared<ov_core::YamlParser>(args.config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  if ((args.adaptive_stride || args.adaptive_stride_shadow) &&
      (params.state_options.do_calib_camera_pose ||
       params.state_options.do_calib_camera_intrinsics ||
       params.state_options.do_calib_camera_timeoffset)) {
    PRINT_ERROR(RED "[adaptive-stride] formal validation requires locked camera extrinsics, "
                    "intrinsics, and time offset (got ext=%d intr=%d toff=%d)\n" RESET,
                params.state_options.do_calib_camera_pose ? 1 : 0,
                params.state_options.do_calib_camera_intrinsics ? 1 : 0,
                params.state_options.do_calib_camera_timeoffset ? 1 : 0);
    return EXIT_FAILURE;
  }
  // [中文] 离线回放: 禁用 VioManager 内部的异步队列, 我们自己严格按时间序喂
  params.use_multi_threading_subs = false;

  // [中文] CLI --gps-time-offset 覆盖 yaml 里 gps_time_offset.
  // 用法示例: --gps-time-offset 36.19 (jc82 18r.bag 锚点修正, 物理推导值)
  if (!std::isnan(args.gps_time_offset_cli)) {
    params.gps_time_offset = args.gps_time_offset_cli;
    PRINT_INFO(CYAN "[ros-free] CLI override: gps_time_offset=%+.3fs\n" RESET,
               params.gps_time_offset);
  }

  // [中文] CLI --cam-toff: override timeshift_cam_imu and disable online calibration.
  // Useful for isolated time-offset diagnostic sweeps.
  if (!std::isnan(args.cam_toff_override)) {
    params.calib_camimu_dt = args.cam_toff_override;
    params.state_options.do_calib_camera_timeoffset = false;
    PRINT_INFO(CYAN "[ros-free] CLI override: cam_toff=%.6fs (online cam-IMU time-cal DISABLED)\n" RESET,
               args.cam_toff_override);
  }

  // Historical yaw aliases only selected the visual update path and left the
  // YAML use_fej value untouched. This atomic mode makes OC-only, FEJ-only,
  // neither, and OC+FEJ explicit and rejects ambiguous legacy flag mixtures.
  if (!args.vio_consistency_mode.empty()) {
    std::transform(args.vio_consistency_mode.begin(), args.vio_consistency_mode.end(),
                   args.vio_consistency_mode.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    std::replace(args.vio_consistency_mode.begin(), args.vio_consistency_mode.end(), '_', '-');
    const bool has_conflicting_yaw_cli =
        !args.vio_yaw_gauge_mode.empty() ||
        !args.vio_yaw_update_mode.empty() ||
        !std::isnan(args.vio_global_yaw_oc_alpha) ||
        args.no_vio_yaw_update;
    if (has_conflicting_yaw_cli) {
      PRINT_ERROR("[ros-free] --vio-consistency-mode cannot be combined with "
                  "visual yaw mode/alpha flags.\n");
      return EXIT_FAILURE;
    }

    if (args.vio_consistency_mode == "baseline" ||
        args.vio_consistency_mode == "oc-current-fej") {
      params.state_options.do_fej = true;
      args.vio_yaw_update_mode = "global_yaw_oc_projection";
      args.vio_global_yaw_oc_alpha = 1.0;
    } else if (args.vio_consistency_mode == "oc") {
      params.state_options.do_fej = false;
      args.vio_yaw_update_mode = "global_yaw_oc_projection";
      args.vio_global_yaw_oc_alpha = 1.0;
    } else if (args.vio_consistency_mode == "oc-fej") {
      params.state_options.do_fej = true;
      args.vio_yaw_update_mode = "global_yaw_oc_fej_projection";
      args.vio_global_yaw_oc_alpha = 1.0;
    } else if (args.vio_consistency_mode == "fej") {
      params.state_options.do_fej = true;
      args.vio_yaw_update_mode = "original";
    } else if (args.vio_consistency_mode == "none") {
      params.state_options.do_fej = false;
      args.vio_yaw_update_mode = "original";
    } else {
      PRINT_ERROR("[ros-free] invalid --vio-consistency-mode=%s; expected baseline, oc, fej, none, oc-fej, or oc-current-fej.\n",
                  args.vio_consistency_mode.c_str());
      return EXIT_FAILURE;
    }

    PRINT_INFO(CYAN "[ros-free] resolved consistency_mode=%s use_fej=%d "
                    "vio_yaw_update_mode=%s alpha=%s\n" RESET,
               args.vio_consistency_mode.c_str(),
               params.state_options.do_fej ? 1 : 0,
               args.vio_yaw_update_mode.c_str(),
               std::isnan(args.vio_global_yaw_oc_alpha) ? "n/a" : "1.0");
  }

  // --vio-yaw-gauge-mode: high-level alias that expands to update_mode + alpha.
  // Applied BEFORE individual --vio-yaw-update-mode overrides, so the explicit
  // flag always wins if both are given.
  if (!args.vio_yaw_gauge_mode.empty()) {
    const std::string &gm = args.vio_yaw_gauge_mode;
    std::string mapped_mode; double mapped_alpha = std::numeric_limits<double>::quiet_NaN();
    // ── Descriptive implementation aliases (use these in scripts and results) ─
    // openvins_fej: OpenVINS original update, FEJ Jacobians, no OC projection
    if (gm == "openvins_fej" || gm == "original") {
      mapped_mode = "original";
    // oc_postchi2_current_gauge: H-space global-yaw OC, current-state gauge, post-chi2
    } else if (gm == "oc_postchi2_current_gauge" || gm == "baseline") {
      mapped_mode = "global_yaw_oc_projection"; mapped_alpha = 1.0;
    // oc_prechi2_fej_gauge: H-space global-yaw OC, FEJ-frozen gauge, pre-chi2
    } else if (gm == "oc_prechi2_fej_gauge" || gm == "oc_prechi2") {
      mapped_mode = "global_yaw_oc_fej_prechi2";
    // oc_4d_prechi2_fej_gauge: 4-DOF OC H-projection, FEJ-frozen gauge, pre-chi2
    } else if (gm == "oc_4d_prechi2_fej_gauge" || gm == "oc_4d") {
      mapped_mode = "visual_4d_oc_fej_prechi2";
    // oc_postchi2_fej_gauge: H-space global-yaw OC, FEJ-frozen gauge, post-chi2 (fly1 Cond3 reference)
    } else if (gm == "oc_postchi2_fej_gauge" || gm == "oc_legacy_fej") {
      mapped_mode = "global_yaw_oc_fej_projection"; mapped_alpha = 1.0;
    // ── Legacy short aliases (accepted, map silently) ────────────────────────
    } else if (gm == "r1" || gm == "oc_legacy") {
      mapped_mode = "global_yaw_oc_projection"; mapped_alpha = 1.0;
    } else if (gm == "fej") {
      mapped_mode = "original";
    // ── Disabled ambiguous labels (error + stop) ────────────────────────────
    } else if (gm == "msckf2" || gm == "msckf2_0" || gm == "msckf2_pure") {
      PRINT_ERROR("[ros-free] DISABLED LABEL: --vio-yaw-gauge-mode=%s\n"
                  "  'msckf2' was used ambiguously and does not uniquely identify an implementation.\n"
                  "  Use a descriptive name:\n"
                  "    openvins_fej          — OpenVINS FEJ, no OC projection\n"
                  "    oc_prechi2_fej_gauge  — H-space OC, FEJ gauge, pre-chi2\n"
                  "    oc_postchi2_current_gauge — H-space OC, current gauge, post-chi2\n",
                  gm.c_str());
      std::exit(1);
    // ── Experimental-only modes (loud warning, proceed) ──────────────────────
    } else if (gm == "schmidt") {
      PRINT_WARNING(YELLOW "[ros-free] EXPERIMENTAL_ONLY: --vio-yaw-gauge-mode=schmidt "
                            "(not validated for GPS-Z flights; not for official results)\n" RESET);
      mapped_mode = "visual_yaw_schmidt_current_gauge";
    } else if (gm == "schmidt_fej") {
      PRINT_WARNING(YELLOW "[ros-free] EXPERIMENTAL_ONLY: --vio-yaw-gauge-mode=schmidt_fej "
                            "(not validated for GPS-Z flights; not for official results)\n" RESET);
      mapped_mode = "visual_yaw_schmidt_fej_gauge";
    } else if (gm == "no_yaw") {
      PRINT_WARNING(YELLOW "[ros-free] EXPERIMENTAL_ONLY: --vio-yaw-gauge-mode=no_yaw "
                            "(gyro-yaw substitution; debug use only)\n" RESET);
      mapped_mode = "hard_gyro_yaw";
    // ── Retired modes (error + stop) ─────────────────────────────────────────
    } else if (gm == "dso") {
      PRINT_ERROR("[ros-free] RETIRED MODE: --vio-yaw-gauge-mode=dso\n"
                  "  DSO v1 K-projection failed fly3 smoke (P_{z,x} corrupted, altitude diverges t=450s).\n"
                  "  Use 'oc_legacy_fej' (mode 10) or 'baseline' (mode 2) instead.\n"
                  "  See failed_modes_retirement_log_20260606.md\n");
      std::exit(1);
    } else if (gm == "vins_nullspace") {
      PRINT_ERROR("[ros-free] RETIRED MODE: --vio-yaw-gauge-mode=vins_nullspace\n"
                  "  VINS v1 smoke showed n_zeroed=0; EVD on S is not VINS marginalization.\n"
                  "  No valid replacement for the VINS nullspace route exists yet.\n"
                  "  See failed_modes_retirement_log_20260606.md\n");
      std::exit(1);
    } else if (gm == "constrained_yaw_nullspace") {
      PRINT_ERROR("[ros-free] RETIRED MODE: --vio-yaw-gauge-mode=constrained_yaw_nullspace\n"
                  "  Smoke FAILED 2026-06-06: P_n exhausted after ~40 calls (time-varying n),\n"
                  "  covariance became indefinite, GPS-Z diverged at t=397.8s.\n"
                  "  See constrained_yaw_nullspace_smoke_report.md\n");
      std::exit(1);
    } else if (gm == "h_proj") {
      PRINT_ERROR("[ros-free] RETIRED MODE: --vio-yaw-gauge-mode=h_proj\n"
                  "  visual_yaw_h_projection_current is superseded by mode 2 (baseline).\n"
                  "  Use '--vio-yaw-gauge-mode baseline' instead.\n");
      std::exit(1);
    } else {
      PRINT_WARNING(YELLOW "[ros-free] Unknown --vio-yaw-gauge-mode '%s'; ignoring\n" RESET, gm.c_str());
    }
    if (!mapped_mode.empty() && args.vio_yaw_update_mode.empty()) {
      args.vio_yaw_update_mode = mapped_mode;
      if (!std::isnan(mapped_alpha) && std::isnan(args.vio_global_yaw_oc_alpha))
        args.vio_global_yaw_oc_alpha = mapped_alpha;
      PRINT_INFO(CYAN "[ros-free] --vio-yaw-gauge-mode=%s → mode=%s alpha=%.1f\n" RESET,
                 gm.c_str(), mapped_mode.c_str(),
                 std::isnan(mapped_alpha) ? -1.0 : mapped_alpha);
    }
  }

  if (args.no_vio_yaw_update) {
    params.enable_vio_yaw_update = false;
    params.vio_yaw_update_mode = "per_block_scale";
    params.vio_yaw_update_scale = 0.0;
    PRINT_INFO(CYAN "[ros-free] CLI override: vio_yaw_update_mode=per_block_scale scale=0.000 (--no-vio-yaw-update)\n" RESET);
  }
  if (!args.vio_yaw_update_mode.empty()) {
    params.vio_yaw_update_mode = args.vio_yaw_update_mode;
    PRINT_INFO(CYAN "[ros-free] CLI override: vio_yaw_update_mode=%s\n" RESET,
               params.vio_yaw_update_mode.c_str());
  }
  if (!std::isnan(args.vio_yaw_update_scale)) {
    if (args.vio_yaw_update_mode.empty() && params.vio_yaw_update_mode == "original") {
      params.vio_yaw_update_mode = "per_block_scale";
      PRINT_INFO(CYAN "[ros-free] CLI override: vio_yaw_update_mode=per_block_scale (scale specified)\n" RESET);
    }
    params.vio_yaw_update_scale = std::max(0.0, std::min(1.0, args.vio_yaw_update_scale));
    params.enable_vio_yaw_update = (params.vio_yaw_update_mode == "original" ||
                                    params.vio_yaw_update_mode == "a_strict_yaw_dx0" ||
                                    params.vio_yaw_update_mode == "strict_yaw_dx0" ||
                                    params.vio_yaw_update_scale > 0.0 ||
                                    params.vio_global_yaw_oc_alpha > 0.0);
    PRINT_INFO(CYAN "[ros-free] CLI override: vio_yaw_update_scale=%.3f\n" RESET,
               params.vio_yaw_update_scale);
  }
  if (!std::isnan(args.vio_global_yaw_oc_alpha)) {
    params.vio_global_yaw_oc_alpha = std::max(0.0, std::min(1.0, args.vio_global_yaw_oc_alpha));
    PRINT_INFO(CYAN "[ros-free] CLI override: vio_global_yaw_oc_alpha=%.3f\n" RESET,
               params.vio_global_yaw_oc_alpha);
  }
  if (args.use_ground_parallel_warp) {
    params.use_ground_parallel_warp = true;
    params.use_gyro_aided_klt = false; // suppressed when warp active (avoids double rotation)
    PRINT_INFO(CYAN "[ros-free] CLI override: use_ground_parallel_warp=true (gyro_aided_klt suppressed)\n" RESET);
  }
  if (args.visual_bgz_update_scale < 1.0 - 1e-12) {
    params.visual_bgz_update_scale = std::max(0.0, std::min(1.0, args.visual_bgz_update_scale));
    PRINT_INFO(CYAN "[ros-free] CLI override: visual_bgz_update_scale=%.4f\n" RESET,
               params.visual_bgz_update_scale);
  }
  if (args.curl_correction_rate_degps != 0.0) {
    params.curl_correction_rate_degps = args.curl_correction_rate_degps;
    PRINT_INFO(CYAN "[ros-free] CLI override: curl_correction_rate=%.3f deg/s\n" RESET,
               args.curl_correction_rate_degps);
  }
  if (!args.vio_yaw_diag_path.empty()) {
    params.vio_yaw_update_diag_path = args.vio_yaw_diag_path;
    PRINT_INFO(CYAN "[ros-free] CLI override: vio_yaw_update_diag_path=%s\n" RESET,
               args.vio_yaw_diag_path.c_str());
  }
  if (!args.visual_obs_diag_path.empty()) {
    params.visual_obs_diag_path = args.visual_obs_diag_path;
    PRINT_INFO(CYAN "[ros-free] CLI override: visual_obs_diag_path=%s\n" RESET,
               args.visual_obs_diag_path.c_str());
  }
  if (!args.schmidt_yaw_diag_path.empty()) {
    params.schmidt_yaw_diag_path = args.schmidt_yaw_diag_path;
    PRINT_INFO(CYAN "[ros-free] CLI override: schmidt_yaw_diag_path=%s\n" RESET,
               args.schmidt_yaw_diag_path.c_str());
  }

  if (!parser->successful()) {
    PRINT_ERROR(RED "[ros-free] failed to parse config %s\n" RESET, args.config_path.c_str());
    return EXIT_FAILURE;
  }

  const bool delayed_vio_yaw_control = args.vio_yaw_control_start_after_init > 1e-9;
  const std::string delayed_vio_yaw_mode = params.vio_yaw_update_mode;
  const double delayed_vio_yaw_scale = params.vio_yaw_update_scale;
  const double delayed_vio_yaw_alpha = params.vio_global_yaw_oc_alpha;
  if (delayed_vio_yaw_control) {
    params.enable_vio_yaw_update = true;
    params.vio_yaw_update_mode = "original";
    params.vio_yaw_update_scale = 1.0;
    params.vio_global_yaw_oc_alpha = 0.0;
    PRINT_INFO(CYAN "[ros-free] delayed VIO yaw control: start original, switch to mode=%s scale=%.3f alpha=%.3f %.1fs after init\n" RESET,
               delayed_vio_yaw_mode.c_str(), delayed_vio_yaw_scale, delayed_vio_yaw_alpha,
               args.vio_yaw_control_start_after_init);
  }
  // Architecture G: must set use_gps_h_offset BEFORE VioManager (State) is constructed
  if (args.gps_alt_arch_g) {
    params.state_options.use_gps_h_offset = true;
    PRINT_INFO(CYAN "[ros-free] Architecture G: state_options.use_gps_h_offset=true "
               "(h_offset Vec(1) added to state, init_sigma=%.1fm walk_sigma=%.4fm/sqrt(s))\n" RESET,
               params.state_options.gps_h_offset_init_sigma,
               params.state_options.gps_h_offset_walk_sigma);
  }
  auto sys = std::make_shared<VioManager>(params);
  if (!args.visual_flow_curl_diag_path.empty() ||
      !args.yaw_update_mechanism_diag_path.empty() ||
      !args.imu_propagation_yaw_diag_path.empty() ||
      !args.slam_feature_yaw_contrib_diag_path.empty() ||
      !args.visual_feature_residual_diag_path.empty() ||
      !args.visual_residual_frame_summary_path.empty() ||
      !args.slam_info_reduction_diag_path.empty() ||
      !args.slam_ekf_leverage_diag_path.empty() ||
      !args.slam_stacked_ekf_diag_path.empty() ||
      !args.slam_landmark_metadata_diag_path.empty() ||
      !args.slam_geometry_refresh_diag_path.empty() ||
      !args.slam_yaw_contrib_cap_diag_path.empty()) {
    sys->set_mechanism_diag_window(args.mech_diag_t0, args.mech_diag_t1);
    sys->get_propagator()->set_yaw_diag_window(args.mech_diag_t0, args.mech_diag_t1);
    PRINT_INFO(CYAN "[ros-free] mechanism diag window: [%.3f, %.3f] camera seconds\n" RESET,
               args.mech_diag_t0, args.mech_diag_t1);
  }
  if (!args.visual_flow_curl_diag_path.empty())
    sys->set_visual_flow_curl_diag_path(args.visual_flow_curl_diag_path);
  if (!args.yaw_update_mechanism_diag_path.empty())
    sys->set_yaw_update_mechanism_diag_path(args.yaw_update_mechanism_diag_path);
  if (!args.imu_propagation_yaw_diag_path.empty())
    sys->get_propagator()->set_yaw_diag_path(args.imu_propagation_yaw_diag_path);
  if (!args.slam_feature_yaw_contrib_diag_path.empty())
    sys->set_slam_feature_yaw_contrib_diag_path(args.slam_feature_yaw_contrib_diag_path);
  if (!args.visual_feature_residual_diag_path.empty() ||
      !args.visual_residual_frame_summary_path.empty()) {
    sys->set_visual_residual_diag_paths(args.visual_feature_residual_diag_path,
                                        args.visual_residual_frame_summary_path);
  }
  if (args.slam_yaw_contrib_cap_enable) {
    sys->configure_slam_yaw_contrib_cap(
        true, args.slam_yaw_contrib_cap_t0, args.slam_yaw_contrib_cap_t1,
        args.slam_yaw_contrib_cap_roll_deg, args.slam_yaw_contrib_cap_yawrate_degps,
        args.slam_yaw_contrib_cap_topk, args.slam_yaw_contrib_cap_ratio,
        args.slam_yaw_contrib_cap_mode);
    PRINT_INFO(CYAN "[ros-free] SLAM yaw contrib cap: window=[%.3f, %.3f] "
               "roll<=%.2f yawrate<=%.2f topk=%d ratio=%.3f mode=%s\n" RESET,
               args.slam_yaw_contrib_cap_t0, args.slam_yaw_contrib_cap_t1,
               args.slam_yaw_contrib_cap_roll_deg,
               args.slam_yaw_contrib_cap_yawrate_degps,
               args.slam_yaw_contrib_cap_topk,
               args.slam_yaw_contrib_cap_ratio,
               args.slam_yaw_contrib_cap_mode.c_str());
  }
  if (!args.slam_yaw_contrib_cap_diag_path.empty())
    sys->set_slam_yaw_contrib_cap_diag_path(args.slam_yaw_contrib_cap_diag_path);
  if (args.slam_info_reduction_enable) {
    sys->configure_slam_info_reduction(true, args.slam_info_reduction_low_ratio,
                                       args.slam_info_reduction_high_ratio,
                                       args.slam_info_reduction_alpha_max);
    PRINT_INFO(CYAN "[ros-free] SLAM info reduction: low=%.3f high=%.3f alpha_max=%.3f\n" RESET,
               args.slam_info_reduction_low_ratio,
               args.slam_info_reduction_high_ratio,
               args.slam_info_reduction_alpha_max);
  }
  if (!args.slam_info_reduction_diag_path.empty())
    sys->set_slam_info_reduction_diag_path(args.slam_info_reduction_diag_path);
  if (!args.slam_ekf_leverage_diag_path.empty())
    sys->set_slam_ekf_leverage_diag_path(args.slam_ekf_leverage_diag_path);
  if (!args.slam_stacked_ekf_diag_path.empty())
    sys->set_slam_stacked_ekf_diag_path(args.slam_stacked_ekf_diag_path);
  if (!args.slam_landmark_metadata_diag_path.empty())
    sys->set_slam_landmark_metadata_diag_path(args.slam_landmark_metadata_diag_path);
  if (args.slam_geometry_lifecycle_refresh_enable) {
    sys->configure_slam_geometry_lifecycle_refresh(
        true,
        args.slam_geometry_refresh_min_depth,
        args.slam_geometry_refresh_min_age,
        args.slam_geometry_refresh_min_pose_lm_cov,
        args.slam_geometry_refresh_min_regular_features,
        args.slam_geometry_refresh_require_anchor_change);
    PRINT_INFO(CYAN "[ros-free] SLAM geometry lifecycle refresh: depth>%.3fm age>%.3fs pose_lm_cov>%.6g min_regular_after=%d require_anchor_change=%d\n" RESET,
               args.slam_geometry_refresh_min_depth,
               args.slam_geometry_refresh_min_age,
               args.slam_geometry_refresh_min_pose_lm_cov,
               args.slam_geometry_refresh_min_regular_features,
               args.slam_geometry_refresh_require_anchor_change ? 1 : 0);
  }
  if (!args.slam_geometry_refresh_diag_path.empty())
    sys->set_slam_geometry_lifecycle_refresh_diag_path(args.slam_geometry_refresh_diag_path);
  if (args.slam_freeze_t0 >= 0.0 && args.slam_freeze_t1 > args.slam_freeze_t0) {
    sys->set_slam_update_freeze_window(args.slam_freeze_t0, args.slam_freeze_t1);
    PRINT_INFO(CYAN "[ros-free] SLAM update freeze window: [%.3f, %.3f]s "
               "(MSCKF/GPS-Z/yaw OC unchanged)\n" RESET,
               args.slam_freeze_t0, args.slam_freeze_t1);
  }
  bool delayed_vio_yaw_control_applied = !delayed_vio_yaw_control;
  bool timed_yaw_switch_applied = args.vio_yaw_switch_mode.empty() || args.vio_yaw_switch_time < 0.0;
  if (!timed_yaw_switch_applied) {
    PRINT_INFO(CYAN "[ros-free] timed yaw switch: at t=%.3f switch to mode=%s alpha=%.3f\n" RESET,
               args.vio_yaw_switch_time, args.vio_yaw_switch_mode.c_str(),
               std::isnan(args.vio_yaw_switch_alpha) ? -1.0 : args.vio_yaw_switch_alpha);
  }

  // Visual update guard (--visual-update-skip-window / --visual-update-guard-log / --visual-update-reject-topn-file)
  sys->set_visual_update_stride(args.visual_update_stride);
  if (args.visual_update_stride > 1) {
    PRINT_INFO(CYAN "[ros-free] Visual update stride: track every fed camera frame, "
               "run MSCKF/SLAM visual updates every %d frames\n" RESET,
               args.visual_update_stride);
  }
  sys->configure_visual_update_adaptive(
      args.visual_update_adaptive,
      args.visual_update_adaptive_target_flow_px,
      args.visual_update_adaptive_min_dt,
      args.visual_update_adaptive_max_dt,
      args.visual_update_adaptive_min_tracks);
  if (args.visual_update_adaptive) {
    PRINT_INFO(CYAN "[ros-free] Visual update adaptive: full-rate tracking, "
               "keyframe observations by accumulated flow>=%.3fpx, min_dt=%.3fs, "
               "max_dt=%.3fs, min_tracks=%d. Skipped-frame observations are removed.\n" RESET,
               args.visual_update_adaptive_target_flow_px,
               args.visual_update_adaptive_min_dt,
               args.visual_update_adaptive_max_dt,
               args.visual_update_adaptive_min_tracks);
  }
  if (args.adaptive_stride || args.adaptive_stride_shadow) {
    PRINT_INFO(CYAN "[adaptive-stride] mode=%s fixed_stride=%d log=%s; "
               "causal inputs only (relative height, VIO speed/attitude, IMU gyro, active features)\n" RESET,
               args.adaptive_stride ? "active" : "shadow", args.cam_subsample,
               args.adaptive_stride_log_path.c_str());
  }
  if (args.camera_frame_adaptive) {
    if (args.camera_frame_adaptive_max_dt > 0.0 &&
        args.camera_frame_adaptive_max_dt < args.camera_frame_adaptive_min_dt) {
      args.camera_frame_adaptive_max_dt = args.camera_frame_adaptive_min_dt;
    }
    args.camera_frame_adaptive_target_ratio = std::max(0.0, args.camera_frame_adaptive_target_ratio);
    args.camera_frame_adaptive_min_dt = std::max(0.0, args.camera_frame_adaptive_min_dt);
    args.camera_frame_adaptive_max_dt = std::max(0.0, args.camera_frame_adaptive_max_dt);
    args.camera_frame_adaptive_min_depth = std::max(0.0, args.camera_frame_adaptive_min_depth);
    args.camera_frame_adaptive_min_tracks = std::max(0, args.camera_frame_adaptive_min_tracks);
    PRINT_INFO(CYAN "[ros-free] Camera frame adaptive input sampling: "
               "target_ratio=%.6f min_dt=%.3fs max_dt=%.3fs min_depth=%.1fm min_tracks=%d. "
               "Frames below the min_dt geometry dead-zone stay full-rate.\n" RESET,
               args.camera_frame_adaptive_target_ratio,
               args.camera_frame_adaptive_min_dt,
               args.camera_frame_adaptive_max_dt,
               args.camera_frame_adaptive_min_depth,
               args.camera_frame_adaptive_min_tracks);
  }
  if (args.visual_skip_t0 >= 0.0 && args.visual_skip_t1 > args.visual_skip_t0) {
    sys->set_visual_skip_window(args.visual_skip_t0, args.visual_skip_t1);
    PRINT_INFO(CYAN "[ros-free] Visual update skip window: [%.3f, %.3f]s\n" RESET,
               args.visual_skip_t0, args.visual_skip_t1);
  }
  if (!args.visual_guard_log_path.empty()) {
    sys->open_visual_guard_log(args.visual_guard_log_path);
  }
  if (!args.visual_reject_file_path.empty()) {
    sys->load_visual_reject_file(args.visual_reject_file_path);
    PRINT_INFO(CYAN "[ros-free] Visual reject file loaded: %s\n" RESET,
               args.visual_reject_file_path.c_str());
  }
  // Guarded-B: push thresholds and open guard diag log
  StateHelper::set_schmidt_guard_config(args.guard_cfg);
  // VinsNullspaceConfig is a no-op (VINS_NUMERIC_NULLSPACE mode retired)
  StateHelper::set_vins_nullspace_config(args.vins_cfg);
  if (!args.guard_diag_log_path.empty()) {
    StateHelper::open_schmidt_guard_log(args.guard_diag_log_path);
    PRINT_INFO(CYAN "[ros-free] Guard diag log: %s\n" RESET, args.guard_diag_log_path.c_str());
  }
  if (params.vio_yaw_update_mode == "visual_yaw_schmidt_guarded") {
    PRINT_INFO(CYAN "[ros-free] Guarded-B: gauge_frac mild=%.2f/severe=%.2f  "
               "norm_dx mild=%.2f/severe=%.2f  Pas mild=%.1f  "
               "R_scale mild=%.0f/severe=%.0f  burst=%d/%.2fs  reject_severe=%d\n" RESET,
               args.guard_cfg.gauge_frac_mild, args.guard_cfg.gauge_frac_severe,
               args.guard_cfg.norm_dx_mild, args.guard_cfg.norm_dx_severe,
               args.guard_cfg.pas_mild,
               args.guard_cfg.r_scale_mild, args.guard_cfg.r_scale_severe,
               args.guard_cfg.burst_count, args.guard_cfg.burst_window_s,
               (int)args.guard_cfg.reject_on_severe);
  }

  // [中文] 设置 P_zz 地板 (如果 CLI 指定)
  if (args.gps_alt_min_pzz > 0) {
    sys->set_gps_alt_min_pzz(args.gps_alt_min_pzz);
    PRINT_INFO(CYAN "[ros-free] GPS alt P_zz floor = %.4f\n" RESET, args.gps_alt_min_pzz);
  }
  // Optional Stage A bootstrap-delay (ablation)
  if (args.gps_alt_min_t_after_init > 0.0) {
    sys->set_gps_alt_min_t_after_init(args.gps_alt_min_t_after_init);
    PRINT_INFO(CYAN "[ros-free] GPS alt bootstrap delay = %.1fs after VIO init\n" RESET,
               args.gps_alt_min_t_after_init);
  }
  // Stage B (ground-plane feature update) — relies on Stage A's z_ground
  if (args.gplane_feat_enable) {
    if (!args.gps_alt_ground_plane) {
      PRINT_WARNING(YELLOW "[ros-free] --gplane-feat requested but --gps-alt-ground-plane not set; "
                    "Stage B will not fire until z_ground is bootstrapped.\n" RESET);
    }
    sys->enable_gplane_feature(args.gplane_feat_sigma_px, args.gplane_feat_max_features,
                               args.gplane_feat_center_frac, args.gplane_feat_min_cos_tilt,
                               args.gplane_feat_max_res_px);
  }
  // Stage B v1 — two-clone H, dry-run + FD by default
  if (args.gplane_feat_v1_enable) {
    if (!args.gps_alt_ground_plane) {
      PRINT_WARNING(YELLOW "[ros-free] --gplane-feat-v1 requested but --gps-alt-ground-plane not set; "
                    "v1 will not fire until z_ground is bootstrapped.\n" RESET);
    }
    bool dry_run = !args.gplane_feat_v1_update;
    sys->enable_gplane_feature_v1(dry_run, args.gplane_feat_sigma_px,
                                  args.gplane_feat_max_features,
                                  args.gplane_feat_center_frac,
                                  args.gplane_feat_min_cos_tilt,
                                  args.gplane_feat_max_res_px,
                                  args.gplane_feat_v1_fd_step_rot,
                                  args.gplane_feat_v1_fd_step_pos,
                                  args.gplane_feat_v1_fd_rel_tol_rot,
                                  args.gplane_feat_v1_fd_rel_tol_pos,
                                  args.gplane_feat_v1_fd_max_abs_rel_tol,
                                  args.gplane_feat_exclude_used_from_msckf,
                                  args.gplane_feat_v1_fd_dump);
  }
  // [中文] 设置创新截断 (如果 CLI 指定, 建议 30m)
  if (args.gps_alt_max_res < 1e8) {
    sys->set_gps_alt_max_res_gate(args.gps_alt_max_res);
    PRINT_INFO(CYAN "[ros-free] GPS alt max-res gate = %.1f m\n" RESET, args.gps_alt_max_res);
  }
  // [中文] 交叉协方差 guard
  if (args.gps_alt_guard_dxy > 0) {
    sys->set_gps_alt_guard_dxy_max(args.gps_alt_guard_dxy);
    PRINT_INFO(CYAN "[ros-free] GPS alt guard |dXY| max = %.3f m\n" RESET, args.gps_alt_guard_dxy);
  }
  if (args.gps_alt_guard_kxy_ratio > 0) {
    sys->set_gps_alt_guard_kxy_ratio_max(args.gps_alt_guard_kxy_ratio);
    PRINT_INFO(CYAN "[ros-free] GPS alt guard |K_xy|/|K_pz| max = %.3f\n" RESET, args.gps_alt_guard_kxy_ratio);
  }
  if (args.gps_alt_guard_dbias > 0) {
    sys->set_gps_alt_guard_dbias_max(args.gps_alt_guard_dbias);
    PRINT_INFO(CYAN "[ros-free] GPS alt guard |dbias| max = %.5f\n" RESET, args.gps_alt_guard_dbias);
  }
  if (!sys->configure_gps_alt_coupled_update(
          args.gps_alt_coupled_mode,
          args.gps_alt_residual_soft_limit,
          args.gps_alt_max_delta_xy,
          args.gps_alt_max_delta_z,
          args.gps_alt_max_delta_velocity,
          args.gps_alt_max_delta_attitude_deg * M_PI / 180.0,
          args.gps_alt_max_delta_accel_bias,
          args.gps_alt_max_delta_gyro_bias,
          args.gps_alt_cov_psd_check_interval)) {
    return EXIT_FAILURE;
  }
  sys->set_gps_alt_nasa_underweight(args.gps_alt_nasa_beta, args.gps_alt_nasa_q_threshold);
  if (args.gps_alt_nasa_beta > 0.0) {
    PRINT_INFO(CYAN "[ros-free] GPS alt NASA/Lear underweight beta=%.4f q_threshold=%.4f m^2\n" RESET,
               args.gps_alt_nasa_beta, args.gps_alt_nasa_q_threshold);
  }
  if (!args.gps_alt_coupled_diag_path.empty())
    sys->set_gps_alt_coupled_diag_path(args.gps_alt_coupled_diag_path);
  PRINT_INFO(CYAN "[ros-free] GPS alt coupled mode=%s residual_limit=%.3f "
                  "dxy=%.3f dz=%.3f dv=%.3f dtheta=%.3fdeg "
                  "dba=%.6f dbg=%.6f psd_interval=%.3fs\n" RESET,
             args.gps_alt_coupled_mode.c_str(),
             args.gps_alt_residual_soft_limit,
             args.gps_alt_max_delta_xy,
             args.gps_alt_max_delta_z,
             args.gps_alt_max_delta_velocity,
             args.gps_alt_max_delta_attitude_deg,
             args.gps_alt_max_delta_accel_bias,
             args.gps_alt_max_delta_gyro_bias,
             args.gps_alt_cov_psd_check_interval);
  if (args.gps_alt_arch_g) {
    if (!params.state_options.use_gps_h_offset) {
      PRINT_ERROR(RED "[ros-free] --gps-alt-arch-g requires state_options.use_gps_h_offset=true "
                  "(set via --gps-alt-arch-g which auto-enables it)\n" RESET);
    }
    sys->set_gps_alt_arch_g(true);
    PRINT_INFO(CYAN "[ros-free] GPS alt ARCHITECTURE-G ENABLED "
               "(h_offset augmented state; GPS_z = p_z + h_offset; standard EKF update)\n" RESET);
  }
  if (args.gps_alt_zonly) {
    sys->set_gps_alt_zonly_update(true);
    PRINT_INFO(CYAN "[ros-free] GPS alt ZONLY-LEGACY mode ENABLED "
               "(only p_z state correction; full covariance update applied)\n" RESET);
  }
  if (args.gps_alt_joseph) {
    sys->set_gps_alt_joseph_update(true);
    PRINT_INFO(CYAN "[ros-free] GPS alt JOSEPH-MASKED mode ENABLED "
               "(PX4-style: p_z state + p_z row/col of P only; Brink 2017)\n" RESET);
  }
  const int num_cams = params.state_options.num_cameras;
  if (args.stereo && num_cams < 2) {
    PRINT_ERROR(RED "[ros-free] --stereo requested but config has num_cameras=%d\n" RESET, num_cams);
    return EXIT_FAILURE;
  }

  // -------------------- load dataset --------------------
  fs::path root(args.dataset_dir);
  std::vector<DatasetReaderEuroc::ImuSample> imu;
  std::vector<DatasetReaderEuroc::CamEntry> cam0, cam1;
  std::vector<DatasetReaderEuroc::GtSample> gt;
  std::vector<DatasetReaderEuroc::GpsSample> gps;

  if (!DatasetReaderEuroc::load_imu((root / "imu0" / "data.csv").string(), imu))
    return EXIT_FAILURE;
  if (!DatasetReaderEuroc::load_cam((root / "cam0").string(), cam0))
    return EXIT_FAILURE;
  if (args.stereo && !DatasetReaderEuroc::load_cam((root / "cam1").string(), cam1))
    return EXIT_FAILURE;
  if (!args.gt_path.empty())
    DatasetReaderEuroc::load_gt(args.gt_path, gt);
  else
    DatasetReaderEuroc::load_gt((root / "state_groundtruth_estimate0" / "data.csv").string(), gt);
  if (!args.gps_path.empty())
    DatasetReaderEuroc::load_gps(args.gps_path, gps);

  // [中文] 应用 GPS 时间偏移 (config: gps_time_offset, 单位秒, 默认 0).
  // 用于数据集 GPS 时间戳与 IMU 不对齐的情况 (例如 jc82 18r.bag,
  // GPS 来自单独 ArduPilot DataFlash log, 锚点偏 +36s).
  // 一次性应用于全部样本, 之后下游 (load_gps 派生 map / feed loop) 不再感知.
  if (!gps.empty() && std::fabs(params.gps_time_offset) > 1e-9) {
    for (auto &s : gps)
      s.timestamp += params.gps_time_offset;
    PRINT_INFO(CYAN "[ros-free] applied gps_time_offset=%+.3fs to %zu GPS samples\n" RESET,
               params.gps_time_offset, gps.size());
  }

  // [中文] --start-time: 跳过前 N 秒的 IMU / cam0 / cam1 (相对 bag 第一条 IMU)
  // 用于复现 ROS bag_start:=58 这种场景, 跳过放置静止段
  if (args.start_time > 0.0 && !imu.empty()) {
    double t0 = imu.front().timestamp;
    double t_skip = t0 + args.start_time;
    size_t imu_before = imu.size(), cam0_before = cam0.size(), cam1_before = cam1.size();
    imu.erase(std::remove_if(imu.begin(), imu.end(),
                              [t_skip](const DatasetReaderEuroc::ImuSample &s) { return s.timestamp < t_skip; }),
               imu.end());
    cam0.erase(std::remove_if(cam0.begin(), cam0.end(),
                               [t_skip](const DatasetReaderEuroc::CamEntry &s) { return s.timestamp < t_skip; }),
                cam0.end());
    cam1.erase(std::remove_if(cam1.begin(), cam1.end(),
                               [t_skip](const DatasetReaderEuroc::CamEntry &s) { return s.timestamp < t_skip; }),
                cam1.end());
    PRINT_INFO(CYAN "[ros-free] --start-time=%.1fs dropped imu=%zu cam0=%zu cam1=%zu entries\n" RESET,
               args.start_time, imu_before - imu.size(), cam0_before - cam0.size(), cam1_before - cam1.size());
  }

  bool fc_init_pending = false;
  FCInitState fc_init_state;
  if (!args.init_from_fc_path.empty()) {
    try {
      fc_init_state = load_fc_init_state_csv(args.init_from_fc_path, args.start_time);
      fc_init_pending = true;
      PRINT_INFO(CYAN "[ros-free] loaded FC init CSV: %s\n" RESET, args.init_from_fc_path.c_str());
      PRINT_INFO(CYAN "[ros-free] FC init row t=%.6f target start=%.6f dt=%.6f |v|=%.3f\n" RESET,
                 fc_init_state.timestamp, args.start_time, fc_init_state.source_dt, fc_init_state.v_IinG.norm());
      PRINT_INFO(CYAN "[ros-free] FC init covariance sigmas: att=%.2fdeg vel=%.2fm/s pos=%.2fm bg=%.4frad/s ba=%.3fm/s^2\n" RESET,
                 args.init_att_sigma_deg, args.init_vel_sigma, args.init_pos_sigma, args.init_bg_sigma, args.init_ba_sigma);
      if (params.use_gyro_aided_klt && params.use_gyro_aided_klt_max_bg_sigma > 0.0 &&
          args.init_bg_sigma > params.use_gyro_aided_klt_max_bg_sigma) {
        PRINT_WARNING(YELLOW "[ros-free] FC init bg sigma %.5f exceeds gyro-aided KLT gate %.5f; "
                             "rotation-aided tracking will be disabled until bg covariance shrinks.\n" RESET,
                      args.init_bg_sigma, params.use_gyro_aided_klt_max_bg_sigma);
      }
    } catch (const std::exception &e) {
      PRINT_ERROR(RED "[ros-free] failed to load --init-from-fc %s: %s\n" RESET,
                  args.init_from_fc_path.c_str(), e.what());
      return EXIT_FAILURE;
    }
  }

  // GT map keyed by time (VIO world = GT world approximately after align)
  std::map<double, Eigen::Vector3d> gt_pos_map;
  for (const auto &s : gt)
    gt_pos_map[s.timestamp] = s.p;
  std::map<double, Eigen::Vector3d> gps_pos_map;
  for (const auto &s : gps)
    gps_pos_map[s.timestamp] = s.xyz;

  // -------------------- diagnostic logger + printer --------------------
  DiagLogger diag_logger(args.diag_csv_path, args.diag_events_path);

  // Mode string built once from active CLI flags (written into every CSV row).
  std::string diag_mode_str;
  std::string diag_init_source;
  {
    std::ostringstream ss;
    if (!args.init_from_fc_path.empty()) ss << "FC";
    if (args.gplane_feat_v1_enable)      ss << "+Bv1";
    else if (args.gplane_feat_enable)    ss << "+B";
    if (args.gps_alt_update)             ss << "+gpsAlt";
    diag_mode_str = ss.str();
    if (diag_mode_str.empty()) diag_mode_str = "DynInit";
    diag_init_source = args.init_from_fc_path.empty() ? "DynInit" : "FC";
  }

  // Accumulated path lengths and GPS tracking (for CSV / [DIAG] print)
  double vio_path_len = 0.0;
  double gps_path_len = 0.0;
  Eigen::Vector3d prev_p_wi = Eigen::Vector3d::Zero();
  bool prev_p_wi_valid = false;
  Eigen::Vector3d prev_gps_xyz = Eigen::Vector3d::Zero();
  double prev_gps_t = -1.0;
  double cur_gps_speed = -1.0;
  Eigen::Vector3d cur_gps_xyz = Eigen::Vector3d::Zero();

  // One-shot event detection flags (used to fire events exactly once)
  bool event_init_fired = false;
  bool event_first_accept_fired = false;
  bool event_first_slam_fired = false;
  double first_slam_t = -1.0;
  double first_accept_t = -1.0;

  // -------------------- dashboard --------------------
  VizDashboard::Options vo;
  vo.show_window = args.show;
  vo.window_title = args.dash_title;
  vo.video_path = args.video_path;
  vo.video_fps = args.video_fps;
  vo.fast = args.viz_fast;
  VizDashboard dash(vo);
  TrajectoryAligner aligner;

  // [中文] 可选: 额外录一个"纯相机 + 光流轨迹"视频 (cam0/cam1 并排, 由 VioManager
  // 的 TrackBase::display_history 提供). 帧尺寸以第一帧为准.
  cv::VideoWriter cam_writer;
  cv::Size cam_video_size;
  bool cam_writer_init = false;

  // -------------------- output file --------------------
  std::ofstream out(args.output_path);
  if (!out.is_open()) {
    PRINT_ERROR(RED "[ros-free] cannot open output: %s\n" RESET, args.output_path.c_str());
    return EXIT_FAILURE;
  }
  out << "# TUM traj (t x y z qx qy qz qw) in VIO (unaligned) frame\n";
  out << std::fixed << std::setprecision(9);

  // [中文] debug: 记录每帧 ba/bg/|v| 到 side-file, 用于分析发散原因
  std::ofstream debug_out(args.output_path + ".bias");
  debug_out << "# t_cam vx vy vz bg_x bg_y bg_z ba_x ba_y ba_z\n";
  debug_out << std::fixed << std::setprecision(6);

  AdaptiveStrideController adaptive_stride_controller;
  std::ofstream adaptive_stride_out;
  if (args.adaptive_stride || args.adaptive_stride_shadow) {
    fs::path p(args.adaptive_stride_log_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    adaptive_stride_out.open(args.adaptive_stride_log_path,
                             std::ofstream::out | std::ofstream::trunc);
    if (!adaptive_stride_out.is_open()) {
      PRINT_ERROR(RED "[adaptive-stride] cannot open policy CSV: %s\n" RESET,
                  args.adaptive_stride_log_path.c_str());
      return EXIT_FAILURE;
    }
    adaptive_stride_out
        << "timestamp,current_fixed_stride,recommended_stride,relative_height,"
        << "horizontal_speed,roll,pitch,gyro_norm,active_msckf_features,"
        << "active_slam_features,reason_height,reason_turn,reason_feature_low,"
        << "hysteresis_state,hold_timer,base_stride,filtered_height,filtered_speed,"
        << "effective_height,filtered_gyro,severe_turn,severe_feature_low,changed,"
        << "switch_reason,height_source,mode,applied_stride,frame_fed\n";
    adaptive_stride_out << std::fixed << std::setprecision(9);
  }

  std::ofstream state_safety_out;
  if (!args.state_safety_diag_path.empty()) {
    fs::path p(args.state_safety_diag_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    state_safety_out.open(args.state_safety_diag_path, std::ofstream::out | std::ofstream::trunc);
    if (!state_safety_out.is_open()) {
      PRINT_WARNING(YELLOW "[STATE-SAFETY] failed to open %s\n" RESET, args.state_safety_diag_path.c_str());
    } else {
      state_safety_out
          << "time,frame_id,cov_dim,n_clones,n_slam,n_aruco,n_should_marg,n_invalid_landmark_id,"
          << "cov_all_finite,cov_nan_count,cov_inf_count,cov_sym_fro,cov_sym_max_abs,"
          << "cov_min_diag,cov_max_diag,cov_trace,cov_neg_diag_count,cov_min_eig,"
          << "bgx,bgy,bgz,bax,bay,baz,vx,vy,vz,px,py,pz,qx,qy,qz,qw\n";
      state_safety_out.flush();
      PRINT_INFO(GREEN "[STATE-SAFETY] writing %s\n" RESET, args.state_safety_diag_path.c_str());
    }
  }

  // -------------------- timeline merge-sort --------------------
  // [中文] OpenVINS 滤波器完全由传感器时间戳驱动。我们把 IMU 样本和相机帧
  // 按时间戳合并排序后顺序喂入, 与 ROS 版本结果数值一致 (仅受初始化线程
  // 非确定性影响)。
  size_t imu_i = 0, cam_i = 0, cam1_i = 0, gps_i = 0;
  // Track last GPS sample index actually fed to the filter, so we don't
  // feed the same physical GPS sample multiple times when cam_hz > gps_hz.
  long long last_fed_gps_i = -1;
  // Reference-only GPS bookkeeping for dashboard/diagnostic CSV. This stays
  // active with --gps even when GPS is not fused into the EKF.
  long long last_diag_gps_i = -1;
  const double INF = std::numeric_limits<double>::infinity();
  double t_init_done = -1; // [中文] 滤波器完成初始化的时刻
  std::deque<std::pair<double, Eigen::Vector3d>> vio_for_align;
  int frame_idx = 0;
  int align_fit_count = 0;
  size_t total_image_input_count = 0;
  size_t processed_image_count = 0;
  size_t skipped_image_count = 0;
  size_t imu_sample_count = 0;
  std::vector<double> processed_image_timestamps;
  size_t adaptive_raw_frames_since_feed = 0;
  size_t adaptive_switch_count = 0;
  size_t adaptive_down_switch_count = 0;
  size_t adaptive_up_switch_count = 0;
  bool adaptive_have_vio_height_origin = false;
  double adaptive_vio_height_origin = 0.0;
  AdaptiveStrideDecision adaptive_decision;
  struct CameraFrameAdaptiveStats {
    size_t enabled = 0;
    size_t feed_count = 0;
    size_t skip_count = 0;
    size_t first_trigger_count = 0;
    size_t target_dt_trigger_count = 0;
    size_t max_dt_trigger_count = 0;
    size_t track_safety_trigger_count = 0;
    size_t no_geometry_feed_count = 0;
    size_t depth_deadzone_feed_count = 0;
    size_t fullrate_deadzone_feed_count = 0;
    double last_feed_time = -1.0;
    double last_dt_since_feed = 0.0;
    double last_desired_dt = 0.0;
    double last_depth_median = -1.0;
    double last_speed = -1.0;
    int last_track_count = -1;
    std::string last_reason = "not_enabled";
  } camera_adaptive_stats;
  camera_adaptive_stats.enabled = args.camera_frame_adaptive ? 1 : 0;

  // ---- Diagnostic tracking state ----
  double prev_t_cam = -1.0;           // previous camera frame timestamp
  size_t imu_i_prev_cam = 0;          // imu_i at start of previous camera frame
  bool diag_triggered = false;        // set on first chi2 wave
  int diag_frames_left = 0;           // countdown of frames to print detailed diag
  int prev_slam_count = 0;            // slam count previous frame (for drop detection)

  // [中文] GPS 高度当 1D 观测量时: 第一帧 GPS 到达时同时记录 GPS z 和 VIO z,
  // 后续 measurement = GPS_now - GPS_ref + VIO_ref, 对齐到 VIO 世界坐标系.
  double gps_alt_ref = std::numeric_limits<double>::quiet_NaN();
  double gps_alt_vio_ref = 0.0;
  Eigen::Vector2d gps_xy_ref = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d gps_xy_vio_ref = Eigen::Vector2d::Zero();
  size_t gps_alt_feeds = 0, gps_alt_rejects = 0;

  auto maybe_align = [&](double t) {
    if (aligner.solved() || t_init_done < 0)
      return;
    if (t - t_init_done < args.align_seconds)
      return;
    std::vector<Eigen::Vector3d> pv, pg;
    // [中文] 先试 GT, 若没有则用 GPS (GPS 5Hz, 放宽容差到 0.15s)
    const auto &align_map = gt_pos_map.empty() ? gps_pos_map : gt_pos_map;
    double align_tol = gt_pos_map.empty() ? 0.15 : 0.03;
    TrajectoryAligner::build_pairs(vio_for_align, align_map, align_tol, pv, pg);
    if (pv.size() >= 10) {
      if (aligner.solve_xy_yaw(pv, pg)) {
        const double yaw_deg = std::atan2(aligner.R()(1, 0), aligner.R()(0, 0)) * 180.0 / M_PI;
        PRINT_INFO(GREEN "[ros-free] XY yaw alignment solved using %zu pairs (%s), yaw=%.2f deg\n" RESET,
                   pv.size(), gt_pos_map.empty() ? "GPS" : "GT", yaw_deg);
        dash.set_alignment(aligner.R(), aligner.t(), true);
        align_fit_count = (int)pv.size();
      }
    }
  };

  auto reference_course_yaw_at = [&](double t, double &yaw_deg) -> bool {
    if (gps.size() < 2)
      return false;
    auto it = std::lower_bound(
        gps.begin(), gps.end(), t,
        [](const DatasetReaderEuroc::GpsSample &s, double tt) { return s.timestamp < tt; });
    size_t i1 = 0;
    if (it == gps.begin()) {
      i1 = 1;
    } else if (it == gps.end()) {
      i1 = gps.size() - 1;
    } else {
      i1 = (size_t)std::distance(gps.begin(), it);
    }
    size_t i0 = i1 > 0 ? i1 - 1 : 0;
    if (i0 == i1 || i1 >= gps.size())
      return false;
    double nearest_dt = std::min(std::fabs(gps[i0].timestamp - t), std::fabs(gps[i1].timestamp - t));
    if (nearest_dt > 0.50)
      return false;
    double dt = gps[i1].timestamp - gps[i0].timestamp;
    if (dt <= 1e-6)
      return false;
    Eigen::Vector3d d = gps[i1].xyz - gps[i0].xyz;
    double speed_xy = std::hypot(d.x(), d.y()) / dt;
    if (speed_xy < 0.5)
      return false;
    yaw_deg = std::atan2(d.y(), d.x()) * 180.0 / M_PI;
    return std::isfinite(yaw_deg);
  };

  while (!g_stop.load() && (imu_i < imu.size() || cam_i < cam0.size())) {
    double t_imu = imu_i < imu.size() ? imu[imu_i].timestamp : INF;
    double t_cam = cam_i < cam0.size() ? cam0[cam_i].timestamp : INF;
    if (t_cam > args.until_time && t_imu > args.until_time) break;

    if (t_imu <= t_cam) {
      ov_core::ImuData m;
      m.timestamp = t_imu;
      m.wm = imu[imu_i].gyro;
      m.am = imu[imu_i].accel;
      sys->feed_measurement_imu(m);
      imu_sample_count++;
      imu_i++;
      continue;
    }

    // ------ camera frame ------
    total_image_input_count++;
    ov_core::CameraData msg;
    msg.timestamp = t_cam;
    cv::Mat img0 = cv::imread(cam0[cam_i].image_path, cv::IMREAD_GRAYSCALE);
    if (img0.empty()) {
      PRINT_WARNING(YELLOW "[ros-free] failed to read %s, skipping\n" RESET,
                    cam0[cam_i].image_path.c_str());
      skipped_image_count++;
      cam_i++;
      continue;
    }
    msg.sensor_ids.push_back(0);
    msg.images.push_back(img0);
    msg.masks.push_back(cv::Mat::zeros(img0.size(), CV_8UC1));

    if (args.stereo && !cam1.empty()) {
      // [中文] Stereo 同步: ROS1 serial runner 策略 (ov_msckf/src/ros1_serial_msckf.cpp:
      // 214-247). 向前搜索 cam1 直到时间戳 >= t_cam - 20ms, 若距离 < 20ms 就配对.
      // 相比之前的 greedy 双向搜索, 这个单向 advance + lower_bound 对重复时间戳
      // 鲁棒 (18r.bag 中 t≈95/164/199/262s 有 5 处重复 ts, 以前的 greedy 会永久
      // 落后 1 帧导致 cam1 被全部丢弃).
      while (cam1_i < cam1.size() && cam1[cam1_i].timestamp < t_cam - 0.02)
        cam1_i++;
      if (cam1_i < cam1.size() && std::fabs(cam1[cam1_i].timestamp - t_cam) < 0.02) {
        cv::Mat img1 = cv::imread(cam1[cam1_i].image_path, cv::IMREAD_GRAYSCALE);
        if (!img1.empty()) {
          msg.sensor_ids.push_back(1);
          msg.images.push_back(img1);
          msg.masks.push_back(cv::Mat::zeros(img1.size(), CV_8UC1));
        }
      }
    }

    if (fc_init_pending && !sys->initialized()) {
      const double att_sigma_rad = args.init_att_sigma_deg * 3.14159265358979323846 / 180.0;
      sys->initialize_with_fc_state(fc_init_state,
                                    att_sigma_rad,
                                    args.init_vel_sigma,
                                    args.init_pos_sigma,
                                    args.init_bg_sigma,
                                    args.init_ba_sigma,
                                    t_cam);
      fc_init_pending = false;
      skipped_image_count++;
      cam_i++;
      continue;
    }

    double t0 = cv::getTickCount() / cv::getTickFrequency();
    double ref_course_yaw = 0.0;
    bool ref_course_valid = reference_course_yaw_at(t_cam, ref_course_yaw);
    sys->set_reference_course_yaw(t_cam, ref_course_yaw, ref_course_valid);
    AdaptiveStrideInput adaptive_input;
    std::string adaptive_height_source = "unavailable";
    int adaptive_prev_stride = adaptive_stride_controller.recommended_stride();
    if (args.adaptive_stride || args.adaptive_stride_shadow) {
      adaptive_input.timestamp = t_cam;
      adaptive_input.initialized = sys->initialized();
      adaptive_input.configured_feature_count = params.num_pts;
      if (imu_i > 0 && imu_i <= imu.size())
        adaptive_input.gyro_norm_radps = imu[imu_i - 1].gyro.norm();

      // Causal relative height: the latest GPS sample at or before t_cam.
      // DatasetReaderEuroc anchors ENU-U at the first (ground) GPS sample.  No
      // future sample, course, reference error, or flight identity is read.
      if (!gps.empty()) {
        auto it = std::upper_bound(
            gps.begin(), gps.end(), t_cam,
            [](double tt, const DatasetReaderEuroc::GpsSample &s) { return tt < s.timestamp; });
        if (it != gps.begin()) {
          --it;
          if (t_cam - it->timestamp <= 2.0) {
            adaptive_input.relative_height_m = it->xyz.z();
            adaptive_height_source = "gps_past_enu_u";
          }
        }
      }

      if (sys->initialized()) {
        auto state_pre = sys->get_state();
        const Eigen::Vector3d p_wi_pre = state_pre->_imu->pos();
        const Eigen::Vector3d v_wi_pre = state_pre->_imu->vel();
        adaptive_input.horizontal_speed_mps = v_wi_pre.head<2>().norm();
        Eigen::Matrix3d R_wi_pre = state_pre->_imu->Rot().transpose();
        Eigen::Quaterniond q_pre(R_wi_pre);
        const double qw = q_pre.w(), qx = q_pre.x(), qy = q_pre.y(), qz = q_pre.z();
        adaptive_input.roll_deg = std::atan2(2.0 * (qw * qx + qy * qz),
                                             1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / M_PI;
        adaptive_input.pitch_deg = std::asin(std::max(-1.0, std::min(1.0,
                                              2.0 * (qw * qy - qz * qx)))) * 180.0 / M_PI;
        adaptive_input.active_slam_features = (int)sys->get_features_SLAM().size();
        ov_core::TrackerWarpVizPacket pkt_stride;
        if (sys->get_warp_viz_packet(0, pkt_stride))
          adaptive_input.active_msckf_features = (int)pkt_stride.curr_pts_raw.size();

        if (!adaptive_have_vio_height_origin) {
          adaptive_vio_height_origin = p_wi_pre.z();
          adaptive_have_vio_height_origin = true;
        }
        if (!std::isfinite(adaptive_input.relative_height_m)) {
          adaptive_input.relative_height_m = std::max(0.0, p_wi_pre.z() - adaptive_vio_height_origin);
          adaptive_height_source = "vio_relative_fallback";
        }
      }
      adaptive_decision = adaptive_stride_controller.update(adaptive_input);
      if (adaptive_decision.changed) {
        adaptive_switch_count++;
        if (adaptive_decision.recommended_stride > adaptive_prev_stride)
          adaptive_up_switch_count++;
        else
          adaptive_down_switch_count++;
        PRINT_INFO(CYAN "[adaptive-stride] t=%.3f %d->%d base=%d reason=%s "
                   "h=%.1f roll=%.1f gyro=%.3f feats=%d+%d\n" RESET,
                   t_cam, adaptive_prev_stride, adaptive_decision.recommended_stride,
                   adaptive_decision.base_stride, adaptive_decision.switch_reason.c_str(),
                   adaptive_input.relative_height_m, adaptive_input.roll_deg,
                   adaptive_input.gyro_norm_radps, adaptive_input.active_msckf_features,
                   adaptive_input.active_slam_features);
      }
    }
    bool do_cam_feed = (args.cam_subsample <= 1) || (frame_idx % args.cam_subsample == 0);
    if (args.adaptive_stride) {
      adaptive_raw_frames_since_feed++;
      do_cam_feed = processed_image_count == 0 ||
                    adaptive_raw_frames_since_feed >=
                        (size_t)std::max(1, adaptive_decision.recommended_stride);
    }
    if (args.camera_frame_adaptive) {
      do_cam_feed = true;
      camera_adaptive_stats.last_reason = "not_initialized";
      camera_adaptive_stats.last_dt_since_feed =
          (camera_adaptive_stats.last_feed_time > 0.0)
              ? std::max(0.0, t_cam - camera_adaptive_stats.last_feed_time)
              : 0.0;
      camera_adaptive_stats.last_desired_dt = 0.0;
      camera_adaptive_stats.last_depth_median = -1.0;
      camera_adaptive_stats.last_speed = -1.0;
      camera_adaptive_stats.last_track_count = -1;

      if (sys->initialized()) {
        auto state_pre = sys->get_state();
        const Eigen::Vector3d p_wi_pre = state_pre->_imu->pos();
        const double speed_pre = state_pre->_imu->vel().norm();
        camera_adaptive_stats.last_speed = speed_pre;

        std::vector<Eigen::Vector3d> slam_pts_pre = sys->get_features_SLAM();
        if (!slam_pts_pre.empty()) {
          std::vector<double> depths;
          depths.reserve(slam_pts_pre.size());
          for (const auto &fp : slam_pts_pre) {
            const double depth = (fp - p_wi_pre).norm();
            if (std::isfinite(depth) && depth > 0.0)
              depths.push_back(depth);
          }
          if (!depths.empty()) {
            std::sort(depths.begin(), depths.end());
            camera_adaptive_stats.last_depth_median = depths[depths.size() / 2];
          }
        }

        ov_core::TrackerWarpVizPacket pkt_adapt;
        if (sys->get_warp_viz_packet(0, pkt_adapt))
          camera_adaptive_stats.last_track_count = (int)pkt_adapt.curr_pts_raw.size();

        const bool first_feed = camera_adaptive_stats.last_feed_time < 0.0;
        const bool track_safety =
            !first_feed &&
            args.camera_frame_adaptive_min_tracks > 0 &&
            camera_adaptive_stats.last_track_count >= 0 &&
            camera_adaptive_stats.last_track_count < args.camera_frame_adaptive_min_tracks;
        const bool have_geometry =
            std::isfinite(camera_adaptive_stats.last_depth_median) &&
            camera_adaptive_stats.last_depth_median > 1.0 &&
            std::isfinite(speed_pre) && speed_pre > 1.0 &&
            args.camera_frame_adaptive_target_ratio > 0.0;

        if (first_feed) {
          do_cam_feed = true;
          camera_adaptive_stats.first_trigger_count++;
          camera_adaptive_stats.last_reason = "adaptive_first";
        } else if (track_safety) {
          do_cam_feed = true;
          camera_adaptive_stats.track_safety_trigger_count++;
          camera_adaptive_stats.last_reason = "adaptive_track_safety";
        } else if (!have_geometry) {
          do_cam_feed = true;
          camera_adaptive_stats.no_geometry_feed_count++;
          camera_adaptive_stats.last_reason = "adaptive_no_geometry";
        } else if (args.camera_frame_adaptive_min_depth > 0.0 &&
                   camera_adaptive_stats.last_depth_median < args.camera_frame_adaptive_min_depth) {
          do_cam_feed = true;
          camera_adaptive_stats.depth_deadzone_feed_count++;
          camera_adaptive_stats.last_reason = "adaptive_depth_deadzone";
        } else {
          const double raw_desired_dt =
              args.camera_frame_adaptive_target_ratio *
              camera_adaptive_stats.last_depth_median / speed_pre;
          double desired_dt = raw_desired_dt;
          if (args.camera_frame_adaptive_max_dt > 0.0)
            desired_dt = std::min(desired_dt, args.camera_frame_adaptive_max_dt);
          camera_adaptive_stats.last_desired_dt = desired_dt;

          if (raw_desired_dt < args.camera_frame_adaptive_min_dt) {
            do_cam_feed = true;
            camera_adaptive_stats.fullrate_deadzone_feed_count++;
            camera_adaptive_stats.last_reason = "adaptive_fullrate_deadzone";
          } else if (camera_adaptive_stats.last_dt_since_feed + 1e-9 >= desired_dt) {
            do_cam_feed = true;
            if (std::fabs(desired_dt - args.camera_frame_adaptive_max_dt) < 1e-9 &&
                raw_desired_dt > desired_dt) {
              camera_adaptive_stats.max_dt_trigger_count++;
              camera_adaptive_stats.last_reason = "adaptive_max_dt";
            } else {
              camera_adaptive_stats.target_dt_trigger_count++;
              camera_adaptive_stats.last_reason = "adaptive_target_ratio";
            }
          } else {
            do_cam_feed = false;
            camera_adaptive_stats.last_reason = "adaptive_wait";
          }
        }
      } else {
        camera_adaptive_stats.no_geometry_feed_count++;
      }

      if (do_cam_feed)
        camera_adaptive_stats.feed_count++;
      else
        camera_adaptive_stats.skip_count++;
    }
    if (adaptive_stride_out.is_open()) {
      adaptive_stride_out
          << t_cam << "," << args.cam_subsample << ","
          << adaptive_decision.recommended_stride << ","
          << adaptive_input.relative_height_m << ","
          << adaptive_input.horizontal_speed_mps << ","
          << adaptive_input.roll_deg << "," << adaptive_input.pitch_deg << ","
          << adaptive_input.gyro_norm_radps << ","
          << adaptive_input.active_msckf_features << ","
          << adaptive_input.active_slam_features << ","
          << (adaptive_decision.reason_height_valid ? 1 : 0) << ","
          << (adaptive_decision.reason_turn ? 1 : 0) << ","
          << (adaptive_decision.reason_feature_low ? 1 : 0) << ","
          << adaptive_decision.hysteresis_state << ","
          << adaptive_decision.hold_timer_s << ","
          << adaptive_decision.base_stride << ","
          << adaptive_decision.filtered_height_m << ","
          << adaptive_decision.filtered_speed_mps << ","
          << adaptive_decision.effective_height_m << ","
          << adaptive_decision.filtered_gyro_radps << ","
          << (adaptive_decision.severe_turn ? 1 : 0) << ","
          << (adaptive_decision.severe_feature_low ? 1 : 0) << ","
          << (adaptive_decision.changed ? 1 : 0) << ","
          << adaptive_decision.switch_reason << ","
          << adaptive_height_source << ","
          << (args.adaptive_stride ? "active" : "shadow") << ","
          << (args.adaptive_stride ? adaptive_decision.recommended_stride : args.cam_subsample) << ","
          << (do_cam_feed ? 1 : 0) << "\n";
    }
    if (do_cam_feed) {
      sys->feed_measurement_camera(msg);
      processed_image_count++;
      processed_image_timestamps.push_back(t_cam);
      if (args.adaptive_stride)
        adaptive_raw_frames_since_feed = 0;
      if (args.camera_frame_adaptive)
        camera_adaptive_stats.last_feed_time = t_cam;
    } else {
      skipped_image_count++;
    }
    double dt_ms = 1000.0 * (cv::getTickCount() / cv::getTickFrequency() - t0);

    // ------ query latest state ------
    dash.set_initialized(sys->initialized());
    if (sys->initialized()) {
      if (t_init_done < 0)
        t_init_done = t_cam;
      if (!delayed_vio_yaw_control_applied &&
          t_init_done >= 0.0 &&
          t_cam - t_init_done >= args.vio_yaw_control_start_after_init) {
        sys->set_vio_yaw_update_mode(delayed_vio_yaw_mode);
        sys->set_vio_yaw_update_scale(delayed_vio_yaw_scale);
        sys->set_vio_global_yaw_oc_alpha(delayed_vio_yaw_alpha);
        delayed_vio_yaw_control_applied = true;
        PRINT_INFO(CYAN "[ros-free] delayed VIO yaw control ENABLED at t=%.3f (dt_init=%.1fs): mode=%s scale=%.3f alpha=%.3f\n" RESET,
                   t_cam, t_cam - t_init_done, delayed_vio_yaw_mode.c_str(),
                   delayed_vio_yaw_scale, delayed_vio_yaw_alpha);
      }
      if (!timed_yaw_switch_applied && t_cam >= args.vio_yaw_switch_time) {
        sys->set_vio_yaw_update_mode(args.vio_yaw_switch_mode);
        if (!std::isnan(args.vio_yaw_switch_alpha))
          sys->set_vio_global_yaw_oc_alpha(args.vio_yaw_switch_alpha);
        timed_yaw_switch_applied = true;
        PRINT_INFO(CYAN "[ros-free] TIMED YAW SWITCH at t=%.3f -> mode=%s alpha=%.3f\n" RESET,
                   t_cam, args.vio_yaw_switch_mode.c_str(),
                   std::isnan(args.vio_yaw_switch_alpha) ? -1.0 : args.vio_yaw_switch_alpha);
      }

      // [中文] GPS 高度 EKF 更新: 仅在 --gps-alt-update 开启且加载了 GPS 数据时
      // 策略: 初始化后第一个靠近 t_cam 的 GPS 采样 -> 设为 gps_alt_ref.
      // 之后每帧 cam update 后, 把 (最近一条 GPS alt - gps_alt_ref) 馈入滤波器.
      double t_gps = -1.0;
      Eigen::Vector3d xyz_raw = Eigen::Vector3d::Zero();
      bool gps_sample_near = false;
      if (!gps.empty()) {
        while (gps_i + 1 < gps.size() && gps[gps_i + 1].timestamp <= t_cam)
          gps_i++;
        t_gps = gps[gps_i].timestamp;
        xyz_raw = gps[gps_i].xyz;
        gps_sample_near = std::fabs(t_gps - t_cam) < 0.10;
        if (gps_sample_near && static_cast<long long>(gps_i) != last_diag_gps_i) {
          last_diag_gps_i = static_cast<long long>(gps_i);
          cur_gps_xyz = xyz_raw;
          if (prev_gps_t > 0.0) {
            double dt_gps = t_gps - prev_gps_t;
            if (dt_gps > 1e-6) {
              double dxyz = (xyz_raw - prev_gps_xyz).norm();
              cur_gps_speed = dxyz / dt_gps;
              gps_path_len += dxyz;
            }
          }
          prev_gps_xyz = xyz_raw;
          prev_gps_t = t_gps;
        }
      }
      if (args.gps_alt_update && gps_sample_near) {
        // advance gps_i 到 <= t_cam 的最新一条
        // [中文] Hold-out: 超过 cutoff 之后不再 feed (验证 VIO 是否真的被 GPS 纠正过 bias/scale)
        bool cutoff_reached = (args.gps_cutoff_time > 0.0 && t_cam > args.gps_cutoff_time);
        // [中文] Feed-every 降采样 (对齐之后使用): 只每 1/gps_feed_every 个采样 feed 一次
        bool feed_this_sample = true;
        if (args.gps_feed_every < 1.0 && args.gps_feed_every > 0.0) {
          int stride = static_cast<int>(std::round(1.0 / args.gps_feed_every));
          feed_this_sample = (static_cast<int>(gps_i) % stride == 0);
        }
        bool gps_sample_is_new = (static_cast<long long>(gps_i) != last_fed_gps_i);
        if (!cutoff_reached && feed_this_sample && gps_sample_is_new) {
          last_fed_gps_i = static_cast<long long>(gps_i);
          // XY remains evaluation-only. Align its origin to the VIO position at
          // the first fused GPS-Z sample, then expose it only to coupled-update
          // diagnostics (it is never inserted into H or the residual).
          if (!gps_xy_ref.allFinite()) {
            gps_xy_ref = xyz_raw.head<2>();
            gps_xy_vio_ref = sys->get_state()->_imu->pos().head<2>();
          }
          sys->set_gps_alt_xy_diagnostic_reference(
              t_gps, xyz_raw.head<2>() - gps_xy_ref + gps_xy_vio_ref, true);
          if (args.gps_alt_ground_plane) {
            // Ground-plane pseudo-rangefinder mode.  zonly is controlled by
            // --gps-alt-zonly (default false: full-state update).
            double alt_raw = xyz_raw(2);
            sys->feed_measurement_gps_ground_plane(t_gps, alt_raw, args.gps_alt_sigma,
                                                   args.gps_alt_zonly);
          } else if (args.gps_alt_relative) {
            // [中文] 相对高度模式: VioManager 管理 bootstrap (gps_ref / vio_ref)
            // 这里直接喂原始 ENU z, 不预设参考。
            double alt_raw = xyz_raw(2);
            sys->feed_measurement_gps_altitude_relative(t_gps, alt_raw, args.gps_alt_sigma,
                                                         args.gps_alt_chi2, args.gps_alt_schmidt,
                                                         args.gps_alt_also_vz);
          } else {
            // [中文] ALT-only 模式: 第一帧 GPS+VIO 同时 bootstrap,
            // measurement = GPS_now - GPS_ref + VIO_ref, 对齐到 VIO 世界坐标系.
            if (std::isnan(gps_alt_ref)) {
              gps_alt_ref = xyz_raw(2);
              auto state_ref = sys->get_state();
              gps_alt_vio_ref = state_ref->_imu->pos()(2);
              PRINT_INFO(GREEN "[GPS-ALT]: bootstrap gps_ref=%.2f vio_ref=%.2f at t=%.3f\n" RESET,
                         gps_alt_ref, gps_alt_vio_ref, t_gps);
            }
            double alt_z = xyz_raw(2) - gps_alt_ref + gps_alt_vio_ref;
            sys->feed_measurement_gps_altitude(t_gps, alt_z, args.gps_alt_sigma, args.gps_alt_chi2,
                                               args.gps_alt_schmidt, args.gps_alt_also_vz,
                                               args.gps_alt_range_mode);
          }
          gps_alt_feeds++;
          // Push GPS diagnostic snapshot to dashboard
          const auto &gd = sys->get_last_gps_alt_update();
          if (gd.t > 0) {
            dash.update_gps_alt_diag(t_cam, gd.gps_z, gd.vio_z,
                                     gd.residual, gd.pzz, gd.kpz);
          }
        }
      }
      auto state = sys->get_state();
      Eigen::Matrix3d R_wi = state->_imu->Rot().transpose(); // [中文] _imu->Rot() 是 R_ItoG^T = R_GtoI
      // OpenVINS 约定: _imu->Rot() 返回 R_GtoI (global->imu). 作图要 R_ItoG = R_GtoI^T
      // 这里 R_wi 即 R_ItoG (world = G frame)
      Eigen::Vector3d p_wi = state->_imu->pos();
      Eigen::Vector3d v_wi = state->_imu->vel();

      vio_for_align.push_back({t_cam, p_wi});
      while (vio_for_align.size() > 5000)
        vio_for_align.pop_front();
      maybe_align(t_cam);

      // output TUM (unaligned, VIO frame)
      Eigen::Matrix3d Rwi = R_wi;
      Eigen::Quaterniond q(Rwi);
      out << t_cam << ' ' << p_wi.x() << ' ' << p_wi.y() << ' ' << p_wi.z() << ' ' << q.x() << ' '
          << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';

      // [中文] bias debug log
      Eigen::Vector3d bg = state->_imu->bias_g();
      Eigen::Vector3d ba = state->_imu->bias_a();
      debug_out << t_cam << ' ' << v_wi.x() << ' ' << v_wi.y() << ' ' << v_wi.z() << ' '
                << std::scientific << std::setprecision(9)
                << bg.x() << ' ' << bg.y() << ' ' << bg.z() << ' '
                << ba.x() << ' ' << ba.y() << ' ' << ba.z() << '\n'
                << std::defaultfloat;

      // backend diagnostic: MSCKF chi2 rejection stats
      auto mstats = sys->get_last_msckf_stats();
      dash.update_backend_diag(t_cam, mstats.n_chi2_rejected, mstats.n_accepted);

      // ---- Diagnostic trigger + printout ----
      // Hoisted out of the block so they are visible in the terminal panel block below.
      int slam_count = (int)sys->get_features_SLAM().size();
      int slam_dropped = std::max(0, prev_slam_count - slam_count);
      prev_slam_count = slam_count;
      if (state_safety_out.is_open() && do_cam_feed) {
        Eigen::MatrixXd P = StateHelper::get_full_covariance(state);
        const int cov_dim = (int)P.rows();
        int cov_nan_count = 0;
        int cov_inf_count = 0;
        int cov_neg_diag_count = 0;
        double cov_min_diag = std::numeric_limits<double>::quiet_NaN();
        double cov_max_diag = std::numeric_limits<double>::quiet_NaN();
        double cov_trace = std::numeric_limits<double>::quiet_NaN();
        double cov_sym_fro = std::numeric_limits<double>::quiet_NaN();
        double cov_sym_max_abs = std::numeric_limits<double>::quiet_NaN();
        double cov_min_eig = std::numeric_limits<double>::quiet_NaN();
        if (cov_dim > 0) {
          cov_min_diag = std::numeric_limits<double>::infinity();
          cov_max_diag = -std::numeric_limits<double>::infinity();
          cov_trace = 0.0;
          for (int r = 0; r < P.rows(); r++) {
            const double d = P(r, r);
            if (std::isfinite(d)) {
              cov_min_diag = std::min(cov_min_diag, d);
              cov_max_diag = std::max(cov_max_diag, d);
              cov_trace += d;
              if (d < 0.0)
                cov_neg_diag_count++;
            } else if (std::isnan(d)) {
              cov_nan_count++;
            } else {
              cov_inf_count++;
            }
            for (int c = 0; c < P.cols(); c++) {
              const double v = P(r, c);
              if (std::isnan(v)) cov_nan_count++;
              else if (!std::isfinite(v)) cov_inf_count++;
            }
          }
          Eigen::MatrixXd Psym = P - P.transpose();
          cov_sym_fro = Psym.norm();
          cov_sym_max_abs = Psym.cwiseAbs().maxCoeff();
          if (args.state_safety_eig_every > 0 &&
              ((frame_idx + 1) % args.state_safety_eig_every) == 0 &&
              P.allFinite()) {
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(
                0.5 * (P + P.transpose()), Eigen::EigenvaluesOnly);
            if (es.info() == Eigen::Success && es.eigenvalues().rows() > 0)
              cov_min_eig = es.eigenvalues().minCoeff();
          }
        }
        int n_aruco = 0;
        int n_should_marg = 0;
        int n_invalid_landmark_id = 0;
        for (const auto &kv : state->_features_SLAM) {
          const auto &lm = kv.second;
          if ((int)kv.first <= 4 * state->_options.max_aruco_features)
            n_aruco++;
          if (lm && lm->should_marg)
            n_should_marg++;
          if (!lm || lm->_featid != kv.first || lm->id() < 0 ||
              lm->id() + lm->size() > cov_dim)
            n_invalid_landmark_id++;
        }
        state_safety_out << std::fixed << std::setprecision(9)
            << t_cam << "," << (frame_idx + 1) << ","
            << cov_dim << "," << state->_clones_IMU.size() << ","
            << state->_features_SLAM.size() << "," << n_aruco << ","
            << n_should_marg << "," << n_invalid_landmark_id << ","
            << (P.allFinite() ? 1 : 0) << "," << cov_nan_count << "," << cov_inf_count << ","
            << cov_sym_fro << "," << cov_sym_max_abs << ","
            << cov_min_diag << "," << cov_max_diag << "," << cov_trace << ","
            << cov_neg_diag_count << "," << cov_min_eig << ","
            << std::scientific << std::setprecision(9)
            << bg.x() << "," << bg.y() << "," << bg.z() << ","
            << ba.x() << "," << ba.y() << "," << ba.z() << ","
            << v_wi.x() << "," << v_wi.y() << "," << v_wi.z() << ","
            << p_wi.x() << "," << p_wi.y() << "," << p_wi.z() << ","
            << q.x() << "," << q.y() << "," << q.z() << "," << q.w()
            << std::defaultfloat << "\n";
      }
      {

        // Trigger on first frame with enough chi2 rejections.
        // The existing verbose per-frame dump fires unconditionally.
        // The event log entry is additive — it does not replace the dump.
        if (!diag_triggered && mstats.n_chi2_rejected >= args.diag_chi2_trigger) {
          diag_triggered = true;
          diag_frames_left = args.diag_window;
          PRINT_INFO(CYAN "\n[DIAG] *** First chi2 wave triggered at t=%.3f (chi2_rej=%d >= threshold=%d) ***\n\n" RESET,
                     t_cam, mstats.n_chi2_rejected, args.diag_chi2_trigger);
          if (diag_logger.event_ok()) {
            DiagMetrics _em;
            _em.t = t_cam; _em.frame_id = frame_idx + 1;
            _em.chi2_rej = mstats.n_chi2_rejected; _em.chi2_acc = mstats.n_accepted;
            _em.vio_speed = v_wi.norm(); _em.ba_norm = ba.norm(); _em.bg_norm = bg.norm();
            _em.slam_count = slam_count; _em.gps_speed = cur_gps_speed;
            char detail[256];
            snprintf(detail, sizeof(detail),
                     "ba_norm=%.3f bg_norm=%.4f slam=%d", ba.norm(), bg.norm(), slam_count);
            diag_logger.log_event(DiagLogger::EventType::CHI2_SPIKE, _em,
                                  "chi2 reject count reached threshold", detail);
          }
        }

        if (diag_frames_left > 0) {
          diag_frames_left--;

          if (do_cam_feed) { // only print DIAG for frames actually fed to VIO

          // --- Timing diagnostics ---
          double dt_cam = (prev_t_cam > 0) ? (t_cam - prev_t_cam) : -1.0;
          int n_imu_this = (int)imu_i - (int)imu_i_prev_cam;
          double imu_dt_min = 1e9, imu_dt_max = 0.0, imu_dt_sum = 0.0;
          int imu_dt_count = 0;
          bool imu_gap_detected = false, imu_backward = false;
          const double nominal_imu_dt = 1.0 / 200.0; // 200 Hz
          const double nominal_cam_dt = std::max(1, args.cam_subsample) / 31.0; // accounts for subsampling
          for (size_t k = imu_i_prev_cam; k + 1 < imu_i && k + 1 < imu.size(); ++k) {
            double d = imu[k + 1].timestamp - imu[k].timestamp;
            imu_dt_min = std::min(imu_dt_min, d);
            imu_dt_max = std::max(imu_dt_max, d);
            imu_dt_sum += d;
            imu_dt_count++;
            if (d > 2.0 * nominal_imu_dt) imu_gap_detected = true;
            if (d <= 0) imu_backward = true;
          }
          double imu_dt_mean = imu_dt_count > 0 ? imu_dt_sum / imu_dt_count : 0.0;
          bool cam_gap = dt_cam > 0 && dt_cam > 1.5 * nominal_cam_dt;

          PRINT_INFO(CYAN "[DIAG-TIMING] t=%.3f  dt_cam=%.4f%s  n_imu=%d  imu_dt[min/max/mean]=%.5f/%.5f/%.5f%s%s\n" RESET,
                     t_cam,
                     dt_cam, cam_gap ? "(!GAP)" : "",
                     n_imu_this,
                     imu_dt_count > 0 ? imu_dt_min : 0.0,
                     imu_dt_count > 0 ? imu_dt_max : 0.0,
                     imu_dt_mean,
                     imu_gap_detected ? " IMU_GAP" : "",
                     imu_backward ? " IMU_BACKWARD" : "");

          // --- Most recent IMU sample at this camera frame ---
          Eigen::Vector3d gyro_now(0, 0, 0), accel_now(0, 0, 0);
          if (imu_i > 0 && imu_i <= imu.size()) {
            gyro_now  = imu[imu_i - 1].gyro;
            accel_now = imu[imu_i - 1].accel;
          }
          double gyro_norm = gyro_now.norm();
          double accel_norm = accel_now.norm();
          // Approximate roll/pitch/yaw rates (body frame angular velocity)
          PRINT_INFO(CYAN "[DIAG-MOTION] gyro=[%.3f,%.3f,%.3f] |g|=%.3f  accel=[%.3f,%.3f,%.3f] |a|=%.3f\n" RESET,
                     gyro_now.x(), gyro_now.y(), gyro_now.z(), gyro_norm,
                     accel_now.x(), accel_now.y(), accel_now.z(), accel_norm);

          // --- State diagnostics ---
          auto st = sys->get_state();
          Eigen::Vector3d bg_now = st->_imu->bias_g();
          Eigen::Vector3d ba_now = st->_imu->bias_a();
          int n_clones = (int)st->_clones_IMU.size();
          PRINT_INFO(CYAN "[DIAG-STATE]  bg=[%.4f,%.4f,%.4f]  ba=[%.3f,%.3f,%.3f]  |v|=%.3f  n_clones=%d\n" RESET,
                     bg_now.x(), bg_now.y(), bg_now.z(),
                     ba_now.x(), ba_now.y(), ba_now.z(),
                     st->_imu->vel().norm(), n_clones);
          {
            ov_core::TrackerWarpVizPacket pkt_diag;
            bool has_pkt = sys->get_warp_viz_packet(0, pkt_diag);
            int klt_attempted = has_pkt ? pkt_diag.n_klt_attempted : -1;
            int klt_good = has_pkt ? (int)pkt_diag.curr_pts_raw.size() : -1;
            int klt_new = has_pkt ? pkt_diag.n_newly_detected : -1;
            int db_count = sys->get_feature_database_size();
            int msckf_viz_count = (int)sys->get_good_features_MSCKF().size();
            PRINT_INFO(CYAN "[DIAG-VISION] klt_attempted=%d klt_good=%d klt_new=%d db=%d msckf_viz=%d slam=%d\n" RESET,
                       klt_attempted, klt_good, klt_new, db_count, msckf_viz_count, slam_count);
          }
          {
            std::ostringstream clones_ss;
            clones_ss << std::fixed << std::setprecision(3);
            for (const auto &cl : st->_clones_IMU)
              clones_ss << " " << cl.first;
            PRINT_INFO(CYAN "[DIAG-CLONES] clone_times:%s\n" RESET, clones_ss.str().c_str());
          }

          // --- MSCKF residual diagnostics ---
          double chi2_mean_rej = mstats.n_chi2_rejected > 0 ? mstats.chi2_sum_rej / mstats.n_chi2_rejected : 0.0;
          double chi2_mean_acc = mstats.n_accepted > 0 ? mstats.chi2_sum_acc / mstats.n_accepted : 0.0;
          double trk_mean_acc  = mstats.n_accepted > 0 ? (double)mstats.track_len_sum_acc / mstats.n_accepted : 0.0;
          double trk_mean_rej  = mstats.n_chi2_rejected > 0 ? (double)mstats.track_len_sum_rej / mstats.n_chi2_rejected : 0.0;
          PRINT_INFO(CYAN "[DIAG-MSCKF]  n_in=%d  n_tri_fail=%d  n_chi2_rej=%d  n_acc=%d\n" RESET,
                     mstats.n_features_in, mstats.n_tri_failed, mstats.n_chi2_rejected, mstats.n_accepted);
          PRINT_INFO(CYAN "[DIAG-MSCKF]  chi2_rej[mean/max]=%.1f/%.1f  chi2_acc[mean/max]=%.1f/%.1f\n" RESET,
                     chi2_mean_rej, mstats.chi2_max_rej, chi2_mean_acc, mstats.chi2_max_acc);
          PRINT_INFO(CYAN "[DIAG-MSCKF]  trk_acc[mean/max]=%.1f/%d  trk_rej[mean/max]=%.1f/%d\n" RESET,
                     trk_mean_acc, mstats.track_len_max_acc, trk_mean_rej, mstats.track_len_max_rej);

          // --- Triangulation rejection breakdown ---
          {
            PRINT_INFO(CYAN "[DIAG-TRI]    DLT:  cond_bad=%d  depth_near=%d  depth_far=%d  nan=%d\n" RESET,
                       mstats.tri_cond_bad, mstats.tri_depth_near, mstats.tri_depth_far, mstats.tri_nan);
            PRINT_INFO(CYAN "[DIAG-TRI]    GN:   depth_near=%d  depth_far=%d  base_ratio=%d  nan=%d\n" RESET,
                       mstats.tri_gn_depth_near, mstats.tri_gn_depth_far,
                       mstats.tri_gn_baseline_ratio, mstats.tri_gn_nan);
            if (mstats.tri_n_geom > 0) {
              double dmean = mstats.tri_depth_sum / mstats.tri_n_geom;
              double bmean = mstats.tri_base_sum  / mstats.tri_n_geom;
              double rmean = mstats.tri_ratio_sum / mstats.tri_n_geom;
              double cmean = mstats.tri_cond_sum  / mstats.tri_n_geom;
              PRINT_INFO(CYAN "[DIAG-TRI]    geo(n=%d): depth[mn/mx]=%.1f/%.1f  base[mn/mx]=%.2f/%.2f"
                         "  ratio[mn/mx]=%.0f/%.0f  cond[mn/mx]=%.0f/%.0f\n" RESET,
                         mstats.tri_n_geom, dmean, mstats.tri_depth_max, bmean, mstats.tri_base_max,
                         rmean, mstats.tri_ratio_max, cmean, mstats.tri_cond_max);
            } else {
              PRINT_INFO(CYAN "[DIAG-TRI]    geo: no features passed triangulation\n" RESET);
            }
          }

          // --- SLAM / coincidence checks ---
          double cam_toff_now = st->_options.do_calib_camera_timeoffset
                                  ? st->_calib_dt_CAMtoIMU->value()(0) : params.calib_camimu_dt;
          PRINT_INFO(CYAN "[DIAG-SLAM]   n_slam=%d  slam_dropped=%d  cam_toff=%.6f\n\n" RESET,
                     slam_count, slam_dropped, cam_toff_now);
        }
      }

      // dashboard data
      dash.update_vio_pose(t_cam, R_wi, p_wi, v_wi);
      dash.update_biases(t_cam, bg, ba); // bg/ba read from state above; always from live EKF
      std::vector<Eigen::Vector3d> slam_pts = sys->get_features_SLAM();
      std::vector<Eigen::Vector3d> msckf_pts = sys->get_good_features_MSCKF();
      dash.update_features(slam_pts, msckf_pts);

      // ---- DiagMetrics collection + optional print/CSV ----
      {
        // VIO path length accumulation
        if (prev_p_wi_valid)
          vio_path_len += (p_wi - prev_p_wi).norm();
        prev_p_wi = p_wi;
        prev_p_wi_valid = true;

        // Tracker counts from the warp viz packet (KLT fields and descriptor fields are disjoint)
        int klt_raw_now = 0, tracked_now = 0;
        int desc_detected_now = 0, desc_pre_gate_now = 0, desc_post_gate_now = 0, desc_tracked_now = 0;
        {
          ov_core::TrackerWarpVizPacket pkt_m;
          if (sys->get_warp_viz_packet(0, pkt_m)) {
            klt_raw_now        = pkt_m.n_klt_attempted;
            tracked_now        = (int)pkt_m.curr_pts_raw.size();
            desc_detected_now  = pkt_m.n_desc_detected;
            desc_pre_gate_now  = pkt_m.n_desc_pre_gate;
            desc_post_gate_now = pkt_m.n_desc_post_gate;
            desc_tracked_now   = pkt_m.n_desc_post_ransac;
          }
        }

        // Feature depths: L2 distance from VIO pos to each SLAM feature
        double depth_med = -1, depth_max = -1;
        if (!slam_pts.empty()) {
          std::vector<double> depths;
          depths.reserve(slam_pts.size());
          for (const auto &fp : slam_pts)
            depths.push_back((fp - p_wi).norm());
          std::sort(depths.begin(), depths.end());
          depth_med = depths[depths.size() / 2];
          depth_max = depths.back();
        }

        // Camera calibration (cam 0)
        double fx = 0, fy = 0, cx_c = 0, cy_c = 0;
        double ext_tx = 0, ext_ty = 0, ext_tz = 0;
        if (!state->_cam_intrinsics.empty()) {
          auto intr = state->_cam_intrinsics.at(0)->value();
          fx = intr(0); fy = intr(1); cx_c = intr(2); cy_c = intr(3);
        }
        if (!state->_calib_IMUtoCAM.empty()) {
          Eigen::Vector3d et = state->_calib_IMUtoCAM.at(0)->pos();
          ext_tx = et(0); ext_ty = et(1); ext_tz = et(2);
        }
        double cam_toff = state->_options.do_calib_camera_timeoffset
                            ? state->_calib_dt_CAMtoIMU->value()(0)
                            : params.calib_camimu_dt;

        // Build DiagMetrics snapshot
        DiagMetrics m;
        m.t             = t_cam;
        m.frame_id      = frame_idx + 1;
        m.run_name      = args.run_name;
        m.mode_str      = diag_mode_str;
        m.init_source   = diag_init_source;
        m.initialized   = true;
        m.aligned       = aligner.solved();
        m.klt_raw       = klt_raw_now;
        m.tracked       = tracked_now;
        m.desc_detected  = desc_detected_now;
        m.desc_pre_gate  = desc_pre_gate_now;
        m.desc_post_gate = desc_post_gate_now;
        m.desc_tracked   = desc_tracked_now;
        m.n_acc         = mstats.n_accepted;
        m.msckf_in      = mstats.n_features_in;
        m.slam_count    = slam_count;
        m.first_accept_t = first_accept_t;
        m.first_slam_t  = first_slam_t;
        m.vio_speed     = v_wi.norm();
        m.gps_speed     = cur_gps_speed;
        m.vio_path_len  = vio_path_len;
        m.gps_path_len  = gps_path_len;
        m.chi2_acc      = mstats.n_accepted;
        m.chi2_rej      = mstats.n_chi2_rejected;
        m.depth_med     = depth_med;
        m.depth_max     = depth_max;
        m.ba_norm       = ba.norm();
        m.bg_norm       = bg.norm();
        m.cam_toff      = cam_toff;
        m.vio_dist      = vio_path_len;
        m.vio_x = p_wi.x(); m.vio_y = p_wi.y(); m.vio_z = p_wi.z();
        m.vio_vx = v_wi.x(); m.vio_vy = v_wi.y(); m.vio_vz = v_wi.z();
        m.gps_x = cur_gps_xyz.x(); m.gps_y = cur_gps_xyz.y(); m.gps_z = cur_gps_xyz.z();
        m.bgx = bg.x(); m.bgy = bg.y(); m.bgz = bg.z();
        m.bax = ba.x(); m.bay = ba.y(); m.baz = ba.z();
        m.fx = fx; m.fy = fy; m.cx = cx_c; m.cy = cy_c;
        m.ext_tx = ext_tx; m.ext_ty = ext_ty; m.ext_tz = ext_tz;

        // ---- One-shot event detection (additive; does not affect existing prints) ----
        if (!event_init_fired) {
          event_init_fired = true;
          char detail[256];
          snprintf(detail, sizeof(detail),
                   "v_vio=%.2f v_gps=%.2f ba_norm=%.3f bg_norm=%.4f",
                   m.vio_speed, m.gps_speed, m.ba_norm, m.bg_norm);
          diag_logger.log_event(DiagLogger::EventType::INIT, m,
                                "VIO filter initialized", detail);
        }
        if (!event_first_accept_fired && mstats.n_accepted > 0) {
          event_first_accept_fired = true;
          first_accept_t = t_cam;
          m.first_accept_t = first_accept_t;
          char detail[256];
          snprintf(detail, sizeof(detail),
                   "n_acc=%d slam=%d v_vio=%.2f", m.n_acc, m.slam_count, m.vio_speed);
          diag_logger.log_event(DiagLogger::EventType::FIRST_ACCEPT, m,
                                "first MSCKF features accepted", detail);
        }
        if (!event_first_slam_fired && !slam_pts.empty()) {
          event_first_slam_fired = true;
          first_slam_t = t_cam;
          m.first_slam_t = first_slam_t;
          char detail[256];
          snprintf(detail, sizeof(detail),
                   "slam=%d n_acc=%d v_vio=%.2f depth_med=%.1f",
                   m.slam_count, m.n_acc, m.vio_speed, m.depth_med);
          diag_logger.log_event(DiagLogger::EventType::FIRST_SLAM, m,
                                "first SLAM features in state", detail);
        }
        // SLAM_DROP: large sudden loss of SLAM features
        {
          int slam_was = slam_count + slam_dropped;
          if (slam_dropped >= 3 && slam_was >= 3) {
            char detail[256];
            snprintf(detail, sizeof(detail),
                     "dropped=%d was=%d now=%d chi2_rej=%d",
                     slam_dropped, slam_was, slam_count, mstats.n_chi2_rejected);
            diag_logger.log_event(DiagLogger::EventType::SLAM_DROP, m,
                                  "large SLAM feature drop", detail);
          }
        }

        // ---- Optional compact terminal print ----
        if (args.diag_print && (frame_idx % std::max(1, args.diag_print_every) == 0))
          DiagPrinter::print(m);

        // ---- Per-frame CSV row ----
        diag_logger.write_row(m);
      } // end DiagMetrics block
          } // end do_cam_feed check

      // latest GT sample (nearest within 0.05s)
      if (!gt_pos_map.empty()) {
        auto it = gt_pos_map.lower_bound(t_cam);
        auto best = gt_pos_map.end();
        double bd = 0.05;
        if (it != gt_pos_map.end() && std::fabs(it->first - t_cam) < bd) {
          best = it;
          bd = std::fabs(it->first - t_cam);
        }
        if (it != gt_pos_map.begin() && std::fabs(std::prev(it)->first - t_cam) < bd) {
          best = std::prev(it);
        }
        if (best != gt_pos_map.end())
          dash.update_gt(best->first, best->second);
      }
      if (!gps_pos_map.empty()) {
        auto it = gps_pos_map.lower_bound(t_cam);
        auto best = gps_pos_map.end();
        double bd = 0.1;
        if (it != gps_pos_map.end() && std::fabs(it->first - t_cam) < bd) {
          best = it;
          bd = std::fabs(it->first - t_cam);
        }
        if (it != gps_pos_map.begin() && std::fabs(std::prev(it)->first - t_cam) < bd) {
          best = std::prev(it);
        }
        if (best != gps_pos_map.end())
          dash.update_gt(best->first, best->second);
      }
    }

    // image for right-top panel
    cv::Mat hist = sys->get_historical_viz_image();
    if (!hist.empty())
      dash.update_image(t_cam, hist);
    else
      dash.update_image(t_cam, img0);
    // Tracker-owned warp viz packet → cross-pane matching view + diagnostic counts
    {
      ov_core::TrackerWarpVizPacket pkt;
      if (sys->get_warp_viz_packet(0, pkt)) {
        dash.update_tracker_flow(pkt.prev_image_for_viz, pkt.prev_pts_for_viz,
                                 pkt.curr_pts_raw, pkt.curr_raw_image,
                                 pkt.warp_active, pkt.t_curr);
        dash.update_tracker_diag(pkt.t_curr, pkt.n_klt_attempted, pkt.n_newly_detected);
      }
    }

    // [中文] cam-only 视频: 拿 VioManager 的历史 viz 图 (stereo panel + TrackBase
    // 历史轨迹叠加), 加一行时间戳文字, 写入 MP4. 帧尺寸在首帧固化.
    if (!args.video_cam_path.empty() && !hist.empty()) {
      cv::Mat cam_frame;
      if (hist.channels() == 1)
        cv::cvtColor(hist, cam_frame, cv::COLOR_GRAY2BGR);
      else
        cam_frame = hist.clone();
      // 顶部状态条
      std::ostringstream os;
      os << "t=" << std::fixed << std::setprecision(2) << t_cam;
      if (sys->initialized()) {
        auto st = sys->get_state();
        os << "  alt=" << std::fixed << std::setprecision(1) << st->_imu->pos()(2) << "m"
           << "  |v|=" << std::fixed << std::setprecision(2) << st->_imu->vel().norm() << "m/s";
        os << "  init=ok";
      } else {
        os << "  init=waiting";
      }
      cv::putText(cam_frame, os.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                  cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      if (!cam_writer_init) {
        cam_video_size = cam_frame.size();
        int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        cam_writer.open(args.video_cam_path, fourcc, args.video_fps, cam_video_size, true);
        if (!cam_writer.isOpened()) {
          PRINT_WARNING(YELLOW "[ros-free] failed to open cam video writer: %s\n" RESET,
                        args.video_cam_path.c_str());
        } else {
          PRINT_INFO(GREEN "[ros-free] writing cam video to %s @ %d fps, size %dx%d\n" RESET,
                     args.video_cam_path.c_str(), args.video_fps, cam_video_size.width,
                     cam_video_size.height);
        }
        cam_writer_init = true;
      }
      if (cam_writer.isOpened()) {
        if (cam_frame.size() != cam_video_size)
          cv::resize(cam_frame, cam_frame, cam_video_size);
        cam_writer.write(cam_frame);
      }
    }

    // Update diagnostic tracking for next frame (only for frames actually fed to VIO)
    if (do_cam_feed) {
      prev_t_cam = t_cam;
      imu_i_prev_cam = imu_i;
    }

    frame_idx++;
    if (frame_idx % std::max(1, args.dash_every) == 0) {
      if (!dash.render_and_show(1)) {
        PRINT_INFO(YELLOW "[ros-free] user requested quit\n" RESET);
        g_stop.store(true);
      }
    }
    if (args.verbose_timing) {
      PRINT_INFO(CYAN "[ros-free] frame=%d t=%.3f feed_ms=%.1f init=%d\n" RESET, frame_idx, t_cam,
                 dt_ms, (int)sys->initialized());
    }

    cam_i++;
  }

  out.close();
  if (!args.camera_stride_audit_path.empty()) {
    std::vector<double> processed_dt;
    for (size_t i = 1; i < processed_image_timestamps.size(); ++i)
      processed_dt.push_back(processed_image_timestamps[i] - processed_image_timestamps[i - 1]);
    auto percentile = [](std::vector<double> values, double q) -> double {
      if (values.empty())
        return std::numeric_limits<double>::quiet_NaN();
      std::sort(values.begin(), values.end());
      const double pos = q * (double)(values.size() - 1);
      const size_t lo = (size_t)std::floor(pos);
      const size_t hi = std::min(values.size() - 1, lo + 1);
      const double a = pos - (double)lo;
      return (1.0 - a) * values[lo] + a * values[hi];
    };
    const double dt_median = percentile(processed_dt, 0.50);
    const double dt_p95 = percentile(processed_dt, 0.95);
    const double processed_ratio = total_image_input_count > 0
        ? (double)processed_image_count / (double)total_image_input_count
        : 0.0;
    const auto &visual_counts = sys->get_visual_update_counters();
    const auto &gps_stats = sys->get_gps_alt_stats();

    fs::path audit_path(args.camera_stride_audit_path);
    if (audit_path.has_parent_path())
      fs::create_directories(audit_path.parent_path());
    std::ofstream audit(args.camera_stride_audit_path, std::ofstream::out | std::ofstream::trunc);
    if (!audit.is_open()) {
      PRINT_WARNING(YELLOW "[ros-free] failed to open camera stride audit: %s\n" RESET,
                    args.camera_stride_audit_path.c_str());
    } else {
      audit << "configured_stride,total_image_input_count,processed_image_count,"
            << "skipped_image_count,effective_processed_ratio,processed_image_dt_median,"
            << "processed_image_dt_p95,total_imu_sample_count,tracker_invocation_count,"
            << "msckf_update_count,regular_slam_update_count,delayed_slam_update_count,"
            << "gps_z_update_count,gps_z_accepted_count,gps_z_rejected_count,"
            << "visual_update_stride,visual_update_frame_count,visual_update_eligible_count,"
            << "visual_update_skipped_count,feature_database_insert_count,"
            << "visual_tracker_call_count,msckf_attempted,msckf_accepted,"
            << "msckf_features_attempted,msckf_features_accepted,"
            << "regular_slam_attempted,regular_slam_accepted,"
            << "regular_slam_features_attempted,regular_slam_features_accepted,"
            << "delayed_slam_attempted,delayed_slam_accepted,"
            << "delayed_slam_features_attempted,delayed_slam_features_accepted,"
            << "tracks_retained_final,tracks_retained_max,tracks_consumed_count,"
            << "tracks_dropped_without_update,"
            << "visual_update_adaptive_enabled,visual_update_adaptive_target_flow_px,"
            << "visual_update_adaptive_min_dt,visual_update_adaptive_max_dt,"
            << "visual_update_adaptive_min_tracks,visual_update_adaptive_keyframe_count,"
            << "visual_update_adaptive_skip_count,visual_update_adaptive_flow_trigger_count,"
            << "visual_update_adaptive_maxdt_trigger_count,visual_update_adaptive_track_trigger_count,"
            << "visual_update_adaptive_first_trigger_count,visual_update_adaptive_min_dt_block_count,"
            << "visual_update_adaptive_drop_current_observations_count,"
            << "visual_update_adaptive_last_dt_s,visual_update_adaptive_last_frame_flow_px,"
            << "visual_update_adaptive_last_accum_flow_px,visual_update_adaptive_last_track_count,"
            << "camera_frame_adaptive_enabled,camera_frame_adaptive_target_ratio,"
            << "camera_frame_adaptive_min_dt,camera_frame_adaptive_max_dt,"
            << "camera_frame_adaptive_min_depth,camera_frame_adaptive_min_tracks,"
            << "camera_frame_adaptive_feed_count,"
            << "camera_frame_adaptive_skip_count,camera_frame_adaptive_first_trigger_count,"
            << "camera_frame_adaptive_target_dt_trigger_count,camera_frame_adaptive_maxdt_trigger_count,"
            << "camera_frame_adaptive_track_safety_trigger_count,camera_frame_adaptive_no_geometry_feed_count,"
            << "camera_frame_adaptive_depth_deadzone_feed_count,"
            << "camera_frame_adaptive_fullrate_deadzone_feed_count,"
            << "camera_frame_adaptive_last_dt_since_feed,camera_frame_adaptive_last_desired_dt,"
            << "camera_frame_adaptive_last_depth_median,camera_frame_adaptive_last_speed,"
            << "camera_frame_adaptive_last_track_count,camera_frame_adaptive_last_reason,"
            << "processed_image_timestamps\n";
      audit << args.cam_subsample << ","
            << total_image_input_count << ","
            << processed_image_count << ","
            << skipped_image_count << ","
            << std::fixed << std::setprecision(9)
            << processed_ratio << ","
            << dt_median << ","
            << dt_p95 << ","
            << imu_sample_count << ","
            << processed_image_count << ","
            << visual_counts.msckf_update_count << ","
            << visual_counts.regular_slam_update_count << ","
            << visual_counts.delayed_slam_update_count << ","
            << gps_alt_feeds << ","
            << gps_stats.n_accepted << ","
            << gps_stats.n_rejected << ","
            << args.visual_update_stride << ","
            << visual_counts.visual_update_frame_count << ","
            << visual_counts.visual_update_eligible_count << ","
            << visual_counts.visual_update_skipped_count << ","
            << visual_counts.feature_database_insert_count << ","
            << visual_counts.tracker_call_count << ","
            << visual_counts.msckf_attempt_count << ","
            << visual_counts.msckf_accept_count << ","
            << visual_counts.msckf_features_attempted << ","
            << visual_counts.msckf_features_accepted << ","
            << visual_counts.regular_slam_attempt_count << ","
            << visual_counts.regular_slam_accept_count << ","
            << visual_counts.regular_slam_features_attempted << ","
            << visual_counts.regular_slam_features_accepted << ","
            << visual_counts.delayed_slam_attempt_count << ","
            << visual_counts.delayed_slam_accept_count << ","
            << visual_counts.delayed_slam_features_attempted << ","
            << visual_counts.delayed_slam_features_accepted << ","
            << visual_counts.tracks_retained_final << ","
            << visual_counts.tracks_retained_max << ","
            << visual_counts.tracks_consumed_count << ","
            << visual_counts.tracks_dropped_without_update << ","
            << visual_counts.visual_update_adaptive_enabled << ","
            << args.visual_update_adaptive_target_flow_px << ","
            << args.visual_update_adaptive_min_dt << ","
            << args.visual_update_adaptive_max_dt << ","
            << args.visual_update_adaptive_min_tracks << ","
            << visual_counts.visual_update_adaptive_keyframe_count << ","
            << visual_counts.visual_update_adaptive_skip_count << ","
            << visual_counts.visual_update_adaptive_flow_trigger_count << ","
            << visual_counts.visual_update_adaptive_maxdt_trigger_count << ","
            << visual_counts.visual_update_adaptive_track_trigger_count << ","
            << visual_counts.visual_update_adaptive_first_trigger_count << ","
            << visual_counts.visual_update_adaptive_min_dt_block_count << ","
            << visual_counts.visual_update_adaptive_drop_current_observations_count << ","
            << visual_counts.visual_update_adaptive_last_dt_s << ","
            << visual_counts.visual_update_adaptive_last_frame_flow_px << ","
            << visual_counts.visual_update_adaptive_last_accum_flow_px << ","
            << visual_counts.visual_update_adaptive_last_track_count << ","
            << camera_adaptive_stats.enabled << ","
            << args.camera_frame_adaptive_target_ratio << ","
            << args.camera_frame_adaptive_min_dt << ","
            << args.camera_frame_adaptive_max_dt << ","
            << args.camera_frame_adaptive_min_depth << ","
            << args.camera_frame_adaptive_min_tracks << ","
            << camera_adaptive_stats.feed_count << ","
            << camera_adaptive_stats.skip_count << ","
            << camera_adaptive_stats.first_trigger_count << ","
            << camera_adaptive_stats.target_dt_trigger_count << ","
            << camera_adaptive_stats.max_dt_trigger_count << ","
            << camera_adaptive_stats.track_safety_trigger_count << ","
            << camera_adaptive_stats.no_geometry_feed_count << ","
            << camera_adaptive_stats.depth_deadzone_feed_count << ","
            << camera_adaptive_stats.fullrate_deadzone_feed_count << ","
            << camera_adaptive_stats.last_dt_since_feed << ","
            << camera_adaptive_stats.last_desired_dt << ","
            << camera_adaptive_stats.last_depth_median << ","
            << camera_adaptive_stats.last_speed << ","
            << camera_adaptive_stats.last_track_count << ","
            << camera_adaptive_stats.last_reason << ",\"";
      for (size_t i = 0; i < processed_image_timestamps.size(); ++i) {
        if (i > 0)
          audit << ';';
        audit << std::fixed << std::setprecision(9) << processed_image_timestamps[i];
      }
      audit << "\"\n";
      PRINT_INFO(GREEN "[ros-free] camera stride audit written to %s "
                 "(stride=%d total=%zu processed=%zu skipped=%zu ratio=%.3f)\n" RESET,
                 args.camera_stride_audit_path.c_str(), args.cam_subsample,
                 total_image_input_count, processed_image_count, skipped_image_count,
                 processed_ratio);
    }
  }
  if (adaptive_stride_out.is_open()) {
    adaptive_stride_out.flush();
    if (!adaptive_stride_out.good()) {
      PRINT_ERROR(RED "[adaptive-stride] policy CSV write failed: %s\n" RESET,
                  args.adaptive_stride_log_path.c_str());
      return EXIT_FAILURE;
    }
    adaptive_stride_out.close();
    PRINT_INFO(GREEN "[adaptive-stride] log=%s switches=%zu up=%zu down=%zu final=%d\n" RESET,
               args.adaptive_stride_log_path.c_str(), adaptive_switch_count,
               adaptive_up_switch_count, adaptive_down_switch_count,
               adaptive_stride_controller.recommended_stride());
  }
  if (cam_writer.isOpened())
    cam_writer.release();
  PRINT_INFO(GREEN "[ros-free] done. trajectory written to %s (frames=%d, aligned_pairs=%d)\n" RESET,
             args.output_path.c_str(), frame_idx, align_fit_count);
  // Log end-of-run event
  if (diag_logger.event_ok()) {
    DiagMetrics _end;
    _end.t = (cam_i > 0 && cam_i <= cam0.size()) ? cam0[std::min(cam_i, cam0.size()) - 1].timestamp : -1;
    _end.frame_id = frame_idx;
    _end.vio_path_len = vio_path_len;
    _end.gps_path_len = gps_path_len;
    char detail[256];
    snprintf(detail, sizeof(detail),
             "frames=%d vio_path=%.1fm gps_path=%.1fm",
             frame_idx, vio_path_len, gps_path_len);
    diag_logger.log_event(DiagLogger::EventType::LANDING_OR_STOP, _end,
                          "dataset replay complete", detail);
  }

  // final GPS altitude / ground-plane statistics
  if (args.gps_alt_ground_plane) {
    if (sys->get_updater_gplane_range())
      sys->get_updater_gplane_range()->print_summary();
  } else if (args.gps_alt_update) {
    sys->print_gps_alt_final_summary();
  }
  if (args.gplane_feat_enable && sys->get_updater_gplane_feature()) {
    sys->get_updater_gplane_feature()->print_summary();
  }
  if (args.gplane_feat_v1_enable && sys->get_updater_gplane_feature_v1()) {
    sys->get_updater_gplane_feature_v1()->print_summary();
  }

  // final frame + hold if window
  dash.render_and_show(1);
  return EXIT_SUCCESS;
}
