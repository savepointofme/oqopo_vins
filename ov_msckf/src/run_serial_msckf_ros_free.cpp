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
  bool gps_alt_zonly = false;            // [中文] Z-only 模式: 只更新 p_z, 其他状态不全改, cov 只降 P_zz
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
  int dash_every = 1;            // [中文] 每 N 帧刷新仪表板
  bool viz_fast = false;         // [中文] 快速仪表板模式 (轨迹/曲线抽帧, 特征限 200)
  bool verbose_timing = false;
  bool no_vio_yaw_update = false; // alias for --vio-yaw-update-scale 0.0
  std::string vio_yaw_update_mode;
  double vio_yaw_update_scale = std::numeric_limits<double>::quiet_NaN();
  double vio_global_yaw_oc_alpha = std::numeric_limits<double>::quiet_NaN();
  double vio_yaw_control_start_after_init = 0.0; // seconds; 0 = apply requested yaw control immediately
  std::string vio_yaw_diag_path;           // per-visual-update yaw CSV
  std::string visual_obs_diag_path;        // VisualObservabilityPolicy per-update CSV
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
               "  --gps-alt-zonly       Only apply IMU p_z correction; zero all other corrections.\n"
               "                        Covariance: only reduce P_zz, cross-terms unchanged.\n"
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
               "  --dash-every N        Refresh dashboard every N camera frames (default 1)\n"
               "                        Recommended: --dash-every 5 with --viz-fast for live runs\n"
               "  --viz-fast            Fast dashboard mode: decimate curves/trajectories,\n"
               "                        cap SLAM/MSCKF features at 200, skip expensive resize.\n"
               "                        Keeps all panels. Reduces render time ~5-10x.\n"
               "  --verbose             Print per-frame timing\n"
               "  --no-vio-yaw-update   Alias for --vio-yaw-update-scale 0.0\n"
               "  --vio-yaw-update-mode M   original|per_block_scale|global_yaw_oc_projection|current_only_scale|hard_gyro_yaw|a_strict_yaw_dx0\n"
               "  --vio-yaw-update-scale S  Scale visual yaw correction for *_scale modes (1=orig, 0=off)\n"
               "  --vio-global-yaw-oc-alpha A  H-projection alpha for global_yaw_oc_projection (0=orig, 1=full)\n"
               "  --vio-yaw-control-start-after-init S  Delay requested visual yaw control until S seconds after init\n"
               "  --vio-yaw-diag PATH   Write per-MSCKF/SLAM yaw update CSV\n"
               "  --visual-obs-diag PATH  Write VisualObservabilityPolicy diagnostics CSV\n"
               "  Modes for --vio-yaw-update-mode (new pre-chi2 OC modes):\n"
               "    global_yaw_oc_fej_prechi2  1-D FEJ yaw OC applied before chi2 gating\n"
               "    visual_4d_oc_fej_prechi2   4-D FEJ (yaw+xyz) OC applied before chi2 gating\n"
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
    else if (s == "--gps-alt-zonly") a.gps_alt_zonly = true;
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
    else if (s == "--dash-every") a.dash_every = std::atoi(next("--dash-every").c_str());
    else if (s == "--viz-fast") a.viz_fast = true;
    else if (s == "--verbose") a.verbose_timing = true;
    else if (s == "--no-vio-yaw-update") a.no_vio_yaw_update = true;
    else if (s == "--vio-yaw-update-mode") a.vio_yaw_update_mode = next("--vio-yaw-update-mode");
    else if (s == "--vio-yaw-update-scale") a.vio_yaw_update_scale = std::atof(next("--vio-yaw-update-scale").c_str());
    else if (s == "--vio-global-yaw-oc-alpha") a.vio_global_yaw_oc_alpha = std::atof(next("--vio-global-yaw-oc-alpha").c_str());
    else if (s == "--vio-global-yaw-schmidt-alpha") a.vio_global_yaw_oc_alpha = std::atof(next("--vio-global-yaw-schmidt-alpha").c_str());
    else if (s == "--vio-yaw-control-start-after-init") a.vio_yaw_control_start_after_init = std::atof(next("--vio-yaw-control-start-after-init").c_str());
    else if (s == "--vio-yaw-diag") a.vio_yaw_diag_path = next("--vio-yaw-diag");
    else if (s == "--visual-obs-diag") a.visual_obs_diag_path = next("--visual-obs-diag");
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
  if (!args.init_from_fc_path.empty() && std::fabs(args.start_time - 930.0) > 1e-6) {
    PRINT_WARNING(YELLOW "[ros-free] --init-from-fc with start_time=%.3f (not 930); proceeding anyway\n" RESET, args.start_time);
  }

  // -------------------- load config --------------------
  auto parser = std::make_shared<ov_core::YamlParser>(args.config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
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
  auto sys = std::make_shared<VioManager>(params);
  bool delayed_vio_yaw_control_applied = !delayed_vio_yaw_control;

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
  if (args.gps_alt_zonly) {
    sys->set_gps_alt_zonly_update(true);
    PRINT_INFO(CYAN "[ros-free] GPS alt Z-only update mode ENABLED "
               "(only p_z correction, cov: only P_zz reduced)\n" RESET);
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

  while (!g_stop.load() && (imu_i < imu.size() || cam_i < cam0.size())) {
    double t_imu = imu_i < imu.size() ? imu[imu_i].timestamp : INF;
    double t_cam = cam_i < cam0.size() ? cam0[cam_i].timestamp : INF;

    if (t_imu <= t_cam) {
      ov_core::ImuData m;
      m.timestamp = t_imu;
      m.wm = imu[imu_i].gyro;
      m.am = imu[imu_i].accel;
      sys->feed_measurement_imu(m);
      imu_i++;
      continue;
    }

    // ------ camera frame ------
    ov_core::CameraData msg;
    msg.timestamp = t_cam;
    cv::Mat img0 = cv::imread(cam0[cam_i].image_path, cv::IMREAD_GRAYSCALE);
    if (img0.empty()) {
      PRINT_WARNING(YELLOW "[ros-free] failed to read %s, skipping\n" RESET,
                    cam0[cam_i].image_path.c_str());
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
      cam_i++;
      continue;
    }

    double t0 = cv::getTickCount() / cv::getTickFrequency();
    sys->feed_measurement_camera(msg);
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

          // --- Timing diagnostics ---
          double dt_cam = (prev_t_cam > 0) ? (t_cam - prev_t_cam) : -1.0;
          int n_imu_this = (int)imu_i - (int)imu_i_prev_cam;
          double imu_dt_min = 1e9, imu_dt_max = 0.0, imu_dt_sum = 0.0;
          int imu_dt_count = 0;
          bool imu_gap_detected = false, imu_backward = false;
          const double nominal_imu_dt = 1.0 / 200.0; // 200 Hz
          const double nominal_cam_dt = 1.0 / 31.0;  // 31 Hz
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

        // KLT tracker counts from the warp viz packet
        int klt_raw_now = 0, tracked_now = 0;
        {
          ov_core::TrackerWarpVizPacket pkt_m;
          if (sys->get_warp_viz_packet(0, pkt_m)) {
            klt_raw_now = pkt_m.n_klt_attempted;
            tracked_now = (int)pkt_m.curr_pts_raw.size();
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

    // Update diagnostic tracking for next frame
    prev_t_cam = t_cam;
    imu_i_prev_cam = imu_i;

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
