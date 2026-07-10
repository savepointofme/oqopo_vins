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
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/StateHelper.h"
#include "update/UpdaterGroundPlaneRange.h"
#include "update/UpdaterGroundPlaneFeature.h"
#include "update/UpdaterGroundPlaneFeatureV1.h"
#include "update/VisualObservabilityPolicy.h"
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

double wrap_rad(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double wrap_deg(double a) {
  while (a > 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

double yaw_from_Rwi(const Eigen::Matrix3d &R_wi) {
  return std::atan2(R_wi(1, 0), R_wi(0, 0));
}

Eigen::Vector3d rpy_from_Rwi(const Eigen::Matrix3d &R_wi) {
  const double pitch = std::asin(std::max(-1.0, std::min(1.0, -R_wi(2, 0))));
  double roll = 0.0;
  double yaw = 0.0;
  if (std::fabs(std::cos(pitch)) > 1e-6) {
    roll = std::atan2(R_wi(2, 1), R_wi(2, 2));
    yaw = std::atan2(R_wi(1, 0), R_wi(0, 0));
  } else {
    yaw = std::atan2(-R_wi(0, 1), R_wi(1, 1));
  }
  return {roll, pitch, yaw};
}

Eigen::Matrix3d Rwi_from_rpy(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  return (Rz * Ry * Rx).toRotationMatrix();
}

double median_tracker_parallax_px(const ov_core::TrackerWarpVizPacket &pkt) {
  const size_t n = std::min(pkt.prev_pts_for_viz.size(), pkt.curr_pts_raw.size());
  if (!pkt.valid || n == 0)
    return std::numeric_limits<double>::quiet_NaN();
  std::vector<double> flow;
  flow.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const double du = pkt.curr_pts_raw[i].x - pkt.prev_pts_for_viz[i].x;
    const double dv = pkt.curr_pts_raw[i].y - pkt.prev_pts_for_viz[i].y;
    const double mag = std::hypot(du, dv);
    if (std::isfinite(mag))
      flow.push_back(mag);
  }
  if (flow.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(flow.begin(), flow.end());
  return flow[flow.size() / 2];
}

std::string json_escape(const std::string &value) {
  std::ostringstream out;
  for (char ch : value) {
    switch (ch) {
    case '\\': out << "\\\\"; break;
    case '"': out << "\\\""; break;
    case '\n': out << "\\n"; break;
    case '\r': out << "\\r"; break;
    case '\t': out << "\\t"; break;
    default: out << ch; break;
    }
  }
  return out.str();
}

std::string sibling_path(const std::string &base_path, const std::string &name) {
  fs::path p(base_path);
  fs::path parent = p.parent_path();
  return (parent.empty() ? fs::path(name) : parent / name).string();
}

struct NavFrameState {
  bool valid = false;
  bool metadata_written = false;
  double init_camera_timestamp = -1.0;
  double selected_fc_timestamp = -1.0;
  double fc_time_offset = 0.0;
  double gps_timestamp = -1.0;
  double gps_age = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d R_Gnav_W0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_Gnav_W0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_Gnav_GPS0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_Gnav_I0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_W0_I0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d gps_antenna_in_imu = Eigen::Vector3d::Zero();
};

struct Args;
void write_nav_metadata(const std::string &path,
                        const NavFrameState &nav,
                        const Args &args);

struct Args {
  std::string config_path;
  std::string dataset_dir;
  std::string gps_path;
  std::string gt_path;
  std::string output_path = "traj_ros_free.txt";
  std::string output_raw_path;
  std::string output_nav_path;
  std::string nav_frame_metadata_path;
  std::string init_from_fc_path;
  std::string video_path;
  std::string video_cam_path;
  std::string dashboard_alignment_json_path;
  int video_fps = 20;
  double align_seconds = 8.0;
  double start_time = 0.0;
  double until_time = std::numeric_limits<double>::infinity(); // stop processing when cam timestamp exceeds this
  bool stereo = false;
  bool gps_alt_update = false;
  double gps_alt_sigma = 2.0;
  double gps_alt_chi2 = 10000.0;
  bool gps_alt_also_vz = false;
  bool gps_alt_relative = false;
  double gps_alt_min_pzz = 0.0;
  double gps_alt_min_t_after_init = 0.0; // delay Stage A z_ground bootstrap (s)
  double gps_alt_max_res = 1e9;
  double gps_alt_guard_dxy = 0.0;
  double gps_alt_guard_kxy_ratio = 0.0;
  double gps_alt_guard_dbias = 0.0;
  std::string gps_alt_coupled_mode = "guarded";
  std::string gps_alt_coupled_diag_path;
  double gps_alt_residual_soft_limit = 0.0;
  double gps_alt_max_delta_xy = 0.0;
  double gps_alt_max_delta_z = 0.0;
  double gps_alt_max_delta_velocity = 0.0;
  double gps_alt_max_delta_attitude_deg = 0.0;
  double gps_alt_max_delta_accel_bias = 0.0;
  double gps_alt_max_delta_gyro_bias = 0.0;
  double gps_alt_cov_psd_check_interval = 1.0;
  double gps_alt_nasa_beta = 0.0;         // NASA underweight coefficient beta (>=0; 0 disables)
  double gps_alt_nasa_q_threshold = 0.0;  // NASA trigger on H'PH (m^2)
  bool gps_alt_joseph = false;           // PX4-style masked Joseph update (Gate 2)
  bool gps_alt_ground_plane = false;     // Enable scalar height ground-plane updater
  bool gps_alt_ground_plane_rangefinder = false; // true: height column is rangefinder/LiDAR range
  // -- Stage B (ground-plane feature update) --
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
  double gps_cutoff_time = -1.0;
  double gps_feed_every = 1.0;
  double gps_time_offset_cli = std::numeric_limits<double>::quiet_NaN();
  double init_att_sigma_deg = 5.0;
  double init_vel_sigma = 5.0;
  double init_pos_sigma = 100.0;
  double init_bg_sigma = 0.05;
  double init_ba_sigma = 1.0;
  double init_from_fc_max_dt = 0.25;
  bool init_from_fc_warn_only = false;
  Eigen::Vector3d gps_antenna_in_imu = Eigen::Vector3d::Zero(); // p_I_GPS meters
  bool show = true;
  std::string dash_title = "OpenVINS ROS-free Dashboard";
  int dash_every = 1;
  int cam_subsample = 1;         // --camera-frame-stride/--cam-subsample N: feed only 1 of every N camera frames to VIO
  bool adaptive_stride_shadow = false; // calculate/log policy but preserve fixed input stride
  bool adaptive_stride = false;        // actively apply the causal parallax policy
  std::string adaptive_stride_log_path;
  bool adaptive_stride_use_parallax = true;
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
  bool viz_fast = false;
  bool verbose_timing = false;
  bool no_vio_yaw_update = false; // alias for --vio-yaw-update-scale 0.0
  std::string vio_yaw_update_mode;
  double vio_yaw_update_scale = std::numeric_limits<double>::quiet_NaN();
  double vio_global_yaw_oc_alpha = std::numeric_limits<double>::quiet_NaN();
  double vio_yaw_control_start_after_init = 0.0; // seconds; 0 = apply requested yaw control immediately
  double visual_bgz_update_scale = 1.0;          // --visual-bgz-update-scale: scale bg_z row of visual K
  double curl_correction_rate_degps = 0.0;       // --curl-correction-rate: camera-fixed curl bias correction (deg/s)
  // Timed visual-yaw policy switch.
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
  bool pose_repair_sim_gps = false;       // sparse reference-image repair simulated by GPS ENU + course yaw
  double pose_repair_period_s = 180.0;
  double pose_repair_pos_sigma = 3.0;
  double pose_repair_yaw_sigma_deg = 5.0;
  double pose_repair_gate_sigma = 3.0;
  double pose_repair_max_pos_correction = 15.0;
  double pose_repair_max_yaw_correction_deg = 20.0;
  double pose_repair_min_speed_mps = 3.0;
  double pose_repair_max_gps_age_s = 0.50;
  bool pose_repair_trusted_reinit = true;
  std::string pose_repair_log_path;
  bool restart_supervisor = false;
  int restart_consecutive_repair_failures = 3;
  double restart_pos_error_m = 80.0;
  double restart_yaw_error_deg = 45.0;
  double restart_cooldown_s = 30.0;
  double restart_visual_settle_s = 3.0;
  bool restart_with_gps_init = true;
  bool restart_on_pose_repair = false;
  double mech_diag_t0 = -std::numeric_limits<double>::infinity();
  double mech_diag_t1 = std::numeric_limits<double>::infinity();
  double slam_freeze_t0 = -1.0;          // --slam-update-freeze-window start
  double slam_freeze_t1 = -1.0;          // --slam-update-freeze-window end
  // Atomic experiment selection for explicit OC / FEJ combinations.
  std::string vio_consistency_mode;
  // High-level gauge-mode alias: expands to vio_yaw_update_mode + alpha
  std::string vio_yaw_gauge_mode;
  // Visual update guard (commit 1)
  double visual_skip_t0 = -1.0;           // --visual-update-skip-window start
  double visual_skip_t1 = -1.0;           // --visual-update-skip-window end
  std::string visual_guard_log_path;       // --visual-update-guard-log
  std::string visual_reject_file_path;     // --visual-update-reject-topn-file
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

void write_nav_metadata(const std::string &path,
                        const NavFrameState &nav,
                        const Args &args) {
  if (path.empty() || !nav.valid)
    return;
  fs::path p(path);
  if (!p.parent_path().empty())
    fs::create_directories(p.parent_path());
  std::ofstream jf(path);
  if (!jf.is_open()) {
    PRINT_WARNING(YELLOW "[NAV-FRAME] failed to write metadata: %s\n" RESET, path.c_str());
    return;
  }
  jf << std::setprecision(17)
     << "{\n"
     << "  \"frame_name\": \"G_nav\",\n"
     << "  \"origin_definition\": \"ENU anchored by DatasetReaderEuroc at the first valid GPS sample in source_gps_file\",\n"
     << "  \"axis_definition\": \"East, North, Up\",\n"
     << "  \"source_fc_file\": \"" << json_escape(args.init_from_fc_path) << "\",\n"
     << "  \"source_gps_file\": \"" << json_escape(args.gps_path) << "\",\n"
     << "  \"init_camera_timestamp\": " << nav.init_camera_timestamp << ",\n"
     << "  \"selected_fc_timestamp\": " << nav.selected_fc_timestamp << ",\n"
     << "  \"fc_time_offset\": " << nav.fc_time_offset << ",\n"
     << "  \"gps_timestamp\": " << nav.gps_timestamp << ",\n"
     << "  \"gps_age_at_init_s\": " << nav.gps_age << ",\n"
     << "  \"position_source\": \"latest_gps_at_or_before_init_camera_timestamp\",\n"
     << "  \"attitude_source\": \"fc_init_attitude_after_existing_fc_to_imu_conversion\",\n"
     << "  \"fc_imu_extrinsic_version\": \"none_identity_v0\",\n"
     << "  \"gps_imu_lever_arm_version\": \"cli_p_I_GPS_v0\",\n"
     << "  \"gps_antenna_in_imu_m\": [" << nav.gps_antenna_in_imu.x() << ", "
     << nav.gps_antenna_in_imu.y() << ", " << nav.gps_antenna_in_imu.z() << "],\n"
     << "  \"R_Gnav_W0\": [["
     << nav.R_Gnav_W0(0, 0) << ", " << nav.R_Gnav_W0(0, 1) << ", " << nav.R_Gnav_W0(0, 2) << "], ["
     << nav.R_Gnav_W0(1, 0) << ", " << nav.R_Gnav_W0(1, 1) << ", " << nav.R_Gnav_W0(1, 2) << "], ["
     << nav.R_Gnav_W0(2, 0) << ", " << nav.R_Gnav_W0(2, 1) << ", " << nav.R_Gnav_W0(2, 2) << "]],\n"
     << "  \"p_Gnav_W0\": [" << nav.p_Gnav_W0.x() << ", " << nav.p_Gnav_W0.y() << ", " << nav.p_Gnav_W0.z() << "],\n"
     << "  \"p_Gnav_GPS0\": [" << nav.p_Gnav_GPS0.x() << ", " << nav.p_Gnav_GPS0.y() << ", " << nav.p_Gnav_GPS0.z() << "],\n"
     << "  \"p_Gnav_I0\": [" << nav.p_Gnav_I0.x() << ", " << nav.p_Gnav_I0.y() << ", " << nav.p_Gnav_I0.z() << "],\n"
     << "  \"p_W0_I0\": [" << nav.p_W0_I0.x() << ", " << nav.p_W0_I0.y() << ", " << nav.p_W0_I0.z() << "],\n"
     << "  \"whether_future_data_used\": false,\n"
     << "  \"evaluation_only\": false,\n"
     << "  \"uses_future_data\": false\n"
     << "}\n";
}

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
               "  --height-mode MODE     GPS-Z fusion mode: guarded|nasa_lean|standard|bounded\n"
               "  --gps-alt-coupled-mode MODE  Same as --height-mode\n"
               "  --gps-alt-coupled-diag PATH  per-update full-gain/covariance diagnostic CSV\n"
               "  --gps-alt-residual-soft-limit M  uniform residual-based gain bound (0=off)\n"
               "  --gps-alt-max-delta-xy M      bounded-mode single-update XY norm limit\n"
               "  --gps-alt-max-delta-z M       bounded-mode single-update |dz| limit\n"
               "  --gps-alt-max-delta-velocity MPS  bounded-mode velocity increment norm limit\n"
               "  --gps-alt-max-delta-attitude-deg DEG  bounded-mode attitude error norm limit\n"
               "  --gps-alt-max-delta-accel-bias V  bounded-mode accel-bias increment norm limit\n"
               "  --gps-alt-max-delta-gyro-bias V   bounded-mode gyro-bias increment norm limit\n"
               "  --gps-alt-cov-psd-check-interval S full LDLT check interval; <=0 checks every update\n"
               "  --gps-alt-nasa-beta B         NASA underweight coefficient (nasa_lean mode; 0=off)\n"
               "  --gps-alt-nasa-q-threshold Q  NASA trigger on H'PH in m^2 (nasa_lean mode)\n"
               "  --gps-alt-joseph-update  PX4-style masked Joseph update (Gate 2):\n"
               "                            only p_z state DOF and p_z row/col of P updated.\n"
               "                            All guard flags still apply.\n"
               "  --gps-alt-ground-plane   Bootstrap a local ground plane from scalar height.\n"
               "                            Default type is gps: h(x)=p_z, no tilt/r22 term.\n"
               "  --gps-alt-ground-plane-type gps|rangefinder\n"
               "                            rangefinder/LiDAR uses h=(p_z-z_ground)/cos_tilt.\n"
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
               "  --init-from-fc-max-dt S Maximum |FC time - camera init time| for FC init (default 0.25)\n"
               "  --init-from-fc-warn-only Warn instead of failing when --init-from-fc-max-dt is exceeded\n"
               "  --gt PATH             ASL 17-col ground truth CSV\n"
               "  --output PATH         Legacy raw TUM trajectory path (default: traj_ros_free.txt)\n"
               "  --output-raw PATH     Canonical raw estimator trajectory path (default: OUTPUT sibling traj_raw.txt)\n"
               "  --output-nav PATH     Formal navigation-frame trajectory path (default: OUTPUT sibling traj_nav.txt)\n"
               "  --nav-frame-metadata-json PATH  Metadata for fixed T_Gnav_W0 (default: OUTPUT sibling nav_frame_metadata.json)\n"
               "  --gps-antenna-in-imu X Y Z  GPS antenna position p_I_GPS in IMU frame meters (default 0 0 0)\n"
               "  --video PATH          Record dashboard to MP4\n"
               "  --video-cam PATH      Record camera-only (cam0/cam1 w/ optical-flow tracks) to MP4\n"
               "  --video-fps N         Video FPS (default 20)\n"
               "  --dashboard-alignment-json PATH  Persist display-only dashboard XY yaw alignment metadata\n"
               "  --align-seconds X     Seconds of data to collect before SE3 align (default 8)\n"
               "  --no-display          Do not create a window (useful headless)\n"
               "  --dash-title TITLE    Window title (default: 'OpenVINS ROS-free Dashboard')\n"
               "  --dash-every N        Refresh dashboard every N camera frames (default 1)\n"
               "  --camera-frame-stride N  Feed only 1 of every N camera frames to VIO before tracker (default 1)\n"
               "  --adaptive-stride-shadow  Compute/log the causal height+turn policy; keep fixed camera stride\n"
               "  --adaptive-stride         Apply the causal parallax policy to camera input (requires fixed stride 1)\n"
               "  --adaptive-stride-log PATH  Per-raw-frame policy CSV (default: OUTPUT.adaptive_stride.csv)\n"
               "  --adaptive-stride-no-parallax  Disable closed-loop stride correction from observed KLT parallax\n"
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
               "  --vio-yaw-update-mode M   original|per_block_scale|global_yaw_oc_projection|global_yaw_oc_fej_projection\n"
               "                            global_yaw_oc_fej_prechi2|visual_4d_oc_fej_prechi2\n"
               "  --vio-yaw-update-scale S  Scale visual yaw correction for *_scale modes (1=orig, 0=off)\n"
               "  --vio-global-yaw-oc-alpha A  H-projection alpha for global_yaw_oc_projection (0=orig, 1=full)\n"
               "  --vio-yaw-control-start-after-init S  Delay requested visual yaw control until S seconds after init\n"
               "  --visual-bgz-update-scale S  Scale bg_z row of visual K_eff (1.0=normal, 0.0=freeze bg_z from visual)\n"
               "  --curl-correction-rate R     Camera-fixed optical-axis curl correction (deg/s, + = CCW in image)\n"
               "  --vio-yaw-switch-mode M   After --vio-yaw-switch-time, switch to this visual yaw mode\n"
               "  --vio-yaw-switch-time T   Absolute timestamp (s) to switch yaw mode\n"
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
                "  --pose-repair-sim-gps    Periodically simulate sparse reference-image pose repair using GPS ENU + course yaw\n"
                "  --pose-repair-period S   Seconds between sparse pose repairs (default 180)\n"
                "  --pose-repair-log PATH   CSV log for sparse pose repair decisions\n"
                "  --pose-repair-no-trusted-reinit  Do not reinitialize from trusted sparse pose anchors after large residuals\n"
                "  --restart-supervisor     Enable hard restart only for severe state health faults by default\n"
                "  --restart-on-pose-repair Allow sparse pose-repair lost suspects to hard restart after visual confirmation\n"
                "  --restart-consecutive-repair-failures N  Failures before restart (default 3)\n"
                "  --restart-pos-error M    Severe GPS-relative position error threshold (default 80m)\n"
                "  --restart-yaw-error-deg DEG  Severe course-yaw error threshold (default 45deg)\n"
                "  --restart-visual-settle S  Seconds to suppress visual updates after restart (default 3)\n"
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
               "    baseline                 global_yaw_oc_projection + alpha=1.0\n"
               "    original                 original (no gauge protection)\n"
               "    oc_prechi2               global_yaw_oc_fej_prechi2\n"
               "    oc_4d                    visual_4d_oc_fej_prechi2\n"
               "    oc_postchi2_fej_gauge    global_yaw_oc_fej_projection + alpha=1.0\n"
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
    else if (s == "--output-raw") a.output_raw_path = next("--output-raw");
    else if (s == "--output-nav") a.output_nav_path = next("--output-nav");
    else if (s == "--nav-frame-metadata-json") a.nav_frame_metadata_path = next("--nav-frame-metadata-json");
    else if (s == "--video") a.video_path = next("--video");
    else if (s == "--video-cam") a.video_cam_path = next("--video-cam");
    else if (s == "--video-fps") a.video_fps = std::atoi(next("--video-fps").c_str());
    else if (s == "--dashboard-alignment-json") a.dashboard_alignment_json_path = next("--dashboard-alignment-json");
    else if (s == "--align-seconds") a.align_seconds = std::atof(next("--align-seconds").c_str());
    else if (s == "--start-time") a.start_time = std::atof(next("--start-time").c_str());
    else if (s == "--until-time") a.until_time = std::atof(next("--until-time").c_str());
    else if (s == "--stereo") a.stereo = true;
    else if (s == "--gps-alt-update") a.gps_alt_update = true;
    else if (s == "--gps-alt-sigma") a.gps_alt_sigma = std::atof(next("--gps-alt-sigma").c_str());
    else if (s == "--gps-alt-chi2") a.gps_alt_chi2 = std::atof(next("--gps-alt-chi2").c_str());
    else if (s == "--gps-alt-also-vz") a.gps_alt_also_vz = true;
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
    else if (s == "--gps-alt-joseph-update") a.gps_alt_joseph = true;
    else if (s == "--gps-alt-ground-plane") a.gps_alt_ground_plane = true;
    else if (s == "--gps-alt-ground-plane-type" || s == "--height-source") {
      const std::string type = next(s.c_str());
      if (type == "gps") {
        a.gps_alt_ground_plane_rangefinder = false;
      } else if (type == "rangefinder" || type == "lidar") {
        a.gps_alt_ground_plane_rangefinder = true;
      } else {
        fprintf(stderr, "unknown ground-plane height type '%s' (expected gps|rangefinder)\n", type.c_str());
        return false;
      }
    }
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
    else if (s == "--init-from-fc-max-dt") a.init_from_fc_max_dt = std::atof(next("--init-from-fc-max-dt").c_str());
    else if (s == "--init-from-fc-warn-only") a.init_from_fc_warn_only = true;
    else if (s == "--gps-antenna-in-imu") {
      a.gps_antenna_in_imu.x() = std::atof(next("--gps-antenna-in-imu X").c_str());
      a.gps_antenna_in_imu.y() = std::atof(next("--gps-antenna-in-imu Y").c_str());
      a.gps_antenna_in_imu.z() = std::atof(next("--gps-antenna-in-imu Z").c_str());
    }
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
    else if (s == "--adaptive-stride-no-parallax")
      a.adaptive_stride_use_parallax = false;
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
    else if (s == "--pose-repair-sim-gps") a.pose_repair_sim_gps = true;
    else if (s == "--pose-repair-period") a.pose_repair_period_s = std::atof(next("--pose-repair-period").c_str());
    else if (s == "--pose-repair-pos-sigma") a.pose_repair_pos_sigma = std::atof(next("--pose-repair-pos-sigma").c_str());
    else if (s == "--pose-repair-yaw-sigma-deg") a.pose_repair_yaw_sigma_deg = std::atof(next("--pose-repair-yaw-sigma-deg").c_str());
    else if (s == "--pose-repair-gate-sigma") a.pose_repair_gate_sigma = std::atof(next("--pose-repair-gate-sigma").c_str());
    else if (s == "--pose-repair-max-pos-correction") a.pose_repair_max_pos_correction = std::atof(next("--pose-repair-max-pos-correction").c_str());
    else if (s == "--pose-repair-max-yaw-correction-deg") a.pose_repair_max_yaw_correction_deg = std::atof(next("--pose-repair-max-yaw-correction-deg").c_str());
    else if (s == "--pose-repair-min-speed") a.pose_repair_min_speed_mps = std::atof(next("--pose-repair-min-speed").c_str());
    else if (s == "--pose-repair-max-gps-age") a.pose_repair_max_gps_age_s = std::atof(next("--pose-repair-max-gps-age").c_str());
    else if (s == "--pose-repair-no-trusted-reinit") a.pose_repair_trusted_reinit = false;
    else if (s == "--pose-repair-log") a.pose_repair_log_path = next("--pose-repair-log");
    else if (s == "--restart-supervisor") a.restart_supervisor = true;
    else if (s == "--restart-consecutive-repair-failures") a.restart_consecutive_repair_failures = std::atoi(next("--restart-consecutive-repair-failures").c_str());
    else if (s == "--restart-pos-error") a.restart_pos_error_m = std::atof(next("--restart-pos-error").c_str());
    else if (s == "--restart-yaw-error-deg") a.restart_yaw_error_deg = std::atof(next("--restart-yaw-error-deg").c_str());
    else if (s == "--restart-cooldown") a.restart_cooldown_s = std::atof(next("--restart-cooldown").c_str());
    else if (s == "--restart-visual-settle") a.restart_visual_settle_s = std::atof(next("--restart-visual-settle").c_str());
    else if (s == "--restart-no-gps-init") a.restart_with_gps_init = false;
    else if (s == "--restart-on-pose-repair") a.restart_on_pose_repair = true;
    else if (s == "--slam-update-freeze-window") {
      a.slam_freeze_t0 = std::atof(next("--slam-update-freeze-window T0").c_str());
      a.slam_freeze_t1 = std::atof(next("--slam-update-freeze-window T1").c_str());
    }
    else if (s == "--visual-update-skip-window") {
      a.visual_skip_t0 = std::atof(next("--visual-update-skip-window T0").c_str());
      a.visual_skip_t1 = std::atof(next("--visual-update-skip-window T1").c_str());
    }
    else if (s == "--visual-update-guard-log") a.visual_guard_log_path = next("--visual-update-guard-log");
    else if (s == "--visual-update-reject-topn-file") a.visual_reject_file_path = next("--visual-update-reject-topn-file");
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
  if (args.pose_repair_sim_gps && args.gps_path.empty()) {
    PRINT_ERROR(RED "[pose-repair] --pose-repair-sim-gps requires --gps PATH\n" RESET);
    return EXIT_FAILURE;
  }
  args.pose_repair_period_s = std::max(1.0, args.pose_repair_period_s);
  args.pose_repair_pos_sigma = std::max(0.01, args.pose_repair_pos_sigma);
  args.pose_repair_yaw_sigma_deg = std::max(0.1, args.pose_repair_yaw_sigma_deg);
  args.pose_repair_gate_sigma = std::max(0.5, args.pose_repair_gate_sigma);
  args.pose_repair_max_gps_age_s = std::max(0.01, args.pose_repair_max_gps_age_s);
  args.restart_consecutive_repair_failures = std::max(1, args.restart_consecutive_repair_failures);
  args.restart_cooldown_s = std::max(0.0, args.restart_cooldown_s);
  args.restart_visual_settle_s = std::max(0.0, args.restart_visual_settle_s);

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

  if (!std::isnan(args.gps_time_offset_cli)) {
    params.gps_time_offset = args.gps_time_offset_cli;
    PRINT_INFO(CYAN "[ros-free] CLI override: gps_time_offset=%+.3fs\n" RESET,
               params.gps_time_offset);
  }

  // CLI --cam-toff: override timeshift_cam_imu and disable online calibration.
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
    std::string mapped_mode;
    double mapped_alpha = std::numeric_limits<double>::quiet_NaN();

    if (gm == "openvins_fej" || gm == "original" || gm == "fej") {
      mapped_mode = "original";
    } else if (gm == "oc_postchi2_current_gauge" || gm == "baseline") {
      mapped_mode = "global_yaw_oc_projection";
      mapped_alpha = 1.0;
    } else if (gm == "oc_prechi2_fej_gauge" || gm == "oc_prechi2") {
      mapped_mode = "global_yaw_oc_fej_prechi2";
    } else if (gm == "oc_4d_prechi2_fej_gauge" || gm == "oc_4d") {
      mapped_mode = "visual_4d_oc_fej_prechi2";
    } else if (gm == "oc_postchi2_fej_gauge") {
      mapped_mode = "global_yaw_oc_fej_projection";
      mapped_alpha = 1.0;
    } else {
      PRINT_ERROR("[ros-free] invalid --vio-yaw-gauge-mode=%s; expected baseline, original, oc_prechi2, oc_4d, or oc_postchi2_fej_gauge.\n",
                  gm.c_str());
      return EXIT_FAILURE;
    }

    if (args.vio_yaw_update_mode.empty()) {
      args.vio_yaw_update_mode = mapped_mode;
      if (!std::isnan(mapped_alpha) && std::isnan(args.vio_global_yaw_oc_alpha))
        args.vio_global_yaw_oc_alpha = mapped_alpha;
      PRINT_INFO(CYAN "[ros-free] --vio-yaw-gauge-mode=%s -> mode=%s alpha=%.1f\n" RESET,
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
                                    params.vio_yaw_update_mode == "global_yaw_oc_projection" ||
                                    params.vio_yaw_update_mode == "global_yaw_oc_fej_projection" ||
                                    VisualObservabilityPolicy::is_prechi2_mode_string(params.vio_yaw_update_mode) ||
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
  if (args.gplane_feat_enable) {
    if (!args.gps_alt_ground_plane) {
      PRINT_WARNING(YELLOW "[ros-free] --gplane-feat requested but --gps-alt-ground-plane not set; "
                    "Stage B will not fire until z_ground is bootstrapped.\n" RESET);
    }
    sys->enable_gplane_feature(args.gplane_feat_sigma_px, args.gplane_feat_max_features,
                               args.gplane_feat_center_frac, args.gplane_feat_min_cos_tilt,
                               args.gplane_feat_max_res_px);
  }
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
  if (args.gps_alt_max_res < 1e8) {
    sys->set_gps_alt_max_res_gate(args.gps_alt_max_res);
    PRINT_INFO(CYAN "[ros-free] GPS alt max-res gate = %.1f m\n" RESET, args.gps_alt_max_res);
  }
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

  if (!gps.empty() && std::fabs(params.gps_time_offset) > 1e-9) {
    for (auto &s : gps)
      s.timestamp += params.gps_time_offset;
    PRINT_INFO(CYAN "[ros-free] applied gps_time_offset=%+.3fs to %zu GPS samples\n" RESET,
               params.gps_time_offset, gps.size());
  }

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
      if (cam0.empty())
        throw std::runtime_error("no camera frames remain after --start-time trim; cannot choose FC init by camera time");
      const double fc_init_target_time = cam0.front().timestamp;
      FCInitLoadOptions fc_init_options;
      fc_init_options.max_abs_dt = args.init_from_fc_max_dt;
      fc_init_options.warn_only = args.init_from_fc_warn_only;
      fc_init_state = load_fc_init_state_csv(args.init_from_fc_path, fc_init_target_time, fc_init_options);
      fc_init_pending = true;
      PRINT_INFO(CYAN "[ros-free] loaded FC init CSV: %s\n" RESET, args.init_from_fc_path.c_str());
      PRINT_INFO(CYAN "[ros-free] FC init row t=%.6f target camera=%.6f requested start=%.6f dt=%.6f |v|=%.3f max_dt=%.3f mode=%s\n" RESET,
                 fc_init_state.timestamp, fc_init_target_time, args.start_time, fc_init_state.source_dt, fc_init_state.v_IinG.norm(),
                 args.init_from_fc_max_dt, args.init_from_fc_warn_only ? "warn-only" : "strict");
      if (fc_init_state.duplicate_timestamp_count > 0 || fc_init_state.non_monotonic_timestamp_count > 0) {
        PRINT_WARNING(YELLOW "[ros-free] FC init CSV timing diagnostics: valid_rows=%d duplicate_timestamps=%d non_monotonic_steps=%d\n" RESET,
                      fc_init_state.valid_row_count, fc_init_state.duplicate_timestamp_count,
                      fc_init_state.non_monotonic_timestamp_count);
      }
      if (std::isfinite(args.init_from_fc_max_dt) && args.init_from_fc_max_dt >= 0.0 &&
          std::fabs(fc_init_state.source_dt) > args.init_from_fc_max_dt) {
        PRINT_WARNING(YELLOW "[ros-free] FC init timestamp mismatch exceeds max_dt; continuing only because --init-from-fc-warn-only is set\n" RESET);
      }
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

  cv::VideoWriter cam_writer;
  cv::Size cam_video_size;
  bool cam_writer_init = false;

  if (args.output_raw_path.empty())
    args.output_raw_path = sibling_path(args.output_path, "traj_raw.txt");
  if (args.output_nav_path.empty())
    args.output_nav_path = sibling_path(args.output_path, "traj_nav.txt");
  if (args.nav_frame_metadata_path.empty())
    args.nav_frame_metadata_path = sibling_path(args.output_path, "nav_frame_metadata.json");

  // -------------------- output files --------------------
  std::ofstream out(args.output_path);
  if (!out.is_open()) {
    PRINT_ERROR(RED "[ros-free] cannot open output: %s\n" RESET, args.output_path.c_str());
    return EXIT_FAILURE;
  }
  out << "# TUM traj (t x y z qx qy qz qw) in W0 raw estimator frame; legacy path\n";
  out << std::fixed << std::setprecision(9);

  std::ofstream raw_out;
  const bool write_raw_alias = !args.output_raw_path.empty() && args.output_raw_path != args.output_path;
  if (write_raw_alias) {
    fs::path p(args.output_raw_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    raw_out.open(args.output_raw_path);
    if (!raw_out.is_open()) {
      PRINT_ERROR(RED "[ros-free] cannot open raw output: %s\n" RESET, args.output_raw_path.c_str());
      return EXIT_FAILURE;
    }
    raw_out << "# TUM traj (t x y z qx qy qz qw) in W0 raw estimator frame\n";
    raw_out << std::fixed << std::setprecision(9);
  }

  std::ofstream nav_out;
  if (!args.output_nav_path.empty()) {
    fs::path p(args.output_nav_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    nav_out.open(args.output_nav_path);
    if (!nav_out.is_open()) {
      PRINT_ERROR(RED "[ros-free] cannot open nav output: %s\n" RESET, args.output_nav_path.c_str());
      return EXIT_FAILURE;
    }
    nav_out << "# traj_nav in G_nav: t x y z vx vy vz qx qy qz qw; T_Gnav_W0 fixed at initialization; no future-data fit\n";
    nav_out << std::fixed << std::setprecision(9);
  }

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
        << "horizontal_speed,roll,pitch,gyro_norm,median_parallax_px,active_msckf_features,"
        << "active_slam_features,reason_height,reason_turn,reason_feature_low,"
        << "reason_parallax_low,reason_parallax_high,hysteresis_state,hold_timer,"
        << "base_stride,filtered_height,filtered_speed,filtered_parallax_px,"
        << "normalized_parallax_px,parallax_target_stride,"
        << "effective_height,filtered_gyro,severe_turn,severe_feature_low,"
        << "severe_parallax_high,force_fullrate_tracking,changed,"
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

  std::ofstream pose_repair_out;
  if (args.pose_repair_sim_gps) {
    if (args.pose_repair_log_path.empty())
      args.pose_repair_log_path = args.output_path + ".pose_repair.csv";
    fs::path p(args.pose_repair_log_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    pose_repair_out.open(args.pose_repair_log_path, std::ofstream::out | std::ofstream::trunc);
    if (!pose_repair_out.is_open()) {
      PRINT_WARNING(YELLOW "[POSE-REPAIR] failed to open %s\n" RESET,
                    args.pose_repair_log_path.c_str());
    } else {
      pose_repair_out
          << "time,scheduled_time,gps_time,gps_age,decision,accepted,"
          << "pos_residual_norm,yaw_residual_deg,nis,gate_ratio,"
          << "predicted_pos_correction_norm,predicted_yaw_correction_deg,"
          << "gps_speed,anchor_x,anchor_y,anchor_z,consecutive_failures,"
          << "restart_requested,restart_reason,restart_init_source\n";
      pose_repair_out << std::fixed << std::setprecision(9);
      PRINT_INFO(GREEN "[POSE-REPAIR] writing %s\n" RESET,
                 args.pose_repair_log_path.c_str());
    }
  }

  // -------------------- timeline merge-sort --------------------
  size_t imu_i = 0, cam_i = 0, cam1_i = 0, gps_i = 0;
  // Track last GPS sample index actually fed to the filter, so we don't
  // feed the same physical GPS sample multiple times when cam_hz > gps_hz.
  long long last_fed_gps_i = -1;
  // Reference-only GPS bookkeeping for dashboard/diagnostic CSV. This stays
  // active with --gps even when GPS is not fused into the EKF.
  long long last_diag_gps_i = -1;
  const double INF = std::numeric_limits<double>::infinity();
  double t_init_done = -1;
  std::deque<std::pair<double, Eigen::Vector3d>> vio_for_align;
  int frame_idx = 0;
  int align_fit_count = 0;
  NavFrameState nav_frame;
  nav_frame.gps_antenna_in_imu = args.gps_antenna_in_imu;
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

  double gps_alt_ref = std::numeric_limits<double>::quiet_NaN();
  double gps_alt_vio_ref = 0.0;
  Eigen::Vector2d gps_xy_ref = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector2d gps_xy_vio_ref = Eigen::Vector2d::Zero();
  size_t gps_alt_feeds = 0;
  double pose_repair_next_time = -1.0;
  Eigen::Vector3d pose_repair_gps_ref = Eigen::Vector3d::Zero();
  Eigen::Vector3d pose_repair_vio_ref = Eigen::Vector3d::Zero();
  bool pose_repair_ref_valid = false;
  size_t pose_repair_attempts = 0;
  size_t pose_repair_accepts = 0;
  size_t pose_repair_rejects = 0;
  int pose_repair_consecutive_failures = 0;
  int pose_repair_severe_failures = 0;
  bool pose_repair_lost_suspect = false;
  std::string pose_repair_lost_reason;
  double pose_repair_lost_time = -std::numeric_limits<double>::infinity();
  int visual_lost_credit = 0;
  double last_visual_lost_time = -std::numeric_limits<double>::infinity();
  bool restart_pending = false;
  std::string restart_reason;
  double last_restart_time = -std::numeric_limits<double>::infinity();
  size_t restart_count = 0;
  bool restart_anchor_override_valid = false;
  FCInitState restart_anchor_fc;
  Eigen::Vector3d restart_anchor_gps_xyz = Eigen::Vector3d::Zero();
  bool restart_anchor_gps_valid = false;
  std::string restart_init_source;

  auto configure_after_restart = [&](const std::shared_ptr<VioManager> &next_sys) {
    if (delayed_vio_yaw_control_applied) {
      next_sys->set_vio_yaw_update_mode(delayed_vio_yaw_mode);
      next_sys->set_vio_yaw_update_scale(delayed_vio_yaw_scale);
      next_sys->set_vio_global_yaw_oc_alpha(delayed_vio_yaw_alpha);
    }
    if (timed_yaw_switch_applied && !args.vio_yaw_switch_mode.empty()) {
      next_sys->set_vio_yaw_update_mode(args.vio_yaw_switch_mode);
      if (!std::isnan(args.vio_yaw_switch_alpha))
        next_sys->set_vio_global_yaw_oc_alpha(args.vio_yaw_switch_alpha);
    }
    next_sys->set_visual_update_stride(args.visual_update_stride);
    next_sys->configure_visual_update_adaptive(
        args.visual_update_adaptive,
        args.visual_update_adaptive_target_flow_px,
        args.visual_update_adaptive_min_dt,
        args.visual_update_adaptive_max_dt,
        args.visual_update_adaptive_min_tracks);
    if (args.visual_skip_t0 >= 0.0 && args.visual_skip_t1 > args.visual_skip_t0)
      next_sys->set_visual_skip_window(args.visual_skip_t0, args.visual_skip_t1);
    if (!args.visual_reject_file_path.empty())
      next_sys->load_visual_reject_file(args.visual_reject_file_path);
    if (args.slam_yaw_contrib_cap_enable) {
      next_sys->configure_slam_yaw_contrib_cap(
          true, args.slam_yaw_contrib_cap_t0, args.slam_yaw_contrib_cap_t1,
          args.slam_yaw_contrib_cap_roll_deg, args.slam_yaw_contrib_cap_yawrate_degps,
          args.slam_yaw_contrib_cap_topk, args.slam_yaw_contrib_cap_ratio,
          args.slam_yaw_contrib_cap_mode);
    }
    if (args.slam_info_reduction_enable) {
      next_sys->configure_slam_info_reduction(true, args.slam_info_reduction_low_ratio,
                                              args.slam_info_reduction_high_ratio,
                                              args.slam_info_reduction_alpha_max);
    }
    if (args.slam_geometry_lifecycle_refresh_enable) {
      next_sys->configure_slam_geometry_lifecycle_refresh(
          true,
          args.slam_geometry_refresh_min_depth,
          args.slam_geometry_refresh_min_age,
          args.slam_geometry_refresh_min_pose_lm_cov,
          args.slam_geometry_refresh_min_regular_features,
          args.slam_geometry_refresh_require_anchor_change);
    }
    if (args.slam_freeze_t0 >= 0.0 && args.slam_freeze_t1 > args.slam_freeze_t0)
      next_sys->set_slam_update_freeze_window(args.slam_freeze_t0, args.slam_freeze_t1);
    if (args.gps_alt_min_pzz > 0.0)
      next_sys->set_gps_alt_min_pzz(args.gps_alt_min_pzz);
    if (args.gps_alt_min_t_after_init > 0.0)
      next_sys->set_gps_alt_min_t_after_init(args.gps_alt_min_t_after_init);
    if (args.gplane_feat_enable) {
      next_sys->enable_gplane_feature(args.gplane_feat_sigma_px, args.gplane_feat_max_features,
                                      args.gplane_feat_center_frac, args.gplane_feat_min_cos_tilt,
                                      args.gplane_feat_max_res_px);
    }
    if (args.gplane_feat_v1_enable) {
      const bool dry_run = !args.gplane_feat_v1_update;
      next_sys->enable_gplane_feature_v1(dry_run, args.gplane_feat_sigma_px,
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
    if (args.gps_alt_max_res < 1e8)
      next_sys->set_gps_alt_max_res_gate(args.gps_alt_max_res);
    if (args.gps_alt_guard_dxy > 0.0)
      next_sys->set_gps_alt_guard_dxy_max(args.gps_alt_guard_dxy);
    if (args.gps_alt_guard_kxy_ratio > 0.0)
      next_sys->set_gps_alt_guard_kxy_ratio_max(args.gps_alt_guard_kxy_ratio);
    if (args.gps_alt_guard_dbias > 0.0)
      next_sys->set_gps_alt_guard_dbias_max(args.gps_alt_guard_dbias);
    next_sys->configure_gps_alt_coupled_update(
        args.gps_alt_coupled_mode,
        args.gps_alt_residual_soft_limit,
        args.gps_alt_max_delta_xy,
        args.gps_alt_max_delta_z,
        args.gps_alt_max_delta_velocity,
        args.gps_alt_max_delta_attitude_deg * M_PI / 180.0,
        args.gps_alt_max_delta_accel_bias,
        args.gps_alt_max_delta_gyro_bias,
        args.gps_alt_cov_psd_check_interval);
    next_sys->set_gps_alt_nasa_underweight(args.gps_alt_nasa_beta,
                                           args.gps_alt_nasa_q_threshold);
    if (args.gps_alt_joseph)
      next_sys->set_gps_alt_joseph_update(true);
  };

  auto maybe_align = [&](double t) {
    if (aligner.solved() || t_init_done < 0)
      return;
    if (t - t_init_done < args.align_seconds)
      return;
    std::vector<Eigen::Vector3d> pv, pg;
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
        if (!args.dashboard_alignment_json_path.empty()) {
          std::ofstream jf(args.dashboard_alignment_json_path);
          if (jf.is_open()) {
            jf << std::setprecision(17)
               << "{\n"
               << "  \"alignment_method\": \"dashboard_xy_yaw_translation\",\n"
               << "  \"display_only\": true,\n"
               << "  \"source_trajectory\": \"" << args.output_path << "\",\n"
               << "  \"target_trajectory\": \"" << (gt_pos_map.empty() ? args.gps_path : args.gt_path) << "\",\n"
               << "  \"target_type\": \"" << (gt_pos_map.empty() ? "GPS" : "GT") << "\",\n"
               << "  \"source_frame\": \"vio_raw_local\",\n"
               << "  \"output_frame\": \"dashboard_reference_frame\",\n"
               << "  \"algorithm\": \"TrajectoryAligner.solve_xy_yaw\",\n"
               << "  \"align_seconds\": " << args.align_seconds << ",\n"
               << "  \"solved_at_t\": " << t << ",\n"
               << "  \"pair_count\": " << pv.size() << ",\n"
               << "  \"yaw_deg\": " << yaw_deg << ",\n"
               << "  \"translation\": [" << aligner.t().x() << ", " << aligner.t().y() << ", " << aligner.t().z() << "],\n"
               << "  \"rotation_matrix\": [["
               << aligner.R()(0, 0) << ", " << aligner.R()(0, 1) << ", " << aligner.R()(0, 2) << "], ["
               << aligner.R()(1, 0) << ", " << aligner.R()(1, 1) << ", " << aligner.R()(1, 2) << "], ["
               << aligner.R()(2, 0) << ", " << aligner.R()(2, 1) << ", " << aligner.R()(2, 2) << "]]\n"
               << "}\n";
          } else {
            PRINT_WARNING(YELLOW "[ros-free] failed to write dashboard alignment metadata: %s\n" RESET,
                          args.dashboard_alignment_json_path.c_str());
          }
        }
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

  double course_to_imu_yaw_offset_deg = 0.0;
  bool course_to_imu_yaw_offset_valid = false;
  if (fc_init_pending) {
    double init_course_yaw_deg = 0.0;
    if (reference_course_yaw_at(fc_init_state.timestamp, init_course_yaw_deg)) {
      const Eigen::Matrix3d Rwi_fc =
          ov_core::quat_2_Rot(fc_init_state.q_GtoI).transpose();
      const double init_imu_yaw_deg = yaw_from_Rwi(Rwi_fc) * 180.0 / M_PI;
      course_to_imu_yaw_offset_deg = wrap_deg(init_imu_yaw_deg - init_course_yaw_deg);
      course_to_imu_yaw_offset_valid = true;
      PRINT_INFO(CYAN "[ros-free] course->IMU yaw offset calibrated: imu_yaw=%.2fdeg course=%.2fdeg offset=%+.2fdeg\n" RESET,
                 init_imu_yaw_deg, init_course_yaw_deg, course_to_imu_yaw_offset_deg);
    } else {
      PRINT_WARNING(YELLOW "[ros-free] course->IMU yaw offset unavailable; sparse repair yaw will use raw course yaw\n" RESET);
    }
  }
  auto imu_yaw_from_course_yaw_deg = [&](double course_yaw_deg) -> double {
    return course_to_imu_yaw_offset_valid
               ? wrap_deg(course_yaw_deg + course_to_imu_yaw_offset_deg)
               : course_yaw_deg;
  };

  auto reference_velocity_at = [&](double t, Eigen::Vector3d &vel) -> bool {
    vel = Eigen::Vector3d::Zero();
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
    vel = (gps[i1].xyz - gps[i0].xyz) / dt;
    return vel.allFinite();
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
      if (!nav_frame.valid) {
        auto gps_it = gps.end();
        if (!gps.empty()) {
          gps_it = std::upper_bound(
              gps.begin(), gps.end(), t_cam,
              [](double tt, const DatasetReaderEuroc::GpsSample &s) { return tt < s.timestamp; });
          if (gps_it != gps.begin())
            --gps_it;
          else
            gps_it = gps.end();
        }
        if (gps_it != gps.end()) {
          auto init_state = sys->get_state();
          const Eigen::Matrix3d R_W0_I0 = init_state->_imu->Rot().transpose();
          const Eigen::Vector3d p_W0_I0 = init_state->_imu->pos();
          const Eigen::Matrix3d R_Gnav_I0 = ov_core::quat_2_Rot(fc_init_state.q_GtoI).transpose();
          const Eigen::Vector3d p_Gnav_GPS0 = gps_it->xyz;
          const Eigen::Vector3d p_Gnav_I0 = p_Gnav_GPS0 - R_Gnav_I0 * args.gps_antenna_in_imu;
          nav_frame.valid = true;
          nav_frame.init_camera_timestamp = t_cam;
          nav_frame.selected_fc_timestamp = fc_init_state.timestamp;
          nav_frame.fc_time_offset = fc_init_state.source_dt;
          nav_frame.gps_timestamp = gps_it->timestamp;
          nav_frame.gps_age = t_cam - gps_it->timestamp;
          nav_frame.R_Gnav_W0 = R_Gnav_I0 * R_W0_I0.transpose();
          nav_frame.p_Gnav_W0 = p_Gnav_I0 - nav_frame.R_Gnav_W0 * p_W0_I0;
          nav_frame.p_Gnav_GPS0 = p_Gnav_GPS0;
          nav_frame.p_Gnav_I0 = p_Gnav_I0;
          nav_frame.p_W0_I0 = p_W0_I0;
          nav_frame.gps_antenna_in_imu = args.gps_antenna_in_imu;
          write_nav_metadata(args.nav_frame_metadata_path, nav_frame, args);
          nav_frame.metadata_written = true;
          const double nav_yaw_deg = std::atan2(nav_frame.R_Gnav_W0(1, 0), nav_frame.R_Gnav_W0(0, 0)) * 180.0 / M_PI;
          PRINT_INFO(GREEN "[NAV-FRAME] fixed T_Gnav_W0 at init t=%.6f using GPS t=%.6f age=%.3fs yaw=%.3fdeg p=[%.3f %.3f %.3f], future_data=0\n" RESET,
                     t_cam, nav_frame.gps_timestamp, nav_frame.gps_age, nav_yaw_deg,
                     nav_frame.p_Gnav_W0.x(), nav_frame.p_Gnav_W0.y(), nav_frame.p_Gnav_W0.z());
        } else {
          PRINT_WARNING(YELLOW "[NAV-FRAME] no GPS sample at or before init t=%.6f; traj_nav will remain empty\n" RESET, t_cam);
        }
      }
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
        if (sys->get_warp_viz_packet(0, pkt_stride)) {
          adaptive_input.active_msckf_features = (int)pkt_stride.curr_pts_raw.size();
          if (args.adaptive_stride_use_parallax)
            adaptive_input.median_parallax_px = median_tracker_parallax_px(pkt_stride);
        }

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
                   "h=%.1f roll=%.1f gyro=%.3f parallax=%.2f feats=%d+%d\n" RESET,
                   t_cam, adaptive_prev_stride, adaptive_decision.recommended_stride,
                   adaptive_decision.base_stride, adaptive_decision.switch_reason.c_str(),
                   adaptive_input.relative_height_m, adaptive_input.roll_deg,
                   adaptive_input.gyro_norm_radps, adaptive_input.median_parallax_px,
                   adaptive_input.active_msckf_features,
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
          << adaptive_input.median_parallax_px << ","
          << adaptive_input.active_msckf_features << ","
          << adaptive_input.active_slam_features << ","
          << (adaptive_decision.reason_height_valid ? 1 : 0) << ","
          << (adaptive_decision.reason_turn ? 1 : 0) << ","
          << (adaptive_decision.reason_feature_low ? 1 : 0) << ","
          << (adaptive_decision.reason_parallax_low ? 1 : 0) << ","
          << (adaptive_decision.reason_parallax_high ? 1 : 0) << ","
          << adaptive_decision.hysteresis_state << ","
          << adaptive_decision.hold_timer_s << ","
          << adaptive_decision.base_stride << ","
          << adaptive_decision.filtered_height_m << ","
          << adaptive_decision.filtered_speed_mps << ","
          << adaptive_decision.filtered_parallax_px << ","
          << adaptive_decision.normalized_parallax_px << ","
          << adaptive_decision.parallax_target_stride << ","
          << adaptive_decision.effective_height_m << ","
          << adaptive_decision.filtered_gyro_radps << ","
          << (adaptive_decision.severe_turn ? 1 : 0) << ","
          << (adaptive_decision.severe_feature_low ? 1 : 0) << ","
          << (adaptive_decision.severe_parallax_high ? 1 : 0) << ","
          << (adaptive_decision.force_fullrate_tracking ? 1 : 0) << ","
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
        bool cutoff_reached = (args.gps_cutoff_time > 0.0 && t_cam > args.gps_cutoff_time);
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
            double alt_raw = xyz_raw(2);
            sys->feed_measurement_gps_ground_plane(t_gps, alt_raw, args.gps_alt_sigma,
                                                   args.gps_alt_ground_plane_rangefinder);
          } else if (args.gps_alt_relative) {
            double alt_raw = xyz_raw(2);
            sys->feed_measurement_gps_altitude_relative(t_gps, alt_raw, args.gps_alt_sigma,
                                                         args.gps_alt_chi2, args.gps_alt_also_vz);
          } else {
            if (std::isnan(gps_alt_ref)) {
              gps_alt_ref = xyz_raw(2);
              auto state_ref = sys->get_state();
              gps_alt_vio_ref = state_ref->_imu->pos()(2);
              PRINT_INFO(GREEN "[GPS-ALT]: bootstrap gps_ref=%.2f vio_ref=%.2f at t=%.3f\n" RESET,
                         gps_alt_ref, gps_alt_vio_ref, t_gps);
            }
            double alt_z = xyz_raw(2) - gps_alt_ref + gps_alt_vio_ref;
            sys->feed_measurement_gps_altitude(t_gps, alt_z, args.gps_alt_sigma,
                                               args.gps_alt_chi2, args.gps_alt_also_vz);
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
      if (args.pose_repair_sim_gps) {
        if (pose_repair_next_time < 0.0 && t_init_done >= 0.0)
          pose_repair_next_time = t_init_done + args.pose_repair_period_s;

        bool repair_gps_near = false;
        size_t repair_gps_i = gps_i;
        double repair_gps_time = -1.0;
        double repair_gps_age = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector3d repair_gps_xyz = Eigen::Vector3d::Zero();
        if (!gps.empty()) {
          repair_gps_i = gps_i;
          if (repair_gps_i + 1 < gps.size() &&
              std::fabs(gps[repair_gps_i + 1].timestamp - t_cam) <
                  std::fabs(gps[repair_gps_i].timestamp - t_cam)) {
            repair_gps_i++;
          }
          repair_gps_time = gps[repair_gps_i].timestamp;
          repair_gps_age = std::fabs(repair_gps_time - t_cam);
          repair_gps_near = repair_gps_age <= args.pose_repair_max_gps_age_s;
          repair_gps_xyz = gps[repair_gps_i].xyz;
        }

        if (!pose_repair_ref_valid && repair_gps_near) {
          pose_repair_gps_ref = repair_gps_xyz;
          pose_repair_vio_ref = sys->get_state()->_imu->pos();
          pose_repair_ref_valid = true;
          PRINT_INFO(GREEN "[POSE-REPAIR] reference bootstrap t=%.3f gps=(%.2f %.2f %.2f) vio=(%.2f %.2f %.2f)\n" RESET,
                     repair_gps_time,
                     pose_repair_gps_ref.x(), pose_repair_gps_ref.y(), pose_repair_gps_ref.z(),
                     pose_repair_vio_ref.x(), pose_repair_vio_ref.y(), pose_repair_vio_ref.z());
        }

        if (pose_repair_next_time >= 0.0 && t_cam + 1e-9 >= pose_repair_next_time) {
          const double scheduled_time = pose_repair_next_time;
          bool restart_requested = false;
          std::string repair_restart_reason;
          std::string decision = "SKIP_NO_GPS";
          bool accepted = false;
          double repair_gps_speed = cur_gps_speed;
          Eigen::Vector3d anchor_pos_log = Eigen::Vector3d::Constant(
              std::numeric_limits<double>::quiet_NaN());
          std::string restart_init_source_this = "";
          VioManager::PoseAnchorLastUpdate pd;

          if (!repair_gps_near) {
            pose_repair_next_time = t_cam + 1.0;
          } else if (!pose_repair_ref_valid) {
            decision = "SKIP_NO_REFERENCE";
            pose_repair_next_time = t_cam + 1.0;
          } else {
            Eigen::Vector3d anchor_pos = repair_gps_xyz - pose_repair_gps_ref + pose_repair_vio_ref;
            double repair_course_yaw = ref_course_yaw;
            bool repair_yaw_valid = reference_course_yaw_at(t_cam, repair_course_yaw);
            double repair_imu_yaw = repair_course_yaw;
            if (repair_gps_i > 0) {
              const auto &g0 = gps[repair_gps_i - 1];
              const auto &g1 = gps[repair_gps_i];
              const double dtg = g1.timestamp - g0.timestamp;
              if (dtg > 1e-6)
                repair_gps_speed = (g1.xyz - g0.xyz).norm() / dtg;
            }
            repair_yaw_valid = repair_yaw_valid &&
                               repair_gps_speed >= args.pose_repair_min_speed_mps;
            if (repair_yaw_valid)
              repair_imu_yaw = imu_yaw_from_course_yaw_deg(repair_course_yaw);
            anchor_pos_log = anchor_pos;

            FCInitState trusted_repair_fc;
            bool trusted_repair_fc_valid = false;
            {
              auto repair_state = sys->get_state();
              Eigen::Matrix3d repair_Rwi = Eigen::Matrix3d::Identity();
              Eigen::Vector3d repair_rpy = Eigen::Vector3d::Zero();
              if (repair_state && repair_state->_imu) {
                repair_Rwi = repair_state->_imu->Rot().transpose();
                if (repair_Rwi.allFinite())
                  repair_rpy = rpy_from_Rwi(repair_Rwi);
              }
              const double yaw_init =
                  repair_yaw_valid ? repair_imu_yaw * M_PI / 180.0 : repair_rpy.z();
              const Eigen::Matrix3d repair_Rwi_init =
                  Rwi_from_rpy(repair_rpy.x(), repair_rpy.y(), yaw_init);
              Eigen::Vector3d repair_vel = Eigen::Vector3d::Zero();
              const bool repair_vel_valid = reference_velocity_at(t_cam, repair_vel);
              trusted_repair_fc.timestamp = repair_gps_time;
              trusted_repair_fc.q_GtoI = ov_core::rot_2_quat(repair_Rwi_init.transpose());
              trusted_repair_fc.p_IinG = anchor_pos;
              trusted_repair_fc.v_IinG =
                  repair_vel_valid ? repair_vel
                                   : (repair_state && repair_state->_imu &&
                                              repair_state->_imu->vel().allFinite()
                                          ? repair_state->_imu->vel()
                                          : Eigen::Vector3d::Zero());
              // A trusted reinit is only requested after the current VIO
              // state has failed against the external anchor. Do not carry
              // the possibly-corrupted VIO biases into the new filter.
              trusted_repair_fc.bg = fc_init_state.bg;
              trusted_repair_fc.ba = fc_init_state.ba;
              trusted_repair_fc_valid =
                  trusted_repair_fc.p_IinG.allFinite() &&
                  trusted_repair_fc.v_IinG.allFinite() &&
                  trusted_repair_fc.q_GtoI.allFinite();
            }

            pose_repair_attempts++;
            accepted = sys->feed_measurement_pose_anchor(
                repair_gps_time, anchor_pos,
                repair_imu_yaw * M_PI / 180.0, repair_yaw_valid,
                args.pose_repair_pos_sigma,
                args.pose_repair_yaw_sigma_deg * M_PI / 180.0,
                args.pose_repair_gate_sigma,
                args.pose_repair_max_pos_correction,
                args.pose_repair_max_yaw_correction_deg * M_PI / 180.0);
            pd = sys->get_last_pose_anchor_update();
            decision = pd.decision;
            if (accepted) {
              pose_repair_accepts++;
              pose_repair_consecutive_failures = 0;
              pose_repair_severe_failures = 0;
              pose_repair_lost_suspect = false;
              pose_repair_lost_reason.clear();
              restart_init_source_this = "ekf_pose_anchor";
            } else {
              const bool anchor_rejected_by_consistency =
                  pd.decision == "REJECT_INNOVATION" ||
                  pd.decision == "REJECT_TRUST_REGION_POS" ||
                  pd.decision == "REJECT_TRUST_REGION_YAW";
              if (args.pose_repair_trusted_reinit &&
                  trusted_repair_fc_valid &&
                  anchor_rejected_by_consistency) {
                const std::string reset_source = repair_yaw_valid
                                                     ? "trusted_global_reset_gps_course"
                                                     : "trusted_global_reset_position";
                const std::string reject_decision = pd.decision;
                const bool reset_ok = sys->apply_trusted_pose_anchor_reset(
                    trusted_repair_fc, t_cam, repair_yaw_valid,
                    trusted_repair_fc.v_IinG.allFinite(), true);
                pd = sys->get_last_pose_anchor_update();
                if (reset_ok) {
                  repair_restart_reason = "trusted_pose_repair_reset_" + reject_decision;
                  restart_init_source_this = reset_source;
                  decision = "TRUSTED_RESET_AFTER_" + reject_decision;
                  accepted = true;
                  pose_repair_gps_ref = repair_gps_xyz;
                  pose_repair_vio_ref = trusted_repair_fc.p_IinG;
                  pose_repair_ref_valid = true;
                }
              }
              if (accepted) {
                pose_repair_accepts++;
                pose_repair_consecutive_failures = 0;
                pose_repair_severe_failures = 0;
                pose_repair_lost_suspect = false;
                pose_repair_lost_reason.clear();
              } else {
                pose_repair_rejects++;
                pose_repair_consecutive_failures++;
              }
            }

            if (!accepted && !restart_requested && args.restart_supervisor &&
                t_cam - last_restart_time >= args.restart_cooldown_s) {
              const bool severe_pos =
                  std::isfinite(pd.pos_residual_norm) &&
                  pd.pos_residual_norm > args.restart_pos_error_m;
              const bool severe_yaw =
                  pd.yaw_used && std::isfinite(pd.yaw_residual_deg) &&
                  std::fabs(pd.yaw_residual_deg) > args.restart_yaw_error_deg;
              const bool repeated_fail =
                  pose_repair_consecutive_failures >= args.restart_consecutive_repair_failures;
              const bool severe_anchor = severe_pos || severe_yaw;
              if (severe_anchor) {
                pose_repair_severe_failures++;
              }
              const bool repeated_severe =
                  pose_repair_severe_failures >= args.restart_consecutive_repair_failures;
              if (severe_anchor || repeated_fail) {
                std::ostringstream rs;
                bool have_reason = false;
                if (repeated_fail) {
                  rs << "repair_failures_" << pose_repair_consecutive_failures;
                  have_reason = true;
                }
                if (severe_pos) {
                  if (have_reason) rs << "+";
                  rs << "pos_error";
                  have_reason = true;
                }
                if (severe_yaw) {
                  if (have_reason) rs << "+";
                  rs << "yaw_error";
                }
                repair_restart_reason = rs.str();
                pose_repair_lost_suspect = true;
                pose_repair_lost_reason = repair_restart_reason;
                pose_repair_lost_time = t_cam;
                // A sparse global/reference match can be wrong or frame-inconsistent.
                // Treat its residual as a lost suspect, and require independent
                // visual-health confirmation before hard restart.
                const bool visual_recent =
                    visual_lost_credit >= 5 && t_cam - last_visual_lost_time <= 3.0;
                if (args.restart_on_pose_repair && repeated_severe && visual_recent) {
                  restart_requested = true;
                  restart_pending = true;
                  restart_reason = repair_restart_reason + "+visual_lost";
                }
              }
            }
            pose_repair_next_time = t_cam + args.pose_repair_period_s;
          }

          if (pose_repair_out.is_open()) {
            pose_repair_out
                << t_cam << "," << scheduled_time << ","
                << repair_gps_time << "," << repair_gps_age << ","
                << decision << "," << (accepted ? 1 : 0) << ","
                << pd.pos_residual_norm << "," << pd.yaw_residual_deg << ","
                << pd.nis << "," << pd.gate_ratio << ","
                << pd.predicted_pos_correction_norm << ","
                << pd.predicted_yaw_correction_deg << ","
                << repair_gps_speed << ","
                << anchor_pos_log.x() << "," << anchor_pos_log.y() << "," << anchor_pos_log.z() << ","
                << pose_repair_consecutive_failures << ","
                << (restart_requested ? 1 : 0) << ","
                << repair_restart_reason << ","
                << restart_init_source_this << "\n";
            pose_repair_out.flush();
          }
        }
      }
      auto state = sys->get_state();
      Eigen::Matrix3d R_wi = state->_imu->Rot().transpose();
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
      if (raw_out.is_open()) {
        raw_out << t_cam << ' ' << p_wi.x() << ' ' << p_wi.y() << ' ' << p_wi.z() << ' ' << q.x() << ' '
                << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      }
      if (nav_out.is_open() && nav_frame.valid) {
        const Eigen::Matrix3d R_Gnav_I = nav_frame.R_Gnav_W0 * R_wi;
        const Eigen::Vector3d p_Gnav_I = nav_frame.R_Gnav_W0 * p_wi + nav_frame.p_Gnav_W0;
        const Eigen::Vector3d v_Gnav_I = nav_frame.R_Gnav_W0 * v_wi;
        Eigen::Quaterniond q_nav(R_Gnav_I);
        nav_out << t_cam << ' '
                << p_Gnav_I.x() << ' ' << p_Gnav_I.y() << ' ' << p_Gnav_I.z() << ' '
                << v_Gnav_I.x() << ' ' << v_Gnav_I.y() << ' ' << v_Gnav_I.z() << ' '
                << q_nav.x() << ' ' << q_nav.y() << ' ' << q_nav.z() << ' ' << q_nav.w() << '\n';
      }

      // Bias debug log
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
      if (args.restart_supervisor && do_cam_feed &&
          t_cam - last_restart_time >= args.restart_cooldown_s) {
        bool state_fault = !p_wi.allFinite() || !v_wi.allFinite() ||
                           !bg.allFinite() || !ba.allFinite() ||
                           !q.coeffs().allFinite();
        if (!state_fault) {
          Eigen::MatrixXd P_health = StateHelper::get_full_covariance(state);
          state_fault = !P_health.allFinite();
        }
        if (state_fault) {
          restart_pending = true;
          restart_reason = "state_health_fault";
        }
      }
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

        const bool low_tracking =
            klt_raw_now > 0 && tracked_now >= 0 && tracked_now < 15;
        const bool poor_msckf_update =
            mstats.n_features_in > 0 &&
            mstats.n_accepted < 5 &&
            mstats.n_chi2_rejected >= args.diag_chi2_trigger;
        const bool slam_collapse =
            slam_dropped >= 3 && (slam_count + slam_dropped) >= 3 && slam_count < 3;
        const bool visual_lost_frame =
            low_tracking || poor_msckf_update ||
            (tracked_now < 20 && mstats.n_accepted < 5 && slam_count == 0);
        if (visual_lost_frame || slam_collapse) {
          visual_lost_credit = std::min(25, visual_lost_credit + 1);
          last_visual_lost_time = t_cam;
        } else {
          visual_lost_credit = std::max(0, visual_lost_credit - 1);
        }
        if (args.restart_supervisor && pose_repair_lost_suspect &&
            !restart_pending &&
            t_cam - last_restart_time >= args.restart_cooldown_s) {
          const bool anchor_recent = t_cam - pose_repair_lost_time <= 10.0;
          const bool visual_confirmed =
              visual_lost_credit >= 5 && t_cam - last_visual_lost_time <= 3.0;
          if (args.restart_on_pose_repair &&
              anchor_recent && visual_confirmed &&
              pose_repair_severe_failures >= args.restart_consecutive_repair_failures) {
            restart_pending = true;
            restart_reason = pose_repair_lost_reason + "+visual_lost";
          }
        }

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

      if (restart_pending && (args.restart_supervisor || restart_anchor_override_valid) &&
          t_cam - last_restart_time >= args.restart_cooldown_s) {
        VioManagerOptions restart_params = params;
        restart_params.vio_yaw_update_diag_path.clear();
        restart_params.visual_obs_diag_path.clear();
        auto next_sys = std::make_shared<VioManager>(restart_params);
        configure_after_restart(next_sys);

        bool did_gps_init = false;
        bool did_gps_vel_init = false;
        if (restart_anchor_override_valid) {
          if (args.restart_visual_settle_s > 0.0)
            next_sys->set_visual_skip_window(t_cam, t_cam + args.restart_visual_settle_s);
          next_sys->set_gps_alt_joseph_update(true);
          next_sys->set_gps_alt_min_t_after_init(0.0);
          next_sys->set_gps_alt_max_res_gate(1e9);
          next_sys->set_gps_alt_guard_dxy_max(-1.0);
          next_sys->set_gps_alt_guard_kxy_ratio_max(-1.0);
          next_sys->set_gps_alt_guard_dbias_max(-1.0);
          next_sys->initialize_with_fc_state(
              restart_anchor_fc,
              std::max(args.init_att_sigma_deg, args.pose_repair_yaw_sigma_deg) * M_PI / 180.0,
              args.init_vel_sigma,
              std::max(args.init_pos_sigma, args.pose_repair_pos_sigma),
              args.init_bg_sigma,
              args.init_ba_sigma,
              t_cam);
          if (imu_i > 0 && imu_i - 1 < imu.size()) {
            ov_core::ImuData seed_imu;
            seed_imu.timestamp = imu[imu_i - 1].timestamp;
            seed_imu.wm = imu[imu_i - 1].gyro;
            seed_imu.am = imu[imu_i - 1].accel;
            next_sys->feed_measurement_imu(seed_imu);
          }
          if (restart_anchor_gps_valid) {
            pose_repair_gps_ref = restart_anchor_gps_xyz;
            pose_repair_vio_ref = restart_anchor_fc.p_IinG;
            pose_repair_ref_valid = true;
          }
          did_gps_init = true;
          did_gps_vel_init = restart_anchor_fc.v_IinG.allFinite();
          if (restart_init_source.empty())
            restart_init_source = "trusted_pose_anchor";
        } else if (args.restart_with_gps_init) {
          bool restart_gps_near = false;
          size_t restart_gps_i = gps_i;
          double restart_gps_time = -1.0;
          Eigen::Vector3d restart_gps_xyz = Eigen::Vector3d::Zero();
          if (!gps.empty()) {
            restart_gps_i = gps_i;
            if (restart_gps_i + 1 < gps.size() &&
                std::fabs(gps[restart_gps_i + 1].timestamp - t_cam) <
                    std::fabs(gps[restart_gps_i].timestamp - t_cam)) {
              restart_gps_i++;
            }
            restart_gps_time = gps[restart_gps_i].timestamp;
            restart_gps_xyz = gps[restart_gps_i].xyz;
            restart_gps_near =
                std::fabs(restart_gps_time - t_cam) <= args.pose_repair_max_gps_age_s;
          }
          if (restart_gps_near) {
            double restart_course_yaw_deg = ref_course_yaw;
            bool restart_yaw_valid = reference_course_yaw_at(t_cam, restart_course_yaw_deg);
            double restart_yaw_deg = restart_yaw_valid
                                         ? imu_yaw_from_course_yaw_deg(restart_course_yaw_deg)
                                         : restart_course_yaw_deg;
            if (!restart_yaw_valid)
              restart_yaw_deg = yaw_from_Rwi(R_wi) * 180.0 / M_PI;
            Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
            if (R_wi.allFinite())
              rpy = rpy_from_Rwi(R_wi);
            const double yaw_init = std::isfinite(restart_yaw_deg)
                                        ? restart_yaw_deg * M_PI / 180.0
                                        : (std::isfinite(rpy.z()) ? rpy.z() : 0.0);
            const double roll_init = std::isfinite(rpy.x()) ? rpy.x() : 0.0;
            const double pitch_init = std::isfinite(rpy.y()) ? rpy.y() : 0.0;
            const Eigen::Matrix3d Rwi_init = Rwi_from_rpy(roll_init, pitch_init, yaw_init);
            Eigen::Vector3d restart_vel = Eigen::Vector3d::Zero();
            did_gps_vel_init = reference_velocity_at(t_cam, restart_vel);
            FCInitState restart_fc;
            restart_fc.timestamp = restart_gps_time;
            restart_fc.q_GtoI = ov_core::rot_2_quat(Rwi_init.transpose());
            restart_fc.p_IinG = pose_repair_ref_valid
                                    ? (restart_gps_xyz - pose_repair_gps_ref + pose_repair_vio_ref)
                                    : restart_gps_xyz;
            restart_fc.v_IinG = did_gps_vel_init ? restart_vel
                                                  : (v_wi.allFinite() ? v_wi : Eigen::Vector3d::Zero());
            // A hard restart means the prior VIO state was deemed lost. Do not
            // carry old bias estimates across the new root state.
            restart_fc.bg = Eigen::Vector3d::Zero();
            restart_fc.ba = Eigen::Vector3d::Zero();
            next_sys->initialize_with_fc_state(
                restart_fc,
                args.init_att_sigma_deg * M_PI / 180.0,
                args.init_vel_sigma,
                args.init_pos_sigma,
                args.init_bg_sigma,
                args.init_ba_sigma,
                t_cam);
            if (imu_i > 0 && imu_i - 1 < imu.size()) {
              ov_core::ImuData seed_imu;
              seed_imu.timestamp = imu[imu_i - 1].timestamp;
              seed_imu.wm = imu[imu_i - 1].gyro;
              seed_imu.am = imu[imu_i - 1].accel;
              next_sys->feed_measurement_imu(seed_imu);
            }
            pose_repair_gps_ref = restart_gps_xyz;
            pose_repair_vio_ref = restart_fc.p_IinG;
            pose_repair_ref_valid = true;
            did_gps_init = true;
            restart_init_source = "nearest_gps_course";
          }
        }

        sys = next_sys;
        restart_count++;
        last_restart_time = t_cam;
        restart_pending = false;
        PRINT_WARNING(YELLOW "[RESTART-SUPERVISOR] restart #%zu at t=%.3f reason=%s gps_init=%d gps_vel=%d init_source=%s\n" RESET,
                      restart_count, t_cam, restart_reason.c_str(),
                      did_gps_init ? 1 : 0, did_gps_vel_init ? 1 : 0,
                      restart_init_source.empty() ? "none" : restart_init_source.c_str());
        restart_reason.clear();
        restart_init_source.clear();
        restart_anchor_override_valid = false;
        restart_anchor_gps_valid = false;
        pose_repair_consecutive_failures = 0;
        pose_repair_severe_failures = 0;
        pose_repair_lost_suspect = false;
        pose_repair_lost_reason.clear();
        visual_lost_credit = 0;
        last_visual_lost_time = -std::numeric_limits<double>::infinity();
        pose_repair_next_time = t_cam + args.pose_repair_period_s;
        gps_alt_ref = std::numeric_limits<double>::quiet_NaN();
        gps_xy_ref = Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
        gps_xy_vio_ref = Eigen::Vector2d::Zero();
        adaptive_have_vio_height_origin = false;
        prev_p_wi_valid = false;
        vio_for_align.clear();
        if (did_gps_init)
          t_init_done = t_cam;
        else
          t_init_done = -1.0;
      }
    }

    // image for right-top panel
    cv::Mat hist = sys->get_historical_viz_image();
    if (!hist.empty())
      dash.update_image(t_cam, hist);
    else
      dash.update_image(t_cam, img0);
    {
      ov_core::TrackerWarpVizPacket pkt;
      if (sys->get_warp_viz_packet(0, pkt)) {
        dash.update_tracker_flow(pkt.prev_image_for_viz, pkt.prev_pts_for_viz,
                                 pkt.curr_pts_raw, pkt.curr_raw_image,
                                 pkt.warp_active, pkt.t_curr);
        dash.update_tracker_diag(pkt.t_curr, pkt.n_klt_attempted, pkt.n_newly_detected);
      }
    }

    if (!args.video_cam_path.empty() && !hist.empty()) {
      cv::Mat cam_frame;
      if (hist.channels() == 1)
        cv::cvtColor(hist, cam_frame, cv::COLOR_GRAY2BGR);
      else
        cam_frame = hist.clone();
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
  if (pose_repair_out.is_open()) {
    pose_repair_out.flush();
    if (!pose_repair_out.good()) {
      PRINT_ERROR(RED "[POSE-REPAIR] CSV write failed: %s\n" RESET,
                  args.pose_repair_log_path.c_str());
      return EXIT_FAILURE;
    }
    pose_repair_out.close();
    PRINT_INFO(GREEN "[POSE-REPAIR] log=%s attempts=%zu accepted=%zu rejected=%zu restarts=%zu\n" RESET,
               args.pose_repair_log_path.c_str(), pose_repair_attempts,
               pose_repair_accepts, pose_repair_rejects, restart_count);
  }
  if (cam_writer.isOpened())
    cam_writer.release();
  PRINT_INFO(GREEN "[ros-free] done. trajectory written to %s (frames=%d, aligned_pairs=%d)\n" RESET,
             args.output_path.c_str(), frame_idx, align_fit_count);
  PRINT_INFO(GREEN "[ros-free] raw=%s nav=%s nav_metadata=%s nav_valid=%d\n" RESET,
             args.output_raw_path.c_str(), args.output_nav_path.c_str(),
             args.nav_frame_metadata_path.c_str(), nav_frame.valid ? 1 : 0);
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
