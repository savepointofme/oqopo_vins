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
#include <array>
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
#include <unordered_map>
#include <unordered_set>
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
#include "ros_free/AdaptiveVisualScheduler.h"
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

VisualMotionMetrics tracker_motion_metrics(
    const ov_core::TrackerWarpVizPacket &packet) {
  VisualMotionMetrics out;
  if (!packet.valid)
    return out;
  const auto &previous_raw = packet.prev_pts_raw.empty()
                                 ? packet.prev_pts_for_viz
                                 : packet.prev_pts_raw;
  const auto &rotation_prediction =
      packet.prev_pts_rotation_compensated.empty()
          ? packet.prev_pts_for_viz
          : packet.prev_pts_rotation_compensated;
  const size_t count = std::min(
      {previous_raw.size(), rotation_prediction.size(),
       packet.curr_pts_raw.size(), packet.feature_ids.size()});
  out.dt_s = packet.t_curr - packet.t_prev;
  out.previous_tracks = std::max(packet.n_klt_attempted,
                                 static_cast<int>(previous_raw.size()));
  out.current_tracks = static_cast<int>(packet.curr_pts_raw.size());
  out.common_tracks = static_cast<int>(count);
  out.survival_ratio =
      static_cast<double>(count) /
      static_cast<double>(std::max(1, out.previous_tracks));
  std::vector<double> raw;
  std::vector<double> rotation;
  std::vector<double> compensated;
  std::vector<double> ages;
  std::vector<double> weights;
  std::vector<Eigen::Vector2d> current_pixels;
  raw.reserve(count);
  rotation.reserve(count);
  compensated.reserve(count);
  ages.reserve(count);
  weights.reserve(count);
  current_pixels.reserve(count);
  for (size_t index = 0; index < count; ++index) {
    const cv::Point2f &old = previous_raw[index];
    const cv::Point2f &predicted = rotation_prediction[index];
    const cv::Point2f &current = packet.curr_pts_raw[index];
    raw.push_back(std::hypot(current.x - old.x, current.y - old.y));
    rotation.push_back(
        std::hypot(predicted.x - old.x, predicted.y - old.y));
    compensated.push_back(std::hypot(current.x - predicted.x,
                                     current.y - predicted.y));
    ages.push_back(1.0);
    const double nx = packet.image_width > 0
                          ? current.x / static_cast<double>(packet.image_width)
                          : 0.5;
    const double ny = packet.image_height > 0
                          ? current.y / static_cast<double>(packet.image_height)
                          : 0.5;
    const double border_distance =
        std::min(std::min(nx, 1.0 - nx), std::min(ny, 1.0 - ny));
    weights.push_back(
        std::max(0.20, std::min(1.0, border_distance / 0.08)));
    current_pixels.emplace_back(current.x, current.y);
  }
  finalize_visual_motion_metrics(
      out, raw, rotation, compensated, ages, weights, current_pixels,
      packet.image_width, packet.image_height);
  return out;
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

std::string json_scalar(double value) {
  if (!std::isfinite(value))
    return "null";
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

void write_json_error_state(
    std::ostream &output, const Eigen::Matrix<double, 15, 1> &value) {
  output << "[";
  for (int index = 0; index < value.rows(); ++index) {
    if (index)
      output << ", ";
    output << json_scalar(value(index));
  }
  output << "]";
}

template <typename Derived>
void write_json_eigen_vector(std::ostream &output,
                             const Eigen::MatrixBase<Derived> &value) {
  output << "[";
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    if (index)
      output << ", ";
    output << json_scalar(value(index));
  }
  output << "]";
}

template <typename Derived>
std::string csv_eigen_vector(const Eigen::MatrixBase<Derived> &value) {
  std::ostringstream output;
  output << std::setprecision(17);
  for (Eigen::Index index = 0; index < value.size(); ++index) {
    if (index)
      output << ';';
    if (std::isfinite(value(index)))
      output << value(index);
  }
  return output.str();
}

template <std::size_t N>
void write_json_int_array(std::ostream &output,
                          const std::array<int, N> &values) {
  output << "[";
  for (std::size_t index = 0; index < N; ++index) {
    if (index)
      output << ", ";
    output << values[index];
  }
  output << "]";
}

template <std::size_t N>
void write_json_bool_array(std::ostream &output,
                           const std::array<bool, N> &values) {
  output << "[";
  for (std::size_t index = 0; index < N; ++index) {
    if (index)
      output << ", ";
    output << (values[index] ? "true" : "false");
  }
  output << "]";
}

std::string sibling_path(const std::string &base_path, const std::string &name) {
  fs::path p(base_path);
  fs::path parent = p.parent_path();
  return (parent.empty() ? fs::path(name) : parent / name).string();
}

struct NavFrameState {
  bool valid = false;
  bool metadata_written = false;
  double dataset_first_imu_timestamp = -1.0;
  double requested_start_offset_s = 0.0;
  double trim_boundary_timestamp = -1.0;
  double init_camera_timestamp = -1.0;
  double first_emitted_timestamp = -1.0;
  double selected_fc_timestamp = -1.0;
  double camera_to_imu_time_offset_s = 0.0;
  double fc_time_offset = 0.0;
  double gps_timestamp = -1.0;
  double gps_age = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d R_Gnav_W0 = Eigen::Matrix3d::Identity();
  Eigen::Vector3d p_Gnav_W0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_Gnav_GPS0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_Gnav_I0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_W0_I0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_W0_Ifirst = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_W0_Ifirst = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_ItoW0_first = (Eigen::Vector4d() << 0, 0, 0, 1).finished();
  Eigen::Vector3d fc_source_position = Eigen::Vector3d::Zero();
  std::string fc_position_frame;
  std::string position_source;
  std::string initialization_mode;
  std::string fc_init_requested_level;
  std::string fc_init_applied_level;
  std::string fc_init_selection_method;
  std::string fc_init_status;
  bool fc_init_fallback_applied = false;
  bool fc_init_future_data_used = false;
  double fc_init_available_time = std::numeric_limits<double>::quiet_NaN();
  double init_att_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  double init_vel_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double init_pos_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double init_bg_sigma_rad_s = std::numeric_limits<double>::quiet_NaN();
  double init_ba_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d gps_antenna_in_imu = Eigen::Vector3d::Zero();
};

struct Args;
void ensure_parent_directory(const std::string &path) {
  if (path.empty())
    return;
  const fs::path output_path(path);
  if (!output_path.parent_path().empty())
    fs::create_directories(output_path.parent_path());
}
void write_nav_metadata(const std::string &path,
                        const NavFrameState &nav,
                        const Args &args);

Eigen::Vector3d parse_declared_vector3(const std::string &value,
                                       const std::string &name) {
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::replace(normalized.begin(), normalized.end(), ';', ' ');
  std::stringstream stream(normalized);
  Eigen::Vector3d vector;
  if (!(stream >> vector.x() >> vector.y() >> vector.z()))
    throw std::runtime_error("invalid three-vector declaration: " + name);
  std::string trailing;
  if (stream >> trailing)
    throw std::runtime_error("extra values in three-vector declaration: " + name);
  return vector;
}

Eigen::Matrix3d parse_declared_matrix3(const std::string &value,
                                       const std::string &name) {
  std::string normalized = value;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');
  std::replace(normalized.begin(), normalized.end(), ';', ' ');
  std::stringstream stream(normalized);
  Eigen::Matrix3d matrix;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      if (!(stream >> matrix(row, col)))
        throw std::runtime_error("invalid 3x3 declaration: " + name);
  std::string trailing;
  if (stream >> trailing)
    throw std::runtime_error("extra values in 3x3 declaration: " + name);
  if (!matrix.allFinite() ||
      (matrix.transpose() * matrix - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
      std::fabs(matrix.determinant() - 1.0) > 1e-6)
    throw std::runtime_error(name + " is not a proper rotation matrix");
  return matrix;
}

double parse_declared_scalar(const std::string &value,
                             const std::string &name) {
  std::stringstream stream(value);
  double scalar = std::numeric_limits<double>::quiet_NaN();
  if (!(stream >> scalar) || !std::isfinite(scalar))
    throw std::runtime_error("invalid scalar declaration: " + name);
  std::string trailing;
  if (stream >> trailing)
    throw std::runtime_error("extra values in scalar declaration: " + name);
  return scalar;
}

bool valid_post_alignment_visual_roi(const std::string &mode) {
  return mode == "full" || mode == "left" || mode == "center" ||
         mode == "right" || mode == "top" || mode == "bottom" ||
         mode == "dynamic_turn";
}

void apply_post_alignment_visual_roi(cv::Mat &mask, const std::string &mode) {
  if (mode == "full")
    return;
  if (mask.empty() || mask.type() != CV_8UC1)
    throw std::runtime_error("post-alignment visual ROI requires a CV_8UC1 mask");

  mask.setTo(cv::Scalar(255));
  cv::Rect keep;
  if (mode == "left") {
    keep = cv::Rect(0, 0, std::max(1, mask.cols / 2), mask.rows);
  } else if (mode == "right") {
    const int x0 = mask.cols / 2;
    keep = cv::Rect(x0, 0, mask.cols - x0, mask.rows);
  } else if (mode == "top") {
    keep = cv::Rect(0, 0, mask.cols, std::max(1, mask.rows / 2));
  } else if (mode == "bottom") {
    const int y0 = mask.rows / 2;
    keep = cv::Rect(0, y0, mask.cols, mask.rows - y0);
  } else {
    const int x0 = mask.cols / 5;
    const int y0 = mask.rows / 5;
    keep = cv::Rect(x0, y0, mask.cols - 2 * x0, mask.rows - 2 * y0);
  }
  mask(keep).setTo(cv::Scalar(0));
}

struct Args {
  std::string config_path;
  std::string dataset_dir;
  std::string gps_path;
  std::string gt_path;
  std::string output_path = "traj_ros_free.txt";
  std::string output_raw_path;
  std::string output_nav_path;
  std::string nav_frame_metadata_path;
  std::string canonical_init_state_path;
  std::string online_alignment_metadata_path;
  std::string online_alignment_window_trace_path;
  bool online_alignment_disable_visual = false;
  bool online_alignment_navigation_allow_without_visual = false;
  // The joint graph estimates both biases. The causal candidate filter has
  // only FC position/velocity observations, so production keeps the graph
  // bias estimates instead of changing them through cross-covariance alone.
  bool online_alignment_retain_bg_prior = true;
  bool online_alignment_retain_ba_prior = true;
  bool online_alignment_release_attitude_from_fc = false;
  bool online_alignment_diagnostic_graph_q_fc_pv_zero_bias = false;
  bool online_alignment_diagnostic_never_anchor = false;
  std::string online_alignment_release_policy = "practical_navigation_start";
  double online_alignment_visual_perturbation_px = 0.0;
  double online_alignment_visual_perturbation_fraction = 0.0;
  std::string initialization_mode;
  std::string init_from_fc_path;
  std::string init_from_fc_position_frame;
  std::string fc_init_level = "I0";
  std::string fc_init_fallback = "fail_closed";
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
  bool gps_alt_guard_fallback_joseph_pz = false;
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
  double init_from_fc_max_bracket_gap = 0.35;
  double init_window_s = 5.0;
  int init_window_min_samples = 12;
  double init_window_max_source_gap = 0.35;
  double init_window_max_attitude_p95_deg = 3.0;
  double init_window_huber_delta_deg = 1.5;
  double init_window_min_speed_mps = 0.0;
  double init_max_bg_norm_rad_s = 0.2;
  double init_max_ba_norm_mps2 = 2.0;
  Eigen::Vector3d gps_antenna_in_imu = Eigen::Vector3d::Zero(); // p_I_GPS meters
  bool dashboard_enabled = true;
  bool show = true;
  std::string dash_title = "OpenVINS ROS-free Dashboard";
  int dash_every = 1;
  int cam_subsample = 1;         // --camera-frame-stride/--cam-subsample N: feed only 1 of every N camera frames to VIO
  int post_alignment_camera_frame_stride = 0; // Diagnostic fixed stride after P4 release; 0 keeps cam_subsample.
  std::string post_alignment_visual_roi = "full"; // Diagnostic KLT/MSCKF/SLAM image ROI after P4 release.
  std::string dynamic_turn_roi_diag_path;
  std::string agl_scene_scale_mode = "off";
  std::string agl_scene_scale_diag_path;
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
  double post_alignment_gyro_z_perturbation_rad_s = 0.0; // Diagnostic-only board gyro-z perturbation after P4 release.
  double post_alignment_gyro_z_scale = 1.0; // Diagnostic-only board gyro-z scale after P4 release.
  double post_alignment_gyro_z_scale_min_abs_rad_s = 0.0; // Diagnostic-only scale activation threshold.
  Eigen::Vector3d post_alignment_camera_extrinsic_left_rotvec_deg =
      Eigen::Vector3d::Zero(); // Diagnostic-only; applied after P4 release.
  bool post_alignment_fc_yaw_aid = false; // Diagnostic-only causal FC-attitude yaw anchor.
  double post_alignment_fc_yaw_aid_period_s = 2.0;
  double post_alignment_fc_yaw_aid_sigma_deg = 2.0;
  double post_alignment_fc_yaw_aid_gate_sigma = 5.0;
  std::string post_alignment_fc_yaw_aid_log_path;
  bool disable_slam_features = false;              // Diagnostic-only MSCKF-only backend ablation.
  bool online_alignment_use_cli_init_covariance = false; // Diagnostic-only: replace P4 covariance, preserve nominal state.
  bool online_alignment_local_estimator_origin = false; // Preserve absolute output while centering internal W0 at P4 release.
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
  double msckf_yaw_freeze_t0 = -1.0;     // --msckf-yaw-freeze-window start
  double msckf_yaw_freeze_t1 = -1.0;     // --msckf-yaw-freeze-window end
  double visual_yaw_gain_zero_t0 = -1.0;  // --visual-yaw-gain-zero-window start
  double visual_yaw_gain_zero_t1 = -1.0;  // --visual-yaw-gain-zero-window end
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

void write_canonical_init_state(const std::string &path,
                                const Args &args,
                                InitializationMode mode,
                                const FCInitResult &result,
                                const FCInitState &canonical_state,
                                double t_seed,
                                double attitude_sigma_rad,
                                double velocity_sigma_mps,
                                double position_sigma_m,
                                double gyro_bias_sigma_rad_s,
                                double accel_bias_sigma_mps2) {
  if (path.empty())
    return;
  fs::path output_path(path);
  if (!output_path.parent_path().empty())
    fs::create_directories(output_path.parent_path());
  std::ofstream output(path);
  if (!output.is_open())
    throw std::runtime_error("cannot write canonical init state: " + path);
  const auto write_vec3 = [&](const Eigen::Vector3d &value) {
    output << '[' << json_scalar(value.x()) << ", " << json_scalar(value.y())
           << ", " << json_scalar(value.z()) << ']';
  };
  const auto write_vec4 = [&](const Eigen::Vector4d &value) {
    output << '[' << json_scalar(value.x()) << ", " << json_scalar(value.y())
           << ", " << json_scalar(value.z()) << ", " << json_scalar(value.w())
           << ']';
  };
  const bool future_fc_data_used =
      std::isfinite(result.window.last_timestamp_s) &&
      result.window.last_timestamp_s > t_seed + 1e-9;
  const double initialization_available_time = future_fc_data_used
                                                   ? result.window.last_timestamp_s
                                                   : t_seed;
  output << "{\n"
         << "  \"schema_version\": \"canonical-init-state-v1\",\n"
         << "  \"initialization_mode\": \"" << initialization_mode_name(mode) << "\",\n"
         << "  \"requested_level\": \"" << fc_init_level_name(result.requested_level) << "\",\n"
         << "  \"applied_level\": \"" << fc_init_level_name(result.applied_level) << "\",\n"
         << "  \"selection_method\": \"" << json_escape(result.selection_method) << "\",\n"
         << "  \"status\": \"" << json_escape(result.status) << "\",\n"
         << "  \"fallback_applied\": " << (result.fallback_applied ? "true" : "false") << ",\n"
         << "  \"fallback_contract\": \"" << json_escape(args.fc_init_fallback) << "\",\n"
         << "  \"t_seed\": " << json_scalar(t_seed) << ",\n"
         << "  \"source_fc_file\": \"" << json_escape(args.init_from_fc_path) << "\",\n"
         << "  \"source_position_frame\": \"" << json_escape(args.init_from_fc_position_frame) << "\",\n"
         << "  \"canonical_frame\": \""
         << (mode == InitializationMode::FC_FULL_STATE ? "G_nav" : "W0_local") << "\",\n"
         << "  \"position_point\": \"board_imu\",\n"
         << "  \"attitude_reference_label\": \"FC attitude reference\",\n"
         << "  \"gps_reference_label\": \"GPS reference\",\n"
         << "  \"no_post_alignment\": true,\n"
         << "  \"fixed_fc_board_correction_deg\": null,\n"
         << "  \"T_C_I_policy\": \"fixed_locked_no_turn_dependent_update\",\n"
         << "  \"future_fc_attitude_data_used\": " << (future_fc_data_used ? "true" : "false") << ",\n"
         << "  \"formal_initialization_delayed_to_window_end\": false,\n"
         << "  \"initialization_available_time\": " << json_scalar(initialization_available_time) << ",\n"
         << "  \"causality_status\": \""
         << (future_fc_data_used ? "NON_CAUSAL_OFFLINE_DIAGNOSTIC" : "CAUSAL_AT_T_SEED") << "\",\n"
         << "  \"bracket\": {\n"
         << "    \"before_timestamp\": " << json_scalar(result.bracket.before.state.timestamp) << ",\n"
         << "    \"after_timestamp\": " << json_scalar(result.bracket.after.state.timestamp) << ",\n"
         << "    \"gap_s\": " << json_scalar(result.bracket.gap_s) << ",\n"
         << "    \"alpha\": " << json_scalar(result.bracket.alpha) << ",\n"
         << "    \"exact_source_row\": " << (result.bracket.exact_source_row ? "true" : "false") << ",\n"
         << "    \"before_source_row_index\": " << result.bracket.before.source_row_index << ",\n"
         << "    \"after_source_row_index\": " << result.bracket.after.source_row_index << ",\n"
         << "    \"before_source_row\": \"" << json_escape(result.bracket.before.source_line) << "\",\n"
         << "    \"after_source_row\": \"" << json_escape(result.bracket.after.source_line) << "\"\n"
         << "  },\n"
         << "  \"state\": {\n"
         << "    \"q_Gnav_to_I_xyzw\": ";
  write_vec4(canonical_state.q_GtoI);
  output << ",\n    \"v_I_in_Gnav_mps\": ";
  write_vec3(canonical_state.v_IinG);
  output << ",\n    \"p_I_in_Gnav_m\": ";
  write_vec3(canonical_state.p_IinG);
  output << ",\n    \"gyro_bias_rad_s\": ";
  write_vec3(canonical_state.bg);
  output << ",\n    \"accel_bias_mps2\": ";
  write_vec3(canonical_state.ba);
  output << "\n  },\n"
         << "  \"covariance_std\": {\n"
         << "    \"attitude_rad\": " << json_scalar(attitude_sigma_rad) << ",\n"
         << "    \"velocity_mps\": " << json_scalar(velocity_sigma_mps) << ",\n"
         << "    \"position_m\": " << json_scalar(position_sigma_m) << ",\n"
         << "    \"gyro_bias_rad_s\": " << json_scalar(gyro_bias_sigma_rad_s) << ",\n"
         << "    \"accel_bias_mps2\": " << json_scalar(accel_bias_sigma_mps2) << "\n"
         << "  },\n"
         << "  \"robust_window\": {\n"
         << "    \"sample_count\": " << result.window.sample_count << ",\n"
         << "    \"requested_duration_s\": " << json_scalar(result.window.requested_duration_s) << ",\n"
         << "    \"actual_duration_s\": " << json_scalar(result.window.actual_duration_s) << ",\n"
         << "    \"first_timestamp_s\": " << json_scalar(result.window.first_timestamp_s) << ",\n"
         << "    \"last_timestamp_s\": " << json_scalar(result.window.last_timestamp_s) << ",\n"
         << "    \"max_source_gap_s\": " << json_scalar(result.window.max_source_gap_s) << ",\n"
         << "    \"attitude_residual_rms_deg\": " << json_scalar(result.window.residual_rms_deg) << ",\n"
         << "    \"attitude_residual_p95_deg\": " << json_scalar(result.window.residual_p95_deg) << ",\n"
         << "    \"attitude_residual_max_deg\": " << json_scalar(result.window.residual_max_deg) << ",\n"
         << "    \"robust_inlier_fraction\": " << json_scalar(result.window.robust_inlier_fraction) << ",\n"
         << "    \"fitted_angular_rate_rad_s\": ["
         << json_scalar(result.window.fitted_angular_rate_rad_s.x()) << ", "
         << json_scalar(result.window.fitted_angular_rate_rad_s.y()) << ", "
         << json_scalar(result.window.fitted_angular_rate_rad_s.z()) << "],\n"
         << "    \"attitude_std_rad\": ["
         << json_scalar(result.window.attitude_std_rad.x()) << ", "
         << json_scalar(result.window.attitude_std_rad.y()) << ", "
         << json_scalar(result.window.attitude_std_rad.z()) << "],\n"
         << "    \"quality_passed\": " << (result.window.quality_passed ? "true" : "false") << ",\n"
         << "    \"quality_reason\": \"" << json_escape(result.window.quality_reason) << "\",\n"
         << "    \"bias_status\": \"" << json_escape(result.window.bias_status) << "\",\n"
         << "    \"bias_fallback_reason\": \"" << json_escape(result.window.bias_fallback_reason) << "\"\n"
         << "  }\n"
         << "}\n";
  if (!output.good())
    throw std::runtime_error("failed while writing canonical init state: " + path);
}

void write_online_alignment_metadata(const std::string &path,
                                     const OnlineAlignmentResult &result,
                                     const Args &args) {
  ensure_parent_directory(path);
  std::ofstream output(path);
  if (!output.is_open())
    throw std::runtime_error("cannot open online alignment metadata: " + path);
  output << std::setprecision(15);
  auto write_vec3 = [&](const Eigen::Vector3d &v) {
    output << "[" << json_scalar(v.x()) << ", " << json_scalar(v.y())
           << ", " << json_scalar(v.z()) << "]";
  };
  auto write_vec4 = [&](const Eigen::Vector4d &v) {
    output << "[" << json_scalar(v(0)) << ", " << json_scalar(v(1))
           << ", " << json_scalar(v(2)) << ", " << json_scalar(v(3))
           << "]";
  };
  auto write_vec15 = [&](const Eigen::Matrix<double, 15, 1> &v) {
    output << "[";
    for (Eigen::Index index = 0; index < v.rows(); ++index) {
      if (index)
        output << ", ";
      output << json_scalar(v(index));
    }
    output << "]";
  };
  auto write_matrix3 = [&](const Eigen::Matrix3d &R) {
    output << "[[" << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2)
           << "], [" << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2)
           << "], [" << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2)
           << "]]";
  };
  auto write_string_list = [&](const std::vector<std::string> &values) {
    output << "[";
    for (size_t i = 0; i < values.size(); ++i) {
      if (i)
        output << ", ";
      output << "\"" << json_escape(values[i]) << "\"";
    }
    output << "]";
  };
  const auto &d = result.diagnostics;
  output << "{\n"
         << "  \"schema\": \"openvins_online_multisensor_alignment_v14\",\n"
         << "  \"mode\": \"online_multisensor_alignment\",\n"
         << "  \"status\": \"" << alignment_readiness_name(result.readiness)
         << "\",\n"
         << "  \"readiness_level\": \""
         << alignment_readiness_name(result.readiness) << "\",\n"
         << "  \"release_policy\": \""
         << alignment_release_policy_name(result.release_policy) << "\",\n"
         << "  \"released_to_openvins\": "
         << (result.released_to_openvins ? "true" : "false") << ",\n"
         << "  \"readiness_reason\": \""
         << json_escape(d.readiness_reason) << "\",\n"
         << "  \"source_fc_stream\": \"" << json_escape(args.init_from_fc_path)
         << "\",\n"
         << "  \"requested_start_offset_s\": " << args.start_time << ",\n"
         << "  \"collection_start_s\": "
         << json_scalar(d.collection_start_time) << ",\n"
         << "  \"window_start_s\": " << d.window_start << ",\n"
         << "  \"t_init_s\": " << d.init_time << ",\n"
         << "  \"result_state_timestamp_s\": " << result.timestamp << ",\n"
         << "  \"solve_time_s\": " << json_scalar(d.solve_time) << ",\n"
         << "  \"decision_time_s\": " << json_scalar(d.decision_time)
         << ",\n"
         << "  \"navigation_ready_time_s\": "
         << json_scalar(d.navigation_ready_time) << ",\n"
         << "  \"full_alignment_ready_time_s\": "
         << json_scalar(d.full_alignment_ready_time) << ",\n"
         << "  \"first_openvins_output_time_s\": "
         << json_scalar(d.first_openvins_output_time) << ",\n"
         << "  \"navigation_startup_latency_s\": "
         << json_scalar(d.navigation_ready_time >= 0.0 &&
                                d.collection_start_time >= 0.0
                            ? d.navigation_ready_time - d.collection_start_time
                            : std::numeric_limits<double>::quiet_NaN())
         << ",\n"
         << "  \"full_alignment_latency_s\": "
         << json_scalar(d.full_alignment_ready_time >= 0.0 &&
                                d.collection_start_time >= 0.0
                            ? d.full_alignment_ready_time -
                                  d.collection_start_time
                            : std::numeric_limits<double>::quiet_NaN())
         << ",\n"
         << "  \"initialization_duration_s\": "
         << json_scalar(d.initialization_duration_s) << ",\n"
         << "  \"nonlinear_solve_attempt_count\": "
         << d.nonlinear_solve_attempt_count << ",\n"
         << "  \"shared_window_bias_model\": "
         << (d.shared_window_bias_model ? "true" : "false") << ",\n"
         << "  \"staged_solver_enabled\": "
         << (d.staged_solver_enabled ? "true" : "false") << ",\n"
         << "  \"stage1_solution_usable\": "
         << (d.stage1_solution_usable ? "true" : "false") << ",\n"
         << "  \"stage1_initial_cost\": " << json_scalar(d.stage1_initial_cost)
         << ",\n"
         << "  \"stage1_final_cost\": " << json_scalar(d.stage1_final_cost)
         << ",\n"
         << "  \"stage1_solver_iterations\": "
         << d.stage1_solver_iterations << ",\n"
         << "  \"stage1_solve_wall_time_s\": "
         << json_scalar(d.stage1_solve_wall_time_s) << ",\n"
         << "  \"final_stage_solve_wall_time_s\": "
         << json_scalar(d.final_stage_solve_wall_time_s) << ",\n"
         << "  \"successful_release_count\": "
         << d.successful_release_count << ",\n"
         << "  \"post_release_try_count\": "
         << d.post_release_try_count << ",\n"
         << "  \"solve_eligibility_skip_count\": "
         << d.solve_eligibility_skip_count << ",\n"
         << "  \"duplicate_window_skip_count\": "
         << d.duplicate_window_skip_count << ",\n"
         << "  \"selected_window_duration_s\": "
         << d.selected_window_duration_s << ",\n"
         << "  \"maximum_buffer_window_s\": "
         << d.maximum_buffer_window_s << ",\n"
         << "  \"window_fingerprint\": \""
         << json_escape(d.window_fingerprint) << "\",\n"
         << "  \"candidate_created_count\": "
         << d.candidate_created_count << ",\n"
         << "  \"candidate_rejected_count\": "
         << d.candidate_rejected_count << ",\n"
         << "  \"candidate_refinement_count\": "
         << d.candidate_refinement_count << ",\n"
         << "  \"gauge_window_observation_count\": "
         << d.gauge_window_observation_count << ",\n"
         << "  \"gauge_stability_required_s\": "
         << json_scalar(d.gauge_stability_required_s) << ",\n"
         << "  \"gauge_stable_duration_s\": "
         << json_scalar(d.gauge_stable_duration_s) << ",\n"
         << "  \"gauge_yaw_delta_from_previous_deg\": "
         << json_scalar(d.gauge_yaw_delta_from_previous_deg) << ",\n"
         << "  \"gauge_translation_delta_from_previous_m\": "
         << json_scalar(d.gauge_translation_delta_from_previous_m) << ",\n"
         << "  \"gauge_yaw_consistency_limit_deg\": "
         << json_scalar(d.gauge_yaw_consistency_limit_deg) << ",\n"
         << "  \"gauge_translation_consistency_limit_m\": "
         << json_scalar(d.gauge_translation_consistency_limit_m) << ",\n"
         << "  \"gauge_metric_scale\": "
         << json_scalar(d.gauge_metric_scale) << ",\n"
         << "  \"gauge_metric_scale_observed\": "
         << json_scalar(d.gauge_metric_scale_observed) << ",\n"
         << "  \"gauge_metric_scale_sigma\": "
         << json_scalar(d.gauge_metric_scale_sigma) << ",\n"
         << "  \"gauge_velocity_fit_rmse_mps\": "
         << json_scalar(d.gauge_velocity_fit_rmse_mps) << ",\n"
         << "  \"gauge_velocity_excitation_mps\": "
         << json_scalar(d.gauge_velocity_excitation_mps) << ",\n"
         << "  \"gauge_yaw_reset_deg\": "
         << json_scalar(d.gauge_yaw_reset_deg) << ",\n"
         << "  \"gauge_translation_G_m\": ";
  write_vec3(d.gauge_translation_G);
  output << ",\n"
         << "  \"gauge_window_quality_passed\": "
         << (d.gauge_window_quality_passed ? "true" : "false") << ",\n"
         << "  \"gauge_consistency_passed\": "
         << (d.gauge_consistency_passed ? "true" : "false") << ",\n"
         << "  \"sliding_overlap_state_count\": "
         << d.sliding_overlap_state_count << ",\n"
         << "  \"sliding_overlap_attitude_max_deg\": "
         << json_scalar(d.sliding_overlap_attitude_max_deg) << ",\n"
         << "  \"sliding_overlap_position_max_m\": "
         << json_scalar(d.sliding_overlap_position_max_m) << ",\n"
         << "  \"sliding_overlap_velocity_max_mps\": "
         << json_scalar(d.sliding_overlap_velocity_max_mps) << ",\n"
         << "  \"sliding_overlap_gyro_bias_max_rad_s\": "
         << json_scalar(d.sliding_overlap_gyro_bias_max_rad_s) << ",\n"
         << "  \"sliding_overlap_accel_bias_max_mps2\": "
         << json_scalar(d.sliding_overlap_accel_bias_max_mps2) << ",\n"
         << "  \"sliding_overlap_normalized_max_sigma\": "
         << json_scalar(d.sliding_overlap_normalized_max_sigma) << ",\n"
         << "  \"sliding_overlap_stable_duration_s\": "
         << json_scalar(d.sliding_overlap_stable_duration_s) << ",\n"
         << "  \"sliding_overlap_stability_required_s\": "
         << json_scalar(d.sliding_overlap_stability_required_s) << ",\n"
         << "  \"sliding_overlap_consistency_passed\": "
         << (d.sliding_overlap_consistency_passed ? "true" : "false")
         << ",\n"
         << "  \"sliding_bias_trend_sample_count\": "
         << d.sliding_bias_trend_sample_count << ",\n"
         << "  \"sliding_bias_trend_span_s\": "
         << json_scalar(d.sliding_bias_trend_span_s) << ",\n"
         << "  \"sliding_bg_trend_projected_change_rad_s\": ";
  write_vec3(d.sliding_bg_trend_projected_change_rad_s);
  output << ",\n  \"sliding_ba_trend_projected_change_mps2\": ";
  write_vec3(d.sliding_ba_trend_projected_change_mps2);
  output << ",\n"
         << "  \"sliding_bias_trend_normalized_max_sigma\": "
         << json_scalar(d.sliding_bias_trend_normalized_max_sigma) << ",\n"
         << "  \"sliding_bias_trend_passed\": "
         << (d.sliding_bias_trend_passed ? "true" : "false") << ",\n"
         << "  \"direct_sliding_state_release\": "
         << (d.direct_sliding_state_release ? "true" : "false") << ",\n"
         << "  \"initial_history_clone_count\": "
         << result.initial_clones.size() << ",\n"
         << "  \"initial_persistent_landmark_count\": "
         << result.initial_landmarks.size() << ",\n"
         << "  \"startup_consumed_feature_count\": "
         << result.startup_consumed_feature_ids.size() << ",\n"
         << "  \"initial_joint_covariance_dimension\": "
         << result.initial_joint_covariance.rows() << ",\n"
         << "  \"initial_history_covariance_recovered\": "
         << (d.initial_history_covariance_recovered ? "true" : "false")
         << ",\n"
         << "  \"handoff_covariance_inflation_applied\": "
         << (d.handoff_covariance_inflation_applied ? "true" : "false")
         << ",\n"
         << "  \"handoff_covariance_model\": \""
         << json_escape(d.handoff_covariance_model) << "\",\n"
         << "  \"handoff_covariance_inflation\": {"
         << "\"orientation\": "
         << json_scalar(d.handoff_covariance_inflation[0]) << ", "
         << "\"velocity\": "
         << json_scalar(d.handoff_covariance_inflation[1]) << ", "
         << "\"gyro_bias\": "
         << json_scalar(d.handoff_covariance_inflation[2]) << ", "
         << "\"accelerometer_bias\": "
         << json_scalar(d.handoff_covariance_inflation[3]) << "},\n"
         << "  \"handoff_raw_covariance_std\": ";
  write_vec15(d.handoff_raw_covariance_std);
  output << ",\n  \"handoff_applied_covariance_std\": ";
  write_vec15(d.handoff_applied_covariance_std);
  output << ",\n"
         << "  \"candidate_validation_duration_s\": "
         << json_scalar(d.candidate_validation_duration_s) << ",\n"
         << "  \"candidate_validation_frames\": "
         << d.candidate_validation_frames << ",\n"
         << "  \"candidate_fc_imu_rotation_residual_deg\": "
         << json_scalar(d.candidate_fc_imu_rotation_residual_deg) << ",\n"
         << "  \"candidate_fc_imu_yaw_residual_deg\": "
         << json_scalar(d.candidate_fc_imu_yaw_residual_deg) << ",\n"
         << "  \"candidate_relative_position_residual_m\": "
         << json_scalar(d.candidate_relative_position_residual_m) << ",\n"
         << "  \"candidate_relative_velocity_residual_mps\": "
         << json_scalar(d.candidate_relative_velocity_residual_mps) << ",\n"
         << "  \"candidate_nis_gate_3d\": "
         << json_scalar(d.candidate_nis_gate_3d) << ",\n"
         << "  \"candidate_last_position_nis\": "
         << json_scalar(d.candidate_last_position_nis) << ",\n"
         << "  \"candidate_last_velocity_nis\": "
         << json_scalar(d.candidate_last_velocity_nis) << ",\n"
         << "  \"candidate_max_position_nis\": "
         << json_scalar(d.candidate_max_position_nis) << ",\n"
         << "  \"candidate_max_velocity_nis\": "
         << json_scalar(d.candidate_max_velocity_nis) << ",\n"
         << "  \"candidate_visual_compensated_p95_px\": "
         << json_scalar(d.candidate_visual_compensated_p95_px) << ",\n"
         << "  \"candidate_visual_reprojection_p50_px\": "
         << json_scalar(d.candidate_visual_reprojection_p50_px) << ",\n"
         << "  \"candidate_visual_reprojection_p95_px\": "
         << json_scalar(d.candidate_visual_reprojection_p95_px) << ",\n"
         << "  \"candidate_visual_reprojection_trend_pxps\": "
         << json_scalar(d.candidate_visual_reprojection_trend_pxps) << ",\n"
         << "  \"candidate_covariance_normalized_residual\": "
         << json_scalar(d.candidate_covariance_normalized_residual) << ",\n"
         << "  \"candidate_release_propagation_duration_s\": "
         << json_scalar(d.candidate_release_propagation_duration_s) << ",\n"
         << "  \"candidate_pre_correction_position_residual_m\": "
         << json_scalar(d.candidate_pre_correction_position_residual_m) << ",\n"
         << "  \"candidate_pre_correction_velocity_residual_mps\": "
         << json_scalar(d.candidate_pre_correction_velocity_residual_mps) << ",\n"
          << "  \"candidate_pre_correction_visual_reprojection_p95_px\": "
          << json_scalar(d.candidate_pre_correction_visual_reprojection_p95_px) << ",\n"
          << "  \"candidate_post_correction_attitude_residual_deg\": "
          << json_scalar(d.candidate_post_correction_attitude_residual_deg)
          << ",\n"
          << "  \"candidate_post_correction_position_residual_m\": "
          << json_scalar(d.candidate_post_correction_position_residual_m)
          << ",\n"
          << "  \"candidate_post_correction_velocity_residual_mps\": "
          << json_scalar(d.candidate_post_correction_velocity_residual_mps)
          << ",\n"
          << "  \"candidate_closed_loop_attitude_correction_deg\": "
         << json_scalar(d.candidate_closed_loop_attitude_correction_deg) << ",\n"
         << "  \"candidate_closed_loop_position_correction_m\": "
         << json_scalar(d.candidate_closed_loop_position_correction_m) << ",\n"
         << "  \"candidate_closed_loop_velocity_correction_mps\": "
         << json_scalar(d.candidate_closed_loop_velocity_correction_mps) << ",\n"
         << "  \"candidate_closed_loop_gyro_bias_correction_rad_s\": "
         << json_scalar(d.candidate_closed_loop_gyro_bias_correction_rad_s) << ",\n"
         << "  \"candidate_closed_loop_accel_bias_correction_mps2\": "
         << json_scalar(d.candidate_closed_loop_accel_bias_correction_mps2) << ",\n"
         << "  \"candidate_closed_loop_update_count\": "
         << d.candidate_closed_loop_update_count << ",\n"
         << "  \"candidate_closed_loop_measurement_update_count\": "
         << d.candidate_closed_loop_measurement_update_count << ",\n"
         << "  \"candidate_closed_loop_rejected_measurement_count\": "
         << d.candidate_closed_loop_rejected_measurement_count << ",\n"
         << "  \"candidate_closed_loop_visual_update_count\": "
         << d.candidate_closed_loop_visual_update_count << ",\n"
         << "  \"candidate_closed_loop_correction_applied\": "
         << (d.candidate_closed_loop_correction_applied ? "true" : "false") << ",\n"
         << "  \"candidate_covariance_joseph_update_applied\": "
         << (d.candidate_covariance_joseph_update_applied ? "true" : "false") << ",\n"
         << "  \"candidate_covariance_error_reset_applied\": "
         << (d.candidate_covariance_error_reset_applied ? "true" : "false") << ",\n"
         << "  \"candidate_fc_attitude_evaluation_only\": "
         << (d.candidate_fc_attitude_evaluation_only ? "true" : "false") << ",\n"
         << "  \"fc_attitude_gauge_factor_enabled\": "
         << (d.fc_attitude_gauge_factor_enabled ? "true" : "false")
         << ",\n"
         << "  \"fc_attitude_gauge_factor_count\": "
         << d.fc_attitude_gauge_factor_count << ",\n"
         << "  \"fc_position_velocity_factor_count\": "
         << d.fc_position_velocity_factor_count << ",\n"
         << "  \"fc_position_velocity_time_weight_sum\": "
         << json_scalar(d.fc_position_velocity_time_weight_sum) << ",\n"
         << "  \"fc_position_velocity_weight_model\": \""
         << json_escape(d.fc_position_velocity_weight_model) << "\",\n"
         << "  \"fc_attitude_gauge_anchor_timestamp_s\": "
         << json_scalar(d.fc_attitude_gauge_anchor_timestamp_s) << ",\n"
         << "  \"fc_attitude_gauge_anchor_rate_rad_s\": "
         << json_scalar(d.fc_attitude_gauge_anchor_rate_rad_s) << ",\n"
         << "  \"fc_attitude_gauge_anchor_rate_residual_rad_s\": "
         << json_scalar(d.fc_attitude_gauge_anchor_rate_residual_rad_s)
         << ",\n"
         << "  \"fc_attitude_gauge_sigma_deg\": "
         << json_scalar(d.fc_attitude_gauge_sigma_deg) << ",\n"
         << "  \"fc_terminal_attitude_residual_deg\": "
         << json_scalar(d.fc_terminal_attitude_residual_deg) << ",\n"
         << "  \"fc_terminal_position_residual_m\": "
         << json_scalar(d.fc_terminal_position_residual_m) << ",\n"
         << "  \"fc_terminal_velocity_residual_mps\": "
         << json_scalar(d.fc_terminal_velocity_residual_mps) << ",\n"
         << "  \"fc_terminal_max_normalized_residual\": "
         << json_scalar(d.fc_terminal_max_normalized_residual) << ",\n"
         << "  \"candidate_feedback_tier\": \""
         << json_escape(d.candidate_feedback_tier) << "\",\n"
         << "  \"candidate_feedback_state_mask\": \""
         << json_escape(d.candidate_feedback_state_mask) << "\",\n"
         << "  \"candidate_attitude_feedback_allowed\": "
         << (d.candidate_attitude_feedback_allowed ? "true" : "false")
         << ",\n"
         << "  \"candidate_gyro_bias_feedback_allowed\": "
         << (d.candidate_gyro_bias_feedback_allowed ? "true" : "false")
         << ",\n"
         << "  \"candidate_accel_bias_feedback_allowed\": "
         << (d.candidate_accel_bias_feedback_allowed ? "true" : "false")
         << ",\n"
         << "  \"candidate_persistent_error_state_order\": "
            "[\"delta_theta_rad\", \"delta_p_m\", \"delta_v_mps\", "
            "\"delta_bg_rad_s\", \"delta_ba_mps2\"],\n"
         << "  \"candidate_persistent_error_state\": ";
  write_json_error_state(output, d.candidate_persistent_error);
  output << ",\n  \"candidate_persistent_std\": ";
  write_json_error_state(output, d.candidate_persistent_std);
  output << ",\n  \"candidate_group_order\": "
            "[\"q\", \"p\", \"v\", \"bg\", \"ba\"],\n"
         << "  \"candidate_group_supported_update_counts\": ";
  write_json_int_array(output, d.candidate_group_supported_update_counts);
  output << ",\n  \"candidate_required_group_support_update_counts\": ";
  write_json_int_array(
      output, d.candidate_required_group_support_update_counts);
  output << ",\n  \"candidate_required_group_history_lengths\": ";
  write_json_int_array(output, d.candidate_required_group_history_lengths);
  output << ",\n  \"candidate_window_effective_measurement_counts\": ";
  write_json_int_array(output, d.candidate_window_effective_measurement_counts);
  output << ",\n  \"candidate_gate_depth_source\": \""
         << json_escape(d.candidate_gate_depth_source) << "\"";
  output << ",\n  \"candidate_group_feedback_counts\": ";
  write_json_int_array(output, d.candidate_group_feedback_counts);
  output << ",\n  \"candidate_group_post_feedback_stable_update_counts\": ";
  write_json_int_array(
      output, d.candidate_group_post_feedback_stable_update_counts);
  output << ",\n  \"candidate_group_gate_passed\": ";
  write_json_bool_array(output, d.candidate_group_gate_passed);
  output << ",\n"
         << "  \"candidate_consumed_fc_event_count\": "
         << d.candidate_consumed_fc_event_count << ",\n"
         << "  \"candidate_accepted_fc_event_count\": "
         << d.candidate_accepted_fc_event_count << ",\n"
         << "  \"candidate_rejected_fc_event_count\": "
         << d.candidate_rejected_fc_event_count << ",\n"
         << "  \"candidate_propagated_imu_step_count\": "
         << d.candidate_propagated_imu_step_count << ",\n"
         << "  \"candidate_duplicate_fc_event_count\": "
         << d.candidate_duplicate_fc_event_count << ",\n"
         << "  \"candidate_duplicate_imu_step_count\": "
         << d.candidate_duplicate_imu_step_count << ",\n"
         << "  \"candidate_covariance_health\": \""
         << json_escape(d.candidate_covariance_health) << "\",\n"
         << "  \"alignment_window_closed\": "
         << (d.alignment_window_closed ? "true" : "false") << ",\n"
         << "  \"alignment_window_close_time_s\": "
         << json_scalar(d.alignment_window_close_time) << ",\n"
         << "  \"future_data_used\": "
         << (d.future_data_used ? "true" : "false") << ",\n"
         << "  \"post_alignment_used\": false,\n"
         << "  \"gps_xy_or_course_used_by_alignment\": false,\n"
         << "  \"camera_imu_T_C_I\": \""
         << json_escape(d.provenance.camera_imu_calibration) << "\",\n"
         << "  \"locked_camera_to_imu_time_offset_s\": "
         << result.camera_to_imu_time_offset_s << ",\n"
         << "  \"manual_7_degree_correction\": false,\n"
         << "  \"manual_4_089_degree_correction\": false,\n"
         << "  \"time_varying_flex_correction\": false,\n"
         << "  \"old_initializer_fallback_used\": "
         << (d.old_initializer_fallback_used ? "true" : "false") << ",\n"
         << "  \"counts\": {\"fc\": " << d.fc_samples << ", \"imu\": "
         << d.imu_samples << ", \"visual_frames\": " << d.stereo_frames
         << ", \"feature_tracks\": " << d.feature_tracks
         << ", \"triangulated_landmarks\": "
         << d.visual_statistics.triangulated_landmark_count
         << ", \"rejected_landmarks\": "
         << d.visual_statistics.rejected_landmark_count
         << ", \"multi_frame_tracks\": "
         << d.visual_statistics.multi_frame_track_count
         << ", \"visual_factors\": "
         << d.visual_statistics.visual_factor_count << "},\n"
         << "  \"max_gaps_s\": {\"fc\": " << d.max_fc_gap_s
         << ", \"imu\": " << d.max_imu_gap_s << ", \"stereo\": "
         << d.max_stereo_gap_s << "},\n"
         << "  \"time_offset_status\": \""
         << estimate_source_status_name(d.time_offset_status) << "\",\n"
         << "  \"startup_misalignment_status\": \""
         << estimate_source_status_name(d.startup_misalignment_status)
         << "\",\n"
         << "  \"fc_to_board_time_offset_s\": "
         << result.fc_to_board_time_offset_s << ",\n"
         << "  \"fc_attitude_to_board_time_offset_s\": "
         << result.fc_attitude_to_board_time_offset_s << ",\n"
         << "  \"fc_navigation_to_board_time_offset_s\": "
         << result.fc_navigation_to_board_time_offset_s << ",\n"
         << "  \"fc_to_board_time_offset_sigma_s\": "
         << d.time_offset_sigma_s << ",\n"
         << "  \"R_FtoI_nominal\": ";
  write_matrix3(result.R_FtoI_nominal);
  output << ",\n  \"R_mount_residual\": ";
  write_matrix3(result.R_mount_residual);
  output << ",\n  \"mount_residual_deg\": " << d.mount_residual_deg
         << ",\n  \"rate_residual_rms_rad_s\": "
         << d.rate_residual_rms_rad_s
         << ",\n  \"visual_imu_rotation_residual_deg\": "
         << json_scalar(d.visual_imu_rotation_residual_deg) << ",\n"
         << "  \"state\": {\"q_Gnav_to_I_xyzw\": ";
  write_vec4(result.q_GtoI);
  output << ", \"p_I_in_Gnav_m\": ";
  write_vec3(result.p_IinG);
  output << ", \"v_I_in_Gnav_mps\": ";
  write_vec3(result.v_IinG);
  output << ", \"gyro_bias_rad_s\": ";
  write_vec3(result.bg);
  output << ", \"accel_bias_mps2\": ";
  write_vec3(result.ba);
  output << "},\n  \"covariance_order\": "
            "[\"delta_theta\", \"delta_p\", \"delta_v\", \"delta_bg\", \"delta_ba\"],\n"
         << "  \"covariance_15x15\": [\n";
  for (int row = 0; row < 15; ++row) {
    output << "    [";
    for (int col = 0; col < 15; ++col) {
      if (col)
        output << ", ";
      output << result.covariance(row, col);
    }
    output << "]" << (row == 14 ? "\n" : ",\n");
  }
  output << "  ],\n  \"startup_misalignment_covariance_3x3\": [";
  for (int row = 0; row < 3; ++row) {
    if (row)
      output << ",";
    output << "[";
    for (int col = 0; col < 3; ++col) {
      if (col)
        output << ",";
      output << json_scalar(result.mount_covariance(row, col));
    }
    output << "]";
  }
  output << "],\n  \"factor_contributions\": [";
  for (size_t i = 0; i < d.factor_contributions.size(); ++i) {
    const auto &factor = d.factor_contributions[i];
    if (i)
      output << ", ";
    output << "{\"family\":\"" << json_escape(factor.family)
           << "\",\"residual_blocks\":" << factor.residual_blocks
           << ",\"residual_dimension\":" << factor.residual_dimension
           << ",\"residual_rms\":" << json_scalar(factor.residual_rms)
           << ",\"residual_p95\":" << json_scalar(factor.residual_p95)
           << ",\"jacobian_frobenius\":"
           << json_scalar(factor.jacobian_frobenius) << "}";
  }
  output << "],\n  \"monocular_visual_statistics\": {"
         << "\"tracked_feature_count\":"
         << d.visual_statistics.tracked_feature_count
         << ",\"multi_frame_track_count\":"
         << d.visual_statistics.multi_frame_track_count
         << ",\"triangulated_landmark_count\":"
         << d.visual_statistics.triangulated_landmark_count
         << ",\"rejected_landmark_count\":"
         << d.visual_statistics.rejected_landmark_count
         << ",\"visual_factor_count\":"
         << d.visual_statistics.visual_factor_count
         << ",\"reprojection_rmse_px\":"
         << json_scalar(d.visual_statistics.reprojection_rmse_px)
         << ",\"reprojection_p95_px\":"
         << json_scalar(d.visual_statistics.reprojection_p95_px)
         << ",\"schur_complement_information_trace\":"
         << json_scalar(
                d.visual_statistics.schur_complement_information_trace)
         << ",\"information_contribution_by_state\":{";
  size_t visual_info_index = 0;
  for (const auto &entry :
       d.visual_statistics.information_contribution_by_state) {
    if (visual_info_index++)
      output << ",";
    output << "\"" << json_escape(entry.first) << "\":"
           << json_scalar(entry.second);
  }
  output << "}},\n  \"state_observability\": [";
  for (size_t i = 0; i < d.state_observability.size(); ++i) {
    const auto &state = d.state_observability[i];
    if (i)
      output << ", ";
    output << "{\"state\":\"" << json_escape(state.state)
           << "\",\"observable\":" << (state.observable ? "true" : "false")
           << ",\"prior_only\":" << (state.prior_only ? "true" : "false")
           << ",\"estimate_status\":\""
           << estimate_source_status_name(state.estimate_status) << "\""
           << ",\"reason\":\"" << json_escape(state.reason)
           << "\",\"covariance_std\":["
           << json_scalar(state.covariance_std.x()) << ","
           << json_scalar(state.covariance_std.y()) << ","
           << json_scalar(state.covariance_std.z()) << "]}";
  }
  output << "],\n  \"estimated_states\": ";
  write_string_list(d.estimated_state_list);
  output << ",\n  \"weakly_observable_states\": ";
  write_string_list(d.weak_state_list);
  output << ",\n  \"fixed_to_prior_states\": ";
  write_string_list(d.fixed_state_list);
  output << ",\n  \"unobservable_states\": ";
  write_string_list(d.unobservable_state_list);
  output << ",\n  \"navigation_failed_gates\": ";
  write_string_list(d.navigation_failed_gates);
  output << ",\n  \"full_alignment_failed_gates\": ";
  write_string_list(d.full_alignment_failed_gates);
  output << ",\n  \"state_transitions\": [";
  for (size_t i = 0; i < d.state_transitions.size(); ++i) {
    const auto &transition = d.state_transitions[i];
    if (i)
      output << ", ";
    output << "{\"from\":\"" << alignment_phase_name(transition.from)
           << "\",\"to\":\"" << alignment_phase_name(transition.to)
           << "\",\"stream_time_s\":" << json_scalar(transition.stream_time)
           << ",\"reason\":\"" << json_escape(transition.reason) << "\"}";
  }
  output << "],\n  \"rejected_intervals\": [";
  for (size_t i = 0; i < d.rejected_intervals.size(); ++i) {
    const auto &interval = d.rejected_intervals[i];
    if (i)
      output << ", ";
    output << "{\"start_s\":" << json_scalar(interval.start_time)
           << ",\"end_s\":" << json_scalar(interval.end_time)
           << ",\"reason\":\"" << json_escape(interval.reason)
           << "\",\"peak_angular_rate_rad_s\":"
           << json_scalar(interval.peak_angular_rate_rad_s) << "}";
  }
  output << "],\n  \"sensor_provenance\": {\"fc_stream\":\""
         << json_escape(d.provenance.fc_stream) << "\",\"imu_stream\":\""
         << json_escape(d.provenance.imu_stream)
         << "\",\"visual_stream\":\""
         << json_escape(d.provenance.stereo_stream)
         << "\",\"camera_imu_calibration\":\""
         << json_escape(d.provenance.camera_imu_calibration)
         << "\",\"fc_axis_mapping\":\""
         << json_escape(d.provenance.fc_axis_mapping)
         << "\",\"gps_used\":" << (d.provenance.gps_used ? "true" : "false")
         << ",\"post_alignment_used\":"
         << (d.provenance.post_alignment_used ? "true" : "false")
         << ",\"manual_mounting_compensation_used\":"
         << (d.provenance.manual_mounting_compensation_used ? "true" : "false")
         << "}\n}\n";
}

void write_online_alignment_failure_metadata(
    const std::string &path, const OnlineAlignmentDiagnostics &diagnostics,
    const std::string &rejection, const Args &args) {
  ensure_parent_directory(path);
  std::ofstream output(path);
  if (!output.is_open())
    throw std::runtime_error("cannot open online alignment failure metadata: " + path);
  output << std::setprecision(15)
         << "{\n"
         << "  \"schema\": \"openvins_online_multisensor_alignment_v14\",\n"
         << "  \"mode\": \"online_multisensor_alignment\",\n"
         << "  \"status\": \"STREAM_ENDED_NOT_INITIALIZED\",\n"
         << "  \"readiness_level\": \""
         << alignment_readiness_name(diagnostics.readiness) << "\",\n"
         << "  \"release_policy\": \""
         << alignment_release_policy_name(diagnostics.release_policy)
         << "\",\n"
         << "  \"rejection\": \"" << json_escape(rejection) << "\",\n"
         << "  \"source_fc_stream\": \""
         << json_escape(args.init_from_fc_path) << "\",\n"
         << "  \"window_start_s\": " << diagnostics.window_start << ",\n"
         << "  \"t_init_s\": " << diagnostics.init_time << ",\n"
         << "  \"solve_time_s\": " << diagnostics.solve_time << ",\n"
         << "  \"future_data_used\": "
         << (diagnostics.future_data_used ? "true" : "false") << ",\n"
         << "  \"phase\": \"FAILED_WAIT_RETRY\",\n"
         << "  \"initialization_duration_s\": "
         << json_scalar(diagnostics.initialization_duration_s) << ",\n"
         << "  \"nonlinear_solve_attempt_count\": "
         << diagnostics.nonlinear_solve_attempt_count << ",\n"
         << "  \"shared_window_bias_model\": "
         << (diagnostics.shared_window_bias_model ? "true" : "false")
         << ",\n"
         << "  \"staged_solver_enabled\": "
         << (diagnostics.staged_solver_enabled ? "true" : "false")
         << ",\n"
         << "  \"stage1_solution_usable\": "
         << (diagnostics.stage1_solution_usable ? "true" : "false")
         << ",\n"
         << "  \"stage1_initial_cost\": "
         << json_scalar(diagnostics.stage1_initial_cost) << ",\n"
         << "  \"stage1_final_cost\": "
         << json_scalar(diagnostics.stage1_final_cost) << ",\n"
         << "  \"stage1_solver_iterations\": "
         << diagnostics.stage1_solver_iterations << ",\n"
         << "  \"stage1_solve_wall_time_s\": "
         << json_scalar(diagnostics.stage1_solve_wall_time_s) << ",\n"
         << "  \"final_stage_solve_wall_time_s\": "
         << json_scalar(diagnostics.final_stage_solve_wall_time_s) << ",\n"
         << "  \"successful_release_count\": "
         << diagnostics.successful_release_count << ",\n"
         << "  \"post_release_try_count\": "
         << diagnostics.post_release_try_count << ",\n"
         << "  \"solve_eligibility_skip_count\": "
         << diagnostics.solve_eligibility_skip_count << ",\n"
         << "  \"duplicate_window_skip_count\": "
         << diagnostics.duplicate_window_skip_count << ",\n"
         << "  \"candidate_created_count\": "
         << diagnostics.candidate_created_count << ",\n"
         << "  \"candidate_rejected_count\": "
         << diagnostics.candidate_rejected_count << ",\n"
         << "  \"candidate_feedback_tier\": \""
         << json_escape(diagnostics.candidate_feedback_tier) << "\",\n"
         << "  \"candidate_feedback_state_mask\": \""
         << json_escape(diagnostics.candidate_feedback_state_mask) << "\",\n"
         << "  \"candidate_persistent_error_state_order\": "
            "[\"delta_theta_rad\", \"delta_p_m\", \"delta_v_mps\", "
            "\"delta_bg_rad_s\", \"delta_ba_mps2\"],\n"
         << "  \"candidate_persistent_error_state\": ";
  write_json_error_state(output, diagnostics.candidate_persistent_error);
  output << ",\n  \"candidate_persistent_std\": ";
  write_json_error_state(output, diagnostics.candidate_persistent_std);
  output << ",\n  \"candidate_group_order\": "
            "[\"q\", \"p\", \"v\", \"bg\", \"ba\"],\n"
         << "  \"candidate_group_supported_update_counts\": ";
  write_json_int_array(
      output, diagnostics.candidate_group_supported_update_counts);
  output << ",\n  \"candidate_required_group_support_update_counts\": ";
  write_json_int_array(
      output, diagnostics.candidate_required_group_support_update_counts);
  output << ",\n  \"candidate_required_group_history_lengths\": ";
  write_json_int_array(
      output, diagnostics.candidate_required_group_history_lengths);
  output << ",\n  \"candidate_window_effective_measurement_counts\": ";
  write_json_int_array(
      output, diagnostics.candidate_window_effective_measurement_counts);
  output << ",\n  \"candidate_gate_depth_source\": \""
         << json_escape(diagnostics.candidate_gate_depth_source) << "\"";
  output << ",\n  \"candidate_group_feedback_counts\": ";
  write_json_int_array(output, diagnostics.candidate_group_feedback_counts);
  output << ",\n  \"candidate_group_post_feedback_stable_update_counts\": ";
  write_json_int_array(
      output, diagnostics.candidate_group_post_feedback_stable_update_counts);
  output << ",\n  \"candidate_group_gate_passed\": ";
  write_json_bool_array(output, diagnostics.candidate_group_gate_passed);
  output << ",\n"
         << "  \"candidate_closed_loop_measurement_update_count\": "
         << diagnostics.candidate_closed_loop_measurement_update_count
         << ",\n"
         << "  \"candidate_closed_loop_rejected_measurement_count\": "
         << diagnostics.candidate_closed_loop_rejected_measurement_count
         << ",\n"
         << "  \"candidate_consumed_fc_event_count\": "
         << diagnostics.candidate_consumed_fc_event_count << ",\n"
         << "  \"candidate_accepted_fc_event_count\": "
         << diagnostics.candidate_accepted_fc_event_count << ",\n"
         << "  \"candidate_rejected_fc_event_count\": "
         << diagnostics.candidate_rejected_fc_event_count << ",\n"
         << "  \"candidate_propagated_imu_step_count\": "
         << diagnostics.candidate_propagated_imu_step_count << ",\n"
         << "  \"candidate_duplicate_fc_event_count\": "
         << diagnostics.candidate_duplicate_fc_event_count << ",\n"
         << "  \"candidate_duplicate_imu_step_count\": "
         << diagnostics.candidate_duplicate_imu_step_count << ",\n"
         << "  \"candidate_covariance_health\": \""
         << json_escape(diagnostics.candidate_covariance_health) << "\",\n"
         << "  \"alignment_window_closed\": "
         << (diagnostics.alignment_window_closed ? "true" : "false")
         << ",\n"
         << "  \"alignment_window_close_time_s\": "
         << json_scalar(diagnostics.alignment_window_close_time) << ",\n"
         << "  \"failed_gates\": [";
  for (size_t i = 0; i < diagnostics.failed_gates.size(); ++i) {
    if (i)
      output << ", ";
    output << "\"" << json_escape(diagnostics.failed_gates[i]) << "\"";
  }
  output << "],\n"
         << "  \"counts\": {\"fc\": " << diagnostics.fc_samples
         << ", \"imu\": " << diagnostics.imu_samples
         << ", \"visual_frames\": " << diagnostics.stereo_frames
         << ", \"feature_tracks\": " << diagnostics.feature_tracks
         << ", \"triangulated_landmarks\": " << diagnostics.stereo_depths << "},\n"
         << "  \"monocular_visual_statistics\": {"
         << "\"tracked_feature_count\":"
         << diagnostics.visual_statistics.tracked_feature_count
         << ",\"multi_frame_track_count\":"
         << diagnostics.visual_statistics.multi_frame_track_count
         << ",\"triangulated_landmark_count\":"
         << diagnostics.visual_statistics.triangulated_landmark_count
         << ",\"rejected_landmark_count\":"
         << diagnostics.visual_statistics.rejected_landmark_count
         << ",\"visual_factor_count\":"
         << diagnostics.visual_statistics.visual_factor_count
         << ",\"reprojection_rmse_px\":"
         << json_scalar(diagnostics.visual_statistics.reprojection_rmse_px)
         << ",\"reprojection_p95_px\":"
         << json_scalar(diagnostics.visual_statistics.reprojection_p95_px)
         << ",\"schur_complement_information_trace\":"
         << json_scalar(diagnostics.visual_statistics
                            .schur_complement_information_trace)
         << "},\n"
         << "  \"navigation_failed_gates\": [";
  for (size_t i = 0; i < diagnostics.navigation_failed_gates.size(); ++i) {
    if (i)
      output << ", ";
    output << "\"" << json_escape(diagnostics.navigation_failed_gates[i])
           << "\"";
  }
  output << "],\n  \"full_alignment_failed_gates\": [";
  for (size_t i = 0; i < diagnostics.full_alignment_failed_gates.size(); ++i) {
    if (i)
      output << ", ";
    output << "\""
           << json_escape(diagnostics.full_alignment_failed_gates[i])
           << "\"";
  }
  output << "],\n"
         << "  \"state_transitions\": [";
  for (size_t i = 0; i < diagnostics.state_transitions.size(); ++i) {
    const auto &transition = diagnostics.state_transitions[i];
    if (i)
      output << ", ";
    output << "{\"from\":\"" << alignment_phase_name(transition.from)
           << "\",\"to\":\"" << alignment_phase_name(transition.to)
           << "\",\"stream_time_s\":" << json_scalar(transition.stream_time)
           << ",\"reason\":\"" << json_escape(transition.reason) << "\"}";
  }
  output << "]\n}\n";
}

void write_online_alignment_attempt_receipts(
    const std::string &path,
    const std::deque<AlignmentAttemptReceipt> &receipts,
    const AlignmentCandidate *candidate) {
  ensure_parent_directory(path);
  std::ofstream output(path, std::ofstream::out | std::ofstream::trunc);
  if (!output.is_open())
    throw std::runtime_error("cannot open online alignment attempt receipts: " +
                             path);
  output << std::setprecision(15)
     << "{\n  \"schema\": \"openvins_p4_sliding_window_attempts_v4\",\n"
         << "  \"attempts\": [\n";
  for (size_t index = 0; index < receipts.size(); ++index) {
    const auto &receipt = receipts[index];
    output << "    {\"window_id\": " << receipt.window_id
           << ", \"optimizer_invocation_index\": "
           << receipt.optimizer_invocation_index
           << ", \"optimizer_invocation_count\": "
           << receipt.optimizer_invocation_count
           << ", \"attempt_timestamp\": "
           << json_scalar(receipt.attempt_timestamp)
           << ", \"trigger\": \"" << json_escape(receipt.trigger)
           << "\", \"window_fingerprint\": \""
           << json_escape(receipt.window_fingerprint)
           << "\", \"window_duration_s\": "
           << json_scalar(receipt.window_duration_s)
           << ", \"window_begin_timestamp\": "
           << json_scalar(receipt.window_begin_timestamp)
           << ", \"window_end_timestamp\": "
           << json_scalar(receipt.window_end_timestamp)
           << ", \"raw_fc_count\": " << receipt.raw_fc_count
           << ", \"valid_fc_count\": " << receipt.valid_fc_count
           << ", \"imu_count\": " << receipt.imu_count
           << ", \"visual_frame_count\": "
           << receipt.visual_frame_count
           << ", \"selected_keyframe_count\": "
           << receipt.selected_keyframe_count
           << ", \"selected_tracks\": " << receipt.selected_tracks
           << ", \"selected_landmarks\": " << receipt.selected_landmarks
           << ", \"factor_count\": " << receipt.factor_count
           << ", \"solve_wall_time_s\": "
           << json_scalar(receipt.solve_wall_time_s)
           << ", \"shared_window_bias_model\": "
           << (receipt.shared_window_bias_model ? "true" : "false")
           << ", \"staged_solver_enabled\": "
           << (receipt.staged_solver_enabled ? "true" : "false")
           << ", \"stage1_solution_usable\": "
           << (receipt.stage1_solution_usable ? "true" : "false")
           << ", \"stage1_initial_cost\": "
           << json_scalar(receipt.stage1_initial_cost)
           << ", \"stage1_final_cost\": "
           << json_scalar(receipt.stage1_final_cost)
           << ", \"stage1_solver_iterations\": "
           << receipt.stage1_solver_iterations
           << ", \"stage1_solve_wall_time_s\": "
           << json_scalar(receipt.stage1_solve_wall_time_s)
           << ", \"final_stage_solve_wall_time_s\": "
           << json_scalar(receipt.final_stage_solve_wall_time_s)
           << ", \"initial_cost\": " << json_scalar(receipt.initial_cost)
           << ", \"final_cost\": " << json_scalar(receipt.final_cost)
           << ", \"q_GtoI_xyzw\": ";
    write_json_eigen_vector(output, receipt.q_GtoI);
    output << ", \"p_IinG_m\": ";
    write_json_eigen_vector(output, receipt.p_IinG);
    output << ", \"v_IinG_mps\": ";
    write_json_eigen_vector(output, receipt.v_IinG);
    output << ", \"bg_rad_s\": ";
    write_json_eigen_vector(output, receipt.bg);
    output << ", \"ba_mps2\": ";
    write_json_eigen_vector(output, receipt.ba);
    output << ", \"bg_prior_center_rad_s\": ";
    write_json_eigen_vector(output, receipt.bg_prior_center);
    output << ", \"ba_prior_center_mps2\": ";
    write_json_eigen_vector(output, receipt.ba_prior_center);
    output << ", \"state_std\": ";
    write_json_eigen_vector(output, receipt.state_std);
    output << ", \"imu_residual_rms\": "
           << json_scalar(receipt.imu_residual_rms)
           << ", \"fc_residual_rms\": "
           << json_scalar(receipt.fc_residual_rms)
           << ", \"visual_reprojection_rmse_px\": "
           << json_scalar(receipt.visual_reprojection_rmse_px)
           << ", \"visual_reprojection_p95_px\": "
           << json_scalar(receipt.visual_reprojection_p95_px)
           << ", \"joint_normalized_cost\": "
           << json_scalar(receipt.joint_normalized_cost)
           << ", \"angular_excitation_rad_s\": "
           << json_scalar(receipt.angular_excitation_rad_s)
           << ", \"second_axis_ratio\": "
           << json_scalar(receipt.second_axis_ratio)
           << ", \"previous_attitude_delta_deg\": "
           << json_scalar(receipt.previous_attitude_delta_deg)
           << ", \"previous_position_delta_m\": "
           << json_scalar(receipt.previous_position_delta_m)
           << ", \"previous_velocity_delta_mps\": "
           << json_scalar(receipt.previous_velocity_delta_mps)
           << ", \"previous_gyro_bias_delta_rad_s\": "
           << json_scalar(receipt.previous_gyro_bias_delta_rad_s)
           << ", \"previous_accel_bias_delta_mps2\": "
           << json_scalar(receipt.previous_accel_bias_delta_mps2)
           << ", \"overlap_state_count\": " << receipt.overlap_state_count
           << ", \"overlap_attitude_max_deg\": "
           << json_scalar(receipt.overlap_attitude_max_deg)
           << ", \"overlap_position_max_m\": "
           << json_scalar(receipt.overlap_position_max_m)
           << ", \"overlap_velocity_max_mps\": "
           << json_scalar(receipt.overlap_velocity_max_mps)
           << ", \"overlap_gyro_bias_max_rad_s\": "
           << json_scalar(receipt.overlap_gyro_bias_max_rad_s)
           << ", \"overlap_accel_bias_max_mps2\": "
           << json_scalar(receipt.overlap_accel_bias_max_mps2)
           << ", \"overlap_normalized_max_sigma\": "
           << json_scalar(receipt.overlap_normalized_max_sigma)
           << ", \"bias_trend_sample_count\": "
           << receipt.bias_trend_sample_count
           << ", \"bias_trend_span_s\": "
           << json_scalar(receipt.bias_trend_span_s)
           << ", \"bg_trend_projected_change_rad_s\": ";
    write_json_eigen_vector(output,
                            receipt.bg_trend_projected_change_rad_s);
    output << ", \"ba_trend_projected_change_mps2\": ";
    write_json_eigen_vector(output,
                            receipt.ba_trend_projected_change_mps2);
    output << ", \"bias_trend_normalized_max_sigma\": "
           << json_scalar(receipt.bias_trend_normalized_max_sigma)
           << ", \"bias_trend_passed\": "
           << (receipt.bias_trend_passed ? "true" : "false")
           << ", \"overlap_stable_duration_s\": "
           << json_scalar(receipt.overlap_stable_duration_s)
           << ", \"overlap_consistency_passed\": "
           << (receipt.overlap_consistency_passed ? "true" : "false")
           << ", \"warm_start_used\": "
           << (receipt.warm_start_used ? "true" : "false")
           << ", \"outcome\": \"" << json_escape(receipt.outcome)
           << "\", \"failed_gate\": \""
           << json_escape(receipt.failed_gate)
           << "\", \"next_eligible_condition\": \""
           << json_escape(receipt.next_eligible_condition)
           << "\", \"selected_frame_timestamps\": [";
    for (size_t frame = 0; frame < receipt.selected_frame_timestamps.size();
         ++frame) {
      if (frame > 0)
        output << ", ";
      output << json_scalar(receipt.selected_frame_timestamps[frame]);
    }
    output << "]}" << (index + 1 == receipts.size() ? "\n" : ",\n");
  }
  output << "  ],\n  \"final_candidate\": ";
  if (candidate == nullptr) {
    output << "null\n";
  } else {
    output << "{\"timestamp\": " << json_scalar(candidate->result.timestamp)
           << ", \"solve_window_start\": "
           << json_scalar(candidate->solve_window_start)
           << ", \"solve_window_end\": "
           << json_scalar(candidate->solve_window_end)
           << ", \"selected_frame_count\": "
           << candidate->selected_frame_timestamps.size()
           << ", \"selected_feature_count\": "
           << candidate->selected_feature_ids.size()
           << ", \"optimized_landmark_count\": "
           << candidate->optimized_landmarks_G.size()
           << ", \"solve_wall_time_s\": "
           << json_scalar(candidate->solve_wall_time_s) << "}\n";
  }
  output << "}\n";
}

void write_online_alignment_sliding_window_trace(
    const std::string &path,
    const std::deque<AlignmentAttemptReceipt> &receipts) {
  ensure_parent_directory(path);
  std::ofstream output(path, std::ofstream::out | std::ofstream::trunc);
  if (!output.is_open())
    throw std::runtime_error(
        "cannot open online alignment sliding-window trace: " + path);
  output << std::setprecision(17)
         << "window_id,optimizer_invocation_index,optimizer_invocation_count,"
            "shared_window_bias_model,staged_solver_enabled,"
            "stage1_solution_usable,stage1_initial_cost,stage1_final_cost,"
            "stage1_solver_iterations,stage1_solve_wall_time_s,"
            "final_stage_solve_wall_time_s,window_begin_timestamp_s,"
            "window_end_timestamp_s,window_duration_s,raw_fc_count,"
            "valid_fc_count,imu_count,visual_frame_count,"
            "selected_keyframe_count,selected_frame_timestamps_s,"
            "solve_wall_time_s,initial_cost,final_cost,q_GtoI_xyzw,"
            "p_IinG_m,v_IinG_mps,bg_rad_s,ba_mps2,"
            "bg_prior_center_rad_s,ba_prior_center_mps2,"
            "q_std_rad,p_std_m,"
            "v_std_mps,bg_std_rad_s,ba_std_mps2,imu_residual_rms,"
            "fc_residual_rms,visual_reprojection_rmse_px,"
            "visual_reprojection_p95_px,joint_normalized_cost,"
            "angular_excitation_rad_s,second_axis_ratio,"
            "previous_attitude_delta_deg,previous_position_delta_m,"
            "previous_velocity_delta_mps,previous_gyro_bias_delta_rad_s,"
            "previous_accel_bias_delta_mps2,overlap_state_count,"
            "overlap_attitude_max_deg,overlap_position_max_m,"
            "overlap_velocity_max_mps,overlap_gyro_bias_max_rad_s,"
            "overlap_accel_bias_max_mps2,overlap_normalized_max_sigma,"
            "overlap_stable_duration_s,bias_trend_sample_count,"
            "bias_trend_span_s,bg_trend_projected_change_rad_s,"
            "ba_trend_projected_change_mps2,"
            "bias_trend_normalized_max_sigma,bias_trend_passed,"
            "overlap_consistency_passed,warm_start_used,solve_status,"
            "failed_gate,window_fingerprint\n";
  for (const auto &receipt : receipts) {
    if (receipt.window_id <= 0)
      continue;
    std::ostringstream timestamps;
    timestamps << std::setprecision(17);
    for (size_t index = 0; index < receipt.selected_frame_timestamps.size();
         ++index) {
      if (index)
        timestamps << ';';
      timestamps << receipt.selected_frame_timestamps[index];
    }
    output << receipt.window_id << ',' << receipt.optimizer_invocation_index
           << ',' << receipt.optimizer_invocation_count << ','
           << (receipt.shared_window_bias_model ? 1 : 0) << ','
           << (receipt.staged_solver_enabled ? 1 : 0) << ','
           << (receipt.stage1_solution_usable ? 1 : 0) << ','
           << receipt.stage1_initial_cost << ',' << receipt.stage1_final_cost
           << ',' << receipt.stage1_solver_iterations << ','
           << receipt.stage1_solve_wall_time_s << ','
           << receipt.final_stage_solve_wall_time_s << ','
           << receipt.window_begin_timestamp << ','
           << receipt.window_end_timestamp << ',' << receipt.window_duration_s
           << ',' << receipt.raw_fc_count << ',' << receipt.valid_fc_count
           << ',' << receipt.imu_count << ',' << receipt.visual_frame_count
           << ',' << receipt.selected_keyframe_count << ",\""
           << timestamps.str() << "\"," << receipt.solve_wall_time_s << ','
           << receipt.initial_cost << ',' << receipt.final_cost << ",\""
           << csv_eigen_vector(receipt.q_GtoI) << "\",\""
           << csv_eigen_vector(receipt.p_IinG) << "\",\""
           << csv_eigen_vector(receipt.v_IinG) << "\",\""
           << csv_eigen_vector(receipt.bg) << "\",\""
           << csv_eigen_vector(receipt.ba) << "\",\""
           << csv_eigen_vector(receipt.bg_prior_center) << "\",\""
           << csv_eigen_vector(receipt.ba_prior_center) << "\",\""
           << csv_eigen_vector(receipt.state_std.segment<3>(0)) << "\",\""
           << csv_eigen_vector(receipt.state_std.segment<3>(3)) << "\",\""
           << csv_eigen_vector(receipt.state_std.segment<3>(6)) << "\",\""
           << csv_eigen_vector(receipt.state_std.segment<3>(9)) << "\",\""
           << csv_eigen_vector(receipt.state_std.segment<3>(12)) << "\","
           << receipt.imu_residual_rms << ',' << receipt.fc_residual_rms << ','
           << receipt.visual_reprojection_rmse_px << ','
           << receipt.visual_reprojection_p95_px << ','
           << receipt.joint_normalized_cost << ','
           << receipt.angular_excitation_rad_s << ','
           << receipt.second_axis_ratio << ','
           << receipt.previous_attitude_delta_deg << ','
           << receipt.previous_position_delta_m << ','
           << receipt.previous_velocity_delta_mps << ','
           << receipt.previous_gyro_bias_delta_rad_s << ','
           << receipt.previous_accel_bias_delta_mps2 << ','
           << receipt.overlap_state_count << ','
           << receipt.overlap_attitude_max_deg << ','
           << receipt.overlap_position_max_m << ','
           << receipt.overlap_velocity_max_mps << ','
           << receipt.overlap_gyro_bias_max_rad_s << ','
           << receipt.overlap_accel_bias_max_mps2 << ','
           << receipt.overlap_normalized_max_sigma << ','
           << receipt.overlap_stable_duration_s << ','
           << receipt.bias_trend_sample_count << ','
           << receipt.bias_trend_span_s << ",\""
           << csv_eigen_vector(receipt.bg_trend_projected_change_rad_s)
           << "\",\""
           << csv_eigen_vector(receipt.ba_trend_projected_change_mps2)
           << "\"," << receipt.bias_trend_normalized_max_sigma << ','
           << (receipt.bias_trend_passed ? 1 : 0) << ','
           << (receipt.overlap_consistency_passed ? 1 : 0) << ','
           << (receipt.warm_start_used ? 1 : 0) << ",\""
           << receipt.outcome << "\",\"" << receipt.failed_gate << "\",\""
           << receipt.window_fingerprint << "\"\n";
  }
}

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
     << "  \"schema_version\": \"nav-frame-v4\",\n"
     << "  \"frame_contract_version\": \"openvins-frame-v2\",\n"
     << "  \"frame_name\": \"G_nav\",\n"
     << "  \"origin_definition\": \"G_nav origin declared by the FC init series; GPS is reference-only\",\n"
     << "  \"axis_definition\": \"East, North, Up\",\n"
     << "  \"source_fc_file\": \"" << json_escape(args.init_from_fc_path) << "\",\n"
     << "  \"source_gps_file\": \"" << json_escape(args.gps_path) << "\",\n"
     << "  \"dataset_first_imu_timestamp\": " << nav.dataset_first_imu_timestamp << ",\n"
     << "  \"requested_start_offset_s\": " << nav.requested_start_offset_s << ",\n"
     << "  \"trim_boundary_timestamp\": " << nav.trim_boundary_timestamp << ",\n"
     << "  \"seed_camera_timestamp\": " << nav.init_camera_timestamp << ",\n"
     << "  \"init_camera_timestamp\": " << nav.init_camera_timestamp << ",\n"
     << "  \"first_emitted_timestamp\": " << nav.first_emitted_timestamp << ",\n"
     << "  \"selected_fc_timestamp\": " << nav.selected_fc_timestamp << ",\n"
     << "  \"camera_to_imu_time_offset_s\": "
     << nav.camera_to_imu_time_offset_s << ",\n"
     << "  \"fc_time_offset\": " << nav.fc_time_offset << ",\n"
     << "  \"initialization_mode\": \"" << json_escape(nav.initialization_mode) << "\",\n"
     << "  \"fc_init_requested_level\": \"" << json_escape(nav.fc_init_requested_level) << "\",\n"
     << "  \"fc_init_applied_level\": \"" << json_escape(nav.fc_init_applied_level) << "\",\n"
     << "  \"fc_init_selection_method\": \"" << json_escape(nav.fc_init_selection_method) << "\",\n"
     << "  \"fc_init_status\": \"" << json_escape(nav.fc_init_status) << "\",\n"
     << "  \"fc_init_fallback_applied\": " << (nav.fc_init_fallback_applied ? "true" : "false") << ",\n"
     << "  \"fc_position_frame\": \"" << json_escape(nav.fc_position_frame) << "\",\n"
     << "  \"fc_source_position_m\": [" << nav.fc_source_position.x() << ", "
     << nav.fc_source_position.y() << ", " << nav.fc_source_position.z() << "],\n"
     << "  \"gps_timestamp\": " << nav.gps_timestamp << ",\n"
     << "  \"gps_age_at_init_s\": " << json_scalar(nav.gps_age) << ",\n"
     << "  \"position_source\": \"" << json_escape(nav.position_source) << "\",\n"
     << "  \"attitude_source\": \"FC attitude reference interpolated at t_seed\",\n"
     << "  \"fc_board_fixed_correction\": null,\n"
     << "  \"fc_imu_extrinsic_version\": \"none_no_fixed_correction_v1\",\n"
     << "  \"gps_imu_lever_arm_version\": \"cli_p_I_GPS_v0\",\n"
     << "  \"gps_antenna_in_imu_m\": [" << nav.gps_antenna_in_imu.x() << ", "
     << nav.gps_antenna_in_imu.y() << ", " << nav.gps_antenna_in_imu.z() << "],\n"
     << "  \"no_post_position_alignment\": true,\n"
     << "  \"no_post_heading_alignment\": true,\n"
     << "  \"no_post_se3_alignment\": true,\n"
     << "  \"init_covariance_std\": {\"attitude_rad\": " << json_scalar(nav.init_att_sigma_rad)
     << ", \"velocity_mps\": " << json_scalar(nav.init_vel_sigma_mps)
     << ", \"position_m\": " << json_scalar(nav.init_pos_sigma_m)
     << ", \"gyro_bias_rad_s\": " << json_scalar(nav.init_bg_sigma_rad_s)
     << ", \"accel_bias_mps2\": " << json_scalar(nav.init_ba_sigma_mps2) << "},\n"
     << "  \"R_Gnav_W0\": [["
     << nav.R_Gnav_W0(0, 0) << ", " << nav.R_Gnav_W0(0, 1) << ", " << nav.R_Gnav_W0(0, 2) << "], ["
     << nav.R_Gnav_W0(1, 0) << ", " << nav.R_Gnav_W0(1, 1) << ", " << nav.R_Gnav_W0(1, 2) << "], ["
     << nav.R_Gnav_W0(2, 0) << ", " << nav.R_Gnav_W0(2, 1) << ", " << nav.R_Gnav_W0(2, 2) << "]],\n"
     << "  \"transform_equation\": \"p_Gnav = R_Gnav_W0 * p_W0 + p_W0inGnav\",\n"
     << "  \"p_W0inGnav\": [" << nav.p_Gnav_W0.x() << ", " << nav.p_Gnav_W0.y() << ", " << nav.p_Gnav_W0.z() << "],\n"
     << "  \"p_Gnav_W0\": [" << nav.p_Gnav_W0.x() << ", " << nav.p_Gnav_W0.y() << ", " << nav.p_Gnav_W0.z() << "],\n"
     << "  \"legacy_p_Gnav_W0_semantics\": \"same value as p_W0inGnav; retained for compatibility\",\n"
     << "  \"p_Gnav_GPS0\": [" << nav.p_Gnav_GPS0.x() << ", " << nav.p_Gnav_GPS0.y() << ", " << nav.p_Gnav_GPS0.z() << "],\n"
     << "  \"p_Gnav_I0\": [" << nav.p_Gnav_I0.x() << ", " << nav.p_Gnav_I0.y() << ", " << nav.p_Gnav_I0.z() << "],\n"
     << "  \"p_W0_I0\": [" << nav.p_W0_I0.x() << ", " << nav.p_W0_I0.y() << ", " << nav.p_W0_I0.z() << "],\n"
     << "  \"p_I_seed_in_W0_m\": [" << nav.p_W0_I0.x() << ", " << nav.p_W0_I0.y() << ", " << nav.p_W0_I0.z() << "],\n"
     << "  \"p_I_first_emitted_in_W0_m\": [" << nav.p_W0_Ifirst.x() << ", " << nav.p_W0_Ifirst.y() << ", " << nav.p_W0_Ifirst.z() << "],\n"
     << "  \"v_I_first_emitted_in_W0_mps\": [" << nav.v_W0_Ifirst.x() << ", " << nav.v_W0_Ifirst.y() << ", " << nav.v_W0_Ifirst.z() << "],\n"
     << "  \"q_ItoW0_first_emitted_xyzw\": [" << nav.q_ItoW0_first.x() << ", " << nav.q_ItoW0_first.y() << ", "
     << nav.q_ItoW0_first.z() << ", " << nav.q_ItoW0_first.w() << "],\n"
     << "  \"whether_future_data_used\": " << (nav.fc_init_future_data_used ? "true" : "false") << ",\n"
     << "  \"formal_initialization_delayed_to_window_end\": false,\n"
     << "  \"initialization_available_time\": " << json_scalar(nav.fc_init_available_time) << ",\n"
     << "  \"causality_status\": \""
     << (nav.fc_init_future_data_used ? "NON_CAUSAL_OFFLINE_DIAGNOSTIC" : "CAUSAL_AT_T_SEED") << "\",\n"
     << "  \"evaluation_only\": false,\n"
     << "  \"estimator_input\": false,\n"
     << "  \"gps_horizontal_semantics\": \"reference_only\",\n"
     << "  \"fc_global_position_estimator_input\": false,\n"
     << "  \"uses_future_data\": " << (nav.fc_init_future_data_used ? "true" : "false") << "\n"
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
               "  --gps-alt-guard-fallback-joseph-pz\n"
               "                            on coupled guard rejection, apply p_z-only Joseph update.\n"
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
                "  --initialization-mode MODE  fc_full_state, fc_attitude_only, vio_only, or online_multisensor_alignment.\n"
                "  --init-from-fc PATH   FC initialization time-series CSV (required by FC modes).\n"
                "  --init-from-fc-position-frame MODE\n"
                "                        Required with --init-from-fc: local_w0_seed or global_gnav.\n"
                "  --fc-init-level LEVEL I0 nearest, I1 exact-time, I2 robust SO(3), I3 quality-gated.\n"
                "  --fc-init-fallback MODE  fail_closed or i1 (I3 only).\n"
               "  --init-vel-sigma MPS  Initial velocity stddev (default 5.0; test 2/5/10)\n"
               "  --init-att-sigma-deg DEG  Initial attitude stddev (default 5.0)\n"
               "  --init-pos-sigma M    Initial position stddev (default 100.0)\n"
               "  --init-bg-sigma RPS   Initial gyro-bias stddev (default 0.05)\n"
               "  --init-ba-sigma MPS2  Initial accel-bias stddev (default 1.0)\n"
                "  --init-from-fc-max-dt S Maximum |FC time - camera init time| for FC init (default 0.25)\n"
                "  --init-from-fc-warn-only Warn instead of failing when --init-from-fc-max-dt is exceeded\n"
                "  --init-from-fc-max-bracket-gap S  Exact-time bracketing gap limit (default 0.35)\n"
                "  --init-window-s S     Robust SO(3) window, clamped to 3..8 s (default 5)\n"
                "  --init-window-min-samples N  Minimum robust-window rows (default 12)\n"
                "  --init-window-max-source-gap S  I3 maximum source gap (default 0.35)\n"
                "  --init-window-max-attitude-p95-deg DEG  I3 residual p95 gate (default 3)\n"
                "  --init-window-huber-delta-deg DEG Robust SO(3) Huber delta (default 1.5)\n"
               "  --gt PATH             ASL 17-col ground truth CSV\n"
               "  --output PATH         Legacy raw TUM trajectory path (default: traj_ros_free.txt)\n"
               "  --output-raw PATH     Canonical raw estimator trajectory path (default: OUTPUT sibling traj_raw.txt)\n"
               "  --output-nav PATH     Formal navigation-frame trajectory path (default: OUTPUT sibling traj_nav.txt)\n"
                "  --nav-frame-metadata-json PATH  Metadata for fixed T_Gnav_W0 (default: OUTPUT sibling nav_frame_metadata.json)\n"
                "  --canonical-init-state-json PATH  Canonical seed-state audit record (default: OUTPUT sibling canonical_init_state.json)\n"
                "  --online-alignment-metadata-json PATH  Causal online alignment release record (default: OUTPUT sibling online_alignment_metadata.json)\n"
                "  --online-alignment-release-policy POLICY  practical_navigation_start (default) or strict_full_alignment.\n"
                "  --online-alignment-disable-visual  Negative control: remove all monocular reprojection factors; full alignment is forbidden.\n"
                "  --online-alignment-navigation-allow-without-visual  Explicitly allow navigation-only release in the no-vision control.\n"
                "  --online-alignment-retain-bg-prior  Compatibility flag; production already retains the joint-graph bg estimate.\n"
                "  --online-alignment-retain-ba-prior  Compatibility flag; production already retains the joint-graph ba estimate.\n"
                "  --online-alignment-enable-candidate-bg-feedback  Diagnostic: allow FC p/v cross-covariance to change graph bg.\n"
                "  --online-alignment-enable-candidate-ba-feedback  Diagnostic: allow FC p/v cross-covariance to change graph ba.\n"
                "  --online-alignment-release-attitude-from-fc  Diagnostic q-only release ablation using synchronized FC attitude and accepted mount.\n"
                "  --online-alignment-diagnostic-never-anchor  Diagnostic isolation: solve P4 windows but never modify the provisional VIO.\n"
                "  --online-alignment-visual-perturbation-px PX  Add a deterministic pixel offset to selected feature tracks.\n"
                "  --online-alignment-visual-perturbation-fraction R  Fraction [0,1] of feature IDs perturbed.\n"
               "  --gps-antenna-in-imu X Y Z  GPS antenna position p_I_GPS in IMU frame meters (default 0 0 0)\n"
               "  --video PATH          Record dashboard to MP4\n"
               "  --video-cam PATH      Record camera-only (cam0/cam1 w/ optical-flow tracks) to MP4\n"
               "  --video-fps N         Video FPS (default 20)\n"
               "  --dashboard-alignment-json PATH  Persist display-only dashboard XY yaw alignment metadata\n"
               "  --align-seconds X     Seconds of data to collect before SE3 align (default 8)\n"
               "  --no-display          Render dashboard without creating a window (useful headless)\n"
               "  --no-dashboard        Disable dashboard data preparation, rendering, and display\n"
               "  --dash-title TITLE    Window title (default: 'OpenVINS ROS-free Dashboard')\n"
               "  --dash-every N        Refresh dashboard every N camera frames (default 1)\n"
               "  --camera-frame-stride N  Feed only 1 of every N camera frames to VIO before tracker (default 1)\n"
               "  --post-alignment-camera-frame-stride N  Override fixed stride only after P4 releases (diagnostic)\n"
               "  --post-alignment-visual-roi MODE  Post-P4 ROI: full|left|center|right|top|bottom|dynamic_turn\n"
               "  --dynamic-turn-roi-diag PATH  Write causal direction/detection/backend-quota diagnostics\n"
               "  --agl-scene-scale MODE  Visual-ground/AGL scene-scale mode: off|shadow|single_reset|guarded_repeat_reset\n"
               "  --agl-scene-scale-diag PATH  Write visual ground, ratio-window and atomic reset diagnostics\n"
               "  --adaptive-stride-shadow  Log continuous visual scheduling; keep fixed camera stride\n"
               "  --adaptive-stride         Apply continuous visual tracking/backend scheduling\n"
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
               "                            global_yaw_oc_centered_projection|global_4d_oc_projection\n"
               "                            global_yaw_oc_fej_prechi2|visual_4d_oc_fej_prechi2\n"
               "  --vio-yaw-update-scale S  Scale visual yaw correction for *_scale modes (1=orig, 0=off)\n"
               "  --vio-global-yaw-oc-alpha A  H-projection alpha for global_yaw_oc_projection (0=orig, 1=full)\n"
               "  --vio-yaw-control-start-after-init S  Delay requested visual yaw control until S seconds after init\n"
               "  --visual-bgz-update-scale S  Scale bg_z row of visual K_eff (1.0=normal, 0.0=freeze bg_z from visual)\n"
               "  --post-alignment-gyro-z-perturbation R  Diagnostic-only board gyro-z perturbation in rad/s after P4 release\n"
               "  --post-alignment-gyro-z-scale S  Diagnostic-only multiplicative board gyro-z scale after P4 release\n"
               "  --post-alignment-gyro-z-scale-min-abs R  Apply the diagnostic scale only when |gyro_z| >= R rad/s\n"
               "  --post-alignment-fc-yaw-aid  Diagnostic-only causal yaw anchor from FC attitude after P4 release\n"
               "  --post-alignment-fc-yaw-aid-period S  FC yaw-anchor period in seconds (default 2)\n"
               "  --post-alignment-fc-yaw-aid-sigma-deg D  FC yaw measurement sigma in degrees (default 2)\n"
               "  --post-alignment-fc-yaw-aid-log PATH  Write FC yaw-anchor decisions to CSV\n"
               "  --disable-slam-features  Diagnostic-only: set max_slam=0 while preserving KLT/MSCKF\n"
               "  --online-alignment-use-cli-init-covariance  Diagnostic-only: preserve the P4 nominal state/release time but use --init-*-sigma as a diagonal 15x15 covariance\n"
               "  --online-alignment-local-estimator-origin  Keep P4 absolute output but run VIO in a release-centered local W0\n"
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
                "  --msckf-yaw-freeze-window T0 T1 Zero only the MSCKF yaw correction in [T0,T1]\n"
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
    else if (s == "--canonical-init-state-json") a.canonical_init_state_path = next("--canonical-init-state-json");
    else if (s == "--online-alignment-metadata-json")
      a.online_alignment_metadata_path = next("--online-alignment-metadata-json");
    else if (s == "--online-alignment-disable-visual")
      a.online_alignment_disable_visual = true;
    else if (s == "--online-alignment-navigation-allow-without-visual")
      a.online_alignment_navigation_allow_without_visual = true;
    else if (s == "--online-alignment-retain-bg-prior")
      a.online_alignment_retain_bg_prior = true;
    else if (s == "--online-alignment-retain-ba-prior")
      a.online_alignment_retain_ba_prior = true;
    else if (s == "--online-alignment-enable-candidate-bg-feedback")
      a.online_alignment_retain_bg_prior = false;
    else if (s == "--online-alignment-enable-candidate-ba-feedback")
      a.online_alignment_retain_ba_prior = false;
    else if (s == "--online-alignment-release-attitude-from-fc")
      a.online_alignment_release_attitude_from_fc = true;
    else if (s == "--online-alignment-diagnostic-graph-q-fc-pv-zero-bias")
      a.online_alignment_diagnostic_graph_q_fc_pv_zero_bias = true;
    else if (s == "--online-alignment-diagnostic-never-anchor")
      a.online_alignment_diagnostic_never_anchor = true;
    else if (s == "--online-alignment-release-policy")
      a.online_alignment_release_policy =
          next("--online-alignment-release-policy");
    else if (s == "--online-alignment-visual-perturbation-px")
      a.online_alignment_visual_perturbation_px = std::atof(
          next("--online-alignment-visual-perturbation-px").c_str());
    else if (s == "--online-alignment-visual-perturbation-fraction")
      a.online_alignment_visual_perturbation_fraction = std::atof(
          next("--online-alignment-visual-perturbation-fraction").c_str());
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
    else if (s == "--gps-alt-guard-fallback-joseph-pz")
      a.gps_alt_guard_fallback_joseph_pz = true;
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
    else if (s == "--initialization-mode") a.initialization_mode = next("--initialization-mode");
    else if (s == "--init-from-fc" || s == "--init-state-csv") a.init_from_fc_path = next("--init-from-fc");
    else if (s == "--init-from-fc-position-frame")
      a.init_from_fc_position_frame = next("--init-from-fc-position-frame");
    else if (s == "--fc-init-level") a.fc_init_level = next("--fc-init-level");
    else if (s == "--fc-init-fallback") a.fc_init_fallback = next("--fc-init-fallback");
    else if (s == "--init-vel-sigma") a.init_vel_sigma = std::atof(next("--init-vel-sigma").c_str());
    else if (s == "--init-att-sigma-deg") a.init_att_sigma_deg = std::atof(next("--init-att-sigma-deg").c_str());
    else if (s == "--init-pos-sigma") a.init_pos_sigma = std::atof(next("--init-pos-sigma").c_str());
    else if (s == "--init-bg-sigma") a.init_bg_sigma = std::atof(next("--init-bg-sigma").c_str());
    else if (s == "--init-ba-sigma") a.init_ba_sigma = std::atof(next("--init-ba-sigma").c_str());
    else if (s == "--init-from-fc-max-dt") a.init_from_fc_max_dt = std::atof(next("--init-from-fc-max-dt").c_str());
    else if (s == "--init-from-fc-warn-only") a.init_from_fc_warn_only = true;
    else if (s == "--init-from-fc-max-bracket-gap") a.init_from_fc_max_bracket_gap = std::atof(next("--init-from-fc-max-bracket-gap").c_str());
    else if (s == "--init-window-s") a.init_window_s = std::atof(next("--init-window-s").c_str());
    else if (s == "--init-window-min-samples") a.init_window_min_samples = std::atoi(next("--init-window-min-samples").c_str());
    else if (s == "--init-window-max-source-gap") a.init_window_max_source_gap = std::atof(next("--init-window-max-source-gap").c_str());
    else if (s == "--init-window-max-attitude-p95-deg") a.init_window_max_attitude_p95_deg = std::atof(next("--init-window-max-attitude-p95-deg").c_str());
    else if (s == "--init-window-huber-delta-deg") a.init_window_huber_delta_deg = std::atof(next("--init-window-huber-delta-deg").c_str());
    else if (s == "--init-window-min-speed-mps") a.init_window_min_speed_mps = std::atof(next("--init-window-min-speed-mps").c_str());
    else if (s == "--init-max-bg-norm-rad-s") a.init_max_bg_norm_rad_s = std::atof(next("--init-max-bg-norm-rad-s").c_str());
    else if (s == "--init-max-ba-norm-mps2") a.init_max_ba_norm_mps2 = std::atof(next("--init-max-ba-norm-mps2").c_str());
    else if (s == "--gps-antenna-in-imu") {
      a.gps_antenna_in_imu.x() = std::atof(next("--gps-antenna-in-imu X").c_str());
      a.gps_antenna_in_imu.y() = std::atof(next("--gps-antenna-in-imu Y").c_str());
      a.gps_antenna_in_imu.z() = std::atof(next("--gps-antenna-in-imu Z").c_str());
    }
    else if (s == "--no-display") a.show = false;
    else if (s == "--no-dashboard") {
      a.dashboard_enabled = false;
      a.show = false;
    }
    else if (s == "--dash-title") a.dash_title = next("--dash-title");
    else if (s == "--dash-every") a.dash_every = std::atoi(next("--dash-every").c_str());
    else if (s == "--cam-subsample" || s == "--camera-frame-stride")
      a.cam_subsample = std::max(1, std::atoi(next(s.c_str()).c_str()));
    else if (s == "--post-alignment-camera-frame-stride")
      a.post_alignment_camera_frame_stride =
          std::max(1, std::atoi(next(s.c_str()).c_str()));
    else if (s == "--post-alignment-visual-roi") {
      a.post_alignment_visual_roi = next(s.c_str());
      if (!valid_post_alignment_visual_roi(a.post_alignment_visual_roi))
        throw std::runtime_error("invalid --post-alignment-visual-roi: " +
                                 a.post_alignment_visual_roi);
    }
    else if (s == "--dynamic-turn-roi-diag")
      a.dynamic_turn_roi_diag_path = next(s.c_str());
    else if (s == "--agl-scene-scale")
      a.agl_scene_scale_mode = next(s.c_str());
    else if (s == "--agl-scene-scale-diag")
      a.agl_scene_scale_diag_path = next(s.c_str());
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
    else if (s == "--post-alignment-gyro-z-perturbation")
      a.post_alignment_gyro_z_perturbation_rad_s =
          std::atof(next("--post-alignment-gyro-z-perturbation").c_str());
    else if (s == "--post-alignment-gyro-z-scale")
      a.post_alignment_gyro_z_scale =
          std::atof(next("--post-alignment-gyro-z-scale").c_str());
    else if (s == "--post-alignment-gyro-z-scale-min-abs")
      a.post_alignment_gyro_z_scale_min_abs_rad_s =
          std::atof(next("--post-alignment-gyro-z-scale-min-abs").c_str());
    else if (s == "--post-alignment-fc-yaw-aid")
      a.post_alignment_fc_yaw_aid = true;
    else if (s == "--post-alignment-fc-yaw-aid-period")
      a.post_alignment_fc_yaw_aid_period_s =
          std::atof(next("--post-alignment-fc-yaw-aid-period").c_str());
    else if (s == "--post-alignment-fc-yaw-aid-sigma-deg")
      a.post_alignment_fc_yaw_aid_sigma_deg =
          std::atof(next("--post-alignment-fc-yaw-aid-sigma-deg").c_str());
    else if (s == "--post-alignment-fc-yaw-aid-log")
      a.post_alignment_fc_yaw_aid_log_path =
          next("--post-alignment-fc-yaw-aid-log");
    else if (s == "--post-alignment-camera-extrinsic-left-rotvec-deg") {
      a.post_alignment_camera_extrinsic_left_rotvec_deg.x() =
          std::atof(next("--post-alignment-camera-extrinsic-left-rotvec-deg").c_str());
      a.post_alignment_camera_extrinsic_left_rotvec_deg.y() =
          std::atof(next("--post-alignment-camera-extrinsic-left-rotvec-deg").c_str());
      a.post_alignment_camera_extrinsic_left_rotvec_deg.z() =
          std::atof(next("--post-alignment-camera-extrinsic-left-rotvec-deg").c_str());
    }
    else if (s == "--disable-slam-features") a.disable_slam_features = true;
    else if (s == "--online-alignment-use-cli-init-covariance")
      a.online_alignment_use_cli_init_covariance = true;
    else if (s == "--online-alignment-local-estimator-origin")
      a.online_alignment_local_estimator_origin = true;
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
    else if (s == "--msckf-yaw-freeze-window") {
      a.msckf_yaw_freeze_t0 = std::atof(next("--msckf-yaw-freeze-window T0").c_str());
      a.msckf_yaw_freeze_t1 = std::atof(next("--msckf-yaw-freeze-window T1").c_str());
    }
    else if (s == "--visual-yaw-gain-zero-window") {
      a.visual_yaw_gain_zero_t0 =
          std::atof(next("--visual-yaw-gain-zero-window T0").c_str());
      a.visual_yaw_gain_zero_t1 =
          std::atof(next("--visual-yaw-gain-zero-window T1").c_str());
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
  if (a.online_alignment_release_policy != "practical_navigation_start" &&
      a.online_alignment_release_policy != "strict_full_alignment") {
    std::cerr << "invalid --online-alignment-release-policy: "
              << a.online_alignment_release_policy << "\n";
    return false;
  }
  if (a.online_alignment_visual_perturbation_px < 0.0 ||
      a.online_alignment_visual_perturbation_fraction < 0.0 ||
      a.online_alignment_visual_perturbation_fraction > 1.0) {
    std::cerr << "online alignment visual perturbation must use PX >= 0 "
                 "and fraction in [0,1]\n";
    return false;
  }
  if (!std::isfinite(a.post_alignment_gyro_z_scale) ||
      a.post_alignment_gyro_z_scale <= 0.0 ||
      !std::isfinite(a.post_alignment_gyro_z_scale_min_abs_rad_s) ||
      a.post_alignment_gyro_z_scale_min_abs_rad_s < 0.0) {
    std::cerr << "post-alignment gyro-z scale must be finite and positive, "
                 "and its activation threshold must be finite and nonnegative\n";
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
  const bool initialization_mode_explicit = !args.initialization_mode.empty();
  if (args.initialization_mode.empty()) {
    if (args.init_from_fc_path.empty())
      args.initialization_mode = "vio_only";
    else if (args.init_from_fc_position_frame == "global_gnav")
      args.initialization_mode = "fc_full_state";
    else
      args.initialization_mode = "fc_attitude_only";
    PRINT_WARNING(YELLOW "[ros-free] inferred legacy --initialization-mode=%s; clean-baseline runs must pass it explicitly\n" RESET,
                  args.initialization_mode.c_str());
  }
  InitializationMode initialization_mode;
  FCInitLevel requested_fc_init_level;
  FCInitFallback requested_fc_init_fallback;
  try {
    initialization_mode = parse_initialization_mode(args.initialization_mode);
    requested_fc_init_level = parse_fc_init_level(args.fc_init_level);
    requested_fc_init_fallback = parse_fc_init_fallback(args.fc_init_fallback);
  } catch (const std::exception &error) {
    PRINT_ERROR(RED "[ros-free] invalid initialization contract: %s\n" RESET, error.what());
    return EXIT_FAILURE;
  }
  if (args.adaptive_stride && args.adaptive_stride_shadow) {
    PRINT_ERROR(RED "[adaptive-stride] active and shadow modes are mutually exclusive\n" RESET);
    return EXIT_FAILURE;
  }
  if ((args.adaptive_stride || args.adaptive_stride_shadow) && args.camera_frame_adaptive) {
    PRINT_ERROR(RED "[adaptive-stride] cannot be combined with --camera-frame-adaptive\n" RESET);
    return EXIT_FAILURE;
  }
  // In P5 active mode camera-frame-stride is the pre-initialization cadence.
  // Once P4 releases the state, the causal state machine owns separate
  // tracking/backend cadences. The clean P4 contract therefore legitimately
  // starts at fixed stride12 instead of forcing raw-frame stride1.
  if ((args.adaptive_stride || args.adaptive_stride_shadow) && args.adaptive_stride_log_path.empty())
    args.adaptive_stride_log_path = args.output_path + ".adaptive_stride.csv";
  if (args.pose_repair_sim_gps && args.gps_path.empty()) {
    PRINT_ERROR(RED "[pose-repair] --pose-repair-sim-gps requires --gps PATH\n" RESET);
    return EXIT_FAILURE;
  }
  const bool agl_scene_scale_enabled =
      args.agl_scene_scale_mode != "off";
  if (agl_scene_scale_enabled && args.gps_alt_update) {
    PRINT_ERROR(RED "[AGL-SCALE] scene-scale reset and --gps-alt-update are "
                    "mutually exclusive; AGL must not enter p_z\n" RESET);
    return EXIT_FAILURE;
  }
  if (agl_scene_scale_enabled && args.gps_path.empty()) {
    PRINT_ERROR(RED "[AGL-SCALE] current AGL source requires --gps PATH; "
                    "GPS XY/course remain evaluation-only\n" RESET);
    return EXIT_FAILURE;
  }
  if (args.post_alignment_fc_yaw_aid &&
      initialization_mode != InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT) {
    PRINT_ERROR(RED "[FC-YAW-AID] requires online_multisensor_alignment and its accepted FC stream\n" RESET);
    return EXIT_FAILURE;
  }
  if (!args.dashboard_enabled && !args.video_path.empty()) {
    PRINT_ERROR(RED "[viz] --no-dashboard cannot be combined with --video\n" RESET);
    return EXIT_FAILURE;
  }
  if (!args.dashboard_enabled && !args.dashboard_alignment_json_path.empty()) {
    PRINT_ERROR(RED "[viz] --no-dashboard cannot be combined with --dashboard-alignment-json\n" RESET);
    return EXIT_FAILURE;
  }
  if (!args.init_from_fc_path.empty() && args.init_from_fc_position_frame.empty()) {
    PRINT_ERROR(RED "[ros-free] --init-from-fc-position-frame is required with --init-from-fc\n" RESET);
    return EXIT_FAILURE;
  }
  if (args.init_from_fc_path.empty() && !args.init_from_fc_position_frame.empty()) {
    PRINT_ERROR(RED "[ros-free] --init-from-fc-position-frame requires --init-from-fc\n" RESET);
    return EXIT_FAILURE;
  }
  if (initialization_mode == InitializationMode::VIO_ONLY &&
      !args.init_from_fc_path.empty()) {
    PRINT_ERROR(RED "[ros-free] vio_only rejects --init-from-fc\n" RESET);
    return EXIT_FAILURE;
  }
  if (initialization_mode != InitializationMode::VIO_ONLY &&
      args.init_from_fc_path.empty()) {
    PRINT_ERROR(RED "[ros-free] FC initialization modes require --init-from-fc\n" RESET);
    return EXIT_FAILURE;
  }
  if (initialization_mode == InitializationMode::FC_FULL_STATE &&
      args.init_from_fc_position_frame != "global_gnav") {
    PRINT_ERROR(RED "[ros-free] fc_full_state requires --init-from-fc-position-frame global_gnav\n" RESET);
    return EXIT_FAILURE;
  }
  if (initialization_mode == InitializationMode::FC_ATTITUDE_ONLY &&
      args.init_from_fc_position_frame != "local_w0_seed") {
    PRINT_ERROR(RED "[ros-free] fc_attitude_only requires --init-from-fc-position-frame local_w0_seed\n" RESET);
    return EXIT_FAILURE;
  }
  if (initialization_mode == InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT) {
    if (args.init_from_fc_position_frame != "global_gnav") {
      PRINT_ERROR(RED "[ONLINE-ALIGN] online mode requires --init-from-fc-position-frame global_gnav\n" RESET);
      return EXIT_FAILURE;
    }
    if (args.cam_subsample != 12) {
      PRINT_ERROR(RED "[ONLINE-ALIGN] online mode requires fixed --camera-frame-stride 12\n" RESET);
      return EXIT_FAILURE;
    }
    // P5 cadence control is dormant until online alignment atomically releases
    // the P4 state, so enabling active/shadow P5 does not alter the frozen P4
    // collection or solve. Repair/restart and the legacy camera controller
    // remain prohibited.
    if (args.camera_frame_adaptive || args.pose_repair_sim_gps ||
        args.restart_supervisor || args.restart_on_pose_repair) {
      PRINT_ERROR(RED "[ONLINE-ALIGN] camera adaptive, pose repair, and restart must be disabled\n" RESET);
      return EXIT_FAILURE;
    }
    if (requested_fc_init_level != FCInitLevel::I0_NEAREST) {
      PRINT_ERROR(RED "[ONLINE-ALIGN] --fc-init-level is not used; omit it or leave I0. Historical I1/I2/I3 are prohibited fallbacks\n" RESET);
      return EXIT_FAILURE;
    }
  }
  if (requested_fc_init_level != FCInitLevel::I0_NEAREST &&
      args.init_from_fc_warn_only) {
    PRINT_ERROR(RED "[ros-free] exact-time initialization is fail-closed and rejects --init-from-fc-warn-only\n" RESET);
    return EXIT_FAILURE;
  }
  if (!initialization_mode_explicit &&
      initialization_mode == InitializationMode::FC_FULL_STATE) {
    PRINT_WARNING(YELLOW "[ros-free] inferred fc_full_state is legacy-compatible but is not a clean-baseline contract\n" RESET);
  }
  if (!std::isfinite(args.start_time) || args.start_time < 0.0) {
    PRINT_ERROR(RED "[ros-free] --start-time must be finite and non-negative\n" RESET);
    return EXIT_FAILURE;
  }
  args.pose_repair_period_s = std::max(1.0, args.pose_repair_period_s);
  args.pose_repair_pos_sigma = std::max(0.01, args.pose_repair_pos_sigma);
  args.pose_repair_yaw_sigma_deg = std::max(0.1, args.pose_repair_yaw_sigma_deg);
  args.pose_repair_gate_sigma = std::max(0.5, args.pose_repair_gate_sigma);
  args.pose_repair_max_gps_age_s = std::max(0.01, args.pose_repair_max_gps_age_s);
  args.post_alignment_fc_yaw_aid_period_s =
      std::max(0.2, args.post_alignment_fc_yaw_aid_period_s);
  args.post_alignment_fc_yaw_aid_sigma_deg =
      std::max(0.1, args.post_alignment_fc_yaw_aid_sigma_deg);
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
  if (initialization_mode == InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT &&
      (params.state_options.do_calib_camera_pose ||
       params.state_options.do_calib_camera_intrinsics ||
       params.state_options.do_calib_camera_timeoffset)) {
    PRINT_ERROR(RED "[ONLINE-ALIGN] June-12 camera intrinsics/distortion, T_C_I, and Camera-IMU time offset must be locked (ext=%d intr=%d toff=%d)\n" RESET,
                params.state_options.do_calib_camera_pose ? 1 : 0,
                params.state_options.do_calib_camera_intrinsics ? 1 : 0,
                params.state_options.do_calib_camera_timeoffset ? 1 : 0);
    return EXIT_FAILURE;
  }
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
                                    params.vio_yaw_update_mode == "global_yaw_oc_centered_projection" ||
                                    params.vio_yaw_update_mode == "global_4d_oc_projection" ||
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
  if (args.post_alignment_visual_roi != "full") {
    if (args.post_alignment_visual_roi == "dynamic_turn") {
      PRINT_INFO(CYAN "[ros-free] causal dynamic turn ROI enabled: left turn->right image, right turn->left image\n" RESET);
    } else {
      PRINT_INFO(CYAN "[ros-free] diagnostic static post-alignment visual ROI=%s\n" RESET,
                 args.post_alignment_visual_roi.c_str());
    }
  }
  if (args.visual_bgz_update_scale < 1.0 - 1e-12) {
    params.visual_bgz_update_scale = std::max(0.0, std::min(1.0, args.visual_bgz_update_scale));
    PRINT_INFO(CYAN "[ros-free] CLI override: visual_bgz_update_scale=%.4f\n" RESET,
               params.visual_bgz_update_scale);
  }
  if ((args.adaptive_stride || args.adaptive_stride_shadow) &&
      args.post_alignment_camera_frame_stride > 0) {
    PRINT_ERROR(RED "[adaptive-stride] cannot be combined with a fixed post-alignment camera stride\n" RESET);
    return EXIT_FAILURE;
  }
  if (args.disable_slam_features) {
    params.state_options.max_slam_features = 0;
    params.state_options.max_slam_in_update = 0;
    PRINT_INFO(CYAN "[ros-free] CLI override: max_slam=0 max_slam_in_update=0\n" RESET);
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
  sys->configure_dynamic_turn_roi(
      args.post_alignment_visual_roi == "dynamic_turn");
  if (!sys->configure_agl_scene_scale(args.agl_scene_scale_mode))
    return EXIT_FAILURE;
  if (args.online_alignment_use_cli_init_covariance) {
    if (!sys->set_online_alignment_release_covariance_override(
            args.init_att_sigma_deg * M_PI / 180.0, args.init_pos_sigma,
            args.init_vel_sigma, args.init_bg_sigma, args.init_ba_sigma)) {
      return EXIT_FAILURE;
    }
    PRINT_INFO(CYAN "[ros-free] P4 covariance ablation: CLI diagonal std "
                    "att=%.6gdeg pos=%.6gm vel=%.6gm/s bg=%.6grad/s ba=%.6gm/s2; "
                    "P4 q/p/v/bg/ba and release time remain unchanged\n" RESET,
               args.init_att_sigma_deg, args.init_pos_sigma,
               args.init_vel_sigma, args.init_bg_sigma, args.init_ba_sigma);
  }
  if (args.online_alignment_local_estimator_origin) {
    sys->set_online_alignment_local_estimator_origin(true);
    PRINT_INFO(CYAN "[ros-free] online alignment will inject a release-centered local W0 position\n" RESET);
  }
  sys->set_tracker_viz_image_payload_enabled(
      args.dashboard_enabled || !args.video_cam_path.empty());
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
  if (args.msckf_yaw_freeze_t0 >= 0.0 &&
      args.msckf_yaw_freeze_t1 > args.msckf_yaw_freeze_t0) {
    sys->set_msckf_yaw_freeze_window(args.msckf_yaw_freeze_t0,
                                     args.msckf_yaw_freeze_t1);
    PRINT_INFO(CYAN "[ros-free] MSCKF yaw freeze window: [%.3f, %.3f]s "
                    "(MSCKF position/velocity, SLAM and GPS-Z unchanged)\n" RESET,
               args.msckf_yaw_freeze_t0, args.msckf_yaw_freeze_t1);
  }
  if (args.visual_yaw_gain_zero_t0 >= 0.0 &&
      args.visual_yaw_gain_zero_t1 > args.visual_yaw_gain_zero_t0) {
    sys->set_visual_yaw_gain_zero_window(args.visual_yaw_gain_zero_t0,
                                         args.visual_yaw_gain_zero_t1);
    PRINT_INFO(CYAN "[ros-free] all-visual current-IMU yaw gain zero window: "
                    "[%.3f, %.3f]s\n" RESET,
               args.visual_yaw_gain_zero_t0, args.visual_yaw_gain_zero_t1);
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
  if (args.post_alignment_camera_extrinsic_left_rotvec_deg.norm() > 1e-12) {
    if (!sys->configure_post_alignment_camera_extrinsic_rotation(
            args.post_alignment_camera_extrinsic_left_rotvec_deg * M_PI / 180.0)) {
      PRINT_ERROR(RED "[ros-free] invalid post-alignment camera extrinsic rotation\n" RESET);
      return EXIT_FAILURE;
    }
  }
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
    PRINT_INFO(CYAN "[adaptive-visual] mode=%s fixed_stride=%d log=%s; "
               "continuous causal geometry (actual dt, compensated KLT motion, "
               "track support, valid AGL footprint, estimator health)\n" RESET,
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
  if (args.gps_alt_guard_fallback_joseph_pz) {
    sys->set_gps_alt_guard_fallback_joseph_pz(true);
    PRINT_INFO(CYAN "[ros-free] GPS alt guarded fallback ENABLED "
               "(unsafe coupled correction -> p_z-only Joseph update)\n" RESET);
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

  if (imu.empty()) {
    PRINT_ERROR(RED "[ros-free] IMU input is empty; startup time contract is undefined\n" RESET);
    return EXIT_FAILURE;
  }
  const double dataset_first_imu_timestamp = imu.front().timestamp;
  const double trim_boundary_timestamp = dataset_first_imu_timestamp + args.start_time;

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
  const bool online_alignment_mode =
      initialization_mode == InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT;
  FCInitSeries online_fc_series;
  size_t online_fc_index = 0;
  FCInitResult fc_init_result;
  FCInitState fc_init_state;
  FCInitPositionBoundary fc_init_position_boundary;
  double applied_init_att_sigma_rad = args.init_att_sigma_deg * M_PI / 180.0;
  double applied_init_vel_sigma = args.init_vel_sigma;
  double applied_init_pos_sigma = args.init_pos_sigma;
  double applied_init_bg_sigma = args.init_bg_sigma;
  double applied_init_ba_sigma = args.init_ba_sigma;
  if (!args.init_from_fc_path.empty()) {
    try {
      if (online_alignment_mode) {
        online_fc_series = load_fc_init_series_csv(args.init_from_fc_path);
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] FC series loaded rows=%zu declarations=%zu\n" RESET,
                   online_fc_series.rows.size(),
                   online_fc_series.declarations.size());
        auto require_declaration = [&](const std::string &key) -> std::string {
          const auto found = online_fc_series.declarations.find(key);
          if (found == online_fc_series.declarations.end() || found->second.empty())
            throw std::runtime_error("online FC stream is missing declaration: " + key);
          return found->second;
        };
        const std::string navigation_frame =
            require_declaration("navigation_frame");
        const std::string fc_body_frame =
            require_declaration("fc_body_frame");
        const std::string board_imu_frame =
            require_declaration("board_imu_frame");
        const std::string attitude_representation =
            require_declaration("attitude_representation");
        const std::string position_frame = require_declaration("position_frame");
        if (attitude_representation != "q_Gnav_to_FC_body_JPL_xyzw")
          throw std::runtime_error(
              "online attitude_representation must be q_Gnav_to_FC_body_JPL_xyzw");
        if (position_frame != "global_gnav")
          throw std::runtime_error("online FC position_frame must be global_gnav");
        if (board_imu_frame != "board_imu")
          throw std::runtime_error(
              "ROS-free board IMU stream frame must be declared board_imu");
        const std::string fc_board_calibration_status =
            require_declaration("fc_board_calibration_status");
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] declarations validated\n" RESET);
        if (fc_board_calibration_status != "accepted_full_flight")
          throw std::runtime_error(
              "online alignment requires an accepted full-flight FC-board "
              "calibration; startup-window mount/time estimation is forbidden");
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] validating FC row contracts\n" RESET);
        for (const auto &row : online_fc_series.rows) {
          const std::vector<std::string> columns =
              fcinit_split_csv(row.source_line);
          if (columns.size() < 21)
            throw std::runtime_error(
                "online FC navigation rows require four quality columns after "
                "the 17 state columns: position_valid,velocity_valid,"
                "attitude_valid,status_valid (source line " +
                std::to_string(row.source_row_index) + ")");
          for (size_t quality_column = 17; quality_column <= 20;
               ++quality_column) {
            if (columns[quality_column] != "0" && columns[quality_column] != "1")
              throw std::runtime_error(
                  "online FC quality columns must be 0 or 1 (source line " +
                  std::to_string(row.source_row_index) + ")");
          }
        }
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] FC row contracts validated\n" RESET);

        OnlineAlignmentOptions online_options;
        // Formal P4: run the upstream OpenVINS visual-inertial dynamic
        // initializer once. The initialized metric local VIO is then paired
        // causally with FC navigation over one real-time window to estimate
        // only global yaw and translation. Metric scale is recovered later by
        // the independent AGL output postprocess, never from FC horizontal
        // velocity. No repeated joint Ceres graph is permitted in this path.
        online_options.upstream_dynamic_init_fc_gauge = true;
        online_options.sliding_window_shadow_only = false;
        online_options.sliding_window_direct_state_release = false;
        online_options.sliding_window_deferred_release_certification = false;
        online_options.sliding_window_terminal_state_covariance_only = false;
        online_options.max_initial_slam_features = 0;
        online_options.max_initial_clones = 0;
        online_options.sliding_window_diagnostic_never_anchor =
            args.online_alignment_diagnostic_never_anchor;
        online_options.fc_attitude_gauge_factor_enabled = true;
        online_options.diagnostic_release_attitude_from_fc =
            args.online_alignment_release_attitude_from_fc;
        online_options.diagnostic_release_graph_attitude_fc_pv_zero_bias =
            args.online_alignment_diagnostic_graph_q_fc_pv_zero_bias;
        online_options.sliding_window_duration_s = 8.0;
        online_options.sliding_window_min_advance_s = 0.5;
        online_options.sliding_window_gauge_stability_duration_s = 8.0;
        online_options.reference_window_duration_s = 8.0;
        online_options.candidate_window_durations_s = {8.0};
        online_options.candidate_short_validation_duration_s = 2.0;
        online_options.candidate_refinement_enabled = true;
        online_options.min_fc_samples = 20;
        online_options.min_imu_samples = 600;
        online_options.min_stereo_frames = 12;
        online_options.min_feature_tracks = 80;
        online_options.min_stereo_depths = 20;
        online_options.min_visual_residual_blocks = 60;
        online_options.max_fc_gap_s = 0.45;
        online_options.max_imu_gap_s = 0.03;
        online_options.max_stereo_gap_s = 0.75;
        online_options.maximum_selected_alignment_frames = 36;
        online_options.max_keyframes =
            online_options.maximum_selected_alignment_frames;
        online_options.max_gyro_bias_norm_rad_s = args.init_max_bg_norm_rad_s;
        online_options.max_accel_bias_norm_mps2 = args.init_max_ba_norm_mps2;
        // Motion is used only to check the accepted external calibration and
        // estimate startup gyro bias. Mounting and timing are locked.
        online_options.min_angular_excitation_rad_s = 0.01;
        online_options.min_second_axis_ratio = 0.005;
        online_options.gravity_G =
            Eigen::Vector3d(0.0, 0.0, params.gravity_mag);
        online_options.camera_to_imu_time_offset_s = params.calib_camimu_dt;
        online_options.expected_navigation_frame = navigation_frame;
        online_options.expected_fc_body_frame = fc_body_frame;
        online_options.expected_board_imu_frame = board_imu_frame;
        online_options.fc_board_calibration_locked = true;
        online_options.R_FtoI_declared = parse_declared_matrix3(
            require_declaration("R_FtoI_calibrated_row_major"),
            "R_FtoI_calibrated_row_major");
        online_options.fc_attitude_to_board_time_offset_s =
            parse_declared_scalar(
                require_declaration(
                    "fc_attitude_to_board_time_offset_s"),
                "fc_attitude_to_board_time_offset_s");
        online_options.fc_navigation_to_board_time_offset_s =
            parse_declared_scalar(
                require_declaration(
                    "fc_navigation_to_board_time_offset_s"),
                "fc_navigation_to_board_time_offset_s");
        online_options.fc_attitude_to_board_time_offset_sigma_s =
            parse_declared_scalar(
                require_declaration(
                    "fc_attitude_to_board_time_offset_sigma_s"),
                "fc_attitude_to_board_time_offset_sigma_s");
        online_options.fc_board_mount_sigma_deg = parse_declared_scalar(
            require_declaration("fc_board_mount_sigma_deg"),
            "fc_board_mount_sigma_deg");
        // The full-flight calibration's fixed-mount sigma is the empirical
        // FC-attitude/board agreement uncertainty for this flight. Use that
        // declared uncertainty for the single full-attitude gauge instead of the
        // generic 2 deg fallback, which made the absolute G_nav gauge almost
        // inert even when a calibration record was accepted.
        online_options.fc_attitude_sigma_deg =
            std::max(0.05, online_options.fc_board_mount_sigma_deg);
        online_options.p_IinF = parse_declared_vector3(
            require_declaration("p_IinF_m"), "p_IinF_m");
        online_options.visual_factors_enabled =
            !args.online_alignment_disable_visual;
        online_options.navigation_allow_without_visual =
            args.online_alignment_navigation_allow_without_visual;
        online_options.visual_perturbation_px =
            args.online_alignment_visual_perturbation_px;
        online_options.visual_perturbation_fraction =
            args.online_alignment_visual_perturbation_fraction;
        online_options.release_policy =
            args.online_alignment_release_policy == "strict_full_alignment"
                ? AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT
                : AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START;
        const auto configure_candidate_gate =
            [&](CandidateStateGroup group, bool require_graph_support,
                bool allow_early_feedback,
                const Eigen::Vector3d &max_abs_error,
                const Eigen::Vector3d &max_std,
                const Eigen::Vector3d &max_std_step,
                const Eigen::Vector3d &max_error_peak_to_peak,
                const Eigen::Vector3d &max_std_peak_to_peak,
                const Eigen::Vector3d &max_feedback_step,
                const Eigen::Vector3d &max_cumulative_feedback) {
              CandidateGroupGate &gate =
                  online_options.candidate_filter_config.group_gates
                      [static_cast<std::size_t>(group)];
              gate.configured = true;
              gate.derive_depth_from_sliding_window = true;
              gate.allow_initial_feedback_before_derived_history =
                  allow_early_feedback;
              gate.require_initial_graph_support = require_graph_support;
              gate.history_duration_s = 0.0;
              gate.history_length_updates = 0;
              gate.min_supported_updates = 0;
              gate.post_feedback_validation_duration_s = 0.0;
              gate.required_post_feedback_stable_updates = 0;
              gate.max_abs_error = max_abs_error;
              gate.max_std = max_std;
              gate.max_std_step = max_std_step;
              gate.max_error_peak_to_peak = max_error_peak_to_peak;
              gate.max_std_peak_to_peak = max_std_peak_to_peak;
              gate.max_feedback_step = max_feedback_step;
              gate.max_cumulative_feedback = max_cumulative_feedback;
            };
        const double deg = M_PI / 180.0;
        configure_candidate_gate(
            CandidateStateGroup::ATTITUDE, true, true,
            Eigen::Vector3d::Constant(1.0 * deg),
            Eigen::Vector3d::Constant(5.0 * deg),
            Eigen::Vector3d::Constant(0.25 * deg),
            Eigen::Vector3d::Constant(0.75 * deg),
            Eigen::Vector3d::Constant(0.50 * deg),
            Eigen::Vector3d::Constant(3.0 * deg),
            Eigen::Vector3d::Constant(3.0 * deg));
        configure_candidate_gate(
            CandidateStateGroup::POSITION, false, false,
            Eigen::Vector3d::Constant(1.0),
            Eigen::Vector3d::Constant(5.0),
            Eigen::Vector3d::Constant(0.25),
            Eigen::Vector3d::Constant(1.0),
            Eigen::Vector3d::Constant(0.50),
            Eigen::Vector3d::Constant(5.0),
            Eigen::Vector3d::Constant(5.0));
        configure_candidate_gate(
            CandidateStateGroup::VELOCITY, false, false,
            Eigen::Vector3d::Constant(0.50),
            Eigen::Vector3d::Constant(2.0),
            Eigen::Vector3d::Constant(0.10),
            Eigen::Vector3d::Constant(0.50),
            Eigen::Vector3d::Constant(0.20),
            Eigen::Vector3d::Constant(3.0),
            Eigen::Vector3d::Constant(3.0));
        configure_candidate_gate(
            CandidateStateGroup::GYRO_BIAS, true, true,
            Eigen::Vector3d::Constant(0.010),
            Eigen::Vector3d::Constant(0.100),
            Eigen::Vector3d::Constant(0.005),
            Eigen::Vector3d::Constant(0.010),
            Eigen::Vector3d::Constant(0.010),
            Eigen::Vector3d::Constant(0.020),
            Eigen::Vector3d::Constant(0.020));
        configure_candidate_gate(
            CandidateStateGroup::ACCEL_BIAS, true, true,
            Eigen::Vector3d::Constant(0.20),
            Eigen::Vector3d::Constant(2.0),
            Eigen::Vector3d::Constant(0.05),
            Eigen::Vector3d::Constant(0.20),
            Eigen::Vector3d::Constant(0.20),
            Eigen::Vector3d::Constant(0.50),
            Eigen::Vector3d::Constant(0.50));
        if (args.online_alignment_retain_bg_prior) {
          online_options.candidate_filter_config.group_gates
              [static_cast<std::size_t>(CandidateStateGroup::GYRO_BIAS)]
                  .configured = false;
          PRINT_INFO(CYAN "[ONLINE-ALIGN][POLICY] bg remains graph-estimated; FC p/v-only candidate bg feedback is disabled\n" RESET);
        }
        if (args.online_alignment_retain_ba_prior) {
          online_options.candidate_filter_config.group_gates
              [static_cast<std::size_t>(CandidateStateGroup::ACCEL_BIAS)]
                  .configured = false;
          PRINT_INFO(CYAN "[ONLINE-ALIGN][POLICY] ba remains graph-estimated; FC p/v-only candidate ba feedback is disabled\n" RESET);
        }
        online_options.candidate_filter_config.nis_gate_3d = 11.345;
        online_options.candidate_max_closed_loop_attitude_correction_deg =
            3.0;
        online_options.candidate_max_closed_loop_position_correction_m = 5.0;
        online_options.candidate_max_closed_loop_velocity_correction_mps =
            3.0;
        online_options.candidate_max_closed_loop_gyro_bias_correction_rad_s =
            0.020;
        online_options.candidate_max_closed_loop_accel_bias_correction_mps2 =
            0.50;
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] configuring initializer\n" RESET);
        if (!sys->configure_online_alignment(online_options))
          throw std::runtime_error(
              "online alignment could not lock the configured camera calibration; "
              "ordinary OpenVINS initialization fallback is forbidden");
        PRINT_INFO(CYAN "[ONLINE-ALIGN][DEBUG] initializer configured\n" RESET);
        PRINT_INFO(CYAN "[ONLINE-ALIGN] loaded %zu FC navigation samples as a stream contract; no row has been selected or injected\n" RESET,
                   online_fc_series.rows.size());
        PRINT_INFO(CYAN "[ONLINE-ALIGN] frames G=%s F=%s I=%s; shared target-parallax visual scheduler, T_C_I locked, manual 7deg/4.089deg corrections absent\n" RESET,
                   navigation_frame.c_str(), fc_body_frame.c_str(),
                   board_imu_frame.c_str());
        PRINT_INFO(CYAN "[ONLINE-ALIGN] formal P4 uses one upstream dynamic VIO initialization followed by an 8s causal FC/VIO velocity-position Sim(3) window; one atomic reset is allowed; repeated joint Ceres is disabled\n" RESET);
      } else {
      if (cam0.empty())
        throw std::runtime_error("no camera frames remain after --start-time trim; cannot choose FC init by camera time");
      const double fc_init_target_time = cam0.front().timestamp;
      FCInitExactOptions fc_init_options;
      fc_init_options.level = requested_fc_init_level;
      fc_init_options.fallback = requested_fc_init_fallback;
      fc_init_options.max_abs_dt_s = args.init_from_fc_warn_only
                                            ? std::numeric_limits<double>::infinity()
                                            : args.init_from_fc_max_dt;
      fc_init_options.max_bracket_gap_s = args.init_from_fc_max_bracket_gap;
      fc_init_options.window_duration_s = args.init_window_s;
      fc_init_options.min_window_samples = args.init_window_min_samples;
      fc_init_options.max_window_source_gap_s = args.init_window_max_source_gap;
      fc_init_options.max_attitude_residual_p95_deg =
          args.init_window_max_attitude_p95_deg;
      fc_init_options.huber_delta_deg = args.init_window_huber_delta_deg;
      fc_init_options.min_speed_mps = args.init_window_min_speed_mps;
      fc_init_options.max_gyro_bias_norm_rad_s = args.init_max_bg_norm_rad_s;
      fc_init_options.max_accel_bias_norm_mps2 = args.init_max_ba_norm_mps2;
      fc_init_result = load_fc_init_result_csv(
          args.init_from_fc_path, fc_init_target_time, fc_init_options);
      fc_init_state = fc_init_result.state;
      fc_init_state.timestamp = fc_init_target_time;
      if (!fc_init_state.declared_position_frame.empty() &&
          fc_init_state.declared_position_frame != args.init_from_fc_position_frame) {
        throw std::runtime_error(
            "FC init position_frame declaration does not match --init-from-fc-position-frame");
      }
      if (initialization_mode == InitializationMode::FC_ATTITUDE_ONLY) {
        fc_init_state.v_IinG.setZero();
        fc_init_state.p_IinG.setZero();
      }
      fc_init_position_boundary = prepare_fc_init_position(
          fc_init_state, parse_fc_init_position_frame(args.init_from_fc_position_frame));
      if (fc_init_result.applied_level == FCInitLevel::I3_QUALITY_GATED) {
        if (fc_init_result.window.attitude_std_rad.allFinite())
          applied_init_att_sigma_rad = std::max(
              applied_init_att_sigma_rad,
              fc_init_result.window.attitude_std_rad.maxCoeff());
        if (std::isfinite(fc_init_result.window.velocity_std_mps))
          applied_init_vel_sigma = std::max(
              applied_init_vel_sigma, fc_init_result.window.velocity_std_mps);
        if (std::isfinite(fc_init_result.window.position_std_m))
          applied_init_pos_sigma = std::max(
              applied_init_pos_sigma, fc_init_result.window.position_std_m);
        if (std::isfinite(fc_init_result.window.gyro_bias_std_rad_s))
          applied_init_bg_sigma = std::max(
              applied_init_bg_sigma, fc_init_result.window.gyro_bias_std_rad_s);
        if (std::isfinite(fc_init_result.window.accel_bias_std_mps2))
          applied_init_ba_sigma = std::max(
              applied_init_ba_sigma, fc_init_result.window.accel_bias_std_mps2);
      }
      fc_init_pending = true;
      PRINT_INFO(CYAN "[ros-free] loaded FC init CSV: %s\n" RESET, args.init_from_fc_path.c_str());
      PRINT_INFO(CYAN "[ros-free] FC init position frame=%s source_p=[%.3f %.3f %.3f] local_seed_p=[%.3f %.3f %.3f]\n" RESET,
                 fc_init_position_frame_name(fc_init_position_boundary.source_frame),
                 fc_init_state.p_IinG.x(), fc_init_state.p_IinG.y(), fc_init_state.p_IinG.z(),
                 fc_init_position_boundary.local_seed.p_IinG.x(),
                 fc_init_position_boundary.local_seed.p_IinG.y(),
                 fc_init_position_boundary.local_seed.p_IinG.z());
      PRINT_INFO(CYAN "[ros-free] FC init mode=%s requested=%s applied=%s method=%s target=%.6f before=%.6f after=%.6f gap=%.6f alpha=%.6f fallback=%d\n" RESET,
                 initialization_mode_name(initialization_mode),
                 fc_init_level_name(fc_init_result.requested_level),
                 fc_init_level_name(fc_init_result.applied_level),
                 fc_init_result.selection_method.c_str(), fc_init_target_time,
                 fc_init_result.bracket.before.state.timestamp,
                 fc_init_result.bracket.after.state.timestamp,
                 fc_init_result.bracket.gap_s, fc_init_result.bracket.alpha,
                 fc_init_result.fallback_applied ? 1 : 0);
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
                 applied_init_att_sigma_rad * 180.0 / M_PI,
                 applied_init_vel_sigma, applied_init_pos_sigma,
                 applied_init_bg_sigma, applied_init_ba_sigma);
      if (params.use_gyro_aided_klt && params.use_gyro_aided_klt_max_bg_sigma > 0.0 &&
          args.init_bg_sigma > params.use_gyro_aided_klt_max_bg_sigma) {
        PRINT_WARNING(YELLOW "[ros-free] FC init bg sigma %.5f exceeds gyro-aided KLT gate %.5f; "
                             "rotation-aided tracking will be disabled until bg covariance shrinks.\n" RESET,
                      args.init_bg_sigma, params.use_gyro_aided_klt_max_bg_sigma);
      }
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
  vo.show_window = args.dashboard_enabled && args.show;
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
  if (args.canonical_init_state_path.empty())
    args.canonical_init_state_path = sibling_path(args.output_path, "canonical_init_state.json");
  if (args.online_alignment_metadata_path.empty())
    args.online_alignment_metadata_path =
        sibling_path(args.output_path, "online_alignment_metadata.json");
  if (args.online_alignment_window_trace_path.empty())
    args.online_alignment_window_trace_path = sibling_path(
        args.output_path, "online_alignment_sliding_windows.csv");

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
  debug_out << std::fixed << std::setprecision(9);

  std::ofstream dynamic_turn_roi_out;
  if (args.post_alignment_visual_roi == "dynamic_turn") {
    if (args.dynamic_turn_roi_diag_path.empty())
      args.dynamic_turn_roi_diag_path =
          sibling_path(args.output_path, "dynamic_turn_roi.csv");
    fs::path p(args.dynamic_turn_roi_diag_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    dynamic_turn_roi_out.open(args.dynamic_turn_roi_diag_path,
                              std::ofstream::out | std::ofstream::trunc);
    if (!dynamic_turn_roi_out.is_open()) {
      PRINT_ERROR(RED "[dynamic-turn-roi] cannot open diagnostics: %s\n" RESET,
                  args.dynamic_turn_roi_diag_path.c_str());
      return EXIT_FAILURE;
    }
    dynamic_turn_roi_out
        << "timestamp,signal_valid,raw_yaw_rate_radps,filtered_yaw_rate_radps,"
        << "direction,preferred_side,strength,preferred_fraction,"
        << "detection_exclusion_fraction,reason,tracking_frame_fed,"
        << "backend_frame_fed,detection_masked_pixels,msckf_available,"
        << "msckf_selected_preferred,msckf_selected_other,slam_available,"
        << "slam_selected_preferred,slam_selected_other,"
        << "delayed_slam_available,delayed_slam_selected_preferred,"
        << "delayed_slam_selected_other\n";
    dynamic_turn_roi_out << std::fixed << std::setprecision(9);
  }

  std::ofstream agl_scene_scale_out;
  size_t agl_scene_scale_last_logged_evaluation = 0;
  if (agl_scene_scale_enabled) {
    if (args.agl_scene_scale_diag_path.empty())
      args.agl_scene_scale_diag_path =
          sibling_path(args.output_path, "agl_scene_scale.csv");
    fs::path p(args.agl_scene_scale_diag_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    agl_scene_scale_out.open(args.agl_scene_scale_diag_path,
                             std::ofstream::out | std::ofstream::trunc);
    if (!agl_scene_scale_out.is_open()) {
      PRINT_ERROR(RED "[AGL-SCALE] cannot open diagnostics: %s\n" RESET,
                  args.agl_scene_scale_diag_path.c_str());
      return EXIT_FAILURE;
    }
    agl_scene_scale_out
        << "timestamp,mode,agl_timestamp,agl_m,agl_age_s,ground_valid,"
        << "ground_reason,map_height_m,map_height_sigma_m,plane_tilt_deg,"
        << "plane_residual_mad_m,plane_residual_rmse_m,input_features,"
        << "candidate_features,inlier_features,inlier_ratio,image_cells,"
        << "u_span_fraction,v_span_fraction,distinct_clone_times,"
        << "window_coverage_s,window_max_gap_s,median_ratio,log_ratio_mad,"
        << "candidate_scale,proposed_scale,cumulative_scale,decision_ready,"
        << "decision_reason,reset_attempted,reset_succeeded,state_modified,"
        << "applied_scale,reset_reason,covariance_min_eigenvalue,"
        << "covariance_psd_projected,covariance_psd_projection_magnitude,"
        << "covariance_psd_projection_limit,"
        << "evaluation_count,valid_ground_count,proposal_count,"
        << "reset_attempt_count,reset_success_count\n";
    agl_scene_scale_out << std::fixed << std::setprecision(9);
  }

  std::ofstream provisional_navigation_out;
  if (online_alignment_mode &&
      args.online_alignment_diagnostic_never_anchor) {
    const std::string provisional_path =
        sibling_path(args.output_path, "provisional_navigation.csv");
    provisional_navigation_out.open(
        provisional_path, std::ofstream::out | std::ofstream::trunc);
    if (!provisional_navigation_out.is_open()) {
      PRINT_ERROR(RED "[ONLINE-ALIGN] cannot open provisional output: %s\n" RESET,
                  provisional_path.c_str());
      return EXIT_FAILURE;
    }
    provisional_navigation_out
        << "timestamp,px,py,pz,vx,vy,vz,qx,qy,qz,qw,bgx,bgy,bgz,bax,bay,baz,"
        << "cov_qx,cov_qy,cov_qz,cov_px,cov_py,cov_pz,cov_vx,cov_vy,cov_vz,"
        << "cov_bgx,cov_bgy,cov_bgz,cov_bax,cov_bay,cov_baz,provisional,"
        << "official_openvins_state,navigation_frame,source,joint_initialization_pending_reason\n";
    provisional_navigation_out << std::fixed << std::setprecision(9);
  }

  // P4 owns its finite-window frame selector and receives every causal startup
  // image. P5 is the only owner of this scheduler and starts from a fresh dense
  // state after P4 has injected the OpenVINS state.
  AdaptiveVisualScheduler adaptive_visual_scheduler;
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
        << "timestamp,mode,decision_reason,tracking_gap,instantaneous_safe_gap,"
        << "tracking_gap_changed,immediate_contraction,expansion_confirmed,"
        << "new_motion_measurement,raw_camera_dt_s,actual_tracking_interval_s,"
        << "motion_timestamp_s,motion_age_s,motion_dt_s,common_tracks,"
        << "effective_tracks,survival_ratio,median_track_age,grid_occupancy,"
        << "grid_entropy,border_track_ratio,raw_p95_px,rotation_p95_px,"
        << "rotation_max_px,compensated_median_px,compensated_p95_px,"
        << "compensated_sigma_px,compensated_p95_ucb_px,target_horizon_s,"
        << "safe_horizon_s,interval_horizon_s,compensated_horizon_s,"
        << "raw_horizon_s,rotation_horizon_s,survival_horizon_s,"
        << "predicted_compensated_median_px,predicted_compensated_p95_ucb_px,"
        << "estimator_protection,tracking_protection,active_features,"
        << "agl_m,agl_source,agl_age_s,ground_geometry_valid,"
        << "ground_overlap_horizon_s,predicted_ground_overlap,"
        << "visual_residual_p95_px,time_since_accepted_backend_update_s,"
        << "covariance_all_finite,covariance_negative_diagonal_count,"
        << "visual_state_correction_valid,visual_position_correction_m,"
        << "visual_velocity_correction_mps,visual_attitude_correction_deg,run_mode,"
        << "tracking_frame_fed,backend_clone_created,raw_frames_since_tracking,"
        << "time_since_tracking_s,time_since_backend_clone_s,backend_trigger,"
        << "backend_reference_committed,backend_reason,backend_information_score,"
        << "backend_elapsed_since_clone_s,backend_minimum_temporal_separation_s,"
        << "backend_temporal_support_ready,backend_translation_baseline_px,"
        << "backend_translation_p95_ucb_px,backend_effective_tracks,"
        << "backend_survival_ratio,backend_grid_occupancy,"
        << "backend_visual_geometry_valid,"
        << "backend_pure_rotation,backend_information_trigger,"
        << "backend_latency_trigger,backend_termination_trigger,"
        << "backend_forced_without_visual_information\n";
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

  std::ofstream fc_yaw_aid_out;
  if (args.post_alignment_fc_yaw_aid) {
    if (args.post_alignment_fc_yaw_aid_log_path.empty())
      args.post_alignment_fc_yaw_aid_log_path =
          args.output_path + ".fc_yaw_aid.csv";
    fs::path p(args.post_alignment_fc_yaw_aid_log_path);
    if (!p.parent_path().empty())
      fs::create_directories(p.parent_path());
    fc_yaw_aid_out.open(args.post_alignment_fc_yaw_aid_log_path,
                        std::ofstream::out | std::ofstream::trunc);
    if (!fc_yaw_aid_out.is_open()) {
      PRINT_ERROR(RED "[FC-YAW-AID] failed to open %s\n" RESET,
                  args.post_alignment_fc_yaw_aid_log_path.c_str());
      return EXIT_FAILURE;
    }
    fc_yaw_aid_out
        << "camera_time,fc_attitude_time,yaw_measurement_deg,yaw_residual_deg,"
           "nis,gate_ratio,predicted_yaw_correction_deg,accepted,decision\n";
    fc_yaw_aid_out << std::fixed << std::setprecision(9);
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
  AlignmentReadiness online_alignment_metadata_readiness =
      AlignmentReadiness::NOT_READY;
  double online_alignment_first_output_time = -1.0;
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
  size_t adaptive_raw_frames_since_tracking = 0;
  double adaptive_last_raw_camera_time =
      std::numeric_limits<double>::quiet_NaN();
  double adaptive_last_parallax_packet_time =
      -std::numeric_limits<double>::infinity();
  double adaptive_last_tracker_packet_time =
      -std::numeric_limits<double>::infinity();
  double adaptive_last_fed_camera_time =
      std::numeric_limits<double>::quiet_NaN();
  double p4_dynamic_last_fed_camera_time =
      std::numeric_limits<double>::quiet_NaN();
  double adaptive_last_backend_camera_time =
      std::numeric_limits<double>::quiet_NaN();
  std::unordered_map<size_t, int> adaptive_track_age;
  std::unordered_set<size_t> adaptive_previous_track_ids;
  double adaptive_median_track_age = std::numeric_limits<double>::quiet_NaN();
  size_t adaptive_tracking_stride_change_count = 0;
  size_t adaptive_emergency_downshift_count = 0;
  size_t adaptive_last_visual_accept_count = 0;
  double adaptive_last_accepted_backend_update_time =
      std::numeric_limits<double>::quiet_NaN();
  bool adaptive_cadence_started = false;
  bool adaptive_planner_started = false;
  bool adaptive_have_covariance_snapshot = false;
  double adaptive_covariance_snapshot_time = -1.0;
  bool adaptive_covariance_all_finite = true;
  int adaptive_covariance_negative_diagonal_count = 0;
  size_t adaptive_switch_count = 0;
  size_t adaptive_down_switch_count = 0;
  size_t adaptive_up_switch_count = 0;
  AdaptiveVisualSchedulerDecision adaptive_decision;
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
  double fc_yaw_aid_next_time = -1.0;
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
    next_sys->configure_dynamic_turn_roi(
        args.post_alignment_visual_roi == "dynamic_turn");
    next_sys->configure_agl_scene_scale(args.agl_scene_scale_mode);
    next_sys->set_tracker_viz_image_payload_enabled(
        args.dashboard_enabled || !args.video_cam_path.empty());
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
    if (args.post_alignment_camera_extrinsic_left_rotvec_deg.norm() > 1e-12) {
      if (!next_sys->configure_post_alignment_camera_extrinsic_rotation(
              args.post_alignment_camera_extrinsic_left_rotvec_deg * M_PI / 180.0)) {
        PRINT_ERROR(RED "[ros-free] invalid post-alignment camera extrinsic rotation on restart\n" RESET);
        std::exit(EXIT_FAILURE);
      }
    }
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
    if (args.msckf_yaw_freeze_t0 >= 0.0 &&
        args.msckf_yaw_freeze_t1 > args.msckf_yaw_freeze_t0)
      next_sys->set_msckf_yaw_freeze_window(args.msckf_yaw_freeze_t0,
                                            args.msckf_yaw_freeze_t1);
    if (args.visual_yaw_gain_zero_t0 >= 0.0 &&
        args.visual_yaw_gain_zero_t1 > args.visual_yaw_gain_zero_t0)
      next_sys->set_visual_yaw_gain_zero_window(args.visual_yaw_gain_zero_t0,
                                                args.visual_yaw_gain_zero_t1);
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
    if (args.gps_alt_guard_fallback_joseph_pz)
      next_sys->set_gps_alt_guard_fallback_joseph_pz(true);
  };

  auto maybe_align = [&](double t) {
    if (!args.dashboard_enabled)
      return;
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

  const auto formal_navigation_ready = [&]() -> bool {
    return sys->initialized() &&
           (!online_alignment_mode || sys->online_alignment_complete());
  };

  while (!g_stop.load() && (imu_i < imu.size() || cam_i < cam0.size())) {
    double t_imu = imu_i < imu.size() ? imu[imu_i].timestamp : INF;
    double t_cam = cam_i < cam0.size() ? cam0[cam_i].timestamp : INF;
    if (t_cam > args.until_time && t_imu > args.until_time) break;

    if (online_alignment_mode) {
      const double causal_horizon = std::min(t_imu, t_cam);
      while (online_fc_index < online_fc_series.rows.size() &&
             online_fc_series.rows[online_fc_index].state.timestamp <=
                 causal_horizon) {
        const auto &row = online_fc_series.rows[online_fc_index];
        const std::vector<std::string> columns =
            fcinit_split_csv(row.source_line);
        FCNavigationSample fc_message;
        fc_message.timestamp = row.state.timestamp;
        fc_message.position_G = row.state.p_IinG;
        fc_message.velocity_G = row.state.v_IinG;
        fc_message.q_GtoF = row.state.q_GtoI;
        fc_message.navigation_frame =
            online_fc_series.declarations.at("navigation_frame");
        fc_message.body_frame =
            online_fc_series.declarations.at("fc_body_frame");
        fc_message.position_valid = columns[17] == "1";
        fc_message.velocity_valid = columns[18] == "1";
        fc_message.attitude_valid = columns[19] == "1";
        fc_message.status_valid = columns[20] == "1";
        if (!sys->feed_measurement_fc_navigation(fc_message) &&
            !sys->online_alignment_complete()) {
          PRINT_WARNING(YELLOW "[ONLINE-ALIGN] rejected FC stream row %d at t=%.6f\n" RESET,
                        row.source_row_index, row.state.timestamp);
        }
        ++online_fc_index;
      }
    }

    if (t_imu <= t_cam) {
      if (online_alignment_mode) {
        BoardImuSample board;
        board.timestamp = t_imu;
        board.angular_velocity = imu[imu_i].gyro;
        if (formal_navigation_ready()) {
          if (std::abs(board.angular_velocity.z()) >=
              args.post_alignment_gyro_z_scale_min_abs_rad_s) {
            board.angular_velocity.z() *= args.post_alignment_gyro_z_scale;
          }
          board.angular_velocity.z() +=
              args.post_alignment_gyro_z_perturbation_rad_s;
        }
        board.linear_acceleration = imu[imu_i].accel;
        board.frame = "board_imu";
        board.status_valid = board.angular_velocity.allFinite() &&
                             board.linear_acceleration.allFinite();
        board.gyro_saturated = board.angular_velocity.cwiseAbs().maxCoeff() >= 20.0;
        board.accel_saturated =
            board.linear_acceleration.cwiseAbs().maxCoeff() >= 160.0;
        sys->feed_measurement_board_imu(board);
      } else {
        ov_core::ImuData m;
        m.timestamp = t_imu;
        m.wm = imu[imu_i].gyro;
        m.am = imu[imu_i].accel;
        sys->feed_measurement_imu(m);
      }
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
    msg.sensor_timestamps.push_back(t_cam);
    msg.images.push_back(img0);
    msg.masks.push_back(cv::Mat::zeros(img0.size(), CV_8UC1));

    if (args.stereo && !cam1.empty()) {
      while (cam1_i < cam1.size() && cam1[cam1_i].timestamp < t_cam - 0.02)
        cam1_i++;
      if (cam1_i < cam1.size() && std::fabs(cam1[cam1_i].timestamp - t_cam) < 0.02) {
        cv::Mat img1 = cv::imread(cam1[cam1_i].image_path, cv::IMREAD_GRAYSCALE);
        if (!img1.empty()) {
          msg.sensor_ids.push_back(1);
          msg.sensor_timestamps.push_back(cam1[cam1_i].timestamp);
          msg.images.push_back(img1);
          msg.masks.push_back(cv::Mat::zeros(img1.size(), CV_8UC1));
        }
      }
    }

    // Diagnostic causal ROI ablation.  P4 always receives the full image; the
    // mask becomes active only after P4 has formally anchored navigation.
    if (formal_navigation_ready() && args.post_alignment_visual_roi != "full" &&
        args.post_alignment_visual_roi != "dynamic_turn") {
      for (cv::Mat &mask : msg.masks)
        apply_post_alignment_visual_roi(mask, args.post_alignment_visual_roi);
    }

    if (fc_init_pending && !sys->initialized()) {
      sys->initialize_with_fc_state(fc_init_position_boundary.local_seed,
                                    applied_init_att_sigma_rad,
                                    applied_init_vel_sigma,
                                    applied_init_pos_sigma,
                                    applied_init_bg_sigma,
                                    applied_init_ba_sigma,
                                    t_cam);
      try {
        write_canonical_init_state(
            args.canonical_init_state_path, args, initialization_mode,
            fc_init_result, fc_init_state, t_cam, applied_init_att_sigma_rad,
            applied_init_vel_sigma, applied_init_pos_sigma,
            applied_init_bg_sigma, applied_init_ba_sigma);
      } catch (const std::exception &error) {
        PRINT_ERROR(RED "[FC-INIT] canonical init-state write failed: %s\n" RESET,
                    error.what());
        return EXIT_FAILURE;
      }
      if (!nav_frame.valid &&
          initialization_mode == InitializationMode::FC_FULL_STATE) {
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
        auto init_state = sys->get_state();
        const Eigen::Matrix3d R_W0_I0 = init_state->_imu->Rot().transpose();
        const Eigen::Vector3d p_W0_I0 = init_state->_imu->pos();
        const Eigen::Matrix3d R_Gnav_I0 =
            ov_core::quat_2_Rot(fc_init_state.q_GtoI).transpose();
        const Eigen::Vector3d p_Gnav_I0 =
            fc_init_position_boundary.p_IinGnav;
        nav_frame.valid = true;
        nav_frame.dataset_first_imu_timestamp = dataset_first_imu_timestamp;
        nav_frame.requested_start_offset_s = args.start_time;
        nav_frame.trim_boundary_timestamp = trim_boundary_timestamp;
        nav_frame.init_camera_timestamp = t_cam;
        nav_frame.selected_fc_timestamp =
            fc_init_result.applied_level == FCInitLevel::I0_NEAREST
                ? fc_init_result.bracket.before.state.timestamp
                : t_cam;
        nav_frame.fc_time_offset =
            fc_init_result.applied_level == FCInitLevel::I0_NEAREST
                ? fc_init_result.state.source_dt
                : 0.0;
        if (gps_it != gps.end()) {
          nav_frame.gps_timestamp = gps_it->timestamp;
          nav_frame.gps_age = t_cam - gps_it->timestamp;
          nav_frame.p_Gnav_GPS0 = gps_it->xyz;
        }
        nav_frame.R_Gnav_W0 = R_Gnav_I0 * R_W0_I0.transpose();
        nav_frame.p_Gnav_W0 =
            p_Gnav_I0 - nav_frame.R_Gnav_W0 * p_W0_I0;
        nav_frame.p_Gnav_I0 = p_Gnav_I0;
        nav_frame.p_W0_I0 = p_W0_I0;
        nav_frame.fc_source_position = fc_init_state.p_IinG;
        nav_frame.fc_position_frame =
            fc_init_position_frame_name(fc_init_position_boundary.source_frame);
        nav_frame.position_source = "fc_init_global_gnav_position_board_imu";
        nav_frame.initialization_mode = initialization_mode_name(initialization_mode);
        nav_frame.fc_init_requested_level =
            fc_init_level_name(fc_init_result.requested_level);
        nav_frame.fc_init_applied_level =
            fc_init_level_name(fc_init_result.applied_level);
        nav_frame.fc_init_selection_method = fc_init_result.selection_method;
        nav_frame.fc_init_status = fc_init_result.status;
        nav_frame.fc_init_fallback_applied = fc_init_result.fallback_applied;
        nav_frame.fc_init_future_data_used =
            std::isfinite(fc_init_result.window.last_timestamp_s) &&
            fc_init_result.window.last_timestamp_s > t_cam + 1e-9;
        nav_frame.fc_init_available_time = nav_frame.fc_init_future_data_used
                                                 ? fc_init_result.window.last_timestamp_s
                                                 : t_cam;
        nav_frame.init_att_sigma_rad = applied_init_att_sigma_rad;
        nav_frame.init_vel_sigma_mps = applied_init_vel_sigma;
        nav_frame.init_pos_sigma_m = applied_init_pos_sigma;
        nav_frame.init_bg_sigma_rad_s = applied_init_bg_sigma;
        nav_frame.init_ba_sigma_mps2 = applied_init_ba_sigma;
        nav_frame.gps_antenna_in_imu = args.gps_antenna_in_imu;
        const double nav_yaw_deg =
            std::atan2(nav_frame.R_Gnav_W0(1, 0),
                       nav_frame.R_Gnav_W0(0, 0)) * 180.0 / M_PI;
        PRINT_INFO(GREEN "[NAV-FRAME] canonical G_nav fixed at t_seed=%.6f from FC full state; yaw=%.3fdeg p=[%.3f %.3f %.3f], GPS_input=0 future_data=%d available_t=%.6f\n" RESET,
                   t_cam, nav_yaw_deg, nav_frame.p_Gnav_W0.x(),
                   nav_frame.p_Gnav_W0.y(), nav_frame.p_Gnav_W0.z(),
                   nav_frame.fc_init_future_data_used ? 1 : 0,
                   nav_frame.fc_init_available_time);
      } else if (initialization_mode == InitializationMode::FC_ATTITUDE_ONLY) {
        PRINT_INFO(CYAN "[FC-INIT] attitude-only seed established in local W0; absolute G_nav output intentionally unavailable\n" RESET);
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
    AdaptiveVisualSchedulerInput adaptive_input;
    double adaptive_agl_m = std::numeric_limits<double>::quiet_NaN();
    double adaptive_agl_age_s = std::numeric_limits<double>::infinity();
    double adaptive_agl_timestamp_s = -1.0;
    std::string adaptive_agl_source = "unavailable";
    // One causal altitude lookup shared by P5 geometry and the independent
    // AGL scene-scale path. Horizontal GPS and course are not passed here.
    if (!gps.empty()) {
      auto altitude_sample = std::upper_bound(
          gps.begin(), gps.end(), t_cam,
          [](double timestamp,
             const DatasetReaderEuroc::GpsSample &sample) {
            return timestamp < sample.timestamp;
          });
      if (altitude_sample != gps.begin()) {
        --altitude_sample;
        adaptive_agl_timestamp_s = altitude_sample->timestamp;
        adaptive_agl_age_s = t_cam - altitude_sample->timestamp;
        if (adaptive_agl_age_s >= 0.0 && adaptive_agl_age_s <= 2.0 &&
            std::isfinite(altitude_sample->xyz.z()) &&
            altitude_sample->xyz.z() > 0.5) {
          adaptive_agl_m = altitude_sample->xyz.z();
          adaptive_agl_source = "past_gps_relative_u";
        }
      }
    }
    if (agl_scene_scale_enabled && formal_navigation_ready() &&
        adaptive_agl_timestamp_s >= 0.0 &&
        std::isfinite(adaptive_agl_m)) {
      sys->feed_measurement_agl(adaptive_agl_timestamp_s, adaptive_agl_m,
                                true);
    }
    const bool p4_visual_cadence_active =
        online_alignment_mode && !sys->online_alignment_complete();
    const bool p5_visual_cadence_active =
        (args.adaptive_stride || args.adaptive_stride_shadow) &&
        formal_navigation_ready();
    const bool visual_cadence_planner_active = p5_visual_cadence_active;
    const int adaptive_previous_gap =
        adaptive_visual_scheduler.tracking_gap();
    if (p5_visual_cadence_active && !adaptive_planner_started) {
      adaptive_planner_started = true;
      adaptive_last_raw_camera_time =
          std::numeric_limits<double>::quiet_NaN();
      adaptive_last_fed_camera_time =
          std::numeric_limits<double>::quiet_NaN();
      adaptive_last_tracker_packet_time =
          -std::numeric_limits<double>::infinity();
      adaptive_last_parallax_packet_time =
          -std::numeric_limits<double>::infinity();
      adaptive_track_age.clear();
      adaptive_previous_track_ids.clear();
      adaptive_median_track_age = std::numeric_limits<double>::quiet_NaN();
    }
    if (visual_cadence_planner_active &&
        std::isfinite(adaptive_last_raw_camera_time))
      adaptive_input.raw_camera_dt_s =
          t_cam - adaptive_last_raw_camera_time;
    if (visual_cadence_planner_active)
      adaptive_last_raw_camera_time = t_cam;
    if (visual_cadence_planner_active) {
      adaptive_input.timestamp = t_cam;
      adaptive_input.initialized = formal_navigation_ready();
      if (std::isfinite(adaptive_last_fed_camera_time))
        adaptive_input.actual_tracking_interval_s =
            std::max(0.0, t_cam - adaptive_last_fed_camera_time);

      if (sys->initialized()) {
        auto state_pre = sys->get_state();
        const Eigen::Vector3d position = state_pre->_imu->pos();
        const Eigen::Vector3d velocity = state_pre->_imu->vel();
        if (std::isfinite(adaptive_agl_m) &&
            state_pre->_cam_intrinsics_cameras.count(0) > 0 &&
            state_pre->_calib_IMUtoCAM.count(0) > 0) {
          const auto camera_model =
              state_pre->_cam_intrinsics_cameras.at(0);
          GroundFootprintPredictionInput footprint;
          footprint.R_GtoC =
              state_pre->_calib_IMUtoCAM.at(0)->Rot() *
              state_pre->_imu->Rot();
          footprint.p_CinG =
              position - footprint.R_GtoC.transpose() *
                             state_pre->_calib_IMUtoCAM.at(0)->pos();
          footprint.velocity_G = velocity;
          footprint.ground_height_G =
              footprint.p_CinG.z() - adaptive_agl_m;
          const std::array<cv::Point2f, 4> corners = {
              cv::Point2f(0.0f, 0.0f),
              cv::Point2f(static_cast<float>(camera_model->w() - 1), 0.0f),
              cv::Point2f(static_cast<float>(camera_model->w() - 1),
                          static_cast<float>(camera_model->h() - 1)),
              cv::Point2f(0.0f,
                          static_cast<float>(camera_model->h() - 1))};
          for (size_t corner = 0; corner < corners.size(); ++corner) {
            const cv::Point2f normalized =
                camera_model->undistort_cv(corners[corner]);
            footprint.corner_rays_C[corner] =
                Eigen::Vector3d(normalized.x, normalized.y, 1.0);
          }
          footprint.valid = true;
          adaptive_input.ground_footprint = footprint;
        }
        if (!adaptive_have_covariance_snapshot ||
            state_pre->_timestamp > adaptive_covariance_snapshot_time + 1.0e-9) {
          const Eigen::MatrixXd covariance =
              StateHelper::get_full_covariance(state_pre);
          adaptive_covariance_all_finite = covariance.allFinite();
          adaptive_covariance_negative_diagonal_count = 0;
          for (int diagonal = 0; diagonal < covariance.rows(); ++diagonal) {
            if (!std::isfinite(covariance(diagonal, diagonal)) ||
                covariance(diagonal, diagonal) < 0.0)
              ++adaptive_covariance_negative_diagonal_count;
          }
          adaptive_have_covariance_snapshot = true;
          adaptive_covariance_snapshot_time = state_pre->_timestamp;
        }
        adaptive_input.covariance_all_finite =
            adaptive_covariance_all_finite;
        adaptive_input.covariance_negative_diagonal_count =
            adaptive_covariance_negative_diagonal_count;
        const auto &visual_state_correction =
            sys->get_latest_visual_update_state_correction();
        adaptive_input.visual_state_correction_valid =
            visual_state_correction.valid;
        adaptive_input.visual_position_correction_m =
            visual_state_correction.position_m;
        adaptive_input.visual_velocity_correction_mps =
            visual_state_correction.velocity_mps;
        adaptive_input.visual_attitude_correction_deg =
            visual_state_correction.attitude_deg;

        const auto visual_health =
            sys->get_latest_visual_residual_health();
        if (visual_health.valid)
          adaptive_input.visual_residual_p95_px =
              visual_health.p95_residual_px;
        const auto &visual_counts = sys->get_visual_update_counters();
        const size_t visual_accept_count =
            visual_counts.msckf_accept_count +
            visual_counts.regular_slam_accept_count +
            visual_counts.delayed_slam_accept_count;
        if (visual_accept_count > adaptive_last_visual_accept_count) {
          adaptive_last_visual_accept_count = visual_accept_count;
          adaptive_last_accepted_backend_update_time = t_cam;
        }
        if (std::isfinite(adaptive_last_accepted_backend_update_time))
          adaptive_input.time_since_accepted_backend_update_s =
              std::max(0.0, t_cam -
                                adaptive_last_accepted_backend_update_time);
      }

      ov_core::TrackerWarpVizPacket tracking_packet;
      if (sys->get_warp_viz_packet(0, tracking_packet)) {
        adaptive_input.active_feature_count =
            static_cast<int>(tracking_packet.curr_pts_raw.size());
        if (tracking_packet.t_curr >
            adaptive_last_tracker_packet_time + 1.0e-9) {
          std::unordered_set<size_t> current_track_ids;
          std::vector<int> track_ages;
          current_track_ids.reserve(tracking_packet.feature_ids.size());
          track_ages.reserve(tracking_packet.feature_ids.size());
          for (size_t id : tracking_packet.feature_ids) {
            current_track_ids.insert(id);
            const int age = adaptive_previous_track_ids.count(id) > 0
                                ? adaptive_track_age[id] + 1
                                : 1;
            adaptive_track_age[id] = age;
            track_ages.push_back(age);
          }
          for (size_t id : adaptive_previous_track_ids)
            if (current_track_ids.count(id) == 0)
              adaptive_track_age.erase(id);
          adaptive_previous_track_ids = std::move(current_track_ids);
          if (!track_ages.empty()) {
            std::sort(track_ages.begin(), track_ages.end());
            adaptive_median_track_age =
                static_cast<double>(
                    track_ages[track_ages.size() / 2]);
          }
          adaptive_last_tracker_packet_time = tracking_packet.t_curr;
        }
        if (args.adaptive_stride_use_parallax) {
          adaptive_input.motion =
              tracker_motion_metrics(tracking_packet);
          adaptive_input.motion.median_track_age =
              adaptive_median_track_age;
          adaptive_input.motion_measurement_timestamp_s =
              tracking_packet.t_curr;
          if (tracking_packet.t_curr >
              adaptive_last_parallax_packet_time + 1.0e-9)
            adaptive_last_parallax_packet_time =
                tracking_packet.t_curr;
        }
      }

      adaptive_decision = adaptive_visual_scheduler.update(adaptive_input);
      if (adaptive_decision.changed) {
        ++adaptive_switch_count;
        ++adaptive_tracking_stride_change_count;
        if (adaptive_decision.tracking_gap > adaptive_previous_gap)
          ++adaptive_up_switch_count;
        else
          ++adaptive_down_switch_count;
        if (adaptive_decision.immediate_contraction)
          ++adaptive_emergency_downshift_count;
        PRINT_INFO(
            CYAN "[adaptive-visual] t=%.3f mode=%s gap=%d->%d "
                 "safe=%d reason=%s comp=%.2f ucb95=%.2f "
                 "Neff=%.1f coverage=%.2f\n" RESET,
            t_cam, adaptive_decision.mode_name.c_str(),
            adaptive_previous_gap, adaptive_decision.tracking_gap,
            adaptive_decision.instantaneous_safe_gap,
            adaptive_decision.reason.c_str(),
            adaptive_input.motion.compensated_median_px,
            adaptive_input.motion.compensated_p95_ucb_px,
            adaptive_input.motion.effective_track_count,
            adaptive_input.motion.grid_occupancy_ratio);
      }
    }
    const int fixed_camera_stride =
        formal_navigation_ready() && args.post_alignment_camera_frame_stride > 0
            ? args.post_alignment_camera_frame_stride
            : args.cam_subsample;
    bool do_cam_feed = (fixed_camera_stride <= 1) ||
                       (frame_idx % fixed_camera_stride == 0);
    if (p4_visual_cadence_active &&
        sys->online_alignment_uses_upstream_dynamic_init()) {
      // DynamicInitializer needs init_dyn_num_pose distinct camera poses in
      // init_window_time. Derive the startup cadence from those two contracts
      // and real camera timestamps; the old fixed stride=12 can otherwise make
      // the requested initialization mathematically impossible.
      const double target_camera_interval =
          params.init_options.init_window_time /
          static_cast<double>(
              std::max(2, params.init_options.init_dyn_num_pose + 1));
      do_cam_feed =
          !std::isfinite(p4_dynamic_last_fed_camera_time) ||
          t_cam - p4_dynamic_last_fed_camera_time + 1.0e-9 >=
              target_camera_interval;
    }
    if (p5_visual_cadence_active) {
      adaptive_raw_frames_since_feed++;
      adaptive_raw_frames_since_tracking++;
    }
    if (p4_visual_cadence_active) {
      // P4's joint graph uses the frozen startup camera cadence. Its selector
      // may thin further, but it must not silently override
      // --camera-frame-stride.
      adaptive_cadence_started = false;
    } else if (args.adaptive_stride && formal_navigation_ready()) {
      if (!adaptive_cadence_started) {
        adaptive_cadence_started = true;
        do_cam_feed = true;
      } else {
        const bool tracking_due =
            adaptive_raw_frames_since_tracking >=
            static_cast<size_t>(std::max(1, adaptive_decision.tracking_gap));
        do_cam_feed = tracking_due;
      }
    }
    if (args.camera_frame_adaptive && !args.adaptive_stride) {
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

      if (formal_navigation_ready()) {
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
    const double adaptive_time_since_last_feed =
        std::isfinite(adaptive_last_fed_camera_time)
            ? std::max(0.0, t_cam - adaptive_last_fed_camera_time)
            : 0.0;
    const double adaptive_time_since_backend =
        std::isfinite(adaptive_last_backend_camera_time)
            ? std::max(0.0, t_cam - adaptive_last_backend_camera_time)
            : 0.0;
    const size_t backend_count_before =
        sys->get_visual_update_counters().backend_frame_count;
    const bool initialized_before_camera_feed = formal_navigation_ready();
    if (do_cam_feed) {
      if (p4_visual_cadence_active &&
          sys->online_alignment_uses_upstream_dynamic_init())
        p4_dynamic_last_fed_camera_time = t_cam;
      if (args.adaptive_stride && initialized_before_camera_feed) {
        sys->feed_measurement_camera_information_cadence(msg);
      } else {
        sys->feed_measurement_camera(msg);
      }
    }
    if (online_alignment_mode && !sys->online_alignment_complete() &&
        provisional_navigation_out.is_open()) {
      ProvisionalNavigationOutput provisional;
      if (sys->get_online_alignment_provisional(t_cam, provisional)) {
        provisional_navigation_out
            << provisional.timestamp << "," << provisional.p_IinG.x() << ","
            << provisional.p_IinG.y() << "," << provisional.p_IinG.z() << ","
            << provisional.v_IinG.x() << "," << provisional.v_IinG.y() << ","
            << provisional.v_IinG.z() << "," << provisional.q_GtoI.x() << ","
            << provisional.q_GtoI.y() << "," << provisional.q_GtoI.z() << ","
            << provisional.q_GtoI.w() << "," << provisional.bg.x() << ","
            << provisional.bg.y() << "," << provisional.bg.z() << ","
            << provisional.ba.x() << "," << provisional.ba.y() << ","
            << provisional.ba.z();
        for (int diagonal = 0; diagonal < 15; ++diagonal)
          provisional_navigation_out << ","
                                     << provisional.covariance(diagonal,
                                                               diagonal);
        provisional_navigation_out
            << ",1,0," << provisional.navigation_frame << ","
            << provisional.source << ","
            << provisional.joint_initialization_pending_reason << "\n";
      }
    }
    const bool tracking_frame_fed = do_cam_feed;
    const bool backend_frame_fed =
        sys->get_visual_update_counters().backend_frame_count >
        backend_count_before;
    if (agl_scene_scale_out.is_open()) {
      const AglSceneScaleRuntimeStats &scale =
          sys->get_agl_scene_scale_runtime_stats();
      if (scale.evaluation_count > agl_scene_scale_last_logged_evaluation) {
        const auto &ground = scale.ground;
        const auto &decision = scale.decision;
        agl_scene_scale_out
            << scale.timestamp << "," << scale.mode_name << ","
            << scale.agl_timestamp << "," << scale.agl_m << ","
            << scale.agl_age_s << "," << (ground.valid ? 1 : 0) << ","
            << ground.reason << "," << ground.map_height_m << ","
            << ground.map_height_sigma_m << "," << ground.plane_tilt_deg
            << "," << ground.plane_residual_mad_m << ","
            << ground.plane_residual_rmse_m << ","
            << ground.input_feature_count << ","
            << ground.candidate_feature_count << ","
            << ground.inlier_feature_count << "," << ground.inlier_ratio
            << "," << ground.occupied_image_cells << ","
            << ground.u_span_fraction << "," << ground.v_span_fraction
            << "," << ground.distinct_clone_times << ","
            << decision.coverage_s << "," << decision.max_gap_s << ","
            << decision.median_ratio << "," << decision.mad << ","
            << decision.candidate_scale << "," << decision.proposed_scale
            << "," << decision.cumulative_scale << ","
            << (decision.ready ? 1 : 0) << "," << decision.reason << ","
            << (scale.reset_attempted ? 1 : 0) << ","
            << (scale.reset_succeeded ? 1 : 0) << ","
            << (scale.state_modified ? 1 : 0) << ","
            << scale.applied_scale << "," << scale.reset_reason << ","
            << scale.covariance_min_eigenvalue << ","
            << (scale.covariance_psd_projected ? 1 : 0) << ","
            << scale.covariance_psd_projection_magnitude << ","
            << scale.covariance_psd_projection_limit << ","
            << scale.evaluation_count << "," << scale.valid_ground_count
            << "," << scale.proposal_count << ","
            << scale.reset_attempt_count << ","
            << scale.reset_success_count << "\n";
        agl_scene_scale_last_logged_evaluation = scale.evaluation_count;
      }
    }
    if (dynamic_turn_roi_out.is_open() && do_cam_feed) {
      const DynamicTurnRoiRuntimeStats &roi =
          sys->get_dynamic_turn_roi_runtime_stats();
      dynamic_turn_roi_out
          << t_cam << "," << (roi.decision.signal_valid ? 1 : 0) << ","
          << roi.decision.raw_yaw_rate_radps << ","
          << roi.decision.filtered_yaw_rate_radps << ","
          << turn_direction_name(roi.decision.direction) << ","
          << preferred_image_side_name(roi.decision.preferred_side) << ","
          << roi.decision.strength << ","
          << roi.decision.preferred_fraction << ","
          << roi.decision.detection_exclusion_fraction << ","
          << roi.decision.reason << "," << (tracking_frame_fed ? 1 : 0)
          << "," << (backend_frame_fed ? 1 : 0) << ","
          << roi.detection_masked_pixels << "," << roi.msckf_available
          << "," << roi.msckf_selected_preferred << ","
          << roi.msckf_selected_other << "," << roi.slam_available << ","
          << roi.slam_selected_preferred << "," << roi.slam_selected_other
          << "," << roi.delayed_slam_available << ","
          << roi.delayed_slam_selected_preferred << ","
          << roi.delayed_slam_selected_other << "\n";
    }
    const AdaptiveBackendDecision backend_decision =
        sys->get_last_backend_update_decision();
    const bool backend_decision_evaluated =
        do_cam_feed && args.adaptive_stride &&
        initialized_before_camera_feed;
    const bool backend_reference_committed =
        backend_decision_evaluated &&
        sys->last_backend_reference_committed();
    if (adaptive_stride_out.is_open()) {
      const VisualMotionMetrics &motion = adaptive_input.motion;
      const ContinuousTrackingDecision &tracking =
          adaptive_decision.tracking;
      adaptive_stride_out
          << t_cam << "," << adaptive_decision.mode_name << ","
          << adaptive_decision.reason << ","
          << adaptive_decision.tracking_gap << ","
          << adaptive_decision.instantaneous_safe_gap << ","
          << (adaptive_decision.changed ? 1 : 0) << ","
          << (adaptive_decision.immediate_contraction ? 1 : 0) << ","
          << (adaptive_decision.expansion_confirmed ? 1 : 0) << ","
          << (adaptive_decision.new_motion_measurement ? 1 : 0) << ","
          << adaptive_input.raw_camera_dt_s << ","
          << adaptive_input.actual_tracking_interval_s << ","
          << adaptive_input.motion_measurement_timestamp_s << ","
          << adaptive_decision.motion_age_s << "," << motion.dt_s << ","
          << motion.common_tracks << "," << motion.effective_track_count << ","
          << motion.survival_ratio << "," << motion.median_track_age << ","
          << motion.grid_occupancy_ratio << "," << motion.grid_entropy << ","
          << motion.border_track_ratio << "," << motion.raw_p95_px << ","
          << motion.rotation_p95_px << "," << motion.rotation_max_px << ","
          << motion.compensated_median_px << ","
          << motion.compensated_p95_px << ","
          << motion.compensated_sigma_px << ","
          << motion.compensated_p95_ucb_px << ","
          << tracking.target_horizon_s << "," << tracking.safe_horizon_s
          << "," << tracking.interval_horizon_s << ","
          << tracking.compensated_horizon_s << ","
          << tracking.raw_horizon_s << "," << tracking.rotation_horizon_s
          << "," << tracking.survival_horizon_s << ","
          << tracking.predicted_compensated_median_px << ","
          << tracking.predicted_compensated_p95_ucb_px << ","
          << (adaptive_decision.estimator_protection ? 1 : 0) << ","
          << (adaptive_decision.tracking_protection ? 1 : 0) << ","
          << adaptive_input.active_feature_count << ","
          << adaptive_agl_m << "," << adaptive_agl_source << ","
          << adaptive_agl_age_s << ","
          << (tracking.ground_geometry_valid ? 1 : 0) << ","
          << tracking.ground_overlap_horizon_s << ","
          << tracking.predicted_ground_overlap << ","
          << adaptive_input.visual_residual_p95_px << ","
          << adaptive_input.time_since_accepted_backend_update_s << ","
          << (adaptive_input.covariance_all_finite ? 1 : 0) << ","
          << adaptive_input.covariance_negative_diagonal_count << ","
          << (adaptive_input.visual_state_correction_valid ? 1 : 0) << ","
          << adaptive_input.visual_position_correction_m << ","
          << adaptive_input.visual_velocity_correction_mps << ","
          << adaptive_input.visual_attitude_correction_deg << ","
          << (args.adaptive_stride ? "active" : "shadow") << ","
          << (tracking_frame_fed ? 1 : 0) << ","
          << (backend_frame_fed ? 1 : 0) << ","
          << adaptive_raw_frames_since_tracking << ","
          << adaptive_time_since_last_feed << ","
          << adaptive_time_since_backend << ","
          << (backend_decision_evaluated && backend_decision.trigger ? 1 : 0)
          << "," << (backend_reference_committed ? 1 : 0) << ","
          << (backend_decision_evaluated ? backend_decision.reason
                                         : "not_evaluated")
          << ","
          << (backend_decision_evaluated
                  ? backend_decision.information_score
                  : 0.0)
          << ","
          << (backend_decision_evaluated
                  ? backend_decision.elapsed_since_clone_s
                  : 0.0)
          << ","
          << (backend_decision_evaluated
                  ? backend_decision.minimum_temporal_separation_s
                  : 0.0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.temporal_support_ready
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated
                  ? backend_decision.translation_baseline_px
                  : 0.0)
          << ","
          << (backend_decision_evaluated
                  ? backend_decision.translation_p95_ucb_px
                  : 0.0)
          << ","
          << (backend_decision_evaluated ? backend_decision.effective_tracks
                                         : 0.0)
          << ","
          << (backend_decision_evaluated ? backend_decision.survival_ratio
                                         : 0.0)
          << ","
          << (backend_decision_evaluated ? backend_decision.grid_occupancy
                                         : 0.0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.visual_geometry_valid
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated && backend_decision.pure_rotation
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.information_trigger
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.latency_trigger
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.termination_trigger
                  ? 1
                  : 0)
          << ","
          << (backend_decision_evaluated &&
                      backend_decision.forced_without_visual_information
                  ? 1
                  : 0)
          << "\n";
    }
    if (do_cam_feed) {
      if (online_alignment_mode) {
        OnlineAlignmentResult online_result;
        if (sys->get_online_alignment_result(online_result)) {
          online_result.diagnostics.first_openvins_output_time =
              online_alignment_first_output_time;
          // Online alignment always defines a fixed, causal G_nav boundary.
          // The local-origin mode changes only the estimator translation
          // origin; it does not perform an evaluation-time alignment.
          if (!nav_frame.valid) {
            nav_frame.valid = true;
            nav_frame.dataset_first_imu_timestamp = dataset_first_imu_timestamp;
            nav_frame.requested_start_offset_s = args.start_time;
            nav_frame.trim_boundary_timestamp = trim_boundary_timestamp;
            nav_frame.init_camera_timestamp = online_result.timestamp;
            nav_frame.selected_fc_timestamp =
                online_result.timestamp +
                online_result.camera_to_imu_time_offset_s -
                online_result.fc_to_board_time_offset_s;
            nav_frame.camera_to_imu_time_offset_s =
                online_result.camera_to_imu_time_offset_s;
            nav_frame.fc_time_offset = online_result.fc_to_board_time_offset_s;
            nav_frame.R_Gnav_W0 = Eigen::Matrix3d::Identity();
            nav_frame.p_Gnav_W0 =
                args.online_alignment_local_estimator_origin
                    ? online_result.p_IinG
                    : Eigen::Vector3d::Zero();
            nav_frame.p_Gnav_I0 = online_result.p_IinG;
            nav_frame.p_W0_I0 =
                args.online_alignment_local_estimator_origin
                    ? Eigen::Vector3d::Zero()
                    : online_result.p_IinG;
            nav_frame.fc_source_position = online_result.p_IinG;
            nav_frame.fc_position_frame = "global_gnav";
            nav_frame.position_source =
                args.online_alignment_local_estimator_origin
                    ? "causal_online_fc_board_joint_alignment_local_w0"
                    : "causal_online_fc_board_joint_alignment";
            nav_frame.initialization_mode = "online_multisensor_alignment";
            nav_frame.fc_init_requested_level = "not_applicable";
            nav_frame.fc_init_applied_level = "not_applicable";
            nav_frame.fc_init_selection_method =
                "causal_joint_window_quality_gated";
            nav_frame.fc_init_status = "released_quality_gated";
            nav_frame.fc_init_fallback_applied = false;
            nav_frame.fc_init_future_data_used = false;
            nav_frame.fc_init_available_time =
                online_result.diagnostics.solve_time;
          }
          if (static_cast<int>(online_result.readiness) >
              static_cast<int>(online_alignment_metadata_readiness)) {
            try {
              write_online_alignment_metadata(
                  args.online_alignment_metadata_path, online_result, args);
              online_alignment_metadata_readiness = online_result.readiness;
              PRINT_INFO(GREEN "[ONLINE-ALIGN] %s metadata written: %s\n" RESET,
                         alignment_readiness_name(online_result.readiness),
                         args.online_alignment_metadata_path.c_str());
            } catch (const std::exception &error) {
              PRINT_ERROR(RED "[ONLINE-ALIGN] metadata write failed: %s\n" RESET,
                          error.what());
              return EXIT_FAILURE;
            }
          }
        }
      }
      processed_image_count++;
      processed_image_timestamps.push_back(t_cam);
      if (p5_visual_cadence_active) {
        adaptive_last_fed_camera_time = t_cam;
        adaptive_raw_frames_since_feed = 0;
        adaptive_raw_frames_since_tracking = 0;
      }
      if (backend_frame_fed) {
        adaptive_last_backend_camera_time = t_cam;
      }
      if (args.camera_frame_adaptive)
        camera_adaptive_stats.last_feed_time = t_cam;
    } else {
      skipped_image_count++;
    }
    double dt_ms = 1000.0 * (cv::getTickCount() / cv::getTickFrequency() - t0);

    // ------ query latest state ------
    if (args.dashboard_enabled)
      dash.set_initialized(formal_navigation_ready());
    if (formal_navigation_ready()) {
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

      if (args.post_alignment_fc_yaw_aid && do_cam_feed) {
        if (fc_yaw_aid_next_time < 0.0)
          fc_yaw_aid_next_time =
              t_init_done + args.post_alignment_fc_yaw_aid_period_s;
        if (t_cam + 1e-9 >= fc_yaw_aid_next_time) {
          double fc_attitude_time = std::numeric_limits<double>::quiet_NaN();
          double yaw_measurement_deg = std::numeric_limits<double>::quiet_NaN();
          bool accepted = false;
          std::string decision = "SKIP_NO_ALIGNMENT_RESULT";
          VioManager::PoseAnchorLastUpdate update;
          OnlineAlignmentResult alignment_result;
          if (sys->get_online_alignment_result(alignment_result)) {
            auto current_state = sys->get_state();
            const double state_time = current_state ? current_state->_timestamp : t_cam;
            fc_attitude_time =
                state_time + alignment_result.camera_to_imu_time_offset_s -
                alignment_result.fc_attitude_to_board_time_offset_s;
            if (fc_attitude_time > t_cam + 1e-9) {
              decision = "SKIP_NONCAUSAL_FC_TIME";
            } else {
              try {
                FCInitExactOptions exact_options;
                exact_options.level = FCInitLevel::I1_EXACT_TIME;
                exact_options.max_bracket_gap_s = 0.45;
                const FCInitResult fc_reference =
                    fcinit_select_exact(online_fc_series, fc_attitude_time,
                                        exact_options);
                const Eigen::Matrix3d R_GtoF =
                    ov_core::quat_2_Rot(fc_reference.state.q_GtoI);
                const Eigen::Matrix3d R_GtoI_reference =
                    alignment_result.R_FtoI_nominal * R_GtoF;
                const double yaw_measurement_rad =
                    yaw_from_Rwi(R_GtoI_reference.transpose());
                yaw_measurement_deg = yaw_measurement_rad * 180.0 / M_PI;
                if (current_state && current_state->_imu &&
                    std::fabs(current_state->_timestamp - t_cam) <= 0.10) {
                  accepted = sys->feed_measurement_pose_anchor(
                      current_state->_timestamp, current_state->_imu->pos(),
                      yaw_measurement_rad, true, 1.0e6,
                      args.post_alignment_fc_yaw_aid_sigma_deg * M_PI / 180.0,
                      args.post_alignment_fc_yaw_aid_gate_sigma, 0.0,
                      5.0 * M_PI / 180.0);
                  update = sys->get_last_pose_anchor_update();
                  decision = update.decision;
                } else {
                  decision = "SKIP_STATE_TIME_MISALIGN";
                }
              } catch (const std::exception &error) {
                decision = std::string("SKIP_FC_INTERPOLATION:") + error.what();
              }
            }
          }
          if (fc_yaw_aid_out.is_open()) {
            fc_yaw_aid_out
                << t_cam << "," << fc_attitude_time << ","
                << yaw_measurement_deg << "," << update.yaw_residual_deg
                << "," << update.nis << "," << update.gate_ratio << ","
                << update.predicted_yaw_correction_deg << ","
                << (accepted ? 1 : 0) << "," << decision << "\n";
          }
          fc_yaw_aid_next_time =
              t_cam + args.post_alignment_fc_yaw_aid_period_s;
        }
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
          if (args.dashboard_enabled && gd.t > 0) {
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
      if (t_cam > state->_timestamp + 1e-9) {
        Eigen::Matrix<double, 13, 1> propagated_state;
        Eigen::Matrix<double, 12, 12> propagated_covariance;
        if (sys->get_propagator()->fast_state_propagate(
                state, t_cam, propagated_state, propagated_covariance)) {
          const Eigen::Matrix3d R_GtoI_fast =
              ov_core::quat_2_Rot(propagated_state.head<4>());
          R_wi = R_GtoI_fast.transpose();
          p_wi = propagated_state.block<3, 1>(4, 0);
          // fast_state_propagate exposes velocity in the current IMU frame.
          v_wi = R_wi * propagated_state.block<3, 1>(7, 0);
        }
      }

      if (args.dashboard_enabled) {
        vio_for_align.push_back({t_cam, p_wi});
        while (vio_for_align.size() > 5000)
          vio_for_align.pop_front();
        maybe_align(t_cam);
      }

      // output TUM (unaligned, VIO frame)
      Eigen::Matrix3d Rwi = R_wi;
      Eigen::Quaterniond q(Rwi);
      if (nav_frame.valid && !nav_frame.metadata_written) {
        if (online_alignment_mode)
          online_alignment_first_output_time = t_cam;
        nav_frame.first_emitted_timestamp = t_cam;
        nav_frame.p_W0_Ifirst = p_wi;
        nav_frame.v_W0_Ifirst = v_wi;
        nav_frame.q_ItoW0_first << q.x(), q.y(), q.z(), q.w();
        write_nav_metadata(args.nav_frame_metadata_path, nav_frame, args);
        nav_frame.metadata_written = true;
      }
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
      // Reset the float format for every row because the bias fields below use
      // scientific notation.  Keeping t_cam at the same fixed precision as the
      // raw/nav trajectories preserves their row-wise timestamp invariant.
      debug_out << std::fixed << std::setprecision(9)
                << t_cam << ' ' << v_wi.x() << ' ' << v_wi.y() << ' ' << v_wi.z() << ' '
                << std::scientific << std::setprecision(9)
                << bg.x() << ' ' << bg.y() << ' ' << bg.z() << ' '
                << ba.x() << ' ' << ba.y() << ' ' << ba.z() << '\n';

      // backend diagnostic: MSCKF chi2 rejection stats
      auto mstats = sys->get_last_msckf_stats();
      if (args.dashboard_enabled)
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
      if (args.dashboard_enabled) {
        dash.update_vio_pose(t_cam, R_wi, p_wi, v_wi);
        dash.update_biases(t_cam, bg, ba); // bg/ba read from state above; always from live EKF
      }
      std::vector<Eigen::Vector3d> slam_pts = sys->get_features_SLAM();
      std::vector<Eigen::Vector3d> msckf_pts = sys->get_good_features_MSCKF();
      if (args.dashboard_enabled)
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
      if (args.dashboard_enabled && !gt_pos_map.empty()) {
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
      if (args.dashboard_enabled && !gps_pos_map.empty()) {
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
        prev_p_wi_valid = false;
        vio_for_align.clear();
        if (did_gps_init)
          t_init_done = t_cam;
        else
          t_init_done = -1.0;
      }
    }

    // Image preparation is dashboard/video-only and must not tax deployment
    // replay when the dashboard is explicitly disabled.
    cv::Mat hist;
    if (args.dashboard_enabled || !args.video_cam_path.empty())
      hist = sys->get_historical_viz_image();
    if (args.dashboard_enabled) {
      if (!hist.empty())
        dash.update_image(t_cam, hist);
      else
        dash.update_image(t_cam, img0);
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
    if (args.dashboard_enabled &&
        frame_idx % std::max(1, args.dash_every) == 0) {
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
      audit << "configured_stride,post_alignment_configured_stride,"
            << "total_image_input_count,processed_image_count,"
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
            << "p5_backend_frame_count,p5_tracking_only_frame_count,"
            << "p5_tracking_only_observation_drop_count,p5_tracking_only_clone_violation_count,"
            << "p5_backend_trigger_count,p5_backend_clone_commit_count,"
            << "p5_backend_trigger_without_clone_count,p5_backend_bootstrap_trigger_count,"
            << "p5_backend_information_trigger_count,p5_backend_latency_fallback_count,"
            << "p5_backend_termination_trigger_count,"
            << "p5_backend_pure_rotation_hold_count,p5_tracking_gap_change_count,"
            << "p5_immediate_contraction_count,"
            << "processed_image_timestamps\n";
      audit << args.cam_subsample << ","
            << args.post_alignment_camera_frame_stride << ","
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
            << camera_adaptive_stats.last_reason << ","
            << visual_counts.backend_frame_count << ","
            << visual_counts.tracking_only_frame_count << ","
            << visual_counts.tracking_only_observation_drop_count << ","
            << visual_counts.tracking_only_clone_violation_count << ","
            << visual_counts.backend_trigger_count << ","
            << visual_counts.backend_clone_commit_count << ","
            << visual_counts.backend_trigger_without_clone_count << ","
            << visual_counts.backend_bootstrap_trigger_count << ","
            << visual_counts.backend_information_trigger_count << ","
            << visual_counts.backend_latency_fallback_count << ","
            << visual_counts.backend_termination_trigger_count << ","
            << visual_counts.backend_pure_rotation_hold_count << ","
            << adaptive_tracking_stride_change_count << ","
            << adaptive_emergency_downshift_count << ","
            << "\"";
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
  if (agl_scene_scale_out.is_open()) {
    agl_scene_scale_out.flush();
    if (!agl_scene_scale_out.good()) {
      PRINT_ERROR(RED "[AGL-SCALE] diagnostic CSV write failed: %s\n" RESET,
                  args.agl_scene_scale_diag_path.c_str());
      return EXIT_FAILURE;
    }
    agl_scene_scale_out.close();
    const auto &scale = sys->get_agl_scene_scale_runtime_stats();
    PRINT_INFO(GREEN "[AGL-SCALE] log=%s evaluations=%zu valid_ground=%zu "
                     "proposals=%zu reset_attempts=%zu reset_success=%zu\n" RESET,
               args.agl_scene_scale_diag_path.c_str(), scale.evaluation_count,
               scale.valid_ground_count, scale.proposal_count,
               scale.reset_attempt_count, scale.reset_success_count);
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
               adaptive_visual_scheduler.tracking_gap());
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

  if (online_alignment_mode) {
    const auto &alignment_receipts = sys->online_alignment_attempt_receipts();
    try {
      write_online_alignment_attempt_receipts(
          sibling_path(args.output_path,
                       "online_alignment_attempt_receipts.json"),
          alignment_receipts,
          sys->online_alignment_candidate());
      write_online_alignment_sliding_window_trace(
          args.online_alignment_window_trace_path, alignment_receipts);
    } catch (const std::exception &error) {
      PRINT_ERROR(RED "[ONLINE-ALIGN] attempt receipt write failed: %s\n" RESET,
                  error.what());
      return EXIT_FAILURE;
    }
    OnlineAlignmentResult final_online_result;
    if (sys->get_online_alignment_result(final_online_result)) {
      final_online_result.diagnostics.first_openvins_output_time =
          online_alignment_first_output_time;
      try {
        write_online_alignment_metadata(args.online_alignment_metadata_path,
                                        final_online_result, args);
      } catch (const std::exception &error) {
        PRINT_ERROR(RED "[ONLINE-ALIGN] final metadata write failed: %s\n" RESET,
                    error.what());
        return EXIT_FAILURE;
      }
    } else {
      try {
        write_online_alignment_failure_metadata(
            args.online_alignment_metadata_path,
            sys->online_alignment_diagnostics(),
            sys->online_alignment_rejection(), args);
      } catch (const std::exception &error) {
        PRINT_ERROR(RED "[ONLINE-ALIGN] failure metadata write failed: %s\n" RESET,
                    error.what());
      }
      PRINT_ERROR(RED "[ONLINE-ALIGN] dataset ended without a navigation-ready release: %s; metadata=%s\n" RESET,
                  sys->online_alignment_rejection().c_str(),
                  args.online_alignment_metadata_path.c_str());
      return EXIT_FAILURE;
    }
  }
  // Final frame only when the dashboard is part of this run.
  if (args.dashboard_enabled)
    dash.render_and_show(1);
  return EXIT_SUCCESS;
}
