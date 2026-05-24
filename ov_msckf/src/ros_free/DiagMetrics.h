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

// DiagMetrics.h — plain data struct that carries all per-frame diagnostic
// metrics from the runner to the printer and logger.  No logic here.

#ifndef OV_MSCKF_ROS_FREE_DIAG_METRICS_H
#define OV_MSCKF_ROS_FREE_DIAG_METRICS_H

#include <string>

namespace ov_msckf {

struct DiagMetrics {
  // ---- identification ----
  double t = -1;           // camera frame timestamp (s)
  int    frame_id = 0;     // 1-based camera frame counter
  std::string run_name;    // arbitrary label set by --run-name
  std::string mode_str;    // active feature flags, e.g. "FC+gpsAlt"
  std::string init_source; // "DynInit" or "FC"

  // ---- filter status ----
  bool initialized = false;
  bool aligned     = false;

  // ---- feature tracking ----
  int klt_raw   = 0;  // KLT tracks attempted before RANSAC
  int tracked   = 0;  // KLT tracks kept after RANSAC
  int n_acc     = 0;  // MSCKF features accepted this frame
  int msckf_in  = 0;  // MSCKF features fed in (before triangulation/chi2)
  int slam_count = 0; // active SLAM features

  // ---- first-event timestamps ----
  double first_accept_t = -1;
  double first_slam_t   = -1;

  // ---- speed and scale ----
  double vio_speed   = 0;   // |v_wi| (m/s)
  double gps_speed   = -1;  // finite-diff GPS speed (m/s); -1 = no GPS yet
  double vio_path_len = 0;  // cumulative VIO odometry (m)
  double gps_path_len = 0;  // cumulative GPS path (m)

  // ---- backend ----
  int    chi2_acc   = 0;   // MSCKF features accepted (= n_acc, named for clarity)
  int    chi2_rej   = 0;   // MSCKF features rejected by chi2 test
  double depth_med  = -1;  // median dist VIO pos → SLAM features (m); -1 = no SLAM
  double depth_max  = -1;  // max    dist VIO pos → SLAM features (m)

  // ---- IMU state ----
  double ba_norm = 0;  // ||accel bias||  (m/s²)
  double bg_norm = 0;  // ||gyro  bias||  (rad/s)
  double cam_toff = 0; // camera-IMU time offset timeshift_cam_imu (s)
  double vio_dist = 0; // alias of vio_path_len; kept for CSV naming clarity

  // ---- raw pose / velocity / bias vectors (for CSV) ----
  double vio_x = 0, vio_y = 0, vio_z = 0;        // position (m)
  double vio_vx = 0, vio_vy = 0, vio_vz = 0;     // velocity (m/s)
  double gps_x = 0, gps_y = 0, gps_z = 0;        // latest GPS ENU (m)
  double bgx = 0, bgy = 0, bgz = 0;               // gyro bias (rad/s)
  double bax = 0, bay = 0, baz = 0;               // accel bias (m/s²)

  // ---- camera calibration (cam 0) ----
  double fx = 0, fy = 0;   // focal lengths (px)
  double cx = 0, cy = 0;   // principal point (px)
  double ext_tx = 0, ext_ty = 0, ext_tz = 0;  // IMU→cam translation (m)
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_DIAG_METRICS_H
