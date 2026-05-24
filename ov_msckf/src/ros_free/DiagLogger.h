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

// DiagLogger — writes per-frame diagnostic CSV and an append-only human-readable
// event log.  Both sinks are optional; pass an empty path to disable.
//
// ============================================================================
// CSV schema
// ============================================================================
// Every column is documented below.  "-1" is the sentinel for "not available"
// (e.g. no GPS, no SLAM features, filter not yet initialized).
//
//  Column          | Meaning                                | Unit   | Freq        | Source
//  --------------- | -------------------------------------- | ------ | ----------- | ------
//  t               | Camera frame timestamp                 | s      | per frame   | cam0/data.csv
//  frame_id        | 1-based camera frame index             | -      | per frame   | runner counter
//  initialized     | Filter initialized (0/1)               | bool   | per frame   | VioManager::initialized()
//  vio_x           | IMU position X in VIO world frame      | m      | per frame   | State::_imu->pos()
//  vio_y           | IMU position Y in VIO world frame      | m      | per frame   | State::_imu->pos()
//  vio_z           | IMU position Z in VIO world frame      | m      | per frame   | State::_imu->pos()
//  vio_vx          | IMU velocity X in VIO world frame      | m/s    | per frame   | State::_imu->vel()
//  vio_vy          | IMU velocity Y in VIO world frame      | m/s    | per frame   | State::_imu->vel()
//  vio_vz          | IMU velocity Z in VIO world frame      | m/s    | per frame   | State::_imu->vel()
//  vio_speed       | ||v_wi||                               | m/s    | per frame   | computed
//  gps_x           | Latest GPS ENU X (same frame as input) | m      | ~GPS rate   | GPS CSV (last known)
//  gps_y           | Latest GPS ENU Y                       | m      | ~GPS rate   | GPS CSV (last known)
//  gps_z           | Latest GPS ENU Z                       | m      | ~GPS rate   | GPS CSV (last known)
//  gps_speed       | GPS speed (finite diff of ENU pos)     | m/s    | ~GPS rate   | computed; -1=no GPS
//  speed_ratio     | vio_speed / gps_speed                  | -      | ~GPS rate   | computed; -1=no GPS
//  klt_raw         | KLT tracks attempted before RANSAC     | count  | per frame   | TrackerWarpVizPacket::n_klt_attempted
//  tracked         | KLT tracks kept after RANSAC           | count  | per frame   | TrackerWarpVizPacket::curr_pts_raw.size()
//  n_acc           | MSCKF features accepted this frame     | count  | per frame   | MsckfLastStats::n_accepted
//  msckf_in        | MSCKF features fed in (pre-tri/chi2)   | count  | per frame   | MsckfLastStats::n_features_in
//  slam_count      | Active SLAM features in state          | count  | per frame   | get_features_SLAM().size()
//  chi2_rej        | MSCKF features rejected by chi2 gate  | count  | per frame   | MsckfLastStats::n_chi2_rejected
//  chi2_acc        | MSCKF features accepted (=n_acc)       | count  | per frame   | MsckfLastStats::n_accepted
//  depth_med       | Median L2 VIO-pos → SLAM feature       | m      | per frame   | computed; -1=no SLAM
//  depth_max       | Max    L2 VIO-pos → SLAM feature       | m      | per frame   | computed; -1=no SLAM
//  bgx             | Gyro bias X in body frame              | rad/s  | per frame   | State::_imu->bias_g()
//  bgy             | Gyro bias Y in body frame              | rad/s  | per frame   | State::_imu->bias_g()
//  bgz             | Gyro bias Z in body frame              | rad/s  | per frame   | State::_imu->bias_g()
//  bax             | Accel bias X in body frame             | m/s²   | per frame   | State::_imu->bias_a()
//  bay             | Accel bias Y in body frame             | m/s²   | per frame   | State::_imu->bias_a()
//  baz             | Accel bias Z in body frame             | m/s²   | per frame   | State::_imu->bias_a()
//  ba_norm         | ||accel bias||                         | m/s²   | per frame   | computed
//  bg_norm         | ||gyro  bias||                         | rad/s  | per frame   | computed
//  cam_toff        | Camera-IMU time offset (timeshift)     | s      | per frame   | State::_calib_dt_CAMtoIMU
//  fx              | Focal length X, cam 0                  | px     | per frame   | State::_cam_intrinsics[0]
//  fy              | Focal length Y, cam 0                  | px     | per frame   | State::_cam_intrinsics[0]
//  cx              | Principal point X, cam 0               | px     | per frame   | State::_cam_intrinsics[0]
//  cy              | Principal point Y, cam 0               | px     | per frame   | State::_cam_intrinsics[0]
//  vio_dist        | Cumulative VIO odometry distance       | m      | per frame   | accumulated sum of ||dp||
//  gps_dist        | Cumulative GPS path length             | m      | ~GPS rate   | accumulated sum of GPS ||dp||
// ============================================================================

#ifndef OV_MSCKF_ROS_FREE_DIAG_LOGGER_H
#define OV_MSCKF_ROS_FREE_DIAG_LOGGER_H

#include "DiagMetrics.h"
#include <fstream>
#include <string>

namespace ov_msckf {

class DiagLogger {
public:
  enum class EventType {
    INIT,
    FIRST_ACCEPT,
    FIRST_SLAM,
    CHI2_SPIKE,
    SLAM_DROP,
    SCALE_COLLAPSE,
    DIVERGENCE,
    LANDING_OR_STOP,
    OTHER
  };

  // Pass empty strings to disable the corresponding sink.
  DiagLogger(const std::string &csv_path, const std::string &event_path);

  // Write one CSV row from a DiagMetrics snapshot.
  void write_row(const DiagMetrics &m);

  // Append one event entry to the event log.
  //   type    — event classification
  //   m       — metric snapshot at the time of the event (for context fields)
  //   reason  — short human-readable description of why the event fired
  //   detail  — optional extra key=value pairs (may contain newlines)
  void log_event(EventType type, const DiagMetrics &m,
                 const std::string &reason,
                 const std::string &detail = "");

  bool csv_ok()   const { return csv_.is_open(); }
  bool event_ok() const { return event_.is_open(); }

private:
  std::ofstream csv_;
  std::ofstream event_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_DIAG_LOGGER_H
