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

#include "DiagLogger.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <iomanip>

#include "utils/colors.h"
#include "utils/print.h"

namespace ov_msckf {

static const char *event_tag(DiagLogger::EventType t) {
  switch (t) {
  case DiagLogger::EventType::INIT:            return "EVENT_INIT";
  case DiagLogger::EventType::FIRST_ACCEPT:    return "EVENT_FIRST_ACCEPT";
  case DiagLogger::EventType::FIRST_SLAM:      return "EVENT_FIRST_SLAM";
  case DiagLogger::EventType::CHI2_SPIKE:      return "EVENT_CHI2_SPIKE";
  case DiagLogger::EventType::SLAM_DROP:       return "EVENT_SLAM_DROP";
  case DiagLogger::EventType::SCALE_COLLAPSE:  return "EVENT_SCALE_COLLAPSE";
  case DiagLogger::EventType::DIVERGENCE:      return "EVENT_DIVERGENCE";
  case DiagLogger::EventType::LANDING_OR_STOP: return "EVENT_LANDING_OR_STOP";
  default:                                     return "EVENT_OTHER";
  }
}

static std::string wall_timestamp() {
  auto now = std::chrono::system_clock::now();
  auto tt  = std::chrono::system_clock::to_time_t(now);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&tt));
  return buf;
}

DiagLogger::DiagLogger(const std::string &csv_path, const std::string &event_path) {
  if (!csv_path.empty()) {
    csv_.open(csv_path);
    if (csv_.is_open()) {
      // Header matches the column order in write_row() below.
      csv_ << "t,frame_id,initialized,"
              "vio_x,vio_y,vio_z,"
              "vio_vx,vio_vy,vio_vz,vio_speed,"
              "gps_x,gps_y,gps_z,gps_speed,speed_ratio,"
              "klt_raw,tracked,n_acc,msckf_in,slam_count,"
              "chi2_rej,chi2_acc,"
              "depth_med,depth_max,"
              "bgx,bgy,bgz,"
              "bax,bay,baz,"
              "ba_norm,bg_norm,"
              "cam_toff,"
              "fx,fy,cx,cy,"
              "vio_dist,gps_dist,"
              "desc_detected,desc_pre_gate,desc_post_gate,desc_tracked\n";
      PRINT_INFO(GREEN "[diag-csv] writing to %s\n" RESET, csv_path.c_str());
    } else {
      PRINT_WARNING(YELLOW "[diag-csv] failed to open %s\n" RESET, csv_path.c_str());
    }
  }
  if (!event_path.empty()) {
    event_.open(event_path, std::ios::app);
    if (event_.is_open()) {
      event_ << "# ---- session start " << wall_timestamp() << " ----\n\n";
      event_.flush();
      PRINT_INFO(GREEN "[diag-events] writing to %s\n" RESET, event_path.c_str());
    } else {
      PRINT_WARNING(YELLOW "[diag-events] failed to open %s\n" RESET, event_path.c_str());
    }
  }
}

void DiagLogger::write_row(const DiagMetrics &m) {
  if (!csv_.is_open()) return;
  double speed_ratio = (m.gps_speed > 0.01) ? m.vio_speed / m.gps_speed : -1.0;
  csv_ << std::fixed << std::setprecision(6)
       << m.t          << ',' << m.frame_id << ',' << (int)m.initialized << ','
       << m.vio_x      << ',' << m.vio_y    << ',' << m.vio_z << ','
       << m.vio_vx     << ',' << m.vio_vy   << ',' << m.vio_vz << ',' << m.vio_speed << ','
       << m.gps_x      << ',' << m.gps_y    << ',' << m.gps_z << ','
       << m.gps_speed  << ',' << speed_ratio << ','
       << m.klt_raw    << ',' << m.tracked   << ',' << m.n_acc << ','
       << m.msckf_in   << ',' << m.slam_count << ','
       << m.chi2_rej   << ',' << m.chi2_acc  << ','
       << m.depth_med  << ',' << m.depth_max << ','
       << m.bgx << ',' << m.bgy << ',' << m.bgz << ','
       << m.bax << ',' << m.bay << ',' << m.baz << ','
       << m.ba_norm    << ',' << m.bg_norm   << ','
       << m.cam_toff   << ','
       << m.fx  << ',' << m.fy  << ',' << m.cx  << ',' << m.cy << ','
       << m.vio_dist   << ',' << m.gps_path_len << ','
       << m.desc_detected << ',' << m.desc_pre_gate << ','
       << m.desc_post_gate << ',' << m.desc_tracked << '\n';
}

void DiagLogger::log_event(EventType type, const DiagMetrics &m,
                           const std::string &reason,
                           const std::string &detail) {
  if (!event_.is_open()) return;

  // Header line: type, sensor time, frame, wall time
  event_ << "[" << event_tag(type) << "]"
         << " t=" << std::fixed << std::setprecision(3) << m.t
         << " frame=" << m.frame_id
         << " reason=\"" << reason << "\""
         << " wall=" << wall_timestamp() << "\n";

  // Standard metric snapshot (always included)
  double speed_ratio = (m.gps_speed > 0.01) ? m.vio_speed / m.gps_speed : -1.0;
  event_ << std::fixed << std::setprecision(2)
         << "  chi2_acc=" << m.chi2_acc << " chi2_rej=" << m.chi2_rej
         << " klt=" << m.klt_raw << " track=" << m.tracked
         << " msckf=" << m.msckf_in << " slam=" << m.slam_count << "\n";
  event_ << "  gps_v=" << m.gps_speed
         << " vio_v=" << m.vio_speed
         << " scale_ratio=" << speed_ratio
         << " depth_med=" << m.depth_med << "\n";

  // Optional caller-supplied detail lines
  if (!detail.empty())
    event_ << "  " << detail << "\n";

  event_ << "\n";
  event_.flush();
}

} // namespace ov_msckf
