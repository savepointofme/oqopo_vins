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

#ifndef OV_MSCKF_ROS_FREE_DATASET_READER_EUROC_H
#define OV_MSCKF_ROS_FREE_DATASET_READER_EUROC_H

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "utils/colors.h"
#include "utils/print.h"

namespace ov_msckf {

/**
 * @brief ASL / EuRoC format dataset reader (ROS-free).
 *
 * [中文] 读 EuRoC / ASL 格式离线数据集, 供 ROS-free 主程序按时间轴顺序回放:
 *  - imu0/data.csv : timestamp[ns], wx, wy, wz, ax, ay, az
 *  - cam{i}/data.csv : timestamp[ns], filename
 *  - cam{i}/data/*.png : 图像文件
 *  - (可选) state_groundtruth_estimate0/data.csv : 17 列 ASL ground truth
 *  - (可选) gps.csv : timestamp[ns], x, y, z  或  timestamp[ns], lat, lon, alt
 *
 * 设计原则: 只依赖 Boost.Filesystem + 标准库, 完全不依赖 ROS。
 */
class DatasetReaderEuroc {
public:
  struct ImuSample {
    double timestamp;                     ///< [中文] 秒 (由 ns/1e9 转换)
    Eigen::Vector3d gyro;                 ///< [中文] 角速度 (rad/s)
    Eigen::Vector3d accel;                ///< [中文] 线加速度 (m/s^2)
  };

  struct CamEntry {
    double timestamp;                     ///< [中文] 秒
    std::string image_path;               ///< [中文] 图像绝对路径
  };

  struct GtSample {
    double timestamp;
    Eigen::Vector3d p;                    ///< [中文] 位置 (ENU / world)
    Eigen::Vector4d q_wxyz;               ///< [中文] 姿态四元数 (w,x,y,z)
    Eigen::Vector3d v;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
  };

  struct GpsSample {
    double timestamp;
    Eigen::Vector3d xyz;                  ///< [中文] ENU 或经纬高转换后的 xyz (米)
  };

  /**
   * @brief Load IMU CSV. Sorted ascending by timestamp.
   */
  static bool load_imu(const std::string &path, std::vector<ImuSample> &out) {
    out.clear();
    std::ifstream file(path);
    if (!file.is_open()) {
      PRINT_ERROR(RED "[euroc] failed to open imu file: %s\n" RESET, path.c_str());
      return false;
    }
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::stringstream ss(line);
      std::array<double, 7> v{};
      std::string tok;
      int idx = 0;
      while (std::getline(ss, tok, ',') && idx < 7) {
        v[idx++] = std::atof(tok.c_str());
      }
      if (idx < 7)
        continue;
      ImuSample s;
      s.timestamp = 1e-9 * v[0];
      s.gyro << v[1], v[2], v[3];
      s.accel << v[4], v[5], v[6];
      out.push_back(s);
    }
    std::sort(out.begin(), out.end(),
              [](const ImuSample &a, const ImuSample &b) { return a.timestamp < b.timestamp; });
    PRINT_INFO(GREEN "[euroc] loaded %zu IMU samples from %s\n" RESET, out.size(), path.c_str());
    return !out.empty();
  }

  /**
   * @brief Load camera timestamp list + resolve image paths.
   */
  static bool load_cam(const std::string &cam_dir, std::vector<CamEntry> &out) {
    out.clear();
    namespace fs = boost::filesystem;
    fs::path csv = fs::path(cam_dir) / "data.csv";
    fs::path data_dir = fs::path(cam_dir) / "data";
    std::ifstream file(csv.string());
    if (!file.is_open()) {
      PRINT_ERROR(RED "[euroc] failed to open cam csv: %s\n" RESET, csv.string().c_str());
      return false;
    }
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::stringstream ss(line);
      std::string ts_str, fname;
      std::getline(ss, ts_str, ',');
      std::getline(ss, fname, ',');
      // trim trailing \r / whitespace
      while (!fname.empty() && (fname.back() == '\r' || fname.back() == ' ' || fname.back() == '\t'))
        fname.pop_back();
      if (ts_str.empty() || fname.empty())
        continue;
      CamEntry e;
      e.timestamp = 1e-9 * std::atof(ts_str.c_str());
      e.image_path = (data_dir / fname).string();
      out.push_back(e);
    }
    std::sort(out.begin(), out.end(),
              [](const CamEntry &a, const CamEntry &b) { return a.timestamp < b.timestamp; });
    PRINT_INFO(GREEN "[euroc] loaded %zu camera entries from %s\n" RESET, out.size(), csv.string().c_str());
    return !out.empty();
  }

  /**
   * @brief Load ASL groundtruth file with 17 columns.
   *
   * Columns: ts[ns], px, py, pz, qw, qx, qy, qz, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz
   */
  static bool load_gt(const std::string &path, std::vector<GtSample> &out) {
    out.clear();
    std::ifstream file(path);
    if (!file.is_open()) {
      PRINT_WARNING(YELLOW "[euroc] no groundtruth found at: %s (skipping)\n" RESET, path.c_str());
      return false;
    }
    std::string line;
    std::getline(file, line); // header
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::stringstream ss(line);
      std::array<double, 17> v{};
      std::string tok;
      int idx = 0;
      while (std::getline(ss, tok, ',') && idx < 17) {
        v[idx++] = std::atof(tok.c_str());
      }
      if (idx < 8)
        continue;
      GtSample s;
      s.timestamp = 1e-9 * v[0];
      s.p << v[1], v[2], v[3];
      s.q_wxyz << v[4], v[5], v[6], v[7];
      if (idx >= 11)
        s.v << v[8], v[9], v[10];
      else
        s.v.setZero();
      if (idx >= 14)
        s.bg << v[11], v[12], v[13];
      else
        s.bg.setZero();
      if (idx >= 17)
        s.ba << v[14], v[15], v[16];
      else
        s.ba.setZero();
      out.push_back(s);
    }
    std::sort(out.begin(), out.end(),
              [](const GtSample &a, const GtSample &b) { return a.timestamp < b.timestamp; });
    PRINT_INFO(GREEN "[euroc] loaded %zu GT samples from %s\n" RESET, out.size(), path.c_str());
    return !out.empty();
  }

  /**
   * @brief Load a GPS-like CSV.
   *
   * Auto-detects whether the 2..4 columns are ENU xyz (meters) or WGS84 lat,lon,alt
   * based on magnitude. If absolute value of col1 <= 90 and col2 <= 180 and this
   * looks like angles, we convert WGS84 -> ENU (local tangent plane at first sample).
   */
  static bool load_gps(const std::string &path, std::vector<GpsSample> &out) {
    out.clear();
    std::ifstream file(path);
    if (!file.is_open()) {
      PRINT_WARNING(YELLOW "[euroc] no GPS found at: %s (skipping)\n" RESET, path.c_str());
      return false;
    }
    std::string line;
    std::getline(file, line); // header
    std::vector<std::array<double, 4>> rows;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#')
        continue;
      std::stringstream ss(line);
      std::array<double, 4> v{};
      std::string tok;
      int idx = 0;
      while (std::getline(ss, tok, ',') && idx < 4) {
        v[idx++] = std::atof(tok.c_str());
      }
      if (idx < 4)
        continue;
      rows.push_back(v);
    }
    if (rows.empty())
      return false;
    // [中文] 如果看起来是经纬高, 做 WGS84 -> ENU 局部切平面转换, 以第一条为原点
    bool looks_wgs = std::fabs(rows[0][1]) <= 90.0 && std::fabs(rows[0][2]) <= 180.0 &&
                     std::fabs(rows[0][3]) < 100000.0 && std::fabs(rows[0][1]) > 1e-3;
    Eigen::Vector3d origin_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_ecef_to_enu = Eigen::Matrix3d::Identity();
    if (looks_wgs) {
      wgs84_to_ecef(rows[0][1], rows[0][2], rows[0][3], origin_ecef);
      ecef_to_enu_rotation(rows[0][1], rows[0][2], R_ecef_to_enu);
      PRINT_INFO(GREEN "[euroc] GPS detected WGS84, anchoring ENU at (%.7f, %.7f, %.3f)\n" RESET,
                 rows[0][1], rows[0][2], rows[0][3]);
    }
    for (const auto &r : rows) {
      GpsSample s;
      s.timestamp = 1e-9 * r[0];
      if (looks_wgs) {
        Eigen::Vector3d ecef;
        wgs84_to_ecef(r[1], r[2], r[3], ecef);
        s.xyz = R_ecef_to_enu * (ecef - origin_ecef);
      } else {
        s.xyz << r[1], r[2], r[3];
      }
      out.push_back(s);
    }
    std::sort(out.begin(), out.end(),
              [](const GpsSample &a, const GpsSample &b) { return a.timestamp < b.timestamp; });
    PRINT_INFO(GREEN "[euroc] loaded %zu GPS samples from %s\n" RESET, out.size(), path.c_str());
    return !out.empty();
  }

private:
  static void wgs84_to_ecef(double lat_deg, double lon_deg, double alt, Eigen::Vector3d &ecef) {
    const double a = 6378137.0;
    const double e2 = 6.69437999014e-3;
    const double lat = lat_deg * M_PI / 180.0;
    const double lon = lon_deg * M_PI / 180.0;
    double s = std::sin(lat);
    double N = a / std::sqrt(1 - e2 * s * s);
    ecef.x() = (N + alt) * std::cos(lat) * std::cos(lon);
    ecef.y() = (N + alt) * std::cos(lat) * std::sin(lon);
    ecef.z() = (N * (1 - e2) + alt) * s;
  }

  static void ecef_to_enu_rotation(double lat_deg, double lon_deg, Eigen::Matrix3d &R) {
    const double lat = lat_deg * M_PI / 180.0;
    const double lon = lon_deg * M_PI / 180.0;
    double sl = std::sin(lat), cl = std::cos(lat);
    double so = std::sin(lon), co = std::cos(lon);
    R << -so, co, 0, -sl * co, -sl * so, cl, cl * co, cl * so, sl;
  }
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_DATASET_READER_EUROC_H
