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
#include <atomic>
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
#include "state/Propagator.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include "ros_free/DatasetReaderEuroc.h"
#include "ros_free/TrajectoryAligner.h"
#include "ros_free/VizDashboard.h"

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
  std::string video_path;
  double align_seconds = 8.0;    // [中文] 初始化完成后收集多少秒数据再做 SE3 对齐
  bool stereo = false;           // [中文] 使用 cam1 配对
  bool show = true;              // [中文] 显示窗口
  int dash_every = 1;            // [中文] 每 N 帧刷新仪表板
  bool verbose_timing = false;
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
               "  --gt PATH             ASL 17-col ground truth CSV\n"
               "  --output PATH         Output TUM trajectory (default: traj_ros_free.txt)\n"
               "  --video PATH          Record dashboard to MP4\n"
               "  --align-seconds X     Seconds of data to collect before SE3 align (default 8)\n"
               "  --no-display          Do not create a window (useful headless)\n"
               "  --dash-every N        Refresh dashboard every N camera frames (default 1)\n"
               "  --verbose             Print per-frame timing\n";
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
    else if (s == "--align-seconds") a.align_seconds = std::atof(next("--align-seconds").c_str());
    else if (s == "--stereo") a.stereo = true;
    else if (s == "--no-display") a.show = false;
    else if (s == "--dash-every") a.dash_every = std::atoi(next("--dash-every").c_str());
    else if (s == "--verbose") a.verbose_timing = true;
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

  // -------------------- load config --------------------
  auto parser = std::make_shared<ov_core::YamlParser>(args.config_path);
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  VioManagerOptions params;
  params.print_and_load(parser);
  // [中文] 离线回放: 禁用 VioManager 内部的异步队列, 我们自己严格按时间序喂
  params.use_multi_threading_subs = false;

  if (!parser->successful()) {
    PRINT_ERROR(RED "[ros-free] failed to parse config %s\n" RESET, args.config_path.c_str());
    return EXIT_FAILURE;
  }
  auto sys = std::make_shared<VioManager>(params);

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

  // GT map keyed by time (VIO world = GT world approximately after align)
  std::map<double, Eigen::Vector3d> gt_pos_map;
  for (const auto &s : gt)
    gt_pos_map[s.timestamp] = s.p;
  std::map<double, Eigen::Vector3d> gps_pos_map;
  for (const auto &s : gps)
    gps_pos_map[s.timestamp] = s.xyz;

  // -------------------- dashboard --------------------
  VizDashboard::Options vo;
  vo.show_window = args.show;
  vo.video_path = args.video_path;
  VizDashboard dash(vo);
  TrajectoryAligner aligner;

  // -------------------- output file --------------------
  std::ofstream out(args.output_path);
  if (!out.is_open()) {
    PRINT_ERROR(RED "[ros-free] cannot open output: %s\n" RESET, args.output_path.c_str());
    return EXIT_FAILURE;
  }
  out << "# TUM traj (t x y z qx qy qz qw) in VIO (unaligned) frame\n";
  out << std::fixed << std::setprecision(9);

  // -------------------- timeline merge-sort --------------------
  // [中文] OpenVINS 滤波器完全由传感器时间戳驱动。我们把 IMU 样本和相机帧
  // 按时间戳合并排序后顺序喂入, 与 ROS 版本结果数值一致 (仅受初始化线程
  // 非确定性影响)。
  size_t imu_i = 0, cam_i = 0, cam1_i = 0;
  const double INF = std::numeric_limits<double>::infinity();
  double t_init_done = -1; // [中文] 滤波器完成初始化的时刻
  std::deque<std::pair<double, Eigen::Vector3d>> vio_for_align;
  int frame_idx = 0;
  int align_fit_count = 0;

  auto maybe_align = [&](double t) {
    if (aligner.solved() || t_init_done < 0)
      return;
    if (t - t_init_done < args.align_seconds)
      return;
    std::vector<Eigen::Vector3d> pv, pg;
    TrajectoryAligner::build_pairs(vio_for_align, gt_pos_map, 0.03, pv, pg);
    if (pv.size() >= 10) {
      if (aligner.solve(pv, pg)) {
        PRINT_INFO(GREEN "[ros-free] SE3 alignment solved using %zu pairs\n" RESET, pv.size());
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
      // [中文] 找 cam1 中离 t_cam 最近的一帧
      while (cam1_i + 1 < cam1.size() &&
             std::fabs(cam1[cam1_i + 1].timestamp - t_cam) < std::fabs(cam1[cam1_i].timestamp - t_cam))
        cam1_i++;
      if (std::fabs(cam1[cam1_i].timestamp - t_cam) < 0.01) {
        cv::Mat img1 = cv::imread(cam1[cam1_i].image_path, cv::IMREAD_GRAYSCALE);
        if (!img1.empty()) {
          msg.sensor_ids.push_back(1);
          msg.images.push_back(img1);
          msg.masks.push_back(cv::Mat::zeros(img1.size(), CV_8UC1));
        }
      }
    }

    double t0 = cv::getTickCount() / cv::getTickFrequency();
    sys->feed_measurement_camera(msg);
    double dt_ms = 1000.0 * (cv::getTickCount() / cv::getTickFrequency() - t0);

    // ------ query latest state ------
    if (sys->initialized()) {
      if (t_init_done < 0)
        t_init_done = t_cam;
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

      // dashboard data
      dash.update_vio_pose(t_cam, R_wi, p_wi, v_wi);
      std::vector<Eigen::Vector3d> slam_pts = sys->get_features_SLAM();
      std::vector<Eigen::Vector3d> msckf_pts = sys->get_good_features_MSCKF();
      dash.update_features(slam_pts, msckf_pts);

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
  PRINT_INFO(GREEN "[ros-free] done. trajectory written to %s (frames=%d, aligned_pairs=%d)\n" RESET,
             args.output_path.c_str(), frame_idx, align_fit_count);

  // final frame + hold if window
  dash.render_and_show(1);
  return EXIT_SUCCESS;
}
