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

#ifndef OV_MSCKF_ROS_FREE_VIZ_DASHBOARD_H
#define OV_MSCKF_ROS_FREE_VIZ_DASHBOARD_H

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace ov_msckf {

/**
 * @brief ROS-free OpenCV 仪表板。
 *
 * [中文] 单窗口 1920x1080, 9 子面板 3 行 3 列:
 *   行1 (h=540):
 *     - 3D iso (640x540): 等距 3D 视角 (azim 45°, elev -30°), GT 绿 / VIO 蓝 / 当前位姿三角
 *     - TOP-DOWN XY (640x540): 顶视 2D 轨迹 + SLAM 红 + MSCKF 白点
 *     - CAM (640x540): 最新相机图像 (含跟踪点叠加)
 *   行2 (h=270):
 *     - ATE (m): 实时 ATE 时序
 *     - Speed (m/s): VIO 速度范数 (蓝) + GT 速度 (绿, 若有)
 *     - Altitude Z (m): VIO z (蓝) + GT z (绿)
 *   行3 (h=270):
 *     - Roll (deg)
 *     - Pitch (deg)
 *     - Yaw (deg)
 *
 * 设计为"push data, render on demand": 主循环每帧调用 update_* 后 render_and_show().
 * 支持可选视频录制 (MP4 via cv::VideoWriter).
 */
class VizDashboard {
public:
  struct Options {
    int width = 1920;
    int height = 1080;
    double traj_scale_margin = 1.2; ///< [中文] 轨迹框体扩边比例
    size_t max_history = 20000;
    size_t timeseries_max = 5000;
    bool show_window = true;
    std::string window_title = "OpenVINS ROS-free Dashboard";
    std::string video_path; ///< [中文] 若非空则写 MP4
    int video_fps = 20;
    // fast mode: decimate curves/trajectories, limit drawn feature count,
    // skip expensive full-res resize.  Enable with --viz-fast.
    bool fast = false;
  };

  explicit VizDashboard(const Options &opts);
  ~VizDashboard();

  /// [中文] 当前 IMU/相机 最新位姿 (world=VIO frame)
  void update_vio_pose(double t, const Eigen::Matrix3d &R_wi, const Eigen::Vector3d &p_wi,
                       const Eigen::Vector3d &v_wi);
  /// [中文] 将 VIO 轨迹用拟合出的 T_GV 投到 GT frame 下, 存入绘制历史
  void set_alignment(const Eigen::Matrix3d &R_gv, const Eigen::Vector3d &t_gv, bool solved);

  /// [中文] 最新的 GPS / GT 点 (GT frame). 若时间戳相近自动与 VIO 配对用于算 ATE。
  void update_gt(double t, const Eigen::Vector3d &p_gt);

  /// [中文] 最新相机图像 (由 VioManager::get_historical_viz_image() 返回, 已带跟踪叠加)
  void update_image(double t, const cv::Mat &img);

  /// [中文] 最新的 SLAM 地图点和 MSCKF 临时特征 (world=VIO frame)
  void update_features(const std::vector<Eigen::Vector3d> &slam_pts,
                       const std::vector<Eigen::Vector3d> &msckf_pts);

  /// [中文] 设置 VIO 是否已初始化, 用于在面板上显示 "waiting init" 占位
  void set_initialized(bool initialized);

  void update_biases(double t, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba);
  void update_tracker_flow(const cv::Mat &prev_img, const std::vector<cv::Point2f> &prev_pts,
                           const std::vector<cv::Point2f> &curr_pts, const cv::Mat &curr_img,
                           bool warp_active, double t);
  void update_gps_alt_diag(double t, double gps_z, double vio_z,
                            double residual, double pzz, double kpz);
  void update_backend_diag(double t, int n_chi2_rej, int n_accepted);
  void update_tracker_diag(double t, int n_klt_attempted, int n_newly_detected);

  /// [中文] 渲染到内部画布, 可选写入视频, 可选 imshow. 返回 false 表示用户按了 q/ESC。
  bool render_and_show(int wait_ms = 1);

  /// [中文] 拿到最近一次渲染的画布 (方便保存最后一帧)
  cv::Mat last_canvas() const { return canvas_.clone(); }

  /// Last render wall time in milliseconds (updated after each render_and_show).
  double last_render_ms() const { return last_render_ms_; }

private:
  struct PoseEntry {
    double t;
    Eigen::Vector3d p;
  };

  // Snapshot of all data needed for one render frame, captured under mu_ then
  // rendered without the lock held.
  struct DrawSnap {
    std::deque<PoseEntry> vio_hist;
    std::deque<PoseEntry> gt_hist;
    std::vector<Eigen::Vector3d> slam_pts;
    std::vector<Eigen::Vector3d> msckf_pts;
    Eigen::Matrix3d R_gv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_gv = Eigen::Vector3d::Zero();
    bool aligned = false;
    double latest_t = -1;
    Eigen::Matrix3d latest_R_wi = Eigen::Matrix3d::Identity();
    Eigen::Vector3d latest_p_wi = Eigen::Vector3d::Zero();
    cv::Mat image;
    double image_t = -1;
    bool initialized = false;
    // time series
    std::deque<std::pair<double, double>> ts_ate;
    std::deque<std::pair<double, double>> ts_speed;
    std::deque<std::pair<double, double>> ts_z_vio_raw;
    std::deque<std::pair<double, double>> ts_z_gt;
    std::deque<std::pair<double, double>> ts_roll;
    std::deque<std::pair<double, double>> ts_pitch;
    std::deque<std::pair<double, double>> ts_yaw;
    std::deque<std::pair<double, double>> ts_slam_count;
    std::deque<std::pair<double, double>> ts_msckf_count;
    std::deque<std::pair<double, double>> ts_tracked_count;
    std::deque<std::pair<double, double>> ts_klt_raw;
    std::deque<std::pair<double, double>> ts_chi2_rejected;
    std::deque<std::pair<double, double>> ts_bg_norm;
    std::deque<std::pair<double, double>> ts_ba_norm;
    std::deque<std::pair<double, double>> ts_gps_res;
    std::deque<std::pair<double, double>> ts_gps_kpz;
  };

  void draw_trajectory_topdown(cv::Mat &roi, const DrawSnap &s, int traj_step);
  void draw_trajectory_iso(cv::Mat &roi, const DrawSnap &s, int traj_step);
  void draw_camera_image(cv::Mat &roi, const DrawSnap &s);
  static void draw_panel_xy(cv::Mat &roi, const std::string &title,
                     const std::vector<std::pair<std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
                     const std::vector<std::string> &labels, bool symmetric_y = false,
                     bool initialized = true, int decimate = 1);
  void draw_waiting_init(cv::Mat &roi, const std::string &title);

  static void draw_grid(cv::Mat &img, cv::Scalar color, int step = 40);
  static void draw_text(cv::Mat &img, const std::string &s, cv::Point p, cv::Scalar color,
                        double scale = 0.5, int thickness = 1);
  static void draw_curve(cv::Mat &roi, const std::deque<std::pair<double, double>> &pts,
                         cv::Scalar color, double y_min, double y_max, double x_min, double x_max,
                         int thickness = 1, int decimate = 1);
  static Eigen::Vector3d quat_to_rpy(const Eigen::Matrix3d &R);

  Options opts_;
  cv::Mat canvas_;
  cv::VideoWriter video_;

  mutable std::mutex mu_;
  std::deque<PoseEntry> vio_hist_;      // [中文] 原始 VIO (未对齐) 历史
  std::deque<PoseEntry> gt_hist_;       // [中文] GT 历史 (GT frame)
  Eigen::Matrix3d R_gv_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_gv_ = Eigen::Vector3d::Zero();
  bool aligned_ = false;
  bool initialized_ = false;

  // latest state
  double latest_t_ = -1;
  Eigen::Matrix3d latest_R_wi_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d latest_p_wi_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_v_wi_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_p_gt_ = Eigen::Vector3d::Zero();
  bool has_gt_ = false;

  cv::Mat latest_image_;
  double latest_image_t_ = -1;

  std::vector<Eigen::Vector3d> slam_pts_;
  std::vector<Eigen::Vector3d> msckf_pts_;

  // time series (all keyed on dt = t - t_start_)
  std::deque<std::pair<double, double>> ts_ate_;
  std::deque<std::pair<double, double>> ts_speed_;
  std::deque<std::pair<double, double>> ts_z_vio_;      // aligned Z (for 3D/2D traj)
  std::deque<std::pair<double, double>> ts_z_vio_raw_;  // raw VIO Z (never aligned) for Alt Z panel
  std::deque<std::pair<double, double>> ts_z_gt_;
  std::deque<std::pair<double, double>> ts_roll_;
  std::deque<std::pair<double, double>> ts_pitch_;
  std::deque<std::pair<double, double>> ts_yaw_;
  std::deque<std::pair<double, double>> ts_slam_count_;
  std::deque<std::pair<double, double>> ts_msckf_count_;
  std::deque<std::pair<double, double>> ts_tracked_count_;
  std::deque<std::pair<double, double>> ts_klt_raw_;        // pre-RANSAC KLT count
  std::deque<std::pair<double, double>> ts_chi2_rejected_;  // MSCKF chi2 rejections per frame
  std::deque<std::pair<double, double>> ts_bg_norm_;
  std::deque<std::pair<double, double>> ts_ba_norm_;
  std::deque<std::pair<double, double>> ts_gps_res_;
  std::deque<std::pair<double, double>> ts_gps_kpz_;
  double t_start_ = -1;

  // render timing
  double last_render_ms_ = 0.0;
  int render_count_ = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_VIZ_DASHBOARD_H
