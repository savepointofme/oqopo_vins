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
#include <atomic>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <vector>

namespace ov_msckf {

/**
 * @brief ROS-free OpenCV dashboard.
 *
 * Single window 1920x1080, grid layout:
 *   Row 1 (540px): 3D iso | TOP-DOWN XY  | cross-pane matches
 *   Row 2 (270px): ATE    | Altitude Z   | (full-height col 2:
 *   Row 3 (270px): RPY    | Speed        |  top=CURRENT RAW, bot=WARPED PREV)
 *
 * Col 2 (640px) is a unified cross-pane matching view sourced from the
 * tracker-owned warp visualization packet — no independent homography path.
 *
 * Render thread runs at fixed rate, fully decoupled from the estimator.
 * Main thread calls show_frame() for lightweight imshow + waitKey.
 * Video writing is done by the render thread.
 */
class VizDashboard {
public:
  struct Options {
    int width = 1920;
    int height = 1080;
    double traj_scale_margin = 1.2;
    size_t max_history = 20000;
    size_t timeseries_max = 5000;
    bool show_window = true;
    std::string video_path;
    int video_fps = 20;
    /// Target render rate for the background render thread (Hz).
    double render_fps = 20.0;
  };

  explicit VizDashboard(const Options &opts);
  ~VizDashboard();

  /// Push latest VIO pose. Thread-safe; returns immediately.
  void update_vio_pose(double t, const Eigen::Matrix3d &R_wi, const Eigen::Vector3d &p_wi,
                       const Eigen::Vector3d &v_wi);
  void set_alignment(const Eigen::Matrix3d &R_gv, const Eigen::Vector3d &t_gv, bool solved);
  void update_gt(double t, const Eigen::Vector3d &p_gt);
  void update_image(double t, const cv::Mat &img);
  /// Push tracker-owned warp visualization packet for cross-pane rendering.
  /// This is the single source of truth — no independent homography path exists.
  void update_tracker_flow(const cv::Mat &warped_img, const std::vector<cv::Point2f> &prev_pts,
                           const std::vector<cv::Point2f> &curr_pts, const cv::Mat &curr_raw,
                           bool warp_active, double t_curr);
  void update_features(const std::vector<Eigen::Vector3d> &slam_pts,
                       const std::vector<Eigen::Vector3d> &msckf_pts);
  void set_initialized(bool initialized);

  /// Push a GPS altitude diagnostic sample (call after each feed_measurement_gps_altitude).
  /// @param t      camera timestamp (seconds)
  /// @param gps_z  GPS altitude fed to the filter (after reference subtraction)
  /// @param vio_z  VIO p_z BEFORE the EKF update
  /// @param residual raw residual (GPS - VIO), before any clipping
  /// @param pzz    P_zz BEFORE the update (after any floor injection)
  /// @param kpz    Kalman gain K_pz
  void update_gps_alt_diag(double t, double gps_z, double vio_z,
                            double residual, double pzz, double kpz);
  /// Push latest IMU bias estimates from the EKF state. Thread-safe; only recorded after t_start is set.
  void update_biases(double t, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba);

  /// Present the latest rendered frame to the screen.
  /// MUST be called from the same thread that created the OpenCV window (main thread).
  /// Very cheap: only imshow + waitKey(1). Returns false if user pressed q/ESC.
  bool show_frame();

  /// Backward-compatible wrapper; delegates to show_frame().
  bool render_and_show(int wait_ms = 1);

  cv::Mat last_canvas() const;

private:
  struct PoseEntry {
    double t;
    Eigen::Vector3d p;
  };

  // All state written by update_*() and read by the render thread.
  // Protected by mu_.
  struct DataState {
    std::deque<PoseEntry> vio_hist;
    std::deque<PoseEntry> gt_hist;
    Eigen::Matrix3d R_gv = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_gv = Eigen::Vector3d::Zero();
    bool aligned = false;
    bool initialized = false;
    double latest_t = -1;
    Eigen::Matrix3d latest_R_wi = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d latest_R_wi_prev = Eigen::Matrix3d::Identity();
    Eigen::Vector3d latest_p_wi = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_v_wi = Eigen::Vector3d::Zero();
    Eigen::Vector3d latest_p_gt = Eigen::Vector3d::Zero();
    bool has_gt = false;
    cv::Mat latest_image; // always a fresh clone; never modified in-place
    double latest_image_t = -1;
    // Tracker-owned flow data for cross-pane visualization
    cv::Mat flow_warped_img;           // warped prev (or raw prev if warp off)
    std::vector<cv::Point2f> flow_prev_pts;
    std::vector<cv::Point2f> flow_curr_pts;
    cv::Mat flow_curr_raw;             // current raw image
    bool flow_warp_active = false;
    bool flow_valid = false;
    double flow_t_curr = 0.0;
    std::vector<Eigen::Vector3d> slam_pts;
    std::vector<Eigen::Vector3d> msckf_pts;
    std::deque<std::pair<double, double>> ts_ate;
    std::deque<std::pair<double, double>> ts_speed;
    std::deque<std::pair<double, double>> ts_z_vio;
    std::deque<std::pair<double, double>> ts_z_gt;
    std::deque<std::pair<double, double>> ts_roll;
    std::deque<std::pair<double, double>> ts_pitch;
    std::deque<std::pair<double, double>> ts_yaw;
    // GPS altitude diagnostics (populated only when GPS fusion is active)
    std::deque<std::pair<double, double>> ts_gps_z;       ///< GPS alt (fed value)
    std::deque<std::pair<double, double>> ts_gps_vio_z;   ///< VIO z at GPS update time
    std::deque<std::pair<double, double>> ts_gps_res;     ///< residual (raw)
    std::deque<std::pair<double, double>> ts_gps_pzz;     ///< P_zz before update
    std::deque<std::pair<double, double>> ts_gps_kpz;     ///< K_pz
    bool has_gps_diag = false;
    // IMU bias time-series — populated by update_biases(), recorded after initialization
    std::deque<std::pair<double, double>> ts_bg_x, ts_bg_y, ts_bg_z;
    std::deque<std::pair<double, double>> ts_ba_x, ts_ba_y, ts_ba_z;
    double t_start = -1;
  };

  void render_loop();
  void render_frame(cv::Mat &canvas, const DataState &d);

  void draw_trajectory_topdown(cv::Mat &roi, const DataState &d);
  void draw_trajectory_iso(cv::Mat &roi, const DataState &d);
  void draw_camera_image(cv::Mat &roi, const DataState &d);
  void draw_panel_rpy(cv::Mat &roi, const DataState &d);
  void draw_panel_gps_diag(cv::Mat &roi, const DataState &d);
  void draw_panel_biases(cv::Mat &roi, const DataState &d);
  void draw_cross_pane_matches(cv::Mat &roi, const DataState &d);
  // series holds const pointers to deques in the DataState snapshot
  static void draw_panel_xy(cv::Mat &roi, const std::string &title,
                     const std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
                     const std::vector<std::string> &labels, bool symmetric_y, bool initialized);
  static void draw_waiting_init(cv::Mat &roi, const std::string &title);

  static void draw_grid(cv::Mat &img, cv::Scalar color, int step = 40);
  static void draw_text(cv::Mat &img, const std::string &s, cv::Point p, cv::Scalar color,
                        double scale = 0.5, int thickness = 1);
  static void draw_curve(cv::Mat &roi, const std::deque<std::pair<double, double>> &pts,
                         cv::Scalar color, double y_min, double y_max, double x_min, double x_max,
                         int thickness = 1);
  static Eigen::Vector3d quat_to_rpy(const Eigen::Matrix3d &R);

  Options opts_;
  cv::VideoWriter video_;

  mutable std::mutex mu_;
  DataState data_; // protected by mu_

  // Render thread
  std::atomic<bool> render_running_{false};
  std::thread render_thread_;
  cv::Mat render_buf_; // only touched by render thread; no lock needed

  // Display double-buffer: render thread writes, show_frame() reads
  mutable std::mutex display_mu_;
  cv::Mat display_mat_; // latest rendered frame

};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_VIZ_DASHBOARD_H
