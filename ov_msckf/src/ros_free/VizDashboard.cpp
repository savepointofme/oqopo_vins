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

#include "VizDashboard.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include "utils/colors.h"
#include "utils/print.h"

namespace ov_msckf {

namespace {
// [中文] 等距 3D 投影矩阵: azim=45°, elev=-30° (从右上俯视).
Eigen::Matrix3d isometric_R() {
  const double az = 45.0 * M_PI / 180.0;
  const double el = -30.0 * M_PI / 180.0;
  Eigen::Matrix3d Rz, Rx;
  Rz << std::cos(az), -std::sin(az), 0, //
      std::sin(az), std::cos(az), 0,    //
      0, 0, 1;
  Rx << 1, 0, 0,                              //
      0, std::cos(el), -std::sin(el),         //
      0, std::sin(el), std::cos(el);
  return Rx * Rz;
}
} // namespace

VizDashboard::VizDashboard(const Options &opts) : opts_(opts) {
  canvas_ = cv::Mat::zeros(opts_.height, opts_.width, CV_8UC3);
  if (!opts_.video_path.empty()) {
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    video_.open(opts_.video_path, fourcc, opts_.video_fps, cv::Size(opts_.width, opts_.height), true);
    if (!video_.isOpened()) {
      PRINT_WARNING(YELLOW "[viz] failed to open video writer: %s\n" RESET, opts_.video_path.c_str());
    } else {
      PRINT_INFO(GREEN "[viz] writing video to %s @ %d fps\n" RESET, opts_.video_path.c_str(), opts_.video_fps);
    }
  }
  if (opts_.fast)
    PRINT_INFO(CYAN "[viz] fast mode ON — curves decimated, features capped at 200\n" RESET);
}

VizDashboard::~VizDashboard() {
  if (video_.isOpened())
    video_.release();
}

void VizDashboard::update_vio_pose(double t, const Eigen::Matrix3d &R_wi, const Eigen::Vector3d &p_wi,
                                   const Eigen::Vector3d &v_wi) {
  std::lock_guard<std::mutex> lk(mu_);
  latest_t_ = t;
  latest_R_wi_ = R_wi;
  latest_p_wi_ = p_wi;
  latest_v_wi_ = v_wi;
  if (t_start_ < 0)
    t_start_ = t;
  vio_hist_.push_back({t, p_wi});
  while (vio_hist_.size() > opts_.max_history)
    vio_hist_.pop_front();

  Eigen::Vector3d rpy = quat_to_rpy(R_wi) * 180.0 / M_PI;
  double dt = t - t_start_;
  ts_speed_.push_back({dt, v_wi.norm()});
  ts_roll_.push_back({dt, rpy.x()});
  ts_pitch_.push_back({dt, rpy.y()});
  ts_yaw_.push_back({dt, rpy.z()});
  Eigen::Vector3d p_in_gt = aligned_ ? (R_gv_ * p_wi + t_gv_) : p_wi;
  ts_z_vio_.push_back({dt, p_in_gt.z()});
  ts_z_vio_raw_.push_back({dt, p_wi.z()});
  auto trim = [&](std::deque<std::pair<double, double>> &q) {
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  trim(ts_speed_);
  trim(ts_roll_);
  trim(ts_pitch_);
  trim(ts_yaw_);
  trim(ts_z_vio_);
  trim(ts_z_vio_raw_);

  if (aligned_ && has_gt_) {
    Eigen::Vector3d p_vio_in_gt = R_gv_ * p_wi + t_gv_;
    Eigen::Vector3d d = latest_p_gt_ - p_vio_in_gt;
    ts_ate_.push_back({dt, d.norm()});
    trim(ts_ate_);
  }
}

void VizDashboard::set_alignment(const Eigen::Matrix3d &R_gv, const Eigen::Vector3d &t_gv, bool solved) {
  std::lock_guard<std::mutex> lk(mu_);
  R_gv_ = R_gv;
  t_gv_ = t_gv;
  aligned_ = solved;
}

void VizDashboard::update_gt(double t, const Eigen::Vector3d &p_gt) {
  std::lock_guard<std::mutex> lk(mu_);
  gt_hist_.push_back({t, p_gt});
  while (gt_hist_.size() > opts_.max_history)
    gt_hist_.pop_front();
  latest_p_gt_ = p_gt;
  has_gt_ = true;
  if (t_start_ > 0) {
    double dt = t - t_start_;
    ts_z_gt_.push_back({dt, p_gt.z()});
    while (ts_z_gt_.size() > opts_.timeseries_max)
      ts_z_gt_.pop_front();
  }
}

void VizDashboard::update_image(double t, const cv::Mat &img) {
  std::lock_guard<std::mutex> lk(mu_);
  if (img.empty())
    return;
  if (img.channels() == 1)
    cv::cvtColor(img, latest_image_, cv::COLOR_GRAY2BGR);
  else
    latest_image_ = img.clone();
  latest_image_t_ = t;
}

void VizDashboard::update_features(const std::vector<Eigen::Vector3d> &slam_pts,
                                   const std::vector<Eigen::Vector3d> &msckf_pts) {
  std::lock_guard<std::mutex> lk(mu_);
  slam_pts_ = slam_pts;
  msckf_pts_ = msckf_pts;
  if (t_start_ > 0) {
    double dt = latest_t_ - t_start_;
    ts_slam_count_.push_back({dt, (double)slam_pts.size()});
    ts_msckf_count_.push_back({dt, (double)msckf_pts.size()});
    while (ts_slam_count_.size() > opts_.timeseries_max)
      ts_slam_count_.pop_front();
    while (ts_msckf_count_.size() > opts_.timeseries_max)
      ts_msckf_count_.pop_front();
  }
}

void VizDashboard::set_initialized(bool initialized) {
  std::lock_guard<std::mutex> lk(mu_);
  initialized_ = initialized;
}

void VizDashboard::update_biases(double t, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba) {
  std::lock_guard<std::mutex> lk(mu_);
  if (t_start_ > 0) {
    double dt = t - t_start_;
    ts_bg_norm_.push_back({dt, bg.norm()});
    ts_ba_norm_.push_back({dt, ba.norm()});
    while (ts_bg_norm_.size() > opts_.timeseries_max)
      ts_bg_norm_.pop_front();
    while (ts_ba_norm_.size() > opts_.timeseries_max)
      ts_ba_norm_.pop_front();
  }
}

void VizDashboard::update_tracker_flow(const cv::Mat & /*warped_img*/,
                                       const std::vector<cv::Point2f> & /*prev_pts*/,
                                       const std::vector<cv::Point2f> &curr_pts,
                                       const cv::Mat & /*curr_raw*/,
                                       bool /*warp_active*/, double t_curr) {
  std::lock_guard<std::mutex> lk(mu_);
  if (t_start_ > 0) {
    double dt = t_curr - t_start_;
    ts_tracked_count_.push_back({dt, (double)curr_pts.size()});
    while (ts_tracked_count_.size() > opts_.timeseries_max)
      ts_tracked_count_.pop_front();
  }
}

void VizDashboard::update_gps_alt_diag(double t, double /*gps_z*/, double /*vio_z*/,
                                        double residual, double /*pzz*/, double kpz) {
  std::lock_guard<std::mutex> lk(mu_);
  if (t_start_ > 0) {
    double dt = t - t_start_;
    ts_gps_res_.push_back({dt, residual});
    ts_gps_kpz_.push_back({dt, kpz});
    while (ts_gps_res_.size() > opts_.timeseries_max)
      ts_gps_res_.pop_front();
    while (ts_gps_kpz_.size() > opts_.timeseries_max)
      ts_gps_kpz_.pop_front();
  }
}

void VizDashboard::update_backend_diag(double t, int n_chi2_rejected, int /*n_accepted*/) {
  std::lock_guard<std::mutex> lk(mu_);
  if (t_start_ > 0) {
    double dt = t - t_start_;
    ts_chi2_rejected_.push_back({dt, (double)n_chi2_rejected});
    while (ts_chi2_rejected_.size() > opts_.timeseries_max)
      ts_chi2_rejected_.pop_front();
  }
}

void VizDashboard::update_tracker_diag(double t, int n_klt_attempted, int n_newly_detected) {
  std::lock_guard<std::mutex> lk(mu_);
  if (t_start_ > 0) {
    double dt = t - t_start_;
    ts_klt_raw_.push_back({dt, (double)n_klt_attempted});
    while (ts_klt_raw_.size() > opts_.timeseries_max)
      ts_klt_raw_.pop_front();
  }
  (void)n_newly_detected;
}

// ============================================================
// Static helpers
// ============================================================

void VizDashboard::draw_grid(cv::Mat &img, cv::Scalar color, int step) {
  for (int x = 0; x < img.cols; x += step)
    cv::line(img, {x, 0}, {x, img.rows - 1}, color, 1, cv::LINE_AA);
  for (int y = 0; y < img.rows; y += step)
    cv::line(img, {0, y}, {img.cols - 1, y}, color, 1, cv::LINE_AA);
}

void VizDashboard::draw_text(cv::Mat &img, const std::string &s, cv::Point p, cv::Scalar color,
                             double scale, int thickness) {
  cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

Eigen::Vector3d VizDashboard::quat_to_rpy(const Eigen::Matrix3d &R) {
  double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  double roll, yaw;
  if (std::fabs(std::cos(pitch)) > 1e-6) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = 0.0;
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  return {roll, pitch, yaw};
}

// draw_curve: renders pts as a polyline. decimate>1 skips intermediate points
// so only every decimate-th point is drawn, reducing cv::line calls proportionally.
void VizDashboard::draw_curve(cv::Mat &roi, const std::deque<std::pair<double, double>> &pts,
                              cv::Scalar color, double y_min, double y_max, double x_min,
                              double x_max, int thickness, int decimate) {
  if (pts.empty() || x_max <= x_min || y_max <= y_min)
    return;
  const int eff_dec = std::max(1, decimate);
  double sx = (roi.cols - 60) / (x_max - x_min);
  double sy = (roi.rows - 50) / (y_max - y_min);
  auto tx = [&](double x, double y) {
    return cv::Point(static_cast<int>(50 + (x - x_min) * sx),
                     static_cast<int>(roi.rows - 30 - (y - y_min) * sy));
  };
  cv::Point prev;
  bool first = true;
  int idx = 0;
  for (const auto &p : pts) {
    if (idx++ % eff_dec != 0)
      continue;
    cv::Point q = tx(p.first, std::max(y_min, std::min(y_max, p.second)));
    if (!first)
      cv::line(roi, prev, q, color, thickness, cv::LINE_AA);
    prev = q;
    first = false;
  }
}

// ============================================================
// Panel drawing
// ============================================================

void VizDashboard::draw_trajectory_topdown(cv::Mat &roi, const DrawSnap &s, int traj_step) {
  roi.setTo(cv::Scalar(20, 20, 20));
  const int step = std::max(1, traj_step);

  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto expand = [&](double x, double y) {
    xmin = std::min(xmin, x);
    xmax = std::max(xmax, x);
    ymin = std::min(ymin, y);
    ymax = std::max(ymax, y);
  };
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return s.aligned ? (s.R_gv * p + s.t_gv) : p;
  };

  // bounding box: sample every step to match drawing stride
  for (size_t i = 0; i < s.vio_hist.size(); i += step) {
    auto pg = tx(s.vio_hist[i].p);
    expand(pg.x(), pg.y());
  }
  for (const auto &e : s.gt_hist)
    expand(e.p.x(), e.p.y());
  for (const auto &p : s.slam_pts) {
    auto pg = tx(p);
    expand(pg.x(), pg.y());
  }
  if (!std::isfinite(xmin)) { xmin = -5; xmax = 5; ymin = -5; ymax = 5; }

  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double sc = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](double x, double y) {
    return cv::Point(static_cast<int>((x - cx) * sc + roi.cols / 2),
                     static_cast<int>(-(y - cy) * sc + roi.rows / 2));
  };

  // fixed-meter grid
  double ideal_cell = range / 6.0;
  double cell_m;
  if (ideal_cell < 10)        cell_m = 5;
  else if (ideal_cell < 25)   cell_m = 10;
  else if (ideal_cell < 50)   cell_m = 20;
  else if (ideal_cell < 100)  cell_m = 50;
  else if (ideal_cell < 250)  cell_m = 100;
  else if (ideal_cell < 500)  cell_m = 200;
  else if (ideal_cell < 1000) cell_m = 500;
  else                        cell_m = 1000;

  double x_left  = cx - (roi.cols / 2.0) / sc;
  double x_right = cx + (roi.cols / 2.0) / sc;
  double x0 = std::floor(x_left / cell_m) * cell_m;
  for (double x = x0; x <= x_right + 1e-3; x += cell_m) {
    int px = static_cast<int>((x - cx) * sc + roi.cols / 2);
    if (px < 0 || px >= roi.cols) continue;
    cv::line(roi, {px, 0}, {px, roi.rows - 1}, {40, 40, 40}, 1, cv::LINE_AA);
    if (px > 20 && px < roi.cols - 20) {
      std::ostringstream os; os << std::fixed << std::setprecision(0) << x;
      draw_text(roi, os.str(), {px + 2, roi.rows - 5}, {90, 90, 90}, 0.35, 1);
    }
  }
  double y_bottom = cy - (roi.rows / 2.0) / sc;
  double y_top    = cy + (roi.rows / 2.0) / sc;
  double y0 = std::floor(y_bottom / cell_m) * cell_m;
  for (double y = y0; y <= y_top + 1e-3; y += cell_m) {
    int py = static_cast<int>(-(y - cy) * sc + roi.rows / 2);
    if (py < 0 || py >= roi.rows) continue;
    cv::line(roi, {0, py}, {roi.cols - 1, py}, {40, 40, 40}, 1, cv::LINE_AA);
    if (py > 10 && py < roi.rows - 10) {
      std::ostringstream os; os << std::fixed << std::setprecision(0) << y;
      draw_text(roi, os.str(), {2, py - 2}, {90, 90, 90}, 0.35, 1);
    }
  }

  // coordinate axes
  cv::Point origin_px = to_px(0, 0);
  if (origin_px.x > 0 && origin_px.x < roi.cols) {
    cv::line(roi, {origin_px.x, 0}, {origin_px.x, roi.rows - 1}, {70, 70, 70}, 1, cv::LINE_AA);
    draw_text(roi, "N", {origin_px.x + 2, 14}, {120, 120, 120}, 0.4, 1);
  }
  if (origin_px.y > 0 && origin_px.y < roi.rows) {
    cv::line(roi, {0, origin_px.y}, {roi.cols - 1, origin_px.y}, {70, 70, 70}, 1, cv::LINE_AA);
    draw_text(roi, "E", {roi.cols - 18, origin_px.y - 2}, {120, 120, 120}, 0.4, 1);
  }

  // GT line (always draw all — typically short)
  for (size_t i = 1; i < s.gt_hist.size(); ++i) {
    cv::line(roi, to_px(s.gt_hist[i - 1].p.x(), s.gt_hist[i - 1].p.y()),
             to_px(s.gt_hist[i].p.x(), s.gt_hist[i].p.y()), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
  }
  // VIO line (decimated by step)
  for (size_t i = 0; i + (size_t)step < s.vio_hist.size(); i += step) {
    auto a = tx(s.vio_hist[i].p);
    auto b = tx(s.vio_hist[i + step].p);
    cv::line(roi, to_px(a.x(), a.y()), to_px(b.x(), b.y()), cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
  }
  // SLAM features (already capped at 200 in fast mode)
  for (const auto &p : s.slam_pts) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(0, 0, 230), -1, cv::LINE_AA);
  }
  // MSCKF features (already capped)
  for (const auto &p : s.msckf_pts) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  }
  // Current camera pose arrow
  if (s.latest_t > 0) {
    Eigen::Vector3d pg = tx(s.latest_p_wi);
    Eigen::Vector3d fwd_vio = s.latest_R_wi * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d fwd_gt = s.aligned ? (s.R_gv * fwd_vio) : fwd_vio;
    double yaw = std::atan2(fwd_gt.y(), fwd_gt.x());
    cv::Point p0 = to_px(pg.x(), pg.y());
    double L = 0.6 * sc;
    double hw = 0.3;
    cv::Point tip(p0.x + static_cast<int>(L * std::cos(yaw)),
                  p0.y - static_cast<int>(L * std::sin(yaw)));
    cv::Point left(p0.x + static_cast<int>(hw * L * std::cos(yaw + 2.5)),
                   p0.y - static_cast<int>(hw * L * std::sin(yaw + 2.5)));
    cv::Point right(p0.x + static_cast<int>(hw * L * std::cos(yaw - 2.5)),
                    p0.y - static_cast<int>(hw * L * std::sin(yaw - 2.5)));
    std::vector<cv::Point> tri{tip, left, right};
    cv::fillConvexPoly(roi, tri, cv::Scalar(255, 255, 0), cv::LINE_AA);
    cv::circle(roi, p0, 4, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
  }
  draw_text(roi, "TOP-DOWN XY (m)", {10, 20}, {200, 200, 200}, 0.55, 1);
  draw_text(roi, "GT green", {10, 40}, {0, 200, 0}, 0.45, 1);
  draw_text(roi, "VIO aligned", {10, 58}, {255, 160, 0}, 0.45, 1);
  draw_text(roi, "SLAM red  MSCKF white", {10, 76}, {220, 220, 220}, 0.45, 1);
  draw_text(roi, std::string("align=") + (s.aligned ? "on" : "off"), {10, 94},
            s.aligned ? cv::Scalar(0, 255, 0) : cv::Scalar(120, 120, 120), 0.45, 1);
}

void VizDashboard::draw_trajectory_iso(cv::Mat &roi, const DrawSnap &s, int traj_step) {
  roi.setTo(cv::Scalar(18, 18, 22));
  draw_grid(roi, cv::Scalar(38, 38, 42), 40);
  const int step = std::max(1, traj_step);

  static const Eigen::Matrix3d R_iso = isometric_R();

  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return s.aligned ? (s.R_gv * p + s.t_gv) : p;
  };
  auto expand = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d sp = R_iso * p_world;
    xmin = std::min(xmin, sp.x());
    xmax = std::max(xmax, sp.x());
    ymin = std::min(ymin, sp.y());
    ymax = std::max(ymax, sp.y());
  };
  for (size_t i = 0; i < s.vio_hist.size(); i += step)
    expand(tx(s.vio_hist[i].p));
  for (const auto &e : s.gt_hist)
    expand(e.p);
  if (!std::isfinite(xmin)) { xmin = -5; xmax = 5; ymin = -5; ymax = 5; }

  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double sc = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d sp = R_iso * p_world;
    return cv::Point(static_cast<int>((sp.x() - cx) * sc + roi.cols / 2),
                     static_cast<int>(-(sp.y() - cy) * sc + roi.rows / 2));
  };

  // coordinate axes
  Eigen::Vector3d ref_origin =
      s.aligned ? (s.gt_hist.empty() ? Eigen::Vector3d::Zero() : s.gt_hist.front().p)
                : (s.vio_hist.empty() ? Eigen::Vector3d::Zero() : s.vio_hist.front().p);
  double axis_len = 5.0;
  cv::Point o = to_px(ref_origin);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(axis_len, 0, 0)), {80, 80, 200}, 1, cv::LINE_AA);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, axis_len, 0)), {80, 200, 80}, 1, cv::LINE_AA);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len)), {200, 80, 80}, 1, cv::LINE_AA);
  draw_text(roi, "X", to_px(ref_origin + Eigen::Vector3d(axis_len * 1.05, 0, 0)), {120, 120, 220}, 0.45, 1);
  draw_text(roi, "Y", to_px(ref_origin + Eigen::Vector3d(0, axis_len * 1.05, 0)), {120, 220, 120}, 0.45, 1);
  draw_text(roi, "Z", to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len * 1.05)), {220, 120, 120}, 0.45, 1);

  // GT line (full)
  for (size_t i = 1; i < s.gt_hist.size(); ++i)
    cv::line(roi, to_px(s.gt_hist[i - 1].p), to_px(s.gt_hist[i].p), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
  // VIO line (decimated)
  for (size_t i = 0; i + (size_t)step < s.vio_hist.size(); i += step) {
    cv::line(roi, to_px(tx(s.vio_hist[i].p)), to_px(tx(s.vio_hist[i + step].p)),
             cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
  }
  // current pose
  if (s.latest_t > 0)
    cv::circle(roi, to_px(tx(s.latest_p_wi)), 5, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);

  draw_text(roi, "3D ISO (azim=45 elev=-30)", {10, 20}, {200, 200, 200}, 0.55, 1);
  draw_text(roi, "GT green   VIO blue", {10, 40}, {200, 200, 200}, 0.45, 1);
}

void VizDashboard::draw_camera_image(cv::Mat &roi, const DrawSnap &s) {
  roi.setTo(cv::Scalar(10, 10, 10));
  if (s.image.empty()) {
    draw_text(roi, "waiting for camera...", {20, 40}, {180, 180, 180}, 0.6, 1);
    return;
  }
  cv::Mat resized;
  double sx = static_cast<double>(roi.cols) / s.image.cols;
  double sy = static_cast<double>(roi.rows - 40) / s.image.rows;
  double sc = std::min(sx, sy);
  int w = static_cast<int>(s.image.cols * sc);
  int h = static_cast<int>(s.image.rows * sc);
  // fast mode: INTER_LINEAR is faster than INTER_AREA and good enough for display
  int interp = opts_.fast ? cv::INTER_LINEAR : cv::INTER_AREA;
  cv::resize(s.image, resized, cv::Size(w, h), 0, 0, interp);
  int ox = (roi.cols - w) / 2;
  int oy = 40 + (roi.rows - 40 - h) / 2;
  resized.copyTo(roi(cv::Rect(ox, oy, w, h)));
  std::ostringstream os;
  os << "CAM  t=" << std::fixed << std::setprecision(3) << s.image_t
     << "  src=" << s.image.cols << "x" << s.image.rows
     << "  init=" << (s.initialized ? "ok" : "wait");
  draw_text(roi, os.str(), {10, 24}, {200, 200, 200}, 0.55, 1);
}

void VizDashboard::draw_panel_xy(
    cv::Mat &roi, const std::string &title,
    const std::vector<std::pair<std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
    const std::vector<std::string> &labels, bool symmetric_y,
    bool initialized, int decimate) {
  roi.setTo(cv::Scalar(15, 15, 15));
  cv::rectangle(roi, {0, 0, roi.cols - 1, roi.rows - 1}, {50, 50, 50}, 1);
  draw_text(roi, title, {10, 18}, {220, 220, 220}, 0.5, 1);
  bool any_data = false;
  for (auto &s : series)
    if (!s.first->empty()) { any_data = true; break; }
  if (!any_data) {
    draw_text(roi, initialized ? "no data" : "waiting init...", {10, roi.rows / 2},
              {120, 120, 120}, 0.5, 1);
    return;
  }
  double xmin = std::numeric_limits<double>::infinity();
  double xmax = -xmin;
  for (auto &s : series) {
    if (s.first->empty()) continue;
    xmin = std::min(xmin, s.first->front().first);
    xmax = std::max(xmax, s.first->back().first);
  }
  if (!std::isfinite(xmin) || xmax <= xmin)
    xmax = xmin + 1.0;
  double ymin = std::numeric_limits<double>::infinity();
  double ymax = -ymin;
  for (auto &s : series) {
    for (const auto &p : *s.first) {
      ymin = std::min(ymin, p.second);
      ymax = std::max(ymax, p.second);
    }
  }
  if (!std::isfinite(ymin)) { ymin = 0; ymax = 1; }
  if (!(ymax > ymin)) ymax = ymin + 1.0;
  double pad = 0.1 * (ymax - ymin);
  ymin -= pad;
  ymax += pad;
  if (symmetric_y) {
    double m = std::max(std::fabs(ymin), std::fabs(ymax));
    ymin = -m; ymax = m;
  }
  cv::line(roi, {50, roi.rows - 30}, {roi.cols - 10, roi.rows - 30}, {90, 90, 90}, 1);
  cv::line(roi, {50, 30}, {50, roi.rows - 30}, {90, 90, 90}, 1);

  double sx = (roi.cols - 60) / (xmax - xmin);
  double sy = (roi.rows - 50) / (ymax - ymin);

  const int n_y_ticks = 4;
  for (int i = 0; i <= n_y_ticks; ++i) {
    double y_val = ymin + (ymax - ymin) * i / n_y_ticks;
    int y_px = static_cast<int>(roi.rows - 30 - (y_val - ymin) * sy);
    y_px = std::max(30, std::min(roi.rows - 30, y_px));
    cv::line(roi, {45, y_px}, {50, y_px}, {120, 120, 120}, 1);
    std::ostringstream yos; yos << std::fixed << std::setprecision(1) << y_val;
    draw_text(roi, yos.str(), {2, y_px + 4}, {140, 140, 140}, 0.32, 1);
  }
  const int n_x_ticks = 4;
  for (int i = 0; i <= n_x_ticks; ++i) {
    double x_val = xmin + (xmax - xmin) * i / n_x_ticks;
    int x_px = static_cast<int>(50 + (x_val - xmin) * sx);
    x_px = std::max(50, std::min(roi.cols - 10, x_px));
    cv::line(roi, {x_px, roi.rows - 30}, {x_px, roi.rows - 25}, {120, 120, 120}, 1);
    std::ostringstream xos; xos << std::fixed << std::setprecision(1) << x_val;
    int baseline = 0;
    cv::Size ts = cv::getTextSize(xos.str(), cv::FONT_HERSHEY_SIMPLEX, 0.32, 1, &baseline);
    draw_text(roi, xos.str(), {x_px - ts.width / 2, roi.rows - 10}, {140, 140, 140}, 0.32, 1);
  }

  for (size_t k = 0; k < series.size(); ++k) {
    if (k < labels.size())
      draw_text(roi, labels[k], {10, 36 + static_cast<int>(k) * 16}, series[k].second, 0.42, 1);
    draw_curve(roi, *series[k].first, series[k].second, ymin, ymax, xmin, xmax, 2, decimate);
  }

  if (!series.empty() && !series.front().first->empty()) {
    std::ostringstream lv; lv << std::fixed << std::setprecision(2);
    for (size_t k = 0; k < series.size(); ++k) {
      if (series[k].first->empty()) continue;
      double v = series[k].first->back().second;
      lv << (k > 0 ? "  " : "")
         << (k < labels.size() ? labels[k] : std::string("s")) << "=" << v;
    }
    int baseline = 0;
    cv::Size ts = cv::getTextSize(lv.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseline);
    draw_text(roi, lv.str(), {roi.cols - 10 - ts.width, 18}, {200, 200, 200}, 0.4, 1);
  }
}

void VizDashboard::draw_waiting_init(cv::Mat &roi, const std::string &title) {
  roi.setTo(cv::Scalar(15, 15, 15));
  cv::rectangle(roi, {0, 0, roi.cols - 1, roi.rows - 1}, {50, 50, 50}, 1);
  draw_text(roi, title, {10, 18}, {220, 220, 220}, 0.5, 1);
  draw_text(roi, "waiting init...", {10, roi.rows / 2}, {120, 120, 120}, 0.5, 1);
}

// ============================================================
// render_and_show
// ============================================================

bool VizDashboard::render_and_show(int wait_ms) {
  auto t_render_start = std::chrono::steady_clock::now();

  // --- 1. Snapshot all render data under lock, then release ---
  // This allows update_* calls (and the estimator) to proceed without
  // waiting for the entire render to complete.
  DrawSnap snap;
  {
    std::lock_guard<std::mutex> lk(mu_);
    snap.vio_hist         = vio_hist_;
    snap.gt_hist          = gt_hist_;
    snap.slam_pts         = slam_pts_;
    snap.msckf_pts        = msckf_pts_;
    snap.image            = latest_image_.empty() ? cv::Mat{} : latest_image_.clone();
    snap.image_t          = latest_image_t_;
    snap.initialized      = initialized_;
    snap.aligned          = aligned_;
    snap.R_gv             = R_gv_;
    snap.t_gv             = t_gv_;
    snap.latest_t         = latest_t_;
    snap.latest_R_wi      = latest_R_wi_;
    snap.latest_p_wi      = latest_p_wi_;
    snap.ts_ate           = ts_ate_;
    snap.ts_speed         = ts_speed_;
    snap.ts_z_vio_raw     = ts_z_vio_raw_;
    snap.ts_z_gt          = ts_z_gt_;
    snap.ts_roll          = ts_roll_;
    snap.ts_pitch         = ts_pitch_;
    snap.ts_yaw           = ts_yaw_;
    snap.ts_slam_count    = ts_slam_count_;
    snap.ts_msckf_count   = ts_msckf_count_;
    snap.ts_tracked_count = ts_tracked_count_;
    snap.ts_klt_raw       = ts_klt_raw_;
    snap.ts_chi2_rejected = ts_chi2_rejected_;
    snap.ts_bg_norm       = ts_bg_norm_;
    snap.ts_ba_norm       = ts_ba_norm_;
    snap.ts_gps_res       = ts_gps_res_;
    snap.ts_gps_kpz       = ts_gps_kpz_;
  }
  // Lock released — all rendering below is lock-free.

  // --- 2. Compute decimation factors from fast mode and data sizes ---
  // traj_step: draw every Nth trajectory segment (target ~800 drawn segments)
  // curve_dec: draw every Nth time-series point  (target ~500 drawn points per curve)
  int traj_step = 1;
  int curve_dec = 1;
  if (opts_.fast) {
    traj_step = std::max(1, static_cast<int>(snap.vio_hist.size()) / 800);
    curve_dec = std::max(1, static_cast<int>(opts_.timeseries_max) / 500);
    // Cap SLAM/MSCKF feature points to avoid O(N) circle draws
    if (snap.slam_pts.size() > 200)
      snap.slam_pts.resize(200);
    if (snap.msckf_pts.size() > 200)
      snap.msckf_pts.resize(200);
  }

  // --- 3. Layout (unchanged from original) ---
  canvas_.setTo(cv::Scalar(0, 0, 0));
  const int W = opts_.width;
  const int H = opts_.height;
  const int row1_h    = H * 420 / 1080;
  const int row_ts_h  = (H - row1_h) / 3;
  const int row2_h    = row_ts_h;
  const int row3_h    = row_ts_h;
  const int row4_h    = H - row1_h - row2_h - row3_h;
  const int col_w     = W / 3;

  // Row 1: 3D iso | Top-down XY | Camera
  cv::Mat r1c1 = canvas_(cv::Rect(0,         0, col_w,         row1_h));
  cv::Mat r1c2 = canvas_(cv::Rect(col_w,     0, col_w,         row1_h));
  cv::Mat r1c3 = canvas_(cv::Rect(2 * col_w, 0, W - 2 * col_w, row1_h));
  draw_trajectory_iso(r1c1, snap, traj_step);
  draw_trajectory_topdown(r1c2, snap, traj_step);
  draw_camera_image(r1c3, snap);

  // Row 2: ATE | Speed | Altitude
  int y2 = row1_h;
  cv::Mat r2c1 = canvas_(cv::Rect(0,         y2, col_w,         row2_h));
  cv::Mat r2c2 = canvas_(cv::Rect(col_w,     y2, col_w,         row2_h));
  cv::Mat r2c3 = canvas_(cv::Rect(2 * col_w, y2, W - 2 * col_w, row2_h));
  draw_panel_xy(r2c1, "ATE (m)",
                {{&snap.ts_ate, {0, 200, 255}}}, {"|p_gt-p_vio|"}, false, snap.initialized, curve_dec);
  draw_panel_xy(r2c2, "Speed (m/s)",
                {{&snap.ts_speed, {0, 255, 120}}}, {"|v|"}, false, snap.initialized, curve_dec);
  draw_panel_xy(r2c3, "Altitude Z (m)",
                {{&snap.ts_z_vio_raw, {255, 160, 0}}, {&snap.ts_z_gt, {0, 200, 0}}},
                {"vio_z", "gt_z"}, false, snap.initialized, curve_dec);

  // Row 3: Roll | Pitch | Yaw
  int y3 = row1_h + row2_h;
  cv::Mat r3c1 = canvas_(cv::Rect(0,         y3, col_w,         row3_h));
  cv::Mat r3c2 = canvas_(cv::Rect(col_w,     y3, col_w,         row3_h));
  cv::Mat r3c3 = canvas_(cv::Rect(2 * col_w, y3, W - 2 * col_w, row3_h));
  draw_panel_xy(r3c1, "Roll (deg)",  {{&snap.ts_roll,  {0, 0, 255}}}, {"roll"},  true, snap.initialized, curve_dec);
  draw_panel_xy(r3c2, "Pitch (deg)", {{&snap.ts_pitch, {0, 255, 0}}}, {"pitch"}, true, snap.initialized, curve_dec);
  draw_panel_xy(r3c3, "Yaw (deg)",   {{&snap.ts_yaw,   {255, 100, 0}}}, {"yaw"}, true, snap.initialized, curve_dec);

  // Row 4: Feature Counts | IMU Biases | Backend/GPS Diag
  int y4 = row1_h + row2_h + row3_h;
  cv::Mat r4c1 = canvas_(cv::Rect(0,         y4, col_w,         row4_h));
  cv::Mat r4c2 = canvas_(cv::Rect(col_w,     y4, col_w,         row4_h));
  cv::Mat r4c3 = canvas_(cv::Rect(2 * col_w, y4, W - 2 * col_w, row4_h));
  draw_panel_xy(r4c1, "Feature Counts",
                {{&snap.ts_klt_raw,      {0, 200, 255}},
                 {&snap.ts_tracked_count,{0, 255, 120}},
                 {&snap.ts_slam_count,   {0, 0, 230}},
                 {&snap.ts_msckf_count,  {255, 255, 255}}},
                {"klt_raw", "tracked", "slam", "msckf"}, false, snap.initialized, curve_dec);
  draw_panel_xy(r4c2, "IMU Biases norm",
                {{&snap.ts_bg_norm, {255, 160, 0}}, {&snap.ts_ba_norm, {0, 200, 255}}},
                {"bg rad/s", "ba m/s^2"}, false, snap.initialized, curve_dec);
  draw_panel_xy(r4c3, "Backend/GPS Diag",
                {{&snap.ts_chi2_rejected, {0, 0, 230}},
                 {&snap.ts_gps_res,       {0, 255, 0}},
                 {&snap.ts_gps_kpz,       {0, 160, 255}}},
                {"chi2_rej", "gps_res_m", "Kpz"}, false, snap.initialized, curve_dec);

  // dividers
  cv::line(canvas_, {col_w, 0},         {col_w, H},         {80, 80, 80}, 1);
  cv::line(canvas_, {2 * col_w, 0},     {2 * col_w, H},     {80, 80, 80}, 1);
  cv::line(canvas_, {0, row1_h},        {W, row1_h},        {80, 80, 80}, 1);
  cv::line(canvas_, {0, row1_h + row2_h}, {W, row1_h + row2_h}, {80, 80, 80}, 1);
  cv::line(canvas_, {0, row1_h + row2_h + row3_h}, {W, row1_h + row2_h + row3_h}, {80, 80, 80}, 1);

  if (video_.isOpened())
    video_.write(canvas_);

  if (opts_.show_window) {
    cv::imshow(opts_.window_title, canvas_);
    int k = cv::waitKey(std::max(1, wait_ms));
    if (k == 'q' || k == 27)
      return false;
  }

  // --- 4. Render timing ---
  last_render_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_render_start).count();
  render_count_++;
  if (render_count_ % 50 == 0) {
    PRINT_DEBUG(CYAN "[viz] render #%d: %.1f ms  fast=%d  traj_step=%d  curve_dec=%d  "
                "snap_vio=%zu  snap_ts=%zu\n" RESET,
                render_count_, last_render_ms_,
                (int)opts_.fast, traj_step, curve_dec,
                snap.vio_hist.size(), snap.ts_speed.size());
  }

  return true;
}

} // namespace ov_msckf
