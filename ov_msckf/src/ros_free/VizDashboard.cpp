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
#include <cmath>
#include <iomanip>
#include <sstream>

#include "utils/colors.h"
#include "utils/print.h"

namespace ov_msckf {

namespace {
// [中文] 等距 3D 投影矩阵: azim=45°, elev=-30° (从右上俯视).
// p_screen2 = R * p_world, 取 (x', y') 二维分量正交投影.
// 屏幕约定: x' 向右, y' 向下 (绘制时取负).
Eigen::Matrix3d isometric_R() {
  const double az = 45.0 * M_PI / 180.0;
  const double el = -30.0 * M_PI / 180.0; // 从上方往下看负 elev
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

  // [中文] 时序: 速度范数、roll/pitch/yaw、altitude (VIO frame z, 对齐后投到 GT 才比较)
  Eigen::Vector3d rpy = quat_to_rpy(R_wi) * 180.0 / M_PI;
  double dt = t - t_start_;
  ts_speed_.push_back({dt, v_wi.norm()});
  ts_roll_.push_back({dt, rpy.x()});
  ts_pitch_.push_back({dt, rpy.y()});
  ts_yaw_.push_back({dt, rpy.z()});
  // VIO altitude in GT frame (after alignment)
  Eigen::Vector3d p_in_gt = aligned_ ? (R_gv_ * p_wi + t_gv_) : p_wi;
  ts_z_vio_.push_back({dt, p_in_gt.z()});
  auto trim = [&](std::deque<std::pair<double, double>> &q) {
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  trim(ts_speed_);
  trim(ts_roll_);
  trim(ts_pitch_);
  trim(ts_yaw_);
  trim(ts_z_vio_);

  // [中文] 如有对齐与 GT, 写 ATE
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
  // GT altitude time series (keyed on dt for axis sync with VIO)
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
}

void VizDashboard::set_initialized(bool initialized) {
  std::lock_guard<std::mutex> lk(mu_);
  initialized_ = initialized;
}

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
  // [中文] ZYX (yaw-pitch-roll) 欧拉角, 返回 (roll, pitch, yaw) 弧度
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

void VizDashboard::draw_trajectory_topdown(cv::Mat &roi) {
  roi.setTo(cv::Scalar(20, 20, 20));
  draw_grid(roi, cv::Scalar(40, 40, 40), 40);
  // [中文] 自适应视野: 取 VIO (对齐后) + GT + features 包围盒
  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto expand = [&](double x, double y) {
    xmin = std::min(xmin, x);
    xmax = std::max(xmax, x);
    ymin = std::min(ymin, y);
    ymax = std::max(ymax, y);
  };
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return aligned_ ? (R_gv_ * p + t_gv_) : p;
  };
  for (const auto &e : vio_hist_) {
    auto pg = tx(e.p);
    expand(pg.x(), pg.y());
  }
  for (const auto &e : gt_hist_)
    expand(e.p.x(), e.p.y());
  for (const auto &p : slam_pts_) {
    auto pg = tx(p);
    expand(pg.x(), pg.y());
  }
  if (!std::isfinite(xmin)) {
    xmin = -5;
    xmax = 5;
    ymin = -5;
    ymax = 5;
  }
  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double s = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](double x, double y) {
    return cv::Point(static_cast<int>((x - cx) * s + roi.cols / 2),
                     static_cast<int>(-(y - cy) * s + roi.rows / 2));
  };

  // GT line (green)
  for (size_t i = 1; i < gt_hist_.size(); ++i) {
    cv::line(roi, to_px(gt_hist_[i - 1].p.x(), gt_hist_[i - 1].p.y()),
             to_px(gt_hist_[i].p.x(), gt_hist_[i].p.y()), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
  }
  // VIO line (cyan/blue)
  for (size_t i = 1; i < vio_hist_.size(); ++i) {
    auto a = tx(vio_hist_[i - 1].p);
    auto b = tx(vio_hist_[i].p);
    cv::line(roi, to_px(a.x(), a.y()), to_px(b.x(), b.y()), cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
  }
  // SLAM features (red)
  for (const auto &p : slam_pts_) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(0, 0, 230), -1, cv::LINE_AA);
  }
  // MSCKF temp features (white) -- per-frame overlay
  for (const auto &p : msckf_pts_) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  }
  // Current camera pose arrow
  if (latest_t_ > 0) {
    Eigen::Vector3d pg = tx(latest_p_wi_);
    Eigen::Vector3d fwd_vio = latest_R_wi_ * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d fwd_gt = aligned_ ? (R_gv_ * fwd_vio) : fwd_vio;
    double yaw = std::atan2(fwd_gt.y(), fwd_gt.x());
    cv::Point p0 = to_px(pg.x(), pg.y());
    double L = 0.6 * s;
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
  draw_text(roi, std::string("align=") + (aligned_ ? "on" : "off"), {10, 94},
            aligned_ ? cv::Scalar(0, 255, 0) : cv::Scalar(120, 120, 120), 0.45, 1);
}

void VizDashboard::draw_trajectory_iso(cv::Mat &roi) {
  roi.setTo(cv::Scalar(18, 18, 22));
  draw_grid(roi, cv::Scalar(38, 38, 42), 40);

  static const Eigen::Matrix3d R_iso = isometric_R();

  // [中文] 计算所有点在 iso 投影平面 (x', y') 的 bounding box
  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return aligned_ ? (R_gv_ * p + t_gv_) : p;
  };
  auto expand = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d s = R_iso * p_world;
    xmin = std::min(xmin, s.x());
    xmax = std::max(xmax, s.x());
    ymin = std::min(ymin, s.y());
    ymax = std::max(ymax, s.y());
  };
  for (const auto &e : vio_hist_)
    expand(tx(e.p));
  for (const auto &e : gt_hist_)
    expand(e.p);
  if (!std::isfinite(xmin)) {
    xmin = -5;
    xmax = 5;
    ymin = -5;
    ymax = 5;
  }
  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double s = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d sp = R_iso * p_world;
    return cv::Point(static_cast<int>((sp.x() - cx) * s + roi.cols / 2),
                     static_cast<int>(-(sp.y() - cy) * s + roi.rows / 2));
  };

  // [中文] 画 iso 坐标轴 (从原点出发, 5m 长度) 作视觉参考
  Eigen::Vector3d ref_origin =
      aligned_ ? (gt_hist_.empty() ? Eigen::Vector3d::Zero() : gt_hist_.front().p)
               : (vio_hist_.empty() ? Eigen::Vector3d::Zero() : vio_hist_.front().p);
  double axis_len = 5.0;
  cv::Point o = to_px(ref_origin);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(axis_len, 0, 0)), {80, 80, 200}, 1,
           cv::LINE_AA); // X red-ish
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, axis_len, 0)), {80, 200, 80}, 1,
           cv::LINE_AA); // Y green
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len)), {200, 80, 80}, 1,
           cv::LINE_AA); // Z blue (BGR: 200,80,80 looks blue-ish)
  draw_text(roi, "X", to_px(ref_origin + Eigen::Vector3d(axis_len * 1.05, 0, 0)),
            {120, 120, 220}, 0.45, 1);
  draw_text(roi, "Y", to_px(ref_origin + Eigen::Vector3d(0, axis_len * 1.05, 0)),
            {120, 220, 120}, 0.45, 1);
  draw_text(roi, "Z", to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len * 1.05)),
            {220, 120, 120}, 0.45, 1);

  // GT line (green)
  for (size_t i = 1; i < gt_hist_.size(); ++i) {
    cv::line(roi, to_px(gt_hist_[i - 1].p), to_px(gt_hist_[i].p), cv::Scalar(0, 200, 0), 2,
             cv::LINE_AA);
  }
  // VIO line (orange/blue)
  for (size_t i = 1; i < vio_hist_.size(); ++i) {
    cv::line(roi, to_px(tx(vio_hist_[i - 1].p)), to_px(tx(vio_hist_[i].p)), cv::Scalar(255, 160, 0),
             2, cv::LINE_AA);
  }
  // current pose marker
  if (latest_t_ > 0) {
    cv::circle(roi, to_px(tx(latest_p_wi_)), 5, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
  }

  draw_text(roi, "3D ISO (azim=45 elev=-30)", {10, 20}, {200, 200, 200}, 0.55, 1);
  draw_text(roi, "GT green   VIO blue", {10, 40}, {200, 200, 200}, 0.45, 1);
}

void VizDashboard::draw_camera_image(cv::Mat &roi) {
  roi.setTo(cv::Scalar(10, 10, 10));
  if (latest_image_.empty()) {
    draw_text(roi, "waiting for camera...", {20, 40}, {180, 180, 180}, 0.6, 1);
    return;
  }
  cv::Mat resized;
  double sx = static_cast<double>(roi.cols) / latest_image_.cols;
  double sy = static_cast<double>(roi.rows - 40) / latest_image_.rows;
  double s = std::min(sx, sy);
  int w = static_cast<int>(latest_image_.cols * s);
  int h = static_cast<int>(latest_image_.rows * s);
  cv::resize(latest_image_, resized, cv::Size(w, h), 0, 0, cv::INTER_AREA);
  int ox = (roi.cols - w) / 2;
  int oy = 40 + (roi.rows - 40 - h) / 2;
  resized.copyTo(roi(cv::Rect(ox, oy, w, h)));
  std::ostringstream os;
  os << "CAM  t=" << std::fixed << std::setprecision(3) << latest_image_t_
     << "  src=" << latest_image_.cols << "x" << latest_image_.rows
     << "  init=" << (initialized_ ? "ok" : "wait");
  draw_text(roi, os.str(), {10, 24}, {200, 200, 200}, 0.55, 1);
}

void VizDashboard::draw_curve(cv::Mat &roi, const std::deque<std::pair<double, double>> &pts,
                              cv::Scalar color, double y_min, double y_max, double x_min,
                              double x_max, int thickness) {
  if (pts.empty() || x_max <= x_min || y_max <= y_min)
    return;
  double sx = (roi.cols - 60) / (x_max - x_min);
  double sy = (roi.rows - 50) / (y_max - y_min);
  auto tx = [&](double x, double y) {
    return cv::Point(static_cast<int>(50 + (x - x_min) * sx),
                     static_cast<int>(roi.rows - 30 - (y - y_min) * sy));
  };
  cv::Point prev;
  bool first = true;
  for (const auto &p : pts) {
    cv::Point q = tx(p.first, std::max(y_min, std::min(y_max, p.second)));
    if (!first)
      cv::line(roi, prev, q, color, thickness, cv::LINE_AA);
    prev = q;
    first = false;
  }
}

void VizDashboard::draw_panel_xy(
    cv::Mat &roi, const std::string &title,
    const std::vector<std::pair<std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
    const std::vector<std::string> &labels, bool symmetric_y) {
  roi.setTo(cv::Scalar(15, 15, 15));
  cv::rectangle(roi, {0, 0, roi.cols - 1, roi.rows - 1}, {50, 50, 50}, 1);
  draw_text(roi, title, {10, 18}, {220, 220, 220}, 0.5, 1);
  bool any_data = false;
  for (auto &s : series)
    if (!s.first->empty()) {
      any_data = true;
      break;
    }
  if (!any_data) {
    draw_text(roi, initialized_ ? "no data" : "waiting init...", {10, roi.rows / 2},
              {120, 120, 120}, 0.5, 1);
    return;
  }
  // x bounds: use min front / max back across series
  double xmin = std::numeric_limits<double>::infinity();
  double xmax = -xmin;
  for (auto &s : series) {
    if (s.first->empty())
      continue;
    xmin = std::min(xmin, s.first->front().first);
    xmax = std::max(xmax, s.first->back().first);
  }
  if (!std::isfinite(xmin) || xmax <= xmin)
    xmax = xmin + 1.0;
  // y bounds
  double ymin = std::numeric_limits<double>::infinity();
  double ymax = -ymin;
  for (auto &s : series) {
    for (const auto &p : *s.first) {
      ymin = std::min(ymin, p.second);
      ymax = std::max(ymax, p.second);
    }
  }
  if (!std::isfinite(ymin)) {
    ymin = 0;
    ymax = 1;
  }
  if (!(ymax > ymin))
    ymax = ymin + 1.0;
  double pad = 0.1 * (ymax - ymin);
  ymin -= pad;
  ymax += pad;
  if (symmetric_y) {
    double m = std::max(std::fabs(ymin), std::fabs(ymax));
    ymin = -m;
    ymax = m;
  }
  // axes
  cv::line(roi, {50, roi.rows - 30}, {roi.cols - 10, roi.rows - 30}, {90, 90, 90}, 1);
  cv::line(roi, {50, 30}, {50, roi.rows - 30}, {90, 90, 90}, 1);
  // y-range readout (right-aligned to avoid title collision)
  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << "[" << ymin << ", " << ymax << "]";
  draw_text(roi, os.str(), {roi.cols - 160, 18}, {160, 160, 160}, 0.4, 1);
  // legend (each label below title, vertical stack)
  for (size_t k = 0; k < series.size(); ++k) {
    if (k < labels.size())
      draw_text(roi, labels[k], {10, 36 + static_cast<int>(k) * 16}, series[k].second, 0.42, 1);
    draw_curve(roi, *series[k].first, series[k].second, ymin, ymax, xmin, xmax, 2);
  }
  // latest value bottom-right
  if (!series.empty() && !series.front().first->empty()) {
    std::ostringstream lv;
    lv << std::fixed << std::setprecision(2);
    for (size_t k = 0; k < series.size(); ++k) {
      if (series[k].first->empty())
        continue;
      double v = series[k].first->back().second;
      lv << (k > 0 ? "  " : "")
         << (k < labels.size() ? labels[k] : std::string("s")) << "=" << v;
    }
    draw_text(roi, lv.str(), {10, roi.rows - 8}, {180, 180, 180}, 0.4, 1);
  }
}

void VizDashboard::draw_waiting_init(cv::Mat &roi, const std::string &title) {
  roi.setTo(cv::Scalar(15, 15, 15));
  cv::rectangle(roi, {0, 0, roi.cols - 1, roi.rows - 1}, {50, 50, 50}, 1);
  draw_text(roi, title, {10, 18}, {220, 220, 220}, 0.5, 1);
  draw_text(roi, "waiting init...", {10, roi.rows / 2}, {120, 120, 120}, 0.5, 1);
}

bool VizDashboard::render_and_show(int wait_ms) {
  std::lock_guard<std::mutex> lk(mu_);
  canvas_.setTo(cv::Scalar(0, 0, 0));

  // [中文] 3 行 3 列布局
  const int W = opts_.width;
  const int H = opts_.height;
  const int row1_h = H * 540 / 1080; // 540/1080
  const int row2_h = (H - row1_h) / 2;
  const int row3_h = H - row1_h - row2_h;
  const int col_w = W / 3;

  // Row 1
  cv::Mat r1c1 = canvas_(cv::Rect(0, 0, col_w, row1_h));
  cv::Mat r1c2 = canvas_(cv::Rect(col_w, 0, col_w, row1_h));
  cv::Mat r1c3 = canvas_(cv::Rect(2 * col_w, 0, W - 2 * col_w, row1_h));
  draw_trajectory_iso(r1c1);
  draw_trajectory_topdown(r1c2);
  draw_camera_image(r1c3);

  // Row 2: ATE | Speed | Altitude
  int y2 = row1_h;
  cv::Mat r2c1 = canvas_(cv::Rect(0, y2, col_w, row2_h));
  cv::Mat r2c2 = canvas_(cv::Rect(col_w, y2, col_w, row2_h));
  cv::Mat r2c3 = canvas_(cv::Rect(2 * col_w, y2, W - 2 * col_w, row2_h));
  draw_panel_xy(r2c1, "ATE (m)", {{&ts_ate_, {0, 200, 255}}}, {"|p_gt-p_vio|"}, false);
  draw_panel_xy(r2c2, "Speed (m/s)", {{&ts_speed_, {0, 255, 120}}}, {"|v|"}, false);
  draw_panel_xy(r2c3, "Altitude Z (m)",
                {{&ts_z_vio_, {255, 160, 0}}, {&ts_z_gt_, {0, 200, 0}}}, {"vio_z", "gt_z"}, false);

  // Row 3: roll | pitch | yaw
  int y3 = row1_h + row2_h;
  cv::Mat r3c1 = canvas_(cv::Rect(0, y3, col_w, row3_h));
  cv::Mat r3c2 = canvas_(cv::Rect(col_w, y3, col_w, row3_h));
  cv::Mat r3c3 = canvas_(cv::Rect(2 * col_w, y3, W - 2 * col_w, row3_h));
  draw_panel_xy(r3c1, "Roll (deg)", {{&ts_roll_, {0, 0, 255}}}, {"roll"}, true);
  draw_panel_xy(r3c2, "Pitch (deg)", {{&ts_pitch_, {0, 255, 0}}}, {"pitch"}, true);
  draw_panel_xy(r3c3, "Yaw (deg)", {{&ts_yaw_, {255, 100, 0}}}, {"yaw"}, true);

  // dividers
  cv::line(canvas_, {col_w, 0}, {col_w, H}, {80, 80, 80}, 1);
  cv::line(canvas_, {2 * col_w, 0}, {2 * col_w, H}, {80, 80, 80}, 1);
  cv::line(canvas_, {0, row1_h}, {W, row1_h}, {80, 80, 80}, 1);
  cv::line(canvas_, {0, row1_h + row2_h}, {W, row1_h + row2_h}, {80, 80, 80}, 1);

  if (video_.isOpened())
    video_.write(canvas_);

  if (opts_.show_window) {
    cv::imshow("OpenVINS ROS-free Dashboard", canvas_);
    int k = cv::waitKey(std::max(1, wait_ms));
    if (k == 'q' || k == 27)
      return false;
  }
  return true;
}

} // namespace ov_msckf
