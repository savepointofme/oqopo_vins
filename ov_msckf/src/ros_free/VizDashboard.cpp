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

  // [中文] 时序: 速度范数、roll/pitch/yaw
  Eigen::Vector3d rpy = quat_to_rpy(R_wi) * 180.0 / M_PI;
  double dt = t - t_start_;
  ts_speed_.push_back({dt, v_wi.norm()});
  ts_roll_.push_back({dt, rpy.x()});
  ts_pitch_.push_back({dt, rpy.y()});
  ts_yaw_.push_back({dt, rpy.z()});
  auto trim = [&](std::deque<std::pair<double, double>> &q) {
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  trim(ts_speed_);
  trim(ts_roll_);
  trim(ts_pitch_);
  trim(ts_yaw_);

  // [中文] 如有对齐与 GT, 写 ATE / xyz 差
  if (aligned_ && has_gt_) {
    Eigen::Vector3d p_vio_in_gt = R_gv_ * p_wi + t_gv_;
    Eigen::Vector3d d = latest_p_gt_ - p_vio_in_gt;
    ts_ate_.push_back({dt, d.norm()});
    ts_dx_.push_back({dt, d.x()});
    ts_dy_.push_back({dt, d.y()});
    ts_dz_.push_back({dt, d.z()});
    trim(ts_ate_);
    trim(ts_dx_);
    trim(ts_dy_);
    trim(ts_dz_);
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

void VizDashboard::draw_trajectory(cv::Mat &roi) {
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
    // [中文] 画面 x 向右, 世界 x 向右; 世界 y 向上 -> 屏幕 y 取反
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
  // MSCKF temp features (white) -- per-frame overlay (implicit "lost后消失")
  for (const auto &p : msckf_pts_) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  }
  // Current camera pose arrow
  if (latest_t_ > 0) {
    Eigen::Vector3d pg = tx(latest_p_wi_);
    // 朝向: VIO frame 下相机 body x 方向, 经 R_gv_ 变换
    Eigen::Vector3d fwd_vio = latest_R_wi_ * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d fwd_gt = aligned_ ? (R_gv_ * fwd_vio) : fwd_vio;
    double yaw = std::atan2(fwd_gt.y(), fwd_gt.x());
    cv::Point p0 = to_px(pg.x(), pg.y());
    double L = 0.6 * s; // 单位: 像素 (0.6m 物理长度)
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
  // Legend / title
  draw_text(roi, "TOP-DOWN XY (m)", {10, 20}, {200, 200, 200}, 0.55, 1);
  draw_text(roi, "GT green", {10, 40}, {0, 200, 0}, 0.45, 1);
  draw_text(roi, "VIO aligned", {10, 58}, {255, 160, 0}, 0.45, 1);
  draw_text(roi, "SLAM (red)  MSCKF temp (white)", {10, 76}, {220, 220, 220}, 0.45, 1);
  {
    std::ostringstream os;
    os << "align=" << (aligned_ ? "on" : "off");
    draw_text(roi, os.str(), {10, 94}, aligned_ ? cv::Scalar(0, 255, 0) : cv::Scalar(120, 120, 120),
              0.45, 1);
  }
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
  {
    std::ostringstream os;
    os << "CAM  t=" << std::fixed << std::setprecision(3) << latest_image_t_
       << "  src=" << latest_image_.cols << "x" << latest_image_.rows;
    draw_text(roi, os.str(), {10, 24}, {200, 200, 200}, 0.55, 1);
  }
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

void VizDashboard::draw_errors(cv::Mat &roi) {
  roi.setTo(cv::Scalar(15, 15, 15));
  // [中文] 子图分四列: ATE | speed | dx/dy/dz 合图
  auto x_bounds = [&](const std::deque<std::pair<double, double>> &q, double &xmin, double &xmax) {
    if (q.empty()) {
      xmin = 0;
      xmax = 1;
      return;
    }
    xmin = q.front().first;
    xmax = std::max(q.back().first, xmin + 1.0);
  };
  auto y_bounds = [&](const std::deque<std::pair<double, double>> &q, double pad, double &ymin,
                      double &ymax) {
    ymin = 0;
    ymax = 1;
    if (q.empty())
      return;
    ymin = std::numeric_limits<double>::infinity();
    ymax = -ymin;
    for (const auto &p : q) {
      ymin = std::min(ymin, p.second);
      ymax = std::max(ymax, p.second);
    }
    if (!(ymax > ymin)) {
      ymax = ymin + 1;
    }
    double r = ymax - ymin;
    ymin -= pad * r;
    ymax += pad * r;
  };
  int sub_w = roi.cols / 3;
  auto draw_panel = [&](cv::Rect r, const std::string &title,
                        const std::vector<std::pair<std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
                        const std::vector<std::string> &labels) {
    cv::Mat sub = roi(r);
    cv::rectangle(sub, {0, 0, sub.cols - 1, sub.rows - 1}, {50, 50, 50}, 1);
    draw_text(sub, title, {10, 20}, {220, 220, 220}, 0.5, 1);
    if (series.empty())
      return;
    double xmin, xmax, ymin, ymax;
    x_bounds(*series[0].first, xmin, xmax);
    ymin = std::numeric_limits<double>::infinity();
    ymax = -ymin;
    for (const auto &s : series) {
      double a, b;
      y_bounds(*s.first, 0.1, a, b);
      ymin = std::min(ymin, a);
      ymax = std::max(ymax, b);
    }
    if (!std::isfinite(ymin)) {
      ymin = 0;
      ymax = 1;
    }
    // axes
    cv::line(sub, {50, sub.rows - 30}, {sub.cols - 10, sub.rows - 30}, {90, 90, 90}, 1);
    cv::line(sub, {50, 30}, {50, sub.rows - 30}, {90, 90, 90}, 1);
    {
      std::ostringstream os;
      os << std::fixed << std::setprecision(2) << "y:[" << ymin << "," << ymax << "]";
      draw_text(sub, os.str(), {sub.cols / 2, 20}, {160, 160, 160}, 0.4, 1);
    }
    for (size_t k = 0; k < series.size(); ++k) {
      draw_curve(sub, *series[k].first, series[k].second, ymin, ymax, xmin, xmax, 2);
      if (k < labels.size())
        draw_text(sub, labels[k], {10, 40 + static_cast<int>(k) * 16}, series[k].second, 0.45, 1);
    }
  };
  draw_panel({0, 0, sub_w, roi.rows}, "ATE (m)", {{&ts_ate_, {0, 200, 255}}}, {"|p_gt - p_vio|"});
  draw_panel({sub_w, 0, sub_w, roi.rows}, "Speed (m/s)", {{&ts_speed_, {0, 255, 120}}}, {"|v|"});
  draw_panel({2 * sub_w, 0, roi.cols - 2 * sub_w, roi.rows}, "GT - VIO xyz (m)",
             {{&ts_dx_, {0, 0, 255}}, {&ts_dy_, {0, 255, 0}}, {&ts_dz_, {255, 0, 0}}},
             {"dx", "dy", "dz"});
}

void VizDashboard::draw_attitude(cv::Mat &roi) {
  roi.setTo(cv::Scalar(15, 15, 15));
  double xmin = 0, xmax = 1;
  if (!ts_roll_.empty()) {
    xmin = ts_roll_.front().first;
    xmax = std::max(ts_roll_.back().first, xmin + 1.0);
  }
  double ymin = -180, ymax = 180;
  cv::rectangle(roi, {0, 0, roi.cols - 1, roi.rows - 1}, {50, 50, 50}, 1);
  draw_text(roi, "Attitude (deg): roll=red pitch=green yaw=blue", {10, 20}, {220, 220, 220}, 0.5, 1);
  cv::line(roi, {50, roi.rows - 30}, {roi.cols - 10, roi.rows - 30}, {90, 90, 90}, 1);
  cv::line(roi, {50, 30}, {50, roi.rows - 30}, {90, 90, 90}, 1);
  draw_curve(roi, ts_roll_, cv::Scalar(0, 0, 255), ymin, ymax, xmin, xmax, 2);
  draw_curve(roi, ts_pitch_, cv::Scalar(0, 255, 0), ymin, ymax, xmin, xmax, 2);
  draw_curve(roi, ts_yaw_, cv::Scalar(255, 100, 0), ymin, ymax, xmin, xmax, 2);
  // numeric readout
  double r = ts_roll_.empty() ? 0 : ts_roll_.back().second;
  double p = ts_pitch_.empty() ? 0 : ts_pitch_.back().second;
  double y = ts_yaw_.empty() ? 0 : ts_yaw_.back().second;
  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << "r=" << r << "  p=" << p << "  y=" << y;
  draw_text(roi, os.str(), {roi.cols / 2 - 80, 20}, {180, 180, 200}, 0.5, 1);
  // header info
  if (latest_t_ > 0) {
    std::ostringstream os2;
    os2 << std::fixed << std::setprecision(3) << "t=" << (latest_t_ - t_start_) << "s  speed="
        << latest_v_wi_.norm() << "m/s  p=[" << latest_p_wi_.x() << "," << latest_p_wi_.y() << ","
        << latest_p_wi_.z() << "]";
    draw_text(roi, os2.str(), {10, roi.rows - 10}, {180, 180, 180}, 0.45, 1);
  }
}

bool VizDashboard::render_and_show(int wait_ms) {
  std::lock_guard<std::mutex> lk(mu_);
  canvas_.setTo(cv::Scalar(0, 0, 0));
  const int tl_w = opts_.width / 2, tl_h = 500;
  const int b_h = opts_.height - tl_h;
  cv::Mat roi_tl = canvas_(cv::Rect(0, 0, tl_w, tl_h));
  cv::Mat roi_tr = canvas_(cv::Rect(tl_w, 0, opts_.width - tl_w, tl_h));
  cv::Mat roi_bl = canvas_(cv::Rect(0, tl_h, opts_.width / 2, b_h));
  cv::Mat roi_br = canvas_(cv::Rect(opts_.width / 2, tl_h, opts_.width - opts_.width / 2, b_h));
  draw_trajectory(roi_tl);
  draw_camera_image(roi_tr);
  draw_errors(roi_bl);
  draw_attitude(roi_br);
  // divider
  cv::line(canvas_, {tl_w, 0}, {tl_w, tl_h}, {80, 80, 80}, 1);
  cv::line(canvas_, {0, tl_h}, {opts_.width, tl_h}, {80, 80, 80}, 1);
  cv::line(canvas_, {opts_.width / 2, tl_h}, {opts_.width / 2, opts_.height}, {80, 80, 80}, 1);

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
