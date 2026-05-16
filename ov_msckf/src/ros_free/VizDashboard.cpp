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
Eigen::Matrix3d isometric_R() {
  const double az = 45.0 * M_PI / 180.0;
  const double el = -30.0 * M_PI / 180.0;
  Eigen::Matrix3d Rz, Rx;
  Rz << std::cos(az), -std::sin(az), 0, std::sin(az), std::cos(az), 0, 0, 0, 1;
  Rx << 1, 0, 0, 0, std::cos(el), -std::sin(el), 0, std::sin(el), std::cos(el);
  return Rx * Rz;
}
} // namespace

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

VizDashboard::VizDashboard(const Options &opts) : opts_(opts) {
  render_buf_ = cv::Mat::zeros(opts_.height, opts_.width, CV_8UC3);
  // display_mat_ must also be pre-allocated: after the first swap render_buf_
  // becomes the old display_mat_, and render_frame() will be called on it.
  // An empty (0×0) mat causes an assertion in cv::Mat::operator()(cv::Rect).
  display_mat_ = cv::Mat::zeros(opts_.height, opts_.width, CV_8UC3);

  if (!opts_.video_path.empty()) {
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    video_.open(opts_.video_path, fourcc, opts_.video_fps, cv::Size(opts_.width, opts_.height), true);
    if (!video_.isOpened()) {
      PRINT_WARNING(YELLOW "[viz] failed to open video writer: %s\n" RESET, opts_.video_path.c_str());
    } else {
      PRINT_INFO(GREEN "[viz] writing video to %s @ %d fps\n" RESET, opts_.video_path.c_str(), opts_.video_fps);
    }
  }

  render_running_.store(true);
  render_thread_ = std::thread(&VizDashboard::render_loop, this);
}

VizDashboard::~VizDashboard() {
  render_running_.store(false);
  if (render_thread_.joinable())
    render_thread_.join();
  if (video_.isOpened())
    video_.release();
}

// ---------------------------------------------------------------------------
// Data update methods (estimator thread)
// ---------------------------------------------------------------------------

void VizDashboard::update_vio_pose(double t, const Eigen::Matrix3d &R_wi, const Eigen::Vector3d &p_wi,
                                   const Eigen::Vector3d &v_wi) {
  std::lock_guard<std::mutex> lk(mu_);
  data_.latest_t = t;
  data_.latest_R_wi_prev = data_.latest_R_wi; // previous frame rotation for frame-to-frame warp
  data_.latest_R_wi = R_wi;
  data_.latest_p_wi = p_wi;
  data_.latest_v_wi = v_wi;
  if (data_.t_start < 0)
    data_.t_start = t;
  data_.vio_hist.push_back({t, p_wi});
  while (data_.vio_hist.size() > opts_.max_history)
    data_.vio_hist.pop_front();

  Eigen::Vector3d rpy = quat_to_rpy(R_wi) * 180.0 / M_PI;
  double dt = t - data_.t_start;
  data_.ts_speed.push_back({dt, v_wi.norm()});
  data_.ts_roll.push_back({dt, rpy.x()});
  data_.ts_pitch.push_back({dt, rpy.y()});
  data_.ts_yaw.push_back({dt, rpy.z()});
  Eigen::Vector3d p_in_gt = data_.aligned ? (data_.R_gv * p_wi + data_.t_gv) : p_wi;
  data_.ts_z_vio.push_back({dt, p_in_gt.z()});

  auto trim = [&](std::deque<std::pair<double, double>> &q) {
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  trim(data_.ts_speed);
  trim(data_.ts_roll);
  trim(data_.ts_pitch);
  trim(data_.ts_yaw);
  trim(data_.ts_z_vio);

  if (data_.aligned && data_.has_gt) {
    Eigen::Vector3d p_vio_in_gt = data_.R_gv * p_wi + data_.t_gv;
    Eigen::Vector3d d = data_.latest_p_gt - p_vio_in_gt;
    data_.ts_ate.push_back({dt, d.norm()});
    trim(data_.ts_ate);
  }
}

void VizDashboard::set_alignment(const Eigen::Matrix3d &R_gv, const Eigen::Vector3d &t_gv, bool solved) {
  std::lock_guard<std::mutex> lk(mu_);
  data_.R_gv = R_gv;
  data_.t_gv = t_gv;
  data_.aligned = solved;
}

void VizDashboard::update_gt(double t, const Eigen::Vector3d &p_gt) {
  std::lock_guard<std::mutex> lk(mu_);
  data_.gt_hist.push_back({t, p_gt});
  while (data_.gt_hist.size() > opts_.max_history)
    data_.gt_hist.pop_front();
  data_.latest_p_gt = p_gt;
  data_.has_gt = true;
  if (data_.t_start > 0) {
    double dt = t - data_.t_start;
    data_.ts_z_gt.push_back({dt, p_gt.z()});
    while (data_.ts_z_gt.size() > opts_.timeseries_max)
      data_.ts_z_gt.pop_front();
  }
}

void VizDashboard::update_image(double t, const cv::Mat &img) {
  if (img.empty())
    return;
  // Always create a fresh Mat so the render thread's snapshot ref stays valid
  // even after this method is called again.
  cv::Mat fresh;
  if (img.channels() == 1)
    cv::cvtColor(img, fresh, cv::COLOR_GRAY2BGR);
  else
    fresh = img.clone();
  std::lock_guard<std::mutex> lk(mu_);
  data_.latest_image = fresh;
  data_.latest_image_t = t;
}

void VizDashboard::update_tracker_flow(const cv::Mat &warped_img, const std::vector<cv::Point2f> &prev_pts,
                                       const std::vector<cv::Point2f> &curr_pts, const cv::Mat &curr_raw,
                                       bool warp_active, double t_curr) {
  if (warped_img.empty() || curr_raw.empty())
    return;
  std::lock_guard<std::mutex> lk(mu_);
  data_.flow_warped_img = warped_img.clone();
  data_.flow_prev_pts = prev_pts;
  data_.flow_curr_pts = curr_pts;
  data_.flow_curr_raw = curr_raw.clone();
  data_.flow_warp_active = warp_active;
  data_.flow_valid = true;
  data_.flow_t_curr = t_curr;
}

void VizDashboard::update_features(const std::vector<Eigen::Vector3d> &slam_pts,
                                   const std::vector<Eigen::Vector3d> &msckf_pts) {
  std::lock_guard<std::mutex> lk(mu_);
  data_.slam_pts = slam_pts;
  data_.msckf_pts = msckf_pts;
}

void VizDashboard::set_initialized(bool initialized) {
  std::lock_guard<std::mutex> lk(mu_);
  data_.initialized = initialized;
}

void VizDashboard::update_biases(double t, const Eigen::Vector3d &bg, const Eigen::Vector3d &ba) {
  std::lock_guard<std::mutex> lk(mu_);
  if (data_.t_start < 0)
    return; // t_start is set by update_vio_pose; skip until then
  double dt = t - data_.t_start;
  auto push = [&](std::deque<std::pair<double, double>> &q, double v) {
    q.push_back({dt, v});
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  push(data_.ts_bg_x, bg.x()); push(data_.ts_bg_y, bg.y()); push(data_.ts_bg_z, bg.z());
  push(data_.ts_ba_x, ba.x()); push(data_.ts_ba_y, ba.y()); push(data_.ts_ba_z, ba.z());
}

void VizDashboard::update_gps_alt_diag(double t, double gps_z, double vio_z,
                                        double residual, double pzz, double kpz) {
  std::lock_guard<std::mutex> lk(mu_);
  if (data_.t_start < 0)
    return;
  double dt = t - data_.t_start;
  auto push = [&](std::deque<std::pair<double, double>> &q, double v) {
    q.push_back({dt, v});
    while (q.size() > opts_.timeseries_max)
      q.pop_front();
  };
  push(data_.ts_gps_z, gps_z);
  push(data_.ts_gps_vio_z, vio_z);
  push(data_.ts_gps_res, residual);
  push(data_.ts_gps_pzz, pzz);
  push(data_.ts_gps_kpz, kpz);
  data_.has_gps_diag = true;
}

// ---------------------------------------------------------------------------
// Public display interface (main thread)
// ---------------------------------------------------------------------------

bool VizDashboard::show_frame() {
  if (!opts_.show_window)
    return true;
  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lk(display_mu_);
    if (display_mat_.empty())
      return true;
    frame = display_mat_.clone();
  }
  // namedWindow before imshow so the window is recreated if external code
  // called destroyAllWindows (e.g. tracker debug visualization cleanup).
  cv::namedWindow("OpenVINS ROS-free Dashboard", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
  cv::imshow("OpenVINS ROS-free Dashboard", frame);
  int k = cv::waitKey(1);
  return !(k == 'q' || k == 27);
}

bool VizDashboard::render_and_show(int /*wait_ms*/) { return show_frame(); }

cv::Mat VizDashboard::last_canvas() const {
  std::lock_guard<std::mutex> lk(display_mu_);
  return display_mat_.clone();
}

// ---------------------------------------------------------------------------
// Render thread
// ---------------------------------------------------------------------------

void VizDashboard::render_loop() {
  using clock = std::chrono::steady_clock;
  const auto interval = std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(1.0 / std::max(1.0, opts_.render_fps)));

  auto next = clock::now() + interval;

  while (render_running_.load()) {
    // Snapshot: hold mu_ only long enough to copy data (no rendering under lock)
    DataState snap;
    {
      std::lock_guard<std::mutex> lk(mu_);
      snap.vio_hist = data_.vio_hist;
      snap.gt_hist = data_.gt_hist;
      snap.R_gv = data_.R_gv;
      snap.t_gv = data_.t_gv;
      snap.aligned = data_.aligned;
      snap.initialized = data_.initialized;
      snap.latest_t = data_.latest_t;
      snap.latest_R_wi = data_.latest_R_wi;
      snap.latest_R_wi_prev = data_.latest_R_wi_prev;
      snap.latest_p_wi = data_.latest_p_wi;
      snap.latest_v_wi = data_.latest_v_wi;
      snap.latest_p_gt = data_.latest_p_gt;
      snap.has_gt = data_.has_gt;
      snap.latest_image = data_.latest_image; // shallow copy; always a distinct allocation
      snap.latest_image_t = data_.latest_image_t;
      snap.flow_warped_img = data_.flow_warped_img;
      snap.flow_prev_pts = data_.flow_prev_pts;
      snap.flow_curr_pts = data_.flow_curr_pts;
      snap.flow_curr_raw = data_.flow_curr_raw;
      snap.flow_warp_active = data_.flow_warp_active;
      snap.flow_valid = data_.flow_valid;
      snap.flow_t_curr = data_.flow_t_curr;
      snap.slam_pts = data_.slam_pts;
      snap.msckf_pts = data_.msckf_pts;
      snap.ts_ate = data_.ts_ate;
      snap.ts_speed = data_.ts_speed;
      snap.ts_z_vio = data_.ts_z_vio;
      snap.ts_z_gt = data_.ts_z_gt;
      snap.ts_roll = data_.ts_roll;
      snap.ts_pitch = data_.ts_pitch;
      snap.ts_yaw = data_.ts_yaw;
      snap.ts_gps_z = data_.ts_gps_z;
      snap.ts_gps_vio_z = data_.ts_gps_vio_z;
      snap.ts_gps_res = data_.ts_gps_res;
      snap.ts_gps_pzz = data_.ts_gps_pzz;
      snap.ts_gps_kpz = data_.ts_gps_kpz;
      snap.has_gps_diag = data_.has_gps_diag;
      snap.ts_bg_x = data_.ts_bg_x; snap.ts_bg_y = data_.ts_bg_y; snap.ts_bg_z = data_.ts_bg_z;
      snap.ts_ba_x = data_.ts_ba_x; snap.ts_ba_y = data_.ts_ba_y; snap.ts_ba_z = data_.ts_ba_z;
      snap.t_start = data_.t_start;
    }

    // Render into private buffer — no lock held
    render_frame(render_buf_, snap);

    // Write video frame before swap
    if (video_.isOpened())
      video_.write(render_buf_);

    // Publish to display buffer
    {
      std::lock_guard<std::mutex> lk(display_mu_);
      std::swap(render_buf_, display_mat_);
      // After swap: display_mat_ = fresh render, render_buf_ = old display (reused next cycle)
    }

    // Rate-limit: sleep until next scheduled wake
    auto now = clock::now();
    if (next < now - interval)
      next = now; // clamp on lag to avoid burst catch-up
    std::this_thread::sleep_until(next);
    next += interval;
  }
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

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

void VizDashboard::draw_curve(cv::Mat &roi, const std::deque<std::pair<double, double>> &pts,
                              cv::Scalar color, double y_min, double y_max, double x_min,
                              double x_max, int thickness) {
  if (pts.empty() || x_max <= x_min || y_max <= y_min)
    return;
  double sx = (roi.cols - 60) / (x_max - x_min);
  double sy = (roi.rows - 50) / (y_max - y_min);
  auto to_px = [&](double x, double y) {
    return cv::Point(static_cast<int>(50 + (x - x_min) * sx),
                     static_cast<int>(roi.rows - 30 - (y - y_min) * sy));
  };
  static constexpr size_t kMaxSegs = 500;
  const size_t stride = std::max(size_t(1), pts.size() / kMaxSegs);
  cv::Point prev;
  bool first = true;
  for (size_t idx = 0; idx < pts.size(); idx += stride) {
    const auto &p = pts[idx];
    cv::Point q = to_px(p.first, std::max(y_min, std::min(y_max, p.second)));
    if (!first)
      cv::line(roi, prev, q, color, thickness, cv::LINE_AA);
    prev = q;
    first = false;
  }
}

// ---------------------------------------------------------------------------
// Panel draw functions
// ---------------------------------------------------------------------------

void VizDashboard::draw_panel_xy(
    cv::Mat &roi, const std::string &title,
    const std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> &series,
    const std::vector<std::string> &labels, bool symmetric_y, bool initialized) {
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
    draw_text(roi, initialized ? "no data" : "waiting init...", {10, roi.rows / 2},
              {120, 120, 120}, 0.5, 1);
    return;
  }

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

  double ymin = std::numeric_limits<double>::infinity();
  double ymax = -ymin;
  for (auto &s : series) {
    const auto &dq = *s.first;
    const size_t st = std::max(size_t(1), dq.size() / 500);
    for (size_t i = 0; i < dq.size(); i += st) {
      ymin = std::min(ymin, dq[i].second);
      ymax = std::max(ymax, dq[i].second);
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

  cv::line(roi, {50, roi.rows - 30}, {roi.cols - 10, roi.rows - 30}, {90, 90, 90}, 1);
  cv::line(roi, {50, 30}, {50, roi.rows - 30}, {90, 90, 90}, 1);

  std::ostringstream os;
  os << std::fixed << std::setprecision(2) << "[" << ymin << ", " << ymax << "]";
  draw_text(roi, os.str(), {roi.cols - 160, 18}, {160, 160, 160}, 0.4, 1);

  for (size_t k = 0; k < series.size(); ++k) {
    if (k < labels.size())
      draw_text(roi, labels[k], {10, 36 + static_cast<int>(k) * 16}, series[k].second, 0.42, 1);
    draw_curve(roi, *series[k].first, series[k].second, ymin, ymax, xmin, xmax, 2);
  }

  if (!series.empty() && !series.front().first->empty()) {
    std::ostringstream lv;
    lv << std::fixed << std::setprecision(2);
    for (size_t k = 0; k < series.size(); ++k) {
      if (series[k].first->empty())
        continue;
      double v = series[k].first->back().second;
      lv << (k > 0 ? "  " : "") << (k < labels.size() ? labels[k] : std::string("s")) << "=" << v;
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

void VizDashboard::draw_trajectory_topdown(cv::Mat &roi, const DataState &d) {
  roi.setTo(cv::Scalar(20, 20, 20));
  draw_grid(roi, cv::Scalar(40, 40, 40), 40);

  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto expand = [&](double x, double y) {
    xmin = std::min(xmin, x);
    xmax = std::max(xmax, x);
    ymin = std::min(ymin, y);
    ymax = std::max(ymax, y);
  };
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return d.aligned ? (d.R_gv * p + d.t_gv) : p;
  };
  static constexpr size_t kBBoxStride = 8;
  for (size_t i = 0; i < d.vio_hist.size(); i += kBBoxStride) {
    auto pg = tx(d.vio_hist[i].p);
    expand(pg.x(), pg.y());
  }
  for (size_t i = 0; i < d.gt_hist.size(); i += kBBoxStride)
    expand(d.gt_hist[i].p.x(), d.gt_hist[i].p.y());
  for (const auto &p : d.slam_pts) {
    auto pg = tx(p);
    expand(pg.x(), pg.y());
  }
  if (!std::isfinite(xmin)) {
    xmin = -5; xmax = 5; ymin = -5; ymax = 5;
  }
  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double s = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](double x, double y) {
    return cv::Point(static_cast<int>((x - cx) * s + roi.cols / 2),
                     static_cast<int>(-(y - cy) * s + roi.rows / 2));
  };

  {
    const size_t st = std::max(size_t(1), d.gt_hist.size() / 1000);
    for (size_t i = st; i < d.gt_hist.size(); i += st)
      cv::line(roi, to_px(d.gt_hist[i - st].p.x(), d.gt_hist[i - st].p.y()),
               to_px(d.gt_hist[i].p.x(), d.gt_hist[i].p.y()), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
  }
  {
    const size_t st = std::max(size_t(1), d.vio_hist.size() / 1000);
    for (size_t i = st; i < d.vio_hist.size(); i += st) {
      auto a = tx(d.vio_hist[i - st].p);
      auto b = tx(d.vio_hist[i].p);
      cv::line(roi, to_px(a.x(), a.y()), to_px(b.x(), b.y()), cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
    }
  }
  for (const auto &p : d.msckf_pts) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 2, cv::Scalar(0, 60, 220), -1, cv::LINE_AA);
  }
  for (const auto &p : d.slam_pts) {
    auto pg = tx(p);
    cv::circle(roi, to_px(pg.x(), pg.y()), 3, cv::Scalar(0, 220, 60), -1, cv::LINE_AA);
  }
  if (d.latest_t > 0) {
    Eigen::Vector3d pg = tx(d.latest_p_wi);
    Eigen::Vector3d fwd_vio = d.latest_R_wi * Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d fwd_gt = d.aligned ? (d.R_gv * fwd_vio) : fwd_vio;
    double yaw = std::atan2(fwd_gt.y(), fwd_gt.x());
    cv::Point p0 = to_px(pg.x(), pg.y());
    double L = 0.6 * s, hw = 0.3;
    cv::Point tip(p0.x + static_cast<int>(L * std::cos(yaw)), p0.y - static_cast<int>(L * std::sin(yaw)));
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
  draw_text(roi, "VIO orange", {10, 58}, {255, 160, 0}, 0.45, 1);
  draw_text(roi, "MSCKF red  SLAM green", {10, 76}, {220, 220, 220}, 0.45, 1);
  draw_text(roi, std::string("align=") + (d.aligned ? "on" : "off"), {10, 94},
            d.aligned ? cv::Scalar(0, 255, 0) : cv::Scalar(120, 120, 120), 0.45, 1);
}

void VizDashboard::draw_trajectory_iso(cv::Mat &roi, const DataState &d) {
  roi.setTo(cv::Scalar(18, 18, 22));
  draw_grid(roi, cv::Scalar(38, 38, 42), 40);
  static const Eigen::Matrix3d R_iso = isometric_R();

  double xmin = std::numeric_limits<double>::infinity(), xmax = -xmin;
  double ymin = xmin, ymax = -xmin;
  auto tx = [&](const Eigen::Vector3d &p) -> Eigen::Vector3d {
    return d.aligned ? (d.R_gv * p + d.t_gv) : p;
  };
  auto expand = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d sp = R_iso * p_world;
    xmin = std::min(xmin, sp.x()); xmax = std::max(xmax, sp.x());
    ymin = std::min(ymin, sp.y()); ymax = std::max(ymax, sp.y());
  };
  static constexpr size_t kIsoBBoxStride = 8;
  for (size_t i = 0; i < d.vio_hist.size(); i += kIsoBBoxStride)
    expand(tx(d.vio_hist[i].p));
  for (size_t i = 0; i < d.gt_hist.size(); i += kIsoBBoxStride)
    expand(d.gt_hist[i].p);
  if (!std::isfinite(xmin)) {
    xmin = -5; xmax = 5; ymin = -5; ymax = 5;
  }
  double cx = 0.5 * (xmin + xmax), cy = 0.5 * (ymin + ymax);
  double range = std::max({xmax - xmin, ymax - ymin, 4.0}) * opts_.traj_scale_margin;
  double s = std::min(roi.cols, roi.rows) / range;
  auto to_px = [&](const Eigen::Vector3d &p_world) {
    Eigen::Vector3d sp = R_iso * p_world;
    return cv::Point(static_cast<int>((sp.x() - cx) * s + roi.cols / 2),
                     static_cast<int>(-(sp.y() - cy) * s + roi.rows / 2));
  };

  Eigen::Vector3d ref_origin =
      d.aligned ? (d.gt_hist.empty() ? Eigen::Vector3d::Zero() : d.gt_hist.front().p)
                : (d.vio_hist.empty() ? Eigen::Vector3d::Zero() : d.vio_hist.front().p);
  double axis_len = 5.0;
  cv::Point o = to_px(ref_origin);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(axis_len, 0, 0)), {80, 80, 200}, 1, cv::LINE_AA);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, axis_len, 0)), {80, 200, 80}, 1, cv::LINE_AA);
  cv::line(roi, o, to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len)), {200, 80, 80}, 1, cv::LINE_AA);
  draw_text(roi, "X", to_px(ref_origin + Eigen::Vector3d(axis_len * 1.05, 0, 0)), {120, 120, 220}, 0.45, 1);
  draw_text(roi, "Y", to_px(ref_origin + Eigen::Vector3d(0, axis_len * 1.05, 0)), {120, 220, 120}, 0.45, 1);
  draw_text(roi, "Z", to_px(ref_origin + Eigen::Vector3d(0, 0, axis_len * 1.05)), {220, 120, 120}, 0.45, 1);

  {
    const size_t st = std::max(size_t(1), d.gt_hist.size() / 1000);
    for (size_t i = st; i < d.gt_hist.size(); i += st)
      cv::line(roi, to_px(d.gt_hist[i - st].p), to_px(d.gt_hist[i].p), cv::Scalar(0, 200, 0), 2, cv::LINE_AA);
  }
  {
    const size_t st = std::max(size_t(1), d.vio_hist.size() / 1000);
    for (size_t i = st; i < d.vio_hist.size(); i += st)
      cv::line(roi, to_px(tx(d.vio_hist[i - st].p)), to_px(tx(d.vio_hist[i].p)),
               cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
  }
  if (d.latest_t > 0)
    cv::circle(roi, to_px(tx(d.latest_p_wi)), 5, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);

  draw_text(roi, "3D ISO (azim=45 elev=-30)", {10, 20}, {200, 200, 200}, 0.55, 1);
  draw_text(roi, "GT green   VIO blue", {10, 40}, {200, 200, 200}, 0.45, 1);
}

void VizDashboard::draw_camera_image(cv::Mat &roi, const DataState &d) {
  roi.setTo(cv::Scalar(10, 10, 10));
  if (d.latest_image.empty()) {
    draw_text(roi, "waiting for camera...", {20, 40}, {180, 180, 180}, 0.6, 1);
    return;
  }
  cv::Mat resized;
  double sx = static_cast<double>(roi.cols) / d.latest_image.cols;
  double sy = static_cast<double>(roi.rows - 40) / d.latest_image.rows;
  double s = std::min(sx, sy);
  int w = static_cast<int>(d.latest_image.cols * s);
  int h = static_cast<int>(d.latest_image.rows * s);
  cv::resize(d.latest_image, resized, cv::Size(w, h), 0, 0, cv::INTER_AREA);
  int ox = (roi.cols - w) / 2;
  int oy = 40 + (roi.rows - 40 - h) / 2;
  resized.copyTo(roi(cv::Rect(ox, oy, w, h)));
  std::ostringstream os;
  os << "CAM  t=" << std::fixed << std::setprecision(3) << d.latest_image_t
     << "  src=" << d.latest_image.cols << "x" << d.latest_image.rows
     << "  init=" << (d.initialized ? "ok" : "wait");
  draw_text(roi, os.str(), {10, 24}, {200, 200, 200}, 0.55, 1);
}

void VizDashboard::draw_panel_rpy(cv::Mat &roi, const DataState &d) {
  // Roll (red) / Pitch (green) / Yaw (orange) — all on one panel, auto y-range
  std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> series = {
      {&d.ts_roll, cv::Scalar(60, 60, 255)},
      {&d.ts_pitch, cv::Scalar(60, 220, 60)},
      {&d.ts_yaw, cv::Scalar(255, 140, 30)},
  };
  draw_panel_xy(roi, "Roll/Pitch/Yaw (deg)", series, {"roll", "pitch", "yaw"}, false,
                d.initialized);
}

void VizDashboard::draw_panel_gps_diag(cv::Mat &roi, const DataState &d) {
  if (!d.has_gps_diag) {
    draw_waiting_init(roi, "GPS Alt Diag (no data)");
    return;
  }

  // Top half: residual + P_zz  |  Bottom half: GPS Z vs VIO Z
  const int half_h = roi.rows / 2;
  cv::Mat top = roi(cv::Rect(0, 0, roi.cols, half_h));
  cv::Mat bot = roi(cv::Rect(0, half_h, roi.cols, roi.rows - half_h));

  // Top: residual (cyan) and P_zz (yellow) on shared time axis
  {
    std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> series_res = {
        {&d.ts_gps_res, cv::Scalar(220, 220, 0)},   // cyan-yellow: residual
    };
    draw_panel_xy(top, "GPS Residual (m)", series_res, {"res=GPS-VIO"}, true, d.initialized);

    // Overlay P_zz on the residual panel as a small secondary curve in magenta
    // We scale P_zz to match residual axis range so it's visible
    if (!d.ts_gps_pzz.empty() && !d.ts_gps_res.empty()) {
      double res_max = 0;
      for (auto &p : d.ts_gps_res)
        res_max = std::max(res_max, std::fabs(p.second));
      res_max = std::max(res_max, 1.0);

      double pzz_max = 0;
      for (auto &p : d.ts_gps_pzz)
        pzz_max = std::max(pzz_max, p.second);
      pzz_max = std::max(pzz_max, 1e-6);

      // Draw P_zz normalized to residual scale — rightmost 80px shows raw value text
      double xmin = d.ts_gps_pzz.front().first;
      double xmax = d.ts_gps_pzz.back().first;
      if (xmax > xmin) {
        double sx = (top.cols - 60.0) / (xmax - xmin);
        cv::Point prev;
        bool first = true;
        const size_t stride = std::max(size_t(1), d.ts_gps_pzz.size() / 500);
        for (size_t i = 0; i < d.ts_gps_pzz.size(); i += stride) {
          const auto &p = d.ts_gps_pzz[i];
          double y_scaled = p.second * (res_max / pzz_max); // scale to residual range
          int px = static_cast<int>(50 + (p.first - xmin) * sx);
          int py = static_cast<int>(top.rows - 30 - (y_scaled + res_max) * (top.rows - 50.0) / (2.0 * res_max));
          py = std::max(0, std::min(top.rows - 1, py));
          cv::Point q(px, py);
          if (!first)
            cv::line(top, prev, q, cv::Scalar(200, 60, 200), 1, cv::LINE_AA);
          prev = q;
          first = false;
        }
      }
      draw_text(top, "P_zz (magenta, scaled)", {10, top.rows - 22}, {200, 60, 200}, 0.38, 1);
      if (!d.ts_gps_pzz.empty()) {
        std::ostringstream os;
        os << "P_zz=" << std::fixed << std::setprecision(4) << d.ts_gps_pzz.back().second
           << "  K_pz=" << std::fixed << std::setprecision(5)
           << (d.ts_gps_kpz.empty() ? 0.0 : d.ts_gps_kpz.back().second);
        draw_text(top, os.str(), {10, top.rows - 8}, {180, 180, 180}, 0.38, 1);
      }
    }
  }

  // Bottom: GPS z (green) and VIO z (orange) compared
  {
    std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> series_z = {
        {&d.ts_gps_z,     cv::Scalar(0, 200, 60)},   // green: GPS z
        {&d.ts_gps_vio_z, cv::Scalar(255, 160, 0)},  // orange: VIO z at GPS time
    };
    draw_panel_xy(bot, "GPS vs VIO Alt (m)", series_z, {"gps_z", "vio_z"}, false, d.initialized);
  }

  // Divider
  cv::line(roi, {0, half_h}, {roi.cols - 1, half_h}, {80, 80, 80}, 1);
}

void VizDashboard::draw_panel_biases(cv::Mat &roi, const DataState &d) {
  const int half_h = roi.rows / 2;
  cv::Mat top = roi(cv::Rect(0, 0, roi.cols, half_h));
  cv::Mat bot = roi(cv::Rect(0, half_h, roi.cols, roi.rows - half_h));
  std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> bg_series = {
      {&d.ts_bg_x, cv::Scalar(80, 80, 255)},
      {&d.ts_bg_y, cv::Scalar(80, 220, 80)},
      {&d.ts_bg_z, cv::Scalar(255, 160, 30)},
  };
  std::vector<std::pair<const std::deque<std::pair<double, double>> *, cv::Scalar>> ba_series = {
      {&d.ts_ba_x, cv::Scalar(80, 80, 255)},
      {&d.ts_ba_y, cv::Scalar(80, 220, 80)},
      {&d.ts_ba_z, cv::Scalar(255, 160, 30)},
  };
  draw_panel_xy(top, "Gyro Bias bg (rad/s)", bg_series, {"bg_x", "bg_y", "bg_z"}, false, d.initialized);
  draw_panel_xy(bot, "Accel Bias ba (m/s^2)", ba_series, {"ba_x", "ba_y", "ba_z"}, false, d.initialized);
  cv::line(roi, {0, half_h}, {roi.cols - 1, half_h}, {80, 80, 80}, 1);
}

void VizDashboard::draw_cross_pane_matches(cv::Mat &roi, const DataState &d) {
  // roi is the full-height rightmost column (col 2).
  // Top half: current raw image + current tracked points (red dots)
  // Bottom half: tracker-owned prev_image_for_viz + prev points (orange dots)
  // No cross-pane lines — clean point-overlay style.

  const int top_h = roi.rows / 2;
  const int bot_h = roi.rows - top_h;
  cv::Mat top_roi = roi(cv::Rect(0, 0, roi.cols, top_h));
  cv::Mat bot_roi = roi(cv::Rect(0, top_h, roi.cols, bot_h));

  // ---------- helper: fit image into a pane, returning scale+offset ----------
  auto fit_image = [](cv::Mat &pane, const cv::Mat &img, int title_bar_h) -> std::pair<double, cv::Point2f> {
    pane.setTo(cv::Scalar(10, 10, 10));
    cv::rectangle(pane, {0, 0, pane.cols - 1, pane.rows - 1}, {50, 50, 50}, 1);
    if (img.empty())
      return {0.0, {0.f, 0.f}};
    double sx = static_cast<double>(pane.cols) / img.cols;
    double sy = static_cast<double>(pane.rows - title_bar_h) / img.rows;
    double s = std::min(sx, sy);
    int w = static_cast<int>(img.cols * s);
    int h = static_cast<int>(img.rows * s);
    if (w <= 0 || h <= 0)
      return {0.0, {0.f, 0.f}};
    cv::Mat resized, bgr;
    cv::resize(img, resized, cv::Size(w, h), 0, 0, cv::INTER_AREA);
    if (resized.channels() == 1)
      cv::cvtColor(resized, bgr, cv::COLOR_GRAY2BGR);
    else
      bgr = resized;
    int ox = (pane.cols - w) / 2;
    int oy = title_bar_h + (pane.rows - title_bar_h - h) / 2;
    if (ox >= 0 && oy >= 0 && ox + w <= pane.cols && oy + h <= pane.rows)
      bgr.copyTo(pane(cv::Rect(ox, oy, w, h)));
    return {s, cv::Point2f(static_cast<float>(ox), static_cast<float>(oy))};
  };

  // kTitleBar is needed both in the fallback path and the normal path below
  const int kTitleBar = 30;

  if (!d.flow_valid) {
    // Fallback: show latest tracker-history image (already carries feature overlays
    // painted by get_historical_viz_image) so the camera panel is never fully blank.
    if (!d.latest_image.empty()) {
      fit_image(top_roi, d.latest_image, kTitleBar);
      draw_text(top_roi, "CURRENT (tracker hist — awaiting flow packet)", {10, 20}, {160, 200, 160}, 0.45, 1);
    } else {
      draw_text(top_roi, "CURRENT RAW  (waiting flow data...)", {10, 24}, {180, 180, 180}, 0.5, 1);
    }
    draw_text(bot_roi, "WARPED PREV  (waiting flow data...)", {10, 24}, {180, 180, 180}, 0.5, 1);
    return;
  }

  // Fit images into panes and get scale+offset for point mapping
  auto fm_top = fit_image(top_roi, d.flow_curr_raw, kTitleBar);
  auto fm_bot = fit_image(bot_roi, d.flow_warped_img, kTitleBar);
  double s_top = fm_top.first;
  cv::Point2f off_top = fm_top.second;
  double s_bot = fm_bot.first;
  cv::Point2f off_bot = fm_bot.second;

  // Map image point to canvas coordinates within a pane
  auto to_canvas = [](const cv::Point2f &pt, double s, const cv::Point2f &off) -> cv::Point {
    return cv::Point(static_cast<int>(pt.x * s + off.x), static_cast<int>(pt.y * s + off.y));
  };

  // ---------- draw points (point-overlay style, no cross-pane lines) ----------
  const cv::Scalar col_prev(60, 180, 255);  // orange for prev points
  const cv::Scalar col_curr(40, 40, 255);    // red for current points
  const int kPtRadius = 3;
  const int kPtThickness = -1; // filled

  // Current raw points in top pane — red filled circles
  for (size_t i = 0; i < d.flow_curr_pts.size(); i++) {
    cv::Point q = to_canvas(d.flow_curr_pts[i], s_top, off_top);
    cv::circle(top_roi, q, kPtRadius, col_curr, kPtThickness, cv::LINE_AA);
  }

  // Prev points in bottom pane — orange filled circles
  for (size_t i = 0; i < d.flow_prev_pts.size(); i++) {
    cv::Point q = to_canvas(d.flow_prev_pts[i], s_bot, off_bot);
    cv::circle(bot_roi, q, kPtRadius, col_prev, kPtThickness, cv::LINE_AA);
  }

  // ---------- labels ----------
  draw_text(top_roi, "CURRENT RAW", {10, 20}, {200, 200, 200}, 0.55, 1);
  std::ostringstream os_top;
  os_top << "n=" << d.flow_curr_pts.size() << "  t=" << std::fixed << std::setprecision(3) << d.flow_t_curr;
  draw_text(top_roi, os_top.str(), {10, 40}, {160, 160, 160}, 0.4, 1);

  if (d.flow_warp_active) {
    draw_text(bot_roi, "WARPED PREV (TRACKER)", {10, 20}, {200, 200, 200}, 0.55, 1);
  } else {
    draw_text(bot_roi, "PREV RAW (WARP OFF)", {10, 20}, {200, 200, 200}, 0.55, 1);
  }
  std::ostringstream os_bot;
  os_bot << "n=" << d.flow_prev_pts.size();
  draw_text(bot_roi, os_bot.str(), {10, 40}, {160, 160, 160}, 0.4, 1);

}

// ---------------------------------------------------------------------------
// Top-level layout
// ---------------------------------------------------------------------------

void VizDashboard::render_frame(cv::Mat &canvas, const DataState &d) {
  // Guard: after a swap the buffer may have wrong size (e.g. if display_mat_
  // was ever left uninitialized). Re-create rather than crash with a bad ROI.
  if (canvas.rows != opts_.height || canvas.cols != opts_.width || canvas.type() != CV_8UC3)
    canvas.create(opts_.height, opts_.width, CV_8UC3);
  canvas.setTo(cv::Scalar(0, 0, 0));

  const int W = opts_.width;
  const int H = opts_.height;
  const int row1_h = H * 540 / 1080; // 540 px
  const int row2_h = (H - row1_h) / 2;
  const int row3_h = H - row1_h - row2_h;
  const int col_w = W / 3; // 640 px

  // Layout:
  //   Row 1 (540px): 3D ISO     | TOP-DOWN XY  |  cross-pane matches
  //   Row 2 (270px): ATE        | Altitude Z   |  (full-height col 2:
  //   Row 3 (270px): RPY        | Speed        |   top=CURRENT RAW, bot=WARPED PREV)
  const int col2_x = 2 * col_w;
  const int col2_w = W - 2 * col_w;
  cv::Mat rc2_full = canvas(cv::Rect(col2_x, 0, col2_w, H));
  draw_cross_pane_matches(rc2_full, d);

  // Row 1: 3D ISO | TOP-DOWN XY
  cv::Mat r1c0 = canvas(cv::Rect(0, 0, col_w, row1_h));
  cv::Mat r1c1 = canvas(cv::Rect(col_w, 0, col_w, row1_h));
  draw_trajectory_iso(r1c0, d);
  draw_trajectory_topdown(r1c1, d);

  // Row 2: ATE | Altitude Z
  int y2 = row1_h;
  cv::Mat r2c0 = canvas(cv::Rect(0, y2, col_w, row2_h));
  cv::Mat r2c1 = canvas(cv::Rect(col_w, y2, col_w, row2_h));
  draw_panel_xy(r2c0, "ATE (m)", {{&d.ts_ate, {0, 200, 255}}}, {"|p_gt-p_vio|"}, false,
                d.initialized);
  draw_panel_xy(r2c1, "Altitude Z (m)",
                {{&d.ts_z_vio, {255, 160, 0}}, {&d.ts_z_gt, {0, 200, 0}}}, {"vio_z", "gt_z"},
                false, d.initialized);

  // Row 3: RPY | GPS Alt Diagnostics (residual + P_zz + GPS-vs-VIO-z comparison)
  int y3 = row1_h + row2_h;
  cv::Mat r3c0 = canvas(cv::Rect(0, y3, col_w, row3_h));
  cv::Mat r3c1 = canvas(cv::Rect(col_w, y3, col_w, row3_h));
  draw_panel_rpy(r3c0, d);
  if (d.has_gps_diag)
    draw_panel_gps_diag(r3c1, d);
  else
    draw_panel_biases(r3c1, d);

  // Speed as compact text overlay on the ATE panel (top-right corner)
  if (!d.ts_speed.empty() && d.initialized) {
    std::ostringstream os_spd;
    os_spd << "|v|=" << std::fixed << std::setprecision(2) << d.ts_speed.back().second << " m/s";
    draw_text(canvas, os_spd.str(), {10, row1_h + 36}, {0, 230, 130}, 0.42, 1);
  }

  // Grid dividers
  cv::line(canvas, {col_w, 0}, {col_w, H}, {80, 80, 80}, 1);           // cols 0|1
  cv::line(canvas, {2 * col_w, 0}, {2 * col_w, H}, {80, 80, 80}, 1);  // cols 1|2
  cv::line(canvas, {0, row1_h}, {2 * col_w, row1_h}, {80, 80, 80}, 1); // row 1|2 (cols 0+1)
  cv::line(canvas, {0, y3}, {2 * col_w, y3}, {80, 80, 80}, 1);         // row 2|3 (cols 0+1)
}

} // namespace ov_msckf
