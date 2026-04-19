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

#ifndef OV_MSCKF_ROS_FREE_TRAJECTORY_ALIGNER_H
#define OV_MSCKF_ROS_FREE_TRAJECTORY_ALIGNER_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <vector>

namespace ov_msckf {

/**
 * @brief SE3-only (no-scale) Umeyama 对齐 + 在线 ATE 统计。
 *
 * 用途: 把 VIO 输出的 world-frame 轨迹 p_V 与 GPS/GT 轨迹 p_G 用一个刚体变换
 *       T_GV 对齐, 之后对每个同步点计算 ATE = || p_G - (R_GV * p_V + t_GV) ||。
 *       align() 使用 Umeyama (不含尺度, 因为 VIO 尺度由 IMU 直接给出)。
 *
 * 典型用法:
 *   - 拿到前 N 组 (p_V, p_G) 对, 调 solve() 拟合 T_GV
 *   - 每一帧用 align_point() 将 VIO 点投到 GT frame 下用于作图与 ATE
 */
class TrajectoryAligner {
public:
  TrajectoryAligner() { R_ = Eigen::Matrix3d::Identity(); t_ = Eigen::Vector3d::Zero(); }

  /**
   * @brief 在已有配对点上拟合 T_GV (gt = R*vio + t)。至少需要 3 个不共线点。
   */
  bool solve(const std::vector<Eigen::Vector3d> &vio, const std::vector<Eigen::Vector3d> &gt) {
    if (vio.size() != gt.size() || vio.size() < 3)
      return false;
    const size_t N = vio.size();
    Eigen::Vector3d cv = Eigen::Vector3d::Zero(), cg = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      cv += vio[i];
      cg += gt[i];
    }
    cv /= static_cast<double>(N);
    cg /= static_cast<double>(N);
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      H.noalias() += (vio[i] - cv) * (gt[i] - cg).transpose();
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d S = Eigen::Matrix3d::Identity();
    if ((V * U.transpose()).determinant() < 0)
      S(2, 2) = -1;
    R_ = V * S * U.transpose();
    t_ = cg - R_ * cv;
    solved_ = true;
    return true;
  }

  /// [中文] 将 VIO world 坐标下的点映射到 GT/GPS 坐标系
  Eigen::Vector3d align_point(const Eigen::Vector3d &p_vio) const { return R_ * p_vio + t_; }

  bool solved() const { return solved_; }
  const Eigen::Matrix3d &R() const { return R_; }
  const Eigen::Vector3d &t() const { return t_; }

  /**
   * @brief Given a GT map keyed by time and a set of (time, p_vio) samples,
   *        pick nearest GT within max_dt and produce matched pairs for solve().
   */
  template <typename GtMap>
  static size_t build_pairs(const std::deque<std::pair<double, Eigen::Vector3d>> &vio_hist,
                            const GtMap &gt_map, double max_dt, std::vector<Eigen::Vector3d> &vio_out,
                            std::vector<Eigen::Vector3d> &gt_out) {
    vio_out.clear();
    gt_out.clear();
    if (gt_map.empty())
      return 0;
    for (const auto &kv : vio_hist) {
      auto it = gt_map.lower_bound(kv.first);
      auto best = gt_map.end();
      double best_dt = max_dt;
      if (it != gt_map.end() && std::fabs(it->first - kv.first) < best_dt) {
        best = it;
        best_dt = std::fabs(it->first - kv.first);
      }
      if (it != gt_map.begin()) {
        auto prev = std::prev(it);
        if (std::fabs(prev->first - kv.first) < best_dt) {
          best = prev;
          best_dt = std::fabs(prev->first - kv.first);
        }
      }
      if (best != gt_map.end()) {
        vio_out.push_back(kv.second);
        gt_out.push_back(best->second);
      }
    }
    return vio_out.size();
  }

private:
  Eigen::Matrix3d R_;
  Eigen::Vector3d t_;
  bool solved_ = false;
};

} // namespace ov_msckf

#endif // OV_MSCKF_ROS_FREE_TRAJECTORY_ALIGNER_H
