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

#include "UpdaterSLAM.h"

#include "UpdaterHelper.h"
#include "VisualObservabilityPolicy.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_map>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

double diag_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

double wrap_deg_local(double x) {
  while (x > 180.0)
    x -= 360.0;
  while (x < -180.0)
    x += 360.0;
  return x;
}

struct LatestFeatureObs {
  bool valid = false;
  size_t cam_id = 0;
  double u = diag_nan();
  double v = diag_nan();
  double prev_u = diag_nan();
  double prev_v = diag_nan();
  double du = diag_nan();
  double dv = diag_nan();
  double created_time = diag_nan();
  double last_update_time = diag_nan();
  int track_age = 0;
};

struct FeatureUvSummary {
  int num_measurements = 0;
  double u_mean = diag_nan();
  double v_mean = diag_nan();
};

FeatureUvSummary summarize_feature_uv(const std::shared_ptr<Feature> &feature) {
  FeatureUvSummary out;
  if (!feature)
    return out;
  double sum_u = 0.0;
  double sum_v = 0.0;
  int count = 0;
  for (const auto &kv : feature->timestamps) {
    const size_t cam_id = kv.first;
    const auto &times = kv.second;
    const auto it_uv = feature->uvs.find(cam_id);
    if (it_uv == feature->uvs.end())
      continue;
    const auto &uvs = it_uv->second;
    const size_t n = std::min(times.size(), uvs.size());
    for (size_t i = 0; i < n; i++) {
      sum_u += uvs[i](0);
      sum_v += uvs[i](1);
      count++;
    }
  }
  out.num_measurements = count;
  if (count > 0) {
    out.u_mean = sum_u / (double)count;
    out.v_mean = sum_v / (double)count;
  }
  return out;
}

LatestFeatureObs latest_feature_obs(const std::shared_ptr<Feature> &feature) {
  LatestFeatureObs out;
  if (!feature)
    return out;
  bool have_time = false;
  for (const auto &kv : feature->timestamps) {
    const size_t cam_id = kv.first;
    const auto &times = kv.second;
    const auto it_uv = feature->uvs.find(cam_id);
    if (it_uv == feature->uvs.end())
      continue;
    const auto &uvs = it_uv->second;
    const size_t n = std::min(times.size(), uvs.size());
    if (n == 0)
      continue;
    out.track_age += (int)n;
    for (size_t i = 0; i < n; i++) {
      if (!have_time || times[i] < out.created_time)
        out.created_time = times[i];
      have_time = true;
    }
    if (!out.valid || times[n - 1] > out.last_update_time) {
      out.valid = true;
      out.cam_id = cam_id;
      out.last_update_time = times[n - 1];
      out.u = uvs[n - 1](0);
      out.v = uvs[n - 1](1);
      if (n >= 2) {
        out.prev_u = uvs[n - 2](0);
        out.prev_v = uvs[n - 2](1);
        out.du = out.u - out.prev_u;
        out.dv = out.v - out.prev_v;
      }
    }
  }
  return out;
}

double roll_deg_from_state(const std::shared_ptr<State> &state) {
  if (!state || !state->_imu)
    return diag_nan();
  Eigen::Matrix3d R_ItoG = state->_imu->Rot().transpose();
  const double roll = std::atan2(R_ItoG(2, 1), R_ItoG(2, 2));
  return roll * 180.0 / M_PI;
}

double yaw_deg_from_state(const std::shared_ptr<State> &state) {
  if (!state || !state->_imu)
    return diag_nan();
  Eigen::Matrix3d R_ItoG = state->_imu->Rot().transpose();
  const double yaw = std::atan2(R_ItoG(1, 0), R_ItoG(0, 0));
  return yaw * 180.0 / M_PI;
}

std::string image_side(double u) {
  if (!std::isfinite(u))
    return "unknown";
  return u < 320.0 ? "left" : "right";
}

std::string image_quadrant(double u, double v) {
  if (!std::isfinite(u) || !std::isfinite(v))
    return "unknown";
  if (u < 320.0 && v < 240.0)
    return "q1";
  if (u >= 320.0 && v < 240.0)
    return "q2";
  if (u < 320.0 && v >= 240.0)
    return "q3";
  return "q4";
}

struct SlamAnchorGeometryDiag {
  double anchor_clone_time = diag_nan();
  double time_since_anchor = diag_nan();
  int anchor_cam_id = -1;
  int representation = -1;
  int has_anchor_change = 0;
  int update_fail_count = 0;
  int obs_count = 0;
  double obs_span_s = diag_nan();
  double bearing_spread_deg = diag_nan();
  double landmark_global_norm = diag_nan();
  double depth_current = diag_nan();
  double inverse_depth_current = diag_nan();
};

double feature_bearing_spread_deg(const std::shared_ptr<Feature> &feature) {
  if (!feature)
    return diag_nan();
  std::vector<Eigen::Vector3d> bearings;
  for (const auto &kv : feature->uvs_norm) {
    for (const auto &uv : kv.second) {
      Eigen::Vector3d b((double)uv(0), (double)uv(1), 1.0);
      const double n = b.norm();
      if (n > 1e-12)
        bearings.push_back(b / n);
    }
  }
  if (bearings.size() < 2)
    return 0.0;
  double max_angle = 0.0;
  for (size_t i = 0; i < bearings.size(); i++) {
    for (size_t j = i + 1; j < bearings.size(); j++) {
      const double c = std::max(-1.0, std::min(1.0, bearings[i].dot(bearings[j])));
      max_angle = std::max(max_angle, std::acos(c));
    }
  }
  return max_angle * 180.0 / M_PI;
}

SlamAnchorGeometryDiag slam_anchor_geometry_diag(
    const std::shared_ptr<State> &state,
    const std::shared_ptr<Feature> &feature,
    const std::shared_ptr<Landmark> &landmark,
    const LatestFeatureObs &obs) {
  SlamAnchorGeometryDiag out;
  if (!state || !landmark)
    return out;
  out.anchor_clone_time = landmark->_anchor_clone_timestamp;
  out.time_since_anchor = std::isfinite(out.anchor_clone_time) ?
      state->_timestamp - out.anchor_clone_time : diag_nan();
  out.anchor_cam_id = landmark->_anchor_cam_id;
  out.representation = (int)landmark->_feat_representation;
  out.has_anchor_change = landmark->has_had_anchor_change ? 1 : 0;
  out.update_fail_count = landmark->update_fail_count;
  out.bearing_spread_deg = feature_bearing_spread_deg(feature);
  if (feature) {
    double t_min = std::numeric_limits<double>::infinity();
    double t_max = -std::numeric_limits<double>::infinity();
    for (const auto &kv : feature->timestamps) {
      out.obs_count += (int)kv.second.size();
      for (double t : kv.second) {
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
      }
    }
    if (std::isfinite(t_min) && std::isfinite(t_max))
      out.obs_span_s = t_max - t_min;
  }

  Eigen::Vector3d p_FinG = landmark->get_xyz(false);
  if (LandmarkRepresentation::is_relative_representation(landmark->_feat_representation) &&
      landmark->_anchor_cam_id >= 0 &&
      state->_calib_IMUtoCAM.find((size_t)landmark->_anchor_cam_id) != state->_calib_IMUtoCAM.end() &&
      state->_clones_IMU.find(landmark->_anchor_clone_timestamp) != state->_clones_IMU.end()) {
    Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at((size_t)landmark->_anchor_cam_id)->Rot();
    Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at((size_t)landmark->_anchor_cam_id)->pos();
    Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(landmark->_anchor_clone_timestamp)->Rot();
    Eigen::Vector3d p_IinG = state->_clones_IMU.at(landmark->_anchor_clone_timestamp)->pos();
    p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (landmark->get_xyz(false) - p_IinC) + p_IinG;
  }
  out.landmark_global_norm = p_FinG.norm();

  const size_t cam_id = obs.valid ? obs.cam_id : (landmark->_anchor_cam_id >= 0 ? (size_t)landmark->_anchor_cam_id : 0);
  double clone_time = obs.valid ? obs.last_update_time : state->_timestamp;
  if (state->_clones_IMU.find(clone_time) == state->_clones_IMU.end())
    clone_time = state->_timestamp;
  if (state->_clones_IMU.find(clone_time) != state->_clones_IMU.end() &&
      state->_calib_IMUtoCAM.find(cam_id) != state->_calib_IMUtoCAM.end()) {
    Eigen::Matrix3d R_GtoI = state->_clones_IMU.at(clone_time)->Rot();
    Eigen::Vector3d p_IinG = state->_clones_IMU.at(clone_time)->pos();
    Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(cam_id)->Rot();
    Eigen::Vector3d p_IinC = state->_calib_IMUtoCAM.at(cam_id)->pos();
    Eigen::Vector3d p_FinI = R_GtoI * (p_FinG - p_IinG);
    Eigen::Vector3d p_FinC = R_ItoC * p_FinI + p_IinC;
    out.depth_current = p_FinC(2);
    out.inverse_depth_current = std::fabs(out.depth_current) > 1e-12 ?
        1.0 / out.depth_current : diag_nan();
  }
  return out;
}

double percentile_or_nan(std::vector<double> v, double q) {
  if (v.empty())
    return diag_nan();
  std::sort(v.begin(), v.end());
  q = std::max(0.0, std::min(1.0, q));
  const double idx = q * (double)(v.size() - 1);
  const size_t lo = (size_t)std::floor(idx);
  const size_t hi = std::min(v.size() - 1, lo + 1);
  const double a = idx - (double)lo;
  return (1.0 - a) * v[lo] + a * v[hi];
}

std::vector<int> build_h_id_local(const std::vector<std::shared_ptr<Type>> &H_order) {
  std::vector<int> H_id;
  int current_it = 0;
  for (const auto &var : H_order) {
    H_id.push_back(current_it);
    current_it += var ? var->size() : 0;
  }
  return H_id;
}

Eigen::Vector3d yaw_position_dir_local(const Eigen::Vector3d &p) {
  return Eigen::Vector3d::UnitZ().cross(p);
}

void fill_pose_yaw_gauge_local(Eigen::VectorXd &n, int col, int size,
                               const Eigen::Matrix3d &R_GtoI,
                               const Eigen::Vector3d &p_IinG,
                               bool has_velocity,
                               const Eigen::Vector3d &v_IinG) {
  if (col < 0 || col + size > n.rows())
    return;
  if (size >= 3)
    n.block(col, 0, 3, 1) = R_GtoI * Eigen::Vector3d::UnitZ();
  if (size >= 6)
    n.block(col + 3, 0, 3, 1) = yaw_position_dir_local(p_IinG);
  if (has_velocity && size >= 9)
    n.block(col + 6, 0, 3, 1) = yaw_position_dir_local(v_IinG);
}

void fill_landmark_yaw_gauge_local(Eigen::VectorXd &n, int col,
                                   const std::shared_ptr<Landmark> &lm,
                                   bool use_fej) {
  if (!lm || col < 0 || col + lm->size() > n.rows())
    return;
  if (LandmarkRepresentation::is_relative_representation(lm->_feat_representation))
    return;
  if (lm->_feat_representation != LandmarkRepresentation::GLOBAL_3D || lm->size() != 3)
    return;
  n.block(col, 0, 3, 1) = yaw_position_dir_local(lm->get_xyz(use_fej));
}

Eigen::VectorXd build_global_yaw_gauge_small_local(
    const std::shared_ptr<State> &state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const std::vector<int> &H_id,
    int H_cols,
    bool use_fej) {
  Eigen::VectorXd n = Eigen::VectorXd::Zero(H_cols);
  if (!state || !state->_imu)
    return n;
  for (size_t i = 0; i < H_order.size(); i++) {
    const auto &var = H_order[i];
    if (!var)
      continue;
    const int col = H_id[i];

    if (var == state->_imu) {
      fill_pose_yaw_gauge_local(n, col, var->size(),
                                use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                                use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                                true,
                                use_fej ? state->_imu->vel_fej() : state->_imu->vel());
      continue;
    }
    if (var == state->_imu->q()) {
      fill_pose_yaw_gauge_local(n, col, var->size(),
                                use_fej ? state->_imu->Rot_fej() : state->_imu->Rot(),
                                use_fej ? state->_imu->pos_fej() : state->_imu->pos(),
                                false, Eigen::Vector3d::Zero());
      continue;
    }
    if (var == state->_imu->p()) {
      n.block(col, 0, 3, 1) = yaw_position_dir_local(use_fej ? state->_imu->pos_fej() : state->_imu->pos());
      continue;
    }
    if (var == state->_imu->v()) {
      n.block(col, 0, 3, 1) = yaw_position_dir_local(use_fej ? state->_imu->vel_fej() : state->_imu->vel());
      continue;
    }
    if (var == state->_imu->bg() || var == state->_imu->ba())
      continue;

    bool matched_clone = false;
    for (const auto &clone : state->_clones_IMU) {
      const auto &pose = clone.second;
      if (!pose)
        continue;
      if (var == pose) {
        fill_pose_yaw_gauge_local(n, col, var->size(),
                                  use_fej ? pose->Rot_fej() : pose->Rot(),
                                  use_fej ? pose->pos_fej() : pose->pos(),
                                  false, Eigen::Vector3d::Zero());
        matched_clone = true;
        break;
      }
      if (var == pose->q()) {
        fill_pose_yaw_gauge_local(n, col, var->size(),
                                  use_fej ? pose->Rot_fej() : pose->Rot(),
                                  use_fej ? pose->pos_fej() : pose->pos(),
                                  false, Eigen::Vector3d::Zero());
        matched_clone = true;
        break;
      }
      if (var == pose->p()) {
        n.block(col, 0, 3, 1) = yaw_position_dir_local(use_fej ? pose->pos_fej() : pose->pos());
        matched_clone = true;
        break;
      }
    }
    if (matched_clone)
      continue;

    fill_landmark_yaw_gauge_local(n, col, std::dynamic_pointer_cast<Landmark>(var), use_fej);
  }
  return n;
}

Eigen::MatrixXd project_global_yaw_from_H_local(
    const std::shared_ptr<State> &state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    double alpha,
    bool use_fej) {
  alpha = std::max(0.0, std::min(1.0, alpha));
  if (alpha <= 1e-12)
    return H;
  std::vector<int> H_id = build_h_id_local(H_order);
  Eigen::VectorXd n = build_global_yaw_gauge_small_local(state, H_order, H_id, H.cols(), use_fej);
  const double n2 = n.squaredNorm();
  if (n2 < 1e-12)
    return H;
  Eigen::VectorXd Hn = H * n;
  return H - (alpha / n2) * Hn * n.transpose();
}

Eigen::MatrixXd project_for_update_diag_local(
    const std::shared_ptr<State> &state,
    const std::vector<std::shared_ptr<Type>> &H_order,
    const Eigen::MatrixXd &H,
    StateHelper::VisualYawUpdateMode mode,
    double alpha) {
  if (mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION)
    return project_global_yaw_from_H_local(state, H_order, H, alpha, false);
  if (mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION)
    return project_global_yaw_from_H_local(state, H_order, H, alpha, true);
  return H;
}

double HN_norm_local(const std::shared_ptr<State> &state,
                     const std::vector<std::shared_ptr<Type>> &H_order,
                     const Eigen::MatrixXd &H,
                     bool use_fej) {
  if (H.rows() == 0 || H.cols() == 0)
    return diag_nan();
  std::vector<int> H_id = build_h_id_local(H_order);
  Eigen::VectorXd n = build_global_yaw_gauge_small_local(state, H_order, H_id, H.cols(), use_fej);
  const double nn = n.norm();
  if (nn < 1e-12)
    return diag_nan();
  return (H * (n / nn)).norm();
}

double mahalanobis_chi2_local(const std::shared_ptr<State> &state,
                              const std::vector<std::shared_ptr<Type>> &H_order,
                              const Eigen::MatrixXd &H,
                              const Eigen::VectorXd &res,
                              const Eigen::MatrixXd &R) {
  if (!state || H.rows() == 0 || H.cols() == 0 || res.rows() != H.rows() || R.rows() != H.rows())
    return diag_nan();
  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S = H * P_small * H.transpose() + R;
  return res.dot(S.llt().solve(res));
}

int guarded_matrix_rank(const Eigen::MatrixXd &H) {
  if (H.rows() == 0 || H.cols() == 0)
    return 0;
  const long long elems = (long long)H.rows() * (long long)H.cols();
  if (elems > 2000000LL)
    return -1;
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(H);
  qr.setThreshold(1e-9);
  return (int)qr.rank();
}

double covariance_block_norm(const std::shared_ptr<State> &state,
                             const std::shared_ptr<Type> &a,
                             const std::shared_ptr<Type> &b) {
  if (!state || !a || !b)
    return 0.0;
  if (a->id() < 0 || b->id() < 0)
    return 0.0;
  if (a == b) {
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, {a});
    return P.norm();
  }
  Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, {a, b});
  if (P.rows() < a->size() + b->size() || P.cols() < a->size() + b->size())
    return 0.0;
  return P.block(0, a->size(), a->size(), b->size()).norm();
}

} // namespace

UpdaterSLAM::UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco, ov_core::FeatureInitializerOptions &feat_init_options)
    : _options_slam(options_slam), _options_aruco(options_aruco) {

  // Save our raw pixel noise squared
  _options_slam.sigma_pix_sq = std::pow(_options_slam.sigma_pix, 2);
  _options_aruco.sigma_pix_sq = std::pow(_options_aruco.sigma_pix, 2);

  // Save our feature initializer
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterSLAM::configure_slam_info_reduction(bool enabled, double low_ratio,
                                                double high_ratio, double alpha_max) {
  slam_info_cfg_.enabled = enabled;
  slam_info_cfg_.low_ratio = std::max(0.0, low_ratio);
  slam_info_cfg_.high_ratio = std::max(slam_info_cfg_.low_ratio + 1e-6, high_ratio);
  slam_info_cfg_.alpha_max = std::max(1.0, alpha_max);
}

void UpdaterSLAM::set_slam_info_reduction_diag_path(const std::string &path) {
  if (of_slam_info_reduction_diag_.is_open())
    of_slam_info_reduction_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_slam_info_reduction_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_slam_info_reduction_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-INFO-REDUCTION-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_slam_info_reduction_diag_
      << "time,feature_id,chi2,chi2_threshold,chi2_ratio,accepted_by_chi2,"
      << "alpha,sqrt_inv_alpha,residual_norm_before,whitened_residual_norm_before,"
      << "residual_norm_after,whitened_residual_norm_after,track_age,num_measurements,"
      << "u_mean,v_mean,image_side,roll,yaw_rate\n";
  of_slam_info_reduction_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-INFO-REDUCTION-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), slam_info_diag_t0_, slam_info_diag_t1_);
}

void UpdaterSLAM::set_slam_ekf_leverage_diag_path(const std::string &path) {
  if (of_slam_ekf_leverage_diag_.is_open())
    of_slam_ekf_leverage_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_slam_ekf_leverage_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_slam_ekf_leverage_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-EKF-LEVERAGE-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_slam_ekf_leverage_diag_
      << "time,feature_id,chi2,chi2_threshold,chi2_ratio,NIS_per_dof,dof,"
      << "residual_norm,whitened_residual_norm,dx_yaw_deg,dx_bgz,dx_pos_norm,"
      << "dx_landmark_norm,dx_norm,K_yaw_row_norm,K_bgz_row_norm,K_pos_row_norm,"
      << "K_landmark_row_norm,H_norm,whitened_H_norm,H_yaw_col_norm,H_bgz_col_norm,"
      << "H_pos_col_norm,H_landmark_norm,H_other_norm,S_cond,S_min_eig,S_max_eig,"
      << "HPH_trace,R_trace,HPH_over_R,P_yaw_var,P_bgz_var,P_pos_trace,"
      << "corr_yaw_bgz,corr_yaw_px,corr_yaw_py,pose_landmark_cov_norm,"
      << "track_age,num_measurements,u_mean,v_mean,image_side,image_quadrant,roll,yaw_rate,"
      << "anchor_clone_time,time_since_anchor,anchor_cam_id,representation,"
      << "has_anchor_change,update_fail_count,obs_count,obs_span_s,bearing_spread_deg,"
      << "landmark_global_norm,depth_current,inverse_depth_current\n";
  of_slam_ekf_leverage_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-EKF-LEVERAGE-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), slam_ekf_leverage_diag_t0_, slam_ekf_leverage_diag_t1_);
}

void UpdaterSLAM::set_slam_stacked_ekf_diag_path(const std::string &path) {
  if (of_slam_stacked_ekf_diag_.is_open())
    of_slam_stacked_ekf_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_slam_stacked_ekf_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_slam_stacked_ekf_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-STACKED-EKF-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_slam_stacked_ekf_diag_
      << "time,num_accepted_features,num_regular_slam_features,total_residual_dim,"
      << "total_state_dim_involved,num_landmarks_involved,num_clones_involved,"
      << "chi2_total_before_projection,chi2_total_after_projection,NIS_per_dof_before,"
      << "NIS_per_dof_after,actual_dx_yaw_deg,pred_dx_yaw_deg,pred_raw_dx_yaw_deg,"
      << "actual_dx_bgz,pred_dx_bgz,actual_dx_pos_norm,pred_dx_pos_norm,"
      << "actual_dx_landmark_norm,pred_dx_landmark_norm,fej_used,oc_projection_applied,"
      << "projection_alpha,chi2_gate_uses_projected,final_ekf_uses_projected,"
      << "HN_norm_before,HN_norm_after,HN_rel_before,HN_rel_after,"
      << "H_yaw_col_norm_before,H_yaw_col_norm_after,rank_before,rank_after,"
      << "residual_norm_before,residual_norm_after,H_norm_before,H_norm_after,"
      << "whitened_H_norm_after,K_yaw_row_norm,K_bgz_row_norm,K_pos_row_norm,"
      << "K_landmark_row_norm,K_yaw_times_r_deg,K_bgz_times_r,K_position_times_r_norm,"
      << "S_cond,HPH_over_R,P_yaw_var,P_bgz_var,corr_yaw_bgz,corr_yaw_px,corr_yaw_py,"
      << "pose_landmark_cov_norm,landmark_cov_norm,pose_cov_norm,"
      << "median_depth_current,p95_depth_current,median_inverse_depth,"
      << "median_bearing_spread,p95_bearing_spread,count_far_depth,"
      << "count_low_parallax,count_high_pose_landmark_cov,count_cross_turn_landmarks\n";
  of_slam_stacked_ekf_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-STACKED-EKF-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), slam_stacked_ekf_diag_t0_, slam_stacked_ekf_diag_t1_);
}

void UpdaterSLAM::set_slam_landmark_metadata_diag_path(const std::string &path) {
  if (of_slam_landmark_metadata_diag_.is_open())
    of_slam_landmark_metadata_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_slam_landmark_metadata_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_slam_landmark_metadata_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-LANDMARK-METADATA-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_slam_landmark_metadata_diag_
      << "time,feature_id,first_seen_time,first_added_to_state_time,"
      << "first_slam_update_time,last_slam_update_time,slam_update_count,"
      << "total_update_obs_count,current_track_age,current_obs_count,"
      << "created_before_diag_window,updated_after_diag_window_start,"
      << "crosses_1139,crosses_1182,anchor_clone_time,first_anchor_clone_time,"
      << "last_anchor_clone_time,time_since_anchor,anchor_cam_id,representation,"
      << "has_anchor_change,anchor_change_count,update_fail_count,bearing_spread_deg,"
      << "obs_span_s,landmark_global_norm,depth_current,inverse_depth_current,"
      << "dx_yaw_deg,NIS_per_dof,pose_landmark_cov_norm,image_side,image_quadrant,"
      << "u,v,roll,yaw_rate\n";
  of_slam_landmark_metadata_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-LANDMARK-METADATA-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), slam_landmark_metadata_diag_t0_, slam_landmark_metadata_diag_t1_);
}

void UpdaterSLAM::set_slam_geometry_lifecycle_refresh_diag_path(const std::string &path) {
  if (of_slam_geometry_refresh_diag_.is_open())
    of_slam_geometry_refresh_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_slam_geometry_refresh_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_slam_geometry_refresh_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-GEOMETRY-REFRESH-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_slam_geometry_refresh_diag_
      << "time,feature_id,refresh,reason,age_since_added,first_added_to_state_time,"
      << "depth_current,inverse_depth_current,landmark_global_norm,time_since_anchor,"
      << "pose_landmark_cov_norm,min_pose_landmark_cov_norm,min_regular_features_after_refresh,"
      << "regular_features_before_refresh,regular_features_after_refresh,"
      << "has_anchor_change,anchor_change_count,state_dim_before,active_slam_count_before,"
      << "landmark_state_id,landmark_state_size,landmark_should_marg_before,feature_to_delete_before,"
      << "anchor_cam_id,anchor_clone_timestamp,min_depth_current,min_age_since_added,"
      << "require_anchor_change,image_side,image_quadrant,u,v\n";
  of_slam_geometry_refresh_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-GEOMETRY-REFRESH-DIAG] writing %s\n" RESET, path.c_str());
}

void UpdaterSLAM::set_feature_yaw_contrib_diag_path(const std::string &path) {
  if (of_feature_yaw_contrib_diag_.is_open())
    of_feature_yaw_contrib_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_feature_yaw_contrib_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_feature_yaw_contrib_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-FEATURE-YAW-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_feature_yaw_contrib_diag_
      << "time,feature_id,track_age,created_time,last_update_time,cam_id,"
      << "u,v,prev_u,prev_v,du,dv,image_side,image_quadrant,"
      << "is_created_during_turn,is_updated_during_turn,roll,yaw_rate,"
      << "is_turn_phase,is_straight_recovery_phase,"
      << "residual_u,residual_v,residual_norm,whitened_residual_norm,"
      << "chi2,chi2_threshold,accepted,rejected,reject_reason,"
      << "H_yaw,yaw_jacobian_proxy,delta_yaw_contribution,"
      << "contribution_sign,abs_contribution,contribution_rank_in_frame,"
      << "sum_delta_yaw_frame,feature_contribution_ratio,"
      << "side_contribution_ratio,quadrant_contribution_ratio\n";
  of_feature_yaw_contrib_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-FEATURE-YAW-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), feature_yaw_diag_t0_, feature_yaw_diag_t1_);
}

void UpdaterSLAM::set_yaw_contrib_cap_diag_path(const std::string &path) {
  if (of_yaw_contrib_cap_diag_.is_open())
    of_yaw_contrib_cap_diag_.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_yaw_contrib_cap_diag_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_yaw_contrib_cap_diag_.is_open()) {
    PRINT_WARNING(YELLOW "[SLAM-YAW-CAP-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_yaw_contrib_cap_diag_
      << "time,roll,yaw_rate,yaw_error_before,cap_enabled,num_slam_features,"
      << "num_same_sign_features,num_capped_features,topk_abs_contrib_sum,"
      << "frame_abs_contrib_sum,cap_abs_contrib_ratio,raw_delta_yaw_pred,"
      << "capped_delta_yaw_pred,actual_delta_yaw_update,dominant_side,"
      << "dominant_quadrant,mean_u_capped,mean_v_capped,feature_ids_capped\n";
  of_yaw_contrib_cap_diag_.flush();
  PRINT_INFO(GREEN "[SLAM-YAW-CAP-DIAG] writing %s\n" RESET, path.c_str());
}

void UpdaterSLAM::delayed_init(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // Get current camera pose
      Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // Append to our map
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }

  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
  auto it1 = feature_vec.begin();
  while (it1 != feature_vec.end()) {

    // Triangulate the feature and remove if it fails
    bool success_tri = true;
    if (initializer_feat->config().triangulate_1d) {
      success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
    } else {
      success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
    }

    // Gauss-newton refine the feature
    bool success_refine = true;
    if (initializer_feat->config().refine_features) {
      success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
    }

    // Remove the feature if not a success
    if (!success_tri || !success_refine) {
      (*it1)->to_delete = true;
      it1 = feature_vec.erase(it1);
      continue;
    }
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    auto feat_rep =
        ((int)feat.featid < state->_options.max_aruco_features) ? state->_options.feat_rep_aruco : state->_options.feat_rep_slam;
    feat.feat_representation = feat_rep;
    if (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // If we are doing the single feature representation, then we need to remove the bearing portion
    // To do so, we project the bearing portion onto the state and depth Jacobians and the residual.
    // This allows us to directly initialize the feature as a depth-old feature
    if (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

      // Append the Jacobian in respect to the depth of the feature
      Eigen::MatrixXd H_xf = H_x;
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + 1);
      H_xf.block(0, H_x.cols(), H_x.rows(), 1) = H_f.block(0, H_f.cols() - 1, H_f.rows(), 1);
      H_f.conservativeResize(H_f.rows(), H_f.cols() - 1);

      // Nullspace project the bearing portion
      // This takes into account that we have marginalized the bearing already
      // Thus this is crucial to ensuring estimator consistency as we are not taking the bearing to be true
      UpdaterHelper::nullspace_project_inplace(H_f, H_xf, res);

      // Split out the state portion and feature portion
      H_x = H_xf.block(0, 0, H_xf.rows(), H_xf.cols() - 1);
      H_f = H_xf.block(0, H_xf.cols() - 1, H_xf.rows(), 1);
    }

    // Create feature pointer (we will always create it of size three since we initialize the single invese depth as a msckf anchored
    // representation)
    int landmark_size = (feat_rep == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 1 : 3;
    auto landmark = std::make_shared<Landmark>(landmark_size);
    landmark->_featid = feat.featid;
    landmark->_feat_representation = feat_rep;
    landmark->_unique_camera_id = (*it2)->anchor_cam_id;
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      landmark->_anchor_cam_id = feat.anchor_cam_id;
      landmark->_anchor_clone_timestamp = feat.anchor_clone_timestamp;
      landmark->set_from_xyz(feat.p_FinA, false);
      landmark->set_from_xyz(feat.p_FinA_fej, true);
    } else {
      landmark->set_from_xyz(feat.p_FinG, false);
      landmark->set_from_xyz(feat.p_FinG_fej, true);
    }

    // Measurement noise matrix
    double sigma_pix_sq =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
    Eigen::MatrixXd R = sigma_pix_sq * Eigen::MatrixXd::Identity(res.rows(), res.rows());

    // Try to initialize, delete new pointer if we failed.
    // For pre-chi2 VOP modes pass a lambda that projects Hup before the gate.
    double chi2_multipler =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
    VisualOcFn oc_fn_delayed = nullptr;
    if (vop_ && vop_->is_active()) {
      auto vop_copy = vop_; // capture by value so the lambda is self-contained
      auto state_copy = state;
      oc_fn_delayed = [vop_copy, state_copy](
          const Eigen::MatrixXd &H,
          const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
          const std::vector<int> &H_id) -> Eigen::MatrixXd {
        return vop_copy->apply(H, H_order, H_id, state_copy);
      };
    }
    if (StateHelper::initialize(state, landmark, Hx_order, H_x, H_f, R, res, chi2_multipler,
                                visual_yaw_update_mode_, visual_yaw_update_scale_,
                                visual_global_yaw_oc_alpha_, oc_fn_delayed)) {
      state->_features_SLAM.insert({(*it2)->featid, landmark});
      auto &meta = slam_landmark_metadata_[(*it2)->featid];
      LatestFeatureObs obs = latest_feature_obs(*it2);
      if (!std::isfinite(meta.first_seen_time))
        meta.first_seen_time = std::isfinite(obs.created_time) ? obs.created_time : state->_timestamp;
      if (!std::isfinite(meta.first_added_to_state_time))
        meta.first_added_to_state_time = state->_timestamp;
      if (!std::isfinite(meta.first_anchor_clone_time))
        meta.first_anchor_clone_time = landmark->_anchor_clone_timestamp;
      meta.last_anchor_clone_time = landmark->_anchor_clone_timestamp;
      meta.anchor_cam_id = landmark->_anchor_cam_id;
      meta.representation = (int)landmark->_feat_representation;
      (*it2)->to_delete = true;
      it2++;
    } else {
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
    }
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  if (!feature_vec.empty()) {
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds initialize (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
    PRINT_ALL("[SLAM-DELAY]: %.4f seconds total\n", (rT3 - rT1).total_microseconds() * 1e-6);
  }
}

void UpdaterSLAM::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Get the landmark and its representation
    // For single depth representation we need at least two measurement
    // This is because we do nullspace projection
    std::shared_ptr<Landmark> landmark = state->_features_SLAM.at((*it0)->featid);
    int required_meas = (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 2 : 1;

    // Remove if we don't have enough
    if (ct_meas < 1) {
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else if (ct_meas < required_meas) {
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  size_t max_hx_size = state->max_covariance_size();

  // Large Jacobian, residual, and measurement noise of *all* features for this update
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  Eigen::MatrixXd R_big = Eigen::MatrixXd::Identity(max_meas_size, max_meas_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 4. Compute linear system for each feature, nullspace project, and reject
  const bool vop_active = vop_ && vop_->is_active();
  const double diag_time = state ? state->_timestamp : diag_nan();
  const double diag_roll = roll_deg_from_state(state);
  const double diag_bgz = (state && state->_imu) ? state->_imu->bias_g()(2) : diag_nan();
  std::vector<double> diag_residual_norms;
  std::vector<double> diag_whitened_norms;
  std::vector<double> diag_chi2s;
  std::vector<double> diag_thresholds;
  std::vector<double> diag_us;
  std::vector<double> diag_left_flags;
  int diag_num_accepted = 0;
  int diag_num_rejected = 0;
  int diag_high_residual_accepted = 0;
  auto record_visual_residual =
      [&](const std::shared_ptr<Feature> &feature, const Eigen::VectorXd &res,
          double chi2, double chi2_threshold, bool accepted,
          const std::string &reject_reason, double sigma_pix) {
        if (!visual_residual_diag_ || !visual_residual_diag_->enabled(diag_time))
          return;
        FeatureUvSummary uv = summarize_feature_uv(feature);
        LatestFeatureObs obs = latest_feature_obs(feature);
        const double residual_norm = res.norm();
        const double whitened_norm = (std::isfinite(chi2) && chi2 >= 0.0) ? std::sqrt(chi2) : diag_nan();
        diag_residual_norms.push_back(residual_norm);
        diag_whitened_norms.push_back(whitened_norm);
        diag_chi2s.push_back(chi2);
        diag_thresholds.push_back(chi2_threshold);
        if (std::isfinite(uv.u_mean)) {
          diag_us.push_back(uv.u_mean);
          diag_left_flags.push_back(uv.u_mean < 320.0 ? 1.0 : 0.0);
        }
        if (accepted) {
          diag_num_accepted++;
          if (std::isfinite(chi2) && std::isfinite(chi2_threshold) && chi2_threshold > 0.0 &&
              chi2 / chi2_threshold > 0.8) {
            diag_high_residual_accepted++;
          }
        } else {
          diag_num_rejected++;
        }

        VisualResidualDiag::FeatureRow row;
        row.time = diag_time;
        row.update_type = "SLAM";
        row.feature_id = feature ? feature->featid : 0;
        row.feature_kind = "slam";
        row.track_age = obs.track_age;
        row.num_measurements = uv.num_measurements;
        row.u_mean = uv.u_mean;
        row.v_mean = uv.v_mean;
        row.image_side = image_side(uv.u_mean);
        row.image_quadrant = image_quadrant(uv.u_mean, uv.v_mean);
        row.residual_norm = residual_norm;
        row.whitened_residual_norm = whitened_norm;
        row.chi2 = chi2;
        row.chi2_threshold = chi2_threshold;
        row.accepted_by_chi2 = accepted;
        row.rejected_by_chi2 = !accepted;
        row.reject_reason = reject_reason;
        row.measurement_sigma_px = sigma_pix;
        row.roll = diag_roll;
        row.bg_z = diag_bgz;
        visual_residual_diag_->log_feature(row);
      };
  struct AcceptedFeatureSystem {
    std::shared_ptr<Feature> feature;
    std::vector<std::shared_ptr<Type>> H_order;
    Eigen::MatrixXd H;
    Eigen::VectorXd res;
    double sigma_pix_sq = 1.0;
    double chi2 = diag_nan();
    double chi2_threshold = diag_nan();
    double yaw_contribution = diag_nan();
    double abs_contribution = diag_nan();
    LatestFeatureObs obs;
    SlamAnchorGeometryDiag geom;
    StateHelper::UpdateDiagnostics ekf_diag;
    bool regular_slam = false;
    bool capped = false;
    double info_alpha = 1.0;
    double info_sqrt_inv_alpha = 1.0;
  };
  std::vector<AcceptedFeatureSystem> accepted_systems;
  accepted_systems.reserve(feature_vec.size());

  auto compute_slam_info_alpha = [&](double chi2_ratio, bool accepted, bool regular_slam_feature) {
    if (!slam_info_cfg_.enabled || !accepted || !regular_slam_feature ||
        !std::isfinite(chi2_ratio)) {
      return 1.0;
    }
    if (chi2_ratio <= slam_info_cfg_.low_ratio)
      return 1.0;
    if (chi2_ratio >= slam_info_cfg_.high_ratio)
      return slam_info_cfg_.alpha_max;
    const double t = (chi2_ratio - slam_info_cfg_.low_ratio) /
                     (slam_info_cfg_.high_ratio - slam_info_cfg_.low_ratio);
    return 1.0 + t * (slam_info_cfg_.alpha_max - 1.0);
  };

  auto log_slam_info_reduction =
      [&](const std::shared_ptr<Feature> &feature, const Eigen::VectorXd &res,
          double chi2, double chi2_threshold, bool accepted,
          double alpha, double sqrt_inv_alpha) {
        if (!of_slam_info_reduction_diag_.is_open() ||
            diag_time < slam_info_diag_t0_ || diag_time > slam_info_diag_t1_)
          return;
        FeatureUvSummary uv = summarize_feature_uv(feature);
        LatestFeatureObs obs = latest_feature_obs(feature);
        const double chi2_ratio = (std::isfinite(chi2) && std::isfinite(chi2_threshold) &&
                                   chi2_threshold > 0.0) ? chi2 / chi2_threshold : diag_nan();
        const double residual_norm_before = res.norm();
        const double whitened_norm_before =
            (std::isfinite(chi2) && chi2 >= 0.0) ? std::sqrt(chi2) : diag_nan();
        const double residual_norm_after = residual_norm_before * sqrt_inv_alpha;
        const double whitened_norm_after =
            std::isfinite(whitened_norm_before) ? whitened_norm_before * sqrt_inv_alpha : diag_nan();
        of_slam_info_reduction_diag_ << std::fixed << std::setprecision(9)
            << diag_time << "," << (feature ? feature->featid : 0) << ","
            << chi2 << "," << chi2_threshold << "," << chi2_ratio << ","
            << (accepted ? 1 : 0) << "," << alpha << "," << sqrt_inv_alpha << ","
            << residual_norm_before << "," << whitened_norm_before << ","
            << residual_norm_after << "," << whitened_norm_after << ","
            << obs.track_age << "," << uv.num_measurements << ","
            << uv.u_mean << "," << uv.v_mean << "," << image_side(uv.u_mean) << ","
            << diag_roll << "," << yaw_cap_yaw_rate_degps_ << "\n";
      };

  auto compute_feature_yaw_contribution =
      [&](const std::vector<std::shared_ptr<Type>> &H_order,
          const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
          double sigma_pix_sq) -> double {
        if (res.rows() < 1 || H.rows() != res.rows())
          return diag_nan();
        Eigen::MatrixXd R_feat = sigma_pix_sq * Eigen::MatrixXd::Identity(res.rows(), res.rows());
        Eigen::VectorXd dx = StateHelper::compute_update_dx(
            state, H_order, H, res, R_feat,
            visual_yaw_update_mode_, visual_yaw_update_scale_,
            visual_global_yaw_oc_alpha_, visual_bgz_update_scale_);
        return StateHelper::yaw_delta_from_full_dx_deg(state, dx);
      };

  auto compute_slam_ekf_update_diag =
      [&](const std::vector<std::shared_ptr<Type>> &H_order,
          const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
          double sigma_pix_sq) {
        Eigen::MatrixXd R_feat = sigma_pix_sq * Eigen::MatrixXd::Identity(res.rows(), res.rows());
        return StateHelper::compute_update_diagnostics(
            state, H_order, H, res, R_feat,
            visual_yaw_update_mode_, visual_yaw_update_scale_,
            visual_global_yaw_oc_alpha_, visual_bgz_update_scale_);
      };

  auto log_slam_ekf_leverage =
      [&](const std::shared_ptr<Feature> &feature,
          const std::shared_ptr<Landmark> &landmark,
          const std::vector<std::shared_ptr<Type>> &H_order,
          const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
          double sigma_pix_sq, double chi2, double chi2_threshold,
          const StateHelper::UpdateDiagnostics &d,
          const LatestFeatureObs &obs,
          const SlamAnchorGeometryDiag &geom) {
        const double t = state ? state->_timestamp : diag_nan();
        if (!of_slam_ekf_leverage_diag_.is_open() ||
            t < slam_ekf_leverage_diag_t0_ || t > slam_ekf_leverage_diag_t1_)
          return;
        (void)landmark;
        (void)H_order;
        (void)H;
        (void)sigma_pix_sq;
        FeatureUvSummary uv = summarize_feature_uv(feature);
        const double chi2_ratio = (std::isfinite(chi2) && std::isfinite(chi2_threshold) &&
                                   chi2_threshold > 0.0) ? chi2 / chi2_threshold : diag_nan();
        const double dof = res.rows();
        const double nis_per_dof = dof > 0.0 ? chi2 / dof : diag_nan();
        const double whitened_residual_norm =
            (std::isfinite(chi2) && chi2 >= 0.0) ? std::sqrt(chi2) : diag_nan();
        of_slam_ekf_leverage_diag_ << std::fixed << std::setprecision(9)
            << t << "," << (feature ? feature->featid : 0) << ","
            << chi2 << "," << chi2_threshold << "," << chi2_ratio << ","
            << nis_per_dof << "," << dof << ","
            << res.norm() << "," << whitened_residual_norm << ","
            << d.dx_yaw_deg << "," << d.dx_bgz << "," << d.dx_pos_norm << ","
            << d.dx_landmark_norm << "," << d.dx_norm << ","
            << d.K_yaw_row_norm << "," << d.K_bgz_row_norm << ","
            << d.K_pos_row_norm << "," << d.K_landmark_row_norm << ","
            << d.H_norm << "," << d.whitened_H_norm << "," << d.H_yaw_col_norm << ","
            << d.H_bgz_col_norm << "," << d.H_pos_col_norm << ","
            << d.H_landmark_norm << "," << d.H_other_norm << ","
            << d.S_cond << "," << d.S_min_eig << "," << d.S_max_eig << ","
            << d.HPH_trace << "," << d.R_trace << "," << d.HPH_over_R << ","
            << d.P_yaw_var << "," << d.P_bgz_var << "," << d.P_pos_trace << ","
            << d.corr_yaw_bgz << "," << d.corr_yaw_px << "," << d.corr_yaw_py << ","
            << d.pose_landmark_cov_norm << ","
            << obs.track_age << "," << uv.num_measurements << ","
            << uv.u_mean << "," << uv.v_mean << "," << image_side(uv.u_mean) << ","
            << image_quadrant(obs.u, obs.v) << "," << diag_roll << ","
            << yaw_cap_yaw_rate_degps_ << ","
            << geom.anchor_clone_time << "," << geom.time_since_anchor << ","
            << geom.anchor_cam_id << "," << geom.representation << ","
            << geom.has_anchor_change << "," << geom.update_fail_count << ","
            << geom.obs_count << "," << geom.obs_span_s << ","
            << geom.bearing_spread_deg << "," << geom.landmark_global_norm << ","
            << geom.depth_current << "," << geom.inverse_depth_current << "\n";
      };

  auto log_slam_landmark_metadata =
      [&](const std::shared_ptr<Feature> &feature,
          const SlamLandmarkMetadata &meta,
          const LatestFeatureObs &obs,
          const SlamAnchorGeometryDiag &geom,
          const StateHelper::UpdateDiagnostics &d,
          double chi2, double chi2_threshold) {
        const double t = state ? state->_timestamp : diag_nan();
        if (!of_slam_landmark_metadata_diag_.is_open() ||
            t < slam_landmark_metadata_diag_t0_ || t > slam_landmark_metadata_diag_t1_)
          return;
        const double dof = obs.track_age > 0 ? diag_nan() : diag_nan();
        (void)dof;
        const double nis_per_dof =
            (std::isfinite(chi2) && chi2_threshold > 0.0 && feature != nullptr) ?
            chi2 / std::max(1, 2 * geom.obs_count) : diag_nan();
        const bool created_before_window =
            std::isfinite(meta.first_seen_time) && meta.first_seen_time < slam_landmark_metadata_diag_t0_;
        const bool updated_after_window_start =
            std::isfinite(meta.last_slam_update_time) && meta.last_slam_update_time >= slam_landmark_metadata_diag_t0_;
        of_slam_landmark_metadata_diag_ << std::fixed << std::setprecision(9)
            << t << "," << (feature ? feature->featid : 0) << ","
            << meta.first_seen_time << "," << meta.first_added_to_state_time << ","
            << meta.first_slam_update_time << "," << meta.last_slam_update_time << ","
            << meta.slam_update_count << "," << meta.total_update_obs_count << ","
            << obs.track_age << "," << geom.obs_count << ","
            << (created_before_window ? 1 : 0) << "," << (updated_after_window_start ? 1 : 0) << ","
            << meta.crosses_1139 << "," << meta.crosses_1182 << ","
            << geom.anchor_clone_time << "," << meta.first_anchor_clone_time << ","
            << meta.last_anchor_clone_time << "," << geom.time_since_anchor << ","
            << meta.anchor_cam_id << "," << meta.representation << ","
            << geom.has_anchor_change << "," << meta.anchor_change_count << ","
            << geom.update_fail_count << "," << geom.bearing_spread_deg << ","
            << geom.obs_span_s << "," << geom.landmark_global_norm << ","
            << geom.depth_current << "," << geom.inverse_depth_current << ","
            << d.dx_yaw_deg << "," << nis_per_dof << "," << d.pose_landmark_cov_norm << ","
            << image_side(obs.u) << "," << image_quadrant(obs.u, obs.v) << ","
            << obs.u << "," << obs.v << "," << diag_roll << ","
            << yaw_cap_yaw_rate_degps_ << "\n";
      };

  auto write_feature_yaw_diag =
      [&](const std::shared_ptr<Feature> &feature,
          const std::vector<std::shared_ptr<Type>> &H_order,
          const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
          double sigma_pix_sq, double chi2, double chi2_threshold,
          bool accepted, const std::string &reject_reason) {
        const double t = state ? state->_timestamp : diag_nan();
        if (!of_feature_yaw_contrib_diag_.is_open() ||
            t < feature_yaw_diag_t0_ || t > feature_yaw_diag_t1_)
          return;

        LatestFeatureObs obs = latest_feature_obs(feature);
        const double roll = roll_deg_from_state(state);
        const bool is_turn = std::isfinite(roll) && std::fabs(roll) > 20.0;
        const bool is_recovery = std::isfinite(roll) && std::fabs(roll) < 10.0;
        const bool created_before_diag = std::isfinite(obs.created_time) && obs.created_time < feature_yaw_diag_t0_;
        const double residual_u = res.rows() > 0 ? res(0) : diag_nan();
        const double residual_v = res.rows() > 1 ? res(1) : diag_nan();
        const double residual_norm = res.norm();
        const double whitened_residual_norm = std::isfinite(chi2) && chi2 >= 0.0 ? std::sqrt(chi2) : diag_nan();

        double H_yaw = 0.0;
        size_t col = 0;
        for (const auto &var : H_order) {
          if (var && var->size() >= 3 && col + 2 < (size_t)H.cols())
            H_yaw += H.col((Eigen::Index)col + 2).squaredNorm();
          col += var ? var->size() : 0;
        }
        H_yaw = H_yaw > 0.0 ? std::sqrt(H_yaw) : 0.0;

        double delta_yaw_contrib = diag_nan();
        if (accepted && res.rows() > 0 && H.rows() == res.rows()) {
          delta_yaw_contrib = compute_feature_yaw_contribution(H_order, H, res, sigma_pix_sq);
        }
        const int sign = std::isfinite(delta_yaw_contrib) ?
            ((delta_yaw_contrib > 0.0) ? 1 : ((delta_yaw_contrib < 0.0) ? -1 : 0)) : 0;
        const double abs_contrib = std::isfinite(delta_yaw_contrib) ? std::fabs(delta_yaw_contrib) : diag_nan();

        of_feature_yaw_contrib_diag_ << std::fixed << std::setprecision(6)
            << t << "," << (feature ? feature->featid : 0) << ","
            << obs.track_age << "," << obs.created_time << "," << obs.last_update_time << ","
            << obs.cam_id << "," << obs.u << "," << obs.v << ","
            << obs.prev_u << "," << obs.prev_v << "," << obs.du << "," << obs.dv << ","
            << image_side(obs.u) << "," << image_quadrant(obs.u, obs.v) << ","
            << (created_before_diag ? 1 : 0) << "," << (is_turn ? 1 : 0) << ","
            << roll << "," << diag_nan() << ","
            << (is_turn ? 1 : 0) << "," << (is_recovery ? 1 : 0) << ","
            << residual_u << "," << residual_v << "," << residual_norm << ","
            << whitened_residual_norm << "," << chi2 << "," << chi2_threshold << ","
            << (accepted ? 1 : 0) << "," << (accepted ? 0 : 1) << ","
            << reject_reason << "," << H_yaw << "," << H_yaw << ","
            << delta_yaw_contrib << "," << sign << "," << abs_contrib << ","
            << -1 << "," << diag_nan() << "," << diag_nan() << ","
            << diag_nan() << "," << diag_nan() << "\n";
      };

  int regular_features_before_refresh = 0;
  for (const auto &feature : feature_vec) {
    if (feature && (int)feature->featid >= state->_options.max_aruco_features &&
        state->_features_SLAM.find(feature->featid) != state->_features_SLAM.end())
      regular_features_before_refresh++;
  }
  int regular_features_refreshed = 0;

  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Ensure we have the landmark and it is the same
    assert(state->_features_SLAM.find((*it2)->featid) != state->_features_SLAM.end());
    assert(state->_features_SLAM.at((*it2)->featid)->_featid == (*it2)->featid);

    // Get our landmark from the state
    std::shared_ptr<Landmark> landmark = state->_features_SLAM.at((*it2)->featid);
    const bool regular_state_slam_feature = (int)(*it2)->featid >= state->_options.max_aruco_features;

    if (slam_geometry_refresh_cfg_.enabled && regular_state_slam_feature) {
      LatestFeatureObs refresh_obs = latest_feature_obs(*it2);
      SlamAnchorGeometryDiag refresh_geom = slam_anchor_geometry_diag(state, *it2, landmark, refresh_obs);
      auto &meta = slam_landmark_metadata_[(*it2)->featid];
      if (!std::isfinite(meta.first_seen_time))
        meta.first_seen_time = std::isfinite(refresh_obs.created_time) ? refresh_obs.created_time : diag_time;
      if (!std::isfinite(meta.first_added_to_state_time))
        meta.first_added_to_state_time = diag_time;
      if (!std::isfinite(meta.first_anchor_clone_time))
        meta.first_anchor_clone_time = refresh_geom.anchor_clone_time;
      const double age_since_added =
          std::isfinite(meta.first_added_to_state_time) ? diag_time - meta.first_added_to_state_time : diag_nan();
      const bool far_depth =
          std::isfinite(refresh_geom.depth_current) &&
          refresh_geom.depth_current > slam_geometry_refresh_cfg_.min_depth_current;
      const bool old_enough =
          std::isfinite(age_since_added) &&
          age_since_added > slam_geometry_refresh_cfg_.min_age_since_added;
      const bool anchor_ok =
          !slam_geometry_refresh_cfg_.require_anchor_change || refresh_geom.has_anchor_change != 0;
      double refresh_pose_lm_cov_norm = diag_nan();
      if (far_depth && old_enough && anchor_ok) {
        const double refresh_q_lm_cov = covariance_block_norm(state, state->_imu->q(), landmark);
        const double refresh_p_lm_cov = covariance_block_norm(state, state->_imu->p(), landmark);
        refresh_pose_lm_cov_norm =
            std::sqrt(refresh_q_lm_cov * refresh_q_lm_cov + refresh_p_lm_cov * refresh_p_lm_cov);
      }
      const bool cov_ok =
          slam_geometry_refresh_cfg_.min_pose_landmark_cov_norm <= 0.0 ||
          (std::isfinite(refresh_pose_lm_cov_norm) &&
           refresh_pose_lm_cov_norm > slam_geometry_refresh_cfg_.min_pose_landmark_cov_norm);
      const bool feature_count_ok =
          slam_geometry_refresh_cfg_.min_regular_features_after_refresh <= 0 ||
          (regular_features_before_refresh - regular_features_refreshed - 1) >=
              slam_geometry_refresh_cfg_.min_regular_features_after_refresh;
      const bool refresh = far_depth && old_enough && anchor_ok && cov_ok && feature_count_ok;
      if (of_slam_geometry_refresh_diag_.is_open() && refresh) {
        of_slam_geometry_refresh_diag_ << std::fixed << std::setprecision(9)
            << diag_time << "," << (*it2)->featid << ",1,far_depth_lifecycle_cov_guard,"
            << age_since_added << "," << meta.first_added_to_state_time << ","
            << refresh_geom.depth_current << "," << refresh_geom.inverse_depth_current << ","
            << refresh_geom.landmark_global_norm << "," << refresh_geom.time_since_anchor << ","
            << refresh_pose_lm_cov_norm << ","
            << slam_geometry_refresh_cfg_.min_pose_landmark_cov_norm << ","
            << slam_geometry_refresh_cfg_.min_regular_features_after_refresh << ","
            << regular_features_before_refresh << ","
            << (regular_features_before_refresh - regular_features_refreshed - 1) << ","
            << refresh_geom.has_anchor_change << "," << meta.anchor_change_count << ","
            << state->max_covariance_size() << "," << state->_features_SLAM.size() << ","
            << landmark->id() << "," << landmark->size() << ","
            << (landmark->should_marg ? 1 : 0) << "," << ((*it2)->to_delete ? 1 : 0) << ","
            << landmark->_anchor_cam_id << "," << landmark->_anchor_clone_timestamp << ","
            << slam_geometry_refresh_cfg_.min_depth_current << ","
            << slam_geometry_refresh_cfg_.min_age_since_added << ","
            << (slam_geometry_refresh_cfg_.require_anchor_change ? 1 : 0) << ","
            << image_side(refresh_obs.u) << "," << image_quadrant(refresh_obs.u, refresh_obs.v) << ","
            << refresh_obs.u << "," << refresh_obs.v << "\n";
      }
      if (refresh) {
        landmark->should_marg = true;
        (*it2)->to_delete = true;
        regular_features_refreshed++;
        it2 = feature_vec.erase(it2);
        continue;
      }
    }

    // Convert the state landmark into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    feat.feat_representation = landmark->_feat_representation;
    if (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = landmark->_anchor_cam_id;
      feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
      feat.p_FinA = landmark->get_xyz(false);
      feat.p_FinA_fej = landmark->get_xyz(true);
    } else {
      feat.p_FinG = landmark->get_xyz(false);
      feat.p_FinG_fej = landmark->get_xyz(true);
    }

    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    // Place Jacobians in one big Jacobian, since the landmark is already in our state vector
    Eigen::MatrixXd H_xf = H_x;
    if (landmark->_feat_representation == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {

      // Append the Jacobian in respect to the depth of the feature
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + 1);
      H_xf.block(0, H_x.cols(), H_x.rows(), 1) = H_f.block(0, H_f.cols() - 1, H_f.rows(), 1);
      H_f.conservativeResize(H_f.rows(), H_f.cols() - 1);

      // Nullspace project the bearing portion
      // This takes into account that we have marginalized the bearing already
      // Thus this is crucial to ensuring estimator consistency as we are not taking the bearing to be true
      UpdaterHelper::nullspace_project_inplace(H_f, H_xf, res);

    } else {

      // Else we have the full feature in our state, so just append it
      H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
      H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;
    }

    // Append to our Jacobian order vector
    std::vector<std::shared_ptr<Type>> Hxf_order = Hx_order;
    Hxf_order.push_back(landmark);

    // Optionally apply VOP projection before chi2 gating.
    Eigen::MatrixXd H_xf_for_gate = H_xf;
    if (vop_active) {
      std::vector<int> H_id = VisualObservabilityPolicy::build_H_id(Hxf_order);
      H_xf_for_gate = vop_->apply(H_xf, Hxf_order, H_id, state);
    }

    // Chi2 distance check (uses OC-projected H when vop active)
    double sigma_pix_sq =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.sigma_pix_sq : _options_slam.sigma_pix_sq;
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hxf_order);
    Eigen::MatrixXd S = H_xf_for_gate * P_marg * H_xf_for_gate.transpose();
    S.diagonal() += sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is more)
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we should delete or not
    double chi2_multipler =
        ((int)feat.featid < state->_options.max_aruco_features) ? _options_aruco.chi2_multipler : _options_slam.chi2_multipler;
    const double chi2_threshold = chi2_multipler * chi2_check;
    const double chi2_ratio = (chi2_threshold > 0.0) ? chi2 / chi2_threshold : diag_nan();
    const bool regular_slam_feature = (int)feat.featid >= state->_options.max_aruco_features;
    if (chi2 > chi2_multipler * chi2_check) {
      record_visual_residual(*it2, res, chi2, chi2_multipler * chi2_check,
                             false, "chi2", std::sqrt(sigma_pix_sq));
      log_slam_info_reduction(*it2, res, chi2, chi2_threshold, false, 1.0, 1.0);
      write_feature_yaw_diag(*it2, Hxf_order, H_xf_for_gate, res, sigma_pix_sq,
                              chi2, chi2_multipler * chi2_check, false, "chi2");
      if ((int)feat.featid < state->_options.max_aruco_features) {
        PRINT_WARNING(YELLOW "[SLAM-UP]: rejecting aruco tag %d for chi2 thresh (%.3f > %.3f)\n" RESET, (int)feat.featid, chi2,
                      chi2_multipler * chi2_check);
      } else {
        landmark->update_fail_count++;
      }
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      continue;
    }

    // Debug print when we are going to update the aruco tags
    if ((int)feat.featid < state->_options.max_aruco_features) {
      PRINT_DEBUG("[SLAM-UP]: accepted aruco tag %d for chi2 thresh (%.3f < %.3f)\n", (int)feat.featid, chi2, chi2_multipler * chi2_check);
    }
    write_feature_yaw_diag(*it2, Hxf_order, H_xf_for_gate, res, sigma_pix_sq,
                           chi2, chi2_multipler * chi2_check, true, "");
    LatestFeatureObs accepted_obs = latest_feature_obs(*it2);
    SlamAnchorGeometryDiag accepted_geom = slam_anchor_geometry_diag(state, *it2, landmark, accepted_obs);
    StateHelper::UpdateDiagnostics accepted_diag;
    const bool need_accepted_ekf_diag =
        regular_slam_feature &&
        ((of_slam_ekf_leverage_diag_.is_open() &&
          diag_time >= slam_ekf_leverage_diag_t0_ && diag_time <= slam_ekf_leverage_diag_t1_) ||
         (of_slam_landmark_metadata_diag_.is_open() &&
          diag_time >= slam_landmark_metadata_diag_t0_ && diag_time <= slam_landmark_metadata_diag_t1_) ||
         (of_slam_stacked_ekf_diag_.is_open() &&
          diag_time >= slam_stacked_ekf_diag_t0_ && diag_time <= slam_stacked_ekf_diag_t1_));
    if (need_accepted_ekf_diag) {
      accepted_diag = compute_slam_ekf_update_diag(Hxf_order, H_xf_for_gate, res, sigma_pix_sq);
    }
    if (regular_slam_feature) {
      log_slam_ekf_leverage(*it2, landmark, Hxf_order, H_xf_for_gate, res,
                            sigma_pix_sq, chi2, chi2_threshold,
                            accepted_diag, accepted_obs, accepted_geom);
    }
    record_visual_residual(*it2, res, chi2, chi2_multipler * chi2_check,
                            true, "", std::sqrt(sigma_pix_sq));
    const double info_alpha = compute_slam_info_alpha(chi2_ratio, true, regular_slam_feature);
    const double info_sqrt_inv_alpha = 1.0 / std::sqrt(std::max(1.0, info_alpha));
    log_slam_info_reduction(*it2, res, chi2, chi2_threshold, true,
                            info_alpha, info_sqrt_inv_alpha);

    if (regular_slam_feature) {
      auto &meta = slam_landmark_metadata_[feat.featid];
      if (!std::isfinite(meta.first_seen_time))
        meta.first_seen_time = std::isfinite(accepted_obs.created_time) ? accepted_obs.created_time : diag_time;
      if (!std::isfinite(meta.first_added_to_state_time))
        meta.first_added_to_state_time = diag_time;
      if (!std::isfinite(meta.first_slam_update_time))
        meta.first_slam_update_time = diag_time;
      if (!std::isfinite(meta.first_anchor_clone_time))
        meta.first_anchor_clone_time = accepted_geom.anchor_clone_time;
      if (std::isfinite(meta.last_anchor_clone_time) &&
          std::isfinite(accepted_geom.anchor_clone_time) &&
          std::fabs(meta.last_anchor_clone_time - accepted_geom.anchor_clone_time) > 1e-9) {
        meta.anchor_change_count++;
      }
      meta.last_anchor_clone_time = accepted_geom.anchor_clone_time;
      meta.anchor_cam_id = accepted_geom.anchor_cam_id;
      meta.representation = accepted_geom.representation;
      meta.last_slam_update_time = diag_time;
      meta.slam_update_count++;
      meta.total_update_obs_count += accepted_geom.obs_count;
      if (std::isfinite(meta.first_seen_time)) {
        if (meta.first_seen_time <= 1139.0 && diag_time >= 1139.0)
          meta.crosses_1139 = 1;
        if (meta.first_seen_time <= 1182.0 && diag_time >= 1182.0)
          meta.crosses_1182 = 1;
      }
      log_slam_landmark_metadata(*it2, meta, accepted_obs, accepted_geom,
                                 accepted_diag, chi2, chi2_threshold);
    }

    AcceptedFeatureSystem sys;
    sys.feature = *it2;
    sys.H_order = Hxf_order;
    sys.H = H_xf_for_gate;
    sys.res = res;
    sys.sigma_pix_sq = sigma_pix_sq;
    sys.chi2 = chi2;
    sys.chi2_threshold = chi2_multipler * chi2_check;
    sys.yaw_contribution = compute_feature_yaw_contribution(Hxf_order, H_xf_for_gate, res, sigma_pix_sq);
    sys.abs_contribution = std::isfinite(sys.yaw_contribution) ? std::fabs(sys.yaw_contribution) : 0.0;
    sys.obs = accepted_obs;
    sys.geom = accepted_geom;
    sys.ekf_diag = accepted_diag;
    sys.regular_slam = regular_slam_feature;
    sys.info_alpha = info_alpha;
    sys.info_sqrt_inv_alpha = info_sqrt_inv_alpha;
    accepted_systems.push_back(std::move(sys));
    it2++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  if (visual_residual_diag_ && visual_residual_diag_->summary_enabled(diag_time)) {
    VisualResidualDiag::FrameSummary summary;
    summary.time = diag_time;
    summary.update_type = "SLAM";
    summary.num_features = diag_num_accepted + diag_num_rejected;
    summary.num_accepted = diag_num_accepted;
    summary.num_rejected = diag_num_rejected;
    summary.mean_residual_norm = VisualResidualDiag::mean(diag_residual_norms);
    summary.p95_residual_norm = VisualResidualDiag::percentile(diag_residual_norms, 0.95);
    summary.mean_whitened_residual_norm = VisualResidualDiag::mean(diag_whitened_norms);
    summary.p95_whitened_residual_norm = VisualResidualDiag::percentile(diag_whitened_norms, 0.95);
    summary.mean_chi2 = VisualResidualDiag::mean(diag_chi2s);
    summary.p95_chi2 = VisualResidualDiag::percentile(diag_chi2s, 0.95);
    summary.chi2_threshold = VisualResidualDiag::mean(diag_thresholds);
    summary.accepted_ratio = summary.num_features > 0 ?
        (double)summary.num_accepted / (double)summary.num_features : diag_nan();
    summary.high_residual_accepted_count = diag_high_residual_accepted;
    summary.high_residual_accepted_ratio = summary.num_accepted > 0 ?
        (double)diag_high_residual_accepted / (double)summary.num_accepted : diag_nan();
    summary.u_mean = VisualResidualDiag::mean(diag_us);
    summary.left_frac = VisualResidualDiag::mean(diag_left_flags);
    summary.roll = diag_roll;
    visual_residual_diag_->log_summary(summary);
  }

  const double roll_deg = roll_deg_from_state(state);
  const bool yaw_rate_ok = std::isfinite(yaw_cap_yaw_rate_degps_) &&
                           std::fabs(yaw_cap_yaw_rate_degps_) <= yaw_cap_cfg_.yawrate_degps;
  const bool roll_ok = std::isfinite(roll_deg) && std::fabs(roll_deg) <= yaw_cap_cfg_.roll_deg;
  const bool yaw_error_ok = yaw_cap_yaw_error_valid_ && std::isfinite(yaw_cap_yaw_error_deg_) &&
                            std::fabs(yaw_cap_yaw_error_deg_) > 1e-9;
  const bool cap_gate = yaw_cap_cfg_.enabled &&
                        state->_timestamp >= yaw_cap_cfg_.t0 &&
                        state->_timestamp <= yaw_cap_cfg_.t1 &&
                        roll_ok && yaw_rate_ok && yaw_error_ok &&
                        yaw_cap_cfg_.topk > 0 &&
                        yaw_cap_cfg_.ratio > 0.0 && yaw_cap_cfg_.ratio < 1.0 &&
                        yaw_cap_cfg_.mode == "soft_scale";

  double frame_abs_contrib_sum = 0.0;
  for (const auto &sys : accepted_systems)
    frame_abs_contrib_sum += sys.abs_contribution;

  std::vector<size_t> same_sign_indices;
  if (cap_gate) {
    for (size_t i = 0; i < accepted_systems.size(); i++) {
      const double c = accepted_systems[i].yaw_contribution;
      if (std::isfinite(c) && c * yaw_cap_yaw_error_deg_ > 0.0)
        same_sign_indices.push_back(i);
    }
    std::sort(same_sign_indices.begin(), same_sign_indices.end(),
              [&](size_t a, size_t b) {
                return accepted_systems[a].abs_contribution > accepted_systems[b].abs_contribution;
              });
    const size_t n_cap = std::min((size_t)yaw_cap_cfg_.topk, same_sign_indices.size());
    for (size_t i = 0; i < n_cap; i++)
      accepted_systems[same_sign_indices[i]].capped = true;
  }

  double capped_abs_contrib_sum = 0.0;
  double capped_u_sum = 0.0;
  double capped_v_sum = 0.0;
  int capped_uv_count = 0;
  std::ostringstream capped_ids;
  std::unordered_map<std::string, double> side_abs, quadrant_abs;
  int capped_count = 0;
  for (const auto &sys : accepted_systems) {
    const std::string side = image_side(sys.obs.u);
    const std::string quadrant = image_quadrant(sys.obs.u, sys.obs.v);
    side_abs[side] += sys.abs_contribution;
    quadrant_abs[quadrant] += sys.abs_contribution;
    if (!sys.capped)
      continue;
    if (capped_count > 0)
      capped_ids << ";";
    capped_ids << (sys.feature ? sys.feature->featid : 0);
    capped_count++;
    capped_abs_contrib_sum += sys.abs_contribution;
    if (std::isfinite(sys.obs.u) && std::isfinite(sys.obs.v)) {
      capped_u_sum += sys.obs.u;
      capped_v_sum += sys.obs.v;
      capped_uv_count++;
    }
  }

  auto dominant_label = [](const std::unordered_map<std::string, double> &m) {
    std::string best = "unknown";
    double best_val = -1.0;
    for (const auto &kv : m) {
      if (kv.second > best_val) {
        best = kv.first;
        best_val = kv.second;
      }
    }
    return best;
  };
  const std::string dominant_side = dominant_label(side_abs);
  const std::string dominant_quadrant = dominant_label(quadrant_abs);
  const double cap_weight_scale = std::sqrt(std::max(0.0, std::min(1.0, yaw_cap_cfg_.ratio)));

  auto build_big_system =
      [&](bool apply_cap,
          bool apply_info_reduction,
          Eigen::MatrixXd &H_out,
          Eigen::VectorXd &res_out,
          Eigen::MatrixXd &R_out,
          std::vector<std::shared_ptr<Type>> &order_out,
          size_t &meas_out,
          size_t &jacob_out) {
        H_out = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
        res_out = Eigen::VectorXd::Zero(max_meas_size);
        R_out = Eigen::MatrixXd::Identity(max_meas_size, max_meas_size);
        order_out.clear();
        std::unordered_map<std::shared_ptr<Type>, size_t> mapping;
        meas_out = 0;
        jacob_out = 0;
        for (const auto &sys : accepted_systems) {
          Eigen::MatrixXd H_feat = sys.H;
          Eigen::VectorXd res_feat = sys.res;
          if (apply_info_reduction && sys.info_sqrt_inv_alpha < 1.0) {
            H_feat *= sys.info_sqrt_inv_alpha;
            res_feat *= sys.info_sqrt_inv_alpha;
          }
          if (apply_cap && sys.capped) {
            H_feat *= cap_weight_scale;
            res_feat *= cap_weight_scale;
          }
          size_t ct_hx_local = 0;
          for (const auto &var : sys.H_order) {
            if (mapping.find(var) == mapping.end()) {
              mapping.insert({var, jacob_out});
              order_out.push_back(var);
              jacob_out += var->size();
            }
            H_out.block(meas_out, mapping[var], H_feat.rows(), var->size()) =
                H_feat.block(0, ct_hx_local, H_feat.rows(), var->size());
            ct_hx_local += var->size();
          }
          R_out.block(meas_out, meas_out, res_feat.rows(), res_feat.rows()) *= sys.sigma_pix_sq;
          res_out.block(meas_out, 0, res_feat.rows(), 1) = res_feat;
          meas_out += res_feat.rows();
        }
        if (meas_out > 0) {
          H_out.conservativeResize(meas_out, jacob_out);
          res_out.conservativeResize(meas_out, 1);
          R_out.conservativeResize(meas_out, meas_out);
        }
      };

  Eigen::MatrixXd raw_Hx_big, raw_R_big;
  Eigen::VectorXd raw_res_big;
  std::vector<std::shared_ptr<Type>> raw_Hx_order;
  size_t raw_ct_meas = 0, raw_ct_jacob = 0;
  build_big_system(false, false, raw_Hx_big, raw_res_big, raw_R_big, raw_Hx_order, raw_ct_meas, raw_ct_jacob);

  build_big_system(true, true, Hx_big, res_big, R_big, Hx_order_big, ct_meas, ct_jacob);

  // We have appended all accepted features to our Hx_big/res_big systems.
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);

  const StateHelper::VisualYawUpdateMode update_mode =
      vop_active ? StateHelper::VisualYawUpdateMode::ORIGINAL : visual_yaw_update_mode_;
  const double update_scale = vop_active ? 1.0 : visual_yaw_update_scale_;
  const double update_alpha = vop_active ? 0.0 : visual_global_yaw_oc_alpha_;
  double raw_delta_yaw_pred = diag_nan();
  double capped_delta_yaw_pred = diag_nan();
  if (raw_ct_meas > 0) {
    Eigen::VectorXd raw_dx = StateHelper::compute_update_dx(
        state, raw_Hx_order, raw_Hx_big, raw_res_big, raw_R_big,
        update_mode, update_scale, update_alpha, visual_bgz_update_scale_);
    raw_delta_yaw_pred = StateHelper::yaw_delta_from_full_dx_deg(state, raw_dx);
    Eigen::VectorXd capped_dx = StateHelper::compute_update_dx(
        state, Hx_order_big, Hx_big, res_big, R_big,
        update_mode, update_scale, update_alpha, visual_bgz_update_scale_);
    capped_delta_yaw_pred = StateHelper::yaw_delta_from_full_dx_deg(state, capped_dx);
  }

  const bool write_stacked_diag =
      of_slam_stacked_ekf_diag_.is_open() &&
      state->_timestamp >= slam_stacked_ekf_diag_t0_ &&
      state->_timestamp <= slam_stacked_ekf_diag_t1_;
  StateHelper::UpdateDiagnostics stacked_raw_diag;
  StateHelper::UpdateDiagnostics stacked_projected_diag;
  Eigen::MatrixXd Hx_projected_for_diag;
  double chi2_total_before_projection = diag_nan();
  double chi2_total_after_projection = diag_nan();
  double HN_norm_before = diag_nan();
  double HN_norm_after = diag_nan();
  double HN_rel_before = diag_nan();
  double HN_rel_after = diag_nan();
  int rank_before = -1;
  int rank_after = -1;
  int num_regular_slam_features = 0;
  int num_landmarks_involved = 0;
  int num_clones_involved = 0;
  double landmark_cov_norm = diag_nan();
  double pose_cov_norm = diag_nan();
  std::vector<double> frame_depths;
  std::vector<double> frame_inv_depths;
  std::vector<double> frame_bearing_spreads;
  int count_far_depth = 0;
  int count_low_parallax = 0;
  int count_high_pose_landmark_cov = 0;
  int count_cross_turn_landmarks = 0;
  std::unordered_map<size_t, Eigen::VectorXd> landmark_values_before;
  double bgz_before_update = diag_nan();
  Eigen::Vector3d pos_before_update = Eigen::Vector3d::Zero();
  if (state && state->_imu) {
    bgz_before_update = state->_imu->bias_g()(2);
    pos_before_update = state->_imu->pos();
  }
  if (write_stacked_diag) {
    Hx_projected_for_diag = project_for_update_diag_local(state, Hx_order_big, Hx_big, update_mode, update_alpha);
    stacked_raw_diag = StateHelper::compute_update_diagnostics(
        state, Hx_order_big, Hx_big, res_big, R_big,
        StateHelper::VisualYawUpdateMode::ORIGINAL, 1.0, 0.0, visual_bgz_update_scale_);
    stacked_projected_diag = StateHelper::compute_update_diagnostics(
        state, Hx_order_big, Hx_big, res_big, R_big,
        update_mode, update_scale, update_alpha, visual_bgz_update_scale_);
    chi2_total_before_projection = mahalanobis_chi2_local(state, Hx_order_big, Hx_big, res_big, R_big);
    chi2_total_after_projection = mahalanobis_chi2_local(state, Hx_order_big, Hx_projected_for_diag, res_big, R_big);
    const bool use_fej_gauge = update_mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
    HN_norm_before = HN_norm_local(state, Hx_order_big, Hx_big, use_fej_gauge);
    HN_norm_after = HN_norm_local(state, Hx_order_big, Hx_projected_for_diag, use_fej_gauge);
    HN_rel_before = (Hx_big.norm() > 1e-18) ? HN_norm_before / Hx_big.norm() : diag_nan();
    HN_rel_after = (Hx_projected_for_diag.norm() > 1e-18) ? HN_norm_after / Hx_projected_for_diag.norm() : diag_nan();
    rank_before = guarded_matrix_rank(Hx_big);
    rank_after = guarded_matrix_rank(Hx_projected_for_diag);

    std::set<size_t> landmark_ids;
    std::set<double> clone_times;
    double lm_cov_sq = 0.0;
    double pose_cov_sq = 0.0;
    for (const auto &var : Hx_order_big) {
      auto lm = std::dynamic_pointer_cast<Landmark>(var);
      if (lm) {
        landmark_ids.insert(lm->_featid);
        landmark_values_before[lm->_featid] = lm->value();
        const double n = covariance_block_norm(state, lm, lm);
        lm_cov_sq += n * n;
        continue;
      }
      if (state && state->_imu &&
          (var == state->_imu || var == state->_imu->pose() ||
           var == state->_imu->q() || var == state->_imu->p())) {
        const double n = covariance_block_norm(state, var, var);
        pose_cov_sq += n * n;
        continue;
      }
      for (const auto &clone : state->_clones_IMU) {
        const auto &pose = clone.second;
        if (pose && (var == pose || var == pose->q() || var == pose->p())) {
          clone_times.insert(clone.first);
          const double n = covariance_block_norm(state, var, var);
          pose_cov_sq += n * n;
          break;
        }
      }
    }
    num_landmarks_involved = (int)landmark_ids.size();
    num_clones_involved = (int)clone_times.size();
    landmark_cov_norm = std::sqrt(std::max(0.0, lm_cov_sq));
    pose_cov_norm = std::sqrt(std::max(0.0, pose_cov_sq));

    for (const auto &sys : accepted_systems) {
      if (sys.regular_slam)
        num_regular_slam_features++;
      if (std::isfinite(sys.geom.depth_current)) {
        frame_depths.push_back(sys.geom.depth_current);
        if (std::fabs(sys.geom.depth_current) > 400.0)
          count_far_depth++;
      }
      if (std::isfinite(sys.geom.inverse_depth_current))
        frame_inv_depths.push_back(sys.geom.inverse_depth_current);
      if (std::isfinite(sys.geom.bearing_spread_deg)) {
        frame_bearing_spreads.push_back(sys.geom.bearing_spread_deg);
        if (sys.geom.bearing_spread_deg < 0.05)
          count_low_parallax++;
      }
      if (sys.ekf_diag.valid && sys.ekf_diag.pose_landmark_cov_norm > 0.002)
        count_high_pose_landmark_cov++;
      if (sys.feature) {
        const auto it_meta = slam_landmark_metadata_.find(sys.feature->featid);
        if (it_meta != slam_landmark_metadata_.end() &&
            (it_meta->second.crosses_1139 || it_meta->second.crosses_1182))
          count_cross_turn_landmarks++;
      }
    }
  }

  // 5. With all good SLAM features update the state.
  // VOP already projected H into Hx_big — use ORIGINAL to avoid double application.
  const double yaw_before_update = yaw_deg_from_state(state);
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big,
                         update_mode, update_scale, update_alpha,
                         visual_bgz_update_scale_);
  const double yaw_after_update = yaw_deg_from_state(state);
  const double actual_delta_yaw_update =
      (std::isfinite(yaw_before_update) && std::isfinite(yaw_after_update)) ?
      wrap_deg_local(yaw_after_update - yaw_before_update) : diag_nan();

  if (write_stacked_diag) {
    const double actual_dx_bgz =
        (state && state->_imu && std::isfinite(bgz_before_update)) ?
        state->_imu->bias_g()(2) - bgz_before_update : diag_nan();
    const double actual_dx_pos_norm =
        (state && state->_imu) ? (state->_imu->pos() - pos_before_update).norm() : diag_nan();
    double lm_dx_sq = 0.0;
    for (const auto &var : Hx_order_big) {
      auto lm = std::dynamic_pointer_cast<Landmark>(var);
      if (!lm)
        continue;
      const auto it = landmark_values_before.find(lm->_featid);
      if (it == landmark_values_before.end() || it->second.rows() != lm->value().rows())
        continue;
      lm_dx_sq += (lm->value() - it->second).squaredNorm();
    }
    const double actual_dx_landmark_norm = std::sqrt(std::max(0.0, lm_dx_sq));
    const double nis_before = ct_meas > 0 ? chi2_total_before_projection / (double)ct_meas : diag_nan();
    const double nis_after = ct_meas > 0 ? chi2_total_after_projection / (double)ct_meas : diag_nan();
    const bool oc_projection_applied =
        update_mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION ||
        update_mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
    const bool fej_used =
        update_mode == StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
    const bool final_ekf_uses_projected = vop_active || oc_projection_applied;
    of_slam_stacked_ekf_diag_ << std::fixed << std::setprecision(9)
        << state->_timestamp << "," << accepted_systems.size() << ","
        << num_regular_slam_features << "," << ct_meas << "," << ct_jacob << ","
        << num_landmarks_involved << "," << num_clones_involved << ","
        << chi2_total_before_projection << "," << chi2_total_after_projection << ","
        << nis_before << "," << nis_after << ","
        << actual_delta_yaw_update << "," << stacked_projected_diag.dx_yaw_deg << ","
        << raw_delta_yaw_pred << "," << actual_dx_bgz << ","
        << stacked_projected_diag.dx_bgz << "," << actual_dx_pos_norm << ","
        << stacked_projected_diag.dx_pos_norm << "," << actual_dx_landmark_norm << ","
        << stacked_projected_diag.dx_landmark_norm << "," << (fej_used ? 1 : 0) << ","
        << (oc_projection_applied ? 1 : 0) << "," << update_alpha << ","
        << (vop_active ? 1 : 0) << "," << (final_ekf_uses_projected ? 1 : 0) << ","
        << HN_norm_before << "," << HN_norm_after << ","
        << HN_rel_before << "," << HN_rel_after << ","
        << stacked_raw_diag.H_yaw_col_norm << "," << stacked_projected_diag.H_yaw_col_norm << ","
        << rank_before << "," << rank_after << ","
        << res_big.norm() << "," << res_big.norm() << ","
        << Hx_big.norm() << "," << Hx_projected_for_diag.norm() << ","
        << stacked_projected_diag.whitened_H_norm << ","
        << stacked_projected_diag.K_yaw_row_norm << ","
        << stacked_projected_diag.K_bgz_row_norm << ","
        << stacked_projected_diag.K_pos_row_norm << ","
        << stacked_projected_diag.K_landmark_row_norm << ","
        << stacked_projected_diag.dx_yaw_deg << ","
        << stacked_projected_diag.dx_bgz << ","
        << stacked_projected_diag.dx_pos_norm << ","
        << stacked_projected_diag.S_cond << "," << stacked_projected_diag.HPH_over_R << ","
        << stacked_projected_diag.P_yaw_var << "," << stacked_projected_diag.P_bgz_var << ","
        << stacked_projected_diag.corr_yaw_bgz << "," << stacked_projected_diag.corr_yaw_px << ","
        << stacked_projected_diag.corr_yaw_py << ","
        << stacked_projected_diag.pose_landmark_cov_norm << ","
        << landmark_cov_norm << "," << pose_cov_norm << ","
        << percentile_or_nan(frame_depths, 0.50) << ","
        << percentile_or_nan(frame_depths, 0.95) << ","
        << percentile_or_nan(frame_inv_depths, 0.50) << ","
        << percentile_or_nan(frame_bearing_spreads, 0.50) << ","
        << percentile_or_nan(frame_bearing_spreads, 0.95) << ","
        << count_far_depth << "," << count_low_parallax << ","
        << count_high_pose_landmark_cov << "," << count_cross_turn_landmarks << "\n";
  }

  if (of_yaw_contrib_cap_diag_.is_open()) {
    const double mean_u_capped = capped_uv_count > 0 ? capped_u_sum / capped_uv_count : diag_nan();
    const double mean_v_capped = capped_uv_count > 0 ? capped_v_sum / capped_uv_count : diag_nan();
    const double cap_abs_ratio = frame_abs_contrib_sum > 0.0 ? capped_abs_contrib_sum / frame_abs_contrib_sum : diag_nan();
    of_yaw_contrib_cap_diag_ << std::fixed << std::setprecision(6)
        << state->_timestamp << "," << roll_deg << "," << yaw_cap_yaw_rate_degps_ << ","
        << yaw_cap_yaw_error_deg_ << "," << ((cap_gate && capped_count > 0) ? 1 : 0) << ","
        << accepted_systems.size() << "," << same_sign_indices.size() << ","
        << capped_count << "," << capped_abs_contrib_sum << ","
        << frame_abs_contrib_sum << "," << cap_abs_ratio << ","
        << raw_delta_yaw_pred << "," << capped_delta_yaw_pred << ","
        << actual_delta_yaw_update << "," << dominant_side << ","
        << dominant_quadrant << "," << mean_u_capped << ","
        << mean_v_capped << "," << capped_ids.str() << "\n";
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[SLAM-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds creating linear system\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[SLAM-UP]: %.4f seconds to update (%d feats of %d size)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size(),
            (int)Hx_big.rows());
  PRINT_ALL("[SLAM-UP]: %.4f seconds total\n", (rT3 - rT1).total_microseconds() * 1e-6);
}

void UpdaterSLAM::change_anchors(std::shared_ptr<State> state) {

  // Return if we do not have enough clones
  if ((int)state->_clones_IMU.size() <= state->_options.max_clone_size) {
    return;
  }

  // Get the marginalization timestep, and change the anchor for any feature seen from it
  // NOTE: for now we have anchor the feature in the same camera as it is before
  // NOTE: this also does not change the representation of the feature at all right now
  double marg_timestep = state->margtimestep();
  for (auto &f : state->_features_SLAM) {
    // Skip any features that are in the global frame
    if (f.second->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D ||
        f.second->_feat_representation == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH)
      continue;
    // Else lets see if it is anchored in the clone that will be marginalized
    assert(marg_timestep <= f.second->_anchor_clone_timestamp);
    if (f.second->_anchor_clone_timestamp == marg_timestep) {
      perform_anchor_change(state, f.second, state->_timestamp, f.second->_anchor_cam_id);
    }
  }
}

void UpdaterSLAM::perform_anchor_change(std::shared_ptr<State> state, std::shared_ptr<Landmark> landmark, double new_anchor_timestamp,
                                        size_t new_cam_id) {

  // Assert that this is an anchored representation
  assert(LandmarkRepresentation::is_relative_representation(landmark->_feat_representation));
  assert(landmark->_anchor_cam_id != -1);

  // Create current feature representation
  UpdaterHelper::UpdaterHelperFeature old_feat;
  old_feat.featid = landmark->_featid;
  old_feat.feat_representation = landmark->_feat_representation;
  old_feat.anchor_cam_id = landmark->_anchor_cam_id;
  old_feat.anchor_clone_timestamp = landmark->_anchor_clone_timestamp;
  old_feat.p_FinA = landmark->get_xyz(false);
  old_feat.p_FinA_fej = landmark->get_xyz(true);

  // Get Jacobians of p_FinG wrt old representation
  Eigen::MatrixXd H_f_old;
  std::vector<Eigen::MatrixXd> H_x_old;
  std::vector<std::shared_ptr<Type>> x_order_old;
  UpdaterHelper::get_feature_jacobian_representation(state, old_feat, H_f_old, H_x_old, x_order_old);

  // Create future feature representation
  UpdaterHelper::UpdaterHelperFeature new_feat;
  new_feat.featid = landmark->_featid;
  new_feat.feat_representation = landmark->_feat_representation;
  new_feat.anchor_cam_id = new_cam_id;
  new_feat.anchor_clone_timestamp = new_anchor_timestamp;

  //==========================================================================
  //==========================================================================

  // OLD: anchor camera position and orientation
  Eigen::Matrix<double, 3, 3> R_GtoIOLD = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->Rot();
  Eigen::Matrix<double, 3, 3> R_GtoOLD = state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->Rot() * R_GtoIOLD;
  Eigen::Matrix<double, 3, 1> p_OLDinG = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->pos() -
                                         R_GtoOLD.transpose() * state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->pos();

  // NEW: anchor camera position and orientation
  Eigen::Matrix<double, 3, 3> R_GtoINEW = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->Rot();
  Eigen::Matrix<double, 3, 3> R_GtoNEW = state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->Rot() * R_GtoINEW;
  Eigen::Matrix<double, 3, 1> p_NEWinG = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->pos() -
                                         R_GtoNEW.transpose() * state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->pos();

  // Calculate transform between the old anchor and new one
  Eigen::Matrix<double, 3, 3> R_OLDtoNEW = R_GtoNEW * R_GtoOLD.transpose();
  Eigen::Matrix<double, 3, 1> p_OLDinNEW = R_GtoNEW * (p_OLDinG - p_NEWinG);
  new_feat.p_FinA = R_OLDtoNEW * landmark->get_xyz(false) + p_OLDinNEW;

  //==========================================================================
  //==========================================================================

  // OLD: anchor camera position and orientation
  Eigen::Matrix<double, 3, 3> R_GtoIOLD_fej = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->Rot_fej();
  Eigen::Matrix<double, 3, 3> R_GtoOLD_fej = state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->Rot() * R_GtoIOLD_fej;
  Eigen::Matrix<double, 3, 1> p_OLDinG_fej = state->_clones_IMU.at(old_feat.anchor_clone_timestamp)->pos_fej() -
                                             R_GtoOLD_fej.transpose() * state->_calib_IMUtoCAM.at(old_feat.anchor_cam_id)->pos();

  // NEW: anchor camera position and orientation
  Eigen::Matrix<double, 3, 3> R_GtoINEW_fej = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->Rot_fej();
  Eigen::Matrix<double, 3, 3> R_GtoNEW_fej = state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->Rot() * R_GtoINEW_fej;
  Eigen::Matrix<double, 3, 1> p_NEWinG_fej = state->_clones_IMU.at(new_feat.anchor_clone_timestamp)->pos_fej() -
                                             R_GtoNEW_fej.transpose() * state->_calib_IMUtoCAM.at(new_feat.anchor_cam_id)->pos();

  // Calculate transform between the old anchor and new one
  Eigen::Matrix<double, 3, 3> R_OLDtoNEW_fej = R_GtoNEW_fej * R_GtoOLD_fej.transpose();
  Eigen::Matrix<double, 3, 1> p_OLDinNEW_fej = R_GtoNEW_fej * (p_OLDinG_fej - p_NEWinG_fej);
  new_feat.p_FinA_fej = R_OLDtoNEW_fej * landmark->get_xyz(true) + p_OLDinNEW_fej;

  // Get Jacobians of p_FinG wrt new representation
  Eigen::MatrixXd H_f_new;
  std::vector<Eigen::MatrixXd> H_x_new;
  std::vector<std::shared_ptr<Type>> x_order_new;
  UpdaterHelper::get_feature_jacobian_representation(state, new_feat, H_f_new, H_x_new, x_order_new);

  //==========================================================================
  //==========================================================================

  // New phi order is just the landmark
  std::vector<std::shared_ptr<Type>> phi_order_NEW;
  phi_order_NEW.push_back(landmark);

  // Loop through all our orders and append them
  std::vector<std::shared_ptr<Type>> phi_order_OLD;
  int current_it = 0;
  std::map<std::shared_ptr<Type>, int> Phi_id_map;
  for (const auto &var : x_order_old) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  for (const auto &var : x_order_new) {
    if (Phi_id_map.find(var) == Phi_id_map.end()) {
      Phi_id_map.insert({var, current_it});
      phi_order_OLD.push_back(var);
      current_it += var->size();
    }
  }
  Phi_id_map.insert({landmark, current_it});
  phi_order_OLD.push_back(landmark);
  current_it += landmark->size();

  // Anchor change Jacobian
  int phisize = (new_feat.feat_representation != LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) ? 3 : 1;
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(phisize, current_it);
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(phisize, phisize);

  // Inverse of our new representation
  // pf_new_error = Hfnew^{-1}*(Hfold*pf_olderror+Hxold*x_olderror-Hxnew*x_newerror)
  Eigen::MatrixXd H_f_new_inv;
  if (phisize == 1) {
    H_f_new_inv = 1.0 / H_f_new.squaredNorm() * H_f_new.transpose();
  } else {
    H_f_new_inv = H_f_new.colPivHouseholderQr().solve(Eigen::Matrix<double, 3, 3>::Identity());
  }

  // Place Jacobians for old anchor
  for (size_t i = 0; i < H_x_old.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_old[i]), phisize, x_order_old[i]->size()).noalias() += H_f_new_inv * H_x_old[i];
  }

  // Place Jacobians for old feat
  Phi.block(0, Phi_id_map.at(landmark), phisize, phisize) = H_f_new_inv * H_f_old;

  // Place Jacobians for new anchor
  for (size_t i = 0; i < H_x_new.size(); i++) {
    Phi.block(0, Phi_id_map.at(x_order_new[i]), phisize, x_order_new[i]->size()).noalias() -= H_f_new_inv * H_x_new[i];
  }

  // Perform covariance propagation
  StateHelper::EKFPropagation(state, phi_order_NEW, phi_order_OLD, Phi, Q);

  // Set state from new feature
  landmark->_featid = new_feat.featid;
  landmark->_feat_representation = new_feat.feat_representation;
  landmark->_anchor_cam_id = new_feat.anchor_cam_id;
  landmark->_anchor_clone_timestamp = new_feat.anchor_clone_timestamp;
  landmark->set_from_xyz(new_feat.p_FinA, false);
  landmark->set_from_xyz(new_feat.p_FinA_fej, true);
  landmark->has_had_anchor_change = true;
}
