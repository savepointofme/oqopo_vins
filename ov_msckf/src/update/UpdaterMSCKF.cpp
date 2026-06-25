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

#include "UpdaterMSCKF.h"

#include "UpdaterHelper.h"
#include "VisualObservabilityPolicy.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <cmath>
#include <limits>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

double diag_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

double roll_deg_from_state(const std::shared_ptr<State> &state) {
  if (!state || !state->_imu)
    return diag_nan();
  Eigen::Matrix3d R_ItoG = state->_imu->Rot().transpose();
  const double roll = std::atan2(R_ItoG(2, 1), R_ItoG(2, 2));
  return roll * 180.0 / M_PI;
}

std::string image_side_from_u(double u) {
  if (!std::isfinite(u))
    return "unknown";
  return u < 320.0 ? "left" : "right";
}

std::string image_quadrant_from_uv(double u, double v) {
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

struct FeatureUvSummary {
  int num_measurements = 0;
  double u_mean = diag_nan();
  double v_mean = diag_nan();
};

FeatureUvSummary summarize_feature_uv(const std::shared_ptr<Feature> &feature) {
  FeatureUvSummary out;
  if (feature == nullptr)
    return out;
  double sum_u = 0.0;
  double sum_v = 0.0;
  int count = 0;
  for (const auto &pair : feature->timestamps) {
    const size_t cam_id = pair.first;
    const auto &times = pair.second;
    const auto it_uv = feature->uvs.find(cam_id);
    if (it_uv == feature->uvs.end())
      continue;
    const auto &uvs = it_uv->second;
    const size_t n = std::min(times.size(), uvs.size());
    for (size_t k = 0; k < n; k++) {
      sum_u += uvs[k](0);
      sum_v += uvs[k](1);
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

void accumulate_latest_uv(const std::shared_ptr<Feature> &feature, int &count, double &sum_u, double &sum_v) {
  if (feature == nullptr)
    return;
  bool found = false;
  double best_t = -1e100;
  Eigen::Vector2f best_uv = Eigen::Vector2f::Zero();
  for (const auto &pair : feature->timestamps) {
    const size_t cam_id = pair.first;
    const auto &times = pair.second;
    const auto it_uv = feature->uvs.find(cam_id);
    if (it_uv == feature->uvs.end())
      continue;
    const auto &uvs = it_uv->second;
    const size_t n = std::min(times.size(), uvs.size());
    for (size_t k = 0; k < n; k++) {
      if (times[k] > best_t) {
        best_t = times[k];
        best_uv = uvs[k];
        found = true;
      }
    }
  }
  if (!found)
    return;
  count++;
  sum_u += best_uv(0);
  sum_v += best_uv(1);
}

} // namespace

UpdaterMSCKF::UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {

  // Save our raw pixel noise squared
  _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

  // Save our feature initializer
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

// =============================================================================
// [中文] UpdaterMSCKF::update
//  按上面 docs-cn/updater.md 的 7 步流程逐步实现:
//    Step 0: 过滤无效特征
//    Step 1: 构造所有 clone 相机位姿 (R_GtoCi, p_CioinG)
//    Step 2: 三角化 + 高斯牛顿精化
//    Step 3: 对每个特征调用 UpdaterHelper::get_feature_jacobian_full,
//            然后 nullspace_project_inplace 消掉 H_f
//    Step 4: 卡方检验 (chi2) 剔除异常
//    Step 5: 对所有通过的特征拼大 H, 用 measurement_compress_inplace 再做 QR 压缩
//    Step 6: StateHelper::EKFUpdate
//    Step 7: 把 feature_vec 里用过的特征全部标 to_delete
// =============================================================================
void UpdaterMSCKF::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  last_stats_ = LastStats{};
  last_oc_diag_ = OcBatchDiag{};
  last_stats_.n_features_in = (int)feature_vec.size();
  if (feature_vec.empty())
    return;

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  // [中文] 滑窗里每个 clone 的时间戳, 用作特征观测的"合法时刻"候选
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
  // [中文] 每个相机都在滑窗里有 N 个克隆位姿 (R_GtoCi, p_CioinG), 传给三角化与 Jacobian
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
  // [中文] 依次三角化特征, 再可选地用高斯牛顿在多视角上精化 p_FinG
  initializer_feat->reset_tri_stats();
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
      last_stats_.n_tri_failed++;
      continue;
    }
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();
  last_stats_.tri = initializer_feat->get_tri_stats();

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  size_t max_hx_size = state->max_covariance_size();
  for (auto &landmark : state->_features_SLAM) {
    max_hx_size -= landmark.second->size();
  }

  // Large Jacobian and residual of *all* features for this update
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 4. Compute linear system for each feature, nullspace project, and reject
  // [中文] 单特征线性系统 + 左零空间投影 + 卡方检验
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
          const std::string &reject_reason, int track_len) {
        if (!visual_residual_diag_ || !visual_residual_diag_->enabled(diag_time))
          return;
        FeatureUvSummary uv = summarize_feature_uv(feature);
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
        row.update_type = "MSCKF";
        row.feature_id = feature ? feature->featid : 0;
        row.feature_kind = "msckf";
        row.track_age = track_len;
        row.num_measurements = uv.num_measurements;
        row.u_mean = uv.u_mean;
        row.v_mean = uv.v_mean;
        row.image_side = image_side_from_u(uv.u_mean);
        row.image_quadrant = image_quadrant_from_uv(uv.u_mean, uv.v_mean);
        row.residual_norm = residual_norm;
        row.whitened_residual_norm = whitened_norm;
        row.chi2 = chi2;
        row.chi2_threshold = chi2_threshold;
        row.accepted_by_chi2 = accepted;
        row.rejected_by_chi2 = !accepted;
        row.reject_reason = reject_reason;
        row.measurement_sigma_px = _options.sigma_pix;
        row.roll = diag_roll;
        row.bg_z = diag_bgz;
        visual_residual_diag_->log_feature(row);
      };
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
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

    // Nullspace project
    // [中文] MSCKF 核心: 找到 H_f 的左零空间 N, 对 H_x 和 r 左乘 N^T, 这样 H_f 消失
    //        后的线性系统只关系状态, 不用为特征状态开协方差列
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

    // VisualObservabilityPolicy: project H_x to remove unobservable directions
    // before chi-square gating (pre-chi2 modes only).
    // For the old GLOBAL_YAW_OC_PROJECTION mode the projection happens inside
    // EKFUpdate, so we pass the raw H_x here and to the accumulation below.
    Eigen::MatrixXd H_x_for_gate = H_x;
    if (vop_active) {
      std::vector<int> H_id = VisualObservabilityPolicy::build_H_id(Hx_order);
      VisualObservabilityPolicy::Diag oc_diag;
      H_x_for_gate = vop_->apply(H_x, Hx_order, H_id, state, &oc_diag);
      // Accumulate batch diagnostics
      last_oc_diag_.n_features++;
      last_oc_diag_.sum_norm_HN_before += oc_diag.norm_HN_before;
      last_oc_diag_.sum_norm_HN_after += oc_diag.norm_HN_after;
      last_oc_diag_.sum_rel_HN_before += oc_diag.rel_norm_HN_before;
      last_oc_diag_.sum_rel_HN_after += oc_diag.rel_norm_HN_after;
      last_oc_diag_.projection_applied = oc_diag.projection_applied;
    }

    /// Chi2 distance check (uses H_x_for_gate — OC-projected if vop active)
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    if (vop_active) {
      // Also track chi2 on raw H for before/after comparison
      Eigen::MatrixXd S_raw = H_x * P_marg * H_x.transpose();
      S_raw.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S_raw.rows());
      last_oc_diag_.sum_chi2_before += res.dot(S_raw.llt().solve(res));
    }
    Eigen::MatrixXd S = H_x_for_gate * P_marg * H_x_for_gate.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));
    if (vop_active)
      last_oc_diag_.sum_chi2_after += chi2;

    // Feature track length (total observations across all cameras)
    int track_len = 0;
    for (const auto &tp : feat.timestamps)
      track_len += (int)tp.second.size();

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
    if (chi2 > _options.chi2_multipler * chi2_check) {
      record_visual_residual(*it2, res, chi2, _options.chi2_multipler * chi2_check,
                             false, "chi2", track_len);
      (*it2)->to_delete = true;
      last_stats_.chi2_threshold_sum_rej += _options.chi2_multipler * chi2_check;
      last_stats_.chi2_threshold_max_rej =
          std::max(last_stats_.chi2_threshold_max_rej, _options.chi2_multipler * chi2_check);
      accumulate_latest_uv(*it2, last_stats_.uv_count_rej,
                           last_stats_.uv_sum_u_rej, last_stats_.uv_sum_v_rej);
      it2 = feature_vec.erase(it2);
      last_stats_.n_chi2_rejected++;
      last_stats_.chi2_sum_rej += chi2;
      last_stats_.chi2_max_rej = std::max(last_stats_.chi2_max_rej, chi2);
      last_stats_.track_len_sum_rej += track_len;
      last_stats_.track_len_max_rej = std::max(last_stats_.track_len_max_rej, track_len);
      continue;
    }

    // Accepted — accumulate stats
    record_visual_residual(*it2, res, chi2, _options.chi2_multipler * chi2_check,
                           true, "", track_len);
    last_stats_.chi2_sum_acc += chi2;
    last_stats_.chi2_max_acc = std::max(last_stats_.chi2_max_acc, chi2);
    last_stats_.chi2_threshold_sum_acc += _options.chi2_multipler * chi2_check;
    last_stats_.chi2_threshold_max_acc =
        std::max(last_stats_.chi2_threshold_max_acc, _options.chi2_multipler * chi2_check);
    last_stats_.track_len_sum_acc += track_len;
    last_stats_.track_len_max_acc = std::max(last_stats_.track_len_max_acc, track_len);
    accumulate_latest_uv(*it2, last_stats_.uv_count_acc,
                         last_stats_.uv_sum_u_acc, last_stats_.uv_sum_v_acc);

    // We are good!!! Append to our large H vector.
    // Use H_x_for_gate (OC-projected when vop active, else raw H_x) so that
    // the subsequent EKF update operates on the already-projected Jacobian.
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {

      // Ensure that this variable is in our Jacobian
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }

      // Append to our large Jacobian
      Hx_big.block(ct_meas, Hx_mapping[var], H_x_for_gate.rows(), var->size()) =
          H_x_for_gate.block(0, ct_hx, H_x_for_gate.rows(), var->size());
      ct_hx += var->size();
    }

    // Append our residual and move forward
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    it2++;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  if (visual_residual_diag_ && visual_residual_diag_->summary_enabled(diag_time)) {
    VisualResidualDiag::FrameSummary summary;
    summary.time = diag_time;
    summary.update_type = "MSCKF";
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

  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  last_stats_.n_accepted = (int)feature_vec.size();
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);

  // 5. Perform measurement compression
  // [中文] 对拼好的大 Hx_big 做 QR 分解, 把观测压缩到 ≤ 状态维度, 避免 EKF 中的 N^3 开销
  UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
  if (Hx_big.rows() < 1) {
    return;
  }
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

  // 6. With all good features update the state.
  // When VOP is active, H in Hx_big is already OC-projected; pass ORIGINAL
  // to EKFUpdate to avoid double-application.
  // When VOP is inactive, fall back to the legacy visual_yaw_update_mode_.
  if (vop_active) {
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big,
                           StateHelper::VisualYawUpdateMode::ORIGINAL, 1.0, 0.0,
                           visual_bgz_update_scale_);
  } else {
    StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big,
                           visual_yaw_update_mode_, visual_yaw_update_scale_,
                           visual_global_yaw_oc_alpha_, visual_bgz_update_scale_);
  }
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds create system (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds compress system\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds update state (%d size)\n", (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}
