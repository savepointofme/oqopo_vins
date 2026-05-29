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

// =============================================================================
// [中文注释] UpdaterMSCKF.h
// -----------------------------------------------------------------------------
// MSCKF 更新器。负责把短轨特征 (不入状态) 转化为滤波观测方程,
// 通过**零空间投影** + 卡方检验 + QR 压缩 + EKF 更新完成。
//
// 单个特征的流程 (update 函数中):
//   Step 0  clean_old_measurements + 删除轨迹过短的特征
//   Step 1  从 state->_clones_IMU 构造每帧相机的 (R, p)
//   Step 2  FeatureInitializer 三角化特征, 并做非线性优化
//   Step 3  对每个特征构造一次词术 Jacobian:
//             H_x : 对状态 (包括相机标定/克隆)
//             H_f : 对特征 3D 位置
//           由于特征不在状态中, 用左零空间投影从 H_x、r 里消掉 H_f
//   Step 4  卡方检验判断更新有效性
//   Step 5  对所有通过的特征拼接大 H, 用 QR 分解再次压缩
//   Step 6  StateHelper::EKFUpdate 更新状态与协方差
//   Step 7  标记特征为 to_delete, 等后续清理
// =============================================================================

#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <memory>

#include "feat/FeatureInitializer.h"
#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"
#include "VisualObservabilityPolicy.h"
#include "state/StateHelper.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core

namespace ov_msckf {

class State;

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
 * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
 * After this we compress all the measurements to have an efficient update and update the state.
 *
 * [中文] MSCKF 更新器。优点是特征位置不入状态, 状态维度始终只与滑窗大小
 *        有关, 但仍能利用多帧观测约束位姿。如果同一个特征需要长期保留, 会由
 *        UpdaterSLAM::delayed_init 升格为 SLAM 特征并进入状态。
 */
class UpdaterMSCKF {

public:
  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   *
   * [中文] 主入口。输入 feature_vec 会被就地修改: 未用的特征保留在容器中,
   *        被用掉的特征会置 to_delete=true 等 VioManager 里统一清理。
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  void set_visual_yaw_update_control(StateHelper::VisualYawUpdateMode mode, double scale, double global_alpha) {
    visual_yaw_update_mode_ = mode;
    visual_yaw_update_scale_ = scale;
    visual_global_yaw_oc_alpha_ = global_alpha;
  }
  void set_visual_bgz_update_scale(double s) { visual_bgz_update_scale_ = s; }

  void set_visual_observability_policy(std::shared_ptr<VisualObservabilityPolicy> policy) {
    vop_ = policy;
  }

  // Aggregated OC diagnostics for the last update() call (over all features).
  struct OcBatchDiag {
    int n_features = 0;
    double sum_norm_HN_before = 0.0;
    double sum_norm_HN_after = 0.0;
    double sum_rel_HN_before = 0.0;
    double sum_rel_HN_after = 0.0;
    double sum_chi2_before = 0.0; // chi2 computed on raw H
    double sum_chi2_after = 0.0;  // chi2 computed on projected H
    bool projection_applied = false;
  };
  const OcBatchDiag &get_last_oc_diag() const { return last_oc_diag_; }

  struct LastStats {
    int n_features_in = 0;
    int n_tri_failed = 0;
    int n_chi2_rejected = 0;
    int n_accepted = 0;
    // Per-feature chi2 statistics (the chi2 statistic, not pixel residual)
    double chi2_sum_rej = 0.0;
    double chi2_max_rej = 0.0;
    double chi2_sum_acc = 0.0;
    double chi2_max_acc = 0.0;
    // Feature track length statistics
    int track_len_sum_acc = 0;
    int track_len_max_acc = 0;
    int track_len_sum_rej = 0;
    int track_len_max_rej = 0;
    // Triangulation rejection breakdown (from FeatureInitializer)
    ov_core::FeatureInitializer::TriBatchStats tri;
  };
  const LastStats &get_last_stats() const { return last_stats_; }

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  LastStats last_stats_;
  OcBatchDiag last_oc_diag_;

  StateHelper::VisualYawUpdateMode visual_yaw_update_mode_ = StateHelper::VisualYawUpdateMode::ORIGINAL;
  double visual_yaw_update_scale_ = 1.0;
  double visual_global_yaw_oc_alpha_ = 0.0;
  double visual_bgz_update_scale_ = 1.0;

  std::shared_ptr<VisualObservabilityPolicy> vop_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
