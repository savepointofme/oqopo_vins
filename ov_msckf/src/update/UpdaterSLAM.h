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

#ifndef OV_MSCKF_UPDATER_SLAM_H
#define OV_MSCKF_UPDATER_SLAM_H

#include <Eigen/Eigen>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"
#include "VisualObservabilityPolicy.h"
#include "VisualResidualDiag.h"
#include "state/StateHelper.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core
namespace ov_type {
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;

/**
 * @brief Will compute the system for our sparse SLAM features and update the filter.
 *
 * This class is responsible for performing delayed feature initialization, SLAM update, and
 * SLAM anchor change for anchored feature representations.
 */
class UpdaterSLAM {

public:
  /**
   * @brief Default constructor for our SLAM updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options_slam Updater options (include measurement noise value) for SLAM features
   * @param options_aruco Updater options (include measurement noise value) for ARUCO features
   * @param feat_init_options Feature initializer options
   */
  UpdaterSLAM(UpdaterOptions &options_slam, UpdaterOptions &options_aruco, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked SLAM features, this will try to use them to update the state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
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

  void set_visual_residual_diag(std::shared_ptr<VisualResidualDiag> diag) {
    visual_residual_diag_ = diag;
  }

  struct SlamInfoReductionConfig {
    bool enabled = false;
    double low_ratio = 0.2;
    double high_ratio = 0.5;
    double alpha_max = 4.0;
  };

  void configure_slam_info_reduction(bool enabled, double low_ratio,
                                     double high_ratio, double alpha_max);
  void set_slam_info_reduction_diag_window(double t0, double t1) {
    slam_info_diag_t0_ = t0;
    slam_info_diag_t1_ = t1;
  }
  void set_slam_info_reduction_diag_path(const std::string &path);

  void set_slam_ekf_leverage_diag_window(double t0, double t1) {
    slam_ekf_leverage_diag_t0_ = t0;
    slam_ekf_leverage_diag_t1_ = t1;
  }
  void set_slam_ekf_leverage_diag_path(const std::string &path);

  void set_slam_stacked_ekf_diag_window(double t0, double t1) {
    slam_stacked_ekf_diag_t0_ = t0;
    slam_stacked_ekf_diag_t1_ = t1;
  }
  void set_slam_stacked_ekf_diag_path(const std::string &path);

  void set_slam_landmark_metadata_diag_window(double t0, double t1) {
    slam_landmark_metadata_diag_t0_ = t0;
    slam_landmark_metadata_diag_t1_ = t1;
  }
  void set_slam_landmark_metadata_diag_path(const std::string &path);

  struct SlamGeometryLifecycleRefreshConfig {
    bool enabled = false;
    double min_depth_current = 400.0;
    double min_age_since_added = 3.0;
    double min_pose_landmark_cov_norm = 0.0;
    int min_regular_features_after_refresh = 0;
    bool require_anchor_change = false;
  };

  void configure_slam_geometry_lifecycle_refresh(bool enabled,
                                                 double min_depth_current,
                                                 double min_age_since_added,
                                                 double min_pose_landmark_cov_norm,
                                                 int min_regular_features_after_refresh,
                                                 bool require_anchor_change) {
    slam_geometry_refresh_cfg_.enabled = enabled;
    slam_geometry_refresh_cfg_.min_depth_current = min_depth_current;
    slam_geometry_refresh_cfg_.min_age_since_added = min_age_since_added;
    slam_geometry_refresh_cfg_.min_pose_landmark_cov_norm = min_pose_landmark_cov_norm;
    slam_geometry_refresh_cfg_.min_regular_features_after_refresh = min_regular_features_after_refresh;
    slam_geometry_refresh_cfg_.require_anchor_change = require_anchor_change;
  }
  void set_slam_geometry_lifecycle_refresh_diag_path(const std::string &path);

  void set_feature_yaw_contrib_diag_window(double t0, double t1) {
    feature_yaw_diag_t0_ = t0;
    feature_yaw_diag_t1_ = t1;
  }

  void set_feature_yaw_contrib_diag_path(const std::string &path);

  struct YawContribCapConfig {
    bool enabled = false;
    double t0 = -std::numeric_limits<double>::infinity();
    double t1 = std::numeric_limits<double>::infinity();
    double roll_deg = 10.0;
    double yawrate_degps = 3.0;
    int topk = 5;
    double ratio = 0.5;
    std::string mode = "soft_scale";
  };

  void set_yaw_contrib_cap_config(const YawContribCapConfig &cfg) { yaw_cap_cfg_ = cfg; }
  void set_yaw_contrib_cap_context(double yaw_error_deg, bool yaw_error_valid, double yaw_rate_degps) {
    yaw_cap_yaw_error_deg_ = yaw_error_deg;
    yaw_cap_yaw_error_valid_ = yaw_error_valid;
    yaw_cap_yaw_rate_degps_ = yaw_rate_degps;
  }
  void set_yaw_contrib_cap_diag_path(const std::string &path);

  /**
   * @brief Given max track features, this will try to use them to initialize them in the state.
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void delayed_init(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  /**
   * @brief Will change SLAM feature anchors if it will be marginalized
   *
   * Makes sure that if any clone is about to be marginalized, it changes anchor representation.
   * By default, this will shift the anchor into the newest IMU clone and keep the camera calibration anchor the same.
   *
   * @param state State of the filter
   */
  void change_anchors(std::shared_ptr<State> state);

protected:
  /**
   * @brief Shifts landmark anchor to new clone
   * @param state State of filter
   * @param landmark landmark whose anchor is being shifter
   * @param new_anchor_timestamp Clone timestamp we want to move to
   * @param new_cam_id Which camera frame we want to move to
   */
  void perform_anchor_change(std::shared_ptr<State> state, std::shared_ptr<ov_type::Landmark> landmark, double new_anchor_timestamp,
                             size_t new_cam_id);

  /// Options used during update for slam features
  UpdaterOptions _options_slam;

  /// Options used during update for aruco features
  UpdaterOptions _options_aruco;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  StateHelper::VisualYawUpdateMode visual_yaw_update_mode_ = StateHelper::VisualYawUpdateMode::ORIGINAL;
  double visual_yaw_update_scale_ = 1.0;
  double visual_global_yaw_oc_alpha_ = 0.0;
  double visual_bgz_update_scale_ = 1.0;

  std::shared_ptr<VisualObservabilityPolicy> vop_;
  std::shared_ptr<VisualResidualDiag> visual_residual_diag_;

  SlamInfoReductionConfig slam_info_cfg_;
  std::ofstream of_slam_info_reduction_diag_;
  double slam_info_diag_t0_ = -std::numeric_limits<double>::infinity();
  double slam_info_diag_t1_ = std::numeric_limits<double>::infinity();

  std::ofstream of_slam_ekf_leverage_diag_;
  double slam_ekf_leverage_diag_t0_ = -std::numeric_limits<double>::infinity();
  double slam_ekf_leverage_diag_t1_ = std::numeric_limits<double>::infinity();

  std::ofstream of_slam_stacked_ekf_diag_;
  double slam_stacked_ekf_diag_t0_ = -std::numeric_limits<double>::infinity();
  double slam_stacked_ekf_diag_t1_ = std::numeric_limits<double>::infinity();

  struct SlamLandmarkMetadata {
    double first_seen_time = std::numeric_limits<double>::quiet_NaN();
    double first_added_to_state_time = std::numeric_limits<double>::quiet_NaN();
    double first_slam_update_time = std::numeric_limits<double>::quiet_NaN();
    double last_slam_update_time = std::numeric_limits<double>::quiet_NaN();
    double first_anchor_clone_time = std::numeric_limits<double>::quiet_NaN();
    double last_anchor_clone_time = std::numeric_limits<double>::quiet_NaN();
    int anchor_cam_id = -1;
    int representation = -1;
    int slam_update_count = 0;
    int total_update_obs_count = 0;
    int anchor_change_count = 0;
    int crosses_1139 = 0;
    int crosses_1182 = 0;
  };
  std::unordered_map<size_t, SlamLandmarkMetadata> slam_landmark_metadata_;
  std::ofstream of_slam_landmark_metadata_diag_;
  double slam_landmark_metadata_diag_t0_ = -std::numeric_limits<double>::infinity();
  double slam_landmark_metadata_diag_t1_ = std::numeric_limits<double>::infinity();

  SlamGeometryLifecycleRefreshConfig slam_geometry_refresh_cfg_;
  std::ofstream of_slam_geometry_refresh_diag_;

  std::ofstream of_feature_yaw_contrib_diag_;
  double feature_yaw_diag_t0_ = -std::numeric_limits<double>::infinity();
  double feature_yaw_diag_t1_ = std::numeric_limits<double>::infinity();

  YawContribCapConfig yaw_cap_cfg_;
  double yaw_cap_yaw_error_deg_ = std::numeric_limits<double>::quiet_NaN();
  bool yaw_cap_yaw_error_valid_ = false;
  double yaw_cap_yaw_rate_degps_ = std::numeric_limits<double>::quiet_NaN();
  std::ofstream of_yaw_contrib_cap_diag_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_SLAM_H
