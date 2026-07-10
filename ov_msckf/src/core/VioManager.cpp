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

#include "VioManager.h"
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterGroundPlaneRange.h"
#include "update/UpdaterGroundPlaneFeature.h"
#include "update/UpdaterGroundPlaneFeatureV1.h"
#include "update/UpdaterZeroVelocity.h"
#include "update/VisualObservabilityPolicy.h"
#include "update/VisualResidualDiag.h"

#include <algorithm>
#include <cctype>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

namespace {

double wrap_degrees(double angle_deg) {
  while (angle_deg > 180.0)
    angle_deg -= 360.0;
  while (angle_deg < -180.0)
    angle_deg += 360.0;
  return angle_deg;
}

Eigen::Vector3d rot_to_rpy(const Eigen::Matrix3d &R) {
  double pitch = std::asin(std::max(-1.0, std::min(1.0, -R(2, 0))));
  double roll = 0.0;
  double yaw = 0.0;
  if (std::fabs(std::cos(pitch)) > 1e-6) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
  return {roll, pitch, yaw};
}

double quiet_nan() {
  return std::numeric_limits<double>::quiet_NaN();
}

double mean_or_nan(const std::vector<double> &v) {
  if (v.empty())
    return quiet_nan();
  return std::accumulate(v.begin(), v.end(), 0.0) / (double)v.size();
}

double std_or_nan(const std::vector<double> &v, double mean) {
  if (v.empty() || !std::isfinite(mean))
    return quiet_nan();
  double acc = 0.0;
  for (double x : v)
    acc += (x - mean) * (x - mean);
  return std::sqrt(acc / (double)v.size());
}

double percentile_or_nan(std::vector<double> v, double q) {
  if (v.empty())
    return quiet_nan();
  std::sort(v.begin(), v.end());
  double idx = std::max(0.0, std::min(1.0, q)) * (double)(v.size() - 1);
  size_t lo = (size_t)std::floor(idx);
  size_t hi = std::min(v.size() - 1, lo + 1);
  double a = idx - (double)lo;
  return (1.0 - a) * v[lo] + a * v[hi];
}

double course_yaw_from_velocity_deg(const Eigen::Vector3d &v) {
  double speed_xy = std::hypot(v.x(), v.y());
  if (speed_xy < 1e-6)
    return quiet_nan();
  return std::atan2(v.y(), v.x()) * 180.0 / M_PI;
}

double wrap_radians(double angle_rad) {
  while (angle_rad > M_PI)
    angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI)
    angle_rad += 2.0 * M_PI;
  return angle_rad;
}

double yaw_from_Rwi_rad(const Eigen::Matrix3d &R_wi) {
  return std::atan2(R_wi(1, 0), R_wi(0, 0));
}

double yaw_from_jpl_q_GtoI_rad(const Eigen::Vector4d &q_GtoI) {
  return yaw_from_Rwi_rad(ov_core::quat_2_Rot(q_GtoI).transpose());
}

Eigen::RowVector3d numerical_yaw_jacobian_wrt_jpl_left_error(
    const Eigen::Vector4d &q_GtoI) {
  Eigen::RowVector3d H;
  const double eps = 1e-6;
  const double yaw0 = yaw_from_jpl_q_GtoI_rad(q_GtoI);
  for (int i = 0; i < 3; ++i) {
    Eigen::Vector4d dq = Eigen::Vector4d::Zero();
    dq(i) = 0.5 * eps;
    dq(3) = 1.0;
    dq = ov_core::quatnorm(dq);
    const Eigen::Vector4d q_plus = ov_core::quat_multiply(dq, q_GtoI);
    H(i) = wrap_radians(yaw_from_jpl_q_GtoI_rad(q_plus) - yaw0) / eps;
  }
  return H;
}

Eigen::Matrix3d rpy_to_rot(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd Rz(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
  return (Rz * Ry * Rx).toRotationMatrix();
}

StateHelper::VisualYawUpdateMode visual_yaw_mode_from_string(const std::string &mode) {
  if (mode == "none" || mode == "original")
    return StateHelper::VisualYawUpdateMode::ORIGINAL;
  if (mode == "per_block_scale")
    return StateHelper::VisualYawUpdateMode::PER_BLOCK_SCALE;
  if (mode == "global_yaw_oc_projection")
    return StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION;
  if (mode == "global_yaw_oc_fej_projection")
    return StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
  if (VisualObservabilityPolicy::is_prechi2_mode_string(mode))
    return StateHelper::VisualYawUpdateMode::ORIGINAL;
  PRINT_WARNING(YELLOW "[VIO-YAW] unknown vio_yaw_update_mode=%s, using original\n" RESET, mode.c_str());
  return StateHelper::VisualYawUpdateMode::ORIGINAL;
}

// Create a VisualObservabilityPolicy for pre-chi2 modes.
// Returns nullptr for modes that are handled inside EKFUpdate instead.
std::shared_ptr<VisualObservabilityPolicy> make_vop(const std::string &mode) {
  if (VisualObservabilityPolicy::is_prechi2_mode_string(mode)) {
    auto vop_mode = VisualObservabilityPolicy::mode_from_string(mode);
    return std::make_shared<VisualObservabilityPolicy>(vop_mode);
  }
  return nullptr;
}

// Apply the selected visual yaw policy to all visual updaters.
void apply_yaw_control_to_updaters(
    const std::string &mode, double scale, double alpha,
    std::shared_ptr<UpdaterMSCKF> &msckf,
    std::shared_ptr<UpdaterSLAM> &slam) {
  auto sh_mode = visual_yaw_mode_from_string(mode);
  auto vop = make_vop(mode);
  // For pre-chi2 VOP modes, EKFUpdate runs ORIGINAL (OC already applied pre-chi2).
  // For non-VOP modes, sh_mode carries the projection.
  auto ekf_mode = vop ? StateHelper::VisualYawUpdateMode::ORIGINAL : sh_mode;
  double ekf_scale = vop ? 1.0 : scale;
  double ekf_alpha = vop ? 0.0 : alpha;
  if (msckf) {
    msckf->set_visual_yaw_update_control(ekf_mode, ekf_scale, ekf_alpha);
    msckf->set_visual_observability_policy(vop);
  }
  if (slam) {
    slam->set_visual_yaw_update_control(ekf_mode, ekf_scale, ekf_alpha);
    slam->set_visual_observability_policy(vop);
  }
}

} // namespace

// =============================================================================
//    2. 创建 State 并将调用者传入的外参/内参/IMU内参写入
// =============================================================================
VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  imu_filter.configure(params.imu_filter);
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  if (!params.vio_yaw_update_diag_path.empty()) {
    set_vio_yaw_update_diag_path(params.vio_yaw_update_diag_path);
  }
  if (!params.visual_obs_diag_path.empty()) {
    set_visual_obs_diag_path(params.visual_obs_diag_path);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio,
        params.desc_max_match_px_dist, params.xfeat_model_path, params.sp_model_path));
  if (!params.xfeat_temporal_hint)
    std::dynamic_pointer_cast<TrackDescriptor>(trackFEATS)->xfeat_temporal_hint_ = false;
  }

  if (params.curl_correction_rate_degps != 0.0 && params.use_klt) {
    double rate_radps = params.curl_correction_rate_degps * M_PI / 180.0;
    trackFEATS->set_curl_correction_rate(rate_radps);
    PRINT_INFO(CYAN "[VioManager] curl_correction_rate=%.3f deg/s (%.6f rad/s) enabled\n" RESET,
               params.curl_correction_rate_degps, rate_radps);
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);
  apply_yaw_control_to_updaters(params.vio_yaw_update_mode, params.vio_yaw_update_scale,
                                params.vio_global_yaw_oc_alpha, updaterMSCKF, updaterSLAM);
  if (params.visual_bgz_update_scale < 1.0 - 1e-12) {
    updaterMSCKF->set_visual_bgz_update_scale(params.visual_bgz_update_scale);
    updaterSLAM->set_visual_bgz_update_scale(params.visual_bgz_update_scale);
  }

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity,
                                                        params.zupt_max_altitude);
  }

  // Ground-plane pseudo-rangefinder updater (created on demand via feed method)
  updaterGPlaneRange = nullptr;
}

VioManager::~VioManager() {
  if (params.imu_filter.enabled) {
    const auto &s = imu_filter.stats();
    PRINT_INFO(CYAN "[IMU-FILTER] samples=%llu resets=%llu coeff_updates=%llu disables=%llu measured_rate=%.3fHz "
                    "mean=%.3fus duplicate=%llu backward=%llu gaps=%llu\n" RESET,
               (unsigned long long)s.sample_count, (unsigned long long)s.reset_count,
               (unsigned long long)s.coefficient_update_count, (unsigned long long)s.disable_count, s.measured_sample_rate_hz,
               s.mean_processing_time_us(), (unsigned long long)s.timestamp_duplicate_count,
               (unsigned long long)s.timestamp_backward_count, (unsigned long long)s.timestamp_gap_count);
  }
}

// =============================================================================
// [中文] IMU 消息入口
//  对下游计算的影响:
//    - initializer / ZUPT 在各自的窗口外丢弃旧测量
// =============================================================================
void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // This is the only filter invocation. Every downstream IMU consumer receives
  // this same sample, and the disabled path returns a value-identical copy.
  const ov_core::ImuData processed_message = imu_filter.process(message);

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    oldest_time = processed_message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  propagator->feed_imu(processed_message, oldest_time);

  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(processed_message, oldest_time);
  }

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(processed_message, oldest_time);
  }
}

bool VioManager::configure_gps_alt_coupled_update(
    const std::string &mode, double residual_soft_limit,
    double max_delta_xy, double max_delta_z, double max_delta_velocity,
    double max_delta_attitude, double max_delta_accel_bias,
    double max_delta_gyro_bias, double covariance_psd_check_interval) {
  std::string normalized = mode;
  std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                 [](unsigned char c) { return (char)std::tolower(c); });
  std::replace(normalized.begin(), normalized.end(), '-', '_');
  if (normalized == "guarded") {
    gps_alt_coupled_mode_ = GpsAltCoupledMode::GUARDED;
  } else if (normalized == "full" || normalized == "standard") {
    // "standard": full standard Kalman gain + coupled Joseph, no underweighting.
    gps_alt_coupled_mode_ = GpsAltCoupledMode::FULL;
  } else if (normalized == "bounded" || normalized == "scaled_joseph") {
    // "scaled_joseph": Codex trust-region uniform-gain scaling (frozen Test 2).
    gps_alt_coupled_mode_ = GpsAltCoupledMode::BOUNDED;
  } else if (normalized == "nasa_lean" || normalized == "nasa_underweight") {
    // NASA measurement underweighting (independent third mode).
    gps_alt_coupled_mode_ = GpsAltCoupledMode::NASA_LEAN;
  } else {
    PRINT_ERROR(RED "[GPS-ALT] unknown coupled mode '%s' "
                    "(expected guarded|full(standard)|bounded(scaled_joseph)|nasa_lean)\n" RESET,
                mode.c_str());
    return false;
  }

  gps_alt_residual_soft_limit_ = std::max(0.0, residual_soft_limit);
  gps_alt_max_delta_xy_ = std::max(0.0, max_delta_xy);
  gps_alt_max_delta_z_ = std::max(0.0, max_delta_z);
  gps_alt_max_delta_velocity_ = std::max(0.0, max_delta_velocity);
  gps_alt_max_delta_attitude_ = std::max(0.0, max_delta_attitude);
  gps_alt_max_delta_accel_bias_ = std::max(0.0, max_delta_accel_bias);
  gps_alt_max_delta_gyro_bias_ = std::max(0.0, max_delta_gyro_bias);
  gps_alt_covariance_psd_check_interval_ =
      std::max(0.0, covariance_psd_check_interval);
  gps_alt_last_covariance_psd_check_time_ = -1.0;

  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::BOUNDED &&
      gps_alt_residual_soft_limit_ <= 0.0 && gps_alt_max_delta_xy_ <= 0.0 &&
      gps_alt_max_delta_z_ <= 0.0 && gps_alt_max_delta_velocity_ <= 0.0 &&
      gps_alt_max_delta_attitude_ <= 0.0 &&
      gps_alt_max_delta_accel_bias_ <= 0.0 &&
      gps_alt_max_delta_gyro_bias_ <= 0.0) {
    PRINT_ERROR(RED "[GPS-ALT] bounded mode requires at least one positive "
                    "residual/state-increment limit\n" RESET);
    return false;
  }
  return true;
}

void VioManager::set_gps_alt_coupled_diag_path(const std::string &path) {
  if (of_gps_alt_coupled_diag.is_open()) of_gps_alt_coupled_diag.close();
  if (path.empty()) return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_gps_alt_coupled_diag.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_gps_alt_coupled_diag.is_open()) {
    PRINT_WARNING(YELLOW "[GPS-ALT-COUPLED] failed to open %s\n" RESET,
                  path.c_str());
    return;
  }
  of_gps_alt_coupled_diag
      << "time,gps_z,gps_x_ref,gps_y_ref,vio_x_before,vio_y_before,vio_z_before,"
      << "residual_raw,residual_used,Pzz_before,S,NIS,Pxz_before,Pyz_before,"
      << "K_px,K_py,K_pz,K_vx,K_vy,K_vz,K_roll,K_pitch,K_yaw,"
      << "K_bax,K_bay,K_baz,K_bgx,K_bgy,K_bgz,"
      << "pred_dx,pred_dy,pred_dz,pred_dvx,pred_dvy,pred_dvz,"
      << "pred_droll,pred_dpitch,pred_dyaw,pred_dbax,pred_dbay,pred_dbaz,"
      << "pred_dbgx,pred_dbgy,pred_dbgz,gain_scale,nasa_beta,r_effective,"
      << "used_dx,used_dy,used_dz,used_dvx,used_dvy,used_dvz,"
      << "used_droll,used_dpitch,used_dyaw,used_dbax,used_dbay,used_dbaz,"
      << "used_dbgx,used_dbgy,used_dbgz,true_xy_error_x,true_xy_error_y,"
      << "direction_cos,expected_xy_improvement,large_coupled,decision,"
      << "Pzz_after,Pxz_after,Pyz_after,cov_finite,cov_symmetry_error,"
      << "cov_min_diagonal,cov_psd_checked,cov_min_ldlt_diagonal,cov_psd\n";
  of_gps_alt_coupled_diag.flush();
}

void VioManager::set_gps_alt_xy_diagnostic_reference(
    double timestamp, const Eigen::Vector2d &xy, bool valid) {
  gps_alt_diag_xy_time_ = timestamp;
  gps_alt_diag_xy_ = xy;
  gps_alt_diag_xy_valid_ = valid && xy.allFinite();
}

bool VioManager::feed_measurement_pose_anchor(
    double timestamp, const Eigen::Vector3d &p_IinG_meas,
    double yaw_meas_rad, bool yaw_valid, double pos_sigma,
    double yaw_sigma_rad, double gate_sigma,
    double max_pos_correction, double max_yaw_correction_rad) {

  pose_anchor_last_ = PoseAnchorLastUpdate();
  pose_anchor_last_.t = state ? state->_timestamp : timestamp;
  pose_anchor_last_.yaw_used = false;

  if (!is_initialized_vio || state == nullptr || state->_imu == nullptr) {
    pose_anchor_last_.decision = "SKIP_NOT_INITIALIZED";
    return false;
  }
  if (!p_IinG_meas.allFinite() || pos_sigma <= 0.0 || gate_sigma <= 0.0) {
    pose_anchor_last_.decision = "REJECT_BAD_INPUT";
    return false;
  }
  if (std::fabs(state->_timestamp - timestamp) > 0.50) {
    pose_anchor_last_.decision = "SKIP_TIME_MISALIGN";
    return false;
  }

  const Eigen::Vector3d p_pred = state->_imu->pos();
  const double yaw_pred = current_imu_yaw_deg() * M_PI / 180.0;
  const bool use_yaw = yaw_valid && std::isfinite(yaw_meas_rad) &&
                       yaw_sigma_rad > 0.0;
  const int rows = use_yaw ? 4 : 3;
  pose_anchor_last_.yaw_used = use_yaw;

  std::vector<std::shared_ptr<ov_type::Type>> H_order;
  Eigen::MatrixXd H;
  Eigen::VectorXd res = Eigen::VectorXd::Zero(rows);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(rows, rows);

  if (use_yaw) {
    H_order = {state->_imu->q(), state->_imu->p()};
    H = Eigen::MatrixXd::Zero(rows, 6);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    H.block<1, 3>(3, 0) =
        numerical_yaw_jacobian_wrt_jpl_left_error(state->_imu->quat());
    res.segment<3>(0) = p_IinG_meas - p_pred;
    res(3) = wrap_radians(yaw_meas_rad - yaw_pred);
    R.block<3, 3>(0, 0) =
        std::pow(pos_sigma, 2) * Eigen::Matrix3d::Identity();
    R(3, 3) = std::pow(yaw_sigma_rad, 2);
  } else {
    H_order = {state->_imu->p()};
    H = Eigen::MatrixXd::Zero(rows, 3);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    res = p_IinG_meas - p_pred;
    R = std::pow(pos_sigma, 2) * Eigen::Matrix3d::Identity();
  }

  pose_anchor_last_.pos_residual_norm = res.segment<3>(0).norm();
  pose_anchor_last_.yaw_residual_deg =
      use_yaw ? res(3) * 180.0 / M_PI : quiet_nan();

  Eigen::MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);
  Eigen::MatrixXd S = H * P_small * H.transpose() + R;
  if (!S.allFinite()) {
    pose_anchor_last_.decision = "REJECT_NONFINITE_S";
    return false;
  }
  Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
  if (ldlt.info() != Eigen::Success) {
    pose_anchor_last_.decision = "REJECT_S_DECOMPOSITION";
    return false;
  }
  const Eigen::VectorXd Sinv_res = ldlt.solve(res);
  if (!Sinv_res.allFinite()) {
    pose_anchor_last_.decision = "REJECT_NONFINITE_NIS";
    return false;
  }
  const double nis = res.dot(Sinv_res);
  const double ratio = nis / (gate_sigma * gate_sigma * (double)rows);
  pose_anchor_last_.nis = nis;
  pose_anchor_last_.gate_ratio = ratio;
  if (!std::isfinite(ratio) || ratio > 1.0) {
    pose_anchor_last_.decision = "REJECT_INNOVATION";
    return false;
  }

  Eigen::VectorXd dx = StateHelper::compute_update_dx(state, H_order, H, res, R);
  if (!dx.allFinite()) {
    pose_anchor_last_.decision = "REJECT_NONFINITE_DX";
    return false;
  }
  const int p_start = state->_imu->p()->id();
  const int q_start = state->_imu->q()->id();
  Eigen::Vector3d dp = Eigen::Vector3d::Zero();
  Eigen::Vector3d dq = Eigen::Vector3d::Zero();
  if (p_start >= 0 && p_start + 2 < dx.rows())
    dp = dx.segment<3>(p_start);
  if (q_start >= 0 && q_start + 2 < dx.rows())
    dq = dx.segment<3>(q_start);
  pose_anchor_last_.predicted_pos_correction_norm = dp.norm();
  pose_anchor_last_.predicted_yaw_correction_deg = dq(2) * 180.0 / M_PI;
  if (max_pos_correction > 0.0 && dp.norm() > max_pos_correction) {
    pose_anchor_last_.decision = "REJECT_TRUST_REGION_POS";
    return false;
  }
  if (use_yaw && max_yaw_correction_rad > 0.0 &&
      std::fabs(dq(2)) > max_yaw_correction_rad) {
    pose_anchor_last_.decision = "REJECT_TRUST_REGION_YAW";
    return false;
  }

  const double yaw_before = current_imu_yaw_deg();
  StateHelper::reset_last_yaw_dx_projection_diag();
  StateHelper::EKFUpdate(state, H_order, H, res, R);
  const double yaw_after = current_imu_yaw_deg();
  pose_anchor_last_.accepted = true;
  pose_anchor_last_.decision = use_yaw ? "ACCEPT_POS_YAW" : "ACCEPT_POS";
  log_vio_yaw_update(state->_timestamp, "POSE_ANCHOR",
                     yaw_before, yaw_after,
                     wrap_degrees(yaw_after - yaw_before),
                     state->_imu->bias_g()(2), 1, nis, 1, 0,
                     trackFEATS ? get_feature_database_size() : -1);
  log_yaw_update_mechanism(state->_timestamp, "POSE_ANCHOR",
                           yaw_before, yaw_after,
                           course_yaw_from_velocity_deg(state->_imu->vel()),
                           course_yaw_from_velocity_deg(state->_imu->vel()),
                           wrap_degrees(yaw_after - yaw_before),
                           1, nis, 1, 0,
                           trackFEATS ? get_feature_database_size() : -1);
  PRINT_INFO(CYAN "[POSE-ANCHOR] %s t=%.3f |pos_res|=%.2f yaw_res=%.2fdeg "
                  "nis=%.2f ratio=%.3f |dp|=%.2f dyaw_pred=%.2fdeg\n" RESET,
             pose_anchor_last_.decision.c_str(), state->_timestamp,
             pose_anchor_last_.pos_residual_norm,
             pose_anchor_last_.yaw_residual_deg,
             pose_anchor_last_.nis, pose_anchor_last_.gate_ratio,
             pose_anchor_last_.predicted_pos_correction_norm,
             pose_anchor_last_.predicted_yaw_correction_deg);
  return true;
}

bool VioManager::apply_trusted_pose_anchor_reset(const FCInitState &fc,
                                                  double camera_timestamp,
                                                  bool yaw_valid,
                                                  bool velocity_valid,
                                                  bool reset_biases) {
  pose_anchor_last_ = PoseAnchorLastUpdate();
  pose_anchor_last_.t = state ? state->_timestamp : camera_timestamp;
  pose_anchor_last_.yaw_used = yaw_valid;

  if (!is_initialized_vio || state == nullptr || state->_imu == nullptr) {
    pose_anchor_last_.decision = "TRUSTED_RESET_SKIP_NOT_INITIALIZED";
    return false;
  }
  if (!fc.p_IinG.allFinite()) {
    pose_anchor_last_.decision = "TRUSTED_RESET_REJECT_BAD_POSITION";
    return false;
  }
  if (std::fabs(state->_timestamp - camera_timestamp) > 0.50) {
    pose_anchor_last_.decision = "TRUSTED_RESET_SKIP_TIME_MISALIGN";
    return false;
  }

  const Eigen::Vector3d p_before = state->_imu->pos();
  const double yaw_before_deg = current_imu_yaw_deg();
  double yaw_delta_rad = 0.0;
  if (yaw_valid && fc.q_GtoI.allFinite()) {
    const Eigen::Matrix3d R_ItoG_meas = ov_core::quat_2_Rot(fc.q_GtoI).transpose();
    const double yaw_meas_rad = rot_to_rpy(R_ItoG_meas)(2);
    yaw_delta_rad = wrap_radians(yaw_meas_rad - yaw_before_deg * M_PI / 180.0);
  }
  const Eigen::Matrix3d R_delta =
      Eigen::AngleAxisd(yaw_delta_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d t_delta = fc.p_IinG - R_delta * p_before;

  auto transform_pose_value = [&](const Eigen::MatrixXd &value) {
    Eigen::Matrix<double, 7, 1> out = value;
    const Eigen::Matrix3d R_ItoG = ov_core::quat_2_Rot(out.block<4, 1>(0, 0)).transpose();
    const Eigen::Matrix3d R_ItoG_new = R_delta * R_ItoG;
    out.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_ItoG_new.transpose());
    out.block<3, 1>(4, 0) = R_delta * out.block<3, 1>(4, 0) + t_delta;
    return out;
  };

  {
    Eigen::Matrix<double, 16, 1> imu = state->_imu->value();
    const Eigen::Matrix3d R_ItoG = ov_core::quat_2_Rot(imu.block<4, 1>(0, 0)).transpose();
    const Eigen::Matrix3d R_ItoG_new = R_delta * R_ItoG;
    imu.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_ItoG_new.transpose());
    imu.block<3, 1>(4, 0) = fc.p_IinG;
    imu.block<3, 1>(7, 0) =
        (velocity_valid && fc.v_IinG.allFinite()) ? fc.v_IinG : R_delta * imu.block<3, 1>(7, 0);
    if (reset_biases && fc.bg.allFinite())
      imu.block<3, 1>(10, 0) = fc.bg;
    if (reset_biases && fc.ba.allFinite())
      imu.block<3, 1>(13, 0) = fc.ba;
    state->_imu->set_value(imu);

    Eigen::Matrix<double, 16, 1> imu_fej = state->_imu->fej();
    const Eigen::Matrix3d R_ItoG_fej = ov_core::quat_2_Rot(imu_fej.block<4, 1>(0, 0)).transpose();
    const Eigen::Matrix3d R_ItoG_fej_new = R_delta * R_ItoG_fej;
    imu_fej.block<4, 1>(0, 0) = ov_core::rot_2_quat(R_ItoG_fej_new.transpose());
    imu_fej.block<3, 1>(4, 0) = R_delta * imu_fej.block<3, 1>(4, 0) + t_delta;
    imu_fej.block<3, 1>(7, 0) =
        (velocity_valid && fc.v_IinG.allFinite()) ? fc.v_IinG : R_delta * imu_fej.block<3, 1>(7, 0);
    if (reset_biases && fc.bg.allFinite())
      imu_fej.block<3, 1>(10, 0) = fc.bg;
    if (reset_biases && fc.ba.allFinite())
      imu_fej.block<3, 1>(13, 0) = fc.ba;
    state->_imu->set_fej(imu_fej);
  }

  for (auto &clone_pair : state->_clones_IMU) {
    clone_pair.second->set_value(transform_pose_value(clone_pair.second->value()));
    clone_pair.second->set_fej(transform_pose_value(clone_pair.second->fej()));
  }

  for (auto &feat_pair : state->_features_SLAM) {
    auto &lm = feat_pair.second;
    if (!lm)
      continue;
    const auto rep = lm->_feat_representation;
    const bool global_rep =
        rep == LandmarkRepresentation::Representation::GLOBAL_3D ||
        rep == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH;
    if (!global_rep)
      continue;
    const Eigen::Vector3d xyz = lm->get_xyz(false);
    if (xyz.allFinite())
      lm->set_from_xyz(R_delta * xyz + t_delta, false);
    const Eigen::Vector3d xyz_fej = lm->get_xyz(true);
    if (xyz_fej.allFinite())
      lm->set_from_xyz(R_delta * xyz_fej + t_delta, true);
  }

  reset_gps_altitude_bootstrap();
  pose_anchor_last_.pos_residual_norm = (fc.p_IinG - p_before).norm();
  pose_anchor_last_.yaw_residual_deg = yaw_delta_rad * 180.0 / M_PI;
  pose_anchor_last_.accepted = true;
  pose_anchor_last_.decision = yaw_valid ? "TRUSTED_RESET_POS_YAW" : "TRUSTED_RESET_POS";

  const double yaw_after_deg = current_imu_yaw_deg();
  log_vio_yaw_update(state->_timestamp, "POSE_ANCHOR_TRUSTED_RESET",
                     yaw_before_deg, yaw_after_deg,
                     wrap_degrees(yaw_after_deg - yaw_before_deg),
                     state->_imu->bias_g()(2), 1, 0.0, 1, 0,
                     trackFEATS ? get_feature_database_size() : -1);
  PRINT_WARNING(YELLOW "[POSE-ANCHOR] %s t=%.3f |pos_res|=%.2f yaw_reset=%.2fdeg "
                       "vel_reset=%d bias_reset=%d\n" RESET,
                pose_anchor_last_.decision.c_str(), state->_timestamp,
                pose_anchor_last_.pos_residual_norm,
                pose_anchor_last_.yaw_residual_deg,
                velocity_valid ? 1 : 0, reset_biases ? 1 : 0);
  return true;
}

void VioManager::feed_measurement_gps_altitude(double timestamp, double altitude_z, double sigma,
                                                double chi2_gate, bool also_update_vz) {

  if (!is_initialized_vio) {
    return;
  }
  if (!std::isfinite(timestamp) || !std::isfinite(altitude_z) ||
      !std::isfinite(sigma) || sigma <= 0.0) {
    gps_alt_stats_.n_rejected++;
    gps_alt_last_.decision = "REJECT_UNHEALTHY_MEASUREMENT";
    PRINT_WARNING(YELLOW "[GPS-ALT] reject non-finite/invalid measurement "
                    "t=%.6f z=%.6f sigma=%.6f\n" RESET,
                  timestamp, altitude_z, sigma);
    return;
  }
  // Optional bootstrap-delay gate (Stage A ablation): drop GPS altitude
  // calls until enough time has passed since VIO init.  This lets
  // monocular scale settle before z_ground / refs are locked.
  if (gps_alt_min_t_after_init_ > 0.0 && startup_time > 0.0 &&
      timestamp - startup_time < gps_alt_min_t_after_init_) {
    return;
  }

  double t_state = state->_timestamp;
  double dt_gps = t_state - timestamp;
  gps_alt_stats_.n_called++;
  gps_alt_stats_.last_eval_time = t_state;
  if (gps_alt_stats_.first_eval_time < 0) gps_alt_stats_.first_eval_time = t_state;
  // Watchdog bookkeeping (interface for a future forced-aiding / recovery stage;
  // this round only records). Every reached call is a pending failure until the
  // accepted path below resets the counter.
  gps_alt_stats_.last_measurement_time = t_state;
  gps_alt_stats_.consecutive_failures++;

  // --- Periodic STAT summary (fires for ALL calls: accept/reject/skip) ---
  if (t_state - gps_alt_stats_.last_summary_time > 30.0) {
    gps_alt_stats_.last_summary_time = t_state;
    size_t n_evals = gps_alt_stats_.n_accepted + gps_alt_stats_.n_rejected;
    double rate = n_evals > 0 ? 100.0 * gps_alt_stats_.n_accepted / n_evals : 0;
    PRINT_INFO(CYAN
               "[GPS-ALT-STAT]\n"
               "+----------+--------+--------+--------+--------+--------+\n"
               "| t_cam    | calls  | acc    | rej    | skip   | rate   |\n"
               "| %8.1f | %6zu | %6zu | %6zu | %6zu | %5.1f%% |\n"
               "+----------+--------+--------+--------+--------+--------+\n"
               "| reject dxy | reject kxy | reject bias | large coupled |\n"
               "| %10zu | %10zu | %11zu | %13zu |\n"
               "+------------+------------+-------------+---------------+\n"
               "| K_pz_mu | K_xy_mu | dxy_mu | dtheta_mu | dba_mu  | dbg_mu  |\n"
               "| %7.5f | %7.5f | %6.4f | %9.4f | %7.5f | %7.5f |\n"
               "+---------+---------+--------+-----------+---------+---------+\n"
               "| P_zz_mu | abs_res_mu | abs_dpz_mu |\n"
               "| %7.4f | %10.2f | %10.3f |\n"
               "+---------+------------+------------+\n" RESET,
               t_state, gps_alt_stats_.n_called, gps_alt_stats_.n_accepted,
               gps_alt_stats_.n_rejected, gps_alt_stats_.n_skipped, rate,
               gps_alt_stats_.n_rejected_dxy, gps_alt_stats_.n_rejected_kxy,
               gps_alt_stats_.n_rejected_bias, gps_alt_stats_.n_large_coupled,
               n_evals > 0 ? gps_alt_stats_.sum_K_pz / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_K_xy_norm / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_dxy_norm / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_dtheta_norm / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_dba_norm / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_dbg_norm / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_P_zz / n_evals : 0.0,
               n_evals > 0 ? gps_alt_stats_.sum_abs_res / n_evals : 0.0,
               gps_alt_stats_.n_accepted > 0 ? gps_alt_stats_.sum_abs_dpz / gps_alt_stats_.n_accepted : 0.0);
  }

  // [中文] 时间差容忍放宽到 200ms. GPS 5Hz 间隔 200ms, cam 20Hz 间隔 50ms.
  if (t_state < timestamp - 0.2 || t_state > timestamp + 0.2) {
    gps_alt_stats_.n_skipped++;
    PRINT_DEBUG(YELLOW "[GPS-ALT-EVAL] status=SKIP t_cam=%.3f t_gps=%.3f dt=%+.3fs (state=%.3f meas=%.3f)\n" RESET,
                t_state, timestamp, dt_gps, t_state, timestamp);
    return;
  }

  Eigen::Vector3d p_IinG = state->_imu->pos();

  // H_order: active state variables included in the Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Eigen::MatrixXd H;
  double z_pred;

  {
    // GPS altitude is already a world-frame height measurement: h(x) = p_z.
    z_pred = p_IinG(2);
    Hx_order.push_back(state->_imu->p());
    if (also_update_vz) {
      Hx_order.push_back(state->_imu->v());
      H = Eigen::MatrixXd::Zero(1, 6);
      H(0, 2) = 1.0;
    } else {
      H = Eigen::MatrixXd::Zero(1, 3);
      H(0, 2) = 1.0;
    }
  }

  double res_scalar = altitude_z - z_pred;

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1);
  R(0, 0) = std::pow(sigma, 2);

  int pz_idx = 2;

  // --- P_zz floor BEFORE update: prevent K_pz from collapsing to ~0 ---
  // Injecting noise before computing S/K keeps the Kalman gain alive.
  if (gps_alt_min_pzz_ > 0) {
    Eigen::MatrixXd P_pre_check = StateHelper::get_marginal_covariance(state, Hx_order);
    double P_pz_pre = P_pre_check(pz_idx, pz_idx);
    if (P_pz_pre < gps_alt_min_pzz_) {
      StateHelper::inject_pz_noise(state, gps_alt_min_pzz_ - P_pz_pre);
    }
  }

  Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, Hx_order);
  double S = (H * P * H.transpose())(0, 0) + R(0, 0);
  double chi2 = res_scalar * res_scalar / S;
  double P_pz = P(pz_idx, pz_idx);

  // Expand the scalar measurement Jacobian into the complete error-state
  // ordering. GPS-Z coupled Test 1/2 uses this unprojected gain for every state,
  // including states absent from H_order but correlated through P.
  const Eigen::MatrixXd P_full_before = StateHelper::get_full_covariance(state);
  const int state_dim = (int)P_full_before.rows();
  Eigen::VectorXd H_full = Eigen::VectorXd::Zero(state_dim);
  int h_col = 0;
  for (const auto &var : Hx_order) {
    if (var->id() >= 0 && var->id() + var->size() <= state_dim)
      H_full.segment(var->id(), var->size()) = H.block(0, h_col, 1, var->size()).transpose();
    h_col += var->size();
  }
  const Eigen::VectorXd gain_numerator_full = P_full_before * H_full;
  S = H_full.dot(gain_numerator_full) + R(0, 0);
  if (!std::isfinite(S) || S <= 1e-18) {
    gps_alt_stats_.n_rejected++;
    gps_alt_stats_.n_rejected_numerical++;
    gps_alt_last_.decision = "REJECT_INVALID_INNOVATION_VARIANCE";
    PRINT_WARNING(YELLOW "[GPS-ALT] invalid innovation variance S=%.9g at t=%.3f\n" RESET,
                  S, t_state);
    return;
  }
  chi2 = res_scalar * res_scalar / S;
  const Eigen::VectorXd K_full = gain_numerator_full / S;
  const Eigen::VectorXd dx_full = K_full * res_scalar;

  const int q_start = state->_imu->q()->id();
  const int p_start = state->_imu->p()->id();
  const int v_start = state->_imu->v()->id();
  const int bg_start = state->_imu->bg()->id();
  const int ba_start = state->_imu->ba()->id();
  const Eigen::Vector3d K_q = K_full.segment(q_start, 3);
  const Eigen::Vector3d K_p = K_full.segment(p_start, 3);
  const Eigen::Vector3d K_v = K_full.segment(v_start, 3);
  const Eigen::Vector3d K_bg = K_full.segment(bg_start, 3);
  const Eigen::Vector3d K_ba = K_full.segment(ba_start, 3);
  const Eigen::Vector3d pred_dq = dx_full.segment(q_start, 3);
  const Eigen::Vector3d pred_dp = dx_full.segment(p_start, 3);
  const Eigen::Vector3d pred_dv = dx_full.segment(v_start, 3);
  const Eigen::Vector3d pred_dbg = dx_full.segment(bg_start, 3);
  const Eigen::Vector3d pred_dba = dx_full.segment(ba_start, 3);
  P_pz = P_full_before(p_start + 2, p_start + 2);

  // --- Innovation gate: reject updates where |res| exceeds threshold ---
  // Large residuals indicate a systematic GPS offset that would pull the state
  // incorrectly.  Skipping the update entirely is safer than clamping.
  const double raw_res = res_scalar;
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED &&
      gps_alt_max_res_gate_ < 1e8 && std::fabs(res_scalar) > gps_alt_max_res_gate_) {
    gps_alt_stats_.n_rejected++;
    PRINT_INFO(YELLOW "[GPS-ALT] REJECT t=%.3f |res|=%.1f > %.1f m - skipping update\n" RESET,
               t_state, std::fabs(raw_res), gps_alt_max_res_gate_);
    return;
  }

  Eigen::VectorXd res = Eigen::VectorXd::Zero(1);
  res(0) = res_scalar;

  // Log large residuals for diagnostics (no rejection)
  if (chi2 > chi2_gate) {
    PRINT_INFO(YELLOW "[GPS-ALT-DIAG] large-res t=%.3f res=%.2f chi2=%.1f P_zz=%.4f - accepting anyway\n" RESET,
               t_state, raw_res, chi2, P_pz);
  }

  // --- Marginal Kalman gain for position block ---
  // K = P * H^T / S.  H has a 1 at p_z (col 2) and 0 elsewhere,
  // so K_p = P[:, 2] / S = [P_xz, P_yz, P_zz]^T / S.
  Eigen::VectorXd K_pz_vec = (P * H.transpose()) / S;  // n×1
  double K_px = K_p(0);
  double K_py = K_p(1);
  double K_pz_gain = K_p(2);
  double K_xy_norm = std::sqrt(K_px * K_px + K_py * K_py);

  // Predicted position correction (error-state): dx = K * residual
  double pred_dx = pred_dp(0);
  double pred_dy = pred_dp(1);
  double pred_dz = pred_dp(2);
  double pred_dxy_norm = std::sqrt(pred_dx * pred_dx + pred_dy * pred_dy);

  // --- Full-state K for orientation/bias diagnostics (only if a guard is active) ---
  double pred_dtheta_norm = pred_dq.norm();
  double pred_dba_norm = pred_dba.norm();
  double pred_dbg_norm = pred_dbg.norm();
  bool guards_active = (gps_alt_guard_dxy_max_ > 0.0 || gps_alt_guard_kxy_ratio_max_ > 0.0 ||
                        gps_alt_guard_dbias_max_ > 0.0);
  if (guards_active && gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED) {
    // Compute full K for the entire state: K_full = P_full * H_full^T / S.
    // H_full has a 1 at the global index of state->_imu->p()(2), 0 elsewhere.
    Eigen::MatrixXd P_full = StateHelper::get_full_covariance(state);
    int pz_global_idx = state->_imu->p()->id() + 2;  // p_z = 3rd element of IMU position
    if (pz_global_idx >= 0 && pz_global_idx < P_full.cols()) {
      Eigen::VectorXd K_full = P_full.col(pz_global_idx) / S;

      // Orientation correction norm (from IMU q() block)
      int q_start = state->_imu->q()->id();
      if (q_start >= 0 && q_start + 2 < K_full.size())
        pred_dtheta_norm = K_full.segment(q_start, 3).norm() * std::fabs(res_scalar);

      // Accel bias correction norm
      int ba_start = state->_imu->ba()->id();
      if (ba_start >= 0 && ba_start + 2 < K_full.size())
        pred_dba_norm = K_full.segment(ba_start, 3).norm() * std::fabs(res_scalar);

      // Gyro bias correction norm
      int bg_start = state->_imu->bg()->id();
      if (bg_start >= 0 && bg_start + 2 < K_full.size())
        pred_dbg_norm = K_full.segment(bg_start, 3).norm() * std::fabs(res_scalar);
    }
  }

  // --- Cross-covariance guard: reject before EKF update if contamination is too large ---
  std::string decision = "APPLY";
  const bool exceeds_dxy_guard =
      gps_alt_guard_dxy_max_ > 0.0 && pred_dxy_norm > gps_alt_guard_dxy_max_;
  const bool exceeds_gain_ratio_guard =
      gps_alt_guard_kxy_ratio_max_ > 0.0 && K_pz_gain != 0.0 &&
      K_xy_norm / std::fabs(K_pz_gain) > gps_alt_guard_kxy_ratio_max_;
  const bool exceeds_bias_guard =
      gps_alt_guard_dbias_max_ > 0.0 &&
      std::max(pred_dba_norm, pred_dbg_norm) > gps_alt_guard_dbias_max_;
  const bool large_residual = gps_alt_max_res_gate_ < 1e8 &&
                              std::fabs(raw_res) > gps_alt_max_res_gate_;
  const bool large_coupled_correction = exceeds_dxy_guard ||
                                        exceeds_gain_ratio_guard ||
                                        exceeds_bias_guard || large_residual;
  if (large_coupled_correction) gps_alt_stats_.n_large_coupled++;

  // Guard 1: predicted XY position correction too large
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED &&
      exceeds_dxy_guard) {
    decision = "REJECT_BY_DXY";
  }
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED &&
      decision == "APPLY" && exceeds_gain_ratio_guard) {
    decision = "REJECT_BY_GAIN_RATIO";
  }
  // Guard 3: predicted bias correction too large
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED &&
      decision == "APPLY" && exceeds_bias_guard) {
    decision = "REJECT_BY_BIAS";
  }

  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED &&
      decision != "APPLY") {
    if (decision == "REJECT_BY_DXY") gps_alt_stats_.n_rejected_dxy++;
    else if (decision == "REJECT_BY_GAIN_RATIO") gps_alt_stats_.n_rejected_kxy++;
    else if (decision == "REJECT_BY_BIAS") gps_alt_stats_.n_rejected_bias++;
    gps_alt_stats_.n_rejected++;

    // Populate snapshot with guard info before returning
    gps_alt_last_.t = t_state;
    gps_alt_last_.gps_z = altitude_z;
    gps_alt_last_.vio_z = p_IinG(2);
    gps_alt_last_.residual = raw_res;
    gps_alt_last_.pzz = P_pz;
    gps_alt_last_.kpz = K_pz_gain;
    gps_alt_last_.kpx = K_px;
    gps_alt_last_.kpy = K_py;
    gps_alt_last_.kxy_norm = K_xy_norm;
    gps_alt_last_.dx = pred_dx;
    gps_alt_last_.dy = pred_dy;
    gps_alt_last_.dz = pred_dz;
    gps_alt_last_.dtheta_norm = pred_dtheta_norm;
    gps_alt_last_.dba_norm = pred_dba_norm;
    gps_alt_last_.dbg_norm = pred_dbg_norm;
    gps_alt_last_.chi2 = chi2;
    gps_alt_last_.decision = decision;

    PRINT_INFO(YELLOW "[GPS-ALT-GUARD] %s t=%.3f res=%.2f |dxy|=%.4f K_xy/|Kz|=%.4f "
               "|dbias|=%.5f |dtheta|=%.5f - skipping update\n" RESET,
               decision.c_str(), t_state, raw_res, pred_dxy_norm,
               (K_pz_gain != 0.0 ? K_xy_norm / std::fabs(K_pz_gain) : -1.0),
               std::max(pred_dba_norm, pred_dbg_norm), pred_dtheta_norm);
    return;
  }

  // Test 2 keeps the complete Kalman direction and applies one common gain
  // scale. Scaling K (rather than clipping state blocks independently) allows
  // the same effective gain to be used in the Joseph covariance update.
  double gain_scale = 1.0;           // BOUNDED-mode uniform gain scale (alpha)
  double nasa_beta_eff = 0.0;        // NASA_LEAN underweight coefficient actually applied
  double R_joseph = R(0, 0);         // measurement noise fed to Joseph (R_eff for NASA)
  Eigen::VectorXd K_used_full;       // effective gain (state injection + Joseph share it)
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::BOUNDED) {
    auto apply_limit = [&gain_scale](double predicted_norm, double limit) {
      if (limit > 0.0 && predicted_norm > limit && predicted_norm > 0.0)
        gain_scale = std::min(gain_scale, limit / predicted_norm);
    };
    apply_limit(std::fabs(raw_res), gps_alt_residual_soft_limit_);
    apply_limit(pred_dp.head<2>().norm(), gps_alt_max_delta_xy_);
    apply_limit(std::fabs(pred_dp(2)), gps_alt_max_delta_z_);
    apply_limit(pred_dv.norm(), gps_alt_max_delta_velocity_);
    apply_limit(pred_dq.norm(), gps_alt_max_delta_attitude_);
    apply_limit(pred_dba.norm(), gps_alt_max_delta_accel_bias_);
    apply_limit(pred_dbg.norm(), gps_alt_max_delta_gyro_bias_);
    gain_scale = std::max(0.0, std::min(1.0, gain_scale));
    if (gain_scale < 1.0 - 1e-12) gps_alt_stats_.n_bounded++;
    K_used_full = gain_scale * K_full;
  } else if (gps_alt_coupled_mode_ == GpsAltCoupledMode::NASA_LEAN) {
    // NASA measurement underweighting (NTRS 20180003657 Eq. 4.36/4.38):
    // reduce the gain by inflating the measurement-space prior uncertainty
    // H'PH, not by clipping the state increment.  Orion-style trigger on H'PH.
    const double q_hph = H_full.dot(gain_numerator_full);   // = H' P H >= 0
    const bool nasa_enabled =
        gps_alt_nasa_beta_ > 0.0 && q_hph > gps_alt_nasa_q_threshold_;
    nasa_beta_eff = nasa_enabled ? gps_alt_nasa_beta_ : 0.0;
    double W_U = 0.0;
    K_used_full = StateHelper::computeLearUnderweightGain(
        gain_numerator_full, q_hph, R(0, 0), nasa_beta_eff, R_joseph, W_U);
    if (nasa_enabled) gps_alt_stats_.n_nasa_underweight++;
  } else {
    // FULL / standard and GUARDED diagnostics: full standard gain.
    K_used_full = K_full;
  }
  const Eigen::VectorXd dx_used_full = K_used_full * raw_res;
  const Eigen::Vector3d used_dq = dx_used_full.segment(q_start, 3);
  const Eigen::Vector3d used_dp = dx_used_full.segment(p_start, 3);
  const Eigen::Vector3d used_dv = dx_used_full.segment(v_start, 3);
  const Eigen::Vector3d used_dbg = dx_used_full.segment(bg_start, 3);
  const Eigen::Vector3d used_dba = dx_used_full.segment(ba_start, 3);
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::FULL)
    decision = large_coupled_correction ? "FULL_LARGE_COUPLED" : "FULL";
  else if (gps_alt_coupled_mode_ == GpsAltCoupledMode::BOUNDED)
    decision = gain_scale < 1.0 - 1e-12 ? "BOUNDED" : "BOUNDED_FULL_GAIN";
  else if (gps_alt_coupled_mode_ == GpsAltCoupledMode::NASA_LEAN)
    decision = nasa_beta_eff > 0.0 ? "NASA_UNDERWEIGHT" : "NASA_FULL_GAIN";

  // === capture state BEFORE update for delta computation ===
  Eigen::Vector3d ba_pre = state->_imu->bias_a();
  Eigen::Vector3d bg_pre = state->_imu->bias_g();
  Eigen::Vector3d v_pre = state->_imu->vel();
  Eigen::Vector3d p_pre = state->_imu->pos();
  Eigen::Matrix3d R_GtoI_pre = state->_imu->Rot();
  double z_before = p_pre(2);
  const double yaw_before_gps_alt = current_imu_yaw_deg();
  StateHelper::reset_last_yaw_dx_projection_diag();

  StateHelper::JosephUpdateHealth covariance_health;
  if (gps_alt_coupled_mode_ != GpsAltCoupledMode::GUARDED) {
    const bool check_psd = gps_alt_covariance_psd_check_interval_ <= 0.0 ||
                           gps_alt_last_covariance_psd_check_time_ < 0.0 ||
                           t_state - gps_alt_last_covariance_psd_check_time_ >=
                               gps_alt_covariance_psd_check_interval_ ||
                           large_coupled_correction || gain_scale < 1.0 - 1e-12 ||
                           nasa_beta_eff > 0.0;
    // NASA_LEAN feeds the matched effective noise R_eff = R + beta*H'PH; all
    // other modes pass the original R unchanged (R_joseph defaults to R(0,0)).
    const bool applied = StateHelper::EKFUpdateJosephChecked(
        state, K_used_full, H_full, R_joseph, raw_res, check_psd,
        &covariance_health);
    if (check_psd) gps_alt_last_covariance_psd_check_time_ = t_state;
    if (!applied) {
      gps_alt_stats_.n_rejected++;
      gps_alt_stats_.n_rejected_numerical++;
      decision = "REJECT_NUMERICAL_COVARIANCE";
      gps_alt_last_.t = t_state;
      gps_alt_last_.gps_z = altitude_z;
      gps_alt_last_.vio_z = z_before;
      gps_alt_last_.residual = raw_res;
      gps_alt_last_.pzz = P_pz;
      gps_alt_last_.kpz = K_pz_gain;
      gps_alt_last_.kpx = K_px;
      gps_alt_last_.kpy = K_py;
      gps_alt_last_.kxy_norm = K_xy_norm;
      gps_alt_last_.dx = pred_dx;
      gps_alt_last_.dy = pred_dy;
      gps_alt_last_.dz = pred_dz;
      gps_alt_last_.dtheta_norm = pred_dtheta_norm;
      gps_alt_last_.dba_norm = pred_dba_norm;
      gps_alt_last_.dbg_norm = pred_dbg_norm;
      gps_alt_last_.chi2 = chi2;
      gps_alt_last_.innovation_variance = S;
      gps_alt_last_.gain_scale = gain_scale;
      gps_alt_last_.large_coupled_correction = large_coupled_correction;
      gps_alt_last_.covariance_psd_checked = covariance_health.psd_checked;
      gps_alt_last_.covariance_psd = covariance_health.psd;
      gps_alt_last_.decision = decision;
      PRINT_ERROR(RED "[GPS-ALT-COUPLED] numerical rejection t=%.3f "
                      "finite=%d sym=%.3e min_diag=%.3e psd_checked=%d "
                      "min_ldlt=%.3e\n" RESET,
                  t_state, covariance_health.finite ? 1 : 0,
                  covariance_health.symmetry_error,
                  covariance_health.min_diagonal,
                  covariance_health.psd_checked ? 1 : 0,
                  covariance_health.min_ldlt_diagonal);
      return;
    }
  } else if (gps_alt_joseph_update_) {
    // PX4-style masked Joseph update (Brink 2017 / PX4 fuseHaglRng).
    // Only p_z state DOF and p_z row/column of P are updated; all other states
    // and cross-covariances are unchanged.  All guard logic above still applies.
    StateHelper::EKFUpdateJosephMasked(state, Hx_order, H, res, R,
                                       state->_imu->p(), 2);
    decision = "JOSEPH_MASKED";
  } else {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
  }

  // === capture state AFTER update ===
  Eigen::Vector3d ba_post = state->_imu->bias_a();
  Eigen::Vector3d bg_post = state->_imu->bias_g();
  Eigen::Vector3d v_post = state->_imu->vel();
  Eigen::Vector3d p_post = state->_imu->pos();
  Eigen::Matrix3d R_GtoI_post = state->_imu->Rot();
  Eigen::Vector3d dba = ba_post - ba_pre;
  Eigen::Vector3d dbg = bg_post - bg_pre;
  Eigen::Vector3d dv = v_post - v_pre;
  Eigen::Vector3d dp = p_post - p_pre;
  const Eigen::MatrixXd P_full_after_update = StateHelper::get_full_covariance(state);
  const double Pzz_after_update = P_full_after_update(p_start + 2, p_start + 2);
  const double Pxz_before = P_full_before(p_start, p_start + 2);
  const double Pyz_before = P_full_before(p_start + 1, p_start + 2);
  const double Pxz_after = P_full_after_update(p_start, p_start + 2);
  const double Pyz_after = P_full_after_update(p_start + 1, p_start + 2);
  if (gps_alt_coupled_mode_ == GpsAltCoupledMode::GUARDED) {
    covariance_health.finite = P_full_after_update.allFinite();
    covariance_health.symmetry_error = covariance_health.finite
                                           ? (P_full_after_update -
                                              P_full_after_update.transpose()).norm()
                                           : std::numeric_limits<double>::infinity();
    covariance_health.symmetric = covariance_health.finite &&
                                  covariance_health.symmetry_error <= 1e-10;
    covariance_health.min_diagonal = covariance_health.finite
                                         ? P_full_after_update.diagonal().minCoeff()
                                         : -std::numeric_limits<double>::infinity();
    covariance_health.nonnegative_diagonal = covariance_health.finite &&
                                             covariance_health.min_diagonal >= -1e-12;
  }

  const bool xy_ref_valid = gps_alt_diag_xy_valid_ &&
                            std::fabs(gps_alt_diag_xy_time_ - timestamp) < 0.35;
  Eigen::Vector2d true_xy_error = Eigen::Vector2d::Constant(quiet_nan());
  if (xy_ref_valid) true_xy_error = gps_alt_diag_xy_ - p_pre.head<2>();
  double direction_cos = quiet_nan();
  double expected_xy_improvement = quiet_nan();
  if (xy_ref_valid && pred_dp.head<2>().norm() > 1e-12 &&
      true_xy_error.norm() > 1e-12) {
    direction_cos = pred_dp.head<2>().dot(true_xy_error) /
                    (pred_dp.head<2>().norm() * true_xy_error.norm());
    expected_xy_improvement = true_xy_error.norm() -
                              (true_xy_error - pred_dp.head<2>()).norm();
  }

  if (of_gps_alt_coupled_diag.is_open() && in_mechanism_diag_window(t_state)) {
    const double nan = quiet_nan();
    of_gps_alt_coupled_diag << std::setprecision(17)
        << t_state << "," << altitude_z << ","
        << (xy_ref_valid ? gps_alt_diag_xy_(0) : nan) << ","
        << (xy_ref_valid ? gps_alt_diag_xy_(1) : nan) << ","
        << p_pre(0) << "," << p_pre(1) << "," << p_pre(2) << ","
        << raw_res << "," << gain_scale * raw_res << "," << P_pz << ","
        << S << "," << chi2 << "," << Pxz_before << "," << Pyz_before << ","
        << K_p(0) << "," << K_p(1) << "," << K_p(2) << ","
        << K_v(0) << "," << K_v(1) << "," << K_v(2) << ","
        << K_q(0) << "," << K_q(1) << "," << K_q(2) << ","
        << K_ba(0) << "," << K_ba(1) << "," << K_ba(2) << ","
        << K_bg(0) << "," << K_bg(1) << "," << K_bg(2) << ","
        << pred_dp(0) << "," << pred_dp(1) << "," << pred_dp(2) << ","
        << pred_dv(0) << "," << pred_dv(1) << "," << pred_dv(2) << ","
        << pred_dq(0) << "," << pred_dq(1) << "," << pred_dq(2) << ","
        << pred_dba(0) << "," << pred_dba(1) << "," << pred_dba(2) << ","
        << pred_dbg(0) << "," << pred_dbg(1) << "," << pred_dbg(2) << ","
        << gain_scale << "," << nasa_beta_eff << "," << R_joseph << ","
        << used_dp(0) << "," << used_dp(1) << "," << used_dp(2) << ","
        << used_dv(0) << "," << used_dv(1) << "," << used_dv(2) << ","
        << used_dq(0) << "," << used_dq(1) << "," << used_dq(2) << ","
        << used_dba(0) << "," << used_dba(1) << "," << used_dba(2) << ","
        << used_dbg(0) << "," << used_dbg(1) << "," << used_dbg(2) << ","
        << (xy_ref_valid ? true_xy_error(0) : nan) << ","
        << (xy_ref_valid ? true_xy_error(1) : nan) << ","
        << direction_cos << "," << expected_xy_improvement << ","
        << (large_coupled_correction ? 1 : 0) << "," << decision << ","
        << Pzz_after_update << "," << Pxz_after << "," << Pyz_after << ","
        << (covariance_health.finite ? 1 : 0) << ","
        << covariance_health.symmetry_error << ","
        << covariance_health.min_diagonal << ","
        << (covariance_health.psd_checked ? 1 : 0) << ","
        << covariance_health.min_ldlt_diagonal << ","
        << (covariance_health.psd ? 1 : 0) << "\n";
    of_gps_alt_coupled_diag.flush();
  }
  double z_after = p_post(2);
  double tilt_pre = std::acos(std::min(1.0, std::max(-1.0, R_GtoI_pre(2, 2)))) * 180.0 / M_PI;
  double tilt_post = std::acos(std::min(1.0, std::max(-1.0, R_GtoI_post(2, 2)))) * 180.0 / M_PI;
  const double yaw_after_gps_alt = current_imu_yaw_deg();
  log_vio_yaw_update(state->_timestamp, "GPS_ALTITUDE",
                     yaw_before_gps_alt, yaw_after_gps_alt,
                     wrap_degrees(yaw_after_gps_alt - yaw_before_gps_alt),
                     state->_imu->bias_g()(2), 1, chi2, 1, 0,
                     trackFEATS ? get_feature_database_size() : -1);
  log_yaw_update_mechanism(state->_timestamp, "GPS_ALTITUDE",
                           yaw_before_gps_alt, yaw_after_gps_alt,
                           course_yaw_from_velocity_deg(v_pre),
                           course_yaw_from_velocity_deg(v_post),
                           wrap_degrees(yaw_after_gps_alt - yaw_before_gps_alt),
                           1, chi2, 1, 0,
                           trackFEATS ? get_feature_database_size() : -1);

  // --- P_zz floor AFTER update: maintain floor for next call ---
  double P_pz_floor_applied = 0.0;
  if (gps_alt_min_pzz_ > 0) {
    Eigen::MatrixXd P_post = StateHelper::get_marginal_covariance(state, Hx_order);
    double P_pz_post = P_post(pz_idx, pz_idx);
    if (P_pz_post < gps_alt_min_pzz_) {
      StateHelper::inject_pz_noise(state, gps_alt_min_pzz_ - P_pz_post);
      P_pz_floor_applied = gps_alt_min_pzz_ - P_pz_post;
    }
  }

  // --- populate diagnostic snapshot ---
  gps_alt_last_.t = t_state;
  gps_alt_last_.gps_z = altitude_z;
  gps_alt_last_.vio_z = z_before;
  gps_alt_last_.residual = raw_res;
  gps_alt_last_.pzz = P_pz;
  gps_alt_last_.kpz = K_pz_gain;
  gps_alt_last_.kpx = K_px;
  gps_alt_last_.kpy = K_py;
  gps_alt_last_.kxy_norm = K_xy_norm;
  gps_alt_last_.dx = pred_dx;
  gps_alt_last_.dy = pred_dy;
  gps_alt_last_.dz = pred_dz;
  gps_alt_last_.dtheta_norm = pred_dtheta_norm;
  gps_alt_last_.dba_norm = pred_dba_norm;
  gps_alt_last_.dbg_norm = pred_dbg_norm;
  gps_alt_last_.chi2 = chi2;
  gps_alt_last_.innovation_variance = S;
  gps_alt_last_.gain_scale = gain_scale;
  gps_alt_last_.nasa_beta = nasa_beta_eff;
  gps_alt_last_.r_effective = R_joseph;
  gps_alt_last_.large_coupled_correction = large_coupled_correction;
  gps_alt_last_.covariance_psd_checked = covariance_health.psd_checked;
  gps_alt_last_.covariance_psd = covariance_health.psd;
  gps_alt_last_.clipped = gain_scale < 1.0 - 1e-12;
  gps_alt_last_.decision = decision;

  // --- statistics ---
  gps_alt_stats_.n_accepted++;
  // Watchdog: record successful fusion and the longest gap between successes.
  if (gps_alt_stats_.last_successful_fusion_time >= 0.0) {
    const double gap = t_state - gps_alt_stats_.last_successful_fusion_time;
    if (gap > gps_alt_stats_.longest_no_fusion_duration)
      gps_alt_stats_.longest_no_fusion_duration = gap;
  }
  gps_alt_stats_.last_successful_fusion_time = t_state;
  gps_alt_stats_.consecutive_failures = 0;
  gps_alt_stats_.sum_K_pz += K_pz_gain;
  gps_alt_stats_.sum_K_xy_norm += K_xy_norm;
  gps_alt_stats_.sum_dxy_norm += pred_dxy_norm;
  if (pred_dtheta_norm >= 0) gps_alt_stats_.sum_dtheta_norm += pred_dtheta_norm;
  if (pred_dba_norm >= 0) gps_alt_stats_.sum_dba_norm += pred_dba_norm;
  if (pred_dbg_norm >= 0) gps_alt_stats_.sum_dbg_norm += pred_dbg_norm;
  gps_alt_stats_.sum_P_zz += P_pz;
  gps_alt_stats_.sum_abs_res += std::abs(raw_res);
  gps_alt_stats_.sum_abs_dpz += std::abs(dp(2));

  // Keep per-update detail behind debug output; normal stdout uses the compact
  // 30-second table above and the final table below.
  PRINT_DEBUG(CYAN "[GPS-ALT-EVAL] status=%s t_cam=%.3f dt=%+.3fs meas=%.2f z_pred=%.2f z_after=%.2f "
              "res=%+.2f chi2=%.1f P_zz=%.4f K_pz=%.5f |K_xy|=%.5f |dxy|_pred=%.4f "
              "|dtheta|_pred=%.4f |dba|_pred=%.5f |dbg|_pred=%.5f dp_z_actual=%+.3f %s%s\n" RESET,
              decision.c_str(),
              t_state, dt_gps, altitude_z, z_before, z_after,
              raw_res, chi2, P_pz, K_pz_gain, K_xy_norm, pred_dxy_norm,
              pred_dtheta_norm, pred_dba_norm, pred_dbg_norm, dp(2),
              also_update_vz ? " +vz" : "",
              P_pz_floor_applied > 0 ? " FLOOR" : "");

  PRINT_DEBUG(MAGENTA "[GPS-ALT-DIAG] t=%.3f | p_pre=[%.2f %.2f %.2f] dp=[%+.3f %+.3f %+.3f] | v_pre=[%.2f %.2f %.2f] dv=[%+.3f %+.3f %+.3f] | tilt %.2f->%.2f deg | ba_pre=[%+.3f %+.3f %+.3f] dba=[%+.4f %+.4f %+.4f] | bg_pre=[%+.4f %+.4f %+.4f] dbg=[%+.5f %+.5f %+.5f]\n" RESET,
              timestamp,
              p_pre(0), p_pre(1), p_pre(2), dp(0), dp(1), dp(2),
              v_pre(0), v_pre(1), v_pre(2), dv(0), dv(1), dv(2),
              tilt_pre, tilt_post,
              ba_pre(0), ba_pre(1), ba_pre(2), dba(0), dba(1), dba(2),
              bg_pre(0), bg_pre(1), bg_pre(2), dbg(0), dbg(1), dbg(2));

}

void VioManager::feed_measurement_gps_altitude_relative(double timestamp, double altitude_z, double sigma,
                                                         double chi2_gate, bool also_update_vz) {

  if (!is_initialized_vio) {
    return;
  }

  Eigen::Vector3d p_IinG = state->_imu->pos();
  double p_z = p_IinG(2);

  // Bootstrap references on first call, then delegate to the absolute method
  // with an effective altitude that shifts GPS into VIO's world frame:
  //   effective_alt = (GPS_now - GPS_ref) + VIO_ref
  // Then res = effective_alt - p_z = (GPS_now - GPS_ref) - (p_z - VIO_ref)
  if (!gps_alt_rel_bootstrapped_) {
    gps_alt_rel_gps_ref_ = altitude_z;
    gps_alt_rel_vio_ref_ = p_z;
    gps_alt_rel_bootstrapped_ = true;
    PRINT_INFO(CYAN "[GPS-ALT-REL] bootstrap: gps_ref=%.3f vio_ref=%.3f\n" RESET,
               gps_alt_rel_gps_ref_, gps_alt_rel_vio_ref_);
  }

  double effective_alt = altitude_z - gps_alt_rel_gps_ref_ + gps_alt_rel_vio_ref_;

  feed_measurement_gps_altitude(timestamp, effective_alt, sigma, chi2_gate, also_update_vz);
}

bool VioManager::feed_measurement_gps_ground_plane(double timestamp, double height,
                                                     double sigma, bool rangefinder) {

  if (!is_initialized_vio)
    return false;
  // Optional bootstrap-delay gate (Stage A ablation): drop ground-plane
  // calls until enough time has passed since VIO init.  This lets the
  // monocular scale settle before z_ground is locked.
  if (gps_alt_min_t_after_init_ > 0.0 && startup_time > 0.0 &&
      timestamp - startup_time < gps_alt_min_t_after_init_) {
    return false;
  }

  // Relative-mode bootstrap: align GPS ENU-z to VIO world z on first call.
  // z_gps is near zero (ENU-relative) while p_z can be tens of meters.
  // We compute effective_height = (gps_z - gps_ref) + vio_ref so the residual
  // reflects VIO drift relative to GPS, not the absolute frame offset.
  double effective_height = height;
  if (!rangefinder && !gps_alt_rel_bootstrapped_) {
    gps_alt_rel_gps_ref_ = height;
    gps_alt_rel_vio_ref_ = state->_imu->pos()(2);
    gps_alt_rel_bootstrapped_ = true;
    PRINT_INFO(CYAN "[GPLANE-HEIGHT] bootstrap: gps_ref=%.3f vio_ref=%.3f\n" RESET,
               gps_alt_rel_gps_ref_, gps_alt_rel_vio_ref_);
  }
  if (!rangefinder)
    effective_height = height - gps_alt_rel_gps_ref_ + gps_alt_rel_vio_ref_;

  const auto height_type = rangefinder
      ? UpdaterGroundPlaneRange::HeightMeasurementType::RANGEFINDER
      : UpdaterGroundPlaneRange::HeightMeasurementType::GPS_ALTITUDE;
  if (updaterGPlaneRange != nullptr &&
      updaterGPlaneRange->measurement_type() != height_type) {
    PRINT_WARNING(YELLOW "[GPLANE-HEIGHT] height source changed; rebuilding updater\n" RESET);
    updaterGPlaneRange.reset();
  }

  // Lazy construction of the ground-plane range updater
  if (updaterGPlaneRange == nullptr) {
    updaterGPlaneRange = std::make_shared<UpdaterGroundPlaneRange>(
        sigma,
        0.3,                   // min_cos_tilt (skip if tilt > 72°)
        10000.0,               // chi2_gate (permissive)
        gps_alt_min_pzz_,      // P_zz floor (share with GPS altitude setting)
        gps_alt_max_res_gate_,
        height_type
    );
    updaterGPlaneRange->set_visual_yaw_update_control(
        visual_yaw_mode_from_string(params.vio_yaw_update_mode),
        params.vio_yaw_update_scale, params.vio_global_yaw_oc_alpha);
    PRINT_INFO(GREEN "[GPLANE-HEIGHT] created: sigma=%.2f type=%s\n" RESET,
               sigma, rangefinder ? "rangefinder" : "gps");
  }

  const double yaw_before = current_imu_yaw_deg();
  const double vio_course_before = course_yaw_from_velocity_deg(state->_imu->vel());
  StateHelper::reset_last_yaw_dx_projection_diag();
  const bool applied = updaterGPlaneRange->try_update(state, state->_timestamp, effective_height, timestamp);
  if (applied) {
    const double yaw_after = current_imu_yaw_deg();
    const double vio_course_after = course_yaw_from_velocity_deg(state->_imu->vel());
    log_vio_yaw_update(state->_timestamp, "GPLANE_RANGE", yaw_before, yaw_after,
                       wrap_degrees(yaw_after - yaw_before), state->_imu->bias_g()(2),
                       1, -1.0, 1, 0, trackFEATS ? get_feature_database_size() : -1);
    log_yaw_update_mechanism(state->_timestamp, "GPLANE_RANGE",
                             yaw_before, yaw_after,
                             vio_course_before, vio_course_after,
                             wrap_degrees(yaw_after - yaw_before),
                             1, -1.0, 1, 0,
                             trackFEATS ? get_feature_database_size() : -1);
  }
  return applied;
}

void VioManager::enable_gplane_feature(double sigma_pixel, int max_features,
                                       double center_frac, double min_cos_tilt,
                                       double max_residual_px) {
  updaterGPlaneFeature = std::make_shared<UpdaterGroundPlaneFeature>(
      sigma_pixel, max_features, center_frac, min_cos_tilt, max_residual_px);
  updaterGPlaneFeature->set_visual_yaw_update_control(
      visual_yaw_mode_from_string(params.vio_yaw_update_mode),
      params.vio_yaw_update_scale, params.vio_global_yaw_oc_alpha);
  PRINT_INFO(GREEN "[GPLANE-FEAT] enabled: sigma_px=%.2f K=%d center=%.2f "
             "min_cos_tilt=%.2f max_res=%.1fpx\n" RESET,
             sigma_pixel, max_features, center_frac, min_cos_tilt, max_residual_px);
}

void VioManager::enable_gplane_feature_v1(bool dry_run, double sigma_pixel,
                                          int max_features, double center_frac,
                                          double min_cos_tilt, double max_residual_px,
                                          double fd_step_rot, double fd_step_pos,
                                          double fd_rel_tol_rot,
                                          double fd_rel_tol_pos,
                                          double fd_max_abs_rel_tol,
                                          bool exclude_used_from_msckf,
                                          int dump_first_n) {
  auto mode = dry_run ? UpdaterGroundPlaneFeatureV1::Mode::DRY_RUN
                      : UpdaterGroundPlaneFeatureV1::Mode::UPDATE;
  updaterGPlaneFeatureV1 = std::make_shared<UpdaterGroundPlaneFeatureV1>(
      mode, sigma_pixel, max_features, center_frac, min_cos_tilt,
      max_residual_px, /*min_lambda*/ 0.5, /*max_lambda*/ 100.0,
      fd_step_rot, fd_step_pos,
      fd_rel_tol_rot, fd_rel_tol_pos, fd_max_abs_rel_tol,
      exclude_used_from_msckf, dump_first_n);
  updaterGPlaneFeatureV1->set_visual_yaw_update_control(
      visual_yaw_mode_from_string(params.vio_yaw_update_mode),
      params.vio_yaw_update_scale, params.vio_global_yaw_oc_alpha);
  PRINT_INFO(GREEN "[GPLANE-V1] enabled mode=%s sigma_px=%.2f K=%d center=%.2f "
             "min_cos_tilt=%.2f max_res=%.1fpx fd_step_rot=%.1e fd_step_pos=%.1e "
             "rel_tol_rot=%.1e rel_tol_pos=%.1e max_abs_rel_tol=%.1e exclude_msckf=%d dump_first_n=%d\n" RESET,
             dry_run ? "DRY_RUN" : "UPDATE",
             sigma_pixel, max_features, center_frac, min_cos_tilt,
             max_residual_px, fd_step_rot, fd_step_pos,
             fd_rel_tol_rot, fd_rel_tol_pos, fd_max_abs_rel_tol,
             exclude_used_from_msckf ? 1 : 0,
             dump_first_n);
}

VioManager::MsckfLastStats VioManager::get_last_msckf_stats() const {
  const auto &s = updaterMSCKF->get_last_stats();
  MsckfLastStats out;
  out.n_features_in     = s.n_features_in;
  out.n_tri_failed      = s.n_tri_failed;
  out.n_chi2_rejected   = s.n_chi2_rejected;
  out.n_accepted        = s.n_accepted;
  out.chi2_sum_rej      = s.chi2_sum_rej;
  out.chi2_max_rej      = s.chi2_max_rej;
  out.chi2_sum_acc      = s.chi2_sum_acc;
  out.chi2_max_acc      = s.chi2_max_acc;
  out.chi2_threshold_sum_acc = s.chi2_threshold_sum_acc;
  out.chi2_threshold_sum_rej = s.chi2_threshold_sum_rej;
  out.chi2_threshold_max_acc = s.chi2_threshold_max_acc;
  out.chi2_threshold_max_rej = s.chi2_threshold_max_rej;
  out.track_len_sum_acc = s.track_len_sum_acc;
  out.track_len_max_acc = s.track_len_max_acc;
  out.track_len_sum_rej = s.track_len_sum_rej;
  out.track_len_max_rej = s.track_len_max_rej;
  out.uv_count_acc      = s.uv_count_acc;
  out.uv_count_rej      = s.uv_count_rej;
  out.uv_sum_u_acc      = s.uv_sum_u_acc;
  out.uv_sum_v_acc      = s.uv_sum_v_acc;
  out.uv_sum_u_rej      = s.uv_sum_u_rej;
  out.uv_sum_v_rej      = s.uv_sum_v_rej;
  const auto &t        = s.tri;
  out.tri_cond_bad          = t.n_cond_bad;
  out.tri_depth_near        = t.n_depth_near;
  out.tri_depth_far         = t.n_depth_far;
  out.tri_nan               = t.n_nan;
  out.tri_gn_depth_near     = t.n_gn_depth_near;
  out.tri_gn_depth_far      = t.n_gn_depth_far;
  out.tri_gn_baseline_ratio = t.n_gn_baseline_ratio;
  out.tri_gn_nan            = t.n_gn_nan;
  out.tri_n_geom            = t.n_geom;
  out.tri_cond_sum          = t.cond_sum;  out.tri_cond_max  = t.cond_max;
  out.tri_depth_sum         = t.depth_sum; out.tri_depth_max = t.depth_max;
  out.tri_base_sum          = t.base_sum;  out.tri_base_max  = t.base_max;
  out.tri_ratio_sum         = t.ratio_sum; out.tri_ratio_max = t.ratio_max;
  return out;
}

void VioManager::set_enable_vio_yaw_update(bool v) {
  params.enable_vio_yaw_update = v;
  if (!v)
    params.vio_yaw_update_mode = "per_block_scale";
  set_vio_yaw_update_scale(v ? params.vio_yaw_update_scale : 0.0);
}

void VioManager::set_vio_yaw_update_mode(const std::string &mode) {
  params.vio_yaw_update_mode = mode;
  apply_yaw_control_to_updaters(params.vio_yaw_update_mode, params.vio_yaw_update_scale,
                                params.vio_global_yaw_oc_alpha, updaterMSCKF, updaterSLAM);
  // Ground-plane updaters use legacy mode only (no VOP support yet)
  const auto m = visual_yaw_mode_from_string(params.vio_yaw_update_mode);
  if (updaterGPlaneRange)
    updaterGPlaneRange->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                      params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeature)
    updaterGPlaneFeature->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                        params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeatureV1)
    updaterGPlaneFeatureV1->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                          params.vio_global_yaw_oc_alpha);
}

void VioManager::set_vio_yaw_update_scale(double scale) {
  params.vio_yaw_update_scale = std::max(0.0, std::min(1.0, scale));
  params.enable_vio_yaw_update = (params.vio_yaw_update_mode == "original" ||
                                  params.vio_yaw_update_mode == "global_yaw_oc_projection" ||
                                  params.vio_yaw_update_mode == "global_yaw_oc_fej_projection" ||
                                  VisualObservabilityPolicy::is_prechi2_mode_string(params.vio_yaw_update_mode) ||
                                  params.vio_yaw_update_scale > 0.0 ||
                                  params.vio_global_yaw_oc_alpha > 0.0);
  apply_yaw_control_to_updaters(params.vio_yaw_update_mode, params.vio_yaw_update_scale,
                                params.vio_global_yaw_oc_alpha, updaterMSCKF, updaterSLAM);
  const auto m = visual_yaw_mode_from_string(params.vio_yaw_update_mode);
  if (updaterGPlaneRange)
    updaterGPlaneRange->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                      params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeature)
    updaterGPlaneFeature->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                        params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeatureV1)
    updaterGPlaneFeatureV1->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                          params.vio_global_yaw_oc_alpha);
}

void VioManager::set_vio_global_yaw_oc_alpha(double alpha) {
  params.vio_global_yaw_oc_alpha = std::max(0.0, std::min(1.0, alpha));
  apply_yaw_control_to_updaters(params.vio_yaw_update_mode, params.vio_yaw_update_scale,
                                params.vio_global_yaw_oc_alpha, updaterMSCKF, updaterSLAM);
  const auto m = visual_yaw_mode_from_string(params.vio_yaw_update_mode);
  if (updaterGPlaneRange)
    updaterGPlaneRange->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                      params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeature)
    updaterGPlaneFeature->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                        params.vio_global_yaw_oc_alpha);
  if (updaterGPlaneFeatureV1)
    updaterGPlaneFeatureV1->set_visual_yaw_update_control(m, params.vio_yaw_update_scale,
                                                          params.vio_global_yaw_oc_alpha);
}

void VioManager::set_visual_bgz_update_scale(double scale) {
  params.visual_bgz_update_scale = std::max(0.0, std::min(1.0, scale));
  if (updaterMSCKF)
    updaterMSCKF->set_visual_bgz_update_scale(params.visual_bgz_update_scale);
  if (updaterSLAM)
    updaterSLAM->set_visual_bgz_update_scale(params.visual_bgz_update_scale);
  PRINT_INFO(CYAN "[VioManager] visual_bgz_update_scale=%.4f\n" RESET, params.visual_bgz_update_scale);
}

void VioManager::set_vio_yaw_update_diag_path(const std::string &path) {
  if (of_vio_yaw_update_diag.is_open())
    of_vio_yaw_update_diag.close();
  params.vio_yaw_update_diag_path = path;
  if (path.empty())
    return;

  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_vio_yaw_update_diag.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_vio_yaw_update_diag.is_open()) {
    PRINT_WARNING(YELLOW "[VIO-YAW-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_vio_yaw_update_diag
      << "timestamp,update_type,yaw_before_deg,yaw_after_deg,delta_yaw_update_deg,"
      << "dx_yaw_before_projection_deg,dx_yaw_after_projection_deg,dx_yaw_projection_valid,"
      << "cumsum_delta_yaw_update_deg,vio_yaw_update_mode,vio_yaw_update_scale,"
      << "vio_global_yaw_oc_alpha,bg_z,num_features,chi2,"
      << "accepted,rejected,tracking_feature_count\n";
  of_vio_yaw_update_diag.flush();
  vio_yaw_update_diag_cumsum_deg = 0.0;
  PRINT_INFO(GREEN "[VIO-YAW-DIAG] writing %s (mode=%s scale=%.3f alpha=%.3f)\n" RESET,
             path.c_str(), params.vio_yaw_update_mode.c_str(), params.vio_yaw_update_scale,
             params.vio_global_yaw_oc_alpha);
}

void VioManager::set_visual_obs_diag_path(const std::string &path) {
  if (of_visual_obs_diag.is_open())
    of_visual_obs_diag.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_visual_obs_diag.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_visual_obs_diag.is_open()) {
    PRINT_WARNING(YELLOW "[VOP-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_visual_obs_diag
      << "timestamp,update_type,mode,num_features_in,num_features_accepted,"
      << "num_features_rejected,norm_HN_before,norm_HN_after,"
      << "rel_norm_HN_before,rel_norm_HN_after,"
      << "rank_N,condition_N,projection_applied,used_fej_basis,"
      << "yaw_delta_update_deg,cumsum_yaw_delta_update_deg,"
      << "chi2_mean_before_oc,chi2_mean_after_oc\n";
  of_visual_obs_diag.flush();
  visual_obs_diag_cumsum_yaw_deg = 0.0;
  PRINT_INFO(GREEN "[VOP-DIAG] writing observability diag: %s (mode=%s)\n" RESET,
             path.c_str(), params.vio_yaw_update_mode.c_str());
}

void VioManager::set_mechanism_diag_window(double t0, double t1) {
  mechanism_diag_t0_ = t0;
  mechanism_diag_t1_ = t1;
  if (updaterSLAM) {
    updaterSLAM->set_feature_yaw_contrib_diag_window(t0, t1);
    updaterSLAM->set_slam_info_reduction_diag_window(t0, t1);
    updaterSLAM->set_slam_ekf_leverage_diag_window(t0, t1);
    updaterSLAM->set_slam_stacked_ekf_diag_window(t0, t1);
    updaterSLAM->set_slam_landmark_metadata_diag_window(t0, t1);
  }
  if (visual_residual_diag_)
    visual_residual_diag_->set_window(t0, t1);
}

bool VioManager::in_mechanism_diag_window(double timestamp) const {
  return timestamp >= mechanism_diag_t0_ && timestamp <= mechanism_diag_t1_;
}

bool VioManager::in_slam_update_freeze_window(double timestamp) const {
  return slam_freeze_t0_ >= 0.0 && slam_freeze_t1_ > slam_freeze_t0_ &&
         timestamp >= slam_freeze_t0_ && timestamp <= slam_freeze_t1_;
}

void VioManager::set_reference_course_yaw(double timestamp, double yaw_deg, bool valid) {
  const bool next_valid = valid && std::isfinite(yaw_deg);
  if (next_valid && reference_course_valid_ && reference_course_time_ >= 0.0 &&
      timestamp > reference_course_time_ + 1e-9) {
    reference_course_yaw_rate_degps_ =
        wrap_degrees(yaw_deg - reference_course_yaw_deg_) / (timestamp - reference_course_time_);
  } else if (!next_valid) {
    reference_course_yaw_rate_degps_ = quiet_nan();
  }
  reference_course_time_ = timestamp;
  reference_course_yaw_deg_ = yaw_deg;
  reference_course_valid_ = next_valid;
}

void VioManager::set_visual_flow_curl_diag_path(const std::string &path) {
  if (of_visual_flow_curl_diag.is_open())
    of_visual_flow_curl_diag.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_visual_flow_curl_diag.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_visual_flow_curl_diag.is_open()) {
    PRINT_WARNING(YELLOW "[FLOW-CURL-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_visual_flow_curl_diag
      << "time,cam_id,num_tracks,num_features_total,num_features_accepted,num_features_rejected,"
      << "image_width,image_height,warp_active,num_klt_attempted,num_klt_newly_detected,"
      << "mean_u,mean_v,std_u,std_v,count_left,count_right,count_top,count_bottom,"
      << "count_q1,count_q2,count_q3,count_q4,"
      << "mean_du,mean_dv,std_du,std_dv,mean_flow_mag,p95_flow_mag,"
      << "flow_curl_proxy,flow_radial_proxy,flow_tangential_proxy,"
      << "left_right_flow_asymmetry,top_bottom_flow_asymmetry,"
      << "mean_track_age,median_track_age,new_track_count,lost_track_count,"
      << "accepted_mean_u,accepted_mean_v,rejected_mean_u,rejected_mean_v\n";
  of_visual_flow_curl_diag.flush();
  PRINT_INFO(GREEN "[FLOW-CURL-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), mechanism_diag_t0_, mechanism_diag_t1_);
}

void VioManager::configure_visual_update_adaptive(bool enabled,
                                                  double target_flow_px,
                                                  double min_dt,
                                                  double max_dt,
                                                  int min_tracks) {
  visual_update_adaptive_enable_ = enabled;
  visual_update_adaptive_target_flow_px_ = std::max(0.0, target_flow_px);
  visual_update_adaptive_min_dt_ = std::max(0.0, min_dt);
  visual_update_adaptive_max_dt_ = std::max(0.0, max_dt);
  if (visual_update_adaptive_max_dt_ > 0.0 &&
      visual_update_adaptive_max_dt_ < visual_update_adaptive_min_dt_) {
    visual_update_adaptive_max_dt_ = visual_update_adaptive_min_dt_;
  }
  visual_update_adaptive_min_tracks_ = std::max(0, min_tracks);
  visual_update_adaptive_last_update_time_ = -1.0;
  visual_update_adaptive_accum_flow_px_ = 0.0;
  visual_update_counters_.visual_update_adaptive_enabled = enabled ? 1 : 0;
}

void VioManager::set_yaw_update_mechanism_diag_path(const std::string &path) {
  if (of_yaw_update_mechanism_diag.is_open())
    of_yaw_update_mechanism_diag.close();
  if (path.empty())
    return;
  boost::filesystem::path p(path);
  if (!p.parent_path().empty())
    boost::filesystem::create_directories(p.parent_path());
  of_yaw_update_mechanism_diag.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_yaw_update_mechanism_diag.is_open()) {
    PRINT_WARNING(YELLOW "[YAW-MECH-DIAG] failed to open %s\n" RESET, path.c_str());
    return;
  }
  of_yaw_update_mechanism_diag
      << "time,update_type,gps_course_yaw,reference_course_yaw,reference_age_s,"
      << "vio_course_yaw_before,vio_course_yaw_after,state_yaw_before,state_yaw_after,"
      << "yaw_error_before,yaw_error_after,state_yaw_error_before,state_yaw_error_after,"
      << "vio_course_yaw_error_before,vio_course_yaw_error_after,"
      << "innovation,residual,jacobian_sign,update_direction,delta_yaw_update,"
      << "accepted,rejected,reject_reason,chi2,threshold,num_features_used,"
      << "mean_feature_u,mean_feature_v,tracking_feature_count\n";
  of_yaw_update_mechanism_diag.flush();
  PRINT_INFO(GREEN "[YAW-MECH-DIAG] writing %s window=[%.3f, %.3f]\n" RESET,
             path.c_str(), mechanism_diag_t0_, mechanism_diag_t1_);
}

void VioManager::set_slam_feature_yaw_contrib_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_feature_yaw_contrib_diag_path(path);
}

void VioManager::set_visual_residual_diag_paths(const std::string &feature_path,
                                                const std::string &summary_path) {
  if (!visual_residual_diag_)
    visual_residual_diag_ = std::make_shared<VisualResidualDiag>();
  visual_residual_diag_->set_window(mechanism_diag_t0_, mechanism_diag_t1_);
  visual_residual_diag_->open(feature_path, summary_path);
  if (updaterMSCKF)
    updaterMSCKF->set_visual_residual_diag(visual_residual_diag_);
  if (updaterSLAM)
    updaterSLAM->set_visual_residual_diag(visual_residual_diag_);
}

void VioManager::configure_slam_yaw_contrib_cap(bool enabled, double t0, double t1,
                                                double roll_deg, double yawrate_degps,
                                                int topk, double ratio,
                                                const std::string &mode) {
  if (!updaterSLAM)
    return;
  UpdaterSLAM::YawContribCapConfig cfg;
  cfg.enabled = enabled;
  cfg.t0 = t0;
  cfg.t1 = t1;
  cfg.roll_deg = roll_deg;
  cfg.yawrate_degps = yawrate_degps;
  cfg.topk = topk;
  cfg.ratio = ratio;
  cfg.mode = mode;
  updaterSLAM->set_yaw_contrib_cap_config(cfg);
}

void VioManager::set_slam_yaw_contrib_cap_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_yaw_contrib_cap_diag_path(path);
}

void VioManager::configure_slam_info_reduction(bool enabled, double low_ratio,
                                               double high_ratio, double alpha_max) {
  if (updaterSLAM)
    updaterSLAM->configure_slam_info_reduction(enabled, low_ratio, high_ratio, alpha_max);
}

void VioManager::set_slam_info_reduction_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_slam_info_reduction_diag_path(path);
}

void VioManager::set_slam_ekf_leverage_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_slam_ekf_leverage_diag_path(path);
}

void VioManager::set_slam_stacked_ekf_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_slam_stacked_ekf_diag_path(path);
}

void VioManager::set_slam_landmark_metadata_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_slam_landmark_metadata_diag_path(path);
}

void VioManager::configure_slam_geometry_lifecycle_refresh(bool enabled,
                                                           double min_depth_current,
                                                           double min_age_since_added,
                                                           double min_pose_landmark_cov_norm,
                                                           int min_regular_features_after_refresh,
                                                           bool require_anchor_change) {
  if (updaterSLAM)
    updaterSLAM->configure_slam_geometry_lifecycle_refresh(
        enabled, min_depth_current, min_age_since_added,
        min_pose_landmark_cov_norm, min_regular_features_after_refresh,
        require_anchor_change);
}

void VioManager::set_slam_geometry_lifecycle_refresh_diag_path(const std::string &path) {
  if (updaterSLAM)
    updaterSLAM->set_slam_geometry_lifecycle_refresh_diag_path(path);
}

void VioManager::log_visual_obs_diag(double timestamp, const std::string &update_type,
                                     int n_in, int n_accepted, int n_rejected,
                                     double norm_HN_before, double norm_HN_after,
                                     double rel_HN_before, double rel_HN_after,
                                     int rank_N, double condition_N,
                                     bool projection_applied, bool used_fej,
                                     double yaw_delta_deg,
                                     double chi2_before, double chi2_after) {
  if (!of_visual_obs_diag.is_open())
    return;
  visual_obs_diag_cumsum_yaw_deg += yaw_delta_deg;
  of_visual_obs_diag << std::fixed << std::setprecision(6)
      << timestamp << ","
      << update_type << ","
      << params.vio_yaw_update_mode << ","
      << n_in << "," << n_accepted << "," << n_rejected << ","
      << norm_HN_before << "," << norm_HN_after << ","
      << rel_HN_before << "," << rel_HN_after << ","
      << rank_N << "," << condition_N << ","
      << (projection_applied ? 1 : 0) << ","
      << (used_fej ? 1 : 0) << ","
      << yaw_delta_deg << ","
      << visual_obs_diag_cumsum_yaw_deg << ","
      << chi2_before << "," << chi2_after << "\n";
}

VioManager::VisualFlowSummary VioManager::summarize_tracker_packet(const ov_core::TrackerWarpVizPacket &pkt) {
  VisualFlowSummary out;
  if (!pkt.valid)
    return out;
  out.valid = true;
  out.timestamp = pkt.t_curr;
  out.cam_id = pkt.cam_id;
  out.image_width = pkt.curr_raw_image.cols;
  out.image_height = pkt.curr_raw_image.rows;
  out.num_tracks = (int)pkt.feature_ids.size();
  out.num_features_total = (int)pkt.curr_pts_raw.size();
  out.num_klt_attempted = pkt.n_klt_attempted;
  out.num_klt_newly_detected = pkt.n_newly_detected;
  out.warp_active = pkt.warp_active;

  const double cx = 0.5 * (double)out.image_width;
  const double cy = 0.5 * (double)out.image_height;
  std::vector<double> us, vs, dus, dvs, mags, curls, radials, tangentials;
  std::vector<double> mag_left, mag_right, mag_top, mag_bottom;
  const size_t n = std::min(pkt.curr_pts_raw.size(), pkt.prev_pts_for_viz.size());
  us.reserve(pkt.curr_pts_raw.size());
  vs.reserve(pkt.curr_pts_raw.size());
  dus.reserve(n);
  dvs.reserve(n);
  mags.reserve(n);
  curls.reserve(n);
  radials.reserve(n);
  tangentials.reserve(n);

  for (const auto &p : pkt.curr_pts_raw) {
    const double u = p.x;
    const double v = p.y;
    us.push_back(u);
    vs.push_back(v);
    if (u < cx) out.count_left++; else out.count_right++;
    if (v < cy) out.count_top++; else out.count_bottom++;
    if (u < cx && v < cy) out.count_q1++;
    else if (u >= cx && v < cy) out.count_q2++;
    else if (u < cx && v >= cy) out.count_q3++;
    else out.count_q4++;
  }

  for (size_t i = 0; i < n; i++) {
    const double u = pkt.curr_pts_raw[i].x;
    const double v = pkt.curr_pts_raw[i].y;
    const double du = pkt.curr_pts_raw[i].x - pkt.prev_pts_for_viz[i].x;
    const double dv = pkt.curr_pts_raw[i].y - pkt.prev_pts_for_viz[i].y;
    const double rx = u - cx;
    const double ry = v - cy;
    const double r = std::hypot(rx, ry);
    const double mag = std::hypot(du, dv);
    dus.push_back(du);
    dvs.push_back(dv);
    mags.push_back(mag);
    if (r > 1e-6) {
      radials.push_back((du * rx + dv * ry) / r);
      tangentials.push_back((-du * ry + dv * rx) / r);
      curls.push_back((-du * ry + dv * rx) / (r * r));
    }
    if (u < cx) mag_left.push_back(mag); else mag_right.push_back(mag);
    if (v < cy) mag_top.push_back(mag); else mag_bottom.push_back(mag);
  }

  out.mean_u = mean_or_nan(us);
  out.mean_v = mean_or_nan(vs);
  out.std_u = std_or_nan(us, out.mean_u);
  out.std_v = std_or_nan(vs, out.mean_v);
  out.mean_du = mean_or_nan(dus);
  out.mean_dv = mean_or_nan(dvs);
  out.std_du = std_or_nan(dus, out.mean_du);
  out.std_dv = std_or_nan(dvs, out.mean_dv);
  out.mean_flow_mag = mean_or_nan(mags);
  out.p95_flow_mag = percentile_or_nan(mags, 0.95);
  out.flow_curl_proxy = mean_or_nan(curls);
  out.flow_radial_proxy = mean_or_nan(radials);
  out.flow_tangential_proxy = mean_or_nan(tangentials);
  out.left_right_flow_asymmetry = mean_or_nan(mag_left) - mean_or_nan(mag_right);
  out.top_bottom_flow_asymmetry = mean_or_nan(mag_top) - mean_or_nan(mag_bottom);

  std::unordered_set<size_t> current_ids;
  current_ids.reserve(pkt.feature_ids.size());
  std::vector<double> ages;
  ages.reserve(pkt.feature_ids.size());
  auto &age_map = flow_track_age_by_cam_[pkt.cam_id];
  const auto &prev_ids = flow_prev_ids_by_cam_[pkt.cam_id];
  for (size_t id : pkt.feature_ids) {
    current_ids.insert(id);
    if (prev_ids.find(id) == prev_ids.end()) {
      out.new_track_count++;
      age_map[id] = 1;
    } else {
      age_map[id] += 1;
    }
    ages.push_back((double)age_map[id]);
  }
  for (size_t id : prev_ids) {
    if (current_ids.find(id) == current_ids.end()) {
      out.lost_track_count++;
      age_map.erase(id);
    }
  }
  out.mean_track_age = mean_or_nan(ages);
  out.median_track_age = percentile_or_nan(ages, 0.5);
  flow_prev_ids_by_cam_[pkt.cam_id] = std::move(current_ids);
  return out;
}

void VioManager::collect_visual_flow_curl_diag(const ov_core::CameraData &message) {
  const bool need_summary = visual_update_adaptive_enable_ || of_visual_flow_curl_diag.is_open();
  if (!need_summary)
    return;
  if (!visual_update_adaptive_enable_ &&
      (!of_visual_flow_curl_diag.is_open() || !in_mechanism_diag_window(message.timestamp)))
    return;
  for (size_t cam_id : message.sensor_ids) {
    ov_core::TrackerWarpVizPacket pkt;
    if (!trackFEATS || !trackFEATS->get_warp_viz_packet(cam_id, pkt))
      continue;
    if (std::fabs(pkt.t_curr - message.timestamp) > 1e-6)
      continue;
    auto s = summarize_tracker_packet(pkt);
    if (s.valid)
      latest_visual_flow_by_cam_[cam_id] = s;
  }
}

void VioManager::log_visual_flow_curl_diag(double timestamp) {
  if (!of_visual_flow_curl_diag.is_open() || !in_mechanism_diag_window(timestamp))
    return;
  const auto &st = updaterMSCKF->get_last_stats();
  double acc_mean_u = st.uv_count_acc > 0 ? st.uv_sum_u_acc / st.uv_count_acc : quiet_nan();
  double acc_mean_v = st.uv_count_acc > 0 ? st.uv_sum_v_acc / st.uv_count_acc : quiet_nan();
  double rej_mean_u = st.uv_count_rej > 0 ? st.uv_sum_u_rej / st.uv_count_rej : quiet_nan();
  double rej_mean_v = st.uv_count_rej > 0 ? st.uv_sum_v_rej / st.uv_count_rej : quiet_nan();
  for (const auto &kv : latest_visual_flow_by_cam_) {
    const auto &s = kv.second;
    if (!s.valid || std::fabs(s.timestamp - timestamp) > 1e-6)
      continue;
    of_visual_flow_curl_diag << std::fixed << std::setprecision(6)
      << timestamp << "," << s.cam_id << ","
      << s.num_tracks << "," << s.num_features_total << ","
      << st.n_accepted << "," << st.n_chi2_rejected << ","
      << s.image_width << "," << s.image_height << ","
      << (s.warp_active ? 1 : 0) << ","
      << s.num_klt_attempted << "," << s.num_klt_newly_detected << ","
      << s.mean_u << "," << s.mean_v << "," << s.std_u << "," << s.std_v << ","
      << s.count_left << "," << s.count_right << "," << s.count_top << "," << s.count_bottom << ","
      << s.count_q1 << "," << s.count_q2 << "," << s.count_q3 << "," << s.count_q4 << ","
      << s.mean_du << "," << s.mean_dv << "," << s.std_du << "," << s.std_dv << ","
      << s.mean_flow_mag << "," << s.p95_flow_mag << ","
      << s.flow_curl_proxy << "," << s.flow_radial_proxy << "," << s.flow_tangential_proxy << ","
      << s.left_right_flow_asymmetry << "," << s.top_bottom_flow_asymmetry << ","
      << s.mean_track_age << "," << s.median_track_age << ","
      << s.new_track_count << "," << s.lost_track_count << ","
      << acc_mean_u << "," << acc_mean_v << "," << rej_mean_u << "," << rej_mean_v << "\n";
  }
}

void VioManager::log_yaw_update_mechanism(double timestamp, const std::string &update_type,
                                          double state_yaw_before_deg, double state_yaw_after_deg,
                                          double vio_course_yaw_before_deg, double vio_course_yaw_after_deg,
                                          double delta_yaw_deg, int num_features, double chi2,
                                          int accepted, int rejected, int tracking_feature_count) {
  if (!of_yaw_update_mechanism_diag.is_open() || !in_mechanism_diag_window(timestamp))
    return;

  const bool ref_ok = reference_course_valid_ && std::fabs(reference_course_time_ - timestamp) < 0.35;
  const double ref_yaw = ref_ok ? reference_course_yaw_deg_ : quiet_nan();
  const double ref_age = ref_ok ? (timestamp - reference_course_time_) : quiet_nan();
  const double state_err_before = ref_ok ? wrap_degrees(state_yaw_before_deg - ref_yaw) : quiet_nan();
  const double state_err_after = ref_ok ? wrap_degrees(state_yaw_after_deg - ref_yaw) : quiet_nan();
  const double course_err_before = (ref_ok && std::isfinite(vio_course_yaw_before_deg)) ?
      wrap_degrees(vio_course_yaw_before_deg - ref_yaw) : quiet_nan();
  const double course_err_after = (ref_ok && std::isfinite(vio_course_yaw_after_deg)) ?
      wrap_degrees(vio_course_yaw_after_deg - ref_yaw) : quiet_nan();
  const double innovation = ref_ok ? wrap_degrees(ref_yaw - state_yaw_before_deg) : quiet_nan();
  const double residual = state_err_before;

  int jacobian_sign = 0;
  std::string update_direction = "no_ref";
  if (ref_ok && std::isfinite(state_err_before) && std::abs(delta_yaw_deg) > 1e-12) {
    jacobian_sign = (delta_yaw_deg * state_err_before > 0.0) ? 1 : -1;
    const double before_abs = std::fabs(state_err_before);
    const double after_abs = std::fabs(state_err_after);
    if (after_abs < before_abs - 1e-9)
      update_direction = "reduce_error";
    else if (after_abs > before_abs + 1e-9)
      update_direction = "increase_error";
    else
      update_direction = "neutral";
  }

  double threshold = quiet_nan();
  double mean_feature_u = quiet_nan();
  double mean_feature_v = quiet_nan();
  if (update_type == "MSCKF") {
    const auto &st = updaterMSCKF->get_last_stats();
    if (st.n_accepted > 0)
      threshold = st.chi2_threshold_sum_acc / st.n_accepted;
    if (st.uv_count_acc > 0) {
      mean_feature_u = st.uv_sum_u_acc / st.uv_count_acc;
      mean_feature_v = st.uv_sum_v_acc / st.uv_count_acc;
    }
  }

  const std::string reject_reason = (rejected > 0) ? "chi2_or_update_reject" : "";
  const int num_features_used = accepted >= 0 ? accepted : num_features;
  of_yaw_update_mechanism_diag << std::fixed << std::setprecision(6)
      << timestamp << "," << update_type << ","
      << ref_yaw << "," << ref_yaw << "," << ref_age << ","
      << vio_course_yaw_before_deg << "," << vio_course_yaw_after_deg << ","
      << state_yaw_before_deg << "," << state_yaw_after_deg << ","
      << state_err_before << "," << state_err_after << ","
      << state_err_before << "," << state_err_after << ","
      << course_err_before << "," << course_err_after << ","
      << innovation << "," << residual << ","
      << jacobian_sign << "," << update_direction << ","
      << delta_yaw_deg << ","
      << accepted << "," << rejected << "," << reject_reason << ","
      << chi2 << "," << threshold << "," << num_features_used << ","
      << mean_feature_u << "," << mean_feature_v << ","
      << tracking_feature_count << "\n";
}

double VioManager::current_imu_yaw_deg() const {
  Eigen::Matrix3d R_ItoG = state->_imu->Rot().transpose();
  return rot_to_rpy(R_ItoG)(2) * 180.0 / M_PI;
}

double VioManager::pose_yaw_deg(const std::shared_ptr<ov_type::PoseJPL> &pose) const {
  Eigen::Matrix3d R_ItoG = pose->Rot().transpose();
  return rot_to_rpy(R_ItoG)(2) * 180.0 / M_PI;
}

void VioManager::restore_imu_yaw_deg(double yaw_deg) {
  Eigen::Matrix3d R_ItoG = state->_imu->Rot().transpose();
  Eigen::Vector3d rpy = rot_to_rpy(R_ItoG);
  Eigen::Matrix3d R_ItoG_new = rpy_to_rot(rpy(0), rpy(1), yaw_deg * M_PI / 180.0);
  Eigen::Matrix<double, 16, 1> new_imu = state->_imu->value();
  new_imu.block(0, 0, 4, 1) = ov_core::rot_2_quat(R_ItoG_new.transpose());
  state->_imu->set_value(new_imu);
}

void VioManager::restore_pose_yaw_deg(const std::shared_ptr<ov_type::PoseJPL> &pose, double yaw_deg) {
  Eigen::Matrix3d R_ItoG = pose->Rot().transpose();
  Eigen::Vector3d rpy = rot_to_rpy(R_ItoG);
  Eigen::Matrix3d R_ItoG_new = rpy_to_rot(rpy(0), rpy(1), yaw_deg * M_PI / 180.0);
  Eigen::Matrix<double, 7, 1> new_pose = pose->value();
  new_pose.block(0, 0, 4, 1) = ov_core::rot_2_quat(R_ItoG_new.transpose());
  pose->set_value(new_pose);
}

VioManager::PoseYawSnapshot VioManager::snapshot_vio_pose_yaws() const {
  PoseYawSnapshot out;
  out.imu_yaw_deg = current_imu_yaw_deg();
  out.clone_yaw_deg.reserve(state->_clones_IMU.size());
  for (const auto &clone : state->_clones_IMU) {
    out.clone_yaw_deg.emplace_back(clone.first, pose_yaw_deg(clone.second));
  }
  return out;
}

void VioManager::restore_vio_pose_yaws(const PoseYawSnapshot &snapshot) {
  restore_imu_yaw_deg(snapshot.imu_yaw_deg);
  for (const auto &item : snapshot.clone_yaw_deg) {
    auto it = state->_clones_IMU.find(item.first);
    if (it != state->_clones_IMU.end()) {
      restore_pose_yaw_deg(it->second, item.second);
    }
  }
}

void VioManager::log_vio_yaw_update(double timestamp, const std::string &update_type,
                                    double yaw_before_deg, double yaw_after_deg,
                                    double delta_yaw_deg, double bg_z,
                                    int num_features, double chi2, int accepted, int rejected,
                                    int tracking_feature_count) {
  if (num_features > 0) {
    if (update_type == "MSCKF")
      visual_update_counters_.msckf_update_count++;
    else if (update_type == "SLAM")
      visual_update_counters_.regular_slam_update_count++;
    else if (update_type == "SLAM_DELAYED")
      visual_update_counters_.delayed_slam_update_count++;
  }
  if (!of_vio_yaw_update_diag.is_open())
    return;
  const auto dx_diag = StateHelper::get_last_yaw_dx_projection_diag();
  vio_yaw_update_diag_cumsum_deg += delta_yaw_deg;
  of_vio_yaw_update_diag << std::fixed << std::setprecision(9)
                         << timestamp << "," << update_type << ","
                         << std::setprecision(6)
                         << yaw_before_deg << "," << yaw_after_deg << ","
                         << delta_yaw_deg << ","
                         << dx_diag.dx_yaw_before_projection_deg << ","
                         << dx_diag.dx_yaw_after_projection_deg << ","
                         << (dx_diag.valid ? 1 : 0) << ","
                         << vio_yaw_update_diag_cumsum_deg << ","
                         << params.vio_yaw_update_mode << "," << params.vio_yaw_update_scale << ","
                         << params.vio_global_yaw_oc_alpha << "," << bg_z << ","
                         << num_features << "," << chi2 << ","
                         << accepted << "," << rejected << "," << tracking_feature_count << "\n";
}

// Visual update guard helpers
void VioManager::open_visual_guard_log(const std::string &path) {
  if (of_visual_guard_log_.is_open()) of_visual_guard_log_.close();
  of_visual_guard_log_.open(path, std::ofstream::out | std::ofstream::trunc);
  if (!of_visual_guard_log_.is_open()) {
    PRINT_WARNING(YELLOW "[VioManager] Cannot open visual guard log: %s\n" RESET, path.c_str());
    return;
  }
  of_visual_guard_log_
    << "timestamp,update_type,decision,reason,num_features\n";
  of_visual_guard_log_.flush();
  visual_guard_log_header_written_ = true;
  PRINT_INFO(CYAN "[VioManager] Visual guard log: %s\n" RESET, path.c_str());
}

void VioManager::load_visual_reject_file(const std::string &path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    PRINT_WARNING(YELLOW "[VioManager] Cannot open reject file: %s\n" RESET, path.c_str());
    return;
  }
  visual_reject_intervals_.clear();
  std::string line;
  int count = 0;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    double t0, t1;
    char sep;
    if (ss >> t0 >> sep >> t1 && sep == ',') {
      visual_reject_intervals_.emplace_back(t0, t1);
    } else {
      std::istringstream ss2(line);
      double t;
      if (ss2 >> t) visual_reject_intervals_.emplace_back(t - 0.005, t + 0.005);
    }
    count++;
  }
  PRINT_INFO(CYAN "[VioManager] Loaded %d reject interval(s) from %s\n" RESET, count, path.c_str());
}

// Inline guard decision helper (returns true if update should be skipped)
static bool visual_guard_should_skip(double t,
                                     double skip_t0, double skip_t1,
                                     const std::vector<std::pair<double,double>> &reject_ivs,
                                     std::string &reason) {
  if (skip_t0 >= 0.0 && t >= skip_t0 && t <= skip_t1) {
    reason = "skip_window";
    return true;
  }
  for (const auto &iv : reject_ivs) {
    if (t >= iv.first && t <= iv.second) {
      reason = "reject_file";
      return true;
    }
  }
  return false;
}

void VioManager::apply_visual_update_with_yaw_diag(const std::string &update_type,
                                                   const std::function<void()> &update_fn,
                                                   int num_features,
                                                   double chi2,
                                                   int accepted) {
  const double t_now = state->_timestamp;
  if (num_features > 0 && update_type == "MSCKF") {
    visual_update_counters_.msckf_attempt_count++;
    visual_update_counters_.msckf_features_attempted += (size_t)num_features;
  }

  // Guard: skip window / reject file
  std::string guard_reason;
  if (visual_guard_should_skip(t_now, visual_skip_t0_, visual_skip_t1_,
                                visual_reject_intervals_, guard_reason)) {
    if (of_visual_guard_log_.is_open()) {
      of_visual_guard_log_ << std::fixed << std::setprecision(9)
        << t_now << "," << update_type << ",skipped," << guard_reason
        << "," << num_features << "\n";
      of_visual_guard_log_.flush();
    }
    PRINT_DEBUG(YELLOW "[GUARD] t=%.3f  %s  decision=skipped  reason=%s  n=%d\n" RESET,
                t_now, update_type.c_str(), guard_reason.c_str(), num_features);
    return;
  }

  const double yaw_before = current_imu_yaw_deg();
  const double vio_course_before = course_yaw_from_velocity_deg(state->_imu->vel());
  StateHelper::reset_last_yaw_dx_projection_diag();
  if (visual_residual_diag_) {
    VisualResidualDiag::Context ctx;
    ctx.yaw_rate = reference_course_yaw_rate_degps_;
    if (gps_alt_last_.t > 0.0 && std::fabs(gps_alt_last_.t - state->_timestamp) < 0.35) {
      ctx.gpsz_residual = gps_alt_last_.residual;
      ctx.gpsz_status = gps_alt_last_.decision;
    }
    visual_residual_diag_->set_context(ctx);
  }
  update_fn();

  if (accepted < 0) {
    if (update_type == "MSCKF") {
      const auto &s = updaterMSCKF->get_last_stats();
      accepted = s.n_accepted;
      chi2 = (s.n_accepted > 0) ? (s.chi2_sum_acc / s.n_accepted) : -1.0;
    } else {
      accepted = num_features;
    }
  }
  if (accepted > 0 && update_type == "MSCKF") {
    visual_update_counters_.msckf_accept_count++;
    visual_update_counters_.msckf_features_accepted += (size_t)accepted;
  }

  const double yaw_after = current_imu_yaw_deg();
  const double vio_course_after = course_yaw_from_velocity_deg(state->_imu->vel());
  const double delta_yaw = wrap_degrees(yaw_after - yaw_before);
  const int tracking_count = trackFEATS ? get_feature_database_size() : -1;
  const int rejected = (accepted >= 0 && num_features >= accepted) ? (num_features - accepted) : -1;
  log_vio_yaw_update(state->_timestamp, update_type, yaw_before, yaw_after, delta_yaw,
                     state->_imu->bias_g()(2), num_features, chi2, accepted, rejected,
                     tracking_count);
  log_yaw_update_mechanism(state->_timestamp, update_type, yaw_before, yaw_after,
                           vio_course_before, vio_course_after, delta_yaw,
                           num_features, chi2, accepted, rejected, tracking_count);
  if (of_visual_guard_log_.is_open()) {
    of_visual_guard_log_ << std::fixed << std::setprecision(9)
      << state->_timestamp << "," << update_type << ",accepted,,"
      << num_features << "\n";
    of_visual_guard_log_.flush();
  }

  // Visual observability policy diagnostics (MSCKF only for now)
  if (of_visual_obs_diag.is_open() && update_type == "MSCKF") {
    const auto &oc = updaterMSCKF->get_last_oc_diag();
    int n = std::max(1, oc.n_features);
    log_visual_obs_diag(state->_timestamp, update_type,
                        num_features, accepted, std::max(0, rejected),
                        oc.sum_norm_HN_before / n, oc.sum_norm_HN_after / n,
                        oc.sum_rel_HN_before / n, oc.sum_rel_HN_after / n,
                        0, 0.0, // rank/condition not aggregated here
                        oc.projection_applied, true,
                        delta_yaw,
                        oc.n_features > 0 ? oc.sum_chi2_before / n : -1.0,
                        oc.n_features > 0 ? oc.sum_chi2_after / n : -1.0);
  }
}

void VioManager::print_gps_alt_final_summary() {
  auto &s = gps_alt_stats_;
  size_t n_evals = s.n_accepted + s.n_rejected;
  double rate = n_evals > 0 ? 100.0 * s.n_accepted / n_evals : 0;
  double span = s.last_eval_time - s.first_eval_time;
  PRINT_INFO(GREEN
             "[GPS-ALT-FINAL]\n"
             "+-------------+-------------+-------------+\n"
             "| span_s      | first_t     | last_t      |\n"
             "| %11.1f | %11.1f | %11.1f |\n"
             "+-------------+-------------+-------------+\n"
             "| calls | accepted | rejected | skipped | rate   |\n"
             "| %5zu | %8zu | %8zu | %7zu | %5.1f%% |\n"
             "+-------+----------+----------+---------+--------+\n"
             "| reject_dxy | reject_kxy | reject_bias | numerical |\n"
             "| %10zu | %10zu | %11zu | %9zu |\n"
             "+------------+------------+-------------+-----------+\n"
             "| large_coupled | bounded | nasa_underweight |\n"
             "| %13zu | %7zu | %16zu |\n"
             "+---------------+---------+------------------+\n"
             "| K_pz_mu | K_xy_mu | dxy_mu | dtheta_mu | dba_mu  | dbg_mu  |\n"
             "| %7.5f | %7.5f | %6.4f | %9.4f | %7.5f | %7.5f |\n"
             "+---------+---------+--------+-----------+---------+---------+\n"
             "| P_zz_mu | abs_res_mu | abs_dpz_mu |\n"
             "| %7.4f | %10.2f | %10.3f |\n"
             "+---------+------------+------------+\n" RESET,
             span, s.first_eval_time, s.last_eval_time,
             s.n_called, s.n_accepted, s.n_rejected, s.n_skipped, rate,
             s.n_rejected_dxy, s.n_rejected_kxy, s.n_rejected_bias,
             s.n_rejected_numerical, s.n_large_coupled, s.n_bounded,
             s.n_nasa_underweight,
             n_evals > 0 ? s.sum_K_pz / n_evals : 0.0,
             n_evals > 0 ? s.sum_K_xy_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dxy_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dtheta_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dba_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dbg_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_P_zz / n_evals : 0.0,
             n_evals > 0 ? s.sum_abs_res / n_evals : 0.0,
             s.n_accepted > 0 ? s.sum_abs_dpz / s.n_accepted : 0.0);
}

void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                             const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM = std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->_cam_intrinsics_cameras, state->_options.max_aruco_features);
    trackFEATS = trackSIM;
    // Need to also replace it in init and zv-upt since it points to the trackFEATS db pointer
    initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());
    if (params.try_zupt) {
      updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                          propagator, params.gravity_mag, params.zupt_max_velocity,
                                                          params.zupt_noise_multiplier, params.zupt_max_disparity,
                                                          params.zupt_max_altitude);
    }
    PRINT_WARNING(RED "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
  }

  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == timestamp);
      propagator->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[SIM]: your vio system should already be initialized before simulating features!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->_cam_intrinsics_cameras.at(camid)->w();
    int height = state->_cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

// =============================================================================
void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // [Gyro-aided KLT] Push a per-camera predicted inter-frame rotation into
  // the tracker before consuming the new image. This dramatically helps the
  // LK solver during fast camera rotations (e.g. turns), where the legacy
  // "previous-pixel" initial guess often falls outside the LK convergence
  // basin. We only do this once VIO is initialised (so we have a real gyro
  // bias estimate and IMU-to-camera extrinsics) and KLT is the active
  // front-end. The prediction is consumed exactly once and cleared after
  // feed_new_camera() so no stale rotation can leak to a later frame.
  // Ground-parallel warp provides its own rotation-aware initial guess via apply_H_kp; suppress
  // the gyro-aided initial-guess here to avoid double-applying the same rotation homography.
  if (params.use_gyro_aided_klt && !params.use_ground_parallel_warp && params.use_klt && is_initialized_vio && propagator != nullptr) {
  double bg_sigma_max = 0.0;
  if (params.use_gyro_aided_klt_max_bg_sigma > 0.0) {
    Eigen::MatrixXd P_bg = StateHelper::get_marginal_covariance(state, {state->_imu->bg()});
    bg_sigma_max = std::sqrt(std::max({P_bg(0, 0), P_bg(1, 1), P_bg(2, 2)}));
  }
  bool bg_sigma_ok = (params.use_gyro_aided_klt_max_bg_sigma <= 0.0) ||
                     (bg_sigma_max <= params.use_gyro_aided_klt_max_bg_sigma);
  if (!bg_sigma_ok) {
    PRINT_DEBUG("[GYRO-KLT-GATE] skip: bg_sigma_max=%.5f > %.5f rad/s\n",
                bg_sigma_max, params.use_gyro_aided_klt_max_bg_sigma);
  }
  if (bg_sigma_ok) {
    double t_off = state->_calib_dt_CAMtoIMU->value()(0);
    double t_curr_imu = message.timestamp + t_off;
    for (size_t i = 0; i < message.sensor_ids.size(); i++) {
      size_t cam_id = message.sensor_ids.at(i);
      auto it_prev = last_track_image_time_.find(cam_id);
      if (it_prev == last_track_image_time_.end() || !(it_prev->second > 0.0)) {
        continue;
      }
      double t_prev_imu = it_prev->second + t_off;
      Eigen::Matrix3d R_I0_to_I1;
      if (!propagator->compute_relative_rotation(state, t_prev_imu, t_curr_imu, R_I0_to_I1)) {
        continue;
      }
      double theta_norm = ov_core::log_so3(R_I0_to_I1).norm();
      if (params.use_gyro_aided_klt_min_rot_rad > 0.0 &&
          theta_norm < params.use_gyro_aided_klt_min_rot_rad) {
        PRINT_DEBUG("[GYRO-KLT-GATE] skip cam%zu: |theta|=%.4f rad < %.4f rad\n",
                    cam_id, theta_norm, params.use_gyro_aided_klt_min_rot_rad);
        continue;
      }
      Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(cam_id)->Rot();
      Eigen::Matrix3d R_C0_to_C1 = R_ItoC * R_I0_to_I1 * R_ItoC.transpose();
      cv::Matx33d R_cv(R_C0_to_C1(0, 0), R_C0_to_C1(0, 1), R_C0_to_C1(0, 2),
                       R_C0_to_C1(1, 0), R_C0_to_C1(1, 1), R_C0_to_C1(1, 2),
                       R_C0_to_C1(2, 0), R_C0_to_C1(2, 1), R_C0_to_C1(2, 2));
      trackFEATS->set_predicted_rotation(cam_id, R_cv);
      PRINT_DEBUG("[GYRO-KLT-GATE] use cam%zu: |theta|=%.4f rad, bg_sigma=%.5f rad/s\n",
                  cam_id, theta_norm, bg_sigma_max);
    }
  }
}

  // [Ground-parallel warp] FRAME-TO-FRAME rotation warp for KLT tracking.
  //
  // TRACKING PATH (how it works):
  //   TrackKLT warps ONLY the stored last image with H = K * R_comp * K^{-1}.
  //   The current image is NOT warped. After this warp the last image appears in the
  //   rotation component. pts_left_new from KLT are directly in raw current-image
  //   coordinates; no un-warp step is required. The estimator receives unmodified
  //   original-space observations.
  //
  // ROTATION USED (frame-to-frame, NOT absolute):
  //   R_comp = R_GtoC_curr * R_GtoC_prev^T
  //   Maps a 3-D ray expressed in the PREVIOUS camera frame to the CURRENT camera
  //   frame. gravity_warp_R_ref_ is updated each frame to store R_GtoC_curr so the
  //   next call can compute the delta.
  //
  //   current R_GtoC as "prev" for the next frame.
  //
  // GYRO-AIDED KLT: suppressed above when this warp is active, to prevent the same
  //   rotation homography being applied twice to the KLT initial guess.
  //
  // Config switch: use_ground_parallel_warp (default: false)
  if (params.use_ground_parallel_warp && params.use_klt && is_initialized_vio) {
    Eigen::Matrix3d R_GtoI_curr = state->_imu->Rot(); // R_GtoI (JPL: global→IMU)
    for (size_t i = 0; i < message.sensor_ids.size(); i++) {
      size_t cam_id = message.sensor_ids.at(i);
      Eigen::Matrix3d R_ItoC = state->_calib_IMUtoCAM.at(cam_id)->Rot();
      Eigen::Matrix3d R_GtoC_curr = R_ItoC * R_GtoI_curr; // global→camera at current time
      // First activation: save current orientation as "prev" and skip warp this frame.
      if (gravity_warp_R_ref_.find(cam_id) == gravity_warp_R_ref_.end()) {
        gravity_warp_R_ref_[cam_id] = R_GtoC_curr;
        continue;
      }
      Eigen::Matrix3d R_GtoC_prev = gravity_warp_R_ref_.at(cam_id);
      // H = K * R_comp * K^{-1} warps last image to current orientation.
      Eigen::Matrix3d R_comp = R_GtoC_curr * R_GtoC_prev.transpose();
      cv::Matx33d R_comp_cv(R_comp(0, 0), R_comp(0, 1), R_comp(0, 2),
                             R_comp(1, 0), R_comp(1, 1), R_comp(1, 2),
                             R_comp(2, 0), R_comp(2, 1), R_comp(2, 2));
      trackFEATS->set_gravity_warp(cam_id, R_comp_cv);
      PRINT_DEBUG("[VIO-WARP] cam%zu R_comp (prev→curr cam rays) = [%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]\n",
                  cam_id,
                  R_comp(0, 0), R_comp(0, 1), R_comp(0, 2),
                  R_comp(1, 0), R_comp(1, 1), R_comp(1, 2),
                  R_comp(2, 0), R_comp(2, 1), R_comp(2, 2));
      // Update "prev" for next frame (must happen before feed_new_camera consumes it).
      gravity_warp_R_ref_[cam_id] = R_GtoC_curr;
    }
    gravity_warp_ref_set_ = true;
  }

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);
  visual_update_counters_.tracker_call_count++;
  if (trackFEATS && trackFEATS->get_feature_database()) {
    const auto feats_now = trackFEATS->get_feature_database()->features_containing(message.timestamp, false, false);
    visual_update_counters_.feature_database_insert_count += feats_now.size();
    const size_t db_size = trackFEATS->get_feature_database()->size();
    visual_update_counters_.tracks_retained_final = db_size;
    visual_update_counters_.tracks_retained_max = std::max(visual_update_counters_.tracks_retained_max, db_size);
  }
  collect_visual_flow_curl_diag(message);

  // Consume the prediction and gravity warp so a later frame can't reuse stale state.
  trackFEATS->clear_predicted_rotations();
  trackFEATS->clear_gravity_warps();

  // Remember the latest image time per camera id for the next call.
  for (size_t i = 0; i < message.sensor_ids.size(); i++) {
    last_track_image_time_[message.sensor_ids.at(i)] = message.timestamp;
  }

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == message.timestamp);
      propagator->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // [Landing gate] If altitude has dropped into the configured safety band AND
  // the platform is descending (not taking off), halt MSCKF/SLAM updates
  // (propagate-only). Uses a 20% hysteresis so flutter near the threshold
  // doesn't toggle the gate. Once latched, the gate clears only when altitude
  // rises above 1.2 * threshold.
  // The velocity gate (vel_z < 0.5 m/s) prevents this from triggering during
  // takeoff when altitude is temporarily below the threshold.
  if (is_initialized_vio && params.landing_safety_min_alt_m > 0.0) {
    double alt_now = state->_imu->pos()(2);
    double vel_z_now = state->_imu->vel()(2); // +up in global frame
    double thr_in  = params.landing_safety_min_alt_m;
    double thr_out = 1.2 * params.landing_safety_min_alt_m;
    if (!landing_update_halted_ && alt_now < thr_in && vel_z_now < 0.5) {
      landing_update_halted_ = true;
      landing_halt_msg_emitted_ = false;
    } else if (landing_update_halted_ && alt_now > thr_out) {
      landing_update_halted_ = false;
      PRINT_INFO(CYAN "[LANDING-GATE] cleared: alt=%.2fm > %.2fm (hysteresis), updates re-enabled\n" RESET,
                 alt_now, thr_out);
    }
    if (landing_update_halted_ && !landing_halt_msg_emitted_) {
      PRINT_WARNING(YELLOW "[LANDING-GATE] alt=%.2fm < %.2fm vel_z=%.1f: MSCKF/SLAM updates HALTED, "
                           "dead-reckoning only. Operator: take manual control / abort evaluation.\n" RESET,
                    alt_now, thr_in, vel_z_now);
      landing_halt_msg_emitted_ = true;
    }
  }

  // Call on our propagate and update function
  if (landing_update_halted_) {
    // Propagate-only path: still advance time and clone window, no update.
    if (state->_timestamp < message.timestamp) {
      propagator->propagate_and_clone(state, message.timestamp);
    }
  } else {
    do_feature_propagate_update(message);
  }
}

// =============================================================================
// do_feature_propagate_update
//    Step 1  propagate_and_clone
//    Step 2  select and classify features
//    Step 3  run EKF feature updates
//    Step 4  retriangulate and marginalize the oldest clone
// =============================================================================
void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->_timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "image received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (message.timestamp - state->_timestamp));
    return;
  }

  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->_timestamp != message.timestamp) {
    propagator->propagate_and_clone(state, message.timestamp);
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can start triangulating things...
  if ((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size, 5)) {
    PRINT_DEBUG("waiting for enough clone states (%d of %d)....\n", (int)state->_clones_IMU.size(),
                std::min(state->_options.max_clone_size, 5));
    return;
  }

  // Return if we where unable to propagate
  if (state->_timestamp != message.timestamp) {
    PRINT_WARNING(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    PRINT_WARNING(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET, message.timestamp - state->_timestamp);
    return;
  }
  has_moved_since_zupt = true;

  visual_update_counters_.visual_update_frame_count++;
  bool visual_update_eligible = true;
  std::string visual_update_skip_reason = "visual_update_stride";

  if (visual_update_adaptive_enable_) {
    bool have_flow_summary = false;
    double frame_flow_px = 0.0;
    int track_count = -1;

    for (size_t cam_id : message.sensor_ids) {
      auto it_flow = latest_visual_flow_by_cam_.find(cam_id);
      if (it_flow == latest_visual_flow_by_cam_.end())
        continue;
      const auto &s = it_flow->second;
      if (!s.valid || std::fabs(s.timestamp - message.timestamp) > 1e-6)
        continue;
      have_flow_summary = true;
      if (std::isfinite(s.mean_flow_mag))
        frame_flow_px = std::max(0.0, s.mean_flow_mag);
      track_count = s.num_tracks;
      break;
    }

    visual_update_adaptive_accum_flow_px_ += frame_flow_px;
    const bool first_keyframe = visual_update_adaptive_last_update_time_ < 0.0;
    const double dt_since_keyframe = first_keyframe
        ? std::numeric_limits<double>::infinity()
        : std::max(0.0, state->_timestamp - visual_update_adaptive_last_update_time_);
    const bool track_safety_trigger =
        have_flow_summary && visual_update_adaptive_min_tracks_ > 0 &&
        track_count >= 0 && track_count < visual_update_adaptive_min_tracks_;
    const bool max_dt_trigger =
        !first_keyframe && visual_update_adaptive_max_dt_ > 0.0 &&
        dt_since_keyframe >= visual_update_adaptive_max_dt_;
    const bool min_dt_satisfied =
        first_keyframe || dt_since_keyframe >= visual_update_adaptive_min_dt_;
    const bool flow_trigger =
        min_dt_satisfied && visual_update_adaptive_target_flow_px_ > 0.0 &&
        visual_update_adaptive_accum_flow_px_ >= visual_update_adaptive_target_flow_px_;

    if (first_keyframe) {
      visual_update_eligible = true;
      visual_update_skip_reason = "adaptive_first";
      visual_update_counters_.visual_update_adaptive_first_trigger_count++;
    } else if (track_safety_trigger) {
      visual_update_eligible = true;
      visual_update_skip_reason = "adaptive_track_safety";
      visual_update_counters_.visual_update_adaptive_track_trigger_count++;
    } else if (max_dt_trigger) {
      visual_update_eligible = true;
      visual_update_skip_reason = "adaptive_max_dt";
      visual_update_counters_.visual_update_adaptive_maxdt_trigger_count++;
    } else if (flow_trigger) {
      visual_update_eligible = true;
      visual_update_skip_reason = "adaptive_flow_target";
      visual_update_counters_.visual_update_adaptive_flow_trigger_count++;
    } else {
      visual_update_eligible = false;
      if (!min_dt_satisfied) {
        visual_update_skip_reason = "adaptive_min_dt";
        visual_update_counters_.visual_update_adaptive_min_dt_block_count++;
      } else {
        visual_update_skip_reason = have_flow_summary ? "adaptive_flow_wait" : "adaptive_no_flow_summary";
      }
    }

    visual_update_counters_.visual_update_adaptive_last_dt_s =
        std::isfinite(dt_since_keyframe) ? dt_since_keyframe : 0.0;
    visual_update_counters_.visual_update_adaptive_last_frame_flow_px = frame_flow_px;
    visual_update_counters_.visual_update_adaptive_last_accum_flow_px =
        visual_update_adaptive_accum_flow_px_;
    visual_update_counters_.visual_update_adaptive_last_track_count = track_count;

    if (visual_update_eligible) {
      visual_update_counters_.visual_update_adaptive_keyframe_count++;
      visual_update_adaptive_last_update_time_ = state->_timestamp;
      visual_update_adaptive_accum_flow_px_ = 0.0;
    } else {
      visual_update_counters_.visual_update_adaptive_skip_count++;
    }
  } else {
    visual_update_eligible =
        (visual_update_stride_ <= 1) ||
        ((visual_update_counters_.visual_update_frame_count - 1) % (size_t)visual_update_stride_ == 0);
  }

  std::string visual_guard_reason;
  if (visual_guard_should_skip(state->_timestamp, visual_skip_t0_, visual_skip_t1_,
                               visual_reject_intervals_, visual_guard_reason)) {
    visual_update_eligible = false;
    visual_update_skip_reason = visual_guard_reason;
  }

  if (visual_update_eligible) {
    visual_update_counters_.visual_update_eligible_count++;
  } else {
    visual_update_counters_.visual_update_skipped_count++;

    size_t db_before_cleanup = 0;
    if (trackFEATS && trackFEATS->get_feature_database())
      db_before_cleanup = trackFEATS->get_feature_database()->size();

    if (visual_update_adaptive_enable_) {
      size_t current_obs_before = 0;
      size_t current_obs_after = 0;
      if (trackFEATS && trackFEATS->get_feature_database()) {
        current_obs_before =
            trackFEATS->get_feature_database()->features_containing(state->_timestamp, false, false).size();
        trackFEATS->get_feature_database()->cleanup_measurements_exact(state->_timestamp);
        current_obs_after =
            trackFEATS->get_feature_database()->features_containing(state->_timestamp, false, false).size();
      }
      if (trackARUCO != nullptr)
        trackARUCO->get_feature_database()->cleanup_measurements_exact(state->_timestamp);
      if (current_obs_before > current_obs_after) {
        visual_update_counters_.visual_update_adaptive_drop_current_observations_count +=
            (current_obs_before - current_obs_after);
      }
    }

    if (message.sensor_ids.at(0) == 0) {
      retriangulate_active_tracks(message);
      good_features_MSCKF.clear();
    }

    if (trackFEATS && trackFEATS->get_feature_database())
      trackFEATS->get_feature_database()->cleanup();
    if (trackARUCO != nullptr)
      trackARUCO->get_feature_database()->cleanup();

    if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
      const double marg_time = state->margtimestep();
      if (trackFEATS && trackFEATS->get_feature_database())
        trackFEATS->get_feature_database()->cleanup_measurements(marg_time);
      if (trackFEATS && trackFEATS->get_feature_database())
        trackFEATS->get_feature_database()->cleanup_measurements_exact(marg_time);
      if (trackARUCO != nullptr)
        trackARUCO->get_feature_database()->cleanup_measurements(marg_time);
      if (trackARUCO != nullptr)
        trackARUCO->get_feature_database()->cleanup_measurements_exact(marg_time);
    }

    size_t db_after_cleanup = 0;
    if (trackFEATS && trackFEATS->get_feature_database())
      db_after_cleanup = trackFEATS->get_feature_database()->size();
    if (db_before_cleanup > db_after_cleanup)
      visual_update_counters_.tracks_dropped_without_update += (db_before_cleanup - db_after_cleanup);
    visual_update_counters_.tracks_retained_final = db_after_cleanup;
    visual_update_counters_.tracks_retained_max = std::max(visual_update_counters_.tracks_retained_max, db_after_cleanup);

    updaterSLAM->change_anchors(state);
    StateHelper::marginalize_old_clone(state);
    propagator->invalidate_cache();
    rT4 = rT5 = rT6 = rT7 = boost::posix_time::microsec_clock::local_time();
    if (of_visual_guard_log_.is_open()) {
      of_visual_guard_log_ << std::fixed << std::setprecision(9)
        << state->_timestamp << ",ALL_VISUAL,skipped," << visual_update_skip_reason << ","
        << db_after_cleanup << "\n";
      of_visual_guard_log_.flush();
    }
    if (visual_update_adaptive_enable_) {
      PRINT_DEBUG(YELLOW "[VISUAL-UPDATE-ADAPTIVE] t=%.3f skipped visual EKF updates "
                  "(reason=%s frame=%zu dt=%.3f accum_flow=%.3f tracks=%d db=%zu)\n" RESET,
                  state->_timestamp,
                  visual_update_skip_reason.c_str(),
                  visual_update_counters_.visual_update_frame_count,
                  visual_update_counters_.visual_update_adaptive_last_dt_s,
                  visual_update_counters_.visual_update_adaptive_last_accum_flow_px,
                  visual_update_counters_.visual_update_adaptive_last_track_count,
                  db_after_cleanup);
    } else {
      PRINT_DEBUG(YELLOW "[VISUAL-UPDATE-STRIDE] t=%.3f skipped visual EKF updates "
                  "(frame=%zu stride=%d db=%zu)\n" RESET,
                  state->_timestamp,
                  visual_update_counters_.visual_update_frame_count,
                  visual_update_stride_,
                  db_after_cleanup);
    }
    return;
  }

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================
  // Select features from the FeatureDatabase for this update.

  // Now, lets get all features that should be used for an update that are lost in the newest frame
  // We explicitly request features that have not been deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;
  feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp, false, true);

  // Don't need to get the oldest features until we reach our max number of clones
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size || (int)state->_clones_IMU.size() > 5) {
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep(), false, true);
    if (trackARUCO != nullptr && message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
    }
  }

  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to use those in the update yet
  // E.g: thus we wait until cam0 process its newest image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(), camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) != feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  // Find tracks that have reached max length, these can be made into SLAM features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      if ((int)cams.second.size() > state->_options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((int)(*it0).second->_featid <= 4 * state->_options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
  if (state->_options.max_slam_features > 0 && message.timestamp - startup_time >= params.dt_slam_delay &&
      (int)state->_features_SLAM.size() < state->_options.max_slam_features + curr_aruco_tags) {
    // Get the total amount to add, then the max amount that we can add given our marginalize feature array
    int amount_to_add = (state->_options.max_slam_features + curr_aruco_tags) - (int)state->_features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
    }
  }

  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // NOTE: if we have a slam feature that has lost tracking, then we should marginalize it out
  // NOTE: we only enforce this if the current camera message is where the feature was seen from
  // NOTE: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  // NOTE: we will also marginalize SLAM features if they have failed their update a couple times in a row
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark : state->_features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->_unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(), landmark.second->_unique_camera_id) != message.sensor_ids.end();
    if (feat2 == nullptr && current_unique_cam)
      landmark.second->should_marg = true;
    if (landmark.second->update_fail_count > 1)
      landmark.second->should_marg = true;
  }

  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());

  //===================================================================================
  // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
  //===================================================================================

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  auto compare_feat = [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  };
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), compare_feat);

  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
  // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
  if ((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end() - state->_options.max_msckf_in_update);

  // [MSCKF parallax filter] Drop features whose max 2-D pixel parallax
  // across the active clone window is below the threshold. They carry
  // essentially no metric scale info at high altitude and just inject
  // noise into the H matrix.
  if (params.min_msckf_parallax_px > 0.0 && !featsup_MSCKF.empty()) {
    int n_in = (int)featsup_MSCKF.size();
    auto needs_drop = [&](const std::shared_ptr<Feature> &f) -> bool {
      double umin = 1e18, umax = -1e18, vmin = 1e18, vmax = -1e18;
      for (const auto &cam_uvs : f->uvs) {
        for (const auto &uv : cam_uvs.second) {
          if (uv(0) < umin) umin = uv(0);
          if (uv(0) > umax) umax = uv(0);
          if (uv(1) < vmin) vmin = uv(1);
          if (uv(1) > vmax) vmax = uv(1);
        }
      }
      double du = (umax > -1e17) ? (umax - umin) : 0.0;
      double dv = (vmax > -1e17) ? (vmax - vmin) : 0.0;
      double parallax_px = std::sqrt(du * du + dv * dv);
      return parallax_px < params.min_msckf_parallax_px;
    };
    featsup_MSCKF.erase(
        std::remove_if(featsup_MSCKF.begin(), featsup_MSCKF.end(), needs_drop),
        featsup_MSCKF.end());
    int n_out = (int)featsup_MSCKF.size();
    if (n_in != n_out) {
      PRINT_DEBUG("[MSCKF-PARALLAX] dropped %d/%d features below %.2f px\n",
                  n_in - n_out, n_in, params.min_msckf_parallax_px);
    }
  }

  // so MSCKF hasn't yet marked features for deletion; we only need to look
  // at the database, we don't consume features.
  if (updaterGPlaneFeature != nullptr && updaterGPlaneRange != nullptr &&
      updaterGPlaneRange->bootstrapped()) {
    double z_g = updaterGPlaneRange->z_ground();
    const double yaw_before_gplane = current_imu_yaw_deg();
    const double vio_course_before_gplane = course_yaw_from_velocity_deg(state->_imu->vel());
    StateHelper::reset_last_yaw_dx_projection_diag();
    bool gplane_fired = updaterGPlaneFeature->try_update(
        state, trackFEATS->get_feature_database(), message.timestamp, z_g);
    if (gplane_fired) {
      const double yaw_after_gplane = current_imu_yaw_deg();
      const double vio_course_after_gplane = course_yaw_from_velocity_deg(state->_imu->vel());
      const auto &last = updaterGPlaneFeature->last_update();
      log_vio_yaw_update(state->_timestamp, "GPLANE_FEATURE",
                         yaw_before_gplane, yaw_after_gplane,
                         wrap_degrees(yaw_after_gplane - yaw_before_gplane),
                         state->_imu->bias_g()(2), last.n_used, -1.0,
                         last.n_used, 0, get_feature_database_size());
      log_yaw_update_mechanism(state->_timestamp, "GPLANE_FEATURE",
                               yaw_before_gplane, yaw_after_gplane,
                               vio_course_before_gplane, vio_course_after_gplane,
                               wrap_degrees(yaw_after_gplane - yaw_before_gplane),
                               last.n_used, -1.0, last.n_used, 0,
                               get_feature_database_size());
    }
  }

  // exclusive with v0 in practice).  Same activation gate as v0: requires
  // Stage A bootstrap and Stage A delay (Stage A enforces the delay itself,
  // so once Stage A has fired, both feature updaters become eligible).
  if (updaterGPlaneFeatureV1 != nullptr && updaterGPlaneRange != nullptr &&
      updaterGPlaneRange->bootstrapped()) {
    double z_g = updaterGPlaneRange->z_ground();
    const double yaw_before_gplane_v1 = current_imu_yaw_deg();
    const double vio_course_before_gplane_v1 = course_yaw_from_velocity_deg(state->_imu->vel());
    StateHelper::reset_last_yaw_dx_projection_diag();
    bool v1_fired = updaterGPlaneFeatureV1->try_update_candidates(
        state, featsup_MSCKF, message.timestamp, z_g);
    if (v1_fired) {
      const double yaw_after_gplane_v1 = current_imu_yaw_deg();
      const double vio_course_after_gplane_v1 = course_yaw_from_velocity_deg(state->_imu->vel());
      const auto &last = updaterGPlaneFeatureV1->last_update();
      log_vio_yaw_update(state->_timestamp, "GPLANE_FEATURE_V1",
                         yaw_before_gplane_v1, yaw_after_gplane_v1,
                         wrap_degrees(yaw_after_gplane_v1 - yaw_before_gplane_v1),
                         state->_imu->bias_g()(2), last.n_used, -1.0,
                         last.n_used, 0, get_feature_database_size());
      log_yaw_update_mechanism(state->_timestamp, "GPLANE_FEATURE_V1",
                               yaw_before_gplane_v1, yaw_after_gplane_v1,
                               vio_course_before_gplane_v1, vio_course_after_gplane_v1,
                               wrap_degrees(yaw_after_gplane_v1 - yaw_before_gplane_v1),
                               last.n_used, -1.0, last.n_used, 0,
                               get_feature_database_size());
    }
    if (v1_fired && updaterGPlaneFeatureV1->exclude_used_from_msckf()) {
      const auto &used = updaterGPlaneFeatureV1->last_used_feat_ids();
      if (!used.empty()) {
        int n_in = (int)featsup_MSCKF.size();
        featsup_MSCKF.erase(
            std::remove_if(
                featsup_MSCKF.begin(), featsup_MSCKF.end(),
                [&used](const std::shared_ptr<ov_core::Feature> &f) {
                  return used.count(f->featid) > 0;
                }),
            featsup_MSCKF.end());
        int n_out = (int)featsup_MSCKF.size();
        if (n_in != n_out) {
          PRINT_DEBUG("[GPLANE-V1] dropped %d/%d MSCKF features (already used in v1 update)\n",
                      n_in - n_out, n_in);
        }
      }
    }
  }

  const int n_msckf_features = (int)featsup_MSCKF.size();
  apply_visual_update_with_yaw_diag(
      "MSCKF",
      [&]() {
        updaterMSCKF->update(state, featsup_MSCKF);
      },
      n_msckf_features, -1.0, -1);
  log_visual_flow_curl_diag(state->_timestamp);
  propagator->invalidate_cache();
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
  const bool freeze_slam_updates = in_slam_update_freeze_window(state->_timestamp);
  if (freeze_slam_updates) {
    const int n_slam_features = (int)feats_slam_UPDATE.size();
    if (n_slam_features > 0) {
      const double yaw_slam = current_imu_yaw_deg();
      const double vio_course_slam = course_yaw_from_velocity_deg(state->_imu->vel());
      StateHelper::reset_last_yaw_dx_projection_diag();
      log_vio_yaw_update(state->_timestamp, "SLAM_FROZEN", yaw_slam, yaw_slam,
                         0.0, state->_imu->bias_g()(2), n_slam_features, -1.0,
                         0, n_slam_features, get_feature_database_size());
      log_yaw_update_mechanism(state->_timestamp, "SLAM_FROZEN",
                               yaw_slam, yaw_slam,
                               vio_course_slam, vio_course_slam,
                               0.0, n_slam_features, -1.0,
                               0, n_slam_features,
                               get_feature_database_size());
    }
    feats_slam_UPDATE.clear();
    rT5 = boost::posix_time::microsec_clock::local_time();
  } else {
    std::vector<std::shared_ptr<Feature>> feats_slam_UPDATE_TEMP;
    while (!feats_slam_UPDATE.empty()) {
      // Get sub vector of the features we will update with
      std::vector<std::shared_ptr<Feature>> featsup_TEMP;
      featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(),
                          feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
      feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(),
                              feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
      // Do the update
      const int n_slam_features = (int)featsup_TEMP.size();
      if (n_slam_features > 0) {
        visual_update_counters_.regular_slam_attempt_count++;
        visual_update_counters_.regular_slam_features_attempted += (size_t)n_slam_features;
      }
      const double yaw_before_slam = current_imu_yaw_deg();
      const double vio_course_before_slam = course_yaw_from_velocity_deg(state->_imu->vel());
      const bool ref_course_ok_for_slam =
          reference_course_valid_ && std::fabs(reference_course_time_ - state->_timestamp) < 0.35;
      const double slam_course_yaw_error =
          (ref_course_ok_for_slam && std::isfinite(vio_course_before_slam)) ?
          wrap_degrees(vio_course_before_slam - reference_course_yaw_deg_) : quiet_nan();
      updaterSLAM->set_yaw_contrib_cap_context(
          slam_course_yaw_error,
          ref_course_ok_for_slam && std::isfinite(slam_course_yaw_error),
          reference_course_yaw_rate_degps_);
      StateHelper::reset_last_yaw_dx_projection_diag();
      updaterSLAM->update(state, featsup_TEMP);
      const int n_slam_accepted = (int)featsup_TEMP.size();
      if (n_slam_accepted > 0) {
        visual_update_counters_.regular_slam_accept_count++;
        visual_update_counters_.regular_slam_features_accepted += (size_t)n_slam_accepted;
      }
      const double yaw_after_slam = current_imu_yaw_deg();
      const double vio_course_after_slam = course_yaw_from_velocity_deg(state->_imu->vel());
      const int n_slam_rejected = std::max(0, n_slam_features - n_slam_accepted);
      log_vio_yaw_update(state->_timestamp, "SLAM", yaw_before_slam, yaw_after_slam,
                         wrap_degrees(yaw_after_slam - yaw_before_slam),
                         state->_imu->bias_g()(2), n_slam_features, -1.0,
                         n_slam_accepted, n_slam_rejected, get_feature_database_size());
      log_yaw_update_mechanism(state->_timestamp, "SLAM",
                               yaw_before_slam, yaw_after_slam,
                               vio_course_before_slam, vio_course_after_slam,
                               wrap_degrees(yaw_after_slam - yaw_before_slam),
                               n_slam_features, -1.0,
                               n_slam_accepted, n_slam_rejected,
                               get_feature_database_size());
      feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
      propagator->invalidate_cache();
    }
    feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
    rT5 = boost::posix_time::microsec_clock::local_time();
  }

  const int n_slam_delayed_features = (int)feats_slam_DELAYED.size();
  if (freeze_slam_updates) {
    if (n_slam_delayed_features > 0) {
      const double yaw_slam_delay = current_imu_yaw_deg();
      const double vio_course_slam_delay = course_yaw_from_velocity_deg(state->_imu->vel());
      StateHelper::reset_last_yaw_dx_projection_diag();
      log_vio_yaw_update(state->_timestamp, "SLAM_DELAYED_FROZEN",
                         yaw_slam_delay, yaw_slam_delay, 0.0,
                         state->_imu->bias_g()(2), n_slam_delayed_features, -1.0,
                         0, n_slam_delayed_features, get_feature_database_size());
      log_yaw_update_mechanism(state->_timestamp, "SLAM_DELAYED_FROZEN",
                               yaw_slam_delay, yaw_slam_delay,
                               vio_course_slam_delay, vio_course_slam_delay,
                               0.0, n_slam_delayed_features, -1.0,
                               0, n_slam_delayed_features,
                               get_feature_database_size());
    }
    feats_slam_DELAYED.clear();
  } else {
    const double yaw_before_slam_delay = current_imu_yaw_deg();
    const double vio_course_before_slam_delay = course_yaw_from_velocity_deg(state->_imu->vel());
    StateHelper::reset_last_yaw_dx_projection_diag();
    updaterSLAM->delayed_init(state, feats_slam_DELAYED);
    const int n_slam_delayed_accepted = (int)feats_slam_DELAYED.size();
    if (n_slam_delayed_features > 0) {
      visual_update_counters_.delayed_slam_attempt_count++;
      visual_update_counters_.delayed_slam_features_attempted += (size_t)n_slam_delayed_features;
    }
    if (n_slam_delayed_accepted > 0) {
      visual_update_counters_.delayed_slam_accept_count++;
      visual_update_counters_.delayed_slam_features_accepted += (size_t)n_slam_delayed_accepted;
    }
    const double yaw_after_slam_delay = current_imu_yaw_deg();
    const double vio_course_after_slam_delay = course_yaw_from_velocity_deg(state->_imu->vel());
    const int n_slam_delayed_rejected = std::max(0, n_slam_delayed_features - n_slam_delayed_accepted);
    log_vio_yaw_update(state->_timestamp, "SLAM_DELAYED", yaw_before_slam_delay,
                       yaw_after_slam_delay,
                       wrap_degrees(yaw_after_slam_delay - yaw_before_slam_delay),
                       state->_imu->bias_g()(2), n_slam_delayed_features, -1.0,
                       n_slam_delayed_accepted, n_slam_delayed_rejected, get_feature_database_size());
    log_yaw_update_mechanism(state->_timestamp, "SLAM_DELAYED",
                             yaw_before_slam_delay, yaw_after_slam_delay,
                             vio_course_before_slam_delay, vio_course_after_slam_delay,
                             wrap_degrees(yaw_after_slam_delay - yaw_before_slam_delay),
                             n_slam_delayed_features, -1.0,
                             n_slam_delayed_accepted, n_slam_delayed_rejected,
                             get_feature_database_size());
  }
  rT6 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Update our visualization feature set, and clean up the old features
  //===================================================================================

  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {

    // Re-triangulate features
    retriangulate_active_tracks(message);

    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // Save all the MSCKF features used in the update
  visual_update_counters_.tracks_consumed_count += featsup_MSCKF.size();
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }
  if (trackFEATS && trackFEATS->get_feature_database()) {
    const size_t db_size = trackFEATS->get_feature_database()->size();
    visual_update_counters_.tracks_retained_final = db_size;
    visual_update_counters_.tracks_retained_max = std::max(visual_update_counters_.tracks_retained_max, db_size);
  }

  // First do anchor change if we are about to lose an anchor pose
  updaterSLAM->change_anchors(state);

  // Cleanup any features older than the marginalization time
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
    }
  }
  if (trackFEATS && trackFEATS->get_feature_database()) {
    const size_t db_size = trackFEATS->get_feature_database()->size();
    visual_update_counters_.tracks_retained_final = db_size;
    visual_update_counters_.tracks_retained_max = std::max(visual_update_counters_.tracks_retained_max, db_size);
  }

  // Finally marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  rT7 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Debug info, and stats tracking
  //===================================================================================

  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
  double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;
  double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;
  double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6;
  double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;
  double time_total = (rT7 - rT1).total_microseconds() * 1e-6;

  // Timing information
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for MSCKF update (%d feats)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
  if (state->_options.max_slam_features > 0) {
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET, time_slam_update, (int)state->_features_SLAM.size());
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
  }
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for re-tri & marg (%d clones in state)\n" RESET, time_marg, (int)state->_clones_IMU.size());

  std::stringstream ss;
  ss << "[TIME]: " << std::setprecision(4) << time_total << " seconds for total (camera";
  for (const auto &id : message.sensor_ids) {
    ss << " " << id;
  }
  ss << ")" << std::endl;
  PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());

  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << "," << std::fixed << std::setprecision(5) << time_track << ","
                  << time_prop << "," << time_msckf << ",";
    if (state->_options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_marg << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // Update our distance traveled
  if (timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
    Eigen::Matrix<double, 3, 1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();
  }
  timelastupdate = message.timestamp;

  // Debug, print our current state
  PRINT_DEBUG("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n", state->_imu->quat()(0),
              state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3), state->_imu->pos()(0), state->_imu->pos()(1),
              state->_imu->pos()(2), distance);
  // [STATS] Per-frame size diagnostics. Helps identify which structure
  // is growing if real-time degrades over a long flight. Cheap (just sizes).
  {
    int n_slam   = (int)state->_features_SLAM.size();
    int n_clones = (int)state->_clones_IMU.size();
    int n_total  = state->max_covariance_size();
    int n_db     = (int)trackFEATS->get_feature_database()->size();
    PRINT_DEBUG("[STATS] state_dim=%d n_clones=%d n_slam=%d db=%d\n",
                n_total, n_clones, n_slam, n_db);
    slam_count_history_.emplace_back(message.timestamp, n_slam);
    while (!slam_count_history_.empty() &&
           message.timestamp - slam_count_history_.front().first > 2.0) {
      slam_count_history_.pop_front();
    }
    if (slam_count_history_.size() >= 2) {
      double t0 = slam_count_history_.front().first;
      int n0   = slam_count_history_.front().second;
      double dt = message.timestamp - t0;
      if (dt > 0.5 && n0 > 5) {
        double rate = (double)(n0 - n_slam) / (double)n0 / dt;
        if (rate > 0.30) {
          PRINT_WARNING(YELLOW "[LANDING-WATCH] SLAM dropping %.0f%%/s (%d -> %d in %.2fs). "
                               "Likely descent or scene change; consider landing_safety_min_alt_m.\n" RESET,
                        100.0 * rate, n0, n_slam, dt);
        }
      }
    }
  }

  PRINT_DEBUG("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
              state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_DEBUG("camera-imu timeoffset = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_DEBUG("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                  calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_DEBUG("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                  calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }

  // Debug for imu intrinsics
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_DEBUG("q_GYROtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_GYROtoIMU->value()(0), state->_calib_imu_GYROtoIMU->value()(1),
                state->_calib_imu_GYROtoIMU->value()(2), state->_calib_imu_GYROtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_DEBUG("q_ACCtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_ACCtoIMU->value()(0), state->_calib_imu_ACCtoIMU->value()(1),
                state->_calib_imu_ACCtoIMU->value()(2), state->_calib_imu_ACCtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_DEBUG("Dw = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
                state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
                state->_calib_imu_dw->value()(5));
    PRINT_DEBUG("Da = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
                state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
                state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_DEBUG("Dw = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
                state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
                state->_calib_imu_dw->value()(5));
    PRINT_DEBUG("Da = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
                state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
                state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.do_calib_imu_g_sensitivity) {
    PRINT_DEBUG("Tg = | %.4f,%.4f,%.4f |  %.4f,%.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_tg->value()(0),
                state->_calib_imu_tg->value()(1), state->_calib_imu_tg->value()(2), state->_calib_imu_tg->value()(3),
                state->_calib_imu_tg->value()(4), state->_calib_imu_tg->value()(5), state->_calib_imu_tg->value()(6),
                state->_calib_imu_tg->value()(7), state->_calib_imu_tg->value()(8));
  }
}
