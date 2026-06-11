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
#include <sstream>

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

#include <algorithm>

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
  if (mode == "global_yaw_oc_projection" || mode == "global_yaw_schmidt")
    return StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_PROJECTION;
  if (mode == "current_only_scale")
    return StateHelper::VisualYawUpdateMode::CURRENT_ONLY_SCALE;
  if (mode == "hard_gyro_yaw" || mode == "gyro_yaw_only")
    return StateHelper::VisualYawUpdateMode::HARD_GYRO_YAW;
  if (mode == "a_strict_yaw_dx0" || mode == "strict_yaw_dx0")
    return StateHelper::VisualYawUpdateMode::A_STRICT_YAW_DX0;
  if (mode == "visual_yaw_schmidt_current_gauge")
    return StateHelper::VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_CURRENT_GAUGE;
  if (mode == "visual_yaw_h_projection_current")
    return StateHelper::VisualYawUpdateMode::VISUAL_YAW_H_PROJECTION_CURRENT;
  if (mode == "visual_yaw_schmidt_guarded")
    return StateHelper::VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_GUARDED;
  if (mode == "visual_yaw_schmidt_fej_gauge")
    return StateHelper::VisualYawUpdateMode::VISUAL_YAW_SCHMIDT_FEJ_GAUGE;
  if (mode == "global_yaw_oc_fej_projection")
    return StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
  if (mode == "constrained_yaw_nullspace") {
    // RETIRED 2026-06-06: smoke FAILED — P_n exhausted after ~40 calls; covariance indefinite.
    // The CLI alias already exits before reaching here, but guard against direct string use.
    PRINT_WARNING(YELLOW "[VIO-YAW] RETIRED: constrained_yaw_nullspace smoke FAILED (P_n exhausted); "
                         "using original. See constrained_yaw_nullspace_smoke_report.md\n" RESET);
    return StateHelper::VisualYawUpdateMode::ORIGINAL;
  }
  if (mode == "dso_increment_ortho") {
    // RETIRED 2026-06-06: v1 K-projection failed fly3 smoke (P_{z,x} corrupted, t=450s).
    // The CLI alias already exits before reaching here, but guard against direct string use.
    PRINT_WARNING(YELLOW "[VIO-YAW] RETIRED: dso_increment_ortho v1 (K-projection smoke fail); "
                         "redirecting to global_yaw_oc_fej_projection (mode 10).\n" RESET);
    return StateHelper::VisualYawUpdateMode::GLOBAL_YAW_OC_FEJ_PROJECTION;
  }
  if (mode == "vins_numeric_nullspace") {
    // RETIRED 2026-06-06: smoke showed n_zeroed=0; EVD on S ≠ VINS marginalization.
    PRINT_WARNING(YELLOW "[VIO-YAW] RETIRED: vins_numeric_nullspace (n_zeroed=0 smoke fail); "
                         "using original.\n" RESET);
    return StateHelper::VisualYawUpdateMode::ORIGINAL;
  }
  // Pre-chi2 VOP modes: EKFUpdate runs ORIGINAL (projection already applied by VOP)
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

// Apply VOP (and legacy yaw mode) to all visual updaters.
void apply_yaw_control_to_updaters(
    const std::string &mode, double scale, double alpha,
    std::shared_ptr<UpdaterMSCKF> &msckf,
    std::shared_ptr<UpdaterSLAM> &slam) {
  auto sh_mode = visual_yaw_mode_from_string(mode);
  auto vop = make_vop(mode);
  // For pre-chi2 VOP modes, EKFUpdate runs ORIGINAL (OC already applied pre-chi2).
  // For visual_yaw_schmidt_current_gauge, EKFUpdate dispatches to the Schmidt path —
  //   no VOP is used, and the mode is passed through directly.
  // For all other (legacy) modes, sh_mode carries the projection.
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
// [中文] 构造函数
//  按顺序搭建整个 VIO 系统:
//    1. 加载与打印配置
//    2. 创建 State 并将调用者传入的外参/内参/IMU内参写入
//    3. 根据 params.use_klt / params.use_aruco 创建前端跟踪器
//    4. 创建 Propagator（传播）、InertialInitializer (初始化),
//       各类更新器 (MSCKF / SLAM / ZUPT)
//  注意构造完成后系统处于"等待初始化"状态, State 的核心变量 (q, p, v, b) 未被赋值,
//  真正的初值在 try_to_initialize() 成功后才写入。
// =============================================================================
VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
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
  if (!params.schmidt_yaw_diag_path.empty()) {
    set_schmidt_yaw_diag_path(params.schmidt_yaw_diag_path);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  // [中文] 初始阶段将总特征数均匀地分配到每个相机上;
  //        初始化完成后, State 内的 num_pts 会被调大为 params.num_pts (正常跟踪阶段)。
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

// =============================================================================
// [中文] IMU 消息入口
//  - 未初始化时: oldest_time = 当前时刻减去初始化窗口, 保留窗内所有 IMU
//  - 已初始化时: oldest_time = 最老克隆的时间（margtimestep）,
//                    小于它的 IMU 已经被用掉, 可安全丢弃
//  对下游计算的影响:
//    - propagator 用其基本保证可拿到下次 propagate_and_clone 所需 IMU
//    - initializer / ZUPT 在各自的窗口外丢弃旧测量
// =============================================================================
void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    // [中文] -0.10 留了 100ms 宽容时间, 避免因相机/IMU 不同步把即将使用的 IMU 误删
    oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  propagator->feed_imu(message, oldest_time);

  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
  }

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}

void VioManager::feed_measurement_gps_altitude(double timestamp, double altitude_z, double sigma,
                                               double chi2_gate, bool use_schmidt,
                                               bool also_update_vz, bool range_mode) {

  // 需要先初始化完成
  if (!is_initialized_vio) {
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

  // --- Periodic STAT summary (fires for ALL calls: accept/reject/skip) ---
  if (t_state - gps_alt_stats_.last_summary_time > 30.0) {
    gps_alt_stats_.last_summary_time = t_state;
    size_t n_evals = gps_alt_stats_.n_accepted + gps_alt_stats_.n_rejected;
    double rate = n_evals > 0 ? 100.0 * gps_alt_stats_.n_accepted / n_evals : 0;
    PRINT_INFO(CYAN "[GPS-ALT-STAT] t=%.1f calls=%zu acc=%zu rej=%zu(dxy=%zu kxy=%zu bias=%zu) skip=%zu rate=%.1f%% "
               "K_pz_mu=%.5f |K_xy|_mu=%.5f |dxy|_mu=%.4f |dtheta|_mu=%.4f |dba|_mu=%.5f |dbg|_mu=%.5f "
               "P_zz_mu=%.4f |res|_mu=%.2f dpz_mu=%.3f\n" RESET,
               t_state, gps_alt_stats_.n_called, gps_alt_stats_.n_accepted, gps_alt_stats_.n_rejected,
               gps_alt_stats_.n_rejected_dxy, gps_alt_stats_.n_rejected_kxy, gps_alt_stats_.n_rejected_bias,
               gps_alt_stats_.n_skipped, rate,
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
  // 调用方已经在最近的 cam 帧触发, 所以差值通常 < 100ms.
  if (t_state < timestamp - 0.2 || t_state > timestamp + 0.2) {
    gps_alt_stats_.n_skipped++;
    PRINT_DEBUG(YELLOW "[GPS-ALT-EVAL] status=SKIP t_cam=%.3f t_gps=%.3f dt=%+.3fs (state=%.3f meas=%.3f)\n" RESET,
                t_state, timestamp, dt_gps, t_state, timestamp);
    return;
  }

  Eigen::Vector3d p_IinG = state->_imu->pos();
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  // body z-axis expressed in world = R_ItoG * e_z = 3rd row of R_GtoI as column
  const double r22 = R_GtoI(2, 2);   // cos(tilt) for a downward LRF pointing along -body_z

  // H_order: active state variables included in the Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Eigen::MatrixXd H;
  double z_pred;

  if (range_mode) {
    // [C-mode] Range model: measurement = body-frame downward range to flat ground.
    //   h(x) = (p_z - z_ground) / R_GtoI(2,2)
    //   Bootstrap z_ground once from the first accepted sample so that any
    //   VIO / altimeter origin mismatch is absorbed into z_ground (and NOT
    //   into a large residual that would corrupt ba through cross-cov).
    // For a non-tilted drone R_GtoI(2,2)≈1 and this reduces to (p_z - z0).
    if (!gps_alt_bootstrapped_) {
      // bootstrap: make predicted range match measured range at first sample
      gps_alt_z_ground_ = p_IinG(2) - altitude_z * r22;
      gps_alt_bootstrapped_ = true;
      PRINT_INFO(CYAN "[GPS-ALT-C]: bootstrap z_ground=%.3fm (p_z=%.3f, meas=%.3f, r22=%.3f)\n" RESET,
                 gps_alt_z_ground_, p_IinG(2), altitude_z, r22);
    }

    if (std::abs(r22) < 0.3) {
      // severely tilted (>72deg) - skip, model unreliable
      PRINT_WARNING(YELLOW "[GPS-ALT-C]: skip, r22=%.3f too small (drone tilted)\n" RESET, r22);
      return;
    }

    z_pred = (p_IinG(2) - gps_alt_z_ground_) / r22;

    // Jacobian: d h / d p_z = 1/r22
    //           d h / d theta_imu (world-frame err.state, left-mult on R_GtoI):
    //     R_GtoI'(2,2) = R_GtoI(2,2) + [R_GtoI(1,2)*dtheta_x - R_GtoI(0,2)*dtheta_y]
    //     so d r22 / d theta = [R(1,2), -R(0,2), 0]
    //     d h / d theta = -(p_z - z0)/r22^2 * [R(1,2), -R(0,2), 0]
    Hx_order.push_back(state->_imu->q());   // 3 (orientation)
    Hx_order.push_back(state->_imu->p());   // 3 (position)
    if (also_update_vz) {
      Hx_order.push_back(state->_imu->v()); // 3 (velocity)
    }
    int ncol = 3 * Hx_order.size();
    H = Eigen::MatrixXd::Zero(1, ncol);
    double coef = -(p_IinG(2) - gps_alt_z_ground_) / (r22 * r22);
    H(0, 0) = coef * R_GtoI(1, 2);   // d/d theta_x
    H(0, 1) = coef * (-R_GtoI(0, 2)); // d/d theta_y
    H(0, 2) = 0.0;                    // d/d theta_z (yaw around g has no effect on r22)
    H(0, 3 + 2) = 1.0 / r22;          // d/d p_z
    // velocity columns (if included) default 0
  } else if (gps_alt_arch_g_ && state->_h_offset && state->_h_offset->id() >= 0) {
    // Architecture G (new_height_aid_research.md §5.3):
    //   GPS_z = p_z + h_offset    (h_offset = GPS-VIO altitude reference bias)
    //   H = [0, 0, 1,  1]   in {p_x, p_y, p_z, h_offset}
    //
    // Both p_z and h_offset receive consistent corrections. No masking.
    // h_offset absorbs measurement innovation; cross-covariances evolve correctly.

    // Inject random-walk process noise into h_offset (models slow drift of bias)
    if (gps_alt_h_offset_last_t_ > 0.0) {
      double dt_noise = t_state - gps_alt_h_offset_last_t_;
      if (dt_noise > 0.0 && dt_noise < 10.0) {
        double walk_var = std::pow(params.state_options.gps_h_offset_walk_sigma, 2) * dt_noise;
        StateHelper::inject_h_offset_noise(state, walk_var);
      }
    }
    gps_alt_h_offset_last_t_ = t_state;

    double h_offset_val = state->_h_offset->value()(0);
    z_pred = p_IinG(2) + h_offset_val;

    Hx_order.push_back(state->_imu->p());
    Hx_order.push_back(state->_h_offset);
    if (also_update_vz) {
      // Insert velocity before h_offset so h_offset is last
      Hx_order.clear();
      Hx_order.push_back(state->_imu->p());
      Hx_order.push_back(state->_imu->v());
      Hx_order.push_back(state->_h_offset);
      H = Eigen::MatrixXd::Zero(1, 7); // p(3) + v(3) + h_offset(1)
      H(0, 2) = 1.0;   // p_z
      H(0, 6) = 1.0;   // h_offset
    } else {
      H = Eigen::MatrixXd::Zero(1, 4); // p(3) + h_offset(1)
      H(0, 2) = 1.0;   // p_z
      H(0, 3) = 1.0;   // h_offset
    }
    // pz_idx stays 2 (within H_order: p is first, p_z is column 2)
  } else {
    // Legacy measurement model: h(x) = p_IinG[2]  (altitude directly in world)
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

  int pz_idx = range_mode ? (3 + 2) : 2;

  // --- P_zz floor BEFORE update: prevent K_pz from collapsing to ~0 ---
  // When P_pz << R, K_pz = P_pz/(P_pz+R) ≈ P_pz/R ≈ 0, making GPS useless.
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

  // --- Innovation gate: reject updates where |res| exceeds threshold ---
  // Large residuals indicate a systematic GPS offset that would pull the state
  // incorrectly.  Skipping the update entirely is safer than clamping.
  const double raw_res = res_scalar;
  if (gps_alt_max_res_gate_ < 1e8 && std::fabs(res_scalar) > gps_alt_max_res_gate_) {
    gps_alt_stats_.n_rejected++;
    PRINT_INFO(YELLOW "[GPS-ALT] REJECT t=%.3f |res|=%.1f > %.1f m — skipping update\n" RESET,
               t_state, std::fabs(raw_res), gps_alt_max_res_gate_);
    return;
  }

  Eigen::VectorXd res = Eigen::VectorXd::Zero(1);
  res(0) = res_scalar;

  // GPS 高度是绝对基准 — VIO 飘了的时候应该信 GPS 而不是拒掉它.
  // 保留 chi2 用于诊断日志, 但不再拒绝更新. Schmidt filter 已保护 IMU bias.
  // Log large residuals for diagnostics (no rejection)
  if (chi2 > chi2_gate) {
    PRINT_INFO(YELLOW "[GPS-ALT-DIAG] large-res t=%.3f res=%.2f chi2=%.1f P_zz=%.4f — accepting anyway\n" RESET,
               t_state, raw_res, chi2, P_pz);
  }

  // --- Marginal Kalman gain for position block ---
  // K = P * H^T / S.  H has a 1 at p_z (col 2) and 0 elsewhere,
  // so K_p = P[:, 2] / S = [P_xz, P_yz, P_zz]^T / S.
  Eigen::VectorXd K_pz_vec = (P * H.transpose()) / S;  // n×1
  double K_px = (K_pz_vec.size() > 0) ? K_pz_vec(0) : 0.0;
  double K_py = (K_pz_vec.size() > 1) ? K_pz_vec(1) : 0.0;
  double K_pz_gain = K_pz_vec(pz_idx);
  double K_xy_norm = std::sqrt(K_px * K_px + K_py * K_py);

  // Predicted position correction (error-state): dx = K * residual
  double pred_dx = K_px * res_scalar;
  double pred_dy = K_py * res_scalar;
  double pred_dz = K_pz_gain * res_scalar;
  double pred_dxy_norm = std::sqrt(pred_dx * pred_dx + pred_dy * pred_dy);

  // --- Full-state K for orientation/bias diagnostics (only if a guard is active) ---
  double pred_dtheta_norm = -1.0;
  double pred_dba_norm = -1.0;
  double pred_dbg_norm = -1.0;
  bool guards_active = (gps_alt_guard_dxy_max_ > 0.0 || gps_alt_guard_kxy_ratio_max_ > 0.0 ||
                        gps_alt_guard_dbias_max_ > 0.0);
  if (guards_active) {
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

  // Guard 1: predicted XY position correction too large
  if (gps_alt_guard_dxy_max_ > 0.0 && pred_dxy_norm > gps_alt_guard_dxy_max_) {
    decision = "REJECT_BY_DXY";
  }
  // Guard 2: K_xy / |K_pz| ratio too large → too much XY leakage per unit Z correction
  if (decision == "APPLY" && gps_alt_guard_kxy_ratio_max_ > 0.0 &&
      K_pz_gain != 0.0 && K_xy_norm / std::fabs(K_pz_gain) > gps_alt_guard_kxy_ratio_max_) {
    decision = "REJECT_BY_GAIN_RATIO";
  }
  // Guard 3: predicted bias correction too large
  if (decision == "APPLY" && gps_alt_guard_dbias_max_ > 0.0 &&
      std::max(pred_dba_norm, pred_dbg_norm) > gps_alt_guard_dbias_max_) {
    decision = "REJECT_BY_BIAS";
  }

  if (decision != "APPLY") {
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
               "|dbias|=%.5f |dtheta|=%.5f — skipping update\n" RESET,
               decision.c_str(), t_state, raw_res, pred_dxy_norm,
               (K_pz_gain != 0.0 ? K_xy_norm / std::fabs(K_pz_gain) : -1.0),
               std::max(pred_dba_norm, pred_dbg_norm), pred_dtheta_norm);
    return;
  }

  // === capture state BEFORE update for delta computation ===
  Eigen::Vector3d ba_pre = state->_imu->bias_a();
  Eigen::Vector3d bg_pre = state->_imu->bias_g();
  Eigen::Vector3d v_pre = state->_imu->vel();
  Eigen::Vector3d p_pre = state->_imu->pos();
  Eigen::Matrix3d R_GtoI_pre = state->_imu->Rot();
  double z_before = p_pre(2);
  const double yaw_before_gps_alt = current_imu_yaw_deg();
  StateHelper::reset_last_yaw_dx_projection_diag();

  if (gps_alt_joseph_update_) {
    // PX4-style masked Joseph update (Brink 2017 / PX4 fuseHaglRng).
    // Only p_z state DOF and p_z row/column of P are updated; all other states
    // and cross-covariances are unchanged.  All guard logic above still applies.
    StateHelper::EKFUpdateJosephMasked(state, Hx_order, H, res, R,
                                       state->_imu->p(), 2);
    decision = "JOSEPH_MASKED";
  } else if (gps_alt_zonly_update_) {
    StateHelper::EKFUpdateZOnly(state, Hx_order, H, res, R);
    decision = "ZONLY";
  } else if (use_schmidt) {
    StateHelper::EKFUpdateSchmidt(state, Hx_order, H, res, R);
  } else {
    const auto yaw_mode = visual_yaw_mode_from_string(params.vio_yaw_update_mode);
    if (yaw_mode == StateHelper::VisualYawUpdateMode::A_STRICT_YAW_DX0) {
      StateHelper::EKFUpdate(state, Hx_order, H, res, R, yaw_mode,
                             params.vio_yaw_update_scale,
                             params.vio_global_yaw_oc_alpha);
    } else {
      StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    }
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
  double z_after = p_post(2);
  double res_after = altitude_z - z_after;
  double tilt_pre = std::acos(std::min(1.0, std::max(-1.0, R_GtoI_pre(2, 2)))) * 180.0 / M_PI;
  double tilt_post = std::acos(std::min(1.0, std::max(-1.0, R_GtoI_post(2, 2)))) * 180.0 / M_PI;
  const double yaw_after_gps_alt = current_imu_yaw_deg();
  log_vio_yaw_update(state->_timestamp, "GPS_ALTITUDE",
                     yaw_before_gps_alt, yaw_after_gps_alt,
                     wrap_degrees(yaw_after_gps_alt - yaw_before_gps_alt),
                     state->_imu->bias_g()(2), 1, chi2, 1, 0,
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
  gps_alt_last_.zonly = gps_alt_zonly_update_;
  gps_alt_last_.clipped = false;
  gps_alt_last_.decision = decision;

  // --- statistics ---
  gps_alt_stats_.n_accepted++;
  gps_alt_stats_.sum_K_pz += K_pz_gain;
  gps_alt_stats_.sum_K_xy_norm += K_xy_norm;
  gps_alt_stats_.sum_dxy_norm += pred_dxy_norm;
  if (pred_dtheta_norm >= 0) gps_alt_stats_.sum_dtheta_norm += pred_dtheta_norm;
  if (pred_dba_norm >= 0) gps_alt_stats_.sum_dba_norm += pred_dba_norm;
  if (pred_dbg_norm >= 0) gps_alt_stats_.sum_dbg_norm += pred_dbg_norm;
  gps_alt_stats_.sum_P_zz += P_pz;
  gps_alt_stats_.sum_abs_res += std::abs(raw_res);
  gps_alt_stats_.sum_abs_dpz += std::abs(dp(2));

  // --- unified EVAL log with guard diagnostics ---
  PRINT_INFO(CYAN "[GPS-ALT-EVAL] status=%s t_cam=%.3f dt=%+.3fs meas=%.2f z_pred=%.2f z_after=%.2f "
             "res=%+.2f chi2=%.1f P_zz=%.4f K_pz=%.5f |K_xy|=%.5f |dxy|_pred=%.4f "
             "|dtheta|_pred=%.4f |dba|_pred=%.5f |dbg|_pred=%.5f dp_z_actual=%+.3f %s%s%s%s\n" RESET,
             gps_alt_zonly_update_ ? "ZONLY" : "ACC",
             t_state, dt_gps, altitude_z, z_before, z_after,
             raw_res, chi2, P_pz, K_pz_gain, K_xy_norm, pred_dxy_norm,
             pred_dtheta_norm, pred_dba_norm, pred_dbg_norm, dp(2),
             gps_alt_zonly_update_ ? " ZONLY" : "",
             use_schmidt ? " Schmidt" : "",
             also_update_vz ? " +vz" : "",
             P_pz_floor_applied > 0 ? " FLOOR" : "");

  // --- detailed DIAG: full state delta (keep for correctness verification) ---
  PRINT_INFO(MAGENTA "[GPS-ALT-DIAG] t=%.3f | p_pre=[%.2f %.2f %.2f] dp=[%+.3f %+.3f %+.3f] | v_pre=[%.2f %.2f %.2f] dv=[%+.3f %+.3f %+.3f] | tilt %.2f->%.2f deg | ba_pre=[%+.3f %+.3f %+.3f] dba=[%+.4f %+.4f %+.4f] | bg_pre=[%+.4f %+.4f %+.4f] dbg=[%+.5f %+.5f %+.5f]\n" RESET,
             timestamp,
             p_pre(0), p_pre(1), p_pre(2), dp(0), dp(1), dp(2),
             v_pre(0), v_pre(1), v_pre(2), dv(0), dv(1), dv(2),
             tilt_pre, tilt_post,
             ba_pre(0), ba_pre(1), ba_pre(2), dba(0), dba(1), dba(2),
             bg_pre(0), bg_pre(1), bg_pre(2), dbg(0), dbg(1), dbg(2));

}

void VioManager::feed_measurement_gps_altitude_relative(double timestamp, double altitude_z, double sigma,
                                                         double chi2_gate, bool use_schmidt,
                                                         bool also_update_vz) {

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

  feed_measurement_gps_altitude(timestamp, effective_alt, sigma, chi2_gate, use_schmidt,
                                also_update_vz, false);
}

bool VioManager::feed_measurement_gps_ground_plane(double timestamp, double z_gps,
                                                     double sigma_range, bool zonly) {

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
  // We compute effective_z = (z_gps - gps_ref) + vio_ref so the residual
  // reflects VIO drift relative to GPS, not the absolute frame offset.
  if (!gps_alt_rel_bootstrapped_) {
    gps_alt_rel_gps_ref_ = z_gps;
    gps_alt_rel_vio_ref_ = state->_imu->pos()(2);
    gps_alt_rel_bootstrapped_ = true;
    PRINT_INFO(CYAN "[GPLANE-RNG] bootstrap: gps_ref=%.3f vio_ref=%.3f\n" RESET,
               gps_alt_rel_gps_ref_, gps_alt_rel_vio_ref_);
  }
  double effective_z = z_gps - gps_alt_rel_gps_ref_ + gps_alt_rel_vio_ref_;

  // Lazy construction of the ground-plane range updater
  if (updaterGPlaneRange == nullptr) {
    updaterGPlaneRange = std::make_shared<UpdaterGroundPlaneRange>(
        sigma_range,           // sigma_range
        0.3,                   // min_cos_tilt (skip if tilt > 72°)
        10000.0,               // chi2_gate (permissive)
        zonly,                 // use_zonly
        gps_alt_min_pzz_,      // P_zz floor (share with GPS altitude setting)
        gps_alt_max_res_gate_  // innovation gate
    );
    updaterGPlaneRange->set_visual_yaw_update_control(
        visual_yaw_mode_from_string(params.vio_yaw_update_mode),
        params.vio_yaw_update_scale, params.vio_global_yaw_oc_alpha);
    PRINT_INFO(GREEN "[GPLANE-RNG] created: sigma=%.2f zonly=%d\n" RESET,
               sigma_range, (int)zonly);
  }

  const double yaw_before = current_imu_yaw_deg();
  StateHelper::reset_last_yaw_dx_projection_diag();
  const bool applied = updaterGPlaneRange->try_update(state, state->_timestamp, effective_z, timestamp);
  if (applied) {
    const double yaw_after = current_imu_yaw_deg();
    log_vio_yaw_update(state->_timestamp, "GPLANE_RANGE", yaw_before, yaw_after,
                       wrap_degrees(yaw_after - yaw_before), state->_imu->bias_g()(2),
                       1, -1.0, 1, 0, trackFEATS ? get_feature_database_size() : -1);
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
  out.track_len_sum_acc = s.track_len_sum_acc;
  out.track_len_max_acc = s.track_len_max_acc;
  out.track_len_sum_rej = s.track_len_sum_rej;
  out.track_len_max_rej = s.track_len_max_rej;
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
                                  params.vio_yaw_update_mode == "a_strict_yaw_dx0" ||
                                  params.vio_yaw_update_mode == "strict_yaw_dx0" ||
                                  params.vio_yaw_update_mode == "visual_yaw_schmidt_current_gauge" ||
                                  params.vio_yaw_update_mode == "visual_yaw_schmidt_fej_gauge" ||
                                  params.vio_yaw_update_mode == "visual_yaw_h_projection_current" ||
                                  params.vio_yaw_update_mode == "visual_yaw_schmidt_guarded" ||
                                  params.vio_yaw_update_mode == "constrained_yaw_nullspace" ||
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

void VioManager::set_schmidt_yaw_diag_path(const std::string &path) {
  params.schmidt_yaw_diag_path = path;
  StateHelper::open_schmidt_yaw_diag_csv(path);
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

// ── Visual update guard helpers ────────────────────────────────────────────
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
      // Single timestamp → ±5ms window
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

  // ── Guard: skip window / reject file ─────────────────────────────────────
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
  StateHelper::reset_last_yaw_dx_projection_diag();
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

  const double yaw_after = current_imu_yaw_deg();
  const double delta_yaw = wrap_degrees(yaw_after - yaw_before);
  const int tracking_count = trackFEATS ? get_feature_database_size() : -1;
  const int rejected = (accepted >= 0 && num_features >= accepted) ? (num_features - accepted) : -1;
  log_vio_yaw_update(state->_timestamp, update_type, yaw_before, yaw_after, delta_yaw,
                     state->_imu->bias_g()(2), num_features, chi2, accepted, rejected,
                     tracking_count);
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
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] ====== GPS Altitude Fusion Summary ======\n" RESET);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] time span: %.1f s (%.1f - %.1f)\n" RESET,
             span, s.first_eval_time, s.last_eval_time);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] calls=%zu  acc=%zu  rej=%zu (dxy=%zu kxy=%zu bias=%zu)  skip=%zu  rate=%.1f%%\n" RESET,
             s.n_called, s.n_accepted, s.n_rejected,
             s.n_rejected_dxy, s.n_rejected_kxy, s.n_rejected_bias,
             s.n_skipped, rate);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] K_pz mean=%.5f  |K_xy| mean=%.5f  |dxy| mean=%.4f m\n" RESET,
             n_evals > 0 ? s.sum_K_pz / n_evals : 0.0,
             n_evals > 0 ? s.sum_K_xy_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dxy_norm / n_evals : 0.0);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] |dtheta| mean=%.4f  |dba| mean=%.5f  |dbg| mean=%.5f\n" RESET,
             n_evals > 0 ? s.sum_dtheta_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dba_norm / n_evals : 0.0,
             n_evals > 0 ? s.sum_dbg_norm / n_evals : 0.0);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] P_zz mean=%.4f  |res| mean=%.2f m  |dp_z| mean=%.3f m\n" RESET,
             n_evals > 0 ? s.sum_P_zz / n_evals : 0.0,
             n_evals > 0 ? s.sum_abs_res / n_evals : 0.0,
             s.n_accepted > 0 ? s.sum_abs_dpz / s.n_accepted : 0.0);
  PRINT_INFO(GREEN "[GPS-ALT-FINAL] ==========================================\n" RESET);
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
// [中文] track_image_and_update
//  相机帧的总分流器。两条主要分支:
//    (A) 已初始化 -> 前端跟踪 -> 尝试 ZUPT -> 否则常规更新 (do_feature_propagate_update)
//    (B) 未初始化 -> 前端跟踪 -> 调用 try_to_initialize
//  注意两条分支都会先走前端跟踪, 这样初始化期间 FeatureDatabase 也能累积有视觉观测,
//  供 DynamicInitializer 使用。
// =============================================================================
void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  // [中文] 可选对每个相机图像+掩码做 1/2 下采样, 播冟在高分辨率设备上加速
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
  //   current camera orientation, so KLT sees only translational parallax — not the
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
  // FIRST ACTIVATION: gravity_warp_R_ref_ is not yet populated → skip warp, save
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
      // Frame-to-frame rotation: maps a ray in last-cam frame → current-cam frame.
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
  // [中文] 视觉前端: KLT / Descriptor / SIM. 内部会更新 FeatureDatabase
  trackFEATS->feed_new_camera(message);

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
  // [中文] 进入完整的 EKF 流程 (传播 + 克隆 + MSCKF/SLAM/ZUPT 更新 + 边缘化)
  if (landing_update_halted_) {
    // Propagate-only path: still advance time and clone window, no update.
    if (state->_timestamp < message.timestamp) {
      propagator->propagate_and_clone(state, message.timestamp);
      // Keep clone window bounded; do_feature_propagate_update normally
      // does this at the end.
      StateHelper::marginalize_old_clone(state);
    }
    PRINT_INFO(YELLOW "[LANDING-GATE] frame %.3f: skipped MSCKF/SLAM update (alt=%.2fm)\n" RESET,
               message.timestamp, state->_imu->pos()(2));
  } else {
    do_feature_propagate_update(message);
  }
}

// =============================================================================
// [中文] do_feature_propagate_update
//  单帧完整的滤波主循环 (见 docs-cn/diagrams/03_vio_manager_flow.png):
//    Step 1  propagate_and_clone : IMU 预测 + 增广一份新克隆
//    Step 2  拉特征并分类:
//              feats_lost       = 在当前时刻没有观测 -> MSCKF
//              feats_marg       = 跳要随最老克隆一起消失 -> MSCKF/SLAM/DELAYED
//              feats_maxtracks  = 轨迹太长的非 SLAM 特征   -> MSCKF
//              feats_slam_UPDATE  = 已在状态里的 SLAM    -> SLAM update
//              feats_slam_DELAYED = 新晔 SLAM 候选         -> SLAM delayed_init
//    Step 3  调用三个更新器做 EKF
//    Step 4  retriangulate + marginalize_old_clone 维持滑窗大小
// =============================================================================
void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================
  // [中文] Step 1。传播与克隆紧耦合: propagate_and_clone 把旧状态传到 message.timestamp,
  //        同时在 _clones_IMU 里插入一个对当前位姿的克隆。

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
  // [中文] 三角化需要至少 2 个视角, 数值稳定则要求 ≥ 5 个克隆才开始走更新
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

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================
  // [中文] Step 2。从 FeatureDatabase 挑选本帧需要参与更新的特征, 并按"去处"分类:
  //   - feats_lost: 本帧没有观测的特征 (跟丢), 只能用作 MSCKF
  //   - feats_marg: 含有最老克隆观测的特征, 若不立即用掉, 协方差块会被 marg_old_clone 删
  //   - feats_maxtracks: 轨迹已经达到 max_clone_size 的非 SLAM 特征, 晋升 SLAM 的候选
  //   - feats_slam: 已经是 SLAM 特征, 由 Aruco 或后续 delayed_init 贡献

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
  // [中文] Step 3。按顺序调用三类更新器:
  //   updaterMSCKF->update        :  短轨迹特征, 左零空间投影 + 卡方 + QR 压缩 + EKF
  //   updaterSLAM->update         :  已在状态里的 SLAM 特征 (可分批做 sequential update)
  //   updaterSLAM->delayed_init   :  将 feats_slam_DELAYED 加入状态并初始化其协方差

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

  // Stage B — ground-plane feature update (optional).  Runs BEFORE MSCKF
  // so MSCKF hasn't yet marked features for deletion; we only need to look
  // at the database, we don't consume features.
  if (updaterGPlaneFeature != nullptr && updaterGPlaneRange != nullptr &&
      updaterGPlaneRange->bootstrapped()) {
    double z_g = updaterGPlaneRange->z_ground();
    const double yaw_before_gplane = current_imu_yaw_deg();
    StateHelper::reset_last_yaw_dx_projection_diag();
    bool gplane_fired = updaterGPlaneFeature->try_update(
        state, trackFEATS->get_feature_database(), message.timestamp, z_g);
    if (gplane_fired) {
      const double yaw_after_gplane = current_imu_yaw_deg();
      const auto &last = updaterGPlaneFeature->last_update();
      log_vio_yaw_update(state->_timestamp, "GPLANE_FEATURE",
                         yaw_before_gplane, yaw_after_gplane,
                         wrap_degrees(yaw_after_gplane - yaw_before_gplane),
                         state->_imu->bias_g()(2), last.n_used, -1.0,
                         last.n_used, 0, get_feature_database_size());
    }
  }

  // Stage B v1 — two-clone H, dry-run + FD check (optional, mutually
  // exclusive with v0 in practice).  Same activation gate as v0: requires
  // Stage A bootstrap and Stage A delay (Stage A enforces the delay itself,
  // so once Stage A has fired, both feature updaters become eligible).
  if (updaterGPlaneFeatureV1 != nullptr && updaterGPlaneRange != nullptr &&
      updaterGPlaneRange->bootstrapped()) {
    double z_g = updaterGPlaneRange->z_ground();
    const double yaw_before_gplane_v1 = current_imu_yaw_deg();
    StateHelper::reset_last_yaw_dx_projection_diag();
    bool v1_fired = updaterGPlaneFeatureV1->try_update_candidates(
        state, featsup_MSCKF, message.timestamp, z_g);
    if (v1_fired) {
      const double yaw_after_gplane_v1 = current_imu_yaw_deg();
      const auto &last = updaterGPlaneFeatureV1->last_update();
      log_vio_yaw_update(state->_timestamp, "GPLANE_FEATURE_V1",
                         yaw_before_gplane_v1, yaw_after_gplane_v1,
                         wrap_degrees(yaw_after_gplane_v1 - yaw_before_gplane_v1),
                         state->_imu->bias_g()(2), last.n_used, -1.0,
                         last.n_used, 0, get_feature_database_size());
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
  propagator->invalidate_cache();
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
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
    const double yaw_before_slam = current_imu_yaw_deg();
    StateHelper::reset_last_yaw_dx_projection_diag();
    updaterSLAM->update(state, featsup_TEMP);
    const int n_slam_accepted = (int)featsup_TEMP.size();
    const double yaw_after_slam = current_imu_yaw_deg();
    const int n_slam_rejected = std::max(0, n_slam_features - n_slam_accepted);
    log_vio_yaw_update(state->_timestamp, "SLAM", yaw_before_slam, yaw_after_slam,
                       wrap_degrees(yaw_after_slam - yaw_before_slam),
                       state->_imu->bias_g()(2), n_slam_features, -1.0,
                       n_slam_accepted, n_slam_rejected, get_feature_database_size());
    feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
    propagator->invalidate_cache();
  }
  feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
  rT5 = boost::posix_time::microsec_clock::local_time();
  const int n_slam_delayed_features = (int)feats_slam_DELAYED.size();
  const double yaw_before_slam_delay = current_imu_yaw_deg();
  StateHelper::reset_last_yaw_dx_projection_diag();
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  const int n_slam_delayed_accepted = (int)feats_slam_DELAYED.size();
  const double yaw_after_slam_delay = current_imu_yaw_deg();
  const int n_slam_delayed_rejected = std::max(0, n_slam_delayed_features - n_slam_delayed_accepted);
  log_vio_yaw_update(state->_timestamp, "SLAM_DELAYED", yaw_before_slam_delay,
                     yaw_after_slam_delay,
                     wrap_degrees(yaw_after_slam_delay - yaw_before_slam_delay),
                     state->_imu->bias_g()(2), n_slam_delayed_features, -1.0,
                     n_slam_delayed_accepted, n_slam_delayed_rejected, get_feature_database_size());
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
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================
  // [中文] Step 4。数据库清理 + SLAM 锚点切换 + 最老克隆边缘化, 将滑窗重新限制回 max_clone_size。

  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
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
  PRINT_INFO("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n", state->_imu->quat()(0),
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

  PRINT_INFO("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_INFO("camera-imu timeoffset = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                 calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }

  // Debug for imu intrinsics
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("q_GYROtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_GYROtoIMU->value()(0), state->_calib_imu_GYROtoIMU->value()(1),
               state->_calib_imu_GYROtoIMU->value()(2), state->_calib_imu_GYROtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("q_ACCtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_ACCtoIMU->value()(0), state->_calib_imu_ACCtoIMU->value()(1),
               state->_calib_imu_ACCtoIMU->value()(2), state->_calib_imu_ACCtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("Dw = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("Dw = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.do_calib_imu_g_sensitivity) {
    PRINT_INFO("Tg = | %.4f,%.4f,%.4f |  %.4f,%.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_tg->value()(0),
               state->_calib_imu_tg->value()(1), state->_calib_imu_tg->value()(2), state->_calib_imu_tg->value()(3),
               state->_calib_imu_tg->value()(4), state->_calib_imu_tg->value()(5), state->_calib_imu_tg->value()(6),
               state->_calib_imu_tg->value()(7), state->_calib_imu_tg->value()(8));
  }
}
