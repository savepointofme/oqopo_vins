/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include "UpdaterGroundPlaneRange.h"

#include <cmath>

#include "state/State.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"

using namespace ov_type;
using namespace ov_msckf;

UpdaterGroundPlaneRange::UpdaterGroundPlaneRange(double sigma, double min_cos_tilt,
                                                 double chi2_gate, double min_pzz,
                                                 double max_res_gate,
                                                 HeightMeasurementType type)
    : sigma_(sigma), min_cos_tilt_(min_cos_tilt), chi2_gate_(chi2_gate),
      min_pzz_(min_pzz), max_res_gate_(max_res_gate), type_(type) {}

void UpdaterGroundPlaneRange::reset() {
  bootstrapped_ = false;
  z_ground_ = 0.0;
  last_ = LastUpdate();
  stats_ = Stats();
}

bool UpdaterGroundPlaneRange::try_update(std::shared_ptr<State> state,
                                         double t_state, double measurement,
                                         double t_meas) {
  stats_.n_called++;
  if (stats_.first_t < 0.0)
    stats_.first_t = t_state;
  stats_.last_t = t_state;

  const double dt = t_state - t_meas;
  if (t_state < t_meas - 0.2 || t_state > t_meas + 0.2) {
    stats_.n_skipped++;
    return false;
  }

  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  Eigen::Vector3d p_IinG = state->_imu->pos();
  const double r22 = R_GtoI(2, 2);

  if (type_ == HeightMeasurementType::RANGEFINDER && std::abs(r22) < min_cos_tilt_) {
    PRINT_DEBUG(YELLOW "[GPLANE-HEIGHT] skip rangefinder: cos_tilt=%.3f < %.3f\n" RESET,
                r22, min_cos_tilt_);
    stats_.n_skipped++;
    return false;
  }

  if (!bootstrapped_) {
    if (type_ == HeightMeasurementType::RANGEFINDER) {
      z_ground_ = p_IinG(2) - measurement * r22;
    } else {
      z_ground_ = p_IinG(2) - measurement;
    }
    bootstrapped_ = true;
    PRINT_INFO(CYAN "[GPLANE-HEIGHT] bootstrap type=%s z_ground=%.3f p_z=%.3f meas=%.3f cos=%.3f\n" RESET,
               type_ == HeightMeasurementType::RANGEFINDER ? "rangefinder" : "gps",
               z_ground_, p_IinG(2), measurement, r22);
  }

  std::vector<std::shared_ptr<Type>> Hx_order;
  Eigen::MatrixXd H;
  double pred = 0.0;
  int pz_idx = 0;

  if (type_ == HeightMeasurementType::RANGEFINDER) {
    pred = (p_IinG(2) - z_ground_) / r22;
    Hx_order.push_back(state->_imu->q());
    Hx_order.push_back(state->_imu->p());
    H = Eigen::MatrixXd::Zero(1, 6);
    const double coef = -(p_IinG(2) - z_ground_) / (r22 * r22);
    H(0, 0) = coef * R_GtoI(1, 2);
    H(0, 1) = coef * (-R_GtoI(0, 2));
    H(0, 5) = 1.0 / r22;
    pz_idx = 5;
  } else {
    pred = p_IinG(2);
    Hx_order.push_back(state->_imu->p());
    H = Eigen::MatrixXd::Zero(1, 3);
    H(0, 2) = 1.0;
    pz_idx = 2;
  }

  const double res_scalar = measurement - pred;
  if (max_res_gate_ < 1e8 && std::fabs(res_scalar) > max_res_gate_) {
    stats_.n_rejected++;
    PRINT_INFO(YELLOW "[GPLANE-HEIGHT] REJECT |res|=%.1f > %.1f m (t=%.3f)\n" RESET,
               std::fabs(res_scalar), max_res_gate_, t_state);
    return false;
  }

  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(1, 1);
  R_meas(0, 0) = sigma_ * sigma_;
  Eigen::VectorXd res = Eigen::VectorXd::Zero(1);
  res(0) = res_scalar;

  if (min_pzz_ > 0.0) {
    Eigen::MatrixXd P_pre = StateHelper::get_marginal_covariance(state, Hx_order);
    const double P_pz_pre = P_pre(pz_idx, pz_idx);
    if (P_pz_pre < min_pzz_)
      StateHelper::inject_pz_noise(state, min_pzz_ - P_pz_pre);
  }

  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  const double S = (H * P_marg * H.transpose())(0, 0) + R_meas(0, 0);
  Eigen::VectorXd K_vec = (P_marg * H.transpose()) / S;

  const double K_px = type_ == HeightMeasurementType::RANGEFINDER ? K_vec(3) : K_vec(0);
  const double K_py = type_ == HeightMeasurementType::RANGEFINDER ? K_vec(4) : K_vec(1);
  const double K_pz = type_ == HeightMeasurementType::RANGEFINDER ? K_vec(5) : K_vec(2);
  const double K_xy_norm = std::sqrt(K_px * K_px + K_py * K_py);
  const double pred_dx = K_px * res_scalar;
  const double pred_dy = K_py * res_scalar;
  const double pred_dz = K_pz * res_scalar;
  const double pred_dxy_norm = std::sqrt(pred_dx * pred_dx + pred_dy * pred_dy);

  double pred_dtheta_norm = 0.0;
  double pred_dba_norm = 0.0;
  double pred_dbg_norm = 0.0;
  {
    const Eigen::MatrixXd P_full = StateHelper::get_full_covariance(state);
    Eigen::VectorXd H_full = Eigen::VectorXd::Zero(P_full.rows());
    int col = 0;
    for (const auto &var : Hx_order) {
      H_full.segment(var->id(), var->size()) = H.block(0, col, 1, var->size()).transpose();
      col += var->size();
    }
    Eigen::VectorXd K_full = (P_full * H_full) / S;
    pred_dtheta_norm = K_full.segment(state->_imu->q()->id(), 3).norm() * std::fabs(res_scalar);
    pred_dba_norm = K_full.segment(state->_imu->ba()->id(), 3).norm() * std::fabs(res_scalar);
    pred_dbg_norm = K_full.segment(state->_imu->bg()->id(), 3).norm() * std::fabs(res_scalar);
  }

  const double chi2 = res_scalar * res_scalar / S;
  if (chi2 > chi2_gate_) {
    PRINT_DEBUG(YELLOW "[GPLANE-HEIGHT] large residual t=%.3f res=%.2f chi2=%.1f\n" RESET,
                t_state, res_scalar, chi2);
  }

  const Eigen::Vector3d p_pre = p_IinG;
  const Eigen::Vector3d ba_pre = state->_imu->bias_a();
  const Eigen::Vector3d bg_pre = state->_imu->bias_g();

  StateHelper::EKFUpdate(state, Hx_order, H, res, R_meas,
                         visual_yaw_update_mode_, visual_yaw_update_scale_,
                         visual_global_yaw_oc_alpha_);

  const Eigen::Vector3d p_post = state->_imu->pos();
  const Eigen::Vector3d dba = state->_imu->bias_a() - ba_pre;
  const Eigen::Vector3d dbg = state->_imu->bias_g() - bg_pre;

  double pzz_floor_applied = 0.0;
  if (min_pzz_ > 0.0) {
    Eigen::MatrixXd P_post_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    const double P_pz_post = P_post_marg(pz_idx, pz_idx);
    if (P_pz_post < min_pzz_) {
      StateHelper::inject_pz_noise(state, min_pzz_ - P_pz_post);
      pzz_floor_applied = min_pzz_ - P_pz_post;
    }
  }

  last_.t = t_state;
  last_.meas = measurement;
  last_.pred = pred;
  last_.residual = res_scalar;
  last_.cos_tilt = r22;
  last_.z_ground = z_ground_;
  last_.pz_before = p_pre(2);
  last_.pz_after = p_post(2);
  last_.K_pz = K_pz;
  last_.K_px = K_px;
  last_.K_py = K_py;
  last_.K_xy_norm = K_xy_norm;
  last_.pred_dx = pred_dx;
  last_.pred_dy = pred_dy;
  last_.pred_dz = pred_dz;
  last_.pred_dtheta_norm = pred_dtheta_norm;
  last_.pred_dba_norm = pred_dba_norm;
  last_.pred_dbg_norm = pred_dbg_norm;
  last_.S = S;
  last_.decision = type_ == HeightMeasurementType::RANGEFINDER ? "RANGEFINDER" : "GPS_ALTITUDE";

  stats_.n_accepted++;
  stats_.sum_K_pz += K_pz;
  stats_.sum_K_xy_norm += K_xy_norm;
  stats_.sum_dxy_norm += pred_dxy_norm;
  stats_.sum_dtheta_norm += pred_dtheta_norm;
  stats_.sum_dba_norm += pred_dba_norm;
  stats_.sum_dbg_norm += pred_dbg_norm;
  stats_.sum_abs_res += std::abs(res_scalar);
  stats_.sum_abs_dpz += std::abs(p_post(2) - p_pre(2));

  if (t_state - stats_.last_summary_t > 30.0) {
    stats_.last_summary_t = t_state;
    const size_t n = stats_.n_accepted + stats_.n_rejected;
    const double rate = n > 0 ? 100.0 * stats_.n_accepted / n : 0.0;
    PRINT_INFO(CYAN
               "[GPLANE-HEIGHT-STAT]\n"
               "+----------+-------------+--------+--------+--------+--------+\n"
               "| t_cam    | type        | calls  | acc    | rej    | skip   |\n"
               "| %8.1f | %11s | %6zu | %6zu | %6zu | %6zu |\n"
               "+----------+-------------+--------+--------+--------+--------+\n"
               "| rate   | z_ground | Kz_mu   | Kxy_mu  | dxy_mu | res_mu |\n"
               "| %5.1f%% | %8.2f | %7.4f | %7.4f | %6.3f | %6.2f |\n"
               "+--------+----------+---------+---------+--------+--------+\n" RESET,
               t_state,
               type_ == HeightMeasurementType::RANGEFINDER ? "rangefinder" : "gps",
               stats_.n_called, stats_.n_accepted, stats_.n_rejected,
               stats_.n_skipped, rate, z_ground_,
               n > 0 ? stats_.sum_K_pz / n : 0.0,
               n > 0 ? stats_.sum_K_xy_norm / n : 0.0,
               n > 0 ? stats_.sum_dxy_norm / n : 0.0,
               n > 0 ? stats_.sum_abs_res / n : 0.0);
  }

  PRINT_DEBUG(CYAN "[GPLANE-HEIGHT] %s t=%.3f dt=%+.3fs meas=%.2f pred=%.2f res=%+.2f "
              "chi2=%.1f cos=%.3f zg=%.2f pz=%.2f->%.2f Kz=%.4f |Kxy|=%.4f "
              "|dxy|=%.3f |dq|=%.4f |dba|=%.5f |dbg|=%.5f%s\n" RESET,
              last_.decision.c_str(), t_state, dt, measurement, pred, res_scalar,
              chi2, r22, z_ground_, p_pre(2), p_post(2), K_pz, K_xy_norm,
              pred_dxy_norm, pred_dtheta_norm, dba.norm(), dbg.norm(),
              pzz_floor_applied > 0.0 ? " FLOOR" : "");

  return true;
}

void UpdaterGroundPlaneRange::print_summary() const {
  const size_t n = stats_.n_accepted + stats_.n_rejected;
  const double span = stats_.last_t - stats_.first_t;
  const double rate = n > 0 ? 100.0 * stats_.n_accepted / n : 0.0;
  PRINT_INFO(GREEN
             "[GPLANE-HEIGHT-FINAL]\n"
             "+-------------+---------+----------+----------+\n"
             "| type        | sigma   | z_ground | span_s   |\n"
             "| %11s | %7.2f | %8.2f | %8.1f |\n"
             "+-------------+---------+----------+----------+\n"
             "| calls | accepted | rejected | skipped | rate   |\n"
             "| %5zu | %8zu | %8zu | %7zu | %5.1f%% |\n"
             "+-------+----------+----------+---------+--------+\n"
             "| Kz_mu  | Kxy_mu | dxy_mu | res_mu | dpz_mu |\n"
             "| %6.4f | %6.4f | %6.3f | %6.2f | %6.3f |\n"
             "+--------+--------+--------+--------+--------+\n" RESET,
             type_ == HeightMeasurementType::RANGEFINDER ? "rangefinder" : "gps",
             sigma_, z_ground_, span,
             stats_.n_called, stats_.n_accepted, stats_.n_rejected,
             stats_.n_skipped, rate,
             n > 0 ? stats_.sum_K_pz / n : 0.0,
             n > 0 ? stats_.sum_K_xy_norm / n : 0.0,
             n > 0 ? stats_.sum_dxy_norm / n : 0.0,
             n > 0 ? stats_.sum_abs_res / n : 0.0,
             stats_.n_accepted > 0 ? stats_.sum_abs_dpz / stats_.n_accepted : 0.0);
}
