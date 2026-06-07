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

#include "UpdaterGroundPlaneRange.h"

#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/colors.h"
#include "utils/print.h"

using namespace ov_type;
using namespace ov_msckf;

UpdaterGroundPlaneRange::UpdaterGroundPlaneRange(double sigma_range, double min_cos_tilt,
                                                 double chi2_gate, bool use_zonly,
                                                 double min_pzz, double max_res_gate)
    : sigma_range_(sigma_range), min_cos_tilt_(min_cos_tilt),
      chi2_gate_(chi2_gate), use_zonly_(use_zonly),
      min_pzz_(min_pzz), max_res_gate_(max_res_gate) {}

void UpdaterGroundPlaneRange::reset() {
  bootstrapped_ = false;
  z_ground_ = 0.0;
  last_ = LastUpdate();
  stats_ = Stats();
}

bool UpdaterGroundPlaneRange::try_update(std::shared_ptr<State> state,
                                         double t_state, double z_gps, double t_gps) {

  stats_.n_called++;
  if (stats_.first_t < 0)
    stats_.first_t = t_state;
  stats_.last_t = t_state;

  // --- time alignment ---
  double dt = t_state - t_gps;
  if (t_state < t_gps - 0.2 || t_state > t_gps + 0.2) {
    stats_.n_skipped++;
    return false;
  }

  // --- current state ---
  Eigen::Matrix3d R_GtoI = state->_imu->Rot();
  Eigen::Vector3d p_IinG = state->_imu->pos();
  double r22 = R_GtoI(2, 2); // cos(tilt)

  // --- severe tilt guard ---
  if (std::abs(r22) < min_cos_tilt_) {
    PRINT_DEBUG(YELLOW "[GPLANE-RNG] skip: cos_tilt=%.3f < %.3f\n" RESET, r22, min_cos_tilt_);
    stats_.n_skipped++;
    return false;
  }

  // --- bootstrap z_ground on first call ---
  // Physical model: z_gps is already in VIO world frame (caller pre-bootstrapped),
  // so on first call z_gps == p_z and z_ground = p_z - z_gps == 0.  This makes
  // the residual exactly 0 at bootstrap and keeps z_ground at the actual
  // VIO-frame altitude of the ground (0 if VIO origin == takeoff point).
  if (!bootstrapped_) {
    z_ground_ = p_IinG(2) - z_gps;
    bootstrapped_ = true;
    PRINT_INFO(CYAN "[GPLANE-RNG] bootstrap: z_ground=%.3f (p_z=%.3f z_gps=%.3f cos_tilt=%.3f)\n" RESET,
               z_ground_, p_IinG(2), z_gps, r22);
  }

  // --- predicted and measured range ---
  double rho_hat = (p_IinG(2) - z_ground_) / r22;
  double rho_meas = (z_gps - z_ground_) / r22;
  double res_scalar = rho_meas - rho_hat;

  // --- innovation gate ---
  if (max_res_gate_ < 1e8 && std::fabs(res_scalar) > max_res_gate_) {
    stats_.n_rejected++;
    PRINT_INFO(YELLOW "[GPLANE-RNG] REJECT |res|=%.1f > %.1f m (t=%.3f)\n" RESET,
               std::fabs(res_scalar), max_res_gate_, t_state);
    return false;
  }

  // --- Jacobian ---
  // H_order: [q (3), p (3)] = 6 columns
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->p());

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
  // Signs match the convention used throughout OpenVINS (see VioManager
  // feed_measurement_gps_altitude for the same derivation on R(2,2)):
  //   d cos_tilt / d θ_x = +R_GtoI(1,2)
  //   d cos_tilt / d θ_y = -R_GtoI(0,2)
  // (Empirically this is what the rest of the codebase uses; switching to the
  // textbook JPL convention destabilises full-state updates at non-trivial
  // tilt — verified by fly4 sigma=0.3 → 500 km XY divergence on attempt.)
  double coef = -(p_IinG(2) - z_ground_) / (r22 * r22);
  H(0, 0) = coef * R_GtoI(1, 2);    // d ρ / d θ_x
  H(0, 1) = coef * (-R_GtoI(0, 2)); // d ρ / d θ_y
  H(0, 2) = 0.0;                    // d ρ / d θ_z
  H(0, 3) = 0.0;                    // d ρ / d p_x (not observed)
  H(0, 4) = 0.0;                    // d ρ / d p_y (not observed)
  H(0, 5) = 1.0 / r22;              // d ρ / d p_z

  // --- measurement noise ---
  Eigen::MatrixXd R_meas = Eigen::MatrixXd::Zero(1, 1);
  R_meas(0, 0) = sigma_range_ * sigma_range_;
  Eigen::VectorXd res = Eigen::VectorXd::Zero(1);
  res(0) = res_scalar;

  // --- P_zz floor BEFORE update ---
  int pz_idx = 5; // 3 (q) + 2 (p_z)
  if (min_pzz_ > 0) {
    Eigen::MatrixXd P_pre = StateHelper::get_marginal_covariance(state, Hx_order);
    double P_pz_pre = P_pre(pz_idx, pz_idx);
    if (P_pz_pre < min_pzz_) {
      StateHelper::inject_pz_noise(state, min_pzz_ - P_pz_pre);
    }
  }

  // --- marginal Kalman gain diagnostics ---
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  double S = (H * P_marg * H.transpose())(0, 0) + R_meas(0, 0);
  double P_pz_marg = P_marg(pz_idx, pz_idx);
  Eigen::VectorXd K_vec = (P_marg * H.transpose()) / S; // 6×1
  double K_px = K_vec(3);        // p_x gain
  double K_py = K_vec(4);        // p_y gain
  double K_pz = K_vec(5);        // p_z gain
  double K_xy_norm = std::sqrt(K_px * K_px + K_py * K_py);
  double pred_dx = K_px * res_scalar;
  double pred_dy = K_py * res_scalar;
  double pred_dz = K_pz * res_scalar;
  double pred_dxy_norm = std::sqrt(pred_dx * pred_dx + pred_dy * pred_dy);

  // --- full-state diagnostics (orientation / bias contamination) ---
  double pred_dtheta_norm = -1.0, pred_dba_norm = -1.0, pred_dbg_norm = -1.0;
  {
    Eigen::MatrixXd P_full = StateHelper::get_full_covariance(state);
    int pz_global = state->_imu->p()->id() + 2;
    if (pz_global >= 0 && pz_global < P_full.cols()) {
      Eigen::VectorXd K_full = P_full.col(pz_global) / S;
      int q_start = state->_imu->q()->id();
      if (q_start >= 0 && q_start + 2 < K_full.size())
        pred_dtheta_norm = K_full.segment(q_start, 3).norm() * std::fabs(res_scalar);
      int ba_start = state->_imu->ba()->id();
      if (ba_start >= 0 && ba_start + 2 < K_full.size())
        pred_dba_norm = K_full.segment(ba_start, 3).norm() * std::fabs(res_scalar);
      int bg_start = state->_imu->bg()->id();
      if (bg_start >= 0 && bg_start + 2 < K_full.size())
        pred_dbg_norm = K_full.segment(bg_start, 3).norm() * std::fabs(res_scalar);
    }
  }

  // --- chi2 (for diagnostics, not gating by default) ---
  double chi2 = res_scalar * res_scalar / S;
  if (chi2 > chi2_gate_) {
    PRINT_DEBUG(YELLOW "[GPLANE-RNG] large-res t=%.3f res=%.2f chi2=%.1f — accepting anyway\n" RESET,
                t_state, res_scalar, chi2);
  }

  // --- state capture for delta ---
  Eigen::Vector3d p_pre = p_IinG;
  Eigen::Vector3d ba_pre = state->_imu->bias_a();
  Eigen::Vector3d bg_pre = state->_imu->bias_g();

  // --- EKF update ---
  if (use_zonly_) {
    StateHelper::EKFUpdateZOnly(state, Hx_order, H, res, R_meas);
    last_.decision = "ZONLY";
  } else {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R_meas,
                           visual_yaw_update_mode_, visual_yaw_update_scale_,
                           visual_global_yaw_oc_alpha_);
    last_.decision = "FULL";
  }

  // --- state after ---
  Eigen::Vector3d p_post = state->_imu->pos();
  Eigen::Vector3d dba = state->_imu->bias_a() - ba_pre;
  Eigen::Vector3d dbg = state->_imu->bias_g() - bg_pre;

  // --- P_zz floor AFTER update ---
  double pzz_floor_applied = 0.0;
  if (min_pzz_ > 0) {
    Eigen::MatrixXd P_post_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    double P_pz_post = P_post_marg(pz_idx, pz_idx);
    if (P_pz_post < min_pzz_) {
      StateHelper::inject_pz_noise(state, min_pzz_ - P_pz_post);
      pzz_floor_applied = min_pzz_ - P_pz_post;
    }
  }

  // --- populate diagnostics ---
  last_.t = t_state;
  last_.rho_meas = rho_meas;
  last_.rho_hat = rho_hat;
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

  // --- statistics ---
  stats_.n_accepted++;
  stats_.sum_K_pz += K_pz;
  stats_.sum_K_xy_norm += K_xy_norm;
  stats_.sum_dxy_norm += pred_dxy_norm;
  if (pred_dtheta_norm >= 0) stats_.sum_dtheta_norm += pred_dtheta_norm;
  if (pred_dba_norm >= 0) stats_.sum_dba_norm += pred_dba_norm;
  if (pred_dbg_norm >= 0) stats_.sum_dbg_norm += pred_dbg_norm;
  stats_.sum_abs_res += std::abs(res_scalar);
  stats_.sum_abs_dpz += std::abs(p_post(2) - p_pre(2));

  // --- periodic summary ---
  if (t_state - stats_.last_summary_t > 30.0) {
    stats_.last_summary_t = t_state;
    size_t n = stats_.n_accepted + stats_.n_rejected;
    PRINT_INFO(CYAN "[GPLANE-RNG-STAT] t=%.1f calls=%zu acc=%zu rej=%zu skip=%zu "
               "K_pz_mu=%.4f |K_xy|_mu=%.4f |dxy|_mu=%.3f |dtheta|_mu=%.4f "
               "|dba|_mu=%.5f |dbg|_mu=%.5f |res|_mu=%.2f dpz_mu=%.3f\n" RESET,
               t_state, stats_.n_called, stats_.n_accepted, stats_.n_rejected, stats_.n_skipped,
               n > 0 ? stats_.sum_K_pz / n : 0.0,
               n > 0 ? stats_.sum_K_xy_norm / n : 0.0,
               n > 0 ? stats_.sum_dxy_norm / n : 0.0,
               n > 0 ? stats_.sum_dtheta_norm / n : 0.0,
               n > 0 ? stats_.sum_dba_norm / n : 0.0,
               n > 0 ? stats_.sum_dbg_norm / n : 0.0,
               n > 0 ? stats_.sum_abs_res / n : 0.0,
               stats_.n_accepted > 0 ? stats_.sum_abs_dpz / stats_.n_accepted : 0.0);
  }

  // --- per-update EVAL log ---
  PRINT_INFO(CYAN "[GPLANE-RNG] status=%s t=%.3f dt=%+.3fs "
             "rho_meas=%.2f rho_hat=%.2f res=%+.2f chi2=%.1f "
             "cos_tilt=%.3f z_ground=%.2f pz=%.2f->%.2f "
             "K_pz=%.4f |K_xy|=%.4f |dxy|_pred=%.3f "
             "|dtheta|_pred=%.4f |dba|_pred=%.5f |dbg|_pred=%.5f%s\n" RESET,
             last_.decision.c_str(), t_state, dt,
             rho_meas, rho_hat, res_scalar, chi2,
             r22, z_ground_, p_pre(2), p_post(2),
             K_pz, K_xy_norm, pred_dxy_norm,
             pred_dtheta_norm, pred_dba_norm, pred_dbg_norm,
             pzz_floor_applied > 0 ? " FLOOR" : "");

  return true;
}

void UpdaterGroundPlaneRange::print_summary() const {
  size_t n = stats_.n_accepted + stats_.n_rejected;
  double span = stats_.last_t - stats_.first_t;
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] ====== Ground-Plane Range Summary ======\n" RESET);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] sigma=%.2f z_ground=%.2f time=%.1f s\n" RESET,
             sigma_range_, z_ground_, span);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] calls=%zu acc=%zu rej=%zu skip=%zu\n" RESET,
             stats_.n_called, stats_.n_accepted, stats_.n_rejected, stats_.n_skipped);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] K_pz_mu=%.4f |K_xy|_mu=%.4f |dxy|_mu=%.3f m\n" RESET,
             n > 0 ? stats_.sum_K_pz / n : 0.0,
             n > 0 ? stats_.sum_K_xy_norm / n : 0.0,
             n > 0 ? stats_.sum_dxy_norm / n : 0.0);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] |dtheta|_mu=%.4f |dba|_mu=%.5f |dbg|_mu=%.5f\n" RESET,
             n > 0 ? stats_.sum_dtheta_norm / n : 0.0,
             n > 0 ? stats_.sum_dba_norm / n : 0.0,
             n > 0 ? stats_.sum_dbg_norm / n : 0.0);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] |res|_mu=%.2f |dpz|_mu=%.3f m\n" RESET,
             n > 0 ? stats_.sum_abs_res / n : 0.0,
             stats_.n_accepted > 0 ? stats_.sum_abs_dpz / stats_.n_accepted : 0.0);
  PRINT_INFO(GREEN "[GPLANE-RNG-FINAL] ===========================================\n" RESET);
}
