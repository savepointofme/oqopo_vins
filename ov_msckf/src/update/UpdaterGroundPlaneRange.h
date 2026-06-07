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

#ifndef OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H
#define OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H

#include <memory>
#include <string>
#include <vector>

#include "UpdaterOptions.h"
#include "state/StateHelper.h"

namespace ov_type {
class Type;
}

namespace ov_msckf {

class State;

/**
 * @brief Pseudo-rangefinder updater with flat-ground assumption.
 *
 * Treats GPS altitude as a pseudo slant-range measurement to a local ground
 * plane.  The ground plane is defined by n_G = [0,0,1]^T and offset d = -z_ground.
 *
 * Measurement model (range along downward body ray):
 *   cos_tilt  = R_GtoI(2,2)                  — body z projection onto world z
 *   rho_meas  = (z_gps - z_ground) / cos_tilt     — pseudo slant range
 *   rho_hat   = (p_z  - z_ground) / cos_tilt     — predicted slant range
 *   r         = rho_meas - rho_hat
 *
 * Jacobian (H = d rho / d [theta, p]).  Signs match the convention used by
 * the rest of OpenVINS (see VioManager::feed_measurement_gps_altitude):
 *   d_cos_tilt / d_theta_x = +R_GtoI(1,2)
 *   d_cos_tilt / d_theta_y = -R_GtoI(0,2)
 * therefore (coef = -(p_z - z0) / cos_tilt^2):
 *   d_rho / d_theta_x =  coef * R_GtoI(1,2)
 *   d_rho / d_theta_y = -coef * R_GtoI(0,2)
 *   d_rho / d_theta_z = 0
 *   d_rho / d_p_z     = 1 / cos_tilt
 *
 * On first accepted measurement, z_ground is bootstrapped so that the
 * predicted range matches the measured range (residual = 0), absorbing any
 * VIO–GPS origin mismatch into z_ground.
 */
class UpdaterGroundPlaneRange {

public:
  /// Per-update diagnostic snapshot (populated by every try_update call).
  struct LastUpdate {
    double t = -1.0;
    double rho_meas = 0.0;
    double rho_hat = 0.0;
    double residual = 0.0;
    double cos_tilt = 0.0;
    double z_ground = 0.0;
    double pz_before = 0.0;
    double pz_after = 0.0;
    double K_pz = 0.0;
    double K_px = 0.0;
    double K_py = 0.0;
    double K_xy_norm = 0.0;
    double pred_dx = 0.0;
    double pred_dy = 0.0;
    double pred_dz = 0.0;
    double pred_dtheta_norm = 0.0;
    double pred_dba_norm = 0.0;
    double pred_dbg_norm = 0.0;
    double S = 0.0;
    std::string decision = "NONE";
  };

  /// Accumulated statistics.
  struct Stats {
    size_t n_called = 0;
    size_t n_skipped = 0;
    size_t n_rejected = 0;
    size_t n_accepted = 0;
    double sum_K_pz = 0.0;
    double sum_K_xy_norm = 0.0;
    double sum_dxy_norm = 0.0;
    double sum_dtheta_norm = 0.0;
    double sum_dba_norm = 0.0;
    double sum_dbg_norm = 0.0;
    double sum_abs_res = 0.0;
    double sum_abs_dpz = 0.0;
    double first_t = -1.0;
    double last_t = -1.0;
    double last_summary_t = -1.0;
  };

  UpdaterGroundPlaneRange(double sigma_range, double min_cos_tilt = 0.3,
                          double chi2_gate = 10000.0, bool use_zonly = true,
                          double min_pzz = 0.0, double max_res_gate = 1e9);

  /// Feed a GPS altitude sample.  Performs time-alignment check, then if
  /// within tolerance, runs the EKF update.
  /// @return true if the measurement was accepted and applied
  bool try_update(std::shared_ptr<State> state, double t_state,
                  double z_gps, double t_gps);

  void set_visual_yaw_update_control(StateHelper::VisualYawUpdateMode mode, double scale, double global_alpha) {
    visual_yaw_update_mode_ = mode;
    visual_yaw_update_scale_ = scale;
    visual_global_yaw_oc_alpha_ = global_alpha;
  }

  /// Accessors
  const LastUpdate &last_update() const { return last_; }
  const Stats &stats() const { return stats_; }
  double z_ground() const { return z_ground_; }
  bool bootstrapped() const { return bootstrapped_; }

  /// Reset bootstrap and statistics (call between runs).
  void reset();

  /// Print final summary.
  void print_summary() const;

private:
  double sigma_range_;      ///< Range measurement noise stddev (m)
  double min_cos_tilt_;     ///< Minimum cos(tilt) before skipping (default 0.3 ~ 72°)
  double chi2_gate_;        ///< Chi^2 gate (default permissive)
  bool use_zonly_;          ///< If true, use Z-only update (only p_z corrected)
  double min_pzz_;          ///< P_zz floor (0 = disabled)
  double max_res_gate_;     ///< |residual| rejection gate (1e9 = disabled)

  // Ground plane bootstrap
  bool bootstrapped_ = false;
  double z_ground_ = 0.0;

  // Diagnostics
  LastUpdate last_;
  Stats stats_;

  StateHelper::VisualYawUpdateMode visual_yaw_update_mode_ = StateHelper::VisualYawUpdateMode::ORIGINAL;
  double visual_yaw_update_scale_ = 1.0;
  double visual_global_yaw_oc_alpha_ = 0.0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H
