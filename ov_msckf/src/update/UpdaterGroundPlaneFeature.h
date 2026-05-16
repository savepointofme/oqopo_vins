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

#ifndef OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_H
#define OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_H

#include <memory>
#include <string>
#include <vector>

namespace ov_core {
class FeatureDatabase;
}

namespace ov_msckf {

class State;

/**
 * @brief Stage B v0 — plane-anchored 2-view feature update.
 *
 * For each currently-tracked feature seen at both the OLDEST clone (anchor)
 * and the LATEST clone (current), we:
 *   1. Undistort the anchor pixel to a normalized ray.
 *   2. Transform that ray into the world frame using the anchor clone pose.
 *   3. Intersect the ray with the ground plane z = z_ground → metric p_FinG.
 *   4. Use p_FinG (treated as fixed) to predict the pixel in the CURRENT
 *      frame; the 2-D pixel residual then gives full 6-DoF observability of
 *      the current clone pose, including monocular scale.
 *
 * Gating (conservative for v0):
 *   - cos_tilt ≥ min_cos_tilt  (drone roughly level)
 *   - anchor pixel inside central center_frac × image
 *   - |u_G.z| ≥ 0.2 (ray not near-horizontal)
 *   - 0.5 ≤ λ ≤ 100 m
 *   - |raw residual| ≤ max_residual_px
 *   - cap at max_features per update
 *
 * Anchor pose is treated as fixed (Schmidt-like).  The unmodelled anchor
 * uncertainty is absorbed by inflating the per-pixel sigma.
 */
class UpdaterGroundPlaneFeature {

public:
  struct LastUpdate {
    double t = -1.0;
    int n_candidates = 0;
    int n_passed_gate = 0;
    int n_used = 0;
    double mean_residual_px = 0.0;
    double max_residual_px = 0.0;
    double dxy_norm = 0.0;
    double dz_after = 0.0;
    std::string decision = "NONE";
  };

  struct Stats {
    size_t n_called = 0;
    size_t n_skipped_tilt = 0;
    size_t n_skipped_no_clones = 0;
    size_t n_skipped_no_features = 0;
    size_t n_accepted_updates = 0;
    size_t n_features_used_total = 0;
    double sum_residual_px = 0.0;
    double sum_dxy_norm = 0.0;
    double sum_dz_after = 0.0;
    double first_t = -1.0;
    double last_t = -1.0;
    double last_summary_t = -1.0;
  };

  UpdaterGroundPlaneFeature(double sigma_pixel = 3.0,
                            int max_features = 5,
                            double center_frac = 0.8,
                            double min_cos_tilt = 0.85,
                            double max_residual_px = 5.0,
                            double min_lambda = 0.5,
                            double max_lambda = 100.0);

  /// Attempt one EKF update at the given camera timestamp.  Returns true if
  /// at least one feature was used.  z_ground comes from Stage A's bootstrap.
  bool try_update(std::shared_ptr<State> state,
                  std::shared_ptr<ov_core::FeatureDatabase> features,
                  double t_state,
                  double z_ground);

  const LastUpdate &last_update() const { return last_; }
  const Stats &stats() const { return stats_; }

  void reset();
  void print_summary() const;

private:
  double sigma_pixel_;
  int max_features_;
  double center_frac_;
  double min_cos_tilt_;
  double max_residual_px_;
  double min_lambda_;
  double max_lambda_;

  LastUpdate last_;
  Stats stats_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_H
