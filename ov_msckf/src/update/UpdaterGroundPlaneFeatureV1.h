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

#ifndef OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_V1_H
#define OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_V1_H

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "state/StateHelper.h"

namespace ov_core {
class FeatureDatabase;
class Feature;
}

namespace ov_msckf {

class State;

/**
 * @brief Stage B v1 — proper two-clone ground-plane feature update.
 *
 * Geometry is identical to v0:
 *   1. Anchor pixel (oldest clone observation) → ray in world frame.
 *   2. Intersect ray with plane z = z_ground → metric p_FinG.
 *   3. Predict the pixel in the CURRENT clone.
 *
 * v1 differs from v0 in the Jacobian:
 *   - v0 treats the anchor pose as FIXED (Schmidt-like) — Hx_order has only
 *     the current clone's (q, p).  This biases the filter: the anchor
 *     pose's true uncertainty silently inflates the measurement.
 *   - v1 includes BOTH the anchor clone and the current clone in Hx_order.
 *     Residual depends on anchor (R_GtoIa, p_IainG) through u_G and through
 *     p_CainG; the analytic Jacobian wires those dependencies through.
 *
 * Modes:
 *   DRY_RUN : compute residual + analytic H + finite-difference H, compare,
 *             log diagnostics, never call EKFUpdate.
 *   UPDATE  : require DRY_RUN to have passed the FD check at least once,
 *             then perform a real EKF update through StateHelper::EKFUpdate.
 *
 * Activation policy (inherited from Stage A):
 *   - Stage A z_ground must have been bootstrapped.
 *   - Stage A's "min-t-after-init" delay must have elapsed (caller's
 *     responsibility — VioManager only calls try_update once Stage A would
 *     itself have fired).
 *   - Optional: at least N Stage A updates have occurred (config'd).
 */
class UpdaterGroundPlaneFeatureV1 {

public:
  enum class Mode { DRY_RUN, UPDATE };

  struct FDCheck {
    bool valid = false;
    // Per-block (anchor θ, anchor p, current θ, current p):
    double analytic_norm[4] = {0, 0, 0, 0};
    double numeric_norm[4]  = {0, 0, 0, 0};
    double max_abs_err[4]   = {0, 0, 0, 0};
    double rel_err[4]       = {0, 0, 0, 0};
    bool   passed[4]        = {false, false, false, false};
    // overall pass:
    bool   passed_all = false;
  };

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
    FDCheck fd; // per-feature FD result is logged separately; this is the last seen
  };

  struct Stats {
    size_t n_called = 0;
    size_t n_skipped_tilt = 0;
    size_t n_skipped_no_clones = 0;
    size_t n_skipped_no_features = 0;
    size_t n_accepted_updates = 0;
    size_t n_features_used_total = 0;
    size_t n_fd_checks = 0;
    size_t n_fd_pass = 0;     // all-4-blocks pass
    size_t n_fd_fail = 0;     // any block failed
    // Per-block tracking (indices: 0=tha, 1=pa, 2=thc, 3=pc):
    size_t n_pass_block[4] = {0, 0, 0, 0};
    size_t n_fail_block[4] = {0, 0, 0, 0};
    double max_rel_err_block[4] = {0, 0, 0, 0};
    double sum_rel_err_block[4] = {0, 0, 0, 0};
    double max_abs_err_block[4] = {0, 0, 0, 0};
    std::vector<double> rel_err_history[4]; // for p95
    double sum_residual_px = 0.0;
    double sum_dxy_norm = 0.0;
    double sum_dz_after = 0.0;
    double max_residual_px = 0.0;
    double max_dxy_norm = 0.0;
    double max_dz_after = 0.0;
    double first_t = -1.0;
    double last_t = -1.0;
    double last_summary_t = -1.0;
  };

  UpdaterGroundPlaneFeatureV1(Mode mode,
                              double sigma_pixel = 50.0,         // E1 (was 3.0)
                              int max_features = 2,              // E1 (was 5)
                              double center_frac = 0.8,
                              double min_cos_tilt = 0.85,
                              double max_residual_px = 5.0,
                              double min_lambda = 0.5,
                              double max_lambda = 100.0,
                              double fd_step_rot = 1e-4,
                              double fd_step_pos = 1e-2,
                              double fd_rel_tol_rot = 1e-3,
                              double fd_rel_tol_pos = 3e-3,
                              double fd_max_abs_rel_tol = 1e-2,
                              bool   exclude_used_from_msckf = false, // E1 (was true)
                              int    dump_first_n = 0);

  /// Attempt one update at the current camera timestamp.  In DRY_RUN mode
  /// runs the FD check and logs results; never touches state/covariance.
  /// In UPDATE mode performs the full EKF update.
  bool try_update(std::shared_ptr<State> state,
                  std::shared_ptr<ov_core::FeatureDatabase> features,
                  double t_state,
                  double z_ground);

  /// Variant used from the MSCKF update path: consume the same candidate
  /// tracks that are about to be used by MSCKF, and pick a valid two-clone
  /// observation span from each track.
  bool try_update_candidates(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_core::Feature>> &feature_candidates,
      double t_state,
      double z_ground);

  /// Feature IDs consumed by the LATEST successful UPDATE call.  When
  /// exclude_used_from_msckf is true, callers should pass these to the
  /// MSCKF updater so the same observations are not double-counted.
  const std::unordered_set<size_t> &last_used_feat_ids() const { return last_used_ids_; }

  /// True iff features consumed by v1's most recent UPDATE call should be
  /// removed from the MSCKF feature pool to prevent double-counting.
  bool exclude_used_from_msckf() const { return exclude_used_from_msckf_; }

  const LastUpdate &last_update() const { return last_; }
  const Stats &stats() const { return stats_; }

  void set_visual_yaw_update_control(StateHelper::VisualYawUpdateMode mode, double scale, double global_alpha) {
    visual_yaw_update_mode_ = mode;
    visual_yaw_update_scale_ = scale;
    visual_global_yaw_oc_alpha_ = global_alpha;
  }

  void reset();
  void print_summary() const;

private:
  Mode mode_;
  double sigma_pixel_;
  int max_features_;
  double center_frac_;
  double min_cos_tilt_;
  double max_residual_px_;
  double min_lambda_;
  double max_lambda_;
  double fd_step_rot_;
  double fd_step_pos_;
  double fd_rel_tol_rot_;
  double fd_rel_tol_pos_;
  double fd_max_abs_rel_tol_;
  bool exclude_used_from_msckf_;

  LastUpdate last_;
  Stats stats_;
  std::unordered_set<size_t> last_used_ids_;

  StateHelper::VisualYawUpdateMode visual_yaw_update_mode_ = StateHelper::VisualYawUpdateMode::ORIGINAL;
  double visual_yaw_update_scale_ = 1.0;
  double visual_global_yaw_oc_alpha_ = 0.0;

  /// Verbose diagnostic dump: dump the full analytic, numeric, and diff
  /// 2x12 matrices + intermediates + step-size sweep for the first
  /// `dump_remaining_` features that pass forward()+analytic+default-FD.
  int dump_remaining_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GROUNDPLANE_FEATURE_V1_H
