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

#ifndef OV_MSCKF_VISUAL_OBSERVABILITY_POLICY_H
#define OV_MSCKF_VISUAL_OBSERVABILITY_POLICY_H

// =============================================================================
// VisualObservabilityPolicy
// -----------------------------------------------------------------------------
// Formal visual observability constraint layer for OpenVINS downward-looking
// UAV flight.  Builds the FEJ-consistent unobservable gauge N, then projects
// the measurement Jacobian H:
//
//   H_oc = H - H * Q * Q^T      (Q = orthonormal basis of col-space of N)
//
// Applied BEFORE chi-square gating, so the gate tests the OC-consistent
// residual rather than the raw one.  This avoids accepting measurements whose
// chi-2 passes only because yaw info inflates the Mahalanobis distance.
//
// Modes
//   DISABLED                    no projection (pass-through)
//   GLOBAL_YAW_OC_FEJ_PRECHI2  1-D global yaw gauge, FEJ state, pre-chi2
//   VISUAL_4D_OC_FEJ_PRECHI2   4-D gauge: global yaw + translation x/y/z,
//                               FEJ state, pre-chi2
//
// The old GLOBAL_YAW_OC_PROJECTION mode (applied inside EKFUpdate) is kept
// separately in StateHelper for backward compatibility and is NOT this class.
// =============================================================================

#include <Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

namespace ov_type {
class Type;
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;

class VisualObservabilityPolicy {
public:
  enum class Mode {
    DISABLED = 0,
    GLOBAL_YAW_OC_FEJ_PRECHI2 = 1,
    VISUAL_4D_OC_FEJ_PRECHI2 = 2,
  };

  struct Diag {
    bool projection_applied = false;
    bool used_fej_basis = false;
    int rank_N = 0;
    double condition_N = 0.0;   // max_sv / min_nonzero_sv of N
    double norm_HN_before = 0.0;
    double norm_HN_after = 0.0;
    double rel_norm_HN_before = 0.0;  // norm_HN_before / norm_H
    double rel_norm_HN_after = 0.0;   // norm_HN_after  / norm_H
  };

  explicit VisualObservabilityPolicy(Mode mode = Mode::DISABLED) : mode_(mode) {}

  static Mode mode_from_string(const std::string &s);
  static bool is_prechi2_mode_string(const std::string &s);

  bool is_active() const { return mode_ != Mode::DISABLED; }
  Mode mode() const { return mode_; }

  // Build the column-offset vector from H_order (same as what EKFUpdate
  // computes internally).  Provided as a convenience so callers don't need to
  // duplicate this loop.
  static std::vector<int> build_H_id(
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order);

  // Apply OC projection to H.
  // H_order / H_id  : variable ordering and per-variable column offsets in H.
  // state           : current filter state (used for FEJ lookups).
  // diag_out        : optional per-call diagnostics (may be nullptr).
  // Returns the projected H (or the original H if projection was skipped).
  Eigen::MatrixXd apply(
      const Eigen::MatrixXd &H,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const std::vector<int> &H_id,
      std::shared_ptr<State> state,
      Diag *diag_out = nullptr) const;

private:
  Mode mode_;

  // Build the 1-D yaw gauge column vector (length = H.cols()).
  // FEJ rotation and position are used when use_fej = true.
  Eigen::VectorXd build_yaw_gauge(
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const std::vector<int> &H_id,
      int H_cols,
      std::shared_ptr<State> state,
      bool use_fej) const;

  // Build the 4-D gauge matrix [yaw | tx | ty | tz] (H.cols() x 4).
  Eigen::MatrixXd build_4d_gauge(
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const std::vector<int> &H_id,
      int H_cols,
      std::shared_ptr<State> state,
      bool use_fej) const;
};

} // namespace ov_msckf

#endif // OV_MSCKF_VISUAL_OBSERVABILITY_POLICY_H
