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

/**
 * @file test_joseph_update.cpp
 *
 * Gate 1 unit tests for StateHelper::josephCovUpdate and StateHelper::EKFUpdateJoseph.
 *
 * These tests verify:
 *   T1  Full-gain Joseph form matches direct Joseph matrix formula
 *   T2  Full-gain Joseph form matches standard EKF update (P -= K*H*P) for optimal K
 *   T3  Masked-gain Joseph form matches direct matrix formula (I-KH)*P*(I-KH)^T + R*K*K^T
 *   T4  Output covariance is symmetric: ||P - P^T||_F < tol
 *   T5  Output covariance diagonal is non-negative
 *   T6  Output covariance is PSD (all eigenvalues >= -tol) — random 100-trial test
 *   T7  Masked K entries produce zero state correction in EKFUpdateJoseph
 *   T8  Active K entries produce non-zero state correction in EKFUpdateJoseph
 *   T9  Real nonzero IMU state (set via _imu->set_value, not sub-state) — full path
 *
 * No GPS, VIO pipeline, or runtime connection is tested here.
 * Only StateHelper::josephCovUpdate and StateHelper::EKFUpdateJoseph are exercised.
 *
 * ============================================================================
 * IMPORTANT — IMU sub-state synchronization (found during Gate 1):
 *
 * In OpenVINS, `IMU::update(dx)` reads from `_imu->_value` (the 16-dim nominal
 * state vector: [q(4), p(3), v(3), bg(3), ba(3)]) and writes a corrected value
 * via `set_value_internal()`, which propagates to the sub-states (_pose, _v, etc.)
 *
 * HOWEVER: `_imu->p()->set_value(v)` only updates `_pose->_p->_value`.
 * It does NOT update `_imu->_value[4..6]`.
 *
 * Consequence: if you call `_imu->p()->set_value(p_init)` before `EKFUpdateJoseph`,
 * then `pos()` reads the sub-state and returns `p_init` correctly.  But when
 * `_imu->update(dx)` runs it will use `_imu->_value[4..6]` (still zero) as the base
 * and compute `p_new = (0,0,0) + dx_p`, overwriting the sub-state.  After the update,
 * `pos()` returns `(0, 0, K_pz*res)` instead of `p_init + (0, 0, K_pz*res)`.
 *
 * CORRECT patterns:
 *   - Use `_imu->set_value(full_16_dim_vector)` to set IMU state.
 *   - Or rely on propagation (which always goes through `_imu->_value`).
 *   - Do NOT use `_imu->p()->set_value(v)` to set initial state for testing.
 *
 * T7/T8 use default zero position to avoid this trap.
 * T9 uses `_imu->set_value()` to test with a nonzero position.
 * ============================================================================
 */

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"

using namespace ov_msckf;
using namespace ov_type;

// ---------------------------------------------------------------------------
// Helper: generate random N×N symmetric positive definite matrix
// ---------------------------------------------------------------------------
static Eigen::MatrixXd rand_spd(int N, std::mt19937 &rng, double scale = 1.0) {
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  Eigen::MatrixXd A(N, N);
  for (int i = 0; i < N; i++)
    for (int j = 0; j < N; j++)
      A(i, j) = ud(rng);
  // P = A*A^T + N*I  — guaranteed SPD
  Eigen::MatrixXd P = scale * (A * A.transpose() + (double)N * Eigen::MatrixXd::Identity(N, N));
  return P;
}

// ---------------------------------------------------------------------------
// Helper: direct Joseph form reference
//   P_ref = (I - K*H^T)*P*(I - K*H^T)^T + R*K*K^T
// ---------------------------------------------------------------------------
static Eigen::MatrixXd direct_joseph(const Eigen::MatrixXd &P, const Eigen::VectorXd &K,
                                      const Eigen::VectorXd &H, double R) {
  const int N = (int)P.rows();
  Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(N, N) - K * H.transpose();
  return IKH * P * IKH.transpose() + R * K * K.transpose();
}

// ---------------------------------------------------------------------------
// Helper: standard EKF update reference (P -= K * M^T where M = P*H)
// ---------------------------------------------------------------------------
static Eigen::MatrixXd standard_ekf(const Eigen::MatrixXd &P, const Eigen::VectorXd &K,
                                     const Eigen::VectorXd &H) {
  Eigen::VectorXd M = P * H;
  Eigen::MatrixXd P_new = P - K * M.transpose();
  // Make symmetric (standard form assumes P_new is already symmetric for optimal K)
  return 0.5 * (P_new + P_new.transpose());
}

// ---------------------------------------------------------------------------
// Check helpers
// ---------------------------------------------------------------------------
struct TestResult {
  std::string name;
  bool passed = true;
  std::string detail;
};

static bool check(TestResult &r, bool cond, const std::string &msg) {
  if (!cond) {
    r.passed = false;
    r.detail += "  FAIL: " + msg + "\n";
  }
  return cond;
}

// ---------------------------------------------------------------------------
// T1: Full-gain Joseph matches direct Joseph formula
// ---------------------------------------------------------------------------
static TestResult test_T1_full_matches_direct(std::mt19937 &rng) {
  TestResult r;
  r.name = "T1 full-gain Joseph == direct Joseph formula";
  const int N = 10;
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H(N);
    std::uniform_real_distribution<double> ud(-1.0, 1.0);
    for (int i = 0; i < N; i++) H(i) = ud(rng);
    double R = 0.5 + std::uniform_real_distribution<double>(0.0, 2.0)(rng);

    // Optimal K
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;

    // Reference
    Eigen::MatrixXd P_ref = direct_joseph(P, K, H, R);

    // Function under test
    Eigen::MatrixXd P_test = P;
    StateHelper::josephCovUpdate(P_test, K, H, R);

    double err = (P_test - P_ref).norm();
    check(r, err < 1e-8, "trial=" + std::to_string(trial) + " ||P_joseph - P_direct||=" + std::to_string(err));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T2: Full-gain Joseph matches standard EKF update (optimal K)
// ---------------------------------------------------------------------------
static TestResult test_T2_full_matches_standard_ekf(std::mt19937 &rng) {
  TestResult r;
  r.name = "T2 full-gain Joseph == standard EKF update (optimal K)";
  const int N = 10;
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H(N);
    std::uniform_real_distribution<double> ud(-1.0, 1.0);
    for (int i = 0; i < N; i++) H(i) = ud(rng);
    double R = 0.5 + std::uniform_real_distribution<double>(0.0, 2.0)(rng);

    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;

    Eigen::MatrixXd P_std = standard_ekf(P, K, H);
    Eigen::MatrixXd P_test = P;
    StateHelper::josephCovUpdate(P_test, K, H, R);

    double err = (P_test - P_std).norm();
    // Note: Joseph form and standard form are algebraically equivalent for optimal K,
    // but differ slightly numerically.  Tolerance 1e-6 is appropriate for N=10.
    check(r, err < 1e-6, "trial=" + std::to_string(trial) + " ||P_joseph - P_standard||=" + std::to_string(err));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T3: Masked-gain Joseph matches direct matrix formula
// ---------------------------------------------------------------------------
static TestResult test_T3_masked_matches_direct(std::mt19937 &rng) {
  TestResult r;
  r.name = "T3 masked-gain Joseph == direct (I-KH)*P*(I-KH)^T + R*K*K^T";
  const int N = 12;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    // Sparse H: only entries 3..5 non-zero (simulates p_z in position block)
    H(3) = ud(rng);
    H(4) = ud(rng);
    H(5) = ud(rng);
    double R = 0.5 + std::uniform_real_distribution<double>(0.0, 2.0)(rng);

    // Optimal K, then mask: keep only entries 3..5 (the "active" block)
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K_opt = (P * H) / S;
    Eigen::VectorXd K_masked = K_opt;
    for (int i = 0; i < N; i++) {
      if (i < 3 || i > 5) K_masked(i) = 0.0;
    }

    Eigen::MatrixXd P_ref = direct_joseph(P, K_masked, H, R);
    Eigen::MatrixXd P_test = P;
    StateHelper::josephCovUpdate(P_test, K_masked, H, R);

    double err = (P_test - P_ref).norm();
    check(r, err < 1e-8, "trial=" + std::to_string(trial) + " ||P_joseph_masked - P_direct||=" + std::to_string(err));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T4: Symmetry — ||P - P^T||_F < tol after update (full and masked)
// ---------------------------------------------------------------------------
static TestResult test_T4_symmetry(std::mt19937 &rng) {
  TestResult r;
  r.name = "T4 symmetry: ||P - P^T||_F < 1e-12 after update";
  const int N = 8;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 40; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    H(2) = ud(rng);
    double R = 1.0;
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;
    // Mask half the entries
    if (trial % 2 == 1) {
      for (int i = N / 2; i < N; i++) K(i) = 0.0;
    }
    StateHelper::josephCovUpdate(P, K, H, R);
    double sym_err = (P - P.transpose()).norm();
    check(r, sym_err < 1e-12, "trial=" + std::to_string(trial) + " sym_err=" + std::to_string(sym_err));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T5: Diagonal non-negative after update
// ---------------------------------------------------------------------------
static TestResult test_T5_diag_nonneg(std::mt19937 &rng) {
  TestResult r;
  r.name = "T5 diagonal non-negative after update";
  const int N = 8;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 40; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    H(2) = ud(rng);
    double R = 1.0;
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;
    if (trial % 2 == 1) {
      for (int i = N / 2; i < N; i++) K(i) = 0.0;
    }
    StateHelper::josephCovUpdate(P, K, H, R);
    double min_diag = P.diagonal().minCoeff();
    check(r, min_diag >= -1e-10, "trial=" + std::to_string(trial) + " min_diag=" + std::to_string(min_diag));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T6: PSD — all eigenvalues >= -tol after 100 random trials
// ---------------------------------------------------------------------------
static TestResult test_T6_psd_random(std::mt19937 &rng) {
  TestResult r;
  r.name = "T6 PSD: all eigenvalues >= -1e-8 after update (100 random trials)";
  const int N = 8;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  int fail_count = 0;
  for (int trial = 0; trial < 100; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng, 0.5 + std::abs(ud(rng)));
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    // Sparse H (random non-zero pattern)
    int nnz = 1 + (trial % 3);
    for (int k = 0; k < nnz; k++) H((trial * 7 + k * 3) % N) = ud(rng);
    double R = 0.1 + std::abs(ud(rng));
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;
    // Randomly mask some entries
    if (trial % 3 == 0) {
      for (int i = 0; i < N; i++) {
        if ((trial + i) % 2 == 0) K(i) = 0.0;
      }
    }
    StateHelper::josephCovUpdate(P, K, H, R);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
    double min_ev = es.eigenvalues().minCoeff();
    if (min_ev < -1e-8) {
      fail_count++;
      r.detail += "  trial=" + std::to_string(trial) + " min_eigenvalue=" + std::to_string(min_ev) + "\n";
    }
  }
  check(r, fail_count == 0, std::to_string(fail_count) + "/100 trials had eigenvalue < -1e-8");
  return r;
}

// ---------------------------------------------------------------------------
// T7+T8: EKFUpdateJoseph — masked state unchanged, active state corrected.
//
// Design note: _imu->p()->set_value(v) only updates the sub-state's _value but
// NOT _imu->_value[4..6].  When update(dx) runs it reads from _imu->_value and
// overwrites the sub-state.  To avoid this confusion the test uses:
//   - Default IMU state (p = (0,0,0), correctly stored in _imu->_value)
//   - A synthetic K_masked (not derived from state's covariance) with known K_pz
//   - Verifies state correction = K_masked * res after the call
//
// Uses StateHelper::get_full_covariance() to read covariance (State::_Cov is private).
// ---------------------------------------------------------------------------
static TestResult test_T7_T8_state_update(std::mt19937 &rng) {
  (void)rng;
  TestResult r;
  r.name = "T7+T8 EKFUpdateJoseph: masked DOF unchanged, active DOF corrected";

  // Build a minimal State: just IMU (no clones, no cameras)
  VioManagerOptions params;
  params.state_options.num_cameras = 0;
  params.state_options.max_clone_size = 0;
  params.state_options.max_slam_features = 0;
  auto state = std::make_shared<State>(params.state_options);
  const int N = (int)StateHelper::get_full_covariance(state).rows();

  // Global indices: p_z is at p()->id() + 2; pose error-state = [dq(3), dp(3), ...]
  const int p_id = state->_imu->p()->id();  // = 3 for standard IMU layout

  // Measurement: h(x) = p_z — H_full has a 1 at p_z only
  Eigen::VectorXd H_full = Eigen::VectorXd::Zero(N);
  H_full(p_id + 2) = 1.0;

  // Synthetic K_masked: known K_pz = 0.5, all other entries zero.
  // Not derived from state->_Cov — this isolates the state-correction logic from
  // the covariance. The Joseph update will still run on _Cov, but correctness of
  // the state correction is independent of _Cov.
  const double K_pz = 0.5;
  Eigen::VectorXd K_masked = Eigen::VectorXd::Zero(N);
  K_masked(p_id + 2) = K_pz;  // only p_z active; all others (p_x, p_y, v, q, bg, ba) masked

  double R = 1.0;
  double res = 4.0;  // expected p_z correction = K_pz * res = 0.5 * 4.0 = 2.0

  // Read default state (default p = (0,0,0) — correctly in _imu->_value)
  Eigen::Vector3d p_before = state->_imu->pos();
  Eigen::Vector3d v_before = state->_imu->vel();
  Eigen::Vector4d q_before = state->_imu->quat();

  // Apply EKFUpdateJoseph with synthetic masked K
  StateHelper::EKFUpdateJoseph(state, K_masked, H_full, R, res);

  Eigen::Vector3d p_after = state->_imu->pos();
  Eigen::Vector3d v_after = state->_imu->vel();
  Eigen::Vector4d q_after = state->_imu->quat();

  // T7: masked DOFs (p_x, p_y) must NOT change
  double dp_x = std::abs(p_after(0) - p_before(0));
  double dp_y = std::abs(p_after(1) - p_before(1));
  check(r, dp_x < 1e-12, "T7 p_x changed: dp_x=" + std::to_string(dp_x));
  check(r, dp_y < 1e-12, "T7 p_y changed: dp_y=" + std::to_string(dp_y));

  // T7: velocity and attitude must NOT change (K entries are zero)
  double dv = (v_after - v_before).norm();
  double dq = (q_after - q_before).norm();
  check(r, dv < 1e-12, "T7 velocity changed: dv=" + std::to_string(dv));
  check(r, dq < 1e-12, "T7 attitude changed: dq=" + std::to_string(dq));

  // T8: p_z MUST change
  double dp_z = std::abs(p_after(2) - p_before(2));
  check(r, dp_z > 1e-6, "T8 p_z did NOT change: dp_z=" + std::to_string(dp_z));

  // T8: p_z correction must equal K_pz * res
  double expected_dpz = K_pz * res;  // = 2.0
  double dpz_err = std::abs(dp_z - expected_dpz);
  check(r, dpz_err < 1e-10,
        "T8 p_z correction: expected=" + std::to_string(expected_dpz) +
        " got=" + std::to_string(dp_z) + " err=" + std::to_string(dpz_err));

  // Covariance symmetry and diagonal after update
  Eigen::MatrixXd P_after = StateHelper::get_full_covariance(state);
  double sym_err = (P_after - P_after.transpose()).norm();
  check(r, sym_err < 1e-12, "covariance not symmetric after EKFUpdateJoseph: sym_err=" + std::to_string(sym_err));
  double min_diag = P_after.diagonal().minCoeff();
  check(r, min_diag >= -1e-10, "negative diagonal after EKFUpdateJoseph: min_diag=" + std::to_string(min_diag));

  return r;
}

// ---------------------------------------------------------------------------
// T_MASKED_BLOCK: Verify P(masked_block, masked_block) is unchanged
// (masked-vs-masked covariance entries must not change)
// ---------------------------------------------------------------------------
static TestResult test_T_masked_block_unchanged(std::mt19937 &rng) {
  TestResult r;
  r.name = "T_MASKED_BLOCK: P entries for fully-masked block unchanged after masked-gain update";
  const int N = 12;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    H(3) = ud(rng);  // only entry 3 is non-zero in H
    double R = 1.0;
    double S = H.transpose() * P * H + R;
    Eigen::VectorXd K = (P * H) / S;
    // Keep K only for entry 3; zero everything else
    Eigen::VectorXd K_masked = Eigen::VectorXd::Zero(N);
    K_masked(3) = K(3);

    // Entries [6..11] are fully masked (K=0 for all of them)
    Eigen::MatrixXd P_block_before = P.block(6, 6, 6, 6);

    StateHelper::josephCovUpdate(P, K_masked, H, R);

    Eigen::MatrixXd P_block_after = P.block(6, 6, 6, 6);
    double block_err = (P_block_after - P_block_before).norm();
    check(r, block_err < 1e-12,
          "trial=" + std::to_string(trial) + " masked block changed: err=" + std::to_string(block_err));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T9: Real nonzero IMU state via _imu->set_value() — full update path
//
// Uses _imu->set_value(16-dim) to initialize a nonzero position (10, 20, 30).
// This exercises the real OpenVINS IMU update path used in runtime propagation.
//
// IMU _value layout (16 dim): [q(4), p(3), v(3), bg(3), ba(3)]
//   q at indices 0..3: JPL [qx, qy, qz, qw], unit = (0,0,0,1)
//   p at indices 4..6
//   v at indices 7..9,  bg at 10..12,  ba at 13..15
//
// IMU error-state layout (15 dim): [δθ(3), δp(3), δv(3), δbg(3), δba(3)]
//   p()->id() = 3  (position error starts at global index 3)
//   p_z global index = p()->id() + 2 = 5
// ---------------------------------------------------------------------------
static TestResult test_T9_real_imu_nonzero(std::mt19937 &rng) {
  (void)rng;
  TestResult r;
  r.name = "T9 real nonzero IMU state via _imu->set_value(): masked Joseph update";

  // Build a minimal State: just IMU (no clones, no cameras)
  VioManagerOptions params;
  params.state_options.num_cameras = 0;
  params.state_options.max_clone_size = 0;
  params.state_options.max_slam_features = 0;
  auto state = std::make_shared<State>(params.state_options);
  const int N = (int)StateHelper::get_full_covariance(state).rows();

  // Set IMU state via the correct full-state path (not sub-state set_value).
  // _value layout: [q(4)=identity, p(3)=[10,20,30], v(3)=0, bg(3)=0, ba(3)=0]
  Eigen::VectorXd imu_val = Eigen::VectorXd::Zero(16);
  imu_val(3)  = 1.0;   // q_w = 1 (JPL unit quaternion: [qx,qy,qz,qw])
  imu_val(4)  = 10.0;  // p_x
  imu_val(5)  = 20.0;  // p_y
  imu_val(6)  = 30.0;  // p_z
  // v, bg, ba stay zero
  state->_imu->set_value(imu_val);

  // Verify set_value worked: pos() should read (10, 20, 30)
  Eigen::Vector3d p_init_check = state->_imu->pos();
  check(r, std::abs(p_init_check(0) - 10.0) < 1e-12, "setup: p_x != 10 after set_value");
  check(r, std::abs(p_init_check(1) - 20.0) < 1e-12, "setup: p_y != 20 after set_value");
  check(r, std::abs(p_init_check(2) - 30.0) < 1e-12, "setup: p_z != 30 after set_value");

  // Measurement: h(x) = p_z → H_full[5] = 1  (p_id=3, p_z_global=5)
  const int p_id = state->_imu->p()->id();  // should be 3
  check(r, p_id == 3, "setup: p()->id() = " + std::to_string(p_id) + " (expected 3)");
  Eigen::VectorXd H_full = Eigen::VectorXd::Zero(N);
  H_full(p_id + 2) = 1.0;  // p_z component

  // K_masked: only p_z active (K_pz = 0.5), all others zero
  const double K_pz  = 0.5;
  const double res   = 6.0;
  const double R     = 1.0;
  Eigen::VectorXd K_masked = Eigen::VectorXd::Zero(N);
  K_masked(p_id + 2) = K_pz;

  // Save pre-update state
  Eigen::Vector3d p_before  = state->_imu->pos();
  Eigen::Vector3d v_before  = state->_imu->vel();
  Eigen::Vector4d q_before  = state->_imu->quat();
  Eigen::Vector3d bg_before = state->_imu->bias_g();
  Eigen::Vector3d ba_before = state->_imu->bias_a();

  // Apply EKFUpdateJoseph (only p_z state correction, masked K)
  StateHelper::EKFUpdateJoseph(state, K_masked, H_full, R, res);

  Eigen::Vector3d p_after  = state->_imu->pos();
  Eigen::Vector3d v_after  = state->_imu->vel();
  Eigen::Vector4d q_after  = state->_imu->quat();
  Eigen::Vector3d bg_after = state->_imu->bias_g();
  Eigen::Vector3d ba_after = state->_imu->bias_a();

  // T9a: p_x and p_y must stay at 10 and 20 (masked)
  check(r, std::abs(p_after(0) - 10.0) < 1e-12,
        "T9a p_x changed: before=10 after=" + std::to_string(p_after(0)));
  check(r, std::abs(p_after(1) - 20.0) < 1e-12,
        "T9a p_y changed: before=20 after=" + std::to_string(p_after(1)));

  // T9b: p_z must change by exactly K_pz * res = 0.5 * 6.0 = 3.0
  double expected_dpz = K_pz * res;  // 3.0
  double actual_dpz   = p_after(2) - p_before(2);
  check(r, std::abs(actual_dpz - expected_dpz) < 1e-10,
        "T9b p_z correction: expected=" + std::to_string(expected_dpz) +
        " actual=" + std::to_string(actual_dpz));

  // T9c: p_z absolute value = 30 + 3.0 = 33.0
  check(r, std::abs(p_after(2) - 33.0) < 1e-10,
        "T9c p_z absolute: expected=33 got=" + std::to_string(p_after(2)));

  // T9d: velocity, attitude, biases unchanged
  check(r, (v_after  - v_before ).norm() < 1e-12, "T9d velocity changed");
  check(r, (q_after  - q_before ).norm() < 1e-12, "T9d attitude changed");
  check(r, (bg_after - bg_before).norm() < 1e-12, "T9d gyro bias changed");
  check(r, (ba_after - ba_before).norm() < 1e-12, "T9d accel bias changed");

  // T9e: no NaN / Inf in position
  for (int i = 0; i < 3; i++) {
    check(r, std::isfinite(p_after(i)), "T9e p_after(" + std::to_string(i) + ") is NaN/Inf");
  }

  // T9f: covariance symmetry
  Eigen::MatrixXd P_after = StateHelper::get_full_covariance(state);
  double sym_err = (P_after - P_after.transpose()).norm();
  check(r, sym_err < 1e-12, "T9f covariance not symmetric: sym_err=" + std::to_string(sym_err));

  // T9g: covariance diagonal non-negative
  double min_diag = P_after.diagonal().minCoeff();
  check(r, min_diag >= -1e-10, "T9g negative covariance diagonal: min_diag=" + std::to_string(min_diag));

  // T9h: no NaN / Inf in covariance
  check(r, P_after.allFinite(), "T9h covariance contains NaN/Inf");

  return r;
}

// ---------------------------------------------------------------------------
// T10: Test-2 trust region scales one complete gain, preserving direction.
// ---------------------------------------------------------------------------
static TestResult test_T10_uniform_gain_scale(std::mt19937 &rng) {
  TestResult r;
  r.name = "T10 bounded full-gain scale preserves direction and PSD";
  const int N = 15;
  Eigen::MatrixXd P = rand_spd(N, rng);
  Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
  H(5) = 1.0;
  const double R = 4.0;
  const double residual = 12.0;
  const double S = H.dot(P * H) + R;
  const Eigen::VectorXd K = (P * H) / S;
  const Eigen::VectorXd dx_full = K * residual;
  const double scale = 0.23;
  const Eigen::VectorXd K_used = scale * K;
  const Eigen::VectorXd dx_used = K_used * residual;

  check(r, (dx_used - scale * dx_full).norm() < 1e-12,
        "uniformly scaled correction does not equal scale*dx_full");
  const double direction_cos = dx_used.dot(dx_full) /
                               (dx_used.norm() * dx_full.norm());
  check(r, std::abs(direction_cos - 1.0) < 1e-12,
        "bounded correction changed full-state direction");

  StateHelper::josephCovUpdate(P, K_used, H, R);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P);
  check(r, P.allFinite(), "scaled-gain Joseph covariance is non-finite");
  check(r, (P - P.transpose()).norm() < 1e-12,
        "scaled-gain Joseph covariance is asymmetric");
  check(r, es.eigenvalues().minCoeff() >= -1e-8,
        "scaled-gain Joseph covariance is not PSD");
  return r;
}

// ---------------------------------------------------------------------------
// T11: Invalid checked update must not partially modify P or nominal state.
// ---------------------------------------------------------------------------
static TestResult test_T11_checked_update_transaction(std::mt19937 &rng) {
  (void)rng;
  TestResult r;
  r.name = "T11 checked Joseph numerical rejection is transactional";
  VioManagerOptions params;
  params.state_options.num_cameras = 0;
  params.state_options.max_clone_size = 0;
  params.state_options.max_slam_features = 0;
  auto state = std::make_shared<State>(params.state_options);
  const Eigen::MatrixXd P_before = StateHelper::get_full_covariance(state);
  const Eigen::Vector3d p_before = state->_imu->pos();
  const int N = (int)P_before.rows();
  Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
  H(state->_imu->p()->id() + 2) = 1.0;
  Eigen::VectorXd K = Eigen::VectorXd::Zero(N);
  K(state->_imu->p()->id() + 2) =
      std::numeric_limits<double>::quiet_NaN();
  StateHelper::JosephUpdateHealth health;
  const bool applied = StateHelper::EKFUpdateJosephChecked(
      state, K, H, 1.0, 3.0, true, &health);
  const Eigen::MatrixXd P_after = StateHelper::get_full_covariance(state);
  check(r, !applied, "non-finite gain was unexpectedly applied");
  check(r, (P_after - P_before).norm() == 0.0,
        "covariance changed after rejected checked update");
  check(r, (state->_imu->pos() - p_before).norm() == 0.0,
        "state changed after rejected checked update");
  return r;
}

// ===========================================================================
// NASA/Lear measurement-underweighting tests (T12–T19)
//
// These exercise StateHelper::computeLearUnderweightGain — the exact shared
// helper VioManager's NASA_LEAR coupled mode uses — and its consistency with
// the Joseph covariance primitive.  Primary source: NASA Navigation Filter
// Best Practices, NTRS 20180003657 Eq. 4.36/4.38 (Lear's method).
//   M     = P H                      (gain numerator)
//   q     = H' P H                   (prior measurement-space variance)
//   W_U   = (1 + beta) q + R         (Eq. 4.38 effective innovation variance)
//   R_eff = R + beta q               (Eq. 4.36 additive residual noise)
//   K_U   = M / W_U
//   P+    = (I - K_U H') P (I - K_U H')' + K_U R_eff K_U' = P - M M' / W_U
// ===========================================================================

// Standard optimal gain K = P H / (H'PH + R).
static Eigen::VectorXd standard_gain(const Eigen::MatrixXd &P, const Eigen::VectorXd &H, double R) {
  const double S = H.dot(P * H) + R;
  return (P * H) / S;
}

// Build an SPD "arrow" covariance: diagonal d, with only the measured index
// (mz) coupled to the others via correlation coefficients rho[i].  SPD as long
// as sum_i rho[i]^2 < 1.  Returns P; sets M=P*e_mz implicitly (= column mz).
static Eigen::MatrixXd arrow_spd(int N, int mz, const Eigen::VectorXd &d,
                                 const Eigen::VectorXd &rho) {
  Eigen::MatrixXd P = d.asDiagonal();
  for (int i = 0; i < N; i++) {
    if (i == mz) continue;
    const double c = rho(i) * std::sqrt(d(i) * d(mz));
    P(i, mz) = c;
    P(mz, i) = c;
  }
  return P;
}

// ---------------------------------------------------------------------------
// T12: beta=0 reduces NASA gain/covariance exactly to the standard update.
// ---------------------------------------------------------------------------
static TestResult test_T12_nasa_beta0_equivalence(std::mt19937 &rng) {
  TestResult r;
  r.name = "T12 NASA beta=0 == standard full-gain state + covariance";
  const int N = 10;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    // Mix of GPS-Z-like single-entry H and general dense H.
    if (trial % 2 == 0) {
      H(2) = 1.0;
    } else {
      for (int i = 0; i < N; i++) H(i) = ud(rng);
    }
    const double R = 0.5 + std::uniform_real_distribution<double>(0.0, 2.0)(rng);
    const Eigen::VectorXd M = P * H;
    const double q = H.dot(M);
    const Eigen::VectorXd K_std = M / (q + R);

    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, /*beta=*/0.0, R_eff, W_U);

    check(r, (K_U - K_std).norm() < 1e-12,
          "trial=" + std::to_string(trial) + " K_U != K_std: " + std::to_string((K_U - K_std).norm()));
    check(r, std::abs(R_eff - R) < 1e-12, "R_eff != R at beta=0");
    check(r, std::abs(W_U - (q + R)) < 1e-9, "W_U != q+R at beta=0");

    // Covariance also identical.
    Eigen::MatrixXd P_nasa = P;
    StateHelper::josephCovUpdate(P_nasa, K_U, H, R_eff);
    Eigen::MatrixXd P_std = P;
    StateHelper::josephCovUpdate(P_std, K_std, H, R);
    check(r, (P_nasa - P_std).norm() < 1e-12,
          "trial=" + std::to_string(trial) + " P_nasa != P_std at beta=0");
  }
  return r;
}

// ---------------------------------------------------------------------------
// T13: beta>0 strictly reduces |K| while preserving the full gain direction.
// ---------------------------------------------------------------------------
static TestResult test_T13_nasa_gain_reduction(std::mt19937 &rng) {
  TestResult r;
  r.name = "T13 NASA beta>0 reduces |K_U|<|K_std|, direction unchanged";
  const int N = 12;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    H(2) = 1.0;  // GPS-Z model
    const double R = 0.5 + std::uniform_real_distribution<double>(0.0, 2.0)(rng);
    const Eigen::VectorXd M = P * H;
    const double q = H.dot(M);
    const Eigen::VectorXd K_std = M / (q + R);
    const double beta = 0.1 + std::abs(ud(rng));  // > 0

    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);

    check(r, K_U.norm() < K_std.norm(),
          "trial=" + std::to_string(trial) + " |K_U| not < |K_std|");
    const double cos = K_U.dot(K_std) / (K_U.norm() * K_std.norm());
    check(r, std::abs(cos - 1.0) < 1e-12,
          "trial=" + std::to_string(trial) + " gain direction changed cos=" + std::to_string(cos));
    // Exact scaling factor gamma = (q+R)/((1+beta)q+R).
    const double gamma = (q + R) / ((1.0 + beta) * q + R);
    check(r, (K_U - gamma * K_std).norm() < 1e-12, "K_U != gamma*K_std");
  }
  return r;
}

// ---------------------------------------------------------------------------
// T14: R_eff/W_U consistency — K_U = M/((1+beta)q+R) = M/(q+R_eff).
// ---------------------------------------------------------------------------
static TestResult test_T14_nasa_reff_consistency(std::mt19937 &rng) {
  TestResult r;
  r.name = "T14 NASA R_eff consistency: M/((1+b)q+R) == M/(q+R_eff)";
  const int N = 9;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    for (int i = 0; i < N; i++) H(i) = ud(rng);
    const double R = 0.3 + std::abs(ud(rng));
    const double beta = std::abs(ud(rng));
    const Eigen::VectorXd M = P * H;
    const double q = H.dot(M);

    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);

    check(r, std::abs(R_eff - (R + beta * q)) < 1e-12, "R_eff != R + beta*q");
    check(r, std::abs(W_U - ((1.0 + beta) * q + R)) < 1e-9, "W_U != (1+beta)q+R");
    const Eigen::VectorXd K_reff = M / (q + R_eff);
    check(r, (K_U - K_reff).norm() < 1e-12,
          "trial=" + std::to_string(trial) + " K_U != M/(q+R_eff)");
  }
  return r;
}

// ---------------------------------------------------------------------------
// T15: Joseph consistency — covariance from (K_U, R_eff) equals the scalar
// rank-one form P - M M'/W_U, i.e. state and P use the same K_U.
// ---------------------------------------------------------------------------
static TestResult test_T15_nasa_joseph_rank_one(std::mt19937 &rng) {
  TestResult r;
  r.name = "T15 NASA Joseph(K_U,R_eff) == P - M M'/W_U";
  const int N = 10;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (int trial = 0; trial < 20; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng);
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    H(3) = 1.0;
    const double R = 0.5 + std::abs(ud(rng));
    const double beta = std::abs(ud(rng));
    const Eigen::VectorXd M = P * H;
    const double q = H.dot(M);

    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);

    Eigen::MatrixXd P_joseph = P;
    StateHelper::josephCovUpdate(P_joseph, K_U, H, R_eff);
    Eigen::MatrixXd P_rank1 = P - (M * M.transpose()) / W_U;
    P_rank1 = 0.5 * (P_rank1 + P_rank1.transpose());
    check(r, (P_joseph - P_rank1).norm() < 1e-9,
          "trial=" + std::to_string(trial) + " Joseph != rank-one: " +
              std::to_string((P_joseph - P_rank1).norm()));
  }
  return r;
}

// ---------------------------------------------------------------------------
// T16: cross-covariance — GPS-Z correction propagates to XY/velocity/attitude/
// bias with the sign of P[:,pz]; zero cross-covariance => zero correction.
// ---------------------------------------------------------------------------
static TestResult test_T16_nasa_cross_covariance(std::mt19937 &rng) {
  (void)rng;
  TestResult r;
  r.name = "T16 NASA cross-covariance corrects XY/v/att/bias with correct signs";
  const int N = 15;
  const int roll = 0, px = 3, py = 4, pz = 5, vx = 6, bgx = 10, bax = 13;
  Eigen::VectorXd d = Eigen::VectorXd::Constant(N, 1.0);
  d(pz) = 4.0;
  Eigen::VectorXd rho = Eigen::VectorXd::Zero(N);
  rho(px) = 0.3;    // P_xz > 0
  rho(py) = -0.3;   // P_yz < 0
  rho(vx) = 0.2;    // P_vx,z != 0
  rho(roll) = 0.2;  // P_roll,z != 0
  rho(bax) = -0.2;  // P_bax,z != 0
  // rho(bgx) stays 0 -> zero cross-covariance
  Eigen::MatrixXd P = arrow_spd(N, pz, d, rho);

  // SPD sanity (sum rho^2 < 1).
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es0(P);
  check(r, es0.eigenvalues().minCoeff() > 0.0, "constructed P not SPD");

  Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
  H(pz) = 1.0;
  const double R = 2.0;
  const double beta = 0.5;
  const double res = 3.0;  // positive residual (GPS above VIO)
  const Eigen::VectorXd M = P * H;
  const double q = H.dot(M);
  check(r, std::abs(q - d(pz)) < 1e-12, "q != P_zz for GPS-Z model");

  double R_eff = -1.0, W_U = -1.0;
  const Eigen::VectorXd K_U =
      StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);
  const Eigen::VectorXd dx = K_U * res;

  // Sign checks (res>0, W_U>0): sign(dx_i) == sign(P_i,z) == sign(rho_i).
  check(r, dx(px) > 0.0, "T16 P_xz>0 did not produce +x correction");
  check(r, dx(py) < 0.0, "T16 P_yz<0 did not produce -y correction");
  check(r, dx(vx) > 0.0, "T16 P_vx,z>0 did not produce +vx correction");
  check(r, dx(roll) > 0.0, "T16 P_roll,z>0 did not produce +roll correction");
  check(r, dx(bax) < 0.0, "T16 P_bax,z<0 did not produce -bax correction");
  check(r, dx(pz) > 0.0, "T16 measured p_z not corrected");
  // Zero cross-covariance -> exactly zero correction.
  check(r, std::abs(dx(bgx)) < 1e-15, "T16 zero-cross-cov state was corrected");
  check(r, std::abs(K_U(bgx)) < 1e-15, "T16 zero-cross-cov gain nonzero");

  // Covariance remains PSD after the coupled NASA update.
  Eigen::MatrixXd P_after = P;
  StateHelper::josephCovUpdate(P_after, K_U, H, R_eff);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_after);
  check(r, es.eigenvalues().minCoeff() >= -1e-9, "T16 NASA covariance not PSD");
  return r;
}

// ---------------------------------------------------------------------------
// T17: increasing beta monotonically shrinks the increment and weakens the
// covariance contraction, without changing direction.
// ---------------------------------------------------------------------------
static TestResult test_T17_nasa_beta_monotonic(std::mt19937 &rng) {
  TestResult r;
  r.name = "T17 NASA increasing beta: |dx| down, contraction down, direction fixed";
  const int N = 12;
  Eigen::MatrixXd P = rand_spd(N, rng);
  Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
  H(4) = 1.0;
  const double R = 1.5;
  const double res = 5.0;
  const Eigen::VectorXd M = P * H;
  const double q = H.dot(M);

  const std::vector<double> betas = {0.0, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0};
  double prev_dx_norm = std::numeric_limits<double>::infinity();
  double prev_contraction = std::numeric_limits<double>::infinity();
  Eigen::VectorXd K0 = standard_gain(P, H, R);
  for (double beta : betas) {
    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);
    const Eigen::VectorXd dx = K_U * res;
    const double dx_norm = dx.norm();
    check(r, dx_norm <= prev_dx_norm + 1e-12,
          "beta=" + std::to_string(beta) + " |dx| not monotonically decreasing");
    // direction preserved vs standard
    const double cos = K_U.dot(K0) / (K_U.norm() * K0.norm());
    check(r, std::abs(cos - 1.0) < 1e-12, "beta=" + std::to_string(beta) + " direction changed");
    // covariance contraction = trace(P - P_NASA) = M'M / W_U
    Eigen::MatrixXd P_after = P;
    StateHelper::josephCovUpdate(P_after, K_U, H, R_eff);
    const double contraction = (P - P_after).trace();
    check(r, contraction <= prev_contraction + 1e-12,
          "beta=" + std::to_string(beta) + " covariance contraction not decreasing");
    check(r, contraction >= -1e-9, "beta=" + std::to_string(beta) + " negative contraction (P grew)");
    prev_dx_norm = dx_norm;
    prev_contraction = contraction;
  }
  return r;
}

// ---------------------------------------------------------------------------
// T18: random trials — P_NASA PSD and Loewner order P_std <= P_NASA <= P_prior.
// ---------------------------------------------------------------------------
static TestResult test_T18_nasa_psd_loewner(std::mt19937 &rng) {
  TestResult r;
  r.name = "T18 NASA PSD + Loewner: P_std <= P_NASA <= P_prior (100 trials)";
  const int N = 8;
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  int fail = 0;
  for (int trial = 0; trial < 100; trial++) {
    Eigen::MatrixXd P = rand_spd(N, rng, 0.5 + std::abs(ud(rng)));
    Eigen::VectorXd H = Eigen::VectorXd::Zero(N);
    int nnz = 1 + (trial % 3);
    for (int k = 0; k < nnz; k++) H((trial * 5 + k * 2) % N) = ud(rng);
    const double R = 0.2 + std::abs(ud(rng));
    const double beta = std::abs(ud(rng)) * 2.0;
    const Eigen::VectorXd M = P * H;
    const double qv = H.dot(M);
    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U = StateHelper::computeLearUnderweightGain(M, qv, R, beta, R_eff, W_U);
    const Eigen::VectorXd K_std = M / (qv + R);

    Eigen::MatrixXd P_nasa = P;
    StateHelper::josephCovUpdate(P_nasa, K_U, H, R_eff);
    Eigen::MatrixXd P_std = P;
    StateHelper::josephCovUpdate(P_std, K_std, H, R);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_nasa(P_nasa);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_hi(P - P_nasa);     // >= 0
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es_lo(P_nasa - P_std); // >= 0
    const double tol = 1e-8 * std::max(1.0, P.diagonal().maxCoeff());
    if (es_nasa.eigenvalues().minCoeff() < -tol ||
        es_hi.eigenvalues().minCoeff() < -tol ||
        es_lo.eigenvalues().minCoeff() < -tol) {
      fail++;
      r.detail += "  trial=" + std::to_string(trial) +
                  " nasa_min=" + std::to_string(es_nasa.eigenvalues().minCoeff()) +
                  " hi_min=" + std::to_string(es_hi.eigenvalues().minCoeff()) +
                  " lo_min=" + std::to_string(es_lo.eigenvalues().minCoeff()) + "\n";
    }
  }
  check(r, fail == 0, std::to_string(fail) + "/100 trials violated PSD/Loewner");
  return r;
}

// ---------------------------------------------------------------------------
// T19: real State injection — EKFUpdateJosephChecked with a NASA K_U applies
// dx = K_U*res through the typed/JPL path; beta>0 reduces the p_z correction
// relative to beta=0; masked states stay put.
// ---------------------------------------------------------------------------
static TestResult test_T19_nasa_real_state_injection(std::mt19937 &rng) {
  (void)rng;
  TestResult r;
  r.name = "T19 NASA real-state injection via EKFUpdateJosephChecked";

  auto build_state = []() {
    VioManagerOptions params;
    params.state_options.num_cameras = 0;
    params.state_options.max_clone_size = 0;
    params.state_options.max_slam_features = 0;
    return std::make_shared<State>(params.state_options);
  };

  // beta=0 reference correction.
  auto run_once = [&](double beta, double &dpz_out, std::shared_ptr<State> &st) {
    st = build_state();
    Eigen::VectorXd imu_val = Eigen::VectorXd::Zero(16);
    imu_val(3) = 1.0;    // q_w
    imu_val(6) = 30.0;   // p_z
    st->_imu->set_value(imu_val);
    const int N = (int)StateHelper::get_full_covariance(st).rows();
    const int p_id = st->_imu->p()->id();
    Eigen::VectorXd H_full = Eigen::VectorXd::Zero(N);
    H_full(p_id + 2) = 1.0;
    const Eigen::MatrixXd P = StateHelper::get_full_covariance(st);
    const Eigen::VectorXd M = P * H_full;
    const double q = H_full.dot(M);
    const double R = 1.0;
    const double res = 6.0;
    double R_eff = -1.0, W_U = -1.0;
    const Eigen::VectorXd K_U =
        StateHelper::computeLearUnderweightGain(M, q, R, beta, R_eff, W_U);
    const double p_z_before = st->_imu->pos()(2);
    StateHelper::JosephUpdateHealth health;
    const bool ok = StateHelper::EKFUpdateJosephChecked(st, K_U, H_full, R_eff, res, true, &health);
    check(r, ok, "EKFUpdateJosephChecked rejected a valid NASA update (beta=" + std::to_string(beta) + ")");
    check(r, health.psd, "NASA covariance not PSD (beta=" + std::to_string(beta) + ")");
    dpz_out = st->_imu->pos()(2) - p_z_before;
    // expected exact correction = K_U(pz)*res
    const double expected = K_U(p_id + 2) * res;
    check(r, std::abs(dpz_out - expected) < 1e-10,
          "p_z correction != K_U*res (beta=" + std::to_string(beta) + ")");
  };

  double dpz0 = 0.0, dpzb = 0.0;
  std::shared_ptr<State> s0, sb;
  run_once(0.0, dpz0, s0);
  run_once(0.8, dpzb, sb);

  // beta>0 reduces the magnitude of the p_z correction.
  check(r, std::abs(dpzb) < std::abs(dpz0),
        "T19 beta>0 did not reduce p_z correction: |dpzb|=" + std::to_string(std::abs(dpzb)) +
            " |dpz0|=" + std::to_string(std::abs(dpz0)));

  // Masked states (p_x, p_y, velocity, attitude, biases) unchanged for diagonal P.
  check(r, std::abs(sb->_imu->pos()(0)) < 1e-12, "T19 p_x changed");
  check(r, std::abs(sb->_imu->pos()(1)) < 1e-12, "T19 p_y changed");
  check(r, sb->_imu->vel().norm() < 1e-12, "T19 velocity changed");
  check(r, sb->_imu->bias_a().norm() < 1e-12, "T19 accel bias changed");
  check(r, sb->_imu->bias_g().norm() < 1e-12, "T19 gyro bias changed");
  return r;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;

  std::mt19937 rng(42);

  std::vector<TestResult> results;
  results.push_back(test_T1_full_matches_direct(rng));
  results.push_back(test_T2_full_matches_standard_ekf(rng));
  results.push_back(test_T3_masked_matches_direct(rng));
  results.push_back(test_T4_symmetry(rng));
  results.push_back(test_T5_diag_nonneg(rng));
  results.push_back(test_T6_psd_random(rng));
  results.push_back(test_T7_T8_state_update(rng));
  results.push_back(test_T_masked_block_unchanged(rng));
  results.push_back(test_T9_real_imu_nonzero(rng));
  results.push_back(test_T10_uniform_gain_scale(rng));
  results.push_back(test_T11_checked_update_transaction(rng));
  results.push_back(test_T12_nasa_beta0_equivalence(rng));
  results.push_back(test_T13_nasa_gain_reduction(rng));
  results.push_back(test_T14_nasa_reff_consistency(rng));
  results.push_back(test_T15_nasa_joseph_rank_one(rng));
  results.push_back(test_T16_nasa_cross_covariance(rng));
  results.push_back(test_T17_nasa_beta_monotonic(rng));
  results.push_back(test_T18_nasa_psd_loewner(rng));
  results.push_back(test_T19_nasa_real_state_injection(rng));

  int passed = 0, failed = 0;
  std::cout << "\n=== josephCovUpdate / EKFUpdateJoseph Gate 1 Tests ===\n";
  for (const auto &r : results) {
    if (r.passed) {
      std::cout << "  [PASS] " << r.name << "\n";
      passed++;
    } else {
      std::cout << "  [FAIL] " << r.name << "\n" << r.detail;
      failed++;
    }
  }
  std::cout << "\n" << passed << " passed, " << failed << " failed\n";
  return (failed == 0) ? 0 : 1;
}
