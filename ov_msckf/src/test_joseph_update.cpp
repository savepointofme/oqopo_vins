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
