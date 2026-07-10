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

#ifndef OV_MSCKF_STATE_HELPER_H
#define OV_MSCKF_STATE_HELPER_H

#include <Eigen/Eigen>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ov_type {
class Type;
} // namespace ov_type

// Callback type used by StateHelper::initialize() to optionally apply a
// pre-chi2 OC projection to the updating (Hup) portion before the chi-square
// gate.  Signature: (H, H_order, H_id) -> projected_H.
// When nullptr the function behaves as before (no pre-chi2 projection).
using VisualOcFn = std::function<Eigen::MatrixXd(
    const Eigen::MatrixXd &,
    const std::vector<std::shared_ptr<ov_type::Type>> &,
    const std::vector<int> &)>;

namespace ov_msckf {

class State;

/**
 * @brief Helper which manipulates the State and its covariance.
 *
 * In general, this class has all the core logic for an Extended Kalman Filter (EKF)-based system.
 * This has all functions that change the covariance along with addition and removing elements from the state.
 * All functions here are static, and thus are self-contained so that in the future multiple states could be tracked and updated.
 * We recommend you look directly at the code for this class for clarity on what exactly we are doing in each and the matching documentation
 * pages.
 */
class StateHelper {

public:
  enum class VisualYawUpdateMode {
    ORIGINAL                         =  0, // FEJ Jacobians, no OC projection
    PER_BLOCK_SCALE                  =  1, // internal "no yaw update" scale path
    GLOBAL_YAW_OC_PROJECTION         =  2, // post-chi2 OC, current gauge
    GLOBAL_YAW_OC_FEJ_PROJECTION     = 10, // post-chi2 OC, FEJ gauge
    // (msckf2_0 / oc_prechi2 is dispatched via VisualObservabilityPolicy::is_prechi2_mode_string, not this enum)
  };

  struct YawDxProjectionDiag {
    bool valid = false;
    VisualYawUpdateMode mode = VisualYawUpdateMode::ORIGINAL;
    double dx_yaw_before_projection_deg = 0.0;
    double dx_yaw_after_projection_deg = 0.0;
  };

  /**
   * @brief Performs EKF propagation of the state covariance.
   *
   * The mean of the state should already have been propagated, thus just moves the covariance forward in time.
   * The new states that we are propagating the old covariance into, should be **contiguous** in memory.
   * The user only needs to specify the sub-variables that this block is a function of.
   * \f[
   * \tilde{\mathbf{x}}' =
   * \begin{bmatrix}
   * \boldsymbol\Phi_1 &
   * \boldsymbol\Phi_2 &
   * \boldsymbol\Phi_3
   * \end{bmatrix}
   * \begin{bmatrix}
   * \tilde{\mathbf{x}}_1 \\
   * \tilde{\mathbf{x}}_2 \\
   * \tilde{\mathbf{x}}_3
   * \end{bmatrix}
   * +
   * \mathbf{n}
   * \f]
   *
   * @param state Pointer to state
   * @param order_NEW Contiguous variables that have evolved according to this state transition
   * @param order_OLD Variable ordering used in the state transition
   * @param Phi State transition matrix (size order_NEW by size order_OLD)
   * @param Q Additive state propagation noise matrix (size order_NEW by size order_NEW)
   */
  static void EKFPropagation(std::shared_ptr<State> state, const std::vector<std::shared_ptr<ov_type::Type>> &order_NEW,
                             const std::vector<std::shared_ptr<ov_type::Type>> &order_OLD, const Eigen::MatrixXd &Phi,
                             const Eigen::MatrixXd &Q);

  /**
   * @brief Performs EKF update of the state (see @ref linear-meas page)
   * @param state Pointer to state
   * @param H_order Variable ordering used in the compressed Jacobian
   * @param H Condensed Jacobian of updating measurement
   * @param res Residual of updating measurement
   * @param R Updating measurement covariance
   */
  static void EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<ov_type::Type>> &H_order, const Eigen::MatrixXd &H,
                        const Eigen::VectorXd &res, const Eigen::MatrixXd &R,
                        VisualYawUpdateMode visual_yaw_update_mode = VisualYawUpdateMode::ORIGINAL,
                        double visual_yaw_update_scale = 1.0,
                        double visual_global_yaw_oc_alpha = 0.0,
                        double visual_bgz_update_scale = 1.0);

  struct UpdateDiagnostics {
    bool valid = false;
    int rows = 0;
    int cols = 0;
    double residual_norm = std::numeric_limits<double>::quiet_NaN();
    double H_norm = std::numeric_limits<double>::quiet_NaN();
    double whitened_H_norm = std::numeric_limits<double>::quiet_NaN();
    double H_yaw_col_norm = std::numeric_limits<double>::quiet_NaN();
    double H_bgz_col_norm = std::numeric_limits<double>::quiet_NaN();
    double H_pos_col_norm = std::numeric_limits<double>::quiet_NaN();
    double H_landmark_norm = std::numeric_limits<double>::quiet_NaN();
    double H_other_norm = std::numeric_limits<double>::quiet_NaN();
    double S_cond = std::numeric_limits<double>::quiet_NaN();
    double S_min_eig = std::numeric_limits<double>::quiet_NaN();
    double S_max_eig = std::numeric_limits<double>::quiet_NaN();
    double HPH_trace = std::numeric_limits<double>::quiet_NaN();
    double R_trace = std::numeric_limits<double>::quiet_NaN();
    double HPH_over_R = std::numeric_limits<double>::quiet_NaN();
    double dx_norm = std::numeric_limits<double>::quiet_NaN();
    double dx_yaw_deg = std::numeric_limits<double>::quiet_NaN();
    double dx_bgz = std::numeric_limits<double>::quiet_NaN();
    double dx_pos_norm = std::numeric_limits<double>::quiet_NaN();
    double dx_landmark_norm = std::numeric_limits<double>::quiet_NaN();
    double K_yaw_row_norm = std::numeric_limits<double>::quiet_NaN();
    double K_bgz_row_norm = std::numeric_limits<double>::quiet_NaN();
    double K_pos_row_norm = std::numeric_limits<double>::quiet_NaN();
    double K_landmark_row_norm = std::numeric_limits<double>::quiet_NaN();
    double P_yaw_var = std::numeric_limits<double>::quiet_NaN();
    double P_bgz_var = std::numeric_limits<double>::quiet_NaN();
    double P_pos_trace = std::numeric_limits<double>::quiet_NaN();
    double corr_yaw_bgz = std::numeric_limits<double>::quiet_NaN();
    double corr_yaw_px = std::numeric_limits<double>::quiet_NaN();
    double corr_yaw_py = std::numeric_limits<double>::quiet_NaN();
    double pose_landmark_cov_norm = std::numeric_limits<double>::quiet_NaN();
  };

  static UpdateDiagnostics compute_update_diagnostics(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
      const Eigen::MatrixXd &R,
      VisualYawUpdateMode visual_yaw_update_mode = VisualYawUpdateMode::ORIGINAL,
      double visual_yaw_update_scale = 1.0,
      double visual_global_yaw_oc_alpha = 0.0,
      double visual_bgz_update_scale = 1.0);

  static Eigen::VectorXd compute_update_dx(std::shared_ptr<State> state,
                                           const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
                                           const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
                                           const Eigen::MatrixXd &R,
                                           VisualYawUpdateMode visual_yaw_update_mode = VisualYawUpdateMode::ORIGINAL,
                                           double visual_yaw_update_scale = 1.0,
                                           double visual_global_yaw_oc_alpha = 0.0,
                                           double visual_bgz_update_scale = 1.0);

  static double yaw_delta_from_full_dx_deg(std::shared_ptr<State> state, const Eigen::VectorXd &dx);

  static void reset_last_yaw_dx_projection_diag();

  static YawDxProjectionDiag get_last_yaw_dx_projection_diag();

  /**
   * @brief This will set the initial covaraince of the specified state elements.
   * Will also ensure that proper cross-covariances are inserted.
   * @param state Pointer to state
   * @param covariance The covariance of the system state
   * @param order Order of the covariance matrix
   */
  static void set_initial_covariance(std::shared_ptr<State> state, const Eigen::MatrixXd &covariance,
                                     const std::vector<std::shared_ptr<ov_type::Type>> &order);

  /**
   * @brief Inject noise into the position-z element of the state covariance.
   * Used to maintain a minimum P_zz floor for GPS altitude fusion.
   * @param state Pointer to state
   * @param noise Amount of noise (variance) to add to P_zz diagonal
   */
  static void inject_pz_noise(std::shared_ptr<State> state, double noise);

  /**
   * @brief Pure-math Joseph-form covariance update with arbitrary gain K.
   *
   * Ported from PX4-Autopilot ekf_helper.cpp:measurementUpdate lines 1127-1171,
   * commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909.
   * BSD-3 License, Copyright (c) 2012-2025 PX4 Development Team.
   *
   * Computes: P = (I - K*H^T)*P*(I - K*H^T)^T + K*R*K^T
   * where H and K are N×1 column vectors in the global state space.
   *
   * Valid for any K (optimal or masked sub-optimal gain).
   * Symmetric enforcement: P(j,i) = P(i,j) for all i,j after update.
   * PSD-preserving for any K when R > 0.
   *
   * No State object required. Operates on plain Eigen matrices.
   * Use EKFUpdateJoseph() for the State-aware version.
   *
   * @param P  Covariance matrix (N×N), modified in-place
   * @param K  Kalman gain column vector (N×1); zero entries skip that state
   * @param H  Jacobian column vector (N×1) in global state space
   * @param R  Scalar measurement noise variance (> 0)
   */
  static void josephCovUpdate(Eigen::MatrixXd &P,
                               const Eigen::VectorXd &K,
                               const Eigen::VectorXd &H,
                               double R);

  /**
   * @brief State-aware Joseph-form update with pre-computed, possibly masked, gain.
   *
   * Applies josephCovUpdate() then corrects state variables via var->update(dx)
   * for all variables with non-zero K_full entries.
   * Caller is responsible for zeroing K_full entries for states that should
   * not receive corrections (masking).
   *
   * Ported from PX4-Autopilot ekf_helper.cpp:measurementUpdate + fuseHaglRng,
   * commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909.
   * BSD-3 License, Copyright (c) 2012-2025 PX4 Development Team.
   *
   * @param state   State to update (covariance + state vector)
   * @param K_full  Pre-computed (possibly masked) gain, N×1 in global space
   * @param H_full  Jacobian, N×1 in global space
   * @param R       Scalar measurement noise variance
   * @param res     Scalar residual (measurement - predicted)
   */
  static void EKFUpdateJoseph(std::shared_ptr<State> state,
                               const Eigen::VectorXd &K_full,
                               const Eigen::VectorXd &H_full,
                               double R, double res);

  /// Numerical health returned by EKFUpdateJosephChecked().
  struct JosephUpdateHealth {
    bool finite = false;
    bool symmetric = false;
    bool nonnegative_diagonal = false;
    bool psd_checked = false;
    bool psd = false;
    double symmetry_error = std::numeric_limits<double>::quiet_NaN();
    double min_diagonal = std::numeric_limits<double>::quiet_NaN();
    double min_ldlt_diagonal = std::numeric_limits<double>::quiet_NaN();
  };

  /**
   * @brief Transactional Joseph update with covariance health checks.
   *
   * The candidate covariance and state increment are formed first. If they are
   * non-finite, asymmetric, have a materially negative diagonal, or fail the
   * requested LDLT PSD check, neither the covariance nor the nominal state is
   * modified.
   *
   * @param check_psd Run a full LDLT PSD check. This is caller-controlled
   * because it is O(N^3), unlike the always-on finite/diagonal checks.
   */
  static bool EKFUpdateJosephChecked(std::shared_ptr<State> state,
                                      const Eigen::VectorXd &K_full,
                                      const Eigen::VectorXd &H_full,
                                      double R, double res,
                                      bool check_psd,
                                      JosephUpdateHealth *health = nullptr);

  /**
   * @brief NASA/Lear scalar-measurement underweighting gain.
   *
   * Implements Lear's method from NASA *Navigation Filter Best Practices*
   * (NTRS 20180003657 Eq. 4.36 / 4.38; NTRS 20250002787 Eq. 5.36). The
   * underweight factor is a fraction @p beta of the mapped prior state
   * uncertainty `H' P H`, not a fixed scaling of the measurement noise.
   *
   * For a scalar measurement with gain numerator `M = P * H_full`, prior
   * measurement-space variance `q = H_full' * P * H_full` (>= 0), measurement
   * noise `R > 0`, and underweight coefficient `beta >= 0` (already gated by the
   * caller — pass 0 to disable):
   * @code
   *   W_U   = (1 + beta) * q + R        // Eq. 4.38
   *   R_eff = R + beta * q              // additive residual noise, U = beta*q (Eq. 4.36)
   *   K_U   = M / W_U
   * @endcode
   * Passing the matched pair `(K_U, R_eff)` to EKFUpdateJosephChecked() yields
   * the consistent Joseph update `P+ = (I-K_U H')P(I-K_U H')' + K_U R_eff K_U'`.
   * With `beta = 0` this reduces *exactly* to the standard full-gain update
   * (`K_U = M/(q+R)`, `R_eff = R`).
   *
   * @param gain_numerator  M = P * H_full (length n)
   * @param hph             q = H_full' * P * H_full (clamped to >= 0 internally)
   * @param R               Scalar measurement noise (> 0)
   * @param beta            Underweight coefficient (>= 0; clamped internally)
   * @param[out] R_eff_out  Effective residual noise R + beta*q (use in Joseph)
   * @param[out] W_U_out    Effective innovation variance (1+beta)*q + R
   * @return Underweighted Kalman gain K_U (zero vector on a degenerate W_U)
   */
  static Eigen::VectorXd computeLearUnderweightGain(const Eigen::VectorXd &gain_numerator,
                                                    double hph, double R, double beta,
                                                    double &R_eff_out, double &W_U_out);

  /**
   * @brief Convenience wrapper: compute optimal K, mask to a single state DOF, call EKFUpdateJoseph.
   *
   * Ported from PX4-Autopilot fuseHaglRng (commit d5a0ca1bbc5e932bba5dc5b2bb58e0e0147f9909):
   *   K = P * H_full / S  (optimal Kalman gain)
   *   K entries for all DOFs except (active_var, active_dof) zeroed (masking)
   *   EKFUpdateJoseph(state, K_masked, H_full, R, res)  (Joseph form, PX4-style)
   *
   * Scalar measurement only (H is 1×n, res is 1×1, R is 1×1).
   *
   * @param state       State to update
   * @param H_order     Variable ordering for compressed Jacobian H
   * @param H           Compressed Jacobian (1×n)
   * @param res         Scalar residual (1×1 vector)
   * @param R           Scalar measurement noise (1×1 matrix)
   * @param active_var  Variable whose active_dof-th DOF receives the state correction
   * @param active_dof  Index within active_var (e.g. 2 for p_z in IMU position Vec)
   */
  static void EKFUpdateJosephMasked(std::shared_ptr<State> state,
                                     const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
                                     const Eigen::MatrixXd &H,
                                     const Eigen::VectorXd &res,
                                     const Eigen::MatrixXd &R,
                                     std::shared_ptr<ov_type::Type> active_var,
                                     int active_dof);

  /**
   * @brief For a given set of variables, this will this will calculate a smaller covariance.
   *
   * That only includes the ones specified with all crossterms.
   * Thus the size of the return will be the summed dimension of all the passed variables.
   * Normal use for this is a chi-squared check before update (where you don't need the full covariance).
   *
   * @param state Pointer to state
   * @param small_variables Vector of variables whose marginal covariance is desired
   * @return Marginal covariance of the passed variables
   */
  static Eigen::MatrixXd get_marginal_covariance(std::shared_ptr<State> state,
                                                 const std::vector<std::shared_ptr<ov_type::Type>> &small_variables);

  /**
   * @brief This gets the full covariance matrix.
   *
   * Should only be used during simulation as operations on this covariance will be slow.
   * This will return a copy, so this cannot be used to change the covariance by design.
   * Please use the other interface functions in the StateHelper to progamatically change to covariance.
   *
   * @param state Pointer to state
   * @return Covariance of current state
   */
  static Eigen::MatrixXd get_full_covariance(std::shared_ptr<State> state);

  /**
   * @brief Marginalizes a variable, properly modifying the ordering/covariances in the state
   *
   * This function can support any Type variable out of the box.
   * Right now the marginalization of a sub-variable/type is not supported.
   * For example if you wanted to just marginalize the orientation of a PoseJPL, that isn't supported.
   * We will first remove the rows and columns corresponding to the type (i.e. do the marginalization).
   * After we update all the type ids so that they take into account that the covariance has shrunk in parts of it.
   *
   * @param state Pointer to state
   * @param marg Pointer to variable to marginalize
   */
  static void marginalize(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> marg);

  /**
   * @brief Clones "variable to clone" and places it at end of covariance
   * @param state Pointer to state
   * @param variable_to_clone Pointer to variable that will be cloned
   */
  static std::shared_ptr<ov_type::Type> clone(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> variable_to_clone);

  /**
   * @brief Initializes new variable into covariance.
   *
   * Uses Givens to separate into updating and initializing systems (therefore system must be fed as isotropic).
   * If you are not isotropic first whiten your system (TODO: we should add a helper function to do this).
   * If your H_L Jacobian is already directly invertable, the just call the initialize_invertible() instead of this function.
   * Please refer to @ref update-delay page for detailed derivation.
   *
   * @param state Pointer to state
   * @param new_variable Pointer to variable to be initialized
   * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
   * @param H_R Jacobian of initializing measurements wrt variables in H_order
   * @param H_L Jacobian of initializing measurements wrt new variable
   * @param R Covariance of initializing measurements (isotropic)
   * @param res Residual of initializing measurements
   * @param chi_2_mult Value we should multiply the chi2 threshold by (larger means it will be accepted more measurements)
   */
  static bool initialize(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> new_variable,
                         const std::vector<std::shared_ptr<ov_type::Type>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L,
                         Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult,
                         VisualYawUpdateMode visual_yaw_update_mode = VisualYawUpdateMode::ORIGINAL,
                         double visual_yaw_update_scale = 1.0,
                         double visual_global_yaw_oc_alpha = 0.0,
                         VisualOcFn oc_fn = nullptr);

  /**
   * @brief Initializes new variable into covariance (H_L must be invertible)
   *
   * Please refer to @ref update-delay page for detailed derivation.
   * This is just the update assuming that H_L is invertable (and thus square) and isotropic noise.
   *
   * @param state Pointer to state
   * @param new_variable Pointer to variable to be initialized
   * @param H_order Vector of pointers in order they are contained in the condensed state Jacobian
   * @param H_R Jacobian of initializing measurements wrt variables in H_order
   * @param H_L Jacobian of initializing measurements wrt new variable (needs to be invertible)
   * @param R Covariance of initializing measurements
   * @param res Residual of initializing measurements
   */
  static void initialize_invertible(std::shared_ptr<State> state, std::shared_ptr<ov_type::Type> new_variable,
                                    const std::vector<std::shared_ptr<ov_type::Type>> &H_order, const Eigen::MatrixXd &H_R,
                                    const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res);

  /**
   * @brief Augment the state with a stochastic copy of the current IMU pose
   *
   * After propagation, normally we augment the state with an new clone that is at the new update timestep.
   * This augmentation clones the IMU pose and adds it to our state's clone map.
   * If we are doing time offset calibration we also make our cloning a function of the time offset.
   * Time offset logic is based on Li and Mourikis @cite Li2014IJRR.
   *
   * We can write the current clone at the true imu base clock time as the
   * follow: \f{align*}{
   * {}^{I_{t+t_d}}_G\bar{q} &= \begin{bmatrix}\frac{1}{2} {}^{I_{t+\hat{t}_d}}\boldsymbol\omega \tilde{t}_d \\
   * 1\end{bmatrix}\otimes{}^{I_{t+\hat{t}_d}}_G\bar{q} \\
   * {}^G\mathbf{p}_{I_{t+t_d}} &= {}^G\mathbf{p}_{I_{t+\hat{t}_d}} + {}^G\mathbf{v}_{I_{t+\hat{t}_d}}\tilde{t}_d
   * \f}
   * where we say that we have propagated our state up to the current estimated true imaging time for the current image,
   * \f${}^{I_{t+\hat{t}_d}}\boldsymbol\omega\f$ is the angular velocity at the end of propagation with biases removed.
   * This is off by some smaller error, so to get to the true imaging time in the imu base clock, we can append some small timeoffset error.
   * Thus the Jacobian in respect to our time offset during our cloning procedure is the following:
   * \f{align*}{
   * \frac{\partial {}^{I_{t+t_d}}_G\tilde{\boldsymbol\theta}}{\partial \tilde{t}_d} &= {}^{I_{t+\hat{t}_d}}\boldsymbol\omega \\
   * \frac{\partial {}^G\tilde{\mathbf{p}}_{I_{t+t_d}}}{\partial \tilde{t}_d} &= {}^G\mathbf{v}_{I_{t+\hat{t}_d}}
   * \f}
   *
   * @param state Pointer to state
   * @param last_w The estimated angular velocity at cloning time (used to estimate imu-cam time offset)
   */
  static void augment_clone(std::shared_ptr<State> state, Eigen::Matrix<double, 3, 1> last_w);

  /**
   * @brief Remove the oldest clone, if we have more then the max clone count!!
   *
   * This will marginalize the clone from our covariance, and remove it from our state.
   * This is mainly a helper function that we can call after each update.
   * It will marginalize the clone specified by State::margtimestep() which should return a clone timestamp.
   *
   * @param state Pointer to state
   */
  static void marginalize_old_clone(std::shared_ptr<State> state);

  /**
   * @brief Marginalize bad SLAM features
   * @param state Pointer to state
   */
  static void marginalize_slam(std::shared_ptr<State> state);

private:
  /**
   * All function in this class should be static.
   * Thus an instance of this class cannot be created.
   */
  StateHelper() {}
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_HELPER_H
