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
#include <memory>
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
    ORIGINAL = 0,
    PER_BLOCK_SCALE = 1,
    GLOBAL_YAW_OC_PROJECTION = 2,
    CURRENT_ONLY_SCALE = 3,
    HARD_GYRO_YAW = 4,
    A_STRICT_YAW_DX0 = 5,
    VISUAL_YAW_SCHMIDT_CURRENT_GAUGE = 6,   // K-space: K_eff = (I-qq^T)K_std
    VISUAL_YAW_H_PROJECTION_CURRENT   = 7   // H-space: H_eff = H - (Hq)q^T, then standard EKF
  };

  struct YawDxProjectionDiag {
    bool valid = false;
    VisualYawUpdateMode mode = VisualYawUpdateMode::ORIGINAL;
    double dx_yaw_before_projection_deg = 0.0;
    double dx_yaw_after_projection_deg = 0.0;
  };

  struct SchmidtYawDiag {
    double timestamp = 0.0;
    std::string update_type;
    std::string mode = "visual_yaw_schmidt_current_gauge";
    int H_rows = 0;
    int H_cols = 0;
    int N_cols = 1;
    int rank_Q = 0;
    double norm_Q = 0.0;
    double norm_H = 0.0;        // Frobenius norm of H
    double condition_N = 0.0;
    double norm_HQ = 0.0;
    double rel_norm_HQ = 0.0;
    double normal_dx_s_coeff_before = 0.0;
    double schmidt_dx_s_coeff_after = 0.0;
    double norm_dx_normal = 0.0;
    double norm_dx_schmidt = 0.0;
    double norm_delta_dx = 0.0;
    double Pss_norm_before = 0.0;
    double Pss_norm_after = 0.0;            // q_old^T P_plus q_old (same q as before)
    double Pss_norm_after_new_q = 0.0;      // q_new^T P_plus q_new (gauge rebuilt from updated state)
    double Pss_change_norm = 0.0;
    double Pas_change_norm = 0.0;
    double yaw_before_update = 0.0;
    double yaw_after_update = 0.0;
    double delta_yaw_update = 0.0;
    double bg_z_before = 0.0;
    double bg_z_after = 0.0;
    // q-energy decomposition (fraction of ||q||^2 in each block; mix of rad/m/m/s)
    double q_energy_imu_ori    = 0.0;
    double q_energy_imu_pos    = 0.0;
    double q_energy_imu_vel    = 0.0;
    double q_energy_clone_ori  = 0.0;
    double q_energy_clone_pos  = 0.0;
    double q_energy_slam       = 0.0;
    double q_energy_bias_calib = 0.0;  // remainder (bg/ba + calibration)
    // Covariance health
    int    neg_diag_clamp_count    = 0;
    double min_cov_diag_before_clamp  = 0.0;
    double min_cov_diag_after_update  = 0.0;
    bool projection_applied = false;
    bool schmidt_applied = false;
    std::string skipped_reason;
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
                        double visual_global_yaw_oc_alpha = 0.0);

  static void reset_last_yaw_dx_projection_diag();

  static YawDxProjectionDiag get_last_yaw_dx_projection_diag();

  /**
   * @brief Consider-Filter (Schmidt-KF) update.
   *
   * 在 Bierman 1977 的 "consider filter" 变体里, 状态分两块:
   *   - active:    出现在 H_order 里的变量, 正常被 K*res 修改 mean, 正常降协方差
   *   - nuisance:  其余所有状态变量, **mean 保持不变**, 只通过 cross-covariance
   *                降低 (active, nuisance) 的相关性. P_NN (nuisance 自相关) **不变**.
   *
   * 用途: 当外部观测 (如激光测距) 精度很高但只观测少量状态 (如 p_z) 时,
   * 标准 EKF 会通过 P_XP 把 residual 反传到 ba/bg 等弱可观测的状态, 在 mono VIO
   * 场景下往往越拉越偏. Schmidt filter 只让 active 拿到测量的修正, 把其他状态
   * 当 "未知常量" 保持原样 — 不乱改, 等 VIO 自己的视觉约束慢慢观测它们.
   *
   * 注意: 这是次优 (sub-optimal) 滤波 (与标准 EKF 相比), 但 mean 更健壮,
   *       协方差稍微保守 (P_NN 不减小 -> 后续 update 有更大的 cross-gain).
   *
   * 数学:
   *   S    = H * P_SS * H^T + R         (只用 active 块, 不用全 P)
   *   K_S  = P_SS * H^T * S^{-1}        (active-only gain)
   *   x_S <- x_S + K_S * res            (nuisance mean 不动)
   *   P_SS <- P_SS - K_S * H * P_SS
   *   P_SN <- P_SN - K_S * H * P_SN     (对每个 nuisance 块 N, 更新与 active 的 cross)
   *   P_NN <- unchanged
   *
   * @param state   状态指针
   * @param H_order 被 H 显式观测到的 active 变量序列
   * @param H       compressed Jacobian (行=观测数, 列=sum(H_order.size()))
   * @param res     观测残差
   * @param R       观测噪声协方差
   */
  static void EKFUpdateSchmidt(std::shared_ptr<State> state,
                               const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
                               const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
                               const Eigen::MatrixXd &R);

  /**
   * @brief Schmidt / consider-state Kalman update protecting the global-yaw gauge subspace.
   *
   * Builds the current-state (non-FEJ) global yaw gauge direction q_full over the
   * FULL N-dimensional covariance state space (IMU + all clones + all SLAM features),
   * not over the H_order-local subspace.  This ensures the protected direction is
   * consistent with the complete filter state.
   *
   * The Schmidt gain is:
   *   K_eff = (I - q_full q_full^T) K_std
   * where K_std is the standard EKFUpdate gain.  By construction q_full^T K_eff = 0,
   * so the yaw-gauge component of dx is zero.
   *
   * All state variables (including those not in H_order) receive the correction
   * K_eff[var] * res through their cross-covariance with the observed H_order block.
   * The covariance is updated with the full symmetric Joseph form:
   *   P+ = P - K_eff M_a^T - M_a K_eff^T + K_eff S K_eff^T
   * which analytically preserves q_full^T P+ q_full = q_full^T P q_full (Pss unchanged).
   *
   * @param state       Filter state
   * @param H_order     Variables explicitly observed by H
   * @param H           Compressed Jacobian (m x n_H)
   * @param res         Residual (m x 1)
   * @param R           Measurement noise (m x m)
   * @param update_type Label string for diagnostics ("msckf", "slam", "slam_delayed")
   * @param diag_out    Optional diagnostics output (may be nullptr)
   */
  static void EKFUpdateSchmidtYawCurrentGauge(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const Eigen::MatrixXd &H,
      const Eigen::VectorXd &res,
      const Eigen::MatrixXd &R,
      const std::string &update_type = "visual",
      SchmidtYawDiag *diag_out = nullptr);

  /// Open (or re-open) the per-update Schmidt yaw diagnostic CSV.
  static void open_schmidt_yaw_diag_csv(const std::string &path);

  /**
   * @brief H-space current-yaw-gauge projection update (mode C).
   *
   * Builds the current-state yaw gauge q restricted to the H_order subspace,
   * then projects: H_eff = H - (H*q_H)*q_H^T  so that H_eff*q_H ≈ 0.
   * A standard EKFUpdate is then applied with H_eff.
   *
   * This is the H-space counterpart of EKFUpdateSchmidtYawCurrentGauge (mode B).
   * Unlike mode B, the measurement Jacobian itself has no yaw component;
   * the residual cannot pull the yaw gauge direction at all.
   *
   * Named: visual_yaw_h_projection_current
   */
  static void EKFUpdateYawGaugeHProjectionCurrent(
      std::shared_ptr<State> state,
      const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
      const Eigen::MatrixXd &H,
      const Eigen::VectorXd &res,
      const Eigen::MatrixXd &R);

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
   * @brief Z-only EKF covariance update: only reduce P_zz, cross-terms unchanged.
   *
   * Standard Joseph-form reduction for a 1-D measurement of p_z:
   *   P_zz_new = P_zz - P_zz^2 / S
   *
   * @param state Pointer to state
   * @param S Innovation covariance (P_zz + R)
   */
  static void ekf_update_zonly(std::shared_ptr<State> state, double R);

  /**
   * @brief Z-only EKF update: full covariance update, only p_z state correction.
   *
   * Performs a standard EKF covariance update (Joseph form) for consistency,
   * but only applies the IMU position-z component of the state correction.
   * All other state components (px, py, v, q, bg, ba, clones, SLAM) are
   * reverted to their pre-update values.
   *
   * Uses the same H_order / H / res / R interface as EKFUpdate.
   *
   * @param state Pointer to state
   * @param H_order Variable ordering used in the compressed Jacobian
   * @param H Condensed Jacobian of updating measurement
   * @param res Residual of updating measurement
   * @param R Updating measurement covariance
   */
  static void EKFUpdateZOnly(std::shared_ptr<State> state,
                             const std::vector<std::shared_ptr<ov_type::Type>> &H_order,
                             const Eigen::MatrixXd &H, const Eigen::VectorXd &res,
                             const Eigen::MatrixXd &R);

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
