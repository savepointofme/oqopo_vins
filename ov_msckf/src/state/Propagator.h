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

// =============================================================================
// [中文注释] Propagator.h
// -----------------------------------------------------------------------------
// IMU 预测器：负责把状态从上一时刻传播到当前相机帧时刻, 并更新协方差。
//
// 工作流程:
//   1. feed_imu               : 储存 IMU 并丢弃过旧的测量
//   2. select_imu_readings    : 选出 [t_prev, t_curr] 之间可用的 IMU,
//                                头尾做线性插值凑足边界
//   3. predict_and_compute    : 对单个区间做 RK4 / 离散积分, 得到
//                                状态传递矩阵 Phi 和噪声匡自矩阵 Qd
//   4. propagate_and_clone    : 对整个时间段积分 + 新增一个克隆
//   5. fast_state_propagate   : 用 cache 加速短时间内的单点预测 (非滤波)
//
// 数学细节: OpenVINS 采用"流形 EKF"实现, 状态在流形上更新,
//   R{G-I} 用四元数表达, 其他量用欧几里得形式。
//   更多请见 docs-cn/vio_manager.md 中“传播”节, 以及官方文档
//   docs/propagation.md 里的推导。
// =============================================================================

#ifndef OV_MSCKF_STATE_PROPAGATOR_H
#define OV_MSCKF_STATE_PROPAGATOR_H

#include <atomic>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "utils/sensor_data.h"

#include "utils/NoiseManager.h"

namespace ov_msckf {

class State;

/**
 * @brief Performs the state covariance and mean propagation using imu measurements
 *
 * We will first select what measurements we need to propagate with.
 * We then compute the state transition matrix at each step and update the state and covariance.
 * For derivations look at @ref propagation page which has detailed equations.
 *
 * [中文] IMU 传播器。
 *  - 以对象形式封装所有与 IMU 有关的预测逻辑;
 *  - 对外暴露 feed_imu (储存测量) 和 propagate_and_clone (执行传播) 两个主入口;
 *  - 内部维护 imu_data (分范加锁) 和一份针对高频查询的缓存 cache_*。
 */
class Propagator {
public:
  /**
   * @brief Default constructor
   * @param noises imu noise characteristics (continuous time)
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   *
   * [中文] 按照连续时间器件参数 (sigma_w / sigma_a / sigma_wb / sigma_ab)
   *        预计算平方缓存, 避免每步都调用 pow。
   */
  Propagator(NoiseManager noises, double gravity_mag) : _noises(noises) {
    _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
    _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
    _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
    _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);
    last_prop_time_offset = 0.0;
    _gravity << 0.0, 0.0, gravity_mag;
  }

  /**
   * @brief Stores incoming inertial readings
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before (in IMU clock)
   *
   * [中文] 并发安全地追加一条 IMU 并清理过旧测量。
   *        oldest_time 再减去 0.10s 的種冲量, 是为了防止边界插值时缺数据。
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Append it to our vector
    std::lock_guard<std::mutex> lck(imu_data_mtx);
    imu_data.emplace_back(message);

    // Clean old measurements
    // std::cout << "PROP: imu_data.size() " << imu_data.size() << std::endl;
    clean_old_imu_measurements(oldest_time - 0.10);
  }

  /**
   * @brief This will remove any IMU measurements that are older then the given measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU clock)
   */
  void clean_old_imu_measurements(double oldest_time) {
    if (oldest_time < 0)
      return;
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Will invalidate the cache used for fast propagation
   */
  void invalidate_cache() noexcept;

  struct FastStateCacheStatus {
    std::uint64_t epoch = 0;
    std::uint64_t published_epoch = 0;
    bool valid = false;
  };

  /**
   * @brief Thread-safe diagnostic snapshot of the fast-propagation cache.
   *
   * `valid` is true only when the published cache belongs to the current
   * invalidation epoch.
   */
  FastStateCacheStatus fast_state_cache_status() const;

  void set_yaw_diag_window(double t0, double t1) {
    yaw_diag_t0_ = t0;
    yaw_diag_t1_ = t1;
  }
  void set_yaw_diag_path(const std::string &path);

  /**
   * @brief Propagate state up to given timestamp and then clone
   *
   * This will first collect all imu readings that occured between the
   * *current* state time and the new time we want the state to be at.
   * If we don't have any imu readings we will try to extrapolate into the future.
   * After propagating the mean and covariance using our dynamics,
   * We clone the current imu pose as a new clone in our state.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to and clone at (CAM clock frame)
   *
   * [中文] 传播 + 克隆。这是 VioManager 在每帧相机到来时调用的“步未来时刻”入口:
   *   1. 在 _calib_dt_CAMtoIMU 的补偿下, 把相机时间戳转化到 IMU 时钟
   *   2. 从 imu_data 中选出 [t_prev, t_curr] 之间可用 IMU
   *   3. 逐区间 predict_and_compute: 积分得到新的 IMU 状态, 同时累乘得到 Phi 和 Qd
   *   4. 调用 StateHelper::EKFPropagation 把 Phi/Qd 加到全局协方差上
   *   5. 调用 StateHelper::augment_clone 在 _clones_IMU 中插入一个当前位姿的克隆
   */
  void propagate_and_clone(std::shared_ptr<State> state, double timestamp);

  /**
   * @brief Gets what the state and its covariance will be at a given timestamp
   *
   * This can be used to find what the state will be in the "future" without propagating it.
   * This will propagate a clone of the current IMU state and its covariance matrix.
   * This is typically used to provide high frequency pose estimates between updates.
   *
   * @param state Pointer to state
   * @param timestamp Time to propagate to (IMU clock frame)
   * @param state_plus The propagated state (q_GtoI, p_IinG, v_IinI, w_IinI)
   * @param covariance The propagated covariance (q_GtoI, p_IinG, v_IinI, w_IinI)
   * @return True if we were able to propagate the state to the current timestep
   */
  // [中文] 不改滤波状态的快速预测 (主要用于高频输出). 内部使用 cache_* 避免重复
  //        从 imu_data 中查找区间, 调用频率可达 IMU 采样率。
  bool fast_state_propagate(std::shared_ptr<State> state, double timestamp, Eigen::Matrix<double, 13, 1> &state_plus,
                            Eigen::Matrix<double, 12, 12> &covariance);

  /**
   * @brief Helper function that given current imu data, will select imu readings between the two times.
   *
   * This will create measurements that we will integrate with, and an extra measurement at the end.
   * We use the @ref interpolate_data() function to "cut" the imu readings at the begining and end of the integration.
   * The timestamps passed should already take into account the time offset values.
   *
   * @param imu_data IMU data we will select measurements from
   * @param time0 Start timestamp
   * @param time1 End timestamp
   * @param warn If we should warn if we don't have enough IMU to propagate with (e.g. fast prop will get warnings otherwise)
   * @return Vector of measurements (if we could compute them)
   */
  static std::vector<ov_core::ImuData> select_imu_readings(const std::vector<ov_core::ImuData> &imu_data, double time0, double time1,
                                                           bool warn = true);

  /**
   * @brief Compute a relative (passive) rotation between two timestamps from the gyro buffer.
   *
   * Integrates the gyroscope readings in (time0, time1] using a simple
   * trapezoidal rule (per-sample dt-weighted sum of bias-corrected angular
   * velocity), then converts the resulting incremental rotation vector to a
   * passive rotation matrix via Rodrigues:
   *
   *    theta = sum_i ( (w_i - bg) * dt_i )
   *    R_I0_to_I1 = exp([-theta]_x)
   *
   * Here R_I0_to_I1 transforms a direction expressed in the IMU frame at
   * time0 to its representation in the IMU frame at time1 (i.e. a passive /
   * coordinate-frame rotation). For a vector at infinity that is fixed in
   * the world:  v_I1 = R_I0_to_I1 * v_I0.
   *
   * This is intentionally cheap — we deliberately do *not* propagate the
   * filter state for this query; the prediction is consumed by the visual
   * front-end (gyro-aided KLT) and never enters the EKF.
   *
   * @param time0 Start timestamp (IMU clock)
   * @param time1 End timestamp (IMU clock)
   * @param bg Current best estimate of the gyro bias (IMU frame)
   * @param[out] R_I0_to_I1 Resulting passive rotation
   * @return True iff at least two IMU samples were available in the window
   */
  bool compute_relative_rotation(std::shared_ptr<State> state, double time0, double time1, Eigen::Matrix3d &R_I0_to_I1);

  /**
   * @brief Nice helper function that will linearly interpolate between two imu messages.
   *
   * This should be used instead of just "cutting" imu messages that bound the camera times
   * Give better time offset if we use this function, could try other orders/splines if the imu is slow.
   *
   * @param imu_1 imu at begining of interpolation interval
   * @param imu_2 imu at end of interpolation interval
   * @param timestamp Timestamp being interpolated to
   */
  static ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1, const ov_core::ImuData &imu_2, double timestamp) {
    // time-distance lambda
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    // PRINT_DEBUG("lambda - %d\n", lambda);
    // interpolate between the two times
    ov_core::ImuData data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }

  /**
   * @brief compute the Jacobians for Dw
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Dw,kalibr} & =
   *   \begin{bmatrix}
   *   {}^w\hat{w}_1 \mathbf{I}_3  & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_2\mathbf{e}_3 & {}^w\hat{w}_3 \mathbf{e}_3
   *   \end{bmatrix} \\
   *   \mathbf{H}_{Dw,rpng} & =
   *   \begin{bmatrix}
   *   {}^w\hat{w}_1\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_1 & {}^w\hat{w}_2\mathbf{e}_2 & {}^w\hat{w}_3 \mathbf{I}_3
   *   \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param w_uncorrected Angular velocity in a frame with bias and gravity sensitivity removed
   */
  static Eigen::MatrixXd compute_H_Dw(std::shared_ptr<State> state, const Eigen::Vector3d &w_uncorrected);

  /**
   * @brief compute the Jacobians for Da
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Da,kalibr} & =
   * \begin{bmatrix}
   *   {}^a\hat{a}_1\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_1 & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_3 \mathbf{I}_3
   * \end{bmatrix} \\
   * \mathbf{H}_{Da,rpng} & =
   * \begin{bmatrix}
   *   {}^a\hat{a}_1 \mathbf{I}_3 &  & {}^a\hat{a}_2\mathbf{e}_2 & {}^a\hat{a}_2\mathbf{e}_3 & {}^a\hat{a}_3\mathbf{e}_3
   * \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param a_uncorrected Linear acceleration in gyro frame with bias removed
   */
  static Eigen::MatrixXd compute_H_Da(std::shared_ptr<State> state, const Eigen::Vector3d &a_uncorrected);

  /**
   * @brief compute the Jacobians for Tg
   *
   * See @ref analytical_linearization_imu for details.
   * \f{align*}{
   * \mathbf{H}_{Tg} & =
   *  \begin{bmatrix}
   *  {}^I\hat{a}_1 \mathbf{I}_3 & {}^I\hat{a}_2 \mathbf{I}_3 & {}^I\hat{a}_3 \mathbf{I}_3
   *  \end{bmatrix}
   * \f}
   *
   * @param state Pointer to state
   * @param a_inI Linear acceleration with bias removed
   */
  static Eigen::MatrixXd compute_H_Tg(std::shared_ptr<State> state, const Eigen::Vector3d &a_inI);

protected:
  /**
   * @brief Propagates the state forward using the imu data and computes the noise covariance and state-transition
   * matrix of this interval.
   *
   * This function can be replaced with analytical/numerical integration or when using a different state representation.
   * This contains our state transition matrix along with how our noise evolves in time.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref propagation_discrete page for details on how discrete model was derived.
   * See the @ref propagation_analytical page for details on how analytic model was derived.
   *
   * @param state Pointer to state
   * @param data_minus imu readings at beginning of interval
   * @param data_plus imu readings at end of interval
   * @param F State-transition matrix over the interval
   * @param Qd Discrete-time noise covariance over the interval
   */
  void predict_and_compute(std::shared_ptr<State> state, const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus,
                           Eigen::MatrixXd &F, Eigen::MatrixXd &Qd);

  /**
   * @brief Discrete imu mean propagation.
   *
   * See @ref disc_prop for details on these equations.
   * \f{align*}{
   * \text{}^{I_{k+1}}_{G}\hat{\bar{q}}
   * &= \exp\bigg(\frac{1}{2}\boldsymbol{\Omega}\big({\boldsymbol{\omega}}_{m,k}-\hat{\mathbf{b}}_{g,k}\big)\Delta t\bigg)
   * \text{}^{I_{k}}_{G}\hat{\bar{q}} \\
   * ^G\hat{\mathbf{v}}_{k+1} &= \text{}^G\hat{\mathbf{v}}_{I_k} - {}^G\mathbf{g}\Delta t
   * +\text{}^{I_k}_G\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t\\
   * ^G\hat{\mathbf{p}}_{I_{k+1}}
   * &= \text{}^G\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k} \Delta t
   * - \frac{1}{2}{}^G\mathbf{g}\Delta t^2
   * + \frac{1}{2} \text{}^{I_k}_{G}\hat{\mathbf{R}}^\top(\mathbf{a}_{m,k} - \hat{\mathbf{b}}_{\mathbf{a},k})\Delta t^2
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                             Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);

  /**
   * @brief RK4 imu mean propagation.
   *
   * See this wikipedia page on [Runge-Kutta Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).
   * We are doing a RK4 method, [this wolfram page](http://mathworld.wolfram.com/Runge-KuttaMethod.html) has the forth order equation
   * defined below. We define function \f$ f(t,y) \f$ where y is a function of time t, see @ref imu_kinematic for the definition of the
   * continuous-time functions.
   *
   * \f{align*}{
   * {k_1} &= f({t_0}, y_0) \Delta t  \\
   * {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
   * {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
   * {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
   * y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1 \over 3}{k_3} + {1 \over 6}{k_4}} \right)
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_rk4(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                        const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2, Eigen::Vector4d &new_q, Eigen::Vector3d &new_v,
                        Eigen::Vector3d &new_p);

  /**
   * @brief Analytically compute the integration components based on ACI^2
   *
   * See the @ref analytical_prop page and @ref analytical_integration_components for details.
   * For computing Xi_1, Xi_2, Xi_3 and Xi_4 we have:
   *
   * \f{align*}{
   * \boldsymbol{\Xi}_1 & = \mathbf{I}_3 \delta t + \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}} \lfloor \hat{\mathbf{k}} \rfloor
   * + \left(\delta t  - \frac{\sin (\hat{\omega} \delta t)}{\hat{\omega}}\right) \lfloor \hat{\mathbf{k}} \rfloor^2 \\
   * \boldsymbol{\Xi}_2 & = \frac{1}{2} \delta t^2 \mathbf{I}_3 +
   * \frac{\hat{\omega} \delta t - \sin (\hat{\omega} \delta t)}{\hat{\omega}^2}\lfloor \hat{\mathbf{k}} \rfloor
   * + \left( \frac{1}{2} \delta t^2 - \frac{1  - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) \lfloor \hat{\mathbf{k}} \rfloor ^2
   * \\ \boldsymbol{\Xi}_3  &= \frac{1}{2}\delta t^2  \lfloor \hat{\mathbf{a}} \rfloor
   * + \frac{\sin (\hat{\omega} \delta t_i) - \hat{\omega} \delta t }{\hat{\omega}^2} \lfloor\hat{\mathbf{a}} \rfloor \lfloor
   * \hat{\mathbf{k}} \rfloor
   * + \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)  }{\hat{\omega}^2}
   * \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
   * + \left( \frac{1}{2} \delta t^2 - \frac{1 - \cos (\hat{\omega} \delta t)}{\hat{\omega}^2} \right) 	\lfloor\hat{\mathbf{a}} \rfloor
   * \lfloor \hat{\mathbf{k}} \rfloor ^2
   * + \left(
   * \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
   *  \right)
   *  \lfloor \hat{\mathbf{k}} \rfloor ^2 \lfloor\hat{\mathbf{a}} \rfloor
   *  + \left(
   *  \frac{1}{2} \delta t^2 + \frac{1 - \cos (\hat{\omega} \delta t) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t) }{\hat{\omega}^2}
   *  \right)  \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
   *  - \frac{ 3 \sin (\hat{\omega} \delta t) - 2 \hat{\omega} \delta t - \hat{\omega} \delta t \cos (\hat{\omega} \delta t)
   * }{\hat{\omega}^2} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor ^2  \\
   * \boldsymbol{\Xi}_4 & = \frac{1}{6}\delta
   * t^3 \lfloor\hat{\mathbf{a}} \rfloor
   * + \frac{2(1 - \cos (\hat{\omega} \delta t)) - (\hat{\omega}^2 \delta t^2)}{2 \hat{\omega}^3}
   *  \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor
   *  + \left(
   *  \frac{2(1- \cos(\hat{\omega} \delta t)) - \hat{\omega} \delta t \sin (\hat{\omega} \delta t)}{\hat{\omega}^3}
   *  \right)
   *  \lfloor \hat{\mathbf{k}} \rfloor\lfloor\hat{\mathbf{a}} \rfloor
   *  + \left(
   *  \frac{\sin (\hat{\omega} \delta t) - \hat{\omega} \delta t}{\hat{\omega}^3} +
   *  \frac{\delta t^3}{6}
   *  \right)
   *  \lfloor\hat{\mathbf{a}} \rfloor \lfloor \hat{\mathbf{k}} \rfloor^2
   *  +
   *  \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
   * \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \lfloor \hat{\mathbf{k}} \rfloor^2\lfloor\hat{\mathbf{a}} \rfloor
   *  +
   *  \frac{\hat{\omega} \delta t - 2 \sin(\hat{\omega} \delta t) + \frac{1}{6}(\hat{\omega} \delta t)^3 + \hat{\omega} \delta t
   * \cos(\hat{\omega} \delta t)}{\hat{\omega}^3} \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor
   *  +
   *  \frac{4 \cos(\hat{\omega} \delta t) - 4 + (\hat{\omega} \delta t)^2 + \hat{\omega} \delta t \sin(\hat{\omega} \delta t) }
   *  {\hat{\omega}^3}
   *  \hat{\mathbf{k}}^{\top} \hat{\mathbf{a}} \lfloor \hat{\mathbf{k}} \rfloor^2
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4 in order
   */
  void compute_Xi_sum(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                      Eigen::Matrix<double, 3, 18> &Xi_sum);

  /**
   * @brief Analytically predict IMU mean based on ACI^2
   *
   * See the @ref analytical_prop page for details.
   *
   * \f{align*}{
   * {}^{I_{k+1}}_G\hat{\mathbf{R}} & \simeq  \Delta \mathbf{R}_k {}^{I_k}_G\hat{\mathbf{R}}  \\
   * {}^G\hat{\mathbf{p}}_{I_{k+1}} & \simeq {}^{G}\hat{\mathbf{p}}_{I_k} + {}^G\hat{\mathbf{v}}_{I_k}\delta t_k  +
   * {}^{I_k}_G\hat{\mathbf{R}}^\top  \Delta \hat{\mathbf{p}}_k - \frac{1}{2}{}^G\mathbf{g}\delta t^2_k \\
   * {}^G\hat{\mathbf{v}}_{I_{k+1}} & \simeq  {}^{G}\hat{\mathbf{v}}_{I_k} + {}^{I_k}_G\hat{\mathbf{R}}^\top + \Delta \hat{\mathbf{v}}_k -
   * {}^G\mathbf{g}\delta t_k
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4
   */
  void predict_mean_analytic(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                             Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p, Eigen::Matrix<double, 3, 18> &Xi_sum);

  /**
   * @brief Analytically compute state transition matrix F and noise Jacobian G based on ACI^2
   *
   * This function is for analytical integration of the linearized error-state.
   * This contains our state transition matrix and noise Jacobians.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref analytical_linearization page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param w_uncorrected Angular velocity in acc frame with bias and gravity sensitivity removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param Xi_sum All the needed integration components, including R_k, Xi_1, Xi_2, Jr, Xi_3, Xi_4
   * @param F State transition matrix
   * @param G Noise Jacobian
   */
  void compute_F_and_G_analytic(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                                const Eigen::Vector3d &w_uncorrected, const Eigen::Vector3d &a_uncorrected, const Eigen::Vector4d &new_q,
                                const Eigen::Vector3d &new_v, const Eigen::Vector3d &new_p, const Eigen::Matrix<double, 3, 18> &Xi_sum,
                                Eigen::MatrixXd &F, Eigen::MatrixXd &G);

  /**
   * @brief compute state transition matrix F and noise Jacobian G
   *
   * This function is for analytical integration or when using a different state representation.
   * This contains our state transition matrix and noise Jacobians.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref error_prop page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat Angular velocity with bias removed
   * @param a_hat Linear acceleration with bias removed
   * @param w_uncorrected Angular velocity in acc frame with bias and gravity sensitivity removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   * @param F State transition matrix
   * @param G Noise Jacobian
   */
  void compute_F_and_G_discrete(std::shared_ptr<State> state, double dt, const Eigen::Vector3d &w_hat, const Eigen::Vector3d &a_hat,
                                const Eigen::Vector3d &w_uncorrected, const Eigen::Vector3d &a_uncorrected, const Eigen::Vector4d &new_q,
                                const Eigen::Vector3d &new_v, const Eigen::Vector3d &new_p, Eigen::MatrixXd &F, Eigen::MatrixXd &G);

  /// Container for the noise values
  NoiseManager _noises;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;
  std::mutex imu_data_mtx;

  std::ofstream of_yaw_diag_;
  double yaw_diag_t0_ = -std::numeric_limits<double>::infinity();
  double yaw_diag_t1_ = std::numeric_limits<double>::infinity();

  /// Gravity vector
  Eigen::Vector3d _gravity;

  // Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  // Cache of the last fast propagated state
  mutable std::mutex cache_imu_mtx;
  std::atomic<std::uint64_t> cache_epoch{0};
  bool cache_imu_valid = false;
  std::uint64_t cache_published_epoch = 0;
  double cache_state_time = std::numeric_limits<double>::quiet_NaN();
  Eigen::MatrixXd cache_state_est;
  Eigen::MatrixXd cache_state_covariance;
  double cache_t_off = 0.0;
  double cache_output_timestamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix<double, 13, 1> cache_output_state_plus =
      Eigen::Matrix<double, 13, 1>::Zero();
  Eigen::Matrix<double, 12, 12> cache_output_covariance =
      Eigen::Matrix<double, 12, 12>::Zero();
};

} // namespace ov_msckf

#endif // OV_MSCKF_STATE_PROPAGATOR_H
