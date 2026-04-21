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

#ifndef OV_TYPE_TYPE_IMU_H
#define OV_TYPE_TYPE_IMU_H

#include "PoseJPL.h"
#include "utils/quat_ops.h"

namespace ov_type {

/**
 * @brief Derived Type class that implements an IMU state
 *
 * Contains a PoseJPL, Vec velocity, Vec gyro bias, and Vec accel bias.
 * This should be similar to that of the standard MSCKF state besides the ordering.
 * The pose is first, followed by velocity, etc.
 */
// =============================================================================
// [中文] IMU — MSCKF 中 IMU 状态对象。
//
// 名义维度 (_value, size=16): [q_GtoI(4), p_IinG(3), v_IinG(3), b_g(3), b_a(3)]
// 误差维度 (local_size, size=15): [δθ(3), δp(3), δv(3), δbg(3), δba(3)]
//   四元数是 4 维存储但只有 3 个自由度 (单位模约束), 误差用 δθ 表示, 因此两种维度差 1。
// 子变量:
//   _pose = PoseJPL  (姿态 q + 位置 p, 另外维护 FEJ 副本)
//   _v    = Vec(3)   (速度)
//   _bg   = Vec(3)   (陀螻零偏)
//   _ba   = Vec(3)   (加速仪零偏)
// 所有子变量共享一段 _Cov 矩阵内的连续索引 (见 set_local_id), 这样在传播/更新时可以整块寻址。
//
// 平环 update() 语义 (EKF 误差注入):
//   dx = [δθ; δp; δv; δbg; δba]  (15×1)
//   姿态: q ← quat_multiply(dq, q),  dq = normalize([0.5δθ; 1])  — 上名义值添加误差旋转
//   其他: p ← p + δp,  v ← v + δv,  bg ← bg + δbg,  ba ← ba + δba
// =============================================================================
class IMU : public Type {

public:
  // [中文] 构造: 对 Type 基类传 15 —— 误差维度。
  //   分别新建 PoseJPL (7→误差6), 速度 Vec(3), 两个偏置 Vec(3)。
  //   初始值: q=(0,0,0,1)→单位四元数,  p=v=bg=ba=0, 同时设置 FEJ 等于同一值。
  IMU() : Type(15) {

    // Create all the sub-variables
    _pose = std::shared_ptr<PoseJPL>(new PoseJPL());
    _v = std::shared_ptr<Vec>(new Vec(3));
    _bg = std::shared_ptr<Vec>(new Vec(3));
    _ba = std::shared_ptr<Vec>(new Vec(3));

    // Set our default state value
    Eigen::VectorXd imu0 = Eigen::VectorXd::Zero(16, 1);
    imu0(3) = 1.0;
    set_value_internal(imu0);
    set_fej_internal(imu0);
  }

  ~IMU() {}

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that we update the sub-variables also.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  // [中文] set_local_id: 设置此 IMU 对象在全局 _Cov 矩阵里的起始行/列, 并让四个子变量按顺序接住。
  //   维度布局 (从 new_id 开始): [pose: 6] [v: 3] [bg: 3] [ba: 3] → 共 15 维。
  //   当 new_id==-1 表示暂未在矩阵中 (临时 Type), 不移动子 id。
  void set_local_id(int new_id) override {
    _id = new_id;
    _pose->set_local_id(new_id);
    _v->set_local_id(_pose->id() + ((new_id != -1) ? _pose->size() : 0));
    _bg->set_local_id(_v->id() + ((new_id != -1) ? _v->size() : 0));
    _ba->set_local_id(_bg->id() + ((new_id != -1) ? _bg->size() : 0));
  }

  /**
   * @brief Performs update operation using JPLQuat update for orientation, then vector updates for
   * position, velocity, gyro bias, and accel bias (in that order).
   *
   * @param dx 15 DOF vector encoding update using the following order (q, p, v, bg, ba)
   */
  // [中文] update: EKF 在传播/更新后用的误差注入函数。
  //   输入 dx = [δθ(3); δp(3); δv(3); δbg(3); δba(3)]  (15×1, 切空间上的误差)
  //   1) 姿态部分用 JPL 左乘注入: dq = normalize([0.5δθ; 1]),  q_new = quat_multiply(dq, q_old)
  //      这个 "0.5" 来自于 q 的时间导数公式 q̇=0.5Ω(w)q 中的系数; 并且在小角度近似 sin(θ/2)≈θ/2。
  //   2) 位置 / 速度 / 偏置都直接加法注入。
  //   注意不操作 FEJ 副本 —— FEJ 只在初始化时设置一次, 后续保持锁住。
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    Eigen::Matrix<double, 16, 1> newX = _value;

    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx.block(0, 0, 3, 1), 1.0;
    dq = ov_core::quatnorm(dq);

    newX.block(0, 0, 4, 1) = ov_core::quat_multiply(dq, quat());
    newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

    newX.block(7, 0, 3, 1) += dx.block(6, 0, 3, 1);
    newX.block(10, 0, 3, 1) += dx.block(9, 0, 3, 1);
    newX.block(13, 0, 3, 1) += dx.block(12, 0, 3, 1);

    set_value(newX);
  }

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value(const Eigen::MatrixXd &new_value) override { set_value_internal(new_value); }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej(const Eigen::MatrixXd &new_value) override { set_fej_internal(new_value); }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<Type>(new IMU());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  // [中文] check_if_subvariable: 子变量查询。StateHelper 用它找到某个指针背后实际管理的 Type。
  //   返回值: nullptr (非此 IMU 的子件) | _pose | _v | _bg | _ba | 或者继续递归到 _pose 的子变量 (q/p)。
  std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) override {
    if (check == _pose) {
      return _pose;
    } else if (check == _pose->check_if_subvariable(check)) {
      return _pose->check_if_subvariable(check);
    } else if (check == _v) {
      return _v;
    } else if (check == _bg) {
      return _bg;
    } else if (check == _ba) {
      return _ba;
    }
    return nullptr;
  }

  /// Rotation access
  Eigen::Matrix<double, 3, 3> Rot() const { return _pose->Rot(); }

  /// FEJ Rotation access
  Eigen::Matrix<double, 3, 3> Rot_fej() const { return _pose->Rot_fej(); }

  /// Rotation access quaternion
  Eigen::Matrix<double, 4, 1> quat() const { return _pose->quat(); }

  /// FEJ Rotation access quaternion
  Eigen::Matrix<double, 4, 1> quat_fej() const { return _pose->quat_fej(); }

  /// Position access
  Eigen::Matrix<double, 3, 1> pos() const { return _pose->pos(); }

  /// FEJ position access
  Eigen::Matrix<double, 3, 1> pos_fej() const { return _pose->pos_fej(); }

  /// Velocity access
  Eigen::Matrix<double, 3, 1> vel() const { return _v->value(); }

  // FEJ velocity access
  Eigen::Matrix<double, 3, 1> vel_fej() const { return _v->fej(); }

  /// Gyro bias access
  Eigen::Matrix<double, 3, 1> bias_g() const { return _bg->value(); }

  /// FEJ gyro bias access
  Eigen::Matrix<double, 3, 1> bias_g_fej() const { return _bg->fej(); }

  /// Accel bias access
  Eigen::Matrix<double, 3, 1> bias_a() const { return _ba->value(); }

  // FEJ accel bias access
  Eigen::Matrix<double, 3, 1> bias_a_fej() const { return _ba->fej(); }

  /// Pose type access
  std::shared_ptr<PoseJPL> pose() { return _pose; }

  /// Quaternion type access
  std::shared_ptr<JPLQuat> q() { return _pose->q(); }

  /// Position type access
  std::shared_ptr<Vec> p() { return _pose->p(); }

  /// Velocity type access
  std::shared_ptr<Vec> v() { return _v; }

  /// Gyroscope bias access
  std::shared_ptr<Vec> bg() { return _bg; }

  /// Acceleration bias access
  std::shared_ptr<Vec> ba() { return _ba; }

protected:
  /// Pose subvariable
  std::shared_ptr<PoseJPL> _pose;

  /// Velocity subvariable
  std::shared_ptr<Vec> _v;

  /// Gyroscope bias subvariable
  std::shared_ptr<Vec> _bg;

  /// Acceleration bias subvariable
  std::shared_ptr<Vec> _ba;

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 16);
    assert(new_value.cols() == 1);

    _pose->set_value(new_value.block(0, 0, 7, 1));
    _v->set_value(new_value.block(7, 0, 3, 1));
    _bg->set_value(new_value.block(10, 0, 3, 1));
    _ba->set_value(new_value.block(13, 0, 3, 1));

    _value = new_value;
  }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej_internal(const Eigen::MatrixXd &new_value) {

    assert(new_value.rows() == 16);
    assert(new_value.cols() == 1);

    _pose->set_fej(new_value.block(0, 0, 7, 1));
    _v->set_fej(new_value.block(7, 0, 3, 1));
    _bg->set_fej(new_value.block(10, 0, 3, 1));
    _ba->set_fej(new_value.block(13, 0, 3, 1));

    _fej = new_value;
  }
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_IMU_H
