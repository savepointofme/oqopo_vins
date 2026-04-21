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

#ifndef OV_TYPE_TYPE_BASE_H
#define OV_TYPE_TYPE_BASE_H

#include <Eigen/Eigen>
#include <memory>

namespace ov_type {

/**
 * @brief Base class for estimated variables.
 *
 * This class is used how variables are represented or updated (e.g., vectors or quaternions).
 * Each variable is defined by its error state size and its location in the covariance matrix.
 * We additionally require all sub-types to have a update procedure.
 */
// =============================================================================
// [中文] Type — 所有状态对象的抽象基类 (IMU / PoseJPL / JPLQuat / Vec / Landmark …)
//
// 核心字段:
//   _value : 名义值 (如四元数 4维, 位置 3维), 存储于 Eigen 矩阵。
//   _fej   : First-Estimate Jacobian 的锁值 —— 对每个可观性敏感的状态在第一次估计时的快照,
//            EKF/MSCKF 计算雅可比时用 _fej (非 _value), 避免性 "假 info" 带入不可观方向。
//   _id    : 在全局 _Cov 矩阵中的起始行/列索引, -1 表示对象不在当前滤波器状态中。
//   _size  : 误差维度 (切空间维度). 对于四元数 _value=4 而 _size=3。
//
// 核心接口:
//   update(dx)    : Boxplus 操作, 把误差 dx (维度=_size) 注入到名义值里。
//   set_value/set_fej: 直接覆盖名义值或 FEJ 副本 (由初始化、克隆入口使用)。
//   clone()       : 深拷贝, 用于 MSCKF 滑窗内克隆当前位姿。
//   check_if_subvariable: 用于 StateHelper 检查 "某指针是不是我管理的子件"。
// =============================================================================
class Type {

public:
  /**
   * @brief Default constructor for our Type
   *
   * @param size_ degrees of freedom of variable (i.e., the size of the error state)
   */
  Type(int size_) { _size = size_; }

  virtual ~Type(){};

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that the minimum ID is -1 which says that the state is not in our covariance.
   * If the ID is larger than -1 then this is the index location in the covariance matrix.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  virtual void set_local_id(int new_id) { _id = new_id; }

  /**
   * @brief Access to variable id (i.e. its location in the covariance)
   */
  int id() { return _id; }

  /**
   * @brief Access to variable size (i.e. its error state size)
   */
  int size() { return _size; }

  /**
   * @brief Update variable due to perturbation of error state
   *
   * @param dx Perturbation used to update the variable through a defined "boxplus" operation
   */
  virtual void update(const Eigen::VectorXd &dx) = 0;

  /**
   * @brief Access variable's estimate
   */
  virtual const Eigen::MatrixXd &value() const { return _value; }

  /**
   * @brief Access variable's first-estimate
   */
  virtual const Eigen::MatrixXd &fej() const { return _fej; }

  /**
   * @brief Overwrite value of state's estimate
   * @param new_value New value that will overwrite state's value
   */
  virtual void set_value(const Eigen::MatrixXd &new_value) {
    assert(_value.rows() == new_value.rows());
    assert(_value.cols() == new_value.cols());
    _value = new_value;
  }

  /**
   * @brief Overwrite value of first-estimate
   * @param new_value New value that will overwrite state's fej
   */
  virtual void set_fej(const Eigen::MatrixXd &new_value) {
    assert(_fej.rows() == new_value.rows());
    assert(_fej.cols() == new_value.cols());
    _fej = new_value;
  }

  /**
   * @brief Create a clone of this variable
   */
  virtual std::shared_ptr<Type> clone() = 0;

  /**
   * @brief Determine if pass variable is a sub-variable
   *
   * If the passed variable is a sub-variable or the current variable this will return it.
   * Otherwise it will return a nullptr, meaning that it was unable to be found.
   *
   * @param check Type pointer to compare our subvariables to
   */
  virtual std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) { return nullptr; }

protected:
  /// First-estimate
  Eigen::MatrixXd _fej;

  /// Current best estimate
  Eigen::MatrixXd _value;

  /// Location of error state in covariance
  int _id = -1;

  /// Dimension of error state
  int _size = -1;
};

} // namespace ov_type

#endif // OV_TYPE_TYPE_BASE_H