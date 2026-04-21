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

#ifndef OV_TYPE_TYPE_LANDMARK_H
#define OV_TYPE_TYPE_LANDMARK_H

#include "LandmarkRepresentation.h"
#include "Vec.h"
#include "utils/colors.h"
#include "utils/print.h"

namespace ov_type {

/**
 * @brief Type that implements a persistent SLAM feature.
 *
 * We store the feature ID that should match the IDs in the trackers.
 * Additionally if this is an anchored representation we store what clone timestamp this is anchored from and what camera.
 * If this features should be marginalized its flag can be set and during cleanup it will be removed.
 */
// =============================================================================
// [中文] Landmark — 持久化到滤波器里的 SLAM 特征点 (不同于 MSCKF 临时特征)
//
// 继承关系: Landmark → Vec → Type
//
// 关键字段:
//   _featid                : 与前端跟踪器对应的全局特征 ID, 用来在观测帧之间关联。
//   _unique_camera_id      : 这条 SLAM 特征最先是从哪个相机看到的 (立体/多相机时区分)。
//   _anchor_cam_id         : 若使用锚点 (anchored) 表示, 此为锚相机 id; 默认 = 首次观测相机。
//   _anchor_clone_timestamp: 锚点对应的位姿克隆时间戳 (状态里 _clones_IMU[t] 为锚位姿)。
//   has_had_anchor_change  : 锚点是否被迁移过 (边缘化老锚时会切换)。
//   should_marg            : 该特征是否即将被边缘化 (在 StateHelper 清理时删除)。
//   update_fail_count      : 连续更新失败次数 (如卡方门限拒绝), 超过阈值直接 marg。
//   uv_norm_zero(_fej)     : 首次观测时的归一化像素 (用于 1D-inverse-depth 表示)。
//   _feat_representation   : 表示类型 (GLOBAL_3D / GLOBAL_INV / ANCHORED_3D / ANCHORED_INV / ...).
//
// 维度根据表示类型不同:
//   - GLOBAL_3D / ANCHORED_3D      : _size = 3  (x, y, z)
//   - GLOBAL_INVERSE_DEPTH         : _size = 3  (azimuth, elevation, 1/depth)  —— 用球坐标 + 逆深度
//   - ANCHORED_INVERSE_DEPTH_SINGLE: _size = 1  (1/depth, 方向已固定)
//
// 锚点表示的好处: 将特征绑定在某个克隆位姿上, 这样全局线性化点不动 → 更好的可观性。
// =============================================================================
class Landmark : public Vec {

public:
  /// Default constructor (feature is a Vec of size 3 or Vec of size 1)
  Landmark(int dim) : Vec(dim) {}

  /// Feature ID of this landmark (corresponds to frontend id)
  size_t _featid;

  /// What unique camera stream this slam feature was observed from
  int _unique_camera_id = -1;

  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  int _anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double _anchor_clone_timestamp = -1;

  /// Boolean if this landmark has had at least one anchor change
  bool has_had_anchor_change = false;

  /// Boolean if this landmark should be marginalized out
  bool should_marg = false;

  /// Number of times the update has failed for this feature (we should remove if it fails a couple times!)
  int update_fail_count = 0;

  /// First normalized uv coordinate bearing of this measurement (used for single depth representation)
  Eigen::Vector3d uv_norm_zero;

  /// First estimate normalized uv coordinate bearing of this measurement (used for single depth representation)
  Eigen::Vector3d uv_norm_zero_fej;

  /// What feature representation this feature currently has
  LandmarkRepresentation::Representation _feat_representation;

  /**
   * @brief Overrides the default vector update rule
   * We want to selectively update the FEJ value if we are using an anchored representation.
   * @param dx Additive error state correction
   */
  void update(const Eigen::VectorXd &dx) override {
    // Update estimate
    assert(dx.rows() == _size);
    set_value(_value + dx);
    // Ensure we are not near zero in the z-direction
    // if (LandmarkRepresentation::is_relative_representation(_feat_representation) && _value(_value.rows() - 1) < 1e-8) {
    //  PRINT_DEBUG(YELLOW "WARNING DEPTH %.8f BECAME CLOSE TO ZERO IN UPDATE!!!\n" RESET, _value(_value.rows() - 1));
    //  should_marg = true;
    // }
  }

  /**
   * @brief Will return the position of the feature in the global frame of reference.
   * @param getfej Set to true to get the landmark FEJ value
   * @return Position of feature either in global or anchor frame
   */
  Eigen::Matrix<double, 3, 1> get_xyz(bool getfej) const;

  /**
   * @brief Will set the current value based on the representation.
   * @param p_FinG Position of the feature either in global or anchor frame
   * @param isfej Set to true to set the landmark FEJ value
   */
  void set_from_xyz(Eigen::Matrix<double, 3, 1> p_FinG, bool isfej);
};
} // namespace ov_type

#endif // OV_TYPE_TYPE_LANDMARK_H
