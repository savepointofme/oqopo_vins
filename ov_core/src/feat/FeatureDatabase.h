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

#ifndef OV_CORE_FEATURE_DATABASE_H
#define OV_CORE_FEATURE_DATABASE_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace ov_core {

class Feature;

/**
 * @brief Database containing features we are currently tracking.
 *
 * Each visual tracker has this database in it and it contains all features that we are tracking.
 * The trackers will insert information into this database when they get new measurements from doing tracking.
 * A user would then query this database for features that can be used for update and remove them after they have been processed.
 *
 *
 * @m_class{m-note m-warning}
 *
 * @par A Note on Multi-Threading Support
 * There is some support for asynchronous multi-threaded access.
 * Since each feature is a pointer just directly returning and using them is not thread safe.
 * Thus, to be thread safe, use the "remove" flag for each function which will remove it from this feature database.
 * This prevents the trackers from adding new measurements and editing the feature information.
 * For example, if you are asynchronous tracking cameras and you chose to update the state, then remove all features you will use in update.
 * The feature trackers will continue to add features while you update, whose measurements can be used in the next update step!
 *
 */
// =============================================================================
// [中文] FeatureDatabase — 前端特征的库 (共享于所有跟踪器 / 滤波器)
//
// 定位: Frontend 插入 + Backend 查询/清理的中间层。
//   - TrackBase (KLT / Descriptor / Aruco) 每帧进来调 update_feature 写入;
//   - VioManager/Initializer 用 features_containing / features_not_containing_newer 等查询;
//   - 更新后用 cleanup / cleanup_measurements 删除过期观测。
//
// 内部结构:
//   features_idlookup : feat_id → shared_ptr<Feature>  (Feature 内部存多帧多相机的 uv/uv_norm)
//   mtx               : 保护映射的互斥锁, 适应跟踪线程+滤波线程异步访问。
//
// 线程安全契约: 返回的 shared_ptr<Feature> 指向库里的原本。如果是异步使用, 一定要传 remove=true,
// 这样库里引用移除, 后续 TrackBase 再 push 新观测也不会改写你手上的特征。
// =============================================================================
class FeatureDatabase {

public:
  /**
   * @brief Default constructor
   */
  FeatureDatabase() {}

  /**
   * @brief Get a specified feature
   * @param id What feature we want to get
   * @param remove Set to true if you want to remove the feature from the database (you will need to handle the freeing of memory)
   * @return Either a feature object, or null if it is not in the database.
   */
  std::shared_ptr<Feature> get_feature(size_t id, bool remove = false);

  /**
   * @brief Get a specified feature clone (pointer is thread safe)
   * @param id What feature we want to get
   * @param feat Feature with data in it
   * @return True if the feature was found
   */
  bool get_feature_clone(size_t id, Feature &feat);

  /**
   * @brief Update a feature object
   * @param id ID of the feature we will update
   * @param timestamp time that this measurement occured at
   * @param cam_id which camera this measurement was from
   * @param u raw u coordinate
   * @param v raw v coordinate
   * @param u_n undistorted/normalized u coordinate
   * @param v_n undistorted/normalized v coordinate
   *
   * This will update a given feature based on the passed ID it has.
   * It will create a new feature, if it is an ID that we have not seen before.
   */
  void update_feature(size_t id, double timestamp, size_t cam_id, float u, float v, float u_n, float v_n);

  /**
   * @brief Get features that do not have newer measurement then the specified time.
   *
   * This function will return all features that do not a measurement at a time greater than the specified time.
   * For example this could be used to get features that have not been successfully tracked into the newest frame.
   * All features returned will not have any measurements occurring at a time greater then the specified.
   */
  std::vector<std::shared_ptr<Feature>> features_not_containing_newer(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief Get features that has measurements older then the specified time.
   *
   * This will collect all features that have measurements occurring before the specified timestamp.
   * For example, we would want to remove all features older then the last clone/state in our sliding window.
   */
  std::vector<std::shared_ptr<Feature>> features_containing_older(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief Get features that has measurements at the specified time.
   *
   * This function will return all features that have the specified time in them.
   * This would be used to get all features that occurred at a specific clone/state.
   */
  // [中文] features_containing: 取所有 "在 timestamp 那一帧被观测到" 的特征。
  //   用来建特定 clone 位姿的观测连接 (比如 MSCKF 对一个克隆的全特征更新)。
  //   remove=true       会从库里引用移除该特征 (避免事后被跟踪线程改写)。
  //   skip_deleted=true 则跳过已被标记删除的特征 (竞争下很颇重要)。
  std::vector<std::shared_ptr<Feature>> features_containing(double timestamp, bool remove = false, bool skip_deleted = false);

  /**
   * @brief This function will delete all features that have been used up.
   *
   * If a feature was unable to be used, it will still remain since it will not have a delete flag set
   */
  // [中文] cleanup: 清理已标记 to_delete 的特征 (MSCKF 更新后的清扫)。
  //   没被用到的特征不会动 — 等后续帧继续观测或变老后被 cleanup_measurements 清除。
  void cleanup();

  /**
   * @brief This function will delete all feature measurements that are older then the specified timestamp
   */
  // [中文] cleanup_measurements: 清理在 timestamp 之前的所有观测记录。
  //   用于滤波滑窗向前推进时, 丢掉 "不再在序列里" 的历史观测 (限制内存 + 避免旧数据干扰)。
  //   注意: 只删某条特征内部的旧观测, 特征本身保留 (可能仍有新观测); 若特征观测列表清空才删除。
  //   初始化中 InertialInitializer::initialize Step 2 会调用它, 保证窗口规范。
  void cleanup_measurements(double timestamp);

  /**
   * @brief This function will delete all feature measurements that are at the specified timestamp
   */
  void cleanup_measurements_exact(double timestamp);

  /**
   * @brief Remove complete tracks for feature IDs whose measurements have
   * already been consumed by an external batch posterior.
   * @return Number of tracks removed from the database.
   */
  size_t remove_features(const std::vector<size_t> &feature_ids);

  /**
   * @brief Returns the size of the feature database
   */
  size_t size() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup.size();
  }

  /**
   * @brief Returns the internal data (should not normally be used)
   */
  // [中文] get_internal_data: 返回整个 feat_id→Feature 的映射的 "浅拷贝" (shared_ptr 拷贝)。
  //   一般用在 InertialInitializer 里扫描最新相机帧时间, 不推荐热点路径上小调。
  std::unordered_map<size_t, std::shared_ptr<Feature>> get_internal_data() {
    std::lock_guard<std::mutex> lck(mtx);
    return features_idlookup;
  }

  /**
   * @brief Gets the oldest time in the database
   */
  double get_oldest_timestamp();

  /**
   * @brief Will update the passed database with this database's latest feature information.
   */
  void append_new_measurements(const std::shared_ptr<FeatureDatabase> &database);

protected:
  /// Mutex lock for our map
  std::mutex mtx;

  /// Our lookup array that allow use to query based on ID
  std::unordered_map<size_t, std::shared_ptr<Feature>> features_idlookup;
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_DATABASE_H */
