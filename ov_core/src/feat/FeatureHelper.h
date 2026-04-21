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

#ifndef OV_CORE_FEATURE_HELPER_H
#define OV_CORE_FEATURE_HELPER_H

#include <Eigen/Eigen>
#include <memory>
#include <mutex>
#include <vector>

#include "Feature.h"
#include "FeatureDatabase.h"
#include "utils/print.h"

namespace ov_core {

/**
 * @brief Contains some nice helper functions for features.
 *
 * These functions should only depend on feature and the feature database.
 */
class FeatureHelper {

public:
  /**
   * @brief This functions will compute the disparity between common features in the two frames.
   *
   * First we find all features in the first frame.
   * Then we loop through each and find the uv of it in the next requested frame.
   * Features are skipped if no tracked feature is found (it was lost).
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param time0 First camera frame timestamp
   * @param time1 Second camera frame timestamp
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   */
  // [中文] compute_disparity (两时刻版): 计算 time0 和 time1 两帧之间 "共同可见特征" 的平均视差。
  //   视差 (disparity) = 同一特征在相邻两帧上像素位置差的 L2 范数, 单位是像素。
  //   它是判断相机是否在运动的最直接信号: 视差大 → 有运动; 视差小 → 静止。
  //   注意: (1) 用 RAW uv (未去畸变); (2) 所有相机合起来统计; (3) 返回 mean/std。
  //   调用链: InertialInitializer::initialize, VioManager::do_feature_propagate_update 等。
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, double time0, double time1, double &disp_mean,
                                double &disp_var, int &total_feats) {

    // Get features seen from the first image
    // [中文] 先在特征库中找到 time0 这帧所含的所有特征 (features_containing):
    //   第 2 个参 remove=false (不从库里删), 第 3 个参 has_timestamp=true 要求特征确实包含 time0。
    std::vector<std::shared_ptr<Feature>> feats0 = db->features_containing(time0, false, true);

    // Compute the disparity
    // [中文] disparities: 全部配对 (feat, camid) 的视差样本。
    std::vector<double> disparities;
    for (auto &feat : feats0) {

      // Get the two uvs for both times
      // [中文] 同一个特征可能在多个相机被观测到 (立体/多目), 每个相机各贡献一个视差样本。
      for (auto &campairs : feat->timestamps) {

        // First find the two timestamps
        // [中文] 在该 camid 的 timestamps 序列里查找 time0 / time1 的位置下标。
        //   若两个中任何一个没找到 → 该特征在这个相机上不是两帧共见, 跳过。
        size_t camid = campairs.first;
        auto it0 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time0);
        auto it1 = std::find(feat->timestamps.at(camid).begin(), feat->timestamps.at(camid).end(), time1);
        if (it0 == feat->timestamps.at(camid).end() || it1 == feat->timestamps.at(camid).end())
          continue;
        auto idx0 = std::distance(feat->timestamps.at(camid).begin(), it0);
        auto idx1 = std::distance(feat->timestamps.at(camid).begin(), it1);

        // Now lets calculate the disparity
        // [中文] feat->uvs.at(cam).at(idx) 存的是三维 (u, v, 1) 齐次坐标, 这里只取前两维。
        //   然后求 uv1 与 uv0 的欧氏距离, 即该特征在两帧上的平移像素数。
        Eigen::Vector2f uv0 = feat->uvs.at(camid).at(idx0).block(0, 0, 2, 1);
        Eigen::Vector2f uv1 = feat->uvs.at(camid).at(idx1).block(0, 0, 2, 1);
        disparities.push_back((uv1 - uv0).norm());
      }
    }

    // If no disparities, just return
    // [中文] 样本不足 2 个 → 无法算样本方差, 输出 -1 作为失败标志。
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    // [中文] 算样本均值 μ = (1/N) Σ d_i, 然后算样本标准差 σ = sqrt((1/(N-1)) Σ (d_i-μ)²)。
    //   变量名上是 "disp_var" 但实际存的是标准差 (sqrt(var)), 用于后面直接和像素级阈值比较。
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

  /**
   * @brief This functions will compute the disparity over all features we have
   *
   * NOTE: this is on the RAW coordinates of the feature not the normalized ones.
   * NOTE: This computes the disparity over all cameras!
   *
   * @param db Feature database pointer
   * @param disp_mean Average raw disparity
   * @param disp_var Variance of the disparities
   * @param total_feats Total number of common features
   * @param newest_time Only compute disparity for ones older (-1 to disable)
   * @param oldest_time Only compute disparity for ones newer (-1 to disable)
   */
  // [中文] compute_disparity (区间版): 不指定具体两帧, 而是对符合时间範围的每个特征,
  //   拿它在区间内的第一帧和最后一帧计算视差。
  //   参数:
  //     oldest_time : 只看时间 > oldest_time 的观测 (-1 禁用下界)
  //     newest_time : 只看时间 < newest_time 的观测 (-1 禁用上界)
  //   常用于“最近一段时间系统是否在动”的粗略判断。
  static void compute_disparity(std::shared_ptr<ov_core::FeatureDatabase> db, double &disp_mean, double &disp_var, int &total_feats,
                                double newest_time = -1, double oldest_time = -1) {

    // Compute the disparity
    std::vector<double> disparities;
    // [中文] 遍历数据库里每个特征, 对其每个相机的观测序列:
    for (auto &feat : db->get_internal_data()) {
      for (auto &campairs : feat.second->timestamps) {

        // Skip if only one observation
        // [中文] 只有 1 次观测 → 不能算视差, 跳过。
        if (campairs.second.size() < 2)
          continue;

        // Now lets calculate the disparity (assumes time array is monotonic)
        // [中文] 遍历观测序列 (已按时间递增), 按顺序捕获第一个 (uv0) 和最后一个 (uv1) 合格观测:
        //   - found0==false: 还没找到第一个, time > oldest_time 就映为 uv0
        //   - found0==true : 正在找第二个, time < newest_time 就映为 uv1 (每次 loop 更新, 最后一次就是最晰)
        size_t camid = campairs.first;
        bool found0 = false;
        bool found1 = false;
        Eigen::Vector2f uv0 = Eigen::Vector2f::Zero();
        Eigen::Vector2f uv1 = Eigen::Vector2f::Zero();
        for (size_t idx = 0; idx < feat.second->timestamps.at(camid).size(); idx++) {
          double time = feat.second->timestamps.at(camid).at(idx);
          if ((oldest_time == -1 || time > oldest_time) && !found0) {
            uv0 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1);
            found0 = true;
            continue;
          }
          if ((newest_time == -1 || time < newest_time) && found0) {
            uv1 = feat.second->uvs.at(camid).at(idx).block(0, 0, 2, 1);
            found1 = true;
            continue;
          }
        }

        // If we found both an old and a new time, then we are good!
        // [中文] 前后两端观测都找齐了才算一次视差样本。
        if (!found0 || !found1)
          continue;
        disparities.push_back((uv1 - uv0).norm());
      }
    }

    // If no disparities, just return
    // [中文] 同上一个 overload: 样本不足 2 个, 输出 -1 作为失败标志。
    if (disparities.size() < 2) {
      disp_mean = -1;
      disp_var = -1;
      total_feats = 0;
    }

    // Compute mean and standard deviation in respect to it
    // [中文] 样本均值 μ 与样本标准差 σ (存入 disp_var)。
    disp_mean = 0;
    for (double disp_i : disparities) {
      disp_mean += disp_i;
    }
    disp_mean /= (double)disparities.size();
    disp_var = 0;
    for (double &disp_i : disparities) {
      disp_var += std::pow(disp_i - disp_mean, 2);
    }
    disp_var = std::sqrt(disp_var / (double)(disparities.size() - 1));
    total_feats = (int)disparities.size();
  }

private:
  // Cannot construct this class
  FeatureHelper() {}
};

} // namespace ov_core

#endif /* OV_CORE_FEATURE_HELPER_H */