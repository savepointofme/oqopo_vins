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

#ifndef OV_CORE_TRACK_DESC_H
#define OV_CORE_TRACK_DESC_H

#include "TrackBase.h"

#ifdef USE_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace ov_core {

/**
 * @brief Descriptor-based visual tracking
 *
 * Here we use descriptor matching to track features from one frame to the next.
 * We track both temporally, and across stereo pairs to get stereo constraints.
 * Right now we use ORB descriptors as we have found it is the fastest when computing descriptors.
 * Tracks are then rejected based on a ratio test and ransac.
 */
class TrackDescriptor : public TrackBase {

public:
  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   * @param fast_threshold FAST detection threshold
   * @param gridx size of grid in the x-direction / u-direction
   * @param gridy size of grid in the y-direction / v-direction
   * @param minpxdist features need to be at least this number pixels away from each other
   * @param knnratio matching ratio needed (smaller value forces top two descriptors during match to be more different)
   */
  explicit TrackDescriptor(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numfeats, int numaruco, bool stereo,
                           HistogramMethod histmethod, int fast_threshold, int gridx, int gridy, int minpxdist, double knnratio,
                           double max_match_px_dist = 0.0, const std::string &xfeat_model_path = "",
                           const std::string &sp_model_path = "")
      : TrackBase(cameras, numfeats, numaruco, stereo, histmethod), threshold(fast_threshold), grid_x(gridx), grid_y(gridy),
        min_px_dist(minpxdist), knn_ratio(knnratio), max_match_px_dist_(max_match_px_dist) {
#ifdef USE_ONNXRUNTIME
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(2);
    if (!sp_model_path.empty()) {
      sp_session_ = std::make_unique<Ort::Session>(ort_env_, sp_model_path.c_str(), opts);
      use_sp_ = true;
      PRINT_INFO("[SP] Loaded SuperPoint model: %s\n", sp_model_path.c_str());
    } else if (!xfeat_model_path.empty()) {
      xfeat_session_ = std::make_unique<Ort::Session>(ort_env_, xfeat_model_path.c_str(), opts);
      use_xfeat_ = true;
      PRINT_INFO("[XFEAT] Loaded model: %s\n", xfeat_model_path.c_str());
    }
#else
    if (!sp_model_path.empty() || !xfeat_model_path.empty())
      PRINT_WARNING("[NEURAL] model path set but built without USE_ONNXRUNTIME — falling back to ORB\n");
#endif
  }

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_new_camera(const CameraData &message) override;

  /**
   * @brief Return per-frame descriptor tracking diagnostics for the given camera.
   * Populated fields: n_desc_detected, n_desc_pre_gate, n_desc_post_gate, n_desc_post_ransac.
   */
  bool get_warp_viz_packet(size_t cam_id, TrackerWarpVizPacket &packet) override;

protected:
  /**
   * @brief Process a new monocular image
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id the camera index in message data vector
   */
  void feed_monocular(const CameraData &message, size_t msg_id);

  /**
   * @brief Process new stereo pair of images
   * @param message Contains our timestamp, images, and camera ids
   * @param msg_id_left first image index in message data vector
   * @param msg_id_right second image index in message data vector
   */
  void feed_stereo(const CameraData &message, size_t msg_id_left, size_t msg_id_right);

  /**
   * @brief Detects new features in the current image
   * @param img0 image we will detect features on
   * @param mask0 mask which has what ROI we do not want features in
   * @param pts0 vector of extracted keypoints
   * @param desc0 vector of the extracted descriptors
   * @param ids0 vector of all new IDs
   *
   * Given a set of images, and their currently extracted features, this will try to add new features.
   * We return all extracted descriptors here since we DO NOT need to do stereo tracking left to right.
   * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
   * See robust_match() for the matching.
   */
  void perform_detection_monocular(const cv::Mat &img0, const cv::Mat &mask0, std::vector<cv::KeyPoint> &pts0, cv::Mat &desc0,
                                   std::vector<size_t> &ids0, const std::vector<cv::KeyPoint> *pts_hint = nullptr);

  /// XFeat neural detector (64-dim float descriptors, 3-channel BGR input).
  void detect_xfeat(const cv::Mat &img, const cv::Mat &mask, std::vector<cv::KeyPoint> &pts, cv::Mat &desc,
                    std::vector<size_t> &ids, const std::vector<cv::KeyPoint> *pts_hint = nullptr);

  /// SuperPoint neural detector (256-dim float descriptors, 1-channel grayscale input).
  /// pts_hint: previous frame's keypoints for temporal continuity (same as XFeat hint).
  /// SP outputs integer NMS keypoints so hint does not cause phantom motion.
  void detect_superpoint(const cv::Mat &img, const cv::Mat &mask, std::vector<cv::KeyPoint> &pts, cv::Mat &desc,
                         std::vector<size_t> &ids, const std::vector<cv::KeyPoint> *pts_hint = nullptr);

  /**
   * @brief Detects new features in the current stereo pair
   * @param img0 left image we will detect features on
   * @param img1 right image we will detect features on
   * @param mask0 mask which has what ROI we do not want features in
   * @param mask1 mask which has what ROI we do not want features in
   * @param pts0 left vector of new keypoints
   * @param pts1 right vector of new keypoints
   * @param desc0 left vector of extracted descriptors
   * @param desc1 left vector of extracted descriptors
   * @param cam_id0 id of the first camera
   * @param cam_id1 id of the second camera
   * @param ids0 left vector of all new IDs
   * @param ids1 right vector of all new IDs
   *
   * This does the same logic as the perform_detection_monocular() function, but we also enforce stereo contraints.
   * We also do STEREO matching from the left to right, and only return good matches that are found in both the left and right.
   * Our vector of IDs will be later overwritten when we match features temporally to the previous frame's features.
   * See robust_match() for the matching.
   */
  void perform_detection_stereo(const cv::Mat &img0, const cv::Mat &img1, const cv::Mat &mask0, const cv::Mat &mask1,
                                std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1, cv::Mat &desc0, cv::Mat &desc1,
                                size_t cam_id0, size_t cam_id1, std::vector<size_t> &ids0, std::vector<size_t> &ids1);

  /**
   * @brief Find matches between two keypoint+descriptor sets.
   * @param pts0 first vector of keypoints
   * @param pts1 second vector of keypoints
   * @param desc0 first vector of descriptors
   * @param desc1 second vector of decriptors
   * @param id0 id of the first camera
   * @param id1 id of the second camera
   * @param matches vector of matches that we have found
   *
   * This will perform a "robust match" between the two sets of points (slow but has great results).
   * First we do a simple KNN match from 1to2 and 2to1, which is followed by a ratio check and symmetry check.
   * Original code is from the "RobustMatcher" in the opencv examples, and seems to give very good results in the matches.
   * https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
   */
  void robust_match(const std::vector<cv::KeyPoint> &pts0, const std::vector<cv::KeyPoint> &pts1, const cv::Mat &desc0,
                    const cv::Mat &desc1, size_t id0, size_t id1, std::vector<cv::DMatch> &matches);

  // Helper functions for the robust_match function
  // Original code is from the "RobustMatcher" in the opencv examples
  // https://github.com/opencv/opencv/blob/master/samples/cpp/tutorial_code/calib3d/real_time_pose_estimation/src/RobustMatcher.cpp
  void robust_ratio_test(std::vector<std::vector<cv::DMatch>> &matches);
  void robust_symmetry_test(std::vector<std::vector<cv::DMatch>> &matches1, std::vector<std::vector<cv::DMatch>> &matches2,
                            std::vector<cv::DMatch> &good_matches);

  // Timing variables
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // Our orb extractor
  cv::Ptr<cv::ORB> orb0 = cv::ORB::create();
  cv::Ptr<cv::ORB> orb1 = cv::ORB::create();

  // Our descriptor matcher (Hamming for ORB; overridden to L2 for float descriptors inside robust_match)
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  // Neural extractors
  bool use_xfeat_ = false;
  bool use_sp_    = false;
#ifdef USE_ONNXRUNTIME
  Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "neural_tracker"};
  std::unique_ptr<Ort::Session> xfeat_session_;
  std::unique_ptr<Ort::Session> sp_session_;
#endif

  // Parameters for our FAST grid detector
  int threshold;
  int grid_x;
  int grid_y;

  // Minimum pixel distance to be "far away enough" to be a different extracted feature
  int min_px_dist;

  // The ratio between two kNN matches, if that ratio is larger then this threshold
  // then the two features are too close, so should be considered ambiguous/bad match
  double knn_ratio;

  // Max pixel distance for spatial gating of descriptor matches (0 = disabled).
  // Matches where |pt_prev - pt_curr| > this value are discarded before RANSAC.
  double max_match_px_dist_ = 0.0;

  // Per-call intermediate counts written by robust_match and consumed by feed_monocular.
  // Not thread-safe for simultaneous calls on the same instance, but monocular mode is fine.
  int match_n_knn_      = 0; // total kNN pairs before any filtering
  int match_n_ratio_    = 0; // after Lowe ratio test (one direction)
  int match_pre_gate_   = 0; // after symmetry test (= pre spatial gate)
  int match_post_gate_  = 0; // after spatial gate
  int match_n_ransac_   = 0; // RANSAC inliers (final geometric matches)

public:
  // Whether to pass previous frame's feature locations as detection hints (Method 2).
  // When true, detect_xfeat prioritises candidates near previous features before grid-fill.
  // Settable from VioManager after construction so it can be toggled from YAML.
  bool xfeat_temporal_hint_ = true;

protected:

  // Per-feature consecutive-frame age counter (for track-lifetime histogram).
  // Maps feature_id → number of consecutive frames it has been tracked.
  std::unordered_map<size_t, int> track_age_;

  // Per-camera diagnostic packets returned by get_warp_viz_packet().
  std::unordered_map<size_t, TrackerWarpVizPacket> desc_viz_packets_;
  std::mutex mtx_desc_diag_;

  // Descriptor matrices
  std::unordered_map<size_t, cv::Mat> desc_last;
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_DESC_H */
