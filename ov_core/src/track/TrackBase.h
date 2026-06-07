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

#ifndef OV_CORE_TRACK_BASE_H
#define OV_CORE_TRACK_BASE_H

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "utils/colors.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

namespace ov_core {

class Feature;
class CamBase;
class FeatureDatabase;

/**
 * @brief Per-camera visualization packet owned by TrackKLT.
 *
 * Contains the exact warped image and warped points that TrackKLT produced
 * during the last feed_new_camera call, together with the final accepted
 * current-frame matches. This is the single source of truth for the cross-pane
 * matching visualization — no independent homography path exists elsewhere.
 */
struct TrackerWarpVizPacket {
  bool valid = false;
  bool warp_active = false;
  size_t cam_id = 0;
  double t_curr = 0.0;
  cv::Mat prev_image_for_viz;                 // warped prev if warp active, else raw prev
  cv::Mat curr_raw_image;                     // current raw image
  std::vector<cv::Point2f> prev_pts_for_viz;  // warped prev pts if warp active, else raw prev pts
  std::vector<cv::Point2f> curr_pts_raw;      // final accepted matches in raw current image
  std::vector<size_t> feature_ids;            // matching feature IDs
  // Diagnostic counters — populated by TrackKLT::feed_monocular
  int n_klt_attempted = 0;   // total points entering KLT (prev tracks + newly detected on prev frame)
  int n_newly_detected = 0;  // FAST corners added to prev frame this step
  // Descriptor-tracker diagnostics — populated by TrackDescriptor, zero for KLT
  int n_desc_detected    = 0;  // FAST keypoints detected this frame
  int n_desc_pre_gate    = 0;  // matches after kNN+ratio+symmetry, before spatial gate
  int n_desc_post_gate   = 0;  // matches after spatial gate (== pre_gate when gate disabled)
  int n_desc_post_ransac = 0;  // matches surviving RANSAC (= tracked from previous frame)
};

/**
 * @brief Visual feature tracking base class
 *
 * This is the base class for all our visual trackers.
 * The goal here is to provide a common interface so all underlying trackers can simply hide away all the complexities.
 * We have something called the "feature database" which has all the tracking information inside of it.
 * The user can ask this database for features which can then be used in an MSCKF or batch-based setting.
 * The feature tracks store both the raw (distorted) and undistorted/normalized values.
 * Right now we just support two camera models, see: undistort_point_brown() and undistort_point_fisheye().
 *
 * @m_class{m-note m-warning}
 *
 * @par A Note on Multi-Threading Support
 * There is some support for asynchronous multi-threaded feature tracking of independent cameras.
 * The key assumption during implementation is that the user will not try to track on the same camera in parallel, and instead call on
 * different cameras. For example, if I have two cameras, I can either sequentially call the feed function, or I spin each of these into
 * separate threads and wait for their return. The @ref currid is atomic to allow for multiple threads to access it without issue and ensure
 * that all features have unique id values. We also have mutex for access for the calibration and previous images and tracks (used during
 * visualization). It should be noted that if a thread calls visualization, it might hang or the feed thread might, due to acquiring the
 * mutex for that specific camera id / feed.
 *
 * This base class also handles most of the heavy lifting with the visualization, but the sub-classes can override
 * this and do their own logic if they want (i.e. the TrackAruco has its own logic for visualization).
 * This visualization needs access to the prior images and their tracks, thus must synchronise in the case of multi-threading.
 * This shouldn't impact performance, but high frequency visualization calls can negatively effect the performance.
 */
class TrackBase {

public:
  /**
   * @brief Desired pre-processing image method.
   */
  enum HistogramMethod { NONE, HISTOGRAM, CLAHE };

  /**
   * @brief Public constructor with configuration variables
   * @param cameras camera calibration object which has all camera intrinsics in it
   * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
   * @param numaruco the max id of the arucotags, so we ensure that we start our non-auroc features above this value
   * @param stereo if we should do stereo feature tracking or binocular
   * @param histmethod what type of histogram pre-processing should be done (histogram eq?)
   */
  TrackBase(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numfeats, int numaruco, bool stereo,
            HistogramMethod histmethod);

  virtual ~TrackBase() {}

  /**
   * @brief Process a new image
   * @param message Contains our timestamp, images, and camera ids
   */
  virtual void feed_new_camera(const CameraData &message) = 0;

  /**
   * @brief Shows features extracted in the last image
   * @param img_out image to which we will overlayed features on
   * @param r1,g1,b1 first color to draw in
   * @param r2,g2,b2 second color to draw in
   * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
   */
  virtual void display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::string overlay = "");

  /**
   * @brief Shows a "trail" for each feature (i.e. its history)
   * @param img_out image to which we will overlayed features on
   * @param r1,g1,b1 first color to draw in
   * @param r2,g2,b2 second color to draw in
   * @param highlighted unique ids which we wish to highlight (e.g. slam feats)
   * @param overlay Text overlay to replace to normal "cam0" in the top left of screen
   */
  virtual void display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2, std::vector<size_t> highlighted = {},
                               std::string overlay = "");

  /**
   * @brief Get the feature database with all the track information
   * @return FeatureDatabase pointer that one can query for features
   */
  std::shared_ptr<FeatureDatabase> get_feature_database() { return database; }

  /**
   * @brief Changes the ID of an actively tracked feature to another one.
   *
   * This function can be helpfull if you detect a loop-closure with an old frame.
   * One could then change the id of an active feature to match the old feature id!
   *
   * @param id_old Old id we want to change
   * @param id_new Id we want to change the old id to
   */
  void change_feat_id(size_t id_old, size_t id_new);

  /// Getter method for active features in the last frame (observations per camera)
  std::unordered_map<size_t, std::vector<cv::KeyPoint>> get_last_obs() {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    return pts_last;
  }

  /// Getter method for active features in the last frame (ids per camera)
  std::unordered_map<size_t, std::vector<size_t>> get_last_ids() {
    std::lock_guard<std::mutex> lckv(mtx_last_vars);
    return ids_last;
  }

  /// Getter method for number of active features
  int get_num_features() { return num_features; }

  /// Setter method for number of active features
  void set_num_features(int _num_features) { num_features = _num_features; }

  /**
   * @brief Set the predicted inter-frame rotation in camera frame.
   *
   * The caller (estimator) should integrate gyroscope readings between the
   * previous and current image timestamps (with current best gyro-bias
   * estimate subtracted) to obtain a relative rotation, expressed in the
   * camera coordinate frame, that maps a normalized direction observed at the
   * previous image to its predicted direction at the current image:
   *
   *    p_C_curr = R_prev_to_curr * p_C_prev  (for a point at infinity)
   *
   * Equivalently, the homography for any finite-depth point under pure
   * rotation is H = K * R_prev_to_curr * K^{-1}, where K is the camera
   * intrinsics. KLT will use H to warp the previous feature pixel locations
   * into a much better initial guess for the LK solver than the standard
   * "zero motion / use previous pixel" assumption.
   *
   * This must be called BEFORE feed_new_camera() for the matching frame, on
   * a per-camera basis. After feed_new_camera() consumes the prediction, the
   * tracker clears the entry for that camera id so a stale rotation is never
   * applied to a later frame.
   *
   * If no rotation is set for a camera, the tracker falls back to the legacy
   * behaviour (identity rotation, i.e. previous pixel as the LK seed).
   *
   * @param cam_id Camera id (same as in CameraData::sensor_ids)
   * @param R_prev_to_curr Rotation from previous frame (cam_id) to current
   *                       frame (cam_id), expressed in camera coordinates.
   */
  void set_predicted_rotation(size_t cam_id, const cv::Matx33d &R_prev_to_curr);

  /// Clear all predicted rotations. Called automatically after each
  /// feed_new_camera() so the prediction is consumed exactly once.
  void clear_predicted_rotations();

  /// Convenience accessor: returns true if a rotation has been set for the
  /// given camera id, and writes it into @p R_out.
  bool get_predicted_rotation(size_t cam_id, cv::Matx33d &R_out);


  /// Accessor for the latest per-camera warp visualization packet.
  /// Base returns false; TrackKLT overrides with the real packet.
  virtual bool get_warp_viz_packet(size_t cam_id, TrackerWarpVizPacket &packet) { return false; }

  /**
   * @brief Set a gravity-aligned rotation warp for the given camera.
   *
   * When set, TrackKLT applies H = K * R_comp * K^{-1} to warp both the
   * "last" and "current" image into a gravity-aligned frame before running
   * calcOpticalFlowPyrLK. Tracked feature positions are un-warped back to
   * original image coordinates before FeatureDatabase storage, so the
   * estimator's camera model is unaffected.
   *
   * This is a TRACKING-PATH operation (not visualization-only). It reduces
   * rotational optical-flow contamination of translational parallax during
   * and after turns, which is the root cause of post-turn scale/height jumps.
   *
   * Must be called BEFORE feed_new_camera() for the matching frame. After
   * feed_new_camera() the caller must call clear_gravity_warps() so the
   * warp is not applied to a later frame. Controlled by
   * use_ground_parallel_warp in VioManagerOptions.
   *
   * @param cam_id  Camera id (same as in CameraData::sensor_ids).
   * @param R_comp  Rotation to apply: warped_ray = R_comp * original_ray.
   *                Typically R_GtoC_ref * R_GtoC_curr^T (reference orientation
   *                relative to current).
   */
  void set_gravity_warp(size_t cam_id, const cv::Matx33d &R_comp);

  /// Clear all gravity warps. Called after each feed_new_camera().
  void clear_gravity_warps();

  /// Returns true if a gravity warp is set for the given camera, writes it to R_out.
  bool get_gravity_warp(size_t cam_id, cv::Matx33d &R_out);

protected:
  /// Camera object which has all calibration in it
  std::unordered_map<size_t, std::shared_ptr<CamBase>> camera_calib;

  /// Database with all our current features
  std::shared_ptr<FeatureDatabase> database;

  /// If we are a fisheye model or not
  std::map<size_t, bool> camera_fisheye;

  /// Number of features we should try to track frame to frame
  int num_features;

  /// If we should use binocular tracking or stereo tracking for multi-camera
  bool use_stereo;

  /// What histogram equalization method we should pre-process images with?
  HistogramMethod histogram_method;

  /// Mutexs for our last set of image storage (img_last, pts_last, and ids_last)
  std::vector<std::mutex> mtx_feeds;

  /// Mutex for editing the *_last variables
  std::mutex mtx_last_vars;

  /// Last set of images (use map so all trackers render in the same order)
  std::map<size_t, cv::Mat> img_last;

  /// Last set of images (use map so all trackers render in the same order)
  std::map<size_t, cv::Mat> img_mask_last;

  /// Last set of tracked points
  std::unordered_map<size_t, std::vector<cv::KeyPoint>> pts_last;

  /// Set of IDs of each current feature in the database
  std::unordered_map<size_t, std::vector<size_t>> ids_last;

  /// Master ID for this tracker (atomic to allow for multi-threading)
  std::atomic<size_t> currid;

  // Timing variables (most children use these...)
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  /// Per-camera predicted inter-frame rotation (set by the estimator before
  /// each feed_new_camera; consumed once and cleared by the tracker).
  std::mutex mtx_predicted_rotation;
  std::unordered_map<size_t, cv::Matx33d> predicted_rotation_per_cam;

  /// Per-camera gravity-aligned warp rotation R_comp (set by VioManager before
  /// each feed_new_camera when use_ground_parallel_warp is enabled; cleared after).
  std::mutex mtx_gravity_warp;
  std::unordered_map<size_t, cv::Matx33d> gravity_warp_per_cam;
};

} // namespace ov_core

#endif /* OV_CORE_TRACK_BASE_H */
