/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

/**
 * @file test_state_sim3_scale_reset.cpp
 *
 * Focused unit coverage for StateHelper::apply_sim3_scale_reset(). This is
 * intentionally a standalone source so the production reset can be reviewed
 * without changing build-system ownership in this branch.
 */

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/Landmark.h"
#include "utils/NoiseManager.h"
#include "utils/quat_ops.h"

using namespace ov_core;
using namespace ov_msckf;
using namespace ov_type;

namespace {

constexpr double kTight = 1e-10;

struct Checks {
  int passed = 0;
  int failed = 0;

  void expect(bool condition, const std::string &message) {
    if (condition) {
      ++passed;
      return;
    }
    ++failed;
    std::cerr << "[FAIL] " << message << "\n";
  }
};

bool near_matrix(const Eigen::MatrixXd &a, const Eigen::MatrixXd &b,
                 double tolerance = kTight) {
  if (a.rows() != b.rows() || a.cols() != b.cols() || !a.allFinite() ||
      !b.allFinite())
    return false;
  const double scale = std::max(1.0, std::max(a.norm(), b.norm()));
  return (a - b).norm() <= tolerance * scale;
}

Eigen::Matrix3d rotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d out;
  out << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return out;
}

Eigen::Matrix<double, 7, 1> pose_value(const Eigen::Matrix3d &R_GtoI,
                                       const Eigen::Vector3d &p_IinG) {
  Eigen::Matrix<double, 7, 1> value;
  value.block<4, 1>(0, 0) = rot_2_quat(R_GtoI);
  value.block<3, 1>(4, 0) = p_IinG;
  return value;
}

Eigen::Matrix<double, 16, 1> imu_value(
    const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
    const Eigen::Vector3d &v_IinG, const Eigen::Vector3d &bg,
    const Eigen::Vector3d &ba) {
  Eigen::Matrix<double, 16, 1> value;
  value.block<4, 1>(0, 0) = rot_2_quat(R_GtoI);
  value.block<3, 1>(4, 0) = p_IinG;
  value.block<3, 1>(7, 0) = v_IinG;
  value.block<3, 1>(10, 0) = bg;
  value.block<3, 1>(13, 0) = ba;
  return value;
}

struct CameraTerms {
  Eigen::Vector3d lever_in_imu;
  Eigen::Vector3d lever_in_global;
  Eigen::Vector3d center;
  Eigen::Matrix3d dcenter_dpose_theta;
  Eigen::Matrix3d dcenter_dextrinsic_theta;
  Eigen::Matrix3d dcenter_dextrinsic_position;
};

CameraTerms camera_terms(const Eigen::Matrix3d &R_GtoI,
                         const Eigen::Vector3d &p_IinG,
                         const Eigen::Matrix3d &R_ItoC,
                         const Eigen::Vector3d &p_IinC) {
  CameraTerms out;
  const Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
  const Eigen::Matrix3d R_CtoI = R_ItoC.transpose();
  out.lever_in_imu = -R_CtoI * p_IinC;
  out.lever_in_global = R_ItoG * out.lever_in_imu;
  out.center = p_IinG + out.lever_in_global;
  out.dcenter_dpose_theta = -R_ItoG * skew(out.lever_in_imu);
  out.dcenter_dextrinsic_theta = R_ItoG * R_CtoI * skew(p_IinC);
  out.dcenter_dextrinsic_position = -R_ItoG * R_CtoI;
  return out;
}

CameraTerms camera_terms(const std::shared_ptr<PoseJPL> &pose,
                         const std::shared_ptr<PoseJPL> &calibration,
                         bool use_fej) {
  return camera_terms(use_fej ? pose->Rot_fej() : pose->Rot(),
                      use_fej ? pose->pos_fej() : pose->pos(),
                      use_fej ? calibration->Rot_fej() : calibration->Rot(),
                      use_fej ? calibration->pos_fej() : calibration->pos());
}

CameraTerms camera_terms(const std::shared_ptr<IMU> &imu,
                         const std::shared_ptr<PoseJPL> &calibration,
                         bool use_fej) {
  return camera_terms(use_fej ? imu->Rot_fej() : imu->Rot(),
                      use_fej ? imu->pos_fej() : imu->pos(),
                      use_fej ? calibration->Rot_fej() : calibration->Rot(),
                      use_fej ? calibration->pos_fej() : calibration->pos());
}

Eigen::Vector3d full_inverse_xyz(const Eigen::Vector3d &inverse) {
  const double radius = 1.0 / inverse(2);
  return radius * Eigen::Vector3d(std::cos(inverse(0)) * std::sin(inverse(1)),
                                  std::sin(inverse(0)) * std::sin(inverse(1)),
                                  std::cos(inverse(1)));
}

Eigen::Vector3d landmark_anchor_xyz(const std::shared_ptr<Landmark> &landmark,
                                    bool use_fej) {
  const Eigen::VectorXd parameters =
      use_fej ? landmark->fej() : landmark->value();
  switch (landmark->_feat_representation) {
  case LandmarkRepresentation::ANCHORED_3D:
    return parameters;
  case LandmarkRepresentation::ANCHORED_FULL_INVERSE_DEPTH:
    return full_inverse_xyz(parameters);
  case LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH:
    return Eigen::Vector3d(parameters(0) / parameters(2),
                           parameters(1) / parameters(2),
                           1.0 / parameters(2));
  case LandmarkRepresentation::ANCHORED_INVERSE_DEPTH_SINGLE:
    return (use_fej ? landmark->uv_norm_zero_fej
                    : landmark->uv_norm_zero) /
           parameters(0);
  default:
    return Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
  }
}

Eigen::Vector3d landmark_global_xyz(const std::shared_ptr<State> &state,
                                    const std::shared_ptr<Landmark> &landmark,
                                    bool use_fej) {
  if (landmark->_feat_representation == LandmarkRepresentation::GLOBAL_3D)
    return use_fej ? landmark->fej() : landmark->value();
  if (landmark->_feat_representation ==
      LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH)
    return full_inverse_xyz(use_fej ? landmark->fej() : landmark->value());

  const auto anchor =
      state->_clones_IMU.at(landmark->_anchor_clone_timestamp);
  const auto calibration = state->_calib_IMUtoCAM.at(
      static_cast<std::size_t>(landmark->_anchor_cam_id));
  const Eigen::Matrix3d R_GtoI =
      use_fej ? anchor->Rot_fej() : anchor->Rot();
  const Eigen::Vector3d p_IinG =
      use_fej ? anchor->pos_fej() : anchor->pos();
  const Eigen::Matrix3d R_ItoC =
      use_fej ? calibration->Rot_fej() : calibration->Rot();
  const Eigen::Vector3d p_IinC =
      use_fej ? calibration->pos_fej() : calibration->pos();
  return R_GtoI.transpose() * R_ItoC.transpose() *
             (landmark_anchor_xyz(landmark, use_fej) - p_IinC) +
         p_IinG;
}

Eigen::Vector2d normalized_projection(
    const std::shared_ptr<PoseJPL> &clone,
    const std::shared_ptr<PoseJPL> &calibration,
    const Eigen::Vector3d &p_FinG, bool use_fej) {
  const Eigen::Matrix3d R_GtoI =
      use_fej ? clone->Rot_fej() : clone->Rot();
  const Eigen::Vector3d p_IinG =
      use_fej ? clone->pos_fej() : clone->pos();
  const Eigen::Matrix3d R_ItoC =
      use_fej ? calibration->Rot_fej() : calibration->Rot();
  const Eigen::Vector3d p_IinC =
      use_fej ? calibration->pos_fej() : calibration->pos();
  const Eigen::Vector3d p_FinC =
      R_ItoC * R_GtoI * (p_FinG - p_IinG) + p_IinC;
  return p_FinC.head<2>() / p_FinC.z();
}

class Fixture {
public:
  explicit Fixture(bool include_all_representations, int num_cameras = 1) {
    StateOptions options;
    options.num_cameras = num_cameras;
    options.do_calib_camera_pose = true;
    options.max_clone_size = 4;
    options.max_slam_features = 8;
    state = std::make_shared<State>(options);
    propagator =
        std::make_unique<Propagator>(NoiseManager(), 9.81);

    calibration = state->_calib_IMUtoCAM.at(0);
    calibration->set_value(pose_value(
        rotation(0.04, -0.08, 0.13), Eigen::Vector3d(0.18, -0.07, 0.11)));
    calibration->set_fej(pose_value(
        rotation(0.035, -0.075, 0.12),
        Eigen::Vector3d(0.175, -0.065, 0.105)));

    state->_imu->set_value(imu_value(
        rotation(0.08, -0.11, 0.22), Eigen::Vector3d(1.2, -2.0, 3.1),
        Eigen::Vector3d(4.0, -1.5, 0.7),
        Eigen::Vector3d(0.011, -0.012, 0.013),
        Eigen::Vector3d(0.021, -0.022, 0.023)));
    state->_imu->set_fej(imu_value(
        rotation(0.075, -0.10, 0.205),
        Eigen::Vector3d(1.1, -1.9, 3.0),
        Eigen::Vector3d(3.8, -1.4, 0.65),
        Eigen::Vector3d(0.010, -0.011, 0.012),
        Eigen::Vector3d(0.020, -0.021, 0.022)));

    clone0 = add_clone(
        1.0, pose_value(rotation(0.03, -0.07, 0.05),
                        Eigen::Vector3d(0.2, -1.4, 2.5)),
        pose_value(rotation(0.025, -0.065, 0.04),
                   Eigen::Vector3d(0.15, -1.35, 2.45)));
    clone1 = add_clone(
        2.0, pose_value(rotation(-0.05, 0.09, 0.31),
                        Eigen::Vector3d(0.7, -1.7, 2.8)),
        pose_value(rotation(-0.045, 0.08, 0.29),
                   Eigen::Vector3d(0.65, -1.65, 2.72)));
    state->_timestamp = 3.0;

    global_3d = add_landmark(
        100, LandmarkRepresentation::GLOBAL_3D,
        Eigen::Vector3d(6.2, 1.0, 8.4),
        Eigen::Vector3d(6.0, 0.9, 8.1));
    anchored_3d = add_landmark(
        101, LandmarkRepresentation::ANCHORED_3D,
        Eigen::Vector3d(0.9, 0.25, 7.2),
        Eigen::Vector3d(0.85, 0.2, 7.0));

    if (include_all_representations) {
      global_full_inverse = add_landmark(
          102, LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH,
          Eigen::Vector3d(0.25, 0.75, 0.12),
          Eigen::Vector3d(0.23, 0.78, 0.125));
      anchored_full_inverse = add_landmark(
          103, LandmarkRepresentation::ANCHORED_FULL_INVERSE_DEPTH,
          Eigen::Vector3d(0.15, 0.55, 0.14),
          Eigen::Vector3d(0.14, 0.58, 0.145));
      anchored_msckf_inverse = add_landmark(
          104, LandmarkRepresentation::ANCHORED_MSCKF_INVERSE_DEPTH,
          Eigen::Vector3d(0.10, 0.05, 0.15),
          Eigen::Vector3d(0.09, 0.045, 0.155));
      anchored_single_inverse = add_landmark(
          105, LandmarkRepresentation::ANCHORED_INVERSE_DEPTH_SINGLE,
          Eigen::VectorXd::Constant(1, 0.14),
          Eigen::VectorXd::Constant(1, 0.145));
      anchored_single_inverse->uv_norm_zero =
          Eigen::Vector3d(0.10, -0.05, 1.0);
      anchored_single_inverse->uv_norm_zero_fej =
          Eigen::Vector3d(0.095, -0.045, 1.0);
    }

    set_covariance(dense_spd_covariance(covariance_size()));
  }

  int covariance_size() const {
    return static_cast<int>(StateHelper::get_full_covariance(state).rows());
  }

  std::vector<std::shared_ptr<Type>> ordered_variables() const {
    std::vector<std::shared_ptr<Type>> variables;
    variables.push_back(state->_imu);
    for (const auto &calibration_pair : state->_calib_IMUtoCAM)
      variables.push_back(calibration_pair.second);
    for (const auto &clone : state->_clones_IMU)
      variables.push_back(clone.second);
    for (const auto &landmark : landmarks)
      variables.push_back(landmark);
    std::sort(variables.begin(), variables.end(),
              [](const std::shared_ptr<Type> &a,
                 const std::shared_ptr<Type> &b) { return a->id() < b->id(); });
    return variables;
  }

  void set_covariance(const Eigen::MatrixXd &covariance) {
    StateHelper::set_initial_covariance(state, covariance,
                                        ordered_variables());
  }

  static Eigen::MatrixXd dense_spd_covariance(int size) {
    Eigen::MatrixXd lower = Eigen::MatrixXd::Zero(size, size);
    for (int row = 0; row < size; ++row) {
      lower(row, row) = 0.8 + 0.01 * row;
      for (int col = 0; col < row; ++col)
        lower(row, col) =
            0.015 * std::sin(0.37 * (row + 1) * (col + 2));
    }
    return lower * lower.transpose() +
           0.2 * Eigen::MatrixXd::Identity(size, size);
  }

  std::shared_ptr<State> state;
  std::unique_ptr<Propagator> propagator;
  std::shared_ptr<PoseJPL> calibration;
  std::shared_ptr<PoseJPL> clone0;
  std::shared_ptr<PoseJPL> clone1;
  std::shared_ptr<Landmark> global_3d;
  std::shared_ptr<Landmark> anchored_3d;
  std::shared_ptr<Landmark> global_full_inverse;
  std::shared_ptr<Landmark> anchored_full_inverse;
  std::shared_ptr<Landmark> anchored_msckf_inverse;
  std::shared_ptr<Landmark> anchored_single_inverse;
  std::vector<std::shared_ptr<Landmark>> landmarks;

private:
  std::shared_ptr<PoseJPL> add_clone(
      double timestamp, const Eigen::Matrix<double, 7, 1> &value,
      const Eigen::Matrix<double, 7, 1> &fej) {
    state->_timestamp = timestamp;
    StateHelper::augment_clone(state, Eigen::Vector3d::Zero());
    auto clone = state->_clones_IMU.at(timestamp);
    clone->set_value(value);
    clone->set_fej(fej);
    return clone;
  }

  std::shared_ptr<Landmark> add_landmark(
      std::size_t feature_id,
      LandmarkRepresentation::Representation representation,
      const Eigen::VectorXd &value, const Eigen::VectorXd &fej) {
    const int dimension =
        representation == LandmarkRepresentation::ANCHORED_INVERSE_DEPTH_SINGLE
            ? 1
            : 3;
    auto landmark = std::make_shared<Landmark>(dimension);
    landmark->_featid = feature_id;
    landmark->_feat_representation = representation;
    landmark->_unique_camera_id = 0;
    landmark->uv_norm_zero = Eigen::Vector3d::UnitZ();
    landmark->uv_norm_zero_fej = Eigen::Vector3d::UnitZ();
    if (LandmarkRepresentation::is_relative_representation(representation)) {
      landmark->_anchor_cam_id = 0;
      landmark->_anchor_clone_timestamp = 1.0;
    }
    landmark->set_value(value);
    landmark->set_fej(fej);

    const std::vector<std::shared_ptr<Type>> empty_order;
    const Eigen::MatrixXd H_R = Eigen::MatrixXd::Zero(dimension, 0);
    const Eigen::MatrixXd H_L = Eigen::MatrixXd::Identity(dimension, dimension);
    const Eigen::MatrixXd R =
        0.25 * Eigen::MatrixXd::Identity(dimension, dimension);
    const Eigen::VectorXd residual = Eigen::VectorXd::Zero(dimension);
    StateHelper::initialize_invertible(state, landmark, empty_order, H_R, H_L,
                                       R, residual);
    state->_features_SLAM.emplace(feature_id, landmark);
    landmarks.push_back(landmark);
    return landmark;
  }
};

struct Snapshot {
  Eigen::MatrixXd covariance;
  std::vector<std::shared_ptr<Type>> variables;
  std::vector<Eigen::MatrixXd> values;
  std::vector<Eigen::MatrixXd> fejs;
  std::vector<Eigen::Vector3d> bearings;
  std::vector<Eigen::Vector3d> bearings_fej;
};

Snapshot snapshot(const Fixture &fixture) {
  Snapshot out;
  out.covariance = StateHelper::get_full_covariance(fixture.state);
  out.variables = fixture.ordered_variables();
  for (const auto &variable : out.variables) {
    out.values.push_back(variable->value());
    out.fejs.push_back(variable->fej());
  }
  for (const auto &landmark : fixture.landmarks) {
    out.bearings.push_back(landmark->uv_norm_zero);
    out.bearings_fej.push_back(landmark->uv_norm_zero_fej);
  }
  return out;
}

bool snapshot_matches(const Fixture &fixture, const Snapshot &before,
                      double tolerance) {
  if (!near_matrix(StateHelper::get_full_covariance(fixture.state),
                   before.covariance, tolerance))
    return false;
  const auto current_variables = fixture.ordered_variables();
  if (current_variables.size() != before.variables.size())
    return false;
  for (std::size_t i = 0; i < current_variables.size(); ++i) {
    if (current_variables[i] != before.variables[i] ||
        !near_matrix(current_variables[i]->value(), before.values[i],
                     tolerance) ||
        !near_matrix(current_variables[i]->fej(), before.fejs[i], tolerance))
      return false;
  }
  for (std::size_t i = 0; i < fixture.landmarks.size(); ++i) {
    if (!near_matrix(fixture.landmarks[i]->uv_norm_zero, before.bearings[i],
                     tolerance) ||
        !near_matrix(fixture.landmarks[i]->uv_norm_zero_fej,
                     before.bearings_fej[i], tolerance))
      return false;
  }
  return true;
}

bool cache_status_equal(const Propagator::FastStateCacheStatus &a,
                        const Propagator::FastStateCacheStatus &b) {
  return a.epoch == b.epoch && a.published_epoch == b.published_epoch &&
         a.valid == b.valid;
}

void feed_fast_cache_imu(Propagator &propagator) {
  const std::array<double, 5> timestamps = {2.99, 3.0, 3.01, 3.02, 3.03};
  for (double timestamp : timestamps) {
    ImuData data;
    data.timestamp = timestamp;
    data.wm = Eigen::Vector3d(0.02, -0.01, 0.03);
    data.am = Eigen::Vector3d(0.1, -0.2, 9.7);
    propagator.feed_imu(data);
  }
}

Eigen::Vector3d local_orientation_error(
    const Eigen::Matrix3d &current_rotation,
    const Eigen::MatrixXd &reference_value) {
  const Eigen::Vector4d reference_quaternion =
      reference_value.block<4, 1>(0, 0);
  const Eigen::Matrix3d reference_rotation =
      quat_2_Rot(reference_quaternion);
  // JPLQuat uses R_plus = Exp(-dtheta) R_reference.
  return -log_so3(current_rotation * reference_rotation.transpose());
}

Eigen::VectorXd local_state_difference(const Fixture &fixture,
                                       const Snapshot &reference) {
  const auto variables = fixture.ordered_variables();
  Eigen::VectorXd difference =
      Eigen::VectorXd::Constant(fixture.covariance_size(),
                                std::numeric_limits<double>::quiet_NaN());
  if (variables.size() != reference.values.size())
    return difference;
  difference.setZero();

  for (std::size_t index = 0; index < variables.size(); ++index) {
    const auto &variable = variables[index];
    const auto &reference_value = reference.values[index];
    if (variable->id() < 0 ||
        variable->id() + variable->size() > difference.rows())
      return Eigen::VectorXd::Constant(
          difference.rows(), std::numeric_limits<double>::quiet_NaN());

    if (const auto imu = std::dynamic_pointer_cast<IMU>(variable)) {
      difference.segment<3>(imu->id()) =
          local_orientation_error(imu->Rot(), reference_value);
      difference.segment<3>(imu->id() + 3) =
          imu->pos() - reference_value.block<3, 1>(4, 0);
      difference.segment<3>(imu->id() + 6) =
          imu->vel() - reference_value.block<3, 1>(7, 0);
      difference.segment<3>(imu->id() + 9) =
          imu->bias_g() - reference_value.block<3, 1>(10, 0);
      difference.segment<3>(imu->id() + 12) =
          imu->bias_a() - reference_value.block<3, 1>(13, 0);
      continue;
    }
    if (const auto pose = std::dynamic_pointer_cast<PoseJPL>(variable)) {
      difference.segment<3>(pose->id()) =
          local_orientation_error(pose->Rot(), reference_value);
      difference.segment<3>(pose->id() + 3) =
          pose->pos() - reference_value.block<3, 1>(4, 0);
      continue;
    }
    if (variable->value().rows() != variable->size() ||
        reference_value.rows() != variable->size())
      return Eigen::VectorXd::Constant(
          difference.rows(), std::numeric_limits<double>::quiet_NaN());
    difference.segment(variable->id(), variable->size()) =
        variable->value() - reference_value;
  }
  return difference;
}

bool perturb_error_column(Fixture &fixture, int column, double amount) {
  for (const auto &variable : fixture.ordered_variables()) {
    if (column < variable->id() ||
        column >= variable->id() + variable->size())
      continue;
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(variable->size());
    delta(column - variable->id()) = amount;
    variable->update(delta);
    return true;
  }
  return false;
}

void append_columns(std::vector<int> &columns, int first, int count) {
  for (int offset = 0; offset < count; ++offset)
    columns.push_back(first + offset);
}

void test_identity(Checks &checks) {
  Fixture fixture(true);
  const Snapshot before = snapshot(fixture);
  const auto result = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.0, fixture.propagator.get(), 0);
  checks.expect(result.success, "scale=1 must succeed");
  checks.expect(!result.state_changed,
                "scale=1 must report an identity transaction");
  checks.expect(!result.propagator_cache_invalidated,
                "scale=1 must not invalidate an unchanged cache");
  checks.expect(snapshot_matches(fixture, before, 0.0),
                "scale=1 must be bitwise state/covariance identity");
}

Eigen::Vector3d full_inverse_parameters(const Eigen::Vector3d &xyz,
                                        double reference_theta) {
  const double radius = xyz.norm();
  double theta = std::atan2(xyz.y(), xyz.x());
  constexpr double pi = 3.14159265358979323846;
  theta += 2.0 * pi *
           std::round((reference_theta - theta) / (2.0 * pi));
  return Eigen::Vector3d(
      theta,
      std::acos(std::max(-1.0, std::min(1.0, xyz.z() / radius))),
      1.0 / radius);
}

void test_global_gauge_identity(Checks &checks) {
  Fixture fixture(true);
  const Snapshot before = snapshot(fixture);
  const auto result = StateHelper::apply_global_yaw_translation_reset(
      fixture.state, 0.0, fixture.state->_imu->pos(),
      fixture.propagator.get());
  checks.expect(result.success && !result.state_changed &&
                    !result.propagator_cache_invalidated,
                "identity global gauge reset must be a no-op");
  checks.expect(snapshot_matches(fixture, before, 0.0),
                "identity global gauge reset must be bitwise identical");
}

void test_global_gauge_geometry_covariance_and_bias(Checks &checks) {
  Fixture fixture(true);
  const Snapshot before = snapshot(fixture);
  const Eigen::MatrixXd covariance_before = before.covariance;
  const Eigen::Vector3d bg_before = fixture.state->_imu->bias_g();
  const Eigen::Vector3d ba_before = fixture.state->_imu->bias_a();
  const Eigen::Vector3d bg_fej_before = fixture.state->_imu->bias_g_fej();
  const Eigen::Vector3d ba_fej_before = fixture.state->_imu->bias_a_fej();
  const Eigen::Vector3d velocity_before = fixture.state->_imu->vel();
  const Eigen::Vector3d velocity_fej_before = fixture.state->_imu->vel_fej();
  const Eigen::MatrixXd anchored_value_before = fixture.anchored_3d->value();
  const Eigen::MatrixXd anchored_fej_before = fixture.anchored_3d->fej();
  const Eigen::MatrixXd calibration_before = fixture.calibration->value();
  const Eigen::MatrixXd calibration_fej_before = fixture.calibration->fej();

  std::vector<Eigen::Vector2d> projections_before;
  std::vector<Eigen::Vector2d> projections_fej_before;
  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    for (const auto &landmark : fixture.landmarks) {
      projections_before.push_back(normalized_projection(
          clone_pair.second, fixture.calibration,
          landmark_global_xyz(fixture.state, landmark, false), false));
      projections_fej_before.push_back(normalized_projection(
          clone_pair.second, fixture.calibration,
          landmark_global_xyz(fixture.state, landmark, true), true));
    }
  }

  constexpr double yaw_delta = 0.37;
  const Eigen::Matrix3d R_delta =
      Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d p_before = fixture.state->_imu->pos();
  const Eigen::Vector3d target(18.0, -7.5, 4.2);
  const Eigen::Vector3d translation = target - R_delta * p_before;

  Eigen::MatrixXd expected_jacobian = Eigen::MatrixXd::Identity(
      fixture.covariance_size(), fixture.covariance_size());
  expected_jacobian.block<3, 3>(fixture.state->_imu->p()->id(),
                                fixture.state->_imu->p()->id()) = R_delta;
  expected_jacobian.block<3, 3>(fixture.state->_imu->v()->id(),
                                fixture.state->_imu->v()->id()) = R_delta;
  for (const auto &clone_pair : fixture.state->_clones_IMU)
    expected_jacobian.block<3, 3>(clone_pair.second->p()->id(),
                                  clone_pair.second->p()->id()) = R_delta;
  expected_jacobian.block<3, 3>(fixture.global_3d->id(),
                                fixture.global_3d->id()) = R_delta;

  const Eigen::Vector3d inverse = fixture.global_full_inverse->value();
  Eigen::Matrix3d inverse_jacobian;
  constexpr double epsilon = 1.0e-7;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = inverse;
    Eigen::Vector3d minus = inverse;
    plus(axis) += epsilon;
    minus(axis) -= epsilon;
    const Eigen::Vector3d plus_out = full_inverse_parameters(
        R_delta * full_inverse_xyz(plus) + translation, inverse(0));
    const Eigen::Vector3d minus_out = full_inverse_parameters(
        R_delta * full_inverse_xyz(minus) + translation, inverse(0));
    inverse_jacobian.col(axis) = (plus_out - minus_out) / (2.0 * epsilon);
  }
  expected_jacobian.block<3, 3>(fixture.global_full_inverse->id(),
                                fixture.global_full_inverse->id()) =
      inverse_jacobian;
  const Eigen::MatrixXd covariance_expected =
      expected_jacobian * covariance_before * expected_jacobian.transpose();

  const auto result = StateHelper::apply_global_yaw_translation_reset(
      fixture.state, yaw_delta, target, fixture.propagator.get());
  checks.expect(result.success && result.state_changed &&
                    result.propagator_cache_invalidated,
                "non-identity global gauge reset must commit atomically");
  checks.expect(near_matrix(fixture.state->_imu->pos(), target, 1e-11) &&
                    near_matrix(fixture.state->_imu->vel(),
                                R_delta * velocity_before, 1e-11) &&
                    near_matrix(fixture.state->_imu->vel_fej(),
                                R_delta * velocity_fej_before, 1e-11),
                "global gauge reset must anchor position and rotate velocity");
  checks.expect(near_matrix(fixture.state->_imu->bias_g(), bg_before, 0.0) &&
                    near_matrix(fixture.state->_imu->bias_a(), ba_before, 0.0) &&
                    near_matrix(fixture.state->_imu->bias_g_fej(),
                                bg_fej_before, 0.0) &&
                    near_matrix(fixture.state->_imu->bias_a_fej(),
                                ba_fej_before, 0.0),
                "global gauge reset must preserve nominal and FEJ biases");
  checks.expect(near_matrix(fixture.anchored_3d->value(),
                            anchored_value_before, 0.0) &&
                    near_matrix(fixture.anchored_3d->fej(),
                                anchored_fej_before, 0.0) &&
                    near_matrix(fixture.calibration->value(),
                                calibration_before, 0.0) &&
                    near_matrix(fixture.calibration->fej(),
                                calibration_fej_before, 0.0),
                "anchored parameters and calibration must remain unchanged");

  std::size_t projection_index = 0;
  bool projections_unchanged = true;
  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    for (const auto &landmark : fixture.landmarks) {
      projections_unchanged =
          projections_unchanged &&
          near_matrix(normalized_projection(
                          clone_pair.second, fixture.calibration,
                          landmark_global_xyz(fixture.state, landmark, false),
                          false),
                      projections_before[projection_index], 2e-9) &&
          near_matrix(normalized_projection(
                          clone_pair.second, fixture.calibration,
                          landmark_global_xyz(fixture.state, landmark, true),
                          true),
                      projections_fej_before[projection_index], 2e-9);
      ++projection_index;
    }
  }
  checks.expect(projections_unchanged,
                "global gauge reset must preserve nominal and FEJ reprojection geometry");
  checks.expect(near_matrix(StateHelper::get_full_covariance(fixture.state),
                            covariance_expected, 2e-7),
                "global gauge reset covariance must equal J P J^T");
}

void test_global_gauge_transactional_rejection(Checks &checks) {
  Fixture fixture(false);
  const Snapshot before = snapshot(fixture);
  const auto result = StateHelper::apply_global_yaw_translation_reset(
      fixture.state, std::numeric_limits<double>::quiet_NaN(),
      Eigen::Vector3d::Zero(), fixture.propagator.get());
  checks.expect(!result.success && !result.state_changed &&
                    snapshot_matches(fixture, before, 0.0),
                "invalid global gauge reset must leave the complete state untouched");
}

void test_global_similarity_geometry_velocity_and_bias(Checks &checks) {
  Fixture fixture(true);
  const Eigen::Vector3d velocity_before = fixture.state->_imu->vel();
  const Eigen::Vector3d bg_before = fixture.state->_imu->bias_g();
  const Eigen::Vector3d ba_before = fixture.state->_imu->bias_a();
  std::vector<Eigen::Vector2d> projections_before;
  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    for (const auto &landmark : fixture.landmarks) {
      projections_before.push_back(normalized_projection(
          clone_pair.second, fixture.calibration,
          landmark_global_xyz(fixture.state, landmark, false), false));
    }
  }

  constexpr double scale = 1.18;
  constexpr double yaw_delta = -0.21;
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3d target(120.0, -45.0, 18.0);
  const auto result =
      StateHelper::apply_global_yaw_scale_translation_reset(
          fixture.state, scale, yaw_delta, target,
          fixture.propagator.get(), 0);
  checks.expect(result.success && result.state_changed &&
                    result.propagator_cache_invalidated &&
                    std::fabs(result.scale - scale) < 1.0e-15,
                "global similarity reset must commit both stages");
  checks.expect(near_matrix(fixture.state->_imu->pos(), target, 1.0e-10) &&
                    near_matrix(fixture.state->_imu->vel(),
                                scale * rotation * velocity_before, 1.0e-10),
                "global similarity reset must anchor position and scale/rotate velocity");
  checks.expect(near_matrix(fixture.state->_imu->bias_g(), bg_before, 0.0) &&
                    near_matrix(fixture.state->_imu->bias_a(), ba_before, 0.0),
                "global similarity reset must preserve both biases");

  std::size_t projection_index = 0;
  bool projections_unchanged = true;
  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    for (const auto &landmark : fixture.landmarks) {
      projections_unchanged =
          projections_unchanged &&
          near_matrix(normalized_projection(
                          clone_pair.second, fixture.calibration,
                          landmark_global_xyz(fixture.state, landmark, false),
                          false),
                      projections_before[projection_index], 3.0e-9);
      ++projection_index;
    }
  }
  checks.expect(projections_unchanged,
                "global similarity reset must preserve reprojection geometry");
  const Eigen::MatrixXd covariance =
      StateHelper::get_full_covariance(fixture.state);
  checks.expect(covariance.allFinite() &&
                    covariance.isApprox(covariance.transpose(), 1.0e-12),
                "global similarity reset covariance must remain finite and symmetric");
}

void test_landmark_get_xyz_uses_fej(Checks &checks) {
  Fixture fixture(true);
  const Eigen::Vector3d msckf_nominal(
      fixture.anchored_msckf_inverse->value()(0) /
          fixture.anchored_msckf_inverse->value()(2),
      fixture.anchored_msckf_inverse->value()(1) /
          fixture.anchored_msckf_inverse->value()(2),
      1.0 / fixture.anchored_msckf_inverse->value()(2));
  const Eigen::Vector3d msckf_fej(
      fixture.anchored_msckf_inverse->fej()(0) /
          fixture.anchored_msckf_inverse->fej()(2),
      fixture.anchored_msckf_inverse->fej()(1) /
          fixture.anchored_msckf_inverse->fej()(2),
      1.0 / fixture.anchored_msckf_inverse->fej()(2));
  checks.expect(
      near_matrix(fixture.anchored_msckf_inverse->get_xyz(false),
                  msckf_nominal) &&
          near_matrix(fixture.anchored_msckf_inverse->get_xyz(true),
                      msckf_fej) &&
          (msckf_nominal - msckf_fej).norm() > 1e-4,
      "anchored MSCKF inverse-depth get_xyz(true) must read FEJ storage");

  const Eigen::Vector3d single_nominal =
      fixture.anchored_single_inverse->uv_norm_zero /
      fixture.anchored_single_inverse->value()(0);
  const Eigen::Vector3d single_fej =
      fixture.anchored_single_inverse->uv_norm_zero_fej /
      fixture.anchored_single_inverse->fej()(0);
  checks.expect(
      near_matrix(fixture.anchored_single_inverse->get_xyz(false),
                  single_nominal) &&
          near_matrix(fixture.anchored_single_inverse->get_xyz(true),
                      single_fej) &&
          (single_nominal - single_fej).norm() > 1e-4,
      "single inverse-depth get_xyz(true) must read FEJ rho and bearing");
}

void test_multicamera_rejection_is_zero_modification(Checks &checks) {
  Fixture fixture(false, 2);
  feed_fast_cache_imu(*fixture.propagator);
  constexpr double target_time = 3.025;
  Eigen::Matrix<double, 13, 1> warmed_state;
  Eigen::Matrix<double, 12, 12> warmed_covariance;
  checks.expect(fixture.propagator->fast_state_propagate(
                    fixture.state, target_time, warmed_state,
                    warmed_covariance),
                "multi-camera cache prewarm must succeed");
  const auto cache_before = fixture.propagator->fast_state_cache_status();
  const Snapshot state_before = snapshot(fixture);

  const auto nonidentity = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.25, fixture.propagator.get(), 0);
  checks.expect(!nonidentity.success && !nonidentity.state_changed &&
                    !nonidentity.propagator_cache_invalidated &&
                    nonidentity.failure_reason.find("exactly one camera") !=
                        std::string::npos,
                "non-identity multi-camera reset must fail closed");
  checks.expect(snapshot_matches(fixture, state_before, 0.0) &&
                    cache_status_equal(
                        fixture.propagator->fast_state_cache_status(),
                        cache_before),
                "multi-camera rejection must preserve state and warm cache");

  const auto identity = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.0, fixture.propagator.get(), 0);
  checks.expect(!identity.success &&
                    identity.failure_reason.find("exactly one camera") !=
                        std::string::npos &&
                    snapshot_matches(fixture, state_before, 0.0) &&
                    cache_status_equal(
                        fixture.propagator->fast_state_cache_status(),
                        cache_before),
                "identity multi-camera reset must also reject with zero modification");

  Eigen::Matrix<double, 13, 1> cached_state;
  Eigen::Matrix<double, 12, 12> cached_covariance;
  checks.expect(fixture.propagator->fast_state_propagate(
                    fixture.state, target_time, cached_state,
                    cached_covariance) &&
                    near_matrix(cached_state, warmed_state, 0.0) &&
                    near_matrix(cached_covariance, warmed_covariance, 0.0),
                "rejected multi-camera reset must retain exact cached output");

  Fixture single_camera(false);
  const Snapshot single_before = snapshot(single_camera);
  const auto wrong_camera = StateHelper::apply_sim3_scale_reset(
      single_camera.state, 1.1, single_camera.propagator.get(), 1);
  checks.expect(!wrong_camera.success &&
                    wrong_camera.failure_reason.find("camera_id 0") !=
                        std::string::npos &&
                    snapshot_matches(single_camera, single_before, 0.0),
                "single-camera reset must reject camera_id != 0 without changes");
}

void test_warm_cache_epoch_reset(Checks &checks) {
  Fixture fixture(false);
  feed_fast_cache_imu(*fixture.propagator);
  constexpr double target_time = 3.025;
  Eigen::Matrix<double, 13, 1> before_reset_state;
  Eigen::Matrix<double, 12, 12> before_reset_covariance;
  checks.expect(fixture.propagator->fast_state_propagate(
                    fixture.state, target_time, before_reset_state,
                    before_reset_covariance),
                "single-camera fast cache prewarm must succeed");
  const auto warmed = fixture.propagator->fast_state_cache_status();
  checks.expect(warmed.valid && warmed.published_epoch == warmed.epoch,
                "prewarmed cache must be valid in its current epoch");

  const auto identity = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.0, fixture.propagator.get(), 0);
  checks.expect(identity.success && !identity.state_changed &&
                    cache_status_equal(
                        fixture.propagator->fast_state_cache_status(), warmed),
                "identity reset must preserve a warm cache and its epoch");

  const auto reset = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.4, fixture.propagator.get(), 0);
  const auto invalidated = fixture.propagator->fast_state_cache_status();
  checks.expect(reset.success && reset.propagator_cache_invalidated &&
                    invalidated.epoch == warmed.epoch + 1 &&
                    invalidated.published_epoch == warmed.published_epoch &&
                    !invalidated.valid,
                "committed reset must advance epoch and make old cache invalid");

  Eigen::Matrix<double, 13, 1> after_reset_state;
  Eigen::Matrix<double, 12, 12> after_reset_covariance;
  checks.expect(fixture.propagator->fast_state_propagate(
                    fixture.state, target_time, after_reset_state,
                    after_reset_covariance),
                "same-timestamp propagation must rebuild after reset epoch");
  const auto rebuilt = fixture.propagator->fast_state_cache_status();
  checks.expect(rebuilt.valid && rebuilt.epoch == invalidated.epoch &&
                    rebuilt.published_epoch == rebuilt.epoch,
                "rebuilt cache must publish only in the new epoch");
  checks.expect((after_reset_state.block<3, 1>(4, 0) -
                 before_reset_state.block<3, 1>(4, 0))
                        .norm() >
                    1e-4,
                "new-epoch same-timestamp result must not replay stale output");
}

void test_geometry_fej_and_reprojection(Checks &checks) {
  Fixture fixture(true);
  constexpr double scale = 1.8;
  const auto imu = fixture.state->_imu;
  const auto calibration = fixture.calibration;
  const CameraTerms pivot_before = camera_terms(imu, calibration, false);
  const CameraTerms pivot_fej_before = camera_terms(imu, calibration, true);
  const Eigen::Vector3d imu_position_before = imu->pos();
  const Eigen::Vector3d imu_position_fej_before = imu->pos_fej();
  const Eigen::Vector3d velocity_before = imu->vel();
  const Eigen::Vector3d velocity_fej_before = imu->vel_fej();
  const Eigen::Vector4d quaternion_before = imu->quat();
  const Eigen::Vector4d quaternion_fej_before = imu->quat_fej();
  const Eigen::Vector3d bg_before = imu->bias_g();
  const Eigen::Vector3d ba_before = imu->bias_a();
  const Eigen::MatrixXd calibration_before = calibration->value();
  const Eigen::MatrixXd calibration_fej_before = calibration->fej();

  std::vector<CameraTerms> clone_before;
  std::vector<CameraTerms> clone_fej_before;
  for (const auto &clone : fixture.state->_clones_IMU) {
    clone_before.push_back(camera_terms(clone.second, calibration, false));
    clone_fej_before.push_back(camera_terms(clone.second, calibration, true));
  }

  std::vector<Eigen::Vector3d> landmark_before;
  std::vector<Eigen::Vector3d> landmark_fej_before;
  std::vector<std::vector<Eigen::Vector2d>> projections_before;
  std::vector<std::vector<Eigen::Vector2d>> projections_fej_before;
  for (const auto &landmark : fixture.landmarks) {
    const Eigen::Vector3d p_FinG =
        landmark_global_xyz(fixture.state, landmark, false);
    const Eigen::Vector3d p_FinG_fej =
        landmark_global_xyz(fixture.state, landmark, true);
    landmark_before.push_back(p_FinG);
    landmark_fej_before.push_back(p_FinG_fej);
    projections_before.emplace_back();
    projections_fej_before.emplace_back();
    for (const auto &clone : fixture.state->_clones_IMU) {
      projections_before.back().push_back(normalized_projection(
          clone.second, calibration, p_FinG, false));
      projections_fej_before.back().push_back(normalized_projection(
          clone.second, calibration, p_FinG_fej, true));
    }
  }
  const Eigen::MatrixXd anchored_3d_before = fixture.anchored_3d->value();
  const Eigen::MatrixXd anchored_full_before =
      fixture.anchored_full_inverse->value();
  const Eigen::MatrixXd anchored_msckf_before =
      fixture.anchored_msckf_inverse->value();
  const Eigen::MatrixXd anchored_single_before =
      fixture.anchored_single_inverse->value();
  const Eigen::Vector3d single_bearing_before =
      fixture.anchored_single_inverse->uv_norm_zero;
  const Eigen::Vector3d single_bearing_fej_before =
      fixture.anchored_single_inverse->uv_norm_zero_fej;

  const auto result = StateHelper::apply_sim3_scale_reset(
      fixture.state, scale, fixture.propagator.get(), 0);
  checks.expect(result.success && result.state_changed,
                "known scale reset must commit");
  checks.expect(result.propagator_cache_invalidated,
                "committed reset must invalidate propagator cache");
  checks.expect(!result.covariance_psd_projected &&
                    result.covariance_psd_projection_magnitude == 0.0 &&
                    std::isfinite(result.covariance_psd_projection_limit) &&
                    result.covariance_psd_projection_limit > 0.0,
                "well-conditioned covariance must need no PSD projection");
  checks.expect(near_matrix(result.pivot_camera_center,
                            pivot_before.center),
                "reported pivot must be the current camera center");

  checks.expect(near_matrix(imu->pos(), imu_position_before),
                "current IMU position must stay fixed at camera pivot");
  checks.expect(near_matrix(imu->pos_fej(), imu_position_fej_before),
                "current IMU FEJ position must stay fixed at FEJ pivot");
  checks.expect(near_matrix(imu->vel(), scale * velocity_before),
                "current velocity must scale");
  checks.expect(near_matrix(imu->vel_fej(), scale * velocity_fej_before),
                "current FEJ velocity must scale");
  checks.expect(near_matrix(imu->quat(), quaternion_before) &&
                    near_matrix(imu->quat_fej(), quaternion_fej_before),
                "current nominal/FEJ orientation must not change");
  checks.expect(near_matrix(imu->bias_g(), bg_before) &&
                    near_matrix(imu->bias_a(), ba_before),
                "IMU biases must not change");
  checks.expect(near_matrix(calibration->value(), calibration_before) &&
                    near_matrix(calibration->fej(), calibration_fej_before),
                "camera extrinsic nominal/FEJ must not change");

  bool lever_arm_changed_naive_solution = false;
  std::size_t clone_index = 0;
  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    const CameraTerms after =
        camera_terms(clone_pair.second, calibration, false);
    const CameraTerms after_fej =
        camera_terms(clone_pair.second, calibration, true);
    checks.expect(
        near_matrix(after.center,
                    pivot_before.center +
                        scale * (clone_before[clone_index].center -
                                 pivot_before.center)),
        "clone camera center must scale around the current camera pivot");
    checks.expect(
        near_matrix(after_fej.center,
                    pivot_fej_before.center +
                        scale * (clone_fej_before[clone_index].center -
                                 pivot_fej_before.center)),
        "clone FEJ camera center must scale around the FEJ pivot");
    const Eigen::Vector3d naive_position =
        imu_position_before +
        scale * (clone_before[clone_index].center -
                 clone_before[clone_index].lever_in_global -
                 imu_position_before);
    lever_arm_changed_naive_solution =
        lever_arm_changed_naive_solution ||
        (clone_pair.second->pos() - naive_position).norm() > 1e-5;
    ++clone_index;
  }
  checks.expect(lever_arm_changed_naive_solution,
                "nonzero rotated extrinsic must distinguish camera-pivot reset "
                "from naive IMU-position scaling");

  for (std::size_t feature = 0; feature < fixture.landmarks.size(); ++feature) {
    const Eigen::Vector3d after = landmark_global_xyz(
        fixture.state, fixture.landmarks[feature], false);
    const Eigen::Vector3d after_fej = landmark_global_xyz(
        fixture.state, fixture.landmarks[feature], true);
    checks.expect(
        near_matrix(after,
                    pivot_before.center +
                        scale * (landmark_before[feature] -
                                 pivot_before.center),
                    2e-10),
        "landmark global geometry must follow the pivoted Sim(3)");
    checks.expect(
        near_matrix(after_fej,
                    pivot_fej_before.center +
                        scale * (landmark_fej_before[feature] -
                                 pivot_fej_before.center),
                    2e-10),
        "landmark FEJ geometry must follow the FEJ Sim(3)");

    std::size_t observation = 0;
    for (const auto &clone : fixture.state->_clones_IMU) {
      checks.expect(
          near_matrix(normalized_projection(clone.second, calibration, after,
                                            false),
                      projections_before[feature][observation], 3e-10),
          "nominal reprojection must be invariant");
      checks.expect(
          near_matrix(normalized_projection(clone.second, calibration,
                                            after_fej, true),
                      projections_fej_before[feature][observation], 3e-10),
          "FEJ reprojection must be invariant");
      ++observation;
    }
  }

  checks.expect(near_matrix(fixture.anchored_3d->value(),
                            scale * anchored_3d_before),
                "anchored 3D coordinates must scale directly");
  checks.expect(
      near_matrix(fixture.anchored_full_inverse->value().block(0, 0, 2, 1),
                  anchored_full_before.block(0, 0, 2, 1)) &&
          std::abs(fixture.anchored_full_inverse->value()(2) -
                   anchored_full_before(2) / scale) < kTight,
      "anchored full inverse-depth bearing/rho contract must hold");
  checks.expect(
      near_matrix(fixture.anchored_msckf_inverse->value().block(0, 0, 2, 1),
                  anchored_msckf_before.block(0, 0, 2, 1)) &&
          std::abs(fixture.anchored_msckf_inverse->value()(2) -
                   anchored_msckf_before(2) / scale) < kTight,
      "anchored MSCKF inverse-depth bearing/rho contract must hold");
  checks.expect(
      std::abs(fixture.anchored_single_inverse->value()(0) -
               anchored_single_before(0) / scale) < kTight &&
          near_matrix(fixture.anchored_single_inverse->uv_norm_zero,
                      single_bearing_before) &&
          near_matrix(fixture.anchored_single_inverse->uv_norm_zero_fej,
                      single_bearing_fej_before),
      "single inverse-depth rho must scale without changing its bearings");

  const Eigen::MatrixXd covariance =
      StateHelper::get_full_covariance(fixture.state);
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covariance);
  checks.expect(covariance.allFinite() &&
                    (covariance - covariance.transpose()).norm() < 1e-10 &&
                    eigen_solver.info() == Eigen::Success &&
                    eigen_solver.eigenvalues().minCoeff() >= -1e-10,
                "reset covariance must be finite, symmetric, and PSD");
}

void test_covariance_jacobian(Checks &checks) {
  Fixture fixture(false);
  constexpr double scale = 1.45;
  const Eigen::MatrixXd covariance_before =
      StateHelper::get_full_covariance(fixture.state);
  const int size = static_cast<int>(covariance_before.rows());
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(size, size);

  const auto imu = fixture.state->_imu;
  const auto calibration = fixture.calibration;
  const CameraTerms pivot = camera_terms(imu, calibration, false);
  jacobian.block(imu->v()->id(), 0, 3, size).setZero();
  jacobian.block<3, 3>(imu->v()->id(), imu->v()->id()) =
      scale * Eigen::Matrix3d::Identity();

  for (const auto &clone_pair : fixture.state->_clones_IMU) {
    const auto clone = clone_pair.second;
    const CameraTerms terms = camera_terms(clone, calibration, false);
    const int row = clone->p()->id();
    jacobian.block(row, 0, 3, size).setZero();
    jacobian.block<3, 3>(row, clone->p()->id()) =
        scale * Eigen::Matrix3d::Identity();
    jacobian.block<3, 3>(row, imu->p()->id()) =
        (1.0 - scale) * Eigen::Matrix3d::Identity();
    jacobian.block<3, 3>(row, clone->q()->id()) =
        (scale - 1.0) * terms.dcenter_dpose_theta;
    jacobian.block<3, 3>(row, imu->q()->id()) =
        (1.0 - scale) * pivot.dcenter_dpose_theta;
    jacobian.block<3, 3>(row, calibration->q()->id()) =
        (scale - 1.0) * (terms.dcenter_dextrinsic_theta -
                         pivot.dcenter_dextrinsic_theta);
    jacobian.block<3, 3>(row, calibration->p()->id()) =
        (scale - 1.0) * (terms.dcenter_dextrinsic_position -
                         pivot.dcenter_dextrinsic_position);
  }

  const int global_row = fixture.global_3d->id();
  jacobian.block(global_row, 0, 3, size).setZero();
  jacobian.block<3, 3>(global_row, global_row) =
      scale * Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(global_row, imu->p()->id()) =
      (1.0 - scale) * Eigen::Matrix3d::Identity();
  jacobian.block<3, 3>(global_row, imu->q()->id()) =
      (1.0 - scale) * pivot.dcenter_dpose_theta;
  jacobian.block<3, 3>(global_row, calibration->q()->id()) =
      (1.0 - scale) * pivot.dcenter_dextrinsic_theta;
  jacobian.block<3, 3>(global_row, calibration->p()->id()) =
      (1.0 - scale) * pivot.dcenter_dextrinsic_position;

  const int anchored_row = fixture.anchored_3d->id();
  jacobian.block(anchored_row, 0, 3, size).setZero();
  jacobian.block<3, 3>(anchored_row, anchored_row) =
      scale * Eigen::Matrix3d::Identity();

  Eigen::MatrixXd expected =
      (jacobian * covariance_before * jacobian.transpose()).eval();
  expected = 0.5 * (expected + expected.transpose());

  const auto result = StateHelper::apply_sim3_scale_reset(
      fixture.state, scale, fixture.propagator.get(), 0);
  const Eigen::MatrixXd covariance_after =
      StateHelper::get_full_covariance(fixture.state);
  checks.expect(result.success, "covariance Jacobian test reset must succeed");
  checks.expect(near_matrix(covariance_after, expected, 2e-10),
                "complete covariance must equal J P J^T");
  checks.expect(
      near_matrix(covariance_after.block(fixture.clone0->p()->id(),
                                         imu->q()->id(), 3, 3),
                  expected.block(fixture.clone0->p()->id(), imu->q()->id(), 3,
                                 3),
                  2e-10),
      "clone/pivot-attitude covariance cross block must follow Jacobian");
  checks.expect(
      (covariance_after.block(fixture.clone0->p()->id(), imu->q()->id(), 3,
                              3) -
       covariance_before.block(fixture.clone0->p()->id(), imu->q()->id(), 3,
                               3))
              .norm() >
          1e-7,
      "scale reset must actually transform covariance cross terms");
}

void test_covariance_jacobian_independent_finite_difference(Checks &checks) {
  constexpr double scale = 1.35;
  constexpr double step = 2e-6;
  Fixture layout(true);
  const int state_size = layout.covariance_size();
  std::vector<int> columns;
  append_columns(columns, layout.state->_imu->q()->id(), 3);
  append_columns(columns, layout.state->_imu->p()->id(), 3);
  append_columns(columns, layout.state->_imu->v()->id(), 3);
  append_columns(columns, layout.calibration->q()->id(), 3);
  append_columns(columns, layout.calibration->p()->id(), 3);
  append_columns(columns, layout.clone0->q()->id(), 3);
  append_columns(columns, layout.clone0->p()->id(), 3);
  append_columns(columns, layout.global_3d->id(), 3);
  append_columns(columns, layout.global_full_inverse->id(), 3);
  append_columns(columns, layout.anchored_3d->id(), 3);
  columns.push_back(layout.anchored_full_inverse->id() + 2);
  columns.push_back(layout.anchored_msckf_inverse->id() + 2);
  columns.push_back(layout.anchored_single_inverse->id());

  Fixture transformed_reference_fixture(true);
  const auto reference_result = StateHelper::apply_sim3_scale_reset(
      transformed_reference_fixture.state, scale,
      transformed_reference_fixture.propagator.get(), 0);
  const Snapshot transformed_reference =
      snapshot(transformed_reference_fixture);

  Eigen::MatrixXd finite_difference =
      Eigen::MatrixXd::Constant(state_size, columns.size(),
                                std::numeric_limits<double>::quiet_NaN());
  bool perturbations_succeeded = reference_result.success;
  for (std::size_t index = 0; index < columns.size(); ++index) {
    Fixture plus(true);
    Fixture minus(true);
    perturbations_succeeded =
        perturb_error_column(plus, columns[index], step) &&
        perturb_error_column(minus, columns[index], -step) &&
        perturbations_succeeded;
    const auto plus_result = StateHelper::apply_sim3_scale_reset(
        plus.state, scale, plus.propagator.get(), 0);
    const auto minus_result = StateHelper::apply_sim3_scale_reset(
        minus.state, scale, minus.propagator.get(), 0);
    perturbations_succeeded =
        plus_result.success && minus_result.success && perturbations_succeeded;
    if (!plus_result.success || !minus_result.success)
      continue;
    const Eigen::VectorXd plus_difference =
        local_state_difference(plus, transformed_reference);
    const Eigen::VectorXd minus_difference =
        local_state_difference(minus, transformed_reference);
    finite_difference.col(index) =
        (plus_difference - minus_difference) / (2.0 * step);
  }
  checks.expect(perturbations_succeeded && finite_difference.allFinite(),
                "independent central-difference reset map must evaluate");

  Eigen::VectorXd variances(columns.size());
  Eigen::MatrixXd selected_covariance =
      Eigen::MatrixXd::Zero(state_size, state_size);
  for (std::size_t index = 0; index < columns.size(); ++index) {
    variances(static_cast<Eigen::Index>(index)) =
        0.3 + 0.01 * static_cast<double>(index);
    selected_covariance(columns[index], columns[index]) =
        variances(static_cast<Eigen::Index>(index));
  }
  const Eigen::MatrixXd expected_covariance =
      finite_difference * variances.asDiagonal() *
      finite_difference.transpose();

  Fixture covariance_fixture(true);
  covariance_fixture.set_covariance(selected_covariance);
  const auto covariance_result = StateHelper::apply_sim3_scale_reset(
      covariance_fixture.state, scale, covariance_fixture.propagator.get(), 0);
  const Eigen::MatrixXd actual_covariance =
      StateHelper::get_full_covariance(covariance_fixture.state);
  const double relative_error =
      (actual_covariance - expected_covariance).norm() /
      std::max(1.0, expected_covariance.norm());
  std::ostringstream message;
  message << "analytic covariance reset must match independent finite "
             "differences (relative error="
          << relative_error << ")";
  checks.expect(covariance_result.success && std::isfinite(relative_error) &&
                    relative_error < 5e-6,
                message.str());
}

void test_psd_projection_is_roundoff_bounded(Checks &checks) {
  {
    Fixture fixture(false);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(
        fixture.covariance_size(), fixture.covariance_size());
    covariance(fixture.state->_imu->v()->id(),
               fixture.state->_imu->v()->id()) = -1e-14;
    fixture.set_covariance(covariance);
    const auto result = StateHelper::apply_sim3_scale_reset(
        fixture.state, 1.2, fixture.propagator.get(), 0);
    const Eigen::MatrixXd repaired =
        StateHelper::get_full_covariance(fixture.state);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(repaired);
    checks.expect(result.success && result.covariance_psd_projected &&
                      result.covariance_psd_projection_magnitude > 0.0 &&
                      result.covariance_psd_projection_magnitude <=
                          result.covariance_psd_projection_limit &&
                      eigen_solver.info() == Eigen::Success &&
                      eigen_solver.eigenvalues().minCoeff() >=
                          -result.covariance_psd_projection_limit,
                  "machine-roundoff negative mode may be projected with diagnostics");
  }

  {
    Fixture fixture(false);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(
        fixture.covariance_size(), fixture.covariance_size());
    covariance(0, 1) = 1.0 + 1e-11;
    covariance(1, 0) = covariance(0, 1);
    fixture.set_covariance(covariance);
    const Snapshot before = snapshot(fixture);
    const auto cache_before = fixture.propagator->fast_state_cache_status();
    const auto result = StateHelper::apply_sim3_scale_reset(
        fixture.state, 1.2, fixture.propagator.get(), 0);
    checks.expect(!result.success && !result.state_changed &&
                      !result.propagator_cache_invalidated &&
                      !result.covariance_psd_projected &&
                      result.covariance_psd_projection_magnitude >
                          result.covariance_psd_projection_limit &&
                      result.failure_reason.find("positive semidefinite") !=
                          std::string::npos,
                  "above-roundoff PSD correction must be rejected diagnostically");
    checks.expect(snapshot_matches(fixture, before, 0.0) &&
                      cache_status_equal(
                          fixture.propagator->fast_state_cache_status(),
                          cache_before),
                  "above-bound PSD rejection must roll back state and cache");
  }
}

void test_invalid_inputs_are_transactional(Checks &checks) {
  Fixture fixture(false);
  const std::array<double, 4> invalid_scales = {
      0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()};
  for (double scale : invalid_scales) {
    const Snapshot before = snapshot(fixture);
    const auto result = StateHelper::apply_sim3_scale_reset(
        fixture.state, scale, fixture.propagator.get(), 0);
    checks.expect(!result.success && !result.state_changed &&
                      !result.propagator_cache_invalidated,
                  "invalid scale must be rejected without commit/cache change");
    checks.expect(snapshot_matches(fixture, before, 0.0),
                  "invalid scale rejection must have no state side effects");
  }

  {
    const Snapshot before = snapshot(fixture);
    const auto result = StateHelper::apply_sim3_scale_reset(
        fixture.state, 1.2, nullptr, 0);
    checks.expect(!result.success,
                  "non-identity reset without cache handle must fail closed");
    checks.expect(snapshot_matches(fixture, before, 0.0),
                  "missing cache handle must not modify state");
  }

  Eigen::MatrixXd indefinite =
      Eigen::MatrixXd::Identity(fixture.covariance_size(),
                                fixture.covariance_size());
  indefinite(0, 1) = 2.0;
  indefinite(1, 0) = 2.0;
  fixture.set_covariance(indefinite);
  const Snapshot before_bad_covariance = snapshot(fixture);
  const auto result = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.3, fixture.propagator.get(), 0);
  checks.expect(!result.success &&
                    result.failure_reason.find("positive semidefinite") !=
                        std::string::npos,
                "indefinite prior covariance must be rejected as non-PSD");
  checks.expect(snapshot_matches(fixture, before_bad_covariance, 0.0),
                "non-PSD rejection must atomically preserve the bad prior and "
                "all nominal/FEJ values");
}

void test_reciprocal_reset(Checks &checks) {
  Fixture fixture(true);
  const Snapshot before = snapshot(fixture);
  constexpr double scale = 1.6;
  const auto forward = StateHelper::apply_sim3_scale_reset(
      fixture.state, scale, fixture.propagator.get(), 0);
  const auto inverse = StateHelper::apply_sim3_scale_reset(
      fixture.state, 1.0 / scale, fixture.propagator.get(), 0);
  checks.expect(forward.success && inverse.success,
                "reciprocal resets must both commit");
  checks.expect(snapshot_matches(fixture, before, 2e-8),
                "successive reciprocal scale resets must recover state, FEJ, "
                "bearings, and covariance");
}

} // namespace

int main() {
  Checks checks;
  test_identity(checks);
  test_global_gauge_identity(checks);
  test_global_gauge_geometry_covariance_and_bias(checks);
  test_global_gauge_transactional_rejection(checks);
  test_global_similarity_geometry_velocity_and_bias(checks);
  test_landmark_get_xyz_uses_fej(checks);
  test_multicamera_rejection_is_zero_modification(checks);
  test_warm_cache_epoch_reset(checks);
  test_geometry_fej_and_reprojection(checks);
  test_covariance_jacobian(checks);
  test_covariance_jacobian_independent_finite_difference(checks);
  test_psd_projection_is_roundoff_bounded(checks);
  test_invalid_inputs_are_transactional(checks);
  test_reciprocal_reset(checks);

  std::cout << "StateHelper Sim(3) scale reset: " << checks.passed
            << " checks passed, " << checks.failed << " failed\n";
  return checks.failed == 0 ? 0 : 1;
}
