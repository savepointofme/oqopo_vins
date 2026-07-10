/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H
#define OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H

#include <memory>
#include <string>
#include <vector>

#include "state/StateHelper.h"

namespace ov_type {
class Type;
}

namespace ov_msckf {

class State;

class UpdaterGroundPlaneRange {
public:
  enum class HeightMeasurementType {
    GPS_ALTITUDE,
    RANGEFINDER
  };

  struct LastUpdate {
    double t = -1.0;
    double meas = 0.0;
    double pred = 0.0;
    double residual = 0.0;
    double cos_tilt = 0.0;
    double z_ground = 0.0;
    double pz_before = 0.0;
    double pz_after = 0.0;
    double K_pz = 0.0;
    double K_px = 0.0;
    double K_py = 0.0;
    double K_xy_norm = 0.0;
    double pred_dx = 0.0;
    double pred_dy = 0.0;
    double pred_dz = 0.0;
    double pred_dtheta_norm = 0.0;
    double pred_dba_norm = 0.0;
    double pred_dbg_norm = 0.0;
    double S = 0.0;
    std::string decision = "NONE";
  };

  struct Stats {
    size_t n_called = 0;
    size_t n_skipped = 0;
    size_t n_rejected = 0;
    size_t n_accepted = 0;
    double sum_K_pz = 0.0;
    double sum_K_xy_norm = 0.0;
    double sum_dxy_norm = 0.0;
    double sum_dtheta_norm = 0.0;
    double sum_dba_norm = 0.0;
    double sum_dbg_norm = 0.0;
    double sum_abs_res = 0.0;
    double sum_abs_dpz = 0.0;
    double first_t = -1.0;
    double last_t = -1.0;
    double last_summary_t = -1.0;
  };

  UpdaterGroundPlaneRange(double sigma, double min_cos_tilt = 0.3,
                          double chi2_gate = 10000.0,
                          double min_pzz = 0.0,
                          double max_res_gate = 1e9,
                          HeightMeasurementType type = HeightMeasurementType::GPS_ALTITUDE);

  bool try_update(std::shared_ptr<State> state, double t_state,
                  double measurement, double t_meas);

  void set_visual_yaw_update_control(StateHelper::VisualYawUpdateMode mode, double scale, double global_alpha) {
    visual_yaw_update_mode_ = mode;
    visual_yaw_update_scale_ = scale;
    visual_global_yaw_oc_alpha_ = global_alpha;
  }

  const LastUpdate &last_update() const { return last_; }
  const Stats &stats() const { return stats_; }
  double z_ground() const { return z_ground_; }
  bool bootstrapped() const { return bootstrapped_; }
  HeightMeasurementType measurement_type() const { return type_; }

  void reset();
  void print_summary() const;

private:
  double sigma_;
  double min_cos_tilt_;
  double chi2_gate_;
  double min_pzz_;
  double max_res_gate_;
  HeightMeasurementType type_;

  bool bootstrapped_ = false;
  double z_ground_ = 0.0;

  LastUpdate last_;
  Stats stats_;

  StateHelper::VisualYawUpdateMode visual_yaw_update_mode_ = StateHelper::VisualYawUpdateMode::ORIGINAL;
  double visual_yaw_update_scale_ = 1.0;
  double visual_global_yaw_oc_alpha_ = 0.0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_GROUNDPLANE_RANGE_H
