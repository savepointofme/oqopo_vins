/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef OV_MSCKF_IMUFILTER_H
#define OV_MSCKF_IMUFILTER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>

#include "thirdparty/px4/Px4NotchFilter.hpp"
#include "utils/sensor_data.h"

namespace ov_msckf {

/// Configuration for the single causal IMU filter chain in VioManager.
struct ImuFilterOptions {
  struct NotchOptions {
    bool enabled = false;
    double frequency_hz = 0.0;
    double bandwidth_hz = 0.0;
  };

  bool enabled = false;
  NotchOptions gyro_notch0;
  NotchOptions gyro_notch1;
  /// Nominal rate. Set to zero to initialize entirely from timestamps.
  double sample_rate_hz = 0.0;
  bool log_raw_and_filtered = false;
  std::string log_path = "imu_filter_raw_filtered.csv";
};

struct ImuFilterStats {
  uint64_t sample_count = 0;
  uint64_t reset_count = 0;
  uint64_t coefficient_update_count = 0;
  uint64_t disable_count = 0;
  uint64_t timestamp_duplicate_count = 0;
  uint64_t timestamp_backward_count = 0;
  uint64_t timestamp_gap_count = 0;
  double measured_sample_rate_hz = 0.0;
  double active_sample_rate_hz = 0.0;
  double total_processing_time_s = 0.0;

  double mean_processing_time_us() const {
    return sample_count == 0 ? 0.0 : 1e6 * total_processing_time_s / static_cast<double>(sample_count);
  }
};

/// Owns the gyro filter chain and processes each raw IMU sample exactly once.
class ImuFilter {
public:
  ImuFilter() = default;
  explicit ImuFilter(const ImuFilterOptions &options) { configure(options); }

  void configure(const ImuFilterOptions &options);
  void set_enabled(bool enabled);
  ov_core::ImuData process(const ov_core::ImuData &raw);

  const ImuFilterOptions &options() const { return options_; }
  const ImuFilterStats &stats() const { return stats_; }

private:
  bool has_active_filters() const;
  void validate_options() const;
  void configure_axes(double sample_rate_hz, const ov_core::ImuData *reset_sample);
  void reset_all(const ov_core::ImuData &sample, ov_core::ImuData *reset_output = nullptr);
  Eigen::Vector3d apply_chain(const Eigen::Vector3d &gyro);
  void update_sample_rate(double instantaneous_rate_hz, const ov_core::ImuData &sample);
  void open_log_if_requested();
  void write_log(const ov_core::ImuData &raw, const ov_core::ImuData &filtered);

  ImuFilterOptions options_;
  std::array<px4::math::NotchFilter<float>, 3> gyro_notch0_;
  std::array<px4::math::NotchFilter<float>, 3> gyro_notch1_;
  ImuFilterStats stats_;
  bool initialized_ = false;
  bool warned_rate_mismatch_ = false;
  bool have_last_raw_timestamp_ = false;
  double last_raw_timestamp_ = 0.0;
  double measured_sample_rate_hz_ = 0.0;
  double active_sample_rate_hz_ = 0.0;
  uint64_t valid_interval_count_ = 0;
  std::ofstream log_stream_;
};

} // namespace ov_msckf

#endif // OV_MSCKF_IMUFILTER_H
