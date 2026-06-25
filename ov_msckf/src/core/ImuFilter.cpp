/*
 * OpenVINS real-time IMU filtering.
 * The byte-identical PX4 algorithms and BSD terms live in thirdparty/px4.
 */

#include "core/ImuFilter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace ov_msckf {

namespace {
constexpr double kRateUpdateRelativeThreshold = 0.05;
constexpr double kRateWarningRelativeThreshold = 0.10;
constexpr double kRateEmaAlpha = 0.02;
constexpr uint64_t kRateWarningMinimumIntervals = 20;

bool valid_positive(double value) {
  return std::isfinite(value) && value > 0.0;
}

} // namespace

bool ImuFilter::has_active_filters() const {
  return options_.enabled &&
         (options_.gyro_notch0.enabled || options_.gyro_notch1.enabled || options_.gyro_lowpass.enabled);
}

void ImuFilter::validate_options() const {
  if (!std::isfinite(options_.sample_rate_hz) || options_.sample_rate_hz < 0.0)
    throw std::invalid_argument("imu_filter.sample_rate_hz must be finite and non-negative");
  if (!options_.enabled)
    return;
  if (options_.gyro_notch0.enabled && (!valid_positive(options_.gyro_notch0.frequency_hz) ||
                                      !valid_positive(options_.gyro_notch0.bandwidth_hz)))
    return;
  if (options_.gyro_notch1.enabled && (!valid_positive(options_.gyro_notch1.frequency_hz) ||
                                      !valid_positive(options_.gyro_notch1.bandwidth_hz)))
    return;
  if (options_.gyro_lowpass.enabled && !valid_positive(options_.gyro_lowpass.cutoff_hz))
    return;
}

void ImuFilter::configure(const ImuFilterOptions &options) {
  if (log_stream_.is_open())
    log_stream_.close();
  options_ = options;
  validate_options();
  stats_ = {};
  initialized_ = false;
  have_last_raw_timestamp_ = false;
  warned_rate_mismatch_ = false;
  measured_sample_rate_hz_ = 0.0;
  active_sample_rate_hz_ = options_.sample_rate_hz;
  valid_interval_count_ = 0;
  if (has_active_filters() && active_sample_rate_hz_ > 0.0)
    configure_axes(active_sample_rate_hz_, nullptr);
  open_log_if_requested();
}

void ImuFilter::set_enabled(bool enabled) {
  if (options_.enabled == enabled)
    return;
  options_.enabled = enabled;
  initialized_ = false;
  have_last_raw_timestamp_ = false;
  measured_sample_rate_hz_ = 0.0;
  valid_interval_count_ = 0;
  warned_rate_mismatch_ = false;
  active_sample_rate_hz_ = options_.sample_rate_hz;
  if (has_active_filters() && active_sample_rate_hz_ > 0.0)
    configure_axes(active_sample_rate_hz_, nullptr);
}

void ImuFilter::configure_axes(double sample_rate_hz, const ov_core::ImuData *reset_sample) {
  if (!std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0)
    throw std::invalid_argument("IMU filter cannot configure with an invalid sample rate");

  auto configure_notch = [&](std::array<px4::math::NotchFilter<float>, 3> &filters,
                             const ImuFilterOptions::NotchOptions &opts) {
    if (!opts.enabled) {
      for (auto &filter : filters)
        filter.disable();
      return;
    }
    for (auto &filter : filters) {
      if (!filter.setParameters(static_cast<float>(sample_rate_hz), static_cast<float>(opts.frequency_hz),
                                static_cast<float>(opts.bandwidth_hz)))
        stats_.disable_count++;
    }
  };

  configure_notch(gyro_notch0_, options_.gyro_notch0);
  configure_notch(gyro_notch1_, options_.gyro_notch1);

  if (options_.gyro_lowpass.enabled) {
    for (auto &filter : gyro_lowpass_)
      filter.set_cutoff_frequency(static_cast<float>(sample_rate_hz), static_cast<float>(options_.gyro_lowpass.cutoff_hz));
  } else {
    for (auto &filter : gyro_lowpass_)
      filter.disable();
  }

  active_sample_rate_hz_ = sample_rate_hz;
  stats_.active_sample_rate_hz = sample_rate_hz;
  stats_.coefficient_update_count++;
  if (reset_sample != nullptr)
    reset_all(*reset_sample);
}

Eigen::Vector3d ImuFilter::apply_chain(const Eigen::Vector3d &gyro) {
  Eigen::Vector3d out = gyro;
  if (options_.gyro_notch0.enabled)
    for (Eigen::Index axis = 0; axis < 3; ++axis)
      out(axis) = static_cast<double>(gyro_notch0_[static_cast<size_t>(axis)].apply(static_cast<float>(out(axis))));
  if (options_.gyro_notch1.enabled)
    for (Eigen::Index axis = 0; axis < 3; ++axis)
      out(axis) = static_cast<double>(gyro_notch1_[static_cast<size_t>(axis)].apply(static_cast<float>(out(axis))));
  if (options_.gyro_lowpass.enabled)
    for (Eigen::Index axis = 0; axis < 3; ++axis)
      out(axis) = static_cast<double>(gyro_lowpass_[static_cast<size_t>(axis)].apply(static_cast<float>(out(axis))));
  return out;
}

void ImuFilter::reset_all(const ov_core::ImuData &sample, ov_core::ImuData *reset_output) {
  Eigen::Vector3d out = sample.wm;
  if (options_.gyro_notch0.enabled) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const Eigen::Index i = static_cast<Eigen::Index>(axis);
      out(i) = std::isfinite(out(i)) ? out(i) : 0.0;
      gyro_notch0_[axis].reset(static_cast<float>(out(i)));
      out(static_cast<Eigen::Index>(axis)) =
          static_cast<double>(gyro_notch0_[axis].apply(static_cast<float>(out(static_cast<Eigen::Index>(axis)))));
    }
  }
  if (options_.gyro_notch1.enabled) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const Eigen::Index i = static_cast<Eigen::Index>(axis);
      out(i) = std::isfinite(out(i)) ? out(i) : 0.0;
      gyro_notch1_[axis].reset(static_cast<float>(out(i)));
      out(static_cast<Eigen::Index>(axis)) =
          static_cast<double>(gyro_notch1_[axis].apply(static_cast<float>(out(static_cast<Eigen::Index>(axis)))));
    }
  }
  if (options_.gyro_lowpass.enabled) {
    for (size_t axis = 0; axis < 3; ++axis) {
      const Eigen::Index i = static_cast<Eigen::Index>(axis);
      out(i) = std::isfinite(out(i)) ? out(i) : 0.0;
      const float output = gyro_lowpass_[axis].reset(static_cast<float>(out(i)));
      out(i) = static_cast<double>(output);
    }
  }
  if (reset_output != nullptr)
    reset_output->wm = out;
  initialized_ = true;
  stats_.reset_count++;
}

void ImuFilter::update_sample_rate(double instantaneous_rate_hz, const ov_core::ImuData &sample) {
  if (!std::isfinite(instantaneous_rate_hz) || instantaneous_rate_hz <= 0.0)
    return;
  valid_interval_count_++;
  if (measured_sample_rate_hz_ <= 0.0)
    measured_sample_rate_hz_ = instantaneous_rate_hz;
  else
    measured_sample_rate_hz_ = (1.0 - kRateEmaAlpha) * measured_sample_rate_hz_ + kRateEmaAlpha * instantaneous_rate_hz;
  stats_.measured_sample_rate_hz = measured_sample_rate_hz_;

  if (options_.sample_rate_hz > 0.0 && valid_interval_count_ >= kRateWarningMinimumIntervals && !warned_rate_mismatch_) {
    const double mismatch = std::abs(measured_sample_rate_hz_ - options_.sample_rate_hz) / options_.sample_rate_hz;
    if (mismatch > kRateWarningRelativeThreshold) {
      std::cerr << "[IMU-FILTER] warning: measured rate " << measured_sample_rate_hz_ << " Hz differs from configured rate "
                << options_.sample_rate_hz << " Hz by " << 100.0 * mismatch << "%\n";
      warned_rate_mismatch_ = true;
    }
  }

  const double max_freq = std::max({options_.gyro_notch0.enabled ? options_.gyro_notch0.frequency_hz : 0.0,
                                    options_.gyro_notch1.enabled ? options_.gyro_notch1.frequency_hz : 0.0,
                                    options_.gyro_lowpass.enabled ? options_.gyro_lowpass.cutoff_hz : 0.0});
  if (valid_interval_count_ >= kRateWarningMinimumIntervals && max_freq >= 0.5 * measured_sample_rate_hz_)
    throw std::runtime_error("measured IMU sample rate makes configured gyro filter violate Nyquist");

  if (active_sample_rate_hz_ <= 0.0 ||
      std::abs(measured_sample_rate_hz_ - active_sample_rate_hz_) / active_sample_rate_hz_ > kRateUpdateRelativeThreshold)
    configure_axes(measured_sample_rate_hz_, &sample);
}

void ImuFilter::open_log_if_requested() {
  if (!has_active_filters() || !options_.log_raw_and_filtered)
    return;
  if (options_.log_path.empty())
    throw std::invalid_argument("imu_filter.log_path must not be empty when logging is enabled");
  const std::filesystem::path path(options_.log_path);
  if (path.has_parent_path())
    std::filesystem::create_directories(path.parent_path());
  log_stream_.open(path, std::ios::out | std::ios::trunc);
  if (!log_stream_)
    throw std::runtime_error("unable to open IMU raw/filtered log: " + options_.log_path);
  log_stream_ << "raw_timestamp,output_timestamp,raw_wx,raw_wy,raw_wz,filtered_wx,filtered_wy,filtered_wz,"
                 "raw_ax,raw_ay,raw_az,filtered_ax,filtered_ay,filtered_az,measured_rate_hz,active_rate_hz,"
                 "reset_count,coefficient_update_count,disable_count,nf0_enabled,nf1_enabled,lp_enabled\n";
  log_stream_ << std::setprecision(17);
}

void ImuFilter::write_log(const ov_core::ImuData &raw, const ov_core::ImuData &filtered) {
  if (!log_stream_.is_open())
    return;
  log_stream_ << raw.timestamp << ',' << filtered.timestamp;
  for (Eigen::Index i = 0; i < 3; ++i)
    log_stream_ << ',' << raw.wm(i);
  for (Eigen::Index i = 0; i < 3; ++i)
    log_stream_ << ',' << filtered.wm(i);
  for (Eigen::Index i = 0; i < 3; ++i)
    log_stream_ << ',' << raw.am(i);
  for (Eigen::Index i = 0; i < 3; ++i)
    log_stream_ << ',' << filtered.am(i);
  log_stream_ << ',' << measured_sample_rate_hz_ << ',' << active_sample_rate_hz_ << ',' << stats_.reset_count << ','
              << stats_.coefficient_update_count << ',' << stats_.disable_count << ',' << (int)options_.gyro_notch0.enabled
              << ',' << (int)options_.gyro_notch1.enabled << ',' << (int)options_.gyro_lowpass.enabled << '\n';
}

ov_core::ImuData ImuFilter::process(const ov_core::ImuData &raw) {
  if (!has_active_filters())
    return raw;
  const auto start = std::chrono::steady_clock::now();
  if (!std::isfinite(raw.timestamp))
    throw std::invalid_argument("non-finite raw IMU timestamp");

  ov_core::ImuData filtered = raw;
  bool reset_for_timestamp = false;
  bool used_reset_output = false;
  if (!initialized_) {
    if (active_sample_rate_hz_ > 0.0) {
      reset_all(raw, &filtered);
      used_reset_output = true;
    } else if (have_last_raw_timestamp_) {
      const double dt = raw.timestamp - last_raw_timestamp_;
      if (dt > 0.0 && dt <= 0.1)
        update_sample_rate(1.0 / dt, raw);
      else if (dt == 0.0)
        stats_.timestamp_duplicate_count++;
      else if (dt < 0.0)
        stats_.timestamp_backward_count++;
      else
        stats_.timestamp_gap_count++;
    }
  } else {
    const double dt = raw.timestamp - last_raw_timestamp_;
    const double gap_limit_s = active_sample_rate_hz_ > 0.0 ? std::max(0.1, 10.0 / active_sample_rate_hz_) : 0.1;
    if (dt == 0.0) {
      stats_.timestamp_duplicate_count++;
      reset_for_timestamp = true;
    } else if (dt < 0.0) {
      stats_.timestamp_backward_count++;
      reset_for_timestamp = true;
    } else if (dt > gap_limit_s) {
      stats_.timestamp_gap_count++;
      reset_for_timestamp = true;
    } else {
      update_sample_rate(1.0 / dt, raw);
    }
    if (reset_for_timestamp) {
      reset_all(raw, &filtered);
      used_reset_output = true;
    }
  }

  if (!initialized_) {
    last_raw_timestamp_ = raw.timestamp;
    have_last_raw_timestamp_ = true;
    write_log(raw, filtered);
    stats_.sample_count++;
    stats_.total_processing_time_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    return filtered;
  }

  const bool nonfinite_measurement = !raw.wm.allFinite() || !raw.am.allFinite();
  if (nonfinite_measurement && !used_reset_output) {
    reset_all(raw, &filtered);
    used_reset_output = true;
  }

  if (!used_reset_output)
    filtered.wm = apply_chain(raw.wm);

  if (!filtered.wm.allFinite() || !filtered.am.allFinite())
    throw std::invalid_argument("non-finite IMU value after gyro filtering");

  last_raw_timestamp_ = raw.timestamp;
  have_last_raw_timestamp_ = true;
  write_log(raw, filtered);
  stats_.sample_count++;
  stats_.total_processing_time_s += std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return filtered;
}

} // namespace ov_msckf
