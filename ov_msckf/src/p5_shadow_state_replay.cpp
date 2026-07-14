#include "ros_free/AdaptiveStrideController.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

std::vector<std::string> split_csv(const std::string &line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (char ch : line) {
    if (ch == '"') {
      quoted = !quoted;
    } else if (ch == ',' && !quoted) {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(ch);
    }
  }
  fields.push_back(field);
  return fields;
}

double number(const std::vector<std::string> &row,
              const std::unordered_map<std::string, size_t> &columns,
              const std::string &name,
              double fallback = std::numeric_limits<double>::quiet_NaN()) {
  auto it = columns.find(name);
  if (it == columns.end() || it->second >= row.size() || row[it->second].empty())
    return fallback;
  char *end = nullptr;
  const double value = std::strtod(row[it->second].c_str(), &end);
  return end == row[it->second].c_str() ? fallback : value;
}

int integer(const std::vector<std::string> &row,
            const std::unordered_map<std::string, size_t> &columns,
            const std::string &name, int fallback = -1) {
  const double value = number(row, columns, name, fallback);
  return std::isfinite(value) ? static_cast<int>(value) : fallback;
}

} // namespace

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: p5_shadow_state_replay INPUT.csv OUTPUT.csv\n";
    return 2;
  }
  std::ifstream input(argv[1]);
  std::ofstream output(argv[2], std::ios::trunc);
  if (!input || !output) {
    std::cerr << "failed to open input/output\n";
    return 3;
  }
  std::string line;
  if (!std::getline(input, line))
    return 4;
  const auto header = split_csv(line);
  std::unordered_map<std::string, size_t> columns;
  for (size_t i = 0; i < header.size(); ++i)
    columns[header[i]] = i;

  output << "timestamp,initialized,policy_state,previous_policy_state,"
            "policy_state_changed,main_trigger,transition_reason,tracking_stride,"
            "backend_update_stride,tracking_stride_changed,backend_stride_changed,"
            "emergency_downshift,normal_recovery,unnecessary_reversal,"
            "minimum_dwell_violation,same_state_target_rewrite,state_dwell_s,"
            "transition_guard_s,low_altitude_band,predicted_overlap,"
            "predicted_median_displacement_px,predicted_p95_displacement_px,"
            "relative_height_m,horizontal_speed_mps,vertical_speed_mps,roll_deg,"
            "pitch_deg,gyro_norm_radps,median_parallax_px,p95_parallax_px,"
            "actual_received_camera_dt_s,actual_tracking_frame_interval_s,"
            "actual_backend_frame_interval_s,active_msckf_features,"
            "active_slam_features,median_track_age_frames,visual_residual_rmse_px,"
            "visual_residual_p95_px,msckf_input_count,msckf_accepted_count,"
            "msckf_rejected_count,time_since_accepted_backend_update_s,"
            "covariance_all_finite,covariance_negative_diagonal_count,"
            "position_jump_m,velocity_jump_mps,attitude_jump_deg\n";
  output << std::setprecision(12);

  ov_msckf::AdaptiveStrideController controller;
  while (std::getline(input, line)) {
    if (line.empty())
      continue;
    const auto row = split_csv(line);
    ov_msckf::AdaptiveStrideInput in;
    in.timestamp = number(row, columns, "timestamp", 0.0);
    in.initialized = integer(row, columns, "initialized", 0) != 0;
    in.relative_height_m = number(row, columns, "relative_height_m");
    in.horizontal_speed_mps = number(row, columns, "horizontal_speed_mps");
    in.vertical_speed_mps = number(row, columns, "vertical_speed_mps");
    in.roll_deg = number(row, columns, "roll_deg");
    in.pitch_deg = number(row, columns, "pitch_deg");
    in.gyro_norm_radps = number(row, columns, "gyro_norm_radps");
    in.median_parallax_px = number(row, columns, "median_parallax_px");
    in.p95_parallax_px = number(row, columns, "p95_parallax_px");
    in.parallax_measurement_timestamp_s =
        number(row, columns, "parallax_measurement_timestamp_s");
    in.parallax_measurement_dt_s =
        number(row, columns, "parallax_measurement_dt_s");
    in.actual_received_camera_dt_s =
        number(row, columns, "actual_received_camera_dt_s");
    in.actual_tracking_frame_interval_s =
        number(row, columns, "actual_tracking_frame_interval_s");
    in.actual_backend_frame_interval_s =
        number(row, columns, "actual_backend_frame_interval_s");
    in.active_msckf_features =
        integer(row, columns, "active_msckf_features");
    in.active_slam_features = integer(row, columns, "active_slam_features");
    in.configured_feature_count =
        integer(row, columns, "configured_feature_count", 400);
    in.median_track_age_frames =
        number(row, columns, "median_track_age_frames");
    in.visual_residual_rmse_px =
        number(row, columns, "visual_residual_rmse_px");
    in.visual_residual_p95_px =
        number(row, columns, "visual_residual_p95_px");
    in.msckf_input_count = integer(row, columns, "msckf_input_count");
    in.msckf_accepted_count = integer(row, columns, "msckf_accepted_count");
    in.msckf_rejected_count = integer(row, columns, "msckf_rejected_count");
    in.time_since_accepted_backend_update_s =
        number(row, columns, "time_since_accepted_backend_update_s");
    in.covariance_all_finite =
        integer(row, columns, "covariance_all_finite", 1) != 0;
    in.covariance_negative_diagonal_count =
        integer(row, columns, "covariance_negative_diagonal_count", 0);
    in.position_jump_m = number(row, columns, "position_jump_m", 0.0);
    in.velocity_jump_mps = number(row, columns, "velocity_jump_mps", 0.0);
    in.attitude_jump_deg = number(row, columns, "attitude_jump_deg", 0.0);

    const auto d = controller.update(in);
    output << in.timestamp << ',' << (in.initialized ? 1 : 0) << ','
           << d.policy_state << ',' << d.previous_policy_state << ','
           << (d.policy_state_changed ? 1 : 0) << ',' << d.main_trigger << ','
           << d.transition_reason << ',' << d.tracking_stride << ','
           << d.backend_update_stride << ','
           << (d.tracking_stride_changed ? 1 : 0) << ','
           << (d.backend_stride_changed ? 1 : 0) << ','
           << (d.emergency_downshift ? 1 : 0) << ','
           << (d.normal_recovery ? 1 : 0) << ','
           << (d.unnecessary_reversal ? 1 : 0) << ','
           << (d.minimum_dwell_violation ? 1 : 0) << ','
           << (d.same_state_target_rewrite ? 1 : 0) << ',' << d.state_dwell_s
           << ',' << d.transition_guard_s << ',' << d.low_altitude_band << ','
           << d.predicted_overlap << ',' << d.predicted_median_displacement_px
           << ',' << d.predicted_p95_displacement_px << ','
           << in.relative_height_m << ',' << in.horizontal_speed_mps << ','
           << in.vertical_speed_mps << ',' << in.roll_deg << ',' << in.pitch_deg
           << ',' << in.gyro_norm_radps << ',' << in.median_parallax_px << ','
           << in.p95_parallax_px << ',' << in.actual_received_camera_dt_s << ','
           << in.actual_tracking_frame_interval_s << ','
           << in.actual_backend_frame_interval_s << ','
           << in.active_msckf_features << ',' << in.active_slam_features << ','
           << in.median_track_age_frames << ',' << in.visual_residual_rmse_px
           << ',' << in.visual_residual_p95_px << ',' << in.msckf_input_count
           << ',' << in.msckf_accepted_count << ',' << in.msckf_rejected_count
           << ',' << in.time_since_accepted_backend_update_s << ','
           << (in.covariance_all_finite ? 1 : 0) << ','
           << in.covariance_negative_diagonal_count << ',' << in.position_jump_m
           << ',' << in.velocity_jump_mps << ',' << in.attitude_jump_deg << '\n';
  }
  return output.good() ? 0 : 5;
}
