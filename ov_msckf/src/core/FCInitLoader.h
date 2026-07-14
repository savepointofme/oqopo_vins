/*
 * Helper for loading a one-shot FC-assisted VIO initialization state.
 */

#ifndef OV_MSCKF_FC_INIT_LOADER_H
#define OV_MSCKF_FC_INIT_LOADER_H

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ov_msckf {

struct FCInitState {
  double timestamp = -1.0;
  Eigen::Vector4d q_GtoI = (Eigen::Vector4d() << 0, 0, 0, 1).finished();
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  double source_dt = 0.0;
  int valid_row_count = 0;
  int duplicate_timestamp_count = 0;
  int non_monotonic_timestamp_count = 0;
  std::string declared_position_frame;
};

struct FCInitLoadOptions {
  double max_abs_dt = 0.25;
  bool warn_only = false;
};

enum class FCInitLevel { I0_NEAREST, I1_EXACT_TIME, I2_ROBUST_SO3, I3_QUALITY_GATED };
enum class FCInitFallback { FAIL_CLOSED, I1_EXACT_TIME };
enum class InitializationMode {
  FC_FULL_STATE,
  FC_ATTITUDE_ONLY,
  VIO_ONLY,
  ONLINE_MULTISENSOR_ALIGNMENT
};

inline const char *initialization_mode_name(InitializationMode mode) {
  switch (mode) {
  case InitializationMode::FC_FULL_STATE: return "fc_full_state";
  case InitializationMode::FC_ATTITUDE_ONLY: return "fc_attitude_only";
  case InitializationMode::VIO_ONLY: return "vio_only";
  case InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT:
    return "online_multisensor_alignment";
  }
  return "unknown";
}

inline InitializationMode parse_initialization_mode(const std::string &value) {
  if (value == "fc_full_state")
    return InitializationMode::FC_FULL_STATE;
  if (value == "fc_attitude_only")
    return InitializationMode::FC_ATTITUDE_ONLY;
  if (value == "vio_only")
    return InitializationMode::VIO_ONLY;
  if (value == "online_multisensor_alignment" || value == "online_alignment")
    return InitializationMode::ONLINE_MULTISENSOR_ALIGNMENT;
  throw std::runtime_error(
      "initialization mode must be fc_full_state, fc_attitude_only, vio_only, "
      "or online_multisensor_alignment");
}

inline const char *fc_init_level_name(FCInitLevel level) {
  switch (level) {
  case FCInitLevel::I0_NEAREST: return "I0_nearest";
  case FCInitLevel::I1_EXACT_TIME: return "I1_exact_time";
  case FCInitLevel::I2_ROBUST_SO3: return "I2_exact_time_robust_so3";
  case FCInitLevel::I3_QUALITY_GATED: return "I3_quality_covariance_fallback";
  }
  return "unknown";
}

inline FCInitLevel parse_fc_init_level(const std::string &value) {
  if (value == "I0" || value == "i0" || value == "nearest")
    return FCInitLevel::I0_NEAREST;
  if (value == "I1" || value == "i1" || value == "exact_time")
    return FCInitLevel::I1_EXACT_TIME;
  if (value == "I2" || value == "i2" || value == "robust_so3")
    return FCInitLevel::I2_ROBUST_SO3;
  if (value == "I3" || value == "i3" || value == "quality_gated")
    return FCInitLevel::I3_QUALITY_GATED;
  throw std::runtime_error("FC init level must be I0, I1, I2, or I3");
}

inline const char *fc_init_fallback_name(FCInitFallback fallback) {
  return fallback == FCInitFallback::I1_EXACT_TIME ? "i1" : "fail_closed";
}

inline FCInitFallback parse_fc_init_fallback(const std::string &value) {
  if (value == "fail_closed")
    return FCInitFallback::FAIL_CLOSED;
  if (value == "i1")
    return FCInitFallback::I1_EXACT_TIME;
  throw std::runtime_error("FC init fallback must be fail_closed or i1");
}

struct FCInitSeriesRow {
  FCInitState state;
  int source_row_index = -1;
  std::string source_line;
};

struct FCInitSeries {
  std::vector<FCInitSeriesRow> rows;
  std::map<std::string, std::string> declarations;
  int duplicate_timestamp_count = 0;
  int non_monotonic_timestamp_count = 0;
};

struct FCInitBracket {
  FCInitSeriesRow before;
  FCInitSeriesRow after;
  double gap_s = std::numeric_limits<double>::quiet_NaN();
  double alpha = std::numeric_limits<double>::quiet_NaN();
  bool exact_source_row = false;
};

struct FCInitWindowDiagnostics {
  int sample_count = 0;
  double requested_duration_s = 0.0;
  double actual_duration_s = 0.0;
  double first_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double last_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  double max_source_gap_s = std::numeric_limits<double>::quiet_NaN();
  double residual_rms_deg = std::numeric_limits<double>::quiet_NaN();
  double residual_p95_deg = std::numeric_limits<double>::quiet_NaN();
  double residual_max_deg = std::numeric_limits<double>::quiet_NaN();
  double robust_inlier_fraction = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d fitted_angular_rate_rad_s = Eigen::Vector3d::Zero();
  Eigen::Vector3d attitude_std_rad = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double velocity_std_mps = std::numeric_limits<double>::quiet_NaN();
  double position_std_m = std::numeric_limits<double>::quiet_NaN();
  double gyro_bias_std_rad_s = std::numeric_limits<double>::quiet_NaN();
  double accel_bias_std_mps2 = std::numeric_limits<double>::quiet_NaN();
  bool quality_passed = false;
  std::string quality_reason;
  std::string bias_status;
  std::string bias_fallback_reason;
};

struct FCInitResult {
  FCInitState state;
  FCInitBracket bracket;
  FCInitWindowDiagnostics window;
  FCInitLevel requested_level = FCInitLevel::I0_NEAREST;
  FCInitLevel applied_level = FCInitLevel::I0_NEAREST;
  bool fallback_applied = false;
  std::string selection_method = "nearest_row_i0";
  std::string status = "ok";
  std::map<std::string, std::string> declarations;
};

struct FCInitExactOptions {
  FCInitLevel level = FCInitLevel::I0_NEAREST;
  FCInitFallback fallback = FCInitFallback::FAIL_CLOSED;
  double max_abs_dt_s = 0.25;
  double max_bracket_gap_s = 0.35;
  double window_duration_s = 5.0;
  int min_window_samples = 12;
  double max_window_source_gap_s = 0.35;
  double max_attitude_residual_p95_deg = 3.0;
  double huber_delta_deg = 1.5;
  double min_speed_mps = 0.0;
  double max_gyro_bias_norm_rad_s = 0.2;
  double max_accel_bias_norm_mps2 = 2.0;
};

enum class FCInitPositionFrame { LOCAL_W0_SEED, GLOBAL_GNAV };

inline const char *fc_init_position_frame_name(FCInitPositionFrame frame) {
  return frame == FCInitPositionFrame::GLOBAL_GNAV ? "global_gnav" : "local_w0_seed";
}

inline FCInitPositionFrame parse_fc_init_position_frame(const std::string &value) {
  if (value == "local_w0_seed")
    return FCInitPositionFrame::LOCAL_W0_SEED;
  if (value == "global_gnav")
    return FCInitPositionFrame::GLOBAL_GNAV;
  throw std::runtime_error(
      "FC init position frame must be 'local_w0_seed' or 'global_gnav'");
}

struct FCInitPositionBoundary {
  FCInitState local_seed;
  FCInitPositionFrame source_frame = FCInitPositionFrame::LOCAL_W0_SEED;
  bool has_global_nav_position = false;
  Eigen::Vector3d p_IinGnav = Eigen::Vector3d::Zero();
};

/// Convert the declared source position into the startup boundary.  Absolute
/// navigation position is retained outside the EKF; the EKF always starts at
/// the local W0 origin.  Only an input explicitly declared as a local W0 seed
/// is required to contain an already-zero position.
inline FCInitPositionBoundary prepare_fc_init_position(
    const FCInitState &state, FCInitPositionFrame frame,
    double local_zero_tolerance_m = 1e-9) {
  if (!state.p_IinG.allFinite())
    throw std::runtime_error("FC init position contains a non-finite value");
  if (frame == FCInitPositionFrame::LOCAL_W0_SEED &&
      state.p_IinG.cwiseAbs().maxCoeff() > local_zero_tolerance_m) {
    std::ostringstream oss;
    oss << "FC init position declared local_w0_seed must be zero; max_abs="
        << state.p_IinG.cwiseAbs().maxCoeff()
        << " m exceeds tolerance=" << local_zero_tolerance_m << " m";
    throw std::runtime_error(oss.str());
  }

  FCInitPositionBoundary boundary;
  boundary.local_seed = state;
  boundary.local_seed.p_IinG.setZero();
  boundary.source_frame = frame;
  boundary.has_global_nav_position = frame == FCInitPositionFrame::GLOBAL_GNAV;
  if (boundary.has_global_nav_position)
    boundary.p_IinGnav = state.p_IinG;
  return boundary;
}

inline std::string fcinit_trim(const std::string &s) {
  size_t a = 0;
  while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a])))
    a++;
  size_t b = s.size();
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1])))
    b--;
  return s.substr(a, b - a);
}

inline std::vector<std::string> fcinit_split_csv(const std::string &line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ','))
    out.push_back(fcinit_trim(item));
  return out;
}

inline bool fcinit_parse_row(const std::string &line, FCInitState &state) {
  std::string s = fcinit_trim(line);
  if (s.empty() || s[0] == '#')
    return false;
  std::vector<std::string> c = fcinit_split_csv(s);
  if (c.size() < 17)
    return false;
  try {
    state.timestamp = std::stod(c[0]);
    state.q_GtoI << std::stod(c[1]), std::stod(c[2]), std::stod(c[3]), std::stod(c[4]);
    state.v_IinG << std::stod(c[5]), std::stod(c[6]), std::stod(c[7]);
    state.p_IinG << std::stod(c[8]), std::stod(c[9]), std::stod(c[10]);
    state.bg << std::stod(c[11]), std::stod(c[12]), std::stod(c[13]);
    state.ba << std::stod(c[14]), std::stod(c[15]), std::stod(c[16]);
    double qn = state.q_GtoI.norm();
    if (qn <= 1e-12)
      throw std::runtime_error("zero quaternion");
    state.q_GtoI /= qn;
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

inline FCInitState load_fc_init_state_csv(const std::string &path,
                                          double target_time,
                                          const FCInitLoadOptions &options = FCInitLoadOptions()) {
  std::ifstream in(path);
  if (!in.is_open())
    throw std::runtime_error("cannot open FC init CSV: " + path);

  bool have = false;
  FCInitState best;
  double best_abs_dt = std::numeric_limits<double>::infinity();
  int valid_row_count = 0;
  int duplicate_timestamp_count = 0;
  int non_monotonic_timestamp_count = 0;
  bool have_previous = false;
  double previous_timestamp = 0.0;
  std::vector<double> seen_timestamps;
  std::string declared_position_frame;
  std::string line;
  while (std::getline(in, line)) {
    const std::string trimmed = fcinit_trim(line);
    const std::string declaration = "# position_frame=";
    if (trimmed.compare(0, declaration.size(), declaration) == 0) {
      const std::string value = fcinit_trim(trimmed.substr(declaration.size()));
      (void)parse_fc_init_position_frame(value);
      if (!declared_position_frame.empty() && declared_position_frame != value)
        throw std::runtime_error("conflicting FC init position_frame declarations");
      declared_position_frame = value;
      continue;
    }
    FCInitState candidate;
    if (!fcinit_parse_row(line, candidate))
      continue;
    valid_row_count++;
    if (have_previous && candidate.timestamp < previous_timestamp)
      non_monotonic_timestamp_count++;
    have_previous = true;
    previous_timestamp = candidate.timestamp;
    for (double seen : seen_timestamps) {
      if (std::fabs(seen - candidate.timestamp) < 1e-9) {
        duplicate_timestamp_count++;
        break;
      }
    }
    seen_timestamps.push_back(candidate.timestamp);
    double abs_dt = std::fabs(candidate.timestamp - target_time);
    if (!have || abs_dt < best_abs_dt) {
      have = true;
      best = candidate;
      best_abs_dt = abs_dt;
    }
  }
  if (!have)
    throw std::runtime_error("no valid FC init rows in: " + path);
  best.source_dt = best.timestamp - target_time;
  best.valid_row_count = valid_row_count;
  best.duplicate_timestamp_count = duplicate_timestamp_count;
  best.non_monotonic_timestamp_count = non_monotonic_timestamp_count;
  best.declared_position_frame = declared_position_frame;
  if (std::isfinite(options.max_abs_dt) && options.max_abs_dt >= 0.0 && best_abs_dt > options.max_abs_dt && !options.warn_only) {
    std::ostringstream oss;
    oss << "FC init timestamp mismatch: selected t=" << best.timestamp
        << " target=" << target_time
        << " dt=" << best.source_dt
        << " exceeds max_abs_dt=" << options.max_abs_dt
        << " from " << path;
    throw std::runtime_error(oss.str());
  }
  return best;
}

inline FCInitSeries load_fc_init_series_csv(const std::string &path) {
  std::ifstream in(path);
  if (!in.is_open())
    throw std::runtime_error("cannot open FC init CSV: " + path);

  FCInitSeries series;
  std::string line;
  int line_number = 0;
  bool have_previous = false;
  double previous_timestamp = 0.0;
  std::vector<double> seen_timestamps;
  while (std::getline(in, line)) {
    line_number++;
    const std::string trimmed = fcinit_trim(line);
    if (!trimmed.empty() && trimmed[0] == '#') {
      const size_t equals = trimmed.find('=');
      if (equals != std::string::npos) {
        const std::string key = fcinit_trim(trimmed.substr(1, equals - 1));
        const std::string value = fcinit_trim(trimmed.substr(equals + 1));
        const auto existing = series.declarations.find(key);
        if (existing != series.declarations.end() && existing->second != value)
          throw std::runtime_error("conflicting FC init declaration: " + key);
        series.declarations[key] = value;
      }
      continue;
    }

    FCInitState state;
    if (!fcinit_parse_row(line, state))
      continue;
    if (!state.q_GtoI.allFinite() || !state.v_IinG.allFinite() ||
        !state.p_IinG.allFinite() || !state.bg.allFinite() ||
        !state.ba.allFinite() || !std::isfinite(state.timestamp))
      throw std::runtime_error("non-finite FC init row at line " +
                               std::to_string(line_number));
    if (have_previous && state.timestamp < previous_timestamp)
      series.non_monotonic_timestamp_count++;
    have_previous = true;
    previous_timestamp = state.timestamp;
    for (double seen : seen_timestamps) {
      if (std::fabs(seen - state.timestamp) < 1e-9) {
        series.duplicate_timestamp_count++;
        break;
      }
    }
    seen_timestamps.push_back(state.timestamp);
    FCInitSeriesRow row;
    row.state = state;
    row.source_row_index = line_number;
    row.source_line = line;
    series.rows.push_back(row);
  }
  if (series.rows.empty())
    throw std::runtime_error("no valid FC init rows in: " + path);
  const auto declared_frame = series.declarations.find("position_frame");
  if (declared_frame != series.declarations.end())
    (void)parse_fc_init_position_frame(declared_frame->second);
  return series;
}

inline Eigen::Quaterniond fcinit_eigen_quaternion(const Eigen::Vector4d &q) {
  // OpenVINS stores JPL/passive quaternions, while Eigen uses Hamilton/active
  // quaternions. Negating the vector part preserves the represented matrix.
  Eigen::Quaterniond out(q(3), -q(0), -q(1), -q(2));
  out.normalize();
  return out;
}

inline Eigen::Vector4d fcinit_jpl_quaternion(const Eigen::Quaterniond &q_in,
                                             const Eigen::Vector4d *reference = nullptr) {
  Eigen::Quaterniond q = q_in.normalized();
  Eigen::Vector4d out(-q.x(), -q.y(), -q.z(), q.w());
  if (reference != nullptr && out.dot(*reference) < 0.0)
    out = -out;
  return out;
}

inline Eigen::Vector3d fcinit_so3_log(const Eigen::Matrix3d &rotation) {
  Eigen::AngleAxisd aa(rotation);
  if (!std::isfinite(aa.angle()) || aa.angle() < 1e-12)
    return Eigen::Vector3d::Zero();
  return aa.axis() * aa.angle();
}

inline Eigen::Matrix3d fcinit_so3_exp(const Eigen::Vector3d &tangent) {
  const double angle = tangent.norm();
  if (angle < 1e-12)
    return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, tangent / angle).toRotationMatrix();
}

inline double fcinit_percentile(std::vector<double> values, double probability) {
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  probability = std::max(0.0, std::min(1.0, probability));
  const double index = probability * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(index));
  const size_t upper = static_cast<size_t>(std::ceil(index));
  const double alpha = index - static_cast<double>(lower);
  return values[lower] * (1.0 - alpha) + values[upper] * alpha;
}

inline double fcinit_median(std::vector<double> values) {
  return fcinit_percentile(std::move(values), 0.5);
}

inline double fcinit_robust_std(const std::vector<double> &values) {
  if (values.size() < 2)
    return std::numeric_limits<double>::quiet_NaN();
  const double median = fcinit_median(values);
  std::vector<double> absolute_deviation;
  absolute_deviation.reserve(values.size());
  for (double value : values)
    absolute_deviation.push_back(std::fabs(value - median));
  return 1.4826 * fcinit_median(std::move(absolute_deviation));
}

inline Eigen::Vector3d fcinit_component_median(
    const std::vector<Eigen::Vector3d> &values) {
  Eigen::Vector3d result = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; axis++) {
    std::vector<double> component;
    component.reserve(values.size());
    for (const auto &value : values)
      component.push_back(value(axis));
    result(axis) = fcinit_median(std::move(component));
  }
  return result;
}

inline double fcinit_vector_robust_std(
    const std::vector<Eigen::Vector3d> &values) {
  if (values.size() < 2)
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d center = fcinit_component_median(values);
  std::vector<double> norms;
  norms.reserve(values.size());
  for (const auto &value : values)
    norms.push_back((value - center).norm());
  return fcinit_robust_std(norms);
}

inline FCInitBracket fcinit_find_bracket(const FCInitSeries &series,
                                         double target_time,
                                         double max_bracket_gap_s) {
  if (series.duplicate_timestamp_count > 0)
    throw std::runtime_error("exact-time FC init rejects duplicate timestamps");
  if (series.non_monotonic_timestamp_count > 0)
    throw std::runtime_error("exact-time FC init rejects non-monotonic timestamps");

  FCInitBracket bracket;
  for (const auto &row : series.rows) {
    if (std::fabs(row.state.timestamp - target_time) < 1e-9) {
      bracket.before = row;
      bracket.after = row;
      bracket.gap_s = 0.0;
      bracket.alpha = 0.0;
      bracket.exact_source_row = true;
      return bracket;
    }
  }
  bool have_before = false;
  bool have_after = false;
  for (const auto &row : series.rows) {
    if (row.state.timestamp < target_time) {
      bracket.before = row;
      have_before = true;
    } else if (row.state.timestamp > target_time) {
      bracket.after = row;
      have_after = true;
      break;
    }
  }
  if (!have_before || !have_after)
    throw std::runtime_error("FC init target is not enclosed by source rows");
  bracket.gap_s = bracket.after.state.timestamp - bracket.before.state.timestamp;
  if (!(bracket.gap_s > 0.0))
    throw std::runtime_error("FC init bracket has a non-positive gap");
  if (std::isfinite(max_bracket_gap_s) && max_bracket_gap_s >= 0.0 &&
      bracket.gap_s > max_bracket_gap_s) {
    std::ostringstream oss;
    oss << "FC init bracket gap=" << bracket.gap_s
        << " exceeds max_bracket_gap_s=" << max_bracket_gap_s;
    throw std::runtime_error(oss.str());
  }
  bracket.alpha = (target_time - bracket.before.state.timestamp) / bracket.gap_s;
  return bracket;
}

inline FCInitState fcinit_interpolate_bracket(const FCInitBracket &bracket,
                                              double target_time,
                                              const FCInitSeries &series) {
  FCInitState state;
  state.timestamp = target_time;
  state.source_dt = 0.0;
  state.valid_row_count = static_cast<int>(series.rows.size());
  state.duplicate_timestamp_count = series.duplicate_timestamp_count;
  state.non_monotonic_timestamp_count = series.non_monotonic_timestamp_count;
  const auto frame = series.declarations.find("position_frame");
  if (frame != series.declarations.end())
    state.declared_position_frame = frame->second;
  if (bracket.exact_source_row) {
    const int valid_count = state.valid_row_count;
    const int duplicates = state.duplicate_timestamp_count;
    const int non_monotonic = state.non_monotonic_timestamp_count;
    const std::string declared_frame = state.declared_position_frame;
    state = bracket.before.state;
    state.timestamp = target_time;
    state.source_dt = 0.0;
    state.valid_row_count = valid_count;
    state.duplicate_timestamp_count = duplicates;
    state.non_monotonic_timestamp_count = non_monotonic;
    state.declared_position_frame = declared_frame;
    return state;
  }
  const double alpha = bracket.alpha;
  const Eigen::Quaterniond q0 = fcinit_eigen_quaternion(bracket.before.state.q_GtoI);
  const Eigen::Quaterniond q1 = fcinit_eigen_quaternion(bracket.after.state.q_GtoI);
  state.q_GtoI = fcinit_jpl_quaternion(q0.slerp(alpha, q1),
                                      &bracket.before.state.q_GtoI);
  state.v_IinG = (1.0 - alpha) * bracket.before.state.v_IinG +
                 alpha * bracket.after.state.v_IinG;
  state.p_IinG = (1.0 - alpha) * bracket.before.state.p_IinG +
                 alpha * bracket.after.state.p_IinG;
  state.bg = (1.0 - alpha) * bracket.before.state.bg + alpha * bracket.after.state.bg;
  state.ba = (1.0 - alpha) * bracket.before.state.ba + alpha * bracket.after.state.ba;
  return state;
}

inline FCInitResult fcinit_select_nearest(const FCInitSeries &series,
                                          double target_time,
                                          double max_abs_dt_s) {
  const FCInitSeriesRow *best = nullptr;
  double best_abs_dt = std::numeric_limits<double>::infinity();
  for (const auto &row : series.rows) {
    const double absolute_dt = std::fabs(row.state.timestamp - target_time);
    if (best == nullptr || absolute_dt < best_abs_dt) {
      best = &row;
      best_abs_dt = absolute_dt;
    }
  }
  if (best == nullptr)
    throw std::runtime_error("no FC init row is available for nearest selection");
  if (std::isfinite(max_abs_dt_s) && max_abs_dt_s >= 0.0 &&
      best_abs_dt > max_abs_dt_s) {
    std::ostringstream oss;
    oss << "FC init nearest-row dt=" << best_abs_dt
        << " exceeds max_abs_dt_s=" << max_abs_dt_s;
    throw std::runtime_error(oss.str());
  }
  FCInitResult result;
  result.state = best->state;
  result.state.source_dt = best->state.timestamp - target_time;
  result.state.valid_row_count = static_cast<int>(series.rows.size());
  result.state.duplicate_timestamp_count = series.duplicate_timestamp_count;
  result.state.non_monotonic_timestamp_count = series.non_monotonic_timestamp_count;
  const auto frame = series.declarations.find("position_frame");
  if (frame != series.declarations.end())
    result.state.declared_position_frame = frame->second;
  result.bracket.before = *best;
  result.bracket.after = *best;
  result.bracket.gap_s = 0.0;
  result.bracket.alpha = 0.0;
  result.bracket.exact_source_row = std::fabs(result.state.source_dt) < 1e-9;
  result.requested_level = FCInitLevel::I0_NEAREST;
  result.applied_level = FCInitLevel::I0_NEAREST;
  result.selection_method = "nearest_row_i0";
  result.declarations = series.declarations;
  return result;
}

inline FCInitResult fcinit_select_exact(const FCInitSeries &series,
                                        double target_time,
                                        const FCInitExactOptions &options) {
  FCInitResult result;
  result.requested_level = options.level;
  result.applied_level = FCInitLevel::I1_EXACT_TIME;
  result.bracket = fcinit_find_bracket(series, target_time,
                                       options.max_bracket_gap_s);
  result.state = fcinit_interpolate_bracket(result.bracket, target_time, series);
  result.selection_method = result.bracket.exact_source_row
                                ? "exact_source_row_i1"
                                : "bracket_slerp_linear_i1";
  result.declarations = series.declarations;
  return result;
}

inline void fcinit_apply_robust_window(const FCInitSeries &series,
                                       double target_time,
                                       const FCInitExactOptions &options,
                                       FCInitResult &result,
                                       bool apply_bias_median) {
  const double requested_duration =
      std::max(3.0, std::min(8.0, options.window_duration_s));
  const double half_window = 0.5 * requested_duration;
  std::vector<const FCInitSeriesRow *> selected;
  for (const auto &row : series.rows) {
    if (row.state.timestamp >= target_time - half_window &&
        row.state.timestamp <= target_time + half_window)
      selected.push_back(&row);
  }
  result.window.requested_duration_s = requested_duration;
  result.window.sample_count = static_cast<int>(selected.size());
  if (static_cast<int>(selected.size()) < options.min_window_samples)
    throw std::runtime_error("FC init robust window has too few samples");

  const double first_time = selected.front()->state.timestamp;
  const double last_time = selected.back()->state.timestamp;
  result.window.first_timestamp_s = first_time;
  result.window.last_timestamp_s = last_time;
  result.window.actual_duration_s = last_time - first_time;
  double max_gap = 0.0;
  for (size_t index = 1; index < selected.size(); index++)
    max_gap = std::max(max_gap, selected[index]->state.timestamp -
                                   selected[index - 1]->state.timestamp);
  result.window.max_source_gap_s = max_gap;

  const Eigen::Matrix3d reference_rotation =
      fcinit_eigen_quaternion(result.state.q_GtoI).toRotationMatrix();
  std::vector<double> times;
  std::vector<Eigen::Vector3d> tangents;
  times.reserve(selected.size());
  tangents.reserve(selected.size());
  for (const auto *row : selected) {
    times.push_back(row->state.timestamp - target_time);
    const Eigen::Matrix3d rotation =
        fcinit_eigen_quaternion(row->state.q_GtoI).toRotationMatrix();
    tangents.push_back(fcinit_so3_log(rotation * reference_rotation.transpose()));
  }

  std::vector<double> weights(selected.size(), 1.0);
  Eigen::Matrix<double, 2, 3> coefficients =
      Eigen::Matrix<double, 2, 3>::Zero();
  const double huber_delta_rad =
      options.huber_delta_deg * 3.14159265358979323846 / 180.0;
  for (int iteration = 0; iteration < 12; iteration++) {
    Eigen::Matrix2d normal = Eigen::Matrix2d::Zero();
    Eigen::Matrix<double, 2, 3> rhs =
        Eigen::Matrix<double, 2, 3>::Zero();
    for (size_t index = 0; index < selected.size(); index++) {
      Eigen::Vector2d design(1.0, times[index]);
      normal += weights[index] * design * design.transpose();
      rhs += weights[index] * design * tangents[index].transpose();
    }
    if (std::fabs(normal.determinant()) < 1e-12)
      throw std::runtime_error("FC init robust window has singular time support");
    coefficients = normal.ldlt().solve(rhs);
    for (size_t index = 0; index < selected.size(); index++) {
      const Eigen::Vector3d fitted =
          coefficients.row(0).transpose() +
          coefficients.row(1).transpose() * times[index];
      const double residual_norm = (tangents[index] - fitted).norm();
      weights[index] = residual_norm <= huber_delta_rad || residual_norm < 1e-12
                           ? 1.0
                           : huber_delta_rad / residual_norm;
    }
  }

  const Eigen::Vector3d intercept = coefficients.row(0).transpose();
  result.window.fitted_angular_rate_rad_s = coefficients.row(1).transpose();
  std::vector<double> residual_norms_rad;
  residual_norms_rad.reserve(selected.size());
  Eigen::Vector3d weighted_component_sse = Eigen::Vector3d::Zero();
  double weight_sum = 0.0;
  int robust_inliers = 0;
  for (size_t index = 0; index < selected.size(); index++) {
    const Eigen::Vector3d fitted = intercept +
        result.window.fitted_angular_rate_rad_s * times[index];
    const Eigen::Vector3d residual = tangents[index] - fitted;
    residual_norms_rad.push_back(residual.norm());
    weighted_component_sse += weights[index] * residual.cwiseProduct(residual);
    weight_sum += weights[index];
    if (weights[index] >= 0.999)
      robust_inliers++;
  }
  const double rad_to_deg = 180.0 / 3.14159265358979323846;
  double squared_sum = 0.0;
  std::vector<double> residual_norms_deg;
  residual_norms_deg.reserve(residual_norms_rad.size());
  for (double residual : residual_norms_rad) {
    squared_sum += residual * residual;
    residual_norms_deg.push_back(residual * rad_to_deg);
  }
  result.window.residual_rms_deg =
      std::sqrt(squared_sum / static_cast<double>(residual_norms_rad.size())) *
      rad_to_deg;
  result.window.residual_p95_deg = fcinit_percentile(residual_norms_deg, 0.95);
  result.window.residual_max_deg =
      *std::max_element(residual_norms_deg.begin(), residual_norms_deg.end());
  result.window.robust_inlier_fraction =
      static_cast<double>(robust_inliers) / static_cast<double>(selected.size());
  if (weight_sum > 2.0)
    result.window.attitude_std_rad =
        (weighted_component_sse / (weight_sum - 2.0)).cwiseSqrt();

  const Eigen::Matrix3d robust_seed_rotation =
      fcinit_so3_exp(intercept) * reference_rotation;
  result.state.q_GtoI = fcinit_jpl_quaternion(
      Eigen::Quaterniond(robust_seed_rotation), &result.state.q_GtoI);

  std::vector<Eigen::Vector3d> velocities;
  std::vector<Eigen::Vector3d> position_residuals;
  std::vector<Eigen::Vector3d> gyro_biases;
  std::vector<Eigen::Vector3d> accel_biases;
  velocities.reserve(selected.size());
  position_residuals.reserve(selected.size());
  gyro_biases.reserve(selected.size());
  accel_biases.reserve(selected.size());
  for (const auto *row : selected) {
    const double dt = row->state.timestamp - target_time;
    velocities.push_back(row->state.v_IinG);
    position_residuals.push_back(
        row->state.p_IinG - (result.state.p_IinG + result.state.v_IinG * dt));
    gyro_biases.push_back(row->state.bg);
    accel_biases.push_back(row->state.ba);
  }
  result.window.velocity_std_mps = fcinit_vector_robust_std(velocities);
  result.window.position_std_m = fcinit_vector_robust_std(position_residuals);
  result.window.gyro_bias_std_rad_s = fcinit_vector_robust_std(gyro_biases);
  result.window.accel_bias_std_mps2 = fcinit_vector_robust_std(accel_biases);
  if (apply_bias_median) {
    result.state.bg = fcinit_component_median(gyro_biases);
    result.state.ba = fcinit_component_median(accel_biases);
    result.window.bias_status = "robust_window_median";
  } else {
    result.window.bias_status = "exact_time_interpolation";
  }
}

inline std::string fcinit_quality_failure(const FCInitResult &result,
                                          const FCInitExactOptions &options) {
  std::vector<std::string> failures;
  if (result.window.sample_count < options.min_window_samples)
    failures.push_back("sample_count");
  const double minimum_duration =
      0.8 * std::max(3.0, std::min(8.0, options.window_duration_s));
  if (!std::isfinite(result.window.actual_duration_s) ||
      result.window.actual_duration_s < minimum_duration)
    failures.push_back("window_duration");
  if (!std::isfinite(result.window.max_source_gap_s) ||
      result.window.max_source_gap_s > options.max_window_source_gap_s)
    failures.push_back("source_gap");
  if (!std::isfinite(result.window.residual_p95_deg) ||
      result.window.residual_p95_deg > options.max_attitude_residual_p95_deg)
    failures.push_back("attitude_residual_p95");
  if (!result.state.q_GtoI.allFinite() || !result.state.p_IinG.allFinite() ||
      !result.state.v_IinG.allFinite() || !result.state.bg.allFinite() ||
      !result.state.ba.allFinite())
    failures.push_back("non_finite_state");
  if (result.state.v_IinG.norm() < options.min_speed_mps)
    failures.push_back("minimum_speed");
  if (result.state.bg.norm() > options.max_gyro_bias_norm_rad_s)
    failures.push_back("gyro_bias_norm");
  if (result.state.ba.norm() > options.max_accel_bias_norm_mps2)
    failures.push_back("accel_bias_norm");
  std::ostringstream reason;
  for (size_t index = 0; index < failures.size(); index++) {
    if (index > 0)
      reason << ';';
    reason << failures[index];
  }
  return reason.str();
}

inline FCInitResult load_fc_init_result_csv(
    const std::string &path, double target_time,
    const FCInitExactOptions &options = FCInitExactOptions()) {
  const FCInitSeries series = load_fc_init_series_csv(path);
  if (options.level == FCInitLevel::I0_NEAREST)
    return fcinit_select_nearest(series, target_time, options.max_abs_dt_s);

  FCInitResult exact = fcinit_select_exact(series, target_time, options);
  exact.requested_level = options.level;
  if (options.level == FCInitLevel::I1_EXACT_TIME)
    return exact;

  try {
    fcinit_apply_robust_window(series, target_time, options, exact,
                               options.level == FCInitLevel::I3_QUALITY_GATED);
  } catch (const std::exception &error) {
    if (options.level == FCInitLevel::I3_QUALITY_GATED &&
        options.fallback == FCInitFallback::I1_EXACT_TIME) {
      exact.applied_level = FCInitLevel::I1_EXACT_TIME;
      exact.fallback_applied = true;
      exact.status = "fallback_i1";
      exact.selection_method = "quality_gate_fallback_i1";
      exact.window.quality_passed = false;
      exact.window.quality_reason = error.what();
      exact.window.bias_status = "exact_time_interpolation";
      exact.window.bias_fallback_reason = error.what();
      return exact;
    }
    throw;
  }

  exact.applied_level = options.level;
  exact.selection_method = options.level == FCInitLevel::I2_ROBUST_SO3
                               ? "exact_time_robust_so3_i2"
                               : "exact_time_quality_gated_i3";
  if (options.level == FCInitLevel::I2_ROBUST_SO3) {
    exact.window.quality_passed = true;
    exact.window.quality_reason = "diagnostic_only_not_gated";
    return exact;
  }

  exact.window.quality_reason = fcinit_quality_failure(exact, options);
  exact.window.quality_passed = exact.window.quality_reason.empty();
  if (exact.window.quality_passed)
    return exact;
  if (options.fallback == FCInitFallback::I1_EXACT_TIME) {
    FCInitResult fallback = fcinit_select_exact(series, target_time, options);
    fallback.requested_level = FCInitLevel::I3_QUALITY_GATED;
    fallback.applied_level = FCInitLevel::I1_EXACT_TIME;
    fallback.fallback_applied = true;
    fallback.status = "fallback_i1";
    fallback.selection_method = "quality_gate_fallback_i1";
    fallback.window = exact.window;
    fallback.window.bias_status = "exact_time_interpolation";
    fallback.window.bias_fallback_reason = exact.window.quality_reason;
    return fallback;
  }
  throw std::runtime_error("FC init I3 quality gate failed: " +
                           exact.window.quality_reason);
}

} // namespace ov_msckf

#endif // OV_MSCKF_FC_INIT_LOADER_H
