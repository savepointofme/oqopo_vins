#include "core/FCInitLoader.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using ov_msckf::FCInitLoadOptions;
using ov_msckf::FCInitExactOptions;
using ov_msckf::FCInitFallback;
using ov_msckf::FCInitLevel;
using ov_msckf::FCInitPositionFrame;
using ov_msckf::FCInitState;
using ov_msckf::InitializationMode;
using ov_msckf::load_fc_init_result_csv;
using ov_msckf::load_fc_init_state_csv;
using ov_msckf::parse_fc_init_position_frame;
using ov_msckf::parse_initialization_mode;
using ov_msckf::prepare_fc_init_position;

static void require_true(bool cond, const std::string &msg) {
  if (!cond)
    throw std::runtime_error(msg);
}

static void require_near(double actual, double expected, double tol, const std::string &msg) {
  if (std::fabs(actual - expected) > tol) {
    throw std::runtime_error(msg + " actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
  }
}

static std::string write_csv(const std::string &name, const std::vector<double> &timestamps) {
  std::string path = name + ".csv";
  std::ofstream out(path);
  if (!out.is_open())
    throw std::runtime_error("failed to open temp csv");
  out << "# t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz\n";
  for (double t : timestamps) {
    out << t << ",0,0,0,1," << t << ",0,0,0,0,0,0,0,0,0,0,0\n";
  }
  return path;
}

static std::string write_declared_csv(const std::string &name,
                                      const std::string &position_frame,
                                      const Eigen::Vector3d &position) {
  std::string path = name + ".csv";
  std::ofstream out(path);
  if (!out.is_open())
    throw std::runtime_error("failed to open declared temp csv");
  out << "# position_frame=" << position_frame << "\n";
  out << "# t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz\n";
  out << "10,0,0,0,1,1,2,3," << position.x() << "," << position.y() << ","
      << position.z() << ",0,0,0,0,0,0\n";
  return path;
}

static std::string write_series_csv(const std::string &name,
                                    int samples,
                                    double dt,
                                    double yaw_rate_rad_s,
                                    int outlier_index = -1) {
  std::string path = name + ".csv";
  std::ofstream out(path);
  if (!out.is_open())
    throw std::runtime_error("failed to open series temp csv");
  out << "# position_frame=global_gnav\n";
  out << "# t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz\n";
  for (int index = 0; index < samples; index++) {
    const double t = index * dt;
    double yaw = yaw_rate_rad_s * t;
    if (index == outlier_index)
      yaw += 0.8;
    const double half = 0.5 * yaw;
    out << t << ",0,0," << -std::sin(half) << ',' << std::cos(half)
        << ",10,2,0," << 10.0 * t << ',' << 2.0 * t
        << ",100,0.01,0.02,0.03,0.1,0.2,0.3\n";
  }
  return path;
}

static bool throws_mismatch(const std::string &path, double target, double max_dt) {
  FCInitLoadOptions options;
  options.max_abs_dt = max_dt;
  try {
    (void)load_fc_init_state_csv(path, target, options);
  } catch (const std::runtime_error &e) {
    return std::string(e.what()).find("exceeds max_abs_dt") != std::string::npos;
  }
  return false;
}

int main() {
  try {
    {
      std::string path = write_csv("fc_init_loader_nearest", {904.4, 924.4, 924.45, 925.0});
      FCInitLoadOptions options;
      options.max_abs_dt = 0.1;
      FCInitState state = load_fc_init_state_csv(path, 924.434644, options);
      require_near(state.timestamp, 924.45, 1e-9, "nearest timestamp selection failed");
      require_near(state.source_dt, 0.015356, 1e-9, "nearest timestamp source_dt failed");
      std::remove(path.c_str());
    }
    {
      std::string path = write_csv("fc_init_loader_threshold", {904.4});
      require_true(throws_mismatch(path, 924.434644, 0.25), "large mismatch did not fail");
      FCInitLoadOptions options;
      options.max_abs_dt = 0.25;
      options.warn_only = true;
      FCInitState state = load_fc_init_state_csv(path, 924.434644, options);
      require_near(state.source_dt, -20.034644, 1e-9, "warn-only mismatch source_dt changed");
      std::remove(path.c_str());
    }
    {
      std::string path = write_csv("fc_init_loader_no_hidden_offset", {924.4});
      FCInitLoadOptions options;
      options.max_abs_dt = 0.25;
      FCInitState state = load_fc_init_state_csv(path, 924.4, options);
      require_near(state.timestamp, 924.4, 1e-9, "timestamp was unexpectedly offset");
      require_near(state.source_dt, 0.0, 1e-12, "source_dt should be zero with matching target");
      std::remove(path.c_str());
    }
    {
      std::string path = write_csv("fc_init_loader_time_diagnostics", {925.0, 924.4, 924.4, 924.45});
      FCInitLoadOptions options;
      options.max_abs_dt = 1.0;
      FCInitState state = load_fc_init_state_csv(path, 924.434644, options);
      require_near(state.timestamp, 924.45, 1e-9, "out-of-order nearest selection failed");
      require_true(state.duplicate_timestamp_count == 1, "duplicate timestamp count failed");
      require_true(state.non_monotonic_timestamp_count == 1, "non-monotonic timestamp count failed");
      std::remove(path.c_str());
    }
    {
      FCInitState state;
      auto local = prepare_fc_init_position(state, FCInitPositionFrame::LOCAL_W0_SEED);
      require_true(local.local_seed.p_IinG.isZero(0.0), "zero local seed changed");
      state.p_IinG.x() = 1e-3;
      bool rejected = false;
      try {
        (void)prepare_fc_init_position(state, FCInitPositionFrame::LOCAL_W0_SEED);
      } catch (const std::runtime_error &e) {
        rejected = std::string(e.what()).find("local_w0_seed must be zero") != std::string::npos;
      }
      require_true(rejected, "non-zero local-W0 FC init position did not fail closed");
    }
    {
      FCInitState state;
      state.p_IinG << 124.0, -53.0, 18.5;
      auto global = prepare_fc_init_position(state, FCInitPositionFrame::GLOBAL_GNAV);
      require_true(global.has_global_nav_position, "global position was not retained");
      require_true(global.local_seed.p_IinG.isZero(0.0), "global position leaked into W0 seed");
      require_true((global.p_IinGnav - state.p_IinG).norm() < 1e-12,
                   "global navigation position changed at the boundary");

      const double yaw = 0.37;
      Eigen::Matrix3d R_Gnav_W0;
      R_Gnav_W0 << std::cos(yaw), -std::sin(yaw), 0.0,
                   std::sin(yaw),  std::cos(yaw), 0.0,
                   0.0,            0.0,           1.0;
      const Eigen::Vector3d p_W0inGnav =
          global.p_IinGnav - R_Gnav_W0 * global.local_seed.p_IinG;
      const Eigen::Vector3d recovered =
          R_Gnav_W0 * global.local_seed.p_IinG + p_W0inGnav;
      require_true((recovered - state.p_IinG).norm() < 1e-12,
                   "nav transform did not recover original global FC position");
    }
    {
      require_true(parse_initialization_mode("fc_full_state") ==
                       InitializationMode::FC_FULL_STATE,
                   "fc_full_state mode parse failed");
      require_true(parse_initialization_mode("fc_attitude_only") ==
                       InitializationMode::FC_ATTITUDE_ONLY,
                   "fc_attitude_only mode parse failed");
      require_true(parse_initialization_mode("vio_only") ==
                       InitializationMode::VIO_ONLY,
                   "vio_only mode parse failed");
    }
    {
      require_true(parse_fc_init_position_frame("local_w0_seed") ==
                       FCInitPositionFrame::LOCAL_W0_SEED,
                   "local position-frame parse failed");
      require_true(parse_fc_init_position_frame("global_gnav") ==
                       FCInitPositionFrame::GLOBAL_GNAV,
                   "global position-frame parse failed");
      bool rejected = false;
      try {
        (void)parse_fc_init_position_frame("implicit");
      } catch (const std::runtime_error &) {
        rejected = true;
      }
      require_true(rejected, "unknown position frame did not fail closed");
    }
    {
      std::string path = write_declared_csv(
          "fc_init_loader_declared_global", "global_gnav",
          Eigen::Vector3d(11.0, -4.0, 2.0));
      FCInitState state = load_fc_init_state_csv(path, 10.0);
      require_true(state.declared_position_frame == "global_gnav",
                   "position-frame declaration was not retained");
      auto boundary = prepare_fc_init_position(
          state, parse_fc_init_position_frame(state.declared_position_frame));
      require_true(boundary.local_seed.p_IinG.isZero(0.0),
                   "declared global position leaked into local seed");
      require_true((boundary.p_IinGnav - state.p_IinG).norm() < 1e-12,
                   "declared global position was not retained");
      std::remove(path.c_str());
    }
    {
      std::string path = write_series_csv("fc_init_loader_i1", 3, 1.0,
                                          3.14159265358979323846 / 2.0);
      FCInitExactOptions options;
      options.level = FCInitLevel::I1_EXACT_TIME;
      options.max_bracket_gap_s = 1.1;
      const auto result = load_fc_init_result_csv(path, 0.5, options);
      require_near(result.bracket.alpha, 0.5, 1e-12, "I1 alpha failed");
      require_near(result.bracket.before.state.timestamp, 0.0, 1e-12,
                   "I1 before timestamp failed");
      require_near(result.bracket.after.state.timestamp, 1.0, 1e-12,
                   "I1 after timestamp failed");
      require_near(result.state.p_IinG.x(), 5.0, 1e-9,
                   "I1 position interpolation failed");
      const Eigen::Matrix3d rotation =
          ov_msckf::fcinit_eigen_quaternion(result.state.q_GtoI).toRotationMatrix();
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      require_near(yaw, 3.14159265358979323846 / 4.0, 1e-6,
                   "I1 quaternion SLERP failed");
      require_true(result.bracket.before.source_row_index > 0 &&
                       !result.bracket.before.source_line.empty(),
                   "I1 source-row audit fields missing");
      std::remove(path.c_str());
    }
    {
      std::string path = write_series_csv("fc_init_loader_exact", 3, 1.0, 0.1);
      FCInitExactOptions options;
      options.level = FCInitLevel::I1_EXACT_TIME;
      const auto result = load_fc_init_result_csv(path, 1.0, options);
      require_true(result.bracket.exact_source_row, "exact source row was not detected");
      require_near(result.bracket.gap_s, 0.0, 1e-12, "exact source-row gap failed");
      std::remove(path.c_str());
    }
    {
      std::string path = write_series_csv("fc_init_loader_gap", 3, 1.0, 0.1);
      FCInitExactOptions options;
      options.level = FCInitLevel::I1_EXACT_TIME;
      options.max_bracket_gap_s = 0.25;
      bool rejected = false;
      try {
        (void)load_fc_init_result_csv(path, 0.5, options);
      } catch (const std::runtime_error &e) {
        rejected = std::string(e.what()).find("bracket gap") != std::string::npos;
      }
      require_true(rejected, "excessive I1 bracket gap did not fail closed");
      std::remove(path.c_str());
    }
    {
      std::string path = write_csv("fc_init_loader_duplicate_exact", {0.0, 1.0, 1.0, 2.0});
      FCInitExactOptions options;
      options.level = FCInitLevel::I1_EXACT_TIME;
      bool rejected = false;
      try {
        (void)load_fc_init_result_csv(path, 0.5, options);
      } catch (const std::runtime_error &e) {
        rejected = std::string(e.what()).find("duplicate") != std::string::npos;
      }
      require_true(rejected, "duplicate timestamps were accepted by exact-time init");
      std::remove(path.c_str());
    }
    {
      const double rate = 0.12;
      std::string path = write_series_csv("fc_init_loader_i2", 81, 0.1, rate, 37);
      FCInitExactOptions options;
      options.level = FCInitLevel::I2_ROBUST_SO3;
      options.max_bracket_gap_s = 0.2;
      options.window_duration_s = 5.0;
      options.min_window_samples = 30;
      const auto result = load_fc_init_result_csv(path, 4.05, options);
      const Eigen::Matrix3d rotation =
          ov_msckf::fcinit_eigen_quaternion(result.state.q_GtoI).toRotationMatrix();
      const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
      require_near(yaw, rate * 4.05, 0.01,
                   "I2 robust SO3 seed did not reject the attitude outlier");
      require_near(result.window.fitted_angular_rate_rad_s.z(), rate, 0.01,
                   "I2 fitted angular rate failed");
      require_true(result.window.sample_count >= 30,
                   "I2 robust window sample count missing");
      require_true(result.window.first_timestamp_s < 5.0 &&
                       result.window.last_timestamp_s > 5.0,
                   "I2 symmetric window did not disclose future data use");
      std::remove(path.c_str());
    }
    {
      std::string path = write_series_csv("fc_init_loader_i3_fallback", 11, 0.5, 0.1);
      FCInitExactOptions options;
      options.level = FCInitLevel::I3_QUALITY_GATED;
      options.fallback = FCInitFallback::I1_EXACT_TIME;
      options.max_bracket_gap_s = 0.6;
      options.window_duration_s = 5.0;
      options.min_window_samples = 20;
      const auto result = load_fc_init_result_csv(path, 2.25, options);
      require_true(result.fallback_applied, "I3 fallback was not recorded");
      require_true(result.applied_level == FCInitLevel::I1_EXACT_TIME,
                   "I3 fallback did not apply I1");
      require_true(!result.window.quality_reason.empty(),
                   "I3 fallback reason was not retained");
      std::remove(path.c_str());
    }
  } catch (const std::exception &e) {
    std::cerr << "test_fc_init_loader failed: " << e.what() << std::endl;
    return 1;
  }
  std::cout << "test_fc_init_loader passed" << std::endl;
  return 0;
}
