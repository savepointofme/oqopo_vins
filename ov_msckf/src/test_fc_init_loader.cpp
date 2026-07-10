#include "core/FCInitLoader.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using ov_msckf::FCInitLoadOptions;
using ov_msckf::FCInitState;
using ov_msckf::load_fc_init_state_csv;

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
  } catch (const std::exception &e) {
    std::cerr << "test_fc_init_loader failed: " << e.what() << std::endl;
    return 1;
  }
  std::cout << "test_fc_init_loader passed" << std::endl;
  return 0;
}
