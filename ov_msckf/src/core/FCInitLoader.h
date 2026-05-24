/*
 * Helper for loading a one-shot FC-assisted VIO initialization state.
 */

#ifndef OV_MSCKF_FC_INIT_LOADER_H
#define OV_MSCKF_FC_INIT_LOADER_H

#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
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
};

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

inline FCInitState load_fc_init_state_csv(const std::string &path, double target_time) {
  std::ifstream in(path);
  if (!in.is_open())
    throw std::runtime_error("cannot open FC init CSV: " + path);

  bool have = false;
  FCInitState best;
  double best_abs_dt = std::numeric_limits<double>::infinity();
  std::string line;
  while (std::getline(in, line)) {
    FCInitState candidate;
    if (!fcinit_parse_row(line, candidate))
      continue;
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
  return best;
}

} // namespace ov_msckf

#endif // OV_MSCKF_FC_INIT_LOADER_H
