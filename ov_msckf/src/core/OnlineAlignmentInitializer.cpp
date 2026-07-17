#include "OnlineAlignmentInitializer.h"

#include "ceres/Factor_GenericPrior.h"
#include "ceres/Factor_ImageReprojCalib.h"
#include "ceres/Factor_ImuCPIv1.h"
#include "ceres/State_JPLQuatLocal.h"
#include "core/p4/factors/Factor_P4Epipolar.h"
#include "core/p4/factors/Factor_P4FcTrajectory.h"
#include "core/p4/factors/Factor_P4ImuSharedBias.h"
#include "cpi/CpiV1.h"
#include "utils/quat_ops.h"

#include <ceres/ceres.h>
#include <ceres/covariance.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <sstream>
#include <unordered_map>

namespace ov_msckf {
namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d so3_log(const Eigen::Matrix3d &R) {
  Eigen::AngleAxisd aa(R);
  if (!std::isfinite(aa.angle()) || aa.angle() < 1e-12)
    return Eigen::Vector3d::Zero();
  return aa.axis() * aa.angle();
}

double rotation_angle(const Eigen::Matrix3d &R) { return so3_log(R).norm(); }

double wrap_radians(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_R_GtoI(const Eigen::Matrix3d &R_GtoI) {
  const Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
  return std::atan2(R_ItoG(1, 0), R_ItoG(0, 0));
}

Eigen::Quaterniond eigen_quaternion_from_jpl(const Eigen::Vector4d &q) {
  Eigen::Quaterniond out(q(3), -q(0), -q(1), -q(2));
  return out.normalized();
}

Eigen::Vector4d normalized_jpl(const double *q_ptr) {
  Eigen::Vector4d q = Eigen::Map<const Eigen::Vector4d>(q_ptr);
  const double norm = q.norm();
  if (norm > 1e-12)
    q /= norm;
  else
    q << 0.0, 0.0, 0.0, 1.0;
  return q;
}

double percentile(std::vector<double> values, double p) {
  if (values.empty())
    return std::numeric_limits<double>::quiet_NaN();
  std::sort(values.begin(), values.end());
  const double index = std::max(0.0, std::min(1.0, p)) * (values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(index));
  const size_t hi = static_cast<size_t>(std::ceil(index));
  return values[lo] + (index - lo) * (values[hi] - values[lo]);
}

double median(std::vector<double> values) { return percentile(std::move(values), 0.5); }

Eigen::Vector3d component_median(const std::vector<Eigen::Vector3d> &values) {
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  if (values.empty())
    return out;
  for (int axis = 0; axis < 3; ++axis) {
    std::vector<double> component;
    component.reserve(values.size());
    for (const auto &value : values)
      component.push_back(value(axis));
    out(axis) = median(std::move(component));
  }
  return out;
}

template <typename Sample>
double max_gap(const std::vector<const Sample *> &samples) {
  if (samples.size() < 2)
    return std::numeric_limits<double>::infinity();
  double gap = 0.0;
  for (size_t i = 1; i < samples.size(); ++i)
    gap = std::max(gap, samples[i]->timestamp - samples[i - 1]->timestamp);
  return gap;
}

double stereo_max_gap(const std::vector<const StereoAlignmentFrame *> &frames) {
  if (frames.size() < 2)
    return std::numeric_limits<double>::infinity();
  double gap = 0.0;
  for (size_t i = 1; i < frames.size(); ++i)
    gap = std::max(gap, frames[i]->left_timestamp - frames[i - 1]->left_timestamp);
  return gap;
}

bool interpolate_imu(const std::deque<BoardImuSample> &samples, double timestamp,
                     BoardImuSample &output) {
  if (samples.size() < 2 || timestamp < samples.front().timestamp ||
      timestamp > samples.back().timestamp)
    return false;
  auto after = std::lower_bound(
      samples.begin(), samples.end(), timestamp,
      [](const BoardImuSample &sample, double t) { return sample.timestamp < t; });
  if (after == samples.end())
    return false;
  if (std::fabs(after->timestamp - timestamp) < 1e-10) {
    output = *after;
    return output.status_valid && !output.gyro_saturated &&
           !output.accel_saturated;
  }
  if (after == samples.begin())
    return false;
  const auto before = std::prev(after);
  if (!before->status_valid || !after->status_valid || before->gyro_saturated ||
      after->gyro_saturated || before->accel_saturated || after->accel_saturated)
    return false;
  const double gap = after->timestamp - before->timestamp;
  if (!(gap > 0.0))
    return false;
  const double alpha = (timestamp - before->timestamp) / gap;
  output = *before;
  output.timestamp = timestamp;
  output.angular_velocity = (1.0 - alpha) * before->angular_velocity +
                            alpha * after->angular_velocity;
  output.linear_acceleration = (1.0 - alpha) * before->linear_acceleration +
                               alpha * after->linear_acceleration;
  output.gyro_saturated = false;
  output.accel_saturated = false;
  return output.angular_velocity.allFinite() &&
         output.linear_acceleration.allFinite();
}

bool interpolate_fc(const std::deque<FCNavigationSample> &samples,
                    double timestamp, FCNavigationSample &state,
                    double *bracket_gap = nullptr) {
  if (samples.size() < 2 || timestamp < samples.front().timestamp ||
      timestamp > samples.back().timestamp)
    return false;
  auto after = std::lower_bound(
      samples.begin(), samples.end(), timestamp,
      [](const FCNavigationSample &sample, double t) {
        return sample.timestamp < t;
      });
  if (after == samples.end())
    return false;
  if (std::fabs(after->timestamp - timestamp) < 1e-10) {
    state = *after;
    if (bracket_gap)
      *bracket_gap = 0.0;
    return state.position_valid && state.velocity_valid &&
           state.attitude_valid && state.status_valid;
  }
  if (after == samples.begin())
    return false;
  const auto before = std::prev(after);
  const double gap = after->timestamp - before->timestamp;
  if (!(gap > 0.0))
    return false;
  const double alpha = (timestamp - before->timestamp) / gap;
  state = *before;
  state.timestamp = timestamp;
  state.position_G = (1.0 - alpha) * before->position_G +
                     alpha * after->position_G;
  state.velocity_G = (1.0 - alpha) * before->velocity_G +
                     alpha * after->velocity_G;
  const Eigen::Quaterniond q0 = eigen_quaternion_from_jpl(before->q_GtoF);
  const Eigen::Quaterniond q1 = eigen_quaternion_from_jpl(after->q_GtoF);
  const Eigen::Quaterniond qi = q0.slerp(alpha, q1).normalized();
  state.q_GtoF << -qi.x(), -qi.y(), -qi.z(), qi.w();
  state.position_valid = before->position_valid && after->position_valid;
  state.velocity_valid = before->velocity_valid && after->velocity_valid;
  state.attitude_valid = before->attitude_valid && after->attitude_valid;
  state.status_valid = before->status_valid && after->status_valid;
  if (bracket_gap)
    *bracket_gap = gap;
  return state.position_valid && state.velocity_valid &&
         state.attitude_valid && state.status_valid;
}

struct FCRateSample {
  double timestamp = -1.0;
  Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_G = Eigen::Vector3d::Zero();
};

std::vector<FCRateSample> make_fc_rates(
    const std::vector<const FCNavigationSample *> &fc) {
  std::vector<FCRateSample> out;
  for (size_t i = 1; i < fc.size(); ++i) {
    const double dt = fc[i]->timestamp - fc[i - 1]->timestamp;
    if (!(dt > 1e-4) || !fc[i - 1]->attitude_valid ||
        !fc[i]->attitude_valid || !fc[i - 1]->velocity_valid ||
        !fc[i]->velocity_valid)
      continue;
    const Eigen::Matrix3d R0 = ov_core::quat_2_Rot(fc[i - 1]->q_GtoF);
    const Eigen::Matrix3d R1 = ov_core::quat_2_Rot(fc[i]->q_GtoF);
    FCRateSample rate;
    rate.timestamp = 0.5 * (fc[i]->timestamp + fc[i - 1]->timestamp);
    rate.omega_F = -so3_log(R1 * R0.transpose()) / dt;
    rate.acceleration_G =
        (fc[i]->velocity_G - fc[i - 1]->velocity_G) / dt;
    if (rate.omega_F.allFinite() && rate.acceleration_G.allFinite())
      out.push_back(rate);
  }
  return out;
}

struct RotationFit {
  bool valid = false;
  Eigen::Matrix3d R_FtoI = Eigen::Matrix3d::Identity();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  double rms = std::numeric_limits<double>::infinity();
  double excitation = 0.0;
  double second_axis_ratio = 0.0;
};

RotationFit evaluate_locked_rotation(
    const std::vector<Eigen::Vector3d> &fc_rates,
    const std::vector<Eigen::Vector3d> &imu_rates,
    const Eigen::Matrix3d &R_FtoI_locked) {
  RotationFit fit;
  if (fc_rates.size() < 6 || fc_rates.size() != imu_rates.size() ||
      !R_FtoI_locked.allFinite())
    return fit;
  fit.R_FtoI = R_FtoI_locked;
  std::vector<Eigen::Vector3d> raw_residuals;
  raw_residuals.reserve(fc_rates.size());
  Eigen::Vector3d mean_f = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < fc_rates.size(); ++i) {
    mean_f += fc_rates[i];
    raw_residuals.push_back(imu_rates[i] - R_FtoI_locked * fc_rates[i]);
  }
  mean_f /= static_cast<double>(fc_rates.size());
  fit.bg = component_median(raw_residuals);
  double sse = 0.0;
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < fc_rates.size(); ++i) {
    sse += (raw_residuals[i] - fit.bg).squaredNorm();
    const Eigen::Vector3d centered = fc_rates[i] - mean_f;
    covariance += centered * centered.transpose();
  }
  fit.rms = std::sqrt(sse / static_cast<double>(raw_residuals.size()));
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
      covariance / static_cast<double>(fc_rates.size()));
  if (eig.info() != Eigen::Success)
    return fit;
  const Eigen::Vector3d values =
      eig.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  fit.excitation = values(2);
  fit.second_axis_ratio =
      values(2) > 1e-12 ? values(1) / values(2) : 0.0;
  fit.valid = std::isfinite(fit.rms);
  return fit;
}

bool interval_imu_samples(const std::deque<BoardImuSample> &buffer, double t0,
                          double t1, std::vector<BoardImuSample> &samples) {
  samples.clear();
  BoardImuSample first, last;
  if (!(t1 > t0) || !interpolate_imu(buffer, t0, first) ||
      !interpolate_imu(buffer, t1, last))
    return false;
  samples.push_back(first);
  for (const auto &sample : buffer)
    if (sample.timestamp > t0 && sample.timestamp < t1)
      samples.push_back(sample);
  samples.push_back(last);
  return samples.size() >= 2;
}

bool fc_angular_rate_at(const std::deque<FCNavigationSample> &buffer,
                        double timestamp, double maximum_gap_s,
                        Eigen::Vector3d &omega_F) {
  if (buffer.size() < 2 || timestamp < buffer.front().timestamp - 1.0e-9 ||
      timestamp > buffer.back().timestamp + 1.0e-9)
    return false;
  auto after = std::lower_bound(
      buffer.begin(), buffer.end(), timestamp,
      [](const FCNavigationSample &sample, double time) {
        return sample.timestamp < time;
      });
  auto before = buffer.end();
  if (after == buffer.end()) {
    after = std::prev(buffer.end());
    before = std::prev(after);
  } else if (after == buffer.begin()) {
    before = after;
    ++after;
  } else if (std::fabs(after->timestamp - timestamp) < 1.0e-10 &&
             std::next(after) != buffer.end()) {
    before = std::prev(after);
    after = std::next(after);
  } else {
    before = std::prev(after);
  }
  const double dt = after->timestamp - before->timestamp;
  if (!(dt > 1.0e-6) || dt > maximum_gap_s + 1.0e-9 ||
      !before->attitude_valid || !after->attitude_valid)
    return false;
  const Eigen::Matrix3d R0 = ov_core::quat_2_Rot(before->q_GtoF);
  const Eigen::Matrix3d R1 = ov_core::quat_2_Rot(after->q_GtoF);
  omega_F = -so3_log(R1 * R0.transpose()) / dt;
  return omega_F.allFinite();
}

struct GraphState {
  double camera_time = -1.0;
  std::array<double, 4> q{};
  std::array<double, 3> p{};
  std::array<double, 3> v{};
  std::array<double, 3> bg{};
  std::array<double, 3> ba{};
};

void assign(std::array<double, 4> &dst, const Eigen::Vector4d &src) {
  for (int i = 0; i < 4; ++i)
    dst[i] = src(i);
}

void assign(std::array<double, 3> &dst, const Eigen::Vector3d &src) {
  for (int i = 0; i < 3; ++i)
    dst[i] = src(i);
}

Eigen::Vector3d map3(const std::array<double, 3> &value) {
  return Eigen::Map<const Eigen::Vector3d>(value.data());
}

class FCObservationFunctor {
public:
  FCObservationFunctor(const Eigen::Vector3d &p_IinG,
                       const Eigen::Vector3d &v_IinG,
                       double position_sigma, double velocity_sigma)
      : p_IinG_(p_IinG), v_IinG_(v_IinG), position_sigma_(position_sigma),
        velocity_sigma_(velocity_sigma) {}

  // This factor contributes FC position/velocity only. Terminal attitude and
  // relative attitude increments are represented by separate factors.
  bool operator()(const double *p_ptr, const double *v_ptr,
                  double *residuals) const {
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.segment<3>(0) =
        (Eigen::Map<const Eigen::Vector3d>(p_ptr) - p_IinG_) /
        std::max(1e-8, position_sigma_);
    residual.segment<3>(3) =
        (Eigen::Map<const Eigen::Vector3d>(v_ptr) - v_IinG_) /
        std::max(1e-8, velocity_sigma_);
    return residual.allFinite();
  }

private:
  Eigen::Vector3d p_IinG_;
  Eigen::Vector3d v_IinG_;
  double position_sigma_;
  double velocity_sigma_;
};

class FCNavigationIncrementFunctor {
public:
  FCNavigationIncrementFunctor(const Eigen::Vector3d &delta_position_G,
                               const Eigen::Vector3d &delta_velocity_G,
                               double position_sigma,
                               double velocity_sigma)
      : delta_position_G_(delta_position_G),
        delta_velocity_G_(delta_velocity_G),
        position_sigma_(position_sigma), velocity_sigma_(velocity_sigma) {}

  bool operator()(const double *p0_ptr, const double *v0_ptr,
                  const double *p1_ptr, const double *v1_ptr,
                  double *residuals) const {
    const Eigen::Map<const Eigen::Vector3d> p0(p0_ptr);
    const Eigen::Map<const Eigen::Vector3d> v0(v0_ptr);
    const Eigen::Map<const Eigen::Vector3d> p1(p1_ptr);
    const Eigen::Map<const Eigen::Vector3d> v1(v1_ptr);
    Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
    residual.segment<3>(0) =
        ((p1 - p0) - delta_position_G_) /
        std::max(1e-8, position_sigma_);
    residual.segment<3>(3) =
        ((v1 - v0) - delta_velocity_G_) /
        std::max(1e-8, velocity_sigma_);
    return residual.allFinite();
  }

private:
  Eigen::Vector3d delta_position_G_;
  Eigen::Vector3d delta_velocity_G_;
  double position_sigma_;
  double velocity_sigma_;
};

class FCAttitudeIncrementFunctor {
public:
  FCAttitudeIncrementFunctor(const Eigen::Matrix3d &R_target_previous,
                             const Eigen::Matrix3d &R_target_current,
                             double sigma_rad)
      : R_target_relative_(R_target_current *
                           R_target_previous.transpose()),
        sigma_rad_(sigma_rad) {}

  bool operator()(const double *q_previous_ptr, const double *q_current_ptr,
                  double *residuals) const {
    const Eigen::Matrix3d R_previous =
        ov_core::quat_2_Rot(normalized_jpl(q_previous_ptr));
    const Eigen::Matrix3d R_current =
        ov_core::quat_2_Rot(normalized_jpl(q_current_ptr));
    const Eigen::Matrix3d R_relative = R_current * R_previous.transpose();
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = so3_log(R_relative * R_target_relative_.transpose()) /
               std::max(1e-8, sigma_rad_);
    return residual.allFinite();
  }

private:
  Eigen::Matrix3d R_target_relative_;
  double sigma_rad_;
};

class QuaternionPriorFunctor {
public:
  QuaternionPriorFunctor(const Eigen::Matrix3d &R_prior, double sigma_rad)
      : R_prior_(R_prior), sigma_(sigma_rad) {}
  bool operator()(const double *q_ptr, double *residuals) const {
    const Eigen::Matrix3d R = ov_core::quat_2_Rot(normalized_jpl(q_ptr));
    Eigen::Vector4d q_error = ov_core::rot_2_quat(R * R_prior_.transpose());
    if (q_error(3) < 0.0)
      q_error = -q_error;
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = 2.0 * q_error.head<3>() / std::max(1e-8, sigma_);
    return residual.allFinite();
  }

private:
  Eigen::Matrix3d R_prior_;
  double sigma_;
};

class VectorPriorFunctor {
public:
  VectorPriorFunctor(const Eigen::Vector3d &prior, double sigma)
      : prior_(prior), sigma_(sigma) {}
  bool operator()(const double *value_ptr, double *residuals) const {
    Eigen::Map<Eigen::Vector3d> residual(residuals);
    residual = (Eigen::Map<const Eigen::Vector3d>(value_ptr) - prior_) /
               std::max(1e-8, sigma_);
    return residual.allFinite();
  }

private:
  Eigen::Vector3d prior_;
  double sigma_;
};

using FCObservationCost =
    ceres::NumericDiffCostFunction<FCObservationFunctor, ceres::CENTRAL, 6, 3,
                                   3>;
using FCNavigationIncrementCost =
    ceres::NumericDiffCostFunction<FCNavigationIncrementFunctor,
                                   ceres::CENTRAL, 6, 3, 3, 3, 3>;
using FCAttitudeIncrementCost =
    ceres::NumericDiffCostFunction<FCAttitudeIncrementFunctor,
                                   ceres::CENTRAL, 3, 4, 4>;
using QuaternionPriorCost =
    ceres::NumericDiffCostFunction<QuaternionPriorFunctor, ceres::CENTRAL, 3,
                                   4>;
using VectorPriorCost =
    ceres::NumericDiffCostFunction<VectorPriorFunctor, ceres::CENTRAL, 3, 3>;
struct FactorFamilies {
  std::vector<ceres::ResidualBlockId> prior;
  std::vector<ceres::ResidualBlockId> imu;
  std::vector<ceres::ResidualBlockId> visual;
  std::vector<ceres::ResidualBlockId> fc;
};

FactorContribution evaluate_family(
    ceres::Problem &problem, const std::string &name,
    const std::vector<ceres::ResidualBlockId> &blocks,
    const std::vector<double *> &tracked_blocks,
    const std::vector<std::string> &tracked_names,
    bool apply_loss_function = true) {
  FactorContribution contribution;
  contribution.family = name;
  contribution.residual_blocks = static_cast<int>(blocks.size());
  if (blocks.empty())
    return contribution;
  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = apply_loss_function;
  options.residual_blocks = blocks;
  options.parameter_blocks = tracked_blocks;
  double cost = 0.0;
  std::vector<double> residuals;
  ceres::CRSMatrix jacobian;
  if (!problem.Evaluate(options, &cost, &residuals, nullptr, &jacobian))
    return contribution;
  contribution.residual_dimension = static_cast<int>(residuals.size());
  double squared = 0.0;
  double maximum = 0.0;
  for (double residual : residuals) {
    squared += residual * residual;
    maximum = std::max(maximum, std::fabs(residual));
  }
  contribution.residual_rms = residuals.empty()
                                  ? std::numeric_limits<double>::infinity()
                                  : std::sqrt(squared / residuals.size());
  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(residuals.size());
  for (double residual : residuals)
    absolute_residuals.push_back(std::fabs(residual));
  contribution.residual_p95 = percentile(std::move(absolute_residuals), 0.95);
  contribution.residual_max_abs = maximum;
  std::vector<double> column_squares(jacobian.num_cols, 0.0);
  double jacobian_squared = 0.0;
  for (size_t i = 0; i < jacobian.values.size(); ++i) {
    const double value = jacobian.values[i];
    const int col = jacobian.cols[i];
    jacobian_squared += value * value;
    if (col >= 0 && col < static_cast<int>(column_squares.size()))
      column_squares[col] += value * value;
  }
  contribution.jacobian_frobenius = std::sqrt(jacobian_squared);
  int offset = 0;
  for (size_t i = 0; i < tracked_names.size(); ++i) {
    double block_squared = 0.0;
    for (int axis = 0; axis < 3 && offset + axis < jacobian.num_cols; ++axis)
      block_squared += column_squares[offset + axis];
    contribution.state_jacobian_frobenius[tracked_names[i]] =
        std::sqrt(block_squared);
    offset += 3;
  }
  // The final state may not directly observe a feature that was tracked in
  // earlier keyframes. Report the whole factor-family Jacobian as the global
  // contribution; per-state direct blocks above remain available separately.
  ceres::Problem::EvaluateOptions global_options;
  global_options.apply_loss_function = apply_loss_function;
  global_options.residual_blocks = blocks;
  ceres::CRSMatrix global_jacobian;
  if (problem.Evaluate(global_options, nullptr, nullptr, nullptr,
                       &global_jacobian)) {
    double global_squared = 0.0;
    for (double value : global_jacobian.values)
      global_squared += value * value;
    contribution.jacobian_frobenius = std::sqrt(global_squared);
  }
  return contribution;
}

FactorContribution evaluate_family_residuals(
    ceres::Problem &problem, const std::string &name,
    const std::vector<ceres::ResidualBlockId> &blocks,
    bool apply_loss_function = true) {
  FactorContribution contribution;
  contribution.family = name;
  contribution.residual_blocks = static_cast<int>(blocks.size());
  if (blocks.empty())
    return contribution;
  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = apply_loss_function;
  options.residual_blocks = blocks;
  double cost = 0.0;
  std::vector<double> residuals;
  if (!problem.Evaluate(options, &cost, &residuals, nullptr, nullptr))
    return contribution;
  contribution.residual_dimension = static_cast<int>(residuals.size());
  double squared = 0.0;
  double maximum = 0.0;
  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(residuals.size());
  for (double residual : residuals) {
    squared += residual * residual;
    maximum = std::max(maximum, std::fabs(residual));
    absolute_residuals.push_back(std::fabs(residual));
  }
  contribution.residual_rms = residuals.empty()
                                  ? std::numeric_limits<double>::infinity()
                                  : std::sqrt(squared / residuals.size());
  contribution.residual_p95 = percentile(std::move(absolute_residuals), 0.95);
  contribution.residual_max_abs = maximum;
  return contribution;
}

const FactorContribution *find_contribution(
    const std::vector<FactorContribution> &contributions,
    const std::string &family) {
  for (const auto &contribution : contributions)
    if (contribution.family == family)
      return &contribution;
  return nullptr;
}

double state_jacobian(const FactorContribution *contribution,
                      const std::string &state) {
  if (contribution == nullptr)
    return 0.0;
  const auto found = contribution->state_jacobian_frobenius.find(state);
  return found == contribution->state_jacobian_frobenius.end()
             ? 0.0
             : found->second;
}

Eigen::MatrixXd dense_jacobian(const ceres::CRSMatrix &crs) {
  Eigen::MatrixXd dense = Eigen::MatrixXd::Zero(crs.num_rows, crs.num_cols);
  for (int row = 0; row < crs.num_rows; ++row)
    for (int index = crs.rows[row]; index < crs.rows[row + 1]; ++index)
      dense(row, crs.cols[index]) = crs.values[index];
  return dense;
}

Eigen::MatrixXd schur_information(
    ceres::Problem &problem,
    const std::vector<ceres::ResidualBlockId> &residual_blocks,
    const std::vector<double *> &parameter_blocks, int target_dimension) {
  if (residual_blocks.empty() || parameter_blocks.empty() ||
      target_dimension <= 0)
    return Eigen::MatrixXd::Zero(std::max(0, target_dimension),
                                 std::max(0, target_dimension));
  ceres::Problem::EvaluateOptions options;
  options.apply_loss_function = true;
  options.residual_blocks = residual_blocks;
  options.parameter_blocks = parameter_blocks;
  double cost = 0.0;
  std::vector<double> residuals;
  ceres::CRSMatrix jacobian;
  if (!problem.Evaluate(options, &cost, &residuals, nullptr, &jacobian) ||
      jacobian.num_cols < target_dimension)
    return Eigen::MatrixXd::Zero(target_dimension, target_dimension);
  const Eigen::MatrixXd J = dense_jacobian(jacobian);
  const Eigen::MatrixXd H = J.transpose() * J;
  Eigen::MatrixXd result = H.topLeftCorner(target_dimension, target_dimension);
  const int nuisance_dimension = H.rows() - target_dimension;
  if (nuisance_dimension > 0) {
    const Eigen::MatrixXd H_ab =
        H.block(0, target_dimension, target_dimension, nuisance_dimension);
    const Eigen::MatrixXd H_bb = H.block(
        target_dimension, target_dimension, nuisance_dimension,
        nuisance_dimension);
    result -= H_ab * H_bb.completeOrthogonalDecomposition().solve(
                           H_ab.transpose());
  }
  return 0.5 * (result + result.transpose());
}

std::string join(const std::vector<std::string> &parts) {
  std::ostringstream output;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i)
      output << ';';
    output << parts[i];
  }
  return output.str();
}

} // namespace

const char *alignment_phase_name(AlignmentPhase phase) {
  switch (phase) {
  case AlignmentPhase::WAIT_INPUTS: return "WAIT_INPUTS";
  case AlignmentPhase::COLLECTING: return "COLLECTING";
  case AlignmentPhase::ALIGNING: return "ALIGNING";
  case AlignmentPhase::VALIDATING: return "VALIDATING";
  case AlignmentPhase::CANDIDATE_VALIDATING: return "CANDIDATE_VALIDATING";
  case AlignmentPhase::CANDIDATE_REFINING: return "CANDIDATE_REFINING";
  case AlignmentPhase::NAVIGATION_READY: return "NAVIGATION_READY";
  case AlignmentPhase::FULL_ALIGNMENT_READY: return "FULL_ALIGNMENT_READY";
  case AlignmentPhase::FAILED_WAIT_RETRY: return "FAILED_WAIT_RETRY";
  case AlignmentPhase::CLOSED_AFTER_RELEASE: return "CLOSED_AFTER_RELEASE";
  case AlignmentPhase::FATAL_CONFIGURATION_ERROR:
    return "FATAL_CONFIGURATION_ERROR";
  }
  return "UNKNOWN";
}

const char *alignment_readiness_name(AlignmentReadiness readiness) {
  switch (readiness) {
  case AlignmentReadiness::NOT_READY: return "NOT_READY";
  case AlignmentReadiness::NAVIGATION_READY: return "NAVIGATION_READY";
  case AlignmentReadiness::FULL_ALIGNMENT_READY:
    return "FULL_ALIGNMENT_READY";
  }
  return "UNKNOWN";
}

const char *alignment_release_policy_name(AlignmentReleasePolicy policy) {
  switch (policy) {
  case AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START:
    return "practical_navigation_start";
  case AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT:
    return "strict_full_alignment";
  }
  return "unknown";
}

const char *estimate_source_status_name(EstimateSourceStatus status) {
  switch (status) {
  case EstimateSourceStatus::ESTIMATED_CURRENT_DATA:
    return "estimated_current_data";
  case EstimateSourceStatus::WEAKLY_OBSERVABLE:
    return "weakly_observable";
  case EstimateSourceStatus::FIXED_TO_PRIOR: return "fixed_to_prior";
  case EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION:
    return "fixed_external_calibration";
  case EstimateSourceStatus::UNOBSERVABLE: return "unobservable";
  }
  return "unknown";
}

OnlineAlignmentInitializer::OnlineAlignmentInitializer(
    const OnlineAlignmentOptions &options)
    : options_(options) {
  if (options_.upstream_dynamic_init_fc_gauge) {
    options_.sliding_window_duration_s = std::max(
        0.5, std::min(12.0, options_.sliding_window_duration_s));
    options_.sliding_window_shadow_only = false;
    options_.sliding_window_direct_state_release = false;
    options_.sliding_window_deferred_release_certification = false;
    options_.sliding_window_terminal_state_covariance_only = false;
    options_.reference_window_duration_s =
        options_.sliding_window_duration_s;
    options_.candidate_window_durations_s = {
        options_.sliding_window_duration_s};
  } else if (options_.sliding_window_shadow_only) {
    options_.sliding_window_duration_s = std::max(
        0.5, std::min(12.0, options_.sliding_window_duration_s));
    options_.sliding_window_min_advance_s =
        std::max(1.0e-3, options_.sliding_window_min_advance_s);
    options_.reference_window_duration_s =
        options_.sliding_window_duration_s;
    options_.candidate_window_durations_s = {
        options_.sliding_window_duration_s};
  } else {
    options_.reference_window_duration_s =
        std::max(0.5, std::min(12.0, options_.reference_window_duration_s));
    for (double &duration : options_.candidate_window_durations_s)
      duration = std::max(0.5, std::min(12.0, duration));
    options_.candidate_window_durations_s.push_back(
        options_.reference_window_duration_s);
    std::sort(options_.candidate_window_durations_s.begin(),
              options_.candidate_window_durations_s.end());
    options_.candidate_window_durations_s.erase(
        std::unique(options_.candidate_window_durations_s.begin(),
                    options_.candidate_window_durations_s.end()),
        options_.candidate_window_durations_s.end());
  }
  maximum_window_duration_s_ = options_.candidate_window_durations_s.back();
  options_.window_duration_s = maximum_window_duration_s_;
  options_.sliding_window_min_advance_s =
      std::max(1.0e-3, options_.sliding_window_min_advance_s);
  options_.sliding_window_gauge_stability_duration_s =
      std::max(options_.sliding_window_min_advance_s,
               options_.sliding_window_gauge_stability_duration_s);
  options_.sliding_window_overlap_stability_duration_s =
      std::max(options_.sliding_window_min_advance_s,
               options_.sliding_window_overlap_stability_duration_s);
  options_.sliding_window_min_overlap_states =
      std::max(2, options_.sliding_window_min_overlap_states);
  options_.sliding_window_overlap_max_normalized_sigma =
      std::max(1.0, options_.sliding_window_overlap_max_normalized_sigma);
  options_.sliding_window_bias_trend_max_normalized_sigma =
      std::max(0.0,
               options_.sliding_window_bias_trend_max_normalized_sigma);
  if (options_.sliding_window_direct_state_release &&
      !options_.sliding_window_shadow_only) {
    fatal_configuration_error_ = true;
    last_rejection_ =
        "direct_state_release_requires_repeated_sliding_windows";
    phase_ = AlignmentPhase::FATAL_CONFIGURATION_ERROR;
  }
  options_.candidate_short_validation_duration_s = std::max(
      options_.sliding_window_min_advance_s,
      options_.candidate_short_validation_duration_s);
  options_.min_keyframes = std::max(3, options_.min_keyframes);
  options_.max_keyframes =
      std::max(options_.min_keyframes, options_.max_keyframes);
  options_.max_initial_clones = std::max(0, options_.max_initial_clones);
  if (options_.sliding_window_terminal_state_covariance_only) {
    options_.max_initial_clones = 0;
    options_.max_initial_slam_features = 0;
  }
  const bool calibration_valid =
      options_.fc_board_calibration_locked &&
      options_.R_FtoI_declared.allFinite() &&
      std::fabs(options_.R_FtoI_declared.determinant() - 1.0) <= 1e-6 &&
      (options_.R_FtoI_declared.transpose() * options_.R_FtoI_declared -
       Eigen::Matrix3d::Identity()).norm() <= 1e-6 &&
      std::isfinite(options_.fc_attitude_to_board_time_offset_s) &&
      std::isfinite(options_.fc_navigation_to_board_time_offset_s);
  if (!calibration_valid) {
    fatal_configuration_error_ = true;
    last_rejection_ = "locked_full_flight_fc_board_calibration_required";
    phase_ = AlignmentPhase::FATAL_CONFIGURATION_ERROR;
  }
  if (options_.formal_causal_lifecycle &&
      (!(options_.fc_process_variance_fraction > 0.0) ||
       !(options_.fc_process_variance_fraction < 1.0))) {
    fatal_configuration_error_ = true;
    last_rejection_ =
        "formal_fc_process_variance_fraction_must_be_between_zero_and_one";
    phase_ = AlignmentPhase::FATAL_CONFIGURATION_ERROR;
  }
  if (options_.formal_causal_lifecycle &&
      (options_.upstream_dynamic_init_fc_gauge ||
       options_.sliding_window_shadow_only ||
       options_.sliding_window_direct_state_release ||
       options_.sliding_window_deferred_release_certification ||
       options_.diagnostic_release_attitude_from_fc ||
       options_.diagnostic_release_graph_attitude_fc_pv_zero_bias ||
       !options_.candidate_refinement_enabled ||
       options_.max_initial_clones != 0 ||
       options_.max_initial_slam_features != 0)) {
    fatal_configuration_error_ = true;
    last_rejection_ =
        "formal_lifecycle_must_be_terminal_only_and_mutually_exclusive";
    phase_ = AlignmentPhase::FATAL_CONFIGURATION_ERROR;
  }
  options_.max_time_offset_s = std::max(
      std::fabs(options_.fc_attitude_to_board_time_offset_s),
      std::fabs(options_.fc_navigation_to_board_time_offset_s));
  AlignmentFrameSelectorConfig selector_config;
  selector_config.minimum_interval_s =
      options_.alignment_frame_minimum_interval_s;
  selector_config.maximum_interval_s =
      options_.alignment_frame_maximum_interval_s;
  selector_config.target_compensated_parallax_px =
      options_.alignment_target_compensated_parallax_px;
  selector_config.minimum_common_tracks =
      std::max(4, options_.min_feature_tracks / 4);
  selector_config.maximum_selected_frames = std::max(
      2, static_cast<int>(std::ceil(
             (maximum_window_duration_s_ + options_.buffer_margin_s +
              options_.max_time_offset_s) /
             std::max(1.0e-3,
                      options_.alignment_frame_minimum_interval_s))) +
             2);
  frame_selector_ = AlignmentFrameSelector(selector_config);

  if (!options_.formal_causal_lifecycle &&
      !options_.sliding_window_shadow_only &&
      !options_.upstream_dynamic_init_fc_gauge) {
    options_.candidate_filter_config.gravity_G = options_.gravity_G;
    options_.candidate_filter_config.noise.sigma_w = options_.imu_sigma_w;
    options_.candidate_filter_config.noise.sigma_wb = options_.imu_sigma_wb;
    options_.candidate_filter_config.noise.sigma_a = options_.imu_sigma_a;
    options_.candidate_filter_config.noise.sigma_ab = options_.imu_sigma_ab;
    options_.candidate_filter_config.position_sigma_m =
        options_.fc_position_sigma_m;
    options_.candidate_filter_config.velocity_sigma_mps =
        options_.fc_velocity_sigma_mps;
    candidate_filter_ =
        OnlineAlignmentCandidateFilter(options_.candidate_filter_config);
    const auto gate_configured = [&](CandidateStateGroup group) {
      return options_.candidate_filter_config
          .group_gates[static_cast<std::size_t>(group)]
          .configured;
    };
    const bool required_candidate_gates =
        gate_configured(CandidateStateGroup::ATTITUDE) &&
        gate_configured(CandidateStateGroup::POSITION) &&
        gate_configured(CandidateStateGroup::VELOCITY) &&
        (options_.release_policy !=
             AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT ||
         (gate_configured(CandidateStateGroup::GYRO_BIAS) &&
          gate_configured(CandidateStateGroup::ACCEL_BIAS)));
    if (!required_candidate_gates) {
      fatal_configuration_error_ = true;
      last_rejection_ = "candidate_filter_required_group_gate_not_configured";
      phase_ = AlignmentPhase::FATAL_CONFIGURATION_ERROR;
    }
  }
}

void OnlineAlignmentInitializer::transition(AlignmentPhase next,
                                            double stream_time,
                                            const std::string &reason) {
  if (phase_ == next)
    return;
  AlignmentTransition transition_record;
  transition_record.from = phase_;
  transition_record.to = next;
  transition_record.stream_time = stream_time;
  transition_record.reason = reason;
  transitions_.push_back(transition_record);
  phase_ = next;
}

void OnlineAlignmentInitializer::copy_runtime_counters(
    OnlineAlignmentDiagnostics &diagnostics) const {
  diagnostics.nonlinear_solve_attempt_count = nonlinear_solve_attempt_count_;
  diagnostics.successful_release_count = successful_release_count_;
  diagnostics.post_release_try_count = post_release_try_count_;
  diagnostics.alignment_window_closed = alignment_window_closed_;
  diagnostics.alignment_window_close_time = alignment_window_close_time_;
  diagnostics.solve_eligibility_skip_count = solve_eligibility_skip_count_;
  diagnostics.duplicate_window_skip_count = duplicate_window_skip_count_;
  diagnostics.candidate_created_count = candidate_created_count_;
  diagnostics.candidate_rejected_count = candidate_rejected_count_;
  diagnostics.candidate_refinement_count = candidate_refinement_count_;
  diagnostics.maximum_buffer_window_s = maximum_window_duration_s_;
  diagnostics.candidate_fc_imu_rotation_residual_deg =
      last_candidate_rotation_residual_deg_;
  diagnostics.candidate_relative_position_residual_m =
      last_candidate_position_residual_m_;
  diagnostics.candidate_relative_velocity_residual_mps =
      last_candidate_velocity_residual_mps_;
  diagnostics.candidate_visual_compensated_p95_px =
      last_candidate_visual_p95_px_;
  diagnostics.candidate_window_effective_measurement_counts =
      candidate_gate_depth_counts_;
  diagnostics.candidate_gate_depth_source = candidate_gate_depth_source_;
  if (candidate_filter_.config().group_gates.size() == 5) {
    for (std::size_t index = 0; index < 5; ++index) {
      const CandidateGroupGate &gate =
          candidate_filter_.config().group_gates[index];
      diagnostics.candidate_required_group_support_update_counts[index] =
          gate.min_supported_updates;
      diagnostics.candidate_required_group_history_lengths[index] =
          static_cast<int>(gate.history_length_updates);
    }
  }
}

void OnlineAlignmentInitializer::fail_retry(double stream_time,
                                            const std::string &reason) {
  // Once a formal refinement enters the nonlinear solver, it is the one
  // bounded refinement allowed by the lifecycle. A failed solve returns to a
  // fresh sliding-window candidate; it must not become a second refinement of
  // the old candidate. While force_refinement_solve_ remains true we are only
  // waiting for an advanced, support-complete window, so nothing is consumed.
  if (options_.formal_causal_lifecycle && formal_refinement_pending_ &&
      !force_refinement_solve_) {
    formal_refinement_pending_ = false;
    formal_candidate_camera_time_ = -1.0;
    formal_candidate_holdout_fc_samples_ = 0;
    formal_candidate_holdout_imu_samples_ = 0;
    formal_candidate_holdout_visual_frames_ = 0;
    formal_candidate_holdout_duration_s_ = 0.0;
    ++candidate_rejected_count_;
  }
  last_rejection_ = reason;
  ++retry_count_;
  if (navigation_released_)
    transition(AlignmentPhase::NAVIGATION_READY, stream_time,
               "navigation_running_full_alignment_pending:" + reason);
  else
    transition(AlignmentPhase::FAILED_WAIT_RETRY, stream_time, reason);
  last_diagnostics_.retry_count = retry_count_;
  last_diagnostics_.failed_gates.push_back(reason);
  last_diagnostics_.collection_start_time = collection_start_time_;
  last_diagnostics_.navigation_ready_time = navigation_ready_time_;
  last_diagnostics_.full_alignment_ready_time = full_alignment_ready_time_;
  last_diagnostics_.decision_time = navigation_ready_time_;
  last_diagnostics_.release_policy = options_.release_policy;
  last_diagnostics_.sliding_overlap_stability_required_s =
      options_.sliding_window_overlap_stability_duration_s;
  last_diagnostics_.direct_sliding_state_release =
      options_.sliding_window_direct_state_release;
  last_diagnostics_.readiness = navigation_released_
                                    ? AlignmentReadiness::NAVIGATION_READY
                                    : AlignmentReadiness::NOT_READY;
  last_diagnostics_.state_transitions = transitions_;
  last_diagnostics_.rejected_intervals = rejected_intervals_;
  last_diagnostics_.provenance = options_.provenance;
  copy_runtime_counters(last_diagnostics_);
  if (!attempt_receipts_.empty() &&
      attempt_receipts_.back().outcome == "solving") {
    attempt_receipts_.back().outcome = "rejected_window";
    attempt_receipts_.back().failed_gate = reason;
    attempt_receipts_.back().next_eligible_condition =
        "new_alignment_frame_or_improved_window_information";
  }
}

void OnlineAlignmentInitializer::mark_fatal(double stream_time,
                                            const std::string &reason) {
  fatal_configuration_error_ = true;
  last_rejection_ = reason;
  transition(AlignmentPhase::FATAL_CONFIGURATION_ERROR, stream_time, reason);
}

bool OnlineAlignmentInitializer::validate_fc(const FCNavigationSample &sample,
                                             std::string &reason) const {
  if (!std::isfinite(sample.timestamp) || !sample.position_G.allFinite() ||
      !sample.velocity_G.allFinite() || !sample.q_GtoF.allFinite())
    reason = "fc_non_finite";
  else if (!sample.position_valid || !sample.velocity_valid ||
           !sample.attitude_valid || !sample.status_valid)
    reason = "fc_quality_invalid";
  else if (std::fabs(sample.q_GtoF.norm() - 1.0) > 1e-3)
    reason = "fc_quaternion_not_unit";
  else if (sample.navigation_frame != options_.expected_navigation_frame)
    reason = "fc_navigation_frame_mismatch";
  else if (sample.body_frame != options_.expected_fc_body_frame)
    reason = "fc_body_frame_mismatch";
  else
    return true;
  return false;
}

bool OnlineAlignmentInitializer::validate_imu(const BoardImuSample &sample,
                                              std::string &reason) const {
  if (!std::isfinite(sample.timestamp) || !sample.angular_velocity.allFinite() ||
      !sample.linear_acceleration.allFinite())
    reason = "imu_non_finite";
  else if (!sample.status_valid)
    reason = "imu_quality_invalid";
  else if (sample.gyro_saturated || sample.accel_saturated)
    reason = "imu_saturated";
  else if (sample.frame != options_.expected_board_imu_frame)
    reason = "imu_frame_mismatch";
  else
    return true;
  return false;
}

bool OnlineAlignmentInitializer::validate_stereo(
    const StereoAlignmentFrame &frame, std::string &reason) const {
  if (!std::isfinite(frame.left_timestamp) ||
      !std::isfinite(frame.right_timestamp))
    reason = "stereo_timestamp_non_finite";
  else if (frame.left_timestamp < 0.0 || frame.right_timestamp < 0.0)
    reason = "stereo_timestamp_negative";
  else if (std::fabs(frame.left_timestamp - frame.right_timestamp) >
           options_.max_stereo_sync_gap_s)
    reason = "stereo_sync_gap";
  else if (!frame.tracking_valid)
    reason = "visual_tracking_invalid";
  else
    return true;
  return false;
}

VisualFrameSnapshot OnlineAlignmentInitializer::make_visual_snapshot(
    const StereoAlignmentFrame &frame) const {
  VisualFrameSnapshot snapshot;
  snapshot.timestamp = frame.left_timestamp;
  snapshot.tracks.reserve(frame.observations.size());
  for (const auto &observation : frame.observations) {
    VisualTrackPoint track;
    track.feature_id = observation.feature_id;
    track.raw = observation.raw_left;
    track.normalized = observation.normalized_left;
    track.track_age = observation.track_length;
    track.valid = observation.left_valid;
    snapshot.tracks.push_back(track);
  }
  return snapshot;
}

Eigen::Matrix3d OnlineAlignmentInitializer::relative_camera_rotation(
    double previous_camera_time, double current_camera_time) const {
  const double t0 = previous_camera_time + options_.camera_to_imu_time_offset_s;
  const double t1 = current_camera_time + options_.camera_to_imu_time_offset_s;
  if (!(t1 > t0))
    return Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_Icurrent_Iprevious = Eigen::Matrix3d::Identity();
  double previous_time = t0;
  BoardImuSample sample;
  if (!interpolate_imu(imu_buffer_, t0, sample))
    return Eigen::Matrix3d::Identity();
  Eigen::Vector3d omega = sample.angular_velocity;
  for (const auto &imu : imu_buffer_) {
    if (imu.timestamp <= t0 || imu.timestamp >= t1)
      continue;
    const double dt = imu.timestamp - previous_time;
    if (dt > 0.0) {
      const Eigen::Vector3d delta = -omega * dt;
      const double angle = delta.norm();
      if (angle > 1e-12)
        R_Icurrent_Iprevious =
            Eigen::AngleAxisd(angle, delta / angle).toRotationMatrix() *
            R_Icurrent_Iprevious;
    }
    previous_time = imu.timestamp;
    omega = imu.angular_velocity;
  }
  const double tail_dt = t1 - previous_time;
  if (tail_dt > 0.0) {
    const Eigen::Vector3d delta = -omega * tail_dt;
    const double angle = delta.norm();
    if (angle > 1e-12)
      R_Icurrent_Iprevious =
          Eigen::AngleAxisd(angle, delta / angle).toRotationMatrix() *
          R_Icurrent_Iprevious;
  }
  const Eigen::Matrix3d R_ItoC = ov_core::quat_2_Rot(options_.q_ItoC[0]);
  return R_ItoC * R_Icurrent_Iprevious * R_ItoC.transpose();
}

bool OnlineAlignmentInitializer::feed_fc_navigation(
    const FCNavigationSample &sample) {
  if (fatal_configuration_error_)
    return false;
  if (alignment_window_closed_) {
    last_rejection_ = navigation_released_
                          ? "alignment_already_released"
                          : "alignment_window_closed_explicit_reset_required";
    return false;
  }
  std::string reason;
  if (!validate_fc(sample, reason)) {
    last_rejection_ = reason;
    if (reason == "fc_navigation_frame_mismatch" ||
        reason == "fc_body_frame_mismatch" ||
        reason == "fc_non_finite" || reason == "fc_quaternion_not_unit")
      mark_fatal(sample.timestamp, reason);
    return false;
  }
  if (!fc_buffer_.empty() && sample.timestamp <= fc_buffer_.back().timestamp) {
    mark_fatal(sample.timestamp, "fc_non_monotonic_or_duplicate");
    return false;
  }
  if (!navigation_released_ && phase_ == AlignmentPhase::FAILED_WAIT_RETRY)
    transition(AlignmentPhase::COLLECTING, sample.timestamp,
               "new_fc_sample_after_failed_window");
  else if (!navigation_released_ && phase_ == AlignmentPhase::WAIT_INPUTS)
    transition(AlignmentPhase::COLLECTING, sample.timestamp,
               "first_valid_fc_sample");
  fc_buffer_.push_back(sample);
  prune(sample.timestamp);
  return true;
}

bool OnlineAlignmentInitializer::feed_board_imu(const BoardImuSample &sample) {
  if (fatal_configuration_error_)
    return false;
  if (alignment_window_closed_) {
    last_rejection_ = navigation_released_
                          ? "alignment_already_released"
                          : "alignment_window_closed_explicit_reset_required";
    return false;
  }
  std::string reason;
  if (!validate_imu(sample, reason)) {
    last_rejection_ = reason;
    if (reason == "imu_frame_mismatch" || reason == "imu_non_finite")
      mark_fatal(sample.timestamp, reason);
    return false;
  }
  if (!imu_buffer_.empty() && sample.timestamp <= imu_buffer_.back().timestamp) {
    mark_fatal(sample.timestamp, "imu_non_monotonic_or_duplicate");
    return false;
  }
  if (collection_start_time_ < 0.0)
    collection_start_time_ = sample.timestamp;
  if (!navigation_released_ && phase_ == AlignmentPhase::FAILED_WAIT_RETRY)
    transition(AlignmentPhase::COLLECTING, sample.timestamp,
               "new_imu_sample_after_failed_window");
  else if (!navigation_released_ && phase_ == AlignmentPhase::WAIT_INPUTS)
    transition(AlignmentPhase::COLLECTING, sample.timestamp,
               "first_valid_imu_sample");
  imu_buffer_.push_back(sample);
  prune(sample.timestamp);
  return true;
}

bool OnlineAlignmentInitializer::feed_stereo(const StereoAlignmentFrame &frame) {
  if (fatal_configuration_error_)
    return false;
  if (alignment_window_closed_) {
    last_rejection_ = navigation_released_
                          ? "alignment_already_released"
                          : "alignment_window_closed_explicit_reset_required";
    return false;
  }
  std::string reason;
  if (!validate_stereo(frame, reason)) {
    last_rejection_ = reason;
    if (reason == "stereo_timestamp_non_finite" ||
        reason == "stereo_timestamp_negative" || reason == "stereo_sync_gap")
      mark_fatal(frame.left_timestamp, reason);
    return false;
  }
  if (previous_tracking_snapshot_valid_ &&
      frame.left_timestamp <= previous_tracking_snapshot_.timestamp) {
    mark_fatal(frame.left_timestamp, "stereo_non_monotonic_or_duplicate");
    return false;
  }
  if (!navigation_released_ && phase_ == AlignmentPhase::FAILED_WAIT_RETRY)
    transition(AlignmentPhase::COLLECTING, frame.left_timestamp,
               "new_stereo_sample_after_failed_window");
  else if (!navigation_released_ && phase_ == AlignmentPhase::WAIT_INPUTS)
    transition(AlignmentPhase::COLLECTING, frame.left_timestamp,
               "first_valid_stereo_sample");
  StereoAlignmentFrame buffered = frame;
  if (options_.visual_perturbation_px > 0.0 &&
      options_.visual_perturbation_fraction > 0.0) {
    const size_t fraction_threshold = static_cast<size_t>(std::llround(
        1000.0 * std::min(1.0, options_.visual_perturbation_fraction)));
    for (auto &observation : buffered.observations) {
      if (observation.feature_id % 1000 >= fraction_threshold)
        continue;
      const double phase =
          2.3 * buffered.left_timestamp +
          0.37 * static_cast<double>(observation.feature_id % 17);
      const Eigen::Vector2d perturbation(
          options_.visual_perturbation_px * std::sin(phase),
          options_.visual_perturbation_px * std::cos(1.7 * phase));
      observation.raw_left += perturbation;
      observation.normalized_left += Eigen::Vector2d(
          perturbation.x() / options_.camera_intrinsics[0](0),
          perturbation.y() / options_.camera_intrinsics[0](1));
      if (observation.right_valid) {
        observation.raw_right += perturbation;
        observation.normalized_right += Eigen::Vector2d(
            perturbation.x() / options_.camera_intrinsics[1](0),
            perturbation.y() / options_.camera_intrinsics[1](1));
      }
    }
  }
  const VisualFrameSnapshot snapshot = make_visual_snapshot(buffered);
  VisualMotionMetrics motion;
  if (previous_tracking_snapshot_valid_) {
    const Eigen::Matrix3d relative_rotation = relative_camera_rotation(
        previous_tracking_snapshot_.timestamp, snapshot.timestamp);
    motion = compute_visual_motion_metrics(
        previous_tracking_snapshot_, snapshot, relative_rotation,
        options_.camera_intrinsics[0](0), options_.camera_intrinsics[0](1));
  }

  if (candidate_active_ && snapshot.timestamp > candidate_result_.timestamp) {
    candidate_visual_snapshots_.push_back(snapshot);
  }

  AlignmentFrameSelectionInput selection_input;
  selection_input.timestamp = snapshot.timestamp;
  selection_input.compensated_parallax_px = motion.compensated_median_px;
  selection_input.baseline_proxy = motion.compensated_p95_px;
  selection_input.survival_ratio = motion.survival_ratio;
  selection_input.common_tracks = motion.common_tracks;
  selection_input.excitation_score = motion.rotation_p95_px;
  selection_input.force_tail = stereo_buffer_.empty();
  double peak_rate = 0.0;
  if (previous_tracking_snapshot_valid_) {
    const double t0 = previous_tracking_snapshot_.timestamp +
                      options_.camera_to_imu_time_offset_s;
    const double t1 = snapshot.timestamp + options_.camera_to_imu_time_offset_s;
    for (const auto &imu : imu_buffer_)
      if (imu.timestamp >= t0 && imu.timestamp <= t1)
        peak_rate = std::max(peak_rate, imu.angular_velocity.norm());
  }
  selection_input.high_angular_rate =
      peak_rate > options_.max_angular_rate_rad_s;
  const AlignmentFrameSelectionDecision selection =
      frame_selector_.evaluate(selection_input);
  previous_tracking_snapshot_ = snapshot;
  previous_tracking_snapshot_valid_ = true;
  if (selection.selected) {
    stereo_buffer_.push_back(std::move(buffered));
    ++selected_frame_serial_;
  }
  prune(frame.left_timestamp);
  return true;
}

void OnlineAlignmentInitializer::prune(double newest_timestamp) {
  double keep_after = newest_timestamp - maximum_window_duration_s_ -
                      options_.buffer_margin_s - options_.max_time_offset_s;
  // Candidate validation is bounded and may reject into a newly advanced
  // refinement/current window. Retain the same finite raw-data horizon while
  // a candidate is active; summarizing it down to the filter cursor would make
  // that required re-solve impossible.
  while (fc_buffer_.size() > 2 && fc_buffer_[1].timestamp < keep_after)
    fc_buffer_.pop_front();
  while (imu_buffer_.size() > 2 && imu_buffer_[1].timestamp < keep_after)
    imu_buffer_.pop_front();
  // Visual alignment snapshots do not provide a bracketing service. Keeping
  // one or two stale frames can pin the common three-stream horizon forever
  // when the selector temporarily rejects new high-rate frames. Remove every
  // visual frame outside the supported solve horizon; the next valid tracking
  // frame will become the new forced tail.
  while (!stereo_buffer_.empty() &&
         stereo_buffer_.front().left_timestamp < keep_after)
    stereo_buffer_.pop_front();
  frame_selector_.prune_before(keep_after);
}

void OnlineAlignmentInitializer::reset() {
  fc_buffer_.clear();
  imu_buffer_.clear();
  stereo_buffer_.clear();
  last_diagnostics_ = OnlineAlignmentDiagnostics();
  last_rejection_.clear();
  phase_ = AlignmentPhase::WAIT_INPUTS;
  transitions_.clear();
  rejected_intervals_.clear();
  retry_count_ = 0;
  nonlinear_solve_attempt_count_ = 0;
  successful_release_count_ = 0;
  post_release_try_count_ = 0;
  navigation_released_ = false;
  full_alignment_recorded_ = false;
  alignment_window_closed_ = false;
  alignment_window_close_time_ = -1.0;
  collection_start_time_ = -1.0;
  navigation_ready_time_ = -1.0;
  full_alignment_ready_time_ = -1.0;
  maximum_window_duration_s_ = options_.candidate_window_durations_s.back();
  last_solve_stream_time_ = -1.0;
  selected_frames_at_last_solve_ = 0;
  selected_frame_serial_ = 0;
  selected_frame_serial_at_last_solve_ = 0;
  solve_eligibility_skip_count_ = 0;
  duplicate_window_skip_count_ = 0;
  last_window_fingerprint_.clear();
  frame_selector_.reset();
  previous_tracking_snapshot_ = VisualFrameSnapshot();
  previous_tracking_snapshot_valid_ = false;
  candidate_active_ = false;
  formal_refinement_pending_ = false;
  formal_candidate_camera_time_ = -1.0;
  formal_candidate_holdout_fc_samples_ = 0;
  formal_candidate_holdout_imu_samples_ = 0;
  formal_candidate_holdout_visual_frames_ = 0;
  formal_candidate_holdout_duration_s_ = 0.0;
  candidate_filter_.reset();
  candidate_record_ = AlignmentCandidate();
  candidate_result_ = AlignmentResult();
  candidate_created_time_ = -1.0;
  candidate_reference_snapshot_ = VisualFrameSnapshot();
  candidate_visual_snapshots_.clear();
  candidate_validation_frames_ = 0;
  candidate_visual_p95_max_ = 0.0;
  candidate_latest_visual_compensated_p95_px_ =
      std::numeric_limits<double>::infinity();
  candidate_latest_visual_reprojection_p95_px_ =
      std::numeric_limits<double>::infinity();
  candidate_latest_visual_reprojection_p50_px_ =
      std::numeric_limits<double>::infinity();
  candidate_last_validation_time_ = -1.0;
  candidate_validation_times_.clear();
  candidate_rotation_residuals_deg_.clear();
  candidate_position_residuals_m_.clear();
  candidate_velocity_residuals_mps_.clear();
  candidate_visual_reprojection_residuals_px_.clear();
  candidate_visual_frame_p95_px_.clear();
  last_candidate_rotation_residual_deg_ =
      std::numeric_limits<double>::infinity();
  last_candidate_position_residual_m_ =
      std::numeric_limits<double>::infinity();
  last_candidate_velocity_residual_mps_ =
      std::numeric_limits<double>::infinity();
  last_candidate_visual_p95_px_ =
      std::numeric_limits<double>::infinity();
  candidate_created_count_ = 0;
  candidate_rejected_count_ = 0;
  candidate_refinement_count_ = 0;
  force_refinement_solve_ = false;
  fatal_configuration_error_ = false;
  candidate_record_ = AlignmentCandidate();
  attempt_receipts_.clear();
  candidate_gate_depth_counts_.fill(0);
  candidate_gate_depth_source_ = "explicit_gate_configuration";
  previous_window_states_.clear();
  previous_window_covariance_.setZero();
  previous_window_covariance_valid_ = false;
  latest_shadow_window_result_ = AlignmentResult();
  latest_shadow_window_result_valid_ = false;
  latest_evidence_diagnostics_ = OnlineAlignmentDiagnostics();
  latest_evidence_diagnostics_valid_ = false;
  last_sliding_window_end_time_ = -1.0;
  sliding_window_id_ = 0;
  sliding_overlap_stable_since_ = -1.0;
  sliding_overlap_last_window_end_ = -1.0;
  sliding_prevalidation_stable_since_ = -1.0;
  sliding_prevalidation_last_window_end_ = -1.0;
}

bool OnlineAlignmentInitializer::interpolate_previous_window_state(
    double timestamp, SlidingWindowStateEstimate &state) const {
  if (previous_window_states_.empty() ||
      timestamp < previous_window_states_.front().timestamp - 1.0e-9 ||
      timestamp > previous_window_states_.back().timestamp + 1.0e-9)
    return false;
  const auto after = std::lower_bound(
      previous_window_states_.begin(), previous_window_states_.end(), timestamp,
      [](const SlidingWindowStateEstimate &sample, double target) {
        return sample.timestamp < target;
      });
  if (after == previous_window_states_.end()) {
    state = previous_window_states_.back();
    return true;
  }
  if (std::fabs(after->timestamp - timestamp) <= 1.0e-9 ||
      after == previous_window_states_.begin()) {
    state = *after;
    state.timestamp = timestamp;
    return true;
  }
  const auto before = std::prev(after);
  const double span = after->timestamp - before->timestamp;
  if (!(span > 0.0))
    return false;
  const double alpha = (timestamp - before->timestamp) / span;
  state.timestamp = timestamp;
  const Eigen::Quaterniond q0 =
      eigen_quaternion_from_jpl(before->q_GtoI);
  const Eigen::Quaterniond q1 = eigen_quaternion_from_jpl(after->q_GtoI);
  const Eigen::Quaterniond qi = q0.slerp(alpha, q1).normalized();
  state.q_GtoI << -qi.x(), -qi.y(), -qi.z(), qi.w();
  state.p_IinG =
      (1.0 - alpha) * before->p_IinG + alpha * after->p_IinG;
  state.v_IinG =
      (1.0 - alpha) * before->v_IinG + alpha * after->v_IinG;
  state.bg = (1.0 - alpha) * before->bg + alpha * after->bg;
  state.ba = (1.0 - alpha) * before->ba + alpha * after->ba;
  return state.q_GtoI.allFinite() && state.p_IinG.allFinite() &&
         state.v_IinG.allFinite() && state.bg.allFinite() &&
         state.ba.allFinite();
}

void OnlineAlignmentInitializer::reject_candidate(
    double now, const std::string &reason, bool request_refinement) {
  if (!attempt_receipts_.empty() &&
      attempt_receipts_.back().outcome == "candidate_ready") {
    attempt_receipts_.back().outcome = "candidate_rejected";
    attempt_receipts_.back().failed_gate = reason;
    attempt_receipts_.back().next_eligible_condition =
        request_refinement ? "single_bounded_refinement"
                           : "new_sliding_window_information";
  }
  candidate_active_ = false;
  candidate_filter_.reset();
  candidate_record_ = AlignmentCandidate();
  candidate_visual_snapshots_.clear();
  formal_refinement_pending_ = false;
  formal_candidate_camera_time_ = -1.0;
  formal_candidate_holdout_fc_samples_ = 0;
  formal_candidate_holdout_imu_samples_ = 0;
  formal_candidate_holdout_visual_frames_ = 0;
  formal_candidate_holdout_duration_s_ = 0.0;
  ++candidate_rejected_count_;
  last_rejection_ = reason;
  if (request_refinement && options_.candidate_refinement_enabled &&
      candidate_refinement_count_ == 0) {
    ++candidate_refinement_count_;
    force_refinement_solve_ = true;
    transition(AlignmentPhase::CANDIDATE_REFINING, now,
               "candidate_validation_requested_single_refinement:" + reason);
  } else {
    force_refinement_solve_ = false;
    transition(AlignmentPhase::FAILED_WAIT_RETRY, now,
               "candidate_rejected_continue_sliding_window:" + reason);
  }
}

bool OnlineAlignmentInitializer::advance_candidate_filter(
    OnlineAlignmentCandidateFilter &filter, double target_board_time,
    std::string &reason) const {
  reason.clear();
  if (!filter.runtime().active || !std::isfinite(target_board_time)) {
    reason = "candidate_filter_not_active_or_target_invalid";
    return false;
  }
  const double tolerance = filter.config().time_tolerance_s;
  if (target_board_time < filter.runtime().board_time - tolerance) {
    reason = "candidate_filter_target_precedes_cursor";
    return false;
  }

  auto propagate_to = [&](double end_time) {
    const double start_time = filter.runtime().board_time;
    if (end_time <= start_time + tolerance)
      return true;
    std::vector<BoardImuSample> segment;
    if (!interval_imu_samples(imu_buffer_, start_time, end_time, segment)) {
      reason = "candidate_filter_waiting_for_imu_bracket";
      return false;
    }
    for (size_t index = 1; index < segment.size(); ++index) {
      CandidateImuStep step;
      step.start_time = segment[index - 1].timestamp;
      step.end_time = segment[index].timestamp;
      step.angular_velocity_start = segment[index - 1].angular_velocity;
      step.angular_velocity_end = segment[index].angular_velocity;
      step.linear_acceleration_start = segment[index - 1].linear_acceleration;
      step.linear_acceleration_end = segment[index].linear_acceleration;
      std::string propagation_reason;
      if (!filter.propagate(step, &propagation_reason)) {
        reason = "candidate_filter_imu_propagation:" + propagation_reason;
        return false;
      }
    }
    return true;
  };

  const double navigation_offset =
      candidate_result_.fc_navigation_to_board_time_offset_s;
  const double attitude_offset =
      candidate_result_.fc_attitude_to_board_time_offset_s;
  for (const auto &fc_navigation : fc_buffer_) {
    const double event_board_time = fc_navigation.timestamp + navigation_offset;
    if (event_board_time <= filter.runtime().board_time + tolerance ||
        fc_navigation.timestamp <=
            filter.runtime().last_fc_source_timestamp + tolerance)
      continue;
    if (event_board_time > target_board_time + tolerance)
      break;

    const double attitude_time = event_board_time - attitude_offset;
    FCNavigationSample fc_attitude;
    Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
    if (!interpolate_fc(fc_buffer_, attitude_time, fc_attitude) ||
        !fc_angular_rate_at(fc_buffer_, attitude_time, options_.max_fc_gap_s,
                            omega_F)) {
      reason = "candidate_filter_waiting_for_fc_attitude_bracket";
      return false;
    }
    if (!propagate_to(event_board_time))
      return false;

    const Eigen::Matrix3d R_GtoF =
        ov_core::quat_2_Rot(fc_attitude.q_GtoF);
    CandidatePositionVelocityMeasurement measurement;
    measurement.source_timestamp = fc_navigation.timestamp;
    measurement.board_time = event_board_time;
    measurement.position_G =
        fc_navigation.position_G + R_GtoF.transpose() * options_.p_IinF;
    measurement.velocity_G =
        fc_navigation.velocity_G +
        R_GtoF.transpose() * omega_F.cross(options_.p_IinF);
    measurement.position_valid = fc_navigation.position_valid;
    measurement.velocity_valid = fc_navigation.velocity_valid;
    measurement.position_sigma_m = options_.fc_position_sigma_m;
    measurement.velocity_sigma_mps = options_.fc_velocity_sigma_mps;
    const CandidatePositionVelocityUpdateResult update =
        filter.updatePositionVelocity(measurement);
    if (!update.committed && !update.duplicate) {
      reason = "candidate_filter_fc_update:" + update.reason;
      return false;
    }
    if (filter.runtime().safety_violation) {
      const auto &runtime = filter.runtime();
      const auto group_summary = [&runtime](CandidateStateGroup group) {
        const std::size_t index = static_cast<std::size_t>(group);
        const auto &group_runtime = runtime.groups[index];
        std::ostringstream summary;
        summary << "{feedback_count=" << group_runtime.feedback_count
                << ",supported_count="
                << group_runtime.supported_update_count
                << ",cumulative_norm="
                << group_runtime.cumulative_feedback.norm()
                << ",last_norm=" << group_runtime.last_feedback.norm()
                << ",clipped=" << (group_runtime.last_feedback_clipped ? 1 : 0)
                << "}";
        return summary.str();
      };
      reason = "candidate_filter_feedback_safety_violation"
               ":consumed=" +
               std::to_string(runtime.consumed_fc_event_count) +
               ":accepted=" +
               std::to_string(runtime.accepted_fc_event_count) +
               ":q=" + group_summary(CandidateStateGroup::ATTITUDE) +
               ":p=" + group_summary(CandidateStateGroup::POSITION) +
               ":v=" + group_summary(CandidateStateGroup::VELOCITY) +
               ":bg=" + group_summary(CandidateStateGroup::GYRO_BIAS) +
               ":ba=" + group_summary(CandidateStateGroup::ACCEL_BIAS);
      return false;
    }
  }

  return propagate_to(target_board_time);
}

bool OnlineAlignmentInitializer::validate_formal_candidate(
    double now, AlignmentResult &result) {
  (void)result;
  if (!candidate_active_ || !std::isfinite(formal_candidate_camera_time_))
    return false;

  OnlineAlignmentDiagnostics diagnostics = candidate_result_.diagnostics;
  const double earliest_fc_offset =
      std::min(candidate_result_.fc_attitude_to_board_time_offset_s,
               candidate_result_.fc_navigation_to_board_time_offset_s);
  double supported_camera_time = now;
  if (!imu_buffer_.empty())
    supported_camera_time = std::min(
        supported_camera_time,
        imu_buffer_.back().timestamp - options_.camera_to_imu_time_offset_s);
  if (!fc_buffer_.empty())
    supported_camera_time = std::min(
        supported_camera_time,
        fc_buffer_.back().timestamp + earliest_fc_offset -
            options_.camera_to_imu_time_offset_s);
  if (!candidate_visual_snapshots_.empty())
    supported_camera_time =
        std::min(supported_camera_time,
                 candidate_visual_snapshots_.back().timestamp);

  const double candidate_board_time =
      formal_candidate_camera_time_ + options_.camera_to_imu_time_offset_s;
  const double supported_board_time =
      supported_camera_time + options_.camera_to_imu_time_offset_s;
  std::vector<const FCNavigationSample *> holdout_fc;
  std::vector<const BoardImuSample *> holdout_imu;
  std::vector<const StereoAlignmentFrame *> holdout_visual;
  for (const auto &sample : fc_buffer_) {
    const double board_time =
        sample.timestamp + options_.fc_navigation_to_board_time_offset_s;
    if (board_time > candidate_board_time + 1.0e-9 &&
        board_time <= supported_board_time + 1.0e-9)
      holdout_fc.push_back(&sample);
  }
  for (const auto &sample : imu_buffer_)
    if (sample.timestamp > candidate_board_time + 1.0e-9 &&
        sample.timestamp <= supported_board_time + 1.0e-9)
      holdout_imu.push_back(&sample);
  for (const auto &frame : stereo_buffer_)
    if (frame.left_timestamp > formal_candidate_camera_time_ + 1.0e-9 &&
        frame.left_timestamp <= supported_camera_time + 1.0e-9)
      holdout_visual.push_back(&frame);

  formal_candidate_holdout_fc_samples_ =
      static_cast<int>(holdout_fc.size());
  formal_candidate_holdout_imu_samples_ =
      static_cast<int>(holdout_imu.size());
  formal_candidate_holdout_visual_frames_ =
      static_cast<int>(holdout_visual.size());
  diagnostics.formal_candidate_holdout_fc_samples =
      formal_candidate_holdout_fc_samples_;
  diagnostics.formal_candidate_holdout_imu_samples =
      formal_candidate_holdout_imu_samples_;
  diagnostics.formal_candidate_holdout_visual_frames =
      formal_candidate_holdout_visual_frames_;
  diagnostics.formal_candidate_holdout_duration_s =
      std::max(0.0, supported_camera_time - formal_candidate_camera_time_);
  formal_candidate_holdout_duration_s_ =
      diagnostics.formal_candidate_holdout_duration_s;

  bool visual_tracking_health = false;
  if (!candidate_visual_snapshots_.empty()) {
    const VisualFrameSnapshot &latest = candidate_visual_snapshots_.back();
    const Eigen::Matrix3d relative_rotation = relative_camera_rotation(
        candidate_reference_snapshot_.timestamp, latest.timestamp);
    const VisualMotionMetrics motion = compute_visual_motion_metrics(
        candidate_reference_snapshot_, latest, relative_rotation,
        options_.camera_intrinsics[0](0), options_.camera_intrinsics[0](1));
    const int minimum_common_tracks =
        std::max(8, options_.min_feature_tracks / 4);
    visual_tracking_health =
        motion.valid && motion.common_tracks >= minimum_common_tracks &&
        std::isfinite(motion.compensated_p95_px);
    diagnostics.candidate_visual_compensated_p95_px =
        motion.compensated_p95_px;
    diagnostics.candidate_validation_frames =
        static_cast<int>(candidate_visual_snapshots_.size());
  }

  // Propagate the frozen candidate with held-out IMU only.  FC samples are
  // evaluated at the causal endpoint but are never fed back into the state.
  // This makes the holdout a real candidate check rather than merely a stream
  // availability timer.
  bool state_health = false;
  CandidateNominalState propagated;
  propagated.R_GtoI = ov_core::quat_2_Rot(candidate_result_.q_GtoI);
  propagated.p_IinG = candidate_result_.p_IinG;
  propagated.v_IinG = candidate_result_.v_IinG;
  propagated.bg = candidate_result_.bg;
  propagated.ba = candidate_result_.ba;
  std::vector<BoardImuSample> propagation_imu;
  if (supported_board_time > candidate_board_time + 1.0e-9 &&
      interval_imu_samples(imu_buffer_, candidate_board_time,
                           supported_board_time, propagation_imu)) {
    bool propagation_valid = true;
    for (std::size_t index = 1; index < propagation_imu.size(); ++index) {
      const BoardImuSample &previous = propagation_imu[index - 1];
      const BoardImuSample &current = propagation_imu[index];
      const double dt = current.timestamp - previous.timestamp;
      if (!(dt > 0.0) || dt > options_.max_imu_gap_s + 1.0e-9) {
        propagation_valid = false;
        break;
      }
      const Eigen::Vector3d omega =
          0.5 * (previous.angular_velocity + current.angular_velocity) -
          propagated.bg;
      const Eigen::Vector3d specific_force =
          0.5 * (previous.linear_acceleration + current.linear_acceleration) -
          propagated.ba;
      const Eigen::Matrix3d midpoint_rotation =
          ov_core::exp_so3(-0.5 * omega * dt) * propagated.R_GtoI;
      const Eigen::Vector3d acceleration_G =
          midpoint_rotation.transpose() * specific_force - options_.gravity_G;
      propagated.p_IinG +=
          propagated.v_IinG * dt + 0.5 * acceleration_G * dt * dt;
      propagated.v_IinG += acceleration_G * dt;
      propagated.R_GtoI =
          ov_core::exp_so3(-omega * dt) * propagated.R_GtoI;
    }

    FCNavigationSample fc_attitude;
    FCNavigationSample fc_navigation;
    Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
    const double fc_attitude_time =
        supported_board_time -
        candidate_result_.fc_attitude_to_board_time_offset_s;
    const double fc_navigation_time =
        supported_board_time -
        candidate_result_.fc_navigation_to_board_time_offset_s;
    if (propagation_valid &&
        interpolate_fc(fc_buffer_, fc_attitude_time, fc_attitude) &&
        interpolate_fc(fc_buffer_, fc_navigation_time, fc_navigation) &&
        fc_angular_rate_at(fc_buffer_, fc_attitude_time,
                           options_.max_fc_gap_s, omega_F)) {
      const Eigen::Matrix3d R_GtoF =
          ov_core::quat_2_Rot(fc_attitude.q_GtoF);
      const Eigen::Matrix3d fc_implied_R =
          candidate_result_.R_FtoI_nominal * R_GtoF;
      const Eigen::Vector3d target_position =
          fc_navigation.position_G +
          R_GtoF.transpose() * options_.p_IinF;
      const Eigen::Vector3d target_velocity =
          fc_navigation.velocity_G +
          R_GtoF.transpose() * omega_F.cross(options_.p_IinF);
      const double attitude_residual_deg =
          rotation_angle(propagated.R_GtoI * fc_implied_R.transpose()) *
          180.0 / kPi;
      const double position_residual_m =
          (propagated.p_IinG - target_position).norm();
      const double velocity_residual_mps =
          (propagated.v_IinG - target_velocity).norm();
      diagnostics.candidate_fc_imu_rotation_residual_deg =
          attitude_residual_deg;
      diagnostics.candidate_relative_position_residual_m =
          position_residual_m;
      diagnostics.candidate_relative_velocity_residual_mps =
          velocity_residual_mps;
      last_candidate_rotation_residual_deg_ = attitude_residual_deg;
      last_candidate_position_residual_m_ = position_residual_m;
      last_candidate_velocity_residual_mps_ = velocity_residual_mps;
      state_health =
          std::isfinite(attitude_residual_deg) &&
          std::isfinite(position_residual_m) &&
          std::isfinite(velocity_residual_mps) &&
          attitude_residual_deg <=
              options_.candidate_max_fc_imu_rotation_residual_deg &&
          position_residual_m <=
              options_.candidate_max_relative_position_residual_m &&
          velocity_residual_mps <=
              options_.candidate_max_relative_velocity_residual_mps;
    }
  }

  bool visual_health = false;
  if (visual_tracking_health && !candidate_visual_snapshots_.empty()) {
    const VisualFrameSnapshot &latest = candidate_visual_snapshots_.back();
    std::map<size_t, const VisualTrackPoint *> latest_tracks;
    for (const auto &track : latest.tracks)
      if (track.valid && track.normalized.allFinite())
        latest_tracks[track.feature_id] = &track;
    const double average_focal = std::sqrt(std::max(
        1.0, options_.camera_intrinsics[0](0) *
                 options_.camera_intrinsics[0](1)));
    const double normalized_sigma =
        options_.visual_pixel_sigma / average_focal;
    Eigen::Vector4d q_start = candidate_result_.q_GtoI;
    Eigen::Vector3d p_start = candidate_result_.p_IinG;
    Eigen::Vector4d q_end = ov_core::rot_2_quat(propagated.R_GtoI);
    Eigen::Vector3d p_end = propagated.p_IinG;
    const double *parameters[4] = {q_start.data(), p_start.data(),
                                   q_end.data(), p_end.data()};
    std::vector<double> epipolar_residuals_px;
    for (const auto &reference : candidate_reference_snapshot_.tracks) {
      const auto found = latest_tracks.find(reference.feature_id);
      if (!reference.valid || !reference.normalized.allFinite() ||
          found == latest_tracks.end())
        continue;
      p4::Factor_P4Epipolar factor(
          reference.normalized, found->second->normalized,
          ov_core::quat_2_Rot(options_.q_ItoC[0]), options_.p_IinC[0],
          normalized_sigma);
      double residual = 0.0;
      if (factor.Evaluate(parameters, &residual, nullptr) &&
          std::isfinite(residual))
        epipolar_residuals_px.push_back(
            std::fabs(residual) * options_.visual_pixel_sigma);
    }
    const double epipolar_p95_px =
        percentile(epipolar_residuals_px, 0.95);
    diagnostics.candidate_visual_reprojection_p95_px = epipolar_p95_px;
    candidate_latest_visual_reprojection_p95_px_ = epipolar_p95_px;
    last_candidate_visual_p95_px_ = epipolar_p95_px;
    visual_health =
        epipolar_residuals_px.size() >=
            static_cast<std::size_t>(
                std::max(8, options_.min_feature_tracks / 4)) &&
        std::isfinite(epipolar_p95_px) &&
        epipolar_p95_px <=
            options_.candidate_reject_visual_reprojection_p95_px;
  }

  const bool duration_complete =
      diagnostics.formal_candidate_holdout_duration_s + 1.0e-9 >=
      options_.candidate_short_validation_duration_s;
  const bool stream_support =
      holdout_fc.size() >= 2 && holdout_imu.size() >= 2 &&
      holdout_visual.size() >= 2 &&
      max_gap(holdout_fc) <= options_.max_fc_gap_s &&
      max_gap(holdout_imu) <= options_.max_imu_gap_s &&
      stereo_max_gap(holdout_visual) <= options_.max_stereo_gap_s;
  const bool holdout_passed =
      duration_complete && stream_support && visual_health && state_health;
  diagnostics.formal_candidate_holdout_passed = holdout_passed;
  copy_runtime_counters(diagnostics);

  if (!holdout_passed) {
    const double deadline =
        options_.candidate_short_validation_duration_s +
        std::max(options_.max_fc_gap_s, options_.max_stereo_gap_s);
    if (diagnostics.formal_candidate_holdout_duration_s + 1.0e-9 >=
        deadline) {
      reject_candidate(now, "formal_candidate_causal_holdout_failed", false);
      diagnostics.readiness_reason =
          "formal_candidate_causal_holdout_failed";
    } else {
      diagnostics.readiness = AlignmentReadiness::NOT_READY;
      diagnostics.readiness_reason = duration_complete
                                         ? "formal_candidate_waiting_for_holdout_quality"
                                         : "formal_candidate_short_causal_hold_pending";
      transition(AlignmentPhase::CANDIDATE_VALIDATING, now,
                 diagnostics.readiness_reason);
      last_rejection_ = diagnostics.readiness_reason;
    }
    diagnostics.state_transitions = transitions_;
    last_diagnostics_ = diagnostics;
    return false;
  }

  // Validation never mutates the candidate and never performs sequential FC
  // feedback.  Its only success action is to permit one solve on the newly
  // advanced current window.  That refinement owns the release covariance and
  // moves the injected state to the current camera horizon.
  candidate_active_ = false;
  formal_refinement_pending_ = true;
  force_refinement_solve_ = true;
  ++candidate_refinement_count_;
  diagnostics.candidate_closed_loop_refinement_applied = false;
  diagnostics.readiness = AlignmentReadiness::NOT_READY;
  diagnostics.readiness_reason =
      "formal_candidate_holdout_passed_refinement_pending";
  transition(AlignmentPhase::CANDIDATE_REFINING, now,
             diagnostics.readiness_reason);
  diagnostics.state_transitions = transitions_;
  copy_runtime_counters(diagnostics);
  last_diagnostics_ = diagnostics;
  last_rejection_ = diagnostics.readiness_reason;
  return false;
}

bool OnlineAlignmentInitializer::validate_candidate(double now,
                                                     AlignmentResult &result) {
  if (!candidate_active_ || !candidate_filter_.runtime().active)
    return false;

  OnlineAlignmentDiagnostics diagnostics = candidate_result_.diagnostics;
  copy_runtime_counters(diagnostics);
  const double requested_board_time =
      now + options_.camera_to_imu_time_offset_s;
  if (!std::isfinite(requested_board_time) || fc_buffer_.size() < 2 ||
      imu_buffer_.size() < 2) {
    last_rejection_ = "candidate_filter_waiting_for_streams";
    last_diagnostics_ = diagnostics;
    return false;
  }

  const double supported_board_time = std::min(
      {requested_board_time, imu_buffer_.back().timestamp,
       fc_buffer_.back().timestamp +
           candidate_result_.fc_navigation_to_board_time_offset_s,
       fc_buffer_.back().timestamp +
           candidate_result_.fc_attitude_to_board_time_offset_s});
  const double candidate_board_time =
      candidate_result_.timestamp + options_.camera_to_imu_time_offset_s;
  const double validation_deadline_s =
      options_.candidate_short_validation_duration_s +
      std::max(options_.max_fc_gap_s, options_.max_stereo_gap_s);
  const auto validation_elapsed = [&](double board_time) {
    return std::max(0.0, board_time - candidate_board_time);
  };
  if (supported_board_time <
      candidate_filter_.runtime().board_time -
          candidate_filter_.config().time_tolerance_s) {
    last_rejection_ = "candidate_filter_common_horizon";
    last_diagnostics_ = diagnostics;
    return false;
  }

  auto fail_or_wait = [&](const std::string &reason) {
    const bool hard_failure =
        reason.find("safety_violation") != std::string::npos ||
        reason.find("numerical") != std::string::npos ||
        reason.find("negative_eigenvalue") != std::string::npos ||
        reason.find("non_finite") != std::string::npos ||
        (reason.find("covariance") != std::string::npos &&
         reason.find("invalid") != std::string::npos);
    const bool deadline_reached =
        validation_elapsed(supported_board_time) + 1.0e-9 >=
        validation_deadline_s;
    if (hard_failure)
      reject_candidate(now, reason, false);
    else if (deadline_reached)
      reject_candidate(now, "candidate_validation_deadline:" + reason,
                       true);
    else {
      last_rejection_ = reason;
      transition(AlignmentPhase::CANDIDATE_VALIDATING, now, reason);
    }
    copy_runtime_counters(diagnostics);
    last_diagnostics_ = diagnostics;
  };

  const Eigen::Matrix3d R_ItoC =
      ov_core::quat_2_Rot(options_.q_ItoC[0]);
  const double fx = options_.camera_intrinsics[0](0);
  const double fy = options_.camera_intrinsics[0](1);
  while (!candidate_visual_snapshots_.empty()) {
    const VisualFrameSnapshot snapshot = candidate_visual_snapshots_.front();
    const double snapshot_board_time =
        snapshot.timestamp + options_.camera_to_imu_time_offset_s;
    if (snapshot_board_time > supported_board_time + 1.0e-9)
      break;

    std::string advance_reason;
    if (!advance_candidate_filter(candidate_filter_, snapshot_board_time,
                                  advance_reason)) {
      fail_or_wait(advance_reason);
      return false;
    }

    const Eigen::Matrix3d relative_rotation = relative_camera_rotation(
        candidate_reference_snapshot_.timestamp, snapshot.timestamp);
    const VisualMotionMetrics motion = compute_visual_motion_metrics(
        candidate_reference_snapshot_, snapshot, relative_rotation, fx, fy);
    if (motion.valid) {
      ++candidate_validation_frames_;
      candidate_latest_visual_compensated_p95_px_ =
          motion.compensated_p95_px;
      candidate_visual_p95_max_ =
          std::max(candidate_visual_p95_max_, motion.compensated_p95_px);
    }

    std::vector<double> reprojection_residuals;
    const CandidateNominalState &nominal =
        candidate_filter_.runtime().nominal;
    for (const auto &track : snapshot.tracks) {
      const auto landmark =
          candidate_record_.optimized_landmarks_G.find(track.feature_id);
      if (!track.valid ||
          landmark == candidate_record_.optimized_landmarks_G.end())
        continue;
      const Eigen::Vector3d p_FinI =
          nominal.R_GtoI * (landmark->second - nominal.p_IinG);
      const Eigen::Vector3d p_FinC =
          R_ItoC * p_FinI + options_.p_IinC[0];
      if (!p_FinC.allFinite() || !(p_FinC.z() > 1.0e-6))
        continue;
      const Eigen::Vector2d predicted = p_FinC.head<2>() / p_FinC.z();
      const Eigen::Vector2d residual(
          fx * (track.normalized.x() - predicted.x()),
          fy * (track.normalized.y() - predicted.y()));
      const double residual_norm = residual.norm();
      if (residual.allFinite() && std::isfinite(residual_norm))
        reprojection_residuals.push_back(residual_norm);
    }
    if (!reprojection_residuals.empty()) {
      candidate_latest_visual_reprojection_p50_px_ =
          percentile(reprojection_residuals, 0.50);
      candidate_latest_visual_reprojection_p95_px_ =
          percentile(reprojection_residuals, 0.95);
      candidate_validation_times_.push_back(snapshot.timestamp);
      candidate_visual_frame_p95_px_.push_back(
          candidate_latest_visual_reprojection_p95_px_);
    }
    candidate_reference_snapshot_ = snapshot;
    candidate_visual_snapshots_.pop_front();
  }

  std::string advance_reason;
  if (!advance_candidate_filter(candidate_filter_, supported_board_time,
                                advance_reason)) {
    fail_or_wait(advance_reason);
    return false;
  }

  const CandidateFilterRuntime &runtime = candidate_filter_.runtime();
  diagnostics.candidate_validation_duration_s =
      std::max(0.0, runtime.board_time -
                        (candidate_result_.timestamp +
                         options_.camera_to_imu_time_offset_s));
  diagnostics.candidate_validation_frames = candidate_validation_frames_;
  diagnostics.candidate_visual_compensated_p95_px =
      candidate_latest_visual_compensated_p95_px_;
  diagnostics.candidate_visual_reprojection_p95_px =
      candidate_latest_visual_reprojection_p95_px_;
  diagnostics.candidate_visual_reprojection_p50_px =
      candidate_latest_visual_reprojection_p50_px_;
  diagnostics.candidate_release_propagation_duration_s =
      diagnostics.candidate_validation_duration_s;
  if (candidate_validation_times_.size() >= 2 &&
      candidate_visual_frame_p95_px_.size() >= 2) {
    const double dt =
        candidate_validation_times_.back() -
        candidate_validation_times_.front();
    if (dt > 1.0e-9)
      diagnostics.candidate_visual_reprojection_trend_pxps =
          (candidate_visual_frame_p95_px_.back() -
           candidate_visual_frame_p95_px_.front()) /
          dt;
  } else {
    diagnostics.candidate_visual_reprojection_trend_pxps = 0.0;
  }

  FCNavigationSample fc_navigation;
  FCNavigationSample fc_attitude;
  Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
  const double navigation_time =
      runtime.board_time -
      candidate_result_.fc_navigation_to_board_time_offset_s;
  const double attitude_time =
      runtime.board_time -
      candidate_result_.fc_attitude_to_board_time_offset_s;
  if (!interpolate_fc(fc_buffer_, navigation_time, fc_navigation) ||
      !interpolate_fc(fc_buffer_, attitude_time, fc_attitude) ||
      !fc_angular_rate_at(fc_buffer_, attitude_time, options_.max_fc_gap_s,
                          omega_F)) {
    last_rejection_ = "candidate_filter_waiting_for_diagnostic_fc_bracket";
    last_diagnostics_ = diagnostics;
    return false;
  }
  const Eigen::Matrix3d R_GtoF =
      ov_core::quat_2_Rot(fc_attitude.q_GtoF);
  const Eigen::Matrix3d fc_implied_R =
      candidate_result_.R_FtoI_nominal * R_GtoF;
  const Eigen::Vector3d target_p =
      fc_navigation.position_G + R_GtoF.transpose() * options_.p_IinF;
  const Eigen::Vector3d target_v =
      fc_navigation.velocity_G +
      R_GtoF.transpose() * omega_F.cross(options_.p_IinF);
  const double attitude_residual_deg =
      rotation_angle(runtime.nominal.R_GtoI * fc_implied_R.transpose()) *
      180.0 / kPi;
  const double yaw_residual_deg =
      std::fabs(wrap_radians(yaw_from_R_GtoI(runtime.nominal.R_GtoI) -
                             yaw_from_R_GtoI(fc_implied_R))) *
      180.0 / kPi;
  const double position_residual_m =
      (runtime.nominal.p_IinG - target_p).norm();
  const double velocity_residual_mps =
      (runtime.nominal.v_IinG - target_v).norm();
  // These values are the pre-feedback residuals at this exact board time.
  // Using last_candidate_* here compared different timestamps and could not
  // establish whether the just-applied correction improved the measurement.
  diagnostics.candidate_pre_correction_position_residual_m =
      position_residual_m;
  diagnostics.candidate_pre_correction_velocity_residual_mps =
      velocity_residual_mps;
  diagnostics.candidate_fc_imu_rotation_residual_deg =
      attitude_residual_deg;
  diagnostics.candidate_fc_imu_yaw_residual_deg = yaw_residual_deg;
  diagnostics.candidate_relative_position_residual_m = position_residual_m;
  diagnostics.candidate_relative_velocity_residual_mps =
      velocity_residual_mps;
  diagnostics.candidate_nis_gate_3d = candidate_filter_.config().nis_gate_3d;
  diagnostics.candidate_last_position_nis = runtime.last_position_nis;
  diagnostics.candidate_last_velocity_nis = runtime.last_velocity_nis;
  diagnostics.candidate_max_position_nis = runtime.max_position_nis;
  diagnostics.candidate_max_velocity_nis = runtime.max_velocity_nis;
  diagnostics.candidate_fc_attitude_evaluation_only =
      !diagnostics.fc_attitude_gauge_factor_enabled;
  last_candidate_rotation_residual_deg_ = attitude_residual_deg;
  last_candidate_position_residual_m_ = position_residual_m;
  last_candidate_velocity_residual_mps_ = velocity_residual_mps;
  last_candidate_visual_p95_px_ =
      candidate_latest_visual_compensated_p95_px_;

  diagnostics.candidate_persistent_error = runtime.dx;
  for (int index = 0; index < 15; ++index)
    diagnostics.candidate_persistent_std(index) =
        std::sqrt(std::max(0.0, runtime.P(index, index)));
  diagnostics.candidate_consumed_fc_event_count =
      runtime.consumed_fc_event_count;
  diagnostics.candidate_accepted_fc_event_count =
      runtime.accepted_fc_event_count;
  diagnostics.candidate_rejected_fc_event_count =
      runtime.rejected_fc_event_count;
  diagnostics.candidate_propagated_imu_step_count =
      runtime.propagated_imu_step_count;
  diagnostics.candidate_duplicate_fc_event_count =
      runtime.duplicate_fc_event_count;
  diagnostics.candidate_duplicate_imu_step_count =
      runtime.duplicate_imu_step_count;
  switch (runtime.covariance_health) {
  case CandidateCovarianceHealth::HEALTHY:
    diagnostics.candidate_covariance_health = "healthy";
    break;
  case CandidateCovarianceHealth::ROUNDOFF_REPAIRED:
    diagnostics.candidate_covariance_health = "roundoff_repaired";
    break;
  case CandidateCovarianceHealth::NON_FINITE:
    diagnostics.candidate_covariance_health = "non_finite";
    break;
  case CandidateCovarianceHealth::MATERIAL_NEGATIVE_EIGENVALUE:
    diagnostics.candidate_covariance_health =
        "material_negative_eigenvalue";
    break;
  case CandidateCovarianceHealth::EIGENSOLVER_FAILURE:
    diagnostics.candidate_covariance_health = "eigensolver_failure";
    break;
  }

  const auto group_index = [](CandidateStateGroup group) {
    return static_cast<std::size_t>(group);
  };
  const CandidateGroupRuntime &attitude_group =
      runtime.groups[group_index(CandidateStateGroup::ATTITUDE)];
  const CandidateGroupRuntime &position_group =
      runtime.groups[group_index(CandidateStateGroup::POSITION)];
  const CandidateGroupRuntime &velocity_group =
      runtime.groups[group_index(CandidateStateGroup::VELOCITY)];
  const CandidateGroupRuntime &gyro_bias_group =
      runtime.groups[group_index(CandidateStateGroup::GYRO_BIAS)];
  const CandidateGroupRuntime &accel_bias_group =
      runtime.groups[group_index(CandidateStateGroup::ACCEL_BIAS)];
  for (std::size_t index = 0; index < 5; ++index) {
    diagnostics.candidate_group_supported_update_counts[index] =
        runtime.groups[index].supported_update_count;
    diagnostics.candidate_group_feedback_counts[index] =
        runtime.groups[index].feedback_count;
    diagnostics.candidate_group_post_feedback_stable_update_counts[index] =
        runtime.groups[index].post_first_feedback_stable_updates;
    diagnostics.candidate_group_gate_passed[index] =
        runtime.groups[index].last_gate_passed;
  }
  diagnostics.candidate_closed_loop_attitude_correction_deg =
      attitude_group.cumulative_feedback.norm() * 180.0 / kPi;
  diagnostics.candidate_closed_loop_position_correction_m =
      position_group.cumulative_feedback.norm();
  diagnostics.candidate_closed_loop_velocity_correction_mps =
      velocity_group.cumulative_feedback.norm();
  diagnostics.candidate_closed_loop_gyro_bias_correction_rad_s =
      gyro_bias_group.cumulative_feedback.norm();
  diagnostics.candidate_closed_loop_accel_bias_correction_mps2 =
      accel_bias_group.cumulative_feedback.norm();
  diagnostics.candidate_closed_loop_update_count =
      runtime.accepted_fc_event_count;
  diagnostics.candidate_closed_loop_measurement_update_count =
      runtime.consumed_fc_event_count;
  diagnostics.candidate_closed_loop_rejected_measurement_count =
      runtime.rejected_fc_event_count;
  diagnostics.candidate_closed_loop_visual_update_count = 0;
  diagnostics.candidate_closed_loop_correction_applied =
      position_group.feedback_count > 0 || velocity_group.feedback_count > 0 ||
      attitude_group.feedback_count > 0 ||
      gyro_bias_group.feedback_count > 0 ||
      accel_bias_group.feedback_count > 0;
  diagnostics.candidate_covariance_joseph_update_applied =
      runtime.accepted_fc_event_count > 0;
  diagnostics.candidate_covariance_error_reset_applied =
      attitude_group.feedback_count > 0;
  diagnostics.candidate_attitude_feedback_allowed =
      candidate_filter_.groupReady(CandidateStateGroup::ATTITUDE);
  diagnostics.candidate_gyro_bias_feedback_allowed =
      candidate_filter_.groupReady(CandidateStateGroup::GYRO_BIAS);
  diagnostics.candidate_accel_bias_feedback_allowed =
      candidate_filter_.groupReady(CandidateStateGroup::ACCEL_BIAS);
  diagnostics.candidate_attitude_stable_duration_s =
      attitude_group.post_feedback_stable_duration_s;
  diagnostics.candidate_gyro_bias_stable_duration_s =
      gyro_bias_group.post_feedback_stable_duration_s;
  diagnostics.candidate_accel_bias_stable_duration_s =
      accel_bias_group.post_feedback_stable_duration_s;
  const CandidateFeedbackTier tier = candidate_filter_.feedbackTier();
  diagnostics.candidate_feedback_tier =
      candidate_feedback_tier_name(tier);
  switch (tier) {
  case CandidateFeedbackTier::PV_ONLY:
    diagnostics.candidate_feedback_state_mask = "p,v";
    break;
  case CandidateFeedbackTier::ATTITUDE:
    diagnostics.candidate_feedback_state_mask = "q,p,v";
    break;
  case CandidateFeedbackTier::ATTITUDE_GYRO_BIAS:
    diagnostics.candidate_feedback_state_mask = "q,p,v,bg";
    break;
  case CandidateFeedbackTier::ATTITUDE_ACCEL_BIAS:
    diagnostics.candidate_feedback_state_mask = "q,p,v,ba";
    break;
  case CandidateFeedbackTier::FULL:
    diagnostics.candidate_feedback_state_mask = "q,p,v,bg,ba";
    break;
  }

  const double attitude_variance =
      std::max(1.0e-12, runtime.P.block<3, 3>(0, 0).trace());
  const double position_variance =
      std::max(1.0e-12, runtime.P.block<3, 3>(3, 3).trace());
  const double velocity_variance =
      std::max(1.0e-12, runtime.P.block<3, 3>(6, 6).trace());
  diagnostics.candidate_covariance_normalized_residual = std::sqrt(
      std::pow(attitude_residual_deg * kPi / 180.0, 2) /
      attitude_variance +
      position_residual_m * position_residual_m / position_variance +
      velocity_residual_mps * velocity_residual_mps / velocity_variance);

  const bool navigation_only_without_visual =
      !options_.visual_factors_enabled &&
      options_.navigation_allow_without_visual;
  const bool visual_motion_healthy =
      navigation_only_without_visual ||
      (candidate_validation_frames_ > 0 &&
      std::isfinite(candidate_latest_visual_compensated_p95_px_) &&
      candidate_latest_visual_compensated_p95_px_ <=
          options_.candidate_max_visual_compensated_p95_px);
  const bool visual_reprojection_healthy =
      navigation_only_without_visual ||
      !std::isfinite(candidate_latest_visual_reprojection_p95_px_) ||
      candidate_latest_visual_reprojection_p95_px_ <=
          options_.candidate_max_visual_reprojection_p95_px;
  const bool visual_trend_healthy =
      navigation_only_without_visual ||
      !std::isfinite(
          diagnostics.candidate_visual_reprojection_trend_pxps) ||
      diagnostics.candidate_visual_reprojection_trend_pxps <=
          options_.candidate_max_visual_reprojection_trend_pxps;
  const double fc_attitude_holdout_limit_deg = std::min(
      options_.candidate_max_fc_imu_rotation_residual_deg,
      std::hypot(options_.fc_attitude_sigma_deg,
                 options_.fc_board_mount_sigma_deg));
  const bool fc_attitude_holdout_healthy =
      !options_.fc_attitude_gauge_factor_enabled ||
      (std::isfinite(yaw_residual_deg) &&
       yaw_residual_deg <= fc_attitude_holdout_limit_deg);
  const bool trust_gyro_bias =
      candidate_filter_.groupReady(CandidateStateGroup::GYRO_BIAS);
  const bool trust_accel_bias =
      candidate_filter_.groupReady(CandidateStateGroup::ACCEL_BIAS);
  const bool retained_gyro_bias_bounded =
      trust_gyro_bias ||
      runtime.dx.segment<3>(9).norm() <=
          options_.candidate_max_closed_loop_gyro_bias_correction_rad_s;
  const bool retained_accel_bias_bounded =
      trust_accel_bias ||
      runtime.dx.segment<3>(12).norm() <=
          options_.candidate_max_closed_loop_accel_bias_correction_mps2;
  const bool short_causal_hold_complete =
      diagnostics.candidate_validation_duration_s + 1.0e-9 >=
      options_.candidate_short_validation_duration_s;
  const bool navigation_ready =
      short_causal_hold_complete && candidate_filter_.navigationReady() &&
      fc_attitude_holdout_healthy &&
      visual_motion_healthy &&
      visual_reprojection_healthy && visual_trend_healthy &&
      retained_gyro_bias_bounded && retained_accel_bias_bounded;
  const bool full_ready = navigation_ready && !navigation_only_without_visual &&
                          candidate_filter_.fullAlignmentReady();
  const bool accepted =
      options_.release_policy == AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT
          ? full_ready
          : navigation_ready;

  diagnostics.navigation_quality_passed = navigation_ready;
  diagnostics.full_alignment_quality_passed = full_ready;
  diagnostics.quality_passed = accepted;
  if (!accepted) {
    if (!short_causal_hold_complete)
      last_rejection_ = "candidate_short_causal_hold_pending";
    else if (!candidate_filter_.groupReady(CandidateStateGroup::ATTITUDE))
      last_rejection_ =
          "candidate_attitude_waiting_for_observability_convergence_feedback";
    else if (!candidate_filter_.groupReady(CandidateStateGroup::POSITION) ||
             !candidate_filter_.groupReady(CandidateStateGroup::VELOCITY))
      last_rejection_ =
          "candidate_position_velocity_waiting_for_convergence";
    else if (!fc_attitude_holdout_healthy)
      last_rejection_ =
          "candidate_fc_attitude_holdout_waiting_for_joint_window_refinement";
    else if (!visual_motion_healthy || !visual_reprojection_healthy ||
             !visual_trend_healthy)
      last_rejection_ = "candidate_visual_health_waiting_for_recovery";
    else if (!retained_gyro_bias_bounded ||
             !retained_accel_bias_bounded)
      last_rejection_ = "candidate_unresolved_bias_correction_unbounded";
    else
      last_rejection_ =
          "candidate_strict_bias_waiting_for_observability_convergence_feedback";
    transition(AlignmentPhase::CANDIDATE_VALIDATING, now,
               last_rejection_);
    diagnostics.readiness = AlignmentReadiness::NOT_READY;
    diagnostics.readiness_reason = last_rejection_;
    diagnostics.state_transitions = transitions_;
    copy_runtime_counters(diagnostics);
    last_diagnostics_ = diagnostics;
    if (diagnostics.candidate_validation_duration_s + 1.0e-9 >=
        validation_deadline_s) {
      const std::string deadline_reason =
          "candidate_validation_deadline:" + last_rejection_;
      reject_candidate(now, deadline_reason, true);
      diagnostics.readiness_reason = deadline_reason;
      diagnostics.state_transitions = transitions_;
      copy_runtime_counters(diagnostics);
      last_diagnostics_ = diagnostics;
    }
    return false;
  }

  OnlineAlignmentCandidateFilter release_filter = candidate_filter_;
  const auto apply_release_feedback =
      [&](CandidateStateGroup group, const std::string &failure_reason) {
        const CandidateFeedbackResult feedback =
            release_filter.feedbackGroup(group);
        if (!feedback.applied || feedback.clipped ||
            feedback.safety_violation) {
          reject_candidate(now, failure_reason, false);
          return false;
        }
        return true;
      };
  const bool release_gyro_bias = trust_gyro_bias &&
                                 !navigation_only_without_visual;
  const bool release_accel_bias = trust_accel_bias &&
                                  !navigation_only_without_visual;
  for (CandidateStateGroup group :
       {CandidateStateGroup::POSITION, CandidateStateGroup::VELOCITY,
        CandidateStateGroup::ATTITUDE}) {
    if (!apply_release_feedback(
            group, "candidate_release_feedback_not_fully_applied"))
      return false;
  }
  if (release_gyro_bias &&
      !apply_release_feedback(
          CandidateStateGroup::GYRO_BIAS,
          "candidate_release_gyro_bias_feedback_not_fully_applied"))
    return false;
  if (release_accel_bias &&
      !apply_release_feedback(
          CandidateStateGroup::ACCEL_BIAS,
          "candidate_release_accel_bias_feedback_not_fully_applied"))
    return false;

  CandidateCovarianceHealth release_covariance_health =
      CandidateCovarianceHealth::HEALTHY;
  const CandidateCovariance release_covariance =
      release_filter.releaseCovariance(
          release_gyro_bias, release_accel_bias, &release_covariance_health);
  if (!release_covariance.allFinite() ||
      release_covariance_health ==
          CandidateCovarianceHealth::NON_FINITE ||
      release_covariance_health ==
          CandidateCovarianceHealth::MATERIAL_NEGATIVE_EIGENVALUE ||
      release_covariance_health ==
          CandidateCovarianceHealth::EIGENSOLVER_FAILURE) {
    reject_candidate(now, "candidate_release_covariance_invalid", false);
    return false;
  }

  const CandidateFilterRuntime &release_runtime =
      release_filter.runtime();
  // Evaluate the same FC target again after q/p/v feedback and covariance
  // reset. This is the causal before/after evidence used by tests and logs.
  diagnostics.candidate_post_correction_attitude_residual_deg =
      rotation_angle(release_runtime.nominal.R_GtoI *
                     fc_implied_R.transpose()) *
      180.0 / kPi;
  diagnostics.candidate_post_correction_position_residual_m =
      (release_runtime.nominal.p_IinG - target_p).norm();
  diagnostics.candidate_post_correction_velocity_residual_mps =
      (release_runtime.nominal.v_IinG - target_v).norm();
  result = candidate_result_;
  result.timestamp =
      release_runtime.board_time -
      options_.camera_to_imu_time_offset_s;
  result.q_GtoI =
      ov_core::rot_2_quat(release_runtime.nominal.R_GtoI);
  result.p_IinG = release_runtime.nominal.p_IinG;
  result.v_IinG = release_runtime.nominal.v_IinG;
  result.bg = release_gyro_bias
                  ? release_runtime.nominal.bg
                  : release_runtime.initial_nominal.bg;
  result.ba = release_accel_bias
                  ? release_runtime.nominal.ba
                  : release_runtime.initial_nominal.ba;
  result.covariance = release_covariance;
  result.readiness =
      release_gyro_bias && release_accel_bias
          ? AlignmentReadiness::FULL_ALIGNMENT_READY
          : AlignmentReadiness::NAVIGATION_READY;
  result.released_to_openvins = true;
  diagnostics.readiness = result.readiness;
  diagnostics.readiness_reason =
      result.readiness == AlignmentReadiness::FULL_ALIGNMENT_READY
          ? "persistent_closed_loop_q_p_v_bg_ba_converged_and_feedback_applied"
          : "persistent_closed_loop_q_p_v_converged_bias_retained_with_conservative_covariance";
  diagnostics.quality_passed = true;
  diagnostics.decision_time = now;
  const auto erase_state_name = [](std::vector<std::string> &states,
                                   const std::string &name) {
    states.erase(std::remove(states.begin(), states.end(), name),
                 states.end());
  };
  const auto set_release_state_status = [&](const std::string &name,
                                            bool trusted) {
    erase_state_name(diagnostics.estimated_state_list, name);
    erase_state_name(diagnostics.weak_state_list, name);
    erase_state_name(diagnostics.fixed_state_list, name);
    erase_state_name(diagnostics.unobservable_state_list, name);
    for (auto &state : diagnostics.state_observability) {
      if (state.state != name)
        continue;
      state.observable = trusted;
      state.prior_only = !trusted;
      state.estimate_status = trusted
                                  ? EstimateSourceStatus::ESTIMATED_CURRENT_DATA
                                  : EstimateSourceStatus::FIXED_TO_PRIOR;
    }
    if (trusted)
      diagnostics.estimated_state_list.push_back(name);
    else
      diagnostics.fixed_state_list.push_back(name);
  };
  set_release_state_status("attitude", true);
  set_release_state_status("position", true);
  set_release_state_status("velocity", true);
  set_release_state_status("gyro_bias", release_gyro_bias);
  set_release_state_status("accelerometer_bias", release_accel_bias);
  navigation_released_ = true;
  ++successful_release_count_;
  navigation_ready_time_ = now;
  alignment_window_closed_ = true;
  alignment_window_close_time_ = now;
  diagnostics.navigation_ready_time = navigation_ready_time_;
  if (result.readiness == AlignmentReadiness::FULL_ALIGNMENT_READY) {
    full_alignment_recorded_ = true;
    full_alignment_ready_time_ = now;
    diagnostics.full_alignment_ready_time = now;
    transition(AlignmentPhase::FULL_ALIGNMENT_READY, now,
               "persistent_candidate_closed_loop_release");
  } else {
    transition(AlignmentPhase::NAVIGATION_READY, now,
               "persistent_candidate_closed_loop_release");
  }
  candidate_active_ = false;
  candidate_visual_snapshots_.clear();
  if (!attempt_receipts_.empty() &&
      attempt_receipts_.back().outcome == "candidate_ready") {
    attempt_receipts_.back().outcome = "validated_release";
    attempt_receipts_.back().next_eligible_condition =
        "closed_after_release";
  }
  fc_buffer_.clear();
  imu_buffer_.clear();
  stereo_buffer_.clear();
  copy_runtime_counters(diagnostics);
  diagnostics.state_transitions = transitions_;
  result.diagnostics = diagnostics;
  last_diagnostics_ = diagnostics;
  last_rejection_.clear();
  return true;
}

bool OnlineAlignmentInitializer::provisional_navigation(
    double now, ProvisionalNavigationOutput &output) const {
  output = ProvisionalNavigationOutput();
  if (alignment_window_closed_ || fc_buffer_.empty())
    return false;
  output.timestamp = now;
  if (candidate_active_ && candidate_filter_.runtime().active &&
      imu_buffer_.size() >= 2 && fc_buffer_.size() >= 2) {
    OnlineAlignmentCandidateFilter shadow = candidate_filter_;
    const double requested_board_time =
        now + options_.camera_to_imu_time_offset_s;
    const double supported_board_time = std::min(
        {requested_board_time, imu_buffer_.back().timestamp,
         fc_buffer_.back().timestamp +
             candidate_result_.fc_navigation_to_board_time_offset_s,
         fc_buffer_.back().timestamp +
             candidate_result_.fc_attitude_to_board_time_offset_s});
    std::string reason;
    if (!advance_candidate_filter(shadow, supported_board_time, reason))
      return false;
    const CandidateFilterRuntime &runtime = shadow.runtime();
    output.timestamp =
        runtime.board_time - options_.camera_to_imu_time_offset_s;
    output.q_GtoI = ov_core::rot_2_quat(runtime.nominal.R_GtoI);
    output.p_IinG = runtime.nominal.p_IinG;
    output.v_IinG = runtime.nominal.v_IinG;
    output.bg = runtime.nominal.bg;
    output.ba = runtime.nominal.ba;
    output.covariance = runtime.P;
    output.source = "persistent_candidate_filter_shadow_provisional";
    output.valid = output.q_GtoI.allFinite() && output.p_IinG.allFinite() &&
                   output.v_IinG.allFinite() && output.covariance.allFinite();
    output.joint_initialization_pending_reason = last_rejection_;
    output.official_openvins_state = false;
    return output.valid;
  }
  double propagation_start = -1.0;
  Eigen::Matrix3d R_GtoI = Eigen::Matrix3d::Identity();
  const double requested_fc_time =
      now + options_.camera_to_imu_time_offset_s;
  const double supported_fc_time =
      std::min(requested_fc_time, fc_buffer_.back().timestamp);
  FCNavigationSample fc_seed;
  if (!interpolate_fc(fc_buffer_, supported_fc_time, fc_seed))
    return false;
  const Eigen::Matrix3d R_GtoF = ov_core::quat_2_Rot(fc_seed.q_GtoF);
  R_GtoI = options_.R_FtoI_provisional_seed * R_GtoF;
  output.p_IinG =
      fc_seed.position_G + R_GtoF.transpose() * options_.p_IinF;
  output.v_IinG = fc_seed.velocity_G;
  propagation_start = supported_fc_time;
  output.source = "exact_time_fc_seed_board_imu_propagated_provisional";
  const double propagation_end =
      now + options_.camera_to_imu_time_offset_s;
  double previous_time = propagation_start;
  for (const auto &imu : imu_buffer_) {
    if (imu.timestamp <= propagation_start || imu.timestamp > propagation_end)
      continue;
    const double dt = imu.timestamp - previous_time;
    if (!(dt > 0.0))
      continue;
    const Eigen::Vector3d acceleration_G =
        R_GtoI.transpose() * (imu.linear_acceleration - output.ba) -
        options_.gravity_G;
    output.p_IinG += output.v_IinG * dt +
                     0.5 * acceleration_G * dt * dt;
    output.v_IinG += acceleration_G * dt;
    const Eigen::Vector3d delta = -(imu.angular_velocity - output.bg) * dt;
    const double angle = delta.norm();
    if (angle > 1e-12)
      R_GtoI = Eigen::AngleAxisd(angle, delta / angle).toRotationMatrix() *
                 R_GtoI;
    previous_time = imu.timestamp;
  }
  const double propagated_duration = std::max(0.0, previous_time - propagation_start);
  output.covariance.diagonal().array() +=
      1.0e3 + propagated_duration * 10.0;
  output.q_GtoI = ov_core::rot_2_quat(R_GtoI);
  output.valid = output.q_GtoI.allFinite() && output.p_IinG.allFinite() &&
                 output.v_IinG.allFinite();
  output.joint_initialization_pending_reason = last_rejection_;
  output.official_openvins_state = false;
  return output.valid;
}

bool OnlineAlignmentInitializer::make_fc_gauge_target(
    double camera_timestamp, AlignmentResult &result) {
  result = AlignmentResult();
  last_diagnostics_ = OnlineAlignmentDiagnostics();
  last_diagnostics_.solve_time = camera_timestamp;
  last_diagnostics_.decision_time = camera_timestamp;
  last_diagnostics_.future_data_used = false;
  last_diagnostics_.post_alignment_used = false;
  last_diagnostics_.provenance = options_.provenance;
  last_diagnostics_.release_policy = options_.release_policy;
  last_diagnostics_.selected_window_duration_s =
      options_.sliding_window_duration_s;
  if (!options_.upstream_dynamic_init_fc_gauge) {
    last_rejection_ = "fc_gauge_target_mode_disabled";
    return false;
  }
  if (fatal_configuration_error_ || alignment_window_closed_ ||
      !std::isfinite(camera_timestamp)) {
    last_rejection_ = fatal_configuration_error_
                          ? "fatal_configuration_error"
                          : (alignment_window_closed_
                                 ? "alignment_window_closed"
                                 : "camera_timestamp_non_finite");
    return false;
  }

  if (fc_buffer_.empty() || imu_buffer_.empty()) {
    last_rejection_ = "fc_gauge_waiting_for_sensor_data";
    return false;
  }
  const double requested_board_time =
      camera_timestamp + options_.camera_to_imu_time_offset_s;
  // The ROS-free runner is strictly causal: when a camera frame is handled,
  // the latest IMU sample is normally just before that frame and there is no
  // future sample available for a right-hand interpolation bracket.  Use the
  // newest board time supported by all three streams instead of requiring a
  // non-causal sample after the current camera timestamp.
  const double board_time = std::min(
      {requested_board_time, imu_buffer_.back().timestamp,
       fc_buffer_.back().timestamp +
           options_.fc_attitude_to_board_time_offset_s,
       fc_buffer_.back().timestamp +
           options_.fc_navigation_to_board_time_offset_s});
  const double supported_camera_time =
      board_time - options_.camera_to_imu_time_offset_s;
  last_diagnostics_.solve_time = supported_camera_time;
  last_diagnostics_.decision_time = supported_camera_time;
  const double window_start_board =
      board_time - options_.sliding_window_duration_s;
  const double attitude_time =
      board_time - options_.fc_attitude_to_board_time_offset_s;
  const double navigation_time =
      board_time - options_.fc_navigation_to_board_time_offset_s;
  const double start_attitude_time =
      window_start_board - options_.fc_attitude_to_board_time_offset_s;
  const double start_navigation_time =
      window_start_board - options_.fc_navigation_to_board_time_offset_s;

  FCNavigationSample fc_attitude;
  FCNavigationSample fc_navigation;
  FCNavigationSample fc_attitude_start;
  FCNavigationSample fc_navigation_start;
  BoardImuSample imu_start;
  BoardImuSample imu_end;
  double attitude_bracket_gap = std::numeric_limits<double>::infinity();
  double navigation_bracket_gap = std::numeric_limits<double>::infinity();
  if (!interpolate_fc(fc_buffer_, attitude_time, fc_attitude,
                      &attitude_bracket_gap) ||
      !interpolate_fc(fc_buffer_, navigation_time, fc_navigation,
                      &navigation_bracket_gap) ||
      !interpolate_fc(fc_buffer_, start_attitude_time, fc_attitude_start) ||
      !interpolate_fc(fc_buffer_, start_navigation_time,
                      fc_navigation_start) ||
      !interpolate_imu(imu_buffer_, window_start_board, imu_start) ||
      !interpolate_imu(imu_buffer_, board_time, imu_end)) {
    last_rejection_ = "fc_gauge_waiting_for_complete_causal_window";
    transition(AlignmentPhase::COLLECTING, supported_camera_time,
               last_rejection_);
    return false;
  }
  if (attitude_bracket_gap > options_.max_fc_gap_s ||
      navigation_bracket_gap > options_.max_fc_gap_s) {
    last_rejection_ = "fc_gauge_terminal_fc_gap";
    return false;
  }

  std::vector<const FCNavigationSample *> fc_window;
  std::vector<const BoardImuSample *> imu_window;
  const double fc_window_begin =
      std::min(start_attitude_time, start_navigation_time) -
      options_.max_fc_gap_s;
  const double fc_window_end =
      std::max(attitude_time, navigation_time) + options_.max_fc_gap_s;
  for (const auto &sample : fc_buffer_) {
    if (sample.timestamp >= fc_window_begin &&
        sample.timestamp <= fc_window_end)
      fc_window.push_back(&sample);
  }
  for (const auto &sample : imu_buffer_) {
    if (sample.timestamp >= window_start_board &&
        sample.timestamp <= board_time)
      imu_window.push_back(&sample);
  }
  const double fc_gap = max_gap(fc_window);
  const double imu_gap = max_gap(imu_window);
  if (!(fc_gap <= options_.max_fc_gap_s) ||
      !(imu_gap <= options_.max_imu_gap_s)) {
    last_diagnostics_.max_fc_gap_s = fc_gap;
    last_diagnostics_.max_imu_gap_s = imu_gap;
    last_rejection_ = !(fc_gap <= options_.max_fc_gap_s)
                          ? "fc_gauge_window_fc_gap"
                          : "fc_gauge_window_imu_gap";
    return false;
  }

  const std::vector<FCRateSample> fc_rates = make_fc_rates(fc_window);
  std::vector<Eigen::Vector3d> raw_rate_residuals;
  std::vector<double> paired_board_times;
  double angular_excitation = 0.0;
  for (const auto &rate : fc_rates) {
    const double rate_board_time =
        rate.timestamp + options_.fc_attitude_to_board_time_offset_s;
    if (rate_board_time < window_start_board - 1.0e-9 ||
        rate_board_time > board_time + 1.0e-9)
      continue;
    BoardImuSample imu;
    if (!interpolate_imu(imu_buffer_, rate_board_time, imu))
      continue;
    raw_rate_residuals.push_back(
        imu.angular_velocity - options_.R_FtoI_declared * rate.omega_F);
    paired_board_times.push_back(rate_board_time);
    angular_excitation =
        std::max(angular_excitation, rate.omega_F.norm());
  }
  if (raw_rate_residuals.size() < 2) {
    last_rejection_ = "fc_gauge_insufficient_rate_pairs";
    return false;
  }
  double paired_gap = 0.0;
  for (std::size_t index = 1; index < paired_board_times.size(); ++index)
    paired_gap = std::max(
        paired_gap, paired_board_times[index] - paired_board_times[index - 1]);
  if (paired_board_times.front() >
          window_start_board + options_.max_fc_gap_s ||
      board_time - paired_board_times.back() > options_.max_fc_gap_s ||
      paired_gap > options_.max_fc_gap_s) {
    last_rejection_ = "fc_gauge_rate_pair_duration_or_gap";
    return false;
  }
  const Eigen::Vector3d rate_bias = component_median(raw_rate_residuals);
  double rate_squared_error = 0.0;
  for (const auto &residual : raw_rate_residuals)
    rate_squared_error += (residual - rate_bias).squaredNorm();
  const double rate_residual_rms = std::sqrt(
      rate_squared_error / static_cast<double>(raw_rate_residuals.size()));
  if (!std::isfinite(rate_residual_rms) ||
      rate_residual_rms > options_.max_rate_residual_rms_rad_s) {
    last_diagnostics_.rate_residual_rms_rad_s = rate_residual_rms;
    last_rejection_ = "fc_gauge_locked_mount_rate_residual";
    return false;
  }

  Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
  if (!fc_angular_rate_at(fc_buffer_, attitude_time, options_.max_fc_gap_s,
                          omega_F)) {
    last_rejection_ = "fc_gauge_terminal_angular_rate_unavailable";
    return false;
  }
  const Eigen::Matrix3d R_GtoF =
      ov_core::quat_2_Rot(fc_attitude.q_GtoF);
  const Eigen::Matrix3d R_GtoI =
      options_.R_FtoI_declared * R_GtoF;
  result.timestamp = supported_camera_time;
  result.q_GtoI = ov_core::rot_2_quat(R_GtoI);
  result.p_IinG =
      fc_navigation.position_G + R_GtoF.transpose() * options_.p_IinF;
  result.v_IinG =
      fc_navigation.velocity_G +
      R_GtoF.transpose() * omega_F.cross(options_.p_IinF);
  result.R_FtoI_nominal = options_.R_FtoI_declared;
  result.R_mount_residual = Eigen::Matrix3d::Identity();
  result.mount_covariance = Eigen::Matrix3d::Zero();
  result.fc_to_board_time_offset_s =
      options_.fc_attitude_to_board_time_offset_s;
  result.fc_attitude_to_board_time_offset_s =
      options_.fc_attitude_to_board_time_offset_s;
  result.fc_navigation_to_board_time_offset_s =
      options_.fc_navigation_to_board_time_offset_s;
  result.camera_to_imu_time_offset_s =
      options_.camera_to_imu_time_offset_s;
  result.release_policy = options_.release_policy;
  result.released_to_openvins = false;
  result.readiness = AlignmentReadiness::NOT_READY;
  result.covariance.setZero();
  const double attitude_sigma_rad =
      options_.fc_attitude_sigma_deg * kPi / 180.0;
  result.covariance.block<3, 3>(0, 0).diagonal().array() =
      attitude_sigma_rad * attitude_sigma_rad;
  result.covariance.block<3, 3>(3, 3).diagonal().array() =
      options_.fc_position_sigma_m * options_.fc_position_sigma_m;
  result.covariance.block<3, 3>(6, 6).diagonal().array() =
      options_.fc_velocity_sigma_mps * options_.fc_velocity_sigma_mps;
  result.covariance.block<3, 3>(9, 9).diagonal().array() = 1.0e6;
  result.covariance.block<3, 3>(12, 12).diagonal().array() = 1.0e6;

  auto &diagnostics = last_diagnostics_;
  diagnostics.collection_start_time = collection_start_time_;
  diagnostics.window_start =
      window_start_board - options_.camera_to_imu_time_offset_s;
  diagnostics.init_time = supported_camera_time;
  diagnostics.initialization_duration_s =
      options_.sliding_window_duration_s;
  diagnostics.fc_samples = static_cast<int>(fc_window.size());
  diagnostics.imu_samples = static_cast<int>(imu_window.size());
  diagnostics.max_fc_gap_s = fc_gap;
  diagnostics.max_imu_gap_s = imu_gap;
  diagnostics.angular_excitation_rad_s = angular_excitation;
  diagnostics.rate_residual_rms_rad_s = rate_residual_rms;
  diagnostics.estimated_fc_to_board_time_offset_s =
      options_.fc_attitude_to_board_time_offset_s;
  diagnostics.navigation_quality_passed = true;
  diagnostics.quality_passed = true;
  diagnostics.readiness = AlignmentReadiness::NOT_READY;
  diagnostics.readiness_reason =
      "causal_fc_target_ready_for_metric_vio_yaw_translation_window";
  diagnostics.gauge_window_quality_passed = true;
  copy_runtime_counters(diagnostics);
  result.diagnostics = diagnostics;
  last_rejection_.clear();
  transition(AlignmentPhase::VALIDATING, supported_camera_time,
             "causal_fc_gauge_target_ready");
  return true;
}

bool OnlineAlignmentInitializer::commit_shadow_gauge_release(
    const AlignmentResult &result) {
  if ((!options_.sliding_window_shadow_only &&
       !options_.upstream_dynamic_init_fc_gauge) ||
      alignment_window_closed_ ||
      navigation_released_ || !result.released_to_openvins ||
      !result.diagnostics.quality_passed ||
      result.readiness == AlignmentReadiness::NOT_READY)
    return false;

  const double decision_time =
      std::isfinite(result.diagnostics.decision_time)
          ? result.diagnostics.decision_time
          : result.timestamp;
  navigation_released_ = true;
  ++successful_release_count_;
  navigation_ready_time_ = decision_time;
  alignment_window_closed_ = true;
  alignment_window_close_time_ = decision_time;
  candidate_active_ = false;
  candidate_visual_snapshots_.clear();
  transition(AlignmentPhase::NAVIGATION_READY, decision_time,
             "sliding_window_gauge_stability_release");

  latest_shadow_window_result_ = result;
  latest_shadow_window_result_valid_ = true;
  last_diagnostics_ = result.diagnostics;
  last_diagnostics_.navigation_ready_time = navigation_ready_time_;
  last_diagnostics_.alignment_window_closed = true;
  last_diagnostics_.alignment_window_close_time =
      alignment_window_close_time_;
  copy_runtime_counters(last_diagnostics_);
  last_diagnostics_.state_transitions = transitions_;
  last_rejection_.clear();

  if (!attempt_receipts_.empty()) {
    attempt_receipts_.back().outcome = "gauge_stability_release";
    attempt_receipts_.back().failed_gate.clear();
    attempt_receipts_.back().next_eligible_condition =
        "closed_after_release";
  }
  fc_buffer_.clear();
  imu_buffer_.clear();
  stereo_buffer_.clear();
  return true;
}

bool OnlineAlignmentInitializer::try_initialize(double now,
                                                 AlignmentResult &result) {
  if (fatal_configuration_error_) {
    last_rejection_ = "fatal_configuration_error:" + last_rejection_;
    return false;
  }
  if (alignment_window_closed_) {
    if (navigation_released_)
      ++post_release_try_count_;
    copy_runtime_counters(last_diagnostics_);
    last_rejection_ = navigation_released_
                          ? "alignment_already_released"
                          : "alignment_window_closed_explicit_reset_required";
    return false;
  }
  if (candidate_active_ && !options_.sliding_window_shadow_only) {
    if (options_.formal_causal_lifecycle)
      return validate_formal_candidate(now, result);
    return validate_candidate(now, result);
  }
  const auto wall_start = std::chrono::steady_clock::now();
  last_diagnostics_ = OnlineAlignmentDiagnostics();
  last_diagnostics_.solve_time = now;
  last_diagnostics_.future_data_used = false;
  last_diagnostics_.post_alignment_used = false;
  last_diagnostics_.retry_count = retry_count_;
  last_diagnostics_.provenance = options_.provenance;
  last_diagnostics_.collection_start_time = collection_start_time_;
  last_diagnostics_.navigation_ready_time = navigation_ready_time_;
  last_diagnostics_.full_alignment_ready_time = full_alignment_ready_time_;
  last_diagnostics_.decision_time = navigation_ready_time_;
  last_diagnostics_.release_policy = options_.release_policy;
  last_diagnostics_.sliding_overlap_stability_required_s =
      options_.sliding_window_overlap_stability_duration_s;
  last_diagnostics_.direct_sliding_state_release =
      options_.sliding_window_direct_state_release;
  copy_runtime_counters(last_diagnostics_);
  if (!std::isfinite(now)) {
    fail_retry(now, "solve_time_non_finite");
    return false;
  }
  const double decision_board_horizon =
      now + options_.camera_to_imu_time_offset_s;
  const double earliest_fc_offset =
      std::min(options_.fc_attitude_to_board_time_offset_s,
               options_.fc_navigation_to_board_time_offset_s);
  std::deque<FCNavigationSample> causal_fc_buffer;
  std::deque<BoardImuSample> causal_imu_buffer;
  for (const auto &sample : fc_buffer_)
    if (sample.timestamp <= decision_board_horizon - earliest_fc_offset + 1e-9)
      causal_fc_buffer.push_back(sample);
  for (const auto &sample : imu_buffer_)
    if (sample.timestamp <= decision_board_horizon + 1e-9)
      causal_imu_buffer.push_back(sample);

  if (!options_.visual_factors_enabled &&
      !options_.navigation_allow_without_visual) {
    fail_retry(now, "visual_factors_disabled_negative_control");
    return false;
  }
  if (stereo_buffer_.empty()) {
    if (causal_fc_buffer.size() >= static_cast<size_t>(options_.min_fc_samples) &&
        causal_imu_buffer.size() >= static_cast<size_t>(options_.min_imu_samples))
      fail_retry(now, "no_visual_fail_closed");
    else
      transition(AlignmentPhase::COLLECTING, now, "waiting_for_stereo");
    return false;
  }

  double init_time = -1.0;
  for (auto it = stereo_buffer_.rbegin(); it != stereo_buffer_.rend(); ++it) {
    if (it->left_timestamp > now)
      continue;
    const double board_time =
        it->left_timestamp + options_.camera_to_imu_time_offset_s;
    FCNavigationSample fc_attitude;
    FCNavigationSample fc_navigation;
    BoardImuSample imu_state;
    if (interpolate_fc(
            causal_fc_buffer,
            board_time - options_.fc_attitude_to_board_time_offset_s,
            fc_attitude) &&
        interpolate_fc(
            causal_fc_buffer,
            board_time - options_.fc_navigation_to_board_time_offset_s,
            fc_navigation) &&
        interpolate_imu(causal_imu_buffer, board_time, imu_state)) {
      init_time = it->left_timestamp;
      break;
    }
  }
  if (init_time < 0.0) {
    transition(AlignmentPhase::COLLECTING, now,
               "common_three_stream_horizon_unavailable");
    last_rejection_ = "common_three_stream_horizon_unavailable";
    return false;
  }

  auto &diag = last_diagnostics_;
  diag.init_time = init_time;
  std::vector<const FCNavigationSample *> fc;
  std::vector<const BoardImuSample *> imu;
  std::vector<const StereoAlignmentFrame *> stereo;
  double window_start = -1.0;
  std::vector<std::string> longest_window_gates;
  for (double duration : options_.candidate_window_durations_s) {
    diag.assessed_window_durations_s.push_back(duration);
    const double candidate_start = init_time - duration;
    std::vector<const FCNavigationSample *> candidate_fc;
    std::vector<const BoardImuSample *> candidate_imu;
    std::vector<const StereoAlignmentFrame *> candidate_stereo;
    for (const auto &sample : causal_fc_buffer)
      if (sample.timestamp >=
              candidate_start + options_.camera_to_imu_time_offset_s -
                  options_.max_time_offset_s - options_.max_fc_gap_s &&
          sample.timestamp <=
              init_time + options_.camera_to_imu_time_offset_s -
                  earliest_fc_offset + 1e-9)
        candidate_fc.push_back(&sample);
    for (const auto &sample : causal_imu_buffer)
      if (sample.timestamp >=
              candidate_start + options_.camera_to_imu_time_offset_s &&
          sample.timestamp <= init_time + options_.camera_to_imu_time_offset_s &&
          sample.timestamp <= decision_board_horizon)
        candidate_imu.push_back(&sample);
    for (const auto &frame : stereo_buffer_)
      if (frame.left_timestamp >= candidate_start &&
          frame.left_timestamp <= init_time)
        candidate_stereo.push_back(&frame);

    std::set<size_t> candidate_features;
    int candidate_depths = 0;
    for (const auto *frame : candidate_stereo)
      for (const auto &observation : frame->observations) {
        if (observation.left_valid)
          candidate_features.insert(observation.feature_id);
        if (observation.left_valid && observation.right_valid &&
            std::isfinite(observation.depth_m) && observation.depth_m > 0.0)
          ++candidate_depths;
      }
    const double candidate_fc_gap = max_gap(candidate_fc);
    const double candidate_imu_gap = max_gap(candidate_imu);
    const double candidate_visual_gap = stereo_max_gap(candidate_stereo);
    std::vector<std::string> gates;
    if (candidate_fc.empty() ||
        candidate_fc.front()->timestamp >
            candidate_start + options_.max_fc_gap_s)
      gates.push_back("window_fc_duration");
    if (candidate_imu.empty() ||
        candidate_imu.front()->timestamp >
            candidate_start + options_.camera_to_imu_time_offset_s +
                options_.max_imu_gap_s)
      gates.push_back("window_imu_duration");
    if (candidate_stereo.empty() ||
        candidate_stereo.front()->left_timestamp >
            candidate_start + options_.max_stereo_gap_s)
      gates.push_back("window_visual_duration");
    if (static_cast<int>(candidate_fc.size()) < options_.min_fc_samples)
      gates.push_back("fc_sample_count");
    if (static_cast<int>(candidate_imu.size()) < options_.min_imu_samples)
      gates.push_back("imu_sample_count");
    if (static_cast<int>(candidate_stereo.size()) < options_.min_stereo_frames)
      gates.push_back("visual_frame_count");
    if (static_cast<int>(candidate_features.size()) < options_.min_feature_tracks)
      gates.push_back("feature_track_count");
    if (!(candidate_fc_gap <= options_.max_fc_gap_s))
      gates.push_back("fc_gap");
    if (!(candidate_imu_gap <= options_.max_imu_gap_s))
      gates.push_back("imu_gap");
    if (!(candidate_visual_gap <= options_.max_stereo_gap_s))
      gates.push_back("visual_gap");
    longest_window_gates = gates;
    if (!gates.empty())
      continue;

    window_start = candidate_start;
    fc.swap(candidate_fc);
    imu.swap(candidate_imu);
    stereo.swap(candidate_stereo);
    diag.selected_window_duration_s = duration;
    diag.window_start = candidate_start;
    diag.fc_samples = static_cast<int>(fc.size());
    diag.imu_samples = static_cast<int>(imu.size());
    diag.stereo_frames = static_cast<int>(stereo.size());
    diag.feature_tracks = static_cast<int>(candidate_features.size());
    diag.stereo_depths = candidate_depths;
    diag.max_fc_gap_s = candidate_fc_gap;
    diag.max_imu_gap_s = candidate_imu_gap;
    diag.max_stereo_gap_s = candidate_visual_gap;
    break;
  }
  if (window_start < 0.0) {
    fail_retry(now, longest_window_gates.empty()
                        ? "no_usable_candidate_window"
                        : join(longest_window_gates));
    return false;
  }

  std::set<size_t> fingerprint_features;
  for (const auto *frame : stereo)
    for (const auto &observation : frame->observations)
      if (observation.left_valid)
        fingerprint_features.insert(observation.feature_id);
  size_t feature_hash = 1469598103934665603ULL;
  for (size_t feature_id : fingerprint_features)
    feature_hash = (feature_hash ^ feature_id) * 1099511628211ULL;
  const std::vector<FCRateSample> fc_rates = make_fc_rates(fc);
  double angular_excitation_proxy = 0.0;
  double acceleration_excitation_proxy = 0.0;
  for (const auto &rate : fc_rates) {
    angular_excitation_proxy =
        std::max(angular_excitation_proxy, rate.omega_F.norm());
    acceleration_excitation_proxy =
        std::max(acceleration_excitation_proxy, rate.acceleration_G.norm());
  }
  size_t frame_time_hash = 1469598103934665603ULL;
  for (const auto *frame : stereo) {
    const size_t ticks = static_cast<size_t>(
        std::llround(frame->left_timestamp * 1000000.0));
    frame_time_hash = (frame_time_hash ^ ticks) * 1099511628211ULL;
  }
  std::ostringstream fingerprint_stream;
  fingerprint_stream << std::fixed << std::setprecision(6) << init_time << ':'
                     << diag.selected_window_duration_s << ':'
                     << stereo.front()->left_timestamp << ':' << stereo.size()
                     << ':' << frame_time_hash << ':' << feature_hash << ':'
                     << fc.front()->timestamp << ':' << fc.back()->timestamp
                     << ':' << imu.front()->timestamp << ':'
                     << imu.back()->timestamp << ':' << angular_excitation_proxy
                     << ':' << acceleration_excitation_proxy << ':'
                     << diag.fc_samples << ':' << diag.imu_samples << ':'
                     << diag.feature_tracks;
  diag.window_fingerprint = fingerprint_stream.str();
  if (last_sliding_window_end_time_ >= 0.0 &&
      init_time - last_sliding_window_end_time_ + 1.0e-9 <
          options_.sliding_window_min_advance_s) {
    ++solve_eligibility_skip_count_;
    last_rejection_ = force_refinement_solve_
                          ? "refinement_waiting_for_new_window_time_advance"
                          : "sliding_window_waiting_for_time_advance";
    copy_runtime_counters(diag);
    return false;
  }
  if (diag.window_fingerprint == last_window_fingerprint_) {
    ++duplicate_window_skip_count_;
    last_rejection_ = "duplicate_window_fingerprint";
    copy_runtime_counters(diag);
    return false;
  }

  // The startup window is allowed to estimate gyro bias, but never the fixed
  // FC-to-board rotation or either time mapping. Those values come from the
  // accepted full-flight calibration contract.
  const double best_offset = options_.fc_attitude_to_board_time_offset_s;
  std::vector<Eigen::Vector3d> paired_fc;
  std::vector<Eigen::Vector3d> paired_imu;
  for (const auto &rate : fc_rates) {
    BoardImuSample sample;
    if (interpolate_imu(causal_imu_buffer, rate.timestamp + best_offset,
                        sample) &&
        rate.omega_F.norm() <= options_.max_angular_rate_rad_s &&
        sample.angular_velocity.norm() <= options_.max_angular_rate_rad_s) {
      paired_fc.push_back(rate.omega_F);
      paired_imu.push_back(sample.angular_velocity);
    }
  }
  RotationFit best_fit = evaluate_locked_rotation(
      paired_fc, paired_imu, options_.R_FtoI_declared);
  const bool mount_time_seed_available = best_fit.valid;
  diag.estimated_fc_to_board_time_offset_s = best_offset;
  diag.rate_residual_rms_rad_s = best_fit.rms;
  diag.angular_excitation_rad_s = best_fit.excitation;
  diag.second_axis_ratio = best_fit.second_axis_ratio;
  diag.time_offset_sigma_s =
      options_.fc_attitude_to_board_time_offset_sigma_s;
  const bool time_offset_observable = options_.fc_board_calibration_locked;
  diag.mount_residual_deg = 0.0;
  if (!mount_time_seed_available)
    diag.full_alignment_failed_gates.push_back("mount_time_seed_unavailable");
  if (!time_offset_observable)
    diag.full_alignment_failed_gates.push_back(
        "startup_time_offset_not_observable");
  // The FC/board rate residual remains visible for evaluation and the robust
  // startup gyro-bias seed. It is not a release gate: a short Velcro flex
  // transient must not invalidate an independently accepted full-flight
  // mounting/time calibration.

  const bool mount_fixed_external_calibration = true;
  if (!mount_time_seed_available || !time_offset_observable) {
    fail_retry(now, join(diag.full_alignment_failed_gates));
    return false;
  }
  diag.startup_mount_fixed_to_prior = false;
  diag.startup_time_offset_fixed_to_prior = false;
  diag.time_offset_status =
      EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION;
  diag.startup_misalignment_status =
      EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION;

  // Reject high-rate intervals before selecting the immutable graph snapshot.
  std::vector<const StereoAlignmentFrame *> usable_frames;
  rejected_intervals_.clear();
  const StereoAlignmentFrame *previous_frame = nullptr;
  for (const auto *frame : stereo) {
    if (previous_frame == nullptr) {
      usable_frames.push_back(frame);
      previous_frame = frame;
      continue;
    }
    const double t0 = previous_frame->left_timestamp +
                      options_.camera_to_imu_time_offset_s;
    const double t1 = frame->left_timestamp +
                      options_.camera_to_imu_time_offset_s;
    double peak_rate = 0.0;
    for (const auto &sample : causal_imu_buffer)
      if (sample.timestamp >= t0 && sample.timestamp <= t1)
        peak_rate = std::max(peak_rate, sample.angular_velocity.norm());
    if (peak_rate > options_.max_angular_rate_rad_s) {
      RejectedAlignmentInterval rejected;
      rejected.start_time = previous_frame->left_timestamp;
      rejected.end_time = frame->left_timestamp;
      rejected.reason = "high_angular_rate";
      rejected.peak_angular_rate_rad_s = peak_rate;
      rejected_intervals_.push_back(rejected);
      previous_frame = frame;
      continue;
    }
    usable_frames.push_back(frame);
    previous_frame = frame;
  }
  if (usable_frames.size() < static_cast<size_t>(options_.min_keyframes)) {
    fail_retry(now, "insufficient_keyframes_after_interval_rejection");
    return false;
  }
  if (usable_frames.size() > static_cast<size_t>(options_.max_keyframes)) {
    std::vector<const StereoAlignmentFrame *> downsampled;
    for (int i = 0; i < options_.max_keyframes; ++i) {
      const size_t index = static_cast<size_t>(std::llround(
          static_cast<double>(i) * (usable_frames.size() - 1) /
          static_cast<double>(options_.max_keyframes - 1)));
      if (downsampled.empty() || downsampled.back() != usable_frames[index])
        downsampled.push_back(usable_frames[index]);
    }
    usable_frames.swap(downsampled);
  }
  diag.keyframes = static_cast<int>(usable_frames.size());

  const bool refinement_attempt = force_refinement_solve_;
  const double seconds_since_last_attempt =
      last_solve_stream_time_ >= 0.0
          ? std::max(0.0, now - last_solve_stream_time_)
          : std::numeric_limits<double>::infinity();
  last_solve_stream_time_ = now;
  last_sliding_window_end_time_ = init_time;
  selected_frames_at_last_solve_ = static_cast<int>(stereo_buffer_.size());
  selected_frame_serial_at_last_solve_ = selected_frame_serial_;
  last_window_fingerprint_ = diag.window_fingerprint;
  force_refinement_solve_ = false;
  diag.nonlinear_solve_attempt_count = nonlinear_solve_attempt_count_;
  AlignmentAttemptReceipt receipt;
  receipt.window_id = ++sliding_window_id_;
  receipt.optimizer_invocation_index = 0;
  receipt.attempt_timestamp = now;
  if (options_.sliding_window_shadow_only) {
    std::ostringstream trigger;
    trigger << "fixed_time_window_advanced;window_end_s=" << init_time;
    receipt.trigger = trigger.str();
  } else if (refinement_attempt) {
    receipt.trigger = "single_candidate_refinement";
  } else {
    std::ostringstream trigger;
    trigger << "fixed_time_window_advanced;attempt_interval_s="
            << seconds_since_last_attempt
            << ";window_fingerprint_changed=true";
    receipt.trigger = trigger.str();
  }
  receipt.window_fingerprint = diag.window_fingerprint;
  receipt.window_duration_s = diag.selected_window_duration_s;
  receipt.window_begin_timestamp = diag.window_start;
  receipt.window_end_timestamp = diag.init_time;
  receipt.raw_fc_count = static_cast<int>(fc.size());
  receipt.valid_fc_count = static_cast<int>(std::count_if(
      fc.begin(), fc.end(), [](const FCNavigationSample *sample) {
        return sample != nullptr && sample->status_valid &&
               sample->attitude_valid &&
               (sample->position_valid || sample->velocity_valid);
      }));
  receipt.imu_count = static_cast<int>(imu.size());
  receipt.visual_frame_count = static_cast<int>(stereo.size());
  receipt.selected_keyframe_count = static_cast<int>(usable_frames.size());
  for (const auto *frame : usable_frames)
    receipt.selected_frame_timestamps.push_back(frame->left_timestamp);
  receipt.selected_tracks = diag.feature_tracks;
  receipt.outcome = "solving";
  receipt.next_eligible_condition = "candidate_or_new_window_information";
  attempt_receipts_.push_back(receipt);
  // A difficult flight can legitimately require more than 128 advanced
  // windows before release. Preserve the complete causal trace for formal
  // validation instead of silently discarding its beginning.
  while (attempt_receipts_.size() > 2048)
    attempt_receipts_.pop_front();
  transition(AlignmentPhase::ALIGNING, now, "immutable_causal_window_ready");

  if (!attempt_receipts_.empty() &&
      attempt_receipts_.back().window_id == sliding_window_id_) {
    attempt_receipts_.back().bg_prior_center = Eigen::Vector3d::Zero();
    attempt_receipts_.back().ba_prior_center = Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d bg_seed = best_fit.bg;
  Eigen::Vector3d ba_seed = Eigen::Vector3d::Zero();
  if (!previous_window_states_.empty()) {
    bg_seed = previous_window_states_.back().bg;
    ba_seed = previous_window_states_.back().ba;
  }

  std::vector<GraphState> states;
  states.reserve(usable_frames.size());
  std::vector<FCNavigationSample> fc_attitude_at_state;
  std::vector<FCNavigationSample> fc_navigation_at_state;
  fc_attitude_at_state.reserve(usable_frames.size());
  fc_navigation_at_state.reserve(usable_frames.size());
  bool window_warm_start_used = false;
  for (const auto *frame : usable_frames) {
    const double board_time = frame->left_timestamp +
                              options_.camera_to_imu_time_offset_s;
    const double fc_attitude_query_time = board_time - best_offset;
    const double fc_navigation_query_time =
        board_time - options_.fc_navigation_to_board_time_offset_s;
    FCNavigationSample fc_attitude_state;
    FCNavigationSample fc_navigation_state;
    if (!interpolate_fc(causal_fc_buffer, fc_attitude_query_time,
                        fc_attitude_state) ||
        !interpolate_fc(causal_fc_buffer, fc_navigation_query_time,
                        fc_navigation_state)) {
      fail_retry(now, "keyframe_fc_interpolation");
      return false;
    }
    const Eigen::Matrix3d R_GtoF =
        ov_core::quat_2_Rot(fc_attitude_state.q_GtoF);
    const Eigen::Matrix3d R_GtoI = best_fit.R_FtoI * R_GtoF;
    const Eigen::Matrix3d R_FtoG = R_GtoF.transpose();
    Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
    if (!fc_rates.empty()) {
      const auto nearest = std::min_element(
          fc_rates.begin(), fc_rates.end(),
          [&](const FCRateSample &a, const FCRateSample &b) {
            return std::fabs(a.timestamp - fc_attitude_query_time) <
                   std::fabs(b.timestamp - fc_attitude_query_time);
          });
      omega_F = nearest->omega_F;
    }
    GraphState state;
    state.camera_time = frame->left_timestamp;
    assign(state.q, ov_core::rot_2_quat(R_GtoI));
    assign(state.p,
           fc_navigation_state.position_G + R_FtoG * options_.p_IinF);
    assign(state.v,
           fc_navigation_state.velocity_G +
               R_FtoG * omega_F.cross(options_.p_IinF));
    assign(state.bg, bg_seed);
    assign(state.ba, ba_seed);
    {
      SlidingWindowStateEstimate warm_state;
      if (interpolate_previous_window_state(frame->left_timestamp,
                                            warm_state)) {
        assign(state.q, warm_state.q_GtoI);
        assign(state.p, warm_state.p_IinG);
        assign(state.v, warm_state.v_IinG);
        assign(state.bg, warm_state.bg);
        assign(state.ba, warm_state.ba);
        window_warm_start_used = true;
      } else if (!previous_window_states_.empty()) {
        // A new tail state has no previous-window timestamp support. Its
        // q/p/v retain the current FC seed; the complete CPI chain refines the
        // one shared window bg/ba pair declared below.
        window_warm_start_used = true;
      }
    }
    states.push_back(state);
    fc_attitude_at_state.push_back(fc_attitude_state);
    fc_navigation_at_state.push_back(fc_navigation_state);
  }

  std::array<double, 3> shared_bg;
  std::array<double, 3> shared_ba;
  assign(shared_bg, bg_seed);
  assign(shared_ba, ba_seed);

  ceres::Problem problem;
  FactorFamilies families;
  for (auto &state : states) {
    problem.AddParameterBlock(state.q.data(), 4,
                              new ov_init::State_JPLQuatLocal());
    problem.AddParameterBlock(state.p.data(), 3);
    problem.AddParameterBlock(state.v.data(), 3);
  }
  problem.AddParameterBlock(shared_bg.data(), 3);
  problem.AddParameterBlock(shared_ba.data(), 3);
  std::array<double, 4> mount_q;
  assign(mount_q, ov_core::rot_2_quat(best_fit.R_FtoI));
  problem.AddParameterBlock(mount_q.data(), 4,
                            new ov_init::State_JPLQuatLocal());
  problem.SetParameterBlockConstant(mount_q.data());

  auto *mount_prior = new QuaternionPriorCost(
      new QuaternionPriorFunctor(
          options_.R_FtoI_declared,
          options_.mount_prior_sigma_deg * kPi / 180.0));
  families.prior.push_back(
      problem.AddResidualBlock(mount_prior, nullptr, mount_q.data()));
  auto *bg_prior = new VectorPriorCost(new VectorPriorFunctor(
      Eigen::Vector3d::Zero(), options_.gyro_bias_prior_sigma_rad_s));
  auto *ba_prior = new VectorPriorCost(new VectorPriorFunctor(
      Eigen::Vector3d::Zero(), options_.accel_bias_prior_sigma_mps2));
  families.prior.push_back(problem.AddResidualBlock(
      bg_prior, nullptr, shared_bg.data()));
  families.prior.push_back(problem.AddResidualBlock(
      ba_prior, nullptr, shared_ba.data()));

  // Preserve the synchronized terminal navigation row with its full declared
  // uncertainty. Earlier FC rows are outputs of the same navigation filter and
  // must not be counted as independent absolute measurements. They constrain
  // only trajectory increments. Increment covariance is proportional to dt,
  // so splitting one physical interval into more keyframes does not increase
  // the total FC trajectory information. This prevents the old normalized
  // window-average model from diluting the state that is actually released.
  const double fc_window_span_s =
      states.back().camera_time - states.front().camera_time;
  if (!(fc_window_span_s > 1.0e-9)) {
    fail_retry(now, "fc_increment_span");
    return false;
  }
  std::vector<Eigen::Matrix3d> fc_target_rotations;
  std::vector<Eigen::Vector3d> fc_target_positions;
  std::vector<Eigen::Vector3d> fc_target_velocities;
  std::vector<Eigen::Vector3d> fc_target_rates;
  fc_target_rotations.reserve(states.size());
  fc_target_positions.reserve(states.size());
  fc_target_velocities.reserve(states.size());
  fc_target_rates.reserve(states.size());
  for (size_t i = 0; i < states.size(); ++i) {
    const Eigen::Matrix3d R_GtoF =
        ov_core::quat_2_Rot(fc_attitude_at_state[i].q_GtoF);
    const Eigen::Matrix3d R_FtoG = R_GtoF.transpose();
    Eigen::Vector3d omega_F = Eigen::Vector3d::Zero();
    if (!fc_rates.empty()) {
      const auto nearest = std::min_element(
          fc_rates.begin(), fc_rates.end(),
          [&](const FCRateSample &a, const FCRateSample &b) {
            return std::fabs(a.timestamp -
                             fc_attitude_at_state[i].timestamp) <
                   std::fabs(b.timestamp -
                             fc_attitude_at_state[i].timestamp);
          });
      omega_F = nearest->omega_F;
    }
    fc_target_rotations.push_back(best_fit.R_FtoI * R_GtoF);
    fc_target_positions.push_back(
        fc_navigation_at_state[i].position_G +
        R_FtoG * options_.p_IinF);
    fc_target_velocities.push_back(
        fc_navigation_at_state[i].velocity_G +
        R_FtoG * omega_F.cross(options_.p_IinF));
    fc_target_rates.push_back(omega_F);
  }

  const double terminal_board_time =
      states.back().camera_time + options_.camera_to_imu_time_offset_s;
  BoardImuSample terminal_imu;
  double terminal_rate_residual = std::numeric_limits<double>::infinity();
  if (interpolate_imu(causal_imu_buffer, terminal_board_time, terminal_imu)) {
    terminal_rate_residual =
        (terminal_imu.angular_velocity -
         best_fit.R_FtoI * fc_target_rates.back() - best_fit.bg)
            .norm();
  }

  std::vector<double> fc_state_timestamps;
  std::vector<double *> fc_parameter_blocks;
  fc_state_timestamps.reserve(states.size());
  fc_parameter_blocks.reserve(3 * states.size());
  for (GraphState &state : states) {
    fc_state_timestamps.push_back(state.camera_time);
    fc_parameter_blocks.push_back(state.q.data());
    fc_parameter_blocks.push_back(state.p.data());
    fc_parameter_blocks.push_back(state.v.data());
  }
  Eigen::Matrix<double, 9, 9> fc_terminal_covariance =
      Eigen::Matrix<double, 9, 9>::Zero();
  const double attitude_sigma_rad =
      options_.fc_attitude_gauge_factor_enabled
          ? options_.fc_attitude_sigma_deg * kPi / 180.0
          : 1.0e6;
  fc_terminal_covariance.block<3, 3>(0, 0) =
      attitude_sigma_rad * attitude_sigma_rad * Eigen::Matrix3d::Identity();
  // P3's accepted mount and attitude-time mapping remain fixed optimization
  // inputs, but their declared uncertainties are not zero.  Isotropic mount
  // uncertainty enters the FC-implied attitude directly; time uncertainty is
  // propagated through the terminal angular rate. Navigation timing and the
  // lever arm have no separate uncertainty declarations in the current stream
  // contract and are therefore explicitly treated as fixed.
  const double mount_sigma_rad =
      options_.fc_board_mount_sigma_deg * kPi / 180.0;
  fc_terminal_covariance.block<3, 3>(0, 0) +=
      mount_sigma_rad * mount_sigma_rad * Eigen::Matrix3d::Identity();
  const Eigen::Vector3d terminal_omega_I =
      best_fit.R_FtoI * fc_target_rates.back();
  fc_terminal_covariance.block<3, 3>(0, 0) +=
      options_.fc_attitude_to_board_time_offset_sigma_s *
      options_.fc_attitude_to_board_time_offset_sigma_s *
      terminal_omega_I * terminal_omega_I.transpose();
  fc_terminal_covariance.block<3, 3>(3, 3) =
      options_.fc_position_sigma_m * options_.fc_position_sigma_m *
      Eigen::Matrix3d::Identity();
  fc_terminal_covariance.block<3, 3>(6, 6) =
      options_.fc_velocity_sigma_mps * options_.fc_velocity_sigma_mps *
      Eigen::Matrix3d::Identity();
  p4::Factor_P4FcTrajectory *fc_trajectory_factor = nullptr;
  try {
    fc_trajectory_factor = new p4::Factor_P4FcTrajectory(
        fc_state_timestamps, fc_target_rotations, fc_target_positions,
        fc_target_velocities, fc_terminal_covariance,
        options_.fc_process_variance_fraction);
  } catch (const std::exception &error) {
    fail_retry(now, std::string("fc_covariance_model:") + error.what());
    return false;
  }
  families.fc.push_back(problem.AddResidualBlock(
      fc_trajectory_factor, nullptr, fc_parameter_blocks));

  const Eigen::MatrixXd &fc_joint_covariance =
      fc_trajectory_factor->residual_covariance();
  double maximum_terminal_increment_correlation = 0.0;
  for (Eigen::Index row = 0; row < 9; ++row) {
    for (Eigen::Index column = 9; column < fc_joint_covariance.cols();
         ++column) {
      const double variance_product =
          fc_joint_covariance(row, row) *
          fc_joint_covariance(column, column);
      if (variance_product > 0.0)
        maximum_terminal_increment_correlation = std::max(
            maximum_terminal_increment_correlation,
            std::fabs(fc_joint_covariance(row, column)) /
                std::sqrt(variance_product));
    }
  }
  diag.fc_attitude_gauge_factor_enabled =
      options_.fc_attitude_gauge_factor_enabled;
  diag.fc_attitude_gauge_factor_count =
      options_.fc_attitude_gauge_factor_enabled ? 1 : 0;
  diag.fc_attitude_gauge_anchor_timestamp_s = states.back().camera_time;
  diag.fc_attitude_gauge_anchor_rate_rad_s = fc_target_rates.back().norm();
  diag.fc_attitude_gauge_anchor_rate_residual_rad_s =
      terminal_rate_residual;
  diag.fc_attitude_gauge_sigma_deg = options_.fc_attitude_sigma_deg;
  diag.fc_position_velocity_factor_count = static_cast<int>(states.size());
  diag.fc_position_velocity_time_weight_sum = 1.0;
  diag.fc_position_velocity_weight_model =
      "single_dense_terminal_plus_correlated_increments";
  diag.fc_error_state_transition_model =
      "e=[dtheta,dp,dv];Phi=I9;Qd=fraction*Pterminal*dt/window;"
      "Ptheta_includes_declared_mount_and_attitude_time_uncertainty;"
      "navigation_time_and_lever_arm_treated_fixed_without_declared_sigma";
  diag.fc_process_variance_fraction =
      options_.fc_process_variance_fraction;
  diag.fc_terminal_increment_max_correlation =
      maximum_terminal_increment_correlation;

  // P4 owns one bg/ba pair for the finite startup window.  The adapter ties
  // both endpoints of the complete upstream 15-D CPI factor to those shared
  // variables; it does not discard bias rows or covariance cross terms.
  for (size_t i = 1; i < states.size(); ++i) {
    const double t0 = states[i - 1].camera_time +
                      options_.camera_to_imu_time_offset_s;
    const double t1 = states[i].camera_time +
                      options_.camera_to_imu_time_offset_s;
    std::vector<BoardImuSample> segment;
    if (!interval_imu_samples(causal_imu_buffer, t0, t1, segment)) {
      fail_retry(now, "imu_preintegration_support");
      return false;
    }
    auto cpi = std::make_shared<ov_core::CpiV1>(
        options_.imu_sigma_w, options_.imu_sigma_wb, options_.imu_sigma_a,
        options_.imu_sigma_ab, true);
    Eigen::Vector3d bg_linearization = map3(shared_bg);
    Eigen::Vector3d ba_linearization = map3(shared_ba);
    cpi->setLinearizationPoints(bg_linearization, ba_linearization);
    for (size_t j = 1; j < segment.size(); ++j)
      cpi->feed_IMU(segment[j - 1].timestamp, segment[j].timestamp,
                    segment[j - 1].angular_velocity,
                    segment[j - 1].linear_acceleration,
                    segment[j].angular_velocity,
                    segment[j].linear_acceleration);
    cpi->P_meas += Eigen::Matrix<double, 15, 15>::Identity() * 1e-12;
    Eigen::Vector3d gravity = options_.gravity_G;
    auto *factor = new p4::Factor_P4ImuSharedBias(
        cpi->DT, gravity, cpi->alpha_tau, cpi->beta_tau, cpi->q_k2tau,
        cpi->b_a_lin, cpi->b_w_lin, cpi->J_q, cpi->J_b, cpi->J_a,
        cpi->H_b, cpi->H_a, cpi->P_meas);
    families.imu.push_back(problem.AddResidualBlock(
        factor, nullptr, states[i - 1].q.data(), shared_bg.data(),
        states[i - 1].v.data(), shared_ba.data(),
        states[i - 1].p.data(), states[i].q.data(), states[i].v.data(),
        states[i].p.data()));
  }

  struct FeatureTrackEntry {
    size_t state_index = 0;
    const StereoAlignmentObservation *observation = nullptr;
  };
  std::map<size_t, std::vector<FeatureTrackEntry>> tracks;
  for (size_t state_index = 0; state_index < usable_frames.size(); ++state_index)
    for (const auto &observation : usable_frames[state_index]->observations)
      if (observation.left_valid)
        tracks[observation.feature_id].push_back(
            {state_index, &observation});

  // Formal P4 does not retain nuisance landmarks.  Each selected feature
  // contributes exactly one maximum-time-baseline pair, so no pixel row is
  // silently counted multiple times inside this initialization likelihood.
  std::vector<std::array<double, 3>> landmarks;
  std::vector<size_t> landmark_feature_ids;
  int accepted_features = 0;
  int multi_frame_tracks = 0;
  int unique_visual_measurements = 0;
  const Eigen::Matrix3d R_ItoC =
      ov_core::quat_2_Rot(options_.q_ItoC[0]);
  const double average_focal = std::sqrt(std::max(
      1.0, options_.camera_intrinsics[0](0) *
               options_.camera_intrinsics[0](1)));
  const double normalized_visual_sigma =
      options_.visual_pixel_sigma / average_focal;
  for (const auto &track : tracks) {
    if (accepted_features >= options_.max_visual_features ||
        track.second.size() < 2)
      continue;
    ++multi_frame_tracks;
    const FeatureTrackEntry &previous = track.second.front();
    const FeatureTrackEntry &current = track.second.back();
    if (previous.observation == nullptr || current.observation == nullptr ||
        !previous.observation->normalized_left.allFinite() ||
        !current.observation->normalized_left.allFinite())
      continue;
    const GraphState &previous_state = states[previous.state_index];
    const GraphState &current_state = states[current.state_index];
    const Eigen::Matrix3d R_GtoI_previous = ov_core::quat_2_Rot(
        normalized_jpl(previous_state.q.data()));
    const Eigen::Matrix3d R_GtoI_current = ov_core::quat_2_Rot(
        normalized_jpl(current_state.q.data()));
    const Eigen::Vector3d previous_center_G =
        map3(previous_state.p) - R_GtoI_previous.transpose() *
                                     R_ItoC.transpose() * options_.p_IinC[0];
    const Eigen::Vector3d current_center_G =
        map3(current_state.p) - R_GtoI_current.transpose() *
                                    R_ItoC.transpose() * options_.p_IinC[0];
    const Eigen::Vector3d previous_bearing_C(
        previous.observation->normalized_left.x(),
        previous.observation->normalized_left.y(), 1.0);
    const Eigen::Vector3d current_bearing_C(
        current.observation->normalized_left.x(),
        current.observation->normalized_left.y(), 1.0);
    const Eigen::Vector3d previous_direction_G =
        (R_GtoI_previous.transpose() * R_ItoC.transpose() *
         previous_bearing_C)
            .normalized();
    const Eigen::Vector3d current_direction_G =
        (R_GtoI_current.transpose() * R_ItoC.transpose() *
         current_bearing_C)
            .normalized();
    const double baseline_m =
        (current_center_G - previous_center_G).norm();
    const double parallax_deg =
        std::acos(std::max(-1.0, std::min(
                                     1.0, previous_direction_G.dot(
                                              current_direction_G)))) *
        180.0 / kPi;
    if (!std::isfinite(baseline_m) || !std::isfinite(parallax_deg) ||
        baseline_m < options_.min_monocular_baseline_m ||
        parallax_deg < options_.min_monocular_parallax_deg)
      continue;
    if (options_.visual_factors_enabled) {
      auto *factor = new p4::Factor_P4Epipolar(
          previous.observation->normalized_left,
          current.observation->normalized_left, R_ItoC,
          options_.p_IinC[0], normalized_visual_sigma);
      families.visual.push_back(problem.AddResidualBlock(
          factor, new ceres::CauchyLoss(2.0),
          states[previous.state_index].q.data(),
          states[previous.state_index].p.data(),
          states[current.state_index].q.data(),
          states[current.state_index].p.data()));
    }
    unique_visual_measurements += 2;
    ++accepted_features;
  }
  diag.stereo_depths = accepted_features;
  diag.visual_statistics.tracked_feature_count =
      static_cast<int>(tracks.size());
  diag.visual_statistics.multi_frame_track_count = multi_frame_tracks;
  diag.visual_statistics.triangulated_landmark_count = 0;
  diag.visual_statistics.rejected_landmark_count =
      std::max(0, multi_frame_tracks - accepted_features);
  diag.visual_statistics.visual_factor_count =
      static_cast<int>(families.visual.size());
  diag.visual_pairing_policy =
      "one_maximum_time_baseline_pair_per_feature_no_pixel_reuse";
  diag.visual_unique_measurement_count = unique_visual_measurements;
  if (options_.visual_factors_enabled &&
      static_cast<int>(families.visual.size()) <
          options_.min_visual_residual_blocks) {
    fail_retry(now, "insufficient_visual_factors");
    return false;
  }

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_SCHUR;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.max_num_iterations = options_.solver_max_iterations;
  solver_options.max_solver_time_in_seconds = options_.solver_max_time_s;
  solver_options.num_threads = 1;
  solver_options.function_tolerance = 1e-7;
  solver_options.gradient_tolerance = 1e-10;
  diag.shared_window_bias_model = true;
  diag.staged_solver_enabled = false;
  ceres::Solver::Summary summary;
  ++nonlinear_solve_attempt_count_;
  diag.nonlinear_solve_attempt_count = nonlinear_solve_attempt_count_;
  const auto solve_wall_start = std::chrono::steady_clock::now();
  ceres::Solve(solver_options, &problem, &summary);
  const double ceres_solve_wall_time_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - solve_wall_start).count();
  diag.final_stage_solve_wall_time_s = ceres_solve_wall_time_s;
  if (!attempt_receipts_.empty() &&
      attempt_receipts_.back().outcome == "solving") {
    attempt_receipts_.back().factor_count =
        static_cast<int>(families.prior.size() + families.fc.size() +
                         families.imu.size() + families.visual.size());
    attempt_receipts_.back().selected_landmarks = 0;
    attempt_receipts_.back().selected_visual_pairs = accepted_features;
    attempt_receipts_.back().solve_wall_time_s = ceres_solve_wall_time_s;
    attempt_receipts_.back().optimizer_invocation_index =
        nonlinear_solve_attempt_count_;
    attempt_receipts_.back().optimizer_invocation_count = 1;
    attempt_receipts_.back().shared_window_bias_model = true;
    attempt_receipts_.back().staged_solver_enabled = false;
    attempt_receipts_.back().final_stage_solve_wall_time_s =
        ceres_solve_wall_time_s;
    attempt_receipts_.back().initial_cost = summary.initial_cost;
    attempt_receipts_.back().final_cost = summary.final_cost;
    attempt_receipts_.back().warm_start_used = window_warm_start_used;
  }
  diag.initial_cost = summary.initial_cost;
  diag.final_cost = summary.final_cost;
  diag.solver_iterations = static_cast<int>(summary.iterations.size());
  diag.solver_converged = summary.IsSolutionUsable();
  for (GraphState &state : states) {
    assign(state.bg, map3(shared_bg));
    assign(state.ba, map3(shared_ba));
  }
  transition(AlignmentPhase::VALIDATING, now, "joint_solver_finished");

  // Compare the two independently optimized trajectories at identical camera
  // timestamps in their overlap. Comparing adjacent window endpoints would
  // mix estimator disagreement with the aircraft's real motion.
  if (summary.IsSolutionUsable() && !previous_window_states_.empty()) {
    double attitude_max_deg = 0.0;
    double position_max_m = 0.0;
    double velocity_max_mps = 0.0;
    double gyro_bias_max_rad_s = 0.0;
    double accel_bias_max_mps2 = 0.0;
    int overlap_count = 0;
    for (const GraphState &current : states) {
      SlidingWindowStateEstimate previous;
      if (!interpolate_previous_window_state(current.camera_time, previous))
        continue;
      const Eigen::Matrix3d current_rotation =
          ov_core::quat_2_Rot(normalized_jpl(current.q.data()));
      const Eigen::Matrix3d previous_rotation =
          ov_core::quat_2_Rot(previous.q_GtoI);
      attitude_max_deg = std::max(
          attitude_max_deg,
          rotation_angle(current_rotation * previous_rotation.transpose()) *
              180.0 / kPi);
      position_max_m =
          std::max(position_max_m,
                   (map3(current.p) - previous.p_IinG).norm());
      velocity_max_mps =
          std::max(velocity_max_mps,
                   (map3(current.v) - previous.v_IinG).norm());
      gyro_bias_max_rad_s =
          std::max(gyro_bias_max_rad_s,
                   (map3(current.bg) - previous.bg).norm());
      accel_bias_max_mps2 =
          std::max(accel_bias_max_mps2,
                   (map3(current.ba) - previous.ba).norm());
      ++overlap_count;
    }
    diag.sliding_overlap_state_count = overlap_count;
    diag.sliding_overlap_attitude_max_deg = attitude_max_deg;
    diag.sliding_overlap_position_max_m = position_max_m;
    diag.sliding_overlap_velocity_max_mps = velocity_max_mps;
    diag.sliding_overlap_gyro_bias_max_rad_s = gyro_bias_max_rad_s;
    diag.sliding_overlap_accel_bias_max_mps2 = accel_bias_max_mps2;
  }
  if (summary.IsSolutionUsable()) {
    previous_window_states_.clear();
    previous_window_states_.reserve(states.size());
    for (const GraphState &state : states) {
      SlidingWindowStateEstimate estimate;
      estimate.timestamp = state.camera_time;
      estimate.q_GtoI = normalized_jpl(state.q.data());
      estimate.p_IinG = map3(state.p);
      estimate.v_IinG = map3(state.v);
      estimate.bg = map3(state.bg);
      estimate.ba = map3(state.ba);
      previous_window_states_.push_back(estimate);
    }
  }

  GraphState &final_state = states.back();
  const Eigen::Matrix3d final_rotation =
      ov_core::quat_2_Rot(normalized_jpl(final_state.q.data()));
  diag.fc_terminal_attitude_residual_deg =
      rotation_angle(final_rotation * fc_target_rotations.back().transpose()) *
      180.0 / kPi;
  diag.fc_terminal_position_residual_m =
      (map3(final_state.p) - fc_target_positions.back()).norm();
  diag.fc_terminal_velocity_residual_mps =
      (map3(final_state.v) - fc_target_velocities.back()).norm();
  diag.fc_terminal_max_normalized_residual = std::max(
      {diag.fc_terminal_attitude_residual_deg /
           std::max(1.0e-8, options_.fc_attitude_sigma_deg),
       diag.fc_terminal_position_residual_m /
           std::max(1.0e-8, options_.fc_position_sigma_m),
       diag.fc_terminal_velocity_residual_mps /
           std::max(1.0e-8, options_.fc_velocity_sigma_mps)});
  const Eigen::Vector3d final_bg = map3(final_state.bg);
  const Eigen::Vector3d final_ba = map3(final_state.ba);
  const Eigen::Matrix3d final_mount =
      ov_core::quat_2_Rot(normalized_jpl(mount_q.data()));
  const double final_mount_residual_deg =
      rotation_angle(final_mount * options_.R_FtoI_declared.transpose()) *
      180.0 / kPi;

  const bool deferred_release_certification =
      options_.sliding_window_shadow_only &&
      options_.sliding_window_direct_state_release &&
      options_.sliding_window_deferred_release_certification;
  if (deferred_release_certification) {
    diag.factor_contributions = {
        evaluate_family_residuals(problem, "prior", families.prior),
        evaluate_family_residuals(problem, "imu_preintegration", families.imu),
        evaluate_family_residuals(problem, "visual_reprojection",
                                  families.visual, false),
        evaluate_family_residuals(problem, "fc_pose_velocity_attitude",
                                  families.fc)};
    const FactorContribution *rolling_imu =
        find_contribution(diag.factor_contributions, "imu_preintegration");
    const FactorContribution *rolling_visual =
        find_contribution(diag.factor_contributions, "visual_reprojection");
    const FactorContribution *rolling_fc = find_contribution(
        diag.factor_contributions, "fc_pose_velocity_attitude");
    if (rolling_visual != nullptr && rolling_visual->residual_blocks > 0) {
      diag.visual_statistics.reprojection_rmse_px =
          rolling_visual->residual_rms * options_.visual_pixel_sigma;
      diag.visual_statistics.reprojection_p95_px =
          rolling_visual->residual_p95 * options_.visual_pixel_sigma;
    }

    const bool physical_overlap_passed =
        diag.sliding_overlap_state_count >=
            options_.sliding_window_min_overlap_states &&
        diag.sliding_overlap_attitude_max_deg <=
            options_.sliding_window_max_overlap_attitude_deg &&
        diag.sliding_overlap_position_max_m <=
            options_.sliding_window_max_overlap_position_m &&
        diag.sliding_overlap_velocity_max_mps <=
            options_.sliding_window_max_overlap_velocity_mps &&
        diag.sliding_overlap_gyro_bias_max_rad_s <=
            options_.sliding_window_max_overlap_gyro_bias_rad_s &&
        diag.sliding_overlap_accel_bias_max_mps2 <=
            options_.sliding_window_max_overlap_accel_bias_mps2;
    const bool rolling_visual_supported =
        rolling_visual != nullptr &&
        rolling_visual->residual_blocks >=
            options_.min_visual_residual_blocks &&
        accepted_features >= options_.min_stereo_depths;
    const bool rolling_factor_quality_passed =
        rolling_imu != nullptr && rolling_imu->residual_blocks > 0 &&
        rolling_imu->residual_rms <=
            options_.navigation_max_imu_residual_rms &&
        rolling_fc != nullptr && rolling_fc->residual_blocks > 0 &&
        rolling_fc->residual_rms <= options_.navigation_max_fc_residual_rms &&
        (options_.navigation_allow_without_visual ||
         (rolling_visual_supported &&
          diag.visual_statistics.reprojection_rmse_px <=
              options_.navigation_max_reprojection_rmse_px &&
          diag.visual_statistics.reprojection_p95_px <=
              options_.navigation_max_reprojection_p95_px));
    const bool rolling_state_quality_passed =
        summary.IsSolutionUsable() && physical_overlap_passed &&
        rolling_factor_quality_passed &&
        diag.fc_terminal_max_normalized_residual <=
            options_.navigation_max_fc_residual_rms &&
        final_bg.norm() <= options_.max_gyro_bias_norm_rad_s &&
        final_ba.norm() <= options_.max_accel_bias_norm_mps2 &&
        final_mount_residual_deg <= options_.max_mount_residual_deg;

    const double maximum_continuous_gap = std::max(
        2.0 * options_.sliding_window_min_advance_s,
        options_.max_stereo_gap_s);
    const bool continuous_with_previous =
        sliding_prevalidation_last_window_end_ >= 0.0 &&
        diag.init_time > sliding_prevalidation_last_window_end_ &&
        diag.init_time - sliding_prevalidation_last_window_end_ <=
            maximum_continuous_gap + 1.0e-9;
    if (!rolling_state_quality_passed) {
      sliding_prevalidation_stable_since_ = -1.0;
    } else if (sliding_prevalidation_stable_since_ < 0.0 ||
               !continuous_with_previous) {
      sliding_prevalidation_stable_since_ = diag.init_time;
    }
    sliding_prevalidation_last_window_end_ = diag.init_time;
    if (rolling_state_quality_passed &&
        sliding_prevalidation_stable_since_ >= 0.0) {
      diag.sliding_overlap_stable_duration_s = std::max(
          0.0, diag.init_time - sliding_prevalidation_stable_since_);
    }

    double bias_history_span_s = 0.0;
    double bias_history_bg_max_rad_s = 0.0;
    double bias_history_ba_max_mps2 = 0.0;
    int bias_history_count = 1;
    for (auto receipt = attempt_receipts_.rbegin();
         receipt != attempt_receipts_.rend(); ++receipt) {
      if (receipt->window_id == sliding_window_id_ ||
          receipt->window_end_timestamp < 0.0 ||
          receipt->window_end_timestamp >= diag.init_time ||
          !receipt->bg.allFinite() || !receipt->ba.allFinite())
        continue;
      bias_history_span_s =
          std::max(bias_history_span_s,
                   diag.init_time - receipt->window_end_timestamp);
      bias_history_bg_max_rad_s = std::max(
          bias_history_bg_max_rad_s, (final_bg - receipt->bg).norm());
      bias_history_ba_max_mps2 = std::max(
          bias_history_ba_max_mps2, (final_ba - receipt->ba).norm());
      ++bias_history_count;
      if (bias_history_span_s + 1.0e-9 >=
          options_.sliding_window_overlap_stability_duration_s)
        break;
    }
    diag.sliding_bias_trend_sample_count = bias_history_count;
    diag.sliding_bias_trend_span_s = bias_history_span_s;
    const bool rolling_bias_history_passed =
        bias_history_span_s + 1.0e-9 >=
            options_.sliding_window_overlap_stability_duration_s &&
        bias_history_bg_max_rad_s <=
            options_.sliding_window_max_overlap_gyro_bias_rad_s &&
        bias_history_ba_max_mps2 <=
            options_.sliding_window_max_overlap_accel_bias_mps2;
    const bool run_release_certification =
        rolling_state_quality_passed && rolling_bias_history_passed &&
        diag.sliding_overlap_stable_duration_s + 1.0e-9 >=
            options_.sliding_window_overlap_stability_duration_s;

    if (!run_release_certification) {
      AlignmentResult shadow_result;
      shadow_result.timestamp = final_state.camera_time;
      shadow_result.q_GtoI = normalized_jpl(final_state.q.data());
      shadow_result.p_IinG = map3(final_state.p);
      shadow_result.v_IinG = map3(final_state.v);
      shadow_result.bg = final_bg;
      shadow_result.ba = final_ba;
      shadow_result.covariance =
          previous_window_covariance_valid_
              ? previous_window_covariance_
              : Eigen::Matrix<double, 15, 15>::Identity() * 1.0e4;
      shadow_result.R_FtoI_nominal = final_mount;
      shadow_result.R_mount_residual =
          final_mount * options_.R_FtoI_declared.transpose();
      const double mount_sigma_rad =
          options_.fc_board_mount_sigma_deg * kPi / 180.0;
      shadow_result.mount_covariance = mount_sigma_rad * mount_sigma_rad *
                                       Eigen::Matrix3d::Identity();
      shadow_result.fc_to_board_time_offset_s =
          options_.fc_navigation_to_board_time_offset_s;
      shadow_result.fc_attitude_to_board_time_offset_s = best_offset;
      shadow_result.fc_navigation_to_board_time_offset_s =
          options_.fc_navigation_to_board_time_offset_s;
      shadow_result.camera_to_imu_time_offset_s =
          options_.camera_to_imu_time_offset_s;
      shadow_result.readiness = AlignmentReadiness::NOT_READY;
      shadow_result.release_policy = options_.release_policy;
      shadow_result.released_to_openvins = false;

      diag.mount_residual_deg = final_mount_residual_deg;
      diag.decision_time = -1.0;
      diag.quality_passed = false;
      diag.navigation_quality_passed = false;
      diag.full_alignment_quality_passed = false;
      diag.readiness = AlignmentReadiness::NOT_READY;
      diag.readiness_reason =
          "rolling_window_stable_evidence_or_release_certification_pending";
      diag.sliding_overlap_consistency_passed = false;
      diag.initialization_duration_s = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - wall_start).count();
      copy_runtime_counters(diag);

      if (!attempt_receipts_.empty() &&
          attempt_receipts_.back().window_id == sliding_window_id_) {
        AlignmentAttemptReceipt &window = attempt_receipts_.back();
        window.q_GtoI = shadow_result.q_GtoI;
        window.p_IinG = shadow_result.p_IinG;
        window.v_IinG = shadow_result.v_IinG;
        window.bg = shadow_result.bg;
        window.ba = shadow_result.ba;
        window.imu_residual_rms =
            rolling_imu != nullptr
                ? rolling_imu->residual_rms
                : std::numeric_limits<double>::infinity();
        window.fc_residual_rms =
            rolling_fc != nullptr
                ? rolling_fc->residual_rms
                : std::numeric_limits<double>::infinity();
        window.visual_reprojection_rmse_px =
            diag.visual_statistics.reprojection_rmse_px;
        window.visual_reprojection_p95_px =
            diag.visual_statistics.reprojection_p95_px;
        int residual_dimension = 0;
        for (const auto &family : diag.factor_contributions)
          residual_dimension += family.residual_dimension;
        if (residual_dimension > 0 && std::isfinite(summary.final_cost))
          window.joint_normalized_cost =
              2.0 * summary.final_cost / residual_dimension;
        window.angular_excitation_rad_s = diag.angular_excitation_rad_s;
        window.second_axis_ratio = diag.second_axis_ratio;
        if (latest_shadow_window_result_valid_) {
          const Eigen::Matrix3d current_rotation =
              ov_core::quat_2_Rot(shadow_result.q_GtoI);
          const Eigen::Matrix3d previous_rotation =
              ov_core::quat_2_Rot(latest_shadow_window_result_.q_GtoI);
          window.previous_attitude_delta_deg =
              rotation_angle(current_rotation *
                             previous_rotation.transpose()) *
              180.0 / kPi;
          window.previous_position_delta_m =
              (shadow_result.p_IinG -
               latest_shadow_window_result_.p_IinG)
                  .norm();
          window.previous_velocity_delta_mps =
              (shadow_result.v_IinG -
               latest_shadow_window_result_.v_IinG)
                  .norm();
          window.previous_gyro_bias_delta_rad_s =
              (shadow_result.bg - latest_shadow_window_result_.bg).norm();
          window.previous_accel_bias_delta_mps2 =
              (shadow_result.ba - latest_shadow_window_result_.ba).norm();
        }
        window.overlap_state_count = diag.sliding_overlap_state_count;
        window.overlap_attitude_max_deg =
            diag.sliding_overlap_attitude_max_deg;
        window.overlap_position_max_m =
            diag.sliding_overlap_position_max_m;
        window.overlap_velocity_max_mps =
            diag.sliding_overlap_velocity_max_mps;
        window.overlap_gyro_bias_max_rad_s =
            diag.sliding_overlap_gyro_bias_max_rad_s;
        window.overlap_accel_bias_max_mps2 =
            diag.sliding_overlap_accel_bias_max_mps2;
        window.overlap_stable_duration_s =
            diag.sliding_overlap_stable_duration_s;
        window.bias_trend_sample_count =
            diag.sliding_bias_trend_sample_count;
        window.bias_trend_span_s = diag.sliding_bias_trend_span_s;
        window.outcome = summary.IsSolutionUsable()
                             ? "shadow_window_solved"
                             : "shadow_window_solver_unusable";
        window.failed_gate = run_release_certification
                                 ? "release_certification_pending"
                                 : "rolling_stability_pending";
        window.next_eligible_condition =
            "sensor_time_stability_then_release_certification";
      }

      transition(AlignmentPhase::COLLECTING, now,
                 "rolling_window_recorded_without_release_certification");
      diag.state_transitions = transitions_;
      shadow_result.diagnostics = diag;
      if (summary.IsSolutionUsable()) {
        latest_shadow_window_result_ = shadow_result;
        latest_shadow_window_result_valid_ = true;
      }
      last_diagnostics_ = diag;
      last_rejection_ = "p4_r1_release_certification_pending";
      result = shadow_result;
      return true;
    }
  }

  std::vector<double *> tracked_blocks = {
      final_state.q.data(), final_state.p.data(), final_state.v.data(),
      shared_bg.data(), shared_ba.data(), mount_q.data()};
  const std::vector<std::string> tracked_names = {
      "attitude", "position", "velocity", "gyro_bias",
      "accelerometer_bias", "startup_misalignment"};
  diag.factor_contributions = {
      evaluate_family(problem, "prior", families.prior, tracked_blocks,
                      tracked_names),
      evaluate_family(problem, "imu_preintegration", families.imu,
                      tracked_blocks, tracked_names),
      evaluate_family(problem, "visual_reprojection", families.visual,
                      tracked_blocks, tracked_names, false),
      evaluate_family(problem, "fc_pose_velocity_attitude", families.fc,
                      tracked_blocks, tracked_names)};

  const FactorContribution *visual_contribution =
      find_contribution(diag.factor_contributions, "visual_reprojection");
  if (visual_contribution != nullptr &&
      visual_contribution->residual_blocks > 0) {
    diag.visual_statistics.reprojection_rmse_px =
        visual_contribution->residual_rms * options_.visual_pixel_sigma;
    diag.visual_statistics.reprojection_p95_px =
        visual_contribution->residual_p95 * options_.visual_pixel_sigma;
  }

  std::vector<double *> schur_blocks = {
      final_state.q.data(), final_state.p.data(), final_state.v.data(),
      shared_bg.data(), shared_ba.data()};
  int schur_target_dimension = 15;
  if (!mount_fixed_external_calibration) {
    schur_blocks.push_back(mount_q.data());
    schur_target_dimension = 18;
  }
  for (size_t state_index = 0; state_index + 1 < states.size(); ++state_index) {
    schur_blocks.push_back(states[state_index].q.data());
    schur_blocks.push_back(states[state_index].p.data());
    schur_blocks.push_back(states[state_index].v.data());
  }
  if (options_.visual_factors_enabled)
    for (auto &landmark : landmarks)
      schur_blocks.push_back(landmark.data());
  std::vector<ceres::ResidualBlockId> nonvisual_blocks = families.prior;
  nonvisual_blocks.insert(nonvisual_blocks.end(), families.imu.begin(),
                          families.imu.end());
  nonvisual_blocks.insert(nonvisual_blocks.end(), families.fc.begin(),
                          families.fc.end());
  std::vector<ceres::ResidualBlockId> full_blocks = nonvisual_blocks;
  full_blocks.insert(full_blocks.end(), families.visual.begin(),
                     families.visual.end());
  std::vector<ceres::ResidualBlockId> data_blocks = families.imu;
  data_blocks.insert(data_blocks.end(), families.fc.begin(), families.fc.end());
  data_blocks.insert(data_blocks.end(), families.visual.begin(),
                     families.visual.end());
  const Eigen::MatrixXd data_schur_information =
      schur_information(problem, data_blocks, schur_blocks,
                        schur_target_dimension);
  const Eigen::MatrixXd visual_schur_increment =
      schur_information(problem, full_blocks, schur_blocks,
                        schur_target_dimension) -
      schur_information(problem, nonvisual_blocks, schur_blocks,
                        schur_target_dimension);
  diag.visual_statistics.schur_complement_information_trace =
      visual_schur_increment.trace();
  for (size_t state_index = 0; state_index < tracked_names.size();
       ++state_index) {
    if (static_cast<Eigen::Index>(3 * state_index + 3) <=
        visual_schur_increment.rows())
      diag.visual_statistics.information_contribution_by_state[
          tracked_names[state_index]] =
          visual_schur_increment.block<3, 3>(3 * state_index,
                                              3 * state_index)
              .trace();
    else
      diag.visual_statistics.information_contribution_by_state[
          tracked_names[state_index]] = 0.0;
  }

  // Recover one coherent covariance for the active IMU, the terminal
  // consecutive historical poses that fit the EKF clone capacity, and every
  // release-visible landmark that will become an OpenVINS state. Earlier graph
  // poses still contribute as nuisance variables to this exact marginal.
  // The ordering is a release contract:
  // [active q,p,v,bg,ba, clone_0 q,p, ..., landmark_0 xyz, ...]. Historical
  // velocities and non-persistent landmarks remain nuisance variables.
  std::vector<size_t> persistent_landmark_indices;
  if (options_.visual_factors_enabled &&
      options_.max_initial_slam_features > 0) {
    persistent_landmark_indices.reserve(std::min(
        static_cast<size_t>(options_.max_initial_slam_features),
        landmark_feature_ids.size()));
    const size_t final_state_index = states.size() - 1;
    for (size_t landmark_index = 0;
         landmark_index < landmark_feature_ids.size() &&
         persistent_landmark_indices.size() <
             static_cast<size_t>(options_.max_initial_slam_features);
         ++landmark_index) {
      const auto track = tracks.find(landmark_feature_ids[landmark_index]);
      if (track == tracks.end())
        continue;
      const bool observed_at_release = std::any_of(
          track->second.begin(), track->second.end(),
          [final_state_index](const FeatureTrackEntry &entry) {
            return entry.state_index == final_state_index &&
                   entry.observation != nullptr &&
                   entry.observation->left_valid;
          });
      if (observed_at_release)
        persistent_landmark_indices.push_back(landmark_index);
    }
  }

  const size_t available_historical_states = states.size() - 1;
  const size_t transferred_clone_count = std::min(
      available_historical_states,
      static_cast<size_t>(options_.max_initial_clones));
  const size_t transferred_clone_start =
      available_historical_states - transferred_clone_count;

  std::vector<double *> initialization_covariance_blocks = {
      final_state.q.data(), final_state.p.data(), final_state.v.data(),
      shared_bg.data(), shared_ba.data()};
  for (size_t state_index = transferred_clone_start;
       state_index + 1 < states.size(); ++state_index) {
    initialization_covariance_blocks.push_back(states[state_index].q.data());
    initialization_covariance_blocks.push_back(states[state_index].p.data());
  }
  for (size_t landmark_index : persistent_landmark_indices)
    initialization_covariance_blocks.push_back(
        landmarks[landmark_index].data());
  std::vector<double *> covariance_parameter_blocks =
      initialization_covariance_blocks;
  if (!mount_fixed_external_calibration)
    covariance_parameter_blocks.push_back(mount_q.data());
  std::vector<std::pair<const double *, const double *>> covariance_blocks;
  for (size_t i = 0; i < covariance_parameter_blocks.size(); ++i)
    for (size_t j = i; j < covariance_parameter_blocks.size(); ++j)
      covariance_blocks.emplace_back(covariance_parameter_blocks[i],
                                     covariance_parameter_blocks[j]);
  ceres::Covariance::Options covariance_options;
  covariance_options.algorithm_type = ceres::DENSE_SVD;
  covariance_options.apply_loss_function = true;
  // Monocular landmarks are nuisance variables and can contain weak depth
  // directions. Drop only numerically null singular directions, then apply
  // strict observability/positive-covariance gates to the released 18-D
  // state. This does not hide an unobservable released state.
  covariance_options.null_space_rank = -1;
  ceres::Covariance covariance_solver(covariance_options);
  bool covariance_ok = covariance_solver.Compute(covariance_blocks, &problem);
  Eigen::MatrixXd covariance_tangent = Eigen::MatrixXd::Zero(
      3 * static_cast<Eigen::Index>(covariance_parameter_blocks.size()),
      3 * static_cast<Eigen::Index>(covariance_parameter_blocks.size()));
  if (covariance_ok) {
    for (size_t i = 0; i < covariance_parameter_blocks.size(); ++i) {
      for (size_t j = i; j < covariance_parameter_blocks.size(); ++j) {
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> block;
        if (!covariance_solver.GetCovarianceBlockInTangentSpace(
                covariance_parameter_blocks[i], covariance_parameter_blocks[j],
                block.data())) {
          covariance_ok = false;
          break;
        }
        covariance_tangent.block<3, 3>(3 * i, 3 * j) = block;
        covariance_tangent.block<3, 3>(3 * j, 3 * i) = block.transpose();
      }
      if (!covariance_ok)
        break;
    }
  }
  const Eigen::Index initial_joint_covariance_dimension =
      15 + 6 * static_cast<Eigen::Index>(transferred_clone_count) +
      3 * static_cast<Eigen::Index>(persistent_landmark_indices.size());
  Eigen::MatrixXd initial_joint_covariance = Eigen::MatrixXd::Zero(
      initial_joint_covariance_dimension, initial_joint_covariance_dimension);
  Eigen::Matrix<double, 18, 18> covariance18 =
      Eigen::Matrix<double, 18, 18>::Zero();
  if (covariance_ok) {
    initial_joint_covariance = covariance_tangent.topLeftCorner(
        initial_joint_covariance_dimension,
        initial_joint_covariance_dimension);
    covariance18.block<15, 15>(0, 0) =
        initial_joint_covariance.block<15, 15>(0, 0);
    if (!mount_fixed_external_calibration) {
      const Eigen::Index mount_offset = initial_joint_covariance_dimension;
      covariance18.block<3, 3>(15, 15) =
          covariance_tangent.block<3, 3>(mount_offset, mount_offset);
      for (Eigen::Index block_index = 0; block_index < 5; ++block_index) {
        covariance18.block<3, 3>(3 * block_index, 15) =
            covariance_tangent.block<3, 3>(3 * block_index, mount_offset);
        covariance18.block<3, 3>(15, 3 * block_index) =
            covariance18.block<3, 3>(3 * block_index, 15).transpose();
      }
    }
  }
  if (mount_fixed_external_calibration) {
    const double sigma = options_.fc_board_mount_sigma_deg * kPi / 180.0;
    covariance18.block<3, 3>(15, 15) =
        sigma * sigma * Eigen::Matrix3d::Identity();
  }
  covariance18 = 0.5 * (covariance18 + covariance18.transpose());
  initial_joint_covariance =
      0.5 * (initial_joint_covariance + initial_joint_covariance.transpose());
  diag.initial_history_clone_count =
      static_cast<int>(transferred_clone_count);
  diag.initial_persistent_landmark_count =
      static_cast<int>(persistent_landmark_indices.size());
  diag.initial_joint_covariance_dimension =
      static_cast<int>(initial_joint_covariance_dimension);
  diag.initial_history_covariance_recovered =
      covariance_ok && initial_joint_covariance.allFinite();
  if (!covariance_ok || !covariance18.allFinite())
    diag.navigation_failed_gates.push_back("covariance_recovery");

  const FactorContribution *prior =
      find_contribution(diag.factor_contributions, "prior");
  const FactorContribution *imu_factor =
      find_contribution(diag.factor_contributions, "imu_preintegration");
  const FactorContribution *visual_factor =
      find_contribution(diag.factor_contributions, "visual_reprojection");
  const FactorContribution *fc_factor = find_contribution(
      diag.factor_contributions, "fc_pose_velocity_attitude");
  // The absolute attitude boundary is attached directly to the released
  // terminal state; relative FC attitude increments support the intervening
  // trajectory without multiplying absolute gauge information.
  const bool fc_attitude_gauge_support =
      options_.fc_attitude_gauge_factor_enabled &&
      diag.fc_attitude_gauge_factor_count == 1 && fc_factor != nullptr &&
      fc_factor->residual_blocks > 0 &&
      fc_factor->jacobian_frobenius > 1.0e-10;
  // Navigation-only control may use the IMU+FC p/v graph without visual
  // support. Keep this explicit mode separate from normal visual release.
  const bool navigation_only_without_visual =
      !options_.visual_factors_enabled &&
      options_.navigation_allow_without_visual;
  for (size_t state_index = 0; state_index < tracked_names.size();
       ++state_index) {
    StateObservability observability;
    observability.state = tracked_names[state_index];
    const Eigen::Matrix3d covariance_block =
        covariance18.block<3, 3>(3 * state_index, 3 * state_index);
    observability.covariance_std =
        covariance_block.diagonal().cwiseMax(0.0).cwiseSqrt();
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    if (covariance_block.allFinite() &&
        std::fabs(covariance_block.determinant()) > 1e-24)
      information = covariance_block.inverse();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(information);
    if (eig.info() == Eigen::Success) {
      observability.information_min_eigenvalue = eig.eigenvalues().minCoeff();
      observability.information_max_eigenvalue = eig.eigenvalues().maxCoeff();
      observability.information_condition =
          observability.information_min_eigenvalue > 0.0
              ? observability.information_max_eigenvalue /
                    observability.information_min_eigenvalue
              : std::numeric_limits<double>::infinity();
    }
    if (static_cast<Eigen::Index>(3 * state_index + 3) <=
        data_schur_information.rows()) {
      const Eigen::Matrix3d data_information =
          0.5 * (data_schur_information
                     .block<3, 3>(3 * state_index, 3 * state_index) +
                 data_schur_information
                     .block<3, 3>(3 * state_index, 3 * state_index)
                     .transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> data_eig(
          data_information);
      if (data_eig.info() == Eigen::Success) {
        observability.data_information_min_eigenvalue =
            std::max(0.0, data_eig.eigenvalues().minCoeff());
        observability.data_information_max_eigenvalue =
            std::max(0.0, data_eig.eigenvalues().maxCoeff());
        observability.data_information_condition =
            observability.data_information_min_eigenvalue > 0.0
                ? observability.data_information_max_eigenvalue /
                      observability.data_information_min_eigenvalue
                : std::numeric_limits<double>::infinity();
      }
    }
    if (observability.state == "attitude")
      observability.information_normalization_scale =
          options_.max_attitude_sigma_deg * kPi / 180.0;
    else if (observability.state == "position")
      observability.information_normalization_scale =
          options_.max_position_sigma_m;
    else if (observability.state == "velocity")
      observability.information_normalization_scale =
          options_.max_velocity_sigma_mps;
    else if (observability.state == "gyro_bias")
      observability.information_normalization_scale =
          options_.max_gyro_bias_sigma_rad_s;
    else if (observability.state == "accelerometer_bias")
      observability.information_normalization_scale =
          options_.max_accel_bias_sigma_mps2;
    else
      observability.information_normalization_scale =
          options_.max_mount_sigma_deg * kPi / 180.0;
    observability.information_normalization_scale = std::max(
        1.0e-12, observability.information_normalization_scale);
    const double information_scale_squared =
        observability.information_normalization_scale *
        observability.information_normalization_scale;
    observability.normalized_data_information_min_eigenvalue =
        information_scale_squared *
        observability.data_information_min_eigenvalue;
    observability.normalized_data_information_max_eigenvalue =
        information_scale_squared *
        observability.data_information_max_eigenvalue;
    observability.normalized_data_information_condition =
        observability.data_information_condition;
    observability.prior_jacobian_norm =
        state_jacobian(prior, observability.state);
    observability.imu_jacobian_norm =
        state_jacobian(imu_factor, observability.state);
    observability.visual_jacobian_norm =
        state_jacobian(visual_factor, observability.state);
    observability.fc_jacobian_norm =
        state_jacobian(fc_factor, observability.state);
    observability.prior_residual_count =
        observability.prior_jacobian_norm > 1e-10 && prior
            ? prior->residual_dimension
            : 0;
    observability.imu_residual_count =
        observability.imu_jacobian_norm > 1e-10 && imu_factor
            ? imu_factor->residual_dimension
            : 0;
    observability.visual_residual_count =
        observability.visual_jacobian_norm > 1e-10 && visual_factor
            ? visual_factor->residual_dimension
            : 0;
    const bool state_fc_support =
        observability.fc_jacobian_norm > 1e-10 ||
        (observability.state == "attitude" && fc_attitude_gauge_support);
    observability.fc_residual_count =
        state_fc_support && fc_factor
            ? fc_factor->residual_dimension
            : 0;
    observability.prior_only =
        observability.prior_jacobian_norm > 1e-10 &&
        observability.imu_jacobian_norm <= 1e-10 &&
        observability.visual_jacobian_norm <= 1e-10 &&
        !state_fc_support;
    const bool information_ok =
        observability.normalized_data_information_min_eigenvalue >=
            options_.min_information_eigenvalue &&
        observability.normalized_data_information_condition <=
            options_.max_information_condition;
    bool factor_support = false;
    double max_sigma = std::numeric_limits<double>::infinity();
    if (observability.state == "attitude") {
      const double visual_information =
          std::fabs(diag.visual_statistics.information_contribution_by_state[
              observability.state]);
      factor_support = observability.imu_jacobian_norm > 1e-10 &&
                       fc_attitude_gauge_support &&
                       (navigation_only_without_visual ||
                        observability.visual_jacobian_norm > 1e-10 ||
                        visual_information > 1e-10);
      max_sigma = options_.max_attitude_sigma_deg * kPi / 180.0;
    } else if (observability.state == "position") {
      const double visual_information =
          std::fabs(diag.visual_statistics.information_contribution_by_state[
              observability.state]);
      factor_support = observability.imu_jacobian_norm > 1e-10 &&
                       (navigation_only_without_visual ||
                        observability.visual_jacobian_norm > 1e-10 ||
                        visual_information > 1e-10) &&
                       observability.fc_jacobian_norm > 1e-10;
      max_sigma = options_.max_position_sigma_m;
    } else if (observability.state == "velocity") {
      factor_support = observability.imu_jacobian_norm > 1e-10 &&
                       observability.fc_jacobian_norm > 1e-10;
      max_sigma = options_.max_velocity_sigma_mps;
    } else if (observability.state == "gyro_bias") {
      factor_support = observability.imu_jacobian_norm > 1e-10;
      max_sigma = options_.max_gyro_bias_sigma_rad_s;
    } else if (observability.state == "accelerometer_bias") {
      factor_support = observability.imu_jacobian_norm > 1e-10;
      max_sigma = options_.max_accel_bias_sigma_mps2;
    } else {
      factor_support = observability.fc_jacobian_norm > 1e-10;
      max_sigma = options_.max_mount_sigma_deg * kPi / 180.0;
    }
    const bool sigma_ok = observability.covariance_std.allFinite() &&
                          observability.covariance_std.maxCoeff() <= max_sigma;
    observability.observable = covariance_ok && information_ok && sigma_ok &&
                               factor_support && !observability.prior_only;
    if (observability.state == "startup_misalignment" &&
        mount_fixed_external_calibration)
      observability.estimate_status =
          EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION;
    else if (observability.observable)
      observability.estimate_status =
          EstimateSourceStatus::ESTIMATED_CURRENT_DATA;
    else if (factor_support && covariance_block.allFinite())
      observability.estimate_status = EstimateSourceStatus::WEAKLY_OBSERVABLE;
    else if (observability.prior_only)
      observability.estimate_status = EstimateSourceStatus::FIXED_TO_PRIOR;
    else
      observability.estimate_status = EstimateSourceStatus::UNOBSERVABLE;
    if (!factor_support)
      observability.reason = "missing_required_factor_jacobian";
    else if (observability.prior_only)
      observability.reason = "prior_only";
    else if (!information_ok)
      observability.reason = "information_rank_or_condition";
    else if (!sigma_ok)
      observability.reason = "covariance_threshold";
    else
      observability.reason = "observable";
    diag.state_observability.push_back(observability);
  }

  auto append_unique = [](std::vector<std::string> &gates,
                          const std::string &gate) {
    if (std::find(gates.begin(), gates.end(), gate) == gates.end())
      gates.push_back(gate);
  };
  auto fail_navigation = [&](const std::string &gate) {
    append_unique(diag.navigation_failed_gates, gate);
  };
  auto fail_full = [&](const std::string &gate) {
    append_unique(diag.full_alignment_failed_gates, gate);
  };
  auto fail_common = [&](const std::string &gate) {
    fail_navigation(gate);
    fail_full(gate);
  };

  if (!summary.IsSolutionUsable())
    fail_common("solver_not_usable");
  if (imu_factor == nullptr || imu_factor->residual_blocks == 0 ||
      imu_factor->jacobian_frobenius <= 1e-10)
    fail_common("imu_factor_contribution");
  if (fc_factor == nullptr || fc_factor->residual_blocks == 0 ||
      fc_factor->jacobian_frobenius <= 1e-10)
    fail_common("fc_factor_contribution");

  const bool visual_factor_supported =
      visual_factor != nullptr &&
      visual_factor->residual_blocks >= options_.min_visual_residual_blocks &&
      visual_factor->jacobian_frobenius > 1e-10 &&
      accepted_features >= options_.min_stereo_depths;
  if (!visual_factor_supported) {
    fail_full("visual_factor_contribution");
    if (!options_.navigation_allow_without_visual)
      fail_navigation("visual_factor_contribution");
  }

  auto find_state = [&](const std::string &name)
      -> const StateObservability * {
    for (const auto &state : diag.state_observability)
      if (state.state == name)
        return &state;
    return nullptr;
  };
  auto navigation_state_gate = [&](const std::string &name,
                                   double max_sigma,
                                   bool require_visual) {
    const StateObservability *state = find_state(name);
    if (state == nullptr) {
      fail_navigation("navigation_state_missing:" + name);
      return;
    }
    const double visual_information =
        std::fabs(diag.visual_statistics.information_contribution_by_state[
            name]);
    const bool source_support =
        state->imu_jacobian_norm > 1e-10 &&
        (state->fc_jacobian_norm > 1e-10 ||
         (name == "attitude" && fc_attitude_gauge_support)) &&
        (!require_visual || state->visual_jacobian_norm > 1e-10 ||
         visual_information > 1e-10 ||
         navigation_only_without_visual);
    if (!source_support || !state->covariance_std.allFinite() ||
        state->covariance_std.maxCoeff() > max_sigma)
      fail_navigation("navigation_state_quality:" + name);
  };
  navigation_state_gate("attitude",
                        options_.navigation_max_attitude_sigma_deg * kPi /
                            180.0,
                        true);
  navigation_state_gate("position", options_.navigation_max_position_sigma_m,
                        true);
  navigation_state_gate("velocity", options_.navigation_max_velocity_sigma_mps,
                        false);
  const StateObservability *gyro_bias_state = find_state("gyro_bias");
  const StateObservability *accelerometer_bias_state =
      find_state("accelerometer_bias");
  auto navigation_bias_gate = [&](const StateObservability *state,
                                  const std::string &name,
                                  double max_sigma) {
    if (state == nullptr || state->imu_jacobian_norm <= 1e-10 ||
        !state->covariance_std.allFinite() ||
        state->covariance_std.maxCoeff() > max_sigma)
      fail_navigation("navigation_state_quality:" + name);
  };
  navigation_bias_gate(gyro_bias_state, "gyro_bias",
                       options_.navigation_max_gyro_bias_sigma_rad_s);
  navigation_bias_gate(accelerometer_bias_state, "accelerometer_bias",
                       options_.navigation_max_accel_bias_sigma_mps2);
  for (const auto &observability : diag.state_observability)
    if (!observability.observable &&
        !(observability.state == "startup_misalignment" &&
          mount_fixed_external_calibration))
      fail_full("unobservable:" + observability.state);

  if (!mount_fixed_external_calibration) {
    const StateObservability *startup_misalignment_state =
        find_state("startup_misalignment");
    if (startup_misalignment_state != nullptr)
      diag.startup_misalignment_status =
          startup_misalignment_state->estimate_status;
  }

  if (visual_factor_supported) {
    if (!(diag.visual_statistics.reprojection_rmse_px <=
          options_.navigation_max_reprojection_rmse_px))
      fail_common("visual_reprojection_rmse");
    if (!(diag.visual_statistics.reprojection_p95_px <=
          options_.navigation_max_reprojection_p95_px))
      fail_common("visual_reprojection_p95");
  }
  if (imu_factor != nullptr &&
      !(imu_factor->residual_rms <= options_.navigation_max_imu_residual_rms))
    fail_common("imu_residual_rms");
  if (fc_factor != nullptr &&
      !(fc_factor->residual_rms <= options_.navigation_max_fc_residual_rms))
    fail_common("fc_residual_rms");
  if (!(diag.fc_terminal_max_normalized_residual <=
        options_.navigation_max_fc_residual_rms))
    fail_common("terminal_fc_boundary_residual");

  if (final_bg.norm() > options_.max_gyro_bias_norm_rad_s)
    fail_common("gyro_bias_norm");
  if (final_ba.norm() > options_.max_accel_bias_norm_mps2)
    fail_common("accel_bias_norm");
  if (final_mount_residual_deg > options_.max_mount_residual_deg)
    fail_full("startup_misalignment_angle");

  Eigen::Matrix<double, 15, 15> release_covariance =
      covariance18.block<15, 15>(0, 0);
  if (gyro_bias_state != nullptr && !gyro_bias_state->observable)
    release_covariance.block<3, 3>(9, 9) +=
        std::pow(options_.gyro_bias_prior_sigma_rad_s, 2) *
        Eigen::Matrix3d::Identity();
  if (accelerometer_bias_state != nullptr &&
      !accelerometer_bias_state->observable)
    release_covariance.block<3, 3>(12, 12) +=
        std::pow(options_.accel_bias_prior_sigma_mps2, 2) *
        Eigen::Matrix3d::Identity();
  release_covariance =
      0.5 * (release_covariance + release_covariance.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> release_eig(
      release_covariance);
  if (release_eig.info() != Eigen::Success ||
      release_eig.eigenvalues().minCoeff() <= 0.0)
    fail_common("release_covariance_not_positive_definite");

  Eigen::MatrixXd release_joint_covariance = initial_joint_covariance;
  if (release_joint_covariance.rows() >= 15 &&
      release_joint_covariance.cols() >= 15) {
    release_joint_covariance.block<15, 15>(0, 0) = release_covariance;
    release_joint_covariance =
        0.5 * (release_joint_covariance + release_joint_covariance.transpose());
  }
  bool release_joint_covariance_valid =
      covariance_ok && release_joint_covariance.rows() ==
                           initial_joint_covariance_dimension &&
      release_joint_covariance.cols() == initial_joint_covariance_dimension &&
      release_joint_covariance.allFinite();
  if (release_joint_covariance_valid) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> joint_eig(
        release_joint_covariance);
    release_joint_covariance_valid =
        joint_eig.info() == Eigen::Success &&
        joint_eig.eigenvalues().minCoeff() > 0.0;
  }
  if (options_.sliding_window_direct_state_release &&
      !release_joint_covariance_valid)
    fail_common("initial_joint_covariance_not_positive_definite");

  // A fixed one-degree test is not a confidence test: on real flight windows
  // it can be tighter than the graph's own one-sigma attitude uncertainty.
  // Normalize each same-timestamp trajectory disagreement by the combined
  // covariance of the two complete joint solutions. Absolute limits remain as
  // independent physical fail-closed guards.
  const bool release_covariance_valid =
      release_eig.info() == Eigen::Success &&
      release_eig.eigenvalues().minCoeff() > 0.0 &&
      release_covariance.allFinite();

  // Pairwise overlap consistency can accept a bias that moves by a modest
  // amount in the same direction on every advanced window. Fit the causal
  // terminal shared-bias history over a real sensor-time span and project its
  // slope over one complete solve window. A converged bias must move by no
  // more than one posterior sigma over that horizon.
  struct BiasTrendSample {
    double timestamp = -1.0;
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  };
  std::vector<BiasTrendSample> bias_trend_samples;
  bias_trend_samples.push_back({diag.init_time, final_bg, final_ba});
  for (auto receipt = attempt_receipts_.rbegin();
       receipt != attempt_receipts_.rend(); ++receipt) {
    if (receipt->window_id == sliding_window_id_ ||
        receipt->window_end_timestamp < 0.0 ||
        receipt->window_end_timestamp >= diag.init_time ||
        !receipt->bg.allFinite() || !receipt->ba.allFinite())
      continue;
    bias_trend_samples.push_back(
        {receipt->window_end_timestamp, receipt->bg, receipt->ba});
    if (diag.init_time - receipt->window_end_timestamp + 1.0e-9 >=
        options_.sliding_window_overlap_stability_duration_s)
      break;
  }
  const bool bias_trend_gate_enabled =
      std::isfinite(options_.sliding_window_bias_trend_max_normalized_sigma);
  if (bias_trend_samples.size() >= 3) {
    double mean_time = 0.0;
    Eigen::Vector3d mean_bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_ba = Eigen::Vector3d::Zero();
    double earliest_time = diag.init_time;
    for (const auto &sample : bias_trend_samples) {
      mean_time += sample.timestamp;
      mean_bg += sample.bg;
      mean_ba += sample.ba;
      earliest_time = std::min(earliest_time, sample.timestamp);
    }
    const double inverse_count =
        1.0 / static_cast<double>(bias_trend_samples.size());
    mean_time *= inverse_count;
    mean_bg *= inverse_count;
    mean_ba *= inverse_count;
    double time_information = 0.0;
    Eigen::Vector3d bg_time_cross = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba_time_cross = Eigen::Vector3d::Zero();
    for (const auto &sample : bias_trend_samples) {
      const double centered_time = sample.timestamp - mean_time;
      time_information += centered_time * centered_time;
      bg_time_cross += centered_time * (sample.bg - mean_bg);
      ba_time_cross += centered_time * (sample.ba - mean_ba);
    }
    diag.sliding_bias_trend_sample_count =
        static_cast<int>(bias_trend_samples.size());
    diag.sliding_bias_trend_span_s = diag.init_time - earliest_time;
    if (time_information > 1.0e-12 && release_covariance_valid) {
      const Eigen::Vector3d bg_slope = bg_time_cross / time_information;
      const Eigen::Vector3d ba_slope = ba_time_cross / time_information;
      diag.sliding_bg_trend_projected_change_rad_s =
          options_.window_duration_s * bg_slope.cwiseAbs();
      diag.sliding_ba_trend_projected_change_mps2 =
          options_.window_duration_s * ba_slope.cwiseAbs();
      const Eigen::Vector3d bg_std =
          release_covariance.block<3, 3>(9, 9)
              .diagonal()
              .cwiseMax(1.0e-18)
              .cwiseSqrt();
      const Eigen::Vector3d ba_std =
          release_covariance.block<3, 3>(12, 12)
              .diagonal()
              .cwiseMax(1.0e-18)
              .cwiseSqrt();
      diag.sliding_bias_trend_normalized_max_sigma = std::max(
          (diag.sliding_bg_trend_projected_change_rad_s
               .cwiseQuotient(bg_std))
              .maxCoeff(),
          (diag.sliding_ba_trend_projected_change_mps2
               .cwiseQuotient(ba_std))
              .maxCoeff());
      diag.sliding_bias_trend_passed =
          diag.sliding_bias_trend_span_s + 1.0e-9 >=
              options_.sliding_window_overlap_stability_duration_s &&
          diag.sliding_bias_trend_normalized_max_sigma <=
              options_.sliding_window_bias_trend_max_normalized_sigma;
    }
  }
  if (!bias_trend_gate_enabled)
    diag.sliding_bias_trend_passed = true;
  if (summary.IsSolutionUsable() && previous_window_covariance_valid_ &&
      release_covariance_valid &&
      diag.sliding_overlap_state_count >=
          options_.sliding_window_min_overlap_states) {
    // Adjacent windows share most of their measurements.  Without their
    // cross-covariance, P_current + P_previous is not the covariance of the
    // estimate difference and must not be advertised as a normalized gate.
    // Retain only explicit physical safety bounds on this dormant diagnostic
    // path. Formal P4 uses the disjoint causal holdout lifecycle above.
    diag.sliding_overlap_normalized_max_sigma =
        std::numeric_limits<double>::infinity();
    const bool physical_safety_passed =
        diag.sliding_overlap_attitude_max_deg <=
            options_.sliding_window_max_overlap_attitude_deg &&
        diag.sliding_overlap_position_max_m <=
            options_.sliding_window_max_overlap_position_m &&
        diag.sliding_overlap_velocity_max_mps <=
            options_.sliding_window_max_overlap_velocity_mps &&
        diag.sliding_overlap_gyro_bias_max_rad_s <=
            options_.sliding_window_max_overlap_gyro_bias_rad_s &&
        diag.sliding_overlap_accel_bias_max_mps2 <=
            options_.sliding_window_max_overlap_accel_bias_mps2;
    diag.sliding_overlap_consistency_passed = physical_safety_passed;
  }
  if (summary.IsSolutionUsable() && release_covariance_valid) {
    previous_window_covariance_ = release_covariance;
    previous_window_covariance_valid_ = true;
  } else {
    previous_window_covariance_valid_ = false;
  }

  for (const auto &observability : diag.state_observability) {
    switch (observability.estimate_status) {
    case EstimateSourceStatus::ESTIMATED_CURRENT_DATA:
      diag.estimated_state_list.push_back(observability.state);
      break;
    case EstimateSourceStatus::WEAKLY_OBSERVABLE:
      diag.weak_state_list.push_back(observability.state);
      break;
    case EstimateSourceStatus::FIXED_TO_PRIOR:
      diag.fixed_state_list.push_back(observability.state);
      break;
    case EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION:
      diag.fixed_state_list.push_back(observability.state +
                                      ":external_calibration");
      break;
    case EstimateSourceStatus::UNOBSERVABLE:
      diag.unobservable_state_list.push_back(observability.state);
      break;
    }
  }
  if (diag.time_offset_status == EstimateSourceStatus::ESTIMATED_CURRENT_DATA)
    diag.estimated_state_list.push_back("fc_to_board_time_offset");
  else if (diag.time_offset_status == EstimateSourceStatus::FIXED_TO_PRIOR)
    diag.fixed_state_list.push_back("fc_to_board_time_offset");
  else if (diag.time_offset_status ==
           EstimateSourceStatus::FIXED_EXTERNAL_CALIBRATION)
    diag.fixed_state_list.push_back(
        "fc_attitude_to_board_time_offset:external_calibration");
  else if (diag.time_offset_status == EstimateSourceStatus::WEAKLY_OBSERVABLE)
    diag.weak_state_list.push_back("fc_to_board_time_offset");
  else
    diag.unobservable_state_list.push_back("fc_to_board_time_offset");

  const auto wall_end = std::chrono::steady_clock::now();
  diag.initialization_duration_s =
      std::chrono::duration<double>(wall_end - wall_start).count();
  diag.rejected_intervals = rejected_intervals_;
  diag.provenance = options_.provenance;
  diag.navigation_quality_passed = diag.navigation_failed_gates.empty();
  diag.full_alignment_quality_passed =
      diag.full_alignment_failed_gates.empty();
  const bool release_full = diag.full_alignment_quality_passed;
  const bool release_navigation =
      options_.release_policy == AlignmentReleasePolicy::PRACTICAL_NAVIGATION_START &&
      diag.navigation_quality_passed;

  bool direct_sliding_release_ready = false;
  if (options_.sliding_window_direct_state_release) {
    // The overlap clock and the bias-trend window are parallel pieces of
    // evidence. The bias trend already spans
    // sliding_window_overlap_stability_duration_s in sensor time; including it
    // in the overlap clock condition would require that full trend gate to stay
    // true for another complete duration (roughly doubling the intended
    // maturation time). Accumulate q/p/v overlap and navigation quality on
    // their own clock, then intersect that mature evidence with the current
    // time-domain bias trend at release.
    const bool current_overlap_consistent =
        summary.IsSolutionUsable() && (release_full || release_navigation) &&
        diag.sliding_overlap_consistency_passed;
    if (deferred_release_certification) {
      direct_sliding_release_ready =
          current_overlap_consistent &&
          diag.sliding_overlap_stable_duration_s + 1.0e-9 >=
              options_.sliding_window_overlap_stability_duration_s &&
          diag.sliding_bias_trend_passed;
    } else {
      const double maximum_continuous_gap = std::max(
          2.0 * options_.sliding_window_min_advance_s,
          options_.max_stereo_gap_s);
      const bool continuous_with_previous =
          sliding_overlap_last_window_end_ >= 0.0 &&
          diag.init_time > sliding_overlap_last_window_end_ &&
          diag.init_time - sliding_overlap_last_window_end_ <=
              maximum_continuous_gap + 1.0e-9;
      if (!current_overlap_consistent) {
        sliding_overlap_stable_since_ = -1.0;
      } else if (sliding_overlap_stable_since_ < 0.0 ||
                 !continuous_with_previous) {
        sliding_overlap_stable_since_ = diag.init_time;
      }
      sliding_overlap_last_window_end_ = diag.init_time;
      if (current_overlap_consistent && sliding_overlap_stable_since_ >= 0.0) {
        diag.sliding_overlap_stable_duration_s =
            std::max(0.0, diag.init_time - sliding_overlap_stable_since_);
        direct_sliding_release_ready =
            diag.sliding_overlap_stable_duration_s + 1.0e-9 >=
                options_.sliding_window_overlap_stability_duration_s &&
            diag.sliding_bias_trend_passed;
      }
    }
  }
  if (options_.sliding_window_shadow_only) {
    AlignmentResult shadow_result;
    shadow_result.timestamp = final_state.camera_time;
    shadow_result.q_GtoI = normalized_jpl(final_state.q.data());
    if (options_.diagnostic_release_attitude_from_fc) {
      const Eigen::Matrix3d R_GtoF_release =
          ov_core::quat_2_Rot(fc_attitude_at_state.back().q_GtoF);
      shadow_result.q_GtoI =
          ov_core::rot_2_quat(final_mount * R_GtoF_release);
    }
    shadow_result.p_IinG = map3(final_state.p);
    shadow_result.v_IinG = map3(final_state.v);
    shadow_result.bg = final_bg;
    shadow_result.ba = final_ba;
    if (options_.diagnostic_release_graph_attitude_fc_pv_zero_bias) {
      const Eigen::Matrix3d R_GtoF_release =
          ov_core::quat_2_Rot(fc_attitude_at_state.back().q_GtoF);
      const Eigen::Matrix3d R_FtoG_release = R_GtoF_release.transpose();
      Eigen::Vector3d omega_F_release = Eigen::Vector3d::Zero();
      if (!fc_rates.empty()) {
        const auto nearest = std::min_element(
            fc_rates.begin(), fc_rates.end(),
            [&](const FCRateSample &a, const FCRateSample &b) {
              return std::fabs(a.timestamp -
                               fc_attitude_at_state.back().timestamp) <
                     std::fabs(b.timestamp -
                               fc_attitude_at_state.back().timestamp);
            });
        omega_F_release = nearest->omega_F;
      }
      shadow_result.p_IinG =
          fc_navigation_at_state.back().position_G +
          R_FtoG_release * options_.p_IinF;
      shadow_result.v_IinG =
          fc_navigation_at_state.back().velocity_G +
          R_FtoG_release * omega_F_release.cross(options_.p_IinF);
      shadow_result.bg.setZero();
      shadow_result.ba.setZero();
      diag.readiness_reason =
          "diagnostic_mixed_graph_attitude_fc_pv_zero_bias";
    }
    shadow_result.covariance = release_covariance;
    const bool coherent_history_release =
        release_joint_covariance_valid &&
        !options_.diagnostic_release_attitude_from_fc &&
        !options_.diagnostic_release_graph_attitude_fc_pv_zero_bias;
    if (coherent_history_release) {
      shadow_result.initial_clones.reserve(transferred_clone_count);
      for (size_t state_index = transferred_clone_start;
           state_index + 1 < states.size();
           ++state_index) {
        AlignmentCloneState clone;
        clone.timestamp = states[state_index].camera_time;
        clone.q_GtoI = normalized_jpl(states[state_index].q.data());
        clone.p_IinG = map3(states[state_index].p);
        shadow_result.initial_clones.push_back(clone);
      }
      shadow_result.initial_landmarks.reserve(
          persistent_landmark_indices.size());
      for (size_t landmark_index : persistent_landmark_indices) {
        AlignmentLandmarkState landmark;
        landmark.feature_id = landmark_feature_ids[landmark_index];
        landmark.p_FinG = Eigen::Vector3d(
            landmarks[landmark_index][0], landmarks[landmark_index][1],
            landmarks[landmark_index][2]);
        shadow_result.initial_landmarks.push_back(landmark);
      }
      shadow_result.startup_consumed_feature_ids = landmark_feature_ids;
      shadow_result.initial_joint_covariance = release_joint_covariance;
    }
    shadow_result.R_FtoI_nominal = final_mount;
    shadow_result.R_mount_residual =
        final_mount * options_.R_FtoI_declared.transpose();
    shadow_result.mount_covariance = covariance18.block<3, 3>(15, 15);
    shadow_result.fc_to_board_time_offset_s =
        options_.fc_navigation_to_board_time_offset_s;
    shadow_result.fc_attitude_to_board_time_offset_s = best_offset;
    shadow_result.fc_navigation_to_board_time_offset_s =
        options_.fc_navigation_to_board_time_offset_s;
    shadow_result.camera_to_imu_time_offset_s =
        options_.camera_to_imu_time_offset_s;
    shadow_result.readiness =
        direct_sliding_release_ready
            ? (release_full ? AlignmentReadiness::FULL_ALIGNMENT_READY
                            : AlignmentReadiness::NAVIGATION_READY)
            : AlignmentReadiness::NOT_READY;
    shadow_result.release_policy = options_.release_policy;
    const bool formal_direct_release =
        direct_sliding_release_ready &&
        !options_.sliding_window_diagnostic_never_anchor;
    shadow_result.released_to_openvins = formal_direct_release;

    diag.mount_residual_deg = final_mount_residual_deg;
    diag.decision_time = direct_sliding_release_ready ? now : -1.0;
    diag.quality_passed = direct_sliding_release_ready;
    diag.readiness = shadow_result.readiness;
    diag.readiness_reason = direct_sliding_release_ready
                                ? "time_based_overlap_consistency_direct_state_release"
                                : "sliding_window_overlap_evidence_pending";
    diag.failed_gates =
        options_.release_policy == AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT
            ? diag.full_alignment_failed_gates
            : diag.navigation_failed_gates;
    copy_runtime_counters(diag);

    if (!attempt_receipts_.empty() &&
        attempt_receipts_.back().window_id == sliding_window_id_) {
      AlignmentAttemptReceipt &window = attempt_receipts_.back();
      window.q_GtoI = shadow_result.q_GtoI;
      window.p_IinG = shadow_result.p_IinG;
      window.v_IinG = shadow_result.v_IinG;
      window.bg = shadow_result.bg;
      window.ba = shadow_result.ba;
      if (release_covariance.allFinite())
        window.state_std =
            release_covariance.diagonal().cwiseMax(0.0).cwiseSqrt();
      const FactorContribution *imu_contribution = find_contribution(
          diag.factor_contributions, "imu_preintegration");
      const FactorContribution *fc_contribution = find_contribution(
          diag.factor_contributions, "fc_pose_velocity_attitude");
      window.imu_residual_rms =
          imu_contribution != nullptr
              ? imu_contribution->residual_rms
              : std::numeric_limits<double>::infinity();
      window.fc_residual_rms =
          fc_contribution != nullptr
              ? fc_contribution->residual_rms
              : std::numeric_limits<double>::infinity();
      window.visual_reprojection_rmse_px =
          diag.visual_statistics.reprojection_rmse_px;
      window.visual_reprojection_p95_px =
          diag.visual_statistics.reprojection_p95_px;
      int residual_dimension = 0;
      for (const auto &family : diag.factor_contributions)
        residual_dimension += family.residual_dimension;
      if (residual_dimension > 0 && std::isfinite(summary.final_cost))
        window.joint_normalized_cost =
            2.0 * summary.final_cost / residual_dimension;
      window.angular_excitation_rad_s = diag.angular_excitation_rad_s;
      window.second_axis_ratio = diag.second_axis_ratio;
      if (latest_shadow_window_result_valid_) {
        const Eigen::Matrix3d current_rotation =
            ov_core::quat_2_Rot(shadow_result.q_GtoI);
        const Eigen::Matrix3d previous_rotation =
            ov_core::quat_2_Rot(latest_shadow_window_result_.q_GtoI);
        window.previous_attitude_delta_deg =
            rotation_angle(current_rotation * previous_rotation.transpose()) *
            180.0 / kPi;
        window.previous_position_delta_m =
            (shadow_result.p_IinG -
             latest_shadow_window_result_.p_IinG)
                .norm();
        window.previous_velocity_delta_mps =
            (shadow_result.v_IinG -
             latest_shadow_window_result_.v_IinG)
                .norm();
        window.previous_gyro_bias_delta_rad_s =
            (shadow_result.bg - latest_shadow_window_result_.bg).norm();
        window.previous_accel_bias_delta_mps2 =
            (shadow_result.ba - latest_shadow_window_result_.ba).norm();
      }
      window.overlap_state_count = diag.sliding_overlap_state_count;
      window.overlap_attitude_max_deg =
          diag.sliding_overlap_attitude_max_deg;
      window.overlap_position_max_m =
          diag.sliding_overlap_position_max_m;
      window.overlap_velocity_max_mps =
          diag.sliding_overlap_velocity_max_mps;
      window.overlap_gyro_bias_max_rad_s =
          diag.sliding_overlap_gyro_bias_max_rad_s;
      window.overlap_accel_bias_max_mps2 =
          diag.sliding_overlap_accel_bias_max_mps2;
      window.overlap_normalized_max_sigma =
          diag.sliding_overlap_normalized_max_sigma;
      window.overlap_stable_duration_s =
          diag.sliding_overlap_stable_duration_s;
      window.overlap_consistency_passed =
          diag.sliding_overlap_consistency_passed;
      window.bias_trend_sample_count =
          diag.sliding_bias_trend_sample_count;
      window.bias_trend_span_s = diag.sliding_bias_trend_span_s;
      window.bg_trend_projected_change_rad_s =
          diag.sliding_bg_trend_projected_change_rad_s;
      window.ba_trend_projected_change_mps2 =
          diag.sliding_ba_trend_projected_change_mps2;
      window.bias_trend_normalized_max_sigma =
          diag.sliding_bias_trend_normalized_max_sigma;
      window.bias_trend_passed = diag.sliding_bias_trend_passed;
      window.outcome = formal_direct_release
                           ? "direct_state_release"
                           : (direct_sliding_release_ready
                                  ? "diagnostic_release_suppressed"
                           : (summary.IsSolutionUsable()
                                  ? "shadow_window_solved"
                                  : "shadow_window_solver_unusable"));
      window.failed_gate = join(diag.failed_gates);
      window.next_eligible_condition =
          "fixed_window_end_advance_s>=" +
          std::to_string(options_.sliding_window_min_advance_s);
    }

    shadow_result.diagnostics = diag;
    if (formal_direct_release) {
      navigation_released_ = true;
      ++successful_release_count_;
      navigation_ready_time_ = now;
      if (release_full) {
        full_alignment_recorded_ = true;
        full_alignment_ready_time_ = now;
      }
      alignment_window_closed_ = true;
      alignment_window_close_time_ = now;
      transition(shadow_result.readiness ==
                         AlignmentReadiness::FULL_ALIGNMENT_READY
                     ? AlignmentPhase::FULL_ALIGNMENT_READY
                     : AlignmentPhase::NAVIGATION_READY,
                 now, "direct_sliding_window_state_release");
      copy_runtime_counters(diag);
      diag.navigation_ready_time = navigation_ready_time_;
      diag.full_alignment_ready_time = full_alignment_ready_time_;
      diag.alignment_window_closed = true;
      diag.alignment_window_close_time = alignment_window_close_time_;
      diag.state_transitions = transitions_;
      shadow_result.diagnostics = diag;
      latest_shadow_window_result_ = shadow_result;
      latest_shadow_window_result_valid_ = true;
      latest_evidence_diagnostics_ = diag;
      latest_evidence_diagnostics_valid_ = true;
      last_diagnostics_ = diag;
      last_rejection_.clear();
      result = shadow_result;
      return true;
    }
    transition(AlignmentPhase::COLLECTING, now,
               "p4_r1_shadow_window_recorded_waiting_for_next_window");
    diag.state_transitions = transitions_;
    shadow_result.diagnostics = diag;
    if (summary.IsSolutionUsable()) {
      latest_shadow_window_result_ = shadow_result;
      latest_shadow_window_result_valid_ = true;
      latest_evidence_diagnostics_ = diag;
      latest_evidence_diagnostics_valid_ = true;
    }
    last_diagnostics_ = diag;
    last_rejection_ =
        direct_sliding_release_ready
            ? "p4_r1_diagnostic_release_suppressed"
            : "p4_r1_shadow_only_no_formal_release";
    result = shadow_result;
    return true;
  }
  if (!release_full && !release_navigation) {
    diag.failed_gates =
        options_.release_policy == AlignmentReleasePolicy::STRICT_FULL_ALIGNMENT
            ? diag.full_alignment_failed_gates
            : diag.navigation_failed_gates;
    const std::string reason = join(diag.failed_gates);
    fail_retry(now, reason);
    return false;
  }
  if (navigation_released_ && !release_full) {
    diag.readiness = AlignmentReadiness::NAVIGATION_READY;
    diag.readiness_reason = "navigation_running_full_alignment_pending";
    diag.navigation_ready_time = navigation_ready_time_;
    transition(AlignmentPhase::NAVIGATION_READY, now,
               diag.readiness_reason);
    diag.state_transitions = transitions_;
    last_diagnostics_ = diag;
    last_rejection_ = join(diag.full_alignment_failed_gates);
    return false;
  }

  result = AlignmentResult();
  result.timestamp = final_state.camera_time;
  result.q_GtoI = normalized_jpl(final_state.q.data());
  if (options_.diagnostic_release_attitude_from_fc) {
    const Eigen::Matrix3d R_GtoF_release =
        ov_core::quat_2_Rot(fc_attitude_at_state.back().q_GtoF);
    result.q_GtoI =
        ov_core::rot_2_quat(final_mount * R_GtoF_release);
  }
  result.p_IinG = map3(final_state.p);
  result.v_IinG = map3(final_state.v);
  result.bg = final_bg;
  result.ba = final_ba;
  result.covariance = release_covariance;
  result.R_FtoI_nominal = final_mount;
  result.R_mount_residual =
      final_mount * options_.R_FtoI_declared.transpose();
  result.mount_covariance = covariance18.block<3, 3>(15, 15);
  // Deprecated compatibility alias follows navigation timing so downstream
  // position outputs can never inherit the attitude-channel delay.
  result.fc_to_board_time_offset_s =
      options_.fc_navigation_to_board_time_offset_s;
  result.fc_attitude_to_board_time_offset_s = best_offset;
  result.fc_navigation_to_board_time_offset_s =
      options_.fc_navigation_to_board_time_offset_s;
  result.camera_to_imu_time_offset_s =
      options_.camera_to_imu_time_offset_s;
  diag.mount_residual_deg = final_mount_residual_deg;
  diag.decision_time = -1.0;
  diag.quality_passed = false;
  diag.readiness = release_full ? AlignmentReadiness::FULL_ALIGNMENT_READY
                                : AlignmentReadiness::NAVIGATION_READY;
  diag.readiness_reason =
      release_full
          ? "navigation_and_full_alignment_quality_gates_passed"
          : "navigation_state_quality_passed_weak_states_registered_to_prior";
  result.released_to_openvins = false;
  result.readiness = diag.readiness;
  result.release_policy = options_.release_policy;
  diag.state_transitions = transitions_;
  result.diagnostics = diag;

  if (options_.formal_causal_lifecycle) {
    if (formal_refinement_pending_) {
      // A formal release is owned by the one newly advanced joint solve after
      // the immutable candidate passed its causal holdout.  No old candidate
      // covariance is mixed into this terminal marginal, and the injected
      // timestamp remains the current camera-clock graph horizon.
      if (!refinement_attempt) {
        fail_retry(now, "formal_refinement_state_machine_inconsistent");
        return false;
      }
      diag.formal_candidate_holdout_fc_samples =
          formal_candidate_holdout_fc_samples_;
      diag.formal_candidate_holdout_imu_samples =
          formal_candidate_holdout_imu_samples_;
      diag.formal_candidate_holdout_visual_frames =
          formal_candidate_holdout_visual_frames_;
      diag.formal_candidate_holdout_duration_s =
          formal_candidate_holdout_duration_s_;
      diag.formal_candidate_holdout_passed = true;
      diag.formal_refinement_release = true;
      diag.candidate_closed_loop_refinement_applied = true;
      diag.quality_passed = true;
      diag.decision_time = now;
      diag.readiness = release_full
                           ? AlignmentReadiness::FULL_ALIGNMENT_READY
                           : AlignmentReadiness::NAVIGATION_READY;
      diag.readiness_reason =
          "formal_candidate_holdout_passed_single_current_window_refinement";
      result.readiness = diag.readiness;
      result.released_to_openvins = true;

      navigation_released_ = true;
      ++successful_release_count_;
      navigation_ready_time_ = now;
      alignment_window_closed_ = true;
      alignment_window_close_time_ = now;
      if (release_full) {
        full_alignment_recorded_ = true;
        full_alignment_ready_time_ = now;
        transition(AlignmentPhase::FULL_ALIGNMENT_READY, now,
                   "formal_single_refinement_atomic_release");
      } else {
        transition(AlignmentPhase::NAVIGATION_READY, now,
                   "formal_single_refinement_atomic_release");
      }
      formal_refinement_pending_ = false;
      force_refinement_solve_ = false;
      candidate_active_ = false;
      candidate_visual_snapshots_.clear();
      diag.navigation_ready_time = navigation_ready_time_;
      diag.full_alignment_ready_time = full_alignment_ready_time_;
      diag.alignment_window_closed = true;
      diag.alignment_window_close_time = alignment_window_close_time_;
      copy_runtime_counters(diag);
      diag.state_transitions = transitions_;
      result.diagnostics = diag;
      candidate_result_ = result;
      candidate_record_.result = result;
      if (!attempt_receipts_.empty()) {
        attempt_receipts_.back().outcome = "formal_refined_release";
        attempt_receipts_.back().failed_gate.clear();
        attempt_receipts_.back().next_eligible_condition =
            "closed_after_atomic_release";
      }
      latest_evidence_diagnostics_ = diag;
      latest_evidence_diagnostics_valid_ = true;
      last_diagnostics_ = diag;
      last_rejection_.clear();
      fc_buffer_.clear();
      imu_buffer_.clear();
      stereo_buffer_.clear();
      return true;
    }

    // Freeze the finite-window solution.  The next sensor-time interval is a
    // holdout only: it can reject this candidate or authorize one advanced
    // joint refinement, but it cannot update q/p/v/bg/ba sequentially.
    diag.quality_passed = false;
    diag.readiness = AlignmentReadiness::NOT_READY;
    diag.readiness_reason = "formal_candidate_short_causal_hold_pending";
    result.readiness = AlignmentReadiness::NOT_READY;
    result.released_to_openvins = false;
    candidate_created_time_ = now;
    formal_candidate_camera_time_ = result.timestamp;
    formal_candidate_holdout_fc_samples_ = 0;
    formal_candidate_holdout_imu_samples_ = 0;
    formal_candidate_holdout_visual_frames_ = 0;
    formal_candidate_holdout_duration_s_ = 0.0;
    candidate_reference_snapshot_ =
        make_visual_snapshot(*usable_frames.back());
    candidate_visual_snapshots_.clear();
    candidate_active_ = true;
    ++candidate_created_count_;
    candidate_gate_depth_counts_.fill(0);
    candidate_gate_depth_source_ =
        "formal_fixed_candidate_causal_holdout_no_feedback";
    transition(AlignmentPhase::CANDIDATE_VALIDATING, now,
               "formal_joint_solution_frozen_for_causal_holdout");
    copy_runtime_counters(diag);
    diag.state_transitions = transitions_;
    result.diagnostics = diag;
    candidate_result_ = result;
    candidate_record_ = AlignmentCandidate();
    candidate_record_.result = result;
    candidate_record_.solve_window_start = diag.window_start;
    candidate_record_.solve_window_end = diag.init_time;
    for (const auto *frame : usable_frames)
      candidate_record_.selected_frame_timestamps.push_back(
          frame->left_timestamp);
    candidate_record_.selected_feature_ids.assign(
        fingerprint_features.begin(), fingerprint_features.end());
    candidate_record_.factor_contributions = diag.factor_contributions;
    candidate_record_.state_observability = diag.state_observability;
    candidate_record_.prior_dominated_states = diag.fixed_state_list;
    candidate_record_.prior_dominated_states.insert(
        candidate_record_.prior_dominated_states.end(),
        diag.weak_state_list.begin(), diag.weak_state_list.end());
    candidate_record_.solve_wall_time_s = diag.initialization_duration_s;
    candidate_record_.provenance = diag.provenance;
    if (!attempt_receipts_.empty()) {
      int factor_count = 0;
      for (const auto &family : diag.factor_contributions)
        factor_count += family.residual_blocks;
      attempt_receipts_.back().factor_count = factor_count;
      attempt_receipts_.back().selected_landmarks = 0;
      attempt_receipts_.back().outcome = "candidate_ready";
      attempt_receipts_.back().failed_gate.clear();
      attempt_receipts_.back().next_eligible_condition =
          "two_second_causal_holdout_then_single_refinement";
    }
    last_diagnostics_ = diag;
    last_rejection_ = diag.readiness_reason;
    return false;
  }

  std::array<bool,
             static_cast<std::size_t>(CandidateStateGroup::COUNT)>
      graph_support{};
  graph_support.fill(false);
  const auto set_graph_support = [&](const std::string &state_name,
                                     CandidateStateGroup group) {
    for (const auto &state : diag.state_observability) {
      if (state.state != state_name)
        continue;
      const bool data_supported =
          state.observable && !state.prior_only &&
          (state.estimate_status ==
               EstimateSourceStatus::ESTIMATED_CURRENT_DATA ||
           state.estimate_status == EstimateSourceStatus::WEAKLY_OBSERVABLE);
      graph_support[static_cast<std::size_t>(group)] = data_supported;
      return;
    }
  };
  set_graph_support("attitude", CandidateStateGroup::ATTITUDE);
  set_graph_support("position", CandidateStateGroup::POSITION);
  set_graph_support("velocity", CandidateStateGroup::VELOCITY);
  set_graph_support("gyro_bias", CandidateStateGroup::GYRO_BIAS);
  set_graph_support("accelerometer_bias", CandidateStateGroup::ACCEL_BIAS);
  CandidateNominalState candidate_nominal;
  candidate_nominal.R_GtoI = ov_core::quat_2_Rot(result.q_GtoI);
  candidate_nominal.p_IinG = result.p_IinG;
  candidate_nominal.v_IinG = result.v_IinG;
  candidate_nominal.bg = result.bg;
  candidate_nominal.ba = result.ba;
  // The recursive candidate uses a causal holdout after the joint graph. Its
  // support/history depth is derived from the actual valid FC measurement rows
  // in this selected sliding window. A keyframe cap or a fixed event quota is
  // not a measurement contract, and no graph FC event is replayed.
  std::array<int, 5> window_measurement_counts{};
  for (const FCNavigationSample *sample : fc) {
    if (sample == nullptr)
      continue;
    const bool position_valid = sample->position_valid;
    const bool velocity_valid = sample->velocity_valid;
    const bool pv_valid = position_valid || velocity_valid;
    if (position_valid)
      ++window_measurement_counts[static_cast<std::size_t>(
          CandidateStateGroup::POSITION)];
    if (velocity_valid)
      ++window_measurement_counts[static_cast<std::size_t>(
          CandidateStateGroup::VELOCITY)];
    if (pv_valid) {
      ++window_measurement_counts[static_cast<std::size_t>(
          CandidateStateGroup::ATTITUDE)];
      ++window_measurement_counts[static_cast<std::size_t>(
          CandidateStateGroup::GYRO_BIAS)];
      ++window_measurement_counts[static_cast<std::size_t>(
          CandidateStateGroup::ACCEL_BIAS)];
    }
  }
  CandidateFilterConfig candidate_filter_config =
      options_.candidate_filter_config;
  bool derived_gate_depth = false;
  for (std::size_t index = 0; index < window_measurement_counts.size();
       ++index) {
    CandidateGroupGate &gate = candidate_filter_config.group_gates[index];
    if (!gate.configured || !gate.derive_depth_from_sliding_window)
      continue;
    if (window_measurement_counts[index] <= 0) {
      fail_retry(now, "candidate_filter_no_valid_window_measurement_for_" +
                          std::to_string(index));
      return false;
    }
    // Candidate certification is a short post-solve holdout, not a second
    // solve-window-length wait. The actual number of valid FC events remains
    // determined by the causal stream.
    gate.history_duration_s =
        options_.candidate_short_validation_duration_s;
    if (index == static_cast<std::size_t>(CandidateStateGroup::ATTITUDE) ||
        index == static_cast<std::size_t>(CandidateStateGroup::GYRO_BIAS) ||
        index == static_cast<std::size_t>(CandidateStateGroup::ACCEL_BIAS)) {
      gate.post_feedback_validation_duration_s =
          options_.candidate_short_validation_duration_s;
      gate.required_post_feedback_stable_updates = 0;
    }
    gate.derive_depth_from_sliding_window = false;
    derived_gate_depth = true;
  }
  candidate_gate_depth_counts_ = window_measurement_counts;
  candidate_gate_depth_source_ =
      derived_gate_depth ? "short_causal_duration_with_observed_fc_rows"
                         : "explicit_gate_configuration";
  candidate_filter_ = OnlineAlignmentCandidateFilter(candidate_filter_config);
  std::string candidate_filter_reason;
  if (!candidate_filter_.initialize(
          result.timestamp + options_.camera_to_imu_time_offset_s,
          candidate_nominal, result.covariance, graph_support,
          CandidateErrorState::Zero(), &candidate_filter_reason)) {
    fail_retry(now, "candidate_filter_initialization:" +
                        candidate_filter_reason);
    return false;
  }

  candidate_created_time_ = now;
  candidate_reference_snapshot_ = make_visual_snapshot(*usable_frames.back());
  candidate_visual_snapshots_.clear();
  candidate_validation_frames_ = 0;
  candidate_visual_p95_max_ = 0.0;
  candidate_latest_visual_compensated_p95_px_ =
      std::numeric_limits<double>::infinity();
  candidate_latest_visual_reprojection_p95_px_ =
      std::numeric_limits<double>::infinity();
  candidate_latest_visual_reprojection_p50_px_ =
      std::numeric_limits<double>::infinity();
  candidate_last_validation_time_ = -1.0;
  candidate_validation_times_.clear();
  candidate_rotation_residuals_deg_.clear();
  candidate_position_residuals_m_.clear();
  candidate_velocity_residuals_mps_.clear();
  candidate_visual_reprojection_residuals_px_.clear();
  candidate_visual_frame_p95_px_.clear();
  candidate_active_ = true;
  ++candidate_created_count_;
  transition(AlignmentPhase::CANDIDATE_VALIDATING, now,
             "joint_solution_became_validation_candidate");
  copy_runtime_counters(diag);
  result.diagnostics = diag;
  candidate_result_ = result;
  candidate_record_ = AlignmentCandidate();
  candidate_record_.result = result;
  candidate_record_.solve_window_start = diag.window_start;
  candidate_record_.solve_window_end = diag.init_time;
  for (const auto *frame : usable_frames)
    candidate_record_.selected_frame_timestamps.push_back(
        frame->left_timestamp);
  candidate_record_.selected_feature_ids.assign(fingerprint_features.begin(),
                                                fingerprint_features.end());
  for (size_t index = 0;
       index < landmarks.size() && index < landmark_feature_ids.size(); ++index)
    candidate_record_.optimized_landmarks_G[landmark_feature_ids[index]] =
        Eigen::Vector3d(landmarks[index][0], landmarks[index][1],
                        landmarks[index][2]);
  candidate_record_.factor_contributions = diag.factor_contributions;
  candidate_record_.state_observability = diag.state_observability;
  candidate_record_.prior_dominated_states = diag.fixed_state_list;
  candidate_record_.prior_dominated_states.insert(
      candidate_record_.prior_dominated_states.end(), diag.weak_state_list.begin(),
      diag.weak_state_list.end());
  candidate_record_.solve_wall_time_s = diag.initialization_duration_s;
  candidate_record_.provenance = diag.provenance;
  if (!attempt_receipts_.empty()) {
    int factor_count = 0;
    for (const auto &family : diag.factor_contributions)
      factor_count += family.residual_blocks;
    attempt_receipts_.back().factor_count = factor_count;
    attempt_receipts_.back().selected_landmarks =
        diag.visual_statistics.triangulated_landmark_count;
    attempt_receipts_.back().outcome = "candidate_ready";
    attempt_receipts_.back().failed_gate.clear();
    attempt_receipts_.back().next_eligible_condition =
        "bounded_future_validation";
  }
  last_diagnostics_ = diag;
  last_rejection_ = "candidate_future_validation_pending";
  return false;
}

} // namespace ov_msckf
