/*
 * External absolute-anchor contract for ROS-free recovery supervision.
 *
 * This header is intentionally estimator-agnostic. It validates source quality,
 * timing, covariance, and optional innovation checks, but it does not fuse or
 * reset the EKF by itself.
 */

#pragma once

#include <Eigen/Eigen>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace ov_msckf {

enum class ExternalAnchorSource {
  SATELLITE_MAP_MATCH,
  FC_NAV,
  GNSS_COORDINATE,
  SIM_GPS_DIAGNOSTIC,
  MANUAL
};

enum ExternalAnchorField : uint32_t {
  HAS_POSITION = 1u << 0,
  HAS_YAW = 1u << 1,
  HAS_ATTITUDE = 1u << 2,
  HAS_VELOCITY = 1u << 3,
  HAS_BIAS = 1u << 4
};

enum class AnchorAction {
  REJECT,
  LOG_ONLY,
  EKF_UPDATE,
  TRUSTED_SOFT_RESET,
  HARD_REINIT
};

enum AnchorRejectReason : uint32_t {
  ANCHOR_REJECT_NONE = 0u,
  ANCHOR_REJECT_STALE = 1u << 0,
  ANCHOR_REJECT_FUTURE = 1u << 1,
  ANCHOR_REJECT_MISSING_POSITION = 1u << 2,
  ANCHOR_REJECT_BAD_COVARIANCE = 1u << 3,
  ANCHOR_REJECT_LOW_CONFIDENCE = 1u << 4,
  ANCHOR_REJECT_LOW_INLIERS = 1u << 5,
  ANCHOR_REJECT_BAD_FIX = 1u << 6,
  ANCHOR_REJECT_LOW_SATELLITES = 1u << 7,
  ANCHOR_REJECT_HIGH_PDOP = 1u << 8,
  ANCHOR_REJECT_HIGH_ACCURACY_ERROR = 1u << 9,
  ANCHOR_REJECT_MISSING_GNSS_QUALITY = 1u << 10,
  ANCHOR_REJECT_INNOVATION = 1u << 11,
  ANCHOR_REJECT_TRUST_REGION = 1u << 12
};

struct ExternalAnchorMeasurement {
  double timestamp = -1.0;
  ExternalAnchorSource source = ExternalAnchorSource::GNSS_COORDINATE;
  uint32_t valid_fields = 0;

  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_GtoI = Eigen::Vector4d(0, 0, 0, 1);
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity();
  double yaw_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity();

  double confidence = std::numeric_limits<double>::quiet_NaN();
  double match_score = std::numeric_limits<double>::quiet_NaN();
  int inlier_count = -1;
  double pdop = std::numeric_limits<double>::quiet_NaN();
  double hdop = std::numeric_limits<double>::quiet_NaN();
  double vdop = std::numeric_limits<double>::quiet_NaN();
  double eph = std::numeric_limits<double>::quiet_NaN();
  double epv = std::numeric_limits<double>::quiet_NaN();
  double sacc = std::numeric_limits<double>::quiet_NaN();
  int satellites = -1;
  int fix_type = -1;
};

struct AnchorInnovationCheck {
  bool available = false;
  double test_ratio = std::numeric_limits<double>::quiet_NaN();
  bool trust_region_pass = true;
};

struct AnchorTrustConfig {
  double max_anchor_age_s = 2.0;
  double max_future_time_s = 0.02;
  double min_pos_sigma_m = 0.50;
  double max_pos_sigma_m = 200.0;
  double min_yaw_sigma_rad = 0.017453292519943295;
  double max_yaw_sigma_rad = 3.14159265358979323846;

  double min_satellite_map_confidence = 0.50;
  int min_satellite_map_inliers = 20;

  int min_gnss_fix_type = 3;
  int min_gnss_satellites = 6;
  double max_gnss_pdop = 4.0;
  double max_gnss_eph_m = 20.0;
  double max_gnss_epv_m = 30.0;
  double max_gnss_sacc_mps = 5.0;
  bool require_gnss_quality_for_use = true;

  bool allow_fc_nav_reinit = true;
  bool allow_gnss_update = false;
  bool allow_gnss_reinit = false;
  bool allow_sim_gps_diagnostic_use = false;
  bool allow_manual_reinit = false;
};

struct AidSourceStatus {
  double sample_time = -1.0;
  double filter_time = -1.0;
  double age_s = std::numeric_limits<double>::infinity();
  ExternalAnchorSource source = ExternalAnchorSource::GNSS_COORDINATE;
  AnchorAction requested_action = AnchorAction::LOG_ONLY;
  AnchorAction recommended_action = AnchorAction::REJECT;

  double source_confidence = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d covariance_in = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d covariance_used = Eigen::Matrix3d::Identity();
  double yaw_sigma_used_rad = std::numeric_limits<double>::quiet_NaN();
  double test_ratio = std::numeric_limits<double>::quiet_NaN();

  bool quality_pass = false;
  bool time_pass = false;
  bool covariance_pass = false;
  bool innovation_pass = true;
  bool trust_region_pass = true;
  bool accepted = false;
  uint32_t reject_reasons = ANCHOR_REJECT_NONE;
  std::string reject_reason = "none";
};

inline bool anchor_has_field(const ExternalAnchorMeasurement &m, ExternalAnchorField field) {
  return (m.valid_fields & static_cast<uint32_t>(field)) != 0;
}

inline const char *external_anchor_source_name(ExternalAnchorSource source) {
  switch (source) {
  case ExternalAnchorSource::SATELLITE_MAP_MATCH:
    return "satellite_map_match";
  case ExternalAnchorSource::FC_NAV:
    return "fc_nav";
  case ExternalAnchorSource::GNSS_COORDINATE:
    return "gnss_coordinate";
  case ExternalAnchorSource::SIM_GPS_DIAGNOSTIC:
    return "sim_gps_diagnostic";
  case ExternalAnchorSource::MANUAL:
    return "manual";
  }
  return "unknown";
}

inline const char *anchor_action_name(AnchorAction action) {
  switch (action) {
  case AnchorAction::REJECT:
    return "reject";
  case AnchorAction::LOG_ONLY:
    return "log_only";
  case AnchorAction::EKF_UPDATE:
    return "ekf_update";
  case AnchorAction::TRUSTED_SOFT_RESET:
    return "trusted_soft_reset";
  case AnchorAction::HARD_REINIT:
    return "hard_reinit";
  }
  return "unknown";
}

inline int anchor_action_rank(AnchorAction action) {
  switch (action) {
  case AnchorAction::REJECT:
    return 0;
  case AnchorAction::LOG_ONLY:
    return 1;
  case AnchorAction::EKF_UPDATE:
    return 2;
  case AnchorAction::TRUSTED_SOFT_RESET:
    return 3;
  case AnchorAction::HARD_REINIT:
    return 4;
  }
  return 0;
}

inline AnchorAction min_anchor_action(AnchorAction a, AnchorAction b) {
  return anchor_action_rank(a) <= anchor_action_rank(b) ? a : b;
}

inline bool finite_matrix3(const Eigen::Matrix3d &m) {
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (!std::isfinite(m(r, c)))
        return false;
    }
  }
  return true;
}

inline void add_anchor_reject(AidSourceStatus &status, uint32_t reason, const std::string &name) {
  status.reject_reasons |= reason;
  if (status.reject_reason == "none")
    status.reject_reason = name;
  else
    status.reject_reason += "|" + name;
}

inline bool sanitize_anchor_position_covariance(const ExternalAnchorMeasurement &m,
                                                const AnchorTrustConfig &cfg,
                                                AidSourceStatus &status) {
  status.covariance_in = m.R_pos;
  if (!finite_matrix3(m.R_pos))
    return false;
  Eigen::Matrix3d cov = 0.5 * (m.R_pos + m.R_pos.transpose());
  const double min_var = cfg.min_pos_sigma_m * cfg.min_pos_sigma_m;
  const double max_var = cfg.max_pos_sigma_m * cfg.max_pos_sigma_m;
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(cov(i, i)) || cov(i, i) <= 0.0 || cov(i, i) > max_var)
      return false;
    cov(i, i) = std::max(cov(i, i), min_var);
  }
  status.covariance_used = cov;
  return true;
}

inline AnchorAction max_action_for_source(ExternalAnchorSource source,
                                          const AnchorTrustConfig &cfg) {
  switch (source) {
  case ExternalAnchorSource::SATELLITE_MAP_MATCH:
    return AnchorAction::HARD_REINIT;
  case ExternalAnchorSource::FC_NAV:
    return cfg.allow_fc_nav_reinit ? AnchorAction::HARD_REINIT : AnchorAction::EKF_UPDATE;
  case ExternalAnchorSource::GNSS_COORDINATE:
    if (cfg.allow_gnss_reinit)
      return AnchorAction::HARD_REINIT;
    if (cfg.allow_gnss_update)
      return AnchorAction::EKF_UPDATE;
    return AnchorAction::LOG_ONLY;
  case ExternalAnchorSource::SIM_GPS_DIAGNOSTIC:
    return cfg.allow_sim_gps_diagnostic_use ? AnchorAction::HARD_REINIT
                                            : AnchorAction::LOG_ONLY;
  case ExternalAnchorSource::MANUAL:
    return cfg.allow_manual_reinit ? AnchorAction::HARD_REINIT : AnchorAction::LOG_ONLY;
  }
  return AnchorAction::REJECT;
}

inline AidSourceStatus evaluate_anchor_trust(
    const ExternalAnchorMeasurement &m, double filter_time, AnchorAction requested_action,
    const AnchorTrustConfig &cfg = AnchorTrustConfig(),
    const AnchorInnovationCheck &innovation = AnchorInnovationCheck()) {
  AidSourceStatus status;
  status.sample_time = m.timestamp;
  status.filter_time = filter_time;
  status.age_s = filter_time - m.timestamp;
  status.source = m.source;
  status.requested_action = requested_action;
  status.source_confidence = m.confidence;

  if (!std::isfinite(m.timestamp) || !std::isfinite(filter_time) ||
      status.age_s > cfg.max_anchor_age_s) {
    add_anchor_reject(status, ANCHOR_REJECT_STALE, "stale");
  }
  if (std::isfinite(status.age_s) && status.age_s < -cfg.max_future_time_s)
    add_anchor_reject(status, ANCHOR_REJECT_FUTURE, "future_sample");
  status.time_pass = (status.reject_reasons &
                      (ANCHOR_REJECT_STALE | ANCHOR_REJECT_FUTURE)) == 0;

  if (!anchor_has_field(m, HAS_POSITION))
    add_anchor_reject(status, ANCHOR_REJECT_MISSING_POSITION, "missing_position");
  if (!sanitize_anchor_position_covariance(m, cfg, status))
    add_anchor_reject(status, ANCHOR_REJECT_BAD_COVARIANCE, "bad_covariance");
  status.covariance_pass =
      (status.reject_reasons & ANCHOR_REJECT_BAD_COVARIANCE) == 0;

  if (anchor_has_field(m, HAS_YAW)) {
    double yaw_sigma = m.yaw_sigma_rad;
    if (!std::isfinite(yaw_sigma) || yaw_sigma <= 0.0)
      yaw_sigma = cfg.max_yaw_sigma_rad;
    status.yaw_sigma_used_rad =
        std::max(cfg.min_yaw_sigma_rad, std::min(cfg.max_yaw_sigma_rad, yaw_sigma));
  }

  if (m.source == ExternalAnchorSource::SATELLITE_MAP_MATCH) {
    if (std::isfinite(m.confidence) && m.confidence < cfg.min_satellite_map_confidence)
      add_anchor_reject(status, ANCHOR_REJECT_LOW_CONFIDENCE, "low_map_confidence");
    if (m.inlier_count >= 0 && m.inlier_count < cfg.min_satellite_map_inliers)
      add_anchor_reject(status, ANCHOR_REJECT_LOW_INLIERS, "low_map_inliers");
  }

  const bool is_gnss_like = m.source == ExternalAnchorSource::GNSS_COORDINATE ||
                            m.source == ExternalAnchorSource::FC_NAV ||
                            m.source == ExternalAnchorSource::SIM_GPS_DIAGNOSTIC;
  if (is_gnss_like) {
    const bool has_quality = m.fix_type >= 0 || m.satellites >= 0 || std::isfinite(m.pdop) ||
                             std::isfinite(m.eph) || std::isfinite(m.epv) ||
                             std::isfinite(m.sacc);
    if (m.source == ExternalAnchorSource::GNSS_COORDINATE &&
        cfg.require_gnss_quality_for_use &&
        anchor_action_rank(requested_action) > anchor_action_rank(AnchorAction::LOG_ONLY) &&
        !has_quality) {
      add_anchor_reject(status, ANCHOR_REJECT_MISSING_GNSS_QUALITY,
                        "missing_gnss_quality");
    }
    if (m.fix_type >= 0 && m.fix_type < cfg.min_gnss_fix_type)
      add_anchor_reject(status, ANCHOR_REJECT_BAD_FIX, "bad_fix");
    if (m.satellites >= 0 && m.satellites < cfg.min_gnss_satellites)
      add_anchor_reject(status, ANCHOR_REJECT_LOW_SATELLITES, "low_satellites");
    if (std::isfinite(m.pdop) && m.pdop > cfg.max_gnss_pdop)
      add_anchor_reject(status, ANCHOR_REJECT_HIGH_PDOP, "high_pdop");
    if (std::isfinite(m.eph) && m.eph > cfg.max_gnss_eph_m)
      add_anchor_reject(status, ANCHOR_REJECT_HIGH_ACCURACY_ERROR, "high_eph");
    if (std::isfinite(m.epv) && m.epv > cfg.max_gnss_epv_m)
      add_anchor_reject(status, ANCHOR_REJECT_HIGH_ACCURACY_ERROR, "high_epv");
    if (std::isfinite(m.sacc) && m.sacc > cfg.max_gnss_sacc_mps)
      add_anchor_reject(status, ANCHOR_REJECT_HIGH_ACCURACY_ERROR, "high_sacc");
  }

  if (innovation.available) {
    status.test_ratio = innovation.test_ratio;
    status.innovation_pass = std::isfinite(innovation.test_ratio) &&
                             innovation.test_ratio <= 1.0;
    status.trust_region_pass = innovation.trust_region_pass;
    if (!status.innovation_pass)
      add_anchor_reject(status, ANCHOR_REJECT_INNOVATION, "innovation_gate");
    if (!status.trust_region_pass)
      add_anchor_reject(status, ANCHOR_REJECT_TRUST_REGION, "trust_region");
  }

  status.quality_pass = status.reject_reasons == ANCHOR_REJECT_NONE;
  if (!status.quality_pass) {
    status.recommended_action = AnchorAction::REJECT;
    return status;
  }

  status.recommended_action =
      min_anchor_action(requested_action, max_action_for_source(m.source, cfg));
  status.accepted =
      anchor_action_rank(status.recommended_action) >= anchor_action_rank(AnchorAction::EKF_UPDATE);
  return status;
}

} // namespace ov_msckf
