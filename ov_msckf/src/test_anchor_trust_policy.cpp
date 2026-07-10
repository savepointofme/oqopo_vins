#include "ros_free/ExternalAnchor.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using ov_msckf::AidSourceStatus;
using ov_msckf::AnchorAction;
using ov_msckf::AnchorInnovationCheck;
using ov_msckf::AnchorTrustConfig;
using ov_msckf::ExternalAnchorMeasurement;
using ov_msckf::ExternalAnchorSource;
using ov_msckf::HAS_POSITION;
using ov_msckf::HAS_YAW;
using ov_msckf::evaluate_anchor_trust;

static constexpr double kPi = 3.14159265358979323846;

static ExternalAnchorMeasurement base_anchor(ExternalAnchorSource source) {
  ExternalAnchorMeasurement m;
  m.timestamp = 10.0;
  m.source = source;
  m.valid_fields = HAS_POSITION | HAS_YAW;
  m.p_IinG = Eigen::Vector3d(1.0, 2.0, 3.0);
  m.R_pos = Eigen::Vector3d(2.0, 2.0, 3.0).array().square().matrix().asDiagonal();
  m.yaw_sigma_rad = 5.0 * kPi / 180.0;
  return m;
}

static ExternalAnchorMeasurement good_satellite_anchor() {
  auto m = base_anchor(ExternalAnchorSource::SATELLITE_MAP_MATCH);
  m.confidence = 0.85;
  m.match_score = 0.90;
  m.inlier_count = 80;
  return m;
}

static ExternalAnchorMeasurement good_gnss_anchor() {
  auto m = base_anchor(ExternalAnchorSource::GNSS_COORDINATE);
  m.fix_type = 3;
  m.satellites = 12;
  m.pdop = 1.8;
  m.eph = 2.0;
  m.epv = 3.0;
  m.sacc = 0.4;
  return m;
}

static bool has_reason(const AidSourceStatus &s, uint32_t reason) {
  return (s.reject_reasons & reason) != 0;
}

int main() {
  {
    auto s = evaluate_anchor_trust(good_satellite_anchor(), 10.5,
                                   AnchorAction::HARD_REINIT);
    assert(s.time_pass);
    assert(s.quality_pass);
    assert(s.covariance_pass);
    assert(s.recommended_action == AnchorAction::HARD_REINIT);
    assert(s.accepted);
  }

  {
    auto m = good_satellite_anchor();
    m.timestamp = 6.0;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_STALE));
  }

  {
    auto m = good_satellite_anchor();
    m.timestamp = 10.6;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_FUTURE));
  }

  {
    auto m = good_satellite_anchor();
    m.confidence = 0.2;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_LOW_CONFIDENCE));
  }

  {
    auto m = good_satellite_anchor();
    m.R_pos(0, 0) = -1.0;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_BAD_COVARIANCE));
  }

  {
    AnchorTrustConfig cfg;
    auto s = evaluate_anchor_trust(good_gnss_anchor(), 10.5,
                                   AnchorAction::HARD_REINIT, cfg);
    assert(s.quality_pass);
    assert(s.recommended_action == AnchorAction::LOG_ONLY);
    assert(!s.accepted);
  }

  {
    AnchorTrustConfig cfg;
    cfg.allow_gnss_reinit = true;
    auto s = evaluate_anchor_trust(good_gnss_anchor(), 10.5,
                                   AnchorAction::HARD_REINIT, cfg);
    assert(s.quality_pass);
    assert(s.recommended_action == AnchorAction::HARD_REINIT);
    assert(s.accepted);
  }

  {
    AnchorTrustConfig cfg;
    cfg.allow_gnss_reinit = true;
    auto m = good_gnss_anchor();
    m.pdop = 8.0;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT, cfg);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_HIGH_PDOP));
  }

  {
    AnchorTrustConfig cfg;
    cfg.allow_gnss_update = true;
    auto m = base_anchor(ExternalAnchorSource::GNSS_COORDINATE);
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::EKF_UPDATE, cfg);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_MISSING_GNSS_QUALITY));
  }

  {
    auto m = base_anchor(ExternalAnchorSource::FC_NAV);
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.quality_pass);
    assert(s.recommended_action == AnchorAction::HARD_REINIT);
  }

  {
    auto m = base_anchor(ExternalAnchorSource::SIM_GPS_DIAGNOSTIC);
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT);
    assert(s.quality_pass);
    assert(s.recommended_action == AnchorAction::LOG_ONLY);
    AnchorTrustConfig cfg;
    cfg.allow_sim_gps_diagnostic_use = true;
    s = evaluate_anchor_trust(m, 10.5, AnchorAction::HARD_REINIT, cfg);
    assert(s.recommended_action == AnchorAction::HARD_REINIT);
  }

  {
    auto m = good_satellite_anchor();
    AnchorInnovationCheck innovation;
    innovation.available = true;
    innovation.test_ratio = 1.2;
    innovation.trust_region_pass = true;
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::EKF_UPDATE,
                                   AnchorTrustConfig(), innovation);
    assert(s.recommended_action == AnchorAction::REJECT);
    assert(has_reason(s, ov_msckf::ANCHOR_REJECT_INNOVATION));
  }

  {
    auto m = good_satellite_anchor();
    m.R_pos = Eigen::Vector3d(0.01, 0.01, 0.01).array().square().matrix().asDiagonal();
    auto s = evaluate_anchor_trust(m, 10.5, AnchorAction::EKF_UPDATE);
    assert(s.quality_pass);
    assert(std::sqrt(s.covariance_used(0, 0)) >= 0.50);
  }

  std::cout << "anchor trust policy tests passed\n";
  return 0;
}
