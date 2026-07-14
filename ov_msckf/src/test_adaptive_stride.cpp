#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "core/BackendUpdateTrigger.h"
#include "core/AlignmentFrameSelector.h"
#include "core/VisualCadencePlanner.h"
#include "ros_free/AdaptiveStrideController.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>

using ov_msckf::AdaptivePolicyState;
using ov_msckf::AdaptiveStrideController;
using ov_msckf::AdaptiveStrideDecision;
using ov_msckf::AdaptiveStrideInput;

namespace {

AdaptiveStrideInput healthy(double t, double height = 180.0) {
  AdaptiveStrideInput in;
  in.timestamp = t;
  in.initialized = true;
  in.relative_height_m = height;
  in.horizontal_speed_mps = 35.0;
  in.vertical_speed_mps = 0.0;
  in.roll_deg = 1.0;
  in.pitch_deg = 2.0;
  in.gyro_norm_radps = 0.03;
  in.actual_received_camera_dt_s = 1.0 / 30.0;
  in.active_msckf_features = 320;
  in.active_slam_features = 40;
  in.configured_feature_count = 400;
  in.median_track_age_frames = 12.0;
  in.msckf_input_count = 20;
  in.msckf_accepted_count = 10;
  in.msckf_rejected_count = 10;
  in.time_since_accepted_backend_update_s = 0.2;
  in.parallax_measurement_timestamp_s = t;
  in.parallax_measurement_dt_s = 0.1;
  in.median_parallax_px = 0.5;
  in.p95_parallax_px = 0.8;
  in.raw_median_parallax_px = 0.6;
  in.raw_p95_parallax_px = 1.0;
  in.rotation_median_parallax_px = 0.1;
  in.rotation_p95_parallax_px = 0.2;
  in.track_survival_ratio = 0.8;
  in.common_track_count = 300;
  return in;
}

AdaptiveStrideDecision run_until(AdaptiveStrideController &controller,
                                 double t0, double t1, double dt,
                                 double height = 180.0) {
  AdaptiveStrideDecision out;
  for (double t = t0; t <= t1 + 1.0e-9; t += dt)
    out = controller.update(healthy(t, height));
  return out;
}

AdaptiveStrideDecision enter_high(AdaptiveStrideController &controller) {
  return run_until(controller, 0.0, 7.0, 0.1, 180.0);
}

void add_flow(AdaptiveStrideInput &in, double tracker_dt, double flow_px,
              double p95_px) {
  in.parallax_measurement_timestamp_s = in.timestamp;
  in.parallax_measurement_dt_s = tracker_dt;
  in.median_parallax_px = flow_px;
  in.p95_parallax_px = p95_px;
}

} // namespace

int main() {
  // 1. Stable low-motion high-altitude cruise reaches the target-parallax cap.
  {
    AdaptiveStrideController c;
    const auto d = enter_high(c);
    assert(d.state == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE);
    assert(d.tracking_stride == 12 && d.backend_update_stride == 12);
  }

  // 2. Descent causes an anticipatory downshift before low altitude.
  {
    AdaptiveStrideController c;
    enter_high(c);
    AdaptiveStrideDecision d;
    for (int i = 71; i <= 100; ++i) {
      auto in = healthy(0.1 * i, 180.0);
      in.vertical_speed_mps = -3.0;
      d = c.update(in);
    }
    assert(d.state == AdaptivePolicyState::DESCENT_SAFETY);
    assert(d.backend_update_stride == 4);
    assert(d.reason_descent);
  }

  // 3. Low altitude forces the registered safe cadence and sub-band.
  {
    AdaptiveStrideController c;
    enter_high(c);
    const auto d = c.update(healthy(7.1, 15.0));
    assert(d.state == AdaptivePolicyState::LOW_ALTITUDE_SAFETY);
    assert(d.low_altitude_band == 3);
    assert(d.tracking_stride == 1 && d.backend_update_stride == 1);
    assert(d.main_trigger == "low_altitude");
  }

  // 4. High angular rate enters turn safety immediately.
  {
    AdaptiveStrideController c;
    enter_high(c);
    auto in = healthy(7.1, 180.0);
    in.gyro_norm_radps = 0.35;
    in.roll_deg = 12.0;
    const auto d = c.update(in);
    assert(d.state == AdaptivePolicyState::TURN_SAFETY);
    assert(d.main_trigger == "severe_turn");
    assert(d.tracking_stride == 1 && d.backend_update_stride == 2);
  }

  // 5. Degraded visual health triggers an emergency downshift.
  {
    AdaptiveStrideController c;
    enter_high(c);
    AdaptiveStrideDecision d;
    bool saw_emergency = false;
    for (int i = 71; i <= 85; ++i) {
      auto in = healthy(0.1 * i, 180.0);
      in.active_msckf_features = 100;
      d = c.update(in);
      saw_emergency = saw_emergency || d.emergency_downshift;
    }
    assert(d.state == AdaptivePolicyState::VISUAL_DEGRADED);
    assert(saw_emergency);
    assert(d.main_trigger == "feature_emergency");
    assert(d.tracking_stride == 1 && d.backend_update_stride == 1);
  }

  // 6. Recovery requires sustained stability.
  {
    AdaptiveStrideController c;
    enter_high(c);
    auto turn = healthy(7.1, 180.0);
    turn.gyro_norm_radps = 0.35;
    turn.roll_deg = 12.0;
    assert(c.update(turn).state == AdaptivePolicyState::TURN_SAFETY);
    auto d = run_until(c, 7.2, 10.9, 0.1, 180.0);
    assert(d.state == AdaptivePolicyState::TURN_SAFETY);
    d = c.update(healthy(12.8, 180.0));
    assert(d.state == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE);
  }

  // 7. Recovery publishes one bounded target, not a rung ladder.
  {
    AdaptiveStrideController c;
    enter_high(c);
    auto turn = healthy(7.1, 180.0);
    turn.gyro_norm_radps = 0.35;
    turn.roll_deg = 12.0;
    c.update(turn);
    int recovery_changes = 0;
    AdaptiveStrideDecision recovered;
    for (int i = 72; i <= 140; ++i) {
      const auto d = c.update(healthy(0.1 * i, 180.0));
      if (d.normal_recovery) {
        recovery_changes++;
        recovered = d;
      }
    }
    assert(recovery_changes == 1);
    assert(recovered.state == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE);
    assert(recovered.backend_update_stride == 12);
  }

  // 8. Noisy turn threshold input does not cause repeated reversal.
  {
    AdaptiveStrideController c;
    enter_high(c);
    int transitions = 0;
    for (int i = 71; i <= 170; ++i) {
      auto in = healthy(0.1 * i, 180.0);
      in.roll_deg = (i % 2 == 0) ? 5.0 : 7.0;
      const auto d = c.update(in);
      transitions += d.policy_state_changed ? 1 : 0;
      assert(!d.unnecessary_reversal);
    }
    assert(transitions == 0);
    assert(c.policy_state() == AdaptivePolicyState::HIGH_ALTITUDE_CRUISE);
  }

  // 9. Pixel displacement uses actual raw-camera dt.
  {
    AdaptiveStrideController c;
    auto in = healthy(0.0, 80.0);
    in.actual_received_camera_dt_s = 0.02;
    add_flow(in, 0.10, 10.0, 15.0);
    auto d = c.update(in);
    assert(std::fabs(d.normalized_parallax_px - 2.0) < 1.0e-9);
    in.timestamp = 0.1;
    in.actual_received_camera_dt_s = 0.04;
    add_flow(in, 0.10, 10.0, 15.0);
    d = c.update(in);
    assert(std::fabs(d.normalized_parallax_px - 4.0) < 1.0e-9);
  }

  // 10. Previous recommendation is not used as the measured interval.
  {
    AdaptiveStrideController sparse;
    enter_high(sparse);
    auto in = healthy(7.1, 180.0);
    in.actual_received_camera_dt_s = 1.0 / 30.0;
    add_flow(in, 0.2, 12.0, 18.0);
    const auto d = sparse.update(in);
    assert(d.backend_update_stride == 12);
    assert(std::fabs(d.normalized_parallax_px -
                     d.filtered_parallax_rate_pxps *
                         in.actual_received_camera_dt_s) < 1.0e-9);
  }

  // 11. Tracking and backend cadences are independently controlled.
  {
    AdaptiveStrideController c;
    enter_high(c);
    auto input = healthy(7.1, 180.0);
    add_flow(input, 0.2, 12.0, 18.0);
    const auto d = c.update(input);
    assert(d.tracking_stride < d.backend_update_stride);
    assert(d.tracking_stride <= d.backend_update_stride);
    assert(d.backend_update_stride == 12);
  }

  // 12/13. A tracking-only timestamp is removed from backend storage while
  // the same feature ID remains continuous across eligible timestamps.
  {
    ov_core::FeatureDatabase db;
    constexpr size_t id = 42;
    db.update_feature(id, 1.0, 0, 10, 20, 0.1f, 0.2f);
    db.update_feature(id, 2.0, 0, 11, 21, 0.11f, 0.21f);
    assert(db.features_containing(2.0).size() == 1);
    db.cleanup_measurements_exact(2.0);
    assert(db.features_containing(2.0).empty());
    assert(db.get_feature(id) != nullptr);
    db.update_feature(id, 3.0, 0, 12, 22, 0.12f, 0.22f);
    const auto feature = db.get_feature(id);
    assert(feature != nullptr && feature->featid == id);
    assert(feature->timestamps.at(0).size() == 2);
    assert(feature->timestamps.at(0).at(0) == 1.0);
    assert(feature->timestamps.at(0).at(1) == 3.0);
  }

  // 14. Emergency downshift bypasses recovery dwell.
  {
    AdaptiveStrideController c;
    enter_high(c);
    auto in = healthy(7.01, 180.0);
    in.covariance_all_finite = false;
    const auto d = c.update(in);
    assert(d.policy_state_changed);
    assert(d.state == AdaptivePolicyState::VISUAL_DEGRADED);
    assert(d.transition_guard_s == 0.0);
  }

  // 15. A stable state never rewrites the same target.
  {
    AdaptiveStrideController c;
    enter_high(c);
    for (int i = 71; i <= 200; ++i) {
      const auto d = c.update(healthy(0.1 * i, 180.0));
      assert(!d.same_state_target_rewrite);
      assert(!d.tracking_stride_changed);
      assert(!d.backend_stride_changed);
    }
  }

  // 16. Transition reasons and metrics are deterministic.
  {
    AdaptiveStrideController a;
    AdaptiveStrideController b;
    enter_high(a);
    enter_high(b);
    auto in = healthy(7.1, 180.0);
    in.active_msckf_features = 100;
    const auto da = a.update(in);
    const auto db = b.update(in);
    assert(da.policy_state == db.policy_state);
    assert(da.main_trigger == db.main_trigger);
    assert(da.transition_reason == db.transition_reason);
    assert(da.emergency_downshift == db.emergency_downshift);
    assert(da.tracking_stride == db.tracking_stride);
    assert(da.backend_update_stride == db.backend_update_stride);
  }

  // 17. Rotation-compensated motion separates raw rotation and translation.
  {
    ov_msckf::VisualFrameSnapshot previous;
    ov_msckf::VisualFrameSnapshot current;
    previous.timestamp = 0.0;
    current.timestamp = 0.1;
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    for (size_t id = 0; id < 20; ++id) {
      ov_msckf::VisualTrackPoint old;
      old.feature_id = id;
      old.normalized = Eigen::Vector2d(-0.2 + 0.02 * id, 0.1);
      old.raw = 400.0 * old.normalized + Eigen::Vector2d(640.0, 360.0);
      old.track_age = 5;
      old.valid = true;
      Eigen::Vector3d ray(old.normalized.x(), old.normalized.y(), 1.0);
      ray = rotation * ray;
      ov_msckf::VisualTrackPoint now = old;
      now.normalized = ray.head<2>() / ray.z() + Eigen::Vector2d(0.002, 0.0);
      now.raw = 400.0 * now.normalized + Eigen::Vector2d(640.0, 360.0);
      previous.tracks.push_back(old);
      current.tracks.push_back(now);
    }
    const auto motion = ov_msckf::compute_visual_motion_metrics(
        previous, current, rotation, 400.0, 400.0);
    assert(motion.valid && motion.rotation_median_px > 1.0);
    assert(std::fabs(motion.compensated_median_px - 0.8) < 1e-6);
  }

  // 18. Target-parallax cadence downshifts immediately and upshifts only
  // after repeated safe observations.
  {
    ov_msckf::VisualCadencePlanner planner;
    ov_msckf::VisualCadencePlannerInput in;
    in.raw_camera_dt_s = 1.0 / 30.0;
    in.safety_stride_cap = 12;
    in.motion.valid = true;
    in.motion.dt_s = 0.1;
    in.motion.common_tracks = 300;
    in.motion.survival_ratio = 0.9;
    in.motion.compensated_median_px = 0.5;
    in.motion.compensated_p95_px = 0.8;
    in.motion.raw_p95_px = 1.0;
    in.motion.rotation_p95_px = 0.2;
    in.timestamp = 0.0;
    in.motion_measurement_timestamp_s = 0.0;
    planner.update(in);
    // Re-reading one KLT packet on skipped raw frames must not manufacture
    // upshift confirmations.
    for (int i = 1; i <= 10; ++i) {
      in.timestamp = 0.03 * i;
      planner.update(in);
      assert(planner.current_stride() == 1);
    }
    for (int i = 1; i <= 3; ++i) {
      in.timestamp = 0.2 * i;
      in.motion_measurement_timestamp_s = in.timestamp;
      planner.update(in);
    }
    assert(planner.current_stride() == 12);
    in.timestamp = 0.7;
    in.motion.valid = false;
    in.motion_measurement_timestamp_s =
        std::numeric_limits<double>::quiet_NaN();
    const auto held = planner.update(in);
    assert(held.stride == 12 &&
           held.reason == "hold_until_new_motion_measurement");
    in.motion.valid = true;
    in.motion.compensated_median_px = 8.0;
    in.motion.compensated_p95_px = 12.0;
    in.motion.raw_p95_px = 15.0;
    in.timestamp = 0.8;
    in.motion_measurement_timestamp_s = 0.8;
    const auto down = planner.update(in);
    assert(down.immediate_downshift && down.stride < 12);
  }

  // 19. Backend update uses accumulated information; frame count is only the
  // elapsed-time fallback.
  {
    ov_msckf::BackendUpdateTrigger trigger;
    ov_msckf::BackendUpdateTriggerInput in;
    in.initialized = true;
    in.timestamp = 1.0;
    in.last_backend_timestamp = 0.8;
    in.motion_from_last_backend.valid = true;
    in.motion_from_last_backend.common_tracks = 100;
    in.motion_from_last_backend.survival_ratio = 0.8;
    in.motion_from_last_backend.compensated_median_px = 6.0;
    auto decision = trigger.evaluate(in);
    assert(decision.trigger && decision.information_trigger &&
           !decision.latency_fallback);
    in.timestamp = 1.4;
    in.motion_from_last_backend.compensated_median_px = 1.0;
    decision = trigger.evaluate(in);
    assert(decision.trigger && decision.latency_fallback);
  }

  // 20. Polygon overlap uses the projected calibrated footprint, not a scalar
  // speed/height approximation.
  {
    ov_msckf::GroundFootprintOverlapInput in;
    in.R_GtoC = Eigen::Matrix3d::Identity();
    in.p_CinG = Eigen::Vector3d(0.0, 0.0, 10.0);
    in.velocity_G = Eigen::Vector3d(1.0, 0.0, 0.0);
    in.ground_height_G = 0.0;
    in.prediction_horizon_s = 1.0;
    in.corner_rays_C = {Eigen::Vector3d(-1.0, -1.0, -1.0),
                        Eigen::Vector3d(1.0, -1.0, -1.0),
                        Eigen::Vector3d(1.0, 1.0, -1.0),
                        Eigen::Vector3d(-1.0, 1.0, -1.0)};
    const double overlap = ov_msckf::ground_footprint_polygon_overlap(in);
    assert(std::isfinite(overlap) && overlap > 0.8 && overlap < 1.0);
  }

  // 21. Alignment frame selection keeps the first/tail or informative frames
  // and rejects redundant/high-rate frames while remaining bounded.
  {
    ov_msckf::AlignmentFrameSelector selector;
    ov_msckf::AlignmentFrameSelectionInput input;
    input.timestamp = 0.0;
    input.common_tracks = 100;
    input.survival_ratio = 0.9;
    assert(selector.evaluate(input).selected);
    input.timestamp = 0.05;
    input.compensated_parallax_px = 0.2;
    assert(!selector.evaluate(input).selected);
    input.timestamp = 0.2;
    input.compensated_parallax_px = 4.0;
    assert(selector.evaluate(input).selected);
    input.timestamp = 1.0;
    input.high_angular_rate = true;
    assert(!selector.evaluate(input).selected);
  }

  std::cout << "P4/P5 visual scheduling tests 1-21 passed\n";
  return 0;
}
