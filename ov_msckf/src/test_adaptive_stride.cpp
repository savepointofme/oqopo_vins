#include "core/AlignmentFrameSelector.h"
#include "core/BackendUpdateTrigger.h"
#include "core/VisualCadencePlanner.h"
#include "ros_free/AdaptiveVisualScheduler.h"

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <iostream>

namespace {

ov_msckf::VisualMotionMetrics healthy_motion(double dt, double comp_median,
                                             double comp_p95) {
  ov_msckf::VisualMotionMetrics motion;
  motion.valid = true;
  motion.dt_s = dt;
  motion.previous_tracks = 100;
  motion.current_tracks = 95;
  motion.common_tracks = 90;
  motion.effective_track_count = 80.0;
  motion.survival_ratio = 0.90;
  motion.median_track_age = 8.0;
  motion.grid_occupancy_ratio = 0.75;
  motion.grid_entropy = 0.85;
  motion.border_track_ratio = 0.10;
  motion.raw_median_px = comp_median;
  motion.raw_p95_px = std::max(comp_p95, comp_median);
  motion.rotation_median_px = 0.0;
  motion.rotation_p95_px = 0.0;
  motion.rotation_max_px = 0.0;
  motion.compensated_median_px = comp_median;
  motion.compensated_p95_px = comp_p95;
  motion.compensated_sigma_px = 0.5;
  motion.compensated_p95_ucb_px = comp_p95 + 0.1;
  return motion;
}

ov_msckf::ContinuousTrackingPlannerInput
tracking_input(double timestamp, double measured_flow) {
  ov_msckf::ContinuousTrackingPlannerInput input;
  input.timestamp = timestamp;
  input.motion_measurement_timestamp_s = timestamp;
  input.raw_camera_dt_s = 1.0 / 30.0;
  input.motion = healthy_motion(0.1, measured_flow, measured_flow * 1.4);
  return input;
}

} // namespace

int main() {
  using namespace ov_msckf;

  // 1. Rotation-compensated motion retains translation while removing the
  // known camera rotation, and computes spatial/effective support.
  {
    VisualFrameSnapshot previous;
    VisualFrameSnapshot current;
    previous.timestamp = 0.0;
    current.timestamp = 0.1;
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitY()).toRotationMatrix();
    for (size_t id = 0; id < 48; ++id) {
      VisualTrackPoint old;
      old.feature_id = id;
      old.valid = true;
      old.track_age = 8;
      old.normalized = Eigen::Vector2d(
          -0.6 + 0.2 * static_cast<double>(id % 7),
          -0.4 + 0.2 * static_cast<double>((id / 7) % 5));
      old.raw = Eigen::Vector2d(640.0 + 400.0 * old.normalized.x(),
                                360.0 + 400.0 * old.normalized.y());
      Eigen::Vector3d ray(old.normalized.x(), old.normalized.y(), 1.0);
      ray = rotation * ray;
      VisualTrackPoint now = old;
      now.normalized = ray.head<2>() / ray.z() + Eigen::Vector2d(0.002, 0.0);
      now.raw = Eigen::Vector2d(640.0 + 400.0 * now.normalized.x(),
                                360.0 + 400.0 * now.normalized.y());
      previous.tracks.push_back(old);
      current.tracks.push_back(now);
    }
    const auto motion = compute_visual_motion_metrics(
        previous, current, rotation, 400.0, 400.0, 1280, 720);
    assert(motion.valid);
    assert(motion.rotation_median_px > 1.0);
    assert(std::fabs(motion.compensated_median_px - 0.8) < 1e-6);
    assert(motion.effective_track_count > 30.0);
    assert(motion.grid_occupancy_ratio > 0.25);
    assert(motion.compensated_p95_ucb_px >= motion.compensated_p95_px);
  }

  // 2. The planner computes an arbitrary integer gap from a conservative
  // fraction of the complete continuous safety horizon. Six is deliberately
  // absent from the deleted legacy ladder.
  {
    ContinuousTrackingPlanner planner;
    auto input = tracking_input(0.0, 2.1);
    auto first = planner.update(input);
    assert(first.tracking_gap == 1);
    input.timestamp = 0.6;
    input.motion_measurement_timestamp_s = 0.6;
    const auto expanded = planner.update(input);
    assert(expanded.expansion_confirmed);
    assert(std::fabs(expanded.target_horizon_s -
                     0.5 * expanded.safe_horizon_s) < 1e-12);
    assert(expanded.tracking_gap == 6);
  }

  // 3. A new unsafe rate contracts directly, without walking through a
  // sequence of fixed modes or fixed stride choices.
  {
    ContinuousTrackingPlanner planner;
    auto input = tracking_input(0.0, 2.1);
    planner.update(input);
    input.timestamp = 0.6;
    input.motion_measurement_timestamp_s = 0.6;
    assert(planner.update(input).tracking_gap == 6);
    input = tracking_input(0.7, 10.0);
    const auto contracted = planner.update(input);
    assert(contracted.immediate_contraction);
    assert(contracted.tracking_gap < 6);
  }

  // 4. Re-reading the same KLT pair does not advance expansion confirmation.
  {
    ContinuousTrackingPlanner planner;
    auto input = tracking_input(0.0, 2.1);
    planner.update(input);
    input.timestamp = 0.4;
    const auto held = planner.update(input);
    assert(!held.new_motion_measurement);
    assert(held.tracking_gap == 1);
    assert(held.reason == "hold_until_new_motion_measurement");
  }

  // 5. A stale motion packet forces dense tracking on wall-clock time.
  {
    ContinuousTrackingPlanner planner;
    auto input = tracking_input(0.0, 2.1);
    planner.update(input);
    input.timestamp = 1.0;
    const auto stale = planner.update(input);
    assert(stale.tracking_gap == 1);
    assert(stale.reason == "motion_stale");
  }

  // 6. Poor image coverage is a protection condition, not a flight-state
  // label mapped to a preselected cadence.
  {
    ContinuousTrackingPlanner planner;
    auto input = tracking_input(0.0, 2.1);
    input.motion.grid_occupancy_ratio = 0.1;
    const auto decision = planner.update(input);
    assert(decision.tracking_gap == 1);
    assert(decision.reason == "insufficient_spatial_support");
  }

  // 7. The outer scheduler consumes estimator health only as a dense safety
  // override; it has no HIGH/NORMAL/TURN/LOW stride table.
  {
    AdaptiveVisualScheduler scheduler;
    AdaptiveVisualSchedulerInput input;
    input.timestamp = 0.0;
    input.initialized = true;
    input.raw_camera_dt_s = 1.0 / 30.0;
    input.motion_measurement_timestamp_s = 0.0;
    input.motion = healthy_motion(0.1, 2.1, 3.0);
    input.active_feature_count = 90;
    input.covariance_all_finite = false;
    const auto protected_decision = scheduler.update(input);
    assert(protected_decision.estimator_protection);
    assert(protected_decision.tracking_gap == 1);
    assert(protected_decision.mode ==
           AdaptiveVisualMode::ESTIMATOR_PROTECTION);

    // State motion between different timestamps is intentionally absent from
    // this interface. Only a correction measured before/after the visual EKF
    // update at one timestamp may trigger the correction safety override.
    AdaptiveVisualScheduler correction_scheduler;
    input.covariance_all_finite = true;
    input.visual_state_correction_valid = true;
    input.visual_position_correction_m = 6.0;
    const auto correction_protected = correction_scheduler.update(input);
    assert(correction_protected.estimator_protection);
    assert(correction_protected.tracking_gap == 1);
  }

  // 8. Backend bootstrap requests a frame but does not by itself claim that a
  // clone/reference has been committed.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.0;
    const auto decision = scheduler.evaluate(input);
    assert(decision.trigger && decision.bootstrap_trigger);
    assert(!backend_reference_commit_allowed(decision.trigger, true, false));
    assert(backend_reference_commit_allowed(decision.trigger, true, true));
  }

  // 9. Healthy translation geometry triggers on information accumulated from
  // the last actual clone.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.4;
    input.last_actual_clone_timestamp = 1.0;
    input.clone_capacity = 10;
    input.motion_from_last_clone = healthy_motion(0.4, 12.0, 16.0);
    const auto decision = scheduler.evaluate(input);
    assert(decision.trigger && decision.information_trigger);
    assert(decision.information_score > 8.0);
    assert(decision.temporal_support_ready);
  }

  // 10. Direct image information alone must not collapse the bounded clone window
  // to a tiny real-time span.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.2;
    input.last_actual_clone_timestamp = 1.0;
    input.clone_capacity = 10;
    input.motion_from_last_clone = healthy_motion(0.2, 16.0, 20.0);
    const auto decision = scheduler.evaluate(input);
    assert(!decision.trigger);
    assert(!decision.temporal_support_ready);
    assert(decision.reason == "accumulate_temporal_window_support");
  }

  // 11. Pure rotation is not mistaken for translation information.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.2;
    input.last_actual_clone_timestamp = 1.0;
    input.motion_from_last_clone = healthy_motion(0.2, 0.5, 1.0);
    input.motion_from_last_clone.rotation_median_px = 12.0;
    input.motion_from_last_clone.rotation_p95_px = 18.0;
    const auto decision = scheduler.evaluate(input);
    assert(!decision.trigger && decision.pure_rotation);
    assert(decision.reason == "accumulate_through_pure_rotation");
  }

  // 12. Maximum real-time latency eventually creates a clone even during
  // pure rotation, and records that visual translation information did not
  // cause the trigger.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.6;
    input.last_actual_clone_timestamp = 1.0;
    input.motion_from_last_clone = healthy_motion(0.6, 0.5, 1.0);
    input.motion_from_last_clone.rotation_median_px = 12.0;
    const auto decision = scheduler.evaluate(input);
    assert(decision.trigger && decision.latency_trigger);
    assert(decision.pure_rotation);
    assert(decision.forced_without_visual_information);
  }

  // 13. Track termination cannot collapse the clone window to the camera
  // period. It is released only after real temporal support is available.
  {
    AdaptiveBackendScheduler scheduler;
    AdaptiveBackendSchedulerInput input;
    input.initialized = true;
    input.timestamp = 1.1;
    input.last_actual_clone_timestamp = 1.0;
    input.clone_capacity = 10;
    input.motion_from_last_clone = healthy_motion(0.1, 1.0, 2.0);
    input.motion_from_last_clone.common_tracks = 10;
    const auto early = scheduler.evaluate(input);
    assert(!early.trigger);
    assert(!early.temporal_support_ready);
    assert(early.reason == "accumulate_termination_temporal_support");

    input.timestamp = 1.4;
    input.motion_from_last_clone.dt_s = 0.4;
    const auto supported = scheduler.evaluate(input);
    assert(supported.trigger && supported.termination_trigger);
    assert(supported.temporal_support_ready);
    assert(!supported.forced_without_visual_information);
  }

  // 14. Reliable height enters as continuous projected-footprint geometry;
  // it does not select a height-band cadence.
  {
    GroundFootprintPredictionInput footprint;
    footprint.valid = true;
    footprint.R_GtoC = Eigen::Matrix3d::Identity();
    footprint.p_CinG = Eigen::Vector3d(0.0, 0.0, 10.0);
    footprint.velocity_G = Eigen::Vector3d(2.0, 0.0, 0.0);
    footprint.ground_height_G = 0.0;
    footprint.corner_rays_C = {
        Eigen::Vector3d(-1.0, -1.0, -1.0),
        Eigen::Vector3d(1.0, -1.0, -1.0),
        Eigen::Vector3d(1.0, 1.0, -1.0),
        Eigen::Vector3d(-1.0, 1.0, -1.0)};
    const double near_overlap = ground_footprint_overlap(footprint, 0.1);
    const double far_overlap = ground_footprint_overlap(footprint, 1.0);
    assert(std::isfinite(near_overlap) && std::isfinite(far_overlap));
    assert(near_overlap > far_overlap);

    ContinuousTrackingPlannerConfig config;
    config.upshift_confirmation_duration_s = 0.0;
    ContinuousTrackingPlanner low_height_planner(config);
    ContinuousTrackingPlanner high_height_planner(config);
    auto low_input = tracking_input(0.0, 0.1);
    low_input.ground_footprint = footprint;
    low_input.ground_footprint.velocity_G =
        Eigen::Vector3d(20.0, 0.0, 0.0);
    auto high_input = low_input;
    high_input.ground_footprint.p_CinG.z() = 100.0;
    const auto low_decision = low_height_planner.update(low_input);
    const auto high_decision = high_height_planner.update(high_input);
    assert(low_decision.ground_geometry_valid);
    assert(high_decision.ground_geometry_valid);
    assert(low_decision.tracking_gap < high_decision.tracking_gap);
  }

  // 15. VioManager's explicit termination input is derived from actual track
  // survival and border occupancy, including cases where full motion geometry
  // is unavailable.
  {
    VisualFrameSnapshot reference;
    VisualFrameSnapshot healthy;
    VisualFrameSnapshot terminating;
    for (size_t id = 0; id < 40; ++id) {
      VisualTrackPoint track;
      track.feature_id = id;
      track.raw = Eigen::Vector2d(100.0 + id, 100.0);
      track.valid = true;
      reference.tracks.push_back(track);
      healthy.tracks.push_back(track);
      if (id < 10)
        terminating.tracks.push_back(track);
    }
    AdaptiveBackendScheduler scheduler;
    assert(!backend_feature_termination_imminent(
        reference, healthy, 640, 480, scheduler.config()));
    assert(backend_feature_termination_imminent(
        reference, terminating, 640, 480, scheduler.config()));
    for (VisualTrackPoint &track : healthy.tracks)
      track.raw = Eigen::Vector2d(2.0, 2.0);
    assert(backend_feature_termination_imminent(
        reference, healthy, 640, 480, scheduler.config()));
  }

  // 16. P4 frame selection remains a separate contract from P5 cadence.
  {
    AlignmentFrameSelector selector;
    AlignmentFrameSelectionInput input;
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
  }

  std::cout << "continuous adaptive visual scheduling tests 1-15 passed\n";
  return 0;
}
