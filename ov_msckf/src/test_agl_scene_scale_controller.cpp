#include "core/AglSceneScaleController.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using ov_msckf::AglSceneScaleController;
using ov_msckf::AglSceneScaleControllerConfig;
using ov_msckf::AglSceneScaleDecision;
using ov_msckf::AglSceneScaleInput;
using ov_msckf::AglSceneScaleResetFeedback;

struct Checks {
  int passed = 0;

  void require(bool condition, const std::string &message) {
    if (!condition)
      throw std::runtime_error(message);
    ++passed;
  }
};

bool near(double lhs, double rhs, double tolerance = 1.0e-10) {
  return std::isfinite(lhs) && std::isfinite(rhs) &&
         std::fabs(lhs - rhs) <= tolerance;
}

AglSceneScaleControllerConfig test_config() {
  AglSceneScaleControllerConfig config;
  config.window_duration_s = 2.0;
  config.maximum_sample_gap_s = 1.1;
  config.maximum_sample_age_s = 0.2;
  config.maximum_log_ratio_mad = 0.02;
  config.minimum_scale_deviation_fraction = 0.05;
  config.cooldown_duration_s = 1.0;
  config.minimum_incremental_scale = 0.80;
  config.maximum_incremental_scale = 1.25;
  config.minimum_cumulative_scale = 0.50;
  config.maximum_cumulative_scale = 2.00;
  return config;
}

AglSceneScaleInput paired_sample(double timestamp_s, double map_height_m,
                                 double agl_m) {
  AglSceneScaleInput input;
  input.causal_timestamp_s = timestamp_s;
  input.sample_timestamp_s = timestamp_s;
  input.map_height_m = map_height_m;
  input.agl_m = agl_m;
  input.map_height_valid = true;
  input.agl_valid = true;
  return input;
}

AglSceneScaleDecision feed_constant_ratio(AglSceneScaleController &controller,
                                          double start_s, double period_s,
                                          int intervals, double map_height_m,
                                          double ratio) {
  AglSceneScaleDecision decision;
  for (int index = 0; index <= intervals; ++index) {
    const double timestamp_s =
        start_s + period_s * static_cast<double>(index);
    decision = controller.update(
        paired_sample(timestamp_s, map_height_m, ratio * map_height_m));
  }
  return decision;
}

AglSceneScaleResetFeedback successful_feedback(double scale) {
  AglSceneScaleResetFeedback feedback;
  feedback.success = true;
  feedback.applied_scale = scale;
  return feedback;
}

void test_window_start_and_stable_release(Checks &checks) {
  AglSceneScaleController controller(test_config());

  const auto first = controller.update(paired_sample(0.0, 100.0, 110.0));
  checks.require(first.window_started, "first valid pair must start the window");
  checks.require(!first.ready, "one endpoint must not release a scale");
  checks.require(first.reason == "window_started",
                 "first pair must be diagnosed as a started time window");
  checks.require(near(first.median_ratio, 1.1),
                 "ratio must be agl_m / map_height_m");
  checks.require(near(first.coverage_s, 0.0),
                 "one endpoint has zero time coverage");

  const auto partial = controller.update(paired_sample(1.0, 100.0, 110.0));
  checks.require(!partial.ready,
                 "a stable but temporally short window must not release");
  checks.require(partial.reason == "window_coverage_insufficient",
                 "short-window rejection reason must be explicit");

  const auto ready = controller.update(paired_sample(2.0, 100.0, 110.0));
  checks.require(ready.ready, "stable full-duration window must release");
  checks.require(near(ready.proposed_scale, 1.1),
                 "h_map=100 and AGL=110 must propose scale 1.1");
  checks.require(near(ready.coverage_s, 2.0),
                 "release must be driven by real elapsed coverage");
  checks.require(near(ready.mad, 0.0),
                 "constant log-ratio must have zero MAD");
}

void test_sampling_rate_invariance(Checks &checks) {
  auto config = test_config();
  config.window_duration_s = 3.0;
  config.maximum_sample_gap_s = 0.55;

  AglSceneScaleController two_hz(config);
  AglSceneScaleController twenty_hz(config);
  const auto sparse =
      feed_constant_ratio(two_hz, 0.0, 0.5, 6, 100.0, 1.1);
  const auto dense =
      feed_constant_ratio(twenty_hz, 0.0, 0.05, 60, 100.0, 1.1);

  checks.require(sparse.ready && dense.ready,
                 "2 Hz and 20 Hz windows must both release");
  checks.require(near(sparse.proposed_scale, dense.proposed_scale),
                 "sampling rate must not change the stable scale conclusion");
  checks.require(near(sparse.coverage_s, 3.0) &&
                     near(dense.coverage_s, 3.0),
                 "both rates must satisfy the same time coverage");
}

void test_gap_and_stability_rejection(Checks &checks) {
  auto gap_config = test_config();
  gap_config.window_duration_s = 3.0;
  gap_config.maximum_sample_gap_s = 0.75;
  AglSceneScaleController gap_controller(gap_config);
  gap_controller.update(paired_sample(0.0, 100.0, 110.0));
  gap_controller.update(paired_sample(0.5, 100.0, 110.0));
  gap_controller.update(paired_sample(1.0, 100.0, 110.0));
  const auto gap =
      gap_controller.update(paired_sample(3.0, 100.0, 110.0));
  checks.require(!gap.ready && gap.reason == "window_gap_too_large",
                 "a time hole must reject an otherwise stable ratio");
  checks.require(near(gap.max_gap_s, 2.0),
                 "diagnostics must expose the rejecting time gap");

  auto stability_config = test_config();
  stability_config.window_duration_s = 3.0;
  stability_config.maximum_sample_gap_s = 1.1;
  AglSceneScaleController stability_controller(stability_config);
  stability_controller.update(paired_sample(0.0, 100.0, 100.0));
  stability_controller.update(paired_sample(1.0, 100.0, 120.0));
  stability_controller.update(paired_sample(2.0, 100.0, 100.0));
  const auto unstable =
      stability_controller.update(paired_sample(3.0, 100.0, 120.0));
  checks.require(!unstable.ready && unstable.reason == "ratio_unstable",
                 "large log-ratio MAD must reject release");
  checks.require(unstable.mad > stability_config.maximum_log_ratio_mad,
                 "MAD diagnostic must exceed the configured stability gate");
}

void test_success_clears_window_and_does_not_repeat(Checks &checks) {
  AglSceneScaleController controller(test_config());
  const auto proposal =
      feed_constant_ratio(controller, 0.0, 1.0, 2, 100.0, 1.1);
  checks.require(proposal.ready && near(proposal.proposed_scale, 1.1),
                 "pre-reset geometry must propose 1.1");

  const auto applied =
      controller.handle_reset_feedback(successful_feedback(1.1));
  checks.require(applied.reason == "reset_applied",
                 "successful StateHelper feedback must be committed");
  checks.require(!applied.window_started,
                 "successful reset must clear the old evidence window");
  checks.require(near(applied.cumulative_scale, 1.1),
                 "successful reset must update cumulative scale");
  checks.require(near(applied.cooldown_remaining_s, 1.0),
                 "successful reset must start a seconds-based cooldown");

  const auto during_cooldown =
      controller.update(paired_sample(2.5, 110.0, 110.0));
  checks.require(!during_cooldown.ready &&
                     during_cooldown.reason == "cooldown_active",
                 "cooldown must be enforced by elapsed seconds");

  controller.update(paired_sample(3.0, 110.0, 110.0));
  controller.update(paired_sample(4.0, 110.0, 110.0));
  const auto post_reset =
      controller.update(paired_sample(4.5, 110.0, 110.0));
  checks.require(!post_reset.ready,
                 "Sim(3)-scaled map height must not trigger the old ratio again");
  checks.require(post_reset.reason == "scale_deviation_below_threshold",
                 "post-reset ratio one must be diagnosed as no scale error");
  checks.require(near(post_reset.median_ratio, 1.0),
                 "post-reset map height must naturally reflect applied scale");
  checks.require(near(post_reset.cumulative_scale, 1.1),
                 "no-repeat path must preserve applied cumulative scale");
}

void test_failed_feedback_preserves_policy_state(Checks &checks) {
  auto config = test_config();
  config.window_duration_s = 1.0;
  config.cooldown_duration_s = 5.0;
  AglSceneScaleController controller(config);
  const auto proposal =
      feed_constant_ratio(controller, 0.0, 1.0, 1, 100.0, 1.1);
  checks.require(proposal.ready, "failure test requires a pending proposal");

  AglSceneScaleResetFeedback failure;
  failure.success = false;
  const auto failed = controller.handle_reset_feedback(failure);
  checks.require(failed.reason == "reset_failed",
                 "failed StateHelper feedback must remain a failure");
  checks.require(failed.window_started,
                 "failed reset must not clear accepted evidence");
  checks.require(near(failed.cumulative_scale, 1.0),
                 "failed reset must not update cumulative scale");
  checks.require(near(failed.cooldown_remaining_s, 0.0),
                 "failed reset must not start cooldown");

  const auto retried = controller.update(paired_sample(2.0, 100.0, 110.0));
  checks.require(retried.ready && near(retried.proposed_scale, 1.1),
                 "new causal evidence may repropose after an honest failure");
}

void test_incremental_and_cumulative_bounds(Checks &checks) {
  auto config = test_config();
  config.window_duration_s = 1.0;
  config.cooldown_duration_s = 0.0;
  config.minimum_scale_deviation_fraction = 0.01;
  config.minimum_incremental_scale = 0.80;
  config.maximum_incremental_scale = 1.25;
  config.minimum_cumulative_scale = 0.70;
  config.maximum_cumulative_scale = 1.30;

  AglSceneScaleController high(config);
  const auto high_rejected =
      feed_constant_ratio(high, 0.0, 1.0, 1, 100.0, 1.30);
  checks.require(!high_rejected.ready &&
                     high_rejected.reason ==
                         "incremental_scale_out_of_bounds",
                 "incremental scale above the closed bound must fail closed");
  checks.require(std::isnan(high_rejected.proposed_scale) &&
                     near(high_rejected.candidate_scale, 1.30),
                 "out-of-range candidate must be diagnosed, never clipped");

  AglSceneScaleController low(config);
  const auto low_rejected =
      feed_constant_ratio(low, 0.0, 1.0, 1, 100.0, 0.79);
  checks.require(!low_rejected.ready &&
                     low_rejected.reason ==
                         "incremental_scale_out_of_bounds",
                 "incremental scale below the closed bound must fail closed");

  AglSceneScaleController upper_edge(config);
  const auto upper_allowed =
      feed_constant_ratio(upper_edge, 0.0, 1.0, 1, 100.0, 1.25);
  checks.require(upper_allowed.ready &&
                     near(upper_allowed.proposed_scale, 1.25),
                 "exact incremental upper boundary must remain admissible");

  AglSceneScaleController lower_edge(config);
  const auto lower_allowed =
      feed_constant_ratio(lower_edge, 0.0, 1.0, 1, 100.0, 0.80);
  checks.require(lower_allowed.ready &&
                     near(lower_allowed.proposed_scale, 0.80),
                 "exact incremental lower boundary must remain admissible");

  auto cumulative_config = config;
  cumulative_config.minimum_incremental_scale = 0.50;
  cumulative_config.maximum_incremental_scale = 2.00;
  cumulative_config.minimum_cumulative_scale = 0.80;
  cumulative_config.maximum_cumulative_scale = 1.20;

  AglSceneScaleController cumulative_high(cumulative_config);
  auto first_high = feed_constant_ratio(cumulative_high, 0.0, 1.0, 1,
                                        100.0, 1.20);
  checks.require(first_high.ready,
                 "exact cumulative upper boundary must be admissible");
  cumulative_high.handle_reset_feedback(successful_feedback(1.20));
  const auto above_cumulative = feed_constant_ratio(
      cumulative_high, 2.0, 1.0, 1, 100.0, 1.10);
  checks.require(!above_cumulative.ready &&
                     above_cumulative.reason ==
                         "cumulative_scale_out_of_bounds",
                 "product above cumulative bound must fail closed");
  checks.require(near(above_cumulative.candidate_scale, 1.10) &&
                     std::isnan(above_cumulative.proposed_scale),
                 "cumulative rejection must not clip the incremental scale");

  AglSceneScaleController cumulative_low(cumulative_config);
  auto first_low = feed_constant_ratio(cumulative_low, 0.0, 1.0, 1,
                                       100.0, 0.80);
  checks.require(first_low.ready,
                 "exact cumulative lower boundary must be admissible");
  cumulative_low.handle_reset_feedback(successful_feedback(0.80));
  const auto below_cumulative = feed_constant_ratio(
      cumulative_low, 2.0, 1.0, 1, 100.0, 0.90);
  checks.require(!below_cumulative.ready &&
                     below_cumulative.reason ==
                         "cumulative_scale_out_of_bounds",
                 "product below cumulative bound must fail closed");
}

void test_invalid_and_noncausal_inputs(Checks &checks) {
  AglSceneScaleController controller(test_config());

  auto input = paired_sample(0.0, 100.0, 110.0);
  input.causal_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  auto decision = controller.update(input);
  checks.require(decision.reason == "causal_timestamp_invalid" &&
                     !decision.window_started,
                 "non-finite causal time must be rejected without state change");

  input = paired_sample(0.0, 100.0, 110.0);
  input.sample_timestamp_s = 0.1;
  decision = controller.update(input);
  checks.require(decision.reason == "sample_timestamp_in_future",
                 "future height pair must be rejected");

  input = paired_sample(1.0, 100.0, 110.0);
  input.sample_timestamp_s = 0.5;
  decision = controller.update(input);
  checks.require(decision.reason == "sample_stale" &&
                     near(decision.sample_age_s, 0.5),
                 "stale causal AGL pair must be rejected with age diagnostic");

  input = paired_sample(2.0, 100.0, 110.0);
  input.map_height_valid = false;
  decision = controller.update(input);
  checks.require(decision.reason == "map_height_invalid",
                 "invalid visual map height must be rejected");

  decision = controller.update(paired_sample(3.0, 0.0, 110.0));
  checks.require(decision.reason == "map_height_non_positive",
                 "non-positive visual map height must be rejected");

  input = paired_sample(4.0, 100.0, 110.0);
  input.agl_valid = false;
  decision = controller.update(input);
  checks.require(decision.reason == "agl_invalid",
                 "invalid AGL must be rejected");

  decision = controller.update(paired_sample(5.0, 100.0, 110.0));
  checks.require(decision.window_started,
                 "first valid pair after rejects must enter the window");

  decision = controller.update(paired_sample(5.0, 100.0, 110.0));
  checks.require(decision.reason == "causal_timestamp_non_monotonic",
                 "duplicate causal time must be rejected");
  decision = controller.update(paired_sample(4.5, 100.0, 110.0));
  checks.require(decision.reason == "causal_timestamp_non_monotonic",
                 "backward causal time must be rejected");

  input = paired_sample(5.1, 100.0, 110.0);
  input.sample_timestamp_s = 5.0;
  decision = controller.update(input);
  checks.require(decision.reason == "sample_timestamp_non_monotonic",
                 "duplicate paired sample time must not be counted twice");

  input = paired_sample(6.0, 100.0, 110.0);
  input.sample_timestamp_s = 5.0;
  decision = controller.update(input);
  checks.require(decision.reason == "sample_stale",
                 "old duplicate sample is rejected as stale before reuse");

  input = paired_sample(7.0, 100.0, 110.0);
  input.sample_timestamp_s = std::numeric_limits<double>::quiet_NaN();
  decision = controller.update(input);
  checks.require(decision.reason == "sample_timestamp_invalid",
                 "non-finite paired sample time must be rejected");

  input = paired_sample(8.0, 100.0, 110.0);
  input.sample_timestamp_s = 7.9;
  decision = controller.update(input);
  checks.require(decision.reason == "window_gap_too_large" ||
                     decision.reason == "window_coverage_insufficient",
                 "a fresh newer paired sample must remain causally admissible");
}

} // namespace

int main() {
  try {
    Checks checks;
    test_window_start_and_stable_release(checks);
    test_sampling_rate_invariance(checks);
    test_gap_and_stability_rejection(checks);
    test_success_clears_window_and_does_not_repeat(checks);
    test_failed_feedback_preserves_policy_state(checks);
    test_incremental_and_cumulative_bounds(checks);
    test_invalid_and_noncausal_inputs(checks);
    std::cout << "AGL scene-scale controller: " << checks.passed
              << " contract checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "AGL scene-scale controller test failed: " << error.what()
              << '\n';
    return 1;
  }
}
