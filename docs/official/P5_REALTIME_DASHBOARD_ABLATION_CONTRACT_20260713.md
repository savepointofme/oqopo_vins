# P5 Realtime and Dashboard Ablation Contract

## Objective

Replace the invalid "fewer processed frames than fixed stride12" release gate with a direct realtime-throughput gate. Measure whether the P5 state-machine candidate can process each registered replay window at least as fast as sensor time advances, and isolate the cost of the ROS-free OpenCV dashboard.

## Frozen estimator contract

- Use the same clean P4 online FC-navigation + board-IMU + monocular-visual initializer.
- Use the same June-12 camera intrinsics, lens distortion, and fixed Camera-IMU transform.
- Keep GPS horizontal position and course evaluation-only.
- Keep repair, restart, anchor reset, adaptive calibration, manual 7-degree/4.089-degree compensation, and post position/heading/SE(3) alignment disabled.
- Do not change P5 policy thresholds, states, tracking cadence, or backend cadence during this ablation.

## Conditions

For fly1 and fly3, replay the same registered minimal active windows with the P5 state-machine candidate:

1. `dashboard_visible`: OpenCV dashboard enabled, `--viz-fast --dash-every 5`.
2. `dashboard_headless_render`: identical dashboard data preparation and canvas rendering, but no window via `--no-display`.
3. `dashboard_disabled`: dashboard data pushes, display-only alignment, image snapshot preparation, canvas rendering, and window display disabled with `--no-dashboard`.

Only one estimator process runs at a time. A realtime benchmark represents the deployed single estimator; concurrent fly replays would measure resource contention rather than the candidate's ability to keep up with one sensor stream. The original multi-process active runs are retained as historical functional evidence but are not authoritative realtime evidence.

## Metrics

- `sensor_span_s`: last raw camera timestamp minus first raw camera timestamp in the registered window.
- `wall_runtime_s`: process wall time measured around the runner.
- `realtime_factor = wall_runtime_s / sensor_span_s`.
- `realtime_margin = sensor_span_s / wall_runtime_s`.
- GNU time user CPU, system CPU, CPU utilization, and maximum resident set size.
- Existing estimator-health, lifecycle, anti-chatter, and navigation metrics remain reported separately.

The wall measurement includes dataset reads, estimator execution, diagnostic CSV output, and the selected dashboard condition. This is conservative for online deployment. Initialization data loading before the measured process is not subtracted after the fact.

## Decision rule

- Per-flight realtime pass: `realtime_factor <= 1.0`.
- Operational-margin diagnostic: `realtime_factor <= 0.8`.
- P5 realtime gate passes only when fly1 and fly3 both pass independently in the deployment condition.
- Frame-count ratios and wall-time ratios versus fixed stride12 remain descriptive; they are not release gates.
- Dashboard causal contribution is reported from the same-flight visible/headless/disabled differences. Visible versus headless isolates window presentation and `waitKey`; headless versus disabled isolates dashboard data preparation and canvas rendering. If disabling the dashboard does not bring both flights to `realtime_factor <= 1.0`, the dashboard is not the sole realtime blocker.

This host-side result does not by itself certify a different embedded target. The report must identify the benchmark host and executable hash.
