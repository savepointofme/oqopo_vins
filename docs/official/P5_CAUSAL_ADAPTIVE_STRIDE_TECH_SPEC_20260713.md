# P5 Causal Adaptive Camera Stride Technical Specification

## Objective

Select the largest currently safe camera stride while preserving visual overlap and estimator health. The controller reduces compute in high, steady cruise and returns toward full camera rate during descent, low altitude, turns, excessive pixel motion, stale visual measurements, or feature starvation.

## Evidence boundary

- The existing 32 fixed-stride runs are read-only performance evidence. They are not rerun.
- The fixed sweep supports a conservative active cap of stride 12. Strides 16, 20, and 30 are not released by this controller because their cross-flight behavior is not uniformly safe.
- Existing stride-1 tracker-flow logs show median raw-frame flow near 2.6 px for fly1/fly3 and 1.2 px for fly2. The independent fly3 stride-12 log has median flow 29.9 px, p90 48.8 px, and p95 66.9 px. These observations support a 30 px target, 45 px high gate, and 65 px emergency gate. They do not prove a formal overlap percentage.
- GPS-derived height in the current offline replay is a causal past-only navigation input for the stride supervisor. GPS horizontal position/course and offline trajectory error are forbidden inputs.

## Runtime inputs

At each received camera frame the runner supplies:

- current timestamp and actual received-camera interval;
- latest estimator-relative height source and horizontal speed;
- roll, pitch, and board-IMU angular-rate norm;
- active KLT/MSCKF and SLAM feature counts;
- the newest accepted KLT median pixel displacement, the two tracker timestamps that produced it, and therefore its actual measurement interval.

The controller never divides pixel displacement by a previous recommendation. It converts a fresh measurement to pixel rate using the actual tracker interval, then converts that rate to predicted pixel displacement per currently received raw-camera interval.

## Control law

1. A valid fresh flow sample gives `flow_rate_pxps = median_parallax_px / parallax_measurement_dt_s`.
2. `parallax_per_received_frame_px = filtered_flow_rate_pxps * actual_received_camera_dt_s`.
3. The unconstrained target is `round(30 px / parallax_per_received_frame_px)` and is clamped to stride 1--12. If the current stride predicts 22--38 px, the controller holds that stride instead of chasing small flow fluctuations.
4. Height is a startup/failure fallback only when fresh parallax rate is unavailable.
5. Severe pixel motion, feature starvation, a stale visual measurement, height below 20 m, or vertical speed below -3 m/s forces stride 1 immediately.
6. Height 20--30 m caps stride at 2; height 30--50 m caps stride at 4. Descent below -1.5 m/s caps stride at 4 until vertical speed remains above -0.5 m/s for 2 s. Severe turns cap stride at 6.
7. Released recommendations use the conservative ladder `{1, 2, 4, 6, 8, 12}`. Downward transitions require at most 0.5 s unless an emergency forces immediate full rate. Upward transitions require 5 s and advance by one ladder state.

## State and audit output

The controller persists filtered height, speed, angular rate, measured parallax, parallax rate, last visual-measurement time, hysteresis state, and the current recommendation. The CSV must record actual received-frame interval, actual parallax measurement interval/age, last applied feed gap, actual frames since feed, recommendation, applied stride, and switch reason.

## Active-release gates

Active screening is permitted only after:

- unit tests prove actual-time normalization, stale-measurement emergency fallback, turn cap, feature emergency fallback, and dwell/hysteresis behavior;
- four-flight shadow timelines use real camera timestamps and real tracker flow where available;
- switch frequency and recovery delay are bounded;
- no shadow sample uses future data, flight identity, GPS horizontal error/course, or offline VIO error;
- the limitations of missing or non-comparable actual parallax evidence are explicitly reported.

If these gates are not met, the deliverable is an explicit active-readiness rejection, not an active run.
