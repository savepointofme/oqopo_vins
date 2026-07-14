# P4/P5 Mainline Experiment Contract (2026-07-12)

## Purpose

This contract freezes the current-source fly1 full-airborne P4 comparison before execution. P5 work starts only after both P4 runs and their canonical evaluation are complete.

## P4 compared conditions

| Item | Legacy condition | Online joint condition | Held fixed |
|---|---|---|---|
| Initialization | `fc_full_state` with `I0` nearest-FC-row seed | `online_multisensor_alignment` with causal FC navigation, board IMU, and monocular feature-window supervision; `practical_navigation_start` release | Current source tree and freshly rebuilt runner |
| Start | Camera time 930.0 s | Camera time 930.0 s | Same fly1 dataset |
| Camera | June-12 ground camera intrinsics and lens distortion; fixed June-12 Camera--IMU transform | Same | `baseline/clean_p4/config/estimator_config.yaml`, online camera calibration disabled |
| Image processing | Fixed stride 12 | Fixed stride 12 | Adaptive stride, repair, restart, and anchor reset disabled |
| Backend | Clean P4 MSCKF configuration | Same | Same binary and runtime flags |
| Navigation aiding | FC state is used only to initialize; GPS altitude is the only continuous GPS update | Same | GPS horizontal position/course are reference-only |
| FC input | Registered FC navigation series sufficient for the legacy seed | Registered causal FC navigation stream | Both are derived from the same fly1 FC navigation source; no single-row formal input |
| Mount correction | None | None | No approximately 7-degree, 4.089-degree, accepted fixed FC-to-board, or time-varying flex correction |
| Formal output | Direct `G_nav` trajectory | Direct `G_nav` trajectory | No post-run position, heading, SE(2), SE(3), or Sim(3) alignment |

The FC input files differ only because the legacy loader consumes a seed-time series while the online initializer requires the causal stream and its quality declarations. This difference is part of the initialization path and must be disclosed in the result.

## Evaluation contract

- Primary: `absolute_navigation_no_post_alignment` sampled at GPS reference times.
- Flight-accuracy window: first formal navigation output through the sample immediately before automatically detected touchdown or sustained ground stationary state.
- The window includes altitude bands above 50 m, 30--50 m, 20--30 m, and below 20 m.
- Post-touchdown samples are reported only as shutdown/stationary diagnostics.
- Each run's available airborne window and their common airborne interval are both reported.
- Secondary relative-drift diagnostics may use start-heading alignment and must never replace the primary result.
- Turn comparisons use registered geometry-based turn segments; no manual error-based segment cutting is allowed.

## P4 screening questions

The result must report initialization delay, 30 s and 60 s errors, first and later turns, height bands, position/velocity/attitude/covariance health, state discontinuities, backend anomalies, and any fallback. P4 is reported independently before P5.

## P5 boundaries

P5 reuses the existing 32 fixed-stride runs. It must use actual camera timestamps/gaps and available observed visual motion/health signals. A recommended stride must not be treated as the stride that produced a measurement. Active adaptive stride remains disabled until shadow evidence establishes a bounded safe region, stable switching behavior, and a minimally informative active-screening plan.
