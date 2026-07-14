# P4/P5 Mainline Status (2026-07-13)

## Frozen decisions

- P4: `P4_FLY1_FULL_AIRBORNE_SCREENING_PASS_FOUR_FLIGHT_VALIDATION_REQUIRED`.
- P5: `P5_ACTIVE_SCREENING_REJECTED_ANTI_CHATTER_INSUFFICIENT`.
- P3 remains `INCONCLUSIVE_DIAGNOSTIC` and does not block this P4/P5 work.
- The clean baseline remains fixed stride 12. Active adaptive stride, repair, restart, and anchor reset remain disabled.

## P4 evidence

The current-source fly1 comparison uses the same June-12 ground camera intrinsics and lens distortion parameters, fixed Camera--IMU `T_C_I`, backend, GPS-altitude-only aiding, and fixed stride 12. The primary evaluation is absolute navigation in `G_nav` with no post-run position, heading, SE(2), SE(3), or Sim(3) alignment.

On the common airborne interval 942.593--1712.593 s, causal online FC navigation + board IMU + monocular feature initialization reduces:

- final XY error from 447.857 m to 272.600 m;
- XY RMSE from 650.444 m to 432.497 m;
- maximum XY error from 1305.261 m to 985.141 m;
- horizontal velocity-vector RMSE from 12.617 m/s to 8.883 m/s;
- VIO-yaw-versus-GPS-course RMSE from 18.458 degrees to 10.499 degrees.

Scalar horizontal speed RMSE increases from 7.733 m/s to 7.965 m/s, and the low-height descent bands are not uniformly better. Therefore this is a fly1 screening pass, not four-flight P4 acceptance.

The 10.807 s `NAVIGATION_READY` practical-navigation result initialized OpenVINS. The 41.22 s `FULL_ALIGNMENT_READY` result was recorded later and did not reset the running state, FEJ, clones, or covariance. The final JSON overwrote the earlier result at the same path, so its `state` is not the canonical state that started the trajectory. The authoritative master records that first-result payload as unavailable instead of substituting the unapplied final state.

## P5 evidence

The actual-time controller uses measured tracker time and received-camera time. Four-flight shadow replay completed with fixed estimator stride 12. The fly1/fly3 minimal active windows completed with finite covariance, zero negative covariance-diagonal rows, and healthy active MSCKF feature counts.

These P5 runtime replays use the current binary but retain legacy single-row nearest-FC initialization to isolate controller behavior. They do not validate the combined clean P4 online initializer plus P5 controller. This lineage limitation does not weaken the anti-chatter rejection; it would have prevented a final clean-baseline acceptance even if the controller gate had passed.

Active switching remained excessive:

| Flight | Switches per minute | Registered limit |
| --- | ---: | ---: |
| fly1 | 10.92 | 6.0 |
| fly3 | 10.43 | 6.0 |

The active controller is therefore not released. The next P5 task is an explicit safety-regime state machine with per-regime entry/exit hysteresis, followed only by the same minimal active windows.

## Prohibited interpretations

- Do not claim four-flight P4 acceptance.
- Do not claim active adaptive stride readiness or baseline release.
- Do not relabel the historical 32 fixed-stride runs as a clean current-source navigation baseline.
- Do not apply or claim acceptance of approximately 7-degree, 4.089-degree, permanent FC-to-board residual, or time-varying flex correction.
