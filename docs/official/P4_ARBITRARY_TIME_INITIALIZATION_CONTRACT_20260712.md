# P4 arbitrary-time online initialization contract

Status: implementation contract, thresholds frozen before six-point replay.

## Scope and current status

- P4 joint initializer core: `FLIGHT_SHORT_REPLAY_PASSED_WITH_EXCITATION`.
- P4 arbitrary-time initialization: `NOT_YET_VALIDATED`.
- P2, P5, full-flight evaluation, historical calibration ranking, flex models,
  repair, and restart remain outside this work.

The existing FC observation factors, board-IMU `CpiV1` preintegration,
monocular multi-frame reprojection factors, locked June-12 Camera–IMU
calibration, and causal window remain unchanged in ownership.

## Readiness levels

`NAVIGATION_READY` means attitude, position, velocity, and a usable IMU-bias
seed are safe enough to start OpenVINS. Startup misalignment, FC/board clock
difference, and weak accelerometer-bias axes may remain fixed to a registered
prior or weakly observable. Their covariance is conservative and their status
must not say that the current window estimated them accurately.

`FULL_ALIGNMENT_READY` means every requested state group is supported by the
current FC, IMU, and image data and passes the full covariance and information
gates. Only this level may claim current-data misalignment estimation and fully
converged bias.

After a practical navigation release, later full readiness is recorded only.
It must not reset or jump the running OpenVINS state, FEJ state, clone window,
or covariance.

## Release policies

- `practical_navigation_start` is the explicit default for arbitrary in-air
  startup. It releases once `NAVIGATION_READY` is reached.
- `strict_full_alignment` releases only at `FULL_ALIGNMENT_READY`.
- Neither policy may call the nearest-row or ordinary OpenVINS initializer.
- Vision-disabled navigation is false by default. It may be enabled only by an
  explicit negative-control flag; such a run can never reach full alignment.

## Pre-registered timing and data gates

| Item | Registered value |
|---|---:|
| causal solve window | 8 s |
| navigation target latency | <= 12 s |
| maximum navigation collection time | 20 s |
| post-navigation full-alignment observation | <= 60 s |
| minimum FC rows | 20 |
| minimum board-IMU rows | 600 |
| minimum monocular frames | 12 |
| minimum tracked feature IDs | 80 |
| minimum visual residual blocks | 60 |
| minimum triangulated landmarks | 20 |
| FC maximum gap | 0.45 s, at most one missing nominal 5 Hz row |
| IMU maximum gap | 0.03 s |
| image maximum gap | 0.75 s |
| minimum monocular camera baseline | 0.20 m |
| minimum monocular ray parallax | 0.5 deg |

## Navigation state gates

| State/signal | Navigation threshold |
|---|---:|
| attitude standard deviation | <= 5 deg per axis |
| position standard deviation | <= 5 m per axis |
| velocity standard deviation | <= 2 m/s per axis |
| gyro-bias standard deviation | <= 0.10 rad/s per axis |
| accelerometer-bias navigation bound | <= 2.0 m/s^2 per axis |
| visual reprojection RMSE | <= 2.5 px |
| visual reprojection P95 | <= 4.0 px |
| normalized IMU residual RMS | <= 3.0 |
| normalized FC residual RMS | <= 3.0 |

The attitude, position, and velocity groups must have current-data FC and IMU
support; attitude and position also require current-data visual support unless
the explicit no-vision negative-control option is enabled. Gyro bias must have
IMU support. Accelerometer bias may be `weakly_observable` at navigation
release if its conservative bound passes.

## Registered weak-state priors

| State | Prior/action when current window is weak | Conservative sigma |
|---|---|---:|
| startup FC-to-board residual attitude | fixed to the declared axis mapping | 15 deg/axis |
| FC/board startup clock difference | fixed to stream-declared zero residual | 1.0 s |
| gyro bias | zero prior, current IMU estimate retained only if supported | 0.10 rad/s |
| accelerometer bias | zero prior or weak current estimate | 2.0 m/s^2 |

These are startup priors, not P3 permanent calibration. Approximately 7 deg,
4.089 deg, and time-varying flex corrections remain prohibited.

## Full-alignment gates

Full alignment preserves the existing strict gates: FC/board time-difference
uncertainty <= 0.05 s, sufficient two-axis angular excitation, acceptable
mount seed residual, nonzero FC/IMU/visual Jacobians, all six state groups
observable and not prior-only, positive-definite covariance, bias norm bounds,
and startup misalignment within 15 deg of the declared axis mapping.

## Visual evidence contract

Runtime output must contain tracked feature IDs, multi-frame tracks,
triangulated and rejected landmarks, reprojection RMSE/P95, factor count,
visual Jacobian contribution by released state group, and the visual
Schur-complement information increment. V0 original vision, V1 vision disabled,
and V2 deterministic pixel perturbation use identical FC and IMU data.

The six real start points are selected before execution and each practical run
stops no later than navigation release plus 60 s. Reports must distinguish
current-data estimates, weak estimates, and fixed priors.

## Pre-registered real replay starts

The `start_time` values below are camera-relative dataset seconds and were
frozen before the six replay jobs were launched. Each replay ends 70 s after
its start, covering the 8 s causal collection window and at most 60 s of later
full-alignment observation.

| Run | Segment meaning | start_time (s) | until_time (s) |
|---|---|---:|---:|
| fly1 straight start | stable straight before the first registered turn | 925 | 995 |
| fly1 weak-motion start | weak/pre-turn interval | 930 | 1000 |
| fly1 post-turn start | registered stable interval after the first turn | 994 | 1064 |
| fly3 straight start | stable straight before the first registered turn | 613 | 683 |
| fly3 weak-motion start | weak/pre-turn interval | 682 | 752 |
| fly3 post-turn start | registered stable interval after the first turn | 750 | 820 |
