# P4 causal visual–inertial–FC joint alignment — technical specification

Status: implementation contract

## Completion boundary

P4 may finish only as one of:

- `P4 IMPLEMENTED_AND_FLIGHT_SHORT_REPLAY_VALIDATED`
- `P4 IMPLEMENTED_BUT_FAILED_ACCEPTANCE`

Synthetic tests, documentation, or a compiled interface alone cannot produce
the first status.

## State machine

```text
WAIT_INPUTS
  -> COLLECTING
  -> ALIGNING
  -> VALIDATING
  -> READY
            \-> FAILED_WAIT_RETRY -> COLLECTING
```

- `WAIT_INPUTS`: no complete FC/board-IMU/monocular-image support exists.
- `COLLECTING`: valid causal samples are buffered; no solver sees future data.
- `ALIGNING`: a snapshot of `[t_window_start,t_init]` is immutable during the
  solve.
- `VALIDATING`: residual, Jacobian, covariance and per-state observability are
  evaluated.
- `READY`: one immutable `AlignmentResult` can be consumed exactly once.
- `FAILED_WAIT_RETRY`: the rejected interval and reason are recorded; arrival
  of a newer valid sample starts collection for a later causal window.

## Factor-graph state

For each selected monocular keyframe `k`:

`x_k = [q_GtoI_k, p_I_k_in_G, v_I_k_in_G, b_g_k, b_a_k]`.

Shared startup-only state:

`delta_theta_FI_startup`, the small residual rotation applied on top of the
declared FC/body-to-board axis convention. It is valid only for this
initialization and is not a P3 permanent mounting calibration.

Camera–IMU extrinsics, camera intrinsics/distortion, and Camera–IMU time offset
are constant parameter blocks loaded from the locked June-12 calibration.
FC-to-board time offset is a bounded startup nuisance selected only from causal
samples. It is not written as a permanent calibration.

## Factors

### IMU continuous-preintegration factor

Every adjacent keyframe pair uses `CpiV1` over board-IMU samples at locked
Camera–IMU shifted timestamps. `Factor_ImuCPIv1` contributes the 15 residuals

`[rotation, gyro-bias random walk, velocity, accel-bias random walk, position]`

and analytic Jacobians with respect to both endpoint poses, velocities and
biases. The factor is weighted by the CPI measurement covariance generated from
the configured IMU noise densities/random walks.

### Visual reprojection factor

Landmarks are initialized from two or more causal views of the same feature ID
using the FC/IMU pose seed, a minimum 0.20 m camera baseline, a minimum 0.5°
ray parallax, cheirality, and normal-matrix rank checks. Every accepted temporal
observation uses `Factor_ImageReprojCalib` with
the Camera–IMU and camera model parameter blocks held constant. Pixel residuals
have nonzero Jacobians with respect to observing pose and landmark. Cauchy loss
is used; tracks without multi-keyframe support are rejected.

### FC observation factor

At each keyframe, causal FC interpolation supplies declared `G_nav` position,
velocity and `q_GtoF`. A 9-dimensional factor contributes:

- SO(3) attitude residual between `q_GtoI_k` and startup-misalignment-composed
  FC attitude;
- position residual including the declared FC-to-board lever arm;
- velocity residual including the lever-arm angular-rate term.

Its Jacobians act on keyframe attitude/position/velocity and startup
misalignment. FC bias columns are never used.

### Priors

- broad zero-mean first-keyframe gyro/accelerometer bias priors;
- broad startup misalignment prior centered on the declared axis mapping;
- no approximately 7-degree, 4.089-degree, or other fitted manual-angle prior;
- absolute pose/velocity gauge is supplied by FC factors, not post alignment.

## Covariance and per-state observability

Ceres recovers tangent-space covariance from the converged common problem. The
released 15-by-15 covariance is the full last-keyframe marginal, including
cross terms. Startup misalignment covariance is reported separately.

For each released group (`attitude`, `position`, `velocity`, `gyro_bias`,
`accelerometer_bias`, `startup_misalignment`) report:

- prior/IMU/visual/FC residual count;
- factor-family Jacobian Frobenius contribution;
- covariance standard deviation per axis;
- marginal information eigenvalues and condition number;
- `observable`, `prior_only`, or `unobservable` status and reason.

`READY` requires every requested state to be `observable`; a prior-only state
is not presented as estimated.

## Result contract

`AlignmentResult` contains:

- `t_init`, `pose_G_nav`, `velocity_G_nav`, `b_g`, `b_a`;
- startup attitude misalignment and covariance;
- full 15-by-15 OpenVINS covariance;
- state-machine transition history;
- per-state observability;
- factor-family residual and Jacobian statistics;
- initialization window/duration;
- rejected intervals;
- FC/IMU/monocular-image and locked-calibration provenance;
- `future_data_used=false` and `post_alignment_used=false`.

OpenVINS release is atomic and single-use. No nearest-row or ordinary
OpenVINS-initializer fallback exists.

## Test and flight gates

Before flight, the ten requested synthetic cases must pass and demonstrate
nonzero FC, IMU and visual Jacobian contributions. Then exactly three short
jobs are executed in parallel:

1. fly1 joint initialization;
2. fly3 joint initialization;
3. fly1 with visual factors disabled as a negative control.

Positive jobs stop after initialization plus 60 seconds, while covering the
first turn and its post-stable segment when those boundaries require a slightly
longer registered short window. The negative control must never reach `READY`.
No whole-flight evaluation or post alignment is permitted.
