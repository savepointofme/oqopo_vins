# P4 causal online joint initializer — implementation audit

Audit status: `P4_IMPLEMENTATION_AUDIT_REWORK_COMPLETED`

Scope: `OnlineAlignmentInitializer` as it existed before the factor-graph
rework requested on 2026-07-12. This audit is about executable code, not the
intended design document.

Post-rework result: the deficiencies below were the trigger for the rewrite
and are retained as the before-state audit. The executable implementation now
contains FC pose/velocity/attitude factors, `CpiV1` IMU preintegration factors,
monocular multi-frame landmark triangulation and reprojection factors, joint
tangent covariance, per-state observability, retryable state transitions, and
single-use release. The ten synthetic cases and the registered fly1/fly3 short
replays passed; the no-visual fly1 control did not release.

## Executive finding

The existing implementation is causal and fail-closed, but it is not a joint
visual/inertial/FC optimizer. It has no common optimization state, no visual
reprojection factor, no IMU preintegration factor, no FC pose/velocity factor
inside a common normal equation, and no recovered joint covariance. Therefore
it cannot enter real-flight acceptance under the current P4 contract.

## Sensor contribution audit

| Sensor | Current executable action | Residual in common solve? | Jacobian in common solve? | Finding |
|---|---|---:|---:|---|
| monocular vision | RANSAC essential matrix; compare only relative rotation angle with integrated gyro; count tracks | no | no | hard gate/diagnostic only |
| board IMU | interpolate gyro/acceleration; gyro-rate Wahba; trapezoidal angle integral; FC-velocity-derived acceleration comparison | no CPI/preintegration state factor | no | not a joint inertial solve |
| FC attitude | finite-difference FC attitude into angular rate; Wahba against board gyro; exact-time attitude copied into final state | no pose factor | no | initial seed plus independent fit |
| FC position | exact-time interpolation copied to final position with declared lever arm | no | no | direct assignment |
| FC velocity | exact-time interpolation copied to final velocity; finite difference used as acceleration surrogate | no | no | direct assignment and noisy derivative baseline |

The `visual_imu_rotation_residual` function reduces each relative rotation to a
single angle magnitude. It discards the rotation axis and never differentiates
the result with respect to pose, gyro bias, accelerometer bias, mounting
misalignment, or landmark state. This does not satisfy “visual influence” in a
joint estimator.

## Per-state audit

| Estimated output | Existing prior/seed | Existing residual | Existing Jacobian | Existing covariance | Existing observability gate |
|---|---|---|---|---|---|
| attitude `q_GtoI` | interpolated FC attitude times independent Wahba mounting result | FC/board angular-rate fit; visual angle gate outside solve | none | scalar heuristic from rate RMS/excitation | angular-rate excitation and visual/IMU angle threshold |
| position `p_IinG` | interpolated FC position plus declared lever arm | none | none | heuristic floor and FC interpolation/time uncertainty | only FC count/gap/finite gates |
| velocity `v_IinG` | interpolated FC velocity plus lever-arm angular term | none | none | heuristic from accelerometer residual | only FC count/gap/finite gates |
| gyro bias `b_g` | zero implicit prior | component median of `gyro_board - R_FI gyro_FC` | none | robust scalar dispersion copied to three axes | norm and scalar sigma threshold |
| accelerometer bias `b_a` | zero implicit prior | component median of board acceleration minus FC velocity-difference model | none | robust scalar dispersion copied to three axes | norm and scalar sigma threshold |
| startup mounting misalignment | declared axis map plus centered Wahba result | angular-rate vector fit | Wahba closed-form only; absent from a joint Hessian | not represented in released covariance | two-axis ratio and maximum residual angle |
| FC/board time nuisance | bounded grid search | angular-rate RMS | none | near-optimum score width heuristic | bounded search and score availability |

The released 15-by-15 covariance is block diagonal and constructed from
independent heuristic scales. Cross-correlation between attitude, velocity,
position, biases, and mounting misalignment is absent. A positive-definite
check on this constructed matrix is not an observability proof.

## State-machine audit

The prior implementation has only a private `released_` boolean and string
rejection reason. It does not implement the required states:

- `WAIT_INPUTS`
- `COLLECTING`
- `ALIGNING`
- `VALIDATING`
- `READY`
- `FAILED_WAIT_RETRY`

It also cannot report rejected window intervals or demonstrate a failed window
transitioning back to collection and succeeding on a later causal window.

## Reference implementation findings

The requested moving-base workspace provides two useful patterns:

1. The VIF/OBA path uses window-integrated IMU quantities and explicitly warns
   against relying on instantaneous GNSS/FC velocity differentiation.
2. The 21-state KF-GINS lineage and staged moving-base code separate state
   masks, measurement masks, residual/condition gates, accepted bias axes, and
   covariance. A state that is weakly observable is marked as prior-only or not
   released instead of being presented as estimated.

For OpenVINS, the closest reusable executable machinery is already in-tree:

- `CpiV1` produces preintegrated rotation, velocity, position, bias Jacobians,
  and 15-by-15 measurement covariance;
- `Factor_ImuCPIv1` supplies the 15-dimensional inertial residual and analytic
  Jacobians between keyframe states;
- `Factor_ImageReprojCalib` supplies pixel reprojection residuals and analytic
  pose/landmark Jacobians while Camera–IMU calibration can be held constant;
- Ceres covariance recovery in `DynamicInitializer` provides a model for
  extracting the joint covariance after convergence.

## Required correction before synthetic or flight acceptance

The replacement must construct one causal factor graph containing:

- keyframe pose and velocity states over the window;
- shared gyro and accelerometer biases;
- one startup-only FC-to-board misalignment state;
- locked Camera–IMU extrinsics/intrinsics/time offset;
- IMU CPI factors between keyframes;
- monocular temporal reprojection factors with causally triangulated landmarks;
- FC attitude, position, and velocity observation factors;
- explicit priors with physical meaning.

It must compute factor-family residual and Jacobian contributions, recover the
joint covariance, evaluate per-state Hessian/covariance observability, execute
the required retryable state machine, and only then atomically release the last
window state to OpenVINS.
