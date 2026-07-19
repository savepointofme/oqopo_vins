# Flex-aware FC relative-yaw factor (experimental, default off)

Status: design contract frozen before implementation. This is not a P4/P5 pass.

## Purpose and boundaries

The prior shadow observers and post-hoc XY rotations do not update the estimator and cannot establish a navigation improvement. The new path is an actual EKF measurement update. It uses only FC relative rotation between two processed camera timestamps and never uses FC absolute yaw, GPS position/velocity, or truth online.

The formal VIO state remains the D455 IMU/camera assembly state. A separate scalar state models the change in body-to-D455 mount yaw. Camera-IMU extrinsics and gyro bias are not repurposed as flex states.

## Frozen frames and error convention

- `R_GtoI = state->_imu->Rot()` maps global coordinates into the D455 IMU frame.
- `R_ItoG = R_GtoI.transpose()`.
- `R_BtoG` maps aircraft-body coordinates into global coordinates and is read only once at factor initialization to define the nominal mount.
- `M0 = R_ItoG(init).transpose() * R_BtoG(init)` maps body coordinates into the D455 IMU frame.
- JPL attitude error is injected on the left: `R_GtoI+ = Exp(-dtheta) R_GtoI`.
- FC input after initialization is only `DeltaR_B = R_BtoG(k)^T R_BtoG(k-1)`. A fixed global left rotation of the complete FC attitude stream therefore cancels exactly.

At a processed camera pair `(a,c)`, the state contains pose clones `R_a,R_c` (`R_GtoI`) and scalar flex clones `f_a,f_c`. The time-varying mount is

`M(f) = M0 Exp(e_z f)`.

The observed D455 relative rotation and the FC-predicted D455 relative rotation are

`DeltaR_I_obs = R_c R_a^T`,

`DeltaR_I_pred = M(f_c) DeltaR_B M(f_a)^T`.

The scalar prediction is

`h(x) = e_z^T M(f_c)^T Log(DeltaR_I_obs DeltaR_I_pred^T)`.

The measurement is zero, so the EKF residual is `r = -h(x)`. The Jacobian is the central-difference derivative of this exact SO(3) model with respect to both clone attitude errors and both flex clones. It is evaluated using the repository's left-error injection convention.

## State, propagation, and update

- Current `flex_yaw` is a one-dimensional additive EKF state, initialized to exactly zero when the nominal mount is frozen.
- It is cloned at every processed camera timestamp, alongside the IMU pose, so a relative factor contains both endpoint flex levels instead of treating the previous level as deterministic.
- It follows a zero-mean random walk. Process noise is injected only for elapsed initialized time; missing FC does not change the mean.
- The factor uses a masked-gain Joseph update. It corrects current/window pose orientations and current/window flex levels, while position, velocity, gyro/accelerometer bias, landmarks, and calibration means are held. The full covariance is still updated with the matched masked gain, and the corrected current attitude affects future IMU propagation. This prevents a scalar yaw residual from creating an instantaneous XY or bias jump through incidental cross-covariance.
- A scalar NIS gate rejects invalid/time-broken/outlier measurements. Rejection leaves the mean unchanged.
- Old flex clones are marginalized with the pose clone at the same timestamp.

## First-version common parameters

Parameters are physical/statistical defaults and are identical for all flights:

- flex initial sigma: `0.10 deg`;
- flex random-walk sigma: `0.02 deg/sqrt(s)`;
- relative-yaw measurement sigma: `0.35 deg` per processed camera interval;
- NIS limit: `9.0` (one dimension);
- maximum FC endpoint interpolation bracket: `0.45 s`.

They are not selected from fly3 or from GPS XY error.

## Required gates

Before four-flight evaluation, tests must prove:

1. non-commuting three-axis frame order and left-error sign;
2. fixed global FC yaw invariance;
3. quaternion sign invariance at the CSV/interpolation boundary;
4. rigid mount gives zero residual;
5. a known flex change is represented by `f_c-f_a`, rather than a constant mount angle;
6. the EKF update reduces residual and covariance without loss of symmetry/PSD;
7. disabled mode and FC-missing mode are byte-identical to the frozen baseline trajectory/bias outputs.

Four-flight acceptance uses the existing stride-12 run contract and `analysis/full_flight_error_analysis.py`: GPS-time sampling, start-only ENU alignment, common per-flight windows, and no truth-dependent online thresholds. The candidate must be compared with both the frozen baseline and existing `hard_gyro_yaw`. A result that only improves a post-processed trajectory, regresses a flight materially, or merely duplicates `hard_gyro_yaw` is rejected.

## Experimental result

Status: **EXPERIMENTAL_FACTOR_FAILED — default off; do not run four-flight integration**.

The implementation passed the SO(3), invariance, Jacobian, Joseph covariance,
and disabled-path tests. After the final timing change, the disabled binary
still reproduced both the frozen trajectory prefix (`E347B729...C488`) and
bias prefix (`924B251A...E86`) byte-for-byte. Three estimator placements were then tested on the same
fly3 stride-12 smoke interval. Moving the factor to the correct causal point
(after propagation/clone and before visual update) removed the known timing
error, but did not pass the XY regression gate:

| fly3 interval 622.2–829.8 s | frozen baseline | pre-visual factor | change |
|---|---:|---:|---:|
| XY RMSE | 73.98 m | 106.03 m | 43.32% worse |
| final XY error | 94.92 m | 107.95 m | 13.73% worse |
| course/yaw RMSE | 1.52 deg | 1.61 deg | 0.09 deg worse |

In the requested 760–800 s window, 100 factors were accepted. Their residuals
sum to `-0.9019 deg`, but the flex state changes only `+0.0010 deg`, while the
current D455 attitude receives `-0.1924 deg` of accumulated yaw correction.
Position, velocity, and biases have zero instantaneous mean update, so the XY
regression is caused by the altered D455 attitude entering later propagation
and visual updates, not by a hidden direct position correction.

This exposes a structural observability problem, not a remaining timing or
gain bug: one FC-minus-D455 relative-yaw residual can be explained either by a
D455 yaw correction or by a change in body–D455 flex. Routing it into D455 yaw
violates the sensor/body separation and degrades XY here. Constraining D455 yaw
and routing it entirely into flex is physically consistent, but then the VIO
sensor trajectory is intentionally unchanged and cannot improve XY without an
additional translational/body-motion measurement. GPS-derived tuning cannot
resolve that ambiguity because GPS is forbidden online.

Authoritative artifacts:

- run: `C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/flex_aware_fc_yaw_factor_ekf_20260719/fly3_smoke_enabled_previsual_masked_joseph_to830_v4`;
- standard evaluation: `C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/flex_aware_fc_yaw_factor_ekf_20260719/evaluation/flex_aware_fc_yaw_factor_ekf_20260719_previsual_smoke`;
- reusable factor summary: `analysis/summarize_flex_relative_yaw_factor.py`.
