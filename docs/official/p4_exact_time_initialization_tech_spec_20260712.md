# P1 Runtime Contract and P4 Exact-Time Initialization Tech Spec

Date: 2026-07-12  
Status: implementation contract; evaluation correction added 2026-07-12  
Scope: ROS-free OpenVINS runner, FC initialization source, canonical init record,
absolute-navigation evaluation, and the minimum I0--I3 experiment chain.

## Decision and dependency correction

- `results_20260612` readonly audit v6 is frozen. No artifact under that directory
  may be regenerated, and the approximately 5 Hz FC attitude-difference flex
  classifier is not extended.
- P3 is `INCONCLUSIVE_DIAGNOSTIC`. Neither approximately 7 degrees, 4.089
  degrees, nor another residual FC-to-board rotation is an accepted calibration.
- P3 does not block P4-I0, I1, I2, or I3. P4-I0--I3 use the declared mechanical
  FC-FRD to board-IMU axis map only and apply no residual mounting correction.
- Only P4-I4 depends on an accepted P3 calibration. I4 is out of scope here.
- `T_C_I` remains one fixed Camera--IMU transform. Turn-correlated FC/IMU
  residuals never make `T_C_I` time-varying.

## Initialization modes

Every new run passes `--initialization-mode` explicitly:

1. `fc_full_state`
   - FC supplies attitude, navigation position, velocity, and optionally biases.
   - The source declares `position_frame=global_gnav`.
   - The EKF may retain an internal local `W0`, but the canonical output is
     `traj_nav.txt` expressed directly in `G_nav` by the single causal transform
     fixed at `t_seed`.
   - `canonical_init_state.json` contains the exact-time FC state at `t_seed`.
2. `fc_attitude_only`
   - FC supplies attitude only. Position is local zero; velocity defaults to zero
     unless a future separately registered contract enables it.
   - Canonical output is explicitly local, not absolute navigation.
3. `vio_only`
   - No FC initialization source is read. The existing VIO initializer supplies
     the local state.
   - Canonical output is explicitly local.

Mode/source mismatch fails closed. `fc_full_state` and `fc_attitude_only` require
an FC source; `vio_only` rejects one.

## FC source contract

The CSV has monotonically increasing, unique camera-time rows:

```text
t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz
```

Required declarations:

```text
# schema_version=fc-init-series-v2
# position_frame=global_gnav|local_w0_seed
# attitude_frame=Gnav_to_board_imu_jpl
# fc_board_rotation=declared_axis_map_only_no_residual_correction
# camera_imu_extrinsic=fixed
```

For `global_gnav`, `p` uses the same WGS84-to-ENU origin and equations as the
registered GPS reference. The converter emits all FC rows needed for the
registered short window, not an interpolated single row.

## P4 levels

### I0 — nearest row

Legacy control. Select the minimum absolute timestamp difference. Preserve the
selected source row and signed `source_dt`. The registered maximum absolute
time difference remains fail-closed unless the run is explicitly diagnostic.

### I1 — exact-time interpolation

- Find strict bracketing rows `t_before <= t_seed <= t_after`.
- Reject duplicate/non-monotonic timestamps, extrapolation, or a bracket gap
  above `--init-max-bracket-gap`.
- Quaternion: shortest-path SLERP.
- Position, velocity, and biases: linear interpolation.
- Persist source row indices, before/after timestamps, gap, alpha, and values.

### I2 — exact-time plus robust 3--8 s SO(3) window

- Position and velocity remain I1 exact-time interpolation.
- Use a configurable 3--8 s window centered on `t_seed`.
- Fit `R(t) = R_seed Exp(omega * (t-t_seed))` on SO(3) with iterative Huber
  weighting. The intercept is the robust `R_seed`; no constant mounting-angle
  correction is estimated or applied.
- Persist sample count, actual window duration, largest source gap, angular
  residual RMS/p95/max, robust weight fraction, fitted angular rate, and
  attitude covariance estimate.

Implementation audit correction: the current implementation centers the
window on `t_seed`. With the registered `--init-window-s 5.0`, it selects FC
rows in `[t_seed-2.5 s, t_seed+2.5 s]`. It therefore uses future FC attitude
samples, does not delay the formal initialization timestamp to the end of the
window, and is non-causal. It is an offline diagnostic candidate, not an
accepted online initialization implementation. Existing metadata that says
`whether_future_data_used=false` for I2 is incorrect and is superseded by the
I2 implementation audit.

### I3 — quality/covariance/bias/fallback gate

- Apply pre-registered gates to I2 sample count, duration, maximum source gap,
  residual p95, finite state, quaternion norm, speed, satellite count when
  available, and covariance.
- Initial EKF covariance uses the larger of configured floors and data-derived
  standard deviations.
- Bias uses robust window medians only when finite and within registered bounds.
  Otherwise use the configured prior and record `bias_fallback_reason`.
- Default fallback is `fail_closed`. Optional `i1` fallback is diagnostic and
  must be recorded in the canonical init record; it cannot silently pass I3.

I4 (accepted P3 FC-board correction) is not implemented.

## Canonical init record

`canonical_init_state.json` is written at `t_seed` before the first trajectory
row. Minimum fields:

- schema/mode/level/status/fallback;
- `t_seed` and source FC path/hash;
- exact state `q_Gnav_to_I`, `p_Gnav_I`, `v_Gnav_I`, `bg`, `ba`;
- 15x15 diagonal or full covariance with units;
- bracket before/after timestamps, row indices, gap, alpha, and source rows;
- SO(3) window metrics and quality decisions;
- fixed Camera--IMU calibration identity;
- `fc_board_residual_correction_applied=false`;
- `post_alignment_applied=false`.

The first canonical `traj_nav` state must agree with the propagated
`canonical_init_state` under the documented seed-to-first-output interval.

## Frames and lever arms

- FC attitude is a reference in the declared FC/board convention, never truth.
- GPS is a navigation reference, never truth.
- `p_I_GPS` is the GNSS-antenna position expressed in board IMU axes. For an
  antenna position `p_Gnav_GPS`, IMU position is
  `p_Gnav_I = p_Gnav_GPS - R_Gnav_I p_I_GPS`.
- FC navigation position is not assumed to be at the GNSS antenna unless its
  source contract states so. The source metadata records the position point.
- Camera--IMU `T_C_I` remains fixed and is not part of the FC lever-arm model.

## Evaluation contracts

Primary `absolute_navigation_no_post_alignment`:

- statistics grid: reference/GPS timestamps;
- compare canonical `traj_nav` directly against FC/GPS reference;
- no post position, yaw, SE(2), SE(3), or Sim(3) transform;
- compare FC attitude as `reference_attitude`, with declared frame/lever-arm
  metadata;
- report `t_seed`, +10 s, +30 s, +60 s, each turn entry/peak/exit/post-stable,
  and the airborne full-flight window.

### Airborne and post-touchdown windows

Whole-dataset overlap is not the airborne flight-accuracy window. The primary
window is `airborne_full`: from the first formal output after initialization to
the detected touchdown boundary, exclusive. It retains all altitude-above-ground
bands `>50 m`, `30--50 m`, `20--30 m`, and `<20 m`. The remaining samples are
`post_touchdown` and are used only for estimator shutdown/zero-velocity
diagnostics.

Touchdown detection version `openvins-touchdown-v1` is deterministic:

1. Prefer a flight-controller landed flag when present and sustained for 2 s.
2. Otherwise estimate ground height from the terminal stationary episode:
   horizontal reference speed `<=1.0 m/s`, absolute vertical reference speed
   `<=0.3 m/s`, cumulative horizontal position change over 5 s `<=5 m`, and a
   minimum sustained duration of 10 s.
3. Search backward for the start of the final touchdown episode: altitude above
   estimated ground `<=2.0 m`, absolute vertical speed `<=1.5 m/s`, horizontal
   speed `<=40 m/s`, cumulative horizontal change over the next 5 s `<=200 m`,
   altitude stays `<=3.0 m` for 10 s, and terminal stationary onset follows
   within 60 s.
4. If the touchdown episode is not observable, use the sustained stationary
   onset. If neither condition is observable, airborne evaluation fails closed;
   it must not silently use dataset end.

All thresholds, detected times, source fields, ground-height estimate, and
fallback path are written to `metadata/flight_window.json`. A 30 m height crop
is forbidden for the primary endpoint; it is only a height-band boundary.

The previous whole-dataset products retain their numeric values but carry
`evaluation_validity=invalid_evaluation_window` and status
`PREVIOUS_EVALUATION_INVALID_POST_TOUCHDOWN_INCLUDED`. Until fly1--fly4 have
same-configuration I0/I1/I2 airborne evidence, the only P4 status is
`P4_AIRBORNE_REEVALUATION_REQUIRED`.

Secondary `relative_drift` permits only one explicitly recorded origin
relative operation. Historical start-heading products remain legacy diagnostic.

The first-turn statement is descriptive: measure whether error changes from
approximately 10 m toward approximately 50 m; do not label the turn as the
start of divergence without a separately registered causal test.

## Clean configuration

- June-12 ground Kalibr K/D/T_CI and Camera--IMU time offset, all locked;
- online Camera intrinsics/distortion/extrinsic/time calibration off;
- no approximately 7-degree or 4.089-degree correction;
- fixed stride 12;
- GPS XY/course and FC attitude evaluation-only;
- no post alignment.

## Verification and experiment gates

Implementation verification:

- synthetic SLERP and exact linear interpolation;
- sign-continuous quaternion brackets;
- duplicate, non-monotonic, missing bracket, extrapolation, and excessive gap
  fail closed;
- robust SO(3) regression with injected outliers;
- I3 quality pass/fail and explicit fallback;
- initialization-mode/source mismatch;
- canonical init JSON and `traj_nav` seed conformance;
- fixed `T_C_I` and no residual FC-board correction in manifest/command.

Experiment sequence:

1. Batch A: fly1 I0/I1/I2 short window in parallel.
2. Batch B: fly3 I0/I1/I2 short window in parallel.
3. Select Ibest from absolute-navigation and quality evidence.
4. Run I0/Ibest full flight for fly1/fly3.
5. With Ibest frozen, isolate C0/C1/C2 K/D conditions. T_C_I, Camera--IMU
   time offset, initialization, backend, and evaluation remain identical.

Long-running batches use one blocking batch wrapper and return only after all
children exit. No 30-second polling loop is permitted.

## P2 parallel offline track

P2 reads the existing 32-run sweep and actual camera timestamp gaps only. It
generates provisional `geometry_only`, `geometry_plus_rotation`, and
`geometry_plus_feature_health` shadow policies. It performs no estimator replay,
creates no active release, and does not repeat the 4x8 sweep.

## Required outputs

- `P4_I0_I2_COMPARISON.csv`
- `ABSOLUTE_NAVIGATION_REPORT.md`
- `TURN_ERROR_BEFORE_AFTER_INIT_FIX.csv`
- `CLEAN_BASELINE_SHORTLIST.json`
- `BASELINE_CALIBRATION_ISOLATION_REPORT.md`
- `P2_PROVISIONAL_OVERLAP_TIMESERIES.parquet`
- `P2_SHADOW_POLICY_COMPARISON.csv`
- updated authoritative `MASTER_FLIGHT_DATA`
- updated P0--P6 status
