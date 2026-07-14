# Online Multisensor Alignment Initializer — Technical Contract

Status: implementation contract, 2026-07-12

## Objective

Replace one-row flight-controller state injection with a finite-window,
fully causal initializer.  The initializer consumes flight-controller (FC)
navigation, board IMU, and calibrated stereo feature streams.  It releases one
atomic OpenVINS initial state only after coverage, synchronization,
observability, residual, bias, covariance, and visual-health gates pass.

The historical symmetric-window I2 implementation is an offline diagnostic. It
is not called by this initializer and is not a release fallback.

## Physical and calibration boundary

| Relationship | Treatment |
|---|---|
| June-12 Camera--IMU transform `T_C_I` | locked; used to convert stereo rays to the board-IMU frame |
| declared FC/body to board axis convention | required fixed input; used before estimating residual mounting rotation |
| nominal FC-to-board residual mounting rotation | estimated once inside the causal initialization window |
| FC-to-board lever arm | declared fixed input; not silently estimated |
| Camera--IMU time offset | locked estimator calibration; not estimated here |
| FC-to-board clock offset | estimated within a bounded causal search and reported with uncertainty |
| approximately 7-degree or 4.089-degree correction | prohibited |
| turn-dependent Velcro/flex correction | prohibited |

The nominal mounting result is valid only for initialization. It does not turn
`T_C_I` into a time-varying calibration and is not applied as a continuous FC
measurement update.

## Inputs and frames

`FCNavigationSample` contains timestamp, position and velocity in a declared
navigation frame, passive JPL quaternion `q_GtoF`, navigation/body frame names,
and independent position/velocity/attitude/status validity flags.

`BoardImuSample` contains timestamp, angular velocity, specific force, declared
board-IMU frame, and validity/saturation flags.

`StereoAlignmentFrame` contains left/right timestamps and per-feature ID, raw
pixels, undistorted normalized observations, triangulated depth, ray residual,
tracking age, and validity. It is constructed from the normal OpenVINS tracker
database after the same locked camera model and `T_C_I` have been applied.

Any frame-name change, non-monotonic timestamp, duplicate timestamp, invalid
quality flag, saturated IMU interval, or non-finite value is rejected or
retained as an explicit gate failure. GPS XY/course is never read by this
initializer.

For ROS-free replay, the FC stream adapter accepts the existing 17 numeric
state columns only as a transport layout; in online mode the quaternion column
means `q_Gnav_to_FC_body`, and FC-provided bias columns are ignored. Four
mandatory per-row columns follow: `position_valid, velocity_valid,
attitude_valid, status_valid`. The file must declare `navigation_frame`,
`fc_body_frame`, `board_imu_frame`, `attitude_representation`,
`position_frame`, `R_FtoI_declared_row_major`, and `p_IinF_m`. Missing or
ambiguous declarations fail before replay. Rows are fed in timestamp order;
the full source container is never passed to the initializer.

## Causal time contract

At wall/stream time `t_now`, all accepted samples satisfy `timestamp <= t_now`.
The solve window is `[t_init - window_duration, t_init]`, where `t_init` is the
latest stereo timestamp for which past-arrived FC and IMU coverage exists.
FC interpolation may use a source row after `t_init` only if that row has
already arrived by `t_now`; release therefore occurs at `t_now`, never before
the data existed. No sample newer than the solve invocation is accessible.

Default duration is 5 s and is constrained to 3--8 s. Buffers retain only the
configured window plus synchronization margin. The released state timestamp is
`t_init`; normal IMU propagation moves it forward after release.

## State and error-state ordering

The nominal OpenVINS state is

`x = [q_GtoI, p_IinG, v_IinG, b_g, b_a]`.

The 15-dimensional covariance/error order follows the existing OpenVINS
error-state injection contract:

`delta_x = [delta_theta, delta_p, delta_v, delta_b_g, delta_b_a]`.

Additional initializer parameters are `R_FtoI_nominal` and scalar
`dt_FC_to_board`. They are reported with covariance/uncertainty but are not
added as online EKF states and are not used for continuous FC fusion.

## Estimation

1. Derive FC angular-rate samples from adjacent FC attitudes.
2. Search bounded FC-to-board time offsets. For every candidate, pair only
   causally available FC rates with interpolated board gyro.
3. Solve a centered Wahba problem for rotation mapping FC rates to board-IMU
   rates, then robustly estimate gyro bias from the residual median.
4. Split the total rotation into the declared axis mapping and a small nominal
   mounting residual. Reject an excessive or unobservable residual.
5. Compare stereo relative rotation with board-IMU integrated rotation. Stereo
   therefore participates in synchronization/ranking and is not merely a file
   presence check.
6. At `t_init`, interpolate FC position, velocity, and attitude. Apply the
   accepted nominal mounting rotation and declared lever arm.
7. Estimate accelerometer bias from FC velocity-derived navigation acceleration,
   gravity, interpolated board attitude, and board specific force. Robust
   medians reject isolated samples.
8. Form covariance from robust residual dispersions and interpolation/time
   uncertainty. Apply configured physical floors; never substitute a
   hand-chosen compensation angle.

## Release gates and fail-closed behavior

The default gates require:

- at least 3 s causal span, 12 valid FC states, 100 valid unsaturated IMU
  samples, 6 stereo frames, and 20 valid tracks with at least 12 stereo depths;
- maximum FC, IMU, and stereo gaps below configured bounds;
- sufficient three-axis angular excitation and a conditioned Wahba solve;
- nominal mounting residual no greater than 15 degrees;
- FC/board angular-rate RMS residual no greater than 0.20 rad/s;
- stereo/IMU relative-rotation residual no greater than 5 degrees;
- `|b_g| <= 0.20 rad/s`, `|b_a| <= 2.0 m/s^2`;
- finite positive covariance with attitude, position, velocity, and bias
  standard deviations below configured ceilings.

Failure leaves `VioManager::initialized()==false`. There is no nearest-row,
I1, historical I2, zero-bias, manual-angle, or VIO-only fallback inside this
mode. Diagnostics name every failed gate and the current observability values.

## Atomic release and runtime exclusions

On a passing result, `VioManager` writes nominal state and FEJ values, writes
the full 15-by-15 initial covariance through `StateHelper`, sets the state
timestamp, cleans pre-initialization tracks, invalidates propagation caches,
and changes initialized state once. Partially accepted states are impossible.

The ROS-free online-alignment mode rejects active adaptive stride, adaptive
stride shadow, camera-frame adaptive mode, pose repair, and restart supervisor.
It requires fixed camera stride 12 until this initializer has passed its
acceptance program.

## Acceptance criteria

1. Unit tests prove strict causality, timestamp/frame rejection, bounded buffer
   retention, no release before three-stream coverage, fail-closed behavior,
   time-offset/mount/bias recovery on deterministic synthetic motion, and
   covariance ordering/positivity.
2. A streaming replay test feeds records in timestamp order and proves the
   result is unchanged when future records are present in the source container
   but withheld from `feed_*` calls.
3. Both ROS-free and ROS library targets compile with the new source.
4. Runtime metadata records window endpoints, arrival/release time, source
   counts/gaps, frame declarations, locked calibration identity, estimated
   mounting/time relationship, bias, covariance, gate results, and
   `future_data_used=false`.
5. Flight accuracy, adaptive stride, P2 shadow, and historical I0/I1/I2 rankings
   are explicitly outside this implementation milestone.
