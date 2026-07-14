# Online multisensor alignment initializer — implementation report

Status: `P4 IMPLEMENTED_AND_FLIGHT_SHORT_REPLAY_VALIDATED`

## What now runs

The formal path consumes a continuous FC navigation stream, the board IMU, and
the normal OpenVINS monocular tracker. It buffers only data already received,
solves an immutable 8 s window, checks every released state, and releases one
`AlignmentResult` exactly once. It never selects a nearest FC row and never
falls back to the ordinary OpenVINS initializer.

The common Ceres problem contains:

- per-keyframe attitude, position, velocity, gyro bias, and accelerometer bias;
- one startup-only FC-to-board attitude misalignment;
- 9-D FC attitude/position/velocity observation factors;
- 15-D `CpiV1` IMU preintegration factors;
- monocular landmarks triangulated from causal multi-frame feature tracks;
- pixel reprojection factors with the June-12 Camera–IMU calibration locked;
- bias and startup-misalignment priors.

The FC/body-to-board matrix in the stream is only an axis convention. The
additional startup misalignment is estimated for that run and is not published
as a permanent P3 calibration. The bounded FC/board clock difference is also a
startup nuisance and must have uncertainty below 0.05 s; it is not written as a
permanent time calibration.

No approximately 7-degree correction, 4.089-degree correction, time-varying
flex correction, GPS input, or post-run position/heading/SE(3) alignment is
used.

## Release checks

`READY` requires finite positive-definite 15-by-15 state covariance, a separate
startup-misalignment covariance, nonzero FC/IMU/visual residual and Jacobian
contributions, and observable non-prior-only attitude, position, velocity,
gyro bias, accelerometer bias, and startup misalignment. Weak monocular depth
directions may be removed as nuisance directions; an unobservable released
state may not be removed or marked observable.

The state machine is:

`WAIT_INPUTS -> COLLECTING -> ALIGNING -> VALIDATING -> READY`, with rejected
windows entering `FAILED_WAIT_RETRY` and returning to collection when new data
arrives.

## Verification

- ROS-free targets build successfully.
- `test_online_alignment_initializer` passes directly and through CTest.
- Synthetic coverage includes FC, IMU, and visual influence; no-visual and
  insufficient-motion rejection; frame mismatch; high-rate interval
  rejection; positive covariance; atomic release; and failed-window retry.
- fly1 released at camera time 934.646347284 s and produced 105.34 s of
  post-initialization trajectory.
- fly3 released at camera time 755.495250225 s and the extended replay produced
  64.49 s of post-initialization trajectory.
- Both positive flights record nonzero FC, IMU, and monocular reprojection
  residuals and Jacobians, all six state groups observable, no future data, and
  one READY transition.
- The fly1 visual-disabled control ended without READY and without a trajectory.

Registered output root:

`C:\Users\baloney\Desktop\实验目录\P4_online_joint_alignment_20260712`
