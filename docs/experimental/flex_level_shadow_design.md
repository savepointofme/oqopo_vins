# Experimental low-frequency flex level observer

Status: experimental shadow design; this document does not claim P4/P5 acceptance.

## Frozen input and state contract

- Baseline input is the desktop-frozen `fly1_oc_stride12` through
  `fly4_oc_stride12` history. The estimator is not replayed to create inputs.
- `R_I_to_G` is the saved OpenVINS/D455 attitude. It remains authoritative and
  is never overwritten.
- FC contributes adjacent relative rotations only. No FC absolute heading,
  GPS position, velocity, or trajectory truth is an observer input.
- The only estimated state is the record-local body-to-D455 mount level
  `R_I_from_B`. Derived aircraft attitude is
  `R_B_to_G = R_I_to_G R_I_from_B`.
- P/V, gyro bias, covariance, and camera--IMU extrinsics have no write path.

For each valid adjacent interval, the observer propagates

`R_I_from_B_obs[k+1] = Delta_R_I[k] R_I_from_B_obs[k] Delta_R_B[k]^T`,

where

`Delta_R_I[k] = R_I_to_G[k+1]^T R_I_to_G[k]` and
`Delta_R_B[k] = R_B_to_G[k+1]^T R_B_to_G[k]`.

The multiplication order is frozen by a non-commuting three-axis synthetic
test. A fixed-mount and a known time-varying-mount transition must both be
recovered to numerical precision.

## Frozen detector

The detector compares the robust SO(3) level of `R_I_from_B_obs` with the
currently adopted level. It does not solve a new hand--eye calibration in each
window.

- candidate evidence: 15 continuous seconds;
- independent confirmation evidence: the next 15 continuous seconds;
- completed windows are evaluated at 1 Hz; this changes trigger latency by at
  most one second and avoids recomputing the same 15-second SO(3) mean for every
  stride-12 image pair;
- candidate and confirmation must each be internally stable and have the same
  final level; confirmation is not required to rotate again in the same
  direction;
- correction cooldown: 30 seconds;
- smooth release: 5 seconds;
- any missing FC/time break resets evidence and holds the adopted flex exactly;
- quality failures cannot create evidence or corrections;
- one common parameter set is used for all flights.

Thresholds are tied to the existing per-record fixed-mount audit uncertainty
(about 0.25--0.34 deg), its observed maximum turn residual (about 3.52 deg),
and the already frozen visual/gyro timing-quality gates. They are not tuned to
the two fly3 event windows.

## Acceptance and fallback

Shadow passes only if both fly3 events each produce one correctly directed
correction, all normal sections across four flights contain at most one false
correction, correction spacing is at least 30 seconds, missing-FC tests produce
no candidate/correction, and frozen VIO files remain byte-identical.

If this observer fails, the only permitted fallback is a fixed-delay,
piecewise-constant SO(3) level smoother with change-point/total-variation
regularization and 30-second minimum dwell. If both fail, the result is
`SHADOW_FAILED` and no rosfree integration is permitted.

## Frozen failed comparison

The previous per-window hand--eye result remains read-only at
`flex_attitude_shadow_v1_true_stride12_20260718_233700`. Its gates and limits
must not be changed. Its failure is: neither fly3 target window produced a
formal correction, while seven normal-section corrections occurred across the
four flights at the frozen 15-second setting.
