# I2 Implementation Audit

Date: 2026-07-12  
Status: source-level audit of the implementation used by the P4 clean runs  
Implementation name: **I2 exact-time plus robust short-window FC attitude initialization**

## Definitions

| Term | Actual action |
|---|---|
| I0 nearest-FC-row initialization | Copy the closest FC state row to `t_seed`, subject to a maximum absolute time difference. |
| I1 exact-time interpolated FC initialization | Quaternion shortest-path SLERP plus linear position, velocity, and bias interpolation between rows bracketing `t_seed`. |
| I2 exact-time plus robust short-window FC attitude initialization | Start from the I1 state, replace only its attitude with the intercept of a Huber-weighted linear regression in an SO(3) tangent space over a symmetric time window. |
| I3 quality-gated initialization | Apply quality gates, data-derived covariance floors, robust bias medians, and an explicit fail-closed or I1 fallback policy. |

Primary sources are `ov_msckf/src/core/FCInitLoader.h`,
`ov_msckf/src/run_serial_msckf_ros_free.cpp`,
`tools/fc_to_init_csv.py`, and
`baseline/clean_p4/scripts/run_p4_init_batch.sh`.

## Answers to the implementation audit

1. **Input fields.** The direct CSV fields are
   `t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz`.
   I2's attitude regression consumes timestamp and quaternion. The I1 base state
   consumes all fields. Window velocity, position residual, gyro-bias, and
   accelerometer-bias samples are used to compute diagnostics; I2 does not use
   those window diagnostics to replace position, velocity, bias, or covariance.
2. **Window duration.** The code clamps the CLI value to `[3,8] s`; it does not
   select a duration from data. The registered batch passes `5.0 s`. Actual
   support is 4.8 s because FC rows arrive at about 5 Hz.
3. **Window placement.** It is symmetric and centered on `t_seed`, selecting
   timestamps in `[t_seed-duration/2,t_seed+duration/2]`.
4. **Future data.** Yes. fly1 used `927.553238869--932.353238821 s` around
   `t_seed=930.046994686 s`, including 2.306 s of future data. fly3 used
   `615.599999905--620.399999857 s` around `t_seed=618.034715652 s`, including
   2.365 s of future data.
   Independently, the I1 exact-time base state also uses the first FC row after
   `t_seed` for bracket interpolation. Thus both the exact-time bracket and the
   symmetric attitude window have future-data availability times; the symmetric
   I2 window dominates the delay in these runs.
5. **Formal initialization delay.** No. The state is still timestamped and
   injected at `t_seed`; replay does not wait until the window end. Current I2
   is therefore a **non-causal offline diagnostic**, not an accepted online
   initialization implementation.
6. **Attitude calculation.** It is not quaternion averaging, a Karcher mean,
   a component median, Cauchy weighting, or a trimmed mean. It is a 12-iteration
   iteratively reweighted least-squares linear regression in one SO(3) tangent
   space using a Huber loss. The fitted tangent intercept at `t_seed` is mapped
   back with the SO(3) exponential and left-multiplied onto the I1 reference
   rotation.
7. **Residual.** With the I1 exact-time rotation `R_ref`, each sample is
   `y_i=Log(R_i R_ref^T)`. The fitted model is
   `y_hat_i=a+omega*(t_i-t_seed)`. The residual is the three-vector
   `r_i=y_i-y_hat_i`; its Euclidean norm drives Huber weighting and reported
   RMS/p95/max values.
8. **Outlier threshold.** Registered Huber delta is `1.5 deg`. Samples are not
   hard-deleted: `w=1` at or below 1.5 deg and `w=delta/||r||` above it. The
   registered `3.0 deg` residual-p95 threshold is an I3 gate only; I2 records
   p95 but labels it diagnostic and does not reject on it.
9. **Other weighting.** Initial weights are equal. Subsequent weights depend
   only on residual norm. There is no angular-speed, sample-quality, satellite,
   or sampling-interval weight. Actual timestamp offsets enter the linear design
   matrix, and fitted angular rate is an output, not a weight.
10. **Position and velocity.** Both remain I1 exact-time linear interpolation
    between the two rows bracketing `t_seed`. They are not window-averaged.
    Window velocity spread and constant-velocity position residual spread are
    diagnostic only for I2.
11. **Bias.** I2 retains I1 exact-time linearly interpolated bias. In these
    generated inputs every bias field is zero. Only I3 requests component-wise
    window medians, subject to its quality contract.
12. **Initial covariance.** I2 uses the configured fixed standard deviations:
    attitude `3 deg`, velocity `5 m/s`, position `5 m`, gyro bias
    `0.003 rad/s`, and accelerometer bias `1 m/s^2`. Although window spreads are
    computed, the runner applies data-derived maxima only when the applied level
    is I3.
13. **Fallback.** The registered I2 runs use `fail_closed`; I2 has no fallback.
    I3 alone can be configured to fall back explicitly to I1, recording the
    applied level and reason.
14. **Fail-closed conditions.** Exact-time selection rejects duplicate or
    non-monotonic timestamps, missing brackets/extrapolation, non-positive gaps,
    and bracket gaps above `0.35 s`. Robust fitting rejects fewer than 12 window
    rows or singular time support. For I2, maximum window source gap, residual
    p95, speed, bias norms, and data-derived covariance are only diagnostics;
    those become fail/fallback gates in I3. This distinction is material.

## Compensation and frame status

| Item | Used? | Meaning |
|---|---|---|
| Fixed June-12 Camera--IMU transform `T_C_I` | Yes | Fixed estimator camera-to-board-IMU calibration. It is never made turn-dependent. |
| Declared FC-to-board axis mapping | Yes | Deterministic FC FRD to board-IMU `x-right, y-forward, z-up` axis convention, plus the declared FC yaw sign convention. This is a frame mapping, not a fitted residual correction. |
| Approximately 7-degree correction | No | Prohibited. |
| 4.089-degree correction | No | Prohibited. |
| Accepted fixed FC-board mounting correction | No | No such accepted calibration exists. |
| Time-varying Velcro/flex correction | No | No time-varying Camera--IMU or FC-board correction is applied. |

The fixed Camera--IMU extrinsic, the declared FC-to-board axis convention, a
fitted fixed residual rotation, and a time-varying flexible-mount model are four
different objects. Only the first two are present.

## Metadata defect and consequence

Existing I2 `nav_frame_metadata.json` records
`whether_future_data_used=false`. That field contradicts the source code and is
invalid for I2. The correct audit state is:

```text
future_fc_attitude_data_used=true
formal_initialization_delayed_to_window_end=false
causality_status=NON_CAUSAL_OFFLINE_DIAGNOSTIC
```

I2 may continue to be compared as an offline initialization diagnostic, but it
cannot become the accepted online P4 path until it is made causal or the formal
initialization timestamp is delayed and evaluated accordingly.
