# OpenVINS Frame Convention Draft (Superseded Audit History)

Date: 2026-07-08

Scope: first-round P0 audit for initialization origin, logging chain, master data columns, and stride experiment indexing. This document records current evidence only; it does not approve a fixed 7 degree compensation.

Normative definitions now live in `FRAME_CONTRACT.md`. This file is retained as
the evidence snapshot that motivated that contract. Correction: the first-round
audit missed `opencv_yaml_parse.h:661-665`, where the fallback transform is in
fact inverted when the opposite key is used.

## 结论

当前代码中最清楚、可落地的在线状态原点是 IMU。`State::_imu` 保存 `q_GtoI, p_IinG, v_IinG, bg, ba`，但这里的 `G` 在代码里是 OpenVINS world/local frame；在本轮审计中统一记为 `W0`，不能直接等同 GPS/导航全局 `G`。

## 统一符号

| Symbol | Meaning | Current source |
| --- | --- | --- |
| `G` | GPS / navigation ENU reference. East, North, Up. | `analysis/FLIGHT_ERROR_ANALYSIS_SKILL.md`; `analysis/full_flight_error_analysis.py` |
| `W0` | OpenVINS initialized local world. Code often names it `G`; keep separate from GPS `G` until a mapping is explicit. | `ov_core/src/types/IMU.h:40`; `ov_msckf/src/state/State.h:146` |
| `F` | Flight-controller body frame. Not yet proven equal to OpenVINS IMU/body frame. | FC init CSV input path in baseline manifest |
| `B` | Vehicle standard body frame. Reserved for calibrated aircraft body convention. | Not yet implemented as a separate online state |
| `I` | OpenVINS IMU frame and filter body state. | `State::_imu` |
| `C` | Camera frame. D455 baseline uses cam0 intrinsics and camera/IMU extrinsic. | `baseline/latest/config/kalibr_imucam_chain.yaml` |

Notation:

| Notation | Meaning |
| --- | --- |
| `R_AtoB` | Passive coordinate transform that maps vector coordinates in frame `A` into frame `B`. |
| `p_AinB` | Origin of frame `A` expressed in frame `B`. |
| `T_AtoB` | Homogeneous transform using `R_AtoB` and the corresponding translated coordinate convention. |
| `q_AtoB` | Quaternion for `R_AtoB`; OpenVINS stores JPL order `[qx, qy, qz, qw]`. |

## 当前状态定义

The IMU state class documents the nominal order as `[q_GtoI, p_IinG, v_IinG, b_g, b_a]` and error order as `[dtheta, dp, dv, dbg, dba]` in `ov_core/src/types/IMU.h:40-41`. For this audit, read that as:

```text
q_W0toI
p_IinW0
v_IinW0
bg_I
ba_I
```

`State` confirms the active state is IMU-centered and the state timestamp is in camera clock frame:

```text
State::_timestamp: last update time in camera clock frame
State::_imu: active IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
State::_calib_dt_CAMtoIMU: t_imu = t_cam + t_off
State::_calib_IMUtoCAM: calibration poses for each camera (R_ItoC, p_IinC)
```

Evidence: `ov_msckf/src/state/State.h:140-159`.

## Camera/IMU Extrinsic Convention

The runtime state stores camera extrinsic as `(R_ItoC, p_IinC)`. `VioManagerOptions` reads a matrix named `T_imu_cam` into local variable `T_CtoI`, then stores:

```text
R_ItoC = R_CtoI^T
p_IinC = -R_CtoI^T * p_CinI
```

Evidence: `ov_msckf/src/core/VioManagerOptions.h:352-359`.

When projecting state positions to camera centers, existing update code uses the equivalent relation:

```text
p_CinW0 = p_IinW0 - R_W0toC^T * p_IinC
```

`baseline/latest/config/kalibr_imucam_chain.yaml` stores `T_cam_imu`. The parser
falls back from the requested `T_imu_cam` key and does invert the loaded matrix at
`ov_core/src/utils/opencv_yaml_parse.h:661-665`; the earlier review stopped at
lines 621-629 and incorrectly left this as a suspected missing inversion. A
numeric regression test remains required, but the production fallback direction
is defined.

## Initialization Origin

For FC-assisted initialization, `initialize_with_fc_state()` directly sets:

```text
q_W0toI <- fc.q_GtoI
p_IinW0 <- fc.p_IinG
v_IinW0 <- fc.v_IinG
bg <- fc.bg
ba <- fc.ba
```

and sets `state->_timestamp` to the first camera timestamp used for EKF seeding. Evidence: `ov_msckf/src/core/VioManagerHelper.cpp:91-117`.

Therefore, the initial output is not guaranteed to be zero. It is zero only if the supplied FC init row has `p_IinG = 0` and no propagation/logging delay changes the state before first output. The current baseline passes an FC init CSV and a nonzero `--start-time`; the runner trims samples before `start_time` and uses the first retained camera timestamp (`ov_msckf/src/run_serial_msckf_ros_free.cpp:1238-1265`).

## Output Layers

| Layer | Definition | Current evidence |
| --- | --- | --- |
| `vio_raw_*` | Estimator state written directly in `W0`, IMU origin. | `traj.txt` header says unaligned VIO frame at `ov_msckf/src/run_serial_msckf_ros_free.cpp:1341`; state write at `2328-2338`. |
| `vio_mapped_*` | Program-internal formal mapping into another frame. | No separate persistent mapped trajectory found in current ros-free output. Dashboard can display aligned points, but `traj.txt` remains raw. |
| `vio_analysis_aligned_*` | Offline evaluation-aligned values. | `analysis/full_flight_error_analysis.py:819-833` applies start-heading alignment and writes `gps_time_aligned_samples.csv`. |

## First-Round Answers

1. Initialization origin: in the online estimator, the active origin is the IMU origin `I` expressed in local `W0`; FC-assisted init may place it at a nonzero FC/GPS-derived coordinate.
2. First output nonzero: expected when `fc.p_IinG` is nonzero, when first logged camera time is after the requested init row, or when comparing camera/vehicle position against IMU state without applying the lever arm.
3. Constant position offset candidates: camera/IMU lever arm, FC/GPS local origin, dashboard yaw+translation alignment, offline start translation, and possible `T_cam_imu`/`T_imu_cam` convention confusion.
4. Constant angle offset candidates: `q_GtoI` vs `q_ItoG`, start-heading alignment, FC body convention, ENU/NED or FLU/FRD conversion, camera/IMU extrinsic direction, and FC/camera time offset.
5. Current analysis does execute a documented second alignment for metrics. It must be stored as `vio_analysis_aligned_*`, never overwrite `vio_raw_*`.
