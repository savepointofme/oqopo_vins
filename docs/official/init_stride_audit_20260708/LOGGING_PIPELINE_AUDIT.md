# Logging Pipeline Audit

Date: 2026-07-08

## 结论

当前 ros-free 主链路已经能区分原始轨迹与分析对齐轨迹：`traj.txt` 是 unaligned VIO frame，`analysis/full_flight_error_analysis.py` 另行生成 start-heading aligned samples。主要缺口是没有统一主表把 `vio_raw_*`、`vio_mapped_*`、`vio_analysis_aligned_*` 同时保留下来。

## Current Chain

1. Dataset load: runner loads IMU, camera, optional GT and GPS from dataset paths (`ov_msckf/src/run_serial_msckf_ros_free.cpp:1211-1229`).
2. GPS timestamp adjustment: `gps_time_offset` is applied to GPS samples only when nonzero (`1231-1235`). Baseline manifest sets `gps_time_offset: 0`.
3. Runtime crop: `--start-time` removes earlier IMU/camera samples (`1238-1252`).
4. FC init: `--init-from-fc` loads one FC init row near `--start-time` (`1255-1265`).
5. State init: `initialize_with_fc_state()` assigns FC quaternion, position, velocity, and biases directly to the IMU state and sets covariance (`ov_msckf/src/core/VioManagerHelper.cpp:91-108`).
6. Raw output: after each initialized camera frame, runner writes `p_wi` and orientation to `traj.txt`; header explicitly says `VIO (unaligned) frame` (`ov_msckf/src/run_serial_msckf_ros_free.cpp:1341`, `2328-2338`).
7. Bias output: runner writes `vx vy vz bg ba` to `traj.txt.bias` (`1344-1346`, `2334-2341`).
8. Diagnostic output: `diag.csv` records per-frame IMU position/velocity, latest GPS, feature counts, biases, camera intrinsics/extrinsic translation, and distances (`ov_msckf/src/ros_free/DiagLogger.h:31-71`).
9. Dashboard alignment: when enough VIO/GPS or VIO/GT pairs exist after init, dashboard solves one XY yaw+translation alignment and displays aligned VIO (`ov_msckf/src/run_serial_msckf_ros_free.cpp:1593-1607`; `ov_msckf/src/ros_free/VizDashboard.cpp:90-116`).
10. Offline analysis: canonical full-flight analysis samples at GPS update times, takes first post-update VIO state, applies start-heading alignment, and writes `gps_time_aligned_samples.csv` (`analysis/full_flight_error_analysis.py:756-865`, `1500`).

## What Is Raw, Mapped, And Aligned

| Name | Current source | Meaning |
| --- | --- | --- |
| `vio_raw_*` | `traj.txt`, `traj.txt.bias`, `diag.csv vio_*` | IMU state in OpenVINS local `W0`, no GPS alignment. |
| `vio_mapped_*` | Not yet found as a persistent output in ros-free baseline. | Reserved for future formal program mapping, for example `W0 -> G` if added. |
| `vio_analysis_aligned_*` | `analysis/data/gps_time_aligned_samples.csv` columns `vio_E/N/U`, `vio_vE/N/U` | Offline start-heading aligned VIO on GPS time grid. |

## Direct Answers Required By Stage 1

1. Initialization origin: FC-assisted path initializes the IMU state, not the camera state, FC antenna, or GPS home.
2. First logged position can be nonzero because FC init position is copied into `p_IinW0`, and because first logged camera time can be after the FC init row.
3. Constant position offsets can be introduced by camera lever arm, FC/GPS origin definition, dashboard display alignment, offline start translation, or extrinsic direction mismatch.
4. Constant angle offsets can be introduced by quaternion direction, FC frame convention, ENU/NED or FLU/FRD conversion, camera/IMU extrinsic direction, dashboard yaw alignment, or offline start-heading yaw alignment.
5. Current analysis executes a documented second alignment. It is valid for metrics only and must not be fed back as raw estimator output.

## Immediate Data Contract

Every future run-level table must carry:

- `vio_raw_*` from unaligned estimator output.
- `vio_mapped_*` only if a program mapping is explicitly applied and logged.
- `vio_analysis_aligned_*` from evaluation alignment, with `alignment_id`, `alignment_mode`, and transform metadata.
- `timestamp_master`, raw sensor timestamps, aligned timestamps, interpolation or nearest-sample delay, and validity flags.

## Open Audit Items

| ID | Risk | Evidence | Required check |
| --- | --- | --- | --- |
| A1 | Key-direction regression must stay covered, but the suspected missing inversion was disproved. | The fallback calls `Inv_se3` at `ov_core/src/utils/opencv_yaml_parse.h:661-665`; runtime conversion is at `ov_msckf/src/core/VioManagerOptions.h:352-359`. | Add a numeric test proving equivalent `T_cam_imu` and `T_imu_cam` files produce identical runtime input. |
| A2 | Dashboard alignment may be mistaken for estimator correction. | Dashboard uses `R_gv*p+t` for display only. | Keep dashboard-aligned columns separate or do not persist them as estimator output. |
| A3 | Analysis alignment may hide initialization yaw/translation errors. | Start-heading alignment at `analysis/full_flight_error_analysis.py:819-833`. | Store raw and aligned columns side by side; report alignment transform. |
| A4 | FC init timestamp and first camera timestamp differ. | `dt_fc_to_camera` printed in `initialize_with_fc_state()`. | Store FC raw timestamp, aligned timestamp, camera timestamp, and `source_dt` in master table. |
