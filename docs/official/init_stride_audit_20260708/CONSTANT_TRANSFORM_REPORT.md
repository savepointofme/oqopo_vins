# Constant Transform Report Draft

Date: 2026-07-08

## 结论

本轮只给出来源假设和已定位代码位置，不能把约 7 度差异认定为安装角。当前最可能被误用为“常值补偿”的环节有三个：相机/IMU外参方向、在线 dashboard 的 XY yaw+translation 显示对齐、离线分析的 start-heading 对齐。

## Confirmed Constant Transforms In Code

| Transform | Where | Applies to estimator state? | Notes |
| --- | --- | --- | --- |
| Camera/IMU fixed extrinsic `(R_ItoC,p_IinC)` | `ov_msckf/src/core/VioManagerOptions.h:352-359` | Yes, through visual measurement model. | Direction must be audited for configs that store `T_cam_imu`. |
| Dashboard `T_GV` yaw+translation | `ov_msckf/src/run_serial_msckf_ros_free.cpp:1593-1607` | No. | Visualization only; `traj.txt` remains raw. |
| Offline start-heading yaw + first-sample translation | `analysis/full_flight_error_analysis.py:819-833` | No. | Official metrics alignment; must be labeled `vio_analysis_aligned_*`. |
| GPS-Z XY diagnostic reference | `ov_msckf/src/run_serial_msckf_ros_free.cpp:2050-2058` | No XY fusion. | XY is diagnostic reference only; altitude path may update Z. |

## Not Yet Proven

- FC frame `F` equals OpenVINS IMU frame `I`.
- FC attitude timestamps are aligned to camera/IMU timestamps well enough for installation-angle estimation.
- The observed 7 degree Euler difference is a pure yaw installation angle.
- The same constant transform holds across straight, turn, climb, and descent windows.

## Next Verification Window

Pick one short window with valid FC, IMU, camera, GPS, `traj.txt`, `traj.txt.bias`, and `diag.csv`. For one timestamp, manually compute:

```text
T_GC = T_GW0 * T_W0I * T_IC
```

and compare it against:

- raw `traj.txt` IMU pose;
- `diag.csv` IMU pose and camera extrinsic fields;
- dashboard displayed pose if alignment is active;
- `analysis/data/gps_time_aligned_samples.csv` aligned pose;
- final plot coordinates.

Only after this manual chain closes should a constant residual transform be fitted over a full flight.

