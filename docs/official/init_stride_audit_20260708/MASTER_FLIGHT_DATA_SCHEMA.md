# Master Flight Data Schema Draft

Date: 2026-07-08

## 结论

主数据表以相机/视觉输出时刻为主时间轴，保留原始、程序映射、分析对齐三套 VIO 字段。Parquet 是权威输出；CSV/XLSX 只是查看副本。

Authoritative file names:

```text
MASTER_FLIGHT_DATA.parquet
MASTER_FLIGHT_DATA.csv
MASTER_FLIGHT_DATA.xlsx
```

## Required Column Groups

| Group | Required examples |
| --- | --- |
| Index | `run_id`, `flight_id`, `config_id`, `frame_id`, `timestamp_master`, `segment_id`, `init_status` |
| Time | `camera_timestamp_raw`, `fc_timestamp_raw`, `fc_timestamp_aligned`, `gps_timestamp_raw`, `gps_timestamp_aligned`, `vio_state_timestamp`, `*_sample_delay_ms`, `*_valid` |
| Stride | `stride_input`, `stride_visual_update`, `stride_analysis_sample`, `overlap_theory_along`, `overlap_polygon_area`, `overlap_valid` |
| FC | `fc_q_x/y/z/w`, `fc_roll_raw`, `fc_pitch_raw`, `fc_yaw_raw`, `fc_roll_calibrated`, `fc_pitch_calibrated`, `fc_yaw_calibrated`, `fc_velocity_e/n/u`, `fc_angular_rate_x/y/z` |
| VIO raw | `vio_raw_q_x/y/z/w`, `vio_raw_position_x/y/z`, `vio_raw_velocity_x/y/z`, `vio_gyro_bias_x/y/z`, `vio_accel_bias_x/y/z`, `vio_cov_*` |
| VIO mapped | `vio_mapped_q_x/y/z/w`, `vio_mapped_position_x/y/z`, `vio_mapped_velocity_x/y/z`, `mapping_id`, `mapping_applied` |
| VIO analysis aligned | `vio_analysis_aligned_position_e/n/u`, `vio_analysis_aligned_velocity_e/n/u`, `alignment_id`, `alignment_mode`, `alignment_rotation_yaw_deg`, `alignment_translation_e/n/u`, `alignment_applied` |
| GPS/reference | `truth_position_e/n/u`, `truth_velocity_e/n/u`, `truth_course_deg`, `gps_quality`, `gps_velocity_source` |
| Errors | `err_e/n/u`, `err_xy`, `err_3d`, `err_along`, `err_cross`, `err_vertical`, `err_v_e/n/u`, `err_v_along`, `err_v_cross`, `err_v_vertical`, `course_error_deg` |
| Diagnostics | `feature_klt_raw`, `feature_tracked`, `feature_msckf_accepted`, `feature_slam_count`, `mean_track_length`, `visual_update_executed`, `data_missing_flags` |
| Provenance | `repo_commit`, `dirty_worktree`, `runner_path`, `config_path`, `config_hash`, `calibration_id`, `command_path`, `analysis_source` |

## Builder Skeleton

The first skeleton is `analysis/master_flight_data.py`.

Supported first-round commands:

```powershell
python analysis/master_flight_data.py schema --out docs/official/init_stride_audit_20260708/MASTER_FLIGHT_DATA_SCHEMA.csv
python analysis/master_flight_data.py index --result-root result --baseline-manifest baseline/latest/manifests/four_fly_baseline_manifest.json --out docs/official/init_stride_audit_20260708/FLIGHT_STRIDE_EXPERIMENT_INDEX.csv
python analysis/master_flight_data.py build --run-dir result/stride12_robustness_8h/current_stride12/fly3_stride12_20260707_010925 --out-root <out_dir>
```

## Current Limitations

- `vio_mapped_*` remains empty until a formal program mapping output is added.
- FC raw attitude columns require FC/MEMS source logs or FC init CSV columns beyond the current run directory metadata.
- Polygon overlap columns require a later camera-footprint module; first-round index records stride definitions only.
- Parquet/XLSX export depends on installed optional pandas engines. The script always writes CSV when `pandas` is available.

