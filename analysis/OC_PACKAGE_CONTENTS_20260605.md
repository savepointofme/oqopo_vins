# OC Package Contents - 2026-06-05

## Package

Source: `artifacts/packages/three_condition_comparison_package_20260605_1727.zip`

Extracted for analysis at:
`artifacts/extracted/three_condition_comparison_package_20260605_1727/three_condition_comparison`

The package contains three conditions for each of four flights (`fly1` through
`fly4`), for 12 run directories total.

| Condition | Inferred configuration | OC status |
|---|---|---|
| `cond1_nogpsz` | `global_yaw_oc_projection`, alpha 1.0, GPS-Z off | OC |
| `cond2_gpsz_original` | original OpenVINS yaw update, GPS-Z on | non-OC comparison |
| `cond3_oc_gpsz` | `global_yaw_oc_projection`, alpha 1.0, GPS-Z on | OC reference |

## Available artifacts

Every run directory contains `traj.txt`, `traj.txt.bias`, `diag.csv`,
`yaw_diag.csv`, `events.txt`, `log.txt`, and start-yaw/best-yaw evaluation
CSVs. The package does not contain GPS CSV files and does not contain
`yaw_forensics.csv` or a dedicated forensics summary.

The exact per-run paths and inferred GPS sources are recorded in
`analysis/oc_package_index_20260605.csv`.

## GPS sources

The analysis uses existing camera-time GPS products already referenced by
project flight-evaluation tooling:

| Flight | GPS source |
|---|---|
| fly1 | `config/d455_fly1/fc_gps_cam_time.csv` |
| fly2 | `C:/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv` |
| fly3 | `C:/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv` |
| fly4 | `C:/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv` |

These are used as references only. GPS timestamps remain the final statistics
grid; VIO is interpolated to GPS time.

## Completeness and limitations

- Fly2 `cond2_gpsz_original` is a failed partial run. The package summary says
  it crashed at about 1858.27 s with a negative covariance diagonal.
- Fly3 `cond1_nogpsz` ends at 1708.01 s, earlier than the two GPS-Z conditions.
- Other trajectories cover their package flight windows.
- No LK-only trajectory or LK pair diagnostics are included, so LK-only
  metrics must remain unavailable rather than reconstructed from insufficient
  inputs.
- Package-wide condition aggregates must retain per-flight results and failure
  notes; a single pooled number can otherwise hide the Fly2 failure and the
  Fly3 no-GPS-Z truncation.
