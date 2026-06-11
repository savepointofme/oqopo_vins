# Flight Error Analysis Skill

## Purpose

Use `analysis/full_flight_error_analysis.py` primarily for complete analysis of
one experiment run. Comparison is an optional second operation over completed
single-run results. Use the same definitions for fly1/fly2/fly3/fly4,
historical packages, and future flights.

## Inputs

Required:

- Camera-time GPS CSV with timestamp plus WGS84 latitude/longitude/altitude or
  ENU E/N/U.
- TUM-style VIO `traj.txt`: `t x y z [qx qy qz qw]`.
- Output directory, flight name, and method name.

Optional:

- `traj.txt.bias` with `t vx vy vz ...`; preferred over differentiating VIO
  position.
- VIO diagnostic and yaw diagnostic CSVs.
- A valid LK-only trajectory CSV.
- LK pair-flow diagnostics. These are not sufficient by themselves unless
  height, camera geometry, and a validated integration schema are available.
- `t0` and `t1`.

## Standard command

```bash
python analysis/full_flight_error_analysis.py \
  --gps <gps_csv> \
  --vio-traj <traj.txt> \
  --vio-bias <traj.txt.bias> \
  --vio-diag <diag.csv> \
  --vio-yaw-diag <yaw_diag.csv> \
  --lk-traj <lk_visual_traj.csv> \
  --lk-flow-csv <lk_flow_yaw_diag.csv> \
  --out-dir <output_dir> \
  --t0 <start_time> \
  --t1 <end_time> \
  --flight-name <flight_name> \
  --method-name <method_name>
```

For comparisons:

```bash
python analysis/full_flight_error_analysis.py --manifest <manifest.json>
```

For a versioned experiment run:

```bash
python analysis/full_flight_error_analysis.py --run-spec <run.json>
```

The run spec must preserve the experiment ID, configuration, status, time
window, source package, archive member, GPS source, and output directory.
Archive members may be read directly with
`<package.zip>::<member/path/traj.txt>`.

## Experiment directory convention

All experiment and analysis outputs belong under:

```text
C:/Users/baloney/Desktop/实验目录/
```

Use this hierarchy:

```text
实验目录/
  <original_experiment_folder_name>/
    <YYYYMMDD_flight_mode_key-config_status>/
      plots/
      tables/
      data/
      reports/
      metadata/
```

Example:

```text
实验目录/gpsz_oc2_1e4_fly3/
  20260606_fly3_oc2_gpsz_fi1e4_成功/
```

The run folder must encode date, flight, applied mode/configuration, and
success/failure status. Do not use generic folders such as `analysis`,
`latest`, or `new`.

## Coordinate and sampling rules

- Use ENU: East, North, Up.
- Treat GPS as the evaluation reference, not perfect ground truth.
- Use original GPS timestamps as the master statistics grid.
- For this ros-free runner, map each GPS update to the first emitted VIO state
  at or after that update. The emitted state is post-update; do not interpolate
  backward across the update discontinuity.
- Do not upsample GPS to camera rate for final statistics.
- Preserve and use flight-controller `Ve/Vn/Vu` as the primary ENU velocity
  reference when generating camera-time GPS files. Position differencing is
  only a compatibility fallback for historical four-column files, and must
  never cross a missing-GPS gap.
- Prefer state velocity from `traj.txt.bias` for VIO velocity.
- Apply `t0`/`t1` to plots, aligned rows, segments, and every reported metric.
  Set `t1` to the last valid experimental time before SLAM feature clearing,
  reinitialization, sustained image-read failure, or another documented
  invalid tail. Record the reason in `crop_reason`.

## Start alignment

Main statistics always use start alignment:

1. Search the first 60 s for the earliest stable 5 s course window with GPS
   speed above 5 m/s, then estimate one horizontal rotation from that window.
2. Rotate VIO position and velocity by that angle.
3. Translate VIO position so the first common GPS/VIO sample agrees in ENU.
4. Preserve scale.

Full-trajectory SE(2)/SE(3)/Sim(3) best fit may be produced only as a clearly
labeled diagnostic. It is never the source of main drift statistics.

## Error definitions

```text
err_E = vio_E - gps_E
err_N = vio_N - gps_N
err_U = vio_U - gps_U
err_XY = hypot(err_E, err_N)
err_3D = norm([err_E, err_N, err_U])

err_vE = vio_vE - gps_vE
err_vN = vio_vN - gps_vN
err_vU = vio_vU - gps_vU
err_speed_xy = vio_speed_xy - gps_speed_xy
err_vXY_vec = norm([err_vE, err_vN])

e_along = [cos(course), sin(course)]
e_cross = [-sin(course), cos(course)]
err_along = dot([err_E, err_N], e_along)
err_cross = dot([err_E, err_N], e_cross)
```

Apply the same heading-frame decomposition to velocity. Report signed mean,
MAE, RMSE, median absolute error, p95 absolute error, max absolute error, final
signed error, and sample count.

## Mileage and drift

Use cumulative horizontal GPS distance. Report:

```text
100 * final_XY_error / GPS_horizontal_distance
100 * XY_RMSE / GPS_horizontal_distance
100 * final_along_error / GPS_horizontal_distance
100 * final_cross_error / GPS_horizontal_distance
```

Segment percentages use segment GPS distance; global percentages use full-run
GPS distance.

## Segment decomposition

- Detect straight candidates from GPS speed, course rate, local heading
  variation, and local line-fit residual.
- Default minimum geometric straight-leg length: 2000 m.
- Split accepted long legs every 500 m by cumulative distance.
- Keep the final short remainder of a geometric straight leg as straight.
- Assign every other sample to turn, transition, short, or unknown.
- Never discard samples between straight legs.

## LK-only handling

- Evaluate a supplied LK-only trajectory with the same GPS grid, alignment, and
  metrics.
- Reconstruct LK dead reckoning only when pair motion, camera geometry,
  altitude/height, timing, and scale conventions are sufficient.
- Label reconstructed output `LK_ONLY_DIAGNOSTIC`.
- Otherwise state exactly what is missing and do not create fake LK metrics.

## Output structure

Single-run output:

```text
plots/
  *.png
  *.svg
  interactive_flight_analysis.html
tables/
  global_summary.csv
  metric_statistics.csv
  segment_index.csv
  segment_error_summary.csv
  straight_leg_summary.csv
  turn_transition_summary.csv
  height_layer_summary.csv
  gps_sampling_quality.csv
  reference_metric_check.csv
data/
  gps_time_aligned_samples.csv
reports/
  FULL_FLIGHT_ERROR_ANALYSIS.md
metadata/
  analysis_provenance.json
  global_summary.json
  RUN_STATUS.txt
```

Manifest comparisons additionally produce:

```text
condition_comparison_summary.csv
flight_condition_comparison_summary.csv
all_segment_error_summary.csv
all_straight_leg_summary.csv
all_turn_transition_summary.csv
THREE_CONDITION_OC_COMPARISON_ANALYSIS.md
```

## Standard figures

1. `trajectory_xy_gps_vio_lk.png`
2. `trajectory_xy_error_over_distance.png`
3. `enu_error_time.png`
4. `along_cross_vertical_error_distance.png`
5. `velocity_error_time.png`
6. `speed_error_distance.png`
7. `course_yaw_error_time.png`
8. `segment_xy_drift_bar.png`
9. `segment_velocity_error_bar.png`
10. `straight_leg_error_profiles.png`
11. `height_layer_error_summary.png`

Mandatory single-run views also include:

- `position_enu_gps_vio`
- `velocity_enu_gps_vio`
- component position and velocity error plots
- XY position and velocity error plots
- XY trajectory
- along/cross/vertical error plots

Write every static plot as both PNG and SVG. Also write
`interactive_flight_analysis.html` so users can zoom, box-select, inspect
values, and toggle traces locally.

Condition comparisons add trajectory overlay, XY-error, course-error, and
segment-drift figures.

## Report structure

Single-run reports cover inputs/window, alignment, GPS sampling, distance and
duration, global statistics, axis-wise position and velocity error,
along/cross/vertical error, drift percentages, segmentation, straight legs,
turns/transitions, figures, and warnings.

Comparison reports cover package/conditions, completeness, global and segment
comparisons, straight and turn behavior, along/cross behavior, velocity,
course, verdict, limitations, and next use.

## Anti-patterns

- Do not create one-off analysis scripts unless the reusable tool cannot
  support the request.
- Do not silently change alignment method.
- Do not use full-trajectory best-fit alignment for main drift statistics.
- Do not mix smoothed GPS course into gyro/course truth checks unless
  explicitly labeled.
- Do not drop short straight remainders into turn segments.
- Do not invent an LK-only trajectory if inputs are insufficient.
- Do not change metric definitions between flights or reports.
- Do not average failed or truncated runs into a condition score without
  retaining explicit per-flight status and explaining the aggregation.
- Do not detach analysis from its experiment ID, source package/member,
  configuration, GPS source, and evaluation window.
- Do not write experiment or analysis outputs outside
  `C:/Users/baloney/Desktop/实验目录/`.
- Do not mix plots, tables, large aligned data, reports, and metadata in one
  directory.
