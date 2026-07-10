# MEMORY.md

## Durable Project Context

- Detected: This repository is an OpenVINS fork focused on UAV VIO/MSCKF research, ROS-free flight replay, GPS-Z aided evaluation, baseline comparison, and flight-data reporting.
- Detected: The active clean baseline lives in `baseline/latest/`; use its scripts before older scattered experiment scripts when validating current fly1-fly4 behavior.
- Detected: The ROS-free runner is `build_ov_msckf/run_serial_msckf_ros_free`; source entry is `ov_msckf/src/run_serial_msckf_ros_free.cpp`.
- Detected: Current baseline defaults are `--yaw-mode baseline`, `--height-mode guarded`, `--camera-frame-stride 12`, and no-thinning control stride `1`.
- Detected: `ov_msckf` now requires C++17; other OpenVINS packages still declare C++14.
- Detected: `mems-ins-workbench/` is a React/TypeScript/Vite MEMS INS flight-data workbench prototype with semantic column parsing and ECharts visualization.

## Repeated Pitfalls To Avoid

- Do not use `--headless` / `--no-display` for flight replay experiments unless display is unavailable or the user explicitly asks. Realtime visualization is expected by default.
- Do not stop adaptive-stride, repair, restart, or baseline validation after code changes. Run the requested fly/globalbaseline experiment and report metrics, logs, and effect.
- Do not fuse GPS horizontal, GPS course, truth, stereo pseudo-reference, or evaluation labels into online estimator logic unless the task explicitly changes estimator inputs.
- Do not tune online thresholds from future evaluation windows or baseline end-window statistics.
- Do not cite stereo pseudo-reference or LK-only diagnostic trajectories as official truth.
- Do not use `--gps-alt-relative` for normal GPS-Z fusion experiments; absolute altitude is the standard unless relative mode is under test.
- Do not declare XFeat/SuperPoint/RAFT/SERAFT/LK-only alternatives better until they are integrated into the estimator path and evaluated under the same criteria.
- Do not rewrite reports in a way that drops required figures, windows, segment definitions, provenance, or precise VINS/SINS/GNSS/MEMS/INS/VIO labels.

## Evaluation Conventions

- Detected: Canonical full-flight analysis is `analysis/full_flight_error_analysis.py` / `analysis/flight_eval_tool.py`.
- Detected: GPS-referenced analysis samples on GPS update time, not a 30 Hz upsampled grid.
- Detected: Reference velocity should prefer flight-controller raw `Ve,Vn,Vu`; VIO velocity should prefer `.bias` `vx,vy,vz`.
- Detected: Main drift metrics use start-heading alignment; best-fit alignment is diagnostic.
- Detected: Segment drift distinguishes local and global drift.
- Detected: Official baseline evaluator `tools/eval/eval_baselines.py` requires explicit `--reference-mode`.
- Detected: For `--reference-mode gps_z`, XY metrics are not official and should be treated as NaN/unused.
- Detected canonical older GPS-Z inputs in `tools/eval/README.md`:
  - fly1: `20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv`
  - fly2: `20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv`
  - fly3: `20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv`
  - fly4: `20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv`
- Detected current `baseline/latest` D455 paths are hardcoded in `baseline/latest/scripts/run_baseline_fly.sh` and mostly point to `C:/Users/baloney/Desktop/...` through WSL paths.

## Operational Notes

- Detected: `run_format.sh` is not a safe narrow formatter; it formats all C/C++ source and currently has an extra `run_serial_msckf_ros_free` invocation appended after formatting.
- Detected: `run_copyright.sh` rewrites headers across source files. Do not run casually.
- Detected: `tools/run/run_iwt5_validation.sh` hardcodes `/mnt/d/vscode_dir/open_vins` and writes under `20260509_fly{N}/result/init_window_time_sweep/`.
- Detected: Chinese logs/docs can mojibake in PowerShell unless UTF-8 is forced. Prefer explicit UTF-8 reads and `PYTHONIOENCODING=utf-8` for Python output.
- Detected: Some scripts and docs assume WSL paths; avoid fragile PowerShell/WSL quoting and CRLF shell script failures.
- Detected: `analysis/flight_eval_tool.py` has a durable expectation for output folders with separate `plots/`, `tables/`, `data/`, `reports/`, and `metadata/`.
- Detected: `analysis/flight_eval/assets/dashboard_app.js` is an offline dashboard consumer; frontend should not recompute backend metrics.

## Verification Expectations

- For C++ estimator changes, rebuild the touched target, usually `run_serial_msckf_ros_free`, and run a targeted binary/test when available.
- For `AdaptiveStrideController` or camera-frame stride changes, include a baseline/fly validation and compare logs/metrics against the requested control.
- For GPS-Z or yaw consistency changes, use baseline modes instead of ad hoc low-level flag combinations when possible.
- For analysis changes, run `inspect-run` before `single` and verify required outputs plus `metadata/provenance.json`.
- For `mems-ins-workbench/`, run `npm run build` and `npm run lint` when frontend files change.

## Needs Confirmation

- Needs confirmation: external flight datasets under Desktop paths may not exist on every machine/session.
- Needs confirmation: WSLg/display availability before realtime visualization.
- Needs confirmation: exact matplotlib Chinese font availability on Windows; missing CJK fonts have caused generated figures to show boxes.
- Needs confirmation: active Git branch name and current dirty changes before making commits; preserve user changes regardless.
