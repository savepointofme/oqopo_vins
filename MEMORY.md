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

## P5 State-Machine Result (2026-07-13)

- Detected: P5 now uses one explicit causal policy state with priority `VISUAL_DEGRADED > LOW_ALTITUDE_SAFETY > TURN_SAFETY > DESCENT_SAFETY > NORMAL_CRUISE > HIGH_ALTITUDE_CRUISE` and independently publishes `tracking_stride` and `backend_update_stride`.
- Detected: tracking-only images advance KLT identities but their exact-timestamp FeatureDatabase observations are removed before `propagate_and_clone`; fly1/fly3 active screening recorded zero clone violations.
- Detected: registered `test_adaptive_stride` covers 16 required state/cadence/lifecycle cases and passes under CTest.
- Detected: the new controller removed old rung-ladder chatter. Active fly1/fly3 state-transition rates were 3.54 and 2.21 per minute, with zero unnecessary reversals, dwell violations, or same-state target rewrites.
- Detected: final short-screening status is `P5_STATE_MACHINE_ACTIVE_SCREENING_PASS`. The old `compute_saving_vs_fixed12` gate is invalid, and the old Dashboard-off wall/sensor 11.684/10.446 is also invalid because the initializer kept re-entering joint alignment after release.
- Detected: authoritative P5 delivery is `C:/Users/baloney/Desktop/实验目录/P5_state_machine_20260713/delivery/`; authoritative active batch is `active/20260713_102754_clean_p4_compare`.
- Detected: `--no-display` only hides the OpenCV window; it still prepares and renders the Dashboard. The new `--no-dashboard` removes Dashboard updates/rendering/history images and KLT visualization image payload clones while preserving lightweight policy points, feature IDs, timestamps, dimensions, and health counters.
- Detected: the single-run Dashboard effect was within about +/-5% (fly1 off faster, fly3 off slower), far too small to explain the 10-12x realtime deficit. Dashboard mode did not change policy cadence; trajectory differences stayed below 1 mm.
- Detected: P4 initialization now closes its finite causal startup window on the first successful atomic release. Later FC, board-IMU, and monocular frames cannot re-enter Ceres or dense covariance recovery; a 20 s collection timeout also closes the attempt until explicit reset.
- Detected: one-shot single-process Dashboard-off replay measured state-machine wall/sensor 0.720 for fly1 and 0.629 for fly3; fixed stride12 measured 0.554/0.491. All four have at least 20% host-side margin despite retaining heavy diagnostics.
- Detected: fly1/fly3 made 7/2 nonlinear solves before release because earlier visual reprojection gates failed, then each released exactly once with zero post-release calls. Navigation latency was 10.80/8.80 s.
- Detected: the selected frontend remains monocular KLT/LK (`use_klt: true`, `use_stereo: false`) with `num_pts: 400` and `max_slam: 50`; observed KLT P95 was 337/351 points and SLAM never exceeded 50.
- Detected: authoritative one-shot realtime evidence is `C:/Users/baloney/Desktop/实验目录/P5_state_machine_20260713/realtime_one_shot_alignment/20260713_191711_single_process`; report is under `delivery/realtime_one_shot/`.
- Pitfall: do not use frame-count ratios, old four-process wall time, or the old 3,231-attempt run as realtime evidence. Mark the latter `invalid_realtime_evaluation_post_release_alignment_repeated`; use single-estimator sensor span versus wall time and direct solve/release counters.

## P4/P5 Persistent-Window Redesign (2026-07-14)

- Detected: the formal P4 initializer is the fork's FC navigation + board IMU + monocular KLT joint initializer. It must retain FC position/velocity/attitude factors, board-IMU preintegration, monocular reprojection, startup FC-to-board relation, `q/p/v/bg/ba`, covariance, observability, and `AlignmentResult` injection. Do not restore or fall back to upstream visual-IMU-only initialization or nearest-FC-row initialization.
- Detected: upstream OpenVINS is referenced only for finite sliding windows, pruning, retry-on-new-data, and one-time state/FEJ/covariance lifecycle mechanics.
- Detected: total P4 collection time now has no elapsed-time failure limit. The old 20 s one-shot timeout statement above is superseded. While valid streams continue, ordinary coverage, excitation, triangulation, residual, covariance, or observability failures reject only the current finite window and collection continues.
- Detected: each solve uses bounded `{2(reference),3,5,8,12}` s candidate windows, chooses the shortest usable duration, and prunes FC, board-IMU, and selected visual snapshots outside the 12 s maximum plus interpolation/time-offset margin. Waiting longer never creates a larger graph.
- Detected: Ceres eligibility uses a changed window fingerprint, at least three new selected frames, and a minimum 0.75 s attempt interval. Identical content is not solved twice.
- Detected: a solved candidate is held fixed and evaluated only before release using a bounded 2–4 s FC/IMU/monocular validation interval. No shadow OpenVINS exists. Refinement is optional and limited to zero or one; the production path currently uses zero because a registered bounded local refinement is not yet enabled.
- Detected: after one validated release, P4 closes permanently, clears high-cost buffers, stops accepting FC/IMU/image initialization inputs, and cannot be reawakened by higher P5 tracking cadence.
- Detected: P5 no longer uses policy-state-to-fixed-stride pairs as its primary algorithm. Shared `VisualCadencePlanner` selects tracking cadence from rotation-compensated target parallax and safety caps; `BackendUpdateTrigger` uses accumulated common-track information, feature loss, latency, motion, and explicit safety triggers. Flight-state labels are safety context/caps only.
