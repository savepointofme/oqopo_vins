# AGENTS.md instructions

## Repository Overview

- Detected: This is an OpenVINS fork for visual-inertial navigation, MSCKF/EKF research, ROS-free flight replay, GPS-Z aided evaluation, and flight-data analysis.
- Detected: Core packages are `ov_core`, `ov_init`, `ov_msckf`, `ov_eval`, with configs under `config/` and flight-analysis tooling under `analysis/`, `tools/`, and `baseline/latest/`.
- Detected: A React/TypeScript Vite prototype exists at `mems-ins-workbench/` for MEMS/INS flight-data visualization.
- Detected: CI workflows build ROS-free, ROS1 Docker, and ROS2 Docker variants under `.github/workflows/`.
- Not found: a single reliable top-level "run all tests" command.

## Communication

- 用中文进行对话、解释、计划、评审、总结和最终报告。
- 代码、变量名、文件名、提交信息、PR 内容、CLI 参数和用户未明确要求中文化的产品文案使用英文。
- 结论先行，再给原因和影响。
- DO NOT send optional commentary.
- Avoid optional or filler commentary.
- Send commentary only when required for tool use, blockers, concise status, or final results.
- 不要盲目附和；发现风险、错误前提或证据不足时直接指出。
- 不要用"要不要我继续？"、"是否需要我帮你？"、"你要我直接做吗？"或"如果你愿意，我可以..."结尾。
- 目标清楚时直接选择最短安全路径并执行；只有目标不清、缺权限或继续会造成真实风险时才提问。

## Scope And Safety

- Preserve unrelated user changes. This repository may have a dirty worktree.
- Do not run destructive commands such as `git reset --hard`, broad `git checkout`, force push, cleanup scripts, or whole-repo formatting unless explicitly requested.
- Do not modify generated datasets, reports, experiment outputs, or external state unless the user asks for that exact work.
- Before editing, inspect the relevant code, docs, scripts, and configs. Prefer small focused patches.
- Never read, print, copy, or store secret values. If secret configuration matters, document only where it is configured.

## Tech Stack

- Detected: C++ OpenVINS packages use CMake, Eigen3, OpenCV 3/4, Boost, and Ceres.
- Detected: `ov_core`, `ov_init`, and `ov_eval` set `CMAKE_CXX_STANDARD 14`; `ov_msckf` sets `CMAKE_CXX_STANDARD 17` for ONNX/XFeat/SuperPoint support.
- Detected: ROS can be disabled with `-DENABLE_ROS=OFF`; ROS1 uses catkin, ROS2 uses colcon.
- Detected: Python analysis scripts use standard Python plus packages such as `numpy`, `pandas`, and `matplotlib`.
- Detected: `mems-ins-workbench/` uses React, TypeScript, Vite, ECharts, lucide-react, npm, and `package-lock.json`.
- Detected: Optional ONNX Runtime support is gated by `-DONNXRUNTIME_DIR=...`.

## Main Directories

- `ov_core/`: camera models, feature tracking, feature database, geometry, simulation, utilities.
- `ov_init/`: static and dynamic inertial initialization.
- `ov_msckf/`: EKF/MSCKF estimator, state/covariance, updaters, ROS runners, ROS-free runner, dashboard, adaptive stride, GPS-Z and ground-plane related work.
- `ov_eval/`: offline trajectory evaluation tools and C++ evaluators.
- `config/`: datasets and drone-specific estimator/calibration YAML files, including `user_drone_mono_jc82/`, `user_drone_stereo_jc82/`, `d455_fly1/`, and `d455_fly2/`.
- `analysis/`: canonical GPS-referenced full-flight evaluation, reports, dashboards, run specs, LK diagnostics, and provenance tooling.
- `tools/`: alignment, conversion, baseline evaluation, diagnostics, and run helpers.
- `baseline/latest/`: current clean baseline entry point, configs, scripts, manifests, and audit notes.
- `docs/`: upstream OpenVINS docs and reports.
- `docs-cn/`: Chinese architecture/source-study docs and diagrams.
- `mems-ins-workbench/`: MEMS INS flight-data workbench frontend prototype.
- `build_ov_msckf/`: existing out-of-tree build output.

## Build And Run Commands

ROS-free build:

```bash
cd ov_msckf
mkdir -p build
cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
```

Existing out-of-tree build target:

```bash
cmake --build build_ov_msckf --target run_serial_msckf_ros_free -j
```

ROS1 build:

```bash
catkin_make --pkg ov_core ov_init ov_msckf ov_eval
```

ROS2 build:

```bash
colcon build --packages-select ov_core ov_init ov_msckf ov_eval
```

ROS-free runner template:

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/user_drone_stereo_jc82/estimator_config.yaml \
  --dataset /path/to/mav0 \
  --stereo \
  --gps /path/to/aligned_gps_cam_time.csv \
  --gps-time-offset 0 \
  --output /path/to/traj.txt
```

Current baseline runner:

```bash
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 12
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 1
bash baseline/latest/scripts/run_all_four_stride12.sh
```

Analyze a baseline run:

```bash
bash baseline/latest/scripts/analyze_latest_run.sh --fly fly3 --run-dir /path/to/run_dir
```

Flight evaluation:

```bash
python3 analysis/flight_eval_tool.py inspect-run --run-spec analysis/run_specs/<run>.json
python3 analysis/flight_eval_tool.py single --run-spec analysis/run_specs/<run>.json --out-root "C:/Users/baloney/Desktop/实验目录"
python3 analysis/flight_eval_tool.py compare --runs <dir1> <dir2> --out-dir <cmp_dir>
```

Frontend workbench:

```powershell
cd mems-ins-workbench
npm install
npm run dev
npm run build
npm run lint
```

## Formatting And Scripts

- Use `.clang-format` for C++ style, but avoid broad reformatting for narrow tasks.
- Detected: `run_format.sh` formats all C/C++ files and currently contains an extra `run_serial_msckf_ros_free` command after formatting. Inspect before running; do not use it as a routine narrow-change formatter.
- Detected: `run_copyright.sh` rewrites headers across source files. Do not run unless explicitly requested.
- Use `rg` for search. If unavailable, use platform-native alternatives.
- For JavaScript/TypeScript in `mems-ins-workbench/`, infer npm from `package-lock.json`.
- In PowerShell, use explicit UTF-8 handling when reading Chinese docs, logs, attachments, or generated reports.
- Avoid fragile mixed PowerShell + WSL quoting, pipes, and heredocs. For complex Linux commands, write a `.sh` script and run it inside WSL.
- Watch for CRLF in shell scripts.

## Workflow

- For complex work use `Specify -> Plan -> Task -> Execute -> Verify`.
- Complex work includes new features, architecture changes, estimator behavior changes, data contracts, security-sensitive changes, deployment changes, multi-file refactors, and unclear acceptance criteria.
- Before complex implementation, create or update the relevant PRD, tech spec, API contract, design note, or repo-equivalent document. Define acceptance criteria, risks, constraints, concrete tasks, and verification.
- For simple tasks, keep process lightweight but still read relevant files, define done, and verify before reporting.
- Prefer first-principles thinking: identify the actual problem, affected users, shortest reliable path, reusable patterns, and what can be simplified or avoided.
- Every changed line must trace back to the user goal.

## Coding Rules

- Reuse existing project patterns before adding abstractions.
- Prefer simple, inspectable, maintainable code over cleverness.
- Do not add speculative features, configuration layers, dependencies, background services, or broad rewrites.
- Keep modules small and ownership boundaries clear.
- Prefer standard library, platform-native capabilities, and already-installed dependencies.
- Comments should explain why, constraints, and design tradeoffs, not restate the code.
- For public complex estimator logic, add concise documentation of assumptions, state dimensions, units, gates, and failure modes.
- Confirm whether a filter is direct-state or error-state before changing estimator logic.
- Identify the state vector and covariance feedback/reset path before changing convergence, release, or update behavior.

## Verification

- Before starting, define what "done" means for the current task.
- Run the narrowest relevant verification before reporting completion.
- C++ build verification usually means rebuilding the touched target, commonly `run_serial_msckf_ros_free` or the relevant `test_*` target.
- Useful test targets detected in `ov_msckf`: `test_joseph_update`, `test_adaptive_stride`, `test_imu_filter`, `test_sim_meas`, `test_sim_repeat`.
- For analysis tooling, run `inspect-run` before `single` when run specs are involved.
- For baseline/evaluation changes, run the requested fly/globalbaseline experiment and report metrics, logs, and effect.
- For frontend changes in `mems-ins-workbench/`, run `npm run build` and `npm run lint` when feasible.
- If verification fails, fix and re-run relevant checks. Do not hand off the first failing draft for user spot-checking.
- Stop only when credentials are missing, permission is missing, required runtime/external data cannot be simulated, the goal is genuinely unclear, or continuing creates real destructive risk.

## Long-Running Tasks

- Avoid silent stalls.
- Use bounded commands and explicit timeouts where possible.
- Do not run indefinite `sleep`, watchers, servers, or blocking replay commands without a bounded verification plan.
- Write logs to files for experiments.
- Report checkpoints based on completed evidence.
- If a command appears stuck, stop it when safe, inspect logs/process state, explain what happened, and choose the shortest reliable next path.
- Do not ask whether to continue when the user goal is clear.

## OpenVINS Experiment Rules

- Do not use `--headless` for OpenVINS flight replay experiments unless the user explicitly overrides this.
- Realtime visualization must be opened for replay experiments by default.
- In WSL or environments with no display, `--headless` / `--no-display` may be necessary, but state that runtime visualization was unavailable.
- When validating experiments such as globalbaseline/fly runs, run the required comparison jobs in parallel when resources allow.
- For adaptive stride / repair / restart validation, do not stop after implementation; run the requested fly/globalbaseline experiment and report metrics, logs, and effect.
- The `baseline/latest` contract uses `--yaw-mode baseline`, `--height-mode guarded`, default stride `12`, no-thinning stride `1`, and common sweep strides `1 2 4 8 12 16 20 30`.
- Prefer `baseline/latest/scripts/run_baseline_fly.sh` for current baseline work because it snapshots command/config into the output directory.
- Use `--headless` in `baseline/latest/scripts/run_baseline_fly.sh` only when display is unavailable; visual runs should normally keep `--viz-fast --dash-every 5`.

## Navigation And Estimator Rules

- Do not use truth/reference/error columns as online estimator inputs.
- Separate online estimator inputs from offline evaluation data.
- Do not tune online release thresholds using future evaluation windows or baseline end-window statistics.
- Treat GNSS XY, GPS course, ground truth, stereo pseudo-reference, and reference labels as evaluation-only unless the project explicitly uses them online.
- Any use of evaluation-only fields in online logic is a correctness bug.
- Use estimator-internal signals for thresholds where possible: covariance, `P`, standard deviation, innovation, gates, correction size, transition behavior, and formal release state.
- Avoid magic constants without physical meaning. Reject obviously unreasonable thresholds and explain user impact.
- Prefer a simple outer supervisor/gate over modifying EKF core when it solves the problem safely.
- Do not change process or measurement noise blindly to improve one metric.
- Respect the current frontend path. If KLT/LK is the selected frontend, do not replace it with learned features without an experiment plan and same-criteria evaluation.
- Keep ONNX/XFeat/SuperPoint, RAFT, SERAFT, and other learned/experimental paths separate unless integration is actually verified.
- Do not claim an algorithm is better until it is integrated, run, and evaluated under the same criteria.
- Distinguish visual analysis, shadow ranking, diagnostic LK-only tracks, and actual estimator integration.

## Flight Evaluation Rules

- Use `analysis/full_flight_error_analysis.py` and `analysis/flight_eval_tool.py` for GPS-referenced flight analysis. Do not create one-off metrics scripts or silently change alignment/metric definitions.
- Primary analysis samples on GPS update time, not upsampled 30 Hz.
- Reference velocity should prefer flight-controller raw `Ve,Vn,Vu`; VIO velocity should prefer `.bias` `vx,vy,vz`.
- Main drift metrics use start-heading alignment; best-fit alignment is diagnostic only.
- Error decomposition should keep along/cross/vertical labels precise.
- Segment drift must distinguish local and global drift.
- Every result should carry provenance.
- Keep labels precise: do not confuse VINS, SINS, GNSS, MEMS, INS, VIO, LK-only, stereo pseudo-reference, or ground truth.
- Required figures and metrics must not disappear during report rewrites.
- State evaluation windows, confidence start times, and segment definitions explicitly.
- Do not hide failed or inconclusive experiments.

## GPS-Z And Reference Discipline

- For official Z evaluation in `tools/eval/eval_baselines.py`, pass `--reference-mode gps_z` explicitly with `--gps-alt-csv`.
- `--reference-mode stereo_pseudo_ref` is debug-only; never cite it as official truth.
- Do not use `truth_asl_*` from `_corrected` fly2 folders or old fly1 offset directories.
- Never use `--gps-alt-relative` for fusion experiments unless explicitly testing relative GPS mode.
- GPS horizontal must not be fused into the EKF for the current baseline. Only `--gps-alt-update` is permitted unless the task explicitly changes estimator inputs.
- Use `--gps-time-offset 0` when GPS CSV is already aligned to camera time.

## UI And Visualization Rules

- This repo contains UI and visualization code: `mems-ins-workbench/`, `analysis/flight_eval/assets/dashboard_app.js`, and `VizDashboard`.
- For frontend work, keep the workbench dense, operational, and engineering-focused. Avoid marketing/landing-page layouts.
- Use existing React/TypeScript/ECharts patterns, semantic colors, and `src/styles/tokens.css`.
- Validate text overflow, panel collapse states, drag/drop or chart interactions, and large-data rendering paths.
- Before integrating real CSV data, keep a static representative flow working and preserve semantic parsing via `src/semantics/`.
- For OpenCV/HTML dashboards, verify that plots, labels, selected segments, and provenance still render after changes.

## Windows, WSL, Docker, And Embedded Notes

- The workspace is on Windows (`D:\vscode_dir\open_vins`) and many scripts assume WSL paths such as `/mnt/d/vscode_dir/open_vins`.
- Some run scripts contain hardcoded Desktop dataset paths. Inspect before running on a new machine or relocated workspace.
- In PowerShell, set or force UTF-8 when Chinese output matters, e.g. `PYTHONIOENCODING=utf-8` for Python tools or explicit UTF-8 file reads.
- Distinguish host-side setup from target/runtime changes. Do not assume Docker, NVIDIA Container Toolkit, CUDA, Jetson clocks, USB, serial devices, or display forwarding are configured.
- Prefer minimal reversible runtime changes before device, kernel, Docker, or boot changes.
- Do not recommend flashing or target OS changes unless actually necessary and verified by repo evidence.

## Security

- Identify risks before security-sensitive work.
- Never hardcode credentials, tokens, API keys, passwords, or private keys.
- Validate and sanitize untrusted input at trust boundaries.
- Avoid leaking sensitive data through logs, reports, generated dashboards, screenshots, or errors.
- Never use unsafe execution patterns such as `eval()` in production code.
- Consider OWASP risks for uploads, user input, network requests, AI tools, dashboards, and external integrations.

## Missing Information

- Not found: a complete local dependency bootstrap script for all C++/Python/frontend components.
- Not found: one canonical command that runs every C++ test and Python analysis test.
- Needs confirmation: exact availability of external flight datasets under `C:/Users/baloney/Desktop/...` on every machine.
- Needs confirmation: display/WSLg availability before realtime visualization runs.
