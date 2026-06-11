# Experiment Logging Convention — 2026-06-06

From this date forward, every experiment must follow this structure.
No undocumented runs. No result folders without status markers.

---

## Current Folder Structure

This convention supersedes the legacy in-repository layout below.

```text
C:/Users/baloney/Desktop/实验目录/
  <original_experiment_folder_name>/
    <YYYYMMDD_flight_MODE_key-config_STATUS>/
      plots/
      tables/
      data/
      reports/
      metadata/
      raw/                    optional original run files
      logs/                   optional stdout/stderr
```

The run folder name must include experiment date, flight, applied mode and key
configuration, and final status. `metadata/` preserves exact source files,
configuration, commit when available, and status.

## Legacy Folder Structure

```
experiments/YYYYMMDD_short_name/
  README.md                   ← required; see template below
  run_manifest.csv            ← one row per run; see schema below
  summary.csv                 ← headline eval metrics per run
  plots/                      ← eval plots (optional)
  logs/                       ← stdout/stderr logs (optional summary)
  packages/                   ← zips of complete result sets
  fly1/
    YYYYMMDD_fly1_MODE_gpsz_fi1e4_STATUS/
      command.txt             ← exact CLI command that produced this run
      config_snapshot.yaml    ← copy of estimator_config.yaml used
      git_commit.txt          ← output of `git rev-parse HEAD`
      stdout_stderr.log       ← captured output
      traj.txt                ← TUM format trajectory
      traj.txt.bias           ← IMU bias log (if generated)
      eval/
        metrics_summary.csv
        yaw_mode_comparison.csv
        gps_xy_overlay.png
        xy_ate_vs_time.png
        yaw_err_vs_time.png
      analysis/
        analysis_provenance.json
        global_summary.csv
        metric_statistics.csv
        segment_error_summary.csv
        figures/
        FULL_FLIGHT_ERROR_ANALYSIS.md
      STATUS_OK.txt           ← OR one of: STATUS_DIVERGED, STATUS_CRASH,
                              ←            STATUS_INTERRUPTED, STATUS_PARTIAL
  fly2/ fly3/ fly4/           ← same structure
```

---

## Run Folder Naming Convention

```
YYYYMMDD_flyN_MODE_gpsz_CONDNUMBER_STATUS
```

Examples:
```
20260606_fly3_oc2_gpsz_fi1e4_OK
20260606_fly2_fej_gpsz_fi1e4_CRASH_t1858
20260606_fly3_globaloc_nogpsz_PARTIAL
20260606_fly4_msckf2_gpsz_fi1e5_OK
20260606_fly1_constrained_gpsz_fi1e4_SMOKE_FAIL
```

Rules:
- `MODE`: use official name — `fej`, `oc2`, `msckf2`, `oc_prechi2`, `oc_legacy_fej`
- `gpsz` / `nogpsz`: whether GPS-Z was enabled
- `fi1e4` / `fi1e5`: fi_max_cond_number
- `STATUS`: always set after run completes

---

## README.md Template

```markdown
# Experiment: [short name]

**Date**: YYYY-MM-DD
**Branch**: [git branch]
**Commit**: [git rev-parse HEAD]

## Goal
[1-2 sentences: what question this experiment answers]

## Hypothesis
[what outcome is expected and why]

## Flights
fly1 / fly2 / fly3 / fly4 (circle which apply)

## Config
- Config file: config/d455_fly2/estimator_config.yaml
- fi_max_cond_number: 10000 / 100000
- use_fej: true / false

## Yaw Mode
- --vio-yaw-gauge-mode: [alias]
- Resolves to: [vio_yaw_update_mode string]

## GPS-Z
- Enabled: yes / no
- Flag set: [exact flags]

## Commands
[exact commands, one per run]

## Eval Window
- t0: [start time s]
- until: [end time s]
- yaw-align-mode: start_yaw

## Headline Metrics
| Flight | Mode | Full ATE RMS | Middle ATE RMS | Late ATE RMS | Yaw RMS |
|--------|------|-------------|----------------|-------------|---------|
| fly3   | oc2  | 323 m       | 249 m          | 399 m       | 6.1°    |

## Known Caveats
[any issues, guard triggers, partial runs, etc.]
```

---

## run_manifest.csv Schema

```csv
run_id,date,flight,mode,gpsz,fi_cond,t0,until,git_commit,status,full_ate_rms,middle_ate_rms,late_ate_rms,yaw_rms,notes
20260606_fly3_oc2_gpsz_fi1e4,2026-06-06,fly3,oc_mode2,yes,1e4,618,1837,2538839f,OK,323.0,249.0,399.0,6.1,fresh 1e4 run
```

---

## Status Marker Files

Create exactly one of these files in each run folder immediately after the run completes:

| File | Meaning |
|------|---------|
| `STATUS_OK.txt` | Run completed normally to the expected end time |
| `STATUS_DIVERGED.txt` | Run completed but trajectory diverged (large position error) |
| `STATUS_CRASH.txt` | Process crashed or exited non-zero (include crash time in filename content) |
| `STATUS_INTERRUPTED.txt` | Run was manually stopped before completing |
| `STATUS_PARTIAL.txt` | Run completed but not to full end time (include actual last_t) |
| `STATUS_SMOKE_FAIL.txt` | Smoke test failed (include criterion that failed) |

---

## command.txt Format

```bash
#!/usr/bin/env bash
# Run date: YYYY-MM-DD HH:MM
# Git commit: XXXXXXXXXX
# Flight: flyN

/path/to/run_serial_msckf_ros_free \
  --config ../../config/d455_fly2/estimator_config.yaml \
  --dataset /path/to/dataset \
  --gps /path/to/gps.csv \
  --gps-time-offset 0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-max-res 80 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --vio-yaw-gauge-mode baseline \
  --start-time 618 \
  --until-time 1837 \
  --output /path/to/result/traj.txt \
  --viz-fast --dash-every 5 \
  2>&1 | tee stdout_stderr.log
```

---

## Enforcement

Before launching any run:
1. Create the experiment folder with `README.md` filled out
2. Write `command.txt` with the exact command
3. Save `config_snapshot.yaml` and `git_commit.txt`

After the run:
1. Create the appropriate `STATUS_*.txt` file
2. Run `eval_stage.py` and save output to `eval/`
3. Add the run to `run_manifest.csv`

**No exceptions.** If a run was done without following this convention (historical),
record it in the manifest with `notes = "pre-convention run"` and do not back-fill docs.
