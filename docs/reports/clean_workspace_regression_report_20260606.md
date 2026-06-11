# Clean Workspace Regression Report — 2026-06-06/07

**Purpose:** Verify that the workspace normalisation commit `e52d99c` did not alter VIO results.
Only two modes tested: `openvins_fej` (baseline) and `official_oc` (oc_postchi2_current_gauge).
Dataset: Fly3 only. Conditions identical to the fly3 OC variant validation session.

---

## 1. Cleanup Commit

| Item | Value |
|------|-------|
| Commit hash | `e52d99c` |
| Message | `cleanup: normalize official VIO modes and experiment workspace` |
| Files changed | 112 files changed, 14 213 insertions(+), 9 087 deletions(-) |
| Branch | `review/stage-schmidt-fly3-yaw-align` |
| Git status after commit | Clean — only `?? artifacts/` (gitignored zip files) |

Changes committed were **documentation, scripts, and workspace structure only**.
No C++ algorithm sources were modified. No config values were changed.

---

## 2. Build

| Item | Value |
|------|-------|
| Build dir | `build_ov_msckf/` (pre-existing out-of-tree) |
| CMake options | `-DENABLE_ROS=OFF` |
| Binary | `build_ov_msckf/run_serial_msckf_ros_free` (49 MB, Jun 7 10:57) |
| Exit code | 0 |
| New warnings | None |
| Build target line | `[100%] Built target run_serial_msckf_ros_free` |

---

## 3. Run Conditions

Identical to fly3 OC variant validation (commit `2538839` session).

| Parameter | Value |
|-----------|-------|
| Config | `config/d455_fly2/estimator_config.yaml` |
| Dataset | `20260527_gsmq_d455_fly3/d455_20260526_174946` |
| GPS | `result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv` |
| FC init | `fc_init_state_618_offset438p0.csv` |
| `--start-time` | 618 |
| `--gps-time-offset` | 0 |
| GPS-Z flags | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-max-res 80 --gps-alt-min-t-after-init 10 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| Display | `--no-display --viz-fast --dash-every 5` |
| Eval window | T0=618, until=1837, yaw-align-mode=start_yaw |

---

## 4. Output Directories

| Mode | Directory |
|------|-----------|
| openvins_fej | `Desktop/20260527_gsmq_d455_fly3/result/20260606_regression_clean_workspace_fly3_openvins_fej/` |
| official_oc | `Desktop/20260527_gsmq_d455_fly3/result/20260606_regression_clean_workspace_fly3_official_oc/` |

Each directory contains: `command.txt`, `git_commit.txt`, `log.txt`, `traj.txt`, `traj.txt.bias`, `eval/`, `STATUS_OK.txt`

---

## 5. Metrics

### Reference (from fly3 OC variant validation, commit `2538839`)

| Mode | Full RMS (m) | Late RMS (m) | Yaw RMS (°) | Yaw max (°) | Lines |
|------|-------------|-------------|------------|------------|-------|
| openvins_fej | 447.81 | 596.95 | 8.72 | 86.24 | 36557 |
| official_oc | 323.48 | 421.94 | 6.50 | 101.45 | 36556 |

### New (regression run, commit `e52d99c`)

| Mode | Full RMS (m) | Late RMS (m) | Yaw RMS (°) | Yaw max (°) | Lines |
|------|-------------|-------------|------------|------------|-------|
| openvins_fej | 447.81 | 596.95 | 8.72 | 86.24 | 36556 |
| official_oc | 323.48 | 421.94 | 6.50 | 101.45 | 36556 |

### Difference (new − reference)

| Mode | ΔFull RMS | ΔLate RMS | ΔYaw RMS | ΔYaw max | ΔLines |
|------|-----------|-----------|----------|----------|--------|
| openvins_fej | 0.00 m | 0.00 m | 0.00° | 0.00° | −1 |
| official_oc | 0.00 m | 0.00 m | 0.00° | 0.00° | 0 |

---

## 6. Pass/Fail Criteria

| Criterion | Threshold | Result |
|-----------|-----------|--------|
| XY Full RMS delta ≤ 1 m | ≤ 1 m | 0.00 m — PASS |
| XY Late RMS delta ≤ 1 m | ≤ 1 m | 0.00 m — PASS |
| Yaw RMS delta ≤ 0.1° | ≤ 0.1° | 0.00° — PASS |
| Yaw max delta ≤ 0.1° | ≤ 0.1° | 0.00° — PASS |
| Line count same ±1 | ±1 | −1 / 0 — PASS |
| official_oc beats openvins_fej on full RMS | B < A | 323 < 448 — PASS |
| Both runs exit 0 | exit 0 | PASS |
| No new build warnings | none | PASS |

## VERDICT: PASS

All metrics match the reference to 0.01 m / 0.01° precision. The cleanup commit `e52d99c` did not affect VIO results.

---

## 7. GPS-Z Altitude Fusion Summary

| Mode | P_zz mean | \|res\| mean | \|dp_z\| mean |
|------|-----------|-------------|--------------|
| openvins_fej | 2.02 m² | 0.70 m | 0.087 m |
| official_oc | 2.01 m² | 0.55 m | 0.036 m |

GPS-Z residuals are healthy in both modes.

---

## 8. What Changed in e52d99c

Items committed (none alter algorithm logic):
- `docs/official/` — new official mode registry and audit files
- `docs/reports/` — fly3 OC variant validation report
- `docs/archive/failed_modes/` — retirement log for experimental modes
- `scripts/` — reorganised run scripts (run_fly3_baseline.sh, run_experimental_oc_modes.sh, etc.)
- `tools/` — workspace and utility scripts
- `config/user_drone_mono_jc82/` — mono config (no change to fi_max_cond_number or GPS-Z params)
- Deletions: ~141 scattered experiment scripts removed from repo root and comparison_plots/

Items **not** changed: any C++ source, any CMakeLists, any YAML numeric value affecting VIO.
