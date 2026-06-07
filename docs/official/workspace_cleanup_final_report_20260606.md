# Workspace Cleanup Final Report — 2026-06-06 (completed 2026-06-07)

This report summarizes the full workspace normalization effort that followed the
fly3 OC variant validation. All cleanup was performed without running new experiments,
without modifying GPS-Z fusion, and without changing the active baseline configs.

---

## 1. Official Modes Kept

### Required Baseline
| Mode | CLI | Fly3 RMS | Source |
|------|-----|----------|--------|
| `openvins_fej` | `--vio-yaw-gauge-mode openvins_fej` | 448m | Mode A validation |

### Official OC Mode
| Mode | CLI | Fly3 RMS | Source |
|------|-----|----------|--------|
| `oc_postchi2_current_gauge` | `--vio-yaw-gauge-mode oc_postchi2_current_gauge` | 323m | Mode B validation |

All future controlled comparisons must include both. The official_oc designation was
settled by the four-mode fly3 comparison completed 2026-06-07.

---

## 2. Experimental Modes Retained (NOT for Baseline)

| Mode | CLI | Fly3 RMS | Status |
|------|-----|----------|--------|
| `oc_prechi2_fej_gauge` | `--vio-yaw-gauge-mode oc_prechi2_fej_gauge` | 638m | EXPERIMENTAL_ONLY |
| `oc_4d_prechi2_fej_gauge` | `--vio-yaw-gauge-mode oc_4d_prechi2_fej_gauge` | 646m | EXPERIMENTAL_ONLY |
| `oc_postchi2_fej_gauge` | `--vio-yaw-gauge-mode oc_postchi2_fej_gauge` | — | ACCESSIBLE (fly1 Cond3 ref) |
| `schmidt_fej` | `--vio-yaw-gauge-mode schmidt_fej` | — | EXPERIMENTAL_ONLY |
| `schmidt_current` | `--vio-yaw-gauge-mode schmidt` | — | EXPERIMENTAL_ONLY |

**Do NOT call C/D modes "MSCKF 2.0".** The IJRR 2013 equivalence has not been audited.
**Do NOT add C/D to baseline tables or four-flight comparison tables.**
See `run_experimental_oc_modes.sh` for re-running them.

---

## 3. Retired / Disabled Modes

These modes cause CLI exit with error. Names are blocked to prevent accidental use.

| Old alias | Blocked because |
|-----------|----------------|
| `msckf2`, `msckf2_0`, `msckf2_pure` | No IJRR 2013 equivalence audited |
| `dso` | Smoke failed: altitude diverges t=450s |
| `vins_nullspace` | Never worked: n_zeroed=0 throughout |
| `constrained_yaw_nullspace` | Smoke failed: P_n exhausted, covariance indefinite |
| `h_proj` | Superseded by oc_postchi2_current_gauge |

Full details: `failed_modes_retirement_log_20260606.md`

---

## 4. Aliases Removed / Renamed

Previously ambiguous aliases that pointed to incorrect or conflicting modes:
- `msckf2_pure` → was pointing to openvins_fej without documentation → **disabled**
- `dso` → was applied to two different routes silently → **disabled**
- `global_yaw_oc_projection` → still accessible but prefer `oc_postchi2_current_gauge`
- `baseline`, `r1` → still accessible as aliases for oc_postchi2_current_gauge

Naming rule: All mode names are descriptive (what the code does), not paper names.

---

## 5. Code Files Changed This Session

### New files staged for commit
| File | Purpose |
|------|---------|
| `run_experimental_oc_modes.sh` | Consolidated runner for C/D experimental modes |
| `experiments/20260606_oc_vs_fej_1e4_fly3_validation/README.md` | Experiment record |
| `experiments/20260606_oc_vs_fej_1e4_fly34/README.md` | Experiment record |
| `experiments/20260606_condition_comparison/README.md` | Experiment record |
| `experiments/archive/README.md` | Archive index |
| `experiments/archive/failed_modes/README.md` | Failed modes index |

### Documentation files (new, stage for commit)
| File | Purpose |
|------|---------|
| `fly3_oc_variant_validation_20260606.md` | Four-mode fly3 OC comparison results |
| `official_yaw_modes_20260606.md` | Mode registry with official_oc designation |
| `orthodox_oc_audit_20260606.md` | Full OC implementation audit with fly3 results |
| `failed_modes_retirement_log_20260606.md` | Retirement log with C/D appended |
| `baseline_mode_cleanup_summary_20260606.md` | What changed this cleanup session |
| `cleanup_git_diff_audit_20260606.md` | This session's git diff classification |
| `workspace_cleanup_final_report_20260606.md` | This file |

### Must REVERT before commit
| File | Reason |
|------|--------|
| `ReadMe.md` | Scratch B1-B4 calibration run commands appended — remove with `git checkout -- ReadMe.md` |

---

## 6. Experiment Folders Organized

New structure under `experiments/`:
```
experiments/
  20260606_oc_vs_fej_1e4_fly3_validation/  ← fly3 four-mode OC comparison
  20260606_oc_vs_fej_1e4_fly34/            ← traj_1e4_fly34_20260606 reference
  20260606_condition_comparison/           ← three_condition_comparison package reference
  archive/
    old_unstructured/                      ← early experiments without naming convention
    failed_modes/                          ← DSO, VINS, constrained yaw smoke results
  iwt5_vs_iwt2_eval/                       ← pre-existing
  pr20_gps_alt_fusion/                     ← pre-existing
  stage_b_v1_diagnostics/                  ← pre-existing
  vel_klt_archived/                        ← pre-existing
```

Actual result data lives on Desktop under `20260527_gsmq_d455_fly3/result/`.
Zip packages preserved at repo root — DO NOT DELETE.

---

## 7. Artifacts Preserved

The following zip packages must NOT be deleted or overwritten:

| Package | Path | Required |
|---------|------|---------|
| `three_condition_comparison_package_20260605_1727.zip` | `d:/vscode_dir/open_vins/` | YES |
| `traj_1e4_fly34_20260606.zip` | `d:/vscode_dir/open_vins/` | YES |
| `successful_runs_20260606.zip` | `d:/vscode_dir/open_vins/` | YES |
| `gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip` | `d:/vscode_dir/open_vins/` | YES |
| `gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip` | `d:/vscode_dir/open_vins/` | YES |

Full manifest: `artifact_manifest_20260606.md`

---

## 8. Untracked Files Classification Summary

Classified in `cleanup_git_diff_audit_20260606.md`. Summary:

| Category | Count | Action |
|----------|-------|--------|
| KEEP (new docs/tools, stage for commit) | ~30 | Stage selectively |
| ARCHIVE (ablation scripts, config variants, failed mode docs) | ~140 | Move to `experiments/archive/` |
| IGNORE/DELETE (junk filenames) | 3 | Delete: `echo`, two garbled filenames |
| UNKNOWN resolved | 1 | `TerrainEstimator1D.h` → ARCHIVE (unreferenced, rejected height aid) |

---

## 9. Run Status Markers

STATUS_OK.txt written to all key fly3 run directories:
- `result/fly3_openvins_fej_gpsz_fi1e4_fcinit/STATUS_OK.txt` (Mode A)
- `result/gpsz_oc2_1e4_fly3/STATUS_OK.txt` (Mode B — official_oc)
- `result/fly3_oc_prechi2_fej_gauge_gpsz_fi1e4_fcinit/STATUS_OK.txt` (Mode C — EXPERIMENTAL)
- `result/fly3_oc_4d_prechi2_fej_gauge_gpsz_fi1e4_fcinit/STATUS_OK.txt` (Mode D — EXPERIMENTAL)
- `result/eval_fly3_{A,B,C,D}/STATUS_OK.txt` (eval outputs)

Each STATUS_OK.txt includes: mode, lines, GPS-Z active, FC init, full RMS, late RMS, eval dir.

---

## 10. Build / Test State

Build not re-run this session (no C++ source files were changed).
Last known good build: build_ov_msckf/ at commit 2538839 (fix spurious VIO-YAW warning).
All fly3 mode A/B/C/D runs completed to 36556-36557 poses — binary is confirmed working.

---

## 11. Remaining TODOs (before commit)

1. `git checkout -- ReadMe.md` — revert scratch commands
2. Delete junk files: `echo`, `"[eval done] AC until=..."`, `"\357\200\242"`
3. Inspect and move ~140 ARCHIVE files to `experiments/archive/` or leave untracked
4. Stage specific files for commit (see Section 5)
5. Confirm `ov_msckf/src/core/TerrainEstimator1D.h` is not referenced (confirmed: safe to archive)
6. Do NOT run fly2, do NOT start threshold exploration, do NOT run more yaw modes

---

## 12. Constraints Not Changed

Per explicit instructions, the following were NOT modified:
- GPS-Z fusion logic (VioManager GPS-Z update code)
- Height aiding (GPS-alt guard flags unchanged)
- Feature settings (fi_max_cond_number, max_slam, sigma_px)
- Active configs (d455_fly2/estimator_config.yaml except comment update)
- Baseline run scripts (run_fly3_baseline.sh only uses openvins_fej and official_oc)
- Existing zip packages (all five packages preserved, not overwritten)
