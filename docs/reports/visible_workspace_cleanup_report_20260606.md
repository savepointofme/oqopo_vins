# Visible Workspace Cleanup Report — 2026-06-06 (executed 2026-06-07)

---

## 1. Desktop Cleanup Summary

**Before:** ~35 experiment folders, 3 generated plots, 5 misc files, 3 ONNX models.

**After:** Only raw datasets and personal files remain on the Desktop.

### Moved to `C:\Users\baloney\Desktop\VIO_experiment_archive_20260606\`

| Subfolder | Count | What |
|-----------|-------|------|
| `old_result_folders/vio_openvins/` | 5 folders | OpenVINS result dirs: external_z_fix, results_1sigma_altitude, eval_modes_out, 123 repo clone |
| `old_result_folders/ofv_other_project/` | 30 folders | All ofv_*, imu_px4_ofv_*, lk_ofv_* (different project) |
| `plots/` | 3 files | bgz_all_flights.png, traj_all_flights.png, traj_xy_all_flights.png |
| `packages/` | 5 items | phase6_traj_results.zip, W74 zip, 3 ONNX models |
| `manifests/` | 1 file | desktop_cleanup_manifest_20260606.md |

### Left on Desktop (unchanged)
- Raw D455 dataset folders: `20260517_gsmq_d455_fly1/`, `20260518_gsmq_d455_fly2/`, `20260527_gsmq_d455_fly3/`, `20260528_gsmq_d455_fly4/`
- Raw dataset tarballs (5–6 GB each, too large to move)
- `DJI_*` aerial footage folders
- `md4all-unsup/` (different project)
- `pianshang/`, `78_imu_log/` (raw data)
- Personal files and shortcuts

---

## 2. Repo Root Cleanup Summary

**Before:** 250+ files in root (scripts, docs, zips, configs, py scripts, data files, junk).

**After:** Root contains only:
```
CLAUDE.md                    ReadMe.md (reverted)     eval_stage.py
eval_smoke.py                run_format.sh            run_copyright.sh
run_size.sh                  run_fly3_baseline.sh
run_gpsz_on_fly1.sh          run_gpsz_on_fly2.sh
run_gpsz_on_fly3.sh          run_gpsz_on_fly4.sh
run_eval_all_flights_fej_oc.sh   run_experimental_oc_modes.sh
LICENSE  .gitignore  .clang-format  Doxyfile  Doxyfile-mcss
Dockerfile_*  (tracked, unchanged)
ov_core/  ov_msckf/  ov_init/  ov_eval/  config/  tools/
docs/  experiments/  artifacts/
```

### Files moved from repo root

| Category | Count | Destination |
|----------|-------|-------------|
| Fly1/3/4 ablation .sh scripts | 53 | `experiments/archive/old_unstructured/scripts/` |
| Check/inspect/smoke/test .sh scripts | 47 | `experiments/archive/old_unstructured/scripts/` |
| Eval/condition/phase scripts | 26 | `experiments/archive/old_unstructured/scripts/` |
| Python diag scripts | 15 | `experiments/archive/old_unstructured/scripts/` |
| Experimental config .yaml files | 21 | `experiments/archive/old_unstructured/configs/` |
| Data files (imu_window_fly3, traj_ros_free) | 4 | `experiments/archive/old_unstructured/data/` |
| Official docs → docs/official/ | 13 | `docs/official/` |
| Experiment reports → docs/reports/ | 9 | `docs/reports/` |
| Failed-mode docs → docs/archive/failed_modes/ | 13 | `docs/archive/failed_modes/` |
| Old design docs | 3 | `docs/archive/old_designs/` |
| Old migration/audit reports | 6 | `docs/archive/old_reports/` |
| Critical zip packages | 5 | `artifacts/packages/` |
| Non-critical zips | 1 | `artifacts/packages/` |
| Failed source files | 2 | `experiments/archive/failed_modes/` |
| Junk files deleted | 3 | deleted: `echo`, `!`, unicode junk |

---

## 3. ReadMe.md Reverted

`ReadMe.md` had scratch B1–B4 calibration run commands appended.
Reverted with `git checkout -- ReadMe.md`. Confirmed clean.

---

## 4. Official Modes Visible in Baseline Scripts

### `run_fly3_baseline.sh` — only these two modes:
```bash
--vio-yaw-gauge-mode openvins_fej         # REQUIRED_BASELINE
--vio-yaw-gauge-mode oc_postchi2_current_gauge  # OFFICIAL_OC
```

### `run_gpsz_on_fly{1,2,3,4}.sh` — GPS-Z modes only (no OC variant):
GPS-Z guarded on, `openvins_fej` mode.

### `run_eval_all_flights_fej_oc.sh` — eval only, no new runs.

---

## 5. Experimental Modes Isolated

### `run_experimental_oc_modes.sh`:
- Supports only `oc_prechi2_fej_gauge` and `oc_4d_prechi2_fej_gauge`
- Starts with `WARNING: EXPERIMENTAL_ONLY / NOT_FOR_BASELINE`
- Not referenced in any baseline script

---

## 6. Failed Modes Not Visible in Normal Entry Points

The following names cause CLI exit with error in `run_serial_msckf_ros_free`:
```
msckf2, msckf2_0, msckf2_pure, dso, vins_nullspace, constrained_yaw_nullspace, h_proj
```
Their design docs moved to `docs/archive/failed_modes/`.
Their source files moved to `experiments/archive/failed_modes/`:
- `TerrainEstimator1D.h` (rejected Arch-G height aid)
- `test_constrained_yaw_nullspace.cpp` (failed smoke test)

---

## 7. New Directory Structure

```
d:\vscode_dir\open_vins\
  docs/
    official/            ← mode registries, audit docs, artifact manifest
    reports/             ← fly3 validation, GPS-Z results, comparison reports
    GPS_REFERENCE_AUDIT.md (pre-existing)
    archive/
      failed_modes/      ← 13 docs: DSO, VINS, CEKF, msckf2, constrained-yaw designs
      old_designs/       ← new_height_aid_research, porting_candidates, research_notes
      old_reports/       ← migration docs, WORKSPACE_AUDIT, pr_body

  experiments/
    20260606_oc_vs_fej_1e4_fly3_validation/   ← README with 4-mode results
    20260606_oc_vs_fej_1e4_fly34/             ← README with fly34 comparison
    20260606_condition_comparison/            ← README with 3-condition results
    archive/
      old_unstructured/
        scripts/         ← 141 archived scripts
        configs/         ← 21 experimental YAML configs
        data/            ← imu_window_fly3, traj_ros_free
      failed_modes/      ← TerrainEstimator1D.h, test_constrained_yaw.cpp

  artifacts/
    packages/            ← all 6 zip packages (5 critical + 1 supporting)
    plots/               ← (empty, plots are in eval dirs on Desktop)
```

---

## 8. Critical Artifacts Preserved

| Package | New location | Required |
|---------|-------------|---------|
| three_condition_comparison_package_20260605_1727.zip | artifacts/packages/ | YES |
| traj_1e4_fly34_20260606.zip | artifacts/packages/ | YES |
| successful_runs_20260606.zip | artifacts/packages/ | YES |
| gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip | artifacts/packages/ | YES |
| gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip | artifacts/packages/ | YES |

Full index: `docs/official/artifact_manifest_20260606.md`

---

## 9. Git Status After Cleanup

**Tracked modified files (M):** 31 files — all intentional branch changes (VioManager, StateHelper, TrackBase, eval_stage.py, both active configs, tools)

**Tracked deleted (D):** 51 files — all intentional:
- `comparison_plots/*` — 34 generated artifacts (deleted earlier)
- Tracked scripts moved to archive: 12 shell scripts
- `BASELINES.md`, `REPRODUCE_stage_schmidt.md` — moved to docs/
- `config/d455_fly2/estimator_config_cond1e6.yaml` — moved to configs archive

**Untracked (??):** 19 items — all legitimate:
```
artifacts/                          ← new packages directory
docs/official/                      ← new official docs directory
docs/reports/                       ← new reports directory
docs/GPS_REFERENCE_AUDIT.md         ← existing doc in docs/
ov_msckf/src/test_joseph_update.cpp ← active unit test
run_eval_all_flights_fej_oc.sh      ← active eval script
run_experimental_oc_modes.sh        ← experimental OC modes script
run_fly3_baseline.sh                ← active fly3 baseline script
run_gpsz_on_fly{1,2,3,4}.sh        ← active GPS-Z scripts
tools/align/align_fc_yaw_to_imu.py  ← utility tool
tools/diag/*.py (5 files)           ← diagnostic plot tools
tools/workspace_migrate.py          ← workspace migration utility
```

**No unrelated dirty files:**
- `.clang-format` — not modified
- `.github/workflows/*` — not modified
- `Dockerfile*` — not modified
- `Doxyfile` — not modified
- `LICENSE` — not modified
- `ReadMe.md` — REVERTED to upstream

---

## 10. Build Result

```
make -j$(nproc) run_serial_msckf_ros_free
[88%] Built target ov_msckf_lib
[100%] Built target run_serial_msckf_ros_free
exit code: 0
```

Build passes. No source files were modified during this cleanup session.

---

## 11. Completion Standard Verification

| Standard | Status |
|----------|--------|
| Desktop no longer covered by random experiment folders | DONE — 35 folders moved to archive |
| Repo root no longer contains random scripts/zips/docs | DONE — 250+ files moved to docs/experiments/artifacts |
| Baseline scripts only expose openvins_fej and official_oc | DONE — confirmed in run_fly3_baseline.sh |
| Experimental OC variants isolated in experimental-only scripts | DONE — run_experimental_oc_modes.sh only |
| Failed DSO/VINS/CEKF modes not visible in normal entry points | DONE — CLI exits with error; docs archived |
| Critical zip/report/metrics artifacts preserved and indexed | DONE — all 5 critical zips in artifacts/packages/ |
| git diff excludes unrelated formatting/Docker/workflow/license/README | DONE — ReadMe reverted; others untouched |
