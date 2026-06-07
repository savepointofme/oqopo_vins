# Git Diff Audit — 2026-06-06 (written 2026-06-07)

Audit of every item in `git status --short` classified as:
- **KEEP** — intentional change, must be in next commit
- **REVERT** — accidental / scratch change, revert before committing
- **ARCHIVE** — untracked file, move to `experiments/archive/` (do not commit to main tree)
- **IGNORE** — untracked junk/temp file, delete or leave outside repo
- **UNKNOWN** — needs human decision before commit

Summary: 64 tracked modified files (1916 ins / 2380 del), ~170 untracked files/dirs.

---

## 1. Modified Tracked Files (M)

### ReadMe.md — REVERT
Scratch B1–B4 calibration run commands appended to end of ReadMe.md.
These are ad-hoc command notes, not proper documentation. **Revert this change.**
```
git checkout -- ReadMe.md
```

### config/d455_fly2/estimator_config.yaml — KEEP
Comment update: FC init comment clarified ("currently under re-audit; do not use deleted ad-hoc fly2 init CSVs"). Functional parameters unchanged.

### config/user_drone_mono_jc82/estimator_config.yaml — KEEP
Active mono drone config. Changes expected from this branch's mono-mode work.

### eval_stage.py — KEEP
Active evaluation script. Extended with GPS-Z, yaw alignment modes, segment eval.

### ov_core/cmake/ROS1.cmake — KEEP
Build system change for this branch.

### ov_core/src/track/TrackBase.h — KEEP
Gyro-aided KLT feature: IMU rotation prediction interface.

### ov_core/src/track/TrackDescriptor.cpp — KEEP
Descriptor tracker updates.

### ov_core/src/track/TrackDescriptor.h — KEEP
Descriptor tracker header updates.

### ov_msckf/CMakeLists.txt — KEEP
Build system: ROS-free runner, new source files added.

### ov_msckf/cmake/ROS1.cmake — KEEP
Build system change for this branch.

### ov_msckf/src/core/VioManager.cpp — KEEP
GPS-Z fusion, OC mode dispatch, FC init, yaw gauge modes. Core of this branch.

### ov_msckf/src/core/VioManager.h — KEEP
VioManager interface changes.

### ov_msckf/src/core/VioManagerOptions.h — KEEP
New CLI flags: GPS-Z, OC mode, FC init, etc.

### ov_msckf/src/ros_free/DiagLogger.cpp — KEEP
ROS-free diagnostic logging.

### ov_msckf/src/ros_free/DiagMetrics.h — KEEP
Diagnostic metrics types.

### ov_msckf/src/ros_free/TrajectoryAligner.h — KEEP
SE3 Umeyama alignment for ROS-free eval.

### ov_msckf/src/ros_free/VizDashboard.cpp — KEEP
OpenCV dashboard implementation.

### ov_msckf/src/ros_free/VizDashboard.h — KEEP
Dashboard header.

### ov_msckf/src/run_serial_msckf_ros_free.cpp — KEEP
ROS-free offline runner entry point.

### ov_msckf/src/state/State.cpp — KEEP
FEJ frozen state, Schmidt gauge state management.

### ov_msckf/src/state/State.h — KEEP
State header with FEJ and gauge additions.

### ov_msckf/src/state/StateHelper.cpp — KEEP
OC projection in EKFUpdate, covariance operations.

### ov_msckf/src/state/StateHelper.h — KEEP
StateHelper interface.

### ov_msckf/src/state/StateOptions.h — KEEP
State options with new OC flags.

### ov_msckf/src/update/UpdaterGroundPlaneFeature.cpp — KEEP
Ground plane feature updater.

### ov_msckf/src/update/UpdaterGroundPlaneFeature.h — KEEP

### ov_msckf/src/update/UpdaterGroundPlaneFeatureV1.cpp — KEEP

### ov_msckf/src/update/UpdaterGroundPlaneFeatureV1.h — KEEP

### ov_msckf/src/update/UpdaterGroundPlaneRange.cpp — KEEP

### ov_msckf/src/update/UpdaterGroundPlaneRange.h — KEEP

### tools/fc_to_gps_csv.py — KEEP
FC flight controller GPS extraction tool.

### tools/fc_to_init_csv.py — KEEP
FC init state extraction tool.

---

## 2. Deleted Tracked Files (D)

All deletions are under `comparison_plots/` — generated artifacts (PNGs, diagnostic scripts,
generated text files). These were committed in a previous session and are no longer needed
in the tracked tree.

### comparison_plots/README.md — ARCHIVE
Generated README for comparison plots. No longer needed in tree.

### comparison_plots/comparison_fly{1,2,3,4}.png — ARCHIVE
Generated comparison plot images. Not needed in repo; backed up in zip packages.

### comparison_plots/comparison_summary.png — ARCHIVE
Generated summary plot.

### comparison_plots/plot_fly_comparison.py — KEEP (do not restore as tracked)
Useful comparison script. If needed, move to `tools/diag/` and stage as new untracked file.

### comparison_plots/stage_b_diag/* (all 19 files) — ARCHIVE
Generated diagnostic plots, scripts, metrics files for Stage B analysis.
All backed up in zip packages. Do not restore.

**Decision:** All deletions under `comparison_plots/` are intentional. Do not restore.

---

## 3. Untracked Files (??)

### Junk / Temp Files — IGNORE (delete)

| File | Reason |
|------|--------|
| `echo` | Shell artifact — `echo something > echo`. Delete. |
| `"[eval done] AC until=\357\201\234"` | Garbled filename from eval script. Delete. |
| `"\357\200\242"` | Unicode junk filename. Delete. |

---

### New Source Files — KEEP (stage for commit)

| File | Reason |
|------|--------|
| `ov_msckf/src/test_joseph_update.cpp` | Active unit test for Joseph-form update. Keep. |
| `ov_msckf/src/core/TerrainEstimator1D.h` | **UNKNOWN** — inspect before deciding. May be active or experimental terrain altitude estimator. |

### New Source Files — ARCHIVE

| File | Reason |
|------|--------|
| `ov_msckf/src/test_constrained_yaw_nullspace.cpp` | Test for retired constrained nullspace mode (smoke failed). Archive. |

---

### Tool Scripts — KEEP (stage for commit)

| File | Reason |
|------|--------|
| `tools/align/align_fc_yaw_to_imu.py` | FC yaw alignment tool, used for FC init. |
| `tools/diag/combine_xy_plots.py` | Diagnostic plot tool. |
| `tools/diag/eval_modes_vs_gps.py` | Active eval diagnostic. |
| `tools/diag/plot_bgz_all_flights.py` | BGZ plot tool. |
| `tools/diag/plot_traj_all_flights.py` | Trajectory plot tool. |
| `tools/diag/plot_traj_vs_gps.py` | Trajectory vs GPS plot tool. |
| `tools/workspace_migrate.py` | Workspace migration script from 2026-05-18 audit. |
| `run_fly3_baseline.sh` | Active fly3 baseline run script (openvins_fej + official_oc). |
| `run_gpsz_on_fly1.sh` | Active GPS-Z run for fly1. |
| `run_gpsz_on_fly2.sh` | Active GPS-Z run for fly2. |
| `run_gpsz_on_fly3.sh` | Active GPS-Z run for fly3. |
| `run_gpsz_on_fly4.sh` | Active GPS-Z run for fly4. |
| `run_eval_all_flights_fej_oc.sh` | Active eval script for all-flight FEJ vs OC comparison. |
| `run_experimental_oc_modes.sh` | Experimental C/D mode runner. EXPERIMENTAL_ONLY label preserved. |
| `pack_results.sh` | Packaging tool for result archives. |

---

### Documentation — KEEP (stage for commit)

| File | Reason |
|------|--------|
| `artifact_manifest_20260606.md` | Active manifest of all experiment packages. |
| `baseline_mode_cleanup_summary_20260606.md` | Cleanup summary doc. |
| `baseline_restoration_report.md` | Historical baseline restoration record. |
| `docs/GPS_REFERENCE_AUDIT.md` | GPS reference system audit. |
| `experiment_logging_convention_20260606.md` | Logging convention reference. |
| `failed_modes_retirement_log_20260606.md` | Retirement log with C/D entries. |
| `fly3_oc_variant_validation_20260606.md` | Fly3 OC validation results (session deliverable). |
| `gps_z_on_four_flights_baseline_results.md` | GPS-Z baseline results. |
| `gps_z_on_four_flights_rerun_results.md` | GPS-Z rerun results. |
| `gpsz_on_yaw_gauge_comparison_brief.md` | GPS-Z vs yaw gauge comparison. |
| `method_selection.md` | Method selection rationale. |
| `oc_vs_fej_1e4_current_evidence_20260606.md` | OC vs FEJ evidence summary. |
| `official_gpsz_modes_20260606.md` | Official GPS-Z mode registry. |
| `official_yaw_modes_20260606.md` | Official yaw mode registry (updated this session). |
| `orthodox_oc_audit_20260606.md` | Full OC mode audit. |
| `workspace_cleanup_inventory_20260606.md` | Workspace inventory. |
| `yaw_gauge_modes_design.md` | Yaw gauge design doc. |

---

### Documentation — ARCHIVE (do not commit to repo root)

| File | Reason |
|------|--------|
| `constrained_ekf_nullspace_design.md` | Failed mode (smoke failed 2026-06-06). |
| `constrained_yaw_nullspace_smoke_report.md` | Failed mode smoke report. |
| `dso_prior_align_design.md` | Failed DSO route design. |
| `dso_route_corrected_derivation.md` | Failed DSO derivation. |
| `dso_vins_alignment_audit.md` | DSO/VINS alignment audit (routes retired). |
| `dso_vins_alignment_implementation_plan.md` | DSO/VINS impl plan (retired). |
| `dso_vins_literature_nullspace_audit.md` | Literature audit for retired routes. |
| `four_route_nullspace_audit.md` | Four-route audit (routes 3/4 retired). |
| `msckf2_ablation_results.md` | MSCKF2 alias disabled. Ablation results archived. |
| `msckf2_code_audit.md` | MSCKF2 code audit (alias disabled). |
| `msckf2_implementation_plan.md` | MSCKF2 impl plan (disabled). |
| `new_height_aid_research.md` | Height aid Arch-G was rejected. |
| `porting_candidates.md` | Research notes on porting. |
| `pr_body.md` | Draft PR body — stale. |
| `research_notes.md` | Ad-hoc research notes. |
| `vins_anchor_gauge_design.md` | VINS route retired (n_zeroed=0, never worked). |

---

### Config Variants — ARCHIVE (do not commit)

All untracked YAML config files are experimental variants used during ablation studies.
None are active baseline configs.

| File | Reason |
|------|--------|
| `config/d455_fly1/estimator_config_fly2params_cond_1e5.yaml` | 1e5 cond sweep (not baseline). |
| `config/d455_fly1/estimator_config_fly2params_cond_1e5_3hz.yaml` | Same, 3hz variant. |
| `config/d455_fly1/estimator_config_gpf_c.yaml` | Ground plane feature experiment. |
| `config/d455_fly1/estimator_config_gpf_e0.yaml` | Ground plane feature experiment. |
| `config/d455_fly1/estimator_config_gpf_e1.yaml` | Ground plane feature experiment. |
| `config/d455_fly1/estimator_config_gplane_B2.yaml` | Ground plane experiment. |
| `config/d455_fly1/estimator_config_gplane_d195.yaml` | Ground plane experiment. |
| `config/d455_fly1/estimator_config_gplane_diag.yaml` | Ground plane experiment. |
| `config/d455_fly1/estimator_config_noslam.yaml` | No-SLAM ablation. |
| `config/d455_fly2/estimator_config_cond1e5.yaml` | 1e5 cond sweep. |
| `config/d455_fly2/estimator_config_cond_1e5.yaml` | 1e5 cond sweep. |
| `config/d455_fly2/estimator_config_cond_1e5_klt_nogyro.yaml` | KLT no-gyro ablation. |
| `config/d455_fly2/estimator_config_cond_1e5_no_gyro_klt.yaml` | Same. |
| `config/d455_fly2/estimator_config_cond_1e5_orb.yaml` | ORB tracker experiment. |
| `config/d455_fly2/estimator_config_cond_1e5_sp.yaml` | SuperPoint experiment. |
| `config/d455_fly2/estimator_config_cond_1e5_xfeat.yaml` | XFeat experiment. |
| `config/d455_fly2/estimator_config_cond_1e5_xfeat_hint.yaml` | XFeat hint experiment. |
| `config/d455_fly2/estimator_config_cond_1e6_sweep.yaml` | 1e6 cond sweep. |
| `config/d455_fly2/estimator_config_cond_3e5.yaml` | 3e5 cond sweep. |
| `config/euroc_mav/estimator_config_orb.yaml` | ORB on EuRoC (not drone config). |
| `config/euroc_mav/estimator_config_xfeat.yaml` | XFeat on EuRoC (not drone config). |

---

### Run Scripts — ARCHIVE (do not commit)

All scripts below were used for specific ablation/experiment runs. They are not active
baseline scripts and should not clutter the repo root.

**Fly4 ablation runs (old date-stamped scripts):**
- `run_20260528_gsmq_d455_fly4_yaw_method_ablation_*.sh` (8 scripts) — ARCHIVE

**Fly1 experiment scripts:**
- `run_fly1_R1_baseline_restore.sh`, `run_fly1_ablation_gps_off.sh` — ARCHIVE
- `run_fly1_ablation_partial_schmidt.sh`, `run_fly1_arch_g_full.sh` — ARCHIVE
- `run_fly1_baseline_oc_1hz/2hz/3hz/30hz.sh` — ARCHIVE
- `run_fly1_bgzblock_2hz.sh`, `run_fly1_gate2_joseph_300s.sh` — ARCHIVE
- `run_fly1_gate_arch_g_300s.sh` — ARCHIVE
- `run_fly1_klt_*.sh` (10hz, 1hz, 1hz_noslam, 2hz, 2hz_noslam, 3hz, 30hz, 30hz_noslam) — ARCHIVE
- `run_fly1_warp_1hz/2hz/3hz.sh` — ARCHIVE
- `run_fly1_yaw_gauge_ablation.sh` — ARCHIVE
- `run_fly1_bgzblock_2hz.sh` — ARCHIVE

**Fly3 experiment scripts (obsolete):**
- `run_fly3_alpha_sweep.sh` — ARCHIVE
- `run_fly3_global_oc_recovered_until1700_offset438p0.sh` — ARCHIVE
- `run_fly3_gpsz_init_ablation.sh` — ARCHIVE
- `run_fly3_gyroklt_compare.sh`, `run_fly3_klt_gyro_compare.sh` — ARCHIVE
- `run_fly3_newoc_test.sh` — ARCHIVE
- `run_fly3_orb.sh`, `run_fly3_sp.sh`, `run_fly3_xfeat.sh`, `run_fly3_xfeat_method1/2.sh` — ARCHIVE
- `run_fly3_yaw_gauge_ablation.sh` — ARCHIVE
- `run_fly3_yaw_method_ablation_offset438p0_start618.sh` — ARCHIVE
- `run_fly3_yaw_method_ablation_until1600_offset438p0.sh` — ARCHIVE

**Fly4 eval scripts:**
- `run_fly4_bc_eval.sh`, `run_fly4_eval_only.sh` — ARCHIVE
- `run_fly4_guard_ablation.sh`, `run_fly4_guard_variants.sh` — ARCHIVE

**Condition comparison eval scripts:**
- `run_eval_cond3.sh`, `run_eval_cond3b.sh`, `run_eval_cond3c.sh` — ARCHIVE
- `run_eval_modes_vs_gps.sh` — ARCHIVE
- `run_ac_eval.sh` — ARCHIVE
- `run_all_yaw0_gpsalt_gplane_ablation_headless.sh` — ARCHIVE
- `run_s2bc_explicit.sh`, `run_schmidt_eval_fresh.sh` — ARCHIVE

**Experimental OC mode run scripts (C and D on all flights):**
- `run_yawcmp_oc_4d_fly{1,2,3,4}.sh` — ARCHIVE (superseded by run_experimental_oc_modes.sh)
- `run_yawcmp_oc_prechi2_fly{1,2,3,4}.sh` — ARCHIVE

**EuRoC scripts:**
- `run_euroc_orb.sh`, `run_euroc_xfeat.sh` — ARCHIVE

**Misc one-off scripts:**
- `setup_xfeat.sh`, `rebuild_xfeat.sh` — ARCHIVE
- `launch_gpsz_tmux.sh`, `launch_yawcmp_tmux.sh` — ARCHIVE

---

### Check/Inspect/Test Scripts — ARCHIVE

All `check_*.sh`, `inspect_*.sh`, `smoke_*.sh`, `test_*.sh`, `verify_*.sh`,
`sweep_*.sh`, `diagnose_*.sh`, `diag_*.sh`, `stage3_report.sh`, `wait_stage3.sh`,
`quick_diag.sh`, `show_gps.sh`, `find_imu.sh` scripts are ad-hoc diagnostics.

| Script group | Count | Decision |
|--------------|-------|----------|
| `check_*.sh` | 8 | ARCHIVE |
| `inspect_*.sh` | 8 | ARCHIVE |
| `smoke_*.sh` | 2 | ARCHIVE |
| `test_*.sh` | ~20 | ARCHIVE |
| `verify_*.sh` | 3 | ARCHIVE |
| `sweep_alpha_*.sh` | 3 | ARCHIVE |
| `diag_*.py` | 5 | ARCHIVE |
| `inspect_xfeat*.py`, `inspect_superpoint.py` | 5 | ARCHIVE |
| `eval_alpha_sweep.sh`, `eval_gpsz_on_rerun.sh` | 2 | ARCHIVE |
| `collect_cond3_metrics.sh`, `compare_track_stats.sh` | 2 | ARCHIVE |
| `find_imu.sh`, `show_gps.sh`, `quick_diag.sh` | 3 | ARCHIVE |
| `stage3_report.sh`, `wait_stage3.sh`, `stage3_report.sh` | 2 | ARCHIVE |
| `eval_phase6.py`, `check_vop_precision.py` | 2 | ARCHIVE |
| `plot_subsample_traj.py`, `plot_yaw_comparison.py` | 2 | ARCHIVE |
| `pack_3cond.py` | 1 | ARCHIVE |

---

### Package Directories — DO NOT DELETE, DO NOT COMMIT AS UNTRACKED

| Directory | Status |
|-----------|--------|
| `gpsz_on_4flight_fresh_rerun_package/` | PRESERVE — zipped as `gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip`. Directory is redundant; do not commit. |
| `gpsz_on_yaw_gauge_comparison_package/` | PRESERVE — zipped as `gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip`. Directory is redundant; do not commit. |

---

## 4. Summary Action Table

| Action | Files |
|--------|-------|
| **REVERT before commit** | `ReadMe.md` (scratch commands appended) |
| **KEEP in next commit** | All 31 modified tracked C++ files, `eval_stage.py`, both configs, `tools/*.py` |
| **KEEP as untracked (stage for commit)** | `run_fly3_baseline.sh`, `run_gpsz_on_fly{1,2,3,4}.sh`, `run_eval_all_flights_fej_oc.sh`, `run_experimental_oc_modes.sh`, `tools/align/`, `tools/diag/`, `tools/workspace_migrate.py`, all `*_20260606.md` docs, `docs/GPS_REFERENCE_AUDIT.md`, `yaw_gauge_modes_design.md`, `method_selection.md`, etc. |
| **ARCHIVE (move to experiments/archive/)** | All failed-mode .md docs, all ablation run scripts, all config variants, diag/inspect/smoke/test scripts |
| **IGNORE / DELETE** | `echo`, `"[eval done] AC until=..."`, `"\357\200\242"` |
| **UNKNOWN — needs inspection** | `ov_msckf/src/core/TerrainEstimator1D.h` |

---

## 5. Immediate Pre-Commit Checklist

Before any `git add` / `git commit`:

- [ ] `git checkout -- ReadMe.md` — revert scratch run commands
- [ ] Inspect `ov_msckf/src/core/TerrainEstimator1D.h` — determine KEEP vs ARCHIVE
- [ ] Delete junk files: `echo`, the two garbled filenames
- [ ] Move ARCHIVE scripts/docs to `experiments/archive/` folder (do not stage them)
- [ ] Confirm `comparison_plots/` deletions are intentional (they are — generated artifacts)
- [ ] Run `./run_format.sh` if any C++ files were touched after last format run
- [ ] Verify build still passes after ReadMe revert

---

## 6. Files That Must NOT Be Committed Regardless

Per operational constraints:
- Do not commit DSO/VINS implementation files in an inconsistent state
- Do not commit experimental C/D mode scripts as "baseline"
- Do not commit any `.env`, credential, or secret files (none detected)
- Do not commit zip packages (too large; already preserved on Desktop)
- Do not commit package directories (`gpsz_on_4flight_fresh_rerun_package/`, etc.)
