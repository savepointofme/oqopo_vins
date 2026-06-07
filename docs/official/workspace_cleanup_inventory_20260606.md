# Workspace Cleanup Inventory — 2026-06-06

## Git State

```
branch:  cleanup/workspace_norm_20260606
base:    review/stage-schmidt-fly3-yaw-align
HEAD:    2538839faf0f541d8fea37ab35bc74eed0b645a9
commit:  fix(vio): suppress spurious VIO-YAW warning for prechi2 OC modes
```

---

## Modified Files (tracked, not committed)

These are relative to upstream. All are pre-existing modifications on the
`review/stage-schmidt-fly3-yaw-align` lineage.

### Core estimator changes (intentional, keep)
- `ov_msckf/src/core/VioManager.cpp` — GPS-Z fusion, yaw gauge dispatch
- `ov_msckf/src/core/VioManager.h` — GPS-Z fields, TerrainEstimator hooks
- `ov_msckf/src/core/VioManagerHelper.cpp` — helper functions
- `ov_msckf/src/core/VioManagerOptions.h` — option structs
- `ov_msckf/src/state/StateHelper.cpp` — yaw OC projection logic
- `ov_msckf/src/state/StateHelper.h` — VisualYawUpdateMode enum, projection APIs
- `ov_msckf/src/state/Propagator.cpp/.h` — gyro-aided KLT prediction
- `ov_msckf/src/state/State.cpp/.h` — h_offset state element
- `ov_msckf/src/state/StateOptions.h` — use_gps_h_offset option
- `ov_msckf/src/update/UpdaterHelper.cpp/.h` — FEJ Jacobian selection
- `ov_msckf/src/update/UpdaterGroundPlaneRange.cpp/.h` — rangefinder updater
- `ov_msckf/src/update/UpdaterGroundPlaneFeature.cpp/.h` — ground-plane feature
- `ov_msckf/src/update/UpdaterGroundPlaneFeatureV1.cpp/.h` — ground-plane v1
- `ov_msckf/src/run_serial_msckf_ros_free.cpp` — ROS-free runner + CLI aliases
- `ov_msckf/src/ros_free/VizDashboard.cpp/.h` — OpenCV dashboard
- `ov_msckf/src/ros_free/DatasetReaderEuroc.h` — GPS loader
- `ov_msckf/src/ros_free/DiagLogger.cpp`, `DiagMetrics.h`, `TrajectoryAligner.h`
- `ov_core/src/track/TrackKLT.cpp/.h` — gyro-aided prediction
- `ov_core/src/track/TrackBase.h/.cpp` — IMU integration hook
- `ov_core/src/track/TrackDescriptor.cpp/.h` — descriptor tracker updates
- `ov_msckf/CMakeLists.txt`, `ov_msckf/cmake/ROS1.cmake` — build changes
- `ov_core/CMakeLists.txt`, `ov_core/cmake/ROS1.cmake`
- `config/d455_fly2/estimator_config.yaml` — active drone config
- `config/user_drone_mono_jc82/estimator_config.yaml` — mono JC82 config
- `eval_stage.py` — evaluation script

### Upstream sync changes (likely large diff noise)
- All other `config/*/`, `docs/`, `docs-cn/`, `ov_data/`, `ov_eval/`, `ov_init/`
- Dockerfiles, `.github/workflows/`, `.clang-format`, `ReadMe.md`

---

## Untracked Files — New Files Added on This Branch

### Source files to keep
- `ov_msckf/src/core/TerrainEstimator1D.h` — 1D terrain height model
- `ov_msckf/src/test_constrained_yaw_nullspace.cpp` — unit tests (T1–T9) for failed CEKF mode
- `ov_msckf/src/test_joseph_update.cpp` — unit test for Joseph-form GPS update
- `eval_stage.py` — eval pipeline (tracked as modified)
- `tools/fc_to_gps_csv.py`, `tools/fc_to_init_csv.py` — FC preprocessing tools
- `tools/align/align_fc_yaw_to_imu.py`
- `tools/diag/*.py` — diagnostic plotting scripts
- `tools/workspace_migrate.py` — workspace migration script

### Design documents (keep as archive)
- `constrained_ekf_nullspace_design.md` — CEKF Candidate E design
- `constrained_yaw_nullspace_smoke_report.md` — **CRITICAL: smoke failure evidence**
- `dso_prior_align_design.md` — DSO route design
- `dso_route_corrected_derivation.md` — DSO audit
- `dso_vins_alignment_audit.md`, `dso_vins_alignment_implementation_plan.md`
- `dso_vins_literature_nullspace_audit.md`
- `four_route_nullspace_audit.md` — all four route status
- `vins_anchor_gauge_design.md` — VINS route design
- `yaw_gauge_modes_design.md` — gauge mode taxonomy
- `gpsz_on_yaw_gauge_comparison_brief.md` — brief comparison report
- `msckf2_ablation_results.md`, `msckf2_code_audit.md`, `msckf2_implementation_plan.md`
- `method_selection.md`, `new_height_aid_research.md`, `research_notes.md`
- `gps_z_on_four_flights_baseline_results.md`, `gps_z_on_four_flights_rerun_results.md`
- `baseline_restoration_report.md`
- `BASELINES.md`, `MIGRATION_DRY_RUN.md`, `MIGRATION_PLAN.md`
- `PHASE0_BASELINE_snapshot.md`, `TARGET_LAYOUT.md`, `WORKSPACE_AUDIT.md`
- `PR20_RESULTS_SUMMARY.md`, `REPRODUCE_stage_schmidt.md`
- `porting_candidates.md`, `pr_body.md`

### Shell scripts — candidate for archive/delete
~100 untracked `.sh` scripts at repo root. Full list below:

```
check_alpha_sweep.sh          check_cond3.sh
check_fly3_progress.sh        check_formats.sh
check_orb_gated.sh            check_progress.sh
check_vop_diag.sh             collect_cond3_metrics.sh
compare_track_stats.sh        diag_east_west_scale.sh
diag_fly4_*.py (4 files)      diagnose_orb_diverge.sh
echo                          eval_alpha_sweep.sh
eval_gpsz_on_rerun.sh         eval_phase6.py
find_imu.sh                   inspect_orb_*.sh (4 files)
inspect_superpoint.py         inspect_xfeat*.py (5 files)
inspect_yaw_diag*.sh (2)      inspect_yaw_sync.sh
launch_gpsz_tmux.sh           launch_yawcmp_tmux.sh
pack_3cond.py                 pack_results.sh
plot_subsample_traj.py        plot_yaw_comparison.py
quick_diag.sh                 rebuild_xfeat.sh
run_20260528_gsmq_d455_fly4_yaw_method_ablation_*.sh (8 files)
run_ac_eval.sh                run_all_yaw0_*.sh (2 files)
run_euroc_*.sh (2 files)      run_eval_*.sh (6 files)
run_fly1_*.sh (20 files)      run_fly3_*.sh (18 files)
run_fly4_*.sh (4 files)
run_gpsz_on_fly*.sh (4 files) run_s2bc_explicit.sh
run_schmidt_eval_fresh.sh     run_yawcmp_*.sh (8 files)
setup_xfeat.sh                show_gps.sh
smoke_fly3_constrained_yaw.sh smoke_fly3_dso_vins.sh
smoke_test_schmidt_a_vs_b.sh  stage3_report.sh
sweep_alpha_*.sh (3 files)    test_all_yaw0_*.sh (6 files)
test_candidate_start_oc1.sh   test_fly1_*.sh (5 files)
test_fly3_start*.sh (8 files) test_hard_gyro_yaw*.sh
test_nooc.sh                  test_oc1*.sh (5 files)
test_start*.sh (2 files)      verify_*.sh (3 files)
wait_stage3.sh
diag_lap2_prelap.py           run_bc_eval.sh etc.
```

All are experiment-specific launch/eval wrappers. None belong in repo root.

### Config variants — candidate for cleanup
Untracked config files in `config/d455_fly1/` and `config/d455_fly2/`:
```
config/d455_fly1/estimator_config_fly2params_cond_1e5*.yaml (2)
config/d455_fly1/estimator_config_gpf_c.yaml
config/d455_fly1/estimator_config_gpf_e0.yaml
config/d455_fly1/estimator_config_gpf_e1.yaml
config/d455_fly1/estimator_config_gplane_B2.yaml
config/d455_fly1/estimator_config_gplane_d195.yaml
config/d455_fly1/estimator_config_gplane_diag.yaml
config/d455_fly1/estimator_config_noslam.yaml
config/d455_fly2/estimator_config_cond1e5.yaml     ← active (1e5 sweep)
config/d455_fly2/estimator_config_cond_1e5*.yaml (8 variants)
config/d455_fly2/estimator_config_cond_1e6_sweep.yaml
config/d455_fly2/estimator_config_cond_3e5.yaml
config/euroc_mav/estimator_config_orb.yaml
config/euroc_mav/estimator_config_xfeat.yaml
```

### Package directories (keep, do not delete)
```
gpsz_on_4flight_fresh_rerun_package/   ← fly1-4 result directories
gpsz_on_yaw_gauge_comparison_package/  ← yaw gauge comparison trajectories
```

---

## Zip/Report Artifacts at Repo Root

| File | Size | Status |
|------|------|--------|
| `three_condition_comparison_package_20260605_1727.zip` | ~184 MB | **PRESERVE** |
| `traj_1e4_fly34_20260606.zip` | ~5 MB | **PRESERVE** |
| `successful_runs_20260606.zip` | ? | **PRESERVE** |
| `gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip` | ? | **PRESERVE** |
| `gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip` | ? | **PRESERVE** |
| `traj_cond1e5_20260606.zip` | ~15 MB | **PRESERVE** |

---

## Failed Experimental Code Paths

| Mode | Enum | Status |
|------|------|--------|
| DSO_INCREMENT_ORTHO | 11 | REMOVED from dispatch (smoke fail: altitude diverge t=450s) |
| VINS_NUMERIC_NULLSPACE | 12 | RETIRED (smoke fail: n_zeroed=0, no nullspace action) |
| CONSTRAINED_YAW_NULLSPACE | 13 | SMOKE FAILED 2026-06-06 (P_n exhausted after 40 calls, covariance indefinite) |
| Arch G GPS-Z | — | REJECTED (292m vs 216m, 113° yaw spike) |
| GPS-Z Joseph partial variants | — | REJECTED |

---

## Candidate Files to Delete (after cleanup)

1. All 100+ `.sh` scripts at repo root → move to `experiments/archive/scripts_YYYYMMDD/`
2. All experimental config variants in `config/d455_fly1/` and `config/d455_fly2/` that
   are not the active baseline → archive
3. `echo` (empty/spurious file at repo root)
4. Files like `"\"\"` (broken filename artifacts)

## Files That Must Not Be Touched

```
config/d455_fly2/estimator_config.yaml        ← active drone config
config/d455_fly2/estimator_config_cond1e5.yaml ← 1e5 sweep config (needed for Stage 8)
config/user_drone_mono_jc82/estimator_config.yaml
ov_msckf/src/core/VioManager.cpp              ← GPS-Z baseline path
ov_msckf/src/state/StateHelper.cpp/.h        ← OC projection logic
ov_msckf/src/run_serial_msckf_ros_free.cpp   ← official runner
eval_stage.py                                 ← eval pipeline
three_condition_comparison_package_20260605_1727.zip
traj_1e4_fly34_20260606.zip
successful_runs_20260606.zip
gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip
gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip
traj_cond1e5_20260606.zip
constrained_yaw_nullspace_smoke_report.md     ← smoke evidence
```
