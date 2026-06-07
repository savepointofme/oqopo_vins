# Artifact Manifest 鈥?2026-06-06

All artifacts that must be preserved for report reproducibility.

---

## Primary Result Packages

### three_condition_comparison_package_20260605_1727.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/three_condition_comparison_package_20260605_1727.zip` |
| **Size** | ~184 MB |
| **Date** | 2026-06-05 17:27 |
| **Contents** | fly1鈥揻ly4 trajectory files for 3 conditions + yaw comparison modes |
| **Flights** | fly1, fly2, fly3, fly4 |
| **Modes** | Cond1 (global_oc no GPS-Z), Cond2 (original+GPS-Z), Cond3 (global_oc+GPS-Z), yawcmp_oc_4d, yawcmp_oc_fej_prechi2 |
| **Config** | `config/d455_fly2/estimator_config.yaml` (fi_max_cond=1e4) |
| **Report use** | Primary three-condition comparison table; fly3 best=Cond2 231m, fly4 Cond3 662m |
| **Required for final report** | YES |

### traj_1e4_fly34_20260606.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/traj_1e4_fly34_20260606.zip` |
| **Date** | 2026-06-06 |
| **Contents** | 4 traj.txt files: fly3_fej, fly3_oc2, fly4_fej, fly4_oc2 |
| **Flights** | fly3, fly4 |
| **Modes** | original_fej (mode0) vs oc_mode2 (mode2), both with guarded GPS-Z ON |
| **Config** | `config/d455_fly2/estimator_config.yaml` (fi_max_cond=1e4) |
| **Eval results** | fly3: OC 323m vs FEJ 416m; fly4: OC 827m vs FEJ 967m |
| **Report use** | OC vs FEJ 1e4 comparison table |
| **Required for final report** | YES |

### successful_runs_20260606.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/successful_runs_20260606.zip` |
| **Date** | 2026-06-06 |
| **Contents** | All successful run trajectories + eval plots + README |
| **Flights** | fly1鈥揻ly4 |
| **Report use** | Comprehensive run package |
| **Required for final report** | YES |

### gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/gpsz_on_4flight_fresh_rerun_package_20260605_1447.zip` |
| **Date** | 2026-06-05 14:47 |
| **Contents** | fly1-4 fresh rerun trajectories with GPS-Z ON |
| **Report use** | Supporting data for GPS-Z analysis |
| **Required for final report** | YES |

### gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/gpsz_on_yaw_gauge_comparison_package_20260605_1447.zip` |
| **Date** | 2026-06-05 14:47 |
| **Contents** | Yaw gauge mode comparison trajectories (GPS-Z ON vs OFF) |
| **Report use** | Supporting yaw comparison analysis |
| **Required for final report** | YES |

### traj_cond1e5_20260606.zip

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/traj_cond1e5_20260606.zip` |
| **Date** | 2026-06-06 |
| **Contents** | fly2 and fly4 1e5 config trajectory comparisons |
| **Flights** | fly2, fly4 |
| **Config** | `config/d455_fly2/estimator_config_cond1e5.yaml` (fi_max_cond=1e5) |
| **Report use** | 1e5 vs 1e4 cond number sensitivity study (Stage 8) |
| **Required for final report** | SUPPORTING (Stage 8 analysis) |

---

## Package Directories (not zipped, in-tree)

### gpsz_on_4flight_fresh_rerun_package/

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/gpsz_on_4flight_fresh_rerun_package/` |
| **Contents** | fly1-4 subdirs with traj.txt, eval outputs, README_manifest.md, summary/ |
| **Required** | YES |

### gpsz_on_yaw_gauge_comparison_package/

| Field | Value |
|-------|-------|
| **Path** | `d:/vscode_dir/open_vins/artifacts/packages/gpsz_on_yaw_gauge_comparison_package/` |
| **Contents** | fly1-4 yaw gauge comparison trajectories |
| **Required** | YES |

---

## Desktop Result Directories (outside repo)

### Fly3 results

| Run | Path |
|-----|------|
| FEJ 1e4 reference | `C:\Users\baloney\Desktop\20260527_gsmq_d455_fly3\result\condgpsz_fly3_original\` |
| OC mode2 1e4 | `C:\Users\baloney\Desktop\20260527_gsmq_d455_fly3\result\gpsz_oc2_1e4_fly3\` |

### Fly4 results

| Run | Path |
|-----|------|
| FEJ 1e4 reference | `C:\Users\baloney\Desktop\20260528_gsmq_d455_fly4\result\condgpsz_fly4_original\` |
| OC mode2 1e4 | `C:\Users\baloney\Desktop\20260528_gsmq_d455_fly4\result\gpsz_oc2_1e4_fly4\` |
| FC rebuild | `C:\Users\baloney\Desktop\20260528_gsmq_d455_fly4\result\fc_rebuild_20260528\` |

---

## Design / Report Documents (repo root, untracked)

| File | Purpose |
|------|---------|
| `constrained_yaw_nullspace_smoke_report.md` | CEKF smoke failure evidence 鈥?preserve |
| `four_route_nullspace_audit.md` | Route taxonomy 鈥?preserve |
| `dso_route_corrected_derivation.md` | DSO audit 鈥?preserve |
| `constrained_ekf_nullspace_design.md` | CEKF design 鈥?preserve |
| `gpsz_on_yaw_gauge_comparison_brief.md` | Brief gauge comparison report |
| `msckf2_ablation_results.md` | MSCKF2 ablation results |
| `gps_z_on_four_flights_baseline_results.md` | GPS-Z four-flight baseline |
| `gps_z_on_four_flights_rerun_results.md` | GPS-Z rerun results |

---

## Active Configuration Files

| File | Purpose | Canonical run |
|------|---------|---------------|
| `config/d455_fly2/estimator_config.yaml` | **Active drone config** (fi=1e4) | All official 1e4 runs |
| `config/d455_fly2/estimator_config_cond1e5.yaml` | 1e5 sweep config | Stage 8 experiments |
