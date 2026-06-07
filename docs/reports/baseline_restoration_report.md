# Baseline Restoration Report
**Date:** 2026-06-04  
**Goal:** Reproduce or explain the old ~216m fly1 XY ATE baseline.  
**Scope:** Verification and reporting only. No new estimator code. No parameter tuning.

---

## Step 1 — Re-evaluation of the old artifact (R0)

**Artifact:** `A1_fcinit_px2_globaloc1p0_traj.txt`  
**Full path:** `/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/A1_fcinit_px2_globaloc1p0_traj.txt`  
**File date:** May 22 2026  
**Traj lines:** 24412 (1 header + 24411 data)  
**Final timestamp:** t=1744.243 s  
**Bias path:** same stem + `.bias`, 24412 lines, same timestamp ✓

**Prior eval (June 3):** eval_stage.py was run with dir-a = dir-b = `eval_baseline_oc_t980_1630/globaloc_oc1p0/` (which contains symlinked copies of the A1 traj/bias). Result stored in `eval_baseline_oc_t980_1630/out_globaloc/metrics_summary.csv`.

**Re-evaluation command (June 3):**
```
python3 eval_stage.py
  --t0 980 --until 1630
  --dir-a eval_baseline_oc_t980_1630/globaloc_oc1p0
  --dir-b eval_baseline_oc_t980_1630/globaloc_oc1p0
  --gps config/d455_fly1/fc_gps_cam_time.csv
  --imu <dataset>/imu0/data.csv
  --out eval_baseline_oc_t980_1630/out_globaloc
```
(dir-a = dir-b = same artifact; A and B results are identical)

**Result from metrics_summary.csv:**

| Metric | Value |
|---|---|
| XY ATE RMS (T0=980, T1=1630) | **216.27 m** |
| XY ATE max | 405.50 m |
| XY ATE P95 | 386.69 m |
| XY ATE final | 396.83 m |
| XY ATE last-330s RMS | 254.82 m |
| Yaw RMS | 8.22° |
| Yaw max | 25.77° |
| Yaw final | 10.64° |
| NaN/Inf | 0 |
| neg_cov | 0 |
| Traj lines | 24411 |
| Final t in eval | 1629.996 s |
| Alignment | start+yaw-only (vel), YAW_WIN=5s |
| GPS file | fc_gps_cam_time.csv |
| IMU file | imu0/data.csv |
| eval_stage.py version | commit `6969c38` (unchanged since June 3) |

**Conclusion for R0:** The old A1 artifact evaluates to **216.27 m** with the current eval_stage.py. The eval script, GPS file, and IMU file have not changed. The 216m result is reproducible from the artifact.

---

## Step 2 — Recovering the exact old run command

**Source:** `A1_fcinit_px2_globaloc1p0_log.txt` (8.1 MB, May 22 2026)

**Recovered flags from log (confirmed from log content):**

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config <d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml>  # inferred (see below)
  --dataset <d455_20260517_174810> \
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 930 \
  --init-from-fc config/d455_fly1/fc_init_state_930.csv \
  --init-att-sigma-deg 3.0 \
  --init-pos-sigma 0.05 \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --vio-yaw-diag <output_dir>/yaw_diag.csv \
  --output <output_dir>/traj.txt \
  [--viz-fast]
  # No --gps-alt-update
  # No --until-time (full flight)
```

**Flags confirmed from log:**
- `att=3.00deg vel=5.00m/s pos=0.05m bg=0.0030rad/s ba=1.000m/s^2` (FC-INIT sigmas line)
- `vio_yaw_update_mode=global_yaw_oc_projection` (CLI override line)
- `vio_global_yaw_oc_alpha=1.000` (CLI override line)
- `fc_init_state_930.csv` (FC init CSV line)
- `fc_gps_cam_time.csv` (GPS line)
- No `GPS-ALT` lines in log → GPS-Z not enabled
- Initial intrinsics: `367.457, 367.140, 328.637, 233.582` → matches `kalibr_imucam_chain.yaml` exactly

**Config file inferred:** The log shows `param T_imu_cam not found, trying T_cam_imu` (a kalibr-chain parse message) and initial intrinsics matching `kalibr_imucam_chain.yaml`. Cross-referencing the A_results_analysis.md description "A1: FC-init + fi_max_dist=500 + sigma_px=2", the only fly1 config with `fi_max_dist=500` and `up_msckf_sigma_px=2.0` referencing `kalibr_imucam_chain.yaml` is:

> `config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml`

The config path is inferred, not directly printed in the log. No committed run script for A1 exists in the repo. The A series runs were executed directly from the shell on May 22.

**Binary used:** The A1 log shows CLI flags (`--vio-yaw-update-mode global_yaw_oc_projection`) that only exist in the current repo's binary (`/mnt/d/vscode_dir/open_vins/build_ov_msckf/`). This was the binary built from the repo at its May 22 state.

**Git commit at time of A1 run (May 22):** Reconstructed from git log with dates. The newest commit before May 22 was:

```
7ecb87c  2026-05-16  baseline: freeze Stage A delay and Stage B v0 diagnostics
```

No commits exist between May 17 and May 24 in the git log. The A1 binary included all commits through May 17, including:
- `7fab7e1 2026-05-11` — gyro-aided KLT (**present in A1 binary**)
- `54846b9 2026-04-24` — GPS-altitude Schmidt fusion code (**present but not enabled**)
- `f41b7f9 2026-04-23` — GPS-Z EKF update code (**present but not enabled**)

The A1 binary did NOT include:
- `2ced00c 2026-05-28` — VisualObservabilityPolicy
- `0736d5c 2026-05-29` — Schmidt yaw update mode
- `d7d11b6 2026-05-29` — guard skip-window
- `76e9b1a 2026-05-29` — GUARDED mode
- Architecture G (`h_offset` state, current session)

Since A1 used `--vio-yaw-update-mode global_yaw_oc_projection` (not Schmidt, not OC policy, not guard), all missing commits are in code paths not exercised by A1. The R1 run (current binary, same mode) should not be affected by these additions.

---

## Step 3 — Old vs current configuration comparison

| Parameter | A1 (216m ref) | baseline_oc_30hz | ablation_gps_off | Notes |
|---|---|---|---|---|
| Config file | `sigma_px_2p0_fcinit_highalt.yaml` (inferred) | `fly2params_cond_1e5.yaml` | `fly2params_cond_1e5.yaml` | **DIFFERENT** |
| kalibr chain | `kalibr_imucam_chain.yaml` (fx=367.45) | `kalibr_imucam_chain.yaml` (fx=367.45) | same | same initial |
| `fi_max_dist` | **500.0** | 2000.0 | 2000.0 | **4× different** |
| `fi_max_baseline` | absent (default) | 1000.0 | 1000.0 | **different** |
| `fi_max_cond_number` | absent (default) | 100000.0 | 100000.0 | **different** |
| `up_msckf_sigma_px` | 2.0 | 2.0 | 2.0 | same |
| `max_clones` | 11 | 11 | 11 | same |
| `max_slam` | 50 | 50 | 50 | same |
| `use_fej` | true | true | true | same |
| `calib_cam_intrinsics` | true | true | true | same |
| `calib_cam_extrinsics` | true | true | true | same |
| `calib_cam_timeoffset` | true | true | true | same |
| `use_gyro_aided_klt` | true (in config) | true | true | same (code present in A1 binary) |
| Start time | 930 | 930 | 930 | same |
| Stop time | none (full flight, t≈1744) | none (t=1938) | none (t=1938) | different (A1 ends earlier) |
| Init file | `fc_init_state_930.csv` | `fc_init_state_930.csv` | `fc_init_state_930.csv` | same |
| `--init-att-sigma-deg` | **3.0°** | **2.0°** | **2.0°** | **DIFFERENT** |
| `--init-pos-sigma` | 0.05 | 0.05 | 0.05 | same |
| `--init-bg-sigma` | 0.003 | 0.003 | 0.003 | same |
| Yaw update mode | `global_yaw_oc_projection` | `global_yaw_oc_projection` | `global_yaw_oc_projection` | same |
| Yaw OC alpha | 1.0 | 1.0 | 1.0 | same |
| GPS-Z altitude update | **disabled** | **enabled** | **disabled** | **DIFFERENT vs baseline_oc_30hz** |
| GPS-Z sigma | N/A | 2.0 m | N/A | N/A |
| GPS-Z guard dxy | N/A | 0.5 m | N/A | N/A |
| GPS-Z guard kxy | N/A | 5.0 | N/A | N/A |
| GPS-Z min-pzz | N/A | 0.01 | N/A | N/A |
| GPS-Z min-t-after-init | N/A | 10 s | N/A | N/A |
| GPS-Z max-res | N/A | 80 m | N/A | N/A |
| Schmidt/OC mode | none (global_oc_projection) | none | none | same |
| B-to-A switch | N/A | N/A | N/A | none in any run |
| Skip-window | N/A | N/A | N/A | none in any run |
| Binary/commit | ~`7ecb87c` (May 17) | `2538839` (May 30, Jun 4 build) | `2538839` | **DIFFERENT** |

**Summary of confirmed differences between A1 and current runs:**

1. **Config file** — `sigma_px_2p0_fcinit_highalt` (fi_max_dist=500) vs `fly2params_cond_1e5` (fi_max_dist=2000, fi_max_baseline=1000, fi_max_cond_number=1e5)
2. **att-sigma** — 3.0° vs 2.0°
3. **GPS-Z** — disabled in A1, enabled in baseline_oc_30hz
4. **Binary** — May 17 state vs current (May 30) build; the added code in May 28-30 is in non-exercised paths under `global_yaw_oc_projection`
5. **Flight duration** — A1 stops at t≈1744 (landing divergence), current runs continue to t=1938

---

## Step 4 — Reproduction matrix

| Run | Binary/Commit | Config | Key CLI diff from A1 | GPS-Z | XY RMS | Yaw RMS | Reproduces 216m? |
|---|---|---|---|---|---|---|---|
| **R0** — old artifact, current eval | A1 binary (May 22) | `sigma_px_2p0_fcinit_highalt` (inferred) | — | off | **216.27 m** | 8.22° | ✅ Eval confirmed |
| **R1** — current binary, old config | `2538839` (Jun 4 17:03) | `sigma_px_2p0_fcinit_highalt` | att=3.0°, no GPS-Z | off | **216.27 m** | 8.22° | ✅ Yes — exact match |
| **R2** — current binary, current config | `2538839` (Jun 4 17:03) | `fly2params_cond_1e5` | att=2.0° | on | **375.49 m** | 7.31° | ❌ No (+159m) |
| **R2b** — current binary, current config, GPS-Z off | `2538839` (Jun 4 17:03) | `fly2params_cond_1e5` | att=2.0° | off | **298.53 m** | 9.76° | ❌ No (+82m) |
| **R3** — old binary + old config | Not available | — | — | — | — | — | ⚠️ Not feasible |
| **R4** — old binary + current config | Not available | — | — | — | — | — | ⚠️ Not feasible |

**Notes on R3/R4:** The May 22 binary is not available. The `da3_openvins` binary at `/mnt/d/工作_新/.../build_gp/` was built Jun 2 (not May 22) and is a different codebase (`build_gp` = ground-plane build). No binary snapshot from May 22 was preserved.

**R1 result (complete):**
- XY ATE RMS: **216.27 m** — exact match to A1 (to 5 significant figures)
- Yaw RMS: 8.22°, max 25.77° — exact match
- Alignment delta_yaw: 8.500° — exact match to A1 canonical value
- NaN/Inf: 0, neg_cov: 0
- Traj lines: 30223 (full flight, t=1938.059)
- Output: `result/R1_baseline_restore/`, eval: `result/eval_R1_baseline_restore/`

---

## Step 5 — What caused the baseline loss

Based on the facts gathered, the following answers are given explicitly for each question:

**Was the 216m caused by a different config?**  
Yes, partially. A1 used `sigma_px_2p0_fcinit_highalt.yaml` (fi_max_dist=500) while the current runs use `fly2params_cond_1e5.yaml` (fi_max_dist=2000, fi_max_baseline=1000). These configs have different feature initialization gates. Whether this alone accounts for the 82m gap between R2b (GPS-Z off, 298m) and 216m is unknown until R1 completes.

**Was it caused by different command-line flags?**  
Yes. `att-sigma=3.0°` in A1 vs `att-sigma=2.0°` in current runs. A larger initial attitude uncertainty gives the EKF more freedom to correct attitude during early flight, which may benefit yaw convergence. GPS-Z disabled in A1 vs enabled in baseline_oc_30hz is the dominant difference between R2 (375m) and A1 (216m).

**Was it caused by `att-sigma`?**  
Unknown in isolation. Both `sigma` (config file) and `att-sigma` differ between A1 and R2b. R1 isolates `att-sigma=3.0°` and old config together. The contribution of each individually is not yet measured.

**Was it caused by GPS-Z flags?**  
Yes — for the gap between baseline_oc_30hz (375m) and ablation_gps_off (298m). GPS-Z fusion causes intrinsic calibration to drift from fx=367 to fx≈385 during the run, changing the measurement model for all subsequent visual updates. This is a 77m (20%) XY degradation specifically attributable to GPS-Z altitude fusion.

**Was it caused by yaw mode / Schmidt / globaloc settings?**  
No. All three runs use `global_yaw_oc_projection` alpha=1.0. Schmidt and OC policy modes were not enabled.

**Was it caused by a later code regression?**  
No. R1 (current binary `2538839`, old config, old flags) produces **216.27m** — identical to A1. The binary changes between May 17 and May 30 (Schmidt, OC policy, guard modes) are all in code paths not exercised by `global_yaw_oc_projection`. No regression exists in the exercised path.

**Was `baseline_oc_30hz` mislabeled or overwritten?**  
No. `baseline_oc_30hz` is a correctly labeled run with its own script (`run_fly1_baseline_oc_30hz.sh`), using `fly2params_cond_1e5.yaml` with GPS-Z enabled. It is not the same run as A1 and was never intended to match 216m. The name "baseline_oc_30hz" refers to a more recent experiment series, not the original 216m baseline.

**Is the current `fly2params_cond_1e5.yaml` actually the old config?**  
No. `fly2params_cond_1e5.yaml` is explicitly described in its header as:
> "Base: estimator_config_sigma_px_2p0_fcinit_highalt.yaml — Keep fly1 calibration / pixel noise / FC-init assumptions, and only port the high-altitude feature initializer gates used by the fly2 recommended line."
It was created later to generalize fly2 parameters to fly1. The old config is `sigma_px_2p0_fcinit_highalt.yaml`.

**Summary of cause cascade:**

```
A1 (216m)
  ↓ change: fi_max_dist 500→2000, fi_max_baseline added, att-sigma 3.0°→2.0°
ablation_gps_off (298m)      [current binary, fly2params config, no GPS-Z]
  ↓ change: GPS-Z altitude fusion enabled
baseline_oc_30hz (375m)      [current binary, fly2params config, GPS-Z on]
```

The 82m gap (ablation_gps_off vs A1) is attributable entirely to config/flag differences (fi_max_dist 500→2000, att-sigma 3.0°→2.0°). R1 confirms there is no code regression — current binary reproduces 216m with old config and flags.

The 77m gap (baseline_oc_30hz vs ablation_gps_off) is attributable to GPS-Z altitude fusion, which causes online intrinsic calibration to drift from fx=367 to fx≈385.

---

## Step 6 — Architecture G status freeze

Architecture G evaluation is frozen pending baseline restoration.

Current status:
```
Architecture G is frozen. The evaluation baseline has drifted from the old ~216m
result to current 298–375m results. Before evaluating Architecture G, the old
baseline must be reproduced or explained. The R1 result (current binary + old config
+ att=3.0° + no GPS-Z) will determine whether the 216m is reproducible with the
current binary.

Until R1 is evaluated: do not compare Architecture G against any current run result.
Do not claim Architecture G is validated or rejected.
```

---

## Appendix — R1 run details

**Script:** `run_fly1_R1_baseline_restore.sh`  
**Output dir:** `result/R1_baseline_restore/`  
**Config:** `estimator_config_sigma_px_2p0_fcinit_highalt.yaml`  
**Binary:** `build_ov_msckf/run_serial_msckf_ros_free` (commit `2538839`, built Jun 4 17:03)  
**Flags:** `--init-att-sigma-deg 3.0 --vio-yaw-update-mode global_yaw_oc_projection --vio-global-yaw-oc-alpha 1.0` (no `--gps-alt-update`)  
**Status at time of report:** In progress at t≈1093. Eval window T0=980–T1=1630 not yet covered.  
**Expected eval command (after completion):**
```bash
python3 eval_stage.py \
  --t0 980 --until 1630 \
  --dir-a result/R1_baseline_restore \
  --dir-b result/R1_baseline_restore \
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --imu <dataset>/imu0/data.csv \
  --out result/eval_R1_baseline_restore
```
