# Feature Gate Config Diff: Fly2 Success vs Failure

**Date:** 2026-06-07
**Purpose:** Isolate which feature-initialization parameters changed between previous successful Fly2 runs and current failed runs.

---

## 1. YAML Version History

The `config/d455_fly2/estimator_config.yaml` has only **two committed versions**:

| Commit | Date | fi_max_cond_number | fi_max_dist | fi_max_baseline | up_msckf_sigma_px |
|--------|------|--------------------|-------------|-----------------|-------------------|
| `3e6915c` | 2026-05-24 | **10000.0 (1e4)** | 2000.0 | 1000.0 | 2.0 |
| `e52d99c` | 2026-06-07 | **10000.0 (1e4)** | 2000.0 | 1000.0 | 2.0 |

**Finding: fi_max_dist did NOT change.** It has been 2000.0 in all committed YAML versions. The user's hypothesis about fi_max_dist changing is not supported by git history.

Code default (FeatureInitializerOptions.h): `max_dist=60`, `max_baseline=40`, `max_cond_number=10000`. The YAML overrides all three.

---

## 2. Run Comparison Table

| Parameter | cond_sweep/run_cond_1e5 | condgpsz_fly2_original | gpsz_on_rerun | gpsz_fej_cond1e5 | 20260606_openvins_fej | 20260606_official_oc |
|-----------|------------------------|------------------------|---------------|------------------|----------------------|---------------------|
| **fi_max_cond_number** | **100000 (1e5)** | 10000 (1e4) | 10000 (1e4) | **unknown (1e5?)** | 10000 (1e4) | 10000 (1e4) |
| **fi_max_dist** | 2000.0 | 2000.0 | 2000.0 | 2000.0 | 2000.0 | 2000.0 |
| **fi_max_baseline** | 1000.0 | 1000.0 | 1000.0 | 1000.0 | 1000.0 | 1000.0 |
| **init_bg_sigma** | 0.003 | **0.003** | **0.003** | 0.050 | 0.050 | 0.050 |
| **up_msckf_sigma_px** | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 | 2.0 |
| **up_msckf_chi2_mult** | 1 | 1 | 1 | 1 | 1 | 1 |
| **use_fej** | true | true | true | true | true | true |
| **GPS file** | offset451 | offset447p5 | offset447p5 | offset447p5 | offset447p5 | offset447p5 |
| **FC init file** | offset451 | offset447p5 | offset447p5 | offset447p5 | offset447p5 | offset447p5 |
| **Gyro-aided KLT** | ENABLED (bg<gate) | **ENABLED** | **ENABLED** | DISABLED | DISABLED | DISABLED |
| **Yaw mode** | global_yaw_oc_projection | original (openvins_fej) | global_yaw_oc_projection | unknown (fej) | openvins_fej | oc_postchi2_current_gauge |
| **Binary commit** | pre-3e6915c (old) | ~2538839 | ~2538839 | pre-e52d99c | **e52d99c** | **e52d99c** |
| **Run date** | ~May 2026 | Jun 5 16:37 | Jun 5 14:14 | Jun 6 16:12 | Jun 6-7 | Jun 6-7 |
| **Result** | COMPLETE | COMPLETE (305 m) | COMPLETE (~5 km XY) | COMPLETE (4.6 km XY) | **DIVERGED (3536 km)** | **DIVERGED (1867 km)** |

**Evidence source for each run:**
- `cond_sweep/run_cond_1e5`: YAML file (`estimator_config_cond_1e5.yaml`) + log.txt line 18 (bg=0.003)
- `condgpsz_fly2_original`: log.txt line 24 (bg=0.003); YAML from git commit
- `gpsz_on_rerun_fly2_global_oc_guarded`: log.txt line 26 (bg=0.003); YAML from git commit
- `gpsz_fej_cond1e5_fly2`: log.txt line 21 (bg=0.050); **no command.txt** — fi_max_cond_number is inferred from directory name only (unconfirmed)
- `20260606` runs: command.txt + git_commit.txt

---

## 3. What Actually Changed

### Factor 1: fi_max_cond_number (partially confirmed)

A previous cond sweep (`cond_sweep_start700_oc_gpsref/`) explicitly tested fi_max_cond_number values of 1e5, 3e5, and 1e6 using separate YAML files. The current failed runs use fi_max_cond_number=10000 (1e4) from the standard YAML. The `gpsz_fej_cond1e5_fly2` directory name implies it used fi=1e5, but no command.txt survives to confirm.

**Key note from cond_sweep YAML comment:**
> "straight-line flight → near-zero perpendicular parallax → condA>>10000 → all DLT triangulations fail before chi2 even runs"

This is the original author's hypothesis for why fi=1e4 fails on Fly2.

### Factor 2: init_bg_sigma (confirmed)

| Run group | init_bg_sigma | Source |
|-----------|--------------|--------|
| All Jun 5 successes | **0.003 rad/s** | log.txt lines |
| gpsz_fej_cond1e5 (Jun 6, old binary) | 0.050 rad/s | log.txt line 21 |
| All Jun 6-7 failures | **0.050 rad/s** | log.txt lines |

The `--init-bg-sigma` CLI flag defaults to 0.050 in the source (`run_serial_msckf_ros_free.cpp:135`). The Jun 5 successful runs overrode this to 0.003. The current failed runs did not include `--init-bg-sigma` and used the default.

**Effect:** When `init_bg_sigma > use_gyro_aided_klt_max_bg_sigma` (gate = 0.005 rad/s), gyro-aided KLT is **disabled** until the covariance shrinks below the gate. With bg_sigma=0.050, gyro-aided KLT is disabled for the early flight when position uncertainty is highest.

### Factor 3: fi_max_dist — NO CHANGE (user hypothesis not supported)

fi_max_dist was **always 2000.0** in all YAML versions (both committed and experiment YAMLs). This parameter is not a confound.

---

## 4. Revised Root Cause Hypothesis

The original report stated: "Root cause: fi1e4 is too restrictive." This was incorrect framing.

**Revised:** The Fly2 failure under the current setup is caused by at least one (possibly both) of:

1. **fi_max_cond_number=1e4**: On Fly2's high-speed straight-line segments, DLT conditioning is poor (near-zero perpendicular parallax). Raising from 1e4 to 1e5 allows these features through. This was the diagnosed issue in the original cond_sweep YAML comment.

2. **init_bg_sigma=0.050 (gyro-aided KLT disabled)**: Without gyro rotation prediction, KLT tracking fails more often under fast motion (39.1 m/s). The Jun 5 successful runs all used bg_sigma=0.003. The Jun 6 failure runs all used bg_sigma=0.050.

These two factors may interact: either alone might be survivable, but both together cause feature starvation followed by EKF divergence.

---

## 5. Corrected Isolation Matrix

The user proposed a 2×2 with fi_max_cond_number × fi_max_dist. Since fi_max_dist did not change, the correct 2×2 is:

**Factor A: fi_max_cond_number** — `fi_max_cond_number` in YAML  
**Factor B: init_bg_sigma** — CLI flag `--init-bg-sigma`

| | fi_max_cond_number = 1e4 (current YAML) | fi_max_cond_number = 1e5 (experiment YAML) |
|---|---|---|
| **bg_sigma = 0.050** (default, gyro-KLT off) | **Run A**: DONE — DIVERGED | **Run B**: new run needed |
| **bg_sigma = 0.003** (Jun5 value, gyro-KLT on) | **Run C**: new run needed | **Run D**: new run needed |

Run A is already done (both yaw modes diverged). Runs B, C, D are the minimal set needed for isolation.

Each run should use: `official_oc`, GPS-Z ON, same GPS/FC init/config, `start_yaw` eval.

**For Run B (fi=1e5, bg=0.050):** Use YAML from `result/fc_rebuild_20260525/cond_sweep_start700_oc_gpsref/estimator_config_cond_1e5.yaml` (must be accessible to binary), or create new `config/d455_fly2/estimator_config_fi1e5.yaml`.

---

## 6. Known Previous Successful Reference

The `cond_sweep/run_cond_1e5` run completed with fi=1e5, bg=0.003, and a slightly different GPS offset (offset451 vs offset447p5). It is the closest documented success to the current failure. Its traj/eval is in `result/fc_rebuild_20260525/cond_sweep_start700_oc_gpsref/run_cond_1e5/`.

---

## 7. Parameters That Are the Same in All Runs (Not Confounds)

| Parameter | Value | Confirmed same? |
|-----------|-------|-----------------|
| fi_max_dist | 2000.0 | Yes (all YAMLs, all commits) |
| fi_max_baseline | 1000.0 | Yes (all YAMLs) |
| up_msckf_sigma_px | 2.0 | Yes (d455_fly2 config) |
| up_msckf_chi2_multipler | 1 | Yes |
| use_fej | true | Yes |
| GPS-Z flags | same gates | Yes (command.txt) |
| FC init file | offset447p5 | Yes (for all Jun 5-7 runs) |
| Start time | 700 | Yes |
| GPS file | offset447p5_cam_time.csv | Yes (for all Jun 5-7 runs) |

---

*Report written: 2026-06-07. Based on: config git history, log.txt FC init lines, run directory listing in Desktop/20260518_gsmq_d455_fly2/result/, source code FeatureInitializerOptions.h and run_serial_msckf_ros_free.cpp.*
