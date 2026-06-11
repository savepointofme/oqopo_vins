# Fly2: official_oc vs openvins_fej — fi1e4 + GPS-Z

**Date:** 2026-06-07
**Git commit:** `e52d99c` (same binary that passed fly3 regression)
**Verdict: BOTH DIVERGED — fi1e4 is insufficient for Fly2**

---

## 1. Setup

| Parameter | Value |
|-----------|-------|
| Config | `config/d455_fly2/estimator_config.yaml` |
| fi_max_cond_number | 10000.0 (fi1e4) |
| Dataset | `d455_20260517_184722` |
| GPS | `gps_from_mems_offset447p5_cam_time.csv` |
| FC init | `canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv` |
| Start | 700 |
| GPS time offset | 0 (CLI override) |
| GPS-Z flags | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-max-res 80 --gps-alt-min-t-after-init 10 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| Eval window | T0=700, until=2445, yaw-align-mode=start_yaw |

## 2. Commands

**openvins_fej:**
```
run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config.yaml \
  --dataset .../d455_20260517_184722 \
  --gps .../gps_from_mems_offset447p5_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 700 \
  --init-from-fc .../canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv \
  --vio-yaw-gauge-mode openvins_fej \
  --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-max-res 80 --gps-alt-min-t-after-init 10 \
  --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5
```

**official_oc:** same with `--vio-yaw-gauge-mode oc_postchi2_current_gauge`

## 3. Output Folders

| Mode | Folder | Status |
|------|--------|--------|
| openvins_fej | `result/20260606_fly2_openvins_fej_gpsz_fi1e4/` | STATUS_DIVERGED |
| official_oc | `result/20260606_fly2_official_oc_gpsz_fi1e4/` | STATUS_DIVERGED |

## 4. Divergence Timeline

Both runs completed with shell exit code 0 (from `tee`/`tail` pipeline) but the binary did not print the `[ros-free] done.` summary, indicating crash or forced termination after state divergence.

| Event | openvins_fej (A) | official_oc (B) |
|-------|-----------------|-----------------|
| VIO init | t=700 | t=700 |
| FC init velocity | 39.1 m/s | 39.1 m/s |
| First GPS-Z accept | t=710 | t=710 |
| Last GPS-Z accept | **t=887** | **t=862** |
| GPS-Z accepts total | 716 | 742 |
| GPS-Z rejects total | 6907 | 6355 |
| XY traj > 10 km | t=1141 | **t=1011** |
| XY traj > 100 km | t=1300 | t=1348 |
| traj.txt last entry | t=2445 | t=2335 |
| traj lines | 52323 | 49041 |
| Neg-cov warnings (-0.00) | 3 | 1 |

**Notable:** official_oc diverged on XY 130s EARLIER than openvins_fej, despite having a slightly longer GPS-Z acceptance window (742 accepts vs 716). This is the reverse of what was observed on fly3, where official_oc was ~27% better.

## 5. Metrics Table

*Full-window metrics are dominated by diverged state and are NOT comparable to fly3/fly4. Early segment is the only partially valid window.*

| Mode | Full RMS (m) | Early RMS (m) | Late RMS (m) | Yaw RMS (°) | Yaw max (°) | Final XY dist |
|------|-------------|---------------|--------------|------------|------------|---------------|
| openvins_fej | **1,339,186** | **218** | 1,585,368 | 13.5 | 111 | 3,443 km |
| official_oc | **721,264** | **315** | 865,527 | 56.3 | 180 | 1,820 km |

> "full RMS" and "late RMS" are not meaningful metrics here — both are entirely driven by the diverged EKF state carrying a position error of 1000s of km. **Do not compare these numbers to Fly1/Fly3/Fly4 results.**

## 6. GPS-Z Behavior

GPS-Z was accepted successfully for the first ~160-187 seconds (t=710 to t=862-887). Both runs bootstrapped correctly:
- gps_ref altitude: 209.47 m (AGL reference)
- vio_ref altitude: 0.48 m

GPS-Z rejected from t≈862-887 onward because the altitude residual exceeded the 80 m gate. This rejection is a symptom of VIO drift, not a cause of it — by the time GPS-Z was rejected, the EKF altitude had already drifted >80m due to VIO failure.

## 7. Leading Hypothesis: Feature Gate Combination

**Do not attribute the failure to fi_max_cond_number alone.** Config diff analysis (see `docs/reports/feature_gate_config_diff_20260607.md`) identified two candidate factors:

**Factor 1 — fi_max_cond_number (1e4 vs 1e5):** Previous cond_sweep experiments confirmed that Fly2's high-speed straight-line segments produce near-zero perpendicular parallax, driving DLT condition numbers well above 10000. With fi=1e4, all such features are rejected before chi2. A separate experiment YAML (`estimator_config_cond_1e5.yaml`) with fi=1e5 was used in earlier successful runs.

**Factor 2 — init_bg_sigma (0.003 vs 0.050):** The June 5 successful runs all used `--init-bg-sigma 0.003`. The June 6-7 failed runs used the default 0.050. When bg_sigma > 0.005 rad/s (the gyro-aided KLT gate), gyro-aided KLT is disabled at startup. Without rotation prediction at 39.1 m/s, more features are lost between frames, compounding the condition-number starvation.

**Attribution requires a controlled 2-factor isolation.** The current failure is under fi=1e4 + bg=0.050 (gyro-KLT off). Prior successful runs used fi=1e4 + bg=0.003 (Jun 5) or fi=1e5 + bg=0.003 (cond_sweep). The combination fi=1e5 + bg=0.050 is untested.

**Note:** fi_max_dist did NOT change — it has been 2000.0 in all YAML versions. This parameter is not a confound.

## 8. Comparison with Fly3/Fly4 Evidence

| Flight | Mode | Full RMS | Status | fi setting |
|--------|------|----------|--------|-----------|
| Fly3 | openvins_fej | 447.81 m | COMPLETE | fi1e4 |
| Fly3 | official_oc | 323.48 m | COMPLETE | fi1e4 |
| Fly4 (from archive) | official_oc | ~180 m (est.) | COMPLETE | fi1e4 |
| **Fly2** | openvins_fej | **DIVERGED** | DIVERGED | fi1e4 |
| **Fly2** | official_oc | **DIVERGED** | DIVERGED | fi1e4 |
| Fly2 (prior run) | official_oc | max 4.68 km XY | COMPLETE | **unknown (possible fi1e5)** |

## 9. Answer to the Decision Rule

> Does official_oc remain more stable than openvins_fej on Fly2 under the cleaned 1e4 setup?

**Answer: Undetermined — both diverged.**

- Both modes failed under fi1e4 on Fly2.
- official_oc diverged on XY earlier (t=1011 vs t=1141 for 10km threshold).
- openvins_fej showed better yaw stability through the diverged period (Yaw RMS 13.5° vs 56.3°).
- Neither result is comparable to the fly3 evidence where official_oc was clearly better.

## 10. Next Steps

Per protocol: **do not proceed to isolation experiments** until this result and the config diff have been reviewed.

The config diff analysis (`docs/reports/feature_gate_config_diff_20260607.md`) proposes a corrected 2×2 isolation:

| | fi_max_cond_number = 1e4 | fi_max_cond_number = 1e5 |
|---|---|---|
| **bg_sigma = 0.050** (gyro-KLT off) | **Run A**: DONE — DIVERGED | Run B: to test |
| **bg_sigma = 0.003** (gyro-KLT on) | Run C: to test | Run D: to test |

All matrix runs: `official_oc`, GPS-Z ON, same GPS/FC init, `start_yaw` eval.

**Do not re-run, do not tune, do not patch.** The runs are logged with STATUS_DIVERGED; the evidence is preserved.

---

*Report written: 2026-06-07. Runs performed at commit `e52d99c`. Eval metrics in `Desktop/20260518_gsmq_d455_fly2/result/eval_fly2_1e4_20260606/metrics_summary.csv`.*
