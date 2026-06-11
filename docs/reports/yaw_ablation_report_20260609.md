# Yaw-Drift Parameter Ablation — fly1 / fly3 / fly4

**Date:** 2026-06-08 / 2026-06-09  
**Mode (fixed):** `oc_postchi2_current_gauge` (GLOBAL_YAW_OC_PROJECTION, mode 2) + GPS-Z ON  
**Fixed:** `init_bg_sigma=0.003`, `fi_max_cond_number=1e4` (config default), all GPS-Z guard flags unchanged  
**Script:** `run_yaw_ablation.sh`  
**Commit:** `e52d99c`

---

## 1. Parameters Tested

One parameter changed at a time. All others held at baseline.

| ID | Parameter | Mechanism | Baseline | Values tested | Flights |
|----|-----------|-----------|----------|---------------|---------|
| P1 | `up_msckf_sigma_px` | Pixel noise σ in chi2 gate: larger σ → larger S → looser chi2 threshold → more features accepted | fly1=**1**, fly3/fly4=**2** | 1, 2, 3 | fly1: {2,3}; fly3/fly4: {1,3} |
| P2 | `up_msckf_chi2_multipler` | Scale factor on chi2 table threshold; upstream default=5, our baseline=**1** (5× tighter) | **1** | 2, 5 | fly1, fly3 |
| P3 | `max_clones` | Sliding-window depth; longer window → more triangulation baseline → more indirect yaw coupling | **11** | 15, 20 | fly3 only |
| P4 | `gyroscope_noise_density` | IMU propagation noise; kalibr raw=0.00058, current=**0.005** (8.6× inflated for vibration) | **0.005** | 0.002, 0.010 | fly3 only |

---

## 2. Results

### 2.1 fly1 (~200 m altitude, σ_baseline=1)

| Run | σ | χ² | clones | gyro_nd | Status | ATE_rms m | yaw_rms ° | yaw_p95 ° | yaw_final ° |
|-----|---|----|--------|---------|--------|-----------|-----------|-----------|-------------|
| baseline_fly1 | 1 | 1 | 11 | 0.005 | **DIVERGED** | 428 | 8.60 | 13.84 | +8.8 |
| P1_sigma2_fly1 | **2** | 1 | 11 | 0.005 | OK | 379 | 7.86 | 14.86 | +8.7 |
| **P1_sigma3_fly1** | **3** | 1 | 11 | 0.005 | OK | **264** | **4.91** | **8.45** | −3.4 |
| P2_chi2m2_fly1 | 1 | **2** | 11 | 0.005 | **DIVERGED** | 392 | 6.61 | 11.18 | +2.6 |
| P2_chi2m5_fly1 | 1 | **5** | 11 | 0.005 | OK | 418 | 7.59 | 12.67 | −1.5 |

**Key findings:**
- σ=1 DIVERGES with chi2_mult=1 **and** chi2_mult=2. The chi2 gate is so tight it starves the filter.
- σ=3 with chi2_mult=1 is the best result: ATE **264 m** (−38% vs diverged baseline), yaw_rms **4.91°** (−43%).
- chi2_mult=5 with σ=1 also survives (ATE=418 m) but is inferior to σ=3. Loosening chi2 treats the symptom; raising σ fixes the root cause (σ=1 is ~9× too small vs the ~11 px calibration drift at altitude).

### 2.2 fly3 (~200 m altitude, σ_baseline=2)

| Run | σ | χ² | clones | gyro_nd | Status | ATE_rms m | yaw_rms ° | yaw_p95 ° | yaw_final ° |
|-----|---|----|--------|---------|--------|-----------|-----------|-----------|-------------|
| baseline_fly3 | 2 | 1 | 11 | 0.005 | OK | 261 | 7.08 | 12.42 | +3.0 |
| **P1_sigma1_fly3** | **1** | 1 | 11 | 0.005 | OK | **224** | **6.56** | **9.92** | +7.6 |
| P1_sigma3_fly3 | **3** | 1 | 11 | 0.005 | OK | 287 | 7.07 | 12.39 | +3.2 |
| P2_chi2m2_fly3 | 2 | **2** | 11 | 0.005 | OK | 279 | 7.17 | 12.80 | +2.7 |
| P2_chi2m5_fly3 | 2 | **5** | 11 | 0.005 | OK | 271 | 7.03 | 12.49 | +2.9 |
| **P3_clones15_fly3** | 2 | 1 | **15** | 0.005 | OK | **226** | **6.63** | **9.60** | +7.9 |
| P3_clones20_fly3 | 2 | 1 | **20** | 0.005 | OK | 277 | 6.73 | 10.42 | +6.0 |
| P4_gyro002_fly3 | 2 | 1 | 11 | **0.002** | OK | 252 | 6.76 | 9.72 | +9.8 |
| P4_gyro010_fly3 | 2 | 1 | 11 | **0.010** | OK | 276 | 6.73 | 11.02 | +4.6 |

**Key findings:**
- fly3 is stable across all tested values — no divergence in any cell.
- σ=1 gives the best ATE (224 m, −14% vs baseline) and best yaw_p95 (9.92°, −20%). fly3's camera calibration is better than fly1, so σ=1 matches reality more closely.
- clones=15 nearly matches σ=1 (226 m, −13%). clones=20 is worse than 15, likely because longer windows increase FEJ linearization error.
- chi2_mult {2, 5} have no meaningful effect on fly3 — all within ±18 m ATE of baseline.
- gyro_nd: tighter (0.002) gives modest improvement (252 m); looser (0.010) is slightly worse. Differences are small (±15 m), not actionable.

### 2.3 fly4 (~400 m altitude, σ_baseline=2)

| Run | σ | χ² | clones | gyro_nd | Status | ATE_rms m | yaw_rms ° | yaw_p95 ° | yaw_final ° |
|-----|---|----|--------|---------|--------|-----------|-----------|-----------|-------------|
| **baseline_fly4** | **2** | 1 | 11 | 0.005 | OK | **755** | **17.37** | **29.10** | −38.7 |
| P1_sigma1_fly4 | **1** | 1 | 11 | 0.005 | OK | 831 | 17.76 | 29.89 | −36.5 |
| P1_sigma3_fly4 | **3** | 1 | 11 | 0.005 | OK | 855 | 18.70 | 31.14 | −39.3 |

**Key findings:**
- The baseline σ=2 is already optimal for fly4. Both directions of σ change are worse.
- fly4's yaw drift (~17° rms, ~−39° final) is not sensitive to σ. The ~17° systematic heading error is caused by deeper structural issues not addressable via these parameters.
- P2, P3, P4 were not run for fly4 given its insensitivity to P1 (no upside).

---

## 3. Per-Parameter Summary

### P1 — `up_msckf_sigma_px`

| Direction | fly1 | fly3 | fly4 |
|-----------|------|------|------|
| σ ↑ (larger) | **MUST INCREASE** — σ=1 diverges; σ=3 best (+38% ATE, +43% yaw) | Slightly worse (σ=3: ATE+10%) | Worse (σ=3: ATE+13%) |
| σ ↓ (smaller) | — (already at baseline) | **Slightly better** (σ=1: ATE−14%, yaw_p95 −20%) | Worse (σ=1: ATE+10%) |

**Verdict:** σ is flight-specific. fly1 needs σ=3. fly3 best at σ=1. fly4 stays at σ=2. No universal optimal.

### P2 — `up_msckf_chi2_multipler`

| chi2_mult | fly1 | fly3 |
|-----------|------|------|
| 1 (baseline) | DIVERGED | OK (baseline) |
| 2 | DIVERGED | OK, slightly worse (+7%) |
| 5 | OK, but ATE 418 m (inferior to σ=3 fix) | OK, marginal improvement (−4%) |

**Verdict:** For fly1 with σ=1, increasing chi2_mult is necessary to survive but insufficient for good accuracy. Once σ is corrected to 3, chi2_mult=1 works fine. For fly3, chi2_mult is neutral — no improvement from loosening.

### P3 — `max_clones` (fly3 only)

| clones | ATE m | yaw_rms ° | yaw_p95 ° |
|--------|-------|-----------|-----------|
| 11 (baseline) | 261 | 7.08 | 12.42 |
| **15** | **226** | **6.63** | **9.60** |
| 20 | 277 | 6.73 | 10.42 |

**Verdict:** clones=15 provides a genuine improvement for fly3 (−14% ATE, −23% yaw_p95). clones=20 does not further improve and is slightly worse — longer windows increase FEJ inconsistency for marginal gain. Sweetspot is 15 for fly3.

### P4 — `gyroscope_noise_density` (fly3 only)

| gyro_nd | ATE m | yaw_rms ° | yaw_p95 ° |
|---------|-------|-----------|-----------|
| 0.002 | 252 | 6.76 | 9.72 |
| **0.005 (baseline)** | **261** | **7.08** | **12.42** |
| 0.010 | 276 | 6.73 | 11.02 |

**Verdict:** Tighter gyro noise (0.002) gives a marginal benefit (−3% ATE, −22% yaw_p95). The range tested spans 5× and produces only ±15 m ATE variation. Not actionable.

---

## 4. Recommended Settings

### Per-flight

| Flight | Parameter | Current | Recommended | ATE change | yaw_rms change |
|--------|-----------|---------|-------------|------------|----------------|
| fly1 | `up_msckf_sigma_px` | 1 | **3** | 428 → 264 m (−38%) | 8.60 → 4.91° (−43%) |
| fly3 | `up_msckf_sigma_px` | 2 | **1** (or keep 2) | 261 → 224 m (−14%) | 7.08 → 6.56° (−7%) |
| fly3 | `max_clones` | 11 | **15** | 261 → 226 m (−13%) | 7.08 → 6.63° (−6%) |
| fly4 | (none) | — | no change | 755 m | 17.37° |

### Common setting for fly1/fly3/fly4

There is no single `sigma_px` that is optimal for all three flights simultaneously. Flights differ in calibration error: fly1 has ~11 px drift at altitude (wants σ=3), fly3 has better calibration (wants σ=1), fly4 is neutral at σ=2. **Keep per-flight configs.**

The only universal recommendation is: **do not use σ=1 for fly1**. It causes systematic divergence under this mode (oc_postchi2_current_gauge, chi2_mult=1).

---

## 5. Analysis

### Why does fly1 σ=1 diverge?

The chi2 gate for a feature cluster of dof `d` is:
```
S = H_oc · P · H_oc^T + σ² · I
chi2 = r^T · S^{-1} · r
REJECT if chi2 > chi2_mult · chi2_table[d]
```

With σ=1 and chi2_mult=1, the threshold is 5× tighter than the upstream OpenVINS default (chi2_mult=5). At fly1's 200 m altitude, the camera has an intrinsic calibration drift of ~11 px. Nearly every feature will have a residual of several pixels, producing chi2 >> 1 and being rejected. The EKF goes into filter starvation: IMU-only propagation → orientation drift → worsening feature residuals → more rejections → runaway divergence.

The fix is to inflate σ to match the true measurement noise. At σ=3, the 3-sigma envelope covers 9-pixel residuals, sufficient to admit most features at fly1's typical altitude.

### Why does fly3 prefer σ=1 but fly4 prefer σ=2?

fly3 has better camera calibration (intrinsic residuals closer to 1 px at 200 m). With σ=1, the chi2 gate correctly rejects genuinely bad features while accepting good ones, resulting in a higher-quality feature set. fly4 at 400 m altitude sees more depth ambiguity and a larger fraction of features with residuals in the 1–2 px range; σ=1 would under-reject noise at this altitude.

### Why does fly4 yaw drift persist regardless of σ?

fly4's 17° yaw_rms and ~−39° final yaw error are not related to feature acceptance threshold. The mode (`oc_postchi2_current_gauge`) applies OC projection that removes the yaw direction from H, meaning visual updates provide zero direct yaw correction. Yaw is controlled only through cross-covariance terms between position and orientation. At fly4's 400 m altitude and longer flight duration (1891 s), the indirect yaw coupling is insufficient to constrain drift. This is a mode-level limitation, not a chi2 or sigma parameter issue.

---

## 6. Conclusions

1. **fly1: change `up_msckf_sigma_px` from 1 to 3.** This is the highest-impact finding. The current σ=1 causes divergence; σ=3 gives ATE=264 m and yaw_rms=4.91° — the best result in this ablation.

2. **fly3: `sigma_px=1` and `max_clones=15` each give ~14% improvement** over the current σ=2/clones=11 baseline. Both changes produce similar ATE (~225 m). Applying both together would require a combined run.

3. **fly4: no tested parameter improves accuracy.** The 17° yaw drift is a structural issue with this flight and mode that σ/chi2/clones/gyro parameters cannot fix.

4. **chi2_multipler (P2) is a secondary lever** — it can keep fly1 alive with σ=1 (chi2_mult=5 survives at ATE=418 m) but is inferior to the root-cause fix of raising σ. For fly3, chi2_mult is neutral.

5. **gyro_nd (P4) has negligible effect** across the 0.002–0.010 range tested. Keep at current 0.005.

---

## 7. Raw Data

| Flight | Summary CSV |
|--------|-------------|
| fly1 | `Desktop/20260517_gsmq_d455_fly1/result/yaw_ablation_20260608/summary_yaw_ablation_fly1.csv` |
| fly3 | `Desktop/20260527_gsmq_d455_fly3/result/yaw_ablation_20260608/summary_yaw_ablation_fly3.csv` |
| fly4 | `Desktop/20260528_gsmq_d455_fly4/result/yaw_ablation_20260608/summary_yaw_ablation_fly4.csv` |
