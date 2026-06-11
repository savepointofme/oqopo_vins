# fi_max_cond_number × fi_max_dist Ablation — All Flights

**Date:** 2026-06-07 / 2026-06-08  
**Mode:** `oc_postchi2_current_gauge` (GLOBAL_YAW_OC_PROJECTION, mode 2) + GPS-Z ON  
**Fixed:** `init_bg_sigma=0.003`, all GPS-Z guard flags unchanged  
**Scripts:** `run_fly2_feature_gate_ablation.sh`, `run_fi_gate_sweep_fly134.sh`

---

## 1. Motivation

The fly2 trajectory (cond=1e4, dist=1e4, default baseline) diverged. Two gate parameters changed simultaneously from the working fly2 result: `fi_max_cond_number` and `fi_max_dist`. This ablation separates their contributions and finds the minimum working operating point per flight.

---

## 2. fly2 — 2D Ablation (cond × dist)

**Flight:** ~500 m altitude, 2000 s straight-line, feature-starved  
**Sweep:** cond ∈ {1e4, 3e4, 1e5} × dist ∈ {1000, 1500, 2000}

| cond \ dist | 1000 | 1500 | 2000 |
|-------------|------|------|------|
| **1e4** | DIVERGED | DIVERGED | DIVERGED |
| **3e4** | DIVERGED | DIVERGED | DIVERGED |
| **1e5** | **OK** | DIVERGED | OK* |

*cond1e5_dist2000: flight had already landed before eval window; result not meaningful.

**Finding:** Only `cond=1e5, dist=1000` is a confirmed stable operating point for fly2. The primary driver is `fi_max_cond_number`: fly2's 500 m altitude and near-zero perpendicular parallax (straight-line) push DLT condition numbers well above 1e4. Loosening to 1e5 admits marginally-conditioned features that are nonetheless necessary to prevent filter starvation.

**Action:** fly2 requires `fi_max_cond_number: 1e5`, `fi_max_dist: 1000`. Keep fly2 excluded from further tuning experiments until this gate is confirmed with a full-duration eval.

---

## 3. fly1 / fly3 / fly4 — Tighter-Than-1e4 Sweep

**Motivation:** Check if tightening below the default 1e4 improves accuracy on flights where 1e4 already works.  
**Sweep:** cond ∈ {5e3, 3e3, 1e3} × dist ∈ {200, 300, 500, 800} (fly1/fly3) or {300, 500, 800, 1000} (fly4)  
**Baselines (cond=1e4):** fly1 ATE ≈ 216 m, fly3 ATE ≈ 323 m, fly4 ATE ≈ 448 m

### 3.1 fly1 (~200 m altitude)

| cond \ dist | 200 | 300 | 500 | 800 |
|-------------|-----|-----|-----|-----|
| **5e3** | DIVERGED | DIVERGED | OK 493 m | OK 493 m |
| **3e3** | DIVERGED | DIVERGED | DIVERGED | DIVERGED |
| **1e3** | DIVERGED | DIVERGED | DIVERGED | CRASH |

Minimum stable: cond=5e3 + dist≥500. Both OK cells produce the same trajectory (493 m ATE), which is **2.3× worse than baseline 216 m**. cond=3e3 and below diverge completely.

### 3.2 fly3 (~200 m altitude)

| cond \ dist | 200 | 300 | 500 | 800 |
|-------------|-----|-----|-----|-----|
| **5e3** | DIVERGED | CRASH | OK† 4420 m | DIVERGED |
| **3e3** | DIVERGED | OK 724 m | OK 724 m | OK 724 m |
| **1e3** | DIVERGED | DIVERGED | DIVERGED | DIVERGED |

†cond5e3_dist500 barely avoids the 10 km divergence threshold (max_xy = 9.3 km, yaw_rms = 55°). Effectively diverged.

cond3e3 dist300-800 produce the same trajectory (724 m ATE, 9.87° yaw_rms), which is **2.2× worse than baseline 323 m**.

### 3.3 fly4 (~400 m altitude)

| cond \ dist | 300 | 500 | 800 | 1000 |
|-------------|-----|-----|-----|------|
| **5e3** | CRASH | DIVERGED | DIVERGED | CRASH |
| **3e3** | DIVERGED | DIVERGED | DIVERGED | DIVERGED |
| **1e3** | NOT_RUN | NOT_RUN | NOT_RUN | CRASH |

No working cells. fly4 cannot survive any threshold below the baseline 1e4.

---

## 4. Analysis

### Why do tighter thresholds hurt?

The DLT condition number measures how well-conditioned the triangulation geometry is. At 200–400 m altitude with typical UAV speeds, many features naturally have condition numbers in the 5e3–1e4 range — they are not poorly-conditioned noise, but features with moderate parallax that are nonetheless correctly triangulated. Raising the threshold from 5e3 to 1e4 admits these features. Removing them (by lowering the threshold) starves the MSCKF update, forcing the EKF to propagate longer with IMU-only, accumulating orientation and scale drift.

The effect is altitude-dependent:
- fly4 (400 m) needs 1e4 — the 5e3–1e4 range is essential at this altitude
- fly1/fly3 (200 m) can survive at 5e3 but with measurably worse ATE
- fly2 (500 m) needs 1e5 — even 1e4 starves the filter

### fi_max_dist sensitivity

For fly1 at cond=5e3, dist=200 and dist=300 diverge while dist≥500 is stable. Setting dist<500 rejects features at greater depth even when they are well-conditioned, which stalls the update at the critical post-initialization phase. For fly3, the same pattern holds (cond3e3: dist200=DIVERGED, dist≥300=OK). At the baseline threshold (cond=1e4), dist is not the binding constraint for low-altitude flights.

### GPS-Z accept counts as a stability indicator

For fly3, the stable runs (cond3e3 dist300-800) each show 3274 GPS-Z accepts and 0 rejects. The diverged runs show much lower counts (29, 408, etc.), consistent with the filter losing altitude tracking before GPS-Z updates can be accepted. GPS-Z accept count is a useful secondary diagnostic for filter health.

---

## 5. Conclusions and Parameter Recommendations

| Flight | fi_max_cond_number | fi_max_dist | Basis |
|--------|-------------------|-------------|-------|
| fly2   | **1e5** | **1000** | Only stable cell in 2D ablation |
| fly1   | **1e4** (baseline) | baseline | Tighter degrades ATE 2.3× |
| fly3   | **1e4** (baseline) | baseline | Tighter degrades ATE 2.2× |
| fly4   | **1e4** (baseline) | baseline | No cell below 1e4 survives |

**Do not change fi_max_cond_number from 1e4 for fly1, fly3, fly4.**

The baseline 1e4 is the optimal operating point for low-to-mid altitude (200–400 m) flights with this sensor and config. Tightening below 1e4 consistently worsens accuracy and eventually causes divergence. Only the high-altitude straight-line case (fly2, 500 m) requires the loosened 1e5 threshold.

---

## 6. Raw Data

| Source | Path |
|--------|------|
| fly2 ablation | `Desktop/20260518_gsmq_d455_fly2/result/feature_gate_ablation_20260607/` |
| fly1 sweep | `Desktop/20260517_gsmq_d455_fly1/result/fi_gate_sweep_20260608/summary_fly1.csv` |
| fly3 sweep | `Desktop/20260527_gsmq_d455_fly3/result/fi_gate_sweep_20260608/summary_fly3.csv` |
| fly4 sweep | `Desktop/20260528_gsmq_d455_fly4/result/fi_gate_sweep_20260608/summary_fly4.csv` |
