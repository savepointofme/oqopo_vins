# OC Velocity Accuracy — fly1 / fly2 / fly3 / fly4

**Date:** 2026-06-08  
**Script:** `analysis/vel_accuracy.py`  
**Output:** `Desktop/vel_accuracy_analysis_20260608/`

---

## 1. Method

**Trajectories used** (best available OC run per flight):

| Flight | Trajectory | Lines | Mode |
|--------|-----------|-------|------|
| fly1 | `result/gpsz_on_yawcmp_fly1_oc_fej_prechi2/traj.txt` | 24,283 | oc_fej_prechi2 |
| fly2 | `result/feature_gate_ablation_20260607/cond1e5_dist1000/traj.txt` | 59,965 | oc_postchi2_current_gauge |
| fly3 | `fc_rebuild_20260527/start618_oc1_gpsalt_guard_offset438p0_viz/traj.txt` | 29,440 | global_yaw_oc_projection |
| fly4 | `result/gpsz_on_yawcmp_fly4_oc_fej_prechi2/traj.txt` | 56,715 | oc_fej_prechi2 |

**Velocity derivation:** Savitzky-Golay filter (VIO: window=31 samples ≈ 1s, poly=3; GPS: window=7 samples, poly=3) applied to positions, followed by central finite differences. This approximates the instantaneous velocity from the position record; it is **not** the EKF's internal velocity state, which would be smoother.

**GPS truth:** WGS84 lat/lon/alt converted to ENU (anchored at first GPS point). GPS velocity derived by the same SG+finite-difference approach on GPS positions.

**Frame alignment:** A single yaw rotation is estimated from the mean horizontal velocity heading over the first 60 s of each eval window (weighted by speed) and applied to VIO vx/vy to align with GPS ENU. VIO vz is used as-is (already in ENU-up via GPS-Z fusion).

**Eval windows:**

| Flight | T0 | Until |
|--------|----|-------|
| fly1 | 930 s | 1740 s |
| fly2 | 700 s | 2700 s |
| fly3 | 618 s | 1600 s |
| fly4 | 924.4 s | 2816 s |

---

## 2. Per-Flight Results

### fly1 (~200 m altitude)

| Metric | vx | vy | vz | \|v\| spd | \|v\|H |
|--------|----|----|----|---------|----|
| RMSE (m/s) | 5.09 | 6.46 | **2.49** | **4.52** | 4.63 |
| Median AE  | 1.84 | 3.64 | **0.48** | **2.69** | 2.71 |
| P95 (m/s)  | 9.29 | 10.82 | 4.47 | 8.21 | 8.84 |
| Bias (m/s) | -0.64 | +0.04 | +0.31 | +1.34 | +1.21 |

- **3D velocity RMSE:** 8.59 m/s  (median: 5.45 m/s)  
- **Heading RMSE:** 14.28°  bias: +2.68°  P95: 18.40°  
- **Yaw alignment:** +0.06° (VIO frame nearly aligned with GPS ENU)  
- vy RMSE > vx: VIO is less accurate in the N direction; likely related to predominant E-W flight path

### fly2 (~500 m altitude) ⚠ worst

| Metric | vx | vy | vz | \|v\| spd | \|v\|H |
|--------|----|----|----|---------|----|
| RMSE (m/s) | 17.26 | 12.66 | **3.80** | **18.82** | 18.48 |
| Median AE  | 3.82 | 6.93 | **0.40** | **2.58** | 2.57 |
| P95 (m/s)  | 20.36 | 20.25 | 3.74 | 28.38 | 24.06 |
| Bias (m/s) | -1.47 | +0.34 | +0.12 | +0.05 | -0.29 |

- **3D velocity RMSE:** 21.74 m/s  (median: 9.60 m/s)  
- **Heading RMSE:** 21.03°  bias: +2.65°  P95: 31.12°  
- **Yaw alignment:** −12.18° (significant frame rotation vs GPS ENU)  
- The RMSE/median ratio is ~7× — dominated by large spikes, not steady-state error. Fly2 is a 2000 s straight-line flight at ~40 m/s with known feature starvation; EKF corrections at feature re-initialization appear as velocity transients.

### fly3 (~200 m altitude) ✓ best

| Metric | vx | vy | vz | \|v\| spd | \|v\|H |
|--------|----|----|----|---------|----|
| RMSE (m/s) | 4.62 | **2.20** | **0.51** | **4.24** | 4.24 |
| Median AE  | 2.51 | **0.68** | **0.17** | **2.40** | 2.40 |
| P95 (m/s)  | 8.73 | 4.07 | 0.83 | 8.73 | 8.73 |
| Bias (m/s) | -0.33 | -0.19 | -0.004 | **−2.56** | −2.57 |

- **3D velocity RMSE:** 5.14 m/s  (median: 2.82 m/s)  
- **Heading RMSE:** 6.86°  bias: +1.32°  P95: 5.85° ← **best heading accuracy**  
- **Yaw alignment:** +3.92°  
- vz RMSE = 0.51 m/s — best of all flights; GPS-Z fusion working well  
- Speed bias = −2.56 m/s: OC consistently under-estimates speed magnitude; likely systematic (scale drift or SG smoothing attenuating speed peaks)  
- vy is accurate (RMSE=2.20, median=0.68); vx carries most of the error

### fly4 (~400 m altitude) ⚠

| Metric | vx | vy | vz | \|v\| spd | \|v\|H |
|--------|----|----|----|---------|----|
| RMSE (m/s) | 14.93 | 14.57 | **1.13** | **13.34** | 13.34 |
| Median AE  | 6.45 | 11.04 | **0.29** | **3.05** | 3.04 |
| P95 (m/s)  | 24.69 | 24.34 | 1.40 | 23.96 | 23.89 |
| Bias (m/s) | -0.87 | -1.22 | -0.003 | -1.09 | -1.10 |

- **3D velocity RMSE:** 20.89 m/s  (median: 15.41 m/s)  
- **Heading RMSE:** 26.63°  **bias: +19.32°** ← large persistent heading offset  
- **Yaw alignment:** −0.25° (VIO frame is nearly aligned in first 60 s)  
- The +19.32° heading bias after alignment means the OC yaw estimate drifts persistently by ~19° during the flight. This is the most significant finding for fly4: the `oc_fej_prechi2` mode on fly4 has a systematic yaw error that shows up as a heading offset in the velocity direction.  
- vz RMSE=1.13 m/s — better than fly1/fly2 but GPS-Z guard may be causing intermittent updates

---

## 3. Cross-Flight Summary

```
Flight  3D RMSE  Spd RMSE  SpdH     Hdg RMSE   vz RMSE  Spd bias  Yaw align
fly1     8.59     4.52     4.63     14.28°      2.49     +1.34     +0.06°
fly2    21.74    18.82    18.48     21.03°      3.80     +0.05    -12.18°
fly3     5.14     4.24     4.24      6.86°      0.51     -2.56     +3.92°
fly4    20.89    13.34    13.34     26.63°      1.13     -1.09     -0.25°
```

**Best velocity accuracy:** fly3 — RMSE=4.24 m/s, heading=6.86°  
**Worst velocity accuracy:** fly2 — RMSE=18.82 m/s, heading=21.03°

---

## 4. Analysis

### Does velocity error correlate with position drift?

Yes, broadly. Flights with better position ATE also tend to have lower velocity RMSE:
- fly3 (best position accuracy) → best velocity RMSE
- fly2 and fly4 (worst position) → worst velocity RMSE

However the correlation is not tight: fly1 has position RMSE ~216 m but velocity RMSE comparable to fly3 (which has 231 m). The **duration and dynamics** of the flight matter as much as position accuracy.

### Does velocity error correlate with yaw drift?

fly4's +19.32° heading bias is the clearest example: persistent yaw error directly appears as a heading bias in the horizontal velocity vector. fly2's large yaw alignment correction (−12.18°) suggests significant frame misalignment throughout that flight.

### Why is fly3 the best?

- Shorter flight (618–1600 s = 982 s vs 2000 s for fly2/fly4)
- Lower altitude (~200 m), better feature geometry and depth observability
- Good feature initialization (varied terrain vs fly2's featureless straight-line)
- vz RMSE=0.51 m/s indicates GPS-Z fusion is making consistent corrections

### Why are fly2 and fly4 poor?

- **fly2:** 2000 s straight-line, 500 m altitude, feature starvation (near-zero perpendicular parallax) causes EKF jumps → large velocity spikes
- **fly4:** 1891 s flight, 400 m altitude; the +19.32° heading bias points to unresolved yaw drift in the `oc_fej_prechi2` mode at this flight duration/trajectory

### RMSE vs median gap

The RMSE/median ratio is 1.7× (fly3) to 7.3× (fly2). High ratio = dominated by transient spikes (EKF corrections, chi2 gates reopening), not steady-state error. The median error (2–3 m/s across all flights) is more representative of typical in-flight velocity quality.

---

## 5. Conclusion

| Flight | Spd RMSE | Median | Hdg RMSE | Hdg bias | vz RMSE | Assessment |
|--------|----------|--------|----------|----------|---------|------------|
| fly3 | 4.24 | 2.40 | 6.86° | +1.32° | 0.51 | Best; borderline usable for coarse velocity |
| fly1 | 4.52 | 2.69 | 14.28° | +2.68° | 2.49 | Comparable to fly3 in speed; heading unreliable |
| fly4 | 13.34 | 3.05 | 26.63° | **+19.32°** | 1.13 | Yaw drift is a hard problem; not usable |
| fly2 | 18.82 | 2.58 | 21.03° | +2.65° | 3.80 | Dominated by EKF jumps from feature starvation |

**OC position-derived velocity is not a reliable velocity source** at the 1–2 m/s accuracy level needed for control. The key reasons:

1. **Position-derived velocity amplifies EKF state jumps** into large transient velocity spikes. The EKF's internal velocity state (not in traj.txt) would be considerably smoother.
2. **Heading accuracy is insufficient** for fly1, fly4 (>14° RMSE after alignment).
3. **fly2 and fly4 have systematic drift** — fly2 from feature starvation, fly4 from yaw divergence.

**What OC improves:** vz is well-controlled (0.51–3.80 m/s), consistent with GPS-Z fusion anchoring the altitude. Fly3's heading RMSE of 6.86° is the only result suggesting OC yaw estimates may be trustworthy for low-altitude, feature-rich, short flights.

**Recommendation:** For velocity use, the EKF internal velocity state should be extracted directly from the binary (via `save_total_state` output) rather than derived by differencing traj.txt positions. That would bypass the SG-smoothing limitation and give the actual filtered velocity estimate.

---

*Analysis script: `analysis/vel_accuracy.py`  
Plots: `Desktop/vel_accuracy_analysis_20260608/*.png`  
CSV: `Desktop/vel_accuracy_analysis_20260608/vel_accuracy_summary.csv`*
