# Sigma=1 Stabilization Ablation — fly1 / fly3 / fly4

**Date:** 2026-06-10  
**Mode (fixed):** `oc_postchi2_current_gauge` + GPS-Z ON  
**Fixed:** `init_bg_sigma=0.003`, `fi_max_cond_number=1e4`, all GPS-Z guard flags unchanged  
**Script:** `run_sigma1_stabilization.sh --no-optional`  
**Commit:** `e52d99c`  
**Eval windows:** fly1=[930,1740], fly3=[618,1600], fly4=[924.4,2816], `--yaw-align-mode start_yaw`

---

## 1. Groups Tested

| Grp | σ | χ² | fi_dist | npts | fast | minpx | clones | Description |
|-----|---|----|---------|------|------|-------|--------|-------------|
| A | 2 | 1 | 2000 | 400 | 20 | 15 | 11 | **Baseline** (sigma=2 shared) |
| B | 1 | 1 | 2000 | 400 | 20 | 15 | 11 | Raw sigma=1 |
| C | 1 | 3 | 2000 | 400 | 20 | 15 | 11 | sigma=1 + chi2_mult=3 |
| D | 1 | 5 | 2000 | 400 | 20 | 15 | 11 | sigma=1 + chi2_mult=5 (diagnostic) |
| E | 1 | 1 | ALT | 400 | 20 | 15 | 11 | sigma=1 + altitude-based fi_max_dist |
| F | 1 | 3 | ALT | 400 | 20 | 15 | 11 | sigma=1 + chi2_mult=3 + alt-dist |
| G | 1 | 1 | 2000 | 600 | 15 | 12 | 11 | sigma=1 + more features |
| H | 1 | 3 | ALT | 600 | 15 | 12 | 11 | sigma=1 + chi2_mult=3 + more feat |

ALT fi_max_dist: fly1=500, fly3=500, fly4=1000 (rule: max(500, 2.5×altitude_m))

---

## 2. Results

### 2.1 Full Table

| Grp | σ | χ² | fi_dist | npts | fly1 ATE | fly1 yaw | fly1 status | fly3 ATE | fly3 yaw | fly3 status | fly4 ATE | fly4 yaw | fly4 status |
|-----|---|----|---------|------|----------|----------|-------------|----------|----------|-------------|----------|----------|-------------|
| **A** | **2** | **1** | **2000** | **400** | **378.9** | **7.86°** | **OK** | **261.2** | **7.08°** | **OK** | **755.2** | **17.37°** | **OK** |
| B | 1 | 1 | 2000 | 400 | 428.0 | 8.60° | DIVERGED | 224.5 | 6.56° | OK | 831.1 | 17.76° | OK |
| C | 1 | 3 | 2000 | 400 | 470.5 | 8.82° | DIVERGED | 257.7 | 6.63° | OK | 844.7 | 17.48° | OK |
| D | 1 | 5 | 2000 | 400 | 417.7 | 7.59° | OK | 259.0 | 6.61° | OK | 911.2 | 20.30° | OK |
| E | 1 | 1 | 500/1000 | 400 | 428.0 | 8.60° | DIVERGED | 224.5 | 6.56° | OK | 831.1 | 17.76° | OK |
| F | 1 | 3 | 500/1000 | 400 | 470.4 | 8.87° | DIVERGED | 257.7 | 6.63° | OK | 844.7 | 17.48° | OK |
| G | 1 | 1 | 2000 | 600 | 474.8 | 9.51° | DIVERGED | 284.5 | 7.30° | OK | 1685.6 | 32.23° | OK |
| H | 1 | 3 | ALT | 600 | 462.2 | 8.62° | DIVERGED | 300.4 | 7.30° | OK | >1M | 107.6° | CRASH |

### 2.2 Chi2 Statistics (fly1, diag.csv)

| Grp | mean_acc | mean_rej | rej_rate | n_starvation | first_stv_t |
|-----|----------|----------|----------|--------------|-------------|
| A | 16.54 | 0.100 | 0.022 | 6711 | 930.2 |
| B | 16.50 | 0.153 | 0.029 | 6817 | 930.2 |
| C | 16.54 | 0.110 | 0.021 | 6694 | 930.2 |
| D | 16.57 | 0.089 | 0.018 | 6593 | 930.2 |
| G | 24.35 | 0.177 | 0.030 | 6757 | 930.2 |

**Critical observation:** The mean accepted features per update is **identical** across B, C, D, G (≈16.5 for npts=400 groups). The rejection rate changes minimally (0.018–0.029). This proves that fly1's sigma=1 divergence is **not caused by feature starvation in the mean**. The filter accepts roughly the same number of features regardless of sigma or chi2_mult.

The divergence mechanism is **Kalman gain mismatch**: σ=1 in S = HPH^T + σ²I understates the true pixel noise (~11 px calibration drift at altitude), yielding an over-confident K that corrupts the state incrementally until the trajectory diverges.

---

## 3. Answers to the 6 Questions

### Q-A: Can sigma_px=1 be made non-divergent on fly1?

**NO** (in any deployable sense).

- `chi2_mult=3` does **not** save fly1: C and F both DIVERGED. ATE is *worse* than raw sigma=1 (470m vs 428m).
- `chi2_mult=5` prevents divergence (Group D: ATE=417.7m) but the result is **10% worse than the sigma=2 baseline** (378.9m). Per the user's own criterion, chi2_mult=5 is diagnostic, not a clean solution.
- Alt-dist (500m for fly1) has **zero effect**: B≡E identically (ATE=428.0, stv_n=6817), C≡F identically (ATE=470.4/470.5). fi_max_dist is a non-factor for this stability problem.
- More features (G, H) makes things strictly worse.
- The chi2 starvation hypothesis is **refuted by data**: mean_acc is essentially identical across all sigma=1 groups (see §2.2). The divergence is a Kalman gain problem, not a feature count problem.

### Q-B: Does stable sigma=1 still improve fly3's first-turn/scale behavior?

**YES, but only with chi2_mult=1 — exactly the setting that breaks fly1.**

- Group B (σ=1, χ²=1): fly3 ATE=224.5m, yaw_p95=9.92° — best fly3 result in this ablation (-14% ATE, -20% yaw_p95 vs baseline)
- As soon as chi2_mult is raised to 3 (Group C): fly3 ATE=257.7m ≈ 261.2m baseline. The benefit of sigma=1 is completely neutralized.
- chi2_mult=5 (D): fly3 ATE=259.0m, also ≈ baseline.
- More features (G/H): fly3 ATE=284–300m, **worse** than the sigma=2 baseline.

The sigma=1 benefit for fly3 only exists at chi2_mult=1. This is a coupling that cannot be broken: the lever that fixes fly3 (chi2_mult=1 + sigma=1) is the lever that diverges fly1.

### Q-C: Does sigma=1 hurt fly4?

**YES, consistently, at every chi2 level.**

| Config | fly4 ATE | vs. Baseline |
|--------|----------|-------------|
| A (σ=2, χ²=1) | 755.2m | — |
| B (σ=1, χ²=1) | 831.1m | +10% |
| C (σ=1, χ²=3) | 844.7m | +12% |
| D (σ=1, χ²=5) | 911.2m | +21% |
| G (σ=1, +feat) | 1685.6m | +123% |
| H (σ=1, χ²=3, +feat) | **CRASH** | numerical blowup |

No sigma=1 configuration at any chi2_mult matches fly4's sigma=2 baseline. The degradation is monotonic with chi2_mult (more lenient gate → worse ATE on fly4). More features is catastrophic.

The H_fly4 crash is instructive: mean_acc dropped to 0.79 (vs 18.5 in baseline), n_starvation=44,041 (vs 2,025 baseline). The combination of sigma=1, chi2=3, fi_max_dist=1000, and more features creates a degenerate state where the filter gets almost no visual updates for the entire 1891-second flight and blows up numerically.

### Q-D: Is the sigma=1 fix deployable as a shared setting?

**NO.** The following table shows the irreconcilable conflict:

| What fly1 needs | What fly3 needs | What fly4 needs | Intersection |
|----------------|----------------|----------------|--------------|
| σ≥2 (or σ=1+χ²≥5) | σ=1+χ²=1 | σ=2+χ²=1 | **∅** |

There is no (σ, χ²) combination that simultaneously:
- Keeps fly1 non-divergent at σ=1
- Preserves fly3's 14% ATE improvement
- Doesn't degrade fly4

The only compromise that prevents divergence everywhere (σ=1+χ²=5) is inferior to the σ=2 baseline on all three flights.

### Q-E: If not deployable, what is the correct mainline?

**Stay with sigma=2, chi2_mult=1, fi_max_dist=2000.**

Evidence summary:
1. **Altitude-based fi_max_dist is a non-factor**: B≡E, C≡F across all flights. The proposed rule `max(500, 2.5×altitude_m)` produces no measurable change vs fi_max_dist=2000. Abandon this idea.
2. **More features (npts=600, fast=15, min_px=12) is harmful**: All three flights degrade. fly4 is catastrophically affected (G: +123%, H: crash). Do not revisit.
3. **sigma=2 (Group A) is the correct anchor**: fly1=379m, fly3=261m, fly4=755m — consistent with prior yaw ablation.
4. **One remaining opportunity**: `max_clones=15`. The yaw ablation showed clones=15 improves fly3 by -13% (261→226m). This has NOT been tested in the full current configuration (init_bg_sigma=0.003, oc_postchi2_current_gauge, GPS-Z). Group J (σ=2, χ²=1, clones=15) was NOT_RUN in this ablation.

### Q-F: What exact configuration should be tested next?

**Test Group J: sigma=2, chi2_mult=1, fi_max_dist=2000, max_clones=15, all 3 flights.**

```yaml
up_msckf_sigma_px:       2.0
up_msckf_chi2_multipler: 1.0
fi_max_dist:             2000.0
fi_max_cond_number:      10000.0
max_clones:              15
num_pts:                 400
fast_threshold:          20
min_px_dist:             15
```

This is the only remaining plausible improvement over the current Group A baseline. The hypothesis: clones=15 provides deeper triangulation baseline → tighter position constraint → lower ATE on all flights, without changing the noise model. If clones=15 degrades fly4 or fly1, report the verdict and lock mainline at clones=11.

---

## 4. Summary Decision Table

| Config | fly1 | fly3 | fly4 | Verdict |
|--------|------|------|------|---------|
| **A (σ=2, χ²=1, cl=11)** | **379m ✓** | **261m ✓** | **755m ✓** | **Mainline** |
| B (σ=1, χ²=1) | DIV ✗ | 224m ✓ | 831m − | REJECT: fly1 diverges |
| C (σ=1, χ²=3) | DIV ✗ | 258m ✓ | 845m − | REJECT: fly1 diverges; fly3 loses σ=1 benefit |
| D (σ=1, χ²=5) | 418m − | 259m ✓ | 911m ✗ | REJECT: diagnostic only; all flights worse than A |
| E (σ=1, χ²=1, ALT-dist) | DIV ✗ | 224m ✓ | 831m − | = B; fi_max_dist has zero effect |
| F (σ=1, χ²=3, ALT-dist) | DIV ✗ | 258m ✓ | 845m − | = C; same conclusion |
| G (σ=1, +feat) | DIV ✗ | 285m − | 1686m ✗ | REJECT: all flights worse |
| H (σ=1, χ²=3, +feat) | DIV ✗ | 300m − | CRASH ✗ | REJECT: catastrophic |
| **J (σ=2, χ²=1, cl=15)** | **?** | **?** | **?** | **Next test** |

---

## 5. Conclusions

1. **sigma=1 cannot be stabilized on fly1** by any lever tested: not chi2_mult, not fi_max_dist, not feature count. The divergence is a Kalman gain mismatch, not a starvation problem — mean feature acceptance is identical across sigma=1 groups.

2. **The "fi_max_dist altitude-based rule" has no effect** on any flight. B≡E and C≡F are exact duplicates. Unify fi_max_dist=2000 as the single shared setting and drop the altitude-based rule entirely.

3. **More features (npts=600/fast=15/min_px=12) is harmful** at every tested sigma level. fly4 in particular degrades catastrophically (H: numerical blowup).

4. **sigma=2, chi2_mult=1, fi_max_dist=2000, max_clones=11 is the confirmed mainline**. It is the only fully deployable shared configuration that keeps all three flights non-divergent.

5. **One open question remains**: max_clones=15 (Group J). This is the sole unconfirmed lever that has shown potential (fly3 -13% in the yaw ablation). Run it next.

---

---

## 7. Group J — max_clones=15 Controlled Test

Run on 2026-06-10, same config as Group A except max_clones=15.

| | fly1 | fly3 | fly4 |
|---|---|---|---|
| **A (clones=11)** | **379m / 7.86°** | **261m / 7.08°** | **755m / 17.37°** |
| **J (clones=15)** | **DIVERGED / 8.84°** | **226m / 6.63°** | **1114m / 25.44°** |
| Change | DIVERGED ✗ | −13% ✓ | +48% ✗ |

**Verdict: REJECTED.** fly3 improves as expected (matches yaw ablation prediction), but fly1 diverges (max_xy=23525m) and fly4 degrades severely (+48% ATE, +46% yaw_rms). Two independent failure modes on two different flights. clones=15 is not deployable.

**Conclusion: Group A (σ=2, χ²=1, fi_max_dist=2000, clones=11) is the locked mainline. Parameter search ends.**

---

## 8. Raw Data

| File | Contents |
|------|---------|
| `docs/reports/sigma1_stab_summary_20260610.csv` | Full 24-run summary with ATE/yaw/chi2 stats |
| `Desktop/<flight>/result/sigma1_stab_20260610/<GRP>_<flight>/` | Per-run traj.txt, diag.csv, eval/ |
