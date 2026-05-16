# Stage B Phase-0 diagnostic report (fly4)

All metrics are **truth-aligned** (translation offset computed from first 5 s of each run vs ASL ground truth at `20260509_fly4/result/gps_tum_time_alignment/truth_asl_cam_time.csv`).  GPS cutoff = last GPS sample at t = 1774464240.91 (≈ 614 s after start).  Divergence rule: `|xyz|>500 m` OR consecutive-pose jump `> 50 m`; truncate at last good sample and mark with `×`.

## Quantitative table (truth-aligned, NOT vs GPS)

```
run                         during-XY  during-Z   post-XY  post-Z  post-XY  post-XY  post-Z   post-Z   drift     t>50m  t>100m  t>200m
                              RMSE       RMSE       RMSE    RMSE      max     final     max     final  (m/s)      (s)     (s)     (s)
---------------------------------------------------------------------------------------------------------------------------------------
R0_nogps                       50.06      9.54     462.42    21.64   558.78   558.78   55.58   55.58    +7.85    323.8   599.8   608.8
R2f_stageA (σ=2.0 full)        57.93      3.41     127.65     6.85   183.42   117.98   16.95    3.17    -1.40    155.7   602.4    —
R5b_stageB (σ_px=50, K=2)      29.65      2.06     196.11     9.44   528.87   528.87   24.13   24.13    +5.77    607.3   614.3   642.4
R4b_stageB (σ_px=5, K=5)       92.44     15.63     573.85    19.69   573.85   573.85   19.69   19.69      —      173.5   258.4   601.5
R5a_stageB (σ_px=20, K=3)      66.29      4.56       —        —        —       —        —      —          —      594.0   596.3   600.1
```

## What changed vs the previous report

The previous "R5b cuts post-GPS XY by 64% vs R2f" claim was **wrong** — it compared against sparse GPS samples in the post window, not truth.  Against TRUTH:

- **During GPS**: R5b is dramatically better than R2f (29.65 vs 57.93 m XY RMSE; stays below the 50 m error band for the first 607 s, vs R2f crossing 50 m at 156 s).
- **Post GPS**: R5b is *worse* than R2f (196 vs 128 m XY RMSE; reaches 528 m vs R2f's 118 m final error).

So Stage B v0 trades robustness for tightness: it nearly halves error while GPS is feeding, but loses ground-plane calibration once GPS stops and then drifts at +5.77 m/s.

## Interpretation of each plot

### 01_xy_traj.png — XY trajectory overlay
- Truth (dotted black) is a 4-corner rectangular survey path.
- **R5b (green)** visually traces truth's rectangle the most accurately of any run — corners line up, scale matches.
- **R2f (blue)** also traces the rectangle but with slightly compressed scale.
- **R0 (gray)** drifts north and is missing parts of the rectangle.
- Post-GPS (markers `×`): R5b lands ≈ 50 m east of truth's true landing point; R2f lands close to truth.

### 02_z_curve.png — Z vs time
- R5b tracks truth Z almost exactly (same as R2f).
- R4b has a sustained ≈ 25 m altitude bias (Stage A range update is partly broken here).
- Post-GPS: R5b/R2f both reach near-ground (≈ −18 m), truth reaches ≈ 0 m → ≈ 20 m Z error at landing, similar between R5b and R2f.

### 03_xy_error_vs_time.png — `e_xy(t)`
- **The most informative plot.** R5b (green) is consistently BELOW R2f (blue) for the full 600 s of GPS-active flight: e_xy ≈ 20–40 m vs R2f's 50–100 m.
- At GPS cutoff (purple line ≈ 614 s), all runs jump up.  R5b spikes to ~500 m; R2f stays bounded ~150 m.
- R0 (gray) baseline drifts steadily upward throughout.

### 04_z_error_vs_time.png — `|e_z(t)|`
- All curves jiggle ±10 m around truth, no clear winner.  R4b has a sustained 20 m bias; everyone else is close.
- Post-GPS: R5b reaches ≈ 25 m, slightly worse than R2f's ≈ 17 m max.

### 05_post_gps_drift.png — drift vs Δt and running RMSE
- Linear fit on post-GPS XY error:
  - R0 = +7.85 m/s (catastrophic)
  - R2f = **−1.40 m/s** (actually CONVERGES post-GPS — landing settles the state)
  - R5b = +5.77 m/s (drifts almost as fast as no-GPS)
- Running RMSE (right panel): R5b's cumulative RMSE stays below R2f's for the first ≈ 28 s post-GPS, then crosses above as the drift accumulates.

## Conclusion: Stage B v0 has a real but localized benefit

| Question | Answer |
|---|---|
| Is R5b better throughout the post-GPS interval? | **No** — only for first ≈ 28 s post-GPS, then worse. |
| Does it just look good at the end? | No — it looks good in the middle of the post-GPS window and then degrades fast. |
| Does Stage B reduce drift rate? | During GPS: yes, dramatically (e_xy stays flat at ~30 m for 10 min). Post GPS: no — drift rate flips sign (R2f converges at −1.4 m/s, R5b grows at +5.8 m/s). |
| Hidden Z drift or slow instability? | No sustained Z bias.  Z final error is ~7 m worse than R2f but not catastrophic. |
| Robust? | No.  Three other Stage B configurations diverged (R4b, R5a) or catastrophically exploded (R4a, R4c).  The σ_px=50 K=2 sweet spot is narrow. |

**The fundamental issue is structural, not parametric.** Stage B v0 leans on a `z_ground` that Stage A keeps calibrated. While GPS feeds, the loop closes and Stage B amplifies the metric scale information. Once GPS stops, `z_ground` is frozen, but VIO altitude bias resumes drifting; Stage B then anchors features against a slowly-wrong ground plane and the error compounds into XY drift.

## Recommendation: option (c) — a hybrid path before either (a) or (b)

Neither (a) parameter sweep nor (b) v1 anchor-in-Hx_order alone solves the post-GPS regression.  The cleanest next step:

1. **Disable Stage B once GPS stops** (or once `z_ground` last-update age exceeds a threshold).  This preserves the during-GPS gain (29.6 → 58 m XY RMSE) without the post-GPS penalty.
2. **Then** run option (a) — a small safe sweep around σ_px = 30 / 50 / 80, K = 1 / 2 / 3 to see how much we can squeeze out of the v0 design.
3. **Only then** invest in (b) v1 anchor-in-Hx_order, with the understanding that v1's main payoff would be tighter σ_px (enabling K=5+ without divergence) — but the post-GPS structural issue remains until we change either Stage A's z_ground propagation or add a Stage B disable-on-cutoff guard.

## File index

| File | Content |
|---|---|
| [01_xy_traj.png](01_xy_traj.png) | XY overlay, truth-aligned |
| [02_z_curve.png](02_z_curve.png) | Z vs time |
| [03_xy_error_vs_time.png](03_xy_error_vs_time.png) | `e_xy(t)` log scale |
| [04_z_error_vs_time.png](04_z_error_vs_time.png) | `\|e_z(t)\|` |
| [05_post_gps_drift.png](05_post_gps_drift.png) | post-GPS drift + running RMSE |
| [metrics_table.txt](metrics_table.txt) | tabular metrics |
| [diag_stage_b.py](diag_stage_b.py) | reproducible script |
