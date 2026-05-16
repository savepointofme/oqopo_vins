# Frozen Baselines

Two reference baselines are frozen on the `baseline-stageA-delay-stageBv0`
branch (commit `7ecb87c`). All subsequent comparisons — Stage B v1 included —
must be measured against these, not against earlier ad-hoc runs.

## B0 — no-GPS baseline

Pure monocular OpenVINS with no Stage A and no Stage B.

| field | value |
| ----- | ----- |
| binary | `build_ov_msckf/run_serial_msckf_ros_free` (built from `7ecb87c`) |
| config | `config/user_drone_mono_jc82/estimator_config.yaml` |
| GPS | none |
| Stage A | disabled |
| Stage B | disabled |
| time offset | `--gps-time-offset 0.0` (irrelevant — no GPS) |

Command line:
```bash
$BIN \
  --config config/user_drone_mono_jc82/estimator_config.yaml \
  --dataset 20260509_flyN/mav0 \
  --no-display \
  --start-time <per-flight, see below> \
  --gps-time-offset 0.0 \
  --output 20260509_flyN/result/baselines_v1/B0_no_gps.txt
```

## B1 — GPS/height baseline (Stage A bootstrap delay = 10s)

Stage A with ground-plane range update only. **No horizontal GPS.**
Bootstrap delay = 10s (this is the `R6_d10`-style setting).

| field | value |
| ----- | ----- |
| binary | same as B0 |
| config | same as B0 |
| GPS CSV | `20260509_flyN/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` |
| `--gps-alt-update` | yes |
| `--gps-alt-ground-plane` | yes |
| `--gps-alt-sigma` | `2.0` |
| `--gps-alt-min-pzz` | `0.01` |
| `--gps-alt-min-t-after-init` | `10` |
| Stage B | disabled |

Command line:
```bash
$BIN \
  --config config/user_drone_mono_jc82/estimator_config.yaml \
  --dataset 20260509_flyN/mav0 \
  --gps 20260509_flyN/result/gps_tum_time_alignment/aligned_gps_cam_time.csv \
  --gps-alt-update --gps-alt-ground-plane \
  --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --no-display \
  --start-time <per-flight> \
  --gps-time-offset 0.0 \
  --output 20260509_flyN/result/baselines_v1/B1_gps_height_d10.txt
```

## Per-flight start-times

Each flight uses a fixed start-time. **All compared runs of the same flight
must use the same start-time.**

| flight | start-time (s) | init status on current binary |
| ------ | -------------: | -------- |
| fly1 | 200 | OK (`[init]: successful initialization in 0.0018s`) |
| fly2 | **160** | OK (was 185 historically; jc82 init refuses 185 because the drone is already mid-flight there. The drone is on the ground at 160s and takes off shortly after — static init fires.) |
| fly3 | **180** | OK (was 244 historically; same reason as fly2. 200 also fails on jc82.) |
| fly4 | 239 | OK — matches current canonical `stage_a_v2/R0_nogps.txt` (R0 and R6_d10 reused as B0/B1) |

History note: the historical start-times (185 / 244) worked under the PR17-era binary because that build's initializer used looser thresholds. The current jc82 mono config requires either a stationary takeoff window or a smaller mid-flight window for dynamic init; the table above lists the validated values.

## fly4 reuses existing canonical runs

`R0_nogps.txt` (B0) and `R6_delay10s.txt` (B1) in
`20260509_fly4/result/stage_a_v2/` were produced from this exact binary/config
and are already deterministic (verified by `R0_fresh_no_gps.txt` ≡
`R0_nogps.txt` byte-for-byte). The fly4 baseline pair is symlinked / copied
into `baselines_v1/` for uniform path access.

## Evaluation rules — applies to every run, every baseline, every report

**Always print these timestamps:**

1. Trajectory start time (first VIO sample, absolute and `t_rel = t - t0_truth`).
2. Evaluation start time (`eval_start`) — defaults to 0s rel, unless a
   transient window is being excluded.
3. GPS / `z_ground` first update time (if applicable).
4. GPS cutoff time (`--gps-cutoff-time`, if set).
5. Detected final-descent start time (`descent_start`) — see auto-detection below.
6. Evaluation end time (`eval_end`) — equals `descent_start` for the main
   window. The descent / landing segment is reported only as an appendix.
7. Trajectory end time.

**Descent auto-detection (truth-based):**

Given the truth `z(t)` time series:

1. Restrict to `t in [10%, 90%]` of duration; let `z_max` = max of `z` there.
2. `z_end` = `z` at the last truth sample.
3. `thresh = z_max - 0.5 * (z_max - z_end)`.
4. `descent_start` = the first `t > argmax(z)` after which `z(t) < thresh`
   continuously until the end. Falls back to "no descent detected" if none
   found; in that case `eval_end = trajectory_end`.

The detected `descent_start` is printed in every report and table.

**Common-window rule when runs end at different times:**

When two runs are compared and one ends earlier than the other, the
evaluator computes metrics on the **intersection** of their valid windows.
The table header marks the comparison "common-window" with the actual
intersection bounds. PR17 ending 53s before R0 must never silently bias a
comparison.

**Main metric window:**

`[eval_start, descent_start]` — the pre-descent window. All headline metrics
(scale_ratio, first-edge XY RMSE, pre XY/Z RMSE, pre max/final XY) are
computed here. The descent appendix gets its own table.

## What is *not* an official baseline

These runs remain as empirical references but **must not** be used in place
of B0 / B1 for cross-experiment ranking:

| ref | what | why kept | why not a baseline |
| --- | ---- | -------- | ------------------ |
| R5b | Stage B v0, `sigma_px=50`, `K=2`, no Stage-A delay | best fly4 pre-descent XY (26.53m) | early-scale under-bias (`ratio=0.927`) |
| R7  | R6_d10 + R5b combined | recovers early scale | not better than R6_d10 in pre-descent XY; descent diverges |
| PR17 | older binary + `user_drone_mono` config | historical comparison | non-bootable on current binary; provenance unclear |

## Cross-flight baseline evaluation

For every flight where both B0 and B1 succeed, the evaluator produces:

- first-edge / early-segment `scale_ratio` (when a clean straight segment exists)
- pre-descent XY RMSE
- pre-descent Z RMSE
- pre-descent max XY
- pre-descent final XY
- XY trajectory plot
- XY error vs time plot
- Z vs time plot
- a flag for whether a valid post-GPS cruise window exists

Cross-flight aggregation: report each metric per flight and the median across
flights. Do not collapse flights into a single number without first showing
the per-flight table.
