# Frozen Baselines

Two reference baselines are frozen on the `baseline-stageA-delay-stageBv0`
branch (commit `7ecb87c`). All subsequent comparisons — Stage B v1 included —
must be measured against these, not against earlier ad-hoc runs.

> **2026-05-17 data-cleanup update — read first.**
> A reference audit found two data problems that invalidate earlier
> cross-flight numbers:
>
> 1. **`20260509_fly1/mav0/gps/GPS.csv` was truncated** to the first 348 s
>    of the flight (one altitude loop instead of two). It has been replaced
>    by the full-flight log from `D:\vscode_dir\20260509_fly1_gps\GPS.csv`
>    (950 s, alt up to 124.85 m). The old file is preserved at
>    `20260509_fly1/mav0/gps/GPS.truncated.DO_NOT_USE.csv` for forensics.
>    See `20260509_fly1/mav0/gps/README_GPS_SOURCE.md`.
> 2. **`20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_cam_time.csv`
>    is cross-contaminated** — it carries fly1's GPS sequence re-timed to
>    fly2's clock. It has been renamed to
>    `aligned_gps_cam_time.DO_NOT_USE_contaminated.csv`. fly2 must use
>    `gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv` as both
>    fusion input and evaluation reference. See the directory README.
> 3. **`truth_asl_*.csv` is stereo VIO output, not real ground truth.** All
>    evaluators have been gated behind `--reference-mode`. The label
>    `stereo_pseudo_ref` MUST be used wherever such files are read; do not
>    cite "truth RMSE" against them. See `GPS_REFERENCE_AUDIT.md`.
>
> **Conclusions affected:** every cross-flight XY/Z RMSE conclusion from
> before 2026-05-17 used at least one of the broken inputs above. They are
> **not official** and must be re-derived after a clean rerun. Specifically
> invalidated: fly1 and fly2 B1 / v1_E1 RMSE numbers, all fly1/2/3 XY RMSEs
> computed against `truth_asl_*` (debug-only henceforth), and any
> first-edge `scale_ratio` derived from `truth_asl_*` XY. fly4 numbers
> derived from its own (correct) GPS and stereo_pseudo_ref are debug-grade
> but their XY component is still pseudo-reference and must be re-labeled.
>
> Stage B v1's analytic Jacobian validation (FD pass rate 99.91%) and the
> runtime stability logs (accepted/called, residuals, update dxy) remain
> valid — they do not depend on the corrupted references.

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
| GPS CSV | per-flight (see "Per-flight GPS fusion inputs" below) |
| `--gps-alt-update` | yes |
| `--gps-alt-ground-plane` | yes |
| `--gps-alt-sigma` | `2.0` |
| `--gps-alt-min-pzz` | `0.01` |
| `--gps-alt-min-t-after-init` | `10` |
| Stage B | disabled |

Command line (replace `<GPS_CSV>` with the per-flight path below):
```bash
$BIN \
  --config config/user_drone_mono_jc82/estimator_config.yaml \
  --dataset 20260509_flyN/mav0 \
  --gps <GPS_CSV> \
  --gps-alt-update --gps-alt-ground-plane \
  --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --no-display \
  --start-time <per-flight> \
  --gps-time-offset 0.0 \
  --output 20260509_flyN/result/baselines_v1/B1_gps_height_d10.txt
```

### Per-flight GPS fusion inputs (post 2026-05-17 cleanup)

| flight | aligned GPS CSV used as `--gps` |
| ------ | --------------------------------- |
| fly1   | `20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv` (950.4 s, AGL ≤ 92 m, matches full fly1 GPS) |
| fly2   | `20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv` (490.6 s, AGL ≤ 55 m) |
| fly3   | `20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` (540.2 s, AGL ≤ 69 m) |
| fly4   | `20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` (618.8 s, AGL ≤ 88 m) |

**Banned as fusion input** (do not use):
- `20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_cam_time.DO_NOT_USE_contaminated.csv` — contaminated.
- `20260509_fly1/mav0/gps/GPS.truncated.DO_NOT_USE.csv` — truncated copy.
- `20260509_fly1/mav0/gps/aligned_gps_from_raw_*.csv` — derived from the
  truncated fly1 GPS; must be regenerated against the full GPS before reuse.

Authoritative raw GPS sources (re-derive aligned files from these if needed):
- `D:\vscode_dir\20260509_fly1_gps\GPS.csv`
- `D:\vscode_dir\20260509_fly2_gps\GPS.csv`
- `D:\vscode_dir\20260509_fly3_gps\GPS.csv`
- `D:\vscode_dir\20260509_fly4_gps\GPS.csv`

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

**Reference-mode discipline (mandatory):**

- `truth_asl_*.csv` is a stereo VIO trajectory. It is NOT real ground truth.
  Report any metric computed against it as `stereo_pseudo_ref` only.
- The official Z metric is the VIO Z compared against the **aligned GPS
  altitude** for that flight (per the table above). Use
  `eval_baselines.py --reference-mode gps_z`.
- XY comparison against `stereo_pseudo_ref` is debug-only — never as
  "truth RMSE". When no high-precision XY reference is available, report
  trajectory shape, length, scale ratio, and side-by-side plots; do not
  fabricate an "XY RMSE" number.
- Auto-detection of `descent_start` must run on the chosen reference's z(t)
  (GPS altitude in `gps_z` mode; stereo Z is acceptable for debug
  pseudo-ref runs but the result is labeled as such).

**Descent auto-detection (reference-based):**

Given the chosen reference's `z(t)` time series:

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
