# GPS / Reference Data Audit (2026-05-17)

This document records the corrected reference-data policy for the
`20260509_fly{1,2,3,4}` datasets, the audit findings that triggered the
policy, and the cleanup actions taken. It is the single source of truth
on **what is and is not** a valid evaluation reference.

## TL;DR

1. **Stereo VIO output is NOT ground truth.** Files derived from
   `reference/*_traj_estimate_stereo.txt` (especially the
   `gps_tum_time_alignment*/truth_asl_*.csv` files) are a stereo VIO
   trajectory estimate. They are debug pseudo-references only. Any script
   that reads them must be invoked with `--reference-mode stereo_pseudo_ref`
   (in `tools/eval_baselines.py`) or with `--ack-stereo-pseudo-ref` (in
   `tools/diag_*.py`).
2. **Official Z reference = aligned GPS altitude** per flight (table
   below). `tools/eval_baselines.py --reference-mode gps_z` is the official
   Z evaluator.
3. **No official XY truth exists** for these flights. Report trajectory
   shape / scale / plots qualitatively. Do not fabricate "XY RMSE vs truth".
4. **fly1 in-tree GPS was truncated** to the first 348 s of the flight. It
   has been replaced by the full 950 s log from
   `D:\vscode_dir\20260509_fly1_gps\GPS.csv`. The truncated original is
   preserved as `GPS.truncated.DO_NOT_USE.csv`.
5. **fly2 `_corrected` aligned GPS was cross-contaminated** (carried fly1's
   GPS sequence). Renamed to `*.DO_NOT_USE_contaminated.csv`. fly2 must use
   the `_vertical` aligned GPS as both fusion input and reference.
6. **Pre-2026-05-17 cross-flight XY/Z RMSE conclusions are invalid** —
   they used at least one of the broken inputs above.

## Canonical raw GPS sources

Authoritative raw GPS dumps (full ArduPilot logs) live outside the repo:

    D:\vscode_dir\20260509_fly1_gps\GPS.csv
    D:\vscode_dir\20260509_fly2_gps\GPS.csv
    D:\vscode_dir\20260509_fly3_gps\GPS.csv
    D:\vscode_dir\20260509_fly4_gps\GPS.csv

These directories also contain `BARO.csv` (barometer), `RFND.csv` (downward
rangefinder), `POS.csv`/`AHR2.csv` (autopilot state estimates),
`summary.json`, and a `gps_plot.html` visualizer. Treat the `GPS.csv` files
as the authoritative raw GPS for each flight.

## Per-flight reference catalog (after cleanup)

| Flight | Raw GPS                                                  | Aligned GPS for fusion + Z eval                                                                       | AGL_max | Loops |
|--------|----------------------------------------------------------|-------------------------------------------------------------------------------------------------------|---------|-------|
| fly1   | `D:\vscode_dir\20260509_fly1_gps\GPS.csv` (950.4 s, 4753 pts) | `20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv` (950.4 s, 4753 pts) | ~92 m   | 50 m AND 90 m |
| fly2   | `D:\vscode_dir\20260509_fly2_gps\GPS.csv` (490.6 s, 2401 pts) | `20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv` (490.6 s, 2401 pts)    | ~55 m   | 50 m only |
| fly3   | `D:\vscode_dir\20260509_fly3_gps\GPS.csv` (540.2 s, 2702 pts) | `20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` (540.2 s, 2702 pts)             | ~69 m   | one (≤ 70 m) |
| fly4   | `D:\vscode_dir\20260509_fly4_gps\GPS.csv` (618.8 s, 3095 pts) | `20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` (618.8 s, 3095 pts)             | ~88 m   | 50 m AND 90 m |

The in-tree `mav0/gps/GPS.csv` matches the external full GPS exactly for
fly2/fly3/fly4. Only fly1's in-tree copy was previously truncated.

The "two altitude loops" (a ~50 m and a ~90 m loop) memory only applies
to **fly1 and fly4**. fly2 and fly3 each have a single altitude loop.

## Banned / quarantined files

| File                                                                                                | Reason                                              |
|-----------------------------------------------------------------------------------------------------|-----------------------------------------------------|
| `20260509_fly1/mav0/gps/GPS.truncated.DO_NOT_USE.csv`                                               | Old truncated copy (348 s vs full 950 s)             |
| `20260509_fly1/mav0/gps/aligned_gps_from_raw_*.csv` (8 files)                                       | Derived from the truncated fly1 GPS — must be regenerated against the full GPS before reuse |
| `20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_cam_time.DO_NOT_USE_contaminated.csv` | Cross-contaminated (fly1 GPS sequence re-timed to fly2 clock) |
| `20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_imu_time.DO_NOT_USE_contaminated.csv` | Same as above (imu_time variant) |

## Reference taxonomy (use this language)

| Term                 | Source                                                  | Valid uses                                          |
|----------------------|---------------------------------------------------------|-----------------------------------------------------|
| `gps_alt` (= GPS-Z)  | aligned GPS altitude CSV                                 | **Official Z reference**. Use in `eval_baselines.py --reference-mode gps_z`. |
| `gps_xy`             | aligned GPS lat/lon (or ENU x/y)                         | Coarse sanity plotting only. Not "XY truth".         |
| `stereo_pseudo_ref`  | `truth_asl_*.csv` derived from `reference/*_traj_estimate_stereo.txt` | Debug-only XY/Z comparison on inspected stable segments. **Never** call this "truth". |
| `estimator_output`   | `B0`, `B1`, `R5b`, `R7`, `v1_E1`, etc.                  | An algorithm result. Never a reference.              |
| `baro_alt` (future)  | `D:\vscode_dir\20260509_flyN_gps\BARO.csv`              | Candidate independent Z reference; not yet wired in.|
| `rfnd_agl` (future)  | `D:\vscode_dir\20260509_flyN_gps\RFND.csv`              | Candidate AGL reference at low altitudes; not yet wired in.|

## Evaluator policy

- **`tools/eval_baselines.py`** is gated behind `--reference-mode {stereo_pseudo_ref, gps_z}`.
  It refuses to run without one of these flags.
  - `gps_z` mode: Z-only metric vs aligned GPS altitude; XY columns NaN.
  - `stereo_pseudo_ref` mode: full XY/Z metric vs `truth_asl_*`, but
    labeled "stereo_pseudo_ref" and accompanied by a warning banner.
- **`tools/diag_step1_coverage.py` / `diag_step2_early_scale.py` /
  `diag_step3_timing.py` / `diag_data_full_alt.py` / `diag_data_coverage.py`**
  require `--ack-stereo-pseudo-ref` to acknowledge that their output is
  debug-only. The acknowledgement helper is `tools/_pseudo_ref_guard.py`.
- **`tools/audit_gps_references.py`** does not depend on stereo VIO; it
  audits raw and aligned GPS only.

## Invalidations — pre-2026-05-17 results

Any result computed before 2026-05-17 that depended on one of the broken
inputs is **not official** and must be re-derived after a clean rerun.
Concretely:

- All cross-flight XY RMSE numbers vs `truth_asl_*` (fly1, fly2, fly3,
  fly4) — these were comparisons to stereo VIO, mis-labeled as "truth".
- fly1 B1 / v1_E1 Z metrics derived from `aligned_gps_cam_time.csv` ARE
  in scope to keep IF the aligned GPS file is the `offset_m4p63` one
  (which spans the full 950 s); the fly1 result files in
  `result/baselines_v1/` were produced from the truncated mav0 GPS,
  however, so they should be **re-run** before any Z metric is cited.
- fly2 B1 / v1_E1 results were produced with the `_corrected`
  (contaminated) aligned GPS as fusion input. These are **invalid** and
  must be re-run with the `_vertical` aligned GPS.
- fly3 and fly4 baseline runs are unaffected by the GPS contamination
  issues — but their XY RMSE numbers (against `truth_asl_*`) are still
  debug-only and must be re-labeled as `stereo_pseudo_ref` comparisons.

What remains valid without re-derivation:

- Stage B v1 analytic Jacobian validation via finite difference (FD pass
  rate 99.91% over 54,765 features). This is independent of the GPS /
  truth pipeline.
- Per-run runtime stability logs: `accepted/called`, `skip_nofeat`,
  residual statistics, update `dxy`. These are intrinsic to the run and
  do not depend on the reference.
- Qualitative trajectory shape comparisons via plots.

## Next steps (post-cleanup, in order)

1. Rerun B0 / B1 / v1_E1 per flight with the corrected GPS table above.
2. Score Z with `eval_baselines.py --reference-mode gps_z`.
3. Use `stereo_pseudo_ref` mode for debug-grade XY plotting only.
4. Open the **first-edge scale inconsistency** as the next diagnostic target
   (per BASELINES.md follow-ups). Do not add gates or sigma sweeps to mask it.
