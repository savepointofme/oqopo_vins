# tools/eval — Evaluation Tools

Evaluate VIO trajectory accuracy against GPS altitude (official Z metric)
or stereo VIO pseudo-reference (debug XY/Z only).

**Reference mode discipline:** Since 2026-05-17, all evaluation tools enforce
`--reference-mode`. You MUST pass this flag explicitly. There is no default.

---

## Scripts

### `eval_baselines.py` [GIT-TRACKED — PRIMARY EVALUATOR]
**Purpose:** Unified windowed evaluator for frozen baselines (BASELINES.md).

**Modes:**
- `--reference-mode gps_z` — Official Z-only evaluation against aligned GPS altitude.
  XY metrics are NaN. This is the only mode for official results.
- `--reference-mode stereo_pseudo_ref` — Debug-only XY/Z vs stereo VIO pseudo-reference.
  All outputs are labeled "stereo_pseudo_ref". Never cite as official.

**Required inputs:**
- `--flight N` — Flight number (1-4)
- `--runs NAME=path ...` — Trajectory files (e.g. `B0=.../B0_no_gps.txt`)
- `--out <dir>` — Output directory for plots and metrics JSON
- `--gps-alt-csv <path>` — Canonical aligned GPS (for `gps_z` mode)
- `--truth <path>` — Stereo pseudo-ref (for `stereo_pseudo_ref` mode)

**Official Z evaluation example (fly1):**
```bash
python tools/eval/eval_baselines.py \
    --flight 1 \
    --reference-mode gps_z \
    --gps-alt-csv 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv \
    --runs B0=20260509_fly1/result/baselines_v1/B0_no_gps.txt \
           B1=20260509_fly1/result/baselines_v1/B1_gps_height_d10.txt \
    --out comparison_plots/baselines_v1/fly1_gpsz
```

**Debug XY/Z example (fly1):**
```bash
python tools/eval/eval_baselines.py \
    --flight 1 \
    --reference-mode stereo_pseudo_ref \
    --truth 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/truth_asl_cam_time.csv \
    --runs B0=20260509_fly1/result/baselines_v1/B0_no_gps.txt \
           B1=20260509_fly1/result/baselines_v1/B1_gps_height_d10.txt \
    --out comparison_plots/baselines_v1/fly1_stereoref
```

**Outputs:** Per-flight plots (trajectory XY, Z timeseries, XY error) and `metrics.json`
with pre-descent ATE, per-edge scale ratios, and descent appendix.

**Pitfalls:**
- The script auto-detects descent start from the reference altitude trace.
  The pre-descent window is [eval_start, descent_start].
- For `gps_z` mode: Z RMSE is meaningful. XY columns are NaN.
- Do not use `truth_asl_*` files from `_corrected` (fly2) or the old fly1 offset directories.

---

### `_pseudo_ref_guard.py`
**Purpose:** Guard module imported by evaluation tools that use stereo pseudo-references.
Raises an error unless `--ack-stereo-pseudo-ref` is passed on the command line.

**Usage:** Imported, not run directly. Prevents accidental use of stereo VIO as truth.

---

### `audit_gps_references.py`
**Purpose:** Audit all GPS reference files in the workspace, checking row counts,
duration, altitude ranges, and ban-list compliance.

**Input:** None (scans the workspace)

**Output:** Console report of GPS file status per flight.

**Example:**
```bash
python tools/eval/audit_gps_references.py
```

---

### `evaluate_gps_start_yaw.py`
**Purpose:** Per-edge evaluation using the GPS altitude as start-of-edge yaw reference.
Computes scale ratio and heading error on each altitude-step edge.

**Input:** Trajectory TUM file, aligned GPS CSV, flight number

**Output:** Per-edge metrics CSV and plots

**WARNING (hardcoded paths):** Contains hardcoded dataset paths. Review before running
on a new dataset.

---

### `plot_trajectories.py`
**Purpose:** Generate XY trajectory comparison plots for multiple runs.

**Input:** List of TUM trajectory files and labels

**Output:** PNG plots

---

### `plot_per_edge_traj.py`
**Purpose:** Generate per-edge trajectory segment plots for scale and drift analysis.

---

### `run_iwt_eval.py`
**Purpose:** Run the evaluator for a sweep of `init_window_time` parameter values
across all 4 flights.

**Input:** Directory pattern for iwt sweep results

**Output:** Cross-flight metric table

---

## Canonical GPS inputs for each flight

Always use these for official Z evaluation:

```
fly1: 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv
fly2: 20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv
fly3: 20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv
fly4: 20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv
```

**NEVER use:** `_corrected` (fly2), `aligned_gps_from_raw_*` (fly1), or any file named `DO_NOT_USE`.

---

## Evaluation Windows

| Flight | eval_start_s | descent_cutoff |
|--------|-------------|----------------|
| fly1 | 200 | auto-detect from GPS z(t) |
| fly2 | 160 | auto-detect |
| fly3 | 180 | auto-detect |
| fly4 | 239 | auto-detect; fly4_until_gps580 variant also exists |

---

## Per-Edge vs Pre-Descent

- **Pre-descent evaluation:** Computes metrics over [eval_start, descent_start].
  This is the official window for comparing baselines.
- **Per-edge evaluation:** Splits the trajectory at GPS altitude edges
  (takeoff, each loop, landing) and computes per-edge scale ratio.
  Useful for diagnosing scale drift but not a single summary number.
