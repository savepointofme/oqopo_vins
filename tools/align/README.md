# tools/align — GPS / Truth Alignment Tools

Align raw GPS (ArduPilot time domain) to VIO camera/IMU time domain.

**Critical rule:** Never skip alignment. Raw GPS `TimeUS` is ArduPilot boot-relative
microseconds, not camera nanoseconds. Using raw GPS directly will produce wrong fusion results.

---

## Scripts

### `align_gps_traj_timestamps.py`
**Purpose:** Align raw GPS CSV (ArduPilot `TimeUS`) to camera time by scanning a
horizontal-speed cross-correlation between GPS velocity profile and a reference VIO trajectory.

**Input:**
- `--gps GPS.csv` — Raw ArduPilot GPS dump
- `--tum reference.txt` — A reference VIO trajectory in TUM format (used only for timing alignment; this can be the stereo pseudo-reference, labeled as such)
- `--imu imu0/data.csv` — IMU CSV (provides camera-IMU timeshift)
- `--output-dir <dir>` — Output directory

**Output:**
- `aligned_gps_cam_time.csv` — GPS with timestamps in camera time (nanoseconds)
- `aligned_gps_imu_time.csv` — GPS with timestamps in IMU time
- `alignment_summary.json` — Documents the offset and method used
- `offset_scan.csv` — Time-offset scan results

**Example (fly1):**
```bash
python tools/align/align_gps_traj_timestamps.py \
    --gps D:/vscode_dir/20260509_fly1_gps/GPS.csv \
    --tum 20260509_fly1/reference/one_traj_estimate_stereo.txt \
    --imu 20260509_fly1/mav0/imu0/data.csv \
    --fixed-offset -4.63 \
    --output-dir 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63
```

**Canonical aligned outputs (DO NOT REGENERATE without reason):**

| Flight | Canonical aligned GPS |
|--------|----------------------|
| fly1 | `20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv` |
| fly2 | `20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv` |
| fly3 | `20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` |
| fly4 | `20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv` |

**WARNING (hardcoded paths):** This script contains hardcoded default paths referencing
the workspace root at `/mnt/d/vscode_dir/open_vins`. Update these if the workspace is moved.

**Pitfalls:**
- The `_corrected` alignment for fly2 is BANNED (cross-contaminated). Always use `_vertical`.
- The `aligned_gps_from_raw_*.csv` files in fly1 `mav0/gps/` are STALE (from truncated GPS). Do not use.
- The alignment uses a VIO trajectory as timing reference, not as position truth.
  The choice of reference TUM file affects the estimated time offset but not the GPS positions.

---

### `align_gps_tum_truth.py`
**Purpose:** Convert GPS altitude CSV into TUM-compatible truth format, or produce
a `truth_asl_*` CSV for use as a positional pseudo-reference.

**Input:** `aligned_gps_cam_time.csv` + VIO trajectory in TUM format

**Output:** `truth_tum_cam_time.txt` (TUM format), `truth_asl_cam_time.csv` (ASL format)

**WARNING:** The `truth_asl_cam_time.csv` files inside `gps_tum_time_alignment*/`
subdirectories are derived from the stereo VIO trajectory, NOT from GPS positions.
They are labeled `stereo_pseudo_ref` and must never be called "ground truth".

**Example:**
```bash
python tools/align/align_gps_tum_truth.py \
    --gps 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv \
    --tum 20260509_fly1/reference/one_traj_estimate_stereo.txt \
    --output-dir 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63
```

---

### `imu_align.py`
**Purpose:** Verify camera-IMU timeshift from `kalibr_imucam_chain.yaml` and apply it
to convert between camera and IMU time domains.

**Input:** `kalibr_imucam_chain.yaml`, camera data CSV

**Output:** IMU-time aligned CSV, timeshift summary

**Canonical timeshift for jc82:** 13080437 ns (0.013080 s), from `calib_results/`.

---

## Common Pitfalls

1. **Using the wrong aligned GPS for fly2:** Only `gps_tum_time_alignment_vertical/` is valid.
   The `gps_tum_time_alignment_corrected/` directory contains BANNED contaminated files.

2. **Regenerating alignment without checking the GPS input:** The `alignment_summary.json`
   for fly1/fly3/fly4 records unexpected GPS source directories. Verify the GPS file
   row count and duration match the expected values before rerunning.

3. **Setting `--gps-time-offset` on the runner:** When using the canonical aligned GPS,
   set `--gps-time-offset 0.0`. The timestamps are already in camera time.
   Only set a nonzero offset if using raw GPS directly (not the aligned CSV).
