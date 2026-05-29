# Reproduction: visual_yaw_schmidt_current_gauge Stage Validation

Branch: `review/stage-schmidt-fly3-yaw-align`  
Dataset: fly3 (`d455_20260526_174946`), offset=438p0, start=618s  
Binary: `./build_ov_msckf/run_serial_msckf_ros_free` (ENABLE_ROS=OFF)

---

## Prerequisites

### 1. Build (ENABLE_ROS=OFF)

```bash
cd /mnt/d/vscode_dir/open_vins/build_ov_msckf
make -j4 run_serial_msckf_ros_free
```

Build dir is the pre-existing out-of-tree build at repo root (`build_ov_msckf/`),
configured with `cmake -DENABLE_ROS=OFF`.

### 2. Pre-filter IMU window (one-time, avoids 373K-row CSV parse)

```bash
cd /mnt/d/vscode_dir/open_vins
awk -F',' 'NR==1{print; next} $1+0>=617.9 && $1+0<=1601.0{print}' \
  '/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946/imu0/data.csv' \
  > imu_window_fly3_full.csv
```

---

## Stage 1: Run both methods to t=900s

```bash
bash run_stage_schmidt.sh 900 stage1
```

Output dirs:
```
$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until900/
$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until900/
```

Where `BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527`.

---

## Stage 2: Run both methods to t=1600s (parallel, with --viz-fast)

Run from a WSL terminal (not Claude Code Bash tool) so WSLg display is available:

```bash
cd /mnt/d/vscode_dir/open_vins

# Clean any partial outputs first
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
rm -rf "$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until1600"
rm -rf "$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until1600"

bash run_stage2_parallel.sh
```

Output dirs:
```
$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until1600/
$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until1600/
```

---

## Evaluation (start+yaw-only alignment)

### Stage 1 (900s)

```bash
BASE=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --until 900 \
  --dir-a "$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until900" \
  --dir-b "$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until900" \
  --gps  "$BASE/gps_from_mems_offset438p0_cam_time.csv" \
  --imu  /mnt/d/vscode_dir/open_vins/imu_window_fly3_full.csv \
  --out  "$BASE/stage_schmidt_eval_until900"
```

### Stage 2 (1600s)

```bash
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --until 1600 \
  --dir-a "$BASE/stage_schmidt_A_global_oc_offset438p0_start618_until1600" \
  --dir-b "$BASE/stage_schmidt_B_fullstate_schmidt_offset438p0_start618_until1600" \
  --gps  "$BASE/gps_from_mems_offset438p0_cam_time.csv" \
  --imu  /mnt/d/vscode_dir/open_vins/imu_window_fly3_full.csv \
  --out  "$BASE/stage_schmidt_eval_until1600"
```

---

## Alignment method

**Start-position + initial-yaw only. No global trajectory fitting.**

```
p_aligned(t) = R(yaw_gps_start - yaw_vio_start) * (p_vio(t) - p_vio_start) + p_gps_start
```

Parameters (documented in `eval_stage.py`):
- `T0 = 618.0` s — evaluation start
- `YAW_WIN = 5.0` s — initial yaw estimation window `[618, 623]`s
- `SPEED_GATE = 2.0` m/s — GPS course speed gate for yaw estimation
- `yaw_gps_start`: circular mean of speed-gated GPS course headings in `[T0, T0+YAW_WIN]`
- `yaw_vio_start`: VIO **velocity direction** at T0: `atan2(vx, vy)` from `traj.txt.bias`
  (columns 2,3 = vx, vy in VIO global frame). GPS course = direction of motion, so the
  velocity vector is the correct counterpart — not the body quaternion heading (which
  introduces a ~5.7° sideslip offset). Residual with velocity method: ~0.03°.
- `p_gps_start`: GPS ENU position interpolated at T0
- `p_vio_start`: VIO position interpolated at T0

No Umeyama, no SE(2) least-squares, no ICP, no time-varying re-alignment, no scale.

---

## Output structure

```
stage_schmidt_eval_until900/
  metrics_summary.csv     — all scalar metrics, A vs B
  gps_xy_overlay.png      — GPS truth + A + B (start+yaw aligned, no global fitting)
  xy_ate_vs_time.png      — XY ATE vs time with segment bands
  yaw_err_vs_time.png     — VIO yaw error vs GPS course truth
  heading_850_930s.png    — local heading detail [850,930]s

stage_schmidt_eval_until1600/
  (same structure, until=1600s with early/middle/late segments)
```

---

## Key configuration flags

| Flag | A | B |
|---|---|---|
| `--vio-yaw-update-mode` | `global_yaw_oc_projection` | `visual_yaw_schmidt_current_gauge` |
| `--vio-global-yaw-oc-alpha` | `1.0` | *(not used)* |
| `--schmidt-yaw-diag` | *(not used)* | `$DIR_B/schmidt_yaw_update_diag.csv` |
| `--gps-alt-update` | yes | yes |
| `--gps-alt-sigma` | 2.0 | 2.0 |
| `--gps-alt-guard-dxy` | 0.5 | 0.5 |
| `--gps-alt-guard-kxy` | 5.0 | 5.0 |
| `--init-from-fc` | offset438p0_start618 | offset438p0_start618 |
| `--init-bg-sigma` | 0.003 | 0.003 |
| `--viz-fast` | yes (Stage 2) | yes (Stage 2) |
