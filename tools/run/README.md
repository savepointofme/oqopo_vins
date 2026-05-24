# tools/run — Standard Run Scripts

Run scripts for reproducing baseline experiments and parameter sweeps.

**Build prerequisite:** Binary must be built first:
```bash
# From WSL:
cd /mnt/d/vscode_dir/open_vins/ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
# Binary: /mnt/d/vscode_dir/open_vins/build_ov_msckf/run_serial_msckf_ros_free
```

---

## Scripts

### `run_iwt5_validation.sh`
**Purpose:** Batch runner for `init_window_time=5.0` validation across all 4 flights.
Runs B1 (GPS-height) and v1_E1 variants.

**Config used:** `config/user_drone_mono_jc82/estimator_config_iwt5_candidate.yaml`
(NOTE: this config file is currently untracked)

**Run from WSL repo root:**
```bash
bash tools/run/run_iwt5_validation.sh both    # run B1 + v1_E1 for all flights
bash tools/run/run_iwt5_validation.sh b1      # run B1 only
bash tools/run/run_iwt5_validation.sh v1      # run v1_E1 only
```

**Outputs:** Written to `20260509_fly{N}/result/init_window_time_sweep/`

**WARNING (hardcoded paths):** This script hardcodes the workspace root as
`/mnt/d/vscode_dir/open_vins`. If the workspace is relocated, update the `cd` line
at the top and the dataset path references.

**GPS references used:**
```
fly1: gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv
fly2: gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv
fly3: gps_tum_time_alignment/aligned_gps_cam_time.csv
fly4: gps_tum_time_alignment/aligned_gps_cam_time.csv
```

---

## Standard Single-Flight Run Template

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
    --config config/user_drone_mono_jc82/estimator_config.yaml \
    --dataset 20260509_fly1/mav0 \
    --no-display \
    --start-time 200 \
    --gps-time-offset 0.0 \
    --gps 20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv \
    --gps-alt-update \
    --gps-alt-sigma 2.0 \
    --gps-alt-min-pzz 0.01 \
    --gps-alt-min-t-after-init 10 \
    --output 20260509_fly1/result/<run_name>.txt \
    > 20260509_fly1/result/<run_name>.log 2>&1
```

**Key flags:**
- `--no-display` — headless (no OpenCV window); required on WSL
- `--start-time N` — skip first N seconds (wait for VIO initialization)
- `--gps-time-offset 0.0` — must be 0 when using canonical aligned GPS
- `--gps-alt-update` — enable GPS altitude fusion (B1 / v1_E1 mode)
- `--gps-alt-sigma 2.0` — GPS altitude measurement noise (2.0 m = default)
- `--gps-alt-min-pzz 0.01` — minimum P_zz floor before K/S computation
- `--gps-alt-min-t-after-init 10` — seconds to wait after VIO init before first GPS update

For B0 (no GPS): omit all `--gps*` flags.

---

## Per-Flight Start Times

| Flight | `--start-time` | Notes |
|--------|---------------|-------|
| fly1 | 200 | VIO needs ~200 s to initialize from static start |
| fly2 | 160 | Shorter init; starts moving earlier |
| fly3 | 180 | |
| fly4 | 239 | Longer static period |

---

## Important Rules

1. **Always record the exact run command** in the experiment manifest.
2. **Always record the git commit** at run time: `git rev-parse HEAD`
3. **Never use `--gps-alt-relative` for fusion experiments** unless explicitly testing
   the relative GPS mode. Absolute altitude is the standard.
4. **GPS horizontal must not be fused** into the EKF. Only `--gps-alt-update` is permitted.
5. **Stereo mode requires different config:** `config/user_drone_stereo_jc82/estimator_config.yaml`.
   Do not mix mono and stereo configs.
