# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

### ROS-free build (primary workflow on this branch)
```bash
cd ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j$(nproc)
```
The build directory is `build_ov_msckf/` at the repo root (pre-existing out-of-tree build).

### ROS1 / ROS2 build
```bash
# ROS1 (catkin workspace)
catkin_make --pkg ov_core ov_init ov_msckf ov_eval

# ROS2 (colcon workspace)
colcon build --packages-select ov_core ov_init ov_msckf ov_eval
```

CMake options:
- `-DENABLE_ROS=OFF` — builds without ROS (uses raw CSV/PNG datasets)
- `-DENABLE_ARUCO_TAGS=OFF` — disable if OpenCV contrib modules are unavailable

Dependencies: Eigen3, OpenCV 3 or 4, Boost (system filesystem thread date_time), Ceres Solver. C++14 required.

## Running the ROS-free offline runner

```bash
./run_serial_msckf_ros_free \
    --config ../../config/user_drone_stereo_jc82/estimator_config.yaml \
    --dataset /path/to/mav0 \
    --stereo \
    --gt    /path/to/mav0/state_groundtruth_estimate0/data.csv \
    --output traj_tum.txt \
    --video  dashboard.mp4 \
    --align-seconds 8.0
```

Key flags: `--no-display` for headless, `--gps <csv>` for GPS overlay, `--verbose` for per-frame timing, `--gps-time-offset <s>` to override the YAML `gps_time_offset`.

Dataset must be EuRoC/ASL format: `mav0/{imu0,cam0,cam1}/data.csv` + `data/*.png`.

## Architecture Overview

### Package dependency order
```
ov_core  →  ov_init  →  ov_msckf
                     →  ov_eval
```

**`ov_core`** — sensor-agnostic library: camera models (`CamRadtan`, `CamEqui`), feature tracking (`TrackKLT`, `TrackDescriptor`, `TrackAruco`), feature container (`Feature`, `FeatureDatabase`, `FeatureInitializer`), state types (`PoseJPL`, `IMU`, `Landmark`), simulation (`BsplineSE3`), utilities.

**`ov_init`** — two-mode inertial initializer: `StaticInitializer` (gyro/accel mean → gravity alignment) and `DynamicInitializer` (Ceres-based SfM+IMU optimization). Entry point: `InertialInitializer`.

**`ov_msckf`** — EKF filter core:
- `VioManager` (`core/`) — central dispatcher; owns all subsystems
- `State` + `StateHelper` (`state/`) — sliding-window state with modular covariance; uses JPL quaternion convention throughout
- `Propagator` (`state/`) — IMU integration (RK4 by default), produces Φ and Q for EKF predict
- `UpdaterMSCKF` / `UpdaterSLAM` / `UpdaterZeroVelocity` (`update/`) — EKF measurement update
- `ROS1Visualizer` / `ROS2Visualizer` (`ros/`) — ROS interface layer
- `run_serial_msckf_ros_free` + `VizDashboard` (`ros_free/`) — ROS-free offline runner with OpenCV dashboard

**`ov_eval`** — offline trajectory evaluation: ATE, RPE, NEES, SE(3)/Sim(3) alignment.

### Data flow
1. **IMU** → `VioManager::feed_measurement_imu` → `Propagator` (accumulate) + `InertialInitializer` (pre-init)
2. **Camera** → `VioManager::feed_measurement_camera` → `track_image_and_update`:
   - `TrackKLT/Descriptor` produces observations → `FeatureDatabase`
   - `propagate_and_clone` advances state and adds a new clone
   - `UpdaterMSCKF` / `UpdaterSLAM` pull from `FeatureDatabase` → EKF update
   - `StateHelper::marginalize_old_clone` removes oldest clone to maintain window size

### State vector layout
```
[ IMU(15) | Clones(6 each) | SLAM features(3 each) | Calibration ]
```
`Type::id()` gives each block's start index in the global covariance. `StateHelper` manages covariance augmentation and marginalization.

### Key conventions
- **JPL quaternion** convention throughout (not Hamilton). `q_GtoI` means rotation from global to IMU.
- **First-Estimate Jacobians (FEJ)** enabled via `use_fej: true` in YAML for EKF consistency.
- `_clones_IMU` is keyed by timestamp (double); the oldest clone is marginalized each update cycle.

## Configuration

YAML configs live in `config/`. This repo's active drone config: `config/user_drone_stereo_jc82/estimator_config.yaml`.

Key YAML parameters to know:
- `use_stereo` / `max_cameras` — stereo vs mono mode
- `max_clones` — sliding window size (trades compute for accuracy)
- `max_slam` / `max_msckf_in_update` — feature budget per update
- `feat_rep_msckf` / `feat_rep_slam` — feature parameterization (e.g. `GLOBAL_3D`, `ANCHORED_MSCKF_INVERSE_DEPTH`)
- `calib_cam_extrinsics/intrinsics/timeoffset` — online calibration flags
- `gps_time_offset` — constant offset (seconds) applied as `t_corrected = t_raw + offset`
- `use_klt_gyro_prediction` — gyro-aided KLT initial guess (custom feature on this branch)

## Custom Modifications on This Branch

This branch (`fix/alt-scale-realtime-landing-20260512`) adds several features not in upstream OpenVINS:

1. **Gyro-aided KLT** (`ov_core/src/track/TrackKLT`): IMU rotation integrated over the inter-frame interval warps the previous-frame keypoints before passing to `calcOpticalFlowPyrLK`, improving tracking under fast rotation. Controlled by `use_klt_gyro_prediction` YAML flag. The `Propagator` exposes predicted rotation via `get_last_imu_rotation_prediction()`.

2. **ROS-free offline runner** (`ov_msckf/src/run_serial_msckf_ros_free.cpp`): Replays EuRoC/ASL datasets without ROS. Merges IMU and camera timestamps in a single loop, feeding `VioManager` in strict time order.

3. **VizDashboard** (`ov_msckf/src/ros_free/VizDashboard.{h,cpp}`): ROS-free OpenCV 1920×1080 nine-panel dashboard (3D iso, top-down XY, camera feed, ATE, speed, altitude, roll, pitch, yaw). SE3-only Umeyama alignment (no scale) to preserve VIO scale observability.

4. **GPS altitude fusion** (`ov_msckf/src/core/VioManager`): 1D EKF measurement `h(x)=p_z` anchors VIO altitude to GPS. Key flags: `--gps-alt-update`, `--gps-alt-sigma 2.0`, `--gps-alt-min-pzz 0.01` (P_zz floor BEFORE K/S computation), `--gps-alt-max-res 30` (rejection gate: skip update when |res|>30m), `--gps-alt-relative` (bootstrap GPS and VIO references on first call, then delegate to absolute). `--gps-time-offset 0` needed when GPS CSV is pre-aligned to camera timestamps. Dataset loader: auto-detects WGS84 (lat,lon,alt) vs ENU xyz.

5. **jc82 drone stereo config** (`config/user_drone_stereo_jc82/`): Stereo fisheye calibration for the JC82 drone. `fi_max_baseline=100` prevents stereo starvation at high altitude.

## Code Style & Formatting

Run `./run_format.sh` (clang-format based) before committing C++ changes. Copyright headers must be preserved on all modified files.

## Flight Error Analysis

Use `analysis/full_flight_error_analysis.py` and follow
`analysis/FLIGHT_ERROR_ANALYSIS_SKILL.md` for GPS-referenced flight analysis.
Do not create a separate one-off metrics script or change alignment/metric
definitions silently. Single-run analysis is the primary workflow and belongs
under `C:/Users/baloney/Desktop/实验目录/<original_experiment_folder_name>/`.
Each run folder name must include date, flight, applied configuration, and
success/failure status, with `plots/`, `tables/`, `data/`, `reports/`, and
`metadata/` kept separate.
