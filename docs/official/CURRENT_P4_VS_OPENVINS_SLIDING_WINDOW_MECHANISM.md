# Current P4 versus OpenVINS sliding-window mechanism

## Purpose

This document compares only the finite-window, pruning, retry, and state-injection mechanisms of upstream OpenVINS with the current FC navigation + board IMU + monocular vision P4 initializer. It does not propose restoring the upstream visual-inertial initializer. The current P4 factor graph remains the formal algorithm base.

Reference revisions:

- upstream: `official-openvins/master`;
- fork working tree: `3d9296c34d19767b967e12d3ec8fbaf18d1ec1f3` plus the current uncommitted P4/P5 implementation.

## 1. How upstream OpenVINS maintains a finite initialization window

Upstream `InertialInitializer::initialize()` scans the shared `FeatureDatabase` for the newest camera timestamp and defines:

```text
oldest_time = newest_cam_time - init_window_time - 0.10
```

It then evaluates only the feature and IMU history remaining after that boundary. A failed call returns `false`; the next admitted image invokes initialization again against a later newest camera timestamp, so the finite window advances naturally with the live streams.

The dynamic path repeats the same construction without the additional 0.10 s feature margin:

```text
oldest_time = newest_cam_time - init_window_time
```

It copies the feature tracks used by the current attempt before building its linear systems and Ceres problem. The copy isolates the current solve from new tracker writes while keeping the live `FeatureDatabase` available to the frontend.

Source:

- `official-openvins/master:ov_init/src/init/InertialInitializer.cpp`, `InertialInitializer::initialize()`, lines 79–102;
- `official-openvins/master:ov_init/src/dynamic/DynamicInitializer.cpp`, `DynamicInitializer::initialize()`, lines 44–87.

## 2. How upstream prunes IMU and feature data outside the window

There are two coordinated pruning points.

First, `VioManager::feed_measurement_imu()` passes an `oldest_time` boundary into `InertialInitializer::feed_imu()`. Before VIO initialization, this boundary is derived from the configured initialization window and Camera–IMU time offset. `feed_imu()` appends the new sample and erases every older sample.

Second, each initialization attempt calls:

```text
FeatureDatabase::cleanup_measurements(oldest_time)
```

and erases IMU samples older than the camera boundary shifted by `calib_camimu_dt`. Dynamic initialization copies only the already-pruned feature tracks into attempt-local storage. Temporary states, landmarks, and Ceres arrays are freed before every return from the dynamic solve.

Source:

- `official-openvins/master:ov_init/src/init/InertialInitializer.cpp`, `feed_imu()` and `initialize()`, lines 55–76 and 96–102;
- `official-openvins/master:ov_init/src/dynamic/DynamicInitializer.cpp`, `initialize()`, lines 58–87 and its `free_state_memory()` return paths;
- `official-openvins/master:ov_core/src/feat/FeatureDatabase.cpp`, `cleanup_measurements()`, lines 226–243;
- relevant history: `eba3500` (bounded IMU lifecycle), `60ac071` (correct erasure iterator), and `4034281` (free dynamic-initializer allocations on failed returns).

## 3. How upstream waits and retries when the current window is insufficient

Insufficient data is represented by a normal `false` return, not a permanent initialization shutdown. Examples include:

- no complete camera window;
- too few features for disparity estimation;
- insufficient IMU coverage;
- too few valid feature tracks or poses;
- insufficient angular motion;
- linear-system failure;
- Ceres or covariance failure.

`VioManager::try_to_initialize()` clears only the asynchronous attempt queue after a failed solve, marks `thread_init_running=false`, and allows a later image to start another attempt. The live tracker and IMU streams continue to advance. No elapsed wall-time or stream-time threshold permanently disables upstream initialization.

Source:

- `official-openvins/master:ov_init/src/init/InertialInitializer.cpp`, `initialize()`, lines 91–155;
- `official-openvins/master:ov_init/src/dynamic/DynamicInitializer.cpp`, `initialize()`, failed precondition and solve returns throughout lines 44–1107;
- `official-openvins/master:ov_msckf/src/core/VioManagerHelper.cpp`, `try_to_initialize()`, lines 78–178.

## 4. How upstream stops after success and injects state, FEJ, and covariance

The upstream initializer writes the recovered IMU nominal state and FEJ reference in the successful static or dynamic path. `VioManager::try_to_initialize()` then:

1. writes the returned covariance with `StateHelper::set_initial_covariance()`;
2. sets `state->_timestamp` and `startup_time`;
3. removes feature measurements older than the initialized time;
4. restores the normal tracking feature count;
5. propagates queued camera timestamps after the initialization timestamp while keeping the clone count bounded;
6. sets `thread_init_success=true`.

The next camera call observes `thread_init_success` and transitions into normal VIO processing. Initialization is not invoked again after `is_initialized_vio` becomes true.

Source:

- `official-openvins/master:ov_init/src/static/StaticInitializer.cpp`, `StaticInitializer::initialize()`, nominal state and FEJ near the successful return;
- `official-openvins/master:ov_init/src/dynamic/DynamicInitializer.cpp`, `DynamicInitializer::initialize()`, state/FEJ at lines 934–946 and 1094;
- `official-openvins/master:ov_msckf/src/core/VioManagerHelper.cpp`, `try_to_initialize()`, lines 89–166;
- `official-openvins/master:ov_msckf/src/core/VioManager.cpp`, pre-initialization call and normal update transition near lines 301–341.

## 5. Where the pre-refactor current P4 differs

The current P4 already has its own finite deque pruning:

```text
keep_after = newest_timestamp
           - window_duration_s
           - buffer_margin_s
           - max_time_offset_s
```

but it has been tied to one fixed `window_duration_s`, configured as 8 s. It does not choose the shortest usable solve window from several bounded candidates.

The pre-refactor P4 also contains a stream-time shutdown:

```text
now - collection_start_time > navigation_max_collection_s
```

which closes the entire alignment attempt after 20 s under the practical policy. That behavior differs from upstream's persistent sequence of finite-window retries.

Each admitted initialization image currently calls `try_initialize()`. Cheap coverage gates avoid some solves, but there is no stable window fingerprint, no explicit minimum attempt interval, and no requirement for material information gain relative to the previous attempt.

The current P4 creates a successful `AlignmentResult` and immediately closes the window. It has no distinct fixed candidate plus bounded future-data validation interval before OpenVINS injection.

Initialization image admission is inherited from the fixed `args.cam_subsample` path, registered as stride12 in the current mainline. It is not selected from actual rotation-compensated parallax.

The pre-refactor P5 assigns fixed tracking/backend stride pairs directly from discrete flight states. Its overlap proxy is the one-dimensional expression `1 - speed * dt / footprint`, and backend eligibility is primarily a frame counter.

Current source locations before this refactor:

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`, `prune()`, lines 794–805;
- same file, `try_initialize()`, fixed window and timeout at lines 858–975;
- same file, immediate result and close at lines 1888–1974;
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`, fixed pre-initialization cadence at lines 3344–3371;
- `ov_msckf/src/ros_free/AdaptiveStrideController.h`, state targets and assignment at lines 44–63 and 641–680.

## 6. What must remain from the current P4

Only the upstream lifecycle mechanics are reused. The following current P4 content remains mandatory:

| Current P4 element | Required preservation |
| --- | --- |
| FC navigation observations | position, velocity, and attitude residual blocks remain active |
| board IMU | `CpiV1` preintegration factors remain in the joint solve |
| monocular vision | current `TrackKLT`, shared `FeatureDatabase`, multi-frame triangulation, and reprojection factors remain active |
| startup relation | startup FC-to-board orientation and time relation remain represented |
| navigation state | `q/p/v/bg/ba` remain solved jointly |
| uncertainty | 15×15 navigation covariance and mount covariance remain produced |
| observability | per-state factor support, covariance, information, and estimate status remain reported |
| state application | `AlignmentResult` continues to write nominal state, FEJ, timestamp, and covariance through `VioManager` |
| fallback discipline | no upstream visual-IMU-only or nearest-FC fallback is introduced |

The redesign therefore changes scheduling, window ownership, candidate validation, and lifecycle control around the current factor graph. It does not replace that graph.

## Mechanisms adopted from upstream

The redesign adopts these specific mechanisms:

1. derive a finite solve window from the latest usable sensor horizon;
2. prune live sensor and feature snapshots outside the maximum supported window plus interpolation margin;
3. treat insufficient current-window conditions as retryable;
4. isolate expensive attempt-local states, landmarks, and residual structures;
5. retain only lightweight statistics after a rejected attempt;
6. apply nominal state, FEJ, covariance, timestamp, and feature cleanup exactly once after success;
7. prevent the initializer from running after OpenVINS starts.

It does not adopt upstream static/dynamic initializer factors, state definition, local-frame output, or fallback selection.

