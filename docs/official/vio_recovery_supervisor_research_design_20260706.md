# VIO Recovery Supervisor Robustness Research And Design

Date: 2026-07-06

Scope: OpenVINS/MSCKF ROS-free flight replay, external-anchor initialization/restart, pose repair, visual health supervision, and adaptive stride.

## 0. Conclusion

The recovery design should be anchor-driven, not map-relocalization-driven.

This codebase is primarily an MSCKF/VIO estimator. It does not maintain a persistent visual map that can support ORB-SLAM-style BoW/MapPoint relocalization. Therefore, the recovery source must be an external absolute anchor:

1. Future main source: satellite-map / reference-map match.
2. Temporary medium-trust source: flight-controller navigation state.
3. Temporary low-trust source: GNSS coordinate / GPS course.
4. Offline diagnostic source: simulated GPS pose repair.

The proposed module should be named `VioRecoverySupervisor`, not `RobustSupervisor`. Its job is to decide whether VIO is healthy, degraded, unobservable, or lost, and whether an external anchor is trustworthy enough for EKF update, soft reset, hard reinitialization, or waiting. It is not a relocalizer and should not claim map relocalization without a map.

The adaptive stride policy should protect tracking continuity first. Large image parallax / optical flow means the camera input stride must go down toward 1, i.e. higher frame rate. Low parallax with healthy tracks should usually reduce backend update frequency or wait for baseline, not blindly drop raw tracking frames. The current code already has useful parts, but the roles of `AdaptiveStrideController`, `camera_frame_adaptive`, and `visual_update_adaptive` should be separated and then unified.

Implementation should not start with EKF-core changes. The first safe code slice is a shadow-mode contract: external-anchor source health, supervisor state/reason logging, and adaptive-stride decision logging. Active reset/reinit should remain disabled until logs prove that the gates do not confuse low-disparity, temporary visual degradation, and true estimator loss.

## 1. Local Code Review Findings

### 1.0 Repository operating constraints from `AGENTS.md` and `MEMORY.md`

The local project instructions materially constrain this design:

- Preserve dirty worktree changes. The current repository already has many modified estimator and baseline files, so the implementation must use small, reviewable patches.
- GPS horizontal, GPS course, truth/reference/error columns, stereo pseudo-reference, and future evaluation windows must not silently become online estimator inputs.
- Current locked baseline is `baseline/latest` with default stride 12 and no-thinning stride 1. Adaptive stride and recovery changes must compare against those controls.
- For adaptive-stride, repair, restart, or baseline changes, validation cannot stop at compilation. It needs the relevant fly/globalbaseline experiment unless runtime data/display is unavailable.
- `run_format.sh` and `run_copyright.sh` are not safe narrow tools for this work.
- Real-time visualization is expected by default for replay experiments; `--headless` is only for unavailable display or explicit override.

Design implication: all FC/GNSS/course-based behavior must be explicitly named as an experimental anchor source, must carry provenance, and must never masquerade as baseline evaluation truth.

### 1.1 Initialization and anchor data are under-modeled

Current one-shot FC initialization:

- `ov_msckf/src/core/FCInitLoader.h`
- `struct FCInitState` contains `timestamp`, `q_GtoI`, `v_IinG`, `p_IinG`, `bg`, `ba`, `source_dt`.
- It has no `source_type`, covariance, source confidence, PDOP/HDOP/VDOP, satellites, fix type, `eph/epv/sacc`, map-match score, inlier count, or validity bitmask.
- `load_fc_init_state_csv(path, target_time)` selects the nearest row by absolute time. That is acceptable for one-shot offline startup, but not safe for causal runtime restart unless wrapped by a provider that only returns current/past samples within a bounded age.

Current FC conversion:

- `tools/fc_to_init_csv.py` parses FC `satellites`, lat/lon/alt, velocity, roll/pitch/yaw, but outputs only the old one-shot init CSV.
- The header explicitly says it does not create a continuous GPS/FC fusion stream.
- This is the right place to add a future `ExternalAnchor` CSV export.

Current GPS loader:

- `ov_msckf/src/ros_free/DatasetReaderEuroc.h::GpsSample` only stores `timestamp` and `xyz`.
- `tools/fc_to_gps_csv.py` outputs `ts_ns,lat,lon,alt,Ve,Vn,Vu,satellites`, but the loader reads only the first four columns.
- Any PDOP/satellite/source confidence design requires extending `GpsSample` or creating a separate anchor-provider parser.

### 1.2 Pose-anchor reset already exists and should be reused

Useful existing primitives:

- `VioManager::feed_measurement_pose_anchor(...)`
  - EKF position/yaw update.
  - Time alignment check.
  - Innovation/NIS gate ratio.
  - Predicted correction trust-region gate.
  - Proper diagnostic fields via `PoseAnchorLastUpdate`.
- `VioManager::apply_trusted_pose_anchor_reset(...)`
  - Applies global translation and yaw delta to IMU state, FEJ, clones, and global SLAM landmarks.
  - Can reset velocity and biases.
  - Resets GPS altitude bootstrap.

This is a good fit for trusted satellite-map anchors. It should not be driven by unqualified nearest-GPS/course values by default.

### 1.3 Current restart path is useful but too permissive

In `ov_msckf/src/run_serial_msckf_ros_free.cpp`:

- `--pose-repair-sim-gps` simulates sparse pose repair using GPS ENU + course yaw.
- `--restart-supervisor` currently allows hard restart on state health fault and optionally pose-repair failure.
- Hard restart builds a new `VioManager`.
- If no trusted anchor override exists, `restart_with_gps_init` can construct `FCInitState` from nearest GPS/course and initialize with source `"nearest_gps_course"`.

Required change:

- Keep this as an explicit diagnostic fallback only, e.g. `--allow-diagnostic-gnss-restart`.
- Production/default restart must require an `ExternalAnchorMeasurement` passing `AnchorTrustPolicy`.

### 1.4 Current visual-loss detection is a seed, not a full supervisor

Current heuristic uses:

- KLT raw/tracked count.
- `n_accepted`, `n_chi2_rejected`.
- SLAM feature collapse.
- A `visual_lost_credit` counter.

This should become a windowed `VisualHealth` score with reasons:

- `LOW_TRACK_COUNT`
- `LOW_TRACK_RATIO`
- `RANSAC_EMPTY`
- `LOW_GRID_COVERAGE`
- `HIGH_NEW_LOST_RATIO`
- `LOW_MSCKF_ACCEPTED`
- `HIGH_CHI2_REJECT_RATIO`
- `TRIANGULATION_STARVATION`
- `BAD_CONDITION_NUMBER`
- `STATE_COV_FAULT`

One frame should not trigger hard restart. Repeated visual/backend/state failures should move through degraded states, and hard reinit should wait for an external anchor.

### 1.5 Adaptive stride has the right signal but the wrong ownership boundary

Current `AdaptiveStrideController` already uses:

- causal height fallback;
- speed/gyro/roll/pitch;
- feature count;
- median parallax from `TrackerWarpVizPacket`;
- hysteresis and hold times.

Problems:

- `--adaptive-stride` drops raw camera frames before the tracker. If a frame is skipped, there is no fresh KLT packet, so the controller reacts late.
- `camera_frame_adaptive` separately uses depth/speed geometry.
- `visual_update_adaptive` separately gates backend visual updates based on flow/time/track safety.

Design direction:

- Keep raw KLT tracking near full rate when possible.
- Use adaptive policy primarily to choose backend update/keyframe/clone cadence.
- Only drop raw camera frames when compute budget requires it, and force `input_stride=1` on high parallax, high gyro, low tracks, or visual degraded state.

### 1.6 Current implementation cut points

The safest implementation cut points are:

- `ov_msckf/src/core/FCInitLoader.h`: keep `FCInitState`, add conversion to a richer anchor only after introducing a new header.
- `ov_msckf/src/ros_free/DatasetReaderEuroc.h`: do not overload `GpsSample` with future map-match semantics; either extend it only for GNSS quality fields or create a separate anchor CSV parser.
- `ov_msckf/src/core/VioManager.{h,cpp}`: reuse existing pose-anchor update/reset primitives; do not add supervisor state logic inside EKF update code unless needed for diagnostics.
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`: currently owns CLI, replay timing, pose-repair simulation, restart supervisor, adaptive input-frame dropping, and policy logs. It is the right first integration point for shadow supervisor logging, but should not grow into a large monolithic policy engine.
- `ov_msckf/src/ros_free/AdaptiveStrideController.h`: should own causal stride policy math, not external-anchor recovery policy.
- `ov_msckf/src/ros_free/DiagLogger.h`: already documents per-frame KLT/MSCKF signals; extend this style for supervisor and anchor status logs.

## 2. Research Taxonomy

Each external trick is classified as one of:

- `Direct`: can be implemented with nearly the same mechanism.
- `Adapt`: small/moderate changes make it suitable for our MSCKF codebase.
- `Borrow`: borrow the design idea, not the mechanism.
- `Reject`: not suitable because it requires persistent map points, BoW database, pose graph, dense map, or a different estimator architecture.

Important: `Reject` does not mean the source is useless. It means the exact mechanism should not be copied. Its state machine, diagnostics, gates, and fallback patterns may still be useful.

### 2.1 Cross-source adaptation matrix

| Trick | Source examples | Classification | Concrete repo adaptation |
|---|---|---|---|
| Per-source aid status with observation, variance, innovation, test ratio, fused/rejected, last-fuse time | PX4 EKF2, ArduPilot EKF3, robot_localization | `Direct` | Add `AidSourceStatus` for external anchors and pose repair. Log covariance-in, covariance-used, residual, NIS/test-ratio, gate, accepted, reason, timeout, fail/pass streak. |
| Continuous source-health windows before accepting aid | PX4 GNSS checks, ArduPilot GPS health | `Direct` | Require `min_anchor_health_time_s` or N consecutive good anchors before init/restart. Single sparse sample can log but cannot hard reinit by default. |
| Innovation gates and covariance floors | PX4, ArduPilot, robot_localization, OpenVINS | `Direct` | Reuse pose-anchor NIS gate. Add physical covariance floors and reject negative/near-zero/unreasonably confident anchor covariance. |
| Reset delta logging and reset covariance treatment | ArduPilot EKF3, PX4 EKF2 | `Adapt` | Log yaw/position/velocity/bias reset deltas. For soft reset, rely on `apply_trusted_pose_anchor_reset`; for hard reinit, write old/new source provenance. |
| Recently-lost grace window before full lost | ORB-SLAM3, stella_vslam, VINS-Mono | `Adapt` | Add `VISUAL_DEGRADED` and `LOST_WAIT_ANCHOR` hysteresis. Use IMU propagation and full-rate tracking before declaring hard lost. |
| Recovery settle window and stricter post-recovery acceptance | ORB-SLAM3, stella_vslam | `Adapt` | After soft reset/hard reinit, force full-rate tracking and require better visual/backend health before normal visual updates. |
| Frontend fallback cascade | VINS-Fusion, stella_vslam, DSO | `Adapt` | Try IMU/gyro predicted LK, normal LK, wider search/redetect, and geometry check before escalating visual state. |
| Forward-backward or reverse consistency checks | VINS-Fusion, LK pipelines | `Adapt` | Add as visual-health diagnostics first; avoid changing estimator input until baseline comparison. |
| Low-disparity as a typed status, not lost | Kimera-VIO, OpenVINS ZUPT/disparity logic | `Direct` | Treat as `VIO_UNOBSERVABLE`: increase tracking density or wait; do not hard restart from low disparity alone. |
| Keyframe/intermediate-frame split | OKVIS, Kimera-VIO, ORB-SLAM3 | `Adapt` | Separate raw tracking cadence from backend visual-update cadence. Use full-rate tracking with adaptive backend updates. |
| Feature lifecycle quality and spatial coverage | ROVIO, OKVIS, Kimera-VIO | `Adapt` | Add visual-health inputs: track age, KLT success ratio, grid occupancy, p95 flow, feature score/coverage when available. |
| GPS/global pose graph as a separate global layer | VINS-Fusion | `Borrow` | Keep future GNSS/global smoothing separate from online baseline; do not hide it inside the MSCKF path. |
| Direct photometric residual health | ROVIO, DSO, LSD-SLAM | `Borrow` | Use image quality/gradient/blur/photometric diagnostics for supervisor only, not as MSCKF residuals. |
| BoW/MapPoint/PnP relocalization | ORB-SLAM3, stella_vslam, VINS-Mono pose graph, LSD-SLAM | `Reject` exact mechanism | Current MSCKF has no persistent map. Replacement is external satellite/reference-map anchor. |
| Sim(3) keyframe graph / loop closure / Atlas map creation | ORB-SLAM3, LSD-SLAM, Kimera-PGO | `Reject` exact mechanism | Not part of this recovery pass; would be a different mapping/global-optimization subsystem. |

## 3. External Source Review

### 3.1 PX4 EKF2 / ECL

Sources:

- PX4 EKF tuning docs: https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf
- PX4-ECL GPS checks: https://github.com/PX4/PX4-ECL/blob/master/EKF/gps_checks.cpp

Engineering tricks:

- GNSS data must pass quality checks before being used: fix type, satellites, PDOP, horizontal/vertical accuracy, speed accuracy, drift, and speed checks.
- Checks must remain continuously healthy for a time window before the source is accepted.
- Innovation test ratios distinguish reject/fuse behavior.
- Aid-source diagnostics expose observation, observation variance, innovation, innovation variance, test ratio, innovation rejected, fused flag, and time of last fuse.
- Fault handling distinguishes automatic mode and dead-reckoning mode.
- Source reset is conditional on whether alternative aiding exists.

Applicability:

- `Direct`: NIS/gate ratio logging for pose anchor and GPS-Z.
- `Adapt`: external anchor health must require continuous pass windows, not single samples.
- `Adapt`: source quality fields should map into covariance/gates, not only boolean acceptance.
- `Borrow`: reset/reject/fuse mode split.

MSCKF caution:

- PX4 fuses multiple navigation sources as normal estimator inputs. This repo currently must not silently fuse GPS XY or course as online truth. Horizontal GNSS use must remain explicit and logged.

### 3.2 ArduPilot EKF3

Sources:

- EKF overview: https://ardupilot.org/dev/docs/extended-kalman-filter.html
- EKF3 parameters/code: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3.cpp
- GPS measurement handling: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp
- reset functions: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp

Engineering tricks:

- GPS-reported speed/position/height accuracy is envelope-filtered and clamped before being used.
- Satellite count scales GPS noise: fewer satellites increase noise instead of immediate hard failure.
- Position, velocity, and height innovations have separate test ratios.
- It separates the observation noise used for Kalman gain from the observation noise used for data consistency checks.
- Source mode transitions distinguish `AID_NONE`, `AID_RELATIVE`, and `AID_ABSOLUTE`.
- Reset functions record reset delta and reset matching covariance blocks.
- Position reset compensates source delay using velocity and constrains timestamp age.
- Glitch handling can temporarily reject, clip, or gradually realize persistent offsets.

Applicability:

- `Direct`: record reset deltas and reset reason in CSV.
- `Adapt`: map `satellites/eph/epv/sacc/pdop` into anchor covariance inflation.
- `Adapt`: bounded-age anchor with timestamp compensation.
- `Adapt`: use separate covariance floors for fusion strength and data-consistency checks, so a bad source cannot pass merely by reporting very large uncertainty.
- `Borrow`: persistent offset/glitch handling for low-trust GNSS fallback.

MSCKF caution:

- ArduPilot has a full multi-source navigation architecture. Our baseline contract does not allow hidden GPS XY fusion.

### 3.3 robot_localization

Sources:

- docs: https://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html
- EKF source: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/ekf.cpp
- filter base: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/src/filter_base.cpp

Engineering tricks:

- Every measurement carries covariance.
- Negative covariance is sanitized; near-zero covariance is floored.
- Mahalanobis threshold gates measurements.
- Joseph-form covariance update is used.
- Dynamic process noise can scale with motion.
- Sensor timeout is explicit.
- Global jumping sources are normally separated from continuous odometry through frame semantics.

Applicability:

- `Direct`: covariance floor for external anchors.
- `Direct`: anchor Mahalanobis/NIS gate and logging.
- `Adapt`: dynamic uncertainty inflation during high-speed/high-gyro degraded states.

MSCKF caution:

- robot_localization is a generic fusion filter; it does not solve visual observability or feature tracking health.
- Its timeout behavior is predict-only, not automatic restart. This is a useful boundary for our supervisor.

### 3.4 VINS-Mono

Sources:

- repo: https://github.com/HKUST-Aerial-Robotics/VINS-Mono
- estimator source: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator.cpp
- node source: https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/estimator_node.cpp
- paper text: https://ar5iv.labs.arxiv.org/html/1708.03852

Engineering tricks:

- Failure detector checks feature count, accelerometer bias, gyro bias, position jump, z jump, and rotation jump.
- Failure can trigger `clearState()` and `setParameter()` reboot.
- ROS node has an explicit restart callback that clears IMU/feature buffers and resets estimator state.
- Image timestamp discontinuity can reset only the frontend tracker and publish a restart signal.
- Image publication frequency is controlled separately from estimator failure logic.
- Relocalization is a separate callback fed by pose-graph matched points.

Applicability:

- `Direct`: estimator-state sanity checks for bias, pose jump, and state finite.
- `Adapt`: explicit restart action should clear old features/buffers and start with a fresh anchor.
- `Borrow`: separate failure detector from recovery action.

MSCKF caution:

- VINS relocalization depends on pose graph / map matches. For our MSCKF, this role should be filled only by future satellite-map anchors.

### 3.5 VINS-Fusion

Sources:

- repo: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- estimator source: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/vins_estimator/src/estimator/estimator.cpp
- global fusion: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/master/global_fusion/src/globalOpt.cpp

Engineering tricks:

- Maintains failure detection/reboot structure, although upstream code disables parts with an early `return false` in some versions.
- Frontend uses predicted optical-flow initial points; if too few LK tracks succeed, it falls back to normal LK.
- Frontend uses forward-backward consistency checks, and stereo uses reverse left-right checks.
- Global fusion keeps VIO and GPS in a separate optimization layer, preserving the VIO estimator boundary.
- GPS factor uses reported position accuracy as weight.

Applicability:

- `Adapt`: keep FC/GNSS/satellite-map anchor logic as an outer layer before deciding reset.
- `Borrow`: separate local VIO from global alignment/fusion.

MSCKF caution:

- GPS global optimization is not the same as online EKF GPS XY fusion. If implemented, it should remain a separate mode with provenance.

### 3.6 ORB-SLAM3

Sources:

- repo: https://github.com/UZ-SLAMLab/ORB_SLAM3
- tracking source: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc
- paper: https://ar5iv.labs.arxiv.org/html/2007.11898

Engineering tricks:

- Tracking state machine: `OK`, `RECENTLY_LOST`, `LOST`.
- In inertial mode, a short recently-lost interval can continue IMU prediction before declaring full lost.
- If tracking is lost and enough map exists, Atlas starts a new map instead of corrupting the old one.
- Relocalization uses BoW candidates, PnP RANSAC, pose optimization, projection search, and inlier thresholds.
- Tracking success checks inlier counts and uses stricter thresholds after relocalization.
- Weak tracking influences keyframe insertion; after relocalization, keyframe insertion is temporarily constrained.

Applicability:

- `Direct`: `RECENTLY_LOST` / `LOST_WAIT_ANCHOR` hysteresis and cooldown.
- `Adapt`: do IMU-only/propagation grace window while waiting for external anchor.
- `Borrow`: do not corrupt current estimator with bad visual updates during lost state.
- `Reject`: BoW/MapPoint/PnP relocalization without a persistent map.

MSCKF caution:

- Our replacement for ORB-SLAM relocalization is satellite-map/reference-map anchor, not internal MapPoint matching.

### 3.7 stella_vslam / OpenVSLAM lineage

Sources:

- repo: https://github.com/stella-cv/stella_vslam
- tracking source: https://github.com/stella-cv/stella_vslam/blob/main/src/stella_vslam/tracking_module.cc

Engineering tricks:

- Configurable relocalization by pose request.
- Tracking tries motion model, BoW match, then robust matching.
- Tracking lost soon after initialization triggers reset.
- Local-map tracking has a fallback mode that can ignore temporal keyframes.
- Frame statistics track failed frames.
- Recovery after relocalization must still pass local-map tracking.
- Post-recovery thresholds and keyframe insertion are stricter for a short window.

Applicability:

- `Direct`: early-post-init loss should reset faster than mature-flight degradation.
- `Adapt`: multi-attempt tracking cascade can map to KLT predicted-rotation, raw LK, wider-window LK, and feature redetect before declaring degraded.
- `Borrow`: explicit external pose-request relocalization is similar to future satellite-map anchor request.
- `Reject`: BoW/local-map landmark relocalization.

### 3.8 DSO

Sources:

- repo: https://github.com/JakobEngel/dso
- README: https://github.com/JakobEngel/dso/blob/master/README.md
- paper: https://jakobengel.github.io/pdf/DSO.pdf

Engineering tricks:

- Coarse tracking tries many pose/light initializations and start pyramid levels.
- Tracks residual per pyramid level and aborts bad hypotheses early.
- Logs coarse-tracking residuals.
- Explicitly documents that pure VO cannot recover by relocalization and fails under large rotation without translation.
- Runtime presets trade point count and realtime constraints.

Applicability:

- `Adapt`: try multiple KLT/reprojection prediction modes before declaring visual lost.
- `Adapt`: per-level residual and flow diagnostics for `VisualHealth`.
- `Borrow`: explicit no-map/no-relocalization limitation should be documented for our supervisor.

MSCKF caution:

- DSO is direct photometric VO. Photometric residuals cannot be blindly used as MSCKF residuals.

### 3.9 LSD-SLAM

Sources:

- repo: https://github.com/tum-vision/lsd_slam
- README: https://github.com/tum-vision/lsd_slam/blob/master/README.md
- paper: https://jakobengel.github.io/pdf/engel14eccv.pdf

Engineering tricks:

- Has explicit manual reset and manual lost trigger.
- Tracking lost starts a relocalizer when mapping is disabled or tracking moves outside known map.
- Relocalization candidate is re-tracked and rejected if good/bad pixel ratio is too poor.
- User-facing parameters include pixel noise, gradient threshold, keyframe thresholds, relocalization threshold, and tracking plots.
- Viewer resets graph automatically.

Applicability:

- `Direct`: manual/explicit recovery commands and diagnostic plotting.
- `Adapt`: accept recovery only after a secondary tracking-quality check.
- `Adapt`: when quality is poor, increase keyframe/update density rather than tuning EKF noise first.
- `Reject`: Sim(3) keyframe graph and relocalizer without a map.

### 3.10 ROVIO

Sources:

- repo: https://github.com/ethz-asl/rovio
- paper: https://www.research-collection.ethz.ch/handle/20.500.11850/120932
- image update source: https://github.com/ethz-asl/rovio/blob/master/include/rovio/ImgUpdate.hpp
- feature manager: https://github.com/ethz-asl/rovio/blob/master/include/rovio/FeatureManager.hpp

Engineering tricks:

- Multi-level patch features carry validity and Shi-Tomasi score.
- Patch alignment has Huber threshold and rejection threshold.
- Image update has outlier detection and Mahalanobis threshold.
- Feature manager balances feature quality and distance from existing features.
- Feature covariance and validity are explicit lifecycle variables.

Applicability:

- `Adapt`: add KLT/patch-quality fields to `TrackerWarpVizPacket`.
- `Adapt`: use feature score distribution and grid coverage in visual-health scoring.
- `Borrow`: feature lifecycle health, not only current count.

MSCKF caution:

- Do not convert the estimator into direct photometric EKF. Use the quality signals outside the update.

### 3.11 OKVIS

Sources:

- repo: https://github.com/ethz-asl/okvis
- paper: https://www.doc.ic.ac.uk/~sleutene/publications/ijrr2014_revision_1.pdf
- estimator source: https://github.com/ethz-asl/okvis/blob/master/okvis_ceres/src/Estimator.cpp

Engineering tricks:

- Keyframe-based fixed-lag nonlinear optimization.
- IMU initialization for first pose.
- Landmark quality computed from Hessian eigenvalue ratio.
- Marginalization/drop strategy separates keyframes and recent IMU frames.
- Optimization time limit and minimum iteration controls.

Applicability:

- `Adapt`: quality from conditioning/eigenvalue ratio maps well to MSCKF triangulation condition diagnostics.
- `Adapt`: time budget should influence adaptive backend update cadence.
- `Borrow`: separate keyframe and non-keyframe frame roles.

MSCKF caution:

- OKVIS landmarks and graph optimization are not the current estimator structure.

### 3.12 Kimera-VIO

Sources:

- repo: https://github.com/MIT-SPARK/Kimera-VIO
- paper: https://arxiv.org/pdf/1910.02490.pdf
- tracker source: https://github.com/MIT-SPARK/Kimera-VIO/blob/master/src/frontend/Tracker.cpp
- pipeline source: https://github.com/MIT-SPARK/Kimera-VIO/blob/master/src/pipeline/Pipeline.cpp

Engineering tricks:

- Frontend returns `TrackingStatus`: valid, invalid, few matches, low disparity.
- Mono/stereo geometric verification uses RANSAC.
- IMU rotation can be used in mono 2-point / stereo 1-point variants.
- Pipeline exposes queue status, frontend status, and restart of frontend workers/queues.
- Debug info logs putatives, inliers, RANSAC timing, and feature-tracking time.

Applicability:

- `Direct`: introduce typed visual status instead of one boolean.
- `Adapt`: `LOW_DISPARITY` should mean unobservable/starved, not immediately lost.
- `Adapt`: frontend queue/runtime health should be logged separately from estimator health.
- `Borrow`: restart frontend workers/queues separately from restarting estimator.

MSCKF caution:

- Smart factors, PGO, and loop-closure consistency checks are not direct replacements for MSCKF update health. Use the status vocabulary and diagnostics, not the graph backend.

### 3.13 OpenVINS / upstream MSCKF

Sources:

- docs: https://docs.openvins.com/
- FEJ docs: https://docs.openvins.com/fej.html
- update nullspace: https://docs.openvins.com/update-null.html
- feature init: https://docs.openvins.com/update-featinit.html
- zero velocity update: https://docs.openvins.com/update-zerovelocity.html

Engineering tricks:

- FEJ is central to consistency.
- MSCKF update marginalizes features through nullspace projection.
- Feature initialization already has geometry gates.
- ZUPT combines inertial and visual disparity logic.
- Delayed initialization and clone/window structure are explicit.

Applicability:

- `Direct`: supervisor should stay outside EKF core where possible.
- `Direct`: use `MsckfLastStats`, `VisualFlowSummary`, `VisualUpdateCounters`, covariance health, and feature initializer stats as first-class inputs.
- `Adapt`: external anchor reset must transform FEJ and clones consistently, as current trusted reset does.

MSCKF caution:

- Do not change EKF noise or feature gates solely to improve one flight metric. Use health gating and external anchors first.

## 4. Proposed Architecture

### 4.1 New data contract: `ExternalAnchorMeasurement`

Add a new struct separate from legacy `FCInitState`:

```cpp
enum class ExternalAnchorSource {
  SATELLITE_MAP_MATCH,
  FC_NAV,
  GNSS_COORDINATE,
  SIM_GPS_DIAGNOSTIC,
  MANUAL
};

enum ExternalAnchorField : uint32_t {
  HAS_POSITION = 1u << 0,
  HAS_YAW      = 1u << 1,
  HAS_ATTITUDE = 1u << 2,
  HAS_VELOCITY = 1u << 3,
  HAS_BIAS     = 1u << 4
};

struct ExternalAnchorMeasurement {
  double timestamp = -1.0;
  ExternalAnchorSource source = ExternalAnchorSource::GNSS_COORDINATE;
  uint32_t valid_fields = 0;

  Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector4d q_GtoI = Eigen::Vector4d(0, 0, 0, 1);
  Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
  Eigen::Vector3d bg = Eigen::Vector3d::Zero();
  Eigen::Vector3d ba = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity();
  double yaw_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d R_vel = Eigen::Matrix3d::Identity();

  double confidence = std::numeric_limits<double>::quiet_NaN();
  double match_score = std::numeric_limits<double>::quiet_NaN();
  int inlier_count = -1;
  double pdop = std::numeric_limits<double>::quiet_NaN();
  double hdop = std::numeric_limits<double>::quiet_NaN();
  double vdop = std::numeric_limits<double>::quiet_NaN();
  double eph = std::numeric_limits<double>::quiet_NaN();
  double epv = std::numeric_limits<double>::quiet_NaN();
  double sacc = std::numeric_limits<double>::quiet_NaN();
  int satellites = -1;
  int fix_type = -1;
};
```

Legacy `FCInitState` can remain for compatibility, but new code should convert it into `ExternalAnchorMeasurement` with explicit `source=FC_NAV`.

### 4.2 Anchor providers

Add providers behind a common interface:

- `SatelliteMapAnchorProvider` future placeholder.
- `FcNavAnchorProvider` for FC logs.
- `GnssAnchorProvider` for GPS CSV.
- `SimGpsDiagnosticAnchorProvider` for current pose-repair simulation.

Provider rule:

- Runtime provider must be causal: latest sample at or before `t_cam`, bounded by `max_anchor_age_s`.
- Nearest future sample is only allowed under an explicit diagnostic flag.

### 4.3 Anchor trust policy

`AnchorTrustPolicy` maps source + quality into an action:

```cpp
enum class AnchorAction {
  REJECT,
  LOG_ONLY,
  EKF_UPDATE,
  TRUSTED_SOFT_RESET,
  HARD_REINIT
};
```

Proposed defaults:

| Source | Default trust | Position | Yaw | Velocity | Allowed action |
|---|---:|---|---|---|---|
| satellite map match | high | covariance from match score/inliers | if match provides heading | if provided | update, soft reset, hard reinit |
| FC nav | medium | covariance from FC/GNSS quality | FC yaw or course only if quality passes | yes if fresh | init, update, limited reinit |
| GNSS coordinate | low | inflated covariance | course yaw only if speed and sacc pass | yes if raw velocity exists | log/update/fallback reinit only with explicit flag |
| sim GPS | diagnostic | configurable | course if configured | yes | offline only |

Quality inflation examples:

- If `pdop` is high, inflate horizontal covariance.
- If `satellites < 6`, inflate or reject depending source.
- If `eph/epv/sacc` exist, use them as lower bounds.
- If source confidence exists, never reduce covariance below a configured physical minimum.
- If yaw comes from course, require speed above threshold and `sacc` below threshold.

### 4.3.1 Aid-source status log contract

Every external anchor source should produce a status row, whether or not it is used:

```cpp
struct AidSourceStatus {
  double sample_time = -1.0;
  double filter_time = -1.0;
  double age_s = std::numeric_limits<double>::infinity();
  ExternalAnchorSource source = ExternalAnchorSource::GNSS_COORDINATE;
  AnchorAction recommended_action = AnchorAction::REJECT;

  double source_confidence = std::numeric_limits<double>::quiet_NaN();
  Eigen::Matrix3d covariance_in = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d covariance_used = Eigen::Matrix3d::Identity();
  Eigen::Vector3d residual_pos = Eigen::Vector3d::Zero();
  double residual_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double nis_pos = std::numeric_limits<double>::quiet_NaN();
  double test_ratio = std::numeric_limits<double>::quiet_NaN();

  bool quality_pass = false;
  bool time_pass = false;
  bool innovation_pass = false;
  bool trust_region_pass = false;
  bool accepted = false;
  int consecutive_pass = 0;
  int consecutive_fail = 0;
  std::string reject_reason;
};
```

CSV fields should be stable and explicit:

```text
timestamp,source,age_s,action,quality_pass,time_pass,innovation_pass,trust_region_pass,
confidence,pdop,hdop,vdop,eph,epv,sacc,satellites,fix_type,
pos_sigma_x,pos_sigma_y,pos_sigma_z,yaw_sigma_rad,
residual_x,residual_y,residual_z,residual_yaw_rad,nis,test_ratio,
pass_streak,fail_streak,accepted,reject_reason
```

This mirrors PX4/ArduPilot aid-source diagnostics and makes later flight reports auditable.

### 4.3.2 Decision table

| Condition | Action | Reason |
|---|---|---|
| Anchor stale, missing covariance, invalid source quality, or future timestamp without diagnostic flag | `REJECT` | Do not let loader artifacts trigger recovery. |
| Anchor plausible but low trust while VIO is healthy | `LOG_ONLY` | Useful diagnostic, not enough to perturb estimator. |
| Anchor high/medium trust, VIO healthy or mildly degraded, innovation pass | `EKF_UPDATE` | Normal bounded correction through existing pose-anchor path. |
| Anchor high trust, VIO state structurally valid, repeated visual/backend failure, correction within reset limit | `TRUSTED_SOFT_RESET` | Preserve estimator object while realigning global pose. |
| Anchor high trust, state/covariance non-finite or soft reset repeatedly failed | `HARD_REINIT` | Build a new estimator from a qualified initialization source. |
| VIO lost but no qualified anchor | `REJECT` plus `LOST_WAIT_ANCHOR` supervisor state | No map means no internal relocalization. |

Reset rule: a sample used for reset/reinit must not also be fused as a normal pose update in the same frame. Log it as `RESET_USED_SAMPLE`.

### 4.4 `VioRecoverySupervisor` inputs

Snapshot inputs:

- Visual frontend:
  - tracked count
  - attempted count
  - KLT success ratio
  - RANSAC empty/fail
  - grid coverage
  - new/lost track ratio
  - median/p95 flow and parallax
  - feature age distribution
- Backend:
  - MSCKF `n_features_in`, `n_accepted`, `n_chi2_rejected`
  - triangulation failures by reason
  - condition number sums/max
  - GPS-Z residual/NIS/reject count
  - pose-anchor NIS/trust-region result
- State:
  - finite state
  - finite covariance
  - covariance symmetry/diagonal/min eigen diagnostics
  - bias norms
  - pose/velocity jumps
- Runtime:
  - camera gap
  - IMU gap
  - processing time / queue depth
  - image read failures
- Anchor:
  - provider age
  - source type
  - confidence fields
  - trust-policy result

### 4.4.1 Visual status vocabulary

Visual status should be typed:

- `VISUAL_OK`: adequate tracks, distribution, and backend acceptance.
- `VISUAL_DEGRADED_LOW_TRACKS`: KLT/RANSAC tracks are below threshold.
- `VISUAL_DEGRADED_BAD_DISTRIBUTION`: tracks exist but coverage/grid occupancy is poor.
- `VISUAL_DEGRADED_HIGH_FLOW`: flow/parallax is too large for current stride.
- `VISUAL_UNOBSERVABLE_LOW_DISPARITY`: low parallax/poor conditioning, not a lost condition by itself.
- `VISUAL_BACKEND_REJECTING`: frontend tracks exist but MSCKF chi2/triangulation rejects dominate.
- `VISUAL_RECENTLY_LOST`: temporary failure inside grace window.
- `VISUAL_LOST`: repeated frontend/backend failure and state no longer trusted for normal visual updates.

This avoids the current trap where low disparity, feature starvation, large flow, and true tracking loss can all increment one generic `visual_lost_credit`.

### 4.5 Supervisor states

```text
OK
  -> VISUAL_DEGRADED
  -> VIO_UNOBSERVABLE
  -> LOST_WAIT_ANCHOR
  -> ANCHOR_AVAILABLE
  -> SOFT_ANCHOR_RESET
  -> HARD_REINIT
  -> COOLDOWN
  -> FAILED_NO_ANCHOR
```

State meanings:

- `VISUAL_DEGRADED`: tracking or backend is worsening, but state is still plausible.
- `VIO_UNOBSERVABLE`: low disparity / poor geometry / feature starvation. This is not the same as lost.
- `LOST_WAIT_ANCHOR`: VIO prior is no longer trusted enough for normal updates; wait for trusted external anchor.
- `ANCHOR_AVAILABLE`: external anchor has passed trust policy.
- `SOFT_ANCHOR_RESET`: use trusted global reset if current state is structurally valid and anchor is high trust.
- `HARD_REINIT`: create a new `VioManager` only when state is bad or lost and anchor is sufficient.
- `FAILED_NO_ANCHOR`: no recovery possible without an external anchor.

Transition principles:

- One bad frame cannot leave `OK`.
- Low disparity moves toward `VIO_UNOBSERVABLE`, not `HARD_REINIT`.
- Large parallax moves first to `INCREASE_FRONTEND_RATE`.
- Hard restart requires either state/covariance structural fault or repeated lost state plus a trusted anchor.
- Recovery goes through `COOLDOWN`; old tracks/clones are not treated as continuing across hard reinit.

### 4.6 Recovery actions

Actions should be escalating:

1. `LOG_ONLY`: shadow mode.
2. `INCREASE_FRONTEND_RATE`: force camera input stride to 1.
3. `LIMIT_BACKEND_UPDATES`: avoid injecting bad low-quality visual updates.
4. `VISUAL_SETTLE_WINDOW`: after reinit/reset, track for a short time before normal updates.
5. `EKF_POSE_ANCHOR_UPDATE`: current `feed_measurement_pose_anchor`.
6. `TRUSTED_SOFT_RESET`: current `apply_trusted_pose_anchor_reset`.
7. `HARD_REINIT`: new `VioManager`, initialized from trusted anchor.
8. `FAIL_STOP`: diagnostic status if no anchor is available.

Hard reinit requirements:

- trusted anchor passed;
- cooldown elapsed;
- old visual tracks are not carried across;
- old clones are not carried across;
- provenance row written with source/confidence/covariance/action/reason;
- GPS-Z bootstrap reset.

### 4.7 Shadow-first implementation slice

First code patch should add behavior-free observability:

- `ExternalAnchorMeasurement` and source enums in a small header.
- `AnchorTrustPolicy` with pure functions and unit tests.
- Supervisor shadow CSV emitted from the runner.
- No automatic pose update, reset, or restart path changed by default.
- Existing CLI behavior remains unchanged unless new `--recovery-supervisor-shadow` or equivalent flag is set.

Second patch can wire existing pose-repair simulation through the anchor interface while preserving its diagnostic-only identity.

Third patch can gate hard restart so that unqualified nearest-GPS/course restart requires an explicit diagnostic flag.

## 5. Adaptive Stride Design

### 5.1 Principle

User rule: larger parallax requires higher frame rate.

Interpretation:

- Large per-frame optical flow means KLT may fail if frames are skipped. Input stride must decrease.
- Low parallax alone is not necessarily bad; it may mean straight high-altitude flight. If tracks are healthy, backend can wait for baseline. If tracks are starved and triangulation condition is bad, force full-rate tracking and adjust backend/keyframe policy.
- High gyro/roll/pitch means rotation-induced flow. Use gyro-aided KLT or warp, and force smaller input stride.

### 5.2 Split two stride concepts

Use two separate outputs:

```cpp
struct FrameRateDecision {
  int input_frame_stride;        // raw camera frames fed to tracker
  int visual_update_stride;      // backend update/keyframe cadence
  bool force_fullrate_tracking;
  bool suppress_visual_update;
  std::string reason;
};
```

Rules:

- `input_frame_stride = 1` for:
  - high median/p95 parallax
  - low track count
  - RANSAC empty/recent failed tracking
  - restart cooldown / visual settle
- High gyro or severe roll/pitch alone should cap stride and strengthen tracking prediction, but should not force full-rate input unless accompanied by high image parallax, low tracks, or recent tracking failures. A fly3 check on 2026-07-06 showed that turn-only full-rate feeding can over-update rotation-dominated frames and worsen XY error.
- `visual_update_stride` can increase when:
  - low parallax but stable tracks
  - sufficient track count
  - backend recently accepted good update
  - CPU budget is exceeded
- `suppress_visual_update` when:
  - visual residuals are inconsistent
  - low disparity makes update underconstrained
  - backend chi2 reject ratio is high

### 5.3 Use existing data

Existing sources to preserve:

- `median_tracker_parallax_px(...)` in runner.
- `TrackerWarpVizPacket` from KLT.
- `VioManager::VisualFlowSummary`.
- `VioManager::VisualUpdateCounters`.
- `VioManager::MsckfLastStats`.

Needed additions:

- KLT attempted/accepted ratio per frame.
- RANSAC fail/empty flag in packet.
- grid occupancy / quadrant coverage.
- p95 flow, not only median.
- backend update reason in one unified stride CSV.

### 5.3.1 Immediate adaptive-stride code changes

The current `AdaptiveStrideController` can be improved without waiting for the full recovery supervisor:

- Add an explicit decision output reason for high parallax forcing lower stride.
- Ensure high parallax is a fast-down transition to stride 1, while high stride recovery is slow and hysteretic.
- Keep `--adaptive-stride-shadow` behavior identical except for richer logging.
- Extend `test_adaptive_stride` with cases:
  - high parallax forces stride 1;
  - low parallax with healthy tracks can increase backend/update interval;
  - low tracks force full-rate tracking even if parallax is low;
  - hold time prevents oscillation.

The bigger architectural split should then move raw input-frame dropping and visual-update cadence under one `FrameRateDecision` so the runner does not maintain three partially overlapping policies.

### 5.4 Flight-specific lessons from repo reports

Existing experiment notes:

- `docs/reports/feature_gate_config_diff_20260607.md`
  - Fly2 failures were linked to high-speed straight flight, near-zero perpendicular parallax, poor feature conditioning, and gyro-aided KLT being disabled when `init_bg_sigma=0.050`.
- `docs/reports/fly2_fi_maxdist_cond_ablation_20260607.md`
  - Fly2 at about 500 m needs `fi_max_cond_number=1e5`, while fly1/fly3/fly4 should stay at 1e4.
  - Tightening gates causes feature starvation.
- `docs/reports/vel_accuracy_oc_fly1234_20260608.md`
  - Fly2 velocity spikes are dominated by EKF jumps from feature starvation.
  - Fly4 has a persistent yaw-drift problem.
- `baseline/latest/README.md`
  - Adaptive stride is not part of locked baseline yet; fixed stride12 and stride1 remain controls.

Design implications:

- Do not solve fly2 by blindly tightening feature gates.
- Preserve gyro-aided KLT early by using correct initialization covariance or explicit gate policy.
- Adaptive stride should prevent tracking loss before feature starvation reaches EKF divergence.
- Evaluation must compare against stride12 and stride1.

## 6. Implementation Plan

### Phase 0: Documentation and shadow logs

Deliverables:

- This design note.
- Add a supervisor shadow CSV contract:
  - timestamp
  - state
  - visual_status
  - backend_status
  - state_status
  - anchor_status
  - recommended_action
  - reason bitmask/string

No estimator behavior change.

Detailed tasks:

1. Add `ov_msckf/src/ros_free/ExternalAnchor.h` or equivalent data-contract header.
2. Add `ov_msckf/src/ros_free/AnchorTrustPolicy.h` with pure functions, no EKF dependencies.
3. Add a small `test_anchor_trust_policy` target or fold narrow tests into an existing ROS-free test target.
4. Add CLI and CSV only for shadow mode.
5. Verify compilation and that default baseline CLI path remains unchanged.

### Phase 1: External anchor model

Deliverables:

- `ExternalAnchorMeasurement`.
- `ExternalAnchorProvider`.
- `AnchorTrustPolicy`.
- FC/GNSS CSV parsers preserving satellites and optional accuracy fields.
- Compatibility converter from old `FCInitState`.

Behavior:

- Existing startup initialization still works.
- New shadow mode logs what the trust policy would do.

Detailed tasks:

1. Convert `FCInitState` to `ExternalAnchorMeasurement` with `source=FC_NAV`, medium default trust, and explicit covariance assumptions.
2. Add optional parsing for `satellites`, `fix_type`, `pdop/hdop/vdop`, `eph/epv/sacc` when present.
3. Keep legacy CSV columns accepted.
4. For GNSS coordinate anchors, default action is `LOG_ONLY` or `REJECT` unless a new explicit experimental flag permits use.

### Phase 2: Visual and backend health monitor

Deliverables:

- `VioHealthSnapshot`.
- `VisualHealthScore`.
- `BackendHealthScore`.
- `StateHealthScore`.
- reason bitmasks.

Behavior:

- No restart by default.
- Existing `visual_lost_credit` is replaced or wrapped.

Detailed tasks:

1. Add `VioHealthSnapshot` in runner-side or ros-free helper code.
2. Populate it from `TrackerWarpVizPacket`, `VisualFlowSummary`, `VisualUpdateCounters`, `MsckfLastStats`, and state/covariance sanity checks.
3. Emit typed visual status and reason bitmask.
4. Keep all actions in shadow mode.

### Phase 3: Safe recovery actions

Deliverables:

- Use `feed_measurement_pose_anchor` via trust policy.
- Use `apply_trusted_pose_anchor_reset` only for high-trust anchors.
- Gate hard reinit behind trusted anchor.
- Move nearest-GPS/course restart behind explicit diagnostic flag.

Behavior:

- No map relocalization claims.
- `LOST_WAIT_ANCHOR` if no external anchor exists.

Detailed tasks:

1. Route accepted medium/high trust anchors to `feed_measurement_pose_anchor`.
2. Permit `apply_trusted_pose_anchor_reset` only for high trust anchors and bounded correction.
3. Mark samples used for reset so they are not also fused.
4. Move `nearest_gps_course` hard restart behind an explicit diagnostic flag.
5. Add cooldown and reset budget.

### Phase 4: Adaptive stride split

Deliverables:

- Separate input tracking cadence and backend visual update cadence.
- Force full-rate tracking for high parallax/high gyro/low tracks.
- Add unified stride decision CSV.
- Update `test_adaptive_stride`.

Behavior:

- Default baseline unchanged.
- Adaptive mode tested in shadow first.

Detailed tasks:

1. Add `FrameRateDecision`.
2. Make high parallax/high gyro/low tracks fast-force `input_frame_stride=1`.
3. Keep backend `visual_update_stride` independently adjustable.
4. Collapse overlapping logs into one decision CSV or include cross-references.
5. Update `test_adaptive_stride`.

### Phase 5: Validation

Use baseline entry points:

```bash
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 12
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 1
```

For recovery/adaptive changes:

- Run shadow mode on fly1-fly4.
- Then active mode on selected flight.
- Use realtime visualization unless display is unavailable.
- Analyze with canonical `analysis/full_flight_error_analysis.py` / `analysis/flight_eval_tool.py`.

Required comparisons:

- fixed stride12
- fixed stride1
- adaptive shadow
- adaptive active
- supervisor shadow
- supervisor active only with explicit anchor source

## 7. Non-Goals

- Do not implement satellite-map matching in this pass.
- Do not claim relocalization without a persistent map or external anchor.
- Do not add hidden GPS XY fusion to the baseline.
- Do not use future evaluation windows or truth columns for online decisions.
- Do not replace KLT with learned features as part of this supervisor plan.
- Do not tune EKF process/measurement noise just to hide a recovery failure.

## 8. Acceptance Criteria

Design acceptance:

- At least 10 external systems/papers reviewed with source links.
- Each borrowed trick is classified as direct/adapt/borrow/reject.
- MSCKF no-map limitations are explicit.
- External-anchor trust and confidence fields are part of the interface.
- Adaptive stride obeys: larger parallax -> higher input frame rate.

Implementation acceptance for later patches:

- Shadow mode compiles and logs without changing trajectory.
- Hard reinit cannot run from unqualified nearest GPS by default.
- Satellite-map anchor can be plugged in without changing supervisor state machine.
- Existing `run_serial_msckf_ros_free` baseline behavior remains available.
- `test_adaptive_stride` covers high-parallax full-rate behavior.
- Flight validation uses canonical baseline/evaluation tooling.

## 9. Implementation Order Recommended For Code

The safest code order after this document is:

1. Adaptive-stride local fix and tests, because the active user file is `AdaptiveStrideController.h` and the rule is clear: high parallax requires higher frame rate.
2. Anchor data contract and trust policy in shadow mode, because it does not change estimator behavior.
3. Supervisor shadow log from runner using existing KLT/MSCKF/pose-anchor diagnostics.
4. Diagnostic GNSS restart guard, moving nearest-GPS/course reinit behind an explicit flag.
5. Active soft reset/reinit only after shadow logs show sane transitions on fly1-fly4.

Do not start by adding map relocalization, GPS XY fusion, or EKF noise retuning.
