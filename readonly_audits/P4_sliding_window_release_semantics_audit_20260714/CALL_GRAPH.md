# P4 实际调用图

## 输入阶段

```text
run_serial_msckf_ros_free.cpp
  ├─ 按 causal_horizon 推进 FC
  │    └─ VioManager::feed_measurement_fc_navigation()
  │         └─ OnlineAlignmentInitializer::feed_fc_navigation()
  │              ├─ validate_fc()
  │              ├─ fc_buffer_.push_back()
  │              └─ prune()
  │
  ├─ 按时间推进 board IMU
  │    └─ VioManager::feed_measurement_board_imu()
  │         ├─ 统一 IMU filter
  │         ├─ Propagator::feed_imu()
  │         └─ OnlineAlignmentInitializer::feed_board_imu()
  │              ├─ validate_imu()
  │              ├─ imu_buffer_.push_back()
  │              └─ prune()
  │
  └─ 当前相机帧进入 KLT 后
       └─ VioManager::track_image_and_update()
            ├─ make_online_stereo_frame()
            ├─ OnlineAlignmentInitializer::feed_stereo()
            │    ├─ AlignmentFrameSelector::evaluate()
            │    ├─ stereo_buffer_.push_back()  [selected only]
            │    ├─ candidate_visual_snapshots_.push_back() [candidate future only]
            │    └─ prune()
            └─ OnlineAlignmentInitializer::try_initialize(now, result)
```

## `try_initialize()` 的关键分叉

源码：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp:1963-1979`。

```text
try_initialize(now)
  ├─ fatal_configuration_error_       -> false
  ├─ alignment_window_closed_         -> false: alignment_already_released
  ├─ candidate_active_                -> validate_candidate(now, result)
  └─ no active candidate
       ├─ 建立 causal FC/IMU local view
       ├─ 找三流共同 horizon
       ├─ 依次测试 candidate_window_durations_s
       ├─ 建立 window fingerprint
       ├─ solve interval / new selected frames / fingerprint gate
       ├─ interval reject + max_keyframes downsample
       ├─ Ceres graph + solve
       ├─ graph validation
       └─ candidate_filter_.initialize(); candidate_active_=true
```

这里最关键的事实是：`candidate_active_` 分支在进入任何窗口扫描之前返回。新 FC/IMU/视觉数据不会把候选重新放回 Ceres 图。

## 候选验证调用图

```text
validate_candidate(now)
  ├─ 计算 candidate 支持的 causal horizon
  ├─ 处理 candidate_visual_snapshots_
  │    └─ 视觉健康/残差诊断；不调用 candidate filter visual update
  ├─ advance_candidate_filter()
  │    ├─ 读取 cursor 之后的 FC rows
  │    ├─ interval_imu_samples()
  │    ├─ candidate_filter_.propagateRuntime()
  │    └─ candidate_filter_.updatePositionVelocity()
  │         ├─ duplicate FC timestamp skip
  │         ├─ p update / v update
  │         ├─ Joseph covariance update
  │         ├─ p/v feedback
  │         └─ q/bg/ba first feedback when gate passes
  ├─ 读取 groupReady/navigationReady/fullAlignmentReady
  ├─ 未满足但非硬失败 -> 保持 CANDIDATE_VALIDATING
  ├─ safety/numerical failure -> reject_candidate()
  └─ accepted
       ├─ release_filter copy
       ├─ apply allowed feedback
       ├─ build AlignmentResult
       ├─ alignment_window_closed_=true
       ├─ clear candidate and raw buffers
       └─ later calls return alignment_already_released
```

## 拒绝路径

`reject_candidate()`（`OnlineAlignmentInitializer.cpp:1172-1185`）清除 candidate filter、candidate visual snapshots 和 candidate record，随后：

- 若启用一次 refinement，则最多请求一次 refinement solve；生产 runner 当前 `candidate_refinement_enabled=false`。
- 否则把下一次资格设为 `new_sliding_window_information`，返回 `FAILED_WAIT_RETRY`，新数据到达后可以重新建立窗口和 Ceres candidate。

因此拒绝后有“新窗口重解”，接受后没有“持续窗口重解”。

