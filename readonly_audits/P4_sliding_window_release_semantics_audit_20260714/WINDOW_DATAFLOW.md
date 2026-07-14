# P4 窗口与数据流审计

## 时间关系

运行时按三流共同因果时间推进：

```text
camera timestamp -> + camera_to_imu_time_offset -> board time
FC attitude/navigation -> 各自的 FC-to-board offset 插值
board IMU -> 直接按 board time 插值/积分
```

`try_initialize(now)` 使用不晚于 `now` 的最新可支持视觉帧作为 `init_time`，然后对 duration 列表逐个测试 `[init_time-duration, init_time]`。这一步是初始 Ceres solve 的输入合同，不是候选活动期间的持续 solve 合同。

## 五个对象的生命周期

| 对象 | 写入 | 裁剪/读取 | 候选期是否改变估计 |
| --- | --- | --- | --- |
| 原始 FC buffer | `feed_fc_navigation()` | `prune()`；`advance_candidate_filter()` 按 cursor 读取 | 只产生递归 p/v update |
| 原始 IMU buffer | `feed_board_imu()` | `prune()`；`propagateRuntime()` 读取 cursor 后区间 | 只传播 candidate filter |
| 选中视觉 buffer | `feed_stereo()` 经 selector 选中后写入 | 初始 solve 从中筛 duration；候选期旧数据裁剪 | 不触发 Ceres 重解 |
| Ceres selected keyframes | 初始 solve 前从窗口构造，最多 10 | candidate 建立后封存到 `candidate_result_` | 不增、不删、不重解 |
| Future snapshots | candidate 创建后，timestamp 晚于 candidate result 才写入 | `validate_candidate()` 一次消费 | 只做视觉诊断，recursive visual update 为 0 |

## 初始窗口的数据流

```text
raw FC/IMU/visual buffers
        |
        v
common causal horizon
        |
        v
duration candidates: 3, 5, 8, 12 (+ reference 2)
        |
        v
sample/gap/feature/depth gates
        |
        v
selected visual frames
        |
        v
interval rejection + uniform max_keyframes=10
        |
        v
Ceres states: q,p,v,bg,ba per keyframe + landmarks + mount
        |
        v
candidate_result_ + candidate_filter_
```

## 候选活动期的数据流

```text
new FC row
  -> causal board time
  -> IMU propagation to FC event
  -> measurement z = FC p/v with lever-arm transform
  -> H/P/K 15-state update + Joseph covariance
  -> history/support/gate
  -> p/v feedback every accepted event
  -> q/bg/ba first feedback only if data gate passes
  -> fixed post-feedback stable update confirmation
```

这条链是递归滤波，不是把过去一段时间的 FC/IMU/视觉因子重新放入一个共同优化问题。当前 `OnlineAlignmentCandidateFilter::updatePositionVelocity()` 只有 position/velocity measurement；没有 Ceres residual creation、landmark re-triangulation 或 visual recursive measurement update。

## 2 秒、实际选择窗口、最大保留窗口的区别

| 名称 | 当前来源 | 作用 |
| --- | --- | --- |
| reference window | `params.init_options.init_window_time`，baseline YAML 为 2 s | 被加入 candidate duration 列表，不代表唯一运行窗口 |
| candidate durations | runner 明确设为 `{3,5,8,12}` | 初始 solve 从短到长选择第一个满足 gate 的窗口 |
| selected window | `diag.selected_window_duration_s` | 本次 Ceres 的实际时间范围；已有 fly1=8 s、fly3=3 s |
| maximum buffer window | `candidate_window_durations_s.back()` | raw buffer 的保留上限，已有 metadata 为 12 s |
| current pending gate history | `options_.window_duration_s` | 构造函数覆盖为 maximum window，目前是 12 s；未构建、未接受 |

如果用户说“只要设置滑窗长度，量测次数自动知道”，至少需要把“selected window duration”作为真正的 confirmation contract，并让新窗口反复进入联合求解。当前代码把 selected window、maximum buffer window、candidate history 和 fixed post-feedback update count 分成了不同变量，且候选期没有联合重求解，所以不能得到这个语义。

