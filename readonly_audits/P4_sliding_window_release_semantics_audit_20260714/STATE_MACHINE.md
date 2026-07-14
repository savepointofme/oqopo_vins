# P4 实际状态机与目标状态机

## 当前源码状态机

```text
WAIT_INPUTS
    -> COLLECTING
    -> ALIGNING
    -> VALIDATING
    -> CANDIDATE_VALIDATING
       ├─ 等待新因果数据继续 validate_candidate()
       ├─ reject_candidate() -> FAILED_WAIT_RETRY -> COLLECTING
       └─ release -> NAVIGATION_READY 或 FULL_ALIGNMENT_READY
                              -> CLOSED_AFTER_RELEASE 语义
```

状态定义位于 `OnlineAlignmentInitializer.h:67-74`；候选状态由 `candidate_active_` 控制，永久关闭由 `alignment_window_closed_` 控制。

### 当前各状态的实际含义

| 状态 | 实际动作 | 是否重新建立 Ceres 图 |
| --- | --- | ---: |
| `WAIT_INPUTS` | 等待有效 FC、IMU、视觉 | 否 |
| `COLLECTING` | 原始缓存和选中视觉帧积累；尝试找到可用初始窗口 | 只有无活动 candidate 且 solve eligibility 通过时 |
| `ALIGNING` | 对选中的初始窗口做一次 Ceres solve | 是 |
| `VALIDATING` | 评估本次图解并构造 candidate | 这次 solve 的后处理 |
| `CANDIDATE_VALIDATING` | 固定 candidate，因果 IMU + FC p/v 递归更新与分组 gate | 否 |
| `FAILED_WAIT_RETRY` | 当前 candidate/窗口失败，等新数据重新收集 | 下次可重新 solve |
| `NAVIGATION_READY` | 释放导航状态 | 否；随后窗口关闭 |
| `FULL_ALIGNMENT_READY` | 释放导航和 bias 状态 | 否；随后窗口关闭 |
| `CLOSED_AFTER_RELEASE` | 代码通过 `alignment_window_closed_` 实现输入/solve 关闭 | 否 |

## 用户要求的目标状态机

目标语义应当是：

```text
COLLECTING
  -> ACTIVE_TIME_WINDOW
       -> JOINT_REESTIMATE(window_t)
       -> GROUP_CONFIDENCE_EVALUATION
       ├─ q/p/v 可释放，bg/ba 保留
       ├─ 反馈后进入 POST_FEEDBACK_CONFIRMATION(window_t+)
       ├─ 未通过则保留并继续向前滑窗、再次联合重估
       └─ 通过的 group 保持已释放，但未通过 group 继续有后续窗口机会
```

目标状态机的核心不在于把 `history_length_updates` 换成一个 duration 字段，而在于每个新时间窗口都能重新计算与当前数据一致的联合状态估计，并让未释放的状态继续进入后续窗口。当前 `candidate_active_` 分支和 `alignment_window_closed_` 分支均不满足这个生命周期。

## q/p/v/bg/ba 的分级结果

当前 `navigationReady()` 需要 attitude、position、velocity 三组 ready；`fullAlignmentReady()` 在此基础上增加 gyro bias、accel bias（`OnlineAlignmentCandidateFilter.cpp:840-875`）。这确实是分组 release API，但它发生在一个固定 candidate filter 上：

- practical 路径可以 `NAVIGATION_READY`，并把 bias 保留为 prior/conservative covariance；
- strict/full 路径要求 bg/ba 也 ready；
- 一旦 practical 导航释放，`alignment_window_closed_` 会阻止未来数据继续估计 bias。

所以“分时释放”部分存在，“未释放状态在后续时间窗继续联合估计”部分不存在。

