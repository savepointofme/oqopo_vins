# P4 滑动窗口与分级释放语义审计

## 审计边界

本审计只回答一个问题：当前 P4 是否真正执行“基于时间滑窗的持续重估，并按状态置信度分级释放”，还是执行“一次有限窗口联合初始化，再对固定候选做递归验证”。

本审计没有修改算法、门限、窗口长度、样本数、性能路径、P5 或 June12，也没有重新运行构建和飞行性能实验。审计针对当前工作树源码，并使用已有的 fly1/fly3 v6 运行元数据与 attempt receipts。当前工作树中仍有一份未接受、未构建的 duration/count 语义补丁；它只能作为“未验证草稿”记录，不能作为运行证据。

源码快照：commit `3d9296c34d19767b967e12d3ec8fbaf18d1ec1f3`。工作树在审计开始时已存在大量用户修改；`git diff --stat` 为 32 个已跟踪修改文件、约 8498 行新增和 1171 行删除，另有未跟踪源码、测试、文档和实验目录。完整 CMake 构建在审计前已停止，不能把旧二进制当作当前源码结果。

## 结论先行

**分类：B，且在候选存活期间具有 C 的特征。**

- B：一次性有限时间窗口 Ceres 联合初始化，加上固定候选的递归 FC 位置/速度验证与反馈。
- C 的部分：原始 FC/IMU/视觉缓存会按时间滑动，候选创建后旧样本还会被裁剪；但候选滤波器已经冻结，缓存滑动不会触发新的 Ceres 联合重估。
- 不是 A：没有在每个新的时间窗上持续重建状态、landmark、IMU 因子和 FC/视觉因子并重新求解。
- 不是单纯 C：候选被拒绝时，系统会回到收集阶段并允许新窗口重新求一次 Ceres；所以全流程是“初始化窗口重求解 + 候选期不重求解 + 接受后永久关闭”的混合流程。

根因只有一句话：**时间窗口目前主要决定原始缓存保留和首次 Ceres 的输入；一旦 `candidate_active_` 成立，`try_initialize()` 直接进入 `validate_candidate()`，不再进入窗口选择和 Ceres 求解，而递归验证又使用固定候选的历史/确认机制；接受释放后还把窗口永久关闭。**

## 实际调用链

```text
FC stream ──> feed_fc_navigation() ──> fc_buffer_ ──┐
board IMU ──> feed_board_imu() ─────> imu_buffer_ ──┼─> try_initialize(now)
KLT frame ──> feed_stereo() ─────────> stereo_buffer_ ─┘
                                                    │
                 candidate_active_ == false        │ candidate_active_ == true
                            │                       │
                            v                       v
              causal horizon + duration gates       validate_candidate()
              keyframe thinning + Ceres             │
                            │                       ├─ future visual snapshots: diagnostic only
                            v                       ├─ causal IMU propagation
                candidate_filter_.initialize()      ├─ FC p/v Kalman update
                candidate_active_ = true             ├─ p/v feedback
                                                    ├─ q/bg/ba first feedback when gate passes
                                                    └─ fixed history/confirmation gates
                                                            │
                                      reject ───────────────┤──────────── release
                                      new solve window       v
                                                     alignment_window_closed_ = true
                                                     FC/IMU/visual buffers clear
                                                     later try => alignment_already_released
```

精确入口和分支：`OnlineAlignmentInitializer::try_initialize()` 在 `OnlineAlignmentInitializer.cpp:1963-1979` 先检查永久关闭，再在 `candidate_active_` 时直接返回 `validate_candidate()`；只有没有活动候选时，才执行 `OnlineAlignmentInitializer.cpp:1997-2210` 的因果窗口扫描、候选窗口门控和 solve eligibility。

## 五类“窗口/数据”的实际含义

| 类别 | 代码对象 | 实际行为 | 是否持续重估 |
| --- | --- | --- | --- |
| 1. 原始传感器缓存 | `fc_buffer_`, `imu_buffer_`, `stereo_buffer_` | 按时间单调接收，`prune()` 删除过旧样本；候选期还按 candidate filter 的时间游标保留插值支撑 | 否；只是缓存滑动 |
| 2. 初始对齐窗口 | `candidate_window_durations_s`，当前生产为 `{3,5,8,12}`，另加入 reference `2` | 在共同因果时间上从短到长寻找第一个满足数量、间隔、特征和三角化条件的窗口 | 仅在没有活动候选、且新窗口 fingerprint 合格时重求解 |
| 3. 优化样本/关键帧 | `AlignmentFrameSelector` 和 `max_keyframes` | 视觉帧先按间隔/视差选择；进入 Ceres 前若超过 `max_keyframes=10`，均匀降到最多 10 个状态节点 | 否；候选期不重新生成 Ceres 图 |
| 4. 候选递归更新 | `OnlineAlignmentCandidateFilter` | 以候选结果为 nominal，因果传播 IMU，再用 FC position/velocity 做 15 维误差状态 Kalman update；没有新的 Ceres 图 | 是递归滤波，但不是时间滑窗联合重估 |
| 5. 反馈后确认数据 | `candidate_visual_snapshots_`、FC/IMU cursor、group history | 候选结果时间之后的数据用于候选验证；视觉快照只做健康/重投影诊断，实际递归闭环视觉更新计数为 0 | 部分；有未来数据，但不是独立的未来时间滑窗重估 |

## 候选是怎样生成的

1. `feed_fc_navigation()`、`feed_board_imu()` 和 `feed_stereo()` 只在窗口未关闭时接受单调有效数据。`feed_stereo()` 先经过 `AlignmentFrameSelector`；被选中的帧才写入 `stereo_buffer_`，并受 `maximum_selected_alignment_frames=36` 上限约束（`OnlineAlignmentInitializer.cpp:902-1058`，`AlignmentFrameSelector.h:31-77`）。
2. `try_initialize()` 以 `now` 和 camera-to-IMU offset 建立因果 horizon，倒序寻找最新的三流都能插值的视觉帧（`OnlineAlignmentInitializer.cpp:1997-2049`）。
3. 对 `{3,5,8,12}` 加 reference duration 逐个测试 FC、IMU、视觉帧数、feature tracks、间隔和 depth/visual 条件，选择第一个可用的 duration（`OnlineAlignmentInitializer.cpp:2061-2145`）。所以这里是“多个候选窗口中的一次选择”，不是一个随着新数据不断向前移动并重解的窗口。
4. 视觉可用帧超过 `max_keyframes` 时均匀抽取最多 10 个。这个 10 是 Ceres 状态节点上限，不是物理量测理论，也不是滑窗长度自动推导出来的次数（`OnlineAlignmentInitializer.cpp:2293-2308`）。
5. 成功求解后，把 q/p/v/bg/ba、协方差和 landmark 图结果包装为 `candidate_result_`，初始化 `candidate_filter_`，然后设置 `candidate_active_=true`（`OnlineAlignmentInitializer.cpp:3332-3368`）。从这一刻开始，后续新数据不会使原 Ceres 图增加节点或重算。

## “滑动”到底滑了什么

`prune()` 在候选创建前保留 `maximum_window_duration_s_` 及 margin；构造函数把它设为候选 duration 列表最大值，当前为 12 秒，并把 `options_.window_duration_s` 覆盖成 12 秒（`OnlineAlignmentInitializer.cpp:628-641,1064-1084`）。因此“配置的 2 秒”不是实际唯一在线 solve 窗口：runner 把 YAML 的 `init_window_time=2.0` 作为 reference duration，再与 `{3,5,8,12}` 合并；实际 fly1 选择 8 秒，fly3 选择 3 秒。

候选创建后，`prune()` 不再保存完整候选历史，而是保留 candidate filter board-time cursor 前的一个插值 bracket，并保留更晚数据；老的 FC/IMU/视觉数据会被删除。这个行为确实是 buffer sliding，但删除旧数据并不会调用 Ceres，也不会改变 `candidate_result_`。因此它不能被称为“滑窗持续重估”。

## 固定计数从哪里进入

固定计数不是由“滑窗长度自动决定量测次数”的数学结论产生的，而是来自三个不同层次的实现约束：

1. **优化节点上限**：`max_keyframes=10`。它最多只让 Ceres 看到 10 个视觉状态节点；fly1/fly3 的 accepted v6 receipt 都明确记录 `selected_frame_count=10`。
2. **候选递归确认的历史/支持统计**：accepted v6 运行元数据记录五个 group 的 `candidate_group_supported_update_counts=[10,10,10,10,10]`，且 `candidate_closed_loop_update_count=10`。这说明实际接受的旧二进制按 10 个 FC 事件完成候选验证，而不是按一个真实时间窗自动决定事件数。
3. **反馈后的稳定确认**：runner 设置 `required_post_feedback_stable_updates=2`（`run_serial_msckf_ros_free.cpp:2617-2631`），它是固定的“更新次数”，不是秒数。当前源码同时保留 `history_length_updates`、`min_supported_updates` 计数接口（`OnlineAlignmentCandidateFilter.h:84-104`）；工作树里存在把 history 改成 duration 的未接受补丁，但没有构建和端到端证据，不能宣称已实现。

当前未接受补丁还有一个语义问题：它在 `OnlineAlignmentInitializer.cpp:3321-3325` 使用 `options_.window_duration_s` 设置 history duration，而构造函数已经在 `:640-641` 把这个值覆盖为最大候选窗口 12 秒；它没有使用本次实际选中的 `diag.selected_window_duration_s`。因此即使只看源码草稿，也不能把它解释成“所选滑窗长度自动决定后续确认窗口”。

## q、p、v、bg、ba 的实际释放路径

| 状态 | 初始 Ceres | 候选期新量测 | 反馈 | 释放条件 | 结论 |
| --- | --- | --- | --- | --- | --- |
| q/attitude | 图状态、IMU、视觉和 mounting prior；当前源码明确 FC attitude 只做单独评价，不是 Ceres residual（`OnlineAlignmentInitializer.cpp:2452-2453`） | FC p/v update 通过 15 维协方差交叉项间接改变 q error；没有递归 FC attitude 或递归视觉状态更新 | 首次通过 gate 可注入；姿态反馈后做 error-state/covariance reset（`CandidateFilter.cpp:500-516,784-800`） | navigation 需 attitude group ready；之后接受即永久关闭 | 有分级反馈，但不是每个新时间窗联合重估 |
| p/position | 图状态和 FC position/velocity factor | 每个新 FC 事件做 position/velocity Kalman update | p 每个 accepted event 都可反馈（`CandidateFilter.cpp:492-504`） | position history/support、误差/std/波动和 navigation gate | 是持续递归修正，不是持续 Ceres |
| v/velocity | 图状态、IMU preintegration、FC velocity factor | 每个新 FC 事件做 velocity update | v 每个 accepted event 都可反馈 | velocity history/support、误差/std/波动和 navigation gate | 同上 |
| bg/gyro bias | 图状态、IMU 因子和首状态 bias prior；当前候选 filter 用初始协方差承接 | 没有独立 bg 量测；只能由 FC p/v 通过状态协方差交叉项间接更新 | 只有首个 data-gated correction 可反馈；后续 supported update 用于反馈后确认（`CandidateFilter.cpp:505-516`） | practical 可以保留 prior；strict/full 还需 bg ready | “可估计”不等于“重新联合求解”；释放后不再有新机会 |
| ba/accel bias | 图状态、IMU 因子和首状态 bias prior | 同样只能通过 FC p/v 间接更新 | gate 允许才反馈 | practical 可明确 prior-retained；full 需 ba ready | fly1 v6 明确没有 ba feedback |

候选 release 使用 `release_filter` 的副本完成一次最终反馈检查，然后把结果写出；成功后 `alignment_window_closed_=true` 并清空 FC/IMU/视觉缓存（`OnlineAlignmentInitializer.cpp:1828-1870`）。之后任何输入都会因为窗口关闭被拒绝，`try_initialize()` 返回 `alignment_already_released`（`:1969-1979`）。所以“后续 bg/ba 继续重估”在当前接受路径中是 **NO**。

## 候选形成后的未来数据是否真正用于确认

答案是 **PARTIAL**：

- 是：候选创建后、时间戳晚于 `candidate_result_.timestamp` 的视觉快照被放入 `candidate_visual_snapshots_`，`validate_candidate()` 会逐一处理；FC/IMU 也通过 cursor 处理候选之后的新数据。
- 否：这些视觉快照不进入新的 Ceres 图，也不形成递归视觉 Kalman update；`candidate_closed_loop_visual_update_count` 在 `validate_candidate()` 中明确置为 0（`OnlineAlignmentInitializer.cpp:1319-1430,1599`）。
- 否/有限：accepted v6 的候选确认是固定 FC 更新事件数和固定 post-feedback stable update 数。它没有“反馈时刻之后再完整覆盖一个新的、由时间长度定义的 confirmation window”这一独立合同。
- 接受时机使用当前候选 filter 的最新状态和 gate，随后永久关闭，因此不会在释放后继续用未来数据发现 q/bg/ba 需要再修正。

## 既有 fly1/fly3 运行证据

证据来源：

- `C:\Users\baloney\Desktop\P4_P5_redesign_20260713\P4_P5_short_validation_20260714_v6\fly1\online_alignment_metadata.json`
- 同目录 `online_alignment_attempt_receipts.json`
- fly3 同名文件

| 飞行 | solve attempts | candidate 结果 | 初始窗口 | 原始样本 | 选中关键帧 | 候选递归更新 | 反馈/释放 |
| --- | ---: | --- | ---: | --- | ---: | ---: | --- |
| fly1 | 1 | validated release | 8 s | FC 43 / IMU 1604 / visual 13 | 10 | 10 FC events | q,p,v,bg；NAVIGATION_READY；ba retained |
| fly3 | 2 | 第一次 rejected，第二次 validated release | 3 s | 每次 FC 20 / IMU 602；visual 30 | 10 | 成功候选 10 FC events | q,p,v,bg,ba；FULL_ALIGNMENT_READY |

fly1 receipt 的 10 个 selected frame timestamp 与 metadata 的 10 个 group supported/update 计数同时存在；它直接证明“10 个关键帧”和“10 个递归 FC 事件”在运行中是两个层次，但最终都表现为固定 10。fly3 的第一次失败后重新生成了第二个 candidate，证明拒绝可以触发重试；第二次成功后没有第三次 Ceres solve，证明成功候选不会持续重解。

这些是旧 v6 二进制的运行证据，不是当前工作树未接受 duration 补丁的证据。新的 v8/v9 诊断运行曾因 feedback safety violation 全部拒绝且 `traj_nav` 为空，因此不能被写成改进结果。

## 用户要求与当前实现的差异

用户要求的关键语义是：

```text
新数据到来
  -> 时间滑窗向前移动
  -> 用窗内完整 FC/IMU/视觉证据持续联合重估
  -> 对 q/p/v/bg/ba 分别判断内部置信度
  -> 先释放可信状态
  -> 反馈后用后续时间数据再次确认
  -> 未可信状态继续保留并在后续窗中获得新机会
```

当前实际语义是：

```text
新数据到来
  -> 原始 buffer 滑动
  -> 没有 candidate 时选择一个可用初始窗并 Ceres solve
  -> candidate 建立后固定候选 + 递归 FC p/v filter
  -> fixed update/history gate 决定 release
  -> release 后永久关闭
```

因此差异不是某个门限值不合适，而是重估对象和生命周期不一致：**当前有“持续递归滤波”，没有“持续时间滑窗联合重估”；有“分组反馈”，但没有“未释放状态在新联合窗口中继续获得估计机会”。**

## 文件

- `AUDIT_REPORT.md`：完整证据与逐状态结论。
- `CALL_GRAPH.md`：实际调用链和分支。
- `STATE_MACHINE.md`：当前状态机与目标状态机对照。
- `WINDOW_DATAFLOW.md`：五类窗口/数据对象的数据流。
- `FIXED_COUNT_INVENTORY.csv`：固定计数、窗口和确认条件清单。
- `RUNTIME_TRACE_FLY1.csv`：已有 fly1 运行轨迹摘要。
- `RUNTIME_TRACE_FLY3.csv`：已有 fly3 运行轨迹摘要。
- `CONCLUSION.md`：不可歧义的最终结论。

