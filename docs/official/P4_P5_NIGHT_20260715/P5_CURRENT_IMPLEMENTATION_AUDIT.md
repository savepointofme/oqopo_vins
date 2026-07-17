# P5 当前实现审计

## 结论

当前仓库同时存在两件容易混淆的东西：

1. 一个已经能由 `--adaptive-stride` 主动接管图像调度的旧候选实现；
2. 一个更新、更严格的目标视差/信息驱动正式 P5 设计。

二者不是同一个完成状态。旧候选实现是 **active-capable、默认关闭、未正式接受**；正式 P5 的研究和算法框架已经完成，但正式实现尚未开始。本轮没有启用 P5 主动接管。三个 P5 核心算法 header 没有修改；为取得当前源码的可执行 shadow 证据并保证它不污染 P4，本轮修改了 runner 的 shadow counter 接线和批处理入口。

## 实际调用链

### raw frame 到 KLT

```text
raw camera timestamp
  -> runner 组装因果状态、IMU、视觉健康和过去 GPS ENU-U 高度
  -> AdaptiveStrideController
       -> 旧 flight-state safety caps
       -> VisualCadencePlanner
  -> tracking due?
       no: 不调用 KLT
       yes: feed_measurement_camera_information_cadence()
```

代码证据：

- `AdaptiveStrideController` 的固定状态和固定安全目标位于
  `ov_msckf/src/ros_free/AdaptiveStrideController.h:19-134`；
- `VisualCadencePlanner` 的目标视差选择位于
  `ov_msckf/src/core/VisualCadencePlanner.h:238-458`；
- runner 在 `ov_msckf/src/run_serial_msckf_ros_free.cpp:3822-4186` 组装输入并决定是否调用 KLT；
- active 路径在 `:4305-4322` 调用 information-cadence API。

### KLT 到后端

```text
KLT writes current observations
  -> build current visual snapshot
  -> direct common-ID motion from last backend snapshot
  -> BackendUpdateTrigger
       -> information / feature-loss / motion-limit
       -> maximum-latency fallback
       -> safety fallback
  -> trigger false:
       cleanup current exact timestamp
       return before ZUPT/propagation/clone/MSCKF/SLAM
  -> trigger true:
       continue normal OpenVINS backend path
```

代码证据：

- KLT 在 `ov_msckf/src/core/VioManager.cpp:2722-2741` 运行；
- 相对 last-backend 的 motion 和触发在 `:2775-2842`；
- tracking-only 精确清理并返回在 `:2845-2878`；
- 触发器本体在 `ov_msckf/src/core/BackendUpdateTrigger.h:14-124`。

这说明当前源码已经不再把后端严格实现成第二个 raw-frame counter。`backend_update_stride` 仍被计算和记录，但 active 路径真正是否进入后端由 KLT 后的 `BackendUpdateTrigger` 决定。

## 三种运行模式的真实含义

| 模式 | KLT cadence | backend cadence | 结论 |
|---|---|---|---|
| 无 adaptive flag | 固定 `camera-frame-stride` | 每个被送入的 frame 默认 backend eligible | 普通固定 cadence |
| `--adaptive-stride-shadow` | 仍使用固定 `camera-frame-stride` | 仍走普通 backend；不调用 `BackendUpdateTrigger` | 只 shadow 前端建议，不是完整 P5 shadow |
| `--adaptive-stride` | `VisualCadencePlanner` 输出实际控制 KLT | KLT 后由 `BackendUpdateTrigger` 控制 | 旧候选 active 路径 |

证据是 runner 只有在 `args.adaptive_stride` 时改写 `do_cam_feed`，并且只有 active 路径调用 `feed_measurement_camera_information_cadence()`；shadow 路径最终调用普通 `feed_measurement_camera()`。

P4 尚未释放时还有一个独立事实：`p4_visual_cadence_active` 会复用同一个前端 planner 为 P4 选择 KLT frame，但不会运行 P5 后端触发器。它是 P4 的前端采样连接点，不等于 P5 已经接管初始化后的 estimator。

## 已经实现的部分

### 视觉运动

`compute_visual_motion_metrics()` 已经：

- 按 stable feature ID 匹配两帧；
- 使用 Camera-IMU 旋转得到 raw、rotation 和 rotation-compensated motion；
- 输出 common tracks、survival、track age 和 P50/P75/P90/P95；
- 保留真实 measurement `dt`。

### Tracking planner

当前 `VisualCadencePlanner` 已经：

- 在 `{1,2,4,6,8,12}` 上直接评估候选 gap；
- 用真实 raw-camera `dt` 和最近 KLT measurement `dt` 预测 motion；
- 检查 maximum tracking interval、raw/rotation/compensated flow、survival 和可选 polygon overlap；
- 在安全集合内选择目标 compensated parallax；
- 立即 downshift；
- 只让新的 KLT measurement 增加 upshift confirmation，跳过 raw frame 不伪造确认。

### Backend trigger 和 feature 生命周期

当前 `BackendUpdateTrigger` 已经：

- 直接比较 last-backend 与 current tracking snapshot，而不是累加相邻 median；
- 使用 compensated median、P95、common tracks、survival 和简化信息代理；
- 提供 0.5 s maximum-latency fallback；
- 为 tracking-only frame 删除精确时间 measurement；
- 在现有历史 active 运行中保持 clone-violation counter 为 0。

## 与正式 P5 的差距

### 1. 旧 flight-state 表仍在主动约束 planner

`AdaptiveStrideController` 仍用 `HIGH_ALTITUDE_CRUISE`、`TURN_SAFETY`、`LOW_ALTITUDE_SAFETY` 等状态给出固定 tracking/backend caps。正式框架要求 height、turn、descent 进入 motion/overlap/health prediction，而不是直接映射为固定 cadence pair。

因此当前实现是“目标视差 planner 嵌在旧状态表内”，不是正式 `TrackingCadencePlanner`。

### 2. overlap 输入没有满足正式地面模型合同

runner 使用不晚于当前相机时刻的 GPS ENU-U 作为 relative height，并以当前相机高度减去该值构造水平地面。它没有 ground-model ID，也没有被证明为 AGL。

既有 P2 审计对 4 flights × 8 strides 的结论为 `NOT_FORMAL`：32/32 run 文件齐全，但缺少 formal AGL、ground model/DEM、accepted exact-time chain 和全 run actual parallax。当前 `minimum_ground_overlap=0.90` 因此只能算 preliminary default，不能算注册安全门限。

### 3. 正式 motion/uncertainty/coverage 量尚不完整

当前实现缺少：

- per-feature reliability weight；
- effective track count `N_eff`；
- grid occupancy/entropy；
- border/termination risk；
- KLT 与 IMU rotation uncertainty；
- safety upper confidence bound；
- pure-rotation 判定；
- fatal timestamp/frame/calibration contract 状态。

### 4. backend trigger 的若干输入没有接线

- `long_track_ending` 在 input 结构中存在，但生产调用没有赋值；
- `visual_health_bad` 只在进入 `VISUAL_DEGRADED` 的状态变化 frame 传入，不表达持续退化；
- 简化信息代理是 `compensated_median * survival * log(1+common_tracks)`，没有 uncertainty、spatial coverage 或正式阈值来源；
- pure rotation 没有被独立识别，当前主要依靠 IMU excitation safety fallback。

### 5. last-backend snapshot 提交时机仍早于实际 clone

`VioManager.cpp:2838-2842` 在 trigger 返回 true 时立即推进 last-backend snapshot/timestamp；随后代码仍可能在 ZUPT 或其他 backend 前置分支提前返回。正式合同要求只有实际 clone/backend path 成功后才能 commit reference。

同样，`backend_frame_count` 当前统计 backend eligible frame，而不是经过 clone 成功确认后的 frame。历史 clone-violation 为 0 不能证明这个 commit 顺序正确。

### 6. shadow 与诊断合同不完整

当前 shadow 模式没有 shadow 运行 `BackendUpdateTrigger`，也没有逐帧记录：

- backend decision reason；
- accumulated-information proxy；
- direct backend motion quantiles；
- pure-rotation/termination status；
- last-backend snapshot ID 和实际 commit result。

因此无法按正式框架 Stage 3 将 shadow trigger 与真实 clone/MSCKF event 做完整对照。

## 直接测试

本轮使用当前工作树和 `-j12` 重建并运行：

```text
cmake --build build_p4_sliding_r1 \
  --target run_serial_msckf_ros_free test_adaptive_stride \
           test_online_alignment_initializer -j12
./build_p4_sliding_r1/test_adaptive_stride
./build_p4_sliding_r1/test_online_alignment_initializer
```

结果：`P4/P5 visual scheduling tests 1-21 passed`，且
`online alignment initializer tests passed`。

测试覆盖了状态切换、实际时间、tracking-only 清理、rotation-compensated motion、planner down/upshift、backend information/latency 触发、polygon overlap 和 P4 frame selection。它没有覆盖上述正式差距，尤其没有覆盖 actual-clone 后 commit、完整 backend shadow、pure rotation、正式 uncertainty 和门限识别。

## 2026-07-15 当前源码 shadow 运行证据

两次正式 shadow 回放使用当前 runner、可视化、global-baseline 输入、固定
`camera-frame-stride=12` 和 `--adaptive-stride-shadow`；命令均不含
`--adaptive-stride`。runner、frame contract 和外层 batch 的退出码均为 0：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_100521_short_visible_shadow_global_baseline_fly1
  20260715_100808_short_visible_shadow_global_baseline_fly3
```

| 检查 | fly1 | fly3 |
|---|---:|---:|
| raw policy rows | 5,097 | 9,056 |
| actual processed frames | 713 | 1,039 |
| current/applied stride | 12 / 12 only | 12 / 12 only |
| tracking > backend violations | 0 | 0 |
| P4 prefix timestamp mismatches | 0 / 713 | 0 / 1,039 |
| 15-D P4 candidate state max abs difference | 0 | 0 |
| tracking-only / clone violation | 0 / 0 | 0 / 0 |

首次诊断运行发现，仅启用 shadow 就会在 P4 初始化期额外重置 cadence counters，
导致 fly1 从第 49 个处理帧开始与冻结 P4-only 前缀分叉。runner 已修复为：active
P5 保持原重置行为；shadow 仅在 P4 释放后维护 fixed-stride counter，P4 释放前沿用
P4-only 生命周期。修复后的两飞在处理时间戳、`t_init`、窗口、readiness、feedback
mask 和 15-D candidate state 上均与冻结 P4-only 证据完全一致。

这证明当前 frontend shadow 可执行且不污染 P4，但不扩大 shadow 合同：当前 shadow
仍不调用 `BackendUpdateTrigger`，所以 tracking-only 为 0，也没有完整 backend decision
telemetry。详细逐帧统计见 `P5_SHADOW_AND_SWEEP_VALIDATION.md` 和
`P5_RUNTIME_EVIDENCE.csv`。

## 既有运行证据的正确解释

### 固定 stride 4×8 sweep

- 32/32 run 文件齐全、exit 0；
- 23/32 有已登记的 1000 m divergence；
- 只有 8/32 满足完整 requested evaluation window；
- 没有 formal AGL、ground model 或全量 direct common-ID compensated flow。

它可以提供历史失败边界和资源趋势，不能直接产生正式 overlap、tracking target 或 backend information threshold。

### 2026-07-14 full active 运行

该运行中的三个 P5 header 内容哈希与当前工作树一致（忽略 linked-worktree CRLF）：

```text
bfadbb...  VisualCadencePlanner.h
f39f86...  BackendUpdateTrigger.h
1e59bb...  AdaptiveStrideController.h
```

两飞均 exit 0，且记录了真实 active 调度：

| 指标 | fly1 | fly3 |
|---|---:|---:|
| raw frames | 30,226 | 36,552 |
| KLT frames | 20,989 | 23,750 |
| KLT ratio | 69.4% | 65.0% |
| backend-eligible frames | 13,809 | 17,643 |
| information triggers | 13,208 | 17,025 |
| latency fallbacks | 379 | 213 |
| safety triggers | 217 | 400 |
| tracking-only observation drops | 1,907,283 | 1,680,757 |
| clone violations | 0 | 0 |

这些数据暴露了两个问题：

1. information trigger 占 backend frame 的约 95.6%/96.5%，说明当前代理和门限几乎让多数 KLT frame 直接进入后端，尚未形成经过识别的信息 cadence；
2. adaptive 日志多数时间处于 `LOW_ALTITUDE_SAFETY`，即使记录的 relative height 常在 150--205 m，主因是 preliminary polygon-overlap gate。状态名和实际约束来源已经错位。

这批运行使用的是当前 P5 headers，但 runner/Vio/P4 源码不是今晚的正式 P4 版本，因此只能作为旧 P5 行为证据，不能认证“正式 P4 + 正式 P5”。

## 审计判定

| 项目 | 状态 |
|---|---|
| 旧 P5 candidate implementation | ACTIVE-capable，默认 OFF |
| 旧 P5 active release | NOT ACCEPTED |
| 当前源码 P5 frontend shadow integration | PASS，且 P4 prefix 无污染 |
| 当前 P5 shadow completeness | PARTIAL，只有 frontend recommendation |
| 正式 P5 research/framework | PASS |
| 正式 P5 implementation | NOT_STARTED |
| 本轮 P5 active takeover | NOT_STARTED |

正式实现不能在旧状态表或旧门限上继续打补丁。下一实现阶段应从完整 shadow instrumentation 开始，先修正 reference commit 和日志合同，再识别门限，最后才允许 minimal active screening。
