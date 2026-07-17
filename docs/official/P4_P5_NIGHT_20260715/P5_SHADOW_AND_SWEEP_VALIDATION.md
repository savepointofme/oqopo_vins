# P5 当前源码 Shadow 与既有 Sweep 证据核验

## 结论

本轮完成了当前源码 P5 的可执行 frontend shadow/调度验证，并修复了一处会让
`--adaptive-stride-shadow` 改变 P4 初始化期送帧节奏的 runner 集成缺陷。

最终分类如下：

| 项目 | 判定 |
|---|---|
| 当前源码 P5 frontend policy 可执行并逐 raw frame 记录 | `PASS` |
| shadow 保持实际 fixed stride 12 | `PASS` |
| shadow 不改变 P4 窗口、释放和候选状态 | `PASS` |
| tracking stride 始终不大于 backend stride | `PASS` |
| 完整 backend-trigger shadow | `NOT_IMPLEMENTED` |
| 正式 P5 active takeover | `NOT_RUN` |
| 正式 P5 算法实现/验收 | `NOT_STARTED` |

因此本轮证明的是 runner shadow 集成与当前旧候选 frontend policy 的可执行性，不是
正式 P5 已完成。

## 当前源码身份与直接测试

- worktree：`D:\vscode_dir\open_vins_p4_sliding_r1`
- commit：`fdd8d745229c7115b56b5d2d765f462a3fd68b4b`
- runner SHA-256：`e573274002c2c0b40a4ea074b0bd0e70badecb204ce7d877a1791efabcea3201`
- runner source SHA-256：`71a3cc846db84b81c62c8e081fe241a4fd6a274ab3b3c7fef015e2c996d22a9d`

使用 `-j12` 重建并运行：

```text
test_adaptive_stride: P4/P5 visual scheduling tests 1-21 passed
test_online_alignment_initializer: online alignment initializer tests passed
```

## 发现并修复的 shadow 污染

第一次 shadow 诊断运行发现，fly1 的处理帧时间戳从第 49 个处理帧起与冻结
P4-only 前缀分叉。根因不是 P5 active takeover，而是 runner 在 P4 初始化期复用了
同一组 cadence counters：

- P4-only 会累计 counters；
- 旧 shadow 分支因为 `args.adaptive_stride_shadow=true`，在每次送帧后额外重置 counters；
- 因此仅增加 shadow 日志标志就改变了 P4 初始化期的 KLT cadence。

修复位于 `run_serial_msckf_ros_free.cpp`：active P5 仍正常重置 counters；shadow 只在
P4 已释放后维护自己的 fixed-stride 实际间隔，P4 尚未释放时完全沿用 P4-only 的
counter 生命周期。

一次运行中修改批处理脚本导致 Bash 收尾文件偏移错误，该 root 被排除；最初包含
污染的 `20260715_094918...` root 也只保留为诊断，不作为通过证据。

## 正式当前源码 shadow 回放

两次回放均使用 `visible`、global-baseline 配置、正式 P4 输入和
`--adaptive-stride-shadow --camera-frame-stride 12`。没有使用
`--adaptive-stride`。

| 项目 | fly1 | fly3 |
|---|---:|---:|
| run root | `20260715_100521_short_visible_shadow_global_baseline_fly1` | `20260715_100808_short_visible_shadow_global_baseline_fly3` |
| raw policy rows | 5,097 | 9,056 |
| post-init raw rows | 4,783 | 8,746 |
| actual processed frames | 713 | 1,039 |
| actual processed ratio | 0.139886 | 0.114731 |
| mode values | shadow only | shadow only |
| current/applied stride values | 12 / 12 only | 12 / 12 only |
| tracking > backend violations | 0 | 0 |
| post-init fixed-stride gaps | 397×12；1×10 initialization boundary | 728×12 |
| tracking-only frames | 0 | 0 |
| clone violations | 0 | 0 |
| runner / frame contract / batch | 0 / 0 / 0 | 0 / 0 / 0 |

这里的 tracking-only 为 0 是预期行为：shadow 不调用 active
`BackendUpdateTrigger`，每个实际 fixed-stride frame 仍走普通完整 backend 路径。

### 当前旧候选给出的 shadow 建议

| flight | post-init policy states | post-init tracking/backend targets |
|---|---|---|
| fly1 | LOW 3,201；DEGRADED 1,560；NORMAL 22 | 2/4 3,159；1/1 1,560；1/4 42；2/8 21；2/2 1 |
| fly3 | LOW 7,916；DEGRADED 816；NORMAL 14 | 2/4 7,753；1/1 816；1/4 163；2/8 14 |

这些 target 只是当前旧 policy 的推荐输出，全部未应用到 estimator cadence。

## P4 不污染证明

对每个 shadow short run，使用同一 flight 的冻结正式 P4-only full run 作为前缀对照：

| 检查 | fly1 | fly3 |
|---|---:|---:|
| processed timestamp count | 713 | 1,039 |
| timestamp mismatch | 0 | 0 |
| `t_init` / decision time | exact match | exact match |
| readiness | NAVIGATION_READY | FULL_ALIGNMENT_READY |
| feedback mask | q,p,v,bg | q,p,v,bg,ba |
| 15-D candidate state max abs difference | 0 | 0 |

所以修复后的 shadow 没有改变 P4 窗口选择、候选估计、释放时刻或状态反馈。

## 既有 4 flights × 8 fixed-stride sweep

核验对象：

```text
C:\Users\baloney\Desktop\实验目录\P2_overlap_existing_sweep_20260711\
  20260711_fourflight_fixed-stride_offline_partial
```

直接重新读取 index、原始 run 目录和命令，并校验 provenance 中四个输出哈希：

- flights：fly1--fly4；
- strides：1、2、4、8、12、16、20、30；
- 32/32 run 目录仍存在；
- 32/32 command 存在且 parse/identity/config hash 通过；
- 32/32 exit 0、file complete；
- 23/32 登记 1000 m divergence；
- 仅 8/32 完整覆盖 requested evaluation window；
- fly1 0/8、fly3 0/8 完整窗口；
- FC initialization 全部是 legacy single-row；raw FC attitude 未被 index；
- yaw mode 全部是 `global_yaw_oc_projection`，并启用 GPS altitude；
- index、first-turn audit 和两份报告的实际 SHA-256 均与 provenance 登记值一致。

正式 overlap/门限证据仍缺少：formal AGL、ground model/DEM、accepted exact-time
chain 和所有完整运行的 actual parallax。因此该 sweep 的正式状态保持
`NOT_FORMAL`，只能用于历史失败边界和资源趋势，不能注册 tracking/backend 门限。

## 既有 2026-07-14 active full 运行

旧 active root：

```text
C:\Users\baloney\Desktop\实验目录\P4_P5_redesign_20260713\runs\
  20260714_032226_full_visible_active_fly1_fly3
```

两飞命令均包含 `--adaptive-stride` 且 exit 0。当前 worktree 的三个 P5 核心 header
经 LF 归一化后与该运行登记哈希一致；runner、Vio 和 P4 不是当前版本。

| 指标 | fly1 | fly3 |
|---|---:|---:|
| raw / KLT | 30,226 / 20,989 | 36,552 / 23,750 |
| backend / tracking-only | 13,809 / 6,882 | 17,643 / 5,947 |
| information triggers | 13,208 | 17,025 |
| information/backend | 95.65% | 96.50% |
| latency / safety | 379 / 217 | 213 / 400 |
| clone violations | 0 | 0 |
| LOW state rows | 19,133 | 26,624 |
| LOW-state height median | 199.36 m | 199.08 m |

该证据证明旧 candidate 能 active 调度，也证明其 information proxy 几乎让大多数
backend frame 直接更新，并在约 200 m 高度大量进入名为 LOW 的状态。它不能证明正式
P5 cadence 已识别，且本轮没有重启 active takeover。

## 最终边界

本轮完成了目标要求的当前源码 shadow/调度运行证据和既有 sweep 核验。正式 P5
仍缺完整 backend-trigger shadow、actual-clone commit telemetry、direct backend
geometry、uncertainty/coverage/reliability、pure-rotation/termination 与正式门限识别。
