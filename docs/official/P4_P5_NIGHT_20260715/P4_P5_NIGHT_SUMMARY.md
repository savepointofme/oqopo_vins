# P4/P5 Night Summary

> 历史快照：本文已被 `P4_EXPERIMENT_MASTER_RECORD_AND_RULES.md` 和机器总账取代。
> 本文中的“完成”只描述当时任务边界，不代表当前 P4 算法通过验收。

Overall validation status: `COMPLETE_WITH_NEGATIVE_ACCEPTANCE_RESULT`

纠正后的目标已经执行完：正式 P4 fly1/fly3 full-flight 回放和官方绝对导航评价已
完成；当前源码 P5 的非接管 shadow/调度实跑和既有 sweep 核验也已完成。结果不是
“P4/P5 已可用”：P4 full acceptance 失败，P5 只通过 frontend shadow 集成验证。

## 总结论

| 工作项 | 状态 | 证据边界 |
|---|---|---|
| 正式 P4 生命周期与 short 运行 | `PASS` | 当前实现、直接测试、fly1/fly3 short |
| P4 full-flight 回放与官方评价 | `COMPLETE` | 两飞 runner/frame contract 均为 0；官方绝对导航指标已生成 |
| P4 full-flight acceptance | `FAIL` | fly3 相对冻结 reference 明显退化；不得称为可用 |
| P5 当前源码 frontend shadow | `PASS` | 两飞逐 raw-frame建议、fixed stride 12、P4 不污染 |
| P5 完整 backend shadow | `NOT_IMPLEMENTED` | 当前 shadow 不运行 `BackendUpdateTrigger` |
| 正式 P5 实现/active takeover | `NOT_STARTED / NOT_RUN` | 本轮明确禁止 active 接管 |
| 既有 4×8 sweep 核验 | `VERIFIED_NOT_FORMAL` | 32/32 文件完整，但不能注册正式门限 |
| P6 / June12 | `NOT_STARTED` | 按范围未进入 |

## P4 full-flight 结果

正式运行根：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_085610_full_visible_p4_only_global_baseline_both
```

- fly1、fly3 runner exit 0；frame contract exit 0；
- fly1 `traj_nav.txt` 29,913 行；fly3 36,244 行；
- fly1 `NAVIGATION_READY`，q/p/v/bg feedback，ba prior-retained；
- fly3 `FULL_ALIGNMENT_READY`，q/p/v/bg/ba feedback；
- 两飞均使用固定 P4-only 路径，未启用 P5 active。

官方评价使用 `analysis/full_flight_error_analysis.py`，GPS update time、airborne
window、absolute navigation、无事后位置/航向/SE(3) 对齐：

| 指标 | fly1 | fly3 |
|---|---:|---:|
| airborne duration | 772.4 s | 1094.8 s |
| GPS distance | 29.653 km | 41.096 km |
| final XY error | 323.065 m | 784.034 m |
| XY RMSE | 205.247 m | 227.417 m |
| vertical RMSE | 8.755 m | 27.132 m |
| speed RMSE | 1.791 m/s | 3.131 m/s |
| Vxy vector RMSE | 4.396 m/s | 4.292 m/s |
| yaw/course RMSE | 6.076° | 5.622° |

同一时间窗与冻结 persistent/globalbaseline reference 比较：

| delta（formal-reference） | fly1 | fly3 |
|---|---:|---:|
| final XY | -4.656 m | +422.265 m |
| XY RMSE | +6.661 m | +11.331 m |
| vertical RMSE | -0.463 m | +5.829 m |
| speed RMSE | -0.181 m/s | +1.482 m/s |
| Vxy vector RMSE | +0.248 m/s | +0.624 m/s |
| yaw/course RMSE | +0.572° | +0.251° |

fly1 有得有失；fly3 位置、垂向和速度明显退化。因此 full 验证已经完成，但 P4
acceptance 必须判 `FAIL`。最直接的差异是 formal fly3 在首个 8 s candidate 上反馈
q/p/v/bg/ba，而冻结 reference 经后续 candidate、3 s window 只反馈 q/p/v；下一轮若
继续 P4，应单独隔离 bias feedback，不能同时调整窗口和门限。

官方评价：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\evaluation\
  20260715_085610_full_p4_only
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\evaluation\
  20260715_085610_matched_full
```

两个 dashboard 已修正非有限值序列化：NaN/Infinity 统一写为 JSON `null`，严格
JSON 校验和真实 Edge DOM 渲染均通过。

## P5 当前源码 shadow 结果

运行前以 `-j12` 重建 runner、`test_adaptive_stride` 和 initializer；21 项 P4/P5
visual scheduling tests 与 initializer tests 均通过。

首次诊断发现 shadow 标志会额外重置 P4 初始化期 cadence counters，确实污染 P4
送帧节奏。runner 已修复为：P4 释放前 shadow 沿用 P4-only counter 生命周期，释放后
才维护自己的 fixed-stride 间隔。修复后两飞与冻结 P4-only 的处理时间戳、窗口、释放
时刻、反馈层级和候选状态完全一致。

| 项目 | fly1 | fly3 |
|---|---:|---:|
| formal shadow root | `20260715_100521...fly1` | `20260715_100808...fly3` |
| raw policy rows | 5,097 | 9,056 |
| mode / actual stride | shadow / 12 | shadow / 12 |
| tracking > backend violations | 0 | 0 |
| post-init 12-frame actual gaps | 397 | 728 |
| P4 prefix timestamp mismatches | 0 / 713 | 0 / 1,039 |
| tracking-only / clone violation | 0 / 0 | 0 / 0 |
| runner / frame / batch | 0 / 0 / 0 | 0 / 0 / 0 |

tracking-only 为 0 是当前 shadow 合同的明确边界：它只记录 frontend recommendation，
没有 shadow-run backend trigger；实际 fixed-stride frame 仍走普通完整 backend。

## 既有证据核验

4 flights × 8 fixed-stride sweep 的 32 个 run 目录、命令和文件仍全部存在，32/32
exit 0，四个审计输出哈希与 provenance 一致。但 23/32 登记 1000 m divergence，只有
8/32 覆盖完整评价窗，fly1/fly3 均为 0/8；并缺 formal AGL、ground model、accepted
exact-time chain 和全量 actual parallax。结论保持 `NOT_FORMAL`。

2026-07-14 旧 active full 证据也被重新核验：两飞确实 active、exit 0、clone violation
为 0，但 information trigger/backend 达 95.65%/96.50%，且约 200 m 高度仍大量进入
`LOW_ALTITUDE_SAFETY`。它证明旧候选可运行，也证明门限/语义尚未识别。

详细证据：`P5_SHADOW_AND_SWEEP_VALIDATION.md`、`P5_RUNTIME_EVIDENCE.csv`。

逐项完成审计：`GOAL_COMPLETION_AUDIT.md`。

## 未作出的声明

- 没有把 P4 full 回放完成写成 P4 acceptance；
- 没有把 P5 frontend shadow 写成完整 P5 shadow；
- 没有把旧 active candidate 写成正式 P5；
- 没有运行新的 P5 active takeover；
- 没有进入 P6 或 June12；
- 没有使用 future/final GPS error 反调在线门限。
