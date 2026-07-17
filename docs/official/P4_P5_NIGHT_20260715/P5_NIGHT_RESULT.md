# P5 Night Result

## 结论

| 项目 | 状态 |
|---|---|
| 当前实现审计 | `PASS` |
| 当前源码 frontend shadow/调度实跑 | `PASS` |
| P4 不污染 | `PASS` |
| 既有 4×8 sweep 核验 | `VERIFIED_NOT_FORMAL` |
| 完整 backend-trigger shadow | `NOT_IMPLEMENTED` |
| 正式 P5 实现 | `NOT_STARTED` |
| P5 active takeover | `NOT_RUN` |
| 仓库旧 candidate | `ACTIVE_CAPABLE_DEFAULT_OFF_NOT_ACCEPTED` |

本轮完成了目标要求的当前源码 shadow 运行证据，但没有实现或启用正式 P5。

## 直接验证

使用当前 worktree 和 `-j12`：

```text
P4/P5 visual scheduling tests 1-21 passed
online alignment initializer tests passed
```

当前 runner：

```text
e573274002c2c0b40a4ea074b0bd0e70badecb204ce7d877a1791efabcea3201
```

## Shadow 集成缺陷与修复

首次 fly1 诊断证明，旧 `--adaptive-stride-shadow` 会在 P4 初始化期额外重置共享
cadence counters，导致只打开日志也改变 P4 KLT 时间戳。runner 已修复：P4 释放前
shadow 不重置这些 counters，P4 释放后才维护 fixed-stride 实际间隔。

修复后 fly1/fly3 与冻结 P4-only 对照的 processed timestamps、P4 `t_init`、decision
time、readiness、feedback mask 和 15-D candidate state 全部一致。

## 当前源码 fly1/fly3 shadow

| 指标 | fly1 | fly3 |
|---|---:|---:|
| raw policy rows | 5,097 | 9,056 |
| post-init raw rows | 4,783 | 8,746 |
| actual processed frames | 713 | 1,039 |
| mode | shadow only | shadow only |
| actual stride | 12 only | 12 only |
| tracking > backend violations | 0 | 0 |
| post-init 12-frame gaps | 397 | 728 |
| P4 prefix timestamp mismatches | 0 | 0 |
| tracking-only / clone violation | 0 / 0 | 0 / 0 |
| runner / frame / batch | 0 / 0 / 0 | 0 / 0 / 0 |

formal run roots：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_100521_short_visible_shadow_global_baseline_fly1
  20260715_100808_short_visible_shadow_global_baseline_fly3
```

当前 shadow 只记录 frontend policy recommendation；实际 fixed-stride frame 走普通完整
backend，不 shadow-run `BackendUpdateTrigger`。所以 tracking-only 为 0 是合同边界，不是
backend trigger 已通过。

## 既有 4 flights × 8 fixed-stride sweep

直接核验结果：32/32 run 目录、命令、identity/config hash、exit 0 和 required files
仍完整；四个审计输出哈希与 provenance 一致。23/32 登记 1000 m divergence，只有
8/32 完整评价窗，fly1/fly3 均为 0/8。

该 sweep 使用 legacy single-row FC init、`global_yaw_oc_projection` 和 GPS altitude，且
缺 formal AGL、ground model/DEM、accepted exact-time chain 和全量 actual parallax。
正式状态保持 `NOT_FORMAL`，不能直接产生 tracking/backend 门限。

## 既有 2026-07-14 active full

旧 active 运行确实两飞 exit 0、clone violation 0，但 information trigger/backend 达
95.65%/96.50%；LOW state 的高度中位数约 199 m。它证明旧候选能主动运行，也证明
information proxy 和 overlap/state 语义尚未识别。该批 runner/Vio/P4 不是当前版本，
本轮没有重新启用 active takeover。

## 最终边界

当前源码 frontend shadow 集成验证完成。正式 P5 仍缺：

- backend-trigger shadow 与 actual-clone commit telemetry；
- direct backend geometry、termination、pure rotation；
- uncertainty/UCB、coverage、reliability 和 border risk；
- 正式门限识别与 active acceptance。

详细证据见 `P5_SHADOW_AND_SWEEP_VALIDATION.md` 和 `P5_RUNTIME_EVIDENCE.csv`。
