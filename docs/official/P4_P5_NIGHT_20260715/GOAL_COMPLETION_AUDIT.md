# P4/P5 目标完成审计

审计时间：2026-07-15

目标只要求取得正式 P4 full-flight/官方绝对导航证据，以及在不启用 P5 active、
不污染 P4 的前提下取得当前源码 P5 shadow/调度证据并核验既有 sweep。它不把
“验证已完成”改写成“算法已通过验收”。

## 最终判定

| 层级 | 判定 |
|---|---|
| 本目标要求的运行与证据工作 | `COMPLETE` |
| P4 full-flight 算法验收 | `FAIL` |
| 当前源码 P5 frontend shadow 集成 | `PASS` |
| 完整 P5 backend shadow | `NOT_IMPLEMENTED` |
| 正式 P5 active takeover | `NOT_RUN` |
| P6 / June12 | `NOT_STARTED` |

## 逐项证据

### 1. 隔离 worktree

- worktree：`D:\vscode_dir\open_vins_p4_sliding_r1`
- branch：`redesign/p4-sliding-window-r1-20260714`
- HEAD：`fdd8d745229c7115b56b5d2d765f462a3fd68b4b`
- 正式 P4 与当前源码 P5 shadow 的 runner 均由该 worktree 的
  `build_p4_sliding_r1` 产生。

### 2. 正式 P4 fly1/fly3 full-flight

运行根：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_085610_full_visible_p4_only_global_baseline_both
```

| 直接检查 | fly1 | fly3 |
|---|---:|---:|
| runner exit | 0 | 0 |
| frame contract exit | 0 | 0 |
| command 含 `--adaptive-stride` | 否 | 否 |
| command 含 shadow | 否 | 否 |
| fixed camera stride | 12 | 12 |
| `traj_nav.txt` 总行数 | 29,913 | 36,244 |
| release | NAVIGATION_READY | FULL_ALIGNMENT_READY |
| selected window | 8 s | 8 s |
| feedback | q,p,v,bg | q,p,v,bg,ba |

两条轨迹均非空，且运行命令、退出码、frame contract 和 alignment metadata 均在
对应 run 目录中。

### 3. 官方绝对导航评价

官方评价根：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\evaluation\
  20260715_085610_full_p4_only
```

- fly1/fly3 evaluation exit 均为 0；
- 在 GPS update time 采样；
- `evaluation_window_mode=airborne_auto`；
- `alignment_mode=absolute_navigation_no_post_alignment`；
- position、heading 和 SE(3) post alignment 均为 false；
- 评价脚本为当前主仓正式 `analysis/full_flight_error_analysis.py`，运行时通过
  `P4_ANALYSIS_SCRIPT` 显式固定，SHA-256 为
  `e92d95bbbe4a77c78278400a8870ba947e1bf4c59e46a9f202aacb769d2b33`。
  该工具只读隔离 worktree 产生的轨迹，不进入 estimator。

| 指标 | fly1 | fly3 |
|---|---:|---:|
| duration | 772.4 s | 1094.8 s |
| distance | 29.653 km | 41.096 km |
| final XY | 323.065 m | 784.034 m |
| XY RMSE | 205.247 m | 227.417 m |
| vertical RMSE | 8.755 m | 27.132 m |
| speed RMSE | 1.791 m/s | 3.131 m/s |
| Vxy vector RMSE | 4.396 m/s | 4.292 m/s |
| yaw/course RMSE | 6.076° | 5.622° |

matched full 对比证明 P4 acceptance 失败：fly3 相对冻结 reference 的 final XY
增加 422.265 m，vertical RMSE 增加 5.829 m，speed RMSE 增加 1.482 m/s。

两个 dashboard 已用 strict JSON 重新生成。生成器现在把 NaN/Infinity 写成 JSON
`null`，并在静态校验中拒绝非有限 JSON token；Edge headless 实际执行验证结果为
fly1/fly3 `rendered=true`、`body_error=false`，随后已重新打开可视页面。

### 4. 当前源码 P5 shadow/调度

正式 shadow 根：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_100521_short_visible_shadow_global_baseline_fly1
  20260715_100808_short_visible_shadow_global_baseline_fly3
```

current runner SHA-256：
`e573274002c2c0b40a4ea074b0bd0e70badecb204ce7d877a1791efabcea3201`。

| 直接检查 | fly1 | fly3 |
|---|---:|---:|
| runner / frame / batch | 0 / 0 / 0 | 0 / 0 / 0 |
| raw policy rows | 5,097 | 9,056 |
| mode | shadow only | shadow only |
| current/applied stride | 12 / 12 only | 12 / 12 only |
| tracking > backend violations | 0 | 0 |
| command 含 active takeover | 否 | 否 |
| tracking-only / clone violation | 0 / 0 | 0 / 0 |

当前 shadow 只执行 frontend policy recommendation，实际 fixed-stride frame 仍走普通
backend；因此它是可执行的 frontend shadow，不是完整 backend shadow。

### 5. P4 不污染

首次诊断发现 shadow 会在 P4 释放前错误重置共享 counters；该 root 已排除。修复后
逐项与冻结 P4-only full 前缀比较：

| 检查 | fly1 | fly3 |
|---|---:|---:|
| processed timestamp prefix | 713/713 全等 | 1,039/1,039 全等 |
| `t_init` | 全等 | 全等 |
| decision time | 全等 | 全等 |
| readiness | 全等 | 全等 |
| feedback mask | 全等 | 全等 |
| 15-D candidate state max abs diff | 0 | 0 |

所以当前 shadow 标志不改变 P4 窗口、候选、释放或反馈。

### 6. 既有 sweep 核验

核验根：

```text
C:\Users\baloney\Desktop\实验目录\P2_overlap_existing_sweep_20260711\
  20260711_fourflight_fixed-stride_offline_partial
```

- 32/32 run 文件完整；
- 23/32 登记 1000 m divergence；
- 仅 8/32 完整评价窗；
- 四个登记输出文件的当前 SHA-256 均与 provenance 一致；
- `formal_status=NOT_FORMAL`；
- `safety_threshold_generated=false`；
- 缺 formal AGL、ground model/DEM、accepted exact-time chain 和全量 actual
  parallax，故未用它注册 P5 门限。

### 7. 构建、测试与范围

最终重跑：

```text
P4/P5 visual scheduling tests 1-21 passed
online alignment initializer tests passed
```

两份 batch script 均通过 `bash -n`。当前 worktree 变更和本轮运行根中没有 P6 或
June12 工作；没有启用新的 P5 active takeover。

## 完成边界

本目标可以关闭，因为它要求的 full-flight、官方评价、P5 非接管 shadow 和 sweep
证据已经逐项取得并验证。关闭目标不改变以下负面结论：P4 算法未通过 full-flight
验收；P5 只完成 frontend shadow 集成验证，正式 P5 实现与 active 验收仍未开始。
