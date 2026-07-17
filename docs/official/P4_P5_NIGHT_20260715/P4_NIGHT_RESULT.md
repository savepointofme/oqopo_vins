# P4 Night Result

> 历史快照：当前统一结论、全部后续实验和强制规范见
> `P4_EXPERIMENT_MASTER_RECORD_AND_RULES.md`。本文不能单独用于判断当前 P4 状态。

Status: `FULL_VALIDATION_COMPLETE_ACCEPTANCE_FAILED`

正式有限时间窗口生命周期、直接测试、fly1/fly3 short 和 full-flight 回放均已执行。
full-flight 运行成功不等于算法通过：相对冻结 persistent/globalbaseline reference，
尤其 fly3 明显退化，因此 P4 不能标记为可用或 accepted。

## 已实现生命周期

```text
finite causal raw buffers
  -> fixed 8 s current window
  -> solve after window end advances in sensor time
  -> joint FC + board-IMU + monocular-visual Ceres solve
  -> candidate
  -> bounded later-data causal validation
  -> optional one advanced-window refinement
  -> one OpenVINS injection
  -> permanent P4 close
```

release 不依赖固定事件数。q/p/v/bg/ba 使用 covariance、innovation/NIS、correction、
residual、time support 和 feedback safety；不可信 bias 保留 prior 和保守 covariance。

## 构建与测试

- 使用 `-j12` 构建 runner、candidate-filter 和 initializer targets；
- candidate-filter tests passed；
- initializer tests passed；
- bounded failure/refinement、unique fingerprint、time advance、warm start 和 release-once
  路径均有直接测试。

## Short 证据

| 项目 | fly1 | fly3 |
|---|---:|---:|
| readiness | NAVIGATION_READY | FULL_ALIGNMENT_READY |
| solve/candidate/release | 1/1/1 | 1/1/1 |
| causal validation | 2.074 s | 2.063 s |
| feedback | q/p/v/bg；ba prior-retained | q/p/v/bg/ba |
| `traj_nav.txt` | 4,783 rows | 8,747 rows |

matched short 相对冻结 v6：fly1 基本持平但位置/航向略差；fly3 short 明显改善。这只
支持 short lifecycle，不支持 full acceptance。

## Full-flight 正式运行

运行根：

```text
C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\P4_P5_redesign_20260713\runs\
  20260715_085610_full_visible_p4_only_global_baseline_both
```

| 项目 | fly1 | fly3 |
|---|---:|---:|
| runner / frame contract | 0 / 0 | 0 / 0 |
| `traj_nav.txt` rows | 29,913 | 36,244 |
| readiness | NAVIGATION_READY | FULL_ALIGNMENT_READY |
| feedback | q/p/v/bg | q/p/v/bg/ba |
| selected window | 8 s | 8 s |

runner SHA-256 为
`8f481c60cf4103e50ee28d33af8e9b4fee10ab3db6d7a8731dfcf50b045de77d`。
随后对 runner 所作改动只隔离 `--adaptive-stride-shadow` 的 counter reset；P4-only
条件仍为 false，full P4 路径没有改变。

## 官方绝对导航评价

使用 `analysis/full_flight_error_analysis.py`，GPS update time、`airborne_auto`、
`absolute_navigation_no_post_alignment`，不做事后位置、航向或 SE(3) 对齐。

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

## 同窗冻结 reference 对比

| delta（formal-reference） | fly1 | fly3 |
|---|---:|---:|
| final XY | -4.656 m | +422.265 m |
| XY RMSE | +6.661 m | +11.331 m |
| vertical RMSE | -0.463 m | +5.829 m |
| speed RMSE | -0.181 m/s | +1.482 m/s |
| Vxy vector RMSE | +0.248 m/s | +0.624 m/s |
| yaw/course RMSE | +0.572° | +0.251° |

fly1 有改善也有退化；fly3 在最终位置、垂向和速度上显著退化。正式 full acceptance
判 `FAIL`。

最小后续假设是单独隔离 bias feedback：formal fly3 在首个 8 s candidate 上反馈
q/p/v/bg/ba，而冻结 reference 经后续 candidate、3 s window 只反馈 q/p/v。该假设尚未
运行，不能写成已定位根因，也不能同时调整窗口和门限掩盖差异。

## 证据位置

- official full：`C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\evaluation\20260715_085610_full_p4_only`
- matched full：`C:\Users\baloney\Desktop\实验目录\P4_formal_20260715\evaluation\20260715_085610_matched_full`
- 两个 interactive dashboard 位于各 flight 的 `reports/interactive_dashboard.html`；
  strict JSON 和真实 Edge 渲染均已验证。

## 最终边界

- full replay 与评价任务已完成；
- P4 生命周期实现仍存在，但 full 导航表现未通过；
- 不更新正式算法报告为“已可用”；
- 不进入 June12。
