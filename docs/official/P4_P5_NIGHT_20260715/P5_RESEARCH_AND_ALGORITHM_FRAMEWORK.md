# P5 研究结论与正式算法框架

Status: `PASS_DESIGN_ONLY`

Formal implementation: `NOT_STARTED`

本文件冻结今晚审计后的正式 P5 合同。它不是对当前旧
`AdaptiveStrideController` 的验收，也不把既有 active 回放写成正式 P5 结果。
完整推导见 `docs/official/P5_OPENVINS_ALGORITHM_FRAMEWORK.md`，公开研究见
`docs/official/P5_ADAPTIVE_VISUAL_SCHEDULING_RESEARCH.md`。

## 1. 设计结论

P5 必须把两个问题分开：

1. `TrackingCadencePlanner`：决定当前 raw frame 是否执行 KLT；
2. `BackendUpdateTrigger`：只在 KLT 已完成后，决定该 tracked frame 是否进入
   propagate/clone/MSCKF/SLAM。

正式 P5 不是 HIGH/NORMAL/TURN 到固定 stride 对的状态表。高度、速度、姿态和
角速度只能进入图像运动预测、地面 overlap、安全上限和 estimator-health 约束；
最终 cadence 由实际时间、直接 common-ID 几何、特征健康和后端信息需求决定。

## 2. 冻结数据流

```text
raw timestamp + causal IMU/navigation/health
  -> motion prediction + SafetyCapEvaluator
  -> TrackingCadencePlanner
       -> skip: no KLT, no FeatureDatabase write, no clone
       -> track: existing TrackKLT
            -> direct common-ID motion from last_tracking_snapshot
            -> direct common-ID motion from last_backend_snapshot
            -> BackendUpdateTrigger
                 -> tracking-only: cleanup exact timestamp and return
                 -> backend: existing propagate/clone/MSCKF/SLAM path
                              commit last_backend_snapshot after actual clone
```

P4 初始化期间可以复用 measurement 与 tracking planner，但 P4 的 alignment-frame
selection、Ceres attempt eligibility 和 P5 backend trigger 是三个不同决策。

## 3. 时间与 reference 合同

每次决策必须记录并区分：

```text
raw_timestamp
motion_measurement_timestamp
decision_timestamp
actual_raw_dt
actual_tracking_dt
actual_backend_dt
```

所有输入都必须满足 `sample.timestamp <= decision_timestamp`。previous recommended
stride 不能替代任何实际 `dt`。

两个 reference snapshot 独立维护：

- `last_tracking_snapshot`：最近一次实际完成 KLT 的 frame；
- `last_backend_snapshot`：最近一次真正创建 clone 的 frame。

trigger 意图、backend eligible 或函数入口都不能提前推进 backend reference。只有
actual clone 成功后才 commit；没有 clone 时继续相对旧 backend reference 累积几何。

## 4. 视觉运动与健康度

只比较 reference/current 两端都存在的 stable feature IDs。利用 board IMU rotation
和固定 `T_C_I` 预测纯旋转图像位置：

```text
R_CbCa = R_CI * R_IbIa * R_IC
x_hat_i^b = project(R_CbCa * unproject(x_i^a))
d_i_raw  = u_i^b - u_i^a
d_i_rot  = pixel(x_hat_i^b) - u_i^a
d_i_comp = u_i^b - pixel(x_hat_i^b)
```

`d_i_comp` 才是平移几何。禁止把 raw flow 或多个相邻 tracking interval 的 median
magnitude 相加后冒充 backend accumulated parallax。

每次 measurement 至少输出：

- raw/rotation/compensated P50、P75、P90、P95；
- common tracks、survival、track-age quantiles；
- reliability-weighted effective count
  `N_eff=(sum w_i)^2/sum(w_i^2)`；
- normalized grid occupancy `C_occ` 与 entropy `H_grid`；
- border/termination risk；
- KLT scatter 加 IMU rotation uncertainty，以及 candidate-horizon UCB；
- 数据时间、来源、age 和 validity。

缺失 uncertainty、coverage 或 reliability 时不能宣称高信息；应进入 tracking
protection，并由 maximum backend latency 防止后端饿死。

## 5. TrackingCadencePlanner

对每个整数 raw-frame gap `s`，用实际 candidate timestamp 计算 horizon，并检查：

```text
UCB(raw_P95(s))         <= registered KLT total-flow limit
UCB(rotation_P95(s))    <= registered rotation limit
rotation_edge_max(s)    <= registered edge limit
UCB(compensated_P95(s)) <= registered translation-flow limit
survival/N_eff/coverage/border risk pass
Delta_t_s               <= maximum_tracking_interval
```

有正式 AGL、ground model、时间和不确定度时，可加入 full camera-footprint polygon
overlap；任一必要量无效时必须输出 `overlap_status=unavailable`，不能用高度带或一维
footprint 代替。

令所有 cap 均通过的集合为 `S_safe`，registered compensated-parallax target band 为
`[P_track_min,P_track_max]`：

```text
S_band = {s in S_safe |
          P_track_min <= predicted_comp_P50(s) <= P_track_max}

if S_band nonempty:                 choose max(S_band)
else if every safe gap is too low:  choose max(S_safe)
else if S_safe nonempty:            choose min(S_safe)
else:                               choose 1 and report emergency
```

任一 cap violation 立即 downshift。upshift 只能由新的、相互独立的 KLT measurement
在持续时间内确认；skipped raw frames 不计确认样本。允许直接跳到最终安全 gap，不要求
1→2→4→6 的固定 ladder。

mode 只表达质量语义：`BOOTSTRAP_DENSE`、`GEOMETRY_CONTROLLED`、
`TRACKING_PROTECTION`、`ESTIMATOR_PROTECTION`，不直接映射固定 stride。

## 6. BackendUpdateTrigger

trigger 只在当前 frame 已完成 KLT 后运行，输入必须相对
`last_backend_snapshot` 直接计算。第一版可解释几何代理为：

```text
snr_i  = ||d_i_comp||^2 / (sigma_i^2 + epsilon)
g_i    = snr_i / (1 + snr_i)
Q_geom = N_eff * C_occ * H_grid * S_survive * weighted_median(g_i)
```

`Q_geom` 必须和 direct parallax、coverage、survival、common tracks 一起记录，不能
单独当作 information rank 证明。可选 Jacobian/logdet score 初期只允许 shadow。

决策优先级冻结为：

```text
FATAL_CONTRACT_ERROR
ESTIMATOR_EMERGENCY
FEATURE_TERMINATION_RISK
MAXIMUM_BACKEND_LATENCY
INFORMATION_READY
ACCUMULATING_INFORMATION
```

`INFORMATION_READY` 至少要求 direct geometry 有效、compensated parallax 达标、
`N_eff`/survival/coverage 达标、不是 pure rotation，并满足注册的 `Q_geom` 或信息条件。

pure rotation 可以要求更密 KLT，但不能因 raw flow 大而触发“平移信息充分”的 backend。
只有 latency、termination 或 estimator emergency 可以在这种情况下强制 backend，且
reason 必须写成 fallback/safety。

后端用 `T_backend_warn` 和 `T_backend_max` 防止 starvation。无有效视觉信息时的强制
backend 必须记录 `forced_without_visual_information`，不能制造 feature residual。

## 7. OpenVINS 生命周期不变量

- skipped raw frame：无 KLT、无 FeatureDatabase write、无 clone/MSCKF/SLAM；
- tracking-only frame：保持 tracker image/keypoint/ID continuity，删除 exact timestamp
  observations，在 propagation/clone 前返回；
- backend frame：沿用现有 OpenVINS backend；
- 没有 clone 的 timestamp 不能成为 MSCKF observation time；
- tracking-only observation 不能残留或被再次融合；
- scheduler reset 不能复活已删除 measurement；
- backend reference 只在 actual clone 后推进；
- GPS XY/course、future error、最终 trajectory error 不得进入 online decision。

## 8. 降级与错误处理

- motion/uncertainty/coverage 暂时不可用：dense tracking + explicit unavailable reason；
- overlap 不可用：禁用 overlap cap，不能伪造数值；
- visual/estimator health 可恢复异常：protection mode；
- timestamp、frame、calibration contract 错误：fatal，停止输出猜测 cadence；
- resource/CPU 只能在安全集合内选择更省计算的 gap，不能放宽几何、安全或 latency cap。

## 9. 门限证据合同

当前不注册正式数值门限。既有 4 flights × 8 fixed-stride sweep 只复用已记录字段，不
重新运行 32 组；缺失量标记 `not_recorded`，不得由 aggregate final error 反推。

正式识别使用 leave-one-flight-out：每折三飞训练，一飞一次性 held-out 验证。阈值由
KLT survival、triangulation conditioning、MSCKF accepted information、innovation、
covariance reduction、starvation 和 deadline 识别；GPS reference 只能离线评价，不能
进入 online feature vector。每个参数必须保存单位、定义、数据 hash、训练/held-out
flight、拟合方法、置信区间、保守值和失败案例。

## 10. 诊断合同

每个 raw frame、KLT frame 和 backend decision 分别记录 event。至少包含实际时间、
tracking/backend reference ID、recommended/actual gap、三类 motion quantiles 与 UCB、
`N_eff`/coverage/survival、overlap provenance、所有 active caps、`Q_geom`、pure-rotation、
primary/secondary reason、实际 clone commit result、KLT/backend wall time 和 deadline
margin。

## 11. 实现与验收顺序

1. `Stage 0`：冻结 contract（本文件完成）；
2. `Stage 1`：只加 measurement、uncertainty、coverage、双 reference 和完整 shadow log；
3. `Stage 2`：tracking planner shadow；
4. `Stage 3`：backend trigger shadow，并与真实 clone/MSCKF event 对照；
5. `Stage 4`：lifecycle、no-double-use、pure-rotation、latency 和 commit tests；
6. `Stage 5`：上述全部通过后才做 minimal active screening；
7. `Stage 6`：经单独批准后做 flight validation。

今晚只完成 Stage 0 和当前实现审计，没有实施正式 P5、没有启用 active takeover，也
没有把旧 active candidate 包装成新框架已经实现。
