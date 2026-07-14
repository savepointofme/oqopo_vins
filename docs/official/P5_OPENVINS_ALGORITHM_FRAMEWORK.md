# P5 目标视差驱动的 OpenVINS 自适应视觉调度算法框架

## 1. 设计结论

P5 应重构为两个相互独立、按时间顺序连接的决策器：

1. `TrackingCadencePlanner`：在每个 raw camera timestamp 之前，决定当前图像是否运行现有 `TrackKLT`；
2. `BackendUpdateTrigger`：只有当前帧已经完成 KLT 后，决定它是否创建 OpenVINS clone 并进入现有 MSCKF/SLAM update。

首要目标不是最小 CPU，也不是按高度或飞行状态查表。首要目标是：

- 相邻实际 KLT frames 之间的图像运动既不能小到几乎没有新增几何，也不能大到 KLT 不稳定；
- 相对上一个实际 backend frame 累积足够的 rotation-compensated translational parallax、feature survival 和信息量后再做视觉 update；
- 在 pure rotation、low altitude、high angular rate、feature degradation、covariance abnormal 或 update starvation 时立即进入保守路径；
- 不改变 OpenVINS 的 `TrackKLT`、`FeatureDatabase`、IMU propagation、clone、MSCKF structureless update、SLAM feature、FEJ 和 covariance 生命周期。

高度、速度、姿态、角速度、下降和 flight phase 只用于 motion prediction、ground-overlap calculation 与 safety caps。它们不得直接映射为 `HIGH -> 4/12`、`TURN -> 1/2` 等固定 tracking/backend stride pair。

本文件是后续实现规格，不表示算法已经实现、阈值已经识别或飞行验证已经完成。研究依据见 [`P5_ADAPTIVE_VISUAL_SCHEDULING_RESEARCH.md`](P5_ADAPTIVE_VISUAL_SCHEDULING_RESEARCH.md)。

## 2. 范围与不变量

### 2.1 保持不变的 OpenVINS 架构

- 单目 `TrackKLT`；
- stable feature IDs；
- raw pixel 与 undistorted normalized observations；
- 一个 `FeatureDatabase`；
- 一个 `VioManager`；
- board IMU propagation；
- clone augmentation；
- MSCKF structureless feature update；
- SLAM feature update 与 delayed initialization；
- FEJ；
- error-state covariance management；
- fixed Camera–IMU `T_C_I` 与已接受的 camera time relation。

### 2.2 不进入第一版 P5 的内容

- VINS-Mono/OKVIS/Kimera bundle-adjustment backend；
- 第二套 map、tracker、FeatureDatabase、VIO filter 或 clone window；
- descriptor、XFeat、SuperPoint 或其他 learned frontend；
- reinforcement learning、Visual-Selective VIO network；
- online Camera–IMU calibration；
- 历史约 7°、4.089°或 time-varying flex compensation；
- repair、restart、anchor reset；
- GPS XY/course、reference trajectory、error columns 或未来评价结果作为在线输入；
- 按 flight phase 固定输出 stride pair；
- 把 CPU reduction 本身当作 accuracy/safety pass。

## 3. 当前 fork 可复用接口

当前实际源码已经提供 P5 所需的主要连接点：

| 作用 | 当前接口 | 位置 |
|---|---|---|
| KLT sidecar | `TrackerWarpVizPacket` 保存 previous/current raw points、rotation-compensated points、stable IDs、实际 timestamp | `ov_core/src/track/TrackBase.h:54` |
| 单目前端 | `TrackKLT::feed_monocular()` | `ov_core/src/track/TrackKLT.cpp:129` |
| observation 写入 | `FeatureDatabase::update_feature()` | `ov_core/src/feat/FeatureDatabase.cpp:59` |
| tracking-only 精确清理 | `FeatureDatabase::cleanup_measurements_exact()` | `ov_core/src/feat/FeatureDatabase.cpp:245` |
| 已有 motion metric | `compute_visual_motion_metrics()` | `ov_msckf/src/core/VisualCadencePlanner.h:173` |
| 已有 preliminary cadence class | `VisualCadencePlanner` | `ov_msckf/src/core/VisualCadencePlanner.h:282` |
| 已有 preliminary backend trigger | `BackendUpdateTrigger::evaluate()` | `ov_msckf/src/core/BackendUpdateTrigger.h:50` |
| KLT 后的 backend decision | `VioManager::track_image_and_update()` | `ov_msckf/src/core/VioManager.cpp:2591` |
| 相对 last backend 的直接 motion | `motion_from_last_backend` | `ov_msckf/src/core/VioManager.cpp:2825` |
| 正常后端 | `VioManager::do_feature_propagate_update()` | `ov_msckf/src/core/VioManager.cpp:3052` |
| MSCKF feature cap | `max_msckf_in_update` | `ov_msckf/src/core/VioManager.cpp:3434` |
| feature reuse prevention | `to_delete` after update/rejection | `ov_msckf/src/update/UpdaterMSCKF.cpp:207`、`:531` |

当前 `VisualCadencePlanner` 与 `BackendUpdateTrigger` 中已经存在一些 preliminary thresholds 和简化 information proxy。它们是现有实现素材，不是本设计已经接受的最终阈值或研究结论。后续实现必须按本文件的信号、时间、uncertainty、lifecycle 和 threshold-identification contract 逐项收敛；本任务不修改这些文件。

当前证据文件 SHA-256：

```text
c9bc466404cf035294e61e828983206c4402f50b8ae6905ea7301322e983dd37  ov_core/src/track/TrackBase.h
587483416891282367c717f9bac89b9056093731cf413b0f96745b9bbae6e57a  ov_core/src/track/TrackKLT.cpp
2ebdd23caadabd6ad76588b4d8de6fcfcab5fae7409e7fab7a1872af60439239  ov_core/src/feat/FeatureDatabase.cpp
bfadbb8538cf8fc64cb24bc8e657a0174ce31f71697d46fa6de278bd9dd2abe7  ov_msckf/src/core/VisualCadencePlanner.h
f39f86e7c6c9a3618c240f09daf92417ac551a0858b009fdab1f589dae62cb16  ov_msckf/src/core/BackendUpdateTrigger.h
85a4b823a6fccb52d9c896ddd20b6b9852a74934de07f5856783ea73d007c989  ov_msckf/src/core/VioManager.cpp
2549f341081542fb7d44915d11b0b01ec6d0fd52da3c7bd661f9b5bc882f0385  ov_msckf/src/update/UpdaterMSCKF.cpp
```

## 4. 总体数据流

```text
raw camera timestamp t_k
        |
        +--> causal IMU rotation prediction to t_k
        +--> latest estimator/navigation/height health with timestamp+age+uncertainty
        |
        v
VisualMotionPredictor
  - predicts raw / rotational / compensated flow for candidate gaps
        |
        v
SafetyCapEvaluator
  - KLT total-flow cap
  - rotational-flow cap
  - compensated-flow cap
  - ground-footprint overlap cap when available
  - feature-health cap
  - estimator-health cap
  - maximum tracking interval
        |
        v
TrackingCadencePlanner
  - choose next integer raw-frame gap directly
        |
        +--> not due: no KLT, no FeatureDatabase mutation
        |
        v
existing OpenVINS TrackKLT
        |
        v
VisualMotionEstimator
  - direct common-ID metrics from last tracked frame
  - direct common-ID metrics from last actual backend frame
        |
        v
BackendUpdateTrigger
  - compensated parallax
  - feature survival/termination risk
  - geometry/information proxy
  - elapsed time / starvation
  - estimator/safety emergency
        |
        +--> tracking-only:
        |      exact-timestamp FeatureDatabase cleanup
        |      no propagation, clone, MSCKF or SLAM
        |
        +--> backend:
               IMU propagate -> clone -> MSCKF -> SLAM -> cleanup
```

P4 初始化期间可复用同一 `VisualMotionEstimator` 与 tracking planner，但 P4 的 alignment-frame selection 和 P4 Ceres attempt eligibility 是另外两个决策，不得与 P5 backend trigger 混成一个阈值。

## 5. 时间与因果合同

### 5.1 三个时间点

每个 decision 必须区分：

- `raw_timestamp`：当前 raw image 的 sensor timestamp；
- `motion_measurement_timestamp`：产生最近一次 KLT common-ID measurement 的 tracking timestamp；
- `decision_timestamp`：控制器做决定的 causal time。

所有输入必须满足：

```text
sample.timestamp <= decision_timestamp
```

不能用下一帧、未来 turn segment、最终 trajectory error 或离线 reference 判断当前是否处理图像。

### 5.2 必须使用实际时间

```text
raw_dt      = current_raw_timestamp - previous_raw_timestamp
tracking_dt = current_tracking_timestamp - previous_tracking_timestamp
backend_dt  = current_tracking_timestamp - last_actual_backend_timestamp
```

previous recommended stride 不能代替任何一个 `dt`。若 raw camera interval 抖动，预测 horizon 必须由实际 timestamp 与候选 raw-frame gap 共同计算。

### 5.3 两个独立 reference snapshot

- `last_tracking_snapshot`：最近一次实际完成 KLT 的 frame；用于估计 KLT motion rate 和下一次 tracking gap；
- `last_backend_snapshot`：最近一次真正创建 clone 的 frame；用于计算 accumulated backend geometry。

二者不能因“计划处理”或“触发意图”提前更新。backend 调用没有真正发生时，`last_backend_snapshot` 保持不变。

## 6. `VisualMotionEstimator`

### 6.1 common-ID measurement

令 `a` 为 reference snapshot，`b` 为当前 KLT snapshot。只使用两帧都存在的 stable feature IDs。对 feature `i`：

```text
u_i^a, u_i^b       raw pixel
x_i^a, x_i^b       normalized bearing coordinates
R_CbCa             rotation mapping previous camera ray into current camera
```

`R_CbCa` 由两时刻之间的 board IMU rotation 与 fixed `T_C_I` 得到：

```text
R_CbCa = R_CI * R_IbIa * R_IC
```

预测纯旋转后的当前 bearing：

```text
x_hat_i^b = project(R_CbCa * unproject(x_i^a))
```

三种图像运动分别为：

```text
d_i_raw  = u_i^b - u_i^a
d_i_rot  = pixel(x_hat_i^b) - u_i^a
d_i_comp = u_i^b - pixel(x_hat_i^b)
```

`d_i_comp` 是本设计用于平移几何的 rotation-compensated parallax。不能把 `||d_i_raw||` 或相邻帧 median flow 的累加值替代它。

### 6.2 robust quantiles

对有效 common tracks 分别计算：

```text
raw:         P50, P75, P90, P95, max
rotation:    P50, P95, image-edge max
compensated: P50, P75, P90, P95
```

tracking planner主要看 total/raw P95、rotation P95/edge max 和 compensated P50/P95；backend trigger 还看相对 last backend frame 的 compensated quantiles。

### 6.3 feature health

```text
N_common  = number of stable IDs present in both snapshots
S_survive = N_common / max(1, min(N_previous, N_current))
```

按 `w_i` 表示 feature reliability 时，计算 effective track count：

```text
N_eff = (sum_i w_i)^2 / max(epsilon, sum_i w_i^2)
```

图像分成固定 normalized grid。令 `p_c` 为有效 common tracks 在 cell `c` 的比例：

```text
C_occ  = occupied_cells / total_cells
H_grid = -sum_c p_c log(p_c) / log(total_cells)
```

还记录：

- track-age P25/P50/P75/P95；
- new/lost/common feature ratios；
- forward-backward KLT error quantiles（若现有 tracker 暴露）；
- RANSAC inlier ratio（若现有路径有）；
- 到 image border 的距离和 candidate horizon 下的 predicted exit risk；
- recent visual residual 与 MSCKF acceptance provenance。

feature count 必须同时报告绝对值和相对 configured budget 的比例，不能用一个固定绝对数跨 camera/config 泛化。

### 6.4 feature reliability weight

第一版用可解释的乘积权重：

```text
w_i = w_track_i * w_residual_i * w_spatial_i * w_age_i
```

每项限制在 `[w_min, 1]`，来源为 causal tracker/backend history：

- `w_track_i`：forward-backward/RANSAC consistency；
- `w_residual_i`：该 feature 最近一次可用 reprojection/innovation consistency；
- `w_spatial_i`：抑制过密 grid cell，保留空间分布；
- `w_age_i`：过短 track 降权，过老且靠近边界的 track 标记 termination risk。

所有权重失效时，不输出“高信息”；进入 `TRACKING_PROTECTION`，强制最密 tracking，并由 maximum backend latency 决定是否建立 backend frame。

### 6.5 uncertainty

compensated flow uncertainty 至少包括 KLT measurement scatter 与 IMU rotation uncertainty：

```text
Sigma_d_i ~= Sigma_KLT_i + J_R_i P_delta_theta J_R_i^T
```

没有 per-feature covariance 时，以 robust scatter 估计 rate uncertainty：

```text
sigma_MAD = 1.4826 * median_i |r_i - median(r)|
r_i       = ||d_i|| / tracking_dt
```

候选 gap 使用 upper confidence bound 进行 safety check，而不是只用 median point estimate：

```text
UCB_q(s) = predicted_q(s) + z_alpha * sigma_q(s)
```

`z_alpha` 和 coverage 目标由后续注册的 threshold-identification 阶段确定。

## 7. `SafetyCapEvaluator`

对每个候选整数 raw-frame gap：

```text
s in {1, 2, ..., max_tracking_stride}
Delta_t_s = predicted timestamp of candidate frame - last tracking timestamp
```

不使用逐级 ladder。planner 可以从当前 gap 直接跳到任意更安全或经确认的候选 gap。

### 7.1 KLT total-flow cap

候选必须满足：

```text
UCB(raw_P95(s)) <= klt_total_flow_limit
```

这个 cap 保护 LK pyramid/search radius 和 forward-backward consistency。阈值由当前 camera/KLT 配置的数据识别，不引用其他论文的 pixel threshold。

### 7.2 rotation cap

board IMU 预测 candidate horizon 内的 camera rotation。除了在当前 common feature locations 上计算 `rotation_P95`，还在 image grid 与四角计算 edge maximum：

```text
UCB(rotation_P95(s)) <= rotation_flow_limit
rotation_edge_max(s) <= rotation_edge_limit
```

高 angular rate 会通过预测图像运动缩小 gap；不直接通过 `TURN -> stride 1` 查表。

### 7.3 compensated-flow cap

```text
UCB(compensated_P95(s)) <= compensated_flow_limit
```

它防止平移导致的长距离 KLT failure。target band 用 compensated P50/P75，安全上限用 P95。

### 7.4 ground-footprint overlap cap

只有在以下量均有效时才计算：

- online-allowed AGL/ground plane；
- source、timestamp、age、uncertainty、validity；
- camera intrinsics 与 fixed `T_C_I`；
- causal attitude/velocity estimate；
- candidate horizon。

对每个 image corner ray `r_j^C`：

```text
r_j^G = R_GC * r_j^C
lambda_j = (h_ground - p_C,z^G) / r_j,z^G
g_j = p_C^G + lambda_j * r_j^G
```

四个 `g_j` 形成当前 footprint polygon。使用 predicted translation 和 IMU rotation 得到 candidate footprint，并计算：

```text
overlap(s) = area(P_now intersect P_predicted) /
             area(P_now union P_predicted)
```

对 AGL、attitude 与 velocity uncertainty 做 sigma-point/bounds evaluation，采用 conservative lower overlap。若 ground ray 不向地面、AGL stale、ground model unavailable 或 polygon degenerate：

```text
overlap_status = unavailable
```

此时不伪造 overlap、不使用一维 `2*h*tan(FOV/2)` 代替；依靠 actual flow、rotation、feature health 与 maximum interval。

### 7.5 feature-health cap

候选 gap 必须使预测的：

- `S_survive` 不低于 registered limit；
- `N_eff`、`C_occ`、`H_grid` 不低于 registered limit；
- predicted border/termination risk 不高于 limit；
- motion measurement 不 stale；
- common-ID sample size 足以支持 quantile uncertainty。

feature health 已经恶化时立即缩小 gap，不等待 upshift/downshift dwell。

### 7.6 estimator-health cap

以下 causal signals 可把 cap 直接降到最密 tracking：

- covariance non-finite 或 negative diagonal；
- innovation/reprojection residual persistent high；
- MSCKF accepted update starvation；
- recent state correction jump；
- timestamp/frame contract error；
- latest motion measurement stale。

可恢复的视觉/estimator health 问题进入保护模式；不可恢复的 timestamp/frame/calibration contract error 由上层停止处理并明确报告，不能继续输出猜测 stride。

### 7.7 maximum interval 与 realtime cap

```text
Delta_t_s <= maximum_tracking_interval
```

CPU load、queue depth 和 measured KLT/backend deadline margin 可以进一步缩小安全集合，但不能放宽任何几何/health cap。换言之，resource controller 只能在 `S_safe` 内选择，不能为了实时性跳过一个已经被安全规则要求处理的 frame。

## 8. `TrackingCadencePlanner`

### 8.1 flow-rate prediction

从最近两次实际 KLT measurements 得到 causal rate：

```text
r_comp_q = compensated_q / tracking_dt
r_raw_q  = raw_q / tracking_dt
r_rot_q  = rotation_q / tracking_dt
```

对候选 `s`：

```text
predicted_comp_q(s) = r_comp_q * Delta_t_s
predicted_raw_q(s)  = r_raw_q  * Delta_t_s
predicted_rot_q(s)  = r_rot_q  * Delta_t_s
```

constant-rate 只用于短 horizon；uncertainty 随 horizon 增长。若 angular acceleration、rate change 或 timestamp jitter 超出 model validity，缩小候选范围或强制 dense。

### 8.2 target parallax selection

定义 registered tracking target band：

```text
[P_track_min, P_track_max]
```

先由所有 caps 得到：

```text
S_safe = {s | all available safety conditions pass}
```

然后：

```text
S_band = {s in S_safe |
          P_track_min <= predicted_comp_P50(s) <= P_track_max}

if S_band is not empty:
    s_target = max(S_band)
else if every s in S_safe is below P_track_min:
    s_target = max(S_safe)
else if S_safe is not empty:
    s_target = min(S_safe)
else:
    s_target = 1
    emergency = true
```

选择 `max(S_band)` 的含义是：在 KLT 与 safety 条件都通过时，避免处理仍然几乎没有新增平移几何的 raw frames。若 stride 1 自身也超过 flow/rotation limit，仍使用 stride 1 并报告 `emergency_dense_over_limit`，因为不存在更密的软件选择。

### 8.3 hysteresis

- downshift：任一当前 cap violation、health degradation 或 candidate prediction over-limit 时立即执行；
- upshift：要求 `K_up` 个新的、相互独立的 KLT motion measurements 连续支持相同或更大的 target，且持续 `T_up`；
- 没有新 KLT measurement 的 skipped raw frames 不增加 confirmation count；
- upshift 可直接跳到最终确认的 gap，不做 1→2→4→6 的 ladder recovery；
- target band 使用 enter/exit margin，避免在边界单点反复切换；
- reset/recovery reason 必须写入 decision record。

### 8.4 mode 只表达质量语义

允许的 planner mode：

| Mode | 含义 | 是否固定输出 stride |
|---|---|---|
| `BOOTSTRAP_DENSE` | 尚无足够 common-ID rate/uncertainty | 否；通常由 safety set 只剩最密候选 |
| `GEOMETRY_CONTROLLED` | target band 与全部 caps 有效 | 否；按公式选择 |
| `TRACKING_PROTECTION` | survival/coverage/border/flow 异常 | 否；由 violated cap 限制 |
| `ESTIMATOR_PROTECTION` | covariance/innovation/starvation 异常 | 否；由 health cap 限制 |

这些 mode 不是 HIGH/NORMAL/TURN 状态表。roll、pitch、height 或 flight phase 不直接确定 `s_target`。

### 8.5 输出

`TrackingCadenceDecision` 至少包含：

```text
decision_timestamp
reference_tracking_timestamp
next_tracking_deadline
selected_raw_gap
target_band
predicted raw/rotation/compensated quantiles + uncertainty
per-cap maximum gap and availability
mode
primary_reason
all active caps
immediate_downshift
upshift_confirmation_count/duration
input provenance and age
```

## 9. `BackendUpdateTrigger`

### 9.1 调用时机

该 trigger 只在当前帧已经完成 KLT 后调用。它使用：

- `last_backend_snapshot`；
- current tracking snapshot；
- 两者之间的 board IMU rotation；
- current causal estimator health；
- backend elapsed time；
- current FeatureDatabase/track lifecycle risk。

相对 backend 的 parallax 必须用两端 common IDs 直接计算，禁止把多个相邻 tracking interval 的 median magnitudes 相加。

### 9.2 cheap geometry/information proxy

对每个 common feature：

```text
snr_i = ||d_i_comp||^2 / (sigma_i^2 + epsilon)
g_i   = snr_i / (1 + snr_i)
```

用第 6.4 节的 reliability weight 得到：

```text
N_eff = (sum w_i)^2 / sum(w_i^2)
Q_geom = N_eff * C_occ * H_grid * S_survive * weighted_median(g_i)
```

`Q_geom` 是可解释、无状态 mutation 的 lightweight proxy。它不能独立证明 filter information rank，因此同时报告 compensated parallax、spatial coverage、common tracks 和 survival，不能只留下单一 score。

### 9.3 optional shadow information score

在后续 threshold-identification 阶段，可以只读计算 tentative geometry Jacobian：

```text
I_vis = sum_i w_i J_i^T R_i^-1 J_i
Delta_info = logdet(I_prior + I_vis + epsilon I) -
             logdet(I_prior + epsilon I)
```

其中 current pose 只用 IMU-propagated state 作 linearization，tentative landmarks 不进入 filter。该 score 初期只作 shadow evidence；只有证明与 MSCKF accepted information/conditioning 一致后才可成为 active trigger。不能为了计算 score 创建第二套 optimizer 或 map。

### 9.4 trigger 条件与优先级

按优先级评估：

```text
1. FATAL_CONTRACT_ERROR
2. ESTIMATOR_EMERGENCY
3. FEATURE_TERMINATION_RISK
4. MAXIMUM_BACKEND_LATENCY
5. INFORMATION_READY
6. ACCUMULATING_INFORMATION
```

`INFORMATION_READY` 至少要求：

```text
valid direct common-ID geometry
compensated parallax reaches registered backend target
N_eff / survival / coverage pass
pure_rotation == false
and (Q_geom >= threshold or registered information condition passes)
```

安全触发可由以下任一条件产生：

- long tracks approaching image border or predicted termination；
- common-track survival 快速下降；
- visual residual/acceptance/covariance abnormal；
- maximum backend latency；
- high rotation/low altitude 等因素通过具体 flow/overlap cap 证明当前 frame 不能再等，而不是由状态名直接触发；
- first backend frame after initialization。

### 9.5 pure rotation

```text
pure_rotation_candidate =
    raw_P95 is high
    and rotation_P95 explains most raw motion
    and compensated_P50/P75 remains below translation-information floor
```

此时：

- tracking planner 可立即缩小 gap 保护 KLT；
- `Q_geom` 不得因 raw flow 大而升高；
- 不以 parallax/information reason 触发 backend；
- maximum latency、feature termination 或 estimator emergency 仍可触发 backend，reason 必须明确是 fallback/safety，不是假称三角化信息充足。

### 9.6 maximum latency 与 starvation

使用两级时间：

```text
T_backend_warn: 进入 must-update-on-next-valid-tracked-frame
T_backend_max:  当前 tracked frame 强制 backend
```

`T_backend_max` 的目的不是保证视觉一定有信息，而是防止 clone/update starvation 和 covariance 长期只传播。若当前视觉完全不可用，仍记录 `forced_without_visual_information`；不能制造 feature residual。

### 9.7 输出

`BackendUpdateDecision` 至少包含：

```text
decision_timestamp
last_actual_backend_timestamp
elapsed_backend_s
trigger
primary_reason and secondary_reasons
direct raw/rotation/compensated quantiles
N_common, N_eff, survival, track age, spatial coverage
Q_geom and optional Delta_info
pure_rotation_candidate
feature_termination_risk
latency_warn/max status
estimator-health inputs
reference snapshot IDs and provenance
```

## 10. OpenVINS feature 数据生命周期

### 10.1 raw frame 未到 tracking deadline

```text
no TrackKLT call
no FeatureDatabase write
no clone
no MSCKF/SLAM
```

只更新 raw timestamp、IMU prediction 和 scheduler clocks。

### 10.2 tracking-only frame

```text
TrackKLT updates previous image/keypoints/stable IDs
current observations are temporarily written
VisualMotionEstimator computes direct metrics
BackendUpdateTrigger returns false
cleanup_measurements_exact(current_timestamp)
return before propagate_and_clone
```

当前 fork 的实际分支位于 `ov_msckf/src/core/VioManager.cpp:2849`，精确清理位于 `:2858`、`:2866`。后续实现优先考虑在写入正式 backend database 前区分 tracking-only；若继续采用“先写后精确清理”，必须保留完整 lifecycle integration tests。

### 10.3 backend frame

```text
TrackKLT
BackendUpdateTrigger returns true
IMU propagate to image time
augment clone
normal lost/marginalized/SLAM/MSCKF feature selection
MSCKF update
SLAM update / delayed initialization
FeatureDatabase cleanup
clone marginalization
commit last_backend_snapshot only after the backend path actually runs
```

### 10.4 no double-use

- tracking-only timestamp 不得留在任何 feature measurement vector；
- 没有 clone 的 timestamp 不得成为 MSCKF observation time；
- MSCKF 已使用/拒绝的 feature 继续遵循 `to_delete`；
- trigger 的 sidecar snapshot 不等于 estimator measurement，不进入 covariance；
- backend reference 只在实际 clone success 后推进；
- scheduler reset 不能复活已删除 observation 或复用已融合 track segment。

## 11. 高度、速度、姿态与资源信号的正确角色

### 11.1 `HeightEstimate` contract

若 P5 使用 AGL/ground information，输入必须包含：

```text
value
frame/datum
source
source_timestamp
age
standard_deviation or bound
validity
ground_model_id
```

高度只进入 ground-footprint overlap 与 uncertainty。没有可靠 AGL 时 overlap unavailable，不按 height band 输出 stride。

### 11.2 velocity/attitude/angular rate

- velocity direction 与 magnitude 进入 candidate footprint 与 translational-flow prediction；
- attitude 与 fixed `T_C_I` 进入 ground-ray geometry；
- board IMU angular rate/integration 进入 rotation flow prediction；
- descent 只通过 velocity、AGL uncertainty 和 predicted overlap 影响 cap；
- turn 只通过 predicted rotation、flow、feature health 和 estimator health 影响 cap。

### 11.3 resource profile

可用信号：

- KLT/backend measured wall time；
- deadline miss；
- queue depth；
- CPU utilization；
- diagnostic/dashboard load separately measured。

resource profile 只能：

- 在多个同样安全的 candidate gaps 中选择更省计算者；
- 限制 optional diagnostics；
- 报告当前策略无法满足硬件 deadline。

它不能放宽 KLT/rotation/overlap/health/latency cap，也不能以“计算量增加”直接否决一个仍满足实时 deadline 的方案。

## 12. 配置与阈值识别方案

### 12.1 当前任务不注册数值阈值

论文中的 20 px、0.5 s、60% overlap 等只描述原方法。当前源码中的 preliminary defaults 也不自动成为正式参数。本项目阈值必须使用相同 Camera–IMU calibration、KLT 配置、image resolution、fixed baseline contract 与实际 timestamp 数据识别。

### 12.2 复用既有 4 flights × 8 fixed-stride evidence

不重新运行 32 组 estimator sweep。先清点既有产物是否真实包含：

- raw camera dt、actual tracking/backend dt；
- raw/rotation-compensated common-ID flow；
- survival、track age、spatial coverage；
- triangulation conditioning；
- residual、innovation、MSCKF accepted/rejected；
- covariance、state jump、update starvation；
- height source/provenance；
- actual stride 与 health outcome。

缺失字段必须标记 `not_recorded`，不能用 previous recommended stride、aggregate report 或 flight final error反推伪造。若需要补充 common-ID sidecar，只能在用户批准的后续阶段做不改变 estimator decision 的 shadow instrumentation/reprocessing；本研究任务不运行它。

### 12.3 leave-one-flight-out

四折分别执行：

```text
train = three flights
held_out = remaining flight
```

每折只用 train flights 识别 thresholds，在 held-out flight 上一次性评价。不得用 held-out future outcome 回调参数；每飞失败单独记录，不能用总体均值覆盖。

### 12.4 各阈值的识别目标

| 参数 | 识别方法 | 在线禁止输入 |
|---|---|---|
| `P_track_min/max` | compensated parallax bin 与 KLT survival、triangulation success、MSCKF innovation quality 的稳定区间；取四折保守交集 | GPS XY/course、最终 drift |
| KLT raw/rotation/compensated P95 limit | flow bin 对 forward-backward error、RANSAC inlier、survival collapse 的 change point/保守 quantile | future feature outcome |
| minimum survival / `N_eff` / coverage | update acceptance 与 triangulation conditioning 的 lower confidence bound | reference error列 |
| maximum tracking interval | first interval where KLT survival/uncertainty显著恶化；同时满足 sensor timing contract | recommended stride替代的 dt |
| backend parallax target | accepted update information/conditioning 随 parallax 的收益拐点 | full-flight final error直接调参 |
| `Q_geom` / `Delta_info` target | 与 MSCKF accepted feature count、innovation rank、covariance reduction 的 calibration | GPS reference在线输入 |
| maximum backend latency | propagation-only covariance growth、accepted-update starvation 和 track termination risk | flight end window |
| overlap limit | 仅在 valid ground polygon 上，按 survival/flow failure 的 conservative fold threshold | 伪造 AGL/一维 footprint |
| upshift confirmation/dwell | 控制 false upshift、switch rate 与安全 violation；计数单位是独立 KLT measurements | skipped raw-frame count |

### 12.5 threshold provenance

每个正式参数必须保存：

```text
name, unit, definition
source runs and hashes
feature/camera/KLT configuration
training flights
held-out flight
fit method
confidence interval
selected conservative value
failure cases
registration date/version
```

GPS reference可以在 offline evaluation 中评价导航效果，但永远不进入 online planner/trigger feature vector。

## 13. 诊断与决策日志

每个 raw frame 一行 scheduler event；每个实际 KLT frame 一行 motion event；每个 backend decision 一行 trigger event。建议字段：

```text
raw_timestamp
decision_timestamp
last_tracking_timestamp
last_backend_timestamp
actual_raw_dt / tracking_dt / backend_dt
selected_tracking_gap
tracking_due / backend_triggered
raw/rotation/compensated P50/P75/P90/P95
uncertainty and UCB
common tracks / survival / N_eff / track-age quantiles
grid occupancy / entropy / border risk
overlap value/status/source/age/uncertainty
per-cap maximum gap
Q_geom / optional Delta_info
pure_rotation_candidate
primary/secondary reasons
upshift confirmation state
estimator covariance/innovation/update-health summary
KLT/backend wall time and deadline margin
```

日志必须把 `recommended_gap`、`actual_tracking_gap` 和 `actual_backend_gap` 分开，不能用推荐值覆盖实际值。

## 14. 伪代码

```text
on_raw_frame(meta, image):
    causal = collect_causal_imu_navigation_health(meta.timestamp)
    prediction = predict_candidate_motion(last_tracking_snapshot, causal)
    caps = evaluate_safety_caps(prediction, causal)
    track_decision = tracking_planner.select(caps, prediction)
    log_raw_decision(track_decision)

    if not track_decision.due_now:
        return

    current_snapshot = TrackKLT(image)
    motion_track = direct_common_id_motion(last_tracking_snapshot,
                                           current_snapshot,
                                           causal.imu_rotation)
    motion_backend = direct_common_id_motion(last_backend_snapshot,
                                             current_snapshot,
                                             causal.imu_rotation_since_backend)

    backend_input = build_backend_input(motion_backend,
                                        feature_health,
                                        estimator_health,
                                        actual_elapsed_time)
    backend_decision = backend_trigger.evaluate(backend_input)

    last_tracking_snapshot = current_snapshot

    if not backend_decision.trigger:
        FeatureDatabase.cleanup_measurements_exact(meta.timestamp)
        assert(no_clone_at(meta.timestamp))
        return

    run_existing_openvins_backend(meta.timestamp)
    last_backend_snapshot = current_snapshot
```

## 15. 后续实现队列

本任务只交付设计。用户确认后，后续实现按以下依赖顺序进行。

### Stage 0：contract freeze

- 固定坐标、timestamp、reference snapshot 与 input provenance；
- 定义 `VisualMotionEstimate`、`SafetyCaps`、`TrackingCadenceDecision`、`BackendUpdateDecision`；
- 明确现有 preliminary classes 中哪些字段保留、重命名或仅作兼容；
- 注册 no-GPS-XY/course/no-reference-input invariant。

### Stage 1：measurement 与 shadow instrumentation

- 复用 current `TrackKLT` stable IDs；
- 完成 uncertainty、grid coverage、feature reliability、border risk；
- 实现 direct last-tracking 与 last-backend measurement；
- ground footprint unavailable 语义；
- 保持 actual estimator cadence 不变，只输出 shadow decisions。

### Stage 2：`TrackingCadencePlanner` shadow

- 整数 candidate gap、target band、所有 safety caps；
- direct jump、immediate downshift、new-measurement upshift confirmation；
- realtime deadline/resource margin 只在安全集合内作用；
- 用已有 evidence 做 leave-one-flight-out threshold identification，不重跑 4×8 sweep。

### Stage 3：`BackendUpdateTrigger` shadow

- direct accumulated compensated parallax；
- `Q_geom`、pure-rotation gate、termination risk；
- warning/max latency；
- optional `Delta_info` 只作 shadow；
- 对比 shadow trigger 与现有真实 clone/MSCKF/SLAM events。

### Stage 4：lifecycle verification

- skipped raw frame 无 KLT/DB/clone；
- tracking-only 保持 tracker ID continuity；
- exact timestamp 无漏删/误删；
- tracking-only 不创建 clone、不进 MSCKF/SLAM；
- no observation double-use；
- backend reference 只在 actual clone 后更新；
- pure rotation 不冒充 translation information；
- maximum tracking/backend latency；
- stale/missing input、invalid overlap、estimator emergency；
- hysteresis 不由 skipped frames 伪计数。

### Stage 5：minimal active screening

- 只在 shadow 与 lifecycle tests 通过后接入 active；
- 先验证 realtime deadline、KLT survival、clone/MSCKF health 和 decision causality；
- dashboard/diagnostic overhead 单独对照；
- 不以计算量增大直接判失败，只看 wall/sensor 与 target hardware deadline。

### Stage 6：flight validation

- 后续由用户批准具体 flights/windows；
- 相同 calibration、frontend/backend、no-post-alignment contract；
- 每飞单独报告 tracking band occupancy、backend reasons、feature survival、MSCKF health、wall/sensor 与 navigation effect；
- 完整航程或 release 状态不在本设计任务中执行或判定。

## 16. 设计验收清单

- [x] `TrackingCadencePlanner` 与 `BackendUpdateTrigger` 是两个决策；
- [x] 目标量是 rotation-compensated translational parallax 与信息/健康度；
- [x] 直接比较 stable common IDs，不累计相邻 median magnitudes；
- [x] 使用实际 timestamp，不用 previous recommended stride 代替 dt；
- [x] height/speed/attitude/rate 只进入 prediction/caps；
- [x] ground overlap 使用 full footprint polygon，invalid 时明确 unavailable；
- [x] pure rotation 不冒充 triangulation information；
- [x] tracking-only 不改变 clone/MSCKF/SLAM 生命周期；
- [x] maximum tracking interval 与 maximum backend latency 明确；
- [x] downshift immediate、upshift 只按新的 KLT measurement 确认；
- [x] 不使用 RL/learned frontend/第二套 backend；
- [x] 阈值识别使用既有 evidence 和 leave-one-flight-out，不重跑 32 组；
- [x] 当前没有把设计写成已验证实现。

## 17. 当前状态

`P5_RESEARCH_AND_ALGORITHM_FRAMEWORK_COMPLETE_DESIGN_ONLY`

`NOT_IMPLEMENTED_NOT_FLIGHT_VALIDATED`
