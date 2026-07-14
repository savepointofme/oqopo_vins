# P5 自适应视觉调度公开研究

## 1. 文档定位

本文只做公开论文、作者项目页和官方仓库调研，并据此界定后续 P5 设计可以借鉴什么、不能移植什么。本文不是实现报告、参数调优记录或实验结论。当前工作区中的 P5 源码、配置、baseline 和运行产物均未因本研究修改。

统一逐项研究表见 [`P5_PUBLIC_RESEARCH_EVIDENCE.csv`](P5_PUBLIC_RESEARCH_EVIDENCE.csv)。该表包含 17 篇直接相关论文，并逐项记录 estimator family、visual frontend、decision target、输入信号、公式/逻辑、hysteresis、fallback、maximum latency、计算/精度目标、官方代码及许可证、可复用代码、只能借鉴的思想和不兼容部分。

研究日期：2026-07-14。

当前 fork 证据基准：Git `HEAD=3d9296c34d19767b967e12d3ec8fbaf18d1ec1f3`，同时存在未提交的用户工作。本文以实际工作区文件内容而不是仅以 commit 为准。

## 2. 研究结论

公开工作共同支持三个结论。

第一，tracking frame 与 backend-selected frame 本来就是两个不同决策。VINS-Mono、Keyframe-based Structureless Filter、IC-GVINS、HybVIO、Kimera 等系统都会持续保持视觉轨迹，但只在满足几何、特征存活或时间条件时选择关键帧/更新帧。这个思想可以转化为 OpenVINS 的 `TrackingCadencePlanner` 与 `BackendUpdateTrigger`，但不能连同它们的 bundle-adjustment window、地图和 marginalization 一起移植。

第二，raw optical flow 不能直接代表可三角化的平移视差。VINS-Mono 论文、IC-GVINS、RD-VIO 和当前 fork 的 IMU rotation warp 都说明，应先用 IMU 与锁定的 Camera–IMU 外参预测纯旋转图像运动，再对相同 feature ID 计算 rotation-compensated translational parallax。纯旋转时可以需要更密的 KLT，却不能据此声称后端已经获得足够平移几何。

第三，任何信息驱动选择都必须有安全兜底。IC-GVINS 用最大无关键帧时间，FAST-LIVO2 resource-constrained 版本在主传感器退化时提高视觉频率，resource-aware adaptation 使用 frame-drop cooldown，HybVIO 限制 update attempts 并避免重复使用 track segment。P5 因此必须同时具备 maximum tracking interval、maximum backend latency、feature-loss risk、estimator-health emergency 和保守恢复滞回，不能只追求减少 CPU。

## 3. 统一研究概览

| # | 工作 | Estimator family | Decision target | 最关键的可迁移内容 | 结论 |
|---:|---|---|---|---|---|
| 1 | OpenVINS | filter / MSCKF | admitted frame、clone、feature update | `TrackKLT`、stable IDs、`FeatureDatabase`、FEJ/covariance、structureless update 生命周期 | 直接复用当前架构，不另建 VIO |
| 2 | MSCKF | filter | multi-pose feature constraint | landmark nullspace elimination、一次性消费 track | 规定 backend frame 的状态与 feature 生命周期 |
| 3 | S-MSCKF | filter | bounded feature/update workload | KLT/grid 与受限 update size | 证明 frontend cadence 是主要算力变量之一 |
| 4 | VINS-Mono | optimization | keyframe | average parallax、low-track-count force | 借鉴判据；不移植滑窗后端 |
| 5 | Resource-aware adaptation | policy over filter/optimization | frame、feature count、iterations/window | motion/resource 双输入、frame-drop cooldown、parameter dwell | 资源只作约束，不替代几何安全规则 |
| 6 | KSF | keyframe structureless filter | ordinary frame / keyframe | convex-hull overlap、match ratio、structureless consistency | 最接近 tracking/backend 分离的 filter 参考 |
| 7 | KSWF | keyframe sliding-window filter | keyframe/state retention | low-parallax/standstill 下的 keyframe state 管理 | 可借鉴退化语义，不移植 self-calibration state |
| 8 | IC-GVINS | factor graph | keyframe / observation frame | INS-assisted tracking/triangulation、rotation-compensated parallax、0.5 s fallback | 借鉴先验几何与 maximum latency |
| 9 | FAST-LIVO2 resource-constrained | ESIKF hybrid | visual frame | degeneration confirmation、dense fallback、adaptive threshold | 借鉴安全优先级；论文 selector 未在审阅的 official repo 中找到 |
| 10 | Visual-Selective VIO | learned recurrent | visual modality | 视觉可以按需跳过 | 只作系统级对照；不引入网络 |
| 11 | Redesigning SLAM | optimization | keyframe | information matrix log-determinant、relative running baseline | 借鉴相对 information gain，不移植地图/BA |
| 12 | HybVIO | filter + optional SLAM | feature-track visual update | 禁止重复使用已融合 track segment、优先长轨迹、bounded attempts | 直接约束 OpenVINS feature reuse 设计 |
| 13 | DynaVINS++ | robust optimization | keyframe | reliability-weighted parallax、all-zero-weight fail-safe | 借鉴残差可靠性权重，不移植 ATLS/恢复后端 |
| 14 | OKVIS | keyframe optimization | keyframe/window | frontend/backend 参数分离、fixed-lag compute reference | 作为资源策略对照，不移植 optimizer |
| 15 | Kimera-VIO | keyframe smoothing | intermediate frame / keyframe | 中间帧 tracking、关键帧 detection/stereo/backend | 支持双 cadence 思想，不移植 factor graph/mesh |
| 16 | RD-VIO | optimization | keyframe / subframe | 纯旋转检测、deferred triangulation | 纯旋转只触发安全，不冒充平移信息 |
| 17 | Adaptive keyframe-threshold VIO | MSCKF-derived filter | keyframe threshold | new-feature ratio 随 mean track length 自适应 | 借鉴归一化 survival 与数据拟合，不照搬 polynomial |

## 4. 强制调研对象的公式与代码核对

### 4.1 OpenVINS：必须保留的生命周期

官方 OpenVINS 是 EKF/MSCKF filter：IMU state 与 camera clones 位于 filter state 中，3D feature 不长期加入 MSCKF state，而是通过 feature Jacobian nullspace projection 消去。当前 fork 继续使用同一套基本结构。

当前工作区中可直接复用的入口为：

- `TrackerWarpVizPacket` 保存 previous/current raw points、rotation-warped points、stable feature IDs 和实际时间戳：`ov_core/src/track/TrackBase.h:54`；
- `TrackKLT::feed_monocular()` 执行现有 KLT 并以相同 ID 写入 raw/normalized observation：`ov_core/src/track/TrackKLT.cpp:129`、`ov_core/src/track/TrackKLT.cpp:303`；
- `FeatureDatabase::cleanup_measurements_exact()` 可精确删除 tracking-only timestamp：`ov_core/src/feat/FeatureDatabase.cpp:245`；
- backend frame 进入 `VioManager::do_feature_propagate_update()`：`ov_msckf/src/core/VioManager.cpp:3052`；
- MSCKF feature 数受 `max_msckf_in_update` 限制：`ov_msckf/src/core/VioManager.cpp:3434`；
- `UpdaterMSCKF` 完成 nullspace projection 后将已消费或拒绝的 feature 标为 `to_delete`：`ov_msckf/src/update/UpdaterMSCKF.cpp:395`、`ov_msckf/src/update/UpdaterMSCKF.cpp:531`。

因此 P5 不能新建第二套 tracker、database、clone window 或 filter。它只允许控制“是否运行现有 KLT”和“已 tracking 的当前帧是否进入现有 clone/MSCKF/SLAM 路径”。官方来源：[OpenVINS paper](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf)、[OpenVINS repository](https://github.com/rpng/open_vins)（GPL-3.0）。

### 4.2 VINS-Mono：论文与公开代码不完全一致

论文关键帧思想是：相对上一参考关键帧计算平均视差，tracking 数过少时强制保留当前帧，并在论文描述中用短期 IMU preintegration 补偿旋转。官方仓库的 `FeatureManager::addFeatureCheckParallax()` 公开实现包含：

```text
if last_track_num < 20:
    choose current frame as keyframe

parallax_sum = sum(compensatedParallax2(feature))
choose keyframe if average_parallax >= MIN_PARALLAX
```

但官方代码的 `compensatedParallax2()` 中，旋转补偿表达式被注释，实际执行 `p_i_comp = p_i`。所以本 P5 不能照抄函数名并声称已做 gyro compensation；必须使用当前 board IMU、固定 `T_C_I`、真实 timestamp 与 stable feature IDs 明确计算 `R_Ccurrent_Cprevious`。官方来源：[VINS-Mono paper](https://arxiv.org/abs/1708.03852)、[official repository](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)（GPL-3.0）。

可借鉴：相对上一参考帧的视差、feature count 降低时强制选择、纯旋转不等于有效平移视差。不可移植：VINS-Mono bundle-adjustment sliding window、marginalization、地图和 keyframe manager。

### 4.3 Resource-aware Online Parameter Adaptation for VINS

该工作把 motion agility 与 CPU profile 分开。运动量以固定历史长度内 angular velocity 与 linear acceleration 的均值描述；资源差值 `Delta` 驱动 frame processing count、feature count、iteration count 和部分 optimization window size。其 online policy 还维护：

```text
kappa_f = frames since last dropped frame
kappa_p = iterations since last parameter change
```

只有超过各自 cooldown 才允许再次 drop frame 或改变参数。对 S-MSCKF，在线改变 feature count 与 iteration count；对 VINS-Mono，主要改变 feature count；对 OKVIS，online 部分主要是 motion-based frame dropping，其他重参数多在运行前固定。

这证明资源信号可以约束计算策略，也证明需要 cooldown/dwell。但 CPU load 不能决定三角化是否安全，motion magnitude 也不能代替 rotation-compensated parallax。P5 第一版应先满足几何与 estimator-health 安全，再在安全集合内考虑资源 margin。论文未提供可核对的官方实现仓库，因此没有可直接复制代码。[Paper](https://arxiv.org/abs/2106.00289)。

### 4.4 A Versatile Keyframe-Based Structureless Filter

KSF 明确区分 ordinary frames 与 keyframes，同时保持 structureless filter consistency。对每个 camera，论文计算：

- `o_k`：已关联 landmark 的 2D feature convex hull 与全部 2D feature convex hull 的 area overlap；
- `r_k`：convex hull 内已关联 landmark feature 数与全部 feature 数之比。

当 `max_k(o_k) < T_o`（论文典型 60%）或 `max_k(r_k) < T_r`（论文典型 20%）时选 keyframe。它的 backend 继续使用 completed feature tracks，并按 ordinary/keyframe 关系管理 motion states。

可迁移的是“tracking frame 不必等于 selected backend frame”、spatial coverage 与 feature association health；不可迁移的是 KSF 自己的 descriptor frontend、keyframe state layout、rolling-shutter/self-calibration states。未在论文和作者公开页中找到可直接复用的 official code。[Paper](https://arxiv.org/abs/2012.15170)。

### 4.5 IC-GVINS

IC-GVINS 用 INS prior 初始化 LK search，并用 INS pose 辅助 landmark triangulation。其 keyframe selection 使用 rotation-compensated average parallax；论文给出的示例阈值是 20 px。若约 0.5 s 没有新 keyframe，则插入一次 observation frame 参加优化，随后移除，以避免长期没有视觉约束。

官方代码中的 `tracking/tracking.cc` 读取 `track_min_parallax`，并基于 INS poses 计算 reference/current parallax；triangulation 还使用独立的 minimum parallax 与正深度/重投影检查。

本 P5 可借鉴：IMU-assisted rotational prediction、严格 triangulation geometry、maximum backend latency。不可移植：GNSS factor、INS-centric factor graph、其 keyframe queue 和 optimization backend。[Paper](https://arxiv.org/abs/2204.04962)、[official repository](https://github.com/i2Nav-WHU/IC-GVINS)（GPL-3.0）。

### 4.6 FAST-LIVO2 on Resource-Constrained Platforms

论文先对 LiDAR normal matrix 做归一化 SVD：

```text
[sigma_min_tilde, sigma_mid_tilde, sigma_max_tilde]
    = Normalize(SVD(n_est n_est^T))
```

当 `sigma_min_tilde` 连续多帧低于阈值时确认 LiDAR degeneration；退化时处理全部视觉帧，正常时稀疏选帧。normal 状态的 pose-change threshold 为：

```text
tau = sqrt(3) * sigma_min_tilde * tau_predefined
```

可迁移的是：退化必须持续确认、退化时立即提高视觉频率、恢复要保守。其主传感器是 LiDAR，退化量不能直接替代 OpenVINS covariance/innovation/feature health。

源码状态必须明确：本次检查了 official `hku-mars/FAST-LIVO2` 的 `main` branch，commit `0d2c0346107b75b59934975adec9a6eeeb913c64`。仓库引用了该论文，但未找到论文中的 degeneration-aware adaptive selector 或上述公式实现。因此只能借鉴思想，不能声称 selector 已公开、可直接复制。[Paper](https://arxiv.org/abs/2501.13876)、[official FAST-LIVO2 repository](https://github.com/hku-mars/FAST-LIVO2)（GPL-2.0）。

### 4.7 Visual-Selective VIO

该方法是 learned recurrent VIO。policy 使用当前 IMU embedding 与上一时刻 recurrent hidden state 生成视觉使用概率：

```text
p_t = Phi(h_{t-1}, x_i^t)
```

训练用 Gumbel-Softmax，推理时做二元视觉选择；跳过视觉时向 pose network 输入 zero-padded visual embedding。loss 在 pose loss 外增加视觉使用惩罚：

```text
L = L_pose + lambda/(T-1) * sum_t d_t
```

它支持“视觉不是每时刻都必须完整处理”的系统思想，但决策依赖训练分布、learned feature/recurrent backend，不能作为当前单目 KLT/OpenVINS 的第一版控制器。official repository commit `27b06b38dcf07a015fba22038050113462387e7d` 中可见 `PolicyNet` 与 `gumbel_softmax`；仓库根目录没有许可证文件，因此不得复制代码到正式实现。[Paper](https://arxiv.org/abs/2205.06187)、[official repository](https://github.com/mingyuyng/Visual-Selective-VIO)。

### 4.8 信息驱动、uncertainty-aware 与 feature-survival 工作

#### Redesigning SLAM for Arbitrary Multi-Camera Systems

该工作对 pose measurement Jacobian 累积信息矩阵：

```text
I_T = J_T^T Sigma_u^-1 J_T
E(T) = log |I_T|
```

绝对 `E(T)` 随环境变化，因此论文维护当前 local map 中的 running average，并在当前 negative entropy 低于 running average 的一定比例时选 keyframe。P5 可借鉴“相对当前信息基线”的设计，但不能移植其 multi-camera map、voxel map 或 BA。[Paper](https://arxiv.org/abs/2003.02014)。

#### HybVIO

HybVIO 明确防止重复使用已经融合过的 track segment。若 feature `j` 上次在 frame `i'` 用于视觉更新，则当前 update 只允许使用后续 segment：

```text
S(i,j) = {b(i,j)} union {max(S(i',j))+1, ..., i}
```

它还定义 track motion length `L(i,j)`，优先从大于当前集合 median length 的 tracks 中选择，并在达到 target accepted updates 或 maximum attempts 后停止。这些思想与 OpenVINS 的 `to_delete` 和 bounded `max_msckf_in_update` 一致。不可移植的是其 pose trail、独立 SLAM 与随机 track update backend。[Paper](https://arxiv.org/abs/2106.11857)、[official repository](https://github.com/SpectacularAI/HybVIO)（GPL-3.0）。

#### DynaVINS++ weighted parallax

DynaVINS++ 对连续追踪 feature 使用可靠性权重：

```text
theta_avg = sum_j(omega_j * theta_j) / sum_j(omega_j)
```

若全部 `omega_j=0`，论文把它解释为大面积遮挡，并重置其 optimization window。P5 不采用该 reset 行为，也不引入 ATLS；可以借鉴的是用 recent residual、track consistency 和 spatial health 降低不可靠 feature 对 parallax/information proxy 的贡献，以及“全部权重失效时进入 dense/emergency，而不是输出虚假信息”。截至本次调研未找到独立的 DynaVINS++ official code；作者公开的是早期 DynaVINS repository commit `0b4f492aeeb1a3752507a9b9a50124e10bfcf091`，不能视为 DynaVINS++ selector 实现。[Paper](https://arxiv.org/abs/2410.15373)、[DynaVINS official repository](https://github.com/url-kaist/dynaVINS)（GPL-3.0）。

#### RD-VIO

RD-VIO 检测 pure rotation，在平移基线不足时 deferred triangulation，并把 pure-rotation frames 作为 subframes 保存。P5 不移植其 optimization window，但必须采用同样的几何语义：large raw/rotational flow 可能要求 stride 1 保护 KLT，却不能触发“平移信息足够”的 backend reason；最终仍由 maximum latency 或安全事件避免 update starvation。[Paper](https://arxiv.org/abs/2310.15072)、[paper-linked XRSLAM repository](https://github.com/openxrlab/xrslam)。

#### Adaptive keyframe-threshold based VIO

该 MSCKF-derived 工作用“newly observed feature 数 / currently tracked feature 数”作为关键帧判据，并用 mean feature tracking length 拟合该比例阈值。可迁移的是用归一化 survival/new-feature ratio 和 track age，而不是绝对 feature count；不能直接复制论文 polynomial，因为 camera、KLT、飞行高度和 feature budget 不同，应在本项目的 leave-one-flight-out 流程中重新识别。[Publisher record](https://doi.org/10.5302/J.ICROS.2020.20.0075)。

## 5. 其他基础参考

### 5.1 MSCKF 与 S-MSCKF

原始 MSCKF 用多个 camera poses 对同一 static feature 形成约束，并通过 nullspace 消除 feature state，使计算复杂度对 feature 数近似线性。这个结构决定：只有创建了 clone 的 frame 才能成为正式 measurement time；已经用于 update 的 track segment 不能再次进入 filter。[MSCKF paper](https://intra.ece.ucr.edu/~mourikis/papers/MourikisRoumeliotis-ICRA07.pdf)。

S-MSCKF 在 autonomous flight 中使用 KLT/grid、受限 feature/update 规模，并报告 frontend 是主要计算开销之一。它支持调整 tracking cadence 的必要性，但没有提供适用于本项目的 target-parallax dual-cadence controller。[Paper](https://arxiv.org/abs/1712.00036)、[official repository](https://github.com/KumarRobotics/msckf_vio)。

### 5.2 KSWF、OKVIS 与 Kimera-VIO

KSWF 说明 keyframe-based filter state management 能处理 low-parallax/standstill，但其 full self-calibration 与 rolling-shutter state 不是本项目目标。[Paper](https://arxiv.org/abs/2201.04989)。

OKVIS 和 Kimera-VIO 都是 keyframe optimization/smoothing family。它们可以作为“intermediate tracking 与 selected backend work 分离”以及 resource-profile 的对照，但不能替换 OpenVINS EKF/MSCKF、FEJ 和 covariance management。[OKVIS official repository](https://github.com/ethz-asl/okvis)、[Kimera paper](https://arxiv.org/abs/1910.02490)、[Kimera-VIO official repository](https://github.com/MIT-SPARK/Kimera-VIO)。

## 6. 开源可复用边界

| 类别 | 可以直接复用 | 只能借鉴 | 不采用 |
|---|---|---|---|
| 当前 OpenVINS fork | `TrackKLT`、stable IDs、raw/normalized observations、`FeatureDatabase`、IMU propagation、clone/MSCKF/SLAM、FEJ/covariance | sidecar metrics 与非状态化 information proxy | 第二套 tracker/filter/map |
| VINS-Mono / IC-GVINS | 无需复制代码；当前 fork 已有所需 KLT/IMU 数据 | rotation-compensated parallax、low-track force、maximum latency | BA window、marginalization、GNSS factor graph |
| KSF / KSWF / HybVIO | 无跨项目代码直接引入 | ordinary/selected frame、coverage、survival、anti-reuse、bounded attempts | 它们的 state layout、self-calibration 或 SLAM backend |
| FAST-LIVO2 resource | 无可核对 selector code | confirmed degradation、dense fallback、recovery | LiDAR degeneration metric 直接套用 |
| Visual-Selective VIO | 无；official repo license 未声明 | modality skipping 的系统思想 | learned policy、training pipeline、learned backend |
| DynaVINS++ / RD-VIO | 无 | reliability weighting、pure rotation/deferred geometry | ATLS、window reset、state recovery backend |

许可证说明只描述官方仓库当前可见状态，不构成法律意见。后续实现应优先使用本仓库已有代码；若确需引入第三方代码，必须另行做许可证与逐文件来源审查。

## 7. 对当前 P5 需求的直接约束

调研后，P5 的正式目标应写成：

1. `TrackingCadencePlanner` 使相邻实际 KLT frames 的 raw/rotational/compensated flow 落在经验安全区间；
2. `BackendUpdateTrigger` 等待相对上一个实际 backend frame 的直接几何基线与信息量，同时避免 feature termination 和 update starvation；
3. height、speed、attitude、angular rate、descent 与 flight phase 只能影响 motion/overlap prediction 和 safety caps，不能直接查表输出固定 stride pair；
4. CPU/resource margin 只能在几何与 filter 安全集合内选择更经济的方案；
5. pure rotation、invalid AGL、stale flow、low survival 和 covariance/innovation 异常必须有显式语义，不能被一个平均分数掩盖；
6. 每个决定必须记录 observation time、decision time、reference frame、actual interval、trigger/cap 和 source provenance；
7. 研究中的典型阈值只用于说明原论文，不是本项目参数。正式阈值必须按后续注册的 leave-one-flight-out 方案识别。

完整可实现框架见 [`P5_OPENVINS_ALGORITHM_FRAMEWORK.md`](P5_OPENVINS_ALGORITHM_FRAMEWORK.md)。

## 8. 当前研究状态

`P5_RESEARCH_AND_ALGORITHM_FRAMEWORK_COMPLETE_DESIGN_ONLY`

`NOT_IMPLEMENTED_NOT_FLIGHT_VALIDATED`
