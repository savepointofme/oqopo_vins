# P4/P5 代码级调研、决策与实现报告

日期：2026-07-17

基线分支：`redesign/p4-sliding-window-r1-20260714`

基线标签：`baseline/p4-p5-chatgpt-work-20260717`

基线 commit：`a96540c68f4211039741ff461883f18479aa9dec`

本 patch 状态：`IMPLEMENTED_AWAITING_NATIVE_AND_FLIGHT_VALIDATION`

> 本报告把“原始代码事实”“由代码和公式导出的结论”“本项目设计选择”以及
> “尚未验证”分开。阅读了标题或摘要但没有进入固定 commit 的实现，不计入
> 代码证据。当前容器没有 CMake、Eigen 和 Ceres，且没有 fly1–fly4 数据，因此
> 本报告不声称 P4/P5 正式通过；它记录已经完成的实现和仍需在 CI/飞行数据上
> 关闭的验收项。

## 1. 结论

基线不是正式 P4/P5：runner 实际选择的是 upstream OpenVINS dynamic
initializer，再做 FC yaw/translation gauge；旧联合图则使用每关键帧独立
`q/p/v/bg/ba`、显式 landmark 重投影、互相独立的 FC 行和高重叠窗口稳定性。
它们都不满足已经确认的正式合同。

本 patch 已把正式入口改成：

1. 单个有限因果窗口内，每关键帧估计 `q/p/v`，窗口只拥有一对 shared
   `bg/ba`；
2. 完整复用 OpenVINS 15D CPI residual/covariance，不抽成错误的 9D 因子；
3. FC 用一个 terminal absolute + chronological increments 的稠密相关 PVA
   likelihood，冻结 `Phi/Qd/Σ/A`，不把相邻 FC 输出当独立绝对观测；
4. 单目视觉使用 landmark-free Sampson residual；每条 feature track 只选一个
   最大时距 pair，不重复使用同一像素；
5. P3 接受的 mount/time 在 P4 固定；只传播已声明的 attitude time uncertainty，
   不在首转弯重新估计 calibration；
6. candidate 冻结后用随后约 2 s 因果数据做 immutable holdout；holdout 不反馈
   FC，只能拒绝或授权一次新终点 joint refinement；
7. refinement 在当前 camera horizon 生成 terminal `q/p/v/bg/ba + 15x15`
   covariance，并原子注入、永久关闭 P4；
8. P5 的 feature-termination trigger 现在来自真实 track survival 和 image-border
   occupancy，而不再永久保持默认 `false`。

实现骨架已完成，但“正式 P4/P5 通过”仍必须等待 GitHub 原生构建、C++ tests、
Monte Carlo covariance consistency 以及 fly1/fly3 + fly2/fly4 holdout。

## 2. 调研方法和证据等级

### 2.1 固定方法

对每个外部工作执行同一套检查：

- 固定仓库和 commit；
- 从入口函数追到实际 optimizer/residual/parameter blocks；
- 区分论文宣称与公开代码已实现能力；
- 记录 state ownership、bias ownership、global alignment 时机、视觉 nuisance
  处理和 covariance 交接；
- 只把能从原代码、原论文公式或明确推导证明的结论写成事实。

### 2.2 标签

| 标签 | 含义 |
| --- | --- |
| **CODE FACT** | 固定 commit 的实际调用链或参数块直接可见 |
| **PAPER FACT** | 作者原论文公式/正文直接支持，但未必进入公开代码 |
| **DERIVATION** | 由明确随机变量或 Jacobian 推导得到 |
| **DESIGN** | 针对本项目约束作出的选择，不冒充已有工作的结论 |
| **PENDING** | 必须靠原生构建、Monte Carlo 或 flight data 验证 |

## 3. 六套相关实现的代码级审查

### 3.1 OpenVINS upstream dynamic initializer

固定版本：`rpng/open_vins@69488123ed9362dd44b6f28e7f4680abbff1442b`。

关键代码：

- [`DynamicInitializer.cpp` 的每时刻状态块和 CPI/视觉图](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/dynamic/DynamicInitializer.cpp#L694-L806)
- [`Factor_ImuCPIv1.cpp` residual 与 Jacobian](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/ceres/Factor_ImuCPIv1.cpp#L77-L263)
- [`DynamicInitializer.cpp` terminal result/covariance 交接](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_init/src/dynamic/DynamicInitializer.cpp#L927-L994)

**CODE FACT**：它为窗口中的每个时刻建立 `q,p,v,bg,ba`，用完整 15D CPI
连接相邻状态，并建立显式 landmark/reprojection；最终返回 terminal camera
timestamp 和 terminal marginal covariance。

**CODE FACT**：CPI residual 顺序不是简单的 `theta,v,p` 九维。它包含 orientation、
两个 bias endpoint difference、velocity 和 position，bias 还通过预积分 Jacobian
进入其它 residual row。

**DERIVATION**：如果短窗只拥有一个 shared `bg` 和一个 shared `ba`，正确适配是
让 upstream factor 的两个 bias endpoint 指针指向同一变量，并按链式法则相加：

\[
J_{b_g}^{shared}=J_{b_{g,i}}+J_{b_{g,j}},\qquad
J_{b_a}^{shared}=J_{b_{a,i}}+J_{b_{a,j}}.
\]

删除 bias-difference row 或只取九维子块会改变 whitening 与 cross-covariance，
不等价。

**DESIGN**：本 patch 直接包装 `Factor_ImuCPIv1`，不复制第二套 preintegration，
也不声称 OpenVINS upstream 图本身就是正式 FC-assisted P4。

### 3.2 VINS-Mono

固定版本：`HKUST-Aerial-Robotics/VINS-Mono@90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`。

关键代码：

- [`Estimator::initialStructure()` 到 `visualInitialAlign()`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp#L218-L390)
- [`solveGyroscopeBias()`、`LinearAlignment()`、`VisualIMUAlignment()`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/initial/initial_aligment.cpp#L3-L207)

**CODE FACT**：调用链先做视觉 SFM/结构恢复，再由
`VisualIMUAlignment()` 先解 gyro bias，再解 velocity/gravity/scale。它是分阶段的
local monocular VI initialization，没有 FC PVA 从第一帧进入同一个全局图。

**DESIGN**：可借鉴 staged seed 和 excitation 检查，不能直接复用为正式 P4，
更不能用它证明 FC correlation 或 terminal global covariance。

### 3.3 ORB-SLAM3

固定版本：`UZ-SLAMLab/ORB_SLAM3@4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`。

关键代码：

- [`Tracking::CreateInitialMapMonocular()`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2448-L2608)
- [`LocalMapping::InitializeIMU()`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1173-L1309)
- [`Optimizer::InertialOptimization()`](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Optimizer.cc#L3042-L3153)

**CODE FACT**：先建 monocular visual map；之后 IMU initialization 在已有 keyframe
poses 上优化 velocity、shared biases、gravity direction 和 scale。它证明“短窗 shared
bias + staged initialization”是合理工程结构，但不是 FC-assisted joint-from-start。

**DESIGN**：本 patch 采用 shared short-window bias 的 ownership 思路；不复制其
地图、g2o 或尺度初始化。

### 3.4 GVINS

固定版本：`HKUST-Aerial-Robotics/GVINS@d2cf40b49c6eb0e6ad3caa4f613713983be3fd74`。

关键代码：

- [`initialStructure()`、`visualInitialAlign()`、`GNSSVIAlign()`](https://github.com/HKUST-Aerial-Robotics/GVINS/blob/d2cf40b49c6eb0e6ad3caa4f613713983be3fd74/estimator/src/estimator.cpp#L334-L640)
- [`GNSSVIInitializer` coarse/yaw/anchor 三阶段](https://github.com/HKUST-Aerial-Robotics/GVINS/blob/d2cf40b49c6eb0e6ad3caa4f613713983be3fd74/estimator/src/initial/gnss_vi_initializer.cpp#L16-L173)

**CODE FACT**：GVINS 先完成 VINS-style local visual-inertial initialization，之后
`GNSSVIAlign()` 做 GNSS coarse localization、yaw alignment 和 anchor refinement。

**结论**：它是高价值的 global-frame/alignment 参考，但公开调用链不是 FC PVA
从初始化开始进入每关键帧 `q/p/v` 的图。把 GVINS 当作“当前正式 P4 已有现成
实现”是不准确的。

### 3.5 MINS

固定版本：`rpng/MINS@d0e0ea2dbdd0f9f0b46c69ffbf94fe1971d747fa`。

关键代码：

- [`UpdaterGPS::try_initialization()`](https://github.com/rpng/MINS/blob/d0e0ea2dbdd0f9f0b46c69ffbf94fe1971d747fa/mins/src/update/gps/UpdaterGPS.cpp#L172-L248)
- [`UpdaterGPS::transform_state_to_ENU()` 与 marginalization](https://github.com/rpng/MINS/blob/d0e0ea2dbdd0f9f0b46c69ffbf94fe1971d747fa/mins/src/update/gps/UpdaterGPS.cpp#L439-L497)

**CODE FACT**：MINS 在滤波器状态中初始化/更新 GPS transform，并在坐标变换时
显式处理 state ownership 与 marginalization。它不是本项目的 batch startup graph。

**DESIGN**：本项目只借鉴“坐标变换、状态所有权和原子操作必须显式”的原则；
不把滤波 GPS update 伪装成 P4 joint initializer。

### 3.6 DRT-VIO-Init

固定版本：`boxuLibrary/drt-vio-init@fb0ac8d3fc4d9f683888565882838b6f1c330435`。

关键代码：

- [`drtVioInit` 构造函数把 `biasa/biasg` 清零](https://github.com/boxuLibrary/drt-vio-init/blob/fb0ac8d3fc4d9f683888565882838b6f1c330435/src/initMethod/drtVioInit.cpp#L35-L42)
- [`gyroBiasEstimator()` 只把 `biasg` 作为 Ceres 参数](https://github.com/boxuLibrary/drt-vio-init/blob/fb0ac8d3fc4d9f683888565882838b6f1c330435/src/initMethod/drtVioInit.cpp#L346-L439)
- [`drtTightlyCoupled` 用 `solved_bias(biasg,biasa)` 重积分](https://github.com/boxuLibrary/drt-vio-init/blob/fb0ac8d3fc4d9f683888565882838b6f1c330435/src/initMethod/drtTightlyCoupled.cpp#L21-L45)

原论文：[DRT-VIO-Init, CVPR 2023](https://openaccess.thecvf.com/content/CVPR2023/papers/He_A_Rotation-Translation-Decoupled_Solution_for_Robust_and_Efficient_Visual-Inertial_Initialization_CVPR_2023_paper.pdf)。

**CODE FACT**：rotation stage 的公开 optimizer 只估 `biasg`；`biasa` 从构造函数
置零后进入后续 `solved_bias`，没有在该 rotation estimator 中成为优化变量。

**PAPER/CODE BOUNDARY**：DRT 支持 rotation/gyro-bias seed 的研究依据；它不证明
本项目的 shared accelerometer bias、FC PVA likelihood、mount/time calibration 或
terminal 15×15 covariance。

**DESIGN**：production patch 不 vendor DRT。未来若引入，只能先做固定输入的
residual/Jacobian golden equivalence 和 GPLv3 处理，然后作为 rotation/bg seed；
它不能替代 OpenVINS CPI。

## 4. Structureless 视觉证据和能力边界

原论文：[Structureless Visual-Inertial Bundle Adjustment, arXiv 2502.16598](https://arxiv.org/abs/2502.16598)。

**PAPER FACT**：structureless formulation 能在消去/不显式维护 landmark 的情况下
保留多视图几何信息。

**几何边界**：两视图 essential constraint 为

\[
r = \frac{x_j^T[t]_{\times}R x_i}
{\sigma_n\sqrt{(Ex_i)_1^2+(Ex_i)_2^2+(E^Tx_j)_1^2+(E^Tx_j)_2^2}}.
\]

它直接约束 pose 的相对旋转和 translation direction；单独不提供 metric scale，
也没有 velocity parameter。metric position/velocity 信息来自 FC 与 IMU chain，
视觉通过 pose coupling 改善轨迹几何。报告和 metadata 不再写“视觉直接完整观测
metric p/v”。

同一 track 如果进入多个 pair，共享像素噪声会被重复当独立观测。本 patch 选择
确定性 policy：每 feature 只使用 first/last（最大时距）一个 pair；每个像素在该
track likelihood 中出现一次。更强的 track-wise structureless factor 可以作为后续
研究，但必须显式处理共享观测 covariance。

## 5. FC 相关噪声合同

### 5.1 为什么基线的独立行不成立

FC 连续 PVA 是同一导航滤波器输出。相邻 increment
`z_i-z_{i-1}` 与 terminal absolute `z_N` 共享 underlying endpoint；如果把两者分别
加成独立 residual block，就会重复计算信息。只写 `PSD * Δt` 也没有定义 absolute
state covariance 和 cross-covariance。

### 5.2 本 patch 冻结的第一版模型

FC error state：

\[
e=[\delta\theta,\delta p,\delta v]\in\mathbb{R}^9,\quad
\Phi_i=I_9.
\]

给定完整窗口 terminal covariance `P_T` 和 process fraction `f∈(0,1)`：

\[
P_0=(1-f)P_T,\qquad
Q_{d,i}=fP_T\frac{\Delta t_i}{T},
\]

\[
\operatorname{Cov}(e_i,e_j)=P_0+
\sum_{k\le \min(i,j)}Q_{d,k}.
\]

构造 absolute joint covariance `Σ_abs`，再用可逆变换 `A` 形成 residual ordering：

\[
r_{raw}=A e=
[e_N, e_1-e_0, e_2-e_1,\ldots]^T,
\quad
Σ_r=AΣ_{abs}A^T.
\]

单个 Ceres factor 用 `L^{-1}r_raw`，其中 `LL^T=Σ_r`。这保留 terminal/increment
相关性；将物理时间段均匀细分时，线性 error path 的总 likelihood 不随 keyframe
density 改变。

**DESIGN，不是假装已标定的事实**：`Phi=I` 和 `f=0.25` 是当前 stream 只声明
terminal PVA sigma 时的可审计第一版。`f` 写入 CLI 和 metadata；必须用 FC 原始
estimator covariance/innovation 或 Monte Carlo 再标定，不得把 0.25 写成物理真值。

### 5.3 calibration uncertainty

P3 nominal mount 与 attitude time offset 固定，但它们已声明的 sigma 分别以
isotropic attitude perturbation 和 terminal angular-rate 一阶 Jacobian 加入 FC
attitude covariance。Camera–IMU、FC navigation time 和 lever arm 在当前输入合同
没有独立 uncertainty 字段，因此本版明确记录 treated fixed；不能假装未知
uncertainty 已被传播。

## 6. 高重叠窗口为什么不能作独立验证

8 s 窗每 0.5 s 推进时，相邻窗口共享 93.75% 数据。两次估计差的 covariance 是：

\[
P_{\Delta}=P_c+P_p-P_{cp}-P_{pc}.
\]

基线没有 `P_cp`。因此使用 `P_current+P_previous` 会隐含错误的独立假设，不能
形成 normalized stability gate。本 patch 对 legacy overlap diagnostic 只保留明确
物理上限，并把 normalized value 标为 unavailable；正式 release 的独立因果证据
来自 candidate 后的新 holdout interval。

## 7. 正式生命周期和时间戳

```text
finite common-support window
  -> joint candidate solve
  -> freeze q/p/v/bg/ba/covariance
  -> ~2 s later causal holdout
       IMU propagates frozen candidate
       FC and epipolar only evaluate, never feed back
  -> fail: discard and slide
  -> pass: authorize one current-window joint refinement
  -> release at refinement terminal camera timestamp
  -> atomic inject and permanently close P4
```

`AlignmentResult.timestamp` 是 terminal camera-clock `t_end_C`；注入的 IMU state
物理时刻对应 `t_end_C + dt_CI`。这与 OpenVINS state 的 camera-clock `_timestamp`
约定一致。当前项目原子交接在
`VioManagerHelper.cpp::initialize_with_online_alignment()` 中写入 state、15×15
covariance 和 `_timestamp`；本 patch 没有回写到旧 seed timestamp。

`NAVIGATION_READY` 或 `FULL_ALIGNMENT_READY` 都是一次性 release terminal，P4
随后关闭。二者区别只在 readiness/weak-state 记录，不能先 NAV release 后再期待
同一个 P4 升级 FULL。

## 8. 基线到本 patch 的实际调用链变化

| 项目 | 基线 | 本 patch 正式入口 |
| --- | --- | --- |
| 初始状态来源 | upstream dynamic init | P4 joint graph，从未初始化状态开始 |
| FC 角色 | 后置 yaw/translation gauge | 从第一窗进入 q/p/v trajectory likelihood |
| 状态拓扑 | legacy 图每帧 q/p/v/bg/ba | 每帧 q/p/v + window-shared bg/ba |
| IMU | full CPI，但 endpoint bias 独立 | full 15D CPI，endpoint bias tied/summed |
| FC covariance | terminal/增量独立 | 一个 dense correlated trajectory factor |
| 视觉 | 显式 landmark reprojection | landmark-free one-pair-per-track epipolar |
| P3 mount/time | 旧图可混入估计 | fixed external calibration；shadow diagnostics |
| candidate 验证 | recursive FC feedback 或 overlap | immutable later causal holdout |
| refinement | 语义混合 | holdout 只授权一次 advanced joint solve |
| release time | 多路径 | current terminal camera timestamp |
| P5 termination | input 从未填充 | real track survival/border risk |

## 9. 实现文件清单

### 9.1 新文件

- `ov_msckf/src/core/p4/factors/Factor_P4ImuSharedBias.h/.cpp`
- `ov_msckf/src/core/p4/factors/Factor_P4FcTrajectory.h/.cpp`
- `ov_msckf/src/core/p4/factors/Factor_P4Epipolar.h/.cpp`
- `ov_msckf/src/test_p4_formal_factors.cpp`
- `ov_msckf/scripts/validate_p4_formal_contract.py`

### 9.2 修改文件

- `OnlineAlignmentInitializer.h/.cpp`：formal state machine、shared state graph、
  dense FC、epipolar pairs、dimensionless observability、terminal covariance；
- `VioManager.cpp`：formal fail-closed path 和 P5 termination 接线；
- `BackendUpdateTrigger.h`：termination risk 函数与 scheduler config exposure；
- `run_serial_msckf_ros_free.cpp`：正式入口、CLI、日志和 metadata；
- `ROS1.cmake` / `ROS2.cmake`：sources 和原生 test target；
- `test_online_alignment_initializer.cpp`：formal lifecycle/atomic handoff tests；
- `test_adaptive_stride.cpp`：healthy、track loss、border termination tests；
- 本冻结规格：把未实现的模块拆分计划改成实际 patch 边界。

## 10. 测试合同与当前结果

### 10.1 当前容器已执行

`validate_p4_formal_contract.py` 独立检查：

- FC terminal covariance 精确保留；
- dense covariance SPD；
- terminal/increment correlation 非零；
- coarse/fine keyframe density likelihood 等价；
- runner formal path 打开且 upstream gauge 关闭；
- shared CPI endpoint 指针绑定和 Jacobian 求和；
- immutable holdout、one-refinement receipt、P5 termination 和 camera-clock handoff。

当前结果：`20 checks passed`。

### 10.2 已加入但当前容器不能执行的 C++ tests

`test_p4_formal_factors`：

- shared-bias adapter residual 与原完整 CPI 在 tied endpoints 下逐元素相等；
- shared bg/ba Jacobian 等于两个 endpoint columns 之和；
- FC covariance SPD/correlation/terminal exact/density invariant；
- FC q/p/v Jacobian 对独立 JPL finite difference；
- epipolar exact geometry、off-epipolar rejection 和 q/p Jacobian finite difference。

`test_online_alignment_initializer`：candidate -> 2 s holdout -> exactly one advanced
refinement -> current camera timestamp atomic release；release 后不再求解。

`test_adaptive_stride`：healthy track set 不触发，track loss 和 border concentration
触发 termination risk。

### 10.3 必须在 GitHub/目标机关闭的 gates

1. ROS-free/ROS1/ROS2 原生编译；按项目约定并行构建使用 `-j12`；
2. 全部 C++ tests；
3. synthetic Monte Carlo：PVA density、shared-bias recovery、epipolar Jacobian、
   terminal 15×15 NEES/coverage；
4. fly1/fly3 调通，fly2/fly4 holdout；
5. 主指标为 absolute navigation / no post alignment；start-heading 和 best-fit
   只作诊断；
6. P4 单独通过后再启用 P5；不得用 P5 或 AGL postprocess 掩盖 P4 误差。

## 11. 已明确拒绝的实现捷径

- 不把 upstream dynamic init + yaw/translation gauge 改名为正式 P4；
- 不把完整 CPI 截成九维 residual；
- 不把 FC 每行或 terminal/increment 当独立观测；
- 不用 `P_current+P_previous` 给高重叠窗口差作 normalized gate；
- 不让每个 feature 的多个 pair 重复使用相同 pixel；
- 不宣称 epipolar 直接观测 velocity 或独立提供 metric scale；
- 不在 P4 重新打开 P3 mount/time；
- 不把 DRT translation/accelerometer-bias 能力写成公开代码已实现；
- 不用固定次数、未来真值或最终轨迹误差决定在线 release；
- 不把静态脚本通过写成 native build/flight pass。

## 12. 实施与验收计划

| 步骤 | 内容 | 当前状态 | 完成证据 |
| --- | --- | --- | --- |
| I0 | 固定 baseline/tag/commit，审计真实生产入口 | 完成 | commit identity + call-chain diff |
| I1 | 固定外部工作 commit，逐函数核对 state/factor/ownership | 完成 | 本报告第 3–4 节 permalinks |
| I2 | 冻结 state/time/frame/lifecycle/noise contracts | 完成 | 修订规格 + 本报告第 5–7 节 |
| I3 | 实现 shared CPI、dense FC、sparse epipolar 和 formal lifecycle | 完成 | source diff + static/math tests |
| I4 | 补齐 P5 real termination input 与测试 | 完成 | scheduler/VioManager/test diff |
| I5 | GitHub native compile/test，修复所有编译或 Jacobian failures | 待 CI | Actions logs 全绿 |
| I6 | Monte Carlo covariance consistency | 待目标环境 | NEES/coverage 报告 |
| I7 | fly1/fly3 + fly2/fly4 absolute/no-post 验收 | 待数据环境 | 每 fly、每窗口数值表 |
| I8 | P4 单独通过后组合 P5 | 待 I7 | combined ablation |

当前可以准确称为“正式合同的实现候选版本”；在 I5–I8 关闭前，不能称为
“P4/P5 正式通过版本”。
