# P4 在线联合初始化：文献、开源实现与架构审查

日期：2026-07-17

> **状态：已由代码级报告取代。** 本文是架构调研草案，其中没有把所有结论
> 逐项绑定到固定 commit、入口函数和 parameter blocks，不能单独作为实现或
> 验收依据。正式依据改为
> [P4_P5_FORMAL_CODE_AUDIT_AND_IMPLEMENTATION_REPORT_20260717.md](P4_P5_FORMAL_CODE_AUDIT_AND_IMPLEMENTATION_REPORT_20260717.md)
> 与修订后的
> [P4_FORMAL_INITIALIZATION_IMPLEMENTATION_SPEC_20260717.md](P4_FORMAL_INITIALIZATION_IMPLEMENTATION_SPEC_20260717.md)。

## 1. 审查范围

本报告只回答 P4 应当建立在什么算法架构上，不修改代码、参数、P5、GPS-Z/AGL 后处理或评价口径。

调研对象分为四类：

1. 单目视觉—惯性初始化；
2. GNSS/视觉—惯性初始化与全局对齐；
3. 飞行中 INS/GNSS 粗对准与主从 INS 传递对准；
4. 多 IMU 旋转、时间偏移、杆臂和柔性形变标定。

只把论文原文、作者/机构页面和官方开源仓库作为主要证据。博客、二次转述和评价真值不作为算法依据。

## 2. 结论先行

### 2.1 当前问题的正确分类

P4 不是普通的“单目 VIO 初始化”，也不只是“VIO 完成后对齐到 GNSS”。其物理结构是：

```text
主系统：FC 导航解 q/p/v
          │
          │ 已知/待校正的时间差、安装角、杆臂、短时柔性形变
          ▼
从系统：board IMU + 固定 Camera–IMU 外参 + 单目视觉
```

因此 P4 更接近：

```text
飞行中主从 INS 传递对准
+ FC 约束下的视觉—惯性初始化
+ FC/board 时空关系校准
```

主飞控已经提供 `G_nav` 中的姿态、位置和速度。板载 IMU/相机从系统应当直接在这个度量全局系中初始化，而不是先生成一个尺度可能错误的局部 VIO，再只做 yaw+translation 对齐。

### 2.2 两条已被证据否决的路径

第一条被否决路径是：

```text
upstream OpenVINS dynamic initialization
-> 局部 VIO q/p/v/bg/ba
-> FC 只估 yaw + translation gauge
-> scale 固定 1
```

原因不是实现细节，而是输入前提不成立。GVINS、InGVIO 一类“VIO 后全局对齐”默认局部 VIO 已经具有可信的重力、速度、bias 和 metric scale。当前 fly1/fly3 已经观察到局部初始化速度量级明显错误，此时 yaw+translation 不可能修复 scale、velocity、gravity/bias 耦合。2026 年 Dai–Filin 的 GNSS 约束初始化工作也明确把“先 VIO、后 GNSS 对齐”作为要克服的松耦合局限，并把 GNSS 位置直接放入初始化平移约束。

第二条不能直接作为最终架构的路径是：

```text
每个视觉关键帧独立 q/p/v/bg/ba
+ 显式 landmark
+ IMU preintegration
+ 每帧 FC q/p/v
-> 每个推进窗口重新做完整非线性 Ceres
```

这套图有来源：upstream OpenVINS `DynamicInitializer` 的确为每个初始化状态建立 q/p/v/bg/ba，并用 15 维 CPI 因子和显式 landmark 重投影联合优化。因此不能说“没人这样做”。但它是通用、纯视觉—惯性、最大似然精化图，不等于对本项目最合适的实时 FC 辅助初始化：

- FC 已经提供度量 q/p/v，继续为每帧开放全部绝对状态会增加不必要自由度；
- FC 连续输出来自同一个导航滤波器，相邻行高度相关，不能当成独立绝对量测反复乘信息；
- 每关键帧独立 bias 加显式 landmark 会显著放大窗口维数；
- 安装角、时间偏移和魔术贴柔性形变若混进 q/bg/ba，会把不同物理误差互相吸收；
- 在没有因子复用和边缘化时，推进窗口反复全图重建不满足实时边界。

所以，完整批图可保留为小窗口最终精化器或离线对照，不应继续承担全部在线粗估、校准、验证和滑窗生命周期。

### 2.3 推荐架构

文献支持的正式方向是“FC 直接建立度量全局初值 + 旋转/平移分阶段求解 + 小规模联合精化”，而不是 upstream fallback，也不是反复重建高维全图：

```text
阶段 0：锁定 Camera–IMU 外参
        离线/长窗估计 FC→board nominal rotation 与 time offset
        明确杆臂；输出先验及协方差

阶段 1：主从 PVA 粗传递
        FC q/p/v + 杆臂 + nominal mount/time
        -> 每个关键时刻 board IMU 在 G_nav 的 metric q/p/v seed

阶段 2：旋转子问题
        FC relative rotation / board gyro / visual relative rotation
        -> delta mount rotation + gyro bias + time-offset residual

阶段 3：平移子问题
        board IMU preintegration + FC p/v + visual relative translation
        -> per-keyframe velocity + shared/slow bias + optional small gravity correction
        metric scale 由 FC 绝对平移/速度直接进入方程，不作为事后 gauge

阶段 4：一次有界联合精化
        小窗口 pose corrections + velocity + shared bg/ba + mount residual
        使用 nullspace/pose-only visual constraints，优先不显式保留 landmark

阶段 5：内部一致性验收
        covariance / information / innovation / residual / excitation
        -> 一次性原子注入 q/p/v/bg/ba 和 15×15 covariance
```

这里“分阶段”是为了可观性、数值条件和实时性，不是把 FC 变成事后评价。FC 的度量约束从阶段 1 开始就在初始化内部。

## 3. 文献与官方代码证据矩阵

| 来源 | 实际输入 | 初始化变量/顺序 | 是否直接解决本 P4 | 结论 |
| --- | --- | --- | --- | --- |
| [OpenVINS DynamicInitializer](https://github.com/rpng/open_vins/blob/master/ov_init/src/dynamic/DynamicInitializer.cpp) | camera + IMU | 线性 seed 后，为每关键帧优化 q/p/v/bg/ba，显式 landmark 和视觉重投影，恢复终端协方差 | 部分 | 可复用 CPI、视觉因子、covariance 和状态注入；不能直接提供 FC 传递对准和全局度量约束 |
| [VINS-Mono](https://arxiv.org/abs/1708.03852) / [官方代码](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) | camera + IMU | visual SfM，gyro bias，velocity/gravity/scale 线性对齐，gravity refinement，再进非线性优化 | 否 | 证明初始化常采用分阶段低维求解；没有 FC/GNSS 主系统 |
| [Inertial-Only Optimization](https://arxiv.org/abs/2003.05766) / [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | 已有视觉地图 + IMU | 固定视觉 pose，优化 per-KF velocity、共享 bg/ba、gravity direction 和单一 scale，之后再做全 VIBA | 否 | 强证据：初始化专用问题不必一开始开放每帧全部 pose/bias/landmark |
| [DRT-VIO Initialization](https://openaccess.thecvf.com/content/CVPR2023/html/He_A_Rotation-Translation-Decoupled_Solution_for_Robust_and_Efficient_Visual-Inertial_Initialization_CVPR_2023_paper.html) / [官方代码](https://github.com/boxuLibrary/drt-vio-init) | camera tracks + IMU | rotation-only 估 gyro bias，再解 velocity/gravity/scale；核心解不重建 3D 点 | 部分 | 可复用旋转—平移解耦和无 landmark 约束；必须加入 FC 绝对 q/p/v、杆臂和 mount/time |
| [Online VI Spatial-Temporal Initialization](https://arxiv.org/abs/2004.05534) | camera + IMU | 先 rotation/bg/time，再 scale/gravity/extrinsic translation，最后 ba 和 nonlinear refinement | 部分 | 支持先标时空与旋转、再处理平移/bias；对象是 Camera–IMU，不是 FC–board |
| [GVINS](https://arxiv.org/abs/2103.07899) / [官方代码](https://github.com/HKUST-Aerial-Robotics/GVINS) | raw GNSS + camera + IMU | 先完成 metric VIO，再估 GNSS anchor/yaw，最后联合因子图 | 否 | 其 4DOF 全局对齐以 VIO scale/velocity 已正确为前提；不能为当前失败的局部 VIO兜底 |
| [InGVIO](https://arxiv.org/abs/2210.15145) / [官方代码](https://github.com/ChangwuLiu/InGVIO) | raw GNSS + camera + IMU | 静止 IMU 建 gravity，GNSS Doppler + VIO velocity 估 yaw，再估 ECEF anchor | 否 | 仍要求稳态 gravity 和已有 VIO velocity；不适用于飞行中启动 |
| [IC-GVINS](https://arxiv.org/abs/2204.04962) / [官方代码](https://github.com/i2Nav-WHU/IC-GVINS) | GNSS + INS + camera | GNSS/INS 先建立 metric/global 状态并做 IMU+GNSS 初始化优化，之后才进入视觉初始化 | 是，架构接近 | 强支持“外部导航先建立全局 metric INS，再引入视觉”，但原实现面向轮式平台，heading/静止假设必须替换 |
| [GNSS-Constrained VIO Initialization](https://isprs-annals.copernicus.org/articles/XI-1-2026/143/2026/) | GNSS position + raw visual/inertial constraints | GNSS 位置直接形成绝对平移约束，闭式线性求解，不先做松耦合 VIO | 是，数学方向接近 | 强支持 FC/GNSS 绝对平移直接进入 P4；论文忽略 GNSS-IMU 杆臂，本项目必须保留杆臂 |
| [In-flight Coarse Alignment](https://arxiv.org/abs/1207.1550) | GPS position/velocity + IMU | 速度/位置积分优化求飞行中初始姿态；明确处理 GPS 杆臂 | 是，粗对准接近 | 支持 FC PVA 直接给 board INS 建 metric/global 初值，不需先跑局部 VIO |
| [Transfer Alignment Flexure Study](https://www.cambridge.org/core/journals/journal-of-navigation/article/investigation-of-flexure-effect-on-transfer-alignment-performance/D63C57E4D0A189C9C6A4151FDE9049F0) | master INS PVA + slave INS | 传递 PVA，再用 velocity/attitude matching 估 misalignment 和 sensor error；分析 flexure/lever arm | 是，物理问题最接近 | P4 应把 FC 当 master INS，把 board 当 slave；固定安装误差和动态 flexure 必须分开 |
| [Fixed-wing Acceleration Matching](https://snu.elsevierpure.com/en/publications/%EA%B0%80%EC%86%8D%EB%8F%84-%EC%A0%95%ED%95%A9-%EA%B8%B0%EB%B0%98-%EC%A0%84%EB%8B%AC-%EC%A0%95%EB%A0%AC%EC%9D%84-%ED%86%B5%ED%95%9C-%EA%B3%A0%EC%A0%95%EC%9D%B5-%ED%95%AD%EA%B3%B5%EA%B8%B0%EC%9D%98-%EC%9C%A0%EC%97%B0%EC%84%B1-%EC%98%A4%EC%B0%A8-%EB%B3%B4%EC%83%81/) | master/slave INS | acceleration matching 显式考虑 lever-arm/flexure | 是，误差模型接近 | 证明转弯时的角速度、杆臂加速度和柔性形变不能都解释成永久安装角 |
| [mix-cal](https://github.com/jongwonjlee/mix-cal) / [论文](https://arxiv.org/abs/2205.14724) | 两套同步 raw IMU | 用 angular rate、specific force、bias random walk 和杆臂刚体关系估相对位姿 | 部分 | 刚体残差可借鉴；FC 是导航解而非第二套 raw IMU，且其代码不估 time offset，不能直接移植 |
| [MVIS](https://arxiv.org/abs/2308.05303) | 多 IMU/gyro + camera | 联合估 IMU-IMU 时空外参、intrinsics，利用刚体约束并分析退化运动 | 部分 | 给出 mount/time 可观性依据；一轴/平面圆周运动会使部分 rotation/translation/time 参数退化 |
| [Motion Correlation Calibration](https://researchportal.hkust.edu.hk/en/publications/real-time-temporal-and-rotational-calibration-of-heterogeneous-se/) | 高频参考 IMU + 可输出 3D rotation 的目标传感器 | 先运动相关估 time offset，再闭式估 extrinsic rotation | 是，FC attitude→board gyro 接近 | 适合作为 FC–board time/rotation 初始标定框架；需处理 FC 姿态滤波延迟和数值微分噪声 |
| [Kalibr](https://github.com/ethz-asl/kalibr) / [iKalibr](https://github.com/Unsigned-Long/iKalibr) | 多传感器原始数据 | batch/continuous-time 时空标定 | 部分 | Camera–IMU 继续使用已锁定 Kalibr 结果；不能把 Camera–IMU 外参与 FC–board 安装误差混为一体 |

### 3.1 是否存在可直接 clone 后运行的同构实现

没有找到一个官方开源仓库同时满足以下输入合同：

```text
FC navigation solution q/p/v
+ independently sampled board raw IMU
+ monocular KLT tracks
+ known Camera–IMU calibration
+ unknown FC–board mount/time/flex
-> in-flight OpenVINS q/p/v/bg/ba initialization in G_nav
```

可直接运行的仓库分别覆盖了问题的不同部分：

- OpenVINS：完整视觉—惯性 batch 状态和 OpenVINS 注入；
- DRT：快速 rotation/translation initialization；
- IC-GVINS：先用 GNSS/INS 建 metric/global state，再进入视觉；
- mix-cal/MVIS：多 IMU 刚体、时空外参和退化运动；
- ORB-SLAM3：低维 inertial-only initialization 和后续 full VIBA。

因此“直接 clone 一个仓库原样替换 P4”在数据合同上不成立。可执行的复用方式是：直接引用并移植这些仓库中已验证的子问题实现，同时保留本项目 FC PVA、杆臂、时间关系和 OpenVINS state injection。任何移植都必须先核对许可证和坐标/时间/误差状态约定，不能只按论文文字重写后宣称等价。

## 4. 对“每关键帧 q/p/v/bg/ba”的准确结论

此前如果把这条路径简单说成“没有依据”，结论不准确。准确表述如下。

### 4.1 有依据的部分

upstream OpenVINS `DynamicInitializer` 在每个选中时刻建立：

```text
q_k, p_k, v_k, bg_k, ba_k
```

相邻状态由完整 `Factor_ImuCPIv1` 连接，视觉因子连接 pose 和 landmark，首姿态用于 gauge 固定。这是标准的批量 MAP/full-state smoothing 形式。

当前工作树 `OnlineAlignmentInitializer.cpp` 的 `GraphState`、每状态五组 parameter block、15 维 CPI 连接和 landmark 重投影明显继承了这一路线。

### 4.2 不应直接照搬的部分

初始化专用文献存在另一条更常见的低维路线：

- VINS-Mono：先 gyro bias，再 velocity/gravity/scale；
- ORB-SLAM3 inertial-only optimization：视觉 pose 固定，只估 velocity、共享 bias、gravity 和 scale；
- DRT：先 rotation/bg，再 translation/velocity/gravity，避免显式 3D 点；
- 2026 GNSS-constrained initializer：GNSS 绝对平移直接加入线性初始化，不做完整 3D reconstruction/nonlinear refinement。

这些方法的共同点不是“永远不能做 full-state batch”，而是先用低维问题消除最严重的 gauge、scale、gravity、bias 耦合，再决定是否做一次完整精化。

### 4.3 本项目的选择

本项目已有 FC PVA，因此最合理的初始自由度比纯 VIO 更少：

- q/p/v 已有由 FC、杆臂、mount/time 转换得到的 metric/global seed；
- 初始化窗口首先需要估计的是相对小修正、bias 和模型不一致，而不是从零重建整条局部轨迹；
- bg/ba 在数秒窗口内应首先采用 shared 或低阶随机游走模型，只有证据表明 bias drift 必须分节点后再开放；
- 视觉优先提供 relative pose/nullspace 约束，不必在每个推进窗口重复优化全部 landmark；
- 最终可保留一次小规模 full-state refinement，但它不能是每次新帧都从头重建的主循环。

这不是以“越少变量越好”为目的，而是让每个自由度都有独立物理来源和可观证据。

## 5. FC 信息应如何进入初始化

### 5.1 不能把 FC 只当事后 gauge

若局部 VIO 的 scale、velocity 或 gravity/bias 已错，4DOF yaw+translation 只会旋转和平移错误轨迹。GVINS/InGVIO 的后对齐之所以可行，是因为它们明确先完成 VIO metric initialization。

对本项目，FC 应至少进入：

1. 关键时刻 board q/p/v seed；
2. 旋转相对运动约束；
3. 位置和速度的绝对终端约束；
4. 窗口内的增量约束或等价的相关噪声模型；
5. scale/gravity/bias 的低维初始化方程。

### 5.2 FC 连续行不是独立真值

FC q/p/v 是飞控导航滤波器的输出，不是彼此独立的原始测量。把每个关键帧的 FC q/p/v 都按同一个对角 sigma 当成独立因子，会随关键帧密度人为增加信息。

可接受的设计有三种：

1. 终端 absolute PVA + 窗口 increment factors；
2. 使用 FC 输出协方差和时间相关模型；
3. 以物理时间为基础进行 whitening/decimation，而不是规定固定“10 次有效量测”。

当前代码中的“terminal absolute + density-invariant increments”比逐行独立 absolute factor 更合理，但仍需用合成数据证明：改变视觉关键帧密度时，FC 总信息、估计值和 covariance 不应系统变化。

### 5.3 FC attitude 与 board gyro 不是同类信号

FC 姿态差分得到的 angular rate 包含导航滤波、采样、延迟和微分噪声；board gyro 是原始惯性测量。它们不能直接按“两颗同步 raw IMU”处理。

正确顺序应是：

```text
FC 3D rotation trajectory
-> 连续/平滑相对旋转或稳健差分
-> 与 board gyro/relative rotation 做 motion correlation
-> 先估 time offset
-> 再估 nominal rotation 和 bg
```

安装角标定的评价量是跨窗口刚体一致性、残差、协方差和重复性，不是把 FC 自身当“安装角真值”。

## 6. 安装角、时间差、bias 与柔性形变必须分开

至少存在四种不同物理量：

| 量 | 时间特性 | 正确处理 |
| --- | --- | --- |
| Camera–IMU 外参 | 固定 | 使用 June12/Kalibr 锁定结果，不在 P4 重估 |
| FC–board nominal mounting | 跨飞行基本固定或缓慢重装变化 | 长窗/离线估计，作为在线先验并带 covariance |
| FC–board time offset/latency | 通常固定，但驱动链可能抖动 | 运动相关或连续时间标定；与 rotation 分阶段估计 |
| Velcro/flex residual | 转弯、滚转、振动相关的短时变化 | 作为 transient state/process noise/outlier，不写回永久 mounting |

MVIS 的可观性分析说明：完整 3D 激励可使多 IMU 时空参数收敛；一轴旋转和圆周平面运动会使部分相对 rotation、translation 和 time offset 退化。因此：

- “q 不可观”不能笼统表述。FC 已知姿态时，board navigation attitude 可被锚定；可能退化的是某些 FC–board calibration 分量；
- 一次左转或右转不能作为永久三轴安装角的唯一证据；
- fly1/fly3 反向转弯必须作为成对验证，防止把转弯方向相关 flexure 固化成 mount；
- 在线 mount residual 必须回到 nominal，而不能无限随机游走。

## 7. 建议的 P4 状态与因子

### 7.1 固定量

```text
T_C_I              locked Kalibr result
p_IinF             declared/measured lever arm
gravity_G          navigation-frame gravity convention
R_FtoI_nominal     offline/full-flight calibration mean
dt_FI_nominal      offline/full-flight temporal calibration
```

### 7.2 在线初始化变量

首选低维状态：

```text
per keyframe:
  delta_theta_k, delta_p_k, v_k

shared over short window:
  bg, ba
  delta_theta_mount
  delta_t_FI (only if excitation supports it)
  optional delta_g_tangent (2 DoF, only if FC gravity convention is not locked)
```

若合成试验证明 shared bias 无法覆盖窗口，再引入 bias random walk 节点；不能默认每关键帧开放独立 bias。

### 7.3 因子

```text
rotation:
  board gyro preintegration
  visual relative rotation
  FC relative rotation after mount/time transform

translation:
  board IMU preintegration
  visual relative translation / nullspace feature constraint
  FC terminal absolute p/v
  FC density-invariant p/v increments
  lever-arm velocity and acceleration terms

priors:
  nominal mount/time covariance
  bg/ba physical prior
  transient flex robust process/outlier model
```

FC absolute/relative factors 与视觉、IMU 在同一个初始化估计中产生 q/p/v/bg/ba；不存在“先估一个错误 VIO，再把 FC 贴上去”的步骤。

## 8. 求解与滑窗生命周期

### 8.1 求解顺序

建议实现为：

1. 选择一个由真实时间长度定义的因果窗口；
2. 复用上一窗口的 feature tracks、IMU preintegration 和线性化结果；
3. 解 rotation/bg/time 子问题；
4. 解 translation/velocity/gravity/ba 子问题；
5. 必要时做一次有界联合 refinement；
6. 计算终端 15×15 covariance 和各状态内部信息；
7. 不满足条件时窗口前移，不建立固定 candidate replay；
8. 满足条件时一次性原子注入 OpenVINS。

### 8.2 “滑窗”必须满足的合同

```text
新数据到达
-> 窗口按传感器时间前移
-> 新增因子加入，过期因子边缘化/删除
-> 复用未变化区间的 preintegration 和视觉约束
-> 重估对象确实包含待释放 q/p/v/bg/ba
```

仅滑动原始 buffer、但固定一个 candidate 做递归 FC p/v 验证，不属于本合同。

### 8.3 实时性来源

实时性不能靠把编译改成单核或减少验证获得，必须来自问题结构：

- rotation/translation 解耦；
- 不重复三角化相同 track；
- 视觉 nullspace/pose-only 消元；
- shared bias；
- preintegration 缓存；
- Schur/marginal prior 复用；
- 固定时间窗口而非全历史回放。

DRT 报告其 10-keyframe 初始化比所比较方法快 8–72 倍，核心原因正是解耦和避免 3D point reconstruction，而不是降低线程数或隐藏计算。

## 9. AGL/GPS 高度尺度后处理的边界

用户要求的第三种高度方案应继续作为独立输出后处理：

```text
OpenVINS 输出相对高度变化
+ AGL/GPS-Z 相对高度变化
-> 一个标量 scale
-> 对相对位置/速度输出做一致缩放
```

它不是 7DoF Sim(3)，不估旋转和平移，也不应进入 P4 的 q/p/v 初值或用来掩盖 P4 metric scale 失败。

P4 本身有 FC p/v 时仍产生严重 scale 错误，首先说明初始化模型或量测合同错误；不能靠 AGL 后处理宣布 P4 成功。

## 10. 被否决、保留和待验证的设计

### 10.1 明确否决

- upstream dynamic VIO + yaw/translation gauge + scale=1；
- ordinary OpenVINS initializer fallback；
- 用评价 GPS course/yaw 或未来轨迹误差决定在线 release；
- 把 Camera–IMU 外参与 FC–board mounting 混合在线估计；
- 把一次转弯的 flexure 作为永久 mounting；
- 每个 FC 输出行按独立同方差 absolute factor 重复乘信息；
- 固定“10 次量测”替代时间窗口；
- 每个新帧从全历史重建全部 landmark 和 Ceres 图；
- 用 AGL scalar scale 掩盖错误初始化。

### 10.2 保留复用

- OpenVINS `CpiV1`、15 维 IMU factor、JPL quaternion、state/FEJ/covariance injection；
- 当前 KLT/FeatureDatabase，不改 learned frontend；
- FC/IMU/camera 三时间轴和因果插值；
- FC 杆臂位置、速度以及必要的加速度项；
- factor family residual/Jacobian/covariance diagnostics；
- 有界时间窗口、retry 和成功后一次性关闭的生命周期。

### 10.3 必须先用实验决定

- shared bg/ba 是否足够，还是需要低频 bias random walk；
- visual relative rotation 使用 KLT essential/epipolar 还是现有 reprojection 消元；
- mount residual 是否在线优化，还是只使用 offline prior；
- time-offset residual 是否在当前飞行激励下可估；
- terminal absolute + increment FC 模型的 covariance 是否密度不变；
- 小规模 nonlinear refinement 是否在实时预算内带来可重复收益。

## 11. 实现前验收设计

### 11.1 合成验证

必须覆盖：

1. 已知 q/p/v/bg/ba、mount、time offset、杆臂的精确恢复；
2. mount correction 正负方向；
3. 左转和右转镜像轨迹；
4. 候选窗口前、窗口内和窗口后的 transient flex；
5. 一轴/平面圆周退化时，校准 covariance 不虚假收缩；
6. 改变 FC 和视觉采样密度，结果和总信息保持一致；
7. shared bias 与 random-walk bias 的受控对照；
8. feedback 后 residual/innovation 确实下降；
9. 终端 15×15 covariance 有限、对称、半正定；
10. 相同数据增量只处理一次，无 O(history²) 回放。

### 11.2 数据回放

先使用半圈到一圈裁剪窗，再做全程：

- fly1、fly3 作为相反转弯方向主验证；
- fly2、fly4 作为跨轨迹验证，禁止只在 fly1/fly3 过拟合；
- 同一代码、同一配置、同一评价口径比较 frozen global baseline、no-change 和新 P4；
- 正式指标使用 GPS update time、start-heading alignment、绝对导航误差；
- best-fit 只作诊断，不能作为通过依据；
- 同时检查初始化同步时刻的 FC q/p/v、内部 residual/NIS/covariance 和全程轨迹，不以单个终点代替全程。

### 11.3 实时性

- 记录每个阶段和每个推进窗口 wall time；
- 证明计算量只由固定时间窗内关键帧/track 数决定，不随飞行历史增长；
- 证明 preintegration 和视觉约束被复用；
- 1× 回放不得因初始化计算持续落后传感器时间；
- 编译可并行 `-j8/-j12`，编译线程与在线算法实时性无关。

## 12. 对当前工作的直接裁决

1. `P4_UPSTREAM_DYNAMIC_INIT_FC_GAUGE_REFACTOR_SPEC_20260717.md` 的正式方向应撤销；它建立在“局部 VIO 已 metric”的错误前提上。
2. 当前 repeated full-state Ceres 代码不能直接宣布为正式方案；其图有 OpenVINS 依据，但 FC 信息相关性、mount/time/flex 分离和实时复用未获得验证。
3. 下一步不是再调门限，而是先实现一个可单独测试的 FC-assisted staged initializer：rotation/bg/time、translation/v/gravity/ba、bounded refinement 三部分。
4. 在合成测试证明状态方向、可观性、密度不变和实时边界前，不运行新的正式 fly1/fly3 调参实验，也不进入 P5。

## 13. 主要一手来源

- OpenVINS dynamic initializer: <https://github.com/rpng/open_vins/blob/master/ov_init/src/dynamic/DynamicInitializer.cpp>
- VINS-Mono: <https://arxiv.org/abs/1708.03852>
- ORB-SLAM3: <https://github.com/UZ-SLAMLab/ORB_SLAM3>
- Inertial-Only Optimization: <https://arxiv.org/abs/2003.05766>
- DRT-VIO initialization: <https://openaccess.thecvf.com/content/CVPR2023/html/He_A_Rotation-Translation-Decoupled_Solution_for_Robust_and_Efficient_Visual-Inertial_Initialization_CVPR_2023_paper.html>
- DRT official code: <https://github.com/boxuLibrary/drt-vio-init>
- Online spatial-temporal VI initialization: <https://arxiv.org/abs/2004.05534>
- GVINS: <https://arxiv.org/abs/2103.07899>
- GVINS official code: <https://github.com/HKUST-Aerial-Robotics/GVINS>
- IC-GVINS: <https://arxiv.org/abs/2204.04962>
- IC-GVINS official code: <https://github.com/i2Nav-WHU/IC-GVINS>
- InGVIO: <https://arxiv.org/abs/2210.15145>
- GNSS-constrained VIO initialization: <https://isprs-annals.copernicus.org/articles/XI-1-2026/143/2026/>
- In-flight coarse alignment: <https://arxiv.org/abs/1207.1550>
- Transfer-alignment flexure study: <https://www.cambridge.org/core/journals/journal-of-navigation/article/investigation-of-flexure-effect-on-transfer-alignment-performance/D63C57E4D0A189C9C6A4151FDE9049F0>
- mix-cal: <https://github.com/jongwonjlee/mix-cal>
- MVIS: <https://arxiv.org/abs/2308.05303>
- Motion-correlation calibration: <https://researchportal.hkust.edu.hk/en/publications/real-time-temporal-and-rotational-calibration-of-heterogeneous-se/>
- Kalibr: <https://github.com/ethz-asl/kalibr>
- iKalibr: <https://github.com/Unsigned-Long/iKalibr>
