# 概念 ↔ 文档 ↔ 代码 速查索引

> 用法: 看官方 docs 或论文时遇到一个概念/公式/符号, 回来这张表找到:
> 1. 哪一篇中文文档有详细解释
> 2. 源码里实际实现在哪

---

## 1. 数学 & Lie 代数

| 概念 | 中文文档 | 代码实现 |
|---|---|---|
| JPL 四元数 (scalar-last) | [math_foundations.md §2](./math_foundations.md#2-jpl-四元数-scalar-last) | `ov_type::JPLQuat`, `ov_core::quat_multiply` |
| 误差状态 EKF 3 条公式 | [math_foundations.md §3](./math_foundations.md#3-误差状态-error-state-ekf) | `ov_type::IMU::update` (L78-96) |
| SO(3) exp/log & skew | [math_foundations.md §4](./math_foundations.md#4-so3se3-lie-小抄) | `ov_core::exp_so3`, `log_so3`, `skew_x` |
| Trawny Eq.101 四元数积分 | [propagation_math.md §2.1](./propagation_math.md#21-旋转积分--trawny-eq101) | `Propagator::predict_mean_discrete` (L525-532) |
| FEJ (首次估计 Jacobian) | [math_foundations.md §6](./math_foundations.md#6-fej-first-estimates-jacobian-一句话版) | 所有 `_fej` 后缀变量; `UpdaterHelper.cpp:89-96` |

## 2. 滤波器状态

| 概念 | 中文文档 | 代码 |
|---|---|---|
| 15 维 IMU 误差状态顺序 | [math_foundations.md §5](./math_foundations.md#5-openvins-滤波器的-15-维-imu-误差状态), [state_and_cov.md §2](./state_and_cov.md#2-type-体系是一切的地基) | `IMU::set_local_id` (IMU.h:64-70) |
| `_Cov` 索引与维度演进 | [state_and_cov.md §3](./state_and_cov.md#3-_cov-的行列顺序-时间演进) | `StateHelper::augment_clone / marginalize / EKFPropagation` |
| `_clones_IMU` 用 `std::map` | [state_and_cov.md §6](./state_and_cov.md#6-_clones_imu-用-stdmap-的理由) | `State.h:150`, `State::margtimestep` |

## 3. IMU 传播

| 概念 | 中文文档 | 代码 |
|---|---|---|
| IMU 测量模型 `a_m, ω_m` | [propagation_math.md §1](./propagation_math.md#1-imu-测量模型) | `Propagator::predict_and_compute` (L442-463) |
| 速度/位置积分 `v̇ = Rᵀa - g` | [propagation_math.md §2.2](./propagation_math.md#22-速度位置积分) | `Propagator.cpp:534-538` |
| 连续时间 `F, G` | [propagation_math.md §3.1](./propagation_math.md#31-连续时间-f-g) | `Propagator::compute_F_and_G_discrete/_analytic` |
| `Φ, Q_d` 离散化 | [propagation_math.md §3.2](./propagation_math.md#32-离散化--φ-q_d) | `Propagator.cpp:482-505` |
| Phi_summed / Qd_summed 累乘 | [propagation_math.md §4](./propagation_math.md#4-多步累乘-为什么要-phi_summed) | `Propagator.cpp:85-108` |
| 随机克隆 | [propagation_math.md §5](./propagation_math.md#5-随机克隆-stochastic-cloning) | `StateHelper::augment_clone` |

## 4. 观测与更新

| 概念 | 中文文档 | 代码 |
|---|---|---|
| 投影链 `p_FinG → p_FinCi → uv` | [measurement_math.md §1](./measurement_math.md#1-像素观测模型-step-3-前的铺垫) | `UpdaterHelper.cpp:329-348` |
| H_f (特征雅可比) | [measurement_math.md §2.1](./measurement_math.md#21-对特征位置-p_fing-或其他表示参数) | `UpdaterHelper.cpp:385-389` |
| H_x (状态雅可比, clone 部分) | [measurement_math.md §2.2](./measurement_math.md#22-对-clone-位姿-δθ-δp-6-维) | `UpdaterHelper.cpp:376-392` |
| 外参 Jacobian | [online_calibration.md §2.1](./online_calibration.md#21-在-jacobian-中的位置) | `UpdaterHelper.cpp:404-413` |
| 内参 / 畸变 Jacobian | [online_calibration.md §3.2](./online_calibration.md#32-jacobian--dz_dzeta) | `UpdaterHelper.cpp:366-367,416-418` |
| 左零空间投影 (消 H_f) | [measurement_math.md §3](./measurement_math.md#3-左零空间投影-msckf-的灵魂) | `UpdaterHelper::nullspace_project_inplace` (L426-454) |
| QR 测量压缩 | [measurement_math.md §4](./measurement_math.md#4-qr-测量压缩-update-compressdox) | `UpdaterHelper::measurement_compress_inplace` (L456-486) |
| 卡方外点剔除 | [measurement_math.md §3.3](./measurement_math.md#33-卡方-chi-square-残差门控) | `UpdaterMSCKF.cpp:228-253` |
| EKF Joseph 更新 | [measurement_math.md §5](./measurement_math.md#5-ekf-更新-updatedox) | `StateHelper::EKFUpdate` |

## 5. 前端与特征

| 概念 | 中文文档 | 代码 |
|---|---|---|
| KLT 流程 (检测+金字塔+光流) | [tracking.md](./tracking.md) | `TrackKLT::feed_monocular/feed_stereo` |
| FeatureDatabase 滚动窗 | [tracking.md](./tracking.md) | `ov_core::FeatureDatabase` |
| 3D 线性三角化 | [feature_triangulation.md §1](./feature_triangulation.md#1-3d-线性三角化-single_triangulation) | `FeatureInitializer::single_triangulation` (L30-112) |
| 1D 深度三角化 | [feature_triangulation.md §1.4](./feature_triangulation.md#14-1d-三角化变体) | `FeatureInitializer::single_triangulation_1d` (L114-195) |
| 高斯牛顿精化 | [feature_triangulation.md §2](./feature_triangulation.md#2-非线性精化--高斯牛顿-single_gaussnewton) | `FeatureInitializer::single_gaussnewton` (L197+) |

## 6. 初始化 & 其他

| 概念 | 中文文档 | 代码 |
|---|---|---|
| 静态 vs 动态初始化选择 | [initialization.md](./initialization.md) | `InertialInitializer::initialize` |
| 静态初始化 (重力对齐) | [initialization.md](./initialization.md) | `ov_init::StaticInitializer` |
| 动态初始化 (MLE/Ceres) | [initialization.md](./initialization.md) | `ov_init::DynamicInitializer` |
| 零速检测 (ZUPT) | [updater.md](./updater.md) | `UpdaterZeroVelocity` |
| ROS-free 离线回放 | [ros_free.md](./ros_free.md) | `run_serial_msckf_ros_free.cpp` + `ros_free/*` |

---

## 7. "一个问题定位三件事"示例

**问题**: "滤波器发散, 高速运动下 yaw 漂走了, 怎么办?"

1. **看中文文档: 哪些环节会影响 yaw 一致性?**
   - [math_foundations.md §6 FEJ](./math_foundations.md#6-fej-first-estimates-jacobian-一句话版) → **yaw 方向的观测性**被 EKF 错误线性化影响
   - [measurement_math.md §3.3 卡方](./measurement_math.md#33-卡方-chi-square-残差门控) → 剔除太严可能丢掉矫正 yaw 的关键观测
   - [online_calibration.md §4 时间偏移](./online_calibration.md#4-时间偏移-t_off--t_imu---t_cam) → 快速运动放大时间不对齐的误差

2. **看代码: 对应开关和变量**
   - `StateOptions::do_fej = true` (默认已开)
   - `UpdaterOptions::chi2_multipler` (默认 1, 考虑调到 3~5)
   - `StateOptions::do_calib_camera_timeoffset = true` + Kalibr 离线估初值

3. **验证**:
   - `PRINT_DEBUG` 打印每特征 `chi2`, 看是不是剔太狠
   - 观察 `_calib_dt_CAMtoIMU` 的协方差收敛情况
   - 用 `ov_eval/src/plot_trajectories.py` 看 yaw error 时序曲线

这个流程对任何"滤波器表现异常"的调试都适用 — **文档是地图, 代码是现场**, 两者配合才能高效诊断。
