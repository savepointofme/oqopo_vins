# OpenVINS P4 在线联合初始化与 P5 自适应步长算法报告

## 1. 报告定位

本文说明当前源码中两套连续工作的算法：

- P4：在固定翼飞行过程中，用实时飞控导航状态、木板 IMU 和单目图像特征，在有限时间窗口内求出 OpenVINS 的初始状态；
- P5：OpenVINS 初始化完成后，根据飞行状态和视觉健康度，分别控制 KLT/LK 前端 tracking cadence 与 EKF 后端 update cadence。

本文只描述算法设计、数据流和当前实现，不汇总实验结果，不讨论测试覆盖或发布状态。

当前主调用链为：

```text
FC navigation stream ─┐
board IMU stream ─────┼→ P4 finite-window joint alignment
mono image + KLT ─────┘        ↓
                         AlignmentResult
                                ↓
                       OpenVINS state injection
                                ↓
raw mono image → P5 policy → tracking cadence → KLT/FeatureDatabase
                                      ↓
                                backend cadence
                                      ↓
                         propagate/clone/MSCKF/SLAM
```

实现入口：

- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`OnlineAlignmentInitializer`，第 350–400 行；
- `ov_msckf/src/core/VioManager.cpp`，`VioManager::track_image_and_update()`，第 2589–2880 行；
- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideController::update()`，第 221–533 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，ROS-free 主循环，第 2954–3758 行。

## 2. 坐标系、时间与符号

### 2.1 坐标系

| 符号 | 含义 |
| --- | --- |
| `G_nav` | 飞控声明的全局导航坐标系，也是 P4 输出状态所在的全局坐标系 |
| `F` | 飞控机体系 `FC_body` |
| `I` | 与相机固定在同一木板上的 OpenVINS IMU 坐标系 `board_imu` |
| `C` | 单目相机坐标系 |

`q_GtoF` 和 `q_GtoI` 使用 passive JPL quaternion，表示从全局坐标系到相应传感器坐标系的旋转。Camera–IMU 外参 `q_ItoC`、`p_IinC` 在本算法中作为固定量使用。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.h`，`FCNavigationSample`、`BoardImuSample`、`OnlineAlignmentOptions`，第 16–38、239–253 行。

### 2.2 三类时间关系

当前实现显式区分：

- camera timestamp；
- board IMU timestamp；
- FC navigation timestamp。

固定 Camera–IMU 时间偏移记为 `dt_CI = camera_to_imu_time_offset_s`。启动期 FC-to-board 时间偏移记为 `dt_FI = fc_to_board_time_offset_s`。在相机关键帧时间 `t_c` 查询飞控状态时，使用：

```text
t_FC = t_c + dt_CI - dt_FI
```

其中 `dt_CI` 从锁定的 OpenVINS calibration state 读取；`dt_FI` 由启动窗口内的飞控角速度与板载 IMU 角速度匹配得到，或在激励不足时采用启动先验。

实现依据：

- `ov_msckf/src/core/VioManager.cpp`，`VioManager::configure_online_alignment()`，第 2884–2934 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`OnlineAlignmentInitializer::try_initialize()`，第 978–1086、1159–1165 行。

## 3. P4 要解决的问题

普通 OpenVINS 初始化主要依赖 IMU 与视觉自身建立局部状态。P4 的目标是在飞机已经运动、不能假设静止的情况下，把 OpenVINS 直接初始化到 `G_nav`，同时给出：

```text
q_GtoI
p_IinG
v_IinG
bg
ba
15×15 initial covariance
startup FC-to-board misalignment
FC-to-board time offset
```

这里的 startup FC-to-board misalignment 只描述本次启动窗口内，飞控机体与传感器木板之间的初始方向关系，不改变 Camera–IMU 固定外参，也不被定义为永久安装标定。

结果结构定义在 `AlignmentResult` 中。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.h`，`AlignmentResult`，第 327–345 行。

## 4. P4 输入如何进入算法

### 4.1 飞控导航状态

每条 `FCNavigationSample` 包含：

- `timestamp`；
- `position_G`；
- `velocity_G`；
- `q_GtoF`；
- `navigation_frame` 与 `body_frame`；
- position、velocity、attitude 和总体 status 的有效标志。

输入首先检查有限值、quaternion 单位长度、质量标志和 frame declaration，然后按时间单调写入 `fc_buffer_`。位置和速度在目标时刻做线性插值，姿态用两侧 quaternion SLERP。

插值为：

```text
alpha = (t - t_before) / (t_after - t_before)
p_F(t) = (1-alpha) p_before + alpha p_after
v_F(t) = (1-alpha) v_before + alpha v_after
q_GF(t) = SLERP(q_before, q_after, alpha)
```

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`FCNavigationSample`，第 16–28 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`interpolate_fc()`，第 134–179 行；
- 同文件，`validate_fc()`、`feed_fc_navigation()`，第 646–721 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，FC stream 构造与送入，第 2959–2986 行。

### 4.2 木板 IMU

每条 `BoardImuSample` 包含：

- `timestamp`；
- `angular_velocity`；
- `linear_acceleration`；
- `frame`；
- status 与 gyro/accel saturation 标志。

`VioManager::feed_measurement_board_imu()` 先让 IMU 通过当前统一 IMU filter，然后把同一份处理后的角速度和加速度同时送给 OpenVINS `Propagator` 与 P4。P4 检查 frame、有限值、质量和饱和状态，并按时间单调缓存。需要精确边界样本时，角速度和加速度在相邻两帧间线性插值。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`BoardImuSample`，第 30–38 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`interpolate_imu()`，第 97–132 行；
- 同文件，`validate_imu()`、`feed_board_imu()`，第 665–747 行；
- `ov_msckf/src/core/VioManager.cpp`，`VioManager::feed_measurement_board_imu()`，第 503–535 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，board IMU 构造与送入，第 2989–3011 行。

### 4.3 单目图像

当前配置使用单目 KLT/LK。相机帧进入 `VioManager::track_image_and_update()` 后，`TrackKLT::feed_new_camera()` 运行金字塔 Lucas–Kanade 光流，并把 feature ID、raw pixel 和 normalized pixel 写入 `FeatureDatabase`。

当前 clean 配置的 `num_pts` 为 400，`max_slam` 为 50，`max_slam_in_update` 为 25，`max_msckf_in_update` 为 40。这些是正常 OpenVINS 前端/后端容量；P4 只读取同一套 KLT 轨迹，不另外增加一套 feature detector。

P4 不单独实现另一套视觉前端。它在正常 KLT 完成后，从 camera 0 的 `FeatureDatabase` 读取当前时间戳的观测，构造 `StereoAlignmentFrame`。该名称是兼容历史接口；在单目路径中，`left_timestamp` 与 `right_timestamp` 都取当前单目时间，`stereo_valid=false`，实际使用的是 camera-0 多帧轨迹。

每个观测包含：

- `feature_id`；
- `raw_left`；
- `normalized_left`；
- `track_length`；
- 单目有效标志。

实现依据：

- `baseline/clean_p4/config/estimator_config.yaml`，第 9、19–21、65–67 行；
- `ov_core/src/track/TrackKLT.cpp`，`TrackKLT::feed_monocular()`，第 129–290 行；光流调用位于第 952、1178 行；
- `ov_msckf/src/core/VioManager.cpp`，`make_online_stereo_frame()`，第 240–323 行；
- 同文件，`VioManager::track_image_and_update()`，第 2717–2767 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`StereoAlignmentObservation` 与 `StereoAlignmentFrame`，第 40–61 行。

## 5. P4 有限窗口与状态机

### 5.1 状态机

P4 状态为：

```text
WAIT_INPUTS
→ COLLECTING
→ ALIGNING
→ VALIDATING
→ NAVIGATION_READY or FULL_ALIGNMENT_READY

任一未满足条件的窗口
→ FAILED_WAIT_RETRY
→ 收到新的有效数据后回到 COLLECTING
```

含义如下：

| 状态 | 实际动作 |
| --- | --- |
| `WAIT_INPUTS` | 尚未收到有效的 FC、IMU 或视觉数据 |
| `COLLECTING` | 缓存有限窗口，等待三流共同覆盖和样本质量 |
| `ALIGNING` | 已冻结当前关键帧集合，构造并运行 Ceres 联合求解 |
| `VALIDATING` | 计算 residual、Jacobian、协方差和逐状态可观性 |
| `NAVIGATION_READY` | 导航启动所需状态满足 practical policy |
| `FULL_ALIGNMENT_READY` | 导航状态与 startup misalignment 均满足完整门限 |
| `FAILED_WAIT_RETRY` | 当前窗口未形成结果，继续等待后续有效窗口 |

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`AlignmentPhase`，第 63–71 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`transition()`、`fail_retry()`，第 598–644 行；
- 同文件，`feed_fc_navigation()`、`feed_board_imu()`、`feed_stereo()`，第 698–791 行。

### 5.2 当前窗口参数

构造函数把 `window_duration_s` 限制在 3–8 秒；当前 ROS-free online alignment 设置为 8 秒。当前主线还设置：

| 参数 | 当前值 |
| --- | ---: |
| `min_fc_samples` | 20 |
| `min_imu_samples` | 600 |
| `min_stereo_frames` | 12，当前含义是单目视觉帧数 |
| `min_feature_tracks` | 80 |
| `min_stereo_depths` | 20，当前单目路径中实际对应接受的三角化 landmark 数 |
| `min_keyframes` | 5 |
| `max_keyframes` | 10 |
| `min_visual_residual_blocks` | 60 |
| `max_fc_gap_s` | 0.45 s |
| `max_imu_gap_s` | 0.03 s |
| `max_stereo_gap_s` | 0.75 s |
| `navigation_max_collection_s` | 20 s |
| `max_time_offset_s` | 1.0 s |

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，构造函数，第 588–596 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`OnlineAlignmentOptions` defaults，第 168–179 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，`OnlineAlignmentOptions` 设置，第 2249–2296 行。

### 5.3 窗口终点与三流共同时间

`try_initialize(now)` 从不晚于 `now` 的视觉帧中倒序查找最新一帧，并要求在对应 board time 上能够同时插值 FC 与 IMU。找到后：

```text
t_init = latest supported camera timestamp
t_window_start = t_init - window_duration_s
```

进入本次计算的视觉帧必须落在 `[t_window_start, t_init]`；IMU 必须落在加上 `dt_CI` 后的窗口内；FC 缓存会额外保留 `max_time_offset_s`，用于 FC-to-board 时间偏移搜索。随后检查窗口起点覆盖、样本数量和最大采样间隔。

ROS-free runner 按 FC、IMU、camera 的时间顺序推进。每次相机处理前只送入不晚于当前 `causal_horizon = min(t_imu, t_cam)` 的 FC 行；IMU 也按时间顺序逐条送入。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`try_initialize()`，第 829–975 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，主时间循环，第 2954–3012 行。

## 6. P4 求解初值

### 6.1 FC 角速度和导航加速度

相邻 FC 姿态与速度先转换为角速度和导航系加速度：

```text
omega_F,k = -Log(R_GF,k R_GF,k-1^T) / dt
a_G,k = (v_G,k - v_G,k-1) / dt
```

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`make_fc_rates()`，第 181–207 行。

### 6.2 FC-to-board 时间与方向种子

算法在 `[-max_time_offset_s, +max_time_offset_s]` 中按 `time_offset_step_s` 搜索 `dt_FI`。对每个候选偏移，把 FC 角速度与插值后的板载 IMU 角速度配对，然后求：

```text
omega_I(t + dt_FI) ≈ R_FtoI omega_F(t) + bg
```

`R_FtoI` 通过 3×3 SVD rotation fit 得到；`bg` 取各配对残差的逐分量 median。评价量为去掉该 bias 后的 angular-rate RMS。产生最小 RMS 的候选偏移作为时间种子。

角运动的三维分布由 FC angular-rate covariance 的特征值衡量：最大轴幅值形成 `excitation`，第二轴与第一轴的比例形成 `second_axis_ratio`。它们用于判断本窗口是否能独立估计 startup misalignment。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`fit_rotation()`，第 209–264 行；
- 同文件，`OnlineAlignmentInitializer::try_initialize()`，第 978–1086 行。

### 6.3 accelerometer bias 种子

对 FC rate 时刻插值 IMU 与 FC 状态，按当前 `R_FtoI` 把导航系运动加速度和重力转换到 IMU 坐标系：

```text
ba_sample = a_measured_I - R_GtoI (a_G + gravity_G)
ba_seed = component_median(ba_sample)
```

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`OnlineAlignmentInitializer::try_initialize()`，第 1140–1153 行。

### 6.4 每个视觉关键帧的状态种子

高角速度视觉区间先被排除；剩余视觉帧过多时等间隔降采样到 `max_keyframes`。对每个保留关键帧，FC 在 `t_c + dt_CI - dt_FI` 插值，然后形成：

```text
R_GtoI = R_FtoI R_GtoF
p_IinG = p_FinG + R_FtoG p_IinF
v_IinG = v_FinG + R_FtoG (omega_F × p_IinF)
bg = rotation-fit bias seed
ba = accelerometer-bias seed
```

`p_IinF` 是声明的 FC 与 board IMU lever arm。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`OnlineAlignmentInitializer::try_initialize()`，第 1088–1193 行。

## 7. P4 联合优化

### 7.1 优化变量

对每个视觉关键帧 `k`，建立：

```text
X_k = {q_GtoI,k, p_IinG,k, v_IinG,k, bg_k, ba_k}
```

全部关键帧共享一个 startup mounting quaternion：

```text
q_FtoI
```

每个接受的视觉 feature 还有一个三维 landmark：

```text
p_f^G
```

Camera–IMU rotation、translation 和 camera intrinsics 被添加为 Ceres parameter block，但立即设为 constant，因此不参与在线标定。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`GraphState` 与 problem 构造，第 281–288、1195–1225、1291–1314 行。

### 7.2 总目标函数

当前 Ceres 问题可概括为：

```text
min  Σ ||r_prior||²
   + Σ ρ_Cauchy(||r_FC||²)
   + Σ ||r_IMU||²
   + Σ ρ_Cauchy(||r_visual||²)
```

其中 FC 与视觉因子使用 `CauchyLoss(2.0)`，IMU 预积分因子使用自身 covariance，先验因子直接使用高斯形式。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，problem residual 构造，第 1212–1289、1414–1439 行。

### 7.3 FC position/velocity/attitude 因子

每个关键帧都有一个 9 维 FC 因子。姿态部分比较当前 IMU 姿态与 `R_FtoI R_GtoF`：

```text
R_error = R_GtoI (R_FtoI R_GtoF)^T
r_att = 2 · vec(q_error) / sigma_att
```

位置和速度部分为：

```text
r_pos = (p_IinG - p_IinG_from_FC_and_lever_arm) / sigma_pos
r_vel = (v_IinG - v_IinG_from_FC_and_lever_arm) / sigma_vel
```

所以 FC 状态不是求解结束后复制到输出，而是作为观测约束窗口内的 `q/p/v` 与共享 `q_FtoI`。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`FCObservationFunctor`，第 304–345 行；
- 同文件，FC residual block 构造，第 1227–1254 行。

### 7.4 IMU preintegration 因子

相邻关键帧之间，算法取精确边界插值后的 board IMU 段，使用 `CpiV1` 预积分角速度和线加速度。`Factor_ImuCPIv1` 同时连接前后关键帧的：

```text
q, bg, v, ba, p
```

并携带 IMU noise、bias random walk、预积分 Jacobian 和 15×15 measurement covariance。因此 IMU 不只是用于启动 gate，而是真正约束状态传播、速度、位置和两类 bias。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`interval_imu_samples()` 及 IMU factor 构造，第 266–279、1256–1289 行。

### 7.5 单目视觉因子

算法按 `feature_id` 把多个关键帧中的 camera-0 观测组成 track。当前单目路径要求至少两个观测，然后利用 FC/IMU 状态种子形成相机中心和全局 ray。

对没有双目 depth 的 feature，使用多条 ray 的最小二乘交会：

```text
A = Σ (I - d_i d_i^T)
b = Σ (I - d_i d_i^T) c_i
p_f^G = A⁻¹ b
```

其中 `c_i` 为第 `i` 个相机中心，`d_i` 为全局单位视线。feature 还必须满足：

- 至少两个有效视角；
- baseline 不小于 `min_monocular_baseline_m`；
- parallax 不小于 `min_monocular_parallax_deg`；
- 三角化矩阵有足够秩；
- landmark 在全部视角中为正深度。

成功三角化后，每个 raw pixel 建立 `Factor_ImageReprojCalib`。该因子连接对应关键帧的 `q/p`、landmark，以及固定 Camera–IMU 外参和固定 camera intrinsics。由此，单目图像直接产生 residual 和 Jacobian，约束窗口内相机运动与 landmark 几何。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，视觉 track、三角化与重投影因子，第 1316–1457 行。

### 7.6 先验

当前问题为 startup mounting、首关键帧 `bg` 和首关键帧 `ba` 添加先验。若角运动不足以独立估计 mounting/time offset，practical policy 把 `q_FtoI` 固定到声明的 `R_FtoI_declared`，时间偏移使用 `weak_time_offset_prior_s`，bias 仍随 IMU 联合问题求解并以较弱状态记录。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，startup prior 选择与 residual 构造，第 1050–1086、1205–1225 行。

### 7.7 Ceres 求解器

当前求解器配置为：

```text
linear_solver_type = DENSE_SCHUR
trust_region_strategy = LEVENBERG_MARQUARDT
max_num_iterations = 30
max_solver_time = 2.0 s
num_threads = 1
```

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，solver defaults，第 231–232 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，Ceres solve，第 1459–1473 行。

## 8. P4 协方差、可观性与启动条件

### 8.1 factor contribution

求解结束后，算法分别对以下 factor family 重新计算 residual 与 Jacobian：

```text
prior
imu_preintegration
visual_reprojection
fc_pose_velocity_attitude
```

每一类都记录 residual block 数、residual 维数、RMS、P95、最大绝对值、全局 Jacobian Frobenius norm，以及对各状态块的 Jacobian norm。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`evaluate_family()`，第 398–467 行；
- 同文件，factor family 求值，第 1476–1500 行。

### 8.2 视觉增量信息

算法分别构造“prior+IMU+FC”和“prior+IMU+FC+visual”的 Schur information，消去历史关键帧和 landmark 后，两者差值表示视觉对最终状态增加的信息：

```text
ΔH_visual = H_schur(all factors) - H_schur(nonvisual factors)
```

该信息按 attitude、position、velocity、gyro bias、accelerometer bias 和 startup misalignment 分块记录。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，`schur_information()` 与 visual information increment，第 497–529、1502–1547 行。

### 8.3 covariance 与逐状态可观性

最终状态采用 18 维 error-state 排列：

```text
[δtheta, δp, δv, δbg, δba, δtheta_mount]
```

Ceres `DENSE_SVD` 恢复其 tangent-space covariance。对每个 3×3 状态块，算法计算：

- covariance standard deviation；
- information minimum/maximum eigenvalue；
- information condition number；
- prior、IMU、visual、FC 各自的 Jacobian support；
- `observable`、`prior_only` 与 `estimate_status`。

实现依据：`ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，covariance recovery 与 observability，第 1549–1721 行。

### 8.4 两种 readiness

`PRACTICAL_NAVIGATION_START` 的导航启动条件主要要求：

- solver solution usable；
- IMU 和 FC factor contribution 非零；
- visual factor 数量、Jacobian 和三角化 landmark 数满足要求；
- attitude、position 同时有 IMU、FC 和 visual information；
- velocity 同时有 IMU 与 FC information；
- gyro/accelerometer bias 有 IMU information；
- residual、bias norm 和 15×15 release covariance 满足门限。

`STRICT_FULL_ALIGNMENT` 在上述基础上，还要求全部状态，包括 startup misalignment，均通过完整逐状态可观性和 mounting angle 条件。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，navigation/full gates，第 1723–1861 行；
- 同文件，readiness 决策，第 1888–1944 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.h`，`AlignmentReadiness` 与 `AlignmentReleasePolicy`，第 75–88 行。

### 8.5 `AlignmentResult`

通过当前 policy 后，最终关键帧形成：

```text
timestamp
q_GtoI
p_IinG
v_IinG
bg
ba
15×15 covariance
R_FtoI_nominal
R_mount_residual
mount_covariance
fc_to_board_time_offset_s
camera_to_imu_time_offset_s
readiness
diagnostics
```

成功结果只释放一次；随后关闭本次初始化窗口，并停止接收后续 FC、IMU 和视觉初始化数据。

实现依据：

- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，结果构造与关闭，第 1921–1974 行；
- 同文件，关闭后的 feed/try 行为，第 698–791、829–838 行；
- `ov_msckf/src/core/VioManager.cpp`，外层输入关闭，第 529–535、2751–2768、2938–2944 行。

## 9. P4 结果如何写入 OpenVINS

### 9.1 注入时机

P4 在正常 KLT 把当前图像写入 `FeatureDatabase` 后调用 `try_initialize()`。若返回的 `AlignmentResult` 标记 `released_to_openvins=true`，且 OpenVINS 尚未初始化，则调用：

```text
VioManager::initialize_with_online_alignment(result)
```

实现依据：`ov_msckf/src/core/VioManager.cpp`，`VioManager::track_image_and_update()`，第 2717–2767 行。

### 9.2 nominal state、FEJ 与 covariance

注入函数先检查结果质量、有限值和 15×15 covariance 正定性，然后按 OpenVINS IMU nominal-state 顺序构造：

```text
[q_GtoI, p_IinG, v_IinG, bg, ba]
```

接着执行：

```text
state->_imu->set_value(imu_state)
state->_imu->set_fej(imu_state)
StateHelper::set_initial_covariance(state, result.covariance, {state->_imu})
state->_timestamp = result.timestamp
is_initialized_vio = true
```

这意味着 P4 同时写入 nominal state、First-Estimate Jacobian reference 和对应 error-state covariance。

实现依据：`ov_msckf/src/core/VioManagerHelper.cpp`，`VioManager::initialize_with_online_alignment()`，第 149–193 行。

### 9.3 初始化后的清理

状态写入后：

- 删除不晚于初始化时间的旧 FeatureDatabase measurement；
- 把 tracker feature 数切换到正常运行配置；
- 清空初始化相机队列；
- 使 propagator cache 失效；
- 根据初始速度设置 `has_moved_since_zupt`；
- 保存 `online_alignment_result_` 供 metadata 与导航 frame 使用。

实现依据：`ov_msckf/src/core/VioManagerHelper.cpp`，`VioManager::initialize_with_online_alignment()`，第 178–205 行。

### 9.4 `G_nav` 输出

P4 结果已经位于 `G_nav`，所以 ROS-free runner 对 online alignment 设置：

```text
R_Gnav_W0 = I
p_Gnav_W0 = 0
```

后续输出直接使用 OpenVINS 当前姿态、位置和速度，不再求另一组位置、航向或 SE(3) 变换。

实现依据：

- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，online navigation frame 建立，第 3584–3621 行；
- 同文件，轨迹输出，第 3989–4029 行。

## 10. P5 要解决的问题

P5 不改变相机硬件采集频率，而是在每个原始相机时刻决定两个问题：

1. 这一帧是否执行 KLT/LK tracking；
2. 如果已经执行 tracking，这一帧是否继续执行 OpenVINS 后端 update。

它输出两个 stride：

```text
tracking_stride
backend_update_stride
```

并始终保证：

```text
tracking_stride <= backend_update_stride
```

数值越小，处理越频繁；stride 1 表示每个原始相机帧都执行相应阶段。

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideDecision`，第 163–219 行；
- 同文件，`assign_targets()`，第 641–680 行。

## 11. P5 输入与因果信号

`AdaptiveStrideInput` 包含：

| 类别 | 当前输入 |
| --- | --- |
| 时间 | raw camera timestamp、actual received camera dt、actual tracking/backend interval |
| 飞行状态 | relative height、horizontal/vertical speed、roll、pitch、gyro norm |
| 视觉运动 | median/P95 parallax、parallax measurement timestamp/dt |
| 特征健康 | active MSCKF/SLAM feature count、median track age |
| 后端健康 | visual residual RMSE/P95、MSCKF input/accepted/rejected、距上次接受更新的时间 |
| filter 健康 | covariance finite/negative diagonal、position/velocity/attitude jump |
| 生命周期 | `initialized` |

这些量来自当前或历史传感器/estimator 状态。P5 在当前 raw frame 被送入 KLT 之前运行，所以本次策略使用的是截至该时刻已经存在的最近 tracker/backend 结果。

当前 runner 中，relative height 优先取不晚于当前相机时间、且年龄不超过 2 秒的 GPS ENU-U；没有可用 GPS 高度时，回退到 OpenVINS 相对高度。GPS XY/course 不用于 P5 状态选择。

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideInput`，第 130–161 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，P5 input 构造，第 3151–3309 行。

## 12. P5 信号处理与几何量

### 12.1 一阶低通

height、vertical speed、gyro、parallax rate 和 backend acceptance ratio 使用按真实时间间隔计算的一阶低通：

```text
alpha = 1 - exp(-dt / tau)
x_filtered ← x_filtered + alpha (x - x_filtered)
```

默认时间常数包括：

| 信号 | `tau` |
| --- | ---: |
| height | 1.0 s |
| vertical speed | 0.5 s |
| gyro norm | 0.20 s |
| parallax rate | 0.35 s |
| backend acceptance ratio | 2.0 s |

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideConfig`，第 101、113–116 行；
- 同文件，`AdaptiveStrideController::update()` 与 `update_lowpass()`，第 227–267、543–552 行。

### 12.2 每个 raw frame 的 normalized parallax

tracker 给出的 parallax 来自两个实际 tracking frame。控制器先除以该 measurement dt 得到 pixel rate，再乘当前实际 raw camera dt：

```text
parallax_rate = measured_parallax / measurement_dt
normalized_parallax_per_raw_frame = parallax_rate × actual_received_camera_dt
```

因此阈值描述的是“每个原始相机间隔的视差”，不把上一次推荐 stride 当作实际时间。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideController::update()` 与 `fill_output()`，第 240–267、700–715 行。

### 12.3 predicted overlap

控制器用高度与水平速度估计 high-altitude backend stride 对应的平移 overlap：

```text
footprint = 2 h tan(horizontal_FOV / 2)
gap_high = actual_raw_dt × high_backend_stride
predicted_overlap = clamp(1 - horizontal_speed × gap_high / footprint, 0, 1)
```

该量与低空高度共同决定是否进入 `LOW_ALTITUDE_SAFETY`。

当 raw height 与 filtered height 都有效时，安全判断使用两者的较小值：

```text
effective_height = min(raw_height, filtered_height)
```

这样下降过程中不会因为低通滞后而高估当前高度。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideController::update()` 与 `filtered_height()`，第 286–303、393–413、607–611 行。

## 13. P5 状态机

### 13.1 六个状态

当前状态为：

```text
HIGH_ALTITUDE_CRUISE
NORMAL_CRUISE
DESCENT_SAFETY
LOW_ALTITUDE_SAFETY
TURN_SAFETY
VISUAL_DEGRADED
```

同一时刻多个条件成立时，优先级为：

```text
VISUAL_DEGRADED
> LOW_ALTITUDE_SAFETY
> TURN_SAFETY
> DESCENT_SAFETY
> NORMAL_CRUISE
> HIGH_ALTITUDE_CRUISE
```

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptivePolicyState`，第 17–42 行；
- 同文件，raw state 选择与 `priority()`，第 415–440、559–575 行。

### 13.2 `VISUAL_DEGRADED`

以下任一类信号可触发 visual degraded candidate：

- parallax 数据过期；
- active feature 数过低；
- normalized P95 parallax 过大；
- visual residual P95 过大；
- MSCKF acceptance ratio 过低；
- 后端长时间没有接受更新；
- covariance 非有限或存在负对角；
- state jump 过大；
- feature 少且 median track age 很短。

其中 stale flow、covariance bad 和 state jump 被视为立即 visual emergency；其他视觉退化默认需要持续 1 秒才进入。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，视觉状态判断与 entry confirm，第 330–391、577–590 行。

### 13.3 `LOW_ALTITUDE_SAFETY`

满足任一条件时形成 low-altitude candidate：

```text
effective height < 50 m
or predicted_overlap < 0.80
```

状态内部再分三个高度带：

| 高度带 | tracking/backend stride |
| --- | --- |
| 30–50 m | 2 / 4 |
| 20–30 m | 1 / 2 |
| <20 m | 1 / 1 |

向更安全、更高频的高度带切换可以立即发生；高度回升时分别使用 22 m、33 m 的退出阈值，并要求 2 秒恢复时间。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，高度阈值、第 65–72 行；low state 判断、第 393–402 行；`update_low_band()`，第 613–639 行；`assign_targets()`，第 655–665 行。

### 13.4 `TURN_SAFETY`

转弯进入条件为以下任一项：

```text
filtered gyro norm >= 0.20 rad/s
|roll| >= 10 deg
|pitch| >= 25 deg
```

严重转弯阈值为 0.35 rad/s、18° roll 或 32° pitch。严重转弯立即进入；普通转弯需持续 0.5 秒。退出要求 gyro、roll 和 pitch 同时回到 0.12 rad/s、6°、20°以内，并持续恢复 5 秒。

该状态目标为 tracking stride 1、backend stride 2。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，转弯阈值、第 76–84 行；转弯判断、第 305–322 行；entry/recovery、第 577–604 行；target、第 667–673 行。

### 13.5 `DESCENT_SAFETY`

filtered vertical speed 不高于 `-1.5 m/s` 时形成 descent candidate，持续 1.5 秒后进入。退出要求 vertical speed 回到 `-0.2 m/s` 以上并持续 5 秒。

目标为 tracking stride 2、backend stride 4。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，下降阈值、第 73–74 行；下降判断、第 323–328、432–436 行；entry/recovery、第 587–603 行；target、第 651–654 行。

### 13.6 `HIGH_ALTITUDE_CRUISE` 与 `NORMAL_CRUISE`

稳定高空需要：

- height 达到进入阈值 120 m；
- predicted overlap 达到 0.90；
- 不处于转弯、下降或视觉退化；
- 条件持续 6 秒。

进入后使用 100 m 与 0.86 overlap 作为退出侧阈值。高空目标为 tracking/backend 4/12，普通巡航为 2/6。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，高空阈值、第 65–66、107–110、125 行；high-cruise 判断、第 403–413 行；target、第 641–650 行。

### 13.7 candidate、确认时间和恢复时间

控制器不会看到一个瞬时条件就反复改 stride。它先保存：

```text
candidate_state
candidate_trigger
candidate_since
```

候选状态和触发原因保持不变达到 `entry_confirm()` 或 `recovery_confirm()` 后，才更新正式 `state_`。进入更高优先级安全状态使用 entry time；退出安全状态使用更长 recovery time，并要求当前状态自身的退出条件成立。

实现依据：`ov_msckf/src/ros_free/AdaptiveStrideController.h`，candidate 与状态切换，第 442–520 行；`entry_confirm()`、`recovery_confirm()`，第 577–605 行。

### 13.8 当前默认 cadence 总表

ROS-free runner 以默认构造方式创建 `AdaptiveStrideController`，因此当前使用 `AdaptiveStrideConfig` 中的默认目标：

| 状态 | `tracking_stride` | `backend_update_stride` |
| --- | ---: | ---: |
| `HIGH_ALTITUDE_CRUISE` | 4 | 12 |
| `NORMAL_CRUISE` | 2 | 6 |
| `DESCENT_SAFETY` | 2 | 4 |
| `LOW_ALTITUDE_SAFETY`, 30–50 m | 2 | 4 |
| `LOW_ALTITUDE_SAFETY`, 20–30 m | 1 | 2 |
| `LOW_ALTITUDE_SAFETY`, <20 m | 1 | 1 |
| `TURN_SAFETY` | 1 | 2 |
| `VISUAL_DEGRADED` | 1 | 1 |

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，`AdaptiveStrideConfig`，第 44–63 行；
- 同文件，`assign_targets()`，第 641–680 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，默认 controller 实例，第 2515 行。

## 14. tracking cadence 与 backend cadence 如何分开

### 14.1 runner 调度

每个原始相机时刻，runner 先更新 P5 input 并调用：

```text
adaptive_decision = adaptive_stride_controller.update(adaptive_input)
```

然后维护两个独立 raw-frame counter：

```text
adaptive_raw_frames_since_tracking
adaptive_raw_frames_since_backend
```

调度条件为：

```text
tracking_due = raw_frames_since_tracking >= tracking_stride
backend_due  = raw_frames_since_backend  >= backend_update_stride

do_cam_feed = tracking_due || backend_due
backend_frame_eligible = backend_due
```

因为状态机保证 `tracking_stride <= backend_update_stride`，backend due 的帧一定也会进入 KLT。tracking 执行后重置 tracking counter；backend 执行后单独重置 backend counter。

实现依据：

- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，双 counter 与 due 判定，第 3344–3371 行；
- 同文件，counter reset，第 3638–3652 行。

### 14.2 两个 camera API

普通完整帧调用：

```text
feed_measurement_camera(message)
→ track_image_and_update(message, true)
```

P5 active 帧调用：

```text
feed_measurement_camera_cadence(message, backend_eligible)
→ track_image_and_update(message, backend_eligible)
```

实现依据：

- `ov_msckf/src/core/VioManager.h`，两个 camera feed API，第 155–170 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，API 选择，第 3579–3583 行。

### 14.3 tracking-only 帧

当 `do_cam_feed=true` 且 `backend_eligible=false`：

1. 当前图像照常执行 KLT；
2. tracker 内部的上一帧图像、keypoint 和 feature ID 得到更新；
3. 当前时间戳观测进入 `FeatureDatabase`；
4. `cleanup_measurements_exact(timestamp)` 删除这些只供 tracking 连续性使用的当前观测；
5. 函数在 ZUPT、propagation、clone、MSCKF 和 SLAM 前返回。

因此 tracking-only 帧提高前端光流的时间连续性，但不增加 EKF clone 和后端视觉更新频率。

实现依据：

- `ov_msckf/src/core/VioManager.cpp`，KLT 与 tracking-only 分支，第 2717–2726、2770–2804 行；
- `ov_core/src/feat/FeatureDatabase.cpp`，`cleanup_measurements_exact()`，第 245–263 行。

### 14.4 backend 帧

当 `backend_eligible=true`，当前帧完成 KLT 后进入 `do_feature_propagate_update()`：

```text
propagate state to image time
→ augment IMU clone
→ wait until clone count is sufficient
→ classify lost/marginalized/SLAM/MSCKF tracks
→ cap MSCKF features
→ MSCKF update
→ regular SLAM update
→ delayed SLAM initialization
→ retriangulate active tracks
→ cleanup FeatureDatabase
→ change SLAM anchors
→ marginalize oldest clone
```

实现依据：

- `ov_msckf/src/core/VioManager.cpp`，`do_feature_propagate_update()`，第 2971–3016 行；
- 同文件，MSCKF update，第 3338–3477 行；
- 同文件，SLAM update/delayed init，第 3479–3603 行；
- 同文件，cleanup 与 clone marginalization，第 3605–3663 行。

## 15. P4 与 P5 的时间协同

### 15.1 初始化之前

在 `sys->initialized()==false` 时：

- P5 controller 可以被调用，但输出 `main_trigger=not_initialized`；
- active 双 cadence 尚未应用；
- 相机仍按 `args.cam_subsample` 的固定输入 stride 进入 KLT 与 P4，当前主线使用 fixed stride12；
- 每个被接收的相机帧先运行 KLT，再尝试形成 P4 三流窗口；
- online alignment 模式不会回退到普通 `InertialInitializer`。

实现依据：

- `ov_msckf/src/ros_free/AdaptiveStrideController.h`，未初始化分支，第 269–284 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，P4 阶段固定 cadence，第 3344–3361 行；
- `ov_msckf/src/core/VioManager.cpp`，禁止 ordinary initializer fallback，第 2825–2841 行。

### 15.2 成功初始化的相机帧

在某个已接收帧上：

```text
KLT writes current observations
→ P4 freezes [t_window_start, t_init]
→ Ceres solve and validation
→ AlignmentResult
→ initialize_with_online_alignment()
→ OpenVINS becomes initialized
```

P4 成功后关闭初始化窗口。该次 `track_image_and_update()` 随后按完整 backend-eligible 路径继续；下一原始相机帧进入 runner 时，P5 scheduler 识别到 initialization 刚完成，并强制执行一次 tracking+backend，使双 cadence 从一个明确边界开始。

实现依据：

- `ov_msckf/src/core/VioManager.cpp`，P4 释放与后续流程，第 2746–2880 行；
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，首次 active cadence 帧，第 3352–3371 行。

### 15.3 初始化之后的每个原始相机帧

初始化完成后，每个 raw camera timestamp 按下列顺序处理：

```text
1. Assemble P5 inputs from past/current sensor and estimator state
2. AdaptiveStrideController::update()
3. Increment tracking/backend raw-frame counters
4. Evaluate tracking_due and backend_due
5a. Neither due:
      do not call KLT or backend
5b. Tracking due only:
      KLT → update tracker continuity → remove exact-time backend observations → return
5c. Backend due:
      KLT → propagate/clone → MSCKF → SLAM → cleanup/output
6. Reset the counter corresponding to the work actually performed
```

实现依据：

- `ov_msckf/src/run_serial_msckf_ros_free.cpp`，P5 input、decision 与调度，第 3151–3371、3579–3652 行；
- `ov_msckf/src/core/VioManager.cpp`，逐帧前端/后端分流，第 2717–2880 行。

## 16. 三种典型逐帧路径

| 当前阶段 | 原始图像是否进入 KLT | 是否进入 P4 | 是否 clone/MSCKF/SLAM |
| --- | --- | --- | --- |
| P4 收集期，固定 stride 未到 | 否 | 否 | 否 |
| P4 收集期，固定 stride 到达 | 是 | 是；收集或尝试联合初始化 | 初始化成功前不进入普通后端 |
| P5 active，tracking/backend 都未到 | 否 | P4 已关闭 | 否 |
| P5 active，仅 tracking 到达 | 是 | P4 已关闭 | 否 |
| P5 active，backend 到达 | 是 | P4 已关闭 | 是 |

这就是 P4 和 P5 的核心协作关系：P4 先在固定、有限、可解释的传感器窗口中决定 OpenVINS 的起点；P5 只在该起点建立后接管图像计算频率，而且把“保持光流连续”与“执行 EKF 后端更新”分成两个 cadence。

## 17. 当前实现边界

当前算法包含：

- 单目 KLT/LK 多帧视觉；
- board IMU preintegration；
- FC position/velocity/attitude observation；
- 启动期 FC-to-board time/misalignment 处理；
- q/p/v/bg/ba 与 covariance 注入；
- P5 高空、普通巡航、下降、低空、转弯和视觉退化状态；
- tracking/backend 双 cadence。

当前算法不包含：

- 在线修改 Camera–IMU `T_C_I`；
- 在线 camera intrinsics/distortion calibration；
- 历史约 7° 或 4.089° 手工补偿；
- 转弯期间的时变 Velcro/flex 模型；
- 把 startup misalignment 直接声明为永久 FC-to-board calibration；
- P5 对相机硬件采集频率的控制；
- learned feature frontend；当前 clean 路径使用 KLT/LK；
- 初始化之后继续运行 P4 联合求解。

实现依据：

- `baseline/clean_p4/config/estimator_config.yaml`，camera/feature/calibration 配置，第 9–14、65–67、92 行；
- `ov_msckf/src/core/VioManager.cpp`，KLT/descriptor 选择，第 417–432 行；
- `ov_msckf/src/core/OnlineAlignmentInitializer.cpp`，固定 camera calibration 与一次关闭，第 1291–1314、1962–1967 行；
- `ov_msckf/src/core/VioManager.cpp`，关闭后停止输入，第 529–535、2751–2768、2938–2944 行。
