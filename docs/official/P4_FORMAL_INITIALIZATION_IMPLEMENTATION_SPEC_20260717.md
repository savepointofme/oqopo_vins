# P4 正式有限时间滑窗联合初始化算法与实现冻结规格

日期：2026-07-17
性质：实现前冻结规格，不是已完成实现，也不是实验结果
当前源码快照：`fdd8d745229c7115b56b5d2d765f462a3fd68b4b`
配套调研：[P4_INITIALIZATION_LITERATURE_AND_ARCHITECTURE_REVIEW_20260717.md](P4_INITIALIZATION_LITERATURE_AND_ARCHITECTURE_REVIEW_20260717.md)

## 1. 本报告解决什么问题

本文把下一版正式 P4 的算法、坐标、时间、状态、因子、窗口生命周期、代码文件、函数输入输出、代码复用来源和验收条件一次冻结。后续实现不得靠“看起来差不多”改变这里的语义；如果实现中发现本规格的数学前提不成立，必须先更新本报告并说明证据，再改代码。

当前工作树的生产入口仍在执行：

    upstream OpenVINS dynamic initializer
      -> local metric VIO
      -> FC yaw + translation gauge
      -> AGL scale postprocess

证据是 `run_serial_msckf_ros_free.cpp` 的在线配置仍把 `upstream_dynamic_init_fc_gauge` 设为 `true`。这条路径已经被前一份架构审查否决：FC 没有进入 q/p/v/bg/ba 的初始联合估计，错误的局部速度、尺度和 bias 也不能由 yaw+translation 修好。

本规格要求替换为：

    FC navigation q/p/v ─┐
    board raw IMU ───────┼─> causal fixed-duration window
    monocular KLT tracks ┘             │
                                      ├─ rotation/time/mount/bg stage
                                      ├─ metric p/v/ba stage
                                      ├─ bounded joint refinement
                                      ├─ covariance/information validation
                                      └─ atomic OpenVINS q/p/v/bg/ba injection

P4 的物理分类是“FC 主系统到 board IMU 从系统的飞行中传递对准，加 FC 约束的单目视觉—惯性初始化”，不是普通纯 VIO 初始化，也不是事后轨迹对齐。

## 2. 完成定义

只有同时满足以下条件，才能称为“正式 P4 已实现”：

1. FC q/p/v 从初始状态生成开始就进入度量联合估计，不再先跑错误局部 VIO 后做 gauge。
2. 窗口由传感器时间长度定义；FC、IMU、视觉量测数由窗口和实际频率自然产生，不以固定 10 次、固定 N 次更新或 3/5/8/12 选择表决定释放。
3. 每次窗口推进都用当前窗内完整 FC/IMU/视觉证据重新估计；旧候选递归滤波器不再代替滑窗联合估计。
4. Camera–IMU 外参、FC–board nominal mounting、FC attitude latency、FC navigation latency 和 Velcro transient flex 是不同物理量。
5. q/p/v/bg/ba 的置信状态分别计算；未被数据支持的量明确标为 prior-retained，不伪装成已标定。
6. 向 OpenVINS 的状态注入是一次原子操作，状态顺序和协方差顺序严格为 `[δθ, δp, δv, δbg, δba]`。
7. 不注入初始化图的 clone 或 landmark；后端从终端 IMU 状态重新建立正常历史。
8. 单元、合成、集成、短程、全程和跨飞行验证全部通过；同口径正式结果不得劣于冻结 global baseline。
9. 单窗求解 p95 墙钟时间小于窗口推进间隔，内存只随固定窗口上限增长，不出现 O(history²) 回放。

本文完成不等于上述代码和实验已经完成。本文交付的是唯一实现合同。

## 3. 明确禁止的实现

以下做法不允许作为本规格的“近似实现”：

- 禁止把 upstream dynamic initializer 的结果当正式 P4，再只估 yaw、translation 或 Sim(3)。
- 禁止把 AGL/GPS 高度后处理写进 P4 状态方程来补救初始化尺度。
- 禁止使用 GPS course、最终轨迹误差、未来数据或评价真值决定在线 release。
- 禁止将每一行 FC q/p/v 当成相互独立、同方差的绝对观测；这会随关键帧密度重复乘信息。
- 禁止每关键帧独立建立 bg/ba；正式短窗采用 shared bg/ba。
- 禁止每个推进窗口重新生成和优化全部显式 landmark。
- 禁止 candidate 创建后冻结图，只用 FC p/v 递归滤波验证。
- 禁止高角速度一律删除视觉帧。转弯既可能有模糊，也提供安装角和 bias 激励，必须按真实视觉健康和三源残差分类。
- 禁止把转弯时的 FC–board flex residual 写回永久 mounting calibration。
- 禁止用固定左转补偿、固定右转补偿或固定 yaw 角抵消 fly1/fly3。
- 禁止 P5 的 adaptive scheduler 在 P4 初始化期间改变 P4 的观测合同。P4 前端输入 cadence 是固定配置，P5 只在 OpenVINS 初始化完成后启用。
- 禁止保留“provenance 是否齐全”作为求解或 release gate。来源说明只由 runner 写入 metadata，不进入热路径。
- 禁止以第一版实验退化为由直接否定算法；先通过合成和残差方向测试证明实现正确，再评价算法。

## 4. 坐标、旋转和时间的唯一约定

### 4.1 坐标系

| 符号 | 含义 |
| --- | --- |
| `G` | FC 声明的全局导航系 `G_nav` |
| `F` | 飞控机体系 `FC_body` |
| `I` | board IMU 系，也是 OpenVINS IMU 状态系 |
| `C` | camera 0 坐标系 |

所有 quaternion 都是 OpenVINS passive JPL `[qx,qy,qz,qw]`。`R_AtoB` 把 A 系表达的向量变为 B 系表达。Ceres quaternion manifold 直接复用：

- `ov_init/src/ceres/State_JPLQuatLocal.h`
- `ov_init/src/ceres/State_JPLQuatLocal.cpp::Plus()`

其更新是左扰动：

    q <- Exp_JPL(δθ) ⊗ q

任何 Eigen Hamilton quaternion 只允许存在于有显式转换的局部适配函数中，不能跨模块裸传。

### 4.2 固定 Camera–IMU 外参

锁定 Kalibr/运行时 calibration state：

    R_ItoC
    p_IinC
    dt_CI

它们不在 P4 中优化。相机位姿由 IMU 位姿计算：

    R_GtoC,k = R_ItoC R_GtoI,k

    p_CinG,k =
      p_IinG,k
      - R_GtoI,k^T R_ItoC^T p_IinC

这个公式必须有 identity、纯旋转、非零平移三个单元测试。

### 4.3 FC–board nominal mounting 与在线 residual

外部接受标定提供：

    R_FtoI_nominal
    P_mount_nominal

在线只估本次启动窗口的小残差：

    R_FtoI =
      Exp(δθ_mount) R_FtoI_nominal

`δθ_mount` 是 I 系左扰动。由 FC 生成 board 姿态：

    R_GtoI,k^FC =
      R_FtoI R_GtoF,k

这个乘法次序必须用三组非交换 roll/pitch/yaw 合成测试锁定，不能只用 yaw 测试。

### 4.4 杆臂

声明量：

    p_IinF
    P_lever_arm

FC q/p/v 转换为 board IMU q/p/v：

    R_FtoG,k = R_GtoF,k^T

    p_IinG,k^FC =
      p_FinG,k + R_FtoG,k p_IinF

    v_IinG,k^FC =
      v_FinG,k
      + R_FtoG,k (ω_F,k × p_IinF)

用于 accelerometer seed 时必须包含完整刚体杆臂加速度：

    a_IinG,k^FC =
      a_FinG,k
      + R_FtoG,k [
          α_F,k × p_IinF
          + ω_F,k × (ω_F,k × p_IinF)
        ]

不能继续使用只含平移加速度、忽略转弯杆臂项的 seed。

### 4.5 重力与 IMU 符号

沿用当前 OpenVINS `Propagator::predict_mean_discrete()`：

    v_j =
      v_i + R_GtoI,i^T (a_m - ba) Δt - gravity_G Δt

    p_j =
      p_i + v_i Δt
      + 0.5 R_GtoI,i^T (a_m - ba) Δt²
      - 0.5 gravity_G Δt²

当前 runner 的 `gravity_G=[0,0,+g]`。所以 accelerometer bias seed 必须使用：

    ba_sample =
      a_m^I - R_GtoI (a_IinG^FC + gravity_G)

报告、代码和测试都不得改成相反符号。

### 4.6 三类时间

定义：

    t_I = t_C + dt_CI

    t_F_att = t_I - dt_FI_att

    t_F_nav = t_I - dt_FI_nav

其中：

- `dt_CI`：锁定 Camera–IMU calibration；
- `dt_FI_nav`：位置/速度时间映射，使用外部声明，不由角速度相关推断；
- `dt_FI_att = dt_FI_att_nominal + δt_att`；
- `δt_att`：只在当前窗口多轴激励、相关峰曲率和边界条件同时成立时估计，否则固定为零并标记 `FIXED_EXTERNAL_CALIBRATION`。

`δt_att` 不直接作为 Ceres 连续参数，因为 FC interpolation 在样本边界的导数不稳定。它使用外层一维搜索，进入最终联合精化时保持固定，并把剩余时间不确定度传播进终端 covariance。

## 5. 输入合同

### 5.1 FC navigation

继续沿用现有 `FCNavigationSample`：

    timestamp
    position_G
    velocity_G
    q_GtoF
    navigation_frame
    body_frame
    position_valid
    velocity_valid
    attitude_valid
    status_valid

新增噪声合同不放进每一行，而放进 `P4Options`：

    fc_terminal_covariance       9x9, [θ,p,v]
    fc_increment_noise_psd       9x9, [θ,p,v] per second
    fc_attitude_latency_sigma_s
    lever_arm_covariance         3x3

如果输入文件没有 FC covariance，runner 可由明确配置的 attitude/position/velocity sigma 构造对角矩阵；不得在 initializer 内偷偷使用硬编码。

### 5.2 board IMU

继续沿用 `BoardImuSample`：

    timestamp
    angular_velocity
    linear_acceleration
    frame
    status_valid
    gyro_saturated
    accel_saturated

P4 接收的必须是与 OpenVINS Propagator 相同的已统一轴系、已应用当前 IMU filter 的数据。P4 不允许再做第二套隐藏滤波。

IMU 噪声直接使用当前 estimator 参数：

    sigma_w
    sigma_wb
    sigma_a
    sigma_ab

### 5.3 monocular KLT

继续使用 `VioManager.cpp::make_online_stereo_frame()` 从同一个 `FeatureDatabase` 构造 `StereoAlignmentFrame`，不新增 detector，不改 feature ID。

单目实际使用：

    frame.left_timestamp
    observation.feature_id
    observation.raw_left
    observation.normalized_left
    observation.track_length
    observation.left_valid

`right_*`、`depth_m` 和 `stereo_valid` 在正式单目 P4 中不参与因子。`normalized_left` 必须是按锁定 intrinsics/distortion 得到的 bearing；新代码只把它扩成 `[x,y,1]` 并归一化。

### 5.4 在线和评价数据边界

在线 P4 只可读取上述 FC、board IMU、KLT 和固定 calibration。以下字段只用于离线验收：

    GPS reference
    GPS course
    truth
    final trajectory error
    start-heading aligned error
    best-fit aligned error

任何在线 diagnostics 出现 `gps_used=true` 都应视为合同失败，而不是普通 warning。

## 6. 输出合同

正式 `AlignmentResult` 只保留：

    timestamp
    q_GtoI
    p_IinG
    v_IinG
    bg
    ba
    covariance_15x15
    R_FtoI_nominal
    R_mount_residual
    mount_covariance_3x3
    fc_attitude_to_board_time_offset_s
    fc_attitude_time_offset_variance
    fc_navigation_to_board_time_offset_s
    camera_to_imu_time_offset_s
    state_status[q,p,v,bg,ba,mount,time]
    readiness
    released_to_openvins
    diagnostics

必须删除或永久置空：

    initial_clones
    initial_landmarks
    startup_consumed_feature_ids
    initial_joint_covariance

终端 covariance 顺序唯一为：

    [δθ, δp, δv, δbg, δba]

`NAVIGATION_READY` 的语义：

- q、p、v、bg 由当前窗口数据支持并通过稳定性检查；
- ba 可以是 `ESTIMATED_CURRENT_DATA`，也可以是 `FIXED_TO_PRIOR`；
- 如果 ba 是 prior-retained，输出 covariance 必须保留其真实先验不确定度，不能给零方差或伪造“已标定”。

`FULL_ALIGNMENT_READY` 额外要求：

- ba 有数据增量信息；
- mount residual 有数据增量信息；
- time residual 若被开放估计，也有内部曲率和跨窗稳定证据。

OpenVINS 实际注入始终是一次原子 q/p/v/bg/ba 注入。所谓“分级”是各状态组在滑窗中的内部 readiness 分级，不是把半个 IMU state 提前写进 EKF。

## 7. 固定量与优化变量

### 7.1 固定量

    R_ItoC, p_IinC, dt_CI
    gravity_G
    p_IinF
    dt_FI_nav
    camera intrinsics/distortion
    IMU noise model

### 7.2 每个关键帧变量

对窗口中的每个 keyframe k：

    q_GtoI,k   4 storage / 3 tangent
    p_IinG,k   3
    v_IinG,k   3

### 7.3 窗口共享变量

    bg               3
    ba               3
    δθ_mount         3

`δt_att` 由外层一维搜索得到，在联合图中固定。

总 tangent 维数：

    9K + 9

这里允许每关键帧 q/p/v，是因为 IMU 和视觉约束连接的是时变 pose/velocity；不允许每关键帧 bg/ba，是因为数秒初始化窗内 bias 首先采用 shared 模型。它与旧图“每帧 q/p/v/bg/ba + landmark”不是同一拓扑。

如果 shared bias 在合成测试和真实数据中被证明不足，下一版只能加入低阶 bias random-walk 节点；在本版验收前不得预先实现。

## 8. 时间滑窗与生命周期

### 8.1 唯一窗口

正式配置只有一个物理窗口长度：

    T_window

当前计划默认 8 s，但它是配置量，不是 3/5/8/12 选择器。窗口终点为最新三流因果支持的 camera timestamp：

    t_end_C

窗口：

    [t_end_C - T_window, t_end_C]

窗口推进条件只按传感器时间：

    t_end_C - last_solved_end_C >= T_advance

当前计划默认 `T_advance=0.5 s`。FC/IMU/视觉样本数只作为 diagnostics；是否可解由时间覆盖、矩阵秩、information eigenvalue、残差和 covariance 决定。

### 8.2 keyframe 选择

窗口内全部健康 KLT frame 都是候选。选择规则：

1. 必保留窗口首尾；
2. 保留能提高 time/parallax/rotation excitation 覆盖的帧；
3. 保留长 track 生存和图像空间分布良好的帧；
4. 不因高角速度本身拒绝；
5. 只有实际模糊、track survival 下降、bearing residual 失真时降低视觉权重；
6. 超过资源上限时做 information-gain 贪心选择，而不是均匀抽成固定 10 帧。

`max_keyframes` 只是内存/实时性上限，默认沿用 36；它不是 release 所需量测次数。

### 8.3 持续重估

每次窗口推进：

    build current window
      -> reuse unchanged raw segments/preintegrations
      -> warm-start overlap states
      -> solve all stages on current window
      -> compute current group readiness
      -> compare overlapping states with previous solved windows
      -> not ready: slide again
      -> ready: atomic release and close P4

不存在 `candidate_active_ -> validate_candidate()` 短路。旧候选滤波器不再是生产路径。

### 8.4 readiness 的时间语义

每个状态组维护：

    ready_since_sensor_time
    latest_ready_window_end
    latest_cross_window_normalized_delta

一个组只有在连续一个 `T_window` 的滑窗推进时间里始终满足内部 gate，才从 `ESTIMATED_UNSTABLE` 变为 `TRUSTED`。这意味着确认长度自动等于实际求解窗口长度，不使用固定 N 次支持或固定 2 次反馈。

窗口重叠比较使用：

    d² = δx^T (P_current + P_previous)^-1 δx

同时保留物理 fail-closed 上限，但 release 依据首先是归一化统计量。对 bias 还检查在一个完整 `T_window` 上的线性趋势是否显著非零。

### 8.5 P4 与 P5

初始化前：

- P5 scheduler 不生效；
- camera 以固定配置的 P4 tracking cadence 进入 KLT；
- P4 自己按时间和信息选择 keyframe。

初始化成功后：

- P4 关闭并停止接收；
- OpenVINS 从 release timestamp 开始正常传播/clone/update；
- P5 从明确的下一原始 camera frame 开始管理 tracking/backend cadence。

## 9. 单窗完整算法

### 9.1 阶段 A：窗口构造与质量检查

输入：

    FC buffer
    IMU buffer
    visual frame buffer
    t_end_C
    P4Options

输出：

    P4Window
    P4WindowDiagnostics
    failure reason

工作：

1. 计算每个 camera 时刻对应的 board/FC attitude/FC navigation 时刻；
2. 在边界上插值 FC 和 IMU；
3. 检查三流单调、有效、有限、frame declaration 和 saturation；
4. 计算实际最大采样 gap；
5. 建立 feature track 索引；
6. 选择 keyframe；
7. 为每个相邻 keyframe 建立可复用 IMU raw segment；
8. 建立相邻和必要长基线 frame pair 的 common bearing。

失败只表示当前窗口不可解；数据继续前移，不产生永久失败，除非 frame/axis/calibration 配置非法。

### 9.2 阶段 B：FC rate、time residual、mount 和 bg seed

先从平滑的 FC rotation trajectory 计算：

    ω_F,k =
      -Log(R_GtoF,k R_GtoF,k-1^T) / Δt

    a_FinG,k =
      (v_FinG,k - v_FinG,k-1) / Δt

    α_F,k =
      (ω_F,k - ω_F,k-1) / Δt

差分前使用按 FC timestamp 的局部 SO(3) smoothing，不对 Euler angle 分量滤波。

对每个 attitude time offset candidate，配对：

    y_k = ω_m^I(t_k + dt_candidate)
    x_k = ω_F(t_k)

求：

    y_k = R_FtoI x_k + bg + n_k

`robust_wahba_with_bias()` 使用 IRLS：

1. 以当前权重计算 x/y 加权均值；
2. 对去均值向量做 3x3 SVD rotation fit；
3. 以 `median(y-Rx)` 更新 bg；
4. 以 residual MAD 更新 Huber 权重；
5. 直到 rotation 和 bias 增量收敛或达到小型固定迭代上限。

time score 是全部 whitened rate residual、FC/IMU relative rotation residual 和 rotation-only visual residual 的和。搜索：

1. 以 `min(median_dt_fc, median_dt_imu)/2` 为 coarse step；
2. 范围由外部 latency sigma 和硬边界共同限定；
3. 取最小 coarse cell；
4. 用相邻三点抛物线做一次 sub-sample refinement；
5. 只有最小值不在边界、曲率为正、二轴 excitation 通过时接受 residual offset；
6. 否则返回 nominal offset 和 `FIXED_EXTERNAL_CALIBRATION`。

输出：

    dt_attitude
    dt_attitude_variance
    R_FtoI_seed
    bg_seed
    excitation eigenvalues
    robust residual statistics

### 9.3 阶段 C：DRT rotation-only visual/bg refinement

直接参考 DRT-VIO 的 rotation-only gyro-bias 子问题。对每个相邻 keyframe pair：

    bearings_i
    bearings_j
    locked R_ItoC
    IMU ΔR(bg_lin), J_q_bg

构造消去 translation 的最小特征值 residual，联合优化：

    bg
    δθ_mount

其中视觉 residual 主要约束 bg/relative rotation；FC relative rotation 把 mount 引入同一旋转子问题。

FC 预测的 I 相对旋转：

    R_Ii_to_Ij^FC =
      R_FtoI
      R_GtoF,j R_GtoF,i^T
      R_FtoI^T

OpenVINS CPI 约定的 residual 必须保持：

    r_R =
      2 vec(
        q_pred_i_to_j
        ⊗ q_imu_i_to_j^-1
        ⊗ q_bg_correction
      )

不能因换成 Eigen quaternion 改变乘法次序。

DRT residual 只用于 rotation seed/refinement，不直接用于最终 covariance，因为其最小特征值不是标准高斯量测。最终联合图使用可按像素噪声 whitening 的 epipolar/Sampson residual。

### 9.4 阶段 D：FC metric q/p/v seed 与 ba seed

对每个 keyframe：

    R_GtoI,k^0 =
      R_FtoI_seed R_GtoF(t_F_att)

    p_IinG,k^0 =
      p_FinG(t_F_nav)
      + R_FtoG(t_F_nav) p_IinF

    v_IinG,k^0 =
      v_FinG(t_F_nav)
      + R_FtoG(t_F_att)
        (ω_F × p_IinF)

用含角加速度和向心项的 `a_IinG^FC` 形成每时刻 ba sample，取 robust location 作为 `ba_seed`。

这一步已经给出 `G_nav` 中的度量 q/p/v。视觉不是用来事后求一个 scale；视觉和 IMU是在后续图中检查并修正这些 seed。

### 9.5 阶段 E：translation/velocity/ba reduced solve

固定阶段 C 的 q、bg、mount 和 time，优化：

    p_k, v_k for every keyframe
    shared ba

加入：

- shared-bias 9D CPI factor；
- FC terminal absolute p/v factor；
- FC density-invariant p/v increment factors；
- monocular bearing epipolar factors；
- ba physical prior。

该阶段的作用是先在较好条件数下解决 metric translation、velocity 和 accelerometer bias，避免一开始把所有旋转和平移耦合交给大图。

### 9.6 阶段 F：bounded joint refinement

变量：

    q_k, p_k, v_k
    shared bg, ba
    shared δθ_mount

固定：

    dt_attitude
    Camera–IMU calibration
    lever arm
    gravity_G

因子：

    shared-bias CPI
    FC one absolute attitude anchor
    FC terminal absolute p/v
    FC relative q/p/v increments
    bearing epipolar/Sampson visual residual
    mount/bg/ba priors

求解器：

    trust_region_strategy = LEVENBERG_MARQUARDT
    linear_solver = SPARSE_NORMAL_CHOLESKY
    fallback = DENSE_QR only if sparse backend unavailable
    max_iterations = 15
    num_threads = configured build/runtime threads

`max_iterations` 是计算上限，不是收敛证据；release 仍由 residual、information、covariance 和跨窗稳定决定。

在准备 release 的窗口上，必须以已求得 bg/ba 重新对全部相邻区间做一次真实 CPI preintegration，然后再做最后一次短 refinement。这样 release 不依赖过远 bias linearization 的一阶修正。普通未 ready 的推进窗口可复用 Jacobian correction。

### 9.7 阶段 G：terminal covariance 与 calibration uncertainty

从最终 tangent Hessian 恢复并边缘化得到：

    P_terminal =
      Cov([δθ_K, δp_K, δv_K, δbg, δba])

`δθ_mount` 是联合 nuisance variable，因此其相关性自然进入 terminal marginal。

time offset 在外层搜索中固定，必须额外传播：

    P_terminal <-
      P_terminal
      + J_x_dt σ_dt² J_x_dt^T

`J_x_dt` 通过在 `dt ± ε` 上重建 time-dependent FC constraints 并各做一次短 solve 的中心差分获得。lever arm covariance 同样通过 analytic/numeric Jacobian加到 terminal p/v 子块。

最后执行：

1. 对称化；
2. 检查全部有限；
3. 检查最小 eigenvalue；
4. 只允许数值级负 eigenvalue 做最近 PSD 修正，并记录修正量；
5. 大于数值容差的非 PSD 直接拒绝。

### 9.8 阶段 H：逐状态信息与 release

对 factor family 分别计算 residual/Jacobian：

    prior
    imu
    fc_terminal
    fc_increment
    visual_epipolar

为 q、p、v、bg、ba、mount 构造：

    posterior std
    data-only minimum information eigenvalue
    prior information contribution
    IMU/FC/visual Jacobian support
    cross-window normalized delta
    full-window trend
    source status

`prior_only` 的定义不是“估计值接近 prior”，而是移除 prior 后该状态组的数据 Schur information 不足。

release 必须同时满足：

- 当前 solve usable；
- terminal covariance finite/PSD；
- q/p/v/bg ready；
- visual 和 IMU 均对 q/p/v 的至少一个相关子空间有非零增量信息；
- FC 对 global gauge 和 metric p/v 有增量信息；
- 没有 saturation、time boundary、flex dominance 或 covariance correction safety failure；
- 各必要状态组连续一个 `T_window` 处于 ready；
- release timestamp 不晚于当前已喂入的 IMU/camera 因果 horizon。

## 10. 因子数学定义

### 10.1 shared-bias CPI factor

新因子 residual 维数 9，参数块：

    q_i, p_i, v_i,
    q_j, p_j, v_j,
    bg, ba

残差按当前 `Factor_ImuCPIv1` 的顺序抽取：

    r_R =
      2 vec(
        q_i_to_j
        ⊗ q_breve^-1
        ⊗ q_bg_correction
      )

    r_v =
      R_GtoI,i (
        v_j - v_i + gravity_G Δt
      )
      - J_b δbg
      - H_b δba
      - beta

    r_p =
      R_GtoI,i (
        p_j - p_i - v_i Δt
        + 0.5 gravity_G Δt²
      )
      - J_a δbg
      - H_a δba
      - alpha

whitening covariance 从原 CPI 15x15 `P_meas` 中按 residual index `[0..2, 6..8, 12..14]` 抽取完整 9x9 子矩阵，保留交叉项。不能简单拿三个 3x3 对角块。

### 10.2 FC terminal factor

选择一个 tri-source consistency 最好的 keyframe作为 attitude anchor，不要求是窗口最后一帧。其绝对姿态 residual：

    r_q_abs =
      Log(
        R_GtoI,anchor
        (R_FtoI R_GtoF,anchor)^T
      )

窗口最后一帧提供 absolute p/v：

    r_p_abs =
      p_K - p_IinG,K^FC

    r_v_abs =
      v_K - v_IinG,K^FC

attitude anchor 使用 `fc_terminal_covariance` 的 attitude 3x3 marginal；terminal p/v 使用同一配置的 p/v 6x6 marginal。因为两者可能不在同一 timestamp，不能伪造它们之间的跨时刻相关项。这样即使 terminal 正处在 Velcro flex 瞬态，global attitude gauge 仍可由窗口内健康 anchor 建立，再由 IMU/视觉传播到 terminal。

### 10.3 FC increment factor

相邻 keyframe：

    r_q_inc =
      Log(
        R_Ii_to_Ij^state
        (R_Ii_to_Ij^FC)^T
      )

    r_p_inc =
      (p_j - p_i)
      - (p_j^FC - p_i^FC)

    r_v_inc =
      (v_j - v_i)
      - (v_j^FC - v_i^FC)

其 covariance 由连续时间 PSD 离散化：

    Q_inc(Δt) =
      FcIncrementNoiseModel::discretize(
        fc_increment_noise_psd, Δt
      )

在 Brownian increment 模型下，改变 keyframe 密度不会凭空增加同一物理时间段的信息。必须用同一合成轨迹的稀/密 keyframe 对照验证 estimate 和 covariance 基本不变。

### 10.4 monocular epipolar factor

对 camera i/j common track：

    R_Ci_to_Cj =
      R_GtoC,j R_GtoC,i^T

    t_Ci_in_Cj =
      R_GtoC,j (p_CinG,i - p_CinG,j)

    e =
      f_j^T [t_Ci_in_Cj]_x
      R_Ci_to_Cj f_i

使用 normalized Sampson denominator，把 residual 转为近似 pixel sigma 可解释量。baseline 太小时该 pair 不建 translation factor，但仍可参与 DRT rotation-only factor。

visual factor 直接连接 q_i,p_i,q_j,p_j 和固定 Camera–IMU calibration，不建立 landmark parameter block。

### 10.5 priors

只允许以下有物理来源的 prior：

    δθ_mount ~ N(0, P_mount_nominal)
    bg ~ N(bg_seed, P_bg_prior)
    ba ~ N(ba_seed, P_ba_prior)

不需要人为固定 first pose/yaw/position，因为 FC absolute factor已经提供 `G_nav` gauge。若移除 FC factor 后图出现 gauge，属于预期；不能再偷偷加一个高信息 first-pose prior让测试通过。

## 11. Velcro/flex 与转弯处理

每个 frame pair 同时计算：

    e_FI = FC relative rotation vs board IMU
    e_VI = visual rotation vs board IMU
    e_FV = FC relative rotation vs visual rotation

分类：

| 条件 | 判断 | 行为 |
| --- | --- | --- |
| `e_FI` 大，`e_VI` 小 | FC–board flex、FC latency 或 FC attitude异常更可疑 | 降低该区间 FC attitude/mount 权重；保留 IMU+visual |
| `e_VI` 大，`e_FI` 小 | 模糊、rolling shutter、track 洗牌更可疑 | 降低视觉权重；保留 FC+IMU |
| 三者都大 | time mapping、IMU saturation 或共同异常 | 当前 interval 不进入 mount 更新，窗口可继续靠其他 interval |
| 三者都小且有多轴激励 | rigid-supported | 可用于 mount/time/bg |

这只是内部残差因果分类，不把 FC 或视觉当真值。

转弯方向相反的 fly1/fly3 必须作为验收对。正式 online 代码不包含“左转固定取右侧、右转固定取左侧”的航向补偿；视觉空间均衡和动态 ROI 属于前端/P5 的独立策略。P4 只允许通过实际 track 质量和三源一致性改变权重。

## 12. 状态机

正式状态机：

    WAIT_INPUTS
      -> COLLECTING_WINDOW
      -> SOLVING_ROTATION
      -> SOLVING_TRANSLATION
      -> JOINT_REFINING
      -> VALIDATING_WINDOW
          -> COLLECTING_WINDOW       not ready, slide
          -> NAVIGATION_READY        atomic release
          -> FULL_ALIGNMENT_READY    atomic release
      -> CLOSED_AFTER_RELEASE

配置非法进入：

    FATAL_CONFIGURATION_ERROR

不再存在生产语义：

    CANDIDATE_VALIDATING
    CANDIDATE_REFINING
    fixed candidate replay
    upstream dynamic init FC gauge
    shadow-only release

## 13. 逐文件实现合同

新实现不能继续堆进当前 226 KB 的 `OnlineAlignmentInitializer.cpp`。正式代码拆成一个薄 supervisor、六个算法模块和五个 P4 专用 factor；旧 candidate/gauge 代码不作为新实现容器。

### 13.1 `ov_msckf/src/core/p4/P4Types.h`

从头实现，只定义数据，不放求解逻辑。

主要类型：

    struct P4Options;
    struct P4Keyframe;
    struct P4ImuInterval;
    struct P4VisualPair;
    struct P4Window;
    struct P4RotationSolution;
    struct P4MetricSolution;
    struct P4JointSolution;
    struct P4GroupEvidence;
    struct P4ReleaseDecision;

`P4Keyframe`：

    double t_camera;
    double t_board;
    double t_fc_attitude;
    double t_fc_navigation;
    FCNavigationSample fc_attitude;
    FCNavigationSample fc_navigation;
    StereoAlignmentFrame visual;
    Eigen::Vector4d q_seed;
    Eigen::Vector3d p_seed;
    Eigen::Vector3d v_seed;

`P4JointSolution`：

    vector<P4State> states;       // q/p/v per keyframe
    Vector3d bg;
    Vector3d ba;
    Vector3d mount_residual;
    double attitude_time_offset;
    MatrixXd tangent_covariance;
    FactorDiagnostics diagnostics;

所有 struct 的 frame、unit、quaternion convention 写在字段注释中。禁止用无 frame 后缀的 `position`、`rotation` 等歧义名称。

### 13.2 `P4WindowBuffer.h/.cpp`

从当前 `OnlineAlignmentInitializer.cpp` 移动并精简以下函数：

| 新函数 | 当前来源 | 处理 |
| --- | --- | --- |
| `interpolateImu()` | `interpolate_imu()`，当前约 107–142 行 | 保留有限值、状态和 saturation 检查；补充 exact bracket diagnostics |
| `interpolateFc()` | `interpolate_fc()`，当前约 144–189 行 | 保留 p/v 线性和 quaternion SLERP；拆分 attitude/nav valid |
| `computeFcRates()` | `make_fc_rates()`，当前约 197–217 行 | 保留 passive rotation 符号；增加 SO(3) smoothing 和 angular acceleration |
| `extractImuInterval()` | `interval_imu_samples()`，当前约 267–279 行 | 保留精确边界插值；输出原始 sample index span |

公开接口：

    class P4WindowBuffer {
    public:
      bool feedFc(
          const FCNavigationSample& sample,
          P4RejectReason* reason);

      bool feedImu(
          const BoardImuSample& sample,
          P4RejectReason* reason);

      bool feedVisual(
          const StereoAlignmentFrame& frame,
          P4RejectReason* reason);

      P4BuildWindowResult buildWindow(
          double causal_camera_horizon,
          const P4Options& options) const;

      void pruneBefore(double camera_timestamp);
      void reset();
    };

`buildWindow()` 输入是因果 camera horizon 和物理配置；输出包含 `status/window/diagnostics`，不修改 buffer。只有 supervisor 接受本次 solve 后才 prune，避免构窗失败破坏数据。

### 13.3 `P4PreintegrationCache.h/.cpp`

直接使用 `ov_core::CpiV1`，不复制另一套 preintegration。

接口：

    class P4PreintegrationCache {
    public:
      const P4Preintegration& getOrBuild(
          const P4ImuInterval& interval,
          const Vector3d& bg_linearization,
          const Vector3d& ba_linearization,
          const P4ImuNoise& noise);

      void invalidateIntervalsBefore(double board_time);
      void clear();
    };

`P4Preintegration` 保存：

    DT
    q_breve
    alpha
    beta
    J_q, J_a, J_b, H_a, H_b
    P_meas_15x15
    bg_linearization
    ba_linearization
    raw_interval_identity

cache key 至少含：

    [start board timestamp,
     end board timestamp,
     first/last raw sample serial,
     IMU noise model identity]

普通推进窗口允许用 CPI bias Jacobian warm start。准备 release 时调用：

    rebuildAtSolvedBiases(
        current_window, solved_bg, solved_ba)

强制真实重积分并重新 refine。

### 13.4 `P4MotionCalibrator.h/.cpp`

从头实现 FC–board rotation/time seed；不复用旧 candidate filter。

接口：

    class P4MotionCalibrator {
    public:
      P4RotationSolution solve(
          const P4Window& window,
          const P4PreintegrationCache& preintegrations,
          const P4Calibration& calibration,
          const P4Options& options) const;

    private:
      vector<P4RatePair> buildRatePairs(
          const P4Window& window,
          double attitude_time_offset) const;

      P4WahbaSolution robustWahbaWithBias(
          const vector<P4RatePair>& pairs,
          const Matrix3d& rotation_prior) const;

      P4TimeSearchResult searchAttitudeTimeOffset(
          const P4Window& window,
          const P4Calibration& calibration) const;

      P4IntervalHealth classifyIntervalHealth(
          const P4RotationEvidence& evidence) const;
    };

输入是当前完整窗口和外部 calibration；输出是本窗口 estimate，不修改永久 calibration。

### 13.5 `ov_msckf/src/core/p4/thirdparty/drt/`

只 vendor DRT rotation-only 子问题的必要代码，保持原许可证和 attribution：

    DrtCayley.h
    DrtSmallestEigenvalue.h
    README.md

精确来源：

- `D:\vscode_dir\p4_init_refs_20260717\drt-vio-init\include\geometry.hpp::Quaternion2Cayley()`；
- `D:\vscode_dir\p4_init_refs_20260717\drt-vio-init\include\initMethod\opengvMethod.hpp::GetSmallestEVwithJacobian()`；
- 原仓库 commit `fb0ac8d3fc4d9f683888565882838b6f1c330435`；
- GPLv3。

vendor helper 保持数值实现不变；只做 namespace、include 和格式适配。OpenVINS 类型转换放在外层 wrapper，不改 helper。

### 13.6 `P4VisualConstraintBuilder.h/.cpp`

接口：

    class P4VisualConstraintBuilder {
    public:
      vector<P4VisualPair> buildPairs(
          const P4Window& window,
          const P4CameraCalibration& camera,
          const P4Options& options) const;

      P4RotationOnlyConstraint buildDrtRotationConstraint(
          const P4VisualPair& pair,
          const P4Preintegration& preintegration,
          const Matrix3d& R_ItoC) const;

      P4TranslationDirection estimateTranslationDirection(
          const P4VisualPair& pair,
          const Matrix3d& relative_rotation,
          const Vector3d& fc_translation_seed) const;

      P4VisualHealth evaluateVisualHealth(
          const P4VisualPair& pair) const;
    };

`buildPairs()`：

- 按 feature ID 收集 common normalized bearing；
- 检查 finite、image-space distribution、survival 和 rank；
- 相邻 frame pair 用于 CPI/rotation；
- 额外长 baseline pair只在提升 translation information 时加入；
- 不三角化 landmark；
- 不按固定 feature 数截断成同一批点，超过计算上限时按 image grid 和 track age 均衡选取。

`estimateTranslationDirection()` 只生成 seed/diagnostic。最终图仍使用原始 correspondence 的 epipolar factor，不能把一个估计方向重复当成多次独立观测。

### 13.7 `P4MetricInitializer.h/.cpp`

从头实现阶段 D/E。

接口：

    class P4MetricInitializer {
    public:
      P4MetricSeed buildFcMetricSeed(
          const P4Window& window,
          const P4RotationSolution& rotation,
          const P4Calibration& calibration) const;

      P4MetricSolution solve(
          const P4Window& window,
          const P4MetricSeed& seed,
          const vector<P4Preintegration>& preintegrations,
          const vector<P4VisualPair>& visual_pairs,
          const P4Options& options) const;
    };

`buildFcMetricSeed()` 是全部杆臂、重力和时间公式的唯一实现位置。其他模块不得各写一套 FC->board 转换。

### 13.8 `P4JointRefiner.h/.cpp`

从头实现正式 Ceres graph。

接口：

    class P4JointRefiner {
    public:
      P4JointSolution refine(
          const P4Window& window,
          const P4RotationSolution& rotation,
          const P4MetricSolution& metric,
          const vector<P4Preintegration>& preintegrations,
          const vector<P4VisualPair>& visual_pairs,
          const P4Calibration& calibration,
          const P4Options& options) const;

      P4JointSolution reintegrateAndRefineForRelease(
          const P4Window& window,
          const P4JointSolution& preliminary,
          P4PreintegrationCache& cache,
          const P4Calibration& calibration,
          const P4Options& options) const;
    };

该文件只负责 parameter blocks、factor blocks、loss、solver 和结果读取；窗口构造、状态 gate、metadata 写出都不放在这里。

### 13.9 `P4ReleaseValidator.h/.cpp`

把当前 `evaluate_family()`、`evaluate_family_residuals()` 和 `schur_information()` 的通用思想移入，删除与 candidate/gauge/landmark history 绑定的分支。

接口：

    class P4ReleaseValidator {
    public:
      P4WindowEvidence evaluateWindow(
          const P4Window& window,
          const P4JointSolution& solution,
          const P4FactorRegistry& factors,
          const P4Options& options) const;

      P4ReleaseDecision updateAcrossWindows(
          const P4WindowEvidence& current,
          P4ReadinessHistory& history,
          const P4Options& options) const;

      Matrix<double,15,15> recoverTerminalCovariance(
          const P4JointSolution& solution,
          const P4FactorRegistry& factors,
          const P4CalibrationUncertainty& uncertainty) const;
    };

`updateAcrossWindows()` 的 history 以 timestamp 存储并按 `T_window` prune，绝不按 deque size 或 update count release。

### 13.10 `P4Initializer.h/.cpp`

这是新算法总控：

    class P4Initializer {
    public:
      bool feedFc(const FCNavigationSample&);
      bool feedImu(const BoardImuSample&);
      bool feedVisual(const StereoAlignmentFrame&);

      P4TryResult tryInitialize(
          double causal_camera_horizon,
          AlignmentResult& release);

      void reset();
    };

`tryInitialize()` 唯一允许的调用顺序：

    buildWindow()
    motion_calibrator.solve()
    visual_builder.buildPairs()
    metric_initializer.buildFcMetricSeed()
    metric_initializer.solve()
    joint_refiner.refine()
    release_validator.evaluateWindow()
    release_validator.updateAcrossWindows()
    if ready:
      joint_refiner.reintegrateAndRefineForRelease()
      release_validator.recoverTerminalCovariance()
      fill AlignmentResult
      close

任何阶段失败都返回结构化 reason，并保留后续窗口重试能力。

### 13.11 `OnlineAlignmentInitializer.h/.cpp`

保留类名以减少 `VioManager` API 扰动，但改为薄 facade：

    OnlineAlignmentInitializer::feed_fc_navigation()
      -> P4Initializer::feedFc()

    OnlineAlignmentInitializer::feed_board_imu()
      -> P4Initializer::feedImu()

    OnlineAlignmentInitializer::feed_stereo()
      -> P4Initializer::feedVisual()

    OnlineAlignmentInitializer::try_initialize()
      -> P4Initializer::tryInitialize()

删除生产成员：

    OnlineAlignmentCandidateFilter
    candidate_active_
    candidate_visual_snapshots_
    make_fc_gauge_target()
    commit_shadow_gauge_release()
    previous_window_states_ old mixed semantics
    upstream_dynamic_init_fc_gauge flags
    shadow/direct/deferred combinatorial flags

旧实现如需保留用于对照，移到明确的 `legacy/` 或只存在于 git 历史；不能与正式路径靠布尔组合共存。

## 14. P4 专用 factor 文件

全部放在：

    ov_msckf/src/core/p4/factors/

### 14.1 `Factor_P4ImuSharedBias.h/.cpp`

来源：复制并适配当前：

- `ov_init/src/ceres/Factor_ImuCPIv1.h`；
- `ov_init/src/ceres/Factor_ImuCPIv1.cpp::Evaluate()`。

保留：

- passive JPL relative quaternion；
- `J_q/J_a/J_b/H_a/H_b` bias correction；
- gravity、position、velocity符号；
- analytic Jacobian结构。

修改：

- residual 从 15 维变 9 维；
- 参数从前后两个 bg/ba 变为一个 shared bg 和一个 shared ba；
- 删除 bias random-walk residual；
- whitening 使用 9x9 完整 covariance 子矩阵；
- 参数顺序改成报告 10.1 节，并为每个 block 写 static assertion/test。

### 14.2 `Factor_P4FcTerminal.h/.cpp`

从头实现。输入：

    FC synchronized PVA
    lever arm
    q/p/v state blocks
    mount residual block
    terminal covariance

输出 3D attitude 或 6D p/v whitened residual。attitude anchor 和 terminal p/v 分成两个 residual block，避免它们必须来自同一个 timestamp。

### 14.3 `Factor_P4FcIncrement.h/.cpp`

从当前 `OnlineAlignmentInitializer.cpp` 的 FC terminal/increment functor 只复用已经通过坐标测试的杆臂和 relative rotation表达；噪声离散化和 factor API 从头实现。

输入：

    synchronized FC endpoint PVA
    state_i/state_j
    shared mount residual
    Q_inc(dt)

输出 9D whitened q/p/v increment residual。

### 14.4 `Factor_P4DrtRotationOnly.h/.cpp`

外层 wrapper 直接适配：

- DRT `optimization.hpp::BiasSolverCostFunctor`，约 26–114 行；
- DRT `drtVioInit.cpp::gyroBiasEstimator()` 的逐相邻帧建因子方式，约 346–399 行。

不复制：

- DRT 自己的 tracker；
- DRT `IMUPreintegrated`；
- DRT camodocal/Sophus 容器；
- DRT 硬编码 `CauchyLoss(1e-5)`；
- DRT 固定 observation count warning；
- DRT 200 次迭代配置。

新 wrapper 使用 OpenVINS `CpiV1` 的 `q_breve/J_q` 和锁定 `R_ItoC`。robust scale 由 normalized bearing noise/MAD 产生。

### 14.5 `Factor_P4Epipolar.h/.cpp`

从头实现 normalized Sampson residual。参数：

    q_i, p_i, q_j, p_j

固定数据：

    f_i, f_j
    R_ItoC, p_IinC
    visual_pixel_sigma

实现 analytic 或 AutoDiff 均可，但必须：

- 与 OpenVINS passive JPL manifold一致；
- 对 q_i/q_j/p_i/p_j 做 finite-difference Jacobian test；
- 对纯旋转/零 baseline 返回“不建因子”，不能除零；
- 对相机平移外参非零的情况通过测试。

## 15. 代码复用与许可证清单

| 来源 | commit | 许可证 | 使用方式 | 不使用部分 |
| --- | --- | --- | --- | --- |
| 当前 OpenVINS 分支 | `fdd8d745...` | GPLv3/MIT mixed by file | 直接复用 CpiV1、JPL manifold、state injection；移动 interpolation/helpers | 旧 candidate/gauge production path |
| DRT-VIO | `fb0ac8d...` | GPLv3 | vendor Cayley/smallest-EV helper；适配 rotation-only bias factor | tracker、preintegrator、gravity/scale完整 pipeline、硬编码门限 |
| ORB-SLAM3 | `4452a3c...` | GPLv3 | 只采用“fixed/seed pose + per-KF velocity + shared bg/ba”的架构依据 | 不复制 g2o 实现 |
| IC-GVINS | `644eed9...` | GPLv3 | 只采用“外部导航先建立 metric/global INS，再加视觉”的架构依据 | 不复制零速、双天线 yaw、轮式假设 |
| mix-cal | `eda6c67...` | BSD-3-Clause | 只参考刚体 angular-rate/specific-force关系和退化分析 | 不把 FC navigation 当第二套同步 raw IMU |

DRT vendor 目录必须保留：

    original repository URL
    original commit
    original file/function names
    GPLv3 notice
    exact semantic changes

在适配完成前先建立 golden equivalence test：用 clone 中原函数对固定 bearings、`q_breve` 和 `J_q` 生成并提交 deterministic golden fixture；仓内 vendor helper 的 residual/Jacobian 必须与 fixture 一致。正常构建不得依赖 `D:\vscode_dir\p4_init_refs_20260717` 外部目录。通过后才允许接 P4。

## 16. VioManager、runner 与 CMake 改动

### 16.1 `VioManager.cpp`

保留：

- `feed_measurement_board_imu()`：同一处理后 IMU 同时进入 Propagator 和 P4；
- `make_online_stereo_frame()`：从正常 KLT database 取观测；
- `configure_online_alignment()`：锁定 Camera–IMU calibration；
- `track_image_and_update()` 中 KLT 后调用 P4。

删除/旁路：

- upstream dynamic initializer + gauge 的生产分支；
- `make_fc_gauge_target()` 和 `OnlineVioFcGaugeAligner` 正式调用；
- provisional VIO先初始化再 anchor 的路径；
- AGL scale 与 P4 release 的耦合。

正式顺序：

    KLT writes current observations
      -> P4 feed visual
      -> P4 tryInitialize(current timestamp)
      -> if released:
           initialize_with_online_alignment(result)
      -> continue normal backend from this boundary

### 16.2 `VioManagerHelper.cpp::initialize_with_online_alignment()`

直接保留当前 237–248 行的核心：

    imu_state << q, p, v, bg, ba
    state->_imu->set_value(imu_state)
    state->_imu->set_fej(imu_state)
    StateHelper::set_initial_covariance(...)
    state->_timestamp = result.timestamp

保留初始化后的 database cleanup、tracker feature count、queue clear、propagator cache invalidation 和 initialized flag。

删除正式路径中的：

- local origin/gauge 特殊重写；
- release covariance CLI diagonal override；
- post-P4 camera extrinsic rotation ablation；
- clone/landmark joint-history injection；
- upstream dynamic initializer inflation特判。

如果需要 covariance inflation，只能由 P4 输出的 prior-retained 状态和 calibration uncertainty在 P4 内完成，handoff 不再二次改变数学含义。

### 16.3 `run_serial_msckf_ros_free.cpp`

删除正式配置：

    upstream_dynamic_init_fc_gauge = true
    candidate_window_durations_s
    candidate_filter gates
    shadow/direct/deferred release switches
    diagnostic mixed-source release switches

新增显式配置：

    p4_window_duration_s
    p4_window_advance_s
    p4_tracking_stride
    fc_terminal_covariance
    fc_increment_noise_psd
    mount_prior_covariance
    lever_arm_covariance
    attitude_time_offset_prior/sigma
    solver runtime budget

metadata schema 升级，必须写出当前算法 ID，例如：

    openvins_p4_fc_metric_drt_shared_bias_v1

不能继续沿用旧 v6/v8/v9 candidate metadata 并把字段置零冒充新算法。

### 16.4 CMake

当前 `ov_msckf/cmake/ROS1.cmake` 和 `ROS2.cmake` 显式列 `LIBRARY_SOURCES`。两处同时加入全部 `src/core/p4/*.cpp` 和 factors；不能只依赖 header glob。

新增 test targets：

    test_p4_coordinates
    test_p4_drt_rotation
    test_p4_shared_bias_imu_factor
    test_p4_fc_factors
    test_p4_visual_factors
    test_p4_window_lifecycle
    test_online_alignment_initializer

旧 `test_online_alignment_candidate_filter` 不再作为正式 P4 验收；如果保留，只证明 legacy 代码，不得列入完成证据。

## 17. 单元与合成验证

### 17.1 坐标和时间

`test_p4_coordinates.cpp` 必须覆盖：

1. identity mount/extrinsic/lever arm；
2. 非交换 roll-pitch-yaw mount composition；
3. passive JPL left perturbation正负号；
4. camera center from `p_IinC`；
5. lever-arm position、velocity、angular-acceleration、centripetal项；
6. `dt_CI/dt_att/dt_nav` 正负号；
7. gravity 与 accelerometer bias seed符号。

这些测试失败时禁止跑真实飞行实验。

### 17.2 DRT golden test

`test_p4_drt_rotation.cpp`：

- 用固定 random seed 生成 bearing pairs；
- 使用导入时由 clone 原始 DRT helper 生成并提交的 golden residual/Jacobian fixture；
- 调 vendor helper；
- residual 绝对差不超过数值双精度容差；
- Jacobian 用 finite difference 复核；
- 加已知 bg 后必须沿正确方向收敛。

### 17.3 shared-bias CPI

`test_p4_shared_bias_imu_factor.cpp`：

- 与原 15D `Factor_ImuCPIv1` 在 `bg_i=bg_j, ba_i=ba_j` 时的 `R/v/p` residual完全一致；
- 9x9 covariance index提取正确；
- q/p/v/bg/ba 每个 Jacobian finite difference；
- 零运动、恒速、恒角速、恒加速度；
- covariance 有交叉项时 whitening正确。

### 17.4 FC factor

`test_p4_fc_factors.cpp`：

- perfect synchronized PVA residual 为零；
- mount perturbation correction方向正确；
- time offset正负方向正确；
- 杆臂在左右相反转弯下符号相反但均能恢复；
- terminal attitude anchor不在末帧时，IMU能正确传播到 terminal；
- keyframe density改变时，increment information不被重复放大。

### 17.5 visual factor

`test_p4_visual_factors.cpp`：

- 已知相对 pose 投影产生零 epipolar residual；
- 纯旋转 pair 只进入 DRT rotation，不进入 translation factor；
- 正/负 translation direction满足 epipolar sign ambiguity；
- 错误 q 和错误 translation direction均产生可检测 residual；
- 非零 `p_IinC`；
- q/p Jacobian finite difference；
- feature 全集中在一条线时 rank gate拒绝。

### 17.6 端到端 synthetic

至少生成：

    straight + acceleration
    left turn
    right turn
    climb/descent
    mixed-axis maneuver
    temporary FC-board flex
    FC attitude latency
    visual outlier burst
    IMU bias

每组知道真实 q/p/v/bg/ba/mount/time。验收使用 normalized error：

    error^T covariance^-1 error

而不是只看绝对误差。还必须验证：

- q/p/v/bg/ba correction方向；
- 反馈/warm-start 后残差下降；
- flex 只降低相关 FC attitude interval权重，不写回永久 mount；
- 左右转得到一致的 nominal mount，不出现固定方向补偿；
- duplicate FC/IMU 不重复建 factor；
- window 前移后旧数据退出、新数据进入；
- sample count变化不改变 release语义；
- 未支持 ba 标为 prior-retained；
- covariance finite/symmetric/PSD；
- clipping、time boundary 或安全失败时不 release。

## 18. 集成验证顺序

### 18.1 构建

正式构建使用并行编译，不再默认 `-j1`：

    cmake --build build_p4_sliding_r1 \
      --target \
        test_p4_coordinates \
        test_p4_drt_rotation \
        test_p4_shared_bias_imu_factor \
        test_p4_fc_factors \
        test_p4_visual_factors \
        test_p4_window_lifecycle \
        test_online_alignment_initializer \
        run_serial_msckf_ros_free \
      -j12

只有出现有证据的内存不足、依赖生成 race 或编译器崩溃时，才降低并行度，并在日志记录原因。

### 18.2 最窄真实数据验证

先使用 fly1/fly3 各半圈到一圈：

- P5 禁用；
- P4 固定 camera input cadence；
- 保存每个窗口的 q/p/v/bg/ba、mount/time、std、information、residual 和 wall time；
- 验证 release、traj_nav 非空、没有 O(history²)；
- 对比冻结 no-change/global baseline。

这一步只用于发现集成错误和明显退化，不用 GPS 未来误差调在线门限。

### 18.3 方向 holdout

fly1/fly3 调通后，固定全部配置，在 fly2/fly4 上直接运行。目的不是再调参，而是检查：

- 相反转弯方向；
- 不同 mounting residual；
- 不同首个转弯位置；
- 视觉/高度/速度分布变化。

如果只在 fly1/fly3 好、fly2明显退化，判定算法未完成。

### 18.4 全程正式验证

按同一代码、配置合同运行 fly1–fly4。正式评价使用现有：

    analysis/full_flight_error_analysis.py
    analysis/flight_eval_tool.py

口径：

- GPS update time 采样；
- start-heading alignment 为主；
- absolute navigation/no post alignment同时报告；
- best-fit 只做诊断；
- reference velocity用 FC raw Ve/Vn/Vu；
- 不用旧 corrected fly2 或错误 truth目录。

对比至少包括：

    frozen global baseline
    frozen no-change
    new P4, P5 disabled
    new P4 + formal P5 integration

新 P4 单独效果必须先通过，不能用 P5 或 AGL postprocess掩盖 P4 退化。

## 19. 正式验收指标

### 19.1 算法正确性

- 所有 coordinate/factor Jacobian测试通过；
- DRT golden equivalence通过；
- synthetic 各状态估计与 covariance统计一致；
- 改变 keyframe density不系统改变解或 covariance；
- 左右转不会产生相反方向的永久 mount补偿；
- FC、IMU、视觉任何一类删除时，diagnostics能准确显示失去的信息子空间。

### 19.2 生命周期

- 每个 advanced window 都有新的 solve receipt；
- receipt 的 sample count由时间窗自然变化；
- 不存在固定 10 次、固定 update count release；
- readiness history以 timestamp prune；
- release后只成功一次；
- release后 P4 不继续改 OpenVINS state。

### 19.3 实时性

    solve_wall_time_p95 < T_advance

同时：

- buffer size受 `T_window+margin` 限制；
- preintegration cache hit rate和重建原因可见；
- 单窗 factor/parameter count有上限；
- 总处理时间随飞行长度线性增长，不随历史长度平方增长。

### 19.4 飞行效果

fly1–fly4 都必须：

- 稳定 release；
- initial q/p/v 与同步 FC PVA在声明 covariance内一致；
- bg/ba/mount 的 estimated/prior-retained状态准确；
- full-flight trajectory非空；
- yaw 首偏、转弯后 yaw 漂移、position RMSE、end error 和 velocity RMSE 全部报告；
- 相同正式口径下不得把明显劣于 frozen global baseline 的结果写成通过。

不接受“初始角度更正确，所以后续更差可以忽略”。如果初始 q 更好而全程更差，必须通过 bias、covariance、first backend update、feature reuse和后端可观性诊断解释并修复。

## 20. 必须写出的 diagnostics

每个窗口一行 `p4_window_trace.csv`：

    window_id
    start/end camera time
    board/FC causal horizon
    FC/IMU/visual raw count
    selected keyframe count
    common track count
    factor counts by family
    time offset estimate/std/status
    mount residual/std/status
    bg/ba estimate/std/status
    terminal q/p/v
    terminal state std
    family RMS/P95/max
    data information eigenvalues
    cross-window normalized deltas
    ready_since for each group
    stage wall times
    cache hit/miss
    release decision/reason

`online_alignment_metadata.json` 保存最终：

    algorithm_id
    config snapshot
    release window
    q/p/v/bg/ba/covariance
    mount/time results
    state source statuses
    release readiness
    build commit

来源字符串可由 runner写入，但 metadata/provenance 缺失不得改变估计或阻塞 release。

## 21. 实现顺序与每步出口

### P0：冻结基准和合同

- 保存当前正确 frozen global baseline 路径、命令、配置和指标；
- 不重跑已有冻结基准；
- 本报告评审通过后才进入代码。

出口：基准 identity和本规格无歧义。

### P1：坐标、DRT 和 factor

- 先写 `test_p4_coordinates`；
- vendor DRT helper并做 golden test；
- 实现 shared-bias CPI、FC、epipolar factor；
- 全部 finite-difference Jacobian通过。

出口：不接 VioManager也能证明数学方向。

### P2：单窗 solver

- 实现 window、cache、motion、metric、joint、covariance；
- synthetic 端到端通过；
- 左右转、flex、latency、density invariance通过。

出口：单窗给出正确 q/p/v/bg/ba 和 covariance。

### P3：真正滑窗

- 实现 timestamp-based advance、warm start、overlap comparison和一整窗 readiness duration；
- 删除 candidate short-circuit；
- 验证每个新窗口都重估。

出口：生命周期与审计要求一致。

### P4：OpenVINS 接入

- `OnlineAlignmentInitializer` 变薄 facade；
- `VioManager` 删除 upstream gauge生产分支；
- terminal-only atomic injection；
- runner/CMake/metadata升级。

出口：最新源码 build/test 全通过，旧二进制不作为证据。

### P5：飞行验证

- fly1/fly3 半圈到一圈；
- fly2/fly4 holdout；
- fly1–fly4 full flight；
- 正式同口径评价；
- P4 单独通过后才组合 P5。

出口：没有隐藏退化、没有 truth调在线 gate、没有后处理掩盖。

## 22. 最终不可变检查表

实现者提交代码前必须逐项回答“是”：

- [ ] `R_GtoI = R_FtoI R_GtoF` 的非交换旋转测试通过。
- [ ] `t_F = t_C + dt_CI - dt_FI` 的符号测试通过。
- [ ] FC nav 和 FC attitude 使用两个独立 time mapping。
- [ ] 杆臂 velocity、angular acceleration和centripetal项都实现。
- [ ] gravity/accelerometer符号与 Propagator一致。
- [ ] q/p/v 是每 keyframe，bg/ba 是 shared。
- [ ] FC metric PVA 在初始化图内，不是事后 gauge。
- [ ] 视觉使用 DRT rotation-only seed和无 landmark epipolar final factor。
- [ ] DRT vendor 与原 clone golden一致。
- [ ] FC terminal absolute只出现一次，increments按物理时间 whitening。
- [ ] 改变 keyframe density不重复乘 FC信息。
- [ ] 高角速度不是自动视觉拒绝条件。
- [ ] flex interval不写回永久 mount。
- [ ] 当前窗推进后完整重估，没有 fixed candidate replay。
- [ ] readiness按 `T_window` 传感器时间，不按次数。
- [ ] covariance包含 mount/time/lever-arm uncertainty影响。
- [ ] ba prior-retained时 covariance和status真实。
- [ ] OpenVINS只注入 terminal q/p/v/bg/ba和15x15 covariance。
- [ ] P4 期间 P5未生效。
- [ ] 构建使用可用并行度，默认 `-j12`。
- [ ] synthetic、短程、holdout和全程正式评价全部完成。
- [ ] 新结果没有明显劣于冻结 global baseline。

任何一项为“否”，都不能写“正式 P4 已完成”。
