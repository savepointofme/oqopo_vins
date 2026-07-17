# P4 正式有限时间联合初始化算法与实现合同（代码审计修订版）

日期：2026-07-17
性质：`REVISED_IMPLEMENTATION_CONTRACT / AWAITING_BUILD_AND_FLIGHT_VALIDATION`
审计基线：tag `baseline/p4-p5-chatgpt-work-20260717`，commit `a96540c68f4211039741ff461883f18479aa9dec`
代码级调研与实现报告：[P4_P5_FORMAL_CODE_AUDIT_AND_IMPLEMENTATION_REPORT_20260717.md](P4_P5_FORMAL_CODE_AUDIT_AND_IMPLEMENTATION_REPORT_20260717.md)

> 2026-07-17 代码审计修订：原稿中“高度重叠窗口连续一整窗确认”、
> `P_current + P_previous` 独立差值 covariance、9D CPI 抽取、P4 内重估
> P3 mount/time，以及以 start-heading alignment 为主口径的条款不成立，
> 已由本版明确替换。代码完成不等于 P4/P5 正式通过；只有依赖齐全的
> native build、Monte Carlo consistency 和 fly1–fly4 验收完成后才能改状态。

## 1. 本报告解决什么问题

本文把下一版正式 P4 的算法、坐标、时间、状态、因子、窗口生命周期、代码文件、函数输入输出、代码复用来源和验收条件一次冻结。后续实现不得靠“看起来差不多”改变这里的语义；如果实现中发现本规格的数学前提不成立，必须先更新本报告并说明证据，再改代码。

冻结基线 `a96540c...` 的生产入口执行：

    upstream OpenVINS dynamic initializer
      -> local metric VIO
      -> FC yaw + translation gauge
      -> AGL scale postprocess

证据是该基线 `run_serial_msckf_ros_free.cpp` 的在线配置把 `upstream_dynamic_init_fc_gauge` 设为 `true`。本 patch 已把正式 runner 改为 `formal_causal_lifecycle=true`、`upstream_dynamic_init_fc_gauge=false`；旧路径只保留为显式 legacy 对照，不再是正式入口。

本规格要求替换为：

    FC navigation q/p/v ─┐
    board raw IMU ───────┼─> causal fixed-duration window
    monocular KLT tracks ┘             │
                                      ├─ fixed P3 mount/time + bg seed
                                      ├─ shared-bias metric joint graph
                                      ├─ immutable causal holdout
                                      ├─ one advanced joint refinement
                                      ├─ covariance/information validation
                                      └─ atomic OpenVINS q/p/v/bg/ba injection

P4 的物理分类是“FC 主系统到 board IMU 从系统的飞行中传递对准，加 FC 约束的单目视觉—惯性初始化”，不是普通纯 VIO 初始化，也不是事后轨迹对齐。

## 2. 完成定义

只有同时满足以下条件，才能称为“正式 P4 已实现”：

1. FC q/p/v 从初始状态生成开始就进入度量联合估计，不再先跑错误局部 VIO 后做 gauge。
2. 窗口由传感器时间长度定义；FC、IMU、视觉量测数由窗口和实际频率自然产生，不以固定 10 次、固定 N 次更新或 3/5/8/12 选择表决定释放。
3. 有限窗先产生 candidate；随后约 2 s 的不相交因果 holdout 只验证、不反馈；通过后最多做一次推进到当前因果终点的完整联合 refinement。旧候选递归滤波器不得代替联合 refinement。
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
- 禁止 candidate 创建后只用 FC p/v 递归反馈改变 q/p/v/bg/ba。允许且要求冻结 candidate 后用随后约 2 s 的 IMU/FC/视觉因果 holdout 做只读验证；成功只授权一次新的完整联合 refinement。
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

### 4.3 FC–board nominal mounting 与 P4 shadow residual

外部接受标定提供：

    R_FtoI_nominal
    P_mount_nominal

正式第一版 P4 把 P3 接受的 nominal mounting 作为固定输入：

    R_FtoI = R_FtoI_nominal

P4 可以计算 `δθ_mount_shadow` 作为诊断，但不得让它改变 q/p/v/bg/ba，
不得写回 nominal calibration，也不得用它吸收 Velcro flex。由 FC 生成
board 姿态：

    R_GtoI,k^FC =
      R_FtoI R_GtoF,k

这个乘法次序必须用三组非交换 roll/pitch/yaw 合成测试锁定，不能只用 yaw 测试。

### 4.4 杆臂

当前声明量：

    p_IinF

FC q/p/v 转换为 board IMU q/p/v：

    R_FtoG,k = R_GtoF,k^T

    p_IinG,k^FC =
      p_FinG,k + R_FtoG,k p_IinF

    v_IinG,k^FC =
      v_FinG,k
      + R_FtoG,k (ω_F,k × p_IinF)

如果未来选择从 FC 二阶差分生成 accelerometer-bias seed，则必须包含完整刚体
杆臂加速度：

    a_IinG,k^FC =
      a_FinG,k
      + R_FtoG,k [
          α_F,k × p_IinF
          + ω_F,k × (ω_F,k × p_IinF)
        ]

本 patch 不使用这种高噪声二阶差分 seed：首窗 `ba_seed=0`，后续窗可用上一窗
shared ba warm start，正式 ba 始终由完整 CPI+FC joint graph 估计。因此当前代码
不得声称已实现 `alpha/centripetal` ba seed；一旦增加该 seed，上式和相应左右转
测试是前置条件。

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
- `dt_FI_att`：使用 P3 接受的外部声明；
- P4 可输出只读 time residual diagnostic，但本版不搜索、不写回、不进入 q/p/v/bg/ba 联合图。

P3 声明的 `σ_dt_att` 必须通过终端角速度 Jacobian 传播到 P4 attitude covariance。`dt_FI_nav` 若没有独立 uncertainty 声明，metadata 必须明确记录“treated fixed by current contract”，不能暗示已传播未知 uncertainty。

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

当前噪声合同不放进每一行，而放进 `OnlineAlignmentOptions`：

    fc_attitude_sigma_deg
    fc_position_sigma_m
    fc_velocity_sigma_mps
    fc_process_variance_fraction scalar in (0,1)
    fc_attitude_to_board_time_offset_sigma_s
    fc_board_mount_sigma_deg

initializer 由这些显式声明构造 `[theta,p,v]` terminal covariance；mount/time
uncertainty进入 attitude block。当前 stream 没有 navigation-time 和 lever-arm
uncertainty 字段，metadata 必须记录 treated fixed，不能偷偷硬编码一个假 covariance。

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

`right_*`、`depth_m` 和 `stereo_valid` 在正式单目 P4 中不参与因子。`normalized_left` 必须是按锁定 intrinsics/distortion 得到的归一化像平面坐标；新代码只把它扩成规范齐次尺度 `[x,y,1]`。不得再把两个齐次向量各自作单位长度归一化，因为 Sampson denominator 对这种独立缩放并不保持不变。

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
- mount/time 只以 P3 external calibration lineage 出现；shadow residual 不得冒充 P4 estimated state。

OpenVINS 实际注入始终是一次原子 q/p/v/bg/ba 注入。所谓“分级”是各状态组在滑窗中的内部 readiness 分级，不是把半个 IMU state 提前写进 EKF。

## 7. 固定量与优化变量

### 7.1 固定量

    R_ItoC, p_IinC, dt_CI
    gravity_G
    p_IinF
    dt_FI_nav
    R_FtoI_nominal, dt_FI_att
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

总 tangent 维数：

    9K + 6

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

### 8.3 有限 candidate、因果 holdout 与一次 refinement

生产生命周期固定为：

    solve finite current window
      -> freeze candidate(q/p/v/bg/ba/P at t_candidate_C)
      -> collect approximately 2 s later causal FC/IMU/visual data
      -> propagate candidate with holdout IMU only
      -> compare endpoint against holdout FC and epipolar observations
      -> fail: discard candidate and slide to a fresh finite window
      -> pass: authorize exactly one newly advanced full joint solve
      -> inject that refined terminal state at current t_end_C
      -> atomically close P4

holdout 期间不得把 FC residual 反馈到 frozen candidate，不得更新 candidate
covariance，也不得增加第二次 refinement。refinement 失败即消耗本次授权并回到
fresh-window retry。

### 8.4 readiness 与相邻窗口统计

readiness 由“当前有限图内部质量 + 随后约 2 s 不相交因果 holdout + 一次
refinement 自身质量”组成，不按固定更新次数，也不要求再连续一个完整
`T_window`。因此默认 8 s 图的最早正常释放约为 10 s 加求解/帧对齐延迟，
不是约 16 s。

相邻 8 s 窗以 0.5 s 推进时共享 93.75% 的时间数据。没有两次估计的
cross-covariance 时，`P_current + P_previous` 不是差值 covariance，禁止把它
用于 normalized release gate。旧 overlap 路径若为实验保留，只能报告物理
差值上限并把 normalized 值标为 unavailable；正式 P4 的独立证据来自随后
holdout。

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

正式 P4 不搜索 P3 time/mount。它只在 P3 固定时间映射上配对 FC rate 与
board gyro，并在固定 `R_FtoI_nominal` 下 robust estimate `bg_seed`。rate residual、
excitation eigenvalues 和 shadow residual 进入 diagnostics，不进入 calibration
写回或独立 release gate。

输出：

    dt_attitude_fixed
    dt_attitude_declared_variance
    R_FtoI_fixed
    bg_seed
    excitation eigenvalues
    robust residual statistics

### 9.3 阶段 C：DRT rotation-only 研究边界

直接参考 DRT-VIO 的 rotation-only gyro-bias 子问题。对每个相邻 keyframe pair：

    bearings_i
    bearings_j
    locked R_ItoC
    IMU ΔR(bg_lin), J_q_bg

DRT 官方实现可构造消去 translation 的最小特征值 residual，并优化：

    bg

但本次 production patch 不 vendor DRT，也不把它当 shared-ba/FC joint graph 的
正确性证据。正式图直接使用固定 P3 calibration、完整 OpenVINS CPI 和
landmark-free epipolar factor；DRT 只保留为未来 bg seed 的独立研究项。

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

本 patch 不调用 DRT residual。若未来把它加入 rotation seed，也不得直接用于最终
covariance，因为其最小特征值不是已冻结的标准高斯量测；当前最终联合图只使用
可按像素噪声 whitening 的 epipolar/Sampson residual、完整 CPI 和 FC PVA。

### 9.4 阶段 D：FC metric q/p/v seed

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

首窗 shared `ba` 从零值开始；后续失败重试窗口可以用上一窗 shared ba warm
start。代码不从 FC velocity 二阶差分伪造一个“已完成的 ba seed”。这一步已经
给出 `G_nav` 中的度量 q/p/v；视觉不是用来事后求 scale，而是在 joint graph 中
通过 pose coupling 提供相对几何。

### 9.5 阶段 E：单窗 joint solve

变量：

    q_k, p_k, v_k
    shared bg, ba

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
    bg/ba priors

求解器：

    trust_region_strategy = LEVENBERG_MARQUARDT
    linear_solver = DENSE_SCHUR
    max_iterations = solver_max_iterations
    max_solver_time = solver_max_time_s
    num_threads = 1

`max_iterations/max_solver_time` 是计算上限，不是收敛证据。每次 solve 都从当前
shared bg/ba linearization 构造完整 CPI；本 patch 依赖 upstream CPI 的 bias
Jacobian 在该次 solve 内修正，不声称已实现 solve 后再次真实重积分。若 Monte
Carlo 或飞行数据表明 linearization 误差不可接受，再增加“solve -> rebuild CPI at
solved biases -> bounded resolve”，并单独计作 joint solve 内部数值步骤，不能突破
生命周期的一次 advanced refinement 语义。

### 9.6 阶段 F：immutable holdout 与一次 advanced refinement

初次 joint solve 只形成 frozen candidate。随后约 2 s holdout 用 IMU 传播该候选，
FC/视觉只评价不反馈。通过后只授权一次新终点 full joint solve；该 solve 重新从
当前窗口原始 IMU 构造 CPI，并拥有最终 covariance。失败则丢弃候选、窗口前移。

### 9.7 阶段 G：terminal covariance 与 calibration uncertainty

从最终 tangent Hessian 恢复并边缘化得到：

    P_terminal =
      Cov([δθ_K, δp_K, δv_K, δbg, δba])

P3 mounting 在本版是 fixed external calibration，不在 P4 联合优化。其声明的
isotropic small-angle sigma 加入 FC terminal attitude covariance，同时 metadata
单独报告 calibration covariance；这表示 calibration uncertainty 的一阶传播，
不是把 fixed mount 伪造成 P4 data posterior。

time offset 在外层搜索中固定，必须额外传播：

    P_terminal <-
      P_terminal
      + J_x_dt σ_dt² J_x_dt^T

当前实现至少把 `σ_dt_att` 通过终端角速度 Jacobian传播到 FC attitude boundary。
`dt_FI_nav` 和 lever arm 若缺少外部 uncertainty 声明，必须明确按 fixed input
处理；不得写成“已传播”。完整 `J_x_dt/J_x_lever` Monte Carlo coverage 是正式
飞行验收前的未完成验证项。

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

    posterior std in physical tangent units
    dimensionless data-only minimum information eigenvalue
    prior information contribution
    IMU/FC/visual Jacobian support
    causal holdout residuals
    source status

`prior_only` 的定义不是“估计值接近 prior”，而是移除 prior 后该状态组的数据 Schur information 不足。

release 必须同时满足：

- 当前 solve usable；
- terminal covariance finite/PSD；
- q/p/v/bg ready；
- visual 和 IMU 均对 q/p/v 的至少一个相关子空间有非零增量信息；
- FC 对 global gauge 和 metric p/v 有增量信息；
- 没有 saturation、time boundary、flex dominance 或 covariance correction safety failure；
- frozen candidate 的随后约 2 s causal holdout通过，且唯一一次 advanced refinement 自身 gate 通过；
- release timestamp 不晚于当前已喂入的 IMU/camera 因果 horizon。

## 10. 因子数学定义

### 10.1 shared-bias CPI factor

新 adapter 保留 upstream `Factor_ImuCPIv1` 的完整 15D residual/covariance，
参数块：

    q_i, shared_bg, v_i, shared_ba, p_i,
    q_j,            v_j,            p_j

adapter 调用 upstream factor 时把 `bg_i/bg_j` 指向同一参数块、把
`ba_i/ba_j` 指向同一参数块；返回 shared-bias Jacobian 时分别求和：

    J_shared_bg = J_bg_i + J_bg_j
    J_shared_ba = J_ba_i + J_ba_j

这样 bias random-walk 两组 residual 在 shared model 下严格为零，但完整 whitening
和它们与 R/v/p 的 cross terms 保留。禁止抽取 9×9 子矩阵，因为 upstream
实际 residual 顺序为 `[R, bg, v, ba, p]`，删行后并不等价于在原 15D likelihood
中绑定参数。

### 10.2 FC terminal factor

FC 使用一个 dense correlated trajectory factor。绝对 boundary 位于被实际
注入的 terminal camera timestamp；其姿态 residual：

    r_q_abs =
      Log(
        R_GtoI,K
        (R_FtoI R_GtoF,K)^T
      )

窗口最后一帧提供 absolute p/v：

    r_p_abs =
      p_K - p_IinG,K^FC

    r_v_abs =
      v_K - v_IinG,K^FC

terminal q/p/v 使用同一个 9×9 `P_terminal`。若输入只声明三个独立 sigma，
off-diagonal 为零；不得臆造相关项。attitude time uncertainty 通过终端角速度
加入其 3×3 block。

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

冻结 FC error state：

    e = [δθ, δp, δv]
    Φ = I9
    P0 = (1-f) P_terminal
    Qd_i = f P_terminal Δt_i / T_window,  0 < f < 1
    Cov(e_i,e_j) = P0 + Σ_{k<=min(i,j)} Qd_k

一次性构造所有 absolute errors 的 joint covariance `Σ_abs`，再用线性变换
`A=[terminal absolute; chronological increments]` 得到 `Σ_r=AΣ_absAᵀ`，
对整个 residual 一次 whitening。terminal 与相邻 increments 相关，不能拆成
独立 Ceres residual blocks。在此 Brownian surrogate 下，改变 keyframe density
不得改变同一线性物理 error path 的 likelihood。`Φ=I9` 是明确的第一版模型，
不是对 FC 内部滤波器真实 dynamics 的宣称；必须由真实数据 density ablation
和 Monte Carlo coverage 验证。

### 10.4 monocular epipolar factor

对 camera i/j common track：

    R_Ci_to_Cj =
      R_GtoC,j R_GtoC,i^T

    t_Ci_in_Cj =
      R_GtoC,j (p_CinG,i - p_CinG,j)

    e =
      f_j^T [t_Ci_in_Cj]_x
      R_Ci_to_Cj f_i

使用 normalized Sampson denominator，把 residual 转为近似 pixel sigma 可解释量。baseline 太小时该 pair 不建 epipolar translation factor；本 patch 的 rotation 信息由 FC attitude、完整 CPI 和其它有效视觉 pair 提供。只有未来通过独立 golden test 引入 DRT 后，纯旋转 pair 才可额外进入 DRT rotation-only seed。

visual factor 直接连接 q_i,p_i,q_j,p_j 和固定 Camera–IMU calibration，不建立 landmark parameter block。

### 10.5 priors

只允许以下有物理来源的 prior：

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
      -> JOINT_SOLVING_CANDIDATE
      -> CANDIDATE_VALIDATING        immutable ~2 s holdout
          -> COLLECTING_WINDOW       reject and slide
          -> CANDIDATE_REFINING      holdout pass, one authorization
      -> JOINT_SOLVING_REFINEMENT
          -> COLLECTING_WINDOW       refinement failure
          -> NAVIGATION_READY        atomic release
          -> FULL_ALIGNMENT_READY    atomic release
      -> CLOSED_AFTER_RELEASE

配置非法进入：

    FATAL_CONFIGURATION_ERROR

不再存在生产语义：candidate sequential FC feedback、无限 refinement、
upstream dynamic init FC gauge、shadow-only release。`CANDIDATE_VALIDATING`
表示 immutable holdout，绝不表示旧 recursive candidate filter。

## 13. 逐文件实现合同

本次 correctness patch 先把新 likelihood 拆到 `core/p4/factors/`，并把 formal
lifecycle 作为显式 option 接入现有 supervisor；旧 candidate/gauge 路径仍仅供
历史对照，不由 runner 正式配置选择。把 buffer/calibrator/solver/validator 再
拆成薄 supervisor 是后续可维护性工作，不能在没有独立行为等价测试时做大规模
搬运。

### 13.1 本 patch 的实际边界

本 patch 不虚构未落地的 `P4Types/P4WindowBuffer/P4JointRefiner` 文件。为降低一次性
搬运造成的坐标和生命周期回归，buffer、插值、构窗、求解、covariance 和 release
gate 暂时保留在已有 `OnlineAlignmentInitializer` 中；新的统计 likelihood 单独放进
`core/p4/factors/`。正式 runner 只选择新的 `formal_causal_lifecycle` 分支，旧
candidate/gauge/shadow 代码不在本次删除，以便现有对照测试继续编译，但不能作为
正式入口。

本 patch 的真实改动文件和责任如下：

| 文件 | 责任 | 复用/实现规则 |
| --- | --- | --- |
| `OnlineAlignmentInitializer.h/.cpp` | 三流因果构窗、共享 bg/ba 图、immutable holdout、一次 refinement、15×15 terminal covariance | 复用现有插值、CPI、Schur/covariance 工具；替换旧 per-keyframe bias、landmark reprojection 和 overlap normalized release |
| `core/p4/factors/Factor_P4ImuSharedBias.*` | 完整 15D CPI 的 shared-bias adapter | 直接调用 OpenVINS `Factor_ImuCPIv1`；两个端点绑定同一 bg/ba，Jacobian 列求和 |
| `core/p4/factors/Factor_P4FcTrajectory.*` | terminal absolute + chronological increments 的单个稠密相关 FC PVA likelihood | 从头实现冻结的 Φ/Qd/Σ/A 和 whitening；不拆成独立 residual blocks |
| `core/p4/factors/Factor_P4Epipolar.*` | 固定 camera–IMU 外参的 landmark-free normalized Sampson residual | 从头实现；每 feature 只建一个最大时距 pair |
| `VioManager.cpp` | 正式 P4 fail-closed 接线、原子注入；P5 termination 输入接线 | upstream initializer 只保留给显式 legacy gauge 实验 |
| `BackendUpdateTrigger.h` | 从真实 track survival/border 状态计算 P5 termination risk | 不依赖固定帧号或未来轨迹 |
| `run_serial_msckf_ros_free.cpp` | 唯一正式配置和可审计 metadata | `formal=true`、`upstream gauge=false`、P3 mount/time fixed |
| `cmake/ROS1.cmake`, `cmake/ROS2.cmake` | 新 factors 和测试编译入口 | 两套构建都必须包含同一 sources/tests |
| `test_p4_formal_factors.cpp` | shared CPI、FC covariance/density、epipolar geometry/Jacobian killer tests | C++ 原生依赖齐全时运行 |
| `scripts/validate_p4_formal_contract.py` | 无 ROS/Ceres 主机上的独立数学与 source topology 检查 | 不能替代 C++ build、Monte Carlo 或 flight validation |

### 13.2 当前调用顺序

    feed_fc_navigation / feed_board_imu / feed_stereo
      -> try_initialize(causal camera horizon)
      -> build one finite common-support window
      -> q/p/v per keyframe + one shared bg/ba
      -> dense FC factor + complete shared-bias CPI + sparse epipolar pairs
      -> solve and recover terminal marginal
      -> freeze candidate
      -> later ~2 s IMU-only propagation, FC/visual holdout evaluation
      -> if fail: discard candidate and slide
      -> if pass: authorize exactly one newly advanced joint solve
      -> atomic camera-clock q/p/v/bg/ba + 15x15 covariance release
      -> close P4 permanently

### 13.3 明确延后而非伪装完成的重构

只有原生 tests、Monte Carlo consistency 和 fly1–fly4 验收通过后，才允许把现有
supervisor 机械拆分成 `P4WindowBuffer`、`P4JointRefiner`、
`P4ReleaseValidator` 等薄模块。该重构必须保持 residual、parameter ordering、
timestamp 和 state-machine traces 的 golden equivalence；它不是本次算法正确性的
前置条件，也不得写成本 patch 已创建的文件。

DRT vendor 同样不在本 patch 范围。若未来引入，只限 rotation/bg seed，必须先有
许可证、固定 commit 和 residual/Jacobian golden fixture；不得用它替换 OpenVINS
CPI，或声称它证明 shared ba、FC PVA 与完整 joint graph。

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

- 参数从前后两个 bg/ba 变为一个 shared bg 和一个 shared ba；
- upstream factor 的前后 bias pointer 绑定到同一 shared block；
- shared Jacobian 分别为两个 endpoint Jacobian 之和；
- residual、完整 15×15 whitening 和 covariance cross terms 原样保留；
- 参数顺序按报告 10.1 节，并为每个 block 写 equivalence test。

### 14.2 `Factor_P4FcTrajectory.h/.cpp`

从头实现 dense correlated trajectory likelihood。输入：

    all synchronized FC PVA targets and timestamps
    all q/p/v state blocks
    terminal 9x9 covariance
    process_variance_fraction f

输出维数 `9K`，顺序为 terminal absolute 后接 chronological increments；
使用完整 `AΣ_absAᵀ` 一次 whitening，不把共享 endpoint 的 residual 当独立。

### 14.3 DRT rotation-only reference（不进入本次 production patch）

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

DRT 官方代码在 `gyroBiasEstimator()` 中只优化 `biasg`；后续 translation/gravity
阶段沿用构造函数置零的 `biasa`。因此 DRT 只能作为 rotation/bg seed 的研究
依据，不能证明本项目 shared-ba/FC joint graph。第一版 production patch 不
vendor DRT 代码，以免把未完成的 license/golden equivalence 冒充正式依赖；
若后续引入，必须先满足上述 golden test。

### 14.4 `Factor_P4Epipolar.h/.cpp`

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
| 当前 OpenVINS 基线 | `a96540c...`（上游接口审计 `6948812...`） | GPLv3/MIT mixed by file | 直接复用完整 CpiV1 likelihood、JPL manifold、atomic state injection | upstream initializer + gauge 正式路径 |
| DRT-VIO | `fb0ac8d...` | GPLv3 | 只作 rotation/bg 研究依据，本 patch 不复制代码 | translation/ba、tracker、preintegrator、硬编码门限 |
| VINS-Mono | `90dabb5...` | GPLv3 | 只作 staged SFM→VI alignment 对照 | 不复制 solver |
| ORB-SLAM3 | `4452a3c...` | GPLv3 | 只采用“fixed/seed pose + per-KF velocity + shared bg/ba”的架构依据 | 不复制 g2o 实现 |
| GVINS | `d2cf40b...` | GPLv3 | 只作 VIO init 后 GNSS alignment 的反例对照 | 不复制 GNSS solver |
| MINS | `d0e0ea2...` | GPLv3 | 只参考多传感器 calibration/state ownership | 不复制 updater |

若未来加入 DRT vendor 目录，必须保留：

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

正式配置必须显式覆盖旧默认：

    formal_causal_lifecycle = true
    upstream_dynamic_init_fc_gauge = false
    sliding_window_shadow_only = false
    sliding_window_direct_state_release = false
    sliding_window_deferred_release_certification = false
    max_initial_clones = 0
    max_initial_slam_features = 0
    candidate_window_durations_s = {8.0}
    candidate_short_validation_duration_s = 2.0
    candidate_refinement_enabled = true

本 patch 新增显式 FC model 配置：

    --online-alignment-fc-process-fraction f, 0 < f < 1

其余 window、tracking、terminal sigma、mount/time declaration 和 solver budget
暂沿用已有已审计字段，不能在本 patch 中改名后丢失历史对照。metadata schema
升级并写出固定算法 ID：

    openvins_p4_fc_pva_shared_bias_epipolar_v1

legacy candidate/gauge 字段可以为兼容保留，但 formal-specific FC covariance、
visual pair policy、holdout counts、one-refinement receipt 和 normalized information
字段必须同时存在；不得靠把旧字段置零冒充新算法。

### 16.4 CMake

当前 `ov_msckf/cmake/ROS1.cmake` 和 `ROS2.cmake` 显式列 `LIBRARY_SOURCES`。两处同时加入全部 `src/core/p4/*.cpp` 和 factors；不能只依赖 header glob。

本 patch 新增并实际接入的正式 test target：

    test_p4_formal_factors
    test_online_alignment_initializer
    test_adaptive_stride

`test_p4_formal_factors` 聚合 shared CPI、FC covariance/density、epipolar geometry
和各参数 Jacobian killer tests。旧 `test_online_alignment_candidate_filter` 保留时
只证明 legacy 代码，不得列入正式 P4 完成证据。

## 17. 单元与合成验证

### 17.1 本 patch 已落地的 dependency-light 检查

`scripts/validate_p4_formal_contract.py` 在没有 ROS/Ceres 的主机上检查：FC dense
covariance SPD、terminal exact、terminal/increment correlation、coarse/fine density
invariance，以及 runner/factor/lifecycle/P5/handoff 的 source topology。它只是一道
独立可执行检查，不能替代 native C++ 或 flight validation。

### 17.2 本 patch 已落地的原生 factor killer tests

`test_p4_formal_factors.cpp` 覆盖：

- shared CPI adapter 与原 15D factor 在 tied endpoints 下 residual 完全一致；
- shared bg/ba Jacobian 等于两个 endpoint columns 之和；
- FC covariance SPD、terminal exact、cross-correlation 和 density invariance；
- FC q/p/v Jacobian 对独立 JPL finite difference；
- known two-view geometry 的 epipolar residual；
- off-epipolar detection；
- epipolar q/p Jacobian 对独立 JPL finite difference；
- quaternion ambient scalar trick 与 `State_JPLQuatLocal` 一致。

### 17.3 生命周期与 P5 tests

`test_online_alignment_initializer.cpp` 增加 formal candidate -> later holdout -> one
advanced refinement -> current camera-clock atomic release；release 后不得再次求解。

`test_adaptive_stride.cpp` 增加 healthy tracks、track loss 和 border concentration
三类 feature-termination 输入检查。

### 17.4 仍属于正式验收而非本容器已通过的 synthetic/Monte Carlo

目标环境仍须覆盖 straight acceleration、left/right turn、climb/descent、mixed-axis
maneuver、temporary flex、FC latency、visual outlier burst 和 IMU bias。每组使用真实
`q/p/v/bg/ba` 和

    error^T covariance^-1 error

做 NEES/coverage，而不只看绝对误差。必须验证坐标/时间/杆臂符号、window 前移、
duplicate data、finite symmetric PSD covariance、左右转一致性和安全失败不 release。

### 17.5 DRT golden test（只在未来 vendor DRT 时）

本 patch 没有 DRT dependency，因此当前正式 build 不要求 DRT golden。未来若
vendor，必须用固定 commit 原 helper 生成 residual/Jacobian fixture，并做 finite
difference 与已知 bg 收敛方向检查；完成前不得进入 production。

## 18. 集成验证顺序

### 18.1 构建

正式构建使用并行编译，不再默认 `-j1`：

    cmake --build build_p4_sliding_r1 \
      --target \
        test_p4_formal_factors \
        test_online_alignment_initializer \
        test_adaptive_stride \
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
- absolute navigation/no post alignment 为主；
- start-heading alignment 只作漂移诊断；
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
- 若未来 vendor DRT，其 golden equivalence先通过；当前 patch不依赖 DRT；
- synthetic 各状态估计与 covariance统计一致；
- 改变 keyframe density不系统改变解或 covariance；
- 左右转不会产生相反方向的永久 mount补偿；
- FC、IMU、视觉任何一类删除时，diagnostics能准确显示失去的信息子空间。

### 19.2 生命周期

- 每个 advanced window 都有新的 solve receipt；
- receipt 的 sample count由时间窗自然变化；
- 不存在固定 10 次、固定 update count release；
- candidate holdout以 sensor timestamp计时，不按固定 update count；
- release后只成功一次；
- release后 P4 不继续改 OpenVINS state。

### 19.3 实时性

    solve_wall_time_p95 < T_advance

同时：

- buffer size受 `T_window+margin` 限制；
- 每个 CPI interval的精确边界支持和失败原因可见；
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
    physical-unit and dimensionless-normalized data information eigenvalues
    formal holdout FC/IMU/visual counts and duration
    formal holdout pass/fail and single-refinement receipt
    stage wall times
    exact CPI interval support/failure reason
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

### I0：冻结基准和合同

- 固定 branch/tag/commit，记录基线真实入口和调用链。

状态：完成。出口：`a96540c...` identity 与基线缺陷表。

### I1：坐标、上游等价性和 factor

- 固定 OpenVINS、VINS-Mono、ORB-SLAM3、GVINS、MINS、DRT commit；
- 对 OpenVINS CPI 做 shared-parameter equivalence；
- 冻结 FC dense covariance 与 epipolar pair policy；
- DRT 只保留代码级研究结论。

状态：源码与 tests 已实现；等待原生 C++ 执行。出口：
`test_p4_formal_factors` 全绿。

### I2：单窗 solver

- 在现有 supervisor 中实现每帧 q/p/v + shared bg/ba；
- 安装 dense FC、完整 CPI、one-pair-per-track epipolar；
- 回收 terminal 15×15 covariance 和 dimensionless information gates。

状态：代码完成，dependency-light density/source checks 通过；等待 native build、
Monte Carlo。出口：单窗真值误差与 covariance coverage 通过。

### I3：因果 lifecycle

- timestamp-based finite candidate、约 2 s immutable holdout；
- holdout 通过只授权一次 advanced joint refinement；
- 禁止缺少 cross-covariance 的 overlap normalized gate；
- current camera-clock atomic release 后永久关闭。

状态：代码和测试场景已加入；等待 native test。出口：state-machine receipt与
timestamp assertions全绿。

### I4：OpenVINS 接入

- runner正式选择 formal path并显式关闭 upstream gauge；
- `VioManager` formal fail-closed；terminal-only atomic injection；
- ROS1/ROS2 CMake、algorithm ID、formal metadata；
- P5 real feature-termination input。

状态：代码完成；旧实现留作 legacy 编译对照但不由正式 runner选择。出口：GitHub
ROS-free/ROS1/ROS2 build全绿，旧二进制不作为证据。

### I5：飞行验证

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
- [ ] 杆臂 position/velocity 项已实现；当前未使用 FC 二阶差分 ba seed，metadata未冒充 angular-acceleration/centripetal seed 已实现。
- [ ] gravity/accelerometer符号与 Propagator一致。
- [ ] q/p/v 是每 keyframe，bg/ba 是 shared。
- [ ] FC metric PVA 在初始化图内，不是事后 gauge。
- [ ] 视觉 final factor 无 landmark，且每 feature 只选一对 pixel observations。
- [ ] production patch 未把 DRT translation/ba 能力写成已证明；若未来 vendor，原 clone golden一致。
- [ ] FC terminal absolute只出现一次，increments按物理时间 whitening。
- [ ] 改变 keyframe density不重复乘 FC信息。
- [ ] 高角速度不是自动视觉拒绝条件。
- [ ] flex interval不写回永久 mount。
- [ ] frozen candidate 的随后约 2 s holdout 不做 FC feedback。
- [ ] holdout 通过后只进行一次 advanced full joint refinement。
- [ ] overlapping windows 未在缺少 cross-covariance 时使用 `Pcur+Pprev` normalized gate。
- [ ] covariance包含已声明的 mount/attitude-time uncertainty；navigation-time/lever-arm缺少声明时明确 treated fixed。
- [ ] ba prior-retained时 covariance和status真实。
- [ ] OpenVINS只注入 terminal q/p/v/bg/ba和15x15 covariance。
- [ ] P4 期间 P5未生效。
- [ ] 构建使用可用并行度，默认 `-j12`。
- [ ] synthetic、短程、holdout和全程正式评价全部完成。
- [ ] 新结果没有明显劣于冻结 global baseline。

任何一项为“否”，都不能写“正式 P4 已完成”。
