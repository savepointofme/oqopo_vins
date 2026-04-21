# OpenVINS 2 个月精通学习计划 (8 周)

> 对象: 你已能运行 ROS-free 版本 (PR#2), 已有一份中文架构/公式文档 (PR#1/#3),
> 同时正在推进标定 (主线工作)。本计划的目标是 **8 周后达到** ——
> (1) 看到 OpenVINS 任意一段代码都能口述它在做什么、对应公式、输入输出、
> (2) 能定位改动点 (例如"我要加一种新观测"、"我要换一个 IMU 预积分形式"),
> (3) 能把标定结果通过 OpenVINS 在线标定验证 (W6)。

## 总体原则

1. **每周 8~10 小时学习 + 你的标定主线**, 不要互相抢时间。
2. **不要从 `main()` 入手**, 从 **`State.h` / `IMU.h` / `StateHelper.h`** 入手。
3. **一次只死磕一个公式**。OpenVINS 的硬骨头大概 5~6 个, 每周一个。
4. **先写测试/打印, 再读代码**。每周安排一个 "动手小实验", 实验失败比读懂强。
5. **不懂就把 chan 截图发给我**。我按你的问题再修/扩 docs-cn/ 里的注释。

---

## 周度路线图

| 周次 | 主题 | 时间占比 | 交付物 |
|------|------|---------|--------|
| W1 | 运行 / 基础数学 / JPL 四元数 | 7h | 能解释"误差状态 EKF"为什么要叫"间接"; Owner 文档 `math_foundations.md` §1-3 内化 |
| W2 | State 内存布局 / KLT 前端 | 8h | 手画一张你自己的 `_Cov` 行列图 (对比 `state_and_cov.md`) |
| W3 | IMU 传播 / Trawny Eq.101 | 9h | 对 `Propagator::predict_mean_discrete` 打印 Φ/Q 并与手推对比 |
| W4 | **MSCKF 更新核心 (最难的一周)** | 12h | 3×2 零空间 + QR 压缩的手写推导, 画流程图 |
| W5 | SLAM 持久特征 / 初始化 | 8h | 改 `StaticInitializer` 的协方差, 观察 NEES 变化 |
| W6 | **在线标定 (对接你的主线)** | 9h | 启用 `calib_cam_extrinsics=1`/`calib_cam_timeoffset=1`, 和你离线标定结果对比 |
| W7 | 可观测性 / FEJ / 退化场景 | 8h | 跑纯直线 vs 带旋转仿真, 看 yaw 协方差曲线 |
| W8 | 综合 + 自选方向 | 8h | 读一篇近年 VIO 论文, 提出一个可实现的改进点 |

---

## W1 — 运行 / 基础数学 / JPL 四元数

### 学习目标
- 说出 OpenVINS 和其他 VIO (VINS-Mono, MSCKF1.0, ORB-SLAM3) 的 3 个关键区别。
- 说出为什么 JPL 四元数是 `[q1 q2 q3 q4]` (q4=标量) 而不是 `[w x y z]`。
- 能手推误差状态 EKF: `x = x̂ ⊞ δx`, `P` 对应 `δx` 的协方差, 而非 `x` 本身。

### 阅读
- `docs-cn/math_foundations.md` §1 JPL 约定 §2 SO(3) hat/exp §3 误差状态 EKF
- `ov_core/src/utils/quat_ops.h` **全文** (已有完整中文注释, 看一遍顶部 `[中文]` 总览块)
- `ov_core/src/types/Type.h` (基类约 50 行)

### 动手实验
1. 跑一次 `run_serial_msckf_ros_free --config config/euroc_mav/estimator_config.yaml --dataset /path/to/V1_01_easy --stereo`
2. 在 `Propagator::predict_and_compute` 入口加一行 `PRINT_INFO("dt=%.4f |w|=%.3f |a|=%.3f\n", dt, w_hat.norm(), a_hat.norm())`, 观察不同运动状态下数值范围。
3. 手写: 对 `q = [0.1, 0.2, 0.3, sqrt(1-0.14)]`, 用 `quat_2_Rot` 计算得 `R1`, 再用 `rot_2_quat(R1)` 倒回去, 检查结果数值吻合。

### 标定并行任务 (1 天)
- 不碰 OpenVINS。你自己的标定继续推进即可。

---

## W2 — State 内存布局 / KLT 前端

### 学习目标
- 闭着眼说出 OpenVINS 的完整状态向量: IMU(16) + N × clone(7) + SLAM × m(1~3) + 标定 × ...
- 理解 `_variables` 顺序如何与 `_Cov` 行列索引绑定 (`set_local_id`)。
- 说出 KLT 前端的 4 个步骤 (金字塔, 检测, 正向追踪, 双向过滤) 及为什么要双向。

### 阅读
- `docs-cn/state_and_cov.md` (概念+内存图)
- `ov_core/src/types/{Type.h, Vec.h, JPLQuat.h, PoseJPL.h, IMU.h, Landmark.h}` (全部已双语注释)
- `ov_msckf/src/state/State.h` / `State.cpp` (关注成员 `_imu`, `_clones_IMU`, `_features_SLAM`, `_calib_*`)
- `ov_msckf/src/state/StateHelper.h` (EKFPropagation / EKFUpdate / augment_clone / marginalize)
- `ov_core/src/track/TrackKLT.cpp` (perform_matching / perform_detection_monocular)

### 动手实验
1. 自己画一张 `_Cov` 的矩阵图 (类似 `docs-cn/diagrams/07_state_structure`), 标出每一块是谁。
2. 在 `State` 构造完成后打印 `_Cov.rows()` 和 `StateHelper::get_marginal_covariance(state, {state->_imu->q()})` 的 3×3 角度协方差, 感受数量级 (rad²)。
3. 改 `TrackKLT::perform_matching` 里 `fwdbwd_thresh` 从默认 0.5 到 2.0, 观察 KLT 跟踪点数量变化。

### 标定并行任务 (2 天)
- 跑你的标定工具, 整理相机内参输出格式 → W6 要对接进 YAML。

---

## W3 — IMU 传播 / Trawny Eq.101

### 学习目标
- 口述 IMU 连续时间微分方程: `q̇ = 0.5Ω(w-bg)q`, `ṗ = v`, `v̇ = R(a-ba) + g`, `ḃg=n_gw`, `ḃa=n_aw`。
- 写出 `F, G, Φ, Q_d` 的含义。
- 说出为什么 RK4 比 Euler 好, 但 OpenVINS 主版本为什么只用 discrete 中点 (性能 + 数值稳定)。

### 阅读
- `docs-cn/propagation_math.md` (完整, 对每一行公式有对应代码位置)
- `ov_msckf/src/state/Propagator.{h,cpp}` **重点**:
  - `propagate_and_clone` (主入口)
  - `predict_and_compute` (一步积分 + 协方差传播)
  - `predict_mean_discrete` / `predict_mean_rk4` (两种积分方案)
  - `compute_F_and_G_analytic` / `compute_Phi_Qd_analytic` (离散化)

### 动手实验 (最有价值)
1. 在 `predict_mean_discrete` 末尾加 `PRINT_INFO("Phi.norm=%.3f Qd.trace=%.3e\n", Phi.norm(), Qd.trace())`。
2. 在 EuRoC V1_01 前 10 秒 (静止) 和第 80 秒 (快速旋转) 各取一次值对比, 应当看到 `Qd.trace` 在旋转时显著增大。
3. 在纸上手推 `Φ = I + F·dt + 0.5·(F·dt)²`, 对一个仅含 δθ 和 δbg 的玩具系统, 确认 OpenVINS 代码对应。

### 标定并行任务 (2 天)
- 把你标定的 IMU 内参 (如果有的话) 填入 `estimator_config.yaml` 的 `imu_noises`/`gyroscope_random_walk`。

---

## W4 — MSCKF 更新核心 **(最难的一周, 让开其他事情)**

### 学习目标
- 手推 H_f (3×3 投影雅可比), H_x (投影对状态的雅可比)。
- 说清楚"左零空间投影"为什么能消掉特征的未知深度, 几何意义是什么。
- 说明 Givens / QR 压缩从 m×n 到 n×n 为什么等价于 EKF 更新结果。

### 阅读
- `docs-cn/measurement_math.md` §1~§5 (全部, 代码行号对齐)
- `docs-cn/feature_triangulation.md` (配合)
- `ov_msckf/src/update/UpdaterHelper.{h,cpp}` **重点**:
  - `get_feature_jacobian_full` (构造 H_f, H_x)
  - `nullspace_project_inplace` (左零空间)
  - `measurement_compress_inplace` (QR 压缩)
- `ov_msckf/src/update/UpdaterMSCKF.cpp::update` (调用链 + 卡方门限)

### 动手推导
- 取一个 3×2 的简易 H_f (一个特征, 两个观测), 手算左零空间 U_⊥ 的 Givens 旋转过程, 确认输出 U_⊥^T H_f = 0。
- 把结果和 `nullspace_project_inplace` 内部用的 `ColPivHouseholderQR` 对齐看。

### 动手实验
1. 在 `UpdaterMSCKF::update` 开始和结束各打印 `Cov.trace()`, 看每次更新协方差下降比例。
2. 把卡方门限 `chi2_multiplier` 从 1.0 改到 5.0, 看离群点 rejection 率 (需要自己加计数打印)。

### 标定并行任务 (1 天, 少量)
- 整理你标定的相机-IMU 外参, W6 要作为 prior 填入 `calib_extrinsics` 部分。

---

## W5 — SLAM 持久特征 / 初始化

### 学习目标
- 说清 SLAM 特征 vs MSCKF 特征的区别: SLAM 点进状态 (有 id + 协方差), MSCKF 点只在本次更新里出现, 之后删除。
- 说明 "anchored inverse depth" 为什么能提高可观性 (线性化点锁在锚 clone)。
- 口述初始化三件套: `InertialInitializer` 选择 → `StaticInitializer` 或 `DynamicInitializer`。

### 阅读
- `docs-cn/slam_update.md` (如果还没写, W5 前我会补上)
- `docs-cn/initialization.md` (已有)
- `ov_msckf/src/update/UpdaterSLAM.{h,cpp}` (delay_init / update_skf / change_anchors)
- `ov_init/src/init/InertialInitializer.cpp` (已有详细中文注释)
- `ov_init/src/static/StaticInitializer.cpp` (已有完整双语块注释)
- `ov_init/src/dynamic/DynamicInitializer.cpp` (大文件, 先看 `initialize()` 6 个 stage, 批次 3 我会补注释)
- `ov_core/src/types/Landmark.h` (已双语)

### 动手实验
1. 改 `StaticInitializer.cpp` 里 `cov.block<3,3>(0,0)` 的 `prior_rot_cov` 因子 (默认 1e-4), 调成 1e-6 或 1e-2, 看 ATE。
2. 启动日志打开 `OV_DEBUG` 级别, 观察 `try to initialize ... feat track cnt` 的输出频率。

### 标定并行任务 (2 天)
- 把你标定的相机内参 (k1,k2,p1,p2 或 Kannala-Brandt) 填入 `cam0_distortion_coeffs`。跑 ATE 做对比 (标定前/后)。

---

## W6 — 在线标定 (对接你的主线)

### 学习目标
- 说出 OpenVINS 支持的在线标定项: 相机内参、相机-IMU 外参、时间偏移, IMU 内参 (可选)。
- 理解它们在状态向量里的位置和在 `_Cov` 中的块。
- 区分 "离线 kalibr 给 prior" vs "在线自举 (no prior)" 的风险。

### 阅读
- `docs-cn/online_calibration.md` (完整, 对每个标定开关和雅可比位置有引用)
- `ov_msckf/src/state/State.h::_calib_*` 成员
- `ov_msckf/src/update/UpdaterHelper.cpp` 里 `calib_cam_extrinsics` / `calib_cam_intrinsics` / `calib_cam_timeoffset` 相关的 Jacobian 分支
- `ov_msckf/src/core/VioManager.cpp::feed_measurement_camera` 中 camera timeoffset 的应用

### 动手实验 (和你标定工作强相关)
1. 用你标定得到的 `T_cam0_imu` 作为 prior 填进 YAML。
2. 先用 `calib_cam_extrinsics=0` 跑一次 baseline ATE。
3. 再用 `calib_cam_extrinsics=1` 跑一次, 记 ATE 和收敛后的外参值。比较差异。
4. 若你有 timeoffset 估计: `calib_cam_timeoffset=1` 验证在线估计能否收敛到你离线值的 ±5ms 内。

### 标定并行任务 (3 天, 主要工作)
- 把 W6 的实验结果写成一页内部报告。这就是你的"标定结论"一部分。

---

## W7 — 可观测性 / FEJ / 退化场景

### 学习目标
- 说清楚 VIO 的 4-DOF 不可观: 3 位置 + 1 yaw (因为重力锁定 roll/pitch)。
- 说明 FEJ (First-Estimates Jacobian) 为什么能防止 yaw 漂移发散。
- 认出常见的退化运动: 纯直线 (scale 弱约束)、纯旋转 (位移不可观)、zero-velocity (需要 ZUPT)。

### 阅读
- `docs-cn/math_foundations.md` §6 FEJ 小节
- OpenVINS 官方 `docs/fej.dox` (英文)
- 论文 Huang et al. 2010 "First-Estimates Jacobian EKF"
- `ov_msckf/src/state/StateHelper.cpp::EKFPropagation` 中 `_options.do_fej` 分支
- `ov_msckf/src/update/UpdaterZeroVelocity.cpp` (ZUPT, 静止检测)

### 动手实验
1. 自己用 `Simulator` 生成一段纯直线运动 (改 Bspline 关键点), 观察 scale 估计的协方差增长。
2. 关闭 FEJ (`do_fej=false`) 跑 EuRoC V1_01, 看 yaw 的长期漂移是否变大。
3. 让 VIO 在桌面静止 30 秒, 观察 `UpdaterZeroVelocity` 是否触发, bias 是否收敛。

### 标定并行任务 (1 天)
- 休息 / 整理前 6 周的笔记。

---

## W8 — 综合 + 自选方向

### 学习目标
- 能读一篇近年 VIO 论文 (如 OpenVINS 2022 ICRA paper, Schubert 2018 rolling shutter, VINS-Fusion) 并说出它相对 OpenVINS 的增量。
- 定义你接下来想做的 1 件事 (可选方向):
  - **方向 A**: 降低 OpenVINS 单帧延迟 (profiling + 内存优化)
  - **方向 B**: 加一个新观测类型 (如 GPS 作为 update)
  - **方向 C**: 把 OpenVINS 换成 iSAM2 / factor graph backend
  - **方向 D**: 嵌入式移植 (ARM / RK3588)
  - **方向 E**: 多传感器标定自动化 (你的主线的延伸)

### 阅读
- 选 1 篇论文 + 它的官方代码 (如果有)
- 回顾 PR#1/#2/#3 的全部文档, 看哪里注释还不够细, 记录下来

### 动手产出
- 写一份 2~3 页的报告: "我理解的 OpenVINS + 我选择的方向 + 下一步计划"
- (可选) 在代码里做一个最小可跑 PoC, 证明方向可行

---

## 各周 Checklist (打钩形式, 自我评估)

### 全程通用
- [ ] 每周至少把 `docs-cn/` 对应章节读一遍
- [ ] 每周至少写一次打印/assert 加到代码里, 自己跑一次数据集看现象
- [ ] 每周总结 1 页笔记 (markdown / 手写都行)
- [ ] 卡住 >2 小时就问 (问我, 或官方 issue)

### W1 目标
- [ ] 能写出 JPL 四元数与 Hamilton 的相互转换公式
- [ ] 能说明误差状态 EKF 相对直接 EKF 的好处
- [ ] 跑通 `run_serial_msckf_ros_free` 至少 1 次

### W2 目标
- [ ] 手画 `_Cov` 矩阵图, 与 `docs-cn/diagrams/07_state_structure` 对照检查
- [ ] 能指出任意 `type->id()` 对应 `_Cov` 哪一行哪一列
- [ ] 能解释 KLT 金字塔层数为什么默认 5

### W3 目标
- [ ] 手推 `q̇ = 0.5Ω(w)q` 到离散化 `q_{k+1} = exp(0.5Ω(w)dt) q_k`
- [ ] 能说明 `Propagator::compute_Phi_Qd_analytic` 为什么比数值近似稳
- [ ] `Phi.norm` 和 `Qd.trace` 的典型量级心中有数

### W4 目标 (卡住就 SOS)
- [ ] 手写一个 3×2 零空间 + QR 压缩的 toy example
- [ ] 能在 `UpdaterHelper.cpp` 里精确定位 H_f / H_x / 投影 / 压缩每一步
- [ ] 能解释卡方门限为什么是 `r^T S^{-1} r`

### W5 目标
- [ ] 区分 SLAM / MSCKF 特征, 列出两者各自的生命周期
- [ ] 能跑 `calib_cam_extrinsics=1` 的仿真 + ATE 收敛

### W6 目标 (和标定工作对接)
- [ ] 一份 ATE 对比表: 标定前 vs 标定后
- [ ] 一份外参收敛曲线图 (在线估计 vs 离线 kalibr)
- [ ] timeoffset 估计 ±5ms 吻合

### W7 目标
- [ ] FEJ on/off 对 yaw 长期漂移的定量对比
- [ ] 至少跑 1 次 ZUPT 成功 (bias 收敛)

### W8 目标
- [ ] 论文 1 篇 + 报告 2-3 页
- [ ] 方向选定 + PoC 落地

---

## 卡住了怎么办

### 第 1 步: 翻 docs-cn
90% 的公式和代码位置已经在 `docs-cn/` 里了。用关键词 grep 就能找到。

### 第 2 步: 打印 + 断点
- `PRINT_INFO` / `PRINT_DEBUG` 加一行, `ulimit -c unlimited && run ...` 取 core 看堆栈。
- 或用 gdb: `gdb --args run_serial_msckf_ros_free ...`

### 第 3 步: 仿真器降噪
`ov_msckf/src/run_simulation.cpp` 用的是 `Simulator` (真值已知), 比真数据集调试容易 10 倍。

### 第 4 步: 问我
- 直接在 PR 评论, 或单独发消息, 附:
  1. 你读到哪一行
  2. 你觉得它在做什么
  3. 实际现象
- 我会加注释或画一张小图回答。

---

## 最后一个不能跳的提醒

**2 个月内做到"精通" 95% 是不现实的, 但做到"看懂 + 改对地方" 是完全可能的**。
重点是**不要试图记住所有代码**, 而是**建立好 "遇到问题 → 查文档 → 定位代码 → 改动 → 验证"** 这条闭环。

OpenVINS 代码 3.6 万行, 核心滤波不到 5000 行, 剩下都是基础设施。8 周把核心 5000 行读通, 就够了。
