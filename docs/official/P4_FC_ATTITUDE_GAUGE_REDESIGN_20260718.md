# P4 FC 姿态约束根因与重构合同

## 结论

`explicit_landmark_reprojection_raw_cov_v12` 已证明显式多视角 landmark 重投影能改善三条 focus，但全程仍失败。三条候选的全程终点沿航迹误差分别为 -218.70 m、-433.55 m、-781.95 m；冻结 global baseline 为 -6.61 m、-113.65 m、150.61 m。中后段 bg/ba 已与冻结基准接近，因此不能再把问题归因于 bias 持续发散。

当前 `Factor_P4FcTrajectory` 同时约束 terminal FC 姿态和全部相邻 FC 姿态增量。该语义会把 FC-board 姿态时间残差与魔术贴瞬态 flex 作为窗口内真实相对姿态，进而与 IMU 预积分和视觉重投影争夺 q/bg/ba。

OpenVINS 官方 `DynamicInitializer` 的对应做法不同：首状态使用 `Factor_GenericPrior` 的 `quat_yaw` 只固定不可观 yaw；roll/pitch 和窗口内相对姿态由 IMU+视觉确定。P4 应保留 FC 的全局 yaw gauge 和 metric p/v，不应复制 FC 的相对姿态轨迹。

## 重构合同

1. FC 姿态只形成一个 terminal `quat_yaw` gauge residual。
2. FC position/velocity 继续形成一个密集相关轨迹因子：terminal absolute p/v 加 chronological p/v increments。
3. 移除全部 FC attitude increment residual；q 的相对运动只由标准 15 维 CPI 和多视角重投影决定。
4. 每关键帧 q/p/v/bg/ba、显式 landmark、固定 camera calibration、因果 holdout 和 terminal Schur marginal handoff 保持不变。
5. 不改变 P5、GPS-Z、KLT、噪声门限或评价对齐方式。

## 验收

- 数学测试验证 p/v 相关 covariance 正定、终端 covariance 保持、关键帧密度不改变线性误差路径 likelihood，以及所有 p/v Jacobian 与有限差分一致。
- initializer 测试要求 FC family 恰有两个 residual block，维数为 `1 + 6*N`，并验证 `terminal_yaw_gauge_plus_dense_pv_terminal_and_correlated_increments`。
- fly1/fly2/fly3 先跑中程 P4-only、fixed stride12；只有三条相对 v12 或冻结基准呈一致改善，才运行 full。
- full 未优于冻结 global baseline 时，P4 仍判失败，禁止进入 P5。
