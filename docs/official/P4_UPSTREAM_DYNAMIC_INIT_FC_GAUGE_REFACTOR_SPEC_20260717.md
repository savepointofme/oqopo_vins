# P4 上游动态初始化与低维 FC Gauge 对齐重构规格

## 结论

正式 P4 不再在每个推进窗口重建 FC/IMU/视觉/landmark 联合 Ceres 图。新的在线路径由两个解耦阶段组成：

1. 直接使用仓库内与上游 `rpng/open_vins` 同源的 `ov_init::DynamicInitializer`，只运行一次视觉—IMU动态初始化，恢复局部重力系中的 q/p/v/bg/ba 和 15×15 terminal covariance；
2. OpenVINS 正常运行后，在固定传感器时间窗内收集局部 VIO 状态与同一时刻 FC 状态，鲁棒估计唯一的全局 yaw gauge 和 translation，随后对完整活动状态执行一次原子 gauge reset。

上游参考固定为：

- `rpng/open_vins@69488123ed9362dd44b6f28e7f4680abbff1442b`；
- `HKUST-Aerial-Robotics/VINS-Mono@90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`；
- `zju3dv/vig-init@e53d440fc15c3572e7819d407a53c835f16d0551`。

其中 OpenVINS 为实际集成内核；VINS-Mono 与 vig-init 用于核对成熟初始化的分阶段结构。vig-init 的垂直边缘重力假设不适合俯视固定翼场景，不进入正式路径。

## 状态与输入合同

动态初始化只使用锁定 Camera–IMU 标定、KLT FeatureDatabase 和 board IMU。FC、GPS、AGL 和评价真值不得进入动态初始化。

低维对齐只使用：

- 已初始化 OpenVINS 在相机时刻的 q/p/v 与当前 15×15 covariance；
- 因果插值的 FC q/p/v；
- 已接受的固定 `R_FtoI_declared`、FC attitude/navigation 时间偏移和 lever arm；
- board IMU 与 FC angular-rate 的窗内一致性诊断。

GPS course、最终轨迹误差、未来样本和事后 SE(3) 对齐不得进入在线决策。

## 时间窗语义

窗口由持续时间定义，不由固定更新次数定义。窗内实际观测数由传感器时间戳自然决定。

每个相机时刻产生至多一个配对：

```text
local VIO q/p/v at t_camera
<-> FC attitude at t_camera + dt_CI - dt_attitude
<-> FC navigation at t_camera + dt_CI - dt_navigation
```

只有配对时间连续覆盖配置的 gauge window duration，且 FC/IMU 最大间隔满足数据合同，才允许释放。

## 低维求解

对每个配对计算由固定 mounting 给出的目标 IMU 姿态、位置和速度。窗口内只估计：

```text
yaw_GW
translation_GW
```

yaw 使用圆周鲁棒中心；translation 使用各配对 `p_target - Rz(yaw) p_vio` 的逐分量鲁棒中心。速度只参与一致性检验。

VIO 已由 IMU 恢复米制尺度，因此：

- 可计算 FC/VIO horizontal velocity scale ratio 作为健康诊断；
- ratio 偏离 1 时只能等待或拒绝；
- 正式 reset 的 scale 固定为 1；
- 禁止用 FC、GPS-Z 或 AGL 对完整 VIO 状态做尺度缩放。

## 安装角语义

`R_FtoI_declared` 来自已接受 Kalibr/full-flight 结果，并作为固定量使用。在线 FC/board angular-rate residual 只用于识别时间错配、异常窗口和 Velcro flex；不得把单个转弯瞬态更新成永久安装角。

## 原子释放

释放调用现有 `StateHelper::apply_global_yaw_scale_translation_reset()`，其中 scale 强制为 1。reset 必须同时变换 nominal、FEJ、全部 clones、SLAM landmarks 和 covariance；bg/ba 数值保持不变。

## 验收

实现只有同时满足以下条件才可接受：

1. 上游动态初始化测试通过；
2. 新 gauge 单测覆盖 circular yaw、translation、反向转弯、重复时间戳、时间窗覆盖不足和 scale 非 1 拒绝；
3. `run_serial_msckf_ros_free` 与直接测试 target 使用 `-j12` 构建通过；
4. fly1/fly3 半圈短程均产生非空 `traj_nav.txt`，正式 P4 释放且无未来数据；
5. 初始化期间墙钟处理速度接近或快于数据时间，不再出现每 0.5 秒重复大图认证；
6. 同口径绝对导航指标不劣于冻结 global baseline；失败则保留证据并停止进入 P5。
