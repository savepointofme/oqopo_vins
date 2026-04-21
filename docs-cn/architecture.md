# 01. OpenVINS 整体架构

> 本章对应图：
> - `diagrams/01_architecture_mindmap.png` — 思维导图
> - `diagrams/02_system_dataflow.png` — 系统数据流图

## 1.1 代码仓库结构

```
open_vins/
├── ov_core/     # 前端与通用库：相机模型、特征跟踪、数据结构、仿真、工具
├── ov_init/     # 初始化：静态对齐 + 动态 SfM/Ceres 优化
├── ov_msckf/    # EKF 滤波主体：状态管理、IMU 传播、MSCKF/SLAM/ZUPT 更新、ROS 接口
├── ov_eval/     # 离线评估：对齐、误差计算、绘图
├── ov_data/     # 小样本数据 & 配置文件
├── config/      # 针对 EuRoC、TUM、KAIST 等公开数据集的 YAML 配置
└── docs-cn/     # (本目录) 中文注释文档
```

## 1.2 思维导图

![思维导图](./diagrams/01_architecture_mindmap.png)

## 1.3 各子模块职责

### `ov_core`：前端 & 通用库

| 子目录 | 负责 | 关键文件 |
| --- | --- | --- |
| `cam/` | 相机投影/反投影模型 | `CamBase`, `CamRadtan` (径向切向), `CamEqui` (等距鱼眼) |
| `track/` | 特征跟踪 | `TrackBase` (抽象), `TrackKLT` (KLT 光流), `TrackDescriptor` (ORB/BRISK 描述子), `TrackAruco` (Aruco 码), `TrackSIM` (仿真) |
| `feat/` | 特征容器 | `Feature` (一个路标的多帧观测), `FeatureDatabase` (数据库), `FeatureInitializer` (三角化 + GN 优化) |
| `types/` | 状态类型 | `Type`, `Vec`, `Quat`, `PoseJPL`, `IMU`, `Landmark` |
| `sim/` | 仿真器 | `Simulator` (生成 IMU + 视觉观测) |
| `utils/` | 工具 | `sensor_data` (数据结构), `quat_ops`, `print`, `colors` |

### `ov_init`：初始化

- **`InertialInitializer`**：入口，按需调用静态 / 动态初始化器。
- **`StaticInitializer`**：假设系统静止，把 IMU 加速度均值当重力，对齐 `z` 轴。仅需陀螺/加计均值即可给出 `q_GtoI`, `bg`, `ba`。
- **`DynamicInitializer`**：假设有足够激励（被晃动），用一段窗口内的视觉 + IMU：
  1. 线性求解 `R_CtoI` 和重力方向（Dong-Si 2012）
  2. 用 Ceres 非线性优化 refine 位姿、速度、重力、特征、外参
  3. 从优化得到协方差
- **`ceres/`**：Ceres 的 factor（IMU 预积分、相机重投影、先验）。

### `ov_msckf`：滤波主体

| 子目录 | 主要类 |
| --- | --- |
| `core/` | `VioManager` (总调度)，`VioManagerOptions` (配置) |
| `state/` | `State` (IMU + 克隆 + SLAM)，`Propagator` (IMU 预测)，`StateHelper` (EKFUpdate / 边缘化 / 增广) |
| `update/` | `UpdaterMSCKF` (短轨 nullspace)，`UpdaterSLAM` (长轨 + delayed init)，`UpdaterZeroVelocity` (静止检测)，`UpdaterHelper` (Jacobian 构建 / QR 压缩) |
| `ros/` | `ROS1Visualizer` / `ROS2Visualizer` (订阅话题、发布轨迹、TF、点云) |
| `sim/` | `SimVisualizer`, 仿真下的联合测试 |

### `ov_eval`：评估工具

- **`calc/`**：ATE、RPE、NEES 等误差指标
- **`alignment/`**：SE(3) / Sim(3) 轨迹对齐（Horn, Umeyama）
- **`utils/`**：加载 EuRoC, TUM 等 groundtruth 文件
- 带 Python 脚本批量绘图。

## 1.4 数据流

![数据流](./diagrams/02_system_dataflow.png)

三条关键数据通路：

1. **IMU → Propagator & Initializer**
   ROS 订阅回调收到 IMU 消息后，`VioManager::feed_measurement_imu` 把它同时交给 `Propagator`（用于未来预测）和 `InertialInitializer`（初始化前）。

2. **Camera → TrackBase → FeatureDatabase**
   图像进入前端 `TrackKLT/TrackDescriptor/TrackAruco`，跟踪结果（每个路标在各帧的 uv 观测）都塞进 `FeatureDatabase`。`FeatureDatabase` 是前端与滤波器的**唯一共享数据源**。

3. **FeatureDatabase → UpdaterMSCKF/UpdaterSLAM → State**
   每帧触发 `do_feature_propagate_update`：先预测 + 增广一个新克隆位姿，再从 `FeatureDatabase` 中挑选"可用于更新的特征"（已经跟丢的、即将被边缘化的、长轨变 SLAM 的），扔给更新器。更新器完成 EKF 后，`StateHelper::marginalize_old_clone` 丢掉最老克隆维持滑窗大小。

## 1.5 状态向量一览

```
State = [ IMU  |  Clones  |  SLAM features  |  Calibration  ]

IMU          : q_GtoI (4), p_IinG (3), v_IinG (3), bg (3), ba (3)         共 15 维（误差 15 维）
Clones       : 每个克隆 = q_GtoI_k (4), p_IinG_k (3)                     每个 6 维（误差 6 维）
SLAM         : 每个路标 = 3 维（全局 XYZ 或单特征逆深度等表示）
Calibration  : t_CAMtoIMU (1), 每相机 R/p_IMUtoCAM (6) + intrinsics (8),
               IMU intrinsics: Dw/Da/Tg + q_GYROtoIMU/q_ACCtoIMU (可选)
```

详见 `diagrams/07_state_structure.png`：

![状态结构](./diagrams/07_state_structure.png)

> `_Cov` 是整块协方差矩阵；`Type` 对象的 `id()` 告诉其自己在协方差中的起止行。`StateHelper` 负责增广/边缘化时的协方差块交换。

## 1.6 典型运行时间线（单目）

```
t0          IMU  --> propagator.feed_imu     (积累 IMU)
                  --> initializer.feed_imu
...
t_K         Cam  --> trackFEATS.feed_new_camera  (跟踪)
                  --> try_to_initialize          (积累窗口)
...
t_init      初始化成功 -> 构建 State, 设 is_initialized_vio = true
...
t_N         Cam  --> track + ZUPT 检测 + do_feature_propagate_update
                  --> propagate_and_clone + MSCKF/SLAM EKF + marg_old_clone
                  --> 发布 Odom/点云
```

---

下一章：[VioManager 主流程](./vio_manager.md)
