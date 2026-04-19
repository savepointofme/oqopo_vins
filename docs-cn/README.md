# OpenVINS 中文架构与源码注释

> 本目录 (`docs-cn/`) 是对 [OpenVINS](https://github.com/rpng/open_vins) 项目的中文注释版文档。
>
> 包含：
> - **整体架构思维导图**、**数据流图**、**核心流程图**（Mermaid 源码 + 渲染后的 PNG）
> - 对 `VioManager / Propagator / UpdaterMSCKF / TrackKLT / InertialInitializer` 五个核心文件的中文注释
> - 每个子模块的职责、依赖、关键函数一览

如果你是第一次接触 VIO/MSCKF，建议按 **01 → 05** 顺序阅读；如果你只想快速定位某个功能在代码里的位置，可以直接跳到 [模块索引](#模块索引)。

---

## 目录

| 文档 | 内容 | 对应图 |
| --- | --- | --- |
| [architecture.md](./architecture.md) | 01. 整体架构 & 思维导图 & 系统数据流 | `01_architecture_mindmap.png` / `02_system_dataflow.png` |
| [vio_manager.md](./vio_manager.md) | 02. `VioManager` 主循环、状态结构 | `03_vio_manager_flow.png` / `07_state_structure.png` |
| [tracking.md](./tracking.md) | 03. 视觉前端（KLT 跟踪 / 描述子 / Aruco） | `04_track_klt_flow.png` |
| [initialization.md](./initialization.md) | 04. 初始化（静态 vs 动态） | `05_initialization_flow.png` |
| [updater.md](./updater.md) | 05. 更新器（MSCKF / SLAM / ZUPT） | `06_updater_flow.png` |

Mermaid 源码位于 [`diagrams/*.mmd`](./diagrams)；PNG 位于 [`diagrams/*.png`](./diagrams)。
若需要重新渲染，执行：

```bash
cd docs-cn/diagrams
for f in *.mmd; do
  mmdc -i "$f" -o "${f%.mmd}.png" -p puppeteer.json -w 1800 -H 1400 --backgroundColor white
done
```

---

## 一张图看懂 OpenVINS

![架构思维导图](./diagrams/01_architecture_mindmap.png)

![系统数据流](./diagrams/02_system_dataflow.png)

> **一句话：** `ov_core` 做前端视觉 + 通用数学；`ov_init` 负责启动时的初值；`ov_msckf` 是 EKF 滤波主体（预测—克隆—更新—边缘化）；`ov_eval` 是离线评估工具。

---

## 模块索引

下面列出的是本次注释重点覆盖的文件：

| 模块 | 文件 | 关键类 / 函数 |
| --- | --- | --- |
| 滤波总调度 | [`ov_msckf/src/core/VioManager.{h,cpp}`](../ov_msckf/src/core/VioManager.cpp) | `VioManager::feed_measurement_imu / feed_measurement_camera / track_image_and_update / do_feature_propagate_update / try_to_initialize` |
| IMU 传播 | [`ov_msckf/src/state/Propagator.{h,cpp}`](../ov_msckf/src/state/Propagator.cpp) | `Propagator::feed_imu / propagate_and_clone / predict_and_compute / select_imu_readings` |
| MSCKF 更新 | [`ov_msckf/src/update/UpdaterMSCKF.{h,cpp}`](../ov_msckf/src/update/UpdaterMSCKF.cpp) | `UpdaterMSCKF::update` + `UpdaterHelper::get_feature_jacobian_full / nullspace_project / measurement_compress` |
| 视觉前端 | [`ov_core/src/track/TrackKLT.{h,cpp}`](../ov_core/src/track/TrackKLT.cpp) | `TrackKLT::feed_new_camera / feed_monocular / feed_stereo / perform_detection_monocular / perform_matching` |
| 初始化 | [`ov_init/src/init/InertialInitializer.{h,cpp}`](../ov_init/src/init/InertialInitializer.cpp) | `InertialInitializer::feed_imu / initialize`（调用 `StaticInitializer` 或 `DynamicInitializer`） |

> 注释风格：在文件头加**总述块**；关键函数前加**算法块注释**；每个"步骤"级别的代码加**行内中文备注**。尽量不改动代码逻辑，只加注释。

---

## 补充阅读

- 官方文档：<https://docs.openvins.com>
- 论文：Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA 2020
- MSCKF 原始论文：Mourikis & Roumeliotis, *A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation*, ICRA 2007

---

> 中文文档由 [Devin](https://devin.ai) 自动生成并由 @gimmijimi 审阅。如与源码不一致，以源码为准。
