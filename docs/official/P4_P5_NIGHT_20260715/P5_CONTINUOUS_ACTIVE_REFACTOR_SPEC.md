# P5 连续自适应调度重构规格

## 目标

P5 必须在每个原始相机时刻，用截至当前时刻已经产生的 KLT 几何与估计器健康量，分别决定：

1. 下一次何时运行 KLT；
2. 当前已经完成 KLT 的图像是否真正创建 OpenVINS clone 并进入后端。

正式实现不得再由 `HIGH/NORMAL/LOW/TURN/DEGRADED` 等状态映射到固定 stride，也不得从 `{1,2,4,6,8,12}` 中选档。

## Tracking 合同

tracking 调度使用实际 tracking pair 的真实时间间隔。对 rotation-compensated flow、raw flow、rotation flow 和 track survival 分别换算为时间速率，再直接求连续安全时域：

```text
T_target = target_compensated_parallax / compensated_median_rate
T_safe   = min(T_interval, T_comp_p95, T_raw_p95, T_rotation_p95,
               T_rotation_edge, T_survival)
T_next   = min(T_target, T_safe)
gap      = floor(T_next / measured_raw_camera_dt)
```

`gap` 只在最后一步因为原始图像是离散事件而取整，并可取 `[1,max_gap]` 内的任意整数；不存在候选档位表。危险证据立即缩短 gap，延长 gap 只能在连续时间确认后发生。

视觉证据必须包含 common tracks、survival、track age、有效样本数 `N_eff`、网格覆盖率、覆盖熵、边缘特征比例、rotation-compensated flow 的 robust dispersion 和上置信界。无新 KLT 量测时保持当前计划，但量测过期后强制 dense tracking。

在线 tracking 决策不读取 GPS 未来误差、最终轨迹误差或 GPS horizontal。高度不映射固定档位；只有不晚于当前相机时刻、年龄合格且以地面首样本为原点的 causal GPS ENU-U 才可作为 AGL，结合相机姿态、内参和速度形成连续 footprint-overlap 安全时域。高度无效或四角光线不能形成有效地面 footprint 时，该约束明确标记 unavailable，由 KLT 几何接管，不能伪造 overlap。

## Backend 合同

backend 不输出或追踪第二个 stride。每个已完成 KLT 的 frame 直接与“上一个实际创建 clone 的 frame”比较，计算：

- rotation-compensated direct baseline；
- `N_eff`、coverage、entropy、survival；
- robust flow uncertainty 与 information score；
- pure-rotation 和 track-termination risk；
- 距上一个实际 clone 的真实时间。

触发优先级为：估计器/数据合同故障、track termination、最大时延、有效平移信息充分、继续积累。IMU 角速度或转弯标签本身不得强制 backend；pure rotation 在未达到最大时延且无终止风险时继续积累。

只有 OpenVINS clone map 中确实出现当前 timestamp 后，才能提交新的 backend reference。ZUPT、乱序返回或其他没有 clone 的路径必须保留旧 reference，并记录 `trigger_without_clone`。

## 删除项

- 删除 `AdaptivePolicyState` 及六状态机；
- 删除状态到 tracking/backend 固定目标表；
- 删除 `allowed_strides={1,2,4,6,8,12}`；
- 删除 runner 中对 `{1,2,4,6,8,12}` 的 overlap 枚举；
- 删除 `backend_update_stride` 作为实际调度变量；
- 删除“turn/low/descent 状态边沿强制 backend”；
- 删除 trigger 成立时提前提交 backend reference 的行为。

## 验收

代码验收：

- 生产调用链不再引用旧 controller、旧状态 enum 或固定 stride 表；
- 单测覆盖任意整数 gap、实际时间缩放、立即降频/延时升频、stale、coverage/`N_eff`、pure rotation、信息触发、时延触发和实际 clone 才提交；
- `-j12` 重建 runner 和新测试通过。

运行验收：

- fly1、fly3 必须用 `--adaptive-stride` active 接管实际 KLT/backend；
- 与同源码 fixed stride 12 对照，记录 raw/KLT/实际 clone/MSCKF/SLAM 计数和真实运行时间；
- `traj_nav.txt` 非空，P4 释放合同未被破坏；
- 正式全程评价使用官方绝对导航口径和共同四边段；
- shadow、旧 21 项单测和离线 fixed-stride sweep 均不能代替 active 验收。
