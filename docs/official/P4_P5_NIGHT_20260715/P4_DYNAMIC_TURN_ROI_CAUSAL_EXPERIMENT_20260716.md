# P4 航向首发偏移：动态转向 ROI 因果实验

## 结论

动态转向 ROI 已按“左转优先右侧、右转优先左侧”完整接入前端补点和
MSCKF/SLAM 配额，并通过同一二进制、同一数据窗的 control 对照。它确实
改变了尺度/高度行为，但没有恢复航向；在 fly1、fly3 的正式指标上均退化，
因此当前实现不能作为修复进入默认配置。

这次负结果不是“ROI 没有作用”：

- fly1 首转末端高度误差由 `-17.48 m` 变为 `+1.54 m`；
- fly3 全段末端高度误差由 `-18.91 m` 变为 `-2.79 m`；
- 但 fly1 航向末值由 `2.32°` 变为 `3.88°`，fly3 由
  `1.27°` 变为 `1.61°`。

所以外侧图像区域与场景尺度/垂直几何有明显耦合，但“外侧点更远”不能直接
推出“外侧点的航向信息更可靠”。ROI 只能继续作为受控消融，不能冒充航向
修复。

## 实现合同

动态 ROI 不使用 GPS course、GPS horizontal、最终误差或未来数据。转向信号
来自 bias-corrected board IMU 角速度在全局竖直轴上的投影：

- 投影为负：当前数据约定下为左转，优先右侧图像；
- 投影为正：当前数据约定下为右转，优先左侧图像；
- 转向强度连续控制补点遮罩比例和后端 preferred-side 配额；
- 已存在的 KLT 轨迹不因 ROI 被硬删除；
- opposite side 始终保留，满强度时后端配额为 32/8，而不是半图清空。

主要实现：

- `ov_msckf/src/core/DynamicTurnRoiPolicy.h`
- `ov_core/src/track/TrackKLT.cpp`
- `ov_msckf/src/core/VioManager.cpp`
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`
- `ov_msckf/src/test_dynamic_turn_roi.cpp`

构建与测试：

```text
cmake --build build_p4_sliding_r1 \
  --target test_online_alignment_initializer test_adaptive_stride \
           test_dynamic_turn_roi run_serial_msckf_ros_free -j12

online alignment initializer tests passed
continuous adaptive visual scheduling tests 1-15 passed
dynamic turn ROI policy tests passed
```

## 同口径运行

动态 ROI：

```text
C:\Users\baloney\Desktop\实验目录\P4_yaw_drift_causal_ablation_20260715\
20260716_152714_screen_visual_roi_dynamic_turn_both
```

fresh same-binary control：

```text
C:\Users\baloney\Desktop\实验目录\P4_yaw_drift_causal_ablation_20260715\
20260716_153115_screen_control_both
```

四条回放均 exit 0，fly1/fly3 frame contract 均为 0；轨迹非空。动态策略在
指定首转中的主要方向与偏好如下：

| 飞行 | 首转实际方向 | 首选图像侧 | 主方向帧 |
| --- | --- | --- | ---: |
| fly1 | LEFT | RIGHT | 73 |
| fly3 | RIGHT | LEFT | 64 |

少量相反方向行来自同一几何转弯内真实角速度反向/回摆，不是 GPS course
驱动。

## 评价器正确性修复

首次正式评价暴露了三项工具错误；修复前产生的约 400 m 初始误差和 N/A
dashboard 全部作废：

1. fly1 GPS CSV 有 34 组内容完全相同的重复时间戳，位置差分把零时间间隔
   送入 `np.gradient`，产生 NaN 并污染整个航向旋转矩阵；
2. `traj_nav.txt` 是 `t p(3) v(3) q(4)` 十一列，旧读取器把三列速度和
   一个四元数分量误当成完整四元数；
3. run spec 从裁剪起点开始，但 P4 约十秒后才输出首状态；旧工具把裁剪起点
   GPS 与未来首个 VIO 状态强行作为同一起点，凭空加入初始化期间飞行距离，
   并把无 VIO 数据的前段送入分段。

修复位置：

- `analysis/flight_eval/io.py`
- `analysis/flight_eval/trajectory.py`
- `analysis/flight_eval_tool.py`
- `analysis/full_flight_error_analysis.py`

回归检查确认 fly1 GPS 时间网格唯一、差分速度全有限、十一列轨迹解析后的
四元数范数为 `0.9999999997`。修复后的四个 dashboard 都包含
`window.RUN_DATA`。

## 正式全段指标

主口径为 GPS update grid、真实 GPS/VIO overlap、start-heading alignment。

| 飞行 | 条件 | 距离 km | 终点 XY m | 漂移率 | XY RMSE m | course RMSE | course 末值 | 末端 Z m |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fly1 | control | 7.766 | 100.65 | 1.296% | 67.86 | 2.09° | 2.32° | +1.12 |
| fly1 | dynamic ROI | 7.766 | 156.55 | 2.016% | 112.97 | 2.78° | 3.88° | +0.99 |
| fly3 | control | 11.853 | 91.39 | 0.771% | 77.94 | 1.55° | 1.27° | -18.91 |
| fly3 | dynamic ROI | 11.853 | 103.18 | 0.871% | 81.54 | 1.65° | 1.61° | -2.79 |

## 用户指定首发窗口

### fly1：首转后第一条直线

| 窗口 | control course 起→止 | dynamic course 起→止 | dynamic 相对 control |
| --- | --- | --- | --- |
| 首转 940.553–983.753 | -0.21° → +0.26° | +0.57° → +1.00° | 首转末多 0.73° |
| 直线前半 983.753–1036.85 | +0.10° → +1.95° | +0.45° → +2.83° | 末端多 0.89° |
| 直线后半 1036.85–1089.75 | +1.94° → +2.70° | +2.80° → +3.49° | 末端多 0.79° |

control 在首转末只有 0.26° course 误差，明显偏移主要在后续直线持续增长；
因此 fly1 不是单一“转弯瞬间打歪后保持”的模型。动态 ROI 在首转引入更大
初始偏差，后续直线仍继续增长。

### fly3：首转后到第二转前直线后半

| 窗口 | control course 起→止 | dynamic course 起→止 | dynamic 相对 control |
| --- | --- | --- | --- |
| 首转 693.2–740.1 | +0.04° → -0.63° | +0.04° → -0.34° | 首转末少 0.28° |
| 直线前半 740.1–818.6 | -0.71° → +1.73° | -0.42° → +1.94° | 末端多 0.21° |
| 直线后半 818.6–897.2 | +1.81° → +1.04° | +2.02° → +1.18° | 末端多 0.14° |

fly3 首转内存在大幅瞬态但末端能恢复到 1°以内；用户指出的直线后半偏移
不是持续单调发散，course 误差先升到约 2–3°，到第二转前又有所回落。
dynamic ROI 只带来 0.1–0.2°量级差异，不能解释或修复该现象。

## 因果结论与下一步

1. ROI 方向规则已被真正执行，不能再把未实现的“左转取右、右转取左”
   当作待验证假设。
2. 该规则强烈影响高度/尺度，却同时恶化航向，说明尺度和 yaw 在视觉更新中
   被耦合分摊；需要把尺度恢复单独实现，不能继续靠半图选择间接补偿。
3. fly1 的主要 course 漂移发生在转后直线，fly3 也有转后直线演化；根因不能
   简化成某个转弯方向的固定角补偿。
4. 下一项实验是 AGL 驱动的事务式场景 Sim(3) 尺度重置：仅用 causal altitude，
   不把高度送入 `p_z` Kalman update，也不使用 GPS yaw/horizontal。只有它能
   区分“尺度耦合诱发的表观 yaw”与“真正的 gyro/视觉 yaw 漂移”。
