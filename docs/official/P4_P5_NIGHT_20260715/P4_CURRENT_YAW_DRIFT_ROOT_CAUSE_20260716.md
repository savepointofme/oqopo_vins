# P4/P5 现状航向漂移根因报告

> 历史因果报告：FC-yaw 只作为诊断正控制。当前统一结论及禁止事项见
> `P4_EXPERIMENT_MASTER_RECORD_AND_RULES.md`，不得从本文推导生产 FC/GPS yaw 融合。

> **状态更正（2026-07-16）**：本报告中的 FC-yaw 干预只证明“误差写入后，单目 VIO 缺少绝对 yaw 观测，因而不能自行恢复”这一**偏差保留机制**。它没有证明误差最初由什么写入，也不是允许上线 FC/GPS yaw 的生产修复。文中把该机制称为“主因”的结论已经被后续全量数据分析降级。用户指定的首错区间也应为 fly1 首次左转 `940.553–983.753 s` 及其后直线、fly3 首次右转 `693.2–740.1 s` 后到第二次转弯 `897.2 s` 前的直线，而不是本文后半使用的下一圈区间。后续有效结论和执行计划见 `P4_YAW_DRIFT_CAUSAL_DECOMPOSITION_AND_ACTION_PLAN_20260716.md`。

## 结论

当前 fly1/fly3 的数度级航向偏移，主因不是 P4 初始化时航向没有对齐，也不是 P5 cadence、GPS-Z、单次特征洗牌或某一类 SLAM 更新本身。已经由正反实验支持的主机制是：

> P4 只在释放时提供一次绝对 `G_nav` 航向；释放后 FC 姿态流被关闭，单目 IMU+视觉后端只剩相对旋转约束，全局 yaw 成为没有持续绝对锚的自由模式。IMU、时间/安装残差和视觉更新中的小误差可以沿该模式积累。转弯及其伴随的特征洗牌、SLAM 数量下降和 FC–board 瞬态会放大或显化偏移，但不是偏移能够长期保留的根因。

历史 global baseline 只作为背景参考。它的 fly3 同样存在问题，不能作为正确答案或门限来源；本报告的判断来自当前 control 与当前单变量干预的同条件比较。

## 因果实验合同

control 与 `fc_yaw_aid` 使用同一 binary、配置、P4 8 s 时间窗、fixed stride12、Camera–IMU 标定、FC–board 标定、IMU/图像/GPS-Z 路径。唯一干预发生在 P4 释放之后：每 2 s 使用当时可获得的 FC 姿态构造 yaw observation。

- FC 目标时刻：`t_state + dt_CI - dt_FC_attitude_to_board`；
- FC 姿态只从目标时刻两侧历史/当前行插值，不读取未来行；
- 使用 P4 接受的 `R_FtoI_nominal` 把 `q_GtoF` 转到 IMU 姿态；
- 不使用 GPS course、GPS XY 或最终轨迹误差；
- GPS 仍只在离线正式评价中使用；
- fly1 接受 173/173 次，覆盖 `942.85–1298.63 s`；
- fly3 接受 248/252 次，覆盖 `630.44–1149.47 s`；4 次因原始 FC 局部间隔超过合同而跳过，重点区间完整覆盖。

原标定包中的 `fullflight_calibrated.csv` 实际只是一份被重命名的截断初始化流：fly1 到 1075.15 s、fly3 到 1113.20 s。为避免用截断数据误判，本实验用权威大表追溯到原始 FC 文件，重新生成完整流。新旧流在重叠区逐行、逐字段完全一致：fly1 1466 行、fly3 3882 行，q/p/v 和全部字段最大绝对差均为 0。重新运行正式全航程 FC–board 标定后，旋转矩阵与时间偏移也与 v3 完全一致。

## P4 初始化身份

| 飞行 | control / intervention window fingerprint | P4 release heading error | q/p/v/bg/ba 最大差 | 结论 |
| --- | --- | ---: | ---: | --- |
| fly1 | 完全相同 | -0.2502° | q `1.60e-11`；p `3.83e-9 m`；v `4.58e-10 m/s`；bg `2.28e-12 rad/s`；ba 0 | 不是初始化差异 |
| fly3 | 完全相同 | +0.4963° | 全部 0 | 不是初始化差异 |

两条 control 在第一轮后端输出时的姿态 yaw 误差分别约 +0.623° 和 -0.518°，均不足以解释随后 3–5° 的偏移。漂移是在释放后逐步形成的。

## 正式绝对导航结果

正式评价在 GPS update time 采样，使用绝对 `G_nav`，不做事后位置、航向或 SE(3) 对齐。

中间曾调用 `flight_eval_tool.py single` 的默认四边航线分段：fly1 因 focus 片段没有完整一圈而失败；fly3 虽生成结果，但其默认合同与 control 的既有正式目录不一致，得到的 343.9 m 终点 XY 不作为比较证据。该失败/非同口径目录仍保留在 `evaluation_fc_yaw_aid`。下表只采用仓库正式 `full_flight_error_analysis.py`、同一 `COMMON_LAP_SEGMENTS.csv`、绝对导航无后对齐结果。

| 飞行 | 条件 | course 终值 | course RMSE | 终点 XY | XY RMSE | Vxy RMSE |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| fly1 | control | +4.061° | 4.260° | 199.4 m | 156.9 m | 3.094 m/s |
| fly1 | causal FC-yaw | -0.090° | 1.659° | 122.8 m | 69.2 m | 1.394 m/s |
| fly3 | control | +3.675° | 2.374° | 82.9 m | 87.3 m | 1.833 m/s |
| fly3 | causal FC-yaw | +0.709° | 1.728° | 97.3 m | 63.8 m | 1.473 m/s |

fly1 的航向、位置和速度同时显著改善。fly3 的 course、XY RMSE 和速度改善，但终点 XY 增加约 14.4 m；这说明诊断干预验证了 yaw 可观测性机制，却还不是可直接上线的生产观测实现。

## 用户指定两处的逐阶段结果

### fly1：第一次转弯及其后直线

| 阶段 | 时间 | control course 中位数 / 斜率 | FC-yaw 中位数 / 斜率 | Vxy 误差中位数 control → FC-yaw |
| --- | --- | --- | --- | ---: |
| 转弯前 | 1059.95–1089.95 | 3.77° / +0.0227°/s | 1.68° / +0.0119°/s | 2.34 → 1.05 m/s |
| 第一次转弯 | 1089.95–1150.95 | 4.32° / +0.0080°/s | 1.78° / -0.0028°/s | 2.79 → 1.22 m/s |
| 出弯直线前半 | 1150.95–1194.85 | 4.28° / +0.0009°/s | 1.14° / -0.0134°/s | 3.18 → 1.28 m/s |
| 出弯直线后半 | 1194.85–1238.75 | 4.64° / -0.0270°/s | 1.13° / -0.0390°/s | 3.52 → 1.30 m/s |

关键点是：明显偏移在转弯前已经存在，转弯使轨迹几何上的偏差更容易看见，但不是从零制造了全部误差。绝对 yaw observation 存在时，同一转弯后没有保留 4–5° 的全局航向偏置。

### fly3：第一次转弯到第二次转弯前的直线后半

| 阶段 | 时间 | control course 中位数 / 斜率 | FC-yaw 中位数 / 斜率 | Vxy 误差中位数 control → FC-yaw |
| --- | --- | --- | --- | ---: |
| 转弯前 | 867.0–897.0 | 2.52° / -0.0057°/s | 0.92° / -0.0076°/s | 1.55 → 0.63 m/s |
| 第一次转弯 | 897.0–946.6 | -0.17° / +0.0569°/s | -2.14° / +0.0442°/s | 1.90 → 2.52 m/s |
| 出弯直线前半 | 946.6–1010.9 | 2.50° / +0.0110°/s | 0.21° / +0.0082°/s | 2.16 → 1.12 m/s |
| 出弯直线后半 | 1010.9–1075.2 | 2.85° / +0.0060°/s | 0.52° / +0.0054°/s | 2.34 → 0.86 m/s |

转弯中 FC 机头航向与 GPS 航迹角会受 bank、侧滑和风影响，不能要求二者瞬时完全相等；真正有判别力的是出弯后的稳定直线。该段 control 保留约 2.5–2.9° 偏置，而 FC-yaw 条件回到约 0.2–0.5°。

## “根因”与“触发因素”分离

### 已证实的根因机制

1. P4 释放后没有持续绝对 yaw observation。
2. 单目视觉 reprojection 只约束相对几何；对整条状态和地图施加共同全局 yaw 旋转时，内部像素残差可以几乎不变。
3. 因此视觉更新能压制无约束 IMU 积分发散，却不能把全局 yaw 拉回 `G_nav`。
4. 因果 FC-yaw 干预在 P4 完全相同的前提下消除了大部分长期 course 偏置，构成正向干预证据。

### 会触发或放大误差、但不是共同根因的现象

- fly1 约 1126 s、fly3 约 897 s 出现明显 feature churn 和特征数骤降；
- 第一次转弯时 SLAM feature 数下降、MSCKF/SLAM residual 峰值和单侧特征分布变化；
- FC–board 相对转动残差分别在约 1116 s 和 901 s 出现峰值；
- 高 bank、gyro norm 和光流幅度在转弯中同时上升。

这些物理/视觉事件在 control 和 FC-yaw 两组中几乎相同。干预组仍经历同样事件，却不再留下同等长期全局 yaw 偏置，所以它们只能解释误差被注入或暂时放大的时刻，不能解释为什么偏置之后无法恢复。

视觉 residual 也没有随着全局航向改善而统一下降：这不是反证，而是 global-yaw gauge 的直接表现。用 reprojection RMS 作为全局 yaw 正确性的验收量在数学上不充分。

## 已排除或降级的假设

- P4 初始 heading：释放误差小，干预前后初始化相同；排除为主因。
- P4 bg/ba 分级反馈：保留 prior 的消融没有消除慢漂；排除为共同主因。
- global-yaw OC/FEJ 选择：切换 FEJ 不修复 fly1并恶化 fly3；排除。
- GPS-Z：关闭或改变 Z 更新不消除共同漂移；排除。
- visual-to-bgz：冻结该通道没有消除漂移；排除。
- SLAM 单独作用：关闭 SLAM 使 fly3 位置灾难性退化，不能解释为“SLAM 导致漂移”；SLAM 是必要稳定器。
- Camera–IMU 小角度扰动：对 course 水平值有系统影响，但基本不改变积累斜率，且内部视觉残差不支持该扰动；降级为次要耦合/补偿杠杆，不接受为根因。
- 固定 gyro-z 偏置和离线 gyro intrinsic：正负扰动及跨架次校准不能同时解释两架次；降级。
- P5 adaptive stride：full-rate/adaptive 会改变响应，但不能消除两种 cadence 都共有的全局 yaw 自由模式；不是共同根因。

## 当前代码状态与下一步

当前 `--post-alignment-fc-yaw-aid` 是因果诊断开关，默认关闭。它证明机制，不是正式 P4/P5 功能。直接复用现有 pose-anchor 更新会通过全状态交叉协方差修正 p/v/bg/ba；fly3 终点 XY 的小幅退化说明生产实现必须单独设计和验收：

1. 把“P4 后持续 FC 姿态 yaw observation”定义为明确的新输入合同，而不是伪装成 P4 初始化或 P5 cadence；
2. 使用正式的一维 yaw residual/Jacobian、Joseph covariance update 和 injection reset；
3. 转弯期间按 bank、角速度、FC–board 相对转动残差和姿态时间间隔进行物理降权，避免把侧滑/柔性瞬态固化为全局航向；
4. 直线巡航时恢复绝对 yaw 约束；门限只使用 innovation/NIS/covariance/FC 状态，不使用 GPS 最终误差；
5. 先做 fly1/fly3 focus 正反实验，再做 full flight；必须同时检查 course、XY、Vxy、U、bias、NIS、拒绝原因和 covariance health。

## 证据位置

- 统一大表：`C:\Users\baloney\Desktop\实验目录\P4_yaw_drift_causal_ablation_20260715\CAUSAL_ANALYSIS\P4_YAW_CAUSAL_MASTER.parquet`
- 总 dashboard：同目录 `interactive_causal_ablation_dashboard.html`
- 总因果报告：同目录 `P4_YAW_DRIFT_CAUSAL_ABLATION_REPORT.md`
- fly1 同步审计：`FC_YAW_AID_ANALYSIS\fly1\ROOT_CAUSE_AUDIT.md`
- fly3 同步审计：`FC_YAW_AID_ANALYSIS\fly3\ROOT_CAUSE_AUDIT.md`
- 正式绝对导航评价：`evaluation_focus\fc_yaw_aid\absolute_navigation_no_post_alignment\fly1` 与 `fly3`
- 完整 FC 输入及重建标定：`FULL_FC_INPUTS_RAW`、`FULL_FC_CALIBRATED_V3_REBUILT`

## 验收边界

可以认定“当前数度级航向偏移的主机制已经通过因果实验定位”。不能认定“生产修复已经完成”，也不能认定“P5 已因此通过”。在正式一维 yaw observation、转弯降权和全程 fly1/fly3 验证完成前，诊断开关不得作为默认功能发布。
