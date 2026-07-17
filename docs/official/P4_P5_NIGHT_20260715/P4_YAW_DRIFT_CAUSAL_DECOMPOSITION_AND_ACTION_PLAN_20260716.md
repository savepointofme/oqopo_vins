# P4 航向偏移因果分解、校准审计与修复计划

## 1. 当前结论

现有证据还不能把 fly1/fly3 的航向偏移归结为一个已经证明的单点错误，但已经把问题收敛为四层：

1. **误差注入的第一嫌疑是混合 Camera–IMU 校准与转弯视觉几何的耦合。** 当前 global baseline 并非一套同源标定，而是 fly3 在线终值的相机内参/畸变、June12 的 `T_C_I` 和手工锁零的 Camera–IMU 时间偏移组合。现有旋转扰动在 fly1/fly3 的首错区间给出一致方向的强响应，但还必须用锁定校准的 2×2/因子消融区分“真实外参误差”和“内参—外参相互补偿”。
2. **转弯方向对应的图像侧确实表现出不同的有害更新比例。** fly1 左转时右侧更干净；fly3 右转时左侧更干净，支持“左转优先右、右转优先左”。但静态半图 mask 会破坏 KLT 连续性并触发错误的 lost-track 更新，不能作为实现。
3. **fly3 首转后的图像质量下降是放大器，不是 fly1/fly3 的共同根因。** fly3 的清晰度在首转后持续下降；fly1 没有同样的崩塌却仍漂移。
4. **单目 VIO 的全局 yaw gauge 只能解释偏差为什么保留，不能解释偏差从哪里来。** FC-yaw 诊断能把偏差拉回，并不等于生产系统应该融合 FC/GPS yaw，更不构成根因定位。

因此现在不接受三种捷径：不接受把 FC/GPS yaw 加回在线状态；不接受把 `z_minus` 固定补偿直接发布；不接受把静态左/右半图 mask 当成动态 ROI。

## 2. 数据合同与首错区间更正

本轮分析使用统一大表：

```text
C:\Users\baloney\Desktop\实验目录\P4_yaw_drift_causal_ablation_20260715\CAUSAL_ANALYSIS\P4_YAW_CAUSAL_MASTER.parquet
```

该表为 `44,736 × 494`，汇总 VIO/FC/GPS、姿态、速度、IMU、KLT、MSCKF、SLAM、协方差、视觉 residual、FC–board residual 和逐类更新诊断。本报告只取 `method=control` 的当前 fixed-stride12/P4 路径；GPS 只用于离线 truth/reference，未进入在线估计器。

之前报告把下一圈转弯当成了用户指定的首次误差段。正确区间是：

| 飞行 | 用户指定事件 | 本报告采用时间 |
| --- | --- | ---: |
| fly1 | P4 后首次主要左转 | `940.553–983.753 s` |
| fly1 | 首转后直线前半/后半 | `983.753–1036.85 s` / `1036.85–1089.75 s` |
| fly3 | P4 后首次主要右转 | `693.2–740.1 s` |
| fly3 | 首转到第二转之间直线前半/后半 | `740.1–818.6 s` / `818.6–897.2 s` |

每圈几何分段只保留两个直线和两个转弯；不会再把逐帧状态切换切成数百个“航段”。

## 3. 首次航向误差形成过程

### 3.1 fly1

| 区间 | attitude heading 首→尾 | course 中位数 | XY 首→尾 | Vxy error 中位数 | U error 中位数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 首次左转 | `+0.623° → -1.498°` | `2.195°` | `1.86 → 54.27 m` | `1.89 m/s` | `-17.31 m` |
| 出弯直线前半 | `-1.385° → -2.809°` | `2.522°` | `47.18 → 77.62 m` | `1.58 m/s` | `-0.28 m` |
| 出弯直线后半 | `-2.872° → -3.661°` | `3.533°` | `75.08 → 187.15 m` | `2.12 m/s` | `-0.23 m` |

转弯中出现约 `-18 m` 的高度瞬态，但出弯后 U error 已恢复到接近 0，heading 和 XY 仍继续恶化。因此高度/尺度错误是重要耦合量，却不能单独解释持续 yaw 偏移。

### 3.2 fly3

| 区间 | attitude heading 首→尾 | course 中位数 | XY 首→尾 | Vxy error 中位数 | U error 中位数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 首转前直线 | `-0.016° → -0.864°` | `0.178°` | `7.01 → 34.62 m` | `0.85 m/s` | `-0.28 m` |
| 首次右转 | `+0.836° → -0.368°` | `-1.068°` | `42.83 → 44.55 m` | `1.39 m/s` | `-0.14 m` |
| 出弯直线前半 | `-0.127° → -1.699°` | `1.328°` | `37.63 → 65.36 m` | `0.89 m/s` | `+0.32 m` |
| 出弯直线后半 | `-1.590° → -2.745°` | `2.216°` | `67.08 → 152.92 m` | `1.42 m/s` | `+0.51 m` |

fly3 的关键事实不是“转弯瞬间跳坏”，而是首转结束后，在姿态已回到近水平、第二次转弯尚未开始时，heading 继续从约 `-0.1°` 漂到 `-2.7°`。这排除了“仅仅是 bank 时 course 与机头不一致”的解释。

## 4. 传感器、状态和视觉证据

### 4.1 转弯时确有惯性/机体不一致，但不持续

| 量 | fly1 首转 | fly3 首转 |
| --- | ---: | ---: |
| bank 峰值 | `33.2°` | `33.1°`，方向相反 |
| gyro model residual 中位/P90 | `1.87 / 4.28 °/s` | `1.52 / 3.21 °/s` |
| accel model residual 中位/P90 | `2.42 / 3.88 m/s²` | `2.08 / 3.64 m/s²` |
| track churn 中位/P90 | `0.277 / 0.473` | `0.232 / 0.416` |
| P95 flow 中位 | `41.85 px` | `36.68 px` |
| MSCKF P95 residual 中位 | `1.60 px` | `1.44 px` |
| SLAM P95 residual 中位 | `1.15 px` | `1.12 px` |

FC–board relative-rotation residual 在转弯升高，但出弯后恢复：fly1 约 `0.702° → 0.383° → 0.239°`，fly3 约 `0.520° → 0.210° → 0.204°`。heading 却在恢复后继续漂移。所以该残差可作为误差注入/激励的触发证据，但还不能被解释成持续存在的物理 Velcro flex。

### 4.2 bias 没有给出足以单独解释现象的阶跃

首转期间 `bg_z` 只有约 `10^-4–10^-3 rad/s` 量级变化，既有正负 gyro-z 扰动和跨架次 gyro intrinsic 不能同时解释 fly1/fly3。bias 可能承接了校准与视觉几何误差，但当前证据不支持把一个固定 `bg_z` 常数直接当根因。

### 4.3 fly3 有独立的图像退化

以原始 JPEG 每秒采样的只读统计：

| 飞行/阶段 | 灰度中位数 | Laplacian variance 中位数 |
| --- | ---: | ---: |
| fly1 转弯前 | `120.7` | `318` |
| fly1 转弯 | `119.5` | `430` |
| fly1 出弯后 | `118–121` | `355–389` |
| fly3 转弯前 | `122.9` | `365` |
| fly3 转弯 | `128.3` | `220` |
| fly3 出弯前半 | `130.6` | `104` |
| fly3 出弯后半 | `123.4` | `69` |

fly3 的纹理/清晰度在首转后显著下降，时间上与直线后半漂移一致。没有大面积黑白饱和，RGB 通道比也没有异常跳变。由于 June12 包缺少曝光、增益、白平衡、固件和采集命令 metadata，无法从现有 artifact 证明当时是否开启 AE/AWB；只能把它列为校准数据 provenance 风险。D455 RGB 使用全局快门，但全局快门并不能消除整帧曝光期间的运动模糊；AE 还可能改变曝光时间和实际帧率，这是需要在新采集时显式锁定并记录的变量。

## 5. 校准谱系和现有扰动证据

### 5.1 当前不是同源校准

| 项 | June12 纯标定 | 当前 global baseline |
| --- | --- | --- |
| intrinsics | `[382.9995, 382.2768, 332.5998, 236.5446]` | fly3 在线终值 `[386.750, 387.233, 330.249, 239.916]` |
| distortion | `[-0.060731, 0.042879, -0.003107, 0.001100]` | fly3 在线终值 `[-0.043, 0.035, -0.001, 0.001]` |
| `T_C_I` | June12 Kalibr 结果 | 同一 June12 `T_C_I` |
| Camera–IMU time offset | `+0.15117493 ms` | 手工锁为 `0 ms` |

June12 残差均值约为 gyro `0.003693 rad/s`、accel `0.01351 m/s²`、重投影 `0.21172 px`。旧标定到 June12 的旋转差约 `1.137°`，小角向量约 `[-0.751°, +0.406°, -0.749°]`。

历史全在线标定还显示参数会互相吸收误差：fly1/fly2/fly4 的 focal length 都移动到约 `385–386 px`，time offset 移动到 `-1.2–-5 ms`；fly4 的外参平移甚至从约 `-0.026 m` 走到 `-0.178 m`。后者没有合理物理解释，说明“在线收敛到某个数”不能自动当作真实安装标定。

### 5.2 首错区间的外参旋转灵敏度

相对当前 control 的终点差：

| 条件 | fly1 heading / course / XY | fly3 heading / course / XY |
| --- | ---: | ---: |
| full rotation plus | `+0.110° / +0.645° / +44.5 m` | `+0.475° / +0.302° / +20.1 m` |
| full rotation minus | `+0.020° / -0.771° / -45.1 m` | `-0.321° / -0.426° / -30.9 m` |
| z minus `-0.7518°` | `-0.456° / -0.288° / -16.2 m` | `-0.419° / -0.313° / -17.5 m` |
| z plus | `+0.540°` heading 退化 | `+0.606°` heading 退化 |

这是目前最强的共同方向响应。但“minus”方向大致沿旧标定→June12 的方向继续走，当前已经使用 June12 `T_C_I`；继续走还能改善，反而提示它可能是在补偿 mixed intrinsics/尺度，而不一定是 June12 外参本身错了。必须做锁定校准因子实验，不能把 `-0.7518°` 直接写进生产配置。

## 6. 图像侧别与 ROI 证据

修正旧脚本的两个错误后重新聚合：旧脚本把 master method 写成不存在的 `fixed`，实际 control 名为 `control`；同时 width/height 引用为空，导致全部 x-bin 为 NaN。因此旧 `20260716_EXISTING_DATA_DYNAMIC_ROI_AUDIT` 不能作为有效证据。

按真实 `640×480` 和 control 重新统计：

| 事件 | 图像半区 | 样本占比 | 有害更新比例 | 有害更新 `|δyaw|` | 修正更新 `|δyaw|` |
| --- | --- | ---: | ---: | ---: | ---: |
| fly1 左转 | 左 | `68.0%` | `57.8%` | `7.77°` | `3.62°` |
| fly1 左转 | 右 | `32.0%` | `40.4%` | `1.94°` | `3.41°` |
| fly3 右转 | 左 | `56.2%` | `37.5%` | `4.96°` | `14.08°` |
| fly3 右转 | 右 | `43.8%` | `49.2%` | `5.63°` | `4.57°` |

这直接支持候选策略：fly1 左转优先右侧，fly3 右转优先左侧。深度不能替代该判据：fly1 转弯右半深度中位数约 `221 m`，反而高于左半约 `213 m`；fly3 才是右半略远。因此应使用“转弯符号 + 实测视觉信息/残差”的因果策略，而不是固定“远点侧”规则。

现有静态半图实验也说明不能硬切：fly1 固定右半图的 yaw RMSE `3.43° → 2.46°`，但终点 XY `182.96 → 188.09 m`、速度 RMSE `0.535 → 1.091 m/s`；fly3 固定左半图使终点 XY 恶化到约 `357.7 m`。这不是对动态 ROI 的否定，而是证明全程静态 mask 会破坏空间覆盖和跟踪连续性。

## 7. 旧“绝对 yaw 缺失”结论为什么不足

P4 释放后全局 yaw 没有绝对观测，意味着一个已经写入的共同 yaw 偏差可以在保持小重投影 residual 的情况下长期存在。FC-yaw 诊断把误差拉回，只证明这个 gauge 机制存在。

它没有回答：

- 为什么误差在 fly1 首次左转和 fly3 首次右转后以不同过程写入；
- 为什么 Camera–IMU z 旋转负扰动对两条轨迹都有效；
- 为什么 fly3 首转后图像清晰度显著下降；
- 为什么左右图像区域的更新贡献与转向存在明显条件关系。

因此 FC/GPS yaw 不进入生产方案。它只保留为诊断正控制，避免再次把“能纠正”误写成“已定位根因”。

## 8. 两条正式实现路线

### 8.1 动态转弯 ROI

目标不是把图像硬裁成左/右半幅，而是：

```text
全图继续 KLT 跟踪已有轨迹
        +
按转弯方向偏置新点检测与后端候选排序
        +
左转优先右、右转优先左、直线恢复全图
```

转弯符号只用 board IMU 的重力轴角速度：

```text
omega_yaw = (omega_I - bg) · (R_GtoI e_z)
```

不使用 FC/GPS yaw。必须把 tracking mask、detection policy 和 backend eligibility 拆开；否则现有 `TrackKLT` 会把 mask 外已有点直接删除，随后把这些人为删除的轨迹当成 lost track 送给 MSCKF，恰好在转弯入口制造一次错误更新突发。完整设计见 `P4_DYNAMIC_TURN_ROI_IMPLEMENTATION_SPEC_20260716.md`。

### 8.2 AGL 场景尺度一致性重置

AGL 不作为 `p_z` Kalman measurement，也不通过交叉协方差改写姿态/bias。它只在 MSCKF/SLAM 已形成稳定场景几何后，估计统一 Sim(3) 尺度并对当前窗口状态、clone 和 landmark 做一致重参数化：

```text
s = robust(h_agl / h_map)
c_i' = c_now + s (c_i - c_now)
p_f' = c_now + s (p_f - c_now)
```

当前相机中心不动，姿态、bias、重力、内参、外参不变；速度和逆深度按尺度一致变换。名义值、FEJ 和 covariance 必须一起做 reset。AGL 不参与 Kalman innovation，但 covariance 仍必须按确定性坐标变换 `P' = J P Jᵀ`，否则滤波器会数学不一致。完整设计见 `P4_AGL_SCENE_SCALE_RESET_SPEC_20260716.md`。

均匀尺度不会直接旋转航向；它能修复的是 translation/depth/velocity 尺度及其后续视觉几何耦合。若 yaw 随之改善，必须解释为后端几何条件改善，不能声称 AGL 直接观测了 yaw。

## 9. Ultra 子代理实验矩阵

实验由 `gpt-5.6-sol / ultra` 子代理独立执行，主代理不重复跑同一矩阵。固定 P4、fixed stride12、full ROI、AGL off、P5 off；只替换校准字段。

| 编号 | intrinsics/distortion | `T_C_I` | time offset | 目的 |
| --- | --- | --- | ---: | --- |
| C0 | current mixed | June12 | `0` | 当前对照 |
| C1 | pure June12 | June12 | `+0.151 ms` | 同源 June12 |
| C2 | pure June12 | June12 | `0` | 单独隔离 intrinsics |
| C3 | current mixed | June12 | `+0.151 ms` | 单独隔离 toff |
| C4 | current mixed | 可追溯 old `T_C_I` | `0` | old/new TCI 因子；找不到精确 artifact 则不猜 |
| C5 | pure June12 | 可追溯 old `T_C_I` | `0` | intrinsics×TCI 交互 |
| C6 | current mixed | June12 + z-minus | `0` | 当前补偿响应 |
| C7 | pure June12 | June12 + z-minus | 明确记录 | 判断补偿是否只在 mixed 下成立 |

每个条件都跑 fly1/fly3 半圈到一圈，覆盖上述首错区间；保存 commit、dirty status、binary hash、完整 config、命令、P4 release fingerprint、q/p/v/bg/ba、NIS/residual/support 和正式绝对导航指标。只有存活条件再跑 full flight。

判别合同：

- C6 改善而 C7 不改善：`z_minus` 是 intrinsics–TCI 补偿，不是真实外参；
- C6/C7 在 fly1/fly3 都一致改善：TCI 候选错误升级；
- C1/C2 与 C3/C0 差很小：time offset 不是主因；
- 同一飞行改善、跨 fly1/fly3 反向：判为不可发布的同飞行补偿；
- P4 release fingerprint 改变时，把初始化差异和后端差异分开，不允许直接归因。

## 10. 优先级与完成条件

### P0：校准耦合锁定消融（正在 Ultra 并行执行）

先决定现有 `z_minus` 是真实 TCI 候选还是 mixed calibration 补偿。这一步决定后续所有 ROI/AGL 结果是否建立在正确投影模型上。

### P1：动态 ROI 正确实现

实现 full-image tracking、side-biased detection、backend eligibility 三层分离；左转取右、右转取左、直线全图。先单测“ROI 切换不会产生人工 lost-track update”，再跑 fly1/fly3 首错窗口。

### P2：AGL Sim(3) 场景尺度重置

先完成纯几何/状态单测，再做 shadow scale estimate；只有 scene-height 比值稳定才启用真实 reset。失败不得退回 output-only trajectory 缩放并冒充集成实验。

### P3：组合与全航程

只组合通过单变量验收的校准、动态 ROI 和 AGL reset。正式评价使用 GPS update time、start-heading、绝对 `G_nav`，不做 best-fit/SE(3) 后对齐；同时检查 course、heading、XY、along/cross、Vxy、U、bias、residual、NIS、covariance 和资源占用。

完成必须同时满足：

1. fly1/fly3 首错段 heading 增量和直线漂移斜率同时下降；
2. 改善不是靠 GPS/FC yaw、GPS XY 或未来误差；
3. P4 release 可解释且没有被无意改写；
4. KLT/MSCKF/SLAM 覆盖、接受率和 residual 不发生结构性退化；
5. 全航程误差不劣于同条件 control/global-baseline 证据；
6. 所有失败条件与 provenance 保留，不用挑飞行、挑终点或事后对齐掩盖。

## 11. 证据边界

当前可以说：校准混合、转弯侧别视觉贡献和 fly3 图像退化已经由数据支持，是下一轮最有判别力的方向。

当前不能说：`T_C_I` 已被重新标定正确；动态 ROI 已经可用；AGL 场景尺度重置已经集成；yaw 根因已经唯一证明。上述结论必须等待锁定消融和真实 estimator integration。

参考 D455 设备约束：[D400 Series Datasheet](https://dev.realsenseai.com/docs/intel-realsense-d400-series-product-family-datasheet)、[librealsense D455 motion-blur discussion](https://github.com/IntelRealSense/librealsense/issues/11180)、[auto-exposure priority and frame-rate discussion](https://github.com/realsenseai/librealsense/discussions/12923)。
