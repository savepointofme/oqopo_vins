# P4/P5 航向偏移因果消融规格

## 目标

针对 fly1 首次转弯附近和 fly3 首次转弯后至第二次转弯前的航向偏移，使用单变量回放区分以下三个候选原因：

1. P4 释放时对 `bg/ba` 的候选闭环反馈；
2. OpenVINS 视觉更新中的 yaw/OC 处理模式；
3. GPS-Z 标量更新经协方差交叉项对姿态或 bias 的间接影响。

P5 active 与固定 cadence 的同条件全程结果已经存在，二者在目标误差段具有相同漂移方向，因此本轮不重复把 P5 cadence 当作首要根因。

## 单变量合同

所有回放固定：数据集、起止时间、P4 8 秒窗口、0.5 秒推进、FC/IMU/视觉输入、全程 FC-board 标定、KLT 配置、camera stride 12、practical release 和 start-only 导航对齐。每次只改变表中一项。

| 变体 | 唯一变化 | 用途 |
| --- | --- | --- |
| `control` | 无 | 当前 P4-only 对照 |
| `retain_bias` | 禁止候选 `bg`、`ba` feedback；保留初始联合图估计及其 prior-retained covariance | 检验 P4 bias feedback 的联合影响 |
| `retain_bg` | 只禁止 `bg` feedback | 在 `retain_bias` 有效时定位 gyro bias |
| `retain_ba` | 只禁止 `ba` feedback | 在 `retain_bias` 有效时定位 accel bias |
| `yaw_fej` | `--yaw-mode fej`，其余不变 | 检验 baseline global-yaw OC 路径 |
| `no_gpsz` | 不运行 GPS-Z update，其余不变 | 检验 GPS-Z 交叉协方差耦合 |
| `mechanism_control` | 与 `control` 状态路径相同，但打开逐次 yaw/IMU/visual observability 日志 | 给机制消融提供同日志对照 |
| `no_visual_yaw` | 视觉更新的 yaw correction row 置零 | 区分视觉 yaw correction 是抑制还是制造漂移 |
| `no_visual_bgz` | 视觉更新的 `bg_z` Kalman gain row 置零 | 检验视觉量测经 yaw-bg 交叉协方差改写 `bg_z` 的影响 |
| `gyro_z_plus` | P4 释放后给 board gyro-z 加 `+1e-4 rad/s`；P4 窗口不变 | 检验剩余 yaw 漂移对惯性 z 轴残差的因果灵敏度 |
| `gyro_z_minus` | P4 释放后给 board gyro-z 加 `-1e-4 rad/s`；P4 窗口不变 | 与正扰动组成对称实验，排除随机回放差异和单方向偶然性 |
| `gyro_z_observed_plus/minus` | P4 释放后给 board gyro-z 加 `±7e-4 rad/s`；幅值覆盖当前目标直线中扣除 `bg` 后实测的约 `0.5–0.75 mrad/s` z 轴模型残差 | 检验现有 `±1e-4` 灵敏度实验是否因幅值远小于实际残差而没有形成可判定结果；要求正负结果对 control 呈方向相反、近似单调的航向响应 |
| `no_slam` | `max_slam=0`、`max_slam_in_update=0`，保留同一 KLT 与 MSCKF | 直接检验 SLAM landmark 更新是否制造目标航向偏移 |
| `gpsz_nasa_lean` | P4 保持不变，仅把 GPS-Z 从 `guarded` 改回历史 global baseline 的 `nasa_lean, beta=0.2` | 检验历史 fly1 较正/fly3 较歪是否来自 GPS-Z 更新合同差异 |
| `legacy_init_covariance` | 保持 P4 解出的 q/p/v/bg/ba、释放时间和后端配置不变，仅把注入 EKF 的协方差替换为历史 global baseline 的对角标准差：姿态 3°、位置 0.05 m、速度 5 m/s、陀螺 bias 0.003 rad/s、加计 bias 1 m/s² | 检验历史 fly1 较直但 fly3 较差是否由初始协方差及其交叉项合同造成 |
| `targeted_slam_freeze` | P4、KLT、MSCKF、GPS-Z 全部保持不变；仅在 fly1 第一转弯 `[1089.0,1151.0]`、fly3 第一转弯后直线后半 `[1000.0,1075.2]` 暂停 SLAM landmark update | 对用户指定的首次偏移区间做局部正反实验，判断该时段 SLAM 更新是在写入航向误差还是在抑制误差 |
| `gpsz_guard_pz_fallback` | 保留现有 GPS-Z residual、DXY、KXY 和 bias 安全门；当完整耦合更新被交叉协方差门拒绝时，不再完全丢弃高度量测，而是只对 `p_z` 状态与 `P` 的 `p_z` 行列执行 masked Joseph 更新 | 验证 fly3 在 `901.358–1081.835 s` 连续无有效 GPS-Z 是否促成首转后直线航向偏移；该变体不放宽门限，也不允许 GPS-Z 改写 XY、姿态、速度或 bias |
| `fc_attitude_release` | 保持同一 P4 窗口、Ceres 图、p/v/bg/ba 解、协方差和候选验证；只把候选 nominal `q_GtoI` 替换为同一释放时刻的同步 FC 姿态与已接受 `R_FtoI` 的组合 | 检验当前释放姿态约 2.4°/3.0°（主要是 roll）残差是否在后续协调转弯中转化为航向漂移；若目标段明显改善，说明问题在 P4 姿态解而非后端 cadence |
| `cam_toff_converged/opposite` | 仅把固定 Camera–IMU 时间偏移从 0 改为 fly1 `-3.94 ms`、fly3 `-1.52 ms`，并以同幅正号做对称反证 | 检验当前锁零的 Camera–IMU 时差是否在角运动和视觉更新中持续写入航向误差；只有负号同时改善、正号同时恶化且响应近似单调，才支持该根因 |
| `adaptive_stride_active` | P4 阶段继续逐原始帧供给并由自身窗口选择器选帧；只在 P4 释放后启用连续 P5 tracking gap 与 information-driven backend cadence | 直接检验正式 P5 是否改变首次航向漂移；P4 窗口指纹必须与 control 相同 |
| `full_rate_post_init` | P4 阶段与 control 相同；P4 释放后把固定 camera stride 从 12 改为 1，使 tracking/backend 都获得最高频率上界 | 判断固定约 0.4 s 稀疏视觉更新是否是航向漂移的主要原因，并为 adaptive P5 提供性能上界 |

`retain_bg`/`retain_ba` 是实验开关，不修改生产默认值。它们只把对应 CandidateGroupGate 设为 unconfigured，使该组不能反馈或被标记为可信；Ceres 图、bias 状态、IMU propagation、FC p/v update 和 q/p/v release 路径保持存在。

## 两阶段执行

第一阶段只跑覆盖目标误差段的 focus replay：fly1 `[930,1300]`，fly3 `[618,1150]`。先运行 `control`、`retain_bias`、`yaw_fej`、`no_gpsz`。若 `retain_bias` 对目标段产生一致且明显影响，再运行 `retain_bg` 和 `retain_ba` 分解；没有影响则不扩展无效矩阵。

第二阶段只把第一阶段能改变目标误差形成过程的变体跑 full flight，并用正式 GPS update-time/start-heading 口径验收。focus 结果只能定位机制，不能替代 full-flight acceptance。

若前四个变体都不能解释漂移，则追加 `mechanism_control`、`no_visual_yaw` 和 `no_visual_bgz`。这三组使用已有 StateHelper 诊断通道；不新增滤波算法，也不把离线 FC/GPS course 送入在线状态。

若上述实验只证明视觉 yaw correction 是稳定器、仍未区分惯性残差与视觉几何，则追加 `gyro_z_plus/minus`。扰动只在 `sys->initialized()` 后进入 board IMU，不改变 P4 Ceres、释放状态或初始化时间。若 gyro-z 是主要剩余机制，正负扰动应相对 control 产生显著、近似反对称的 yaw 斜率变化；若两者都落在重复性包络内，则该量级的 z 轴残差不足以解释现象。

`no_slam` 是视觉后端结构消融：P4、KLT、MSCKF、GPS-Z 与 cadence 保持不变，只去掉 SLAM landmark 状态、更新和 delayed initialization。若目标段 yaw 漂移显著消失且不是整体发散，则支持 SLAM 几何为原因；若漂移保留或恶化，则 SLAM 不是制造者，而是稳定器或次要项。

历史 global baseline 的 YAML 与当前有效 KLT/MSCKF/SLAM 参数一致，但其 GPS-Z 使用 `nasa_lean`，当前 P4 使用 `guarded`。`gpsz_nasa_lean` 只恢复这一项历史运行合同，初始化仍由 P4 完成，因此可以把 GPS-Z 模式与旧的一行 FC 初始化分开。

历史 global baseline 还使用固定对角初始协方差，而当前 P4 注入联合求解恢复的完整 15×15 协方差。`legacy_init_covariance` 不替换 P4 名义状态，也不改变 P4 的 solve/release 时刻，只替换 EKF release covariance；因此该实验专门识别协方差尺度与交叉项对后续 visual/SLAM Kalman 增益的影响。

当前 fly3 运行中，GPS 输入自身连续，但 `guarded` 模式从 `901.357621431 s` 到 `1081.835362434 s` 因 `REJECT_BY_DXY` 没有任何成功高度融合。`gpsz_guard_pz_fallback` 是针对这一内部量测中断的单变量实验：完整耦合修正仍被拒绝，只保留不污染水平状态的标量高度约束。若它不能在 `1000.0–1075.2 s` 之前改变航向误差斜率，则该 180.48 秒中断不是目标偏移的主要原因。

## 判定量

- 初始化时 heading/course 误差；
- 第一次达到 1° 航向误差的时间；
- 转弯前直线、转弯本体和转弯后直线的 heading error 增量与斜率；
- XY cross-track、along-track、速度向量、roll/pitch/yaw、gyro、`bg/ba`、GPS-Z update correction；
- KLT/MSCKF/SLAM feature 数、track age、accept/reject、visual residual；
- P4 release mask、释放 bias、协方差和 readiness。

因果支持要求：变体必须只改变指定输入合同，并在目标误差开始之前改变误差增长率或转弯增量；只改善最终点、只改变初始化常数或只在单架次偶然改善，不足以判为根因。
