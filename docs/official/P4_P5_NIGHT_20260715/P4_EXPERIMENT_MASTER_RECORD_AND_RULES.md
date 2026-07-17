# P4 实验总记录与强制试验规范

## 1. 文档地位

本文是当前 P4 实验工作的唯一总入口，统一回答四件事：

1. 已经做过哪些运行和评价；
2. 哪些结论仍有效，哪些已经被后续证据推翻；
3. 新实验开始前、运行中、评价时和宣称完成时必须做什么；
4. 哪些做法明确禁止。

证据优先级从高到低为：

```text
当前源码 + 当前二进制 + 同条件三飞行正式全程评价
> 当前源码的同条件 focus/半圈因果消融
> 当前源码单元/集成测试与运行 metadata
> 旧二进制正式评价
> shadow、诊断开关、离线扰动和相关性分析
> 设计文档中的未验证推测
```

旧报告与本文冲突时，以本文、机器总账中的实际产物和最新同条件正式评价为准。
设计文档中的“应当”“计划”“待验证”不得覆盖已经运行出的负结果。

## 2. 总账组成和覆盖范围

### 2.1 机器总账

- `P4_EXPERIMENT_LEDGER.csv`：逐条记录所有可发现的 run spec、原始运行、batch
  receipt 和评价目录；失败、中断、无轨迹和旧路径记录不会被删除。
- `P4_EXPERIMENT_CAMPAIGN_SUMMARY.csv`：按实验 campaign 聚合记录数、成功/失败
  运行、正式/诊断评价、飞行和目录合规性。
- `P4_EXPERIMENT_LEDGER_SUMMARY.json`：机器可读覆盖率和异常统计。
- `P4_EXPERIMENT_DECISION_LEDGER.csv`：按假设和算法代际记录结论、处理决定和是否
  允许再次运行。
- `analysis/build_p4_experiment_registry.py`：只读扫描并重建前三份机器总账。

本轮扫描范围为：

```text
C:\Users\baloney\Desktop\实验目录\P4*
C:\Users\baloney\Desktop\P4*              # 历史违规位置，保留但标记 legacy
analysis/run_specs/**/*P4*.json
```

截至 2026-07-17 本次重建，共登记 1321 条可发现记录：

| 类型 | 数量 | 含义 |
| --- | ---: | --- |
| raw run | 492 | runner 实际运行目录，包括失败和无退出回执 |
| evaluation | 571 | 含 `tables/global_summary.csv` 的评价目录 |
| batch | 201 | batch/preflight/exit receipt |
| run spec | 57 | 版本化评价输入合同 |

其中 365 个原始运行有成功退出和非空轨迹，109 条运行/batch 明确失败，63 个运行有
轨迹但缺少退出回执，3 个目录不完整或已放弃；8 个评价虽然生成了目录，但主 XY
指标为 N/A，已标为无效/不完整评价。1135 条记录位于正式实验根，186 条位于旧
Desktop 路径。所有 57 份 run spec 的轨迹输入当前均存在。

“全部记录”指上述两个扫描根中仍然存在、且带运行/评价证据的全部可发现 artifact。
已经被人工删除、移动到未知位置或从未落盘的运行无法伪造补录。

### 2.2 记录状态

| 状态 | 可用于什么结论 |
| --- | --- |
| `SUCCESS_WITH_TRAJECTORY` | 只证明运行成功，不证明算法有效 |
| `EVALUATED + FORMAL_MAIN` | 可用于同口径主指标比较 |
| `EVALUATED + DIAGNOSTIC_EVALUATION` | 只用于机制定位 |
| `FAILED` | 证明该次运行失败，必须保留 |
| `SUCCESS_NO_TRAJECTORY` | 运行退出但没有导航结果，按失败处理 |
| `ARTIFACT_WITHOUT_EXIT_RECEIPT` | 有数据但身份不完整，不能作为正式验收 |
| `INCOMPLETE_OR_ABANDONED` | 不得引用为算法结果 |
| `SPEC_ONLY` | 只证明评价合同存在 |

## 3. 已完成实验和当前有效结论

逐个目录和逐次评价见机器总账。本节按因果问题和算法代际汇总全部 campaign；完整
一一对应关系见 `P4_EXPERIMENT_DECISION_LEDGER.csv`。

### 3.1 2026-07-12 至 2026-07-14：旧生命周期

| 实验族 | 结果 | 当前用途 |
| --- | --- | --- |
| clean baseline、online joint alignment、arbitrary start | 建立单行 FC、一次有限窗 P4 和任意起点回放；多次失败和成功均已登记 | 历史参考，不代表当前滑窗实现 |
| P4/P5 mainline、redesign 20260713 | 实现一次 Ceres candidate 加递归候选滤波和分组反馈 | 被滑窗语义审计判为“固定 candidate 验证”，不是持续滑窗联合重估 |
| window-count v1–v9 | v6 曾释放；v8/v9 因 feedback safety 拒绝；多次 full/short 失败 | 固定次数/候选门限路线已冻结，禁止继续打补丁 |
| sliding-window semantic audit | 证明旧实现只有 buffer 滑动，没有 candidate 存活期的联合图重解 | 有效架构结论 |

### 3.2 2026-07-15：正式 P4 与首次 full-flight

正式有限窗生命周期、构建、单测和 fly1/fly3 full replay 均完成，但算法验收失败：

| 指标 | fly1 | fly3 |
| --- | ---: | ---: |
| final XY | 323.065 m | 784.034 m |
| XY RMSE | 205.247 m | 227.417 m |
| speed RMSE | 1.791 m/s | 3.131 m/s |
| course RMSE | 6.076° | 5.622° |

fly3 相对冻结 reference 的 final XY 增加 422.265 m。该 campaign 的正确状态是
`RUN_COMPLETE / ACCEPTANCE_FAILED`，不是“P4 已完成”。

### 3.3 2026-07-15 至 2026-07-16：航向漂移因果消融

| 假设/消融 | 当前结论 | 生产处理 |
| --- | --- | --- |
| 保留 `bg/ba` prior、分别冻结 `bg`/`ba` feedback | 没有同时消除 fly1/fly3 漂移 | 不是共同根因；不得重复当作首选修复 |
| baseline yaw OC 与 FEJ | 切换不能修复两飞行，部分条件退化 | 否决为共同根因 |
| 关闭或改变 GPS-Z | 不能消除共同航向漂移 | GPS-Z 只保留高度用途，禁止借此解释全部 yaw |
| visual yaw correction / visual-to-`bg_z` | 视觉 yaw 通道总体是稳定器；冻结 `bg_z` 通道不能消除漂移 | 不允许直接删视觉 yaw 行或 `bg_z` 行 |
| no-SLAM、局部 SLAM freeze | no-SLAM 在 fly3 明显恶化；SLAM 点有好有坏 | SLAM 不是单独制造者，禁止整体关闭冒充修复 |
| gyro-z 正负扰动和 intrinsic | 跨飞行没有一致、近似反对称的因果响应 | 固定 gyro-z 补偿被否决 |
| Camera–IMU time offset | 毫秒级扰动有灵敏度，但无跨飞行单调证据 | 不允许按最终 GPS 误差选择符号 |
| FC-yaw aid | 能拉回全局 yaw，证明偏差写入后单目 VIO 无绝对 yaw 恢复能力 | 仅诊断正控制；GPS/FC yaw 不得进入正式 VIO |
| Camera `K/D/T_C_I` factorial | 证明当前配置存在 mixed-calibration 耦合和参数互相补偿；没有唯一正确组合 | 未取得同源重标定前不得发布经验角补偿 |
| 静态左右半图 | 单架次局部可能改善，另一架次或速度/位置退化 | 否决 |
| 动态转弯 ROI | 真正实现“左转取右、右转取左”；高度/尺度改变明显，但 fly1/fly3 航向和 XY 均未通过 | 默认关闭，不能再重复同一策略 |
| AGL scene-scale single reset | reset 数学合同生效，但 AGL/map height 不是全局惯性尺度；XY/速度立即退化 | 否决全局 Sim(3)；shadow 仅诊断 |

用户指定的正确首错区间固定为：

| 飞行 | 首次主要转弯 | 转后关键直线 |
| --- | --- | --- |
| fly1 | 940.553–983.753 s 左转 | 983.753–1089.750 s |
| fly3 | 693.200–740.100 s 右转 | 740.100–897.200 s |

旧报告中使用 1089 s 后 fly1 转弯和 897 s 后 fly3 转弯作为“首次误差”的分析已经
降级，不得继续引用为首次根因窗口。

### 3.4 2026-07-16：姿态 gauge、yaw sigma 和转向方向验证

| 实验族 | 当前结论 |
| --- | --- |
| `fc_attitude_release` / attitude gauge | 证明原 P4 图缺少绝对姿态 gauge 会允许释放姿态偏离 FC；直接替换 FC 姿态只是诊断，不能发布 |
| calibrated yaw sigma / low-dynamic gauge / q release | 对某些飞行和区间有效，但跨飞行、全程和 bias 组合结果不统一；没有形成正式生产配置 |
| final-yaw-only | fly1 局部形状改善但尾段仍漂；不是完整 P4 方案 |
| fly2 direction validation | 证明固定方向补偿会随转弯方向反转；同时完成多组 q/p/v、bias、covariance 交叉替换 | 只支持状态强耦合，不支持某个分块单独覆盖 |

### 3.5 2026-07-17：临时 VIO、直接联合图和交接消融

| 代际 | 结果 | 判定 |
| --- | --- | --- |
| provisional VIO + P4 yaw/position gauge + history Sim(3) | fly1 有收益，fly3 第一条直线出现 4–7 m/s 顺航向速度过冲 | 否决；惯性状态不能任意整体缩放 |
| 直接 8 s 滑窗、每关键帧 q/p/v/bg/ba、15D CPI | 图拓扑与 OpenVINS DynamicInitializer 一致 | 拓扑本身保留 |
| 窗内共享 bias + 两阶段重预积分 | `ba_z` 随窗口内容在约 -2.02 至 -1.32 m/s² 变化，无法稳定释放 | 否决共享 bias |
| 单个 FC q/p/v anchor | 结果强烈依赖 anchor 时刻，部分窗口末端速度错误极大 | 否决 |
| FC p/v 归一化时间积分 | 避免相关 FC 行随帧数重复增权，并改善 fly2 半圈 | 当前 v14 图模型，但未通过三飞行全程 |
| FC gauge/relative-shape | 首个后端视觉更新产生约 3.5 m/s 速度改动，半圈劣于单行 | 已撤销 |
| 只注入末端状态、删除历史 | 首转后直线可改善，但首转和视觉冷启动会退化 | 历史消融已做，不是新发现 |
| 注入 clone/history、重复使用 P4 像素 | 短程改善、全程退化，属于视觉信息双重使用 | 否决 |
| 注入 history + persistent landmark + 联合 covariance | 避免部分冷启动，但 v14 三飞行结果未通过 | 未验收 |
| 对完整联合 covariance 做 `D P D^T` inflation | 与上游 OpenVINS 实际实现不一致，改变全部交叉相关 | 当前已撤销，不能称为原生语义 |

### 3.6 v14 与同运行时单行对照

主口径：GPS update grid、start-heading、相同后端配置和公共时间窗。

| 飞行 | 条件 | XY RMSE | final XY | course RMSE | final course |
| --- | --- | ---: | ---: | ---: | ---: |
| fly1 | v14 P4 | 164.34 m | 106.18 m | 4.08° | 12.30° |
| fly1 | current single-row | 108.87 m | 111.33 m | 3.02° | 5.29° |
| fly2 | v14 P4 | 354.28 m | 657.73 m | 2.21° | 3.91° |
| fly2 | current single-row | 118.75 m | 168.43 m | 1.49° | 2.03° |
| fly3 | v14 P4 | 241.92 m | 253.14 m | 5.43° | 19.79° |
| fly3 | current single-row | 242.92 m | 352.05 m | 5.40° | 15.43° |

结论：v14 不能验收。fly1/fly2 的主要退化被同运行时对照定位到 P4 初始化/交接
路径；fly3 位置混合、尾段航向更差。大 bias 本身不是完整根因，因为运行一段时间后
P4 与单行 bias 轨迹趋近。

### 3.7 当前未完成项

最新“仅终端状态 + 上游对角块 inflation”只完成构建和单元测试。当前 focus 回放中：

- fly3 产生成功轨迹；
- fly2 没有产生可评价轨迹并退出 1；
- fly1 有轨迹但缺退出回执；
- 该方向与既有 no-history 消融高度重叠。

因此它的状态是 `DIAGNOSTIC_RECHECK / NOT_ACCEPTED`，不得写成修复完成，也不得在未
消化既有 no-history 结果前继续扩展全程矩阵。

## 4. 强制试验流程

### 4.1 开始前必须完成

每个实验开始前必须：

1. 查询 `P4_EXPERIMENT_LEDGER.csv` 和 `P4_EXPERIMENT_DECISION_LEDGER.csv`，确认没有
   相同代码、配置、数据、时间窗和干预合同的既有运行。
2. 写明唯一假设、唯一干预量、因果链和结果决策表。必须预先回答“改善、无变化、
   退化分别意味着下一步改什么”；不能跑完后才临时解释。
3. 保存同二进制、同配置、同数据、同时间窗的 no-change 或单行 baseline。不同日期
   的历史 global baseline 只能作为第二参考，不能替代同运行时 control。
4. 冻结数据身份、FC/GPS 文件、时间偏移、坐标系、单位、相机/IMU 标定、P4 窗口、
   camera stride、GPS-Z 模式、yaw/OC 模式和 KLT/MSCKF/SLAM 配置。
5. 写 run spec，并先执行 `flight_eval_tool.py inspect-run`。run spec 缺输入时不得运行
   正式评价。
6. 明确在线输入与离线 reference。GPS XY/course/yaw 和最终轨迹误差不得进入 P4、
   OpenVINS、release gate 或参数选择。
7. 使用 `-j12` 构建受影响 target；记录实际 runner 和共享库 hash。不得因习惯改成
   单核编译。

### 4.2 实现后必须验证

1. 先运行直接相关单元/集成测试，再运行最短但覆盖目标事件的 focus/半圈。
2. focus 只用于否定错误机制或决定是否值得 full；不能替代 full acceptance。
3. 任何准备进入生产的 P4 改动必须用同一实现验证 fly1、fly2、fly3。fly2 是反向
   转弯验证，不能省略。
4. P4-only 实验必须关闭 P5 active。P4 收集阶段使用冻结的固定输入 cadence；若测试
   P5，P5 只能在 P4 release 后启动，并证明 P4 window fingerprint 与 control 一致。
5. 必须检查非空 `traj_nav.txt`、退出码、frame contract、release readiness、窗口
   fingerprint、q/p/v/bg/ba、covariance、NIS/residual、MSCKF/SLAM accepted/rejected、
   运行耗时和内存。
6. correction、reset 或 covariance 语义发生变化时，必须验证 nominal、FEJ、完整
   cross-covariance、对称性、PSD、cache invalidation 和下一次真实 backend update。
7. 运行失败时先定位失败层级。preflight/provenance/validator 失败与 estimator 失败
   必须分开记录；不得把没有进入估计器的失败算成算法结果。

### 4.3 正式评价必须遵守

1. 使用仓库标准 `analysis/full_flight_error_analysis.py` 或
   `analysis/flight_eval_tool.py`，禁止另造不同口径的一次性脚本。
2. GPS 原始更新时间为统计网格；每个 GPS 时刻取其后第一条 post-update VIO 状态，
   不跨更新向后插值。
3. 主评价使用 ENU、start-heading 和首样本 translation，只做起点对齐；保留尺度和
   漂移。best-fit/SE(2)/SE(3)/Sim(3) 只能标为诊断。
4. reference velocity 优先使用 FC raw `Ve/Vn/Vu`；VIO velocity 优先使用
   `traj.txt.bias`。
5. control/candidate 必须裁剪到严格公共有效时间窗；初始化前无 VIO、图像读取失败、
   SLAM 清空或重初始化后的无效尾段不得进入指标。
6. 必须报告 XY RMSE、final XY、E/N/U、along/cross、speed magnitude、Vxy vector、
   course/yaw、里程漂移、分段结果和样本覆盖率。
7. 每圈路线报告只保留两个直线和两个主要转弯；标准几何细分可以保留在明细表，
   不能在用户 dashboard 中制造几百个“航段”。
8. dashboard 必须包含真实 `window.RUN_DATA`、整体误差曲线、速度/位置/姿态/航向、
   分段和轨迹图；大量 `N/A`、缺曲线或脚本未执行均判 dashboard 失败。
9. failed、truncated、N/A 和无轨迹运行必须显示，禁止静默从汇总中删除。

### 4.4 结论和验收必须满足

1. 单元测试通过只能写“测试通过”；不能写“算法可用”。
2. shadow 只能写“shadow”；诊断替换、FC-yaw aid、GPS truth 干预不能写“生产修复”。
3. 相关性、同步峰值或单飞行改善不能写“根因已证明”。根因需要单变量干预在目标
   误差形成前改变其增长过程，并有反向/跨飞行证据。
4. P4 正式验收至少要求 fly1/fly2/fly3 全程稳定 release，且相对同运行时单行和冻结
   global baseline 不出现关键导航指标的实质退化。任一飞行显著退化即判失败。
5. 必须检查首次转弯、转后第一直线、第二转弯和尾段；不能用终点偶然抵消掩盖中途
   速度、尺度或航向错误。
6. full replay 成功不等于 acceptance 通过。只有正式评价通过才能更新算法报告、恢复
   P5 active 验收或进入 June12。

## 5. 明确禁止事项

以下行为一律禁止：

1. 未查询总账就重复既有实验；同合同重复运行必须标记 `REPEATABILITY_RUN` 并预先
   写明样本数目的。
2. 只修改参数、门限或两行代码，就把未验证的架构性问题称为“完整重构”。
3. 同一次因果实验同时改变初始化、GPS-Z、yaw mode、cadence、KLT/SLAM 配置或评价
   时间窗。
4. 根据最终 GPS 误差、未来轨迹或 best-fit 结果反向选择在线门限、bias、安装角、
   Camera–IMU offset、ROI 方向或补偿符号。
5. 把 GPS/FC yaw 加入生产 VIO来消除漂移；该量只能作为明确标注的诊断正控制，除非
   用户正式改变系统输入合同。
6. 把 GPS horizontal 融入当前 baseline；GPS-Z 只能走已有显式模式。
7. 把运行成功、轨迹非空、standalone 单测、shadow 日志或 dashboard 能打开当作算法
   验收。
8. 用 start-yaw、全程 SE(3)、Sim(3) 或 best-fit 对齐掩盖初始化和尺度误差。
9. 使用旧二进制、缺失 `libov_msckf_lib.so` 的残留可执行文件或无法核对 hash 的结果
   冒充当前源码证据。
10. 删除失败目录、覆盖旧结果、只汇报改善项，或把 N/A 从比较表中移除。
11. 让深度 provenance/hash 扫描阻塞正式回放。只保留轻量 command/config/binary/input
    身份；记录失败不能在估计器启动前无条件终止实验。
12. 在 P4 未通过正式三飞行 full acceptance 前切换到 P5、P6 或 June12，并把旁支
    工作写成 P4 完成。
13. 继续在已否决的固定 candidate、固定次数确认、共享 bias、固定左右半图、全局
    AGL Sim(3)、固定 gyro-z 或经验安装角补偿上调参。
14. 实验跑完只给一句“无可用修复”。每次实验必须落实为保留、否决、修改模型或停止
    某条路线中的一个明确决策。

## 6. 目录与最小证据合同

新实验必须写入：

```text
C:/Users/baloney/Desktop/实验目录/<campaign>/
  <YYYYMMDD_flight_method_scope_status>/
    command.txt
    exit_code.txt
    stdout.log
    stderr.log
    process_start.txt
    process_end.txt
    implementation_sha256.txt
    config_snapshot/
    traj_nav.txt
    traj.txt.bias
    diag.csv
    online_alignment_metadata.json
    metadata/
```

正式评价必须包含标准 `plots/`、`tables/`、`data/`、`reports/` 和 `metadata/`，并保留
`global_summary.csv`、`gps_time_aligned_samples.csv`、正式报告和可交互 dashboard。

## 7. 总账重建与提交前检查

重建命令：

```powershell
python analysis\build_p4_experiment_registry.py `
  --repo D:\vscode_dir\open_vins_p4_sliding_r1 `
  --experiment-root C:\Users\baloney\Desktop\实验目录 `
  --legacy-root C:\Users\baloney\Desktop `
  --out-csv docs\official\P4_P5_NIGHT_20260715\P4_EXPERIMENT_LEDGER.csv `
  --out-summary docs\official\P4_P5_NIGHT_20260715\P4_EXPERIMENT_LEDGER_SUMMARY.json `
  --out-campaign-csv docs\official\P4_P5_NIGHT_20260715\P4_EXPERIMENT_CAMPAIGN_SUMMARY.csv
```

每次新运行或评价后必须重新生成总账，并检查：

- 新 record 数量符合预期；
- run spec 输入不存在项为 0；
- 当前正式结果位于实验根；
- 新失败和无退出回执均已解释；
- `run_contract_hash` 的重复组有明确重复理由；
- decision ledger 已更新结论和下一步。
