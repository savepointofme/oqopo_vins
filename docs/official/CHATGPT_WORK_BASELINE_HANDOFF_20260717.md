# P4/P5 当前代码与实验基线交接（2026-07-17）

## 1. 一句话结论

本基线是**可构建、局部测试通过，但 P4 与 P5 均未通过正式算法验收的实验快照**。

- P4：已做了大量联合初始化、姿态 gauge、标定、注入、尺度、ROI 和航向漂移消融；现有长窗方案在同运行时对照中没有稳定优于单行 FC 初始化，fly1/fly2 明显退化。当前代码不能称为正式 P4。
- P5：连续前端调度器的局部测试和 shadow 非污染检查成立，但正式 backend event trigger、active takeover 和全程同口径收益尚未完成。当前代码不能称为可用 P5。
- 2026-07-17 新写的正式 P4 实现规格是下一版设计合同，**尚未在当前代码中实现**。不要把规格文档与当前运行行为混为一谈。

本快照的用途是让下一位直接复核真实代码和证据，不再从口头记忆、错误 global baseline 或零散实验目录重新猜。

## 2. Git 身份与复现入口

- 分支：`redesign/p4-sliding-window-r1-20260714`
- 标签：`baseline/p4-p5-chatgpt-work-20260717`
- 本轮开始前父提交：`fdd8d745229c7115b56b5d2d765f462a3fd68b4b`
- 工作树：`D:\vscode_dir\open_vins_p4_sliding_r1`
- WSL 路径：`/mnt/d/vscode_dir/open_vins_p4_sliding_r1`
- 远端：`origin`，仓库 `savepointofme/oqopo_vins`

拉取方式：

```bash
git fetch origin --tags
git switch --track origin/redesign/p4-sliding-window-r1-20260714
git checkout baseline/p4-p5-chatgpt-work-20260717
```

标签指向的提交就是本交接快照。大型飞行输出、build 目录、PID 和运行日志没有提交；实验命令、run spec、分析脚本、小型 evidence、正式结论和实验总账已提交。

## 3. 当前代码到底是什么

### 3.1 P4 当前运行架构

当前 `OnlineAlignmentInitializer` 不是已经冻结的新正式方案，而是多轮实验叠加后的大体量实现，包含：

- FC/board IMU/单目 KLT 输入缓存；
- 多候选时间长度与关键帧选择；
- Ceres 联合图、FC gauge、候选滤波、终端注入和多种实验分支；
- q/p/v/bg/ba、协方差、clone/history、SLAM landmark、AGL scene scale、turn ROI 等试验接口；
- runner 中大量用于消融的环境变量和 metadata 字段。

旧主架构曾经实际执行：

```text
首次有限窗 Ceres
  -> 固定 candidate
  -> IMU 因果传播 + FC p/v 递归验证
  -> 分组反馈
  -> 接受后永久关闭
```

审计已经证明这不是用户要求的“每个新时间窗使用完整 FC/IMU/视觉证据持续联合重估”。不要继续围绕 candidate 固定次数、history count 或 release threshold 修补。

后续又加入 upstream dynamic-init FC gauge、直接长窗图、terminal injection、history/landmark 注入等路径；这些路径有局部正结果，但都没有形成 fly1/fly2/fly3 可迁移的正式通过结果。

### 3.2 P4 新设计合同

下一版必须以以下两份文档为设计入口：

- `docs/official/P4_INITIALIZATION_LITERATURE_AND_ARCHITECTURE_REVIEW_20260717.md`
- `docs/official/P4_FORMAL_INITIALIZATION_IMPLEMENTATION_SPEC_20260717.md`

新合同核心是：

```text
一个明确的有限时间窗
  -> 每关键帧 q/p/v
  -> 窗口共享 bg/ba
  -> 标准 15 维 IMU 预积分
  -> FC metric p/v/attitude 观测直接进入图
  -> 视觉使用不显式保留 landmark 的相对约束/消元语义
  -> 内部 covariance / innovation / residual 验收
  -> 一次原子化注入 OpenVINS nominal + FEJ + covariance
```

新设计不允许：

- 把普通 OpenVINS dynamic initializer 再跑一遍后只做外部贴合；
- 用单个 FC 时间戳硬锚定整窗；
- 用 GPS/FC yaw 作为生产期持续航向输入；
- 把同一批 P4 pixel/landmark 再送入后端造成视觉双计数；
- 从 1D AGL 高度构造全局 7DoF Sim3；
- 用未来 GPS 误差调在线门限。

### 3.3 P5 当前运行架构

旧 `AdaptiveStrideController` 和旧 shadow replay 已删除，当前新增/修改包括：

- `ov_msckf/src/ros_free/AdaptiveVisualScheduler.h`
- `ov_msckf/src/core/VisualCadencePlanner.h`
- `ov_msckf/src/core/BackendUpdateTrigger.h`
- runner、tracker、VioManager 和测试接线。

目标语义是连续的 tracking horizon 和 event-driven backend，而不是在 `8/4/2/1` 四档中选择。任意整数 gap 只能是连续控制量最终落到离散帧时刻的结果，backend 不能再伪装成第二个固定 stride。

但是当前正式状态仍为：

- 连续前端 scheduler 局部测试：通过；
- shadow 对 P4 非污染：通过；
- 完整 backend trigger shadow：未完成；
- active takeover：未运行正式验收；
- full-flight P5 与固定基准的成本/精度收益：未建立。

## 4. 代码规模与构建状态

相对父提交，当前已跟踪修改约为 38 个文件、8518 行新增、2870 行删除，另有新的源码、测试、规范、实验总账和分析脚本。

本次交接前使用 `-j12` 完成直接相关构建：

```bash
cmake --build build_p4_sliding_r1 \
  --target run_serial_msckf_ros_free \
           test_adaptive_stride \
           test_online_alignment_initializer \
           test_online_alignment_candidate_filter \
           test_online_alignment_window_injection \
           test_online_vio_fc_gauge_aligner \
           test_state_sim3_scale_reset \
           test_agl_scene_scale_controller \
           test_agl_scene_scale_ground_estimator \
           test_dynamic_turn_roi \
           test_visual_observability_policy \
  -j12
```

构建结果：上述目标全部成功。第一次组合命令因把真实 target `test_dynamic_turn_roi` 写成了不存在的 `test_dynamic_turn_roi_policy` 而退出；查明 target 后已补建成功。不能把第一次退出删掉或改写成全程一次通过。

直接相关测试结果为 10/10 通过：

```text
continuous adaptive visual scheduling tests 1-15 passed
AGL scene-scale controller: 57 contract checks passed
AGL scene-scale ground estimator: 17 checks passed=17 failed=0
dynamic turn ROI policy tests passed
online alignment candidate filter tests passed
online alignment initializer tests passed
online alignment window injection tests passed
online VIO-FC gauge aligner tests passed
StateHelper Sim(3) scale reset: 113 checks passed, 0 failed
visual observability policy tests passed
```

这些测试只证明局部代码合同，不证明任何被拒绝的算法重新有效。

构建慢的直接技术原因也必须保留：`run_serial_msckf_ros_free.cpp` 与 `OnlineAlignmentInitializer.cpp` 已膨胀成很大的单翻译单元。`-j12` 对多个 `.cpp` 有效，但无法并行编译同一个巨大 `.cpp`；在 `/mnt/d` 链接大型共享库也有明显 I/O 延迟。不要再次错误地要求 `-j1`。

## 5. 实验记录总览

正式总账入口：

- `docs/official/P4_P5_NIGHT_20260715/P4_EXPERIMENT_MASTER_RECORD_AND_RULES.md`
- `docs/official/P4_P5_NIGHT_20260715/P4_EXPERIMENT_LEDGER.csv`
- `docs/official/P4_P5_NIGHT_20260715/P4_EXPERIMENT_CAMPAIGN_SUMMARY.csv`
- `docs/official/P4_P5_NIGHT_20260715/P4_EXPERIMENT_DECISION_LEDGER.csv`
- `docs/official/P4_P5_NIGHT_20260715/P5_EXPERIMENT_LEDGER.csv`

机器扫描记录：

| 项目 | 数量 |
| --- | ---: |
| 总记录 | 1321 |
| raw run | 492 |
| evaluation | 571 |
| batch | 201 |
| run spec | 57 |
| success 且轨迹非空 raw run | 365 |
| 明确失败的 run/batch | 109 |
| 缺少 exit receipt 的轨迹 | 63 |
| incomplete/abandoned | 3 |
| 主 XY 为 N/A 的 evaluation | 8 |
| 正式实验根目录记录 | 1135 |
| legacy Desktop 记录 | 186 |

57 份 run spec 指向的 trajectory 当前均存在。数量不能替代质量；大量记录是诊断、失败、旧二进制或非正式评价。

正式评价合同：GPS update time 采样；仅使用起始平移和 start heading 对齐；保持尺度；reference velocity 使用 FC `Ve/Vn/Vu`；VIO velocity 使用 `.bias`；best-fit 只作诊断。主脚本是 `analysis/full_flight_error_analysis.py` 与 `analysis/flight_eval_tool.py`。

## 6. 已做实验与可保留结论

### 6.1 旧 candidate 闭环与滑窗语义审计

已做：一次 Ceres candidate、递归 FC p/v update、q/p/v/bg/ba 分组反馈、covariance reset、未来量测确认、重复输入和 safety gate 测试；并对源码与 fly1/fly3 receipt 做完整调用链审计。

结论：

- buffer 在滑动，但 candidate 存活期间 Ceres 图不滑动、不重建、不重解；
- 固定 10 个关键帧与固定 10 个 FC 更新是两个实现层次，都不是由实际时间窗自然推出；
- release 后永久关闭，未可信 bias 没有后续新联合窗机会；
- 因此旧架构不是目标算法，不能再调 gate 冒充修好。

### 6.2 2026-07-15 正式 P4 full-flight

| 飞行 | final XY | XY RMSE | speed RMSE | course RMSE |
| --- | ---: | ---: | ---: | ---: |
| fly1 | 323.065 m | 205.247 m | 1.791 m/s | 6.076° |
| fly3 | 784.034 m | 227.417 m | 3.131 m/s | 5.622° |

fly3 相对冻结参考 final XY 退化约 422.265 m。正式判定：FAIL。

### 6.3 航向漂移根因与消融

用户指定的首次大误差窗口：

- fly1：首个大左转 `940.553–983.753 s`，转后直线 `983.753–1089.750 s`；
- fly3：首个大右转 `693.200–740.100 s`，转后直线 `740.100–897.200 s`。

可保留结论：

- 冻结或加强 bg/ba prior 没有同时消除 fly1/fly3 漂移；
- OpenVINS yaw OC/FEJ 开关和 GPS-Z 模式不是共同修复；
- 正常 visual yaw correction 总体在稳定 yaw，删除 visual yaw/bg_z correction 是错误方向；
- no-SLAM 会让 fly3 更差，SLAM 点不是唯一根因；
- gyro-z 正负补偿、IMU intrinsic、camera-IMU time offset 均未显示跨飞行一致的单调因果响应；
- FC yaw aid 可以把 yaw 拉回，只能作为正对照，生产环境没有该绝对 yaw 输入；
- K/D/T_C_I 因子实验表现为混合耦合，不能从单次好结果宣称唯一正确组合；
- fly1 与 fly3 转向相反，固定符号补偿存在跨飞行反作用风险；
- final-yaw-only 在 fly1 局部形状改善，但后段漂移仍存在，不能验收。

### 6.4 动态转弯 ROI

用户要求的策略最终确实实现为：左转取右半图，右转取左半图；依据是滚转内侧视野更容易集中远点，外侧更可能保留近地几何。

实验结论：实现改变了高度/尺度相关统计，但 fly1/fly3 的 yaw 与 XY 没有形成一致改善，正式拒绝，默认关闭。静态固定左/右半图也不可迁移。

这不等于 ROI 永远无价值；它只否定当前二分半图、当前触发和当前数据路径是正式修复。

### 6.5 AGL 尺度后处理

已实现与测试 scene-scale controller、ground estimator 和 StateHelper Sim3 reset。

结论：

- 代码层 reset 数学检查通过；
- 只有 AGL 单维高度，不能恢复任意 7DoF Sim3；
- AGL/地图地面高度不等于全局惯性尺度观测；
- 对全局 q/p/v/clone/landmark 做统一尺度重置会立即破坏 XY/velocity，一致退化；
- 全局 Sim3 路径正式拒绝，仅保留 shadow/诊断代码。

### 6.6 June12 intrinsics/distortion 拆分

`evidence_staging/FINAL_NEWCODE_KD_VERDICT.md` 是最终因果结论。

- Focus 2×2 `C0/KONLY/DONLY/KD`：fly1/fly3 共 8/8 成功；
- Full `C0/KONLY/DONLY`：fly1/fly3 共 6/6 成功；
- June12 distortion D 是主导有害量：
  - fly1 XY RMSE `180.625 -> 561.228 m`，增加 380.603 m；
  - fly3 XY RMSE `226.887 -> 1847.374 m`，增加 1620.488 m；
- June12 K 没有可迁移收益：focus 两飞均恶化约 47–48 m；full fly1 恶化 38.641 m，fly3 改善 61.523 m，符号相反；
- K×D 存在拮抗补偿，K 能部分掩盖 D，但 KD 仍不可接受；
- 分辨率、camera model、参数顺序和 downsample 的几何审计通过，只证明输入合同一致，不证明该物理标定适用于当前飞行数据；
- 该实验没有证明 `T_C_I` 或 time offset 正确/错误。

### 6.7 长窗、gauge、注入与同运行时单行对照

已试路径包括：

- provisional VIO + FC gauge + history Sim3；
- 每关键帧 q/p/v/bg/ba + 15D CPI 的直接 8 s 图；
- shared bias 两阶段 re-preintegration；
- 单 FC q/p/v anchor；
- FC p/v normalized time integral；
- terminal-only injection；
- clone/history 注入；
- persistent landmark/history covariance；
- joint covariance inflation 与 upstream diagonal block inflation。

关键结论：

- 原 P4 图缺少绝对姿态 gauge 是真实问题，但直接替换 FC attitude 只构成诊断，不是完整解；
- shared-bias 解对窗口很敏感，`ba_z` 曾在约 `-2.02` 到 `-1.32 m/s²` 间变化，拒绝；
- 单 FC anchor 强依赖 anchor timestamp，拒绝；
- history + P4 pixels/landmarks 会造成视觉信息复用/双计数风险，短程可好、全程变差，拒绝；
- 对完整联合 covariance 直接做 `D P D^T` inflation 不是 upstream OpenVINS 语义，会改变交叉协方差，已回退；
- terminal-only 注入可改善某些转后直线，但会恶化转弯和视觉冷启动，尚非正式方案；
- 初次 backend update 曾出现约 3.5 m/s 的速度修正，说明初始化 handoff 与后端先验不一致。

当前 v14 与同运行时单行 FC 初始化的 common-window 对比：

| 飞行 | 方案 | XY RMSE | final XY | course RMSE | final course |
| --- | --- | ---: | ---: | ---: | ---: |
| fly1 | v14 | 164.34 m | 106.18 m | 4.08° | 12.30° |
| fly1 | single-row | 108.87 m | 111.33 m | 3.02° | 5.29° |
| fly2 | v14 | 354.28 m | 657.73 m | 2.21° | 3.91° |
| fly2 | single-row | 118.75 m | 168.43 m | 1.49° | 2.03° |
| fly3 | v14 | 241.92 m | 253.14 m | 5.43° | 19.79° |
| fly3 | single-row | 242.92 m | 352.05 m | 5.40° | 15.43° |

正式结论：v14 FAIL。fly1/fly2 的退化主要定位在 P4 初始化/交接，fly3 是混合结果。所谓 single-row 只是同运行时隔离长窗是否有害的控制组，不是用户最终要求的算法，也不能替代冻结 global baseline。

### 6.8 P5 shadow、固定 stride sweep 与旧 active 结果

当前 shadow：

- fly1/fly3 raw policy rows 为 5097/9056；
- 实际仍用固定 stride 12；
- tracking/backend 顺序约束无违规；
- 修复计数器污染后，P4 prefix timestamps 与 15D candidate state 精确一致；
- tracking-only 为 0，clone violation 为 0；
- 这里只证明前端建议器可以旁路记录，不证明 active 调度收益。

4 flights × 8 fixed strides：

- 32/32 目录和 exit receipt 完整；
- 23/32 在 1000 m divergence；
- 仅 8/32 完成指定窗口；fly1/fly3 为 0/8；
- 缺少正式 AGL/ground/time/parallax 合同，不能用于确定 adaptive 阈值。

旧 2026-07-14 active full：

- fly1 raw/KLT/backend：30226/20989/13809；
- fly3：36552/23750/17643；
- information trigger 触发率约 95.65%/96.50%，几乎等于总在触发，说明 proxy 无有效节省；
- 约 200 m 高度仍经常被 polygon overlap 判为 LOW，语义错误；
- 旧 runner/Vio/P4 与当前代码不一致，不能认证当前正式 P5。

## 7. 当前可接受、已拒绝和未证明项

### 可保留为代码/证据

- 当前工作树能构建，10 个直接局部测试通过；
- 实验总账、正式评价工具、run spec 和关键小型 evidence；
- 滑窗语义审计、航向漂移窗口和多项负因果结论；
- June12 K/D 拆分结论；
- 新 P4 文献审查和实现规格；
- P5 连续 scheduler 的接口方向与非污染测试。

### 已正式拒绝

- 旧 fixed candidate/count 架构；
- 当前 v14 作为正式 P4；
- 单 FC anchor；
- 强制 FC/GPS yaw 生产融合；
- 全局 AGL Sim3；
- 当前 dynamic half-image ROI 作为默认修复；
- history/pixel/landmark 双计数注入；
- June12 distortion；
- 把旧 P5 active 结果当作当前正式验证。

### 仍未证明

- 新正式 P4 规格能否实现且跨 fly1/fly2/fly3 优于单行和冻结 global baseline；
- 安装误差角是否应使用何种保守先验及其在线变化模型；
- current K、`T_C_I`、camera-IMU time offset 的唯一可迁移组合；
- 正式连续 P5 的计算节省、精度保持和 backend event trigger；
- P4+P5 组合全程验收。

## 8. 不可饶恕的错误与责任清单

这些错误必须原样交代，不能用“探索过程”淡化。

| 错误 | 造成的后果 | 后续硬约束 |
| --- | --- | --- |
| 在 full-flight P4/P5 未验证时宣称任务完成 | 用户被误导，验收顺序中断 | 只有同口径全程结果满足标准才能写完成 |
| 把一次 Ceres candidate + 递归验证称为滑动窗口持续联合重估 | 架构与明确需求相反，数天围绕错误对象打补丁 | 先画真实调用链；新窗必须重建/更新完整图才可称滑窗 |
| 发明固定 10 次 update/history release 语义 | 把实现常数冒充物理时间证据 | release 只用时间覆盖与内部统计量，不用任意固定次数替代窗口 |
| P4 未验收就切入 P5 | 主任务失焦，产生大量不可组合结果 | 先冻结通过 P4，再做 P5 active |
| 长时间使用错误的“global baseline”目录 | 多轮比较和因果判断建立在错误参考上 | baseline 必须由 hash、run spec、配置快照和轨迹共同识别 |
| 面对已有全程 FC/GPS 大表却声称没有真值/到处找零散文件 | 重复劳动，遗漏完整量测 | 首先读取 master table 与 registry，不从截断初始化表推断全局数据不存在 |
| 没有 Hessian/covariance/激励证据就宣称 q/bg/ba 不可观 | 错误否定原始联合初始化目标 | “不可观”必须绑定状态定义、窗口、Jacobian rank、covariance 与激励证据 |
| 提议用 FC/GPS yaw 消除生产漂移 | 违反运行时输入合同，等于绕开 VIO 问题 | 只能作离线正对照，不得进入正式在线方案 |
| 仅凭 1D AGL 高度提出全局 7DoF Sim3 | 数学自由度错误，并破坏惯性状态一致性 | 高度只约束垂直/尺度的明确模型，不得冒充全局 Sim3 |
| 没先实现用户指定的左转取右、右转取左，就跑了模糊 ROI 实验 | 实验没有回答用户问题 | 每个实验先写 intervention、预期分支和通过/拒绝动作 |
| 把 standalone test、shadow、非空轨迹、网页打开称为算法证据 | 以次充好 | 明确区分 compile/test/shadow/run/evaluation/acceptance 六层证据 |
| dashboard 缺 `window.RUN_DATA`、误差曲线、有效四段分段且大量 N/A 时仍称验收通过 | 用户看到不可用页面，正式评价信息丢失 | dashboard 必须有整体误差曲线和一圈 2 直线+2 转弯，N/A 必须先修数据合同 |
| provenance/hash/preflight 设计过重并阻塞主实验 | 时间消耗在流程而非估计器正确性 | provenance 只保留最小 commit/config/command/input/output receipt |
| 曾坚持 `-j1` 串行构建并反复重建 | 无谓拖慢，掩盖单翻译单元膨胀问题 | 默认 `-j8/-j12`；只在可证明的竞争/污染问题上串行 |
| 没有从一开始维护统一实验总账 | 重复跑眼熟实验，无法及时回答“结论是什么” | 每次运行前登记问题，结束即登记指标、结论和下一动作 |
| 用少量参数/两行补丁冒充“完整重构” | 组合开关膨胀、语义互相污染 | 新正式 P4 建独立清晰模块，禁止继续在旧 candidate 分支叠 flag |
| 混用 absolute、start-heading、best-fit、错误基准并没有解释 single-row | 指标不可比，用户无法判断实际优劣 | 所有表格写明数据身份、对齐方式、采样网格和 control 含义 |
| 已运行结束却继续重跑或迟迟不出结论 | 浪费数小时并损害信任 | 先解析已有 receipt/trajectory；只有缺失指定证据才重跑 |
| 不尊重 joint state/covariance 直接强制 q/p/v 或重复注入 history | 首次后端大修正、信息双计数、短好长坏 | 注入必须原子化且保持 cross-covariance 与 FEJ 语义 |
| 从相关现象直接宣布根因，缺少针对性 intervention | 结论不可用于修复 | 怀疑项必须对应可区分因果分支的消融和跨飞行 holdout |

## 9. 下一位的执行顺序

1. 不再调当前 candidate gate、固定次数、噪声或 release threshold。
2. 先读本交接、P4 master record、新文献审查、新正式实现规格和 K/D verdict。
3. 从当前 runner 画出实际启用路径，明确哪些实验 flag 必须从新正式路径隔离。
4. 按正式规格新建清晰的 P4 graph builder/solver/result injector；优先复用 upstream `DynamicInitializer`、`Factor_ImuCPIv1`、CPI 和 `StateHelper` 的正确状态/协方差语义，不复制整段旧实验分支。
5. 先做 synthetic：坐标系、时间偏移、安装角方向、15D residual、shared bias、covariance、原子注入、首个 backend innovation。
6. 用同运行时 single-row 作为隔离对照，但正式比较仍必须包含冻结 global baseline。
7. focus 只用于开发：fly1 首转、fly3 首反向转；fly2 必须作为 holdout。通过后跑 fly1/fly2/fly3 full。
8. 正式评价只用 start-heading/absolute corroboration，不用 best-fit 掩盖初始化误差。
9. 只有 P4 全程通过后才继续 P5 backend event trigger 与 active takeover。
10. P5 必须与固定 cadence 比较精度、KLT 次数、backend 次数、运行时间和状态分布；shadow 无收益结论不能算完成。

## 10. 验收底线

P4 至少同时满足：

- fly1/fly2/fly3 均产生完整、非空、退出状态明确的正式轨迹；
- 初始化时间、q/p/v/bg/ba、covariance、residual、innovation 和首个 backend correction 均可审计；
- 相同运行时、相同评价口径下，不能系统性劣于 single-row，更不能劣于冻结 global baseline；
- 左转/右转结果不能依赖固定符号补偿；
- 不使用未来 GPS/FC yaw 调门限；
- 不发生视觉双计数或 handoff 后巨幅状态修正。

P5 至少同时满足：

- 真正 active，实际改变 tracking/backend 工作，而非 shadow 推荐；
- tracking 是连续 horizon 推导，backend 是 event-driven；
- P4 初始化前缀与 P5 off 精确一致；
- fly1/fly2/fly3/fly4 中计算量有可测下降，精度退化受明确预算约束；
- dashboard、CSV、JSON 和 metadata 能说明每次决策原因与实际执行事件。

在这些条件满足前，任何“完成”“可用”“通过验收”都是错误表述。
