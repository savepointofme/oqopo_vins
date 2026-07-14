# P4 全程 FC–Board 标定与持久闭环初始化技术合同

## 1. 状态与结论

本文是 P4 重构的实现前合同。旧的固定 12--20 s 候选验证、局部全窗口 replay、末端 permission 追溯到窗口开头的实现全部撤回。

P4 分成两个边界清楚的阶段：

1. 离线使用 global baseline 全程数据估计并验收固定 FC–board 安装旋转和 attitude-channel 时间残差；
2. 在线锁定上述标定，建立持久的 15 维 error-state alignment filter，用因果 FC `p/v` 更新闭环修正 `q/p/v/bg/ba`，再按分组可观性和收敛结果决定反馈与 release tier。

`PV_ONLY` 只能是内部阶段。正常 P4 release 至少要求 `q/p/v` 已被数据支持、收敛并实际反馈；`bg/ba` 可分别进入 data-estimated 或 prior-retained tier，不能把未收敛 bias 写成已标定。

参考代码的准确语义、缺陷和移植边界见 `P4_MOVE_BASELINE_REFERENCE_AUDIT_20260714.md`。

## 2. 离线 FC–Board 标定合同

### 2.1 固定量

P4 在线阶段只读：

```text
R_FtoI_cal
dt_FI_att
dt_FI_nav
```

其中：

```text
omega_I(t + dt_FI_att) ~= R_FtoI_cal * omega_F(t) + bg
t_FC_att = t_board - dt_FI_att
t_FC_nav = t_board - dt_FI_nav
```

`dt_FI_att` 来自全程 FC attitude/rate 与 board gyro 标定。gyro correlation 不能证明 position/velocity channel 具有相同延迟；没有独立导航通道时延证据时，`dt_FI_nav` 保持输入数据声明的 camera-time/navigation-time 合同，不从 `dt_FI_att` 复制。

### 2.2 允许与禁止输入

标定允许使用 raw FC timestamp/attitude、board IMU timestamp/gyro、坐标轴声明和输入 provenance。VIO pose、GPS horizontal/course、truth/error、事后轨迹误差只能用于评价，不能进入标定目标函数或在线 gate。

### 2.3 验收

标定 artifact 必须记录输入 hash、时间范围、坐标方向、时间符号、搜索范围、鲁棒残差、三轴激励、前后半程稳定性和不确定度。转弯魔术贴 flex 作为短暂评价基准偏移单独记录，不在线追随，也不覆盖固定安装中心。

## 3. 在线状态与误差定义

持久 nominal：

```text
x_nom = {R_GtoI, p_IinG, v_IinG, bg, ba}
```

持久 15 维 error state：

```text
dx = [dtheta, dp, dv, dbg, dba]
P  = covariance(dx)
```

P4 采用当前 OpenVINS/JPL 误差约定：

```text
R_true = Exp(-dtheta) * R_nom
R_nom  <- Exp(-applied_dtheta) * R_nom
p/v/bg/ba_nom <- nominal + applied_delta
dx <- G(applied_delta) * (dx - applied_delta)
P  <- G(applied_delta) * P * G(applied_delta)'
```

其中 additive-only feedback 的 `G=I`；姿态反馈时：

```text
G_theta = I - 0.5 * skew(applied_dtheta)
```

该 reset 必须作用于完整 `dx/P`，包括 cross-covariance，不能只改 attitude 的 3x3 对角块。参考 MATLAB/ALIGN19 没有可直接复制的 reset；符号和 Jacobian 必须由 P4 自身正负扰动测试确认。

## 4. 持久运行时

`OnlineAlignmentInitializer` 内新增唯一的 candidate runtime：

```text
CandidateFilterRuntime
  active
  board_time
  nominal
  dx[15]
  P[15x15]
  last_processed_imu_timestamp
  last_processed_fc_timestamp
  accepted/rejected counts by measurement and state group
  rolling convergence history by state group/axis
  cumulative feedback and clipping provenance
  current feedback tier and release tier
```

候选生成时只初始化一次。之后每次 `try_initialize()` 只消费尚未处理的数据；相同 timestamp 重复调用必须幂等。不得再创建局部 `FeedbackReplay`，不得从候选起点反复重放，也不得用区间末端结果追溯修改早期 feedback permission。

buffer 只保留当前插值边界、尚未处理的数据和视觉健康检查所需的有限历史，不再为全候选 replay 永久保留起点。

## 5. 每个历元的闭环顺序

```text
unprocessed board IMU
  -> propagate nominal with current bg/ba
  -> propagate dx and P

supported FC p/v measurement
  -> quality and innovation/NIS gate
  -> innovation = z - h(nominal) - H*dx
  -> update dx and P with full-K Joseph form
  -> update state-group information/convergence histories
  -> apply feedback only to currently permitted groups
  -> subtract applied correction from dx
  -> apply attitude covariance reset when attitude is injected
  -> continue next IMU interval from corrected nominal and bias
```

measurement update 与 feedback permission 必须分开：

- 允许估计的 q/p/v/bg/ba 都可通过完整 H/P/K 耦合积累 `dx` 和 covariance 信息；
- feedback mask 只决定哪些已估计误差可以注入 nominal；
- 禁止为了“冻结反馈”直接把 K 对应行清零，因为这会同时阻止误差历史和可观性形成；
- 引入持久 `dx` 后，innovation 必须扣除 `H*dx`；继续使用 `z-h(nominal)` 会重复估计尚未反馈的同一误差；
- additive p/v feedback 不改变其误差坐标；姿态 feedback 必须 reset covariance；bias feedback 必须改变后续 IMU compensation。

robust measurement 若使用放大的 effective R，Joseph covariance 右项必须使用同一个 effective R。

## 6. 分组反馈

### 6.1 `p/v`

FC `p/v` 通过质量与 innovation/NIS gate 后可直接反馈，建立导航平移和速度闭环。每次 correction、NIS、accepted/rejected 原因都记录。

### 6.2 attitude

attitude 不直接使用候选期 FC attitude residual 追随魔术贴 flex。它由 IMU error dynamics、FC `p/v` 运动约束、初始图信息和 covariance 交叉项形成。只有 attitude 组达到第 7 节条件后才反馈；正常 release 必须至少发生一次有界 attitude feedback，并在后续 accepted updates 中保持稳定。

### 6.3 `bg/ba`

`bg` 与 `ba` 独立判断，必要时按轴判断。满足条件的组才注入 nominal 并进入后续 IMU；未满足的组保留在 `dx` 中继续估计。release 时仍未通过的 bias：

- provenance 标记为 `prior_retained` 或 `weakly_observed`；
- 不进入 `estimated_state_list`；
- covariance 不得使用已估计 bias 的收紧结果冒充可信度；
- retained `dx_bg/dx_ba` 不得静默丢弃；practical release 若保留 prior nominal，bias block 至少使用 `initial_P + dx*dx'` 的保守 MSE，并清除其与已释放状态的 cross-covariance；retained correction 超过安全界时不得 release；
- practical policy 可产生 bias-partial 的 `NAVIGATION_READY`；strict policy 不得释放。

## 7. 可观性、收敛与反馈许可

每个状态组或轴维护：

- 对该组有实际信息增益的 accepted update 数；
- 当前 pre-feedback `abs(dx)`；
- 当前 `sqrt(diag(P))`；
- 相邻 standard deviation 变化；
- rolling `dx` peak-to-peak；
- rolling std peak-to-peak；
- innovation/NIS acceptance；
- correction clipping 与累计 correction；
- 初始联合图中去除 prior 后的数据支持。

一个 update 只有在该组 Kalman support/information gain 非退化时才给该组累计 accepted count；height-only、无关量测和 rejected measurement 不得给其他组记数。

反馈条件按组配置，但配置值必须有单位和数据来源。旧 MATLAB 的 240 s 阈值、硬编码 50 点历史和 `gi_engine` duration fallback 均不得直接复制。candidate age 只作诊断或 fail/retry 上限，不能作为成功条件。

反馈后只清对应 convergence flag；滚动历史和累计 accepted count 保留，下一次 accepted update 在更新后的历史上重新判断。若 correction 打破稳定性，应自然暂停该组后续反馈，而不是回放历史或强行保持已开放状态。

## 8. release 合同

### 8.1 正常 navigation release

必须同时满足：

- solver/initial graph 可用，初始 q/p/v/P 有限；
- 持久闭环实际执行了多次 accepted FC `p/v` update；
- q/p/v 均通过数据支持、covariance、误差历史、correction 和 clipping gate；
- attitude 已反馈并在后续更新中保持稳定；
- 当前 nominal、dx、P 和 metadata 来自同一个 persistent runtime；
- visual 轨迹/重投影只作为初始图贡献和独立健康 gate，除非它真正进入递归 measurement update；
- practical/strict policy 与 bias tier 一致。

`PV_ONLY` 不得产生正常 release。timeout 只能 fail/retry 或明确 degraded，不能产生正常 success。

### 8.2 readiness

```text
NAVIGATION_READY
  q/p/v closed-loop converged and fed back
  bg and/or ba may retain honest prior/weak status

FULL_ALIGNMENT_READY
  q/p/v/bg/ba all satisfy the defined data-supported gates
```

释放状态必须取 persistent runtime 的当前 nominal 和 P，不能取静态 `candidate_result_`，也不能另做 terminal FC snap。provisional output 同样读取该 runtime，再只传播尚未处理的尾段。

## 9. FC attitude 与 flex 边界

候选验证阶段的 FC attitude residual 是评价量，不是递归 attitude measurement。前置 Ceres 当前仍含 FC attitude factor，因此必须明确其 robust weighting 与高动态/flex 行为，并补“flex 出现在候选创建前/初始图窗口内”的测试；只在 candidate 创建后注入 flex 不能证明整个 P4 不追随 FC attitude。

若该测试不能证明初始图对 turn-flex 鲁棒，应在初始图中对相应 FC attitude factor 做因果高动态降权或剔除，而不是事后改输出坐标。

## 10. 验证与完成定义

实现前先封存当前 global baseline 的源码、二进制、配置、FC stream、GPS、dataset manifest、命令、退出状态和 unit-test 状态。P4 已声明输出在 `G_nav`，初始化绝对误差验收使用 `traj_nav.txt` 和 `absolute_navigation_no_post_alignment`；start-heading/best-fit 只能另列诊断，不能掩盖初始化错位。

### 10.1 2026-07-14 无改动诊断基线

无改动版本在 fly1 的 provenance capture 完成后进入可视化全航程回放。候选约在 camera time `938.38 s` 建立；该时刻相对锁定全程 FC-board 标定参考的姿态差约 `2.13 deg`，位置差约 `1.2 m`，速度差约 `0.9 m/s`，说明候选初值本身不是后续数十公里发散的来源。

随后实现持续返回 `candidate_validation_common_horizon`，没有消费候选期 FC `p/v` 闭环量测，`traj_nav.txt` 始终为空。到 camera time `1656.31 s` 仍未释放，provisional 已仅靠 board IMU 从候选起点开环传播并严重发散。与此同时，每个 camera call 从候选起点重放全部历史，运行成本随窗口增长，已表现为二次复杂度。该 run 因终止性设计故障被停止并保留为 `nochange_diagnostic_preflight_failed`，不生成伪造的正式轨迹指标；候选版必须用相同 global-baseline 输入跑出非空正式 `traj_nav.txt` 才有资格进入绝对 `G_nav` 误差评价。

最小单元测试：

1. q/p/v/bg/ba 正负 correction 均使对应 residual 下降；
2. 第一次 update 后的 nominal/dx/P 被下一次调用继承；
3. 相同 timestamp 重复调用不重复处理；
4. bg/ba feedback 改变下一段 IMU propagation；
5. estimation mask 与 feedback mask 独立；
6. 每组收敛真值表、滚动历史和反馈后重新判断正确；
7. rejected/无关 measurement 不累计该组 accepted count；
8. p/v 收敛但 attitude 未收敛时不得 release；
9. bias partial 时 readiness、provenance、estimated-state list 和 covariance 一致；
10. strict policy 拒绝 bias-partial 状态；
11. 每次 Joseph/reset 后 P 对称、有限、PSD，不能依赖无条件 eigenvalue floor 掩盖错误；
12. turn flex 在初始图前和候选期注入时均不造成错误姿态追随；
13. 同一历元多次 update 的 correction/clipping provenance 累计完整。

完成条件：相关测试与 runner 构建通过；global baseline fly1/fly3 可视化全程成功；同输入、同绝对 `G_nav` 评价下，候选初始化 q/p/v 和全程指标不得比封存 no-change 结果退化。global baseline 通过前不进入 June12。
