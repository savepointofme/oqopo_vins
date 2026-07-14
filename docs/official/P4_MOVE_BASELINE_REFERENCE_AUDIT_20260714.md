# P4 对 move-baseline 对准代码的端到端参考审计

## 1. 结论

这次先不改 P4 估计器。源码审计得到的结论是：

1. move-baseline 不是一套可以概括为“每次量测都反馈全部状态”的统一实现。`CorseAlign.m` 每 0.2 s 无条件做一次 KF 更新，随后按轴检查收敛并只反馈满足条件且配置允许的状态；`3DownFilter20250504.m` 在量测通过门控后采用同类条件反馈；formal ALIGN19 C++ 则在每个 accepted heading/position/velocity update 后无条件反馈全部 19 维误差状态；
2. 三条实现共同支持的结论是：闭环修正必须改变后续名义传播，而不是窗口末端只改一次输出；收敛或 release 不能由 2 s、12 s 或 20 s 本身代替；
3. 三套判据必须分开：旧 MATLAB 检查当前误差估计、当前标准差以及部分状态的标准差变化和估计值峰峰值；`gi_engine.cpp` 的 staged-feedback 路径检查标准差、相邻标准差变化和标准差窗口峰峰值；formal ALIGN19 使用独立 supervisor 的当前标准差、残差、修正量、clipping、连续帧数和 hold；
4. 旧配置允许 `p/v/attitude/bg/ba` 分状态修正，但给定配置 `111111111000000` 实际只开启位置、速度和姿态反馈，关闭 `bg/ba` 反馈。代码具备 bias 反馈能力不等于这份配置实际用了 bias 反馈；
5. 当前 P4 已有 Joseph covariance update、姿态误差注入和 reset Jacobian，但候选阶段写死 12--20 s，并在每次验证时从初始候选重新重放完整区间。这不是持久逐历元闭环；参考实现本身没有显式姿态 covariance reset，因此 P4 的 reset 只能依据自身误差定义保留并单独验证，不能宣称是直接移植；
6. 当前 P4 的递归闭环只有 FC `p/v` 量测，`q/bg/ba` 依靠传播产生的交叉协方差间接修正；闭环视觉更新计数被固定为 0。因而不能把它描述成“FC+IMU+visual 的逐历元闭环修正”；
7. 之前写死 12--20 s 的规格和相应实现草稿没有依据，已经冻结；global baseline 的无改动结果和完整参考移植验证完成前，不进入 June12 数据。

## 2. 实际审计的参考层次

### 2.1 原始 MATLAB 实现

主参考：

- `C:/Users/baloney/xwechat_files/wxid_60q5yd762sa352_8a98/msg/file/2026-06/20250211/20250211/CorseAlign.m`
- 同目录 `配置文件-精对准.txt`
- `C:/Users/baloney/xwechat_files/wxid_60q5yd762sa352_8a98/msg/file/2026-06/20250506备份/3DownFilter20250504.m`

这些文件给出了实际滤波循环、收敛判断、反馈限幅和误差清除。两份 MATLAB 的 bias 闭环并不相同：`CorseAlign.m` 对准主循环没有把累计 `bg/ba` 传给下一次 IMU 解算，`3DownFilter20250504.m` 才在每个 IMU 历元把累计 bias 用于组合导航支路。

### 2.2 formal ALIGN19 C++ 实现

辅助参考：

- `D:/vscode_dir/px4_ekf2_moving_base_alignment/workspaces/KF-GINS_ALIGN19_LT/src/kf-gins/alignment/state_layout.h`
- `.../alignment/error_model_align19_lt.cpp`
- `.../alignment/gnss_position_adapter_align19_lt.cpp`
- `.../alignment/gnss_velocity_adapter_align19_lt.cpp`
- `.../alignment/feedback_align19_lt.cpp`
- `.../alignment/alignment_engine_align19_lt.cpp`
- `.../alignment/alignment_supervisor.cpp`

formal ALIGN19 在每个 accepted measurement 后反馈全部误差并清零 `dx`，`bg/ba` 随后补偿 IMU；release 由独立 supervisor 决定。其 19 维 `lever/time_delay` 状态、GNSS 模型和无条件全状态反馈不等同于 P4，不能整体照搬。

### 2.3 `gi_engine.cpp` staged-feedback 路径

`gi_engine.cpp:430-525` 的三项历史稳定门控属于另一条 staged-feedback 实现，不是 formal ALIGN19 supervisor。该路径的阶段推进仍允许 duration fallback（`gi_engine.cpp:714-733`），因此只能参考统计量结构，不能把它写成 formal ALIGN19 的 release 机制，也不能照搬 timeout 语义。

### 2.4 不能作为滤波器主体的代码

`tools/combnav_alignment_v2/staged_p_gate.py` 只生成 stage permission，不执行滤波、状态反馈或 covariance reset。`p_gated_online_filter_17d_v1.py` 是历史原型，包含 oracle 输入和 timeout force-release 路径。它们可解释阶段控制，但不能作为 P4 主实现直接移植。

## 3. 原始代码到底怎么运行

`CorseAlign.m` 的 17 维误差状态为：

```text
[phi_N, phi_U, phi_E,
 V_N, V_U, V_E,
 Lat, Height, Lon,
 bg_x, bg_y, bg_z,
 ba_x, ba_y, ba_z,
 install_1, install_2]
```

`CorseAlign.m` 的对准主循环实际顺序是：

```text
IMU mechanization with zero correction argument
  -> X = A X
  -> P = A P A' + Q
  -> every 40 IMU samples form a navigation residual
  -> K = P H' (H P H' + R)^-1
  -> X = X + K (Z - H X)
  -> Joseph covariance update
  -> append X and diag(P) to history
  -> judgeConvergence()
  -> modify() conditionally feeds back eligible axes
  -> corrected q/p/v enter the next nominal propagation
  -> X subtracts only the applied correction
```

对应源码位置：

- 对准期 IMU 调用显式传入全零 correction：`CorseAlign.m:200-202`；
- 每 40 个 5 ms IMU 样本无条件进入一次 KF 更新，没有 measurement accept/reject gate：`CorseAlign.m:210-244`；
- 量测更新和 Joseph form：`CorseAlign.m:242-243`；
- 历史维护：`CorseAlign.m:246-249`；
- 收敛判断与条件反馈检查：`CorseAlign.m:251-262`；
- 反馈函数：`CorseAlign.m:670-906`。

`CorseAlign.m` 的 `insNavigation()` 内部具备按 `sumModify` 扣除 bias 的代码（441--457 行），但对准主循环没有传入它；只有 240 s 对准结束后的纯惯导段在 349 行传入 `sumModify`。因此不能据此宣称 Corse 对准期完成了 `bg/ba` 持久闭环。

`3DownFilter20250504.m` 才是 MATLAB 中实际的 bias 闭环支路：101 行每个 IMU 历元把 `sumModify` 传给组合导航，213--239 行在量测通过质量/卡方门控后更新，220--235 行维护历史、判断收敛并条件反馈。这里的 bias 修改会影响后续 IMU 传播。

formal ALIGN19 是第三种语义：`alignment_engine_align19_lt.cpp:266-275` 在每个 accepted heading、position 或 velocity update 后调用 `applyFeedbackAlign19Lt()`，`feedback_align19_lt.cpp:40-93` 注入全部 19 维误差并清零 `dx`，`alignment_engine_align19_lt.cpp:135-137` 用更新后的 `bg/ba` 补偿后续 IMU。

这说明参考实现的“闭环”是滤波器运行方式，不是 release 前后的验证名字，也不是窗口末端的状态 snap。

## 4. 收敛条件的准确含义

### 4.1 原始 MATLAB 判据

`CorseAlign.m:953-988` 的 `judgeConvergenceSingle()` 没有“均值低于阈值”这一项。准确条件如下：

| 状态组 | 有效更新数 | 当前 `abs(X_end)` | 当前 `sqrt(P_end)` | 相邻标准差变化 | `X` 窗口峰峰值 |
| --- | --- | --- | --- | --- | --- |
| position / velocity | 要求 | 要求 | 要求 | 不要求 | 不要求 |
| horizontal attitude / heading | 要求 | 要求 | 要求 | 要求 | 要求 |
| gyro bias / accel bias | 要求 | 要求 | 要求 | 要求 | 要求 |

因此用户记得的“峰峰值、变化小、幅值小”来自这一组条件，但源码中的幅值是当前误差估计绝对值，不是窗口均值。

反馈不是只发生一次。某轴满足收敛、`Ctrl.Revise` 允许且达到反馈周期后，该轴才被限幅反馈。反馈后只清该轴的 `flagConvergence`；`counterKalman`、`XkPeakPeak` 和 `PkPeakPeak` 都不清零。下一次更新会在保留的滚动历史上重新判断，新注入的估计值可能暂时打破峰峰值条件，但这不是“从零重新积累全部证据”。

### 4.2 后续 C++ 的三项稳定判据

`gi_engine.cpp:430-525` 对 position、velocity、attitude、gyro bias、accel bias 分组计算 `P` 对角线标准差的最大值，并要求以下三项连续满足指定帧数：

1. 当前最大标准差低于阈值；
2. 当前与上一拍最大标准差之差低于阈值；
3. 当前稳定窗口内最大标准差的峰峰值低于阈值。

这是用户所说“三种稳定条件”在 `gi_engine.cpp` staged-feedback 路径中的准确实现。它只看标准差稳定性；原始 MATLAB 还看误差估计 `X` 的幅值和峰峰值。该函数在当前 GNSS update 之前求值，随后阶段推进还允许 duration fallback，因此它不是一套可以直接复制的完整 release 判据。

formal ALIGN19 不使用上述三项历史门控。它在每个 accepted update 后已反馈全部状态，随后由 `alignment_supervisor.cpp` 检查当前 std、残差、修正量、clipping、`required_frames` 和 hold。其 hold 从历史首次 full-gate pass 计时；gate 中断只清连续帧计数，不重置首次通过时间，而且 `test_alignment_supervisor.cpp:77-92` 明确保护这一语义。P4 必须自行决定是否需要真正的连续稳定时长，不能误称 formal ALIGN19 已经这样做。

### 4.3 原始配置的实际含义

`配置文件-精对准.txt` 中：

- position：5 次有效更新，标准差 50 m，估计值 5000 m；
- velocity：5 次有效更新，标准差 0.5 m/s，估计值 15 m/s；
- horizontal attitude：10 次，标准差 0.05 deg，相邻变化 0.01 deg，估计值 5 deg，估计值峰峰值 0.05 deg；
- heading：900 次，标准差 0.3 deg，相邻变化 0.01 deg，估计值 10 deg，峰峰值 0.05 deg；
- gyro bias：900 次，标准差 0.3 deg/h，相邻变化 0.05 deg/h，估计值 15 deg/h，峰峰值 0.2 deg/h；
- accel bias：900 次，标准差 80 ug，相邻变化 30 ug，估计值 10000 ug，峰峰值 30 ug。

这些数值来自另一套传感器、量测频率和 240 s 对准任务，不能直接复制给 P4。可直接复用的是判断结构和反馈语义，P4 数值必须按本系统单位、更新率、噪声模型和 global baseline 无改动统计重新验收。

## 5. 反馈和 reset 的准确语义

旧 MATLAB 对每一轴在满足收敛、配置开关和反馈周期时执行：

```text
correction = clamp(X_axis, revise_limit)
nominal <- nominal corrected by correction
X_axis <- X_axis - correction
convergence_flag_axis <- false
```

位置、速度和姿态反馈到名义导航状态。`bg/ba` 被允许时累加到 `sumModify`，但只有 `3DownFilter20250504.m` 在滤波运行期间把它传给后续 IMU；`CorseAlign.m` 的对准期没有这样做，而且给定 `111111111000000` 配置本来就关闭了 bias 反馈。

formal ALIGN19 C++ 在每个 accepted measurement 后调用 `applyFeedbackAlign19Lt()`：修正 `p/v/q/bg/ba/lever/time_delay`，然后 `state->dx.setZero()`。它不是“先等某组收敛再反馈该组”的实现；分阶段只用于 supervisor/release。timeout 最多给 `DEGRADED_RELEASE`，不能伪装成 COMPLETE。

需要注意以下不能盲抄的缺陷和边界：

1. 两份 MATLAB 和 formal ALIGN19 都没有显式姿态 covariance reset Jacobian；P4 的 reset 必须按自身 error-state 约定推导，并补姿态符号、PSD 和一致性测试；现有 `test_align19_feedback_sign.cpp` 没有设置 `PHI`，也没有测试 covariance reset；
2. formal ALIGN19 robust 路径用放大后的 `R` 重算 `S/K`，但 Joseph 右项仍使用原始 `R`；这会造成 covariance 不一致，P4 不得照搬；
3. formal ALIGN19 同一 GNSS 历元可连续做 heading、position、velocity 三次反馈，但 supervisor 只看到最后一次 `feedback`，可能漏掉前一次较大的 correction；P4 必须累计同历元 correction provenance；
4. formal supervisor 的 hold 不是“当前连续通过时长”，而是“历史首次通过至今的时长 + 当前连续帧数”；这是受测试保护的现有语义，不应误写成偶然 bug；
5. 两份 MATLAB 的 accel feedback 限幅都误用了 `Gyro.ReviseTh`；`3DownFilter20250504.m:417` 的 GyroY 又错误读取 `Xk(10)` 而非 `Xk(11)`；
6. 两份 MATLAB 的姿态反馈在零修正时可能计算 `sin(datt0/2)/datt0`，存在 `0/0`；
7. MATLAB 虽读取 `PeakNum`，历史宽度仍硬编码为 50；`PkPeakPeak` 还用零 `Xk` 初始化，而不是初始 `diag(P)`；
8. 3Down 的 16:17 安装角进入 H/P/Q，但不在 15 维 feedback 中，也没有注入 nominal；同时 208--211 行可能在视觉无效或仅高度有效时仍覆盖 H 的前两行，形成安装角/姿态伪量测风险；
9. 3Down 用全局 update counter 判断多类状态收敛，height-only update 也可能给视觉相关状态累计“有效更新数”。P4 必须按实际影响该状态组的 accepted measurement 计数。

## 6. 当前 P4 与参考实现的逐项差异

| 机制 | 可借鉴的参考语义 | 当前 P4 草稿 | 判定 |
| --- | --- | --- | --- |
| 反馈时机 | MATLAB：每次更新后检查、按组条件反馈；formal ALIGN19：每个 accepted update 后全状态反馈 | 验证时从候选起点重放整个区间 | 必须改为持久逐历元状态；P4 应采用符合用户要求的分组条件反馈 |
| 后续传播 | q/p/v 修正进入下一历元；3Down/formal ALIGN19 的 bias 也进入下一段 IMU | 单次 replay 内满足；不同验证调用不保留 replay 状态 | 只实现了窗口内模拟闭环 |
| 误差状态 | MATLAB/formal ALIGN19 都持有可传播、可更新、可反馈清除的 `X/dx` | 直接计算 `delta=K innovation` 并立即注入，没有持久 `dx` | 无法让未获反馈许可的 q/bg/ba 继续积累 |
| 估计与反馈许可 | 先估计误差，再决定哪些组可注入 | 用 `active_state_mask` 直接把 K 对应行清零 | 混淆 estimation mask 与 feedback mask |
| 收敛依据 | `X`、`sqrt(P)`、相邻变化、峰峰值、有效更新数，或 formal supervisor 的当前 gate | correction / sigma 小于 0.35 并保持 3 s | 小 correction 也可能只是交叉协方差弱，不能代表已观测/收敛 |
| 时间条件 | 更新次数和历史宽度；满足即反馈 | 至少 12 s，20 s 拒绝 | 固定 12--20 s 撤回 |
| p/v 更新 | 多次量测更新 | 多次 FC p/v 更新 | 机制可保留 |
| attitude/bg/ba | 由误差传播和 H/P 耦合估计，按轴/组收敛后反馈 | 由 p/v 更新的交叉协方差间接修正，按初始 observability + normalized correction 放行 | 需要参考式历史 gate |
| 因果性 | 当前历元只使用截至当前的许可 | 先用 shadow replay 的末端稳定结果生成 mask，再从起点用该最终 mask 重放 | 把未来许可追溯应用到早期历元，必须删除 |
| visual 闭环 | 3Down 中是实际量测路径 | P4 replay 中 `visual_updates=0`，只在前置 Ceres 和 release residual 中使用 | 不能宣称视觉参与递归闭环 |
| covariance | Joseph update；旧代码无显式姿态 reset Jacobian | Joseph update + attitude reset Jacobian | 保留，但补符号/PSD 测试 |
| mount/time | 不属于这套 15D 闭环主体 | 已锁定为 full-flight external calibration | 继续锁定，不在启动窗重估 |
| release 声明 | 反馈/收敛结果必须与状态和 covariance 一致 | readiness 主要按 bias permission 切换，observability/provenance 仍沿用初始 Ceres 结论，strict policy 未在最终处重新核验 | 声明、实际 feedback tier 和 covariance 可能不一致 |
| timeout | formal ALIGN19 只允许 degraded；`gi_engine` staged 路径却存在 duration fallback | 20 s 触发 candidate rejection，不会直接成功 | P4 正常成功不得靠 timeout，固定时长仍无依据 |
| FC attitude | 自对准配置关闭 heading 装订；主要靠运动约束 | release 验证称 evaluation-only，但前置 Ceres 仍含 FC attitude factor | 文档和实现语义冲突 |

当前 `ClosedLoopState` 和 `FeedbackReplay` 都是 `validate_candidate()` 的局部对象。前 12 s 直接返回；达到门限后从静态 `candidate_result_` 和初始 covariance 重放全部历史；下一次验证又从头开始。更严重的是 shadow replay 先看完整区间末端，再用末端得到的 `release_mask` 从候选起点重放，形成非因果的追溯反馈。

因此问题不在于把 12 s 改成另一个数字，也不在于继续调整 normalized-correction 阈值。必须删除局部全窗口 replay，新增持久 nominal、`dx`、`P`、时间游标、按状态组计数和反馈历史；每条 IMU/FC 数据只能处理一次。

## 7. 源码到 P4 的移植映射

| 参考机制 | 参考位置 | P4 目标 | 处理方式 |
| --- | --- | --- | --- |
| 逐 IMU 误差传播 | `CorseAlign.m:213-214`；`error_model_align19_lt.cpp` | `OnlineAlignmentInitializer` 的持久 15D alignment state | 按 P4 坐标和误差定义适配 |
| measurement update 与 Joseph covariance | `CorseAlign.m:241-243`；`3DownFilter20250504.m:213-217`；formal `ekfUpdate()` | 把当前 `apply_closed_loop_update()` 拆成只更新 `dx/P` 的函数 | 复用数学结构；robust 时必须在 Joseph 项使用实际 effective R |
| 更新后检查分组反馈 | `CorseAlign.m:251-262`；3Down 220--235 行 | 每个 accepted P4 measurement 后更新各组 gate | 适配；Corse 的无门控更新周期不能照搬 |
| feedback 后清已注入误差 | MATLAB `modify()`；formal `state->dx.setZero()` | `apply_candidate_feedback(mask)` | P4 采用部分 mask：`dx -= applied`，不是 formal ALIGN19 的全量清零 |
| q/p/v 修正进入后续传播 | 两份 MATLAB `modify()`；formal feedback | 持久 nominal state | 直接复用闭环语义，按 P4 误差符号实现 |
| bias 进入后续 IMU | `3DownFilter20250504.m:101`；formal `compensateImu():135-137` | board IMU propagation 使用修正后 `bg/ba` | 适配；不得再引用 Corse 对准期作为该机制依据 |
| 分状态/分轴收敛 | `judgeConvergenceSingle()` | q/p/v/bg/ba group/axis history | 结构移植，单位和阈值适配 |
| 三项 std 稳定 gate | `gi_engine.cpp:430-525` | 每个 P4 状态组的 covariance history | 只复用统计量；不复制 duration fallback，也不冒充 formal ALIGN19 supervisor |
| 量测有效性和卡方门控 | `3DownFilter20250504.m:126-235` | FC p/v 与视觉 innovation acceptance | 按 P4 量测模型适配 |
| feedback 限幅和 clipping provenance | `modify()`；formal feedback | per-update、同历元累计、total correction log | 结构移植；clipping 禁止作为收敛，不能只记录最后一次 update |
| timeout 不等于 normal complete | formal `alignment_supervisor.cpp` | P4 release supervisor | 复用语义；不复制其历史首次 pass hold 语义，P4 要求当前连续稳定 |
| install / lever / time-delay 在线状态 | legacy 16:17；ALIGN19 15:18 | P4 | 不适用；P4 的 FC-board mount/time 来自离线 full-flight calibration |
| 固定 240 s 和旧数值阈值 | legacy config | P4 | 不直接移植 |

## 8. 由参考实现约束出的 P4 设计边界

以下只是审计结论，不是已经完成的代码：

1. 离线阶段先用 global baseline 全程数据估计并验收固定 FC-board 安装角和 attitude-channel 时间残差；转弯魔术贴 flex 单独标记为评价基准瞬态，不让在线导航追随；
2. 在线联合图优化仍可提供初始 `q/p/v/bg/ba/P`，但其后应建立一个持久的 causal alignment state：nominal、15D `dx`、15x15 `P`、当前时刻、数据游标、按状态组 accepted/rejected 计数、历史和累计 feedback；每条 IMU 和每个量测只处理一次；
3. 每个 accepted FC p/v 或可实现的 visual update 先更新 `dx/P`，再评估每个状态组的反馈许可，只注入当前已收敛的组；未获许可的 q/bg/ba 继续保留在 `dx` 中积累；下一段 IMU 使用已注入的 q/bg/ba；
4. `PV_ONLY` 只能是内部中间阶段，不能称为完成对准。P4 正常 release 至少要求 `q/p/v` 收敛；`bg` 和 `ba` 分组、必要时分轴判断，满足条件才反馈和声明为本次数据估计；
5. 未收敛的 bias 可以保持带 provenance 的 prior 并扩大相应 release covariance，但不得在报告中写成已标定；
6. release 由对相应状态组真正有信息的 accepted update 数、当前误差估计、当前标准差、相邻变化、窗口峰峰值、innovation acceptance 和无 clipping 共同决定；没有固定 2 s、12 s 或 20 s 成功门槛；
7. timeout 只能 fail/retry 或明确 degraded，不能产生正常 P4 success；
8. 在视觉没有真正进入递归 update 前，只能说视觉参与初始联合图和独立 release 检查，不能说视觉参与闭环反馈。

## 9. 实现前必须先有的验证

顺序固定为 global baseline，再考虑 June12：

1. 保存当前无改动 global baseline 的命令、配置、轨迹和标准误差报告；
2. 单元测试误差定义和反馈符号：正负 position、velocity、attitude、`bg`、`ba` correction 必须使对应 residual 下降；
3. 验证一次反馈后的状态确实进入下一段 IMU propagation；
4. 验证 Joseph update、姿态 reset 后 covariance 对称、有限、正定；
5. 对每个状态组验证收敛条件真值表、滚动历史保留、反馈后重新判断、无关量测不计数和 rejected measurement 不计数；
6. 验证 p/v 可收敛但 attitude 不收敛时不得正常 release；
7. 验证 attitude 收敛而某个 bias 不收敛时，mask、provenance 和 covariance 与实际一致；
8. 验证 turn-flex FC attitude 残差只影响评价标记，不触发错误的名义姿态追随；
9. 验证同一历元多次 update 的 correction/clipping 被累计，而不是只保留最后一次；
10. 同一 global baseline、同一评价口径比较无改动版本与候选版本。候选的 release q/p/v 或全程指标变差即失败，不在退化结果上继续调次级门限。

在这些映射和测试落实前，当前固定 12--20 s replay 草稿不具备继续 debug 或实验调参的资格。
