# P4 滑动窗口在线运行时重构规格（2026-07-17）

## 问题与证据

当前正式 P4 在每个推进后的 8 s 窗口上都重建并求解完整
q/p/v/bg/ba 图，这一语义必须保留。但求解结束后，每个普通窗口还会执行
最终释放级别的全 Jacobian、三次稠密 Schur 信息消元以及 DENSE_SVD
联合协方差恢复。

fly1 同条件运行已经证明瓶颈不在 Ceres：

| 输入安装角 | 窗口数 | Ceres 总时间 | 进程 user CPU |
| --- | ---: | ---: | ---: |
| Kalibr conservative | 53 | 2.02 s | 280.56 s |
| Kalibr full | 79 | 3.35 s | 421.62 s |
| frozen v3 | 123 | 5.72 s | 636.82 s |

每个最终窗口的完整处理约 4.6--5.8 s，而 Ceres 本身每窗约
0.04--0.05 s。重型认证被重复到每个普通滑窗，导致回放和在线路径均无法
满足实时性。

## 冻结语义

- 固定 8 s 时间窗口继续随新数据前移。
- 每个合格窗口继续重新选择窗内 FC、IMU、单目关键帧，并执行一次标准
  q/p/v/bg/ba 联合图求解。
- 不改成固定量测次数、固定关键帧目标或一次性 candidate 滤波。
- 稳定性继续由传感器时间、相同时间戳重叠状态和 bias 趋势决定。
- 在线逻辑不得使用 GPS XY、GPS course、未来轨迹误差或最终真值。
- 正式释放仍必须具有完整 factor、Schur 信息、状态可观性和正定 15 维
  终端协方差证据。

## 新的两级执行路径

### A. 每窗滚动估计

每个推进窗口执行：

1. 重建窗内图、三角化 landmark、IMU 预积分并运行 Ceres；
2. 输出该窗 terminal q/p/v/bg/ba；
3. 计算相同时间戳的物理重叠差和因果 bias 趋势；
4. 只计算形成候选所需的 residual-only 统计；
5. 保存 warm start 和时间窗口 receipt。

本阶段不得恢复 clone/landmark 联合协方差，不得运行三次稠密 Schur，也
不得为全部 factor 重复构造全局 Jacobian。

### B. 释放认证

只有 A 阶段的物理重叠、残差和 bias 趋势在真实传感器时间上达到稳定条件
时，才对当前窗口执行一次完整认证：

1. factor family residual/Jacobian contribution；
2. terminal state 的数据 Schur 信息与视觉增量信息；
3. 生产路径实际注入的 15 维 terminal covariance；
4. 逐状态可观性、正定性和正式 release gates。

认证失败后保持滑窗继续前移；下一次认证由新的时间窗口和稳定证据触发，
不使用固定更新次数。

## 生产协方差合同

当前 `VioManagerHelper` 的正式 handoff 只注入 terminal q/p/v/bg/ba，并明确
清空初始化 clones、landmarks 和 joint history covariance。因此生产 P4 不应
在每个窗口恢复随后必定丢弃的 clone/landmark covariance。通用 initializer
测试仍可保留完整 posterior-transfer 模式；ROS-free 正式配置使用
terminal-only covariance 模式。

## 验收标准

1. initializer 单测和 R1 contract validator 通过；
2. 每个窗口仍恰好执行一次标准联合图求解，窗口起止和选中帧真实前移；
3. release metadata 仍具有完整 factor、Schur、可观性和 15x15 covariance；
4. 普通窗口不得标记为完整认证窗口；
5. fly1/fly3 正式释放结果不得因运行时重构改变 nominal 解或释放门限语义；
6. 同机同数据上，普通窗口处理时间应低于窗口推进对应的传感器时间，且
   Ceres 外重复开销至少下降一个数量级；
7. 最终完整认证只在时间稳定候选上发生，次数和总耗时写入 metadata 与
   sliding-window trace。
