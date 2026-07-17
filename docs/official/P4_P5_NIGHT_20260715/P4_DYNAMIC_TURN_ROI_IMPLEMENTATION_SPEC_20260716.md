# P4 后端动态转弯 ROI 实现规格

## 目标

在不使用 FC/GPS yaw 的前提下，根据 board IMU 的因果转弯方向，左转时优先图像右侧、右转时优先图像左侧；直线恢复全图。实现必须保持 KLT 连续性，不能因 ROI 切换人为制造 lost-track MSCKF 更新。

## 现有实现为什么不能复用

runner 当前 `apply_post_alignment_visual_roi()` 直接修改 camera mask。`TrackKLT::feed_monocular()` 在 mask 外直接丢弃已有轨迹。被丢弃的 feature 随后会被后端视为真实 lost track，可能在转弯入口集中触发 MSCKF 更新。因此静态半图结果同时混入：

- 新点不再检测；
- 已有点立即死亡；
- 空间覆盖改变；
- lost-track 更新时机改变。

它不能回答“只优先安全侧后端证据”会怎样。

## 输入和方向

唯一方向信号：

```text
omega_yaw = (omega_I - bg) · (R_GtoI e_z)
```

其中姿态和 bias 取当前因果状态。正负号在单元测试中以合成左/右转验证，再用 FC 仅离线核对，不把 FC 姿态送入策略。

状态只用于滞回，不输出固定 cadence 档位：

```text
STRAIGHT
TURN_LEFT
TURN_RIGHT
RECOVERING
```

`turn_strength ∈ [0,1]` 由低通后的 `|omega_yaw|` 连续产生。方向反转必须先经过零区/恢复区，不能一帧内从左侧权重跳到右侧权重。

## 三层分离

### 1. Tracking continuity

已有 KLT track 始终允许在全图继续跟踪。ROI 不得删除已有 feature ID，不得导致 database measurement 被当成真实丢失。

### 2. Detection allocation

只对新 feature 检测施加侧别配额：

- 左转：右侧目标配额随 `turn_strength` 连续增加；
- 右转：左侧目标配额随 `turn_strength` 连续增加；
- 直线：恢复全图均衡网格；
- 非优先侧保留最小配额，防止单侧几何退化。

不能用二值半图 mask 取代配额；应在网格/候选排序阶段完成。

### 3. Backend eligibility

每个 track 计算因果分数：

```text
score = geometry_information
      × track_health
      × side_weight(u, turn_direction, turn_strength)
      × residual_health
```

MSCKF cap、regular SLAM update 和 delayed SLAM initialization 都使用同一 side policy，但保留各自几何/生命周期规则。被降权的现有轨迹继续跟踪，不因“暂不进入后端”而被删除。

## 必须记录的运行量

- `omega_yaw_raw/filtered`、turn direction、turn strength；
- 左/右 existing tracks、新检测、后端输入、接受、拒绝数量；
- 因 ROI policy 暂缓的 feature ID 数，不得与 lost 数混淆；
- MSCKF/SLAM residual、NIS/chi²、yaw correction contribution；
- track churn、空间 entropy、feature center；
- 每帧 tracking/backend 实际执行时间。

## 测试

1. 合成左转只提高右侧 backend score；合成右转只提高左侧；直线两侧相等。
2. ROI 状态切换前后，已有 feature ID 连续，人工 lost count 为 0。
3. 同一组输入反转 `omega_yaw` 符号，侧别结果严格镜像。
4. 非优先侧保留配额，信息矩阵秩/空间 entropy 不低于安全下限。
5. mask 只用于传感器无效区，不再承载动态侧别策略。
6. fly1/fly3 focus replay 的 P4 fingerprint 与 control 一致。

## 实验判定

先跑四个单变量：control、仅 detection bias、仅 backend bias、二者组合。这样可以区分收益来自新点分布还是后端选择，不再把所有作用混进一个 mask。

候选必须同时满足：fly1 左转和 fly3 右转的首错段 heading 增量下降；出弯直线漂移斜率下降；XY/Vxy 不发生静态 ROI 那种灾难性退化；feature coverage、residual、covariance 健康。只改善一架次不能发布。
