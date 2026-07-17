# P4 后端内部航向 gauge 原点修复实验规格

## 问题

P4 把状态直接初始化到 `G_nav`，当前 fly1/fly3 的水平位置模长约为数千米；历史单行 FC 初始化则在几十米量级的局部原点附近运行。现有 `global_yaw_oc_projection` 用

```text
n_yaw = [R_GtoI e_z, e_z × p, e_z × v, ...]
H_oc  = H - H n_yaw n_yaw^T / ||n_yaw||²
```

保护单目 VIO 的全局航向零空间。因为只投影一维 yaw、没有同时构造全局 XY translation 零空间，`e_z × p` 会随任意全局平移改变。P4 的公里量级绝对位置因此支配 `||n_yaw||`，使同一视觉问题仅因导航坐标原点不同而得到不同 OC Jacobian。这违反 VIO 对全局平移的 gauge 不变性。

## 候选修复

新增独立实验模式 `global_yaw_oc_centered_projection`。它保持现有 FEJ、视觉 residual、chi-square、MSCKF、SLAM 和 Kalman update 不变，只把 yaw gauge 中的位置统一改为相对最老 IMU clone 的位置：

```text
p_rel = p - p_anchor
p_f_rel = p_f - p_anchor
n_yaw_centered = [R_GtoI e_z, e_z × p_rel, e_z × v, ...]
```

没有 clone 时以当前 IMU 位置为 anchor。速度项不平移。该模式不读取 GPS、FC 姿态、GPS course、最终误差或未来数据；FC 仍只用于 P4 初始化，GPS 只用于离线评价。

同时增加更完整的坐标合同消融 `local_estimator_origin`：P4 的绝对位置仍保存在 `OnlineAlignmentResult`，但注入 OpenVINS 的内部位置改为零；固定输出变换设为

```text
R_Gnav_W0 = I
p_W0inGnav = p_IinG_release
```

因此 `traj_nav` 与原合同仍在绝对 `G_nav`，而 IMU state、clone、landmark 和视觉 Jacobian 在释放点附近的局部 `W0` 运行。GPS-Z 现有实现本来就在首个量测处建立 GPS/VIO 高度差，所以不需要也不允许引入 GPS XY/yaw。该消融用于区分“仅 yaw OC 原点依赖”与“整个后端直接使用大坐标”的影响。

上述两项只能确认坐标依赖，不能通过“挑一个表现更好的原点”作为正式修复。正式候选 `full_4d_oc` 使用 `VisualObservabilityPolicy` 的完整四维 FEJ 子空间：三列全局平移和一列全局 yaw。投影使用该子空间的正交基 `Q`：

```text
H_oc = H - H Q Q^T
```

全局原点平移只会让 yaw 列增加三列平移基的线性组合，不改变四维列空间，因此该投影对原点选择严格不变。它在 MSCKF、regular SLAM 和 delayed SLAM 的 chi-square gate 前执行，接受后的 EKF 使用同一已投影 Jacobian；不读取 GPS/FC yaw。

## 验收

1. 直接 target 使用并行构建完成，相关初始化测试通过；
2. fly1/fly3 使用与 control 相同的 P4、KLT、fixed stride12、GPS-Z 和 focus 时间窗；`centered_yaw_oc`、`local_estimator_origin` 和 `full_4d_oc` 分别作为单变量条件，不相互混用；
3. `traj_nav.txt` 非空，协方差有限、对称、无负对角；
4. 同时检查用户指定的 fly1 首转及出弯直线、fly3 首转后至第二转前直线后半；
5. 使用官方 GPS-time、绝对 `G_nav` 无后对齐评价，同时报告 course、XY、Vxy、U、visual residual、feature churn、`bg_z` 和逐类视觉 yaw correction；
6. 只有 fly1/fly3 均有一致改善且内部 residual/covariance 不退化，才允许把该模式升级为默认；否则保留为失败消融并继续调查。
