# P4 临时 VIO 与全局 Gauge 锚定重构规格

> 已否决的历史规格：后续飞行实验出现惯性速度/尺度不连续，该路线不得作为当前
> P4 实现依据。详见 `P4_EXPERIMENT_DECISION_LEDGER.csv` 的 `P4-D021`。

## 1. 结论

当前 P4 的失败不是“8 秒窗口不如单行数据”，而是释放合同错误：P4 在窗口结束时只把末端 `q/p/v/bg/ba` 和 15×15 covariance 写入一个空的 OpenVINS filter，并删除释放时间以前的 feature measurements。它没有继承已经运行的 clone、landmark、FEJ 与状态交叉协方差。

同时间重注入实验已经证明这一点：在相同 P4 释放时刻，把旧单行初始化后已经运行约 24 秒的 OpenVINS 状态重新注入，XY RMSE 从 P4 末状态注入的约 50.1 m 降到约 33.8 m，接近连续运行旧初始化的约 31.8 m。交换 `q/p/v` 与 bias 的实验还表明这些量存在强耦合，不能把窗口图的末状态分块覆盖到另一个滤波器。

因此正式重构采用以下合同：

```text
首个因果 FC-board provisional state
  -> 启动临时 OpenVINS，正常积累 IMU、KLT、clone、MSCKF、SLAM、bias 与 covariance
  -> P4 在旁路继续执行固定时间滑窗 FC/IMU/视觉联合估计
  -> P4 通过内部质量门限
  -> 只把 P4 提供的全局 yaw 与全局 position 作为 4-DOF gauge anchor
  -> 对现有完整 OpenVINS 状态执行一次原子 yaw+translation 变换
  -> 正式释放导航；以后不再覆盖 q/p/v/bg/ba
```

P4 释放前的 OpenVINS 只属于内部 warm-up，不是正式导航输出，不启动 P5。

## 2. 不采用的方案

- 不再在 P4 释放时调用 `initialize_with_online_alignment()` 覆盖末端 15 维状态。
- 不把 P4 的 `v/bg/ba` 写入已经运行的 OpenVINS。
- 不用 FC/GPS course 或未来轨迹误差作为在线 yaw 输入。
- 不把 position/velocity 作为释放后的连续 FC/GNSS 融合。
- 不通过调整 process noise、measurement noise、release threshold 掩盖状态不一致。
- 不清空已有 clone、landmark 或 FeatureDatabase 历史。

## 3. 坐标变换

设当前 OpenVINS 全局系为 `W`，P4 输出全局系为 `G_nav`。P4 只确定允许外部锚定的 yaw 与 translation：

```text
Delta_psi = yaw(R_ItoGnav_P4) - yaw(R_ItoW_VIO)
R_delta   = Rz(Delta_psi)
t_delta   = p_IinGnav_P4 - R_delta p_IinW_VIO
```

对 nominal 和 FEJ 同时执行：

```text
R_ItoGnav = R_delta R_ItoW
p_inGnav  = R_delta p_inW + t_delta
v_inGnav  = R_delta v_inW
```

同一变换应用到当前 IMU pose、全部 IMU clones 和 global landmarks。`bg`、`ba`、camera/IMU calibration 与 anchored landmark 的 anchor-frame 参数保持不变。因为重力轴不变，该变换只能是绕 `G` 系 z 轴的旋转，不能包含 roll/pitch 或 scale。

## 4. Covariance 与 FEJ 合同

OpenVINS 使用 JPL 左乘姿态误差：

```text
R_GtoI_true = Exp(-delta_theta) R_GtoI_nominal
```

对固定的全局 yaw gauge 右乘后，`delta_theta` 不变。因此完整 error-state reset Jacobian `J` 为：

- IMU/clone orientation：`I3`；
- global position：`R_delta`；
- global velocity：`R_delta`；
- `bg/ba` 和 calibration：单位阵；
- global 3D landmark：`R_delta`；
- global inverse-depth landmark：对 `repr(R_delta xyz + t_delta)` 计算局部参数 Jacobian；
- anchored landmark：单位阵，因为 anchor pose 和全局场景一起变换，anchor-frame 参数不变。

完整 covariance 必须一次性更新：

```text
P_new = J P_old J^T
P_new = 0.5 (P_new + P_new^T)
```

变换前后都检查有限性、对称性和半正定性。任何检查失败都回滚 nominal、FEJ 和 covariance，不能留下半变换状态。成功后必须 invalidate propagator cache。

## 5. 生命周期

### 5.1 临时启动

`OnlineAlignmentInitializer::provisional_navigation()` 已经按锁定的 FC-board calibration、lever arm 和时间关系生成因果 board-IMU state。获得首个有效 provisional state 后，以旧单行初始化相同的 covariance 合同启动 OpenVINS。

临时启动只改变 OpenVINS 内部生命周期：

- P4 继续接收 FC、board IMU 和 KLT observations；
- P4 仍按固定 8 秒滑窗和因果 holdout/refinement 运行；
- P4 初始化阶段仍使用固定相机 cadence；
- P5、正式 `traj_nav` 和正式 navigation-frame metadata 保持关闭。

### 5.2 正式释放

P4 返回 `released_to_openvins=true` 时：

1. 要求临时 OpenVINS 已初始化，并且 P4 result 与当前 camera/state timestamp 对齐；
2. 从 P4 result 读取目标 yaw 与 position；
3. 对现有完整状态应用一次全局 yaw+translation reset；
4. 保留当前 OpenVINS `v/bg/ba`，其中 velocity 只随 global yaw 旋转；
5. 保留 clone、landmark、FeatureDatabase 与完整 covariance；
6. reset 成功后才标记 P4 正式释放并开放正式输出/P5；失败则 fail closed，不伪造 release。

## 6. 验收标准

### 6.1 单元与集成

- 任意 yaw+translation 后，所有 camera-to-landmark relative coordinates 与 reprojection geometry 保持不变；
- IMU/clone nominal 与 FEJ 同步变换；
- `bg/ba` nominal 与 FEJ bitwise/数值不变；
- anchored landmark 参数不变，global landmark 正确变换；
- 完整 covariance 与独立数值 Jacobian 的 `J P J^T` 一致、有限、对称、PSD；
- identity reset 不改变状态；非法输入和非 PSD covariance 原子回滚；
- propagator cache 只在成功非 identity reset 后失效；
- P4 释放前 VIO 已运行但 P5 和正式导航输出未启动；
- P4 只允许成功锚定一次，不能重复 reset。

### 6.2 飞行回放

使用冻结旧单行 baseline 和相同 backend-time/GPS-time common grid，对 fly1、fly2、fly3 执行短程和全程比较：

- P4 初始绝对 yaw/position 精度保留；
- start-heading 后的 XY RMSE、末端 XY、course error 不得劣于同源单行初始化；
- `bg/ba` 演化、MSCKF/SLAM accepted counts 和第一转弯前后轨迹连续；
- reset 前后视觉 residual、relative clone geometry 和 landmark reprojection 不发生跳变；
- 正式评价只使用 `traj_nav.txt`、GPS update time 和 start-heading alignment；best-fit 只作诊断。

只有上述测试和 fly1/2/3 全程验收通过，才能认定重构完成并恢复 P5 工作。
