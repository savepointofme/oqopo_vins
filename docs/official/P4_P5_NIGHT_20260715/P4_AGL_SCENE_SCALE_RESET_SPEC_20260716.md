# P4 后端 AGL 场景尺度一致性重置规格

## 目标与边界

目标是在 MSCKF/SLAM 已形成稳定单目场景后，使用外部 AGL 恢复场景的统一米制尺度；不是把高度 residual 送进 `p_z` Kalman update，也不是事后只缩放输出轨迹。

允许输入：因果 AGL 标量及其时间/质量、当前 VIO 姿态、clone、三角化 feature/SLAM landmark。禁止输入：GPS XY、GPS course、FC/GPS yaw、最终轨迹误差和未来窗口。

## 为什么 output-only 缩放无效

只改 `traj_nav.txt` 不会修改：

- clone baseline；
- landmark depth/逆深度；
- velocity；
- FEJ linearization point；
- covariance 与状态交叉项；
- 下一次视觉 residual/Jacobian。

因此已有 output-only 缩放退化不能否定 estimator-consistent 场景尺度重置。

## 场景高度估计

在每次成功 backend update 后，从当前窗口内新近三角化、正深度且通过 residual/几何门的 feature 构造地面候选。优先复用 ground-plane updater 的投影和候选几何，但只做 dry-run 估计，不调用 EKF update。

采用重力方向约束的稳健平面：

- 法向与当前重力方向夹角受限；
- RANSAC/Huber 拒绝树木、建筑和天空错误点；
- 要求图像空间覆盖和多 clone 支撑；
- 输出 `h_map`、inlier ratio、平面 residual、时间跨度和不确定度；
- `h_agl/h_map` 在时间窗内用 log-scale 稳健聚合。

只有 scale estimate 在新数据上稳定，才允许一次 reset。不得把超限 correction clip 后仍应用；超限必须 fail closed。

## Sim(3) 重置

以当前相机中心 `c*` 为固定点：

```text
s = h_agl / h_map
c_i' = c* + s (c_i - c*)
p_f' = c* + s (p_f - c*)
```

Camera–IMU 外参保持物理长度不变。每个 clone 先变换相机中心，再恢复 IMU 位置：

```text
c_i = p_Ii + R_ItoG_i t_CinI
c_i' = c* + s(c_i-c*)
p_Ii' = c_i' - R_ItoG_i t_CinI
```

由此当前相机中心与当前 IMU 位姿不发生无意义跳变。

状态变换合同：

| 量 | 处理 |
| --- | --- |
| q、bg、ba、gravity | 不变 |
| 当前/clone camera center | 围绕 `c*` 统一缩放 |
| IMU position | 由缩放后的 camera center 和固定 lever arm 恢复 |
| velocity | 按尺度变换；若考虑 lever arm，缩放 camera-center velocity 后恢复 IMU velocity |
| GLOBAL_3D landmark | 围绕 `c*` 缩放 |
| anchored 3D | anchor-frame 坐标乘 `s` |
| inverse depth | bearing 不变，`rho' = rho / s` |
| intrinsics、distortion、`T_C_I` | 不变 |

名义值与 FEJ 必须执行同一变换。当前 `apply_trusted_pose_anchor_reset()` 只处理部分 global landmark，且没有 covariance reset，不能直接复用。

## Covariance reset

AGL 不参与 Kalman innovation/Kalman gain，但状态坐标发生确定性重参数化，必须执行：

```text
P' = J_reset P J_reset^T
```

`J_reset` 覆盖 IMU、全部 clone、SLAM feature 及其交叉项，并包含当前 pivot 和 Camera–IMU lever arm 对姿态误差的导数。完成后对称化、检查有限/半正定/负对角，并使 propagator cache 失效。任何检查失败都回滚整次 reset。

## 测试

1. 合成已知尺度误差能够恢复 `s_true`。
2. reset 前后所有有效 observation 的重投影像素保持不变。
3. 当前相机中心不动，q/bg/ba 不变。
4. GLOBAL_3D、anchored 3D、full inverse depth、single inverse depth 分别正确变换。
5. nominal/FEJ 同步，covariance 有限、对称、PSD，无负对角。
6. 连续两次互逆 scale reset 可恢复原状态到数值容差。
7. 无稳定地面、AGL 过期、scale 超限、covariance 检查失败时不修改任何状态。
8. reset 后下一次 propagate/MSCKF/SLAM 正常，且没有 cache/history 混用。

## 分阶段验证

1. `shadow_scale`：只输出 `h_map/h_agl/s` 和质量，不改状态；验证时间一致性与稳定性。
2. `single_reset`：每条 focus 轨迹只允许一次稳定 reset，隔离重参数化正确性。
3. `guarded_repeat_reset`：只有新独立时间窗再次稳定时才允许后续 reset。
4. fly1/fly3 半圈到一圈通过后才跑 full flight。

验收同时检查尺度、XY/Vxy/U、heading/course、visual residual、landmark depth、bias、covariance 和运行开销。尺度改善但 yaw 不变是允许且有意义的结果；yaw 改善但重投影不守恒或 covariance 不健康则判失败。
