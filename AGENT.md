# FC-IMU Residual Calibration

## 本轮唯一 Goal

基于 fly1～fly4 中现有 FC 与 VIO 原始姿态数据，联合估计 FC 相对于 OpenVINS IMU 的剩余小时间偏移和固定三维安装旋转；本轮不修改 estimator、初始化、nav 输出、dashboard 或 official evaluation 对齐逻辑。

## 坐标方向最终定义

- `F`: FC 原始机体系，raw body 为 FRD。
- `I`: OpenVINS IMU 坐标系。
- FC 原始姿态按 NED ZYX Euler 读取，当前 nominal 转换使用 `yaw_sign=-1` 和 `frd_to_xright_yfwd_zup`。
- `fc_q_*`: 已转换到 OpenVINS nominal IMU 约定的 JPL `q_GtoI`，不是原始 FC 四元数。
- `vio_raw_q_*`: `traj_raw.txt`/`traj.txt` 中 Hamilton `[qx,qy,qz,qw]`，表示 `R_ItoW0`。
- 相对旋转使用 `A = R_WI(t)^T R_WI(t+dT)`，`B = R_WF(t+delta_t_fc)^T R_WF(t+dT+delta_t_fc)`。
- `R_I_F` 方向: 将 raw FC FRD body-frame 向量旋转到 OpenVINS IMU-frame。
- nominal 轴转换: `R_nominal_axis_transform = [[0,1,0],[1,0,0],[0,0,-1]]`。
- residual 组合顺序: `R_I_F = R_residual * R_nominal_axis_transform`。

## 时间偏移符号定义

FC attitude used at VIO time `t` is `R_F(t + delta_t_fc)`。

## 使用的训练/验证 run

- fly1: `D:\vscode_dir\open_vins\result\stride12_robustness_8h\current_stride12\fly1_stride12_20260707_010925`
- fly2: `D:\vscode_dir\open_vins\result\stride12_robustness_8h\current_stride12\fly2_stride12_20260707_010925`
- fly3: `C:\Users\baloney\Desktop\openvins_fc_init_fix_validation_20260708\fly3_stride12_20260708_135312`
- fly4: `C:\Users\baloney\Desktop\openvins_fc_init_fix_validation_20260708\fly4_stride12_20260708_135312`

## 最终 delta_t_fc

`delta_t_fc_s = -0.379000000`

## 最终 R_I_F

```text
[ 0.129974402251,  0.990197853478, -0.051135757804]
[ 0.991360888290, -0.130697120866, -0.011038648706]
[-0.017613742573, -0.049259248512, -0.998630703768]
```

`q_I_F = [-0.751548772018, -0.659158399145, 0.0228692744023, 0.0127139452651]`

`R_residual` display RPY deg: `[-1.01047211427, 2.82348968159, -7.51905871563]`

## 验证残差

- Baseline A: median `1.584 deg`, P95 `5.468 deg`, max `28.971 deg`, inlier `0.922`。
- Baseline B: `delta_t_fc=-0.227 s`, median `1.409 deg`, P95 `5.089 deg`, max `13.889 deg`, inlier `0.947`。
- Baseline C: median `1.512 deg`, P95 `5.405 deg`, max `28.667 deg`, inlier `0.932`。
- Candidate D: `delta_t_fc=-0.379 s`, median `1.283 deg`, P95 `4.692 deg`, max `14.350 deg`, inlier `0.955`。
- LOFO holdout fly1: train delta `-0.199 s`, validation median `1.288 deg`, P95 `4.084 deg`。
- LOFO holdout fly2: train delta `-0.499 s`, validation median `1.585 deg`, P95 `8.565 deg`。
- LOFO holdout fly3: train delta `-0.198 s`, validation median `2.340 deg`, P95 `6.227 deg`。
- LOFO holdout fly4: train delta `-0.379 s`, validation median `1.273 deg`, P95 `4.450 deg`。
- Single-flight delta spread: fly1 `-0.477 s`, fly2 `-0.121 s`, fly3 `-0.500 s`, fly4 `-0.256 s`。
- Max single-flight rotation spread: `7.752 deg`。
- Motion groups A→D median/P95: straight `1.235/3.943 -> 0.825/2.651 deg`; left turn `2.590/7.964 -> 2.249/5.392 deg`; right turn `1.527/4.484 -> 1.544/4.650 deg`。
- Absolute first-anchor check A→D mean window median: fly1 `3.777 -> 2.797 deg`; fly2 `3.229 -> 2.477 deg`; fly3 `4.314 -> 4.400 deg`; fly4 `3.685 -> 3.417 deg`。

## accepted

`accepted: false`

拒绝原因: single-flight rotation spread `7.752 deg` 超过 `5.000 deg`；single-flight `delta_t_fc` 有一个落在 `-0.50 s` 搜索边界；single-flight `delta_t_fc` spread `0.379 s` 超过 `0.250 s`；right-turn median 未改善。

## 下一步唯一动作

针对不可观方向、动态延迟或数据时间轴问题补充最小验证，不接入初始化。
