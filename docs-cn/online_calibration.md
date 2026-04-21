# 在线标定 (Online Calibration)

> **前置**: [`math_foundations.md`](./math_foundations.md), [`state_and_cov.md`](./state_and_cov.md), [`measurement_math.md`](./measurement_math.md).
> **官方数学**: `docs/update-feat.dox` (末尾讨论外参/内参 Jacobian).
> **涉及代码**: `State.h`, `StateOptions.h`, `VioManager::*` (读 YAML), `UpdaterHelper::get_feature_jacobian_full`.

OpenVINS 支持运行时**在线**估计下列标定量, 全部通过 `StateOptions` 的若干 bool 开关打开。开启后对应变量会被加进 `_variables`, `_Cov` 维度变大, 每次相机更新时会把 Jacobian 里相应列填好, 标定参数自然随状态一起收敛。

---

## 1. 可标定项总览

| 选项 | 含义 | 维度 | 代码变量 |
|---|---|---|---|
| `do_calib_camera_pose` | IMU-相机外参 `(R_ItoC, p_IinC)` | 6 × num_cams | `state->_calib_IMUtoCAM[cam_id]` |
| `do_calib_camera_intrinsics` | 相机内参 fx/fy/cx/cy + 4 畸变系数 | 8 × num_cams | `state->_cam_intrinsics[cam_id]` |
| `do_calib_camera_timeoffset` | IMU-相机时间偏移 `t_imu = t_cam + t_off` | 1 | `state->_calib_dt_CAMtoIMU` |
| `do_calib_imu_intrinsics` | 陀螺/加计尺度+非正交 `Dw, Da` + 轴旋转 | 12 或 15 | `_calib_imu_dw/da/GYROtoIMU/ACCtoIMU` |
| `do_calib_imu_g_sensitivity` | 陀螺对加速度敏感度 `T_g` (g-sensitivity) | 9 | `_calib_imu_tg` |

---

## 2. 相机外参 `(R_ItoC, p_IinC)` 的数学

### 2.1 在 Jacobian 中的位置

特征观测对 **外参** 的偏导 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="404-413" />:
```cpp
if (state->_options.do_calib_camera_pose) {
    Eigen::MatrixXd dpfc_dcalib(3, 6);
    dpfc_dcalib.block(0, 0, 3, 3) = skew_x(p_FinCi - p_IinC);          // ∂p_FinCi / ∂δθ_ItoC
    dpfc_dcalib.block(0, 3, 3, 3) = I;                                   // ∂p_FinCi / ∂δp_IinC
    H_x.block(2*c, map_hx[calibration], 2, 6) += dz_dpfc * dpfc_dcalib;
}
```

**推导**:
```
p_FinCi = R_ItoC · p_FinIi + p_IinC
∂p_FinCi / ∂δθ_ItoC ≈ ⌊R_ItoC · p_FinIi⌋ = ⌊p_FinCi - p_IinC⌋
∂p_FinCi / ∂δp_IinC = I
```

注意这里没用 FEJ — 大部分 calibration 量变化很慢, 收敛后就"锁"在最近均值附近, 不会引入一致性问题。

### 2.2 什么情况下 **不要** 开?

- **EuRoC 自带的标定已经很好** (< 1°), 开外参 online 反而因为特征噪声让它微抖, 一般不建议。
- 长距离、少特征的数据 (高速公路) 容易和外参 baseline 混淆, 会让外参"吸收"其实是外参固定下的估计漂移。
- **最佳用法**: 离线用 Kalibr 标一次, 打开 `do_calib_camera_pose: true` 但通过 `init_cov` 把对角线设得很小 (如 `1e-4` rad², `1e-4` m²), 表达"我相信离线结果, 只允许微小在线修正"。

---

## 3. 相机内参 + 畸变

### 3.1 参数化

OpenVINS 用 8 维: `[fx, fy, cx, cy, d0, d1, d2, d3]`, 畸变模型分 `radtan` (k1,k2,p1,p2) 和 `fisheye/equi` (k1,k2,k3,k4)。见 `ov_core/src/cam/CamRadtan.h` 和 `CamEqui.h`。

### 3.2 Jacobian — `dz_dzeta`

<ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="366-367" />:
```cpp
Eigen::MatrixXd dz_dzn, dz_dzeta;
state->_cam_intrinsics_cameras.at(pair.first)->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta);
```

- `dz_dzn` (2×2): 畸变对归一化坐标的 Jacobian, 用于主雅可比链式;
- `dz_dzeta` (2×8): 残差对 8 个内参参数的 Jacobian, 直接填进 `H_x` 中相应列 (`H_x.block(2*c, map_hx[distortion], 2, 8) = dz_dzeta`)。

### 3.3 打开它的代价与收益

- **收益**: 长时间运行能把内参估到相当准 (fx/fy 稳定到 ±0.5 px)。
- **代价**: 状态维度 +8/cam, `_Cov` 膨胀 → `EKFUpdate` 更慢。
- **建议**: 对高精度应用 (如 AR) 开, 对移动机器人 (imu 主导) 可以关。

---

## 4. 时间偏移 `t_off = t_imu - t_cam`

### 4.1 为什么一定要估?

硬件触发的相机快门与 IMU 时钟几乎从不完美对齐, 常有 `1~20 ms` 的偏差。若不估, 所有运动快的动作会被当作"轨迹本身有残差", 产生持续性误差 (典型现象: 静态时平稳, 动起来就漂, 停下后又稳定)。

### 4.2 在代码里的身影

(a) Propagation 段使用 `_calib_dt_CAMtoIMU` 决定 IMU 积分区间 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="63-78" />:
```cpp
if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    ...
}
double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);
double time0 = state->_timestamp + last_prop_time_offset;
double time1 = timestamp + t_off_new;
```

(b) 随机克隆时用 **最后一次** 角速度 `last_w` 把克隆位姿"回退"到相机时钟 `t_cam`:
```cpp
// Propagator.cpp:146
StateHelper::augment_clone(state, last_w);

// augment_clone 内部:
//   R_clone = exp(-last_w · dt_off) · R_imu
//   p_clone = p_imu + v_imu · dt_off + ½ a · dt_off²   (dt_off = _calib_dt_CAMtoIMU 值)
```

这就把 "IMU 积到的 t_imu 时刻的姿态" 校正回 "相机快门 t_cam 时刻的姿态", 供后续 measurement 用。

### 4.3 YAML 配置
```yaml
estimator_options:
  calib_camtimeoffset: true       # do_calib_camera_timeoffset
  init_camtimeoffset: -0.01       # 初始猜测 (s)
```
初始值如果错得离谱 (差几百 ms), 滤波器很难自己拉回来 — 建议先离线用 Kalibr-imucam 估一次再启动。

---

## 5. IMU 内参 `(D_w, D_a, T_g, R_GYROtoIMU, R_ACCtoIMU)`

绝大多数消费级 IMU 用默认值 (`I, I, 0, I, I`) 即可 — 即假设 IMU 没有尺度误差、正交、无重力漂移。

开启 `do_calib_imu_intrinsics` 会在 `Propagator::predict_and_compute` 里让:
```
a_corrected = R_ACCtoIMU · D_a · (a_m - b_a)                    (代码 L449-451)
ω_corrected = R_GYROtoIMU · D_w · (ω_m - b_g - T_g · a_corrected)  (代码 L454-463)
```
所有这些 `D*, T_g, R_*` 出现在状态里, 有对应的 Jacobian 块 (在 `compute_F_and_G_*`) 填入 `F`, `G`。 数学推导见官方 `docs/propagation-analytical.dox`。

**实战**: 高端 IMU 标定 (如 Vicon Tracker+Kalibr-imu) 能给 `D_a, D_w` 到 `1%` 精度, 开在线标定几乎没收益 (除非硬件热漂移严重)。默认 **关闭**。

---

## 6. 一个经验配置矩阵

| 场景 | cam_pose | cam_intrinsics | cam_timeoffset | imu_intrinsics |
|---|---|---|---|---|
| EuRoC (研究/benchmark) | off | off | on | off |
| 自制无人机, 用 MYNT 双目 | on | on | on | off |
| AR/MR 头戴, 需亚像素精度 | on | on | on | on (初值 Kalibr) |
| 长期部署机器人 | on | off (离线标 + freeze) | on | off |

---

## 7. 速查

| 问题 | 代码位置 |
|---|---|
| 时间偏移读写 | `State::_calib_dt_CAMtoIMU`, `Propagator.cpp:63-78` |
| 外参 Jacobian | `UpdaterHelper.cpp:404-413` |
| 内参 / 畸变 Jacobian | `UpdaterHelper.cpp:366-367, 416-418` + `ov_core/src/cam/Cam*.cpp` |
| IMU 内参 Jacobian | `Propagator::compute_F_and_G_*` |
| 是否开启在线标定 | `StateOptions.h` 各 `do_calib_*` 开关 |
