# 数学基础速查 (JPL 四元数 / 误差状态 / EKF)

> 目的: 在读代码前, 先把 OpenVINS 里频繁出现的数学"黑话"和它们在 C++ 类里的对应搞清楚。
> 每个公式都会告诉你它出现在代码哪里, 方便你打开源文件边看边对。
>
> 参考官方文档 (英文): `docs/propagation.dox`, `docs/fej.dox`, 以及类注释 `ov_core/src/types/JPLQuat.h:47-91`。
> 原始技术报告: [Trawny & Roumeliotis 2005](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) (代码里缩写为 @cite Trawny2005TR)。

---

## 1. 坐标系与符号

| 记号 | 含义 | 代码访问 |
|---|---|---|
| `{G}` | 全局(惯性)系 | 所有 `*_G*`, `p_*inG`, `R_Gto*` |
| `{I}` | IMU 体坐标系 (滤波器的 "本体") | `state->_imu`, `R_GtoI`, `p_IinG` |
| `{C}` | 相机坐标系 | `state->_calib_IMUtoCAM[cam_id]` |
| `{A}` | 特征的 anchor 相机系 (用于 anchored 表示) | `feature.anchor_clone_timestamp`, `p_FinA` |
| `q̄` (`q_bar`) | JPL 四元数 (`[q_xyz, q_w]`, w 在末尾) | `ov_type::JPLQuat`, `ov_core::quat_multiply` |
| `R` | 旋转矩阵 | `Rot()`, `quat_2_Rot()` |
| `⌊·⌋`, `[·]_×` | 3D 向量的反对称矩阵 (skew) | `ov_core::skew_x(v)` |

> 命名规则: `R_AtoB` 表示"从 A 系到 B 系的旋转", `p_XinY` 表示"X 点在 Y 系下的坐标"。
> 这个约定在源码里非常一致, 读函数签名时可以一眼判断。

---

## 2. JPL 四元数 (scalar-last)

### 2.1 定义
OpenVINS 全程使用 **JPL 约定**, `q̄ = [q₁, q₂, q₃, q₄]ᵀ`, `q₄` 为实部 (Hamilton 约定是 `q₀` 在首位, 完全相反)。

```
x_idx = 0, 1, 2  (虚部 / vector part)
  idx = 3        (实部 / scalar part)
```

代码实证 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/IMU.h" lines="49-51" />:
```cpp
Eigen::VectorXd imu0 = Eigen::VectorXd::Zero(16, 1);
imu0(3) = 1.0;  // 实部 = 1, 代表单位旋转
```

⚠️ 如果你习惯 ROS 的 `geometry_msgs/Quaternion` (那是 Hamilton 约定), 传进 OpenVINS 前需要重新排列或用 `ov_core::quat_multiply` 等函数, 否则旋转会反。

### 2.2 乘法顺序与误差状态
JPL 约定下, 姿态更新是**左乘**:

```
q̄ = δq̄ ⊗ q̂      (δq̄ ≈ [½δθ, 1]ᵀ, 对应旋转 ≈ I - ⌊δθ⌋)
```

对应代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/JPLQuat.h" lines="114-125" />:
```cpp
Eigen::Matrix<double, 4, 1> dq;
dq << .5 * dx, 1.0;             // 小扰动四元数 [½δθ, 1]
dq = ov_core::quatnorm(dq);
set_value(ov_core::quat_multiply(dq, _value));  // q̄ = δq̄ ⊗ q̂
```

误差状态用 **3 维** `δθ ∈ ℝ³` (不是 4 维), 所以协方差矩阵里旋转部分是 `3×3`, 这就是 `ov_type::JPLQuat` 继承 `Type(3)` 的原因 (<ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/JPLQuat.h" lines="95-100" />)。

### 2.3 可交换的几个工具函数
位于 `ov_core/src/utils/quat_ops.h`:

| 数学符号 | C++ 函数 |
|---|---|
| `R(q)` | `quat_2_Rot(q)` |
| `q₁ ⊗ q₂` | `quat_multiply(q1, q2)` |
| `exp_so3(θ)` | `exp_so3(Eigen::Vector3d)` |
| `log_so3(R)` | `log_so3(Eigen::Matrix3d)` |
| `Ω(ω)` | `Omega(w)` (四元数右乘矩阵, 用于积分) |
| `⌊v⌋` | `skew_x(v)` |

---

## 3. 误差状态 (error-state) EKF

### 3.1 为什么不直接用全状态
旋转属于 `SO(3)` 流形, 加减没有意义; 协方差矩阵对 4 维 `q̄` 做高斯分布会违反 `|q̄|=1` 约束。
解决方案: **均值用 4 维四元数, 协方差用 3 维 δθ**。其他变量 (`p, v, bg, ba, p_F`) 是向量, 误差就是普通减法。

### 3.2 关键等式 (只有 3 条, 背下来)

(a) **状态注入** (一次 EKF 更新后把 `δx` 注回 `x̂`):
```
q̂⁺ = δq̄(δθ) ⊗ q̂⁻         # 旋转: 左乘小四元数
p̂⁺ = p̂⁻ + δp                # 向量: 直接加
……                          # 其他向量变量同理
```
代码 — `IMU::update` <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/IMU.h" lines="78-96" />
输入 `dx` 是 15 维 `(δθ, δp, δv, δbg, δba)`, 前 3 维走四元数更新, 后 12 维直接 `+=`。

(b) **协方差预测**:
```
P⁺ = Φ P⁻ Φᵀ + Q_d         # (离散时间, Φ = I + F·dt 或 Φ = exp(F·dt))
```
代码 — `Propagator::propagate_and_clone` <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="85-110" />
`Phi_summed` 累乘, `Qd_summed` 带 `Φ·Q·Φᵀ + Q_i` 递推。

(c) **EKF 更新** (标准 Kalman 增益):
```
K = P Hᵀ (H P Hᵀ + R)⁻¹
δx̂ = K r
P⁺ = (I - K H) P (I - K H)ᵀ + K R Kᵀ
```
代码 — `StateHelper::EKFUpdate` (搜 `ov_msckf/src/state/StateHelper.cpp`)。

> **为什么 Joseph form?** `(I-KH) P (I-KH)ᵀ + K R Kᵀ` 对舍入误差更稳定, 保证 P 半正定; 而简单的 `(I-KH)P` 在数值上有可能变得非对称。OpenVINS 选择了 Joseph 形式。

---

## 4. SO(3)/SE(3) Lie 小抄

### 4.1 常用恒等式
```
exp_so3(θ) = I + sin|θ|/|θ| · ⌊θ⌋ + (1-cos|θ|)/|θ|² · ⌊θ⌋²     # Rodrigues
⌊R·v⌋ = R ⌊v⌋ Rᵀ                                              # 伴随关系
R(δθ) ≈ I - ⌊δθ⌋          (左乘小扰动)
δ(R·p) ≈ -R ⌊p⌋ δθ + R δp (即误差状态对 `Rp` 求偏导)
```
最后一行直接对应雅可比计算, 你会在 `UpdaterHelper.cpp` 里看到 `-R * skew_x(p)` 反复出现。

### 4.2 Trawny Eq.(101) — 四元数 0 阶积分
```
q̄(t+dt) = [cos(½|ω|dt) · I + sin(½|ω|dt)/|ω| · Ω(ω)] q̄(t)
```
代码 — `predict_mean_discrete` <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="525-532" />。
如果 `|ω| ≈ 0`, 退化为 `I + ½·dt·Ω(ω)` (see line 528).

---

## 5. OpenVINS 滤波器的 15 维 IMU 误差状态

```
δx_imu = [δθ_G_I, δp_IinG, δv_IinG, δb_g, δb_a]   ∈ ℝ¹⁵
```

字段 | 语义 | 代码索引 (当 `imu->id() = 0`)
--- | --- | ---
`δθ` | 旋转误差 | row 0..2
`δp` | 位置 | row 3..5
`δv` | 速度 | row 6..8
`δb_g` | 陀螺零偏 | row 9..11
`δb_a` | 加计零偏 | row 12..14

> 这个顺序只在 `ov_type::IMU::set_local_id` 里定义 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/IMU.h" lines="64-70" />。
> 注意 `Propagator::propagate_and_clone` 里出现的 `imu_intrinsic_size() + 15`, `15` 就是这个 IMU 误差状态大小。

**扩展**: 若开启 `do_calib_imu_intrinsics`, 会额外加 `Dw, Da` (6+6=12) 加 `R_GYROtoIMU` (3) 共 +15; 若再开 `do_calib_imu_g_sensitivity` 再 +9 (即 Tg 矩阵)。这就是 `State::imu_intrinsic_size()` 的返回值 (<ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/State.h" lines="126-135" />)。

---

## 6. FEJ (First-Estimates Jacobian) 一句话版

**问题**: 非线性 EKF 在同一状态变量上多次线性化会引入错误的"观测性", 导致偏航 (yaw) 方向出现虚假的信息增益, 最终滤波器过于自信然后发散。

**做法**: 所有涉及雅可比的线性化点 (linearization point) **只用第一次 propagate 时的估计**, 之后不再跟随均值更新。均值照常更新, 只有 Jacobians 锁死。

代码体现 (到处都是):
```cpp
// UpdaterHelper.cpp:89-96  (本 PR 内容)
if (state->_options.do_fej) {
    Eigen::Vector3d p_FinG_best = R_GtoI.transpose() * R_ItoC.transpose() * (feature.p_FinA - p_IinC) + p_IinG;
    R_GtoI = state->_clones_IMU.at(feature.anchor_clone_timestamp)->Rot_fej();   // ← 用 FEJ 值
    p_IinG = state->_clones_IMU.at(feature.anchor_clone_timestamp)->pos_fej();
    p_FinA = (R_GtoI.transpose() * R_ItoC.transpose()).transpose() * (p_FinG_best - p_IinG) + p_IinC;
}
```

每个 `Type` 都维护两套值 — `_value` (均值) 和 `_fej` (首次线性化点), 访问器分别是 `Rot()` / `Rot_fej()`, `pos()` / `pos_fej()` 等。

详细推导参考官方 `docs/fej.dox` 以及论文 *Huang et al., "Observability-based consistent EKF estimators for multi-robot cooperative localization"*。

---

## 7. 快速对照表: "我在公式里看到 X, 代码在哪?"

| 公式 | 代码位置 |
|---|---|
| `q̂ ⊗ δq̄` | `quat_multiply(dq, quat())` |
| `R(q̂)` | `quat_2_Rot(q)` 或 `_imu->Rot()` |
| `⌊v⌋` | `skew_x(v)` |
| `exp_so3(θ)` | `exp_so3(theta)` |
| Trawny Eq.(101) 四元数积分 | `Propagator::predict_mean_discrete` (<ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" />:516) |
| `F, G` 连续时间雅可比 | `Propagator::compute_F_and_G_discrete / _analytic` |
| `Φ, Q_d` 离散化 | `predict_and_compute` 结尾 (<ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" />:482) |
| `P⁺ = Φ P Φᵀ + Q_d` | `StateHelper::EKFPropagation` |
| `K = P Hᵀ (HPHᵀ+R)⁻¹; P⁺ = Joseph(...)` | `StateHelper::EKFUpdate` |
| 像素残差 `r = uv_m - π(p_FinC)` | `UpdaterHelper::get_feature_jacobian_full` (<ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" />:348) |
| `H_f` 左零空间投影 | `UpdaterHelper::nullspace_project_inplace` (UpdaterHelper.cpp:426) |
| QR 测量压缩 | `UpdaterHelper::measurement_compress_inplace` (UpdaterHelper.cpp:456) |

> **阅读建议**: 每次打开 `UpdaterHelper.cpp` 或 `Propagator.cpp`, 先对着这张表快速过一遍 — 你会发现"数学->代码"的翻译其实只有十来个核心惯用法在反复使用。

---

## 8. 进一步阅读

1. 官方英文 docs (`docs/*.dox`) → 渲染版 <https://docs.openvins.com/>
2. 配套论文: Geneva et al., *OpenVINS: A Research Platform for Visual-Inertial Estimation*, ICRA 2020.
3. 基础: Sola, *Quaternion kinematics for the error-state Kalman filter*, arXiv:1711.02508 — 配合中文知乎笔记阅读即可。
4. **接下来的章节会反复引用本文的公式编号**, 所以如果你对上面某节有困惑建议先把它消化。
