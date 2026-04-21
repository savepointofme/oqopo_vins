# IMU 传播公式 → 代码逐行对照

> **前置**: [`math_foundations.md`](./math_foundations.md) §3,5 (误差状态 + 15 维 IMU); [`state_and_cov.md`](./state_and_cov.md).
> **官方数学**: `docs/propagation.dox`, `docs/propagation-discrete.dox`, `docs/propagation-analytical.dox`.
> **主文件**: <ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" /> (1050 行)

---

## 1. IMU 测量模型

官方公式 (propagation.dox 开头):
```
ω_m(t) = ω(t) + R_GYRO·T_g·a(t) + b_g(t) + n_g(t)
a_m(t) = a(t)                     + b_a(t) + n_a(t)
```

对应代码 — `predict_and_compute` <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="442-463" />:
```cpp
Eigen::Vector3d a_hat1 = data_minus.am - state->_imu->bias_a();   // a_m - b_a
...
a_hat1 = R_ACCtoIMU * Da * a_hat1;                                // 轴偏差 + 尺度修正

Eigen::Vector3d w_hat1 = data_minus.wm - state->_imu->bias_g()    // w_m - b_g
                         - Tg * a_hat1;                           // 减掉 Tg·a
...
w_hat1 = R_GYROtoIMU * Dw * w_hat1;                               // 轴偏差 + 尺度修正
```

| 公式变量 | 代码变量 | 解释 |
|---|---|---|
| `a_m, ω_m` | `data_minus.am, .wm` | 原始 IMU 读数 |
| `b_g, b_a` | `_imu->bias_g(), bias_a()` | 当前零偏估计 |
| `D_w, D_a` | `Dw = State::Dm(imu_model, _calib_imu_dw)` | 尺度/非正交矩阵 |
| `T_g` | `Tg = State::Tg(_calib_imu_tg)` | 陀螺对加速度敏感度 (重力漂移) |
| `R_GYROtoIMU` | `_calib_imu_GYROtoIMU` | 陀螺轴↔IMU body 的小旋转 |

> **多数数据集 (EuRoC 包括)** `Dw = Da = I`, `Tg = 0`, `R_GYROtoIMU = I`, 所以上面这堆线代操作其实没做什么; 但开 `do_calib_imu_intrinsics` 后就派上用场。

---

## 2. 状态连续时间运动方程

```
q̇_GtoI = ½ Ω(ω) q
ṗ_IinG = v_IinG
v̇_IinG = R_GtoIᵀ · a - g_G
ḃ_g    = n_bg          (随机游走)
ḃ_a    = n_ba
```

### 2.1 旋转积分 — Trawny Eq.(101)

离散公式:
```
q(t+dt) = [cos(½|ω|dt)·I + sin(½|ω|dt)/|ω| · Ω(ω)] · q(t)
```

代码 — `predict_mean_discrete` <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="525-532" />:
```cpp
if (w_norm > 1e-12) {
    bigO = cos(0.5 * w_norm * dt) * I_4x4 + 1/w_norm * sin(0.5*w_norm*dt) * Omega(w_hat);
} else {
    bigO = I_4x4 + 0.5 * dt * Omega(w_hat);        // 小角度退化
}
new_q = quatnorm(bigO * state->_imu->quat());
```

OpenVINS 还提供 **RK4** 和 **Analytical** 积分, 切换逻辑在 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="473-481" />。RK4 更准但更慢, EuRoC 默认 `DISCRETE`。

### 2.2 速度/位置积分

公式 (零阶 hold on `a`):
```
v(t+dt) = v(t) + Rᵀ·a · dt - g · dt
p(t+dt) = p(t) + v(t) · dt + ½ Rᵀ·a · dt² - ½ g · dt²
```

代码 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="534-538" />:
```cpp
new_v = state->_imu->vel() + R_Gtoi.transpose() * a_hat * dt - _gravity * dt;
new_p = state->_imu->pos() + state->_imu->vel() * dt
      + 0.5 * R_Gtoi.transpose() * a_hat * dt * dt - 0.5 * _gravity * dt * dt;
```

逐字对应。`_gravity = [0, 0, 9.81]` (由 config `gravity_mag` 决定)。

> **注意坐标系**: `R_Gtoi.transpose()` 把 body 系下的 `a_hat` 转到全局系 `{G}`, 因为速度/位置都在 `{G}` 里表达。

---

## 3. 误差状态传播 (F, G, Φ, Q_d)

### 3.1 连续时间 `F`, `G`

官方 `propagation-discrete.dox` 给出 15×15 的 `F` 和 15×12 的 `G`。核心几块:

```
F_θθ  = -⌊ω̂⌋           F_θbg = -I
F_pv  =  I
F_vθ  = -R_GtoIᵀ ⌊â⌋   F_vba = -R_GtoIᵀ
F_bb  =  0              (b_g, b_a 随机游走)

G_θng  = -I
G_vna  = -R_GtoIᵀ
G_bgnbg = G_bana = I
```

对应函数: `compute_F_and_G_discrete` 与 `compute_F_and_G_analytic` (Propagator.cpp 后半段, 可直接搜函数名)。本 PR 在这两个函数上方加了完整块注释说明每个子块对应 `docs/propagation-discrete.dox` 的哪个公式编号。

### 3.2 离散化 → `Φ, Q_d`

连续时间模型到离散时间 (Trawny Eq.129-130):
```
Φ    = exp(F · dt) ≈ I + F · dt + ½ (F·dt)² + ...      (analytic/RK4 用精确版; discrete 用 I+F·dt)
Q_d  = ∫₀^dt Φ(dt-τ) G Q_c Gᵀ Φ(dt-τ)ᵀ dτ ≈ G (Q_c · dt) Gᵀ
```

代码 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="496-505" />:
```cpp
Eigen::Matrix<double, 12, 12> Qc = Eigen::Matrix<double, 12, 12>::Zero();
Qc.block(0, 0, 3, 3) = std::pow(_noises.sigma_w,  2) / dt * I;   // n_g
Qc.block(3, 3, 3, 3) = std::pow(_noises.sigma_a,  2) / dt * I;   // n_a
Qc.block(6, 6, 3, 3) = std::pow(_noises.sigma_wb, 2) / dt * I;   // n_bg
Qc.block(9, 9, 3, 3) = std::pow(_noises.sigma_ab, 2) / dt * I;   // n_ba

Qd = G * Qc * G.transpose();
Qd = 0.5 * (Qd + Qd.transpose());                                // 强制对称
```

> `Qc` 里除以 `dt` 是把 **连续时间 PSD (单位 [rad/s/√Hz]²)** 转成 **离散时间方差**, 然后 `G·Qc·Gᵀ` 在 `dt` 累积区间里得到 `Q_d`。常见推导里会看到 `σ² · dt` (方差), 这里是等价的 — 配置 YAML 给的是连续时间 PSD `σ_w, σ_a` (单位 rad/s/√Hz, m/s²/√Hz)。

---

## 4. 多步累乘: 为什么要 `Phi_summed`?

相机帧之间通常夹 5~20 个 IMU 样本 (`dt_cam ≈ 50ms, dt_imu ≈ 5ms → 10 步`)。如果每步都对整个 `_Cov` 做一次 `Φ P Φᵀ`, 那 10 步就是 10 次大矩阵乘法 — 贵。

优化: 先在 IMU 小循环里**只对 15×15 子块**累乘 `Phi_summed`, 再对整个 `_Cov` 乘一次。

代码 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/Propagator.cpp" lines="85-139" />:
```cpp
Phi_summed = Eigen::MatrixXd::Identity(15+imu_intrinsic_size, ...);
Qd_summed  = Eigen::MatrixXd::Zero(...);

for (i = 0; i < prop_data.size() - 1; i++) {
    predict_and_compute(..., F, Qdi);        // 单步 F, Q
    Phi_summed = F * Phi_summed;             // Φ_tot = Φ_n … Φ_2 Φ_1
    Qd_summed  = F * Qd_summed * F.T + Qdi;  // Q_tot = Φ_n Q_{n-1} Φ_nᵀ + Q_n
    Qd_summed  = 0.5 * (Qd_summed + Qd_summed.T);  // 强制对称
}

StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_summed, Qd_summed);
```

`EKFPropagation` 内部会把 `Phi_summed` 只作用在 `Phi_order` 覆盖的行列 — 即 IMU + IMU 内参块; 对 clones / features / 相机 calib 行列只做**相关性更新** (`P_{cn} = P_{cn} · Φᵀ`), 不改它们自身。

---

## 5. 随机克隆 (Stochastic Cloning)

每次相机帧到来, 在传播结束后调用:

```cpp
StateHelper::augment_clone(state, last_w);   // Propagator.cpp:146
```

这会:
1. 深拷贝当前 `_imu->_pose` 为一个新的 `PoseJPL`, 时间戳 = 当前相机时间。
2. 在 `_variables` 末尾追加, 在 `_Cov` 上做 `conservativeResize(+6, +6)`。
3. 填充新块: `P_new_clone, P_clone_other` 都从 `P_imu_pose` 拷贝 (因为它们此刻完全一样)。

**数学本质**: 把 `PoseJPL_clone = PoseJPL_imu` 写成 `[I ; I] · x̂` 的线性关系, 然后按 EKF 写协方差:
```
[ P_xx   P_xx ]
[ P_xx   P_xx ]
```
新旧两块位姿的协方差完全相同, 直到未来的传播/更新让它们发散。

如果 `do_calib_camera_timeoffset` 开启, 还会考虑 `last_w`:
```
R_clone = exp(-last_w · dt_off) · R_imu
```
这样克隆的位姿是"相机快门实际触发那一瞬间"的估计, 不是 IMU 时钟上那一刻。具体参考 `StateHelper::augment_clone` 实现 (Propagator.cpp:146, 进入后即可). 

---

## 6. 快速自检清单

读完这份文档后, 建议打开 `Propagator.cpp` 直接找到下列位置, 确认你都能看懂:

- [ ] L42-146  `propagate_and_clone` 主流程
- [ ] L330-414 `select_imu_readings` (IMU 样本插值)
- [ ] L429-514 `predict_and_compute` (F, Q 生成)
- [ ] L516-539 `predict_mean_discrete` (离散积分)
- [ ] 搜 `compute_F_and_G_discrete` → 15×15 状态转移核心
- [ ] 搜 `compute_Phi_Qd_analytic`  → 解析版精确 Φ, Q_d 构造

任何一块看不懂, 反过来用本文的数学公式对应着读, 通常就能通。如果还是没懂, 打开 `docs/propagation-discrete.dox` 看完整推导 (公式编号和代码注释是匹配的, 例如代码里 "Equation (129) and (130) of Trawny tech report" 就是指那份 PDF)。

---

## 7. 本章速查表

| 想看的数学 | 打开文件 + 行号 |
|---|---|
| ω̂ = ω_m - b_g - T_g·a 偏差补偿 | Propagator.cpp:454 |
| Trawny Eq.(101) 四元数积分 | Propagator.cpp:525 |
| `v̇ = Rᵀa - g`, `ṗ = v` | Propagator.cpp:534-538 |
| 连续时间 `F, G` | `compute_F_and_G_discrete/analytic` |
| `Φ, Q_d` 离散化 | Propagator.cpp:496-505 |
| 多步累乘 (Phi_summed / Qd_summed) | Propagator.cpp:85-108 |
| `StateHelper::EKFPropagation` | `state/StateHelper.cpp` |
| 随机克隆 `augment_clone` | `state/StateHelper.cpp`, 入口 Propagator.cpp:146 |
