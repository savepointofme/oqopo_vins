# 特征三角化与精化 — `FeatureInitializer`

> **前置**: [`measurement_math.md`](./measurement_math.md) §1 (投影模型); [`math_foundations.md`](./math_foundations.md).
> **官方数学**: `docs/update-featinit.dox`.
> **主文件**: <ref_file file="/home/ubuntu/repos/open_vins/ov_core/src/feat/FeatureInitializer.cpp" /> (423 行)

## 0. 概览

MSCKF / SLAM 两条线都需要"在多视几何里把一个特征点的 3D 位置估出来", 这就是 `FeatureInitializer` 的工作:

```
single_triangulation         : 3D 线性三角化 (DLT, 封闭解)
single_triangulation_1d      : 1D 深度三角化 (固定 bearing, 只解深度)
single_gaussnewton           : 在 DLT 结果基础上做非线性精化
```

三个函数都带一个相同的参数: `clonesCAM = {cam_id → {timestamp → ClonePose(R_GtoCi, p_CiinG)}}` — 即所有 clone 在全局系下的相机位姿。

---

## 1. 3D 线性三角化 (`single_triangulation`)

### 1.1 几何推导
给定 `N` 个视角的归一化 bearing `b_i ∈ ℝ³` (单位向量) 和相机中心 `C_i`:
```
点 p 在某个视角的真实 bearing = (p - C_i) / ||p - C_i||
理论上  (p - C_i) ∥ b_i   →   ⌊b_i⌋ · (p - C_i) = 0
```
`⌊·⌋` 是 3×3 反对称矩阵 (skew), 秩 2。把 N 个视角堆叠:
```
min_p  Σᵢ || ⌊b_i⌋ (p - C_i) ||²
```
令 `B_i⊥ = ⌊b_i⌋`, `A_i = B_i⊥ᵀ B_i⊥`, 上式等价于解:
```
A · p = b,  其中  A = Σ A_i,  b = Σ A_i · C_i
```

### 1.2 代码对照 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/feat/FeatureInitializer.cpp" lines="48-88" />

```cpp
Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
Eigen::Vector3d b = Eigen::Vector3d::Zero();

for (auto const &pair : feat->timestamps) {
    for (size_t m = 0; m < feat->timestamps.at(pair.first).size(); m++) {
        ...
        Eigen::Matrix<double, 3, 1> b_i;
        b_i << feat->uvs_norm.at(pair.first).at(m)(0),
               feat->uvs_norm.at(pair.first).at(m)(1),
               1;
        b_i = R_AtoCi.transpose() * b_i;
        b_i = b_i / b_i.norm();
        Eigen::Matrix3d Bperp = skew_x(b_i);

        Eigen::Matrix3d Ai = Bperp.transpose() * Bperp;
        A += Ai;
        b += Ai * p_CiinA;
    }
}

Eigen::MatrixXd p_f = A.colPivHouseholderQr().solve(b);       // A·p = b
```

注意: 所有计算都在 **anchor 相机系** `{A}` 里做 — 代码开头挑选观测最多的相机/时刻作为 anchor (`anchor_most_meas`), 把其他 `C_i` 和 `b_i` 都转到 `{A}` 下, 数值稳定性更好。

### 1.3 失败判定
```cpp
double condA = singularValues(0,0) / singularValues.rows()-1,0);
if (|condA| > max_cond_number || p_f(2) < min_dist || p_f(2) > max_dist)
    return false;
```
条件数大 = 多视角几乎共线 (基线太短), 深度不可信。
`p_f(2)` = anchor 系下的 Z 分量, 必须在 `[min_dist, max_dist]` 之间。这些阈值在 `FeatureInitializerOptions` 里。

**默认参数**: `max_cond_number=1e4`, `min_dist=0.10`, `max_dist=60`. 配置在 `config/*/estimator_config.yaml` 的 `featinit_options` 段。

### 1.4 1D 三角化变体

`single_triangulation_1d` 把 bearing 固定为 anchor 帧的观测方向, 只解一个深度标量 `λ`:
```
p = C_A + λ · bearing_A
```
对每个其他视角:
```
proj(C_i + R_AtoCiᵀ · (p - C_i)) ≈ uv_i        (1 维未知数 λ)
```
代码把它拆成标量 `A · λ = b` 解方程。这种模式更鲁棒 — 当相机轨迹几乎是直线时, 3D 三角化会病态, 但 1D 模式只要有足够长的平动基线即可。YAML 开关: `triangulate_1d: true`。

---

## 2. 非线性精化 — 高斯牛顿 (`single_gaussnewton`)

### 2.1 目标
线性三角化最小化的是"到 bearing 线的代数距离", 不是 **真实的像素误差**。精化阶段改为:
```
min_p  Σᵢ || uv_i - π(p_FinCi) ||²
```
其中 `π(·)` 是投影 (归一化 + 畸变)。

### 2.2 参数化 — `(α, β, ρ) = (x/z, y/z, 1/z)_A`
在 anchor 系里用 **inverse depth**:
- 提高远距点的数值稳定性
- 避免 `z → ∞` 时优化目标发散

对应代码中对 `feat->p_FinA` 的转换参数化, 具体看 `single_gaussnewton` 函数。

### 2.3 高斯牛顿迭代
```
迭代直到收敛:
    J  = ∂res/∂(α,β,ρ)    (3×1 对每个残差)
    H  = Σ JᵢᵀJᵢ
    g  = Σ Jᵢᵀ rᵢ
    Δ  = - H⁻¹ · g
    (α,β,ρ) += Δ · λ      (λ 为 Levenberg 阻尼, 若新残差更大则回退)
```
代码位置: `FeatureInitializer::single_gaussnewton` (从 L197 开始)。

### 2.4 失败判定
迭代不收敛 (步长发散) 或最终重投影残差过大 → 返回 `false`, 特征被标 `to_delete` (调用方处理)。

---

## 3. 与更新器的互动 (为什么两次都要三角化?)

在 `UpdaterMSCKF::update` 的 Step 2~3:
```cpp
if (config.triangulate_1d) success_tri = single_triangulation_1d(...);
else                       success_tri = single_triangulation(...);

if (config.refine_features) success_refine = single_gaussnewton(...);

if (!success_tri || !success_refine) {
    (*it1)->to_delete = true;                       // 放弃
    ...
}
```

在 `UpdaterSLAM` / `UpdaterMSCKF` 里都是同样模式。三角化结果 `p_FinA / p_FinG` 被存回 `feat` 对象, 紧接着在 `UpdaterHelper::get_feature_jacobian_full` 里作为线性化点 (可能 + FEJ 最近点) 使用。

---

## 4. 常见坑

| 现象 | 可能原因 | 调整 |
|---|---|---|
| 大量特征被拒 (三角化失败) | 轨迹近似直线 / 基线太短 | 开 `triangulate_1d: true` |
| 初始化漂移, 前几秒轨迹跳 | anchor 挑错 (观测数最多 ≠ 最稳) | 关 `refine_features` 对比看是否 GN 发散 |
| 遥远特征 (>50m) 全被丢 | `max_dist` 太小 | 调大到 200~500 |
| 近距特征 (<10cm) 全被丢 | `min_dist` 太大 | 保持默认 0.1 (更小会导致发散) |

---

## 5. 速查

| 需求 | 代码位置 |
|---|---|
| DLT 3D 三角化 | `single_triangulation` L30-112 |
| 1D 深度三角化 | `single_triangulation_1d` L114-195 |
| 高斯牛顿精化 | `single_gaussnewton` L197-423 |
| 选 anchor 策略 | L36-46 |
| 条件数判断 | L91-106 |
| 被谁调用 | `UpdaterMSCKF.cpp:139-149`, `UpdaterSLAM.cpp` 同类模式 |
