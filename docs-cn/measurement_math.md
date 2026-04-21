# MSCKF 观测与更新 — 公式→代码直解

> **前置**: [`math_foundations.md`](./math_foundations.md), [`propagation_math.md`](./propagation_math.md), [`state_and_cov.md`](./state_and_cov.md).
> **官方数学**: `docs/update.dox`, `docs/update-feat.dox`, `docs/update-null.dox`, `docs/update-compress.dox`.
> **主文件**: <ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" /> (487 行), <ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp" /> (316 行)

---

## 0. 大图: 一次 MSCKF 更新的 7 步

```
Step 1 清理特征 (删除没跟踪上 clone 的测量)
Step 2 收集所有 clone 的相机位姿 (R_GtoCi, p_CioinG)
Step 3 三角化 + 高斯牛顿精修 → p_FinG
Step 4 对每个特征: 算雅可比 H_x, H_f, res → 左零空间投影 → 卡方检验
Step 5 拼大矩阵 + QR 压缩 (去掉测量数量 >> 状态维度时的冗余行)
Step 6 EKF 更新: K, δx̂, Joseph 协方差
Step 7 把特征标 to_delete
```

直接打开 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp" lines="71-316" /> 的 `UpdaterMSCKF::update` 对照, 每段代码前都有相应的 `// N.` 行号标注。

---

## 1. 像素观测模型 (Step 3 前的铺垫)

### 1.1 投影链
对于第 `i` 个 clone 上的第 `m` 次测量, 投影路径为:
```
p_FinCi = R_ItoC · R_GtoIi (p_FinG - p_IiinG) + p_IinC          [3D 点在 {C_i}]
uv_norm = [p_FinCi.x / p_FinCi.z,  p_FinCi.y / p_FinCi.z]       [归一化像平面]
uv_dist = distort(uv_norm, intrinsics[cam_id])                   [径向/切向 或 鱼眼模型]
res     = uv_measured - uv_dist                                  [像素残差]
```

代码 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="329-348" />:
```cpp
Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_IiinG);
Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
Eigen::Vector2d uv_norm;
uv_norm << p_FinCi(0)/p_FinCi(2), p_FinCi(1)/p_FinCi(2);

Eigen::Vector2d uv_dist = state->_cam_intrinsics_cameras.at(pair.first)->distort_d(uv_norm);
Eigen::Vector2d uv_m;
uv_m << feature.uvs[pair.first].at(m)(0), feature.uvs[pair.first].at(m)(1);
res.block(2*c, 0, 2, 1) = uv_m - uv_dist;
```

### 1.2 从 IMU clone → 相机克隆

注意 clone 存的是 `{G}→{I_i}` 的位姿, **相机位姿**是合成的:
```
R_GtoCi  = R_ItoC · R_GtoIi
p_CiinG  = p_IiinG - R_GtoCiᵀ · p_IinC        (对照 UpdaterMSCKF.cpp:121-122)
```

所以特征雅可比里会反复看到 `R_ItoC * R_GtoIi` 这个复合旋转, 它对应代码中的 `dpfc_dpfg`。

---

## 2. 雅可比 `H_f, H_x` (链式法则一步步拆)

### 2.1 对特征位置 `p_FinG` (或其他表示参数)

```
∂res / ∂p_FinG = ∂res/∂uv_dist · ∂uv_dist/∂uv_norm · ∂uv_norm/∂p_FinCi · ∂p_FinCi/∂p_FinG
               = (-I)            · dz_dzn            · dzn_dpfc          · R_ItoC · R_GtoIi
```

代码 (完全一一对应) — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="365-389" />:
```cpp
Eigen::MatrixXd dz_dzn, dz_dzeta;
cam->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta);

Eigen::MatrixXd dzn_dpfc(2, 3);
dzn_dpfc << 1/p_FinCi(2),             0, -p_FinCi(0)/p_FinCi(2)²,
            0,             1/p_FinCi(2), -p_FinCi(1)/p_FinCi(2)²;

Eigen::MatrixXd dpfc_dpfg = R_ItoC * R_GtoIi;

Eigen::MatrixXd dz_dpfc = dz_dzn * dzn_dpfc;
Eigen::MatrixXd dz_dpfg = dz_dpfc * dpfc_dpfg;

H_f.block(2*c, 0, 2, H_f.cols()) = dz_dpfg * dpfg_dlambda;   // 链式最后一段: p_FinG → 特征表示参数
```

`dpfg_dlambda` 是从 "特征表示参数" (`GLOBAL_3D` 时就是单位矩阵, `ANCHORED_*` 时是另一个 3×3 或 3×1 矩阵) 到 `p_FinG` 的偏导, 代码 <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="32-190" />。
这就是为什么 OpenVINS 能支持 `GLOBAL_3D`, `GLOBAL_INVERSE_DEPTH`, `ANCHORED_3D`, `ANCHORED_FULL_INVERSE_DEPTH`, `ANCHORED_MSCKF_INVERSE_DEPTH`, `ANCHORED_INVERSE_DEPTH_SINGLE` 六种表示 — 只有 `dpfg_dlambda` 变, 其他链不变。

### 2.2 对 clone 位姿 `[δθ, δp]` (6 维)

```
∂p_FinCi / ∂δθ_Ii = R_ItoC · ⌊p_FinIi⌋      ← 误差状态 θ 变化导致特征位置的转动
∂p_FinCi / ∂δp_Ii = - R_ItoC · R_GtoIi      ← clone 平移的影响
```

代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="376-392" />:
```cpp
Eigen::MatrixXd dpfc_dclone(3, 6);
dpfc_dclone.block(0, 0, 3, 3) = R_ItoC * skew_x(p_FinIi);       // ∂/∂δθ
dpfc_dclone.block(0, 3, 3, 3) = -dpfc_dpfg;                     // ∂/∂δp

H_x.block(2*c, map_hx[clone_Ii], 2, clone_Ii->size()) = dz_dpfc * dpfc_dclone;
```

> **为什么是 `+ R_ItoC · ⌊p_FinIi⌋` 而不是 `- ...`?** 因为 JPL 的误差状态约定是**左扰动** (`R = R̂ (I - ⌊δθ⌋)` 等价于 `R̂ᵀ (I + ⌊δθ⌋)`), 推导完链式法则后正号。若你阅读其他使用 Hamilton 右扰动的代码库, 这里会有符号差异, 不要混用。

### 2.3 对相机外参 `R_ItoC, p_IinC` (若 `do_calib_camera_pose=true`)

代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="404-413" />:
```cpp
if (state->_options.do_calib_camera_pose) {
    Eigen::MatrixXd dpfc_dcalib(3, 6);
    dpfc_dcalib.block(0, 0, 3, 3) = skew_x(p_FinCi - p_IinC);
    dpfc_dcalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    H_x.block(2*c, map_hx[calibration], 2, 6) += dz_dpfc * dpfc_dcalib;
}
```

### 2.4 对畸变参数 (若 `do_calib_camera_intrinsics=true`)

代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="416-418" />:
```cpp
if (state->_options.do_calib_camera_intrinsics) {
    H_x.block(2*c, map_hx[distortion], 2, distortion->size()) = dz_dzeta;
}
```
`dz_dzeta` 来自 `cam->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta)`, 每种相机模型 (`CamRadtan`, `CamEqui`) 有各自实现, 位于 `ov_core/src/cam/`。

---

## 3. 左零空间投影 (MSCKF 的灵魂)

### 3.1 为什么需要它

对第 `k` 个特征, 单独的线性化系统是:
```
r_k = H_x_k · δx + H_f_k · δp_F_k + n_k     (n_k ~ N(0, σ²I))
```

如果让 `δp_F_k` 进状态, 每个 MSCKF 特征会贡献 1~3 维, 几十个特征就是几百维, `_Cov` 急剧膨胀。
**MSCKF 的做法**: 找一个矩阵 `N_L` 满足 `N_Lᵀ · H_f_k = 0` (左零空间), 左乘消掉 `H_f`:
```
N_Lᵀ · r_k = (N_Lᵀ · H_x_k) · δx + N_Lᵀ · n_k
```
新残差仍然是零均值高斯, 噪声协方差仍是 `σ²I` (因为 `N_L` 是正交的)。结果: 维度从 `2 m` 降为 `2m - 3` (或 `2m - 1` 对 SINGLE 表示), **特征自由度彻底解耦**。

### 3.2 代码实现 — Givens 旋转

OpenVINS **不**直接 SVD 求零空间, 而是用 Givens 旋转把 `H_f` 原地变成上三角, 同时对 `H_x, res` 做一样的旋转。顶部三角块被抛弃, 剩下就是零空间投影结果。

代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterHelper.cpp" lines="426-454" />:
```cpp
void UpdaterHelper::nullspace_project_inplace(Eigen::MatrixXd &H_f,
                                              Eigen::MatrixXd &H_x,
                                              Eigen::VectorXd &res) {
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_f.cols(); ++n) {                         // 对 H_f 的每一列 (最多 3 列)
        for (int m = (int)H_f.rows()-1; m > n; m--) {               // 从底往上做 Givens
            tempHo_GR.makeGivens(H_f(m-1, n), H_f(m, n));
            (H_f.block(m-1, n, 2, H_f.cols()-n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (H_x.block(m-1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
            (res.block(m-1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        }
    }
    // 抛弃 H_f 的前 rows (=H_f.cols()) 行
    H_x = H_x.block(H_f.cols(), 0, H_x.rows()-H_f.cols(), H_x.cols()).eval();
    res = res.block(H_f.cols(), 0, res.rows()-H_f.cols(), res.cols()).eval();
}
```

算法出自 *Golub & Van Loan, Matrix Computations 4e*, Alg. 5.2.4 (代码注释里有引用)。

### 3.3 卡方 (chi-square) 残差门控

雅可比算完后要对每个特征做 `\chi²` 检验, 剔除外点:
```
S     = H_x · P_marg · H_xᵀ + σ²·I
chi2  = rᵀ · S⁻¹ · r
通过: chi2 < threshold(dof)
```
代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp" lines="228-253" />:
```cpp
Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
double chi2 = res.dot(S.llt().solve(res));

double chi2_check = chi_squared_table[res.rows()];      // 0.95 分位, dof = res.rows()
if (chi2 > _options.chi2_multipler * chi2_check) {
    (*it2)->to_delete = true;                            // 拒绝该特征
    ...
}
```
`chi2_multipler` 在 config 里默认 1, 可放宽。若你数据集比较"抖" (比如激烈运动), 把它调到 5~10 常常能保住更多特征。

---

## 4. QR 测量压缩 (update-compress.dox)

### 4.1 问题与方案

拼完大矩阵后可能有 `m = 1000` 行, `n = 100` 列。`K = P Hᵀ (H P Hᵀ + R)⁻¹` 要做 `m×m` 的求逆, 代价 `O(m³)`。

OpenVINS 做 **薄 QR 分解**:
```
H_x = Q · [R; 0]
令 T_H = Qᵀ H_x, T_r = Qᵀ r, T_R = Qᵀ R Q (各项同时左乘 Qᵀ)
```
由于下方全零, 只保留前 `n` 行:
```
(上 n 行)  T_H_top · δx = T_r_top + n'     (n' ~ N(0, T_R_top))
```
求逆变成 `n×n`, 代价 `O(n³)`。对 1000×100 的情形加速 `(1000/100)³ = 1000 倍`, 非常显著。

### 4.2 代码实现

仍然用 Givens 原地分解:
```cpp
// UpdaterHelper::measurement_compress_inplace (UpdaterHelper.cpp:456-486)
if (H_x.rows() <= H_x.cols()) return;                    // 已经是瘦矩阵, 无需压缩
for (int n = 0; n < H_x.cols(); n++)
    for (int m = (int)H_x.rows()-1; m > n; m--) {
        tempHo_GR.makeGivens(H_x(m-1, n), H_x(m, n));
        H_x.block(m-1, n, 2, H_x.cols()-n).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        res.block(m-1, 0, 2, 1).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
int r = std::min(H_x.rows(), H_x.cols());
H_x.conservativeResize(r, H_x.cols());
res.conservativeResize(r, res.cols());
```

调用位置 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp" lines="293-298" />:
```cpp
UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
if (Hx_big.rows() < 1) return;
```

**为什么左零空间与 QR 压缩用同一套 Givens?** 两者在数学上都是"找正交矩阵左乘"。实现可以同一套代码模板, 只是被压缩/被消去的是不同块 (前者消 H_f 上三角, 后者消 H_x 下方零块)。

---

## 5. EKF 更新 (update.dox)

代码 — <ref_snippet file="/home/ubuntu/repos/open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp" lines="302-307" />:
```cpp
Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());
StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
```

`StateHelper::EKFUpdate` 内部做:
```
K = P Hᵀ (H P Hᵀ + R)⁻¹
δx̂ = K · r
P⁺ = (I - K H) P (I - K H)ᵀ + K R Kᵀ        [Joseph form]
```
然后把 `δx̂` 按 `Hx_order_big` 里每个 `Type` 的位置切片, 调各自的 `update(dx)` 注回均值。

> **为什么 Joseph?** 数值上更稳定; 对存在线性化误差的 EKF, Joseph form 保证 `P` 始终对称半正定。更多背景: Anderson & Moore, *Optimal Filtering* §6.5。

---

## 6. 与 SLAM 特征更新的区别

MSCKF 特征 **不进状态**, 每帧算完就抛弃 → `to_delete = true`。
SLAM 特征 **进状态**, 协方差多几行几列, 后续观测可以反复利用 (类似 EKF-SLAM)。

流程差异:
- `UpdaterSLAM::change_anchors` / `update` / `delayed_init` 见 `ov_msckf/src/update/UpdaterSLAM.cpp`。
- 初始化新 SLAM 特征用的是 **delayed initialization** (官方 `update-delay.dox`): 先按 MSCKF 算 `H_x, H_f, res`, 但不消 `H_f`, 而是把 `H_f` 作为新状态扩维的基础。

---

## 7. 速查卡

| 需求 | 函数 + 行号 |
|---|---|
| 像素残差 `uv - π(p_FinCi)` | `UpdaterHelper.cpp:329-348` |
| 链式法则 H_f / H_x | `UpdaterHelper.cpp:365-422` |
| ∂/∂clone δθ 用 skew(p_FinIi) | `UpdaterHelper.cpp:377-379` |
| 畸变 Jacobian `dz_dzeta` | `UpdaterHelper.cpp:365-367, 416-418` + `CamRadtan/CamEqui::compute_distort_jacobian` |
| 左零空间 Givens | `UpdaterHelper.cpp:426-454` |
| QR 测量压缩 | `UpdaterHelper.cpp:456-486` |
| 卡方外点剔除 | `UpdaterMSCKF.cpp:228-253` |
| EKF 更新入口 | `UpdaterMSCKF.cpp:306`, 实现在 `StateHelper::EKFUpdate` |
| FEJ 分支 (`p_FinG_fej`) | `UpdaterHelper.cpp:354-363` |

---

## 8. 调试时最有效的 3 个 print

1. 打印每个特征的 `chi2 / chi2_check`, 看是不是阈值过严:
   ```cpp
   PRINT_DEBUG("featid=%d, chi2=%.3f thr=%.3f\n", feat.featid, chi2, chi2_check);
   ```
2. 打印压缩前后的 `H_x` 维度:
   ```cpp
   PRINT_DEBUG("before: H %dx%d  after: H %dx%d\n", max_meas_size, max_hx_size, Hx_big.rows(), Hx_big.cols());
   ```
3. 检查更新后 `_Cov` 对称性:
   ```cpp
   double asym = (state->_Cov - state->_Cov.transpose()).norm();
   PRINT_DEBUG("Cov asymmetry = %.3e\n", asym);
   ```

跑一次 EuRoC 你会发现这些数据几乎贯穿整个 MSCKF 的行为, 是 debug "为什么发散"的头号武器。
