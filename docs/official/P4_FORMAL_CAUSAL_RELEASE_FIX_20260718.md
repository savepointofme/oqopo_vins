# P4 正式联合初始化因果释放修正

## 问题证据

原实现先冻结一个 8 秒联合图候选，用约 2 秒后续数据验证；验证通过后却丢弃该候选，重新求解一个向前移动的新窗口，并把这个未经未来数据验证的新结果立即释放。

真实 fly1/fly2/fly3 运行中，首候选的后续速度残差分别为 1.700、1.449、0.837 m/s。二次求解会显著改变速度和 accelerometer bias；fly3 的二次求解联合归一化代价还从 0.2644 退化到 0.3672，但仍被标记为成功释放。释放后的前 2 秒纯 IMU 传播已经形成 1.83、1.11、2.88 m/s 的速度误差，早于首次视觉更新。

因此根因不是 Ceres 运行时间或第一次 MSCKF 更新，而是释放对象与验证对象不一致，并且 3 m/s 的旧速度安全上限不能作为初始化交接精度标准。

## 修正后的生命周期

```text
8 s FC + board IMU + monocular epipolar joint graph
  -> freeze exact q/p/v/bg/ba + 15x15 covariance
  -> collect a disjoint >=2 s FC/IMU/visual holdout
  -> propagate the frozen state with holdout IMU only
  -> compare endpoint q/p/v against synchronized FC and visual epipolar geometry
     -> fail: discard candidate; advance the fixed-time window and solve again
     -> pass: release the exact frozen state and its covariance once
```

验证阶段不向候选反馈 FC，不修改 q/p/v/bg/ba，也不在通过后以一个新求解替换已验证状态。OpenVINS 在当前相机回调中从候选图时间传播到当前时间，因此被注入状态与用于验证的 bias 完全一致。

## 交接精度合同

- attitude：不超过既有 5° 候选物理界。`fc_board_mount_sigma_deg` 只表示锁定安装角的全程拟合精度，不能冒充单窗 FC—VIO 状态残差模型；
- velocity：不超过 FC velocity 的声明 1-sigma；
- position：允许 FC position 1-sigma 加完整 holdout 内由 velocity 1-sigma 产生的位移；
- visual：使用正式的 8 px P95 上限，不再误用仅供硬拒绝的 16 px safety bound；
- transient flex：一方面检查全部优化关键帧相对“锁定安装角 × 同步 FC 姿态”的最大完整姿态残差；另一方面在高角速度样本被普通 rotation fit 排除前，检查全部同步 FC/board-IMU 角速度残差峰值。超过已有 10° 姿态界或 3.5 rad/s 物理角速度界即拒绝整个候选，直到瞬态退出滑窗；
- 时间：holdout 按传感器时间，不按固定量测次数。

这些条件只使用在线可用的 FC、IMU、视觉及声明协方差，不使用 GPS course、未来全程误差或评价真值。

## 验收

1. formal factor、initializer 和静态合同测试通过；
2. attempt receipt 必须包含非默认的 q/p/v/bg/ba、残差和代价；
3. fly1/fly2/fly3 短程均能由 `formal_holdout_verified_release` 释放，且首个后端前纯传播速度误差不超过交接合同；
4. 三条全程使用 GPS update grid、start-heading、FC raw velocity 和 VIO `.bias` 正式评价；
5. 对冻结 global baseline 无稳定优势前，P4 不算通过，P5 不启动。
