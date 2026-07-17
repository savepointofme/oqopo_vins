# P4 Kalibr FC-to-board 安装角注入与验收规格

## 目标

只替换 P4 FC 输入声明中的固定 `R_FtoI_calibrated_row_major`，验证直接运行
Kalibr `IccImu.findOrientationPrior()` 得到的 FC-to-board 安装残差是否改善当前
滑窗初始化和后续导航。June12 Camera--IMU 标定、FC 导航时间合同、视觉/IMU 数据、
P4 窗口、释放策略、GPS-Z 模式和后端配置均保持不变。

## 对照

同一代码、数据和回放时间分别运行：

1. `frozen_v3`：现有全程自写标定输入，作为无改动对照；
2. `kalibr_full`：注入每架次 Kalibr 全程旋转矩阵；
3. `kalibr_conservative`：仅沿 Kalibr 残差旋转轴施加保守比例，避免把魔术贴
   随飞行晃动的全部瞬态当作固定安装角。

保守比例由同一 Kalibr 原始函数的前后半程旋转差异决定：

```text
fraction = clamp(1 - half_to_half_rotation_difference / full_residual_angle, 0, 1)
```

当前数据对应 fly1 `0.925939943`、fly3 `0.502952974`。这不是最终生产常数，只是由
当前全程稳定性证据定义的收缩对照；不得用最终 GPS 误差反调该比例。

## 验收

- 首先要求 initializer test 和每个短程 runner 成功，`traj_nav.txt`、滑窗 trace、
  metadata 与 acceptance JSON 完整；
- 比较相同释放窗口的 q/p/v/bg/ba、窗口残差、释放层级和内部 covariance；
- 旧对照退化或新方案不能稳定释放时直接判失败；
- 短程通过后只运行胜出的保守/全量方案长程，并使用正式 GPS-time、
  start-heading 评价；GPS course/yaw 只用于离线评价，绝不进入初始化器。
