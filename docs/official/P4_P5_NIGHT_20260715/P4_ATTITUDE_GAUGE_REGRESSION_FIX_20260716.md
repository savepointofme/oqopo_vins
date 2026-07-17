# P4 姿态基准回退修复设计

## 已确认的问题

当前正式 P4 的 Ceres 图只把 FC 位置和速度作为因子。FC 姿态虽然用于状态种子和事后残差计算，却不进入优化，也不进入最终 release gate。单目视觉和 IMU 能约束窗口内相对旋转，但不能独立提供 `G_nav` 的绝对航向基准；因此求解可以在视觉/IMU 残差很小的同时偏离同步 FC 姿态。

同一 C0 配置下，当前 release 的 FC–IMU 姿态残差为 fly1 `2.39 deg`、fly3 `3.05 deg`，但二者仍被标为可信姿态。只在诊断运行中把 P4 姿态基准恢复到同步 FC 姿态后，其他输入和后端不变，焦点段结果变为：

| flight | C0 final XY | diagnostic final XY | C0 course RMSE | diagnostic course RMSE |
| --- | ---: | ---: | ---: | ---: |
| fly1 | 511.51 m | 219.11 m | 4.87 deg | 2.54 deg |
| fly3 | 390.43 m | 94.50 m | 1.60 deg | 1.60 deg |

该诊断只用于定位，不能作为正式实现，因为它直接替换了姿态而没有保留联合优化和转弯挠曲鲁棒性。

## 正式修复

1. 在联合图中恢复一个独立的 FC attitude gauge factor。它只连接窗口姿态状态，不与 FC position/velocity 拼成一个残差块。
2. 固定全程标定的 `R_FtoI` 和已接受的姿态时间偏移；不在线修改安装角，不使用 GPS course、GPS yaw 或最终轨迹误差。
3. 姿态因子使用鲁棒损失。正负转弯产生相反瞬态时按残差幅值对称降权，不采用固定方向补偿。
4. candidate 阶段不能再把“FC 姿态仅供诊断”写成可信姿态。release 必须具备联合图中的 FC attitude Jacobian support，并通过因果 holdout 的同步姿态残差检查。
5. `q/bg/ba` 不得在 release 瞬间注入一份从未经过后续数据验证的最终 correction。若需要 material correction，则进入下一前移联合窗口重新估计；`p/v` 仍可用直接 FC position/velocity 量测做因果闭环。

## 验收标准

- 受影响 target 和 P4 单元/集成测试通过。
- fly1、fly3 使用同一正式配置稳定释放，metadata 明确记录 FC attitude factor contribution 和非零 attitude Jacobian support。
- 焦点段同时不劣于当前 C0；不能出现只改善一个转弯方向、恶化另一个方向。
- 通过焦点段后再跑两飞行全程；正式结论使用 GPS update-time、start-heading 主口径和 absolute no-post-alignment 佐证。
- 任何 GPS horizontal/course/yaw 只用于离线评价，不进入 P4 或 OpenVINS 状态更新。
