# 低频 Flex shadow 四飞最终结论（experimental）

最终状态：**SHADOW_FAILED**。持续相对姿态观察器和唯一允许的固定延迟
SO(3) TV/change-point 备用方案都未达到硬门槛，因此没有接入 rosfree，也没有
修改 VIO/D455 姿态、P/V、gyro bias、Camera--IMU 外参或 covariance。

## 采用的方法

主方案从每条记录自己的 nominal `R_I_from_B` 出发，使用保存的 VIO 姿态增量
和 FC 相邻相对旋转递推观测 mount level：

`R_obs[k+1] = Delta_R_I[k] R_obs[k] Delta_R_B[k]^T`。

非交换三轴、固定 mount、时变 mount、两阶段确认、平滑释放、cooldown 和 FC
缺失共 6 个主方案合成测试通过。正式 detector 使用共同参数：15 s candidate、
独立 15 s confirmation、30 s cooldown、5 s release。既有 KLT cache 只作为
视觉/gyro 质量 gate，没有重新运行第二套 KLT。

主方案失败后只运行了一个备用方案：对同一 `R_obs(t)` 做 45 s 因果固定延迟
的鲁棒单 change-point/TV 分段常值比较，pre/post level 均至少 15 s，最短 dwell
30 s。没有再发明第三种模型，也没有按 fly 调参。

## fly3 两个指定事件

### 760–800 s

主方案在 763.334 s 形成 candidate，763.334–778.537 s confirmation，778.537 s
正式 correction；估计的 level change start 却是 699.303 s，不在指定分歧起点。
补正量为 `[2.6089, 0.9688, -1.6781] deg`，角度 3.2497°。离线 FC body
评价显示补正后误差增加 0.9879°，所以方向错误，不能接受。窗口内观测 level
仅从 `[2.4149, 1.1500, -1.4314] deg` 变到
`[2.5900, 1.1429, -2.1809] deg`。

备用 TV 方案在该窗口为 0 candidate、0 correction：稳定 change-point 证据不足。

### 995–1025 s

主方案和备用方案均为 0 correction。窗口内观测 level 从
`[-0.1383, 2.8312, -3.0999] deg` 变到
`[-0.5846, 2.9312, -3.4959] deg`；在冻结的 1.05° noise floor、稳定度和 TV
代价约束下不能构成持续新 level。

## 四飞结果

主方案：

| flight | candidate | correction | 正常区 correction | 误触发/min | body median baseline→shadow | body P95 baseline→shadow |
|---|---:|---:|---:|---:|---:|---:|
| fly1 | 16 | 0 | 0 | 0.0000 | 4.484°→4.484° | 21.127°→21.127° |
| fly2 | 26 | 1 | 1 | 0.0375 | 3.403°→3.749° | 5.529°→7.192° |
| fly3 | 18 | 3 | 2 | 0.1045 | 4.711°→4.318° | 25.242°→22.533° |
| fly4 | 10 | 0 | 0 | 0.0000 | 5.202°→5.202° | 10.796°→10.796° |

正常区 correction 合计 3 次，超过最多 1 次；fly2 正常姿态明显退化。所有
correction 间隔不少于 30 s。运行时间 fly1–fly4 分别为
40.73/31.70/43.08/57.10 s，RSS 为 192.2/211.8/238.3/276.4 MB。

备用方案四飞均为 0 candidate、0 correction、0 误触发，body attitude 严格等于
原 baseline，但两个 fly3 事件也都没有有效补正，因此仍失败。四轮 LOFO 使用
相同参数 hash，均无灾难性退化；这不能弥补目标事件没有通过。

FC 缺失审计为 0 candidate、0 correction、target/applied mount 按位保持；真实
四飞 invalid pair 上也没有 target change。所有输入 VIO 轨迹、bias、配置和诊断
文件在运行前后 SHA256 完全一致。

## 结果与决定

- 主方案结果：
  `flex_level_observer_shadow_stride12_final_20260719_022606`。
- 备用方案结果：`flex_level_tv_fallback_stride12_20260719_022031`。
- 真实曲线：各结果目录的 `FLY3_REQUIRED_EVENTS_*.png` 及四飞逐飞 PNG。
- 完整 lineage、命令和 SHA256：
  `analysis/run_specs/flex_level_shadow_experiment_20260719.json`。
- 上一版 hand--eye 失败目录和参数 hash 保持只读，未改 gate。

下一步唯一建议：**不要继续放宽 gate 或让 FC attitude 进入导航；先证明相对
mount 观测在用户指定分歧起点具有可辨识、可重复且方向正确的 level shift。**
当前数据中第一处可接受候选追溯到 699 s，第二处窗口内变化低于冻结 noise
floor；在这个观测合同下继续补正会把正常运动/VIO 相对误差误当成 flex。
