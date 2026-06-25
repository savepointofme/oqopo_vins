# Run Experiment Skill — VIO + LK 并行实验流程

## 用途

在 WSL 中运行 VIO（后台）和 LK 纯视觉轨迹（前台），同步产出轨迹文件，
随后用 `flight_eval_tool.py` 分析。所有结果按统一目录结构存储，实验条件记录完整，
可在未来任意时间回溯重现。

---

## 实验目录结构

```
实验目录/<实验组名>/
  <DATE>_<flight>_<配置关键词>_<成功|失败>/
    data/
      traj.txt            — VIO 轨迹（TUM 格式）
      traj.txt.bias       — VIO 偏置状态（自动生成）
      lk_traj.txt         — LK 纯视觉轨迹（TUM ENU）
      diag.csv            — VIO 诊断（每帧统计）
      vio_yaw_diag.csv    — 偏航更新诊断
      lk_flow.csv         — LK 光流逐帧数据
      vio.log             — VIO 运行日志
      lk.log              — LK 运行日志
    metadata/
      command.txt         — 完整 VIO 运行命令（可重现）
      run_spec.json       — 分析工具输入规格（Windows 路径）
      vio_config.yaml     — 本次使用的 YAML 配置副本
      experiment_summary.md — 实验条件表格（中文摘要）
    plots/                — 分析生成（flight_eval_tool.py 填入）
    tables/               — 分析生成
    reports/              — 分析生成
```

**命名规则：**
- `<实验组名>`: 数据集范围 + 实验类型，如 `NOCAL_NEWCALIB_D455_FLY123`
- `<配置关键词>`: 本次与上次基线的最关键差异，如 `nocal_newcalib_gpsz`
- 状态 `成功/失败` 由脚本自动判断（VIO exit=0 且 traj.txt 非空）

---

## 第一步：运行实验

### 主脚本

```bash
# 在 WSL 中执行
bash /mnt/d/vscode_dir/open_vins/ov_msckf/scripts/run_d455_fly123_nocal.sh fly3
bash /mnt/d/vscode_dir/open_vins/ov_msckf/scripts/run_d455_fly123_nocal.sh fly1 fly2
```

脚本会：
1. 创建 `<实验组>/<RUNNAME>_进行中/data/` 和 `metadata/`
2. 将 VIO 配置复制到 `metadata/vio_config.yaml`
3. 后台运行 VIO，前台运行 LK（并发）
4. VIO 完成后将目录重命名为 `_成功` 或 `_失败`
5. 自动写 `metadata/run_spec.json` 和 `metadata/experiment_summary.md`
6. 打印分析命令

### VIO 关键参数（参考 fly4 accel_allan 成功实验）

| 参数 | 值 | 来源 |
|------|----|------|
| `--vio-yaw-gauge-mode` | `oc_postchi2_current_gauge` | fly4 command.txt |
| `--init-bg-sigma` | 0.003 | fly4 command.txt |
| `--gps-alt-sigma` | 2.0 | fly4 command.txt |
| `--gps-alt-max-res` | 80 | fly4 command.txt |
| `--gps-alt-guard-dxy` | 0.5 | fly4 command.txt |
| `--gps-alt-guard-kxy` | 5.0 | fly4 command.txt |
| `--gps-alt-min-t-after-init` | 10 | fly4 command.txt |

fly1 额外参数：`--init-att-sigma-deg 3 --init-pos-sigma 0.05`（来源：run_gpsz_on_fly1.sh）

> `oc_postchi2_current_gauge` = `global_yaw_oc_projection + alpha=1.0`（高级别别名）

### 各飞行参数速查

| 飞行 | start_time | fi_max_cond | fi_max_dist | GPS 偏移 |
|------|-----------|-------------|-------------|---------|
| fly1 | 930s | 1e4 | 1200m | fc_gps_cam_time.csv (已对齐) |
| fly2 | 700s | 1e5 | 1000m | offset447p5 |
| fly3 | 618s | 1e4 | 1200m | offset438p0 |

---

## 第二步：分析结果

### 标准分析命令

```bash
python3 D:/vscode_dir/open_vins/analysis/flight_eval_tool.py single \
  --run-spec "C:/Users/baloney/Desktop/实验目录/<实验组>/<RUN>/metadata/run_spec.json" \
  --out-root "C:/Users/baloney/Desktop/实验目录/<实验组>/<RUN>"
```

分析工具会在 `<RUN>/plots/`、`<RUN>/tables/`、`<RUN>/reports/` 填入产物。

### run_spec.json 字段说明

```json
{
  "experiment_id": "NOCAL_NEWCALIB_D455_FLY123",
  "flight_name": "fly1",
  "method_name": "nocal_newcalib_gpsz",
  "date": "20260614",
  "status": "success",
  "t0": 930,
  "t1": null,              // 填写: 轨迹有效结束时间（秒）
  "alignment": {"mode": "start_heading", "course_window_s": 60.0},
  "sampling": {"mode": "gps_update_after_vio", "max_delay_s": 0.2},
  "segmentation": {
    "mode": "four_side_lap",
    "min_speed_mps": 5.0,
    "primary_heading_tolerance_deg": 20.0,
    "min_primary_side_len_m": 2000.0
  },
  "velocity_source": "flight_controller_raw",
  "gps_csv": "C:/Users/baloney/...",
  "vio_traj": "C:/Users/baloney/.../data/traj.txt",
  "vio_bias": "C:/Users/baloney/.../data/traj.txt.bias",
  "diag_csv": "C:/Users/baloney/.../data/diag.csv",
  "vio_yaw_diag": "C:/Users/baloney/.../data/vio_yaw_diag.csv",
  "lk_traj": "C:/Users/baloney/.../data/lk_traj.txt",
  "lk_flow_csv": "C:/Users/baloney/.../data/lk_flow.csv"
}
```

**运行后手动填 `t1`**：查看 VIO 日志或 diag.csv 最后有效时间戳，填入 run_spec.json。

---

## 第三步：LK 纯视觉轨迹原理

`analysis/lk_trajectory_gen.py` 读取 EuRoC cam0 图像序列，输出 TUM 格式 ENU 轨迹：

1. GFTT 特征检测 → 金字塔 LK 正向追踪 → 反向验证（FB误差 < 1px）
2. RANSAC 相似变换（平移+旋转+等比缩放）估计相机平面运动
3. **像素→ENU 坐标**（nadir 下视相机）：
   ```
   gx = -tx_px * alt / fx   # 无人机向右位移（相机X轴取反）
   gy = -ty_px * alt / fy   # 无人机向前位移
   dE = gx*sin(θ) + gy*cos(θ)   # θ = GPS 航向（弧度）
   dN = -gx*cos(θ) + gy*sin(θ)
   ```
4. 跳过高度 < 5m 或速度 < 1m/s 的帧
5. 首个有效帧锚定到第一个 GPS 点

**关键 CLI 参数：**

```bash
python3 analysis/lk_trajectory_gen.py \
  --dataset <euroc_dir>        # mav0 或 cam0/data.csv 所在目录
  --gps <gps_csv>              # ENU 或 WGS84 GPS，与相机时间对齐
  --output <lk_traj.txt>       # TUM: t E N U 0 0 0 1
  --flow-csv <lk_flow.csv>     # 逐帧: t dt n_feat n_inliers tx_px ty_px rot_deg alt_m speed_mps disp_m
  --start-time <s>             # 与 VIO 相同
  --min-alt-m 5.0              # 跳过低于此高度的帧（默认5m）
  --min-speed-mps 1.0          # 跳过慢速帧（默认1m/s）
  --fx 367.4568 --fy 367.1397  # D455 焦距（默认已配置）
```

---

## 数据分析工具变更说明（20260614）

从 Claude Design 审计后的 zip 包集成，以下文件已更新：

| 文件 | 变更 |
|------|------|
| `analysis/flight_eval_tool.py` | 主入口，支持 `single` 和 `compare` 子命令 |
| `analysis/flight_eval/io.py` | GPS/VIO/LK 数据加载 |
| `analysis/flight_eval/plotting.py` | 轨迹、误差、速度、偏航图表 |
| `analysis/flight_eval/run_spec.py` | run_spec.json 解析和路径解析 |
| `analysis/flight_eval/segmentation.py` | 四边巡飞段分割 |
| `analysis/flight_eval/time_alignment.py` | 时间对齐和采样 |
| `analysis/flight_eval/__init__.py` | 模块导出 |
| `analysis/flight_eval/assets/dashboard_app.js` | 交互式仪表板 JS |

新增：
- `analysis/flight_eval/fc_gps.py` — 飞控 GPS 数据专项处理
- `analysis/lk_flow_frontend.py` — LK 光流前端工具

**评价指标定义见：** `analysis/skills/flight-error-analysis/references/definitions.md`

---

## 常见问题

**VIO 立刻退出（exit≠0）**
- 检查 `data/vio.log` 末尾
- `--init-from-fc` 文件路径是否正确
- GPS CSV 时间戳是否与相机时间对齐（fly1 不加偏移，fly2/fly3 已预先对齐）

**LK 轨迹全为 0**
- 检查 GPS CSV 格式：是否有 `alt_m` 列（或 `altitude`）
- 检查 `--start-time` 是否在 GPS 数据范围内

**analysis 报 "t1 is null"**
- 在 `metadata/run_spec.json` 中手动填入 `"t1": <实际结束时间>`
- 通过 `diag.csv` 最后一行时间戳确定
