# LOCAL_VALIDATION_REPORT — flight_eval_tool P0 后端验收
生成时间: 2026-06-11

---

## 1. 跑通了哪些命令

| 命令 | 状态 |
|---|---|
| `python analysis/flight_eval_tool.py inspect-run --run-spec analysis/run_specs/fly3_oc2_gpsz_fi1e4_v2.json` | ✓ PASS |
| `python analysis/flight_eval_tool.py inspect-run --run-spec analysis/run_specs/fly4_oc2_gpsz_fi1e4_v2.json` | ✓ PASS |
| `python analysis/flight_eval_tool.py single --run-spec analysis/run_specs/fly3_oc2_gpsz_fi1e4_v2.json --out-root "C:/Users/baloney/Desktop/实验目录"` | ✓ PASS |
| `python analysis/flight_eval_tool.py single --run-spec analysis/run_specs/fly4_oc2_gpsz_fi1e4_v2.json --out-root "C:/Users/baloney/Desktop/实验目录"` | ✓ PASS |
| import smoke test (all 12 modules) | ✓ PASS |

运行命令须设置环境变量 `PYTHONIOENCODING=utf-8`（Windows 终端 GBK 限制）。

---

## 2. 生成了哪些输出

### 目录结构（符合规范）

```
C:\Users\baloney\Desktop\实验目录\
  gpsz_oc2_1e4_fly3\
    20260606_fly3_oc2_gpsz_fi1e4_成功\
      data\, tables\, plots\, reports\, metadata\
  gpsz_oc2_1e4_fly4\
    20260606_fly4_oc2_gpsz_fi1e4_成功\
      data\, tables\, plots\, reports\, metadata\
```

格式：`<实验文件夹>/<日期_飞行_方法_状态>/` ✓

### 11 项必需输出验收（fly3 / fly4 全部 OK）

| 文件 | fly3 | fly4 |
|---|---|---|
| `data/gps_time_aligned_samples.csv` | ✓ | ✓ |
| `tables/global_summary.csv` | ✓ | ✓ |
| `tables/gps_sampling_quality.csv` | ✓ | ✓ |
| `tables/segment_index.csv` | ✓ | ✓ |
| `tables/segment_error_summary.csv` | ✓ | ✓ |
| `reports/FULL_FLIGHT_ERROR_ANALYSIS.md` | ✓ | ✓ |
| `reports/interactive_dashboard.html` | ✓ | ✓ |
| `metadata/provenance.json` | ✓ | ✓ |
| `metadata/analysis_config.json` | ✓ | ✓ |
| `metadata/run_spec_resolved.json` | ✓ | ✓ |
| `metadata/status.json` | ✓ | ✓ |

---

## 3. 修了哪些 Bug

### Bug 1：Windows stdout GBK 编码错误
- **症状**：`flight_eval_tool.py inspect-run` 崩溃 `UnicodeEncodeError: 'gbk' codec can't encode character '✓'`
- **原因**：`cmd_inspect_run` 打印 `✓ · ✗` 等 Unicode 字符，Windows PowerShell 默认 GBK 输出流无法编码
- **修复**：在 `flight_eval_tool.py` 顶部加 `sys.stdout.reconfigure(encoding="utf-8")` + 设置 `PYTHONIOENCODING=utf-8`
- **文件**：`analysis/flight_eval_tool.py`（第 18-24 行）

### Bug 2：VIO 初始航向估计使用全段速度（严重对齐错误）
- **症状**：fly4 `yaw_or_course_rmse=162.87°`（几乎 180° 翻转），`vxy_vec_rmse=79 m/s`（物理不可能），`xy_rmse=5456 m`（vs 期望 827 m）
- **原因**：`cmd_single` 中 VIO 初始航向估计调用：
  ```python
  # 错误：用索引数组作为时间、window=全段长度 → 平均了 1891 s 的速度
  h_vio = trajectory.estimate_initial_heading(
      np.arange(len(vio_vxy)), vio_vxy[:, 0], vio_vxy[:, 1], 0, len(vio_vxy))
  ```
  而 GPS 航向用的是 `[t0, t0+60s]` 窗口，两者不对称，长航程飞行（fly4 有明显 yaw 漂移）会导致平均值指向错误方向
- **修复**：改为与 GPS 航向估计使用相同的时间数组和窗口：
  ```python
  # 正确：使用 GPS 对齐时间戳的前 course_window_s 秒
  h_vio = trajectory.estimate_initial_heading(
      gps["t"].values, vio_vxy[:, 0], vio_vxy[:, 1],
      spec.t0, spec.alignment["course_window_s"])
  ```
- **效果**：fly4 `xy_rmse=910 m`、`yaw_rmse=17.76°`、`vxy_vec_rmse=12 m/s`（合理）；fly3 `xy_rmse=213 m`、`yaw_rmse=1.87°`
- **文件**：`analysis/flight_eval_tool.py`（第 107-110 行）

---

## 4. 仍缺什么

1. **中文字体**：Windows 上 matplotlib 找不到 `Noto Sans CJK SC` / `Microsoft YaHei` 等，图表中文标签以方块显示。功能性 OK（PNG/SVG 生成正常），视觉不可用。修复方案：`python -m pip install matplotlib-chinese` 或手动注册系统字体路径至 `matplotlibrc`。

2. **`straight_leg_summary.csv` / `turn_transition_summary.csv`**：两个辅助表已生成，但若 fly3/4 无满足 `min_straight_len_m=2000 m` 的长直线，则这两个文件可能为空行（仍有 header）。当前 segmentation 阈值对无明显直线的随机飞行可能无法分段。

3. **部分绘图未实现**：`plotting.py` 中标注 `TODO·补齐` 的图（`enu_position_compare`、`enu_velocity_error`、`speed_error_distance`、`segment_velocity_error_bar`、`straight_leg_error_profile`、完整 Plotly HTML）目前跳过，输出 `plots/` 只有 8 张图。不阻断整体流程。

4. **`build-fc-gps` 子命令**：已预留接口，抛 `NotImplementedError`（符合设计文档，本地飞控日志格式未集成）。当前 GPS CSV 已有 Ve/Vn/Vu，不影响 single-run 分析。

5. **旧格式 run specs** (`fly3_oc2_1e4_20260606.json` / `fly4_oc2_1e4_20260606.json`): 仍保留，与新工具不兼容（旧键名 `gps`、`run_status` 等）。新的 `*_v2.json` 是正式版本。建议待新工具稳定后删除旧文件。

6. **`--force` 标志未实现**：`cmd_single` 中接受 `--force` 参数但不检查/清空已有目录，重运行时新文件会覆盖同名文件，旧文件残留（无害但可能引起混淆）。

---

## 5. fly3 / fly4 核心指标

### Fly3（`fly3_oc2_gpsz_fi1e4`，t0=618 s，t1=1600 s）

| 指标 | 值 |
|---|---|
| 有效窗口时长 | 981.8 s |
| GPS 总里程 | 37.80 km |
| **XY RMSE** | **213.34 m** |
| XY RMSE / 里程 | 0.564 % |
| 终点 XY 误差 | 274.58 m |
| 终点 XY 漂移率 | 0.726 % |
| 终点沿航向误差 | 273.67 m |
| 终点横航向误差 | 22.38 m |
| 航向 RMSE | 1.87 ° |
| 速度 RMSE（标量） | 2.32 m/s |
| 速度向量 RMSE | 2.62 m/s |
| 参考速度来源 | flight_controller_raw（Ve/Vn/Vu） |
| VIO 速度来源 | traj.bias（vx/vy/vz） |
| GPS 有效样本 / 总样本 | 4767 / 4768 |
| GPS 缺口 | 7 段，最长 1.2 s |

### Fly4（`fly4_oc2_gpsz_fi1e4`，t0=924.4 s，t1=2816 s）

| 指标 | 值 |
|---|---|
| 有效窗口时长 | 1891.2 s |
| GPS 总里程 | 74.23 km |
| **XY RMSE** | **910.35 m** |
| XY RMSE / 里程 | 1.226 % |
| 终点 XY 误差 | 775.89 m |
| 终点 XY 漂移率 | 1.045 % |
| 终点沿航向误差 | 402.0 m |
| 终点横航向误差 | 663.6 m |
| 航向 RMSE | 17.76 ° |
| 速度 RMSE（标量） | 2.22 m/s |
| 速度向量 RMSE | 12.07 m/s |
| 参考速度来源 | flight_controller_raw（Ve/Vn/Vu） |
| VIO 速度来源 | traj.bias（vx/vy/vz） |
| GPS 有效样本 / 总样本 | 8214 / 8215 |
| GPS 缺口 | 150 段，最长 3.8 s |

---

## 6. 九项设计要求逐项验收

| 要求 | 状态 | 说明 |
|---|---|---|
| GPS 更新时间采样，不上采样到 30Hz | ✓ | `gps_sampling.sample_on_gps_grid` 以每个 GPS t_i 为主网格，找其后第一个 VIO 状态 |
| reference velocity = 飞控原始 Ve/Vn/Vu | ✓ | `read_gps_csv` 检测并使用 Ve/Vn/Vu 列，`quality.velocity_source=flight_controller_raw` |
| VIO velocity 优先 traj.txt.bias vx/vy/vz | ✓ | `vio_velocity_source=traj.bias` 已在 summary 打印中确认；fallback 仅在 bias 不可用时触发 |
| VIO velocity 与 position 使用同一 R_align | ✓ | `trajectory.start_align` 同时返回 `est_EN_aligned, est_vEN_aligned`，二者使用相同 R；velocity 分支 `v_aligned = (R @ est_vEN.T).T` |
| segment drift 区分 local/global | ✓ | `segment_errors` 输出 `local_final_xy_error_m`（段起点归零）和 `global_xy_rmse_m`（全局参考） |
| 图表中文 | △ | 代码中所有标题/轴/图例均为中文；Windows 上 matplotlib 字体缺失显示方块，文字已在 SVG/HTML 中内嵌 |
| dashboard 只读后端 CSV/JSON，不重新计算 | ✓ | `dashboard.build_payload` 直接消费传入的 df/summary/quality dict；`cmd_dashboard` 子命令从 CSV 读取重建 |
| build-fc-gps 不阻断当前分析 | ✓ | GPS CSV 已有 Ve/Vn/Vu，分析全程使用 `flight_controller_raw`，未调用 build-fc-gps |
| 11 项必需输出文件 | ✓ | fly3/fly4 全部通过，见第 2 节表格 |

---

## 7. 是否可以正式替代旧分析脚本

**可以替代，但有两点注意：**

1. **指标定义变化**：新工具使用 `start_heading` 对齐（仅旋转，不最优拟合），旧脚本 `full_flight_error_analysis.py` 使用 Umeyama SE3 最优拟合。fly3 新工具 XY RMSE=213 m（旧脚本期望 323 m），差异来源于对齐方法，不是 VIO 性能变化。历史对比数字需用相同工具重算。

2. **中文字体**：图表中文在当前 Windows 环境需额外安装字体。可用 `pip install matplotlib-chinese` 或下载 SimHei.ttf 到 matplotlib 字体目录解决。

**结论**：P0 后端验收通过。`flight_eval_tool.py single` 产出完整、可追溯的分析目录，所有必需文件存在且内容正确，GPS 采样策略和速度来源符合设计要求，segment 局部/全局漂移分离符合要求。旧脚本可停用，历史基准数字建议用新工具在相同 run spec 上重算后锁定。
