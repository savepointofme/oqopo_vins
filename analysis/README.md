# analysis/ — OpenVINS 飞行试验评价工具

统一、可复用的飞行试验评价工具链。主线：**single-run 完整分析**。

> 配套设计文档（线框图 + 数据契约 + schema）：仓库根目录
> `OpenVINS 飞行试验评价工具 设计方案.html`。

## 安装

```bash
pip install numpy pandas matplotlib
# .bias / TUM 用 numpy.loadtxt；中文图字体见 plotting.set_chinese_font
```

## 快速开始

```bash
# 1. 体检：实验是否具备分析条件
python3 analysis/flight_eval_tool.py inspect-run \
    --run-spec analysis/run_specs/fly4_oc2_gpsz_fi1e4.json

# 2. 主功能：单条实验完整分析
python3 analysis/flight_eval_tool.py single \
    --run-spec analysis/run_specs/fly4_oc2_gpsz_fi1e4.json \
    --out-root "C:/Users/baloney/Desktop/实验目录"

# 3. （可选）多条件对比，只读已完成目录
python3 analysis/flight_eval_tool.py compare \
    --runs <dir1> <dir2> <dir3> --out-dir <cmp_dir>

# 4. FC/IMU/camera 时间戳对齐
python3 analysis/flight_eval_tool.py align-time \
    --fc-log <raw> --imu-csv <imu> --camera-csv <cam> --out <align_dir>

# 5. 仅重建离线交互页
python3 analysis/flight_eval_tool.py dashboard --analysis-dir <dir>
```

## 模块

```
flight_eval/
  io.py            路径解析（含 zip://archive::member）、TUM/.bias/CSV 读取
  run_spec.py      schema 校验 + resolve + 落盘
  trajectory.py    ENU 转换、start-heading 对齐（位置+速度同旋转）、里程
  gps_sampling.py  GPS 更新时间采样网格 + 延迟/缺口质量
  metrics.py       误差定义唯一真源 + 统计
  segmentation.py  直线检测 + 长直线精细切分 + 局部/全局段漂移
  time_alignment.py FC/IMU/camera 角运动相似性 offset 搜索
  lk_only.py       纯视觉诊断（优先级 1/2/3）
  plotting.py      中文 SVG 静态图
  dashboard.py     离线交互 dashboard（布局A：控制条+轨迹动画+段内起点对齐+联动，纯SVG，前端不重算）
                   + assets/dashboard_app.js（自包含前端）
  reports.py       中文 markdown 报告
  provenance.py    metadata 四件套
```

## 关键约定（详见 FLIGHT_ERROR_ANALYSIS_SKILL.md）

- 以 **GPS 更新时间**采样，不上采样到 30Hz。
- 参考速度优先**飞控原始 Ve/Vn/Vu**；VIO 速度优先 **`.bias` 的 vx,vy,vz**。
- 主漂移指标用**起点对齐**，best-fit 仅作 diagnostic。
- 误差主分解用**沿/横/垂**，以横航向 + 垂直为主。
- 段漂移区分**局部（主）/ 全局（参考）**。
- 每份结果带完整 **provenance**。

## 已实现 / 待本地核验

**已实现**（基于 OpenVINS 官方 ov_eval 输出格式与常见自驾仪导出）:
- `build-fc-gps`: 解析 `.csv/.tsv`（列名归一化）、`.ulg`（PX4，需 pyulog）、
  `.bin`（ArduPilot，需 pymavlink），输出标准 `ts_ns,lat,lon,alt,Ve,Vn,Vu,satellites`；
  NED 的 Vd 自动转 Vu=-Vd；缺速度列报错而非伪造。
- `time-align`: 角速度互相关 offset 搜索 + 多候选 + 诊断图（相关曲线 / gyro 叠加）。
- `lk-only`: 优先现成轨迹；其次 yaw 诊断；构建诊断轨迹时尺度按真实小孔模型
  （altitude/focal_px，来自 flow 列或 run_spec.camera），缺尺度则标 unavailable，不伪造。
- 中文 SVG 静态图 + SVG v2 / window.RUN_DATA 离线交互 dashboard（前端不重算）。

**待本地核验**: 特定飞控 CSV 的精确表头、zip 内部目录层级、分段阈值取值、
相机内参 focal_px、matplotlib 中文字体名、git commit 注入。可选依赖：
PX4 日志装 `pyulog`，ArduPilot 日志装 `pymavlink`。
