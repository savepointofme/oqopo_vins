"""flight_eval — OpenVINS 飞行试验评价工具核心包.

主线：single-run 完整分析。一条 run spec → 一个标准分析目录
（data/ tables/ plots/ reports/ metadata/）。

模块职责（每个指标只在一处定义，避免漂移）:
  io            路径解析（含 zip://archive::member）、TUM / .bias / CSV 读取
  run_spec      schema 校验 + resolve + 落盘
  trajectory    ENU 转换、start-heading 对齐（位置+速度同旋转）、里程
  gps_sampling  以 GPS 更新时间为采样网格（不上采样）
  metrics       误差定义唯一真源（ENU / 速度 / 沿-横-垂）+ 统计
  segmentation  直线检测 + 长直线精细切分 + 转弯归类 + 局部/全局段漂移
  time_alignment FC/IMU/camera 角运动相似性时间偏移搜索
  lk_only       纯视觉诊断轨迹构建与评估
  plotting      PNG + SVG + Plotly HTML（中文）
  dashboard     离线 interactive_dashboard.html 组装
  reports       中文 markdown 报告
  provenance    run_spec_resolved / analysis_config / status / command
"""

__version__ = "0.1.0"
METRIC_DEFINITIONS_VERSION = "1.0.0"

__all__ = [
    "io",
    "run_spec",
    "trajectory",
    "gps_sampling",
    "metrics",
    "segmentation",
    "time_alignment",
    "lk_only",
    "plotting",
    "dashboard",
    "reports",
    "provenance",
]
