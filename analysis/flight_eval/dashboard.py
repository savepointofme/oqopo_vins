"""dashboard.py — 离线 interactive_dashboard.html 组装.

把后端产物（samples / segments / summary / sampling / meta）打包为一个
内联 JSON（window.RUN_DATA），注入自包含 HTML，由 assets/dashboard_app.js
渲染设计稿“布局A·分析优先”的完整交互页面：
  控制条（GPS/VIO/LK 显隐·分段叠加·里程↔时间·横垂↔沿横垂·速度分量↔模值向量·
          仅有效↔含缺口·全程↔选中段）、指标卡片、
  XY 轨迹（分段叠加+点击高亮+随时间动画+同步游标）、
  XY 误差、横/垂位置误差、速度对比/误差、航向误差、
  各段局部漂移条形（点击联动）、GPS 采样质量、
  航段浏览器 + 明细（段内起点对齐沿/横航向轨迹）、溯源面板。
纯 SVG 实现，无外部依赖，离线可用；前端只渲染、不重算核心指标。
"""
from __future__ import annotations

import json
import os

import numpy as np
import pandas as pd

DASHBOARD_FORMAT_ERROR = (
    "ERROR: generated dashboard is not SVG v2 / window.RUN_DATA format. "
    "Plotly dashboard is deprecated."
)
_FORBIDDEN_HTML_MARKERS = ("cdn.plot.ly", "Plotly.newPlot", "plotly.js")

# 前端需要的样本列（与设计文档“前端数据模型”一致）
SAMPLE_COLS = [
    "t", "cum_dist",
    "gps_E", "gps_N", "gps_U", "gps_vE", "gps_vN", "gps_vU",
    "gps_speed_xy", "gps_course_deg",
    "vio_E", "vio_N", "vio_U", "vio_vE", "vio_vN", "vio_vU",
    "vio_speed_xy", "vio_course_deg", "vio_delay_s",
    "err_E", "err_N", "err_U", "err_XY", "err_3D",
    "err_along", "err_cross", "err_vertical",
    "err_vE", "err_vN", "err_vU", "err_speed_xy", "err_vXY_vec",
    "err_v_along", "err_v_cross", "err_v_vertical",
    "course_err_deg", "segment_id", "segment_type", "segment_heading_deg",
    "heading_source", "heading_quality", "valid", "gap",
]
LK_COLS = ["lk_E", "lk_N", "lk_U", "err_lk_XY"]


def build_payload(df: pd.DataFrame, seg_index: pd.DataFrame, seg_err: pd.DataFrame,
                  summary: dict, quality: dict, meta: dict,
                  *, decimate_to: int = 4000) -> dict:
    """组装前端 JSON 数据契约。"""
    cols = [c for c in SAMPLE_COLS if c in df.columns]
    cols += [c for c in LK_COLS if c in df.columns]
    sub = df[cols]
    if len(sub) > decimate_to:
        # Keep payload bounded; use ceil so 4001 rows do not remain undecimated.
        step = (len(sub) + decimate_to - 1) // decimate_to
        keep = set(range(0, len(sub), step))
        keep.add(len(sub) - 1)
        # Segment detail metrics use exact segment endpoints.  Preserve both
        # ends of every segment so a decimated browser cannot draw a different
        # endpoint from the value printed beside it.
        if "segment_id" in sub.columns:
            segment_ids = sub["segment_id"].to_numpy()
            for segment_id in pd.unique(segment_ids):
                positions = np.flatnonzero(segment_ids == segment_id)
                if len(positions):
                    keep.add(int(positions[0]))
                    keep.add(int(positions[-1]))
        sub = sub.iloc[sorted(keep)]

    # segments: 合并 index + error
    seg = seg_index.merge(seg_err, on="segment_id", how="left", suffixes=("", "_e"))
    seg_records = json.loads(seg.to_json(orient="records"))

    return {
        "meta": meta,
        "summary": summary,
        "samples": json.loads(sub.to_json(orient="records")),
        "segments": seg_records,
        "sampling": {**quality, "gaps": meta.get("gaps", [])},
    }


_HEAD = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>飞行评价 · {experiment_id}</title>
<style>
:root{{--ink:#1d2733;--muted:#828c99;--line:#d6dae0;--line2:#eef0f3;--panel:#fff;--bg:#eceef1}}
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:"Microsoft YaHei","Noto Sans SC",system-ui,sans-serif;background:var(--bg);color:var(--ink)}}
.topbar{{position:sticky;top:0;z-index:30;background:#11171f;color:#fff;padding:12px 20px;display:flex;align-items:center;gap:14px}}
.topbar h1{{font-size:15px;font-weight:600}}
.topbar .sub{{font-size:12px;color:#9fb0c4;font-family:ui-monospace,monospace}}
.topbar .sp{{flex:1}}
.topbar .b{{font:10.5px ui-monospace,monospace;padding:3px 8px;border-radius:5px;background:#22303f;color:#7fb2ff}}
.dashbody{{padding-bottom:24px}}
.syn2{{display:flex;gap:8px;align-items:center;padding:7px 20px;background:#fdf6ee;color:#8a5512;font-size:11.5px;border-bottom:1px solid var(--line)}}
.syn2 b{{font:10px ui-monospace,monospace;letter-spacing:.06em}}
.ctrls{{display:flex;flex-wrap:wrap;gap:8px 14px;align-items:center;padding:11px 20px;background:#f6f7f9;border-bottom:1px solid var(--line);position:sticky;top:44px;z-index:20}}
.cg{{display:flex;align-items:center;gap:7px}}
.glbl{{font:11px ui-monospace,monospace;color:var(--muted)}}
.segctl{{display:flex;border:1px solid var(--line);border-radius:6px;overflow:hidden;background:#fff}}
.segctl button{{border:none;border-right:1px solid var(--line);padding:5px 9px;font-size:11.5px;cursor:pointer;background:transparent;color:var(--ink);font-family:inherit}}
.segctl button:last-child{{border-right:none}}
.segctl button.on{{background:var(--ink);color:#fff}}
.chip{{display:inline-flex;align-items:center;gap:6px;border:1px solid var(--line);background:#fff;border-radius:20px;padding:4px 11px;font-size:11.5px;cursor:pointer;color:var(--ink);user-select:none}}
.chip.off{{opacity:.45;text-decoration:line-through}}
.chip .sw{{width:9px;height:9px;border-radius:2px;background:var(--muted)}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px;padding:16px 20px}}
.card{{background:var(--panel);border:1px solid var(--line);border-radius:9px;padding:13px 14px}}
.card .k{{font-size:11px;color:var(--muted);margin-bottom:5px}}
.card .v{{font-family:ui-monospace,monospace;font-size:22px;letter-spacing:-.02em}}
.card .u{{font-size:12px;color:var(--muted)}}
.card.accent{{border-top:3px solid #2563c9}}
.card.warnc{{border-top:3px solid #d33a35}}
.card.okc{{border-top:3px solid #1f7a4d}}
.rowgrid{{display:grid;gap:12px;padding:0 20px 12px}}
.panel{{background:var(--panel);border:1px solid var(--line);border-radius:9px;display:flex;flex-direction:column;min-width:0}}
.panel.pad{{padding:16px}}
.ch{{display:flex;align-items:baseline;gap:9px;padding:11px 13px 6px}}
.ch .ct{{font-size:12.5px;font-weight:600}}
.ch .cu{{font-size:10.5px;color:var(--muted);font-family:ui-monospace,monospace}}
.ch .sp{{flex:1}}
.q{{font-size:11px;color:var(--muted);padding:0 13px 8px;line-height:1.45}}
.cv{{padding:0 8px 8px}}
.lgrow{{display:flex;flex-wrap:wrap;gap:10px;padding:0 13px 11px;align-items:center}}
.lgi{{display:inline-flex;align-items:center;gap:6px;font-size:11px;color:var(--ink);cursor:pointer}}
.lgi.muted{{color:var(--muted);cursor:default}}
.lgln{{width:14px;border-top:2.5px solid var(--muted)}}
.tlabel{{font:11px ui-monospace,monospace;color:#2563c9;margin-left:auto}}
.playbtn{{border:1px solid #2563c9;background:#fff;color:#2563c9;border-radius:6px;padding:4px 11px;font-size:11.5px;cursor:pointer;font-family:inherit;white-space:nowrap}}
.playbtn.on{{background:#2563c9;color:#fff}}
.badge{{font:10.5px ui-monospace,monospace;padding:3px 8px;border-radius:5px;display:inline-flex;gap:5px;align-items:center}}
.badge.ok{{background:#e6f3ec;color:#1f7a4d}} .badge.warn{{background:#fdf3f2;color:#d33a35}}
.badge .dotc{{width:7px;height:7px;border-radius:50%;background:currentColor}}
.seglist{{max-height:320px;overflow-y:auto}}
.segrow{{display:grid;grid-template-columns:auto 1fr auto;gap:10px;align-items:center;padding:7px 13px;border-bottom:1px solid var(--line2);cursor:pointer}}
.segrow:hover{{background:#f6f7f9}}
.segrow.sel{{background:#f3eefe;box-shadow:inset 3px 0 0 #8b5cf6}}
.stype{{font:10px ui-monospace,monospace;padding:2px 6px;border-radius:4px}}
.stype.straight{{background:#eaf0fc;color:#2563c9}} .stype.turn{{background:#fdf6ee;color:#b46410}}
.sname{{font-size:12px}} .sname .sub{{font:10px ui-monospace,monospace;color:var(--muted)}}
.sval{{text-align:right;font:11.5px ui-monospace,monospace}} .sval .sub{{font-size:10px;color:var(--muted)}}
.sdh{{display:flex;gap:8px;align-items:center;margin-bottom:10px}} .sdh b{{font-size:13px}}
.gh{{font:10.5px ui-monospace,monospace;margin:12px 0 6px;letter-spacing:.04em}}
.g3{{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}}
.gk{{font-size:10.5px;color:var(--muted)}} .gv{{font:15px ui-monospace,monospace}}
.kv{{display:grid;grid-template-columns:170px 1fr;gap:2px 14px;font:11px ui-monospace,monospace;padding:14px}}
.kv dt{{color:var(--muted);padding:5px 0;border-bottom:1px solid var(--line2)}}
.kv dd{{padding:5px 0;border-bottom:1px solid var(--line2);word-break:break-all}}
.muted{{color:var(--muted)}}
@media(max-width:900px){{.rowgrid{{grid-template-columns:1fr!important}}}}
</style>
</head>
<body>
<div class="topbar">
  <h1>飞行误差分析仪表板</h1><span class="sub">{out_folder}</span>
  <span class="sp"></span><span class="b">{status}</span><span class="b">起点对齐 start-heading</span>
</div>
<div id="app"></div>
<script id="run-data" type="application/json">{payload}</script>
<script>window.RUN_DATA = JSON.parse(document.getElementById('run-data').textContent);</script>
<script>{app_js}</script>
</body>
</html>"""


def _read_app_js() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "assets", "dashboard_app.js"), "r", encoding="utf-8") as f:
        return f.read()


def validate_dashboard_html(html: str) -> None:
    """Fail fast if an interactive dashboard is not the SVG v2 format."""
    problems = []
    if "window.RUN_DATA" not in html:
        problems.append("missing window.RUN_DATA")
    if '"segments"' not in html:
        problems.append("missing segments data")
    for marker in _FORBIDDEN_HTML_MARKERS:
        if marker in html:
            problems.append(f"contains {marker}")
    if problems:
        raise RuntimeError(f"{DASHBOARD_FORMAT_ERROR} ({'; '.join(problems)})")


def validate_dashboard_file(path: str) -> None:
    with open(path, "r", encoding="utf-8") as f:
        validate_dashboard_html(f.read())


def write_dashboard(payload: dict, reports_dir: str) -> str:
    os.makedirs(reports_dir, exist_ok=True)
    meta = payload["meta"]
    html = _HEAD.format(
        experiment_id=meta.get("experiment_id", ""),
        out_folder=meta.get("out_folder", ""),
        status=meta.get("status", ""),
        # Escape closing script tags defensively before embedding JSON in HTML.
        payload=json.dumps(payload, ensure_ascii=False).replace("</", "<\\/"),
        app_js=_read_app_js(),
    )
    validate_dashboard_html(html)
    p = os.path.join(reports_dir, "interactive_dashboard.html")
    with open(p, "w", encoding="utf-8") as f:
        f.write(html)
    validate_dashboard_file(p)
    return p
