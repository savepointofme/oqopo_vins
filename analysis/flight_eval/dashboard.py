"""dashboard.py — 离线 interactive_dashboard.html 组装.

把后端产物（samples / segments / summary / sampling / meta）打包为一个
内联 JSON，注入到一个自包含 HTML 模板。前端只渲染、不重算核心指标。

设计参考: 见仓库内 “OpenVINS 飞行试验评价工具 设计方案.html” 的
布局 A（分析优先，推荐默认）。本函数生成数据契约 + 最小可用页面骨架；
完整图表交互（同步游标、段内起点对齐分析、轨迹动画）由前端脚本实现。
"""
from __future__ import annotations

import json
import os

import numpy as np
import pandas as pd

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
    "course_err_deg", "segment_id", "segment_type", "valid", "gap",
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
        step = len(sub) // decimate_to
        sub = sub.iloc[::step]

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


_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>飞行评价 · {experiment_id}</title>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js"></script>
<style>
*{{box-sizing:border-box;margin:0;padding:0}}
body{{font-family:"Microsoft YaHei","Noto Sans SC",system-ui,sans-serif;background:#eceef1;color:#1d2733}}
.topbar{{background:#11171f;color:#fff;padding:12px 20px;display:flex;align-items:center;gap:16px}}
.topbar h1{{font-size:15px;font-weight:600}}
.topbar .sub{{font-size:12px;color:#9fb0c4;font-family:monospace}}
.offline-badge{{background:#fdf6ee;color:#8a5512;border:1px solid #e8c98a;border-radius:4px;padding:2px 8px;font-size:11px}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;padding:14px 20px}}
.card{{background:#fff;border:1px solid #d6dae0;border-radius:8px;padding:12px 14px}}
.card .k{{font-size:11px;color:#828c99;margin-bottom:4px}}
.card .v{{font-size:20px;font-family:monospace;font-weight:600}}
.card .u{{font-size:11px;color:#828c99}}
.card.warn .v{{color:#c0392b}}
.section{{margin:0 20px 20px;background:#fff;border:1px solid #d6dae0;border-radius:8px;overflow:hidden}}
.section-title{{padding:10px 16px;font-size:13px;font-weight:600;border-bottom:1px solid #f0f2f5;background:#f8f9fb;display:flex;justify-content:space-between;align-items:center}}
.section-title .hint{{font-size:11px;color:#9aa3ad;font-weight:400}}
.plotbox{{padding:4px}}
.grid2{{display:grid;grid-template-columns:1fr 1fr;gap:0}}
.grid2 .section{{margin:0}}
.grid-wrap{{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin:0 20px 20px}}
.seg-table{{width:100%;border-collapse:collapse;font-size:12px}}
.seg-table th{{background:#f8f9fb;padding:6px 10px;text-align:left;border-bottom:1px solid #e4e8ed;color:#555}}
.seg-table td{{padding:5px 10px;border-bottom:1px solid #f0f2f5}}
.seg-table tr:hover td{{background:#f8f9fb}}
.badge{{display:inline-block;padding:1px 7px;border-radius:10px;font-size:11px}}
.badge.straight{{background:#dbeafe;color:#1e40af}}
.badge.turn{{background:#fef3c7;color:#92400e}}
.badge.transition{{background:#e0e7ff;color:#3730a3}}
.badge.short{{background:#f3f4f6;color:#6b7280}}
footer{{padding:12px 20px;font-size:11px;color:#9aa3ad;border-top:1px solid #e4e8ed;margin-top:8px}}
</style>
</head>
<body>
<div class="topbar">
  <h1>飞行误差分析仪表板</h1>
  <span class="sub">{out_folder}</span>
  <span class="offline-badge">OFFLINE · 前端不重算</span>
</div>

<div id="cards" class="cards"></div>

<div class="section">
  <div class="section-title">XY 轨迹对比（ENU · 起点对齐）<span class="hint">绿=GPS飞控参考，蓝=VIO估计</span></div>
  <div class="plotbox"><div id="plt-traj" style="height:420px"></div></div>
</div>

<div class="grid-wrap">
  <div class="section">
    <div class="section-title">XY 误差 vs 里程<span class="hint">GPS 更新时间采样，非插值</span></div>
    <div class="plotbox"><div id="plt-xy-err" style="height:280px"></div></div>
  </div>
  <div class="section">
    <div class="section-title">横航向 / 垂直 位置误差<span class="hint">横航向=cross-track，垂直=altitude</span></div>
    <div class="plotbox"><div id="plt-cross-vert" style="height:280px"></div></div>
  </div>
</div>

<div class="section">
  <div class="section-title">ENU 速度对比（vE 东向）<span class="hint">绿=飞控原始速度，蓝=VIO traj.bias</span></div>
  <div class="plotbox"><div id="plt-vel-e" style="height:240px"></div></div>
</div>

<div class="section">
  <div class="section-title">ENU 速度对比（vN 北向）</div>
  <div class="plotbox"><div id="plt-vel-n" style="height:240px"></div></div>
</div>

<div class="grid-wrap">
  <div class="section">
    <div class="section-title">航向 / Course 误差 vs 里程<span class="hint">VIO course − GPS course，±180° wrap</span></div>
    <div class="plotbox"><div id="plt-yaw" style="height:260px"></div></div>
  </div>
  <div class="section">
    <div class="section-title">速度标量误差<span class="hint">|VIO speed| − |GPS speed|</span></div>
    <div class="plotbox"><div id="plt-spd-err" style="height:260px"></div></div>
  </div>
</div>

<div class="section">
  <div class="section-title">各段局部漂移率（段起点归零后）<span class="hint">蓝=直线段，橙=转弯/过渡</span></div>
  <div class="plotbox"><div id="plt-seg-drift" style="height:320px"></div></div>
</div>

<div class="section">
  <div class="section-title">GPS 采样质量 — VIO 对齐延迟<span class="hint">每个 GPS 更新时刻对应的下一帧 VIO 延迟</span></div>
  <div class="plotbox"><div id="plt-delay" style="height:220px"></div></div>
</div>

<div class="section">
  <div class="section-title">分段误差明细</div>
  <div style="overflow-x:auto;padding:8px 4px">
  <table class="seg-table" id="seg-table">
    <thead><tr>
      <th>#</th><th>类型</th><th>段长 m</th>
      <th>局部漂移 %</th><th>全局XY RMSE m</th>
      <th>沿航向 RMSE m</th><th>横航向 RMSE m</th><th>速度 RMSE m/s</th>
    </tr></thead>
    <tbody></tbody>
  </table>
  </div>
</div>

<footer>
  数据源: window.RUN_DATA（内联 JSON） · 前端仅渲染，不重算误差 ·
  实验ID: {experiment_id}
</footer>

<script id="run-data" type="application/json">{payload}</script>
<script>
const D = JSON.parse(document.getElementById('run-data').textContent);
window.RUN_DATA = D;
const S = D.summary, Q = D.sampling, META = D.meta;
const samps = D.samples;
const segs  = D.segments;

// ── helper: filter valid samples ──
const valid = samps.filter(r => r.valid && !r.gap);

// ── metric cards ──
const cardDefs = [
  ['总里程','gps_distance_km','km'],
  ['时长','duration_s','s'],
  ['终点XY误差','final_xy_error_m','m', v=>v>500],
  ['终点漂移','final_xy_drift_percent','%'],
  ['XY RMSE','xy_rmse_m','m'],
  ['速度RMSE','speed_rmse_mps','m/s'],
  ['航向RMSE','yaw_or_course_rmse_deg','°'],
  ['GPS有效样本','valid_aligned_count','',null,Q],
  ['VIO延迟p95','p95_delay_s','s',null,Q],
  ['GPS缺口数','gps_gap_count','段',null,Q],
];
const cc = document.getElementById('cards');
cardDefs.forEach(([label, key, unit, warnFn, src]) => {{
  const obj = src || S;
  const val = obj[key];
  const warn = warnFn && val != null && warnFn(val);
  const d = document.createElement('div');
  d.className = 'card' + (warn ? ' warn' : '');
  d.innerHTML = `<div class="k">${{label}}</div><div class="v">${{val ?? '—'}} <span class="u">${{unit}}</span></div>`;
  cc.appendChild(d);
}});

// ── colour palette ──
const C = {{ gps:'#1f7a4d', vio:'#2563c9', err:'#d33a35', cross:'#e07b1a', vert:'#8b5cf6', invalid:'#9aa3ad' }};

const cfg = {{ responsive:true, displayModeBar:true, modeBarButtonsToRemove:['lasso2d','select2d'] }};
const font = {{ family:'"Microsoft YaHei","Noto Sans SC",sans-serif', size:12 }};
const marg = {{ l:55, r:20, t:30, b:45 }};

// ── 1. XY 轨迹 ──
{{
  const gE=samps.map(r=>r.gps_E), gN=samps.map(r=>r.gps_N);
  const vE=samps.map(r=>r.vio_E), vN=samps.map(r=>r.vio_N);
  const txt=samps.map(r=>`t=${{r.t?.toFixed(1)}}s  里程=${{(r.cum_dist/1000)?.toFixed(2)}}km`);
  const traces=[
    {{x:gE,y:gN,mode:'lines',name:'GPS（飞控参考）',line:{{color:C.gps,width:2}},hovertemplate:'%{{text}}<extra>GPS</extra>',text:txt}},
    {{x:vE,y:vN,mode:'lines',name:'VIO估计',line:{{color:C.vio,width:1.5}},hovertemplate:'%{{text}}<extra>VIO</extra>',text:txt}},
    {{x:[gE[0]],y:[gN[0]],mode:'markers',name:'起点',marker:{{color:C.gps,size:10,symbol:'circle-open',line:{{width:2}}}},showlegend:true}},
  ];
  const layout={{xaxis:{{title:'东向 E / m',scaleanchor:'y',scaleratio:1}},yaxis:{{title:'北向 N / m'}},
    legend:{{orientation:'h',y:1.05}},margin:marg,font,hovermode:'closest'}};
  Plotly.newPlot('plt-traj',traces,layout,cfg);
}}

// ── 2. XY 误差 vs 里程 ──
{{
  const x=valid.map(r=>r.cum_dist/1000), y=valid.map(r=>r.err_XY);
  const txt=valid.map(r=>`t=${{r.t?.toFixed(1)}}s`);
  Plotly.newPlot('plt-xy-err',[
    {{x,y,mode:'lines',name:'VIO XY误差',line:{{color:C.vio,width:1.8}},text:txt,hovertemplate:'里程%{{x:.2f}}km  误差%{{y:.1f}}m<br>%{{text}}<extra></extra>'}}
  ],{{xaxis:{{title:'GPS累计里程 / km'}},yaxis:{{title:'XY误差 / m'}},margin:marg,font}},cfg);
}}

// ── 3. 横/垂误差 ──
{{
  const x=valid.map(r=>r.cum_dist/1000);
  const cr=valid.map(r=>r.err_cross), vt=valid.map(r=>r.err_vertical);
  Plotly.newPlot('plt-cross-vert',[
    {{x,y:cr,mode:'lines',name:'横航向 cross-track',line:{{color:C.err,width:2}}}},
    {{x,y:vt,mode:'lines',name:'垂直 vertical',line:{{color:C.vert,width:2}}}},
    {{x:[x[0],x[x.length-1]],y:[0,0],mode:'lines',line:{{color:'#ccc',width:1,dash:'dash'}},showlegend:false}},
  ],{{xaxis:{{title:'GPS累计里程 / km'}},yaxis:{{title:'误差 / m'}},
     legend:{{orientation:'h',y:1.08}},margin:marg,font}},cfg);
}}

// ── 4/5. ENU 速度对比 ──
const vt_t = valid.map(r=>r.t);
['E','N'].forEach((ax,i) => {{
  const gv=valid.map(r=>r[`gps_v${{ax}}`]), vv=valid.map(r=>r[`vio_v${{ax}}`]);
  Plotly.newPlot(`plt-vel-${{ax.toLowerCase()}}`,[
    {{x:vt_t,y:gv,mode:'lines',name:`飞控 v${{ax}}（参考）`,line:{{color:C.gps,width:1.8}}}},
    {{x:vt_t,y:vv,mode:'lines',name:`VIO v${{ax}}`,line:{{color:C.vio,width:1.2}}}},
  ],{{xaxis:{{title:'时间 t / s'}},yaxis:{{title:`v${{ax}} / m·s⁻¹`}},
     legend:{{orientation:'h',y:1.1}},margin:marg,font}},cfg);
}});

// ── 6. 航向误差 ──
{{
  const x=valid.map(r=>r.cum_dist/1000), y=valid.map(r=>r.course_err_deg);
  Plotly.newPlot('plt-yaw',[
    {{x,y,mode:'lines',name:'course误差',line:{{color:C.err,width:1.8}}}},
    {{x:[x[0],x[x.length-1]],y:[0,0],mode:'lines',line:{{color:'#ccc',width:1,dash:'dash'}},showlegend:false}},
  ],{{xaxis:{{title:'GPS累计里程 / km'}},yaxis:{{title:'航向误差 / °'}},margin:marg,font}},cfg);
}}

// ── 7. 速度标量误差 ──
{{
  const x=valid.map(r=>r.cum_dist/1000), y=valid.map(r=>r.err_speed_xy);
  Plotly.newPlot('plt-spd-err',[
    {{x,y,mode:'lines',name:'速度误差 |VIO|−|GPS|',line:{{color:C.cross,width:1.8}}}},
    {{x:[x[0],x[x.length-1]],y:[0,0],mode:'lines',line:{{color:'#ccc',width:1,dash:'dash'}},showlegend:false}},
  ],{{xaxis:{{title:'GPS累计里程 / km'}},yaxis:{{title:'速度误差 / m·s⁻¹'}},margin:marg,font}},cfg);
}}

// ── 8. 分段漂移条图 ──
{{
  const ss = segs.filter(r => r.local_drift_percent != null);
  const labels = ss.map(r => `#${{r.segment_id}} ${{r.segment_type?.slice(0,4)}}`);
  const vals   = ss.map(r => r.local_drift_percent ?? 0);
  const colors = ss.map(r => r.segment_type==='straight' ? C.vio : C.cross);
  Plotly.newPlot('plt-seg-drift',[
    {{x:vals,y:labels,type:'bar',orientation:'h',
      marker:{{color:colors}},
      text:vals.map(v=>v?.toFixed(1)+'%'),textposition:'outside',
      hovertemplate:'%{{y}}<br>局部漂移: %{{x:.2f}}%<extra></extra>'}},
  ],{{xaxis:{{title:'段内局部漂移率 / %',autorange:true}},
     yaxis:{{autorange:'reversed',tickfont:{{size:10}}}},
     margin:{{l:90,r:50,t:20,b:45}},font,height:Math.max(280,ss.length*18)}},cfg);
}}

// ── 9. GPS 延迟散点 ──
{{
  const allT = samps.map(r=>r.t), allD = samps.map(r=>r.vio_delay_s);
  const badMask = samps.map(r=>!r.valid && !r.gap);
  Plotly.newPlot('plt-delay',[
    {{x:allT.filter((_,i)=>!badMask[i]), y:allD.filter((_,i)=>!badMask[i]),
      mode:'markers',name:'有效对齐',marker:{{color:C.vio,size:3,opacity:0.5}}}},
    {{x:allT.filter((_,i)=>badMask[i]), y:allD.filter((_,i)=>badMask[i]),
      mode:'markers',name:'延迟超阈值',marker:{{color:C.err,size:6}}}},
    {{x:[allT[0],allT[allT.length-1]], y:[Q.p95_delay_s,Q.p95_delay_s],
      mode:'lines',name:`p95=${{Q.p95_delay_s?.toFixed(3)}}s`,
      line:{{color:C.err,dash:'dash',width:1.5}}}},
  ],{{xaxis:{{title:'时间 t / s'}},yaxis:{{title:'VIO对齐延迟 / s'}},
     legend:{{orientation:'h',y:1.12}},margin:marg,font}},cfg);
}}

// ── 10. 分段表格 ──
{{
  const tbody = document.querySelector('#seg-table tbody');
  segs.forEach(r => {{
    if(r.dist_m < 50) return;
    const tr = document.createElement('tr');
    const lp = r.local_drift_percent != null ? r.local_drift_percent.toFixed(2) : '—';
    const badgeType = r.segment_type || 'unknown';
    tr.innerHTML = `
      <td>${{r.segment_id}}</td>
      <td><span class="badge ${{badgeType}}">${{badgeType}}</span></td>
      <td>${{r.dist_m?.toFixed(0) ?? '—'}}</td>
      <td style="color:${{parseFloat(lp)>10?'#c0392b':'inherit'}}">${{lp}}</td>
      <td>${{r.global_xy_rmse_m?.toFixed(1) ?? '—'}}</td>
      <td>${{r.along_rmse_m?.toFixed(1) ?? '—'}}</td>
      <td>${{r.cross_rmse_m?.toFixed(1) ?? '—'}}</td>
      <td>${{r.speed_rmse_mps?.toFixed(3) ?? '—'}}</td>
    `;
    tbody.appendChild(tr);
  }});
}}
</script>
</body>
</html>"""


def write_dashboard(payload: dict, reports_dir: str) -> str:
    os.makedirs(reports_dir, exist_ok=True)
    meta = payload["meta"]
    html = _HTML.format(
        experiment_id=meta.get("experiment_id", ""),
        out_folder=meta.get("out_folder", ""),
        payload=json.dumps(payload, ensure_ascii=False),
    )
    p = os.path.join(reports_dir, "interactive_dashboard.html")
    with open(p, "w", encoding="utf-8") as f:
        f.write(html)
    return p
