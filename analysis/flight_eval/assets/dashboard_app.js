/* dashboard_app.js v2 — 飞行误差分析仪表板前端 v2.0
 * 纯 SVG，无外部依赖，离线可用。前端只渲染、不重算核心指标。
 * 主要变更 v2：
 *   - XY 轨迹：双游标（GPS 圆 + VIO 菱），选中段同时高亮 GPS + VIO 轨迹
 *   - 位置误差主面板（合并 XY误差+横/垂误差），内部切换：航迹系 | ENU
 *   - 速度面板：同时显示 vE/vN/vU（三分量），内部切换：ENU | 航迹系
 *   - 所有误差线图叠加航段背景色带，gap 缺口有标注图例
 *   - 去掉全局 errMode/velMode 控制按钮（移入各图表标题栏）
 *   - 航段浏览器：新增沿/横/垂局部误差列，选中段自动滚动到可见区域
 *   - 航段明细：新增局部系沿/横/垂终点误差，使用段净位移航向
 *   - GPS 采样质量移入可折叠诊断面板，gap 标注有图例
 *   - LK 不可用时清晰显示原因
 */
(function () {
  'use strict';
  var D = window.RUN_DATA;
  if (!D) { document.body.innerHTML = '<p style="padding:24px;font-family:sans-serif">缺少 window.RUN_DATA</p>'; return; }
  var S = D.samples || [], SEG = D.segments || [], SUM = D.summary || {}, Q = D.sampling || {}, M = D.meta || {};
  var C = {
    gps: '#1f7a4d', vio: '#2563c9', lk: '#e07b1a', err: '#d33a35',
    invalid: '#9aa3ad', sel: '#8b5cf6', ink: '#1d2733', muted: '#828c99',
    line: '#e6e9ed', line2: '#eef0f3',
    gps2: '#2e9d6a', gps3: '#3fbf85',
    vio2: '#4b7dd6', vio3: '#7ea5e8',
    along: '#8b5cf6', cross: '#d33a35', vert: '#e07b1a',
  };
  var NS = 'http://www.w3.org/2000/svg';
  var lkOn = !!(M.lk && M.lk.available && S.length && ('lk_E' in S[0]));
  var fcFallback = (M.velocity_source || '').indexOf('flight_controller') < 0;

  // ── helpers ──────────────────────────────────────────────────────────────
  function el(t, c, h) { var e = document.createElement(t); if (c) e.className = c; if (h != null) e.innerHTML = h; return e; }
  function sv(t, a) { var e = document.createElementNS(NS, t); for (var k in a) e.setAttribute(k, a[k]); return e; }
  function f(v, d) { return (v == null || isNaN(v)) ? '—' : (+v).toFixed(d == null ? 2 : d); }
  function decimate(a, n) { if (!a || a.length <= n) return (a || []).slice(); var s = a.length / n, o = []; for (var i = 0; i < a.length; i += s) o.push(a[Math.floor(i)]); if (o[o.length - 1] !== a[a.length - 1]) o.push(a[a.length - 1]); return o; }
  function ticks(lo, hi, n) { var sp = hi - lo, raw = sp / n, mag = Math.pow(10, Math.floor(Math.log10(raw || 1))), nm = raw / mag, st = mag * (nm < 1.5 ? 1 : nm < 3 ? 2 : nm < 7 ? 5 : 10), o = []; for (var v = Math.ceil(lo / st) * st; v <= hi + 1e-9; v += st) o.push(v); return o; }
  function segOf(id) { for (var i = 0; i < SEG.length; i++) if (SEG[i].segment_id === id) return SEG[i]; return null; }
  function nearestByT(data, t) { var b = data[0], bd = 1e18; for (var i = 0; i < data.length; i++) { var dd = Math.abs(data[i].t - t); if (dd < bd) { bd = dd; b = data[i]; } } return b; }
  function gapRange(g) { if (!g) return null; if (Array.isArray(g)) return [g[0], g[1]]; var a = g.t0 != null ? g.t0 : g.t_start, b = g.t1 != null ? g.t1 : g.t_end; return (a == null || b == null) ? null : [a, b]; }

  function segBgFill(stype) { if (stype === 'straight') return 'rgba(37,99,201,0.07)'; if (stype === 'connector' || stype === 'turn') return 'rgba(224,123,26,0.09)'; return 'rgba(154,163,173,0.09)'; }
  function segFgColor(stype) { if (stype === 'straight') return C.vio; if (stype === 'connector' || stype === 'turn') return C.lk; return C.muted; }

  // Along/cross velocity projection (frontend, uses GPS course for both)
  function vAlong(p, gps) { var c = p.gps_course_deg * Math.PI / 180, ve = gps ? p.gps_vE : p.vio_vE, vn = gps ? p.gps_vN : p.vio_vN; if (ve == null || vn == null) return null; return ve * Math.cos(c) + vn * Math.sin(c); }
  function vCross(p, gps) { var c = p.gps_course_deg * Math.PI / 180, ve = gps ? p.gps_vE : p.vio_vE, vn = gps ? p.gps_vN : p.vio_vN; if (ve == null || vn == null) return null; return -ve * Math.sin(c) + vn * Math.cos(c); }

  // ── cursor / tooltip bus ─────────────────────────────────────────────────
  var bus = [];
  function onCursor(fn) { bus.push(fn); }
  function emit(t, src) { for (var i = 0; i < bus.length; i++) bus[i](t, src); }
  var tip = el('div'); tip.style.cssText = 'position:fixed;z-index:9999;pointer-events:none;background:#11171f;color:#eef2f6;border-radius:6px;padding:7px 9px;font:11px/1.5 ui-monospace,monospace;box-shadow:0 6px 20px rgba(0,0,0,.25);opacity:0;transition:opacity .08s;max-width:300px'; document.body.appendChild(tip);
  function showTip(html, ev) { tip.innerHTML = html; tip.style.opacity = 1; var w = tip.offsetWidth, h = tip.offsetHeight, x = ev.clientX + 14, y = ev.clientY + 14; if (x + w > innerWidth) x = ev.clientX - w - 14; if (y + h > innerHeight) y = ev.clientY - h - 14; tip.style.left = x + 'px'; tip.style.top = y + 'px'; }
  function hideTip() { tip.style.opacity = 0; }

  // ── view state ────────────────────────────────────────────────────────────
  var st = { gps: true, vio: true, lk: lkOn, seg: true, axis: 'dist', posMode: 'acv', velSys: 'enu', showInvalid: false, scope: 'full', segFilter: 'all', sel: null };
  function scoped() { if (st.scope === 'seg' && st.sel != null) { var s = segOf(st.sel); if (s) return S.filter(function (p) { return p.t >= s.t_start && p.t <= s.t_end; }); } return S; }
  function xKey() { return st.axis === 't' ? 't' : 'cum_dist'; }

  // ── segment background bands ─────────────────────────────────────────────
  function drawSegBands(gEl, iw, ih, m, X, xk) {
    SEG.forEach(function (s) {
      var x0 = X(xk === 't' ? (s.t_start || 0) : (s.d_start || 0));
      var x1 = X(xk === 't' ? (s.t_end || 0) : (s.d_end || 0));
      if (x1 <= m.l || x0 >= m.l + iw) return;
      x0 = Math.max(x0, m.l); x1 = Math.min(x1, m.l + iw);
      var w = Math.max(0.5, x1 - x0);
      gEl.appendChild(sv('rect', { x: x0, y: m.t, width: w, height: ih, fill: segBgFill(s.segment_type) }));
      if (st.sel === s.segment_id)
        gEl.appendChild(sv('rect', { x: x0, y: m.t, width: w, height: ih, fill: 'rgba(139,92,246,0.12)' }));
    });
    (Q.gaps || []).forEach(function (g) {
      var gr = gapRange(g); if (!gr) return;
      var x0, x1;
      if (xk === 't') { x0 = X(gr[0]); x1 = X(gr[1]); }
      else {
        var pa = S.find(function (p) { return p.t >= gr[0]; });
        var pb = S.find(function (p) { return p.t >= gr[1]; });
        if (!pa || !pb) return;
        x0 = X(pa.cum_dist); x1 = X(pb.cum_dist);
      }
      x0 = Math.max(x0, m.l); x1 = Math.min(x1, m.l + iw);
      if (x1 > x0) gEl.appendChild(sv('rect', { x: x0, y: m.t, width: Math.max(2, x1 - x0), height: ih, fill: 'rgba(154,163,173,0.22)', stroke: '#9aa3ad', 'stroke-width': 0.5, 'stroke-dasharray': '3 2' }));
    });
  }

  // ── line chart ────────────────────────────────────────────────────────────
  function lineChart(opts) {
    var host = el('div'); var W = 720, H = opts.height || 196, m = { l: 52, r: 14, t: 12, b: 30 };
    var iw = W - m.l - m.r, ih = H - m.t - m.b;
    var svg = sv('svg', { viewBox: '0 0 ' + W + ' ' + H, style: 'width:100%;display:block;font-family:ui-monospace,monospace' }); host.appendChild(svg);
    var xk = xKey(), data = decimate(opts.data || scoped(), 440);
    var xs = data.map(function (p) { return p[xk]; }), xmin = Math.min.apply(0, xs), xmax = Math.max.apply(0, xs);
    var vis = {}; opts.series.forEach(function (s) { vis[s.key] = s.on !== false; });
    function ydom() {
      var lo = 1e18, hi = -1e18;
      opts.series.forEach(function (s) { if (!vis[s.key]) return; data.forEach(function (p) { var y = s.f(p); if (y == null || isNaN(y)) return; if (y < lo) lo = y; if (y > hi) hi = y; }); });
      if (lo === 1e18) { lo = -1; hi = 1; }
      if (opts.zero) { lo = Math.min(lo, 0); hi = Math.max(hi, 0); }
      var pad = (hi - lo) * .12 || 1; return [lo - pad, hi + pad];
    }
    var yd = ydom();
    function X(v) { return m.l + (v - xmin) / (xmax - xmin || 1) * iw; }
    function Y(v) { return m.t + (1 - (v - yd[0]) / (yd[1] - yd[0] || 1)) * ih; }
    var gBands = sv('g', {}), gG = sv('g', {}), gD = sv('g', {}), gC = sv('g', {});
    [gBands, gG, gD, gC].forEach(function (g) { svg.appendChild(g); });
    function render() {
      yd = ydom(); gBands.textContent = ''; gG.textContent = ''; gD.textContent = '';
      if (opts.segBands !== false) drawSegBands(gBands, iw, ih, m, X, xk);
      ticks(yd[0], yd[1], 4).forEach(function (v) {
        gG.appendChild(sv('line', { x1: m.l, x2: m.l + iw, y1: Y(v), y2: Y(v), stroke: C.line2 }));
        var t = sv('text', { x: m.l - 6, y: Y(v) + 3, 'text-anchor': 'end', fill: C.muted, 'font-size': 9.5 }); t.textContent = Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(0) : v.toFixed(1); gG.appendChild(t);
      });
      ticks(xmin, xmax, 5).forEach(function (v) { var t = sv('text', { x: X(v), y: H - 15, 'text-anchor': 'middle', fill: C.muted, 'font-size': 9.5 }); t.textContent = xk === 't' ? Math.round(v) : (v / 1000).toFixed(1); gG.appendChild(t); });
      var xl = sv('text', { x: m.l + iw, y: H - 2, 'text-anchor': 'end', fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); xl.textContent = xk === 't' ? '时间 t / s' : 'GPS 累计里程 / km'; gG.appendChild(xl);
      if (opts.yLabel) { var yl = sv('text', { x: 4, y: 10, fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); yl.textContent = opts.yLabel; gG.appendChild(yl); }
      if (opts.zero && yd[0] < 0 && yd[1] > 0) gG.appendChild(sv('line', { x1: m.l, x2: m.l + iw, y1: Y(0), y2: Y(0), stroke: '#bcc3cc', 'stroke-dasharray': '3 3' }));
      opts.series.forEach(function (s) {
        if (!vis[s.key]) return; var d = '', started = false;
        data.forEach(function (p) { var y = s.f(p); var broken = !st.showInvalid && p.valid === false; if (y == null || isNaN(y) || broken) { started = false; return; } d += (started ? 'L' : 'M') + X(p[xk]).toFixed(1) + ' ' + Y(y).toFixed(1) + ' '; started = true; });
        gD.appendChild(sv('path', { d: d, fill: 'none', stroke: s.color, 'stroke-width': s.w || 1.6, 'stroke-dasharray': s.dash || '', 'stroke-linejoin': 'round', opacity: s.dim ? .5 : 1 }));
      });
      if (st.showInvalid) data.filter(function (p) { return p.valid === false && !p.gap; }).forEach(function (p) { var s0 = opts.series.find(function (s) { return vis[s.key]; }); if (!s0) return; var y = s0.f(p); if (y == null) return; gD.appendChild(sv('circle', { cx: X(p[xk]), cy: Y(y), r: 2.3, fill: 'none', stroke: C.invalid, 'stroke-width': 1.1 })); });
    }
    var cl = sv('line', { y1: m.t, y2: m.t + ih, stroke: C.ink, 'stroke-width': 1, 'stroke-dasharray': '3 3', opacity: 0 }); gC.appendChild(cl);
    var dots = opts.series.map(function () { var c = sv('circle', { r: 3, fill: '#fff', 'stroke-width': 1.6, opacity: 0 }); gC.appendChild(c); return c; });
    function nearest(xv) { var b = data[0], bd = 1e18; for (var i = 0; i < data.length; i++) { var dd = Math.abs(data[i][xk] - xv); if (dd < bd) { bd = dd; b = data[i]; } } return b; }
    function moveTo(xv, ev) {
      var p = nearest(xv), px = X(p[xk]); cl.setAttribute('x1', px); cl.setAttribute('x2', px); cl.setAttribute('opacity', 1);
      var rows = ''; if (ev) { var head = xk === 't' ? 't=' + f(p.t, 1) + 's' : '里程=' + f(p.cum_dist / 1000, 3) + 'km · t=' + f(p.t, 0) + 's'; rows = '<div style="color:#7fb2ff;margin-bottom:3px">' + head + '</div><div style="font-size:10px;color:' + C.gps + '">GPS E=' + f(p.gps_E, 0) + ' N=' + f(p.gps_N, 0) + ' m</div><div style="font-size:10px;color:' + C.vio + '">VIO E=' + f(p.vio_E, 0) + ' N=' + f(p.vio_N, 0) + ' m</div>'; }
      opts.series.forEach(function (s, i) { var c = dots[i]; if (!vis[s.key]) { c.setAttribute('opacity', 0); return; } var y = s.f(p); if (y == null || isNaN(y)) { c.setAttribute('opacity', 0); return; } c.setAttribute('cx', px); c.setAttribute('cy', Y(y)); c.setAttribute('stroke', s.color); c.setAttribute('opacity', 1); rows += '<div style="display:flex;gap:8px;justify-content:space-between"><span><span style="display:inline-block;width:8px;height:8px;border-radius:2px;background:' + s.color + ';margin-right:4px"></span>' + s.label + '</span><b>' + f(y, s.dp == null ? 2 : s.dp) + (opts.unit || '') + '</b></div>'; });
      if (ev) showTip(rows, ev);
    }
    function clear() { cl.setAttribute('opacity', 0); dots.forEach(function (d) { d.setAttribute('opacity', 0); }); }
    var rect = sv('rect', { x: m.l, y: m.t, width: iw, height: ih, fill: 'transparent', cursor: 'crosshair' }); gC.appendChild(rect);
    rect.addEventListener('mousemove', function (e) { var r = svg.getBoundingClientRect(); var xpix = (e.clientX - r.left) / r.width * W; var xv = xmin + (xpix - m.l) / iw * (xmax - xmin); moveTo(xv, e); emit(nearest(xv).t, host); });
    rect.addEventListener('mouseleave', function () { hideTip(); clear(); emit(null, host); });
    onCursor(function (t, src) { if (src === host) return; if (t == null) { clear(); return; } moveTo(nearestByT(data, t)[xk], null); });
    render();
    return { el: host, toggle: function (k) { vis[k] = !vis[k]; render(); return vis[k]; }, refresh: function () { data = decimate(opts.data || scoped(), 440); xk = xKey(); xs = data.map(function (p) { return p[xk]; }); xmin = Math.min.apply(0, xs); xmax = Math.max.apply(0, xs); render(); } };
  }

  // ── trajectory chart ──────────────────────────────────────────────────────
  function trajChart(opts) {
    var host = el('div'); var W = 720, H = opts.height || 380, mg = 28;
    var svg = sv('svg', { viewBox: '0 0 ' + W + ' ' + H, style: 'width:100%;display:block;font-family:ui-monospace,monospace' }); host.appendChild(svg);
    var data = decimate(S, 900), full = S;
    var lo = [1e18, 1e18], hi = [-1e18, -1e18];
    full.forEach(function (p) { ['gps', 'vio'].concat(lkOn ? ['lk'] : []).forEach(function (k) { var e = p[k + '_E'], n = p[k + '_N']; if (e == null) return; if (e < lo[0]) lo[0] = e; if (e > hi[0]) hi[0] = e; if (n < lo[1]) lo[1] = n; if (n > hi[1]) hi[1] = n; }); });
    var span = Math.max(hi[0] - lo[0], hi[1] - lo[1]) * 1.10 || 1, cx = (lo[0] + hi[0]) / 2, cy = (lo[1] + hi[1]) / 2;
    var sc = Math.min(W - 2 * mg, H - 2 * mg) / span;
    function X(e) { return W / 2 + (e - cx) * sc; } function Y(n) { return H / 2 - (n - cy) * sc; }
    var gGrid = sv('g', {}), gSeg = sv('g', {}), gPath = sv('g', {}), gMark = sv('g', {}), gAnim = sv('g', {}), gCur = sv('g', {});
    [gGrid, gSeg, gPath, gMark, gAnim, gCur].forEach(function (g) { svg.appendChild(g); });
    for (var eg = Math.ceil(lo[0] / 500) * 500; eg <= hi[0]; eg += 500) gGrid.appendChild(sv('line', { x1: X(eg), x2: X(eg), y1: mg, y2: H - mg, stroke: C.line2 }));
    for (var ng = Math.ceil(lo[1] / 500) * 500; ng <= hi[1]; ng += 500) gGrid.appendChild(sv('line', { x1: mg, x2: W - mg, y1: Y(ng), y2: Y(ng), stroke: C.line2 }));
    function trajPath(key, color, w, dash, on) { if (!on) return; var d = ''; data.forEach(function (p, i) { if (p[key + '_E'] == null) return; d += (i ? 'L' : 'M') + X(p[key + '_E']).toFixed(1) + ' ' + Y(p[key + '_N']).toFixed(1) + ' '; }); gPath.appendChild(sv('path', { d: d, fill: 'none', stroke: color, 'stroke-width': w, 'stroke-dasharray': dash || '', 'stroke-linejoin': 'round' })); }
    function render() {
      gSeg.textContent = ''; gPath.textContent = ''; gMark.textContent = '';
      if (st.seg) SEG.forEach(function (s) {
        if (s.segment_id == null) return;
        var pts = full.filter(function (p) { return p.segment_id === s.segment_id; }); if (!pts.length) return;
        var seld = st.sel === s.segment_id, turn = s.segment_type === 'turn' || s.segment_type === 'connector';
        var dg = ''; decimate(pts, 60).forEach(function (p, i) { dg += (i ? 'L' : 'M') + X(p.gps_E).toFixed(1) + ' ' + Y(p.gps_N).toFixed(1) + ' '; });
        gSeg.appendChild(sv('path', { d: dg, fill: 'none', stroke: seld ? C.sel : (turn ? 'rgba(224,123,26,.28)' : 'rgba(37,99,201,.15)'), 'stroke-width': seld ? 8 : 6, 'stroke-linecap': 'round', 'data-seg': s.segment_id, cursor: 'pointer' }));
        if (seld) {
          var dv = ''; decimate(pts, 60).forEach(function (p, i) { dv += (i ? 'L' : 'M') + X(p.vio_E).toFixed(1) + ' ' + Y(p.vio_N).toFixed(1) + ' '; });
          gSeg.appendChild(sv('path', { d: dv, fill: 'none', stroke: 'rgba(139,92,246,0.55)', 'stroke-width': 5, 'stroke-linecap': 'round', 'stroke-dasharray': '6 3' }));
          var p0 = pts[0], pz = pts[pts.length - 1];
          gSeg.appendChild(sv('circle', { cx: X(p0.gps_E), cy: Y(p0.gps_N), r: 6, fill: 'none', stroke: C.sel, 'stroke-width': 2 }));
          gSeg.appendChild(sv('circle', { cx: X(p0.vio_E), cy: Y(p0.vio_N), r: 6, fill: 'none', stroke: '#8b5cf6', 'stroke-width': 2, 'stroke-dasharray': '3 2' }));
          gSeg.appendChild(sv('rect', { x: X(pz.gps_E) - 4, y: Y(pz.gps_N) - 4, width: 8, height: 8, fill: C.sel }));
          gSeg.appendChild(sv('rect', { x: X(pz.vio_E) - 4, y: Y(pz.vio_N) - 4, width: 8, height: 8, fill: '#8b5cf6', opacity: 0.7 }));
        }
      });
      trajPath('gps', C.gps, 2.2, '', st.gps); trajPath('vio', C.vio, 1.8, '', st.vio); if (lkOn) trajPath('lk', C.lk, 1.6, '5 4', st.lk);
      var a = data[0], z = data[data.length - 1];
      if (st.gps) { gMark.appendChild(sv('circle', { cx: X(a.gps_E), cy: Y(a.gps_N), r: 5, fill: '#fff', stroke: C.gps, 'stroke-width': 2 })); gMark.appendChild(sv('rect', { x: X(z.gps_E) - 4, y: Y(z.gps_N) - 4, width: 8, height: 8, fill: C.gps })); }
      if (st.vio) gMark.appendChild(sv('rect', { x: X(z.vio_E) - 4, y: Y(z.vio_N) - 4, width: 8, height: 8, fill: C.vio }));
      if (st.lk && lkOn) gMark.appendChild(sv('rect', { x: X(z.lk_E) - 4, y: Y(z.lk_N) - 4, width: 8, height: 8, fill: C.lk }));
      var lab = sv('text', { x: X(a.gps_E) + 8, y: Y(a.gps_N) - 6, fill: C.muted, 'font-size': 9.5 }); lab.textContent = '起点 START'; gMark.appendChild(lab);
    }
    gSeg.addEventListener('click', function (e) { var id = e.target.getAttribute && e.target.getAttribute('data-seg'); if (id != null) selectSeg(+id); });
    // 双游标：GPS 圆 + VIO 圆
    var curGPS = sv('circle', { r: 4.5, fill: '#fff', stroke: C.gps, 'stroke-width': 2, opacity: 0 }); gCur.appendChild(curGPS);
    var curVIO = sv('circle', { r: 4.5, fill: '#fff', stroke: C.vio, 'stroke-width': 2, opacity: 0 }); gCur.appendChild(curVIO);
    svg.addEventListener('mousemove', function (e) {
      var r = svg.getBoundingClientRect(), ex = (e.clientX - r.left) / r.width * W - W / 2, ey = (e.clientY - r.top) / r.height * H - H / 2;
      var enu_e = ex / sc + cx, enu_n = -ey / sc + cy, best = full[0], bd = 1e18;
      for (var i = 0; i < full.length; i++) { var de = full[i].gps_E - enu_e, dn = full[i].gps_N - enu_n, dd = de * de + dn * dn; if (dd < bd) { bd = dd; best = full[i]; } }
      emit(best.t, host);
      showTip('<div style="color:#7fb2ff;margin-bottom:3px">t=' + f(best.t, 1) + 's · ' + f(best.cum_dist / 1000, 3) + 'km</div><div style="color:' + C.gps + '">GPS E=' + f(best.gps_E, 0) + ' N=' + f(best.gps_N, 0) + ' U=' + f(best.gps_U, 0) + 'm</div><div style="color:' + C.vio + '">VIO E=' + f(best.vio_E, 0) + ' N=' + f(best.vio_N, 0) + ' U=' + f(best.vio_U, 0) + 'm</div>' + (best.err_XY != null ? '<div style="color:' + C.err + '">XY误差=' + f(best.err_XY, 1) + 'm · along=' + f(best.err_along, 1) + ' cross=' + f(best.err_cross, 1) + 'm</div>' : ''), e);
    });
    svg.addEventListener('mouseleave', function () { hideTip(); emit(null, host); });
    onCursor(function (t) { if (t == null) { curGPS.setAttribute('opacity', 0); curVIO.setAttribute('opacity', 0); return; } var p = nearestByT(full, t); if (!p) return; curGPS.setAttribute('cx', X(p.gps_E)); curGPS.setAttribute('cy', Y(p.gps_N)); curGPS.setAttribute('opacity', 1); curVIO.setAttribute('cx', X(p.vio_E)); curVIO.setAttribute('cy', Y(p.vio_N)); curVIO.setAttribute('opacity', 1); });
    // animation
    var playing = false, raf = null, idx = 0;
    function trail(key, color, w, i) { var d = ''; for (var k = 0; k <= i; k++) { d += (k ? 'L' : 'M') + X(full[k][key + '_E']).toFixed(1) + ' ' + Y(full[k][key + '_N']).toFixed(1) + ' '; } gAnim.appendChild(sv('path', { d: d, fill: 'none', stroke: color, 'stroke-width': w, 'stroke-linejoin': 'round' })); var h = full[i]; gAnim.appendChild(sv('circle', { cx: X(h[key + '_E']), cy: Y(h[key + '_N']), r: 4.5, fill: color, stroke: '#fff', 'stroke-width': 1.6 })); }
    function stop() { playing = false; if (raf) cancelAnimationFrame(raf); raf = null; gAnim.textContent = ''; gPath.setAttribute('opacity', 1); if (opts.onPlay) opts.onPlay(false); }
    function play() { if (playing) { stop(); return; } playing = true; if (idx >= full.length - 1) idx = 0; if (opts.onPlay) opts.onPlay(true); var step = Math.max(1, Math.round(full.length / 260)), last = 0; function tick(ts) { if (!playing) return; if (ts - last > 28) { last = ts; idx += step; if (idx >= full.length - 1) { idx = full.length - 1; gAnim.textContent = ''; gPath.setAttribute('opacity', .18); if (st.gps) trail('gps', C.gps, 2.6, idx); if (st.vio) trail('vio', C.vio, 2.2, idx); if (st.lk && lkOn) trail('lk', C.lk, 1.8, idx); emit(full[idx].t, host); if (opts.onFrame) opts.onFrame(full[idx]); stop(); return; } gAnim.textContent = ''; gPath.setAttribute('opacity', .18); if (st.gps) trail('gps', C.gps, 2.6, idx); if (st.vio) trail('vio', C.vio, 2.2, idx); if (st.lk && lkOn) trail('lk', C.lk, 1.8, idx); emit(full[idx].t, host); if (opts.onFrame) opts.onFrame(full[idx]); } raf = requestAnimationFrame(tick); } raf = requestAnimationFrame(tick); }
    render();
    return { el: host, play: play, stop: stop, render: render };
  }

  // ── segment start-aligned chart (uses GPS net-displacement heading) ────────
  function segAlignChart(pts) {
    var host = el('div'); var W = 720, H = 220, m = { l: 52, r: 16, t: 14, b: 30 }, iw = W - m.l - m.r, ih = H - m.t - m.b;
    var svg = sv('svg', { viewBox: '0 0 ' + W + ' ' + H, style: 'width:100%;display:block;font-family:ui-monospace,monospace' }); host.appendChild(svg);
    if (!pts || pts.length < 2) { host.appendChild(el('div', null, '<div style="padding:20px;color:var(--muted)">样本不足</div>')); return host; }
    // GPS 净位移方向作为局部 along 轴（与 Python segment_errors 保持一致）
    var gE0 = pts[0].gps_E, gN0 = pts[0].gps_N, gEz = pts[pts.length - 1].gps_E, gNz = pts[pts.length - 1].gps_N;
    var dE = gEz - gE0, dN = gNz - gN0, span = Math.hypot(dE, dN);
    var ea, ec;
    if (span >= 10) {
      ea = [dE / span, dN / span];
    } else {
      var cr = pts.map(function (p) { return p.gps_course_deg * Math.PI / 180; });
      var cm = Math.cos(cr.reduce(function (a, b) { return a + b; }) / cr.length);
      var sm = Math.sin(cr.reduce(function (a, b) { return a + b; }) / cr.length);
      var nm = Math.hypot(cm, sm); ea = nm > 0.05 ? [cm / nm, sm / nm] : [1, 0];
    }
    ec = [-ea[1], ea[0]];
    function pjG(p) { var dE = p.gps_E - gE0, dN = p.gps_N - gN0; return { a: dE * ea[0] + dN * ea[1], c: dE * ec[0] + dN * ec[1] }; }
    var vE0 = pts[0].vio_E, vN0 = pts[0].vio_N;
    function pjV(p) { var dE = p.vio_E - vE0, dN = p.vio_N - vN0; return { a: dE * ea[0] + dN * ea[1], c: dE * ec[0] + dN * ec[1] }; }
    var g = pts.map(pjG), v = pts.map(pjV);
    var aAll = g.concat(v).map(function (p) { return p.a; }), cAll = g.concat(v).map(function (p) { return p.c; });
    var aMin = Math.min.apply(0, aAll), aMax = Math.max.apply(0, aAll), cMin = Math.min.apply(0, cAll), cMax = Math.max.apply(0, cAll);
    var cpad = (cMax - cMin) * .30 || .5; cMin -= cpad; cMax += cpad;
    function XA(a) { return m.l + (a - aMin) / ((aMax - aMin) || 1) * iw; }
    function YC(cc) { return m.t + (1 - (cc - cMin) / ((cMax - cMin) || 1)) * ih; }
    var exag = ((aMax - aMin) / iw) / (((cMax - cMin) / ih) || 1);
    svg.appendChild(sv('line', { x1: m.l, x2: m.l + iw, y1: YC(0), y2: YC(0), stroke: '#bcc3cc', 'stroke-dasharray': '3 3' }));
    ticks(cMin, cMax, 4).forEach(function (cc) { var t = sv('text', { x: m.l - 6, y: YC(cc) + 3, 'text-anchor': 'end', fill: C.muted, 'font-size': 9.5 }); t.textContent = cc.toFixed(1); svg.appendChild(t); });
    function poly(arr, color, w) { var d = ''; arr.forEach(function (p, i) { d += (i ? 'L' : 'M') + XA(p.a).toFixed(1) + ' ' + YC(p.c).toFixed(1) + ' '; }); svg.appendChild(sv('path', { d: d, fill: 'none', stroke: color, 'stroke-width': w, 'stroke-linejoin': 'round' })); }
    poly(g, C.gps, 2.2); poly(v, C.vio, 2);
    svg.appendChild(sv('circle', { cx: XA(0), cy: YC(0), r: 5, fill: '#fff', stroke: C.ink, 'stroke-width': 2 }));
    var ge = g[g.length - 1], ve = v[v.length - 1];
    svg.appendChild(sv('rect', { x: XA(ge.a) - 4, y: YC(ge.c) - 4, width: 8, height: 8, fill: C.gps }));
    svg.appendChild(sv('rect', { x: XA(ve.a) - 4, y: YC(ve.c) - 4, width: 8, height: 8, fill: C.vio }));
    // along error arrow
    svg.appendChild(sv('line', { x1: XA(ge.a), x2: XA(ve.a), y1: YC(0) - 6, y2: YC(0) - 6, stroke: C.along, 'stroke-width': 1.4, 'stroke-dasharray': '3 2' }));
    var at = sv('text', { x: (XA(ge.a) + XA(ve.a)) / 2, y: YC(0) - 10, 'text-anchor': 'middle', fill: C.along, 'font-size': 9.5 }); at.textContent = '沿 ' + ve.a.toFixed(2) + 'm'; svg.appendChild(at);
    // cross error arrow
    svg.appendChild(sv('line', { x1: XA(ve.a), x2: XA(ve.a), y1: YC(0), y2: YC(ve.c), stroke: C.cross, 'stroke-width': 1.4, 'stroke-dasharray': '3 2' }));
    var ct = sv('text', { x: XA(ve.a) + 6, y: (YC(0) + YC(ve.c)) / 2 + 3, fill: C.cross, 'font-size': 9.5 }); ct.textContent = '横 ' + ve.c.toFixed(2) + 'm'; svg.appendChild(ct);
    var xl = sv('text', { x: m.l + iw, y: H - 3, 'text-anchor': 'end', fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); xl.textContent = '沿航向 along / m（GPS 起止点净位移方向）'; svg.appendChild(xl);
    var yl = sv('text', { x: 4, y: 10, fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); yl.textContent = '横 / m (放大 ×' + exag.toFixed(0) + ')'; svg.appendChild(yl);
    return host;
  }

  // ── bar chart (drift) ─────────────────────────────────────────────────────
  function barChart(metric) {
    var host = el('div');
    var items = SEG.filter(function (s) { return st.segFilter === 'all' || (st.segFilter === 'straight' && s.segment_type === 'straight') || (st.segFilter === 'turn' && (s.segment_type === 'turn' || s.segment_type === 'connector')); })
      .map(function (s) { return { id: s.segment_id, label: '#' + s.segment_id + ' ' + (s.segment_type === 'straight' ? '直' : (s.segment_type === 'turn' || s.segment_type === 'connector') ? '转' : '·'), value: metric === 'speed' ? s.speed_rmse_mps : s.local_drift_percent, color: segFgColor(s.segment_type) }; });
    var W = 720, bh = 17, gap = 5, padL = 74, padR = 62, Hh = Math.max(60, items.length * (bh + gap) + 10);
    var svg = sv('svg', { viewBox: '0 0 ' + W + ' ' + Hh, style: 'width:100%;display:block;font-family:ui-monospace,monospace' }); host.appendChild(svg);
    var max = Math.max.apply(0, items.map(function (d) { return d.value || 0; })) * 1.12 || 1;
    function X(v) { return padL + (v / max) * (W - padL - padR); }
    items.forEach(function (d, i) {
      var y = 5 + i * (bh + gap), seld = st.sel === d.id;
      var gg = sv('g', { cursor: 'pointer' }); gg.appendChild(sv('rect', { x: 0, y: y - 2, width: W, height: bh + 4, fill: seld ? 'rgba(139,92,246,.10)' : 'transparent' }));
      var l = sv('text', { x: padL - 6, y: y + bh / 2 + 3, 'text-anchor': 'end', fill: C.muted, 'font-size': 9 }); l.textContent = d.label; gg.appendChild(l);
      gg.appendChild(sv('rect', { x: padL, y: y, width: Math.max(1, X(d.value || 0) - padL), height: bh, rx: 2, fill: seld ? C.sel : d.color, opacity: d.value == null ? .25 : 1 }));
      var vt = sv('text', { x: X(d.value || 0) + 6, y: y + bh / 2 + 3, fill: C.muted, 'font-size': 9 }); vt.textContent = d.value == null ? 'n/a' : f(d.value, metric === 'speed' ? 3 : 2) + (metric === 'speed' ? ' m/s' : '%'); gg.appendChild(vt);
      gg.addEventListener('click', function () { selectSeg(d.id); }); svg.appendChild(gg);
    });
    return { el: host };
  }

  // ── sampling quality chart (used inside diagnostics panel) ────────────────
  function samplingChart() {
    var host = el('div'); var W = 720, H = 170, m = { l: 52, r: 14, t: 12, b: 28 }, iw = W - m.l - m.r, ih = H - m.t - m.b;
    var svg = sv('svg', { viewBox: '0 0 ' + W + ' ' + H, style: 'width:100%;display:block;font-family:ui-monospace,monospace' }); host.appendChild(svg);
    var data = decimate(S, 500), xs = data.map(function (p) { return p.t; }), xmin = Math.min.apply(0, xs), xmax = Math.max.apply(0, xs);
    var thr = Q.p95_delay_s || 0.2, ymax = Math.max(0.3, thr * 1.4);
    function X(v) { return m.l + (v - xmin) / (xmax - xmin || 1) * iw; } function Y(v) { return m.t + (1 - Math.min(v, ymax) / ymax) * ih; }
    (Q.gaps || []).forEach(function (g) { var gr = gapRange(g); if (!gr) return; svg.appendChild(sv('rect', { x: X(gr[0]), y: m.t, width: Math.max(3, X(gr[1]) - X(gr[0])), height: ih, fill: 'rgba(154,163,173,0.3)', stroke: '#9aa3ad', 'stroke-width': 0.5, 'stroke-dasharray': '3 2' })); });
    ticks(0, ymax, 4).forEach(function (v) { svg.appendChild(sv('line', { x1: m.l, x2: m.l + iw, y1: Y(v), y2: Y(v), stroke: C.line2 })); var t = sv('text', { x: m.l - 6, y: Y(v) + 3, 'text-anchor': 'end', fill: C.muted, 'font-size': 9.5 }); t.textContent = v.toFixed(2); svg.appendChild(t); });
    svg.appendChild(sv('line', { x1: m.l, x2: m.l + iw, y1: Y(thr), y2: Y(thr), stroke: C.err, 'stroke-width': 1.2, 'stroke-dasharray': '5 3' }));
    var tl = sv('text', { x: m.l + iw, y: Y(thr) - 4, 'text-anchor': 'end', fill: C.err, 'font-size': 9.5 }); tl.textContent = 'p95阈值 ' + thr.toFixed(3) + 's'; svg.appendChild(tl);
    data.forEach(function (p) { if (p.gap) return; var bad = p.vio_delay_s > thr; svg.appendChild(sv('circle', { cx: X(p.t), cy: Y(p.vio_delay_s || 0), r: bad ? 2.6 : 1.5, fill: bad ? C.err : C.vio, opacity: bad ? 1 : .55 })); });
    var yl = sv('text', { x: 4, y: 10, fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); yl.textContent = 'VIO 对齐延迟 / s'; svg.appendChild(yl);
    var xl = sv('text', { x: m.l + iw, y: H - 3, 'text-anchor': 'end', fill: C.ink, 'font-size': 10, 'font-family': 'sans-serif' }); xl.textContent = '时间 t / s'; svg.appendChild(xl);
    // gap legend
    var legRow = el('div', 'lgrow');
    legRow.appendChild(el('span', null, '<span class="lgi"><span style="display:inline-block;width:14px;height:10px;background:rgba(154,163,173,0.3);border:1px dashed #9aa3ad;border-radius:2px;margin-right:5px"></span><span style="font-size:11px">GPS缺口（VIO 插值补全）</span></span>'));
    legRow.appendChild(el('span', null, '<span class="lgi"><span style="display:inline-block;width:14px;height:10px;background:' + C.err + ';border-radius:7px;margin-right:5px"></span><span style="font-size:11px">延迟超阈值</span></span>'));
    host.appendChild(legRow);
    return host;
  }

  // ── panel / legend / badge ────────────────────────────────────────────────
  function panel(title, unit, q, body, o) {
    o = o || {}; var p = el('div', 'panel');
    var h = el('div', 'ch'); h.appendChild(el('span', 'ct', title)); if (unit) h.appendChild(el('span', 'cu', unit)); h.appendChild(el('span', 'sp')); if (o.badge) h.appendChild(o.badge);
    p.appendChild(h); if (q) p.appendChild(el('div', 'q', q));
    var cv = el('div', 'cv'); cv.appendChild(body); p.appendChild(cv);
    if (o.legend) p.appendChild(o.legend);
    return p;
  }
  function legend(series, chart) {
    var row = el('div', 'lgrow');
    // gap legend item (always shown if there are gaps)
    if (Q.gaps && Q.gaps.length) { var gi = el('span', 'lgi'); gi.innerHTML = '<span style="display:inline-block;width:14px;height:4px;background:rgba(154,163,173,0.35);border:1px dashed #9aa3ad;border-radius:1px;margin-right:4px"></span><span style="font-size:11px">GPS缺口</span>'; row.appendChild(gi); }
    series.forEach(function (s) { var i = el('span', 'lgi'); var ln = el('span', 'lgln'); ln.style.borderTopStyle = s.dash ? 'dashed' : 'solid'; ln.style.borderTopColor = s.color; i.appendChild(ln); i.appendChild(document.createTextNode(s.label)); if (chart) i.onclick = function () { var on = chart.toggle(s.key); i.style.opacity = on ? 1 : .4; }; row.appendChild(i); });
    return row;
  }
  function badge(txt, kind) { var b = el('span', 'badge ' + (kind || 'ok')); b.appendChild(el('span', 'dotc')); b.appendChild(document.createTextNode(txt)); return b; }
  function modeBar(key, opts_arr) {
    var g = el('div', 'segctl');
    opts_arr.forEach(function (o) { var b = el('button', st[key] === o[0] ? 'on' : '', o[1]); b.onclick = function () { st[key] = o[0]; rerender(); }; g.appendChild(b); });
    return g;
  }

  // ── position error panel (merged: acv/enu toggle) ─────────────────────────
  function posErrPanel() {
    var acv = st.posMode === 'acv';
    var seriesACV = [
      { key: 'al', label: '沿航向 along', color: C.along, w: 1.4, f: function (p) { return p.err_along; } },
      { key: 'cr', label: '横航向 cross', color: C.cross, w: 2.0, f: function (p) { return p.err_cross; } },
      { key: 'vt', label: '垂直 vertical', color: C.vert, w: 1.4, f: function (p) { return p.err_vertical; } },
      { key: 'xy', label: 'XY模值 ref', color: C.ink, w: 1.0, dash: '4 2', dim: true, f: function (p) { return p.err_XY; } },
    ];
    var seriesENU = [
      { key: 'ee', label: 'err_E (东)', color: C.vio, w: 1.6, f: function (p) { return p.err_E; } },
      { key: 'en', label: 'err_N (北)', color: C.gps, w: 1.6, f: function (p) { return p.err_N; } },
      { key: 'eu', label: 'err_U (天)', color: C.lk, w: 1.6, f: function (p) { return p.err_U; } },
      { key: 'xy', label: 'XY模值 ref', color: C.ink, w: 1.0, dash: '4 2', dim: true, f: function (p) { return p.err_XY; } },
    ];
    var series = acv ? seriesACV : seriesENU;
    var c = lineChart({ data: scoped(), height: 210, zero: true, unit: ' m', yLabel: '位置误差/m', segBands: true, series: series });
    var modeBadge = modeBar('posMode', [['acv', '航迹系 along/cross'], ['enu', 'ENU']]);
    var note = acv ? '沿/横/垂 = 全局投影至瞬时 GPS 航向（非段局部系）；段详情中可查段局部坐标系误差。' : 'ENU 三分量误差 + XY 模值参考线。背景色带标注航段类型（直线=蓝 转弯=橙 残段=灰）。';
    return panel('位置误差（主面板）', 'm', note, c.el, { legend: legend(series, c), badge: modeBadge });
  }

  // ── velocity panel (3 components, ENU/track toggle) ───────────────────────
  function velPanel() {
    var enu = st.velSys === 'enu';
    var prefix = fcFallback ? 'GPS差分' : '飞控';
    var series;
    if (enu) {
      series = [
        { key: 'gve', label: prefix + ' vE', color: C.gps, w: 2.0, f: function (p) { return p.gps_vE; }, dp: 2 },
        { key: 'gvn', label: prefix + ' vN', color: C.gps2, w: 1.6, dash: '6 3', f: function (p) { return p.gps_vN; }, dp: 2 },
        { key: 'gvu', label: prefix + ' vU', color: C.gps3, w: 1.4, dash: '2 3', f: function (p) { return p.gps_vU; }, dp: 2 },
        { key: 'vve', label: 'VIO vE', color: C.vio, w: 2.0, f: function (p) { return p.vio_vE; }, dp: 2 },
        { key: 'vvn', label: 'VIO vN', color: C.vio2, w: 1.6, dash: '6 3', f: function (p) { return p.vio_vN; }, dp: 2 },
        { key: 'vvu', label: 'VIO vU', color: C.vio3, w: 1.4, dash: '2 3', f: function (p) { return p.vio_vU; }, dp: 2 },
      ];
    } else {
      series = [
        { key: 'gal', label: prefix + ' v_along', color: C.gps, w: 2.0, f: function (p) { return vAlong(p, true); }, dp: 2 },
        { key: 'gcr', label: prefix + ' v_cross', color: C.gps2, w: 1.6, dash: '6 3', f: function (p) { return vCross(p, true); }, dp: 2 },
        { key: 'gvt', label: prefix + ' v_vert', color: C.gps3, w: 1.4, dash: '2 3', f: function (p) { return p.gps_vU; }, dp: 2 },
        { key: 'val', label: 'VIO v_along', color: C.vio, w: 2.0, f: function (p) { return vAlong(p, false); }, dp: 2 },
        { key: 'vcr', label: 'VIO v_cross', color: C.vio2, w: 1.6, dash: '6 3', f: function (p) { return vCross(p, false); }, dp: 2 },
        { key: 'vvt', label: 'VIO v_vert', color: C.vio3, w: 1.4, dash: '2 3', f: function (p) { return p.vio_vU; }, dp: 2 },
      ];
    }
    var c = lineChart({ data: scoped(), height: 196, unit: ' m/s', yLabel: '速度/m·s⁻¹', segBands: true, series: series });
    var note = enu ? '3分量对比（GPS/飞控 vs VIO）；实线=E/along，长虚=N/cross，短点=U/vert。' : '航迹系速度（前端投影：沿=v·e_a，横=v·e_c）。';
    var modeBadge = el('div', null, ''); modeBadge.style.display = 'flex'; modeBadge.style.gap = '6px'; modeBadge.style.alignItems = 'center';
    modeBadge.appendChild(modeBar('velSys', [['enu', 'ENU'], ['track', '航迹系']]));
    if (fcFallback) modeBadge.appendChild(badge('参考=位置差分 fallback', 'warn'));
    return panel('速度对比（3分量）', 'm/s', note, c.el, { legend: legend(series, c), badge: modeBadge });
  }

  // ── velocity error panel (3 components) ──────────────────────────────────
  function velErrPanel() {
    var enu = st.velSys === 'enu';
    var series;
    if (enu) {
      series = [
        { key: 've', label: 'err vE', color: C.vio, f: function (p) { return p.err_vE; }, dp: 3 },
        { key: 'vn', label: 'err vN', color: C.gps, f: function (p) { return p.err_vN; }, dp: 3 },
        { key: 'vu', label: 'err vU', color: C.lk, f: function (p) { return p.err_vU; }, dp: 3 },
      ];
    } else {
      series = [
        { key: 'val', label: 'err v_along', color: C.along, f: function (p) { return p.err_v_along; }, dp: 3 },
        { key: 'vcr', label: 'err v_cross', color: C.cross, f: function (p) { return p.err_v_cross; }, dp: 3 },
        { key: 'vvt', label: 'err v_vert', color: C.vert, f: function (p) { return p.err_v_vertical; }, dp: 3 },
      ];
    }
    var c = lineChart({ data: scoped(), height: 196, zero: true, unit: ' m/s', yLabel: '速度误差/m·s⁻¹', segBands: true, series: series });
    return panel(enu ? 'ENU 速度误差（3分量）' : '航迹系速度误差（3分量）', 'm/s', '问题：速度误差是否在转弯处变大？标量匹配但向量不匹配 → 提示航向误差。', c.el, { legend: legend(series, c) });
  }

  // ── course error chart ─────────────────────────────────────────────────────
  function chCourse() { var series = [{ key: 'c', label: 'VIO course − GPS course', color: C.err, f: function (p) { return p.course_err_deg; }, dp: 2 }]; var c = lineChart({ data: scoped(), height: 196, zero: true, unit: '°', yLabel: '航向误差/°', segBands: true, series: series }); return panel('航向误差', '°', '问题：是否存在持续增长的航向漂移（可解释横航向误差）？', c.el); }

  // ── diagnostics panel (collapsible, contains sampling chart) ─────────────
  function diagnosticsPanel() {
    var wrap = el('div', 'panel');
    var h = el('div', 'ch'); h.appendChild(el('span', 'ct', '诊断 · GPS采样质量'));
    var gapCnt = (Q.gaps || []).length, badCnt = Q.invalid_delay_count || 0;
    h.appendChild(el('span', 'sp')); h.appendChild(badge(badCnt + ' 延迟超限 · ' + gapCnt + ' 缺口', (badCnt || gapCnt) ? 'warn' : 'ok')); wrap.appendChild(h);
    wrap.appendChild(el('div', 'q', '问题：统计网格是否可信？缺口（灰色条带）= GPS 数据中断，VIO 靠自身传播；橙点 = VIO 对齐延迟超 p95 阈值。'));
    var cv = el('div', 'cv'); cv.appendChild(samplingChart()); wrap.appendChild(cv);
    // LK status
    var lkDiv = el('div'); lkDiv.style.cssText = 'padding:8px 13px 12px;font:11px ui-monospace,monospace';
    if (M.lk) {
      if (M.lk.available) {
        lkDiv.innerHTML = '<b style="color:' + C.lk + '">LK 光流</b>: 可用 · 模式=' + (M.lk.mode || '—') + (M.lk.scale ? ' · 尺度=' + (+M.lk.scale).toFixed(4) + 'm/px' : '');
      } else {
        var reason = M.lk.reason || M.lk.unavailable_reason || '未提供';
        lkDiv.innerHTML = '<b style="color:' + C.muted + '">LK 光流</b>: <span style="color:' + C.err + '">不可用</span> — 原因: ' + reason + '<br><small style="color:var(--muted)">LK 尺度需要 altitude_m 和 focal_px。缺少输入时仅 VIO 轨迹参与评估。</small>';
      }
    } else {
      lkDiv.innerHTML = '<span style="color:' + C.muted + '">LK 光流: 未配置（run spec 未提供 lk_traj / lk_flow_csv）</span>';
    }
    wrap.appendChild(lkDiv);
    return wrap;
  }

  // ── segment browser ───────────────────────────────────────────────────────
  function segBrowser() {
    var wrap = el('div', 'panel');
    var h = el('div', 'ch'); h.appendChild(el('span', 'ct', '航段浏览器')); h.appendChild(el('span', 'cu', SEG.length + ' 段')); h.appendChild(el('span', 'sp'));
    h.appendChild(modeBar('segFilter', [['all', '全部'], ['straight', '直线'], ['turn', '转弯']]));
    wrap.appendChild(h);
    var list = el('div', 'seglist');
    // Header row
    var hdr = el('div'); hdr.style.cssText = 'display:grid;grid-template-columns:60px 1fr 80px 70px 70px;gap:6px;padding:5px 13px;font:10px ui-monospace,monospace;color:var(--muted);border-bottom:1px solid var(--line)';
    ['类型', '标签 / 里程·时长', '漂移%', '横 RMSE', '沿 RMSE'].forEach(function (t) { hdr.appendChild(el('div', null, t)); }); list.appendChild(hdr);
    var filtered = SEG.filter(function (s) { return st.segFilter === 'all' || (st.segFilter === 'straight' && s.segment_type === 'straight') || (st.segFilter === 'turn' && (s.segment_type === 'turn' || s.segment_type === 'connector')); });
    var selectedRow = null;
    filtered.forEach(function (s) {
      var r = el('div'); r.style.cssText = 'display:grid;grid-template-columns:60px 1fr 80px 70px 70px;gap:6px;align-items:center;padding:7px 13px;border-bottom:1px solid var(--line2);cursor:pointer' + (st.sel === s.segment_id ? ';background:#f3eefe;box-shadow:inset 3px 0 0 #8b5cf6' : '');
      r.addEventListener('mouseover', function () { if (st.sel !== s.segment_id) r.style.background = '#f6f7f9'; });
      r.addEventListener('mouseout', function () { if (st.sel !== s.segment_id) r.style.background = ''; });
      var zh = s.segment_type === 'straight' ? '直线' : (s.segment_type === 'turn' || s.segment_type === 'connector') ? '转弯' : s.segment_type === 'partial' ? '残段' : s.segment_type;
      var ty = el('span', 'stype ' + ((s.segment_type === 'turn' || s.segment_type === 'connector') ? 'turn' : 'straight'), zh);
      var nm = el('div'); nm.innerHTML = '<div style="font-size:12px">#' + s.segment_id + ' ' + (s.label || '') + '</div><div style="font:10px ui-monospace,monospace;color:var(--muted)">' + f(s.dist_m, 0) + 'm · ' + f(s.duration_s, 0) + 's</div>';
      var driftVal = s.local_drift_percent == null ? 'n/a' : f(s.local_drift_percent, 2) + '%';
      var crossVal = s.cross_rmse_m == null ? 'n/a' : f(s.cross_rmse_m, 1) + 'm';
      var alongVal = s.along_rmse_m == null ? 'n/a' : f(s.along_rmse_m, 1) + 'm';
      [ty, nm, el('div', null, '<div style="font:11.5px ui-monospace,monospace">' + driftVal + '</div>'), el('div', null, '<div style="font:11.5px ui-monospace,monospace;color:' + C.cross + '">' + crossVal + '</div>'), el('div', null, '<div style="font:11.5px ui-monospace,monospace;color:' + C.along + '">' + alongVal + '</div>')].forEach(function (c) { r.appendChild(c); });
      r.onclick = function () { selectSeg(s.segment_id); };
      list.appendChild(r);
      if (st.sel === s.segment_id) selectedRow = r;
    });
    wrap.appendChild(list);
    if (selectedRow) setTimeout(function () { selectedRow.scrollIntoView({ block: 'nearest', behavior: 'smooth' }); }, 50);
    return wrap;
  }

  // ── segment detail ────────────────────────────────────────────────────────
  function segDetail() {
    var box = el('div', 'panel pad'); var s = segOf(st.sel);
    if (!s) { box.innerHTML = '<div class="muted" style="font-size:12.5px">未选择航段 — 在轨迹图、条形图或下方列表点击一段，查看段局部坐标系误差与<b>起点对齐轨迹</b>（沿航向/横航向偏移）。</div>'; return box; }
    var zh = s.segment_type === 'straight' ? '直线' : (s.segment_type === 'turn' || s.segment_type === 'connector') ? '转弯' : s.segment_type;
    box.appendChild(el('div', 'sdh', '<span class="stype ' + ((s.segment_type === 'turn' || s.segment_type === 'connector') ? 'turn' : 'straight') + '">' + zh + '</span><b>#' + s.segment_id + ' ' + (s.label || '') + '</b>'));
    function grid(title, color, rows) { box.appendChild(el('div', 'gh', title)).style.color = color; var g = el('div', 'g3'); rows.forEach(function (kv) { g.appendChild(el('div', null, '<div class="gk">' + kv[0] + '</div><div class="gv">' + kv[1] + '</div>')); }); box.appendChild(g); }
    grid('段局部坐标系（主判据）— 段 GPS 净位移方向为沿航向', C.sel, [
      ['里程', f(s.dist_m, 0) + ' m'],
      ['局部漂移率', s.local_drift_percent == null ? 'n/a' : f(s.local_drift_percent, 2) + '%'],
      ['终点 XY 误差', f(s.local_final_xy_error_m, 2) + ' m'],
      ['终点沿航向', f(s.local_final_along_error_m, 2) + ' m'],
      ['终点横航向', f(s.local_final_cross_error_m, 2) + ' m'],
      ['终点垂直', f(s.local_final_vertical_error_m, 2) + ' m'],
    ]);
    grid('段局部 RMSE（主判据）', C.ink, [
      ['沿航向 RMSE', f(s.along_rmse_m, 2) + ' m'],
      ['横航向 RMSE', f(s.cross_rmse_m, 2) + ' m'],
      ['垂直 RMSE', f(s.vertical_rmse_m, 2) + ' m'],
    ]);
    grid('全局累计误差（参考）', C.muted, [
      ['全局 XY RMSE', f(s.global_xy_rmse_m, 1) + ' m'],
      ['全局终点误差', f(s.global_final_xy_error_m, 1) + ' m'],
      ['速度 RMSE', f(s.speed_rmse_mps, 3) + ' m/s'],
    ]);
    box.appendChild(el('div', 'gh', '段内起点对齐轨迹（VIO 段起点平移到 GPS 段起点）')).style.color = C.gps;
    var pts = S.filter(function (p) { return p.t >= s.t_start && p.t <= s.t_end; });
    box.appendChild(segAlignChart(pts));
    box.appendChild(el('div', 'q', '以 GPS 净位移方向为沿轴（与上表 RMSE 一致），纵轴放大显示横向偏移。紫色箭头=沿航向误差，红色箭头=横航向误差。'));
    return box;
  }

  // ── provenance ────────────────────────────────────────────────────────────
  function provenance() {
    var wrap = el('div', 'panel'); var h = el('div', 'ch'); h.appendChild(el('span', 'ct', '溯源 / 配置 Provenance')); h.appendChild(el('span', 'sp')); h.appendChild(el('span', 'cu', 'metadata/*.json')); wrap.appendChild(h);
    var dl = el('div', 'kv');
    var lkStatus = M.lk ? (M.lk.available ? '可用 · ' + (M.lk.mode || '—') : '不可用 · ' + (M.lk.reason || M.lk.unavailable_reason || '原因未知')) : '未配置';
    var rows = [['experiment_id', M.experiment_id], ['flight / method', (M.flight || '') + ' / ' + (M.method || '')], ['status', M.status], ['valid window', 't0=' + M.t0 + 's  t1=' + M.t1 + 's'], ['alignment', (M.alignment || '') + '（course ' + (M.course_window_s || '') + 's）'], ['reference velocity', M.velocity_source + (fcFallback ? '（⚠ fallback=位置差分）' : '')], ['vio velocity', M.vio_velocity_source], ['LK-only', lkStatus], ['out_folder', M.out_folder]];
    rows.forEach(function (kv) { dl.appendChild(el('dt', null, kv[0])); dl.appendChild(el('dd', null, kv[1] == null ? '—' : String(kv[1]))); });
    var body = el('div', 'pad'); body.appendChild(dl); wrap.appendChild(body); return wrap;
  }

  // ── controls (global bar, without errMode/velMode) ────────────────────────
  function selectSeg(id) { st.sel = (st.sel === id ? null : id); rerender(); }
  function controls() {
    var bar = el('div', 'ctrls');
    function chip(k, label, key) { var c = el('span', 'chip' + (st[key] ? '' : ' off')); var sw = el('span', 'sw'); sw.style.background = k === 'gps' ? C.gps : k === 'vio' ? C.vio : k === 'lk' ? C.lk : C.sel; c.appendChild(sw); c.appendChild(document.createTextNode(label)); c.onclick = function () { st[key] = !st[key]; rerender(); }; return c; }
    function seg(label, key, opts) { var g = el('div', 'cg'); g.appendChild(el('span', 'glbl', label)); var s = el('div', 'segctl'); opts.forEach(function (o) { var b = el('button', st[key] === o[0] ? 'on' : '', o[1]); b.onclick = function () { st[key] = o[0]; rerender(); }; s.appendChild(b); }); g.appendChild(s); return g; }
    var g1 = el('div', 'cg'); g1.appendChild(el('span', 'glbl', '轨迹')); g1.appendChild(chip('gps', 'GPS', 'gps')); g1.appendChild(chip('vio', 'VIO', 'vio')); if (lkOn) g1.appendChild(chip('lk', 'LK-only', 'lk')); g1.appendChild(chip('seg', '分段叠加', 'seg')); bar.appendChild(g1);
    bar.appendChild(seg('横轴', 'axis', [['dist', '里程'], ['t', '时间']]));
    bar.appendChild(seg('样本', 'showInvalid', [[false, '仅有效'], [true, '含无效+缺口']]));
    bar.appendChild(seg('范围', 'scope', [['full', '全程'], ['seg', '选中段']]));
    return bar;
  }

  // ── cards ──────────────────────────────────────────────────────────────────
  function cards() {
    var wrap = el('div', 'cards');
    var defs = [['总里程', SUM.gps_distance_km, 'km', 'okc'], ['飞行时长', SUM.duration_s, 's', ''], ['终点 XY 误差', SUM.final_xy_error_m, 'm', 'accent'], ['终点 XY 漂移', SUM.final_xy_drift_percent, '%', 'accent'], ['XY RMSE', SUM.xy_rmse_m, 'm', ''], ['速度 RMSE', SUM.speed_rmse_mps, 'm/s', ''], ['航向 RMSE', SUM.yaw_or_course_rmse_deg, '°', ''], ['有效样本', (Q.valid_aligned_count != null ? Q.valid_aligned_count : '—'), '/' + (Q.gps_sample_count || '—'), ''], ['VIO延迟 p95', Q.p95_delay_s, 's', Q.invalid_delay_count ? 'warnc' : '']];
    defs.forEach(function (d) { var c = el('div', 'card ' + d[3]); c.innerHTML = '<div class="k">' + d[0] + '</div><div class="v">' + (d[1] == null ? '—' : d[1]) + ' <span class="u">' + d[2] + '</span></div>'; wrap.appendChild(c); });
    return wrap;
  }
  function rowGrid(cols, items) { var g = el('div', 'rowgrid'); g.style.gridTemplateColumns = cols; items.forEach(function (i) { g.appendChild(i); }); return g; }

  // ── main render ────────────────────────────────────────────────────────────
  var root;
  function rerender() {
    bus = [];
    var body = el('div', 'dashbody');
    body.appendChild(controls());
    var syn = el('div', 'syn2', '<b>OFFLINE</b><span>前端不重算指标，数值来自本次分析输出。位置误差面板：航迹系=全局投影至瞬时 GPS 航向，段局部系详见段明细。' + (fcFallback ? ' ⚠ 参考速度=位置差分 fallback。' : '') + '</span>');
    body.appendChild(syn);
    body.appendChild(cards());
    // Row 1: trajectory + merged position error panel
    var playBtn = el('button', 'playbtn', '▶ 轨迹动画');
    var tlabel = el('span', 'tlabel', '随时间播放 →');
    var traj = trajChart({ height: 360, onPlay: function (on) { playBtn.textContent = on ? '⏸ 暂停' : '▶ 轨迹动画'; playBtn.classList.toggle('on', on); }, onFrame: function (p) { tlabel.textContent = 't=' + p.t.toFixed(0) + 's · ' + (p.cum_dist / 1000).toFixed(2) + 'km'; } });
    playBtn.onclick = function () { traj.play(); };
    var trajLeg = el('div', 'lgrow');
    [['GPS（飞控参考）', C.gps, 0], ['VIO', C.vio, 0]].concat(lkOn ? [['LK-only', C.lk, 1]] : []).forEach(function (x) { var i = el('span', 'lgi'); var ln = el('span', 'lgln'); ln.style.borderTopColor = x[1]; if (x[2]) ln.style.borderTopStyle = 'dashed'; i.appendChild(ln); i.appendChild(document.createTextNode(x[0])); trajLeg.appendChild(i); });
    trajLeg.appendChild(el('span', 'lgi muted', '○GPS起 ·VIO起  ■GPS末·VIO末 · 点击段高亮双轨'));
    trajLeg.appendChild(tlabel);
    body.appendChild(rowGrid('1.35fr 1fr', [panel('XY 轨迹对比 + 动画', 'E–N / m', '悬停显示 GPS + VIO 双游标，点击段同时高亮 GPS（紫实）和 VIO（紫虚）轨迹。', traj.el, { legend: trajLeg, badge: playBtn }), posErrPanel()]));
    // Row 2: velocity (3-comp) + course error
    body.appendChild(rowGrid('1fr 1fr', [velPanel(), chCourse()]));
    // Row 3: velocity error (3-comp) + drift bars
    body.appendChild(rowGrid('1fr 1.2fr', [velErrPanel(), panel('各段局部漂移率', '% (local, 段局部坐标系)', '段漂移率 = 局部终点 XY误差 / 里程 × 100。段局部系：GPS 净位移方向为沿轴，VIO 起点对齐 GPS 起点。', barChart('drift').el)]));
    // Row 4: diagnostics + GPS sampling
    body.appendChild(rowGrid('1fr 1fr', [diagnosticsPanel(), panel('各段漂移 vs 段长诊断', 'm', '漂移条形（参考）。蓝=直线 橙=转弯 灰=残段。', barChart('speed').el, { badge: badge((Q.invalid_delay_count || 0) + ' 延迟 · ' + (Q.gps_gap_count || 0) + ' 缺口', 'warn') })]));
    // Row 5: segment browser + detail
    body.appendChild(rowGrid('1fr 1.2fr', [segBrowser(), segDetail()]));
    // Row 6: provenance
    body.appendChild(rowGrid('1fr', [provenance()]));
    if (root) root.replaceChildren(body); else { root = el('div'); document.body.appendChild(root); root.appendChild(body); }
  }

  function mount() { root = document.getElementById('app'); if (!root) { root = el('div'); root.id = 'app'; document.body.appendChild(root); } rerender(); }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
})();
