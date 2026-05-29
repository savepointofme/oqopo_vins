#!/usr/bin/env python3
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""Step 2 — Per-flight early-scale diagnostic.

For each flight:
 1. Scan truth XY for the first window after init where the drone moves in a
    near-straight line (path_length / chord >= ~1.05) and net displacement
    >= 30 m.  This is the "first clean straight segment".
 2. For B0 / B1 / v1_E1 (and R5b/R7 when available for fly4), compute:
    - truth_segment_length
    - estimated_segment_length (after 5s SE3 translation alignment)
    - scale_ratio = est / truth
    - first-edge XY RMSE
    - first-edge final XY error
 3. Save a per-flight first-edge plot.
"""
import math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# truth_asl_*.csv is stereo VIO pseudo-reference, not ground truth.
# Require explicit acknowledgement before running.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _pseudo_ref_guard import require_pseudo_ref_ack
require_pseudo_ref_ack(__file__)

def load_traj(p):
    rows=[]; lt=-math.inf
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps=ln.split()
            try: t=float(ps[0]); x=float(ps[1]); y=float(ps[2]); z=float(ps[3])
            except: continue
            if t<=lt: continue
            rows.append((t,x,y,z)); lt=t
    return np.asarray(rows) if rows else np.zeros((0,4))

def load_truth(p):
    rows=[]
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps=ln.split(',')
            try: rows.append((float(ps[0])*1e-9,float(ps[1]),float(ps[2]),float(ps[3])))
            except: continue
    a=np.asarray(rows); return a[np.argsort(a[:,0])]

def detect_descent(t):
    z=t[:,3]; dur=t[-1,0]-t[0,0]
    if dur<=0: return None
    m=(t[:,0]>=t[0,0]+0.10*dur)&(t[:,0]<=t[0,0]+0.90*dur)
    if m.sum()<5: return None
    zmax=float(z[m].max()); zend=float(z[-1])
    if zmax-zend<1.0: return None
    thresh=zmax-0.5*(zmax-zend)
    ipk=int(np.argmax(z*m.astype(float)))
    for i in range(ipk+1,len(z)):
        if z[i]>=thresh: continue
        if np.all(z[i:]<thresh): return float(t[i,0])
    return None

def path_len_xy(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))) if len(xy)>=2 else 0.0

def chord_xy(xy):
    return float(np.linalg.norm(xy[-1]-xy[0])) if len(xy)>=2 else 0.0

def detect_first_edge(truth, t0, descent_rel, min_len_m=30.0, min_straightness=1.05, min_dur=20.0, max_dur=120.0):
    """Walk along the truth post-init, find a candidate window where
       path_length / chord <= min_straightness AND chord >= min_len_m."""
    t = truth[:, 0] - t0
    xy = truth[:, 1:3]
    # restrict to pre-descent
    end_rel = descent_rel if descent_rel else t[-1]
    # Sample step: 5s
    starts = np.arange(5.0, max(end_rel-min_dur, 5.0), 5.0)
    for s in starts:
        for win_dur in [60.0, 80.0, 100.0]:
            e = s + win_dur
            if e > end_rel - 5: break
            m = (t >= s) & (t <= e)
            if m.sum() < 10: continue
            sub = xy[m]
            pl = path_len_xy(sub); ch = chord_xy(sub)
            if ch < min_len_m: continue
            straightness = pl / ch if ch>0 else 9.0
            if straightness <= min_straightness:
                return (s, e, pl, ch, straightness)
    return None

def translation_offset(vio, truth, window_sec=5.0):
    t0 = vio[0,0]; m=(vio[:,0]-t0)<window_sec
    sub = vio[m] if m.sum()>=5 else vio[:5]
    out=[]
    for k in range(3):
        ta = np.interp(sub[:,0], truth[:,0], truth[:,1+k], left=np.nan, right=np.nan)
        d = ta - sub[:,1+k]; v = ~np.isnan(d); out.append(d[v].mean() if v.any() else 0.0)
    return np.asarray(out)

def first_edge_metrics(vio, truth, t0, edge_start_rel, edge_end_rel, edge_truth_xy, edge_truth_len, align_window=5.0):
    if vio.shape[0]<5: return None
    off = translation_offset(vio, truth, align_window)
    aligned = vio[:,1:4] + off
    in_edge = (vio[:,0]>=t0+edge_start_rel) & (vio[:,0]<=t0+edge_end_rel)
    if in_edge.sum()<5: return None
    est_xy = aligned[in_edge, :2]
    est_len = path_len_xy(est_xy)
    ratio = est_len / edge_truth_len if edge_truth_len>0 else float('nan')
    # XY RMSE in edge
    tr = np.zeros((in_edge.sum(), 2))
    for k in range(2):
        tr[:,k] = np.interp(vio[in_edge,0], truth[:,0], truth[:,1+k])
    err_xy = np.sqrt((est_xy[:,0]-tr[:,0])**2 + (est_xy[:,1]-tr[:,1])**2)
    rmse = float(np.sqrt(np.mean(err_xy**2)))
    final = float(err_xy[-1])
    return dict(off=off, aligned=aligned, est_len=est_len, ratio=ratio, edge_xy_rmse=rmse, edge_xy_final=final)

FLIGHTS = [
    (1, 200,
     [('B0',    '20260509_fly1/result/baselines_v1/B0_no_gps.txt'),
      ('B1',    '20260509_fly1/result/baselines_v1/B1_gps_height_d10.txt'),
      ('v1_E1', '20260509_fly1/result/baselines_v1/v1_E1_fly1.txt')]),
    (2, 160,
     [('B0',    '20260509_fly2/result/baselines_v1/B0_no_gps.txt'),
      ('B1',    '20260509_fly2/result/baselines_v1/B1_gps_height_d10.txt'),
      ('v1_E1', '20260509_fly2/result/baselines_v1/v1_E1_fly2.txt')]),
    (3, 180,
     [('B0',    '20260509_fly3/result/baselines_v1/B0_no_gps.txt'),
      ('B1',    '20260509_fly3/result/baselines_v1/B1_gps_height_d10.txt'),
      ('v1_E1', '20260509_fly3/result/baselines_v1/v1_E1_fly3.txt')]),
    (4, 239,
     [('B0',    '20260509_fly4/result/stage_a_v2/R0_nogps.txt'),
      ('B1',    '20260509_fly4/result/stage_a_v2/R6_delay10s.txt'),
      ('R5b',   '20260509_fly4/result/stage_a_v2/R5b_extreme_K2.txt'),
      ('R7',    '20260509_fly4/result/stage_a_v2/R7_d10_plus_R5b.txt'),
      ('v1_E1', '20260509_fly4/result/baselines_v1/v1_E1_s50_K2_exfalse_fly4.txt')]),
]

os.makedirs('comparison_plots/baselines_v1/early_scale_diag', exist_ok=True)

print(f'{"flight":>6s}  {"start":>5s}  {"edge_start":>10s}  {"edge_end":>8s}  {"win":>4s}  {"truth_len":>9s}  {"chord":>6s}  {"straight":>8s}')
print('-'*80)
edges = {}
for fly, st, runs in FLIGHTS:
    truth = load_truth(f'20260509_fly{fly}/result/gps_tum_time_alignment/truth_asl_cam_time.csv')
    t0 = truth[0,0]; ds = detect_descent(truth); ds_rel = ds-t0 if ds else None
    if fly == 4:
        # use the previously-pinned window
        e = (48.2, 116.4)
        m=(truth[:,0]>=t0+e[0])&(truth[:,0]<=t0+e[1]); sub=truth[m,1:3]
        edges[fly] = (e[0], e[1], path_len_xy(sub), chord_xy(sub), path_len_xy(sub)/max(chord_xy(sub),1e-9))
    else:
        de = detect_first_edge(truth, t0, ds_rel)
        edges[fly] = de
    s,e,pl,ch,sg = edges[fly] if edges[fly] else (None,)*5
    print(f'{fly:>6d}  {st:>5d}  {s:>10.2f}  {e:>8.2f}  {(e-s):>4.0f}  {pl:>9.2f}  {ch:>6.2f}  {sg:>8.3f}')
print()
print(f'{"flight":>6s}  {"run":>8s}  {"edge_len":>9s}  {"truth_len":>9s}  {"ratio":>7s}  {"edge_RMSE":>10s}  {"edge_final":>10s}')
print('-'*80)
for fly, st, runs in FLIGHTS:
    truth = load_truth(f'20260509_fly{fly}/result/gps_tum_time_alignment/truth_asl_cam_time.csv')
    t0 = truth[0,0]
    e_start, e_end, truth_len, chord, sg = edges[fly]
    m=(truth[:,0]>=t0+e_start)&(truth[:,0]<=t0+e_end); edge_truth_xy = truth[m,1:3]
    # plot
    fig, ax = plt.subplots(1,1,figsize=(8,8))
    ax.plot(edge_truth_xy[:,0], edge_truth_xy[:,1], 'k:', lw=2, label=f'truth ({truth_len:.1f}m, str={sg:.2f})')
    for label, path in runs:
        if not os.path.isfile(path): continue
        v = load_traj(path)
        m_v = first_edge_metrics(v, truth, t0, e_start, e_end, edge_truth_xy, truth_len)
        if m_v is None:
            print(f'  {fly:>6d}  {label:>8s}: no edge data'); continue
        print(f'  {fly:>6d}  {label:>8s}  {m_v["est_len"]:>9.2f}  {truth_len:>9.2f}  {m_v["ratio"]:>7.3f}  {m_v["edge_xy_rmse"]:>10.2f}  {m_v["edge_xy_final"]:>10.2f}')
        in_edge=(v[:,0]>=t0+e_start)&(v[:,0]<=t0+e_end)
        ax.plot(m_v['aligned'][in_edge,0], m_v['aligned'][in_edge,1], '-', lw=1.4,
                label=f'{label} (len={m_v["est_len"]:.1f}, r={m_v["ratio"]:.3f}, rmse={m_v["edge_xy_rmse"]:.1f})')
    ax.set_aspect('equal','datalim'); ax.grid(alpha=0.3)
    ax.set_title(f'fly{fly} first-edge zoom  [{e_start:.1f},{e_end:.1f}]s rel'); ax.legend(loc='best', fontsize=9)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)')
    p=f'comparison_plots/baselines_v1/early_scale_diag/fly{fly}_first_edge.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'  [saved] {p}')
    print()
