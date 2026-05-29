#!/usr/bin/env python3
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""Step 1 — Run completeness + eval-window audit.

For each flight, print B0/B1/v1_E1:
  - traj first/last timestamps and t_rel relative to truth t0
  - number of output states
  - requested start-time
  - actual init time (first traj sample wallclock)
  - eval_start (= max of trajectory first across compared runs)
  - detected descent_start / eval_end
  - trajectory_end
  - whether the run covers the full eval window
  - common overlap range
"""
from __future__ import annotations
import math
import numpy as np
import os, sys
# truth_asl_*.csv is stereo VIO pseudo-reference, not ground truth.
# Require explicit acknowledgement before running. See
# GPS_REFERENCE_AUDIT.md and tools/_pseudo_ref_guard.py.
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

FLIGHTS = [
    (1, 200,
     '20260509_fly1/result/baselines_v1/B0_no_gps.txt',
     '20260509_fly1/result/baselines_v1/B1_gps_height_d10.txt',
     '20260509_fly1/result/baselines_v1/v1_E1_fly1.txt'),
    (2, 160,
     '20260509_fly2/result/baselines_v1/B0_no_gps.txt',
     '20260509_fly2/result/baselines_v1/B1_gps_height_d10.txt',
     '20260509_fly2/result/baselines_v1/v1_E1_fly2.txt'),
    (3, 180,
     '20260509_fly3/result/baselines_v1/B0_no_gps.txt',
     '20260509_fly3/result/baselines_v1/B1_gps_height_d10.txt',
     '20260509_fly3/result/baselines_v1/v1_E1_fly3.txt'),
    (4, 239,
     '20260509_fly4/result/stage_a_v2/R0_nogps.txt',
     '20260509_fly4/result/stage_a_v2/R6_delay10s.txt',
     '20260509_fly4/result/baselines_v1/v1_E1_s50_K2_exfalse_fly4.txt'),
]

for fly, start_time, p_B0, p_B1, p_v1 in FLIGHTS:
    truth_path = f'20260509_fly{fly}/result/gps_tum_time_alignment/truth_asl_cam_time.csv'
    truth = load_truth(truth_path)
    if truth.shape[0] < 10:
        print(f'\n===== fly{fly}: truth missing =====')
        continue
    t0 = truth[0,0]
    t_truth_end_abs = truth[-1,0]
    t_truth_end_rel = t_truth_end_abs - t0
    ds_abs = detect_descent(truth)
    ds_rel = (ds_abs-t0) if ds_abs is not None else None
    print(f'\n===== fly{fly}  start={start_time}s  truth_end_rel={t_truth_end_rel:.2f}s  descent_start_rel={ds_rel:.2f}s =====')
    print(f'  truth_csv: {truth_path}')
    rows=[]
    for label, path in [('B0',p_B0),('B1',p_B1),('v1_E1',p_v1)]:
        if not os.path.isfile(path):
            print(f'  {label:>8s}: MISSING {path}')
            continue
        v = load_traj(path)
        if v.shape[0]<2:
            print(f'  {label:>8s}: too short ({v.shape[0]} samples)')
            continue
        first_abs = v[0,0]; last_abs = v[-1,0]
        first_rel = first_abs - t0; last_rel = last_abs - t0
        n_states = v.shape[0]
        # Truth-bounded last_rel
        last_rel_in_truth = min(last_rel, t_truth_end_rel)
        # Coverage to descent
        covers_descent = (ds_rel is not None) and (last_rel >= ds_rel)
        rows.append((label, first_abs, first_rel, last_abs, last_rel, n_states, covers_descent))
        print(f'  {label:>8s}: '
              f'first={first_abs:.3f} (rel {first_rel:+.2f}s)  '
              f'last={last_abs:.3f} (rel {last_rel:+.2f}s, truncated_to_truth_rel={last_rel_in_truth:.2f}s)  '
              f'states={n_states}  covers_descent={covers_descent}')
    if rows:
        common_first = max(r[1] for r in rows)
        common_last  = min(r[3] for r in rows)
        print(f'  common_overlap: [{common_first:.3f}, {common_last:.3f}]  '
              f'(rel [{common_first-t0:+.2f}, {common_last-t0:+.2f}]s)  '
              f'span {common_last-common_first:.1f}s')
        eval_end = ds_abs if ds_abs is not None else common_last
        eval_end = min(eval_end, common_last)
        print(f'  eval_window=[{common_first-t0:+.2f}, {eval_end-t0:.2f}]s  '
              f'(descent_used={ds_abs is not None})')
        # truncation-vs-truth concern
        if ds_abs is not None:
            for label, fa, fr, la, lr, n, cov in rows:
                if not cov:
                    print(f'  WARNING: run {label} ends BEFORE descent_start (last_rel={lr:.2f} < descent_rel={ds_rel:.2f})')
