#!/usr/bin/env python3
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""Step 3 — Timing audit per flight.

Print for each flight (parsing the v1_E1 log + B1 log + truth):
  - VIO init time (first trajectory sample timestamp, vs --start-time)
  - First GPS altitude measurement time (first GPLANE-RNG line)
  - z_ground bootstrap time (first time z_ground != 0 OR first GPLANE-RNG status=FULL)
  - First Stage A update (first GPLANE-RNG status=FULL)
  - First Stage B v1 update (first [GPLANE-V1] t= line)
  - First-edge start/end (from Step 2)
  - descent_start

All times reported relative to truth_t0.
"""
import math, os, re, sys
import numpy as np
# truth_asl_*.csv is stereo VIO pseudo-reference, not ground truth.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _pseudo_ref_guard import require_pseudo_ref_ack
require_pseudo_ref_ack(__file__)

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

def first_traj_t(p):
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps=ln.split()
            try: return float(ps[0])
            except: continue
    return None

def first_log_match(log_path, regex):
    pat = re.compile(regex)
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln_clean = re.sub(r'\x1b\[[0-9;]*m','',ln)
            m = pat.search(ln_clean)
            if m: return ln_clean.rstrip(), m
    return None, None

def first_gplane_rng_t(log_path, require_full=False):
    """Return first [GPLANE-RNG] timestamp (and status), optionally only status=FULL."""
    pat = re.compile(r'\[GPLANE-RNG\] status=(\w+) t=([\d.]+)')
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln_clean = re.sub(r'\x1b\[[0-9;]*m','',ln)
            m = pat.search(ln_clean)
            if not m: continue
            status, t = m.group(1), float(m.group(2))
            if require_full and status != 'FULL': continue
            return t, status, ln_clean.rstrip()
    return None, None, None

def first_gplane_v1_t(log_path):
    """First [GPLANE-V1] t=... line."""
    pat = re.compile(r'\[GPLANE-V1\] t=([\d.]+)')
    with open(log_path, errors='replace') as f:
        for ln in f:
            ln_clean = re.sub(r'\x1b\[[0-9;]*m','',ln)
            m = pat.search(ln_clean)
            if m: return float(m.group(1)), ln_clean.rstrip()
    return None, None

FLIGHTS = [
    (1, 200,
     '20260509_fly1/result/baselines_v1/B1_gps_height_d10.log',
     '20260509_fly1/result/baselines_v1/v1_E1_fly1.log',
     '20260509_fly1/result/baselines_v1/v1_E1_fly1.txt',
     (20.0, 120.0)),
    (2, 160,
     '20260509_fly2/result/baselines_v1/B1_gps_height_d10.log',
     '20260509_fly2/result/baselines_v1/v1_E1_fly2.log',
     '20260509_fly2/result/baselines_v1/v1_E1_fly2.txt',
     (5.0, 105.0)),
    (3, 180,
     '20260509_fly3/result/baselines_v1/B1_gps_height_d10.log',
     '20260509_fly3/result/baselines_v1/v1_E1_fly3.log',
     '20260509_fly3/result/baselines_v1/v1_E1_fly3.txt',
     (5.0, 105.0)),
    (4, 239,
     '20260509_fly4/result/stage_a_v2/R6_delay10s.log',
     '20260509_fly4/result/baselines_v1/v1_E1_s50_K2_exfalse_fly4.log',
     '20260509_fly4/result/baselines_v1/v1_E1_s50_K2_exfalse_fly4.txt',
     (48.2, 116.4)),
]

for fly, start, b1_log, v1_log, v1_traj, edge in FLIGHTS:
    truth_path = f'20260509_fly{fly}/result/gps_tum_time_alignment/truth_asl_cam_time.csv'
    truth = load_truth(truth_path); t0=truth[0,0]
    ds = detect_descent(truth); ds_rel = (ds-t0) if ds else None
    print(f'\n===== fly{fly}  start={start}s  t_truth0={t0:.3f}  descent_rel={ds_rel:.2f}s =====')
    if not os.path.isfile(v1_log):
        print(f'  v1 log missing: {v1_log}'); continue
    # 1) Init = first trajectory sample
    t_init = first_traj_t(v1_traj)
    print(f'  VIO_init                : t={t_init:.3f} (rel {t_init-t0:+.2f}s)' if t_init else '  VIO_init: unknown')
    # 2) First GPS-ALT delay note
    ln, m = first_log_match(v1_log, r'gps[_ ]alt[_ ]bootstrap[_ ]delay = ([\d.]+)s')
    delay = float(m.group(1)) if m else None
    print(f'  Stage_A_min_t_after_init: {delay} s')
    # 3) First GPS-RNG (any status, marks first GPS sample reached)
    t_any, status_any, ln_any = first_gplane_rng_t(v1_log, require_full=False)
    if t_any:
        print(f'  first_GPS_RNG_call      : t={t_any:.3f} (rel {t_any-t0:+.2f}s) status={status_any}')
    else:
        print('  first_GPS_RNG_call      : none')
    # 4) First GPS-RNG status=FULL = first Stage A update accepted
    t_full, status_full, ln_full = first_gplane_rng_t(v1_log, require_full=True)
    if t_full:
        print(f'  first_StageA_FULL       : t={t_full:.3f} (rel {t_full-t0:+.2f}s)')
        # extract z_ground from FULL line if present
        m = re.search(r'z_ground=([\-\d.]+)', ln_full)
        if m: print(f'     z_ground_at_first_FULL: {float(m.group(1)):.3f}')
    else:
        print('  first_StageA_FULL       : never')
    # 5) z_ground bootstrap: scan for first time z_ground != 0.0
    pat_zg = re.compile(r'z_ground=([\-\d.]+)')
    t_zg = None
    with open(v1_log, errors='replace') as f:
        for ln in f:
            ln_clean = re.sub(r'\x1b\[[0-9;]*m','',ln)
            mz = pat_zg.search(ln_clean)
            if mz and abs(float(mz.group(1)))>1e-9:
                mt = re.search(r't=([\d.]+)', ln_clean)
                if mt: t_zg = float(mt.group(1)); break
    if t_zg:
        print(f'  z_ground_bootstrap_nonzero: t={t_zg:.3f} (rel {t_zg-t0:+.2f}s)')
    else:
        print('  z_ground stays 0 throughout (or no [GPLANE-RNG] activity)')
    # 6) First v1 update
    t_v1, ln_v1 = first_gplane_v1_t(v1_log)
    if t_v1:
        print(f'  first_StageB_v1_update  : t={t_v1:.3f} (rel {t_v1-t0:+.2f}s)')
    else:
        print('  first_StageB_v1_update  : none')
    # 7) First-edge window
    print(f'  first_edge_window_rel   : [{edge[0]:.2f}, {edge[1]:.2f}]s')
    # 8) Derived: time between init and first edge start
    if t_init:
        gap_init_to_edge = edge[0] - (t_init - t0)
        print(f'  edge_starts_relative_to_init: t_edge_start - t_init = {gap_init_to_edge:+.2f}s')
    if t_full and t_init:
        print(f'  StageA_first_FULL  rel to init : {t_full - t_init:+.2f}s')
    if t_v1 and t_init:
        print(f'  v1_first_update    rel to init : {t_v1   - t_init:+.2f}s')
