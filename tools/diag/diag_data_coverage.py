#!/usr/bin/env python3
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""Data coverage audit — find where the truth/GPS/trajectory got clipped.

For each flight, scan every candidate data file:
  - raw GPS (any *.csv under flyN/result/ matching gps/altitude)
  - aligned GPS at cam time
  - aligned GPS at imu time
  - truth_asl at cam time / imu time
  - all trajectory outputs
  - mav0/{imu0,cam0}/data.csv

Print first/last timestamp, duration, row count, alt min/max where applicable.
Also save a comparison plot per flight showing altitude vs time across files.
"""
import os, sys, glob, math, re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# truth_asl_*.csv is stereo VIO pseudo-reference, not ground truth.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _pseudo_ref_guard import require_pseudo_ref_ack
require_pseudo_ref_ack(__file__)

def safe_first_last(p):
    rows=[]
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try:
                t=float(ps[0])
                rows.append(t)
            except:
                continue
    if not rows: return None, None, 0
    return rows[0], rows[-1], len(rows)

def detect_unit(t0):
    """Guess timestamp unit: seconds if t0 < 1e11, else nanoseconds."""
    return 'ns' if t0 > 1e11 else 's'

def load_alt(p, col_alt=3):
    """Load a CSV/TXT and try to extract (t_s, alt) pairs.
    col_alt is 1-indexed for some formats, here we treat 3 = 4th column (z)."""
    rows=[]
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            if len(ps) <= col_alt: continue
            try:
                t=float(ps[0])
                a=float(ps[col_alt])
                rows.append((t, a))
            except:
                continue
    if not rows: return None
    arr = np.asarray(rows)
    # auto-rescale ns->s
    if arr[0,0] > 1e11:
        arr[:,0] = arr[:,0] * 1e-9
    return arr

FLIGHTS = [1, 2, 3, 4]
os.makedirs('comparison_plots/baselines_v1/data_audit', exist_ok=True)

for fly in FLIGHTS:
    root = f'20260509_fly{fly}'
    print(f'\n=================== fly{fly} ===================')
    candidates = []
    # mav0
    for p in [f'{root}/mav0/imu0/data.csv', f'{root}/mav0/cam0/data.csv']:
        if os.path.isfile(p): candidates.append(('mav0', p))
    # result-dir files
    for p in glob.glob(f'{root}/result/*.csv') + \
             glob.glob(f'{root}/result/gps_tum_time_alignment/*.csv') + \
             glob.glob(f'{root}/result/gps_tum_time_alignment/*.txt') + \
             glob.glob(f'{root}/result/baselines_v1/*.txt'):
        if os.path.isfile(p):
            candidates.append(('result', p))
    # fly4 also has stage_a_v2
    for p in glob.glob(f'{root}/result/stage_a_v2/*.txt'):
        if os.path.isfile(p) and not p.endswith('.bias'):
            candidates.append(('stage_a', p))

    print(f'{"src":>10s} {"first_t":>20s} {"last_t":>20s} {"unit":>4s} {"dur_s":>8s} {"rows":>7s}  path')
    print('-'*120)
    for src, p in candidates:
        t0,t1,n = safe_first_last(p)
        if t0 is None: continue
        unit = detect_unit(t0)
        if unit == 'ns':
            dur = (t1-t0)*1e-9
            t0_disp = t0; t1_disp = t1
        else:
            dur = t1-t0; t0_disp = t0; t1_disp = t1
        rel_path = p.replace('\\','/')
        print(f'{src:>10s} {t0_disp:>20.3f} {t1_disp:>20.3f} {unit:>4s} {dur:>8.1f} {n:>7d}  {rel_path}')

    # Altitude comparison plot — for files we know have (t, ..., z)
    fig, ax = plt.subplots(1,1,figsize=(13,6))
    plotted=0
    altitude_sources = []
    # truth_asl_cam_time.csv: col layout from gps_tum_time_alignment (ts_ns, x, y, z, ...)
    for src_label, path, alt_col in [
        ('truth_asl_cam_time',
         f'{root}/result/gps_tum_time_alignment/truth_asl_cam_time.csv', 3),
        ('truth_asl_imu_time',
         f'{root}/result/gps_tum_time_alignment/truth_asl_imu_time.csv', 3),
        ('aligned_gps_cam_time',
         f'{root}/result/gps_tum_time_alignment/aligned_gps_cam_time.csv', 3),
        ('aligned_gps_imu_time',
         f'{root}/result/gps_tum_time_alignment/aligned_gps_imu_time.csv', 3),
        ('aligned_gps_altitude_flyN',
         f'{root}/result/aligned_gps_altitude_fly{fly}.csv', 3),
    ]:
        if not os.path.isfile(path): continue
        arr = load_alt(path, col_alt=alt_col)
        if arr is None: continue
        # rel to first truth_asl_cam_time t0 if available
        ax.plot(arr[:,0]-arr[0,0], arr[:,1], lw=1.0, label=f'{src_label} (n={len(arr)}, dur={arr[-1,0]-arr[0,0]:.1f}s, alt[{arr[:,1].min():.1f},{arr[:,1].max():.1f}])')
        altitude_sources.append((src_label, arr))
        plotted+=1
    if plotted:
        ax.set_xlabel('time since file_t0 (s)'); ax.set_ylabel('altitude / z (m)')
        ax.set_title(f'fly{fly}: altitude/Z coverage across candidate sources'); ax.grid(alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        p=f'comparison_plots/baselines_v1/data_audit/fly{fly}_altitude_sources.png'
        fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
        print(f'  [saved] {p}')

    # Per-source altitude stats summary
    print(f'\nAltitude stats per source:')
    for src_label, arr in altitude_sources:
        alts = arr[:,1]
        # detect altitude plateaus: simple binning by 10m
        hist, edges = np.histogram(alts, bins=np.arange(-5, max(alts.max()+10, 110), 10))
        # which bins have > 5% of samples?
        top = np.argsort(hist)[::-1][:5]
        bins_str = ', '.join(f'[{edges[i]:.0f}-{edges[i+1]:.0f}]m:{hist[i]}' for i in top if hist[i]>0)
        print(f'  {src_label:>22s}  range=[{alts.min():.1f},{alts.max():.1f}]  median={np.median(alts):.1f}  duration={arr[-1,0]-arr[0,0]:.1f}s  populated_bins(top5)={bins_str}')
