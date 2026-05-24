#!/usr/bin/env python3
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""Plot full-time-span altitude per flight from EVERY available altitude source
(raw GPS, aligned GPS, truth_asl, stereo VIO reference) on a common time axis."""
import os, sys, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# truth_asl_*.csv is stereo VIO pseudo-reference, not ground truth.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _pseudo_ref_guard import require_pseudo_ref_ack
require_pseudo_ref_ack(__file__)

def load_csv_xy(p, t_col, val_col, sep=None):
    rows=[]
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split(sep) if sep else ln.replace(',', ' ').split()
            try:
                t = float(ps[t_col]); v = float(ps[val_col])
                rows.append((t, v))
            except: continue
    a = np.asarray(rows)
    if a.size and a[0,0] > 1e11: a[:,0] = a[:,0]*1e-9   # ns->s
    return a

def load_stereo_z(p):
    """timestamp(s) tx ty tz qx qy qz qw ..."""
    rows=[]
    with open(p) as f:
        for ln in f:
            ln=ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split()
            try:
                t=float(ps[0]); z=float(ps[3])
                rows.append((t,z))
            except: continue
    return np.asarray(rows)

# common t0 per flight = first sample of aligned_gps_cam_time (covers full flight)
FLIGHTS = [(1,'one'), (2,'two'), (3,'three'), (4,'four')]
os.makedirs('comparison_plots/baselines_v1/data_audit', exist_ok=True)
for fly, ref_name in FLIGHTS:
    root = f'20260509_fly{fly}'
    gps_cam = f'{root}/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'
    truth_cam = f'{root}/result/gps_tum_time_alignment/truth_asl_cam_time.csv'
    raw_gps = f'{root}/result/raw_gps_altitude_fly{fly}.csv'
    stereo = f'{root}/reference/{ref_name}_traj_estimate_stereo.txt'

    a_gps = load_csv_xy(gps_cam, 0, 3) if os.path.isfile(gps_cam) else None
    a_truth = load_csv_xy(truth_cam, 0, 3) if os.path.isfile(truth_cam) else None
    a_raw = load_csv_xy(raw_gps, 0, 1) if os.path.isfile(raw_gps) else None  # raw has (t_us_rel, alt)
    a_stereo = load_stereo_z(stereo) if os.path.isfile(stereo) else None

    # Anchor time axis at aligned_gps first sample
    if a_gps is None or a_gps.size == 0:
        print(f'fly{fly}: no aligned GPS, skipping'); continue
    t_anchor = a_gps[0,0]

    fig, ax = plt.subplots(1,1,figsize=(13,6))
    if a_gps is not None and a_gps.size:
        ax.plot(a_gps[:,0]-t_anchor, a_gps[:,1], 'b-', lw=1.0,
                label=f'aligned_gps_cam_time (alt, GPS-WGS84)  n={len(a_gps)}  dur={a_gps[-1,0]-a_gps[0,0]:.1f}s  rng=[{a_gps[:,1].min():.1f},{a_gps[:,1].max():.1f}]m')
    if a_stereo is not None and a_stereo.size:
        ax.plot(a_stereo[:,0]-t_anchor, a_stereo[:,1], 'g--', lw=1.0,
                label=f'stereo VIO reference (tz)  n={len(a_stereo)}  dur={a_stereo[-1,0]-a_stereo[0,0]:.1f}s  rng=[{a_stereo[:,1].min():.1f},{a_stereo[:,1].max():.1f}]m')
    if a_truth is not None and a_truth.size:
        ax.plot(a_truth[:,0]-t_anchor, a_truth[:,1], 'r-', lw=2.0,
                label=f'truth_asl_cam_time (p_z)  n={len(a_truth)}  dur={a_truth[-1,0]-a_truth[0,0]:.1f}s  rng=[{a_truth[:,1].min():.1f},{a_truth[:,1].max():.1f}]m')
    ax.set_xlabel('time since aligned_gps t0 (s)')
    ax.set_ylabel('altitude / z (m)')
    ax.set_title(f'fly{fly}: altitude across all candidate truth sources')
    ax.grid(alpha=0.3); ax.legend(loc='best', fontsize=8)
    p=f'comparison_plots/baselines_v1/data_audit/fly{fly}_altitude_full_span.png'
    fig.tight_layout(); fig.savefig(p, dpi=120); plt.close(fig)
    print(f'[saved] {p}')

    # Print time intervals where the truth differs from aligned GPS
    if a_truth is not None and a_truth.size and a_gps is not None and a_gps.size:
        truth_span = (a_truth[0,0]-t_anchor, a_truth[-1,0]-t_anchor)
        gps_span = (a_gps[0,0]-t_anchor, a_gps[-1,0]-t_anchor)
        stereo_span = (a_stereo[0,0]-t_anchor, a_stereo[-1,0]-t_anchor) if a_stereo is not None else (None,None)
        print(f'  spans (rel to aligned_gps t0):  truth=[{truth_span[0]:.1f}, {truth_span[1]:.1f}]  '
              f'gps=[{gps_span[0]:.1f}, {gps_span[1]:.1f}]  stereo=[{stereo_span[0]:.1f}, {stereo_span[1]:.1f}]')
