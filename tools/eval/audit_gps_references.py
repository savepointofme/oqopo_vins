#!/usr/bin/env python3
"""GPS reference audit.

Catalog every GPS / GPS-aligned file we have per flight. Print:
  - path
  - first / last timestamp (in seconds, absolute and relative-to-file-t0)
  - duration
  - row count
  - altitude min / max / median
  - whether it appears to cover both the ~50m loop AND the ~90m loop
    (heuristic: max altitude difference vs ground >= 60m AND samples in
    both [40..60]m and [80..100]m bands)
  - timestamp domain: 'cam_time' (aligned), 'imu_time' (aligned), 'raw_GPS_us',
    or 'unknown'
  - role: 'fusion_input' (currently consumed by VioManager GPS-alt fusion),
    'eval_only', or 'unused' (best-effort guess based on filename + currently
    known scripts)

Notes:
  * Stereo VIO outputs (reference/*.txt, truth_asl_*) are NOT GPS and are
    NOT included. They are pseudo-references only and must not be called
    truth.
  * The 'cam_time' files have header `#timestamp_ns,lat_deg,lon_deg,alt_m`.
  * The raw GPS.csv has header
    `TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U`
    where TimeUS is microseconds (autopilot clock, NOT UTC), and Alt is
    WGS84/MSL altitude in meters (column index 9, 0-indexed).
"""
import os, glob, csv
import numpy as np

# Per-flight catalog of files to audit
FLIGHTS = {
    1: {
        'raw_gps':   '20260509_fly1/mav0/gps/GPS.csv',
        'aligned': [
            ('aligned_offset_m4p63',
             '20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv'),
            ('aligned_offset_m4p63_imu',
             '20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_imu_time.csv'),
            ('aligned_gps_raw_horizontal_v5',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_horizontal_v5.csv'),
            ('aligned_gps_raw_best',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_best.csv'),
            ('aligned_gps_raw_propstart',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_propstart.csv'),
            ('aligned_gps_raw_takeoff',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_takeoff.csv'),
            ('aligned_gps_raw_takeoff_v2',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_takeoff_v2.csv'),
            ('aligned_gps_raw_takeoff_v3_early',
             '20260509_fly1/mav0/gps/aligned_gps_from_raw_takeoff_v3_early.csv'),
            ('aligned_gps_mav0',
             '20260509_fly1/mav0/gps/aligned_gps.csv'),
        ],
    },
    2: {
        'raw_gps':   '20260509_fly2/mav0/gps/GPS.csv',
        'aligned': [
            ('aligned_corrected_cam',
             '20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_cam_time.DO_NOT_USE_contaminated.csv'),
            ('aligned_corrected_imu',
             '20260509_fly2/result/gps_tum_time_alignment_corrected/aligned_gps_imu_time.DO_NOT_USE_contaminated.csv'),
            ('aligned_vertical_cam',
             '20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv'),
            ('aligned_vertical_imu',
             '20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_imu_time.csv'),
        ],
    },
    3: {
        'raw_gps':   '20260509_fly3/mav0/gps/GPS.csv',
        'aligned': [
            ('aligned_default_cam',
             '20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
            ('aligned_default_imu',
             '20260509_fly3/result/gps_tum_time_alignment/aligned_gps_imu_time.csv'),
            ('aligned_fly4gps_cam',
             '20260509_fly3/result/gps_tum_time_alignment_fly4gps/aligned_gps_cam_time.csv'),
            ('aligned_fly4gps_imu',
             '20260509_fly3/result/gps_tum_time_alignment_fly4gps/aligned_gps_imu_time.csv'),
        ],
    },
    4: {
        'raw_gps':   '20260509_fly4/mav0/gps/GPS.csv',
        'aligned': [
            ('aligned_default_cam',
             '20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv'),
            ('aligned_default_imu',
             '20260509_fly4/result/gps_tum_time_alignment/aligned_gps_imu_time.csv'),
        ],
    },
}

# Known consumers (best-effort guess based on current YAML / CLI). Mark
# fusion_input if currently used as GPS-alt fusion source in baselines;
# eval_only if only used by evaluators / plotting; unused if neither.
# Post 2026-05-17 cleanup: see GPS_REFERENCE_AUDIT.md.
# fly2's _corrected aligned GPS is contaminated and must NOT be used.
# fly2 must use the _vertical aligned GPS.
FUSION_INPUTS = {
    1: 'aligned_offset_m4p63',
    2: 'aligned_vertical_cam',
    3: 'aligned_default_cam',
    4: 'aligned_default_cam',
}
QUARANTINED_LABELS = {
    'aligned_corrected_cam',
    'aligned_corrected_imu',
}

def load_raw_gps(path):
    """Raw GPS.csv -> (t_s_autopilot, alt_m) using TimeUS, Alt columns."""
    rows = []
    with open(path, newline='') as f:
        rdr = csv.DictReader(f)
        for r in rdr:
            try:
                t_us = float(r['TimeUS'])
                alt  = float(r['Alt'])
                rows.append((t_us * 1e-6, alt))
            except Exception:
                continue
    return np.asarray(rows)

def load_aligned_csv(path):
    """`#timestamp_ns,lat,lon,alt`  -> (t_s, alt_m). Auto-detect ns vs s."""
    rows = []
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.replace(',', ' ').split()
            try:
                t   = float(ps[0])
                alt = float(ps[3])
                rows.append((t, alt))
            except Exception:
                continue
    a = np.asarray(rows)
    if a.size and a[0,0] > 1e11:
        a[:,0] = a[:,0] * 1e-9
    return a

def loop_coverage(alts):
    """Return (has_50m_loop, has_90m_loop, agl_range_m).

    Uses the floor (min altitude) as ground and looks for samples in:
        ~50m loop:  AGL in [40, 60] m
        ~90m loop:  AGL in [80, 100] m
    """
    if alts.size == 0:
        return False, False, 0.0
    ground = float(np.percentile(alts, 1))  # robust ground
    agl    = alts - ground
    has50  = bool(((agl >= 40) & (agl <= 60)).sum() > 50)
    has90  = bool(((agl >= 80) & (agl <= 100)).sum() > 50)
    return has50, has90, float(agl.max())

def domain_of(path):
    p = path.lower()
    if 'mav0/gps/gps.csv' in p.replace('\\','/'):
        return 'raw_GPS_us'
    if 'cam_time' in p:
        return 'cam_time'
    if 'imu_time' in p:
        return 'imu_time'
    if 'aligned_gps_from_raw' in p:
        return 'cam_time(?)'  # not 100% sure of frame; tag tentatively
    if p.endswith('aligned_gps.csv'):
        return 'cam_time(?)'
    return 'unknown'

def role_of(fly, label):
    if label in QUARANTINED_LABELS:
        return 'QUARANTINED'
    if FUSION_INPUTS.get(fly) == label:
        return 'fusion_input'
    return 'eval_only'

print(f'{"flight":>2s}  {"label":<35s} {"domain":>14s} {"role":>14s} '
      f'{"first_s":>10s} {"last_s":>10s} {"dur_s":>7s} {"rows":>6s} '
      f'{"alt_min":>7s} {"alt_max":>7s} {"alt_med":>7s} '
      f'{"has50":>5s} {"has90":>5s} {"agl_max":>7s}')
print('-' * 165)

for fly, cfg in FLIGHTS.items():
    # Raw GPS first
    raw_path = cfg['raw_gps']
    if os.path.isfile(raw_path):
        a = load_raw_gps(raw_path)
        if a.size:
            has50, has90, agl_max = loop_coverage(a[:,1])
            print(f'{fly:>2d}  {"raw_GPS":<35s} {"raw_us":>14s} {"raw":>14s} '
                  f'{a[0,0]:>10.3f} {a[-1,0]:>10.3f} {a[-1,0]-a[0,0]:>7.1f} {len(a):>6d} '
                  f'{a[:,1].min():>7.2f} {a[:,1].max():>7.2f} {np.median(a[:,1]):>7.2f} '
                  f'{str(has50):>5s} {str(has90):>5s} {agl_max:>7.2f}  '
                  f'{raw_path}')
    # Aligned GPS candidates
    for label, path in cfg['aligned']:
        if not os.path.isfile(path):
            print(f'{fly:>2d}  {label:<35s}  MISSING  {path}')
            continue
        a = load_aligned_csv(path)
        if a.size == 0:
            print(f'{fly:>2d}  {label:<35s}  EMPTY    {path}')
            continue
        has50, has90, agl_max = loop_coverage(a[:,1])
        dom  = domain_of(path)
        role = role_of(fly, label)
        print(f'{fly:>2d}  {label:<35s} {dom:>14s} {role:>14s} '
              f'{a[0,0]:>10.3f} {a[-1,0]:>10.3f} {a[-1,0]-a[0,0]:>7.1f} {len(a):>6d} '
              f'{a[:,1].min():>7.2f} {a[:,1].max():>7.2f} {np.median(a[:,1]):>7.2f} '
              f'{str(has50):>5s} {str(has90):>5s} {agl_max:>7.2f}  '
              f'{path}')
    print()
