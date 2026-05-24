#!/usr/bin/env python3
"""Audit `gps_altitude_data_v2.zip` against canonical raw GPS sources.

The package contains two file kinds per flight:
  - raw_gps_altitude_flyN.csv     (raw timestamps + lat/lon/alt subset)
  - aligned_gps_altitude_flyN.csv (camera-time-aligned (t_s, alt_m) pairs)

This script does NOT trust the comments inside the zip; it validates the
DATA by:
  1. comparing row count, first/last TimeUS, alt range, first lat/lon
     against the canonical external GPS at D:\\vscode_dir\\20260509_flyN_gps\\GPS.csv
  2. confirming aligned-cam-time GPS spans a sensible range with the right
     row count and altitude curve shape
  3. flagging contamination (fly2 with 4753 rows / alt to 125 m, fly1 with
     1743 rows, stereo-pseudo-ref content, etc.)

Reports a usability verdict per file.
"""
import csv, os, sys
import numpy as np

ZIP_DIR = os.environ.get('GPS_ALT_AUDIT_DIR',
    os.path.expanduser('~/AppData/Local/Temp/gps_alt_audit'))
CANONICAL = {
    1: 'D:/vscode_dir/20260509_fly1_gps/GPS.csv',
    2: 'D:/vscode_dir/20260509_fly2_gps/GPS.csv',
    3: 'D:/vscode_dir/20260509_fly3_gps/GPS.csv',
    4: 'D:/vscode_dir/20260509_fly4_gps/GPS.csv',
}

def load_canonical(p):
    """ArduPilot GPS.csv -> rows of (t_us, lat, lon, alt)."""
    out = []
    with open(p, newline='') as f:
        rdr = csv.DictReader(f)
        for r in rdr:
            try:
                out.append((float(r['TimeUS']), float(r['Lat']),
                            float(r['Lng']), float(r['Alt'])))
            except Exception:
                continue
    return np.asarray(out)

def load_raw_alt_pkg(p):
    """zip raw_gps_altitude_flyN.csv -> rows of (t_us, lat, lon, alt).
    Columns (positional): TimeUS, GMS, GWk, Lat, Lng, Alt, NSats, HDop, Status.
    """
    out = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split(',')
            try:
                out.append((float(ps[0]), float(ps[3]),
                            float(ps[4]), float(ps[5])))
            except Exception:
                continue
    return np.asarray(out)

def load_aligned_pkg(p):
    """zip aligned_gps_altitude_flyN.csv -> rows of (t_s, alt_m). 2 columns."""
    out = []
    with open(p) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith('#'): continue
            ps = ln.split(',')
            try:
                out.append((float(ps[0]), float(ps[1])))
            except Exception:
                continue
    return np.asarray(out)

def stats(a, label_cols):
    """Pretty stat dict."""
    return {
        'n': len(a),
        f'first_{label_cols[0]}': float(a[0, 0]),
        f'last_{label_cols[0]}':  float(a[-1, 0]),
        'dur': float(a[-1, 0] - a[0, 0]),
        f'first_alt': float(a[0, -1]),
        f'last_alt':  float(a[-1, -1]),
        'alt_min': float(a[:, -1].min()),
        'alt_max': float(a[:, -1].max()),
        'alt_med': float(np.median(a[:, -1])),
    }

def verdict_raw(fly, pkg_raw, canon):
    notes = []
    ok = True
    # Row count must match
    if len(pkg_raw) != len(canon):
        ok = False
        notes.append(f'row mismatch: pkg={len(pkg_raw)} canon={len(canon)}')
    # First/last TimeUS must match
    if abs(pkg_raw[0, 0] - canon[0, 0]) > 1.0:
        ok = False
        notes.append(f'first TimeUS mismatch: pkg={pkg_raw[0,0]} canon={canon[0,0]}')
    if abs(pkg_raw[-1, 0] - canon[-1, 0]) > 1.0:
        ok = False
        notes.append(f'last TimeUS mismatch: pkg={pkg_raw[-1,0]} canon={canon[-1,0]}')
    # First lat/lon
    if abs(pkg_raw[0, 1] - canon[0, 1]) > 1e-5:
        ok = False
        notes.append(f'first lat mismatch: pkg={pkg_raw[0,1]} canon={canon[0,1]}')
    if abs(pkg_raw[0, 2] - canon[0, 2]) > 1e-5:
        ok = False
        notes.append(f'first lon mismatch: pkg={pkg_raw[0,2]} canon={canon[0,2]}')
    # Alt range
    if abs(pkg_raw[:, 3].min() - canon[:, 3].min()) > 0.1:
        ok = False
        notes.append(f'alt min mismatch: pkg={pkg_raw[:,3].min():.2f} canon={canon[:,3].min():.2f}')
    if abs(pkg_raw[:, 3].max() - canon[:, 3].max()) > 0.1:
        ok = False
        notes.append(f'alt max mismatch: pkg={pkg_raw[:,3].max():.2f} canon={canon[:,3].max():.2f}')
    # Contamination heuristics
    if fly == 2 and (len(pkg_raw) > 3000 or pkg_raw[:, 3].max() > 100):
        ok = False
        notes.append('CONTAMINATED: fly2 file shaped like fly1 (rows/alt too large)')
    if fly == 1 and (len(pkg_raw) < 3000 or pkg_raw[:, 0].max() - pkg_raw[:, 0].min() < 5e8):
        ok = False
        notes.append('TRUNCATED: fly1 file too short / few rows')
    return ok, notes

def verdict_aligned(fly, pkg_alt, canon):
    notes = []
    ok = True
    if len(pkg_alt) != len(canon):
        ok = False
        notes.append(f'row mismatch: pkg={len(pkg_alt)} canon={len(canon)}')
    # Alt range
    if abs(pkg_alt[:, 1].min() - canon[:, 3].min()) > 0.1:
        ok = False
        notes.append(f'alt min mismatch: pkg={pkg_alt[:,1].min():.2f} canon={canon[:,3].min():.2f}')
    if abs(pkg_alt[:, 1].max() - canon[:, 3].max()) > 0.1:
        ok = False
        notes.append(f'alt max mismatch: pkg={pkg_alt[:,1].max():.2f} canon={canon[:,3].max():.2f}')
    # Duration
    dur_pkg = pkg_alt[-1, 0] - pkg_alt[0, 0]
    dur_canon = (canon[-1, 0] - canon[0, 0]) * 1e-6  # us -> s
    if abs(dur_pkg - dur_canon) > 0.5:
        ok = False
        notes.append(f'duration mismatch: pkg={dur_pkg:.2f}s canon={dur_canon:.2f}s')
    # Sample-by-sample alt sequence should match canonical (sorted by time)
    if len(pkg_alt) == len(canon):
        diff_alt = np.abs(pkg_alt[:, 1] - canon[:, 3])
        if diff_alt.max() > 0.01:
            ok = False
            notes.append(f'per-sample alt mismatch (max diff = {diff_alt.max():.3f} m)')
    # Contamination heuristics
    if fly == 2 and (len(pkg_alt) > 3000 or pkg_alt[:, 1].max() > 100):
        ok = False
        notes.append('CONTAMINATED: fly2 aligned file shaped like fly1')
    return ok, notes

print('=' * 110)
print(' GPS_ALTITUDE_DATA_V2 PACKAGE AUDIT')
print('=' * 110)

for fly in [1, 2, 3, 4]:
    canon = load_canonical(CANONICAL[fly])
    pkg_raw_path = f'{ZIP_DIR}/raw_gps_altitude_fly{fly}.csv'
    pkg_aln_path = f'{ZIP_DIR}/aligned_gps_altitude_fly{fly}.csv'
    pkg_raw = load_raw_alt_pkg(pkg_raw_path)
    pkg_aln = load_aligned_pkg(pkg_aln_path)

    print(f'\n----- fly{fly} -----')
    print(f' canonical: {CANONICAL[fly]}')
    sc = {'n': len(canon),
          'first_TimeUS': float(canon[0, 0]),
          'last_TimeUS': float(canon[-1, 0]),
          'dur_s': float((canon[-1, 0] - canon[0, 0]) * 1e-6),
          'first_alt': float(canon[0, 3]),
          'last_alt': float(canon[-1, 3]),
          'alt_min': float(canon[:, 3].min()),
          'alt_max': float(canon[:, 3].max()),
          'first_lat': float(canon[0, 1]),
          'first_lon': float(canon[0, 2])}
    print(f'  canon  -> {sc}')

    # raw_gps_altitude_flyN.csv
    print(f'\n raw:     {pkg_raw_path}')
    sr = {'n': len(pkg_raw),
          'first_TimeUS': float(pkg_raw[0, 0]),
          'last_TimeUS': float(pkg_raw[-1, 0]),
          'dur_s': float((pkg_raw[-1, 0] - pkg_raw[0, 0]) * 1e-6),
          'first_alt': float(pkg_raw[0, 3]),
          'last_alt': float(pkg_raw[-1, 3]),
          'alt_min': float(pkg_raw[:, 3].min()),
          'alt_max': float(pkg_raw[:, 3].max()),
          'first_lat': float(pkg_raw[0, 1]),
          'first_lon': float(pkg_raw[0, 2])}
    print(f'  pkg    -> {sr}')
    ok_raw, notes_raw = verdict_raw(fly, pkg_raw, canon)
    tag = '[OK]' if ok_raw else '[FAIL]'
    print(f'  verdict: {tag}')
    for n in notes_raw: print(f'    - {n}')

    # aligned_gps_altitude_flyN.csv
    print(f'\n aligned: {pkg_aln_path}')
    sa = {'n': len(pkg_aln),
          'first_t_s': float(pkg_aln[0, 0]),
          'last_t_s': float(pkg_aln[-1, 0]),
          'dur_s': float(pkg_aln[-1, 0] - pkg_aln[0, 0]),
          'first_alt': float(pkg_aln[0, 1]),
          'last_alt': float(pkg_aln[-1, 1]),
          'alt_min': float(pkg_aln[:, 1].min()),
          'alt_max': float(pkg_aln[:, 1].max())}
    print(f'  pkg    -> {sa}')
    ok_aln, notes_aln = verdict_aligned(fly, pkg_aln, canon)
    tag = '[OK]' if ok_aln else '[FAIL]'
    print(f'  verdict: {tag}')
    for n in notes_aln: print(f'    - {n}')

    # Check claimed "Source:" comment vs reality
    src_comment = None
    with open(pkg_raw_path) as f:
        for ln in f:
            if ln.startswith('# Source:'):
                src_comment = ln.strip()
                break
    print(f'  raw "Source:" comment: {src_comment}')
    if src_comment:
        # Extract whatever fly number the comment refers to
        import re
        m = re.search(r'20260509_fly(\d+)_gps', src_comment)
        if m:
            claimed_fly = int(m.group(1))
            if claimed_fly != fly:
                print(f'  WARN: comment says fly{claimed_fly} source but data matches fly{fly}')
