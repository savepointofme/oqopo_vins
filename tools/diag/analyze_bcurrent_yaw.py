#!/usr/bin/env python3
"""Time-series analysis of B_current heading error vs bg_z, Pss, gauge pressure."""
import csv, math, numpy as np

GPS_FILE  = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv"
DIAG_FULL = "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/fly4_schmidt_B_offsetm202p2_start924p4_until2816/schmidt_yaw_update_diag.csv"

# ── GPS heading ────────────────────────────────────────────────────────────
print("Loading GPS...")
gps_rows = []
with open(GPS_FILE) as f:
    for r in csv.DictReader(f):
        t = float(r['ts_ns']) / 1e9
        lat = float(r['lat'])
        lon = float(r['lon'])
        gps_rows.append((t, lat, lon))
gps_rows.sort(key=lambda x: x[0])

lat0 = math.radians(gps_rows[0][1])
lon0 = math.radians(gps_rows[0][2])
R_earth = 6371000.0

def ll2en(lat_deg, lon_deg):
    dlat = math.radians(lat_deg) - lat0
    dlon = math.radians(lon_deg) - lon0
    east  = R_earth * math.cos(lat0) * dlon
    north = R_earth * dlat
    return east, north

en = [(t, ) + ll2en(lat, lon) for t, lat, lon in gps_rows]

gps_head = []
for i in range(1, len(en)-1):
    t  = en[i][0]
    dt = en[i+1][0] - en[i-1][0]
    if dt < 1e-6:
        continue
    de = en[i+1][1] - en[i-1][1]
    dn = en[i+1][2] - en[i-1][2]
    spd = math.hypot(de, dn) / dt
    if spd < 1.5:
        continue
    hdg = math.degrees(math.atan2(de, dn))
    gps_head.append((t, hdg, spd))
print(f"  GPS heading samples: {len(gps_head)}  t=[{gps_head[0][0]:.1f}, {gps_head[-1][0]:.1f}]")

def gps_hdg_at(tq):
    if not gps_head:
        return None
    best = min(gps_head, key=lambda x: abs(x[0]-tq))
    return best[1] if abs(best[0]-tq) < 2.0 else None

# ── B_current diag ─────────────────────────────────────────────────────────
print("Loading B_current diag...")
diag = []
with open(DIAG_FULL) as f:
    for r in csv.DictReader(f):
        try:
            diag.append({
                't':         float(r['timestamp']),
                'yaw_vio':   float(r['yaw_before_update']),
                'bg_z':      float(r['bg_z_before']),
                'Pss':       float(r['Pss_norm_before']),
                'ndx':       abs(float(r['normal_dx_s_coeff_before'])),
                'Pas':       float(r['Pas_change_norm']),
                'dy':        float(r['delta_yaw_update']),
            })
        except (ValueError, KeyError):
            pass
diag.sort(key=lambda x: x['t'])
print(f"  {len(diag)} updates  t=[{diag[0]['t']:.1f}, {diag[-1]['t']:.1f}]")

# ── Align VIO to GPS at t=924.4 ────────────────────────────────────────────
T_ALIGN = 924.4
vio0 = next((d['yaw_vio'] for d in diag if d['t'] >= T_ALIGN), None)
gps0 = gps_hdg_at(T_ALIGN)
if vio0 is None or gps0 is None:
    print("ERROR: cannot align")
    offset = 0.0
else:
    offset = vio0 - gps0
    print(f"  Align: VIO={vio0:.2f}°  GPS={gps0:.2f}°  offset={offset:.2f}°")

def yaw_err(vio, t):
    gh = gps_hdg_at(t)
    if gh is None:
        return None
    e = vio - gh - offset
    while e >  180: e -= 360
    while e < -180: e += 360
    return e

# ── Binned time-series ─────────────────────────────────────────────────────
BIN = 50.0
T0, T1 = 924.0, 2820.0
bins = np.arange(T0, T1, BIN)

print(f"\n{'='*108}")
print(f"  {'Win':11s}  {'n':>5s}  {'yaw_VIO':>8s}  {'yaw_GPS':>8s}  {'yaw_err':>8s}  "
      f"{'bg_z×1e4':>9s}  {'Pss×1e-6':>9s}  {'ndx_mean':>9s}  {'ndx_P95':>8s}  {'ndx_max':>8s}  {'Pas_mean':>9s}")
print(f"{'='*108}")

yaw_onset = None
bgz_onset = None

for t0 in bins:
    t1 = t0 + BIN
    rows = [d for d in diag if t0 <= d['t'] < t1]
    if not rows:
        continue
    n    = len(rows)
    ymn  = float(np.mean([r['yaw_vio'] for r in rows]))
    bgz  = float(np.mean([r['bg_z']    for r in rows]))
    Pss  = float(np.mean([r['Pss']     for r in rows]))
    ndxs = [r['ndx'] for r in rows]
    ndxm = float(np.mean(ndxs))
    ndxp = float(np.percentile(ndxs, 95))
    ndxM = float(np.max(ndxs))
    Pas  = float(np.mean([r['Pas']     for r in rows]))

    gh  = gps_hdg_at(t0 + BIN/2)
    ye  = None
    if gh is not None:
        ye = ymn - gh - offset
        while ye >  180: ye -= 360
        while ye < -180: ye += 360

    flag = ""
    if ye is not None and abs(ye) > 5 and yaw_onset is None:
        yaw_onset = t0
        flag += " ←YAW>5°"
    if abs(bgz) > 1e-4 and bgz_onset is None:
        bgz_onset = t0
        flag += " ←BGZ>1e-4"

    gps_s = f"{gh:8.2f}" if gh is not None else f"{'N/A':>8s}"
    ye_s  = f"{ye:8.2f}" if ye is not None else f"{'N/A':>8s}"

    print(f"  {t0:5.0f}-{t1:5.0f}  {n:5d}  {ymn:8.2f}  {gps_s}  {ye_s}  "
          f"{bgz*1e4:9.4f}  {Pss/1e6:9.3f}  {ndxm:9.4f}  {ndxp:8.4f}  {ndxM:8.4f}  {Pas:9.4f}{flag}")

print(f"{'='*108}")
print(f"\n  bg_z > 1e-4 onset:  t={bgz_onset}")
print(f"  yaw err > 5° onset: t={yaw_onset}")

# ── Cumulative yaw delta by epoch ─────────────────────────────────────────
print(f"\n{'='*72}")
print("  Cumulative VIO yaw delta (signed, deg) and gauge-pressure by 200s epoch")
print(f"{'='*72}")
print(f"  {'Epoch':14s}  {'Σdelta_yaw':>12s}  {'ndx>0.5':>9s}  {'n_total':>9s}  {'frac%':>7s}")
print(f"  {'-'*70}")
for t0 in np.arange(T0, T1, 200.0):
    t1 = t0 + 200.0
    rows = [d for d in diag if t0 <= d['t'] < t1]
    if not rows:
        continue
    cum  = sum(r['dy'] for r in rows)
    nhigh = sum(1 for r in rows if r['ndx'] > 0.5)
    print(f"  {t0:.0f}-{t1:.0f}        {cum:12.4f}  {nhigh:9d}  {len(rows):9d}  {100*nhigh/len(rows):7.1f}%")

# ── Lead-lag: does bg_z precede yaw_err? ──────────────────────────────────
print(f"\n{'='*64}")
print("  Lead-lag check: bg_z_before vs yaw_error (50s bins)")
print(f"  (negative bg_z = CCW gyro bias → heading drifts CW over time)")
print(f"{'='*64}")
print(f"  {'Win':11s}  {'bg_z×1e4':>10s}  {'Δbg_z×1e4':>11s}  {'yaw_err':>10s}  {'Δyaw_err':>10s}")
print(f"  {'-'*62}")
prev_bgz = None
prev_ye  = None
for t0 in bins:
    t1 = t0 + BIN
    rows = [d for d in diag if t0 <= d['t'] < t1]
    if not rows:
        continue
    bgz = float(np.mean([r['bg_z'] for r in rows]))
    ymn = float(np.mean([r['yaw_vio'] for r in rows]))
    gh  = gps_hdg_at(t0 + BIN/2)
    if gh is None:
        prev_bgz = bgz
        continue
    ye = ymn - gh - offset
    while ye > 180: ye -= 360
    while ye < -180: ye += 360

    d_bgz = (bgz - prev_bgz)*1e4 if prev_bgz is not None else float('nan')
    d_ye  = ye  - prev_ye        if prev_ye  is not None else float('nan')
    d_bgz_s = f"{d_bgz:11.4f}" if math.isfinite(d_bgz) else f"{'':>11s}"
    d_ye_s  = f"{d_ye:10.4f}"  if math.isfinite(d_ye)  else f"{'':>10s}"

    print(f"  {t0:5.0f}-{t1:5.0f}   {bgz*1e4:10.4f}  {d_bgz_s}  {ye:10.4f}  {d_ye_s}")
    prev_bgz = bgz
    prev_ye  = ye

print("\nDONE")
