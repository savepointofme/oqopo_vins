#!/usr/bin/env python3
import csv

flights = [
    ("fly1", "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/gps_aligned.csv", 0),
    ("fly2", "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/gps_from_mems_offset450p5_cam_time.csv", 0),
    ("fly3", "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv", 0),
    ("fly4", "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv", 0),
]

for name, path, _ in flights:
    try:
        rows = []
        with open(path) as f:
            reader = csv.DictReader(f)
            for r in reader:
                ts_col = list(r.keys())[0]
                t = float(r[ts_col])
                if t > 1e9:  # nanoseconds
                    t /= 1e9
                rows.append(t)
        print(f"{name}: t=[{rows[0]:.1f}, {rows[-1]:.1f}]  n={len(rows)}")
    except Exception as e:
        print(f"{name}: ERROR {e}")
