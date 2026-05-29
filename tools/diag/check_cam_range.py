#!/usr/bin/env python3
import csv

datasets = {
    "fly1": "/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810/cam0/data.csv",
    "fly2": "/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/d455_20260517_184722/cam0/data.csv",
    "fly3": "/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946/cam0/data.csv",
    "fly4": "/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/d455_20260527_090549/cam0/data.csv",
}

for name, path in datasets.items():
    try:
        ts = []
        with open(path) as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'): continue
                try: ts.append(float(row[0]) / 1e9)
                except: pass
        if len(ts) >= 2:
            print(f"{name}: cam t=[{ts[0]:.1f}, {ts[-1]:.1f}]  n={len(ts)}")
    except Exception as e:
        print(f"{name}: {e}")
