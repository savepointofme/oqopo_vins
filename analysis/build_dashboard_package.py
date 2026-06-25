#!/usr/bin/env python3
"""Generate the official interactive_dashboard.html for every trajectory in a
runs/ directory and assemble a browsable, paired delivery package.

For each run dir (containing traj.txt + traj.txt.bias + diag.csv) it runs the
canonical full_flight_error_analysis.py (which emits reports/interactive_dashboard
.html), then copies the dashboard + traj.txt into package/<run>/ and writes a
top-level index.html linking every trajectory to its dashboard.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def winpath(p: str) -> str:
    if p.startswith("/mnt/") and len(p) > 6 and p[5].isalpha() and p[6] == "/":
        return p[5].upper() + ":" + p[6:]
    return p


def flight_of(name: str) -> str:
    for f in ("fly1", "fly2", "fly3", "fly4"):
        if name.startswith(f):
            return f
    return ""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--runs-root", required=True)
    ap.add_argument("--package", required=True)
    ap.add_argument("--windows-root", required=True,
                    help="dir holding COMMON_fly*/COMMON_LAP_SEGMENTS.csv")
    ap.add_argument("--glob", default="*_stride*")
    ap.add_argument("--reuse-existing", action="store_true",
                    help="reuse a package run when its canonical dashboard and global summary already exist")
    args = ap.parse_args()

    man = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    lap_segments = {}
    for f in ("fly1", "fly2", "fly3", "fly4"):
        sf = Path(args.windows_root) / f"COMMON_{f}" / "COMMON_LAP_SEGMENTS.csv"
        if sf.exists():
            lap_segments[f] = sf
    pkg = Path(args.package)
    (pkg).mkdir(parents=True, exist_ok=True)
    rows = []
    package_metadata = []
    runs = sorted(p for p in Path(args.runs_root).glob(args.glob)
                  if p.is_dir() and ".old_" not in p.name and (p / "traj.txt").exists())
    for rd in runs:
        fl = flight_of(rd.name)
        if not fl or fl not in man:
            print(f"  skip {rd.name}: unknown flight"); continue
        m = man[fl]
        out = pkg / rd.name
        adir = out / "analysis"
        existing_dash = adir / "reports" / "interactive_dashboard.html"
        existing_gsum = adir / "tables" / "global_summary.csv"
        if not (args.reuse_existing and existing_dash.exists() and existing_gsum.exists()):
            cmd = [sys.executable, str(HERE / "full_flight_error_analysis.py"),
                   "--gps", winpath(m["gps"]),
                   "--vio-traj", str(rd / "traj.txt"),
                   "--vio-bias", str(rd / "traj.txt.bias"),
                   "--vio-diag", str(rd / "diag.csv"),
                   "--vio-yaw-diag", str(rd / "vio_yaw_diag.csv"),
                   "--t0", str(m["start_time"]),
                   "--t1", str(m["full_end_time"]),
                   "--crop-at-sustained-divergence",
                   "--divergence-xy-threshold-m", "1000",
                   "--crop-reason", "full requested flight window; per-trajectory first-divergence crop enabled",
                   "--flight-name", fl, "--method-name", rd.name,
                   "--out-dir", str(adir)]
            if fl in lap_segments:
                cmd += ["--segment-index-csv", str(lap_segments[fl])]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except subprocess.CalledProcessError as e:
                print(f"  FAIL {rd.name}: {e}"); continue
        command_path = rd / "command.txt"
        command_text = command_path.read_text(encoding="utf-8", errors="replace") if command_path.exists() else ""
        forbidden_yaw_flags = ("--gps-yaw", "--gps-course-update", "--gps-heading")
        if any(flag in command_text for flag in forbidden_yaw_flags):
            raise RuntimeError(f"{rd.name}: forbidden GPS-yaw fusion flag found in command.txt")
        provenance_path = adir / "metadata" / "analysis_provenance.json"
        if provenance_path.exists():
            provenance = json.loads(provenance_path.read_text(encoding="utf-8"))
            provenance["gps_yaw_fusion"] = False
            provenance["gps_altitude_update"] = True
            provenance_path.write_text(json.dumps(provenance, indent=2, ensure_ascii=False) + "\n",
                                       encoding="utf-8")
        out.mkdir(parents=True, exist_ok=True)
        dash = adir / "reports" / "interactive_dashboard.html"
        gsum = adir / "tables" / "global_summary.csv"
        shutil.copy2(rd / "traj.txt", out / "traj.txt")
        if dash.exists():
            shutil.copy2(dash, out / "interactive_dashboard.html")
        course = scale = stable_end = div_t = div_km = "-"
        if gsum.exists():
            import csv
            with open(gsum) as fh:
                r = next(csv.DictReader(fh))
                course = round(float(r.get("yaw/course_rmse_deg", 0)), 2)
                scale = round(float(r.get("speed_rmse_mps", 0)), 2)
                stable_end = round(float(r.get("last_stable_time_s", 0)), 1)
                if str(r.get("divergence_detected", "")).lower() == "true":
                    div_t = round(float(r["divergence_time_s"]), 1)
                    div_km = round(float(r["divergence_gps_distance_km"]), 2)
        rows.append((rd.name, fl, course, scale, stable_end, div_t, div_km,
                     f"{rd.name}/interactive_dashboard.html", f"{rd.name}/traj.txt"))
        package_metadata.append({
            "run": rd.name,
            "flight": fl,
            "gps_reference": winpath(m["gps"]),
            "gps_evaluation_reference": True,
            "gps_altitude_update": True,
            "gps_yaw_fusion": False,
            "command_file": f"../runs/{rd.name}/command.txt",
        })
        print(f"  [ok] {rd.name}  course={course} speed={scale}")

    # index.html
    body = ["<html><head><meta charset='utf-8'><title>Trajectory dashboard package</title>",
            "<style>body{font-family:sans-serif;margin:24px}table{border-collapse:collapse}",
            "td,th{border:1px solid #ccc;padding:6px 10px}tr:nth-child(even){background:#f6f6f6}</style></head><body>",
            f"<h2>Trajectory dashboard package ({len(rows)} trajectories)</h2>",
            "<table><tr><th>run</th><th>flight</th><th>course RMSE (stable)</th>",
            "<th>speed RMSE (pre-divergence)</th><th>pre-divergence cutoff t (s)</th>"
            "<th>divergence t (s)</th><th>divergence distance (km)</th>"
            "<th>dashboard</th><th>traj.txt</th></tr>"]
    for name, fl, c, s, stable, dt, dk, dh, tj in rows:
        body.append(f"<tr><td>{name}</td><td>{fl}</td><td>{c}</td><td>{s}</td>"
                    f"<td>{stable}</td><td>{dt}</td><td>{dk}</td>"
                    f"<td><a href='{dh}'>interactive_dashboard.html</a></td>"
                    f"<td><a href='{tj}'>traj.txt</a></td></tr>")
    body.append("</table></body></html>")
    (pkg / "index.html").write_text("\n".join(body), encoding="utf-8")
    (pkg / "PACKAGE_METADATA.json").write_text(
        json.dumps({"trajectory_count": len(package_metadata),
                    "runs": package_metadata}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    print(f"\nwrote {len(rows)} dashboards + index.html to {pkg}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
