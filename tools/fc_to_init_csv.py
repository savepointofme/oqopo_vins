#!/usr/bin/env python3
"""Convert the GSMQ FC log into an OpenVINS init-state CSV.

Output columns:
  t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz

This is for FC-assisted VIO initialization only. It does not create a
continuous GPS/FC fusion stream.
"""

import argparse
import csv
import datetime as dt
import math
import os
import sys

import numpy as np


FC_COLUMNS = [
    "pitch_deg",
    "roll_deg",
    "yaw_deg",
    "gps_time",
    "satellites",
    "lat",
    "lon",
    "alt",
    "ve",
    "vn",
    "vu",
]


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rot_to_jpl_quat(rot):
    q = np.zeros(4)
    tr = float(np.trace(rot))
    if tr > rot[0, 0] and tr > rot[1, 1] and tr > rot[2, 2]:
        q[3] = 0.5 * math.sqrt(1.0 + tr)
        s = 0.25 / q[3]
        q[0] = (rot[1, 2] - rot[2, 1]) * s
        q[1] = (rot[2, 0] - rot[0, 2]) * s
        q[2] = (rot[0, 1] - rot[1, 0]) * s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        q[0] = 0.5 * math.sqrt(max(0.0, 1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]))
        s = 0.25 / q[0]
        q[1] = (rot[0, 1] + rot[1, 0]) * s
        q[2] = (rot[2, 0] + rot[0, 2]) * s
        q[3] = (rot[1, 2] - rot[2, 1]) * s
    elif rot[1, 1] > rot[2, 2]:
        q[1] = 0.5 * math.sqrt(max(0.0, 1.0 - rot[0, 0] + rot[1, 1] - rot[2, 2]))
        s = 0.25 / q[1]
        q[0] = (rot[0, 1] + rot[1, 0]) * s
        q[2] = (rot[1, 2] + rot[2, 1]) * s
        q[3] = (rot[2, 0] - rot[0, 2]) * s
    else:
        q[2] = 0.5 * math.sqrt(max(0.0, 1.0 - rot[0, 0] - rot[1, 1] + rot[2, 2]))
        s = 0.25 / q[2]
        q[0] = (rot[2, 0] + rot[0, 2]) * s
        q[1] = (rot[1, 2] + rot[2, 1]) * s
        q[3] = (rot[0, 1] - rot[1, 0]) * s
    if q[3] < 0:
        q = -q
    return q / np.linalg.norm(q)


def jpl_quat_to_rot(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    qv = q[:3]
    qw = q[3]
    qx = np.array([[0.0, -qv[2], qv[1]],
                   [qv[2], 0.0, -qv[0]],
                   [-qv[1], qv[0], 0.0]], dtype=float)
    return (2.0 * qw * qw - 1.0) * np.eye(3) - 2.0 * qw * qx + 2.0 * np.outer(qv, qv)


def heading_deg_from_enu(vec):
    return math.degrees(math.atan2(vec[0], vec[1])) % 360.0


def angle_diff_deg(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def aircraft_forward_axis_imu(body_to_imu):
    if body_to_imu in ("identity", "frd_to_flu"):
        return np.array([1.0, 0.0, 0.0])
    if body_to_imu == "frd_to_xright_yfwd_zup":
        return np.array([0.0, 1.0, 0.0])
    raise ValueError("unknown body_to_imu: %s" % body_to_imu)


def parse_fc_time_utc(text):
    text = text.strip()
    if "_" in text:
        base, ms = text.rsplit("_", 1)
        micros = int(ms.ljust(3, "0")[:3]) * 1000
    else:
        base = text
        micros = 0
    stamp = dt.datetime.strptime(base, "%Y-%m-%d %H:%M:%S")
    return stamp.replace(microsecond=micros, tzinfo=dt.timezone.utc).timestamp()


def iter_clean_csv_rows(path):
    with open(path, "rb") as f:
        for raw in f:
            raw = raw.replace(b"\x00", b"").strip()
            if not raw or raw.startswith(b"#"):
                continue
            yield raw.decode("utf-8-sig", "replace")


def camera_epoch_unix(dataset_dir):
    imu_path = os.path.join(dataset_dir, "imu0", "data.csv")
    for line in iter_clean_csv_rows(imu_path):
        row = next(csv.reader([line]))
        return float(row[1]) - float(row[0])
    raise RuntimeError("imu0/data.csv had no data rows")


def load_imu_window(dataset_dir, t0, t1):
    imu_path = os.path.join(dataset_dir, "imu0", "data.csv")
    acc = []
    gyro = []
    for line in iter_clean_csv_rows(imu_path):
        row = next(csv.reader([line]))
        t = float(row[0])
        if t < t0:
            continue
        if t > t1:
            break
        gyro.append([float(row[5]), float(row[6]), float(row[7])])
        acc.append([float(row[8]), float(row[9]), float(row[10])])
    if not acc:
        return None, None
    return np.mean(np.array(acc), axis=0), np.mean(np.array(gyro), axis=0)


def load_fc_rows(fc_path, cam_epoch=None, fc_rel_to_cam_offset=None):
    with open(fc_path, newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        raw_rows = [r for r in reader if r]
    if not raw_rows:
        raise RuntimeError("FC CSV is empty")
    header = [h.strip() for h in raw_rows[0]]
    fc0_unix = None
    rows = []
    for raw in raw_rows[1:]:
        if len(raw) < 11:
            continue
        try:
            unix = parse_fc_time_utc(raw[3])
            if fc0_unix is None:
                fc0_unix = unix
            if fc_rel_to_cam_offset is None:
                if cam_epoch is None:
                    raise RuntimeError("cam_epoch is required in absolute time mode")
                t_s = unix - cam_epoch
            else:
                t_s = (unix - fc0_unix) - fc_rel_to_cam_offset
            values = dict(zip(FC_COLUMNS, raw[:11]))
            rows.append(
                {
                    "t_s": t_s,
                    "fc_unix": unix,
                    "pitch_deg": float(values["pitch_deg"]),
                    "roll_deg": float(values["roll_deg"]),
                    "yaw_deg": float(values["yaw_deg"]),
                    "satellites": int(float(values["satellites"])),
                    "lat": float(values["lat"]),
                    "lon": float(values["lon"]),
                    "alt": float(values["alt"]),
                    "ve": float(values["ve"]),
                    "vn": float(values["vn"]),
                    "vu": float(values["vu"]),
                    "raw_time": values["gps_time"],
                }
            )
        except Exception:
            continue
    if not rows:
        raise RuntimeError("No FC data rows parsed")
    return header, raw_rows[1:], rows


def interp_angle_deg(a, b, alpha):
    d = (b - a + 180.0) % 360.0 - 180.0
    return a + alpha * d


def interpolate(rows, target_t):
    if target_t <= rows[0]["t_s"]:
        return dict(rows[0]), abs(rows[0]["t_s"] - target_t)
    if target_t >= rows[-1]["t_s"]:
        return dict(rows[-1]), abs(rows[-1]["t_s"] - target_t)
    for i in range(len(rows) - 1):
        a, b = rows[i], rows[i + 1]
        if a["t_s"] <= target_t <= b["t_s"]:
            alpha = (target_t - a["t_s"]) / (b["t_s"] - a["t_s"])
            out = {"t_s": target_t, "fc_unix": a["fc_unix"] + alpha * (b["fc_unix"] - a["fc_unix"])}
            for key in ["pitch_deg", "roll_deg", "ve", "vn", "vu", "lat", "lon", "alt", "satellites"]:
                out[key] = a[key] + alpha * (b[key] - a[key])
            out["yaw_deg"] = interp_angle_deg(a["yaw_deg"], b["yaw_deg"], alpha)
            out["raw_time"] = "%s..%s" % (a["raw_time"], b["raw_time"])
            return out, min(abs(target_t - a["t_s"]), abs(target_t - b["t_s"]))
    raise RuntimeError("Interpolation failed")


def fc_attitude_to_R_GtoI(roll_deg, pitch_deg, yaw_deg, yaw_sign=-1.0, body_to_imu="frd_to_xright_yfwd_zup"):
    """Return OpenVINS R_GtoI.

    FC attitude is interpreted as NED ZYX Euler, body frame FRD. The observed
    log has yaw sign opposite GPS track, so yaw_sign defaults to -1.
    D455 IMU gravity plus optical-flow direction supports
    FRD -> x_right, y_forward, z_up for aircraft-body to IMU.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_sign * yaw_deg)
    R_B_to_NED = rz(y) @ ry(p) @ rx(r)
    R_ENU_to_NED = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], dtype=float)
    R_G_to_FRD = R_B_to_NED.T @ R_ENU_to_NED
    if body_to_imu == "identity":
        R_I_from_FRD = np.eye(3)
    elif body_to_imu == "frd_to_flu":
        R_I_from_FRD = np.diag([1.0, -1.0, -1.0])
    elif body_to_imu == "frd_to_xright_yfwd_zup":
        R_I_from_FRD = np.array([[0.0, 1.0, 0.0],
                                 [1.0, 0.0, 0.0],
                                 [0.0, 0.0, -1.0]], dtype=float)
    else:
        raise ValueError("unknown body_to_imu: %s" % body_to_imu)
    return R_I_from_FRD @ R_G_to_FRD


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fc", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-time", type=float, required=True)
    parser.add_argument("--yaw-sign", type=float, default=-1.0)
    parser.add_argument("--body-to-imu", choices=["frd_to_xright_yfwd_zup", "frd_to_flu", "identity"],
                        default="frd_to_xright_yfwd_zup")
    parser.add_argument("--fc-rel-to-cam-offset", type=float, default=None,
                        help="Use manual timing t_cam = (fc_time - first_fc_time) - OFFSET instead of absolute UTC.")
    parser.add_argument("--max-heading-diff-deg", type=float, default=25.0,
                        help="Fail if aircraft-forward heading and FC velocity track differ by more than this.")
    parser.add_argument("--allow-heading-mismatch", action="store_true",
                        help="Only warn on heading mismatch instead of failing.")
    args = parser.parse_args()

    if args.fc_rel_to_cam_offset is None:
        cam_epoch = camera_epoch_unix(args.dataset)
        time_mode = "absolute UTC: t_cam = fc_unix - camera_epoch"
    else:
        cam_epoch = None
        time_mode = "manual relative offset: t_cam = (fc_time - first_fc_time) - %.6f" % args.fc_rel_to_cam_offset
    header, raw_rows, rows = load_fc_rows(args.fc, cam_epoch, args.fc_rel_to_cam_offset)
    state, dt_nearest = interpolate(rows, args.start_time)

    print("FC file path:", args.fc)
    print("Header columns:", header)
    print("First 5 rows:")
    for row in raw_rows[:5]:
        print(row)
    print("Last 5 rows:")
    for row in raw_rows[-5:]:
        print(row)
    print("Time mode:", time_mode)
    if cam_epoch is not None:
        print("Camera epoch unix: %.6f" % cam_epoch)
        print("Camera epoch UTC:", dt.datetime.fromtimestamp(cam_epoch, dt.timezone.utc).isoformat())
    print("FC unix range: %.3f %.3f" % (rows[0]["fc_unix"], rows[-1]["fc_unix"]))
    print("FC camera-time range: %.3f %.3f" % (rows[0]["t_s"], rows[-1]["t_s"]))

    speed = math.sqrt(state["ve"] ** 2 + state["vn"] ** 2 + state["vu"] ** 2)
    track = math.degrees(math.atan2(state["ve"], state["vn"])) % 360.0
    yaw_heading = (args.yaw_sign * state["yaw_deg"]) % 360.0
    print("FC state nearest/interp camera t=%.3f, dt_nearest=%.3f" % (args.start_time, dt_nearest))
    print("  roll/pitch/yaw_raw deg: %.4f %.4f %.4f" % (state["roll_deg"], state["pitch_deg"], state["yaw_deg"]))
    print("  yaw_heading_after_sign deg: %.4f, GPS track deg: %.4f, diff: %.4f" %
          (yaw_heading, track, ((yaw_heading - track + 180.0) % 360.0) - 180.0))
    print("  Ve/Vn/Vu m/s: %.4f %.4f %.4f, speed=%.4f" % (state["ve"], state["vn"], state["vu"], speed))
    print("  lat/lon/alt: %.8f %.8f %.3f" % (state["lat"], state["lon"], state["alt"]))

    R_GtoI = fc_attitude_to_R_GtoI(
        state["roll_deg"], state["pitch_deg"], state["yaw_deg"], args.yaw_sign, args.body_to_imu
    )
    q = rot_to_jpl_quat(R_GtoI)
    forward_axis_I = aircraft_forward_axis_imu(args.body_to_imu)
    forward_G = jpl_quat_to_rot(q).T @ forward_axis_I
    forward_heading = heading_deg_from_enu(forward_G)
    heading_diff = angle_diff_deg(forward_heading, track)
    print("  aircraft-forward heading from q: %.4f deg, GPS track: %.4f deg, diff: %.4f deg" %
          (forward_heading, track, heading_diff))
    if speed > 10.0 and abs(heading_diff) > args.max_heading_diff_deg:
        msg = ("FC init heading sanity failed: |%.2f deg| > %.2f deg. "
               "Check yaw_sign, body_to_imu, time alignment, or active/passive rotation convention." %
               (heading_diff, args.max_heading_diff_deg))
        if args.allow_heading_mismatch:
            print("WARNING:", msg, file=sys.stderr)
        else:
            raise RuntimeError(msg)
    pred_acc = R_GtoI @ np.array([0.0, 0.0, 9.81])
    acc_avg, gyro_avg = load_imu_window(args.dataset, args.start_time - 0.5, args.start_time + 0.5)
    if acc_avg is not None:
        cosang = float(np.dot(pred_acc / np.linalg.norm(pred_acc), acc_avg / np.linalg.norm(acc_avg)))
        angle = math.degrees(math.acos(max(-1.0, min(1.0, cosang))))
        print("  avg IMU accel [%.4f %.4f %.4f] norm=%.4f" %
              (acc_avg[0], acc_avg[1], acc_avg[2], np.linalg.norm(acc_avg)))
        print("  predicted gravity accel [%.4f %.4f %.4f], angle_to_avg=%.3f deg" %
              (pred_acc[0], pred_acc[1], pred_acc[2], angle))
        print("  avg IMU gyro [%.5f %.5f %.5f] norm=%.5f" %
              (gyro_avg[0], gyro_avg[1], gyro_avg[2], np.linalg.norm(gyro_avg)))

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", newline="") as f:
        f.write("# FC-assisted VIO initialization, no continuous GPS/FC fusion\n")
        if args.fc_rel_to_cam_offset is None:
            f.write("# FC time parsed as absolute UTC; FC yaw sign %.1f; body_to_imu %s\n" %
                    (args.yaw_sign, args.body_to_imu))
        else:
            f.write("# FC time used relatively only; fc_rel_to_cam_offset %.6f; FC yaw sign %.1f; body_to_imu %s\n" %
                    (args.fc_rel_to_cam_offset, args.yaw_sign, args.body_to_imu))
        f.write("# t_s,qx,qy,qz,qw,vx,vy,vz,px,py,pz,bgx,bgy,bgz,bax,bay,baz\n")
        f.write(
            "%.6f,%.10f,%.10f,%.10f,%.10f,%.6f,%.6f,%.6f,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0\n"
            % (args.start_time, q[0], q[1], q[2], q[3], state["ve"], state["vn"], state["vu"])
        )
    print("Wrote:", args.output)

    if speed < 25.0:
        print("WARNING: speed is below fixed-wing airborne expectation; check time alignment", file=sys.stderr)


if __name__ == "__main__":
    main()
