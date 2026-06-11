#!/usr/bin/env python3
"""
scale_gyro_dataset.py — create a shadow EuRoC dataset with gyro axes scaled.

Used to test the unmodeled-gyro-scale-factor hypothesis (calib_imu_intrinsics
is false on this branch, Tw = I, so a BMI055-class sensitivity error of up to
+/-1 % is invisible to the filter and integrates as yaw drift proportional to
heading turned: 1 % = 3.6 deg per lap).

If scaling w_z by (1+s) for some small s flattens the per-lap yaw drift in
yaw_drift_forensics.py, the true scale error is approximately -s.

cam0/cam1 are symlinked (falls back to a Windows junction under WSL on /mnt
drives); only imu0/data.csv is rewritten. Gyro columns of the EuRoC csv are
1..3 (w_x w_y w_z), accel 4..6 — untouched.

Example:
  python3 analysis/scale_gyro_dataset.py \
      --src /mnt/c/.../d455_20260527_090549 \
      --dst /mnt/c/.../d455_20260527_090549_wz_p0p5pct \
      --scale-wz 1.005
"""
import argparse
import os
import subprocess
import sys


def link_dir(src, dst):
    try:
        os.symlink(src, dst)
        return 'symlink'
    except OSError:
        pass
    # WSL on DrvFs: symlinks may be refused; a Windows junction works without
    # elevation and is visible from both sides.
    try:
        wsrc = subprocess.check_output(['wslpath', '-w', src]).decode().strip()
        wdst = subprocess.check_output(['wslpath', '-w', dst]).decode().strip()
        subprocess.check_call(['cmd.exe', '/c', 'mklink', '/J', wdst, wsrc],
                              stdout=subprocess.DEVNULL)
        return 'junction'
    except Exception as e:
        raise SystemExit(f'could not link {src} -> {dst} ({e}); '
                         f'copy the camera folder manually instead')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='dataset root containing imu0/ cam0/')
    ap.add_argument('--dst', required=True)
    ap.add_argument('--scale-wx', type=float, default=1.0)
    ap.add_argument('--scale-wy', type=float, default=1.0)
    ap.add_argument('--scale-wz', type=float, default=1.0)
    args = ap.parse_args()

    if os.path.exists(args.dst):
        raise SystemExit(f'{args.dst} already exists, refusing to overwrite')
    src_imu = os.path.join(args.src, 'imu0', 'data.csv')
    if not os.path.isfile(src_imu):
        raise SystemExit(f'{src_imu} not found')

    os.makedirs(os.path.join(args.dst, 'imu0'))
    for cam in ('cam0', 'cam1'):
        s = os.path.join(args.src, cam)
        if os.path.isdir(s):
            how = link_dir(s, os.path.join(args.dst, cam))
            print(f'{cam}: linked ({how})')

    scales = [args.scale_wx, args.scale_wy, args.scale_wz]
    n = 0
    with open(src_imu) as fin, open(os.path.join(args.dst, 'imu0', 'data.csv'), 'w') as fout:
        for line in fin:
            s = line.strip()
            if not s or s.startswith('#'):
                fout.write(line)
                continue
            c = s.split(',')
            for ax in range(3):
                if scales[ax] != 1.0:
                    c[1 + ax] = f'{float(c[1 + ax]) * scales[ax]:.12g}'
            fout.write(','.join(c) + '\n')
            n += 1
    print(f'imu0/data.csv: {n} samples, scales w=({scales[0]}, {scales[1]}, {scales[2]})')
    with open(os.path.join(args.dst, 'PERTURBATION.txt'), 'w') as f:
        f.write(f'gyro scales wx={scales[0]} wy={scales[1]} wz={scales[2]} '
                f'from {args.src}\n')


if __name__ == '__main__':
    main()
