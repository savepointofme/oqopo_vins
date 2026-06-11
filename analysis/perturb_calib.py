#!/usr/bin/env python3
"""
perturb_calib.py — create a perturbed copy of a flight config dir for
cam-IMU sensitivity runs (time offset, extrinsic rotation).

Edits kalibr_imucam_chain.yaml in the copy:
  --toff-ms X         add X milliseconds to timeshift_cam_imu
  --ext-yaw-deg Y     premultiply T_cam_imu by Rz(Y): rotates the camera frame
                      about its optical axis (= world yaw for a nadir camera)
  --ext-roll-deg R    premultiply T_cam_imu by Rx(R): tilts the camera about
                      the camera x-axis (lateral tilt / bore-sight roll)
  --ext-pitch-deg P   premultiply T_cam_imu by Ry(P): tilts the camera about
                      the camera y-axis (longitudinal tilt / bore-sight pitch)
Optionally freezes online calibration in estimator_config.yaml so the filter
cannot re-absorb the injected perturbation (otherwise the run measures
observability of the parameter instead of sensitivity to it):
  --freeze toff,ext,int

Files are OpenCV %YAML:1.0 — edited line-based on purpose (pyyaml chokes on
the directive), everything else copied verbatim.

Example:
  python3 analysis/perturb_calib.py --cfg-dir config/d455_fly2 \
      --out-dir /tmp/cfg_fly4_toff_p10ms --toff-ms 10 --freeze toff
"""
import argparse
import math
import os
import re
import shutil


def _parse_T_cam_imu(lines):
    """Return (T, row_idx) where T is 4x4 list-of-lists and row_idx are line numbers."""
    for i, ln in enumerate(lines):
        if re.match(r'^\s*T_cam_imu:\s*$', ln):
            break
    else:
        raise SystemExit('T_cam_imu not found in yaml')
    rows, row_idx = [], []
    j = i + 1
    while j < len(lines) and len(rows) < 4:
        nums = re.findall(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?', lines[j])
        if lines[j].lstrip().startswith('-') and len(nums) == 4:
            rows.append([float(x) for x in nums])
            row_idx.append(j)
        elif lines[j].strip() and not lines[j].lstrip().startswith('#'):
            break
        j += 1
    if len(rows) != 4:
        raise SystemExit('could not parse 4x4 T_cam_imu')
    T = [[rows[r][c] for c in range(4)] for r in range(4)]
    return T, row_idx


def _write_T_cam_imu(lines, T, row_idx):
    indent = lines[row_idx[0]][:len(lines[row_idx[0]]) - len(lines[row_idx[0]].lstrip())]
    for r in range(4):
        vals = ', '.join(f'{v: .9f}' for v in T[r])
        lines[row_idx[r]] = f'{indent}- [{vals}]'


def _matmul3x4(R3, T):
    """Premultiply 3x3 R3 into the upper-left 3x3 of 4x4 T; translation column included."""
    Tn = [[sum(R3[r][k] * T[k][col] for k in range(3)) for col in range(4)]
          for r in range(3)] + [T[3]]
    return Tn


def edit_imucam(path, toff_ms, ext_yaw_deg, ext_roll_deg=None, ext_pitch_deg=None):
    lines = open(path).read().splitlines()
    changed = []

    if toff_ms is not None:
        for i, ln in enumerate(lines):
            mm = re.match(r'^(\s*timeshift_cam_imu:\s*)([-\d.eE+]+)\s*$', ln)
            if mm:
                old = float(mm.group(2))
                new = old + toff_ms * 1e-3
                lines[i] = f'{mm.group(1)}{new:.9f}'
                changed.append(f'timeshift_cam_imu: {old:.6f} -> {new:.6f} s '
                               f'({toff_ms:+.1f} ms)')
                break
        else:
            raise SystemExit(f'timeshift_cam_imu not found in {path}')

    # Collect which rotation(s) to apply; apply in order yaw → pitch → roll
    rotations = []
    if ext_yaw_deg is not None:
        th = math.radians(ext_yaw_deg)
        c, s = math.cos(th), math.sin(th)
        rotations.append(([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
                          f'Rz({ext_yaw_deg:+.3f} deg) [optical-axis / yaw]'))
    if ext_pitch_deg is not None:
        th = math.radians(ext_pitch_deg)
        c, s = math.cos(th), math.sin(th)
        rotations.append(([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
                          f'Ry({ext_pitch_deg:+.3f} deg) [camera-y / pitch]'))
    if ext_roll_deg is not None:
        th = math.radians(ext_roll_deg)
        c, s = math.cos(th), math.sin(th)
        rotations.append(([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
                          f'Rx({ext_roll_deg:+.3f} deg) [camera-x / roll]'))

    if rotations:
        T, row_idx = _parse_T_cam_imu(lines)
        for R, label in rotations:
            T = _matmul3x4(R, T)
            changed.append(f'T_cam_imu: premultiplied {label}')
        _write_T_cam_imu(lines, T, row_idx)

    open(path, 'w').write('\n'.join(lines) + '\n')
    return changed


def freeze_estimator(path, freeze):
    key_map = {'toff': 'calib_cam_timeoffset',
               'ext': 'calib_cam_extrinsics',
               'int': 'calib_cam_intrinsics'}
    lines = open(path).read().splitlines()
    changed = []
    for tag in freeze:
        key = key_map[tag]
        for i, ln in enumerate(lines):
            if re.match(rf'^{key}:', ln):
                lines[i] = f'{key}: false'
                changed.append(f'{key}: false (online calib frozen)')
                break
    open(path, 'w').write('\n'.join(lines) + '\n')
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cfg-dir', required=True,
                    help='source config dir (estimator_config.yaml + kalibr yamls)')
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--toff-ms', type=float, default=None)
    ap.add_argument('--ext-yaw-deg', type=float, default=None,
                    help='rotate camera about optical axis (deg)')
    ap.add_argument('--ext-roll-deg', type=float, default=None,
                    help='rotate camera about camera-x axis, bore-sight roll (deg)')
    ap.add_argument('--ext-pitch-deg', type=float, default=None,
                    help='rotate camera about camera-y axis, bore-sight pitch (deg)')
    ap.add_argument('--freeze', default='',
                    help='comma list of toff,ext,int to set calib_cam_* false')
    args = ap.parse_args()

    any_extrinsic = (args.ext_yaw_deg is not None or args.ext_roll_deg is not None
                     or args.ext_pitch_deg is not None)
    if args.toff_ms is None and not any_extrinsic and not args.freeze:
        raise SystemExit('nothing to do: give --toff-ms, --ext-yaw-deg, '
                         '--ext-roll-deg, and/or --ext-pitch-deg')
    if os.path.exists(args.out_dir):
        raise SystemExit(f'{args.out_dir} already exists, refusing to overwrite')

    shutil.copytree(args.cfg_dir, args.out_dir)
    changed = edit_imucam(os.path.join(args.out_dir, 'kalibr_imucam_chain.yaml'),
                          args.toff_ms, args.ext_yaw_deg,
                          ext_roll_deg=args.ext_roll_deg,
                          ext_pitch_deg=args.ext_pitch_deg)
    freeze = [t.strip() for t in args.freeze.split(',') if t.strip()]
    bad = [t for t in freeze if t not in ('toff', 'ext', 'int')]
    if bad:
        raise SystemExit(f'unknown --freeze tags: {bad}')
    if freeze:
        changed += freeze_estimator(os.path.join(args.out_dir, 'estimator_config.yaml'),
                                    freeze)

    print(f'created {args.out_dir}')
    for c in changed:
        print(f'  {c}')
    with open(os.path.join(args.out_dir, 'PERTURBATION.txt'), 'w') as f:
        f.write('\n'.join(changed) + '\n')


if __name__ == '__main__':
    main()
