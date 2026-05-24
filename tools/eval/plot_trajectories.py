"""
PR20 GPS Altitude Fusion — Trajectory Visualization
Plots XY overlay, altitude vs time, 3D trajectory, and error vs distance
for each flight comparing Reference / PR17 baseline / PR20 NEW.
"""
import numpy as np
import os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D

BASE = "/mnt/d/vscode_dir/open_vins"
OUT_DIR = f"{BASE}/pr20_final_package/plots"
os.makedirs(OUT_DIR, exist_ok=True)

CUTOFFS = {"fly1": 780, "fly3": 400, "fly4": 580}
SAVE_DPI = 180

# Color scheme
COLORS = {
    'Reference (stereo)': '#2ca02c',      # green
    'PR17 baseline (mono)': '#d62728',    # red
    'PR20 NEW (mono+GPS)': '#1f77b4',     # blue
}

CONFIG = {
    'fly1': {
        'ref': f"{BASE}/20260509_fly1/reference/one_traj_estimate_stereo.txt",
        'baseline': f"{BASE}/20260509_fly1/result/pr17_mono_rerun_onlinecalib_tum.txt",
        'pr20': f"{BASE}/20260509_fly1/result/pr20_fixed_abs_fly1.txt",
        'title': 'fly1 (cutoff=780s)',
    },
    'fly3': {
        'ref': f"{BASE}/20260509_fly3/reference/three_traj_estimate_stereo.txt",
        'baseline': f"{BASE}/20260509_fly3/result/pr17_mono_tum.txt",
        'pr20': f"{BASE}/20260509_fly3/result/pr20_fixed_rel_fly3.txt",
        'title': 'fly3 (cutoff=400s)',
    },
    'fly4': {
        'ref': f"{BASE}/20260509_fly4/reference/four_traj_estimate_stereo.txt",
        'baseline': f"{BASE}/20260509_fly4/result/pr17_mono_tum.txt",
        'pr20': f"{BASE}/20260509_fly4/result/pr20_fixed_abs_fly4.txt",
        'title': 'fly4 (cutoff=580s)',
    },
}


def load_tum(path):
    if not os.path.exists(path): return None
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            v = list(map(float, line.split()))
            if len(v) >= 8: rows.append(v[:8])
    return np.array(rows)


def interp_ref(ref_t, ref_xyz, query_t):
    """Interpolate reference trajectory at query times."""
    interp = []
    valid = []
    for i, t in enumerate(query_t):
        idx = np.searchsorted(ref_t, t)
        if idx == 0 or idx >= len(ref_t): continue
        a = (t - ref_t[idx-1]) / (ref_t[idx] - ref_t[idx-1]) if ref_t[idx] > ref_t[idx-1] else 0
        interp.append((1-a)*ref_xyz[idx-1] + a*ref_xyz[idx])
        valid.append(i)
    return np.array(interp), valid


def align_to_ref(src_t, src_xyz, ref_t, ref_xyz):
    """Align src to ref: interpolate ref at src times, then compute SE3 transform.
    Returns aligned src_xyz and the interpolated ref_xyz (matched pairs)."""
    ref_interp, valid = interp_ref(ref_t, ref_xyz, src_t)
    src_matched = src_xyz[valid]
    if len(src_matched) < 10:
        return None, None
    mu_src = src_matched.mean(0); mu_ref = ref_interp.mean(0)
    A = (src_matched - mu_src).T @ (ref_interp - mu_ref)
    U, _, Vt = np.linalg.svd(A)
    d = np.linalg.det(Vt.T @ U.T)
    S = np.diag([1, 1, d])
    R = Vt.T @ S @ U.T
    t = mu_ref - R @ mu_src
    aligned_all = (R @ src_xyz.T).T + t
    return aligned_all, ref_interp


def apply_cutoff(traj, cutoff):
    """Truncate trajectory to first `cutoff` seconds."""
    t0 = traj[0, 0]
    mask = (traj[:, 0] - t0) <= cutoff
    return traj[mask]


def plot_xy_overlay(ax, ref, baseline, pr20, fly, cutoff):
    """Top-down XY trajectory overlay."""
    ref_c = apply_cutoff(ref, cutoff)
    bl_c = apply_cutoff(baseline, cutoff)
    pr_c = apply_cutoff(pr20, cutoff)

    # Align baseline and pr20 to reference
    bl_aligned, _ = align_to_ref(bl_c[:, 0], bl_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])
    pr_aligned, _ = align_to_ref(pr_c[:, 0], pr_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])

    if bl_aligned is None or pr_aligned is None:
        ax.text(0.5, 0.5, 'Alignment failed', transform=ax.transAxes, ha='center')
        return

    ax.plot(ref_c[:, 1], ref_c[:, 2], color=COLORS['Reference (stereo)'], lw=1.8, ls='-', alpha=0.85, label='Ref (stereo)')
    ax.plot(bl_aligned[:, 0], bl_aligned[:, 1], color=COLORS['PR17 baseline (mono)'], lw=1.2, ls='--', alpha=0.75, label='PR17 baseline')
    ax.plot(pr_aligned[:, 0], pr_aligned[:, 1], color=COLORS['PR20 NEW (mono+GPS)'], lw=1.2, ls='-', alpha=0.85, label='PR20 NEW')

    # Mark start points
    ax.scatter([ref_c[0, 1]], [ref_c[0, 2]], color=COLORS['Reference (stereo)'], s=50, marker='o', zorder=5, edgecolors='black', linewidths=0.5)
    ax.scatter([bl_aligned[0, 0]], [bl_aligned[0, 1]], color=COLORS['PR17 baseline (mono)'], s=30, marker='s', zorder=5)
    ax.scatter([pr_aligned[0, 0]], [pr_aligned[0, 1]], color=COLORS['PR20 NEW (mono+GPS)'], s=30, marker='^', zorder=5)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(f'{fly} — XY Trajectory (top-down)', fontsize=11)
    ax.grid(True, alpha=0.25)
    ax.set_aspect('equal', adjustable='box')
    ax.legend(fontsize=7.5, loc='upper right')


def plot_altitude_time(ax, ref, baseline, pr20, fly, cutoff):
    """Altitude vs time (aligned to reference)."""
    ref_c = apply_cutoff(ref, cutoff)
    bl_c = apply_cutoff(baseline, cutoff)
    pr_c = apply_cutoff(pr20, cutoff)

    bl_aligned, _ = align_to_ref(bl_c[:, 0], bl_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])
    pr_aligned, _ = align_to_ref(pr_c[:, 0], pr_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])

    if bl_aligned is None or pr_aligned is None:
        ax.text(0.5, 0.5, 'Alignment failed', transform=ax.transAxes, ha='center')
        return

    t_ref = ref_c[:, 0] - ref_c[0, 0]
    t_bl = bl_c[:, 0] - bl_c[0, 0]
    t_pr = pr_c[:, 0] - pr_c[0, 0]

    ax.plot(t_ref, ref_c[:, 3], color=COLORS['Reference (stereo)'], lw=1.8, ls='-', alpha=0.85, label='Ref (stereo)')
    ax.plot(t_bl, bl_aligned[:, 2], color=COLORS['PR17 baseline (mono)'], lw=1.2, ls='--', alpha=0.75, label='PR17 baseline')
    ax.plot(t_pr, pr_aligned[:, 2], color=COLORS['PR20 NEW (mono+GPS)'], lw=1.2, ls='-', alpha=0.85, label='PR20 NEW')

    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Z [m]')
    ax.set_title(f'{fly} — Altitude vs Time', fontsize=11)
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=7.5, loc='upper left')


def plot_3d_trajectory(ax, ref, baseline, pr20, fly, cutoff):
    """3D trajectory view."""
    ref_c = apply_cutoff(ref, cutoff)
    bl_c = apply_cutoff(baseline, cutoff)
    pr_c = apply_cutoff(pr20, cutoff)

    bl_aligned, _ = align_to_ref(bl_c[:, 0], bl_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])
    pr_aligned, _ = align_to_ref(pr_c[:, 0], pr_c[:, 1:4], ref_c[:, 0], ref_c[:, 1:4])

    if bl_aligned is None or pr_aligned is None:
        ax.text2D(0.5, 0.5, 'Alignment failed', transform=ax.transAxes, ha='center')
        return

    # Subsample for 3D plot clarity
    step = max(1, len(ref_c) // 3000)
    ax.plot(ref_c[::step, 1], ref_c[::step, 2], ref_c[::step, 3],
            color=COLORS['Reference (stereo)'], lw=1.5, ls='-', alpha=0.8, label='Ref')
    ax.plot(bl_aligned[::step, 0], bl_aligned[::step, 1], bl_aligned[::step, 2],
            color=COLORS['PR17 baseline (mono)'], lw=1.0, ls='--', alpha=0.7, label='Baseline')
    ax.plot(pr_aligned[::step, 0], pr_aligned[::step, 1], pr_aligned[::step, 2],
            color=COLORS['PR20 NEW (mono+GPS)'], lw=1.0, ls='-', alpha=0.85, label='PR20')

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title(f'{fly} — 3D Trajectory', fontsize=11)
    ax.legend(fontsize=7, loc='upper right')


def plot_error_vs_distance(ax_xy, ax_z, ax_ate, ref, baseline, pr20, fly, cutoff):
    """XY error, Z error, and ATE vs distance traveled."""
    ref_c = apply_cutoff(ref, cutoff)
    bl_c = apply_cutoff(baseline, cutoff)
    pr_c = apply_cutoff(pr20, cutoff)

    # Compute distance along reference
    ref_xyz = ref_c[:, 1:4]
    d_ref = np.zeros(len(ref_c))
    for i in range(1, len(ref_c)):
        d_ref[i] = d_ref[i-1] + np.linalg.norm(ref_xyz[i] - ref_xyz[i-1])

    # Align estimate to reference, compute matched errors
    def compute_errors(est, ref_full, d_full):
        aligned, ref_interp = align_to_ref(est[:, 0], est[:, 1:4], ref_full[:, 0], ref_full[:, 1:4])
        if aligned is None: return None, None, None, None, None
        _, valid = interp_ref(ref_full[:, 0], ref_full[:, 1:4], est[:, 0])
        d_interp = np.interp(est[valid, 0], ref_full[:, 0], d_full)
        err = aligned[valid] - ref_interp
        xy_err = np.sqrt((err[:, :2]**2).sum(1))
        z_err = err[:, 2]
        ate = np.sqrt((err**2).sum(1))
        return d_interp, xy_err, z_err, ate, est[valid, 0] - est[0, 0]

    d_bl, xy_bl, z_bl, ate_bl, t_bl = compute_errors(bl_c, ref_c, d_ref)
    d_pr, xy_pr, z_pr, ate_pr, t_pr = compute_errors(pr_c, ref_c, d_ref)

    if d_bl is not None:
        ax_xy.plot(d_bl, xy_bl, color=COLORS['PR17 baseline (mono)'], lw=0.6, alpha=0.6, label='PR17 baseline')
        ax_z.plot(d_bl, np.abs(z_bl), color=COLORS['PR17 baseline (mono)'], lw=0.6, alpha=0.6, label='PR17 baseline')
        ax_ate.plot(d_bl, ate_bl, color=COLORS['PR17 baseline (mono)'], lw=0.6, alpha=0.6, label='PR17 baseline')

    if d_pr is not None:
        ax_xy.plot(d_pr, xy_pr, color=COLORS['PR20 NEW (mono+GPS)'], lw=0.8, alpha=0.85, label='PR20 NEW')
        ax_z.plot(d_pr, np.abs(z_pr), color=COLORS['PR20 NEW (mono+GPS)'], lw=0.8, alpha=0.85, label='PR20 NEW')
        ax_ate.plot(d_pr, ate_pr, color=COLORS['PR20 NEW (mono+GPS)'], lw=0.8, alpha=0.85, label='PR20 NEW')

    ax_xy.set_xlabel('Distance along reference [m]')
    ax_xy.set_ylabel('XY Error [m]')
    ax_xy.set_title(f'{fly} — XY Error vs Distance', fontsize=10)
    ax_xy.grid(True, alpha=0.25)
    ax_xy.legend(fontsize=7)

    ax_z.set_xlabel('Distance along reference [m]')
    ax_z.set_ylabel('|Z Error| [m]')
    ax_z.set_title(f'{fly} — |Z| Error vs Distance', fontsize=10)
    ax_z.grid(True, alpha=0.25)
    ax_z.legend(fontsize=7)

    ax_ate.set_xlabel('Distance along reference [m]')
    ax_ate.set_ylabel('ATE [m]')
    ax_ate.set_title(f'{fly} — ATE vs Distance', fontsize=10)
    ax_ate.grid(True, alpha=0.25)
    ax_ate.legend(fontsize=7)


def make_fly_figure(fly, cfg, cutoff):
    """Create a comprehensive 2x3 figure for one flight."""
    print(f"  Loading {fly}...")
    ref = load_tum(cfg['ref'])
    baseline = load_tum(cfg['baseline'])
    pr20 = load_tum(cfg['pr20'])

    if ref is None or baseline is None or pr20 is None:
        print(f"  SKIP {fly}: missing files")
        return

    print(f"  Ref={len(ref)} poses, Baseline={len(baseline)} poses, PR20={len(pr20)} poses")

    fig = plt.figure(figsize=(20, 14))
    fig.suptitle(cfg['title'], fontsize=14, fontweight='bold', y=0.98)

    # Row 1: XY overlay + Altitude vs time + 3D
    ax_xy = fig.add_subplot(2, 3, 1)
    plot_xy_overlay(ax_xy, ref, baseline, pr20, fly, cutoff)

    ax_alt = fig.add_subplot(2, 3, 2)
    plot_altitude_time(ax_alt, ref, baseline, pr20, fly, cutoff)

    ax_3d = fig.add_subplot(2, 3, 3, projection='3d')
    plot_3d_trajectory(ax_3d, ref, baseline, pr20, fly, cutoff)

    # Row 2: Error vs distance
    ax_xy_err = fig.add_subplot(2, 3, 4)
    ax_z_err = fig.add_subplot(2, 3, 5)
    ax_ate = fig.add_subplot(2, 3, 6)
    plot_error_vs_distance(ax_xy_err, ax_z_err, ax_ate, ref, baseline, pr20, fly, cutoff)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = f"{OUT_DIR}/{fly}_trajectory_comparison.png"
    fig.savefig(out_path, dpi=SAVE_DPI, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {out_path}")


def make_summary_table():
    """Generate a summary table plot."""
    fig, ax = plt.subplots(figsize=(12, 3))
    ax.axis('off')

    data = [
        ['fly1', '780s', '26.20', '19.42', '58.03 (+121%)', '8.47 (-56%)', '44.09 (+68%)', '24.96 (+28%)'],
        ['fly3', '400s', '54.23', '5.17', '42.58 (-21%)', '1.41 (-73%)', '52.59 (-3%)', '1.36 (-74%)'],
        ['fly4', '580s', '24.95', '3.44', '30.89 (+24%)', '2.20 (-36%)', '21.32 (-15%)', '2.42 (-30%)'],
    ]
    columns = ['Fly', 'Cutoff', 'BL XY', 'BL Z', 'OLD XY', 'OLD Z', 'NEW XY', 'NEW Z']
    col_widths = [0.06, 0.08, 0.10, 0.10, 0.17, 0.17, 0.17, 0.17]

    table = ax.table(cellText=data, colLabels=columns, colWidths=col_widths,
                     cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.5)

    # Color header
    for j in range(len(columns)):
        table[0, j].set_facecolor('#404040')
        table[0, j].set_text_props(color='white', fontweight='bold')

    ax.set_title('PR20 GPS Altitude Fusion — RMSE Comparison (SE3 aligned, effective segment)',
                 fontsize=12, fontweight='bold', pad=20)

    out_path = f"{OUT_DIR}/summary_table.png"
    fig.savefig(out_path, dpi=SAVE_DPI, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {out_path}")


def main():
    print("=" * 60)
    print("PR20 Trajectory Visualization")
    print("=" * 60)

    for fly in ['fly1', 'fly3', 'fly4']:
        cutoff = CUTOFFS[fly]
        make_fly_figure(fly, CONFIG[fly], cutoff)

    make_summary_table()

    print(f"\nAll plots saved to: {OUT_DIR}")
    print("Done.")


if __name__ == '__main__':
    main()
