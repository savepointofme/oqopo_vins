"""Detect first straight edge from truth XY after takeoff settles."""

import numpy as np
from diag_pre_descent import load_truth, TRUTH_CSV

truth = load_truth(TRUTH_CSV)
t0 = truth[0, 0]
t_rel = truth[:, 0] - t0
xy = truth[:, 1:3]
z = truth[:, 3]

# 1) takeoff-done: first time z reaches 0.9 * peak
peak_z = z.max()
takeoff_done_idx = int(np.where(z >= 0.9 * peak_z)[0][0])
t_takeoff = t_rel[takeoff_done_idx]
print(f"Peak z = {peak_z:.2f} m at t_rel = {t_rel[z.argmax()]:.1f}s")
print(f"Takeoff done (z first reaches 0.9 peak): t_rel = {t_takeoff:.1f}s, xy = {xy[takeoff_done_idx]}")

# 2) starting from takeoff-done, compute direction over a small window and
# find when direction stabilises (small angular variation over ~5s) — that's
# the start of the first straight edge.
# Use 1s steps (~ every 8 truth samples since truth is ~50 Hz).
dt_window = 5.0   # seconds
turn_thresh_deg = 8.0
edge_idx_start = None
i = takeoff_done_idx
while i < len(t_rel) - 50:
    # accumulate direction over next dt_window seconds
    end = i
    while end < len(t_rel) and t_rel[end] - t_rel[i] < dt_window:
        end += 1
    if end - i < 5:
        i += 1
        continue
    # compute direction at i and at midpoint and at end
    d_start = xy[i + 2] - xy[i]
    d_end = xy[end - 1] - xy[end - 3]
    n_start = np.linalg.norm(d_start)
    n_end = np.linalg.norm(d_end)
    if n_start < 1e-6 or n_end < 1e-6:
        i += 1
        continue
    cos_a = np.clip((d_start @ d_end) / (n_start * n_end), -1.0, 1.0)
    ang = np.degrees(np.arccos(cos_a))
    if ang < turn_thresh_deg and n_start > 1.0:  # consistent direction
        edge_idx_start = i
        break
    i += 1

if edge_idx_start is None:
    edge_idx_start = takeoff_done_idx
t_edge_start = t_rel[edge_idx_start]
print(f"First-edge start: t_rel = {t_edge_start:.1f}s, xy = {xy[edge_idx_start]}")

# 3) edge ends when the direction changes by > turn_thresh_deg sustained
ref_dir = xy[min(edge_idx_start + 5, len(xy) - 1)] - xy[edge_idx_start]
ref_dir /= np.linalg.norm(ref_dir)
edge_idx_end = None
window_size = 5  # samples for direction smoothing
for j in range(edge_idx_start + window_size, len(xy) - window_size):
    d = xy[j + window_size] - xy[j]
    n = np.linalg.norm(d)
    if n < 0.5:
        continue
    cos_a = np.clip((d @ ref_dir) / n, -1.0, 1.0)
    ang = np.degrees(np.arccos(cos_a))
    if ang > 25.0:  # turning out of first edge
        edge_idx_end = j
        break
if edge_idx_end is None:
    edge_idx_end = len(xy) - 1

t_edge_end = t_rel[edge_idx_end]
print(f"First-edge end:   t_rel = {t_edge_end:.1f}s, xy = {xy[edge_idx_end]}")
print(f"First-edge duration:  {t_edge_end - t_edge_start:.1f}s")

# Truth path length along first edge
edge_xy = xy[edge_idx_start : edge_idx_end + 1]
seg = np.diff(edge_xy, axis=0)
truth_length = float(np.sum(np.linalg.norm(seg, axis=1)))
straight_dist = float(np.linalg.norm(edge_xy[-1] - edge_xy[0]))
print(f"Truth first-edge path length = {truth_length:.2f} m")
print(f"Truth first-edge straight-line distance = {straight_dist:.2f} m")
print(f"Truth first-edge straightness ratio = {straight_dist / truth_length:.3f}  (1.0 = perfectly straight)")
print()
print(f"#### Recommended first-edge window: t_rel in [{t_edge_start:.1f}, {t_edge_end:.1f}] s")
print(f"#### Equivalent absolute t: [{t0 + t_edge_start:.3f}, {t0 + t_edge_end:.3f}]")
