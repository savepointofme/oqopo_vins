#!/usr/bin/env python3
"""Tile the four gps_xy_overlay.png panels into one figure."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

PLOTS = [
    ("/mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/result/eval_fej_oc_vs_globaloc_start930/gps_xy_overlay.png",
     "Fly 1  (t=930→1744 s)   A=global_oc_alpha1  B=FEJ-OC-prechi2"),
    ("/mnt/c/Users/baloney/Desktop/20260518_gsmq_d455_fly2/result/fc_rebuild_20260525/eval_fej_oc_vs_globaloc_start700_until1350/gps_xy_overlay.png",
     "Fly 2  (t=700→1350 s)   A=global_oc_alpha1  B=FEJ-OC-prechi2"),
    ("/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/result/fc_rebuild_20260527/eval_fej_oc_vs_globaloc_start618_until1600/gps_xy_overlay.png",
     "Fly 3  (t=618→1600 s)   A=global_oc_alpha1  B=FEJ-OC-prechi2"),
    ("/mnt/c/Users/baloney/Desktop/20260528_gsmq_d455_fly4/result/fc_rebuild_20260528/eval_fej_oc_vs_globaloc_start924p4/gps_xy_overlay.png",
     "Fly 4  (t=924→2816 s)   A=global_oc_alpha1  B=FEJ-OC-prechi2"),
]

fig, axes = plt.subplots(2, 2, figsize=(18, 14))
fig.suptitle("XY Trajectory — A=global_oc_alpha1  vs  B=global_yaw_oc_fej_prechi2\n"
             "(start+yaw-only alignment to GPS)",
             fontsize=13, fontweight="bold", y=1.00)

for ax, (path, title) in zip(axes.flat, PLOTS):
    try:
        img = mpimg.imread(path)
        ax.imshow(img)
        ax.set_title(title, fontsize=9, pad=3)
    except FileNotFoundError:
        ax.text(0.5, 0.5, f"MISSING\n{path}", ha="center", va="center",
                transform=ax.transAxes, fontsize=8, color="red")
    ax.axis("off")

plt.tight_layout(rect=[0, 0, 1, 0.97])
OUT = "/mnt/c/Users/baloney/Desktop/traj_xy_all_flights.png"
plt.savefig(OUT, dpi=130, bbox_inches="tight")
print(f"Saved: {OUT}")
