import numpy as np
from diag_pre_descent import load_truth, TRUTH_CSV

truth = load_truth(TRUTH_CSV)
t0 = truth[0, 0]
t_rel = truth[:, 0] - t0
z = truth[:, 3]
print("t_rel  z(m)")
for i in range(0, len(truth), 100):
    print(f"{t_rel[i]:6.1f}  {z[i]:7.2f}")
print("---")
peak = z.max()
print(f"peak z: {peak:.2f} at t_rel = {t_rel[z.argmax()]:.1f}")
print(f"last z:  {z[-1]:.2f} at t_rel = {t_rel[-1]:.1f}")
for frac in [0.95, 0.90, 0.80, 0.70, 0.50]:
    mask = z >= frac * peak
    if mask.any():
        idx = np.where(mask)[0][-1]
        print(f"last sample where z >= {frac:.2f}*peak ({frac * peak:.2f}m): t_rel = {t_rel[idx]:.1f}")
# Plot dz/dt summary
dz = np.gradient(z, truth[:, 0])
for tmin, tmax in [(0, 50), (50, 200), (200, 400), (400, 500), (500, 600), (600, 650)]:
    mask = (t_rel >= tmin) & (t_rel < tmax)
    if mask.any():
        print(f"t in [{tmin},{tmax}): mean dz/dt = {dz[mask].mean():+.3f} m/s, "
              f"min = {dz[mask].min():+.3f}, max = {dz[mask].max():+.3f}")
