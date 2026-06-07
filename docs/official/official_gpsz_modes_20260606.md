# Official GPS-Z Fusion Modes — 2026-06-06

---

## Official GPS-Z Mode Table

| # | Name | CLI Flags | Code Path | Recommended | Used in Reports |
|---|---|---|---|---|---|
| 1 | **GPS-Z OFF** | *(no `--gps-alt-update` flag)* | `feed_measurement_gps_altitude` never called | Baseline / reference | YES — Cond1 no-GPS-Z runs |
| 2 | **Guarded GPS-Z ON** | See "Baseline flag set" below | `VioManager::feed_measurement_gps_altitude`, guarded path | **YES — primary fusion** | YES — all Cond2/Cond3 runs, all 1e4 comparisons |

---

## Official GPS-Z Baseline Flag Set

Every production run that uses GPS-Z must use exactly this flag set (no deviation without
documenting in the experiment README):

```bash
--gps-alt-update \
--gps-alt-sigma 2.0 \
--gps-alt-min-pzz 0.01 \
--gps-alt-max-res 80 \
--gps-alt-min-t-after-init 10 \
--gps-alt-guard-dxy 0.5 \
--gps-alt-guard-kxy 5.0
```

### Parameter meanings

| Flag | Value | Meaning |
|------|-------|---------|
| `--gps-alt-sigma` | 2.0 | GPS altitude measurement noise (m) |
| `--gps-alt-min-pzz` | 0.01 | Floor on P_zz before K computation (prevents over-confident updates) |
| `--gps-alt-max-res` | 80 | Residual gate: skip if |z_gps − z_pred| > 80 m |
| `--gps-alt-min-t-after-init` | 10 | Bootstrap delay: no GPS-Z for 10 s after VIO init |
| `--gps-alt-guard-dxy` | 0.5 | Reject update if predicted XY displacement |dXY| > 0.5 m |
| `--gps-alt-guard-kxy` | 5.0 | Reject update if |K_xy| / |K_pz| > 5.0 (lateral gain too large) |

---

## Guarded GPS-Z — Code Path

**Entry point**: `VioManager::feed_measurement_gps_altitude` (VioManager.cpp, line ~359)

**Flow**:
1. Bootstrap: first call after `min_t_after_init` anchors `gps_z_ground_` and `vio_z_ref_`
2. Predict: `z_pred = p_IinG(2)` (absolute) or relative offset
3. Guard checks: dXY guard, K_xy/K_pz guard, residual gate
4. EKF measurement update: scalar `h(x) = p_z`, standard Kalman gain
5. State updated; `P` modified via standard Joseph form

**Output stats**: `[GPS-ALT-STAT]` logged every 30 s.

---

## Retired / Rejected GPS-Z Variants

### Architecture G (gps_alt_arch_g_)

**Status: REJECTED**

Augments state with a scalar `h_offset` element (GPS-VIO altitude bias).
Measurement model: `GPS_z = p_z + h_offset`.
Enabled via `set_gps_alt_arch_g(true)` + `use_gps_h_offset: true` in YAML.

**Rejection reason** (from project_height_aid_arch_g_20260604 memory):
- Full fly1 result: 292 m RMSE vs baseline 216 m (R1, start_yaw)
- Introduced 113° yaw spike — covariance cross-coupling from h_offset state
- Net: 35% worse than baseline

**Code location**: `VioManager.cpp` line ~463, guarded by `gps_alt_arch_g_` flag.
Code path is NOT removed (preserve for reference / future laser-ranging).
Must not be enabled in official runs.

### GPS-Z Joseph partial variants

**Status: REJECTED**

Various experiments with masked Joseph-form updates, partial covariance updates,
and sigma-inflation strategies. None outperformed the guarded baseline.
These are not in the main code path.

### Temporary ground-plane pseudo-GPS

**Status: EXPERIMENTAL_ONLY**

`UpdaterGroundPlaneRange` and `UpdaterGroundPlaneFeature` implement pseudo-rangefinder
altitude updates using visual feature depth.  
These are **not** the GPS-Z baseline and must be kept separately labelled.

---

## Rangefinder / Laser Candidate (future work)

`UpdaterGroundPlaneRange` is the intended entry point for a future laser rangefinder
update. It is NOT GPS-Z baseline fusion. It must be activated separately and labelled
`rangefinder` in experiment logs.

Files:
- `ov_msckf/src/update/UpdaterGroundPlaneRange.cpp/.h`
- `ov_msckf/src/update/UpdaterGroundPlaneFeature.cpp/.h`
- `ov_msckf/src/update/UpdaterGroundPlaneFeatureV1.cpp/.h`
- `ov_msckf/src/core/TerrainEstimator1D.h`

---

## Known Limitations of Official GPS-Z Baseline

1. **Fly2 FEJ divergence**: With `original_fej` mode, GPS-Z triggers divergence at t≈1858 s.
   OC modes do not show this crash. Root cause not yet fully characterized.
2. **Fly4 yaw drift in late stage**: Both FEJ and OC show large yaw drift after the 2nd half.
   GPS-Z altitude correction alone does not stabilize yaw.
3. **Guard thresholds are empirical**: `dxy=0.5`, `kxy=5.0`, `max_res=80` were tuned on fly3.
   May need adjustment on other platforms.
