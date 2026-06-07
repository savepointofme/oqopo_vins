# Baseline Mode Cleanup Summary — 2026-06-06

This document records the cleanup of mode aliases, retired modes, and official designation
changes made on 2026-06-06 as part of the OC variant validation campaign.

---

## What Changed

### 1. Disabled ambiguous/wrong labels (CLI exits with error)

The following `--vio-yaw-gauge-mode` aliases now exit immediately with an error message,
because their meaning was either ambiguous or their implementation was never verified:

| Old alias | Reason disabled |
|-----------|----------------|
| `msckf2` | Applied to oc_prechi2_fej_gauge without proving it matched Li & Mourikis IJRR 2013 |
| `msckf2_0` | Same |
| `msckf2_pure` | Was used as alias for openvins_fej (pure FEJ) without documentation |
| `dso` | Was used for two different (both wrong) implementations, now redirected to error |
| `vins_nullspace` | Never implemented correctly (n_zeroed=0) |
| `h_proj` | Superseded by oc_postchi2_current_gauge |
| `constrained_yaw_nullspace` | Smoke test failed 2026-06-06 (P_n exhausted) |

These changes prevent silent confusion where a user runs `--vio-yaw-gauge-mode msckf2`
believing they are running the Li & Mourikis reference, when the code did something else.

### 2. Added descriptive mode aliases (CLI accepted)

New primary aliases map to unambiguous descriptions of what the code does:

| New alias | Old alias | What it does |
|-----------|-----------|-------------|
| `openvins_fej` | `original` | Pure OpenVINS FEJ, no OC projection |
| `oc_postchi2_current_gauge` | `baseline`, `r1` | H-projection post-chi2, current-gauge |
| `oc_prechi2_fej_gauge` | `oc_prechi2` | H-projection pre-chi2, FEJ-gauge |
| `oc_4d_prechi2_fej_gauge` | `oc_4d` | 4-D H-projection pre-chi2, FEJ-gauge |
| `oc_postchi2_fej_gauge` | `oc_legacy_fej` | H-projection post-chi2, FEJ-gauge |

Old aliases (`original`, `baseline`, `oc_prechi2`, etc.) are still accepted as secondary
aliases for backward compatibility.

### 3. Four-mode OC validation (fly3)

A controlled four-mode comparison on fly3 was run to determine `official_oc`.

All modes used identical conditions:
- Same config, GPS, FC init, gps-time-offset, fi1e4 condition number
- Matching GPS-Z fusion flags
- Result dirs named `fly3_[MODE]_gpsz_fi1e4_fcinit/`

See `fly3_oc_variant_validation_20260606.md` for full results.

---

## What Was NOT Changed

- GPS-Z fusion flags and logic: unchanged
- Height aiding arch: unchanged
- Feature settings (fi_max_cond_number=1e4): unchanged
- Mode B (`gpsz_oc2_1e4_fly3`) reference: not overwritten
- Three-condition package `three_condition_comparison_package_20260605_1727.zip`: preserved
- `traj_1e4_fly34_20260606.zip` and `successful_runs_20260606.zip`: preserved

---

## Official Baseline Status

**openvins_fej** remains the required reference baseline for all cross-condition comparisons.

**official_oc**: to be designated after fly3 validation completes.
See `fly3_oc_variant_validation_20260606.md` and `official_yaw_modes_20260606.md`.

---

## Files Modified (2026-06-06)

| File | Change |
|------|--------|
| `ov_msckf/src/core/VioManager.cpp` | Added descriptive alias dispatch; retired mode exits |
| `ov_msckf/src/state/StateHelper.h` | VisualYawUpdateMode enum annotations |
| `official_yaw_modes_20260606.md` | Mode registry with status column |
| `orthodox_oc_audit_20260606.md` | Full OC audit with Fly3 TBD fields |
| `fly3_oc_variant_validation_20260606.md` | Validation document (created 2026-06-07) |
| `baseline_mode_cleanup_summary_20260606.md` | This file |
