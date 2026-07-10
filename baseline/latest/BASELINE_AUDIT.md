# Baseline Audit Notes

## Visual Consistency Modes

The old workspace exposed `use_fej`, `vio_yaw_update_mode`,
`vio_global_yaw_oc_alpha`, and `vio_yaw_gauge_mode` as independent knobs.  That
made it easy to create ambiguous combinations.

The clean runner now uses `--yaw-mode`:

| Mode | FEJ Jacobians | OC Gauge | Projection Timing | Intended Use |
| --- | --- | --- | --- | --- |
| `baseline` | on | current state | post-chi2 | Current best recipe, kept behavior-compatible. |
| `fej` | on | none | none | OpenVINS FEJ-only ablation. |
| `oc` | off | current state | post-chi2 | OC-only ablation; not recommended as baseline. |
| `oc-fej` | on | FEJ state | post-chi2 | Cleaner FEJ+OC ablation. |
| `none` | off | none | none | Non-FEJ original EKF ablation. |

Observed code fact: `global_yaw_oc_projection` is an independent current-gauge
OC projection.  It does not require FEJ, but it is not the same as a
Li-Mourikis style FEJ+OC design.  Therefore poor results from `use_fej=false`
plus current-gauge OC do not prove an implementation bug; they show that this
ablation is not the validated baseline.

## GPS-Z Height Modes

The clean runner now uses `--height-mode`:

| Mode | Low-Level Mode | Intended Use |
| --- | --- | --- |
| `guarded` | `guarded` | Baseline guarded GPS-Z update. |
| `standard` | `full` | Full coupled Joseph update without underweighting. |
| `bounded` | `bounded` | Uniform gain-scale trust-region experiment. |
| `nasa-lean` | `nasa_lean` | Measurement-space NASA underweighting experiment. |

`guarded` and `nasa-lean` are not two different altitude measurements.  They
use the same GPS-Z observation model and differ in how the Kalman correction is
protected.

## Local Cleanup

Large historical experiment material is intentionally outside git:

```text
D:/vscode_dir/open_vins_legacy_quarantine_20260625
```

The repository should publish only source, canonical analysis tooling, and
`baseline/latest`.
