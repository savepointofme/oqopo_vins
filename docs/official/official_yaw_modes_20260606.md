# Official Yaw / OC Modes — 2026-06-06 (revised: descriptive names only)

Names are based on what the code actually does, not on paper/system labels.
See `orthodox_oc_audit_20260606.md` for full implementation details.
See `fly3_oc_variant_validation_20260606.md` for Fly3 validation results.

---

## Naming Rule

> Name the mode by what the code actually does, not by what paper or system it vaguely resembles.

Forbidden labels unless implementation is formally audited:
- `msckf2`, `msckf2_0`, `msckf2_pure` — **DISABLED** (exit with error)
- `dso` — **DISABLED** (exit with error)
- `vins_nullspace` — **DISABLED** (exit with error)

---

## Mode Registry

| Name | CLI alias (`--vio-yaw-gauge-mode`) | Mode string | Enum | Gauge | Chi2 | Status |
|------|------------------------------------|-------------|------|-------|------|--------|
| **openvins_fej** | `openvins_fej`, `original`, `fej` | `original` | 0 | — (no OC) | — | REQUIRED BASELINE |
| **oc_postchi2_current_gauge** | `oc_postchi2_current_gauge`, `baseline` | `global_yaw_oc_projection` | 2 | current state | post | **OFFICIAL_OC** (fly3: 323m RMS) |
| **oc_prechi2_fej_gauge** | `oc_prechi2_fej_gauge`, `oc_prechi2` | `global_yaw_oc_fej_prechi2` | prechi2 path | FEJ-frozen | pre | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE (fly3: 638m, worse than A) |
| **oc_4d_prechi2_fej_gauge** | `oc_4d_prechi2_fej_gauge`, `oc_4d` | `visual_4d_oc_fej_prechi2` | prechi2 path | FEJ-frozen | pre | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE (fly3: 646m, worst) |
| **oc_postchi2_fej_gauge** | `oc_postchi2_fej_gauge`, `oc_legacy_fej` | `global_yaw_oc_fej_projection` | 10 | FEJ-frozen | post | ACCESSIBLE (fly1 Cond3 ref) |
| **schmidt_fej** | `schmidt_fej` | `visual_yaw_schmidt_fej_gauge` | 9 | FEJ | — | EXPERIMENTAL_ONLY |
| **schmidt_current** | `schmidt` | `visual_yaw_schmidt_current_gauge` | 6 | current | — | EXPERIMENTAL_ONLY |

---

## Retired / Disabled Modes

| Name | Old alias | Reason |
|------|-----------|--------|
| DSO K-projection v1 | `dso` | CLI exits — smoke failed (altitude diverges t=450s) |
| VINS S-EVD v1 | `vins_nullspace` | CLI exits — n_zeroed=0, no nullspace action |
| Constrained yaw nullspace | `constrained_yaw_nullspace` | CLI exits — P_n exhausted, covariance indefinite |
| H-projection current | `h_proj` | CLI exits — superseded by oc_postchi2_current_gauge |
| Per-block scale | (internal only) | Not accessible via gauge alias |
| Current-only scale | (internal only) | Not accessible via gauge alias |

---

## Official Baseline for Results

**Fly3 validation complete (2026-06-07).**

| Role | Mode | CLI alias | Fly3 RMS |
|------|------|-----------|---------|
| **REQUIRED BASELINE** | openvins_fej | `--vio-yaw-gauge-mode openvins_fej` | 448m |
| **OFFICIAL_OC** | oc_postchi2_current_gauge | `--vio-yaw-gauge-mode oc_postchi2_current_gauge` | 323m |

All future controlled comparisons must include both roles.
Pre-chi2 modes (oc_prechi2_fej_gauge, oc_4d_prechi2_fej_gauge) are accessible but
NOT_RECOMMENDED — they produce worse XY accuracy than no-OC on fly3.

All three-condition and four-flight comparisons published so far used:
- FEJ reference: openvins_fej (mode string `original`, use_fej=true)
- OC reference: oc_postchi2_current_gauge or oc_postchi2_fej_gauge (see artifact manifests)

---

## Required YAML Config for All Controlled Runs

```yaml
use_fej: true
fi_max_cond_number: 10000.0   # standard 1e4 (document clearly if using 1e5)
```
