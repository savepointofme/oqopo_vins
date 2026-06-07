# Orthodox OC Audit — 2026-06-06

Audit of every OC variant candidate. Names are descriptive of what the code does,
not of what paper it may resemble. The Fly3 validation results will be filled in
once runs complete.

---

## Candidate Table

| # | Old alias | New descriptive name | Enum | Mode string | Gauge type | Chi2 timing | What gets projected | K/dx/P modified? | Status |
|---|-----------|---------------------|------|-------------|-----------|-------------|---------------------|------------------|--------|
| A | `original` | **openvins_fej** | 0 | `original` | FEJ | No projection | Nothing (pure FEJ) | No | BASELINE_REQUIRED |
| B | `baseline`, `r1` | **oc_postchi2_current_gauge** | 2 | `global_yaw_oc_projection` | current state | post-chi2 | H (yaw column zeroed) | No — H only | **OFFICIAL_OC** (fly3: 323m) |
| C | `oc_prechi2` | **oc_prechi2_fej_gauge** | via prechi2 path | `global_yaw_oc_fej_prechi2` | FEJ-frozen | pre-chi2 | H (yaw column zeroed) | No — H only | EXPERIMENTAL_ONLY (fly3: 638m, worse than A) |
| D | `oc_4d` | **oc_4d_prechi2_fej_gauge** | via prechi2 path | `visual_4d_oc_fej_prechi2` | FEJ-frozen | pre-chi2 | H (4-DOF: yaw + 3D position) | No — H only | EXPERIMENTAL_ONLY (fly3: 646m, worst) |
| E | `oc_legacy_fej` | **oc_postchi2_fej_gauge** | 10 | `global_yaw_oc_fej_projection` | FEJ-frozen | post-chi2 | H (yaw column zeroed) | No — H only | ACCESSIBLE (fly1 Cond3 ref) |

---

## Detailed Mode Descriptions

### A — openvins_fej

**What the code does:**  
Standard MSCKF EKF update. When `use_fej: true` is set in YAML, `UpdaterHelper.cpp` (lines 46,
89-94) selects frozen linearization points (`Rot_fej()`, `pos_fej()`, `p_FinG_fej`) for
constructing H. No explicit H-space yaw projection is applied. The yaw direction remains
observable through the standard visual measurement model.

**K/dx/P modified?** No — standard EKF.

**use_fej required?** Yes (`use_fej: true` in YAML).

**CLI alias:** `--vio-yaw-gauge-mode openvins_fej` (also `original`)

---

### B — oc_postchi2_current_gauge

**What the code does:**  
After the chi2 outlier test passes, `StateHelper::EKFUpdate` is called with the OC projection
flag set. `VisualObservabilityPolicy::project_H_global_yaw_OC` computes the global-yaw gauge
direction `n` from the **current state estimate** (not frozen). The OC projection then
zeroes the yaw-sensitive component of H: `H_eff = H − H·n·nᵀ`. Standard EKF update
proceeds with H_eff.  
Alpha = 1.0 means full projection strength.

**K/dx/P modified?** No — only H is modified before standard EKF.

**use_fej required?** Yes (FEJ still active in H construction for the other state blocks).

**Current gauge freshness:** current state → linearization point updates every step.

**CLI alias:** `--vio-yaw-gauge-mode oc_postchi2_current_gauge` (also `baseline`)

---

### C — oc_prechi2_fej_gauge

**What the code does:**  
Before the chi2 outlier test, `VisualObservabilityPolicy` applies the OC H-projection using
a **FEJ-frozen gauge direction** `n_fej` (from frozen linearization points, not current state).
The projection zeroes the yaw-sensitive component: `H_eff = H − H·n_fej·n_fej^T`.
The chi2 test then uses H_eff. The EKF update at `StateHelper::EKFUpdate` sees the
already-projected H, so `VisualYawUpdateMode::ORIGINAL` is returned by the string dispatch
(the projection has already happened).

**K/dx/P modified?** No — only H, before chi2.

**use_fej required?** Yes (both for H construction and for the gauge `n_fej`).

**Current gauge freshness:** FEJ-frozen → gauge direction does not change after initialization.

**CLI alias:** `--vio-yaw-gauge-mode oc_prechi2_fej_gauge` (also `oc_prechi2`)

---

### D — oc_4d_prechi2_fej_gauge

**What the code does:**  
Same as C but the projection is 4-dimensional: nullspace of **yaw + XYZ position** of a
reference landmark, not just yaw. This is a stronger observability constraint that removes
sensitivity to the full unobservable subspace of yaw + global scale.  
Applied pre-chi2 with FEJ-frozen gauge.

**K/dx/P modified?** No — only H, before chi2.

**use_fej required?** Yes.

**CLI alias:** `--vio-yaw-gauge-mode oc_4d_prechi2_fej_gauge` (also `oc_4d`)

---

### E — oc_postchi2_fej_gauge (accessible, not primary)

**What the code does:**  
Post-chi2 H-projection, same as B, but gauge direction uses **FEJ-frozen** state instead
of current state. This was used for fly1 Cond3 in the three-condition comparison package.
It is the mode that showed 280 m vs 405 m FEJ on fly1.

**K/dx/P modified?** No — only H.

**use_fej required?** Yes.

**CLI alias:** `--vio-yaw-gauge-mode oc_postchi2_fej_gauge` (also `oc_legacy_fej`)

---

## Disabled / Forbidden Labels

The following labels are disabled and will cause CLI exit with error:

| Old label | Reason disabled |
|-----------|----------------|
| `msckf2` | Ambiguous — was applied to mode C (oc_prechi2_fej_gauge) without proof that this matches Li & Mourikis IJRR 2013 exactly. The MSCKF 2.0 paper requires specific observability analysis and linearization conditions that have not been audited against this implementation. |
| `msckf2_0` | Same — variant of the above |
| `msckf2_pure` | Same — was used as an alias for openvins_fej without documentation |
| `dso` | Was used for K-projection v1 (smoke failed) and then silently redirected to mode E. Neither usage is principled DSO. |
| `vins_nullspace` | Never implemented correctly (n_zeroed=0). No valid VINS implementation exists. |
| `h_proj` | Superseded by mode B. |
| `constrained_yaw_nullspace` | Smoke failed 2026-06-06. |

---

## Fly3 Validation Results (2026-06-07)

See `fly3_oc_variant_validation_20260606.md` for full details and decision rationale.
Conditions: FC init, GPS-Z, gps-time-offset=0, fi1e4, use_fej=true, t=[618,1837]s.

| Mode | Full ATE RMS | Late ATE RMS | Yaw RMS | Yaw max | Classification |
|------|-------------|-------------|---------|---------|----------------|
| openvins_fej | 448m | 597m | 8.7° | 86° | BASELINE_REQUIRED |
| oc_postchi2_current_gauge | **323m** | **422m** | **6.5°** | 101° | **OFFICIAL_OC** |
| oc_prechi2_fej_gauge | 638m | 909m | 8.7° | 83° | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE |
| oc_4d_prechi2_fej_gauge | 646m | 920m | 9.4° | 78° | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE |

Mode B (oc_postchi2_current_gauge) is designated **official_oc** for controlled comparisons.
Pre-chi2 modes (C, D) produced worse XY accuracy than no-OC (A) despite lower yaw_max,
because H projection before the chi2 gate corrupts the outlier-rejection calibration.
