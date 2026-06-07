# Scalar Height / Range Aiding — Method Selection and Implementation Design
*Revised 2026-06-03. GPS-Z only (no GPS XY, no GPS yaw). GPS-Z is a proxy for laser
rangefinder. All candidates are altitude-only / scalar-height-only.*

---

## Design Questions — Answered

**1. Is the proposed method generic to scalar height/range aiding, or GPS-specific?**
The partial Schmidt update is generic. The measurement model (H, R) is the only
GPS-specific part. Replacing GPS-Z with a laser rangefinder requires only changing
the residual computation and the bootstrap; the EKF update math is identical.

**2. Can the same estimator design later accept laser rangefinder measurements?**
Yes. `UpdaterGroundPlaneRange` (which already exists) is already designed for this.
It uses AGL height = (z_gps - z_ground)/cos_tilt — exactly the rangefinder model.
For a laser, pass `z_laser = rho_laser * cos_tilt + z_ground` as the altitude input.

**3. What changes when replacing GPS-Z with laser range?**
- Residual: GPS → absolute altitude reference. Laser → range-to-ground (slant range).
- Noise model: GPS σ ≈ 2–5m. Laser σ ≈ 0.05–0.3m (much tighter).
- Datum: GPS needs bootstrap for datum. Laser bootstraps `z_ground` from VIO height.
- Update rate: GPS 5–10 Hz. Laser 50–200 Hz (may need decimation or rate gating).
- Tilt guard: laser needs stronger tilt guard (beam diverges from nadir → model error).

**4. Which states are updated by the scalar measurement?**
Partial Schmidt: only `p_z` receives the state correction.
No other states (q, p_xy, v, bg, ba, clones, SLAM) are modified.

**5. Which states are intentionally protected from the scalar measurement?**
All states except p_z. In particular:
- `p_xy` (horizontal position) — GPS-Z has no horizontal information
- `q` (attitude/orientation) — scalar height does not determine attitude
- `v` (velocity) — not updated unless `--height-aid-update-vz` is explicitly enabled
- `bg`, `ba` (biases) — protected to prevent spurious bias drift from GPS noise
- All clones and SLAM features — protected

**6. Does the covariance update remain symmetric and positive semi-definite?**
Yes. The partial Schmidt update modifies only the p_z row/column of P symmetrically.
Brink 2017 Theorem 1 proves PSD is maintained. See exact equations in §Candidate B below.

**7. Does the method preserve VIO observability and consistency?**
The partial Schmidt update does NOT inject false observability. It does NOT change
P(q,q), P(pxy,pxy), or any non-p_z block. VIO's unobservable directions (global yaw,
global XY position) remain consistent. The FEJ linearization points are unaffected.

**8. Does it avoid degrading XY and yaw?**
Yes. P(pxy,pxy) and P(q,q) are explicitly protected (unchanged). The XY degradation
path (P overconfidence → visual gain underestimation → XY drift) is severed.

**9. What papers support this method?**
- Brink 2017 JGCD: exact equations for partial-update Schmidt-KF
- Geneva 2019 CVPR SEVIS: Schmidt-EKF in MSCKF context (proves zero-gain = valid)
- Standard EKF textbooks (Brown & Hwang 2012): scalar measurement update baseline

**10. What open-source implementations can be referenced?**
- Existing `EKFUpdateSchmidtYawCurrentGauge` in this codebase: Schmidt update structure
- Existing `UpdaterGroundPlaneRange` in this codebase: measurement model already correct
- Brink 2017: no code, but equations are in §II of the paper (3 lines in the finite case)

---

## Candidate Methods

### Candidate A: Current naive GPS-Z EKF update (full, no revert)

**Current state:** `gps_alt_zonly_update_ = false` (full EKF with all states corrected)

**Measurement model:**
```
h(x) = p_z (for GPS altitude in VIO frame, after bootstrap absorbs datum)
h(x) = (p_z - z_ground) / cos_tilt  (range-mode / UpdaterGroundPlaneRange)
```

**H Jacobian (range mode):**
```
H = [H_q_x, H_q_y, 0,  0, 0, 1/r22]  (1×6, over [q, p] blocks)
H_q_x = -(p_z - z_ground) / r22^2 * R_GtoI(1,2)
H_q_y = -(p_z - z_ground) / r22^2 * (-R_GtoI(0,2))
```

**States updated:** ALL (q, p_xyz, v, bg, ba, clones, SLAM)

**States protected:** NONE

**Covariance update:** Standard EKF: `P+ = (I-KH)P(I-KH)^T + KRK^T`

**XY protection:** NONE

**Observed behavior:** XY RMSE 375m (new 30Hz run) vs 216m (old reference without GPS-Z).
**Leading hypothesis:** Covariance overconfidence from GPS-Z EKF updates.
*(Causal attribution to be confirmed by Phase 1 ablation.)*

---

### Candidate B: Partial Schmidt update for p_z only ← SELECTED

**Paper:** Brink 2017 JGCD §II, Geneva 2019 CVPR SEVIS §III

**Implementation:** New function `StateHelper::EKFUpdateSingleStatePartial`

The function is generic: it takes H, res, R, and an active state variable pointer.
For GPS-Z: active variable = `state->_imu->p()`, active dof = 2 (z component).
For laser rangefinder: same active variable, same math, different H and residual.
An optional `debug_label` string (e.g. `"gps_z"`, `"laser_range"`) is used in log output.

**Measurement model:** Same as Candidate A. H and residual unchanged.

**States updated:** ONLY p_z

**States protected:** q, p_xy, v, bg, ba, ALL clones, ALL SLAM features

**Exact update equations:**

Let:
- `j_pz` = `state->_imu->p()->id() + 2`  (global index of p_z in state->_Cov)
- `H_full` (1×N): H expanded to full state space (zero except at q and p_z)
- `M` = `state->_Cov * H_full.transpose()`  (N×1)
- `S` = `H_full * M + R_scalar`  (scalar)
- `K_pz` = `M(j_pz) / S`  (scalar)
- `res` = residual (scalar)

**State update:**
```
p_z  +=  K_pz * res   (only p_z changes)
```

**Covariance update:**
```
P.col(j_pz)       -= K_pz * M
P.row(j_pz)       -= K_pz * M.transpose()
P(j_pz, j_pz)    += K_pz * M(j_pz)   // correct double-subtract on diagonal
```

Resulting P (in closed form):
```
P+(j_pz, j_pz)  =  P(j_pz, j_pz)  -  K_pz * M(j_pz)   [standard Kalman for p_z]
P+(i,  j_pz)    =  P(i, j_pz)     -  K_pz * M(i)       [p_z cross-terms updated]
P+(i,  j)       =  P(i, j)                              [all non-p_z blocks UNCHANGED]
```

**Symmetry:** P.col and P.row modify opposite sides identically. Symmetry preserved. ✓

**PSD:** Brink 2017 Theorem 1. Also: P+(j_pz,j_pz) = P_zz - M_zz²/S ≥ 0 since S ≥ M_zz²/P_zz. ✓

**FEJ:** The linearization point for p_z (used in H_pz = 1/r22) is from the current EKF
state, consistent with the rest of OpenVINS FEJ practice for GPS measurements.
(GPS measurements do not have a stored FEJ point; they use current state.)

**What the partial Schmidt update prevents (and what it does not):**

P(pxy,pxy) and P(q,q) are not falsely reduced by the scalar height update, so the main
covariance-overconfidence path is removed. Cross-covariances involving p_z (e.g. P(pxy,pz),
P(q,pz)) do still change, reflecting the actual p_z update. The net effect on XY and yaw
must still be verified by ablation — the partial Schmidt update removes the leading
candidate mechanism, but it does not guarantee zero XY effect a priori.

**GPS-Z noise model:**
- Sigma: `--height-aid-sigma 2.0` (default; consumer GPS ≈ 2-5m)
- Model: R_scalar = sigma^2 (scalar, no cross-terms needed)
- For laser: sigma ≈ 0.05–0.3m, must be set separately

**Altitude datum / frame:**
- Bootstrap: on first accepted measurement, `z_ground = p_z - z_gps` absorbs datum offset
- Subsequent: residual = `(z_gps - p_z_vio + z_ground)` = altitude error in VIO frame
- Sign: both VIO and GPS use z-up → no sign flip needed

**GPS lever arm:**
- Currently: lever arm assumed zero (GPS antenna = IMU position)
- If lever arm is non-zero: add H_lv terms (d h / d p_lever = 1 for z component)
- Acceptable approximation for this application (lever arm typically < 0.3m → residual ≈ 0.3m)

**GPS time offset:**
- Handled by `--gps-time-offset`: applied in dataset loader before feeding to VioManager
- Time-alignment check: ±200ms gate in `feed_measurement_gps_altitude`

**Source files to modify:**
- `ov_msckf/src/state/StateHelper.h`: declare `EKFUpdateSingleStatePartial`
- `ov_msckf/src/state/StateHelper.cpp`: implement `EKFUpdateSingleStatePartial`
- `ov_msckf/src/core/VioManager.cpp`: add new branch in `feed_measurement_gps_altitude`
  when `height_aid_mode == partial_schmidt` → call `EKFUpdateSingleStatePartial` instead of
  `EKFUpdateZOnly`. Keep `EKFUpdateZOnly` branch as `zonly_legacy` for ablation.
- `ov_msckf/src/update/UpdaterGroundPlaneRange.cpp`: same: add `partial_schmidt` path
  alongside existing `use_zonly_` path (no deletion of legacy code).
- `ov_msckf/src/run_serial_msckf_ros_free.cpp`: add `--height-aid-mode` flag dispatch

**What is copied vs reimplemented:**
- Brink 2017 equations: reimplemented (no code available)
- Measurement model: already in codebase, unchanged
- Bootstrap: already in codebase, unchanged
- Guard infrastructure (innovation gate, P_zz floor): already in codebase, unchanged

**Expected config flags:**
```
--height-aid-mode  none | naive | partial_schmidt | zonly_legacy
--height-aid-sigma <float>        (measurement noise stddev, meters)
--height-aid-min-pzz <float>      (P_zz floor, meters^2; 0 = disabled)
--height-aid-max-res <float>      (innovation gate, meters)
--height-aid-delay <float>        (seconds after VIO init before first update)
```

---

### Candidate C: Current EKFUpdateZOnly (state revert, full covariance update)

**What it is:** The existing implementation in `StateHelper::EKFUpdateZOnly`.

**Is it mathematically consistent?** **No.**

The covariance P is fully updated by the standard EKF (P ← (I-KH)P) assuming all states
were corrected by K. But only p_z is actually corrected. This leaves P(pxy,pxy) and P(q,q)
**reduced** as if they were corrected, creating covariance overconfidence.

Over many updates, this overconfidence in P(pxy,pxy) causes the visual MSCKF to
under-correct XY, leading to XY drift.

**Should this be kept as an ablation point?** YES — to measure the improvement from
switching to partial Schmidt (Candidate B).

---

### Candidate D: GPS-Z used only for logging / vertical drift monitoring (no EKF update)

**What it is:** Complete disabling of GPS-Z EKF update. GPS-Z is logged alongside VIO
trajectory for post-hoc vertical accuracy evaluation only.

**When to select:** If ablation experiments show that neither naive EKF nor partial Schmidt
can improve vertical accuracy without degrading XY or yaw, this is the correct conclusion.

**Configuration:** `--height-aid-mode none` (current default when `--gps-alt-update` is
NOT specified)

---

### Candidate E: GPS-Z update with robust gating only (no Schmidt change)

**What it is:** Keep Candidate A (full EKF) but add aggressive gating:
- `--gps-alt-guard-dxy-max <m>`: reject if predicted |dp_xy| > threshold per update
- `--gps-alt-guard-kxy-ratio <ratio>`: reject if |K_xy|/|K_pz| > ratio
- Result: most GPS-z updates are rejected → near-zero average contamination

**Is this paper-backed?** The gating mechanism exists (it is already in the codebase).
The guards reject contaminating updates but do not fix the covariance update math.
For updates that pass the gate, the covariance is still inconsistently modified.

**Verdict:** A stopgap. Acceptable as an ablation variant. Not a principled fix.

---

## Method Comparison Summary

| | A: naive EKF | B: partial Schmidt | C: ZOnly legacy | D: off | E: guard-only |
|--|--|--|--|--|--|
| State protection (pxy, q) | NO | **YES** | Partial (state reverted, P wrong) | N/A | Partial (rejected updates) |
| Covariance consistency | NO | **YES** | **NO** | N/A | NO (passing updates) |
| PSD guaranteed | EKF guarantee | **Brink 2017** | Not guaranteed | N/A | Not guaranteed |
| Paper support | EKF textbook | **Brink 2017, Geneva 2019** | None | N/A | None |
| Altitude correction | YES (biased) | **YES (unbiased)** | YES (unbiased) | NO | YES (sparse) |
| XY protection | NONE | **FULL** | PARTIAL (wrong covariance) | N/A | PARTIAL (rejected updates) |
| Laser rangefinder compatible | YES | **YES** | YES | YES | YES |

**Selected method for Phase 2: Candidate B (Partial Schmidt)**
*(Subject to Phase 1 ablation confirming baseline recovery)*

---

## Ablation Plan

### Ablation runs required (fly1, T0=980 T1=1630)

All runs use: `--init-att-sigma-deg 2.0 --init-pos-sigma 0.05 --vio-yaw-update-mode global_yaw_oc_projection --vio-global-yaw-oc-alpha 1.0`

Config: `estimator_config_fly2params_cond_1e5.yaml`

| Run label | GPS mode | `height-aid-mode` | `height-aid-sigma` | Guard flags | Purpose |
|-----------|----------|-------------------|-------------------|-------------|---------|
| `ablation_gps_off` | none | none | — | — | Confirm 216m baseline recoverable |
| `ablation_naive_s2` | GPS-Z | naive | 2.0m | — | Replicate current regression (375m) |
| `ablation_zonly_s2` | GPS-Z | zonly_legacy | 2.0m | — | Quantify ZOnly vs naive |
| `ablation_partial_s2` | GPS-Z | partial_schmidt | 2.0m | — | Main proposed fix |
| `ablation_partial_s5` | GPS-Z | partial_schmidt | 5.0m | — | Effect of larger sigma |
| `ablation_partial_s10` | GPS-Z | partial_schmidt | 10.0m | — | Weak GPS trust |
| `ablation_partial_s1` | GPS-Z | partial_schmidt | 1.0m | — | Aggressive GPS trust |
| `ablation_partial_gate30` | GPS-Z | partial_schmidt | 2.0m | `--height-aid-max-res 30` | Outlier rejection |
| `ablation_partial_kxy05` | GPS-Z | partial_schmidt + guard | 2.0m | `--guard-kxy-ratio 0.5` | Hybrid guard |

### Required metrics per run

| Metric | Purpose |
|--------|---------|
| XY RMSE / mean / max (m) | Verify GPS-Z does not degrade horizontal accuracy |
| Z RMSE / mean / max (m) | Verify GPS-Z actually helps vertical accuracy |
| Yaw RMSE vs GPS course (deg) | Verify GPS-Z does not disturb attitude/yaw |
| GPS-Z residual histogram | Check model validity |
| Innovation covariance S time-series | Check Kalman gain health |
| Mahalanobis distance per update | Outlier statistics |
| Accepted / rejected update count | Guard effectiveness |
| |dp_xy| per update (distribution) | XY contamination direct measure |
| |K_xy| / |K_pz| ratio (distribution) | Cross-covariance level |
| P_zz over time | Covariance health (no collapse) |
| P_pxpx over time | XY covariance health (should not shrink with GPS-Z in partial Schmidt) |

All metrics must be measured over T0=980 to T1=1630 (cruise window, before descent).
Z metrics must be measured over the full flight (GPS-Z should help when aircraft is at altitude).

### Success criteria

| Criterion | Threshold |
|-----------|----------|
| XY RMSE not worse than GPS-off baseline | ≤ 216m + 10% (≤ ~238m) |
| Z RMSE better than GPS-off | At least 10% improvement in vertical RMSE |
| Yaw RMSE not worse | ≤ GPS-off yaw RMSE + 2° |
| P_pxpx trend | Must not decrease monotonically due to GPS-Z updates |
| |dp_xy|_mean per update | < 0.1m (contamination budget per update) |

### Acceptable conclusion (if evidence says GPS-Z is unsafe)

If ablation results show:
- Partial Schmidt still degrades XY compared to GPS-off baseline
- P_pxpx is affected despite partial Schmidt
- Vertical improvement is negligible (< 5% Z RMSE reduction)

Then the correct conclusion is: **disable GPS-Z update (`--height-aid-mode none`)
and use GPS-Z only for vertical accuracy logging/evaluation.**

This is an explicitly acceptable outcome and should be stated clearly in the experiment
report, not treated as a failure.

---

## Implementation Phases

### Phase 1 (first): Ablation — GPS-Z off  ← COMPLETE

**Results (eval_stage.py, T0=980 T1=1630):**

| att-sigma | GPS-Z OFF | GPS-Z ON |
|-----------|----------:|----------:|
| 3° (old ref) | 216m | *missing* |
| 2° (Phase 1) | **299m** | 375m |

**Confirmed:** GPS-Z update degrades XY by +77m under the current att=2° configuration.

**Not yet confirmed:** The old 216m reference also differs from the current 299m GPS-off baseline,
likely due to att-sigma (3°→2°) and possibly binary/config differences. The two effects
(GPS-Z and att-sigma) should NOT be treated as linearly additive. The estimator is nonlinear
and effects may interact. The missing run is att=3°, GPS-Z ON (run 5 in the factorial table).

**Yaw observation:** GPS-Z ON shows lower yaw RMS (7.5° vs 10.1° GPS-Z OFF) on this run.
This should be treated as a suspicious secondary effect, not as evidence that GPS-Z improves
attitude. A scalar height update changing yaw indicates the same cross-covariance contamination
path we are trying to remove, or a flight-specific survivor bias. This must be checked by
segmented analysis (first turn, cruise, pre-descent) and by the Phase 2 ablation.

**Phase 2 baseline:** att=2°, GPS-Z OFF = 299m. Phase 2 success criterion: XY RMSE ≤ 329m
(299m + 10%). Final conclusion requires the full factorial table (6 runs).

### Phase 2 (targeted fix): Implement Partial Schmidt

**Files:** StateHelper.h, StateHelper.cpp, VioManager.cpp, UpdaterGroundPlaneRange.cpp
**New function:** `StateHelper::EKFUpdateScalarPartialZ`
**Equations:** Exact equations in Candidate B §above (Brink 2017)
**No changes to:** measurement model, bootstrap, gating infrastructure
**Test:** ablation_partial_s2 vs ablation_gps_off

### Phase 3 (validation): Ablation sweep

Run all 9 ablation variants. Generate:
1. XY trajectory comparison plot (all variants)
2. Z error vs GPS altitude plot
3. Yaw error vs time
4. P_pxpx and P_zz over time for partial Schmidt vs naive
5. |K_xy|/|K_pz| histogram for naive vs partial Schmidt

### Phase 4 (optional): Laser rangefinder integration

If the project transitions to a real laser altimeter:
1. Modify `UpdaterGroundPlaneRange::try_update` to accept raw range `rho_meas` directly
2. Add `height-aid-source` flag: `gps_z | laser_range | barometer`
3. For laser: add range validity check (`rho_min < rho < rho_max`)
4. The EKF update math (partial Schmidt) is identical
5. Adjust sigma: laser σ ≈ 0.05–0.3m instead of GPS σ ≈ 2–5m

---

## Why Full 3D GPS Is NOT Implemented Here

Methods from Lee 2020 (ICRA), MINS (rpng/MINS), VINS-Fusion GPS, GVINS are marked
**NOT ALLOWED** for this project:

- They require GPS XY (full 3D ENU position input)
- They require estimating a 4-DoF GPS-VIO global frame transform
- This transform estimation depends on GPS XY observability (impossible without GPS XY)
- Any "GPS position" input in these methods is an XY+Z bundle, not Z-only

There is no known paper or open-source system that fuses ONLY GPS-Z (without GPS XY)
as a full-3D position constraint. GPS-Z-only IS a scalar constraint, and the scalar
partial Schmidt update (Brink 2017) is the correct and only paper-backed method for it.
