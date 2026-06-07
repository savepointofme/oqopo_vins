# yaw_gauge_modes_design.md

## Purpose

Honest classification and evaluation plan for estimator-side yaw/gauge/observability handling
in OpenVINS MSCKF on the D455 fly1 dataset.

**This document covers `--vio-yaw-gauge-mode`, which controls the EKF update inside the
estimator. It has nothing to do with `eval_stage.py --yaw-align-mode` (post-processing only).**

---

## Locked reference

```
R1_baseline_restore
config:   sigma_px_2p0_fcinit_highalt.yaml (fi_max_dist=500, sigma_px=2, use_fej=true)
GPS-Z:    off
att-sig:  3.0°
eval:     T0=980, T1=1630, --yaw-align-mode start_yaw
XY RMS:   216.27m
```

All modes compare against this. A mode is not an improvement unless it beats 216.27m without
yaw spikes or covariance pathologies.

---

## What `--vio-yaw-gauge-mode` actually is

The flag is a **CLI wrapper**. It maps short alias names to `--vio-yaw-update-mode` +
`--vio-global-yaw-oc-alpha`. It does not implement new algorithms; it selects among modes
that were already in `StateHelper::VisualYawUpdateMode` and `VisualObservabilityPolicy`.

Correct label: "Added a unified CLI wrapper for existing yaw/gauge modes."
Incorrect: "Implemented MSCKF2.0 / VINS / FEJ / OC."

---

## FEJ in this codebase

`use_fej: true` in the YAML enables First-Estimate Jacobians in OpenVINS:
- `get_feature_jacobian_full()` uses `clone->Rot_fej()` and `clone->pos_fej()` for H
- All H Jacobians are linearized at frozen first-estimate clone/IMU poses
- This is the FEJ-EKF of Li & Mourikis, ICRA 2013 ("Optimization-Based Estimator Design
  for Vision-Aided Inertial Navigation")

**R1 already uses FEJ correctly.** No re-implementation is needed. All modes below run with
`use_fej: true` in the YAML.

The key open question is whether the **gauge direction** used for OC projection is also FEJ-
consistent. See M0 inconsistency note below.

---

## Classification

Each mode is classified as:

- **A** — Trusted baseline (empirically validated)
- **B** — Existing legacy heuristic (not paper-backed, not validated)
- **C** — Existing partial implementation of a paper method (exists in code, worth evaluating)
- **D** — New paper-backed implementation required (not yet in code)
- **E** — Reject (inconsistent, duplicate, or unsafe)

---

## Mode inventory

### A. Trusted baseline

#### M0 — `baseline` / `r1`

**Alias:** `--vio-yaw-gauge-mode baseline`  
**Expands to:** `--vio-yaw-update-mode global_yaw_oc_projection --vio-global-yaw-oc-alpha 1.0`  
**Enum:** `GLOBAL_YAW_OC_PROJECTION = 2`

**What the code does:**
```
n = build_global_yaw_gauge_small(state, H_order, H_id, H.cols())   // use_fej=false (current state)
H_eff = H - (H·n)·n^T / ||n||^2                                    // alpha=1.0 full projection
EKFUpdate with H_eff, then standard K·res dx
```
Applied post-chi2 inside `EKFUpdate`. Chi2 gate sees unmodified H.

**Inconsistency:** H Jacobians are FEJ (from YAML), but the gauge direction n is built from
the **current state**, not the FEJ state. The yaw projection direction is not FEJ-consistent.

**Status:** Validated to 216.27m XY RMS on fly1. Keep as default. Do not modify.

**What paper it is NOT:** This is not OC-VINS or OC-EKF in the strict sense. Li & Mourikis
IJRR 2013 requires FEJ-consistent gauge; this uses current-state gauge. It works empirically
but is theoretically inconsistent in the sense that the gauge does not match the linearization.

---

### B. Existing legacy heuristics — do not evaluate as primary candidates

These modes are in the codebase but are not backed by a specific paper method. Use only as
negative controls if a quick sanity check is needed.

#### B1 — `per_block_scale` (enum=1)

Per-variable H-block yaw attenuation with alpha∈[0,1]. Pure heuristic. Not in ablation script.

#### B2 — `current_only_scale` (enum=3)

Like B1 but only attenuates the current-IMU block, not clones. Heuristic.

#### B3 — `a_strict_yaw_dx0` (enum=5)

Per-block H projection (scale=0) + explicit `zero_visual_yaw_dx` after update.
Double-protection: H is modified AND dx is zeroed. The covariance is updated as if dx was
applied, then dx is zeroed — inconsistent (P updated, x unchanged). Reject.

#### B4 — `hard_gyro_yaw` (enum=4, alias `no_yaw`)

H projection with alpha=1.0 (same as M0) + explicit `zero_visual_yaw_dx` after update.
Same double-protection inconsistency as B3. Additionally duplicates M0's H projection.
Use only as bound-check, not as a real evaluation candidate.

#### B5 — `visual_yaw_h_projection_current` (enum=7, alias `h_proj`)

H-space projection but the gauge is built from H_order-restricted variables only (same as
`build_global_yaw_gauge_small`). Functionally similar to M0. No advantage over M0 and lacks
the tested alpha parameter. Not worth evaluating separately.

#### B6 — `visual_yaw_schmidt_guarded` (enum=8)

Schmidt + pre-update guard (R-inflate or reject suspicious updates). Complex heuristic with
multiple hand-tuned thresholds. Not from a paper. Not in ablation script.

#### B7 — `visual_yaw_schmidt_current_gauge` (enum=6, alias `schmidt`)

K-space Schmidt update with **current-state** gauge. FEJ Jacobians + current-state gauge =
mixed inconsistency (same inconsistency as M0 but in K-space). Not paper-backed in this
specific form. Downgrade from primary candidate to negative control.

---

### C. Existing partial implementations worth evaluating

These modes implement recognizable pieces of paper methods and are worth comparing to R1.

#### C1 — `original` (enum=0, alias `original`)

**What it is:** Standard MSCKF with FEJ Jacobians, no OC projection.  
**Paper:** Mourikis & Roumeliotis ICRA 2007 + FEJ from Li & Mourikis ICRA 2013.  
**Use:** Negative control. Expected to be worse than R1 because yaw is updated unconstrained.  
**Value:** Establishes what the OC projection in R1 is actually buying.

#### C2 — `oc_prechi2` (VOP mode, alias `oc_prechi2`)

**Expands to:** `--vio-yaw-update-mode global_yaw_oc_fej_prechi2`  
**VOP enum:** `GLOBAL_YAW_OC_FEJ_PRECHI2 = 1`

**What the code does:**
```
n_fej = build_yaw_gauge(H_order, H_id, state, use_fej=true)   // FEJ-consistent gauge
H_oc  = H - H·n_fej·n_fej^T / ||n_fej||^2                    // 1-D OC projection
chi2 gate on residual with H_oc                                // gate sees OC residual
EKFUpdate(H_oc, mode=ORIGINAL)                                 // no second projection
```

**Paper connection:** Li & Mourikis, IJRR 2013, "High-precision, consistent EKF-based
visual-inertial odometry", sec. 4. The unobservable subspace for monocular VIO contains
global yaw; FEJ Jacobians + FEJ-gauge projection enforces consistency.

**Improvements over M0:**
1. Gauge is FEJ-consistent (not current-state like M0)
2. Chi2 gate tests the OC-projected residual, not the raw one (avoids chi2 inconsistency)

**Limitations:** 1-D gauge (yaw only), not the full 4-D. H_order-restricted gauge.

**Verdict: Best theoretical improvement candidate over R1. Evaluate first.**

#### C3 — `oc_4d` (VOP mode, alias `oc_4d`)

**Expands to:** `--vio-yaw-update-mode visual_4d_oc_fej_prechi2`  
**VOP enum:** `VISUAL_4D_OC_FEJ_PRECHI2 = 2`

**What the code does:**
```
N_fej = [n_yaw | n_tx | n_ty | n_tz]    // 4-col FEJ gauge matrix
Q, _  = QR(N_fej)                        // orthonormal basis
H_oc  = H - H·Q·Q^T                     // project out all 4 unobservable directions
chi2 gate and EKFUpdate with H_oc
```

**Paper connection:** Li & Mourikis IJRR 2013, full 4-DOF unobservable subspace for
monocular VIO (yaw + global xyz are all unobservable without absolute position reference).

**Risk:** Removing global xyz corrections from visual updates means the filter gets no
absolute-frame position correction from vision at all. On a long flight, this will accumulate
more drift than M0 or C2. Expected to be WORSE on absolute XY ATE.

**Verdict: Evaluate as upper bound on OC conservatism. Expect worse than R1 for XY ATE.**

#### C4 — `schmidt_fej` (alias `schmidt_fej`)

**Expands to:** `--vio-yaw-update-mode visual_yaw_schmidt_fej_gauge`  
**Enum:** `VISUAL_YAW_SCHMIDT_FEJ_GAUGE = 9`

**What the code does (K-space Schmidt):**
```
Q_full = build_global_yaw_gauge_full(state, N, use_fej=true)   // FEJ, full N-dim
q_hat  = Q_full / ||Q_full||
K_std  = P H^T (H P H^T + R)^{-1}
K_eff  = K_std - q_hat · (q_hat^T K_std)                       // remove yaw from gain
dx     = K_eff · residual
P+ via Joseph form: K_eff (not K_std) — ensures q^T P+ q = q^T P q
```

**Theoretical grounding:** Schmidt complement filter / consistent EKF with unobservable
subspace. The K-space approach differs from H-space: it does not modify H (so chi2 gate
uses unmodified H) but zeroes the gain in the gauge direction (so yaw state is not updated).
Using FEJ gauge makes it consistent with FEJ Jacobians.

**This is NOT from a named published paper in VIO.** It is a custom implementation of the
Schmidt complement filter principle applied to the yaw gauge. The Schmidt subspace filter
concept appears in estimation theory (Simon 2006, Chapter 13) but this specific
implementation is original to this codebase.

**Advantage over H-space (M0):** Preserves cross-covariance between yaw and other states
exactly. Joseph form guarantees gauge direction is preserved exactly: q^T P+ q = q^T P q.

**Risk:** Full N-dim gauge construction is expensive. FEJ gauge may misalign if FEJ estimates
have large error (long flight).

**Verdict: Theoretically sound alternative to M0. Worth evaluating alongside C2.**

---

### D. New implementations required

#### D1 — `global_yaw_oc_fej_projection` (post-chi2, FEJ gauge)

**Status: Not yet implemented.**  
**What it would be:** M0 with FEJ-consistent gauge. Fills the inconsistency in R1 where H
is FEJ but gauge is current-state.

**Implementation:** 1-line change in `project_global_yaw_from_H`:
```cpp
// Current (R1):
Eigen::VectorXd n = build_global_yaw_gauge_small(state, H_order, H_id, H.cols());
// Fixed (FEJ-consistent):
Eigen::VectorXd n = build_global_yaw_gauge_small(state, H_order, H_id, H.cols(), /*use_fej=*/true);
```
Expose via new mode string `"global_yaw_oc_fej_projection"` and alias
`--vio-yaw-gauge-mode oc_legacy_fej`.

**Why:** Makes M0's gauge consistent with its FEJ Jacobians. Bridge between M0 (current
gauge, post-chi2) and C2 (FEJ gauge, pre-chi2). Minimal code change, easy to verify.

**Paper connection:** Li & Mourikis IJRR 2013 — FEJ-consistent linearization requires FEJ
gauge for the unobservable subspace analysis to hold.

---

### E. Reject / do not implement

The following are explicitly rejected:

#### VINS-Mono style
VINS-Mono is a sliding-window BA (non-linear optimization) with marginalization. It does not
use an MSCKF EKF update; yaw is handled via marginalization priors and first-pose gauge
fixing in the optimization problem. Adapting this to MSCKF would require replacing the entire
update stage. Incompatible without a complete rewrite. **Reject.**

#### MSCKF2.0
"MSCKF2.0" is not a specific published paper title. Without identifying the exact paper,
equations, and required estimator changes, this label cannot be implemented. **Reject until
paper is named.** If referring to "A Robust and Versatile Monocular Visual-Inertial State
Estimator" (Qin, Li, Shen 2018) or "Robocentric Visual–Inertial Odometry" (Huang 2019),
name it explicitly.

#### Any mode that requires GPS-Z or height aiding to look good
By definition, any mode whose XY ATE improvement is explained by GPS-Z drift correction is
not a valid yaw/gauge method. **Reject class.**

---

## Focused evaluation plan

Run only these four, in order:

| Run | Alias | What it tests |
|---|---|---|
| baseline | `baseline` | R1 reproduction gate (must be 216.27m) |
| original | `original` | FEJ-MSCKF without OC — negative control |
| oc_prechi2 | `oc_prechi2` | FEJ-gauge OC pre-chi2 — best theoretical improvement |
| schmidt_fej | `schmidt_fej` | K-space Schmidt with FEJ gauge — alternative approach |

Skip: oc_4d (expected worse), schmidt (inconsistent gauge), h_proj (weaker M0),
no_yaw/hard_gyro_yaw (inconsistent), all B-class modes.

If D1 (`global_yaw_oc_fej_projection`) is implemented, run it as a 5th candidate between
baseline and oc_prechi2.

**Command for each (in WSL):**
```bash
bash run_fly1_yaw_gauge_ablation.sh baseline
bash run_fly1_yaw_gauge_ablation.sh original
bash run_fly1_yaw_gauge_ablation.sh oc_prechi2
bash run_fly1_yaw_gauge_ablation.sh schmidt_fej
```

---

## Evaluation protocol (fixed)

```bash
eval_stage.py \
  --t0 980 --until 1630 \
  --yaw-align-mode start_yaw \    # headline metric
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --imu .../imu0/data.csv
```

Also run `--yaw-align-mode best_yaw_fit` as diagnostic only — never headline.

Required metrics per run:
- XY RMS / max / P95 / final (start_yaw)
- XY RMS (best_yaw_fit) — labeled OPTIMISTIC
- XY late-window RMS [1200–1630]
- Yaw RMS / max / final (start_yaw)
- NaN/Inf count
- neg_cov count
- Visual update count
- Chi2 rejection count

---

## Success criteria

A mode is worth keeping only if ALL hold:
1. XY RMS (start_yaw) ≤ 216.27m (beats R1) — no tolerance given mode must earn its place
2. Yaw max < 30° (no spikes)
3. No NaN/Inf
4. No neg_cov
5. Late-window [1200–1630] XY RMS not dramatically worse than early window
6. Does not require GPS-Z or height aiding
7. Does not improve only because of lucky first-turn alignment

If no mode beats 216.27m: **keep R1 as default, document why alternatives fail.**

---

## What was NOT done here

- Did not claim FEJ/OC/Schmidt/MSCKF2.0 implementation from naming alone
- Did not implement VINS-style (incompatible with MSCKF)
- Did not implement "MSCKF2.0" (no paper specified)
- Did not run all 8 modes — legacy heuristics excluded from main evaluation
- `--vio-yaw-gauge-mode` is a CLI wrapper, not an algorithm implementation

---

## Code locations

| Component | File |
|---|---|
| EKFUpdate with gauge dispatch | `ov_msckf/src/state/StateHelper.cpp` lines 574–704 |
| `build_global_yaw_gauge_small` | `StateHelper.cpp` lines 126–195 |
| `project_global_yaw_from_H` | `StateHelper.cpp` lines 332–343 |
| `build_global_yaw_gauge_full` | `StateHelper.cpp` lines 212–305 |
| `EKFUpdateSchmidtYawCurrentGauge` | `StateHelper.cpp` lines ~808–995 |
| `VisualObservabilityPolicy` (VOP) | `ov_msckf/src/update/VisualObservabilityPolicy.h` |
| VOP apply (pre-chi2 projection) | `ov_msckf/src/update/VisualObservabilityPolicy.cpp` |
| `--vio-yaw-gauge-mode` CLI wrapper | `ov_msckf/src/run_serial_msckf_ros_free.cpp` |
| Ablation runner | `run_fly1_yaw_gauge_ablation.sh` |

---

## Baseline verification status

- `ygm_baseline` VIO run: DONE (traj.txt written, 30227 frames)
- `ygm_baseline` eval vs R1: PENDING (eval_stage.py not yet run)
- `R1_baseline_restore` eval: 216.27m confirmed (prior session)

The baseline VIO run used identical flags to R1. Eval will confirm reproduction.
