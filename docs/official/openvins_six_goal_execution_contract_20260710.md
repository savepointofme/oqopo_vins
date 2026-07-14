# OpenVINS Six-Goal Execution Contract

Date: 2026-07-10  
Corrected: 2026-07-12  
Status: active, corrected goal mapping  
Initial review commit: `3d9296c34d19767b967e12d3ec8fbaf18d1ec1f3`

## Decision

The project has more implementation scaffolding than accepted evidence, and
online adaptive/recovery work was started before source, frame, time, and data
contracts were frozen. The earlier execution contract then introduced a second
error: it reassigned P2 to terminal-divergence diagnosis, delayed overlap work to
P5, and let recovery occupy P6. This correction restores the original six goals
and uses a gated dependency chain in which P4 I0--I3 are independent of P3.
Only I4 may consume a separately accepted P3 calibration.

| Gate | Corrected objective | Status on 2026-07-12 |
| --- | --- | --- |
| P0 | Freeze and index existing source, data, calibration, and experiments | `PASS_AUDIT_WITH_LEGACY_RECEIPT_GAPS`: 32-run binding and global-baseline lineage audit complete; historical source identity gaps remain disclosed |
| P1 | Define initialization modes plus frame/time/output contracts | `RUNTIME_CONTRACT_IMPLEMENTED_I2_CAUSALITY_DEFECT_DISCLOSED`: all modes exist, but current I2 uses future FC attitude rows without delaying formal initialization |
| P2 | Derive overlap and maximum safe stride from the existing fixed-stride sweep | `PROVISIONAL_SHADOW_COMPLETE_OFFLINE_ONLY`: 32 runs reused, no estimator replay and no active release; clean-baseline revalidation remains pending |
| P3 | Re-estimate FC/IMU time offset and mounting rotation | `INCONCLUSIVE_DIAGNOSTIC`: no accepted residual mounting correction; P3 does not block P4 I0-I3 |
| P4 | Implement exact-time, short-window FC initialization | `P4_AIRBORNE_REEVALUATION_REQUIRED`: I2 improves the registered short window; the previous whole-dataset non-inferiority decision is invalid because it included post-touchdown ground samples; no P4 pass/fail decision is active |
| P5 | Implement overlap-bounded adaptive stride, shadow before active | `NOT_RELEASED`: provisional P2 shadow is complete; active release awaits a performance-accepted clean baseline and P2 release revalidation, not P3 |
| P6 | Deliver the unified data product and final six-goal acceptance | `AUTHORITATIVE_DATA_PRODUCT_UPDATE_IN_PROGRESS_FINAL_ACCEPTANCE_PENDING`: invalid whole-dataset rows and corrected airborne rows must remain separately labeled; four-flight P4/P5 evidence is incomplete |

Recovery, anchor trust, restart, pose repair, and reset are not P0--P6 work.
They remain closed and may be reopened as a separate project only after P6.

## 2026-07-11 Global-Baseline Lineage Addendum

The read-only audit of `C:\Users\baloney\Desktop\results_20260612` and its
downstream experiment paths changes the baseline interpretation:

- the directory is a small calibration bundle, not a global experiment archive;
- packaged intrinsics/distortion are the exact final values of the single fly3
  `np700_chi0p20` online run, while `T_C_I` comes from the June-12 Kalibr
  candidate and the Camera--IMU time offset was manually forced to zero;
- preserved fly1/fly2/fly3 online finals differ, so four-flight common
  convergence and the historical "final global calibration" claim are rejected;
- all 32 historical fixed-stride runs share one binary/config/calibration
  identity tuple, but use legacy one-row nearest FC initialization and
  start-heading evaluation; their performance metrics remain `PROVISIONAL_P3_P4`;
- the raw FC/board audit uses no VIO attitude. Its pooled rotation differs from
  the declared mechanical axis transform by about `4.089 deg`, but the maximum
  per-flight-to-pooled difference is about `3.880 deg`, so the cross-flight gate
  fails. Among 221 detected turns there are zero recovering small-angle flex
  candidates; large nonrecovering/model-mismatch residuals are retained rather
  than converted into a fixed compensation;
- `T_C_I` remains fixed. Neither an approximately 7-degree FC correction nor a
  time-varying Camera--IMU extrinsic is authorized.

The authoritative audit bundle is
`C:\Users\baloney\Desktop\实验目录\global_baseline_lineage_audit_20260711\20260711_results_20260612_readonly_audit_v6`.
Its products are frozen read-only as audit v6 and are not extended further. E0
reuses the historical result. P4 I0-I3 and E1/E2 are not blocked by P3; only I4
depends on an independently accepted P3 calibration.

## 2026-07-12 P1/P4 Dependency And Execution Addendum

- P3 is `INCONCLUSIVE_DIAGNOSTIC`. Approximately 7 degrees, `4.089 deg`, and
  every other fixed FC-to-board residual correction are prohibited.
- `fc_full_state`, `fc_attitude_only`, and `vio_only` are explicit runtime
  modes. Clean P4 uses `fc_full_state`.
- I0 nearest-row, I1 bracketed exact-time SLERP/linear interpolation, I2 robust
  3--8 s SO(3), and I3 quality/covariance/bias/fallback handling are implemented.
  I4 is not implemented.
- `canonical_init_state.json` records `t_seed`, source rows/timestamps, bracket
  gap, alpha, applied state, covariance, quality and fallback status.
- Formal `traj_nav` is in `G_nav`; primary evaluation is
  `absolute_navigation_no_post_alignment`. GPS and FC attitude are references,
  not truth. No position, heading, or SE(3) post-alignment is allowed.
- June-12 ground Kalibr K/D/T_C_I/time is locked and all online camera
  calibration is off for the clean baseline. `T_C_I` is fixed.
- fly1/fly3 short-window batches passed the runtime frame contract. I2 exact-time
  plus robust short-window FC attitude initialization improves the registered
  short window. The previous whole-dataset evaluation included post-touchdown
  ground samples and is invalid for P4 non-inferiority.
- Current I2 uses a symmetric five-second window and therefore consumes future
  FC attitude rows without delaying the formal initialization time. It remains
  a non-causal offline diagnostic until corrected.
- P2 produced all three provisional offline shadow policies from the existing
  32 fixed-stride runs using actual persisted camera gaps. It did not run the
  estimator, extend FC-flex classification, or generate an active release.

Until a later gate explicitly passes:

- `fixed stride12` remains the frozen comparison recipe, not a whole-flight safety claim;
- active adaptive stride, simulated-GPS pose repair, restart supervision, and hard reset are `NO-GO`;
- approximately 7 degrees, `4.089 deg`, and every other fixed FC-to-board
  residual correction are prohibited;
- the 2026-06-24 calibration/OC/stride performance conclusions are historical
  candidates, not accepted conclusions, because their FC initialization lineage
  is not proven correct;
- GPS horizontal position/course and other reference fields remain evaluation-only;
- GPS-Z is permitted only in the registered guarded baseline and in explicit A/B tests.

### Historical Terminal-Landing Diagnostic — Not A Gate

This subsection records an earlier diagnostic and does not exclude samples from
P2/P5 acceptance. For fly3, the historical evaluation boundary was the final
sustained downward crossing of GPS-relative ENU-U `30 m`: the first excluded GPS
sample is `t=1700.600 s` (`U=29.745 m`, horizontal speed `30.638 m/s`), and the
last included sample is `t=1700.400 s`.  This phase boundary is derived only from
reference data and never enters the estimator or an online stride/recovery gate.

At stride12 the measured camera cadence is approximately `0.400 s`, so the
vehicle moves about `12.25 m` per processed frame at the boundary.  A level,
planar pinhole approximation using the registered intrinsics gives only about
`67%--75%` along-track overlap at `30 m` and `51%--63%` at `20 m`; attitude,
terrain, and blur can reduce it further. These figures are a diagnostic sanity
check only; P2 must replace them with attitude-aware footprint intersections on
actual camera timestamps and must not adopt them as a safety threshold. The later threshold crossing at
`t=1746.800 s` (`U=0.283 m`) is therefore recorded as a terminal-landing failure,
not as evidence that active recovery is required.

P2 retains every sample and labels terminal, height, straight, turn, and
transition strata explicitly. P5 uses full-flight primary acceptance and reports
terminal/height strata separately; no landing crop can replace the full result.

## Evidence Behind The Decision

| Finding | Repository evidence | Decision |
| --- | --- | --- |
| Baseline provenance was incomplete | The wrapper now captures commit/patch/untracked recovery archive, binary/input hashes and toolchain evidence, then seals an atomic run receipt after post-run verification. The legacy 32-run sweep is now bound by command/binary/config/calibration/analysis hashes, but those historical runs do not contain equivalent atomic source-patch and dataset-content receipts. | The corrected fly3 P0 provenance sub-gate passed; the legacy sweep index is built with explicit receipt gaps rather than retroactively declared fully reproducible. |
| Raw/nav layers and the initialization-mode contract are implemented | `traj_raw.txt`, `traj_nav.txt`, `nav_frame_metadata.json`, and `canonical_init_state.json` are emitted; all three modes are explicit and `fc_full_state` outputs directly in `G_nav`. | P1 is `PASS_RUNTIME_CONTRACT`. |
| Exact-time initialization is implemented with a disclosed causality defect | `FCInitLoader.h` provides nearest-row, bracketed SLERP/linear interpolation, Huber-weighted SO(3) tangent regression, and quality/covariance/bias/fallback behavior. The current symmetric I2 window uses future FC rows. | Runtime evidence is retained; no P4 performance pass/fail decision is active. |
| The unified master contains current evidence | The authoritative product is being revised to retain old whole-dataset rows as `invalid_evaluation_window` and add airborne full-flight rows with explicit gate/status fields. | P6 final acceptance is pending. |
| Narrow tests cover the runtime contract | The standalone loader test covers the three modes, exact-time interpolation, gap/duplicate fail-closed behavior, robust SO(3), and I3 fallback; the frame validator suite passes 8/8. | P1/P4 runtime evidence passes; ROS1/ROS2 parity remains outside this batch. |
| Raw FC/board calibration remains inconclusive | The frozen replacement audit no longer uses VIO attitude, and its cross-flight gate does not produce an accepted correction. The roughly 5 Hz FC attitude is not extended into further flex classification. | P3 is `INCONCLUSIVE_DIAGNOSTIC`; no fixed FC-to-board correction is permitted. |
| The fixed-stride controls already exist | The complete sibling `stable_config_stride_validation_20260622/runs` tree contains the file-complete OC raw set for four flights and strides `1,2,4,8,12,16,20,30`; per-run legacy metric outputs are under that sibling tree's `delivery_package/<run>/analysis`, and aggregate stride scans are under `official_analysis/<flight>_stride_scan`. The separate compact `DELIVERY_stable_config_20260623` has only 28 fixed-stride trajectories and omits diagnostics. Only 9/32 complete sibling analyses avoid the registered 1000 m divergence trigger; failures remain indexed. | P2 reuses the complete sibling tree; no 4x8 fixed-stride replay is authorized. |
| Historical whole-flight adaptive evidence fails | The legacy extended result under the unaccepted FC-init lineage reports fixed stride12 divergence on 3/4 flights and active adaptive divergence on 4/4 flights. | It supports keeping active adaptive `NO-GO`; it is not accepted P5 performance evidence. |

## Clarifications To The Review Text

Three acceptance statements are made precise here:

1. The `<= 5 deg` FC/IMU criterion applies to residual mounting-rotation error
   after the declared nominal FRD/IMU axis transform, not to the full mounting
   rotation, which can legitimately contain 90/180 degree axis changes.
2. Offline dashboard/report code does not have to parse Parquet directly. It must
   consume a versioned payload derived from the authoritative master table and
   must not independently recompute frame or time mappings. The live dashboard
   instead reuses the same transform implementation/metadata and is checked by
   master-table replay.
3. Recovery design notes are retained as research history only. They create no
   implementation or experiment task inside the corrected P0--P6 plan.

### Conclusion Quarantine

The 2026-06-24 report remains useful for candidate camera parameters and for
locating historical experiments. Its quantitative claims are not final performance
evidence for P2-release or P3--P6. In particular, the calibration ranking, online-calibration verdict,
OC/FEJ comparison, stride ranking, and claims that other parameters are
second-order must be recomputed after clean P4 establishes a correct, timestamped FC
initial state. The final camera intrinsics, distortion, and camera--IMU transform
may be used as locked P2-candidate geometry inputs; this does not accept the old
trajectory/error or stride-ranking conclusions. P2 release must regenerate its
trajectory outcomes after accepted P1/P4 timing and initialization are applied.

Registered historical source:

- document: `C:/Users/baloney/Downloads/固定翼OpenVINS标定收敛_OC-FEJ与不同步长试验汇报_标定结果修订版_20260624.docx`;
- SHA256: `221c523d6b37dcf5f38598feabbae4cc967e7b27f9a42564f586a58b0eef2030`;
- candidate geometry: `K=[386.750,387.233,330.249,239.916]`, distortion
  `[-0.043,0.035,-0.001,0.001]`, reported translation
  `[-26.296,+28.573,-19.145] mm`, and reported rotation
  `[[0.999873,-0.015861,0.001552],[-0.015828,-0.999691,-0.019150],[0.001856,0.019123,-0.999815]]`;
  the direction must be bound to the P1 frame contract before use rather than
  inferred from the matrix values;
- the document contains no FC/flight-controller lineage, first-turn analysis, or
  P1/P4 evidence. Its sole `initial value` occurrence describes a calibration
  optimizer initial value, not the FC initialization. It therefore cannot prove
  the initialization lineage or a first-turn cause.

The frozen sweep exposes a separate lineage contradiction: `config_run.yaml`
says fly1 should use `fc_init_state_930_correct.csv`, while every captured fly1
command uses `fc_init_state_930.csv`; no `_correct.csv` artifact is present in the
registered inputs. Also, `kalibr_imucam_chain.yaml` says the locked intrinsics and
distortion came from the fly3 `np700_chi0p20` online final state while the
extrinsic was already locked to `newcalib`. This is not a four-flight global fit.
These facts justify quarantine; they do not identify the corrected FC state or
replace the clean P4 experiments.

`C:/Users/baloney/Desktop/实验目录/P0_P1_FINAL_STATUS.md` is also superseded as a
gate-status statement: its historical P0 receipt evidence remains valid, but its
`P1 PASS` label was temporarily reopened by this contract and is now superseded
by the verified `P1 PASS_RUNTIME_CONTRACT` result dated 2026-07-12.

## Data And Frame Boundary

Online estimator inputs for the frozen baseline are IMU, stereo images, the FC
initialization artifact, and guarded GPS altitude. GPS ENU XY, GPS course, FC
post-initialization navigation state, dashboard alignment, and evaluation-aligned
columns are reference-only unless a later, separately registered experiment
changes the estimator-input contract.

The authoritative frame names are:

- `W0`: initialized OpenVINS local world;
- `G_nav`: causal navigation frame established at initialization;
- `G_eval`: offline evaluation frame;
- `I`: OpenVINS IMU frame;
- `C0/C1`: camera frames;
- `FC`: declared flight-controller body frame;
- `A_gnss`: GNSS antenna phase-center frame.

No column may change frame semantics across runs. Every transform records source,
direction, version, timestamp policy, whether future data were used, and whether
it is estimator-input or evaluation-only.

### Startup Time, Origin, And Mode Boundary

The initial review's claim that the current baseline starts raw `W0` at a
non-zero position is not supported by the registered four-flight inputs.  All
four FC-init artifacts contain `p=[0,0,0]`, and the runner writes that value at
the EKF seed timestamp.  The first trajectory row is emitted only after the
estimator becomes output-ready, about `1.63 s` later, so a non-zero first-row
position is propagated motion rather than an origin violation.

The startup timeline has four distinct times:

```text
t_trim = dataset_first_imu_timestamp + requested_start_offset
t_seed = first retained camera timestamp
t_fc   = selected FC-init source timestamp
t_emit = first output-ready trajectory timestamp, with t_emit >= t_seed
```

For the legacy fly3 `local_w0_seed` replay, `t_seed=618.034715652 s`, `t_fc=618.000000000 s`, and
`t_emit=619.668312788 s`.  At `t_seed`, `p_IinW0=[0,0,0]`; at `t_emit`, the
position norm is `68.259 m`. Its legacy `G_nav` origin is GPS-anchored. This is
historical evidence, not the definition for `fc_full_state`, whose `G_nav` comes
from the declared FC navigation frame.

Two persisted records have different purposes:

- `canonical_init_state` is a non-trajectory initialization record at `t_seed`;
- the first visual trajectory row is the output-ready state at `t_emit`.

A master table must not invent a visual trajectory row at `t_seed` or relabel
the first emitted row as the seed. It must persist `canonical_init_state`
separately. In `fc_full_state`, its timestamped pose, velocity, and attitude must
match the exact-time FC state at `t_seed` within registered tolerances.

P1 defines three explicit initialization modes:

1. `fc_full_state`: FC supplies timestamped pose and velocity. Canonical output
   is directly in `G_nav`; its first formal position may and normally will be
   non-zero. No later dashboard or offline pose/yaw alignment may redefine that
   canonical output. The persisted `canonical_init_state` agrees with exact-time
   FC pose, velocity, and attitude. An internal local `W0` is permitted, but is
   diagnostic only.
2. `fc_attitude_only`: FC supplies only declared attitude and optionally velocity;
   position uses a local zero origin and canonical output is labeled local.
3. `vio_only`: position uses a local zero origin and yaw follows the pure-VIO
   gauge contract.

The current `local_w0_seed`/`global_gnav` boundary is useful scaffolding, not a
completed three-mode implementation. Exact-time interpolation, short-window
attitude estimation, covariance, and fallback behavior belong to P4.

## Gated Plan

### P0 — Frozen Reproducible Baseline

Deliverables:

- frozen commit and branch/worktree identity;
- exact runner/config/command snapshots;
- SHA256 for runner, configs, FC init, GPS/reference, and dataset inventory;
- binary dirty patch plus its SHA256;
- untracked source/config inventory, hashes, and a deterministic recovery
  archive, excluding declared generated-result roots;
- compiler, CMake, OpenCV, Eigen, Ceres, Boost, and linked-library versions;
- explicit feature-state manifest proving active adaptive, repair, and restart are off.
- a formal index that links the compact `DELIVERY_stable_config_20260623`
  package to its complete sibling run tree and records that the compact package
  omits the four stride1 runs and all per-run diagnostics;
- a machine-readable post-run verification report and atomic `run_receipt.json`
  binding the source snapshot, command/config, dataset identity, raw outputs,
  termination event, and validation reports.

Exit gate:

- a clean worktree can reproduce the fixed-stride12 command;
- the current reproducible baseline maps uniquely to source, patch, config,
  binary, and input data; legacy controls bind every identity that still exists
  and state missing historical receipts explicitly rather than fabricating them;
- all 32 actual sweep results are indexed, and every compact-delivery trajectory
  is bound to its sibling raw run while omitted stride1/diagnostic files are
  explicit;
- provenance validation fails closed when required fields are absent;
- shallow receipt verification remains valid after later derived analysis is
  added, while deep verification can re-hash the current repo and full dataset.

### P1 — Initialization-Mode Contract And Runtime Conformance

Deliverables:

- one `FRAME_CONTRACT` covering all frames, quaternion directions, Euler order,
  units, lever arms, and online/evaluation boundaries;
- explicit `fc_full_state`, `fc_attitude_only`, and `vio_only` modes with
  machine-readable mode metadata;
- `fc_full_state` canonical pose and velocity expressed directly in `G_nav`,
  including a permitted non-zero first formal position and no later canonical
  pose/yaw alignment;
- a versioned schema for a persisted non-trajectory `canonical_init_state` at
  `t_seed`; for `fc_full_state` the contract requires exact-time FC pose,
  velocity, and attitude, while real-flight conformance is verified after P4;
- explicitly local canonical output for `fc_attitude_only` and `vio_only`;
- raw-to-nav invariant audit and exactly one explicit transform per value;
- an explicit `start offset -> trim -> EKF seed -> first emitted row` timeline;
- code-generated schema documentation and experiment index;
- versioned master-derived payload for dashboards/reports;
- registered CTest entries and ROS1/ROS2 target parity for relevant tests.

`P1-contract` exit gate, before P2-candidate:

- raw-to-nav rotation/position reconstruction residuals are near floating precision;
- timestamps are monotonic, seed/output-ready are not conflated, and FC/GPS
  ages are auditable in schema and synthetic fixtures;
- every mode contract passes positive and fail-closed synthetic tests, and no
  validator imposes a zero-position rule on `fc_full_state` canonical output;
- the first trajectory row remains explicitly labeled `t_emit`, separate from
  the initialization record at `t_seed`.

`P1-runtime-conformance` exit gate, after P4:

- the real-flight `fc_full_state` initialization record matches synchronized,
  exact-time FC pose, velocity, and attitude at `t_seed` within registered
  tolerances;
- a representative visual replay proves each implemented mode and output frame;
- raw-to-nav rotation/position reconstruction residuals remain near floating
  precision under the accepted P4 initialization path.

### P2 — Existing-Sweep Footprint Overlap And Safe Stride

Do not run the estimator. Reuse the complete sibling run tree under
`stable_config_stride_validation_20260622`, not only the compact delivery copy.
The registered candidate strides are `1,2,4,8,12,16,20,30` on all four flights.
P2 has two gates to avoid a dependency deadlock:

- `P2-candidate`: index the existing sweep and compute diagnostic footprints and
  a provisional envelope using the currently declared time/transform policy,
  with uncertainty and input gaps explicit. Passing this gate allows P3 to start.
- `P2-release-revalidation`: after a performance-accepted clean P4 path,
  recompute geometry and trajectory outcomes offline. Promote an outcome from
  provisional to revalidated only when the accepted P1/P4 frame, time, and
  initialization lineage checks pass. P3 is required only if a P3-derived
  correction is proposed. Only this version may release an
  envelope to P5, and it must persist machine-readable
  `lineage_status=revalidated` or an equivalent versioned field.

The effective dependency chain is
`P0 existing-sweep binding + P1 contract -> P2 candidate`, in parallel with
`P1 contract -> P4 I0--I3 -> P1 runtime conformance`; then
`performance-accepted P4 + P2 release revalidation -> P5 -> P6`.
The separate branch `P3 accepted calibration -> I4` does not block I0--I3.

Registered analysis contract:

- build `EXISTING_STRIDE_SWEEP_INDEX.csv` with source run, command, binary/config/
  calibration identity, data and camera ranges, FC/GPS inputs, diagnostic files,
  canonical evaluation window, and completeness status;
- use actual camera timestamps, registered intrinsics/distortion, camera--IMU
  extrinsics, time-aligned attitude/position, and AGL plus a declared ground
  model to intersect sampled image-boundary rays with the ground;
- for every candidate stride store footprint polygon, area overlap, along-route
  overlap, perpendicular-to-route overlap, camera rotation, predicted median/P95
  pixel displacement, actual `dt`, height, speed, roll/pitch, and angular rate;
- generate `tracking_backend_health` only from estimator-internal KLT/tracked
  count, track length where available, MSCKF input/accepted/rejected, parallax,
  and update/jump diagnostics;
- store GPS-referenced error and divergence only as
  `trajectory_outcome_provisional` until clean-P4 revalidation. These offline labels
  cannot set the candidate safety threshold and can never become P5 online inputs;
- derive thresholds from data with leave-one-flight-out validation. The
  `92%--98%` interval is a hypothesis, never a hard-coded answer;
- report straight, left-turn, right-turn, and height bands `<20 m`, `20--30 m`,
  `30--50 m`, and `>50 m` separately;
- missing AGL/ground, exact-time attitude, or actual-parallax inputs must produce
  an explicit `NOT_FORMAL` status. A planar diagnostic and candidate envelope may
  be emitted with that status, but it may not be labeled or consumed as the P5
  release envelope.

Required outputs:

- `EXISTING_STRIDE_SWEEP_INDEX.csv`;
- `OVERLAP_TIMESERIES.parquet`;
- `OVERLAP_HEALTH_DATASET.parquet`;
- `SAFE_OVERLAP_ENVELOPE.csv`;
- `MAX_SAFE_STRIDE_LOOKUP.csv`;
- `P2_OVERLAP_REPORT.md`.

`P2-candidate` exit gate: all 32 existing controls are indexed without replay;
geometry inputs are traceable; internal health and provisional trajectory outcome
are separate; and every assumption has a sensitivity/uncertainty record.

`P2-release-revalidation` exit gate: accepted P1/P4 timing, transform, and
initialization lineage are applied; a P3 calibration is required only if a
P3-derived correction is proposed; leave-one-flight-out bounds are reported with
uncertainty; and no release threshold depends on assumed AGL, assumed attitude, a
level-camera shortcut, or an offline reference field as an online input.

### P3 — FC/IMU Spatiotemporal Calibration

Use gyro cross-correlation only for coarse timing, then relative-rotation hand-eye
optimization for fine time/mounting rotation. Use genuine leave-one-flight-out
folds and separate straight, left-turn, right-turn, and angular-rate strata. The
old `accepted:false` package and every calibration derived from a VIO trajectory
with the unaccepted FC initial state are diagnostic inputs only. The accepted P3
fit must use raw FC attitude and raw IMU angular motion, or another input chain
whose relative rotations are demonstrably independent of estimator
initialization. It records sensor sources, timestamp policy, frame transforms,
and source-data identity.

Exit gate: every held-out fold and both turn directions improve; no solution is on
the search boundary; residual rotation error is `<= 5 deg`; per-flight time-offset
spread is `<= 0.25 s`. Failure emits diagnostics only and no online calibration.

### P4 — Exact-Time Short-Window Initialization

Run registered single-factor experiments `I0` through `I4`: nearest row,
exact-time only, robust 3-8 second SO(3) window, quality/bias/covariance handling,
then accepted external calibration. For `fc_full_state`, an internal `W0` may
start at zero while canonical output starts in non-zero `G_nav`; the other two
modes remain explicitly local. Position, velocity, and attitude are evaluated at
the final initialization camera timestamp. Moving single-frame bias estimates
are not trusted.

Exit gate: synthetic timing tests pass, raw `W0` obeys the origin contract, and
every flight is non-inferior to I0 at 30 s, 60 s, first turn, and full duration.
The accepted P4 path must also pass the P1-runtime-conformance gate on real-flight
`canonical_init_state` records.
After clean P4, rerun only the minimum registered representative conditions needed
to update the quarantined calibration/OC/stride conclusions.

### P5 — Overlap-Bounded Adaptive Stride

Start from P2 outputs. Compare three offline shadow policies in parallel:
`geometry_only`, `geometry_plus_rotation`, and
`geometry_plus_feature_health`. Separate tracking cadence from backend update
cadence, use actual gap/`dt`, and log reason codes, dwell time, switching rate,
predicted overlap, and safety margin. Retain at most two policies for active
validation. Do not rerun fixed-stride controls.

Exit gate: all four full flights pass per-flight non-inferiority as the primary
acceptance, estimator diagnostics improve, and the real-time budget passes. The
new active experiment budget is four flights times one candidate, or at most four
flights times two candidates. Terminal/landing and every height band and turn
direction are reported as separate strata; they cannot be cropped out to pass the
primary result, and no pooled average may hide a failed flight.

### P6 — Unified Data Product And Final Acceptance

Deliver one versioned four-flight data product that binds canonical output,
local diagnostic output, selected initialization mode, FC/GPS/reference fields,
camera geometry, footprint/overlap, requested and actual stride, tracking/backend
health, errors, segment labels, provenance, and gate status. Extend the existing
181-column schema rather than maintaining a parallel ad-hoc table.

Exit gate: schema, Parquet, CSV export, documentation, and dashboard payload have
identical field semantics; all six goals have traceable evidence; failed or
inconclusive flights remain explicit; and the final report updates every
quarantined 2026-06-24 conclusion. Recovery remains out of scope.

## Progress Snapshot — 2026-07-11

- The P0 provenance sub-gate passed on the corrected fly3 fixed-stride12 replay.
  The legacy 32-run binding index is now built, with missing historical atomic
  source-patch/dataset receipts retained as gaps. The corrected fly3 package captures
  `56,115` dataset files (`3,497,654,589` bytes), preserves all 11 untracked
  source/config files in `untracked_source.tar` (SHA256
  `12b4d15caab90f5d25ac6ae69b7d0edf14a74dde5bc72387eaa6d620bd97e741`),
  binds the passing machine-readable post-run verification, and seals
  `run_receipt.json` (SHA256
  `9833d5f1fe0513289b77f3086155a1e29e84c410863f9087ba2476f7568da29a`).
  Independent shallow receipt verification passes after derived analysis was
  added.
- As a legacy P0 reproduction diagnostic, the complete replay reached the
  dataset-end event at `t=1856.041 s`. With the historical terminal-landing
  exclusion, the fly3 diagnostic window covers
  `40.699 km`, has no `1000 m` divergence, ends at `167.763 m` XY error, and has
  `192.761 m` XY RMSE.
- The earlier P2 event timeline is retained as a terminal-flight diagnostic, not
  as a P2 gate or a proven root cause. It creates no stride1 replay requirement.
- The historical v2 P1 replay passed all 36 then-registered frame-validation checks with 36,512
  raw/nav/bias rows. Maximum position/rotation/velocity reconstruction residuals
  are about `1.03e-9 m`, `8.97e-17 rad`, and `4.04e-14 m/s`.
- The historical P1 schema artifact before the startup extension contains 36,512 rows and 159 columns; authoritative
  Parquet, CSV, and generated schema columns match exactly. Every row records a
  passing frame gate and complete run provenance.
- P1 startup serialization passed a real visual diagnostic
  replay with 3,387 rows. `nav-frame-v2` records
  `t_trim=618.013596535 s`, `t_seed=618.034715652 s`, and
  `t_emit=619.668312788 s`; seed position is exactly zero and the first emitted
  position/velocity/quaternion agree with persisted output at about `1e-10`.
  The live master schema has 181 fields and its diagnostic Parquet/CSV copy has
  all 14 startup-group fields populated. A later full v3 receipt/master also
  passed its implemented checks. P1 is nevertheless reopened. The current v4
  validator has 54 checks and correctly permits a non-zero mapped navigation
  position while retaining an internal zero-origin `W0`, which is not itself a
  contract violation. It still covers only two position-frame scaffolding cases
  and does not validate the three initialization modes, `canonical_init_state`,
  or exact-time FC runtime conformance.
- The compact organized delivery, the Desktop copy, and the ZIP have identical
  97-file payloads. They contain 28 OC trajectories for strides
  `2,4,8,12,16,20,30` plus four `origin_stride4` trajectories, but omit the four
  OC stride1 runs and all per-run diagnostics. The complete sibling tree contains
  all 32 OC controls, with matching runner/config/calibration identities and
  legacy canonical-metric outputs under the unaccepted FC-init lineage.
- The read-only P2 inventory at
  `C:/Users/baloney/Desktop/实验目录/P2_overlap_existing_sweep_20260711/20260711_fourflight_fixed-stride_offline_partial`
  contains 32/32 file-complete, exit-zero indexed runs and aligned samples in
  96/96 initial-straight/first-turn/post-turn rows; 88/96 pass the registered
  duration/distance/sample support gates. It records 23/32 registered 1000 m
  divergences separately from file completeness, only 8/32 complete requested
  evaluation windows, eight fly1 GPS-reference-truncated windows, 28 matching
  compact trajectories, four allowed compact stride1 omissions, and four
  non-contract `origin_stride4` extras. All 32 FC-init files are labeled legacy
  single-row artifacts; raw FC attitude is explicitly `not_indexed`. No
  short-segment realignment is performed. Its status is `NOT_FORMAL`: no overlap
  envelope or maximum-safe-stride lookup was generated.
- File completeness is not estimator health: 23/32 legacy canonical-metric outputs hit the
  registered 1000 m divergence trigger and only 9/32 do not. Fly2 stride1 is a
  particularly important failed control (`t=916.84 s`). The legacy
  `FULL_RATE_STRIDE1_BASELINE.csv` reports pathological full-window values
  (`95.82 deg`, `1040.76 m/s`), while the legacy diagnostic pre-divergence crop reports
  `1.13 deg` and `1.52 m/s` through `t=846.84 s`. P2 must preserve both the
  failure and the crop semantics; it may not pool this row as a healthy
  full-window sample.
- A controlled FLY3 comparison already exists with identical binary and config
  hashes. On the initial straight, stride1 accumulates `11.21 m` local XY error
  over `3237.44 m`, while stride12 accumulates `96.20 m` over `3170.73 m`. On the
  first connector/turn, local error-vector drift remains `47.06 m` at stride1
  and `57.86 m` at stride12 over the same `1586.81 m`. Turn course RMSE is
  `1.66 deg` versus `2.86 deg`; median tracked features are `332` versus `284`,
  and MSCKF acceptance is `99.99%` versus `98.72%`. In this legacy FLY3
  pair/window, stride12 is worse, but the first-turn local drift also persists at
  stride1 and is not accompanied by an aggregate feature-count or acceptance-rate
  collapse.
- Every legacy first-turn row in this snapshot reuses the full-flight
  start-heading alignment. Its first evaluation position error is zero by
  construction, so a small early straight error is not proof that FC
  initialization is correct. Error-magnitude change and ENU error-vector drift
  must both be reported; for fly3 stride12 the first-turn magnitude falls by
  `17.58 m` while the vector still moves `57.86 m`, and for fly4 stride12 the
  magnitude changes only `0.40 m` while the vector moves `7.48 m`.
- The canonical-default alignment-window reconstruction overlaps fly1's first
  turn in all 8/8 strides, and all fly1 initial-straight phases fail at least the
  five-second duration support gate. Fly1's early-small-error observation is
  therefore not an independent initialization test. The fly2 stride1 post-turn
  row is also explicitly `cropped_partial` at 56.25% of its registered COMMON
  phase; it cannot be read as recovery. Processed-update diagnostics match all
  first-turn timestamps, so repeated inter-frame `diag.csv` snapshots no longer
  inflate chi-square counts.
- In the current FLY3 P1 fixed12 artifact, a provisional diagnostic over the
  pre-turn straight (`680.4--692.6 s`) and first turn (`692.8--747.2 s`) shows:
  first-turn XY
  velocity-vector RMSE rises from `0.62` to `2.21 m/s`, mean P95 visual residual
  from `0.68` to `3.50 px`, and mean P95 flow from `32.25` to `39.54 px`, while
  median tracked features remain `300` and the aggregate chi-square acceptance
  remains `98.53%`. This is consistent with turn/rotation-sensitive measurement or
  state-model stress, not yet a causal attribution. Clean P4 must correct timing and
  initialization before the old global calibration conclusion is updated.
- P2 geometry inputs are incomplete for an accepted threshold. Intrinsics,
  distortion, camera--IMU transform, images/timestamps, raw FC attitude outside
  the current index, and GPS velocity exist, but there is no common formal
  AGL/ground-plane or DEM input, no release-accepted P1/P4 FC--camera exact-time
  attitude chain, and no complete actual-parallax log. A planar footprint result
  can only be labeled diagnostic.

## Updated Task Order And Completion State

1. Completed 2026-07-11: generated the 32-row existing-sweep binding index from
   the complete sibling tree and linked all 28 compact fixed-stride trajectories;
   the four omitted stride1 runs and file-complete/divergence states are explicit.
2. Completed 2026-07-12: froze readonly audit v6 and stopped FC-attitude-difference
   turn-flex expansion.
3. Completed 2026-07-12: implemented P1 runtime modes and P4 I0--I3; all clean
   short/full runs pass the runtime frame contract. No P3 correction is used.
4. Corrected 2026-07-12: I2 improves the registered short window. The prior
   rejection is withdrawn because its whole-dataset endpoint included the
   post-touchdown ground segment. Formal four-flight airborne evaluation remains pending.
5. Completed 2026-07-12: ran C0/C1/C2 isolation with only K/D changed; neither
   C1 nor C2 passes per-flight non-inferiority for global adoption.
6. Completed 2026-07-12: generated all three P2 provisional shadow policies from
   the existing 32-run sweep without estimator replay or active release.
7. Completed 2026-07-12: updated the authoritative master with real clean-P4 and
   1 Hz deterministic P2 evidence. The full P2 series remains in its dedicated
   parquet.
8. Outstanding: complete same-configuration airborne evaluation for all four
   flights, correct I2 causality, identify the in-air error mechanism, revalidate P2, and
   only then consider P5 active candidates.

P2 does not authorize a repeated 4x8 fixed-stride sweep. P4 I0--I3 do not wait
for P3; only I4 requires accepted P3 calibration. P5 active
validation remains prohibited until P2-release passes. Recovery experiments are
outside this contract at every gate.
