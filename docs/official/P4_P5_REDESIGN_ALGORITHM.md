# P4/P5 persistent sliding-window and target-parallax redesign

## 1. Objective

Refactor the existing FC navigation + board IMU + monocular KLT P4 joint initializer into a persistent sequence of finite, bounded solve windows. Replace P5's fixed state-to-stride mapping as the primary scheduling algorithm with a shared target-parallax planner and accumulated-information backend trigger.

The redesign keeps the existing OpenVINS tracker, `FeatureDatabase`, IMU propagation, FEJ, covariance, MSCKF, and SLAM architecture.

## 2. Non-negotiable estimator contract

- P4 keeps FC position, velocity, and attitude factors.
- P4 keeps board-IMU `CpiV1` preintegration factors.
- P4 keeps monocular multi-frame triangulation and reprojection factors from the existing KLT database.
- P4 keeps startup FC-to-board orientation and time relation.
- P4 estimates `q/p/v/bg/ba` and returns the 15×15 navigation covariance plus per-state observability.
- No upstream visual-IMU-only initializer or nearest-FC-row fallback is reachable in online-alignment mode.
- Camera–IMU `T_C_I`, intrinsics, distortion, and Camera–IMU time offset remain locked.
- No 7-degree, 4.089-degree, permanent mounting, or time-varying flex correction is added.
- A practical navigation release may use a startup FC-to-board orientation/time prior only when that prior is explicitly accepted by the runtime contract. With the current P3 status, a window that fixes either startup relation to an unaccepted prior returns to `COLLECTING`; it cannot release `NAVIGATION_READY` as `0 s / 0 deg`.
- P4 releases once, closes after the applied release, and cannot be re-entered by P5.

## 3. P4 lifecycle

```text
WAIT_INPUTS
  -> COLLECTING
  -> WINDOW_NOT_READY          (cheap checks only)
  -> SOLVING                   (bounded current factor graph)
  -> CANDIDATE_READY
  -> VALIDATING_CANDIDATE      (fixed candidate, no shadow filter)
  -> [optional one-shot REFINING_CANDIDATE]
  -> FINAL_ALIGNMENT_READY
  -> CLOSED_AFTER_RELEASE

unrecoverable frame/time/calibration contract error
  -> FATAL_CONFIGURATION_ERROR
```

Ordinary data insufficiency, residual failure, covariance failure, or observability failure returns to `COLLECTING`. No elapsed-time threshold closes the initializer while valid streams continue.

## 4. Adaptive finite solve-window selection

Supported durations are the 2 s upstream-reference window plus `{3, 5, 8, 12}` seconds. The selector evaluates them in ascending order and chooses the shortest duration satisfying coverage and information preconditions.

For each duration it computes:

- FC/IMU/camera coverage and maximum gap;
- selected alignment-frame count;
- common feature tracks and survival;
- raw and rotation-compensated parallax percentiles;
- camera translational baseline and triangulation angle/conditioning proxies;
- IMU angular and specific-force excitation;
- FC attitude and velocity excitation;
- expected factor counts, state dimension, rank proxy, and solve-time proxy.
- whether the startup FC-to-board orientation and time relation come from current-window evidence or only from an unaccepted fallback prior.

Waiting duration never changes this candidate list. The largest retained live buffer is:

```text
[latest sensor time - 12 s - interpolation/time-offset margin, latest sensor time]
```

Hard caps apply independently of window duration:

- selected alignment frames / pose states;
- selected feature IDs and landmarks;
- observations per landmark;
- FC, IMU, and visual residual blocks;
- Ceres iterations and wall time.

## 5. Attempt eligibility and fingerprint

Cheap window assessment runs when a new selected alignment frame arrives. Ceres runs only when all preconditions pass and the new window has material information relative to the last attempt.

The fingerprint includes:

```text
window start/end
selected frame timestamps
selected feature-ID hash
selected FC first/last timestamps and count
selected IMU first/last timestamps and count
parallax/baseline/excitation summary
expected factor-count summary
```

An identical fingerprint is never solved twice. A changed fingerprint is eligible only after a minimum attempt interval and at least one information-gain trigger: accepted alignment frame, higher compensated parallax, larger baseline, better track survival, more triangulatable landmarks, higher IMU/FC excitation, or improved rank/observability proxy.

Rejected attempts retain only counters, reason histograms, best residual/observability summaries, and compact attempt receipts.

## 6. Shared visual geometry

`VisualCadencePlanner` owns the shared definitions used during P4 collection and P5 operation.

For common feature IDs between two actual tracking frames:

```text
x_rot = project(R_Ccur_Cprev * unproject(x_prev))
d_comp = ||x_cur - x_rot||
```

It reports raw, rotational, and compensated median/P75/P90/P95 displacement; common-track count; survival; track age; actual raw-camera interval; actual tracker interval; and accepted-frame interval.

For each integer candidate tracking stride it predicts compensated translation flow, rotation flow, total P95 flow, feature survival, overlap availability/value, and interval. It selects the largest candidate inside the target-parallax band and every active safety cap. A smaller stride is immediate; a larger stride requires stable confirmation and hysteresis.

## 7. Alignment frame selection

`AlignmentFrameSelector` sees only frames already processed by the existing KLT tracker. It selects a tracking frame for the P4 solve snapshot when one of these occurs:

- direct compensated parallax from the last selected frame reaches target;
- baseline or visual-information proxy reaches target;
- common-track survival starts falling;
- maximum selected-frame interval expires;
- a new motion/turn excitation is informative;
- the latest solve-window endpoint needs a terminal frame.

Only selected frames enter P4's expensive solve snapshot. The selector keeps a fixed maximum frame count and never creates a VINS-Mono keyframe manager.

## 8. Candidate and bounded validation

`AlignmentCandidate` stores the solved navigation state and covariance, startup relation, selected solve-window provenance, selected frames/features, accepted landmarks, factor/residual statistics, observability, prior-dominated states, solve time, and fingerprint.

The candidate is not immediately injected. It remains fixed while approximately 2 s of future FC, board-IMU, and monocular tracking data are collected. The validation duration is bounded; if minimum evidence is unavailable at 2 s it may extend only to a configured short maximum.

Validation does not create another OpenVINS instance or clone/FEJ/covariance tree. A lightweight nominal propagation of the fixed candidate uses board IMU. At validation timestamps it computes:

- FC-derived attitude/position/velocity residuals;
- IMU-propagated state drift;
- future reprojection residuals for retained candidate landmarks;
- P50/P95, trend, and covariance-normalized residuals;
- `q/p/v/bg/ba` proxy stability;
- startup misalignment stability from future rate pairs.

An unstable candidate is discarded and collection resumes. A candidate whose startup relation is fixed to an unaccepted prior is also discarded before release, even if its short holdout residuals happen to pass. A stable candidate satisfying the registered startup-relation contract is released directly or refined once.

## 9. Optional one-shot refinement lifecycle

The lifecycle and counter permit zero or one refinement. The production configuration currently selects zero: `candidate_refinement_enabled=false`. A previous whole-graph re-solve was not accepted as a true bounded local candidate correction because it did not independently prove correction magnitude, covariance consistency, and validation improvement. It therefore remains disabled rather than being presented as a completed refinement algorithm.

If a future registered local correction is enabled, it must reuse only the bounded candidate/validation snapshot and obey the same frame, landmark, observation, residual, iteration, and time caps. There is no periodic refinement, post-release solve, shadow filter, filter swap, or OpenVINS reset.

## 10. Provisional navigation output

Before final P4 release, a separate `ProvisionalNavigationOutput` stream provides an explicitly non-VIO diagnostic state. It uses an exact-time FC state supported by the causal buffer, applies the declared FC-to-board mapping and lever arm, and propagates with board IMU to the output timestamp. It includes a conservative covariance, frame, timestamps, provenance, `provisional=true`, `openvins_initialized=false`, and the current pending reason.

It is written separately from the formal OpenVINS trajectory and is never injected into the filter.

## 11. P5 target-parallax tracking

After P4 closes, the same `VisualCadencePlanner` selects tracking cadence from actual compensated-flow rate, camera time, IMU camera rotation, KLT flow limits, feature survival, overlap, visual health, and maximum interval.

Discrete flight-state labels remain diagnostic safety context only. Height, descent, turn, and visual-health signals produce upper caps on the selected stride; they no longer directly select a fixed stride pair.

The final tracking choice is:

```text
tracking_stride = min(
  target_parallax_selection,
  rotation_cap,
  overlap_cap when available,
  visual_health_cap,
  maximum_interval_cap)
```

## 12. Ground-footprint overlap cap

When AGL and a local ground plane are valid, the overlap estimator projects the four calibrated image-corner rays through fixed `T_C_I` and current attitude onto the ground plane. It advances the camera using candidate `dt` and the current velocity vector, builds the predicted footprint polygon, and computes polygon intersection area divided by current footprint area. AGL source, timestamp, age, uncertainty, and validity accompany every decision.

If the ground intersection is invalid or AGL is unavailable, overlap is explicitly unavailable and is not replaced by the previous one-dimensional footprint approximation.

## 13. Accumulated-information backend trigger

After every admitted tracking frame, `BackendUpdateTrigger` directly compares that frame with the last backend frame using common feature IDs and camera rotation compensation. It triggers a backend update on any of:

- compensated parallax target;
- visual-information proxy target;
- feature-survival loss;
- maximum backend latency;
- low-altitude, turn, or visual-degraded safety event;
- covariance/state-jump fault;
- update starvation;
- long tracks approaching the active clone/track limit.

The frame counter is retained only for diagnostics and a maximum-latency fallback. It is not the primary trigger.

Tracking-only frames update the existing KLT image, feature positions, and feature IDs, but create no clone and enter no MSCKF/SLAM update. If the current architecture writes their observations first, exact-timestamp cleanup remains mandatory.

## 14. State application

Only a validated final candidate enters `VioManager::initialize_with_online_alignment()`. The applied result is stored separately from later diagnostics. The function writes nominal state, FEJ, 15×15 covariance, initialization timestamp, feature cleanup boundary, and propagation-cache state exactly once. It then marks VIO initialized and closes/clears all P4 high-cost input.

## 15. Verification contract

Verification must demonstrate:

- current FC, IMU, and monocular visual factors remain nonzero;
- no upstream or nearest-row fallback exists;
- retry continues beyond 20 s and 60 s with bounded buffers and no elapsed-time shutdown;
- duplicate fingerprints do not solve twice;
- solve size and attempt rate remain bounded;
- candidate validation rejects unstable candidates and releases stable candidates once;
- refinement count is zero or one;
- post-release solve count is zero;
- target-parallax decisions respond to compensated flow and IMU rotation;
- height alone cannot select an exact stride;
- backend updates are explained by accumulated information or an explicit safety/latency trigger;
- tracking-only frames create no clone and are not reused by MSCKF/SLAM;
- fly1/fly3 comparison retains real-time throughput and produces no post alignment.

## 16. Implementation order

1. persistent P4 lifecycle and bounded pruning;
2. adaptive solve-window assessment;
3. attempt fingerprint and information-gain eligibility;
4. shared visual geometry and initialization cadence;
5. alignment-frame selection;
6. candidate validation and optional refinement;
7. provisional output;
8. target-parallax P5 tracking;
9. accumulated-information backend trigger;
10. registered tests, existing 32-run threshold extraction, and fly1/fly3 validation.
