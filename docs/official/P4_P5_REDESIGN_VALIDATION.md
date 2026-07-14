# P4/P5 Persistent Sliding-Window and Target-Parallax Validation

## 1. Disposition

Final status:

`P4_P5_REDESIGN_IMPLEMENTED_BUT_VALIDATION_FAILED`

The requested architecture is implemented and both fly1 and fly3 complete a validated one-time P4 release followed by a full P5 replay. The implementation evidence is strong enough to prove the new lifecycle, bounded solve, target-parallax tracking, information-triggered backend, and Dashboard cost isolation. It is not yet strong enough to authorize four-flight acceptance for four reasons:

1. absolute airborne navigation error remains large: fly1/fly3 XY RMSE is 737.4/1170.9 m without post alignment;
2. fly1 is worse than the available historical fixed-8-second joint-initialization full-flight reference (737.4 m versus 432.5 m XY RMSE), while no matching fly3 old-joint full-flight artifact exists for a two-flight non-inferiority decision;
3. the visible Dashboard path is not realtime (`wall/sensor=1.240/1.172`), although the identical Dashboard-disabled estimator path is realtime (`0.817/0.766`);
4. the registered legacy 4-flight × 8-stride evidence does not contain common-ID compensated pixel flow, feature survival, the new backend information proxy, or candidate holdout metrics. Those thresholds therefore cannot honestly be described as leave-one-flight-out fitted; the current pixel/candidate thresholds remain conservative implementation defaults.

This is a validation result, not a confirmed implementation blocker. The redesigned mechanisms run to completion, preserve the estimator architecture, and produce reproducible receipts.

## 2. Experiment differences

| Condition | P4 solve-window policy | Initialization image cadence | Candidate validation | Post-release P5 | Dashboard |
| --- | --- | --- | --- | --- | --- |
| Historical current P4 control | fixed 8 s | fixed stride12 | none | fixed stride12 or old state-machine path | disabled |
| Redesigned P4/P5 full replay | shortest usable 2 s upstream-reference/3/5/8/12 s sliding window | shared rotation-compensated target-parallax planner | fixed candidate, bounded 2–4 s causal holdout | target-parallax tracking plus information-triggered backend | visible |
| Realtime isolation | identical to redesigned full replay | identical | identical | identical | fully disabled |

All redesigned replays use the same June-12 locked camera intrinsics, lens distortion parameters, Camera–IMU `T_C_I`, and Camera–IMU time offset. Camera calibration is not estimated online. No approximately 7-degree, 4.089-degree, permanent FC-to-board, or time-varying flex correction is applied. Repair, restart, and anchor reset remain off. GPS horizontal position/course do not enter the lateral estimator; registered past GPS altitude is used by the existing height update and P5 safety-cap provenance.

## 3. What was implemented

### 3.1 P4 joint factor graph is preserved

`OnlineAlignmentInitializer` still solves one graph containing:

- FC navigation position, velocity, and attitude residuals;
- board-IMU `CpiV1` preintegration residuals;
- monocular KLT multi-frame triangulation and reprojection residuals;
- startup FC-to-board orientation/time relation;
- `q/p/v/bg/ba`, 15×15 navigation covariance, mount covariance, and per-state observability.

The final fly1 graph contains 10 pose states, 80 landmarks, 9 IMU blocks, 524 monocular reprojection blocks, and 10 FC blocks. The final fly3 graph contains 10 pose states, 80 landmarks, 9 IMU blocks, 395 monocular reprojection blocks, and 10 FC blocks. All three sensor families have nonzero residual dimensions and Jacobian norms.

Online-alignment mode cannot call the upstream visual-IMU-only initializer and cannot use nearest-FC-row initialization. Upstream OpenVINS contributes only the finite-window, pruning, retry, and single state/FEJ/covariance application mechanisms documented in `CURRENT_P4_VS_OPENVINS_SLIDING_WINDOW_MECHANISM.md`.

### 3.2 Persistent finite windows

The live buffers retain only the maximum 12 s candidate window plus 0.75 s interpolation margin and the accepted FC-to-board time-offset margin. Candidate durations are tested in ascending order `{2(reference),3,5,8,12}` and the shortest currently usable window is selected. Waiting duration does not enlarge this set.

Each solve remains bounded by 10 selected solve states, 80 visual features/landmarks in the registered replay configuration, 30 Ceres iterations, and 2 s solver time. The alignment-frame snapshot itself is bounded at 36 frames and a maximum 0.60 s selected-frame interval. Rejected attempt-local states, landmarks, residuals, and Jacobians are destroyed; only compact receipts and counters remain.

There is no elapsed-time shutdown. The compatibility field `navigation_max_collection_s` is infinite and ignored. Coverage, gap, excitation, triangulation, residual, covariance, and observability failures reject only the current window. Stream end and fatal frame/time/calibration/input-contract errors remain explicit terminal outcomes.

### 3.3 Solve attempt eligibility

Ceres requires all cheap gates, a changed window fingerprint, at least three new selected alignment frames, and at least 0.75 s since the previous solve. The fingerprint covers the solve time range, selected frame timestamps, feature-ID hash, FC/IMU ranges, excitation summary, and expected factor counts. Identical fingerprints are skipped.

Every recorded full-replay solve is triggered by `new_alignment_frames=N`, `window_fingerprint_changed=true`, and a registered attempt interval. fly1 performs 9 solves: 8 unsuccessful windows precede the final release, including 4 candidates rejected by holdout validation. fly3 performs 4 solves: 3 unsuccessful windows precede release, including 2 rejected candidates. Post-release calls and solves are zero.

### 3.4 Candidate validation

The solved candidate is not injected immediately. It remains fixed while later FC, board-IMU, and monocular KLT observations are collected. Lightweight IMU propagation evaluates relative FC attitude/position/velocity, propagated-state drift, retained-landmark reprojection P50/P95/trend, and covariance-normalized residual. It does not create a shadow OpenVINS, clone tree, FEJ tree, or second backend.

| flight | validation | frames | FC/IMU rotation | relative position | relative velocity | visual P50/P95 | visual trend | normalized residual |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fly1 | 2.001 s | 29 | 0.496° | 1.887 m | 1.192 m/s | 1.340/3.921 px | 1.994 px/s | 1.510 |
| fly3 | 2.001 s | 29 | 0.397° | 4.730 m | 1.922 m/s | 0.829/0.829 px | 0.000 px/s | 3.248 |

Both accepted candidates are propagated to the validation-end release timestamp before injection. Refinement count is zero on both flights. Production refinement is intentionally disabled until a true bounded local correction with correction-size, covariance, and improvement gates is registered; the required lifecycle limit of zero or one is enforced. Some rejected-candidate receipts retain the requested next action text `single_bounded_refinement`; with `candidate_refinement_enabled=false` this is a request label, not an executed refinement, and collection actually resumes on a new window.

### 3.5 One release and state application

The validated result enters the existing `VioManager::initialize_with_online_alignment()` path once. It writes the OpenVINS nominal IMU state, FEJ value, 15×15 covariance, and timestamp, cleans pre-release feature measurements, invalidates propagation cache, marks VIO initialized, and closes/clears P4. Subsequent FC, board-IMU, and image data do not enter P4. Higher P5 tracking cadence cannot reopen it.

## 4. Shared visual cadence and alignment-frame selection

`VisualCadencePlanner` directly compares common feature IDs at actual tracking timestamps. It separates raw image flow, board-IMU/fixed-`T_C_I` pure-rotation flow, and rotation-compensated translation parallax. Candidate strides are `{1,2,4,6,8,12}`. The largest safe stride is selected subject to:

- target compensated median parallax 2–8 px;
- compensated P95, total P95, rotation P95, and image-edge rotation limits;
- feature survival at least 0.55;
- tracking interval no greater than 0.40 s;
- polygon ground-footprint overlap at least 0.90 when a valid AGL/ground intersection exists;
- visual-health and safety caps.

Downshift is immediate. Upshift requires four new KLT measurements over at least 0.5 s and 1.25 ratio hysteresis. A skipped raw frame cannot be counted as a new parallax observation.

`AlignmentFrameSelector` is not another tracker or keyframe manager. It selects an already tracked frame for P4 only when compensated parallax, translation/information, survival loss, excitation, maximum interval, or solve-window endpoint justifies it. Frames not selected continue to support KLT continuity but do not create P4 pose states.

## 5. P5 target-parallax and backend behavior

Policy-state labels no longer select the main tracking/backend stride pair. The target-parallax planner chooses tracking stride; height, overlap, turn, angular rate, descent, and visual health only reduce the allowed gap or request safety work.

`BackendUpdateTrigger` compares current tracking observations directly with the last backend frame. It triggers on accumulated compensated parallax, information proxy, feature-loss risk, motion limit, long-track ending, maximum 0.5 s latency, or explicit safety/health events. A fixed 6/12-frame counter is not the primary rule.

| flight | raw frames | KLT frames | backend frames | tracking-only | information | latency | safety | clone violations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fly1 | 30,226 | 20,989 | 13,809 | 6,882 | 13,208 | 379 | 217 | 0 |
| fly3 | 36,552 | 23,750 | 17,643 | 5,947 | 17,025 | 213 | 400 | 0 |

The median compensated-parallax distribution is centered inside the 2–8 px target band: 4.390 px for fly1 and 4.524 px for fly3. The fifth-percentile feature survival is 0.600/0.952 and the median is 0.981/0.982. A tracking-only frame advances the existing KLT state and IDs, then removes its exact-timestamp `FeatureDatabase` observations before clone/MSCKF/SLAM; both full runs record zero clone violations.

## 6. Flight-state and height coverage

The primary window is airborne only. Touchdown is detected by the registered `airborne_auto` combined reference conditions; post-touchdown ground samples are excluded from navigation RMSE.

| flight | >50 m | 30–50 m | 20–30 m | <20 m | turn state | visual-degraded state |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| fly1 | 744.70 s | 16.97 s | 6.40 s | 14.81 s | 0 s | 127.28 s |
| fly3 | 1060.36 s | 22.18 s | 5.80 s | 16.81 s | 12.37 s | 194.24 s |

All airborne height decisions use a past-only GPS altitude sample with timestamp, age, uncertainty, and validity recorded. GPS XY/course remain evaluation/reference fields and are not P5 inputs.

## 7. Realtime and Dashboard isolation

| flight | visible wall/sensor | Dashboard-disabled wall/sensor | visible RSS | disabled RSS | estimator decisions identical |
| --- | ---: | ---: | ---: | ---: | --- |
| fly1 | 1.240 | 0.817 | 201,288 KiB | 152,956 KiB | yes |
| fly3 | 1.172 | 0.766 | 196,800 KiB | 149,980 KiB | yes |

Tracking, backend, trigger, P4 solve/candidate/release, and clone-violation counts are identical between the two modes. Absolute-navigation results are also numerically equivalent: fly1 final XY differs by 0.022 m and fly3 summary values are identical. Dashboard preparation/rendering is therefore the cause of the visible-path realtime miss in this experiment, not a change in estimator scheduling.

The deployable Dashboard-disabled path satisfies host single-stream throughput. The visible Dashboard path does not.

## 8. Absolute airborne navigation result

Primary evaluation uses GPS timestamps and `alignment_mode=absolute_navigation_no_post_alignment`. `R_Gnav_W0=I` and translation is zero; no post position, heading, or SE(3) fit is applied.

| flight | airborne duration | GPS distance | final XY | XY RMSE | max XY | final vertical | vertical RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fly1 | 765.6 s | 29.36 km | 727.5 m | 737.4 m | 1471.3 m | 127.8 m | 46.7 m |
| fly3 | 1094.6 s | 41.09 km | 388.1 m | 1170.9 m | 2858.4 m | 220.3 m | 124.1 m |

The result is not hidden by the successful initialization lifecycle. fly1 exceeds the available historical fixed-8-second online-joint full-flight XY RMSE (432.5 m) and final XY (272.6 m). fly3 improves over the historical nearest-FC-row artifact (1805.5 m XY RMSE), but that artifact is not a matching old-joint control. Two-flight navigation non-inferiority is therefore not established.

## 9. Existing 32-run leave-one-flight-out evidence

The existing `P2_PROVISIONAL_OVERLAP_TIMESERIES.parquet` is reused read-only: 1,376,008 rows, exactly four flights × eight fixed strides. No estimator sweep was rerun. Each fold fits three flights and evaluates the held-out flight.

Derived values include maximum tracking interval 0.40 s, minimum rotation-aware overlap 0.9065, minimum feature-health overlap 0.8092, minimum feature-health proxy 0.8405, and minimum tracked-feature count 255.

The evidence does not contain common-ID rotation-compensated pixel flow, direct last-backend-to-current flow, common-ID survival, the current information proxy, or candidate holdout residuals. Tracking target pixels, backend target pixels, feature-survival threshold, information threshold, and candidate-validation thresholds are explicitly marked `not_identifiable_from_legacy_32`; they are not relabeled as data-fitted results.

## 10. Registered verification

The current build and both registered focused tests pass:

- `test_online_alignment_initializer`: joint FC/IMU/monocular contributions, state/covariance/observability, no fallback, persistent retry beyond 20 s and 60 s, bounded pruning/problem size, fingerprint suppression, candidate rejection/retry, one release, zero post-release calls, stream-end/fatal semantics, and stale visual-tail pruning;
- `test_adaptive_stride`: 21 target-parallax, rotation compensation, hysteresis, safety-cap, polygon overlap, alignment-frame selection, information-trigger, exact FeatureDatabase cleanup, and no-clone lifecycle cases.

The beyond-20-second and beyond-60-second cases are synthetic deterministic coverage. The two real flights release at 16.944 s and 10.155 s, so they do not independently demonstrate late real-flight success.

## 11. Required questions

| # | Answer |
| ---: | --- |
| 1 | FC position/velocity/attitude, board-IMU preintegration, and monocular reprojection factors remain in one Ceres graph with nonzero residual/Jacobian contributions. |
| 2 | Upstream finite-window ownership, old-data pruning, failed-call retry, and one-time state/FEJ/covariance application are reused as lifecycle patterns only. |
| 3 | No upstream visual-IMU-only or nearest-FC-row fallback is reachable. |
| 4 | `{2(reference),3,5,8,12}` s is evaluated shortest first after cheap coverage/information checks. |
| 5 | fly1 released from 3 s; fly3's 3 s attempts failed and 5 s was the shortest passing window. Longer windows were unnecessary. |
| 6 | FC, board-IMU, selected visual snapshots, stale candidate data, and attempt-local landmarks/residual/Jacobian storage outside 12 s plus margin are deleted. |
| 7 | Yes for implementation/tests and observed RSS: graph caps are fixed and disabled-run RSS is 153/150 MiB. Long real waiting was not observed. |
| 8 | Eight/three solve windows failed before fly1/fly3 success; four/two of them reached and failed candidate validation. |
| 9 | Each solve receipt records new selected frames, a changed fingerprint, and the attempt interval; duplicate fingerprints are not solved. |
| 10 | Raw images are skipped when the largest safe target-parallax interval has not elapsed. |
| 11 | Tracked frames skip P4 Ceres unless direct alignment-frame parallax/baseline/survival/excitation/interval/endpoint criteria select them. |
| 12 | A fixed candidate is IMU-propagated and compared with later FC/IMU/monocular observations; no second filter is created. |
| 13 | Both accepted holdouts are 2.001 s because the minimum duration and evidence count were already satisfied. |
| 14 | Refinement did not occur; count and correction are zero. Production uses zero until a bounded local refinement is registered. |
| 15 | Validation needs only fixed-state propagation and residual evaluation; a shadow filter would duplicate OpenVINS lifecycle/state ownership. |
| 16 | Yes: one release, zero post-release calls/solves, closed buffers. |
| 17 | The planner predicts each candidate stride from actual compensated-flow rate and raw-camera time, then takes the largest stride inside the target band and every cap. |
| 18 | Full-run counts attribute backend updates to information, latency, or explicit safety triggers; information dominates. |
| 19 | They form rotation, polygon-overlap, maximum-interval, feature-survival, residual, starvation, covariance, and state-jump caps; they do not directly look up a fixed stride pair. |
| 20 | Yes for ordinary retryable failures; only success, stream end/user stop, or fatal input/frame/time/calibration contracts terminate. |
| 21 | Yes: P4 uses `VisualCadencePlanner` independently of post-release P5 mode; base `--camera-frame-stride 12` is only a registered command-line fallback value. |
| 22 | Yes as the primary algorithm. State labels remain only as safety context/caps. |
| 23 | Yes: the existing KLT tracker, `FeatureDatabase`, IMU propagation, FEJ, covariance, MSCKF, and SLAM ownership remain intact. |
| 24 | It adds unlimited total retry with bounded windows, shortest-window selection, solve fingerprints, pre-release holdout validation, target-parallax tracking, information-triggered backend, polygon overlap, and explicit receipts. It does not yet improve the available fly1 absolute-navigation reference. |
| 25 | No. The two-flight mechanics are validated, but navigation non-inferiority, real late-start coverage, complete data-fitted thresholds, and visible-path realtime are not all satisfied. |

## 12. Authoritative artifacts

- visible full replay: `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/runs/20260714_032226_full_visible_active_fly1_fly3`;
- Dashboard-disabled full replay: `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/runs/20260714_034919_full_disabled_active_fly1_fly3`;
- absolute airborne evaluation: `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/evaluation/`;
- leave-one-flight-out evidence: `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/thresholds/`;
- delivery tables, tests, and manifest: `C:/Users/baloney/Desktop/实验目录/P4_P5_redesign_20260713/delivery/`.
