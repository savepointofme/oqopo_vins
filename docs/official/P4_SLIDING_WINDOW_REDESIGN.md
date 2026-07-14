# P4 Sliding Window Redesign

## Scope

P4-R1 replaces the active runtime path that freezes one nonlinear candidate with a shadow-only fixed-time sliding window. It does not release or inject any state into OpenVINS. P4-R2 state-group release and shadow confirmation are explicitly out of scope until R1 passes on fly1 and fly3.

Frozen comparison source:

- Git snapshot: `e1f6c2b153f373619a638917ff5efd75aa36b331`
- Branch: `snapshot/p4-pre-sliding-window-redesign-20260714`
- Existing v6 runtime artifacts remain comparison-only.

Implementation branch: `redesign/p4-sliding-window-r1-20260714`.

## R1 runtime contract

```text
new causal FC / board IMU / selected visual frame
  -> append raw data
  -> retain only the fixed window plus interpolation margin
  -> form [window_end - W, window_end]
  -> require window_end to advance by at least delta_t
  -> select current-window keyframes
  -> rebuild a bounded Ceres problem
  -> warm start overlapping states from the previous solved window
  -> solve q/p/v/bg/ba and nuisance landmarks
  -> record the window solution and confidence diagnostics
  -> discard the Ceres problem
  -> keep alignment active and wait for the next window
```

Production R1 settings are architectural controls, not release gates:

- fixed window duration `W = 8.0 s`;
- minimum window-end advance `delta_t = 0.5 s`;
- maximum Ceres keyframes remains `10` only as a bounded graph-size limit;
- no fixed FC update count controls solve or release;
- no candidate filter participates in R1;
- no result is marked `released_to_openvins`;
- `alignment_window_closed_` remains false throughout R1.

## Warm start

The previous window stores every optimized keyframe state. A new keyframe whose timestamp lies inside the previous solved span is initialized by timestamp interpolation of q/p/v/bg/ba. A new tail keyframe outside that span uses the current FC/IMU seed for q/p/v and carries the latest solved bg/ba. Warm start never feeds the formal OpenVINS state.

## Per-window evidence

Every attempted nonlinear window records:

- window id, begin/end timestamp and fingerprint;
- raw/valid FC count, IMU count, visual-frame count and selected keyframe timestamps;
- optimizer invocation index, solve wall time, initial/final cost and solve status;
- final-window q/p/v/bg/ba estimate and 15-state standard deviation;
- IMU, FC and visual residual summaries;
- angular excitation and second-axis ratio;
- q/p/v/bg/ba delta from the previous window;
- whether warm start was used.

The runner writes these records to `online_alignment_sliding_windows.csv` and the existing attempt-receipt JSON. Old candidate update counters are not R1 evidence.

## R1 acceptance

R1 is KEEP only if both fly1 and fly3 demonstrate:

1. optimizer invocation count increases beyond one;
2. window begin and end timestamps strictly advance;
3. selected keyframe timestamps change and remain inside each fixed window;
4. stale timestamps leave later windows and new timestamps enter;
5. each window reports new q/p/v/bg/ba estimates and standard deviations;
6. no duplicate window fingerprint is solved;
7. no formal OpenVINS initialization or feedback occurs;
8. raw buffers and graph size remain bounded;
9. no fixed update count controls R1 execution.

Any run with mismatched binary provenance, an empty/missing window trace, or only one optimizer invocation is INVALID.

