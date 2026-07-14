# P4 one-shot causal alignment and realtime contract

Status: supersedes the post-release observation behavior in
`P4_ARBITRARY_TIME_INITIALIZATION_CONTRACT_20260712.md`.

## Problem being corrected

The practical online path released a valid initialization result to OpenVINS,
but continued to send every later camera frame into the nonlinear joint
alignment solver while waiting for a stronger `FULL_ALIGNMENT_READY` label.
That behavior is not initialization. It made the finite startup problem run for
the rest of the replay, produced thousands of solve attempts, and invalidated
realtime conclusions drawn from those runs.

## Runtime contract

1. One startup attempt owns a finite causal collection interval. Its rolling
   solve window is eight seconds and it may collect for at most twenty seconds
   from the first accepted board-IMU sample.
2. Before release, data-quality failures may move the state machine to
   `FAILED_WAIT_RETRY`; a later causal window within the same twenty-second
   startup interval may retry.
3. The first result that passes the configured release policy is copied once
   into an immutable `AlignmentResult` and atomically released to OpenVINS.
4. Release closes the startup interval. FC, board-IMU, and monocular feature
   samples after the release time are not accepted by the initializer. The
   nonlinear solver and covariance recovery are never called again during that
   process lifetime.
5. If no result passes before the twenty-second collection deadline, the
   startup interval closes in `FAILED_WAIT_RETRY`. A new attempt requires an
   explicit initializer reset; continuing camera frames cannot silently grant
   unlimited solve time.
6. `NAVIGATION_READY` is a final release outcome for the practical policy, not
   an invitation to keep optimizing in the background. A later
   `FULL_ALIGNMENT_READY` upgrade is prohibited because it would use data
   outside the released initialization interval.

## Realtime diagnostics

Every metadata record must report:

- `nonlinear_solve_attempt_count`: actual entries into the Ceres joint solve;
- `successful_release_count`: zero or one;
- `post_release_try_count`: direct API calls attempted after release;
- `alignment_window_closed` and `alignment_window_close_time_s`;
- the causal `window_start`, `t_init`, and decision time.

For a successful replay, the acceptance gates are
`successful_release_count == 1`, `post_release_try_count == 0`, and no solver
entry after the decision time. Counting state-transition strings is not an
acceptable substitute for these counters.

## Frontend and estimator controls

- The selected frontend remains the configured monocular KLT/LK optical-flow
  tracker (`use_klt: true`, `use_stereo: false`).
- The June-12 fixed camera intrinsics, lens distortion, and Camera-IMU
  transform remain locked.
- No approximately 7-degree, 4.089-degree, permanent FC-to-board, or
  time-varying flex correction is added.
- Dashboard rendering is measured separately from estimator work.
- Feature-count and SLAM-landmark limits may be investigated only after the
  post-release solver bug is removed and the same replay is measured again.

## Verification

1. Unit test a navigation-only practical release, then feed more FC, IMU, and
   monocular frames and call `try_initialize` again. Buffers, released result,
   phase, and nonlinear-solve count must not change.
2. Unit test that timeout closes the collection interval until explicit reset.
3. Build the ROS-free runner and run the online-alignment and adaptive-stride
   tests.
4. Run fly1 and fly3 short replays one process at a time with dashboard work
   disabled. Compare actual solver counters, wall time, sensor time, KLT call
   count, backend update count, feature database size, and SLAM feature count.

