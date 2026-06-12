# Previous Flight Analysis Task Audit

## Verdict

The previous task was only partially correct.

Completed:

- A reusable entry point and project skill existed.
- GPS-time sampling, ENU errors, tables, figures, and comparison mode existed.
- The historical package was indexed and analyzed.

Incorrect or incomplete:

- Start heading used `atan2(E, N)` but applied the result as a conventional XY
  rotation, reversing the required rotation sign.
- Segment drift divided absolute accumulated endpoint error by local segment
  distance, producing meaningless values for short segments.
- The root workflow emphasized multi-condition aggregation instead of one run.
- fly3/fly4 used older extracted condition folders instead of
  `artifacts/packages/traj_1e4_fly34_20260606.zip`.
- Experiment provenance and local interactive inspection were missing.

## Fixes

- Correct ENU course rotation with `atan2(vN, vE)`.
- Compute segment drift from the change in the XY error vector over the segment.
- Do not report drift percentage for segments shorter than 50 m.
- Add experiment-linked `--run-spec` single-run analysis.
- Support direct `package.zip::member/path` trajectory input.
- Store analysis under the canonical experiment run directory.
- Add complete metric statistics, provenance, reference checks, mandatory
  PNG/SVG figures, and an interactive HTML dashboard.

## Revalidation

- Synthetic -90 degree alignment test final XY error: about `1.2e-14 m`.
- fly3 OC2 segment coverage: 5,904 / 5,904 GPS samples.
- fly4 OC2 segment coverage: 8,296 / 8,296 GPS samples.
- Core XY, velocity-vector, along-track, and cross-track errors contain no NaN.
- No segment shorter than 50 m receives a drift percentage.

## Reference metric difference

The archived headline values came from the older `eval_stage.py` start-yaw
convention. It mixes navigation bearing (`atan2(E, N)`) with a Cartesian
rotation and uses VIO velocity at one instant. The corrected tool uses a stable
course window and a consistent ENU rotation.

| Run | Corrected XY RMSE | Archived headline |
|---|---:|---:|
| fly3 OC2 | 377.659 m | 323 m |
| fly4 OC2 | 875.845 m | 827 m |

These are different metric definitions. The archived values remain historical
records, but they do not validate the corrected pipeline.
