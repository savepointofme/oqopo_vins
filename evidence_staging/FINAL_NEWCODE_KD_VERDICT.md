# June12 K/D split — final new-code causal verdict

Generated 2026-07-16 from one coherent frozen runner/library pair.  This report
combines the complete focus 2x2 factorial with the preregistered full-flight
confirmation of C0, KONLY, and DONLY.  GPS was used only for offline evaluation
on its update-time grid.  Primary alignment is scale-preserving start-heading;
absolute-navigation-no-post-alignment is the corroborating view.  No best-fit or
post-hoc SE(3) result is used.

## Decision

- **June12 D is causally harmful in this fixed P4 pipeline.**  Its error increase
  is large, cross-flight, visible in XY, Z, FC-referenced velocity, and full-run
  course, and survives absolute-navigation evaluation.  It is the dominant
  cause of the harmful June12 K+D result.
- **June12 K has no transferable beneficial effect.**  In the focus window it
  worsens XY RMSE by about 47–48 m on both flights.  In the full window it remains
  harmful on fly1 but improves fly3.  That cross-flight reversal fails the
  acceptance contract; June12 K must not replace current K on this evidence.
- **K×D is antagonistic compensation, not validation of KD.**  The interaction
  `KD-K-D+C0` is negative for XY/Z/velocity/course RMSE on both flights: June12 K
  partially offsets the damage caused by June12 D.  Nevertheless KD remains far
  worse than C0 in XY, Z, and velocity.  A locally better course number—especially
  fly1—cannot rescue the geometry.

These are causal effects of changing the calibration fields on the complete
P4 initialization-plus-backend pipeline.  They do not by themselves identify a
physical lens-calibration error, because the same release window produces
different initialized q/p/v/bg/ba states.

## Completed matrix

- Focus, 930–1300 s fly1 and 618–947 s fly3: C0/KONLY/DONLY/KD × fly1/fly3,
  8/8 successful and validated.
- Full confirmation, 930–1938 s fly1 and 618–1837 s fly3:
  C0/KONLY/DONLY × fly1/fly3, 6/6 successful and validated.
- Incomplete estimator cells: none.
- Every full cell has process/frame/finalize exit `0`; config audit and frame
  contract pass.  `traj_nav` contains 29,912 rows for each fly1 cell and 36,243
  rows for each fly3 cell.
- A focus parent-controller status-summary race produced a controller-only
  nonzero return during one parallel launch; all six estimator cells themselves
  succeeded, and the final status was regenerated from their retained evidence.

## Frozen identity and actual calibration interventions

- Source commit: `fdd8d745229c7115b56b5d2d765f462a3fd68b4b`
- Runner SHA256: `f873565b03a85d1fe91695652c006b374d1406ede29f08af426b078b99281a76`
- `libov_msckf_lib.so` SHA256:
  `40ac217cfd15319fb15ba705d44d7781e9b1798170020f21416995e228306080`
- Runner and library hashes match at start and end in every run manifest.
- C0 K: `[386.75, 387.233, 330.249, 239.916]`
- C0 D: `[-0.043, 0.035, -0.001, 0.001]`
- June12 K: `[382.9994702701404, 382.2768370977604,
  332.5997999732952, 236.54464292967157]`
- June12 D: `[-0.06073119983113053, 0.0428789308670328,
  -0.003107075293581619, 0.0010996103095215512]`
- All cells retain C0/June12 `T_C_I`, `timeshift_cam_imu=0`, P4-only,
  stride 12, guarded height, and disabled P5/adaptive stride/dynamic ROI/AGL
  scale reset.  Actual config snapshots—not labels—were audited.
- No cmake/make/build action was run for this experiment.

## Focus factorial estimates

All values below are signed candidate-minus-C0 deltas on start-heading metrics;
positive RMSE delta is worse.

| Factor | Flight | XY RMSE m | Z RMSE m | FC vXY RMSE m/s | Course RMSE deg |
|---|---|---:|---:|---:|---:|
| KONLY | fly1 | +48.0047 | -5.3556 | +0.6379 | +0.6630 |
| KONLY | fly3 | +47.1059 | -0.0163 | +0.1127 | -0.4452 |
| DONLY | fly1 | +221.3311 | +234.0719 | +4.1101 | -0.9414 |
| DONLY | fly3 | +639.1787 | +172.7490 | +7.0281 | +6.3945 |
| KD total | fly1 | +171.9984 | +179.2394 | +3.5469 | -1.4344 |
| KD total | fly3 | +493.1782 | +114.0667 | +5.7246 | +4.7317 |

The focus interaction `KD-K-D+C0` is:

| Flight | XY RMSE m | Z RMSE m | FC vXY RMSE m/s | Course RMSE deg |
|---|---:|---:|---:|---:|
| fly1 | -97.3375 | -49.4770 | -1.2011 | -1.1560 |
| fly3 | -193.1063 | -58.6660 | -1.4162 | -1.2176 |

Thus June12 K reduces D's damage under June12 D (conditional K effect on XY is
-49.33 m fly1 and -146.00 m fly3), while K alone at current D is harmful in the
focus window.  This is the falsifiable signature of K–D compensation.

## Full-flight confirmation

Start-heading values and candidate-minus-C0 deltas:

| Factor | Flight | XY RMSE m | delta | Z RMSE m | delta | FC vXY RMSE m/s | delta | Course RMSE deg | delta |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| C0 | fly1 | 180.625 | — | 8.345 | — | 4.140 | — | 5.043 | — |
| KONLY | fly1 | 219.266 | +38.641 | 7.440 | -0.905 | 4.595 | +0.455 | 6.045 | +1.002 |
| DONLY | fly1 | 561.228 | +380.603 | 352.665 | +344.320 | 11.287 | +7.147 | 21.502 | +16.459 |
| C0 | fly3 | 226.887 | — | 26.782 | — | 4.125 | — | 4.894 | — |
| KONLY | fly3 | 165.364 | -61.523 | 22.777 | -4.005 | 2.221 | -1.904 | 1.579 | -3.315 |
| DONLY | fly3 | 1847.374 | +1620.488 | 187.209 | +160.427 | 22.017 | +17.892 | 32.617 | +27.723 |

DONLY crossed the formal 1 km divergence criterion at 1405.753 s on fly1 and
858.800 s on fly3.  Metrics above retain the whole automatic airborne window;
they were not cropped to hide the divergence.

Absolute-navigation-no-post-alignment corroborates the signs: KONLY XY RMSE is
224.225 vs 201.612 m on fly1 and 172.995 vs 221.528 m on fly3; DONLY is 548.992
and 1840.030 m.  Therefore the conclusion is not created by start-heading
alignment.

## First turn and post-turn straight

Signed course medians and slopes (deg, deg/s), full confirmation:

| Factor | Flight | Turn median C0→candidate | Turn slope C0→candidate | Late-straight median C0→candidate | Late-straight slope C0→candidate |
|---|---|---|---|---|---|
| KONLY | fly1 | 3.273→4.146 | +0.00808→+0.01285 | 3.771→4.749 | -0.03092→-0.04220 |
| KONLY | fly3 | -0.208→-1.423 | +0.06383→+0.03185 | 2.350→0.255 | +0.00942→+0.00783 |
| DONLY | fly1 | 3.273→-0.423 | +0.00808→-0.01523 | 3.771→-2.384 | -0.03092→-0.01828 |
| DONLY | fly3 | -0.208→2.478 | +0.06383→+0.09152 | 2.350→9.927 | +0.00942→+0.03566 |

K's direction changes by flight.  D's apparently favorable signed course values
in fly1's first-turn focus segment are contradicted by its enormous full-flight
XY/Z/velocity errors and 21.50 deg course RMSE; this is precisely why a local
heading-only screen is insufficient.

## P4 release and initialization separation

For each flight all four focus conditions have the same release-window
fingerprint, readiness level, release time, first-output time, and support counts
`[11,11,11,11,11]`:

- fly1: `NAVIGATION_READY`, release 940.353238821 s, first output
  940.512762547 s.
- fly3: `FULL_ALIGNMENT_READY`, release 628.199999809 s, first output
  628.338513374 s.

However the initialized state changes.  Relative to C0, total quaternion-angle
changes are K/D/KD = 1.198/1.387/1.474 deg on fly1 and
0.164/1.457/1.352 deg on fly3.  ZYX yaw diagnostics are
-0.316/+0.385/+0.058 deg on fly1 and +0.018/+0.111/+0.136 deg on fly3;
p/v/bg/ba and release NIS also change.  The same initialization deltas reproduce
in the full reruns.  Therefore the evidence separates an unchanged release
window/time from a changed initialization solution, but cannot assign the whole
trajectory delta solely to post-release backend geometry.

## Read-only camera-geometry audit

Audit result is PASS: June12 and flight imagery are 640x480; model is
`pinhole-radtan`; K order is `[fx,fy,cx,cy]`; D order is `[k1,k2,p1,p2]`;
OpenVINS parses `[fx,fy,cx,cy,k1,k2,p1,p2]`; camera downsampling is false and
the scale factor is 1.  This rules out the enumerated resolution/model/order/
scaling mistakes.  It does not prove June12 K/D are accurate for flight imagery.
Exposure/white-balance provenance is unavailable and is not used as a geometric
explanation.

## Evidence map and remaining limits

- Focus root: `attempts/newcode_f873_40ac`
- Full-confirmation root: `attempts/newcode_f873_40ac_full`
- Each `runs/<cell>/run_manifest.json` records source status, frozen identities,
  command, complete config snapshot, allowlisted environment, inputs, process
  artifacts, and start/end hashes.
- `summary/global_metrics_start_heading.csv` and
  `summary/global_metrics_absolute.csv` contain the formal global metrics.
- `summary/phase_metrics_*.csv` contain the pre-turn/turn/post-straight CSVs and
  slopes; canonical curves/CSVs/PNGs are under `analysis/full_flight_*`.
- Focus `summary/factorial_effects_and_interaction.csv` is the complete 2x2
  interaction table; full `summary/factor_pair_deltas.csv` contains confirmation
  deltas.
- `summary/p4_release_fingerprints.csv` and
  `summary/initialization_state_deltas_vs_C0.csv` preserve the release/init split.
- `provenance/CALIBRATION_GEOMETRY_AUDIT.md` preserves the parser-format audit.

Still unproved: which physical calibration is closest to ground truth; whether
the K effect reversal is caused by scene/trajectory dependence or numerical
sensitivity; how much of each factor effect is mediated by P4 initialization
versus later visual updates; and whether a newly collected, flight-matched
camera calibration would generalize beyond fly1/fly3.  No claim about T_C_I or
time offset follows from this K/D split alone.
