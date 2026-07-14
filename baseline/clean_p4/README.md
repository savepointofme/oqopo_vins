# P4 Clean Baseline

This profile is independent of the frozen readonly audit v6. It establishes a
clean FC full-state initialization path with no historical residual FC-board
correction and no post alignment.

Locked contract:

- June-12 ground Kalibr K/D/T_C_I/Camera--IMU time offset;
- online intrinsics, distortion, extrinsic and time calibration off;
- declared FC-FRD to board-IMU axis map only;
- no approximately 7-degree, 4.089-degree or other fixed correction;
- fixed camera stride 12;
- GPS XY/course reference-only; guarded GPS-Z remains the registered height
  update;
- primary evaluation `absolute_navigation_no_post_alignment`;
- secondary evaluation `relative_drift`.

Short-window I0/I1/I2 batch:

```bash
bash baseline/clean_p4/scripts/run_p4_init_batch.sh --fly fly1 --scope short
bash baseline/clean_p4/scripts/run_p4_init_batch.sh --fly fly3 --scope short
```

After Ibest selection, full-flight I0/I2 batch:

```bash
bash baseline/clean_p4/scripts/run_p4_init_batch.sh --fly fly1 --scope full
bash baseline/clean_p4/scripts/run_p4_init_batch.sh --fly fly3 --scope full
```

Calibration contamination isolation runs C0/C1/C2 concurrently per flight:

```bash
bash baseline/clean_p4/scripts/run_calibration_isolation_batch.sh --fly fly1
bash baseline/clean_p4/scripts/run_calibration_isolation_batch.sh --fly fly3
```

The batch wrappers use blocking `wait` and do not poll estimator processes.
Batch mode is explicitly headless. Each run persists the canonical init state,
raw/local trajectory, G_nav trajectory, metadata, hashes/logs and exit code.
