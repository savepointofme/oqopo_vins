# Latest OpenVINS Flight Baseline

This folder is the clean entry point for the current flight baseline.  It
does not replace the research history; it gives the repo one small place to
run the current recipe, switch camera-frame stride manually, and analyze the
result with the canonical full-flight tool.

## Baseline Contract

- Estimator config: `config/estimator_config.yaml`
- Camera/IMU calibration: `config/kalibr_imucam_chain.yaml`
- IMU noise: `config/kalibr_imu_chain.yaml`
- Calibration lineage: `references/d455_converged/`
- Default yaw mode: `--yaw-mode baseline`
- Default GPS-Z: `--height-mode guarded` with the guarded parameters used in the June 2026
  reruns
- Default run stride: `12`
- No-thinning control stride: `1`
- Common sweep strides: `1 2 4 8 12 16 20 30`

The estimator YAML is deliberately stride-free.  Stride is a runtime choice:

```bash
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 12
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 1
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 12 --yaw-mode oc-fej
bash baseline/latest/scripts/run_baseline_fly.sh --fly fly3 --stride 12 --height-mode nasa-lean
```

The run script writes a timestamped folder and snapshots the exact command and
baseline config into the output directory.

## Mode Names

- `--yaw-mode baseline`: FEJ Jacobians plus current-gauge yaw OC.  This keeps
  the current best recipe unchanged.
- `--yaw-mode fej`: OpenVINS FEJ only, no yaw OC projection.
- `--yaw-mode oc`: current-gauge yaw OC with FEJ disabled.  This is a clean
  ablation, not the recommended baseline.
- `--yaw-mode oc-fej`: FEJ Jacobians plus FEJ-gauge yaw OC projection.
- `--height-mode guarded`: guarded GPS-Z path used by the baseline.
- `--height-mode nasa-lean`: NASA underweighting experiment on the same
  GPS-Z measurement model.

## Four Flight Examples

```bash
bash baseline/latest/scripts/run_fly1_stride12.sh
bash baseline/latest/scripts/run_fly2_stride12.sh
bash baseline/latest/scripts/run_fly3_stride12.sh
bash baseline/latest/scripts/run_fly4_stride12.sh
```

Or run all four:

```bash
bash baseline/latest/scripts/run_all_four_stride12.sh
```

Use `--headless` only when there is no display available.  Visual runs should
normally keep the dashboard enabled with `--viz-fast --dash-every 5`.

## Analyze A Run

```bash
bash baseline/latest/scripts/analyze_latest_run.sh \
  --fly fly3 \
  --run-dir /mnt/c/Users/baloney/Desktop/openvins_latest_baseline_runs_20260625/fly3_stride12_YYYYMMDD_HHMMSS
```

The analysis wrapper calls `analysis/full_flight_error_analysis.py`, which is
the canonical GPS-grid evaluator for flight results.

## What Is Excluded

The low-pass/notch IMU work, adaptive stride variants, and older calibration
forks are not part of this baseline yet.  They are kept in the legacy
quarantine until the current baseline is verified on fly1 through fly4.
