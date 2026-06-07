# Fly3 OC Variant Validation — 2026-06-06

Controlled four-mode comparison on fly3 (`d455_20260526_174946`, t=[618,1837]s).
Purpose: determine which OC mode to designate as `official_oc`.

---

## Run Conditions (all four modes identical)

| Parameter | Value |
|-----------|-------|
| Config | `config/d455_fly2/estimator_config.yaml` |
| Dataset | `/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3/d455_20260526_174946` |
| GPS file | `gps_from_mems_offset438p0_cam_time.csv` |
| GPS time offset | `--gps-time-offset 0` (overrides YAML −1.78B s) |
| FC init | `fc_init_state_618_offset438p0.csv` (start=618, \|v\|=41.968 m/s) |
| Start time | `--start-time 618` |
| GPS-Z flags | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01` |
|  | `--gps-alt-max-res 80 --gps-alt-min-t-after-init 10` |
|  | `--gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| fi_max_cond_number | 10000 (1e4, from config YAML) |
| use_fej | true |

**Why all conditions must match:** GPS-Z fusion is the same for all modes, so any
difference in ATE/yaw is attributable to the yaw update policy only.

---

## Result Directories

| Mode | Directory |
|------|-----------|
| A — openvins_fej | `result/fly3_openvins_fej_gpsz_fi1e4_fcinit/` |
| B — oc_postchi2_current_gauge | `result/gpsz_oc2_1e4_fly3/` (reference, previously validated) |
| C — oc_prechi2_fej_gauge | `result/fly3_oc_prechi2_fej_gauge_gpsz_fi1e4_fcinit/` |
| D — oc_4d_prechi2_fej_gauge | `result/fly3_oc_4d_prechi2_fej_gauge_gpsz_fi1e4_fcinit/` |

---

## Run Commands

All three new runs (A, C, D) launched 2026-06-07 with task IDs:
- A: `b7flq29tq` (background)
- C: `b4fauk0oe` (background)
- D: `bk8cdxypx` (background)

Mode B (`gpsz_oc2_1e4_fly3`) was previously validated and not rerun.

---

## Evaluation Commands

```bash
BASEDIR=/mnt/c/Users/baloney/Desktop/20260527_gsmq_d455_fly3
GPS=$BASEDIR/result/fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv
IMU=$BASEDIR/d455_20260526_174946/imu0/data.csv

# Mode A self-eval
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --t0 618 --until 1837 \
  --dir-a $BASEDIR/result/fly3_openvins_fej_gpsz_fi1e4_fcinit \
  --dir-b $BASEDIR/result/fly3_openvins_fej_gpsz_fi1e4_fcinit \
  --gps $GPS --imu $IMU \
  --out $BASEDIR/result/eval_fly3_A \
  --yaw-align-mode start_yaw

# Mode B self-eval (reference)
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --t0 618 --until 1837 \
  --dir-a $BASEDIR/result/gpsz_oc2_1e4_fly3 \
  --dir-b $BASEDIR/result/gpsz_oc2_1e4_fly3 \
  --gps $GPS --imu $IMU \
  --out $BASEDIR/result/eval_fly3_B \
  --yaw-align-mode start_yaw

# Mode C self-eval
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --t0 618 --until 1837 \
  --dir-a $BASEDIR/result/fly3_oc_prechi2_fej_gauge_gpsz_fi1e4_fcinit \
  --dir-b $BASEDIR/result/fly3_oc_prechi2_fej_gauge_gpsz_fi1e4_fcinit \
  --gps $GPS --imu $IMU \
  --out $BASEDIR/result/eval_fly3_C \
  --yaw-align-mode start_yaw

# Mode D self-eval
python3 /mnt/d/vscode_dir/open_vins/eval_stage.py \
  --t0 618 --until 1837 \
  --dir-a $BASEDIR/result/fly3_oc_4d_prechi2_fej_gauge_gpsz_fi1e4_fcinit \
  --dir-b $BASEDIR/result/fly3_oc_4d_prechi2_fej_gauge_gpsz_fi1e4_fcinit \
  --gps $GPS --imu $IMU \
  --out $BASEDIR/result/eval_fly3_D \
  --yaw-align-mode start_yaw
```

---

## Metrics Table (start_yaw alignment, t=[618,1837])

"Late" = t=[1200,1837] (last 637s). Eval dirs: `result/eval_fly3_{A,B,C,D}/`.

| Mode | Full RMS (m) | Late RMS (m) | Yaw RMS (°) | Yaw max (°) | Final ATE (m) | Completed? |
|------|-------------|-------------|-------------|-------------|---------------|------------|
| A — openvins_fej | 448 | 597 | 8.7 | 86 | 992 | Yes (36557 poses) |
| **B — oc_postchi2_current_gauge** | **323** | **422** | **6.5** | 101 | 1668 | Yes (36556 poses) |
| C — oc_prechi2_fej_gauge | 638 | 909 | 8.7 | 83 | 3261 | Yes (36557 poses) |
| D — oc_4d_prechi2_fej_gauge | 646 | 920 | 9.4 | 78 | 3946 | Yes (36557 poses) |

Note: Mode D cleared the previous covariance crash point (t≈999s, ~11466 poses) in this run.
GPS-Z fusion stabilized the covariance that previously diverged without altitude anchoring.

All runs confirmed `CLI override: gps_time_offset=+0.000s` in logs.

---

## Previous Run History (diagnostic context)

Prior fly3 runs without FC init all diverged catastrophically:
- Mode A (no FC init): 57km ATE, crashed at t=1527s
- Mode C (no FC init): 604km ATE
- Mode D (no FC init): crashed at t=999s (negative covariance)

Prior fly3 runs with FC init but wrong GPS time offset (`--gps-time-offset` missing):
- All three applied gps_time_offset=−1779014842.675s → GPS-Z never fired
- Killed and relaunched 2026-06-07

Mode B worked because `gpsz_oc2_1e4_fly3` was originally launched with `--gps-time-offset 0`.

---

## Classification and Decision

| Mode | Classification | Reason |
|------|---------------|--------|
| A — openvins_fej | **BASELINE_REQUIRED** | Required FEJ reference; completes but no OC protection |
| B — oc_postchi2_current_gauge | **OFFICIAL_OC** | Best full RMS (323m) and late RMS (422m) |
| C — oc_prechi2_fej_gauge | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE | Full RMS=638m, late=909m — worse than A (no OC) |
| D — oc_4d_prechi2_fej_gauge | EXPERIMENTAL_ONLY / NOT_FOR_BASELINE | Full RMS=646m, late=920m — worst on XY position |

**official_oc = oc_postchi2_current_gauge (Mode B)**

### Why C and D underperform

C and D apply the H-space yaw projection **before** the chi2 outlier gate. The chi2 gate is
calibrated for the original H residual. After projection, the effective measurement covariance
is modified in a way that biases the chi2 statistic. This appears to cause incorrect
accept/reject decisions on visual measurements, degrading XY position accuracy even though
yaw_max is slightly lower (83°/78° vs 101° for B).

Mode B applies the same OC projection **post-chi2**, after the measurement has already passed
the gate using the unmodified residual. This is safer because the chi2 statistic remains
calibrated.

Modes C/D also use a FEJ-frozen gauge direction. After hundreds of seconds of flight, the
frozen gauge `n_fej` may diverge from the current unobservable direction, making the
projection incorrect.

### Mode D crash history

Mode D previously crashed at t≈999s in two runs (one without FC init, one with FC init but
no GPS-Z). This run (with FC init + GPS-Z) completed the full dataset. GPS-Z altitude
anchoring appears to prevent the covariance singularity that caused earlier crashes.
Despite completing, the position accuracy (646m RMS) is the worst of all four modes.

### Decision rule applied

1. All four modes completed — no DQ on crash
2. Rank by full RMS: B(323) < A(448) < C(638) < D(646)
3. Mode B is best OC mode → **official_oc = B**
