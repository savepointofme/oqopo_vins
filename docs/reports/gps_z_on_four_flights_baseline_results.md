# GPS-Z ON Baseline Results — Four Flights

**Date**: 2026-06-05  
**Branch**: review/stage-schmidt-fly3-yaw-align  
**Purpose**: Collect clean GPS-Z ON baseline results for fly1–fly4. No cross-mode comparison. No tuning.

---

## 1. Commands Used

### Fly1 — existing result, not re-run

Script: `run_fly1_baseline_oc_30hz.sh`

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly1/estimator_config_fly2params_cond_1e5.yaml \
  --dataset /mnt/c/Users/baloney/Desktop/20260517_gsmq_d455_fly1/d455_20260517_174810 \
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 930 \
  --init-from-fc config/d455_fly1/fc_init_state_930.csv \
  --init-att-sigma-deg 2.0 \
  --init-pos-sigma 0.05 \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5 \
  --output result/baseline_oc_30hz/traj.txt
```

Eval:
```bash
python3 eval_stage.py --t0 930 --until 1740 \
  --dir-a result/baseline_oc_30hz --dir-b result/baseline_oc_30hz \
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --imu .../d455_20260517_174810/imu0/data.csv \
  --out result/gpsz_baseline_eval_fly1 \
  --yaw-align-mode all
```

---

### Fly2 — existing result, not re-run

Existing run: `fc_rebuild_20260525/clean_timestamp_all_yaw0_gpsalt_offset447p5`

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260517_184722 \
  --gps .../gps_from_mems_offset447p5_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 700 \
  --init-from-fc .../canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode per_block_scale \
  --vio-yaw-update-scale 0.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --output .../clean_timestamp_all_yaw0_gpsalt_offset447p5/traj.txt
```

**Note**: Yaw mode is `per_block_scale scale=0.0` — yaw correction is frozen (no OC projection applied). GPS-Z guard flags were NOT used for this run. This was the only complete GPS-Z ON run available for fly2 that did not diverge and used the correct GPS alignment offset. A fresh run with `global_yaw_oc_projection alpha=1.0` would take ~2 hours and was not completed.

Eval:
```bash
python3 eval_stage.py --t0 700 --until 2500 \
  --dir-a .../clean_timestamp_all_yaw0_gpsalt_offset447p5 \
  --dir-b .../clean_timestamp_all_yaw0_gpsalt_offset447p5 \
  --gps .../gps_from_mems_offset447p5_cam_time.csv \
  --imu .../d455_20260517_184722/imu0/data.csv \
  --out .../gpsz_baseline_eval_fly2 \
  --yaw-align-mode all
```

---

### Fly3 — existing result, not re-run

Existing run: `phase6_oc_experiments_offset438p0_start618_until1600/global_oc_current_late_baseline_offset438p0_until1600`

From `run_phase6_oc_experiments_offset438p0_until1600.sh`:

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260526_174946 \
  --gps .../gps_from_mems_offset438p0_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 618 \
  --init-from-fc .../canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --output .../global_oc_current_late_baseline_.../traj.txt
```

**Note**: att-sigma not overridden; from FC init CSV default: att=5.0°, bg=0.003 rad/s, ba=1.0 m/s².

Eval:
```bash
python3 eval_stage.py --t0 618 --until 1600 \
  --dir-a .../global_oc_current_late_baseline_offset438p0_until1600 \
  --dir-b .../global_oc_current_late_baseline_offset438p0_until1600 \
  --gps .../gps_from_mems_offset438p0_cam_time.csv \
  --imu .../d455_20260526_174946/imu0/data.csv \
  --out .../gpsz_baseline_eval_fly3 \
  --yaw-align-mode all
```

---

### Fly4 — existing result, not re-run

Existing run: `fc_rebuild_20260528/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1`

From `run_20260528_gsmq_d455_fly4_yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz.sh`:

```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260527_090549 \
  --gps .../gps_from_mems_offsetm202p2_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 904.4 \
  --init-from-fc .../canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update \
  --gps-alt-sigma 2.0 \
  --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 \
  --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 \
  --gps-alt-guard-kxy 5.0 \
  --output .../global_oc_alpha1/traj.txt
```

**Note**: att-sigma not overridden; from FC init CSV default: att=5.0°.

Eval:
```bash
python3 eval_stage.py --t0 924.4 --until 2816 \
  --dir-a .../yaw_method_ablation_.../global_oc_alpha1 \
  --dir-b .../yaw_method_ablation_.../global_oc_alpha1 \
  --gps .../gps_from_mems_offsetm202p2_cam_time.csv \
  --imu .../d455_20260527_090549/imu0/data.csv \
  --out .../gpsz_baseline_eval_fly4 \
  --yaw-align-mode all
```

---

## 2. Per-Flight Setup Table

| Parameter | Fly1 | Fly2 | Fly3 | Fly4 |
|---|---|---|---|---|
| **Dataset** | d455_20260517_174810 | d455_20260517_184722 | d455_20260526_174946 | d455_20260527_090549 |
| **Config** | d455_fly1/estimator_config_fly2params_cond_1e5.yaml | d455_fly2/estimator_config_cond_1e5.yaml | d455_fly2/estimator_config_cond_1e5.yaml | d455_fly2/estimator_config_cond_1e5.yaml |
| **Start time** | 930.0 s | 700.0 s | 618.0 s | 904.4 s |
| **Eval T0** | 930 s | 700 s | 618 s | 924.4 s |
| **Eval T1** | 1740 s | 2500 s | 1600 s | 2816 s |
| **Eval duration** | 810 s | 1800 s | 982 s | 1892 s |
| **FC init file** | fc_init_state_930.csv | fc_init_state_700_offset447p5.csv | fc_init_state_618_offset438p0.csv | fc_init_state_904p4_offsetm202p2.csv |
| **GPS file** | fc_gps_cam_time.csv (repo) | gps_from_mems_offset447p5_cam_time.csv | gps_from_mems_offset438p0_cam_time.csv | gps_from_mems_offsetm202p2_cam_time.csv |
| **GPS time offset** | 0 | 0 | 0 | 0 |
| **Yaw/gauge mode** | `global_yaw_oc_projection alpha=1.0` | `per_block_scale scale=0.0` (**frozen — no OC**) | `global_yaw_oc_projection alpha=1.0` | `global_yaw_oc_projection alpha=1.0` |
| **att-sigma** | 2.0° (explicit) | 5.0° (FC init default) | 5.0° (FC init default) | 5.0° (FC init default) |
| **GPS-Z sigma** | 2.0 m | 2.0 m | 2.0 m | 2.0 m |
| **GPS-Z min-pzz** | 0.01 | 0.01 | 0.01 | 0.01 |
| **GPS-Z min-t** | 10 s | 10 s | 10 s | 10 s |
| **GPS-Z max-res** | 80 m | 80 m | 80 m | 80 m |
| **GPS-Z guard-dxy** | 0.5 m | none | 0.5 m | 0.5 m |
| **GPS-Z guard-kxy** | 5.0 | none | 5.0 | 5.0 |
| **Output dir** | result/baseline_oc_30hz/ | fc_rebuild_20260525/clean_timestamp_all_yaw0_gpsalt_offset447p5/ | phase6_.../global_oc_current_late_baseline_offset438p0_until1600/ | fc_rebuild_20260528/yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz/global_oc_alpha1/ |
| **Run source** | run_fly1_baseline_oc_30hz.sh | run_all_yaw0_gpsalt_gplane_ablation_headless.sh | run_phase6_oc_experiments_offset438p0_until1600.sh | run_20260528_..._offsetm202p2_start904p4_gpsz_viz.sh |
| **Completed?** | Yes | Yes | Yes | Yes |
| **Final timestamp** | 1938.1 s | 2639.6 s | 1604.5 s | 2840.2 s |
| **NaN/Inf in traj** | None | None | None | None |
| **neg_cov observed** | Not observed | Not observed | Not observed | Not observed |

---

## 3. Per-Flight Metric Table

All XY metrics use `start_yaw` alignment (canonical). best_yaw_fit is diagnostic only.

| Metric | Fly1 | Fly2 | Fly3 | Fly4 |
|---|---|---|---|---|
| **XY RMS (start_yaw)** | **245.9 m** | **356.7 m** | **273.5 m** | **743.6 m** |
| **XY max** | 462.4 m | 599.6 m | 497.0 m | 1921.5 m |
| **XY final** | 96.6 m | 273.9 m | 414.6 m | 337.1 m |
| **XY late-window RMS** | 241.7 m | 354.5 m | 334.9 m | 816.7 m |
| **XY p95** | 412.4 m | — | — | 1566.2 m |
| **XY RMS (best_yaw_fit)** | 233.9 m | 324.1 m | 231.9 m | 662.6 m |
| **Yaw delta at T0** | +0.38° | −0.68° | +0.03° | +1.64° |
| **Yaw RMS** | 8.45° | 10.87° | 6.52° | 9.65° |
| **Yaw max** | 21.6° | 25.4° | 21.8° | 26.3° |
| **Yaw final** | −20.5° | −15.1° | +7.1° | −19.1° |
| **Yaw late-window RMS** | 10.0° | 11.5° | 7.1° | 10.3° |
| **n_eval (GPS samples)** | 24,283 | 53,970 | 29,440 | 56,721 |
| **GPS-Z calls (at last stat)** | 4,557 | 8,420 | 4,659 | 8,194 |
| **GPS-Z accepted** | 4,518 (99.2%) | 8,419 (100.0%) | 4,658 (100.0%) | 8,105 (98.9%) |
| **GPS-Z rejected** | 38 (kxy guard) | 0 | 0 | 88 (dxy=63, kxy=25) |
| **GPS-Z mean \|res\|** | 0.51 m | 1.29 m | 0.54 m | 0.80 m |
| **GPS-Z mean K_pz** | 0.047 | 0.053 | 0.048 | 0.057 |
| **chi2 accepted** | 534,073 | 1,233,114 | 692,960 | 1,300,268 |
| **chi2 rejected** | 3,576 | 3,886 | 105 | 2,558 |
| **chi2 rejection rate** | 0.67% | 0.31% | 0.015% | 0.20% |

---

## 4. Notes on Failures and Incomplete Runs

**Fly2 — yaw mode differs from all other flights.**  
All other flights use `global_yaw_oc_projection alpha=1.0`. Fly2 uses `per_block_scale scale=0.0` (yaw correction frozen at zero). A fresh fly2 run with `global_yaw_oc_projection alpha=1.0` was attempted but terminated after ~5 minutes due to very slow processing rate (~14 s of data per minute of real time, estimated 2+ hours to complete). The existing stable run with frozen-yaw was used instead. This makes fly2 XY metrics not directly comparable to fly1/fly3/fly4 on the yaw-mode axis.

**Fly2 — no GPS-Z guard flags.**  
The existing fly2 run did not include `--gps-alt-guard-dxy` or `--gps-alt-guard-kxy`. This may explain the 100% acceptance rate but higher mean residual (1.29 m vs 0.51–0.80 m for guarded flights). Updates that would be rejected by the guard in other flights are accepted here.

**Fly4 — significantly worse XY RMS (743 m vs 246–357 m).**  
Fly4 has 88 GPS-Z rejections (63 by dxy guard, 25 by kxy guard), the largest yaw max (26.3°), and the largest late-window XY RMS (817 m), indicating error accumulation in the second half. The source of fly4 degradation is not diagnosed here.

**Fly1 — uses tighter att-sigma (2.0°) vs all other flights (5.0°).**  
This initialization difference is not controlled. It may contribute to fly1's relatively better XY RMS.

**All flights: Z metrics not computed as ATE.**  
There is no altitude groundtruth for these flights. The GPS-Z residual stats (`|res|_mu`) from the `GPS-ALT-STAT` log line are the closest proxy for Z performance. Z values in the traj files are VIO-frame coordinates (not altitude ATE).

**All flights: neg_cov not explicitly checked.**  
No negative covariance warnings were observed in the logs. Formal neg_cov detection would require parsing the diag.csv covariance columns.

---

## 5. Configs and Scripts Used

| Flight | Estimator config | Run script |
|---|---|---|
| Fly1 | `config/d455_fly1/estimator_config_fly2params_cond_1e5.yaml` | `run_fly1_baseline_oc_30hz.sh` |
| Fly2 | `config/d455_fly2/estimator_config_cond_1e5.yaml` | `run_all_yaw0_gpsalt_gplane_ablation_headless.sh` |
| Fly3 | `config/d455_fly2/estimator_config_cond_1e5.yaml` | `run_phase6_oc_experiments_offset438p0_until1600.sh` |
| Fly4 | `config/d455_fly2/estimator_config_cond_1e5.yaml` | `run_20260528_gsmq_d455_fly4_yaw_method_ablation_offsetm202p2_start904p4_gpsz_viz.sh` |

Eval script (all flights): `eval_stage.py` (repo root)

Eval output dirs:
- Fly1: `result/gpsz_baseline_eval_fly1/`
- Fly2: `result/gpsz_baseline_eval_fly2/`
- Fly3: `result/gpsz_baseline_eval_fly3/`
- Fly4: `result/gpsz_baseline_eval_fly4/`

---

## 6. No Tuning Recommendation

This document is a baseline collection only. No comparison between modes is drawn. No conclusion about which mode is best is made. The fly2 yaw mode difference is documented as a confound, not as a finding. Cross-flight yaw gauge comparison is a separate task.
