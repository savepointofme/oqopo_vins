# GPS-Z ON — Four Flights Fresh Rerun Results

**Generated:** 2026-06-05  
**Session:** tmux `gpsz_on_4flight_rerun` (5 windows: fly1, fly2, fly3, fly4, monitor)  
**Controlled setup:** All four runs are fresh. No results reused.

---

## Controlled Setup (all four flights)

| Parameter | Value |
|---|---|
| Yaw/gauge mode | `global_yaw_oc_projection` |
| `vio-global-yaw-oc-alpha` | `1.0` |
| GPS-Z | ON (`--gps-alt-update`) |
| `gps-alt-sigma` | `2.0` |
| `gps-alt-min-pzz` | `0.01` |
| `gps-alt-min-t-after-init` | `10` |
| `gps-alt-max-res` | `80` |
| `gps-alt-guard-dxy` | `0.5` |
| `gps-alt-guard-kxy` | `5.0` |
| `--viz-fast` | YES (visualization open) |
| `--dash-every` | `5` |

Confirmed in every run log:
```
CLI override: vio_yaw_update_mode=global_yaw_oc_projection
CLI override: vio_global_yaw_oc_alpha=1.000
GPS alt P_zz floor = 0.0100
GPS alt bootstrap delay = 10.0s after VIO init
GPS alt max-res gate = 80.0 m
GPS alt guard |dXY| max = 0.500 m
GPS alt guard |K_xy|/|K_pz| max = 5.000
```

---

## Fly1

| Field | Value |
|---|---|
| **Flight** | Fly1 |
| **Yaw/gauge mode** | `global_yaw_oc_projection` alpha=1.0 |
| **GPS-Z flags** | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| **Dataset** | `20260517_gsmq_d455_fly1/d455_20260517_174810` |
| **Config** | `config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml` |
| **Start time** | 930 |
| **Eval window** | 930–1744 |
| **FC init** | `config/d455_fly1/fc_init_state_930.csv` (att=3°, pos=0.05m, bg=0.003) |
| **GPS file** | `config/d455_fly1/fc_gps_cam_time.csv` |
| **Output directory** | `20260517_gsmq_d455_fly1/result/gpsz_on_rerun_fly1_global_oc_guarded/` |
| **Completed?** | YES — Done: Fri Jun 5 13:31:56 CST 2026 |
| **Final timestamp** | 1743.98 |

**Command:**
```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly1/estimator_config_sigma_px_2p0_fcinit_highalt.yaml \
  --dataset .../d455_20260517_174810 \
  --gps config/d455_fly1/fc_gps_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 930 \
  --init-from-fc config/d455_fly1/fc_init_state_930.csv \
  --init-att-sigma-deg 3 --init-pos-sigma 0.05 --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5 \
  --output .../gpsz_on_rerun_fly1_global_oc_guarded/traj.txt
```

**Output verification:**
- `traj.txt`: EXISTS (30224 lines)
- `traj.txt.bias`: EXISTS (30224 lines)
- Last line timestamp: 1895.xx (post-landing, past eval window — expected)
- Final eval timestamp: 1743.98 (full window covered)
- NaN/Inf: **0**
- neg_cov: **0**

**GPS-Z statistics (within run):**
| Metric | Value |
|---|---|
| Accepted updates | 3792 |
| Guard-rejected (REJECT_BY_DXY) | 877 |
| Max-res-rejected (>80m) | 0 |
| Residual mean (accepted) | −0.308 m |
| Residual RMS (accepted) | 0.840 m |
| Mean chi² (accepted) | 0.148 |

**Eval results — `start_yaw` (canonical):**
| Metric | Value |
|---|---|
| XY RMS (full window) | **282.81 m** |
| XY max | 620.77 m |
| XY final | 620.77 m |
| XY late-window RMS (last 444s) | 259.79 m |
| Yaw RMS (full window) | 5.27° |
| Yaw max | 99.04° |
| Yaw final | −0.36° |
| Yaw late-window RMS | 4.36° |

**Eval results — `best_yaw_fit` (diagnostic only, optimistic):**
| Metric | Value |
|---|---|
| XY RMS | 180.11 m |
| delta_yaw | +6.40° |

---

## Fly2

| Field | Value |
|---|---|
| **Flight** | Fly2 |
| **Yaw/gauge mode** | `global_yaw_oc_projection` alpha=1.0 |
| **GPS-Z flags** | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| **Dataset** | `20260518_gsmq_d455_fly2/d455_20260517_184722` |
| **Config** | `config/d455_fly2/estimator_config_cond_1e5.yaml` |
| **Start time** | 700 |
| **Eval window** | 700–2500 |
| **FC init** | `fc_rebuild_20260525/canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv` (att=5°, pos=100m, bg=0.003) |
| **GPS file** | `fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv` |
| **Output directory** | `20260518_gsmq_d455_fly2/result/gpsz_on_rerun_fly2_global_oc_guarded/` |
| **Completed?** | YES (eval window fully covered, runner processing post-eval tail) |
| **Final timestamp** | 2499.98 (eval window) |

**Command:**
```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260517_184722 \
  --gps .../fc_rebuild_20260525/gps_from_mems_offset447p5_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 700 \
  --init-from-fc .../canonical_offset447p5_start700/fc_init_state_700_offset447p5.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5 \
  --output .../gpsz_on_rerun_fly2_global_oc_guarded/traj.txt
```

**Output verification:**
- `traj.txt`: EXISTS (61096+ lines at eval cutoff)
- `traj.txt.bias`: EXISTS
- Final eval timestamp: 2499.98 (full window covered)
- NaN/Inf: **0**
- neg_cov: **0**

**GPS-Z statistics (within run):**
| Metric | Value |
|---|---|
| Accepted updates | 7088 |
| Guard-rejected (REJECT_BY_DXY) | 1296 |
| Max-res-rejected (>80m) | 411 |
| Residual mean (accepted) | +0.324 m |
| Residual RMS (accepted) | 1.704 m |
| Mean chi² (accepted) | 0.658 |

**Eval results — `start_yaw` (canonical):**
| Metric | Value |
|---|---|
| XY RMS (full window) | **501.82 m** |
| XY max | 1412.13 m |
| XY final | 720.51 m |
| XY late-window RMS (last 1200s) | 582.21 m |
| Yaw RMS (full window) | 19.02° |
| Yaw max | 178.01° |
| Yaw final | −33.08° |
| Yaw late-window RMS | 22.39° |

**Eval results — `best_yaw_fit` (diagnostic only, optimistic):**
| Metric | Value |
|---|---|
| XY RMS | 413.94 m |
| delta_yaw | −7.91° |

---

## Fly3

| Field | Value |
|---|---|
| **Flight** | Fly3 |
| **Yaw/gauge mode** | `global_yaw_oc_projection` alpha=1.0 |
| **GPS-Z flags** | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| **Dataset** | `20260527_gsmq_d455_fly3/d455_20260526_174946` |
| **Config** | `config/d455_fly2/estimator_config_cond_1e5.yaml` |
| **Start time** | 618 |
| **Eval window** | 618–1600 |
| **FC init** | `fc_rebuild_20260527/canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv` (att=5°, pos=100m, bg=0.003) |
| **GPS file** | `fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv` |
| **Output directory** | `20260527_gsmq_d455_fly3/result/gpsz_on_rerun_fly3_global_oc_guarded/` |
| **Completed?** | YES — Done: Fri Jun 5 13:37:35 CST 2026 |
| **Final timestamp** | 1599.99 |

**Command:**
```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260526_174946 \
  --gps .../fc_rebuild_20260527/gps_from_mems_offset438p0_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 618 \
  --init-from-fc .../canonical_offset438p0_start618/fc_init_state_618_offset438p0.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5 \
  --output .../gpsz_on_rerun_fly3_global_oc_guarded/traj.txt
```

**Output verification:**
- `traj.txt`: EXISTS (36557 lines)
- `traj.txt.bias`: EXISTS (36557 lines)
- Last line timestamp: ~1837 (post-landing, past eval window — expected)
- Final eval timestamp: 1599.99 (full window covered)
- NaN/Inf: **0**
- neg_cov: **0**

**GPS-Z statistics (within run):**
| Metric | Value |
|---|---|
| Accepted updates | 5427 |
| Guard-rejected (REJECT_BY_DXY) | 64 |
| Max-res-rejected (>80m) | 364 |
| Residual mean (accepted) | −0.207 m |
| Residual RMS (accepted) | 0.880 m |
| Mean chi² (accepted) | 0.172 |

**Eval results — `start_yaw` (canonical):**
| Metric | Value |
|---|---|
| XY RMS (full window) | **273.52 m** |
| XY max | 496.97 m |
| XY final | 414.60 m |
| XY late-window RMS (last 300s) | 334.93 m |
| Yaw RMS (full window) | 6.52° |
| Yaw max | 21.78° |
| Yaw final | 7.05° |
| Yaw late-window RMS | 7.13° |

**Eval results — `best_yaw_fit` (diagnostic only, optimistic):**
| Metric | Value |
|---|---|
| XY RMS | 231.85 m |
| delta_yaw | +3.21° |

---

## Fly4

| Field | Value |
|---|---|
| **Flight** | Fly4 |
| **Yaw/gauge mode** | `global_yaw_oc_projection` alpha=1.0 |
| **GPS-Z flags** | `--gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0` |
| **Dataset** | `20260528_gsmq_d455_fly4/d455_20260527_090549` |
| **Config** | `config/d455_fly2/estimator_config_cond_1e5.yaml` |
| **Start time** | 924.4 |
| **Eval window** | 924.4–2816 |
| **FC init** | `fc_rebuild_20260528/canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv` (att=5°, pos=100m, bg=0.003) |
| **GPS file** | `fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv` |
| **Output directory** | `20260528_gsmq_d455_fly4/result/gpsz_on_rerun_fly4_global_oc_guarded/` |
| **Completed?** | YES — Done: Fri Jun 5 13:53:16 CST 2026 |
| **Final timestamp** | 2815.99 |

**Command:**
```bash
./build_ov_msckf/run_serial_msckf_ros_free \
  --config config/d455_fly2/estimator_config_cond_1e5.yaml \
  --dataset .../d455_20260527_090549 \
  --gps .../fc_rebuild_20260528/gps_from_mems_offsetm202p2_cam_time.csv \
  --gps-time-offset 0 \
  --start-time 924.4 \
  --init-from-fc .../canonical_offsetm202p2_start904p4/fc_init_state_904p4_offsetm202p2.csv \
  --init-bg-sigma 0.003 \
  --vio-yaw-update-mode global_yaw_oc_projection \
  --vio-global-yaw-oc-alpha 1.0 \
  --gps-alt-update --gps-alt-sigma 2.0 --gps-alt-min-pzz 0.01 \
  --gps-alt-min-t-after-init 10 --gps-alt-max-res 80 \
  --gps-alt-guard-dxy 0.5 --gps-alt-guard-kxy 5.0 \
  --viz-fast --dash-every 5 \
  --output .../gpsz_on_rerun_fly4_global_oc_guarded/traj.txt
```

**Output verification:**
- `traj.txt`: EXISTS (57441 lines)
- `traj.txt.bias`: EXISTS (57441 lines)
- Final eval timestamp: 2815.99 (full window covered)
- NaN/Inf: **0**
- neg_cov: **0**

**GPS-Z statistics (within run):**
| Metric | Value |
|---|---|
| Accepted updates | 7913 |
| Guard-rejected (REJECT_BY_DXY) | 354 |
| Max-res-rejected (>80m) | 0 |
| Residual mean (accepted) | +0.040 m |
| Residual RMS (accepted) | 0.926 m |
| Mean chi² (accepted) | 0.188 |

**Eval results — `start_yaw` (canonical):**
| Metric | Value |
|---|---|
| XY RMS (full window) | **1304.89 m** |
| XY max | 3994.56 m |
| XY final | 896.56 m |
| XY late-window RMS (last 1516s) | 1452.29 m |
| Yaw RMS (full window) | 26.23° |
| Yaw max | 54.94° |
| Yaw final | −52.61° |
| Yaw late-window RMS | 28.70° |

**Eval results — `best_yaw_fit` (diagnostic only, optimistic):**
| Metric | Value |
|---|---|
| XY RMS | 959.76 m |
| delta_yaw | −18.01° |

---

## Summary Table (start_yaw alignment, all fresh reruns)

| Flight | Yaw mode | GPS-Z | Start | Eval window | Config | FC init | GPS file | Output dir | Done? | final_t | XY RMS | XY max | XY final | XY late RMS | Yaw RMS | Yaw max | Yaw final | Yaw late RMS | GPS-Z acc | GPS-Z guard-rej | GPS-Z maxres-rej | Res RMS | NaN/Inf | neg_cov |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Fly1 | global_yaw_oc_projection α=1.0 | ON | 930 | 930–1744 | d455_fly1/sigma_px_2p0_fcinit_highalt | fc_init_state_930.csv att=3° | fc_gps_cam_time.csv | gpsz_on_rerun_fly1_global_oc_guarded | YES | 1743.98 | **282.81m** | 620.77m | 620.77m | 259.79m | 5.27° | 99.04° | −0.36° | 4.36° | 3792 | 877 | 0 | 0.840m | 0 | 0 |
| Fly2 | global_yaw_oc_projection α=1.0 | ON | 700 | 700–2500 | d455_fly2/cond_1e5 | fc_init_state_700_offset447p5.csv att=5° | gps_offset447p5 | gpsz_on_rerun_fly2_global_oc_guarded | YES | 2499.98 | **501.82m** | 1412.13m | 720.51m | 582.21m | 19.02° | 178.01° | −33.08° | 22.39° | 7088 | 1296 | 411 | 1.704m | 0 | 0 |
| Fly3 | global_yaw_oc_projection α=1.0 | ON | 618 | 618–1600 | d455_fly2/cond_1e5 | fc_init_state_618_offset438p0.csv att=5° | gps_offset438p0 | gpsz_on_rerun_fly3_global_oc_guarded | YES | 1599.99 | **273.52m** | 496.97m | 414.60m | 334.93m | 6.52° | 21.78° | 7.05° | 7.13° | 5427 | 64 | 364 | 0.880m | 0 | 0 |
| Fly4 | global_yaw_oc_projection α=1.0 | ON | 924.4 | 924.4–2816 | d455_fly2/cond_1e5 | fc_init_state_904p4_offsetm202p2.csv att=5° | gps_offsetm202p2 | gpsz_on_rerun_fly4_global_oc_guarded | YES | 2815.99 | **1304.89m** | 3994.56m | 896.56m | 1452.29m | 26.23° | 54.94° | −52.61° | 28.70° | 7913 | 354 | 0 | 0.926m | 0 | 0 |

---

## Eval output locations

| Flight | start_yaw eval dir | best_yaw eval dir |
|---|---|---|
| Fly1 | `20260517_.../result/gpsz_on_rerun_fly1_global_oc_guarded/eval_start_yaw/` | `eval_best_yaw/` |
| Fly2 | `20260518_.../result/gpsz_on_rerun_fly2_global_oc_guarded/eval_start_yaw/` | `eval_best_yaw/` |
| Fly3 | `20260527_.../result/gpsz_on_rerun_fly3_global_oc_guarded/eval_start_yaw/` | `eval_best_yaw/` |
| Fly4 | `20260528_.../result/gpsz_on_rerun_fly4_global_oc_guarded/eval_start_yaw/` | `eval_best_yaw/` |

Each eval dir contains: `metrics_summary.csv`, `gps_xy_overlay.png`, `xy_ate_vs_time.png`, `yaw_err_vs_time.png`, `heading_850_930s.png`, `yaw_mode_comparison.csv`, `eval_stdout.txt`.

---

## Provenance

All four runs executed 2026-06-05 in parallel.  
tmux session `gpsz_on_4flight_rerun` on WSL Ubuntu 20.04, DISPLAY=:0.  
Runner binary: `build_ov_msckf/run_serial_msckf_ros_free`  
Run scripts: `run_gpsz_on_fly[1-4].sh`  
Eval script: `eval_gpsz_on_rerun.sh`  
No old artifacts reused. No runs skipped. No yaw mode changed. No GPS-Z guards removed.
