# GPS-Z ON — Yaw/Gauge Comparison Brief

**Date:** 2026-06-05  
**Session:** tmux `gpsz_yawcmp` (8 windows)  
**Eval alignment:** `start_yaw` (canonical). `best_yaw_fit` is diagnostic only.

---

## Run completion table

| Run | Mode | Output dir | Done? | final_t | traj lines |
|---|---|---|---|---|---|
| fly1-A | global_yaw_oc_projection α=1 | gpsz_on_rerun_fly1_global_oc_guarded | YES | 1743.98 | 30223 |
| fly1-B | global_yaw_oc_fej_prechi2 | gpsz_on_yawcmp_fly1_oc_fej_prechi2 | YES | 1743.98 | 30223 |
| fly1-C | visual_4d_oc_fej_prechi2 | gpsz_on_yawcmp_fly1_oc_4d | YES | 1743.98 | 30223 |
| fly2-A | global_yaw_oc_projection α=1 | gpsz_on_rerun_fly2_global_oc_guarded | YES | 2499.98 | 61096 |
| fly2-B | global_yaw_oc_fej_prechi2 | gpsz_on_yawcmp_fly2_oc_fej_prechi2 | YES | 2499.98 | 62451 |
| fly2-C | visual_4d_oc_fej_prechi2 | gpsz_on_yawcmp_fly2_oc_4d | YES | 2499.98 | 62451 |
| fly3-A | global_yaw_oc_projection α=1 | gpsz_on_rerun_fly3_global_oc_guarded | YES | 1599.99 | 36556 |
| fly3-B | global_yaw_oc_fej_prechi2 | gpsz_on_yawcmp_fly3_oc_fej_prechi2 | YES | 1599.99 | 36556 |
| fly3-C | visual_4d_oc_fej_prechi2 | gpsz_on_yawcmp_fly3_oc_4d | YES | 1599.99 | 36556 |
| fly4-A | global_yaw_oc_projection α=1 | gpsz_on_rerun_fly4_global_oc_guarded | YES | 2815.99 | 57440 |
| fly4-B | global_yaw_oc_fej_prechi2 | gpsz_on_yawcmp_fly4_oc_fej_prechi2 | YES | 2815.99 | 57440 |
| fly4-C | visual_4d_oc_fej_prechi2 | gpsz_on_yawcmp_fly4_oc_4d | YES | 2815.99 | 57440 |

All NaN/Inf = 0. All neg_cov = 0.

---

## Metric table (start_yaw alignment)

| Flight | Mode | XY RMS | XY max | XY final | XY late RMS | Yaw RMS | Yaw max | Yaw final | best_yaw XY |
|---|---|---|---|---|---|---|---|---|---|
| fly1 | A global_oc | 282.81m | 620.77m | 620.77m | 259.79m | 5.27° | 99.04° | −0.36° | 180.11m |
| fly1 | B oc_prechi2 | 333.09m | 760.62m | 593.64m | 397.48m | 11.61° | 27.93° | −27.76° | 330.08m |
| fly1 | C oc_4d | 355.45m | 848.46m | 523.14m | 431.35m | 12.44° | 27.99° | −27.73° | 350.66m |
| fly2 | A global_oc | 501.82m | 1412.13m | 720.51m | 582.21m | 19.02° | 178.01° | −33.08° | 413.94m |
| fly2 | B oc_prechi2 | 778.21m | 2073.92m | 1034.01m | 923.46m | 26.89° | 52.25° | −46.90° | 485.67m |
| fly2 | C oc_4d | 579.31m | 1552.21m | 818.73m | 690.47m | 21.04° | 125.35° | −39.10° | 430.39m |
| fly3 | A global_oc | 273.52m | 496.97m | 414.60m | 334.93m | 6.52° | 21.78° | 7.05° | 231.85m |
| fly3 | B oc_prechi2 | **222.75m** | **324.00m** | **88.74m** | **207.18m** | 6.46° | 62.03° | 2.53° | **220.53m** |
| fly3 | C oc_4d | 256.08m | 582.16m | 266.93m | 314.49m | **5.96°** | **17.92°** | 6.93° | 235.82m |
| fly4 | A global_oc | 1304.89m | 3994.56m | 896.56m | 1452.29m | 26.23° | 54.94° | −52.61° | 959.76m |
| fly4 | B oc_prechi2 | **1152.41m** | **3659.06m** | **785.41m** | **1282.56m** | **25.06°** | 158.66° | −50.54° | **778.56m** |
| fly4 | C oc_4d | 1259.14m | 3991.49m | 608.77m | 1402.18m | 27.54° | 179.69° | −56.43° | 872.21m |

Bold = best in column per flight.

Late-window windows: fly1 last 444s, fly2 last 1200s, fly3 last 300s, fly4 last 1516s.

---

## Red flags (no conclusions)

**Fly1:** Global_oc (A) has lowest XY RMS but yaw max=99° spike. B and C have lower yaw max (~28°) but higher XY RMS.

**Fly2:** All three modes bad. Global_oc (A) is least-bad XY (502m). Yaw max is worst in A (178°). B has worst XY (778m). C is intermediate on both.

**Fly3:** B (oc_prechi2) best XY RMS (222m), lowest XY final (88m), but yaw max=62°. C has lowest yaw max (17.9°) and best yaw RMS (5.96°) but higher XY (256m). A is intermediate.

**Fly4:** All three modes very bad (1152–1305m XY RMS). B (oc_prechi2) slightly best on XY but yaw max=159°. C has yaw max=180°. A yaw max=55°. No mode controls fly4 well.

**Yaw max spikes:** Present in all flights/modes. Fly2-A (178°), fly4-C (180°), fly4-B (159°), fly2-C (125°), fly1-A (99°), fly3-B (62°) are the most severe.

**best_yaw_fit gap:** Large gap between start_yaw and best_yaw_fit for fly2/fly4 indicates yaw misalignment accumulates over the long flights regardless of mode.

---

## GPS-Z flags per Stage 2 mode

GPS-Z guard (REJECT_BY_DXY) fired in all runs near t=930–960 for fly1 (early init phase). All modes use identical GPS-Z settings. GPS-Z accepted counts expected to be similar to Stage 1 (3792–7913 per flight).

---

## Output locations

All Stage 2 runs:
- fly1: `Desktop/20260517_.../result/gpsz_on_yawcmp_fly1_{oc_fej_prechi2,oc_4d}/`
- fly2: `Desktop/20260518_.../result/gpsz_on_yawcmp_fly2_{oc_fej_prechi2,oc_4d}/`
- fly3: `Desktop/20260527_.../result/gpsz_on_yawcmp_fly3_{oc_fej_prechi2,oc_4d}/`
- fly4: `Desktop/20260528_.../result/gpsz_on_yawcmp_fly4_{oc_fej_prechi2,oc_4d}/`

Each has `traj.txt`, `traj.txt.bias`, `log.txt`, `diag.csv`, `yaw_diag.csv`, `eval_start_yaw/`, `eval_best_yaw/`.
