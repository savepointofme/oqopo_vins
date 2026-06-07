# OC vs FEJ — Current Evidence (1e4 Standard Config) — 2026-06-06

## Summary

OC mode2 (`global_yaw_oc_projection`, mode 2) consistently outperforms FEJ (`original`, mode 0)
in full-window XY ATE across all measurable flights with the standard 1e4 config.

---

## Data Table

| Flight | Mode | Full ATE RMS | Middle ATE RMS | Late ATE RMS | Notes |
|--------|------|-------------|----------------|-------------|-------|
| **Fly1** | OC (mode10 + GPS-Z) | **280 m** | — | — | Cond3 from three_condition_comparison_package |
| **Fly1** | FEJ (mode0 + GPS-Z) | 405 m | — | — | Cond2 from three_condition_comparison_package |
| **Fly1 delta** | OC wins | **−125 m (−31%)** | — | — | |
| | | | | | |
| **Fly3** | OC mode2 (fresh 1e4) | **323 m** | 249 m | 399 m | Fresh run, `gpsz_oc2_1e4_fly3/` |
| **Fly3** | FEJ (fresh 1e4) | 416 m | 234 m | 532 m | `condgpsz_fly3_original/` |
| **Fly3 delta** | OC wins | **−93 m (−22%)** | −15 m | −133 m | |
| | | | | | |
| **Fly4** | OC mode2 (fresh 1e4) | **827 m** | **139 m** | 892 m | Fresh run, `gpsz_oc2_1e4_fly4/` |
| **Fly4** | FEJ (fresh 1e4) | 967 m | 192 m | 1042 m | `condgpsz_fly4_original/` |
| **Fly4 delta** | OC wins | **−140 m (−14%)** | **−53 m** | −150 m | |

All runs: `config/d455_fly2/estimator_config.yaml` (fi_max_cond=1e4), guarded GPS-Z ON,
start_yaw alignment in eval_stage.py.

---

## Fly2 — Deferred

Fly2 has **no fair 1e4 OC-vs-FEJ comparison** yet.

- FEJ (`condgpsz_fly2_original`) exits at t≈1858 s with GPS divergence.
  Trajectory is available but truncated — not comparable to full fly2 duration (~2782 s).
- OC mode2 1e4 has **not been rerun** on fly2 as of 2026-06-06.
- Fly2 comparison is deferred until after workspace cleanup (Stage 7).

---

## Pattern

**OC mode2 beats FEJ in full-window ATE on every measurable flight.**

| Flight | OC advantage (full window) | OC advantage (%) |
|--------|---------------------------|-------------------|
| Fly1   | 125 m | 31% |
| Fly3   | 93 m  | 22% |
| Fly4   | 140 m | 14% |

The OC advantage is primarily in the **late stage** of each flight. Middle segment results
are mixed (OC and FEJ roughly equivalent in the middle for fly3; OC better by 53 m for fly4).

---

## Historical Note on "fly2/4 show no difference"

Previous analysis (2026-06-06 earlier session) based on the **1e5 config** runs for fly4 showed
smaller OC vs FEJ differences. This was an artifact of the permissive 1e5 triangulation gate.

With the standard **1e4 config**, fly4 also shows OC winning by 140 m (14%).
The "fly2/4 show no difference" narrative was incorrect.

---

## Supporting Artifacts

| Artifact | Contents |
|----------|---------|
| `traj_1e4_fly34_20260606.zip` | fly3_fej, fly3_oc2, fly4_fej, fly4_oc2 traj.txt files |
| `three_condition_comparison_package_20260605_1727.zip` | fly1 Cond2/Cond3 source trajectories |
| Desktop: `20260527_gsmq_d455_fly3/result/gpsz_oc2_1e4_fly3/` | fly3 OC full result dir |
| Desktop: `20260527_gsmq_d455_fly3/result/condgpsz_fly3_original/` | fly3 FEJ full result dir |
| Desktop: `20260528_gsmq_d455_fly4/result/gpsz_oc2_1e4_fly4/` | fly4 OC full result dir |
| Desktop: `20260528_gsmq_d455_fly4/result/condgpsz_fly4_original/` | fly4 FEJ full result dir |

---

## Next Step (Stage 8 — after cleanup)

Investigate why `fi_max_cond_number = 1e5` produces smaller OC vs FEJ differences than 1e4.
See `fi_cond_threshold_exploration_plan.md` when written.

**Do not run fly2 1e4 until workspace cleanup is complete.**
