#!/usr/bin/env python3
"""Select one common two-timescale flex parameter with four-flight LOFO."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


FLIGHTS = ("fly1", "fly2", "fly3", "fly4")


def load_summary(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def weighted(values: list[float], distances: list[float]) -> float:
    return float(np.average(values, weights=np.maximum(distances, 1e-9)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-root", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--taus", type=float, nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True)

    baseline: dict[str, dict] = {}
    for flight in FLIGHTS:
        baseline[flight] = load_summary(
            args.baseline_root
            / "runs"
            / flight
            / "baseline_stride12"
            / "metadata"
            / "global_summary.json"
        )

    rows: list[dict] = []
    by_tau: dict[float, dict[str, dict]] = {}
    for tau in args.taus:
        tau_key = int(tau) if float(tau).is_integer() else tau
        by_tau[tau] = {}
        for flight in FLIGHTS:
            candidate = load_summary(
                args.sweep_root
                / f"tau{tau_key}"
                / flight
                / "evaluation"
                / "metadata"
                / "global_summary.json"
            )
            by_tau[tau][flight] = candidate
            before = float(baseline[flight]["xy_rmse_m"])
            after = float(candidate["xy_rmse_m"])
            rows.append(
                {
                    "tau_baseline_s": tau,
                    "flight": flight,
                    "gps_distance_km": float(candidate["gps_distance_km"]),
                    "baseline_xy_rmse_m": before,
                    "candidate_xy_rmse_m": after,
                    "rmse_improvement_percent": 100.0 * (before - after) / before,
                    "baseline_final_xy_error_m": float(
                        baseline[flight]["final_xy_error_m"]
                    ),
                    "candidate_final_xy_error_m": float(candidate["final_xy_error_m"]),
                }
            )
    per_flight = pd.DataFrame(rows)
    per_flight.to_csv(args.output_dir / "per_flight_candidates.csv", index=False)

    aggregate_rows: list[dict] = []
    for tau in args.taus:
        selected = per_flight[per_flight.tau_baseline_s == tau]
        aggregate_before = weighted(
            selected.baseline_xy_rmse_m.tolist(),
            selected.gps_distance_km.tolist(),
        )
        aggregate_after = weighted(
            selected.candidate_xy_rmse_m.tolist(),
            selected.gps_distance_km.tolist(),
        )
        improvements = selected.rmse_improvement_percent.to_numpy(dtype=float)
        aggregate_rows.append(
            {
                "tau_baseline_s": tau,
                "aggregate_baseline_xy_rmse_m": aggregate_before,
                "aggregate_candidate_xy_rmse_m": aggregate_after,
                "aggregate_improvement_percent": 100.0
                * (aggregate_before - aggregate_after)
                / aggregate_before,
                "minimum_flight_improvement_percent": float(np.min(improvements)),
                "median_flight_improvement_percent": float(np.median(improvements)),
                "improved_flight_count": int(np.sum(improvements > 0.0)),
            }
        )
    aggregate = pd.DataFrame(aggregate_rows).sort_values("tau_baseline_s")
    aggregate.to_csv(args.output_dir / "aggregate_candidates.csv", index=False)

    lofo_rows: list[dict] = []
    for holdout in FLIGHTS:
        training = [flight for flight in FLIGHTS if flight != holdout]
        scores = []
        for tau in args.taus:
            selected = per_flight[
                (per_flight.tau_baseline_s == tau) & (per_flight.flight.isin(training))
            ]
            scores.append(
                (
                    float(selected.rmse_improvement_percent.min()),
                    float(selected.rmse_improvement_percent.median()),
                    -float(tau),
                    tau,
                )
            )
        selected_tau = max(scores)[3]
        holdout_row = per_flight[
            (per_flight.tau_baseline_s == selected_tau) & (per_flight.flight == holdout)
        ].iloc[0]
        lofo_rows.append(
            {
                "holdout_flight": holdout,
                "selected_tau_baseline_s": selected_tau,
                "training_minimum_improvement_percent": max(scores)[0],
                "holdout_improvement_percent": float(
                    holdout_row.rmse_improvement_percent
                ),
                "holdout_passed": bool(holdout_row.rmse_improvement_percent > 0.0),
            }
        )
    lofo = pd.DataFrame(lofo_rows)
    lofo.to_csv(args.output_dir / "lofo_results.csv", index=False)

    best = aggregate.sort_values(
        [
            "minimum_flight_improvement_percent",
            "median_flight_improvement_percent",
            "aggregate_improvement_percent",
        ],
        ascending=False,
    ).iloc[0]
    status = (
        "TWO_TIMESCALE_SHADOW_PASSED"
        if best.minimum_flight_improvement_percent > 0.0
        and bool(lofo.holdout_passed.all())
        else "TWO_TIMESCALE_SHADOW_FAILED"
    )
    summary = {
        "schema_version": 1,
        "status": status,
        "selection_rule": "maximize worst-flight XY RMSE improvement, then median, then aggregate",
        "selected_tau_baseline_s": float(best.tau_baseline_s),
        "selected_aggregate_improvement_percent": float(
            best.aggregate_improvement_percent
        ),
        "selected_minimum_flight_improvement_percent": float(
            best.minimum_flight_improvement_percent
        ),
        "selected_improved_flight_count": int(best.improved_flight_count),
        "lofo_passed_count": int(lofo.holdout_passed.sum()),
    }
    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    report = f"""# Two-timescale yaw-flex parameter selection

Status: **{status}**

Selection rule: maximize the minimum per-flight XY RMSE improvement, then the
median per-flight improvement, then the distance-weighted aggregate.

Selected common slow time constant: **{best.tau_baseline_s:g} s**.

{aggregate.to_markdown(index=False)}

## Leave-one-flight-out

{lofo.to_markdown(index=False)}
"""
    (args.output_dir / "report.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if status == "TWO_TIMESCALE_SHADOW_PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
