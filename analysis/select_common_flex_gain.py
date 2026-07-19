#!/usr/bin/env python3
"""Select one common flex correction gain with four-flight LOFO."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


FLIGHTS = ("fly1", "fly2", "fly3", "fly4")


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def tag(value: float) -> str:
    return str(value).replace("-", "m").replace(".", "p")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-root", type=Path, required=True)
    parser.add_argument("--baseline-root", type=Path, required=True)
    parser.add_argument("--gains", type=float, nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        raise RuntimeError(f"refusing to overwrite {args.output_dir}")
    args.output_dir.mkdir(parents=True)

    baseline = {
        flight: load(
            args.baseline_root
            / "runs"
            / flight
            / "baseline_stride12"
            / "metadata"
            / "global_summary.json"
        )
        for flight in FLIGHTS
    }
    rows = []
    for gain in args.gains:
        for flight in FLIGHTS:
            candidate = load(
                args.sweep_root
                / f"gain{tag(gain)}"
                / flight
                / "evaluation"
                / "metadata"
                / "global_summary.json"
            )
            before = float(baseline[flight]["xy_rmse_m"])
            after = float(candidate["xy_rmse_m"])
            rows.append(
                {
                    "flex_gain": gain,
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

    aggregate_rows = []
    for gain in args.gains:
        selected = per_flight[per_flight.flex_gain == gain]
        distances = np.maximum(selected.gps_distance_km.to_numpy(dtype=float), 1e-9)
        before = float(np.average(selected.baseline_xy_rmse_m, weights=distances))
        after = float(np.average(selected.candidate_xy_rmse_m, weights=distances))
        improvements = selected.rmse_improvement_percent.to_numpy(dtype=float)
        aggregate_rows.append(
            {
                "flex_gain": gain,
                "aggregate_baseline_xy_rmse_m": before,
                "aggregate_candidate_xy_rmse_m": after,
                "aggregate_improvement_percent": 100.0 * (before - after) / before,
                "minimum_flight_improvement_percent": float(np.min(improvements)),
                "median_flight_improvement_percent": float(np.median(improvements)),
                "improved_flight_count": int(np.sum(improvements > 0.0)),
            }
        )
    aggregate = pd.DataFrame(aggregate_rows).sort_values("flex_gain")
    aggregate.to_csv(args.output_dir / "aggregate_candidates.csv", index=False)

    lofo_rows = []
    for holdout in FLIGHTS:
        training = [flight for flight in FLIGHTS if flight != holdout]
        scores = []
        for gain in args.gains:
            selected = per_flight[
                (per_flight.flex_gain == gain) & (per_flight.flight.isin(training))
            ]
            scores.append(
                (
                    float(selected.rmse_improvement_percent.min()),
                    float(selected.rmse_improvement_percent.median()),
                    -float(gain),
                    gain,
                )
            )
        selected_gain = max(scores)[3]
        holdout_row = per_flight[
            (per_flight.flex_gain == selected_gain) & (per_flight.flight == holdout)
        ].iloc[0]
        lofo_rows.append(
            {
                "holdout_flight": holdout,
                "selected_flex_gain": selected_gain,
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
        "COMMON_GAIN_SHADOW_PASSED"
        if best.minimum_flight_improvement_percent > 0.0
        and bool(lofo.holdout_passed.all())
        else "COMMON_GAIN_SHADOW_FAILED"
    )
    summary = {
        "schema_version": 1,
        "status": status,
        "selection_rule": "maximize worst-flight XY RMSE improvement, then median, then aggregate",
        "selected_flex_gain": float(best.flex_gain),
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
    (args.output_dir / "report.md").write_text(
        "# Common flex gain selection\n\n"
        f"Status: **{status}**\n\n"
        f"Selected common gain: **{best.flex_gain:g}**.\n\n"
        + aggregate.to_markdown(index=False)
        + "\n\n## Leave-one-flight-out\n\n"
        + lofo.to_markdown(index=False)
        + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if status == "COMMON_GAIN_SHADOW_PASSED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
