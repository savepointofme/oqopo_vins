#!/usr/bin/env python3
"""Summarize a ros-free flex relative-yaw factor diagnostic consistently."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean


NUMERIC_FIELDS = (
    "prediction_deg",
    "residual_deg",
    "nis",
    "flex_anchor_before_deg",
    "flex_current_before_deg",
    "flex_current_after_deg",
    "posterior_prediction_deg",
    "state_yaw_delta_deg",
    "position_delta_norm_m",
    "velocity_delta_norm_mps",
    "gyro_bias_delta_norm_radps",
    "accel_bias_delta_norm_mps2",
)


def finite(value: str) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        lines = (line for line in stream if not line.startswith("#"))
        return list(csv.DictReader(lines))


def summarize(rows: list[dict[str, str]]) -> dict[str, object]:
    accepted = [row for row in rows if row.get("accepted") == "1"]
    result: dict[str, object] = {
        "row_count": len(rows),
        "accepted_count": len(accepted),
        "decisions": {},
    }
    decisions: dict[str, int] = {}
    for row in rows:
        decision = row.get("decision", "")
        decisions[decision] = decisions.get(decision, 0) + 1
    result["decisions"] = decisions
    for field in NUMERIC_FIELDS:
        values = [value for row in accepted if (value := finite(row.get(field, ""))) is not None]
        if values:
            result[field] = {
                "mean": fmean(values),
                "max_abs": max(abs(value) for value in values),
                "sum": sum(values),
                "first": values[0],
                "last": values[-1],
            }
    if accepted:
        result["time_start_s"] = finite(accepted[0]["current_time_s"])
        result["time_end_s"] = finite(accepted[-1]["current_time_s"])
        first_flex = finite(accepted[0]["flex_anchor_before_deg"])
        last_flex = finite(accepted[-1]["flex_current_after_deg"])
        if first_flex is not None and last_flex is not None:
            result["flex_net_change_deg"] = last_flex - first_flex
    return result


def parse_window(text: str) -> tuple[float, float]:
    start_text, end_text = text.split(":", maxsplit=1)
    start, end = float(start_text), float(end_text)
    if not end > start:
        raise argparse.ArgumentTypeError("window must satisfy END > START")
    return start, end


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--factor-csv", type=Path, required=True)
    parser.add_argument("--window", type=parse_window, action="append", default=[])
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    rows = load_rows(args.factor_csv)
    report: dict[str, object] = {"full": summarize(rows), "windows": {}}
    for start, end in args.window:
        selected = [
            row for row in rows
            if (time := finite(row.get("current_time_s", ""))) is not None
            and start <= time <= end
        ]
        report["windows"][f"{start:g}:{end:g}"] = summarize(selected)

    payload = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
