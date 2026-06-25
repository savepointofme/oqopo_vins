"""Backend validation for refined flight segments."""
from __future__ import annotations

import os

import numpy as np
import pandas as pd


def _wrap_deg(a):
    return (np.asarray(a, dtype=float) + 180.0) % 360.0 - 180.0


def _load(analysis_dir: str):
    df = pd.read_csv(os.path.join(analysis_dir, "data", "gps_time_aligned_samples.csv"))
    seg_index = pd.read_csv(os.path.join(analysis_dir, "tables", "segment_index.csv"))
    seg_err = pd.read_csv(os.path.join(analysis_dir, "tables", "segment_error_summary.csv"))
    return df, seg_index, seg_err


def validate_analysis_dir(
    analysis_dir: str,
    *,
    heading_tol_deg: float = 4.0,
    time_tol_s: float = 0.25,
    error_tol_m: float = 0.05,
) -> tuple[list[str], list[str], list[str]]:
    """Return (summary_lines, warnings, errors) for a completed analysis dir."""
    df, seg_index, seg_err = _load(analysis_dir)
    err_by_id = seg_err.set_index("segment_id")
    warnings: list[str] = []
    errors: list[str] = []
    lines: list[str] = []

    required_index = {
        "segment_id", "segment_type", "t_start_raw", "t_end_raw", "t_start",
        "t_end", "dist_m", "segment_heading_deg", "heading_source",
        "heading_quality",
    }
    missing = sorted(required_index - set(seg_index.columns))
    if missing:
        errors.append(f"segment_index missing columns: {', '.join(missing)}")
        return lines, warnings, errors

    for _, seg in seg_index.iterrows():
        sid = int(seg["segment_id"])
        samples = df[df["segment_id"] == sid]
        if len(samples) == 0:
            errors.append(f"segment {sid}: no samples with this segment_id")
            continue
        t_min = float(samples["t"].min())
        t_max = float(samples["t"].max())
        if abs(t_min - float(seg["t_start"])) > time_tol_s:
            errors.append(
                f"segment {sid}: samples min(t) {t_min:.3f} != "
                f"segment t_start {float(seg['t_start']):.3f}"
            )
        if abs(t_max - float(seg["t_end"])) > time_tol_s:
            errors.append(
                f"segment {sid}: samples max(t) {t_max:.3f} != "
                f"segment t_end {float(seg['t_end']):.3f}"
            )

        heading = float(seg["segment_heading_deg"])
        valid = samples[samples["valid"].astype(bool)] if "valid" in samples else samples
        if len(valid) == 0:
            valid = samples
        course_diff = np.abs(_wrap_deg(valid["gps_course_deg"].to_numpy() - heading))
        p50 = float(np.percentile(course_diff, 50)) if len(course_diff) else float("nan")
        p90 = float(np.percentile(course_diff, 90)) if len(course_diff) else float("nan")
        cmax = float(np.max(course_diff)) if len(course_diff) else float("nan")
        cstart = float(course_diff[0]) if len(course_diff) else float("nan")
        cend = float(course_diff[-1]) if len(course_diff) else float("nan")
        if seg["segment_type"] == "straight":
            warn_lim = heading_tol_deg + 1.0
            if cstart > warn_lim or cend > warn_lim or p90 > warn_lim:
                warnings.append(
                    f"segment {sid}: straight course residual high "
                    f"(start={cstart:.2f}, end={cend:.2f}, p90={p90:.2f} deg); "
                    "may still include turn transition."
                )

        if sid in err_by_id.index and len(valid):
            row = err_by_id.loc[sid]
            last = valid.iloc[-1]
            theta = np.deg2rad(heading)
            along = float(last["err_E"] * np.cos(theta) + last["err_N"] * np.sin(theta))
            cross = float(-last["err_E"] * np.sin(theta) + last["err_N"] * np.cos(theta))
            stored_along = float(row["local_final_along_error_m"])
            stored_cross = float(row["local_final_cross_error_m"])
            if abs(along - stored_along) > error_tol_m:
                errors.append(
                    f"segment {sid}: final along recompute {along:.3f} != "
                    f"stored {stored_along:.3f}"
                )
            if abs(cross - stored_cross) > error_tol_m:
                errors.append(
                    f"segment {sid}: final cross recompute {cross:.3f} != "
                    f"stored {stored_cross:.3f}"
                )

        lines.append(
            "segment "
            f"{sid:02d} {seg['segment_type']:<9s} "
            f"raw={float(seg['t_start_raw']):.1f}-{float(seg['t_end_raw']):.1f}s "
            f"refined={float(seg['t_start']):.1f}-{float(seg['t_end']):.1f}s "
            f"dist={float(seg['dist_m']):.1f}m "
            f"heading={heading:.2f}deg "
            f"source={seg['heading_source']} quality={seg['heading_quality']} "
            f"course(start/end/p50/p90/max)="
            f"{cstart:.2f}/{cend:.2f}/{p50:.2f}/{p90:.2f}/{cmax:.2f}deg"
        )

    return lines, warnings, errors


def print_report(analysis_dir: str, *, heading_tol_deg: float = 4.0) -> int:
    lines, warnings, errors = validate_analysis_dir(
        analysis_dir, heading_tol_deg=heading_tol_deg
    )
    print(f"[validate-segments] analysis_dir={analysis_dir}")
    for line in lines:
        print(line)
    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}")
    print(
        f"[validate-segments] segments={len(lines)} "
        f"warnings={len(warnings)} errors={len(errors)}"
    )
    return 1 if errors else 0
