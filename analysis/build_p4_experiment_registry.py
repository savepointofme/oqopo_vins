#!/usr/bin/env python3
"""Build the authoritative P4 experiment and evaluation registry.

The registry is read-only with respect to experiment artifacts. It inventories
all discoverable P4 run specifications, raw replay directories, batch roots,
and canonical evaluation outputs from both the compliant experiment root and
legacy Desktop locations.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import shlex
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable


CSV_FIELDS = [
    "record_id",
    "record_type",
    "experiment_domain",
    "campaign",
    "date",
    "flight",
    "method",
    "scope",
    "record_status",
    "evidence_grade",
    "location_contract",
    "source_path",
    "linked_trajectory",
    "linked_run_spec",
    "start_time_s",
    "end_time_s",
    "initialization_mode",
    "yaw_mode",
    "height_mode",
    "camera_stride",
    "alignment_mode",
    "sampling_mode",
    "exit_code",
    "trajectory_rows",
    "readiness",
    "selected_window_s",
    "handoff_model",
    "duration_s",
    "gps_distance_km",
    "xy_rmse_m",
    "final_xy_error_m",
    "speed_rmse_mps",
    "vxy_vec_rmse_mps",
    "course_rmse_deg",
    "course_final_deg",
    "final_vertical_error_m",
    "run_contract_hash",
    "repeat_group_size",
    "artifact_group_size",
    "notes",
]

RAW_MARKERS = {
    "command.txt",
    "exit_code.txt",
    "process_start.txt",
    "process_end.txt",
    "traj_nav.txt",
    "online_alignment_metadata.json",
    "r1_acceptance.json",
}

BATCH_MARKERS = {
    "batch_exit_codes.csv",
    "preflight.log",
    "preflight_initializer_test.log",
    "git_head.txt",
}

OUTPUT_PATH_FLAGS = {
    "--adaptive-stride-log",
    "--camera-stride-audit",
    "--diag-csv",
    "--diag-events",
    "--nav-frame-metadata-json",
    "--online-alignment-attempt-receipts-json",
    "--online-alignment-metadata-json",
    "--output",
    "--output-nav",
    "--output-raw",
}

PRUNE_DIRS = {
    ".git",
    "__pycache__",
    "data",
    "frozen_binary",
    "images",
    "plots",
    "source_snapshots",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--experiment-root", type=Path, required=True)
    parser.add_argument("--legacy-root", type=Path, action="append", default=[])
    parser.add_argument("--out-csv", type=Path, required=True)
    parser.add_argument("--out-summary", type=Path, required=True)
    parser.add_argument("--out-campaign-csv", type=Path, required=True)
    return parser.parse_args()


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8-sig", errors="replace").strip()
    except OSError:
        return ""


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def read_csv_first(path: Path) -> dict[str, str]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            return next(csv.DictReader(stream), {})
    except (OSError, StopIteration):
        return {}


def canonical_path(value: str | Path) -> str:
    text = str(value).strip().replace("\\", "/")
    match = re.match(r"^/mnt/([a-zA-Z])/(.*)$", text)
    if match:
        text = f"{match.group(1).upper()}:/{match.group(2)}"
    if re.match(r"^[a-zA-Z]:/", text):
        text = text[0].upper() + text[1:]
    return re.sub(r"/+", "/", text).rstrip("/")


def local_path(value: str | Path) -> Path | None:
    text = canonical_path(value)
    if re.match(r"^[A-Z]:/", text):
        return Path(text)
    return None


def is_under(path: str | Path, root: Path) -> bool:
    canonical = canonical_path(path).casefold()
    prefix = canonical_path(root).casefold() + "/"
    return canonical == canonical_path(root).casefold() or canonical.startswith(prefix)


def infer_flight(*values: object) -> str:
    for value in values:
        match = re.search(r"(?i)(?:^|[^a-z0-9])(fly[1-4])(?:[^a-z0-9]|$)", str(value))
        if match:
            return match.group(1).lower()
    return ""


def infer_date(*values: object) -> str:
    for value in values:
        match = re.search(r"(?<!\d)(20\d{6})(?!\d)", str(value))
        if match:
            return match.group(1)
    return ""


def infer_scope(*values: object) -> str:
    text = " ".join(str(value).casefold() for value in values)
    for scope in ("full", "focus", "screen", "short", "half_lap", "one_lap"):
        if scope in text:
            return scope
    return ""


def infer_domain(text: str, initialization_mode: str = "") -> str:
    folded = text.casefold().replace("\\", "/")
    has_p5 = any(token in folded for token in ("adaptive_stride", "adaptive-stride", "/p5/", "p5_"))
    has_p4 = initialization_mode == "online_multisensor_alignment" or any(
        token in folded for token in (
            "online_alignment",
            "online-alignment",
            "p4_direct",
            "p4_only",
            "p4_sliding",
            "/p4/",
        )
    )
    if has_p4 and has_p5:
        return "P4_WITH_P5"
    if has_p4:
        return "P4"
    if has_p5:
        return "P5_ONLY_IN_P4_CAMPAIGN"
    return "P4_RELATED_UNCLASSIFIED"


def count_lines(path: Path) -> int:
    if not path.is_file():
        return 0
    count = 0
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                count += block.count(b"\n")
    except OSError:
        return 0
    return count


def flag_value(tokens: list[str], name: str) -> str:
    try:
        index = tokens.index(name)
    except ValueError:
        return ""
    return tokens[index + 1] if index + 1 < len(tokens) else ""


def command_tokens(command: str) -> list[str]:
    try:
        return shlex.split(command, posix=True)
    except ValueError:
        return command.split()


def normalized_run_contract(tokens: list[str]) -> tuple[str, str]:
    kept: list[str] = []
    skip_next = False
    for token in tokens:
        if skip_next:
            skip_next = False
            continue
        if token in OUTPUT_PATH_FLAGS:
            kept.extend((token, "<OUTPUT>"))
            skip_next = True
            continue
        kept.append(canonical_path(token) if "/" in token or "\\" in token else token)
    normalized = " ".join(kept)
    digest = hashlib.sha256(normalized.encode("utf-8")).hexdigest() if normalized else ""
    return normalized, digest


def record_id(record_type: str, source_path: str) -> str:
    digest = hashlib.sha1(f"{record_type}|{canonical_path(source_path)}".encode("utf-8")).hexdigest()
    return f"P4-{record_type.upper()}-{digest[:12]}"


def base_record(record_type: str, campaign: str, source_path: Path) -> dict[str, Any]:
    record = {field: "" for field in CSV_FIELDS}
    record.update(
        record_id=record_id(record_type, str(source_path)),
        record_type=record_type,
        experiment_domain="P4_RELATED_UNCLASSIFIED",
        campaign=campaign,
        date=infer_date(source_path),
        flight=infer_flight(source_path),
        scope=infer_scope(source_path),
        source_path=canonical_path(source_path),
    )
    return record


def campaign_for(path: Path, scan_root: Path) -> str:
    try:
        relative = path.relative_to(scan_root)
        return relative.parts[0] if relative.parts else path.name
    except ValueError:
        return path.name


def location_contract(path: str, experiment_root: Path) -> str:
    return "COMPLIANT_EXPERIMENT_ROOT" if is_under(path, experiment_root) else "LEGACY_OUTSIDE_EXPERIMENT_ROOT"


def discover_campaign_roots(scan_root: Path) -> Iterable[Path]:
    if not scan_root.is_dir():
        return []
    return sorted(
        path for path in scan_root.iterdir()
        if path.is_dir() and path.name.casefold().startswith("p4")
    )


def parse_raw_run(path: Path, campaign: str, experiment_root: Path) -> dict[str, Any]:
    record = base_record("raw_run", campaign, path)
    command_path = path / "command.txt"
    command = read_text(command_path)
    tokens = command_tokens(command)
    _, contract_hash = normalized_run_contract(tokens)
    trajectory = path / "traj_nav.txt"
    if not trajectory.is_file() and (path / "traj.txt").is_file():
        trajectory = path / "traj.txt"
    exit_text = read_text(path / "exit_code.txt").splitlines()
    exit_code = exit_text[0].strip() if exit_text else ""
    rows = count_lines(trajectory)
    metadata = read_json(path / "online_alignment_metadata.json")
    acceptance = read_json(path / "r1_acceptance.json")
    if exit_code:
        status = "SUCCESS_WITH_TRAJECTORY" if exit_code == "0" and rows > 0 else (
            "SUCCESS_NO_TRAJECTORY" if exit_code == "0" else "FAILED"
        )
    elif rows > 0:
        status = "ARTIFACT_WITHOUT_EXIT_RECEIPT"
    else:
        status = "INCOMPLETE_OR_ABANDONED"
    readiness = metadata.get("readiness", metadata.get("alignment_readiness", ""))
    window = metadata.get("selected_window_duration_s", metadata.get("window_duration_s", ""))
    handoff = metadata.get("handoff_covariance_model", metadata.get("handoff_model", ""))
    initialization_mode = flag_value(tokens, "--initialization-mode")
    notes: list[str] = []
    if acceptance:
        notes.append(f"acceptance={acceptance.get('status', acceptance.get('passed', 'present'))}")
    if not command:
        notes.append("missing_command")
    record.update(
        date=infer_date(path, metadata.get("generated_at", "")),
        flight=infer_flight(path, command, metadata.get("flight", "")),
        method=path.name,
        experiment_domain=infer_domain(command, initialization_mode),
        scope=infer_scope(path, command),
        record_status=status,
        evidence_grade="RUNTIME_ONLY",
        location_contract=location_contract(path, experiment_root),
        linked_trajectory=canonical_path(trajectory) if trajectory.is_file() else "",
        start_time_s=flag_value(tokens, "--start-time"),
        end_time_s=flag_value(tokens, "--until-time"),
        initialization_mode=initialization_mode,
        yaw_mode=flag_value(tokens, "--yaw-mode"),
        height_mode=flag_value(tokens, "--height-mode"),
        camera_stride=flag_value(tokens, "--camera-frame-stride"),
        exit_code=exit_code,
        trajectory_rows=rows,
        readiness=readiness,
        selected_window_s=window,
        handoff_model=handoff,
        run_contract_hash=contract_hash,
        notes=";".join(notes),
    )
    return record


def parse_batch(path: Path, campaign: str, experiment_root: Path) -> dict[str, Any]:
    record = base_record("batch", campaign, path)
    batch_rows = read_text(path / "batch_exit_codes.csv")
    failed = bool(re.search(r"(?m)^[^,]+,(?!0\s*$)\d+\s*$", batch_rows))
    status = "FAILED" if failed else ("BATCH_RECEIPT_PRESENT" if batch_rows else "INCOMPLETE_OR_ABANDONED")
    record.update(
        method=path.name,
        experiment_domain=infer_domain(str(path)),
        record_status=status,
        evidence_grade="ORCHESTRATION_ONLY",
        location_contract=location_contract(path, experiment_root),
        notes="batch_exit_codes.csv" if batch_rows else "no_batch_exit_receipt",
    )
    return record


def parse_evaluation(path: Path, campaign: str, experiment_root: Path) -> dict[str, Any]:
    record = base_record("evaluation", campaign, path)
    summary = read_csv_first(path / "tables" / "global_summary.csv")
    spec = read_json(path / "metadata" / "run_spec_resolved.json")
    config = read_json(path / "metadata" / "analysis_config.json")
    inputs = spec.get("inputs", {}) if isinstance(spec.get("inputs"), dict) else {}
    traj_input = inputs.get("vio_traj", {}) if isinstance(inputs.get("vio_traj"), dict) else {}
    trajectory = str(traj_input.get("raw", spec.get("vio_traj", "")))
    alignment = config.get("align_mode", "")
    if not alignment and isinstance(spec.get("alignment"), dict):
        alignment = spec["alignment"].get("mode", "")
    sampling = ""
    if isinstance(config.get("sampling"), dict):
        sampling = config["sampling"].get("mode", "")
    if not sampling and isinstance(spec.get("sampling"), dict):
        sampling = spec["sampling"].get("mode", "")
    primary_metrics_present = bool(summary.get("xy_rmse_m", "") and summary.get("final_xy_error_m", ""))
    if not primary_metrics_present:
        grade = "INVALID_OR_INCOMPLETE_EVALUATION"
    elif alignment == "start_heading" and sampling == "gps_update_after_vio":
        grade = "FORMAL_MAIN"
    elif alignment in {"absolute_navigation_no_post_alignment", "absolute"}:
        grade = "FORMAL_ABSOLUTE_NO_POST"
    else:
        grade = "DIAGNOSTIC_EVALUATION"
    record.update(
        date=str(spec.get("date", infer_date(path))),
        flight=str(spec.get("flight_name", infer_flight(path))),
        method=str(spec.get("method_name", path.name)),
        experiment_domain=infer_domain(f"{spec.get('method_name', '')} {spec.get('notes', '')}"),
        scope=infer_scope(path, spec.get("notes", "")),
        record_status=(
            "EVALUATED" if summary and primary_metrics_present else
            "EVALUATION_WITH_NA_PRIMARY_METRICS" if summary else
            "EVALUATION_INCOMPLETE"
        ),
        evidence_grade=grade,
        location_contract=location_contract(path, experiment_root),
        linked_trajectory=canonical_path(trajectory) if trajectory else "",
        start_time_s=spec.get("t0", ""),
        end_time_s=spec.get("t1", ""),
        alignment_mode=alignment,
        sampling_mode=sampling,
        duration_s=summary.get("duration_s", ""),
        gps_distance_km=summary.get("gps_distance_km", ""),
        xy_rmse_m=summary.get("xy_rmse_m", ""),
        final_xy_error_m=summary.get("final_xy_error_m", ""),
        speed_rmse_mps=summary.get("speed_rmse_mps", ""),
        vxy_vec_rmse_mps=summary.get("vxy_vec_rmse_mps", ""),
        course_rmse_deg=summary.get("yaw_or_course_rmse_deg", ""),
        course_final_deg=summary.get("yaw_or_course_final_deg", ""),
        final_vertical_error_m=summary.get("final_vertical_error_m", ""),
        notes=str(spec.get("notes", "")),
    )
    return record


def parse_run_spec(path: Path, repo: Path, experiment_root: Path) -> dict[str, Any]:
    spec = read_json(path)
    campaign = str(spec.get("experiment_id", path.parent.name))
    record = base_record("run_spec", campaign, path)
    trajectory = str(spec.get("vio_traj", ""))
    trajectory_path = local_path(trajectory)
    exists = bool(trajectory_path and trajectory_path.is_file())
    alignment = spec.get("alignment", {}) if isinstance(spec.get("alignment"), dict) else {}
    sampling = spec.get("sampling", {}) if isinstance(spec.get("sampling"), dict) else {}
    record.update(
        date=str(spec.get("date", infer_date(path))),
        flight=str(spec.get("flight_name", infer_flight(path, spec))),
        method=str(spec.get("method_name", "")),
        experiment_domain="P4",
        scope=infer_scope(path, spec.get("notes", "")),
        record_status="SPEC_INPUTS_PRESENT" if exists else "SPEC_INPUT_MISSING",
        evidence_grade="SPEC_ONLY",
        location_contract=location_contract(trajectory, experiment_root) if trajectory else "NO_TRAJECTORY_DECLARED",
        linked_trajectory=canonical_path(trajectory) if trajectory else "",
        linked_run_spec=canonical_path(path.relative_to(repo)),
        start_time_s=spec.get("t0", ""),
        end_time_s=spec.get("t1", ""),
        alignment_mode=alignment.get("mode", ""),
        sampling_mode=sampling.get("mode", ""),
        trajectory_rows=count_lines(trajectory_path) if exists and trajectory_path else 0,
        notes=str(spec.get("notes", "")),
    )
    return record


def discover_records(repo: Path, experiment_root: Path, legacy_roots: list[Path]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for spec_path in sorted((repo / "analysis" / "run_specs").rglob("*.json")):
        spec = read_json(spec_path)
        identity = " ".join((str(spec_path), str(spec.get("experiment_id", "")), str(spec.get("method_name", ""))))
        if "p4" in identity.casefold():
            records.append(parse_run_spec(spec_path, repo, experiment_root))

    for scan_root in [experiment_root, *legacy_roots]:
        for campaign_root in discover_campaign_roots(scan_root):
            campaign = campaign_root.name
            for current, dirs, files in os.walk(campaign_root):
                dirs[:] = [name for name in dirs if name not in PRUNE_DIRS and not name.startswith(".")]
                path = Path(current)
                file_set = set(files)
                if (path / "tables" / "global_summary.csv").is_file():
                    records.append(parse_evaluation(path, campaign, experiment_root))
                raw_support = file_set & RAW_MARKERS
                if raw_support and ({"traj_nav.txt", "exit_code.txt", "online_alignment_metadata.json"} & file_set):
                    records.append(parse_raw_run(path, campaign, experiment_root))
                if (file_set & BATCH_MARKERS) and "batch_exit_codes.csv" in file_set:
                    records.append(parse_batch(path, campaign, experiment_root))
    unique: dict[tuple[str, str], dict[str, Any]] = {}
    for record in records:
        key = (str(record["record_type"]), canonical_path(record["source_path"]).casefold())
        unique[key] = record
    return list(unique.values())


def annotate_groups(records: list[dict[str, Any]]) -> None:
    contract_groups: defaultdict[str, list[dict[str, Any]]] = defaultdict(list)
    artifact_groups: defaultdict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        if record.get("run_contract_hash"):
            contract_groups[str(record["run_contract_hash"])].append(record)
        if record.get("linked_trajectory"):
            artifact_groups[canonical_path(record["linked_trajectory"]).casefold()].append(record)
    for group in contract_groups.values():
        for record in group:
            record["repeat_group_size"] = len(group)
    for group in artifact_groups.values():
        for record in group:
            record["artifact_group_size"] = len(group)


def write_outputs(records: list[dict[str, Any]], args: argparse.Namespace) -> None:
    records.sort(key=lambda row: (str(row["date"]), str(row["campaign"]), str(row["flight"]), str(row["record_type"]), str(row["source_path"])))
    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.out_csv.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(records)

    missing_specs = [
        record["source_path"] for record in records
        if record["record_type"] == "run_spec" and record["record_status"] == "SPEC_INPUT_MISSING"
    ]
    summary = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "repo": canonical_path(args.repo),
        "experiment_root": canonical_path(args.experiment_root),
        "legacy_roots": [canonical_path(path) for path in args.legacy_root],
        "record_count": len(records),
        "counts_by_type": dict(sorted(Counter(str(row["record_type"]) for row in records).items())),
        "counts_by_experiment_domain": dict(sorted(Counter(str(row["experiment_domain"]) for row in records).items())),
        "counts_by_status": dict(sorted(Counter(str(row["record_status"]) for row in records).items())),
        "counts_by_evidence_grade": dict(sorted(Counter(str(row["evidence_grade"]) for row in records).items())),
        "counts_by_location_contract": dict(sorted(Counter(str(row["location_contract"]) for row in records).items())),
        "counts_by_campaign": [
            {"campaign": campaign, "count": count}
            for campaign, count in sorted(Counter(str(row["campaign"]) for row in records).items())
        ],
        "missing_run_spec_inputs": missing_specs,
        "repeat_contract_groups": sum(1 for row in records if str(row.get("repeat_group_size", "")) not in {"", "1"}),
        "artifact_link_groups": sum(1 for row in records if str(row.get("artifact_group_size", "")) not in {"", "1"}),
    }
    args.out_summary.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    campaign_fields = [
        "campaign",
        "record_count",
        "raw_run_count",
        "successful_raw_run_count",
        "failed_raw_run_count",
        "evaluation_count",
        "formal_main_evaluation_count",
        "diagnostic_evaluation_count",
        "run_spec_count",
        "batch_count",
        "legacy_location_count",
        "flights",
        "domains",
        "first_date",
        "last_date",
    ]
    campaign_groups: defaultdict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        campaign_groups[str(record["campaign"])].append(record)
    args.out_campaign_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.out_campaign_csv.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=campaign_fields)
        writer.writeheader()
        for campaign, group in sorted(campaign_groups.items()):
            dates = sorted(str(row["date"]) for row in group if str(row["date"]))
            writer.writerow({
                "campaign": campaign,
                "record_count": len(group),
                "raw_run_count": sum(row["record_type"] == "raw_run" for row in group),
                "successful_raw_run_count": sum(
                    row["record_type"] == "raw_run" and row["record_status"] == "SUCCESS_WITH_TRAJECTORY"
                    for row in group
                ),
                "failed_raw_run_count": sum(
                    row["record_type"] == "raw_run" and row["record_status"] in {"FAILED", "SUCCESS_NO_TRAJECTORY"}
                    for row in group
                ),
                "evaluation_count": sum(row["record_type"] == "evaluation" for row in group),
                "formal_main_evaluation_count": sum(row["evidence_grade"] == "FORMAL_MAIN" for row in group),
                "diagnostic_evaluation_count": sum(row["evidence_grade"] == "DIAGNOSTIC_EVALUATION" for row in group),
                "run_spec_count": sum(row["record_type"] == "run_spec" for row in group),
                "batch_count": sum(row["record_type"] == "batch" for row in group),
                "legacy_location_count": sum(row["location_contract"] == "LEGACY_OUTSIDE_EXPERIMENT_ROOT" for row in group),
                "flights": ";".join(sorted({str(row["flight"]) for row in group if str(row["flight"])})),
                "domains": ";".join(sorted({str(row["experiment_domain"]) for row in group if str(row["experiment_domain"])})),
                "first_date": dates[0] if dates else "",
                "last_date": dates[-1] if dates else "",
            })


def main() -> int:
    args = parse_args()
    records = discover_records(args.repo.resolve(), args.experiment_root.resolve(), [path.resolve() for path in args.legacy_root])
    annotate_groups(records)
    write_outputs(records, args)
    print(json.dumps({
        "records": len(records),
        "out_csv": canonical_path(args.out_csv),
        "out_summary": canonical_path(args.out_summary),
        "out_campaign_csv": canonical_path(args.out_campaign_csv),
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
