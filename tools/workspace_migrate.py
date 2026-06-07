#!/usr/bin/env python3
"""
Workspace migration tool for open_vins.

Default mode: DRY-RUN. Prints all planned operations without making changes.
Apply mode:   Pass --apply to execute moves and copies.

Usage:
    python tools/workspace_migrate.py [OPTIONS]

Options:
    --apply          Execute the migration (default: dry-run only)
    --groups GROUP   Comma-separated list of groups to run (default: all)
                     Groups: tools,diag,scratch,docs,archive,experiments,truth
    --workspace DIR  Workspace root (default: parent of this script)
    --manifest PATH  Path to write migration manifest JSON
    --help           Show this help message

Safety rules (always enforced):
    - Never overwrite an existing file
    - Skip source files in PROTECTED list for MOVE operations
      (copy operations from protected sources are allowed — Strategy B)
    - Warn about large files (>100 MB)
    - Warn about git-tracked files (use git mv separately)
    - Log every planned/executed operation
    - Default is dry-run; --apply must be explicit

Operation types:
    move               — shutil.move(src, dst)
    copy               — shutil.copy2(src, dst); original stays in place
    move_with_fixup    — move + add sys.path fixup if _pseudo_ref_guard import detected

Strategy B (truth/ copies):
    Canonical GPS and pseudo-ref files are COPIED into truth/jc82/flyN/.
    Originals remain at their protected locations.
    A SOURCES.yaml with checksum, row-count, and timestamp range is written
    alongside the copied files.

Example (dry-run — default):
    python tools/workspace_migrate.py

Example (scratch group only — dry-run):
    python tools/workspace_migrate.py --groups scratch

Example (apply everything):
    python tools/workspace_migrate.py --apply

Example (apply scratch group only):
    python tools/workspace_migrate.py --groups scratch --apply

Notes:
    - Git-tracked files are listed as warnings but NOT moved by this script.
      Use 'git mv' manually for those.
    - open_vins_pr17/ (3.6 GB stale repo copy) is NOT moved automatically.
      Recommended action: delete manually after confirming it is not needed.
      (git checkout covers all its content if needed.)
    - Hardcoded /mnt/d/vscode_dir/open_vins paths remain in several scripts.
      See tools/diag/README.md and tools/run/README.md for the affected files.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime
from pathlib import Path


# ---------------------------------------------------------------------------
# Protected paths — MOVE operations on these sources are blocked
# COPY operations FROM these sources are allowed (Strategy B)
# ---------------------------------------------------------------------------
PROTECTED_PREFIXES = [
    # Raw datasets
    "20260509_fly1/mav0",
    "20260509_fly2/mav0",
    "20260509_fly3/mav0",
    "20260509_fly4/mav0",
    # Canonical aligned GPS + pseudo-ref directories
    "20260509_fly1/result/gps_tum_time_alignment_offset_m4p63",
    "20260509_fly2/result/gps_tum_time_alignment_vertical",
    "20260509_fly3/result/gps_tum_time_alignment",
    "20260509_fly4/result/gps_tum_time_alignment",
    # Pseudo-reference trajectory directories
    "20260509_fly1/reference",
    "20260509_fly2/reference",
    "20260509_fly3/reference",
    "20260509_fly4/reference",
    # Build output
    "build_ov_msckf",
    # Git-tracked source packages
    "ov_core", "ov_init", "ov_msckf", "ov_eval", "ov_data", "config",
    "docs", "docs-cn",
    # Calibration
    "calib_results",
    # Root-level canonical docs
    "BASELINES.md", "CLAUDE.md", "README.md", "ReadMe.md", "LICENSE",
    ".gitignore", ".clang-format", ".github",
    # Already-placed canonical tools
    "tools/eval_baselines.py",
    "tools/_pseudo_ref_guard.py",
]

# Git-tracked files — warn but do not move; user must use git mv
GIT_TRACKED = [
    "BASELINES.md",
    "tools/eval_baselines.py",
    "run_format.sh", "run_copyright.sh", "run_size.sh",
]

LARGE_FILE_THRESHOLD_MB = 100

# Guessed purpose for each scratch file (keyed by stem without extension)
SCRATCH_PURPOSES = {
    "analyze_dt_ab_scan":               "diagnostic: A/B delta-t parameter scan analysis",
    "analyze_gps_diff":                 "diagnostic: GPS trace comparison between flights",
    "check_fly2":                       "one-off: fly2 data sanity check",
    "check_gps_match":                  "one-off: GPS timestamp alignment verification",
    "clock_analysis":                   "diagnostic: IMU/camera clock drift analysis",
    "compare_csv":                      "utility: general CSV diff/comparison",
    "compare_experiments":              "diagnostic: compare two experiment run outputs",
    "compare_fly4_all":                 "diagnostic: compare all fly4 experiment variants",
    "compare_pr20":                     "diagnostic: PR20 result comparison",
    "compare_pr20_v2":                  "diagnostic: PR20 result comparison v2",
    "compare_trend":                    "diagnostic: performance trend plots",
    "compare_vs_vins":                  "diagnostic: compare against VINS-Mono baseline",
    "debug_fly2":                       "diagnostic: fly2 debugging",
    "debug_fly4":                       "diagnostic: fly4 debugging",
    "diagnose_gps_baseline":            "diagnostic: GPS baseline quality check",
    "diagnose_gps_overlap":             "diagnostic: GPS/VIO time overlap analysis",
    "diagnose_gps_tum_alignment":       "diagnostic: GPS-TUM alignment verification",
    "diagnose_raw_gps_tum_alignment":   "diagnostic: raw GPS alignment pipeline check",
    "diagnose_vngps_diff":              "diagnostic: VN GPS difference analysis",
    "final_compare":                    "diagnostic: final result comparison for reporting",
    "generate_algorithm_accuracy_report_cn": "report: Chinese-language accuracy report generator",
    "investigate_fly3_missing":         "diagnostic: fly3 missing data investigation",
    "investigate_missing2":             "diagnostic: missing data investigation (iteration 2)",
    "plot_fly4_guard":                  "diagnostic: fly4 guard/threshold behavior plot",
    "quick_compare":                    "diagnostic: quick trajectory comparison",
    "report_pr20_gps_alt_fusion":       "report: PR20 GPS altitude fusion results summary",
    "revise_fly_report_v18_cn":         "report: Chinese report revision v18",
    "scan_dt_ab_comprehensive":         "diagnostic: comprehensive A/B delta-t scan",
    "spot_check_final":                 "diagnostic: spot-check final results",
    "verify_merge":                     "diagnostic: branch merge verification",
    "StaticInitializer_copy":           "source backup: stale copy of ov_init/src/static/StaticInitializer.cpp",
    "traj_ros_free":                    "temp output: trajectory from ros-free runner test",
    "compare_summary":                  "note: temporary comparison summary",
    "pr20_compare_summary":             "note: PR20 comparison notes",
    "run_pr17_mono_stereo_commands":    "commands: PR17 monocular/stereo historical run commands",
}


# ---------------------------------------------------------------------------
# Operation constructors
# ---------------------------------------------------------------------------
def op(src, dst, reason, confirmation_required=False, skip_if_missing=True):
    return {
        "op_type": "move",
        "src": str(src),
        "dst": str(dst),
        "reason": reason,
        "confirmation_required": confirmation_required,
        "skip_if_missing": skip_if_missing,
    }


def copy_op(src, dst, reason, skip_if_missing=True):
    """Copy operation — source is NOT deleted. Used for Strategy B truth copies."""
    return {
        "op_type": "copy",
        "src": str(src),
        "dst": str(dst),
        "reason": reason,
        "confirmation_required": False,
        "skip_if_missing": skip_if_missing,
    }


def fixup_op(src, dst, reason, skip_if_missing=True):
    """Move + add sys.path fixup header if _pseudo_ref_guard import detected."""
    return {
        "op_type": "move_with_fixup",
        "src": str(src),
        "dst": str(dst),
        "reason": reason,
        "confirmation_required": False,
        "skip_if_missing": skip_if_missing,
    }


# ---------------------------------------------------------------------------
# Migration plan
# ---------------------------------------------------------------------------
def get_migration_plan(workspace):
    w = workspace
    plan = []

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: tools — reusable scripts from root → tools/<category>/
    # ─────────────────────────────────────────────────────────────────────
    for f in ["align_gps_traj_timestamps.py", "align_gps_tum_truth.py", "imu_align.py"]:
        plan.append(op(w / f, w / "tools" / "align" / f,
                       "reusable alignment tool → tools/align/"))
    for f in ["extract_bag.py", "extract_gps.sh"]:
        plan.append(op(w / f, w / "tools" / "convert" / f,
                       "reusable conversion tool → tools/convert/"))
    for f in ["run_iwt_eval.py", "evaluate_gps_start_yaw.py", "plot_trajectories.py"]:
        plan.append(op(w / f, w / "tools" / "eval" / f,
                       "reusable evaluation tool → tools/eval/"))
    for f in ["run_iwt5_validation.sh"]:
        plan.append(op(w / f, w / "tools" / "run" / f,
                       "reusable run script → tools/run/"))

    # Reorganize tools/ internal eval scripts
    for f in ["plot_per_edge_traj.py", "audit_gps_alt_v2_zip.py", "audit_gps_references.py"]:
        plan.append(op(w / "tools" / f, w / "tools" / "eval" / f,
                       "eval tool — reorganize within tools/ → tools/eval/"))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: diag — tools/diag_*.py → tools/diag/
    # Import fixup applied during --apply for scripts importing _pseudo_ref_guard
    # ─────────────────────────────────────────────────────────────────────
    for fpath in sorted(glob.glob(str(w / "tools" / "diag_*.py"))):
        fname = os.path.basename(fpath)
        plan.append(fixup_op(w / "tools" / fname, w / "tools" / "diag" / fname,
                              "diagnostic script → tools/diag/"))

    # lilili evaluate_vio_gps.py → tools/eval/evaluate_by_li.py (copy before archive)
    plan.append(copy_op(
        w / "lilili" / "evaluate_vio_gps.py",
        w / "tools" / "eval" / "evaluate_by_li.py",
        "reusable VIO-GPS evaluator extracted from lilili/ → tools/eval/",
    ))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: scratch — one-off scripts from root → scratch/
    # ─────────────────────────────────────────────────────────────────────
    SCRATCH_SCRIPTS = [
        "analyze_dt_ab_scan.py",
        "analyze_gps_diff.py",
        "check_fly2.py",
        "check_gps_match.py",
        "clock_analysis.py",
        "compare_csv.py",
        "compare_experiments.py",
        "compare_fly4_all.py",
        "compare_pr20.py",
        "compare_pr20_v2.py",
        "compare_trend.py",
        "compare_vs_vins.py",
        "debug_fly2.py",
        "debug_fly4.py",
        "diagnose_gps_baseline.py",
        "diagnose_gps_overlap.py",
        "diagnose_gps_tum_alignment.py",
        "diagnose_raw_gps_tum_alignment.py",
        "diagnose_vngps_diff.py",
        "final_compare.py",
        "generate_algorithm_accuracy_report_cn.py",
        "investigate_fly3_missing.py",
        "investigate_missing2.py",
        "plot_fly4_guard.py",
        "quick_compare.py",
        "report_pr20_gps_alt_fusion.py",
        "revise_fly_report_v18_cn.py",
        "scan_dt_ab_comprehensive.py",
        "spot_check_final.py",
        "verify_merge.py",
        "traj_ros_free.txt",
        "traj_ros_free.txt.bias",
    ]
    for f in SCRATCH_SCRIPTS:
        plan.append(op(w / f, w / "scratch" / f,
                       "one-off script or temp output → scratch/"))

    # StaticInitializer copy — approved for scratch (no confirmation required)
    plan.append(op(
        w / "StaticInitializer copy.cpp",
        w / "scratch" / "StaticInitializer_copy.cpp",
        "stale source backup → scratch/ (canonical at ov_init/src/static/StaticInitializer.cpp)",
    ))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: docs — markdown docs from root → docs/ or scratch/
    # ─────────────────────────────────────────────────────────────────────
    plan.append(op(w / "GPS_REFERENCE_AUDIT.md", w / "docs" / "GPS_REFERENCE_AUDIT.md",
                   "canonical GPS policy doc → docs/"))
    for f in ["compare_summary.md", "pr20_compare_summary.md"]:
        plan.append(op(w / f, w / "scratch" / f, "temp comparison note → scratch/"))
    plan.append(op(w / "run_pr17_mono_stereo_commands.md",
                   w / "scratch" / "run_pr17_mono_stereo_commands.md",
                   "historical run commands → scratch/"))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: archive — zip packages and historical directories
    # ─────────────────────────────────────────────────────────────────────
    for f in ["pr20_compare.zip", "pr20_compare_v2.zip",
              "pr20_final_package.zip", "pr20_tum_package.zip"]:
        plan.append(op(w / f, w / "archive" / "pr20_packages" / f,
                       "PR20 archive → archive/pr20_packages/"))
    plan.append(op(w / "openvins-fix-20260512.tar.gz",
                   w / "archive" / "openvins-fix-20260512.tar.gz",
                   "build snapshot → archive/"))

    # GPS zip archives — non-canonical, no confirmation required
    for f in ["gps_altitude_data.zip", "gps_altitude_data_v2.zip"]:
        plan.append(op(w / f, w / "archive" / f,
                       "GPS data archive — non-canonical → archive/"))

    # lilili/ directory — archive after evaluate_vio_gps.py is extracted above
    plan.append(op(w / "lilili", w / "archive" / "lilili",
                   "test workspace → archive/lilili/"))

    # Other old directories
    plan.append(op(w / "evaluation_start_yaw", w / "archive" / "evaluation_start_yaw",
                   "old eval using stereo_pseudo_ref → archive/"))
    plan.append(op(w / "diagnostics_time_alignment", w / "archive" / "diagnostics_time_alignment",
                   "old diagnostic plots → archive/"))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: experiments — directory-level experiment results
    # ─────────────────────────────────────────────────────────────────────
    plan.append(op(w / "pr20_final_package",
                   w / "experiments" / "pr20_gps_alt_fusion",
                   "PR20 GPS-alt fusion experiment → experiments/pr20_gps_alt_fusion/"))
    plan.append(op(w / "evaluation_iwt5_vs_iwt2",
                   w / "experiments" / "iwt5_vs_iwt2_eval",
                   "iwt sweep evaluation results → experiments/iwt5_vs_iwt2_eval/"))
    plan.append(op(w / "comparison_plots",
                   w / "experiments" / "stage_b_v1_diagnostics" / "plots",
                   "stage B diagnostic plots → experiments/stage_b_v1_diagnostics/plots/"))
    plan.append(op(w / "report_pr20_gps_alt_fusion.docx",
                   w / "experiments" / "pr20_gps_alt_fusion" / "report_pr20_gps_alt_fusion.docx",
                   "PR20 report doc → experiments/pr20_gps_alt_fusion/"))

    # ─────────────────────────────────────────────────────────────────────
    # GROUP: truth — Strategy B: COPY canonical GPS + pseudo-ref to truth/
    # Original files are NOT moved — they stay in their protected locations.
    # A SOURCES.yaml with metadata is written to each truth/jc82/flyN/ directory.
    #
    # Intentional omissions (documented in SOURCES.yaml):
    #   - fly1 mav0/gps/aligned_gps_from_raw_*.csv (8 files): STALE — derived
    #     from truncated GPS (348 s). Do not copy.
    #   - fly1 mav0/gps/GPS.truncated.DO_NOT_USE.csv: BANNED — 348 s only.
    #   - fly2 gps_tum_time_alignment_corrected/: BANNED — cross-contaminated
    #     (fly1 GPS retimed to fly2 clock).
    #   - truth_asl_imu_time.csv: omitted — imu-time pseudo-ref not useful
    #     for evaluation; only cam_time pseudo-ref is provided.
    # ─────────────────────────────────────────────────────────────────────
    TRUTH_COPIES = [
        # (src_relative, dst_relative, description)

        # --- Raw GPS (WGS84) ---
        (
            "20260509_fly1/mav0/gps/GPS.csv",
            "truth/jc82/fly1/gps_raw.csv",
            "raw ArduPilot GPS fly1 (WGS84: Lat,Lng,Alt — NOT ENU XYZ) [copy]",
        ),
        (
            "20260509_fly2/mav0/gps/GPS.csv",
            "truth/jc82/fly2/gps_raw.csv",
            "raw ArduPilot GPS fly2 (WGS84: Lat,Lng,Alt — NOT ENU XYZ) [copy]",
        ),
        (
            "20260509_fly3/mav0/gps/GPS.csv",
            "truth/jc82/fly3/gps_raw.csv",
            "raw ArduPilot GPS fly3 (WGS84: Lat,Lng,Alt — NOT ENU XYZ) [copy]",
        ),
        (
            "20260509_fly4/mav0/gps/GPS.csv",
            "truth/jc82/fly4/gps_raw.csv",
            "raw ArduPilot GPS fly4 (WGS84: Lat,Lng,Alt — NOT ENU XYZ) [copy]",
        ),

        # --- Aligned GPS (cam-time, ENU XYZ) ---
        (
            "20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_cam_time.csv",
            "truth/jc82/fly1/gps_aligned_cam_time.csv",
            "canonical GPS fly1 cam-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_cam_time.csv",
            "truth/jc82/fly2/gps_aligned_cam_time.csv",
            "canonical GPS fly2 cam-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly3/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
            "truth/jc82/fly3/gps_aligned_cam_time.csv",
            "canonical GPS fly3 cam-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly4/result/gps_tum_time_alignment/aligned_gps_cam_time.csv",
            "truth/jc82/fly4/gps_aligned_cam_time.csv",
            "canonical GPS fly4 cam-time (ENU XYZ) [copy — original stays in place]",
        ),

        # --- Aligned GPS (imu-time, ENU XYZ) ---
        (
            "20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/aligned_gps_imu_time.csv",
            "truth/jc82/fly1/gps_aligned_imu_time.csv",
            "canonical GPS fly1 imu-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly2/result/gps_tum_time_alignment_vertical/aligned_gps_imu_time.csv",
            "truth/jc82/fly2/gps_aligned_imu_time.csv",
            "canonical GPS fly2 imu-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly3/result/gps_tum_time_alignment/aligned_gps_imu_time.csv",
            "truth/jc82/fly3/gps_aligned_imu_time.csv",
            "canonical GPS fly3 imu-time (ENU XYZ) [copy — original stays in place]",
        ),
        (
            "20260509_fly4/result/gps_tum_time_alignment/aligned_gps_imu_time.csv",
            "truth/jc82/fly4/gps_aligned_imu_time.csv",
            "canonical GPS fly4 imu-time (ENU XYZ) [copy — original stays in place]",
        ),

        # --- Stereo pseudo-reference trajectories (DEBUG ONLY — not ground truth) ---
        (
            "20260509_fly1/result/gps_tum_time_alignment_offset_m4p63/truth_asl_cam_time.csv",
            "truth/jc82/fly1/stereo_pseudo_ref_cam_time.csv",
            "stereo pseudo-ref fly1 [DEBUG ONLY — NOT ground truth] (copy)",
        ),
        (
            "20260509_fly2/result/gps_tum_time_alignment_vertical/truth_asl_cam_time.csv",
            "truth/jc82/fly2/stereo_pseudo_ref_cam_time.csv",
            "stereo pseudo-ref fly2 [DEBUG ONLY — NOT ground truth] (copy)",
        ),
        (
            "20260509_fly3/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
            "truth/jc82/fly3/stereo_pseudo_ref_cam_time.csv",
            "stereo pseudo-ref fly3 [DEBUG ONLY — NOT ground truth] (copy)",
        ),
        (
            "20260509_fly4/result/gps_tum_time_alignment/truth_asl_cam_time.csv",
            "truth/jc82/fly4/stereo_pseudo_ref_cam_time.csv",
            "stereo pseudo-ref fly4 [DEBUG ONLY — NOT ground truth] (copy)",
        ),
    ]
    for src_rel, dst_rel, desc in TRUTH_COPIES:
        plan.append(copy_op(w / src_rel, w / dst_rel, desc))

    return plan


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def is_protected(path, workspace):
    try:
        rel = path.relative_to(workspace)
    except ValueError:
        return False
    rel_str = str(rel).replace("\\", "/")
    for prefix in PROTECTED_PREFIXES:
        if rel_str == prefix or rel_str.startswith(prefix + "/"):
            return True
    return False


def is_git_tracked(path, workspace):
    try:
        rel = str(path.relative_to(workspace)).replace("\\", "/")
    except ValueError:
        return False
    return rel in GIT_TRACKED


def file_size_mb(path):
    try:
        if path.is_dir():
            total = sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
            return total / (1024 * 1024)
        return path.stat().st_size / (1024 * 1024)
    except Exception:
        return 0.0


def format_size(mb):
    if mb >= 1024:
        return "{:.1f} GB".format(mb / 1024)
    if mb >= 1:
        return "{:.1f} MB".format(mb)
    return "{:.0f} KB".format(mb * 1024)


def compute_file_metadata(path):
    """Return checksum, row count, timestamp range, and altitude range for a CSV/TUM file."""
    meta = {
        "size_bytes": 0,
        "sha256": "",
        "rows": 0,
        "first_timestamp_s": None,
        "last_timestamp_s": None,
    }
    if not path.exists() or path.is_dir():
        return meta

    meta["size_bytes"] = path.stat().st_size

    h = hashlib.sha256()
    rows = 0
    first_ts = None
    last_ts = None
    alt_vals = []

    try:
        with open(str(path), "rb") as f:
            for raw_line in f:
                h.update(raw_line)
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                # Handle comma-separated CSV (e.g. raw GPS WGS84: TimeUS,Lat,Lng,Alt,...)
                if len(parts) == 1 and "," in parts[0]:
                    parts = parts[0].split(",")
                rows += 1
                try:
                    ts = float(parts[0])
                    if first_ts is None:
                        first_ts = ts
                    last_ts = ts
                    if len(parts) >= 4:
                        alt_vals.append(float(parts[3]))
                except (ValueError, IndexError):
                    pass
    except Exception:
        pass

    meta["sha256"] = h.hexdigest()
    meta["rows"] = rows
    if first_ts is not None:
        meta["first_timestamp_s"] = round(first_ts, 6)
        meta["last_timestamp_s"] = round(last_ts, 6)
    if alt_vals:
        meta["alt_min_m"] = round(min(alt_vals), 3)
        meta["alt_max_m"] = round(max(alt_vals), 3)

    return meta


def write_sources_yaml(truth_dir, entries, flight_id):
    """Write truth/jc82/flyN/SOURCES.yaml documenting all copied files and intentional omissions."""

    # Per-flight intentional omissions
    OMISSIONS = {
        "fly1": [
            "mav0/gps/aligned_gps_from_raw_*.csv (8 files) — STALE: derived from truncated GPS (348 s)",
            "mav0/gps/GPS.truncated.DO_NOT_USE.csv — BANNED: truncated to first 348 s",
        ],
        "fly2": [
            "gps_tum_time_alignment_corrected/ — BANNED: cross-contaminated (fly1 GPS retimed to fly2 clock)",
        ],
        "fly3": [
            "gps_tum_time_alignment_fly4gps/ — UNCERTAIN: fly4 GPS cross-tested with fly3. Not canonical.",
        ],
        "fly4": [
            # No intentional omissions for fly4
        ],
    }

    lines = [
        "# SOURCES.yaml — canonical truth/GPS sources for {}".format(flight_id),
        "# Generated: {} by tools/workspace_migrate.py".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
        "#",
        "# WARNING: stereo_pseudo_ref files are DEBUG ONLY — NOT ground truth.",
        "# NEVER use them as official Z reference.",
        "#",
        "# File types:",
        "#   gps_raw                  — raw ArduPilot GPS (WGS84: Lat,Lng,Alt,TimeUS in microseconds)",
        "#   gps_aligned_cam_time     — GPS aligned to camera time (ENU XYZ)",
        "#   gps_aligned_imu_time     — GPS aligned to IMU time (ENU XYZ)",
        "#   stereo_pseudo_ref        — stereo VIO output (DEBUG ONLY, not truth)",
        "#",
        "files:",
    ]
    for entry in entries:
        lines.append("  {}:".format(entry["filename"]))
        lines.append("    original_path: \"{}\"".format(entry["original_path"]))
        lines.append("    type: \"{}\"".format(entry["file_type"]))
        lines.append("    note: \"{}\"".format(entry["note"]))
        meta = entry.get("meta", {})
        lines.append("    size_bytes: {}".format(meta.get("size_bytes", 0)))
        lines.append("    rows: {}".format(meta.get("rows", 0)))
        if meta.get("first_timestamp_s") is not None:
            lines.append("    first_timestamp_s: {}".format(meta["first_timestamp_s"]))
            lines.append("    last_timestamp_s: {}".format(meta["last_timestamp_s"]))
        if "alt_min_m" in meta:
            lines.append("    alt_min_m: {}".format(meta["alt_min_m"]))
            lines.append("    alt_max_m: {}".format(meta["alt_max_m"]))
        lines.append("    sha256: \"{}\"".format(meta.get("sha256", "")))

    omission_list = OMISSIONS.get(flight_id, [])
    if omission_list:
        lines.append("")
        lines.append("# -----------------------------------------------------------------------")
        lines.append("# Intentional omissions — known files that exist but were NOT copied")
        lines.append("# -----------------------------------------------------------------------")
        lines.append("intentional_omissions:")
        for o in omission_list:
            lines.append("  - \"{}\"".format(o))

    sources_path = truth_dir / "SOURCES.yaml"
    if not sources_path.exists():
        sources_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print("  WRITE  {}".format(sources_path))
    else:
        print("  SKIP   SOURCES.yaml (already exists)")


def needs_import_fixup(path):
    """Return True if this script imports _pseudo_ref_guard (will break after move to subdir)."""
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
        return "_pseudo_ref_guard" in content
    except Exception:
        return False


_IMPORT_FIXUP = """\
# path-setup: added by workspace_migrate.py so _pseudo_ref_guard is found after
# this script was moved from tools/ to tools/diag/
import os as _ws_os, sys as _ws_sys
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..'))
_ws_sys.path.insert(0, _ws_os.path.join(_ws_os.path.dirname(_ws_os.path.abspath(__file__)), '..', 'eval'))
del _ws_os, _ws_sys
"""


def apply_import_fixup(path):
    """Insert sys.path fixup into the script content after the first line (shebang/docstring)."""
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
        # Insert after shebang line if present, otherwise at top
        lines = content.splitlines(keepends=True)
        insert_at = 0
        if lines and lines[0].startswith("#!"):
            insert_at = 1
        new_lines = lines[:insert_at] + [_IMPORT_FIXUP] + lines[insert_at:]
        path.write_text("".join(new_lines), encoding="utf-8")
        print("    +fix  sys.path fixup injected (needed for _pseudo_ref_guard import)")
    except Exception as exc:
        print("    WARN  could not apply import fixup: {}".format(exc))


def generate_scratch_index(workspace, scratch_moves):
    """Write scratch/SCRATCH_INDEX.md listing all moved scripts with guessed purpose."""
    index_path = workspace / "scratch" / "SCRATCH_INDEX.md"
    if index_path.exists():
        print("  SKIP   scratch/SCRATCH_INDEX.md (already exists)")
        return

    lines = [
        "# scratch/ Index",
        "",
        "Files in this directory were moved here by `tools/workspace_migrate.py`.",
        "They are one-off diagnostic scripts or temporary outputs.",
        "**Rescue policy:** if a script is still useful, move it to `tools/` or a proper location.",
        "**Cleanup policy:** delete anything here that has never been needed after a few months.",
        "",
        "| Original path | scratch/ file | Guessed purpose |",
        "|---------------|---------------|-----------------|",
    ]
    for src_rel, dst_rel in scratch_moves:
        stem = Path(dst_rel).stem
        purpose = SCRATCH_PURPOSES.get(stem, "unknown — inspect manually")
        lines.append("| `{}` | `{}` | {} |".format(src_rel, dst_rel, purpose))

    lines.append("")
    index_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("  WRITE  scratch/SCRATCH_INDEX.md  ({} entries)".format(len(scratch_moves)))


# ---------------------------------------------------------------------------
# Core migration logic
# ---------------------------------------------------------------------------
def run_migration(workspace, apply, groups, manifest_path):
    plan = get_migration_plan(workspace)

    GROUP_KEYWORDS = {
        "tools":       ["tools/align/", "tools/convert/", "tools/eval/", "tools/run/"],
        "diag":        ["tools/diag/"],
        "scratch":     ["scratch/"],
        "docs":        ["docs/"],
        "archive":     ["archive/"],
        "experiments": ["experiments/"],
        "truth":       ["truth/jc82/"],
    }

    if groups:
        filtered = []
        for item in plan:
            for g in groups:
                if g in GROUP_KEYWORDS:
                    if any(kw in item["dst"].replace("\\", "/") for kw in GROUP_KEYWORDS[g]):
                        filtered.append(item)
                        break
        plan = filtered

    # Collect target directories from the plan
    target_dirs = set()
    for item in plan:
        target_dirs.add(Path(item["dst"]).parent)

    # New directories to create (always safe)
    new_dirs = [
        workspace / "tools" / "convert",
        workspace / "tools" / "align",
        workspace / "tools" / "eval",
        workspace / "tools" / "run",
        workspace / "tools" / "diag",
        workspace / "truth",
        workspace / "truth" / "jc82",
        workspace / "truth" / "jc82" / "fly1",
        workspace / "truth" / "jc82" / "fly2",
        workspace / "truth" / "jc82" / "fly3",
        workspace / "truth" / "jc82" / "fly4",
        workspace / "experiments",
        workspace / "experiments" / "pr20_gps_alt_fusion",
        workspace / "experiments" / "iwt5_vs_iwt2_eval",
        workspace / "experiments" / "stage_b_v1_diagnostics" / "plots",
        workspace / "skills" / "openvins_experiment",
        workspace / "scratch",
        workspace / "archive",
        workspace / "archive" / "pr20_packages",
        workspace / "docs",
    ]
    new_dirs += list(target_dirs)

    print("=" * 70)
    print("OpenVINS Workspace Migration")
    print("  Workspace: {}".format(workspace))
    print("  Mode:      {}".format("APPLY" if apply else "DRY-RUN (no changes)"))
    print("  Date:      {}".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S")))
    if groups:
        print("  Groups:    {}".format(", ".join(groups)))
    print("=" * 70)

    # ── Phase 1: directories ──────────────────────────────────────────────
    print("\n{}\nPhase 1: Create new directories\n{}".format("─" * 70, "─" * 70))
    dirs_to_create = [d for d in new_dirs if not d.exists()]
    if not dirs_to_create:
        print("  All target directories already exist.")
    for d in sorted(set(str(x) for x in dirs_to_create)):
        try:
            rel = Path(d).relative_to(workspace)
        except ValueError:
            rel = Path(d)
        print("  mkdir  {}".format(rel))
        if apply:
            Path(d).mkdir(parents=True, exist_ok=True)

    # ── Phase 2: file operations ──────────────────────────────────────────
    print("\n{}\nPhase 2: File operations ({} planned)\n{}".format(
        "─" * 70, len(plan), "─" * 70))

    operations_log = []
    skipped = []
    warned = []
    scratch_moves = []   # (src_rel, dst_rel) for SCRATCH_INDEX.md
    truth_copies = {}    # dst_dir -> list of entries for SOURCES.yaml

    for item in plan:
        src = Path(item["src"])
        dst = Path(item["dst"])
        reason = item["reason"]
        op_type = item.get("op_type", "move")
        needs_confirm = item.get("confirmation_required", False)

        try:
            src_rel = str(src.relative_to(workspace))
        except ValueError:
            src_rel = str(src)
        try:
            dst_rel = str(dst.relative_to(workspace))
        except ValueError:
            dst_rel = str(dst)

        # Missing source
        if not src.exists():
            if item.get("skip_if_missing", True):
                continue
            skipped.append({"src": src_rel, "reason": "source not found"})
            continue

        # Protected check — only for move/move_with_fixup, not copy
        if op_type in ("move", "move_with_fixup") and is_protected(src, workspace):
            warned.append({"src": src_rel, "reason": "PROTECTED — cannot move"})
            print("  PROT   {} (protected — skipped)".format(src_rel))
            continue

        # Git-tracked check
        if is_git_tracked(src, workspace):
            warned.append({"src": src_rel, "reason": "GIT-TRACKED — use git mv manually"})
            print("  WARN   {} → {}".format(src_rel, dst_rel))
            print("         (git-tracked: run 'git mv {} {}' manually)".format(src_rel, dst_rel))
            continue

        # Destination exists
        if dst.exists():
            warned.append({"src": src_rel, "reason": "destination exists: {}".format(dst_rel)})
            print("  SKIP   {} (destination already exists)".format(src_rel))
            continue

        # Large file warning
        size_mb = file_size_mb(src)
        size_str = format_size(size_mb)
        if size_mb > LARGE_FILE_THRESHOLD_MB:
            print("  LARGE  {} ({})".format(src_rel, size_str))
            warned.append({"src": src_rel, "reason": "large ({})".format(size_str)})

        # Confirmation required
        if needs_confirm:
            print("  SKIP   {} (confirmation required)".format(src_rel))
            print("         Reason: {}".format(reason))
            skipped.append({"src": src_rel, "reason": "confirmation required"})
            continue

        # Print action
        type_label = {"move": "MOVE", "copy": "COPY", "move_with_fixup": "MOVE"}[op_type]
        print("  {}   {}".format(type_label, src_rel))
        print("    →    {}".format(dst_rel))
        if op_type == "copy":
            print("         (original stays in place — Strategy B copy)")

        entry = {
            "op_type": op_type,
            "src": src_rel,
            "dst": dst_rel,
            "reason": reason,
            "size_mb": round(size_mb, 2),
        }

        if apply:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if op_type == "copy":
                if src.is_dir():
                    shutil.copytree(str(src), str(dst))
                else:
                    shutil.copy2(str(src), str(dst))
                # Collect for SOURCES.yaml
                truth_dir_key = str(dst.parent)
                if truth_dir_key not in truth_copies:
                    truth_copies[truth_dir_key] = []
                meta = compute_file_metadata(dst)
                if "gps_raw" in dst.name:
                    file_type = "gps_raw"
                    note = "Raw ArduPilot GPS in WGS84 (Lat,Lng,Alt). TimeUS in microseconds. NOT ENU XYZ."
                elif "gps_aligned_imu_time" in dst.name:
                    file_type = "gps_aligned_imu_time"
                    note = "GPS aligned to IMU time (ENU XYZ). Use with --gps-time-offset 0 if feeding to runner."
                elif "gps_aligned_cam_time" in dst.name:
                    file_type = "gps_aligned_cam_time"
                    note = "GPS aligned to camera time (ENU XYZ). Use with --gps-time-offset 0.0 for fusion."
                elif "pseudo_ref" in dst.name:
                    file_type = "stereo_pseudo_ref"
                    note = "DEBUG ONLY — stereo VIO output, NOT ground truth. Never use as official reference."
                else:
                    file_type = "unknown"
                    note = "Copy from {} → {}".format(src_rel, Path(dst_rel).name)
                truth_copies[truth_dir_key].append({
                    "filename": dst.name,
                    "original_path": src_rel,
                    "file_type": file_type,
                    "note": note,
                    "meta": meta,
                })
            elif op_type == "move_with_fixup":
                shutil.move(str(src), str(dst))
                if needs_import_fixup(dst):
                    apply_import_fixup(dst)
            else:
                shutil.move(str(src), str(dst))

            entry["status"] = "done"
        else:
            # Dry-run: preview fixup
            if op_type == "move_with_fixup" and needs_import_fixup(src):
                print("    +fix  sys.path fixup will be injected (imports _pseudo_ref_guard)")
            entry["status"] = "planned"

        operations_log.append(entry)

        # Track scratch moves for index
        if "scratch/" in dst_rel.replace("\\", "/"):
            scratch_moves.append((src_rel, dst_rel))

    # ── Phase 3: post-processing ──────────────────────────────────────────
    if apply and scratch_moves:
        print("\n{}\nPhase 3: Generate scratch/SCRATCH_INDEX.md\n{}".format("─" * 70, "─" * 70))
        generate_scratch_index(workspace, scratch_moves)

    if apply and truth_copies:
        print("\n{}\nPhase 3b: Write truth/jc82/flyN/SOURCES.yaml\n{}".format("─" * 70, "─" * 70))
        for truth_dir_str, entries in truth_copies.items():
            truth_dir = Path(truth_dir_str)
            flight_id = truth_dir.name  # "fly1", "fly2", etc.
            write_sources_yaml(truth_dir, entries, flight_id)

    # ── Summary ───────────────────────────────────────────────────────────
    moves_done = sum(1 for e in operations_log if e["op_type"] in ("move", "move_with_fixup"))
    copies_done = sum(1 for e in operations_log if e["op_type"] == "copy")
    print("\n{}\nSummary\n{}".format("─" * 70, "─" * 70))
    print("  Moves planned/executed:  {}".format(moves_done))
    print("  Copies planned/executed: {} (Strategy B — originals stay in place)".format(copies_done))
    print("  Skipped (missing):       {}".format(
        sum(1 for s in skipped if "not found" in s["reason"])))
    print("  Skipped (confirmation):  {}".format(
        sum(1 for s in skipped if "confirmation" in s["reason"])))
    print("  Warnings:                {}".format(len(warned)))
    if not apply:
        print("\n  This was a DRY-RUN. Pass --apply to execute.")
        print()
        print("  MANUAL ACTION REQUIRED (not done by this script):")
        print("  - open_vins_pr17/ (3.6 GB): stale repo copy — recommended DELETE.")
        print("    Confirm: git log does NOT depend on it, then: rm -rf open_vins_pr17/")
        print("  - estimator_config_iwt5_candidate.yaml: git-add if adopting as canonical.")

    if apply or operations_log:
        manifest = {
            "date": datetime.now().isoformat(),
            "workspace": str(workspace),
            "apply": apply,
            "operations": operations_log,
            "skipped": skipped,
            "warnings": warned,
        }
        with open(str(manifest_path), "w", encoding="utf-8") as f:
            json.dump(manifest, f, indent=2)
        print("\n  Manifest: {}".format(manifest_path))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Workspace migration tool for open_vins.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--apply", action="store_true",
                        help="Execute the migration (default: dry-run)")
    parser.add_argument("--groups", default=None,
                        help="Comma-separated groups: tools,diag,scratch,docs,archive,experiments,truth")
    parser.add_argument("--workspace", default=None,
                        help="Workspace root (default: parent of this script)")
    parser.add_argument("--manifest", default=None,
                        help="Path for migration manifest JSON")
    args = parser.parse_args()

    if args.workspace:
        workspace = Path(args.workspace).resolve()
    else:
        workspace = Path(__file__).resolve().parent.parent

    if not workspace.exists():
        print("ERROR: workspace does not exist: {}".format(workspace), file=sys.stderr)
        sys.exit(1)

    manifest_path = Path(args.manifest) if args.manifest else workspace / "migration_manifest.json"

    groups = None
    if args.groups:
        groups = [g.strip() for g in args.groups.split(",")]
        valid = {"tools", "diag", "scratch", "docs", "archive", "experiments", "truth"}
        unknown = set(groups) - valid
        if unknown:
            print("ERROR: unknown groups: {}. Valid: {}".format(unknown, valid), file=sys.stderr)
            sys.exit(1)

    run_migration(workspace, apply=args.apply, groups=groups, manifest_path=manifest_path)


if __name__ == "__main__":
    main()
