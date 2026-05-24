"""Tiny guard shared by diag_*.py scripts that read truth_asl_*.csv.

`truth_asl_cam_time.csv` is a STEREO VIO trajectory and is NOT real ground
truth. Any script that reads it must require an explicit acknowledgement
flag at runtime, to prevent silent reuse of stereo pseudo-truth in official
reporting. Usage:

    from tools._pseudo_ref_guard import require_pseudo_ref_ack
    require_pseudo_ref_ack(__file__)

Pass `--ack-stereo-pseudo-ref` on the command line to acknowledge.
"""
import sys


def require_pseudo_ref_ack(script_name: str) -> None:
    ack_flag = "--ack-stereo-pseudo-ref"
    if ack_flag not in sys.argv:
        print("=" * 72, file=sys.stderr)
        print(f" REFUSED TO RUN  {script_name}", file=sys.stderr)
        print(" This script reads truth_asl_*.csv (= STEREO VIO trajectory,", file=sys.stderr)
        print(" NOT ground truth). Per the 2026-05-17 data cleanup policy", file=sys.stderr)
        print(" (see GPS_REFERENCE_AUDIT.md), any use of stereo pseudo-ref", file=sys.stderr)
        print(" must be explicit. Re-run with:", file=sys.stderr)
        print(f"   {ack_flag}", file=sys.stderr)
        print(" Output is debug-only and must not be cited as 'truth RMSE'.", file=sys.stderr)
        print("=" * 72, file=sys.stderr)
        sys.exit(2)
    # Acknowledged — emit a softer banner on stdout so it shows in any logs.
    print("=" * 72)
    print(f" [stereo_pseudo_ref] {script_name}: using truth_asl_*.csv as")
    print(" debug pseudo-reference (NOT ground truth). XY/Z metrics here")
    print(" are NOT official; do not cite as 'truth RMSE'.")
    print("=" * 72)
    # Remove the ack flag so downstream argparse in the script does not see it.
    try:
        sys.argv.remove(ack_flag)
    except ValueError:
        pass
