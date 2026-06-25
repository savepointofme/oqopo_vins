#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for fly in fly1 fly2 fly3 fly4; do
  "$SCRIPT_DIR/run_baseline_fly.sh" --fly "$fly" --stride 12 "$@"
done
