#!/bin/bash
BASE="/mnt/d/vscode_dir/open_vins"
PKG="$BASE/pr20_final_package"
> "$PKG/gps_diagnostics.txt"

logs=(
  "20260509_fly1/result/pr20_robust_gps_fly1.log"
  "20260509_fly1/result/pr20_fixed_abs_fly1.log"
  "20260509_fly1/result/pr20_fixed_rel_fly1.log"
  "20260509_fly3/result/pr20_robust_gps_rel_fly3.log"
  "20260509_fly3/result/pr20_fixed_rel_fly3.log"
  "20260509_fly3/result/pr20_fixed_abs_fly3.log"
  "20260509_fly4/result/pr20_robust_gps_fly4.log"
  "20260509_fly4/result/pr20_fixed_abs_fly4.log"
  "20260509_fly4/result/pr20_fixed_rel_fly4.log"
)

for log in "${logs[@]}"; do
  echo "=== $log ===" >> "$PKG/gps_diagnostics.txt"
  grep "GPS-ALT-FINAL" "$BASE/$log" >> "$PKG/gps_diagnostics.txt" 2>/dev/null
  echo "" >> "$PKG/gps_diagnostics.txt"
done
echo "GPS diagnostics extracted"
wc -l "$PKG/gps_diagnostics.txt"
