# Move Summary

Created on 2026-06-25 as the staging area for old experiment material.  The
large files are stored outside the git repository at:

```text
D:/vscode_dir/open_vins_legacy_quarantine_20260625
```

## Current Contents

| Area | Files | Size | Reason |
| --- | ---: | ---: | --- |
| `root_run_scripts` | 62 | tiny | Old one-off flight runners replaced by `baseline/latest/scripts`. |
| `analysis_oneoffs` | 67 | tiny | Historical diagnostics and temporary analysis scripts. |
| `analysis_run_specs` | 109 | tiny | Old JSON run specs for obsolete sweeps. |
| `old_artifacts` | 8955 | 8.818 GB | Historical generated artifacts and dashboards. |
| `old_builds` | 387 | 5.291 GB | Low-pass/notch and other dated build trees. |
| `old_outputs` | 301 | 3.229 GB | Historical root-level generated results. |
| `old_config_packs` | 3 | tiny | Superseded clean-config package. |
| `old_analysis_snapshots` | 214 | tiny | Old copied analysis-tool snapshots. |
| `handoff_packages` | 128 | 0.021 GB | Old handoff bundles and generated patches. |
| `stray_root_files` | 5 files + 1 empty dir | tiny | Root-level reports/zips/noise files, plus the space-named directory that was causing `git status` warnings. |

## Deletion Rule

Delete this folder only after:

1. `baseline/latest/scripts/run_fly1_stride12.sh` runs successfully.
2. `baseline/latest/scripts/run_fly2_stride12.sh` runs successfully.
3. `baseline/latest/scripts/run_fly3_stride12.sh` runs successfully.
4. `baseline/latest/scripts/run_fly4_stride12.sh` runs successfully.
5. `baseline/latest/scripts/analyze_latest_run.sh` produces the expected analysis package for all four runs.

Until then, this folder is recovery material, not the current baseline.
