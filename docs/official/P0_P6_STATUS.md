# OpenVINS P0–P6 status (2026-07-14)

| Project | Status | Current evidence and boundary |
| --- | --- | --- |
| P0 | `COMPLETED_FROZEN` | Historical lineage audit and the existing 32 fixed-stride run index remain frozen; this task did not expand them. |
| P1 | `RUNTIME_CONTRACT_IMPLEMENTED` | P4 output is directly in `G_nav`; formal evaluation uses `absolute_navigation_no_post_alignment`. GPS XY/course remain reference-only for lateral navigation. |
| P2 | `PROVISIONAL_SHADOW_FROZEN_READ_ONLY_EVIDENCE_REUSED` | The existing four-flight × eight-stride parquet was reused with leave-one-flight-out folds. No estimator sweep was rerun. |
| P3 | `INCONCLUSIVE_DIAGNOSTIC` | No approximately 7-degree, 4.089-degree, permanent FC-to-board, or time-varying flex correction is used. Startup misalignment is a launch state, not permanent calibration. |
| P4 | `PERSISTENT_SLIDING_WINDOW_IMPLEMENTED_BUT_VALIDATION_FAILED` | FC+board-IMU+monocular factors, bounded 2 s reference/3/5/8/12 s windows, persistent retry, fingerprinted solves, 2 s candidate holdout, one release, and zero post-release solves are implemented and fly1/fly3 execute successfully. Four-flight progression is held because absolute navigation non-inferiority and real >20/>60 s late-start evidence are not established. |
| P5 | `TARGET_PARALLAX_INFORMATION_TRIGGER_IMPLEMENTED_BUT_VALIDATION_FAILED` | Shared target-parallax tracking, polygon-overlap/rotation/visual caps, information-triggered backend, and tracking-only lifecycle run through both full flights with zero clone violations. Dashboard-disabled throughput passes; visible Dashboard throughput fails. Required pixel/candidate thresholds are not identifiable from the legacy 32 runs. |
| P6 | `UNCHANGED_NOT_IN_SCOPE` | No repair, restart, anchor reset, or unrelated master-table expansion was enabled. |

## Current disposition

`P4_P5_REDESIGN_IMPLEMENTED_BUT_VALIDATION_FAILED`

The implementation is not blocked: registered focused tests pass and fly1/fly3 both complete. The current evidence does not justify four-flight acceptance because navigation non-inferiority, real late-release coverage, complete threshold provenance, and visible-path realtime are not all satisfied.
