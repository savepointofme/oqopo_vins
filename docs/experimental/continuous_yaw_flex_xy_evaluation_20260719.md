# Continuous yaw-flex XY evaluation (experimental)

Status: **SHADOW_FAILED — do not integrate into formal ROS-free navigation**.

## Evaluation contract

- Data: frozen desktop `fly1–fly4_oc_stride12` baseline at `f8664c8`.
- Windows: fly1 `934.2–1644.8 s`, fly2 `704.2–846.8 s`, fly3
  `622.2–1406.2 s`, fly4 `928.6–2436.2 s`.
- GPS-time sampling and start-only alignment use
  `analysis/full_flight_error_analysis.py` without trajectory best fitting.
- Baseline, scalar-yaw flex, and body-axis SO(3) flex use identical windows.
- The experimental XY trajectory preserves the source trajectory and rotates
  each post-init VIO XY increment by the negative causal flex output. This is
  an offline navigation-effect experiment, not estimator-state feedback.

## Global result

| condition | distance-weighted XY RMSE | change vs baseline | distance-weighted final XY error | change vs baseline |
|---|---:|---:|---:|---:|
| frozen stride-12 baseline | 173.599 m | — | 96.072 m | — |
| current scalar-yaw flex | 131.248 m | **24.40% better** | 116.325 m | **21.08% worse** |
| body-axis SO(3) flex | 180.051 m | **3.72% worse** | 106.756 m | **11.12% worse** |

The current scalar observer reduces the distance-weighted full-flight RMSE,
but it does not provide a uniform four-flight improvement. The SO(3)
measurement does not improve the four-flight aggregate.

## Per-flight XY RMSE

| flight | baseline | current scalar | scalar change | body-axis SO(3) | SO(3) change |
|---|---:|---:|---:|---:|---:|
| fly1 | 158.177 m | 163.587 m | 3.42% worse | 267.557 m | 69.15% worse |
| fly2 | 23.979 m | 26.252 m | 9.48% worse | 54.045 m | 125.39% worse |
| fly3 | 201.609 m | 186.509 m | 7.49% better | 191.424 m | 5.05% better |
| fly4 | 181.593 m | 99.108 m | 45.42% better | 146.464 m | 19.34% better |

## Per-flight final XY error

| flight | baseline | current scalar | body-axis SO(3) |
|---|---:|---:|---:|
| fly1 | 165.872 m | 6.826 m | 60.587 m |
| fly2 | 22.847 m | 30.073 m | 136.373 m |
| fly3 | 95.984 m | 189.165 m | 140.484 m |
| fly4 | 71.150 m | 139.061 m | 108.272 m |

## Verdict

Neither method passes a cross-flight navigation criterion. The scalar method
has a favorable aggregate RMSE because fly4 improves strongly, while fly1 and
fly2 regress and three of four final errors worsen. The SO(3) observer improves
fly3/fly4 RMSE but severely regresses fly1/fly2. Both accumulated observers can
reach the common 6-degree output cap, showing that long-term VIO–FC residual is
still being treated as flex.

Authoritative comparison output:
`C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/continuous_yaw_flex_rosfree_stride12_20260719_161121/xy_flex_three_condition_comparison_20260719_v2`.

## Follow-up optimization

Status: **FLEX_XY_OPTIMIZATION_FAILED**.

### Two-timescale SO(3) observer

The slow path absorbs the complete long-term VIO–FC level without a Huber
clip. The fast path retains the original 12 s time constant, 1.5-degree Huber
innovation, 0.25 deg/s rate limit, and 6-degree absolute limit. A common slow
time constant was swept over `45/60/90/120/180/240 s`.

- Maximin selection chose `45 s`.
- Distance-weighted XY RMSE changed from `173.599 m` to `176.747 m`:
  **1.81% worse**.
- Worst flight was **48.26% worse**.
- Only `1/4` flights improved and LOFO passed only `1/4` holdouts.

Result directory:
`C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/continuous_yaw_flex_rosfree_stride12_20260719_161121/two_timescale_sweep_v2_20260719/selection`.

### Common positive correction gain

The original scalar observer was held fixed and only one common physical gain
was swept over `0.1/0.2/0.3/0.4/0.5/0.65/0.8`.

- Maximin selection chose `gain=0.1`.
- Aggregate XY RMSE improved **4.80%**.
- fly1/fly3/fly4 improved, but fly2 regressed **0.57%**.
- LOFO passed `3/4`; therefore the common-gain shadow failed.
- Larger gains improve the aggregate more strongly but monotonically worsen
  fly2. At gain `0.8`, aggregate improvement is `25.63%` while fly2 regresses
  `6.95%`.

Result directory:
`C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/continuous_yaw_flex_rosfree_stride12_20260719_161121/scalar_gain_sweep_20260719/selection`.

### Sign falsification

Opposite gains `-0.1/-0.2/-0.4/-0.8` were evaluated with the identical GPS
contract. The maximin result was `-0.1`: aggregate XY RMSE regressed **5.06%**,
the worst flight regressed **8.35%**, and only `1/4` flights improved. The XY
correction sign is not the cause of the cross-flight failure.

### Gyro–posterior agreement gate

Saved gyro bias was removed and D455 gyro was integrated over the same 5 Hz FC
intervals as the VIO posterior. fly2 has the strongest posterior/gyro agreement
of all four flights:

- increment correlation: `0.861`;
- final posterior cumulative residual: `28.58 deg`;
- final gyro cumulative residual: `26.87 deg`.

Nevertheless every positive XY correction gain worsens fly2. A gyro–posterior
agreement gate would therefore accept, rather than suppress, the harmful fly2
correction. It cannot make this model pass.

Result directory:
`C:/Users/baloney/Desktop/实验目录/P4_formal_joint_20260718/continuous_yaw_flex_rosfree_stride12_20260719_161121/gyro_vio_flex_agreement_20260719_v3`.

### Final technical conclusion

Body–D455 yaw flex changes the mapping between the D455 attitude and aircraft
body attitude. It does not justify rotating already world-referenced VIO XY
translation increments. Apart from a measured body/D455 translational lever
arm, the sensor trajectory position is unchanged by this relative yaw. The
gain, sign, two-timescale, and gyro-gate experiments all reject direct
flex-to-XY correction as a cross-flight navigation model.

The flex observer must remain output-only. A future navigation experiment must
instead formulate FC-minus-flex as an explicit relative-yaw measurement with
innovation covariance and consistent state/covariance feedback; it cannot be
approximated by rotating the completed XY trajectory.
