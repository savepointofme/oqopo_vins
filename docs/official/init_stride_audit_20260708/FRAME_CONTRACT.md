# OpenVINS Frame And Time Contract

Contract version: `openvins-frame-v2`  
Date: 2026-07-10  
Status: normative for new P0/P1 artifacts; existing runs are non-conforming until
their metadata and invariant checks pass.

This file is the single naming and transform contract for the ROS-free flight
baseline. Older `FRAME_CONVENTION.md` and `FRAME_TREE.md` files are audit history,
not alternate definitions.

## 1. Notation

`R_B_A` maps vector coordinates from frame `A` to frame `B`:

```text
v_B = R_B_A v_A
```

`p_AinB` is the position of frame-`A` origin expressed in frame `B`. A rigid
mapping from `A` coordinates to `B` coordinates is:

```text
p_B = R_B_A p_A + p_AinB
```

All persisted matrices use the destination/source order above. Code identifiers
that use legacy `R_AtoB` names mean the same coordinate mapping, but metadata and
new analysis code must use `R_B_A`.

Angles are radians in estimator code and degrees only in explicitly suffixed
diagnostic columns (`*_deg`). Distances are metres, velocity is metres/second,
angular rate is radians/second, and timestamps are seconds.

## 2. Frames

| Frame | Definition | May enter estimator after initialization? |
| --- | --- | --- |
| `W0` | OpenVINS local world fixed when the EKF seed is applied. This is earlier than the first output-ready/emitted trajectory row. Internal code historically calls it `G`. | Yes; this is the estimator state frame. |
| `G_src` | Source GPS local ENU produced by `DatasetReaderEuroc`, anchored at the first valid source GPS sample. | Z only in the registered guarded GPS-Z baseline; XY/course are reference-only. |
| `G_nav` | Causal ENU navigation frame connected to `W0` once at initialization by `T_Gnav_W0`. | The mapping may be logged/displayed; mapped XY must not be fed back into the current baseline. |
| `G_eval` | Offline evaluation frame produced by the registered start-heading alignment. | No. It may use later samples and is evaluation-only. |
| `I` | OpenVINS IMU/body state frame. The nominal state is IMU-centred. | Yes. |
| `C0`, `C1` | Camera optical frames. | Yes, through calibrated visual measurement models. |
| `FC_raw` | Flight-controller vehicle body frame before the declared nominal axis conversion; current source contract is FRD. | No direct use. It must first be converted to the declared `I` convention. |
| `B` | Reserved aircraft/body convention used only when an explicit calibrated `R_I_B` exists. | No implicit identity with `I` or `FC_raw`. |
| `A_gnss` | GNSS antenna phase centre. | Position lever arm is allowed only through the declared initialization mapping. |

`W0`, `G_src`, `G_nav`, and `G_eval` are never synonyms even when a particular
run produces an identity transform between two of them.

## 3. State And Quaternion Direction

The internal IMU state is:

```text
[q_W0toI, p_IinW0, v_IinW0, bg_I, ba_I]
```

`q_W0toI` is an OpenVINS JPL quaternion stored as `[qx,qy,qz,qw]`; its matrix is
`R_I_W0`. `_imu->Rot()` returns `R_I_W0`.

The TUM raw output is deliberately different: the runner transposes the internal
matrix and serializes an Eigen/Hamilton quaternion `[qx,qy,qz,qw]` for
`R_W0_I`, the orientation of `I` in `W0`.

The FC initialization row stores orientation, velocity, position, and biases,
but position semantics are declared separately and may never be inferred from
whether the numeric value happens to be zero:

```text
t, q_source_to_I(JPL x y z w), v_IinSource, p_IinSource, bg_I, ba_I
```

Every `--init-from-fc` command must pass exactly one explicit
`--init-from-fc-position-frame` value:

- `local_w0_seed`: `p` is an already-local seed placeholder and must be zero;
- `global_gnav`: `p` is the IMU global navigation position, may be non-zero,
  is retained in `T_Gnav_W0`/origin metadata, and is never copied into the EKF.

For both modes the EKF receives `p_I_seed_in_W0=[0,0,0]`. At the seed,

```text
p_W0inGnav = p_I_seed_in_Gnav - R_Gnav_W0 p_I_seed_in_W0
```

so a `global_gnav` input is recovered exactly by the persisted nav transform.
The metadata field `fc_global_position_estimator_input` is always false.

The registered fly1--fly4 artifacts all contain `p=[0,0,0]` and are invoked as
`local_w0_seed`. New converter output also records
`# position_frame=local_w0_seed`; if an artifact declaration and command-line
declaration disagree, loading fails closed.

FC source Euler input is NED yaw-pitch-roll with vehicle body FRD. The current
converter uses:

```text
R_B_NED = Rz(yaw_sign * yaw) Ry(pitch) Rx(roll)
R_I_Gnav = R_I_FCraw R_FCraw_Gnav
```

This is the ZYX attitude convention; reported `[roll,pitch,yaw]` diagnostics are
the corresponding XYZ component tuple in degrees. Euler values are never used to
compose repeated transforms when a quaternion/matrix is available.

## 4. Camera/IMU Extrinsic

The accepted physical definitions are:

```text
T_cam_imu: p_C = R_C_I p_I + p_IinC
T_imu_cam: p_I = R_I_C p_C + p_CinI = inverse(T_cam_imu)
```

The runtime state stores `(R_C_I, p_IinC)` and documents it as
`(R_ItoC, p_IinC)`.

An input containing `T_imu_cam` must be inverted once before storage. An input
containing `T_cam_imu` must be stored directly. The current generic YAML parser
does preserve this rule: when it falls back to the opposite key it applies
`Inv_se3` (`opencv_yaml_parse.h:661-665`), after which `VioManagerOptions` performs
the expected conversion into runtime storage. The earlier audit missed those
lines. A numeric equivalence test for both key representations is still required
as a P1 regression gate, but the fallback itself is not an open direction bug.

## 5. GNSS Antenna Lever Arm

`gps_antenna_in_imu_m` is:

```text
p_Agnss_inI
```

At initialization:

```text
p_I0inGnav = p_Agnss0inGsrc - R_Gnav_I0 p_Agnss_inI
```

The value, source, version, and units must be recorded. Zero is a declared value,
not an absent value. A moving-antenna velocity conversion must include
`R_Gnav_I (omega_I x p_Agnss_inI)` when antenna velocity, rather than IMU-origin
velocity, is used. The current one-shot mapping does not claim that correction.

## 6. Causal `W0 -> G_nav` Mapping

At the EKF seed camera timestamp `t_seed`, the runner chooses only the latest GPS
sample satisfying `t_gps <= t_seed` and records its age. With:

```text
R_W0_I_seed   = orientation from the seeded VIO state
R_Gnav_I_seed = orientation from the FC initialization artifact
```

the fixed mapping is:

```text
R_Gnav_W0 = R_Gnav_I_seed R_W0_I_seed^T
p_W0inGnav = p_I_seed_in_Gnav - R_Gnav_W0 p_I_seed_in_W0
```

For every raw sample:

```text
p_IinGnav = R_Gnav_W0 p_IinW0 + p_W0inGnav
v_IinGnav = R_Gnav_W0 v_IinW0
R_Gnav_I  = R_Gnav_W0 R_W0_I
```

Repository metadata currently calls `p_W0inGnav` `p_Gnav_W0`; this legacy JSON
key is retained for compatibility but its formula and semantics are fixed by this
contract. A future metadata version must use the unambiguous name.

`T_Gnav_W0` is computed once, uses no future samples, and must never be refit from
the completed trajectory.

## 7. `G_eval` Alignment

The canonical evaluation samples on GPS update time and applies one registered
start-heading rotation plus translation. The outputs are named
`vio_eval_aligned_*`, carry `alignment_id`/transform metadata, and live in
`G_eval`. Dashboard fits over later trajectory samples are a distinct
`vio_dashboard_aligned_*` evaluation/display layer and must record
`uses_future_data=true`.

No value may undergo more than one transform of the same layer. In particular:

- raw estimator output is never overwritten by nav/evaluation values;
- nav output is never fed into start-heading code as if it were raw without an
  explicit registered source-frame conversion;
- dashboard alignment is not reused as official metric alignment;
- any GPS-derived value is named `reference_*`, not `truth_*`, unless an actual
  independently validated ground-truth system supplied it.

## 8. Time Contract

The master axis is camera/VIO state time.

```text
t_imu = t_cam + calib_dt_CAMtoIMU
```

The startup timeline is:

```text
t_trim = dataset_first_imu_timestamp + requested_start_offset
t_seed = first retained camera timestamp
t_emit = first output-ready/emitted trajectory timestamp
t_trim <= t_seed <= t_emit
```

The first retained camera applies the FC seed and is skipped as a visual update;
therefore `traj_raw.txt` starts at `t_emit`, not `t_seed`. The first trajectory
position must never be used as the seed position or to reconstruct
`T_Gnav_W0`.

An already camera-aligned GPS CSV uses `--gps-time-offset 0`. FC initialization
selection targets `t_seed`. A conforming exact-time
initializer records the two bracketing FC timestamps, interpolation fraction,
maximum endpoint age, interpolation/propagation method, and any extrapolation.
Nearest-row selection remains `I0` only and must record signed `source_dt`.

All time series must be strictly monotonic after an explicitly documented
duplicate policy. Out-of-range, duplicate, or non-monotonic input may not be
silently accepted.

## 9. Required Output Layers

| Artifact/columns | Frame | Transform policy |
| --- | --- | --- |
| `traj_raw.txt`, `vio_raw_*` | `W0` | None. |
| `traj_nav.txt`, `vio_nav_*` | `G_nav` | Fixed causal `T_Gnav_W0`, once. |
| `vio_dashboard_aligned_*` | display/evaluation layer | Registered dashboard transform; future-data flag required. |
| `vio_eval_aligned_*` | `G_eval` | Registered canonical evaluation transform, once. |
| `reference_position_*`, `reference_velocity_*`, `reference_course_deg` | source reference frame declared per run | Never estimator state. |

Every new run records `frame_contract_version=openvins-frame-v2`, serializes
metadata as `nav-frame-v3`, and is checked by
`openvins-frame-validation-v4`. Historical v1 receipts remain immutable and
are not silently promoted.

## 10. Conformance Gates

A P1 run passes only if an automated validator confirms:

1. raw/nav timestamps match exactly for emitted rows;
2. recomputed nav position, velocity, and rotation agree with `traj_nav.txt` at
   the declared numeric tolerance;
3. every quaternion is finite and unit norm within tolerance;
4. all persisted timestamps are monotonic and FC/GPS sample ages are present;
5. metadata says whether future data were used and whether a layer is
   estimator-input or evaluation-only;
6. schema documentation and physical Parquet/CSV columns match the live schema;
7. the camera/IMU fallback regression test passes for both `T_cam_imu` and
   `T_imu_cam` representations of the same transform;
8. trim, seed, and first-emitted timestamps are ordered and internally
   consistent;
9. seed position and first-emitted position remain distinct, and metadata's
   first-emitted values match the first raw trajectory row exactly.
10. FC position-frame semantics are explicit and valid;
11. a non-zero `global_gnav` position is recovered by `T_Gnav_W0` while raw
    seed position remains zero;
12. a non-zero `local_w0_seed` and any artifact/CLI frame mismatch fail closed.

Until these gates pass on a newly produced representative run, raw/nav output is
an implementation candidate and not accepted evidence of frame closure.
