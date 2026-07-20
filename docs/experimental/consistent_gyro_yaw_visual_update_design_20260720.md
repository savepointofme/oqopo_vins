# Consistent gyro-yaw visual update protection

Status: four-flight experimental candidate passed the frozen stride-12 XY
comparison. The mode remains explicit/default-off and is not a P4/P5 release.

## Failure evidence

On frozen stride-12 fly1 (`934.2–1644.8 s`), accepted MSCKF updates inject
`+5.247 deg` cumulative yaw while VIO-minus-FC SO(3) yaw changes from
`-1.04 deg` to `-4.91 deg`. Their correlation is `-0.890`. Open-loop D455 gyro
using the saved `bg(t)` is closer to FC than the VIO posterior (yaw median/P95
absolute error `0.94/2.38 deg` versus `2.72/5.26 deg`).

After removing the initialization-time nominal mount, fly1 still carries an
approximately constant `-2 deg` roll and `-1 deg` pitch FC-minus-D455 residual,
with short larger transients. This is not explained by gyro bias and is not
treated as an online roll/pitch correction here: it may be a post-anchor mount
level, flex, or FC attitude-convention residual. The yaw selector uses an SO(3)
projection onto the frozen body yaw axis, and the state gain is scaled along
the current IMU local-gravity direction, so the constant cross-axis residual is
not integrated or reinterpreted as yaw flex.

The current `global_yaw_oc_projection` preserves the joint global-yaw gauge
direction in the measurement Jacobian. It does not prevent the current IMU or
clone attitudes from receiving equal-and-opposite yaw corrections inside that
joint gauge. The historical `hard_gyro_yaw` mode zeroes yaw only after the
standard covariance update, so its nominal-state and covariance semantics do
not match.

## Failed hard-removal isolation

Two gain-masking isolations were run and retained. Masking current and clone
yaw destroyed visual relative-rotation observability and diverged before the
fly1 evaluation end. Masking only current yaw while retaining clone yaw avoided
the immediate structural failure, but still accumulated a large XY divergence
by about 1591 s. Therefore visual yaw feedback is required; the repair cannot
be a binary off switch.

The historical update audit also falsifies a spike-only explanation. The
accepted fly1 MSCKF yaw steps sum to 5.247 deg. A 0.05 deg/update clip still
leaves 4.442 deg, so most of the error is many small same-direction updates.

## Uniform visual-information ablation

Reducing visual information globally was tested as an alternative to the
directional guard on the complete frozen fly1 window. Each run changed exactly
one configuration concept; no guard was enabled.

| condition | XY RMSE [m] | final XY [m] | yaw/course RMSE [deg] |
|---|---:|---:|---:|
| frozen baseline: 400 points, noise 1.0x | 158.177 | 165.872 | 1.524 |
| directional guard | **155.243** | **146.935** | **1.465** |
| 350 points | 180.775 | 220.102 | 1.513 |
| 300 points | 184.338 | 245.764 | 2.030 |
| 200 points | 183.732 | 205.171 | 1.645 |
| visual pixel sigma 1.10x | 163.222 | 175.210 | 1.662 |
| visual pixel sigma 1.25x | 166.520 | 151.856 | 1.479 |
| visual pixel sigma 1.50x | 178.777 | 168.794 | 1.619 |

All six uniform reductions fail the fly1 XY gate, so they were not promoted to
four-flight replay. Fewer points change the spatial/track subset and remove
necessary translation and relative-rotation information. Uniformly larger
measurement noise also weakens useful visual constraints; `1.25x` slightly
improves yaw RMSE but still worsens XY RMSE by `5.27%`. This supports selective
directional attenuation rather than a global feature-count or confidence
reduction.

## Guarded bounded-feedback candidate

Keep the frozen global-yaw OC measurement model and all clone-relative
rotation information. At each processed camera interval:

1. Form an FC relative SO(3) increment. Absolute FC attitude is used only once
   with D455 attitude to freeze the nominal mount.
2. Integrate D455 gyro minus the current saved bias on the identical interval.
3. Form initialization-anchored relative VIO-minus-FC yaw, then apply a causal
   `tau=5 s` first-order filter with a `1 deg` clipped innovation. A single FC
   or VIO spike cannot move this decision signal quickly.
4. Require `|filtered error| >= 1 deg` with a consistent sign for at least
   `10 s`, full gyro-minus-FC SO(3) disagreement below `0.35 deg`, and a visual
   yaw update whose direction would increase the existing relative error.
5. When all conditions pass, scale only the current-IMU gravity-axis visual
   gain to `0.95`. Otherwise select the untouched baseline gain.
6. Keep clone orientation, roll/pitch, P/V, and bias gains unchanged. Apply the
   selected arbitrary gain to both mean and covariance using

   `P+ = P - K M^T - M K^T + K S K^T`,

   where `M=P H^T` and `S=H P H^T+R` are formed from the same projected visual
   model.

FC invalidity or a time gap selects the exact baseline gain. GPS P/V and truth
never enter the selector. A diagnostic CSV records FC/gyro disagreement and the
selected gain at every processed frame.

## Spike policy and diagnostic correction

No low-pass is applied to IMU propagation or the formal VIO attitude. The gate
low-passes only its independent relative-yaw decision signal. Raw attitude and
gyro bandwidth remain unchanged.

The first dwell plotting script also admitted invalid visual diagnostic rows,
whose default `0 deg` field created artificial downward spikes at duplicate
timestamps. The reusable audit now requires
`dx_yaw_projection_valid == 1` and overlays the raw guard observation with the
actual causal filtered state. Across fly1--fly4, raw absolute-step P95 is
`0.40--0.62 deg`; filtered absolute-step P95 is about `0.077 deg`. The largest
raw fly4 spike is `15.26 deg`, while the largest filtered single step is
`0.59 deg`, and the dwell requirement still prevents a one-frame action.

## Four-flight result

All metrics use GPS timestamps, the frozen per-flight valid windows, start-only
heading alignment, and no scale or full-trajectory fit.

| flight | stride-12 XY RMSE [m] | guarded XY RMSE [m] | change | stride-12 final XY [m] | guarded final XY [m] |
|---|---:|---:|---:|---:|---:|
| fly1 | 158.177 | 155.243 | -1.86% | 165.872 | 146.935 |
| fly2 | 23.979 | 23.979 | 0.00% | 22.847 | 22.847 |
| fly3 | 201.609 | 193.767 | -3.89% | 95.984 | 88.253 |
| fly4 | 181.593 | 176.649 | -2.72% | 71.150 | 31.843 |
| distance weighted | 173.599 | 168.638 | -2.86% | 96.072 | 70.888 |

The distance-weighted speed RMSE also changes from `1.233` to `1.146 m/s`, and
course/yaw RMSE from `3.170` to `3.055 deg`. Fly2 has no guarded update and is
identical to its frozen baseline output.

For fly3, this guard does not claim to repair the first `760--800 s` event:
the filtered relative-yaw reference moves from `-0.610` to `-0.134 deg`, below
the threshold, so there are zero attenuated updates in that window. In
`995--1025 s`, it stays at `+1.50--1.71 deg`; 101 harmful-direction visual
updates are attenuated. The global XY improvement is therefore evidence for
bounded visual-yaw feedback, not proof that every historical flex event has
been corrected.

## Acceptance

- Unit test: selected gain scaling changes only the current local-gravity yaw
  component and leaves clone, roll/pitch, P/V, and bias rows unchanged.
- Existing Joseph PSD/symmetry tests pass.
- fly1: lower VIO-minus-FC yaw median/P95 and lower standard XY RMSE than the
  exact frozen stride-12 baseline on the same time window.
- fly1–fly4: one common mode, no flight-specific threshold, no numerical
  failure, and distance-weighted XY RMSE improves without per-flight XY-RMSE
  regression.
- Default baseline behavior remains unchanged unless the explicit experimental
  yaw mode is selected.
