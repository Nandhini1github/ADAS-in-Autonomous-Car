# Comparison methodology

The checked-in results/model measurements come from the deterministic longitudinal
model. They exercise the actual Python controllers with identical conditions.
They are not CARLA, ROS timing, or physical vehicle measurements.

Run `python3 scripts/run_model_benchmark.py --allow-stress-failures` to reproduce
all six pairs, including the two overlap failures. Each case includes CSV,
metadata, Markdown, JSON, PNG, and SVG outputs. Use `--no-plots` without matplotlib.

## Validity and provenance

A CSV sidecar records source, controller (pid/mpc), shared settings, source hash,
and CSV hash. Settings and sources must match; the CLI verifies CSV hashes.
Undocumented files require `--allow-unverified`, which produces diagnostics,
suppresses comparison deltas, and exits with code 2. Missing or nonfinite fields
(other than positive infinite TTC), malformed flags, empty files, and
nonincreasing timestamps are rejected.

A valid pair needs actual AEB activation, AEB phase, COMPLETE terminal phase,
positive gap, a latched AEB signal, FCW by activation, and no speed rebound above
the stop threshold. Aborted and incomplete runs receive no comparison deltas.

Every observation through activation must match: source time, ego/lead speed,
gap, closing speed, TTC, FCW/AEB flags, steering, acceleration, and phase.
The absolute tolerance defaults to 0.001 in each field's logged unit.
The activation brake command is excluded because it is the compared output.
Metadata is still necessary: matching traces alone cannot identify experiment settings.

## Measurements

| Metric | Definition |
|---|---|
| Events | First FCW, THREAT, AEB, and post-AEB stop-threshold sample |
| Activation | Speed and gap at first AEB |
| Minimum gap | Smallest value over the full run; gap ≤ 0 is an overlap proxy |
| Minimum finite TTC | Smallest finite value over the full run; nonclosing infinite TTC is excluded |
| Stop time | First sampled speed ≤ 1 mph after AEB minus activation time |
| Stop distance | Trapezoidal integral of ego speed from AEB to that threshold, using actual time intervals |
| Peak deceleration | Largest negative speed finite difference, beginning with the interval ending at activation |
| Peak absolute jerk | Difference between successive finite-difference accelerations divided by interval-midpoint separation |
| Runtime | Unavailable unless each CSV sample includes controller_runtime_ms; reports mean/max |
| Contact | Unavailable in model CSV; CARLA requires collision sensor evidence in metadata |

Speeds convert using 1 mph = 0.44704 m/s. Stop metrics describe the configured
sampled threshold, not exact standstill. Acceleration and jerk are unsmoothed and
depend on sampling and quantization. Plot acceleration is the logged signal;
table deceleration/jerk are speed finite differences. These can differ.

JSON uses finite numbers or null for unavailable metrics, never NaN/Infinity.
Dotted plot lines locate activation. Positive infinite TTC is omitted from curves.

There is no overall winner. Shorter stops can trade against braking severity or
jerk; gap alone does not establish safety. Signed MPC-minus-PID deltas appear
only for valid pairs. Deterministic scenarios are not independent statistical
trials and do not support confidence intervals or real-world safety claims.
