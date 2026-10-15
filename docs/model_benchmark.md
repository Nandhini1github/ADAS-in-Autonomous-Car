# Longitudinal model benchmark

`scripts/run_model_benchmark.py` runs the same production Python AEB PID, MPC,
safety supervisor, and scenario state machine against a deterministic 1-D plant.
The generated files live under `results/model/`; the original CARLA result paths
are reserved for CARLA. No measurements are invented or copied between backends.

## Experimental conditions

| Case | Ego / lead (mph) | Initial bumper gap (m) | Grip multiplier | Ego command delay (s) |
|---|---:|---:|---:|---:|
| urban_15_10 | 15 / 10 | 80 | 1.00 | 0.10 |
| urban_25_15 | 25 / 15 | 80 | 1.00 | 0.10 |
| arterial_35_20 | 35 / 20 | 80 | 1.00 | 0.10 |
| highway_50_25 | 50 / 25 | 120 | 1.00 | 0.10 |
| reduced_grip_35_20 | 35 / 20 | 80 | 0.55 | 0.10 |
| delayed_brake_50_25 | 50 / 25 | 120 | 1.00 | 0.50 |

The time step is 0.05 s with the checked-in configuration. Vehicles start at their
requested steady speeds, enter HOLD, then the lead vehicle brakes after the shared
3 s stable interval. This isolates emergency response from launch acceleration.
The same initial conditions, supervisor, and lead trajectory apply to both controllers.

Ego brake demand is delayed by `ceil(delay / dt)` samples and passed through a
first-order actuator with a 0.15 s time constant using
`alpha = 1 - exp(-dt / tau)`. Effective ego deceleration is
`8 * grip_scale * applied_brake` m/s². Lead deceleration is
`7 * grip_scale * target_brake` m/s², without actuator lag. Straight-line position
is integrated with constant acceleration within each step, including exact
in-step stopping; speed cannot become negative. Acceleration in CSV is the
measured speed difference over the previous step. CSV brake percentage is the
new command, before delay/lag. Jerk therefore reflects sample differences and
the final stop transition as well as controller behavior.

The PID and MPC retain their nominal 8 m/s² deceleration model across grip/delay
cases. The MPC assumes 7 m/s² lead braking. Its candidates are constant
decelerations over a finite horizon, with only the first action applied before
replanning. It is not a full nonlinear vehicle MPC and does not estimate grip.

An overlap (`gap <= 0`), premature AEB before THREAT, or timeout aborts the run.
Failed cases remain in the report and cause the CLI to return a nonzero status.
Use `--allow-stress-failures` to permit the two documented stress-case overlap
outcomes in automated reproduction. This does not make their comparisons valid
and does not suppress other execution or comparison failures.
The comparison also checks activation and completion, valid samples, provenance,
and matched observations before activation. A smaller stopping distance alone
does not establish an overall winner: gap, braking severity, and jerk matter too.

## Reproduction and provenance

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e '.[plots]'
python3 scripts/run_model_benchmark.py
# Standard-library-only execution:
python3 scripts/run_model_benchmark.py --output-dir /tmp/adas-model --no-plots
```

Every CSV has a `.metadata.json` sidecar with the complete settings, backend,
controller identity, Python version, source hash, CSV hash, and outcome. Identical
settings and source produce identical sample bytes. Plots may differ in embedded
metadata across library versions. No confidence intervals are reported because
these cases are deterministic scenarios, not independent noisy trials.

The model excludes sensing uncertainty, lateral motion, tires, road geometry,
collisions with other actors, and real actuator calibration. Results support
algorithm comparison within this plant; CARLA and vehicle validation remain
separate steps.
