# ADAS in Autonomous Car

Reproducible **PID versus MPC automatic emergency braking (AEB)** experiments, a
shared safety supervisor, a CARLA adapter, and ROS 2 Jazzy nodes.

## Scenario 1: 60 mph lead vehicle sudden braking

**[Open Scenario 1 GIFs, videos, CSVs, and sensor details](<AEB scenario 1/README.md>)**

Both vehicles cruise at 60 mph (96.6 km/h), then the lead vehicle applies full
braking. The reviewed CARLA PID/MPC pair completed with zero recorded contacts:
minimum gap **9.272 m for PID** and **15.133 m for MPC**. The scenario guide
documents the rolling start, actual event speeds, controller tradeoffs, and
reproduction commands.

The RGB camera records the scenario and the collision sensor records
contact. Braking uses CARLA ground-truth vehicle state. ROS 2 Jazzy and Autoware
are available in the container; this experiment runs the repository's Python
controllers directly through CARLA, without an Autoware perception pipeline.

**Target system: Windows → WSL2 → Ubuntu 24.04 + ROS 2 Jazzy + Autoware,
with CARLA running in Docker.** Start with the
[WSL2 / Autoware / Docker setup guide](docs/wsl2_autoware.md).

## Docker Desktop workflow

`adas-autoware-ros2-jazzy` runs in Docker Desktop (`desktop-linux` context), under
the **adas** group. Its image extends the existing
`do18-ros:mpc-rviz-2026-09-30` image with an isolated CARLA Python environment.
ROS 2 Jazzy, Autoware, FFmpeg, and plotting libraries are available. The project
is mounted at `/workspace/adas`, so generated files persist in this checkout.

The GPU simulator runs as `adas-carla` in the host Docker engine (`default`
context), using NVIDIA CDI and the locally installed CARLA 0.10.0 assets.
The Desktop container reaches it at `host.docker.internal:2020`.

These Docker files reproduce the local Linux Docker setup. They require
the existing local base images (`do18-ros:mpc-rviz-2026-09-30` in Docker Desktop
and `d018-ros:jazzy-autoware` in the host engine), CARLA 0.10.0 Linux assets and
its Python 3.12 wheel, and an NVIDIA GPU configured for CDI on the host engine.
The base images provide Python 3.12, ROS 2 Jazzy, Autoware, NumPy, Matplotlib,
Pillow, and FFmpeg. They are not downloaded or included in this repository.
Set `CARLA_ROOT`, `CARLA_API_DIR`, and `AUTOWARE_OVERLAY_DIR` to your installation
paths when using a different machine; the compose defaults describe this run's
local installation.

```bash
bash scripts/docker.sh simulator
bash scripts/docker.sh start
bash scripts/docker.sh preview
```

Open **http://localhost:8765**, click **Start comparison**, and watch the live
camera and telemetry. When both runs finish, review their videos side by side
and the comparison graphs. Click **Save results** only when you want to keep
them. The preview stages new recordings under `.runtime/`; saving copies the
reviewed outputs into a new `results/run_...` directory. Starting a
preview never automatically publishes its recordings into `results/`.

### 60 mph sudden-braking scenario

Use the high-speed configuration with spawn 105:

```bash
bash scripts/docker.sh preview \
  --config "AEB scenario 1/config.json" --spawn-index 105
```

Both vehicles start at 60 mph (96.6 km/h) and must remain within 1 mph of that
speed for 1.5 seconds before the lead vehicle applies full braking. The nominal
initial gap is 55 m; the CSV records the actual measured bumper gap. This uses a
rolling start: velocity is initialized once, then CARLA vehicle physics and the
controllers determine motion. The Lincoln's cruise gear is held at 5 and the
Charger's at 6, with separately initialized cruise throttle. PID and MPC runs
use the same scenario, cruise settings, and safety supervisor.

The videos include speed, gap, braking, and AEB overlays. An RGB camera supplies
the recordings, and a collision sensor records contacts. AEB uses CARLA's
ground-truth positions, speeds, and acceleration; radar, LiDAR, and camera
object detection are not part of this experiment. These runs compare controller
behavior with ideal vehicle-state information and do not validate perception
or real-world safety. Stopping time and distance use a 1 mph stop threshold.

To reopen an existing temporary comparison without running again, add
`--review-stage .runtime/<run-directory>` with its matching configuration.
Review the videos and graphs before clicking **Save results**.

Use `bash scripts/docker.sh shell` to enter the ROS/Autoware shell.
For unattended recording directly into a final output directory, use a new
directory for each experiment:

```bash
bash scripts/docker.sh exec python scripts/run_town10_adas.py \
  --host host.docker.internal --port 2020 --controller both --spawn-index 105 \
  --reload-world --video --output-dir results/run_01
bash scripts/docker.sh exec python scripts/compare_aeb_results.py \
  --pid results/run_01/town10_aeb_pid.csv \
  --mpc results/run_01/town10_aeb_mpc.csv \
  --output-dir results/run_01/comparison
```

Each controller produces a time-series CSV, a metadata JSON, and a chase-camera
MP4 at simulation speed. The comparison produces graphs, a report, and
`summary.csv` with one metrics row per controller. Check its validity fields
before interpreting differences. Video frame zero corresponds to the first CSV
sample, and successive frames correspond to successive rows.

`status` shows the Desktop container, `stop` stops it, and `simulator-stop` stops
the GPU simulator. `CARLA_ROOT`, `CARLA_API_DIR`, and `AUTOWARE_OVERLAY_DIR` can
override local installation paths. Both compose files record the setup in
`docker/`. Existing CSVs are not inputs to the live simulator.

## Additional model comparisons

**[View the six-case comparison, raw data, and plots →](results/model/README.md)**

| Evidence | Status |
|---|---|
| Deterministic 1-D model | Six paired scenarios: 15/10, 25/15, 35/20, 50/25 mph, reduced grip, delayed braking |
| CARLA Town10 | [Scenario 1](<AEB scenario 1/README.md>): reviewed 60/60 mph paired run with videos, fresh CSVs, provenance, and contact checks |
| ROS 2 | Typed telemetry, PID/MPC shadow controller, recorder, replay launch, optional Autoware monitor |

The results under `results/model/` are outputs of the disclosed **1-D model**.
Scenario 1 contains separately recorded CARLA outputs. Reports compare
stopping time/distance, minimum gap/TTC, peak
deceleration, and jerk. They check matched pre-AEB inputs and retain invalid runs
as failures. The six cases are a parameter sweep, not statistical repetitions.

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e '.[plots]'
python3 scripts/run_model_benchmark.py --allow-stress-failures
python3 -m unittest discover -s tests -v
```

Standard-library-only reproduction:

```bash
python3 scripts/run_model_benchmark.py --output-dir /tmp/adas-model --no-plots --allow-stress-failures
```

Compare existing files using their `.metadata.json` sidecars:

```bash
python3 scripts/compare_aeb_results.py \
  --pid results/model/urban_15_10/pid.csv \
  --mpc results/model/urban_15_10/mpc.csv \
  --output-dir /tmp/adas-comparison
```

See [model assumptions](docs/model_benchmark.md) and
[metric definitions and integrity checks](docs/comparison_methodology.md).

Both controllers reach overlap in the reduced-grip and delayed-braking stress
cases. The flag above permits those documented failures during reproduction;
their reports remain invalid and visible.

## Scenario 1 control flow

The CARLA adapter reads live simulator vehicle state, evaluates the shared AEB
supervisor, and applies the selected PID or MPC brake command to the ego
vehicle. The lead vehicle brakes after the stable-speed hold. CSVs record the
observations and commands produced by this loop.

![Scenario 1 CARLA control flow](<AEB scenario 1/architecture.svg>)

[Edit the diagram in draw.io](<AEB scenario 1/architecture.drawio>) ·
[Sensor details and recorded outputs](<AEB scenario 1/README.md>)

## Optional ROS 2 tools

The ROS 2 packages support offline CSV replay, shadow brake recommendations,
and recording. The diagram above describes the live CARLA run used for
Scenario 1. ROS/Autoware nodes do not execute its braking algorithm.
Autoware is needed only for the optional velocity monitor.

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to town10_adas_ros
source install/setup.bash
```

**[ROS 2 launch commands, topics, QoS, and runtime validation](docs/ros2_nodes.md)**

The controller publishes shadow brake recommendations. Replay does not apply
these recommendations to a vehicle, so replay alone cannot measure closed-loop
stopping performance.

## CARLA experiment

The adapter targets **CARLA 0.10.0**, **Town10HD_Opt**, and the configured vehicle
blueprints. It directly controls its own `scenario_ego` and lead actors. An
external controller must not command the same actor.

Edit [config/town10_adas.json](config/town10_adas.json) for your host and scenario;
the default address is `127.0.0.1:2000`, the CARLA Docker port published to WSL.
Use `--host` and `--port` to override it for your Docker network.

```bash
python3 scripts/check_carla_integration.py
python3 scripts/run_town10_adas.py --controller both
python3 scripts/compare_aeb_results.py \
  --pid results/town10_aeb_pid.csv \
  --mpc results/town10_aeb_mpc.csv \
  --output-dir results/carla_comparison
```

Read [CARLA run integrity and validation](docs/carla_validation.md) before
interpreting results. Traffic, route geometry, contacts, and physics need separate
validation. Empty CARLA placeholders are rejected by the analyzer; model runs
never overwrite them.

## Control design

| Component | Role |
|---|---|
| Pure Pursuit | Bounded steering along the CARLA fixed lane route |
| Speed PID | Normal CARLA ego/lead speed regulation |
| Safety supervisor | Shared FCW/TTC and stopping-distance thresholds; latched AEB |
| AEB PID | Feedback tracking of requested emergency deceleration |
| AEB MPC | Enumerated constant-deceleration actions over a receding 1-D horizon |
| Scenario | `ACCELERATE → HOLD → THREAT → AEB → COMPLETE`, or explicit `ABORTED` |

The MPC is a lightweight discrete-action predictive controller; its simplified
model does not establish superiority over PID in all conditions. Each model pair
shares its supervisor, time step, initial conditions, and lead behavior.

CSV units: speeds **mph**, distance **m**, time **s**, acceleration **m/s²**, brake
**percent**. ROS messages use SI speeds and brake percentages. The original
12-column CSV schema is preserved.

## Validation

Tests cover bounds, invalid inputs, overlap-triggered AEB, state transitions,
repeatable model runs, comparison integrity, and metric edge cases. CI runs the
Python tests, reproduces the model suite, and builds standalone ROS packages.
Live CARLA validation remains separate. Local validation passed **36 Python tests**,
the **four-package Jazzy/Autoware build**, and real DDS replay checks for PID and
MPC (288 source rows each, with stale/duplicate/NaN/watchdog/recovery checks).
Replay checks passed with both Cyclone DDS and Fast DDS.
The Autoware VelocityReport monitor also passed its DDS check.

The historical [cruise-only analysis](docs/baseline_csv_analysis.md) is retained;
it is not evidence of emergency braking. See [architecture](docs/system_architecture.md).
