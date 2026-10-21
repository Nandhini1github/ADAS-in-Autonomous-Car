# ADAS in Autonomous Car

**PID versus MPC automatic emergency braking (AEB) in CARLA 0.10.0.**
Scenario 1 compares the two brake controllers when a lead vehicle suddenly
brakes from 60 mph. The Docker environment includes ROS 2 Jazzy and Autoware;
the recorded experiment runs the repository's Python CARLA adapter and
controller core directly.

[Docker setup](#docker-setup) · [Scenario 1](#scenario-1) ·
[Videos and CSVs](#videos-and-csvs) · [Sensors](#sensors-and-controller-inputs) ·
[Results](#measured-comparison)

## Docker setup

The recorded experiment uses two Docker engines on the same Linux machine:

| Component | Docker context | Container and role |
|---|---|---|
| Controller and review UI | `desktop-linux` — Docker Desktop | **`adas-autoware-ros2-jazzy`**, shown under the **`adas`** Compose group; runs the CARLA client, controllers, recording, plotting, and browser review server |
| GPU simulator | `default` — host Docker engine | **`adas-carla`**; runs CARLA with NVIDIA CDI and the local CARLA 0.10.0 assets |

Both use the image `adas-autoware-ros2-jazzy:carla010` in their respective
engines. The Desktop container reaches the simulator at
`host.docker.internal:2020`. The repository is mounted at `/workspace/adas`,
so generated files remain in this checkout. The review server is published
only on `127.0.0.1:8765`.

### Prerequisites

The Docker files reproduce this local setup. They depend on these existing
components, which are not included in the repository:

- Docker Desktop with the `desktop-linux` context and a host Docker engine
  with the `default` context; Docker Compose available for both.
- Local base image **`do18-ros:mpc-rviz-2026-09-30`** in Docker Desktop and
  **`d018-ros:jazzy-autoware`** in the host engine. These supply Python 3.12,
  ROS 2 Jazzy, Autoware, NumPy, Matplotlib, Pillow, and FFmpeg.
- CARLA **0.10.0** Linux assets and its Python **3.12** wheel.
- An NVIDIA GPU and NVIDIA CDI configured on the host engine, where the
  simulator requests `nvidia.com/gpu=all`.

Set the following environment variables before starting if your installation
paths differ from this machine:

| Variable | Purpose and current default |
|---|---|
| `CARLA_ROOT` | CARLA installation: `/home/nandhir/CARLA/Carla-0.10.0-Linux-Shipping` |
| `CARLA_API_DIR` | Directory containing the CARLA wheel: `/home/nandhir/CARLA/Carla-0.10.0-Linux-Shipping/PythonAPI/carla/dist` |
| `AUTOWARE_OVERLAY_DIR` | Autoware overlay used by the host Compose configuration: `/home/nandhir/DO18/docker/ros2/autoware` |
| `ADAS_DOCKER_CONTEXT` | Controller container context; defaults to `desktop-linux`. Set to `default` to use the host configuration. |
| `ADAS_ROS_DOMAIN_ID` | ROS domain; defaults to `42`. ROS messaging is not part of the recorded Scenario 1 control loop. |

The launcher sets the container UID and GID to the calling user's IDs. Build
and runtime definitions are in [Dockerfile](docker/adas.Dockerfile),
[Desktop Compose](docker/compose.desktop.yaml),
[host Compose](docker/compose.yaml), and [Docker launcher](scripts/docker.sh).

### Start Scenario 1 and review before saving

Run from the repository root:

```bash
bash scripts/docker.sh simulator
bash scripts/docker.sh start
bash scripts/docker.sh preview \
  --config 'AEB scenario 1/config.json' --spawn-index 105
```

Open **http://localhost:8765** and select **Start comparison**. Watch the live
camera and telemetry, then review the PID and MPC videos side by side with the
comparison graphs. Select **Save results** when you want to retain the pair.

New recordings are staged under `.runtime/`. Saving copies the reviewed
outputs into a new `results/run_...` directory. Starting the preview does not
automatically save final results. The simulator generates new observations;
archived CSVs are not used to drive the vehicles. Each controller run reloads
the world. An external controller must not command the same actors.

To reopen an existing temporary comparison, add
`--review-stage .runtime/<run-directory>` to the preview command, keeping its
matching configuration and spawn index. Each run produces a time-series CSV,
metadata JSON, and MP4 for each controller, plus comparison reports, plots,
and a summary CSV.

| Command | Action |
|---|---|
| `bash scripts/docker.sh status` | Show the controller container in the selected Docker context |
| `bash scripts/docker.sh shell` | Enter the container with the ROS/Autoware environment initialized |
| `bash scripts/docker.sh exec COMMAND...` | Run a command in the same environment |
| `bash scripts/docker.sh stop` | Stop services in the selected context; the default Desktop context contains the controller container |
| `bash scripts/docker.sh simulator-stop` | Stop the GPU simulator in the host Docker engine |

## Scenario 1

### 60 mph lead vehicle sudden braking

The reviewed comparison was recorded on **16 October 2026**. Both vehicles
approach **60 mph (96.6 km/h)**, the lead vehicle applies full braking, and the
ego vehicle responds through the shared AEB supervisor and the selected PID
or MPC brake controller.

**Both runs completed with positive measured gaps and zero collision sensor
contacts.** The saved comparison reports matching provenance and observations
through the first AEB activation, with `comparison_valid=true`. These are
newly recorded simulator outputs; existing CSV files were not inputs to the
experiment.

All Scenario 1 recordings, comparison outputs, configurations, and diagram
files are in [AEB scenario 1](<AEB scenario 1/>). This README documents the
archive. Shared controller and recording code is in
[src/town10_adas](src/town10_adas/) and [scripts](scripts/).

### Videos and CSVs

| PID - Scenario 1 | MPC - Scenario 1 |
|---|---|
| ![PID - Scenario 1](<AEB scenario 1/town10_aeb_pid.gif>) | ![MPC - Scenario 1](<AEB scenario 1/town10_aeb_mpc.gif>) |

The looping GIFs contain the complete recorded clips at **640 × 360, 10 fps**.
The original MP4s are **960 × 540, 20 fps**. Both include speed, gap, braking,
and AEB overlays. MP4 frame zero corresponds to the first CSV sample;
successive frames correspond to successive rows.

| Output | PID | MPC |
|---|---|---|
| Chase camera video | [Watch/download PID MP4](<AEB scenario 1/town10_aeb_pid.mp4>) | [Watch/download MPC MP4](<AEB scenario 1/town10_aeb_mpc.mp4>) |
| Recorded time series | [PID CSV](<AEB scenario 1/town10_aeb_pid.csv>) | [MPC CSV](<AEB scenario 1/town10_aeb_mpc.csv>) |
| Run evidence and provenance | [PID metadata](<AEB scenario 1/town10_aeb_pid.metadata.json>) | [MPC metadata](<AEB scenario 1/town10_aeb_mpc.metadata.json>) |

Use [summary.csv](<AEB scenario 1/comparison/summary.csv>) for bar charts with
one row per controller. Its `source_csv` paths are relative to the
`comparison/` directory. Use the individual traces for time plots, with
`time_s` on the horizontal axis and `ego_speed_mph`, `target_speed_mph`,
`distance_m`, `ttc_s`, `brake_percent`, or `ego_accel_mps2` as measurements or
commands. The original 12-column CSV schema is preserved.

CSV units are **mph** for speed, **m** for distance, **s** for time, **m/s²** for
acceleration, and **percent** for brake command. `inf` TTC means the vehicles
are not closing sufficiently for a finite TTC; leave those points blank when
plotting. Empty summary measurements mean unavailable, not zero. Controller
execution time was not recorded in this pair.

The [Markdown report](<AEB scenario 1/comparison/comparison.md>),
[JSON report](<AEB scenario 1/comparison/comparison.json>), and
[SVG figure](<AEB scenario 1/comparison/comparison.svg>) provide the comparison
in other formats.

![Scenario 1 PID and MPC recorded comparison](<AEB scenario 1/comparison/comparison.png>)

### Scenario setup

| Setting | Recorded configuration |
|---|---|
| CARLA client and server | **0.10.0** |
| Map and route | `Town10HD_Opt`, spawn index **105**, common fixed lane route |
| Simulation step | Synchronous, **0.05 s**; 20 observations per simulated second |
| Ego vehicle | Lincoln MKZ, `vehicle.lincoln.mkz` |
| Lead vehicle | Dodge Charger, `vehicle.dodge.charger` |
| Requested cruise speed | **60 mph** for both vehicles, approximately **96.6 km/h** |
| Nominal placement gap | **55 m** configured; first recorded measured bumper gap **59.060 m** |
| Lead braking trigger | Both speeds within **±1 mph** of their targets for **1.5 s** |
| Lead braking command | **100% brake**, zero throttle |
| Stop measurement | First ego sample at or below **1 mph** after AEB; a further **1 s** stop hold is required to complete the scenario |

The [saved preview configuration](<AEB scenario 1/preview_config.json>) records
the settings used for this pair;
[config.json](<AEB scenario 1/config.json>) is the reusable configuration.
Each controller run uses the same route, vehicle settings, cruise controllers,
and safety supervisor after reloading the world.

The scenario uses a **rolling start**. After 20 settling ticks, each vehicle
receives one initial velocity assignment and a warmup tick. Subsequent motion
follows CARLA physics and controller commands. Manual cruise gears are held at
**5 for the ego** and **6 for the lead**. Cruise throttle is initialized to
**0.35** and **0.80**, respectively; these are initial values, not fixed throttle
throughout the run. The measured speeds in the event timeline document the
result of that initialization.

### Control flow

The CARLA adapter reads live vehicle state, evaluates the shared supervisor,
and applies the selected PID or MPC brake command to the ego vehicle. The
lead vehicle brakes after the stable-speed hold. CSVs record observations and
commands produced by this loop for the subsequent comparison.

![Scenario 1 CARLA control flow](<AEB scenario 1/architecture.svg>)

[Open the editable draw.io diagram](<AEB scenario 1/architecture.drawio>) in
draw.io to edit the architecture.

| Component | Role |
|---|---|
| Pure pursuit | Shared bounded steering along the fixed CARLA lane route |
| Speed PID | Normal ego and lead speed regulation |
| Safety supervisor | Shared FCW/TTC and stopping-distance thresholds; latched AEB |
| AEB PID | Acceleration feedback tracking the requested emergency deceleration |
| AEB MPC | Ego speed, lead speed, and gap used to select constant-deceleration actions over a receding 1-D horizon |
| Scenario state machine | `ACCELERATE → HOLD → THREAT → AEB → COMPLETE`, or explicit `ABORTED` |

The MPC is a lightweight predictive controller with discrete actions. Its
simplified model does not establish superiority over PID across conditions.

### Sensors and controller inputs

| Component | Configuration and role |
|---|---|
| RGB camera | `sensor.camera.rgb`, **960 × 540**, **20 fps**, **90° field of view**. Rigid ego chase mount at **(x, y, z) = (−7, 0, 3.5) m**, pitch **−15°**. Supplies the live view and video recording; image pixels do not enter the controller. |
| Ego collision sensor | `sensor.other.collision`, attached to the ego. Records contact events and supports run termination on contact. It does not detect the lead vehicle for AEB or compute brake demand. Both archived metadata files contain zero contacts. |
| Vehicle state | CARLA actor positions and velocity magnitudes supply measured gap and speeds. Actor acceleration, projected along velocity, supplies PID deceleration feedback and the acceleration trace. |
| Lateral guidance | CARLA map waypoints and actor pose feed the shared pure pursuit steering controller. |

Gap is the Euclidean center distance minus the vehicles' longitudinal bounding
box half-lengths, an approximation on curved roads. Closing speed is the
positive ego-minus-lead speed difference; TTC is gap divided by closing speed
when that difference is sufficiently positive. These quantities are calculated
from **simulator ground truth**.

**Radar, lidar, GNSS, IMU, and camera object detection are not used in this
scenario.** ROS 2 Jazzy and Autoware are installed in the Docker environment,
but ROS/Autoware nodes do not execute the braking algorithm in this pair.

The common supervisor latches AEB on its TTC condition or its
stopping-distance-plus-margin condition. Configured TTC thresholds are
**3 s for FCW** and **2 s for AEB**, with a **5 m** distance margin and an
**8 m/s²** stopping-distance deceleration assumption. In this run, the distance
condition triggers AEB at a recorded TTC of **4.506 s**. FCW is also asserted
when AEB latches, so both flags first appear at the same sample.

### Recorded event timeline

Times are elapsed simulation times from the CSV. PID and MPC observations are
identical through AEB activation at the reported precision.

| Event | Time (s) | Ego speed (mph) | Lead speed (mph) | Measured gap (m) |
|---|---:|---:|---:|---:|
| First recorded sample, both runs | 0.050 | 59.316 | 59.186 | 59.060 |
| Lead applies full brake, both runs | 1.550 | 59.941 | 60.166 | 59.107 |
| First FCW and AEB, both runs | 3.300 | 59.938 | 35.299 | 49.629 |
| PID first sample at or below 1 mph | 7.750 | 0.655 | 0.000 | 9.283 |
| MPC first sample at or below 1 mph | 8.100 | 0.998 | 0.000 | 15.252 |
| PID complete after stop hold | 8.750 | 0.000 | 0.000 | 9.272 |
| MPC complete after stop hold | 9.100 | 0.000 | 0.000 | 15.133 |

### Measured comparison

| Metric | PID | MPC | MPC minus PID |
|---|---:|---:|---:|
| Samples | 175 | 182 | 7 |
| Speed at AEB (mph) | 59.938 | 59.938 | 0.000 |
| Gap at AEB (m) | 49.629 | 49.629 | 0.000 |
| Minimum measured gap (m) | 9.272 | 15.133 | +5.861 |
| Minimum finite TTC (s) | 1.763 | 2.564 | +0.801 |
| AEB to 1 mph stop threshold (s) | 4.450 | 4.800 | +0.350 |
| Distance from AEB to stop threshold (m) | 60.032 | 54.057 | −5.975 |
| Peak deceleration, finite difference (m/s²) | 7.063 | 7.519 | +0.456 |
| Peak absolute jerk, finite difference (m/s³) | 77.606 | 57.579 | −20.027 |
| Collision sensor contacts | 0 | 0 | 0 |

In this pair, MPC leaves a larger minimum gap and travels a shorter distance
after AEB, while taking **0.350 s longer** to reach the 1 mph threshold. Its peak
deceleration is higher and its measured peak absolute jerk is lower. These
different outcomes do not establish an overall winner.

### Measurement method and limits

Stopping distance integrates ego speed from the first AEB sample to the first
sample at or below 1 mph using trapezoids. Peak deceleration and jerk use
unsmoothed finite differences of rounded CSV speeds, so they depend on the
**0.05 s** sampling interval and quantization. The stop metric does not mean
exact zero velocity. Contact evidence comes from collision sensor events in
the metadata; positive gap alone is not contact evidence.

The comparison checks shared settings, controller identities, source hashes,
CSV hashes, and observations through AEB activation with an absolute tolerance
of **0.001**. Both traces must complete without contact or gap overlap for the
reported comparison to be valid. Check the validity fields before interpreting
controller differences.

Raw recordings and metadata are preserved; comparison reports were regenerated
for this archive. Original staging paths in metadata are historical
provenance. The links in this README point to the archived files, whose
checksums are recorded in the
[artifact manifest](<AEB scenario 1/artifact_manifest.json>). The
[preview log](<AEB scenario 1/preview.log>) is retained as well.

This is **one paired simulator demonstration with ideal vehicle-state
information**. It provides neither statistical significance, perception
validation, nor road safety certification. See
[metric definitions and integrity checks](docs/comparison_methodology.md) and
[CARLA run integrity](docs/carla_validation.md) for the comparison method and
additional validation requirements.

### Regenerate the archived comparison

Use the Docker environment to recompute the reports from the saved Scenario 1
traces and metadata, without running CARLA or changing the archive:

```bash
bash scripts/docker.sh exec python scripts/compare_aeb_results.py \
  --pid 'AEB scenario 1/town10_aeb_pid.csv' \
  --mpc 'AEB scenario 1/town10_aeb_mpc.csv' \
  --output-dir .runtime/scenario_1_recomputed
```

This produces summary CSV, JSON, Markdown, PNG, and SVG outputs. To record a
fresh pair, use the [Docker preview workflow](#start-scenario-1-and-review-before-saving).

## Additional tools and documentation

- [ROS 2 nodes](docs/ros2_nodes.md): optional offline CSV replay, shadow brake
  recommendations, recording, and an Autoware velocity monitor. Replay does
  not apply commands to a vehicle and cannot measure closed-loop stopping
  performance. ROS messages use SI speeds and brake percentages.
- [Model benchmark](results/model/README.md) and
  [model assumptions](docs/model_benchmark.md): a separate deterministic 1-D
  parameter sweep, including documented stress-case failures. These model
  outputs are distinct from the recorded CARLA Scenario 1 pair.
- [System architecture](docs/system_architecture.md) and
  [WSL2 / Autoware setup](docs/wsl2_autoware.md): supporting architecture and
  an additional environment setup guide.
- [Historical cruise-only analysis](docs/baseline_csv_analysis.md): retained
  background, not evidence of emergency braking.
