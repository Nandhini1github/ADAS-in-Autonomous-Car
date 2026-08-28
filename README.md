# ADAS in Autonomous Car

A modular Automatic Emergency Braking (AEB) experiment for **CARLA 0.10.0**,
**Town10HD_Opt**, **ROS 2 Jazzy**, and an existing **Autoware 1.9.0** environment.
The experiment compares PID and lightweight MPC emergency braking under one
shared route, scenario state machine, FCW/TTC supervisor, and CSV schema.

## What is implemented

| Module | Responsibility |
|---|---|
| `lateral.py` | Conservative fixed-route Pure Pursuit lane tracking |
| `longitudinal.py` | Normal ego/target speed regulation with throttle and service brake |
| `safety.py` | Closing speed, TTC, FCW, stopping-distance check, and latched AEB decision |
| `aeb.py` | AEB deceleration PID and 1-D receding-horizon MPC |
| `scenario.py` | `ACCELERATE → HOLD → THREAT → AEB → COMPLETE` orchestration |
| `carla_runner.py` | CARLA 0.10.0 spawning, synchronous stepping, actuation, and cleanup |
| `csv_logger.py` | Common PID/MPC tuning and comparison log |
| `town10_adas_monitor` | Read-only ROS 2 subscriber for Autoware vehicle velocity |

The controller math and state machine do not import CARLA. CARLA-specific code is
confined to the adapter, which makes the safety and control logic unit-testable.

## Why these controllers were chosen

The controllers were selected to make the Town10 experiment stable, explainable,
and fair to reproduce before attempting higher-speed or full Autoware integration.

| Controller | Selection reason | Important limitation |
|---|---|---|
| Pure Pursuit lateral control | The scenario follows a known, fixed lane route, so a geometry-based tracker is sufficient and requires few tuning parameters. Speed-dependent lookahead smooths steering as speed increases, while the ±0.18 limit keeps corrections conservative. The uploaded baseline supports this choice: maximum absolute steering was only 0.0313 during the smooth run. | It does not optimize a vehicle model or surrounding traffic and must be retuned if the route, speed range, or vehicle steering response changes substantially. |
| Longitudinal speed PID | PID is computationally inexpensive, interpretable, and appropriate for holding fixed ego and target speeds. The implementation can command service brake as well as throttle, correcting the earlier behavior in which both vehicles settled above their requested speeds. | Its gains depend on CARLA vehicle physics and should be rechecked for different vehicle blueprints or high-speed tests. |
| TTC/FCW safety supervisor | Keeping threat detection separate from brake control gives PID and MPC identical activation conditions. TTC provides an intuitive closing-risk measure, while the stopping-distance check covers situations where TTC alone is insufficient. AEB is latched so braking cannot chatter on and off around a threshold. | TTC assumes the current relative motion continues and is not a complete prediction of curved paths or cut-in behavior. |
| AEB PID | A deceleration-tracking PID is the transparent baseline: it reacts to measured ego acceleration, has low runtime cost, and makes gain effects easy to inspect in the CSV. | It reacts to deceleration error after braking begins and does not explicitly predict the future gap. The uploaded cruise-only CSV did not activate AEB, so these emergency-braking gains still require CARLA validation. |
| AEB MPC | The lightweight MPC predicts ego speed, target speed, and gap over a short horizon, then selects a bounded braking action. It provides a meaningful predictive comparison against PID without requiring an external optimization package. | Its 1-D constant-deceleration model simplifies tire, road, actuator-delay, and target-motion behavior; results must be validated against CARLA physics. |

This is deliberately a **PID-versus-MPC emergency-braking comparison**, not a claim
that either controller is universally best. Both use the same route, speed control,
scenario timing, safety thresholds, fixed simulation step, and log schema so the
comparison changes only the emergency-braking law.

## Units

- Speeds in configuration, terminal output, and CSV: **mph**
- Distances and gaps: **m**
- Acceleration/deceleration: **m/s²**
- TTC and time: **s**
- CARLA boundary calculations: SI units, converted explicitly for logging/configuration
- Throttle, brake, and normalized steering: `[0, 1]` (steering is signed)

## Scenario

The checked-in low-speed validation configuration uses a 15 mph ego, 10 mph lead
vehicle, and 80 m initial bumper gap. It is intentionally conservative.

1. Both vehicles accelerate to their configured speeds.
2. Both must stay within ±1 mph for 3 s.
3. The target receives an explicit 100% brake command.
4. The common supervisor evaluates TTC and stopping distance.
5. The selected PID or MPC controller commands ego braking after AEB activation.
6. Completion requires ego speed below 1 mph for 1 s. Route-end proximity does not
   terminate the experiment.
7. Actors remain visible for 5 s, then are destroyed and CARLA world settings are restored.

Edit [config/town10_adas.json](config/town10_adas.json) for later 25/15,
35/20, or 50/25 mph validation. Increase speed only after reviewing each generated
CSV; high-speed behavior is not claimed as validated by this repository.

## Environment

| Component | Expected configuration |
|---|---|
| Simulator / Python API | CARLA 0.10.0 |
| Map | `Town10HD_Opt` |
| ROS | ROS 2 Jazzy, Fast DDS |
| Autonomous-driving stack | Autoware 1.9.0 |
| Python | 3.12 |
| Container host address | `host.docker.internal:2000` |

Confirm CARLA is reachable and the correct map is loaded:

```bash
cd /home/aw/platform/autoware/scenarios/01_aeb
python3 scripts/check_carla_integration.py
```

The output must end in `Town10HD_Opt` before running the scenario.

## Run the direct-CARLA experiment

From the repository root inside the Autoware/CARLA container:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
python3 scripts/run_town10_adas.py --controller both
```

The command runs the same scenario once with PID and once with MPC. Each run
overwrites its assigned file, so the `results` directory contains one canonical CSV
per controller rather than accumulating duplicate logs:

```text
results/town10_aeb_pid.csv
results/town10_aeb_mpc.csv
```

The repository does not ignore these files, so completed PID and MPC results are
visible to Git and can be committed as simulation evidence.

Both files use the uploaded Town10 reference schema:

```text
time_s,phase,ego_speed_mph,target_speed_mph,distance_m,closing_speed_mph,
ttc_s,fcw,aeb_active,brake_percent,ego_steer,ego_accel_mps2
```

Run only one controller when a comparison pair is not required:

```bash
python3 scripts/run_town10_adas.py --controller pid
python3 scripts/run_town10_adas.py --controller mpc
```

Summarize either run and confirm that it actually exercised AEB:

```bash
python3 scripts/analyze_aeb_csv.py results/town10_aeb_pid.csv
python3 scripts/analyze_aeb_csv.py results/town10_aeb_mpc.csv
```

Use an explicit path or alternative configuration when needed:

```bash
python3 scripts/run_town10_adas.py \
  --controller pid \
  --config config/town10_adas.json \
  --output results/my_pid_run.csv
```

`--output` is available only for a single-controller run. It is intentionally
rejected with `--controller both` so the two comparison filenames remain unambiguous.

## ROS 2 and Autoware usage

There are two distinct paths; they should not be confused:

- `run_town10_adas.py` uses the CARLA Python API and directly applies
  `carla.VehicleControl` to the two scenario actors. It does **not** publish
  Autoware control commands and must not compete with Autoware for authority over
  the same ego actor. Its ego role is `scenario_ego`, deliberately distinct from
  Autoware's usual `ego_vehicle` role.
- The included ROS 2 package is read-only. It proves that the repository can consume
  Autoware vehicle status while the existing `autoware_carla_interface` exposes
  CARLA data to ROS 2. It does not make the direct controller an Autoware controller.

In a terminal where ROS 2 Jazzy and the Autoware workspace are sourced:

```bash
source /opt/ros/jazzy/setup.bash
source /home/aw/platform/autoware/install/setup.bash
cd /home/aw/platform/autoware/scenarios/01_aeb/ros2_ws
colcon build --symlink-install --packages-select town10_adas_monitor
source install/setup.bash
ros2 run town10_adas_monitor vehicle_monitor
```

Useful integration checks:

```bash
ros2 topic list | grep -E '^/sensing/|^/vehicle/status/'
ros2 topic echo /vehicle/status/velocity_status --once
ros2 topic hz /sensing/lidar/top/pointcloud_before_sync
ros2 topic hz /sensing/camera/CAM_FRONT/image_raw
```

Previously observed interfaces include:

```text
/sensing/camera/CAM_FRONT/image_raw
/sensing/camera/CAM_FRONT/camera_info
/sensing/lidar/top/pointcloud_before_sync
/sensing/gnss/pose_with_covariance
/sensing/imu/tamagawa/imu_raw
/vehicle/status/velocity_status
/vehicle/status/steering_status
/vehicle/status/gear_status
/vehicle/status/control_mode
/vehicle/status/actuation_status
```

Topic availability depends on the launched Autoware/CARLA interface configuration.
A future Autoware-integrated controller should publish the correct Autoware control
message through a separately validated command path; this repository does not claim
that integration yet.

## CSV evidence and tuning status

The uploaded baseline CSV contained 414 samples over 26.545 s. Maximum absolute
steering was only 0.0313, supporting retention of conservative lateral tuning.
However, every sample remained in `CRUISE`; minimum TTC was 7.555 s and there were
zero FCW, AEB, or braking samples. The run therefore did **not** validate AEB gains.

The complete evidence table and interpretation are in
[docs/baseline_csv_analysis.md](docs/baseline_csv_analysis.md). New PID/MPC runs
must demonstrate `THREAT` and `AEB` rows before controller performance is compared.

## Validation and acceptance checks

Local tests validate configuration loading, unit-level steering bounds, overspeed
braking, TTC/FCW/AEB behavior, AEB latching, normalized PID/MPC output, and the full
scenario state sequence. A real CARLA run is additionally required because unit tests
cannot verify vehicle physics, blueprint availability, map topology, or Autoware topics.

For each CARLA run, verify:

- the log contains `HOLD`, `THREAT`, `AEB`, and `COMPLETE`;
- the target brakes only after the stable hold;
- FCW precedes or coincides with AEB;
- brake stays between 0% and 100%;
- steering stays bounded by ±0.18;
- minimum gap remains positive;
- PID and MPC are compared at the same config and fixed time step.
