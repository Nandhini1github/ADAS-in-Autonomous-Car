# ROS 2 replay and shadow evaluation

The Jazzy pipeline uses the shared safety supervisor and PID/MPC algorithms. It
replays full scenario CSVs and records recommendations with their source samples.
It publishes no vehicle actuation commands. Recorded motion does not change;
replay alone cannot demonstrate improved stopping performance.

| Package | Purpose |
|---|---|
| town10_adas_core | Installs the repository's shared Python source |
| town10_adas_interfaces | Typed Telemetry and ShadowCommand messages |
| town10_adas_ros | Replay, controller, recorder, launch and parameters |
| town10_adas_monitor | Optional Autoware velocity subscriber |

## Build and launch

Keep the workspace inside this repository because core installation references
../../../src/town10_adas from its package directory. A normal installation works
without the source tree; symlink installation needs the tree.

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/ADAS-in-Autonomous-Car/ros2_ws
rosdep install --from-paths src/town10_adas_core src/town10_adas_interfaces src/town10_adas_ros --ignore-src -r -y
colcon build --packages-up-to town10_adas_ros
source install/setup.bash

ros2 launch town10_adas_ros replay.launch.py \
  input_csv:=/path/to/ADAS-in-Autonomous-Car/results/model/urban_15_10/pid.csv \
  output_csv:=/tmp/urban_pid_shadow.csv controller:=pid
```

After Ctrl-C, change output_csv to a new filename and controller to mpc to replay
the same observations through MPC. The optional monitor needs
autoware_vehicle_msgs and is excluded from the standalone build dependency graph.

## Topics and behavior

| Topic | Type | Units |
|---|---|---|
| /adas/telemetry | town10_adas_interfaces/msg/Telemetry | Speeds m/s, gap m, acceleration m/s², observed brake percent |
| /adas/shadow_command | town10_adas_interfaces/msg/ShadowCommand | TTC s, stopping distance m, brake percent, processing ms |

csv_replay validates the full schema, numbers, flags, terminal rows, and increasing
source times. It waits for a subscriber then delays one second for graph discovery.
Playback uses source time scaled by rate. EOF leaves nodes alive for watchdog
inspection; stop the launch with Ctrl-C.

aeb_shadow validates publication age, numeric values, sequence and source time.
New run IDs reset the controller and AEB latch. The watchdog emits one invalid
recommendation after input silence. A source_valid=false message invalidates the
whole recommendation: its zero brake field does not instruct a vehicle to release
its brakes. Node parameters are read-only after startup.

metrics_recorder embeds the exact input with each recommendation, preserving all
12 observed CSV fields and adding shadow_* outputs, run ID, sequence, validity,
reason and processing_ms. Invalid/timeout rows are diagnostic events, including
duplicate source times, and cannot be passed directly to the closed-loop metrics
analyzer. Filter them and split run IDs before separate analysis. The recorder
refuses existing files unless overwrite is enabled. Ctrl-C writes summary JSON.
Callback processing time excludes DDS and end-to-end latency.

Pass params_file:=/absolute/custom.yaml to override
[defaults.yaml](../ros2_ws/src/town10_adas_ros/config/defaults.yaml). Defaults match
the checked-in JSON controller gains. Replay rate changes wall time; controller
integration uses source time. Slow replay or sparse samples may require a larger
stale_timeout_s.

The header stamp is ROS publication time; source_time_s remains experiment time.
frame_id defaults to base_link; these nodes do not publish transforms. Reliable
QoS with depth 100 is the replay default. Set qos_reliability:=best_effort on the
controller for a best-effort live publisher. See the official
[ROS QoS guide](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html).

## Runtime validation

```bash
ROS_DOMAIN_ID=87 python3 tests/smoke_replay.py ../results/model/urban_15_10/pid.csv
ros2 interface show town10_adas_interfaces/msg/Telemetry
```

The smoke script uses real DDS, both controllers, temporary files, and unique
topic names. It checks every source field, bounds, summaries, stale input,
duplicates, NaN rejection, watchdog expiry, and new-run recovery. Domain 87
separates it from the simulator's default graph. No CARLA or Autoware is required.

Validated in the existing Ubuntu 24.04/Jazzy/Autoware container: all four packages
built; both replay modes preserved all 288 fixture rows and passed the rejection
and recovery checks with both Cyclone DDS and Fast DDS. The optional monitor received a real DDS VelocityReport and
converted 4.4704 m/s to 10 mph. Run that check after sourcing Autoware with
`ROS_DOMAIN_ID=87 python3 tests/smoke_autoware_monitor.py`.
