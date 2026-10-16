# System architecture

`src/town10_adas` contains controller math, the safety supervisor, orchestration,
CSV logging, model simulation, and metrics. The math and model do not import ROS
or CARLA. Both adapters use the same core.

## Scenario 1 live CARLA loop

![Scenario 1 CARLA control flow](<../AEB scenario 1/architecture.svg>)

[Editable draw.io source](<../AEB scenario 1/architecture.drawio>) ·
[Scenario 1 configuration, sensors, and recordings](../README.md#scenario-1)

The CARLA adapter reads simulator vehicle positions, velocities, and
acceleration. The safety supervisor evaluates closing speed, TTC, and stopping
distance. Once AEB latches, the chosen PID or MPC controller commands ego
braking through the CARLA API. Normal speed regulation and pure pursuit
steering share the same adapter. The scenario orchestrator commands the lead
vehicle's hard braking after both vehicles hold their cruise speeds.

The RGB camera feeds the live preview and recorded video. The collision sensor
records contacts and can abort a run. Time-series CSVs and metadata are outputs;
the comparison script reads those outputs to generate metrics and plots.

## Other execution modes

The longitudinal model runner uses the shared controller core with a separate
1-D plant and writes its own CSVs and provenance under `results/model/`.
The optional ROS tools can replay recorded telemetry into a shadow controller
and record its recommendations. These offline tools are separate from the
Scenario 1 live control loop.

## Command authority

The CARLA adapter commands its own actors. The model runner commands its local
plant. The ROS shadow controller produces recommendations without commanding
CARLA or Autoware. The optional Autoware monitor only subscribes to vehicle status.

## Fair comparisons

Model pairs share settings and initial conditions. Source/CSV hashes identify
inputs. The comparison layer validates time ordering, numbers, event/completion
status, provenance, and pre-AEB observations. Invalid pairs remain diagnostics.

CARLA pairs also need matching route/spawn/environment provenance and contact
observations. Matching metadata cannot eliminate nondeterministic traffic or
establish physical safety. See the CARLA validation guide.

## ROS packaging

`town10_adas_core` installs the shared Python source without duplicating the
algorithm. `town10_adas_interfaces` defines typed messages. `town10_adas_ros`
installs replay/controller/recorder executables and launch files. The optional
`town10_adas_monitor` remains a separate Autoware-dependent package.
