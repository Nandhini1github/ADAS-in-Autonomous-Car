# CARLA validation and run integrity

The local CARLA Python API imported successfully, but a read-only connection to
127.0.0.1:2000 was refused during this update. No CARLA comparison was collected.
The canonical results/town10_aeb_pid.csv and town10_aeb_mpc.csv remain empty
placeholders. Model outputs are stored separately.

## Prepare a comparison

Use an isolated CARLA server with no other synchronous ticker or competing
controller. Start CARLA using your normal simulator setup, load Town10HD_Opt, and
adjust the host in config/town10_adas.json. This repository's probe only reads
server state:

```bash
python3 scripts/check_carla_integration.py --host 127.0.0.1 --port 2000 --timeout 3
python3 scripts/run_town10_adas.py --controller both --spawn-index 0
```

Both runs use the same explicit spawn index. If a pose is occupied or the lane
cannot provide the route length, the run fails rather than silently selecting a
different location. Choose another index explicitly for a fresh comparison.
Paired execution requires synchronous mode.

Each run attaches an ego collision sensor, records events, and aborts on contact,
gap overlap, or AEB before the scheduled threat. Timeout remains an explicit
ABORTED phase. The adapter stops/destroys its owned sensors and actors and restores
the previous world settings in cleanup. Mock tests cover fixed-spawn failure
cleanup and provenance; full physical cleanup still needs a live-server check.

## Evidence

After the CSV closes, a .metadata.json sidecar records source=carla, controller,
complete config, chosen route, actor transforms, map, client/server versions,
world time settings, weather, collision count/events, outcome, source hash and
CSV hash. Exceptions after logging begins leave incomplete evidence. Metadata
cannot certify that external traffic or a different vehicle calibration was absent.

The gap remains the existing Euclidean center distance minus vehicle half-lengths,
now preserving negative values. On curves this approximates bumper separation;
it is not lane-relative distance. The collision sensor provides separate contact
evidence. Do not interpret a positive approximate gap alone as collision freedom.

Compare files using scripts/compare_aeb_results.py. The comparison rejects
different settings and mismatched pre-AEB observations; CARLA may require repeated
isolated runs to achieve matching traces. Loosening a tolerance must be disclosed
and justified, not used to hide a different experiment.

Before publishing CARLA results, inspect THREAT/AEB/COMPLETE phases, source
settings, both trajectories, lane tracking, collision callbacks, positive gaps,
bounded commands, and stopping metrics. Verify the lead vehicle brakes after the
stable hold and that no other process ticks or controls this world. The current
model benchmark does not substitute for these checks.
