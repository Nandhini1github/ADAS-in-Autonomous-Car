# System Architecture

## Direct scenario-control path

```text
config/town10_adas.json
          │
          ▼
ScenarioOrchestrator ──► target cruise / hard-brake state
          │
          ├──► Pure Pursuit ──────────────► ego steering
          ├──► Longitudinal PID ──────────► cruise throttle/brake
          └──► FCW/TTC SafetySupervisor
                         │
                         ├──► AEB PID ─┐
                         └──► AEB MPC ─┴──► ego emergency brake
                                             │
                                             ▼
                                CARLA Python API 0.10.0
                                             │
                                             ▼
                                CARLA Town10HD_Opt actors
```

PID and MPC share the same scenario, lateral controller, trigger logic, units, and
CSV logger. Only the emergency-brake law changes.

## Existing ROS 2 / Autoware observation path

```text
CARLA 0.10.0
      │
      ▼
autoware_carla_interface
      │
      ▼
ROS 2 Jazzy topics ──► town10_adas_monitor (read-only)
      │
      ▼
Autoware 1.9.0
```

The direct scenario runner does not publish Autoware control commands. Running it
against an actor already controlled by Autoware would create competing control
authority and is unsupported. The ROS 2 package deliberately subscribes only to
vehicle status so this boundary remains explicit. The direct actor uses the
`scenario_ego` role rather than Autoware's conventional `ego_vehicle` role.

## Runtime safety boundaries

- The map name must end in `Town10HD_Opt` or the runner exits.
- A route is built before actor control starts and must exceed the configured length.
- FCW/AEB logic is independent from PID/MPC selection.
- AEB is latched for the remainder of an event.
- Completion depends on an actual stopped ego, never distance to route end.
- Actor cleanup occurs in `finally`, and original CARLA world settings are restored.
