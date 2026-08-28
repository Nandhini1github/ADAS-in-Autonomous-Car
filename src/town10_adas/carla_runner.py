"""CARLA 0.10.0 adapter for the modular Town10 ADAS stack."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time
from typing import Any, Sequence

from .aeb import AEBMPC, AEBPID
from .config import AppConfig, load_config
from .csv_logger import ScenarioCsvLogger
from .lateral import Pose2D, PurePursuitController, RoutePoint
from .longitudinal import ControlDemand, LongitudinalPID
from .safety import SafetySupervisor
from .scenario import ScenarioObservation, ScenarioOrchestrator
from .units import mph_to_mps, mps_to_mph


def _speed_mps(actor: Any) -> float:
    velocity = actor.get_velocity()
    return math.sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)


def _acceleration_mps2(actor: Any) -> float:
    acceleration = actor.get_acceleration()
    velocity = actor.get_velocity()
    speed = _speed_mps(actor)
    if speed < 0.1:
        return 0.0
    return (
        acceleration.x * velocity.x
        + acceleration.y * velocity.y
        + acceleration.z * velocity.z
    ) / speed


def _route_distance(points: Sequence[Any]) -> float:
    return sum(
        points[index].transform.location.distance(points[index - 1].transform.location)
        for index in range(1, len(points))
    )


def _yaw_delta_degrees(first: Any, second: Any) -> float:
    return abs((second.transform.rotation.yaw - first.transform.rotation.yaw + 180.0) % 360.0 - 180.0)


def build_fixed_lane_route(start: Any, minimum_length_m: float, resolution_m: float = 2.0) -> list[Any]:
    """Follow the least-turning driving-lane continuation through Town10."""
    route = [start]
    while _route_distance(route) < minimum_length_m:
        candidates = route[-1].next(resolution_m)
        candidates = [candidate for candidate in candidates if str(candidate.lane_type).endswith("Driving")]
        if not candidates:
            raise RuntimeError("route ended before route_minimum_m; choose another spawn point")
        route.append(min(candidates, key=lambda candidate: _yaw_delta_degrees(route[-1], candidate)))
    return route


def _waypoint_at_distance(route: Sequence[Any], distance_m: float) -> Any:
    travelled = 0.0
    for index in range(1, len(route)):
        travelled += route[index].transform.location.distance(route[index - 1].transform.location)
        if travelled >= distance_m:
            return route[index]
    raise RuntimeError("route is too short for requested spawn separation")


def _spawn_vehicle(world: Any, blueprint_id: str, role_name: str, waypoint: Any) -> Any:
    blueprint = world.get_blueprint_library().find(blueprint_id)
    blueprint.set_attribute("role_name", role_name)
    transform = waypoint.transform
    transform.location.z += 0.35
    actor = world.try_spawn_actor(blueprint, transform)
    if actor is None:
        raise RuntimeError(f"could not spawn {role_name} ({blueprint_id})")
    return actor


def _spawn_scenario_actors(world: Any, config: AppConfig) -> tuple[Any, Any, list[Any]]:
    """Try Town10 spawn points until a long route and two clear poses are found."""
    spawn_allowance_m = config.scenario.initial_gap_m + 8.0
    required_route_m = config.scenario.route_minimum_m + spawn_allowance_m
    last_error: Exception | None = None
    for spawn_transform in world.get_map().get_spawn_points():
        ego = None
        try:
            start_waypoint = world.get_map().get_waypoint(
                spawn_transform.location,
                project_to_road=True,
            )
            route = build_fixed_lane_route(start_waypoint, required_route_m)
            target_waypoint = _waypoint_at_distance(route, spawn_allowance_m)
            ego = _spawn_vehicle(
                world,
                config.carla.ego_blueprint,
                "scenario_ego",
                start_waypoint,
            )
            target = _spawn_vehicle(
                world,
                config.carla.target_blueprint,
                "aeb_target",
                target_waypoint,
            )
            return ego, target, route
        except RuntimeError as exc:
            last_error = exc
            if ego is not None and ego.is_alive:
                ego.destroy()
    raise RuntimeError(
        "no Town10 spawn point provided both a clear two-vehicle placement "
        f"and a {required_route_m:.1f} m route"
    ) from last_error


def _controller_objects(config: AppConfig, controller_name: str) -> tuple[Any, ...]:
    controls = config.controllers
    lateral = PurePursuitController(
        wheelbase_m=controls.wheelbase_m,
        wheel_max_angle_deg=controls.wheel_max_angle_deg,
        lookahead_base_m=controls.lookahead_base_m,
        lookahead_gain_s=controls.lookahead_gain_s,
        max_steer=controls.max_steer,
    )
    speed_args = dict(
        kp=controls.speed_kp,
        ki=controls.speed_ki,
        kd=controls.speed_kd,
        max_throttle=controls.max_throttle,
        service_brake_decel_mps2=controls.service_brake_decel_mps2,
    )
    supervisor = SafetySupervisor(
        fcw_ttc_s=controls.fcw_ttc_s,
        aeb_ttc_s=controls.aeb_ttc_s,
        distance_margin_m=controls.aeb_distance_margin_m,
        max_decel_mps2=controls.max_aeb_decel_mps2,
    )
    if controller_name == "pid":
        emergency = AEBPID(
            kp=controls.aeb_pid_kp,
            ki=controls.aeb_pid_ki,
            kd=controls.aeb_pid_kd,
            target_decel_mps2=controls.aeb_pid_target_decel_mps2,
            max_decel_mps2=controls.max_aeb_decel_mps2,
        )
    else:
        emergency = AEBMPC(
            max_decel_mps2=controls.max_aeb_decel_mps2,
            horizon_s=controls.mpc_horizon_s,
            step_s=controls.mpc_step_s,
        )
    return lateral, LongitudinalPID(**speed_args), LongitudinalPID(**speed_args), supervisor, emergency


def run(config: AppConfig, controller_name: str, output_path: Path) -> int:
    try:
        import carla
    except ImportError as exc:
        raise RuntimeError("CARLA Python API 0.10.0 is not importable in this environment") from exc

    client = carla.Client(config.carla.host, config.carla.port)
    client.set_timeout(config.carla.timeout_s)
    world = client.get_world()
    map_name = world.get_map().name
    if not map_name.endswith(config.carla.map):
        raise RuntimeError(f"expected {config.carla.map}, but CARLA loaded {map_name}")

    original_settings = world.get_settings()
    actors: list[Any] = []
    logger: ScenarioCsvLogger | None = None
    try:
        if config.carla.synchronous_mode:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = config.carla.fixed_delta_seconds
            world.apply_settings(settings)

        ego, target, route = _spawn_scenario_actors(world, config)
        actors.append(ego)
        actors.append(target)

        route_points = [
            RoutePoint(point.transform.location.x, point.transform.location.y)
            for point in route
        ]
        lateral, ego_speed_pid, target_speed_pid, supervisor, emergency = _controller_objects(
            config, controller_name
        )
        scenario = ScenarioOrchestrator(
            ego_target_mph=config.scenario.ego_speed_mph,
            target_target_mph=config.scenario.target_speed_mph,
            stable_tolerance_mph=config.scenario.stable_tolerance_mph,
            stable_hold_s=config.scenario.stable_hold_s,
            stop_speed_mph=config.scenario.stop_speed_mph,
            stop_hold_s=config.scenario.stop_hold_s,
            timeout_s=config.scenario.timeout_s,
        )
        logger = ScenarioCsvLogger(output_path)
        start_simulation_s = world.get_snapshot().timestamp.elapsed_seconds
        dt_s = config.carla.fixed_delta_seconds
        print(f"Town10 AEB | controller={controller_name.upper()} | log={output_path}")

        while not scenario.finished:
            if config.carla.synchronous_mode:
                world.tick()
                snapshot = world.get_snapshot()
            else:
                snapshot = world.wait_for_tick()
                dt_s = max(snapshot.timestamp.delta_seconds, 1.0e-3)

            elapsed_s = snapshot.timestamp.elapsed_seconds - start_simulation_s
            ego_speed = _speed_mps(ego)
            target_speed = _speed_mps(target)
            center_distance = ego.get_location().distance(target.get_location())
            gap_m = max(0.0, center_distance - ego.bounding_box.extent.x - target.bounding_box.extent.x)
            assessment = supervisor.evaluate(ego_speed, target_speed, gap_m)
            phase = scenario.update(
                ScenarioObservation(
                    elapsed_s=elapsed_s,
                    ego_speed_mph=mps_to_mph(ego_speed),
                    target_speed_mph=mps_to_mph(target_speed),
                    aeb_active=assessment.aeb,
                )
            )

            transform = ego.get_transform()
            steer = lateral.compute(
                Pose2D(transform.location.x, transform.location.y, math.radians(transform.rotation.yaw)),
                ego_speed,
                route_points,
            )
            if assessment.aeb:
                if controller_name == "pid":
                    brake = emergency.compute(actual_accel_mps2=_acceleration_mps2(ego), dt_s=dt_s)
                else:
                    brake = emergency.compute(
                        ego_speed_mps=ego_speed,
                        target_speed_mps=target_speed,
                        gap_m=gap_m,
                    )
                ego_demand = ControlDemand(throttle=0.0, brake=brake)
            else:
                ego_demand = ego_speed_pid.compute(mph_to_mps(config.scenario.ego_speed_mph), ego_speed, dt_s)

            if scenario.target_hard_brake:
                target_demand = ControlDemand(throttle=0.0, brake=config.scenario.target_brake)
            else:
                target_demand = target_speed_pid.compute(
                    mph_to_mps(config.scenario.target_speed_mph), target_speed, dt_s
                )

            target_transform = target.get_transform()
            target_steer = lateral.compute(
                Pose2D(
                    target_transform.location.x,
                    target_transform.location.y,
                    math.radians(target_transform.rotation.yaw),
                ),
                target_speed,
                route_points,
            )
            ego.apply_control(
                carla.VehicleControl(
                    throttle=ego_demand.throttle,
                    brake=ego_demand.brake,
                    steer=steer,
                )
            )
            target.apply_control(
                carla.VehicleControl(
                    throttle=target_demand.throttle,
                    brake=target_demand.brake,
                    steer=target_steer,
                )
            )
            logger.write(
                time_s=f"{elapsed_s:.3f}",
                phase=phase.value,
                ego_speed_mph=f"{mps_to_mph(ego_speed):.3f}",
                target_speed_mph=f"{mps_to_mph(target_speed):.3f}",
                distance_m=f"{gap_m:.3f}",
                closing_speed_mph=f"{mps_to_mph(assessment.closing_speed_mps):.3f}",
                ttc_s="inf" if math.isinf(assessment.ttc_s) else f"{assessment.ttc_s:.3f}",
                fcw=int(assessment.fcw),
                aeb_active=int(assessment.aeb),
                brake_percent=f"{100.0 * ego_demand.brake:.2f}",
                ego_steer=f"{steer:.4f}",
                ego_accel_mps2=f"{_acceleration_mps2(ego):.4f}",
            )

        print(f"Scenario ended in phase {scenario.phase.value}")
        if config.scenario.retain_actors_s > 0.0:
            print(f"Retaining actors for {config.scenario.retain_actors_s:.1f} s for visual inspection")
            time.sleep(config.scenario.retain_actors_s)
        return 0 if scenario.phase.value == "COMPLETE" else 2
    finally:
        if logger is not None:
            logger.close()
        for actor in reversed(actors):
            if actor.is_alive:
                actor.destroy()
        if config.carla.synchronous_mode:
            world.apply_settings(original_settings)


def default_output_path(config: AppConfig, controller_name: str) -> Path:
    """Return the one stable result path assigned to each AEB controller."""
    return Path(config.logging.output_directory) / f"town10_aeb_{controller_name}.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the modular Town10 AEB scenario")
    parser.add_argument("--controller", choices=("pid", "mpc", "both"), default="both")
    parser.add_argument("--config", default="config/town10_adas.json")
    parser.add_argument(
        "--output",
        default=None,
        help="custom CSV path for a single PID or MPC run; unavailable with --controller both",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if args.controller == "both":
        if args.output is not None:
            parser.error("--output cannot be used with --controller both")
        results = [
            run(config, controller_name, default_output_path(config, controller_name))
            for controller_name in ("pid", "mpc")
        ]
        return max(results)

    output = Path(args.output) if args.output else default_output_path(config, args.controller)
    return run(config, args.controller, output)


if __name__ == "__main__":
    raise SystemExit(main())
