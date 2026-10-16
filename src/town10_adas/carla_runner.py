"""CARLA 0.10.0 adapter for the modular Town10 ADAS stack."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import time
from typing import Any, Callable, Sequence

from .aeb import AEBMPC, AEBPID
from .config import AppConfig, load_config
from .csv_logger import ScenarioCsvLogger
from .lateral import Pose2D, PurePursuitController, RoutePoint
from .longitudinal import ControlDemand, LongitudinalPID
from .provenance import write_carla_metadata
from .safety import SafetySupervisor
from .scenario import ScenarioObservation, ScenarioOrchestrator, ScenarioPhase
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


def _spawn_scenario_actors(world: Any, config: AppConfig, spawn_index: int = 0) -> tuple[Any, Any, list[Any]]:
    """Use the same explicit spawn index in each comparison; never fall back."""
    spawn_allowance_m = config.scenario.initial_gap_m + 8.0
    required_route_m = config.scenario.route_minimum_m + spawn_allowance_m
    last_error: Exception | None = None
    spawn_points = world.get_map().get_spawn_points()
    if not 0 <= spawn_index < len(spawn_points):
        raise ValueError(f"spawn_index must be in [0, {len(spawn_points) - 1}]")
    for spawn_transform in spawn_points[spawn_index:spawn_index + 1]:
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
        f"spawn index {spawn_index} did not provide a clear two-vehicle placement "
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


def _pose_evidence(actor: Any) -> dict[str, float]:
    transform = actor.get_transform()
    return {name: float(getattr(transform.location, name)) for name in ("x", "y", "z")} | {
        name: float(getattr(transform.rotation, name)) for name in ("pitch", "yaw", "roll")}


def _route_lateral_error(location: Any, route: Sequence[RoutePoint]) -> float:
    """Distance to the intended route polyline, independent of adjacent road lanes."""
    minimum = math.inf
    for first, second in zip(route, route[1:]):
        dx, dy = second.x_m - first.x_m, second.y_m - first.y_m
        length_sq = dx * dx + dy * dy
        if length_sq == 0:
            continue
        fraction = max(0.0, min(1.0, ((location.x - first.x_m) * dx
                                     + (location.y - first.y_m) * dy) / length_sq))
        minimum = min(minimum, math.hypot(location.x - first.x_m - fraction * dx,
                                        location.y - first.y_m - fraction * dy))
    return minimum


def run(config: AppConfig, controller_name: str, output_path: Path, spawn_index: int = 0,
        *, video_path: Path | None = None, reload_world: bool = False,
        preview_callback: Callable[[Any, dict[str, Any]], None] | None = None,
        video_overlay: bool = False) -> int:
    if controller_name not in {"pid", "mpc"}:
        raise ValueError("controller must be pid or mpc")
    if preview_callback is not None and video_path is None:
        raise ValueError("live preview requires video capture")
    if video_path is not None:
        for destination in (output_path, output_path.with_suffix(".metadata.json"), video_path):
            if destination.exists():
                raise FileExistsError(f"output already exists; choose a fresh output directory: {destination}")
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
    video = None
    collision_sensor = None
    collision_events: list[dict[str, Any]] = []
    evidence: dict[str, Any] = {
        "completed": False, "outcome": "not_started", "collision_count": None,
        "collision_sensor_attached": False, "map": map_name,
        "versions": {"client": client.get_client_version(), "server": client.get_server_version()},
    }
    try:
        logger = ScenarioCsvLogger(output_path)
        if config.carla.synchronous_mode:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = config.carla.fixed_delta_seconds
            world.apply_settings(settings)
        if reload_world:
            world = client.reload_world(reset_settings=False)

        evidence["world_reloaded"] = reload_world

        ego, target, route = _spawn_scenario_actors(world, config, spawn_index)
        actors.append(ego)
        actors.append(target)
        evidence["spawns"] = {"ego": _pose_evidence(ego), "lead": _pose_evidence(target)}
        evidence["route"] = {
            "spawn_index": spawn_index,
            "waypoints": [[p.road_id, p.lane_id, float(p.s),
                           float(p.transform.location.x), float(p.transform.location.y)] for p in route],
        }
        settings = world.get_settings()
        evidence["world_settings"] = {"synchronous_mode": settings.synchronous_mode,
                                     "fixed_delta_seconds": settings.fixed_delta_seconds}
        weather = world.get_weather()
        evidence["weather"] = {name: float(getattr(weather, name)) for name in (
            "cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
            "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "wetness")}
        evidence["gap_measurement"] = "Euclidean center distance minus longitudinal half-lengths; curved-road approximation"
        collision_sensor = world.spawn_actor(
            world.get_blueprint_library().find("sensor.other.collision"), carla.Transform(), attach_to=ego)
        actors.append(collision_sensor)
        collision_sensor.listen(lambda event: collision_events.append({
            "frame": int(event.frame), "other_actor_type": event.other_actor.type_id}))
        evidence["collision_sensor_attached"] = True
        if video_path is not None:
            from .video import CarlaVideoRecorder
            video = CarlaVideoRecorder(world, ego, video_path,
                                       fixed_delta_seconds=config.carla.fixed_delta_seconds)

        # Let the spawned vehicles settle onto the road before recording observations.
        for actor in (ego, target):
            actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
        for _ in range(20):
            if config.carla.synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()
        for actor in (ego, target):
            actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=False))
        evidence["settle_ticks"] = 20
        if config.scenario.start_at_cruise_speed:
            for actor, speed_mph, gear, throttle in (
                (ego, config.scenario.ego_speed_mph, config.scenario.rolling_ego_gear,
                 config.scenario.rolling_ego_throttle),
                (target, config.scenario.target_speed_mph, config.scenario.rolling_target_gear,
                 config.scenario.rolling_target_throttle),
            ):
                actor.apply_control(carla.VehicleControl(throttle=throttle, brake=0.0,
                                                        manual_gear_shift=gear > 0, gear=gear))
                forward = actor.get_transform().get_forward_vector()
                speed = mph_to_mps(speed_mph)
                actor.set_target_velocity(carla.Vector3D(x=forward.x * speed,
                                                        y=forward.y * speed,
                                                        z=forward.z * speed))
            # Exclude the one-time velocity initialization from acceleration measurements.
            if config.carla.synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()
        evidence["initialization"] = {
            "mode": "rolling_start" if config.scenario.start_at_cruise_speed else "from_rest",
            "warmup_ticks": 1 if config.scenario.start_at_cruise_speed else 0,
            "description": ("Velocity set once after settling; subsequent motion uses CARLA physics and controllers"
                            if config.scenario.start_at_cruise_speed else "Vehicles accelerate from rest"),
        }
        evidence["lane_tracking"] = {"max_ego_error_m": 0.0, "max_target_error_m": 0.0,
                                     "abort_error_m": 1.5}

        route_points = [
            RoutePoint(point.transform.location.x, point.transform.location.y)
            for point in route
        ]
        lateral, ego_speed_pid, target_speed_pid, supervisor, emergency = _controller_objects(
            config, controller_name
        )
        rolling = config.scenario.start_at_cruise_speed
        if rolling:
            ego_speed_pid.reset(initial_throttle=config.scenario.rolling_ego_throttle)
            target_speed_pid.reset(initial_throttle=config.scenario.rolling_target_throttle)
        scenario = ScenarioOrchestrator(
            ego_target_mph=config.scenario.ego_speed_mph,
            target_target_mph=config.scenario.target_speed_mph,
            stable_tolerance_mph=config.scenario.stable_tolerance_mph,
            stable_hold_s=config.scenario.stable_hold_s,
            stop_speed_mph=config.scenario.stop_speed_mph,
            stop_hold_s=config.scenario.stop_hold_s,
            timeout_s=config.scenario.timeout_s,
        )
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
            ego_lane_error = _route_lateral_error(ego.get_location(), route_points)
            target_lane_error = _route_lateral_error(target.get_location(), route_points)
            for key, error in (("max_ego_error_m", ego_lane_error),
                               ("max_target_error_m", target_lane_error)):
                evidence["lane_tracking"][key] = max(evidence["lane_tracking"][key], error)
            center_distance = ego.get_location().distance(target.get_location())
            gap_m = center_distance - ego.bounding_box.extent.x - target.bounding_box.extent.x
            assessment = supervisor.evaluate(ego_speed, target_speed, gap_m)
            lead_braking_before_tick = scenario.target_hard_brake
            phase = scenario.update(
                ScenarioObservation(
                    elapsed_s=elapsed_s,
                    ego_speed_mph=mps_to_mph(ego_speed),
                    target_speed_mph=mps_to_mph(target_speed),
                    aeb_active=assessment.aeb,
                )
            )
            if collision_events or gap_m <= 0:
                scenario.phase = phase = ScenarioPhase.ABORTED
                evidence["termination_reason"] = "contact_or_gap_overlap"
            elif max(ego_lane_error, target_lane_error) > evidence["lane_tracking"]["abort_error_m"]:
                scenario.phase = phase = ScenarioPhase.ABORTED
                evidence["termination_reason"] = "left_intended_route"
            elif assessment.aeb and not lead_braking_before_tick:
                scenario.phase = phase = ScenarioPhase.ABORTED
                evidence["termination_reason"] = "aeb_before_scheduled_threat"

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
                if "lead_brake_event" not in evidence:
                    evidence["lead_brake_event"] = {
                        "time_s": elapsed_s, "ego_speed_mph": mps_to_mph(ego_speed),
                        "target_speed_mph": mps_to_mph(target_speed), "gap_m": gap_m,
                        "brake_percent": 100.0 * config.scenario.target_brake,
                    }
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
            if scenario.finished:
                ego_demand = target_demand = ControlDemand(throttle=0.0, brake=1.0)
            ego.apply_control(
                carla.VehicleControl(
                    throttle=ego_demand.throttle,
                    brake=ego_demand.brake,
                    steer=steer,
                    manual_gear_shift=rolling and config.scenario.rolling_ego_gear > 0,
                    gear=config.scenario.rolling_ego_gear if rolling else 0,
                )
            )
            target.apply_control(
                carla.VehicleControl(
                    throttle=target_demand.throttle,
                    brake=target_demand.brake,
                    steer=target_steer,
                    manual_gear_shift=rolling and config.scenario.rolling_target_gear > 0,
                    gear=config.scenario.rolling_target_gear if rolling else 0,
                )
            )
            sample = dict(
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
            logger.write(**sample)
            if video is not None:
                preview_sample = sample | {
                    "target_brake_percent": f"{100.0 * target_demand.brake:.2f}",
                    "target_accel_mps2": f"{_acceleration_mps2(target):.4f}",
                    "lane_error_m": f"{ego_lane_error:.3f}",
                }
                image = video.capture(snapshot.frame, telemetry=preview_sample if video_overlay else None,
                                      controller=controller_name)
                if preview_callback is not None:
                    preview_callback(image, preview_sample)

        print(f"Scenario ended in phase {scenario.phase.value}")
        evidence["outcome"] = scenario.phase.value
        evidence["completed"] = scenario.phase == ScenarioPhase.COMPLETE
        if config.scenario.retain_actors_s > 0.0:
            print(f"Retaining actors for {config.scenario.retain_actors_s:.1f} s for visual inspection")
            time.sleep(config.scenario.retain_actors_s)
        return 0 if scenario.phase.value == "COMPLETE" else 2
    except BaseException as exc:
        evidence["outcome"] = "exception"
        evidence["failure"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if logger is not None:
            logger.close()
        try:
            if video is not None:
                try:
                    video.close()
                except Exception as exc:
                    evidence["completed"] = False
                    evidence["video_failure"] = f"{type(exc).__name__}: {exc}"
                    raise
                finally:
                    evidence["video"] = video.metadata
        finally:
            _cleanup_run(world, original_settings, config, actors, collision_sensor,
                         collision_events, logger, output_path, controller_name, evidence)


def _cleanup_run(world: Any, original_settings: Any, config: AppConfig, actors: list[Any],
                 collision_sensor: Any, collision_events: list[dict[str, Any]],
                 logger: ScenarioCsvLogger | None, output_path: Path,
                 controller_name: str, evidence: dict[str, Any]) -> None:
    try:
        if collision_sensor is not None:
            collision_sensor.stop()
    finally:
        try:
            for actor in reversed(actors):
                if actor.is_alive:
                    actor.destroy()
        finally:
            try:
                if config.carla.synchronous_mode:
                    world.apply_settings(original_settings)
            finally:
                if logger is not None:
                    evidence["collision_events"] = list(collision_events)
                    evidence["collision_count"] = (len(collision_events)
                                                   if evidence["collision_sensor_attached"] else None)
                    write_carla_metadata(output_path, config, controller_name, evidence)


def default_output_path(config: AppConfig, controller_name: str) -> Path:
    """Return the one stable result path assigned to each AEB controller."""
    return Path(config.logging.output_directory) / f"town10_aeb_{controller_name}.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the modular Town10 AEB scenario")
    parser.add_argument("--controller", choices=("pid", "mpc", "both"), default="both")
    parser.add_argument("--config", default="config/town10_adas.json")
    parser.add_argument("--host", help="CARLA address: default is the config's published Docker host port")
    parser.add_argument("--port", type=int, help="override CARLA RPC port")
    parser.add_argument("--spawn-index", type=int, default=0,
                        help="fixed map spawn index shared by both controllers; no automatic fallback")
    parser.add_argument("--output-dir", help="directory for fresh run CSV, metadata, and video files")
    parser.add_argument("--video", action="store_true", help="record a synchronized RGB chase-camera MP4")
    parser.add_argument("--reload-world", action="store_true",
                        help="reset the isolated CARLA world before each controller run")
    parser.add_argument(
        "--output",
        default=None,
        help="custom CSV path for a single PID or MPC run; unavailable with --controller both",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if args.output_dir:
        config = replace(config, logging=replace(config.logging, output_directory=args.output_dir))
    if args.video and not config.carla.synchronous_mode:
        parser.error("video recording requires synchronous_mode=true")
    if args.host is not None:
        if not args.host.strip():
            parser.error("--host must not be empty")
        config = replace(config, carla=replace(config.carla, host=args.host))
    if args.port is not None:
        if not 1 <= args.port <= 65535:
            parser.error("--port must be between 1 and 65535")
        config = replace(config, carla=replace(config.carla, port=args.port))
    if args.controller == "both":
        if not config.carla.synchronous_mode:
            parser.error("paired comparison requires synchronous_mode=true")
        if args.output is not None:
            parser.error("--output cannot be used with --controller both")
        results = [
            run(config, controller_name, default_output_path(config, controller_name), args.spawn_index,
                video_path=default_output_path(config, controller_name).with_suffix(".mp4") if args.video else None,
                reload_world=args.reload_world)
            for controller_name in ("pid", "mpc")
        ]
        return max(results)

    output = Path(args.output) if args.output else default_output_path(config, args.controller)
    return run(config, args.controller, output, args.spawn_index,
               video_path=output.with_suffix(".mp4") if args.video else None,
               reload_world=args.reload_world)


if __name__ == "__main__":
    raise SystemExit(main())
