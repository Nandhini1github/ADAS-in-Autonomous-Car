"""Scenario 2: a reversing car and a child crossing a Town10 parking space.

Rear RGB detection and depth pixels drive AEB. Actor state is used for scene choreography,
ego odometry, and independent outcome measurements, never pedestrian detection.
Requires the packaged simulator and Python API; no CARLA source build is needed.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import queue
import time
from typing import Any

from .longitudinal import LongitudinalPID
from .rear_perception import RearAEB, detect_pedestrians

ROOT = Path(__file__).resolve().parents[2]
FIELDS = (
    "frame", "time_s", "phase", "signed_speed_mps", "reverse_speed_mps",
    "reverse_distance_m", "reverse_gear", "pedestrian_visible", "pedestrian_count",
    "rear_clearance_m", "lateral_m", "ttc_s", "stopping_distance_m",
    "aeb_active", "aeb_reason", "throttle", "brake", "child_crossing",
    "child_crossed_path", "child_finished", "measured_child_speed_mps", "collision_count",
    "gt_rear_clearance_m", "gt_child_lateral_m",
)


def load_config(path: Path) -> dict[str, Any]:
    config = json.loads(path.read_text(encoding="utf-8"))
    if config.get("perception", "rgb_depth") not in {"rgb_depth", "instance_depth"}:
        raise ValueError("perception must be rgb_depth or instance_depth")
    if config["map"] not in {"Town10HD", "Town10HD_Opt"}:
        raise ValueError("Scenario 2 parking coordinates require Town10HD or Town10HD_Opt")
    for key in ("fixed_delta_seconds", "reverse_speed_mps", "child_speed_mps",
                "crossing_trigger_distance_m", "maximum_reverse_distance_m", "timeout_s",
                "stop_speed_mps", "stop_hold_s"):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    if not 0.01 <= config["fixed_delta_seconds"] <= 0.1:
        raise ValueError("fixed_delta_seconds must be between .01 and .1")
    if config["crossing_trigger_distance_m"] >= config["maximum_reverse_distance_m"]:
        raise ValueError("crossing trigger must precede reverse travel limit")
    camera = config["camera"]
    for key in ("width", "height"):
        if type(camera[key]) is not int or camera[key] < 64 or camera[key] % 2:
            raise ValueError(f"camera {key} must be an even integer >= 64")
    for key in ("fov_deg", "height_m", "bumper_offset_m", "max_range_m"):
        if type(camera[key]) not in (int, float) or not math.isfinite(camera[key]) or camera[key] <= 0:
            raise ValueError(f"camera {key} must be positive and finite")
    if not 30 < camera["fov_deg"] < 170 or not camera["bumper_offset_m"] < camera["max_range_m"] < 1000:
        raise ValueError("invalid rear camera FOV/range")
    for name in ("ego", "child", "child_end"):
        for key in ("x", "y", "z") + (() if name == "child_end" else ("yaw",)):
            value = config["parking_spot"][name][key]
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{name}.{key} must be finite")
    start, end = config["parking_spot"]["child"], config["parking_spot"]["child_end"]
    if math.hypot(end["x"] - start["x"], end["y"] - start["y"]) < 1.0:
        raise ValueError("child crossing path must be at least one meter")
    if "ego_half_width_m" in config["aeb"]:
        raise ValueError("ego_half_width_m is measured from the spawned vehicle")
    confidence = config.get("detector_confidence", 0.35)
    if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 < confidence < 1:
        raise ValueError("detector_confidence must be between zero and one")
    walker_scale = config.get("walker_speed_scale", 1.0)
    if type(walker_scale) not in (int, float) or not math.isfinite(walker_scale) or walker_scale <= 0:
        raise ValueError("walker_speed_scale must be positive and finite")
    RearAEB(**config["aeb"])  # Validate before touching the simulator.
    return config


def frame_from_queue(frames: queue.Queue, frame: int, timeout_s: float = 15.0) -> Any:
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(f"camera did not deliver simulation frame {frame}")
        try:
            image = frames.get(timeout=remaining)
        except queue.Empty as exc:
            raise TimeoutError(f"camera did not deliver simulation frame {frame}") from exc
        if image.frame < frame:
            continue
        if image.frame != frame:
            raise RuntimeError(f"camera skipped frame {frame}; received {image.frame}")
        return image


def _pose(actor: Any) -> dict[str, float]:
    transform = actor.get_transform()
    return {key: float(getattr(transform.location, key)) for key in ("x", "y", "z")} | {
        "yaw": float(transform.rotation.yaw)}


def _ground_truth(ego: Any, child: Any) -> tuple[float, float]:
    """Evaluation only: project box extents to measure signed rear clearance."""
    et, ct = ego.get_transform(), child.get_transform()
    ec = et.transform(ego.bounding_box.location)
    cc = ct.transform(child.bounding_box.location)
    forward, right = et.get_forward_vector(), et.get_right_vector()
    delta = cc - ec
    cforward, cright = ct.get_forward_vector(), ct.get_right_vector()
    child_extent = (abs(cforward.dot(forward)) * child.bounding_box.extent.x
                    + abs(cright.dot(forward)) * child.bounding_box.extent.y)
    clearance = -delta.dot(forward) - ego.bounding_box.extent.x - child_extent
    return clearance, delta.dot(right)


def _spawn_camera(world: Any, carla: Any, kind: str, transform: Any, camera: dict,
                  sensors: list, *, ego: Any = None) -> queue.Queue:
    bp = world.get_blueprint_library().find(kind)
    for key, value in {"image_size_x": camera["width"], "image_size_y": camera["height"],
                       "fov": camera["fov_deg"], "sensor_tick": 0.0,
                       "lens_k": 0.0, "lens_kcube": 0.0}.items():
        bp.set_attribute(key, str(value))
    kwargs = {"attach_to": ego, "attachment_type": carla.AttachmentType.Rigid} if ego else {}
    sensor = world.spawn_actor(bp, transform, **kwargs)
    sensors.append(sensor)
    frames: queue.Queue = queue.Queue(maxsize=16)

    def receive(image):
        try:
            frames.put_nowait(image)
        except queue.Full:
            try:
                frames.get_nowait()
            except queue.Empty:
                pass
            try:
                frames.put_nowait(image)
            except queue.Full:
                pass

    sensor.listen(receive)
    return frames


def run(config: dict, output: Path, *, host: str = "127.0.0.1", port: int = 2020,
        video: bool = True, load_map: bool = False, model_path: Path | None = None) -> int:
    import carla

    output.mkdir(parents=True, exist_ok=False)
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    client = carla.Client(host, port)
    client.set_timeout(20.0)
    world = client.get_world()
    dynamic = [a for a in world.get_actors() if a.type_id.startswith(("vehicle.", "walker.", "sensor."))]
    if dynamic:
        raise RuntimeError("Scenario 2 requires an idle server; active vehicles/walkers/sensors are present")
    if world.get_map().name.rsplit("/", 1)[-1] != config["map"]:
        if not load_map:
            raise RuntimeError(f"expected {config['map']}; use --load-map to load it on this idle server")
        world = client.load_world(config["map"])
    original_settings, original_weather = world.get_settings(), world.get_weather()
    if original_settings.synchronous_mode:
        raise RuntimeError("server is already synchronous; stop its controlling client first")
    actors: list[Any] = []
    sensors: list[Any] = []
    collisions: list[dict] = []
    recorder = None
    ego = child = None
    dt = config["fixed_delta_seconds"]
    evidence: dict[str, Any] = {
        "schema_version": 1, "scenario": config["scenario"], "source": "carla",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "completed": False, "outcome": "not_started", "map": world.get_map().name,
        "versions": {"client": client.get_client_version(), "server": client.get_server_version()},
        "perception": config.get("perception", "rgb_depth"),
        "pedestrian_actor_state_used_for_braking": False,
        "config": config, "events": [], "collision_events": collisions,
        "row_count": 0, "cleanup_errors": [],
    }

    def event(name, t, **details):
        evidence["events"].append({"name": name, "time_s": round(t, 4), **details})
        print(f"{t:6.2f}s {name} {details}", flush=True)

    try:
        detector = None
        if evidence["perception"] == "rgb_depth":
            from .rear_rgb import RearRGBDetector
            model_path = model_path or ROOT / ".runtime/models/object_detection_yolox_2022nov.onnx"
            if not model_path.is_file():
                raise FileNotFoundError("run python scripts/download_rear_detector.py before Scenario 2")
            detector = RearRGBDetector(model_path, config["camera"]["fov_deg"],
                camera_to_bumper_m=config["camera"]["bumper_offset_m"],
                max_range_m=config["camera"]["max_range_m"], confidence=config.get("detector_confidence", 0.35))
            evidence["detector"] = {"name": "OpenCV Zoo YOLOX-s COCO person", "path": str(model_path),
                "sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
                "confidence": config.get("detector_confidence", 0.35)}
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        settings.no_rendering_mode = False
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        world.apply_settings(settings)
        world.set_weather(carla.WeatherParameters.ClearNoon)
        evidence["world_settings"] = {"synchronous_mode": True, "fixed_delta_seconds": dt,
                                      "weather": "ClearNoon"}
        library = world.get_blueprint_library()
        child_bp = library.find(config["child_blueprint"])
        if not child_bp.has_attribute("age") or child_bp.get_attribute("age").as_str() != "child":
            raise RuntimeError("configured walker is not a runtime-verified child blueprint")
        child_bp.set_attribute("is_invincible", "false")
        ego_bp = library.find(config["ego_blueprint"])
        ego_bp.set_attribute("role_name", "scenario_2_reverse_ego")

        def transform(pose):
            return carla.Transform(carla.Location(**{k: pose[k] for k in ("x", "y", "z")}),
                                   carla.Rotation(yaw=pose.get("yaw", 0.0)))

        for bp, pose in ((ego_bp, config["parking_spot"]["ego"]),
                         (child_bp, config["parking_spot"]["child"])):
            actor = world.try_spawn_actor(bp, transform(pose))
            if actor is None:
                raise RuntimeError(f"parking placement blocked for {bp.id}; no fallback spawn is used")
            actors.append(actor)
        ego, child = actors
        ego.apply_control(carla.VehicleControl(brake=1.0, reverse=True, manual_gear_shift=False, gear=-1))
        child.apply_control(carla.WalkerControl(speed=0.0))
        collision = world.spawn_actor(library.find("sensor.other.collision"), carla.Transform(), attach_to=ego)
        sensors.append(collision)
        collision.listen(lambda e: collisions.append({
            "frame": e.frame, "other_actor_id": e.other_actor.id, "other_actor_type": e.other_actor.type_id,
            "impulse": math.sqrt(e.normal_impulse.x**2 + e.normal_impulse.y**2 + e.normal_impulse.z**2)}))
        for _ in range(30):
            world.tick()
        if collisions:
            raise RuntimeError("collision during placement/settling")
        evidence["spawns"] = {"ego": _pose(ego), "child": _pose(child)}
        evidence["child"] = {"blueprint": child.type_id, "age": "child", "actor_id": child.id}
        evidence["child"]["requested_speed_mps"] = config["child_speed_mps"]
        evidence["child"]["walker_control_speed"] = config["child_speed_mps"] * config.get("walker_speed_scale", 1.0)
        evidence["child"]["speed_scale_reason"] = "CARLA 0.10 walker input divides by 4096 cm/s; child movement cap is 200 cm/s"
        evidence["ego"] = {"blueprint": ego.type_id, "half_length_m": ego.bounding_box.extent.x,
                           "half_width_m": ego.bounding_box.extent.y}
        waypoint = world.get_map().get_waypoint(ego.get_location(), lane_type=carla.LaneType.Any)
        evidence["parking_lane"] = {"road_id": waypoint.road_id, "lane_id": waypoint.lane_id,
                                    "lane_type": str(waypoint.lane_type), "s": waypoint.s}
        camera = config["camera"]
        rear_x = ego.bounding_box.location.x - ego.bounding_box.extent.x - camera["bumper_offset_m"]
        rear_pose = carla.Transform(carla.Location(x=rear_x, z=camera["height_m"]), carla.Rotation(yaw=180.0))
        evidence["rear_camera"] = {**camera, "x_m": rear_x, "yaw_deg": 180.0, "lens_k": 0.0, "lens_kcube": 0.0}
        kinds = [("rgb", "sensor.camera.rgb"), ("depth", "sensor.camera.depth")]
        if detector is None:
            kinds.append(("instance", "sensor.camera.instance_segmentation"))
        queues = {name: _spawn_camera(world, carla, kind, rear_pose, camera, sensors, ego=ego)
                  for name, kind in kinds}
        if video:
            from .rear_video import RearVideoRecorder
            overview_pose = carla.Transform(carla.Location(x=19.0, y=140.0, z=12.0),
                                            carla.Rotation(pitch=-37.0, yaw=-123.0))
            queues["overview"] = _spawn_camera(world, carla, "sensor.camera.rgb", overview_pose,
                                               {**camera, "fov_deg": 75.0}, sensors)
            recorder = RearVideoRecorder(output / "rear_aeb.mp4", camera["width"], camera["height"], dt)
        start_pose = ego.get_transform()
        start_forward = start_pose.get_forward_vector()
        start_right = start_pose.get_right_vector()
        child_start = child.get_location()
        child_end = carla.Location(**config["parking_spot"]["child_end"])
        direction = child_end - child_start
        direction.z = 0
        length = math.hypot(direction.x, direction.y)
        if length <= 0:
            raise ValueError("child crossing path must have nonzero length")
        direction = carla.Vector3D(x=direction.x / length, y=direction.y / length)
        initial_lateral = (child_start - ego.get_location()).dot(start_right)
        supervisor = RearAEB(ego_half_width_m=ego.bounding_box.extent.y, **config["aeb"])
        speed_pid = LongitudinalPID(kp=0.3, ki=0.08, kd=0.0, max_throttle=0.3)
        crossing = crossed = finished = False
        active_previous = False
        first_visible = False
        stopped_s = 0.0
        stop_recorded = False
        min_clearance = math.inf
        start_time = world.get_snapshot().timestamp.elapsed_seconds
        evidence["physics_wakeup"] = {"speed_mps": 0.1, "assignments": 1,
                                      "reason": "CARLA 0.10 Chaos sleeping-body workaround at reverse launch"}
        ego.apply_control(carla.VehicleControl(reverse=True, manual_gear_shift=False, gear=-1))
        ego.set_target_velocity(carla.Vector3D(x=-start_forward.x * 0.1, y=-start_forward.y * 0.1))
        evidence["outcome"] = "timeout"
        with (output / "rear_aeb.csv").open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDS)
            writer.writeheader()
            for _ in range(math.ceil(config["timeout_s"] / dt)):
                frame = world.tick()
                snapshot = world.get_snapshot()
                t = snapshot.timestamp.elapsed_seconds - start_time
                images = {name: frame_from_queue(q, frame) for name, q in queues.items()}
                et = ego.get_transform()
                velocity = ego.get_velocity()
                signed_speed = velocity.dot(et.get_forward_vector())
                total_speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
                speed = max(0.0, -signed_speed)
                reverse_distance = -(et.location - start_pose.location).dot(start_forward)
                lateral_drift = abs((et.location - start_pose.location).dot(start_right))
                yaw_drift = abs((et.rotation.yaw - start_pose.rotation.yaw + 180) % 360 - 180)
                if not crossing and reverse_distance >= config["crossing_trigger_distance_m"]:
                    crossing = True
                    event("child_started_crossing", t, reverse_distance_m=reverse_distance)
                if crossing and not finished:
                    progress = (child.get_location() - child_start).dot(direction)
                    finished = progress >= length - 0.15
                    if finished:
                        event("child_finished_crossing", t)
                    child.apply_control(carla.WalkerControl(direction=direction,
                        speed=0.0 if finished else config["child_speed_mps"] * config.get("walker_speed_scale", 1.0)))
                gt_clearance, gt_lateral = _ground_truth(ego, child)
                # Ground truth is recorded only for outcome checks/choreography.
                if crossing and not crossed and gt_lateral * initial_lateral < 0:
                    crossed = True
                    event("child_crossed_reverse_centerline", t)
                if abs(gt_lateral) < ego.bounding_box.extent.y + child.bounding_box.extent.y:
                    min_clearance = min(min_clearance, gt_clearance)
                detections = (detector.detect(images["rgb"], images["depth"]) if detector else
                    detect_pedestrians(images["instance"], images["depth"], camera["fov_deg"],
                        camera_to_bumper_m=camera["bumper_offset_m"], max_range_m=camera["max_range_m"]))
                previous_control = ego.get_control()
                decision = supervisor.update(detections, speed_mps=speed,
                    reverse=previous_control.reverse and previous_control.gear == -1, time_s=t)
                nearest = next((d for d in detections if d.instance_id == decision.target_instance_id),
                               detections[0] if detections else None)
                visible = bool(detections)
                if visible and not first_visible:
                    first_visible = True
                    event("pedestrian_visible_in_rear_camera", t)
                if decision.active and not active_previous:
                    event("aeb_activated", t, speed_mps=speed, clearance_m=decision.clearance_m,
                          reason=decision.reason, target_instance_id=decision.target_instance_id,
                          bbox_xyxy=list(nearest.bbox_xyxy) if nearest else None)
                    images["rgb"].save_to_disk(str(output / "rear_at_brake.png"))
                    if "instance" in images:
                        images["instance"].save_to_disk(str(output / "instance_at_brake.png"))
                    images["depth"].save_to_disk(str(output / "depth_at_brake.png"))
                    if video:
                        images["overview"].save_to_disk(str(output / "overview_at_brake.png"))
                active_previous = decision.active
                stopped_s = stopped_s + dt if decision.active and total_speed <= config["stop_speed_mps"] else 0.0
                if stopped_s > 0 and not stop_recorded:
                    stop_recorded = True
                    event("vehicle_stopped", t, speed_mps=total_speed, reverse_distance_m=reverse_distance)
                demand = speed_pid.compute(config["reverse_speed_mps"], speed, dt)
                brake, throttle = (1.0, 0.0) if decision.active else (demand.brake, demand.throttle)
                phase = "BRAKING" if decision.active else "CROSSING" if crossing else "REVERSING"
                if stopped_s >= config["stop_hold_s"]:
                    phase = "HOLDING"
                terminal = None
                if collisions:
                    terminal = "collision"
                elif reverse_distance >= config["maximum_reverse_distance_m"]:
                    terminal = "reverse_travel_limit"
                elif lateral_drift > 0.75 or yaw_drift > 8.0 or signed_speed > 0.25:
                    terminal = "unexpected_vehicle_motion"
                elif decision.active and crossed and finished and stopped_s >= config["stop_hold_s"]:
                    terminal = "completed"
                elif t + dt / 2 >= config["timeout_s"]:
                    terminal = "timeout"
                if terminal:
                    brake, throttle = 1.0, 0.0
                    phase = "COMPLETE" if terminal == "completed" else "ABORTED"
                    evidence["outcome"] = terminal
                ego.apply_control(carla.VehicleControl(throttle=throttle, brake=brake, steer=0.0,
                                                       reverse=True, manual_gear_shift=False, gear=-1))
                row = dict(frame=frame, time_s=round(t, 4), phase=phase,
                    signed_speed_mps=signed_speed, reverse_speed_mps=speed,
                    reverse_distance_m=reverse_distance, reverse_gear=int(previous_control.reverse and previous_control.gear == -1),
                    pedestrian_visible=int(visible), pedestrian_count=len(detections),
                    rear_clearance_m=nearest.clearance_m if nearest else None,
                    lateral_m=nearest.lateral_m if nearest else None, ttc_s=decision.ttc_s,
                    stopping_distance_m=decision.stopping_distance_m, aeb_active=int(decision.active),
                    aeb_reason=decision.reason, throttle=throttle, brake=brake,
                    child_crossing=int(crossing), child_crossed_path=int(crossed), child_finished=int(finished),
                    measured_child_speed_mps=math.sqrt(sum(getattr(child.get_velocity(), k)**2 for k in ("x", "y", "z"))),
                    collision_count=len(collisions), gt_rear_clearance_m=gt_clearance, gt_child_lateral_m=gt_lateral)
                writer.writerow(row)
                evidence["row_count"] += 1
                if recorder:
                    recorder.write(images["rgb"], row, detections, overview_image=images["overview"])
                if terminal:
                    event(terminal, t)
                    break
        evidence["minimum_gt_clearance_during_overlap_m"] = min_clearance if math.isfinite(min_clearance) else None
        evidence["child_crossed_path"] = crossed
        evidence["child_finished"] = finished
        evidence["aeb_triggered"] = supervisor.latched
    except BaseException as exc:
        evidence["outcome"] = "interrupted" if isinstance(exc, KeyboardInterrupt) else "error"
        evidence["error"] = f"{type(exc).__name__}: {exc}"
        print(evidence["error"], flush=True)
    finally:
        if ego is not None and ego.is_alive:
            try:
                ego.apply_control(carla.VehicleControl(brake=1.0, reverse=True, manual_gear_shift=False, gear=-1))
                if child is not None and child.is_alive:
                    child.apply_control(carla.WalkerControl(speed=0.0))
                # Drain final collision callbacks before declaring a successful run.
                for _ in range(5):
                    world.tick()
            except Exception as exc:
                evidence["cleanup_errors"].append(str(exc))
        for sensor in reversed(sensors):
            try:
                sensor.stop()
            except Exception as exc:
                evidence["cleanup_errors"].append(str(exc))
            try:
                sensor.destroy()
            except Exception as exc:
                evidence["cleanup_errors"].append(str(exc))
        if recorder:
            try:
                recorder.close()
            except Exception as exc:
                evidence["cleanup_errors"].append(str(exc))
            evidence["video"] = recorder.metadata
        for actor in reversed(actors):
            try:
                actor.destroy()
            except Exception as exc:
                evidence["cleanup_errors"].append(str(exc))
        try:
            world.set_weather(original_weather)
            world.apply_settings(original_settings)
        except Exception as exc:
            evidence["cleanup_errors"].append(str(exc))
        if collisions:
            evidence["outcome"] = "collision"
        elif evidence["outcome"] == "completed" and evidence["cleanup_errors"]:
            evidence["outcome"] = "cleanup_error"
        evidence["collision_count"] = len(collisions)
        evidence["completed"] = evidence["outcome"] == "completed" and not evidence["cleanup_errors"]
        package = Path(__file__).parent
        evidence["source_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in sorted(package.glob("*.py"))}
        evidence["artifacts_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                       for p in sorted(output.iterdir()) if p.is_file()}
        (output / "metadata.json").write_text(json.dumps(evidence, indent=2, allow_nan=False) + "\n")
    print(f"Scenario 2: {evidence['outcome']}; contacts={len(collisions)}; output={output}", flush=True)
    return 0 if evidence["completed"] else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/town10_rear_aeb.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2020)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--model", type=Path, help="OpenCV Zoo YOLOX-s ONNX path (default: .runtime/models)")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--load-map", action="store_true", help="load configured map if an idle server has another map")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        output = args.output_dir or ROOT / "results" / ("scenario_2_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ"))
        return run(config, output, host=args.host, port=args.port, video=not args.no_video,
                   load_map=args.load_map, model_path=args.model)
    except Exception as exc:
        parser.exit(1, f"Scenario 2 failed: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
