"""Typed configuration loader for the Town10 ADAS scenario."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CarlaConfig:
    host: str
    port: int
    timeout_s: float
    map: str
    fixed_delta_seconds: float
    synchronous_mode: bool
    ego_blueprint: str
    target_blueprint: str


@dataclass(frozen=True)
class ScenarioConfig:
    ego_speed_mph: float
    target_speed_mph: float
    initial_gap_m: float
    route_minimum_m: float
    stable_tolerance_mph: float
    stable_hold_s: float
    target_brake: float
    stop_speed_mph: float
    stop_hold_s: float
    timeout_s: float
    retain_actors_s: float


@dataclass(frozen=True)
class ControllerConfig:
    wheelbase_m: float
    wheel_max_angle_deg: float
    lookahead_base_m: float
    lookahead_gain_s: float
    max_steer: float
    speed_kp: float
    speed_ki: float
    speed_kd: float
    max_throttle: float
    service_brake_decel_mps2: float
    fcw_ttc_s: float
    aeb_ttc_s: float
    aeb_distance_margin_m: float
    max_aeb_decel_mps2: float
    aeb_pid_kp: float
    aeb_pid_ki: float
    aeb_pid_kd: float
    aeb_pid_target_decel_mps2: float
    mpc_horizon_s: float
    mpc_step_s: float


@dataclass(frozen=True)
class LoggingConfig:
    output_directory: str


@dataclass(frozen=True)
class AppConfig:
    carla: CarlaConfig
    scenario: ScenarioConfig
    controllers: ControllerConfig
    logging: LoggingConfig


def _section(raw: dict[str, Any], name: str) -> dict[str, Any]:
    value = raw.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"configuration section '{name}' must be an object")
    return value


def load_config(path: str | Path) -> AppConfig:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        raw = json.load(stream)

    return AppConfig(
        carla=CarlaConfig(**_section(raw, "carla")),
        scenario=ScenarioConfig(**_section(raw, "scenario")),
        controllers=ControllerConfig(**_section(raw, "controllers")),
        logging=LoggingConfig(**_section(raw, "logging")),
    )
