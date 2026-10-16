"""Typed configuration loader for the Town10 ADAS scenario."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
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
    start_at_cruise_speed: bool = False
    rolling_ego_gear: int = 0
    rolling_target_gear: int = 0
    rolling_ego_throttle: float = 0.0
    rolling_target_throttle: float = 0.0


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

    config = AppConfig(
        carla=CarlaConfig(**_section(raw, "carla")),
        scenario=ScenarioConfig(**_section(raw, "scenario")),
        controllers=ControllerConfig(**_section(raw, "controllers")),
        logging=LoggingConfig(**_section(raw, "logging")),
    )
    validate_config(config)
    return config


def validate_config(config: AppConfig) -> None:
    """Fail before spawning actors or writing logs for invalid numeric settings."""
    sections = asdict(config)
    if not isinstance(config.scenario.start_at_cruise_speed, bool):
        raise ValueError("scenario.start_at_cruise_speed must be boolean")
    for section, values in sections.items():
        for name, value in values.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f"{section}.{name} must be finite and nonnegative")
    positive = {
        "carla": ("port", "timeout_s", "fixed_delta_seconds"),
        "scenario": ("ego_speed_mph", "initial_gap_m", "route_minimum_m", "timeout_s", "stop_speed_mph"),
        "controllers": ("wheelbase_m", "wheel_max_angle_deg", "service_brake_decel_mps2",
                        "fcw_ttc_s", "aeb_ttc_s", "max_aeb_decel_mps2", "aeb_pid_target_decel_mps2",
                        "mpc_horizon_s", "mpc_step_s"),
    }
    for section, names in positive.items():
        for name in names:
            if sections[section][name] <= 0:
                raise ValueError(f"{section}.{name} must be positive")
    controls = config.controllers
    for name in ("rolling_ego_gear", "rolling_target_gear"):
        if type(getattr(config.scenario, name)) is not int:
            raise ValueError(f"scenario.{name} must be a nonnegative integer")
    for name in ("rolling_ego_throttle", "rolling_target_throttle"):
        value = getattr(config.scenario, name)
        if not 0 <= value <= controls.max_throttle:
            raise ValueError(f"scenario.{name} must be within the configured throttle limit")
        if config.scenario.start_at_cruise_speed and value > 0 and controls.speed_ki <= 0:
            raise ValueError("rolling throttle initialization requires positive speed_ki")
    if controls.aeb_ttc_s > controls.fcw_ttc_s:
        raise ValueError("AEB TTC threshold cannot exceed FCW threshold")
    if controls.aeb_pid_target_decel_mps2 > controls.max_aeb_decel_mps2:
        raise ValueError("PID target deceleration cannot exceed maximum deceleration")
    if not controls.mpc_step_s <= controls.mpc_horizon_s <= 10000 * controls.mpc_step_s:
        raise ValueError("MPC horizon must contain 1 to 10000 steps")
    for name, value in (("max_steer", controls.max_steer), ("max_throttle", controls.max_throttle),
                        ("target_brake", config.scenario.target_brake)):
        if not 0 < value <= 1:
            raise ValueError(f"{name} must be in (0, 1]")
