"""Deterministic longitudinal model benchmark, explicitly separate from CARLA.

The plant integrates straight-line motion with finite brake delay and a first-order
actuator. There is no perception noise, steering, tire model, or traffic. Vehicles
start at steady speed to isolate emergency braking from acceleration transients.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import platform

from .aeb import AEBMPC, AEBPID
from .config import AppConfig
from .csv_logger import ScenarioCsvLogger
from .safety import SafetySupervisor
from .scenario import ScenarioObservation, ScenarioOrchestrator, ScenarioPhase
from .units import mph_to_mps, mps_to_mph


@dataclass(frozen=True)
class ModelCase:
    name: str
    ego_speed_mph: float
    target_speed_mph: float
    initial_gap_m: float = 80.0
    grip_scale: float = 1.0
    brake_delay_s: float = 0.10
    actuator_tau_s: float = 0.15
    target_max_decel_mps2: float = 7.0

    def __post_init__(self) -> None:
        values = (self.ego_speed_mph, self.target_speed_mph, self.initial_gap_m,
                  self.grip_scale, self.brake_delay_s, self.actuator_tau_s,
                  self.target_max_decel_mps2)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("model settings must be finite")
        if not (self.ego_speed_mph > self.target_speed_mph >= 0 and self.initial_gap_m > 0
                and 0 < self.grip_scale <= 1 and self.brake_delay_s >= 0
                and self.actuator_tau_s > 0 and self.target_max_decel_mps2 > 0):
            raise ValueError("invalid speeds, gap, grip, delay, or actuator settings")
        if not self.name or any(c not in "abcdefghijklmnopqrstuvwxyz0123456789_" for c in self.name):
            raise ValueError("case name must contain lowercase letters, digits, or underscores")


DEFAULT_CASES = (
    ModelCase("urban_15_10", 15, 10),
    ModelCase("urban_25_15", 25, 15),
    ModelCase("arterial_35_20", 35, 20),
    ModelCase("highway_50_25", 50, 25, initial_gap_m=120),
    ModelCase("reduced_grip_35_20", 35, 20, grip_scale=0.55),
    ModelCase("delayed_brake_50_25", 50, 25, initial_gap_m=120, brake_delay_s=0.50),
)


def integrate_motion(speed: float, accel: float, dt: float) -> tuple[float, float]:
    """Return new speed and distance, integrating an in-step stop exactly."""
    moving_time = min(dt, speed / -accel) if accel < 0 else dt
    distance = speed * moving_time + 0.5 * accel * moving_time ** 2
    return max(0.0, speed + accel * dt), max(0.0, distance)


def run_model(config: AppConfig, case: ModelCase, controller: str, output: Path) -> dict:
    if controller not in {"pid", "mpc"}:
        raise ValueError("controller must be pid or mpc")
    dt = config.carla.fixed_delta_seconds
    if not math.isfinite(dt) or not 0 < dt <= 0.1:
        raise ValueError("model dt must be in (0, 0.1] seconds")
    controls = config.controllers
    scenario_config = replace(config.scenario, ego_speed_mph=case.ego_speed_mph,
                              target_speed_mph=case.target_speed_mph, initial_gap_m=case.initial_gap_m)
    scenario = ScenarioOrchestrator(
        scenario_config.ego_speed_mph, scenario_config.target_speed_mph,
        scenario_config.stable_tolerance_mph, scenario_config.stable_hold_s,
        scenario_config.stop_speed_mph, scenario_config.stop_hold_s, scenario_config.timeout_s)
    supervisor = SafetySupervisor(controls.fcw_ttc_s, controls.aeb_ttc_s,
                                  controls.aeb_distance_margin_m, controls.max_aeb_decel_mps2)
    emergency = (AEBPID(controls.aeb_pid_kp, controls.aeb_pid_ki, controls.aeb_pid_kd,
                        controls.aeb_pid_target_decel_mps2, controls.max_aeb_decel_mps2)
                 if controller == "pid" else
                 AEBMPC(controls.max_aeb_decel_mps2, controls.mpc_horizon_s, controls.mpc_step_s))
    ego = mph_to_mps(case.ego_speed_mph)
    target = mph_to_mps(case.target_speed_mph)
    gap = case.initial_gap_m
    accel = 0.0
    applied_brake = 0.0
    delay_steps = math.ceil(case.brake_delay_s / dt - 1e-12)
    delay = deque([0.0] * delay_steps)
    alpha = 1.0 - math.exp(-dt / case.actuator_tau_s)
    overlap = False
    premature = False
    with ScenarioCsvLogger(output) as logger:
        for tick in range(math.ceil(scenario_config.timeout_s / dt) + 2):
            elapsed = tick * dt
            assessment = supervisor.evaluate(ego, target, gap)
            premature = premature or (assessment.aeb and scenario.phase in
                                     {ScenarioPhase.ACCELERATE, ScenarioPhase.HOLD})
            # Preserve the initial condition as ACCELERATE; both cars already have
            # their equilibrium speeds, so HOLD starts on the next sample.
            if tick:
                scenario.update(ScenarioObservation(elapsed, mps_to_mph(ego),
                                                     mps_to_mph(target), assessment.aeb))
            overlap = overlap or gap <= 0
            if overlap or premature:
                scenario.phase = ScenarioPhase.ABORTED
            brake = 0.0
            if assessment.aeb:
                brake = emergency.compute(actual_accel_mps2=accel, dt_s=dt,
                                          ego_speed_mps=ego, target_speed_mps=target, gap_m=gap,
                                          target_decel_mps2=case.target_max_decel_mps2)
            logger.write(time_s=round(elapsed, 6), phase=scenario.phase.value,
                         ego_speed_mph=mps_to_mph(ego), target_speed_mph=mps_to_mph(target),
                         distance_m=gap, closing_speed_mph=mps_to_mph(assessment.closing_speed_mps),
                         ttc_s=assessment.ttc_s, fcw=int(assessment.fcw),
                         aeb_active=int(assessment.aeb), brake_percent=100 * brake,
                         ego_steer=0.0, ego_accel_mps2=accel)
            if scenario.finished:
                break
            delay.append(brake)
            delayed_brake = delay.popleft()
            applied_brake += alpha * (delayed_brake - applied_brake)
            requested_accel = -controls.max_aeb_decel_mps2 * case.grip_scale * applied_brake
            old_ego = ego
            ego, ego_distance = integrate_motion(ego, requested_accel, dt)
            accel = (ego - old_ego) / dt
            target_accel = (-case.target_max_decel_mps2 * case.grip_scale * scenario_config.target_brake
                            if scenario.target_hard_brake else 0.0)
            target, target_distance = integrate_motion(target, target_accel, dt)
            gap += target_distance - ego_distance

    settings = {"case": asdict(case), "dt_s": dt, "scenario": asdict(scenario_config),
                "controllers": asdict(controls), "initialization": "steady_speed",
                "effective_brake_delay_s": delay_steps * dt,
                "plant_version": 1, "controller_model_knows_grip_or_delay": False}
    # Hash controller/plant sources so a rerun identifies the exact implementation.
    source_files = ("benchmark.py", "aeb.py", "safety.py", "scenario.py", "units.py", "csv_logger.py", "config.py")
    source_hash = hashlib.sha256(b"".join(
        (Path(__file__).parent / name).read_bytes() for name in source_files)).hexdigest()
    metadata = {"source": "deterministic_1d_model", "controller": controller,
                "shared_settings": settings, "source_sha256": source_hash,
                "csv_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                "python_version": platform.python_version(),
                "outcome": scenario.phase.value, "overlap_detected": overlap,
                "premature_aeb": premature,
                "limitations": ["No CARLA or road validation", "Straight-line ideal sensing",
                                "Steady-speed initialization", "No statistical uncertainty estimate",
                                "Gap <= 0 detects model overlap, not a CARLA contact sensor"]}
    output.with_suffix(".metadata.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    return metadata
