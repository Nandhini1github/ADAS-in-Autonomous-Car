"""Normal-driving longitudinal speed control."""

from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class ControlDemand:
    throttle: float
    brake: float


class LongitudinalPID:
    """Bidirectional speed PID producing mutually exclusive throttle/brake."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        max_throttle: float = 0.60,
        service_brake_decel_mps2: float = 4.0,
        integral_limit: float = 10.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_throttle = max_throttle
        self.service_brake_decel_mps2 = service_brake_decel_mps2
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def compute(self, desired_mps: float, actual_mps: float, dt_s: float) -> ControlDemand:
        dt_s = max(dt_s, 1.0e-3)
        error = desired_mps - actual_mps
        self.integral = clamp(
            self.integral + error * dt_s,
            -self.integral_limit,
            self.integral_limit,
        )
        derivative = 0.0 if not self.initialized else (error - self.previous_error) / dt_s
        self.previous_error = error
        self.initialized = True
        effort = self.kp * error + self.ki * self.integral + self.kd * derivative

        if effort >= 0.0:
            return ControlDemand(throttle=clamp(effort, 0.0, self.max_throttle), brake=0.0)
        brake = clamp(-effort / self.service_brake_decel_mps2, 0.0, 1.0)
        return ControlDemand(throttle=0.0, brake=brake)
