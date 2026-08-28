"""Alternative AEB PID and lightweight receding-horizon MPC controllers."""

from __future__ import annotations


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class AEBPID:
    """Track a target emergency deceleration and return normalized brake."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        target_decel_mps2: float = 6.5,
        max_decel_mps2: float = 8.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_decel_mps2 = target_decel_mps2
        self.max_decel_mps2 = max_decel_mps2
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.initialized = False

    def compute(self, actual_accel_mps2: float, dt_s: float, **_: float) -> float:
        dt_s = max(dt_s, 1.0e-3)
        actual_decel = max(0.0, -actual_accel_mps2)
        error = self.target_decel_mps2 - actual_decel
        self.integral = clamp(self.integral + error * dt_s, -10.0, 10.0)
        derivative = 0.0 if not self.initialized else (error - self.previous_error) / dt_s
        self.previous_error = error
        self.initialized = True
        requested_decel = self.target_decel_mps2 + (
            self.kp * error + self.ki * self.integral + self.kd * derivative
        )
        return clamp(requested_decel / self.max_decel_mps2, 0.0, 1.0)


class AEBMPC:
    """Enumerate constant-deceleration actions over a 1-D prediction horizon."""

    def __init__(self, max_decel_mps2: float = 8.0, horizon_s: float = 2.5, step_s: float = 0.05) -> None:
        self.max_decel_mps2 = max_decel_mps2
        self.horizon_s = horizon_s
        self.step_s = step_s

    def reset(self) -> None:
        return None

    def compute(
        self,
        ego_speed_mps: float,
        target_speed_mps: float,
        gap_m: float,
        target_decel_mps2: float = 7.0,
        **_: float,
    ) -> float:
        best_decel = self.max_decel_mps2
        best_cost = float("inf")
        steps = max(1, int(self.horizon_s / self.step_s))
        candidates = [0.5 * index for index in range(int(self.max_decel_mps2 / 0.5) + 1)]

        for candidate in candidates:
            ego = max(0.0, ego_speed_mps)
            target = max(0.0, target_speed_mps)
            gap = gap_m
            minimum_gap = gap
            for _step in range(steps):
                target = max(0.0, target - target_decel_mps2 * self.step_s)
                ego = max(0.0, ego - candidate * self.step_s)
                gap += (target - ego) * self.step_s
                minimum_gap = min(minimum_gap, gap)

            collision_cost = 1.0e8 if minimum_gap <= 0.0 else 2000.0 / max(minimum_gap, 0.25)
            closing_cost = 20.0 * max(0.0, ego - target) ** 2
            control_cost = 0.25 * candidate * candidate
            cost = collision_cost + closing_cost + control_cost
            if cost < best_cost:
                best_cost = cost
                best_decel = candidate

        return clamp(best_decel / self.max_decel_mps2, 0.0, 1.0)
