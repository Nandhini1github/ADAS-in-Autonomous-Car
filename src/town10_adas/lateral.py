"""Conservative fixed-route Pure Pursuit lateral control."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class RoutePoint:
    x_m: float
    y_m: float


class PurePursuitController:
    """Track a route with speed-dependent lookahead and bounded steering."""

    def __init__(
        self,
        wheelbase_m: float = 2.85,
        wheel_max_angle_deg: float = 35.0,
        lookahead_base_m: float = 6.0,
        lookahead_gain_s: float = 0.45,
        max_steer: float = 0.18,
    ) -> None:
        self.wheelbase_m = wheelbase_m
        self.wheel_max_angle_rad = math.radians(wheel_max_angle_deg)
        self.lookahead_base_m = lookahead_base_m
        self.lookahead_gain_s = lookahead_gain_s
        self.max_steer = max_steer

    def compute(self, pose: Pose2D, speed_mps: float, route: Sequence[RoutePoint]) -> float:
        if not route:
            raise ValueError("route must contain at least one point")

        nearest = min(
            range(len(route)),
            key=lambda index: (route[index].x_m - pose.x_m) ** 2
            + (route[index].y_m - pose.y_m) ** 2,
        )
        lookahead_m = max(4.0, self.lookahead_base_m + self.lookahead_gain_s * speed_mps)
        target_index = nearest
        travelled_m = 0.0
        while target_index + 1 < len(route) and travelled_m < lookahead_m:
            current = route[target_index]
            following = route[target_index + 1]
            travelled_m += math.hypot(following.x_m - current.x_m, following.y_m - current.y_m)
            target_index += 1

        target = route[target_index]
        dx = target.x_m - pose.x_m
        dy = target.y_m - pose.y_m
        local_y = -math.sin(pose.yaw_rad) * dx + math.cos(pose.yaw_rad) * dy
        distance_m = max(math.hypot(dx, dy), 0.1)
        curvature = 2.0 * local_y / (distance_m * distance_m)
        wheel_angle_rad = math.atan(self.wheelbase_m * curvature)
        normalized = wheel_angle_rad / self.wheel_max_angle_rad
        return clamp(normalized, -self.max_steer, self.max_steer)
