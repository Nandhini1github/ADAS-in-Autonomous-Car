"""FCW/TTC calculation and the common AEB supervisor."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ThreatAssessment:
    closing_speed_mps: float
    ttc_s: float
    stopping_distance_m: float
    fcw: bool
    aeb: bool


class SafetySupervisor:
    """Keep threat assessment independent from the selected brake controller."""

    def __init__(
        self,
        fcw_ttc_s: float = 3.0,
        aeb_ttc_s: float = 2.0,
        distance_margin_m: float = 5.0,
        max_decel_mps2: float = 8.0,
    ) -> None:
        if aeb_ttc_s > fcw_ttc_s:
            raise ValueError("AEB TTC threshold cannot exceed FCW TTC threshold")
        self.fcw_ttc_s = fcw_ttc_s
        self.aeb_ttc_s = aeb_ttc_s
        self.distance_margin_m = distance_margin_m
        self.max_decel_mps2 = max_decel_mps2
        self.aeb_latched = False

    def reset(self) -> None:
        self.aeb_latched = False

    def evaluate(self, ego_speed_mps: float, target_speed_mps: float, gap_m: float) -> ThreatAssessment:
        closing_speed_mps = max(0.0, ego_speed_mps - target_speed_mps)
        ttc_s = gap_m / closing_speed_mps if gap_m > 0.0 and closing_speed_mps > 1.0e-3 else math.inf
        stopping_distance_m = ego_speed_mps * ego_speed_mps / (2.0 * self.max_decel_mps2)
        fcw = gap_m > 0.0 and closing_speed_mps > 0.0 and ttc_s <= self.fcw_ttc_s
        trigger = gap_m > 0.0 and closing_speed_mps > 0.0 and (
            ttc_s <= self.aeb_ttc_s
            or gap_m <= stopping_distance_m + self.distance_margin_m
        )
        self.aeb_latched = self.aeb_latched or trigger
        return ThreatAssessment(
            closing_speed_mps=closing_speed_mps,
            ttc_s=ttc_s,
            stopping_distance_m=stopping_distance_m,
            fcw=fcw or self.aeb_latched,
            aeb=self.aeb_latched,
        )
