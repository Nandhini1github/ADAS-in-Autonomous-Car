"""Deterministic Town10 AEB scenario state machine."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ScenarioPhase(str, Enum):
    ACCELERATE = "ACCELERATE"
    HOLD = "HOLD"
    THREAT = "THREAT"
    AEB = "AEB"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class ScenarioObservation:
    elapsed_s: float
    ego_speed_mph: float
    target_speed_mph: float
    aeb_active: bool


class ScenarioOrchestrator:
    """Guarantee an explicit target-brake event after stable cruising."""

    def __init__(
        self,
        ego_target_mph: float,
        target_target_mph: float,
        stable_tolerance_mph: float,
        stable_hold_s: float,
        stop_speed_mph: float,
        stop_hold_s: float,
        timeout_s: float,
    ) -> None:
        self.ego_target_mph = ego_target_mph
        self.target_target_mph = target_target_mph
        self.stable_tolerance_mph = stable_tolerance_mph
        self.stable_hold_s = stable_hold_s
        self.stop_speed_mph = stop_speed_mph
        self.stop_hold_s = stop_hold_s
        self.timeout_s = timeout_s
        self.phase = ScenarioPhase.ACCELERATE
        self.stable_since_s: float | None = None
        self.stopped_since_s: float | None = None

    @property
    def target_hard_brake(self) -> bool:
        return self.phase in {ScenarioPhase.THREAT, ScenarioPhase.AEB, ScenarioPhase.COMPLETE}

    @property
    def finished(self) -> bool:
        return self.phase in {ScenarioPhase.COMPLETE, ScenarioPhase.ABORTED}

    def update(self, observation: ScenarioObservation) -> ScenarioPhase:
        if self.finished:
            return self.phase
        if observation.elapsed_s >= self.timeout_s:
            self.phase = ScenarioPhase.ABORTED
            return self.phase

        on_speed = (
            abs(observation.ego_speed_mph - self.ego_target_mph) <= self.stable_tolerance_mph
            and abs(observation.target_speed_mph - self.target_target_mph) <= self.stable_tolerance_mph
        )
        if self.phase == ScenarioPhase.ACCELERATE and on_speed:
            self.phase = ScenarioPhase.HOLD
            self.stable_since_s = observation.elapsed_s
        elif self.phase == ScenarioPhase.HOLD:
            if not on_speed:
                self.phase = ScenarioPhase.ACCELERATE
                self.stable_since_s = None
            elif self.stable_since_s is not None and observation.elapsed_s - self.stable_since_s >= self.stable_hold_s:
                self.phase = ScenarioPhase.THREAT
        elif self.phase == ScenarioPhase.THREAT and observation.aeb_active:
            self.phase = ScenarioPhase.AEB
        elif self.phase == ScenarioPhase.AEB:
            if observation.ego_speed_mph <= self.stop_speed_mph:
                if self.stopped_since_s is None:
                    self.stopped_since_s = observation.elapsed_s
                elif observation.elapsed_s - self.stopped_since_s >= self.stop_hold_s:
                    self.phase = ScenarioPhase.COMPLETE
            else:
                self.stopped_since_s = None
        return self.phase
