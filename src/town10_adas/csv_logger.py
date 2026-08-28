"""Stable CSV schema for controller tuning and PID/MPC comparison."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any


FIELDNAMES = [
    "time_s",
    "phase",
    "ego_speed_mph",
    "target_speed_mph",
    "distance_m",
    "closing_speed_mph",
    "ttc_s",
    "fcw",
    "aeb_active",
    "brake_percent",
    "ego_steer",
    "ego_accel_mps2",
]


class ScenarioCsvLogger:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._stream, fieldnames=FIELDNAMES)
        self._writer.writeheader()

    def write(self, **row: Any) -> None:
        missing = set(FIELDNAMES) - set(row)
        if missing:
            raise ValueError(f"missing CSV fields: {sorted(missing)}")
        self._writer.writerow({name: row[name] for name in FIELDNAMES})
        self._stream.flush()

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.close()

    def __enter__(self) -> "ScenarioCsvLogger":
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
