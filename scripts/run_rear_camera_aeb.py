#!/usr/bin/env python3
"""Run Scenario 2: Town10 curbside reversing and a crossing child."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from town10_adas.rear_runner import main

if __name__ == "__main__":
    raise SystemExit(main())
