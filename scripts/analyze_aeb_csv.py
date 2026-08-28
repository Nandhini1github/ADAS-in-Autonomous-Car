#!/usr/bin/env python3
"""Summarize a Town10 ADAS CSV and flag whether an AEB event was exercised."""

import argparse
import csv
import math
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize a Town10 AEB result CSV")
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args()

    with args.csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("CSV contains no samples")

    phases = sorted({row["phase"] for row in rows})
    finite_ttc = [float(row["ttc_s"]) for row in rows if row["ttc_s"].lower() != "inf"]
    minimum_ttc = min(finite_ttc, default=math.inf)
    aeb_samples = sum(int(row["aeb_active"]) for row in rows)
    print(f"samples: {len(rows)}")
    print(f"duration_s: {float(rows[-1]['time_s']):.3f}")
    print(f"phases: {', '.join(phases)}")
    print(f"minimum_gap_m: {min(float(row['distance_m']) for row in rows):.3f}")
    print(f"minimum_ttc_s: {'inf' if math.isinf(minimum_ttc) else f'{minimum_ttc:.3f}'}")
    print(f"maximum_abs_steer: {max(abs(float(row['ego_steer'])) for row in rows):.4f}")
    print(f"fcw_samples: {sum(int(row['fcw']) for row in rows)}")
    print(f"aeb_samples: {aeb_samples}")
    print(f"brake_samples: {sum(float(row['brake_percent']) > 0.0 for row in rows)}")
    print(f"aeb_event_exercised: {'yes' if aeb_samples else 'no'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
