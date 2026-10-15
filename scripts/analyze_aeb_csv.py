#!/usr/bin/env python3
"""Validate and summarize one AEB CSV."""
import argparse
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.metrics import load_csv, summarize_run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--stop-speed-mph", type=float, default=1.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    try:
        summary = summarize_run(load_csv(args.csv_path), args.stop_speed_mph)
    except (OSError, ValueError) as exc:
        parser.exit(2, f"analysis error: {exc}\n")
    if args.json:
        print(json.dumps(summary, indent=2, allow_nan=False))
    else:
        for key, value in summary.items():
            print(f"{key}: {'unavailable' if value is None else value}")
    return 0 if summary["valid_for_comparison"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
