#!/usr/bin/env python3
"""Compare documented PID/MPC traces and produce JSON, Markdown, and figures."""
import argparse
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.metrics import compare_runs, load_csv, load_metadata, write_report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=Path, required=True)
    parser.add_argument("--mpc", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--stop-speed-mph", type=float, default=1.0)
    parser.add_argument("--allow-unverified", action="store_true", help="Diagnostics only without matching provenance")
    parser.add_argument("--pre-aeb-tolerance", type=float, default=0.001)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    try:
        pid, mpc = load_csv(args.pid), load_csv(args.mpc)
        report = compare_runs(pid, mpc, stop_speed_mph=args.stop_speed_mph,
                              provenance={"pid": load_metadata(args.pid), "mpc": load_metadata(args.mpc)},
                              allow_unverified=args.allow_unverified, pre_aeb_tolerance=args.pre_aeb_tolerance)
        paths = write_report(report, pid, mpc, args.output_dir, plots=not args.no_plots)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f"comparison error: {exc}\n")
    print(paths["markdown"])
    if not report["comparison_valid"]:
        print("Diagnostics only: " + ", ".join(report["invalid_reasons"]), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
