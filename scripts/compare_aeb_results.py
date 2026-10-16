#!/usr/bin/env python3
"""Compare documented PID/MPC traces and produce CSV, JSON, Markdown, and figures."""
import argparse
import csv
import os
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from town10_adas.metrics import compare_runs, load_csv, load_metadata, write_report


def write_summary_csv(report, source_paths, output_dir):
    """Export one graphable row per controller; unavailable values stay blank."""
    rows = []
    for controller in ("pid", "mpc"):
        summary = report["runs"][controller]
        row = {
            "controller": controller.upper(),
            "source": report["source"],
            "source_csv": Path(os.path.relpath(source_paths[controller].resolve(),
                                              Path(output_dir).resolve())).as_posix(),
            "comparison_valid": int(report["comparison_valid"]),
            "provenance_verified": int(report["provenance_verified"]),
            "pre_aeb_traces_match": int(report["pre_aeb_traces_match"]),
            "comparison_invalid_reasons": ";".join(report["invalid_reasons"]),
            "run_trace_valid": int(summary["valid_for_comparison"]),
            "run_invalid_reasons": ";".join(summary["invalid_reasons"]),
            "phases": ";".join(summary["phases"]),
        }
        # Keep scalar metric names and units identical to comparison.json.
        for field, value in summary.items():
            if field not in {"valid_for_comparison", "invalid_reasons", "phases",
                             "event_times_s", "controller_runtime_ms"}:
                row[field] = int(value) if isinstance(value, bool) else value
        for event, value in summary["event_times_s"].items():
            row[f"{event}_time_s"] = value
        runtime = summary["controller_runtime_ms"] or {}
        row.update({
            "controller_runtime_mean_ms": runtime.get("mean"),
            "controller_runtime_max_ms": runtime.get("max"),
            "controller_runtime_sample_count": runtime.get("sample_count"),
        })
        rows.append(row)
    path = Path(output_dir) / "summary.csv"
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return path


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
        paths["csv"] = write_summary_csv(report, {"pid": args.pid, "mpc": args.mpc}, args.output_dir)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f"comparison error: {exc}\n")
    print(paths["markdown"])
    print(paths["csv"])
    if not report["comparison_valid"]:
        print("Diagnostics only: " + ", ".join(report["invalid_reasons"]), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
