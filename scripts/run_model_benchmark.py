#!/usr/bin/env python3
"""Run paired controller experiments in the disclosed 1-D model."""

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.benchmark import DEFAULT_CASES, run_model
from town10_adas.config import load_config
from town10_adas.metrics import compare_runs, load_csv, summarize_run, write_report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/town10_adas.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/model")
    parser.add_argument("--case", choices=[case.name for case in DEFAULT_CASES], action="append")
    parser.add_argument("--no-plots", action="store_true", help="generate CSV/JSON/Markdown without matplotlib")
    parser.add_argument("--allow-stress-failures", action="store_true",
                        help="exit successfully for the two documented stress-case overlap failures; still report them as invalid")
    args = parser.parse_args()
    config = load_config(args.config)
    cases = [case for case in DEFAULT_CASES if not args.case or case.name in args.case]
    summaries = []
    valid = True
    for case in cases:
        directory = args.output_dir / case.name
        directory.mkdir(parents=True, exist_ok=True)
        metadata, rows = {}, {}
        for controller in ("pid", "mpc"):
            path = directory / f"{controller}.csv"
            metadata[controller] = run_model(config, case, controller, path)
            rows[controller] = load_csv(path)
        report = compare_runs(rows["pid"], rows["mpc"],
                              stop_speed_mph=config.scenario.stop_speed_mph, provenance=metadata)
        write_report(report, rows["pid"], rows["mpc"], directory, plots=not args.no_plots)
        case_valid = report["comparison_valid"]
        expected_overlap = (case.name in {"reduced_grip_35_20", "delayed_brake_50_25"}
                            and report["provenance_verified"] and report["pre_aeb_traces_match"]
                            and set(report["invalid_reasons"]) <= {
                                f"{name}:{reason}" for name in ("pid", "mpc")
                                for reason in ("aborted_run", "no_stop_after_aeb", "gap_overlap_proxy")}
                            and all(meta["overlap_detected"] and not meta["premature_aeb"]
                                    for meta in metadata.values()))
        valid = valid and (case_valid or args.allow_stress_failures and expected_overlap)
        summaries.append({"case": case.name, "comparison_valid": case_valid,
                          "pid": summarize_run(rows["pid"], config.scenario.stop_speed_mph),
                          "mpc": summarize_run(rows["mpc"], config.scenario.stop_speed_mph)})
        print(f"{case.name}: {'valid' if case_valid else 'invalid; inspect report'}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "summary.json").write_text(json.dumps(
        {"source": "deterministic_1d_model", "cases": summaries}, indent=2, allow_nan=False) + "\n")
    lines = ["# PID versus MPC: longitudinal model results", "",
             "These are measured outputs of the repository's deterministic 1-D model, not CARLA or road measurements.",
             "Each pair uses identical settings and observations up to AEB activation. Six cases are a parameter sweep, not statistical repetitions.",
             "", "Both controllers fail the reduced-grip and delayed-brake stress cases with their current nominal models. These failures are included below.",
             "", "| Case | Controller | Min gap (m) | AEB → stop (s) | Stop distance (m) | Comparison |", "|---|---|---:|---:|---:|---|"]
    for entry in summaries:
        name = entry["case"]
        for controller in ("pid", "mpc"):
            metrics = entry[controller]
            def fmt(key):
                value = metrics.get(key)
                return "N/A" if value is None else f"{value:.3f}"
            lines.append(f"| [{name}]({name}/comparison.md) | {controller.upper()} | "
                         f"{fmt('minimum_gap_m')} | {fmt('stop_time_from_aeb_s')} | "
                         f"{fmt('stop_distance_from_aeb_m')} | {'valid' if entry['comparison_valid'] else 'invalid / overlap'} |")
    lines.extend(["", "## Reproduce", "", "```bash", "python3 scripts/run_model_benchmark.py --allow-stress-failures", "```", "",
                  "Without `--allow-stress-failures`, the documented overlap cases cause a nonzero exit code. They remain invalid comparisons with either option.",
                  "The plant includes brake delay, an actuator time constant, and grip scaling. Controllers retain their nominal model in the stress cases.",
                  "See [methodology](../../docs/comparison_methodology.md) and [model assumptions](../../docs/model_benchmark.md).", ""])
    if not args.no_plots and any(case.name == "highway_50_25" for case in cases):
        lines.extend(["## Example: 50/25 mph model traces", "",
                      "![50/25 mph PID and MPC model comparison](highway_50_25/comparison.png)", ""])
    (args.output_dir / "README.md").write_text("\n".join(lines))
    return 0 if valid else 1


if __name__ == "__main__":
    raise SystemExit(main())
