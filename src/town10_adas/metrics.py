"""Strict CSV validation and reproducible paired AEB measurements."""
from __future__ import annotations
import csv
import hashlib
import json
import math
from pathlib import Path
from .csv_logger import FIELDNAMES
from .units import mph_to_mps

PHASES = {"ACCELERATE", "HOLD", "THREAT", "AEB", "COMPLETE", "ABORTED", "CRUISE"}


def validate_rows(rows):
    if not rows:
        raise ValueError("CSV contains no samples; header-only placeholders are not results")
    data, previous, terminal = [], -math.inf, False
    for number, raw in enumerate(rows, 2):
        missing = set(FIELDNAMES) - raw.keys()
        if missing:
            raise ValueError(f"row {number}: missing fields {sorted(missing)}")
        if None in raw or any(v is None for v in raw.values()):
            raise ValueError(f"row {number}: inconsistent CSV column count")
        phase = str(raw["phase"]).strip()
        if phase not in PHASES or terminal:
            raise ValueError(f"row {number}: unknown phase or samples after terminal phase")
        terminal = phase in {"COMPLETE", "ABORTED"}
        row = {"phase": phase}
        for field in FIELDNAMES:
            if field == "phase":
                continue
            try:
                value = float(raw[field])
            except (TypeError, ValueError) as exc:
                raise ValueError(f"row {number}: {field} must be numeric") from exc
            if not math.isfinite(value) and not (field == "ttc_s" and value == math.inf):
                raise ValueError(f"row {number}: {field} must be finite; only +inf TTC is allowed")
            row[field] = value
        if row["time_s"] < 0 or row["time_s"] <= previous:
            raise ValueError(f"row {number}: timestamps must be nonnegative and strictly increasing")
        previous = row["time_s"]
        for field in ("ego_speed_mph", "target_speed_mph", "ttc_s", "closing_speed_mph"):
            if row[field] < 0:
                raise ValueError(f"row {number}: {field} must be nonnegative")
        for field in ("fcw", "aeb_active"):
            if row[field] not in (0, 1):
                raise ValueError(f"row {number}: {field} must be 0 or 1")
            row[field] = int(row[field])
        if not 0 <= row["brake_percent"] <= 100 or not -1 <= row["ego_steer"] <= 1:
            raise ValueError(f"row {number}: brake percent or normalized steering outside bounds")
        if "controller_runtime_ms" in raw:
            runtime = float(raw["controller_runtime_ms"])
            if not math.isfinite(runtime) or runtime < 0:
                raise ValueError(f"row {number}: runtime must be finite and nonnegative")
            row["controller_runtime_ms"] = runtime
        data.append(row)
    return data


def load_csv(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        header = reader.fieldnames or []
        if len(header) != len(set(header)):
            raise ValueError("duplicate CSV columns")
        if set(FIELDNAMES) - set(header):
            raise ValueError("CSV missing required columns")
        return validate_rows(list(reader))


def summarize_run(rows, stop_speed_mph=1.0):
    if not math.isfinite(stop_speed_mph) or stop_speed_mph < 0:
        raise ValueError("stop_speed_mph must be finite and nonnegative")
    data = validate_rows(rows)
    first = lambda predicate: next((i for i, r in enumerate(data) if predicate(r)), None)
    aeb = first(lambda r: r["aeb_active"] == 1)
    fcw = first(lambda r: r["fcw"] == 1)
    threat = first(lambda r: r["phase"] == "THREAT")
    stop = None if aeb is None else next((i for i in range(aeb, len(data)) if data[i]["ego_speed_mph"] <= stop_speed_mph), None)
    errors, phase = [], data[-1]["phase"]
    if aeb is None:
        errors.append("no_aeb_activation")
    if not any(r["phase"] == "AEB" for r in data):
        errors.append("missing_aeb_phase")
    if phase not in {"COMPLETE", "ABORTED"}:
        errors.append("missing_terminal_phase")
    elif phase == "ABORTED":
        errors.append("aborted_run")
    if stop is None:
        errors.append("no_stop_after_aeb")
    elif any(r["ego_speed_mph"] > stop_speed_mph for r in data[stop:]):
        errors.append("speed_rebounded_after_stop_threshold")
    if aeb is not None:
        if data[aeb]["ego_speed_mph"] <= stop_speed_mph:
            errors.append("aeb_activated_at_or_below_stop_threshold")
        if fcw is None or fcw > aeb:
            errors.append("fcw_missing_or_after_aeb")
        if any(not r["aeb_active"] for r in data[aeb:]):
            errors.append("aeb_did_not_remain_latched")
    overlap = any(r["distance_m"] <= 0 for r in data)
    if overlap:
        errors.append("gap_overlap_proxy")
    stop_time = distance = None
    if aeb is not None and stop is not None:
        stop_time = data[stop]["time_s"] - data[aeb]["time_s"]
        distance = sum(0.5 * mph_to_mps(data[i-1]["ego_speed_mph"] + data[i]["ego_speed_mph"]) *
                       (data[i]["time_s"] - data[i-1]["time_s"]) for i in range(aeb + 1, stop + 1))
    accelerations, midpoints = [], []
    samples = [] if aeb is None else data[max(0, aeb - 1):]
    for left, right in zip(samples, samples[1:]):
        accelerations.append(mph_to_mps(right["ego_speed_mph"] - left["ego_speed_mph"]) /
                             (right["time_s"] - left["time_s"]))
        midpoints.append((left["time_s"] + right["time_s"]) / 2)
    jerks = [abs((b-a)/(t2-t1)) for a, b, t1, t2 in zip(accelerations, accelerations[1:], midpoints, midpoints[1:])]
    ttc = [r["ttc_s"] for r in data if math.isfinite(r["ttc_s"])]
    runtime = [r["controller_runtime_ms"] for r in data if "controller_runtime_ms" in r]
    event_time = lambda index: None if index is None else data[index]["time_s"]
    return {
        "valid_for_comparison": not errors, "invalid_reasons": errors,
        "outcome": "aborted" if phase == "ABORTED" else "incomplete_or_invalid" if errors else "completed_with_positive_gap",
        "sample_count": len(data), "duration_s": data[-1]["time_s"] - data[0]["time_s"],
        "start_time_s": data[0]["time_s"], "end_time_s": data[-1]["time_s"],
        "phases": list(dict.fromkeys(r["phase"] for r in data)), "final_phase": phase,
        "event_times_s": {"first_fcw": event_time(fcw), "first_threat": event_time(threat),
                          "first_aeb": event_time(aeb), "first_stop_threshold": event_time(stop)},
        "activation_speed_mph": None if aeb is None else data[aeb]["ego_speed_mph"],
        "activation_gap_m": None if aeb is None else data[aeb]["distance_m"],
        "minimum_gap_m": min(r["distance_m"] for r in data),
        "minimum_finite_ttc_s": min(ttc, default=None), "finite_ttc_sample_count": len(ttc),
        "stop_speed_threshold_mph": stop_speed_mph,
        "stop_time_from_aeb_s": stop_time, "stop_distance_from_aeb_m": distance,
        "peak_deceleration_fd_mps2": max((max(0.0, -a) for a in accelerations), default=None),
        "peak_absolute_jerk_fd_mps3": max(jerks, default=None),
        "maximum_abs_steer": max(abs(r["ego_steer"]) for r in data),
        "aeb_sample_count": sum(r["aeb_active"] for r in data),
        "fcw_sample_count": sum(r["fcw"] for r in data),
        "brake_sample_count": sum(r["brake_percent"] > 0 for r in data),
        "gap_overlap_proxy": overlap, "collision_sensor_contact": None,
        "collision_evidence": "gap <= 0 proxy only; no contact sensor in CSV schema",
        "controller_runtime_ms": None if not runtime else {"mean": sum(runtime)/len(runtime),
                                                           "max": max(runtime), "sample_count": len(runtime)},
    }


def load_metadata(csv_path):
    path = Path(csv_path)
    sidecar = path.with_suffix(".metadata.json")
    if not sidecar.exists():
        return None
    metadata = json.loads(sidecar.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError(f"{sidecar}: expected JSON object")
    if metadata.get("csv_sha256") and metadata["csv_sha256"] != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError(f"{sidecar}: CSV content hash mismatch")
    json.dumps(metadata, allow_nan=False)
    return metadata


def compare_runs(pid_rows, mpc_rows, stop_speed_mph=1.0, provenance=None,
                 allow_unverified=False, pre_aeb_tolerance=0.001):
    if not math.isfinite(pre_aeb_tolerance) or pre_aeb_tolerance < 0:
        raise ValueError("pre_aeb_tolerance must be finite and nonnegative")
    pid, mpc = validate_rows(pid_rows), validate_rows(mpc_rows)
    metadata, reasons = provenance or {}, []
    source = "unverified CSV (source unavailable)"
    paired = "pid" in metadata or "mpc" in metadata
    if paired:
        p, m = metadata.get("pid") or {}, metadata.get("mpc") or {}
        source = str(p.get("source", source))
        if not p.get("source") or p.get("source") != m.get("source"):
            reasons.append("source_missing_or_mismatched")
        if not p.get("shared_settings") or p.get("shared_settings") != m.get("shared_settings"):
            reasons.append("shared_settings_missing_or_mismatched")
        if p.get("controller") != "pid" or m.get("controller") != "mpc":
            reasons.append("controller_identities_missing_or_mismatched")
        if p.get("source_sha256") != m.get("source_sha256"):
            reasons.append("controller_source_hash_mismatch")
    else:
        source = str(metadata.get("source", source))
        if not metadata.get("source") or not metadata.get("shared_settings"):
            reasons.append("provenance_unavailable")
        if metadata.get("controllers") != {"pid": "pid", "mpc": "mpc"}:
            reasons.append("controller_identities_missing_or_mismatched")
    provenance_valid = not reasons
    if reasons and not allow_unverified:
        raise ValueError("matching provenance required: " + ", ".join(reasons) +
                         "; supply metadata or explicitly use --allow-unverified for diagnostics")
    p_index = next((i for i, r in enumerate(pid) if r["aeb_active"]), None)
    m_index = next((i for i, r in enumerate(mpc) if r["aeb_active"]), None)
    trace_reasons = []
    if p_index is None or m_index is None:
        trace_reasons.append("pre_aeb_match_unavailable_without_activation")
    elif p_index != m_index:
        trace_reasons.append("different_number_of_pre_aeb_samples")
    elif p_index == 0:
        trace_reasons.append("no_pre_aeb_samples")
    else:
        fields = ("time_s", "ego_speed_mph", "target_speed_mph", "distance_m",
                  "closing_speed_mph", "ttc_s", "fcw", "aeb_active", "ego_steer", "ego_accel_mps2")
        for i in range(p_index + 1):
            if pid[i]["phase"] != mpc[i]["phase"]:
                trace_reasons.append(f"pre_aeb_phase_mismatch_sample_{i}")
                break
            mismatches = [f for f in fields if pid[i][f] != mpc[i][f] and
                          not math.isclose(pid[i][f], mpc[i][f], rel_tol=0, abs_tol=pre_aeb_tolerance)]
            if mismatches:
                trace_reasons.append(f"pre_aeb_mismatch_sample_{i}_" + "_".join(mismatches))
                break
    runs = {"pid": summarize_run(pid, stop_speed_mph), "mpc": summarize_run(mpc, stop_speed_mph)}
    reasons += trace_reasons
    for name, summary in runs.items():
        reasons.extend(f"{name}:{reason}" for reason in summary["invalid_reasons"])
        evidence = metadata.get(name) if paired else None
        if evidence and source == "carla":
            count = evidence.get("collision_count")
            summary["collision_sensor_contact"] = None if count is None else count > 0
            if count is None or not evidence.get("completed"):
                reasons.append(f"{name}:incomplete_carla_evidence")
            elif count > 0:
                reasons.append(f"{name}:carla_contact_detected")
    keys = ("minimum_gap_m", "minimum_finite_ttc_s", "stop_time_from_aeb_s",
            "stop_distance_from_aeb_m", "peak_deceleration_fd_mps2", "peak_absolute_jerk_fd_mps3")
    deltas = {key: runs["mpc"][key] - runs["pid"][key]
              if not reasons and runs["mpc"][key] is not None and runs["pid"][key] is not None
              else None for key in keys}
    report = {
        "schema_version": 1, "source": source, "comparison_valid": not reasons,
        "invalid_reasons": reasons, "provenance_verified": provenance_valid,
        "pre_aeb_traces_match": not trace_reasons, "pre_aeb_absolute_tolerance": pre_aeb_tolerance,
        "provenance": dict(metadata), "runs": runs, "mpc_minus_pid": deltas,
        "interpretation": "No overall winner: gap, response and comfort trade off. Deltas require complete, matched, documented runs.",
        "limitations": [
            ("CARLA traces use simulator ground-truth vehicle state; they do not validate sensor perception or road safety."
             if source == "carla" else
             "Model traces are controller experiments, not CARLA or road validation."),
            "Gap <= 0 is an overlap proxy; contact requires separate sensor evidence.",
            "TTC assumes constant relative motion; nonclosing TTC is unavailable.",
            "Finite differences are unsmoothed and depend on sampling and speed quantization.",
            "Stop is the first sample at or below the speed threshold, not exact zero; distance uses trapezoids.",
            "One paired run does not establish statistical significance.",
        ],
    }
    json.dumps(report, allow_nan=False)
    return report


def _display(value):
    return "unavailable" if value is None else f"{value:.4f}" if isinstance(value, float) else str(value)


def render_markdown(report):
    status = "VALID PAIRED COMPARISON" if report["comparison_valid"] else "DIAGNOSTICS ONLY — COMPARISON INVALID"
    lines = ["# PID and MPC AEB comparison", "", f"**Source:** {report['source']}", "", f"**Status:** {status}", ""]
    if report["source"] == "deterministic_1d_model":
        lines += ["Measured outputs of the simplified longitudinal model. These are not CARLA or physical vehicle results.", ""]
    if report["invalid_reasons"]:
        lines += ["Invalidity reasons: " + ", ".join(report["invalid_reasons"]), ""]
    lines += ["| Metric | PID | MPC | MPC − PID |", "|---|---:|---:|---:|"]
    fields = [
        ("sample_count", "Samples"), ("duration_s", "Observed duration (s)"),
        ("activation_speed_mph", "Speed at AEB (mph)"), ("activation_gap_m", "Gap at AEB (m)"),
        ("minimum_gap_m", "Minimum gap (m)"), ("minimum_finite_ttc_s", "Minimum finite TTC (s)"),
        ("stop_time_from_aeb_s", "AEB to stop threshold (s)"), ("stop_distance_from_aeb_m", "AEB to stop threshold (m)"),
        ("peak_deceleration_fd_mps2", "Peak deceleration, finite difference (m/s²)"),
        ("peak_absolute_jerk_fd_mps3", "Peak absolute jerk, finite difference (m/s³)"),
        ("gap_overlap_proxy", "Gap ≤ 0 proxy"), ("collision_sensor_contact", "Collision sensor contact"),
        ("final_phase", "Final phase"), ("outcome", "Outcome"),
    ]
    for key, label in fields:
        delta = _display(report["mpc_minus_pid"][key]) if key in report["mpc_minus_pid"] else "—"
        lines.append(f"| {label} | {_display(report['runs']['pid'][key])} | {_display(report['runs']['mpc'][key])} | {delta} |")
    threshold = report["runs"]["pid"]["stop_speed_threshold_mph"]
    lines += ["", f"Stop threshold: **{threshold:g} mph**. Distance integrates ego speed from first AEB.", ""]
    for name, summary in report["runs"].items():
        events = summary["event_times_s"]
        lines += [f"- **{name.upper()} events (s):** FCW {_display(events['first_fcw'])}; threat {_display(events['first_threat'])}; AEB {_display(events['first_aeb'])}; stop {_display(events['first_stop_threshold'])}. Controller runtime (ms): {_display(summary['controller_runtime_ms'])}."]
    lines += ["", report["interpretation"], "", "## Measurement limits", ""]
    lines += [f"- {item}" for item in report["limitations"]]
    return "\n".join(lines) + "\n"


def _write_plots(pid, mpc, output_dir, source):
    try:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
    except ImportError as exc:
        raise RuntimeError("Plots require matplotlib; install the plots extra or use --no-plots") from exc
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    fields = [("ego_speed_mph", "Ego speed (mph)"), ("distance_m", "Gap (m)"),
              ("ttc_s", "Finite TTC (s)"), ("brake_percent", "Brake (%)"),
              ("ego_accel_mps2", "Logged acceleration (m/s²)"), ("aeb_active", "AEB active")]
    for name, data, color in [("PID", pid, "#3465a4"), ("MPC", mpc, "#c45218")]:
        times = [r["time_s"] for r in data]
        activation = next((r["time_s"] for r in data if r["aeb_active"]), None)
        for axis, (field, label) in zip(axes.flat, fields):
            axis.plot(times, [r[field] if math.isfinite(r[field]) else math.nan for r in data],
                      label=name, color=color, linewidth=1.6)
            if activation is not None:
                axis.axvline(activation, color=color, linestyle=":", linewidth=0.8, alpha=0.6)
            axis.set_ylabel(label)
            axis.grid(alpha=0.25)
    axes[0, 0].legend()
    axes[0, 1].axhline(0, color="#666666", linestyle="--", linewidth=0.8)
    for axis in axes[-1, :]:
        axis.set_xlabel("Elapsed time (s); dotted line = first AEB")
    fig.suptitle(f"PID / MPC • {source}\nObserved traces; gap is not collision-sensor evidence")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    for extension in ("png", "svg"):
        fig.savefig(output_dir / f"comparison.{extension}", dpi=160)
    svg_path = output_dir / "comparison.svg"
    svg_path.write_text("\n".join(line.rstrip() for line in svg_path.read_text().splitlines()) + "\n")
    plt.close(fig)


def write_report(report, pid_rows, mpc_rows, output_dir, plots=True):
    destination = Path(output_dir)
    serialized = json.dumps(report, indent=2, allow_nan=False) + "\n"
    pid, mpc = validate_rows(pid_rows), validate_rows(mpc_rows)
    destination.mkdir(parents=True, exist_ok=True)
    markdown = render_markdown(report)
    paths = {"json": str(destination / "comparison.json"), "markdown": str(destination / "comparison.md")}
    if plots:
        _write_plots(pid, mpc, destination, report["source"])
        markdown += "\n## Traces\n\n![PID/MPC traces](comparison.png)\n\n[Download SVG](comparison.svg)\n"
        paths.update({ext: str(destination / f"comparison.{ext}") for ext in ("png", "svg")})
    Path(paths["json"]).write_text(serialized, encoding="utf-8")
    Path(paths["markdown"]).write_text(markdown, encoding="utf-8")
    return paths
