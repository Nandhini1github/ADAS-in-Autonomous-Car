"""CSV-bound CARLA provenance, written only after closing the CSV."""
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


def write_carla_metadata(output_path, config, controller, evidence):
    if controller not in {"pid", "mpc"}:
        raise ValueError("controller must be pid or mpc")
    evidence = json.loads(json.dumps(evidence, allow_nan=False))
    completed = evidence.get("completed")
    count = evidence.get("collision_count")
    if not isinstance(completed, bool):
        raise ValueError("completed must be boolean")
    if count is not None and (type(count) is not int or count < 0):
        raise ValueError("collision_count must be nonnegative or unknown")
    if count is not None and len(evidence.get("collision_events", [])) != count:
        raise ValueError("collision events/count mismatch")
    if completed and (count is None or not evidence.get("collision_sensor_attached")
                      or any(not evidence.get(k) for k in ("route", "spawns", "map", "versions"))):
        raise ValueError("completed CARLA evidence is incomplete")
    path = Path(output_path)
    with path.open(newline="") as stream:
        row_count = sum(1 for _ in csv.DictReader(stream))
    if completed and not row_count:
        raise ValueError("completed CSV has no samples")
    package = Path(__file__).parent
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(package.glob("*.py"))}
    source_hash = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
    settings = {"config": asdict(config), **{key: evidence.get(key) for key in
                ("route", "spawns", "map", "versions", "world_settings", "weather",
                 "world_reloaded", "settle_ticks")}}
    metadata = {
        "schema_version": 1, "source": "carla", "controller": controller,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "shared_settings": settings, "source_sha256": source_hash,
        "csv_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "row_count": row_count, "completed": completed, "outcome": evidence.get("outcome"),
        "collision_count": count, "evidence": evidence,
    }
    serialized = json.dumps(metadata, indent=2, allow_nan=False) + "\n"
    destination = path.with_suffix(".metadata.json")
    temporary = path.with_suffix(".metadata.json.tmp")
    temporary.write_text(serialized, encoding="utf-8")
    temporary.replace(destination)
    return destination
