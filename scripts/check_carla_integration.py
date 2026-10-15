#!/usr/bin/env python3
"""Read-only CARLA connectivity and version probe."""
import argparse
import json
import math
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    host, port = "127.0.0.1", 2000
    if args.config:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
        from town10_adas.config import load_config
        try:
            config = load_config(args.config)
            host, port = config.carla.host, config.carla.port
        except (OSError, ValueError, TypeError) as exc:
            parser.exit(2, f"configuration error: {exc}\n")
    host = args.host if args.host is not None else host
    port = args.port if args.port is not None else port
    if not host or not 1 <= port <= 65535 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("host, port, and timeout must be valid")
    try:
        import carla
        client = carla.Client(host, port)
        client.set_timeout(args.timeout)
        world = client.get_world()
        settings = world.get_settings()
        report = {
            "host": host, "port": port, "map": world.get_map().name,
            "client_version": client.get_client_version(), "server_version": client.get_server_version(),
            "actor_count": len(world.get_actors()), "synchronous_mode": settings.synchronous_mode,
            "fixed_delta_seconds": settings.fixed_delta_seconds,
        }
    except (ImportError, RuntimeError, OSError) as exc:
        parser.exit(1, f"CARLA unavailable at {host}:{port}: {exc}\n")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
