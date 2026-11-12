#!/usr/bin/env python3
"""Create the local station configuration from a role-specific example."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=("radar0", "radar1"))
    parser.add_argument("--interface", help="Interface connected to the DCA1000")
    parser.add_argument("--camera", choices=("on", "off"))
    parser.add_argument("--output", type=Path, default=PROJECT / "radar_station_config.json")
    parser.add_argument("--force", action="store_true", help="Replace an existing config")
    args = parser.parse_args()

    example = PROJECT / "config" / "examples" / f"{args.role}.json"
    config = json.loads(example.read_text(encoding="utf-8"))
    if args.interface:
        config["radar_interface"] = args.interface
    if args.camera:
        config["camera_enabled"] = args.camera == "on"
    if args.output.exists() and not args.force:
        parser.error(f"config already exists: {args.output}; use --force to replace it")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (PROJECT / "data" / "sessions").mkdir(parents=True, exist_ok=True)
    (PROJECT / "logs").mkdir(parents=True, exist_ok=True)
    print(f"{args.role}: {args.output}")


if __name__ == "__main__":
    main()
