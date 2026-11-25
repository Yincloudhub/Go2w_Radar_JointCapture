#!/usr/bin/env python3
"""Create a dog-side fleet config without editing capture code."""

from __future__ import annotations

import argparse
import json
from pathlib import Path, PurePosixPath


PROJECT = Path(__file__).resolve().parents[1]


def station_config_path(root: str, path: str) -> str:
    return str(PurePosixPath(root) / "configFiles" / PurePosixPath(path).name)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radar0-ip", default="192.168.55.1")
    parser.add_argument("--radar1-ip", default="192.168.56.1")
    parser.add_argument("--radar0-user", default="radar")
    parser.add_argument("--radar1-user", default="ysy")
    parser.add_argument("--radar0-root", default="/home/radar/pyRadar-main")
    parser.add_argument("--radar1-root", default="/home/ysy/radar_capture_workspace")
    parser.add_argument("--station-code-layout", choices=("release", "git"), default="release")
    parser.add_argument("--output", type=Path, default=PROJECT / "tools" / "radar_fleet_config.json")
    parser.add_argument("--force", action="store_true", help="Replace an existing config")
    args = parser.parse_args()

    config = json.loads((PROJECT / "config" / "radar_fleet_config.example.json").read_text(encoding="utf-8"))
    roots = {"radar0": args.radar0_root, "radar1": args.radar1_root}
    hosts = {"radar0": args.radar0_ip, "radar1": args.radar1_ip}
    users = {"radar0": args.radar0_user, "radar1": args.radar1_user}
    config["output_root"] = "data/sessions"
    config["default_remote_root"] = roots["radar0"]
    config["station_code_layout"] = args.station_code_layout
    for station in config["stations"]:
        role = station["station_id"]
        station.update(host=hosts[role], host_candidates=[hosts[role]],
                       user=users[role], remote_root=roots[role])
        if args.station_code_layout == "git":
            station.pop("release_root", None)
            station.pop("current_link", None)
            station.pop("station_config_path", None)
    for profile in config.get("radar_profiles", {}).values():
        if profile.get("config_path"):
            profile["config_path"] = station_config_path(roots["radar0"], profile["config_path"])
        for role, path in (profile.get("station_paths") or {}).items():
            profile["station_paths"][role] = station_config_path(roots[role], path)
    for profile in config.get("operator_profiles", {}).values():
        for role, overrides in (profile.get("station_overrides") or {}).items():
            if overrides.get("radar_config"):
                overrides["radar_config"] = station_config_path(roots[role], overrides["radar_config"])
    default = config["default_radar_profile"]
    config["default_radar_config"] = config["radar_profiles"][default]["config_path"]
    if args.output.exists() and not args.force:
        parser.error(f"config already exists: {args.output}; use --force to replace it")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (PROJECT / "data" / "sessions").mkdir(parents=True, exist_ok=True)
    (PROJECT / "logs").mkdir(parents=True, exist_ok=True)
    print(f"fleet config: {args.output}")


if __name__ == "__main__":
    main()
