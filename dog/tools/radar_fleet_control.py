from __future__ import annotations

import argparse
import concurrent.futures
import contextlib
import csv
import getpass
import ipaddress
import json
import math
import os
import shlex
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path, PurePosixPath
from typing import Any, Callable

REPO_ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = REPO_ROOT.parent / "_vendor"
if VENDOR_DIR.exists():
    sys.path.insert(0, str(VENDOR_DIR))
try:
    import paramiko  # type: ignore
except ImportError:  # pragma: no cover - only needed for the explicit sync action
    paramiko = None

from stm32_trigger import Stm32Trigger, TriggerError, frame_count as hardware_frame_count


REMOTE_HELPER = REPO_ROOT / "tools" / "radar_station_remote.py"
CONTROLLER_IMU_HELPER = REPO_ROOT / "tools" / "controller_telemetry_capture.py"
CONTROLLER_VIDEO_HELPER = REPO_ROOT / "tools" / "controller_video_capture.py"
FLEET_CLOCK_HELPER = REPO_ROOT / "tools" / "fleet_clock_check.py"
DEFAULT_CONFIG_PATH = REPO_ROOT / "tools" / "radar_fleet_config.json"
DEFAULT_RADAR_MODE = "60f128"
DEFAULT_RADAR_CONFIG = f"/home/radar/pyRadar-main/configFiles/mmwaveconfig_0508_{DEFAULT_RADAR_MODE}.txt"
NEGATIVE_CHIRP_REMOTE_CONFIG = "/home/radar/pyRadar-main/configFiles/AWR2243_mmwaveconfig_neg.txt"
DEFAULT_RADAR_HOST_CANDIDATES = ["192.168.3.20", "192.168.43.27", "192.168.43.28"]
STATION_OVERRIDE_CASTERS: dict[str, Any] = {
    "radar_config": str,
    "reader_mode": str,
    "duration_seconds": float,
    "countdown_seconds": float,
    "camera_device": str,
    "camera_width": int,
    "camera_height": int,
    "camera_fps": int,
    "camera_fourcc": str,
    "camera_format": str,
    "camera_save_every_n": int,
    "imu_port": str,
    "imu_baud": int,
}

DEFAULT_OPERATOR_PROFILES: dict[str, dict[str, Any]] = {
    "quick": {
        "label": "Quick",
        "description": "Short low-overhead capture for checking a scene quickly.",
        "duration_seconds": 6.0,
        "countdown_seconds": 1.5,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 3,
        "selected_stations": [],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": True,
        "notes": [
            "Use this when you need a fast field check.",
            "Skips the controller-level duplicate preflight but still keeps station internal checks and clock snapshots.",
        ],
    },
    "field": {
        "label": "Field",
        "description": "Default dual-station field mode.",
        "duration_seconds": 15.0,
        "countdown_seconds": 3.0,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 5,
        "selected_stations": [],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": False,
        "notes": [
            "Recommended default mode. Uses radar 60f128 unless a station override or --radar-config is provided.",
            "Keep raw data on each radar-side NX and write only logs and summaries on the dog NX.",
        ],
    },
    "sync": {
        "label": "Sync",
        "description": "Dual-station mode that favors simultaneous start.",
        "duration_seconds": 15.0,
        "countdown_seconds": 3.0,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 9,
        "selected_stations": [],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": False,
        "notes": [
            "Use the same countdown on both stations.",
            "Take denser clock snapshots before and after capture.",
        ],
    },
    "anti-interference": {
        "label": "Anti-Interference",
        "description": "Offset one station countdown to reduce simultaneous emission overlap.",
        "duration_seconds": 15.0,
        "countdown_seconds": 3.0,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 7,
        "selected_stations": [],
        "station_countdown_offsets": {
            "radar0": 0.0,
            "radar1": 0.35,
        },
        "station_overrides": {},
        "skip_preflight": False,
        "notes": [
            "This mode intentionally sacrifices strict simultaneity.",
            "Current implementation uses a fixed delayed start rather than waveform changes.",
        ],
    },
    "solo-radar0": {
        "label": "Solo radar0",
        "description": "Run only radar0.",
        "duration_seconds": 15.0,
        "countdown_seconds": 1.5,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 3,
        "selected_stations": ["radar0"],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": True,
        "notes": [
            "Single-station fast mode. Run Field preflight before the capture when hardware was just connected.",
        ],
    },
    "solo-radar1": {
        "label": "Solo radar1",
        "description": "Run only radar1.",
        "duration_seconds": 15.0,
        "countdown_seconds": 1.5,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 3,
        "selected_stations": ["radar1"],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": True,
        "notes": [
            "Single-station fast mode. Run Field preflight before the capture when hardware was just connected.",
        ],
    },
    "negative-chirp": {
        "label": "Negative Chirp",
        "description": "Keep radar0 on the default up-chirp and run radar1 with a down-chirp config.",
        "duration_seconds": 15.0,
        "countdown_seconds": 3.0,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 7,
        "selected_stations": ["radar0", "radar1"],
        "station_countdown_offsets": {},
        "station_overrides": {
            "radar1": {
                "radar_config": NEGATIVE_CHIRP_REMOTE_CONFIG,
            },
        },
        "skip_preflight": False,
        "notes": [
            "Experimental mode.",
            "Requires the negative chirp config file on the radar-side NX.",
        ],
    },
    "smoke": {
        "label": "Smoke",
        "description": "Very short hardware smoke test.",
        "duration_seconds": 2.0,
        "countdown_seconds": 1.0,
        "result_mode": "station-local-only",
        "allow_no_camera": True,
        "allow_no_imu": False,
        "clock_snapshot_samples": 3,
        "selected_stations": [],
        "station_countdown_offsets": {},
        "station_overrides": {},
        "skip_preflight": True,
        "notes": [
            "Use after rewiring or rebooting before a real capture.",
            "Skips the controller-level duplicate preflight to keep the test short.",
        ],
    },
}

OPERATOR_PROFILE_ALIASES = {
    "default": "field",
    "portable": "field",
    "field": "field",
    "quick": "quick",
    "fast": "quick",
    "smoke": "smoke",
    "test": "smoke",
    "sync": "sync",
    "interference": "anti-interference",
    "anti": "anti-interference",
    "anti-interference": "anti-interference",
    "anti_interference": "anti-interference",
    "solo0": "solo-radar0",
    "solo-radar0": "solo-radar0",
    "solo1": "solo-radar1",
    "solo-radar1": "solo-radar1",
    "neg": "negative-chirp",
    "negative": "negative-chirp",
    "negative-chirp": "negative-chirp",
    "negative_chirp": "negative-chirp",
}

DEFAULT_UI_MODES = {
    "dual": {"label": "双雷达联采", "operator_profile": "joint-dual-rx03-77g", "stations": ["radar0", "radar1"]},
    "nx1": {"label": "仅 NX1 采集", "operator_profile": "joint-single-rx03-77g", "stations": ["radar0"]},
    "nx2": {"label": "仅 NX2 采集", "operator_profile": "solo-radar1", "stations": ["radar1"]},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Controller-side orchestrator that lets one machine-dog NX drive multiple radar-side NX stations."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)

    subparsers = parser.add_subparsers(dest="command", required=True)

    show_config = subparsers.add_parser("show-config", help="Print the normalized fleet configuration.")
    add_station_filter_args(show_config)

    set_ips = subparsers.add_parser(
        "set-ips",
        help="Update radar station IPs in tools/radar_fleet_config.json from the dog NX.",
    )
    set_ips.add_argument("--radar0", default="", help="IP address for radar0, for example 192.168.3.20.")
    set_ips.add_argument("--radar1", default="", help="IP address for radar1.")
    set_ips.add_argument("--enable-radar1", action="store_true", help="Enable radar1 after setting/checking its IP.")
    set_ips.add_argument("--disable-radar1", action="store_true", help="Disable radar1 for one-station operation.")
    set_ips.add_argument("--disable-radar0", action="store_true", help="Disable radar0.")
    set_ips.add_argument("--check", action="store_true", help="Ping/SSH-check enabled stations after writing config.")

    sync_code = subparsers.add_parser(
        "sync-code",
        help="从 NX1 主版本源生成版本包，并校验后原子同步到 NX2。",
    )
    add_station_filter_args(sync_code)

    preflight = subparsers.add_parser("preflight", help="Run preflight on one or more remote stations.")
    add_station_filter_args(preflight)
    add_common_override_args(preflight)
    preflight.add_argument("--allow-no-camera", action="store_true")
    preflight.add_argument("--allow-no-imu", action="store_true")
    preflight.add_argument("--skip-network-setup", action="store_true")
    preflight.add_argument("--no-controller-imu", action="store_true")

    capture = subparsers.add_parser("capture", help="Run synchronized multi-station capture from the controller NX.")
    add_station_filter_args(capture)
    capture.add_argument("--name", required=True, help="Shared scene or experiment label.")
    capture.add_argument("--capture-id", default="", help="Shared capture identifier.")
    capture.add_argument("--skip-preflight", action="store_true")
    capture.add_argument(
        "--force-capture",
        action="store_true",
        help="Continue into capture even when one or more station preflight checks fail.",
    )
    capture.add_argument(
        "--no-pull",
        action="store_true",
        help="Do not download each station session back to the controller NX.",
    )
    capture.add_argument(
        "--result-mode",
        choices=["pull-full", "station-local-only"],
        default=None,
        help="How station results are handled after capture. 'station-local-only' keeps data on each NX and only returns status/summary.",
    )
    capture.add_argument("--no-controller-imu", action="store_true")
    capture.add_argument("--no-clock-snapshots", action="store_true")
    capture.add_argument(
        "--clock-snapshot-samples",
        type=int,
        default=None,
        help="Optional override for how many SSH midpoint samples are taken before/after capture.",
    )
    add_common_override_args(capture)
    capture.add_argument("--allow-no-camera", action="store_true")
    capture.add_argument("--allow-no-imu", action="store_true")
    capture.add_argument("--skip-network-setup", action="store_true")

    field_preflight = subparsers.add_parser(
        "field-preflight",
        help="Recommended field preflight: keep the main chain running even if camera or IMU is temporarily unstable.",
    )
    add_station_filter_args(field_preflight)
    field_preflight.add_argument("--strict-camera", action="store_true")
    field_preflight.add_argument("--strict-imu", action="store_true")
    field_preflight.add_argument("--skip-network-setup", action="store_true")
    field_preflight.add_argument("--no-controller-imu", action="store_true")

    field_capture = subparsers.add_parser(
        "field-capture",
        help="Recommended field capture: station-local save by default, shorter parameters, and auto-generated labels when omitted.",
    )
    add_station_filter_args(field_capture)
    field_capture.add_argument("--name", default="", help="Optional scene label. Auto-generated when omitted.")
    field_capture.add_argument("--capture-id", default="", help="Optional capture id. Auto-generated when omitted.")
    field_capture.add_argument("--seconds", type=float, default=15.0, help="Radar active capture window in seconds.")
    field_capture.add_argument("--countdown", type=float, default=3.0, help="Countdown before station capture starts.")
    field_capture.add_argument("--pull", action="store_true", help="After capture, also pull each station session back to the dog NX.")
    field_capture.add_argument("--skip-preflight", action="store_true")
    field_capture.add_argument("--force-capture", action="store_true")
    field_capture.add_argument("--strict-camera", action="store_true")
    field_capture.add_argument("--strict-imu", action="store_true")
    field_capture.add_argument("--skip-network-setup", action="store_true")
    field_capture.add_argument("--no-controller-imu", action="store_true")
    field_capture.add_argument("--no-clock-snapshots", action="store_true")
    field_capture.add_argument(
        "--clock-snapshot-samples",
        type=int,
        default=None,
        help="Optional override for how many SSH midpoint samples are taken before/after capture.",
    )

    pull_results = subparsers.add_parser(
        "pull-results",
        help="For an existing fleet session, pull the referenced station-local sessions back to the dog NX.",
    )
    add_station_filter_args(pull_results)
    pull_results.add_argument(
        "--fleet-session-dir",
        type=Path,
        required=True,
        help="Existing fleet session directory on the dog NX.",
    )
    pull_results.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional local output root. Defaults to <fleet-session-dir>/stations.",
    )

    ingest = subparsers.add_parser(
        "ingest",
        help="Scan fleet sessions and write one searchable session catalog JSON.",
    )
    ingest.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Fleet session root directory. Defaults to output_root from the config.",
    )
    ingest.add_argument(
        "--session-dir",
        action="append",
        default=[],
        help="Optional explicit fleet session directory. Repeat the flag to ingest selected sessions only.",
    )
    ingest.add_argument(
        "--catalog-path",
        type=Path,
        default=None,
        help="Optional output path for the generated catalog JSON. Defaults to <root>/fleet_session_index.json.",
    )

    sync_report = subparsers.add_parser(
        "sync-report",
        help="Summarize multi-run timing stability, clock drift, and station-level alignment across fleet sessions.",
    )
    sync_report.add_argument(
        "--root",
        type=Path,
        default=None,
        help="Fleet session root directory. Defaults to output_root from the config.",
    )
    sync_report.add_argument(
        "--session-dir",
        action="append",
        default=[],
        help="Optional explicit fleet session directory. Repeat the flag to analyze selected sessions only.",
    )
    sync_report.add_argument(
        "--output-path",
        type=Path,
        default=None,
        help="Optional JSON output path. Defaults to <root>/fleet_sync_report.json.",
    )

    profiles = subparsers.add_parser(
        "profiles",
        help="List the simplified one-command operator profiles.",
    )
    subparsers.add_parser(
        "modes",
        help="Alias of 'profiles'.",
    )

    go = subparsers.add_parser(
        "go",
        help="One-command capture: just give a name and one mode profile, such as 'field', 'sync', or 'anti-interference'.",
    )
    add_station_filter_args(go)
    go.add_argument("name", help="Scene or experiment label.")
    go.add_argument(
        "mode",
        nargs="?",
        default=None,
        help="Operator profile. Omit to use default_operator_profile from config.",
    )
    go.add_argument("--capture-id", default="", help="Optional capture id. Auto-generated when omitted.")
    go.add_argument("--seconds", type=float, default=None, help="Optional override for radar active capture seconds.")
    go.add_argument("--countdown", type=float, default=None, help="Optional override for countdown seconds.")
    go.add_argument("--radar-config", default=None, help="Optional global radar config override applied to all selected stations.")
    go.add_argument("--pull", action="store_true", help="Also pull station-local sessions back to the dog NX.")
    go.add_argument("--skip-preflight", action="store_true")
    go.add_argument("--force-capture", action="store_true")
    go.add_argument("--strict-camera", action="store_true")
    go.add_argument("--strict-imu", action="store_true")
    go.add_argument("--skip-network-setup", action="store_true")
    go.add_argument("--no-controller-imu", action="store_true")
    go.add_argument("--no-clock-snapshots", action="store_true")
    go.add_argument(
        "--clock-snapshot-samples",
        type=int,
        default=None,
        help="Optional override for SSH midpoint sample count.",
    )

    ui = subparsers.add_parser(
        "ui",
        help="Interactive SSH-friendly terminal UI for common fleet operations.",
    )
    add_station_filter_args(ui)

    return parser.parse_args()


def add_station_filter_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--station",
        action="append",
        default=[],
        help="Optional station_id filter. Repeat the flag to target multiple stations.",
    )


def add_common_override_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--radar-config", default=None)
    parser.add_argument("--reader-mode", default=None)
    parser.add_argument("--duration-seconds", type=float, default=None)
    parser.add_argument("--countdown-seconds", type=float, default=None)
    parser.add_argument("--camera-device", default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=int, default=None)
    parser.add_argument("--camera-fourcc", default=None)
    parser.add_argument("--camera-format", default=None)
    parser.add_argument("--camera-save-every-n", type=int, default=None)
    parser.add_argument("--imu-port", default=None)
    parser.add_argument("--imu-baud", type=int, default=None)


def load_config(config_path: Path) -> dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    if not isinstance(config, dict):
        raise RuntimeError(f"Fleet config must be a JSON object: {config_path}")
    return config


def normalize_controller_imu_config(config: dict[str, Any]) -> dict[str, Any]:
    controller_imu_raw = config.get("controller_imu") or {}
    if not isinstance(controller_imu_raw, dict):
        raise RuntimeError("controller_imu must be a JSON object when present.")

    ros_setup_scripts = controller_imu_raw.get("ros_setup_scripts")
    if ros_setup_scripts in (None, ""):
        ros_setup_scripts = ["/opt/ros/foxy/setup.bash", "~/cyclonedds_ws/install/setup.bash"]
    if not isinstance(ros_setup_scripts, list):
        raise RuntimeError("controller_imu.ros_setup_scripts must be a JSON array.")

    return {
        "enabled": bool(controller_imu_raw.get("enabled", True)),
        "required": bool(controller_imu_raw.get("required", False)),
        "topic": str(controller_imu_raw.get("topic", "/dog_imu_raw")).strip() or "/dog_imu_raw",
        **{key: str(controller_imu_raw.get(key, "")).strip() for key in ("odom_topic", "sport_topic", "lidar_imu_topic")},
        "timeout_seconds": float(controller_imu_raw.get("timeout_seconds", 3.0)),
        "ros_setup_scripts": [str(item).strip() for item in ros_setup_scripts if str(item).strip()],
        "notes": list(controller_imu_raw.get("notes", [])),
    }


def normalize_controller_video_config(config: dict[str, Any]) -> dict[str, Any]:
    raw = config.get("controller_video") or {}
    if not isinstance(raw, dict):
        raise RuntimeError("controller_video must be a JSON object when present.")
    return {
        "enabled": bool(raw.get("enabled", True)),
        "required": bool(raw.get("required", False)),
        "source": str(raw.get("source", "device")).strip() or "device",
        "device": str(raw.get("device", "auto")).strip() or "auto",
        "topic": str(raw.get("topic", "/frontvideostream")).strip() or "/frontvideostream",
        "field": str(raw.get("field", "video720p")).strip() or "video720p",
        "width": int(raw.get("width", 1920)),
        "height": int(raw.get("height", 1080)),
        "fps": int(raw.get("fps", 30)),
        "fourcc": str(raw.get("fourcc", "MJPG")).strip() or "MJPG",
        "timeout_seconds": float(raw.get("timeout_seconds", 3.0)),
        "ros_setup_scripts": [
            str(item).strip()
            for item in raw.get("ros_setup_scripts", ["/opt/ros/foxy/setup.bash", "~/cyclonedds_ws/install/setup.bash"])
            if str(item).strip()
        ],
        "notes": list(raw.get("notes", [])),
    }


def normalize_result_mode(raw_value: Any, pull_results_fallback: bool) -> str:
    mode = str(raw_value or "").strip().lower().replace("_", "-")
    aliases = {
        "pull": "pull-full",
        "pull-full": "pull-full",
        "full": "pull-full",
        "station-local-only": "station-local-only",
        "local-only": "station-local-only",
        "station-only": "station-local-only",
        "remote-only": "station-local-only",
    }
    if not mode:
        return "pull-full" if pull_results_fallback else "station-local-only"
    if mode not in aliases:
        raise RuntimeError(
            "result_mode must be one of: pull-full, station-local-only."
        )
    return aliases[mode]


def normalize_clock_snapshot_config(config: dict[str, Any]) -> dict[str, Any]:
    raw = config.get("clock_snapshots") or {}
    if not isinstance(raw, dict):
        raise RuntimeError("clock_snapshots must be a JSON object when present.")
    return {
        "enabled": bool(raw.get("enabled", True)),
        "required": bool(raw.get("required", True)),
        "samples": int(raw.get("samples", 3)),
        "max_offset_ms": float(raw.get("max_offset_ms", 10.0)),
        "notes": list(raw.get("notes", [])),
    }


def canonical_operator_profile_name(name: str) -> str:
    key = str(name or "").strip().lower().replace("_", "-")
    return OPERATOR_PROFILE_ALIASES.get(key, key)


def normalize_profile_selected_stations(value: Any, field_name: str) -> list[str]:
    if value in (None, ""):
        return []
    if not isinstance(value, list):
        raise RuntimeError(f"{field_name} must be a JSON array when present.")
    ordered: list[str] = []
    seen: set[str] = set()
    for item in value:
        station_id = str(item).strip()
        if not station_id or station_id in seen:
            continue
        seen.add(station_id)
        ordered.append(station_id)
    return ordered


def normalize_station_overrides(value: Any, field_name: str) -> dict[str, dict[str, Any]]:
    if value in (None, ""):
        return {}
    if not isinstance(value, dict):
        raise RuntimeError(f"{field_name} must be a JSON object when present.")
    normalized: dict[str, dict[str, Any]] = {}
    for station_id_raw, override_raw in value.items():
        station_id = str(station_id_raw).strip()
        if not station_id:
            raise RuntimeError(f"{field_name} contains an empty station id.")
        if not isinstance(override_raw, dict):
            raise RuntimeError(f"{field_name}.{station_id} must be a JSON object.")
        station_override: dict[str, Any] = {}
        for key_raw, raw_value in override_raw.items():
            key = str(key_raw).strip()
            caster = STATION_OVERRIDE_CASTERS.get(key)
            if caster is None:
                allowed = ", ".join(sorted(STATION_OVERRIDE_CASTERS))
                raise RuntimeError(f"Unsupported station override '{field_name}.{station_id}.{key}'. Allowed keys: {allowed}")
            if raw_value in (None, ""):
                continue
            station_override[key] = caster(raw_value)
        normalized[station_id] = station_override
    return normalized


def normalize_host_candidates(station_raw: dict[str, Any], primary_host: str) -> list[str]:
    raw_candidates = station_raw.get("host_candidates", [])
    candidates: list[str] = []
    if isinstance(raw_candidates, list):
        candidates.extend(str(item).strip() for item in raw_candidates)
    elif raw_candidates not in (None, ""):
        candidates.append(str(raw_candidates).strip())
    candidates.insert(0, primary_host)
    if str(station_raw.get("station_id", "")).strip() == "radar0":
        candidates.extend(DEFAULT_RADAR_HOST_CANDIDATES)
    ordered: list[str] = []
    seen: set[str] = set()
    for candidate in candidates:
        if not candidate or candidate in seen:
            continue
        seen.add(candidate)
        ordered.append(candidate)
    return ordered


def tcp_port_open(host: str, port: int, timeout_seconds: float = 0.5) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout_seconds):
            return True
    except OSError:
        return False


def resolve_station_host(station: dict[str, Any]) -> dict[str, Any]:
    candidates = list(station.get("host_candidates") or [station["host"]])
    for candidate in candidates:
        if tcp_port_open(candidate, int(station.get("port", 22)), 0.5):
            resolved = dict(station)
            resolved["configured_host"] = station["host"]
            resolved["host"] = candidate
            resolved["host_resolution"] = {
                "status": "ok",
                "configured_host": station["host"],
                "selected_host": candidate,
                "candidates": candidates,
            }
            if candidate != station["host"]:
                print(
                    f"[{station['station_id']}] using reachable host {candidate} "
                    f"(configured {station['host']})",
                    file=sys.stderr,
                    flush=True,
                )
            return resolved
    resolved = dict(station)
    resolved["host_resolution"] = {
        "status": "unreachable",
        "configured_host": station["host"],
        "selected_host": station["host"],
        "candidates": candidates,
    }
    return resolved


def resolve_station_hosts(stations: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [resolve_station_host(station) for station in stations]


def normalize_operator_profiles(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    raw_profiles = config.get("operator_profiles") or {}
    if not isinstance(raw_profiles, dict):
        raise RuntimeError("operator_profiles must be a JSON object when present.")

    profiles: dict[str, dict[str, Any]] = {}
    for name, default_profile in DEFAULT_OPERATOR_PROFILES.items():
        profiles[name] = {
            "name": name,
            "label": str(default_profile.get("label", name)),
            "description": str(default_profile.get("description", "")).strip(),
            "duration_seconds": float(default_profile.get("duration_seconds", 15.0)),
            "countdown_seconds": float(default_profile.get("countdown_seconds", 3.0)),
            "result_mode": normalize_result_mode(default_profile.get("result_mode", "station-local-only"), False),
            "allow_no_camera": bool(default_profile.get("allow_no_camera", True)),
            "allow_no_imu": bool(default_profile.get("allow_no_imu", False)),
            "clock_snapshot_samples": max(1, int(default_profile.get("clock_snapshot_samples", 5))),
            "selected_stations": normalize_profile_selected_stations(
                default_profile.get("selected_stations", []),
                f"default operator_profiles.{name}.selected_stations",
            ),
            "station_countdown_offsets": {
                str(station_id): float(offset_seconds)
                for station_id, offset_seconds in (default_profile.get("station_countdown_offsets") or {}).items()
            },
            "station_overrides": normalize_station_overrides(
                default_profile.get("station_overrides", {}),
                f"default operator_profiles.{name}.station_overrides",
            ),
            "skip_preflight": bool(default_profile.get("skip_preflight", False)),
            "notes": list(default_profile.get("notes", [])),
        }

    for raw_name, raw_profile in raw_profiles.items():
        if not isinstance(raw_profile, dict):
            raise RuntimeError(f"operator_profiles.{raw_name} must be a JSON object.")
        name = canonical_operator_profile_name(str(raw_name))
        base = profiles.get(
            name,
            {
                "name": name,
                "label": name,
                "description": "",
                "duration_seconds": 15.0,
                "countdown_seconds": 3.0,
                "result_mode": "station-local-only",
                "allow_no_camera": True,
                "allow_no_imu": False,
                "clock_snapshot_samples": 5,
                "selected_stations": [],
                "station_countdown_offsets": {},
                "station_overrides": {},
                "skip_preflight": False,
                "notes": [],
            },
        )
        station_offsets_raw = raw_profile.get("station_countdown_offsets", base.get("station_countdown_offsets", {})) or {}
        if not isinstance(station_offsets_raw, dict):
            raise RuntimeError(f"operator_profiles.{raw_name}.station_countdown_offsets must be a JSON object.")
        profiles[name] = {
            "name": name,
            "label": str(raw_profile.get("label", base["label"])),
            "description": str(raw_profile.get("description", base["description"])).strip(),
            "duration_seconds": float(raw_profile.get("duration_seconds", base["duration_seconds"])),
            "countdown_seconds": float(raw_profile.get("countdown_seconds", base["countdown_seconds"])),
            "result_mode": normalize_result_mode(raw_profile.get("result_mode", base["result_mode"]), False),
            "allow_no_camera": bool(raw_profile.get("allow_no_camera", base["allow_no_camera"])),
            "allow_no_imu": bool(raw_profile.get("allow_no_imu", base["allow_no_imu"])),
            "clock_snapshot_samples": max(
                1,
                int(raw_profile.get("clock_snapshot_samples", base["clock_snapshot_samples"])),
            ),
            "selected_stations": normalize_profile_selected_stations(
                raw_profile.get("selected_stations", base["selected_stations"]),
                f"operator_profiles.{raw_name}.selected_stations",
            ),
            "station_countdown_offsets": {
                str(station_id): float(offset_seconds)
                for station_id, offset_seconds in station_offsets_raw.items()
            },
            "station_overrides": normalize_station_overrides(
                raw_profile.get("station_overrides", base["station_overrides"]),
                f"operator_profiles.{raw_name}.station_overrides",
            ),
            "skip_preflight": bool(raw_profile.get("skip_preflight", base.get("skip_preflight", False))),
            "notes": list(raw_profile.get("notes", base["notes"])),
        }

    return profiles


def normalize_radar_config_library(config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    raw_profiles = config.get("radar_profiles") or {}
    if not isinstance(raw_profiles, dict):
        return {}
    library: dict[str, dict[str, Any]] = {}
    for raw_name, raw_profile in raw_profiles.items():
        if not isinstance(raw_profile, dict):
            continue
        name = str(raw_name).strip()
        config_path = str(raw_profile.get("config_path", "")).strip()
        if not name or not config_path:
            continue
        library[name] = {
            "name": name,
            "label": str(raw_profile.get("label", name)),
            "mode": str(raw_profile.get("mode", "")),
            "config_path": config_path,
            "reader_mode": str(raw_profile.get("reader_mode", "")).strip(),
            "hardware_trigger": bool(raw_profile.get("hardware_trigger", False)),
            "station_paths": dict(raw_profile.get("station_paths") or {}),
            "fps": raw_profile.get("fps"),
            "chirps_per_frame": raw_profile.get("chirps_per_frame"),
            "description": str(raw_profile.get("description", "")),
            "notes": list(raw_profile.get("notes", [])),
        }
    return library


def normalize_ui_modes(config: dict[str, Any], operator_profiles: dict[str, Any], stations: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    raw_modes = config.get("ui_modes") or {}
    if not isinstance(raw_modes, dict):
        raise RuntimeError("ui_modes must be a JSON object when present.")
    station_ids = {station["station_id"] for station in stations}
    defaults = {name: mode for name, mode in DEFAULT_UI_MODES.items()
                if mode["operator_profile"] in operator_profiles and set(mode["stations"]).issubset(station_ids)}
    if not defaults:
        defaults = {"default": {"label": "当前模式", "operator_profile":
                    canonical_operator_profile_name(str(config.get("default_operator_profile", "field"))),
                    "stations": [station["station_id"] for station in stations if station["enabled"]]}}
    modes: dict[str, dict[str, Any]] = {}
    for name, mode in {**defaults, **raw_modes}.items():
        if not isinstance(mode, dict):
            raise RuntimeError(f"ui_modes.{name} must be a JSON object.")
        profile_name = str(mode.get("operator_profile", "")).strip()
        selected = list(mode.get("stations") or [])
        if profile_name not in operator_profiles or not selected or any(item not in station_ids for item in selected):
            raise RuntimeError(f"ui_modes.{name} references an unknown profile or station.")
        modes[str(name)] = {
            "name": str(name), "label": str(mode.get("label", name)),
            "operator_profile": profile_name, "stations": selected,
        }
    return modes


def resolve_default_radar_config(config: dict[str, Any], radar_config_library: dict[str, dict[str, Any]]) -> str:
    requested = str(config.get("default_radar_profile", "") or config.get("radar_profile", "")).strip()
    if requested and requested in radar_config_library:
        return str(radar_config_library[requested]["config_path"])
    requested_path = str(config.get("default_radar_config", "") or config.get("radar_config", "")).strip()
    return requested_path or DEFAULT_RADAR_CONFIG


def normalize_config(config: dict[str, Any], config_path: Path) -> dict[str, Any]:
    controller_id = str(config.get("controller_id", "dog_nx"))
    output_path = Path(str(config.get("output_root", "data/sessions"))).expanduser()
    output_root = str((config_path.resolve().parent.parent / output_path).resolve()
                      if not output_path.is_absolute() else output_path)
    ssh_config = str(config.get("ssh_config", "~/.ssh/config"))
    default_user = str(config.get("default_user", "radar"))
    default_port = int(config.get("default_port", 22))
    default_key_file = str(config.get("default_key_file", "")).strip()
    default_remote_root = str(config.get("default_remote_root", "/home/radar/pyRadar-main"))
    password_env = str(config.get("password_env", "")).strip()
    sudo_password_env = str(config.get("sudo_password_env", "")).strip()
    pull_results = bool(config.get("pull_results", True))
    result_mode = normalize_result_mode(config.get("result_mode", ""), pull_results)
    preflight_before_capture = bool(config.get("preflight_before_capture", True))
    default_operator_profile = canonical_operator_profile_name(str(config.get("default_operator_profile", "sync")))
    radar_config_library = normalize_radar_config_library(config)
    default_radar_config = resolve_default_radar_config(config, radar_config_library)
    if not radar_config_library:
        radar_config_library = {"default": {"name": "default", "label": "当前雷达配置", "mode": "custom",
                                            "config_path": default_radar_config, "reader_mode": "", "station_paths": {},
                                            "fps": None, "chirps_per_frame": None, "description": "", "notes": []}}

    stations_raw = config.get("stations") or []
    if not isinstance(stations_raw, list) or not stations_raw:
        raise RuntimeError("Fleet config must define at least one station entry.")

    stations: list[dict[str, Any]] = []
    seen_station_ids: set[str] = set()
    for station_raw in stations_raw:
        if not isinstance(station_raw, dict):
            raise RuntimeError("Each station entry must be a JSON object.")
        station_id = str(station_raw.get("station_id", "")).strip()
        host = str(station_raw.get("host", "")).strip()
        if not station_id:
            raise RuntimeError("Each station entry must include station_id.")
        if station_id in seen_station_ids:
            raise RuntimeError(f"Duplicate station_id in fleet config: {station_id}")
        if not host:
            raise RuntimeError(f"Station '{station_id}' must define host.")
        seen_station_ids.add(station_id)
        stations.append(
            {
                "station_id": station_id,
                "host": host,
                "host_candidates": normalize_host_candidates(station_raw, host),
                "user": str(station_raw.get("user", default_user)).strip(),
                "port": int(station_raw.get("port", default_port)),
                "key_file": str(station_raw.get("key_file", default_key_file)).strip(),
                "ssh_config": str(station_raw.get("ssh_config", ssh_config)).strip(),
                "remote_root": str(station_raw.get("remote_root", default_remote_root)).strip(),
                "release_root": str(station_raw.get("release_root", station_raw.get("remote_root", default_remote_root))).strip(),
                "current_link": str(station_raw.get("current_link", "current")).strip() or "current",
                "station_config_path": str(station_raw.get("station_config_path", "station/radar_station_config.json")).strip()
                or "station/radar_station_config.json",
                "password_env": str(station_raw.get("password_env", password_env)).strip(),
                "sudo_password_env": str(station_raw.get("sudo_password_env", sudo_password_env)).strip(),
                "camera_required": bool(station_raw.get("camera_required", True)),
                "enabled": bool(station_raw.get("enabled", True)),
                "notes": list(station_raw.get("notes", [])),
            }
        )

    operator_profiles = normalize_operator_profiles(config)
    known_station_ids = {station["station_id"] for station in stations}
    for profile_name, profile in operator_profiles.items():
        selected = set(profile.get("selected_stations", []))
        missing_selected = selected - known_station_ids
        if missing_selected:
            raise RuntimeError(
                f"operator_profiles.{profile_name}.selected_stations references unknown station(s): {', '.join(sorted(missing_selected))}"
            )
        overridden = set((profile.get("station_overrides") or {}).keys())
        missing_overrides = overridden - known_station_ids
        if missing_overrides:
            raise RuntimeError(
                f"operator_profiles.{profile_name}.station_overrides references unknown station(s): {', '.join(sorted(missing_overrides))}"
            )

    return {
        "config_path": str(config_path),
        "controller_id": controller_id,
        "output_root": output_root,
        "ssh_config": ssh_config,
        "default_user": default_user,
        "default_port": default_port,
        "default_key_file": default_key_file,
        "default_remote_root": default_remote_root,
        "pull_results": pull_results,
        "result_mode": result_mode,
        "preflight_before_capture": preflight_before_capture,
        "require_complete_joint_capture": bool(config.get("require_complete_joint_capture", False)),
        "default_operator_profile": default_operator_profile,
        "default_radar_config": default_radar_config,
        "radar_config_library": radar_config_library,
        "ui_modes": normalize_ui_modes(config, operator_profiles, stations),
        "controller_imu": normalize_controller_imu_config(config),
        "controller_video": normalize_controller_video_config(config),
        "stm32_trigger": dict(config.get("stm32_trigger") or {}),
        "clock_snapshots": normalize_clock_snapshot_config(config),
        "controller_data_transfer": dict(config.get("controller_data_transfer") or {}),
        "operator_profiles": operator_profiles,
        "stations": stations,
    }


def select_stations(config: dict[str, Any], station_filters: list[str]) -> list[dict[str, Any]]:
    requested = {station_id.strip() for station_id in station_filters if station_id.strip()}
    all_station_ids = {station["station_id"] for station in config["stations"]}
    missing = requested - all_station_ids
    if missing:
        raise RuntimeError(f"Unknown station_id(s): {', '.join(sorted(missing))}")
    selected = [
        station
        for station in config["stations"]
        if station["enabled"] and (not requested or station["station_id"] in requested)
    ]
    if not selected:
        raise RuntimeError("No enabled stations matched the request.")
    return selected


def append_common_override_args(command: list[str], args: argparse.Namespace) -> None:
    append_optional(command, "--radar-config", args.radar_config)
    append_optional(command, "--reader-mode", getattr(args, "reader_mode", None))
    append_optional(command, "--duration-seconds", args.duration_seconds)
    append_optional(command, "--countdown-seconds", args.countdown_seconds)
    append_optional(command, "--start-at-wall-ns", getattr(args, "start_at_wall_ns", None))
    append_optional(command, "--camera-device", args.camera_device)
    append_optional(command, "--camera-width", args.camera_width)
    append_optional(command, "--camera-height", args.camera_height)
    append_optional(command, "--camera-fps", args.camera_fps)
    append_optional(command, "--camera-fourcc", args.camera_fourcc)
    append_optional(command, "--camera-format", args.camera_format)
    append_optional(command, "--camera-save-every-n", args.camera_save_every_n)
    append_optional(command, "--imu-port", args.imu_port)
    append_optional(command, "--imu-baud", args.imu_baud)


def append_preflight_override_args(command: list[str], args: argparse.Namespace) -> None:
    append_optional(command, "--radar-config", args.radar_config)
    append_optional(command, "--reader-mode", getattr(args, "reader_mode", None))
    append_optional(command, "--camera-device", args.camera_device)
    append_optional(command, "--camera-width", args.camera_width)
    append_optional(command, "--camera-height", args.camera_height)
    append_optional(command, "--camera-fps", args.camera_fps)
    append_optional(command, "--camera-fourcc", args.camera_fourcc)
    append_optional(command, "--camera-format", args.camera_format)
    append_optional(command, "--camera-save-every-n", args.camera_save_every_n)
    append_optional(command, "--imu-port", args.imu_port)
    append_optional(command, "--imu-baud", args.imu_baud)


def append_optional(command: list[str], flag: str, value: Any) -> None:
    if value in (None, ""):
        return
    command.extend([flag, str(value)])


def controller_time_snapshot() -> dict[str, Any]:
    wall_time_s = time.time()
    return {
        "wall_time_s": wall_time_s,
        "wall_time_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(wall_time_s)),
        "suspicious": wall_time_s < 1_704_067_200,
    }


def run_text_command(command: list[str], timeout_seconds: float = 20.0) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=timeout_seconds,
        )
        return {
            "command": command,
            "returncode": completed.returncode,
            "stdout_tail": completed.stdout.splitlines()[-20:],
            "stderr_tail": completed.stderr.splitlines()[-20:],
        }
    except Exception as exc:
        return {
            "command": command,
            "returncode": -1,
            "stdout_tail": [],
            "stderr_tail": [str(exc)],
        }


def wired_route_issue(station_id: str, host: str, route: dict[str, Any]) -> str:
    if host not in {"192.168.55.1", "192.168.56.1"}:
        return ""
    device = str(route.get("dev") or "")
    gateway = str(route.get("gateway") or "")
    source = str(route.get("prefsrc") or "")
    expected_network = ipaddress.ip_network(f"{host}/24", strict=False)
    try:
        direct_source = ipaddress.ip_address(source) in expected_network
    except ValueError:
        direct_source = False
    if device and not device.startswith(("wl", "wlan")) and not gateway and direct_source:
        return ""
    path = f"{device or '未知接口'} / {source or '无源地址'}"
    if gateway:
        path += f"，经网关 {gateway}"
    return (f"{station_id}（{host}）当前路由：{path}；采集必须接回机器狗到该 NX 的有线接口，"
            "不能经调试 Wi-Fi")


def wired_station_route_issues(stations: list[dict[str, Any]]) -> list[str]:
    if os.name != "posix":
        return []
    issues = []
    for station in stations:
        host = str(station.get("host") or "")
        if host not in {"192.168.55.1", "192.168.56.1"}:
            continue
        result = run_text_command(["ip", "-j", "route", "get", host], timeout_seconds=3.0)
        try:
            routes = json.loads("\n".join(result["stdout_tail"]))
            route = routes[0] if result["returncode"] == 0 and routes else {}
        except (ValueError, IndexError, TypeError):
            route = {}
        if not route:
            issues.append(f"{station['station_id']}（{host}）没有可用路由；检查机器狗与雷达 NX 的有线连接")
            continue
        issue = wired_route_issue(station["station_id"], host, route)
        if issue:
            issues.append(issue)
    return issues


def run_controller_time_sync() -> dict[str, Any]:
    before = controller_time_snapshot()
    attempted: list[dict[str, Any]] = []

    helper = Path("/usr/local/sbin/nx-online-time-sync")
    if os.name == "posix" and helper.exists():
        if hasattr(os, "geteuid") and os.geteuid() == 0:
            attempted.append(run_text_command([str(helper), "fleet-controller"], timeout_seconds=45.0))
        elif shutil.which("sudo"):
            sudo_check = run_text_command(["sudo", "-n", "true"], timeout_seconds=5.0)
            if sudo_check["returncode"] == 0:
                attempted.append(
                    run_text_command(["sudo", "-n", str(helper), "fleet-controller"], timeout_seconds=45.0)
                )

    if shutil.which("chronyc") and hasattr(os, "geteuid") and os.geteuid() == 0:
        attempted.append(run_text_command(["chronyc", "online"], timeout_seconds=10.0))
        attempted.append(run_text_command(["chronyc", "burst", "4/8"], timeout_seconds=20.0))
        attempted.append(run_text_command(["chronyc", "makestep"], timeout_seconds=20.0))

    after = controller_time_snapshot()
    tracking = run_text_command(["chronyc", "tracking"], timeout_seconds=10.0) if shutil.which("chronyc") else None
    sync_check = (run_text_command(["chronyc", "waitsync", "5", "0.05", "0", "1"], timeout_seconds=8.0)
                  if os.name == "posix" and shutil.which("chronyc") else None)
    status = ("time_suspicious" if after["suspicious"] else
              "time_unsynchronized" if os.name == "posix" and
              (sync_check is None or sync_check["returncode"] != 0) else "ok")
    return {
        "status": status,
        "before": before,
        "after": after,
        "attempted": attempted,
        "chrony_tracking": tracking,
        "chrony_sync_check": sync_check,
        "warning": (
            "Controller clock is earlier than 2024-01-01 UTC; check network time sync before capture."
            if status == "time_suspicious" else
            "Machine-dog clock has no verified chrony/NTP synchronization; check Wi-Fi/NTP or wired radar0 time."
            if status == "time_unsynchronized" else ""
        ),
    }


def resolve_secret(env_name: str) -> str:
    if not env_name:
        return ""
    value = os.environ.get(env_name, "").strip()
    if value:
        return value
    password_files = {
        "RADAR_NX_PASSWORD": ".radar_nx_password",
        "RADAR_NX2_PASSWORD": ".radar_nx2_password",
    }
    if env_name in password_files:
        password_file = Path.home() / password_files[env_name]
        if password_file.exists():
            return password_file.read_text(encoding="utf-8").strip()
    return ""


def make_auto_name(prefix: str = "scene") -> str:
    return f"{sanitize_label(prefix)}_{time.strftime('%Y%m%d_%H%M%S')}"


def make_auto_capture_id(prefix: str = "exp") -> str:
    return f"{sanitize_label(prefix)}_{time.strftime('%Y%m%d_%H%M%S')}"


def resolve_operator_profile(config: dict[str, Any], requested_name: str) -> dict[str, Any]:
    profiles = config["operator_profiles"]
    canonical_name = canonical_operator_profile_name(requested_name)
    if canonical_name not in profiles:
        raise RuntimeError(
            f"Unknown operator profile '{requested_name}'. Available profiles: {', '.join(sorted(profiles))}"
        )
    return profiles[canonical_name]


def summarize_operator_profiles(config: dict[str, Any]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for name in sorted(config["operator_profiles"]):
        profile = config["operator_profiles"][name]
        rows.append(
            {
                "name": name,
                "label": profile["label"],
                "description": profile["description"],
                "duration_seconds": profile["duration_seconds"],
                "countdown_seconds": profile["countdown_seconds"],
                "result_mode": profile["result_mode"],
                "allow_no_camera": profile["allow_no_camera"],
                "allow_no_imu": profile["allow_no_imu"],
                "clock_snapshot_samples": profile["clock_snapshot_samples"],
                "selected_stations": profile["selected_stations"],
                "station_countdown_offsets": profile["station_countdown_offsets"],
                "station_overrides": profile["station_overrides"],
                "skip_preflight": profile["skip_preflight"],
                "notes": profile["notes"],
            }
        )
    return rows


def resolve_result_mode(config: dict[str, Any], args: argparse.Namespace) -> str:
    requested = getattr(args, "result_mode", None)
    if requested:
        return normalize_result_mode(requested, pull_results_fallback=config["pull_results"])
    if getattr(args, "no_pull", False):
        return "station-local-only"
    return str(config["result_mode"])


def resolve_clock_snapshot_samples(config: dict[str, Any], args: argparse.Namespace) -> int:
    requested = getattr(args, "clock_snapshot_samples", None)
    if requested is None:
        return int(config["clock_snapshots"]["samples"])
    return max(1, int(requested))


def station_countdown_offset_for(args: argparse.Namespace, station_id: str) -> float:
    offsets = getattr(args, "station_countdown_offsets", {}) or {}
    if not isinstance(offsets, dict):
        return 0.0
    try:
        return float(offsets.get(station_id, 0.0) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def station_override_map_for(args: argparse.Namespace, station_id: str) -> dict[str, Any]:
    overrides = getattr(args, "station_overrides", {}) or {}
    if not isinstance(overrides, dict):
        return {}
    station_override = overrides.get(station_id, {}) or {}
    return dict(station_override) if isinstance(station_override, dict) else {}


def apply_station_args(
    args: argparse.Namespace,
    station: dict[str, Any],
    *,
    apply_countdown_offset: bool,
) -> argparse.Namespace:
    updates = station_override_map_for(args, station["station_id"])
    if apply_countdown_offset:
        base_countdown = updates.get("countdown_seconds", getattr(args, "countdown_seconds", None))
        if base_countdown is not None:
            updates["countdown_seconds"] = max(
                0.0,
                float(base_countdown) + station_countdown_offset_for(args, station["station_id"]),
            )
    if not updates:
        return args
    return clone_args(args, **updates)


def build_capture_plan(args: argparse.Namespace, stations: list[dict[str, Any]]) -> list[dict[str, Any]]:
    base_duration = float(getattr(args, "duration_seconds", 0.0) or 0.0)
    base_countdown = float(getattr(args, "countdown_seconds", 0.0) or 0.0)
    plan: list[dict[str, Any]] = []
    for station in stations:
        offset = station_countdown_offset_for(args, station["station_id"])
        station_args = apply_station_args(args, station, apply_countdown_offset=True)
        plan.append(
            {
                "station_id": station["station_id"],
                "host": station["host"],
                "configured_host": station.get("configured_host", station["host"]),
                "host_resolution": station.get("host_resolution", {}),
                "duration_seconds": float(getattr(station_args, "duration_seconds", base_duration) or 0.0),
                "base_countdown_seconds": base_countdown,
                "countdown_offset_seconds": offset,
                "effective_countdown_seconds": float(getattr(station_args, "countdown_seconds", 0.0) or 0.0),
                "scheduled_start_wall_ns": getattr(station_args, "start_at_wall_ns", None),
                "radar_config": getattr(station_args, "radar_config", None),
                "reader_mode": getattr(station_args, "reader_mode", None),
            }
        )
    return plan


def apply_station_capture_args(args: argparse.Namespace, station: dict[str, Any]) -> argparse.Namespace:
    return apply_station_args(args, station, apply_countdown_offset=True)


def build_go_capture_args(args: argparse.Namespace, config: dict[str, Any]) -> argparse.Namespace:
    requested_mode = args.mode or config.get("default_operator_profile", "sync")
    profile = resolve_operator_profile(config, requested_mode)
    duration_seconds = float(args.seconds) if args.seconds is not None else float(profile["duration_seconds"])
    countdown_seconds = float(args.countdown) if args.countdown is not None else float(profile["countdown_seconds"])
    result_mode = "pull-full" if args.pull else str(profile["result_mode"])
    allow_no_camera = False if args.strict_camera else bool(profile["allow_no_camera"])
    allow_no_imu = False if args.strict_imu else bool(profile["allow_no_imu"])
    clock_snapshot_samples = (
        int(args.clock_snapshot_samples)
        if args.clock_snapshot_samples is not None
        else int(profile["clock_snapshot_samples"])
    )
    selected_stations = list(args.station) if getattr(args, "station", None) else list(profile["selected_stations"])
    return clone_args(
        args,
        command="capture",
        station=selected_stations,
        name=args.name.strip(),
        capture_id=(args.capture_id.strip() or make_auto_capture_id("exp")),
        duration_seconds=duration_seconds,
        countdown_seconds=countdown_seconds,
        allow_no_camera=allow_no_camera,
        allow_no_imu=allow_no_imu,
        skip_preflight=bool(getattr(args, "skip_preflight", False) or profile.get("skip_preflight", False)),
        no_pull=(result_mode != "pull-full"),
        result_mode=result_mode,
        mode_profile=profile["name"],
        mode_profile_label=profile["label"],
        mode_profile_description=profile["description"],
        mode_profile_notes=profile["notes"],
        station_overrides=dict(profile["station_overrides"]),
        station_countdown_offsets=dict(profile["station_countdown_offsets"]),
        radar_config=args.radar_config or config.get("default_radar_config", DEFAULT_RADAR_CONFIG),
        reader_mode=getattr(args, "reader_mode", None),
        camera_device=None,
        camera_width=None,
        camera_height=None,
        camera_fps=None,
        camera_fourcc=None,
        camera_format=None,
        camera_save_every_n=None,
        imu_port=None,
        imu_baud=None,
        clock_snapshot_samples=clock_snapshot_samples,
        operator_entrypoint="go",
    )


def clock_snapshots_enabled(config: dict[str, Any], args: argparse.Namespace) -> bool:
    if getattr(args, "no_clock_snapshots", False):
        return False
    return bool(config["clock_snapshots"]["enabled"])


def build_remote_helper_command(
    station: dict[str, Any],
    args: argparse.Namespace,
    subcommand: str,
    pull_to: Path | None = None,
    remote_session_dir: str = "",
    output_dir: Path | None = None,
) -> list[str]:
    command = [
        sys.executable,
        str(REMOTE_HELPER),
        "--host",
        station["host"],
    ]
    if station["user"]:
        command.extend(["--user", station["user"]])
    if station["port"]:
        command.extend(["--port", str(station["port"])])
    if station["ssh_config"]:
        command.extend(["--ssh-config", station["ssh_config"]])
    if station["key_file"]:
        command.extend(["--key-file", station["key_file"]])
    password = resolve_secret(station["password_env"])
    if password:
        command.extend(["--password", password])
    sudo_password = resolve_secret(station["sudo_password_env"])
    if sudo_password:
        command.extend(["--sudo-password", sudo_password])
    if station["remote_root"]:
        command.extend(["--remote-root", station["remote_root"]])
    if pull_to is not None:
        command.extend(["--pull-to", str(pull_to)])

    command.append(subcommand)

    if subcommand == "capture":
        command.extend(["--name", args.name])
        if args.capture_id:
            command.extend(["--capture-id", args.capture_id])
    elif subcommand == "arm":
        append_optional(command, "--radar-config", getattr(args, "radar_config", None))
    elif subcommand == "pull-session":
        if not remote_session_dir:
            raise RuntimeError("remote_session_dir is required for pull-session.")
        if output_dir is None:
            raise RuntimeError("output_dir is required for pull-session.")
        command.extend(["--session-dir", remote_session_dir, "--output-dir", str(output_dir)])

    if subcommand == "capture":
        append_common_override_args(command, args)
        if getattr(args, "allow_no_camera", False):
            command.append("--allow-no-camera")
        if getattr(args, "allow_no_imu", False):
            command.append("--allow-no-imu")
        if getattr(args, "skip_network_setup", False):
            command.append("--skip-network-setup")

    if subcommand == "preflight":
        append_preflight_override_args(command, args)
        if getattr(args, "allow_no_camera", False):
            command.append("--allow-no-camera")
        if getattr(args, "allow_no_imu", False):
            command.append("--allow-no-imu")
        if getattr(args, "skip_network_setup", False):
            command.append("--skip-network-setup")

    return command


def parse_json_output(text: str) -> dict[str, Any] | None:
    stripped = text.strip()
    if not stripped:
        return None
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        return None
    return parsed if isinstance(parsed, dict) else {"value": parsed}


def extract_remote_report(response: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(response, dict):
        return {}
    report = response.get("report")
    if isinstance(report, dict):
        return report
    return response


def run_remote_helper(
    station: dict[str, Any],
    args: argparse.Namespace,
    subcommand: str,
    pull_to: Path | None = None,
    status_callback: Callable[[dict[str, Any]], None] | None = None,
    cancel_event: threading.Event | None = None,
) -> dict[str, Any]:
    command = build_remote_helper_command(station, args, subcommand, pull_to=pull_to)
    if subcommand == "capture":
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        stderr_lines: list[str] = []

        def cancel_capture() -> None:
            if cancel_event is None:
                return
            while process.poll() is None and not cancel_event.wait(0.2):
                pass
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()

        if cancel_event is not None:
            threading.Thread(target=cancel_capture, daemon=True).start()

        def pump_stderr() -> None:
            assert process.stderr is not None
            for line in process.stderr:
                stderr_lines.append(line.rstrip("\n"))
                print(f"[{station['station_id']}] {line}", end="", file=sys.stderr, flush=True)
                if status_callback is not None:
                    phase = ("station_armed" if line.startswith("RADAR_ARMED_WAITING_FOR_TRIGGER ") else
                             "station_started" if line.startswith("RADAR_RECORDING_STARTED ") else
                             "station_saved" if line.startswith("RADAR_RAW_READY ") else "")
                    if phase:
                        try:
                            status_callback({"phase": phase, "station_id": station["station_id"]})
                        except Exception as exc:
                            print(f"UI progress callback failed: {exc}", file=sys.stderr, flush=True)

        stderr_thread = threading.Thread(target=pump_stderr, daemon=True)
        stderr_thread.start()
        assert process.stdout is not None
        stdout_text = process.stdout.read()
        returncode = process.wait()
        stderr_thread.join(timeout=2.0)
        stderr_text = "\n".join(stderr_lines)
    else:
        completed = subprocess.run(command, capture_output=True, text=True, check=False)
        stdout_text = completed.stdout
        stderr_text = completed.stderr
        returncode = completed.returncode

    response = parse_json_output(stdout_text)
    remote_report = extract_remote_report(response)
    remote_status = str(remote_report.get("status", "error")) if remote_report else "error"

    result: dict[str, Any] = {
        "station_id": station["station_id"],
        "host": station["host"],
        "command": subcommand,
        "returncode": returncode,
        "remote_status": remote_status,
        "response": response,
        "stderr_tail": stderr_text.splitlines()[-20:],
    }
    if response is None:
        result["stdout_tail"] = stdout_text.splitlines()[-20:]
        result["remote_status"] = "parse_failed"
    return result


def check_station_code_versions(
    stations: list[dict[str, Any]], args: argparse.Namespace
) -> dict[str, Any]:
    """Read release metadata only; this path never copies code automatically."""
    results = run_parallel(stations, lambda station: run_remote_helper(station, args, "code-info"))
    rows: list[dict[str, Any]] = []
    for result in results:
        report = extract_remote_report(result.get("response"))
        rows.append({
            "station_id": result.get("station_id"),
            "host": result.get("host"),
            "status": report.get("status") if report else result.get("remote_status"),
            "root": report.get("root", ""),
            "version": report.get("version", ""),
            "hash_ok": bool(report.get("hash_ok")),
            "manifest_sha256": report.get("manifest_sha256", ""),
            "detail": report.get("detail") or station_remote_issue(result),
        })
    verified = [row for row in rows if row["status"] == "ok" and row["hash_ok"]]
    versions = {str(row["version"]) for row in verified if row.get("version")}
    manifests = {str(row["manifest_sha256"]) for row in verified if row.get("manifest_sha256")}
    if len(stations) <= 1:
        status = "ok" if len(verified) == len(rows) else "warning"
        errors = [] if status == "ok" else ["当前雷达站没有可验证的版本清单；多雷达采集前请先执行“同步代码”"]
    elif len(verified) != len(rows):
        status = "mismatch"
        errors = ["代码版本不一致：至少一个雷达站缺少 VERSION/SHA256SUMS 或校验失败"]
    elif len(versions) != 1 or len(manifests) != 1:
        status = "mismatch"
        errors = ["代码版本不一致：两台 NX 的版本号或 SHA256SUMS 不一致，请先执行选项 4 同步代码"]
    else:
        status, errors = "ok", []
    return {"status": status, "stations": rows, "errors": errors}


def _direct_station_client(station: dict[str, Any]):
    if paramiko is None:
        raise RuntimeError("同步代码需要 paramiko，请先安装机器狗端依赖")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_config_path = Path(str(station.get("ssh_config") or "")).expanduser()
    lookup: dict[str, Any] = {}
    if ssh_config_path.exists():
        ssh_config = paramiko.SSHConfig()
        with ssh_config_path.open("r", encoding="utf-8") as handle:
            ssh_config.parse(handle)
        lookup = ssh_config.lookup(station["host"])
    kwargs: dict[str, Any] = {
        "hostname": str(lookup.get("hostname") or station["host"]),
        "port": int(lookup.get("port") or station.get("port") or 22),
        "username": str(lookup.get("user") or station.get("user") or "radar"),
        "timeout": 15,
        "banner_timeout": 15,
        "auth_timeout": 15,
        "allow_agent": True,
        "look_for_keys": True,
    }
    key_file = str(station.get("key_file") or "").strip()
    identities = [key_file] if key_file else list(lookup.get("identityfile") or [])
    if identities:
        kwargs["key_filename"] = [str(Path(item).expanduser()) for item in identities]
    password = resolve_secret(station.get("password_env", ""))
    if password:
        kwargs["password"] = password
    client.connect(**kwargs)
    return client


def _ssh_exec_text(client: Any, command: str) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(command)
    del stdin
    code = stdout.channel.recv_exit_status()
    return code, stdout.read().decode("utf-8", errors="replace"), stderr.read().decode("utf-8", errors="replace")


def _release_build_command(root: str, version: str) -> tuple[str, str]:
    root_q, version_q = shlex.quote(root), shlex.quote(version)
    archive = f"{root}/.fleet-release-{version}.tar.gz"
    archive_q = shlex.quote(archive)
    lines = [
        "set -eu",
        f"root={root_q}",
        f"version={version_q}",
        'tmp="$root/releases/.build-$version"',
        'release="$root/releases/$version"',
        f"archive={archive_q}",
        'rm -rf "$tmp" "$release"',
        'mkdir -p "$tmp" "$root/releases" "$root/station"',
        'if [ ! -f "$root/station/radar_station_config.json" ] && [ -f "$root/radar_station_config.json" ]; then cp -p "$root/radar_station_config.json" "$root/station/radar_station_config.json"; fi',
        "tar -C \"$root\" --exclude='./data' --exclude='./logs' --exclude='./.git' --exclude='./releases' --exclude='./current' --exclude='./station' --exclude='./radar_station_config.json' --exclude='./__pycache__' --exclude='*/__pycache__' --exclude='*.pyc' --exclude='*backup*' --exclude='*.bak' -cf - . | tar -C \"$tmp\" -xf -",
        'rm -f "$tmp/radar_station_config.json"',
        'ln -s ../../station/radar_station_config.json "$tmp/radar_station_config.json"',
        'printf \'%s\\n\' "$version" > "$tmp/VERSION"',
        '(cd "$tmp" && find . -type f ! -name VERSION ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum) > "$tmp/SHA256SUMS"',
        'mv "$tmp" "$release"',
        'rm -f "$root/current.next"',
        'ln -s "releases/$version" "$root/current.next"',
        'mv -Tf "$root/current.next" "$root/current"',
        'tar -C "$release" -czf "$archive" .',
        'printf \'%s\\n\' "$archive"',
    ]
    return "\n".join(lines), archive


def _install_release_command(root: str, version: str, archive: str) -> str:
    root_q, version_q, archive_q = shlex.quote(root), shlex.quote(version), shlex.quote(archive)
    lines = [
        "set -eu",
        f"root={root_q}",
        f"version={version_q}",
        f"archive={archive_q}",
        'mkdir -p "$root/releases" "$root/station"',
        'if [ ! -f "$root/station/radar_station_config.json" ] && [ -f "$root/radar_station_config.json" ]; then cp -p "$root/radar_station_config.json" "$root/station/radar_station_config.json"; fi',
        'tmp="$root/releases/.incoming-$version"',
        'release="$root/releases/$version"',
        'rm -rf "$tmp" "$release"',
        'mkdir -p "$tmp"',
        'tar -xzf "$archive" -C "$tmp"',
        '(cd "$tmp" && sha256sum -c SHA256SUMS)',
        'mv "$tmp" "$release"',
        'rm -f "$root/current.next"',
        'ln -s "releases/$version" "$root/current.next"',
        'mv -Tf "$root/current.next" "$root/current"',
        'rm -f "$archive"',
        "restart='not_requested'",
        "if systemctl --user list-unit-files nx-radar-ui.service >/dev/null 2>&1; then if systemctl --user restart nx-radar-ui.service >/dev/null 2>&1; then restart='restarted'; else restart='restart_failed'; fi; fi",
        'printf \'%s\\n\' "$restart"',
    ]
    return "\n".join(lines)


def run_sync_code(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]]) -> tuple[int, dict[str, Any]]:
    station_map = {station["station_id"]: station for station in stations}
    if "radar0" not in station_map or "radar1" not in station_map:
        return 2, {"status": "error", "message": "同步代码需要同时选中 radar0（NX1）和 radar1（NX2）。"}
    source, target = station_map["radar0"], station_map["radar1"]
    version = "v" + time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    source_root = source.get("release_root") or source["remote_root"]
    target_root = target.get("release_root") or target["remote_root"]
    local_archive = Path(config["output_root"]).expanduser() / f".fleet-release-{version}.tar.gz"
    local_archive.parent.mkdir(parents=True, exist_ok=True)
    source_client = target_client = None
    try:
        source_client = _direct_station_client(source)
        build_command, source_archive = _release_build_command(source_root, version)
        code, stdout, stderr = _ssh_exec_text(source_client, build_command)
        if code != 0:
            return 2, {"status": "error", "message": f"NX1 生成版本包失败：{(stderr or stdout).strip()[-500:]}"}
        with source_client.open_sftp() as sftp:
            sftp.get(source_archive, str(local_archive))
        _ssh_exec_text(source_client, f"rm -f {shlex.quote(source_archive)}")
        target_client = _direct_station_client(target)
        target_archive = f"{target_root}/.incoming/{version}.tar.gz"
        prepare = f"mkdir -p {shlex.quote(target_root + '/.incoming')} {shlex.quote(target_root + '/releases')} {shlex.quote(target_root + '/station')}"
        code, _, stderr = _ssh_exec_text(target_client, prepare)
        if code != 0:
            return 2, {"status": "error", "message": f"NX2 准备临时目录失败：{stderr.strip()[-500:]}"}
        with target_client.open_sftp() as sftp:
            sftp.put(str(local_archive), target_archive)
        install_code, install_out, install_err = _ssh_exec_text(target_client, _install_release_command(target_root, version, target_archive))
        if install_code != 0:
            return 2, {"status": "error", "message": f"NX2 校验或切换失败：{(install_err or install_out).strip()[-700:]}"}
        restart_status = (install_out.strip().splitlines() or ["unknown"])[-1]
        return 0, {"status": "ok", "version": version,
                    "source": {"station_id": "radar0", "root": source_root},
                    "target": {"station_id": "radar1", "root": target_root, "restart": restart_status},
                    "message": "NX1 版本包已生成，NX2 SHA256 校验通过并完成 current 原子切换。"}
    except Exception as exc:
        return 2, {"status": "error", "message": f"同步代码失败：{exc}"}
    finally:
        for client in (source_client, target_client):
            if client is not None:
                client.close()
        try:
            local_archive.unlink()
        except OSError:
            pass


def check_radar_ready(stations: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    results = run_parallel(
        stations,
        lambda station: run_remote_helper(station, args, "ready-status"),
    )
    errors: list[str] = []
    rows: list[dict[str, Any]] = []
    by_id = {item["station_id"]: item for item in results}
    for station in stations:
        station_id = station["station_id"]
        result = by_id[station_id]
        report = extract_remote_report(result.get("response"))
        expected = station_override_map_for(args, station_id).get("radar_config") or getattr(args, "radar_config", None)
        ready = result["remote_status"] == "ready" and report.get("radar_config") == expected
        rows.append({"station_id": station_id, "host": station["host"], "ready": ready, "radar_config": report.get("radar_config"), "expected_radar_config": expected})
        if not ready:
            detail = ("已预配置但雷达参数与当前模式不一致，请重新执行采前检查及预配置"
                      if result["remote_status"] == "ready" else station_remote_issue(result))
            errors.append(f"{station_id}: {detail}")
    return {"status": "ready" if not errors else "not_ready", "stations": rows, "errors": errors}


def station_remote_issue(result: dict[str, Any]) -> str:
    report = extract_remote_report(result.get("response"))
    detail = str(report.get("message") or " ".join(result.get("stderr_tail") or []))
    if any(token in detail for token in ("BadHostKeyException", "REMOTE HOST IDENTIFICATION HAS CHANGED", "Host key for server")):
        return "SSH 主机指纹不匹配；核对 IP 是否仍指向这台 NX，确认设备身份后再更新信任记录"
    if any(token in detail for token in ("NoValidConnectionsError", "No route to host", "timed out", "Unable to connect")):
        if result.get("host") in {"192.168.55.1", "192.168.56.1"}:
            return "有线 SSH/IP 不可达；检查机器狗到该 NX 的 USB/网线、接口地址和供电"
        return "调试 Wi-Fi 或 SSH 不可达；检查当前热点连接和 NX 地址"
    if any(token in detail for token in ("PasswordRequiredException", "private key file is encrypted")):
        return "SSH 私钥需要口令；检查私钥文件和口令，或改用已配置的登录密码"
    if any(token in detail for token in ("AuthenticationException", "Permission denied", "No authentication methods")):
        return "SSH 认证失败；检查该雷达站的用户名、登录密码或私钥授权"
    return (str(report.get("message") or "") or detail.splitlines()[-1] if detail else "未预配置，请先执行采前检查及预配置")[:240]


def run_remote_pull_session(
    station: dict[str, Any],
    args: argparse.Namespace,
    remote_session_dir: str,
    output_dir: Path,
) -> dict[str, Any]:
    command = build_remote_helper_command(
        station,
        args,
        "pull-session",
        remote_session_dir=remote_session_dir,
        output_dir=output_dir,
    )
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    response = parse_json_output(completed.stdout)
    remote_status = str(response.get("status", "error")) if isinstance(response, dict) else "parse_failed"
    result: dict[str, Any] = {
        "station_id": station["station_id"],
        "host": station["host"],
        "command": "pull-session",
        "returncode": completed.returncode,
        "remote_status": remote_status,
        "response": response,
        "stderr_tail": completed.stderr.splitlines()[-20:],
    }
    if response is None:
        result["stdout_tail"] = completed.stdout.splitlines()[-20:]
    return result


def parse_json_from_completed_process(completed: subprocess.CompletedProcess[str]) -> dict[str, Any] | None:
    response = parse_json_output(completed.stdout)
    if response is not None:
        return response
    return parse_json_output(completed.stderr)


def run_clock_snapshot(
    config: dict[str, Any],
    args: argparse.Namespace,
    stations: list[dict[str, Any]],
    phase: str,
) -> dict[str, Any]:
    if not clock_snapshots_enabled(config, args):
        return {
            "phase": phase,
            "status": "disabled",
            "returncode": 0,
            "response": {
                "status": "disabled",
                "message": "Clock snapshots disabled for this capture.",
            },
            "stderr_tail": [],
        }

    command = [
        shutil.which("python3") or sys.executable,
        str(FLEET_CLOCK_HELPER),
        "--config",
        str(Path(config["config_path"]).expanduser()),
        "--samples",
        str(resolve_clock_snapshot_samples(config, args)),
        "--max-offset-ms",
        str(config["clock_snapshots"]["max_offset_ms"]),
    ]
    for station in stations:
        command.extend(["--station", station["station_id"]])

    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    response = parse_json_from_completed_process(completed)
    status = str(response.get("status", "error")) if isinstance(response, dict) else "parse_failed"
    result: dict[str, Any] = {
        "phase": phase,
        "status": status,
        "returncode": completed.returncode,
        "response": response,
        "stderr_tail": completed.stderr.splitlines()[-20:],
    }
    if response is None:
        result["stdout_tail"] = completed.stdout.splitlines()[-20:]
    return result


def run_parallel(
    stations: list[dict[str, Any]],
    worker: callable,
) -> list[dict[str, Any]]:
    results: list[dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(stations)) as executor:
        future_map = {executor.submit(worker, station): station for station in stations}
        for future in concurrent.futures.as_completed(future_map):
            station = future_map[future]
            try:
                results.append(future.result())
            except Exception as exc:
                results.append(
                    {
                        "station_id": station["station_id"],
                        "host": station["host"],
                        "command": "unknown",
                        "returncode": -1,
                        "remote_status": "local_error",
                        "response": {
                            "status": "error",
                            "message": str(exc),
                        },
                        "stderr_tail": [],
                    }
                )
    results.sort(key=lambda item: item["station_id"])
    return results


def run_hardware_trigger_stations(
    stations: list[dict[str, Any]], args: argparse.Namespace, session_dir: Path,
    progress: Callable[[dict[str, Any]], None] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    station_ids = {station["station_id"] for station in stations}
    if station_ids != {"radar0", "radar1"}:
        raise TriggerError("Hardware TDM requires both radar0 and radar1.")
    count = hardware_frame_count(float(args.duration_seconds))
    port = str(getattr(args, "stm32_port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"))
    report_path = session_dir / "reports" / "stm32_trigger.json"
    cancel_event = threading.Event()
    armed: set[str] = set()
    armed_lock = threading.Lock()
    report: dict[str, Any] = {"status": "not_started", "requested_pairs": count,
                              "period_ms": 650, "station_roles": {"radar0": "A", "radar1": "B"},
                              "port": port, "report_path": str(report_path)}

    def on_status(event: dict[str, Any]) -> None:
        if event.get("phase") == "station_armed":
            with armed_lock:
                armed.add(str(event["station_id"]))
        if progress is not None:
            progress(event)

    try:
        with Stm32Trigger(port, event_log_path=report_path.with_suffix(".jsonl")) as trigger:
            try:
                report["idle_ack"] = trigger.confirm_idle()
                with concurrent.futures.ThreadPoolExecutor(max_workers=len(stations)) as executor:
                    try:
                        futures = {
                            executor.submit(run_remote_helper, station,
                                            apply_station_capture_args(args, station), "capture",
                                            status_callback=on_status, cancel_event=cancel_event): station
                            for station in stations
                        }
                        deadline = time.monotonic() + 60.0
                        while time.monotonic() < deadline:
                            with armed_lock:
                                all_armed = armed == station_ids
                            if all_armed or any(future.done() for future in futures):
                                break
                            time.sleep(0.05)
                        with armed_lock:
                            all_armed = armed == station_ids
                        if not all_armed:
                            missing = ", ".join(sorted(station_ids - armed))
                            raise TriggerError(f"STM32 was not started: radar stations did not arm ({missing}).")
                        if any(future.done() for future in futures):
                            raise TriggerError("STM32 was not started: a radar station exited after arming.")
                        if progress is not None:
                            progress({"phase": "trigger_armed", "pairs": count})
                        ack = trigger.start(count)
                        report["status"] = "started"
                        report["ack"] = ack
                        if progress is not None:
                            progress({"phase": "trigger_started", "pairs": count,
                                      "ack_wall_time_ns": ack["host_wall_time_ns"]})
                        results = [future.result(timeout=max(30.0, count * 0.7 + 20.0)) for future in futures]
                        results.sort(key=lambda item: item["station_id"])
                        done = trigger.wait_done(count, 3.0)
                        report["status"] = "done"
                        report["done"] = done
                        return results, report
                    except Exception:
                        cancel_event.set()
                        raise
            except Exception as exc:
                cancel_event.set()
                report["status"] = "error"
                report["error"] = str(exc)
                raise
            finally:
                trigger.save(report_path, count, report["status"], report.get("error", ""))
                report["events"] = list(trigger.events)
    except Exception:
        write_json(report_path, report)
        raise


def sanitize_label(label: str) -> str:
    cleaned = "".join(ch.lower() if ch.isalnum() else "_" for ch in label.strip())
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.strip("_") or "session"


def make_session_dir(output_root: Path, controller_id: str, label: str) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base_name = f"{time.strftime('%Y%m%d_%H%M%S')}__{sanitize_label(controller_id)}__{sanitize_label(label)}"
    session_dir = output_root / base_name
    suffix = 1
    while session_dir.exists():
        session_dir = output_root / f"{base_name}_{suffix:02d}"
        suffix += 1
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        json.dump(payload, output_file, indent=2)


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output_file:
        output_file.write(text)


def save_station_password(station_id: str, password: str) -> Path:
    """Store a station login password outside JSON, with mode 0600."""
    path = Path.home() / (".radar_nx_password" if station_id == "radar0" else ".radar_nx2_password")
    staged = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    staged.write_text(password.rstrip("\n") + "\n", encoding="utf-8")
    os.chmod(staged, 0o600)
    os.replace(staged, path)
    return path


def update_station_ip_config(
    config_path: Path,
    raw_config: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[int, dict[str, Any]]:
    stations = raw_config.get("stations")
    if not isinstance(stations, list):
        raise RuntimeError("Fleet config must contain a stations list.")

    station_map: dict[str, dict[str, Any]] = {}
    for station in stations:
        if not isinstance(station, dict):
            continue
        station_id = str(station.get("station_id", "")).strip()
        if station_id:
            station_map[station_id] = station

    for station_id in ("radar0", "radar1"):
        if station_id not in station_map:
            station = {
                "station_id": station_id,
                "host": "",
                "enabled": False,
                "notes": [],
            }
            stations.append(station)
            station_map[station_id] = station

    changes: list[str] = []
    for station_id, value in (("radar0", args.radar0), ("radar1", args.radar1)):
        ip = str(value or "").strip()
        if not ip:
            continue
        station = station_map[station_id]
        old_ip = str(station.get("host", ""))
        station["host"] = ip
        station["enabled"] = True
        station["notes"] = [f"Configured on dog NX by set-ips at {time.strftime('%Y-%m-%d %H:%M:%S')}."]
        changes.append(f"{station_id}: {old_ip or '<empty>'} -> {ip}")

    for station_id in ("radar0", "radar1"):
        station = station_map[station_id]
        user = str(getattr(args, f"{station_id}_user", "") or "").strip()
        password = getattr(args, f"{station_id}_password", None)
        if user:
            old_user = str(station.get("user", "")) or "<default>"
            station["user"] = user
            changes.append(f"{station_id}: user {old_user} -> {user}")
        if password not in (None, ""):
            env_name = "RADAR_NX_PASSWORD" if station_id == "radar0" else "RADAR_NX2_PASSWORD"
            station["password_env"] = env_name
            station["sudo_password_env"] = env_name
            save_station_password(station_id, str(password))
            changes.append(f"{station_id}: password saved to credential file")

    if args.disable_radar0:
        station_map["radar0"]["enabled"] = False
        changes.append("radar0 disabled")
    if args.disable_radar1:
        station_map["radar1"]["enabled"] = False
        changes.append("radar1 disabled")
    if args.enable_radar1:
        if not str(station_map["radar1"].get("host", "")).strip():
            raise RuntimeError("--enable-radar1 requires --radar1 or an existing radar1 host.")
        station_map["radar1"]["enabled"] = True
        changes.append("radar1 enabled")

    if not changes:
        changes.append("no changes requested")

    config_path = config_path.expanduser()
    backup_path = config_path.with_name(
        f"{config_path.name}.bak_{time.strftime('%Y%m%d_%H%M%S')}"
    )
    if config_path.exists():
        shutil.copy2(config_path, backup_path)
    write_json(config_path, raw_config)

    normalized = normalize_config(raw_config, config_path)
    enabled_stations = [station for station in normalized["stations"] if station["enabled"]]
    reachability = []
    if args.check and enabled_stations:
        for station in enabled_stations:
            reachability.append(check_station_reachability(station))

    payload = {
        "status": "ok",
        "config_path": str(config_path),
        "backup_path": str(backup_path) if backup_path.exists() else "",
        "changes": changes,
        "stations": normalized["stations"],
        "enabled_stations": [
            {"station_id": station["station_id"], "host": station["host"]}
            for station in enabled_stations
        ],
        "reachability": reachability,
    }
    return 0, payload


def check_station_reachability(station: dict[str, Any]) -> dict[str, Any]:
    host = str(station.get("host", "")).strip()
    ping = run_text_command(["ping", "-c", "1", "-W", "2", host], timeout_seconds=5.0) if host else {}
    tcp = (
        run_text_command(["bash", "-lc", f"timeout 3 bash -lc '</dev/tcp/{shlex.quote(host)}/22'"], timeout_seconds=5.0)
        if host and shutil.which("bash")
        else {}
    )
    return {
        "station_id": station.get("station_id"),
        "host": host,
        "ping_ok": ping.get("returncode") == 0,
        "ssh_port_open": tcp.get("returncode") == 0,
        "ping_tail": ping.get("stdout_tail", [])[-4:],
        "tcp_tail": (tcp.get("stdout_tail", []) + tcp.get("stderr_tail", []))[-4:],
    }


def controller_imu_requested(config: dict[str, Any], args: argparse.Namespace) -> bool:
    controller_imu = config["controller_imu"]
    return bool(controller_imu.get("enabled")) and not getattr(args, "no_controller_imu", False)


def controller_host_supported() -> tuple[bool, str]:
    if os.name != "posix":
        return False, "Controller-side IMU capture requires a POSIX host."
    if shutil.which("bash") is None:
        return False, "bash is not available on the controller host."
    return True, ""


def build_controller_imu_command(
    controller_imu: dict[str, Any],
    controller_id: str,
    subcommand: str,
    *,
    output_dir: Path | None = None,
    capture_id: str = "",
    name: str = "",
) -> list[str]:
    python_executable = shutil.which("python3") or sys.executable
    helper_command = [
        python_executable,
        str(CONTROLLER_IMU_HELPER),
        subcommand,
        "--topic",
        controller_imu["topic"],
    ]
    for key in ("odom_topic", "sport_topic", "lidar_imu_topic"):
        if controller_imu.get(key):
            helper_command.extend(["--" + key.replace("_", "-"), controller_imu[key]])
    if subcommand == "preflight":
        helper_command.extend(["--timeout-seconds", str(max(6.0, controller_imu["timeout_seconds"]))])
    elif subcommand == "record":
        if output_dir is None:
            raise RuntimeError("output_dir is required for controller IMU recording.")
        helper_command.extend(
            [
                "--output-dir",
                str(output_dir),
                "--controller-id",
                controller_id,
                "--capture-id",
                capture_id,
                "--name",
                name,
            ]
        )
    else:
        raise RuntimeError(f"Unsupported controller IMU subcommand: {subcommand}")

    shell_steps = ["set -eo pipefail"]
    for setup_script in controller_imu["ros_setup_scripts"]:
        shell_steps.append(f"source {shlex.quote(str(Path(setup_script).expanduser()))}")
    shell_steps.append("exec " + " ".join(shlex.quote(part) for part in helper_command))
    return ["bash", "-lc", " && ".join(shell_steps)]


def disabled_controller_imu_result(required: bool, reason: str) -> dict[str, Any]:
    return {
        "command": "controller_imu",
        "enabled": False,
        "required": required,
        "status": "disabled",
        "returncode": 0,
        "response": {
            "status": "disabled",
            "message": reason,
        },
        "stderr_tail": [],
    }


def skipped_controller_imu_result(required: bool, reason: str) -> dict[str, Any]:
    return {
        "command": "controller_imu",
        "enabled": True,
        "required": required,
        "status": "skipped",
        "returncode": 0,
        "response": {
            "status": "skipped",
            "message": reason,
        },
        "stderr_tail": [],
    }


def run_controller_imu_preflight(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    controller_imu = config["controller_imu"]
    if not controller_imu_requested(config, args):
        return disabled_controller_imu_result(bool(controller_imu.get("required")), "Controller IMU preflight disabled.")

    supported, reason = controller_host_supported()
    if not supported:
        return skipped_controller_imu_result(bool(controller_imu.get("required")), reason)

    command = build_controller_imu_command(controller_imu, config["controller_id"], "preflight")
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False,
                                   timeout=max(20., float(controller_imu["timeout_seconds"])+12.))
    except subprocess.TimeoutExpired:
        return {"command":"controller_imu_preflight", "enabled":True,
                "required":bool(controller_imu.get("required")), "status":"preflight_failed", "returncode":2,
                "response":{"error":"Robot telemetry preflight timed out; recording was not started."},
                "integrity_errors":["Robot telemetry preflight timed out."], "stderr_tail":[]}
    response = parse_json_output(completed.stdout)
    status = str(response.get("status", "error")) if isinstance(response, dict) else "parse_failed"

    result: dict[str, Any] = {
        "command": "controller_imu_preflight",
        "enabled": True,
        "required": bool(controller_imu.get("required")),
        "topic": controller_imu["topic"],
        "returncode": completed.returncode,
        "status": status,
        "response": response,
        "stderr_tail": completed.stderr.splitlines()[-20:],
    }
    if response is None:
        result["stdout_tail"] = completed.stdout.splitlines()[-20:]
    return validate_required_telemetry(result, "preflight")


def read_log_tail(path: Path, limit: int = 20) -> list[str]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", errors="replace") as log_file:
        return log_file.read().splitlines()[-limit:]


def start_controller_imu_capture(
    config: dict[str, Any],
    args: argparse.Namespace,
    session_dir: Path,
    reports_dir: Path,
    capture_id: str,
) -> dict[str, Any]:
    controller_imu = config["controller_imu"]
    if not controller_imu_requested(config, args):
        return disabled_controller_imu_result(bool(controller_imu.get("required")), "Controller IMU capture disabled.")

    supported, reason = controller_host_supported()
    if not supported:
        return skipped_controller_imu_result(bool(controller_imu.get("required")), reason)

    output_dir = session_dir / "controller_imu"
    log_path = reports_dir / "controller_imu_capture.log"
    command = build_controller_imu_command(
        controller_imu,
        config["controller_id"],
        "record",
        output_dir=output_dir,
        capture_id=capture_id,
        name=args.name,
    )
    log_handle = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
    except Exception as exc:
        log_handle.close()
        return {
            "command": "controller_imu_capture",
            "enabled": True,
            "required": bool(controller_imu.get("required")),
            "status": "error",
            "returncode": -1,
            "response": {
                "status": "error",
                "message": str(exc),
            },
            "stderr_tail": [],
            "log_path": str(log_path),
        }

    return {
        "command": "controller_imu_capture",
        "enabled": True,
        "required": bool(controller_imu.get("required")),
        "status": "recording",
        "returncode": None,
        "response": None,
        "stderr_tail": [],
        "process": process,
        "log_handle": log_handle,
        "log_path": str(log_path),
        "manifest_path": str(output_dir / "imu_manifest.json"),
    }


def stop_controller_imu_capture(record: dict[str, Any]) -> dict[str, Any]:
    process = record.pop("process", None)
    log_handle = record.pop("log_handle", None)
    manifest_path_value = record.get("manifest_path", "")
    manifest_path = Path(manifest_path_value) if manifest_path_value else None
    log_path = Path(record["log_path"]) if record.get("log_path") else None

    try:
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        if log_handle is not None:
            log_handle.close()
    finally:
        if log_handle is not None and not log_handle.closed:
            log_handle.close()

    if process is not None:
        record["returncode"] = process.returncode

    manifest: dict[str, Any] | None = None
    if manifest_path is not None and manifest_path.exists():
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            manifest = json.load(manifest_file)

    if manifest is not None:
        record["status"] = str(manifest.get("status", "error"))
        record["response"] = {
            "status": record["status"],
            "manifest": manifest,
        }
    elif record.get("status") == "recording":
        record["status"] = "error"
        record["response"] = {
            "status": "error",
            "message": "Controller IMU manifest was not produced.",
        }

    if log_path is not None:
        record["stderr_tail"] = read_log_tail(log_path)

    return validate_required_telemetry(record, "capture")


def controller_imu_blocks_result(result: dict[str, Any]) -> bool:
    if not result:
        return False
    if not bool(result.get("required")):
        return False
    return str(result.get("status", "")) != "ok"


def validate_required_telemetry(result: dict[str, Any], phase: str) -> dict[str, Any]:
    """Raw IMU supplies acceleration/gyro; valid pose and velocity must come from odometry."""
    if not result.get("required"):
        return result
    response = result.get("response") or {}
    manifest = response.get("manifest", response)
    streams = manifest.get("streams") or {}
    errors: list[str] = []
    if result.get("status") != "ok" or result.get("returncode", 0) != 0:
        errors.append(str(response.get("error") or response.get("message") or "Required robot telemetry failed."))
    for name in ("imu", "odom"):
        stream = streams.get(name) or {}
        try:
            if int(stream.get("sample_count", 0)) < 2:
                errors.append(f"Required {name} stream has insufficient samples.")
            if int(stream["publisher_count"]) != 1:
                errors.append(f"{name}: expected exactly one publisher; check sensor runtime and duplicate SLAM instances.")
            if phase == "preflight" and float(stream["received_duration_seconds"]) < 4.0:
                errors.append(f"{name}: continuous observation is shorter than 4 seconds.")
            if float(stream["last_receive_age_seconds"]) > 0.5:
                errors.append(f"{name}: stream stopped before the check completed.")
            for field in ("nonfinite_sample_count", "invalid_message_stamp_count", "out_of_order_message_stamp_count"):
                if int(stream[field]) != 0:
                    errors.append(f"{name}: {field} is nonzero.")
            if name == "odom" and int(stream["invalid_orientation_count"]) != 0:
                errors.append("Odometry has invalid pose orientation.")
            for field, limit in (("max_receive_gap_seconds", 0.5), ("max_abs_source_clock_delta_seconds", 1.0)):
                value = float(stream[field])
                if not math.isfinite(value) or value < 0 or value > limit:
                    errors.append(f"{name}: {field} must be within 0..{limit}.")
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"Required {name} quality metadata is missing or invalid.")
    result["integrity_errors"] = errors
    if errors:
        result["status"] = "preflight_failed" if phase == "preflight" else "capture_failed"
        response["error"] = "; ".join(errors)
        result["response"] = response
    return result


def controller_video_blocks_result(result: dict[str, Any]) -> bool:
    return bool(result.get("required")) and result.get("status") != "ok"


def joint_clock_issues(snapshot: dict[str, Any], station_ids: list[str], limit_ms: float) -> list[str]:
    errors: list[str] = []
    if snapshot.get("status") != "ok":
        errors.append("Cross-host clock snapshot did not succeed.")
    response = snapshot.get("response") or {}
    by_id = {item.get("station_id"): item for item in response.get("stations", []) if isinstance(item, dict)}
    for station_id in station_ids:
        item = by_id.get(station_id) or {}
        try:
            offset = float(item.get("best_effort_offset_ms") if item.get("best_effort_offset_ms") is not None else item["median_offset_ms"])
            bound = float(item["max_possible_abs_offset_ms"])
            if (item.get("measurement_method") not in ("persistent_ssh_four_timestamp", "authenticated_udp_four_timestamp")
                    or item.get("status") != "ok" or not math.isfinite(offset)
                    or not math.isfinite(bound) or bound < abs(offset) or bound > limit_ms):
                errors.append(f"{station_id}: clock offset exceeds {limit_ms} ms or clock check failed.")
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"{station_id}: clock evidence is missing.")
    return errors


def complete_joint_integrity(
    config: dict[str, Any], station_records: list[dict[str, Any]],
    telemetry: dict[str, Any], video: dict[str, Any],
    clocks: dict[str, Any], session_dir: Path,
) -> dict[str, Any]:
    if not config.get("require_complete_joint_capture"):
        return {"status": "not_required", "errors": [], "warnings": []}
    errors: list[str] = []
    warnings = list(telemetry.get("integrity_errors") or [])
    if telemetry.get("status") != "ok":
        warnings.append("Robot IMU/odometry is unavailable or incomplete (optional).")
    if config.get("controller_video", {}).get("enabled") and video.get("status") != "ok":
        warnings.append("Robot video is unavailable or incomplete (optional).")
    controller = extract_controller_imu_manifest(telemetry)
    video_manifest = extract_controller_imu_manifest(video)
    if config.get("controller_video", {}).get("enabled") and (video_manifest.get("status") != "ok" or video_manifest.get("error")):
        warnings.append("Robot video manifest reports an error or is missing (optional).")
    if config.get("controller_video", {}).get("enabled") and video_manifest.get("status") == "ok" and config.get("controller_video", {}).get("source") == "ros_topic":
        decoded = video_manifest.get("decode_validation") or {}
        expected_height = {"video720p": 720, "video360p": 360, "video180p": 180}.get(config["controller_video"]["field"])
        if (decoded.get("status") != "ok" or decoded.get("errors") or
                int(decoded.get("decoded_frames") or 0) != int(video_manifest.get("saved_frames") or 0) or
                int(decoded.get("decoded_frames") or 0) < 2 or decoded.get("height") != expected_height):
            warnings.append("Robot video lacks complete decode/count/resolution validation (optional).")
    windows: list[tuple[str, int, int]] = []
    camera_required_by_station = {
        station["station_id"]: station.get("camera_required", True)
        for station in config["stations"]
    }
    for station in station_records:
        station_id = station["station_id"]
        manifest = extract_station_capture_manifest(station.get("capture"))
        camera = manifest.get("camera") or {}
        if manifest.get("status") != "ok":
            errors.append(f"{station_id}: required radar capture did not succeed.")
        radar = manifest.get("radar") or {}
        packet_stats = radar.get("packet_stats") or {}
        if (not packet_stats.get("raw_size_matches") or
                packet_stats.get("content_quality_status") != "ok" or
                int(packet_stats.get("usable_radar_frames_by_content") or 0) < int(radar.get("num_frames") or 0)):
            errors.append(f"{station_id}: radar raw data failed frame count or content quality checks.")
        camera_invalid = (not camera.get("enabled") or camera.get("error") or
                int(camera.get("saved_frames") or 0) < 2 or
                camera.get("covers_radar_window") is not True or
                camera.get("eos_finalized") is not True or camera.get("integrity_errors"))
        if camera_invalid:
            detail = f"{station_id}: external camera is disabled, interrupted, or does not cover the radar window."
            (errors if camera_required_by_station.get(station_id, True) else warnings).append(detail)
        try:
            timing = manifest["timing"]
            origin_wall = int(timing["requested_start_wall_time_ns"])
            origin_mono = int(timing["requested_start_monotonic_ns"])
            start_mono = (timing.get("radar_first_packet_receive_monotonic_ns")
                          if timing.get("radar_start_timestamp_kind") == "host_udp_receive_proxy"
                          else timing.get("radar_sensor_start_monotonic_ns"))
            start = origin_wall + int(start_mono) - origin_mono
            stop = origin_wall + int(timing["radar_sensor_stop_monotonic_ns"]) - origin_mono
            if not 0 < start < stop:
                raise ValueError("Invalid radar interval")
            windows.append((station_id, start, stop))
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"{station_id}: radar timing metadata is missing or invalid.")
    if not station_records:
        errors.append("No required radar stations were captured.")
    common_overlap_seconds = None
    radar_start_skew_ms = None
    if len(windows) > 1:
        starts = [start for _, start, _ in windows]
        common_overlap_seconds = max(0.0, (min(stop for _, _, stop in windows) - max(starts)) / 1e9)
        radar_start_skew_ms = (max(starts) - min(starts)) / 1e6
        if common_overlap_seconds == 0:
            errors.append("Required radar recording windows do not overlap.")
        elif radar_start_skew_ms > 100:
            warnings.append(f"Radar start skew is {radar_start_skew_ms:.1f} ms; use timestamps to select the common window.")
    station_ids = [station["station_id"] for station in station_records]
    for phase in ("before_capture", "after_capture"):
        clock_issues = [f"{phase}: {error}" for error in joint_clock_issues(
            clocks.get(phase) or {}, station_ids, float(config["clock_snapshots"]["max_offset_ms"]))]
        (errors if config["clock_snapshots"].get("required", True) else warnings).extend(clock_issues)
    # Cross-host monotonic clocks must never be compared. Use wall time only after clock checks.
    streams = dict(controller.get("streams") or {})
    if config.get("controller_video", {}).get("enabled"):
        streams["robot_video"] = {
            "first_sample_wall_time_ns": video_manifest.get("first_frame_wall_time_ns"),
            "last_sample_wall_time_ns": video_manifest.get("last_frame_wall_time_ns"),
        }
    tolerance_ns = 200_000_000  # Endpoint allowance for discrete 10 Hz odometry / video frames.
    for name in ("imu", "odom") + (("robot_video",) if config.get("controller_video", {}).get("enabled") else ()):
        stream = streams.get(name) or {}
        try:
            first, last = int(stream["first_sample_wall_time_ns"]), int(stream["last_sample_wall_time_ns"])
            if not 0 < first < last:
                raise ValueError("Invalid stream interval")
            for station_id, start, stop in windows:
                if first > start + tolerance_ns or last < stop - tolerance_ns:
                    warnings.append(f"{name}: does not cover {station_id} radar window (endpoint tolerance 0.2 s).")
        except (KeyError, TypeError, ValueError, OverflowError):
            warnings.append(f"{name}: stream timing evidence is missing (optional).")
    if video_manifest.get("status") == "ok" and video_manifest.get("frames_csv") and video_manifest.get("video_file"):
        try:
            video_root = (session_dir / "controller_video").resolve()
            csv_path = (video_root / video_manifest["frames_csv"]).resolve()
            video_path = (video_root / video_manifest["video_file"]).resolve()
            csv_path.relative_to(video_root)
            video_path.relative_to(video_root)
            if not video_path.is_file() or video_path.stat().st_size == 0:
                warnings.append("Robot video file is empty or missing (optional).")
            with csv_path.open(newline="", encoding="utf-8") as handle:
                stamps = [int(row["host_monotonic_ns"]) for row in csv.DictReader(handle)]
            if len(stamps) < 2 or len(stamps) != int(video_manifest["saved_frames"]):
                warnings.append("Robot video frame timestamps/count are incomplete (optional).")
            if any(value <= 0 for value in stamps) or any(not 0 < b - a <= 500_000_000 for a, b in zip(stamps, stamps[1:])):
                warnings.append("Robot video has invalid timestamps or a frame gap exceeding 0.5 seconds (optional).")
        except (OSError, KeyError, TypeError, ValueError, OverflowError, csv.Error) as exc:
            warnings.append(f"Robot video evidence cannot be validated (optional): {exc}")
    return {"status": "failed" if errors else "ok", "errors": errors, "warnings": list(dict.fromkeys(warnings)),
            "radar_start_skew_ms": radar_start_skew_ms,
            "radar_common_overlap_seconds": common_overlap_seconds,
            "endpoint_tolerance_seconds": 0.2, "max_stream_gap_seconds": 0.5,
            "max_source_clock_delta_seconds": 1.0}


def controller_imu_sample_count(result: dict[str, Any] | None) -> int:
    if not isinstance(result, dict):
        return 0
    response = result.get("response")
    if not isinstance(response, dict):
        return 0
    manifest = response.get("manifest")
    if not isinstance(manifest, dict):
        return 0
    return int(manifest.get("sample_count") or 0)


def controller_video_requested(config: dict[str, Any], args: argparse.Namespace) -> bool:
    return bool(config.get("controller_video", {}).get("enabled", False))


def build_controller_video_command(
    controller_video: dict[str, Any],
    controller_id: str,
    subcommand: str,
    *,
    output_dir: Path | None = None,
    capture_id: str = "",
    name: str = "",
) -> list[str]:
    python_executable = shutil.which("python3") or sys.executable
    command = [
        python_executable,
        str(CONTROLLER_VIDEO_HELPER),
        subcommand,
        "--source",
        str(controller_video["source"]),
        "--device",
        str(controller_video["device"]),
        "--topic",
        str(controller_video["topic"]),
        "--field",
        str(controller_video["field"]),
        "--width",
        str(controller_video["width"]),
        "--height",
        str(controller_video["height"]),
        "--fps",
        str(controller_video["fps"]),
        "--fourcc",
        str(controller_video["fourcc"]),
        "--timeout-seconds",
        str(controller_video["timeout_seconds"]),
    ]
    if subcommand == "record":
        if output_dir is None:
            raise RuntimeError("output_dir is required for controller video recording.")
        command.extend(
            [
                "--output-dir",
                str(output_dir),
                "--controller-id",
                controller_id,
                "--capture-id",
                capture_id,
                "--name",
                name,
            ]
        )
    if str(controller_video.get("source", "device")) == "ros_topic":
        shell_steps = ["set -eo pipefail"]
        for setup_script in controller_video.get("ros_setup_scripts", []):
            shell_steps.append(f"source {shlex.quote(str(Path(setup_script).expanduser()))}")
        shell_steps.append("exec " + " ".join(shlex.quote(part) for part in command))
        return ["bash", "-lc", " && ".join(shell_steps)]
    return command


def disabled_controller_video_result(required: bool, reason: str) -> dict[str, Any]:
    return {
        "command": "controller_video",
        "enabled": False,
        "required": required,
        "status": "disabled",
        "returncode": 0,
        "response": {"status": "disabled", "message": reason},
        "stderr_tail": [],
    }


def run_controller_video_preflight(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    video = config["controller_video"]
    if not controller_video_requested(config, args):
        return disabled_controller_video_result(bool(video.get("required")), "Robot video is disabled.")
    command = build_controller_video_command(video, config["controller_id"], "preflight")
    try:
        completed = subprocess.run(command, capture_output=True, text=True, check=False,
                                   timeout=max(15.0, float(video["timeout_seconds"]) + 10.0))
        response = parse_json_output(completed.stdout) or {}
        status = str(response.get("status", "parse_failed"))
        if completed.returncode != 0 or status != "ok":
            status = "preflight_failed"
        return {"command": "controller_video_preflight", "required": bool(video.get("required")),
                "status": status, "returncode": completed.returncode, "response": response,
                "stderr_tail": completed.stderr.splitlines()[-20:]}
    except (OSError, subprocess.SubprocessError) as exc:
        return {"command": "controller_video_preflight", "required": bool(video.get("required")),
                "status": "preflight_failed", "response": {"error": str(exc)}}


def start_controller_video_capture(
    config: dict[str, Any],
    args: argparse.Namespace,
    session_dir: Path,
    reports_dir: Path,
    capture_id: str,
) -> dict[str, Any]:
    controller_video = config.get("controller_video", {})
    if not controller_video_requested(config, args):
        return disabled_controller_video_result(bool(controller_video.get("required")), "Controller video capture disabled.")
    if os.name != "posix":
        return disabled_controller_video_result(bool(controller_video.get("required")), "Controller video requires a POSIX host.")

    output_dir = session_dir / "controller_video"
    log_path = reports_dir / "controller_video_capture.log"
    command = build_controller_video_command(
        controller_video,
        config["controller_id"],
        "record",
        output_dir=output_dir,
        capture_id=capture_id,
        name=args.name,
    )
    log_handle = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(command, stdout=log_handle, stderr=subprocess.STDOUT, text=True)
    except Exception as exc:
        log_handle.close()
        return {
            "command": "controller_video_capture",
            "enabled": True,
            "required": bool(controller_video.get("required")),
            "status": "error",
            "returncode": -1,
            "response": {"status": "error", "message": str(exc)},
            "stderr_tail": [],
            "log_path": str(log_path),
        }

    return {
        "command": "controller_video_capture",
        "enabled": True,
        "required": bool(controller_video.get("required")),
        "status": "recording",
        "returncode": None,
        "response": None,
        "stderr_tail": [],
        "process": process,
        "log_handle": log_handle,
        "log_path": str(log_path),
        "manifest_path": str(output_dir / "controller_video_manifest.json"),
    }


def stop_controller_video_capture(record: dict[str, Any]) -> dict[str, Any]:
    process = record.pop("process", None)
    log_handle = record.pop("log_handle", None)
    manifest_path = Path(record["manifest_path"]) if record.get("manifest_path") else None
    log_path = Path(record["log_path"]) if record.get("log_path") else None
    try:
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        if log_handle is not None:
            log_handle.close()
    finally:
        if log_handle is not None and not log_handle.closed:
            log_handle.close()

    if process is not None:
        record["returncode"] = process.returncode
    manifest = None
    if manifest_path is not None and manifest_path.exists():
        with manifest_path.open("r", encoding="utf-8") as f:
            manifest = json.load(f)
    if manifest is not None:
        record["status"] = str(manifest.get("status", "error"))
        record["response"] = {"status": record["status"], "manifest": manifest}
    elif record.get("status") == "recording":
        record["status"] = "error"
        record["response"] = {"status": "error", "message": "Controller video manifest was not produced."}
    if log_path is not None:
        record["stderr_tail"] = read_log_tail(log_path)
    return record


def controller_video_frame_count(result: dict[str, Any] | None) -> int:
    if not isinstance(result, dict):
        return 0
    response = result.get("response")
    manifest = response.get("manifest") if isinstance(response, dict) else None
    return int(manifest.get("saved_frames") or 0) if isinstance(manifest, dict) else 0


def build_summary(
    station_records: list[dict[str, Any]],
    controller_imu_capture: dict[str, Any] | None = None,
    controller_video_capture: dict[str, Any] | None = None,
) -> dict[str, Any]:
    ok_stations = 0
    failed_stations = 0
    total_radar_raw_bytes = 0
    total_imu_samples = 0
    total_camera_saved_frames = 0

    for station in station_records:
        capture = station.get("capture") or {}
        if capture.get("remote_status") == "ok":
            ok_stations += 1
        elif capture:
            failed_stations += 1

        capture_response = capture.get("response") or {}
        summary = capture_response.get("summary") if isinstance(capture_response, dict) else None
        if isinstance(summary, dict):
            total_radar_raw_bytes += int(summary.get("radar_raw_bytes") or 0)
            total_imu_samples += int(summary.get("imu_samples") or 0)
            total_camera_saved_frames += int(summary.get("camera_saved_frames") or 0)

    return {
        "station_count": len(station_records),
        "ok_stations": ok_stations,
        "failed_stations": failed_stations,
        "total_radar_raw_bytes": total_radar_raw_bytes,
        "total_imu_samples": total_imu_samples,
        "total_camera_saved_frames": total_camera_saved_frames,
        "controller_imu_status": (
            str(controller_imu_capture.get("status", "disabled"))
            if isinstance(controller_imu_capture, dict)
            else "disabled"
        ),
        "controller_imu_samples": controller_imu_sample_count(controller_imu_capture),
        "controller_telemetry_streams": extract_controller_imu_manifest(controller_imu_capture).get("streams", {}),
        "controller_telemetry_warnings": extract_controller_imu_manifest(controller_imu_capture).get("warnings", []),
        "controller_video_status": (
            str(controller_video_capture.get("status", "disabled"))
            if isinstance(controller_video_capture, dict)
            else "disabled"
        ),
        "controller_video_frames": controller_video_frame_count(controller_video_capture),
    }


def build_operator_request_record(
    args: argparse.Namespace,
    result_mode: str,
    stations: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "entrypoint": getattr(args, "operator_entrypoint", args.command),
        "command": args.command,
        "requested_name": getattr(args, "name", ""),
        "capture_id": getattr(args, "capture_id", ""),
        "mode_profile": getattr(args, "mode_profile", "manual"),
        "mode_profile_label": getattr(args, "mode_profile_label", ""),
        "mode_profile_description": getattr(args, "mode_profile_description", ""),
        "mode_profile_notes": list(getattr(args, "mode_profile_notes", []) or []),
        "station_overrides": dict(getattr(args, "station_overrides", {}) or {}),
        "duration_seconds": getattr(args, "duration_seconds", None),
        "countdown_seconds": getattr(args, "countdown_seconds", None),
        "result_mode": result_mode,
        "pull_results": result_mode == "pull-full",
        "allow_no_camera": bool(getattr(args, "allow_no_camera", False)),
        "allow_no_imu": bool(getattr(args, "allow_no_imu", False)),
        "skip_preflight": bool(getattr(args, "skip_preflight", False)),
        "force_capture": bool(getattr(args, "force_capture", False)),
        "controller_imu_enabled": not bool(getattr(args, "no_controller_imu", False)),
        "clock_snapshots_enabled": not bool(getattr(args, "no_clock_snapshots", False)),
        "clock_snapshot_samples": getattr(args, "clock_snapshot_samples", None),
        "selected_stations": [station["station_id"] for station in stations],
        "capture_plan": build_capture_plan(args, stations),
    }


def build_capture_overview_markdown(payload: dict[str, Any]) -> str:
    summary = payload.get("summary") if isinstance(payload.get("summary"), dict) else {}
    operator_request = payload.get("operator_request") if isinstance(payload.get("operator_request"), dict) else {}
    stations = payload.get("stations") if isinstance(payload.get("stations"), list) else []
    lines = [
        "# 采集概览",
        "",
        f"- 状态: `{payload.get('status', '')}`",
        f"- 场景名: `{payload.get('requested_name', '')}`",
        f"- capture_id: `{payload.get('capture_id', '')}`",
        f"- 模式: `{operator_request.get('mode_profile', 'manual')}`",
        f"- 结果模式: `{payload.get('result_mode', '')}`",
        f"- session 目录: `{payload.get('session_dir', '')}`",
        f"- 成功站点: `{summary.get('ok_stations', 0)}/{summary.get('station_count', 0)}`",
        f"- 总雷达字节数: `{summary.get('total_radar_raw_bytes', 0)}`",
        f"- 总站点 IMU 样本: `{summary.get('total_imu_samples', 0)}`",
        f"- 总相机帧数: `{summary.get('total_camera_saved_frames', 0)}`",
        f"- 机器狗 IMU 样本: `{summary.get('controller_imu_samples', 0)}`",
        "",
        "## 站点",
        "",
    ]
    for station in stations:
        capture = station.get("capture") if isinstance(station.get("capture"), dict) else {}
        response = capture.get("response") if isinstance(capture.get("response"), dict) else {}
        station_summary = response.get("summary") if isinstance(response.get("summary"), dict) else {}
        lines.extend(
            [
                f"- `{station.get('station_id', '')}`",
                f"  - 状态: `{capture.get('remote_status', '')}`",
                f"  - 雷达字节数: `{station_summary.get('radar_raw_bytes', 0)}`",
                f"  - IMU 样本: `{station_summary.get('imu_samples', 0)}`",
                f"  - 相机帧数: `{station_summary.get('camera_saved_frames', 0)}`",
                f"  - 起采准备耗时: `{station_summary.get('requested_start_to_sensor_start_seconds', '')}`",
                f"  - 写盘收尾耗时: `{station_summary.get('sensor_stop_to_raw_written_seconds', '')}`",
            ]
        )
    lines.append("")
    return "\n".join(lines)


def build_transfer_summary(
    requested_mode: str,
    requested_station_count: int,
    pull_results: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    pull_results = pull_results or []
    ok_pulls = sum(1 for item in pull_results if item.get("remote_status") == "ok")
    failed_pulls = sum(1 for item in pull_results if item.get("remote_status") != "ok")
    attempted = len(pull_results)
    skipped = max(0, requested_station_count - attempted)

    if requested_mode != "pull-full":
        status = "not_requested"
    elif attempted == 0 and requested_station_count > 0:
        status = "not_attempted"
    elif failed_pulls == 0:
        status = "ok"
    elif ok_pulls == 0:
        status = "pull_failed"
    else:
        status = "partial_failure"

    return {
        "requested_mode": requested_mode,
        "attempted_stations": attempted,
        "ok_stations": ok_pulls,
        "failed_stations": failed_pulls,
        "skipped_stations": skipped,
        "status": status,
    }


def build_station_records(
    stations: list[dict[str, Any]],
    preflight_results: list[dict[str, Any]] | None,
    capture_results: list[dict[str, Any]] | None,
    pull_results: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    record_map: dict[str, dict[str, Any]] = {
        station["station_id"]: {
            "station_id": station["station_id"],
            "host": station["host"],
            "configured_host": station.get("configured_host", station["host"]),
            "host_resolution": station.get("host_resolution", {}),
            "user": station["user"],
            "remote_root": station["remote_root"],
            "camera_required": station.get("camera_required", True),
            "notes": station["notes"],
        }
        for station in stations
    }

    if preflight_results is not None:
        for result in preflight_results:
            record_map[result["station_id"]]["preflight"] = result

    if capture_results is not None:
        for result in capture_results:
            record_map[result["station_id"]]["capture"] = result

    if pull_results is not None:
        for result in pull_results:
            record_map[result["station_id"]]["pull"] = result

    records = list(record_map.values())
    records.sort(key=lambda item: item["station_id"])
    return records


def build_station_storage_summary(station_records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for station in station_records:
        capture = station.get("capture") if isinstance(station.get("capture"), dict) else {}
        response = capture.get("response") if isinstance(capture.get("response"), dict) else {}
        report = response.get("report") if isinstance(response.get("report"), dict) else {}
        summary = response.get("summary") if isinstance(response.get("summary"), dict) else {}
        manifest = report.get("manifest") if isinstance(report.get("manifest"), dict) else {}
        radar = manifest.get("radar") if isinstance(manifest.get("radar"), dict) else {}
        rows.append(
            {
                "station_id": station.get("station_id"),
                "host": station.get("host"),
                "capture_status": capture.get("remote_status"),
                "remote_session_dir": summary.get("session_dir") or report.get("session_dir"),
                "remote_manifest_path": summary.get("manifest_path") or report.get("manifest_path"),
                "radar_config": summary.get("radar_config") or radar.get("radar_config"),
                "radar_raw_bytes": summary.get("radar_raw_bytes"),
                "imu_samples": summary.get("imu_samples"),
                "camera_saved_frames": summary.get("camera_saved_frames"),
                "pulled_to_dog": bool(station.get("pull")),
            }
        )
    return rows


def coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def seconds_between(start_ns: Any, stop_ns: Any) -> float | None:
    start = coerce_int(start_ns)
    stop = coerce_int(stop_ns)
    if start is None or stop is None:
        return None
    return (stop - start) / 1_000_000_000


def extract_response_dict(result: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(result, dict):
        return {}
    response = result.get("response")
    return response if isinstance(response, dict) else {}


def extract_remote_report_from_result(result: dict[str, Any] | None) -> dict[str, Any]:
    response = extract_response_dict(result)
    report = response.get("report")
    return report if isinstance(report, dict) else {}


def extract_station_capture_manifest(result: dict[str, Any] | None) -> dict[str, Any]:
    report = extract_remote_report_from_result(result)
    manifest = report.get("manifest")
    return manifest if isinstance(manifest, dict) else {}


def extract_controller_imu_manifest(result: dict[str, Any] | None) -> dict[str, Any]:
    response = extract_response_dict(result)
    manifest = response.get("manifest")
    return manifest if isinstance(manifest, dict) else {}


def build_preflight_health_matrix(
    controller_imu_result: dict[str, Any],
    station_records: list[dict[str, Any]],
) -> dict[str, Any]:
    controller_response = extract_response_dict(controller_imu_result)
    controller_sample = controller_response.get("sample")
    controller_health = {
        "status": controller_imu_result.get("status"),
        "required": bool(controller_imu_result.get("required")),
        "topic": controller_imu_result.get("topic"),
        "sample_frame_id": (
            controller_sample.get("frame_id")
            if isinstance(controller_sample, dict)
            else ""
        ),
        "message": (
            controller_response.get("error")
            or controller_response.get("message")
            or ""
        ),
    }

    stations_health: list[dict[str, Any]] = []
    for station in station_records:
        preflight = station.get("preflight")
        report = extract_remote_report_from_result(preflight)
        network = report.get("network") if isinstance(report.get("network"), dict) else {}
        imu = report.get("imu") if isinstance(report.get("imu"), dict) else {}
        camera = report.get("camera") if isinstance(report.get("camera"), dict) else {}
        errors = report.get("errors") if isinstance(report.get("errors"), list) else []
        addr_state = str(network.get("addr_state", "")).strip()
        stations_health.append(
            {
                "station_id": station["station_id"],
                "status": report.get("status") or preflight.get("remote_status"),
                "radar_usb_present": bool(report.get("radar_usb_present")),
                "network_up": (" UP " in f" {addr_state} ") if addr_state else False,
                "network_addr_state": addr_state,
                "imu_port": str(imu.get("port", "")),
                "imu_probe_hits": int(imu.get("probe_hits") or 0),
                "camera_ok": bool(camera.get("ok")) if camera else False,
                "camera_device": str(camera.get("device", "")),
                "errors": errors,
            }
        )

    return {
        "controller_imu": controller_health,
        "stations": stations_health,
    }


def build_station_sync_assessment(
    station_id: str,
    manifest: dict[str, Any],
) -> dict[str, Any]:
    alignment_tolerance_seconds = 0.02
    timing = manifest.get("timing") if isinstance(manifest.get("timing"), dict) else {}
    imu = manifest.get("imu") if isinstance(manifest.get("imu"), dict) else {}
    camera = manifest.get("camera") if isinstance(manifest.get("camera"), dict) else {}

    radar_start_ns = coerce_int(timing.get("radar_sensor_start_monotonic_ns"))
    radar_stop_ns = coerce_int(timing.get("radar_sensor_stop_monotonic_ns"))
    imu_first_ns = coerce_int(imu.get("first_sample_monotonic_ns"))
    imu_last_ns = coerce_int(imu.get("last_sample_monotonic_ns"))
    camera_first_ns = coerce_int(camera.get("first_saved_monotonic_ns"))
    camera_last_ns = coerce_int(camera.get("last_saved_monotonic_ns"))

    imu_covers_radar_window = None
    if None not in (imu_first_ns, imu_last_ns, radar_start_ns, radar_stop_ns):
        imu_covers_radar_window = bool(
            imu_first_ns <= radar_start_ns <= imu_last_ns and imu_first_ns <= radar_stop_ns <= imu_last_ns
        )
    imu_overlaps_radar_window = None
    if None not in (imu_first_ns, imu_last_ns, radar_start_ns, radar_stop_ns):
        imu_overlaps_radar_window = bool(imu_first_ns <= radar_stop_ns and imu_last_ns >= radar_start_ns)

    imu_first_minus_radar_start_seconds = seconds_between(radar_start_ns, imu_first_ns)
    imu_last_minus_radar_stop_seconds = seconds_between(radar_stop_ns, imu_last_ns)
    imu_aligned_with_radar_window = None
    if imu_first_minus_radar_start_seconds is not None and imu_last_minus_radar_stop_seconds is not None:
        imu_aligned_with_radar_window = bool(
            abs(imu_first_minus_radar_start_seconds) <= alignment_tolerance_seconds
            and abs(imu_last_minus_radar_stop_seconds) <= alignment_tolerance_seconds
        )

    camera_overlaps_radar_window = None
    if None not in (camera_first_ns, camera_last_ns, radar_start_ns, radar_stop_ns):
        camera_overlaps_radar_window = bool(camera_first_ns <= radar_stop_ns and camera_last_ns >= radar_start_ns)

    return {
        "station_id": station_id,
        "status": manifest.get("status"),
        "clock_domain": station_id,
        "cross_host_alignment": "not_comparable_without_clock_sync",
        "radar_active_duration_seconds": timing.get("radar_active_duration_seconds"),
        "imu_enabled": bool(imu.get("enabled")),
        "imu_samples": int(imu.get("sample_count") or 0),
        "imu_first_minus_radar_start_seconds": imu_first_minus_radar_start_seconds,
        "imu_last_minus_radar_stop_seconds": imu_last_minus_radar_stop_seconds,
        "imu_covers_radar_window": imu_covers_radar_window,
        "imu_overlaps_radar_window": imu_overlaps_radar_window,
        "imu_aligned_with_radar_window": imu_aligned_with_radar_window,
        "imu_alignment_tolerance_seconds": alignment_tolerance_seconds,
        "camera_enabled": bool(camera.get("enabled")),
        "camera_saved_frames": int(camera.get("saved_frames") or 0),
        "camera_first_minus_radar_start_seconds": seconds_between(radar_start_ns, camera_first_ns),
        "camera_last_minus_radar_stop_seconds": seconds_between(radar_stop_ns, camera_last_ns),
        "camera_overlaps_radar_window": camera_overlaps_radar_window,
    }


def summarize_clock_snapshot_phase(clock_snapshot: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(clock_snapshot, dict):
        return {
            "phase": "",
            "status": "missing",
            "station_count": 0,
            "stations": [],
        }

    response = clock_snapshot.get("response")
    response_dict = response if isinstance(response, dict) else {}
    stations_raw = response_dict.get("stations")
    stations = [item for item in stations_raw if isinstance(item, dict)] if isinstance(stations_raw, list) else []
    summarized_stations: list[dict[str, Any]] = []
    for station in stations:
        summarized_stations.append(
            {
                "station_id": station.get("station_id"),
                "status": station.get("status"),
                "sample_count": station.get("sample_count"),
                "median_rtt_ms": station.get("median_rtt_ms"),
                "median_offset_ms": station.get("median_offset_ms"),
                "best_effort_offset_ms": station.get("best_effort_offset_ms"),
                "offset_stddev_ms": station.get("offset_stddev_ms"),
                "max_abs_offset_ms": station.get("max_abs_offset_ms"),
            }
        )

    return {
        "phase": clock_snapshot.get("phase", ""),
        "status": clock_snapshot.get("status"),
        "station_count": len(summarized_stations),
        "stations": summarized_stations,
        "checked_at_local": response_dict.get("checked_at_local"),
        "max_offset_ms": response_dict.get("max_offset_ms"),
    }


def build_clock_snapshot_drift(clock_snapshots: dict[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(clock_snapshots, dict):
        return []
    before = summarize_clock_snapshot_phase(clock_snapshots.get("before_capture"))
    after = summarize_clock_snapshot_phase(clock_snapshots.get("after_capture"))
    before_map = {
        str(item.get("station_id")): item
        for item in before.get("stations", [])
        if isinstance(item, dict) and item.get("station_id")
    }
    after_map = {
        str(item.get("station_id")): item
        for item in after.get("stations", [])
        if isinstance(item, dict) and item.get("station_id")
    }

    drift_rows: list[dict[str, Any]] = []
    for station_id in sorted(set(before_map) | set(after_map)):
        before_item = before_map.get(station_id, {})
        after_item = after_map.get(station_id, {})
        before_offset = before_item.get("best_effort_offset_ms")
        if before_offset is None:
            before_offset = before_item.get("median_offset_ms")
        after_offset = after_item.get("best_effort_offset_ms")
        if after_offset is None:
            after_offset = after_item.get("median_offset_ms")

        drift_ms = None
        if before_offset is not None and after_offset is not None:
            drift_ms = float(after_offset) - float(before_offset)

        drift_rows.append(
            {
                "station_id": station_id,
                "before_offset_ms": before_offset,
                "after_offset_ms": after_offset,
                "drift_ms": drift_ms,
            }
        )
    return drift_rows


def build_sync_assessment(
    station_records: list[dict[str, Any]],
    controller_imu_capture: dict[str, Any] | None = None,
    clock_snapshots: dict[str, Any] | None = None,
) -> dict[str, Any]:
    controller_manifest = extract_controller_imu_manifest(controller_imu_capture)
    station_assessments: list[dict[str, Any]] = []
    for station in station_records:
        station_manifest = extract_station_capture_manifest(station.get("capture"))
        if station_manifest:
            station_assessments.append(build_station_sync_assessment(station["station_id"], station_manifest))
        else:
            station_assessments.append(
                {
                    "station_id": station["station_id"],
                    "status": station.get("capture", {}).get("remote_status") if isinstance(station.get("capture"), dict) else "",
                    "clock_domain": station["station_id"],
                    "cross_host_alignment": "not_comparable_without_clock_sync",
                    "error": "Station capture manifest unavailable.",
                }
            )

    return {
        "mode": "session_level_orchestration",
        "controller_clock_domain": "controller_nx",
        "cross_host_clock_sync_required": True,
        "recommended_clock_sync": ["chrony", "PTP", "hardware_trigger_or_pps"],
        "controller_imu": {
            "status": (
                str(controller_imu_capture.get("status", "disabled"))
                if isinstance(controller_imu_capture, dict)
                else "disabled"
            ),
            "topic": controller_manifest.get("topic") or "",
            "sample_count": int(controller_manifest.get("sample_count") or 0),
            "received_duration_seconds": controller_manifest.get("received_duration_seconds"),
            "first_sample_monotonic_ns": controller_manifest.get("first_sample_monotonic_ns"),
            "last_sample_monotonic_ns": controller_manifest.get("last_sample_monotonic_ns"),
        },
        "clock_snapshots": {
            "before_capture": summarize_clock_snapshot_phase(
                clock_snapshots.get("before_capture") if isinstance(clock_snapshots, dict) else None
            ),
            "after_capture": summarize_clock_snapshot_phase(
                clock_snapshots.get("after_capture") if isinstance(clock_snapshots, dict) else None
            ),
            "drift": build_clock_snapshot_drift(clock_snapshots),
        },
        "stations": station_assessments,
        "notes": [
            "Controller IMU and station-local monotonic clocks are valid only inside each host.",
            "Cross-host offsets become meaningful only after LAN clock sync such as chrony/PTP, or with hardware trigger/PPS.",
        ],
    }


def resolve_ingest_session_dirs(root: Path, explicit_session_dirs: list[str]) -> list[Path]:
    if explicit_session_dirs:
        session_dirs = [Path(item).expanduser() for item in explicit_session_dirs if str(item).strip()]
    else:
        if not root.exists():
            raise RuntimeError(f"Fleet session root does not exist: {root}")
        session_dirs = [
            candidate
            for candidate in root.iterdir()
            if candidate.is_dir() and (candidate / "fleet_manifest.json").exists()
        ]
    session_dirs.sort(key=lambda path: path.name, reverse=True)
    return session_dirs


def clone_args(args: argparse.Namespace, **updates: Any) -> argparse.Namespace:
    payload = vars(args).copy()
    payload.update(updates)
    return argparse.Namespace(**payload)


def load_fleet_manifest(session_dir: Path) -> dict[str, Any]:
    manifest_path = session_dir / "fleet_manifest.json"
    if not manifest_path.exists():
        raise RuntimeError(f"fleet_manifest.json does not exist: {manifest_path}")
    with manifest_path.open("r", encoding="utf-8") as manifest_file:
        payload = json.load(manifest_file)
    if not isinstance(payload, dict):
        raise RuntimeError(f"fleet_manifest.json is not a JSON object: {manifest_path}")
    return payload


def find_station_item(items: Any, station_id: str) -> dict[str, Any]:
    if not isinstance(items, list):
        return {}
    for item in items:
        if not isinstance(item, dict):
            continue
        if str(item.get("station_id", "")) == station_id:
            return item
    return {}


def float_or_none(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def int_or_none(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def mean_or_none(values: list[float | None]) -> float | None:
    cleaned = [float(value) for value in values if value is not None]
    if not cleaned:
        return None
    return statistics.mean(cleaned)


def std_or_none(values: list[float | None]) -> float | None:
    cleaned = [float(value) for value in values if value is not None]
    if not cleaned:
        return None
    if len(cleaned) == 1:
        return 0.0
    return statistics.pstdev(cleaned)


def max_abs_or_none(values: list[float | None]) -> float | None:
    cleaned = [abs(float(value)) for value in values if value is not None]
    if not cleaned:
        return None
    return max(cleaned)


def build_station_catalog_entry(station_record: dict[str, Any]) -> dict[str, Any]:
    capture = station_record.get("capture")
    pull = station_record.get("pull")
    response = extract_response_dict(capture)
    report = extract_remote_report_from_result(capture)
    summary = response.get("summary") if isinstance(response.get("summary"), dict) else {}
    manifest = report.get("manifest") if isinstance(report.get("manifest"), dict) else {}
    pull_response = extract_response_dict(pull)

    return {
        "station_id": station_record.get("station_id"),
        "host": station_record.get("host"),
        "status": summary.get("status") or report.get("status") or (capture.get("remote_status") if isinstance(capture, dict) else ""),
        "remote_session_dir": report.get("session_dir") or "",
        "pull_status": pull.get("remote_status") if isinstance(pull, dict) else "",
        "pulled_to": response.get("pulled_to") or pull_response.get("local_session_dir") or "",
        "radar_raw_bytes": summary.get("radar_raw_bytes"),
        "radar_active_duration_seconds": summary.get("radar_active_duration_seconds"),
        "imu_samples": summary.get("imu_samples"),
        "camera_saved_frames": summary.get("camera_saved_frames"),
        "camera_actual_saved_fps": summary.get("camera_actual_saved_fps"),
        "requested_elapsed_seconds": summary.get("requested_elapsed_seconds"),
        "camera_launch_to_ready_seconds": summary.get("camera_launch_to_ready_seconds"),
        "requested_start_to_sensor_start_seconds": summary.get("requested_start_to_sensor_start_seconds"),
        "sensor_stop_to_raw_written_seconds": summary.get("sensor_stop_to_raw_written_seconds"),
        "imu_first_minus_radar_start_seconds": summary.get("imu_first_minus_radar_start_seconds"),
        "imu_last_minus_radar_stop_seconds": summary.get("imu_last_minus_radar_stop_seconds"),
        "camera_first_saved_minus_radar_start_seconds": summary.get("camera_first_saved_minus_radar_start_seconds"),
        "camera_last_saved_minus_radar_stop_seconds": summary.get("camera_last_saved_minus_radar_stop_seconds"),
        "manifest_status": manifest.get("status") if manifest else "",
        "error": report.get("error") if isinstance(report, dict) else None,
    }


def build_catalog_entry(session_dir: Path, fleet_manifest: dict[str, Any]) -> dict[str, Any]:
    controller_imu = fleet_manifest.get("controller_imu") if isinstance(fleet_manifest.get("controller_imu"), dict) else {}
    controller_capture = controller_imu.get("capture") if isinstance(controller_imu.get("capture"), dict) else {}
    controller_manifest = extract_controller_imu_manifest(controller_capture)
    station_records_raw = fleet_manifest.get("stations")
    station_records = [item for item in station_records_raw if isinstance(item, dict)] if isinstance(station_records_raw, list) else []

    return {
        "session_name": session_dir.name,
        "session_dir": str(session_dir),
        "status": fleet_manifest.get("status"),
        "controller_id": fleet_manifest.get("controller_id"),
        "capture_id": fleet_manifest.get("capture_id"),
        "requested_name": fleet_manifest.get("requested_name"),
        "created_at_local": fleet_manifest.get("created_at_local"),
        "result_mode": fleet_manifest.get("result_mode"),
        "controller_imu_status": controller_capture.get("status") if controller_capture else "",
        "controller_imu_samples": int(controller_manifest.get("sample_count") or 0),
        "summary": fleet_manifest.get("summary"),
        "stations": [build_station_catalog_entry(item) for item in station_records],
        "sync_assessment": build_sync_assessment(
            station_records,
            controller_capture,
            fleet_manifest.get("clock_snapshots") if isinstance(fleet_manifest.get("clock_snapshots"), dict) else None,
        ),
        "clock_snapshots": fleet_manifest.get("clock_snapshots"),
        "artifacts": fleet_manifest.get("artifacts"),
    }


def run_ingest(args: argparse.Namespace, config: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    root = (args.root or Path(config["output_root"])).expanduser()
    session_dirs = resolve_ingest_session_dirs(root, args.session_dir)
    catalog_path = (args.catalog_path or (root / "fleet_session_index.json")).expanduser()

    sessions: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    for session_dir in session_dirs:
        manifest_path = session_dir / "fleet_manifest.json"
        if not manifest_path.exists():
            skipped.append({"session_dir": str(session_dir), "reason": "fleet_manifest.json is missing"})
            continue
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            fleet_manifest = json.load(manifest_file)
        if not isinstance(fleet_manifest, dict):
            skipped.append({"session_dir": str(session_dir), "reason": "fleet_manifest.json is not a JSON object"})
            continue
        sessions.append(build_catalog_entry(session_dir, fleet_manifest))

    payload = {
        "status": "ok",
        "generated_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "root_dir": str(root),
        "catalog_path": str(catalog_path),
        "session_count": len(sessions),
        "skipped_count": len(skipped),
        "sessions": sessions,
        "skipped": skipped,
    }
    write_json(catalog_path, payload)
    return 0, payload


def build_sync_report_session_overview(session_dir: Path, fleet_manifest: dict[str, Any]) -> dict[str, Any]:
    controller_imu = fleet_manifest.get("controller_imu") if isinstance(fleet_manifest.get("controller_imu"), dict) else {}
    controller_capture = controller_imu.get("capture") if isinstance(controller_imu.get("capture"), dict) else {}
    controller_manifest = extract_controller_imu_manifest(controller_capture)
    summary = fleet_manifest.get("summary") if isinstance(fleet_manifest.get("summary"), dict) else {}
    transfer_summary = fleet_manifest.get("transfer_summary") if isinstance(fleet_manifest.get("transfer_summary"), dict) else {}
    return {
        "session_name": session_dir.name,
        "session_dir": str(session_dir),
        "status": fleet_manifest.get("status"),
        "requested_name": fleet_manifest.get("requested_name"),
        "capture_id": fleet_manifest.get("capture_id"),
        "created_at_local": fleet_manifest.get("created_at_local"),
        "result_mode": fleet_manifest.get("result_mode"),
        "controller_imu_status": controller_capture.get("status") if controller_capture else "",
        "controller_imu_samples": int(controller_manifest.get("sample_count") or 0),
        "ok_stations": int(summary.get("ok_stations") or 0),
        "failed_stations": int(summary.get("failed_stations") or 0),
        "total_radar_raw_bytes": int(summary.get("total_radar_raw_bytes") or 0),
        "total_imu_samples": int(summary.get("total_imu_samples") or 0),
        "total_camera_saved_frames": int(summary.get("total_camera_saved_frames") or 0),
        "transfer_status": transfer_summary.get("status") or "",
    }


def build_sync_report_station_row(session_dir: Path, fleet_manifest: dict[str, Any], station_record: dict[str, Any]) -> dict[str, Any]:
    station_id = str(station_record.get("station_id", ""))
    capture = station_record.get("capture")
    response = extract_response_dict(capture)
    summary = response.get("summary") if isinstance(response.get("summary"), dict) else {}
    report = extract_remote_report_from_result(capture)
    radar_packet_stats = summary.get("radar_packet_stats") if isinstance(summary.get("radar_packet_stats"), dict) else {}
    sync_assessment = fleet_manifest.get("sync_assessment") if isinstance(fleet_manifest.get("sync_assessment"), dict) else {}
    station_sync = find_station_item(sync_assessment.get("stations"), station_id)
    clock_sync = sync_assessment.get("clock_snapshots") if isinstance(sync_assessment.get("clock_snapshots"), dict) else {}
    before_entry = find_station_item(
        (clock_sync.get("before_capture") or {}).get("stations") if isinstance(clock_sync.get("before_capture"), dict) else [],
        station_id,
    )
    after_entry = find_station_item(
        (clock_sync.get("after_capture") or {}).get("stations") if isinstance(clock_sync.get("after_capture"), dict) else [],
        station_id,
    )
    drift_entry = find_station_item(clock_sync.get("drift"), station_id)

    return {
        "session_name": session_dir.name,
        "requested_name": fleet_manifest.get("requested_name"),
        "capture_id": fleet_manifest.get("capture_id"),
        "station_id": station_id,
        "host": station_record.get("host"),
        "capture_status": summary.get("status") or report.get("status") or "",
        "remote_session_dir": report.get("session_dir") or "",
        "radar_raw_bytes": int_or_none(summary.get("radar_raw_bytes")),
        "radar_active_duration_seconds": float_or_none(summary.get("radar_active_duration_seconds")),
        "packet_expected": int_or_none(radar_packet_stats.get("expected_packet_num")),
        "packet_received": int_or_none(radar_packet_stats.get("received_packet_num")),
        "packet_loss": (
            int_or_none(radar_packet_stats.get("expected_packet_num")) - int_or_none(radar_packet_stats.get("received_packet_num"))
            if int_or_none(radar_packet_stats.get("expected_packet_num")) is not None
            and int_or_none(radar_packet_stats.get("received_packet_num")) is not None
            else None
        ),
        "imu_samples": int_or_none(summary.get("imu_samples")),
        "imu_aligned_with_radar_window": station_sync.get("imu_aligned_with_radar_window"),
        "imu_covers_radar_window": station_sync.get("imu_covers_radar_window"),
        "imu_first_minus_radar_start_seconds": float_or_none(summary.get("imu_first_minus_radar_start_seconds")),
        "imu_last_minus_radar_stop_seconds": float_or_none(summary.get("imu_last_minus_radar_stop_seconds")),
        "camera_saved_frames": int_or_none(summary.get("camera_saved_frames")),
        "camera_actual_saved_fps": float_or_none(summary.get("camera_actual_saved_fps")),
        "camera_first_saved_minus_radar_start_seconds": float_or_none(summary.get("camera_first_saved_minus_radar_start_seconds")),
        "camera_last_saved_minus_radar_stop_seconds": float_or_none(summary.get("camera_last_saved_minus_radar_stop_seconds")),
        "requested_start_to_sensor_start_seconds": float_or_none(summary.get("requested_start_to_sensor_start_seconds")),
        "sensor_stop_to_raw_written_seconds": float_or_none(summary.get("sensor_stop_to_raw_written_seconds")),
        "before_best_effort_offset_ms": float_or_none(before_entry.get("best_effort_offset_ms")),
        "before_median_offset_ms": float_or_none(before_entry.get("median_offset_ms")),
        "before_offset_stddev_ms": float_or_none(before_entry.get("offset_stddev_ms")),
        "after_best_effort_offset_ms": float_or_none(after_entry.get("best_effort_offset_ms")),
        "after_median_offset_ms": float_or_none(after_entry.get("median_offset_ms")),
        "after_offset_stddev_ms": float_or_none(after_entry.get("offset_stddev_ms")),
        "drift_ms": float_or_none(drift_entry.get("drift_ms")),
    }


def build_sync_report_station_summary(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    station_ids = sorted({str(row.get("station_id", "")) for row in rows if row.get("station_id")})
    summaries: list[dict[str, Any]] = []
    for station_id in station_ids:
        station_rows = [row for row in rows if row.get("station_id") == station_id]
        ok_capture_count = sum(1 for row in station_rows if row.get("capture_status") == "ok")
        imu_aligned_count = sum(1 for row in station_rows if row.get("imu_aligned_with_radar_window") is True)
        camera_nonzero_count = sum(1 for row in station_rows if (row.get("camera_saved_frames") or 0) > 0)
        summaries.append(
            {
                "station_id": station_id,
                "session_count": len(station_rows),
                "ok_capture_count": ok_capture_count,
                "capture_success_rate": (ok_capture_count / len(station_rows)) if station_rows else None,
                "imu_aligned_count": imu_aligned_count,
                "imu_alignment_rate": (imu_aligned_count / len(station_rows)) if station_rows else None,
                "camera_nonzero_count": camera_nonzero_count,
                "camera_nonzero_rate": (camera_nonzero_count / len(station_rows)) if station_rows else None,
                "radar_raw_bytes_mean": mean_or_none([float_or_none(row.get("radar_raw_bytes")) for row in station_rows]),
                "packet_loss_mean": mean_or_none([float_or_none(row.get("packet_loss")) for row in station_rows]),
                "packet_loss_max": max(
                    [int(row["packet_loss"]) for row in station_rows if row.get("packet_loss") is not None],
                    default=None,
                ),
                "imu_samples_mean": mean_or_none([float_or_none(row.get("imu_samples")) for row in station_rows]),
                "camera_saved_frames_mean": mean_or_none([float_or_none(row.get("camera_saved_frames")) for row in station_rows]),
                "before_best_effort_offset_ms_mean": mean_or_none(
                    [float_or_none(row.get("before_best_effort_offset_ms")) for row in station_rows]
                ),
                "before_best_effort_offset_ms_std": std_or_none(
                    [float_or_none(row.get("before_best_effort_offset_ms")) for row in station_rows]
                ),
                "after_best_effort_offset_ms_mean": mean_or_none(
                    [float_or_none(row.get("after_best_effort_offset_ms")) for row in station_rows]
                ),
                "after_best_effort_offset_ms_std": std_or_none(
                    [float_or_none(row.get("after_best_effort_offset_ms")) for row in station_rows]
                ),
                "offset_stddev_ms_mean": mean_or_none(
                    [float_or_none(row.get("before_offset_stddev_ms")) for row in station_rows]
                ),
                "drift_ms_mean": mean_or_none([float_or_none(row.get("drift_ms")) for row in station_rows]),
                "drift_ms_max_abs": max_abs_or_none([float_or_none(row.get("drift_ms")) for row in station_rows]),
            }
        )
    return summaries


def run_sync_report(args: argparse.Namespace, config: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    root = (args.root or Path(config["output_root"])).expanduser()
    session_dirs = resolve_ingest_session_dirs(root, args.session_dir)
    output_path = (args.output_path or (root / "fleet_sync_report.json")).expanduser()

    session_overview: list[dict[str, Any]] = []
    station_rows: list[dict[str, Any]] = []
    skipped: list[dict[str, Any]] = []
    for session_dir in session_dirs:
        try:
            fleet_manifest = load_fleet_manifest(session_dir)
        except Exception as exc:
            skipped.append({"session_dir": str(session_dir), "reason": str(exc)})
            continue
        session_overview.append(build_sync_report_session_overview(session_dir, fleet_manifest))
        station_records_raw = fleet_manifest.get("stations")
        station_records = [item for item in station_records_raw if isinstance(item, dict)] if isinstance(station_records_raw, list) else []
        for station_record in station_records:
            station_rows.append(build_sync_report_station_row(session_dir, fleet_manifest, station_record))

    payload = {
        "status": "ok",
        "generated_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "root_dir": str(root),
        "output_path": str(output_path),
        "session_count": len(session_overview),
        "station_run_count": len(station_rows),
        "sessions": session_overview,
        "station_runs": station_rows,
        "station_summary": build_sync_report_station_summary(station_rows),
        "skipped": skipped,
        "notes": [
            "Offsets and drift come from SSH midpoint clock snapshots and are best-effort field indicators.",
            "For strict cross-host time alignment, deploy chrony/PTP/PPS before drawing hard fusion conclusions.",
        ],
    }
    write_json(output_path, payload)
    return 0, payload


def collect_joint_preflight(config, args, stations):
    """Both menu 3 and capture admission use this exact set of checks."""
    controller_imu_result = run_controller_imu_preflight(config, args)
    controller_video_result = run_controller_video_preflight(config, args)
    results = run_parallel(
        stations,
        lambda station: run_remote_helper(
            station,
            apply_station_args(args, station, apply_countdown_offset=False),
            "preflight",
        ),
    )
    code_versions = check_station_code_versions(stations, args)
    clock = {"phase": "preflight", "status": "not_run", "response": {}}
    clock_errors = []
    if config.get("require_complete_joint_capture") and (
        all(result["remote_status"] == "ok" for result in results)
        and not controller_imu_blocks_result(controller_imu_result)
        and not controller_video_blocks_result(controller_video_result)
    ):
        clock = run_clock_snapshot(config, args, stations, "preflight")
        clock_errors = joint_clock_issues(clock, [s["station_id"] for s in stations],
                                          float(config["clock_snapshots"]["max_offset_ms"]))
    return controller_imu_result, controller_video_result, results, clock, clock_errors, code_versions


def run_preflight(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]]) -> tuple[int, dict[str, Any]]:
    if config.get("require_complete_joint_capture"):
        args = clone_args(args, allow_no_camera=True, no_clock_snapshots=False)
    controller_time_sync = run_controller_time_sync()
    early_errors = wired_station_route_issues(stations)
    if controller_time_sync.get("status") != "ok":
        early_errors.insert(0, controller_time_sync.get("warning") or "Machine-dog clock is not synchronized.")
    if early_errors:
        payload = {
            "status": "preflight_failed", "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            "controller_id": config["controller_id"], "controller_time_sync": controller_time_sync,
            "errors": early_errors, "stations": [], "clock_errors": [],
            "clock_required": config["clock_snapshots"]["required"],
            "cross_host_clock": {"phase": "preflight", "status": "not_run"},
        }
        return 2, payload
    stations = resolve_station_hosts(stations)
    controller_imu_result, controller_video_result, results, clock, clock_errors, code_versions = collect_joint_preflight(config, args, stations)
    status = (
        "ok"
        if all(result["remote_status"] == "ok" for result in results)
        and not controller_imu_blocks_result(controller_imu_result)
        and not controller_video_blocks_result(controller_video_result)
        and code_versions.get("status") in ("ok", "warning")
        and (not config["clock_snapshots"]["required"] or not clock_errors)
        else "preflight_failed"
    )
    payload = {
        "status": status,
        "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "controller_id": config["controller_id"],
        "errors": list(code_versions.get("errors") or []),
        "controller_time_sync": controller_time_sync,
        "cross_host_clock": clock,
        "clock_errors": clock_errors,
        "clock_required": config["clock_snapshots"]["required"],
        "code_versions": code_versions,
        "controller_imu": {
            "preflight": controller_imu_result,
        },
        "controller_video": {"preflight": controller_video_result},
        "stations": build_station_records(stations, results, None),
        "summary": {
            "station_count": len(results),
            "ok_stations": sum(1 for result in results if result["remote_status"] == "ok"),
            "failed_stations": sum(1 for result in results if result["remote_status"] != "ok"),
            "controller_imu_status": controller_imu_result["status"],
            "controller_imu_required": bool(controller_imu_result.get("required")),
            "controller_video_status": controller_video_result.get("status"),
            "controller_video_required": bool(controller_video_result.get("required")),
        },
    }
    payload["health_matrix"] = build_preflight_health_matrix(controller_imu_result, payload["stations"])
    return (0 if status == "ok" else 2), payload


def run_capture(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]],
                progress: Callable[[dict[str, Any]], None] | None = None) -> tuple[int, dict[str, Any]]:
    def notify_progress(phase: str, **details: Any) -> None:
        if progress is not None:
            progress({"phase": phase, **details})

    notify_progress("ready_check")
    hardware_triggered = bool(getattr(args, "hardware_trigger", False))
    require_complete = bool(config.get("require_complete_joint_capture"))
    if require_complete:
        args = clone_args(args, allow_no_camera=True,
                          skip_preflight=bool(getattr(args, "ui_prechecked", False)),
                          force_capture=False, no_clock_snapshots=False)
    route_errors = wired_station_route_issues(stations)
    if route_errors:
        notify_progress("ready_check_done", ready=False, stations=[])
        return 2, {"status": "preflight_failed", "recording_started": False,
                   "requested_name": args.name, "errors": route_errors, "stations": []}
    stations = resolve_station_hosts(stations)
    if getattr(args, "ui_require_armed", False) or hardware_triggered or getattr(args, "mode_profile", "") in {"joint-single-rx03-77g", "joint-dual-rx03-77g"}:
        ready = check_radar_ready(stations, args)
        notify_progress("ready_check_done", ready=ready["status"] == "ready",
                        stations=ready["stations"], station_sensors=getattr(args, "ui_station_sensors", {}))
        if ready["status"] != "ready":
            return 2, {
                "status": "preflight_failed", "recording_started": False,
                "requested_name": args.name, "errors": ready["errors"], "radar_ready": ready,
                "stations": [{"station_id": item["station_id"], "host": item["host"],
                              "preflight": {"remote_status": "ok" if item["ready"] else "not_ready"}}
                             for item in ready["stations"]],
            }
    controller_time_sync = run_controller_time_sync()
    if controller_time_sync.get("status") != "ok":
        return 2, {"status": "preflight_failed", "recording_started": False,
                   "requested_name": args.name, "controller_time_sync": controller_time_sync,
                   "errors": [controller_time_sync.get("warning") or
                              "Machine-dog clock is not synchronized."], "stations": []}
    output_root = Path(config["output_root"]).expanduser()
    session_dir = make_session_dir(output_root, config["controller_id"], args.name)
    reports_dir = session_dir / "reports"
    pulled_root = session_dir / "stations"
    capture_id = args.capture_id.strip() or time.strftime("exp%Y%m%d_%H%M%S")
    result_mode = resolve_result_mode(config, args)
    operator_request = build_operator_request_record(args, result_mode, stations)
    write_json(session_dir / "fleet_config_snapshot.json", config)
    write_json(reports_dir / "operator_request.json", operator_request)

    controller_imu_preflight = disabled_controller_imu_result(
        bool(config["controller_imu"].get("required")),
        "Controller IMU preflight was skipped.",
    )
    controller_video_preflight = disabled_controller_video_result(
        bool(config["controller_video"].get("required")), "Robot video preflight was skipped.")
    preflight_results: list[dict[str, Any]] | None = None
    # Required robot telemetry must be checked even in fast/skip-preflight modes.
    if (config["preflight_before_capture"] and not args.skip_preflight) or config["controller_imu"].get("required"):
        notify_progress("preflight")
        controller_imu_preflight, controller_video_preflight, preflight_results, preflight_clock, preflight_clock_errors = collect_joint_preflight(config, args, stations)
        write_json(reports_dir / "controller_imu_preflight.json", controller_imu_preflight)
        write_json(reports_dir / "controller_video_preflight.json", controller_video_preflight)
        write_json(reports_dir / "clock_preflight.json", preflight_clock)
        for result in preflight_results:
            write_json(reports_dir / f"{result['station_id']}_preflight.json", result)
        failed_preflight_station_ids = [
            result["station_id"] for result in preflight_results if result["remote_status"] != "ok"
        ]
        if failed_preflight_station_ids and not args.force_capture and not require_complete:
            ok_station_ids = {result["station_id"] for result in preflight_results if result["remote_status"] == "ok"}
            if ok_station_ids and not controller_imu_blocks_result(controller_imu_preflight):
                skipped = ", ".join(failed_preflight_station_ids)
                print(
                    f"FIELD_WARNING preflight failed on {skipped}; continuing with healthy station(s): "
                    f"{', '.join(sorted(ok_station_ids))}",
                    file=sys.stderr,
                    flush=True,
                )
                stations = [station for station in stations if station["station_id"] in ok_station_ids]
                preflight_results = [result for result in preflight_results if result["station_id"] in ok_station_ids]
                operator_request["active_station_ids_after_preflight"] = [station["station_id"] for station in stations]
                write_json(reports_dir / "operator_request.json", operator_request)
        blocking_preflight_failure = (
            controller_imu_blocks_result(controller_imu_preflight)
            or controller_video_blocks_result(controller_video_preflight)
            or (config["clock_snapshots"]["required"] and bool(preflight_clock_errors))
            or (require_complete and bool(failed_preflight_station_ids))
            or not stations
            or all(result["remote_status"] != "ok" for result in preflight_results)
        )
        optional_count = sum(
            result.get("status") not in (None, "ok") and not result.get("required")
            for result in (controller_imu_preflight,)
        )
        if config["controller_video"].get("enabled") and controller_video_preflight.get("status") not in (None, "ok"):
            optional_count += 1
        if not config["clock_snapshots"]["required"]:
            optional_count += len(preflight_clock_errors)
        notify_progress("preflight_done", ok=not blocking_preflight_failure, warnings=optional_count)
        if blocking_preflight_failure and not args.force_capture:
            station_records = build_station_records(stations, preflight_results, None)
            payload = {
                "status": "preflight_failed",
                "controller_id": config["controller_id"],
                "requested_name": args.name,
                "capture_id": capture_id,
                "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
                "session_dir": str(session_dir),
                "controller_time_sync": controller_time_sync,
                "cross_host_clock": preflight_clock,
                "clock_errors": preflight_clock_errors,
                "clock_required": config["clock_snapshots"]["required"],
                "requested_capture": {
                    "duration_seconds": args.duration_seconds,
                    "countdown_seconds": args.countdown_seconds,
                },
                "operator_request": operator_request,
                "result_mode": result_mode,
                "controller_imu": {
                    "preflight": controller_imu_preflight,
                },
                "controller_video": {"preflight": controller_video_preflight},
                "stations": station_records,
                "summary": build_summary(station_records),
                "transfer_summary": build_transfer_summary(result_mode, len(stations), None),
                "clock_snapshots": {
                    "before_capture": {
                        "phase": "before_capture",
                        "status": "not_run",
                    },
                    "after_capture": {
                        "phase": "after_capture",
                        "status": "not_run",
                    },
                },
            }
            payload["recording_started"] = False
            payload["errors"] = preflight_failure_details(payload)
            payload["summary"]["controller_imu_status"] = "not_started"
            payload["summary"]["controller_video_status"] = "not_started"
            write_json(session_dir / "fleet_manifest.json", payload)
            write_text(reports_dir / "capture_overview.md", build_capture_overview_markdown(payload))
            return 2, payload
    elif getattr(args, "ui_prechecked", False):
        notify_progress("preflight_reused")

    notify_progress("clock_before")
    clock_before_capture = run_clock_snapshot(config, args, stations, "before_capture")
    write_json(reports_dir / "clock_before_capture.json", clock_before_capture)
    notify_progress("clock_before_done", ok=clock_before_capture.get("status") == "ok")

    notify_progress("sensors")
    controller_imu_capture = start_controller_imu_capture(config, args, session_dir, reports_dir, capture_id)
    controller_video_capture = start_controller_video_capture(config, args, session_dir, reports_dir, capture_id)
    if controller_video_capture.get("process") is not None:
        ready_path = session_dir / "controller_video" / "video_ready.json"
        deadline = time.monotonic() + max(6.0, config["controller_video"]["timeout_seconds"])
        while time.monotonic() < deadline and controller_video_capture["process"].poll() is None:
            if ready_path.exists():
                controller_video_capture["ready"] = json.loads(ready_path.read_text(encoding="utf-8"))
                break
            time.sleep(0.05)
        if "ready" not in controller_video_capture:
            print("FIELD_WARNING controller video has not produced its first frame; inspect video manifest after capture.", file=sys.stderr, flush=True)
    capture_results: list[dict[str, Any]] = []
    startup_errors: list[str] = []
    trigger_report: dict[str, Any] = {"status": "not_requested"}
    if require_complete:
        if config["clock_snapshots"]["required"]:
            startup_errors.extend(joint_clock_issues(
                clock_before_capture, [station["station_id"] for station in stations],
                float(config["clock_snapshots"]["max_offset_ms"])))
        if controller_video_capture.get("required") and "ready" not in controller_video_capture:
            startup_errors.append("Required robot video never produced its first frame.")
        if config["controller_imu"]["required"]:
            deadline = time.monotonic() + max(3.0, config["controller_imu"]["timeout_seconds"])
            required_csvs = [session_dir / "controller_imu" / name for name in ("imu_samples.csv", "odom_samples.csv")]
            process = controller_imu_capture.get("process")
            while process is not None and process.poll() is None and time.monotonic() < deadline:
                if all(path.exists() and path.stat().st_size > 0 for path in required_csvs):
                    break
                time.sleep(0.05)
            if process is None or process.poll() is not None or not all(path.exists() and path.stat().st_size > 0 for path in required_csvs):
                startup_errors.append("Required robot IMU/odometry recorder is not ready.")
    topic_ready = {"imu": False, "odom": False}
    process = controller_imu_capture.get("process")
    if progress is not None and process is not None:
        paths = {key: session_dir / "controller_imu" / f"{key}_samples.csv" for key in topic_ready}
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and process.poll() is None:
            topic_ready = {key: path.exists() and path.stat().st_size > 0 for key, path in paths.items()}
            if all(topic_ready.values()):
                break
            time.sleep(0.05)
    notify_progress("sensors_done", imu_topic=config["controller_imu"]["topic"],
                    odom_topic=config["controller_imu"].get("odom_topic"),
                    imu_ready=topic_ready["imu"], odom_ready=topic_ready["odom"])
    try:
        if startup_errors:
            capture_results = [{"station_id": station["station_id"], "host": station["host"],
                                "command": "capture", "remote_status": "not_run", "returncode": 2,
                                "response": {"status": "not_run", "message": "; ".join(startup_errors)}}
                               for station in stations]
        else:
            if not hardware_triggered and (getattr(args, "ui_scheduled_start", False) or (getattr(args, "mode_profile", "") == "joint-dual-rx03-77g" and len(stations) > 1)):
                lead_seconds = 25 if getattr(args, "ui_scheduled_start", False) else 15
                scheduled_start_wall_ns = time.time_ns() + lead_seconds * 1_000_000_000
                args = clone_args(args, start_at_wall_ns=scheduled_start_wall_ns)
                operator_request["scheduled_radar_start_wall_ns"] = scheduled_start_wall_ns
                write_json(reports_dir / "operator_request.json", operator_request)
                local_target = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(scheduled_start_wall_ns / 1e9))
                print(f"DUAL_RADAR_SCHEDULED start_at_local={local_target} target_wall_ns={scheduled_start_wall_ns}", file=sys.stderr, flush=True)
                notify_progress("scheduled", target_wall_ns=scheduled_start_wall_ns, local_target=local_target)
            if hardware_triggered:
                try:
                    capture_results, trigger_report = run_hardware_trigger_stations(
                        stations, args, session_dir, progress=progress)
                except Exception as exc:
                    trigger_report = {"status": "error", "error": str(exc),
                                      "report_path": str(reports_dir / "stm32_trigger.json")}
                    startup_errors.append(str(exc))
                    capture_results = [{"station_id": station["station_id"], "host": station["host"],
                                        "command": "capture", "remote_status": "trigger_failed", "returncode": 2,
                                        "response": {"status": "error", "message": str(exc)}}
                                       for station in stations]
            else:
                capture_results = run_parallel(
                    stations,
                    lambda station: run_remote_helper(
                        station,
                        apply_station_capture_args(args, station),
                        "capture", status_callback=progress),
                )
    finally:
        controller_imu_capture = stop_controller_imu_capture(controller_imu_capture)
        controller_video_capture = stop_controller_video_capture(controller_video_capture)
        write_json(reports_dir / "controller_imu_capture.json", controller_imu_capture)
        write_json(reports_dir / "controller_video_capture.json", controller_video_capture)

    notify_progress("validating")
    clock_after_capture = run_clock_snapshot(config, args, stations, "after_capture")
    write_json(reports_dir / "clock_after_capture.json", clock_after_capture)

    for result in capture_results:
        write_json(reports_dir / f"{result['station_id']}_capture.json", result)

    pull_results: list[dict[str, Any]] | None = None
    if result_mode == "pull-full":
        stations_to_pull: list[dict[str, Any]] = []
        remote_session_dirs: dict[str, str] = {}
        for station in stations:
            capture_result = next((item for item in capture_results if item["station_id"] == station["station_id"]), None)
            report = extract_remote_report_from_result(capture_result)
            remote_session_dir = str(report.get("session_dir", "")).strip()
            if capture_result is None or capture_result.get("remote_status") != "ok" or not remote_session_dir:
                continue
            stations_to_pull.append(station)
            remote_session_dirs[station["station_id"]] = remote_session_dir

        if stations_to_pull:
            pull_results = run_parallel(
                stations_to_pull,
                lambda station: run_remote_pull_session(
                    station,
                    args,
                    remote_session_dirs[station["station_id"]],
                    pulled_root / station["station_id"],
                ),
            )
            for result in pull_results:
                write_json(reports_dir / f"{result['station_id']}_pull.json", result)

    station_records = build_station_records(stations, preflight_results, capture_results, pull_results)
    write_json(reports_dir / "station_storage_summary.json", build_station_storage_summary(station_records))
    summary = build_summary(station_records, controller_imu_capture, controller_video_capture)
    controller_capture_blocks = (controller_imu_blocks_result(controller_imu_capture)
                                 or controller_video_blocks_result(controller_video_capture))
    if summary["failed_stations"] == 0 and not controller_capture_blocks:
        status = "ok"
        exit_code = 0
    elif summary["ok_stations"] == 0:
        status = "capture_failed"
        exit_code = 2
    else:
        status = "partial_failure"
        exit_code = 2

    payload = {
        "status": status,
        "controller_id": config["controller_id"],
        "requested_name": args.name,
        "capture_id": capture_id,
        "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "session_dir": str(session_dir),
        "controller_time_sync": controller_time_sync,
        "requested_capture": {
            "duration_seconds": args.duration_seconds,
            "countdown_seconds": args.countdown_seconds,
        },
        "operator_request": operator_request,
        "stm32_trigger": trigger_report,
        "result_mode": result_mode,
        "pull_results": result_mode == "pull-full",
        "controller_imu": {
            "preflight": controller_imu_preflight,
            "capture": controller_imu_capture,
        },
        "controller_video": {
            "preflight": controller_video_preflight,
            "capture": controller_video_capture,
        },
        "stations": station_records,
        "capture_plan": build_capture_plan(args, stations),
        "summary": summary,
        "transfer_summary": build_transfer_summary(result_mode, len(stations), pull_results),
        "clock_snapshots": {
            "before_capture": clock_before_capture,
            "after_capture": clock_after_capture,
        },
        "sync_assessment": build_sync_assessment(
            station_records,
            controller_imu_capture,
            {
                "before_capture": clock_before_capture,
                "after_capture": clock_after_capture,
            },
        ),
        "artifacts": {
            "fleet_manifest": "fleet_manifest.json",
            "stm32_trigger": "reports/stm32_trigger.json" if hardware_triggered else "",
            "reports_dir": "reports",
            "stations_dir": "stations" if result_mode == "pull-full" and pull_results else "",
            "controller_imu_dir": "controller_imu" if controller_imu_requested(config, args) else "",
            "controller_telemetry_manifest": "controller_imu/imu_manifest.json" if controller_imu_requested(config, args) else "",
            "fleet_config_snapshot": "fleet_config_snapshot.json",
            "controller_video_dir": "controller_video" if controller_video_requested(config, args) else "",
        },
    }
    try:
        integrity = complete_joint_integrity(
            config, station_records, controller_imu_capture, controller_video_capture,
            {"before_capture": clock_before_capture, "after_capture": clock_after_capture}, session_dir)
    except Exception as exc:
        integrity = {"status": "failed", "errors": [f"Required joint integrity check failed: {exc}"]}
    if startup_errors:
        integrity["errors"].extend(startup_errors)
        integrity["status"] = "failed"
    payload["joint_integrity"] = integrity
    if integrity["status"] == "failed":
        payload["status"] = "capture_failed"
        exit_code = 2
    write_json(session_dir / "fleet_manifest.json", payload)
    write_text(reports_dir / "capture_overview.md", build_capture_overview_markdown(payload))
    notify_progress("transfer")
    from controller_transfer import transfer_controller_session
    transfer = transfer_controller_session(config, stations, station_records, session_dir, capture_id, resolve_secret)
    payload["controller_data_transfer"] = transfer
    if transfer.get("status") == "error":
        integrity.setdefault("warnings", []).append("Controller data transfer failed; originals remain on the robot (optional).")
        print("FIELD_WARNING controller data transfer failed; local originals retained; see fleet_manifest.json.", file=sys.stderr, flush=True)
    write_json(reports_dir / "controller_data_transfer.json", transfer)
    write_json(session_dir / "fleet_manifest.json", payload)
    write_text(reports_dir / "capture_overview.md", build_capture_overview_markdown(payload))
    return exit_code, payload


def run_pull_results(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]]) -> tuple[int, dict[str, Any]]:
    session_dir = args.fleet_session_dir.expanduser().resolve()
    reports_dir = session_dir / "reports"
    output_dir = (args.output_dir.expanduser().resolve() if args.output_dir is not None else (session_dir / "stations"))
    fleet_manifest = load_fleet_manifest(session_dir)
    station_records_raw = fleet_manifest.get("stations")
    existing_records = [item for item in station_records_raw if isinstance(item, dict)] if isinstance(station_records_raw, list) else []
    existing_map = {str(item.get("station_id")): item for item in existing_records if item.get("station_id")}
    selected_ids = {station["station_id"] for station in stations}

    pullable_stations: list[dict[str, Any]] = []
    remote_session_dirs: dict[str, str] = {}
    pull_errors: list[dict[str, Any]] = []
    for station in stations:
        existing = existing_map.get(station["station_id"], {})
        capture = existing.get("capture") if isinstance(existing, dict) else None
        report = extract_remote_report_from_result(capture if isinstance(capture, dict) else None)
        remote_session_dir = str(report.get("session_dir", "")).strip()
        if not remote_session_dir:
            pull_errors.append(
                {
                    "station_id": station["station_id"],
                    "host": station["host"],
                    "command": "pull-session",
                    "returncode": 2,
                    "remote_status": "missing_remote_session_dir",
                    "response": {
                        "status": "error",
                        "message": "No remote session_dir was recorded in fleet_manifest.json for this station.",
                    },
                    "stderr_tail": [],
                }
            )
            continue
        pullable_stations.append(station)
        remote_session_dirs[station["station_id"]] = remote_session_dir

    pull_results = run_parallel(
        pullable_stations,
        lambda station: run_remote_pull_session(
            station,
            args,
            remote_session_dirs[station["station_id"]],
            output_dir / station["station_id"],
        ),
    ) if pullable_stations else []
    pull_results.extend(pull_errors)
    pull_results.sort(key=lambda item: item["station_id"])

    for result in pull_results:
        write_json(reports_dir / f"{result['station_id']}_pull.json", result)

    updated_records: list[dict[str, Any]] = []
    pull_result_map = {item["station_id"]: item for item in pull_results}
    for record in existing_records:
        station_id = str(record.get("station_id"))
        updated_record = dict(record)
        if station_id in selected_ids and station_id in pull_result_map:
            updated_record["pull"] = pull_result_map[station_id]
        updated_records.append(updated_record)

    fleet_manifest["stations"] = updated_records
    artifacts = fleet_manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        artifacts = {}
        fleet_manifest["artifacts"] = artifacts
    artifacts["stations_dir"] = (
        "stations" if output_dir == (session_dir / "stations") else str(output_dir)
    )
    fleet_manifest["transfer_summary"] = build_transfer_summary("pull-full", len(stations), pull_results)
    write_json(session_dir / "fleet_manifest.json", fleet_manifest)

    ok_count = sum(1 for item in pull_results if item.get("remote_status") == "ok")
    failed_count = sum(1 for item in pull_results if item.get("remote_status") != "ok")
    if failed_count == 0:
        status = "ok"
        exit_code = 0
    elif ok_count == 0:
        status = "pull_failed"
        exit_code = 2
    else:
        status = "partial_failure"
        exit_code = 2

    payload = {
        "status": status,
        "fleet_session_dir": str(session_dir),
        "output_dir": str(output_dir),
        "station_count": len(stations),
        "ok_stations": ok_count,
        "failed_stations": failed_count,
        "results": pull_results,
        "transfer_summary": fleet_manifest["transfer_summary"],
        "manifest_path": str(session_dir / "fleet_manifest.json"),
    }
    return exit_code, payload


def build_ui_base_args(args: argparse.Namespace) -> argparse.Namespace:
    return clone_args(
        args,
        radar_config=None,
        duration_seconds=None,
        countdown_seconds=None,
        reader_mode=None,
        camera_device=None,
        camera_width=None,
        camera_height=None,
        camera_fps=None,
        camera_fourcc=None,
        camera_format=None,
        camera_save_every_n=None,
        imu_port=None,
        imu_baud=None,
        allow_no_camera=True,
        allow_no_imu=False,
        skip_network_setup=False,
        no_controller_imu=False,
        no_clock_snapshots=False,
        clock_snapshot_samples=None,
        strict_camera=False,
        strict_imu=False,
        skip_preflight=False,
        force_capture=False,
        pull=False,
        capture_id="",
        seconds=None,
        countdown=None,
        mode="field",
        name="",
        station_overrides={},
    )


def preflight_failure_details(payload):
    if payload.get("radar_ready"):
        return list(payload.get("errors") or [])
    errors = list(payload.get("errors") or [])
    if payload.get("clock_required", True):
        errors.extend(payload.get("clock_errors") or [])
    for key in ("controller_imu", "controller_video"):
        result = (payload.get(key) or {}).get("preflight") or {}
        if result.get("required") and result.get("status") not in (None, "ok"):
            response = result.get("response") or {}
            detail = response.get("error") or response.get("message") or "; ".join(result.get("stderr_tail") or [])
            errors.append(f"{key}: {result.get('status')}: {detail or 'No valid preflight result.'}")
    for station in payload.get("stations", []):
        result = station.get("preflight") or {}
        if result.get("remote_status") not in (None, "ok"):
            response = result.get("response") or {}
            report = response.get("report") or {}
            detail = (report.get("errors") or response.get("error") or response.get("message")
                      or result.get("stderr_tail") or result.get("stdout_tail") or "No valid remote report.")
            errors.append(f"{station.get('station_id')}: {result.get('remote_status')}: {detail}")
    return list(dict.fromkeys(errors))


def optional_preflight_warnings(payload: dict[str, Any]) -> list[str]:
    warnings = [] if payload.get("clock_required", True) else list(payload.get("clock_errors") or [])
    for key in ("controller_imu", "controller_video"):
        result = (payload.get(key) or {}).get("preflight") or {}
        if key == "controller_video" and result.get("status") == "disabled":
            continue
        if not result.get("required") and result.get("status") not in (None, "ok"):
            warnings.append(f"{key}: {result.get('status')} (optional)")
    camera_required = {station.get("station_id"): station.get("camera_required", True) for station in payload.get("stations", [])}
    health = (payload.get("health_matrix") or {}).get("stations") or []
    for station in health:
        station_id = station.get("station_id")
        if not camera_required.get(station_id, True) and not station.get("camera_ok"):
            warnings.append(f"{station_id}: station camera unavailable or disabled (optional)")
    return warnings


def summarize_payload_for_terminal(payload: dict[str, Any]) -> dict[str, Any]:
    if payload.get("status") == "preflight_failed":
        return {
            "status": "preflight_failed", "stage": "preflight", "recording_started": False,
            "message": "Required checks failed; recording was not started. Inspect errors below.",
            "requested_name": payload.get("requested_name"), "session_dir": payload.get("session_dir"),
            "errors": preflight_failure_details(payload),
            "warnings": optional_preflight_warnings(payload),
            "controller_imu_status": ((payload.get("controller_imu") or {}).get("preflight") or {}).get("status"),
            "controller_video_status": ((payload.get("controller_video") or {}).get("preflight") or {}).get("status"),
            "cross_host_clock": payload.get("cross_host_clock"),
            "stations": [{"station_id": s.get("station_id"), "host": s.get("host"),
                          "preflight_status": (s.get("preflight") or {}).get("remote_status"),
                          "capture_status": "not_started"} for s in payload.get("stations", [])],
        }
    summary = payload.get("summary") if isinstance(payload.get("summary"), dict) else {}
    controller_time_sync = (
        payload.get("controller_time_sync") if isinstance(payload.get("controller_time_sync"), dict) else {}
    )
    station_rows: list[dict[str, Any]] = []
    stations = payload.get("stations") if isinstance(payload.get("stations"), list) else []
    for station in stations:
        if not isinstance(station, dict):
            continue
        capture = station.get("capture") if isinstance(station.get("capture"), dict) else {}
        response = capture.get("response") if isinstance(capture.get("response"), dict) else {}
        station_summary = response.get("summary") if isinstance(response.get("summary"), dict) else {}
        report = response.get("report") if isinstance(response.get("report"), dict) else {}
        manifest = report.get("manifest") if isinstance(report.get("manifest"), dict) else {}
        camera = manifest.get("camera") if isinstance(manifest.get("camera"), dict) else {}
        camera_frames = station_summary.get("camera_saved_frames")
        camera_error = camera.get("error") or ("No camera frames saved." if camera.get("enabled") and camera_frames == 0 else "")
        station_rows.append(
            {
                "station_id": station.get("station_id"),
                "host": station.get("host"),
                "configured_host": station.get("configured_host"),
                "capture_status": capture.get("remote_status"),
                "radar_config": station_summary.get("radar_config"),
                "radar_raw_bytes": station_summary.get("radar_raw_bytes"),
                "radar_quality_status": station_summary.get("radar_quality_status"),
                "radar_usable_frames": station_summary.get("radar_usable_frames_by_content"),
                "radar_expected_frames": station_summary.get("radar_expected_frames"),
                "raw_size_matches": station_summary.get("radar_raw_size_matches"),
                "zero_fill_bytes": station_summary.get("radar_file_reader_zero_filled_bytes"),
                "zero_fill_ratio": station_summary.get("radar_file_reader_zero_fill_ratio"),
                "zero_tail": station_summary.get("radar_zero_tail_detected"),
                "imu_samples": station_summary.get("imu_samples"),
                "camera_frames": camera_frames,
                "camera_error": camera_error,
                "radar_start_delay_s": station_summary.get("requested_start_to_sensor_start_seconds"),
                "save_after_stop_s": station_summary.get("sensor_stop_to_raw_written_seconds"),
            }
        )
    return {
        "status": payload.get("status"),
        "requested_name": payload.get("requested_name"),
        "capture_id": payload.get("capture_id"),
        "session_dir": payload.get("session_dir"),
        "result_mode": payload.get("result_mode"),
        "controller_time_sync": {
            "status": controller_time_sync.get("status"),
            "local": (controller_time_sync.get("after") or {}).get("local")
            if isinstance(controller_time_sync.get("after"), dict)
            else "",
            "warning": controller_time_sync.get("warning", ""),
        },
        "summary": summary,
        "controller_video_error": (
            "Robot video capture failed; check reports/controller_video_capture.json."
            if summary.get("controller_video_status") == "error"
            else ""
        ),
        "joint_integrity": payload.get("joint_integrity"),
        "warnings": (payload.get("joint_integrity") or {}).get("warnings", []),
        "stations": station_rows,
    }


def summarize_preflight_for_terminal(payload: dict[str, Any]) -> dict[str, Any]:
    summary = payload.get("summary") if isinstance(payload.get("summary"), dict) else {}
    health = payload.get("health_matrix") if isinstance(payload.get("health_matrix"), dict) else {}
    controller = health.get("controller_imu") if isinstance(health.get("controller_imu"), dict) else {}
    video = (payload.get("controller_video") or {}).get("preflight") or {}
    video_response = video.get("response") or {}
    controller_time_sync = (
        payload.get("controller_time_sync") if isinstance(payload.get("controller_time_sync"), dict) else {}
    )
    stations = health.get("stations") if isinstance(health.get("stations"), list) else []

    station_rows: list[dict[str, Any]] = []
    for item in stations:
        if not isinstance(item, dict):
            continue
        station_rows.append(
            {
                "station_id": item.get("station_id"),
                "status": item.get("status"),
                "radar": bool(item.get("radar_usb_present")),
                "network": bool(item.get("network_up")),
                "imu": {
                    "port": item.get("imu_port") or "",
                    "hits": int(item.get("imu_probe_hits") or 0),
                },
                "camera": bool(item.get("camera_ok")),
                "errors": item.get("errors") or [],
            }
        )

    return {
        "status": payload.get("status"),
        "errors": preflight_failure_details(payload),
        "warnings": optional_preflight_warnings(payload),
        "cross_host_clock": payload.get("cross_host_clock"),
        "checked_at_local": payload.get("checked_at_local"),
        "summary": {
            "station_count": summary.get("station_count"),
            "ok_stations": summary.get("ok_stations"),
            "failed_stations": summary.get("failed_stations"),
        },
        "controller_imu": {
            "status": controller.get("status"),
            "required": bool(controller.get("required")),
            "topic": controller.get("topic"),
            "message": controller.get("message") or "",
        },
        "controller_video": {
            "status": video.get("status"), "required": bool(video.get("required")),
            "message": video_response.get("error") or video_response.get("message") or "",
        },
        "controller_time_sync": {
            "status": controller_time_sync.get("status"),
            "local": (controller_time_sync.get("after") or {}).get("local")
            if isinstance(controller_time_sync.get("after"), dict)
            else "",
            "warning": controller_time_sync.get("warning", ""),
        },
        "stations": station_rows,
    }


def build_recent_session_list(config: dict[str, Any], limit: int = 8) -> list[dict[str, Any]]:
    root = Path(config["output_root"]).expanduser()
    if not root.exists():
        return []
    session_dirs = resolve_ingest_session_dirs(root, [])[:limit]
    items: list[dict[str, Any]] = []
    for session_dir in session_dirs:
        try:
            manifest = load_fleet_manifest(session_dir)
        except Exception as exc:
            items.append({"session_name": session_dir.name, "status": "error", "message": str(exc)})
            continue
        summary = manifest.get("summary") if isinstance(manifest.get("summary"), dict) else {}
        items.append(
            {
                "session_name": session_dir.name,
                "status": manifest.get("status"),
                "requested_name": manifest.get("requested_name"),
                "capture_id": manifest.get("capture_id"),
                "result_mode": manifest.get("result_mode"),
                "ok_stations": summary.get("ok_stations"),
                "station_count": summary.get("station_count"),
                "total_camera_saved_frames": summary.get("total_camera_saved_frames"),
                "total_imu_samples": summary.get("total_imu_samples"),
            }
        )
    return items


def new_ui_log_path(label: str) -> Path:
    log_dir = REPO_ROOT / "logs" / "ui"
    log_dir.mkdir(parents=True, exist_ok=True)
    return log_dir / f"{time.strftime('%Y%m%d_%H%M%S')}__{sanitize_label(label)}__{time.time_ns() % 1_000_000_000:09d}.log"


def ui_style(message: str, color: str, terminal: Any) -> str:
    if not terminal.isatty() or os.environ.get("NO_COLOR") is not None or os.environ.get("TERM") == "dumb":
        return message
    codes = {"red": "31", "green": "32", "orange": "38;5;208", "cyan": "36"}
    return f"\x1b[{codes[color]}m{message}\x1b[0m"


class UiCaptureProgress:
    def __init__(self, terminal: Any, station_ids: list[str]) -> None:
        self.terminal = terminal
        self.station_ids = set(station_ids)
        self.started: set[str] = set()
        self.lock = threading.Lock()
        self.stop_countdown = threading.Event()
        self.countdown_thread: threading.Thread | None = None
        self.overwrite_active = False
        self.started_announced = False

    def _line_locked(self, message: str) -> None:
        if self.overwrite_active:
            self.terminal.write("\r\x1b[2K")
            self.overwrite_active = False
        self.terminal.write(message + "\n")
        self.terminal.flush()

    def line(self, message: str) -> None:
        with self.lock:
            self._line_locked(message)

    def result(self, message: str, color: str) -> None:
        self.line(ui_style(message, color, self.terminal))

    def countdown(self, target_wall_ns: int) -> None:
        last_seconds = None
        while not self.stop_countdown.is_set():
            remaining_ns = target_wall_ns - time.time_ns()
            seconds = max(0, (remaining_ns + 999_999_999) // 1_000_000_000)
            if seconds != last_seconds:
                message = ui_style(f"  >>> 计划起采倒计时 {seconds:02d} 秒 | 请准备移动平台 <<<", "orange", self.terminal)
                if self.terminal.isatty():
                    with self.lock:
                        self.terminal.write("\r\x1b[2K" + message)
                        self.terminal.flush()
                        self.overwrite_active = True
                else:
                    self.line(message)
                last_seconds = seconds
            if seconds == 0:
                with self.lock:
                    if not self.started_announced:
                        self._line_locked("  计划时刻已到，等待雷达起采确认；暂勿移动平台")
                return
            self.stop_countdown.wait(0.08)

    def handle(self, event: dict[str, Any]) -> None:
        phase = event.get("phase")
        if phase == "ready_check":
            self.line("雷达侧：正在检查连接与预配置状态…")
        elif phase == "ready_check_done":
            rows = event.get("stations") or []
            sensors = event.get("station_sensors") or {}
            if not rows:
                self.result("  雷达：已就绪" if event.get("ready") else "  雷达：有站点未就绪，请执行选项 5", "green" if event.get("ready") else "red")
            for row in rows:
                station_id = str(row["station_id"])
                label = {"radar0": "NX1", "radar1": "NX2"}.get(station_id, station_id)
                radar_ready = bool(row.get("ready"))
                self.result(f"  {label} 雷达：{'已就绪' if radar_ready else '未就绪'}", "green" if radar_ready else "red")
                sensor = sensors.get(station_id) or {}
                for key, name in (("camera", "相机"), ("imu", "串口 IMU")):
                    status = sensor.get(key, "unknown")
                    state = {"ready": "选项 5 检查可用", "disabled": "未启用", "unavailable": "选项 5 未检测到"}.get(status, "状态未记录")
                    device = sensor.get("camera_device" if key == "camera" else "imu_port") if status == "ready" else ""
                    self.result(f"  {label} {name}：{state}" + (f"（{device}）" if device else ""),
                                "green" if status == "ready" else "orange")
        elif phase == "preflight":
            self.line("正在预检雷达及可选传感器，请稍候…")
        elif phase == "preflight_done":
            ok = event.get("ok")
            warnings = int(event.get("warnings") or 0)
            message = "  采前检查：通过" if ok else "  采前检查：未通过"
            if warnings:
                message += f"；可选设备提醒 {warnings} 项"
            self.result(message, "red" if not ok else "orange" if warnings else "green")
        elif phase == "preflight_reused":
            self.result("  雷达侧采前检查：沿用选项 5 的结果", "green")
        elif phase == "clock_before":
            self.line("正在记录采前时间状态…")
        elif phase == "clock_before_done":
            ok = event.get("ok")
            self.result("  采前时间：检查通过" if ok else "  采前时间：记录不完整，采后需复核", "green" if ok else "orange")
        elif phase == "sensors":
            self.line("机器狗：正在准备 ROS 话题记录…")
        elif phase == "sensors_done":
            for key, label in (("imu", "机器狗 IMU"), ("odom", "机器狗里程计")):
                topic = event.get(f"{key}_topic") or "未配置"
                ready = bool(event.get(f"{key}_ready"))
                self.result(f"  {label}（{topic}）：{'已收到数据' if ready else '未收到数据，雷达继续采集'}",
                            "green" if ready else "orange")
        elif phase == "scheduled":
            self.line("\n" + "=" * 52)
            self.line(f"计划起采时间：{event['local_target']}（以两站起采确认作为行动信号）")
            self.countdown_thread = threading.Thread(
                target=self.countdown, args=(int(event["target_wall_ns"]),), daemon=True
            )
            self.countdown_thread.start()
        elif phase == "station_armed":
            self.line(f"{event['station_id']}：已准备接收硬件触发")
        elif phase == "trigger_armed":
            self.result(f"两台雷达已就绪，准备发送 STM32 触发（每路 {event['pairs']} 帧）", "orange")
        elif phase == "trigger_started":
            self.result("STM32 已接受触发命令，等待两台雷达的首包确认…", "orange")
        elif phase == "station_started":
            with self.lock:
                self.started.add(str(event["station_id"]))
                all_started = self.station_ids.issubset(self.started)
                announce = all_started and not self.started_announced
                if announce:
                    self.started_announced = True
            if announce:
                self.stop_countdown.set()
                self.result("\n>>> 雷达已起采，开始移动平台！ <<<\n" + "=" * 52, "green")
            elif not all_started:
                self.line(f"{event['station_id']} 已起采，等待其他雷达站确认…")
        elif phase == "station_saved":
            self.line(f"{event['station_id']} 雷达数据已保存")
        elif phase == "validating":
            self.stop_countdown.set()
            with self.lock:
                started = bool(self.started)
            if started:
                self.line("雷达录制结束，正在校验数据和同步文件…")
            else:
                self.result("雷达未起采，正在整理失败原因…", "red")
        elif phase == "transfer":
            self.line("正在同步机器狗记录并整理会话…")

    def close(self) -> None:
        self.stop_countdown.set()
        if self.countdown_thread is not None:
            self.countdown_thread.join(timeout=1)
        with self.lock:
            if self.overwrite_active:
                self.terminal.write("\r\x1b[2K\n")
                self.terminal.flush()
                self.overwrite_active = False


def print_ui_capture_result(payload: dict[str, Any], log_path: Path, terminal: Any) -> None:
    status = payload.get("status")
    if status == "ok":
        print("\n" + ui_style("采集完成：所有必需雷达数据通过校验", "green", terminal), file=terminal)
    elif status == "preflight_failed":
        print("\n" + ui_style("未开始采集：雷达连接、预配置或采前检查未通过", "red", terminal), file=terminal)
    else:
        print("\n" + ui_style("采集未完整成功，请检查雷达站状态和详细日志", "red", terminal), file=terminal)

    ready = payload.get("radar_ready") or {}
    for item in ready.get("stations") or []:
        if not item.get("ready"):
            print(f"  {item['station_id']}: 未就绪；请检查网络并重新执行选项 5", file=terminal)
    for station in payload.get("stations") or []:
        response = (station.get("capture") or {}).get("response") or {}
        summary = response.get("summary") or {}
        frames = summary.get("radar_usable_frames_by_content")
        expected = summary.get("radar_expected_frames")
        if frames is not None and expected is not None:
            quality = "通过" if summary.get("radar_quality_status") == "ok" else "需检查"
            print(f"  {station['station_id']}: 雷达 {frames}/{expected} 帧，校验{quality}", file=terminal)
        elif status != "preflight_failed":
            print(f"  {station.get('station_id')}: 未保存有效雷达数据，原因见日志", file=terminal)
        if summary.get("session_dir"):
            print(f"    数据：{summary['session_dir']}", file=terminal)

    integrity = payload.get("joint_integrity") or {}
    overlap = integrity.get("radar_common_overlap_seconds")
    if overlap is not None:
        print(f"  双雷达共同录制：{overlap:.2f} 秒", file=terminal)
    summary = payload.get("summary") or {}
    if summary:
        streams = summary.get("controller_telemetry_streams") or {}
        print(f"  机器狗：IMU {summary.get('controller_imu_samples', 0)} 条，"
              f"里程计 {(streams.get('odom') or {}).get('sample_count', 0)} 条", file=terminal)
    transfer = payload.get("controller_data_transfer") or {}
    if transfer.get("status") == "ok":
        print("  机器狗数据：已同步到雷达站", file=terminal)

    if status == "preflight_failed" and not ready:
        print("  请查看日志中的预检失败项，修复后再采集", file=terminal)
    warnings = integrity.get("warnings") or []
    if any("clock" in str(item).lower() for item in warnings):
        print(ui_style("  注意：时间同步检查不完整，跨设备时间对齐精度需复核", "orange", terminal), file=terminal)
    if warnings:
        print(ui_style(f"  可选传感器或时间同步提醒 {len(warnings)} 项，详见日志", "orange", terminal), file=terminal)
    if payload.get("session_dir"):
        print(f"机器狗会话：{payload['session_dir']}", file=terminal)
    print(f"详细日志：{log_path}", file=terminal, flush=True)


def run_ui_capture(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]]) -> dict[str, Any]:
    log_path = new_ui_log_path(args.name)
    terminal = sys.stdout
    print(f"\n准备采集：{args.name} | 雷达站 {', '.join(station['station_id'] for station in stations)} | {args.duration_seconds:g} 秒", file=terminal, flush=True)
    progress = UiCaptureProgress(terminal, [station["station_id"] for station in stations])
    payload: dict[str, Any]
    with log_path.open("x", encoding="utf-8") as log:
        os.chmod(log_path, 0o600)
        try:
            with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                _exit_code, payload = run_capture(args, config, stations, progress=progress.handle)
            log.write(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
        except Exception as exc:
            traceback.print_exc(file=log)
            payload = {"status": "error", "requested_name": args.name, "stations": []}
        finally:
            progress.close()
    print_ui_capture_result(payload, log_path, terminal)
    return payload


def disarm_ui_station(station: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    stop = run_remote_helper(station, args, "disarm")
    for _ in range(5):
        status = run_remote_helper(station, args, "ready-status")
        if status.get("remote_status") != "ready":
            return {"station_id": station["station_id"], "status": "ok", "stop": stop, "last_status": status}
        time.sleep(0.5)
    return {"station_id": station["station_id"], "status": "error", "stop": stop,
            "message": "Previous radar service did not stop."}


def ui_station_sensor_snapshot(preflight: dict[str, Any]) -> dict[str, dict[str, Any]]:
    sensors: dict[str, dict[str, Any]] = {}
    for station in preflight.get("stations") or []:
        report = extract_remote_report((station.get("preflight") or {}).get("response"))
        settings = report.get("effective_settings") or {}
        camera = report.get("camera") or {}
        imu = report.get("imu") or {}
        imu_port = str(settings.get("imu_port") or "").strip()
        camera_enabled = bool(settings.get("camera_enabled"))
        sensors[station["station_id"]] = {
            "camera": ("disabled" if not camera_enabled else "ready" if camera.get("ok") else "unavailable"),
            "camera_device": str(camera.get("resolved_device") or camera.get("device") or ""),
            "imu": ("disabled" if imu_port.lower() in ("", "none") else
                    "ready" if int(imu.get("probe_hits") or 0) > 0 else "unavailable"),
            "imu_port": str(imu.get("port") or imu_port),
        }
    return sensors


def ui_issue_text(value: Any, limit: int = 180) -> str:
    text = " ".join(str(value).split())
    return text if len(text) <= limit else text[:limit - 1] + "…"


def run_ui_prepare(args: argparse.Namespace, config: dict[str, Any], stations: list[dict[str, Any]]) -> dict[str, Any]:
    log_path = new_ui_log_path("prepare")
    terminal = sys.stdout
    print("正在检查网络、时间及传感器…", file=terminal, flush=True)
    preflight: dict[str, Any] = {}
    disarm_results: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    ready: dict[str, Any] = {}
    failed = False
    failure_reason = ""
    serial_ready = not getattr(args, "hardware_trigger", False)
    with log_path.open("x", encoding="utf-8") as log:
        os.chmod(log_path, 0o600)
        try:
            with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
                preflight_args = clone_args(args, command="preflight", allow_no_camera=True, allow_no_imu=True)
                _code, preflight = run_preflight(preflight_args, config, stations)
                if getattr(args, "hardware_trigger", False):
                    try:
                        with Stm32Trigger(getattr(args, "stm32_port")) as trigger:
                            trigger.confirm_idle()
                            serial_ready = True
                    except Exception as exc:
                        log.write(f"STM32 serial port check failed: {exc}\n")
                if preflight.get("status") == "ok":
                    selected = resolve_station_hosts(stations)
                    disarm_results = run_parallel(
                        selected, lambda station: disarm_ui_station(station, args)
                    )
                    if all(item.get("status") == "ok" for item in disarm_results):
                        results = run_parallel(
                            selected,
                            lambda station: run_remote_helper(
                                station,
                                apply_station_args(args, station, apply_countdown_offset=False),
                                "arm",
                            ),
                        )
                        if all(item.get("remote_status") == "ready" for item in results):
                            ready = check_radar_ready(selected, args)
            log.write(json.dumps({"preflight": preflight, "disarm": disarm_results, "arm": results, "ready": ready}, indent=2, ensure_ascii=False) + "\n")
        except Exception as exc:
            traceback.print_exc(file=log)
            failed = True
            failure_reason = ui_issue_text(exc)
    clock_warnings = optional_preflight_warnings(preflight) if preflight else []
    code_versions = preflight.get("code_versions") or {}
    if code_versions:
        if code_versions.get("status") == "ok":
            versions = ", ".join(
                f"{row.get('station_id')}={row.get('version')}"
                for row in code_versions.get("stations") or []
            )
            print(ui_style(f"代码版本：已校验（{versions}）", "green", terminal), file=terminal)
        elif code_versions.get("status") == "mismatch":
            print(ui_style("代码版本不一致：请先执行选项 4“同步代码”", "red", terminal), file=terminal)
        else:
            print(ui_style("代码版本：未完成校验（单雷达可继续，多雷达请先同步）", "orange", terminal), file=terminal)
    if preflight.get("status") != "ok":
        print(ui_style("采前检查：未通过，本次没有预配置雷达", "red", terminal), file=terminal)
        failed_station_ids = {
            str(station.get("station_id")) for station in preflight.get("stations") or []
            if (station.get("preflight") or {}).get("remote_status") not in (None, "ok")
        }
        for issue in preflight_failure_details(preflight):
            if any(str(issue).startswith(f"{station_id}:") for station_id in failed_station_ids):
                continue
            print(ui_style(f"  {ui_issue_text(issue)}", "red", terminal), file=terminal)
        for station in preflight.get("stations") or []:
            station_result = station.get("preflight") or {}
            if station_result.get("remote_status") in (None, "ok"):
                continue
            report = extract_remote_report(station_result.get("response"))
            errors = report.get("errors") or []
            network = report.get("network") or {}
            link_state = str(network.get("addr_state") or "")
            dca_error = any("DCA1000" in str(item) for item in errors)
            if link_state and " UP " not in f" {link_state} ":
                message = "雷达专用网口未连接或未配置，请检查网线与供电"
            else:
                message = ("DCA 无应答，请检查供电和网线" if dca_error else
                           ui_issue_text(errors[0]) if errors else station_remote_issue(station_result))
            print(ui_style(f"  {station['station_id']}（{station.get('host') or 'IP 未知'}）：{message}", "red", terminal), file=terminal)
    elif failed:
        print(ui_style(f"预配置中断：{failure_reason or '未知错误'}", "red", terminal), file=terminal)
    else:
        color = "orange" if clock_warnings else "green"
        message = "采前检查：通过" + (f"，有 {len(clock_warnings)} 项提醒" if clock_warnings else "")
        print(ui_style(message, color, terminal), file=terminal)
    telemetry = ((preflight.get("controller_imu") or {}).get("preflight") or {}).get("response") or {}
    streams = telemetry.get("streams") or {}
    if preflight.get("controller_imu"):
        for key, label in (("imu", "机器狗 IMU 话题"), ("odom", "机器狗里程计话题")):
            stream = streams.get(key) or {}
            topic = stream.get("topic") or config["controller_imu"].get("topic" if key == "imu" else "odom_topic") or "未配置"
            ok = int(stream.get("sample_count") or 0) > 0
            print(ui_style(f"  {label}（{topic}）：{'可用' if ok else '未收到数据'}", "green" if ok else "orange", terminal), file=terminal)
    else:
        print(ui_style("  机器狗 IMU/里程计：未检查（请先解决上方错误）", "orange", terminal), file=terminal)
    for item in results:
        if item["remote_status"] == "ready":
            print(ui_style(f"  {item['station_id']}: 已准备好", "green", terminal), file=terminal)
        else:
            print(ui_style(f"  {item['station_id']}: 失败，{station_remote_issue(item)}", "red", terminal), file=terminal)
    for item in disarm_results:
        if item.get("status") != "ok":
            print(ui_style(f"  {item['station_id']}: 旧预配置未能停止", "red", terminal), file=terminal)
    if getattr(args, "hardware_trigger", False):
        print(ui_style("  STM32 串口：协议回包正常" if serial_ready else
                       "  STM32 串口：不可用，检查 USB 线和设备权限",
                       "green" if serial_ready else "red", terminal), file=terminal)
    prepared = not failed and serial_ready and ready.get("status") == "ready"
    if prepared:
        print(ui_style("预配置完成，可以按 5 开始采集", "green", terminal), file=terminal)
    elif preflight.get("status") == "ok":
        for issue in (ready.get("errors") or [])[:4]:
            print(ui_style(f"  {ui_issue_text(issue)}", "red", terminal), file=terminal)
        print(ui_style("雷达尚未全部就绪，请解决上方错误后重试选项 5", "red", terminal), file=terminal)
    if any("clock" in str(item).lower() for item in clock_warnings):
        print(ui_style("  注意：时间同步检查不完整，跨设备对齐精度需复核", "orange", terminal), file=terminal)
    sensors = ui_station_sensor_snapshot(preflight)
    for station_id, sensor in sensors.items():
        for key, label in (("camera", "雷达侧相机"), ("imu", "雷达侧串口 IMU")):
            status = sensor[key]
            text = "可用" if status == "ready" else "未启用" if status == "disabled" else "未检测到"
            device = sensor.get("camera_device" if key == "camera" else "imu_port") if status == "ready" else ""
            color = "green" if status == "ready" else "orange"
            print(ui_style(f"  {station_id} {label}：{text}" + (f"（{device}）" if device else ""), color, terminal), file=terminal)
    print(f"详细日志：{log_path}", file=terminal, flush=True)
    return {"prepared": prepared, "station_sensors": sensors}




def parse_ui_number(text: str, label: str, default: float | None = None, *, allow_zero: bool = False) -> float | None:
    if not text:
        return default
    try:
        value = float(text)
    except ValueError as exc:
        raise ValueError(f"{label}请输入数字") from exc
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f"{label}必须是{'非负' if allow_zero else '大于零的'}有限数字")
    return value


def ui_state_path(config: dict[str, Any]) -> Path:
    return Path(config["config_path"]).with_name("nxui_selection.json")


def ui_station_hosts(config: dict[str, Any]) -> dict[str, str]:
    return {station["station_id"]: station["host"] for station in config["stations"]}


def load_ui_selection(config: dict[str, Any]) -> dict[str, Any]:
    mode_name = next((name for name, mode in config["ui_modes"].items()
                      if mode["operator_profile"] == config["default_operator_profile"]), next(iter(config["ui_modes"])))
    profile_name = next((name for name, profile in config["radar_config_library"].items()
                         if profile["config_path"] == config["default_radar_config"]),
                        next(iter(config["radar_config_library"])))
    selection = {"mode": mode_name, "radar_profile": profile_name, "prepared": False,
                 "station_hosts": ui_station_hosts(config)}
    path = ui_state_path(config)
    if path.exists():
        try:
            saved = json.loads(path.read_text(encoding="utf-8"))
            if saved.get("mode") in config["ui_modes"] and saved.get("radar_profile") in config["radar_config_library"]:
                selection.update({"mode": saved["mode"], "radar_profile": saved["radar_profile"]})
                selected_path = config["radar_config_library"][selection["radar_profile"]]["config_path"]
                selection["prepared"] = (bool(saved.get("prepared"))
                                         and saved.get("station_hosts") == selection["station_hosts"]
                                         and saved.get("profile_config_path") == selected_path
                                         and isinstance(saved.get("station_sensors"), dict))
                if selection["prepared"]:
                    selection["station_sensors"] = saved["station_sensors"]
        except (OSError, ValueError, AttributeError):
            pass
    return selection


def save_ui_selection(config: dict[str, Any], selection: dict[str, Any]) -> None:
    path = ui_state_path(config)
    staged = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    saved = dict(selection, profile_config_path=config["radar_config_library"][selection["radar_profile"]]["config_path"])
    try:
        write_json(staged, saved)
        os.chmod(staged, 0o600)
        os.replace(staged, path)
    finally:
        if staged.exists():
            staged.unlink()


def ui_config_for_station(profile: dict[str, Any], station: dict[str, Any]) -> str:
    station_id = station["station_id"]
    explicit = profile.get("station_paths") or {}
    if station_id in explicit:
        return str(explicit[station_id])
    if station_id == "radar0":
        return str(profile["config_path"])
    return str(PurePosixPath(station["remote_root"]) / "configFiles" / PurePosixPath(profile["config_path"]).name)


def build_ui_capture_args(base_args: argparse.Namespace, config: dict[str, Any], selection: dict[str, Any],
                          name: str, seconds: float, prepared: bool) -> tuple[argparse.Namespace, list[dict[str, Any]]]:
    mode = config["ui_modes"][selection["mode"]]
    profile = config["radar_config_library"][selection["radar_profile"]]
    stations = select_stations(config, mode["stations"])
    if {station["station_id"] for station in stations} != set(mode["stations"]):
        raise RuntimeError("当前模式所需的雷达站未全部启用")
    request = clone_args(base_args, command="go", name=name, mode=mode["operator_profile"],
                         station=list(mode["stations"]), seconds=seconds, countdown=None, pull=False)
    resolved = build_go_capture_args(request, config)
    overrides = dict(resolved.station_overrides)
    for station in stations:
        station_override = dict(overrides.get(station["station_id"], {}))
        station_override["radar_config"] = ui_config_for_station(profile, station)
        station_override.pop("reader_mode", None)
        if profile.get("reader_mode"):
            station_override["reader_mode"] = profile["reader_mode"]
        overrides[station["station_id"]] = station_override
    resolved = clone_args(resolved, station_overrides=overrides, radar_config=profile["config_path"],
                          hardware_trigger=bool(profile.get("hardware_trigger")),
                          stm32_port=str(config.get("stm32_trigger", {}).get("port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")),
                          ui_require_armed=True, ui_scheduled_start=True, ui_prechecked=prepared,
                          ui_station_sensors=selection.get("station_sensors") or {}, skip_preflight=prepared)
    return resolved, stations


def ui_choose(title: str, items: dict[str, dict[str, Any]], current: str) -> str | None:
    while True:
        print(f"\n{title}")
        names = list(items)
        for index, name in enumerate(names, 1):
            item = items[name]
            selected = " [当前]" if name == current else ""
            print(f"  {index}. {item['label']}{selected}")
        print("  0. 返回")
        choice = input("请选择编号: ").strip()
        if choice == "0":
            return None
        if choice.isdecimal() and 1 <= int(choice) <= len(names):
            return names[int(choice) - 1]
        print("无效编号，请重新选择")


def run_ui(args: argparse.Namespace, config: dict[str, Any]) -> tuple[int, dict[str, Any]]:
    base_args = build_ui_base_args(args)
    selection = load_ui_selection(config)
    while True:
        mode = config["ui_modes"][selection["mode"]]
        profile = config["radar_config_library"][selection["radar_profile"]]
        hosts = ui_station_hosts(config)
        host_label = " / ".join(f"{name.upper()}: {host}" for name, host in hosts.items())
        state_label = "已预配置，采集时复核" if selection["prepared"] else "待检查和预配置"
        print("\n=== 雷达联采控制台 ===")
        print("1. 设置雷达侧 IP " + ui_style(f"（当前 IP：{host_label}）", "red", sys.stdout))
        print("2. 查看/选择雷达配置 " + ui_style(f"（当前配置：{profile['label']}）", "red", sys.stdout))
        print("3. 查看/设置采集模式 " + ui_style(f"（当前模式：{mode['label']}）", "red", sys.stdout))
        print("4. 同步代码 " + ui_style("（多雷达的时候才需要同步代码）", "orange", sys.stdout))
        print("5. 采集前检查及预配置 " + ui_style(f"（当前状态：{state_label}）", "red", sys.stdout))
        print("6. 开始采集")
        print("q. 退出")
        choice = input("请选择: ").strip().lower()

        if choice in {"q", "quit", "exit"}:
            return 0, {"status": "ok"}
        if choice == "1":
            current = {item["station_id"]: item for item in config["stations"]}
            radar0 = input(f"NX1 IP [{current.get('radar0', {}).get('host', '未设置')}，回车保持]: ").strip()
            radar0_user = input(f"NX1 用户名 [{current.get('radar0', {}).get('user', config.get('default_user', 'radar'))}，回车保持]: ").strip()
            radar0_password = getpass.getpass("NX1 密码 [回车保持原密码]: ")
            radar1 = input(f"NX2 IP [{current.get('radar1', {}).get('host', '未设置')}，回车保持]: ").strip()
            radar1_user = input(f"NX2 用户名 [{current.get('radar1', {}).get('user', config.get('default_user', 'radar'))}，回车保持]: ").strip()
            radar1_password = getpass.getpass("NX2 密码 [回车保持原密码]: ")
            if not any((radar0, radar1, radar0_user, radar1_user, radar0_password, radar1_password)):
                print("连接信息未更改")
                continue
            change_args = clone_args(base_args, command="set-ips", radar0=radar0, radar1=radar1,
                                     radar0_user=radar0_user, radar1_user=radar1_user,
                                     radar0_password=radar0_password, radar1_password=radar1_password,
                                     enable_radar1=bool(radar1), disable_radar1=False, disable_radar0=False, check=True)
            path = Path(config["config_path"])
            _code, result = update_station_ip_config(path, load_config(path), change_args)
            config = normalize_config(load_config(path), path)
            selection["station_hosts"] = ui_station_hosts(config)
            selection["prepared"] = False
            selection.pop("station_sensors", None)
            save_ui_selection(config, selection)
            for item in result.get("reachability") or []:
                color = "green" if item.get("ssh_port_open") else "red"
                status = "SSH 可连接" if item.get("ssh_port_open") else "SSH 不可连接"
                print(ui_style(f"  {item['station_id']}: {status}", color, sys.stdout))
            for station_id, password in (("radar0", radar0_password), ("radar1", radar1_password)):
                if password:
                    print(ui_style(f"  {station_id}: 密码已保存（权限 600）", "green", sys.stdout))
            continue
        if choice == "2":
            selected = ui_choose("雷达配置", config["radar_config_library"], selection["radar_profile"])
            if selected is not None:
                if selected != selection["radar_profile"]:
                    selection["radar_profile"] = selected
                    selection["prepared"] = False
                    selection.pop("station_sensors", None)
                    save_ui_selection(config, selection)
                    print(ui_style("配置已选择，请执行选项 5", "orange", sys.stdout))
                selected_profile = config["radar_config_library"][selection["radar_profile"]]
                print(f"  NX1 文件：{selected_profile['config_path']}")
                if "radar1" in mode["stations"]:
                    nx2 = next(station for station in config["stations"] if station["station_id"] == "radar1")
                    print(f"  NX2 文件：{ui_config_for_station(selected_profile, nx2)}")
            continue
        if choice == "3":
            selected = ui_choose("采集模式", config["ui_modes"], selection["mode"])
            if selected is not None and selected != selection["mode"]:
                selection["mode"] = selected
                selection["prepared"] = False
                selection.pop("station_sensors", None)
                save_ui_selection(config, selection)
                print(ui_style("模式已选择，请执行选项 5", "orange", sys.stdout))
            continue
        if choice == "4":
            code, result = run_sync_code(base_args, config, select_stations(config, []))
            if code == 0:
                selection["prepared"] = False
                selection.pop("station_sensors", None)
                save_ui_selection(config, selection)
                print(ui_style(f"同步完成：{result.get('version')}；NX2 服务：{result.get('target', {}).get('restart', '未知')}", "green", sys.stdout))
            else:
                print(ui_style(str(result.get("message") or "同步代码失败"), "red", sys.stdout))
            continue
        if choice == "5":
            try:
                profile_default = float(config["operator_profiles"][mode["operator_profile"]]["duration_seconds"])
                prepared_args, stations = build_ui_capture_args(base_args, config, selection, "ui_prepare", profile_default, False)
                preparation = run_ui_prepare(prepared_args, config, stations)
                selection["prepared"] = bool(preparation["prepared"])
                selection["station_sensors"] = preparation["station_sensors"]
            except Exception as exc:
                selection["prepared"] = False
                selection.pop("station_sensors", None)
                print(ui_style(f"准备失败：{exc}", "red", sys.stdout))
            save_ui_selection(config, selection)
            continue
        if choice == "6":
            if not selection["prepared"]:
                print(ui_style("请先执行选项 5，完成当前配置和模式的检查及预配置", "red", sys.stdout))
                continue
            name = input("场景名称 [回车自动生成]: ").strip() or make_auto_name("scene")
            default_seconds = float(config["operator_profiles"][mode["operator_profile"]]["duration_seconds"])
            seconds_text = input(f"采集时长（秒）[回车 {default_seconds:g}]: ").strip()
            try:
                seconds = parse_ui_number(seconds_text, "采集时长", default_seconds)
                capture_args, stations = build_ui_capture_args(base_args, config, selection, name, seconds, True)
            except (ValueError, RuntimeError) as exc:
                print(ui_style(f"无法开始采集：{exc}", "red", sys.stdout))
                continue
            result = run_ui_capture(capture_args, config, stations)
            if result.get("status") != "ok":
                selection["prepared"] = False
                selection.pop("station_sensors", None)
                save_ui_selection(config, selection)
            continue
        print("无效选项，请输入 1-6 或 q")




def main() -> int:
    args = parse_args()
    raw_config = load_config(args.config)

    if args.command == "set-ips":
        exit_code, payload = update_station_ip_config(args.config, raw_config, args)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    config = normalize_config(raw_config, args.config)
    field_override_defaults = {
        "radar_config": config.get("default_radar_config", DEFAULT_RADAR_CONFIG),
        "duration_seconds": None,
        "countdown_seconds": None,
        "camera_device": None,
        "camera_width": None,
        "camera_height": None,
        "camera_fps": None,
        "camera_fourcc": None,
        "camera_format": None,
        "camera_save_every_n": None,
        "imu_port": None,
        "imu_baud": None,
        "station_overrides": {},
    }

    if args.command == "sync-code":
        stations = select_stations(config, args.station)
        exit_code, payload = run_sync_code(args, config, stations)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "show-config":
        stations = select_stations(config, args.station)
        payload = dict(config)
        payload["stations"] = stations
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0

    if args.command in {"profiles", "modes"}:
        payload = {
            "status": "ok",
            "profiles": summarize_operator_profiles(config),
        }
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0

    if args.command == "field-preflight":
        field_args = clone_args(
            args,
            **field_override_defaults,
            command="preflight",
            allow_no_camera=not args.strict_camera,
            allow_no_imu=not args.strict_imu,
        )
        stations = select_stations(config, args.station)
        exit_code, payload = run_preflight(field_args, config, stations)
        print(json.dumps(summarize_preflight_for_terminal(payload), indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "preflight":
        stations = select_stations(config, args.station)
        exit_code, payload = run_preflight(args, config, stations)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "field-capture":
        field_args = clone_args(
            args,
            **field_override_defaults,
            command="capture",
            name=(args.name.strip() or make_auto_name("scene")),
            capture_id=(args.capture_id.strip() or make_auto_capture_id("exp")),
            duration_seconds=float(args.seconds),
            countdown_seconds=float(args.countdown),
            allow_no_camera=not args.strict_camera,
            allow_no_imu=not args.strict_imu,
            no_pull=not args.pull,
            result_mode=("pull-full" if args.pull else "station-local-only"),
            operator_entrypoint="field-capture",
        )
        stations = select_stations(config, args.station)
        exit_code, payload = run_capture(field_args, config, stations)
        print(json.dumps(summarize_payload_for_terminal(payload), indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "go":
        go_args = build_go_capture_args(args, config)
        stations = select_stations(config, go_args.station)
        exit_code, payload = run_capture(go_args, config, stations)
        print(json.dumps(summarize_payload_for_terminal(payload), indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "capture":
        stations = select_stations(config, args.station)
        exit_code, payload = run_capture(args, config, stations)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "pull-results":
        stations = select_stations(config, args.station)
        exit_code, payload = run_pull_results(args, config, stations)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "ingest":
        exit_code, payload = run_ingest(args, config)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "sync-report":
        exit_code, payload = run_sync_report(args, config)
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return exit_code

    if args.command == "ui":
        exit_code, _payload = run_ui(args, config)
        return exit_code

    raise RuntimeError(f"Unsupported command: {args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        if "ui" in sys.argv[1:]:
            try:
                log_path = new_ui_log_path("startup_error")
                with log_path.open("x", encoding="utf-8") as log:
                    os.chmod(log_path, 0o600)
                    traceback.print_exc(file=log)
                print(f"控制台启动失败，请查看详细日志：{log_path}", file=sys.stderr)
            except Exception:
                print("控制台启动失败，请检查配置文件和日志目录", file=sys.stderr)
        else:
            print(json.dumps({"status": "error", "message": str(exc)}, ensure_ascii=False), file=sys.stderr)
        raise SystemExit(2)
