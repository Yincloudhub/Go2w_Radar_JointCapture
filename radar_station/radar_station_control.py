from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from capture_guided_session import (
    DCA1000,
    ROOT,
    CameraProbeResult,
    current_lsusb,
    estimate_num_frames,
    list_video_nodes,
    probe_camera,
    radar_usb_present,
)
from capture_session_awr2243 import SessionCaptureError, detect_imu_port, prepare_radar_interface, restore_session_ownership
from capture_session_awr2243 import available_serial_ports, host_time_snapshot


DEFAULT_CONFIG_PATH = ROOT / "radar_station_config.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Standardized control entrypoint for one radar-side NX station."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show-config", help="Print the effective station configuration as JSON.")
    arm_parser = subparsers.add_parser("arm", help="Initialize radar once and keep it ready for finite captures.")
    arm_parser.add_argument("--radar-config", default=None)
    subparsers.add_parser("ready-status", help="Show the persistent radar's ready state.")
    subparsers.add_parser("disarm", help="Power off the persistent radar service.")

    preflight_parser = subparsers.add_parser("preflight", help="Run preflight checks only.")
    preflight_parser.add_argument("--skip-network-setup", action="store_true", default=None)
    preflight_parser.add_argument("--allow-no-camera", action="store_true")
    preflight_parser.add_argument("--allow-no-imu", action="store_true")
    add_override_args(preflight_parser)

    camera_preflight_parser = subparsers.add_parser(
        "camera-preflight",
        help="Run a camera-only preflight that skips radar and IMU requirements.",
    )
    add_camera_override_args(camera_preflight_parser)

    capture_parser = subparsers.add_parser("capture", help="Run one fixed-duration capture session.")
    capture_parser.add_argument("--name", required=True, help="Scene or experiment label.")
    capture_parser.add_argument("--capture-id", default="", help="Shared identifier for one multi-device experiment.")
    capture_parser.add_argument("--duration-seconds", type=float, default=None)
    capture_parser.add_argument("--countdown-seconds", type=float, default=None)
    capture_parser.add_argument("--start-at-wall-ns", type=int, default=None)
    capture_parser.add_argument("--skip-network-setup", action="store_true", default=None)
    capture_parser.add_argument("--allow-no-camera", action="store_true")
    capture_parser.add_argument("--allow-no-imu", action="store_true")
    add_override_args(capture_parser)

    camera_capture_parser = subparsers.add_parser(
        "camera-capture",
        help="Run one fixed-duration camera-only session with the same session folder layout.",
    )
    camera_capture_parser.add_argument("--name", required=True, help="Scene or experiment label.")
    camera_capture_parser.add_argument("--capture-id", default="", help="Optional shared identifier.")
    camera_capture_parser.add_argument("--duration-seconds", type=float, default=None)
    camera_capture_parser.add_argument("--countdown-seconds", type=float, default=None)
    add_camera_override_args(camera_capture_parser)

    return parser.parse_args()


def add_override_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--radar-config", default=None)
    parser.add_argument("--camera-device", default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=int, default=None)
    parser.add_argument("--camera-fourcc", default=None)
    parser.add_argument("--camera-format", default=None)
    parser.add_argument("--camera-save-every-n", type=int, default=None)
    parser.add_argument("--imu-port", default=None)
    parser.add_argument("--imu-baud", type=int, default=None)


def add_camera_override_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--camera-device", default=None)
    parser.add_argument("--camera-width", type=int, default=None)
    parser.add_argument("--camera-height", type=int, default=None)
    parser.add_argument("--camera-fps", type=int, default=None)
    parser.add_argument("--camera-fourcc", default=None)
    parser.add_argument("--camera-format", default=None)
    parser.add_argument("--camera-save-every-n", type=int, default=None)


def load_station_config(config_path: Path) -> dict[str, Any]:
    config_path = config_path.expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as config_file:
        config = json.load(config_file)
    if not isinstance(config, dict):
        raise SessionCaptureError(f"Station config must be a JSON object: {config_path}")
    for key in ("output_root", "radar_config", "radar_config_base", "dca_config"):
        if config.get(key):
            path = Path(str(config[key])).expanduser()
            config[key] = str((config_path.parent / path).resolve() if not path.is_absolute() else path)
    return config


def sanitize_label(label: str) -> str:
    cleaned = "".join(ch.lower() if ch.isalnum() else "_" for ch in label.strip())
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.strip("_") or "session"


def require_root() -> None:
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() != 0:
        raise SessionCaptureError("Use sudo/root to run the station control entrypoint on Linux.")


def build_effective_capture_settings(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    return {
        "station_id": str(config.get("station_id", "radar")),
        "output_root": str(config.get("output_root", ROOT / "data" / "sessions")),
        "radar_config": (
            str(args.radar_config)
            if getattr(args, "radar_config", None) not in (None, "")
            else str(config.get("radar_config", ROOT / "configFiles" / "AWR2243_mmwaveconfig.txt"))
        ),
        "dca_config": str(config.get("dca_config", ROOT / "configFiles" / "cf.json")),
        "dca_static_ip": str(config.get("dca_static_ip", "192.168.33.30")),
        "dca_adc_ip": str(config.get("dca_adc_ip", "192.168.33.180")),
        "radar_interface": str(config.get("radar_interface", "eno1")),
        "radar_interface_cidr": str(config.get("radar_interface_cidr", "192.168.33.30/24")),
        "imu_port": (
            str(args.imu_port)
            if getattr(args, "imu_port", None) not in (None, "")
            else str(config.get("imu_port", "auto"))
        ),
        "imu_baud": (
            int(args.imu_baud)
            if getattr(args, "imu_baud", None) is not None
            else int(config.get("imu_baud", 0))
        ),
        "imu_probe_seconds": float(config.get("imu_probe_seconds", 0.8)),
        "imu_warmup_seconds": float(config.get("imu_warmup_seconds", 1.0)),
        "allow_no_imu": bool(getattr(args, "allow_no_imu", False) or config.get("allow_no_imu", False)),
        "camera_enabled": bool(config.get("camera_enabled", True)),
        "camera_device": (
            str(args.camera_device)
            if getattr(args, "camera_device", None) not in (None, "")
            else str(config.get("camera_device", "auto"))
        ),
        "camera_width": (
            int(args.camera_width)
            if getattr(args, "camera_width", None) is not None
            else int(config.get("camera_width", 640))
        ),
        "camera_height": (
            int(args.camera_height)
            if getattr(args, "camera_height", None) is not None
            else int(config.get("camera_height", 480))
        ),
        "camera_fps": (
            int(args.camera_fps)
            if getattr(args, "camera_fps", None) is not None
            else int(config.get("camera_fps", 15))
        ),
        "camera_fourcc": (
            str(args.camera_fourcc)
            if getattr(args, "camera_fourcc", None) not in (None, "")
            else str(config.get("camera_fourcc", "MJPG"))
        ),
        "camera_format": (
            str(args.camera_format)
            if getattr(args, "camera_format", None) not in (None, "")
            else str(config.get("camera_format", "video"))
        ),
        "camera_warmup_seconds": float(config.get("camera_warmup_seconds", 1.5)),
        "camera_save_every_n": (
            int(args.camera_save_every_n)
            if getattr(args, "camera_save_every_n", None) is not None
            else int(config.get("camera_save_every_n", 2))
        ),
        "allow_no_camera": bool(getattr(args, "allow_no_camera", False) or config.get("allow_no_camera", False)),
        "duration_seconds": (
            float(args.duration_seconds)
            if getattr(args, "duration_seconds", None) is not None
            else float(config.get("default_duration_seconds", 1.0))
        ),
        "countdown_seconds": (
            float(args.countdown_seconds)
            if getattr(args, "countdown_seconds", None) is not None
            else float(config.get("default_countdown_seconds", 1.0))
        ),
        "start_at_wall_ns": getattr(args, "start_at_wall_ns", None),
        "skip_network_setup": bool(
            getattr(args, "skip_network_setup", None)
            if getattr(args, "skip_network_setup", None) is not None
            else config.get("skip_network_setup", False)
        ),
    }


def run_preflight(settings: dict[str, Any], config_path: Path) -> dict[str, Any]:
    host_time = host_time_snapshot()
    lsusb_lines = current_lsusb()
    video_nodes = list_video_nodes()
    serial_ports = available_serial_ports()
    radar_present = radar_usb_present(lsusb_lines)
    network_summary = None
    if not settings["skip_network_setup"]:
        network_summary = prepare_radar_interface(settings["radar_interface"], settings["radar_interface_cidr"])
    else:
        addr = subprocess.run(
            ["ip", "-brief", "addr", "show", "dev", settings["radar_interface"]],
            capture_output=True,
            text=True,
            check=False,
        )
        network_summary = {
            "interface": settings["radar_interface"],
            "configured_cidr": settings["radar_interface_cidr"],
            "addr_state": addr.stdout.strip(),
            "setup_skipped": True,
        }

    imu_probe = detect_imu_port(settings["imu_port"], settings["imu_baud"], settings["imu_probe_seconds"])
    camera_probe: CameraProbeResult | None = None
    preflight_errors: list[str] = []

    if not radar_present:
        preflight_errors.append("Radar FTDI device (0451:fd03) not found in lsusb.")

    if network_summary is not None:
        addr_state = str(network_summary.get("addr_state", ""))
        if " UP " not in f" {addr_state} ":
            preflight_errors.append(
                f"Radar interface {settings['radar_interface']} is not link-up yet: {addr_state}"
            )

    dca_probe: dict[str, Any] = {"status": "error"}
    try:
        dca = DCA1000(static_ip=settings["dca_static_ip"], adc_ip=settings["dca_adc_ip"])
        try:
            reply = dca.sys_alive_check()
        finally:
            dca.close()
        dca_probe = {"status": "ok" if reply == "success" else "error", "reply": reply}
    except Exception as exc:
        dca_probe = {"status": "error", "message": str(exc)}
    if dca_probe["status"] != "ok":
        preflight_errors.append("DCA1000 did not answer on the radar Ethernet link; check its power and cable.")

    if imu_probe is None and not settings["allow_no_imu"] and settings["imu_port"].lower() != "none":
        preflight_errors.append("No decodable IMU stream was detected.")

    if settings["camera_enabled"]:
        camera_probe = probe_camera(
            camera_device=settings["camera_device"],
            width=settings["camera_width"],
            height=settings["camera_height"],
            fps=settings["camera_fps"],
            fourcc=settings["camera_fourcc"],
            warmup_seconds=settings["camera_warmup_seconds"],
        )
        if not camera_probe.ok and not settings["allow_no_camera"]:
            preflight_errors.append(camera_probe.error or "Camera probe failed.")

    warnings = []
    if host_time.get("warning"):
        warnings.append(str(host_time["warning"]))

    num_frames, adc_params, cfg_params, frame_period_ns = estimate_num_frames(
        Path(settings["radar_config"]),
        settings["duration_seconds"],
    )

    return {
        "status": "ok" if not preflight_errors else "preflight_failed",
        "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "station": {
            "station_id": settings["station_id"],
            "host_role": "radar_side_nx",
            "hostname": socket.gethostname(),
            "user": os.environ.get("SUDO_USER") or os.environ.get("USER") or "",
            "config_path": str(config_path),
        },
        "host_time": host_time,
        "effective_settings": settings,
        "lsusb": lsusb_lines,
        "video_nodes": video_nodes,
        "serial_ports": serial_ports,
        "radar_usb_present": radar_present,
        "network": network_summary,
        "dca": dca_probe,
        "imu": (
            None
            if imu_probe is None
            else {"port": imu_probe.port, "baudrate": imu_probe.baudrate, "probe_hits": imu_probe.hits}
        ),
        "camera": (
            None
            if camera_probe is None
            else {
                "ok": camera_probe.ok,
                "device": camera_probe.device,
                "resolved_device": camera_probe.resolved_device,
                "backend": camera_probe.backend,
                "width": camera_probe.width,
                "height": camera_probe.height,
                "actual_fps": camera_probe.actual_fps,
                "actual_fourcc": camera_probe.actual_fourcc,
                "error": camera_probe.error,
                "attempts": camera_probe.attempts,
                "candidate_devices": camera_probe.candidate_devices,
            }
        ),
        "warnings": warnings,
        "frame_budget": {
            "duration_seconds_requested": settings["duration_seconds"],
            "num_frames": num_frames,
            "frame_period_ns": frame_period_ns,
            "expected_duration_seconds": num_frames * frame_period_ns / 1_000_000_000,
            "adc_params": adc_params,
            "cfg_params": cfg_params,
        },
        "errors": preflight_errors,
    }


def run_camera_preflight(settings: dict[str, Any], config_path: Path) -> dict[str, Any]:
    host_time = host_time_snapshot()
    camera_probe: CameraProbeResult | None = None
    preflight_errors: list[str] = []
    if not settings["camera_enabled"]:
        preflight_errors.append("Camera is disabled in the station config.")
    else:
        camera_probe = probe_camera(
            camera_device=settings["camera_device"],
            width=settings["camera_width"],
            height=settings["camera_height"],
            fps=settings["camera_fps"],
            fourcc=settings["camera_fourcc"],
            warmup_seconds=settings["camera_warmup_seconds"],
        )
        if not camera_probe.ok:
            preflight_errors.append(camera_probe.error or "Camera probe failed.")

    warnings = []
    if host_time.get("warning"):
        warnings.append(str(host_time["warning"]))

    return {
        "status": "ok" if not preflight_errors else "preflight_failed",
        "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "station": {
            "station_id": settings["station_id"],
            "host_role": "radar_side_nx",
            "hostname": socket.gethostname(),
            "user": os.environ.get("SUDO_USER") or os.environ.get("USER") or "",
            "config_path": str(config_path),
        },
        "host_time": host_time,
        "effective_settings": {
            "station_id": settings["station_id"],
            "output_root": settings["output_root"],
            "duration_seconds": settings["duration_seconds"],
            "countdown_seconds": settings["countdown_seconds"],
            "camera_enabled": settings["camera_enabled"],
            "camera_device": settings["camera_device"],
            "camera_width": settings["camera_width"],
            "camera_height": settings["camera_height"],
            "camera_fps": settings["camera_fps"],
            "camera_fourcc": settings["camera_fourcc"],
            "camera_format": settings["camera_format"],
            "camera_warmup_seconds": settings["camera_warmup_seconds"],
            "camera_save_every_n": settings["camera_save_every_n"],
        },
        "video_nodes": list_video_nodes(),
        "camera": (
            None
            if camera_probe is None
            else {
                "ok": camera_probe.ok,
                "device": camera_probe.device,
                "resolved_device": camera_probe.resolved_device,
                "backend": camera_probe.backend,
                "width": camera_probe.width,
                "height": camera_probe.height,
                "actual_fps": camera_probe.actual_fps,
                "actual_fourcc": camera_probe.actual_fourcc,
                "error": camera_probe.error,
                "attempts": camera_probe.attempts,
                "candidate_devices": camera_probe.candidate_devices,
            }
        ),
        "warnings": warnings,
        "errors": preflight_errors,
    }


def build_capture_command(settings: dict[str, Any], session_dir: Path, name: str, capture_id: str) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "capture_guided_session.py"),
        "--name",
        name,
        "--station-id",
        settings["station_id"],
        "--capture-id",
        capture_id,
        "--output-root",
        settings["output_root"],
        "--session-dir",
        str(session_dir),
        "--duration-seconds",
        str(settings["duration_seconds"]),
        "--countdown-seconds",
        str(settings["countdown_seconds"]),
        "--radar-config",
        settings["radar_config"],
        "--dca-config",
        settings["dca_config"],
        "--dca-static-ip",
        settings["dca_static_ip"],
        "--dca-adc-ip",
        settings["dca_adc_ip"],
        "--radar-interface",
        settings["radar_interface"],
        "--radar-interface-cidr",
        settings["radar_interface_cidr"],
        "--imu-port",
        settings["imu_port"],
        "--imu-baud",
        str(settings["imu_baud"]),
        "--imu-probe-seconds",
        str(settings["imu_probe_seconds"]),
        "--imu-warmup-seconds",
        str(settings["imu_warmup_seconds"]),
        "--camera-device",
        settings["camera_device"],
        "--camera-width",
        str(settings["camera_width"]),
        "--camera-height",
        str(settings["camera_height"]),
        "--camera-fps",
        str(settings["camera_fps"]),
        "--camera-fourcc",
        settings["camera_fourcc"],
        "--camera-format",
        settings["camera_format"],
        "--camera-warmup-seconds",
        str(settings["camera_warmup_seconds"]),
        "--camera-save-every-n",
        str(settings["camera_save_every_n"]),
    ]
    if settings["skip_network_setup"]:
        command.append("--skip-network-setup")
    if settings["start_at_wall_ns"] is not None:
        command.extend(["--start-at-wall-ns", str(settings["start_at_wall_ns"])])
    if settings["allow_no_imu"]:
        command.append("--allow-no-imu")
    if settings["allow_no_camera"]:
        command.append("--allow-no-camera")
    if not settings["camera_enabled"]:
        command.append("--disable-camera")
    return command


def build_camera_capture_command(
    settings: dict[str, Any],
    session_dir: Path,
    name: str,
    capture_id: str,
) -> list[str]:
    return [
        sys.executable,
        str(ROOT / "capture_camera_guided_session.py"),
        "--name",
        name,
        "--station-id",
        settings["station_id"],
        "--capture-id",
        capture_id,
        "--output-root",
        settings["output_root"],
        "--session-dir",
        str(session_dir),
        "--duration-seconds",
        str(settings["duration_seconds"]),
        "--countdown-seconds",
        str(settings["countdown_seconds"]),
        "--camera-device",
        settings["camera_device"],
        "--camera-width",
        str(settings["camera_width"]),
        "--camera-height",
        str(settings["camera_height"]),
        "--camera-fps",
        str(settings["camera_fps"]),
        "--camera-fourcc",
        settings["camera_fourcc"],
        "--camera-format",
        settings["camera_format"],
        "--camera-warmup-seconds",
        str(settings["camera_warmup_seconds"]),
        "--camera-save-every-n",
        str(settings["camera_save_every_n"]),
    ]


def make_session_dir(output_root: Path, station_id: str, label: str) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base_name = f"{time.strftime('%Y%m%d_%H%M%S')}__{sanitize_label(station_id)}__{sanitize_label(label)}"
    session_dir = output_root / base_name
    suffix = 1
    while session_dir.exists():
        session_dir = output_root / f"{base_name}_{suffix:02d}"
        suffix += 1
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def concise_capture_status_line(line: str) -> str | None:
    stripped = line.strip()
    if not stripped:
        return None
    for marker in (
        "RADAR_ARMED_WAITING_FOR_TRIGGER ",
        "RADAR_RECORDING_STARTED ",
        "RADAR_RECORDING_STOPPED ",
        "RADAR_RAW_READY ",
        "RADAR_WARNING ",
        "RADAR_STATUS raw_quality_before_save ",
        "RADAR_STATUS recording_complete_before_save ",
        "CAPTURE_STATUS ",
    ):
        if marker in stripped:
            return stripped[stripped.index(marker) :]
    if "Camera ready before radar start:" in stripped:
        return "CAMERA_RECORDING_STARTED " + stripped.split("Camera ready before radar start:", 1)[1].strip()
    if "SYNC CHECK after warm-up: READY" in stripped:
        return "SYNC_CHECK_READY software_sync_only=true use_metadata_timestamps=true"
    if "SYNC CHECK after warm-up: NOT READY" in stripped:
        return "SYNC_CHECK_NOT_READY camera_first_frame_timestamp_missing=true"
    if "Recording starts now for scene" in stripped:
        return "CAPTURE_LAUNCH_REQUESTED " + stripped.split("Recording starts now for scene", 1)[1].strip()
    if "Capture complete for" in stripped:
        return "CAPTURE_RECORDING_COMPLETE " + stripped.split("Capture complete for", 1)[1].strip()
    if stripped.startswith("Session complete:"):
        return "CAPTURE_SESSION_COMPLETE " + stripped.split("Session complete:", 1)[1].strip()
    if stripped.startswith("Saved to:"):
        return "CAPTURE_SAVED session_dir=" + stripped.split("Saved to:", 1)[1].strip()
    return None


def run_capture(settings: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    session_dir = make_session_dir(Path(settings["output_root"]), settings["station_id"], args.name)
    command = build_capture_command(settings, session_dir, args.name, args.capture_id)
    started_at = time.strftime("%Y-%m-%d %H:%M:%S")
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    stdout_lines: list[str] = []
    assert process.stdout is not None
    for line in process.stdout:
        stdout_lines.append(line.rstrip("\n"))
        status_line = concise_capture_status_line(line)
        if status_line is not None:
            print(status_line, file=sys.stderr, flush=True)
    returncode = process.wait()

    manifest_path = session_dir / "session_manifest.json"
    manifest = None
    if manifest_path.exists():
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            manifest = json.load(manifest_file)

    summary = {
        "status": "ok" if returncode == 0 else "capture_failed",
        "started_at_local": started_at,
        "finished_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "station_id": settings["station_id"],
        "capture_id": args.capture_id,
        "requested_name": args.name,
        "session_dir": str(session_dir),
        "returncode": returncode,
        "stdout_tail": stdout_lines[-20:],
        "stderr_tail": [],
        "manifest_path": str(manifest_path) if manifest_path.exists() else "",
        "manifest": manifest,
    }
    restore_session_ownership(session_dir)
    return summary


def run_camera_capture(settings: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    session_dir = make_session_dir(Path(settings["output_root"]), settings["station_id"], args.name)
    command = build_camera_capture_command(settings, session_dir, args.name, args.capture_id)
    started_at = time.strftime("%Y-%m-%d %H:%M:%S")
    completed = subprocess.run(command, capture_output=True, text=True, check=False)

    manifest_path = session_dir / "session_manifest.json"
    manifest = None
    if manifest_path.exists():
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            manifest = json.load(manifest_file)

    summary = {
        "status": "ok" if completed.returncode == 0 else "capture_failed",
        "started_at_local": started_at,
        "finished_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "station_id": settings["station_id"],
        "capture_id": args.capture_id,
        "requested_name": args.name,
        "session_dir": str(session_dir),
        "returncode": completed.returncode,
        "stdout_tail": completed.stdout.splitlines()[-10:],
        "stderr_tail": completed.stderr.splitlines()[-10:],
        "manifest_path": str(manifest_path) if manifest_path.exists() else "",
        "manifest": manifest,
    }
    restore_session_ownership(session_dir)
    return summary


def main() -> int:
    args = parse_args()
    require_root()
    config = load_station_config(args.config)
    settings = build_effective_capture_settings(config, args)

    if args.command == "show-config":
        print(json.dumps({"config_path": str(args.config), "effective_settings": settings}, indent=2))
        return 0

    if args.command in {"arm", "ready-status", "disarm"}:
        from radar_ready import DEFAULT_SOCKET, arm, request

        if args.command == "arm":
            if not settings["skip_network_setup"]:
                prepare_radar_interface(settings["radar_interface"], settings["radar_interface_cidr"])
            report = arm(
                DEFAULT_SOCKET,
                Path(settings["radar_config"]),
                Path(settings["dca_config"]),
                settings["dca_static_ip"],
                settings["dca_adc_ip"],
                ROOT / "logs" / "radar-ready.log",
            )
        else:
            report = request(DEFAULT_SOCKET, {"command": "status" if args.command == "ready-status" else "stop"})
        print(json.dumps(report, indent=2))
        return 0

    if args.command == "preflight":
        report = run_preflight(settings, args.config)
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "ok" else 2

    if args.command == "camera-preflight":
        report = run_camera_preflight(settings, args.config)
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "ok" else 2

    if args.command == "capture":
        report = run_capture(settings, args)
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "ok" else 2

    if args.command == "camera-capture":
        report = run_camera_capture(settings, args)
        print(json.dumps(report, indent=2))
        return 0 if report["status"] == "ok" else 2

    raise SessionCaptureError(f"Unsupported command: {args.command}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SessionCaptureError as exc:
        print(json.dumps({"status": "error", "message": str(exc)}), file=sys.stderr)
        raise SystemExit(2)
