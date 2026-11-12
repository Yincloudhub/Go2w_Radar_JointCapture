from __future__ import annotations

import argparse
import json
import os
import posixpath
import shlex
import sys
import time
from pathlib import Path
from stat import S_ISDIR
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
VENDOR_DIR = REPO_ROOT.parent / "_vendor"
if VENDOR_DIR.exists():
    sys.path.insert(0, str(VENDOR_DIR))

import paramiko  # type: ignore


DEFAULT_REMOTE_ROOT = "/home/radar/pyRadar-main"
DEFAULT_SSH_CONFIG = Path.home() / ".ssh" / "config"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Remote helper for one radar-side NX station: preflight, capture, and optional pull."
    )
    parser.add_argument("--host", required=True, help="Radar-side NX IP or hostname.")
    parser.add_argument(
        "--user",
        default=None,
        help="Remote Linux username. Defaults to the SSH config entry or 'radar'.",
    )
    parser.add_argument("--password", default=os.environ.get("RADAR_REMOTE_PASSWORD", ""))
    parser.add_argument(
        "--sudo-password",
        default=os.environ.get("RADAR_REMOTE_SUDO_PASSWORD", ""),
        help="Remote sudo password. Defaults to login password when omitted.",
    )
    parser.add_argument(
        "--key-file",
        default="",
        help="Optional SSH private key path. Defaults to SSH agent / SSH config when omitted.",
    )
    parser.add_argument(
        "--ssh-config",
        default=str(DEFAULT_SSH_CONFIG),
        help="Optional OpenSSH config file used to resolve host aliases, user, port, and IdentityFile.",
    )
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--remote-root", default=DEFAULT_REMOTE_ROOT)
    parser.add_argument(
        "--pull-to",
        type=Path,
        default=None,
        help="Optional local directory used to pull the resulting session directory after capture.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show-config")
    arm = subparsers.add_parser("arm")
    arm.add_argument("--radar-config", default=None)
    subparsers.add_parser("ready-status")
    subparsers.add_parser("disarm")

    preflight = subparsers.add_parser("preflight")
    add_common_override_args(preflight)
    preflight.add_argument("--allow-no-camera", action="store_true")
    preflight.add_argument("--allow-no-imu", action="store_true")
    preflight.add_argument("--skip-network-setup", action="store_true")

    camera_preflight = subparsers.add_parser("camera-preflight")
    add_camera_override_args(camera_preflight)

    capture = subparsers.add_parser("capture")
    capture.add_argument("--name", required=True)
    capture.add_argument("--capture-id", default="")
    capture.add_argument("--start-at-wall-ns", type=int, default=None)
    add_common_override_args(capture)
    capture.add_argument("--allow-no-camera", action="store_true")
    capture.add_argument("--allow-no-imu", action="store_true")
    capture.add_argument("--skip-network-setup", action="store_true")

    camera_capture = subparsers.add_parser("camera-capture")
    camera_capture.add_argument("--name", required=True)
    camera_capture.add_argument("--capture-id", default="")
    add_camera_override_args(camera_capture)
    camera_capture.add_argument("--duration-seconds", type=float, default=None)
    camera_capture.add_argument("--countdown-seconds", type=float, default=None)

    pull = subparsers.add_parser("pull-session")
    pull.add_argument("--session-dir", required=True, help="Remote session directory to download.")
    pull.add_argument("--output-dir", type=Path, required=True, help="Local target directory.")

    return parser.parse_args()


def add_common_override_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--radar-config", default=None)
    parser.add_argument("--reader-mode", default=None)
    parser.add_argument("--duration-seconds", type=float, default=None)
    parser.add_argument("--countdown-seconds", type=float, default=None)
    add_camera_override_args(parser)
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


def create_ssh_client(args: argparse.Namespace) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh_lookup = resolve_ssh_lookup(args.host, args.ssh_config)
    hostname = str(ssh_lookup.get("hostname") or args.host)
    username = str(args.user or ssh_lookup.get("user") or "radar")
    port = int(args.port or ssh_lookup.get("port") or 22)
    connect_kwargs: dict[str, Any] = {
        "hostname": hostname,
        "port": port,
        "username": username,
        "timeout": 15,
        "banner_timeout": 15,
        "auth_timeout": 15,
        "allow_agent": True,
        "look_for_keys": True,
    }
    key_files = resolve_identity_files(args.key_file, ssh_lookup)
    if key_files:
        connect_kwargs["key_filename"] = key_files
    if args.password:
        # Keep password authentication available when an SSH config also lists
        # an IdentityFile. Paramiko may try both; passphrase only unlocks a key.
        connect_kwargs["password"] = args.password
        connect_kwargs["passphrase"] = args.password
    client.connect(**connect_kwargs)
    return client


def resolve_ssh_lookup(host: str, ssh_config_path: str | Path | None) -> dict[str, Any]:
    if ssh_config_path in (None, ""):
        return {}
    config_path = Path(ssh_config_path).expanduser()
    if not config_path.exists():
        return {}
    ssh_config = paramiko.SSHConfig()
    with config_path.open("r", encoding="utf-8") as config_file:
        ssh_config.parse(config_file)
    return ssh_config.lookup(host)


def resolve_identity_files(explicit_key_file: str, ssh_lookup: dict[str, Any]) -> list[str]:
    if explicit_key_file:
        return [str(Path(explicit_key_file).expanduser())]
    identity_files = ssh_lookup.get("identityfile") or []
    resolved: list[str] = []
    for identity_file in identity_files:
        resolved.append(str(Path(identity_file).expanduser()))
    return resolved


def build_remote_command(args: argparse.Namespace) -> str:
    runner = posixpath.join(args.remote_root, "run_radar_station.sh")
    parts = [shlex.quote(runner), args.command]

    if args.command in {"capture", "camera-capture"}:
        parts.extend(["--name", shlex.quote(args.name)])
        if args.capture_id:
            parts.extend(["--capture-id", shlex.quote(args.capture_id)])

    if args.command in {"preflight", "capture"}:
        append_optional(parts, "--radar-config", args.radar_config)
        append_optional(parts, "--duration-seconds", args.duration_seconds)
        append_optional(parts, "--countdown-seconds", args.countdown_seconds)
        if args.command == "capture":
            append_optional(parts, "--start-at-wall-ns", getattr(args, "start_at_wall_ns", None))
        append_optional(parts, "--camera-device", args.camera_device)
        append_optional(parts, "--camera-width", args.camera_width)
        append_optional(parts, "--camera-height", args.camera_height)
        append_optional(parts, "--camera-fps", args.camera_fps)
        append_optional(parts, "--camera-fourcc", args.camera_fourcc)
        append_optional(parts, "--camera-format", args.camera_format)
        append_optional(parts, "--camera-save-every-n", args.camera_save_every_n)
        append_optional(parts, "--imu-port", args.imu_port)
        append_optional(parts, "--imu-baud", args.imu_baud)
        if getattr(args, "allow_no_camera", False):
            parts.append("--allow-no-camera")
        if getattr(args, "allow_no_imu", False):
            parts.append("--allow-no-imu")
        if getattr(args, "skip_network_setup", False):
            parts.append("--skip-network-setup")

    if args.command == "arm":
        append_optional(parts, "--radar-config", args.radar_config)

    if args.command in {"camera-preflight", "camera-capture"}:
        append_optional(parts, "--duration-seconds", args.duration_seconds)
        append_optional(parts, "--countdown-seconds", args.countdown_seconds)
        append_optional(parts, "--camera-device", args.camera_device)
        append_optional(parts, "--camera-width", args.camera_width)
        append_optional(parts, "--camera-height", args.camera_height)
        append_optional(parts, "--camera-fps", args.camera_fps)
        append_optional(parts, "--camera-fourcc", args.camera_fourcc)
        append_optional(parts, "--camera-format", args.camera_format)
        append_optional(parts, "--camera-save-every-n", args.camera_save_every_n)

    env_parts: list[str] = []
    if getattr(args, "reader_mode", None) not in (None, ""):
        env_parts.append("RADAR_READER_MODE=" + shlex.quote(str(args.reader_mode)))

    runner_cmd = " ".join(env_parts + parts)
    sudo_password = args.sudo_password or args.password
    time_sync_cmd = ""
    if sudo_password:
        quoted_password = shlex.quote(sudo_password)
        time_sync_cmd = (
            "if [ -x /usr/local/sbin/nx-online-time-sync ]; then "
            f"printf %s\\\\n {quoted_password} | sudo -S -p '' /usr/local/sbin/nx-online-time-sync remote >/dev/null 2>&1 || true; "
            "fi; "
        )
        return f"{time_sync_cmd}RADAR_STATION_SUDO_PASSWORD={quoted_password} {runner_cmd}"
    time_sync_cmd = (
        "if [ -x /usr/local/sbin/nx-online-time-sync ]; then "
        "sudo -n /usr/local/sbin/nx-online-time-sync remote >/dev/null 2>&1 || true; "
        "fi; "
    )
    return time_sync_cmd + runner_cmd


def append_optional(parts: list[str], flag: str, value: Any) -> None:
    if value is None or value == "":
        return
    parts.extend([flag, shlex.quote(str(value))])


def run_remote_command(client: paramiko.SSHClient, command: str) -> tuple[int, str, str]:
    stdin, stdout, stderr = client.exec_command(command)
    del stdin
    channel = stdout.channel
    stdout_parts: list[str] = []
    stderr_parts: list[str] = []
    stdout_line_buffer = ""
    stderr_line_buffer = ""

    def forward_status_line_buffer(buffer_name: str, chunk: str, final: bool = False) -> None:
        nonlocal stdout_line_buffer, stderr_line_buffer
        if buffer_name == "stdout":
            stdout_line_buffer += chunk
            line_buffer = stdout_line_buffer
        else:
            stderr_line_buffer += chunk
            line_buffer = stderr_line_buffer

        while "\n" in line_buffer:
            line, line_buffer = line_buffer.split("\n", 1)
            if is_realtime_status_line(line):
                print(line, file=sys.stderr, flush=True)
        if final and line_buffer:
            line = line_buffer
            line_buffer = ""
            if is_realtime_status_line(line):
                print(line, file=sys.stderr, flush=True)
        if buffer_name == "stdout":
            stdout_line_buffer = line_buffer
        else:
            stderr_line_buffer = line_buffer

    while not channel.exit_status_ready():
        if channel.recv_ready():
            chunk = channel.recv(65536).decode("utf-8", errors="replace")
            stdout_parts.append(chunk)
            forward_status_line_buffer("stdout", chunk)
        if channel.recv_stderr_ready():
            chunk = channel.recv_stderr(65536).decode("utf-8", errors="replace")
            stderr_parts.append(chunk)
            forward_status_line_buffer("stderr", chunk)
        time.sleep(0.05)

    while channel.recv_ready():
        chunk = channel.recv(65536).decode("utf-8", errors="replace")
        stdout_parts.append(chunk)
        forward_status_line_buffer("stdout", chunk)
    while channel.recv_stderr_ready():
        chunk = channel.recv_stderr(65536).decode("utf-8", errors="replace")
        stderr_parts.append(chunk)
        forward_status_line_buffer("stderr", chunk)

    forward_status_line_buffer("stdout", "", final=True)
    forward_status_line_buffer("stderr", "", final=True)
    return channel.recv_exit_status(), "".join(stdout_parts), "".join(stderr_parts)


def is_realtime_status_line(line: str) -> bool:
    return line.startswith(
        (
            "RADAR_ARMED_WAITING_FOR_TRIGGER ",
            "RADAR_RECORDING_STARTED ",
            "RADAR_RECORDING_STOPPED ",
            "RADAR_RAW_READY ",
            "RADAR_WARNING ",
            "RADAR_STATUS raw_quality_before_save ",
            "RADAR_STATUS recording_complete_before_save ",
            "CAMERA_RECORDING_STARTED ",
            "SYNC_CHECK_READY ",
            "SYNC_CHECK_NOT_READY ",
            "CAPTURE_LAUNCH_REQUESTED ",
            "CAPTURE_STATUS ",
            "CAPTURE_RECORDING_COMPLETE ",
            "CAPTURE_SESSION_COMPLETE ",
            "CAPTURE_SAVED ",
        )
    )


def extract_last_json_blob(text: str) -> Any:
    decoder = json.JSONDecoder()
    last_obj = None
    idx = 0
    while idx < len(text):
        ch = text[idx]
        if ch not in "{[":
            idx += 1
            continue
        try:
            obj, end = decoder.raw_decode(text[idx:])
        except json.JSONDecodeError:
            idx += 1
            continue
        last_obj = obj
        idx += end
    return last_obj


def ensure_local_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def sftp_download_tree(sftp: paramiko.SFTPClient, remote_dir: str, local_dir: Path) -> None:
    ensure_local_dir(local_dir)
    for entry in sftp.listdir_attr(remote_dir):
        remote_path = posixpath.join(remote_dir, entry.filename)
        local_path = local_dir / entry.filename
        if S_ISDIR(entry.st_mode):
            sftp_download_tree(sftp, remote_path, local_path)
        else:
            local_path.parent.mkdir(parents=True, exist_ok=True)
            sftp.get(remote_path, str(local_path))


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


def extract_event_monotonic_ns(events: Any, event_name: str) -> int | None:
    if not isinstance(events, list):
        return None
    for event in events:
        if not isinstance(event, dict):
            continue
        if str(event.get("name", "")) != event_name:
            continue
        return coerce_int(event.get("host_monotonic_ns"))
    return None


def summarize_capture_result(report: dict[str, Any]) -> dict[str, Any]:
    manifest = report.get("manifest") or {}
    metadata = manifest.get("session_metadata") or {}
    camera = manifest.get("camera") or {}
    imu = manifest.get("imu") or {}
    radar = manifest.get("radar") or {}
    radar_quality = radar.get("quality") if isinstance(radar.get("quality"), dict) else {}
    timing = manifest.get("timing") or {}
    radar_events = radar.get("events") or []
    radar_start_ns = timing.get("radar_sensor_start_monotonic_ns")
    radar_stop_ns = timing.get("radar_sensor_stop_monotonic_ns")
    sensor_stop_ns = extract_event_monotonic_ns(radar_events, "sensor_stop_detected")
    raw_written_ns = extract_event_monotonic_ns(radar_events, "radar_raw_written")
    return {
        "status": report.get("status"),
        "station_id": report.get("station_id"),
        "capture_id": report.get("capture_id"),
        "session_dir": report.get("session_dir"),
        "manifest_path": report.get("manifest_path"),
        "requested_name": report.get("requested_name"),
        "duration_seconds_requested": (manifest.get("requested_capture") or {}).get("duration_seconds_requested"),
        "countdown_seconds": (manifest.get("requested_capture") or {}).get("countdown_seconds"),
        "radar_config": radar.get("radar_config"),
        "radar_raw_bytes": radar.get("raw_bytes"),
        "radar_packet_stats": radar.get("packet_stats"),
        "radar_quality_status": radar_quality.get("capture_quality_status"),
        "radar_usable_frames_by_content": radar_quality.get("usable_radar_frames_by_content"),
        "radar_expected_frames": radar_quality.get("expected_frames"),
        "radar_raw_size_matches": radar_quality.get("raw_size_matches"),
        "radar_file_reader_zero_filled_bytes": radar_quality.get("file_reader_zero_filled_bytes"),
        "radar_file_reader_zero_fill_ratio": radar_quality.get("file_reader_zero_fill_ratio"),
        "radar_zero_tail_detected": radar_quality.get("zero_tail_detected"),
        "radar_active_duration_seconds": timing.get("radar_active_duration_seconds"),
        "imu_samples": imu.get("sample_count"),
        "camera_saved_frames": camera.get("saved_frames"),
        "camera_actual_saved_fps": camera.get("actual_saved_fps"),
        "sync_readiness_after_warmup": metadata.get("sync_readiness_after_warmup"),
        "radar_recording_complete_before_save_monotonic_ns": metadata.get(
            "radar_recording_complete_before_save_monotonic_ns"
        ),
        "requested_elapsed_seconds": timing.get("requested_elapsed_seconds"),
        "camera_launch_to_ready_seconds": seconds_between(
            timing.get("camera_launch_monotonic_ns"),
            timing.get("camera_ready_monotonic_ns"),
        ),
        "requested_start_to_sensor_start_seconds": seconds_between(
            timing.get("requested_start_monotonic_ns"),
            timing.get("radar_sensor_start_monotonic_ns"),
        ),
        "sensor_stop_to_raw_written_seconds": seconds_between(sensor_stop_ns, raw_written_ns),
        "imu_first_minus_radar_start_seconds": seconds_between(
            radar_start_ns,
            imu.get("first_sample_monotonic_ns"),
        ),
        "imu_last_minus_radar_stop_seconds": seconds_between(
            radar_stop_ns,
            imu.get("last_sample_monotonic_ns"),
        ),
        "camera_first_saved_minus_radar_start_seconds": seconds_between(
            radar_start_ns,
            camera.get("first_saved_monotonic_ns"),
        ),
        "camera_last_saved_minus_radar_stop_seconds": seconds_between(
            radar_stop_ns,
            camera.get("last_saved_monotonic_ns"),
        ),
        "artifacts": manifest.get("artifacts"),
    }


def main() -> int:
    args = parse_args()
    if args.command == "pull-session":
        client = create_ssh_client(args)
        try:
            with client.open_sftp() as sftp:
                output_dir = args.output_dir.resolve()
                target_dir = output_dir / Path(args.session_dir).name
                sftp_download_tree(sftp, args.session_dir, target_dir)
            print(
                json.dumps(
                    {
                        "status": "ok",
                        "remote_session_dir": args.session_dir,
                        "local_session_dir": str(target_dir),
                    },
                    indent=2,
                )
            )
            return 0
        finally:
            client.close()

    client = create_ssh_client(args)
    try:
        exit_code, stdout_text, stderr_text = run_remote_command(client, build_remote_command(args))
        report = extract_last_json_blob(stdout_text)
        if report is None:
            print(
                json.dumps(
                    {
                        "status": "error",
                        "message": "Remote output did not contain JSON.",
                        "exit_code": exit_code,
                        "stdout": stdout_text,
                        "stderr": stderr_text,
                    },
                    indent=2,
                )
            )
            return 2

        response: dict[str, Any] = {
            "remote_exit_code": exit_code,
            "stderr_tail": stderr_text.splitlines()[-10:],
            "report": report,
        }

        if args.command in {"capture", "camera-capture"}:
            response["summary"] = summarize_capture_result(report)
            if args.pull_to is not None and report.get("session_dir"):
                with client.open_sftp() as sftp:
                    local_root = args.pull_to.resolve()
                    local_target = local_root / Path(str(report["session_dir"])).name
                    sftp_download_tree(sftp, str(report["session_dir"]), local_target)
                response["pulled_to"] = str(local_target)

        print(json.dumps(response, indent=2))
        return 0 if exit_code == 0 else 2
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
