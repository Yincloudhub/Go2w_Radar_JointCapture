from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from capture_camera_session import (
    build_gstreamer_command,
    normalize_device as normalize_camera_device,
    parse_rate_text,
    probe_video_with_ffprobe,
)
from capture_session_awr2243 import (
    DEFAULT_DCA_CONFIG,
    DEFAULT_RADAR_CONFIG,
    DCA1000,
    SessionCaptureError,
    available_serial_ports,
    detect_imu_port,
    host_time_snapshot,
    prepare_radar_interface,
    resolve_session_dir,
    restore_session_ownership,
)
from radar_modes import parse_config as parse_radar_mode_config
from radar_modes import expected_raw_bytes as expected_radar_raw_bytes
from radar_modes import expected_frames as expected_radar_frames
from radar_modes import expected_window_seconds as expected_radar_window_seconds
from radar_modes import validate_summary as validate_radar_mode_summary


ROOT = Path(__file__).resolve().parent
DEFAULT_OUTPUT_ROOT = ROOT / "data" / "sessions"


def required_camera_issues(camera: dict[str, Any], frames_csv: Path) -> list[str]:
    """Reject incomplete required video while retaining all captured artifacts."""
    issues: list[str] = []
    if camera.get("error"):
        issues.append(str(camera["error"]))
    if not camera.get("enabled") or int(camera.get("saved_frames") or 0) < 2:
        issues.append("Required external camera has fewer than two saved frames.")
    if camera.get("covers_radar_window") is not True:
        issues.append("Required external camera does not cover the complete radar window.")
    if camera.get("eos_finalized") is not True:
        issues.append("Required camera video was not successfully finalized.")
    try:
        video_root = frames_csv.parent.resolve()
        video_path = (video_root / camera["video_file"]).resolve()
        video_path.relative_to(video_root)
        if not video_path.is_file() or video_path.stat().st_size == 0:
            issues.append("Required external camera video file is empty or missing.")
        with frames_csv.open(newline="", encoding="utf-8") as handle:
            stamps = [int(row["host_monotonic_ns"]) for row in csv.DictReader(handle)]
        if len(stamps) != int(camera.get("saved_frames") or 0) or len(stamps) < 2:
            issues.append("Camera timestamp CSV does not match the saved frame count.")
        if any(stamp <= 0 for stamp in stamps) or any(b <= a for a, b in zip(stamps, stamps[1:])):
            issues.append("Camera timestamps are invalid or out of order.")
        max_gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=0) / 1e9
        camera["max_receive_gap_seconds"] = max_gap
        if max_gap > 0.5:
            issues.append("Required external camera has a frame gap exceeding 0.5 seconds.")
    except (OSError, KeyError, ValueError, TypeError, csv.Error) as exc:
        issues.append(f"Required camera timestamp validation failed: {exc}")
    return issues


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Preflight-check radar/camera/IMU, wait a short countdown, then capture one fixed-duration "
            "session with structured output folders and logs."
        )
    )
    parser.add_argument("--name", required=True, help="Scene or test label used in the session folder name.")
    parser.add_argument("--station-id", default=os.environ.get("RADAR_STATION_ID", "radar"))
    parser.add_argument("--capture-id", default="")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--duration-seconds", type=float, default=3.0)
    parser.add_argument("--countdown-seconds", type=float, default=1.0)
    parser.add_argument("--start-at-wall-ns", type=int, default=None)
    parser.add_argument("--radar-config", type=Path, default=DEFAULT_RADAR_CONFIG)
    parser.add_argument("--dca-config", type=Path, default=DEFAULT_DCA_CONFIG)
    parser.add_argument("--dca-static-ip", default="192.168.33.30")
    parser.add_argument("--dca-adc-ip", default="192.168.33.180")
    parser.add_argument("--radar-interface", default="eno1")
    parser.add_argument("--radar-interface-cidr", default="192.168.33.30/24")
    parser.add_argument("--skip-network-setup", action="store_true")
    parser.add_argument("--imu-port", default="auto")
    parser.add_argument("--imu-baud", type=int, default=0)
    parser.add_argument("--imu-probe-seconds", type=float, default=0.8)
    parser.add_argument("--imu-warmup-seconds", type=float, default=1.0)
    parser.add_argument("--allow-no-imu", action="store_true")
    parser.add_argument("--camera-device", default="auto")
    parser.add_argument("--camera-width", type=int, default=1280)
    parser.add_argument("--camera-height", type=int, default=720)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--camera-fourcc", default="MJPG")
    parser.add_argument(
        "--camera-format",
        choices=("video", "jpg", "png"),
        default="video",
        help=(
            "Kept for manifest compatibility. The current station camera pipeline stores the full video plus "
            "a timestamp CSV and does not write per-frame image files."
        ),
    )
    parser.add_argument("--camera-warmup-seconds", type=float, default=1.5)
    parser.add_argument("--camera-save-every-n", type=int, default=1)
    parser.add_argument("--disable-camera", action="store_true")
    parser.add_argument("--allow-no-camera", action="store_true")
    return parser.parse_args()


def sanitize_label(label: str) -> str:
    cleaned = "".join(ch.lower() if ch.isalnum() else "_" for ch in label.strip())
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    cleaned = cleaned.strip("_")
    return cleaned or "session"


def require_root() -> None:
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() != 0:
        raise SessionCaptureError("This guided capture entrypoint should be started with sudo/root on Linux.")


class SessionLogger:
    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()

    def write(self, message: str) -> None:
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{timestamp}] {message}"
        print(line)
        with self._lock:
            with self.log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(line + "\n")


@dataclass
class CameraProbeResult:
    ok: bool
    device: str
    resolved_device: str
    backend: str
    width: int
    height: int
    actual_fps: float | None
    actual_fourcc: str
    error: str | None
    attempts: list[dict[str, Any]]
    candidate_devices: list[dict[str, Any]]


def list_video_nodes() -> list[str]:
    ordered_roots = [
        (Path("/dev/v4l/by-id"), "*video-index0"),
        (Path("/dev/v4l/by-path"), "*video-index0"),
        (Path("/dev"), "video*"),
    ]
    nodes: list[str] = []
    seen_resolved: set[str] = set()
    for root, pattern in ordered_roots:
        if not root.exists():
            continue
        for path in sorted(root.glob(pattern)):
            try:
                resolved = str(path.resolve())
            except OSError:
                resolved = str(path)
            if resolved in seen_resolved:
                continue
            seen_resolved.add(resolved)
            nodes.append(str(path))
    return nodes


def canonical_video_device(device: str) -> str:
    try:
        return str(Path(device).resolve())
    except OSError:
        return device


def decode_fourcc_value(raw_value: float | int | None) -> str:
    try:
        value = int(raw_value or 0)
    except (TypeError, ValueError):
        return ""
    chars = [chr((value >> shift) & 0xFF) for shift in range(0, 32, 8)]
    decoded = "".join(ch for ch in chars if ch != "\x00")
    return decoded.strip()


def extract_v4l2_section_lines(text: str, header: str) -> list[str]:
    lines: list[str] = []
    active = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith(header):
            active = True
            continue
        if active:
            if not line.startswith((" ", "\t")):
                break
            if stripped:
                lines.append(stripped)
    return lines


def inspect_video_node(device: str) -> dict[str, Any]:
    info: dict[str, Any] = {
        "device": device,
        "resolved_device": canonical_video_device(device),
        "stable_device": device if Path(device).is_symlink() else "",
        "supports_video_capture": None,
        "supports_metadata_capture": None,
        "supports_streaming": None,
        "card": "",
        "bus_info": "",
        "error": None,
    }
    if shutil.which("v4l2-ctl") is None:
        return info

    completed = subprocess.run(
        ["v4l2-ctl", "-d", device, "--all"],
        capture_output=True,
        text=True,
        check=False,
    )
    detail = (completed.stdout or "") + (completed.stderr or "")
    if completed.returncode != 0:
        info["error"] = detail.strip() or f"v4l2-ctl exited with {completed.returncode}"
        return info

    for line in detail.splitlines():
        stripped = line.strip()
        if stripped.startswith("Card type"):
            info["card"] = stripped.split(":", 1)[1].strip()
        elif stripped.startswith("Bus info"):
            info["bus_info"] = stripped.split(":", 1)[1].strip()

    device_caps_lines = extract_v4l2_section_lines(detail, "Device Caps")
    if not device_caps_lines:
        device_caps_lines = extract_v4l2_section_lines(detail, "Capabilities")
    info["supports_video_capture"] = "Video Capture" in device_caps_lines
    info["supports_metadata_capture"] = "Metadata Capture" in device_caps_lines
    info["supports_streaming"] = "Streaming" in device_caps_lines
    return info


def resolve_camera_candidates(camera_device: str) -> tuple[list[str], list[dict[str, Any]]]:
    raw_candidates = [camera_device] if camera_device.lower() != "auto" else list_video_nodes()
    candidate_devices = [inspect_video_node(candidate) for candidate in raw_candidates]
    preferred = [
        item["device"]
        for item in candidate_devices
        if item.get("supports_video_capture") is not False
    ]
    fallback = [
        item["device"]
        for item in candidate_devices
        if item.get("supports_video_capture") is False
    ]
    return preferred + fallback, candidate_devices


def probe_camera(camera_device: str, width: int, height: int, fps: int, fourcc: str, warmup_seconds: float) -> CameraProbeResult:
    attempts: list[dict[str, Any]] = []
    candidates, candidate_devices = resolve_camera_candidates(camera_device)
    if not candidates:
        return CameraProbeResult(
            ok=False,
            device="",
            resolved_device="",
            backend="",
            width=0,
            height=0,
            actual_fps=None,
            actual_fourcc="",
            error="No /dev/video* nodes are present.",
            attempts=attempts,
            candidate_devices=candidate_devices,
        )

    if shutil.which("gst-launch-1.0") is None:
        return CameraProbeResult(
            ok=False,
            device="",
            resolved_device="",
            backend="",
            width=0,
            height=0,
            actual_fps=None,
            actual_fourcc="",
            error="gst-launch-1.0 is required for direct camera probing.",
            attempts=attempts,
            candidate_devices=candidate_devices,
        )
    if shutil.which("ffprobe") is None:
        return CameraProbeResult(
            ok=False,
            device="",
            resolved_device="",
            backend="",
            width=0,
            height=0,
            actual_fps=None,
            actual_fourcc="",
            error="ffprobe is required for direct camera probing.",
            attempts=attempts,
            candidate_devices=candidate_devices,
        )

    requested_fourcc = fourcc.strip().upper()
    if requested_fourcc != "MJPG":
        return CameraProbeResult(
            ok=False,
            device="",
            resolved_device="",
            backend="",
            width=0,
            height=0,
            actual_fps=None,
            actual_fourcc="",
            error=f"Direct camera probing currently supports MJPG only, got {requested_fourcc}.",
            attempts=attempts,
            candidate_devices=candidate_devices,
        )

    probe_frames = max(3, min(30, int(round(max(warmup_seconds, 0.2) * max(fps, 1)))))
    for candidate in candidates:
        device = normalize_camera_device(candidate)
        probe_path = Path("/tmp") / f"camera_probe_{os.getpid()}_{time.time_ns()}.avi"
        try:
            completed = subprocess.run(
                build_gstreamer_command(device, width, height, fps, probe_path)[:4]
                + ["num-buffers=%d" % probe_frames]
                + build_gstreamer_command(device, width, height, fps, probe_path)[4:],
                capture_output=True,
                text=True,
                check=False,
            )
            if completed.returncode != 0:
                detail = (completed.stderr or "").strip() or (completed.stdout or "").strip()
                attempts.append(
                    {
                        "device": candidate,
                        "resolved_device": canonical_video_device(candidate),
                        "error": detail or f"gst-launch exited with {completed.returncode}",
                    }
                )
                continue

            stream = probe_video_with_ffprobe(probe_path)
            frame_count = int(str(stream.get("nb_read_packets") or "0") or "0")
            if frame_count <= 0:
                attempts.append(
                    {
                        "device": candidate,
                        "resolved_device": canonical_video_device(candidate),
                        "error": "gstreamer probe recorded no packets",
                    }
                )
                continue

            actual_fps = (
                parse_rate_text(str(stream.get("avg_frame_rate") or ""))
                or parse_rate_text(str(stream.get("r_frame_rate") or ""))
                or float(fps)
            )
            return CameraProbeResult(
                ok=True,
                device=candidate,
                resolved_device=canonical_video_device(candidate),
                backend="gstreamer-probe",
                width=int(stream.get("width") or width),
                height=int(stream.get("height") or height),
                actual_fps=actual_fps,
                actual_fourcc=requested_fourcc,
                error=None,
                attempts=attempts,
                candidate_devices=candidate_devices,
            )
        except Exception as exc:
            attempts.append(
                {
                    "device": candidate,
                    "resolved_device": canonical_video_device(candidate),
                    "error": str(exc),
                }
            )
        finally:
            if probe_path.exists():
                probe_path.unlink()

    return CameraProbeResult(
        ok=False,
        device="",
        resolved_device="",
        backend="",
        width=0,
        height=0,
        actual_fps=None,
        actual_fourcc="",
        error="All camera candidates failed to produce frames.",
        attempts=attempts,
        candidate_devices=candidate_devices,
    )


def run_checked(command: list[str]) -> str:
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or f"exit code {completed.returncode}"
        raise SessionCaptureError(f"{' '.join(command)}: {detail}")
    return completed.stdout.strip()


def current_lsusb() -> list[str]:
    output = run_checked(["lsusb"])
    return [line for line in output.splitlines() if line.strip()]


def radar_usb_present(lsusb_lines: list[str]) -> bool:
    return any("0451:fd03" in line or "AR-DevPack-EVM-012" in line for line in lsusb_lines)


def estimate_num_frames(radar_config: Path, duration_seconds: float) -> tuple[int, dict[str, Any], dict[str, Any], int]:
    _, _, adc_params, cfg_params = DCA1000.AWR2243_read_config_compat(str(radar_config))
    mode = parse_radar_mode_config(radar_config)
    validate_radar_mode_summary(mode)
    frame_period_ns = int(round(float(mode["effective_frame_period_ms"]) * 1_000_000))
    hardware = mode.get("hardware_trigger") or {}
    num_frames = expected_radar_frames(duration_seconds, float(mode["frame_period_ms"]),
                                       hardware.get("external_trigger_period_ms"))
    return num_frames, adc_params, cfg_params, frame_period_ns


def event_monotonic_ns(section: dict[str, Any], event_name: str) -> int | None:
    for item in section.get("events", []):
        if item.get("name") == event_name:
            return item.get("host_monotonic_ns")
    return None


def actual_video_duration_seconds(camera_summary: dict[str, Any]) -> float | None:
    duration_ns = camera_summary.get("saved_duration_ns")
    if duration_ns is not None:
        return float(duration_ns) / 1_000_000_000
    duration = camera_summary.get("actual_video_duration") or camera_summary.get("video_duration_seconds")
    return None if duration is None else float(duration)


def format_delta_ms(start_ns: int | None, stop_ns: int | None) -> str:
    if start_ns is None or stop_ns is None:
        return "unknown"
    return f"{(stop_ns - start_ns) / 1_000_000:.1f} ms"


def build_radar_mode_summary(
    *,
    radar_timing_summary: dict[str, Any] | None,
    num_frames: int,
    duration_seconds: float,
    raw_path: Path,
) -> dict[str, Any]:
    summary = dict(radar_timing_summary or {})
    bytes_per_frame = None
    expected_bytes = None
    if radar_timing_summary is not None:
        bytes_per_frame = expected_radar_raw_bytes(radar_timing_summary, 1)
        expected_bytes = expected_radar_raw_bytes(radar_timing_summary, num_frames)

    actual_bytes = raw_path.stat().st_size if raw_path.exists() else None
    inferred_frames = None
    if actual_bytes is not None and bytes_per_frame:
        inferred_frames = int(round(actual_bytes / bytes_per_frame))

    summary.update(
        {
            "requested_duration_sec": duration_seconds,
            "expected_radar_frames": num_frames,
            "expected_raw_bytes": expected_bytes,
            "actual_raw_bytes": actual_bytes,
            "actual_radar_frames": inferred_frames,
            "nearest_inferred_actual_frames": inferred_frames,
            "file_size_matches": (
                None if actual_bytes is None or expected_bytes is None else actual_bytes == expected_bytes
            ),
            "raw_byte_difference": (
                None if actual_bytes is None or expected_bytes is None else actual_bytes - expected_bytes
            ),
            "bytes_per_frame": bytes_per_frame,
            "output_path": str(raw_path) if raw_path.exists() else "",
        }
    )
    return summary


def stream_process_output(
    process: subprocess.Popen[str],
    log_path: Path,
    logger: SessionLogger,
    radar_event_callback: Any | None = None,
    radar_started_event: threading.Event | None = None,
) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("w", encoding="utf-8") as log_file:
        assert process.stdout is not None
        for line in process.stdout:
            log_file.write(line)
            log_file.flush()
            if radar_started_event is not None and "RADAR_RECORDING_STARTED " in line:
                radar_started_event.set()
            if radar_event_callback is not None and line.startswith("RADAR_EVENT "):
                try:
                    _, name, mono_ns, wall_ns = line.strip().split(maxsplit=3)
                    radar_event_callback(
                        {
                            "name": name,
                            "host_monotonic_ns": int(mono_ns),
                            "host_wall_time_ns": int(wall_ns),
                        }
                    )
                except Exception as exc:
                    logger.write(f"Could not parse radar event line: {exc}")
            logger.write(f"[radar] {line.rstrip()}")


def move_artifact(source: Path, destination: Path, session_dir: Path) -> str:
    if not source.exists():
        return ""
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        destination.unlink()
    shutil.move(str(source), str(destination))
    return str(destination.relative_to(session_dir))


def _compute_rate(sample_count: int, first_ns: int | None, last_ns: int | None) -> tuple[int | None, float | None]:
    if first_ns is None or last_ns is None or last_ns <= first_ns or sample_count <= 1:
        return None, None
    duration_ns = last_ns - first_ns
    return duration_ns, (sample_count - 1) / (duration_ns / 1_000_000_000)


def read_camera_csv_rows(csv_path: Path) -> tuple[list[str], list[dict[str, str]]]:
    if not csv_path.exists():
        return [], []
    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        rows = list(reader)
    return fieldnames, rows


def write_camera_csv_rows(csv_path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_camera_rows(rows: list[dict[str, str]]) -> dict[str, Any]:
    first_ns = int(rows[0]["host_monotonic_ns"]) if rows else None
    last_ns = int(rows[-1]["host_monotonic_ns"]) if rows else None
    duration_ns, actual_saved_fps = _compute_rate(len(rows), first_ns, last_ns)
    return {
        "saved_frames": len(rows),
        "first_saved_monotonic_ns": first_ns,
        "last_saved_monotonic_ns": last_ns,
        "saved_duration_ns": duration_ns,
        "actual_saved_fps": actual_saved_fps,
    }


def move_camera_capture_to_final(
    temp_video_path: Path,
    temp_csv_path: Path,
    final_video_path: Path,
    final_csv_path: Path,
) -> dict[str, Any]:
    fieldnames, rows = read_camera_csv_rows(temp_csv_path)
    final_video_path.parent.mkdir(parents=True, exist_ok=True)
    final_csv_path.parent.mkdir(parents=True, exist_ok=True)
    if temp_video_path.exists():
        if final_video_path.exists():
            final_video_path.unlink()
        shutil.move(str(temp_video_path), str(final_video_path))
    if temp_csv_path.exists():
        if final_csv_path.exists():
            final_csv_path.unlink()
        shutil.move(str(temp_csv_path), str(final_csv_path))
    summary = summarize_camera_rows(rows)
    summary.update(
        {
            "video_file": final_video_path.name if final_video_path.exists() else "",
            "frames_csv": final_csv_path.name if final_csv_path.exists() else "",
            "trimmed_to_radar_window": False,
            "single_frame_images_saved": False,
            "timestamp_strategy": "full_video_plus_synthesized_timestamp_csv",
        }
    )
    return summary


def trim_imu_csv_to_window(csv_path: Path, start_ns: int, stop_ns: int) -> dict[str, Any]:
    if not csv_path.exists():
        return {}

    with csv_path.open("r", newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        original_rows = list(reader)
        kept_rows = [
            row
            for row in original_rows
            if start_ns <= int(row.get("host_monotonic_ns", "0")) <= stop_ns
        ]

    if original_rows and not kept_rows:
        # Preserve the original IMU capture instead of silently erasing it.
        kept_rows = original_rows
        trimmed_to_window = False
        trim_warning = (
            "No IMU rows landed inside the requested radar window; preserved the original IMU capture to avoid data loss."
        )
    else:
        trimmed_to_window = True
        trim_warning = None

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(kept_rows)

    first_ns = int(kept_rows[0]["host_monotonic_ns"]) if kept_rows else None
    last_ns = int(kept_rows[-1]["host_monotonic_ns"]) if kept_rows else None
    return {
        "sample_count": len(kept_rows),
        "first_sample_monotonic_ns": first_ns,
        "last_sample_monotonic_ns": last_ns,
        "trimmed_to_radar_window": trimmed_to_window,
        "trim_window_start_monotonic_ns": start_ns,
        "trim_window_stop_monotonic_ns": stop_ns,
        "trim_warning": trim_warning,
    }


def latest_boot_imu_csv() -> Path | None:
    configured = os.environ.get("RADAR_BOOT_IMU_CSV", "").strip()
    if configured:
        path = Path(configured).expanduser()
        if path.exists():
            return path
    root = Path(os.environ.get("RADAR_BOOT_IMU_ROOT", str(ROOT / "data" / "boot_imu_logs"))).expanduser()
    if not root.exists():
        return None
    candidates = sorted(
        root.glob("boot_*/imu_samples.csv"),
        key=lambda item: item.stat().st_mtime if item.exists() else 0.0,
    )
    return candidates[-1] if candidates else None


def copy_imu_csv_window(source_csv: Path, dest_csv: Path, start_ns: int, stop_ns: int) -> dict[str, Any]:
    if not source_csv.exists():
        return {}
    dest_csv.parent.mkdir(parents=True, exist_ok=True)
    sample_count = 0
    first_ns: int | None = None
    last_ns: int | None = None
    with source_csv.open("r", newline="", encoding="utf-8") as source_file:
        reader = csv.DictReader(source_file)
        fieldnames = reader.fieldnames or []
        if "host_monotonic_ns" not in fieldnames:
            return {
                "sample_count": 0,
                "first_sample_monotonic_ns": None,
                "last_sample_monotonic_ns": None,
                "trimmed_to_radar_window": False,
                "trim_window_start_monotonic_ns": start_ns,
                "trim_window_stop_monotonic_ns": stop_ns,
                "trim_warning": "Boot IMU CSV did not contain host_monotonic_ns.",
                "source": "boot_imu_logger",
                "source_csv": str(source_csv),
            }
        with dest_csv.open("w", newline="", encoding="utf-8") as dest_file:
            writer = csv.DictWriter(dest_file, fieldnames=fieldnames)
            writer.writeheader()
            for row in reader:
                try:
                    mono_ns = int(row.get("host_monotonic_ns", "0"))
                except ValueError:
                    continue
                if start_ns <= mono_ns <= stop_ns:
                    writer.writerow(row)
                    sample_count += 1
                    if first_ns is None:
                        first_ns = mono_ns
                    last_ns = mono_ns
    return {
        "enabled": True,
        "port": os.environ.get("RADAR_BOOT_IMU_PORT", "/dev/ttyUSB0"),
        "baudrate": int(os.environ.get("RADAR_BOOT_IMU_BAUD", "921600")),
        "csv_file": dest_csv.name,
        "sample_count": sample_count,
        "first_sample_monotonic_ns": first_ns,
        "last_sample_monotonic_ns": last_ns,
        "trimmed_to_radar_window": True,
        "trim_window_start_monotonic_ns": start_ns,
        "trim_window_stop_monotonic_ns": stop_ns,
        "trim_warning": None if sample_count > 0 else "No boot IMU rows landed inside the radar window.",
        "source": "boot_imu_logger",
        "source_csv": str(source_csv),
    }


def trim_camera_capture_to_window(
    temp_video_path: Path,
    temp_csv_path: Path,
    final_video_path: Path,
    final_csv_path: Path,
    start_ns: int,
    stop_ns: int,
    output_fourcc: str,
    output_fps: float,
) -> dict[str, Any]:
    fieldnames, rows = read_camera_csv_rows(temp_csv_path)
    if not rows:
        return {}
    summary = move_camera_capture_to_final(
        temp_video_path,
        temp_csv_path,
        final_video_path,
        final_csv_path,
    )
    summary.update(
        {
            "trimmed_to_radar_window": False,
            "trim_window_start_monotonic_ns": start_ns,
            "trim_window_stop_monotonic_ns": stop_ns,
            "alignment_mode": "offline_postprocess_required",
            "alignment_note": (
                "Full camera video and full timestamp CSV were preserved. "
                "Align the camera stream to the radar window offline after pulling the session."
            ),
            "requested_output_fourcc": output_fourcc,
            "requested_output_fps": output_fps,
        }
    )
    return summary


def main() -> int:
    args = parse_args()
    require_root()

    timestamp_prefix = time.strftime("%Y%m%d_%H%M%S")
    session_label = f"{timestamp_prefix}__{sanitize_label(args.station_id)}__{sanitize_label(args.name)}"
    session_dir = resolve_session_dir(args.output_root, session_label, args.session_dir)
    logs_dir = session_dir / "logs"
    radar_dir = session_dir / "radar"
    imu_dir = session_dir / "imu"
    camera_dir = session_dir / "camera"
    tmp_dir = session_dir / "_radar_capture"
    logs_dir.mkdir(parents=True, exist_ok=True)
    radar_dir.mkdir(parents=True, exist_ok=True)
    imu_dir.mkdir(parents=True, exist_ok=True)
    camera_dir.mkdir(parents=True, exist_ok=True)
    logger = SessionLogger(logs_dir / "session.log")

    logger.write(f"Session folder created: {session_dir}")
    logger.write(f"Station id: {args.station_id}")
    logger.write(f"Requested label: {args.name}")

    host_time = host_time_snapshot()
    lsusb_lines = current_lsusb()
    video_nodes = list_video_nodes()
    serial_ports = available_serial_ports()
    network_summary: dict[str, Any] | None = None
    imu_probe = None
    boot_imu_csv = latest_boot_imu_csv()
    use_boot_imu_for_session = False
    camera_probe: CameraProbeResult | None = None
    preflight_errors: list[str] = []
    preflight_warnings: list[str] = []

    if host_time.get("warning"):
        preflight_warnings.append(str(host_time["warning"]))
        logger.write(f"Preflight warning: {host_time['warning']}")

    if not args.skip_network_setup:
        network_summary = prepare_radar_interface(args.radar_interface, args.radar_interface_cidr)
        logger.write(f"Radar interface prepared: {network_summary['addr_state']}")

    if not radar_usb_present(lsusb_lines):
        preflight_errors.append("Radar FTDI device (0451:fd03) not found in lsusb.")

    if boot_imu_csv is not None and args.imu_port.lower() == "auto":
        use_boot_imu_for_session = True
        preflight_warnings.append(
            f"External IMU will be sliced from boot logger CSV instead of reopening the serial port: {boot_imu_csv}"
        )
    else:
        imu_probe = detect_imu_port(args.imu_port, args.imu_baud, args.imu_probe_seconds)
        if imu_probe is None and not args.allow_no_imu and args.imu_port.lower() != "none":
            preflight_errors.append("No decodable IMU stream was detected.")

    if not args.disable_camera:
        camera_probe = probe_camera(
            camera_device=args.camera_device,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
            fourcc=args.camera_fourcc,
            warmup_seconds=args.camera_warmup_seconds,
        )
        if not camera_probe.ok and not args.allow_no_camera:
            preflight_errors.append(camera_probe.error or "Camera probe failed.")

    radar_timing_summary: dict[str, Any] | None = None
    try:
        radar_timing_summary = parse_radar_mode_config(args.radar_config)
        validate_radar_mode_summary(radar_timing_summary)
    except Exception as exc:
        preflight_errors.append(f"Radar frame timing validation failed: {exc}")

    num_frames, adc_params, cfg_params, frame_period_ns = estimate_num_frames(
        args.radar_config,
        args.duration_seconds,
    )
    expected_duration_seconds = expected_radar_window_seconds(radar_timing_summary, num_frames)
    logger.write(
        "Radar capture budget: "
        f"requested={args.duration_seconds:.3f}s, frame_period_ms={float(adc_params['frame_periodicity']):.3f}, "
        f"trigger_period_ms={frame_period_ns / 1_000_000:.3f}, "
        f"num_frames={num_frames}, expected_duration={expected_duration_seconds:.3f}s"
    )

    preflight_summary = {
        "checked_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host_time": host_time,
        "lsusb": lsusb_lines,
        "video_nodes": video_nodes,
        "serial_ports": serial_ports,
        "radar_usb_present": radar_usb_present(lsusb_lines),
        "network": network_summary,
        "imu": (
            {
                "port": os.environ.get("RADAR_BOOT_IMU_PORT", "/dev/ttyUSB0"),
                "baudrate": int(os.environ.get("RADAR_BOOT_IMU_BAUD", "921600")),
                "probe_hits": 0,
                "source": "boot_imu_logger",
                "source_csv": str(boot_imu_csv) if boot_imu_csv is not None else "",
            }
            if use_boot_imu_for_session
            else (
                None
                if imu_probe is None
                else {"port": imu_probe.port, "baudrate": imu_probe.baudrate, "probe_hits": imu_probe.hits}
            )
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
        "frame_budget": {
            "duration_seconds_requested": args.duration_seconds,
            "num_frames": num_frames,
            "frame_period_ns": frame_period_ns,
            "expected_duration_seconds": expected_duration_seconds,
            "radar_timing_validation": radar_timing_summary,
            "adc_params": adc_params,
            "cfg_params": cfg_params,
        },
        "warnings": list(preflight_warnings),
        "errors": list(preflight_errors),
    }
    with (logs_dir / "preflight.json").open("w", encoding="utf-8") as preflight_file:
        json.dump(preflight_summary, preflight_file, indent=2)

    if preflight_errors:
        for item in preflight_errors:
            logger.write(f"Preflight failed: {item}")
        manifest = {
            "status": "preflight_failed",
            "session_name": session_dir.name,
            "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
            "station_id": args.station_id,
            "capture_id": args.capture_id,
            "requested_name": args.name,
            "preflight": preflight_summary,
            "error": {"stage": "preflight", "message": "; ".join(preflight_errors)},
            "artifacts": {
                "session_log": "logs/session.log",
                "preflight": "logs/preflight.json",
            },
        }
        with (session_dir / "session_manifest.json").open("w", encoding="utf-8") as manifest_file:
            json.dump(manifest, manifest_file, indent=2)
        restore_session_ownership(session_dir)
        return 2

    logger.write("Preflight OK: required devices are available.")
    logger.write(
        "Software sync only. Use metadata timestamps for alignment. "
        "Video duration may be longer than radar duration by design."
    )

    if args.start_at_wall_ns is None:
        logger.write(f"Countdown: {args.countdown_seconds:.1f}s before capture start.")
        time.sleep(max(args.countdown_seconds, 0.0))
    else:
        logger.write(f"Scheduled common radar start wall_ns={args.start_at_wall_ns}.")

    camera_process: subprocess.Popen[str] | None = None
    camera_log_handle = None
    camera_manifest_path = logs_dir / "camera_capture_manifest.json"
    camera_ready_path = logs_dir / "camera_capture_ready.json"
    camera_ready: dict[str, Any] | None = None
    camera_launch_wall_ns: int | None = None
    camera_launch_mono_ns: int | None = None
    camera_stop_mono_ns: int | None = None
    camera_temp_video_path = camera_dir / "_camera_full.avi"
    camera_temp_csv_path = camera_dir / "_camera_frames_full.csv"
    camera_final_video_path = camera_dir / "camera_video.avi"
    camera_final_csv_path = camera_dir / "camera_frames.csv"
    if camera_probe is not None and camera_probe.ok:
        camera_cmd = [
            sys.executable,
            str(ROOT / "capture_camera_session.py"),
            "--device",
            camera_probe.device,
            "--width",
            str(args.camera_width),
            "--height",
            str(args.camera_height),
            "--fps",
            str(args.camera_fps),
            "--fourcc",
            args.camera_fourcc,
            "--video-path",
            str(camera_temp_video_path),
            "--csv-path",
            str(camera_temp_csv_path),
            "--manifest-path",
            str(camera_manifest_path),
            "--ready-path",
            str(camera_ready_path),
            "--save-every-n",
            str(args.camera_save_every_n),
        ]
        camera_log_handle = (logs_dir / "camera_capture.log").open("w", encoding="utf-8")
        logger.write(f"Launching camera capture subprocess on {camera_probe.device} ({camera_probe.backend}).")
        camera_launch_wall_ns = time.time_ns()
        camera_launch_mono_ns = time.monotonic_ns()
        camera_process = subprocess.Popen(
            camera_cmd,
            stdout=camera_log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
        )
        ready_deadline = time.monotonic() + max(args.camera_warmup_seconds, 5.0)
        while time.monotonic() < ready_deadline:
            if camera_ready_path.exists():
                with camera_ready_path.open("r", encoding="utf-8") as ready_file:
                    camera_ready = json.load(ready_file)
                logger.write(
                    "Camera ready before radar start: "
                    f"device={camera_ready.get('device', camera_probe.device)}, "
                    f"resolved={camera_ready.get('resolved_device', camera_probe.resolved_device)}, "
                    f"ready_ns={camera_ready.get('first_frame_monotonic_ns')}"
                )
                break
            if camera_process.poll() is not None:
                logger.write("Camera subprocess exited before reporting a ready frame.")
                break
            time.sleep(0.05)
        if camera_ready is None:
            logger.write("Camera did not report a ready frame before radar start; continuing with runtime monitoring.")

    requested_start_wall_ns = time.time_ns()
    requested_start_mono_ns = time.monotonic_ns()
    camera_first_ready_ns = (camera_ready or {}).get("first_frame_monotonic_ns")
    sync_readiness = {
        "status": "camera_disabled" if args.disable_camera else ("ready" if camera_first_ready_ns else "not_ready"),
        "software_sync_only": True,
        "radar_pair_hardware_triggered": bool(radar_timing_summary and radar_timing_summary.get("hardware_trigger")),
        "camera_first_frame_monotonic_ns": camera_first_ready_ns,
        "radar_launch_request_monotonic_ns": requested_start_mono_ns,
        "camera_preroll_before_radar_request_ms": (
            None if camera_first_ready_ns is None else (requested_start_mono_ns - int(camera_first_ready_ns)) / 1_000_000
        ),
        "message": "",
    }
    if args.disable_camera:
        sync_readiness["message"] = "Camera disabled; radar-only capture."
    elif camera_first_ready_ns:
        sync_readiness["message"] = (
            "READY for software-synced recording. Camera is already recording; "
            "radar/camera alignment will use metadata timestamps."
        )
        logger.write(
            "SYNC CHECK after warm-up: READY for software-synced recording. "
            f"Camera pipeline was ready {format_delta_ms(int(camera_first_ready_ns), requested_start_mono_ns)} "
            "before radar launch request. Camera is not hardware-synchronized; use metadata timestamps for alignment."
        )
    else:
        sync_readiness["message"] = (
            "NOT READY: camera did not provide a first-frame timestamp before radar launch. "
            "Capture may still run, but camera/radar alignment must be treated as weak."
        )
        logger.write(
            "SYNC CHECK after warm-up: NOT READY. Camera did not provide a first-frame timestamp before "
            "radar launch; any camera/radar alignment should be treated as weak."
        )
    hardware_triggered = bool(radar_timing_summary and radar_timing_summary.get("hardware_trigger"))
    if hardware_triggered:
        logger.write(f"Arming radar for external trigger in scene '{args.name}'.")
    else:
        logger.write(f"Radar launch requested for scene '{args.name}'; sensor start is scheduled separately." if args.start_at_wall_ns else f"Recording starts now for scene '{args.name}'.")

    radar_cmd = [
        sys.executable,
        str(ROOT / "capture_session_awr2243.py"),
        "--session-dir",
        str(tmp_dir),
        "--radar-config",
        str(args.radar_config),
        "--dca-config",
        str(args.dca_config),
        "--numframes",
        str(num_frames),
        "--dca-static-ip",
        args.dca_static_ip,
        "--dca-adc-ip",
        args.dca_adc_ip,
        "--radar-interface",
        args.radar_interface,
        "--radar-interface-cidr",
        args.radar_interface_cidr,
        "--imu-probe-seconds",
        str(args.imu_probe_seconds),
            "--imu-warmup-seconds",
            str(args.imu_warmup_seconds),
            "--emit-events",
        ]
    if args.start_at_wall_ns is not None:
        radar_cmd.extend(["--start-at-wall-ns", str(args.start_at_wall_ns)])
    if args.skip_network_setup:
        radar_cmd.append("--skip-network-setup")
    if use_boot_imu_for_session:
        radar_cmd.extend(["--allow-no-imu", "--imu-port", "none"])
        logger.write(f"External IMU: using boot logger CSV for this session ({boot_imu_csv}).")
    elif args.allow_no_imu:
        radar_cmd.append("--allow-no-imu")
    if args.imu_port.lower() != "auto":
        if not use_boot_imu_for_session:
            radar_cmd.extend(["--imu-port", args.imu_port])
    elif imu_probe is not None:
        radar_cmd.extend(["--imu-port", imu_probe.port, "--imu-baud", str(imu_probe.baudrate)])
    elif args.imu_port.lower() == "none":
        radar_cmd.extend(["--imu-port", "none"])
    elif args.imu_baud:
        radar_cmd.extend(["--imu-baud", str(args.imu_baud)])

    camera_stop_reason: dict[str, Any] = {}

    def terminate_camera(reason: str, details: dict[str, Any] | None = None) -> None:
        nonlocal camera_stop_mono_ns
        if camera_process is None:
            return
        if camera_process.poll() is not None:
            if camera_stop_mono_ns is None:
                camera_stop_mono_ns = time.monotonic_ns()
            return
        if camera_stop_mono_ns is None:
            camera_stop_mono_ns = time.monotonic_ns()
        camera_stop_reason.update(
            {
                "reason": reason,
                **(details or {}),
            }
        )
        try:
            camera_process.terminate()
        except Exception as exc:
            camera_stop_reason["error"] = str(exc)

    logger.write("Launching radar raw capture subprocess.")
    radar_env = os.environ.copy()
    if hardware_triggered:
        radar_env["RADAR_READER_MODE"] = "framefile2"
    radar_process = subprocess.Popen(
        radar_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=radar_env,
    )
    radar_started_event = threading.Event()

    def report_start_wait_status() -> None:
        if radar_process.poll() is not None:
            return
        if radar_started_event.is_set():
            logger.write("CAPTURE_STATUS after_command_3p5s radar_started=true")
        elif args.start_at_wall_ns is not None:
            logger.write(f"CAPTURE_STATUS scheduled_start_pending target_wall_ns={args.start_at_wall_ns}")
        else:
            logger.write(
                "CAPTURE_STATUS after_command_3p5s command_sent=true "
                "waiting_for_RADAR_RECORDING_STARTED=true"
            )

    start_wait_timer = threading.Timer(3.5, report_start_wait_status)
    start_wait_timer.daemon = True
    start_wait_timer.start()
    # Keep full video until the radar subprocess finishes, including configuration and raw writes.
    stream_process_output(
        radar_process,
        logs_dir / "radar_capture.log",
        logger,
        radar_started_event=radar_started_event,
    )
    radar_exit_code = radar_process.wait()
    start_wait_timer.cancel()
    requested_stop_wall_ns = time.time_ns()
    requested_stop_mono_ns = time.monotonic_ns()
    logger.write(f"Radar subprocess finished with exit code {radar_exit_code}.")

    camera_summary: dict[str, Any]
    if camera_process is not None:
        if camera_process.poll() is None:
            if camera_stop_mono_ns is None:
                camera_stop_mono_ns = time.monotonic_ns()
            camera_process.terminate()
            try:
                camera_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                camera_process.kill()
                camera_process.wait(timeout=5.0)
        elif camera_stop_mono_ns is None:
            camera_stop_mono_ns = time.monotonic_ns()
        if camera_log_handle is not None:
            camera_log_handle.close()

        camera_summary = {
            "enabled": True,
            "device": camera_probe.device if camera_probe is not None else args.camera_device,
            "resolved_device": camera_probe.resolved_device if camera_probe is not None else args.camera_device,
            "saved_frames": 0,
            "frames_csv": "",
            "error": "Camera child manifest was not produced.",
        }
        if camera_manifest_path.exists():
            with camera_manifest_path.open("r", encoding="utf-8") as camera_manifest_file:
                camera_manifest = json.load(camera_manifest_file)
            camera_summary = camera_manifest.get("camera", camera_summary)
        if camera_stop_reason:
            camera_summary["stop_reason"] = camera_stop_reason
    else:
        camera_summary = {
            "enabled": not args.disable_camera,
            "device": args.camera_device,
            "resolved_device": camera_probe.resolved_device if camera_probe is not None else args.camera_device,
            "saved_frames": 0,
            "frames_csv": "",
            "error": None if args.disable_camera else (camera_probe.error if camera_probe is not None else None),
        }

    radar_manifest_path = tmp_dir / "session_manifest.json"
    radar_manifest: dict[str, Any] | None = None
    radar_error: dict[str, Any] | None = None
    if radar_manifest_path.exists():
        with radar_manifest_path.open("r", encoding="utf-8") as radar_manifest_file:
            radar_manifest = json.load(radar_manifest_file)
    else:
        radar_error = {"stage": "radar_capture", "message": "Radar child manifest was not produced."}

    radar_section = {}
    imu_section = {}
    network_section = None
    if radar_manifest is not None:
        radar_section = radar_manifest.get("radar") or {}
        imu_section = radar_manifest.get("imu") or {}
        network_section = radar_manifest.get("network")

    radar_config_start_monotonic_ns = event_monotonic_ns(radar_section, "radar_config_start")
    radar_arm_monotonic_ns = event_monotonic_ns(radar_section, "sensor_start_sent")
    first_packet_event = next(
        (event for event in radar_section.get("events", []) if event.get("name") == "first_udp_packet_received"),
        None,
    )
    radar_start_monotonic_ns = (
        int(first_packet_event["udp_receive_monotonic_ns"])
        if hardware_triggered and first_packet_event else
        None if hardware_triggered else radar_arm_monotonic_ns
    )
    radar_stop_monotonic_ns = event_monotonic_ns(radar_section, "sensor_stop_detected")
    radar_recording_complete_before_save_ns = event_monotonic_ns(
        radar_section,
        "recording_complete_before_raw_write",
    )

    if (
        radar_start_monotonic_ns is not None
        and radar_stop_monotonic_ns is not None
        and radar_stop_monotonic_ns > radar_start_monotonic_ns
    ):
        if use_boot_imu_for_session and boot_imu_csv is not None:
            trimmed_imu = copy_imu_csv_window(
                boot_imu_csv,
                tmp_dir / "imu_samples.csv",
                radar_start_monotonic_ns,
                radar_stop_monotonic_ns,
            )
        else:
            trimmed_imu = trim_imu_csv_to_window(
                tmp_dir / "imu_samples.csv",
                radar_start_monotonic_ns,
                radar_stop_monotonic_ns,
            )
        if trimmed_imu:
            imu_section.update(trimmed_imu)

    # Preserve the complete video and original frame timestamps; never re-encode/crop it.
    if camera_process is not None:
        full_camera = move_camera_capture_to_final(
            camera_temp_video_path, camera_temp_csv_path,
            camera_final_video_path, camera_final_csv_path,
        )
        camera_summary.update(full_camera)
        camera_summary["timestamp_strategy"] = "full_video_actual_buffer_receive_timestamps_and_gst_pts"
        camera_summary["recording_policy"] = "camera_ready_before_radar_until_radar_subprocess_exit"
        first_video = camera_summary.get("first_saved_monotonic_ns")
        last_video = camera_summary.get("last_saved_monotonic_ns")
        camera_summary["covers_radar_window"] = bool(
            first_video is not None and last_video is not None
            and radar_start_monotonic_ns is not None and radar_stop_monotonic_ns is not None
            and first_video <= radar_start_monotonic_ns and last_video >= radar_stop_monotonic_ns
        )

    artifacts = {
        "session_log": "logs/session.log",
        "preflight": "logs/preflight.json",
        "radar_log": "logs/radar_capture.log",
        "radar_child_manifest": "",
        "radar_raw": "",
        "radar_mode_summary": "",
        "radar_quality_summary": "",
        "radar_frame_quality": "",
        "radar_frames": "",
        "imu_samples": "",
        "camera_log": "",
        "camera_child_manifest": "",
        "camera_ready": "",
        "camera_frames": camera_summary.get("frames_csv", ""),
        "camera_video": camera_summary.get("video_file", ""),
    }

    artifacts["radar_raw"] = move_artifact(tmp_dir / "radar_raw.bin", radar_dir / "radar_raw.bin", session_dir)
    artifacts["radar_frames"] = move_artifact(tmp_dir / "radar_frames.csv", radar_dir / "radar_frames.csv", session_dir)
    artifacts["radar_quality_summary"] = move_artifact(
        tmp_dir / "radar_quality_summary.json",
        radar_dir / "radar_quality_summary.json",
        session_dir,
    )
    artifacts["radar_frame_quality"] = move_artifact(
        tmp_dir / "radar_frame_quality.csv",
        radar_dir / "radar_frame_quality.csv",
        session_dir,
    )
    artifacts["imu_samples"] = move_artifact(tmp_dir / "imu_samples.csv", imu_dir / "imu_samples.csv", session_dir)
    artifacts["radar_child_manifest"] = move_artifact(
        radar_manifest_path,
        logs_dir / "radar_capture_manifest.json",
        session_dir,
    )
    if camera_manifest_path.exists():
        artifacts["camera_child_manifest"] = str(camera_manifest_path.relative_to(session_dir))
    if camera_ready_path.exists():
        artifacts["camera_ready"] = str(camera_ready_path.relative_to(session_dir))
    camera_log_path = logs_dir / "camera_capture.log"
    if camera_log_path.exists():
        artifacts["camera_log"] = str(camera_log_path.relative_to(session_dir))
    if camera_summary.get("frames_csv"):
        artifacts["camera_frames"] = str((camera_dir / camera_summary["frames_csv"]).relative_to(session_dir))
    if camera_summary.get("video_file"):
        artifacts["camera_video"] = str((camera_dir / camera_summary["video_file"]).relative_to(session_dir))

    radar_mode_summary = build_radar_mode_summary(
        radar_timing_summary=radar_timing_summary,
        num_frames=num_frames,
        duration_seconds=args.duration_seconds,
        raw_path=radar_dir / "radar_raw.bin",
    )
    radar_mode_summary_path = radar_dir / "radar_mode_summary.json"
    with radar_mode_summary_path.open("w", encoding="utf-8") as mode_summary_file:
        json.dump(radar_mode_summary, mode_summary_file, indent=2)
    artifacts["radar_mode_summary"] = str(radar_mode_summary_path.relative_to(session_dir))

    camera_first_frame_monotonic_ns = (
        (camera_ready or {}).get("first_frame_monotonic_ns")
        or camera_summary.get("first_frame_monotonic_ns")
        or camera_summary.get("first_saved_monotonic_ns")
    )
    radar_mode = os.environ.get("RADAR_MODE")
    if not radar_mode and radar_timing_summary is not None:
        radar_mode = radar_timing_summary.get("mode")
    session_metadata = {
        "sync_readiness_after_warmup": sync_readiness,
        "camera_start_monotonic_ns": camera_launch_mono_ns,
        "camera_first_frame_monotonic_ns": camera_first_frame_monotonic_ns,
        "radar_config_start_monotonic_ns": radar_config_start_monotonic_ns,
        "radar_start_monotonic_ns": radar_start_monotonic_ns,
        "radar_arm_monotonic_ns": radar_arm_monotonic_ns,
        "radar_start_timestamp_kind": "host_udp_receive_proxy" if hardware_triggered else "sensor_start_command",
        "station_trigger_role": (
            (radar_timing_summary.get("hardware_trigger") or {}).get("station_roles", {}).get(args.station_id)
            if hardware_triggered else None
        ),
        "radar_stop_monotonic_ns": radar_stop_monotonic_ns,
        "radar_recording_complete_before_save_monotonic_ns": radar_recording_complete_before_save_ns,
        "camera_stop_monotonic_ns": camera_stop_mono_ns,
        "radar_mode": radar_mode or "custom",
        "radar_config_path": str(args.radar_config),
        "camera_fps": args.camera_fps,
        "camera_resolution": {
            "width": args.camera_width,
            "height": args.camera_height,
        },
        "requested_duration_sec": args.duration_seconds,
        "expected_radar_frames": num_frames,
        "expected_raw_bytes": radar_mode_summary.get("expected_raw_bytes"),
        "actual_raw_bytes": radar_mode_summary.get("actual_raw_bytes"),
        "actual_radar_frames": radar_mode_summary.get("actual_radar_frames"),
        "file_size_matches": radar_mode_summary.get("file_size_matches"),
        "raw_byte_difference": radar_mode_summary.get("raw_byte_difference"),
        "radar_mode_summary": radar_mode_summary,
        "radar_quality": radar_section.get("quality") if radar_section else None,
        "actual_video_frames": camera_summary.get("saved_frames"),
        "actual_video_duration": actual_video_duration_seconds(camera_summary),
        "radar_timing_validation": radar_timing_summary,
        "sync_warning": (
            "Software sync only. Use metadata timestamps for alignment. "
            "Video duration may be longer than radar duration by design."
        ),
        "camera_preroll_postroll_note": (
            "Camera pre-roll and post-roll are intentionally retained. "
            "Crop video to the radar_start/radar_stop interval using timestamps, not raw file duration."
        ),
    }
    with (session_dir / "session_metadata.json").open("w", encoding="utf-8") as metadata_file:
        json.dump(session_metadata, metadata_file, indent=2)
    artifacts["session_metadata"] = "session_metadata.json"

    if tmp_dir.exists():
        shutil.rmtree(tmp_dir, ignore_errors=True)

    final_status = "ok"
    if radar_exit_code != 0:
        final_status = "error"
        if radar_manifest is not None:
            radar_error = radar_manifest.get("error")
        elif radar_error is None:
            radar_error = {"stage": "radar_capture", "message": f"Radar subprocess exit code {radar_exit_code}"}
    elif not args.allow_no_camera and not args.disable_camera:
        camera_issues = required_camera_issues(camera_summary, camera_final_csv_path)
        camera_summary["integrity_errors"] = camera_issues
        if camera_issues:
            final_status = "error"
            radar_error = {"stage": "camera_record", "message": "; ".join(camera_issues)}

    final_manifest = {
        "status": final_status,
        "session_name": session_dir.name,
        "station_id": args.station_id,
        "capture_id": args.capture_id,
        "requested_name": args.name,
        "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host_time": host_time,
        "host": {
            "hostname": os.uname().nodename if hasattr(os, "uname") else "",
            "user": os.environ.get("SUDO_USER") or os.environ.get("USER") or "",
        },
        "requested_capture": {
            "duration_seconds_requested": args.duration_seconds,
            "countdown_seconds": args.countdown_seconds,
            "scheduled_start_wall_time_ns": args.start_at_wall_ns,
            "num_frames_planned": num_frames,
            "frame_period_ns": frame_period_ns,
            "expected_duration_seconds": expected_duration_seconds,
        },
        "preflight": preflight_summary,
        "timing": {
            "requested_start_wall_time_ns": requested_start_wall_ns,
            "requested_start_monotonic_ns": requested_start_mono_ns,
            "requested_stop_wall_time_ns": requested_stop_wall_ns,
            "requested_stop_monotonic_ns": requested_stop_mono_ns,
            "requested_elapsed_seconds": (requested_stop_mono_ns - requested_start_mono_ns) / 1_000_000_000,
            "camera_launch_wall_time_ns": camera_launch_wall_ns,
            "camera_launch_monotonic_ns": camera_launch_mono_ns,
            "camera_ready_monotonic_ns": camera_first_frame_monotonic_ns,
            "camera_stop_monotonic_ns": camera_stop_mono_ns,
            "radar_config_start_monotonic_ns": radar_config_start_monotonic_ns,
            "radar_sensor_start_monotonic_ns": radar_arm_monotonic_ns,
            "radar_first_packet_receive_monotonic_ns": radar_start_monotonic_ns if hardware_triggered else None,
            "radar_arm_monotonic_ns": radar_arm_monotonic_ns,
            "radar_start_timestamp_kind": "host_udp_receive_proxy" if hardware_triggered else "sensor_start_command",
            "radar_sensor_stop_monotonic_ns": radar_stop_monotonic_ns,
            "radar_active_duration_seconds": (
                None
                if radar_start_monotonic_ns is None or radar_stop_monotonic_ns is None
                else (radar_stop_monotonic_ns - radar_start_monotonic_ns) / 1_000_000_000
            ),
            "scheduled_start_wall_time_ns": args.start_at_wall_ns,
            "radar_start_lateness_ms": (radar_section or {}).get("start_lateness_ms"),
        },
        "radar": radar_section or None,
        "imu": imu_section or None,
        "network": network_section,
        "camera": camera_summary,
        "reference_timestamps": {
            "requested_start_monotonic_ns": requested_start_mono_ns,
            "radar_config_start_monotonic_ns": radar_config_start_monotonic_ns,
            "radar_sensor_start_monotonic_ns": radar_start_monotonic_ns,
            "imu_first_sample_monotonic_ns": (
                None if radar_manifest is None else imu_section.get("first_sample_monotonic_ns")
            ),
            "camera_first_frame_monotonic_ns": camera_first_frame_monotonic_ns,
        },
        "session_metadata": session_metadata,
        "radar_mode_summary": radar_mode_summary,
        "error": radar_error,
        "artifacts": artifacts,
    }

    with (session_dir / "session_manifest.json").open("w", encoding="utf-8") as manifest_file:
        json.dump(final_manifest, manifest_file, indent=2)

    restore_session_ownership(session_dir)

    if final_status != "ok":
        logger.write(f"Capture failed: {radar_error}")
        return 2

    logger.write(
        f"Capture complete for '{args.name}': radar_raw={artifacts['radar_raw']}, "
        f"imu_samples={artifacts['imu_samples']}, camera_frames={camera_summary.get('saved_frames', 0)}"
    )
    print(f"Session complete: {args.name}")
    print(f"Saved to: {session_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SessionCaptureError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
