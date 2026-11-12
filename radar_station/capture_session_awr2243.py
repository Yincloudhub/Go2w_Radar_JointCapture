from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import site
import subprocess
import sys
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np


ROOT = Path(__file__).resolve().parent
DEFAULT_RADAR_CONFIG = ROOT / "configFiles" / "mmwaveconfig_tx0_rx03_77g_25m_1200s_256c_650ms.txt"
DEFAULT_DCA_CONFIG = ROOT / "configFiles" / "cf.json"
DEFAULT_SESSION_ROOT = ROOT / "data" / "sessions"
DEFAULT_IMU_BAUDRATES = [921600, 460800, 115200]
IMU_CAPTURE_HELPER = ROOT / "capture_imu_session.py"


def _directory_has_native_fpga_udp(path: Path) -> bool:
    if not path.exists():
        return False
    return any(path.glob("fpga_udp*.so")) or any(path.glob("fpga_udp*.pyd"))


def _resolve_native_fpga_udp_dir(path: str | Path) -> Path | None:
    path = Path(path).expanduser()
    if _directory_has_native_fpga_udp(path):
        return path
    if path.exists():
        for child in sorted(path.iterdir()):
            if child.is_dir() and _directory_has_native_fpga_udp(child):
                return child
    return None


def _find_native_fpga_udp_dir() -> Path | None:
    search_roots: list[Path] = []

    reader_mode = os.environ.get("RADAR_READER_MODE", "").strip().lower()
    experimental_reader = reader_mode in {"framefile2", "framefile", "file"}
    if experimental_reader:
        direct_native = _resolve_native_fpga_udp_dir(ROOT / "fpga_udp")
        if direct_native is not None:
            return direct_native

    override_dir = os.environ.get("FPGA_UDP_OVERRIDE_DIR", "").strip()
    if override_dir:
        override_path = Path(override_dir).expanduser()
        resolved_override = _resolve_native_fpga_udp_dir(override_path)
        if resolved_override is not None:
            return resolved_override

    override_root = os.environ.get("FPGA_UDP_OVERRIDE_ROOT", "").strip()
    if override_root:
        override_build = Path(override_root).expanduser() / "fpga_udp" / "build"
        if override_build.exists():
            search_roots.extend(path for path in sorted(override_build.iterdir()) if path.is_dir())

    if experimental_reader:
        build_dir = ROOT / "fpga_udp" / "build"
        if build_dir.exists():
            search_roots.extend(path for path in sorted(build_dir.iterdir()) if path.is_dir())

    for site_dir in site.getsitepackages():
        search_roots.append(Path(site_dir))

    user_site = site.getusersitepackages()
    if user_site:
        search_roots.append(Path(user_site))

    for search_root in search_roots:
        resolved_search_root = _resolve_native_fpga_udp_dir(search_root)
        if resolved_search_root is not None:
            return resolved_search_root
    return None


_native_dir = _find_native_fpga_udp_dir()
if _native_dir is not None and str(_native_dir) not in sys.path:
    sys.path.insert(0, str(_native_dir))

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

import fpga_udp

from imu_hwt906p import IMU_CSV_HEADER, MixedStreamParser
from mmwave.dataloader import DCA1000
from radar_modes import expected_window_seconds, parse_config, validate_summary


class SessionCaptureError(RuntimeError):
    pass


@dataclass
class ImuProbeResult:
    port: str
    baudrate: int
    hits: int


def host_time_snapshot() -> dict[str, Any]:
    wall_time_s = time.time()
    local_text = time.strftime("%Y-%m-%d %H:%M:%S")
    utc_text = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(wall_time_s))
    suspicious = wall_time_s < 1_704_067_200  # 2024-01-01T00:00:00Z
    warning = (
        "System wall clock is earlier than 2024-01-01 UTC; check RTC/NTP before field capture."
        if suspicious
        else ""
    )
    return {
        "wall_time_s": wall_time_s,
        "wall_time_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "local": local_text,
        "utc": utc_text,
        "suspicious": suspicious,
        "warning": warning,
    }


def available_serial_ports() -> list[dict[str, str]]:
    if list_ports is None:
        return []
    result: list[dict[str, str]] = []
    for port in sorted(list_ports.comports(), key=lambda item: item.device):
        result.append(
            {
                "device": port.device,
                "description": port.description or "",
                "hwid": port.hwid or "",
                "manufacturer": port.manufacturer or "",
                "product": port.product or "",
                "serial_number": port.serial_number or "",
            }
        )
    return result


class ImuRecorderThread(threading.Thread):
    def __init__(self, port: str, baudrate: int, csv_path: Path, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.csv_path = csv_path
        self.stop_event = stop_event
        self.sample_count = 0
        self.first_sample_monotonic_ns: int | None = None
        self.last_sample_monotonic_ns: int | None = None
        self.error: str | None = None

    def run(self) -> None:  # pragma: no cover
        if serial is None:
            self.error = "pyserial is not available"
            return

        parser = MixedStreamParser()
        try:
            with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(IMU_CSV_HEADER)

                with serial.Serial(self.port, self.baudrate, timeout=0.05) as conn:
                    try:
                        conn.reset_input_buffer()
                    except Exception:
                        pass

                    while not self.stop_event.is_set():
                        data = conn.read(256)
                        if not data:
                            continue
                        wall_time_ns = time.time_ns()
                        monotonic_ns = time.monotonic_ns()
                        for sample in parser.feed(
                            data,
                            wall_time_ns=wall_time_ns,
                            monotonic_ns=monotonic_ns,
                        ):
                            writer.writerow(sample.as_csv_row())
                            csv_file.flush()
                            self.sample_count += 1
                            if self.first_sample_monotonic_ns is None:
                                self.first_sample_monotonic_ns = sample.host_monotonic_ns
                            self.last_sample_monotonic_ns = sample.host_monotonic_ns
        except Exception as exc:
            self.error = str(exc)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture an AWR2243 session plus HWT906P IMU samples with host-side timestamp estimates."
    )
    parser.add_argument("--radar-config", type=Path, default=DEFAULT_RADAR_CONFIG)
    parser.add_argument("--dca-config", type=Path, default=DEFAULT_DCA_CONFIG)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--session-name", default="")
    parser.add_argument("--numframes", type=int, default=50)
    parser.add_argument("--start-at-wall-ns", type=int, default=None)
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
    parser.add_argument(
        "--emit-events",
        action="store_true",
        help="Print simple radar event records as soon as they occur.",
    )
    return parser.parse_args()


def sorted_ports() -> list[str]:
    if list_ports is None:
        return []
    ports = list(list_ports.comports())
    ports.sort(
        key=lambda port: (
            "/dev/ttyUSB" not in port.device,
            "/dev/ttyACM" not in port.device,
            port.device,
        )
    )
    return [port.device for port in ports]


def probe_imu_port(port: str, baudrate: int, probe_seconds: float) -> int:
    if serial is None:
        return 0

    parser = MixedStreamParser()
    try:
        with serial.Serial(port, baudrate, timeout=0.04) as conn:
            try:
                conn.reset_input_buffer()
            except Exception:
                pass
            deadline = time.monotonic() + probe_seconds
            hits = 0
            while time.monotonic() < deadline:
                data = conn.read(256)
                if not data:
                    continue
                hits += len(
                    parser.feed(
                        data,
                        wall_time_ns=time.time_ns(),
                        monotonic_ns=time.monotonic_ns(),
                    )
                )
                if hits >= 2:
                    return hits
            return hits
    except Exception:
        return 0


def detect_imu_port(explicit_port: str, explicit_baud: int, probe_seconds: float) -> ImuProbeResult | None:
    if explicit_port.lower() == "none":
        return None
    if serial is None or list_ports is None:
        raise SessionCaptureError("pyserial is required for IMU capture")

    if explicit_port.lower() != "auto":
        baudrates = [explicit_baud] if explicit_baud else DEFAULT_IMU_BAUDRATES
        for baudrate in baudrates:
            hits = probe_imu_port(explicit_port, baudrate, probe_seconds)
            if hits > 0:
                return ImuProbeResult(port=explicit_port, baudrate=baudrate, hits=hits)
        return None

    ports = sorted_ports()
    if not ports:
        return None

    baudrates = [explicit_baud] if explicit_baud else DEFAULT_IMU_BAUDRATES
    for port in ports:
        for baudrate in baudrates:
            hits = probe_imu_port(port, baudrate, probe_seconds)
            if hits > 0:
                return ImuProbeResult(port=port, baudrate=baudrate, hits=hits)
    return None


def start_imu_capture_process(
    port: str,
    baudrate: int,
    csv_path: Path,
    manifest_path: Path,
    log_path: Path,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(IMU_CAPTURE_HELPER),
        "--port",
        port,
        "--baudrate",
        str(baudrate),
        "--csv-path",
        str(csv_path),
        "--manifest-path",
        str(manifest_path),
    ]
    log_handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    return {
        "process": process,
        "log_handle": log_handle,
        "manifest_path": manifest_path,
        "log_path": log_path,
    }


def read_log_tail(path: Path, limit: int = 20) -> list[str]:
    if not path.exists():
        return []
    with path.open("r", encoding="utf-8", errors="replace") as log_file:
        return log_file.read().splitlines()[-limit:]


def stop_imu_capture_process(record: dict[str, Any] | None) -> dict[str, Any] | None:
    if not record:
        return None

    process = record.get("process")
    log_handle = record.get("log_handle")
    manifest_path = record.get("manifest_path")

    try:
        if process is not None and process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
            except Exception:
                process.terminate()
            try:
                process.wait(timeout=10)
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

    if isinstance(manifest_path, Path) and manifest_path.exists():
        with manifest_path.open("r", encoding="utf-8") as manifest_file:
            manifest = json.load(manifest_file)
        if isinstance(manifest, dict):
            manifest["log_tail"] = read_log_tail(Path(record["log_path"]))
            return manifest

    return {
        "status": "error",
        "sample_count": 0,
        "first_sample_monotonic_ns": None,
        "last_sample_monotonic_ns": None,
        "error": "IMU capture manifest was not produced.",
        "log_tail": read_log_tail(Path(record["log_path"])),
    }


def make_session_dir(output_root: Path, session_name: str) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base_name = session_name.strip() or time.strftime("%Y%m%d_%H%M%S")
    session_dir = output_root / base_name
    suffix = 1
    while session_dir.exists():
        session_dir = output_root / f"{base_name}_{suffix:02d}"
        suffix += 1
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def resolve_session_dir(output_root: Path, session_name: str, explicit_session_dir: Path | None) -> Path:
    if explicit_session_dir is None:
        return make_session_dir(output_root, session_name)

    session_dir = explicit_session_dir.expanduser()
    if session_dir.exists():
        if any(session_dir.iterdir()):
            raise SessionCaptureError(f"Session directory is not empty: {session_dir}")
    else:
        session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def event_record(name: str) -> dict[str, Any]:
    return {
        "name": name,
        "host_wall_time_ns": time.time_ns(),
        "host_monotonic_ns": time.monotonic_ns(),
    }


def restore_session_ownership(path: Path) -> None:
    sudo_uid = os.environ.get("SUDO_UID", "").strip()
    sudo_gid = os.environ.get("SUDO_GID", "").strip()
    if not sudo_uid or not sudo_gid:
        return

    try:
        uid = int(sudo_uid)
        gid = int(sudo_gid)
    except ValueError:
        return

    for root, dirs, files in os.walk(path):
        os.chown(root, uid, gid)
        for name in dirs:
            os.chown(os.path.join(root, name), uid, gid)
        for name in files:
            os.chown(os.path.join(root, name), uid, gid)


def _run_checked(command: list[str]) -> str:
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0:
        stderr = completed.stderr.strip()
        stdout = completed.stdout.strip()
        detail = stderr or stdout or f"command failed with exit code {completed.returncode}"
        raise SessionCaptureError(f"{' '.join(command)}: {detail}")
    return completed.stdout.strip()


def prepare_radar_interface(interface: str, cidr: str) -> dict[str, Any]:
    _run_checked(["ip", "link", "set", interface, "up"])
    _run_checked(["ip", "address", "replace", cidr, "dev", interface])
    addr_output = _run_checked(["ip", "-brief", "addr", "show", "dev", interface])
    return {
        "interface": interface,
        "configured_cidr": cidr,
        "addr_state": addr_output,
    }


def write_radar_frame_estimates(
    csv_path: Path,
    num_frames: int,
    configured_frame_period_ns: int,
    sensor_start_ns: int,
    sensor_stop_ns: int,
) -> None:
    total_config_span_ns = configured_frame_period_ns * num_frames
    measured_span_ns = max(sensor_stop_ns - sensor_start_ns, total_config_span_ns)
    measured_frame_period_ns = measured_span_ns // max(num_frames, 1)

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "frame_index",
                "config_start_monotonic_ns",
                "config_end_monotonic_ns",
                "window_start_monotonic_ns",
                "window_end_monotonic_ns",
                "configured_frame_period_ns",
                "window_frame_period_ns",
            ]
        )
        for frame_index in range(num_frames):
            config_start = sensor_start_ns + frame_index * configured_frame_period_ns
            window_start = sensor_start_ns + frame_index * measured_frame_period_ns
            writer.writerow(
                [
                    frame_index,
                    config_start,
                    config_start + configured_frame_period_ns,
                    window_start,
                    window_start + measured_frame_period_ns,
                    configured_frame_period_ns,
                    measured_frame_period_ns,
                ]
            )


def write_hardware_frame_estimates(
    csv_path: Path, num_frames: int, first_packet_receive_ns: int,
    trigger_period_ns: int, configured_frame_period_ns: int,
) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "frame_index", "first_packet_receive_monotonic_ns",
            "nominal_trigger_offset_ns", "packet_receive_proxy_monotonic_ns",
            "configured_frame_period_ns", "external_trigger_period_ns",
            "timestamp_note",
        ])
        for frame_index in range(num_frames):
            offset_ns = frame_index * trigger_period_ns
            writer.writerow([
                frame_index, first_packet_receive_ns, offset_ns,
                first_packet_receive_ns + offset_ns, configured_frame_period_ns,
                trigger_period_ns, "Host UDP receive proxy; not a measured STM32 trigger edge",
            ])


def ensure_root_if_needed() -> None:
    if os.name == "posix" and hasattr(os, "geteuid") and os.geteuid() != 0:
        raise SessionCaptureError(
            "AWR2243 initialization on Linux usually requires root because the FTDI/SPI path is used."
        )


def require_success(step: str, result: Any) -> None:
    if isinstance(result, bool):
        if not result:
            raise SessionCaptureError(f"{step} failed")
        return
    if isinstance(result, int):
        if result != 0:
            raise SessionCaptureError(f"{step} failed with code {result}")
        return
    if result is None:
        return


def analyze_raw_buffer_quality(
    data_buf: Any,
    *,
    expected_frames: int,
    frame_bytes: int,
    max_samples_per_frame: int = 16384,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Detect fixed-size captures whose trailing frames were never filled with real UDP data."""
    expected_frames = int(expected_frames)
    frame_bytes = int(frame_bytes)
    actual_bytes = int(getattr(data_buf, "nbytes", 0))
    expected_bytes = expected_frames * frame_bytes
    bytes_per_int16 = 2
    ints_per_frame = frame_bytes // bytes_per_int16 if frame_bytes > 0 else 0
    actual_complete_frames = actual_bytes // frame_bytes if frame_bytes > 0 else 0
    incomplete_tail_bytes = actual_bytes % frame_bytes if frame_bytes > 0 else actual_bytes

    summary: dict[str, Any] = {
        "quality_version": 1,
        "expected_frames": expected_frames,
        "frame_bytes": frame_bytes,
        "expected_raw_bytes": expected_bytes,
        "actual_raw_bytes": actual_bytes,
        "raw_size_matches": actual_bytes == expected_bytes,
        "actual_complete_frames_by_size": int(actual_complete_frames),
        "incomplete_tail_bytes": int(incomplete_tail_bytes),
        "sampled_int16_per_frame": 0,
        "baseline_mean_abs": None,
        "valid_mean_abs_threshold": None,
        "valid_nonzero_fraction_threshold": 0.001,
        "usable_radar_frames_by_content": 0,
        "last_valid_frame_index": None,
        "invalid_tail_frames": expected_frames,
        "zero_tail_detected": False,
        "capture_quality_status": "failed",
        "warnings": [],
    }
    frame_rows: list[dict[str, Any]] = []

    if expected_frames <= 0 or frame_bytes <= 0 or ints_per_frame <= 0:
        summary["warnings"].append("Invalid expected frame geometry; cannot check raw content quality.")
        return summary, frame_rows
    if actual_bytes <= 0:
        summary["warnings"].append("Raw buffer is empty.")
        return summary, frame_rows

    byte_view = np.asarray(data_buf).view(np.uint8).reshape(-1)
    complete_bytes = int(min(actual_complete_frames, expected_frames) * frame_bytes)
    if complete_bytes <= 0:
        summary["warnings"].append("Raw buffer does not contain one complete frame.")
        return summary, frame_rows

    int16_view = byte_view[:complete_bytes].view(np.int16)
    stride = max(1, ints_per_frame // max(1, int(max_samples_per_frame)))
    sampled_count = int((ints_per_frame + stride - 1) // stride)
    summary["sampled_int16_per_frame"] = sampled_count

    means: list[float] = []
    nonzero_fractions: list[float] = []
    for frame_index in range(int(min(actual_complete_frames, expected_frames))):
        start = frame_index * ints_per_frame
        stop = start + ints_per_frame
        sample = int16_view[start:stop:stride]
        if sample.size == 0:
            mean_abs = 0.0
            nonzero_fraction = 0.0
        else:
            mean_abs = float(np.mean(np.abs(sample.astype(np.int32))))
            nonzero_fraction = float(np.count_nonzero(sample) / sample.size)
        means.append(mean_abs)
        nonzero_fractions.append(nonzero_fraction)

    baseline_candidates = [value for value in means[: min(20, len(means))] if value > 0.0]
    baseline = float(np.median(baseline_candidates)) if baseline_candidates else 0.0
    mean_abs_threshold = max(4.0, baseline * 0.03)
    nonzero_threshold = float(summary["valid_nonzero_fraction_threshold"])
    summary["baseline_mean_abs"] = baseline
    summary["valid_mean_abs_threshold"] = mean_abs_threshold

    last_valid_frame_index: int | None = None
    invalid_frame_count = 0
    for frame_index, (mean_abs, nonzero_fraction) in enumerate(zip(means, nonzero_fractions)):
        valid = mean_abs >= mean_abs_threshold and nonzero_fraction >= nonzero_threshold
        if valid:
            last_valid_frame_index = frame_index
        else:
            invalid_frame_count += 1
        frame_rows.append(
            {
                "frame_index": frame_index,
                "mean_abs_sample": f"{mean_abs:.6f}",
                "nonzero_fraction_sample": f"{nonzero_fraction:.8f}",
                "valid_by_content": int(valid),
            }
        )

    usable_frames = 0 if last_valid_frame_index is None else last_valid_frame_index + 1
    invalid_tail_frames = expected_frames - usable_frames
    tail_threshold_frames = max(3, int(np.ceil(expected_frames * 0.01)))
    zero_tail_detected = invalid_tail_frames >= tail_threshold_frames

    summary.update(
        {
            "usable_radar_frames_by_content": int(usable_frames),
            "last_valid_frame_index": last_valid_frame_index,
            "invalid_frame_count_in_complete_region": int(invalid_frame_count),
            "invalid_tail_frames": int(invalid_tail_frames),
            "zero_tail_threshold_frames": int(tail_threshold_frames),
            "zero_tail_detected": bool(zero_tail_detected),
        }
    )
    if not summary["raw_size_matches"]:
        summary["warnings"].append(
            f"Raw size mismatch: actual={actual_bytes}, expected={expected_bytes}, "
            f"difference={actual_bytes - expected_bytes}."
        )
    if zero_tail_detected:
        summary["warnings"].append(
            f"Detected {invalid_tail_frames} missing/empty trailing radar frames by content scan."
        )
    elif invalid_frame_count:
        summary["warnings"].append(
            f"Detected {invalid_frame_count} low-energy frames in sampled content; inspect radar_frame_quality.csv."
        )

    if summary["raw_size_matches"] and not zero_tail_detected:
        summary["capture_quality_status"] = "warning" if invalid_frame_count else "ok"
    else:
        summary["capture_quality_status"] = "failed"
    return summary, frame_rows


def write_raw_quality_files(
    summary_path: Path,
    frame_csv_path: Path,
    summary: dict[str, Any],
    frame_rows: list[dict[str, Any]],
) -> None:
    with summary_path.open("w", encoding="utf-8") as summary_file:
        json.dump(summary, summary_file, indent=2)
    with frame_csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        fieldnames = ["frame_index", "mean_abs_sample", "nonzero_fraction_sample", "valid_by_content"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(frame_rows)


def max_safe_single_capture_bytes() -> int:
    raw_value = os.environ.get("RADAR_MAX_SINGLE_CAPTURE_RAW_BYTES", "4000000000").strip()
    try:
        return int(raw_value)
    except ValueError:
        return 4_000_000_000


def allow_large_single_capture() -> bool:
    return os.environ.get("RADAR_ALLOW_GT4GB_RAW", "").strip().lower() in {"1", "true", "yes", "y"}


def radar_stream_chunk_bytes() -> int:
    raw_value = os.environ.get("RADAR_STREAM_CHUNK_BYTES", str(64 * 1024 * 1024)).strip()
    try:
        return max(1, int(raw_value))
    except ValueError:
        return 64 * 1024 * 1024


def radar_stream_chunk_frames(frame_bytes: int) -> int:
    if frame_bytes <= 0:
        return 1
    return max(1, radar_stream_chunk_bytes() // int(frame_bytes))


def radar_stream_buffer_frames(chunk_frames: int, total_frames: int) -> int:
    if total_frames <= 0:
        return max(1, chunk_frames)
    return min(int(total_frames), max(int(chunk_frames) * 4, int(chunk_frames) + 8))


def radar_reader_mode() -> str:
    mode = os.environ.get("RADAR_READER_MODE", "thread").strip().lower()
    if mode not in {"framefile2", "framefile", "file", "stream", "thread", "async", "blocking"}:
        return "thread"
    return mode


def apply_file_reader_quality_thresholds(
    quality_summary: dict[str, Any],
    file_reader_stats: dict[str, Any],
    *,
    expected_raw_bytes: int,
) -> None:
    if not file_reader_stats:
        return
    copied_raw_bytes = int(file_reader_stats.get("copied_raw_bytes") or expected_raw_bytes)
    zero_filled_bytes = max(0, int(expected_raw_bytes) - copied_raw_bytes)
    zero_fill_ratio = (zero_filled_bytes / float(expected_raw_bytes)) if expected_raw_bytes > 0 else 0.0
    warn_ratio = float(os.environ.get("RADAR_ZERO_FILL_WARN_RATIO", "0.0001"))
    fail_ratio = float(os.environ.get("RADAR_ZERO_FILL_FAIL_RATIO", "0.001"))
    file_reader_stats["zero_filled_bytes"] = zero_filled_bytes
    file_reader_stats["zero_fill_ratio"] = zero_fill_ratio
    file_reader_stats["zero_fill_warn_ratio"] = warn_ratio
    file_reader_stats["zero_fill_fail_ratio"] = fail_ratio
    quality_summary["file_reader_zero_filled_bytes"] = zero_filled_bytes
    quality_summary["file_reader_zero_fill_ratio"] = zero_fill_ratio
    if zero_filled_bytes <= 0:
        return
    warnings = quality_summary.setdefault("warnings", [])
    message = (
        "Framefile reader zero-filled missing UDP payload bytes: "
        f"{zero_filled_bytes} bytes ({zero_fill_ratio:.6%})."
    )
    warnings.append(message)
    if zero_fill_ratio >= fail_ratio:
        quality_summary["capture_quality_status"] = "failed"
    elif quality_summary.get("capture_quality_status") == "ok" and zero_fill_ratio >= warn_ratio:
        quality_summary["capture_quality_status"] = "warning"


def capture_radar(
    radar_config: Path,
    dca_config: Path,
    num_frames: int,
    static_ip: str,
    adc_ip: str,
    raw_bin_path: Path,
    frame_estimates_path: Path,
    event_callback: Callable[[dict[str, Any]], None] | None = None,
    reuse_initialized: bool = False,
    start_at_wall_ns: int | None = None,
) -> dict[str, Any]:
    if num_frames <= 0:
        raise SessionCaptureError("The first recorder version only supports a fixed positive --numframes value.")

    mode_summary = parse_config(radar_config)
    validate_summary(mode_summary)
    hardware_trigger = mode_summary.get("hardware_trigger")
    reader_mode = radar_reader_mode()
    if hardware_trigger and reader_mode != "framefile2":
        raise SessionCaptureError("Hardware trigger requires RADAR_READER_MODE=framefile2 for the UDP first-packet marker.")

    ensure_root_if_needed()

    events: list[dict[str, Any]] = []

    def record_event(name: str, **details: Any) -> dict[str, Any]:
        record = event_record(name)
        record.update(details)
        events.append(record)
        if event_callback is not None:
            event_callback(dict(record))
        return record

    def record_sensor_stop() -> dict[str, Any]:
        event = record_event("sensor_stop_detected")
        print(
            "RADAR_RECORDING_STOPPED "
            f"monotonic_ns={event['host_monotonic_ns']} expected_frames={num_frames_out} "
            f"expected_raw_bytes={expected_raw_bytes}",
            flush=True,
        )
        return event

    dca = DCA1000(static_ip=static_ip, adc_ip=adc_ip)
    udp_thread_started = False
    try:
        record_event("dca_created")
        if not reuse_initialized:
            dca.reset_radar()
            record_event("radar_reset_sent")
            dca.reset_fpga()
            record_event("fpga_reset_sent")
            time.sleep(1.0)

            record_event("radar_config_start")
            require_success("AWR2243_init", fpga_udp.AWR2243_init(str(radar_config)))
            record_event("awr2243_init_done")
        else:
            record_event("radar_config_reused")
        require_success("AWR2243_setFrameCfg", fpga_udp.AWR2243_setFrameCfg(num_frames))
        record_event("frame_cfg_set")

        lvds_bytes_per_chirp, max_send_bytes_per_chirp, adc_params, cfg_params = DCA1000.AWR2243_read_config_compat(
            str(radar_config)
        )
        expected_raw_bytes_before_start = int(num_frames) * int(adc_params["frame_bytes"])
        safe_limit_bytes = max_safe_single_capture_bytes()
        if expected_raw_bytes_before_start > safe_limit_bytes and not allow_large_single_capture():
            raise SessionCaptureError(
                "Requested radar raw size exceeds the safe single-capture limit: "
                f"expected={expected_raw_bytes_before_start} bytes, limit={safe_limit_bytes} bytes. "
                "The current DCA UDP buffer path has shown >4GB blank-tail/overflow risk. "
                "Use a shorter duration or split the scene into multiple captures. "
                "Set RADAR_ALLOW_GT4GB_RAW=1 only for explicit debugging."
            )
        dca.refresh_parameter()

        fpga_status = {
            "system_connect": dca.sys_alive_check(),
            "fpga_version": dca.read_fpga_version(),
            "config_fpga": dca.config_fpga(str(dca_config)),
            "config_record": dca.config_record(str(dca_config)),
        }
        record_event("dca_configured")

        target_monotonic_ns = None
        if start_at_wall_ns is not None:
            remaining_ns = int(start_at_wall_ns) - time.time_ns()
            if remaining_ns < 500_000_000:
                raise SessionCaptureError("Scheduled radar start is less than 0.5 seconds away after setup.")
            target_monotonic_ns = time.monotonic_ns() + remaining_ns
            record_event("scheduled_start_target", target_wall_time_ns=int(start_at_wall_ns), target_monotonic_ns=target_monotonic_ns)
            while time.monotonic_ns() < target_monotonic_ns - 7_000_000_000:
                time.sleep(min(0.05, (target_monotonic_ns - 7_000_000_000 - time.monotonic_ns()) / 1e9))

        dca.stream_start()
        record_event("stream_start_sent")
        if target_monotonic_ns is not None:
            while time.monotonic_ns() < target_monotonic_ns - 500_000_000:
                time.sleep(min(0.05, (target_monotonic_ns - 500_000_000 - time.monotonic_ns()) / 1e9))
        sort_in_c = True
        num_frames_out = int(num_frames)
        frame_bytes = int(adc_params["frame_bytes"])
        expected_raw_bytes = int(num_frames) * frame_bytes
        stream_chunks: list[dict[str, Any]] = []
        file_reader_stats: dict[str, Any] = {}
        raw_part_path = raw_bin_path.with_name(raw_bin_path.name + ".part")
        if reader_mode == "framefile2":
            if raw_bin_path.exists():
                raw_bin_path.unlink()
            if raw_part_path.exists():
                raw_part_path.unlink()
            if not hasattr(fpga_udp, "read_data_udp_frames_to_file2_async_start"):
                raise SessionCaptureError(
                    "RADAR_READER_MODE=framefile2 requires the experimental fpga_udp two-thread framefile extension."
                )
            if hardware_trigger and not all(hasattr(fpga_udp, name) for name in (
                "framefile2_arm_first_packet_marker", "framefile2_first_packet_seq",
                "framefile2_first_packet_monotonic_ns",
            )):
                raise SessionCaptureError("The loaded fpga_udp extension lacks hardware-trigger first-packet markers.")
            fpga_udp.read_data_udp_frames_to_file2_async_start(
                dca.data_socket.fileno(),
                num_frames_out,
                frame_bytes,
                1466,
                2,
                str(raw_bin_path),
                int(os.environ.get("RADAR_FRAMEFILE_CHUNK_FRAMES", "32")),
                int(os.environ.get("RADAR_FRAMEFILE_REORDER_MARGIN_FRAMES", "2")),
                int(os.environ.get("RADAR_FRAMEFILE_QUEUE_PACKETS", "131072")),
                int(os.environ.get("RADAR_SOCKET_RCVBUF_BYTES", "268435456")),
            )
            record_event("udp_framefile2_async_start")
            print(
                "RADAR_STATUS framefile2_reader_start "
                f"expected_frames={num_frames_out} expected_raw_bytes={expected_raw_bytes} "
                f"output={raw_bin_path}",
                flush=True,
            )
        elif reader_mode == "framefile":
            if raw_bin_path.exists():
                raw_bin_path.unlink()
            if raw_part_path.exists():
                raw_part_path.unlink()
            if not hasattr(fpga_udp, "read_data_udp_frames_to_file_async_start"):
                raise SessionCaptureError(
                    "RADAR_READER_MODE=framefile requires the experimental fpga_udp framefile extension."
                )
            fpga_udp.read_data_udp_frames_to_file_async_start(
                dca.data_socket.fileno(),
                num_frames_out,
                frame_bytes,
                1466,
                2,
                str(raw_bin_path),
                int(os.environ.get("RADAR_FRAMEFILE_CHUNK_FRAMES", "32")),
                int(os.environ.get("RADAR_FRAMEFILE_REORDER_MARGIN_FRAMES", "2")),
            )
            record_event("udp_framefile_async_start")
            print(
                "RADAR_STATUS framefile_reader_start "
                f"expected_frames={num_frames_out} expected_raw_bytes={expected_raw_bytes} "
                f"output={raw_bin_path}",
                flush=True,
            )
        elif reader_mode == "file":
            if raw_bin_path.exists():
                raw_bin_path.unlink()
            if raw_part_path.exists():
                raw_part_path.unlink()
            fpga_udp.read_data_udp_to_file_async_start(
                dca.data_socket.fileno(),
                num_frames_out,
                frame_bytes,
                1466,
                2,
                True,
                str(raw_bin_path),
            )
            record_event("udp_file_async_start")
            print(
                "RADAR_STATUS file_reader_start "
                f"expected_frames={num_frames_out} expected_raw_bytes={expected_raw_bytes} "
                f"output={raw_bin_path}",
                flush=True,
            )
        elif reader_mode == "stream":
            chunk_frames = radar_stream_chunk_frames(frame_bytes)
            buffer_frames = radar_stream_buffer_frames(chunk_frames, num_frames_out)
            if raw_part_path.exists():
                raw_part_path.unlink()
            dca.fastRead_in_Cpp_thread_start(buffer_frames)
            udp_thread_started = True
            record_event("udp_stream_thread_start")
            print(
                "RADAR_STATUS stream_reader_start "
                f"chunk_frames={chunk_frames} buffer_frames={buffer_frames} "
                f"expected_frames={num_frames_out} expected_raw_bytes={expected_raw_bytes}",
                flush=True,
            )
        elif reader_mode == "thread":
            dca.fastRead_in_Cpp_thread_start(num_frames_out)
            udp_thread_started = True
            record_event("udp_thread_start")
        elif reader_mode == "async":
            num_frames_out, sort_in_c = dca.fastRead_in_Cpp_async_start(num_frames, sortInC=True)
            record_event("udp_async_start")
        else:
            record_event("udp_blocking_reader_selected")

        if target_monotonic_ns is not None:
            if time.monotonic_ns() > target_monotonic_ns + 100_000_000:
                raise SessionCaptureError("Radar receiver missed the scheduled start by more than 100 ms.")
            while time.monotonic_ns() < target_monotonic_ns:
                time.sleep(min(0.005, (target_monotonic_ns - time.monotonic_ns()) / 1e9))
        if hardware_trigger:
            fpga_udp.framefile2_arm_first_packet_marker()
        sensor_start_event = record_event("sensor_start_sent")
        require_success("AWR2243_sensorStart", fpga_udp.AWR2243_sensorStart())
        sensor_start_confirmed_event = record_event("sensor_start_confirmed")
        first_packet_receive_ns = None
        if hardware_trigger:
            print(
                "RADAR_ARMED_WAITING_FOR_TRIGGER "
                f"confirmed_monotonic_ns={sensor_start_confirmed_event['host_monotonic_ns']} "
                f"expected_frames={num_frames_out}",
                flush=True,
            )
            wait_seconds = float(os.environ.get("RADAR_HARDWARE_TRIGGER_WAIT_SECONDS", "30"))
            deadline_ns = time.monotonic_ns() + int(wait_seconds * 1_000_000_000)
            first_seq = 0
            while time.monotonic_ns() < deadline_ns:
                first_seq = int(fpga_udp.framefile2_first_packet_seq())
                if first_seq:
                    first_packet_receive_ns = int(fpga_udp.framefile2_first_packet_monotonic_ns())
                    break
                time.sleep(0.005)
            if not first_seq or not first_packet_receive_ns:
                try:
                    fpga_udp.AWR2243_sensorStop()
                except Exception:
                    pass
                try:
                    fpga_udp.read_data_udp_to_file_async_wait()
                except Exception:
                    pass
                raise SessionCaptureError("No radar UDP packet arrived after hardware-trigger arming.")
            record_event("first_udp_packet_received", packet_sequence=first_seq,
                         udp_receive_monotonic_ns=first_packet_receive_ns)
        capture_span_seconds = (
            expected_window_seconds(mode_summary, num_frames_out) if hardware_trigger
            else num_frames_out * float(adc_params["frame_periodicity"]) / 1000.0
        )
        print(
            "RADAR_RECORDING_STARTED "
            f"start_command_monotonic_ns={sensor_start_event['host_monotonic_ns']} "
            f"confirmed_monotonic_ns={sensor_start_confirmed_event['host_monotonic_ns']} "
            f"first_packet_receive_monotonic_ns={first_packet_receive_ns or 0} "
            f"timestamp_kind={'host_udp_receive_proxy' if hardware_trigger else 'sensor_start_command'} "
            f"duration_seconds={capture_span_seconds:.6f} "
            f"expected_frames={num_frames_out} expected_raw_bytes={expected_raw_bytes} "
            f"reader_mode={reader_mode}",
            flush=True,
        )

        if reader_mode == "framefile2":
            require_success("AWR2243_waitSensorStop", fpga_udp.AWR2243_waitSensorStop())
            sensor_stop_event = record_sensor_stop()
            file_reader_stats = dict(fpga_udp.read_data_udp_to_file_async_wait())
            record_event("udp_framefile2_async_wait_done")
            before_write_event = record_event("recording_complete_before_raw_write")
            actual_raw_bytes = int(raw_bin_path.stat().st_size) if raw_bin_path.exists() else 0
            quality_source = np.memmap(raw_bin_path, dtype=np.uint8, mode="r")
            try:
                quality_summary, frame_quality_rows = analyze_raw_buffer_quality(
                    quality_source,
                    frame_bytes=frame_bytes,
                    expected_frames=num_frames_out,
                )
            finally:
                del quality_source
            apply_file_reader_quality_thresholds(
                quality_summary,
                file_reader_stats,
                expected_raw_bytes=expected_raw_bytes,
            )
        elif reader_mode == "framefile":
            require_success("AWR2243_waitSensorStop", fpga_udp.AWR2243_waitSensorStop())
            sensor_stop_event = record_sensor_stop()
            file_reader_stats = dict(fpga_udp.read_data_udp_to_file_async_wait())
            record_event("udp_framefile_async_wait_done")
            before_write_event = record_event("recording_complete_before_raw_write")
            actual_raw_bytes = int(raw_bin_path.stat().st_size) if raw_bin_path.exists() else 0
            quality_source = np.memmap(raw_bin_path, dtype=np.uint8, mode="r")
            try:
                quality_summary, frame_quality_rows = analyze_raw_buffer_quality(
                    quality_source,
                    expected_frames=int(num_frames),
                    frame_bytes=frame_bytes,
                )
            finally:
                del quality_source
            apply_file_reader_quality_thresholds(
                quality_summary,
                file_reader_stats,
                expected_raw_bytes=expected_raw_bytes,
            )
        elif reader_mode == "file":
            require_success("AWR2243_waitSensorStop", fpga_udp.AWR2243_waitSensorStop())
            sensor_stop_event = record_sensor_stop()
            file_reader_stats = dict(fpga_udp.read_data_udp_to_file_async_wait())
            record_event("udp_file_async_wait_done")
            before_write_event = record_event("recording_complete_before_raw_write")
            actual_raw_bytes = int(raw_bin_path.stat().st_size) if raw_bin_path.exists() else 0
            quality_source = np.memmap(raw_bin_path, dtype=np.uint8, mode="r")
            try:
                quality_summary, frame_quality_rows = analyze_raw_buffer_quality(
                    quality_source,
                    expected_frames=int(num_frames),
                    frame_bytes=frame_bytes,
                )
            finally:
                del quality_source
            apply_file_reader_quality_thresholds(
                quality_summary,
                file_reader_stats,
                expected_raw_bytes=expected_raw_bytes,
            )
        elif reader_mode == "stream":
            frames_remaining = int(num_frames_out)
            frames_written_requested = 0
            actual_raw_bytes = 0
            chunk_index = 0
            before_write_event: dict[str, Any] | None = None
            with raw_part_path.open("wb", buffering=1024 * 1024) as raw_part_file:
                while frames_remaining > 0:
                    this_frames = min(chunk_frames, frames_remaining)
                    chunk_data = dca.fastRead_in_Cpp_thread_get(numframes=this_frames, sortInC=sort_in_c)
                    chunk_array = np.asarray(chunk_data)
                    chunk_bytes = int(chunk_array.nbytes)
                    chunk_array.tofile(raw_part_file)
                    raw_part_file.flush()
                    actual_raw_bytes += chunk_bytes
                    frames_written_requested += int(this_frames)
                    frames_remaining -= int(this_frames)
                    chunk_event = record_event("stream_chunk_written")
                    chunk_record = {
                        "chunk_index": int(chunk_index),
                        "requested_frames": int(this_frames),
                        "bytes": int(chunk_bytes),
                        "total_requested_frames": int(frames_written_requested),
                        "total_bytes": int(actual_raw_bytes),
                        "host_monotonic_ns": int(chunk_event["host_monotonic_ns"]),
                    }
                    stream_chunks.append(chunk_record)
                    print(
                        "RADAR_STATUS stream_chunk "
                        f"index={chunk_index} frames={this_frames} bytes={chunk_bytes} "
                        f"total_frames={frames_written_requested}/{num_frames_out} "
                        f"total_bytes={actual_raw_bytes}/{expected_raw_bytes}",
                        flush=True,
                    )
                    chunk_index += 1
            require_success("AWR2243_waitSensorStop", fpga_udp.AWR2243_waitSensorStop())
            sensor_stop_event = record_sensor_stop()
            before_write_event = record_event("recording_complete_before_raw_write")
            try:
                dca.fastRead_in_Cpp_thread_stop()
            finally:
                udp_thread_started = False
            record_event("udp_stream_thread_stop")
            os.replace(raw_part_path, raw_bin_path)
            record_event("radar_raw_written")
            actual_raw_bytes = int(raw_bin_path.stat().st_size)
            quality_source = np.memmap(raw_bin_path, dtype=np.uint8, mode="r")
            try:
                quality_summary, frame_quality_rows = analyze_raw_buffer_quality(
                    quality_source,
                    expected_frames=int(num_frames),
                    frame_bytes=frame_bytes,
                )
            finally:
                del quality_source
        else:
            require_success("AWR2243_waitSensorStop", fpga_udp.AWR2243_waitSensorStop())
            sensor_stop_event = record_sensor_stop()

            if reader_mode == "thread":
                data_buf = dca.fastRead_in_Cpp_thread_get(numframes=num_frames_out, sortInC=sort_in_c)
                record_event("udp_thread_get_done")
                try:
                    dca.fastRead_in_Cpp_thread_stop()
                finally:
                    udp_thread_started = False
                record_event("udp_thread_stop")
            elif reader_mode == "async":
                data_buf = dca.fastRead_in_Cpp_async_wait(numframes=num_frames_out, sortInC=sort_in_c)
                record_event("udp_async_wait_done")
            else:
                data_buf = dca.fastRead_in_Cpp(numframes=num_frames_out, sortInC=sort_in_c)
                record_event("udp_blocking_read_done")
            before_write_event = record_event("recording_complete_before_raw_write")
            actual_raw_bytes = int(data_buf.nbytes)
            quality_summary, frame_quality_rows = analyze_raw_buffer_quality(
                data_buf,
                expected_frames=int(num_frames),
                frame_bytes=frame_bytes,
            )
            data_buf.tofile(raw_bin_path)
            record_event("radar_raw_written")
        if reader_mode in {"framefile2", "framefile", "file", "stream"}:
            raw_bytes_for_status = actual_raw_bytes
        else:
            raw_bytes_for_status = int(data_buf.nbytes)
        print(
            "RADAR_STATUS recording_complete_before_save "
            f"monotonic_ns={before_write_event['host_monotonic_ns']} "
            f"expected_frames={num_frames_out} raw_bytes_in_memory={0 if reader_mode in {'framefile2', 'file', 'stream'} else raw_bytes_for_status} "
            f"raw_bytes_on_disk={actual_raw_bytes} output={raw_bin_path}",
            flush=True,
        )
        print(
            "RADAR_STATUS raw_quality_before_save "
            f"status={quality_summary['capture_quality_status']} "
            f"usable_frames={quality_summary['usable_radar_frames_by_content']} "
            f"expected_frames={quality_summary['expected_frames']} "
            f"zero_tail_detected={quality_summary['zero_tail_detected']} "
            f"raw_size_matches={quality_summary['raw_size_matches']}",
            flush=True,
        )
        if quality_summary["capture_quality_status"] != "ok":
            print(
                "RADAR_WARNING raw_quality "
                + "; ".join(str(item) for item in quality_summary.get("warnings", [])),
                flush=True,
            )
        print(
            "RADAR_RAW_READY "
            f"monotonic_ns={record_event('radar_raw_ready')['host_monotonic_ns']} "
            f"raw_bytes={actual_raw_bytes} expected_raw_bytes={expected_raw_bytes} "
            f"raw_size_matches={quality_summary['raw_size_matches']} "
            f"quality={quality_summary['capture_quality_status']} output={raw_bin_path}",
            flush=True,
        )
        quality_summary_path = raw_bin_path.with_name("radar_quality_summary.json")
        frame_quality_path = raw_bin_path.with_name("radar_frame_quality.csv")
        write_raw_quality_files(quality_summary_path, frame_quality_path, quality_summary, frame_quality_rows)

        # frame_periodicity in the TI config is expressed in milliseconds
        configured_frame_period_ns = int(round(float(adc_params["frame_periodicity"]) * 1_000_000))
        if hardware_trigger:
            write_hardware_frame_estimates(
                frame_estimates_path, num_frames, int(first_packet_receive_ns),
                int(round(mode_summary["effective_frame_period_ms"] * 1_000_000)),
                configured_frame_period_ns,
            )
        else:
            write_radar_frame_estimates(
                frame_estimates_path,
                num_frames=num_frames,
                configured_frame_period_ns=configured_frame_period_ns,
                sensor_start_ns=sensor_start_event["host_monotonic_ns"],
                sensor_stop_ns=sensor_stop_event["host_monotonic_ns"],
            )

        packet_stats = {
            "expected_raw_bytes": expected_raw_bytes,
            "actual_raw_bytes": actual_raw_bytes,
            "raw_size_matches": actual_raw_bytes == expected_raw_bytes,
            "content_quality_status": quality_summary["capture_quality_status"],
            "usable_radar_frames_by_content": quality_summary["usable_radar_frames_by_content"],
            "zero_tail_detected": quality_summary["zero_tail_detected"],
            "file_reader_zero_filled_bytes": quality_summary.get("file_reader_zero_filled_bytes", 0),
            "file_reader_zero_fill_ratio": quality_summary.get("file_reader_zero_fill_ratio", 0.0),
            "packet_counter_note": (
                "Packet counter reads are skipped in single-TX HRRP capture because the DCA C extension "
                "can report stale counters or crash after valid fixed-size reads. Raw byte count plus "
                "per-frame content quality are used as the capture integrity check."
            ),
        }

        integrity_error = ""
        if actual_raw_bytes <= 0:
            integrity_error = (
                "Radar UDP capture returned an empty raw buffer. Check power, eno1 link state, and DCA/radar data path."
            )
        elif actual_raw_bytes != expected_raw_bytes:
            integrity_error = (
                "Radar raw byte count mismatch: "
                f"actual={actual_raw_bytes}, expected={expected_raw_bytes}, "
                f"difference={actual_raw_bytes - expected_raw_bytes}."
            )
        elif quality_summary["capture_quality_status"] == "failed":
            integrity_error = (
                "Radar raw content quality failed: "
                f"usable_frames={quality_summary['usable_radar_frames_by_content']}, "
                f"expected_frames={quality_summary['expected_frames']}, "
                f"zero_tail_detected={quality_summary['zero_tail_detected']}."
            )
        if integrity_error:
            packet_stats["integrity_error"] = integrity_error

        return {
            "model": "AWR2243",
            "radar_config": str(radar_config),
            "dca_config": str(dca_config),
            "num_frames": int(num_frames),
            "static_ip": static_ip,
            "adc_ip": adc_ip,
            "lvds_bytes_per_chirp": int(lvds_bytes_per_chirp),
            "max_send_bytes_per_chirp": int(max_send_bytes_per_chirp),
            "adc_params": adc_params,
            "cfg_params": cfg_params,
            "fpga_status": fpga_status,
            "packet_stats": packet_stats,
            "reader_mode": reader_mode,
            "file_reader_stats": file_reader_stats,
            "configured_frame_period_ns": configured_frame_period_ns,
            "effective_frame_period_ns": int(round(mode_summary["effective_frame_period_ms"] * 1_000_000)),
            "trigger_select": int(mode_summary["trigger_select"]),
            "hardware_trigger": hardware_trigger,
            "first_packet_receive_monotonic_ns": first_packet_receive_ns,
            "scheduled_start_wall_time_ns": start_at_wall_ns,
            "start_lateness_ms": None if start_at_wall_ns is None else (sensor_start_event["host_wall_time_ns"] - start_at_wall_ns) / 1_000_000,
            "raw_file": raw_bin_path.name,
            "raw_bytes": int(actual_raw_bytes),
            "frame_estimates_file": frame_estimates_path.name,
            "quality_summary_file": quality_summary_path.name,
            "frame_quality_file": frame_quality_path.name,
            "quality": quality_summary,
            "stream_chunks": stream_chunks,
            "integrity_error": integrity_error,
            "events": events,
        }
    finally:
        if udp_thread_started:
            try:
                dca.fastRead_in_Cpp_thread_stop()
            except Exception:
                pass
        try:
            if reuse_initialized:
                try:
                    dca.stream_stop()
                except Exception:
                    pass
            dca.close()
        finally:
            if not reuse_initialized:
                try:
                    fpga_udp.AWR2243_poweroff()
                except Exception:
                    pass


def main() -> int:
    args = parse_args()
    session_dir = resolve_session_dir(args.output_root, args.session_name, args.session_dir)
    imu_csv_path = session_dir / "imu_samples.csv"
    radar_raw_path = session_dir / "radar_raw.bin"
    radar_frames_path = session_dir / "radar_frames.csv"
    manifest_path = session_dir / "session_manifest.json"

    imu_process_record: dict[str, Any] | None = None
    imu_capture_manifest_path = session_dir / "imu_capture_manifest.json"
    imu_capture_log_path = session_dir / "imu_capture.log"
    imu_probe: ImuProbeResult | None = None
    radar_summary: dict[str, Any] | None = None
    network_summary: dict[str, Any] | None = None
    error_summary: dict[str, Any] | None = None
    exit_code = 0

    try:
        if not args.skip_network_setup:
            network_summary = prepare_radar_interface(args.radar_interface, args.radar_interface_cidr)
        imu_probe = detect_imu_port(args.imu_port, args.imu_baud, args.imu_probe_seconds)
        if imu_probe is None and not args.allow_no_imu and args.imu_port.lower() != "none":
            raise SessionCaptureError("No decodable IMU stream was detected. Use --allow-no-imu for radar-only testing.")

        if imu_probe is not None:
            imu_process_record = start_imu_capture_process(
                port=imu_probe.port,
                baudrate=imu_probe.baudrate,
                csv_path=imu_csv_path,
                manifest_path=imu_capture_manifest_path,
                log_path=imu_capture_log_path,
            )
            time.sleep(max(args.imu_warmup_seconds, 0.0))

        def emit_radar_event(record: dict[str, Any]) -> None:
            name = str(record.get("name") or "")
            mono_ns = int(record.get("host_monotonic_ns") or 0)
            wall_ns = int(record.get("host_wall_time_ns") or 0)
            print(f"RADAR_EVENT {name} {mono_ns} {wall_ns}", flush=True)

        capture_options = dict(
            radar_config=args.radar_config,
            dca_config=args.dca_config,
            num_frames=args.numframes,
            static_ip=args.dca_static_ip,
            adc_ip=args.dca_adc_ip,
            raw_bin_path=radar_raw_path,
            frame_estimates_path=radar_frames_path,
            start_at_wall_ns=args.start_at_wall_ns,
        )
        ready_socket = os.environ.get("RADAR_READY_SOCKET", "")
        if ready_socket:
            from radar_ready import request_capture
            deadline = time.monotonic() + 2.0
            while True:
                try:
                    radar_summary = request_capture(
                        ready_socket,
                        capture_options,
                        emit_radar_event if args.emit_events else None,
                    )
                    break
                except (FileNotFoundError, ConnectionRefusedError):
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.1)
        else:
            radar_summary = capture_radar(
                **capture_options,
                event_callback=emit_radar_event if args.emit_events else None,
            )
        integrity_error = str(radar_summary.get("integrity_error") or "")
        if integrity_error:
            error_summary = {
                "type": "SessionCaptureError",
                "message": integrity_error,
            }
            exit_code = 2
    except Exception as exc:
        traceback.print_exc()
        error_summary = {
            "type": exc.__class__.__name__,
            "message": str(exc),
        }
        exit_code = 2
    finally:
        imu_result = stop_imu_capture_process(imu_process_record)

    imu_summary = {
        "enabled": imu_probe is not None,
        "port": imu_probe.port if imu_probe is not None else "",
        "baudrate": imu_probe.baudrate if imu_probe is not None else 0,
        "probe_hits": imu_probe.hits if imu_probe is not None else 0,
        "csv_file": imu_csv_path.name if imu_probe is not None else "",
        "sample_count": int(imu_result.get("sample_count") or 0) if isinstance(imu_result, dict) else 0,
        "first_sample_monotonic_ns": (
            imu_result.get("first_sample_monotonic_ns") if isinstance(imu_result, dict) else None
        ),
        "last_sample_monotonic_ns": (
            imu_result.get("last_sample_monotonic_ns") if isinstance(imu_result, dict) else None
        ),
        "error": imu_result.get("error") if isinstance(imu_result, dict) else None,
    }
    if imu_probe is not None and error_summary is None and imu_summary["sample_count"] == 0 and not args.allow_no_imu:
        error_summary = {
            "type": "SessionCaptureError",
            "message": imu_summary["error"] or "No IMU samples were recorded during the session.",
        }
        exit_code = 2

    manifest = {
        "session_name": session_dir.name,
        "created_wall_time_ns": time.time_ns(),
        "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host_time": host_time_snapshot(),
        "status": "failed" if error_summary is not None else "ok",
        "radar": radar_summary,
        "imu": imu_summary,
        "network": network_summary,
        "error": error_summary,
        "alignment_strategy": {
            "type": "stm32_external_frame_trigger" if (radar_summary or {}).get("hardware_trigger") else "host_side_monotonic_ns",
            "notes": (
                [
                    "A/B radar triggers are staggered by 1250 us for chirp TDM.",
                    "Recorded first-packet time is the NX host UDP receive time, not the STM32 trigger edge.",
                    "A measured trigger-edge timestamp is needed for absolute radar/robot alignment.",
                ] if (radar_summary or {}).get("hardware_trigger") else [
                    "Radar frame timestamps are estimates derived from host monotonic time and configured frame periodicity.",
                ]
            ),
        },
        "artifacts": {
            "manifest": manifest_path.name,
            "radar_raw": radar_raw_path.name,
            "radar_frames": radar_frames_path.name,
            "radar_quality_summary": "radar_quality_summary.json" if radar_summary is not None else "",
            "radar_frame_quality": "radar_frame_quality.csv" if radar_summary is not None else "",
            "imu_samples": imu_csv_path.name if imu_probe is not None else "",
            "imu_capture_manifest": imu_capture_manifest_path.name if imu_probe is not None else "",
            "imu_capture_log": imu_capture_log_path.name if imu_probe is not None else "",
        },
    }

    with manifest_path.open("w", encoding="utf-8") as manifest_file:
        json.dump(manifest, manifest_file, indent=2)
    restore_session_ownership(session_dir)

    print(f"Session saved to: {session_dir}")
    quality = (radar_summary or {}).get("quality") or {}
    if quality:
        print(
            "Radar content quality: "
            f"{quality.get('capture_quality_status')} "
            f"usable_frames={quality.get('usable_radar_frames_by_content')}/"
            f"{quality.get('expected_frames')} "
            f"zero_tail={quality.get('zero_tail_detected')}"
        )
    if error_summary is not None:
        print(f"Capture failed: {error_summary['type']}: {error_summary['message']}")
        return exit_code
    if imu_probe is not None:
        print(f"IMU: {imu_probe.port} @ {imu_probe.baudrate}, samples={imu_summary['sample_count']}")
    else:
        print("IMU: disabled")
    print(f"Radar raw bytes: {radar_summary['raw_bytes']}")
    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except SessionCaptureError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
