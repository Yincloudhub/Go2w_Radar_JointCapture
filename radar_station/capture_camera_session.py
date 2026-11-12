from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import shutil
import subprocess
import time
from pathlib import Path


STOP_REQUESTED = False


def handle_stop_signal(_signum: int, _frame: object) -> None:  # pragma: no cover
    global STOP_REQUESTED
    STOP_REQUESTED = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record one camera stream as direct-to-disk video plus timestamp CSV."
    )
    parser.add_argument("--device", required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--video-path", type=Path, required=True)
    parser.add_argument("--csv-path", type=Path, required=True)
    parser.add_argument("--manifest-path", type=Path, required=True)
    parser.add_argument("--ready-path", type=Path, default=None)
    parser.add_argument("--save-every-n", type=int, default=1)
    return parser.parse_args()


def normalize_device(device: str) -> str:
    if device.isdigit():
        return f"/dev/video{device}"
    return device


def write_manifest(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as manifest_file:
        json.dump(payload, manifest_file, indent=2)


def parse_rate_text(raw_value: str | None) -> float | None:
    text = (raw_value or "").strip()
    if not text:
        return None
    if "/" in text:
        left, right = text.split("/", 1)
        try:
            numerator = float(left)
            denominator = float(right)
        except ValueError:
            return None
        if denominator == 0:
            return None
        return numerator / denominator
    try:
        return float(text)
    except ValueError:
        return None


def probe_video_with_ffprobe(video_path: Path) -> dict[str, object]:
    if shutil.which("ffprobe") is None:
        return {}
    completed = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-count_packets",
            "-show_entries",
            "stream=nb_read_packets,avg_frame_rate,r_frame_rate,width,height,duration",
            "-of",
            "json",
            str(video_path),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        return {}
    try:
        payload = json.loads(completed.stdout or "{}")
    except json.JSONDecodeError:
        return {}
    streams = payload.get("streams") or []
    if not streams:
        return {}
    stream = streams[0]
    return stream if isinstance(stream, dict) else {}


def write_frame_csv(
    csv_path: Path,
    frame_count: int,
    width: int,
    height: int,
    first_wall_time_ns: int | None,
    first_monotonic_ns: int | None,
    fps: float | None,
) -> tuple[int | None, int | None, int | None, float | None]:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if frame_count <= 0 or first_wall_time_ns is None or first_monotonic_ns is None:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["frame_index", "host_wall_time_ns", "host_monotonic_ns", "width", "height"])
        return None, None, None, None

    effective_fps = fps if fps and fps > 0 else None
    frame_interval_ns = (
        int(round(1_000_000_000 / effective_fps)) if effective_fps is not None else None
    )
    last_monotonic_ns = first_monotonic_ns

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["frame_index", "host_wall_time_ns", "host_monotonic_ns", "width", "height"])
        for frame_index in range(frame_count):
            if frame_interval_ns is None:
                host_wall_time_ns = first_wall_time_ns
                host_monotonic_ns = first_monotonic_ns
            else:
                host_wall_time_ns = first_wall_time_ns + frame_index * frame_interval_ns
                host_monotonic_ns = first_monotonic_ns + frame_index * frame_interval_ns
            last_monotonic_ns = host_monotonic_ns
            writer.writerow(
                [
                    frame_index + 1,
                    host_wall_time_ns,
                    host_monotonic_ns,
                    width,
                    height,
                ]
            )

    duration_ns = None
    actual_saved_fps = None
    if frame_count > 1 and frame_interval_ns is not None:
        duration_ns = (frame_count - 1) * frame_interval_ns
        actual_saved_fps = (frame_count - 1) / (duration_ns / 1_000_000_000)
    return first_monotonic_ns, last_monotonic_ns, duration_ns, actual_saved_fps


def ensure_direct_capture_supported(args: argparse.Namespace) -> None:
    if os.name != "posix":
        raise RuntimeError("Direct camera capture requires Linux/posix.")
    if shutil.which("gst-launch-1.0") is None:
        raise RuntimeError("gst-launch-1.0 is required for direct camera capture.")
    if shutil.which("ffprobe") is None:
        raise RuntimeError("ffprobe is required for camera timestamp generation.")
    if args.fourcc.strip().upper() != "MJPG":
        raise RuntimeError("Direct camera capture currently supports MJPG only.")
    if max(1, args.save_every_n) != 1:
        raise RuntimeError("Direct camera capture requires --save-every-n 1.")


def build_gstreamer_command(device: str, width: int, height: int, fps: int, video_path: Path) -> list[str]:
    return [
        "gst-launch-1.0",
        "-e",
        "v4l2src",
        f"device={device}",
        "io-mode=2",
        "!",
        f"image/jpeg,width={width},height={height},framerate={fps}/1",
        "!",
        "jpegparse",
        "!",
        "avimux",
        "!",
        "filesink",
        f"location={video_path}",
    ]


def run_direct_capture(args: argparse.Namespace, summary: dict[str, object]) -> int:
    from camera_gst_timestamps import record_camera
    return record_camera(args, summary, lambda: STOP_REQUESTED)


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGTERM, handle_stop_signal)
    signal.signal(signal.SIGINT, handle_stop_signal)

    args.video_path.parent.mkdir(parents=True, exist_ok=True)
    args.csv_path.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_path.parent.mkdir(parents=True, exist_ok=True)
    if args.ready_path is not None:
        args.ready_path.parent.mkdir(parents=True, exist_ok=True)

    normalized_device = normalize_device(args.device)
    summary: dict[str, object] = {
        "enabled": True,
        "device": normalized_device,
        "resolved_device": str(Path(normalized_device).resolve()) if Path(normalized_device).exists() else normalized_device,
        "backend": "",
        "opened": False,
        "requested_width": args.width,
        "requested_height": args.height,
        "requested_fps": args.fps,
        "requested_fourcc": args.fourcc,
        "actual_width": None,
        "actual_height": None,
        "actual_fps": None,
        "actual_fourcc": "",
        "saved_frames": 0,
        "first_frame_wall_time_ns": None,
        "first_frame_monotonic_ns": None,
        "first_saved_monotonic_ns": None,
        "last_saved_monotonic_ns": None,
        "saved_duration_ns": None,
        "actual_saved_fps": None,
        "read_failures": 0,
        "video_file": args.video_path.name,
        "frames_csv": args.csv_path.name,
        "single_frame_images_saved": False,
        "timestamp_strategy": "full_video_plus_synthesized_timestamp_csv",
        "error": None,
    }
    exit_code = 0

    try:
        exit_code = run_direct_capture(args, summary)
    except Exception as exc:
        summary["error"] = str(exc)
        exit_code = 2

    manifest = {
        "status": "ok" if exit_code == 0 else "error",
        "created_at_local": time.strftime("%Y-%m-%d %H:%M:%S"),
        "camera": summary,
    }
    write_manifest(args.manifest_path, manifest)
    print(json.dumps(manifest, indent=2))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
