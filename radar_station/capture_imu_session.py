from __future__ import annotations

import argparse
import csv
import json
import signal
import sys
import time
from pathlib import Path
from typing import Any

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

from imu_hwt906p import IMU_CSV_HEADER, MixedStreamParser


STOP_REQUESTED = False


def request_stop(_signum: int, _frame: object) -> None:  # pragma: no cover
    global STOP_REQUESTED
    STOP_REQUESTED = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record one serial IMU stream into CSV until SIGINT/SIGTERM."
    )
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, required=True)
    parser.add_argument("--csv-path", type=Path, required=True)
    parser.add_argument("--manifest-path", type=Path, required=True)
    return parser.parse_args()


def write_manifest(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as manifest_file:
        json.dump(payload, manifest_file, indent=2)


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    csv_path = args.csv_path.expanduser()
    manifest_path = args.manifest_path.expanduser()
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)

    if serial is None:
        manifest = {
            "status": "error",
            "port": args.port,
            "baudrate": args.baudrate,
            "csv_file": csv_path.name,
            "sample_count": 0,
            "first_sample_monotonic_ns": None,
            "last_sample_monotonic_ns": None,
            "error": "pyserial is not available",
        }
        write_manifest(manifest_path, manifest)
        print(json.dumps(manifest, indent=2))
        return 2

    parser = MixedStreamParser()
    sample_count = 0
    first_sample_monotonic_ns: int | None = None
    last_sample_monotonic_ns: int | None = None
    error: str | None = None

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(IMU_CSV_HEADER)

            with serial.Serial(args.port, args.baudrate, timeout=0.05) as conn:
                try:
                    conn.reset_input_buffer()
                except Exception:
                    pass

                while not STOP_REQUESTED:
                    data = conn.read(256)
                    if not data:
                        continue
                    wall_time_ns = time.time_ns()
                    monotonic_ns = time.monotonic_ns()
                    samples = parser.feed(
                        data,
                        wall_time_ns=wall_time_ns,
                        monotonic_ns=monotonic_ns,
                    )
                    if not samples:
                        continue
                    for sample in samples:
                        writer.writerow(sample.as_csv_row())
                        sample_count += 1
                        if first_sample_monotonic_ns is None:
                            first_sample_monotonic_ns = sample.host_monotonic_ns
                        last_sample_monotonic_ns = sample.host_monotonic_ns
                    csv_file.flush()
    except Exception as exc:
        error = str(exc)

    manifest = {
        "status": "ok" if sample_count > 0 and error is None else ("capture_failed" if error is None else "error"),
        "port": args.port,
        "baudrate": args.baudrate,
        "csv_file": csv_path.name,
        "sample_count": sample_count,
        "first_sample_monotonic_ns": first_sample_monotonic_ns,
        "last_sample_monotonic_ns": last_sample_monotonic_ns,
        "error": error,
    }
    write_manifest(manifest_path, manifest)
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["status"] == "ok" else 2


if __name__ == "__main__":
    raise SystemExit(main())
