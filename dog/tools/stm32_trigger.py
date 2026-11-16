from __future__ import annotations

import json
import os
import select
import subprocess
import threading
import time
from pathlib import Path
from typing import Any


class TriggerError(RuntimeError):
    pass


def frame_count(duration_seconds: float, frame_period_ms: float = 650.0,
                trigger_period_ms: float = 650.0) -> int:
    import math
    if not math.isfinite(duration_seconds) or duration_seconds <= 0:
        raise TriggerError("Capture duration must be positive and finite.")
    return max(1, math.ceil(max(0.0, duration_seconds * 1000 - frame_period_ms)
                            / trigger_period_ms) + 1)


class Stm32Trigger:
    def __init__(self, port: str, baudrate: int = 115200,
                 event_log_path: Path | None = None) -> None:
        self.port = str(Path(port).resolve(strict=True))
        self.baudrate = baudrate
        self.event_log_path = event_log_path
        self.fd: int | None = None
        self.events: list[dict[str, Any]] = []
        self._condition = threading.Condition()
        self._reader: threading.Thread | None = None
        self._running = False
        self._buffer = bytearray()

    def __enter__(self) -> "Stm32Trigger":
        import fcntl
        import termios
        import tty

        if self.baudrate != 115200:
            raise TriggerError("STM32 firmware requires 115200 baud.")
        subprocess.run(
            ["stty", "-F", self.port, "115200", "cs8", "-cstopb", "-parenb", "raw", "-echo"],
            check=True, capture_output=True, text=True, timeout=3.0,
        )
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            tty.setraw(self.fd)
            attrs = termios.tcgetattr(self.fd)
            attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
            attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            if hasattr(termios, "CRTSCTS"):
                attrs[2] &= ~termios.CRTSCTS
            attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[4] = attrs[5] = termios.B115200
            termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
            time.sleep(0.8)
            self._running = True
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()
            self._record("port_open", self.port)
            return self
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

    def __exit__(self, *_exc: object) -> None:
        self._running = False
        if self._reader is not None:
            self._reader.join(timeout=1.0)
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _record(self, kind: str, line: str) -> None:
        event = {"kind": kind, "line": line, "host_wall_time_ns": time.time_ns(),
                 "host_monotonic_ns": time.monotonic_ns()}
        with self._condition:
            self.events.append(event)
            if self.event_log_path is not None:
                self.event_log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.event_log_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(event) + "\n")
            self._condition.notify_all()

    def _read_loop(self) -> None:
        assert self.fd is not None
        while self._running:
            try:
                if not select.select([self.fd], [], [], 0.1)[0]:
                    continue
                chunk = os.read(self.fd, 256)
            except (OSError, ValueError) as exc:
                self._record("error", str(exc))
                return
            if not chunk:
                continue
            self._buffer.extend(chunk)
            while b"\n" in self._buffer:
                raw, _, remainder = self._buffer.partition(b"\n")
                self._buffer = bytearray(remainder)
                try:
                    line = raw.rstrip(b"\r").decode("ascii")
                except UnicodeDecodeError:
                    self._record("error", "Non-ASCII STM32 response")
                    continue
                if line:
                    self._record("rx", line)
            if len(self._buffer) > 256:
                self._record("error", "STM32 response line exceeded 256 bytes")
                self._buffer.clear()

    def _send(self, line: str) -> None:
        if self.fd is None:
            raise TriggerError("STM32 serial port is not open.")
        data = (line + "\n").encode("ascii")
        view = memoryview(data)
        while view:
            if not select.select([], [self.fd], [], 2.0)[1]:
                raise TriggerError("STM32 serial write timed out.")
            view = view[os.write(self.fd, view):]
        self._record("tx", line)

    def _wait_for(self, predicate: Any, timeout_seconds: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_seconds
        with self._condition:
            while True:
                for event in reversed(self.events):
                    result = predicate(event)
                    if result:
                        if result == "error":
                            raise TriggerError(f"STM32 rejected command: {event['line']}")
                        return event
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TriggerError("STM32 response timed out; start must not be retried blindly.")
                self._condition.wait(remaining)

    def start(self, count: int) -> dict[str, Any]:
        if not 1 <= count <= 0xFFFFFFFF:
            raise TriggerError("Trigger pair count is outside uint32 range.")
        with self._condition:
            self.events.clear()
        self._send(f"start {count}")
        return self._wait_for(
            lambda event: "error" if event["kind"] == "error" or
            (event["kind"] == "rx" and event["line"].startswith("ERR ")) else
            event["line"] == f"OK START {count}" and event["kind"] == "rx",
            3.0,
        )

    def confirm_idle(self) -> dict[str, Any]:
        ack = None
        for attempt in range(2):
            with self._condition:
                self.events.clear()
            self._send("stop")
            try:
                ack = self._wait_for(
                    lambda event: "error" if event["kind"] == "error" or
                    (event["kind"] == "rx" and event["line"].startswith("ERR ")) else
                    event["kind"] == "rx" and event["line"] in ("OK IDLE", "OK STOP"),
                    3.0,
                )
                break
            except TriggerError as exc:
                if attempt or "timed out" not in str(exc):
                    raise
                time.sleep(0.5)
        assert ack is not None
        if ack["line"] == "OK STOP":
            self._wait_for(lambda event: event["kind"] == "rx" and
                           event["line"].startswith("STOPPED "), 3.0)
        return ack

    def wait_done(self, count: int, timeout_seconds: float) -> dict[str, Any]:
        return self._wait_for(
            lambda event: "error" if event["kind"] == "error" or
            (event["kind"] == "rx" and event["line"].startswith(("ERR ", "STOPPED ", "READY"))) else
            event["line"] == f"DONE {count}" and event["kind"] == "rx",
            timeout_seconds,
        )

    def save(self, path: Path, count: int, status: str, error: str = "") -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"port": self.port, "baudrate": self.baudrate,
                                    "requested_pairs": count, "status": status,
                                    "error": error, "events": self.events}, indent=2) + "\n",
                        encoding="utf-8")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("probe",))
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--handshake", action="store_true",
                        help="Send stop and require OK IDLE/OK STOP; emits no new trigger pulses.")
    options = parser.parse_args()
    with Stm32Trigger(options.port) as device:
        try:
            ack = device.confirm_idle() if options.handshake else None
        except TriggerError as exc:
            print(json.dumps({"status": "error", "message": str(exc),
                              "events": device.events}))
            raise SystemExit(2)
        print(json.dumps({"status": "port_open", "port": device.port,
                          "baudrate": device.baudrate, "ack": ack,
                          "events": device.events}))
