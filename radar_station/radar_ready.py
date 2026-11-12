"""Keep the AWR initialized between finite, frame-aligned capture sessions."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Callable


DEFAULT_SOCKET = Path("/run/radar-ready.sock")
DEFAULT_LOG = Path(__file__).resolve().parent / "logs" / "radar-ready.log"


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def send_line(connection: socket.socket, payload: dict[str, Any]) -> None:
    connection.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8"))


def request(socket_path: str | Path, payload: dict[str, Any], on_message: Callable[[dict[str, Any]], None] | None = None) -> dict[str, Any]:
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(300)
        connection.connect(str(socket_path))
        send_line(connection, payload)
        with connection.makefile("r", encoding="utf-8") as stream:
            for line in stream:
                message = json.loads(line)
                if message.get("type") == "result":
                    return message["value"]
                if message.get("type") == "error":
                    raise RuntimeError(message.get("message") or "Radar ready service failed")
                if on_message:
                    on_message(message)
    raise RuntimeError("Radar ready service closed without a result")


def request_capture(socket_path: str, options: dict[str, Any], event_callback: Callable[[dict[str, Any]], None] | None = None) -> dict[str, Any]:
    payload = {"command": "capture", "options": {key: str(value) if isinstance(value, Path) else value for key, value in options.items()}}

    def forward(message: dict[str, Any]) -> None:
        if message.get("type") == "stdout":
            print(message.get("line", ""), flush=True)
        elif message.get("type") == "event" and event_callback:
            event_callback(message["record"])

    return request(socket_path, payload, forward)


class SocketWriter:
    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self.buffer = ""

    def write(self, text: str) -> int:
        self.buffer += text
        while "\n" in self.buffer:
            line, self.buffer = self.buffer.split("\n", 1)
            send_line(self.connection, {"type": "stdout", "line": line})
        return len(text)

    def flush(self) -> None:
        if self.buffer:
            send_line(self.connection, {"type": "stdout", "line": self.buffer})
            self.buffer = ""


def serve(socket_path: Path, radar_config: Path, dca_config: Path, static_ip: str, adc_ip: str) -> int:
    import fcntl

    os.environ["RADAR_READER_MODE"] = "framefile2"
    from capture_session_awr2243 import DCA1000, capture_radar, fpga_udp, require_success

    lock_path = socket_path.with_suffix(".lock")
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock_file = lock_path.open("a+")
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("Another radar ready service already owns this station") from None
    radar_config = radar_config.resolve(strict=True)
    dca_config = dca_config.resolve(strict=True)
    config_hash = file_hash(radar_config)
    dca_hash = file_hash(dca_config)
    dca = DCA1000(static_ip=static_ip, adc_ip=adc_ip)
    init_attempted = False
    listener: socket.socket | None = None
    try:
        dca.reset_radar()
        dca.reset_fpga()
        time.sleep(1)
        init_attempted = True
        require_success("AWR2243_init", fpga_udp.AWR2243_init(str(radar_config)))
        dca.config_fpga(str(dca_config))
        dca.config_record(str(dca_config))
        dca.close()

        socket_path.parent.mkdir(parents=True, exist_ok=True)
        socket_path.unlink(missing_ok=True)
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        listener.bind(str(socket_path))
        socket_path.chmod(0o600)
        listener.listen(1)
        print(f"RADAR_READY pid={os.getpid()} config={radar_config}", flush=True)
        while True:
            connection, _ = listener.accept()
            with connection:
                message: dict[str, Any] = {}
                try:
                    with connection.makefile("r", encoding="utf-8") as stream:
                        line = stream.readline()
                    if not line:
                        continue
                    message = json.loads(line)
                    command = message.get("command")
                    if command == "status":
                        send_line(connection, {"type": "result", "value": {"status": "ready", "pid": os.getpid(), "radar_config": str(radar_config), "radar_config_sha256": config_hash, "dca_config": str(dca_config), "dca_config_sha256": dca_hash}})
                    elif command == "stop":
                        send_line(connection, {"type": "result", "value": {"status": "stopping"}})
                        break
                    elif command == "capture":
                        options = message.get("options") or {}
                        if Path(options.get("radar_config", "")).resolve() != radar_config or file_hash(radar_config) != config_hash:
                            raise RuntimeError("Armed radar config differs from requested capture; disarm and arm the requested profile")
                        if Path(options.get("dca_config", "")).resolve() != dca_config or file_hash(dca_config) != dca_hash:
                            raise RuntimeError("Armed DCA config differs from requested capture")
                        if options.get("static_ip") != static_ip or options.get("adc_ip") != adc_ip:
                            raise RuntimeError("Armed DCA address differs from requested capture")
                        writer = SocketWriter(connection)
                        with contextlib.redirect_stdout(writer):
                            result = capture_radar(
                                radar_config=radar_config,
                                dca_config=dca_config,
                                num_frames=int(options["num_frames"]),
                                static_ip=static_ip,
                                adc_ip=adc_ip,
                                raw_bin_path=Path(options["raw_bin_path"]),
                                frame_estimates_path=Path(options["frame_estimates_path"]),
                                event_callback=lambda record: send_line(connection, {"type": "event", "record": record}),
                                reuse_initialized=True,
                                start_at_wall_ns=int(options["start_at_wall_ns"]) if options.get("start_at_wall_ns") is not None else None,
                            )
                        writer.flush()
                        send_line(connection, {"type": "result", "value": result})
                    else:
                        raise RuntimeError(f"Unknown radar ready command: {command}")
                except Exception as exc:
                    try:
                        send_line(connection, {"type": "error", "message": str(exc)})
                    except OSError:
                        pass
                    if message.get("command") == "capture":
                        print(f"RADAR_READY_ERROR {exc}", file=sys.stderr, flush=True)
                        break
    finally:
        if listener:
            listener.close()
        socket_path.unlink(missing_ok=True)
        try:
            dca.close()
        except Exception:
            pass
        if init_attempted:
            try:
                fpga_udp.AWR2243_sensorStop()
            except Exception:
                pass
            try:
                fpga_udp.AWR2243_poweroff()
            except Exception:
                pass
        lock_file.close()
    return 0


def arm(socket_path: Path, radar_config: Path, dca_config: Path, static_ip: str, adc_ip: str, log_path: Path) -> dict[str, Any]:
    radar_config = radar_config.resolve(strict=True)
    dca_config = dca_config.resolve(strict=True)
    if socket_path.exists():
        try:
            status = request(socket_path, {"command": "status"})
            if (status["radar_config"] != str(radar_config) or status["radar_config_sha256"] != file_hash(radar_config)
                    or status["dca_config"] != str(dca_config) or status["dca_config_sha256"] != file_hash(dca_config)):
                raise RuntimeError("A different radar or DCA configuration is already armed; disarm it first")
            return status
        except RuntimeError:
            raise
        except OSError:
            socket_path.unlink(missing_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    command = [sys.executable, str(Path(__file__).resolve()), "serve", "--socket", str(socket_path), "--radar-config", str(radar_config), "--dca-config", str(dca_config), "--static-ip", static_ip, "--adc-ip", adc_ip]
    environment = os.environ.copy()
    environment["RADAR_READER_MODE"] = "framefile2"
    with log_path.open("a", encoding="utf-8") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment)
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Radar ready service exited with {process.returncode}; inspect {log_path}")
        if socket_path.exists():
            try:
                return request(socket_path, {"command": "status"})
            except (OSError, RuntimeError):
                pass
        time.sleep(0.1)
    raise RuntimeError(f"Radar ready service did not become ready; inspect {log_path}")


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("serve", "arm"):
        sub = subparsers.add_parser(command)
        sub.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
        sub.add_argument("--radar-config", type=Path, required=True)
        sub.add_argument("--dca-config", type=Path, required=True)
        sub.add_argument("--static-ip", default="192.168.33.30")
        sub.add_argument("--adc-ip", default="192.168.33.180")
        if command == "arm":
            sub.add_argument("--log", type=Path, default=DEFAULT_LOG)
    for command in ("status", "stop"):
        sub = subparsers.add_parser(command)
        sub.add_argument("--socket", type=Path, default=DEFAULT_SOCKET)
    args = parser.parse_args()
    if args.command == "serve":
        return serve(args.socket, args.radar_config, args.dca_config, args.static_ip, args.adc_ip)
    if args.command == "arm":
        result = arm(args.socket, args.radar_config, args.dca_config, args.static_ip, args.adc_ip, args.log)
    else:
        result = request(args.socket, {"command": args.command})
    print(json.dumps(result, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
