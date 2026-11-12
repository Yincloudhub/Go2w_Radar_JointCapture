from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from typing import Any


IMU_CSV_HEADER = [
    "host_wall_time_ns",
    "host_monotonic_ns",
    "protocol",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "mx_raw",
    "my_raw",
    "mz_raw",
]


@dataclass
class ImuSample:
    host_wall_time_ns: int = field(default_factory=time.time_ns)
    host_monotonic_ns: int = field(default_factory=time.monotonic_ns)
    protocol: str = "unknown"
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0

    def stamp(self, wall_time_ns: int | None = None, monotonic_ns: int | None = None) -> None:
        if wall_time_ns is not None:
            self.host_wall_time_ns = int(wall_time_ns)
        if monotonic_ns is not None:
            self.host_monotonic_ns = int(monotonic_ns)

    def as_csv_row(self) -> list[float | int | str]:
        return [
            self.host_wall_time_ns,
            self.host_monotonic_ns,
            self.protocol,
            self.roll,
            self.pitch,
            self.yaw,
            self.ax,
            self.ay,
            self.az,
            self.gx,
            self.gy,
            self.gz,
            self.mx,
            self.my,
            self.mz,
        ]


class TextImuParser:
    KEY_ALIASES = {
        "roll": "roll",
        "r": "roll",
        "pitch": "pitch",
        "p": "pitch",
        "yaw": "yaw",
        "heading": "yaw",
        "h": "yaw",
        "ax": "ax",
        "accx": "ax",
        "acc_x": "ax",
        "ay": "ay",
        "accy": "ay",
        "acc_y": "ay",
        "az": "az",
        "accz": "az",
        "acc_z": "az",
        "gx": "gx",
        "gyrox": "gx",
        "gyro_x": "gx",
        "wx": "gx",
        "gy": "gy",
        "gyroy": "gy",
        "gyro_y": "gy",
        "wy": "gy",
        "gz": "gz",
        "gyroz": "gz",
        "gyro_z": "gz",
        "wz": "gz",
        "mx": "mx",
        "magx": "mx",
        "mag_x": "mx",
        "hx": "mx",
        "my": "my",
        "magy": "my",
        "mag_y": "my",
        "hy": "my",
        "mz": "mz",
        "magz": "mz",
        "mag_z": "mz",
        "hz": "mz",
    }
    PAIR_RE = re.compile(
        r"([A-Za-z_][A-Za-z0-9_]*)\s*[:=]\s*(-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)"
    )
    NUMBER_RE = re.compile(r"-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?")

    @classmethod
    def parse_line(cls, line: str) -> ImuSample | None:
        text = line.strip()
        if not text:
            return None
        for parser in (cls._parse_json, cls._parse_pairs, cls._parse_csv):
            sample = parser(text)
            if sample is not None:
                sample.protocol = "text"
                return sample
        return None

    @classmethod
    def _parse_json(cls, text: str) -> ImuSample | None:
        if not text.startswith("{"):
            return None
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return None
        if not isinstance(payload, dict):
            return None
        return cls._sample_from_mapping(payload)

    @classmethod
    def _parse_pairs(cls, text: str) -> ImuSample | None:
        matches = cls.PAIR_RE.findall(text)
        if not matches:
            return None
        payload: dict[str, Any] = {}
        for key, raw_value in matches:
            payload[key] = raw_value
        return cls._sample_from_mapping(payload)

    @classmethod
    def _parse_csv(cls, text: str) -> ImuSample | None:
        if "=" in text or ":" in text:
            return None
        values = [float(token) for token in cls.NUMBER_RE.findall(text)]
        if len(values) >= 12:
            ordered = values[-12:]
            return ImuSample(
                roll=ordered[0],
                pitch=ordered[1],
                yaw=ordered[2],
                ax=ordered[3],
                ay=ordered[4],
                az=ordered[5],
                gx=ordered[6],
                gy=ordered[7],
                gz=ordered[8],
                mx=ordered[9],
                my=ordered[10],
                mz=ordered[11],
            )
        if len(values) >= 9:
            ordered = values[-9:]
            return ImuSample(
                roll=ordered[0],
                pitch=ordered[1],
                yaw=ordered[2],
                ax=ordered[3],
                ay=ordered[4],
                az=ordered[5],
                gx=ordered[6],
                gy=ordered[7],
                gz=ordered[8],
            )
        return None

    @classmethod
    def _sample_from_mapping(cls, payload: dict[str, Any]) -> ImuSample | None:
        normalized: dict[str, float] = {}
        for key, raw_value in payload.items():
            alias = cls.KEY_ALIASES.get(str(key).strip().lower())
            if alias is None:
                continue
            try:
                normalized[alias] = float(raw_value)
            except (TypeError, ValueError):
                continue
        if not normalized:
            return None
        sample = ImuSample()
        for field_name in (
            "roll",
            "pitch",
            "yaw",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "mx",
            "my",
            "mz",
        ):
            if field_name in normalized:
                setattr(sample, field_name, normalized[field_name])
        return sample


class WitFrameDecoder:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.latest = ImuSample(protocol="wit")

    @staticmethod
    def _s16(lo: int, hi: int) -> int:
        value = (hi << 8) | lo
        if value >= 0x8000:
            value -= 0x10000
        return value

    def feed(
        self,
        data: bytes,
        wall_time_ns: int | None = None,
        monotonic_ns: int | None = None,
    ) -> list[ImuSample]:
        self.buffer.extend(data)
        samples: list[ImuSample] = []

        while len(self.buffer) >= 11:
            if self.buffer[0] != 0x55:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:11])
            checksum = sum(frame[:10]) & 0xFF
            if checksum != frame[10]:
                del self.buffer[0]
                continue
            del self.buffer[:11]

            frame_type = frame[1]
            payload = frame[2:10]
            sample = self._decode_frame(frame_type, payload, wall_time_ns, monotonic_ns)
            if sample is not None:
                samples.append(sample)

        return samples

    def _decode_frame(
        self,
        frame_type: int,
        payload: bytes,
        wall_time_ns: int | None = None,
        monotonic_ns: int | None = None,
    ) -> ImuSample | None:
        if frame_type == 0x51:
            self.latest.ax = self._s16(payload[0], payload[1]) / 32768.0 * 16.0
            self.latest.ay = self._s16(payload[2], payload[3]) / 32768.0 * 16.0
            self.latest.az = self._s16(payload[4], payload[5]) / 32768.0 * 16.0
            return None
        if frame_type == 0x52:
            self.latest.gx = self._s16(payload[0], payload[1]) / 32768.0 * 2000.0
            self.latest.gy = self._s16(payload[2], payload[3]) / 32768.0 * 2000.0
            self.latest.gz = self._s16(payload[4], payload[5]) / 32768.0 * 2000.0
            return None
        if frame_type == 0x53:
            self.latest.roll = self._s16(payload[0], payload[1]) / 32768.0 * 180.0
            self.latest.pitch = self._s16(payload[2], payload[3]) / 32768.0 * 180.0
            self.latest.yaw = self._s16(payload[4], payload[5]) / 32768.0 * 180.0
            return None
        if frame_type == 0x54:
            self.latest.mx = float(self._s16(payload[0], payload[1]))
            self.latest.my = float(self._s16(payload[2], payload[3]))
            self.latest.mz = float(self._s16(payload[4], payload[5]))
            sample = ImuSample(
                protocol="wit",
                roll=self.latest.roll,
                pitch=self.latest.pitch,
                yaw=self.latest.yaw,
                ax=self.latest.ax,
                ay=self.latest.ay,
                az=self.latest.az,
                gx=self.latest.gx,
                gy=self.latest.gy,
                gz=self.latest.gz,
                mx=self.latest.mx,
                my=self.latest.my,
                mz=self.latest.mz,
            )
            sample.stamp(wall_time_ns=wall_time_ns, monotonic_ns=monotonic_ns)
            return sample
        return None


class MixedStreamParser:
    def __init__(self) -> None:
        self.wit = WitFrameDecoder()
        self.line_buffer = bytearray()

    def feed(
        self,
        data: bytes,
        wall_time_ns: int | None = None,
        monotonic_ns: int | None = None,
    ) -> list[ImuSample]:
        samples = self.wit.feed(data, wall_time_ns=wall_time_ns, monotonic_ns=monotonic_ns)
        self.line_buffer.extend(data)
        while b"\n" in self.line_buffer:
            raw_line, _, rest = self.line_buffer.partition(b"\n")
            self.line_buffer = bytearray(rest)
            text = raw_line.decode("utf-8", errors="ignore")
            sample = TextImuParser.parse_line(text)
            if sample is not None:
                sample.stamp(wall_time_ns=wall_time_ns, monotonic_ns=monotonic_ns)
                samples.append(sample)
        if len(self.line_buffer) > 4096:
            self.line_buffer.clear()
        return samples
