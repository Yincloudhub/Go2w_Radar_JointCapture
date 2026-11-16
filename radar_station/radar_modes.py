from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
STATION_CONFIG = ROOT / "radar_station_config.json"
DEFAULT_BASE_CONFIG = ROOT / "configFiles" / "mmwaveconfig_0508.txt"
DEFAULT_MODE = "60f128"

IQ_COUNT = 2
ADC_BYTES = 2
DEFAULT_RX_COUNT = 4
DEFAULT_ADC_SAMPLES = 1024

SUPPORTED_MODES: dict[str, dict[str, int]] = {
    "30f128_safe": {
        "chirpStartIdx": 0,
        "chirpEndIdx": 0,
        "profileIdCPCFG": 0,
        "txEnable": 1,
        "chirpStartIdxFCF": 0,
        "chirpEndIdxFCF": 0,
        "loopCount": 128,
        "periodicity": 6666667,
    },
    "60f64_safe": {
        "chirpStartIdx": 0,
        "chirpEndIdx": 0,
        "profileIdCPCFG": 0,
        "txEnable": 1,
        "chirpStartIdxFCF": 0,
        "chirpEndIdxFCF": 0,
        "loopCount": 64,
        "periodicity": 3333333,
    },
    "60f128": {
        "chirpStartIdx": 0,
        "chirpEndIdx": 0,
        "profileIdCPCFG": 0,
        "txEnable": 1,
        "chirpStartIdxFCF": 0,
        "chirpEndIdxFCF": 0,
        "loopCount": 128,
        "periodicity": 3333333,
    },
    "30f256": {
        "chirpStartIdx": 0,
        "chirpEndIdx": 1,
        "profileIdCPCFG": 0,
        "txEnable": 1,
        "chirpStartIdxFCF": 0,
        "chirpEndIdxFCF": 1,
        "loopCount": 128,
        "periodicity": 6666667,
    },
    "15f512": {
        "chirpStartIdx": 0,
        "chirpEndIdx": 3,
        "profileIdCPCFG": 0,
        "txEnable": 1,
        "chirpStartIdxFCF": 0,
        "chirpEndIdxFCF": 3,
        "loopCount": 128,
        "periodicity": 13333333,
    },
}


def read_station_config() -> dict[str, Any]:
    with STATION_CONFIG.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_station_config(config: dict[str, Any]) -> None:
    with STATION_CONFIG.open("w", encoding="utf-8") as f:
        json.dump(config, f, ensure_ascii=False, indent=2)
        f.write("\n")


def get_value(text: str, key: str, default: int | None = None) -> int:
    match = re.search(rf"(?m)^\s*{re.escape(key)}\s*=\s*([-0-9]+)\s*;?", text)
    if not match:
        if default is not None:
            return default
        raise RuntimeError(f"Missing {key}=... in radar config.")
    return int(match.group(1))


def set_value(text: str, key: str, value: int) -> str:
    pattern = rf"(?m)^(\s*{re.escape(key)}\s*=\s*)[-0-9]+(\s*;?.*)$"
    replaced, count = re.subn(pattern, rf"\g<1>{value}\2", text, count=1)
    if count != 1:
        raise RuntimeError(f"Could not update {key}=... in radar config.")
    return replaced


def parse_chirp_blocks(text: str) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    for match in re.finditer(r"(?ms)^#\s*\n(?P<body>.*?^#END\s*$)", text):
        block_text = match.group(0)
        if not re.search(r"(?m)^\s*chirpStartIdx\s*=", block_text):
            continue
        fields = {
            "chirpStartIdx": get_value(block_text, "chirpStartIdx"),
            "chirpEndIdx": get_value(block_text, "chirpEndIdx"),
            "profileIdCPCFG": get_value(block_text, "profileIdCPCFG"),
            "txEnable": get_value(block_text, "txEnable"),
        }
        blocks.append(
            {
                "start": match.start(),
                "end": match.end(),
                "text": block_text,
                "fields": fields,
            }
        )
    return blocks


def clone_single_tx_chirp_block(template: str, chirp_index: int) -> str:
    block = set_value(template, "chirpStartIdx", chirp_index)
    block = set_value(block, "chirpEndIdx", chirp_index)
    block = set_value(block, "profileIdCPCFG", 0)
    block = set_value(block, "txEnable", 1)
    return block.rstrip() + "\n"


def force_single_tx_chirps(text: str, chirp_start: int, chirp_end: int) -> str:
    blocks = parse_chirp_blocks(text)
    if not blocks:
        raise RuntimeError("No chirp config blocks found; cannot build single-TX mode.")

    by_index: dict[int, dict[str, Any]] = {}
    for block in blocks:
        fields = block["fields"]
        if fields["chirpStartIdx"] == fields["chirpEndIdx"]:
            by_index[fields["chirpStartIdx"]] = block

    template = by_index.get(0, blocks[0])["text"]
    replacements: list[tuple[int, int, str]] = []
    missing: list[str] = []
    for chirp_index in range(chirp_start, chirp_end + 1):
        new_block = clone_single_tx_chirp_block(template, chirp_index)
        if chirp_index in by_index:
            block = by_index[chirp_index]
            replacements.append((block["start"], block["end"], new_block))
        else:
            missing.append(new_block)

    for start, end, new_block in sorted(replacements, reverse=True):
        text = text[:start] + new_block + text[end:]

    if missing:
        insert_after = max(block["end"] for block in blocks)
        text = text[:insert_after] + "\n" + "\n".join(missing) + text[insert_after:]
    return text


def selected_chirp_defs(text: str, chirp_start: int, chirp_end: int) -> list[dict[str, int]]:
    selected: list[dict[str, int]] = []
    for block in parse_chirp_blocks(text):
        fields = block["fields"]
        start = fields["chirpStartIdx"]
        end = fields["chirpEndIdx"]
        if start > chirp_end or end < chirp_start:
            continue
        selected.append(fields)
    return selected


def bit_count(value: int) -> int:
    return bin(max(0, int(value))).count("1")


def infer_mode_from_values(loop_count: int, chirp_start: int, chirp_end: int, periodicity: int) -> str:
    for mode, values in SUPPORTED_MODES.items():
        if (
            loop_count == values["loopCount"]
            and chirp_start == values["chirpStartIdxFCF"]
            and chirp_end == values["chirpEndIdxFCF"]
            and periodicity == values["periodicity"]
        ):
            return mode
    return "custom"


def expected_frames(duration_seconds: float, frame_period_ms: float,
                    external_trigger_period_ms: float | None = None) -> int:
    if external_trigger_period_ms is not None:
        remaining_ms = max(0.0, float(duration_seconds) * 1000.0 - float(frame_period_ms))
        return max(1, int(math.ceil(remaining_ms / float(external_trigger_period_ms))) + 1)
    return max(1, int(round((float(duration_seconds) * 1000.0) / frame_period_ms)))


def expected_raw_bytes(summary: dict[str, Any], frames: int) -> int:
    return (
        int(frames)
        * int(summary["chirps_per_frame"])
        * int(summary["rx_count"])
        * int(summary["adc_samples"])
        * IQ_COUNT
        * ADC_BYTES
    )


def expected_window_seconds(summary: dict[str, Any], frames: int) -> float:
    return (float(summary["frame_period_ms"]) +
            max(0, int(frames) - 1) * float(summary["effective_frame_period_ms"])) / 1000.0


def add_duration_expectations(summary: dict[str, Any], duration_seconds: float | None) -> dict[str, Any]:
    if duration_seconds is None:
        return summary
    hardware = summary.get("hardware_trigger") or {}
    frames = expected_frames(float(duration_seconds), float(summary["frame_period_ms"]),
                             hardware.get("external_trigger_period_ms"))
    summary["requested_duration_sec"] = float(duration_seconds)
    summary["expected_frames"] = frames
    summary["expected_raw_bytes"] = expected_raw_bytes(summary, frames)
    summary["expected_window_seconds"] = expected_window_seconds(summary, frames)
    return summary


def parse_config(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    idle_time_const = get_value(text, "idleTimeConst")
    ramp_end_time = get_value(text, "rampEndTime")
    loop_count = get_value(text, "loopCount")
    chirp_start = get_value(text, "chirpStartIdxFCF")
    chirp_end = get_value(text, "chirpEndIdxFCF")
    periodicity = get_value(text, "periodicity")
    trigger_select = get_value(text, "triggerSelect", default=1)
    frame_trigger_delay_us = get_value(text, "triggerDelay", default=0) * 0.005
    advanced_frame_trigger_delay_us = get_value(text, "frameTrigDelay", default=0) * 0.005
    hardware_trigger = None
    if trigger_select == 2:
        metadata_path = path.with_suffix(".json")
        if not metadata_path.is_file():
            raise RuntimeError(f"Hardware-trigger timing metadata is missing: {metadata_path}")
        hardware_trigger = json.loads(metadata_path.read_text(encoding="utf-8"))
    num_chirp_indexes = chirp_end - chirp_start + 1
    chirps_per_frame = num_chirp_indexes * loop_count
    chirp_period_us = (idle_time_const + ramp_end_time) * 0.01
    active_time_ms = chirps_per_frame * chirp_period_us / 1000.0
    frame_period_ms = periodicity * 5e-6
    effective_frame_period_ms = (
        float(hardware_trigger["external_trigger_period_ms"]) if hardware_trigger is not None else frame_period_ms
    )
    fps = 1000.0 / effective_frame_period_ms if effective_frame_period_ms > 0 else None
    duty_cycle = active_time_ms / effective_frame_period_ms if effective_frame_period_ms > 0 else None
    tdm_timing = None
    if hardware_trigger is not None:
        offsets = hardware_trigger["edge_offsets_us"]
        separation_us = float(offsets["B"]) - float(offsets["A"])
        last_b_end_us = (chirps_per_frame - 1) * chirp_period_us + separation_us + ramp_end_time * 0.01
        tdm_timing = {
            "a_to_b_us": separation_us,
            "b_to_next_a_us": chirp_period_us - separation_us,
            "last_b_end_from_a_frame_start_us": last_b_end_us,
            "next_a_frame_guard_us": effective_frame_period_ms * 1000.0 - last_b_end_us,
        }
    selected = selected_chirp_defs(text, chirp_start, chirp_end)
    rx_mask = get_value(text, "rxAntMask", default=0) or get_value(text, "channelRx", default=15)
    rx_chan_en = get_value(text, "rxChanEn", default=rx_mask)
    rx_count = get_value(text, "numRxChan", default=0) or bit_count(rx_mask)
    adc_fmt = get_value(text, "adcFmt", default=1)
    adc_samples = get_value(text, "numAdcSamples", DEFAULT_ADC_SAMPLES)
    sample_rate = get_value(text, "digOutSampleRate", default=22500)
    slope_const = get_value(text, "freqSlopeConst", default=1657)
    start_freq_const = get_value(text, "startFreqConst")
    inferred_mode = infer_mode_from_values(loop_count, chirp_start, chirp_end, periodicity)
    profile_ids = sorted({int(item["profileIdCPCFG"]) for item in selected})
    tx_enables = sorted({int(item["txEnable"]) for item in selected})
    return {
        "mode": inferred_mode,
        "config_path": str(path),
        "profile_id": profile_ids[0] if len(profile_ids) == 1 else profile_ids,
        "tx_mode": "TX0 only" if tx_enables == [1] else f"invalid txEnable values: {tx_enables}",
        "txEnable": tx_enables,
        "rx_count": rx_count,
        "rx_channel_mask": rx_mask,
        "rx_chan_en": rx_chan_en,
        "adc_fmt": adc_fmt,
        "adc_samples": adc_samples,
        "sample_rate_ksps": sample_rate,
        "freqSlopeConst": slope_const,
        "startFreqConst": start_freq_const,
        "loopCount": loop_count,
        "chirpStartIdxFCF": chirp_start,
        "chirpEndIdxFCF": chirp_end,
        "num_chirp_indexes": num_chirp_indexes,
        "chirps_per_frame": chirps_per_frame,
        "periodicity": periodicity,
        "idleTimeConst": idle_time_const,
        "rampEndTime": ramp_end_time,
        "chirp_period_us": chirp_period_us,
        "active_chirp_time_ms": active_time_ms,
        "active_time_ms": active_time_ms,
        "frame_period_ms": frame_period_ms,
        "effective_frame_period_ms": effective_frame_period_ms,
        "trigger_select": trigger_select,
        "frame_trigger_delay_us": frame_trigger_delay_us,
        "advanced_frame_trigger_delay_us": advanced_frame_trigger_delay_us,
        "hardware_trigger": hardware_trigger,
        "tdm_timing": tdm_timing,
        "expected_fps": fps,
        "fps": fps,
        "duty_cycle": duty_cycle,
        "selected_chirps": selected,
        "valid": True,
    }


def validate_summary(summary: dict[str, Any]) -> None:
    failures: list[str] = []
    trigger_select = int(summary.get("trigger_select", 1))
    if trigger_select not in (1, 2):
        failures.append(f"unsupported triggerSelect={trigger_select}")
    if trigger_select == 2:
        hardware = summary.get("hardware_trigger") or {}
        try:
            offsets = hardware["edge_offsets_us"]
            roles = hardware["station_roles"]
            a_us, b_us = float(offsets["A"]), float(offsets["B"])
            chirp_us = float(summary["chirp_period_us"])
            ramp_us = float(summary["rampEndTime"]) * 0.01
            period_ms = float(hardware["external_trigger_period_ms"])
            pulse_width_us = float(hardware["pulse_width_us"])
            if hardware.get("trigger_source") != "stm32_dual_tdm" or int(hardware["schema_version"]) != 1:
                failures.append("hardware-trigger profile metadata is unsupported")
            if roles.get("radar0") == roles.get("radar1") or set(roles.values()) != {"A", "B"}:
                failures.append("radar0 and radar1 must use different A/B trigger outputs")
            if not all(math.isfinite(value) for value in (a_us, b_us, chirp_us, ramp_us, period_ms, pulse_width_us)):
                failures.append("hardware-trigger timing values must be finite")
            else:
                separation_us = b_us - a_us
                if not 0 < separation_us < chirp_us:
                    failures.append("A/B trigger separation must be positive and shorter than one chirp period")
            if ramp_us >= min(b_us - a_us, chirp_us - (b_us - a_us)):
                failures.append("A/B chirp ramps would overlap")
            if float(summary["tdm_timing"]["next_a_frame_guard_us"]) <= 0:
                failures.append("last B chirp would overlap the next A frame")
            if period_ms < float(summary["frame_period_ms"]):
                failures.append("external trigger period must not be shorter than the configured frame period")
            if not 0.025 <= pulse_width_us <= 4.0:
                failures.append("SYNC_IN pulse width is outside 0.025..4 us")
            if summary["rx_channel_mask"] != summary["rx_chan_en"]:
                failures.append("channelRx and rxChanEn masks differ")
            if float(summary["frame_trigger_delay_us"]) != 0 or float(summary["advanced_frame_trigger_delay_us"]) != 0:
                failures.append("triggerDelay and frameTrigDelay must both be zero for TDM")
            if not math.isclose(float(hardware["first_pair_delay_ms"]), period_ms, abs_tol=0.01):
                failures.append("first trigger pair delay must match the STM32 period")
            for key, actual in (("expected_chirps_per_frame", summary["chirps_per_frame"]),
                                ("expected_rx_count", summary["rx_count"]),
                                ("expected_rx_mask", summary["rx_channel_mask"]),
                                ("expected_adc_fmt", summary["adc_fmt"]),
                                ("expected_adc_samples", summary["adc_samples"]),
                                ("expected_sample_rate_ksps", summary["sample_rate_ksps"]),
                                ("expected_start_freq_const", summary["startFreqConst"]),
                                ("expected_freq_slope_const", summary["freqSlopeConst"]),
                                ("expected_chirp_period_us", chirp_us),
                                ("expected_ramp_us", ramp_us)):
                if not math.isclose(float(hardware[key]), float(actual), rel_tol=0, abs_tol=0.01):
                    failures.append(f"{key} does not match the radar waveform")
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            failures.append(f"incomplete hardware-trigger metadata: {exc}")
    if int(summary["loopCount"]) > 255:
        failures.append(f"loopCount={summary['loopCount']} exceeds legacy-frame limit 255")
    if int(summary["num_chirp_indexes"]) <= 0:
        failures.append(
            f"invalid chirp range {summary['chirpStartIdxFCF']}..{summary['chirpEndIdxFCF']}"
        )
    if float(summary["active_time_ms"]) >= float(summary["frame_period_ms"]):
        failures.append(
            "active chirp time is not shorter than frame period: "
            f"{summary['active_time_ms']:.6f} ms >= {summary['frame_period_ms']:.6f} ms"
        )
    selected = summary.get("selected_chirps") or []
    expected_indexes = set(range(int(summary["chirpStartIdxFCF"]), int(summary["chirpEndIdxFCF"]) + 1))
    found_indexes: set[int] = set()
    for item in selected:
        start = int(item["chirpStartIdx"])
        end = int(item["chirpEndIdx"])
        for chirp_index in range(start, end + 1):
            if chirp_index in expected_indexes:
                found_indexes.add(chirp_index)
        if int(item["profileIdCPCFG"]) != 0 and not (end < min(expected_indexes) or start > max(expected_indexes)):
            failures.append(f"chirp {start}..{end} uses profileIdCPCFG={item['profileIdCPCFG']}, expected 0")
        if int(item["txEnable"]) != 1 and not (end < min(expected_indexes) or start > max(expected_indexes)):
            failures.append(f"chirp {start}..{end} uses txEnable={item['txEnable']}, expected 1 for TX0 only")
    missing = sorted(expected_indexes - found_indexes)
    if missing:
        failures.append(f"missing chirp definitions for selected indexes: {missing}")
    if failures:
        raise RuntimeError("; ".join(failures))


def base_config_path(station_config: dict[str, Any]) -> Path:
    raw = station_config.get("radar_config_base") or str(DEFAULT_BASE_CONFIG)
    path = Path(raw)
    if not path.is_absolute():
        path = ROOT / path
    return path


def mode_config_path(mode: str) -> Path:
    return ROOT / "configFiles" / f"mmwaveconfig_0508_{mode}.txt"


def generate_mode_config(mode: str, station_config: dict[str, Any]) -> Path:
    if mode not in SUPPORTED_MODES:
        allowed = ", ".join(SUPPORTED_MODES)
        raise RuntimeError(f"Unsupported single-TX HRRP mode '{mode}'. Supported: {allowed}.")

    values = SUPPORTED_MODES[mode]
    src = base_config_path(station_config)
    text = src.read_text(encoding="utf-8", errors="replace")
    text = force_single_tx_chirps(text, values["chirpStartIdx"], values["chirpEndIdx"])
    for key in ("chirpStartIdxFCF", "chirpEndIdxFCF", "loopCount", "periodicity"):
        text = set_value(text, key, values[key])

    dest = mode_config_path(mode)
    dest.write_text(text, encoding="utf-8", newline="")
    summary = parse_config(dest)
    validate_summary(summary)
    return dest


def apply_mode(mode: str, duration_seconds: float | None = None) -> dict[str, Any]:
    station_config = read_station_config()
    if "radar_config_base" not in station_config:
        station_config["radar_config_base"] = str(DEFAULT_BASE_CONFIG)
    dest = generate_mode_config(mode, station_config)
    station_config["radar_mode"] = mode
    station_config["radar_config"] = str(dest)
    write_station_config(station_config)
    summary = parse_config(dest)
    summary["active_mode"] = mode
    return add_duration_expectations(summary, duration_seconds)


def active_config_summary(duration_seconds: float | None = None) -> dict[str, Any]:
    station_config = read_station_config()
    path = Path(station_config.get("radar_config") or mode_config_path(DEFAULT_MODE))
    if not path.is_absolute():
        path = ROOT / path
    summary = parse_config(path)
    validate_summary(summary)
    configured_mode = station_config.get("radar_mode") or summary["mode"]
    summary["active_mode"] = configured_mode if configured_mode == summary["mode"] else summary["mode"]
    summary["camera_fps"] = station_config.get("camera_fps")
    summary["camera_resolution"] = [station_config.get("camera_width"), station_config.get("camera_height")]
    return add_duration_expectations(summary, duration_seconds)


def print_summary(summary: dict[str, Any]) -> None:
    validate_summary(summary)
    keys = [
        "active_mode",
        "mode",
        "config_path",
        "profile_id",
        "tx_mode",
        "rx_count",
        "rx_channel_mask",
        "adc_fmt",
        "adc_samples",
        "chirps_per_frame",
        "loopCount",
        "chirpStartIdxFCF",
        "chirpEndIdxFCF",
        "periodicity",
        "expected_fps",
        "frame_period_ms",
        "effective_frame_period_ms",
        "trigger_select",
        "hardware_trigger",
        "tdm_timing",
        "active_chirp_time_ms",
        "duty_cycle",
        "requested_duration_sec",
        "expected_frames",
        "expected_raw_bytes",
        "sample_rate_ksps",
        "freqSlopeConst",
        "selected_chirps",
        "camera_fps",
        "camera_resolution",
    ]
    payload = {key: summary[key] for key in keys if key in summary}
    payload["active_mode"] = summary.get("active_mode") or summary.get("mode")
    print(json.dumps(payload, indent=2))


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    apply_parser = subparsers.add_parser("apply")
    apply_parser.add_argument("mode")
    apply_parser.add_argument("--duration", type=float, default=None)
    config_parser = subparsers.add_parser("config")
    config_parser.add_argument("--duration", type=float, default=None)
    validate_parser = subparsers.add_parser("validate")
    validate_parser.add_argument("path", nargs="?")
    validate_parser.add_argument("--duration", type=float, default=None)
    args = parser.parse_args()

    try:
        if args.command == "apply":
            print_summary(apply_mode(args.mode, args.duration))
        elif args.command == "config":
            print_summary(active_config_summary(args.duration))
        elif args.command == "validate":
            if args.path:
                path = Path(args.path)
            else:
                path = Path(read_station_config()["radar_config"])
            if not path.is_absolute():
                path = ROOT / path
            summary = add_duration_expectations(parse_config(path), args.duration)
            print_summary(summary)
        return 0
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
