#!/usr/bin/env python3
"""Telemetry-only DJI smooth-shot detector (MVP).

Requires ExifTool.  ffprobe is optional but recommended for exact video-frame
timestamps; when it is unavailable, timestamps are calculated from the FPS.
No video frames are decoded by this program.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from statistics import median
from typing import Any


CHANNELS = ("gimbal_yaw", "gimbal_pitch", "drone_yaw", "drone_pitch", "drone_roll")
ALIASES = {
    "gimbal_yaw": ("gimbalyaw",), "gimbal_pitch": ("gimbalpitch",),
    "drone_yaw": ("droneyaw", "flightyaw"), "drone_pitch": ("dronepitch", "flightpitch"),
    "drone_roll": ("droneroll", "flightroll"),
}


@dataclass
class Config:
    window_seconds: float = 0.75
    start_threshold: float = 85.0
    end_threshold: float = 70.0
    good_persistence: float = 0.75
    bad_persistence: float = 0.20
    min_duration: float = 4.0
    gimbal_yaw_weight: float = 0.38
    gimbal_pitch_weight: float = 0.32
    roll_weight: float = 0.12
    jerk_sensitivity: float = 1.0
    fps: float | None = None


def run_json(command: list[str]) -> Any:
    try:
        result = subprocess.run(command, text=True, capture_output=True, check=True)
        return json.loads(result.stdout)
    except FileNotFoundError as exc:
        raise RuntimeError(f"Required command not found: {command[0]}") from exc
    except (subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        detail = getattr(exc, "stderr", "") or str(exc)
        raise RuntimeError(f"Command failed: {' '.join(command)}\n{detail}") from exc


def key_for(name: str) -> str | None:
    normalized = "".join(c.lower() for c in name if c.isalpha())
    for channel, aliases in ALIASES.items():
        if any(alias in normalized for alias in aliases):
            return channel
    return None


def number(value: Any) -> float | None:
    if isinstance(value, (int, float)) and math.isfinite(float(value)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value.split()[0])
        except (ValueError, IndexError):
            return None
    return None


def telemetry_records(payload: Any) -> list[dict[str, float | None]]:
    """Accept ExifTool rows, arrays of values, and ``DocN:Tag`` timed records."""
    objects = payload if isinstance(payload, list) else [payload]
    rows: list[dict[str, float | None]] = []
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        # With -ee3 -G3, recent ExifTool versions put each timed DJI sample in
        # a distinct DocN group within one JSON object.  Preserve those groups
        # instead of overwriting each channel with the final sample.
        documents: dict[int, dict[str, float | None]] = {}
        for name, value in obj.items():
            group, separator, tag = name.partition(":")
            if not (separator and group.startswith("Doc") and group[3:].isdigit()):
                continue
            channel = key_for(tag)
            if channel:
                record = documents.setdefault(int(group[3:]), {key: None for key in CHANNELS})
                record[channel] = number(value)
        if documents:
            rows.extend(documents[index] for index in sorted(documents))
            continue
        found: dict[str, Any] = {}
        for name, value in obj.items():
            channel = key_for(name)
            if channel:
                found[channel] = value
        if not found:
            continue
        count = max((len(v) for v in found.values() if isinstance(v, list)), default=1)
        for index in range(count):
            row = {}
            for channel in CHANNELS:
                value = found.get(channel)
                row[channel] = number(value[index] if isinstance(value, list) and index < len(value) else value)
            rows.append(row)
    return rows


def extract_telemetry(source: Path) -> list[dict[str, float | None]]:
    # -ee3 exposes timed DJI records; -n preserves numerical angles.
    payload = run_json([
        "exiftool", "-ee3", "-api", "LargeFileSupport=1", "-G3", "-n", "-j",
        "-GimbalYaw", "-GimbalPitch", "-DroneYaw", "-DronePitch", "-DroneRoll", str(source),
    ])
    rows = telemetry_records(payload)
    if not rows:
        raise RuntimeError("No DJI attitude telemetry found. Confirm this is an original DJI MP4 and that ExifTool supports its djmd stream.")
    return rows


def parse_rate(value: str | None) -> float | None:
    if not value or value == "0/0": return None
    try:
        a, b = value.split("/")
        return float(a) / float(b)
    except (ValueError, ZeroDivisionError): return None


def media_timestamps(source: Path, samples: int, fallback_fps: float | None) -> tuple[float, list[float]]:
    """Get stream rate cheaply, then construct timestamp positions from it.

    Asking ffprobe for every frame PTS can decode/scan a large HEVC file and
    defeats the purpose of a telemetry-only first pass. DJI Mini 4 Pro footage
    is constant-frame-rate in the target workflow, so sample_index / FPS gives
    stable time coordinates without touching video frames.
    """
    fps = fallback_fps or 59.94005994
    probe = shutil.which("ffprobe")
    if probe:
        try:
            data = run_json([probe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                "stream=avg_frame_rate,r_frame_rate", "-of", "json", str(source)])
            stream = (data.get("streams") or [{}])[0]
            fps = fallback_fps or parse_rate(stream.get("avg_frame_rate")) or parse_rate(stream.get("r_frame_rate")) or fps
        except RuntimeError:
            pass
    return fps, [index / fps for index in range(samples)]


def fill_gaps(values: list[float | None], max_gap: int) -> list[float | None]:
    result = values[:]
    known = [i for i, x in enumerate(values) if x is not None]
    if not known: return result
    for left, right in zip(known, known[1:]):
        gap = right - left - 1
        if 0 < gap <= max_gap:
            for i in range(left + 1, right):
                result[i] = values[left] + (values[right] - values[left]) * (i - left) / (right - left)  # type: ignore[operator]
    return result


def unwrap(values: list[float | None]) -> list[float | None]:
    result: list[float | None] = []
    offset = 0.0; previous: float | None = None
    for value in values:
        if value is None:
            result.append(None); continue
        if previous is not None:
            delta = value - previous
            if delta > 180: offset -= 360
            elif delta < -180: offset += 360
        result.append(value + offset); previous = value
    return result


def derivative(values: list[float | None], timestamps: list[float]) -> list[float | None]:
    output: list[float | None] = [None] * len(values)
    for i in range(1, len(values)):
        if values[i] is not None and values[i - 1] is not None:
            dt = timestamps[i] - timestamps[i - 1]
            if dt > 0: output[i] = (values[i] - values[i - 1]) / dt  # type: ignore[operator]
    return output


def rolling_rms(values: list[float | None], radius: int) -> list[float | None]:
    """Windowed energy retains brief corrections; state persistence handles noise."""
    answer: list[float | None] = []
    for i in range(len(values)):
        nearby = [x for x in values[max(0, i-radius):i+radius+1] if x is not None]
        answer.append(math.sqrt(sum(x * x for x in nearby) / len(nearby)) if nearby else None)
    return answer


def robust_scale(values: list[float | None]) -> float:
    usable = sorted(abs(x) for x in values if x is not None)
    if not usable: return 1.0
    return max(0.05, usable[int(0.90 * (len(usable) - 1))])


def analyze(rows: list[dict[str, float | None]], timestamps: list[float], cfg: Config) -> list[dict[str, Any]]:
    intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps)) if timestamps[i] > timestamps[i - 1]]
    if not intervals:
        raise RuntimeError(f"Need at least two ordered telemetry samples; found {len(rows)}.")
    fps = 1 / median(intervals)
    gap = max(1, round(fps * 0.15))
    radius = max(1, round(fps * cfg.window_seconds / 2))
    signals: dict[str, dict[str, list[float | None]]] = {}
    for channel in CHANNELS:
        position = fill_gaps([r[channel] for r in rows], gap)
        if "yaw" in channel: position = unwrap(position)
        velocity = derivative(position, timestamps)
        acceleration = derivative(velocity, timestamps)
        jerk = derivative(acceleration, timestamps)
        signals[channel] = {"position": position, "velocity": velocity, "acceleration": acceleration, "jerk": jerk,
                            "jerk_roll": rolling_rms(jerk, radius)}
    # Data-adaptive normalisation makes the default work across slow pans and fast moves.
    scales = {c: robust_scale(signals[c]["jerk_roll"]) * cfg.jerk_sensitivity for c in CHANNELS}
    weights = {"gimbal_yaw": cfg.gimbal_yaw_weight, "gimbal_pitch": cfg.gimbal_pitch_weight,
               "drone_roll": cfg.roll_weight, "drone_yaw": 0.10, "drone_pitch": 0.08}
    diagnostics: list[dict[str, Any]] = []
    for i, row in enumerate(rows):
        total = weight = 0.0
        for channel, channel_weight in weights.items():
            metric = signals[channel]["jerk_roll"][i]
            if metric is not None:
                # Exponential penalty preserves high scores for continuously moving shots.
                total += channel_weight * 100 * math.exp(-metric / scales[channel])
                weight += channel_weight
        score = total / weight if weight else 0.0
        diagnostics.append({"sample": i, "timestamp": timestamps[i], "score": score,
                            **{f"{c}_{kind}": signals[c][kind][i] for c in CHANNELS for kind in ("position", "velocity", "acceleration", "jerk")}})
    return diagnostics


def segments(diag: list[dict[str, Any]], cfg: Config) -> list[dict[str, float]]:
    result: list[dict[str, float]] = []; active: int | None = None; good_since: int | None = None; bad_since: int | None = None
    for i, sample in enumerate(diag):
        t, score = sample["timestamp"], sample["score"]
        if active is None:
            good_since = i if score >= cfg.start_threshold and good_since is None else (good_since if score >= cfg.start_threshold else None)
            if good_since is not None and t - diag[good_since]["timestamp"] >= cfg.good_persistence:
                active = good_since; bad_since = None
        else:
            bad_since = i if score < cfg.end_threshold and bad_since is None else (bad_since if score < cfg.end_threshold else None)
            if bad_since is not None and t - diag[bad_since]["timestamp"] >= cfg.bad_persistence:
                end_i = bad_since
                start, end = diag[active]["timestamp"], diag[end_i]["timestamp"]
                if end - start >= cfg.min_duration:
                    result.append({"start": start, "end": end, "duration": end-start, "score": sum(x["score"] for x in diag[active:end_i+1])/(end_i-active+1)})
                active = good_since = bad_since = None
    if active is not None:
        start, end = diag[active]["timestamp"], diag[-1]["timestamp"]
        if end - start >= cfg.min_duration:
            result.append({"start": start, "end": end, "duration": end-start, "score": sum(x["score"] for x in diag[active:])/len(diag[active:])})
    return result


def display_time(value: float) -> str:
    minutes, seconds = divmod(value, 60); hours, minutes = divmod(int(minutes), 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:06.3f}"


def discover(path: Path, recursive: bool) -> list[Path]:
    if path.is_file(): return [path]
    iterator = path.rglob("*") if recursive else path.iterdir()
    return sorted(x for x in iterator if x.is_file() and x.suffix.lower() == ".mp4")


def companions(source: Path) -> dict[str, str | None]:
    """Report matching DJI sidecars without making them a prerequisite for MVP."""
    result: dict[str, str | None] = {}
    for suffix, key in ((".LRF", "lrf"), (".SRT", "srt")):
        candidate = source.with_suffix(suffix)
        result[key] = candidate.name if candidate.is_file() else None
    return result


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    keys = list(rows[0]) if rows else ["sample", "timestamp", "score"]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys); writer.writeheader(); writer.writerows(rows)


def write_losslesscut_csv(path: Path, found: list[dict[str, float]]) -> None:
    """Write LosslessCut's headerless CSV edit-decision-list format."""
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        for index, item in enumerate(found, 1):
            writer.writerow((item["start"], item["end"], f"Smooth segment {index} (score {item['score']:.0f})"))


def cut_segments(source: Path, found: list[dict[str, float]], output_dir: Path) -> list[Path]:
    """Stream-copy selected ranges into MP4 files without re-encoding."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("Required command not found: ffmpeg")
    targets = [output_dir / f"{source.stem}.smooth-{index:02d}.mp4" for index in range(1, len(found) + 1)]
    collisions = [str(target) for target in targets if target.exists()]
    if collisions:
        raise RuntimeError("Refusing to overwrite existing cut file(s): " + ", ".join(collisions))
    output_dir.mkdir(parents=True, exist_ok=True)
    for item, target in zip(found, targets):
        duration = item["end"] - item["start"]
        command = [
            ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error", "-ss", str(item["start"]), "-i", str(source),
            "-t", str(duration), "-map", "0:v:0", "-map", "0:a?", "-map", "0:s?", "-c", "copy",
            "-avoid_negative_ts", "make_zero", "-n", str(target),
        ]
        try:
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError(f"FFmpeg could not create {target}: {exc}") from exc
    return targets


def main() -> int:
    parser = argparse.ArgumentParser(description="Find smooth DJI drone shots from embedded djmd telemetry.")
    parser.add_argument("input", type=Path, help="DJI MP4 file or directory")
    parser.add_argument("--recursive", action="store_true")
    parser.add_argument("--json", dest="json_path", type=Path, help="write JSON (file for one input, directory for many)")
    parser.add_argument("--debug-csv", type=Path, help="write per-sample diagnostics CSV (file for one input, directory for many)")
    parser.add_argument("--losslesscut-csv", type=Path, help="write LosslessCut segment-import CSV (file for one input, directory for many)")
    parser.add_argument("--cut", type=Path, metavar="OUTPUT_DIR", help="stream-copy each detected segment to this directory (no re-encoding)")
    parser.add_argument("--config", type=Path, help="JSON file overriding detector settings")
    for name, typ in (("window-seconds", float), ("start-threshold", float), ("end-threshold", float), ("good-persistence", float), ("bad-persistence", float), ("min-duration", float), ("jerk-sensitivity", float), ("fps", float)):
        parser.add_argument("--" + name, type=typ)
    args = parser.parse_args()
    settings = asdict(Config())
    if args.config:
        settings.update(json.loads(args.config.read_text(encoding="utf-8")))
    for name in settings:
        value = getattr(args, name, None)
        if value is not None: settings[name] = value
    cfg = Config(**settings)
    files = discover(args.input, args.recursive)
    if not files: parser.error("no .MP4 files found")
    failed = False
    for source in files:
        try:
            rows = extract_telemetry(source)
            fps, timestamps = media_timestamps(source, len(rows), cfg.fps)
            diag = analyze(rows, timestamps, cfg); found = segments(diag, cfg)
            report = {"file": source.name, "fps": fps, "telemetry_samples": len(rows), "companions": companions(source),
                      "segments": [{**s, "score": round(s["score"])} for s in found]}
            print(f"\n{source.name}  ({fps:.5f} fps, {len(rows)} telemetry samples)")
            if not found: print("No smooth segments meeting the configured thresholds.")
            for n, item in enumerate(report["segments"], 1):
                print(f"\nSmooth Segment {n}\nStart: {display_time(item['start'])}\nEnd:   {display_time(item['end'])}\nDuration: {item['duration']:.3f} sec\nScore: {item['score']}")
            if args.json_path:
                target = args.json_path / f"{source.stem}.smoothness.json" if len(files) > 1 or args.json_path.is_dir() else args.json_path
                target.parent.mkdir(parents=True, exist_ok=True); target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
            if args.debug_csv:
                target = args.debug_csv / f"{source.stem}.diagnostics.csv" if len(files) > 1 or args.debug_csv.is_dir() else args.debug_csv
                target.parent.mkdir(parents=True, exist_ok=True); write_csv(target, diag)
            if args.losslesscut_csv:
                target = args.losslesscut_csv / f"{source.stem}.losslesscut.csv" if len(files) > 1 or args.losslesscut_csv.is_dir() else args.losslesscut_csv
                target.parent.mkdir(parents=True, exist_ok=True); write_losslesscut_csv(target, found)
            if args.cut and found:
                for target in cut_segments(source, found, args.cut):
                    print(f"Wrote lossless cut: {target}")
        except RuntimeError as exc:
            print(f"\n{source.name}: ERROR: {exc}", file=sys.stderr)
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
