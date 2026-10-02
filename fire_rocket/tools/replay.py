"""Generate sensor-only fault fixtures and replay the firmware's native C++ core.

No actuator, staging, recovery, trajectory prediction or scoring implementation.
Uses only the Python standard library. All generated motion is synthetic.
"""
from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path
import struct
import subprocess


def frame(command: int, data: bytes) -> bytes:
    body = bytes([command]) + data
    return b"\xfa\xfb" + bytes([len(body) + 1]) + body + bytes([sum(body) & 255]) + b"\xfc\xfd"


def sample(height: float = 100.0) -> bytes:
    data = bytes([0x0C])  # Ten-axis, magnetometer quality 3.
    data += struct.pack("<3h", 0, 0, 2048)
    data += struct.pack("<3h", 0, 0, 0)
    data += struct.pack("<3h", 0, 0, 0)
    data += struct.pack("<3h", 1000, 2000, 3000)
    data += struct.pack("<4h", 32767, 0, 0, 0)
    data += struct.pack("<hIi", 2500, 419430400, round(height / 0.0010728836))
    return frame(0, data)


def demo() -> str:
    rows = ["# Synthetic sensor stream; not a flight model.", "0 " + frame(0x19, bytes([255, 7, 6, 1])).hex()]
    for now in range(0, 10001, 50):
        if 4000 <= now < 4900:
            rows.append(f"{now} TICK")
            continue
        height = 100 + (0.0 if now <= 2500 else 0.2 * math.sin(now / 500))
        if 7000 <= now <= 8000:
            height += (now - 7000) / 500  # Deliberate sensor drift, not rocket dynamics.
        packet = sample(height)
        if now in (3000, 7500):
            packet = packet[:-3] + bytes([packet[-3] ^ 0x80]) + packet[-2:]
        if now == 5500:
            rows.append(f"{now} 00fafbff01")  # Corrupted length followed by a good frame.
        if now == 6000:
            rows.append(f"{now} {packet[:9].hex()}")
            rows.append(f"{now} {packet[9:].hex()}")
        else:
            rows.append(f"{now} {packet.hex()}")
        if now == 2000:
            rows.append(f"{now} ZERO")
    rows.append("11000 TICK")
    return "\n".join(rows) + "\n"


def run(executable: Path, trace: Path, output: Path, check_demo: bool) -> dict:
    process = subprocess.run(
        [str(executable.resolve())], input=trace.read_text(encoding="utf-8"),
        text=True, capture_output=True, check=True,
    )
    rows = list(csv.DictReader(io.StringIO(process.stdout)))
    if not rows:
        raise ValueError("Replay produced no records")
    output.mkdir(parents=True, exist_ok=True)
    (output / "telemetry.csv").write_text(process.stdout, encoding="utf-8")
    report = {
        "scope": "sensor_monitor_only",
        "input_records": len(rows),
        "accepted_samples": int(rows[-1]["samples"]),
        "checksum_errors": int(rows[-1]["checksum_errors"]),
        "framing_errors": int(rows[-1]["framing_errors"]),
        "data_errors": int(rows[-1]["data_errors"]),
        "stale_snapshots": sum(row["fresh"] == "0" for row in rows),
        "successful_zero_events": sum(row["event"] == "ZERO_OK" for row in rows),
    }
    if check_demo:
        checks = {
            "two_bad_checksums_rejected": report["checksum_errors"] == 2,
            "length_corruption_recovered": report["framing_errors"] >= 1,
            "all_other_samples_received": report["accepted_samples"] == 181,
            "stable_manual_zero": report["successful_zero_events"] == 1,
            "gap_is_stale": any(4500 <= int(row["t_ms"]) < 4900 and row["fresh"] == "0" for row in rows),
            "recovery_is_fresh": any(int(row["t_ms"]) == 4900 and row["fresh"] == "1" for row in rows),
            "stale_height_not_zero": rows[-1]["fresh"] == "0" and math.isnan(float(rows[-1]["relative_m"])),
        }
        report["checks"] = checks
        report["passed"] = all(checks.values())
    (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    make = commands.add_parser("demo")
    make.add_argument("--output", type=Path, required=True)
    replay = commands.add_parser("run")
    replay.add_argument("--exe", type=Path, required=True)
    replay.add_argument("--input", type=Path, required=True)
    replay.add_argument("--output", type=Path, required=True)
    replay.add_argument("--check-demo", action="store_true")
    args = parser.parse_args()
    if args.command == "demo":
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(demo(), encoding="utf-8")
        print(args.output)
    else:
        result = run(args.exe, args.input, args.output, args.check_demo)
        print(json.dumps(result, indent=2))
        if args.check_demo and not result["passed"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
