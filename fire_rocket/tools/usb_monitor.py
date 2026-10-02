"""USB text commands and file export for the telemetry-only firmware.

Requires pyserial (included in PlatformIO's Python environment).
Never enumerates, opens or resets a device unless a port is explicitly supplied.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import time


def save_export(port, kind: str, output: Path, timeout: float = 120) -> None:
    partial = output.with_name(output.name + ".partial")
    if output.exists() or partial.exists():
        raise FileExistsError(f"Refusing to overwrite {output} or {partial}")
    output.parent.mkdir(parents=True, exist_ok=True)
    port.write(f"EXPORT {kind}\n".encode("ascii"))
    deadline = time.monotonic() + timeout
    begun = False
    with partial.open("x", encoding="utf-8", newline="") as stream:
        while time.monotonic() < deadline:
            raw = port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="strict").rstrip("\r\n")
            if line.startswith("ERR"):
                raise RuntimeError(line)
            if not begun:
                if line == f"BEGIN {kind}":
                    begun = True
                continue
            if line == "END":
                break
            stream.write(line + "\n")
        else:
            raise TimeoutError("Incomplete export; partial file retained")
    partial.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Explicit serial port, for example COM5")
    commands = parser.add_subparsers(dest="action", required=True)
    send = commands.add_parser("command")
    send.add_argument("text", help="One firmware command, e.g. STATUS")
    export = commands.add_parser("export")
    export.add_argument("--output", type=Path, required=True, help="New destination directory")
    args = parser.parse_args()
    import serial

    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=2) as port:
        port.reset_input_buffer()
        if args.action == "export":
            for kind, filename in (("CSV", "telemetry.csv"), ("META", "session.json"),
                                   ("EVENTS", "events.csv"), ("END", "end.json")):
                destination = args.output / filename
                save_export(port, kind, destination)
                print(destination)
        else:
            if "\n" in args.text or "\r" in args.text:
                parser.error("Send one command at a time")
            port.write((args.text + "\n").encode("ascii"))
            deadline, quiet = time.monotonic() + 3, None
            failed = False
            while time.monotonic() < deadline:
                line = port.readline()
                if line:
                    text = line.decode("utf-8").rstrip()
                    print(text)
                    failed = failed or text.startswith("ERR")
                    quiet = time.monotonic() + 0.3
                elif quiet is not None and time.monotonic() > quiet:
                    break
            if quiet is None:
                raise TimeoutError("No reply from monitor firmware")
            if failed:
                raise SystemExit(1)


if __name__ == "__main__":
    main()
