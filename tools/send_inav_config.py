#!/usr/bin/env python3

import argparse
import sys
import time
from serial import Serial


def iter_commands(path):
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line_no, raw in enumerate(fh, 1):
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                continue
            yield line_no, line


def read_until_quiet(ser, quiet_timeout=0.35, max_total=2.0):
    end_total = time.monotonic() + max_total
    end_quiet = time.monotonic() + quiet_timeout
    out = bytearray()

    while True:
        now = time.monotonic()
        if now >= end_total:
            break
        if now >= end_quiet and len(out) > 0:
            break

        chunk = ser.read(1)
        if chunk:
            out.extend(chunk)
            end_quiet = time.monotonic() + quiet_timeout

    if not out:
        return ""
    return out.decode("utf-8", errors="replace")


def send_line(ser, line, line_no, quiet_timeout, max_total, show_no_response):
    data = (line + "\n").encode("utf-8", errors="replace")
    ser.write(data)
    print(f">> [{line_no}] {line}")

    resp = read_until_quiet(ser, quiet_timeout=quiet_timeout, max_total=max_total)
    if resp:
        for part in resp.splitlines():
            print(f"<< {part}")
    elif show_no_response:
        print("<< (no response)")


def main():
    parser = argparse.ArgumentParser(
        description="Send INAV CLI commands from a config .txt file over serial"
    )
    parser.add_argument("device", help="serial device, e.g. /dev/tty.usbserial-A50285BI")
    parser.add_argument("baud", type=int, help="serial baud rate")
    parser.add_argument("config", help="path to INAV config .txt file")
    parser.add_argument("--no-enter-cli", action="store_true", help="do not send '#' before commands")
    parser.add_argument("--delay", type=float, default=0.05, help="delay between commands in seconds (default: 0.05)")
    parser.add_argument("--quiet-timeout", type=float, default=0.35, help="response quiet timeout after each command in seconds (default: 0.35)")
    parser.add_argument("--max-response", type=float, default=2.0, help="max response wait per command in seconds (default: 2.0)")
    parser.add_argument("--show-no-response", action="store_true", help="print marker when a command has no immediate response")
    parser.add_argument("--dry-run", action="store_true", help="print commands without sending")
    args = parser.parse_args()

    commands = list(iter_commands(args.config))
    if not commands:
        print("No commands found (file may contain only comments/blank lines).")
        return

    print(f"Loaded {len(commands)} commands from {args.config}")

    if args.dry_run:
        for line_no, line in commands:
            print(f"[dry-run] [{line_no}] {line}")
        return

    ser = Serial(args.device, args.baud, timeout=0.05)
    print(f"Opened {args.device} @ {args.baud}")

    try:
        if not args.no_enter_cli:
            print("Entering INAV CLI with '#'")
            send_line(
                ser,
                "#",
                0,
                quiet_timeout=args.quiet_timeout,
                max_total=args.max_response,
                show_no_response=args.show_no_response,
            )
            time.sleep(max(args.delay, 0.05))

        for line_no, line in commands:
            send_line(
                ser,
                line,
                line_no,
                quiet_timeout=args.quiet_timeout,
                max_total=args.max_response,
                show_no_response=args.show_no_response,
            )
            if args.delay > 0:
                time.sleep(args.delay)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        ser.close()
        print("Serial closed")


if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)