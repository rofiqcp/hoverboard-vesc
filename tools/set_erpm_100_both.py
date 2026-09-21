#!/usr/bin/env python3
"""Kirim eRPM yang sama ke drive left dan right sampai Ctrl+C.

Port serial tidak di-hardcode. Device selalu di-resolve dari physical USB hub
DRIVE, sehingga perubahan ttyUSB setelah reconnect/boot tidak memengaruhi tool.
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from usb_hub_resolver import resolve_tty_from_hub_port
from vesc_dual import VescDual

DRIVE_HUB_PORT = "1-2.3.1"
BAUD = 921600
TARGET_ERPM = 200
COMMAND_HZ = 50.0


def stop_drive(link: VescDual) -> None:
    """Lepaskan torque kedua motor saat program dihentikan."""
    for _ in range(3):
        link.set_current(0.0, 0.0)
        time.sleep(0.02)


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Kirim eRPM yang sama ke drive left dan right."
    )
    ap.add_argument(
        "--hub",
        default=DRIVE_HUB_PORT,
        help=f"physical USB hub drive; default {DRIVE_HUB_PORT}",
    )
    ap.add_argument(
        "--port",
        default=None,
        help="override port manual; default resolve otomatis dari --hub",
    )
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--erpm", type=int, default=TARGET_ERPM)
    ap.add_argument("--hz", type=float, default=COMMAND_HZ)
    args = ap.parse_args()

    port = args.port or resolve_tty_from_hub_port(args.hub)
    hz = max(1.0, min(100.0, float(args.hz)))

    print(f"HUB  : {args.hub}")
    print(f"PORT : {port} -> {os.path.realpath(port)}")
    print(f"BAUD : {args.baud}")
    print(f"Sending LEFT={args.erpm} eRPM | RIGHT={args.erpm} eRPM")
    print("Ctrl+C untuk stop.")

    link = VescDual(port, args.baud, timeout=0.12)
    period = 1.0 / hz
    next_tx = time.monotonic()

    try:
        while True:
            link.set_rpm(args.erpm, args.erpm)
            next_tx += period
            time.sleep(max(0.0, next_tx - time.monotonic()))
    except KeyboardInterrupt:
        print("\nSTOP requested.")
    finally:
        try:
            stop_drive(link)
        finally:
            link.close()

    print("Drive stopped, serial closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
