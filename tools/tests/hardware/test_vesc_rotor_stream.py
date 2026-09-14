#!/usr/bin/env python3
"""Hardware check for VESC COMM_SET_DETECT -> 100 Hz COMM_ROTOR_POSITION."""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == "tools")
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from vesc_dual import VescDual

MODE_OBSERVER = 2
MODE_PID_POS = 4


def wrapped_diff(a: float, b: float) -> float:
    return ((a - b + 180.0) % 360.0) - 180.0


def sample_stream(link: VescDual, right: bool, mode: int, seconds: float):
    link.set_detect(mode, right)
    timestamps: list[float] = []
    positions: list[float] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            position = link.recv_rotor_position(timeout=min(0.05, max(0.005, deadline - time.monotonic())))
        except TimeoutError:
            continue
        timestamps.append(time.monotonic())
        positions.append(position)
    values = link.values(right)
    link.set_detect(0, right)
    time.sleep(0.04)
    return timestamps, positions, values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", nargs="?", default="auto")
    parser.add_argument("--seconds", type=float, default=1.2)
    args = parser.parse_args()

    link = VescDual(args.port, 115200, timeout=0.3)
    ok = True
    try:
        for right in (False, True):
            name = "RIGHT-ID2" if right else "LEFT-ID1"
            for mode, label in ((MODE_OBSERVER, "OBSERVER"), (MODE_PID_POS, "PID_POS")):
                timestamps, positions, values = sample_stream(link, right, mode, args.seconds)
                hz = ((len(timestamps) - 1) / (timestamps[-1] - timestamps[0])) if len(timestamps) > 1 else 0.0
                periods = [(b - a) * 1000.0 for a, b in zip(timestamps, timestamps[1:])]
                finite_range = bool(positions) and all(-0.01 <= x <= 360.01 for x in positions)
                diff = wrapped_diff(positions[-1], values.position) if positions and mode == MODE_PID_POS else 0.0
                good = (
                    len(positions) >= int(args.seconds * 85)
                    and 85.0 <= hz <= 115.0
                    and values.fault == 0
                    and finite_range
                    and (mode != MODE_PID_POS or abs(diff) < 1.0)
                )
                avg_period = statistics.mean(periods) if periods else 0.0
                stream_pos = positions[-1] if positions else 0.0
                print(
                    f"{name} {label}: n={len(positions)} rate={hz:.2f}Hz "
                    f"avg_period={avg_period:.2f}ms stream={stream_pos:.5f}deg "
                    f"values={values.position:.5f}deg diff={diff:.5f} fault={values.fault} "
                    f"RESULT={'PASS' if good else 'FAIL'}"
                )
                ok &= good
    finally:
        for right in (False, True):
            try:
                link.set_detect(0, right)
            except Exception:
                pass
        link.close()

    print("VESC_ROTOR_POSITION_PERIODIC_PASS" if ok else "VESC_ROTOR_POSITION_PERIODIC_FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
