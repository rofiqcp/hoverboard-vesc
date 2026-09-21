#!/usr/bin/env python3
"""HOME steering LEFT encoder lalu tahan COMM_SET_POS dalam derajat."""
from __future__ import annotations

import argparse
import os
import time

from usb_hub_resolver import resolve_tty_from_hub_port
from vesc_dual import DEFAULT_BAUD, VescDual, parse_fw

STEERING_HUB_PORT = "1-2.3.4"


def open_steering_encoder(
    requested: str | None, hub_port: str, baud: int
) -> tuple[str, VescDual, dict]:
    port = requested or resolve_tty_from_hub_port(hub_port)
    print(f"HUB  : {hub_port}")
    print(f"PROBE: {port} -> {os.path.realpath(port)}")

    link: VescDual | None = None
    try:
        link = VescDual(port, baud, timeout=0.35)
        fw = parse_fw(link.fw(False))
        cal = link.steering_calibration()
        print(
            f"  {fw} role={cal.get('role')} "
            f"encoder_cfg={cal.get('encoder_configured')} "
            f"fault={cal.get('fault')}"
        )
        if cal.get("role") != "encoder":
            raise RuntimeError("device pada hub steering bukan steering encoder")
        if not cal.get("encoder_configured"):
            raise RuntimeError("encoder belum configured")
        if cal.get("fault"):
            raise RuntimeError(f"fault={cal.get('fault')}")
        return port, link, cal
    except Exception:
        if link is not None:
            try:
                link.close()
            except Exception:
                pass
        raise


def main() -> int:
    ap = argparse.ArgumentParser(
        description="HOME steering LEFT encoder, lalu tahan posisi dalam derajat."
    )
    ap.add_argument(
        "--hub",
        default=STEERING_HUB_PORT,
        help=f"physical USB hub steering; default {STEERING_HUB_PORT}",
    )
    ap.add_argument(
        "--port",
        default=None,
        help="override port manual; default resolve otomatis dari --hub",
    )
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--position", type=float, default=180.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument(
        "--hold",
        type=float,
        default=0.0,
        help="lama refresh posisi; default 0 = terus sampai Ctrl+C",
    )
    args = ap.parse_args()
    hz = max(1.0, min(100.0, args.hz))
    period = 1.0 / hz

    print(f"BAUD : {args.baud}")
    print(f"Urutan: DETECT ENCODER -> HOME -> POS {args.position:.1f} deg")
    port, link, before = open_steering_encoder(args.port, args.hub, args.baud)
    print(f"PORT : {port} -> {os.path.realpath(port)}")
    try:
        print("CAL before:", before)
        print("HOME : mulai")
        home = link.home_steering()
        print("HOME :", home)
        if home.get("status") != 0 or not home.get("homed"):
            raise RuntimeError(f"HOME gagal: {home}")

        after = link.steering_calibration()
        print("CAL after :", after)
        if after.get("role") != "encoder" or not after.get("homed"):
            raise RuntimeError(f"verifikasi HOME gagal: {after}")

        print(f"POS  : target {args.position:.1f} deg @ {hz:.1f} Hz")
        started = time.monotonic()
        next_tick = started
        next_print = started
        while args.hold <= 0 or time.monotonic() - started < args.hold:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
            link.set_pos_one(args.position, right=False)
            link.alive(False)
            next_tick += period

            now = time.monotonic()
            if now >= next_print:
                v = link.values(False)
                print(
                    f"POS  : cmd={args.position:.1f} deg "
                    f"read={v.position:.2f} deg fault={v.fault}"
                )
                if v.fault:
                    raise RuntimeError(f"fault saat posisi: {v.fault}")
                next_print = now + 0.5

        print("DONE : HOME + POS command selesai")
        return 0
    except KeyboardInterrupt:
        print("\nSTOP : Ctrl+C, serial ditutup")
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
