#!/usr/bin/env python3
"""Resolve a stable serial device from a physical Linux USB hub path."""
from __future__ import annotations

from pathlib import Path


def resolve_tty_from_hub_port(hub_port: str) -> str:
    """Return /dev/serial/by-path when possible, otherwise current /dev/ttyUSBX."""
    hub_port = str(hub_port).strip()
    if not hub_port:
        raise RuntimeError("USB hub port kosong")

    sysfs_root = Path("/sys/bus/usb/devices") / hub_port
    if not sysfs_root.exists():
        raise RuntimeError(
            f"USB hub {hub_port} tidak ditemukan di /sys/bus/usb/devices"
        )

    tty_names: set[str] = set()
    for entry in sysfs_root.rglob("ttyUSB*"):
        name = entry.name
        if (
            name.startswith("ttyUSB")
            and name[6:].isdigit()
            and (Path("/dev") / name).exists()
        ):
            tty_names.add(name)

    if not tty_names:
        raise RuntimeError(f"hub {hub_port} belum memiliki ttyUSB aktif")
    if len(tty_names) != 1:
        raise RuntimeError(
            f"hub {hub_port} ambigu, tty aktif: {sorted(tty_names)}"
        )

    dev = Path("/dev") / next(iter(tty_names))
    dev_real = dev.resolve()

    by_path_dir = Path("/dev/serial/by-path")
    if by_path_dir.exists():
        for link in sorted(by_path_dir.iterdir()):
            try:
                if link.resolve() == dev_real:
                    return str(link)
            except OSError:
                continue

    return str(dev)
