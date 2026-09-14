#!/usr/bin/env python3
"""Minimal ST-Link/OpenOCD uploader for the F103 bootloader/application layout."""
from __future__ import annotations

import argparse
import os
import struct
import subprocess
import tempfile
from pathlib import Path

from vesc_common import crc16

FLASH_END = 0x08040000
BOOT_BASE = 0x08000000
BOOT_SIZE = 0x00002800
APP_BASE = 0x08002800
APP_SIZE = 0x0003C000
META_BASE = 0x0803E800
EEPROM_BASE = 0x0803F000
META_MAGIC = 0x56455343
META_STATE_CONFIRMED = 0x434E464D
META_VERSION = 2


def parse_int(value: str) -> int:
    return int(value, 0)


def confirmed_meta(image: bytes) -> bytes:
    size = len(image)
    crc = crc16(image)
    return struct.pack(
        '<IIIIHHHHHHII',
        META_MAGIC, META_STATE_CONFIRMED, size, (~size) & 0xFFFFFFFF,
        crc, (~crc) & 0xFFFF, META_VERSION, (~META_VERSION) & 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    )


def openocd_paths() -> tuple[Path, Path]:
    pio_home = Path(os.environ.get('PLATFORMIO_CORE_DIR', Path.home() / '.platformio'))
    root = pio_home / 'packages' / 'tool-openocd'
    exe = root / 'bin' / ('openocd.exe' if os.name == 'nt' else 'openocd')
    scripts = root / 'openocd' / 'scripts'
    if not exe.is_file():
        raise SystemExit(f'STLINK_UPLOAD_FAIL: OpenOCD missing: {exe}')
    return exe, scripts


def validate_partition(address: int, size: int, max_size: int, meta_address: int | None) -> None:
    if address == BOOT_BASE:
        if max_size != BOOT_SIZE or meta_address is not None:
            raise SystemExit('STLINK_UPLOAD_FAIL: invalid bootloader partition arguments')
    elif address == APP_BASE:
        if max_size != APP_SIZE or meta_address != META_BASE:
            raise SystemExit('STLINK_UPLOAD_FAIL: invalid application partition arguments')
    else:
        raise SystemExit(f'STLINK_UPLOAD_FAIL: unsupported address 0x{address:08X}')
    if size <= 0 or size > max_size or address + size > FLASH_END:
        raise SystemExit(f'STLINK_UPLOAD_FAIL: image size {size} exceeds partition')
    if meta_address is not None and meta_address + 36 > EEPROM_BASE:
        raise SystemExit('STLINK_UPLOAD_FAIL: metadata overlaps EEPROM')


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--image', required=True)
    ap.add_argument('--address', required=True, type=parse_int)
    ap.add_argument('--max-size', required=True, type=parse_int)
    ap.add_argument('--adapter-khz', type=int, default=1000)
    ap.add_argument('--confirmed-meta-address', type=parse_int)
    args = ap.parse_args()

    image = Path(args.image).resolve()
    if not image.is_file():
        raise SystemExit(f'STLINK_UPLOAD_FAIL: image missing: {image}')
    data = image.read_bytes()
    validate_partition(args.address, len(data), args.max_size, args.confirmed_meta_address)

    openocd, scripts = openocd_paths()
    actions = [
        'init',
        'reset halt',
        f'flash write_image erase {{{image}}} 0x{args.address:08X} bin',
        f'verify_image {{{image}}} 0x{args.address:08X} bin',
    ]

    meta_path: Path | None = None
    try:
        if args.confirmed_meta_address is not None:
            with tempfile.NamedTemporaryFile(prefix='f103_meta_', suffix='.bin', delete=False) as tmp:
                tmp.write(confirmed_meta(data))
                meta_path = Path(tmp.name)
            actions += [
                f'flash write_image erase {{{meta_path}}} 0x{args.confirmed_meta_address:08X} bin',
                f'verify_image {{{meta_path}}} 0x{args.confirmed_meta_address:08X} bin',
            ]
        actions += ['reset run', 'shutdown']
        cmd = [
            str(openocd), '-s', str(scripts),
            '-f', 'interface/stlink.cfg',
            '-f', 'target/stm32f1x.cfg',
            '-c', f'adapter speed {max(50, int(args.adapter_khz))}',
            '-c', '; '.join(actions),
        ]
        print(f'[STLINK] flash {image.name} -> 0x{args.address:08X} ({len(data)} bytes)')
        subprocess.run(cmd, check=True)
        print('STLINK_UPLOAD_PASS')
    finally:
        if meta_path is not None:
            meta_path.unlink(missing_ok=True)


if __name__ == '__main__':
    main()
