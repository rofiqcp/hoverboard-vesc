#!/usr/bin/env python3
from pathlib import Path
import hashlib
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
BOOT = ROOT / ".pio/build/BOOTLOADER_STLINK/firmware.bin"
APP = ROOT / ".pio/build/APP_STLINK/firmware.bin"
OUT = ROOT / "dist/f103_factory_boot_app.bin"

FLASH_BASE = 0x08000000
BOOT_SIZE = 0x2800
APP_BASE = FLASH_BASE + BOOT_SIZE
APP_REGION_SIZE = 0x3C000
META_BASE = 0x0803E800
META_OFFSET = META_BASE - FLASH_BASE
META_MAGIC = 0x56455343
META_STATE_CONFIRMED = 0x434E464D
META_VERSION = 2
RAM_LO = 0x20000000
RAM_HI = 0x2000C000
def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def confirmed_meta(app: bytes) -> bytes:
    size = len(app); c = crc16(app)
    return struct.pack('<IIIIHHHHHHII', META_MAGIC, META_STATE_CONFIRMED, size, (~size) & 0xFFFFFFFF,
                       c, (~c) & 0xFFFF, META_VERSION, (~META_VERSION) & 0xFFFF,
                       0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF)

def fail(msg: str) -> None:
    raise SystemExit(f"FACTORY_IMAGE_FAIL: {msg}")


def main() -> None:
    if not BOOT.is_file() or not APP.is_file():
        fail("build BOOTLOADER_STLINK and APP_STLINK first")
    boot = BOOT.read_bytes()
    app = APP.read_bytes()
    if not boot or len(boot) > BOOT_SIZE:
        fail(f"bootloader size {len(boot)} exceeds {BOOT_SIZE}")
    if not app or len(app) > APP_REGION_SIZE:
        fail(f"application size {len(app)} exceeds {APP_REGION_SIZE}")
    if len(app) < 8:
        fail("application vector table missing")

    sp, rv = struct.unpack_from("<II", app, 0)
    if not (RAM_LO <= sp <= RAM_HI and (sp & 3) == 0):
        fail(f"invalid application MSP 0x{sp:08X}")
    pc = rv & ~1
    if (rv & 1) == 0 or not (APP_BASE <= pc < APP_BASE + APP_REGION_SIZE):
        fail(f"invalid application Reset_Handler 0x{rv:08X}")

    meta = confirmed_meta(app)
    image = bytearray(b"\xFF" * (META_OFFSET + len(meta)))
    image[: len(boot)] = boot
    image[BOOT_SIZE : BOOT_SIZE + len(app)] = app
    image[META_OFFSET : META_OFFSET + len(meta)] = meta
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(image)

    digest = hashlib.sha256(image).hexdigest()
    print(
        "FACTORY_IMAGE_PASS "
        f"boot={len(boot)} app={len(app)} meta={len(meta)} total={len(image)} "
        f"app_base=0x{APP_BASE:08X} meta_base=0x{META_BASE:08X} crc16=0x{crc16(app):04X} sha256={digest}"
    )
    print(f"FACTORY_IMAGE={OUT}")


if __name__ == "__main__":
    main()
