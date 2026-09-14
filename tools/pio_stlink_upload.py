#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time
import struct
import tempfile
from pathlib import Path

from stlink_target_guard import resolve_stlink_transport, verify_f103_target


META_MAGIC = 0x56455343
META_STATE_CONFIRMED = 0x434E464D
META_VERSION = 2
FLASH_BASE = 0x08000000
FLASH_END = 0x08040000
BOOT_BASE = 0x08000000
BOOT_SIZE = 0x00002800
APP_BASE = 0x08002800
APP_REGION_SIZE = 0x0003C000
META_BASE = 0x0803E800
EEPROM_BASE = 0x0803F000
SRAM_BASE = 0x20000000
SRAM_BOOT_REQUEST = 0x2000BFF0

def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def confirmed_meta(image: bytes) -> bytes:
    size = len(image)
    c = crc16(image)
    return struct.pack(
        '<IIIIHHHHHHII',
        META_MAGIC, META_STATE_CONFIRMED, size, (~size) & 0xFFFFFFFF,
        c, (~c) & 0xFFFF, META_VERSION, (~META_VERSION) & 0xFFFF,
        0xFFFF, 0xFFFF, 0xFFFFFFFF, 0xFFFFFFFF)

def validate_project_image(image: bytes, address: int, max_size: int, meta_address):
    if len(image) < 8:
        raise RuntimeError('image vector table missing')
    if address == BOOT_BASE:
        if max_size != BOOT_SIZE or meta_address is not None:
            raise RuntimeError('bootloader upload partition arguments do not match final layout')
    elif address == APP_BASE:
        if max_size != APP_REGION_SIZE or meta_address != META_BASE:
            raise RuntimeError('application upload requires 240-KiB APP partition plus CONFIRMED metadata at 0x0803E800')
    else:
        raise RuntimeError(f'unsupported project flash base 0x{address:08X}')
    if address < FLASH_BASE or address + len(image) > FLASH_END or len(image) > max_size:
        raise RuntimeError('image crosses its authorized flash partition')
    sp, rv = struct.unpack_from('<II', image, 0)
    pc = rv & ~1
    if not (SRAM_BASE <= sp <= SRAM_BOOT_REQUEST and (sp & 3) == 0):
        raise RuntimeError(f'invalid initial MSP 0x{sp:08X}')
    if (rv & 1) == 0 or not (address <= pc < address + len(image)):
        raise RuntimeError(f'invalid Thumb Reset_Handler 0x{rv:08X} for image at 0x{address:08X}')
    if meta_address is not None:
        if meta_address != META_BASE or meta_address + 36 > EEPROM_BASE:
            raise RuntimeError('metadata location would overlap EEPROM or is not canonical')

def openocd_program_cmd(openocd, scripts, transport, image, address, speed, meta_path=None, meta_address=None):
    cmd = [str(openocd), '-s', str(scripts), '-f', 'interface/stlink.cfg',
           '-c', f'transport select {transport}']
    cmd += ['-f', 'target/stm32f1x.cfg', '-c', f'adapter speed {speed}']
    actions = [
        'init', 'reset halt',
        f'flash write_image erase {{{image}}} 0x{address:08X} bin',
        f'verify_image {{{image}}} 0x{address:08X} bin',
    ]
    if meta_path is not None:
        actions += [
            f'flash write_image erase {{{meta_path}}} 0x{meta_address:08X} bin',
            f'verify_image {{{meta_path}}} 0x{meta_address:08X} bin',
        ]
    actions += ['reset run', 'shutdown']
    cmd += ['-c', '; '.join(actions)]
    return cmd

def parse_int(value: str) -> int:
    return int(value, 0)


def unique_speeds(preferred: int):
    # Fast first when requested, then progressively more conservative SWD clocks.
    values = [preferred, 400, 100, 50]
    out = []
    for value in values:
        value = max(50, min(int(value), 4000))
        if value not in out:
            out.append(value)
    return out


def wait_for_application_runtime(size: int, wanted_crc: int, timeout_s: float = 15.0) -> bool:
    """Verify the freshly flashed app through the production USART3 protocol.

    If no direct USB-UART is physically present, APP_STLINK can still be used as
    an ST-Link-only recovery path and returns False. If a UART candidate exists,
    however, failure to reach the matching CONFIRMED runtime is a hard failure.
    """
    try:
        import pio_vesc_upload as vu
        candidates=vu._serial_candidates()
    except Exception as exc:
        print(f'[STLINK] UART runtime proof unavailable: {exc}', flush=True)
        return False
    if not candidates:
        print('[STLINK] no direct USB-UART detected; runtime proof limited to image/metadata + SWD', flush=True)
        return False
    deadline=time.monotonic()+max(1.0,float(timeout_s)); last='no response'; attempt=0
    while time.monotonic()<deadline:
        attempt += 1
        try:
            candidates=vu._serial_candidates() or candidates
        except Exception:
            pass
        for dev,_desc in candidates:
            link=None
            try:
                ns=argparse.Namespace(serial_port=dev,baud=115200)
                link=vu.Link(ns)
                hw=vu.fw_version(link,1.0)
                if hw and 'bootloader' not in hw.lower():
                    info=vu.app_update_info(link,1.0)
                    if info['state']==vu.STATE_CONFIRMED and info['size']==size and info['crc']==wanted_crc:
                        print(f'[STLINK] application runtime PASS uart={dev} hw={hw} CONFIRMED size={size} crc16=0x{wanted_crc:04X}', flush=True)
                        return True
                    last=f'{hw} metadata={info}'
                else:
                    last=hw or 'empty identity'
            except Exception as exc:
                last=f'{dev}: {type(exc).__name__}: {exc}'
            finally:
                if link is not None:
                    try: link.close()
                    except Exception: pass
        if attempt==1 or attempt%3==0:
            remain=max(0.0,deadline-time.monotonic())
            print(f'[STLINK] waiting application UART runtime attempt={attempt} remaining={remain:.1f}s last={last}', flush=True)
        time.sleep(.30)
    raise RuntimeError(f'application UART runtime did not become healthy: {last}')


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--address", required=True, type=parse_int)
    ap.add_argument("--max-size", required=True, type=parse_int)
    ap.add_argument("--adapter-khz", type=int, default=1000)
    ap.add_argument("--confirmed-meta-address", type=parse_int, default=None,
                    help="also write v2 CONFIRMED metadata for this application image")
    args = ap.parse_args()

    image = Path(args.image).resolve()
    if not image.is_file():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: image missing: {image}")
    image_bytes = image.read_bytes()
    size = len(image_bytes)
    if size <= 0 or size > args.max_size:
        raise SystemExit(f"STLINK_UPLOAD_FAIL: size {size} exceeds {args.max_size}")
    try:
        validate_project_image(image_bytes, args.address, args.max_size, args.confirmed_meta_address)
    except RuntimeError as exc:
        raise SystemExit(f"STLINK_UPLOAD_FAIL: {exc}; NO FLASH WRITE PERFORMED")

    pio_home = Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio"))
    openocd_root = pio_home / "packages" / "tool-openocd"
    scripts = openocd_root / "openocd" / "scripts"
    openocd_candidates = [
        openocd_root / "bin" / "openocd.exe",
        openocd_root / "bin" / "openocd",
    ]
    openocd = next((candidate for candidate in openocd_candidates if candidate.is_file()), None)
    if openocd is None:
        checked = ", ".join(str(candidate) for candidate in openocd_candidates)
        raise SystemExit(f"STLINK_UPLOAD_FAIL: OpenOCD missing; checked: {checked}")
    if not scripts.is_dir():
        raise SystemExit(f"STLINK_UPLOAD_FAIL: OpenOCD scripts missing: {scripts}")

    transport = resolve_stlink_transport(scripts)

    # Production uploader is deliberately NORMAL-SWD only. The repository has
    # no connect-under-reset code path: a firmware image that strands SWD fails
    # here instead of being hidden by tooling.
    try:
        verify_f103_target(openocd, scripts, 100)
    except RuntimeError as exc:
        raise SystemExit(f'STLINK_NORMAL_SWD_ATTACH_FAIL: {exc}')
    print('[STLINK] normal SWD attach PASS', flush=True)

    meta_tmp = None
    meta_path = None
    try:
        if args.confirmed_meta_address is not None:
            meta_tmp = tempfile.NamedTemporaryFile(prefix='f103_confirmed_', suffix='.bin', delete=False)
            meta_tmp.write(confirmed_meta(image_bytes)); meta_tmp.flush(); meta_tmp.close()
            meta_path = Path(meta_tmp.name)
            print(f'[STLINK] CONFIRMED metadata prepared size={size} crc16=0x{crc16(image_bytes):04X} '
                  f'address=0x{args.confirmed_meta_address:08X}', flush=True)

        last_rc = 1
        speeds = unique_speeds(args.adapter_khz)
        for attempt, speed in enumerate(speeds, start=1):
            cmd = openocd_program_cmd(openocd, scripts, transport, image, args.address, speed,
                                      meta_path, args.confirmed_meta_address)
            mode = 'normal'
            print(f'[STLINK] attempt={attempt}/{len(speeds)} image={image.name} size={size} '
                  f'address=0x{args.address:08X} swd={speed}kHz mode={mode}', flush=True)
            last_rc = subprocess.run(cmd, check=False).returncode
            if last_rc == 0:
                # Post-run proof must be non-invasive. On this ST-Link V2/OpenOCD pair,
                # halting a healthy running F103 can report an "unknown state" and
                # fabricate PC/MSP=0 even though normal SWD examination remains valid.
                # The image and CONFIRMED metadata were already byte-verified above;
                # here we prove repeated normal-SWD attach without reset/under-reset.
                app_runtime = args.confirmed_meta_address is not None
                checks = ((0.75, 1), (1.25, 2), (3.00, 3))
                for delay_s, check_no in checks:
                    time.sleep(delay_s)
                    try:
                        verify_f103_target(openocd, scripts, 100)
                    except RuntimeError as exc:
                        raise SystemExit(
                            f'STLINK_POSTRUN_NORMAL_ATTACH_FAIL check={check_no}/3 '
                            f'app_runtime={int(app_runtime)}: {exc}')
                    print(f'[STLINK] post-run normal SWD check {check_no}/3 PASS', flush=True)
                runtime_verified=False
                if app_runtime:
                    try:
                        runtime_verified=wait_for_application_runtime(size,crc16(image_bytes))
                    except RuntimeError as exc:
                        raise SystemExit(f'STLINK_POSTRUN_APP_RUNTIME_FAIL: {exc}')
                print(f'STLINK_BIN_UPLOAD_PASS swd={speed}kHz normal_attach_stable=3/3 '
                      f'app_image_verified={int(app_runtime)} app_runtime_verified={int(runtime_verified)}', flush=True)
                return
            if attempt < len(speeds):
                print(f'[STLINK] retrying at safer SWD clock after rc={last_rc}', file=sys.stderr, flush=True)
                time.sleep(0.30)
        raise SystemExit(last_rc)
    finally:
        if meta_path is not None:
            try: meta_path.unlink()
            except OSError: pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(130)
