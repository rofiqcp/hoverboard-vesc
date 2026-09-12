#!/usr/bin/env python3
import argparse
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UPLOADER = ROOT / "tools/pio_stlink_upload.py"

TARGETS = {
    "bootloader": {
        "image": ROOT / ".pio/build/BOOTLOADER_STLINK/firmware.bin",
        "address": "0x08000000",
        "max_size": "0x2800",
        "extra": [],
    },
    "app": {
        "image": ROOT / ".pio/build/APP_STLINK/firmware.bin",
        "address": "0x08002800",
        "max_size": "0x3C000",
        "extra": ["--confirmed-meta-address", "0x0803E800"],
    },
}

def main() -> int:
    ap = argparse.ArgumentParser(description="Continuously catch STM32F103 under reset and flash immediately when reachable.")
    ap.add_argument("target", choices=TARGETS)
    ap.add_argument("--interval", type=float, default=0.20)
    ap.add_argument("--attempts", type=int, default=0, help="0 = retry forever")
    args = ap.parse_args()
    cfg = TARGETS[args.target]
    image = cfg["image"]
    if not image.is_file():
        print(f"AUTOCATCH_FAIL image missing: {image}", file=sys.stderr, flush=True)
        return 2

    cmd = [
        sys.executable, str(UPLOADER),
        "--image", str(image),
        "--address", cfg["address"],
        "--max-size", cfg["max_size"],
        "--rescue-under-reset",
        *cfg["extra"],
    ]

    print(f"AUTOCATCH_ARMED target={args.target} image={image.name}", flush=True)
    print("Hold MCU NRST to GND now. Release it when ready; flashing starts on the first valid F103 attach.", flush=True)

    attempt = 0
    while args.attempts == 0 or attempt < args.attempts:
        attempt += 1
        if attempt == 1 or (attempt % 10) == 0:
            print(f"[AUTOCATCH] attempt={attempt}", flush=True)
        cp = subprocess.run(cmd, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, check=False)
        if cp.returncode == 0:
            print(cp.stdout, end="" if cp.stdout.endswith("\n") else "\n", flush=True)
            print(f"AUTOCATCH_FLASH_PASS target={args.target} attempts={attempt}", flush=True)
            return 0
        if attempt == 1 or (attempt % 10) == 0:
            tail = (cp.stdout or "").strip().splitlines()[-1:] or ["no response"]
            print(f"[AUTOCATCH] waiting: {tail[0]}", flush=True)
        time.sleep(max(0.02, args.interval))

    print(f"AUTOCATCH_FAIL exhausted attempts={attempt}", file=sys.stderr, flush=True)
    return 1

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("AUTOCATCH_ABORTED", flush=True)
        raise SystemExit(130)


 