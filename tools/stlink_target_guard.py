#!/usr/bin/env python3
"""Fail-closed ST-Link target guard for the AGV STM32F103RCT6 motor controller."""
import argparse
import re
import subprocess
from pathlib import Path

EXPECTED_CORE = "Cortex-M3"
EXPECTED_DEV_ID = 0x414  # STM32F103 high-density (xC/xD/xE)


def resolve_stlink_transport(scripts: Path) -> str:
    """Select the transport supported by the installed OpenOCD ST-Link driver."""
    cfg = scripts / "interface" / "stlink.cfg"
    try:
        text = cfg.read_text(encoding="utf-8", errors="ignore").lower()
    except OSError:
        text = ""
    if "adapter driver hla" in text or "hla_layout stlink" in text:
        return "hla_swd"
    return "swd"


def verify_f103_target(openocd: Path, scripts: Path, speed: int = 100, under_reset: bool = False,
                       resume_before_shutdown: bool = False, expected_vtor: int | None = None,
                       pc_min: int | None = None, pc_max: int | None = None) -> str:
    transport = resolve_stlink_transport(scripts)
    cmd = [str(openocd), "-s", str(scripts), "-f", "interface/stlink.cfg",
           "-c", f"transport select {transport}"]
    if under_reset:
        cmd += ["-c", "reset_config srst_only srst_nogate connect_assert_srst"]
    actions = ["init", "halt", "flash info 0"]
    if expected_vtor is not None or pc_min is not None or pc_max is not None:
        actions += ["mdw 0xE000ED08 1", "reg pc"]
    if resume_before_shutdown:
        actions += ["resume"]
    actions += ["shutdown"]
    cmd += ["-f", "target/stm32f1x.cfg",
           "-c", f"adapter speed {int(speed)}",
           "-c", "; ".join(actions)]
    cp = subprocess.run(cmd, text=True, stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, check=False)
    out = cp.stdout or ""
    match = re.search(r"device id\s*=\s*0x([0-9a-fA-F]+)", out)
    raw_id = int(match.group(1), 16) if match else None
    dev_id = (raw_id & 0xFFF) if raw_id is not None else None
    core_ok = re.search(r"\bCortex-M3\b", out) is not None
    if cp.returncode != 0 or not core_ok or dev_id != EXPECTED_DEV_ID:
        core = EXPECTED_CORE if core_ok else "non-M3/unknown"
        did = f"0x{dev_id:03X}" if dev_id is not None else "unknown"
        raise RuntimeError(
            f"STLINK_TARGET_REJECTED: expected STM32F103RCT6 {EXPECTED_CORE} "
            f"DEV_ID=0x{EXPECTED_DEV_ID:03X}, got core={core} DEV_ID={did}; NO FLASH WRITE PERFORMED")
    vtor = None
    pc = None
    if expected_vtor is not None:
        m = re.search(r"0xe000ed08:\s*([0-9a-fA-F]{8})", out, re.I)
        vtor = int(m.group(1), 16) if m else None
        if vtor != expected_vtor:
            got = f"0x{vtor:08X}" if vtor is not None else "unknown"
            raise RuntimeError(f"STLINK_RUNTIME_REJECTED: expected VTOR=0x{expected_vtor:08X}, got {got}")
    if pc_min is not None or pc_max is not None:
        m = re.search(r"pc\s+\(/32\):\s*0x([0-9a-fA-F]+)", out, re.I)
        if not m:
            m = re.search(r"\bpc:\s*0x([0-9a-fA-F]+)", out, re.I)
        pc = int(m.group(1), 16) if m else None
        if pc is None or (pc_min is not None and pc < pc_min) or (pc_max is not None and pc >= pc_max):
            got = f"0x{pc:08X}" if pc is not None else "unknown"
            raise RuntimeError(f"STLINK_RUNTIME_REJECTED: PC {got} outside expected runtime image")
    extra = ''
    if vtor is not None: extra += f" VTOR=0x{vtor:08X}"
    if pc is not None: extra += f" PC=0x{pc:08X}"
    print(f"STLINK_TARGET_F103_PASS core={EXPECTED_CORE} DEV_ID=0x{dev_id:03X}{extra}", flush=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--openocd", required=True)
    ap.add_argument("--scripts", required=True)
    ap.add_argument("--speed", type=int, default=100)
    ap.add_argument("--under-reset", action="store_true")
    ap.add_argument("--resume", action="store_true", help="resume core before shutdown after a read-only probe")
    args = ap.parse_args()
    try:
        verify_f103_target(Path(args.openocd), Path(args.scripts), args.speed, args.under_reset, args.resume)
    except RuntimeError as exc:
        raise SystemExit(str(exc))


if __name__ == "__main__":
    main()
