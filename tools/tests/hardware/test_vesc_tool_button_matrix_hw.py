#!/usr/bin/env python3
"""Hardware matrix for non-energizing VESC Tool 6.00 button/protocol behavior."""
import sys
from pathlib import Path
TOOLS_DIR = next(p for p in Path(__file__).resolve().parents if p.name == 'tools')
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import argparse, hashlib, sys, time
TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from vesc_dual import COMM_FW_VERSION, VescDual, parse_fw


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()[:16]


def one_motor(link: VescDual, right: bool) -> dict:
    name = "RIGHT-ID2" if right else "LEFT-ID1"
    fw = parse_fw(link.fw(right))
    vals = link.values(right)
    setup = link.setup_values(right)
    diag = link.diag(right)
    tune = link.get_tuning(right)
    pos = link.position_state(right)
    print(f"{name} FW={fw} fault={vals.fault} setup_id={setup.vesc_id} diag_id={diag.vesc_id}")
    if setup.vesc_id != (2 if right else 1) or diag.vesc_id != (2 if right else 1):
        raise RuntimeError(f"{name} VESC ID mismatch")
    if vals.fault or diag.fault:
        raise RuntimeError(f"{name} fault active")

    mc = link.get_mcconf_raw(right)
    mc_def = link.get_mcconf_raw(right, default=True)
    app = link.get_appconf_raw(right)
    app_def = link.get_appconf_raw(right, default=True)
    print(f"{name} MC len={len(mc)} sha={sha(mc)} default={sha(mc_def)} APP len={len(app)} sha={sha(app)} default={sha(app_def)}")

    # Tombol Write Motor Config: echo payload persis yang baru dibaca, lalu readback byte-identik.
    link.set_mcconf_raw(mc, right)
    mc_rb = link.get_mcconf_raw(right)
    if mc_rb != mc:
        raise RuntimeError(f"{name} MC write/readback differs: {sha(mc)} != {sha(mc_rb)}")

    # App Config NO_STORE dan STORE diuji dengan payload identik agar tidak mengubah perilaku.
    link.set_appconf_raw(app, right, store=False)
    if link.get_appconf_raw(right) != app:
        raise RuntimeError(f"{name} App no-store readback mismatch")
    link.set_appconf_raw(app, right, store=True)
    app_rb = link.get_appconf_raw(right)
    if app_rb != app:
        raise RuntimeError(f"{name} App store readback mismatch")
    # MC Temp: baca lalu tulis nilai identik dengan ACK, tanpa store.
    mt = link.get_mcconf_temp(right)
    link.set_mcconf_temp(mt, store=False, forward=False, ack=True, right=right)
    mt2 = link.get_mcconf_temp(right)
    if mt2 != mt:
        raise RuntimeError(f"{name} MC temp roundtrip mismatch")

    # Battery cut standard Commands::get/setBatteryCut dengan nilai yang sama.
    bc = link.get_battery_cut(right)
    link.set_battery_cut(bc[0], bc[1], store=False, forward=False, right=right)
    bc2 = link.get_battery_cut(right)
    if max(abs(bc2[0]-bc[0]), abs(bc2[1]-bc[1])) > 0.002:
        raise RuntimeError(f"{name} battery cut roundtrip mismatch {bc}->{bc2}")

    # Custom App Data: diag/tuning/position harus request/reply dan no-store echo harus stabil.
    tune2 = link.set_tuning(tune, right=right, store=False)
    if tune2 != tune:
        raise RuntimeError(f"{name} tuning App Data echo mismatch")
    pos2 = link.set_position_limits(pos.minimum, pos.maximum, right=right)
    if pos2.minimum != pos.minimum or pos2.maximum != pos.maximum:
        raise RuntimeError(f"{name} position App Data limit echo mismatch")

    # Terminal sync harus kembali sebagai COMM_PRINT framed reply.
    fw_text = link.terminal("fw", right=right, sync=True).strip()
    status_text = link.terminal("status", right=right, sync=True).strip()
    if "FW 6.00" not in fw_text or "fault=" not in status_text:
        raise RuntimeError(f"{name} terminal reply invalid")
    print(f"{name} terminal='{fw_text}' status='{status_text}'")
    return {"mc": mc, "app": app, "odo": setup.odometer_m}

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="auto")
    ap.add_argument("--reboot-check", action="store_true")
    ap.add_argument("--arm", action="store_true", help="required because this test writes persistent MC/App configuration")
    args = ap.parse_args()
    if not args.arm:
        ap.error("persistent configuration test requires --arm")
    link = VescDual(args.port, timeout=0.8)
    link.require_platform_compatible(True)
    try:
        ids = link.ping_can()
        print("PING_CAN", ids)
        if 2 not in ids:
            raise RuntimeError("virtual CAN ID 2 missing")
        adc = link.decoded_adc()
        print("DECODED_ADC", ", ".join(f"{v:.5f}" for v in adc))
        left = one_motor(link, False)
        right = one_motor(link, True)
        # Set Odometer menggunakan nilai existing: membuktikan command tanpa mengubah odometer.
        link.set_odometer(left["odo"], False)
        link.set_odometer(right["odo"], True)
        time.sleep(0.05)
        if link.setup_values(False).odometer_m != left["odo"] or link.setup_values(True).odometer_m != right["odo"]:
            raise RuntimeError("odometer same-value write/readback mismatch")
        if not args.reboot_check:
            print("VESC_TOOL_BUTTON_MATRIX_PASS")
            return 0
        before = {False: left, True: right}
        print("REBOOT local controller...")
        link.reboot(False)
    finally:
        link.close()

    # Reboot readiness is not a fixed sleep: STM32F1 startup includes the
    # master-hoverFOC 900-ms melody plus EEPROM restore and 2000-sample ADC
    # offset calibration. Probe the actual VESC endpoint until FW+setup are
    # valid, while keeping a bounded failure deadline and reporting the latency.
    ready_start = time.monotonic()
    ready_deadline = ready_start + 6.0
    link = None
    last_error = None
    while time.monotonic() < ready_deadline:
        candidate = None
        try:
            candidate = VescDual(args.port, timeout=0.35)
            fw = candidate.fw(False)
            setup = candidate.setup_values(False)
            if fw[0] == COMM_FW_VERSION and setup.vesc_id == 1:
                link = candidate
                break
            candidate.close()
        except Exception as exc:
            last_error = exc
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
        time.sleep(0.10)
    if link is None:
        raise RuntimeError(f"VESC not ready within 6 s after reboot: {last_error}")
    print(f"REBOOT_READY latency={time.monotonic()-ready_start:.3f}s")
    try:
        for right in (False, True):
            if link.get_mcconf_raw(right) != before[right]["mc"]:
                raise RuntimeError("MC Config changed across reboot")
            if link.get_appconf_raw(right) != before[right]["app"]:
                raise RuntimeError("App Config changed across reboot")
        if 2 not in link.ping_can():
            raise RuntimeError("CAN ID2 missing after reboot")
        print("VESC_TOOL_BUTTON_MATRIX_REBOOT_PERSISTENCE_PASS")
        return 0
    finally:
        link.close()

if __name__ == "__main__":
    raise SystemExit(main())
