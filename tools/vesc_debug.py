#!/usr/bin/env python3
"""Diagnostik lengkap VESC 6.00 dual hoverboard STM32F103.

Contoh cepat:
  python3 tools/vesc_debug.py /dev/ttyUSB0 selftest
  python3 tools/vesc_debug.py /dev/ttyUSB0 info
  python3 tools/vesc_debug.py /dev/ttyUSB0 rt --motor both --hz 50 --seconds 10
  python3 tools/vesc_debug.py /dev/ttyUSB0 hall --motor left --amps 1.0 --arm
  python3 tools/vesc_debug.py /dev/ttyUSB0 current --motor left --amps 3 --seconds 3 --arm
  python3 tools/vesc_debug.py /dev/ttyUSB0 rpm --motor left --erpm 50 --seconds 5 --arm
  python3 tools/vesc_debug.py /dev/ttyUSB0 pos-limits --motor both --min -1000000 --max 1000000
  python3 tools/vesc_debug.py /dev/ttyUSB0 pos-count --motor left --count -250 --seconds 2 --arm

Motor yang dapat bergerak hanya dijalankan bila --arm diberikan. Semua setpoint aktif
secara periodik 50 Hz agar ownership/timeout VESC 500 ms tidak memutus command.
"""
from __future__ import annotations

import argparse
import csv
import math
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Callable

from vesc_dual import (
    VescDual, Values, Diag, PositionState, PacketDecoder,
    frame, crc16, parse_fw, parse_selective, parse_diag, parse_position_state,
    COMM_SET_CURRENT, COMM_SET_RPM, COMM_SET_DUTY, COMM_SET_POS,
    COMM_GET_VALUES_SELECTIVE, COMM_CUSTOM_APP_DATA,
    HB_MAGIC, HB_VERSION, HB_GET_DIAG, HB_GET_POS_STATE,
    VALUE_MASK, POLE_PAIRS,
)

INT32_MIN = -2147483648
INT32_MAX = 2147483647
DEFAULT_HZ = 50.0


def motor_flags(name: str) -> tuple[bool, bool]:
    n = name.lower()
    if n in ("left", "l"):
        return True, False
    if n in ("right", "r"):
        return False, True
    if n in ("both", "all"):
        return True, True
    raise ValueError(f"motor tidak dikenal: {name}")


def send_one(link: VescDual, motor: str, cmd: int, raw: int) -> None:
    payload = bytes((cmd,)) + struct.pack(">i", int(raw))
    with link.io_lock:
        if motor == "right":
            link.send(link.fwd(payload))
        else:
            link.send(payload)


def release_one(link: VescDual, motor: str) -> None:
    # Zero-current lalu berhenti refresh. Firmware sengaja mempertahankan flag
    # ownership setelah timeout agar input legacy tidak mengambil alih; karena
    # itu syarat release yang benar adalah state OFF dan telemetry sudah settle,
    # bukan override==0.
    is_right = motor == "right"
    for _ in range(3):
        send_one(link, motor, COMM_SET_CURRENT, 0)
        time.sleep(0.02)
    deadline = time.monotonic() + 1.5
    settled = 0
    while time.monotonic() < deadline:
        try:
            d = link.diag(is_right)
            v = link.values(is_right)
            if d.state == 0 and abs(v.duty) < 0.002 and abs(v.current_motor) < 0.6:
                settled += 1
                if settled >= 3:
                    return
            else:
                settled = 0
        except Exception:
            settled = 0
        time.sleep(0.04)
    print(f"[WARN] {motor} belum OFF/settle setelah 1.5 s")


class Refresher:
    def __init__(self, fn: Callable[[], None], hz: float = DEFAULT_HZ):
        if hz <= 0:
            raise ValueError("hz harus > 0")
        self.fn = fn
        self.period = 1.0 / hz
        self.stop_evt = threading.Event()
        self.errors: list[str] = []
        self.count = 0
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        nxt = time.monotonic()
        while not self.stop_evt.is_set():
            now = time.monotonic()
            if now < nxt:
                self.stop_evt.wait(min(0.002, nxt - now))
                continue
            try:
                self.fn()
                self.count += 1
            except Exception as exc:  # hardware diagnostic: report, jangan matikan thread diam-diam
                self.errors.append(str(exc))
            nxt += self.period
            if now - nxt > self.period * 4:
                nxt = now + self.period

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.stop_evt.set()
        self.thread.join(timeout=1.0)


def print_values(prefix: str, v: Values) -> None:
    print(
        f"{prefix}: id={v.vesc_id} ERPM={v.rpm:8.1f} duty={v.duty*100:7.2f}% "
        f"Imotor={v.current_motor:7.3f}A Iin={v.current_in:7.3f}A "
        f"Id={v.id:7.3f}A Iq={v.iq:7.3f}A Vd={v.vd:7.3f}V Vq={v.vq:7.3f}V "
        f"Vin={v.vin:6.2f}V pos={v.position:7.2f}deg fault={v.fault}"
    )


def print_diag(prefix: str, d: Diag) -> None:
    print(prefix + ": " + d.short())
    print(
        f"  pos_limit=[{d.position_min}, {d.position_max}] hall_table={d.hall_table} "
        f"hall_store={'OK' if d.hall_store_ok else 'NO/NOT-YET'} rx_ok={d.rx_ok} crc_err={d.rx_crc_errors}"
    )
    if d.phase_raw is not None:
        phase_deg = d.phase_raw * 360.0 / 65536.0
        hall_deg = d.phase_hall_raw * 360.0 / 65536.0
        target_deg = d.phase_target_raw * 360.0 / 65536.0
        center_deg = (d.hall_angle200 * 1.8) if d.hall_angle200 is not None and d.hall_angle200 < 200 else float('nan')
        print(f"  HALL_PHASE state={d.hall} table={d.hall_angle200}/200 center={center_deg:.1f}deg "
              f"phase={phase_deg:.1f} hall_phase={hall_deg:.1f} target={target_deg:.1f} "
              f"dir={d.hall_direction} interp={int(bool(d.hall_interp))} period={d.hall_period} ticks={d.hall_ticks} "
              f"reject_seq={d.hall_sequence_rejects} reject_period={d.hall_period_rejects} "
              f"last_reject={d.hall_last_reject_reason}:{d.hall_last_reject_from}->{d.hall_last_reject_to}")
    if d.motor_poles is not None:
        print(f"  CURRENT_OFFSETS phase0={d.current_offset_phase0} phase1={d.current_offset_phase1} dc={d.current_offset_dc} "
              f"DRIVETRAIN poles={d.motor_poles} pp={d.pole_pairs} gear={d.gear_ratio:.3f} "
              f"motor_rpm={d.motor_mech_rpm:.3f} output_rpm={d.output_rpm:.3f} qdrop={d.rx_queue_drops} "
              f"isr={d.foc_isr_cycles}/{d.foc_isr_cycles_max}cy")
        if d.phase_trip_count is not None:
            print(f"  OC_DIAG total={d.current_trips} phase={d.phase_trip_count} dc={d.dc_trip_count} "
                  f"streak={d.phase_overcurrent_streak} lastsrc=0x{d.last_trip_source:02x} "
                  f"lastI=[{d.last_trip_phase0_a:.2f},{d.last_trip_phase1_a:.2f},{d.last_trip_phase2_a:.2f}]A "
                  f"lastDC={d.last_trip_dc_a:.2f}A lastDuty={100*d.last_trip_duty:.1f}%")
        if d.driven_offset0 is not None:
            print(f"  CONTROL_OFFSETS raw=[{d.driven_offset0},{d.driven_offset1},{d.driven_offset_dc}] "
                  f"boot_valid={int(bool(d.current_offset_valid))} driven_valid={int(d.driven_offset_valid)} "
                  f"calibrating={int(d.driven_offset_calibrating)} samples={d.driven_offset_samples}")


def hall_table_valid(table: list[int]) -> tuple[bool, str]:
    if len(table) != 8:
        return False, "panjang bukan 8"
    if table[0] != 255 or table[7] != 255:
        return False, "state 0/7 harus 255"
    vals = table[1:7]
    if any(x == 255 or x > 199 for x in vals):
        return False, "state Hall 1..6 belum semuanya valid"
    sv = sorted(vals)
    gaps = [sv[i + 1] - sv[i] for i in range(5)] + [sv[0] + 200 - sv[5]]
    if any(g < 18 or g > 48 for g in gaps):
        return False, f"spacing tidak wajar: {gaps}"
    return True, f"spacing={gaps}"


def require_arm(args) -> None:
    if not args.arm:
        raise SystemExit("Perintah ini dapat menggerakkan motor. Jalankan ulang dengan --arm.")


def cmd_selftest(_args) -> int:
    payload = bytes((COMM_GET_VALUES_SELECTIVE,)) + struct.pack(">I", VALUE_MASK)
    raw = frame(payload)
    dec = PacketDecoder()
    chunks = [raw[:1], raw[1:4], raw[4:9], raw[9:]]
    got = []
    for c in chunks:
        got += dec.feed(c)
    assert got == [payload]
    assert crc16(payload) == ((raw[-3] << 8) | raw[-2])

    # custom position parser
    pos_payload = bytes((COMM_CUSTOM_APP_DATA,)) + HB_MAGIC + bytes((HB_VERSION, HB_GET_POS_STATE, 0)) + struct.pack(">iiii", -12, 34, INT32_MIN, INT32_MAX)
    ps = parse_position_state(pos_payload, HB_GET_POS_STATE)
    assert ps.current == -12 and ps.target == 34 and ps.minimum == INT32_MIN and ps.maximum == INT32_MAX

    # custom diagnostic parser, layout harus tetap sinkron firmware
    diag_payload = bytearray(bytes((COMM_CUSTOM_APP_DATA,)) + HB_MAGIC + bytes((HB_VERSION, HB_GET_DIAG, 0)))
    diag_payload += struct.pack(">8B", 1, 3, 1, 0, 5, 1, 1, 0)
    diag_payload += struct.pack(">10i", 3000, 2500, 2400, 10, 50, 1234, -7, -8, -1000, 1000)
    diag_payload += struct.pack(">4I", 2, 3, 100, 0)
    diag_payload += bytes((255, 83, 17, 50, 150, 117, 183, 255))
    d = parse_diag(bytes(diag_payload))
    assert math.isclose(d.iq_target_a, 3.0) and d.erpm == 50 and d.hall == 5
    ok, _ = hall_table_valid(d.hall_table)
    assert ok
    print("VESC_DEBUG_SELFTEST_PASS")
    return 0


def cmd_info(args, link: VescDual) -> int:
    print("LOCAL :", parse_fw(link.fw(False)))
    ids = link.ping_can()
    print("CAN   :", ids)
    if 2 in ids:
        print("RIGHT :", parse_fw(link.fw(True)))
    else:
        print("RIGHT : ID 2 tidak muncul pada PING_CAN")
    print_values("L", link.values(False))
    if 2 in ids:
        print_values("R", link.values(True))
    try:
        print_diag("L", link.diag(False))
        if 2 in ids:
            print_diag("R", link.diag(True))
    except Exception as exc:
        print("[WARN] custom diagnostic:", exc)
    return 0


def cmd_diag(args, link: VescDual) -> int:
    left, right = motor_flags(args.motor)
    if left:
        print_diag("L", link.diag(False))
    if right:
        print_diag("R", link.diag(True))
    return 0


def cmd_rt(args, link: VescDual) -> int:
    if not (1 <= args.hz <= 100):
        raise SystemExit("--hz harus 1..100")
    left, right = motor_flags(args.motor)
    period = 1.0 / args.hz
    end = time.monotonic() + args.seconds
    nxt = time.monotonic()
    ok_count = 0
    errors = 0
    rows = []
    start = time.monotonic()
    while time.monotonic() < end:
        now = time.monotonic()
        if now < nxt:
            time.sleep(min(0.001, nxt - now))
            continue
        timestamp = time.monotonic() - start
        try:
            if left:
                v = link.values(False); ok_count += 1
                rows.append((timestamp, "L", v.rpm, v.current_motor, v.current_in, v.id, v.iq, v.duty, v.vin, v.position, v.fault))
            if right:
                v = link.values(True); ok_count += 1
                rows.append((timestamp, "R", v.rpm, v.current_motor, v.current_in, v.id, v.iq, v.duty, v.vin, v.position, v.fault))
        except Exception as exc:
            errors += 1
            print("[RT ERROR]", exc)
        nxt += period
        if now - nxt > period * 4:
            nxt = now + period
    elapsed = max(time.monotonic() - start, 1e-9)
    channels = int(left) + int(right)
    expected = args.hz * elapsed * channels
    actual_hz_per_motor = ok_count / elapsed / max(channels, 1)
    print(f"RT_RESULT replies={ok_count} errors={errors} elapsed={elapsed:.3f}s rate_per_motor={actual_hz_per_motor:.2f}Hz expected~{expected:.0f}")
    if rows:
        last_l = next((r for r in reversed(rows) if r[1] == "L"), None)
        last_r = next((r for r in reversed(rows) if r[1] == "R"), None)
        if last_l: print("last L:", last_l)
        if last_r: print("last R:", last_r)
    if args.csv:
        out = Path(args.csv)
        with out.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(("time_s", "motor", "erpm", "imotor_a", "iin_a", "id_a", "iq_a", "duty", "vin_v", "pos_deg", "fault"))
            w.writerows(rows)
        print("CSV:", out)
    # toleransi scheduler/serial 10%; target utamanya 50 Hz saat --hz 50
    if errors or actual_hz_per_motor < args.hz * 0.90:
        print("RESULT: FAIL/UNSTABLE realtime rate")
        return 2
    print("RESULT: PASS realtime polling")
    return 0


def cmd_hall(args, link: VescDual) -> int:
    require_arm(args)
    left, right = motor_flags(args.motor)
    result = 0
    for name, is_right in (("LEFT", False), ("RIGHT", True)):
        if (is_right and not right) or ((not is_right) and not left):
            continue
        before = link.diag(is_right)
        print_diag(name + " BEFORE", before)
        print(f"{name}: Hall detect {args.amps:.2f} A ...")
        ok, table = link.detect_hall(args.amps, is_right)
        valid, why = hall_table_valid(table)
        after = link.diag(is_right)
        print(f"{name}: protocol_result={'OK' if ok else 'FAIL'} table={table} {why}")
        print_diag(name + " AFTER", after)
        # Sesuai VESC 6.00, COMM_DETECT_HALL_FOC hanya mengembalikan tabel
        # hasil deteksi lalu memulihkan konfigurasi lama. Apply/store dilakukan
        # oleh VESC Tool melalui MC Config atau oleh Detect All FOC.
        if not ok or not valid:
            print(f"{name}: RESULT FAIL (detect/table invalid)")
            result = 2
        else:
            print(f"{name}: RESULT PASS standalone Hall Detect (no auto-store, sesuai VESC 6.00)")
    return result


def run_motion_test(args, link: VescDual, motor: str, cmd: int, raw: int, label: str) -> int:
    require_arm(args)
    is_right = motor == "right"
    before = link.diag(is_right)
    print_diag("BEFORE", before)
    sender = lambda: send_one(link, motor, cmd, raw)
    samples: list[tuple[float, Values, Diag | None]] = []
    start = time.monotonic()
    try:
        # Deterministic single-UART scheduler. Do not run a background SET_*
        # writer concurrently with VESC request/reply transactions: even with a
        # byte-level lock that can reorder reply expectations under load.
        end = start + args.seconds
        next_cmd = start
        next_sample = start
        next_diag = start
        cmd_period = 1.0 / max(args.hz, 1.0)
        latest_diag = None
        refresh_count = 0
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_cmd:
                sender(); refresh_count += 1
                next_cmd = now + cmd_period
            if now >= next_diag:
                latest_diag = link.diag(is_right)
                next_diag = now + 0.20
            if now >= next_sample:
                v = link.values(is_right)
                samples.append((now - start, v, latest_diag))
                # Hardware safety envelope. A low-energy diagnostic must never
                # silently become an unloaded high-speed run.
                speed_limit = 2000.0
                if cmd == COMM_SET_RPM:
                    speed_limit = max(1500.0, abs(float(raw)) * 1.8)
                if v.fault or abs(v.rpm) > speed_limit or abs(v.current_motor) > 5.0:
                    print(f"MOTION_SAFETY_ABORT fault={v.fault} erpm={v.rpm:.0f} Imotor={v.current_motor:.2f}A")
                    return 2
                next_sample = now + 0.05
            time.sleep(0.002)
        print(f"setpoint refresh count={refresh_count} errors=0")
    finally:
        release_one(link, motor)
    time.sleep(0.10)
    after = link.diag(is_right)
    print_diag("AFTER", after)
    if samples:
        print_values("LAST", samples[-1][1])
        if samples[-1][2] is not None:
            print_diag("ACTIVE", samples[-1][2])
    if not samples:
        print("RESULT: FAIL tidak ada telemetry")
        return 2
    if after.fault:
        print(f"RESULT: FAIL fault={after.fault}")
        return 2
    print(f"RESULT: {label} test selesai; lihat target/ref/actual di atas")
    return 0


def cmd_current(args, link: VescDual) -> int:
    left, right = motor_flags(args.motor)
    if left and right:
        raise SystemExit("current test jalankan satu motor: --motor left atau right")
    motor = "right" if right else "left"
    raw = round(args.amps * 1000.0)
    rc = run_motion_test(args, link, motor, COMM_SET_CURRENT, raw, f"CURRENT {args.amps:.3f}A")
    # Verifikasi jalur command saat paket masih direfresh. Satu paket 30 ms
    # setelah release terlalu singkat dan menghasilkan false-negative pada DMA/queue.
    require_arm(args)
    sender = lambda: send_one(link, motor, COMM_SET_CURRENT, raw)
    # Same deterministic request/reply rule as run_motion_test: refresh a few
    # times in-band, then read the active diagnostic without a writer thread.
    for _ in range(3):
        sender(); time.sleep(0.025)
    active = link.diag(right)
    release_one(link, motor)
    print(f"COMMAND_PATH Iq_target={active.iq_target_a:.3f}A expected={args.amps:.3f}A mode={active.control_mode} ownership={active.override}")
    if abs(active.iq_target_a - args.amps) > 0.06 or not active.override:
        print("CURRENT_COMMAND_PATH_FAIL")
        return 2
    print("CURRENT_COMMAND_PATH_PASS")
    return rc


def cmd_rpm(args, link: VescDual) -> int:
    left, right = motor_flags(args.motor)
    if left and right:
        raise SystemExit("rpm test jalankan satu motor: --motor left atau right")
    motor = "right" if right else "left"
    dcfg = link.diag(right)
    pp = dcfg.pole_pairs or POLE_PAIRS
    erpm = int(round(args.erpm if args.erpm is not None else args.mech_rpm * pp))
    gear = dcfg.gear_ratio or 1.0
    motor_rpm = erpm / pp
    output_rpm = motor_rpm / gear
    print(f"RPM target = {erpm} ERPM | poles={2*pp} pp={pp} | "
          f"motor={motor_rpm:.3f} RPM | gear={gear:.3f} | output={output_rpm:.3f} RPM")
    return run_motion_test(args, link, motor, COMM_SET_RPM, erpm, f"RPM {erpm} ERPM")


def cmd_pos_vesc(args, link: VescDual) -> int:
    require_arm(args)
    if not 0.0 <= args.deg <= 360.0:
        raise SystemExit("VESC Tool/COMM_SET_POS standard dibatasi 0..360 derajat pada utility ini")
    left, right = motor_flags(args.motor)
    if left and right:
        raise SystemExit("pos-vesc test jalankan satu motor")
    motor = "right" if right else "left"
    raw = round(args.deg * 1_000_000.0)
    return run_motion_test(args, link, motor, COMM_SET_POS, raw, f"VESC POS {args.deg:.3f}deg")


def cmd_pos_limits(args, link: VescDual) -> int:
    if not INT32_MIN <= args.minimum <= INT32_MAX or not INT32_MIN <= args.maximum <= INT32_MAX:
        raise SystemExit("min/max harus signed int32")
    if args.minimum > args.maximum:
        raise SystemExit("min harus <= max")
    left, right = motor_flags(args.motor)
    if left:
        print("L", link.set_position_limits(args.minimum, args.maximum, False))
    if right:
        print("R", link.set_position_limits(args.minimum, args.maximum, True))
    return 0


def cmd_pos_state(args, link: VescDual) -> int:
    left, right = motor_flags(args.motor)
    if left: print("L", link.position_state(False))
    if right: print("R", link.position_state(True))
    return 0


def cmd_pos_reset(args, link: VescDual) -> int:
    left, right = motor_flags(args.motor)
    if left: print("L", link.reset_position(False))
    if right: print("R", link.reset_position(True))
    return 0


def cmd_pos_count(args, link: VescDual) -> int:
    require_arm(args)
    if not INT32_MIN <= args.count <= INT32_MAX:
        raise SystemExit("count harus signed int32")
    left, right = motor_flags(args.motor)
    if left and right:
        raise SystemExit("pos-count test jalankan satu motor")
    is_right = right
    motor = "right" if right else "left"
    def sender():
        link.set_position_counts(args.count, is_right)
    try:
        with Refresher(sender, args.hz) as ref:
            end = time.monotonic() + args.seconds
            while time.monotonic() < end:
                st = link.position_state(is_right)
                d = link.diag(is_right)
                print(f"{motor}: current={st.current} target={st.target} limits=[{st.minimum},{st.maximum}] Iq={d.iq_a:.3f}A")
                if d.fault or abs(d.iq_a) > 0.90 or abs(st.current - st.target) > 4:
                    print(f"POSITION_SAFETY_ABORT fault={d.fault} Iq={d.iq_a:.3f} error={st.target-st.current}")
                    return 2
                time.sleep(0.20)
            print(f"refresh count={ref.count} errors={len(ref.errors)}")
            final = link.position_state(is_right)
            if final.current != final.target:
                print(f"POSITION_NOT_SETTLED current={final.current} target={final.target}")
                return 2
            print("POSITION_SETTLED_PASS")
    finally:
        release_one(link, motor)
    return 0



def _phase_diff_deg(raw_a: int, raw_b: int) -> float:
    d = (int(raw_a) - int(raw_b)) & 0xFFFF
    if d >= 0x8000:
        d -= 0x10000
    return d * 360.0 / 65536.0


def cmd_hall_phase(args, link: VescDual) -> int:
    """Active proof of VESC Hall table -> electrical phase -> FOC phase.

    VESC FOC Hall values are 0..199 for 0..360 electrical degrees; 255 is
    invalid. During sensored closed-loop m_phase must be the corrected/rate-
    limited Hall phase, while interpolation stays within the current sector.
    """
    require_arm(args)
    left, right = motor_flags(args.motor)
    if left and right:
        raise SystemExit("hall-phase test jalankan satu motor")
    is_right = right
    motor = "right" if right else "left"
    if args.erpm is not None:
        erpm = int(round(args.erpm))
    elif args.mech_rpm is not None:
        cfg = link.diag(is_right)
        erpm = int(round(args.mech_rpm * (cfg.pole_pairs or POLE_PAIRS)))
    else:
        erpm = 750
    if abs(erpm) < 300 or abs(erpm) > 1200:
        raise SystemExit("hall-phase safety window: |ERPM| harus 300..1200")

    before = link.diag(is_right)
    seq0 = before.hall_sequence_rejects if before.hall_sequence_rejects is not None else before.hall_invalid
    per0 = before.hall_period_rejects or 0
    trip0 = before.current_trips
    pos0 = before.position
    seen: set[int] = set()
    checked = 0
    max_center_err = 0.0
    max_phase_use_err = 0.0
    sender = lambda: send_one(link, motor, COMM_SET_RPM, erpm)
    start = time.monotonic()
    try:
        with Refresher(sender, min(max(args.hz, 20.0), 60.0)) as ref:
            end = start + max(args.seconds, 2.0)
            while time.monotonic() < end:
                d = link.diag(is_right)
                elapsed = time.monotonic() - start
                if d.fault or d.current_trips != trip0:
                    print(f"HALL_PHASE_FAIL fault/trip fault={d.fault} trips={trip0}->{d.current_trips}")
                    return 2
                speed_limit=max(1500.0,abs(float(erpm))*1.8)
                if abs(d.erpm)>speed_limit or abs(d.iq_a)>5.0:
                    print(f"HALL_PHASE_FAIL safety erpm={d.erpm} Iq={d.iq_a:.2f}A limit={speed_limit:.0f}")
                    return 2
                if d.phase_raw is None or d.phase_hall_raw is None or d.phase_target_raw is None:
                    print("HALL_PHASE_FAIL firmware diagnostic extension belum tersedia")
                    return 2
                if d.hall in range(1, 7):
                    table_angle = d.hall_table[d.hall]
                    if table_angle >= 200 or d.hall_angle200 != table_angle:
                        print(f"HALL_PHASE_FAIL table mismatch state={d.hall} diag={d.hall_angle200} table={table_angle}")
                        return 2
                    center_raw = (table_angle * 65536) // 200
                    center_err = abs(_phase_diff_deg(d.phase_hall_raw, center_raw))
                    phase_use_err = abs(_phase_diff_deg(d.phase_raw, d.phase_hall_raw))
                    if elapsed > 0.45:
                        seen.add(d.hall); checked += 1
                        max_center_err = max(max_center_err, center_err)
                        max_phase_use_err = max(max_phase_use_err, phase_use_err)
                        # Jangan jadikan jumlah state yang tertangkap serial sebagai bukti
                        # coverage. Pada 750 ERPM ada ~75 edge Hall/s, sedangkan satu
                        # transaksi diagnostic besar hanya puluhan Hz sehingga sampling
                        # dapat alias tepat ke 2 state. VESC juga rate-limit koreksi Hall;
                        # sesaat setelah edge, fase yang dipakai boleh tertinggal dari
                        # center sektor. Yang wajib: FOC benar-benar memakai phase Hall,
                        # tidak keluar lebih dari satu sektor, dan ISR menerima edge tanpa
                        # reject. Coverage final dibuktikan dari counter posisi ISR.
                        if center_err > 70.0 or phase_use_err > 3.0:
                            print(f"HALL_PHASE_FAIL state={d.hall} table={table_angle}/200 center_err={center_err:.1f}deg phase_use_err={phase_use_err:.1f}deg erpm={d.erpm}")
                            return 2
                time.sleep(0.04)
            print(f"refresh count={ref.count} errors={len(ref.errors)}")
    finally:
        release_one(link, motor)

    after = link.diag(is_right)
    seq1 = after.hall_sequence_rejects if after.hall_sequence_rejects is not None else after.hall_invalid
    per1 = after.hall_period_rejects or 0
    edge_delta = abs(int(after.position) - int(pos0))
    print(f"HALL_PHASE_RESULT motor={motor} cmd={erpm:+d} samples={checked} sampled_states={sorted(seen)} "
          f"accepted_edges={edge_delta} max_center_err={max_center_err:.1f}deg "
          f"max_phase_use_err={max_phase_use_err:.2f}deg seq_reject={seq0}->{seq1} "
          f"period_reject={per0}->{per1}")
    if checked < 4 or edge_delta < 6:
        print(f"HALL_PHASE_FAIL coverage kurang: samples={checked} accepted_edges={edge_delta}")
        return 2
    if seq1 != seq0:
        print("HALL_PHASE_FAIL ada electrical Hall sequence reject")
        return 2
    if after.fault or after.current_trips != trip0:
        print("HALL_PHASE_FAIL fault/current-trip sesudah test")
        return 2
    print("HALL_PHASE_PASS table200->360e->FOC phase aktif konsisten")
    return 0

def cmd_wiring_check(args, link: VescDual) -> int:
    """Safe recommissioning workflow after changing phase or Hall wiring."""
    require_arm(args)
    rc = 0
    print("=== WIRING CHECK: MOTOR HARUS BEBAS BERPUTAR / RODA TERANGKAT ===")
    print("=== DETECT ALL FOC: DETECT + APPLY + STORE BOTH MOTORS ===")
    try:
        detect_result = link.detect_all_foc(max_power_loss=50.0, min_current_in=-8.0,
                                            max_current_in=8.0, openloop_rpm=250.0,
                                            sl_erpm=2500.0, detect_can=True)
    except Exception as exc:
        print(f"WIRING_CHECK_FAIL Detect All exception: {exc}")
        return 2
    print(f"DETECT_ALL_RESULT={detect_result}")
    if detect_result < 0:
        print("WIRING_CHECK_FAIL Detect All returned error")
        return 2
    for motor, is_right in (("left", False), ("right", True)):
        motor_rc = 0
        d=link.diag(is_right)
        ok,why=hall_table_valid(d.hall_table)
        if not ok or not d.hall_store_ok:
            print(f"WIRING_CHECK_FAIL {motor}: Hall table/persistence {why}")
            rc=2
            continue
        print(f"{motor.upper()} Hall aktif+stored {d.hall_table}")
        for amps in (0.10, -0.10):
            print(f"=== {motor.upper()} CURRENT {amps:+.2f} A ===")
            class C: pass
            c=C(); c.arm=True; c.motor=motor; c.amps=amps; c.seconds=0.8; c.hz=50.0
            if cmd_current(c, link):
                motor_rc=2
                break
        if motor_rc == 0:
            print(f"=== {motor.upper()} HALL/FOC PHASE +750 ERPM ===")
            class P: pass
            p=P(); p.arm=True; p.motor=motor; p.erpm=750.0; p.mech_rpm=None; p.seconds=2.0; p.hz=40.0
            if cmd_hall_phase(p, link): motor_rc=2
        release_one(link,motor)
        if motor_rc: rc=2
    print("WIRING_CHECK_RESULT:", "PASS - wiring learned and low-energy direction/current checks passed" if rc==0 else "FAIL - jangan gunakan normal drive sebelum diperbaiki")
    return rc


def cmd_all(args, link: VescDual) -> int:
    rc = 0
    print("=== INFO ===")
    rc |= cmd_info(args, link)
    print("=== RT 50 HZ, 3 s ===")
    class Tmp: pass
    rt = Tmp(); rt.hz=50.0; rt.seconds=3.0; rt.motor="both"; rt.csv=None
    rc |= cmd_rt(rt, link)
    print("=== POSITION API ===")
    print("L", link.position_state(False)); print("R", link.position_state(True))
    if args.arm:
        print("=== HALL DETECT BOTH 1.0 A ===")
        class H: pass
        h=H(); h.arm=True; h.motor="both"; h.amps=1.0
        rc |= cmd_hall(h, link)
        for motor in ("left", "right"):
            print(f"=== CURRENT {motor.upper()} 0.2 A / 0.3 s ===")
            class C: pass
            c=C(); c.arm=True;c.motor=motor;c.amps=0.2;c.seconds=0.3;c.hz=50.0
            rc |= cmd_current(c, link)
            print(f"=== RPM {motor.upper()} +750 ERPM / 2 s ===")
            class R: pass
            r=R();r.arm=True;r.motor=motor;r.erpm=750.0;r.mech_rpm=None;r.seconds=2.0;r.hz=50.0
            rc |= cmd_rpm(r, link)
        print("=== POSITION +/-1 COUNT BOTH ===")
        link.reset_position(False); link.reset_position(True)
        link.set_position_limits(-20,20,False); link.set_position_limits(-20,20,True)
        for motor,is_right in (("left",False),("right",True)):
            for target in (1,-1):
                link.reset_position(is_right)
                class P: pass
                p=P();p.arm=True;p.motor=motor;p.count=target;p.seconds=1.5;p.hz=30.0
                rc |= cmd_pos_count(p, link)
    else:
        print("Motor-moving tests dilewati; tambahkan --arm untuk Hall/current/RPM/position.")
    print("ALL_TESTS_RESULT:", "PASS" if rc == 0 else f"CHECK rc={rc}")
    return rc


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="VESC 6.00 dual STM32F103 hardware/protocol debugger")
    p.add_argument("port", nargs="?", default="auto")
    p.add_argument("command", nargs="?", default="info",
                   choices=("selftest","info","diag","rt","hall","hall-phase","wiring-check","current","rpm","pos-vesc","pos-limits","pos-state","pos-reset","pos-count","all"))
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--motor", default="both", choices=("left","right","both"))
    p.add_argument("--hz", type=float, default=50.0)
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("--amps", type=float, default=1.0)
    p.add_argument("--erpm", type=float)
    p.add_argument("--mech-rpm", type=float)
    p.add_argument("--deg", type=float, default=0.0)
    p.add_argument("--min", dest="minimum", type=int, default=INT32_MIN)
    p.add_argument("--max", dest="maximum", type=int, default=INT32_MAX)
    p.add_argument("--count", type=int, default=0)
    p.add_argument("--csv")
    p.add_argument("--arm", action="store_true", help="izinkan test yang memberi energi ke motor")
    return p


def main() -> int:
    argv = sys.argv[1:]
    # shorthand offline: `python3 tools/vesc_debug.py selftest`
    if argv and argv[0] == "selftest":
        argv.insert(0, "/dev/ttyUSB0")
    args = build_parser().parse_args(argv)
    if args.command == "selftest":
        return cmd_selftest(args)
    if args.command == "rpm" and args.erpm is None and args.mech_rpm is None:
        raise SystemExit("rpm memerlukan --erpm N atau --mech-rpm N")
    link = VescDual(args.port, args.baud, timeout=0.25)
    try:
        if args.command == "info": return cmd_info(args, link)
        if args.command == "diag": return cmd_diag(args, link)
        if args.command == "rt": return cmd_rt(args, link)
        if args.command == "hall": return cmd_hall(args, link)
        if args.command == "hall-phase": return cmd_hall_phase(args, link)
        if args.command == "wiring-check": return cmd_wiring_check(args, link)
        if args.command == "current": return cmd_current(args, link)
        if args.command == "rpm": return cmd_rpm(args, link)
        if args.command == "pos-vesc": return cmd_pos_vesc(args, link)
        if args.command == "pos-limits": return cmd_pos_limits(args, link)
        if args.command == "pos-state": return cmd_pos_state(args, link)
        if args.command == "pos-reset": return cmd_pos_reset(args, link)
        if args.command == "pos-count": return cmd_pos_count(args, link)
        if args.command == "all": return cmd_all(args, link)
        raise AssertionError(args.command)
    finally:
        link.close()


if __name__ == "__main__":
    raise SystemExit(main())
