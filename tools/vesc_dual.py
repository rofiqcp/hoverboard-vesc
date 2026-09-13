#!/usr/bin/env python3
"""VESC-standard USART3 client for the bare-metal STM32F103 dual hoverboard port.

Motor mapping:
  Left  = local VESC serial controller
  Right = virtual CAN controller ID 2 using COMM_FORWARD_CAN

The control worker refreshes setpoints at 50 Hz (VESC timeout is 500 ms) and
polls selective mc_values telemetry from both motors.
"""
from __future__ import annotations
import argparse
import os
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

try:
    import serial
except ImportError:
    serial = None

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_POS = 9
COMM_SET_HANDBRAKE = 10
COMM_SET_DETECT = 11
COMM_SET_MCCONF = 13
COMM_GET_MCCONF = 14
COMM_GET_MCCONF_DEFAULT = 15
COMM_SET_APPCONF = 16
COMM_GET_APPCONF = 17
COMM_GET_APPCONF_DEFAULT = 18
COMM_TERMINAL_CMD = 20
COMM_PRINT = 21
COMM_ROTOR_POSITION = 22
COMM_DETECT_MOTOR_R_L = 25
COMM_DETECT_MOTOR_FLUX_LINKAGE = 26
COMM_DETECT_ENCODER = 27
COMM_REBOOT = 29
COMM_DETECT_HALL_FOC = 28
COMM_ALIVE = 30
COMM_GET_DECODED_ADC = 32
COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36
COMM_GET_VALUES_SETUP = 47
COMM_SET_MCCONF_TEMP = 48
COMM_SET_MCCONF_TEMP_SETUP = 49
COMM_GET_VALUES_SELECTIVE = 50
COMM_GET_VALUES_SETUP_SELECTIVE = 51
COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57
COMM_DETECT_APPLY_ALL_FOC = 58
COMM_PING_CAN = 62
COMM_APP_DISABLE_OUTPUT = 63
COMM_TERMINAL_CMD_SYNC = 64
COMM_SET_CURRENT_REL = 84
COMM_SET_BATTERY_CUT = 86
COMM_GET_MCCONF_TEMP = 91
COMM_SET_ODOMETER = 110
COMM_GET_BATTERY_CUT = 115
COMM_SET_APPCONF_NO_STORE = 149
COMM_SHUTDOWN = 156
RIGHT_ID = 2
POLE_PAIRS = 15
STOP_ERPM = 5 * POLE_PAIRS  # 5 mechanical rpm
DEFAULT_BAUD = 115200
EXPECTED_LOCAL_TARGETS = {"motor_left", "f103rc_bootloader"}
# STM32F1 EEPROM emulation can compact a 205-variable page on persistent writes.
# Keep normal request/reply deadlines short; only commands that explicitly store
# configuration get this bounded hardware-aware deadline.
PERSISTENT_WRITE_TIMEOUT = 8.0


def _stable_serial_path(device: str) -> str:
    """Prefer /dev/serial/by-id so an F103 USB-UART keeps a stable name."""
    if os.name != "posix":
        return device
    root = Path("/dev/serial/by-id")
    try:
        target = os.path.realpath(device)
        for q in sorted(root.iterdir()):
            if os.path.realpath(str(q)) == target:
                return str(q)
    except OSError:
        pass
    return device


def _direct_serial_candidates() -> list[tuple[str, str]]:
    """Enumerate generic USB-UART ports; no intermediate MCU gateway assumptions."""
    if serial is None:
        return []
    import serial.tools.list_ports
    ranked=[]
    for q in serial.tools.list_ports.comports():
        dev=_stable_serial_path(q.device)
        meta=" ".join(str(x or "") for x in (q.description,q.manufacturer,q.hwid)).lower()
        if q.vid is None and "/dev/ttyUSB" not in q.device and "/dev/ttyACM" not in q.device and "usb" not in meta:
            continue
        score=0
        if q.vid is not None: score+=100
        if "ch340" in meta or "ch341" in meta or q.vid==0x1A86: score+=80
        if "cp210" in meta or q.vid==0x10C4: score+=60
        if "ftdi" in meta or q.vid==0x0403: score+=50
        if "/dev/serial/by-id/" in dev: score+=30
        if "/dev/ttyUSB" in q.device: score+=20
        ranked.append((-score,dev,q.description or "USB serial"))
    ranked.sort(key=lambda x:(x[0],x[1]))
    out=[]
    seen=set()
    for _,dev,desc in ranked:
        if dev not in seen:
            seen.add(dev); out.append((dev,desc))
    return out


def _probe_f103_uart(path: str, baud: int = DEFAULT_BAUD, timeout: float = 0.01):
    """Open and positively identify an F103 VESC/bootloader on a USB-UART."""
    if serial is None:
        raise RuntimeError("pyserial required: python -m pip install pyserial")
    ser=serial.Serial(path,baud,timeout=max(0.002,timeout),write_timeout=2,exclusive=True)
    try:
        try: ser.reset_input_buffer(); ser.reset_output_buffer()
        except Exception: pass
        dec=PacketDecoder()
        ser.write(frame(bytes((COMM_FW_VERSION,)))); ser.flush()
        deadline=time.monotonic()+0.8
        while time.monotonic()<deadline:
            data=ser.read(ser.in_waiting or 1)
            for payload in dec.feed(data):
                if payload and payload[0]==COMM_FW_VERSION and len(payload)>=4:
                    end=payload.find(b"\0",3)
                    name=payload[3:end if end>=0 else len(payload)].decode(errors="replace")
                    if name not in EXPECTED_LOCAL_TARGETS:
                        raise RuntimeError(f"unexpected VESC target {name!r}; expected direct F103 {sorted(EXPECTED_LOCAL_TARGETS)}")
                    return ser,name
        raise TimeoutError("no COMM_FW_VERSION reply")
    except Exception:
        ser.close(); raise


def open_transport(port: str, baud: int = DEFAULT_BAUD, timeout: float = 0.01):
    """Open the direct PC/NUC <-> F103 USART3 USB-UART link."""
    target=(port or "auto").strip()
    if target in {"direct","usb"}:
        target="auto"
    if target=="auto":
        errors=[]
        for dev,desc in _direct_serial_candidates():
            try:
                ser,name=_probe_f103_uart(dev,baud,timeout)
                print(f"[VESC-AUTO] direct F103 UART selected {dev} ({desc}) target={name}",flush=True)
                return ser
            except Exception as exc:
                errors.append(f"{dev}:{type(exc).__name__}")
        if not errors:
            raise RuntimeError("no USB-UART ports detected for F103")
        raise RuntimeError("no F103 VESC target responded: "+", ".join(errors))
    if target.startswith("direct:"):
        target=target.split(":",1)[1]
    # Explicit device names are not trusted implicitly. Positively identify
    # the direct F103 endpoint exactly like AUTO so a wrong USB-UART cannot
    # receive motor/config traffic merely because the path exists.
    ser,name=_probe_f103_uart(target,baud,timeout)
    print(f"[VESC-DIRECT] verified {target} target={name}",flush=True)
    return ser

HB_MAGIC = b"HB"
HB_VERSION = 2
HB_GET_DIAG = 1
HB_GET_POS_STATE = 2
HB_SET_POS_LIMITS = 3
HB_SET_POS_TARGET = 4
HB_RESET_POSITION = 5
HB_GET_TUNING = 6
HB_SET_TUNING = 7
HB_SET_ID_TEST = 8
HB_SET_STEERING_DEG = 9
HB_GET_STEERING_CAL = 10
HB_STEERING_HOME = 13
HB_ENCODER_DEBUG = 14
HB_GET_ISR_PROFILE = 17
HB_GET_TRACE_META = 18
HB_GET_TRACE_SAMPLE = 19
HB_CLEAR_TRACE = 20
HB_GET_PLATFORM_HEALTH = 21
HB_GET_COMMS_HEALTH = 24
HB_GET_PLATFORM_INFO = 25
HB_GET_ADC_VALIDITY = 26
HB_ARM_CURRENT_STEP = 27
HB_GET_STEP_STATUS = 28
HB_GET_POSITION_D_STATE = 30
HB_BOOT_HANDOFF = 29
HB_PLATFORM_SCHEMA = 2
HB_DIAG_SCHEMA = 4
HB_ISR_SCHEMA = 3
HB_TRACE_SCHEMA = 3
HB_ISR_PROFILE_REVISION = 0x00030000
HB_FREEZE_TRACE = 22

# currentMotor,currentIn,Id,Iq,duty,rpm,Vin,fault,vescId,Vd,Vq
VALUE_MASK = sum(1 << b for b in (2, 3, 4, 5, 6, 7, 8, 15, 16, 17, 19, 20))


def crc16(data: bytes) -> int:
    crc = 0
    for x in data:
        crc ^= x << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    n = len(payload)
    if not 0 < n <= 65535:
        raise ValueError("invalid VESC payload size")
    if n <= 255:
        head = bytes((2, n))
    else:
        head = bytes((3, (n >> 8) & 0xFF, n & 0xFF))
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


class PacketDecoder:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while self.buf:
            try:
                start = next(i for i, b in enumerate(self.buf) if b in (2, 3, 4))
            except StopIteration:
                self.buf.clear()
                break
            if start:
                del self.buf[:start]
            if not self.buf:
                break
            kind = self.buf[0]
            h = kind
            if len(self.buf) < h:
                break
            if kind == 2:
                n = self.buf[1]
            elif kind == 3:
                n = (self.buf[1] << 8) | self.buf[2]
                if n < 255:
                    del self.buf[0]
                    continue
            else:
                n = (self.buf[1] << 16) | (self.buf[2] << 8) | self.buf[3]
                if n < 65535:
                    del self.buf[0]
                    continue
            total = h + n + 3
            if len(self.buf) < total:
                break
            raw = bytes(self.buf[:total])
            del self.buf[:total]
            if raw[-1] != 3:
                continue
            payload = raw[h:h+n]
            rx_crc = (raw[h+n] << 8) | raw[h+n+1]
            if crc16(payload) == rx_crc:
                out.append(payload)
        return out


@dataclass
class Values:
    current_motor: float = 0.0
    current_in: float = 0.0
    id: float = 0.0
    iq: float = 0.0
    duty: float = 0.0
    rpm: float = 0.0
    vin: float = 0.0
    position: float = 0.0
    fault: int = 0
    vesc_id: int = 0
    vd: float = 0.0
    vq: float = 0.0

    def short(self) -> str:
        return (f"id={self.vesc_id} rpm={self.rpm:.0f} duty={100*self.duty:.1f}% "
                f"Imot={self.current_motor:.2f}A Iin={self.current_in:.2f}A "
                f"Id={self.id:.2f}A Iq={self.iq:.2f}A Vd={self.vd:.2f}V "
                f"Vq={self.vq:.2f}V Vin={self.vin:.1f}V pos={self.position:.2f}deg fault={self.fault}")



@dataclass
class SetupValues:
    """Field COMM_GET_VALUES_SETUP VESC Tool 6.00 dalam satuan fisik."""
    temp_mos: float
    temp_motor: float
    current_total: float
    current_in_total: float
    duty: float
    rpm: float
    speed_m_s: float
    vin: float
    battery_level: float
    amp_hours: float
    amp_hours_charged: float
    watt_hours: float
    watt_hours_charged: float
    distance_m: float
    distance_abs_m: float
    position_deg: float
    fault: int
    vesc_id: int
    num_vescs: int
    wh_battery_left: float
    odometer_m: int
    uptime_ms: int


@dataclass
class McconfTemp:
    """Subset limit runtime yang dibaca COMM_GET_MCCONF_TEMP VESC 6.00."""
    current_min_scale: float
    current_max_scale: float
    min_erpm: float
    max_erpm: float
    min_duty: float
    max_duty: float
    watt_min: float
    watt_max: float
    input_current_min: float
    input_current_max: float
    motor_poles: int
    gear_ratio: float
    wheel_diameter: float


@dataclass
class PositionState:
    current: int
    target: int
    minimum: int
    maximum: int


@dataclass
class Tuning:
    kpq: int; kiq: int; kpd: int; kid: int
    kps: int; kis: int; kds: int
    kpp: int; kip: int; kdp: int
    telem_filter_q16: int = 6553
    current_limit_q4: int = 800

    @property
    def physical(self):
        return dict(foc_q_kp=self.kpq/1536.0, foc_q_ki=self.kiq/4.608,
                    foc_d_kp=self.kpd/1536.0, foc_d_ki=self.kid/4.608,
                    speed_kp=self.kps/100000.0, speed_ki=self.kis/100000.0, speed_kd=self.kds/100000.0,
                    pos_kp=self.kpp/1000.0, pos_ki=self.kip/1000.0, pos_kd=self.kdp/1000.0,
                    telemetry_filter=self.telem_filter_q16/65535.0, current_limit_a=self.current_limit_q4/800.0)


@dataclass
class Diag:
    vesc_id: int
    control_mode: int
    state: int
    fault: int
    hall: int
    override: bool
    hall_store_ok: bool
    link_armed: bool
    iq_target_a: float
    iq_ref_a: float
    iq_a: float
    id_a: float
    erpm: int
    duty: float
    position: int
    position_target: int
    position_min: int
    position_max: int
    hall_invalid: int
    current_trips: int
    rx_ok: int
    rx_crc_errors: int
    hall_table: list[int]
    hall_angle200: int | None = None
    hall_edge200: int | None = None
    hall_center200: int | None = None
    hall_direction: int | None = None
    hall_interp: bool | None = None
    hall_last_reject_reason: int | None = None
    hall_last_reject_from: int | None = None
    hall_last_reject_to: int | None = None
    phase_raw: int | None = None
    phase_hall_raw: int | None = None
    phase_target_raw: int | None = None
    hall_period: int | None = None
    hall_ticks: int | None = None
    hall_period_rejects: int | None = None
    hall_sequence_rejects: int | None = None
    current_offset_phase0: int | None = None
    current_offset_phase1: int | None = None
    current_offset_dc: int | None = None
    motor_poles: int | None = None
    pole_pairs: int | None = None
    gear_ratio: float | None = None
    motor_mech_rpm: float | None = None
    output_rpm: float | None = None
    rx_queue_drops: int | None = None
    foc_isr_cycles: int | None = None
    foc_isr_cycles_max: int | None = None
    phase_trip_count: int | None = None
    dc_trip_count: int | None = None
    phase_overcurrent_streak: int | None = None
    last_trip_source: int | None = None
    last_trip_phase0_a: float | None = None
    last_trip_phase1_a: float | None = None
    last_trip_phase2_a: float | None = None
    last_trip_dc_a: float | None = None
    last_trip_duty: float | None = None
    driven_offset0: int | None = None
    driven_offset1: int | None = None
    driven_offset_dc: int | None = None
    driven_offset_samples: int | None = None
    driven_offset_valid: bool | None = None
    driven_offset_calibrating: bool | None = None
    raw_adc_phase0: int | None = None
    raw_adc_phase1: int | None = None
    raw_adc_dc: int | None = None
    off_offset0: int | None = None
    off_offset1: int | None = None
    off_offset_dc: int | None = None
    off_offset_samples: int | None = None
    off_settle_ticks: int | None = None
    off_offset_valid: bool | None = None
    current_offset_valid: bool | None = None
    tx_queue_drops: int | None = None
    tx_start_failures: int | None = None
    rx_queue_highwater: int | None = None
    process_gap_max_ms: int | None = None
    usart3_rx_errors: int | None = None
    usart3_rx_restarts: int | None = None
    isr_count: int | None = None
    position_no_motion_ticks: int | None = None
    position_breakaway_ticks: int | None = None
    position_last_motion_count: int | None = None
    position_error_counts: int | None = None
    prof_sensor_max_cycles: int | None = None
    prof_current_max_cycles: int | None = None
    prof_regulator_max_cycles: int | None = None
    prof_svpwm_max_cycles: int | None = None
    profiler_reentry_guard_total: int | None = None

    def short(self) -> str:
        return (
            f"id={self.vesc_id} mode={self.control_mode} state={self.state} fault={self.fault} "
            f"hall={self.hall} arm={int(self.link_armed)} own={int(self.override)} Iq_tgt={self.iq_target_a:.3f}A "
            f"Iq_ref={self.iq_ref_a:.3f}A Iq={self.iq_a:.3f}A Id={self.id_a:.3f}A "
            f"erpm={self.erpm} duty={100*self.duty:.2f}% pos={self.position} "
            f"target={self.position_target} trips={self.current_trips} hall_bad={self.hall_invalid}"
        )


def parse_custom_header(payload: bytes, op: int) -> int:
    if len(payload) < 6 or payload[0] != COMM_CUSTOM_APP_DATA:
        raise ValueError("not COMM_CUSTOM_APP_DATA")
    if payload[1:3] != HB_MAGIC or payload[3] != HB_VERSION or payload[4] != op:
        raise ValueError("custom app header mismatch")
    return payload[5]


def parse_position_state(payload: bytes, op: int) -> PositionState:
    status = parse_custom_header(payload, op)
    if status:
        raise RuntimeError(f"custom position command failed status={status}")
    if len(payload) < 22:
        raise ValueError(f"short position state reply: {len(payload)}")
    return PositionState(*struct.unpack_from(">iiii", payload, 6))


def parse_diag(payload: bytes) -> Diag:
    status = parse_custom_header(payload, HB_GET_DIAG)
    if status:
        raise RuntimeError(f"diagnostic command failed status={status}")
    if len(payload) < 78:
        raise ValueError(f"short diagnostic reply: {len(payload)}")
    vid, mode, state, fault, hall, own, store_ok, link_armed = struct.unpack_from(">8B", payload, 6)
    vals = struct.unpack_from(">10i", payload, 14)
    hall_invalid, trips, rx_ok, rx_crc = struct.unpack_from(">4I", payload, 54)
    table = list(payload[70:78])
    ext = {}
    if len(payload) >= 104:
        (ang200, edge200, center200, direction_u8, interp, rej_reason, rej_from, rej_to) = struct.unpack_from(">8B", payload, 78)
        phase_raw, phase_hall_raw, phase_target_raw, hall_period, hall_ticks = struct.unpack_from(">5H", payload, 86)
        period_rej, sequence_rej = struct.unpack_from(">2I", payload, 96)
        direction = direction_u8 - 256 if direction_u8 >= 128 else direction_u8
        ext = dict(hall_angle200=ang200, hall_edge200=edge200, hall_center200=center200,
                   hall_direction=direction, hall_interp=bool(interp),
                   hall_last_reject_reason=rej_reason, hall_last_reject_from=rej_from,
                   hall_last_reject_to=rej_to, phase_raw=phase_raw,
                   phase_hall_raw=phase_hall_raw, phase_target_raw=phase_target_raw,
                   hall_period=hall_period, hall_ticks=hall_ticks,
                   hall_period_rejects=period_rej, hall_sequence_rejects=sequence_rej)
    if len(payload) >= 124:
        po0, po1, dco = struct.unpack_from(">3h", payload, 104)
        poles, pp = struct.unpack_from(">2B", payload, 110)
        gear_milli, mech_milli, out_milli = struct.unpack_from(">3i", payload, 112)
        ext.update(current_offset_phase0=po0, current_offset_phase1=po1, current_offset_dc=dco,
                   motor_poles=poles, pole_pairs=pp, gear_ratio=gear_milli/1000.0,
                   motor_mech_rpm=mech_milli/1000.0, output_rpm=out_milli/1000.0)
    if len(payload) >= 128:
        ext["rx_queue_drops"] = struct.unpack_from(">I", payload, 124)[0]
    if len(payload) >= 136:
        ext["foc_isr_cycles"], ext["foc_isr_cycles_max"] = struct.unpack_from(">2I", payload, 128)
    if len(payload) >= 156:
        phase_trips, dc_trips = struct.unpack_from(">2I", payload, 136)
        phase_streak, last_source = struct.unpack_from(">2B", payload, 144)
        lp0, lp1, lp2, ldc, lduty = struct.unpack_from(">5h", payload, 146)
        ext.update(phase_trip_count=phase_trips, dc_trip_count=dc_trips,
                   phase_overcurrent_streak=phase_streak, last_trip_source=last_source,
                   last_trip_phase0_a=lp0/50.0, last_trip_phase1_a=lp1/50.0,
                   last_trip_phase2_a=lp2/50.0, last_trip_dc_a=ldc/50.0,
                   last_trip_duty=lduty/1000.0)
    if len(payload) >= 166:
        do0, do1, dodc, dsamp, dvalid, dcal = struct.unpack_from(">3hH2B", payload, 156)
        ext.update(driven_offset0=do0, driven_offset1=do1, driven_offset_dc=dodc,
                   driven_offset_samples=dsamp, driven_offset_valid=bool(dvalid),
                   driven_offset_calibrating=bool(dcal))
    if len(payload) >= 178:
        rla, rlb, dcl, rrb, rrc, dcr = struct.unpack_from(">6H", payload, 166)
        if vid == 1:
            ext.update(raw_adc_phase0=rla, raw_adc_phase1=rlb, raw_adc_dc=dcl)
        else:
            ext.update(raw_adc_phase0=rrb, raw_adc_phase1=rrc, raw_adc_dc=dcr)
    if len(payload) >= 189:
        oo0, oo1, oodc, osamp, osettle, oval = struct.unpack_from(">3hHHB", payload, 178)
        ext.update(off_offset0=oo0, off_offset1=oo1, off_offset_dc=oodc,
                   off_offset_samples=osamp, off_settle_ticks=osettle, off_offset_valid=bool(oval))
    if len(payload) >= 197:
        txdrop, txfail = struct.unpack_from(">2I", payload, 189)
        ext.update(tx_queue_drops=txdrop, tx_start_failures=txfail)
    if len(payload) >= 205:
        rxhi, gapmax = struct.unpack_from(">2I", payload, 197)
        ext.update(rx_queue_highwater=rxhi, process_gap_max_ms=gapmax)
    if len(payload) >= 217:
        isr_count, uart_err, uart_restart = struct.unpack_from(">3I", payload, 205)
        ext.update(isr_count=isr_count, usart3_rx_errors=uart_err, usart3_rx_restarts=uart_restart)
    if len(payload) >= 253:
        no_motion, breakaway = struct.unpack_from(">2I", payload, 217)
        last_motion, pos_error = struct.unpack_from(">2i", payload, 225)
        ps, pc, pr, pv, po = struct.unpack_from(">5I", payload, 233)
        ext.update(position_no_motion_ticks=no_motion, position_breakaway_ticks=breakaway,
                   position_last_motion_count=last_motion, position_error_counts=pos_error,
                   prof_sensor_max_cycles=ps, prof_current_max_cycles=pc,
                   prof_regulator_max_cycles=pr, prof_svpwm_max_cycles=pv,
                   profiler_reentry_guard_total=po)
    if len(payload) >= 254:
        ext["current_offset_valid"] = bool(payload[253])
    return Diag(
        vesc_id=vid, control_mode=mode, state=state, fault=fault, hall=hall,
        override=bool(own), hall_store_ok=bool(store_ok), link_armed=bool(link_armed),
        iq_target_a=vals[0] / 1000.0, iq_ref_a=vals[1] / 1000.0,
        iq_a=vals[2] / 1000.0, id_a=vals[3] / 1000.0,
        erpm=vals[4], duty=vals[5] / 100000.0,
        position=vals[6], position_target=vals[7],
        position_min=vals[8], position_max=vals[9],
        hall_invalid=hall_invalid, current_trips=trips,
        rx_ok=rx_ok, rx_crc_errors=rx_crc, hall_table=table, **ext,
    )

def _i16(data: bytes, i: int, scale: float):
    return struct.unpack_from(">h", data, i)[0] / scale, i + 2


def _i32(data: bytes, i: int, scale: float):
    return struct.unpack_from(">i", data, i)[0] / scale, i + 4


def parse_selective(payload: bytes, expected_mask: int = VALUE_MASK) -> Values:
    if len(payload) < 5 or payload[0] != COMM_GET_VALUES_SELECTIVE:
        raise ValueError("not COMM_GET_VALUES_SELECTIVE")
    mask = struct.unpack_from(">I", payload, 1)[0]
    if mask != expected_mask:
        raise ValueError(f"mask mismatch 0x{mask:08x}")
    i = 5
    v = Values()
    for bit in range(22):
        if not (mask & (1 << bit)):
            continue
        if bit == 0: _, i = _i16(payload, i, 10)
        elif bit == 1: _, i = _i16(payload, i, 10)
        elif bit == 2: v.current_motor, i = _i32(payload, i, 100)
        elif bit == 3: v.current_in, i = _i32(payload, i, 100)
        elif bit == 4: v.id, i = _i32(payload, i, 100)
        elif bit == 5: v.iq, i = _i32(payload, i, 100)
        elif bit == 6: v.duty, i = _i16(payload, i, 1000)
        elif bit == 7: v.rpm, i = _i32(payload, i, 1)
        elif bit == 8: v.vin, i = _i16(payload, i, 10)
        elif bit in (9, 10, 11, 12): _, i = _i32(payload, i, 10000)
        elif bit in (13, 14): _, i = _i32(payload, i, 1)
        elif bit == 15: v.fault, i = payload[i], i + 1
        elif bit == 16: v.position, i = _i32(payload, i, 1000000)
        elif bit == 17: v.vesc_id, i = payload[i], i + 1
        elif bit == 18: i += 6
        elif bit == 19: v.vd, i = _i32(payload, i, 1000)
        elif bit == 20: v.vq, i = _i32(payload, i, 1000)
        elif bit == 21: i += 1
    return v


def _pack_float32_auto(value: float) -> bytes:
    """Encode float32_auto VESC; untuk angka normal layout-nya IEEE-754 big-endian."""
    return struct.pack(">f", float(value))


def _unpack_float32_auto(data: bytes, offset: int) -> tuple[float, int]:
    """Decode satu float32_auto VESC dan kembalikan nilai serta offset berikutnya."""
    if offset + 4 > len(data):
        raise ValueError("short float32_auto")
    return struct.unpack_from(">f", data, offset)[0], offset + 4


class VescDual:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 0.15):
        self.ser = open_transport(port, baud, timeout=0.01)
        # A software reboot can leave an incomplete pre-reset VESC frame in the
        # USB-UART driver's RX queue. Start each new host session at a packet
        # boundary; otherwise the fresh PacketDecoder can prepend stale bytes to
        # the first FW_VERSION reply and report a false timeout.
        self.ser.reset_input_buffer()
        self.timeout = timeout
        self.dec = PacketDecoder()
        # io_lock serializes request/reply readers. tx_lock is intentionally
        # separate so no-reply watchdog/setpoint frames can still be transmitted
        # while a slow telemetry reply is pending. This is safe because only one
        # thread reads/decodes replies, while every physical write is serialized.
        self.io_lock = threading.Lock()
        self.tx_lock = threading.Lock()

    def close(self):
        self.ser.close()

    def send(self, payload: bytes):
        packet = frame(payload)
        lock = getattr(self, "tx_lock", None)
        if lock is None:
            self.tx_lock = threading.Lock()
            lock = self.tx_lock
        with lock:
            written = self.ser.write(packet)
            if written != len(packet):
                raise IOError(f"short serial write {written}/{len(packet)}")
            # The reply timeout starts only after the full VESC frame has reached
            # the OS/USB-UART transmit path. This matters especially immediately
            # after COMM_REBOOT when a fresh tty session is opened.
            self.ser.flush()

    def send_no_reply(self, payload: bytes, right: bool = False) -> None:
        """Send one command that is defined to produce no reply.

        Unlike transact(), this does not acquire io_lock, so motor watchdog and
        setpoint refresh traffic is not starved by a slow GET_VALUES response.
        tx_lock inside send() still guarantees frame writes never interleave.
        """
        self.send(self.fwd(payload) if right else payload)

    def recv(self, expected_cmd: int, timeout: float | None = None) -> bytes:
        end = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < end:
            chunk = self.ser.read(256)
            if not chunk:
                continue
            for p in self.dec.feed(chunk):
                if p and p[0] == expected_cmd:
                    return p
        raise TimeoutError(f"no reply for COMM {expected_cmd}")

    def transact(self, payload: bytes, expected_cmd: int, timeout: float | None = None) -> bytes:
        with self.io_lock:
            self.send(payload)
            return self.recv(expected_cmd, timeout)

    @staticmethod
    def fwd(payload: bytes) -> bytes:
        return bytes((COMM_FORWARD_CAN, RIGHT_ID)) + payload

    def ping_can(self):
        p = self.transact(bytes((COMM_PING_CAN,)), COMM_PING_CAN, 0.5)
        return list(p[1:])

    def fw(self, right=False) -> bytes:
        req = bytes((COMM_FW_VERSION,))
        return self.transact(self.fwd(req) if right else req, COMM_FW_VERSION, 0.5)

    def alive(self, right: bool = False) -> None:
        """Commands::sendAlive: refresh timeout without changing the setpoint."""
        req=bytes((COMM_ALIVE,))
        self.send_no_reply(req, right)

    def set_detect(self, mode: int, right: bool = False) -> None:
        """Commands::setDetect: select VESC Tool rotor-position display mode."""
        if not 0 <= int(mode) <= 7:
            raise ValueError("display position mode harus 0..7")
        req=bytes((COMM_SET_DETECT,int(mode)))
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            self.ser.flush()

    def recv_rotor_position(self, timeout: float = 0.25) -> float:
        p=self.recv(COMM_ROTOR_POSITION,timeout)
        if len(p)!=5:
            raise ValueError(f"unexpected rotor-position packet length {len(p)}")
        return struct.unpack_from(">i",p,1)[0]/100000.0

    def set_current(self, left: float, right: float):
        l = bytes((COMM_SET_CURRENT,)) + struct.pack(">i", round(left * 1000))
        r = bytes((COMM_SET_CURRENT,)) + struct.pack(">i", round(right * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def set_current_rel(self, left: float, right: float):
        """Commands::setCurrentRel VESC Tool: kirim signed value x1e5 apa adanya."""
        l = bytes((COMM_SET_CURRENT_REL,)) + struct.pack(">i", round(left * 100000.0))
        r = bytes((COMM_SET_CURRENT_REL,)) + struct.pack(">i", round(right * 100000.0))
        with self.io_lock:
            self.send(l)
            self.send(self.fwd(r))

    def get_mcconf_raw(self, right: bool = False, default: bool = False) -> bytes:
        """Baca payload MC Config mentah seperti tombol Read/Read Default VESC Tool."""
        cmd = COMM_GET_MCCONF_DEFAULT if default else COMM_GET_MCCONF
        req = bytes((cmd,))
        p = self.transact(self.fwd(req) if right else req, cmd, 1.2)
        if len(p) < 8:
            raise ValueError("short MC Config reply")
        return p[1:]

    def set_mcconf_raw(self, raw: bytes, right: bool = False) -> None:
        """Tulis kembali payload MC Config 6.00 persis seperti tombol Write VESC Tool."""
        req = bytes((COMM_SET_MCCONF,)) + bytes(raw)
        p = self.transact(self.fwd(req) if right else req, COMM_SET_MCCONF, PERSISTENT_WRITE_TIMEOUT)
        if p != bytes((COMM_SET_MCCONF,)):
            raise ValueError("invalid MC Config ACK")

    def get_appconf_raw(self, right: bool = False, default: bool = False) -> bytes:
        """Baca payload App Config mentah tanpa mengubah schema yang dibuat VESC 6.00."""
        cmd = COMM_GET_APPCONF_DEFAULT if default else COMM_GET_APPCONF
        req = bytes((cmd,))
        p = self.transact(self.fwd(req) if right else req, cmd, 1.2)
        if len(p) < 8:
            raise ValueError("short App Config reply")
        return p[1:]

    def set_appconf_raw(self, raw: bytes, right: bool = False, store: bool = True) -> None:
        """Tulis App Config dengan semantik SET_APPCONF atau SET_APPCONF_NO_STORE."""
        cmd = COMM_SET_APPCONF if store else COMM_SET_APPCONF_NO_STORE
        req = bytes((cmd,)) + bytes(raw)
        p = self.transact(self.fwd(req) if right else req, cmd,
                          PERSISTENT_WRITE_TIMEOUT if store else 1.2)
        if p != bytes((cmd,)):
            raise ValueError("invalid App Config ACK")

    def decoded_adc(self) -> tuple[float, float, float, float]:
        """Baca ADC1/ADC2 seperti tab Realtime ADC VESC Tool."""
        p = self.transact(bytes((COMM_GET_DECODED_ADC,)), COMM_GET_DECODED_ADC, 0.5)
        if len(p) != 17:
            raise ValueError(f"unexpected decoded ADC reply length {len(p)}")
        return tuple(struct.unpack_from(">i", p, 1 + 4 * i)[0] / 1_000_000.0 for i in range(4))

    def setup_values(self, right: bool = False) -> SetupValues:
        """Baca COMM_GET_VALUES_SETUP dan parse dengan urutan Commands::getValuesSetup VESC 6.00."""
        req = bytes((COMM_GET_VALUES_SETUP,))
        p = self.transact(self.fwd(req) if right else req, COMM_GET_VALUES_SETUP, 0.5)
        i = 1
        def i16(scale: float) -> float:
            nonlocal i
            v = struct.unpack_from(">h", p, i)[0] / scale; i += 2; return v
        def i32(scale: float) -> float:
            nonlocal i
            v = struct.unpack_from(">i", p, i)[0] / scale; i += 4; return v
        vals = [i16(10), i16(10), i32(100), i32(100), i16(1000), i32(1),
                i32(1000), i16(10), i16(1000), i32(10000), i32(10000),
                i32(10000), i32(10000), i32(1000), i32(1000), i32(1_000_000)]
        fault = p[i]; vesc_id = p[i + 1]; num_vescs = p[i + 2]; i += 3
        wh_left = i32(1000)
        odometer = struct.unpack_from(">I", p, i)[0]; i += 4
        uptime = struct.unpack_from(">I", p, i)[0]; i += 4
        if i != len(p):
            raise ValueError(f"Setup Values parser consumed {i}, packet has {len(p)}")
        return SetupValues(*vals, fault, vesc_id, num_vescs, wh_left, odometer, uptime)

    def terminal(self, command: str, right: bool = False, sync: bool = True) -> str:
        """Kirim command Terminal VESC Tool dan tunggu COMM_PRINT framed reply."""
        cmd = COMM_TERMINAL_CMD_SYNC if sync else COMM_TERMINAL_CMD
        req = bytes((cmd,)) + command.encode("utf-8")
        p = self.transact(self.fwd(req) if right else req, COMM_PRINT, 0.8)
        return p[1:].decode("utf-8", errors="replace")

    def set_odometer(self, meters: int, right: bool = False) -> None:
        """Kirim tombol Set Odometer VESC Tool; command standar ini tidak memiliki ACK."""
        if not 0 <= int(meters) <= 0xFFFFFFFF:
            raise ValueError("odometer harus 0..4294967295 meter")
        req = bytes((COMM_SET_ODOMETER,)) + struct.pack(">I", int(meters))
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            self.ser.flush()

    def shutdown_safe(self, right: bool = False) -> None:
        """Kirim COMM_SHUTDOWN; pada board ini firmware memetakan ke safe bridge release."""
        req = bytes((COMM_SHUTDOWN, 0, 0))
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            self.ser.flush()

    def disable_app_output(self, time_ms: int, forward_can: bool = False,
                           right: bool = False) -> None:
        """Commands::disableAppOutput: [fwd_can][signed time_ms]."""
        req=bytes((COMM_APP_DISABLE_OUTPUT,1 if forward_can else 0))+struct.pack(">i",int(time_ms))
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            self.ser.flush()

    def measure_r_l(self, right: bool = False) -> dict[str, float]:
        """Non-persistent VESC R/L measurement. Firmware restores MC config after reply."""
        req=bytes((COMM_DETECT_MOTOR_R_L,))
        p=self.transact(self.fwd(req) if right else req,COMM_DETECT_MOTOR_R_L,12.0)
        if len(p)!=13: raise RuntimeError(f"measure_r_l unexpected length {len(p)}")
        r_raw,l_raw,ld_raw=struct.unpack_from(">iii",p,1)
        r=r_raw/1_000_000.0; l=l_raw/1_000_000_000.0; ld_lq=ld_raw/1_000_000_000.0
        if not (r>0.0 and l>0.0): raise RuntimeError(f"measure_r_l invalid R/L r={r} l={l}")
        return {"r_ohm":r,"l_h":l,"ld_lq_h":ld_lq}

    def measure_flux_openloop(self, current_a: float, erpm_per_sec: float, duty: float,
                              resistance_ohm: float, inductance_h: float,
                              right: bool = False) -> float:
        """Non-persistent VESC open-loop flux measurement using an already qualified R/L model."""
        if not (0.0<abs(current_a)<=15.0 and 50.0<=abs(erpm_per_sec)<=20000.0 and
                0.0<abs(duty)<=1.0 and 0.0<resistance_ohm<=2.0 and 0.0<inductance_h<=0.1):
            raise ValueError("invalid flux measurement arguments")
        req=bytearray((COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,))
        req+=struct.pack(">iiiii",round(current_a*1e3),round(erpm_per_sec*1e3),round(duty*1e3),
                         round(resistance_ohm*1e6),round(inductance_h*1e8))
        p=self.transact(self.fwd(bytes(req)) if right else bytes(req),COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,15.0)
        if len(p)!=14: raise RuntimeError(f"measure_flux_openloop unexpected length {len(p)}")
        flux=struct.unpack_from(">i",p,1)[0]/1e7
        if not (0.0001<=flux<=1.0): raise RuntimeError(f"measure_flux_openloop invalid flux={flux}")
        return flux

    def detect_encoder(self, current_a: float = 1.0, right: bool = False):
        """Commands::measureEncoder packet/reply format from VESC Tool."""
        req=bytes((COMM_DETECT_ENCODER,))+struct.pack(">i",round(current_a*1000.0))
        p=self.transact(self.fwd(req) if right else req,COMM_DETECT_ENCODER,90.0)
        if len(p)!=10:
            raise ValueError(f"unexpected encoder detect reply length {len(p)}")
        off=struct.unpack_from(">i",p,1)[0]/1_000_000.0
        ratio=struct.unpack_from(">i",p,5)[0]/1_000_000.0
        return off,ratio,bool(p[9])

    def detect_all_foc(self, max_power_loss: float = 50.0, min_current_in: float = -8.0,
                       max_current_in: float = 8.0, openloop_rpm: float = 250.0,
                       sl_erpm: float = 2500.0, detect_can: bool = True) -> int:
        """Jalankan tombol Detect All FOC VESC Tool 6.00 pada kedua motor on-board."""
        out = bytearray((COMM_DETECT_APPLY_ALL_FOC, 1 if detect_can else 0))
        for value in (max_power_loss, min_current_in, max_current_in, openloop_rpm, sl_erpm):
            out += struct.pack(">i", round(value * 1000.0))
        # Utility::detectAllFoc VESC Tool: gate application output during the
        # complete detection and always restore it afterwards, even on timeout.
        self.disable_app_output(180000, forward_can=True)
        try:
            p = self.transact(bytes(out), COMM_DETECT_APPLY_ALL_FOC, 180.0)
            if len(p) != 3:
                raise ValueError(f"unexpected Detect All reply length {len(p)}")
            return struct.unpack_from(">h", p, 1)[0]
        finally:
            self.disable_app_output(0, forward_can=True)

    def get_battery_cut(self, right: bool = False) -> tuple[float, float]:
        """Baca l_battery_cut_start/end dengan format persis VESC 6.00."""
        req = bytes((COMM_GET_BATTERY_CUT,))
        p = self.transact(self.fwd(req) if right else req, COMM_GET_BATTERY_CUT)
        if len(p) != 9:
            raise ValueError(f"unexpected battery-cut reply length {len(p)}")
        return struct.unpack_from(">i", p, 1)[0] / 1000.0, struct.unpack_from(">i", p, 5)[0] / 1000.0

    def set_battery_cut(self, start_v: float, end_v: float, *, store: bool = False,
                        forward: bool = False, right: bool = False) -> None:
        """Terapkan Commands::setBatteryCut; ACK wajib diterima sebelum fungsi selesai."""
        if not (0.0 <= end_v < start_v <= 80.0):
            raise ValueError("battery cut harus 0 <= end < start <= 80 V")
        req = (bytes((COMM_SET_BATTERY_CUT,)) +
               struct.pack(">iiBB", round(start_v * 1000.0), round(end_v * 1000.0),
                           1 if store else 0, 1 if forward else 0))
        p = self.transact(self.fwd(req) if right else req, COMM_SET_BATTERY_CUT,
                          PERSISTENT_WRITE_TIMEOUT if store else self.timeout)
        if p != bytes((COMM_SET_BATTERY_CUT,)):
            raise ValueError("invalid battery-cut ACK")

    def get_mcconf_temp(self, right: bool = False) -> McconfTemp:
        """Baca limit runtime halaman Setup VESC Tool tanpa mengubah konfigurasi."""
        req = bytes((COMM_GET_MCCONF_TEMP,))
        p = self.transact(self.fwd(req) if right else req, COMM_GET_MCCONF_TEMP)
        if len(p) < 50:
            raise ValueError(f"short MC temp reply: {len(p)}")
        i = 1
        vals = []
        for _ in range(10):
            value, i = _unpack_float32_auto(p, i)
            vals.append(value)
        poles = p[i]; i += 1
        gear, i = _unpack_float32_auto(p, i)
        wheel, i = _unpack_float32_auto(p, i)
        if i != len(p):
            raise ValueError(f"MC temp parser consumed {i}, packet has {len(p)}")
        return McconfTemp(*vals, motor_poles=poles, gear_ratio=gear, wheel_diameter=wheel)

    def set_mcconf_temp(self, conf: McconfTemp, *, store: bool = False,
                        forward: bool = False, ack: bool = True,
                        divide_by_controllers: bool = False, setup_units: bool = False,
                        right: bool = False) -> None:
        """Kirim Commands::setMcconfTemp/Setup persis urutan field VESC Tool 6.00."""
        cmd = COMM_SET_MCCONF_TEMP_SETUP if setup_units else COMM_SET_MCCONF_TEMP
        out = bytearray((cmd, 1 if store else 0, 1 if forward else 0,
                         1 if ack else 0, 1 if divide_by_controllers else 0))
        # commands.cpp VESC Tool mengirim tepat delapan float32_auto. Firmware
        # VESC dapat menerima dua battery-current limit tambahan secara backward-
        # compatible, tetapi GUI VESC Tool standar tidak mengirim field opsional itu.
        for value in (conf.current_min_scale, conf.current_max_scale, conf.min_erpm, conf.max_erpm,
                      conf.min_duty, conf.max_duty, conf.watt_min, conf.watt_max):
            out += _pack_float32_auto(value)
        req = bytes(out)
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            if ack:
                p = self.recv(cmd, PERSISTENT_WRITE_TIMEOUT if store else self.timeout)
                if p != bytes((cmd,)):
                    raise ValueError("invalid MC temp ACK")

    def appconf_roundtrip_no_store(self, right: bool = False) -> None:
        """Uji tombol App Config no-store tanpa perlu memahami schema: GET lalu echo payload identik sebagai NO_STORE."""
        get_req = bytes((COMM_GET_APPCONF,))
        current = self.transact(self.fwd(get_req) if right else get_req, COMM_GET_APPCONF, 0.8)
        if len(current) < 8:
            raise ValueError("short App Config reply")
        req = bytes((COMM_SET_APPCONF_NO_STORE,)) + current[1:]
        p = self.transact(self.fwd(req) if right else req, COMM_SET_APPCONF_NO_STORE, 0.8)
        if p != bytes((COMM_SET_APPCONF_NO_STORE,)):
            raise ValueError("invalid App Config no-store ACK")

    def reboot(self, right: bool = False) -> None:
        """Kirim COMM_REBOOT. Tidak menunggu ACK karena VESC 6.00 langsung reset."""
        req = bytes((COMM_REBOOT,))
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            self.ser.flush()

    def set_rpm(self, left: int, right: int):
        l = bytes((COMM_SET_RPM,)) + struct.pack(">i", int(left))
        r = bytes((COMM_SET_RPM,)) + struct.pack(">i", int(right))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def set_rpm_one(self, erpm: int, right: bool = False) -> None:
        """Command exactly one virtual VESC; do not energize the other endpoint with RPM=0."""
        req = bytes((COMM_SET_RPM,)) + struct.pack(">i", int(erpm))
        self.send_no_reply(req, right)

    def set_duty(self, left: float, right: float):
        l = bytes((COMM_SET_DUTY,)) + struct.pack(">i", round(left * 100000))
        r = bytes((COMM_SET_DUTY,)) + struct.pack(">i", round(right * 100000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def set_pos(self, left_deg: float, right_deg: float):
        """Commands::setPos VESC Tool: signed degree value x1e6, tanpa client clamp."""
        def enc(deg: float) -> bytes:
            return bytes((COMM_SET_POS,)) + struct.pack(">i", round(deg * 1000000.0))
        with self.io_lock:
            self.send(enc(left_deg)); self.send(self.fwd(enc(right_deg)))

    def set_pos_one(self, deg: float, right: bool = False) -> None:
        """Command exactly one virtual VESC; useful for independent hardware validation."""
        req = bytes((COMM_SET_POS,)) + struct.pack(">i", round(deg * 1000000.0))
        self.send_no_reply(req, right)

    def brake(self, left_a: float, right_a: float):
        l = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(left_a * 1000))
        r = bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(right_a * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def handbrake(self, left_a: float, right_a: float):
        l = bytes((COMM_SET_HANDBRAKE,)) + struct.pack(">i", round(left_a * 1000))
        r = bytes((COMM_SET_HANDBRAKE,)) + struct.pack(">i", round(right_a * 1000))
        with self.io_lock:
            self.send(l); self.send(self.fwd(r))

    def values(self, right=False) -> Values:
        req = bytes((COMM_GET_VALUES_SELECTIVE,)) + struct.pack(">I", VALUE_MASK)
        expected_id = RIGHT_ID if right else 1
        deadline = time.monotonic() + self.timeout
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no selective values reply from VESC ID {expected_id}")
                p = self.recv(COMM_GET_VALUES_SELECTIVE, remaining)
                v = parse_selective(p)
                if v.vesc_id == expected_id:
                    return v

    def detect_hall(self, current_a: float = 1.0, right: bool = False):
        if current_a <= 0.0:
            raise ValueError("Hall detect current must be > 0 A")
        req = bytes((COMM_DETECT_HALL_FOC,)) + struct.pack(">i", round(current_a * 1000.0))
        # Bare-metal F103 runs the VESC-compatible 1-degree, 3F+3R Hall sweep
        # cooperatively while the 16-kHz FOC ISR stays live. Under load this can
        # legitimately exceed 20 s; VESC Tool waits for the asynchronous reply.
        p = self.transact(self.fwd(req) if right else req, COMM_DETECT_HALL_FOC, 90.0)
        if len(p) != 10:
            raise ValueError(f"unexpected Hall detect reply length {len(p)}")
        table = list(p[1:9])
        ok = p[9] == 0
        return ok, table

    @staticmethod
    def _custom(op: int, data: bytes = b"") -> bytes:
        return bytes((COMM_CUSTOM_APP_DATA,)) + HB_MAGIC + bytes((HB_VERSION, op)) + data

    def custom_transact(self, op: int, data: bytes = b"", right: bool = False,
                        timeout: float | None = None) -> bytes:
        req = self._custom(op, data)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        with self.io_lock:
            self.send(self.fwd(req) if right else req)
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no custom reply for op {op}")
                p = self.recv(COMM_CUSTOM_APP_DATA, remaining)
                # COMM_CUSTOM_APP_DATA multiplexes all Hoverboard extension ops.
                # Ignore a delayed reply from another op instead of letting it
                # poison tuning/diagnostic transactions.
                if len(p) >= 6 and p[1:3] == HB_MAGIC and p[3] == HB_VERSION and p[4] == op:
                    return p

    def get_tuning(self, right: bool = False) -> Tuning:
        p=self.custom_transact(HB_GET_TUNING, right=right)
        status=parse_custom_header(p,HB_GET_TUNING)
        if status or len(p)<30: raise RuntimeError(f"get_tuning status={status} len={len(p)}")
        vals=struct.unpack_from(">10H",p,6)
        filt,ilim=struct.unpack_from(">Hh",p,26)
        return Tuning(*vals,telem_filter_q16=filt,current_limit_q4=ilim)

    def set_tuning(self, tune: Tuning, right: bool = False, store: bool = False) -> Tuning:
        self.require_platform_compatible(require_build=True)
        vals=(tune.kpq,tune.kiq,tune.kpd,tune.kid,tune.kps,tune.kis,tune.kds,tune.kpp,tune.kip,tune.kdp)
        data=struct.pack(">10HHB",*vals,max(1,min(65535,int(tune.telem_filter_q16))),1 if store else 0)
        p=self.custom_transact(HB_SET_TUNING,data,right=right)
        status=parse_custom_header(p,HB_SET_TUNING)
        if status or len(p)<30: raise RuntimeError(f"set_tuning status={status} len={len(p)}")
        vals2=struct.unpack_from(">10H",p,6); filt,ilim=struct.unpack_from(">Hh",p,26)
        return Tuning(*vals2,telem_filter_q16=filt,current_limit_q4=ilim)

    def set_id_test(self, current_a: float, phase_deg: float = 0.0, right: bool = False):
        self.require_platform_compatible(require_build=True)
        data=struct.pack(">ii",round(current_a*1000.0),round(phase_deg*1000.0))
        p=self.custom_transact(HB_SET_ID_TEST,data,right=right)
        status=parse_custom_header(p,HB_SET_ID_TEST)
        if status: raise RuntimeError(f"set_id_test status={status}")

    def position_state(self, right: bool = False) -> PositionState:
        return parse_position_state(self.custom_transact(HB_GET_POS_STATE, right=right), HB_GET_POS_STATE)

    def set_position_limits(self, minimum: int, maximum: int, right: bool = False) -> PositionState:
        if not (-2147483648 <= minimum <= 2147483647 and -2147483648 <= maximum <= 2147483647):
            raise ValueError("position limits must fit signed int32")
        if minimum > maximum:
            raise ValueError("minimum must be <= maximum")
        p = self.custom_transact(HB_SET_POS_LIMITS, struct.pack(">ii", minimum, maximum), right=right)
        return parse_position_state(p, HB_SET_POS_LIMITS)

    def set_position_counts(self, target: int, right: bool = False) -> PositionState:
        if not -2147483648 <= target <= 2147483647:
            raise ValueError("position target must fit signed int32")
        p = self.custom_transact(HB_SET_POS_TARGET, struct.pack(">i", target), right=right)
        return parse_position_state(p, HB_SET_POS_TARGET)

    def reset_position(self, right: bool = False) -> PositionState:
        return parse_position_state(self.custom_transact(HB_RESET_POSITION, right=right), HB_RESET_POSITION)

    def home_steering(self):
        """Bounded LEFT ABI startup alignment + persisted-span homing."""
        p=self.custom_transact(HB_STEERING_HOME,right=False,timeout=20.0)
        status=parse_custom_header(p,HB_STEERING_HOME)
        if len(p)<11:
            raise ValueError(f"short steering home reply: {len(p)}")
        flags=p[6]
        span=struct.unpack_from(">i",p,7)[0]
        return {"status":status,"calibrated":bool(flags&1),"homed":bool(flags&2),
                "encoder_synced":bool(flags&4),"span":span}

    def encoder_debug(self):
        """Read-only LEFT ABI alignment/detect black-box diagnostics."""
        p=self.custom_transact(HB_ENCODER_DEBUG,right=False,timeout=2.0)
        status=parse_custom_header(p,HB_ENCODER_DEBUG)
        if status or len(p)<80:
            raise ValueError(f"encoder_debug status={status} len={len(p)}")
        q=6
        align_stage,steer_stage,detect_stage,inverted,configured,synced=p[q:q+6]; q+=6
        offset_mdeg,ratio_milli=struct.unpack_from(">ii",p,q); q+=8
        raw,before,jog,back=struct.unpack_from(">IIII",p,q); q+=16
        dj,db,plus_mdeg,minus_mdeg=struct.unpack_from(">iiii",p,q); q+=16
        edge_a,edge_b,edge_pb5,samples=struct.unpack_from(">IIII",p,q); q+=16
        current_ma=struct.unpack_from(">H",p,q)[0]; q+=2
        span,pos,target=struct.unpack_from(">iii",p,q); q+=12
        pid_target=struct.unpack_from(">i",p,q)[0] if len(p)>=q+4 else target
        if len(p)>=q+4: q+=4
        return {"align_stage":align_stage,"steering_stage":steer_stage,"detect_stage":detect_stage,
                "inverted":bool(inverted),"configured":bool(configured),"synced":bool(synced),
                "offset_deg":offset_mdeg/1000.0,"ratio":ratio_milli/1000.0,"raw":raw,
                "before":before,"jog":jog,"back":back,"dj":dj,"db":db,
                "plus_deg":plus_mdeg/1000.0,"minus_deg":minus_mdeg/1000.0,
                "edge_a":edge_a,"edge_b":edge_b,"edge_pb5":edge_pb5,"samples":samples,"current_ma":current_ma,
                "span":span,"position":pos,"target":target,"pid_target":pid_target}

    def platform_health(self) -> dict[str, int | bool]:
        p=self.custom_transact(HB_GET_PLATFORM_HEALTH,right=False,timeout=max(self.timeout,1.2))
        status=parse_custom_header(p,HB_GET_PLATFORM_HEALTH)
        if status or len(p)!=46:
            raise RuntimeError(f"platform_health status={status} len={len(p)}")
        enabled,healthy,boot_iwdg,_=p[6:10]
        vals=struct.unpack_from(">9I",p,10)
        names=("boot_reset_csr","boot_reset_reason","boot_reset_stage","feed_count","reject_count",
               "adc_heartbeat","left_heartbeat","right_heartbeat","last_feed_ms")
        out=dict(zip(names,vals)); out.update(enabled=bool(enabled),healthy=bool(healthy),boot_iwdg=bool(boot_iwdg))
        return out

    def comms_health(self) -> dict[str, int]:
        p=self.custom_transact(HB_GET_COMMS_HEALTH,right=False,timeout=max(self.timeout,1.2))
        status=parse_custom_header(p,HB_GET_COMMS_HEALTH)
        names=("rx_ok","rx_crc_err","rx_timeout_reset","rx_queue_drop","rx_queue_highwater","rt_cmd_coalesced",
               "tx_queue_drop","tx_start_fail","tx_queue_highwater","process_gap_max_ms","pending_count","tx_count",
               "tx_active","rx_active","uart_rx_error","uart_rx_restart","uart_forced_recovery",
               "main_vesc_max_cycles","main_house_max_cycles","main_tail_max_cycles")
        if status or len(p)!=6+4*len(names):
            raise RuntimeError(f"comms_health status={status} len={len(p)}")
        vals=struct.unpack_from(">"+"I"*len(names),p,6)
        return dict(zip(names,vals))

    def platform_info(self) -> dict[str, int | str]:
        p=self.custom_transact(HB_GET_PLATFORM_INFO,right=False,timeout=max(self.timeout,1.2))
        status=parse_custom_header(p,HB_GET_PLATFORM_INFO)
        if status or len(p)!=46: raise RuntimeError(f"platform_info status={status} len={len(p)}")
        platform_schema,diag_schema,isr_schema,trace_schema=struct.unpack_from(">4H",p,6)
        features,build_id,git_hi=struct.unpack_from(">3I",p,14)
        git_lo,control_div=struct.unpack_from(">2H",p,26)
        cpu_hz,pwm_hz,profile_revision=struct.unpack_from(">3I",p,30)
        trace_capacity,slot_capacity=struct.unpack_from(">2H",p,42)
        return dict(platform_schema=platform_schema,diag_schema=diag_schema,isr_schema=isr_schema,trace_schema=trace_schema,
                    features=features,build_id=build_id,git_sha=f"{git_hi:08x}{git_lo:04x}",control_div=control_div,
                    cpu_hz=cpu_hz,pwm_hz=pwm_hz,profile_revision=profile_revision,trace_capacity=trace_capacity,slot_capacity=slot_capacity)

    def require_platform_compatible(self, require_build: bool = True) -> dict[str, int | str]:
        x=self.platform_info()
        expected=(HB_PLATFORM_SCHEMA,HB_DIAG_SCHEMA,HB_ISR_SCHEMA,HB_TRACE_SCHEMA,HB_ISR_PROFILE_REVISION)
        got=(x['platform_schema'],x['diag_schema'],x['isr_schema'],x['trace_schema'],x['profile_revision'])
        if got!=expected: raise RuntimeError(f"TUNING_REFUSED schema mismatch got={got} expected={expected}")
        if x['cpu_hz']!=64_000_000 or x['pwm_hz']!=16_000 or not (1<=x['control_div']<=6):
            raise RuntimeError(f"TUNING_REFUSED platform timing mismatch {x}")
        if require_build:
            try:
                from build_identity import compute
                local=compute(Path(__file__).resolve().parents[1])
                if x['build_id']!=local['build_id32']:
                    raise RuntimeError(f"TUNING_REFUSED build mismatch fw=0x{x['build_id']:08x} local=0x{local['build_id32']:08x}")
            except ImportError:
                raise RuntimeError('TUNING_REFUSED local build identity unavailable')
        return x

    def isr_profile(self, reset: bool = False) -> dict[str, int]:
        names=(
            "total_max","deadline_miss","pre_max","control_max","post_max",
            "pre_gate","pre_offset","pre_protect","left_step","right_step",
            "left_control","right_control","left_hold","right_hold",
            "sensor","pll","current","regulator","position_pid","speed_pid",
            "current_circle","id_pi","iq_pi","decouple_limit","svpwm","duty_mag","reentry_guard",
            "slot0_max","slot1_max","slot2_max","slot3_max","slot4_max","slot5_max",
            "slot0_miss","slot1_miss","slot2_miss","slot3_miss","slot4_miss","slot5_miss",
            "slot0_count","slot1_count","slot2_count","slot3_count","slot4_count","slot5_count",
            "detail_sample_count",
            "detail_slot0_count","detail_slot1_count","detail_slot2_count","detail_slot3_count","detail_slot4_count","detail_slot5_count",
            "steady_isr_count","slot_sequence_errors","fast_hold_svpwm","profile_revision","active_slot_count","reset_epoch",
            "outer_max","outer_miss","outer_jitter","outer_period_min","outer_period_max",
            "adc_heartbeat","left_heartbeat","right_heartbeat",
            "snapshot_dwt","irq_entry","irq_exit","left_step_count","right_step_count",
            "dma_tc_pending_exit")
        p=self.custom_transact(HB_GET_ISR_PROFILE,bytes((1 if reset else 0,)),right=False,timeout=max(self.timeout,1.2))
        status=parse_custom_header(p,HB_GET_ISR_PROFILE)
        if status or len(p)!=6+4*len(names):
            raise RuntimeError(f"isr_profile status={status} len={len(p)}")
        vals=struct.unpack_from(">"+"I"*len(names),p,6)
        return dict(zip(names,vals))

    def trace_meta(self) -> dict[str, int]:
        p=self.custom_transact(HB_GET_TRACE_META,right=False,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_GET_TRACE_META)
        if status or len(p)!=18: raise RuntimeError(f"trace_meta status={status} len={len(p)}")
        write_count=struct.unpack_from(">I",p,6)[0]
        frozen,trigger_motor,trigger_fault,count,head,capacity=struct.unpack_from(">6B",p,10)
        sample_size=struct.unpack_from(">H",p,16)[0]
        return {"write_count":write_count,"frozen":frozen,"trigger_motor":trigger_motor,"trigger_fault":trigger_fault,"count":count,"head":head,"capacity":capacity,"sample_size":sample_size}

    def trace_sample(self, index: int) -> dict[str, int]:
        if index < 0 or index > 255: raise ValueError("trace index out of range")
        p=self.custom_transact(HB_GET_TRACE_SAMPLE,bytes((index,)),right=False,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_GET_TRACE_SAMPLE)
        if status or len(p)!=68: raise RuntimeError(f"trace_sample status={status} len={len(p)}")
        vals=struct.unpack_from(">IHBB14h4i3H4B",p,6)
        names=("pwm_tick","isr_cycles","control_slot","event_bits",
               "left_id_q4","left_iq_q4","left_id_set_q4","left_iq_set_q4","left_vd","left_vq","left_erpm",
               "right_id_q4","right_iq_q4","right_id_set_q4","right_iq_set_q4","right_vd","right_vq","right_erpm",
               "left_id_integrator","left_iq_integrator","right_id_integrator","right_iq_integrator",
               "left_sample_window","right_sample_window","vin_adc","left_fault","right_fault","left_quality","right_quality")
        return dict(zip(names,vals))

    def download_trace(self) -> list[dict[str, int]]:
        """Download one frozen chronological ISR trace snapshot."""
        meta=self.trace_meta()
        if not meta["frozen"]:
            raise RuntimeError(f"TRACE_NOT_FROZEN meta={meta}")
        rows=[]
        for i in range(meta["count"]):
            rows.append(self.trace_sample(i))
        return rows

    def adc_validity(self, right: bool = False) -> dict[str, int]:
        p=self.custom_transact(HB_GET_ADC_VALIDITY,right=right,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_GET_ADC_VALIDITY)
        if status or len(p)!=29: raise RuntimeError(f"adc_validity status={status} len={len(p)}")
        vals=struct.unpack_from(">7HI5B",p,6)
        names=("ccr_a","ccr_b","ccr_c","zero_window","min_window","guard","adc_phase","invalid_count",
               "sector","window_valid","offset_valid","driven_offset_valid","bridge_settled")
        return dict(zip(names,vals))

    def arm_current_step(self, pre_a: float, step_a: float, pre_samples: int = 8, post_samples: int = 24, right: bool = False) -> int:
        self.require_platform_compatible(require_build=True)
        if not (2<=pre_samples<40 and 2<=post_samples<40 and pre_samples+post_samples<=40):
            raise ValueError("step samples must each be 2..39 and total <= trace capacity 40")
        data=struct.pack(">iiBB",round(pre_a*1000.0),round(step_a*1000.0),pre_samples,post_samples)
        p=self.custom_transact(HB_ARM_CURRENT_STEP,data,right=right,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_ARM_CURRENT_STEP)
        if status or len(p)!=10: raise RuntimeError(f"arm_current_step status={status} len={len(p)}")
        return struct.unpack_from(">I",p,6)[0]

    def step_status(self, right: bool = False) -> dict[str, int]:
        p=self.custom_transact(HB_GET_STEP_STATUS,right=right,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_GET_STEP_STATUS)
        if status or len(p)!=20: raise RuntimeError(f"step_status status={status} len={len(p)}")
        seq,pre_q4,step_q4,active,second,pre_n,post_n,fired,done=struct.unpack_from(">Ihh6B",p,6)
        return dict(sequence=seq,pre_q4=pre_q4,step_q4=step_q4,active=active,second=second,pre_remaining=pre_n,post_remaining=post_n,step_fired=fired,done=done)

    def position_d_state(self, right: bool = False) -> dict[str, int]:
        p=self.custom_transact(HB_GET_POSITION_D_STATE,right=right,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_GET_POSITION_D_STATE)
        if status or len(p)!=22: raise RuntimeError(f"position_d_state status={status} len={len(p)}")
        pos,target,dproc=struct.unpack_from(">iii",p,6)
        mode,phase_mode,enc_inv,motor_inv=struct.unpack_from(">4B",p,18)
        return dict(position=pos,target=target,dproc_q15=dproc,control_mode=mode,phase_mode=phase_mode,encoder_inverted=enc_inv,motor_inverted=motor_inv)

    def boot_handoff(self) -> None:
        p=self.custom_transact(HB_BOOT_HANDOFF,right=False,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_BOOT_HANDOFF)
        if status: raise RuntimeError(f"boot_handoff status={status}")

    def trace_all(self) -> list[dict[str, int]]:
        meta=self.trace_meta()
        return [self.trace_sample(i) for i in range(meta["count"])]

    def run_current_step(self, pre_a: float, step_a: float, pre_samples: int = 8,
                         post_samples: int = 24, right: bool = False,
                         timeout: float = 1.0) -> list[dict[str, int]]:
        """Run one firmware-synchronized step and return the frozen raw trace."""
        seq=self.arm_current_step(pre_a,step_a,pre_samples,post_samples,right)
        deadline=time.monotonic()+timeout
        st=None
        while time.monotonic()<deadline:
            st=self.step_status(right)
            if st["sequence"]!=seq: raise RuntimeError(f"step sequence changed {st['sequence']} != {seq}")
            if st["done"] and not st["active"]: break
            time.sleep(0.002)
        else: raise TimeoutError(f"current step seq={seq} did not complete: {st}")
        meta=self.trace_meta()
        if not meta["frozen"]: raise RuntimeError(f"step trace not frozen: {meta}")
        rows=self.trace_all()
        if not rows: raise RuntimeError("step trace empty")
        t0=[i for i,r in enumerate(rows) if r["event_bits"]&(1<<3)]
        if len(t0)!=1: raise RuntimeError(f"step trace needs exactly one t0 marker, got {t0}")
        return rows

    def trace_clear(self):
        p=self.custom_transact(HB_CLEAR_TRACE,right=False,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_CLEAR_TRACE)
        if status: raise RuntimeError(f"trace_clear status={status}")

    def trace_freeze(self):
        p=self.custom_transact(HB_FREEZE_TRACE,right=False,timeout=max(self.timeout,1.2));status=parse_custom_header(p,HB_FREEZE_TRACE)
        if status: raise RuntimeError(f"trace_freeze status={status}")

    def set_steering_deg(self, deg: float):
        """LEFT steering signed physical degrees for ROS/Web (-30..+30)."""
        deg=max(-30.0,min(30.0,float(deg)))
        payload=self._custom(HB_SET_STEERING_DEG,struct.pack(">i",round(deg*1000.0)))
        self.send(payload)


    def diag(self, right: bool = False) -> Diag:
        # Diagnostic reply can overlap a previous endpoint reply on the single
        # UART transport. Filter by embedded VESC ID exactly like values().
        expected_id = RIGHT_ID if right else 1
        last = None
        for _ in range(3):
            try:
                p = self.custom_transact(HB_GET_DIAG, right=right, timeout=max(self.timeout, 1.20))
                d = parse_diag(p)
                if d.vesc_id == expected_id:
                    return d
                last = RuntimeError(f"stale diagnostic endpoint id={d.vesc_id}, expected={expected_id}")
            except (TimeoutError, RuntimeError) as exc:
                last = exc
        raise last


class ReplWorker:
    def __init__(self, link: VescDual, hz: float, telemetry_hz: float):
        self.link = link
        self.period = 1.0 / hz
        self.telemetry_period = 1.0 / telemetry_hz
        self.lock = threading.Lock()
        self.mode = "current"
        self.left = 0.0
        self.right = 0.0
        self.active = False
        self.stop_flag = False
        self.exit = False
        self.last_l = Values(vesc_id=1)
        self.last_r = Values(vesc_id=2)
        try:
            dl, dr = self.link.diag(False), self.link.diag(True)
            self.stop_erpm_l = 5 * (dl.pole_pairs or POLE_PAIRS)
            self.stop_erpm_r = 5 * (dr.pole_pairs or POLE_PAIRS)
        except Exception:
            self.stop_erpm_l = self.stop_erpm_r = STOP_ERPM
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set(self, mode: str, left: float, right: float):
        with self.lock:
            self.mode, self.left, self.right = mode, left, right
            self.active, self.stop_flag = True, False

    def stop_controlled(self):
        with self.lock:
            # Speed uses firmware RPM ramp-to-zero. Current/duty/position issue
            # zero current once and stop refreshing; the 500-ms ownership timeout
            # then releases the bridge to free-run.
            if self.mode == "rpm":
                self.left = 0.0
                self.right = 0.0
                self.stop_flag = True
                self.active = True
            else:
                self.stop_flag = False
                self.active = False
        if self.mode != "rpm":
            self.link.set_current(0.0, 0.0)

    def release(self):
        self.link.set_current(0.0, 0.0)
        with self.lock:
            self.active = False
            self.stop_flag = False

    def run(self):
        next_tick = next_tel = time.monotonic()
        while not self.exit:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now)); continue
            next_tick += self.period
            with self.lock:
                mode, l, r, active, stopping = self.mode, self.left, self.right, self.active, self.stop_flag
            try:
                if active:
                    if stopping and mode == "rpm":
                        self.link.set_rpm(0, 0)
                    elif mode == "current": self.link.set_current(l, r)
                    elif mode == "rpm": self.link.set_rpm(int(l), int(r))
                    elif mode == "duty": self.link.set_duty(l, r)
                    elif mode == "pos": self.link.set_pos(l, r)
                if now >= next_tel:
                    next_tel = now + self.telemetry_period
                    self.last_l = self.link.values(False)
                    self.last_r = self.link.values(True)
                    if stopping and mode == "rpm" and abs(self.last_l.rpm) <= self.stop_erpm_l and abs(self.last_r.rpm) <= self.stop_erpm_r:
                        self.link.set_rpm(0, 0)
                        with self.lock:
                            self.active = False; self.stop_flag = False
            except Exception as e:
                print(f"[WARN] {e}")

    def shutdown(self):
        self.exit = True
        self.thread.join(timeout=1.0)
        try: self.link.set_current(0.0, 0.0)
        except Exception: pass


def parse_fw(p: bytes) -> str:
    if len(p) < 4: return repr(p)
    major, minor = p[1], p[2]
    end = p.find(b"\0", 3)
    hw = p[3:end].decode(errors="replace") if end >= 0 else "?"
    return f"FW {major}.{minor:02d} HW={hw}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="auto",
                    help="auto | direct | direct:/dev/ttyUSB0 | raw /dev/ttyUSBx")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--command-hz", type=float, default=50.0)
    ap.add_argument("--telemetry-hz", type=float, default=50.0,
                    help="selective VESC telemetry polling; default 50 Hz")
    args = ap.parse_args()
    link = VescDual(args.port, args.baud)
    try:
        print("local:", parse_fw(link.fw(False)))
        print("virtual CAN:", link.ping_can())
        print("right:", parse_fw(link.fw(True)))
        w = ReplWorker(link, args.command_hz, args.telemetry_hz)
        print("commands: current L R [A] | rpm L R [ERPM] | duty L R [-1..1] | pos L R [deg] | hall [A] [left|right|both] | diag | posstate | stop | release | values | scan | fw | quit")
        while True:
            try: line = input("vesc-dual> ").strip()
            except (EOFError, KeyboardInterrupt): break
            if not line: continue
            a = line.split(); cmd = a[0].lower()
            try:
                if cmd == "current" and len(a) == 3: w.set("current", float(a[1]), float(a[2]))
                elif cmd == "rpm" and len(a) == 3: w.set("rpm", float(a[1]), float(a[2]))
                elif cmd == "duty" and len(a) == 3: w.set("duty", float(a[1]), float(a[2]))
                elif cmd in ("pos", "position") and len(a) == 3: w.set("pos", float(a[1]), float(a[2]))
                elif cmd == "stop": w.stop_controlled()
                elif cmd == "release": w.release()
                elif cmd == "values": print("L", w.last_l.short()); print("R", w.last_r.short())
                elif cmd == "scan": print("virtual CAN IDs:", link.ping_can())
                elif cmd == "fw": print("L", parse_fw(link.fw(False))); print("R", parse_fw(link.fw(True)))
                elif cmd == "diag": print("L", link.diag(False).short()); print("R", link.diag(True).short())
                elif cmd == "posstate": print("L", link.position_state(False)); print("R", link.position_state(True))
                elif cmd == "hall":
                    current = float(a[1]) if len(a) >= 2 else 1.0
                    which = a[2].lower() if len(a) >= 3 else "both"
                    if which in ("left", "l", "both"):
                        ok, tab = link.detect_hall(current, False); print("Hall LEFT", "OK" if ok else "FAIL", tab)
                    if which in ("right", "r", "both"):
                        ok, tab = link.detect_hall(current, True); print("Hall RIGHT", "OK" if ok else "FAIL", tab)
                elif cmd in ("quit", "exit", "q"): break
                else: print("usage: current L R | rpm L R | duty L R | pos L R | hall [A] [left|right|both] | stop | release | values | scan | fw | quit")
            except Exception as e:
                print("[ERR]", e)
        w.shutdown()
    finally:
        link.close()

if __name__ == "__main__":
    main()
