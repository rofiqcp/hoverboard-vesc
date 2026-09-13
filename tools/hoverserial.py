#!/usr/bin/env python3
"""Interactive USART3 terminal for VESC-style baremetal dual-motor hoverboard firmware.

The same USART3 link carries:
- binary independent Left/Right motor commands (host -> STM32)
- binary telemetry (STM32 -> host)
- ASCII firmware debug commands (GET/SET/WATCH/INIT/SAVE/HELP)

Normal workflow:
    mode 1
    start 50,50
    drive 100,100
    stop

`start` begins CSV logging and starts the motors. `stop` is mode-aware:
VLT/TRQ ramp their command/current to zero and then release the bridge for
free-running, while SPD ramps its speed setpoint to zero before resetting the
PI state and releasing the motor.
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "pyserial is required. Install with: python -m pip install -r tools/requirements.txt"
    ) from exc

try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.patch_stdout import patch_stdout
except ImportError as exc:
    raise SystemExit(
        "prompt_toolkit is required so telemetry does not overwrite typed commands. "
        "Install with: python -m pip install -r tools/requirements.txt"
    ) from exc

START = 0xABCD
START_BYTES = struct.pack("<H", START)

# Command: start, cmdL, cmdR, checksum
CMD = struct.Struct("<HhhH")

# Feedback V2 must match Src/main.c SerialFeedback exactly.  The field names
# intentionally mirror VESC mc_values semantics while transport stays the proven
# baremetal USART3 framing used by this board.
# H,H = start, version
# 20h = cmdL/R, rpmL/R, dutyL/R, currentMotorL/R, currentInL/R,
#       idL/R, iqL/R, vdL/R, vqL/R, vIn, boardTemp
# 11H = hallL/R, faultL/R, six raw ADC values, status
# I,H = foc_isr_cycles, checksum
FB = struct.Struct("<HH20h11HIH")

CPU_HZ = 64_000_000
FOC_HZ = 16_000
FOC_BUDGET_CYCLES = CPU_HZ // FOC_HZ
CMD_MIN = -1500
CMD_MAX = 1500

MODE_NAMES = {
    1: "VLT",
    2: "SPD",
    3: "TRQ current (command in cA)",
    4: "SVPWM sensorless Id-current PI (command magnitude in A)",
    5: "POS multi-turn Hall position",
}


def clamp_cmd(value: int, mode: int | None = None) -> int:
    value = int(value)
    if mode == 5:
        limit = 0
    elif mode == 4:
        # Mode 4 command magnitude is Id target in whole amperes.
        limit = 15
    elif mode == 3:
        # Mode 3 command is centiamperes: +/-1500 = +/-15.00 A.
        limit = 1500
    else:
        # Voltage/speed retain the established normalized +/-1000 range.
        limit = 1000
    return max(-limit, min(limit, value))


def xor16(words) -> int:
    value = 0
    for word in words:
        value ^= int(word) & 0xFFFF
    return value & 0xFFFF


def make_command(cmd_l: int, cmd_r: int) -> bytes:
    # Values are already mode-clamped by the interactive layer. Keep only the
    # protocol int16 safety bound here so mode 3 can legally transmit +/-1500.
    cmd_l = max(-32768, min(32767, int(cmd_l)))
    cmd_r = max(-32768, min(32767, int(cmd_r)))
    return CMD.pack(START, cmd_l, cmd_r, xor16((START, cmd_l, cmd_r)))


def format_amp_centi(value_cA: int) -> str:
    """Compact ampere display: 200 cA -> '2', 235 cA -> '2.35'."""
    text = f"{value_cA / 100.0:.2f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


@dataclass(frozen=True)
class Telemetry:
    """VESC-style user-facing telemetry over the existing baremetal frame.

    Right rpm/iq/currentMotor are normalized by firmware so positive follows the
    same forward/positive-wheel-torque convention as cmdR.  Id, Hall and raw ADC
    stay in the physical controller convention for diagnostics.
    """
    version: int
    cmd_l: int
    cmd_r: int
    rpm_l: int
    rpm_r: int
    duty_l_x1000: int
    duty_r_x1000: int
    current_motor_l_cA: int
    current_motor_r_cA: int
    current_in_l_cA: int
    current_in_r_cA: int
    id_l_cA: int
    id_r_cA: int
    iq_l_cA: int
    iq_r_cA: int
    vd_l_cV: int
    vd_r_cV: int
    vq_l_cV: int
    vq_r_cV: int
    battery_x100: int
    temp_x10: int
    hall_l: int
    hall_r: int
    fault_l: int
    fault_r: int
    adc_dcl: int
    adc_rla: int
    adc_rlb: int
    adc_dcr: int
    adc_rrb: int
    adc_rrc: int
    status: int
    foc_cycles: int

    @property
    def enabled(self) -> bool:
        return bool(self.status & 0x01)

    @property
    def timeout(self) -> bool:
        return bool(self.status & 0x02)

    @property
    def stopped(self) -> bool:
        return self.cmd_l == 0 and self.cmd_r == 0

    def summary(self) -> str:
        return (
            f"cmd={self.cmd_l},{self.cmd_r} "
            f"rpm={self.rpm_l},{self.rpm_r} "
            f"duty={self.duty_l_x1000/10:.1f}%,{self.duty_r_x1000/10:.1f}% "
            f"iq={format_amp_centi(self.iq_l_cA)},{format_amp_centi(self.iq_r_cA)} "
            f"id={format_amp_centi(self.id_l_cA)},{format_amp_centi(self.id_r_cA)} "
            f"iin={format_amp_centi(self.current_in_l_cA)},{format_amp_centi(self.current_in_r_cA)} "
            f"vd={self.vd_l_cV/100:.2f},{self.vd_r_cV/100:.2f} "
            f"vq={self.vq_l_cV/100:.2f},{self.vq_r_cV/100:.2f} "
            f"fault={self.fault_l},{self.fault_r} "
            f"hall={self.hall_l & 0x7:03b},{self.hall_r & 0x7:03b} "
            f"dcl={self.adc_dcl} rla={self.adc_rla} rlb={self.adc_rlb} "
            f"dcr={self.adc_dcr} rrb={self.adc_rrb} rrc={self.adc_rrc} "
            f"cycle={self.foc_cycles}"
        )


def decode_feedback(packet: bytes) -> Telemetry | None:
    if len(packet) != FB.size:
        return None
    fields = FB.unpack(packet)
    if fields[0] != START or fields[1] != 2:
        return None

    # XOR all 16-bit words before checksum.  The 32-bit ISR cycle counter
    # contributes its low and high words exactly as firmware feedbackChecksum().
    words = list(fields[:33])
    cycles = fields[33]
    words.extend((cycles & 0xFFFF, (cycles >> 16) & 0xFFFF))
    expected = xor16(words)
    if fields[34] != expected:
        return None

    return Telemetry(*fields[1:34])


class CsvLogger:
    FIELDNAMES = [
        "host_time", "elapsed_s", "telemetry_version",
        "cmdL", "cmdR", "rpmL", "rpmR", "dutyL_pct", "dutyR_pct",
        "currentMotorL_A", "currentMotorR_A", "currentInL_A", "currentInR_A",
        "idL_A", "idR_A", "iqL_A", "iqR_A",
        "vdL_V", "vdR_V", "vqL_V", "vqR_V",
        "hallL", "hallR", "faultL", "faultR",
        "dcl_raw", "rla_raw", "rlb_raw", "dcr_raw", "rrb_raw", "rrc_raw",
        "battery_V", "temperature_C", "foc_isr_cycles", "foc_isr_us",
        "foc_isr_load_pct", "status",
    ]

    def __init__(self, directory: Path):
        self.directory = directory
        self._lock = threading.Lock()
        self._fp = None
        self._writer = None
        self._path: Path | None = None
        self._started_mono = 0.0
        self._rows = 0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._fp is not None

    @property
    def path(self) -> Path | None:
        with self._lock:
            return self._path

    def start(self, mode: int | None) -> Path:
        with self._lock:
            if self._fp is not None:
                raise RuntimeError("CSV logging is already active")
            self.directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            suffix = f"_mode{mode}" if mode in MODE_NAMES else ""
            path = self.directory / f"hover_{stamp}{suffix}.csv"
            self._fp = path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._fp, fieldnames=self.FIELDNAMES)
            self._writer.writeheader()
            self._fp.flush()
            self._path = path
            self._started_mono = time.monotonic()
            self._rows = 0
            return path

    def write(self, tel: Telemetry) -> None:
        with self._lock:
            if self._fp is None or self._writer is None:
                return
            now = datetime.now().isoformat(timespec="milliseconds")
            elapsed = time.monotonic() - self._started_mono
            self._writer.writerow(
                {
                    "host_time": now, "elapsed_s": f"{elapsed:.3f}",
                    "telemetry_version": tel.version,
                    "cmdL": tel.cmd_l, "cmdR": tel.cmd_r,
                    "rpmL": tel.rpm_l, "rpmR": tel.rpm_r,
                    "dutyL_pct": f"{tel.duty_l_x1000 / 10.0:.1f}",
                    "dutyR_pct": f"{tel.duty_r_x1000 / 10.0:.1f}",
                    "currentMotorL_A": f"{tel.current_motor_l_cA / 100.0:.2f}",
                    "currentMotorR_A": f"{tel.current_motor_r_cA / 100.0:.2f}",
                    "currentInL_A": f"{tel.current_in_l_cA / 100.0:.2f}",
                    "currentInR_A": f"{tel.current_in_r_cA / 100.0:.2f}",
                    "idL_A": f"{tel.id_l_cA / 100.0:.2f}",
                    "idR_A": f"{tel.id_r_cA / 100.0:.2f}",
                    "iqL_A": f"{tel.iq_l_cA / 100.0:.2f}",
                    "iqR_A": f"{tel.iq_r_cA / 100.0:.2f}",
                    "vdL_V": f"{tel.vd_l_cV / 100.0:.2f}",
                    "vdR_V": f"{tel.vd_r_cV / 100.0:.2f}",
                    "vqL_V": f"{tel.vq_l_cV / 100.0:.2f}",
                    "vqR_V": f"{tel.vq_r_cV / 100.0:.2f}",
                    "hallL": f"{tel.hall_l & 0x7:03b}", "hallR": f"{tel.hall_r & 0x7:03b}",
                    "faultL": tel.fault_l, "faultR": tel.fault_r,
                    "dcl_raw": tel.adc_dcl, "rla_raw": tel.adc_rla, "rlb_raw": tel.adc_rlb,
                    "dcr_raw": tel.adc_dcr, "rrb_raw": tel.adc_rrb, "rrc_raw": tel.adc_rrc,
                    "battery_V": f"{tel.battery_x100 / 100.0:.2f}",
                    "temperature_C": f"{tel.temp_x10 / 10.0:.1f}",
                    "foc_isr_cycles": tel.foc_cycles,
                    "foc_isr_us": f"{tel.foc_cycles * 1_000_000.0 / CPU_HZ:.3f}",
                    "foc_isr_load_pct": f"{tel.foc_cycles * 100.0 / FOC_BUDGET_CYCLES:.2f}",
                    "status": tel.status,
                }
            )
            self._rows += 1
            if (self._rows % 20) == 0:
                self._fp.flush()

    def stop(self) -> tuple[Path | None, int]:
        with self._lock:
            path = self._path
            rows = self._rows
            if self._fp is not None:
                self._fp.flush()
                self._fp.close()
            self._fp = None
            self._writer = None
            self._path = None
            self._rows = 0
            return path, rows


class HoverSerial:
    def __init__(self, port: str, baud: int, logger: CsvLogger, print_hz: float = 10.0):
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=0.5)
        self.logger = logger
        self.stop_event = threading.Event()
        self.buffer = bytearray()
        self.write_lock = threading.Lock()
        self.telemetry_lock = threading.Lock()
        self.latest: Telemetry | None = None
        self.telemetry_seq = 0
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.last_print = 0.0
        self.print_period = 1.0 / max(1.0, print_hz)

    def start_reader(self) -> None:
        self.ser.reset_input_buffer()
        self.reader.start()

    def close(self) -> None:
        self.stop_event.set()
        self.reader.join(timeout=0.5)
        if self.ser.is_open:
            self.ser.close()

    def send_drive(self, cmd_l: int, cmd_r: int) -> None:
        with self.write_lock:
            self.ser.write(make_command(cmd_l, cmd_r))

    def debug(self, command: str) -> None:
        command = command.strip()
        if not command:
            return
        with self.write_lock:
            self.ser.write((command + "\n").encode("ascii", "strict"))

    def snapshot(self) -> tuple[int, Telemetry | None]:
        with self.telemetry_lock:
            return self.telemetry_seq, self.latest

    def _print_ascii(self, data: bytes) -> None:
        if not data:
            return
        text = data.decode("ascii", "ignore")
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if line:
                print(f"[DBG] {line}")

    def _consume(self) -> None:
        while self.buffer:
            idx = self.buffer.find(START_BYTES)
            if idx < 0:
                # Keep one byte in case it is the first byte of the start marker.
                if len(self.buffer) > 1:
                    self._print_ascii(bytes(self.buffer[:-1]))
                    del self.buffer[:-1]
                return

            if idx > 0:
                self._print_ascii(bytes(self.buffer[:idx]))
                del self.buffer[:idx]

            if len(self.buffer) < FB.size:
                return

            packet = bytes(self.buffer[: FB.size])
            telemetry = decode_feedback(packet)
            if telemetry is None:
                self._print_ascii(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            del self.buffer[: FB.size]
            with self.telemetry_lock:
                self.latest = telemetry
                self.telemetry_seq += 1

            # Log every valid feedback frame, even though display is rate-limited.
            self.logger.write(telemetry)

            now = time.monotonic()
            if now - self.last_print >= self.print_period:
                print(telemetry.summary())
                self.last_print = now

    def _reader_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as exc:
                print(f"[ERR] serial read: {exc}", file=sys.stderr)
                self.stop_event.set()
                return
            if chunk:
                self.buffer.extend(chunk)
                self._consume()


def print_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for port in ports:
        print(f"{port.device:12s} {port.description} {port.hwid}")


def parse_pair(parts: list[str], mode: int | None) -> tuple[int, int]:
    """Accept `50,50` or `50 50` after start/drive."""
    if len(parts) == 2 and "," in parts[1]:
        values = parts[1].split(",", 1)
    elif len(parts) == 3:
        values = parts[1:3]
    else:
        raise ValueError("expected two motor commands, e.g. 50,50")
    return clamp_cmd(int(values[0].strip()), mode), clamp_cmd(int(values[1].strip()), mode)


def print_local_help() -> None:
    print("Commands:")
    print("  mode 1|2|3|4|5     change mode only while fully stopped")
    print("                     1=VLT 2=SPD 3=TRQ 4=SVPWM sensorless Id PI 5=POS")
    print("  mode 3 scaling     50=0.50A, 100=1.00A, 1500=15.00A (Iq_ref, Id_ref=0)")
    print("  mode 4 scaling     start 2,2 => Id_ref=2A, Iq_ref=0; sign selects rotation direction")
    print("                     SET SVPWM_RPM 10 is the safe default; change it explicitly if needed")
    print("  start L,R          start CSV + command Left/Right motors")
    print("  pos L R            set signed int32 position targets in mode 5")
    print("  reset pos          reset position counters")
    print("  drive L,R          change independent Left/Right command")
    print("  stop               VLT/TRQ ramp then free-run; SPD speed-ramp then free-run")
    print("  status             print latest telemetry once")
    print("  get/set/watch/...   pass firmware debug command over USART3")
    print("  quit                safe ramp-down, save CSV, exit")


def wait_until_fully_stopped(link: HoverSerial, timeout_s: float) -> bool:
    """Wait for post-command feedback confirming applied cmdL=cmdR=0."""
    start_seq, _ = link.snapshot()
    deadline = time.monotonic() + timeout_s
    zero_frames = 0
    last_seq = start_seq

    while time.monotonic() < deadline and not link.stop_event.is_set():
        seq, tel = link.snapshot()
        if seq != last_seq:
            last_seq = seq
            if seq > start_seq and tel is not None and tel.stopped:
                zero_frames += 1
                if zero_frames >= 3:
                    return True
            else:
                zero_frames = 0
        time.sleep(0.01)
    return False



def wait_until_motion_stopped(link: HoverSerial, timeout_s: float, rpm_deadband: int = 5) -> bool:
    """Wait for both measured Hall speeds to settle near zero for 3 frames."""
    deadline = time.monotonic() + timeout_s
    start_seq, _ = link.snapshot()
    zero_frames = 0
    while time.monotonic() < deadline and not link.stop_event.is_set():
        seq, tel = link.snapshot()
        if seq > start_seq and tel is not None:
            if abs(tel.rpm_l) <= rpm_deadband and abs(tel.rpm_r) <= rpm_deadband:
                zero_frames += 1
                if zero_frames >= 3:
                    return True
            else:
                zero_frames = 0
        time.sleep(0.01)
    return False

def wait_until_current_reference_relaxed(link: HoverSerial, timeout_s: float, current_deadband_cA: int = 20) -> bool:
    """Wait until measured Iq is small before releasing torque mode to free-run."""
    deadline = time.monotonic() + timeout_s
    start_seq, _ = link.snapshot()
    quiet_frames = 0
    while time.monotonic() < deadline and not link.stop_event.is_set():
        seq, tel = link.snapshot()
        if seq > start_seq and tel is not None:
            if abs(tel.iq_l_cA) <= current_deadband_cA and abs(tel.iq_r_cA) <= current_deadband_cA:
                quiet_frames += 1
                if quiet_frames >= 3:
                    return True
            else:
                quiet_frames = 0
        time.sleep(0.01)
    return False


def interactive(
    link: HoverSerial,
    logger: CsvLogger,
    rate_hz: float,
    stop_timeout_s: float,
) -> None:
    state_lock = threading.Lock()
    target = [0, 0]
    tx_stop = threading.Event()
    selected_mode: int | None = None

    def set_target(cmd_l: int, cmd_r: int) -> None:
        with state_lock:
            target[0] = clamp_cmd(cmd_l, selected_mode)
            target[1] = clamp_cmd(cmd_r, selected_mode)

    def get_target() -> tuple[int, int]:
        with state_lock:
            return target[0], target[1]

    def tx_loop() -> None:
        period = 1.0 / max(1.0, rate_hz)
        next_tx = time.monotonic()
        while not tx_stop.is_set() and not link.stop_event.is_set():
            cmd_l, cmd_r = get_target()
            try:
                link.send_drive(cmd_l, cmd_r)
            except serial.SerialException as exc:
                print(f"[ERR] serial write: {exc}", file=sys.stderr)
                link.stop_event.set()
                return
            next_tx += period
            delay = next_tx - time.monotonic()
            if delay > 0:
                tx_stop.wait(delay)
            else:
                next_tx = time.monotonic()

    def save_csv() -> None:
        path, rows = logger.stop()
        if path is not None:
            print(f"[CSV] saved {rows} rows -> {path}")

    def safe_stop() -> bool:
        # Host target becomes zero immediately; firmware has the authoritative
        # per-mode ramp and release semantics. Keep TX alive until the legacy
        # command path itself has reached zero.
        set_target(0, 0)
        print("[STOP] target=0,0; waiting for rate-limited cmdL/cmdR to reach 0,0 ...")
        stopped = wait_until_fully_stopped(link, stop_timeout_s)
        if stopped:
            print("[STOP] cmdL=0 cmdR=0 confirmed")

            if selected_mode == 2:
                print("[STOP] SPD ramp-to-zero active; waiting near zero RPM before release ...")
                motion_stopped = wait_until_motion_stopped(link, stop_timeout_s, rpm_deadband=5)
                if motion_stopped:
                    print("[STOP] SPD near-zero confirmed; PI reset + free-run release")
                else:
                    print("[WARN] SPD did not settle within timeout; requesting safe free-run release")
                link.debug("SET MOT_RUN 0")
            elif selected_mode == 3:
                print("[STOP] TRQ Iq ramp-to-zero active; no braking ...")
                current_relaxed = wait_until_current_reference_relaxed(link, min(stop_timeout_s, 1.5))
                if current_relaxed:
                    print("[STOP] TRQ current relaxed; releasing motor to free-run")
                else:
                    print("[WARN] TRQ current did not fully settle; requesting free-run release")
                link.debug("SET MOT_RUN 0")
            elif selected_mode == 1:
                print("[STOP] VLT zero reached; releasing bridge to free-run")
                link.debug("SET MOT_RUN 0")
            else:
                # Mode 4 keeps its existing Id ramp-down behavior.
                link.debug("SET MOT_RUN 0")
        else:
            print(
                "[WARN] zero target is still being transmitted, but telemetry did not "
                f"confirm cmdL=0/cmdR=0 within {stop_timeout_s:.1f}s"
            )
        save_csv()
        return stopped

    tx = threading.Thread(target=tx_loop, daemon=True)
    tx.start()

    # Allow the first valid 0,0 command/telemetry frames to settle.
    print("Left/Right independent control over USART3")
    print("mode: 1=VLT 2=SPD 3=TRQ(cA) 4=SVPWM Id(A) 5=POS")
    print("example: mode 1 -> start 50,50 -> drive 100,100 -> stop")
    print("type 'help' for commands")

    session = PromptSession()

    try:
        # patch_stdout redraws the prompt and partially typed text after each
        # telemetry/debug print, preventing the old overlapping-terminal issue.
        with patch_stdout(raw=True):
            while not link.stop_event.is_set():
                try:
                    line = session.prompt("hover> ").strip()
                except EOFError:
                    break
                if not line:
                    continue

                parts = line.split()
                op = parts[0].lower()

                try:
                    if op == "mode":
                        if len(parts) != 2 or not parts[1].isdigit():
                            raise ValueError("usage: mode 1|2|3|4|5")
                        mode = int(parts[1])
                        if mode not in MODE_NAMES:
                            raise ValueError("mode must be 1, 2, 3, 4, or 5")
                        if logger.active:
                            print("[ERR] stop the active run before changing mode")
                            continue
                        if get_target() != (0, 0):
                            print("[ERR] mode can change only with target 0,0")
                            continue
                        _, tel = link.snapshot()
                        if tel is None or not tel.stopped:
                            print("[ERR] mode can change only after telemetry confirms cmdL=0 cmdR=0")
                            continue
                        link.debug(f"SET CTRL_MOD {mode}")
                        selected_mode = mode
                        print(f"[MODE] requested {mode}={MODE_NAMES[mode]}")

                    elif op == "start":
                        cmd_l, cmd_r = parse_pair(parts, selected_mode)
                        if logger.active:
                            print("[ERR] run already active; use drive L,R or stop")
                            continue
                        _, tel = link.snapshot()
                        if get_target() != (0, 0) or tel is None or not tel.stopped:
                            print("[ERR] start is allowed only from fully stopped cmdL=0 cmdR=0")
                            continue
                        # Clear any explicit TRQ-stop latch before a new run.
                        # In modes 1/2/4 this parameter is intentionally ignored.
                        link.debug("SET MOT_RUN 1")
                        path = logger.start(selected_mode)
                        print(f"[CSV] recording -> {path}")
                        set_target(cmd_l, cmd_r)
                        print(f"[START] cmdL={cmd_l} cmdR={cmd_r}")

                    elif op == "drive":
                        cmd_l, cmd_r = parse_pair(parts, selected_mode)
                        if not logger.active:
                            print("[ERR] no active run; use start L,R first")
                            continue
                        set_target(cmd_l, cmd_r)
                        print(f"[DRIVE] cmdL={cmd_l} cmdR={cmd_r}")

                    elif op == "stop":
                        safe_stop()

                    elif op == "reset" and len(parts)==2 and parts[1].lower()=="pos":
                        link.debug("RESETPOS"); print("[POS] reset")

                    elif op in {"pos","position"}:
                        if selected_mode!=5: raise ValueError("select mode 5 first")
                        if len(parts)!=3: raise ValueError("usage: pos LEFT RIGHT")
                        pl,pr=int(parts[1]),int(parts[2])
                        if not(-2147483648<=pl<=2147483647 and -2147483648<=pr<=2147483647): raise ValueError("position must fit int32")
                        link.debug(f"SET PSETL {pl}"); link.debug(f"SET PSETR {pr}"); link.debug("SET MOT_RUN 1")
                        print(f"[POS] target={pl},{pr}")

                    elif op == "status":
                        _, tel = link.snapshot()
                        print(tel.summary() if tel is not None else "[STATUS] no telemetry yet")

                    elif op == "help":
                        print_local_help()

                    elif op in {"get", "set", "watch", "save", "init"}:
                        # Keep legacy firmware debug terminal access on USART3.
                        link.debug(line.upper())

                    elif op in {"fwhelp", "fw-help"}:
                        link.debug("HELP")

                    elif op in {"quit", "exit", "q"}:
                        break

                    else:
                        print("[ERR] unknown command; type 'help'")

                except ValueError as exc:
                    print(f"[ERR] {exc}")

    finally:
        # Always use the same gradual firmware rate limiter on normal exit.
        if get_target() != (0, 0) or logger.active:
            safe_stop()
        else:
            set_target(0, 0)
            save_csv()
        # Give the TX loop one final chance to send 0,0 before closing.
        try:
            link.send_drive(0, 0)
        except serial.SerialException:
            pass
        time.sleep(0.03)
        tx_stop.set()
        tx.join(timeout=0.5)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="DEPRECATED legacy hoverboard serial tool (USART3 is VESC-only)"
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--rate", type=float, default=20.0, help="binary command TX rate Hz")
    parser.add_argument("--print-rate", type=float, default=10.0, help="telemetry display rate Hz")
    parser.add_argument(
        "--stop-timeout",
        type=float,
        default=5.0,
        help="seconds to wait for rate-limited cmdL/cmdR to reach zero",
    )
    parser.add_argument(
        "--csv-dir",
        default=str(Path(__file__).resolve().parent / "logs"),
        help="directory for automatic CSV files",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="telemetry/debug monitor only; do not transmit motor commands",
    )
    args = parser.parse_args()

    raise SystemExit(
        "DEPRECATED: legacy HoverSerial frames are not accepted by the VESC-exclusive "
        "F103 USART3 transport. Use tools/vesc_debug.py or tools/vesc_dual.py @ 115200."
    )

    if args.list_ports:
        print_ports()
        if not args.port:
            return

    logger = CsvLogger(Path(args.csv_dir))
    link = HoverSerial(args.port, args.baud, logger=logger, print_hz=args.print_rate)
    link.start_reader()
    print(
        f"Connected {args.port} @ {args.baud}; feedback={FB.size} bytes; "
        f"FOC budget={FOC_BUDGET_CYCLES} cycles"
    )

    try:
        if args.monitor:
            while not link.stop_event.is_set():
                time.sleep(0.2)
        else:
            interactive(link, logger, args.rate, args.stop_timeout)
    except KeyboardInterrupt:
        pass
    finally:
        path, rows = logger.stop()
        if path is not None:
            print(f"[CSV] saved {rows} rows -> {path}")
        link.close()


if __name__ == "__main__":
    main()
