#!/usr/bin/env python3
"""Single interactive VESC Tool-like CLI for the dual F103 hoverboard controller.

LEFT is the local VESC endpoint (ID 1). RIGHT is VESC ID 2 through
COMM_FORWARD_CAN. All wire encoding/decoding is provided by vesc_dual.py.

The interactive shell keeps VESC telemetry scrolling while prompt_toolkit keeps
partially typed commands intact, similar to a serial/Arduino monitor.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shlex
import struct
import threading
import time
from contextlib import contextmanager
from dataclasses import asdict, replace
from pathlib import Path

try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
    from prompt_toolkit.completion import WordCompleter
    from prompt_toolkit.history import FileHistory
    from prompt_toolkit.patch_stdout import patch_stdout
except ImportError as exc:
    raise SystemExit("prompt_toolkit required: python3 -m pip install -r tools/requirements.txt") from exc

from vesc_dual import (
    DEFAULT_BAUD, VescDual, PacketDecoder, frame, crc16, parse_fw,
    COMM_SET_DUTY, COMM_SET_CURRENT, COMM_SET_CURRENT_BRAKE, COMM_SET_RPM,
    COMM_SET_POS, COMM_SET_HANDBRAKE, COMM_SET_CURRENT_REL,
)

TARGETS = {"l": "left", "left": "left", "r": "right", "right": "right",
           "b": "both", "both": "both", "all": "both"}
MOTION_MODES = {"current", "current_rel", "rpm", "duty", "pos", "brake", "handbrake"}


def motors(spec: str) -> list[tuple[str, bool]]:
    spec = TARGETS.get(spec.lower(), spec.lower())
    if spec == "left": return [("L", False)]
    if spec == "right": return [("R", True)]
    if spec == "both": return [("L", False), ("R", True)]
    raise ValueError("target harus left | right | both")


def split_target(words: list[str], default: str) -> tuple[list[str], str]:
    if words and words[-1].lower() in TARGETS:
        return words[:-1], TARGETS[words[-1].lower()]
    return words, default


def hash_raw(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def enc_setpoint(mode: str, value: float) -> bytes:
    if mode == "current":
        if abs(value) > 30.0: raise ValueError("current dibatasi CLI ke +/-30 A")
        return bytes((COMM_SET_CURRENT,)) + struct.pack(">i", round(value * 1000.0))
    if mode == "current_rel":
        if abs(value) > 1.0: raise ValueError("current_rel harus -1..1")
        return bytes((COMM_SET_CURRENT_REL,)) + struct.pack(">i", round(value * 100000.0))
    if mode == "rpm":
        if abs(value) > 15000: raise ValueError("ERPM CLI dibatasi +/-15000")
        return bytes((COMM_SET_RPM,)) + struct.pack(">i", round(value))
    if mode == "duty":
        if abs(value) > 1.0: raise ValueError("duty harus -1..1")
        return bytes((COMM_SET_DUTY,)) + struct.pack(">i", round(value * 100000.0))
    if mode == "pos":
        raw = round(value * 1_000_000.0)
        if not -2147483648 <= raw <= 2147483647: raise ValueError("position x1e6 overflow int32")
        return bytes((COMM_SET_POS,)) + struct.pack(">i", raw)
    if mode == "brake":
        if abs(value) > 30.0: raise ValueError("brake dibatasi CLI ke +/-30 A")
        return bytes((COMM_SET_CURRENT_BRAKE,)) + struct.pack(">i", round(value * 1000.0))
    if mode == "handbrake":
        if abs(value) > 30.0: raise ValueError("handbrake dibatasi CLI ke +/-30 A")
        return bytes((COMM_SET_HANDBRAKE,)) + struct.pack(">i", round(value * 1000.0))
    raise ValueError(f"mode setpoint tidak dikenal: {mode}")


class LiveWorker:
    def __init__(self, link: VescDual, command_hz: float = 50.0, telemetry_hz: float = 5.0):
        self.link = link
        self.command_hz = max(1.0, min(100.0, command_hz))
        self.telemetry_hz = max(0.0, min(50.0, telemetry_hz))
        self.telemetry_enabled = self.telemetry_hz > 0
        self.active: dict[bool, tuple[str, float] | None] = {False: None, True: None}
        self.latest = {False: None, True: None}
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.pause_evt = threading.Event()
        self.last_error = 0.0
        # Match upstream VESC Tool: while connected, COMM_ALIVE is sent every
        # 200 ms even with no active motor setpoint. Telemetry polling is
        # independent from motor activity, so idle/released motors remain live
        # on the realtime display without injecting a fake zero setpoint.
        self.alive_hz = 5.0
        self.thread = threading.Thread(target=self._run, daemon=True, name="vesc-tool-live")
        self.thread.start()

    def set_target(self, spec: str, mode: str, vals: list[float]) -> None:
        ms = motors(spec)
        if len(ms) == 2:
            if len(vals) == 1: vals = [vals[0], vals[0]]
            if len(vals) != 2: raise ValueError("target both: beri satu nilai (sama) atau dua nilai LEFT RIGHT")
        elif len(vals) != 1:
            raise ValueError("target satu motor membutuhkan tepat satu nilai")
        with self.lock:
            for (_, right), val in zip(ms, vals):
                enc_setpoint(mode, val)  # validate before storing
                self.active[right] = (mode, val)

    def stop(self, spec: str = "both") -> None:
        with self.lock:
            for _, right in motors(spec): self.active[right] = None
        zero = enc_setpoint("current", 0.0)
        for _ in range(3):
            for _, right in motors(spec): self.link.send_no_reply(zero, right)
            time.sleep(0.015)

    def set_telemetry(self, enabled: bool, hz: float | None = None) -> None:
        with self.lock:
            if hz is not None: self.telemetry_hz = max(0.2, min(50.0, float(hz)))
            self.telemetry_enabled = enabled

    @contextmanager
    def suspended(self, stop_motion: bool = False):
        if stop_motion: self.stop("both")
        self.pause_evt.set(); time.sleep(0.03)
        try: yield
        finally: self.pause_evt.clear()

    def _send_active(self) -> None:
        with self.lock: snapshot = dict(self.active)
        for right, item in snapshot.items():
            if item is None: continue
            mode, value = item
            self.link.send_no_reply(enc_setpoint(mode, value), right)

    def _send_alive(self) -> None:
        # VESC Tool MainWindow::timerSlot() sends COMM_ALIVE every 10 x 20 ms.
        # Keep both virtual controllers alive even while idle; COMM_ALIVE does
        # not alter the requested duty/current/RPM/position setpoint.
        self.link.alive(False)
        self.link.alive(True)

    def _commanded_value(self, right: bool, expected_mode: str) -> float | None:
        with self.lock:
            item = self.active[right]
        if item is None or item[0] != expected_mode:
            return None
        return float(item[1])

    @staticmethod
    def _fmt_setpoint(value: float | None, width: int, precision: int = 0) -> str:
        if value is None:
            return "-".rjust(width)
        return f"{value:{width}.{precision}f}"

    def telemetry_once(self, spec: str = "both") -> None:
        selected = motors(spec)
        values = {}
        for label, right in selected:
            v = self.link.values(right)
            self.latest[right] = v
            values[right] = v

        if spec == "both" or len(selected) == 2:
            left = values.get(False)
            right = values.get(True)
            if left is None or right is None:
                return
            set_pos = self._fmt_setpoint(self._commanded_value(False, "pos"), 7, 2)
            set_erpm = self._fmt_setpoint(self._commanded_value(True, "rpm"), 7, 0)
            vin = (left.vin + right.vin) * 0.5
            print(
                f"[RT] "
                f"L SETPOS={set_pos}deg GETPOS={left.position:7.2f}deg "
                f"Iq={left.iq:6.2f}A Id={left.id:6.2f}A "
                f"Imot={left.current_motor:6.2f}A Ibat={left.current_in:6.2f}A || "
                f"R SETERPM={set_erpm} GETERPM={right.rpm:7.0f} "
                f"Iq={right.iq:6.2f}A Id={right.id:6.2f}A "
                f"Imot={right.current_motor:6.2f}A Ibat={right.current_in:6.2f}A || "
                f"Vin={vin:5.1f}V",
                flush=True,
            )
            return

        label, right = selected[0]
        v = values[right]
        if right:
            set_erpm = self._fmt_setpoint(self._commanded_value(True, "rpm"), 7, 0)
            print(
                f"[RT R] SETERPM={set_erpm} GETERPM={v.rpm:7.0f} "
                f"Iq={v.iq:6.2f}A Id={v.id:6.2f}A "
                f"Imot={v.current_motor:6.2f}A Ibat={v.current_in:6.2f}A || Vin={v.vin:5.1f}V",
                flush=True,
            )
        else:
            set_pos = self._fmt_setpoint(self._commanded_value(False, "pos"), 7, 2)
            print(
                f"[RT L] SETPOS={set_pos}deg GETPOS={v.position:7.2f}deg "
                f"Iq={v.iq:6.2f}A Id={v.id:6.2f}A "
                f"Imot={v.current_motor:6.2f}A Ibat={v.current_in:6.2f}A || Vin={v.vin:5.1f}V",
                flush=True,
            )

    def _run(self) -> None:
        next_cmd = next_tel = next_alive = time.monotonic()
        while not self.stop_evt.is_set():
            if self.pause_evt.is_set():
                self.stop_evt.wait(0.01)
                next_cmd = next_tel = next_alive = time.monotonic()
                continue
            now = time.monotonic()
            try:
                if now >= next_alive:
                    self._send_alive()
                    next_alive = now + 1.0 / self.alive_hz
                if now >= next_cmd:
                    self._send_active(); next_cmd = now + 1.0 / self.command_hz
                with self.lock: tel_on, tel_hz = self.telemetry_enabled, self.telemetry_hz
                if tel_on and tel_hz > 0 and now >= next_tel:
                    self.telemetry_once("both"); next_tel = time.monotonic() + 1.0 / tel_hz
            except Exception as exc:
                if time.monotonic() - self.last_error > 1.0:
                    print(f"[RT WARN] {type(exc).__name__}: {exc}", flush=True); self.last_error = time.monotonic()
            self.stop_evt.wait(0.002)

    def close(self) -> None:
        try: self.stop("both")
        except Exception: pass
        self.stop_evt.set(); self.thread.join(timeout=1.0)


HELP = """
Core:
  help [firmware]                     daftar command / HELP firmware
  target left|right|both              target default berikutnya
  fw [target] | scan | info           identitas VESC / virtual CAN
  telemetry on [Hz]|off|once [target] realtime sambil prompt tetap aktif
  values|diag|setup [target]          telemetry/detail
  health                               watchdog/platform health
  platform | comms | isr [reset]       platform/comms/ISR diagnostics
  adc [target]                         ADC sampling validity

Control (direfresh otomatis 50 Hz):
  set current A [A_R] [target]         COMM_SET_CURRENT
  set current_rel R [R_R] [target]     COMM_SET_CURRENT_REL (-1..1)
  set rpm ERPM [ERPM_R] [target]       COMM_SET_RPM
  set duty D [D_R] [target]            COMM_SET_DUTY (-1..1)
  set pos DEG [DEG_R] [target]         COMM_SET_POS, x1e6 seperti VESC Tool
  set brake A [A_R] [target]           COMM_SET_CURRENT_BRAKE
  set handbrake A [A_R] [target]       COMM_SET_HANDBRAKE
  stop [target]                         zero-current + berhenti refresh

Steering / sensor:
  steering status|zero|center|home|reset|invert 0|1
  steering set DEG                      custom ROS steering -30..+30 (LEFT)
  encoder                               debug ABI LEFT
  sensor encoder|hall [target] [store]  ubah sensor via terminal firmware
  posstate [target]
  poscount status [target]
  poscount set COUNT [target]
  poscount limits MIN MAX [target]
  poscount reset [target]

Tuning:
  tuning get [target]
  tuning set pos KP KI KD [target] [store]
  tuning set speed KP KI KD [target] [store]
  tuning set focq KP KI [target] [store]
  tuning set focd KP KI [target] [store]
  tuning set filter VALUE [target] [store]

Detection / config:
  detect hall A [target] [store]         standalone Hall; store opsional
  detect encoder A [target]              electrical + span (LEFT normal)
  detect rl [target]
  detect flux I ERPM_S DUTY R_OHM L_H [target]
  detect all                              VESC Detect All FOC
  hall-phase ERPM [SECONDS] target        validasi Hall->phase FOC aktif
  current-step PRE STEP [q|d] target       synchronized D/Q current-step trace
  trace meta|clear|freeze|download [FILE]  ISR trace operations
  mc get|raw|default [target]
  app get|raw|default [target]
  config save FILE.yaml                   snapshot exact MC/App raw both
  config restore FILE.yaml [target]       restore exact payload MC+App
  battery get [target]
  battery set START END [target] [store]
  odometer METERS [target]
  alive [target] | shutdown [target] | reboot [target]
  bootloader                             handoff local F103 ke resident bootloader

Terminal VESC firmware:
  term COMMAND... [@left|@right|@both]    contoh: term status @left
  term steering zero @left
  term set sensor encoder @left
  term save mcconf @left
  term help @left                         seluruh command terminal firmware

Shortcut: current/rpm/duty/pos/brake/handbrake ... = set <mode> ...
quit / exit / q
""".strip()


class Console:
    def __init__(self, link: VescDual, worker: LiveWorker):
        self.link, self.worker = link, worker
        self.target = "both"

    def _term_targets(self, words: list[str]) -> tuple[list[str], str]:
        spec = self.target
        if words and words[-1].startswith("@"):
            raw = words[-1][1:].lower()
            if raw not in TARGETS: raise ValueError("@target harus @left/@right/@both")
            spec = TARGETS[raw]; words = words[:-1]
        return words, spec

    def _print_tuning(self, spec: str) -> None:
        for label, right in motors(spec):
            t = self.link.get_tuning(right)
            print(label, asdict(t)); print("  physical", t.physical)

    def _set_tuning(self, words: list[str]) -> None:
        store = False
        if words and words[-1].lower() == "store": store = True; words = words[:-1]
        words, spec = split_target(words, self.target)
        if len(words) < 2: raise ValueError("usage: tuning set pos|speed|focq|focd|filter ...")
        kind, nums = words[0].lower(), [float(x) for x in words[1:]]
        for label, right in motors(spec):
            t = self.link.get_tuning(right)
            if kind in ("pos", "position") and len(nums) == 3:
                t = replace(t, kpp=round(nums[0]*1000), kip=round(nums[1]*1000), kdp=round(nums[2]*1000))
            elif kind == "speed" and len(nums) == 3:
                t = replace(t, kps=round(nums[0]*100000), kis=round(nums[1]*100000), kds=round(nums[2]*100000))
            elif kind == "focq" and len(nums) == 2:
                t = replace(t, kpq=round(nums[0]*1536), kiq=round(nums[1]*4.608))
            elif kind == "focd" and len(nums) == 2:
                t = replace(t, kpd=round(nums[0]*1536), kid=round(nums[1]*4.608))
            elif kind == "filter" and len(nums) == 1:
                t = replace(t, telem_filter_q16=max(1,min(65535,round(nums[0]*65535))))
            else: raise ValueError("jumlah nilai tuning salah; ketik help")
            got = self.link.set_tuning(t, right=right, store=store)
            print(f"{label} tuning {'STORED' if store else 'RAM'}", got.physical)

    def _set_motion(self, mode: str, words: list[str]) -> None:
        words, spec = split_target(words, self.target)
        if not words: raise ValueError(f"usage: set {mode} VALUE [VALUE_R] [target]")
        vals = [float(x) for x in words]
        self.worker.set_target(spec, mode, vals)
        print(f"[SET] {mode} target={spec} values={vals} refresh={self.worker.command_hz:.1f}Hz")

    def _terminal(self, words: list[str]) -> None:
        words, spec = self._term_targets(words)
        if not words: raise ValueError("usage: term COMMAND... [@target]")
        command = " ".join(words)
        with self.worker.suspended(False):
            for label, right in motors(spec):
                print(f"--- {label} terminal: {command} ---")
                print(self.link.terminal(command, right).rstrip())

    def _snapshot(self, path: str) -> None:
        try: import yaml
        except ImportError as exc: raise RuntimeError("PyYAML required untuk config save/restore") from exc
        root = {"format_version": 2, "source": "tools/vesc_tool.py", "captured_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
        for name, right in (("left", False), ("right", True)):
            mc, app = self.link.get_mcconf_raw(right), self.link.get_appconf_raw(right)
            d = self.link.diag(right)
            root[name] = {
                "mc_config": {"raw_hex": mc.hex(), "length_bytes": len(mc), "sha256": hash_raw(mc), "runtime": asdict(self.link.get_mcconf_temp(right))},
                "app_config": {"raw_hex": app.hex(), "length_bytes": len(app), "sha256": hash_raw(app)},
                "tuning": asdict(self.link.get_tuning(right)), "hall_table": d.hall_table,
            }
        root["left"]["steering_calibration"] = self.link.steering_calibration()
        data = {"/**": {"ros__parameters": {"hoverboard_vesc": root}}}
        out = Path(path).expanduser(); out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
        print(f"CONFIG_SAVED {out}")

    def _restore(self, path: str, spec: str) -> None:
        try: import yaml
        except ImportError as exc: raise RuntimeError("PyYAML required untuk config save/restore") from exc
        data = yaml.safe_load(Path(path).expanduser().read_text())
        root = data["/**"]["ros__parameters"]["hoverboard_vesc"]
        with self.worker.suspended(True):
            for label, right in motors(spec):
                node = root["right" if right else "left"]
                mcnode = node["mc_config"].get("active", node["mc_config"])
                appnode = node["app_config"].get("active", node["app_config"])
                mc = bytes.fromhex(mcnode["raw_hex"])
                app = bytes.fromhex(appnode["raw_hex"])
                if mcnode.get("sha256") and hash_raw(mc) != mcnode["sha256"]: raise ValueError(f"{label} MC SHA mismatch")
                if appnode.get("sha256") and hash_raw(app) != appnode["sha256"]: raise ValueError(f"{label} APP SHA mismatch")
                self.link.set_mcconf_raw(mc, right); self.link.set_appconf_raw(app, right, store=True)
                print(f"{label} MC+APP restored exact bytes")

    def _mc_app(self, which: str, words: list[str]) -> None:
        words, spec = split_target(words, self.target); action = words[0].lower() if words else "get"
        for label, right in motors(spec):
            if which == "mc":
                raw = self.link.get_mcconf_raw(right, default=(action=="default"))
                if action == "get": print(label, "MC", asdict(self.link.get_mcconf_temp(right)), "bytes", len(raw), "sha256", hash_raw(raw))
                elif action in ("raw","default"): print(label, raw.hex())
                else: raise ValueError("mc get|raw|default [target]")
            else:
                raw = self.link.get_appconf_raw(right, default=(action=="default"))
                if action == "get": print(label, "APP bytes", len(raw), "sha256", hash_raw(raw))
                elif action in ("raw","default"): print(label, raw.hex())
                else: raise ValueError("app get|raw|default [target]")

    def _detect(self, words: list[str]) -> None:
        if not words: raise ValueError("detect hall|encoder|rl|flux|all ...")
        kind = words.pop(0).lower(); store = False
        if words and words[-1].lower() == "store": store = True; words.pop()
        words, spec = split_target(words, self.target)
        with self.worker.suspended(True):
            if kind == "hall":
                amps = float(words[0]) if words else 1.0
                for label, right in motors(spec):
                    ok, table = self.link.detect_hall(amps, right); print(label, "HALL", ok, table)
                    if ok and store:
                        raw = bytearray(self.link.get_mcconf_raw(right)); raw[223:231] = bytes(table)
                        self.link.set_mcconf_raw(bytes(raw), right); print(label, "Hall table STORED via MC Config")
            elif kind == "encoder":
                amps = float(words[0]) if words else 1.0
                for label, right in motors(spec): print(label, "ENCODER", self.link.detect_encoder(amps, right))
            elif kind == "rl":
                for label, right in motors(spec): print(label, self.link.measure_r_l(right))
            elif kind == "flux":
                if len(words) != 5: raise ValueError("detect flux I ERPM_S DUTY R_OHM L_H [target]")
                a, erpm_s, duty, r, l = map(float, words)
                for label, right in motors(spec): print(label, "flux_Wb", self.link.measure_flux_openloop(a, erpm_s, duty, r, l, right))
            elif kind == "all":
                print("DETECT_ALL result", self.link.detect_all_foc())
            else: raise ValueError("detect hall|encoder|rl|flux|all")

    @staticmethod
    def _phase_diff_deg(a: int, b: int) -> float:
        d=(int(a)-int(b)) & 0xffff
        if d>=0x8000: d-=0x10000
        return d*360.0/65536.0

    def _hall_phase(self, words: list[str]) -> None:
        words,spec=split_target(words,self.target)
        ms=motors(spec)
        if len(ms)!=1: raise ValueError("hall-phase harus satu motor: hall-phase ERPM [SECONDS] left|right")
        if not words: raise ValueError("hall-phase ERPM [SECONDS] left|right")
        erpm=int(round(float(words[0]))); seconds=float(words[1]) if len(words)>1 else 2.5
        if abs(erpm)<300 or abs(erpm)>1200: raise ValueError("hall-phase safety: |ERPM| 300..1200")
        label,right=ms[0]
        before=self.link.diag(right); seq0=before.hall_sequence_rejects or 0; per0=before.hall_period_rejects or 0
        trip0=before.current_trips; pos0=before.position; seen=set(); max_use=0.0; checked=0
        rpm_packet=enc_setpoint("rpm",erpm)
        with self.worker.suspended(True):
            try:
                end=time.monotonic()+max(2.0,seconds); next_tx=0.0
                while time.monotonic()<end:
                    now=time.monotonic()
                    if now>=next_tx:
                        self.link.send_no_reply(rpm_packet,right); next_tx=now+0.025
                    d=self.link.diag(right)
                    if d.fault or d.current_trips!=trip0: raise RuntimeError(f"fault/trip fault={d.fault} trips={trip0}->{d.current_trips}")
                    if d.phase_raw is None or d.phase_hall_raw is None: raise RuntimeError("firmware Hall phase diagnostic extension unavailable")
                    if d.hall in range(1,7):
                        seen.add(d.hall); checked+=1
                        max_use=max(max_use,abs(self._phase_diff_deg(d.phase_raw,d.phase_hall_raw)))
                    time.sleep(0.025)
            finally:
                for _ in range(3): self.link.send_no_reply(enc_setpoint("current",0.0),right); time.sleep(0.015)
        after=self.link.diag(right); seq1=after.hall_sequence_rejects or 0; per1=after.hall_period_rejects or 0
        edges=abs(after.position-pos0)
        print(f"HALL_PHASE {label} erpm={erpm:+d} samples={checked} states={sorted(seen)} edges={edges} phase_use_max={max_use:.2f}deg seq_reject={seq0}->{seq1} period_reject={per0}->{per1}")
        if checked<3 or edges<6 or seq1!=seq0 or max_use>3.0 or after.fault:
            raise RuntimeError("HALL_PHASE_FAIL")
        print("HALL_PHASE_PASS")

    def execute(self, line: str) -> bool:
        a = shlex.split(line)
        if not a: return True
        cmd = a.pop(0).lower()
        if cmd in ("q","quit","exit"): return False
        if cmd == "help":
            if a and a[0].lower() == "firmware": self._terminal(["help", "@left"])
            else: print(HELP)
        elif cmd in ("target","use"):
            if len(a)!=1 or a[0].lower() not in TARGETS: raise ValueError("target left|right|both")
            self.target = TARGETS[a[0].lower()]; print("target =", self.target)
        elif cmd == "scan": print("virtual CAN IDs:", self.link.ping_can())
        elif cmd == "fw":
            a, spec = split_target(a, self.target)
            for label,right in motors(spec): print(label, parse_fw(self.link.fw(right)))
        elif cmd == "info":
            print("CAN", self.link.ping_can())
            for label,right in motors("both"): print(label, parse_fw(self.link.fw(right))); print(" ", self.link.values(right).short())
        elif cmd == "telemetry":
            op = a[0].lower() if a else "once"
            if op == "on": self.worker.set_telemetry(True, float(a[1]) if len(a)>1 else None); print("telemetry ON")
            elif op == "off": self.worker.set_telemetry(False); print("telemetry OFF")
            elif op == "once": self.worker.telemetry_once(TARGETS.get(a[1].lower(), self.target) if len(a)>1 else self.target)
            else: raise ValueError("telemetry on [Hz]|off|once [target]")
        elif cmd in ("values","diag","setup"):
            a, spec = split_target(a, self.target)
            for label,right in motors(spec):
                obj = self.link.values(right) if cmd=="values" else self.link.diag(right) if cmd=="diag" else self.link.setup_values(right)
                print(label, obj.short() if hasattr(obj,"short") else obj)
        elif cmd == "set":
            if not a: raise ValueError("set current|rpm|duty|pos|brake|handbrake ...")
            mode = a.pop(0).lower()
            if mode == "steer":
                if len(a)!=1: raise ValueError("set steer DEG")
                self.link.set_steering_deg(float(a[0])); print("LEFT steer target", a[0], "deg")
            elif mode in MOTION_MODES: self._set_motion(mode, a)
            else: raise ValueError("set mode tidak dikenal")
        elif cmd in MOTION_MODES: self._set_motion(cmd, a)
        elif cmd == "stop":
            a, spec = split_target(a, self.target); self.worker.stop(spec); print("STOP", spec)
        elif cmd == "alive":
            a, spec = split_target(a, self.target)
            for _,right in motors(spec): self.link.alive(right)
        elif cmd in ("term","terminal"): self._terminal(a)
        elif cmd == "steering":
            op = a[0].lower() if a else "status"
            if op == "status": print(self.link.steering_calibration())
            elif op in ("zero","center"): print(self.link.terminal("steering zero", False).rstrip())
            elif op == "home": print(self.link.home_steering())
            elif op == "reset": print(self.link.terminal("steering reset", False).rstrip())
            elif op == "invert" and len(a)==2: print(self.link.terminal(f"steering invert {int(a[1])}", False).rstrip())
            elif op == "set" and len(a)==2: self.link.set_steering_deg(float(a[1])); print("LEFT steer", a[1])
            else: raise ValueError("steering status|zero|center|home|reset|invert 0|1|set DEG")
        elif cmd == "encoder": print(self.link.encoder_debug())
        elif cmd == "posstate":
            a, spec = split_target(a, self.target)
            for label,right in motors(spec): print(label, self.link.position_state(right))
        elif cmd in ("poscount","position-count"):
            op=a.pop(0).lower() if a else "status"
            a,spec=split_target(a,self.target)
            if op in ("status","get"):
                for label,right in motors(spec): print(label,self.link.position_state(right))
            elif op=="set" and len(a)==1:
                for label,right in motors(spec): print(label,self.link.set_position_counts(int(a[0]),right))
            elif op=="limits" and len(a)==2:
                for label,right in motors(spec): print(label,self.link.set_position_limits(int(a[0]),int(a[1]),right))
            elif op=="reset" and not a:
                for label,right in motors(spec): print(label,self.link.reset_position(right))
            else: raise ValueError("poscount status|set COUNT|limits MIN MAX|reset [target]")
        elif cmd == "sensor":
            if not a or a[0].lower() not in ("encoder","hall"): raise ValueError("sensor encoder|hall [target] [store]")
            sensor=a.pop(0); store=False
            if a and a[-1].lower()=="store": store=True; a.pop()
            a,spec=split_target(a,self.target)
            with self.worker.suspended(True):
                for label,right in motors(spec):
                    print(label, self.link.terminal(f"set sensor {sensor}", right).rstrip())
                    if store: print(label, self.link.terminal("save mcconf", right).rstrip())
        elif cmd == "tuning":
            if not a or a[0].lower()=="get":
                rest=a[1:] if a else []; rest,spec=split_target(rest,self.target); self._print_tuning(spec)
            elif a[0].lower()=="set": self._set_tuning(a[1:])
            else: raise ValueError("tuning get|set ...")
        elif cmd == "detect": self._detect(a)
        elif cmd == "hall-phase": self._hall_phase(a)
        elif cmd in ("zero","center"): print(self.link.terminal("steering zero",False).rstrip())
        elif cmd == "all":
            for sub in ("info","diag both","tuning get both","steering status","platform","comms"): self.execute(sub)
        elif cmd in ("mc","app"): self._mc_app(cmd,a)
        elif cmd == "config":
            if len(a)<2: raise ValueError("config save FILE | config restore FILE [target]")
            op,path=a[0].lower(),a[1]
            if op=="save":
                with self.worker.suspended(False): self._snapshot(path)
            elif op=="restore":
                _,spec=split_target(a[2:],self.target); self._restore(path,spec)
            else: raise ValueError("config save|restore")
        elif cmd == "battery":
            if not a: raise ValueError("battery get [target] | set START END [target] [store]")
            op=a.pop(0).lower(); store=False
            if a and a[-1].lower()=="store": store=True; a.pop()
            a,spec=split_target(a,self.target)
            if op=="get":
                for label,right in motors(spec): print(label, self.link.get_battery_cut(right))
            elif op=="set" and len(a)==2:
                for label,right in motors(spec): self.link.set_battery_cut(float(a[0]),float(a[1]),store=store,right=right); print(label,"battery cut set")
            else: raise ValueError("battery get | battery set START END [target] [store]")
        elif cmd == "odometer":
            if not a: raise ValueError("odometer METERS [target]")
            meters_n=int(a.pop(0)); a,spec=split_target(a,self.target)
            for label,right in motors(spec): self.link.set_odometer(meters_n,right); print(label,"odometer",meters_n)
        elif cmd == "health":
            print(self.link.platform_health())
        elif cmd == "platform":
            print(self.link.platform_info())
        elif cmd == "comms":
            print(self.link.comms_health())
        elif cmd == "isr":
            print(self.link.isr_profile(reset=bool(a and a[0].lower() == "reset")))
        elif cmd == "trace":
            op = a.pop(0).lower() if a else "meta"
            if op == "meta":
                print(self.link.trace_meta())
            elif op == "clear":
                self.link.trace_clear()
                print("TRACE_CLEARED")
            elif op == "freeze":
                self.link.trace_freeze()
                print("TRACE_FROZEN")
            elif op == "download":
                rows = self.link.download_trace()
                if a:
                    out = Path(a[0]).expanduser()
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
                    print(f"TRACE_SAVED {out} samples={len(rows)}")
                else:
                    print(json.dumps(rows, indent=2))
            else:
                raise ValueError("trace meta|clear|freeze|download [FILE]")
        elif cmd in ("current-step", "current_step"):
            a, spec = split_target(a, self.target)
            selected = motors(spec)
            if len(selected) != 1:
                raise ValueError("current-step harus satu motor: current-step PRE STEP [q|d] left|right")
            if len(a) not in (2, 3):
                raise ValueError("current-step PRE_A STEP_A [q|d] left|right")
            pre_a, step_a = float(a[0]), float(a[1])
            axis = a[2].lower() if len(a) == 3 else "q"
            label, right = selected[0]
            with self.worker.suspended(True):
                rows = self.link.run_current_step(pre_a, step_a, right=right, axis=axis)
            t0 = next(i for i, row in enumerate(rows) if row["event_bits"] & (1 << 3))
            print(f"CURRENT_STEP {label} axis={axis} samples={len(rows)} t0={t0}")
        elif cmd == "adc":
            a,spec=split_target(a,self.target)
            for label,right in motors(spec): print(label,self.link.adc_validity(right))
        elif cmd == "shutdown":
            a,spec=split_target(a,self.target); self.worker.stop(spec)
            for label,right in motors(spec): self.link.shutdown_safe(right); print(label,"shutdown/release sent")
        elif cmd == "reboot":
            a, spec = split_target(a, self.target)
            self.worker.stop(spec)
            order = motors(spec)
            order.sort(key=lambda x: not x[1])  # RIGHT first, local LEFT last
            for label, right in order:
                self.link.reboot(right)
                print(label, "reboot sent")
                time.sleep(0.1)
            if any(not right for _, right in order):
                return False
        elif cmd == "bootloader":
            if a:
                raise ValueError("bootloader tidak menerima argumen; hanya local LEFT/F103")
            self.worker.stop("both")
            self.link.boot_handoff()
            print("BOOTLOADER_HANDOFF_SENT")
            return False
        else: raise ValueError("command tidak dikenal; ketik help")
        return True


def selftest() -> int:
    payload = b"\x04test-vesc-tool"
    raw = frame(payload); dec = PacketDecoder(); got=[]
    for chunk in (raw[:1],raw[1:4],raw[4:]): got += dec.feed(chunk)
    assert got == [payload]
    assert crc16(payload) == ((raw[-3]<<8)|raw[-2])
    assert enc_setpoint("current",1.25)[1:] == struct.pack(">i",1250)
    assert enc_setpoint("duty",0.5)[1:] == struct.pack(">i",50000)
    assert enc_setpoint("pos",180)[1:] == struct.pack(">i",180000000)
    print("VESC_TOOL_SELFTEST_PASS framing=1 crc=1 scaling=1")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap=argparse.ArgumentParser(description="single VESC Tool-like CLI for LEFT encoder + RIGHT Hall F103")
    ap.add_argument("port",nargs="?",default="/dev/ttyUSB0", help="serial port (default: /dev/ttyUSB0)")
    ap.add_argument("--baud",type=int,default=DEFAULT_BAUD)
    ap.add_argument("--command-hz",type=float,default=50.0)
    ap.add_argument("--telemetry-hz",type=float,default=5.0)
    ap.add_argument("--no-telemetry",action="store_true")
    ap.add_argument("--exec",dest="one_command",help="jalankan satu command lalu keluar")
    ap.add_argument("--selftest",action="store_true")
    args=ap.parse_args(argv)
    if args.selftest: return selftest()
    link=VescDual(args.port,args.baud,timeout=0.35)
    worker=LiveWorker(link,args.command_hz,0.0 if args.no_telemetry or args.one_command else args.telemetry_hz)
    console=Console(link,worker)
    try:
        print("LEFT :",parse_fw(link.fw(False)))
        print("CAN  :",link.ping_can())
        try: print("RIGHT:",parse_fw(link.fw(True)))
        except Exception as exc: print("RIGHT: unavailable:",exc)
        if args.one_command:
            return 0 if console.execute(args.one_command) else 0
        print("Interactive VESC CLI ready. Telemetry scrolls without destroying typed input. Type 'help'.")
        command_words = [
            "help","target","use","fw","scan","info","telemetry","values","diag","setup",
            "set","current","current_rel","rpm","duty","pos","brake","handbrake","stop","alive",
            "term","terminal","steering","zero","center","encoder","posstate","poscount","sensor",
            "tuning","detect","hall-phase","mc","app","config","battery","odometer","platform",
            "health","comms","isr","trace","current-step","adc","shutdown","reboot","bootloader","all","left","right","both","store","quit"
        ]
        session=PromptSession(
            history=FileHistory(str(Path.home()/".vesc_tool_history")),
            auto_suggest=AutoSuggestFromHistory(),
            completer=WordCompleter(command_words, ignore_case=True),
        )
        with patch_stdout(raw=True):
            while True:
                try: line=session.prompt(lambda: f"vesc[{console.target}]> ").strip()
                except (EOFError,KeyboardInterrupt): break
                if not line: continue
                try:
                    if not console.execute(line): break
                except Exception as exc: print(f"[ERR] {type(exc).__name__}: {exc}")
        return 0
    finally:
        worker.close(); link.close()


if __name__ == "__main__":
    raise SystemExit(main())
