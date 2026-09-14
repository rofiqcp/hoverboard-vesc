#!/usr/bin/env python3
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
cli=(R/'tools/vesc_tool.py').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
for token in ('COMM_SET_CURRENT','COMM_SET_RPM','COMM_SET_POS','COMM_SET_CURRENT_REL','COMM_SET_HANDBRAKE',
              'target left|right|both','tuning set pos','config save FILE.yaml','term COMMAND','prompt_toolkit'):
    assert token in cli, token
assert 'class ReplWorker' not in dual and 'input("vesc-dual>' not in dual
assert 'self.alive_hz = 5.0' in cli, 'idle VESC connection must keep standard 200-ms COMM_ALIVE cadence'
assert 'self.link.alive(False)' in cli and 'self.link.alive(True)' in cli, 'idle keepalive must cover LEFT and RIGHT'
assert 'if tel_on and tel_hz > 0 and now >= next_tel:' in cli, 'telemetry polling must remain independent of active setpoint state'
assert 'bottom_toolbar=worker.status_line' in cli and 'refresh_interval=0.2' in cli and 'wrap_lines=False' in cli, 'interactive telemetry must use one stable prompt_toolkit status line'
assert 'self.telemetry_once("both", emit=False)' in cli, 'background telemetry must update status without printing/scrolling'
assert 'f"L={lset}/{left.position:.2f}deg "' in cli and 'f"R={rset}/{right.rpm:.0f} "' in cli, 'compact L=set/get and R=set/get status format missing'
assert not (R/'tools/vesc_debug.py').exists()
assert not (R/'tools/hoverserial.py').exists()
print('VESC_TOOL_CLI_CONSOLIDATION_PASS')
