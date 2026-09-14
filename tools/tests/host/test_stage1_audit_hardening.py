#!/usr/bin/env python3
from pathlib import Path
import re
from _test_utils import read_source
R=Path(__file__).resolve().parents[3]


dual=read_source(R, 'tools/vesc_dual.py')
cli=read_source(R, 'tools/vesc_tool.py')
assert 'DEFAULT_BAUD = 115200' in dual
assert 'EXPECTED_LOCAL_TARGETS = {"motor_left", "f103rc_bootloader"}' in dual
assert 'ser,name=_probe_f103_uart(target,baud,timeout)' in dual, 'explicit serial path must be positively probed'
assert 'unexpected VESC target' in dual
assert 'default=DEFAULT_BAUD' in cli, 'CLI baud default must live in vesc_tool.py'
assert 'class ReplWorker' not in dual, 'backend must not regain a second interactive CLI'

for rel in ('tools/vesc_debug.py','tools/hoverserial.py','tools/pio_stlink_bin_upload.py','tools/stlink_autocatch_flash.py'):
    assert not (R/rel).exists(), f'obsolete/dead tool must stay removed: {rel}'

hwdir=R/'tools/tests/hardware'
for p in hwdir.glob('*.py'):
    s=p.read_text(errors='ignore')
    bad=(r'VescDual\([^\n]*1000000',r'open_transport\([^\n]*1000000',r'baud\s*=\s*1000000',r'default\s*=\s*1000000')
    for pat in bad:
        assert not re.search(pat,s), f'stale 1-Mbaud hardware transport in {p.name}: {pat}'

moving=(
 'test_brake_handbrake_hw.py','test_duty_95_staged.py','test_duty_ramp100.py',
 'test_duty_safe.py','test_hall_detect_repeat.py','test_speed_pid_sweep_hw.py',
 'test_speed_ramp_hw.py','vesc_response_lab.py')
for name in moving:
    s=read_source(R, 'tools/tests/hardware/'+name)
    assert '--arm' in s, f'{name} can actuate without explicit arm gate'
    assert 'require_platform_compatible(True)' in s, f'{name} can actuate mismatched firmware'

button=read_source(R, 'tools/tests/hardware/test_vesc_tool_button_matrix_hw.py')
assert '--arm' in button and 'require_platform_compatible(True)' in button, 'persistent button matrix lacks explicit opt-in/build gate'

sweep=read_source(R, 'tools/tests/hardware/test_speed_pid_sweep_hw.py')
ramp=read_source(R, 'tools/tests/hardware/test_speed_ramp_hw.py')
for name,s in [('speed_pid_sweep',sweep),('speed_ramp',ramp)]:
    for forbidden in ('set_mcconf_raw(', 'COMM_SET_MCCONF', 'store=True', '/tmp/mc_speed'):
        assert forbidden not in s, f'{name} regained persistent candidate write: {forbidden}'
    assert 'set_tuning(' in s and 'store=False' in s, f'{name} must use RAM-only candidate tuning'
assert "terminal(f'set speed_ramp" in ramp, 'speed ramp candidates must use RAM-only terminal setter'
assert 'persistent_writes=False' in sweep
print('STAGE1_AUDIT_HARDENING_PASS baud=115200 explicit_probe=1 arm_gate=1 build_gate=1 speed_sweep_ram_only=1')
