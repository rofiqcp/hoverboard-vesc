#!/usr/bin/env python3
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
wd=(R/'Src/platform_watchdog.c').read_text()
wh=(R/'Src/platform_watchdog.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()

# STM32F1 IWDG must follow ST HAL ordering. HAL_IWDG_Init starts IWDG before
# PR/RLR update and waits for PVU/RVU to clear; the old pre-start wait deadlocked.
assert 'HAL_IWDG_Init(&hiwdg)' in wd
assert 'IWDG_PRESCALER_256' in wd and 'PLATFORM_IWDG_RELOAD' in wd
assert 'IWDG->KR=0x5555u' not in wd and 'IWDG->KR=0xCCCCu' not in wd
assert 's_init_fail_stage' in wd and 'out->iwdg_sr=IWDG->SR' in wd
assert all(x in wh for x in ('init_failed','init_fail_stage','iwdg_sr','iwdg_pr','iwdg_rlr'))

# Probation must require a genuinely active/feed-progressing watchdog.
assert 'const bool realtime_ok=w.enabled && w.feed_count>fw_test_wd_base.feed_count;' in main
assert 'platform_watchdog_maintenance_kick();' in main

# Runtime diagnostics must expose whether init really succeeded on hardware.
for token in ('w.init_failed','w.init_fail_stage','w.iwdg_sr','w.iwdg_pr','w.iwdg_rlr'):
    assert token in vp, token

# Python direct-UART enumeration must not shadow the module-level `serial` name.
assert 'from serial.tools import list_ports' in dual
assert 'import serial.tools.list_ports' not in dual
assert 'for q in list_ports.comports()' in dual

print('WATCHDOG_RUNTIME_HARDENING_PASS hal_sequence=1 feed_gate=1 hw_diag=1 uart_autodiscovery=1')
