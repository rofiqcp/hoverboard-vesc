#!/usr/bin/env python3
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
main=(R/'Src/main.c').read_text(); vp=(R/'Src/vesc/vesc_protocol.c').read_text()
boot=(R/'Src/bootloader/main.c').read_text(); upd=(R/'Src/vesc/flash_update_f103.c').read_text()
layout=(R/'Src/vesc/f103_boot_layout.h').read_text(); host=(R/'tools/pio_vesc_upload.py').read_text()
for token in ('F103_STAGE_PROBATION_GATE_BASE','F103_STAGE_PROBATION_TIMEOUT_BASE',
              'F103_STAGE_PROBATION_CONFIRM_OK','F103_STAGE_PROBATION_VECTOR_FAIL','F103_STAGE_PROBATION_CRC_FAIL'):
    assert token in layout,token
# TEST confirmation is autonomous: USB host traffic is evidence, never a prerequisite.
assert 'const bool protocol_seen=' in main
assert 'if (elapsed_ok && realtime_ok && main_ok && uart_ok)' in main
assert 'protocol_seen && realtime_ok' not in main and 'protocol_ok &&' not in main
assert 'F103_STAGE_PROBATION_GATE_BASE | gate_bits' in main and 'F103_STAGE_PROBATION_TIMEOUT_BASE | gate_bits' in main
assert 'F103_STAGE_PROBATION_CONFIRM_OK' in upd and 'F103_STAGE_PROBATION_VECTOR_FAIL' in upd and 'F103_STAGE_PROBATION_CRC_FAIL' in upd
# Only magic/version/ACK custom handoff may enter bootloader from a running APP.
jump=vp[vp.index('case COMM_JUMP_TO_BOOTLOADER:'):vp.index('case COMM_GET_VALUES:',vp.index('case COMM_JUMP_TO_BOOTLOADER:'))]
assert 'f103_fw_reset_to_bootloader' not in jump and 's_boot_handoff_pending=1u' not in jump
custom=vp[vp.index('if (op == HB_CUSTOM_BOOT_HANDOFF)'):vp.index('if (op == HB_CUSTOM_GET_FW_UPDATE_STATE)')]
assert 's_boot_handoff_pending=1u' in custom and 'uart_send_payload' in custom
prob=vp[vp.index('static bool probation_packet_allowed'):vp.index('static void process_top_packet')]
assert 'id==COMM_JUMP_TO_BOOTLOADER' not in prob
# Recovery exposes its black-box and TEST failures persist as RECOVERY.
assert 'F103_RESET_REASON_ADDR' in boot and 'F103_RESET_STAGE_ADDR' in boot
assert 'write_meta(F103_UPDATE_STATE_RECOVERY' in boot
assert "'reset_reason':" in host and "'reset_stage':" in host
assert 'link.reconnect_transport()' in host[host.index('def _wait_confirmed'):host.index('def _wait_application')]
print('BOOT_STATE_MACHINE_HARDENING_PASS autonomous_confirm=1 raw_jump_blocked=1 custom_handoff=1 recovery_blackbox=1 reconnect_confirm=1')
