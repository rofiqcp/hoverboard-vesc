#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
serialh=(R/'Src/vesc/mcconf_serial.h').read_text()
serialc=(R/'Src/vesc/mcconf_serial.c').read_text()
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mci=(R/'Src/motor/mc_interface.c').read_text()
eeh=(R/'Src/eeprom.h').read_text()
ld=(R/'STM32F103RCTx_APP.ld').read_text(); bld=(R/'STM32F103RCTx_BOOTLOADER.ld').read_text()
# Firmware/protocol identity must be VESC 6.00.
assert re.search(r'#define\s+VESC_FW_MAJOR\s+6u',vp)
assert re.search(r'#define\s+VESC_FW_MINOR\s+0u',vp)
assert '#define MCCONF_SIGNATURE 776184161u' in serialh
assert '#define APPCONF_SIGNATURE 486554156u' in serialh
# Standard command mapping and dual virtual-CAN semantics.
for token in ('case COMM_SET_DUTY:','case COMM_SET_CURRENT:','case COMM_SET_RPM:','case COMM_SET_POS:',
              'case COMM_GET_VALUES:','case COMM_GET_MCCONF:','case COMM_GET_MCCONF_DEFAULT:',
              'case COMM_SET_MCCONF:','case COMM_DETECT_HALL_FOC:'):
    assert token in vp, token
assert 'COMM_FORWARD_CAN' in vp and 'VESC_SECOND_MOTOR_ID        2u' in vp
assert 'right_sign' not in vp
assert 'mc_interface_set_duty(duty)' in vp
assert 'mc_interface_set_current(current)' in vp
assert 'mc_interface_set_pid_speed(rpm)' in vp
assert 'mc_interface_set_pid_pos(pos)' in vp
# Standard telemetry fields and scaling.
for token in ('v->current_motor, 1e2f','v->current_in, 1e2f','v->id, 1e2f','v->iq, 1e2f',
              'v->duty_now, 1e3f','v->rpm, 1e0f','v->position, 1e6f','v->vd, 1e3f','v->vq, 1e3f'):
    assert token in vp, token
# Hall table must be a live control input and detection must update it.
assert 'm->m_conf.foc_hall_table' in mc and 'hall_table_angle' in mc
assert 'bool mcpwm_foc_hall_detect' in mc
assert 'mcpwm_foc_set_openloop_phase(current' in mc
assert 'fails == 2u' in mc and 'mcpwm_foc_hall_detect_angle200' in mc
assert 'detect_all_prepare_hall' in vp and 'foc_hall_table' in vp
assert 'mc_interface_store_configuration_motor(second)' in vp
# Dual-motor forwarding selects motor thread only; no virtual-right coordinate rewrite.
assert 'Right power stage is physically mirrored' not in vp
# Config wire format is the 6.00 adapter, not 7.x additions.
assert 'appconf6_append_balance_placeholder' in serialc
assert 'appconf6_skip_balance_placeholder' in serialc
assert 'coast_brake_level' not in serialc and 'coast_brake_ramp_time' not in serialc
# Independent left/right persistence with Hall table.
assert 'EE_L_HALL0 = 3, EE_R_HALL0 = 11' in mci
assert 'EE_L_KPQ = 19' in mci and 'EE_R_KPQ = 29' in mci
assert 'EE_CFG_SIGNATURE_VALUE 0x6022u' in mci and 'EE_CFG_SIGNATURE_V35   0x6021u' in mci and 'EE_CFG_SIGNATURE_V34   0x6020u' in mci and 'EE_CFG_SIGNATURE_V33   0x601Fu' in mci and 'EE_CFG_SIGNATURE_V32   0x601Eu' in mci and 'EE_CFG_SIGNATURE_V31   0x601Du' in mci and 'EE_CFG_SIGNATURE_V30   0x601Cu' in mci and 'EE_CFG_SIGNATURE_V29   0x601Bu' in mci and 'EE_CFG_SIGNATURE_V28   0x601Au' in mci and 'EE_CFG_SIGNATURE_V27   0x6019u' in mci and 'EE_CFG_SIGNATURE_V26   0x6018u' in mci and 'EE_CFG_SIGNATURE_V25   0x6017u' in mci and 'EE_CFG_SIGNATURE_V24   0x6016u' in mci and 'EE_CFG_SIGNATURE_V23   0x6015u' in mci and 'EE_CFG_SIGNATURE_V22   0x6014u' in mci and 'EE_CFG_SIGNATURE_V21   0x6013u' in mci and 'EE_CFG_SIGNATURE_V20   0x6012u' in mci and 'EE_CFG_SIGNATURE_V19   0x6011u' in mci and 'EE_CFG_SIGNATURE_V18   0x6010u' in mci and 'EE_CFG_SIGNATURE_V17   0x600Fu' in mci and 'EE_CFG_SIGNATURE_V16   0x600Eu' in mci
assert 'mc_interface_store_configuration_motor(bool second)' in mci
assert 'mc_interface_load_configuration_motor(bool second)' in mci
# EEPROM must never overlap executable flash.
assert '0x0803F000u' in eeh and '0x0803F800u' in eeh and '0x0803FC00u' not in eeh
assert 'FLASH_PAGE_SIZE != 0x800U' in eeh
assert re.search(r'#define\s+NB_OF_VAR\s+270u',eeh)
util=(R/'Src/util.c').read_text(); addrs=[int(x) for x in re.search(r'VirtAddVarTab\[NB_OF_VAR\]\s*=\s*\{([^}]*)\}',util,re.S).group(1).split(',')]; assert addrs==list(range(1000,1270))
assert re.search(r'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x8002800,\s*LENGTH\s*=\s*240K',ld)
assert re.search(r'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x8000000,\s*LENGTH\s*=\s*10K',bld)
# Stock SET MCCONF ACK and default/read separation.
assert 'uint8_t ack = COMM_SET_MCCONF' in vp
assert 'if (id == COMM_GET_MCCONF_DEFAULT)' in vp and 'mcpwm_foc_get_default_configuration(c, second)' in vp
print('V14_FEATURE_STATIC_PASS fw=6.00 mcconf=6.00 hall_live=1 hall_eeprom=dual telemetry=vesc eeprom_reserved=1 can2=1')
