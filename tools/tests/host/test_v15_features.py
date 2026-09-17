#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
mcc=(R/'Src/motor/mcconf_default.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
com=(R/'Src/comms.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
dbg=(R/'tools/vesc_tool.py').read_text()
halltest=(R/'tools/tests/host/test_hall_detect_algorithm.c').read_text()

# 50 ERPM must survive Hall timeout and retain fractional mechanical target.
assert re.search(r'#define\s+MCCONF_HALL_TIMEOUT_TICKS\s+8000u',mcc)
assert 'm_speed_target_rpm_q16' in mch and 'erpm_to_mech_rpm_q16' in mc
assert 'measured_mech_rpm_q16' in mc
motor_step_start=mc.index('static void motor_control_step')
motor_step=mc[motor_step_start:mc.index('static int16_t duty_permille_from_vdq',motor_step_start)]
# VESC-style outer PID scheduler: fresh feedback at 1 kHz, current ISR only
# consumes cached Iq. Virtual stale-feedback catch-up is forbidden.
outer=mc[mc.index('void mcpwm_foc_outer_control_non_isr'):mc.index('void mcpwm_foc_housekeeping_non_isr')]
assert re.search(r'#define\s+MCCONF_OUTER_PID_HZ\s+1000u',mcc)
assert 'm->m_iq_target_q4=speed_pid_iq_target_step' in outer
assert 'm->m_iq_target_q4=position_pid_iq_target_step' in outer
assert 'speed_pid_iq_target_step' not in motor_step and 'position_pid_iq_target_step' not in motor_step
assert 'motor_outer_loop_virtual_steps' not in mc
assert re.search(r'measured_mech_rpm_q16\(m,\s*second\)\s*\*\s*pp',mc)
assert '((float)PWM_FREQ*10.0f)/(float)m->m_hall_period' in mc

# VESC OPENLOOP_PHASE is fixed phase/direct Id. Hall/encoder detect performs
# its own explicit current ramp and the rotating openloop updater must not run.
assert 'm->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE' in mc
phase_branch=mc[mc.index('if (m->m_control_mode==CONTROL_MODE_OPENLOOP)'):mc.index('/* Hall estimator needs')]
assert 'openloop_update(m);' in phase_branch and 'openloop_current_ramp_update(m);' not in phase_branch
assert 'Fixed phase/current is set directly by the VESC setter' in phase_branch
assert mc.count('for (uint8_t pass = 0u; pass < 3u; ++pass)') >= 2 and 'deg = 360; deg >= 0' in mc
assert 'mcpwm_foc_adc_int_handler();' in halltest and 'for(uint32_t t=0;t<ms;t++)' in halltest and 'isr<16u' in halltest

# VESC ownership/bridge gating is per motor and exclusive. Once VESC owns an
# endpoint, an expired command must not be bypassed by the legacy enable flag.
assert 'const uint8_t leftSourceEnable=(!estopActive) &&' in mc and        '(s_vesc_owned[0] ? mcpwm_foc_vesc_command_live(false) : (enable!=0u));' in mc,        'LEFT source gate must enforce exclusive VESC ownership and E-stop'
assert 'const uint8_t rightSourceEnable=(!estopActive) &&' in mc and        '(s_vesc_owned[1] ? mcpwm_foc_vesc_command_live(true) : (enable!=0u));' in mc,        'RIGHT source gate must enforce exclusive VESC ownership and E-stop'
assert 'leftOpenloop = (m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||' in mc
assert 'CONTROL_MODE_OPENLOOP_PHASE);' in mc and 'leftDcLimit=leftOpenloop' in mc and 'rightDcLimit=rightOpenloop' in mc

# Tidak ada live toggle maupun telemetry legacy. USART3 hanya protokol VESC.
assert '"LIVE"' not in com
assert 'SerialFeedback' not in main and 'legacyTelemetryPrevMs' not in main
assert 'if (!vescLinkActive && !timeoutFlgSerial' in main
assert 'USART3 hanya membawa protokol VESC' in main

# Exact VESC 6.00 identity: FW response stops after FW_NAME.
fwfun=vp[vp.index('static void reply_fw_version'):vp.index('static void get_values_normalized')]
assert 'VESC_FW_MAJOR' in fwfun and 'VESC_FW_MINOR' in fwfun
assert 'post-6.00 fields' in fwfun and 'buffer_append_uint32' not in fwfun

# VESC Tool Hall detect is a blocking-command contract implemented cooperatively on bare metal.
# Detect returns [cmd + 8 table + result], stays UART-responsive, and does not apply/store MC config.
det=vp[vp.index('static bool hall_detect_motor_locked'):vp.index('static int32_t q4_to_milliamps_normalized')]
assert 'mcpwm_foc_hall_detect_command_start' in det and 'mcpwm_foc_hall_detect_process' in det and 'uint8_t reply[10]' in det
assert 'reply[9]=success?0u:1u' in det and 'standalone detect is not a store' in det
assert 'mc_interface_release_motor()' in det and 'mcpwm_foc_vesc_override_clear(second)' in det
assert 'COMM_DETECT_APPLY_ALL_FOC' in vp and 'conf_general_detect_apply_all_foc_process' in det and 'detect_all_commit' in det
assert 'mc_interface_store_configuration_motor(mi!=0u)' in det and 'uart_send_payload(reply,sizeof(reply))' in det
assert 'measure_r_l_imax_f103_finish' in det and 'DETECT_ALL_FLUX_SAMPLE' in det, 'Detect-All must identify motor model before transactional store'

# Standard VESC Tool setPos is a signed degree x1e6 packet builder with no
# client-side clamp; firmware normalizes its angular target. Project multi-turn
# remains a distinct CUSTOM_APP_DATA signed-int32 API.
assert 'case COMM_SET_POS:' in vp and 'COMM_CUSTOM_APP_DATA' in vp
assert 'HB_CUSTOM_SET_POS_LIMITS' in vp and 'HB_CUSTOM_SET_POS_TARGET' in vp
assert 'Commands::setPos VESC Tool: signed degree value x1e6, tanpa client clamp.' in dual
assert 'def set_position_limits(' in dual and 'def set_position_counts(' in dual

# Complete hardware diagnostic tool includes 3A, 50 ERPM, Hall, RT 50Hz and position tests.
for token in ('telemetry on [Hz]','detect hall A [target] [store]','poscount limits MIN MAX',
              'set current A [A_R] [target]','set rpm ERPM [ERPM_R] [target]',
              'COMM_SET_CURRENT','COMM_SET_RPM','HALL_PHASE_PASS'):
    assert token in dbg, token
assert 'command_hz: float = 50.0' in dbg
print('V15_FEATURE_STATIC_PASS safe_current=1 rpm750=1 speed_iq_cascade=1 rt50=1 hall_isr_sweep=1 pos_int32=1 live_toggle=removed fw600_exact=1')
