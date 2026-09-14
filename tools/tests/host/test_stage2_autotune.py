#!/usr/bin/env python3
from pathlib import Path
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
mc=(R/'Src/motor/mcpwm_foc.c').read_text();mh=(R/'Src/motor/mcpwm_foc.h').read_text();vp=(R/'Src/vesc/vesc_protocol.c').read_text();dual=(R/'tools/vesc_dual.py').read_text();stage=(R/'tools/autotune_stage2.py').read_text()
assert 'MCPWM_FOC_RELAY_SPEED = 1u' in mh and 'MCPWM_FOC_RELAY_POSITION = 2u' in mh
outer=mc[mc.index('void mcpwm_foc_outer_control_non_isr'):mc.index('void mcpwm_foc_housekeeping_non_isr')]
assert 'relay_process(now_ms);' in outer and outer.index('relay_process(now_ms);') < outer.index('mcpwm_foc_motor_t *motors[2]')
relay=mc[mc.index('static int32_t relay_speed_erpm_now'):mc.index('void mcpwm_foc_outer_control_non_isr')]
assert 'measured_mech_rpm_q16' in relay
assert 'relay_steering_mdeg_now' in relay and 'relay_steering_iq_sign' in relay
assert 'relay_hall_position_active' in relay and 'relay_hall_position_mdeg_now' in relay and 'relay_angle_delta_mdeg' in relay
assert 'if(relay_hall_position_active(m))' in relay and 'target=relay_hall_position_mdeg_now(m,second);' in relay
assert 'if(second || !m->m_steering_calibrated || !m->m_steering_homed || !m->m_encoder_synced)return false;' in relay
assert 'required_crossings<6u' in relay and 'relay_current_ma>3000u' in relay and 'timeout_ms>20000u' in relay
assert 'mcpwm_foc_release_motor(second)' in relay[relay.index('static void relay_finish'):relay.index('static void relay_apply_current')]
for token in ('HB_CUSTOM_START_RELAY_AUTOTUNE','HB_CUSTOM_GET_RELAY_AUTOTUNE','HB_CUSTOM_ABORT_RELAY_AUTOTUNE'): assert token in vp
for token in ('HB_START_RELAY_AUTOTUNE = 31','HB_GET_RELAY_AUTOTUNE = 32','HB_ABORT_RELAY_AUTOTUNE = 33','def start_relay_autotune','def relay_autotune_status','def abort_relay_autotune','def steering_calibration'): assert token in dual
assert "ku_i=4.0*d/(math.pi*amp)" in stage and "ku=20.0*ku_i/max(current_limit_a,0.1)" in stage
assert "kp=ku/3.2; ti=2.2*pu" in stage and "kp=ku/2.2; ti=2.2*pu; td=pu/6.3" in stage
assert "speed_sides=[False,True] if left_role=='hall' else [True]" in stage
assert "for right in (False,True):" in stage and "role='hall' if right else left_role" in stage
assert "policy':'Hall=SPEED+POSITION; LEFT Encoder=POSITION'" in stage
assert 'sample_hall_position' in stage and 'sample_encoder_position' in stage and 'combined_role_gate' in stage
assert "if not stage1.get('pass')" in stage and "--store-best" in stage
print('STAGE2_AUTOTUNE_STATIC_PASS sensor_aware=1 left_hall_speed_pos=1 right_hall_speed_pos=1 left_encoder_span_pos=1')
