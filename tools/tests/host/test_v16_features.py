#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mci=(R/'Src/motor/mc_interface.c').read_text()
fm=(R/'Src/motor/foc_math.c').read_text()
assert 'second ? FOC_SENSOR_MODE_HALL : FOC_SENSOR_MODE_ENCODER' in mc and 'second ? SENSOR_PORT_MODE_HALL : SENSOR_PORT_MODE_ABI' in mc, 'Blank/default config must be LEFT ABI encoder + RIGHT Hall'
mch=(R/'Src/motor/mcpwm_foc.h').read_text()
mcc=(R/'Src/motor/mcconf_default.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
host=(R/'tools/tests/host/test_vesc_protocol_host.c').read_text()

# User-visible identity must be exact in VESC Tool for local and virtual motor 2.
assert 'second ? "motor_right" : "motor_left"' in vp
assert re.search(r'#define\s+VESC_SECOND_MOTOR_ID\s+2u',vp)

# Hall table values are sector centers. V16 must begin interpolation at the
# midpoint between old/new centers, not at the new center as V15 did.
assert 'hall_midpoint200' in mc and 'center_delta / 2' in mc
assert 'm->m_hall_pos = hall_midpoint200(previous_center, ad);' in mc
assert 'edge_phase = hall_angle200_to_phase(m->m_hall_pos)' in mc
assert 'm_phase_hall_target' in mch and 'phase_diff_u16' in mc
assert 'rate-limits corrected Hall phase' in mc
assert 'm_hall_reject_counted_state' in mch and 'm->m_hall_reject_counted_state != h' in mc
assert 'period_for_filter' in mc and 'floor_period' in mc and 'm->m_hall_period_reject_count++' in mc, 'Hall timing outlier slew-limit path missing'
motor_step_start=mc.index('static void motor_control_step')
motor_step=mc[motor_step_start:mc.index('static int16_t duty_permille_from_vdq',motor_step_start)]
outer=mc[mc.index('void mcpwm_foc_outer_control_non_isr'):mc.index('void mcpwm_foc_housekeeping_non_isr')]
assert re.search(r'#define\s+MCCONF_OUTER_PID_HZ\s+1000u',mcc)
assert 'm->m_iq_target_q4=speed_pid_iq_target_step' in outer
assert 'm->m_iq_target_q4=position_pid_iq_target_step' in outer
assert 'speed_pid_iq_target_step' not in motor_step and 'position_pid_iq_target_step' not in motor_step
assert 'm->m_iq_set_q4=m->m_iq_target_q4;' in motor_step
assert 'motor_outer_loop_virtual_steps' not in mc
assert 'm->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_iq_set_ramp_q16=0;' in mc, 'speed STOP must force VESC zero-vector reference'
assert 'speed_pid_iq_target_step' in outer and 'm->m_iq_target_q4=speed_pid_iq_target_step' in outer, 'speed PID must feed the cached Iq target through executable code, not a comment contract'
assert 'min_erpm_q16' in mc and re.search(r'if\s*\(\s*target_abs_q16\s*<\s*min_erpm_q16\s*\)',mc), 'speed release threshold must be enforced independent of source formatting'
assert 'speed PI drives Vq directly' not in mc
assert 'MCCONF_POSITION_PHASE_DEADBAND_MDEG' not in mcc and 'MCCONF_POSITION_RUN_CURRENT_MAX_MA' not in mcc and 'MCCONF_POSITION_BREAKAWAY_CURRENT_MA' not in mcc
assert 'm_position_prev_proc_phase' in mch and 'm_position_kd_proc_phase_coeff_q4' in mch
# Standard COMM_SET_POS mengikuti foc_run_pid_control_pos VESC: shortest-path
# angular PID -> Iq, lengkap dengan anti-windup dan D-on-measurement. State
# machine Hall per-sektor hanya dipakai API custom count multi-putaran.
assert 'VESC foc_run_pid_control_pos: shortest-path angular PID' in mc
assert 'position_p_term_q15' in mc and 'i_lim_q15=32768-' in mc and 'position_i_step_q16' in mc
assert 'm_position_d_filter_q15' in mch and 'm_position_d_proc_filter_q15' in mch
assert 'proc_now=position_feedback_phase_u16(m,second)' in mc and 'proc_delta=(int16_t)(proc_now-m->m_position_prev_proc_phase)' in mc
assert 'out_q15=p_q15+(m->m_position_integrator>>16)+' in mc
assert 'hall_motion_same_direction' not in mc and 'position_brake_iq_q4' not in mc, 'hidden kinetic brake helper must stay removed'
mi=(R/'Src/motor/mc_interface.c').read_text()
assert 'EE_CFG_SIGNATURE_V16' in mi and 'EE_CFG_SIGNATURE_V17' in mi and 'migrate_speed_pid' in mi

# VESC-like detector: 1s current ramp, 3 forward + 3 reverse complete 1-degree sweeps.
assert 'i < 1000u' in mc and 'HAL_Delay(1u)' in mc
assert 'pass < 3u' in mc and 'deg < 360u' in mc and 'deg = 360; deg >= 0' in mc and 'HAL_Delay(5u)' in mc
assert 'mcpwm_foc_hall_detect_angle200' in mc and 'foc_atan2_phase_u16(ys,xc)' in mc
assert 'foc_sin_cos_q15(ph,&sn,&cs)' in mc and 'int64_t best_dot' not in mc and 'int64_t best_dot' not in vp, 'Hall must use F103 Q15 circular samples with upstream atan2 finalization, not nearest-bin search'
assert 'mcpwm_foc_hall_detect_command_start' in vp and 'mcpwm_foc_hall_detect_process' in vp and 'standalone detect is not a store' in vp
assert 'mc_interface_store_configuration_motor(second)' in vp and 'case COMM_SET_MCCONF:' in vp and 'case COMM_DETECT_APPLY_ALL_FOC:' in vp
assert 'm->m_fault != FAULT_CODE_NONE' in mc
assert (R/'tools/tests/hardware/test_hall_detect_repeat.py').exists()
assert 'mcpwm_foc_vesc_override_clear(second)' in mc
assert 'COMM_DETECT_HALL_FOC, 90.0)' in dual

# Released bridge remains torque-free while passive DC-link telemetry may
# continue from the calibrated high-Z path. Check behavior, not old helper names.
assert 'm_id_telem_q4' in mc and 'm_current_in_telem_counts' in mc
assert 'const bool inactive = !source_enabled || !feedback_ready ||' in mc
assert 'if (inactive && !control_update)' in mc
assert 'Released motors still need one Clarke/Park measurement' in mc
assert 'm->m_control_mode==CONTROL_MODE_NONE && m->m_off_dc_valid' in mc
assert 'telemetry_avg_push(m,0,0,passive_dc)' in mc
idx=mc.index('if (inactive) {')
off=mc[idx:mc.index('return;',idx)]
for token in ('m->m_state=MC_STATE_OFF','m->m_vd=0','m->m_vq=0',
              'm->m_pwm_a=0','m->m_pwm_b=0','m->m_pwm_c=0',
              'm->m_iq_set_q4=0','m->m_iq_target_q4=0','m->m_id_set_q4=0',
              'm->m_dq_sample_fresh=0u'):
    assert token in off, token
for forbidden in ('m->m_id_q4=0','m->m_iq_q4=0','m->m_current_in_counts=0'):
    assert forbidden not in off, forbidden

# RX burst handling remains bounded and 16-deep. GET_VALUES stays strict request/reply;
# the only standard unsolicited stream is COMM_ROTOR_POSITION after SET_DETECT,
# matching vedderb/bldc's 10-ms periodic_thread behavior.
assert re.search(r'#define\s+VESC_RX_QUEUE_DEPTH\s+16u',vp)
assert 's_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD]' in vp
assert 's_pending_count < VESC_RX_QUEUE_DEPTH' in vp
assert 'VESC_RT_PERIOD_MS' not in vp and 's_rt_stream' not in vp
assert 'case COMM_SET_DETECT:' in vp and 'COMM_ROTOR_POSITION' in vp
assert 'vesc_protocol_periodic(uint32_t now_ms)' in vp
assert 'vesc_now_ms = HAL_GetTick();' in main and 'vesc_protocol_periodic(vesc_now_ms)' in main and 'usart3_recovery_tick(vesc_now_ms)' in main
# Bare-metal equivalent of VESC's independent PID thread: control deadline must
# execute before potentially long packet/config processing.
assert main.index('mcpwm_foc_outer_control_non_isr(vesc_now_ms)') < main.index('vesc_protocol_process_pending()'), 'outer PID must have priority over VESC parser'
# Timing/control regressions found against VESC + EFeru references.
assert 'm->m_position_dt_ticks=0u' in mc and 'm->m_position_proc_dt_ticks=0u' in mc, 'position derivative dt must start at zero'
assert re.search(r'const\s+uint32_t\s+den\s*=\s*\(uint32_t\)m->m_hall_period\s*\*\s*pp\s*;',mc), 'Hall mechanical RPM must use runtime pole pairs rather than a hard-coded pole count'
assert 'if (m->m_speed_ramp_rpm_s == 0u)' in mc and 'm->m_speed_set_ramp_q16 = target_q16;' in mc, 'zero VESC speed ramp must mean direct setpoint'
assert 'm_speed_release_erpm_q16' in mch and re.search(r'min_erpm_q16\s*=\s*\(int64_t\)m->m_speed_release_erpm_q16',mc), 's_pid_min_erpm must stay in electrical ERPM domain'
assert outer.index('driven_offset_finalize_non_isr();') < outer.index('if(s_outer_pid_last_ms==0u)'), 'powered offset finalization must not wait for 200-Hz housekeeping'
assert 'outer_max_cycles, outer_miss_count, outer_jitter_max_cycles' in mch and 'APPP(p.outer_max_cycles)' in vp and '"outer_jitter"' in dual, 'outer timing profiler must be readable through existing diagnostic packet'
assert 'send_values_packet' in vp and 'send_values_setup_packet' in vp
assert 'strict request/reply' in vp and 'one request -> one reply' in vp
assert 'realtime mailbox latest VESC-tool mapped setpoint' in host and 'request/reply only' in host
assert 'observer must not alias active phase' in host and 'obs-vs-enc must be observer-encoder' in host and 'right observer independent value' in host
assert 'm_observer_x1' in mch and 'mcpwm_foc_get_phase_observer_motor' in mc and 'foc_observer_update_diag' in mc, 'independent VESC observer diagnostic missing'


assert 'MCCONF_HALL_DEBOUNCE_SAMPLES' in mcc and 'm_hall_candidate_count' in mc, 'Hall GPIO debounce missing'
assert 'MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT' in mcc and 'm_hall_filter_window' in mc and 'm_hall_sample_history[41]' in mch and 'exactly one GPIO snapshot' in mc, 'bounded VESC Hall extra-sample rolling-majority filter missing'
assert 'MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES' in mcc and 'm_hall_direction_stable_edges' in mc, 'Hall reversal/acceleration warmup missing'
assert 'm_brake_direction' not in mch and 'm_brake_current_q4' in mch and 'feedback_motion_direction' in mc and 'encoder_motion_fresh' in mc
assert 'm->m_hall_ticks>fresh' in mc and 'MCCONF_TRQ_STOP_RPM_DEADBAND' in mc and 'CONTROL_MODE_CURRENT_BRAKE' in mc

assert 'MCCONF_POSITION_CURRENT_MAX_MA' in mc or 'MCCONF_POSITION_CURRENT_MAX_MA' in mcc
assert 'MCCONF_POSITION_SETTLE_CURRENT_MA' not in mc and 'MCCONF_POSITION_SETTLE_CURRENT_MA' not in mcc
assert 'MCCONF_POSITION_SETTLE_MS' not in mc and 'MCCONF_POSITION_SETTLE_MS' not in mcc

assert 'm->m_hall_direction == dir' in mc, 'Hall period-outlier filter must not reject direction reversals'
assert 'hall_table_runtime_sane' in mc and 'Preserve the last known-good table' in mc, 'runtime Hall-table validation missing'
assert 'hall_feedback_valid' in mc and 'angle==m->m_hall_pos_prev' in mc, 'rejected Hall state must be excluded from feedback-ready gate'
assert 'leftFeedbackReadyPost' in mc and 'rightFeedbackReadyPost' in mc, 'post-control Hall readiness/MOE race guard missing'
assert 'm->m_hall_sequence_reject_count++' in mc and 'static bool hall_drive_ready' in mc and 'm->m_hall_reject_counted_state = h;' in mc, 'Hall sequence reject must keep diagnostic state while bridge readiness remains separately gated'
assert vp.count('const float sl_erpm=buffer_get_float32(data,1e3f,&k);') == 1, 'Detect-All must consume exactly one sl_erpm field from VESC Tool packet'
assert 'measure_r_l_imax_f103_finish' in vp and 'conf_general_calc_apply_foc_cc_kp_ki_gain' in vp and 'mcconf->foc_current_kp=mcconf->foc_motor_l*bw' in vp and 'mcconf->foc_current_ki=mcconf->foc_motor_r*bw' in vp, 'Detect-All must identify R/L and use upstream VESC gain equations'
assert 'left_sensor_encoder' in vp and 'if(s_detect_all.left_sensor_encoder)' in vp and 'detect_all_prepare_hall(mi)' in vp, 'Detect-All must honor LEFT Encoder/Hall selection while RIGHT remains Hall'
assert 'DETECT_ALL_ENCODER_FIXED_SPAN_COUNTS' not in vp and 'mcpwm_foc_encoder_detect(MCCONF_STEERING_DETECT_CURRENT_START_A' in vp and 'steering_span_pending' not in vp, 'Detect-All Encoder must perform electrical detect only and never create/overwrite steering hard-stop span'
assert 'mc_interface_steering_detect_calibrate(' in vp and 'case COMM_DETECT_ENCODER:' in vp and 'mcpwm_foc_encoder_detect(current,false,&eoff,&eratio,&einv)' in vp, 'Standalone Detect Encoder must run electrical ABI detect then physical span calibration'
enc_case=vp[vp.index('case COMM_DETECT_ENCODER:'):vp.index('case COMM_DETECT_HALL_FOC:')]
assert enc_case.index('mcpwm_foc_encoder_detect') < enc_case.index('mc_interface_steering_detect_calibrate'), 'Manual encoder order must be electrical detect before hard-stop span'
assert 'measure_r_l_imax_f103_start(0u,detect_time_now())' in vp and 'conf_general_autodetect_apply_sensors_foc_start(now_time)' in vp, 'Detect-All must identify R/L/flux before sensor commissioning'
assert 's_detect_all.result[mi].sensor_mode=SENSOR_MODE_SENSORLESS' not in vp and 's_detect_all.result[mi].foc_sensor_mode=FOC_SENSOR_MODE_SENSORLESS' not in vp, 'Detect-All must not expose temporary LEFT sensorless selection; OPENLOOP itself is sensor-independent'
assert 'next.foc_sensor_mode!=FOC_SENSOR_MODE_SENSORLESS' not in mc and 'next.sensor_mode=SENSOR_MODE_SENSORED' in mc, 'LEFT ABI/Hall runtime contract must not allow persistent sensorless feedback mode'
assert 'detect_all_sensor_current' in vp and 's_detect_all.imax[mi]/3.0f' in vp and 'mcpwm_foc_hall_detect_start(true,detect_all_sensor_current(1u))' in vp and 's_detect_all_last_detail=second?10:11' in vp, 'Detect-All Hall must use upstream-style Imax/3 current and stage-specific failure detail'
assert 'DETECT_ALL_FLUX_SAMPLE' in vp and 's_detect_all.imax[mi]/2.5f' in vp and 'const float flux=bemf/omega-ldrop' in vp, 'Detect-All flux must follow VESC Imax/2.5 and R/L-corrected open-loop linkage equation'
assert all(('case '+x+':') in vp for x in ('COMM_DETECT_MOTOR_R_L','COMM_DETECT_MOTOR_FLUX_LINKAGE','COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP')), 'Stock VESC Tool R/L/flux commands 25/26/57 must all have handlers'
assert 'local measure R/L values/units' in host and 'local measure flux26' in host and 'right flux57 value/wire encoder tuple' in host, 'R/L/flux wire regressions missing'
assert 'encoder_detect_calls' in host and 'standalone encoder must run electrical ABI detect first' in host, 'Manual encoder electrical-before-span regression missing'
assert 'if(fails!=2u)success=false;' in vp and 'mcpwm_foc_hall_table_sane(table)' not in vp, 'Hall detect success must match upstream fails==2 criterion'
assert 'Standard VESC OPENLOOP_CURRENT' in mc and 'm->m_iq_target_q4=amp_to_q4(m,current)' in mc and 'OPENLOOP_ERPM_Q16_TO_PHASE_Q32_Q24' in mc, 'standard VESC openloop must rotate signed Iq at electrical RPM with precomputed ISR coefficient'
assert 'hall-phase' in (R/'tools/vesc_tool.py').read_text() and 'HALL_PHASE_PASS' in (R/'tools/vesc_tool.py').read_text(), 'active Hall phase-check utility missing'


# F103 realtime efficiency contract: hardware ADC-DMA ISR at 16 kHz, staggered
# PWM/6 regulators, LUT trig, and integer-only R/L sufficient statistics in ISR.
assert 'static const int16_t s_sin_q15[257]' in fm and 'foc_sin_cos_q15' in fm and 'foc_park_q4' in fm and 'foc_inv_park' in fm, 'Q15 trig LUT must drive Park/inverse-Park'
assert 'void DMA1_Channel1_IRQHandler(void)' in mc and 'mcpwm_foc_adc_int_handler();' in mc, 'ADC DMA ISR must own the realtime FOC path'
assert 'if(++s_foc_control_div>=MCCONF_FOC_CONTROL_DIV)' in mc and 'control_slot==0u' in mc and 'control_slot==1u' in mc, 'dual regulator slots must be staggered by FOC_CONTROL_DIV'
assert 'm->m_rl_sum_di2+=(int64_t)p_di2' in mc and 'm->m_rl_sum_div+=(int64_t)p_div' in mc and 'no float/division in ISR' in mc, 'R/L capture must remain integer sufficient-statistics in ISR'
assert 'foc_isr_cycles_max' in mc and 'm_overrun_count' in mch, 'ISR cycle/overrun instrumentation missing'
# Heavy floating-point diagnostics are explicitly outside motor_control_step/ADC ISR.
isr_body=mc[mc.index('static void motor_control_step'):mc.index('void mcpwm_foc_housekeeping_non_isr')]
for forbidden in ('atan2f(', 'sinf(', 'cosf(', 'sqrtf(', 'sqrt(', 'powf(', 'logf('):
    assert forbidden not in isr_body, f'heavy math leaked into motor_control_step ISR path: {forbidden}'
assert 'foc_observer_update_diag(&m_motor_1,dt)' not in isr_body, 'float observer must remain outside ISR'
assert 'mcpwm_foc_housekeeping_non_isr' in mc and 'foc_observer_update_diag(&m_motor_1,dt)' in mc, 'observer housekeeping split missing'

# Standard VESC handbrake command must not be silently ignored.
assert 'case COMM_SET_HANDBRAKE:' in vp and 'mc_interface_set_handbrake(current)' in vp
dual=(R/'tools/vesc_dual.py').read_text()
assert 'COMM_SET_HANDBRAKE = 10' in dual and 'def handbrake(' in dual

assert 'leftOffDcValid' in mc and 'rightOffDcValid' in mc and 'only DC-link current is authoritative' in mc, 'OFF telemetry must remain a real calibrated DC-link measurement with phase shunts excluded in high-Z'
assert 'steering_center_after_span_calibration' in mci, 'steering detect must have bounded midpoint finalizer'
assert 'ok=mc_interface_store_configuration_motor(false);' in vp, 'Detect Encoder must persist detected ABI electrical config before success'
assert 'MCCONF_STEERING_CENTER_TOL_COUNTS' in mci and 'steering_stage_set(0xE8u)' in mci, 'detect must fail closed when midpoint centering fails'
assert 'mcpwm_foc_steering_rebase_center()' in mci, 'successful detect must rebase measured midpoint to logical zero'
print('V16_FEATURE_STATIC_PASS names=1 hall_midpoint=1 hall_rate_limit=1 hall_debounce=1 reversal_warmup=1 detect_1deg_6sweep=1 current_off_live=1 rx_fifo16=1 vesc_request_reply=1 hidden_brake_removed=1 std_pos=1 custom_count_cap=1 std_openloop=1')

assert 'MCCONF_STEERING_POS_MIN_DEG' in (R/'Src/motor/mcconf_default.h').read_text()
assert 'normalized 0..360 steering-actuator coordinate' in vp
assert 'ROS/ROS Web owns that calibration' in vp
assert 'HB_CUSTOM_SET_STEERING_DEG' in vp
