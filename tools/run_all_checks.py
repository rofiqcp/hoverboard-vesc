#!/usr/bin/env python3
from pathlib import Path
import hashlib, re, shutil, subprocess, sys, tempfile

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_DATATYPES_SHA256 = '4ecae1f31c12c1ab415d47dd997396d0792e94249203cbeb877ada75f76d5340'

def run(cmd):
    print('+', ' '.join(map(str, cmd)))
    r = subprocess.run(cmd, cwd=ROOT, text=True)
    if r.returncode:
        raise SystemExit(r.returncode)

def check_static():
    required = [
        'platformio.ini','STM32F103RCTx_APP.ld','STM32F103RCTx_BOOTLOADER.ld',
        'Src/main.c','Src/setup.c','Src/util.c','Src/comms.c','Src/config.h',
        'Src/motor/foc_math.c','Src/motor/mc_interface.c','Src/motor/mcpwm_foc.c',
        'Src/encoder/encoder.c','Src/encoder/encoder.h','Src/encoder/encoder_datatype.h',
        'Src/encoder/enc_abi.c','Src/encoder/enc_abi.h','Src/encoder/encoder_cfg.c','Src/encoder/encoder_cfg.h',
        'Src/vesc/buffer.c','Src/vesc/crc.c','Src/vesc/mcconf_serial.c','Src/vesc/vesc_protocol.c','Src/vesc/flash_update_f103.c','Src/vesc/flash_update_f103.h','Src/vesc/f103_boot_layout.h','Src/bootloader/main.c',
        'Src/motor/mcpwm_foc.c','Src/motor/mcpwm_foc.h','Src/motor/mc_interface.c','Src/motor/mc_interface.h',
        'Src/motor/foc_math.c','Src/motor/foc_math.h','Src/motor/mcconf_default.h',
        'Src/vesc/datatypes.h','Src/vesc/vesc_protocol.c','Src/vesc/vesc_protocol.h',
        'Src/vesc/buffer.c','Src/vesc/crc.c','Src/vesc/mcconf_serial.c',
        'tools/build_factory_image.py','tools/install_bootloader_stlink.sh','tools/pio_vesc_upload.py','tools/tests/target/test_isr_callgraph.py','tools/vesc_dual.py','tools/vesc_tool.py','tools/tests/hardware/test_vesc_tool_rt50.py','tools/tests/host/test_hall_3rev_runtime.py','tools/tests/host/test_hall_3rev_runtime.c','tools/tests/host/test_motor_control_v12.py','tools/tests/host/test_motor_control_v12.c','tools/tests/host/test_motor_control_v13.py','tools/tests/host/test_motor_control_v13.c','tools/tests/host/test_v13_features.py','tools/tests/host/test_v14_features.py','tools/tests/host/test_v15_features.py'
    ]
    missing=[x for x in required if not (ROOT/x).exists()]
    assert not missing, f'missing required files: {missing}'
    for obsolete in ('Src/BLDC_controller.c','Src/BLDC_controller.h','Src/BLDC_controller_data.c','Src/bldc.c','Src/rtwtypes.h','Src/current_scale.h'):
        assert not (ROOT/obsolete).exists(), f'obsolete generated file still present: {obsolete}'
    port_files=list((ROOT/'Src').rglob('port_*.c')) + list((ROOT/'Src').rglob('port_*.h'))
    assert not port_files, f'port wrapper files still present: {[str(x.relative_to(ROOT)) for x in port_files]}'
    source_text='\n'.join(p.read_text(errors='ignore') for p in (ROOT/'Src').rglob('*') if p.is_file())
    # Historical comments may mention the generated model provenance. Strip C/C++
    # comments before checking active dependencies so documentation cannot fail CI.
    active_source_text=re.sub(r'/\*.*?\*/|//[^\n]*','',source_text,flags=re.S)
    for token in ('BLDC_controller','rtwtypes.h','rtP_Left','rtP_Right','rtDW_Left','rtDW_Right'):
        assert token not in active_source_text, f'obsolete generated dependency in live source: {token}'
    ini=(ROOT/'platformio.ini').read_text()
    for token in ('src_dir = Src','[env:APP_STLINK]','[env:APP_USART_PC]','[env:BOOTLOADER_STLINK]','board = genericSTM32F103RC','build_src_flags =','-Wall','-Wextra','-Werror','-I.'):
        assert token in ini, f'platformio.ini missing {token}'
    # The default target may be switched intentionally for deployment/recovery,
    # but it must always name a declared PlatformIO environment.
    m = re.search(r'^default_envs\s*=\s*([A-Za-z0-9_,-]+)\s*$', ini, re.M)
    assert m, 'platformio.ini must declare default_envs'
    for env_name in (x.strip() for x in m.group(1).split(',') if x.strip()):
        assert f'[env:{env_name}]' in ini, f'default PlatformIO env is not declared: {env_name}'
    # Warning policy: project sources use -Wall/-Wextra/-Werror via build_src_flags only.
    # Framework STM32Cube must not inherit project -Werror (avoids HAL_PCD unused-parameter build failure).
    before_build_flags=ini.split('build_flags =',1)[0]
    assert '-Werror' in before_build_flags and 'build_src_flags =' in before_build_flags, 'project -Werror is not scoped with build_src_flags'
    global_build=ini.split('build_flags =',1)[1]
    assert '-Werror' not in global_build and '-Wextra' not in global_build and '-Wall' not in global_build, 'warning flags leaked into framework build_flags'
    cfg=(ROOT/'Src/config.h').read_text()
    assert 'current_scale.h' not in cfg and 'CURRENT_COUNTS_PER_A' not in cfg, 'obsolete current_scale dependency remains'
    assert re.search(r'#define\s+A2BIT_CONV\s+50\b',cfg), 'A2BIT_CONV must be defined directly as 50 in config.h'
    assert re.search(r'#define\s+ADC_CLOCK_DIV\s+6\b',cfg), 'F103 ADC clock must be PCLK2/6 (10.667 MHz at 64 MHz PCLK2)'
    mainc_hw=(ROOT/'Src/main.c').read_text()
    assert 'RCC_ADCPCLK2_DIV6' in mainc_hw and 'RCC_ADCPCLK2_DIV4' not in mainc_hw, 'production ADC clock must not exceed STM32F103 14-MHz limit'
    assert re.search(r'#define\s+SERIAL_BUFFER_SIZE\s+768\b',cfg), 'USART3 RX DMA buffer is not 768 bytes'
    mc=(ROOT/'Src/motor/mcpwm_foc.c').read_text()
    mathc=(ROOT/'Src/motor/foc_math.c').read_text()
    mcc=(ROOT/'Src/motor/mcconf_default.h').read_text()
    assert re.search(r'#define\s+MCCONF_FOC_CONTROL_DIV\s+6u',mcc), 'FOC scheduler must match current 1-of-6 CPU-safe cadence'
    assert 'c->foc_f_zv=(float)PWM_FREQ;' in mc and 'MCCONF_FOC_DT_US_MAX' in mc and 'dt_ns' in mc, 'fixed 16-kHz PWM or bounded foc_dt_us canonicalization missing'
    assert 'c->foc_overmod_factor=1.0f;' in mc and 'c->foc_mag_vd_max=1.0f;' in mc, 'unsupported VESC overmod/Vd fields must read back as fixed-safe values'
    assert 'foc_deadtime_sign_q15' in mc and 'nilai kompensasi TIDAK ditambahkan ke switching command' in mc, 'VESC dead-time model compensation missing'
    assert 'speed_pid_iq_target_step' in mc and 'error_q16' in mc, 'VESC speed PID fixed-point ERPM path missing'
    motor_step_start=mc.index('static void motor_control_step')
    motor_step_end=mc.index('static int16_t duty_permille_from_vdq',motor_step_start)
    motor_step=mc[motor_step_start:motor_step_end]
    # Upstream VESC semantics: SPEED/POS run in a dedicated PID scheduler, while
    # the ADC ISR only closes Id/Iq current control against a cached Iq target.
    outer_start=mc.index('void mcpwm_foc_outer_control_non_isr')
    outer_end=mc.index('void mcpwm_foc_housekeeping_non_isr',outer_start)
    outer=mc[outer_start:outer_end]
    assert re.search(r'#define\s+MCCONF_OUTER_PID_HZ\s+1000u',mcc), 'outer VESC PID scheduler must be 1 kHz'
    assert 'm->m_iq_target_q4=speed_pid_iq_target_step' in outer, 'speed PID must run in 1-kHz outer scheduler'
    assert 'm->m_iq_target_q4=position_pid_iq_target_step' in outer, 'position PID must run in 1-kHz outer scheduler'
    assert 'speed_pid_iq_target_step' not in motor_step and 'position_pid_iq_target_step' not in motor_step, 'SPEED/POS PID must not execute in hot ADC ISR'
    assert 'elapsed_ms=now_ms-s_outer_pid_last_ms' in outer and 'speed_setpoint_slew_step(m,elapsed_ms)' in outer and 'pid_dt_ms=elapsed_ms' in outer, 'outer scheduler must use real wall-time slew and bounded fresh PID dt without stale virtual catch-up'
    assert 'float' not in outer and 'sqrtf' not in outer and 'lroundf' not in outer, '1-kHz outer scheduler must remain fixed-point/integer'
    assert 'motor_outer_loop_virtual_steps' not in mc, 'stale-feedback virtual outer-loop architecture must not remain'
    assert 'speed PI drives Vq directly' not in mc, 'obsolete EFeru speed-PI-to-Vq architecture remains'
    assert 'current_pi_vesc_state' in motor_step and 'm_current_kpq_v_q16' in mc, 'ISR must retain upstream-style physical Id/Iq current PI'
    assert 'if(m->m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_DISABLED)' in motor_step, 'redundant final vector sqrt must be skipped when decoupling is disabled'
    assert 'CONTROL_MODE_CURRENT_BRAKE' not in mc[mc.index('if (mode==TRQ_MODE)'):mc.index('} else if (mode==SPD_MODE)')], 'legacy TRQ STOP must not brake'
    assert 'm_duty_limit_permille' in mc and 'MCCONF_FOC_DUTY_VOLTAGE_MAX' in mc and 'voltage_circle_q_limit' in mc, 'runtime l_max_duty voltage-circle anti-windup missing'
    assert 's_sqrt_q7[257]' in mathc and '__builtin_clz' in mathc and 'for(uint8_t k=0u;k<4u' in mathc, 'ISR integer sqrt must use bounded Flash LUT implementation'
    assert 'uint32_t op=x, res=0, one=1u<<30' not in mathc, 'legacy iterative restoring sqrt remains in ISR math'
    assert 'watt_current_limits_refresh' in mc and 'm_watt_current_max_q4' in mc and 'm_watt_current_regen_q4' in mc, 'VESC watt P/V limit must be cached outside ISR'
    fault_block=mc[mc.index('static void motor_fault_set'):mc.index('static void motor_reset')]
    assert 'uint64_t' not in fault_block and '__aeabi' not in fault_block and 'm_fault_stop_ticks' in fault_block, 'fault set path must use precomputed timeout ticks'
    house=mc[mc.index('void mcpwm_foc_housekeeping_non_isr'):mc.index('void mcpwm_foc_adc_int_handler')]
    adc_handler=mc[mc.index('void mcpwm_foc_adc_int_handler'):mc.index('void mcpwm_foc_set_board_temperature_x10')]
    dma_isr=mc[mc.index('void f103_DMA1_Channel1_IRQHandler_impl'):mc.index('#if !defined(__arm__)', mc.index('void f103_DMA1_Channel1_IRQHandler_impl'))]
    # Fault recovery and E-stop duration remain slow bookkeeping, but the actuator
    # command deadline itself must be decremented by the 16-kHz hardware clock so
    # a deadlocked main/parser cannot hold stale torque. The ISR only marks expiry
    # and drops MOE; main performs the heavier release/brake transition afterwards.
    assert 'm_fault_recovery_ticks' in house and 's_estop_ticks' in house and 's_vesc_timeout_expired' in house, 'slow recovery/E-stop and timeout transition missing from housekeeping'
    assert 's_vesc_timeout_ticks' in dma_isr and 's_vesc_timeout_expired[wi]=1u' in dma_isr and 'BDTR&=~TIM_BDTR_MOE' in dma_isr, 'hard VESC actuator deadline must fail closed in the 16-kHz DMA ISR'
    assert 'm_fault_recovery_ticks' not in adc_handler and 's_vesc_timeout_ticks' not in adc_handler and 'mcpwm_foc_set_mode_command' not in adc_handler, 'current-regulator helper must not service recovery/timeout/source arbitration'
    assert 'm->m_iq_set_q4=m->m_iq_target_q4' in mc and 'MCCONF_CURRENT_SLEW_A_PER_S' not in mcc, 'SET_CURRENT must be direct VESC reference without legacy slew'
    assert 'MCCONF_SPEED_GAIN_SCALE' in mc and 'MCCONF_SPEED_GAIN_SCALE' in mcc, 'high-resolution speed PID gain scale missing'
    assert 'm_brake_current_q4' in mc and 'feedback_motion_direction' in mc and 'encoder_motion_fresh' in mc and 'm->m_hall_ticks>fresh' in mc and 'CONTROL_MODE_CURRENT_BRAKE' in mc, 'VESC live-direction current brake path missing'
    assert 'CONTROL_MODE_HANDBRAKE' in mc and 'm->m_phase=0u' in mc and 'mcpwm_foc_set_handbrake' in mc, 'VESC handbrake fixed-phase mode missing'
    assert 'leftDqFresh=m_motor_1.m_dq_sample_fresh' in mc and 'rightDqFresh=m_motor_2.m_dq_sample_fresh' in mc and 'leftDqFresh&&leftPhaseExceeded' in mc and 'rightDqFresh&&rightPhaseExceeded' in mc and 'leftDcTrip = leftCurrentSampleValid' in mc and 'rightDcTrip = rightCurrentSampleValid' in mc, 'D/Q ABS must use distinct fresh samples while raw DC trip remains active every driven ISR'
    assert 'leftDriveRequest' in mc and 'rightDriveRequest' in mc, 'free-run must gate each motor bridge/MOE'
    assert 'Safety gate phase 2' in mc and 'leftFeedbackReadyPost' in mc and 'rightFeedbackReadyPost' in mc and 'if(leftDriveRequest && leftFeedbackReadyPost && !leftCurrentTrip' in mc, 'bridge must arm only after FOC CCR update and post-Hall readiness check'
    assert 'MCCONF_HALL_PERIOD_OUTLIER_RATIO' in mc, 'Hall chatter outlier rejection missing'
    telem_start=mc.index('void mcpwm_foc_get_values_scaled')
    telem=mc[telem_start:]
    assert telem.count('foc_telem_isr_snapshot(m,&is)') >= 2, 'scaled and float VESC telemetry must each take one coherent ISR snapshot'
    assert 'is.encoder_erpm_q16' in telem and 'is.hall_ref_rpm' in telem and 'motor_pole_pairs(second)' in telem, 'VESC RPM paths must use encoder/Hall fields captured in the ISR snapshot'
    assert 'm->m_encoder_erpm_q16' not in telem and 'm->m_hall_ref_rpm' not in telem, 'VESC telemetry RPM must not mix live ISR fields outside the coherent snapshot'
    assert 'erpm_to_mech_rpm_q16' in mc and 'measured_mech_rpm_q16' in mc, 'VESC COMM_SET_RPM fractional ERPM conversion missing'
    assert 'if(openloop_phase) m->m_phase=m->m_phase_openloop;' in mc and 'm_phase_openloop + (65536/12)' not in mc, 'mode4 has incorrect +30deg phase offset'
    assert 'hall_table_angle' in mc and 'm->m_conf.foc_hall_table' in mc, 'Hall estimator must use VESC foc_hall_table'
    assert 'mcpwm_foc_hall_detect' in mc and 'fails == 2u' in mc and 'foc_atan2_phase_u16(ys,xc)' in mc, 'upstream-equivalent Hall detector/fixed-point finalization missing'
    assert 'MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT' in mcc and 'hall_interp_recompute' in mc and 'm_hall_interp_max_ticks' in mc and 'm_hall_rate_min_step' in mc and 'err_same_direction' in mc, 'VESC foc_hall_interp_erpm runtime semantics missing'
    assert 'MCCONF_HALL_PHASE_ADVANCE_TICKS' in mc and 'debounce_adv' in mc, 'Hall debounce phase-delay compensation missing'
    assert 'phase_current_counts_to_q4' in mc and '27200' in mc, 'generated current input saturation missing'
    assert 'duty_control_iq_target_step' in mc and 'm->m_iq_target_q4=duty_control_iq_target_step(m,second)' in mc, 'mode1 must use VESC-style current-controlled duty with per-motor ERPM governor'
    assert 'foc_isqrt_u32(mag2)' in mc and 'mag*1000u' in mc and 'MCCONF_FOC_DUTY_VOLTAGE_MAX' in mc, 'VESC duty telemetry must normalize EFeru full-safe vector to 1.0'
    assert 'MCCONF_FOC_DUTY_VOLTAGE_MAX' in mc and 'duty_v>MCCONF_FOC_DUTY_VOLTAGE_MAX' in mc, 'mode1 EFeru full-safe modulation ceiling missing'
    assert re.search(r'#define\s+MCCONF_L_MAX_DUTY\s+0\.95f',mcc), 'configured duty ceiling must retain the 0.95 two-shunt sampling safety margin'
    assert re.search(r'#define\s+MCCONF_L_IN_CURRENT_MAX\s+15\.0f',mcc) and re.search(r'#define\s+MCCONF_L_IN_CURRENT_MIN\s+-15\.0f',mcc), 'DC-link soft limit must be +/-15A'
    assert ('const int32_t mod_q=(int32_t)m->m_vq;' in mc and 'const int32_t in_num=in_lim*(int32_t)MCCONF_FOC_VOLTAGE_MAX;' in mc and 'if(in_num < lim*amod_q)' in mc and 'motor_from_input=in_num/amod_q' in mc), 'FOC input-current limit must retain exact VESC mod_q*Iq semantics with divide only when it can tighten the limit'
    assert '|Ibus| ~= |Iq|*|duty|' not in mc, 'obsolete duty-magnitude input-current approximation remains'
    assert 'MCCONF_DUTY_RAMP_STEP_DEFAULT' in mcc and 'm_duty_ramp_permille' not in mc and 'duty_setpoint_slew_step' not in mc and 'm_duty_set_permille' in mc, 'FOC duty must use direct VESC target; m_duty_ramp_step is wire-compatible BLDC config only'
    assert re.search(r'#define\s+VESC_DUTY_PHYSICAL_SCALE_PERMILLE\s+960',cfg), 'board VESC duty scale must be 0.960'
    assert re.search(r'#define\s+FOC_SVPWM_VECTOR_FULL_SAFE\s+14238',cfg), 'EFeru full-safe SVPWM reference vector must be 14238'
    assert re.search(r'#define\s+FOC_PWM_MARGIN_COUNTS\s+110',cfg), 'EFeru FOC PWM margin must be 110 counts'
    assert 'MCCONF_FOC_DUTY_VOLTAGE_MAX          FOC_SVPWM_VECTOR_MAX' in mcc and 'MCCONF_PWM_MARGIN_COUNTS          FOC_PWM_MARGIN_COUNTS' in mcc, 'MC config must consume config.h hardware PWM scaling'
    assert 'MCCONF_ABS_CURRENT_QUAL_SAMPLES' in mc and 'const uint32_t fast2=' in mc and 'const uint32_t slow2=' in mc and 'leftDcTrip' in mc and 'm_phase_trip_count' in mc and 'm_dc_trip_count' in mc and 'm_dq_sample_fresh' in mc, 'fresh-sample DQ-qualified ABS/DC hard protection diagnostics missing'
    assert 'MCCONF_BRIDGE_SETTLE_SAMPLES' in mc and 'm_bridge_settle_ticks' in mc and 'leftCurrentSampleValid' in mc, 'OFF-to-RUN current sample blanking missing'
    assert 'm_telem_sum_id_q4' in mc and 'm_telem_avg_samples' in mc and 'telemetry_avg_push' in mc, 'VESC-style read/reset current averaging missing'
    assert 'l_abs_current_max' in mc and 'MCCONF_L_ABS_CURRENT_MAX' in mc, 'VESC-style absolute current fault limit missing'
    assert 'trq_ca_to_q4' in mc and 'A2BIT_CONV*16)/100' in mc, 'mode3 centiampere scaling missing'
    assert 'm_openloop_id_ramp_q16' in mc and 'MCCONF_OPENLOOP_ID_SLEW_A_S' in mc, 'mode4 Id slew protection missing'
    assert re.search(r'#define\s+SVPWM_MAX_ID_A\s+6u',cfg), 'mode4 Id safety ceiling must be 6A'
    assert re.search(r'#define\s+SVPWM_PHASE_LIMIT_A\s+8u',cfg), 'mode4 phase-current trip must be 8A'
    assert re.search(r'#define\s+SVPWM_DC_LIMIT_A\s+8u',cfg), 'mode4 DC-link trip must be 8A'
    assert re.search(r'#define\s+SVPWM_OPENLOOP_RPM_DEFAULT\s+10u',cfg), 'mode4 default open-loop speed must be 10 rpm'
    assert re.search(r'#define\s+SVPWM_ID_SLEW_A_PER_S\s+4u',cfg), 'mode4 Id slew must be 4 A/s'
    assert 'm->m_iq_q4=raw.q; m->m_id_q4=raw.d;' in mc and 'integrator += Ierr * Ki * dt' in mc, 'VESC raw-current PI equation missing'
    enc=(ROOT/'Src/encoder/encoder.c').read_text(); abi=(ROOT/'Src/encoder/enc_abi.c').read_text(); ench=(ROOT/'Src/encoder/encoder.h').read_text()
    assert 'SENSOR_PORT_MODE_ABI' in enc and 'enc_abi_init' in enc and 'encoder_read_deg' in enc, 'VESC ABI encoder dispatcher missing'
    assert 'TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1' in abi and 'TIM4' in (ROOT/'Src/encoder/encoder_cfg.c').read_text(), 'TIM4 quadrature ABI driver missing'
    assert 'encoder_feedback_update' in mc and 'm_phase_encoder' in mc and 'encoder_feedback_selected' in mc, 'ABI phase not integrated into FOC'
    for dead in ('encoder_read_deg_multiturn','encoder_reset_multiturn','encoder_reset_errors','encoder_get_error_rate','encoder_check_faults','encoder_pin_isr','encoder_tim_isr','encoder_get_counts','enc_abi_pin_isr'):
        assert dead not in enc and dead not in ench and dead not in abi, f'dead encoder API remains: {dead}'
    assert 'mcpwm_foc_encoder_startup_align' in mc and 'mcpwm_foc_encoder_detect' in mc, 'ABI lifecycle/detect missing'
    assert 'PB6/PB7' in mc or 'PB6/PB7' in abi, 'shared Hall/ABI pin constraint not documented'
    vp=(ROOT/'Src/vesc/vesc_protocol.c').read_text()
    assert re.search(r'#define\s+VESC_MAX_PAYLOAD\s+700u',vp), 'VESC payload buffer is not 700 bytes'
    assert 'VESC_TX_QUEUE_DEPTH' in vp and 'vesc_tx_service' in vp and 'HAL_UART_Transmit_DMA(&huart3, s_tx_frame[slot], n)' in vp, 'VESC reply path must use nonblocking USART3 TX DMA FIFO'
    assert 'while (huart3.gState != HAL_UART_STATE_READY)' not in vp, 'VESC reply path must never busy-wait on UART TX'
    assert re.search(r'#define\s+VESC_FW_MAJOR\s+6u',vp) and re.search(r'#define\s+VESC_FW_MINOR\s+0u',vp), 'firmware must identify as VESC 6.00'
    assert 'COMM_DETECT_HALL_FOC' in vp and 'mcpwm_foc_hall_detect_command_start' in vp and 'mcpwm_foc_hall_detect_process' in vp, 'VESC-standard async Hall detect command missing'
    assert 'case COMM_DETECT_ENCODER:' in vp and 'mc_interface_steering_detect_calibrate' in vp and 'buffer_append_float32(reply,off,1e6f' in vp, 'VESC-standard encoder detect + steering commissioning command missing'
    assert 'DETECT_ALL_ENCODER_FIXED_SPAN_COUNTS' not in vp and 'steering_span_pending' not in vp, 'Detect-All must never invent or overwrite LEFT steering hard-stop span'
    assert 'mcpwm_foc_encoder_detect(MCCONF_STEERING_DETECT_CURRENT_START_A' in vp, 'Detect-All LEFT encoder electrical commissioning missing'
    assert 'RIGHT Hall-only' in vp and 'if(!second&&!strcmp(sv,"encoder"))' in vp[vp.index('static int terminal_cfg_one'):vp.index('static void process_terminal_command')], 'RIGHT Hall-only contract missing from terminal/config path'
    assert 'mc_interface_store_configuration_motor(second)' in vp, 'VESC MC config/Hall persistence missing'
    serial=(ROOT/'Src/vesc/mcconf_serial.h').read_text()
    assert 'MCCONF_SIGNATURE 776184161u' in serial, 'VESC 6.00 MC config signature mismatch'
    assert 'APPCONF_SIGNATURE 486554156u' in serial, 'VESC 6.00 App config signature mismatch'
    serc=(ROOT/'Src/vesc/mcconf_serial.c').read_text()
    assert 'appconf6_append_balance_placeholder' in serc and 'appconf6_skip_balance_placeholder' in serc, 'VESC 6.00 balance wire block adapter missing'
    assert 'coast_brake_level' not in serc and 'coast_brake_ramp_time' not in serc, 'post-6.00 Chuk fields leaked into VESC 6.00 app wire format'
    assert 'if (c->si_motor_poles < 2u || (c->si_motor_poles & 1u)) c->si_motor_poles = 30u;' in vp and 'c->si_gear_ratio >= 0.01f' in vp, 'SET_MCCONF runtime poles/gear validation missing'
    mci=(ROOT/'Src/motor/mc_interface.c').read_text()
    assert 'EE_L_MOTOR_POLES' in mci and 'EE_L_GEAR_X64' in mci and 'mcpwm_foc_get_pole_pairs(second)' in mci, 'runtime motor poles/gear persistence missing'
    assert 'EE_L_CFG_SIGNATURE = 43, EE_R_CFG_SIGNATURE = 44' in mci and \
        'EE_CFG_SIGNATURE_VALUE 0x6022u' in mci and 'EE_CFG_SIGNATURE_V35   0x6021u' in mci and 'EE_CFG_SIGNATURE_V34   0x6020u' in mci and 'EE_CFG_SIGNATURE_V33   0x601Fu' in mci and 'EE_CFG_SIGNATURE_V32   0x601Eu' in mci and 'EE_CFG_SIGNATURE_V31   0x601Du' in mci and \
        'EE_CFG_SIGNATURE_V30   0x601Cu' in mci and 'EE_CFG_SIGNATURE_V29   0x601Bu' in mci and \
        'EE_CFG_SIGNATURE_V28   0x601Au' in mci and 'EE_CFG_SIGNATURE_V27   0x6019u' in mci and \
        'EE_CFG_SIGNATURE_V26   0x6018u' in mci and 'EE_CFG_SIGNATURE_V25   0x6017u' in mci and \
        'EE_CFG_SIGNATURE_V24   0x6016u' in mci and 'EE_CFG_SIGNATURE_V23   0x6015u' in mci and \
        'EE_CFG_SIGNATURE_V22   0x6014u' in mci and 'EE_CFG_SIGNATURE_V21   0x6013u' in mci and \
        'EE_CFG_SIGNATURE_V20   0x6012u' in mci and 'EE_CFG_SIGNATURE_V19   0x6011u' in mci and \
        'EE_CFG_SIGNATURE_V18   0x6010u' in mci and 'EE_CFG_SIGNATURE_V17   0x600Fu' in mci and \
        'EE_CFG_SIGNATURE_V16   0x600Eu' in mci and 'EE_L_EXT_CURRENT_MIN_CA = 123' in mci and \
        'EE_R_EXT_CURRENT_MIN_CA = 129' in mci and 'EE_L_CC_MIN_CURRENT_CA = 141' in mci and \
        'EE_R_CC_MIN_CURRENT_CA = 142' in mci and 'EE_L_EXT2_BAT_CUT_START_CV = 143' in mci and \
        'EE_R_EXT2_BAT_CUT_START_CV = 150' in mci and 'EE_L_EXT2_BAT_META = 157' in mci and \
        'EE_R_EXT2_BAT_META = 159' in mci and 'EE_L_EXT5_HALL_INTERP_ERPM = 179' in mci and \
        'EE_R_EXT5_HALL_INTERP_ERPM = 180' in mci and 'EE_L_EXT6_ENCODER_FLAGS = 181' in mci and \
        'EE_L_EXT6_ENCODER_RATIO_X10000_HI' in mci and 'EE_EXT7_DIRECTION_FLAGS = 186' in mci and \
        'EE_L_EXT7_ENCODER_OFFSET_F32_LO' in mci and 'EE_L_EXT7_ENCODER_RATIO_F32_LO' in mci and \
        'EE_L_EXT7_PID_KD_PROC_F32_LO' in mci and 'EE_L_EXT7_PID_GAIN_DEC_X10' in mci and \
        'EE_L_EXT9_R_LO' in mci and 'EE_R_EXT9_FLUX_HI' in mci and 'foc_hall_table' in mci, 'VESC 6.00 dual EEPROM persistence/migration missing'
    assert 'COMM_GET_DECODED_ADC' in vp and 'reply_decoded_adc' in vp, 'VESC decoded ADC command missing'
    assert 'board_temp_deg_c * 0.1f' in vp, 'VESC temperature telemetry must convert deci-C to C'
    assert 'mcpwm_foc_energy_update' in mc and 'v->amp_hours=m->m_amp_seconds/3600.0f' in mc and 'v->watt_hours=m->m_watt_seconds/3600.0f' in mc, 'VESC Ah/Wh live counters missing'
    appv=(ROOT/'Src/vesc/app_vesc.c').read_text()
    assert 'adc_buffer.adc2_spare4' in appv and 'adc_buffer.adc2_spare5' in appv, 'PA2/PA3 ADC2 app mapping missing'
    assert 'APP_ADC_UART' in appv and 'app_vesc_process' in appv, 'APP_ADC/UART runtime missing'
    assert 'second ? -amp : amp' not in appv and 'second ? -duty : duty' not in appv and 'second ? -erpm : erpm' not in appv, 'App ADC must use mc_interface DIR_MULT, not endpoint-specific sign hacks'
    assert 'ADC_CTRL_TYPE_NONE is telemetry-only' in appv and 'if (c->ctrl_type == ADC_CTRL_TYPE_NONE)' in appv, 'ADC NONE must not energize/touch motor'
    assert 'c.app_adc_conf.throttle_exp_mode != THR_EXP_POLY' in appv and 'powf(' not in appv and 'expf(' not in appv, 'F103 App ADC must canonicalize heavy EXPO/NATURAL curves to standard POLY'
    assert 'a->timeout_msec = VESC_RUNTIME_TIMEOUT_DEFAULT_MS;' in appv, 'VESC App Config default must use local hard watchdog'
    assert 'VESC_RUNTIME_TIMEOUT_DEFAULT_MS 1000u' in cfg and 'VESC_RUNTIME_TIMEOUT_MAX_MS     1000u' in cfg, 'F103 hard watchdog bounds missing'
    assert 'c.timeout_msec == 0u' in appv and 'VESC_RUNTIME_TIMEOUT_MAX_MS' in appv, 'App Config must not disable/extend actuator watchdog'
    assert 'mcpwm_foc_vesc_timeout_configure(second, c.timeout_msec, c.timeout_brake_current)' in appv, 'App Config timeout/brake must drive motor watchdog'
    assert 'mc_interface_set_current_rel(rel)' in appv and 'mc_interface_set_brake_current_rel(rel)' in appv, 'App ADC current modes must use upstream VESC relative-current helpers'
    assert 'void mc_interface_set_current_rel(float val)' in mci and 'void mc_interface_set_brake_current_rel(float val)' in mci, 'VESC current-rel helpers missing from mc_interface'
    for setter in ('mc_interface_set_duty(', 'mc_interface_set_current(', 'mc_interface_set_brake_current(', 'mc_interface_set_pid_speed('):
        pos=vp.find(setter)
        assert pos >= 0 and vp.rfind('touch_motor(second)', max(0,pos-140), pos) >= 0, f'VESC ownership must be claimed before {setter}'
    assert 'alive_local' in vp and 'alive_right' in vp and 'mcpwm_foc_vesc_override_touch(alive_right)' in vp, 'COMM_ALIVE must bypass RX FIFO after CRC validation'
    assert 'vesc_protocol_link_active() ? 1u : 0u' in vp, 'diagnostic ARM must follow live VESC Tool/Python telemetry link'
    assert 'step>=64251u && step<=66873u' in mc and 'const float div=m->m_conf.p_pid_ang_div;' not in mc[mc.find('static void position_feedback_update'):mc.find('static void encoder_runtime_configure')], '16-kHz position feedback must not use soft-float comparisons'
    assert 'detect_time_now' in vp and 'return buzzerTimer;' in vp and 'PWM_FREQ / 1000u' in vp and 'detect_time_elapsed_ms(s_hall_detect.align_start_time, now_time)' in vp and 'elapsed_ms < 1000u' in vp, 'Hall/Detect-All timing must use DWT real elapsed time on hardware, not lossy SysTick/main-loop visits'
    assert 's_vesc_owned[2]' in mc and 's_vesc_timeout_ticks[2]' in mc, 'VESC ownership and timeout must be separate state'
    assert 'POWER_OFF_ENABLE          0' in cfg and 'POWER_BUTTON_BYPASS       1' in cfg, 'development power latch bypass missing'
    assert 'm_fault_recovery_ticks' in mc and 'm_fault_stop_time_ms' in mc, 'fault recovery timer missing'
    assert 'COMM_FORWARD_CAN' in vp and 'COMM_PING_CAN' in vp, 'virtual CAN routing missing'
    assert 'right_sign' not in vp and 'mc_interface_set_duty(duty)' in vp and 'mc_interface_set_pid_speed(rpm)' in vp, 'VESC protocol must forward motor-local coordinates unchanged'
    assert re.search(r'#define\s+VESC_SECOND_MOTOR_ID\s+2u',vp), 'virtual right ID must be 2'
    dual=(ROOT/'tools/vesc_dual.py').read_text()
    assert 'RIGHT_ID = 2' in dual and 'COMM_FORWARD_CAN = 34' in dual, 'Python right virtual CAN routing mismatch'
    util=(ROOT/'Src/util.c').read_text()
    for token in ('usart3_recovery_tick', 'USART3_VALID_PROGRESS_TIMEOUT_MS',
                  'USART3_RAW_RECENT_MS', 'USART3_RECOVERY_COOLDOWN_MS',
                  'mcpwm_foc_release_motor(false)', 'mcpwm_foc_release_motor(true)',
                  'HAL_UART_DMAStop(&huart3)', 'vesc_protocol_transport_reset()'):
        assert token in util, f'F103 USART3 robust recovery missing: {token}'
    recovery=util[util.index('void usart3_recovery_tick'):util.index('void readCommand')]
    assert 'NVIC_SystemReset' not in recovery and 'USART3_RECOVERY_BEFORE_RESET' not in recovery, 'UART corruption must never escalate to MCU reset'
    assert 'void vesc_protocol_transport_reset(void)' in vp and 's_rx_ok = 0u' not in vp[vp.index('void vesc_protocol_transport_reset(void)'):vp.index('bool vesc_protocol_rx_in_progress')], 'F103 recovery must preserve valid-frame progress counter'
    eeh=(ROOT/'Src/eeprom.h').read_text()
    lds=(ROOT/'STM32F103RCTx_APP.ld').read_text(); bootlds=(ROOT/'STM32F103RCTx_BOOTLOADER.ld').read_text()
    assert '0x0803F000u' in eeh and '0x0803F800u' in eeh and '0x0803FC00u' not in eeh, 'EEPROM must use two distinct 2-KiB xE flash pages'
    assert 'FLASH_PAGE_SIZE != 0x800U' in eeh, 'EEPROM must assert STM32F103xE 2-KiB page size'
    assert re.search(r'#define\s+NB_OF_VAR\s+270u', eeh), 'EEPROM virtual variable count mismatch'
    util=(ROOT/'Src/util.c').read_text(); addrs=[int(x) for x in re.search(r'VirtAddVarTab\[NB_OF_VAR\]\s*=\s*\{([^}]*)\}',util,re.S).group(1).split(',')]; assert len(addrs)==270 and addrs[0]==1000 and addrs[-1]==1269 and addrs==list(range(1000,1270)), 'EEPROM VirtAddVarTab must exactly cover all 270 append-only variables'
    assert 'app_vesc_load_configuration(false)' in util and 'app_vesc_load_configuration(true)' in util, 'App Config EEPROM load hook missing'
    assert re.search(r'#define\s+PAGE1\s+\(\(uint16_t\)0x0001\)', eeh), 'EEPROM PAGE1 logical index must be 1'
    eec=(ROOT/'Src/eeprom.c').read_text()
    assert 'const uint32_t endAddress = Address + PAGE_SIZE - 1u;' in eec, 'EEPROM page erase verification must cover PAGE1 too'
    assert re.search(r'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x8002800,\s*LENGTH\s*=\s*240K', lds), 'application linker must start after 10-KiB bootloader and use the 240-KiB active region'
    assert re.search(r'FLASH\s+\(rx\)\s*:\s*ORIGIN\s*=\s*0x8000000,\s*LENGTH\s*=\s*10K', bootlds), 'bootloader linker must own immutable first 10 KiB'
    mainc=(ROOT/'Src/main.c').read_text()
    assert '!vescLinkActive && !timeoutFlgSerial && enable == 0' in mainc, 'legacy blocking enable handshake must be suppressed while VESC link is armed'
    assert 'SerialFeedback' not in mainc and 'legacyTelemetryPrevMs' not in mainc and 'if (0 &&' not in mainc, 'dead legacy 72-byte telemetry must be removed completely'
    assert 'HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&feedback' not in mainc, 'legacy raw USART3 telemetry must not coexist with VESC transport'
    h=hashlib.sha256((ROOT/'Src/vesc/datatypes.h').read_bytes()).hexdigest()
    assert h == EXPECTED_DATATYPES_SHA256, f'datatypes.h SHA mismatch: {h}'
    # Verify quoted project includes resolve locally, excluding STM32Cube/CMSIS framework includes.
    framework_prefixes=('stm32f1xx','core_cm','cmsis')
    for d in ('Src',):
        for p in (ROOT/d).rglob('*.[ch]'):
            txt=p.read_text(errors='ignore')
            for inc in re.findall(r'#include\s+"([^"]+)"',txt):
                if inc.startswith(framework_prefixes):
                    continue
                candidates=[p.parent/inc, ROOT/inc, ROOT/'Src'/inc, ROOT/'Src/motor'/inc, ROOT/'Src/vesc'/inc]
                assert any(c.exists() for c in candidates), f'unresolved local include {inc} from {p.relative_to(ROOT)}'
    print('STATIC_PROJECT_AUDIT_PASS')
    print('DATATYPES_SHA256',h)

def config_size_check():
    compilers=[c for c in ('gcc','clang') if shutil.which(c)]
    assert compilers, 'gcc/clang unavailable'
    with tempfile.TemporaryDirectory(prefix='vesc-config-size-') as td:
        for cc in compilers:
            out=Path(td)/f'cfg_{cc}'
            run([cc,'-std=c11','-O0','-Wall','-Wextra','-Werror','-I.','-ISrc',
                 'tools/tests/host/test_config_sizes.c','Src/vesc/buffer.c','Src/vesc/mcconf_serial.c','-o',str(out)])
            run([str(out)])
    print('CONFIG_SERIALIZER_GCC_CLANG_PASS')

if __name__ == '__main__':
    check_static()
    py_files=sorted(str(p.relative_to(ROOT)) for p in (ROOT/'tools').rglob('*.py'))
    run([sys.executable,'-m','py_compile',*py_files])
    run([sys.executable,'tools/tests/host/host_compile_check.py'])
    run([sys.executable,'tools/tests/host/test_foc_math.py'])
    run([sys.executable,'tools/tests/host/test_buffer_float_auto.py'])
    run([sys.executable,'tools/tests/host/test_motor_control_v12.py'])
    run([sys.executable,'tools/tests/host/test_motor_control_v13.py'])
    run([sys.executable,'tools/tests/host/test_vesc_protocol_host.py'])
    config_size_check()
    run([sys.executable,'tools/tests/host/test_vesc_dual.py'])
    run([sys.executable,'tools/tests/host/test_vesc_tool_cli.py'])
    run([sys.executable,'tools/tests/host/test_v13_features.py'])
    run([sys.executable,'tools/tests/host/test_v14_features.py'])
    run([sys.executable,'tools/tests/host/test_swd_boot_safety.py'])
    run([sys.executable,'tools/tests/host/test_stlink_update_safety.py'])
    run([sys.executable,'tools/tests/host/test_bootloader_layout.py'])
    run([sys.executable,'tools/tests/host/test_watchdog_runtime_hardening.py'])
    run([sys.executable,'tools/tests/host/test_boot_state_machine_hardening.py'])
    run([sys.executable,'tools/tests/host/test_external_stream_resume.py'])
    run([sys.executable,'tools/tests/host/test_pio_vesc_uploader.py'])
    run([sys.executable,'tools/tests/host/test_pio_vesc_uploader_recovery.py'])
    run([sys.executable,'tools/tests/host/test_boot_handoff_reconnect.py'])
    run([sys.executable,'tools/tests/host/test_v15_features.py'])
    run([sys.executable,'tools/tests/host/test_v16_features.py'])
    run([sys.executable,'tools/tests/host/test_isr_profiler_stage1.py'])
    run([sys.executable,'tools/tests/host/test_comms_isr_isolation_stage2.py'])
    run([sys.executable,'tools/tests/host/test_stage2_production_gate.py'])
    run([sys.executable,'tools/tests/host/test_stage1_audit_hardening.py'])
    run([sys.executable,'tools/tests/host/test_stage2_autotune.py'])
    run([sys.executable,'tools/tests/host/test_measurement_authority_hardening.py'])
    run([sys.executable,'tools/vesc_tool.py','--selftest'])
    run([sys.executable,'tools/tests/host/test_hall_detect_algorithm.py'])
    run([sys.executable,'tools/tests/host/test_hall_3rev_runtime.py'])
    run([sys.executable,'tools/tests/host/test_encoder_abi_runtime.py'])
    run([sys.executable,'tools/tests/host/test_eeprom_persistence.py'])
    print('ALL_FINAL_HOST_CHECKS_PASS')
