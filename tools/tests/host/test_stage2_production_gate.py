#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[3]
def text(p): return (R/p).read_text(errors='ignore')
model=text('tools/tests/hardware/qualify_motor_model.py'); tuner=text('tools/tests/hardware/tune_foc_staged.py')
adc=text('tools/tests/hardware/profile_adc_envelope.py'); campaign=text('tools/tests/hardware/tuning_campaign_10x5.py')
ctrl=text('tools/tests/hardware/qualify_control_modes.py'); posd=text('tools/tests/hardware/qualify_position_process_d.py')
stlink=text('tools/pio_stlink_upload.py')
main=text('Src/main.c'); vp=text('Src/vesc/vesc_protocol.c'); dual=text('tools/vesc_dual.py')
# Every motor-moving qualification path is explicit opt-in.
for name,s in [('model',model),('tuner',tuner),('10x5',campaign),('control_modes',ctrl),('position_d',posd)]:
    assert "--arm" in s and 'ARM_REQUIRED' in s, f'{name} lost explicit arm gate'
# Model identification must be non-persistent; Detect-All is not used by this tool.
assert 'measure_r_l(' in model and 'measure_flux_openloop(' in model
assert 'detect_all_foc(' not in model and 'save mcconf' not in model and 'store=True' not in model
assert 'force_dq_equal' in model and 'cv' in model
# FOC selection authority is synchronized raw trace, never host values polling.
assert 'arm_current_step(' in tuner and 'download_trace()' in tuner and 'isr_profile(' in tuner
assert '.values(' not in tuner and 'sleep(0.008)' not in tuner
assert 'kpq=quantize_u16(kp,1536),kpd=quantize_u16(kp,1536)' in tuner and 'kiq=quantize_u16(ki,4.608),kid=quantize_u16(ki,4.608)' in tuner
assert "--store-best" in tuner and 'persistence_started=True' in tuner, 'persistent tuning must require a separate explicit flag'
assert 'set_tuning(originals[right],right,store=True)' in tuner, 'persistent tuning failure must roll back originals'
assert 'actual_pre_a' in tuner and 'actual_step_a' in tuner, 'tuner must score actual firmware-clamped current'
assert "u32_delta(p0['dma_tc_pending_exit'],prof['dma_tc_pending_exit'])" in tuner, 'tuner must gate on monotonic DMA delta'
# ADC envelope collector is observational only.
assert 'adc_validity(' in adc and 'values(' in adc
for forbidden in ('set_current(', 'set_rpm_one(', 'set_duty(', 'set_pos_one(', 'set_tuning('): assert forbidden not in adc
# Integrated 10x5 is qualification-only and cannot mutate/store gains.
for forbidden in ('set_tuning(', 'save mcconf', 'store=True', 'CANDS='): assert forbidden not in campaign
assert 'for rep in range(1,6)' in campaign and '--repeat must remain 5' in campaign
# A/B modes are RAM-only and restored; process-D uses direct internal observability.
assert "set_ram(v,right,'decoupling'" in ctrl and "set_ram(v,right,'speed_src'" in ctrl and "report['restored']" in ctrl
assert 'position_d_state(' in posd and 'delta*d[\'dproc_q15\']<0' in posd
assert 'HB_CUSTOM_GET_POSITION_D_STATE' in vp and 'm_position_d_proc_filter_q15' in vp
assert 'HB_GET_POSITION_D_STATE' in dual and 'def position_d_state' in dual
mc=text('Src/motor/mcpwm_foc.c')
assert 'delta_mdeg*=position_error_sign(m,second)' in mc and 'const int64_t num=-(int64_t)delta_mdeg' in mc, 'process-D must oppose logical measured motion'
# Production ST-Link uploader has no under-reset path and runtime refreshes SWJ.
assert 'connect_assert_srst' not in stlink and 'verify_f103_target' not in stlink
assert 'flash write_image erase' in stlink and 'verify_image' in stlink and 'reset run' in stlink
assert 'f103_debug_keepalive();' in main and 'debugKeepalivePrevMs' in main
# Production motor transport is direct F103 USB-UART only. Gateway-era routes
# must never return to active source/tooling. Historical result CSVs and recovery
# backups are intentionally outside this gate.
for rel in ('tools/vesc_dual.py','tools/pio_vesc_upload.py','platformio.ini','BOOTLOADER_AND_UPLOAD.md',
            'Src/vesc/f103_boot_layout.h','Src/vesc/vesc_protocol.c','Src/vesc/flash_update_f103.c','Src/config.h'):
    live=text(rel)
    for forbidden in ('F411','stmf4','65101','APP_F411','python-maintenance','VESC_F411_USB','f411_direct'):
        assert forbidden not in live, f'legacy gateway token {forbidden} returned in {rel}'
assert '_direct_serial_candidates' in dual and '_probe_f103_uart' in dual
uploader=text('tools/pio_vesc_upload.py')
assert "choices=['serial']" in uploader and 'reconnect_transport()' in uploader

# Transport corruption cannot reboot MCU.
util=text('Src/util.c'); rec=util[util.index('void usart3_recovery_tick'):util.index('void readCommand')]
assert 'NVIC_SystemReset' not in rec
print('STAGE2_PRODUCTION_GATE_PASS trace_authority=1 model_repeatability=1 qualification_only=1 standard_stlink=1 process_d_observable=1')
