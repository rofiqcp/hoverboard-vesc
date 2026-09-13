#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[3]
def text(rel): return (R/rel).read_text(errors='ignore')
mc=text('Src/motor/mcpwm_foc.c')
vp=text('Src/vesc/vesc_protocol.c')
dual=text('tools/vesc_dual.py')
tuner=text('tools/tune_foc_staged.py')
ctrl=text('tools/qualify_control_modes.py')
posd=text('tools/qualify_position_process_d.py')
campaign=text('tools/tuning_campaign_10x5.py')
model=text('tools/qualify_motor_model.py')
main=text('Src/main.c'); boot=text('Src/bootloader/main.c')

# Step trace is capacity-safe and begins only after powered-offset/current sampling settles.
assert 'requested>MCPWM_FOC_TRACE_CAPACITY' in mc
assert 'settling' in mc and 'settle_remaining=255u' in mc
for token in ('m_current_offset_valid','m_driven_offset_valid','!sm->m_driven_offset_calibrating',
              'm_bridge_settle_ticks==0u','m_sample_window_valid'):
    assert token in mc, token
ready=mc[mc.index('const bool ready='):mc.index('uint32_t profMotorStart', mc.index('const bool ready='))]
assert 'foc_trace_clear_isr_owned();' in ready and 's_foc_step_test.settling=0u' in ready
assert 'pre_samples+post_samples<=40' in dual

# Trace timing must belong to the same IRQ frame, not the prior frame.
assert 'foc_trace_finalize_isr_cycles(uint32_t elapsed)' in mc
monitor=mc[mc.index('static void foc_isr_monitor_end'):mc.index('static void foc_isr_profile_clear_isr_owned')]
assert 'foc_trace_finalize_isr_cycles(elapsed);' in monitor
capture=mc[mc.index('static void foc_trace_capture_internal'):mc.index('#define OPENLOOP_ERPM_Q16_TO_PHASE')]
assert 't->isr_cycles=0u' in capture and 's_foc_trace_cycle_pending=1u' in capture

# Monotonic DMA evidence is always checked as a per-run delta.
for name,src in [('tuner',tuner),('control',ctrl),('position_d',posd),('10x5',campaign)]:
    assert 'def d32(' in src, name
    assert "dma_tc_pending_exit" in src and 'd32(' in src, name
assert 'dma_tc_pending_exit_delta' in tuner

# Tuning is authority-safe: actual clamped step, signed-step metric, rollback, delayed persistence.
assert 'actual_pre_a' in tuner and 'actual_step_a' in tuner
assert 'overs=max(0.0,(max(norm)-1.0)*100.0)' in tuner
assert 'max([-x for x in norm])' not in tuner
assert 'persistence_started=True' in tuner
assert 'set_tuning(originals[right],right,store=True)' in tuner
assert 'pre_samples+a.post_samples>40' in tuner

# Bounded F103 flux worker does not expose a fake duty tuning knob.
assert "add_argument('--duty'" not in model

# Persistent writes cannot ACK success after their commit barrier reports failure.
mcconf=vp[vp.index('static void set_mcconf('):vp.index('static void reply_appconf(')]
assert 'if(mc_interface_store_configuration_motor(second))' in mcconf
assert 'if (committed)' in mcconf
appconf=vp[vp.index('static void set_appconf('):vp.index('static void set_current_relative(')]
assert 'ok=app_vesc_store_configuration(second)' in appconf and 'if(!ok)' in appconf
bcut=vp[vp.index('static void set_battery_cut('):vp.index('/** Kirim konfigurasi limit sementara')]
assert 'if(ok){uint8_t ack=COMM_SET_BATTERY_CUT' in bcut
mct=vp[vp.index('static void set_mcconf_temp('):vp.index('static void reply_decoded_adc(')]
assert 'if (ack && ok)' in mct
custom=vp[vp.index('if (op == HB_CUSTOM_GET_TUNING || op == HB_CUSTOM_SET_TUNING)'):vp.index('if (op == HB_CUSTOM_SET_ID_TEST)')]
assert 'tuning_status=3u' in custom and '!mc_interface_store_configuration_motor(second)' in custom

# SWD is enforced before HAL/peripheral setup and continuously in resident recovery.
for name,src in [('app',main),('boot',boot)]:
    assert 'AFIO_MAPR_SWJ_CFG_Msk' in src and 'AFIO_MAPR_SWJ_CFG_RESET' in src, name
    assert 'DBGMCU_CR_DBG_IWDG_STOP' in src, name
assert '(uint32_t)(now-debug_keepalive) >= 25u' in boot
print('MEASUREMENT_AUTHORITY_HARDENING_PASS settle_gate=1 trace_capacity=40 same_frame_timing=1 dma_delta=4 transactional_tuning=1 persistence_ack=fail_closed swd_early=1')
