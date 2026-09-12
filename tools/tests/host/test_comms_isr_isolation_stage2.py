#!/usr/bin/env python3
from pathlib import Path
import re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mh=(R/'Src/motor/mcpwm_foc.h').read_text()
it=(R/'Src/stm32f1xx_it.c').read_text()
setup=(R/'Src/setup.c').read_text()
main=(R/'Src/main.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()

# Protocol parsing/queues run only in main context; communication must never mask FOC.
assert '__disable_irq()' not in vp and '__enable_irq()' not in vp
assert 'for (uint8_t processed = 0u; processed < 2u; ++processed)' in vp
assert 'VESC_RX_QUEUE_DEPTH          16u' in vp and 'VESC_TX_QUEUE_DEPTH          8u' in vp
assert 'HAL_UART_Transmit_DMA(&huart3' in vp and 'HAL_UART_Transmit(&huart3' not in vp
assert 's_rt_cmd_coalesced++' in vp and 's_rx_queue_drop++' in vp and 's_tx_queue_drop++' in vp
assert 's_tx_queue_highwater' in vp and 's_rx_queue_highwater' in vp
# ISR-owned telemetry snapshots are retry-coherent without globally masking IRQ.
assert 'void mcpwm_foc_get_irq_epoch' in mc and 'void mcpwm_foc_get_irq_epoch' in mh
assert 'foc_telem_isr_snapshot' in mc and 'if(e0==e1 && x0==x1 && e1==x1)break;' in mc
scaled=mc[mc.index('void mcpwm_foc_get_values_scaled'):mc.index('void mcpwm_foc_get_values(',mc.index('void mcpwm_foc_get_values_scaled'))]
vals=mc[mc.index('void mcpwm_foc_get_values('):]
assert '__disable_irq()' not in scaled
assert '__disable_irq()' not in vals[:vals.find('\nfloat mcpwm_foc_',1)]
assert 'foc_telem_consume_main' in scaled and 'foc_telem_consume_main' in vals
speed=mc[mc.index('static void encoder_feedback_finalize_speed_non_isr'):mc.index('static void encoder_tachometer_update_non_isr')]
assert speed.index('if(m->m_encoder_speed_ticks<MCCONF_ENCODER_SPEED_WINDOW_TICKS)return;') < speed.index('__disable_irq()')
tacho=mc[mc.index('static void encoder_tachometer_update_non_isr'):mc.index('static int16_t duty_permille_from_vdq',mc.index('static void encoder_tachometer_update_non_isr'))]
assert '__disable_irq()' not in tacho and 'mcpwm_foc_get_irq_epoch' in tacho

# UART IRQ only acknowledges IDLE; DMA parsing stays in main context.
uart_irq=it[it.index('void f103_USART3_IRQHandler_impl'):it.index('/******************************************************************************/',it.index('void f103_USART3_IRQHandler_impl'))]
assert 'usart3_rx_check()' not in re.sub(r'/\*.*?\*/','',uart_irq,flags=re.S)
assert '__HAL_UART_CLEAR_IDLEFLAG' in uart_irq
assert 'usart3_rx_check();\n    vesc_protocol_process_pending();' in main
assert main.index('mcpwm_foc_outer_control_non_isr') < main.index('usart3_rx_check();')

# Priority hierarchy keeps ADC/FOC above UART RX and TX completion.
assert 'HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);' in setup
assert 'HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 1, 0);' in setup
assert 'HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);' in setup
assert 'HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);' in setup
# Health ABI exposes queue pressure, UART recovery and main-loop timing.
assert '#define HB_CUSTOM_GET_COMMS_HEALTH                 24u' in vp
for token in ('s_rx_queue_drop','s_rx_queue_highwater','s_rt_cmd_coalesced','s_tx_queue_drop',
              's_tx_start_fail','s_tx_queue_highwater','s_process_gap_max_ms','usart3_forced_recovery_count()',
              'main_prof_vesc_max_cycles','main_prof_house_max_cycles','main_prof_tail_max_cycles'):
    assert token in vp, token
assert 'HB_GET_COMMS_HEALTH = 24' in dual and 'def comms_health(self)' in dual

# Profile coherent snapshot also rejects a read made while the IRQ itself is active.
prof=mc[mc.index('void mcpwm_foc_get_isr_profile'):mc.index('/* Normalisasi fitur',mc.index('void mcpwm_foc_get_isr_profile'))]
assert 'if(e0!=x0)continue;' in prof
assert 'if(e0==e1 && x0==x1 && e1==x1)break;' in prof
print('COMMS_ISR_ISOLATION_STAGE2_STATIC_PASS protocol_irq_mask=0 rx_depth=16 tx_depth=8 batch=2 adc_prio=0 uart_rx_prio=1 tx_prio=2 coherent_snapshots=1 health_op=24')
