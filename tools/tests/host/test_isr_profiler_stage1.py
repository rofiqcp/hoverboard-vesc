#!/usr/bin/env python3
from pathlib import Path
import ast, math, re
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
mh=(R/'Src/motor/mcpwm_foc.h').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
dual=(R/'tools/vesc_dual.py').read_text()
assert '#define FOC_PROF_SAMPLE_PERIOD 31u' in mc and math.gcd(31,6)==1
assert '#define MCPWM_FOC_ISR_PROFILE_REVISION  0x00030000u' in mh
assert 'const uint32_t focIsrStartCycles = DWT->CYCCNT;' in mc
assert 'const uint32_t end = DWT->CYCCNT;' in mc
assert 'elapsed > FOC_ISR_BUDGET_CYCLES' in mc and 'foc_isr_deadline_miss_count++' in mc
assert 'DMA1->ISR & DMA_ISR_TCIF1' in mc and 'foc_irq_dma_tc_pending_exit_count++' in mc
assert 'foc_isr_steady_count++' in mc and 'foc_isr_slot_sequence_error_count++' in mc
assert 'foc_prof_detail_slot_count[isrSlot]++' in mc
assert '% FOC_PROF_SAMPLE_PERIOD' not in mc
writers={
 'sensor':'foc_prof_sensor_max_cycles=used', 'pll':'foc_prof_pll_max_cycles=used',
 'current':'foc_prof_current_max_cycles=used', 'regulator':'foc_prof_regulator_max_cycles=used',
 'id_pi':'foc_prof_id_pi_max_cycles=used', 'iq_pi':'foc_prof_iq_pi_max_cycles=used',
 'current_circle':'foc_prof_current_circle_max_cycles=used',
 'decouple_limit':'foc_prof_decouple_limit_max_cycles=used',
 'main_svpwm':'foc_prof_svpwm_max_cycles=used',
 'fast_hold_svpwm':'foc_prof_fast_hold_svpwm_max_cycles=used',
 'duty_mag':'foc_prof_duty_mag_max_cycles=used',
 'speed_pid':'foc_prof_speed_pid_max_cycles=profSpeedUsed',
 'position_pid':'foc_prof_position_pid_max_cycles=profPosUsed'}
for name,pat in writers.items(): assert pat in mc, f'missing real writer for {name}'
assert mc.count('foc_prof_current_circle_max_cycles=used')>=2
assert mc.count('foc_prof_iq_pi_max_cycles=used')>=2
assert 'if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;' in mc
assert 'if(foc_prof_detail_sample && control_update)profRegulatorStart=DWT->CYCCNT;' in mc
assert 'uint32_t detail_sample_count, detail_slot_count[6];' in mh
assert 'uint32_t steady_isr_count, slot_sequence_error_count;' in mh
assert 'uint32_t active_slot_count, reset_epoch;' in mh
assert 'foc_prof_reset_request' in mc and 'foc_prof_reset_ack' in mc and 'foc_isr_profile_clear_isr_owned' in mc
assert 'uint32_t fast_hold_svpwm_max_cycles, profile_revision;' in mh
assert 'uint8_t b[320]' in vp and 'APPP(p.fast_hold_svpwm_max_cycles)' in vp
assert 'APPP(p.slot_sequence_error_count)' in vp and 'APPP(p.steady_isr_count)' in vp
assert 'for(uint8_t si=0u;si<6u;++si)APPP(p.detail_slot_count[si]);' in vp
s=dual.index('    def isr_profile'); e=dual.index('    def trace_meta',s); body=dual[s:e]
m=re.search(r'names=\((.*?)\)\n\s*p=self',body,re.S); names=ast.literal_eval('('+m.group(1)+')')
for n in ('fast_hold_svpwm','slot_sequence_errors','steady_isr_count','detail_slot0_count','profile_revision','active_slot_count','reset_epoch'):
    assert n in names
assert len(names)==72 and 6+4*len(names)==294
print('ISR_PROFILER_STAGE1_STATIC_PASS revision=0x00030000 continuous_authority=1 coherent_reset=1 dynamic_active_slots=1 sampled_detail=1 sample_period=31 slot_capacity=6 fields=72 long_frame=294')
