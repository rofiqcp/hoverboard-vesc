#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
vp=(ROOT/'Src/vesc/vesc_protocol.c').read_text()
mc=(ROOT/'Src/motor/mcpwm_foc.c').read_text()
ref=(Path('/home/otomasi/agv/referensi/bldc/comm/commands.c').read_text()
     if Path('/home/otomasi/agv/referensi/bldc/comm/commands.c').exists() else '')
# VESC 6.00 wire scaling for realtime commands.
checks={
 'duty':'buffer_get_int32(d, &k) / 100000.0f',
 'current':'buffer_get_int32(d, &k) / 1000.0f',
 'rpm':'buffer_get_int32(d, &k)',
 'pos':'buffer_get_int32(d,&k)/1000000.0f',
 'current_rel':'buffer_get_int32(data, &ind) / 100000.0f',
}
for name,s in checks.items(): assert s in vp, (name,s)
# Full and selective must share exactly one serializer.
assert 'send_values_packet(second, selective, mask);' in vp
assert 'selective ? COMM_GET_VALUES_SELECTIVE : COMM_GET_VALUES' in vp
# GET_VALUES VESC semantics: OFF D/Q/Imotor zero, Ibat independent; RUN Imotor sign from Ibus.
scaled=mc[mc.index('void mcpwm_foc_get_values_scaled'):mc.index('void mcpwm_foc_get_values(',mc.index('void mcpwm_foc_get_values_scaled'))]
assert 'v->current_motor_x100=0;' in scaled and 'v->id_x100=0;' in scaled and 'v->iq_x100=0;' in scaled
assert 'if(ibus_counts>0)im=-im;' in scaled and 'else if(ibus_counts==0)im=0;' not in scaled
assert 'pqi' not in scaled
# Stage-4 VESC input-current mapping: DCL/DCR public Ibat polarity feeds the
# measured-current LPF, and the resulting mapped motor-current ceiling is applied
# only on battery-drawing quadrants in addition to the fast mod_q*Iq limiter.
assert 'input_current_map_update_non_isr' in mc
assert 'target_q4=-(int32_t)m->m_current_in_counts' in mc
assert 'm->m_input_map_current_limit_q4<lim' in mc
assert 'm->m_in_current_map_start_q15<32113u' in mc

# Duty-now must come from actual limited D/Q vector, never command echo.
assert re.search(r'm->m_duty_now_permille\s*=\s*duty_permille_from_vdq\(m->m_vd,m->m_vq\);', mc)
# VESC direction normalization: Iq/duty/RPM/Vq/tacho are direction-relative, Id/Imotor/Ibat are not.
for s in ['dir*v.iq_x100','dir*(int32_t)v.duty_x1000','dir*v.erpm','dir*v.tachometer','dir*v.vq_x1000']:
    assert s in vp, s
for bad in ['dir*v.current_motor_x100','dir*v.current_in_x100','dir*v.id_x100']:
    assert bad not in vp, bad

# Stage 5: VESC PLL / FAST / FASTER selector must remain distinct at runtime.
assert 'case S_PID_SPEED_SRC_FASTER:' in mc
assert 'm_speed_faster_erpm_q16' in mc and 'm_speed_fast_erpm_q16' in mc
assert 'if((uint8_t)next.s_pid_speed_source>(uint8_t)S_PID_SPEED_SRC_FASTER)' in mc
assert 'i<0||i>2' in vp and 'speed_src' in vp

print('VESC_REALTIME_SEMANTICS_PASS')
