#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
vp=(ROOT/'Src/vesc/vesc_protocol.c').read_text()
mc=(ROOT/'Src/motor/mcpwm_foc.c').read_text()
mci=(ROOT/'Src/motor/mc_interface.c').read_text()
serial=(ROOT/'Src/vesc/mcconf_serial.c').read_text()
serial_h=(ROOT/'Src/vesc/mcconf_serial.h').read_text()
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

# Stage 5: VESC PLL / FAST / FASTER ownership must remain distinct.
# Upstream default and invalid/migration fallback are PLL.
assert 'c->s_pid_speed_source = S_PID_SPEED_SRC_PLL;' in mc
assert 'next.s_pid_speed_source=S_PID_SPEED_SRC_PLL;' in mc
assert 'm->m_conf.s_pid_speed_source=S_PID_SPEED_SRC_PLL;' in mci
# Only speed PID follows the selector. Public RPM is PLL, FAST/FASTER stay
# alternate speed-loop estimators, decoupling/FW stay tied to FAST upstream.
sp=mc[mc.index('static int32_t speed_pid_mech_rpm_q16'):mc.index('static bool encoder_motion_fresh')]
pub=mc[mc.index('static int32_t measured_mech_rpm_public_q16'):mc.index('static int32_t speed_pid_mech_rpm_q16')]
dec=mc[mc.index('static int32_t motor_erpm_for_decoupling'):mc.index('static void current_decoupling_apply')]
assert 'switch(m->m_conf.s_pid_speed_source)' in sp and 'case S_PID_SPEED_SRC_FASTER:' in sp
assert 's_pid_speed_source' not in pub and 'm->m_pll_valid' in pub
assert 'm->m_speed_est_valid' in dec and 'm->m_speed_fast_erpm_q16' in dec
assert 'motor_erpm_for_decoupling(m,second)' in mc[mc.index('static void field_weakening_update_non_isr'):mc.index('void mcpwm_foc_outer_control_non_isr')]
# PLL runs continuously; FAST mode must never turn it off.
assert 'pll_needed' not in mc
assert 'foc_pll_update_fixed(m,second,control_update);' in mc
assert 'if(is.pll_valid)v->erpm=is.pll_erpm_q16/65536;' in mc
assert 'if(is.pll_valid)' in mc and 'v->rpm=(float)is.pll_erpm_q16/65536.0f;' in mc
assert 'if((uint8_t)next.s_pid_speed_source>(uint8_t)S_PID_SPEED_SRC_FASTER)' in mc
assert 'i<0||i>2' in vp and 'speed_src' in vp and 'speed_est' in vp
# Wire schema is intentionally VESC-6 legacy. Do not splice fields from a newer
# confgenerator without changing the signature/tool compatibility as a unit.
assert '#define MCCONF_SIGNATURE 776184161u' in serial_h
assert 'conf->s_pid_speed_source' not in serial
assert 'GPD fields existed in 6.00' in serial
# SET_MCCONF must preserve local extension fields that are absent from the 6.00
# payload by deserializing on top of the current motor configuration.
setmc=vp[vp.index('static void set_mcconf(bool second'):vp.index('static void reply_appconf')]
assert '*backup = *mc_interface_get_configuration_motor(second);' in setmc
assert '*c = *backup;' in setmc
assert setmc.index('*c = *backup;') < setmc.index('confgenerator_deserialize_mcconf(data, c)')

print('VESC_REALTIME_SEMANTICS_PASS')
