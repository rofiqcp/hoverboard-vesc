#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
vp=(ROOT/'Src/vesc/vesc_protocol.c').read_text()
mc=(ROOT/'Src/motor/mcpwm_foc.c').read_text()
mci=(ROOT/'Src/motor/mc_interface.c').read_text()
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
assert 'speed_src 0PLL|1FAST|2FASTER' in vp
assert 'c->s_pid_speed_source = S_PID_SPEED_SRC_PLL;' in mc
assert 'measured_mech_rpm_public_q16' in mc and 'speed_pid_mech_rpm_q16' in mc
assert 'foc_pll_update_fixed(m,second,control_update);' in mc and 'pll_needed' not in mc
assert 'fast3_to_step_q20' in mc and 'm_speed_fast_erpm_q16' in mc
assert 'return m ? (float)m->m_pll_erpm_q16/65536.0f : 0.0f;' in mc
assert 'COMM_GET_VALUES uses the same FOC RPM authority as upstream' in mc
assert 'v->erpm=foc_telem_public_erpm(&is,second);' in mc
assert 'if(!sensor_ready && !synthetic_phase)' in mc
assert 'CONTROL_MODE_OPENLOOP_PHASE' in mc and 'CONTROL_MODE_OPENLOOP' in mc
assert 'return is ? (is->pll_erpm_q16/65536) : 0;' in mc
assert 'No Hall/ABI fallback is exposed here' in mc
serial=(ROOT/'Src/vesc/mcconf_serial.c').read_text()
assert 'buffer[ind++] = conf->s_pid_speed_source;' in serial
assert 'conf->s_pid_speed_source = buffer[ind++];' in serial
assert 'buffer_append_float16(buffer, conf->foc_fw_backoff, 1000, &ind);' in serial
assert 'conf->foc_fw_backoff = buffer_get_float16(buffer, 1000, &ind);' in serial
assert 'buffer_append_float32_auto(buffer, conf->foc_sl_erpm_start, &ind);' in serial
assert 'conf->foc_sl_erpm_start = buffer_get_float32_auto(buffer, &ind);' in serial
assert 'buffer[ind++] = conf->foc_control_sample_mode;' in serial
assert 'buffer[ind++] = conf->foc_current_sample_mode;' in serial
assert 'conf->foc_control_sample_mode = (mc_foc_control_sample_mode)buffer[ind++];' in serial
assert 'conf->foc_current_sample_mode = (mc_foc_current_sample_mode)buffer[ind++];' in serial
assert 'm->m_conf.s_pid_speed_source=S_PID_SPEED_SRC_PLL;' in mci

# Stage 5 MC-config wire order: presence alone is insufficient because one
# omitted byte shifts every following VESC Tool field. Assert the exact local
# order of the FOC/HFI/FW/speed-PID transition block against VESC 6.00.
wire_order=[
 'conf->foc_hall_interp_erpm','conf->foc_sl_erpm_start','conf->foc_sl_erpm',
 'conf->foc_control_sample_mode','conf->foc_current_sample_mode','conf->foc_sat_comp_mode',
 'conf->foc_sat_comp','conf->foc_temp_comp','conf->foc_temp_comp_base_temp',
 'conf->foc_current_filter_const','conf->foc_cc_decoupling','conf->foc_observer_type',
 'conf->foc_hfi_amb_mode','conf->foc_hfi_amb_current',
 'conf->foc_hfi_amb_tres','conf->foc_hfi_voltage_start','conf->foc_hfi_voltage_run',
 'conf->foc_hfi_voltage_max','conf->foc_hfi_gain','conf->foc_hfi_max_err',
 'conf->foc_hfi_hyst','conf->foc_sl_erpm_hfi','conf->foc_hfi_reset_erpm',
 'conf->foc_hfi_start_samples','conf->foc_hfi_obs_ovr_sec','conf->foc_hfi_samples',
 'conf->foc_offsets_cal_mode','conf->foc_offsets_current','conf->foc_offsets_voltage',
 'conf->foc_offsets_voltage_undriven','conf->foc_phase_filter_enable',
 'conf->foc_phase_filter_disable_fault','conf->foc_phase_filter_max_erpm',
 'conf->foc_mtpa_mode','conf->foc_fw_current_max','conf->foc_fw_duty_start',
 'conf->foc_fw_ramp_time','conf->foc_fw_q_current_factor','conf->foc_fw_backoff',
 'conf->foc_speed_soure','conf->foc_short_ls_on_zero_duty',
 'conf->foc_overmod_factor','conf->foc_mag_vd_max','conf->sp_pid_loop_rate',
 'conf->s_pid_kp','conf->s_pid_ki','conf->s_pid_kd','conf->s_pid_kd_filter',
 'conf->s_pid_min_erpm','conf->s_pid_allow_braking','conf->s_pid_ramp_erpms_s',
 'conf->s_pid_speed_source']
ser_block=serial[serial.index('int32_t confgenerator_serialize_mcconf'):serial.index('bool confgenerator_deserialize_mcconf')]
des_block=serial[serial.index('bool confgenerator_deserialize_mcconf'):serial.index('bool confgenerator_deserialize_appconf')]
for name,block in [('serialize',ser_block),('deserialize',des_block)]:
    pos=-1
    for tok in wire_order:
        nxt=block.find(tok,pos+1)
        assert nxt>pos, ('mcconf '+name+' wire order',tok,pos,nxt)
        pos=nxt

print('VESC_REALTIME_SEMANTICS_PASS')
