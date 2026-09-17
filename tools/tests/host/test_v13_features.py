#!/usr/bin/env python3
from pathlib import Path
import hashlib
R=next(p for p in Path(__file__).resolve().parents if (p/'platformio.ini').exists())
com=(R/'Src/comms.c').read_text()
mc=(R/'Src/motor/mcpwm_foc.c').read_text()
vp=(R/'Src/vesc/vesc_protocol.c').read_text()
main=(R/'Src/main.c').read_text()
cli=(R/'tools/vesc_tool.py').read_text()
dual=(R/'tools/vesc_dual.py').read_text()

# V13 tuning/position parameters tetap ada. Jalur telemetry legacy/LIVE telah
# dihapus; realtime sekarang hanya melalui polling protokol VESC standar.
for name in ('KPQ','KIQ','KPD','KID','KPS','KIS','KDS','KPP','KIP','KDP','PMIN','PMAX','PSETL','PSETR'):
    assert f'"{name}"' in com, f'missing parameter {name}'
assert '"LIVE"' not in com, 'obsolete LIVE parameter must be removed'
assert 'op == "live"' not in com.lower(), 'obsolete live command handler must be removed'
assert '"RESETPOS"' in com, 'position reset command missing'
assert 'int64_t value = 0;' in com, 'parameter averaging is not int64-safe'
assert '2147483648LL' in com and '2147483647LL' in com, 'debug parser does not cover full int32 range'
assert 'positionMinUser = INT32_MIN' in com and 'positionMaxUser = INT32_MAX' in com
assert 'mcpwm_foc_set_position_user_limits(positionMinUser, positionMaxUser, false)' in com
assert 'mcpwm_foc_set_position_user_limits(positionMinUser, positionMaxUser, true)' in com
assert 'user_position_to_internal' in mc and 'positionCommandR' in mc
assert 'CONTROL_MODE_POS' in mc and 'm_position_target_counts' in mc
assert 'position_pid_iq_target_step' in mc, 'position PID must feed Iq target'
assert 'm_current_kpq_v_q16' in mc and 'm_current_kiq_dt_v_q16' in mc and 'm_current_kpd_v_q16' in mc and 'm_current_kid_dt_v_q16' in mc
assert 'm->m_kps_q11' in mc and 'm->m_kis_q16' in mc and 'm->m_kds_q11' in mc and 'speed_pid_iq_target_step' in mc
assert 'm->m_kpp_q11' in mc and 'm->m_kip_q16' in mc and 'm->m_kdp_q11' in mc
assert 'MCCONF_STEERING_POSITION_CURRENT_MAX_MA  10000u' in (R/'Src/motor/mcconf_default.h').read_text(), 'steering current ceiling regression must match the active 10.0 A steering runtime limit; global hard current remains 15 A'
assert 'MCCONF_STEERING_POSITION_KP_MULTIPLIER' not in (R/'Src/motor/mcconf_default.h').read_text(), 'hidden steering Kp multiplier must stay removed'
assert 'const int32_t dc_foc=m->m_conf.foc_encoder_inverted?-dc:dc;' in mc, 'ABI RPM estimator must apply encoder inversion exactly once'
assert 'encoder_count_mode && m->m_conf.foc_encoder_inverted' not in mc, 'process-D must not double-apply encoder inversion after RPM correction'
assert 'SerialFeedback' not in main and 'legacyTelemetryPrevMs' not in main, 'dead legacy telemetry must stay removed'
assert 'USART3 hanya membawa protokol VESC' in main, 'VESC-exclusive USART3 rationale missing'
assert 'case COMM_SET_POS:' in vp and 'COMM_FORWARD_CAN' in vp and 'COMM_PING_CAN' in vp
assert 'Right power stage is physically mirrored' not in vp and 'right_sign' not in vp, 'virtual-right protocol must not rewrite VESC coordinates'
assert not (R/'tools/hoverserial.py').exists(), 'legacy HoverSerial tool must stay removed'
assert 'poscount reset' in cli and 'poscount limits' in cli and 'telemetry on' in cli
assert 'COMM_SET_POS = 9' in dual and 'def set_pos(' in dual and 'RIGHT_ID = 2' in dual
# vesc/datatypes.h must remain exact reference, never custom-extended for these parameters.
h=hashlib.sha256((R/'Src/vesc/datatypes.h').read_bytes()).hexdigest()
assert h=='4ecae1f31c12c1ab415d47dd997396d0792e94249203cbeb877ada75f76d5340', h
print('V13_FEATURE_STATIC_PASS full_int32=1 live_toggle=removed host_polling=1 pos=1 separate_dq=1 can2=1 datatypes_exact=1')
