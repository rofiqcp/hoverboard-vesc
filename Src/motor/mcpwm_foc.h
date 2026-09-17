#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include <stdint.h>
#include <stdbool.h>
#include "vesc/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Konvensi dual-motor mengikuti VESC: is_second_motor=false berarti
 * motor 1/LEFT, is_second_motor=true berarti motor 2/RIGHT. Pada hardware ini
 * LEFT dapat Hall atau ABI; RIGHT dikunci Hall-only. */

typedef struct {
    mc_configuration m_conf;
    volatile mc_state m_state;
    volatile mc_control_mode m_control_mode;
    /* Normal user STOP / zero-command sensing standby. Unlike a safety release,
     * FOC remains powered with Id*=Iq*=0 so phase-current sensing stays valid
     * during rotor coast. The current PI can generate Vd/Vq to cancel BEMF; once
     * stationary its zero-current equilibrium returns close to centered PWM. */
    volatile uint8_t m_standby_sense;
    volatile mc_fault_code m_fault;
    /* Main-context MC config publication can span many float/cache calculations.
     * While this flag is set the ADC ISR must never read the partially published
     * configuration or re-arm MOE. */
    volatile uint8_t m_config_update_active;
    volatile uint32_t m_fault_recovery_ticks;
    volatile uint32_t m_fault_safe_ticks; /* safe-condition dwell before automatic clear */
    uint32_t m_fault_stop_ticks; /* m_fault_stop_time_ms -> tick PWM, dihitung di slow path */
    /* Safety runtime per motor. Semua threshold mahal dihitung saat config
     * berubah; ISR hanya melakukan compare/counter integer. */
    volatile uint8_t m_current_offset_valid;
    volatile uint8_t m_overspeed_streak;
    uint32_t m_abs_erpm_fault;

    /* VESC-style setpoints. Fixed-point values are authoritative in the ISR. */
    volatile int16_t m_iq_set_q4;       /* slewed/active Iq reference */
    volatile int16_t m_iq_target_q4;    /* requested Iq reference */
    volatile int16_t m_id_set_q4;
    /* VESC field-weakening state. m_i_fw_set_q4 is a positive magnitude;
     * the closed-loop D-axis target is -m_i_fw_set_q4. Coefficients are
     * precomputed outside the ISR from standard foc_fw_* MC configuration. */
    volatile int16_t m_i_fw_set_q4;
    /* VESC m_current_off_delay equivalent in milliseconds. FW refreshes this
     * to 1000 ms while active; normal outer-loop ticks count it down. It keeps
     * zero-current modulation alive after leaving FW so BEMF/body-diode
     * transients cannot abruptly collapse the switching state. */
    volatile uint16_t m_current_off_delay_ms;
    int16_t m_fw_current_max_q4;
    uint16_t m_fw_duty_start_permille;
    uint16_t m_fw_q_current_factor_q15;
    uint32_t m_fw_backoff_q15;
    uint32_t m_fw_ramp_time_ms;
    volatile int16_t m_speed_set_rpm;       /* active/slewed mechanical RPM */
    volatile int16_t m_speed_target_rpm;    /* requested mechanical RPM, integer view */
    volatile int32_t m_speed_target_rpm_q16; /* authoritative requested mechanical RPM Q16 */
    volatile int16_t m_duty_set_permille;      /* requested VESC duty */
    int32_t m_duty_i_q15;
    uint32_t m_duty_kp_q12_per_permille;
    uint32_t m_duty_ki_q12_per_permille;
    uint32_t m_duty_kp_base_q12_x100; /* numerator gain untuk pembagian Vin di slow path */
    uint32_t m_duty_ki_base_q12_x100; /* numerator Ki*dt untuk pembagian Vin */
    uint8_t m_duty_pi_active;

    volatile uint16_t m_kpq_q11, m_kiq_q16;
    volatile uint16_t m_kpd_q11, m_kid_q16;
    /* VESC current PI coefficients in physical-voltage fixed point. Kp is
     * V/A Q16; Ki_dt is V/A per regulator step Q16. These make the ISR
     * implement vd_int += Ierr*Ki*dt; vd = vd_int + Ierr*Kp exactly. */
    uint32_t m_current_kpq_v_q16, m_current_kiq_dt_v_q16;
    uint32_t m_current_kpd_v_q16, m_current_kid_dt_v_q16;
    /* Precomputed VESC physical PI gains per Q4-current-count, Q8 scaling.
     * ISR uses multiply+shift only; no Cortex-M3 software 64-bit division. */
    uint32_t m_current_kpq_err_q8, m_current_kiq_err_q8;
    uint32_t m_current_kpd_err_q8, m_current_kid_err_q8;
    volatile uint16_t m_kps_q11, m_kis_q16, m_kds_q11;
    /* Precomputed speed-PID coefficients. Configuration may use float, but the
     * 16-kHz ISR executes multiply+shift only (no __aeabi_ldivmod). */
    uint32_t m_speed_kp_coeff_q16;
    uint32_t m_speed_ki_coeff_q16;
    uint32_t m_speed_kd_coeff_q8;
    uint16_t m_speed_kd_filter_q16;      /* alpha LPF D speed, standar VESC */
    int32_t m_speed_d_filter_q4;         /* state D terfilter dalam arus Q4 */
    volatile uint16_t m_kpp_q11, m_kip_q16, m_kdp_q11;
    /* Extension posisi multi-putaran proyek. Pada Hall satu count = satu edge
     * Hall; pada ABI satu count = satu quadrature count. Jangan gunakan field
     * ini sebagai tachometer wire VESC. */
    volatile int32_t m_position_counts;
    volatile uint32_t m_position_abs_counts;
    /* Tachometer standar VESC: selalu resolusi 60 derajat elektrik, terlepas
     * dari sumber rotor Hall atau encoder dan terlepas dari CPR encoder. */
    volatile int32_t m_tachometer;
    volatile uint32_t m_tachometer_abs;
    uint8_t m_tacho_step_last;
    volatile int32_t m_position_target_counts;
    volatile int32_t m_position_pid_target_counts; /* slew-limited target actually used by PID */
    int32_t m_position_target_ramp_q16;
    uint32_t m_position_target_ramp_step_q16;
    volatile int32_t m_position_min_counts;
    volatile int32_t m_position_max_counts;
    /* LEFT steering calibration. span is signed user-direction count travel
     * from physical left stop (-30 deg) to right stop (+30 deg). */
    volatile int32_t m_steering_span_counts;
    volatile uint8_t m_steering_calibrated;
    volatile uint8_t m_steering_homed;
    /* Stock VESC COMM_SET_POS is single-turn electrical rotor position. Keep it
     * separate from this project's long-range Hall-count position extension. */
    volatile uint16_t m_pos_pid_set_phase;
    volatile uint8_t m_pos_pid_phase_mode;
    /* VESC p_pid_ang_div tracking. feedback_phase adalah m_pos_pid_now internal
     * sebelum mc_interface menerapkan inversion/direction/p_pid_offset. */
    volatile uint16_t m_pos_pid_feedback_phase;
    uint16_t m_pos_pid_raw_last;
    uint16_t m_pos_pid_div_accum;
    uint32_t m_pos_pid_ang_div_inv_q16; /* (1/p_pid_ang_div)*65536 */
    volatile int16_t m_current_limit_q4;      /* batas Iq motoring (+), Q4 */
    volatile int16_t m_current_limit_neg_q4;  /* magnitudo batas Iq regen/brake (-), Q4 */
    volatile uint16_t m_battery_cut_start_adc;/* awal derating baterai dalam hitungan ADC */
    volatile uint16_t m_battery_cut_end_adc;  /* arus motoring nol dalam hitungan ADC */
    volatile uint16_t m_battery_regen_cut_start_adc; /* awal derating regen */
    volatile uint16_t m_battery_regen_cut_end_adc;   /* regen nol sebelum hard OV */
    volatile int16_t m_input_current_max_q4;   /* batas arus DC positif */
    volatile int16_t m_input_current_regen_q4; /* magnitudo batas arus DC regeneratif */
    volatile uint16_t m_vin_min_adc;           /* l_min_vin dalam hitungan ADC baterai */
    volatile uint16_t m_vin_max_adc;           /* l_max_vin dalam hitungan ADC baterai */
    volatile uint32_t m_watt_max_x10;          /* batas daya motoring, 0,1 W */
    volatile uint32_t m_watt_regen_x10;        /* magnitudo batas daya regeneratif, 0,1 W */
    volatile int16_t m_watt_current_max_q4;    /* Pmax/Vbus -> batas Iin, cache slow path */
    volatile int16_t m_watt_current_regen_q4;  /* |Pregen|/Vbus -> batas Iin, cache slow path */
    volatile int16_t m_temp_fet_start_x10;     /* awal derating temperatur board/MOS, 0,1 C */
    volatile int16_t m_temp_fet_end_x10;       /* akhir derating / fault, 0,1 C */
    volatile uint32_t m_wrong_voltage_integrator;
    volatile int16_t m_abs_current_limit_counts; /* precomputed l_abs_current_max * A2BIT_CONV */
    volatile int16_t m_duty_limit_permille;      /* precomputed l_max_duty * 1000 */
    int16_t m_voltage_limit_counts;              /* l_max_duty * FOC voltage ceiling, slow-precomputed */
    volatile int16_t m_duty_start_permille;      /* awal current derating terhadap duty */
    volatile int16_t m_duty_end_current_q4;      /* VESC: 5*cc_min_current di max duty */
    volatile int16_t m_cc_min_current_q4;         /* floor current controller standar VESC */
    volatile int32_t m_erpm_pos_start;            /* +ERPM awal derating */
    volatile int32_t m_erpm_pos_end;              /* +ERPM current nol */
    volatile int32_t m_erpm_neg_start;            /* -ERPM awal derating */
    volatile int32_t m_erpm_neg_end;              /* -ERPM current nol */
    volatile int16_t m_temp_fet_accel_start_x10;  /* batas suhu akselerasi awal */
    volatile int16_t m_temp_fet_accel_end_x10;    /* batas suhu akselerasi akhir */
    uint16_t m_in_current_map_start_q15;           /* l_in_current_map_start */
    uint16_t m_in_current_map_filter_q16;          /* alpha LPF measured Iin */
    int32_t m_in_current_map_lpf_q20;              /* measured Iin Q4 disimpan Q20 */
    volatile int16_t m_input_map_current_limit_q4; /* measured-Iin mapped positive motor-current ceiling */

    /* Current state, same Q4 current-count unit as the legacy generated FOC. */
    volatile int16_t m_i_alpha_q4;
    volatile int16_t m_i_beta_q4;
    volatile int16_t m_id_q4;
    volatile int16_t m_iq_q4;
    volatile int16_t m_vd;
    volatile int16_t m_vq;
    volatile int16_t m_current_in_counts;
    /* Set only when Clarke/Park has produced a new D/Q sample on this motor's
     * 2.667-kHz per-motor regulator slot. The outer ABS protection consumes this flag so
     * its three-sample qualification always means three distinct ADC samples. */
    volatile uint8_t m_dq_sample_fresh;
    /* Monitoring-only filtered currents. Upstream VESC keeps a separate current
     * filter for non-time-critical telemetry so the fast current controller is
     * not slowed down by display smoothing. */
    volatile int16_t m_id_telem_q4;
    volatile int16_t m_iq_telem_q4;
    volatile int16_t m_current_in_telem_counts;
    int32_t m_telem_current_lpf_q16[3];
    uint16_t m_telem_current_filter_q16;
    /* VESC-style read/reset telemetry averages. These are accumulated at the
     * 2.667-kHz per-motor control cadence and atomically consumed by COMM_GET_VALUES. */
    volatile int32_t m_telem_sum_id_q4;
    volatile int32_t m_telem_sum_iq_q4;
    /* Upstream VESC accumulates filtered motor-current magnitude itself;
     * do not reconstruct Imotor later from averaged D/Q components. */
    volatile int32_t m_telem_sum_imotor_q4;
    volatile int32_t m_telem_sum_ibus_counts;
    volatile uint16_t m_telem_avg_samples;
    /* Short OFF->RUN sample blanking; fixed startup control offsets are never
     * modified here. */
    volatile uint16_t m_bridge_settle_ticks;
    volatile int16_t m_rpm;
    volatile int16_t m_duty_now_permille;
    volatile uint8_t m_driven_offset_calibrating;
    volatile uint8_t m_driven_offset_valid;
    /* Upstream VESC distinguishes driven (50% PWM / zero-vector) calibration
     * from the undriven bridge-OFF baseline. This flag means a powered baseline
     * has actually completed successfully during this boot. */
    volatile uint8_t m_driven_offset_powered_valid;
    volatile uint16_t m_driven_offset_samples;
    volatile int16_t m_driven_offset0, m_driven_offset1, m_driven_offsetdc;
    /* Akumulasi zero-vector powered dilakukan tanpa pembagian di ISR. Setelah
     * 80 sampel lengkap, housekeeping menghitung mean/validity sementara ISR
     * tetap menahan bridge pada zero-vector (settle_ticks=1). */
    int32_t m_driven_offset_sum0, m_driven_offset_sum1, m_driven_offset_sumdc;
    volatile uint8_t m_driven_offset_finalize_pending;
    /* Separate zero-current ADC offsets while the bridge is high-impedance.
     * The low-side current amplifiers shift operating point between bridge-OFF
     * and centered-PWM states on this hoverboard hardware. */
    volatile int16_t m_off_offset0, m_off_offset1, m_off_offsetdc;
    volatile uint16_t m_off_offset_samples;
    volatile uint16_t m_off_settle_ticks;
    volatile uint8_t m_off_offset_valid;
    /* DC-link shunt remains physically observable with PWM released. Keep its
     * zero independently valid across RUN->OFF while phase-shunt high-Z zero
     * is re-acquired after each driven interval. */
    volatile uint8_t m_off_dc_valid;
    volatile uint8_t m_off_dc_track_div;
    /* Passive OFF DC-current LPF, Q16 ADC-counts. Independent from phase FOC. */
    int32_t m_off_dc_current_lpf_q16;
    int32_t m_off_offset_sum0, m_off_offset_sum1, m_off_offset_sumdc;

    /* VESC energy counters since boot. Upstream exposes separate drawn and
     * charged Ah/Wh counters in COMM_GET_VALUES. Updated from the measured
     * DC-link current at the 5-ms housekeeping cadence, never from telemetry. */
    float m_amp_seconds;
    float m_amp_seconds_charged;
    float m_watt_seconds;
    float m_watt_seconds_charged;

    /* Electrical phase: 0..65535 = 0..360 degrees. */
    volatile uint16_t m_phase;
    volatile uint16_t m_phase_hall;
    volatile uint16_t m_phase_hall_target;
    volatile uint16_t m_phase_encoder;      /* corrected electrical ABI phase */
    volatile uint16_t m_encoder_mech_phase; /* raw mechanical ABI angle 0..360 */
    volatile uint16_t m_phase_openloop;
    volatile uint8_t m_phase_override;

    /* PLL VESC dalam fixed-point. phase_acc memakai satu putaran = 2^32;
     * speed_step_q32 adalah increment phase-Q32 per slot kontrol (DIV/PWM_FREQ). */
    uint32_t m_pll_phase_acc_q32;
    int32_t m_pll_speed_step_q32;
    volatile int32_t m_pll_erpm_q16;
    volatile int32_t m_pll_mech_rpm_q16;
    /* VESC speed-PID low-latency estimators. Both are derived from corrected
     * electrical phase delta at the real current-control cadence. FAST uses
     * alpha=0.01; FASTER uses alpha=0.20, matching upstream foc_math. */
    volatile int32_t m_speed_fast_erpm_q16;
    volatile int32_t m_speed_faster_erpm_q16;
    uint16_t m_speed_est_phase_prev;
    volatile uint8_t m_speed_est_valid;
    uint32_t m_pll_kp_dt_q16;
    uint32_t m_pll_ki_dt2_q16;
    int32_t m_pll_speed_limit_step_q32;
    volatile uint8_t m_pll_valid;
    /* Feed-forward decoupling D/Q VESC, koefisien Q24 langsung menghasilkan
     * satuan modulation-count internal dari current-Q4 dan ERPM. */
    int32_t m_dec_lq_coeff_q24;
    int32_t m_dec_ld_coeff_q24;
    int32_t m_dec_flux_coeff_q24;

    /* Independent VESC FOC flux observer used by the standard Rotor Position
     * diagnostics. It is deliberately separate from m_phase: Encoder/Hall may
     * be authoritative for Park/SVPWM while Observer must remain an estimator. */
    float m_observer_x1;
    float m_observer_x2;
    float m_observer_l_ia;
    float m_observer_l_ib;
    volatile uint8_t m_observer_valid;

    /* VESC ABI encoder runtime state. Only motor LEFT can own TIM4/PB6/PB7. */
    volatile uint32_t m_encoder_raw_count;
    uint32_t m_encoder_prev_count;
    uint32_t m_encoder_counts;
    uint32_t m_encoder_count_to_phase_q16;
    uint32_t m_encoder_mdeg_per_count_q12; /* precomputed 360000/counts, Q12 */
    uint32_t m_encoder_mech_rpm_coeff_q3; /* precomputed 60*PWM*8/counts */
    uint32_t m_encoder_ratio_q16;
    uint16_t m_encoder_offset_phase;
    int32_t m_encoder_delta_accum;
    uint16_t m_encoder_speed_ticks;
    uint16_t m_encoder_idle_ticks;
    volatile int32_t m_encoder_erpm_q16;
    volatile int32_t m_encoder_mech_rpm_q16;
    volatile uint8_t m_encoder_configured;
    volatile uint8_t m_encoder_synced;
    uint16_t m_encoder_max_delta_per_tick; /* glitch ceiling precomputed */

    /* Hall estimator and fixed point regulators. */
    uint8_t m_hall_state;              /* debounced Hall state used by FOC */
    uint8_t m_hall_raw_state;          /* instantaneous GPIO sample */
    uint8_t m_hall_filtered_state;     /* VESC-style majority sample for detection */
    uint8_t m_hall_candidate_state;
    uint8_t m_hall_candidate_count;
    uint8_t m_hall_debounce_initialized;
    /* VESC m_hall_extra_samples adapted to the EFeru hard realtime path:
     * rolling 1+2N sample majority using one synchronized GPIO snapshot per
     * 16-kHz ADC frame. O(1) update avoids repeated GPIO loops inside ISR. */
    uint8_t m_hall_sample_history[41];
    uint8_t m_hall_sample_index;
    uint8_t m_hall_sample_count;
    uint8_t m_hall_sample_sum_u;
    uint8_t m_hall_sample_sum_v;
    uint8_t m_hall_sample_sum_w;
    uint8_t m_hall_filter_window;
    uint8_t m_hall_filter_delay_ticks;
    uint8_t m_hall_direction_stable_edges;
    uint8_t m_hall_pos;
    uint8_t m_hall_pos_prev;
    int8_t m_hall_direction;
    uint16_t m_hall_ticks;
    uint16_t m_hall_period;
    uint32_t m_hall_interp_step_q16; /* electrical phase units/tick in Q16 */
    uint16_t m_hall_rate_limit_step; /* Hall phase correction slew from latest edge */
    uint16_t m_hall_interp_erpm;     /* VESC foc_hall_interp_erpm, cached integer */
    uint16_t m_hall_interp_max_ticks;/* max(edge age,last period) for interpolation */
    uint16_t m_hall_rate_min_step;   /* VESC 1.5x minimum rate-limit step */
    uint16_t m_hall_period_hist[4];
    uint8_t m_hall_hist_pos;
    uint8_t m_hall_initialized;
    uint8_t m_hall_interp_active;
    /* Raw Hall state whose current rejection has already been counted.
     * 0xff means no pending/rejected edge is latched. This prevents a single
     * rejected edge from incrementing the diagnostic counter at 16 kHz while
     * still allowing the same persistent edge to be re-evaluated after its
     * debounce/outlier hold time has elapsed. */
    uint8_t m_hall_reject_counted_state;
    uint32_t m_hall_invalid_transition_count;
    /* Split Hall diagnostics: impossible state/angle sequence vs period filter.
     * hall_invalid_transition_count remains the true electrical-sequence error
     * counter; a legitimate acceleration/reversal must never inflate it. */
    uint32_t m_hall_period_reject_count;
    uint32_t m_hall_sequence_reject_count;
    uint8_t m_hall_last_reject_reason; /* 0 none, 1 period, 2 sequence */
    uint8_t m_hall_last_reject_from;
    uint8_t m_hall_last_reject_to;

    /* Detect-All R/L capture. Updated only on the motor's 2.667-kHz current
     * regulator slot. The main loop starts/stops and snapshots it with IRQs
     * masked, so no 64-bit accumulator can tear on Cortex-M3. */
    volatile uint8_t m_rl_capture_active;
    uint8_t m_rl_capture_have_prev;
    int16_t m_rl_capture_prev_id_q4;
    uint32_t m_rl_capture_n;
    int64_t m_rl_sum_di2;
    int64_t m_rl_sum_div;
    int64_t m_rl_sum_dii;
    int64_t m_rl_sum_di;

    int32_t m_iq_integrator;
    int32_t m_iq_set_ramp_q16;
    int32_t m_id_integrator;
    /* Speed integrator is Iq(q4) Q16; previous error is ERPM Q16. */
    int32_t m_speed_integrator;
    int32_t m_speed_prev_error;
    int32_t m_position_integrator;
    int16_t m_position_prev_error; /* retained for custom count diagnostics */
    int32_t m_position_prev_error_mdeg;
    uint16_t m_position_dt_ticks;
    int32_t m_position_d_filter_q15;
    int32_t m_position_d_proc_filter_q15;
    uint16_t m_position_prev_proc_phase;
    int32_t m_position_prev_proc_count; /* measured position for VESC process-D in count mode */
    uint16_t m_position_proc_dt_ticks;
    uint16_t m_position_breakaway_ticks;
    uint16_t m_position_no_motion_ticks;
    uint8_t m_position_motion_seen;
    uint8_t m_position_step_braking; /* one Hall edge -> brake-to-stop before next sector */
    int8_t m_position_brake_direction; /* latched accepted-edge direction; never follows rebound */
    int32_t m_position_last_motion_count; /* only validated Hall edge movement arms tracking */
    uint16_t m_position_kd_filter_q16;
    /* Precomputed p_pid_kd_proc process-derivative coefficient. This preserves
     * VESC-scale sub-millith gain resolution without float math in the ISR. */
    uint32_t m_position_kd_proc_coeff_q16;
    uint16_t m_position_kd_proc_phase_coeff_q4;
    uint32_t m_position_gain_dec_mdeg; /* p_pid_gain_dec_angle/p_pid_ang_div */
    uint32_t m_position_gain_dec_inv_q31; /* reciprocal for ISR gain scaling */
    uint8_t m_position_sat_hold;
    int8_t m_position_drive_direction;
    uint16_t m_position_settle_ticks;
    int32_t m_speed_set_ramp_q16;
    uint16_t m_speed_ramp_rpm_s;
    uint32_t m_speed_release_erpm_q16; /* exact VESC s_pid_min_erpm runtime threshold */
    uint8_t m_iq_sat_hold;
    uint8_t m_id_sat_hold;
    uint8_t m_speed_sat_hold;
    uint8_t m_speed_zero_hold_quiet; /* 0=brake, 1=quiet hysteresis, 2=zero-cross latched */
    int8_t m_speed_zero_hold_dir;   /* initial braking direction; catches Hall zero crossing */
    /* Brake current is stored as a magnitude. CONTROL_MODE_CURRENT_BRAKE
     * recomputes its sign from fresh Hall speed every control update, matching
     * VESC's -SIGN(speed)*abs(current) semantics without reverse run-away. */
    int16_t m_brake_current_q4;
    int16_t m_handbrake_current_q4;
    int16_t m_brake_vq_prev;            /* Vq sebelumnya untuk deteksi sign crossing */
    int8_t m_brake_speed_dir_prev;       /* arah gerak sebelumnya */
    uint8_t m_brake_zero_duty_samples;   /* minimum zero-vector transition VESC */
    int32_t m_current_lpf_q16[2];

    uint32_t m_openloop_phase_acc_q32;
    uint32_t m_openloop_step_per_rpm_q32; /* mechanical RPM -> electrical phase increment */
    int32_t m_openloop_speed_q16;
    uint16_t m_openloop_align_ticks;
    int8_t m_openloop_direction;
    uint8_t m_openloop_primed;
    int16_t m_openloop_id_target_q4;
    int32_t m_openloop_id_ramp_q16;

    /* PWM and diagnostics. */
    volatile int16_t m_pwm_a;
    volatile int16_t m_pwm_b;
    volatile int16_t m_pwm_c;
    volatile uint16_t m_ccr_a;
    volatile uint16_t m_ccr_b;
    volatile uint16_t m_ccr_c;
    /* Current-sampling observability. With fixed TIM8-TRGO sampling the
     * centered-SVPWM low-side zero-vector half-window is ARR-max(CCR).
     * This is diagnostic evidence only; Stage-2 never shifts ADC timing. */
    volatile uint16_t m_sample_zero_window_counts;
    volatile uint16_t m_sample_window_min_counts;
    volatile uint16_t m_sample_guard_counts;
    volatile uint16_t m_sample_adc_phase_counts;
    volatile uint32_t m_sample_invalid_count;
    volatile uint8_t m_sample_sector;
    volatile uint8_t m_sample_window_valid;
    volatile uint32_t m_isr_count;
    volatile uint32_t m_overrun_count;
    volatile uint32_t m_current_trip_count;
    volatile uint32_t m_phase_trip_count;
    volatile uint32_t m_dc_trip_count;
    volatile uint8_t m_phase_overcurrent_streak;
    volatile uint8_t m_last_trip_source; /* bit0 phase, bit1 DC-link */
    volatile int16_t m_last_trip_phase0_counts;
    volatile int16_t m_last_trip_phase1_counts;
    volatile int16_t m_last_trip_phase2_counts;
    volatile int16_t m_last_trip_dc_counts;
    volatile int16_t m_last_trip_duty_permille;
} mcpwm_foc_motor_t;

/* Snapshot telemetry sudah diskalakan persis ke unit wire COMM_GET_VALUES.
 * Tujuannya menghindari puluhan operasi soft-float pada STM32F103 tanpa FPU
 * saat VESC Tool melakukan polling realtime 50 Hz. */
typedef struct {
    int32_t current_motor_x100;
    int32_t current_in_x100;
    int32_t id_x100;
    int32_t iq_x100;
    int16_t duty_x1000;
    int32_t erpm;
    int16_t vin_x10;
    int32_t ah_x10000;
    int32_t ah_charged_x10000;
    int32_t wh_x10000;
    int32_t wh_charged_x10000;
    int32_t tachometer;
    int32_t tachometer_abs;
    uint8_t fault;
    int32_t vd_x1000;
    int32_t vq_x1000;
} mcpwm_foc_values_scaled_t;

extern mcpwm_foc_motor_t m_motor_1;
extern mcpwm_foc_motor_t m_motor_2;

void mcpwm_foc_init(void);
mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool is_second_motor);
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool is_second_motor);

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool is_second_motor);
const volatile mc_configuration *mcpwm_foc_get_configuration(bool is_second_motor);

void mcpwm_foc_set_duty(float duty, bool is_second_motor);
void mcpwm_foc_set_pid_speed(float rpm, bool is_second_motor);
void mcpwm_foc_set_current(float current, bool is_second_motor);
void mcpwm_foc_set_pid_pos(float position_deg, bool is_second_motor);
void mcpwm_foc_set_position_counts(int32_t position_counts, bool is_second_motor);
/* User-facing long-range position API. Left/right share the same sign convention;
 * right is mirrored only internally. Values are full signed int32 counts. */
void mcpwm_foc_set_position_user_counts(int32_t position_counts, bool is_second_motor);
void mcpwm_foc_set_position_user_limits(int32_t min_counts, int32_t max_counts, bool is_second_motor);
int32_t mcpwm_foc_get_position_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_target_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_min_user_counts(bool is_second_motor);
int32_t mcpwm_foc_get_position_max_user_counts(bool is_second_motor);
void mcpwm_foc_reset_position(bool is_second_motor);
void mcpwm_foc_set_brake_current(float current, bool is_second_motor);
void mcpwm_foc_set_handbrake(float current, bool is_second_motor);
void mcpwm_foc_set_openloop_current(float current, float rpm, bool is_second_motor);
void mcpwm_foc_set_openloop_phase(float current, float phase, bool is_second_motor);
/* User STOP/zero command: keep zero-vector PWM and current sensing alive while
 * the VESC command link is healthy. Fault/E-stop/watchdog still use hard release. */
void mcpwm_foc_enter_standby(bool is_second_motor);
bool mcpwm_foc_encoder_startup_align(bool is_second_motor);
bool mcpwm_foc_encoder_is_synced(bool is_second_motor);
bool mcpwm_foc_encoder_detect(float current, bool is_second_motor, float *offset, float *ratio, bool *inverted);
void mcpwm_foc_release_motor(bool is_second_motor);
/* Manual VESC-style fault reset. Outputs are released before the current
 * per-motor fault latch is cleared. Persistent unsafe conditions are allowed
 * to qualify and fault again normally. */
void mcpwm_foc_clear_fault(bool is_second_motor);
void mcpwm_foc_clear_faults(void);
/* Publish a hardware-IWDG reboot through the standard VESC fault enum. The
 * fault is transient and uses the configured VESC fault-stop interval. */
void mcpwm_foc_report_watchdog_reset_fault(void);
void mcpwm_foc_force_bridges_off(void);
/* VESC COMM_MOTOR_ESTOP: hentikan kedua bridge dan abaikan perintah motor
 * selama duration_ms. Nilai 0 berarti release sekali tanpa hold tambahan. */
void mcpwm_foc_estop_both(uint16_t duration_ms);
bool mcpwm_foc_estop_active(void);
void mcpwm_foc_vesc_timeout_configure(bool is_second_motor, uint32_t timeout_ms, float brake_current);
void mcpwm_foc_vesc_override_touch(bool is_second_motor);
bool mcpwm_foc_vesc_override_active(bool is_second_motor);
bool mcpwm_foc_vesc_command_live(bool is_second_motor);
void mcpwm_foc_vesc_override_clear(bool is_second_motor);
void mcpwm_foc_energy_update(uint32_t now_ms);
void mcpwm_foc_outer_control_non_isr(uint32_t now_ms);
void mcpwm_foc_housekeeping_non_isr(uint32_t now_ms);
void mcpwm_foc_set_board_temperature_x10(int16_t temperature_x10);

/* Integer API used by the bare-metal command layer. */
void mcpwm_foc_set_mode_command(uint8_t mode, int16_t command, bool run_request,
                                uint16_t openloop_rpm, bool is_second_motor);

float mcpwm_foc_get_tot_current_motor(bool is_second_motor);
float mcpwm_foc_get_tot_current_in_motor(bool is_second_motor);
float mcpwm_foc_get_erpm_motor(bool is_second_motor);  /* VESC electrical RPM */
float mcpwm_foc_get_motor_mechanical_rpm(bool is_second_motor);
float mcpwm_foc_get_output_rpm(bool is_second_motor); /* after si_gear_ratio */
uint16_t mcpwm_foc_get_pole_pairs(bool is_second_motor);
float mcpwm_foc_get_gear_ratio(bool is_second_motor);
float mcpwm_foc_get_duty_cycle_motor(bool is_second_motor);
float mcpwm_foc_get_id_motor(bool is_second_motor);
float mcpwm_foc_get_iq_motor(bool is_second_motor);
float mcpwm_foc_get_vd_motor(bool is_second_motor);
float mcpwm_foc_get_vq_motor(bool is_second_motor);
float mcpwm_foc_get_phase_motor(bool is_second_motor);
float mcpwm_foc_get_phase_observer_motor(bool is_second_motor); /* independent FOC observer */
bool mcpwm_foc_observer_valid(bool is_second_motor);
float mcpwm_foc_get_phase_encoder_motor(bool is_second_motor); /* corrected electrical */
float mcpwm_foc_get_encoder_position_motor(bool is_second_motor); /* raw mechanical ABI */
float mcpwm_foc_get_pid_pos_now_motor(bool is_second_motor);
float mcpwm_foc_get_pid_pos_set_motor(bool is_second_motor);
mc_state mcpwm_foc_get_state_motor(bool is_second_motor);
mc_fault_code mcpwm_foc_get_fault_motor(bool is_second_motor);
void mcpwm_foc_get_values(mc_values *values, bool is_second_motor);
void mcpwm_foc_get_values_scaled(mcpwm_foc_values_scaled_t *values, bool is_second_motor);
void mcpwm_foc_sync_tuning_to_conf(bool is_second_motor);
void mcpwm_foc_apply_tuning_from_conf(bool is_second_motor);
void mcpwm_foc_refresh_hall_interpolation(bool is_second_motor);
void mcpwm_foc_refresh_encoder_configuration(bool is_second_motor, bool reinitialize);
void mcpwm_foc_refresh_position_configuration(bool is_second_motor);
void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool is_second_motor);
/* VESC-compatible Hall FOC detection. Returns table[8] in 0..199 electrical-angle units. */
/* VESC mcpwm_foc_hall_detect method. F103 dual-motor extension keeps an
 * explicit motor selector. Circular samples use the existing Q15 LUT to save
 * flash; final angle uses upstream atan2/truncate semantics outside ADC ISR. */
bool mcpwm_foc_hall_detect(float current, bool is_second_motor, uint8_t table[8]);
uint8_t mcpwm_foc_hall_detect_angle200(int64_t sum_s, int64_t sum_c, uint16_t samples);

typedef struct {
    uint32_t samples;
    int64_t sum_di2;
    int64_t sum_div;
    int64_t sum_dii;
    int64_t sum_di;
} mcpwm_foc_rl_capture_t;
void mcpwm_foc_rl_capture_start(bool is_second_motor);
void mcpwm_foc_rl_capture_stop(bool is_second_motor);
void mcpwm_foc_rl_capture_get(bool is_second_motor, mcpwm_foc_rl_capture_t *out);

#define MCPWM_FOC_TRACE_CAPACITY 40u

typedef struct {
    volatile uint32_t guard;
    uint32_t pwm_tick;
    uint16_t isr_cycles;
    uint8_t control_slot, event_bits;
    int16_t left_id_q4, left_iq_q4, left_id_set_q4, left_iq_set_q4, left_vd, left_vq, left_erpm;
    int16_t right_id_q4, right_iq_q4, right_id_set_q4, right_iq_set_q4, right_vd, right_vq, right_erpm;
    int32_t left_id_integrator, left_iq_integrator, right_id_integrator, right_iq_integrator;
    uint16_t left_sample_window, right_sample_window;
    uint16_t vin_adc;
    uint8_t left_fault, right_fault, left_quality, right_quality;
} mcpwm_foc_trace_sample_t;

typedef struct {
    uint32_t write_count;
    uint8_t frozen, trigger_motor, trigger_fault, count, head, capacity;
    uint16_t sample_size;
} mcpwm_foc_trace_meta_t;

bool mcpwm_foc_trace_clear(void);
bool mcpwm_foc_trace_freeze(void);
void mcpwm_foc_trace_get_meta(mcpwm_foc_trace_meta_t *out);
bool mcpwm_foc_trace_read(uint8_t chronological_index, mcpwm_foc_trace_sample_t *out);

typedef struct {
    uint16_t ccr_a, ccr_b, ccr_c;
    uint16_t zero_window_counts, min_window_counts, guard_counts, adc_phase_counts;
    uint32_t invalid_count;
    uint8_t sector, window_valid, offset_valid, driven_offset_valid, bridge_settled;
} mcpwm_foc_adc_sample_diag_t;
void mcpwm_foc_get_adc_sample_diag(bool is_second_motor, mcpwm_foc_adc_sample_diag_t *out);

typedef enum {
    MCPWM_FOC_STEP_AXIS_Q = 0u,
    MCPWM_FOC_STEP_AXIS_D = 1u
} mcpwm_foc_step_axis_t;

typedef struct {
    uint32_t sequence;
    int16_t pre_q4, step_q4;
    uint8_t active, second, pre_remaining, post_remaining, step_fired, done;
} mcpwm_foc_step_test_status_t;
bool mcpwm_foc_step_test_arm_axis(float pre_current_a, float step_current_a, uint8_t pre_samples, uint8_t post_samples,
                                  bool is_second_motor, mcpwm_foc_step_axis_t axis);
bool mcpwm_foc_step_test_arm(float pre_current_a, float step_current_a, uint8_t pre_samples, uint8_t post_samples, bool is_second_motor);
void mcpwm_foc_step_test_get(mcpwm_foc_step_test_status_t *out);

typedef enum {
    MCPWM_FOC_RELAY_NONE = 0u,
    MCPWM_FOC_RELAY_SPEED = 1u,
    MCPWM_FOC_RELAY_POSITION = 2u
} mcpwm_foc_relay_mode_t;

typedef struct {
    uint32_t sequence, elapsed_ms, period_sum_ms;
    int32_t target, hysteresis, measurement, minimum, maximum;
    uint16_t relay_current_ma, period_count;
    uint8_t active, done, failed, mode, second, relay_positive, crossings, required_crossings;
} mcpwm_foc_relay_status_t;

bool mcpwm_foc_relay_start(mcpwm_foc_relay_mode_t mode, bool is_second_motor, int32_t target,
                           int32_t hysteresis, uint16_t relay_current_ma, uint8_t required_crossings,
                           uint32_t timeout_ms);
void mcpwm_foc_relay_abort(void);
void mcpwm_foc_relay_get(mcpwm_foc_relay_status_t *out);

#define MCPWM_FOC_PROFILE_SLOT_CAPACITY 6u
#define MCPWM_FOC_ISR_PROFILE_REVISION  0x00030000u

typedef struct {
    uint32_t total_max_cycles, deadline_miss_count;
    uint32_t pre_max_cycles, control_max_cycles, post_max_cycles;
    uint32_t pre_gate_max_cycles, pre_offset_max_cycles, pre_protect_max_cycles;
    uint32_t motor_step_max_cycles[2], motor_control_max_cycles[2], motor_hold_max_cycles[2];
    uint32_t sensor_max_cycles, pll_max_cycles, current_max_cycles, regulator_max_cycles;
    uint32_t position_pid_max_cycles, speed_pid_max_cycles, current_circle_max_cycles;
    uint32_t id_pi_max_cycles, iq_pi_max_cycles, decouple_limit_max_cycles;
    uint32_t svpwm_max_cycles, duty_mag_max_cycles, reentry_guard_total;
    /* Profiler acceptance minimal: worst-case dan miss per scheduler slot 0..5.
     * Tidak menambah DWT read baru; memakai elapsed ISR yang sudah tersedia. */
    uint32_t slot_max_cycles[6], slot_miss_count[6], slot_count[6];
    uint32_t detail_sample_count, detail_slot_count[6];
    uint32_t steady_isr_count, slot_sequence_error_count;
    uint32_t fast_hold_svpwm_max_cycles, profile_revision;
    uint32_t active_slot_count, reset_epoch;
    /* Main-context 1-kHz SPEED/POS scheduler. Kept in the same diagnostic
     * transaction so timing can be verified without attaching SWD to F103. */
    uint32_t outer_max_cycles, outer_miss_count, outer_jitter_max_cycles;
    uint32_t outer_period_min_cycles, outer_period_max_cycles;
    uint32_t adc_heartbeat;
    uint32_t motor_heartbeat[2];
    /* Monotonic validity counters. These are intentionally not cleared by
     * mcpwm_foc_reset_isr_profile(); host tests compare snapshot deltas. */
    uint32_t snapshot_dwt, irq_entry_count, irq_exit_count;
    uint32_t motor_step_count[2];
    uint32_t dma_tc_pending_exit_count;
} mcpwm_foc_isr_profile_t;
void mcpwm_foc_get_isr_profile(mcpwm_foc_isr_profile_t *out);
void mcpwm_foc_get_irq_epoch(uint32_t *entry, uint32_t *exit);
bool mcpwm_foc_reset_isr_profile(void);

/* Hardware calibration / ISR diagnostics. */
bool mcpwm_foc_dc_cal_done(void);
void mcpwm_foc_get_current_offsets(int16_t *pha0, int16_t *pha1, int16_t *dc,
                                   bool is_second_motor);
uint32_t mcpwm_foc_get_isr_cycles(void);
uint32_t mcpwm_foc_get_isr_cycles_max(void);
void mcpwm_foc_get_liveness(uint32_t *adc_heartbeat, uint32_t motor_heartbeat[2]);

/* Called from the original DMA1_Channel1_IRQHandler after ADC frame acquisition. */
void mcpwm_foc_adc_int_handler(void);

#ifdef __cplusplus
}
#endif


void mcpwm_foc_steering_clear_calibration(void);
bool mcpwm_foc_steering_set_span(int32_t span_counts, bool homed);
bool mcpwm_foc_steering_rebase_left(void);
bool mcpwm_foc_steering_rebase_center(void);
bool mcpwm_foc_steering_is_calibrated(void);
bool mcpwm_foc_steering_is_homed(void);
int32_t mcpwm_foc_steering_span_counts(void);
int32_t mcpwm_foc_steering_safe_span_counts(void);
float mcpwm_foc_get_steering_deg(void);
bool mcpwm_foc_set_steering_deg(float deg);
#endif
