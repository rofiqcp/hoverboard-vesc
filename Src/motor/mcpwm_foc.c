#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "setup.h"
#include "util.h"
#include "motor/mcconf_default.h"
#include "motor/foc_math.h"
#include "motor/mcpwm_foc.h"
#include "encoder/encoder.h"
#include "platform_watchdog.h"

/* ========================================================================== */
/* Bare-metal VESC-style dual FOC state                                       */
/* ========================================================================== */
mcpwm_foc_motor_t m_motor_1;
mcpwm_foc_motor_t m_motor_2;

extern volatile adc_buf_t adc_buffer;
extern uint8_t ctrlModReq;

volatile int pwml = 0;
volatile int pwmr = 0;
uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
uint8_t buzzerCount = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t buzzerPrev = 0;
static uint8_t buzzerIdx = 0;
/* Hold E-stop dalam tick ADC 16 kHz agar gate tetap deterministik tanpa HAL tick. */
static volatile uint32_t s_estop_ticks = 0u;

static inline int32_t q16_from_i32_sat(int32_t v) {
    if(v>INT32_MAX/65536)return INT32_MAX;
    if(v<INT32_MIN/65536)return INT32_MIN;
    return v*65536;
}

static inline void buzzer_pin_toggle_fast(void) {
    /* ISR F103: akses register langsung, deterministic beberapa cycle. */
    BUZZER_PORT->ODR ^= BUZZER_PIN;
}

static inline void buzzer_pin_low_fast(void) {
    BUZZER_PORT->ODR &= ~(uint32_t)BUZZER_PIN;
}

uint8_t enable = 0;
volatile uint8_t motorRunReq = 1u;
volatile uint16_t svpwmOpenloopRpm = SVPWM_OPENLOOP_RPM_DEFAULT;
volatile int32_t positionCommandL = 0;
volatile int32_t positionCommandR = 0;

volatile uint32_t foc_isr_cycles = 0;
static volatile uint32_t foc_adc_heartbeat = 0u;
static volatile uint32_t foc_motor_heartbeat[2] = {0u, 0u};
/* Raw DMA-IRQ accounting is separate from per-motor m_isr_count. */
static volatile uint32_t foc_irq_entry_count=0u, foc_irq_exit_count=0u;
static volatile uint32_t foc_irq_dma_tc_pending_exit_count=0u;
static mcpwm_foc_trace_sample_t s_foc_trace[MCPWM_FOC_TRACE_CAPACITY];
static volatile uint8_t s_foc_trace_head=0u, s_foc_trace_count=0u, s_foc_trace_frozen=0u;
static volatile uint8_t s_foc_trace_trigger_motor=0u, s_foc_trace_trigger_fault=0u;
static volatile uint8_t s_foc_trace_clear_req=0u, s_foc_trace_freeze_req=0u;
static volatile uint8_t s_foc_trace_fault_pending=0u, s_foc_trace_fault_motor=0u, s_foc_trace_fault_code=0u;
static volatile uint32_t s_foc_trace_write_count=0u, s_foc_trace_req_seq=0u, s_foc_trace_ack_seq=0u;
static volatile uint8_t s_foc_trace_event_latch=0u;
/* Patch each captured row with this IRQ's full elapsed time at ISR exit. */
static volatile uint8_t s_foc_trace_cycle_pending=0u, s_foc_trace_cycle_index=0u;
typedef struct {
    volatile uint32_t sequence;
    volatile int16_t pre_q4, step_q4;
    volatile uint8_t active, second, pre_remaining, post_remaining, step_fired, done;
    volatile uint8_t settling, axis;
    volatile uint16_t settle_remaining;
} foc_step_test_state_t;
static foc_step_test_state_t s_foc_step_test={0};
static void foc_trace_service_requests_isr(void);
static void foc_trace_clear_isr_owned(void);
volatile uint8_t encoder_detect_stage = 0u;
/* Startup-align black box. Kept separate from full encoder detect so HOME
 * failures after reboot can be diagnosed without repeating hard-stop calibration. */
volatile uint8_t encoder_align_stage = 0u;
volatile uint32_t encoder_align_before_count = 0u;
volatile uint32_t encoder_align_jog_count = 0u;
volatile uint32_t encoder_align_back_count = 0u;
volatile int32_t encoder_align_jog_delta = 0;
volatile int32_t encoder_align_back_delta = 0;
volatile uint16_t encoder_align_current_ma = 0u;
volatile int32_t encoder_detect_plus_mdeg = 0;
volatile int32_t encoder_detect_minus_mdeg = 0;
volatile uint32_t encoder_detect_plus_count = 0u;
volatile uint32_t encoder_detect_minus_count = 0u;
volatile uint32_t encoder_detect_origin_count = 0u;
volatile uint32_t encoder_gpio_edge_a = 0u; /* PB6 */
volatile uint32_t encoder_gpio_edge_b = 0u; /* PB7 */
volatile uint32_t encoder_gpio_edge_pb5 = 0u;
volatile uint32_t encoder_gpio_samples = 0u;
volatile uint8_t encoder_gpio_last_ab = 0u;
volatile uint8_t encoder_gpio_last_pb5 = 0u;
volatile int16_t encoder_detect_plus_id_q4 = 0;
volatile int16_t encoder_detect_plus_iq_q4 = 0;
volatile int16_t encoder_detect_minus_id_q4 = 0;
volatile int16_t encoder_detect_minus_iq_q4 = 0;
static inline void encoder_stage_set(uint8_t stage) {
    encoder_detect_stage = stage;
#ifdef STM32F103xE
    volatile uint32_t *const w = (volatile uint32_t *)0x2000BFFCu;
    *w = (*w & 0xFFFFFF00u) | (uint32_t)stage;
#endif
}
volatile uint32_t foc_isr_cycles_max = 0;
volatile uint32_t foc_prof_sensor_max_cycles=0u;
volatile uint32_t foc_prof_current_max_cycles=0u;
volatile uint32_t foc_prof_regulator_max_cycles=0u;
volatile uint32_t foc_prof_svpwm_max_cycles=0u;
volatile uint32_t foc_isr_deadline_miss_count=0u;
static volatile uint32_t foc_isr_slot_max_cycles[6]={0u,0u,0u,0u,0u,0u};
static volatile uint32_t foc_isr_slot_miss_count[6]={0u,0u,0u,0u,0u,0u};
static volatile uint32_t foc_isr_slot_count[6]={0u,0u,0u,0u,0u,0u};
static volatile uint8_t foc_isr_profile_slot=0xffu;
/* Detailed stage timing is sampled once every 31 steady-state ISR frames.
 * 31 is coprime with CONTROL_DIV=6, so samples walk across every scheduler
 * slot without modulo/division or continuous DWT reads in the hot path. */
#define FOC_PROF_SAMPLE_PERIOD 31u
#define FOC_ISR_PROFILE_REVISION MCPWM_FOC_ISR_PROFILE_REVISION
_Static_assert(MCCONF_FOC_CONTROL_DIV >= 1u && MCCONF_FOC_CONTROL_DIV <= MCPWM_FOC_PROFILE_SLOT_CAPACITY, "FOC control divider exceeds profiler slot capacity");
static uint8_t foc_prof_sample_down=0u;
static uint8_t foc_prof_detail_sample=0u;
static volatile uint32_t foc_prof_detail_sample_count=0u;
static volatile uint32_t foc_prof_reset_request=0u, foc_prof_reset_ack=0u;
static volatile uint32_t foc_prof_detail_slot_count[6]={0u,0u,0u,0u,0u,0u};
static volatile uint32_t foc_isr_steady_count=0u;
static volatile uint32_t foc_isr_slot_sequence_error_count=0u;
static uint8_t foc_isr_expected_slot=0xffu;
volatile uint32_t foc_prof_pre_max_cycles=0u;
volatile uint32_t foc_prof_control_max_cycles=0u;
volatile uint32_t foc_prof_post_max_cycles=0u;
static volatile uint32_t foc_prof_pre_gate_max_cycles=0u;
static volatile uint32_t foc_prof_pre_offset_max_cycles=0u;
static volatile uint32_t foc_prof_pre_protect_max_cycles=0u;
static volatile uint32_t foc_prof_motor_step_max_cycles[2]={0u,0u};
static volatile uint32_t foc_prof_motor_control_max_cycles[2]={0u,0u};
static volatile uint32_t foc_prof_motor_hold_max_cycles[2]={0u,0u};
static volatile uint32_t foc_prof_pll_max_cycles=0u;
static volatile uint32_t foc_prof_position_pid_max_cycles=0u;
static volatile uint32_t foc_prof_speed_pid_max_cycles=0u;
static volatile uint32_t foc_prof_current_circle_max_cycles=0u;
static volatile uint32_t foc_prof_id_pi_max_cycles=0u;
static volatile uint32_t foc_prof_iq_pi_max_cycles=0u;
static volatile uint32_t foc_prof_decouple_limit_max_cycles=0u;
static volatile uint32_t foc_prof_duty_mag_max_cycles=0u;
static volatile uint32_t foc_prof_fast_hold_svpwm_max_cycles=0u;
#define FOC_ISR_BUDGET_CYCLES (CPU_CLOCK_HZ / (uint32_t)PWM_FREQ_HZ)
#define OUTER_PID_PERIOD_CYCLES (CPU_CLOCK_HZ / (uint32_t)MCCONF_OUTER_PID_HZ)
#define TELEMETRY_PERIOD_MS ((1000u + (uint32_t)MCCONF_TELEMETRY_HZ - 1u) / (uint32_t)MCCONF_TELEMETRY_HZ)
#if defined(__arm__) || defined(__thumb__)
#define FOC_MEMORY_BARRIER() __DMB()
#else
#define FOC_MEMORY_BARRIER() __asm__ volatile("" ::: "memory")
#endif
volatile int16_t foc_iqL_q4 = 0;
volatile int16_t foc_iqR_q4 = 0;
volatile int16_t foc_idL_q4 = 0;
volatile int16_t foc_idR_q4 = 0;

int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt = ((400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE) * 65536;

static float bus_voltage_now(void);
static void duty_pi_apply_vbus(mcpwm_foc_motor_t *m, uint32_t vin_cv);
static void pll_coeff_recompute(mcpwm_foc_motor_t *m);
static void decoupling_coeff_recompute(mcpwm_foc_motor_t *m);
static int16_t duty_permille_from_vdq(int16_t vd, int16_t vq);
/* VESC: mod = Vdq * 1.5 / Vbus. Internal 16000 == mod 1.0.
 * Cache reciprocal scales when the slow battery filter updates so each motor's
 * 2.667-kHz current regulator needs only multiply/shift, no float/division. */
static uint32_t s_mod_counts_per_volt_q16 = 39321600u; /* 40 V startup */
static uint32_t s_volt_q24_per_mod_count = 27962u;      /* 40 V startup */
static int16_t s_voltage_scale_bat_adc = INT16_MIN;

static void watt_current_limits_refresh(mcpwm_foc_motor_t *m, uint32_t vin_cv) {
    if(!m)return;
    const uint32_t cv=vin_cv?vin_cv:1u;
    /* VESC 6.00: I_in <= P_limit / V_in. Konversi P(0,1 W), V(cV) ke
     * current Q4 menghasilkan P_x10*8000/V_cV. Operasi 64-bit/division ini
     * hanya berjalan di slow path; ISR cukup membaca cache int16. */
    uint64_t q=((uint64_t)m->m_watt_max_x10*8000u)/cv;
    uint32_t hard=(uint32_t)I_DC_MAX*(uint32_t)FOC_CURRENT_Q4_PER_A;
    if(q>hard)q=hard;
    if(q>INT16_MAX)q=INT16_MAX;
    m->m_watt_current_max_q4=(int16_t)q;
    q=((uint64_t)m->m_watt_regen_x10*8000u)/cv;
    if(q>hard)q=hard;
    if(q>INT16_MAX)q=INT16_MAX;
    m->m_watt_current_regen_q4=(int16_t)q;
}

static void current_voltage_scale_refresh(void) {
    const uint32_t vin_cv=((uint32_t)(batVoltage>0?batVoltage:1)*(uint32_t)BAT_CALIB_REAL_VOLTAGE)/(uint32_t)BAT_CALIB_ADC;
    const uint32_t cv=vin_cv?vin_cv:1u;
    uint64_t a=(uint64_t)2400000u*65536u;
    s_mod_counts_per_volt_q16=(uint32_t)(a/cv);
    s_volt_q24_per_mod_count=(uint32_t)(((uint64_t)cv*16777216u+1200000u)/2400000u);
    if(s_volt_q24_per_mod_count==0u)s_volt_q24_per_mod_count=1u;
    /* Gain duty down-ramp dan watt/current VESC dihitung di slow path, bukan
     * di ISR 16 kHz. Ini menjaga worst-case ISR bebas __aeabi_uldivmod. */
    duty_pi_apply_vbus(&m_motor_1,cv);
    duty_pi_apply_vbus(&m_motor_2,cv);
    watt_current_limits_refresh(&m_motor_1,cv);
    watt_current_limits_refresh(&m_motor_2,cv);
    /* Feed-forward D/Q memakai skala modulation-count per volt. Saat Vbus
     * berubah, hanya koefisien slow-path ini yang diperbarui; ISR tetap integer. */
    decoupling_coeff_recompute(&m_motor_1);
    decoupling_coeff_recompute(&m_motor_2);
    s_voltage_scale_bat_adc=batVoltage;
}


/* ADC offset calibration is intentionally identical to the proven EFeru ISR. */
static uint16_t offsetcount = 0;
static int16_t offsetrlA = 2000;
static int16_t offsetrlB = 2000;
static int16_t offsetrrB = 2000;
static int16_t offsetrrC = 2000;
static int16_t offsetdcl = 2000;
static int16_t offsetdcr = 2000;

static const uint16_t pwm_res = CPU_CLOCK_HZ / 2u / PWM_FREQ_HZ; /* 2000 */
static int16_t pwm_margin = MCCONF_PWM_MARGIN_COUNTS;
static int16_t curDC_max  = (I_DC_MAX * A2BIT_CONV);

int16_t odom_l = 0, odom_r = 0;
static volatile uint8_t s_overrun = 0;
/* VESC command ownership is separate from its safety timeout. Upstream VESC
 * keeps the last motor setpoint while COMM_ALIVE resets timeout_reset(); when
 * the timeout expires the motor is stopped/braked, but an unrelated legacy
 * input source must not immediately overwrite that VESC state. */
static volatile uint8_t s_vesc_owned[2] = {0u, 0u};
static volatile uint8_t s_vesc_timeout_braking[2] = {0u, 0u};
static volatile uint8_t s_vesc_timeout_expired[2] = {0u, 0u};
static volatile uint32_t s_vesc_timeout_ticks[2] = {0u, 0u};
static volatile uint32_t s_vesc_timeout_ms[2] = {1000u, 1000u};
static volatile int16_t s_vesc_timeout_brake_q4[2] = {0, 0};
static uint32_t s_energy_last_ms = 0u;
static uint8_t s_foc_control_div = 0u;
static uint32_t s_housekeeping_last_ms = 0u;
static uint32_t s_outer_tick_remainder = 0u;
/* Scheduler outer-loop VESC: satu evaluasi dengan feedback fresh setiap 1 ms.
 * Jika main terlambat, dt aktual dipakai sekali; tidak ada virtual stale catch-up. */
static uint32_t s_outer_pid_last_ms = 0u;
static uint32_t s_outer_pid_last_cycle = 0u;
typedef struct {
    volatile uint32_t sequence;
    volatile uint32_t start_ms, timeout_ms, period_sum_ms, last_upper_ms, last_lower_ms;
    volatile int32_t target, hysteresis, measurement, minimum, maximum;
    volatile int16_t relay_q4;
    volatile uint16_t relay_current_ma, period_count;
    volatile uint8_t active, done, failed, mode, second, relay_positive, crossings, required_crossings;
} foc_relay_state_t;
static foc_relay_state_t s_foc_relay={0};
volatile uint32_t outer_control_max_cycles = 0u;
volatile uint32_t outer_control_miss_count = 0u;
volatile uint32_t outer_control_jitter_max_cycles = 0u;
volatile uint32_t outer_control_period_min_cycles = UINT32_MAX;
volatile uint32_t outer_control_period_max_cycles = 0u;
/* LEFT ABI VESC tachometer cache. Jangan hitung sektor dari sampled phase 200 Hz:
 * pada ERPM tinggi beberapa sektor bisa lewat antar snapshot. Gunakan delta count
 * mekanik kumulatif + remainder numerator di slow path, sehingga tidak ada edge
 * hilang dan ADC ISR tidak mendapat satu operasi tambahan pun. */
static int32_t s_left_abi_tacho_pos_last = 0;
static int64_t s_left_abi_tacho_remainder = 0;
static uint8_t s_left_abi_tacho_tracking = 0u;
static volatile int16_t s_board_temperature_x10 = 250; /* 25,0 C sampai housekeeping pertama */

static bool encoder_port_active(const mcpwm_foc_motor_t *m, bool second);
static bool encoder_feedback_selected(const mcpwm_foc_motor_t *m, bool second);
static void encoder_feedback_update(mcpwm_foc_motor_t *m, bool second, uint16_t elapsed_pwm_ticks);

/* VESC FOC Hall table: 0..199 = 0..360 electrical degrees, 255 = invalid.
 * These defaults reproduce the previously proven hard-coded sector centers. */
static const uint8_t s_default_foc_hall_table[8] = {255u, 83u, 17u, 50u, 150u, 117u, 183u, 255u};

static uint16_t default_motor_poles(bool second) {
#if MCCONF_POLE_PAIRS_LEFT == MCCONF_POLE_PAIRS_RIGHT
    (void)second;
    return (uint16_t)(2u * MCCONF_POLE_PAIRS_LEFT);
#else
    return (uint16_t)(2u * (second ? MCCONF_POLE_PAIRS_RIGHT : MCCONF_POLE_PAIRS_LEFT));
#endif
}

static uint16_t motor_pole_pairs(bool second) {
    const mc_configuration *c = second ? &m_motor_2.m_conf : &m_motor_1.m_conf;
    uint16_t poles = c->si_motor_poles;
    if (poles < 2u || (poles & 1u)) poles = default_motor_poles(second);
    return (uint16_t)(poles / 2u);
}

static int16_t foc_trace_erpm(const mcpwm_foc_motor_t *m, bool second) {
    int32_t e=m->m_pll_valid?(m->m_pll_erpm_q16>>16):((int32_t)m->m_rpm*(int32_t)motor_pole_pairs(second));
    return (int16_t)CLAMP(e,-32768,32767);
}

static uint8_t foc_trace_quality(const mcpwm_foc_motor_t *m, bool second) {
    uint8_t q=0u;
    if(m->m_current_offset_valid)q|=1u<<0;
    if(m->m_driven_offset_valid)q|=1u<<1;
    if(m->m_sample_window_valid)q|=1u<<2;
    if(m->m_bridge_settle_ticks==0u)q|=1u<<3;
    if(m->m_id_sat_hold)q|=1u<<4;
    if(m->m_iq_sat_hold)q|=1u<<5;
    if(second ? ((RIGHT_TIM->BDTR&TIM_BDTR_MOE)!=0u) : ((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u))q|=1u<<6;
    if(m->m_dq_sample_fresh)q|=1u<<7;
    return q;
}

static void foc_sample_window_update(mcpwm_foc_motor_t *m, bool second) {
    uint16_t mx=m->m_ccr_a; if(m->m_ccr_b>mx)mx=m->m_ccr_b; if(m->m_ccr_c>mx)mx=m->m_ccr_c;
    const uint16_t win=(mx<pwm_res)?(uint16_t)(pwm_res-mx):0u;
    m->m_sample_zero_window_counts=win;
    m->m_sample_guard_counts=FOC_CURRENT_SAMPLE_GUARD_COUNTS;
    m->m_sample_adc_phase_counts=pwm_res;
    m->m_sample_sector=(uint8_t)((((uint32_t)m->m_phase*6u)>>16)+1u);
    m->m_sample_window_valid=(win>=FOC_CURRENT_SAMPLE_GUARD_COUNTS)?1u:0u;
    if(m->m_sample_window_min_counts==0u || win<m->m_sample_window_min_counts)m->m_sample_window_min_counts=win;
    const bool bridge_on=second ? ((RIGHT_TIM->BDTR&TIM_BDTR_MOE)!=0u) : ((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u);
    if(bridge_on && m->m_control_mode!=CONTROL_MODE_NONE && !m->m_sample_window_valid)m->m_sample_invalid_count++;
}

static void foc_trace_capture_internal(uint8_t slot) {
    if(s_foc_trace_frozen)return;
    const uint8_t idx=s_foc_trace_head;
    mcpwm_foc_trace_sample_t *t=&s_foc_trace[idx];
    uint32_t g=t->guard; if(g&1u)g++;
    t->guard=g+1u; FOC_MEMORY_BARRIER();
    t->pwm_tick=buzzerTimer; t->isr_cycles=0u;
    t->control_slot=slot;
    t->event_bits=(uint8_t)(((LEFT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u)|((RIGHT_TIM->BDTR&TIM_BDTR_MOE)?2u:0u)|s_foc_trace_event_latch);
    s_foc_trace_event_latch=0u;
    t->left_id_q4=m_motor_1.m_id_q4; t->left_iq_q4=m_motor_1.m_iq_q4; t->left_id_set_q4=m_motor_1.m_id_set_q4; t->left_iq_set_q4=m_motor_1.m_iq_set_q4;
    t->left_vd=m_motor_1.m_vd; t->left_vq=m_motor_1.m_vq; t->left_erpm=foc_trace_erpm(&m_motor_1,false);
    t->right_id_q4=m_motor_2.m_id_q4; t->right_iq_q4=m_motor_2.m_iq_q4; t->right_id_set_q4=m_motor_2.m_id_set_q4; t->right_iq_set_q4=m_motor_2.m_iq_set_q4;
    t->right_vd=m_motor_2.m_vd; t->right_vq=m_motor_2.m_vq; t->right_erpm=foc_trace_erpm(&m_motor_2,true);
    t->left_id_integrator=m_motor_1.m_id_integrator; t->left_iq_integrator=m_motor_1.m_iq_integrator;
    t->right_id_integrator=m_motor_2.m_id_integrator; t->right_iq_integrator=m_motor_2.m_iq_integrator;
    t->left_sample_window=m_motor_1.m_sample_zero_window_counts; t->right_sample_window=m_motor_2.m_sample_zero_window_counts;
    t->vin_adc=(uint16_t)(batVoltage>0?batVoltage:0); t->left_fault=(uint8_t)m_motor_1.m_fault; t->right_fault=(uint8_t)m_motor_2.m_fault;
    t->left_quality=foc_trace_quality(&m_motor_1,false); t->right_quality=foc_trace_quality(&m_motor_2,true);
    FOC_MEMORY_BARRIER(); t->guard=g+2u;
    s_foc_trace_head=(uint8_t)((idx+1u)%MCPWM_FOC_TRACE_CAPACITY);
    if(s_foc_trace_count<MCPWM_FOC_TRACE_CAPACITY)s_foc_trace_count++;
    s_foc_trace_write_count++;
    s_foc_trace_cycle_index=idx; FOC_MEMORY_BARRIER(); s_foc_trace_cycle_pending=1u;
}

static void foc_trace_finalize_isr_cycles(uint32_t elapsed) {
    if(!s_foc_trace_cycle_pending)return;
    const uint8_t idx=s_foc_trace_cycle_index;
    if(idx>=MCPWM_FOC_TRACE_CAPACITY){s_foc_trace_cycle_pending=0u;return;}
    mcpwm_foc_trace_sample_t *t=&s_foc_trace[idx];
    uint32_t g=t->guard;if(g&1u)g++;
    t->guard=g+1u;FOC_MEMORY_BARRIER();
    t->isr_cycles=(uint16_t)(elapsed>65535u?65535u:elapsed);
    FOC_MEMORY_BARRIER();t->guard=g+2u;s_foc_trace_cycle_pending=0u;
}


#define OPENLOOP_ERPM_Q16_TO_PHASE_Q32_Q24 \
    ((uint32_t)((((uint64_t)65536u << 24) + (30u * (uint32_t)PWM_FREQ)) / \
                (60u * (uint32_t)PWM_FREQ)))

static void openloop_phase_coeff_recompute(mcpwm_foc_motor_t *m, bool second) {
    if(!m)return;
    const uint32_t den=60u*(uint32_t)PWM_FREQ;
    const uint32_t pp=(uint32_t)motor_pole_pairs(second);
    m->m_openloop_step_per_rpm_q32=(uint32_t)((((uint64_t)pp<<32)+(den/2u))/den);
}

static float motor_gear_ratio(bool second) {
    const float ratio = (second ? m_motor_2.m_conf.si_gear_ratio : m_motor_1.m_conf.si_gear_ratio);
    return (ratio >= 0.01f && ratio <= 1000.0f) ? ratio : 1.0f;
}

static int32_t user_position_to_internal(int32_t position_counts, bool second) {
    if (!second) return position_counts;
    /* Right hardware/Hall direction is mirrored. Avoid UB on -INT32_MIN. */
    return (position_counts == INT32_MIN) ? INT32_MAX : -position_counts;
}

static int32_t internal_position_to_user(int32_t position_counts, bool second) {
    if (!second) return position_counts;
    return (position_counts == INT32_MIN) ? INT32_MAX : -position_counts;
}

static int16_t erpm_to_mech_rpm(float erpm, bool second) {
    /* COMM_SET_RPM authority is electrical RPM, exactly like VESC. The caller
     * already clamps ERPM to l_min_erpm/l_max_erpm. Do not apply the legacy
     * N_MOT_MAX mechanical-RPM clamp here: on LEFT (4 pole-pairs) that turned
     * the configured 15000 ERPM limit into an unintended 4000 ERPM ceiling. */
    const float pp = (float)motor_pole_pairs(second);
    float mech = (pp > 0.0f) ? (erpm / pp) : erpm;
    if (mech > 32767.0f) mech = 32767.0f;
    if (mech < -32767.0f) mech = -32767.0f;
    return (int16_t)(mech >= 0.0f ? mech + 0.5f : mech - 0.5f);
}

static int32_t erpm_to_mech_rpm_q16(float erpm, bool second) {
    /* Same rule as erpm_to_mech_rpm(): ERPM is bounded by mcconf, mechanical
     * conversion is only an internal representation and must not add a second
     * speed limit. 15000 ERPM is 3750 mech RPM on LEFT and 1000 on RIGHT. */
    const float pp = (float)motor_pole_pairs(second);
    float mech = (pp > 0.0f) ? (erpm / pp) : erpm;
    const float scaled = mech * 65536.0f;
    if (scaled >= 2147483520.0f) return INT32_MAX;
    if (scaled <= -2147483520.0f) return INT32_MIN;
    return (int32_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

/* Internal Hall direction must remain in the motor-local FOC convention.
 * User/VESC direction inversion belongs exclusively to mc_interface via
 * m_invert_direction.  Flipping Hall here double-inverts RIGHT feedback and
 * makes the speed PID run away (target -ERPM while feedback appears +ERPM). */
static int8_t hall_motion_direction(bool second, int8_t raw_dir) {
    (void)second;
    return raw_dir;
}

static int32_t measured_mech_rpm_raw_q16(const mcpwm_foc_motor_t *m, bool second) {
    if (encoder_feedback_selected(m,second) && m->m_encoder_configured)
        return m->m_encoder_mech_rpm_q16;
    /* One Hall transition is 60 electrical degrees, i.e. six transitions per
     * electrical revolution. At a 16-kHz estimator tick rate this is
     * ERPM = PWM_FREQ * 10 / hall_period. Keep the mechanical speed in Q16 so
     * a VESC target such as 50 ERPM @15 pole-pairs remains 3.333... RPM instead
     * of being truncated to 3 RPM inside the controller. */
    if (m->m_hall_initialized && m->m_hall_direction != 0 &&
        m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS &&
        m->m_hall_ticks <= MCCONF_HALL_TIMEOUT_TICKS) {
        const uint32_t pp = motor_pole_pairs(second);
        if (pp > 0u) {
            /* Cortex-M3 64-bit integer division is a software helper and was
             * one of the largest speed-loop ISR costs. Q12 mechanical RPM is
             * already far finer than Hall timing resolution, so compute with a
             * single 32-bit divide then promote to Q16. Numerator 16000*10*4096
             * is 655,360,000 and safely fits uint32_t. */
            const uint32_t den = (uint32_t)m->m_hall_period * pp;
            uint32_t mag_q12 = ((uint32_t)PWM_FREQ * 10u * 4096u) / den;
            int32_t q16 = (int32_t)(mag_q12 << 4);
            if (hall_motion_direction(second, m->m_hall_direction) < 0) q16 = -q16;
            return q16;
        }
    }
    return (int32_t)m->m_rpm * 65536;
}

static int32_t measured_mech_rpm_q16(const mcpwm_foc_motor_t *m, bool second) {
    if(m && m->m_conf.s_pid_speed_source==S_PID_SPEED_SRC_PLL && m->m_pll_valid)
        return m->m_pll_mech_rpm_q16;
    return measured_mech_rpm_raw_q16(m,second);
}

static bool encoder_motion_fresh(const mcpwm_foc_motor_t *m, bool second) {
    if (!m || !encoder_feedback_selected(m,second) || !m->m_encoder_configured ||
        !m->m_encoder_synced || m->m_encoder_counts < 4u) return false;
    /* Pada ABI ada banyak edge per revolution. Gunakan umur edge terakhir
     * sebagai batas monoton seperti Hall brake, sehingga RPM estimator 20-ms
     * yang terakhir tidak menahan brake setelah rotor sudah berhenti. */
    uint32_t max_age=((uint32_t)PWM_FREQ*60u) /
        (m->m_encoder_counts*(uint32_t)MCCONF_TRQ_STOP_RPM_DEADBAND);
    if(max_age<1u)max_age=1u;
    return m->m_encoder_idle_ticks<=max_age;
}

static int8_t feedback_motion_direction(const mcpwm_foc_motor_t *m, bool second) {
    if (encoder_feedback_selected(m,second)) {
        if (!encoder_motion_fresh(m,second)) return 0;
        const int32_t lim=(int32_t)MCCONF_TRQ_STOP_RPM_DEADBAND*65536;
        if(m->m_encoder_mech_rpm_q16>lim)return 1;
        if(m->m_encoder_mech_rpm_q16<-lim)return -1;
        return 0;
    }
    if (!m || !m->m_hall_initialized || m->m_hall_direction==0 ||
        m->m_hall_period==0u || m->m_hall_period>=MCCONF_HALL_TIMEOUT_TICKS) return 0;
    uint32_t fresh=(uint32_t)m->m_hall_period*2u;
    if(fresh>MCCONF_HALL_TIMEOUT_TICKS)fresh=MCCONF_HALL_TIMEOUT_TICKS;
    if(m->m_hall_ticks>fresh)return 0;
    if(m->m_rpm>MCCONF_TRQ_STOP_RPM_DEADBAND)return 1;
    if(m->m_rpm<-MCCONF_TRQ_STOP_RPM_DEADBAND)return -1;
    return 0;
}

static int16_t voltage_circle_q_limit(int16_t vd, int16_t vmax) {
    const int32_t d = vd;
    const int32_t max = vmax;
    const uint32_t d2 = (uint32_t)(d * d);
    const uint32_t max2 = (uint32_t)(max * max);
    if (d2 >= max2) return 0;
    return (int16_t)foc_isqrt_u32(max2 - d2);
}

static int16_t current_circle_iq_limit_q4(const mcpwm_foc_motor_t *m, int16_t iq_cmd_q4) {
    /* VESC FOC applies the input-current envelope through q-axis modulation:
     *     Ibus ~= mod_q * Iq
     * not through total duty-vector magnitude. m_vq is the previous/held q-axis
     * modulation in fixed-point form (MCCONF_FOC_VOLTAGE_MAX == mod 1.0), so
     * this stays integer-only in the F103 ISR. The physical DC shunt remains the
     * independent measured-current telemetry and immediate hard-fault layer. */
    int32_t lim = MCCONF_MOTOR_CURRENT_MAX_Q4;
    if (m) {
        if (iq_cmd_q4 < 0 && m->m_current_limit_neg_q4 > 0) lim=m->m_current_limit_neg_q4;
        else if (iq_cmd_q4 >= 0 && m->m_current_limit_q4 > 0) lim=m->m_current_limit_q4;
        /* Upstream mcpwm_foc.c limits Iq with lo_in_current_{min,max}/mod_q.
         * Use the already-computed/held Vq modulation from the preceding control
         * update. A tiny modulation is intentionally ignored, matching VESC's
         * |mod_q| <= 0.001 guard and avoiding an unnecessary ISR divide. */
        const int32_t mod_q=(int32_t)m->m_vq;
        const int32_t amod_q=ABS(mod_q);
        const int32_t mod_q_min=(MCCONF_FOC_VOLTAGE_MAX+999)/1000; /* 0.001 */
        if (amod_q > mod_q_min) {
            const bool drawing=((iq_cmd_q4>=0)==(mod_q>=0));
            int32_t in_lim=drawing?m->m_input_current_max_q4:m->m_input_current_regen_q4;
            /* VESC 6.00 watt limit: I_in <= P_limit / V_in. P/V dikonversi
             * ke Q4 di slow path setiap Vbus/config berubah. ISR hanya memilih
             * nilai cache sehingga worst-case tidak pernah memanggil 64-bit div. */
            const int32_t watt_lim=drawing?m->m_watt_current_max_q4:m->m_watt_current_regen_q4;
            if(watt_lim>0 && watt_lim<in_lim)in_lim=watt_lim;
            if (in_lim > 0) {
                /* Exact fast gate: division Iin/mod_q is only needed when it can
                 * actually tighten the existing motor-current envelope. Compare
                 * cross-products first (all bounds fit int32). At low modulation
                 * this removes a useless Cortex-M3 SDIV without changing one bit
                 * of the limiting result. */
                const int32_t in_num=in_lim*(int32_t)MCCONF_FOC_VOLTAGE_MAX;
                if(in_num < lim*amod_q){
                    const int32_t motor_from_input=in_num/amod_q;
                    if (motor_from_input < lim) lim=motor_from_input;
                }
            }
            /* VESC l_battery_cut_start/end: hanya arus yang mengambil daya dari
             * baterai yang diderating. Semua hitungan di ISR integer/ADC agar
             * Cortex-M3 tidak menjalankan floating point pada loop 16 kHz. */
            if (drawing && m->m_battery_cut_start_adc > m->m_battery_cut_end_adc) {
                const int32_t vb=batVoltage;
                if (vb <= (int32_t)m->m_battery_cut_end_adc) return 0;
                if (vb < (int32_t)m->m_battery_cut_start_adc) {
                    lim=(lim*(vb-(int32_t)m->m_battery_cut_end_adc))/
                        ((int32_t)m->m_battery_cut_start_adc-(int32_t)m->m_battery_cut_end_adc);
                }
            }
        }
    }
    if (m && m->m_temp_fet_end_x10 > m->m_temp_fet_start_x10) {
        const int32_t t=s_board_temperature_x10;
        if (t >= m->m_temp_fet_end_x10) return 0;
        if (t > m->m_temp_fet_start_x10) {
            lim=(lim*((int32_t)m->m_temp_fet_end_x10-t))/
                ((int32_t)m->m_temp_fet_end_x10-(int32_t)m->m_temp_fet_start_x10);
        }
    }
    if (lim < 1) lim=1;
    int32_t id = m ? m->m_id_q4 : 0;
    if (id < 0) id = -id;
    if (id >= lim) return 0;
    const uint32_t lim2 = (uint32_t)(lim * lim);
    const uint32_t id2 = (uint32_t)(id * id);
    const int32_t qmax = (int32_t)foc_isqrt_u32(lim2 - id2);
    return (int16_t)CLAMP((int32_t)iq_cmd_q4, -qmax, qmax);
}


static void driven_offset_finalize_non_isr(void);

#ifdef STM32F103xE
static void encoder_gpio_diag_sample_non_isr(void) {
    const bool commissioning=(encoder_detect_stage!=0u) ||
        (encoder_align_stage!=0u && encoder_align_stage!=9u);
    if(!commissioning)return;
    const uint32_t idr=GPIOB->IDR;
    const uint8_t ab=(uint8_t)(((idr & GPIO_PIN_6)?1u:0u) | ((idr & GPIO_PIN_7)?2u:0u));
    const uint8_t pb5=(idr & GPIO_PIN_5)?1u:0u;
    const uint8_t ch=(uint8_t)(ab ^ encoder_gpio_last_ab);
    if(ch & 1u)encoder_gpio_edge_a++;
    if(ch & 2u)encoder_gpio_edge_b++;
    if(pb5!=encoder_gpio_last_pb5)encoder_gpio_edge_pb5++;
    encoder_gpio_last_ab=ab; encoder_gpio_last_pb5=pb5; encoder_gpio_samples++;
}
#endif

static void foc_bounded_delay_ms(uint32_t ms) {
    if (ms == 0u) return;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0u) {
        while(ms-- > 0u){
            HAL_Delay(1u);
#ifdef STM32F103xE
            encoder_gpio_diag_sample_non_isr();
#endif
            driven_offset_finalize_non_isr();
            platform_watchdog_service();
        }
        return;
    }
    const uint32_t cycles_per_ms = 64000u;
    while (ms-- > 0u) {
        const uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles_per_ms) { (void)DWT->CYCCNT; }
        /* Detect/encoder-align adalah worker blocking di main context. Sampling
         * GPIO diagnostik dan finalisasi offset dilakukan di sini, bukan ISR. */
#ifdef STM32F103xE
        encoder_gpio_diag_sample_non_isr();
#endif
        driven_offset_finalize_non_isr();
        platform_watchdog_service();
    }
}

static void foc_isr_monitor_end(uint32_t start, uint32_t post_start, uint8_t detail_sample) {
    const uint32_t end = DWT->CYCCNT;
    const uint32_t elapsed = end - start;
    if(detail_sample){
        const uint32_t post=end-post_start;
        if(post>foc_prof_post_max_cycles)foc_prof_post_max_cycles=post;
    }
#ifdef STM32F103xE
    /* Read-only backlog evidence: after TCIF1 was cleared at ISR entry, a set
     * TC flag or NVIC pending bit here means the next ADC frame arrived before
     * this handler completed. Do not clear either flag here. */
    if((DMA1->ISR & DMA_ISR_TCIF1)!=0u)foc_irq_dma_tc_pending_exit_count++;
#endif
    foc_trace_finalize_isr_cycles(elapsed);
    foc_irq_exit_count++;
    foc_isr_cycles = elapsed;
    if (elapsed > foc_isr_cycles_max) foc_isr_cycles_max = elapsed;
    if (elapsed > FOC_ISR_BUDGET_CYCLES) foc_isr_deadline_miss_count++;
    /* Slot profiler memakai timestamp entry/exit yang memang sudah ada. Tidak
     * ada DWT read tambahan di hot path; startup diberi slot 0xff dan tidak
     * dicampur dengan statistik scheduler steady-state. */
    const uint8_t slot=foc_isr_profile_slot;
    if(slot<MCCONF_FOC_CONTROL_DIV){
        foc_isr_slot_count[slot]++;
        if(elapsed>foc_isr_slot_max_cycles[slot])foc_isr_slot_max_cycles[slot]=elapsed;
        if(elapsed>FOC_ISR_BUDGET_CYCLES)foc_isr_slot_miss_count[slot]++;
    }
}


static void foc_isr_profile_clear_isr_owned(void) {
    foc_isr_cycles_max=0u; foc_isr_deadline_miss_count=0u; foc_isr_profile_slot=0xffu;
    foc_prof_sample_down=0u; foc_prof_detail_sample=0u; foc_prof_detail_sample_count=0u;
    foc_isr_steady_count=0u; foc_isr_slot_sequence_error_count=0u; foc_isr_expected_slot=0xffu;
    for(uint8_t i=0u;i<MCPWM_FOC_PROFILE_SLOT_CAPACITY;++i){foc_isr_slot_max_cycles[i]=0u;foc_isr_slot_miss_count[i]=0u;foc_isr_slot_count[i]=0u;foc_prof_detail_slot_count[i]=0u;}
    foc_prof_pre_max_cycles=foc_prof_control_max_cycles=foc_prof_post_max_cycles=0u;
    foc_prof_pre_gate_max_cycles=foc_prof_pre_offset_max_cycles=foc_prof_pre_protect_max_cycles=0u;
    for(uint8_t i=0u;i<2u;++i){foc_prof_motor_step_max_cycles[i]=0u;foc_prof_motor_control_max_cycles[i]=0u;foc_prof_motor_hold_max_cycles[i]=0u;}
    foc_prof_sensor_max_cycles=foc_prof_current_max_cycles=foc_prof_regulator_max_cycles=foc_prof_svpwm_max_cycles=0u;
    foc_prof_pll_max_cycles=foc_prof_position_pid_max_cycles=foc_prof_speed_pid_max_cycles=0u;
    foc_prof_current_circle_max_cycles=foc_prof_id_pi_max_cycles=foc_prof_iq_pi_max_cycles=0u;
    foc_prof_decouple_limit_max_cycles=foc_prof_duty_mag_max_cycles=foc_prof_fast_hold_svpwm_max_cycles=0u;
    m_motor_1.m_overrun_count=0u; m_motor_2.m_overrun_count=0u;
}

bool mcpwm_foc_reset_isr_profile(void) {
    outer_control_max_cycles=0u; outer_control_miss_count=0u; outer_control_jitter_max_cycles=0u;
    outer_control_period_min_cycles=UINT32_MAX; outer_control_period_max_cycles=0u;
#if defined(__arm__) || defined(__thumb__)
    const uint32_t req=foc_prof_reset_request+1u; foc_prof_reset_request=req; FOC_MEMORY_BARRIER();
    if((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)==0u){foc_isr_profile_clear_isr_owned();foc_prof_reset_ack=req;return true;}
    const uint32_t start=DWT->CYCCNT;
    while(foc_prof_reset_ack!=req){ if((uint32_t)(DWT->CYCCNT-start)>(FOC_ISR_BUDGET_CYCLES*16u))return false; }
    return true;
#else
    const uint32_t req=foc_prof_reset_request+1u; foc_prof_reset_request=req; foc_isr_profile_clear_isr_owned(); foc_prof_reset_ack=req; return true;
#endif
}
void mcpwm_foc_get_irq_epoch(uint32_t *entry, uint32_t *exit) {
    FOC_MEMORY_BARRIER();
    if(entry)*entry=foc_irq_entry_count;
    if(exit)*exit=foc_irq_exit_count;
    FOC_MEMORY_BARRIER();
}

void mcpwm_foc_get_isr_profile(mcpwm_foc_isr_profile_t *out) {
    if(!out)return;
    /* Main-context read can be preempted by the priority-0 DMA ISR. Retry the
     * entire snapshot if an IRQ entered while any field was copied; this keeps
     * max/count/stage data from different frames out of one diagnostic record
     * without ever masking the motor interrupt. */
    uint32_t e0=0u,x0=0u,e1=0u,x1=0u;
    bool snapshot_ok=false;
    for(uint8_t retry=0u; retry<64u; ++retry) {
        mcpwm_foc_get_irq_epoch(&e0,&x0);
        if(e0!=x0)continue;
        out->total_max_cycles=foc_isr_cycles_max; out->deadline_miss_count=foc_isr_deadline_miss_count;
        out->pre_max_cycles=foc_prof_pre_max_cycles; out->control_max_cycles=foc_prof_control_max_cycles; out->post_max_cycles=foc_prof_post_max_cycles;
        out->pre_gate_max_cycles=foc_prof_pre_gate_max_cycles; out->pre_offset_max_cycles=foc_prof_pre_offset_max_cycles; out->pre_protect_max_cycles=foc_prof_pre_protect_max_cycles;
        for(uint8_t i=0u;i<2u;++i){out->motor_step_max_cycles[i]=foc_prof_motor_step_max_cycles[i];out->motor_control_max_cycles[i]=foc_prof_motor_control_max_cycles[i];out->motor_hold_max_cycles[i]=foc_prof_motor_hold_max_cycles[i];}
        out->sensor_max_cycles=foc_prof_sensor_max_cycles; out->pll_max_cycles=foc_prof_pll_max_cycles; out->current_max_cycles=foc_prof_current_max_cycles; out->regulator_max_cycles=foc_prof_regulator_max_cycles;
        out->position_pid_max_cycles=foc_prof_position_pid_max_cycles; out->speed_pid_max_cycles=foc_prof_speed_pid_max_cycles; out->current_circle_max_cycles=foc_prof_current_circle_max_cycles;
        out->id_pi_max_cycles=foc_prof_id_pi_max_cycles; out->iq_pi_max_cycles=foc_prof_iq_pi_max_cycles; out->decouple_limit_max_cycles=foc_prof_decouple_limit_max_cycles;
        out->svpwm_max_cycles=foc_prof_svpwm_max_cycles; out->duty_mag_max_cycles=foc_prof_duty_mag_max_cycles; out->reentry_guard_total=m_motor_1.m_overrun_count+m_motor_2.m_overrun_count;
        for(uint8_t i=0u;i<6u;++i){out->slot_max_cycles[i]=foc_isr_slot_max_cycles[i];out->slot_miss_count[i]=foc_isr_slot_miss_count[i];out->slot_count[i]=foc_isr_slot_count[i];}
        out->outer_max_cycles=outer_control_max_cycles; out->outer_miss_count=outer_control_miss_count;
        out->outer_jitter_max_cycles=outer_control_jitter_max_cycles;
        out->outer_period_min_cycles=(outer_control_period_min_cycles==UINT32_MAX)?0u:outer_control_period_min_cycles;
        out->outer_period_max_cycles=outer_control_period_max_cycles;
        out->adc_heartbeat=foc_adc_heartbeat;
        out->motor_heartbeat[0]=foc_motor_heartbeat[0]; out->motor_heartbeat[1]=foc_motor_heartbeat[1];
        out->snapshot_dwt=DWT->CYCCNT; out->irq_entry_count=e0; out->irq_exit_count=x0;
        out->motor_step_count[0]=m_motor_1.m_isr_count; out->motor_step_count[1]=m_motor_2.m_isr_count;
        out->dma_tc_pending_exit_count=foc_irq_dma_tc_pending_exit_count;
        out->detail_sample_count=foc_prof_detail_sample_count;
        for(uint8_t i=0u;i<6u;++i)out->detail_slot_count[i]=foc_prof_detail_slot_count[i];
        out->steady_isr_count=foc_isr_steady_count; out->slot_sequence_error_count=foc_isr_slot_sequence_error_count;
        out->fast_hold_svpwm_max_cycles=foc_prof_fast_hold_svpwm_max_cycles;
        out->profile_revision=FOC_ISR_PROFILE_REVISION; out->active_slot_count=MCCONF_FOC_CONTROL_DIV; out->reset_epoch=foc_prof_reset_ack;
        mcpwm_foc_get_irq_epoch(&e1,&x1);
        if(e0==e1 && x0==x1 && e1==x1){snapshot_ok=true;break;}
    }
    if(!snapshot_ok){
        memset(out,0,sizeof(*out));
        out->profile_revision=0u;
    }
}

/* Normalisasi fitur generic VESC yang memang tidak tersedia pada hardware F103.
 * Prinsipnya: konfigurasi harus benar-benar bekerja atau dibaca kembali sebagai
 * mode disabled/fixed. Tidak boleh ada opsi "diterima tetapi diam-diam diabaikan". */
static void f103_mcconf_canonicalize_unsupported(mc_configuration *c) {
    if (!c) return;
    c->pwm_mode=PWM_MODE_SYNCHRONOUS;
    c->comm_mode=COMM_MODE_INTEGRATE;
    c->motor_type=MOTOR_TYPE_FOC;
    /* PWM/ADC cadence adalah properti hardware F103, bukan parameter runtime.
     * foc_dt_us tetap field VESC yang nyata: pada board tanpa phase-voltage ADC
     * ia dipakai untuk mengoreksi model tegangan observer, bukan command SVPWM.
     * Quantize 1 ns supaya readback == nilai yang benar-benar dipersist/dipakai. */
    c->foc_f_zv=(float)PWM_FREQ;
    if (!(c->foc_dt_us >= 0.0f && c->foc_dt_us <= MCCONF_FOC_DT_US_MAX)) {
        c->foc_dt_us=MCCONF_FOC_DT_US_DEFAULT;
    } else {
        uint32_t dt_ns=(uint32_t)(c->foc_dt_us*1000.0f+0.5f);
        if(dt_ns>MCCONF_FOC_DT_NS_MAX)dt_ns=MCCONF_FOC_DT_NS_MAX;
        c->foc_dt_us=(float)dt_ns*0.001f;
    }
    c->foc_overmod_factor=1.0f;
    c->foc_mag_vd_max=1.0f;
    c->foc_control_sample_mode=FOC_CONTROL_SAMPLE_MODE_V0;
    c->foc_current_sample_mode=FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO;
    c->foc_sat_comp_mode=SAT_COMP_DISABLED;
    c->foc_sat_comp=0.0f;
    c->foc_temp_comp=false;
    c->foc_observer_type=FOC_OBSERVER_ORTEGA_ORIGINAL;
    c->foc_phase_filter_enable=false;
    c->foc_phase_filter_disable_fault=true;
    c->foc_phase_filter_max_erpm=0.0f;
    c->foc_mtpa_mode=MTPA_MODE_OFF;
    c->foc_fw_current_max=0.0f;
    c->foc_fw_duty_start=1.0f;
    c->foc_fw_ramp_time=0.0f;
    c->foc_fw_q_current_factor=0.0f;
    c->foc_fw_backoff=0.0f;
    c->foc_speed_soure=FOC_SPEED_SRC_CORRECTED;
    c->sp_pid_loop_rate=PID_RATE_1000_HZ;
    c->m_motor_temp_sens_type=TEMP_SENSOR_DISABLED;
    c->m_out_aux_mode=OUT_AUX_MODE_OFF;
    c->foc_hfi_voltage_start=0.0f;
    c->foc_hfi_voltage_run=0.0f;
    c->foc_hfi_voltage_max=0.0f;
    c->foc_hfi_gain=0.0f;
    c->foc_hfi_hyst=0.0f;
    c->foc_sl_erpm_hfi=0.0f;
    c->foc_hfi_start_samples=0u;
    c->foc_hfi_obs_ovr_sec=0.0f;
    c->foc_hfi_samples=0u;
}

static void conf_defaults(mc_configuration *c, bool second) {
    memset(c, 0, sizeof(*c));
    f103_mcconf_canonicalize_unsupported(c);
    c->motor_type = MOTOR_TYPE_FOC;
    c->sensor_mode = SENSOR_MODE_SENSORED;
    /* Project hardware is mixed-sensor: LEFT steering uses the 4096-count ABI
     * encoder on PB6/PB7; RIGHT traction remains Hall. Detect-All may refine
     * offset/ratio/inversion, but a blank EEPROM must boot with the right port. */
    c->foc_sensor_mode = second ? FOC_SENSOR_MODE_HALL : FOC_SENSOR_MODE_ENCODER;
    c->l_current_max = MCCONF_L_CURRENT_MAX;
    c->l_current_min = MCCONF_L_CURRENT_MIN;
    c->l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
    /* This two-low-side-shunt board cannot safely use reconstructed raw phase
     * samples as an ABS source near switching boundaries. "Fast" therefore
     * means the FOC feedback D/Q current; "slow" means the additional VESC
     * monitoring LPF. Both preserve the VESC setting semantics without using an
     * unobservable raw phase as a fault source. */
    c->l_slow_abs_current = false;
    c->l_min_duty = MCCONF_L_MIN_DUTY;
    c->l_max_duty = MCCONF_L_MAX_DUTY;
    c->m_fault_stop_time_ms = (int32_t)MCCONF_FAULT_STOP_TIME_MS;
    c->m_duty_ramp_step = MCCONF_DUTY_RAMP_STEP_DEFAULT;
    c->cc_min_current = MCCONF_CC_MIN_CURRENT;
    c->foc_duty_dowmramp_kp = MCCONF_FOC_DUTY_DOWNRAMP_KP;
    c->foc_duty_dowmramp_ki = MCCONF_FOC_DUTY_DOWNRAMP_KI;
    c->l_in_current_max = MCCONF_L_IN_CURRENT_MAX;
    c->l_in_current_min = MCCONF_L_IN_CURRENT_MIN;
    c->l_in_current_map_start = MCCONF_L_IN_CURRENT_MAP_START;
    c->l_in_current_map_filter = MCCONF_L_IN_CURRENT_MAP_FILTER;
    c->l_current_max_scale = MCCONF_L_CURRENT_MAX_SCALE;
    c->l_current_min_scale = MCCONF_L_CURRENT_MIN_SCALE;
    c->l_erpm_start = MCCONF_L_ERPM_START;
    c->l_duty_start = MCCONF_L_DUTY_START;
    c->l_temp_accel_dec = MCCONF_L_TEMP_ACCEL_DEC;
    c->l_additional_faults = 0;
    c->l_battery_cut_start = MCCONF_L_BATTERY_CUT_START;
    c->l_battery_cut_end = MCCONF_L_BATTERY_CUT_START > MCCONF_L_BATTERY_CUT_END ?
                         MCCONF_L_BATTERY_CUT_END : MCCONF_L_BATTERY_CUT_START;
    c->l_battery_regen_cut_start = MCCONF_L_BATTERY_REGEN_CUT_START;
    c->l_battery_regen_cut_end = MCCONF_L_BATTERY_REGEN_CUT_END;
    c->l_min_vin = MCCONF_L_MIN_VIN;
    c->l_max_vin = MCCONF_L_MAX_VIN;
    c->l_temp_fet_start = MCCONF_L_TEMP_FET_START;
    c->l_temp_fet_end = MCCONF_L_TEMP_FET_END;
    c->l_temp_motor_start = MCCONF_L_TEMP_MOTOR_START;
    c->l_temp_motor_end = MCCONF_L_TEMP_MOTOR_END;
    c->m_sensor_port_mode = second ? SENSOR_PORT_MODE_HALL : SENSOR_PORT_MODE_ABI;
    /* Hardware default only: the right bridge/motor installation is mirrored.
     * VESC packets remain motor-local; mc_interface DIR_MULT applies this
     * standard Motor Configuration field for motor thread 2. */
    c->m_invert_direction = second;
    c->m_motor_temp_sens_type = TEMP_SENSOR_DISABLED;
    c->l_watt_max = MCCONF_L_WATT_MAX;
    c->l_watt_min = MCCONF_L_WATT_MIN;
    c->l_max_erpm = MCCONF_L_MAX_ERPM;
    c->l_min_erpm = MCCONF_L_MIN_ERPM;
    /* Expose VESC configuration in physical units. The ISR remains fixed-point:
     * Kp ~= 0.800 V/A and Ki ~= 266.7 V/(A*s) at the 2.667-kHz control cadence. */
    c->foc_current_kp = 0.80013f;
    c->foc_current_ki = 266.710f;
    c->foc_current_filter_const = MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
    c->foc_pll_kp = MCCONF_FOC_PLL_KP_DEFAULT;
    c->foc_pll_ki = MCCONF_FOC_PLL_KI_DEFAULT;
    /* Fitur yang sudah mempunyai runtime nyata tetap default konservatif.
     * PLL tersedia sebagai source speed pilihan; decoupling harus diaktifkan
     * eksplisit setelah R/L/flux motor teridentifikasi. */
    c->foc_cc_decoupling = FOC_CC_DECOUPLING_DISABLED;
    /* Fitur generic VESC yang tidak mempunyai jalur hardware pada hoverboard
     * F103 dikunci ke mode aman. Ini mencegah VESC Tool menampilkan seolah-olah
     * sebuah opsi aktif padahal runtime tidak pernah mengeksekusinya. */
    c->foc_control_sample_mode = FOC_CONTROL_SAMPLE_MODE_V0;
    c->foc_current_sample_mode = FOC_CURRENT_SAMPLE_MODE_LONGEST_ZERO;
    c->foc_sat_comp_mode = SAT_COMP_DISABLED;
    c->foc_sat_comp = 0.0f;
    c->foc_temp_comp = false;
    c->foc_phase_filter_enable = false;
    c->foc_phase_filter_disable_fault = true;
    c->foc_mtpa_mode = MTPA_MODE_OFF;
    c->foc_fw_current_max = 0.0f;
    c->foc_fw_duty_start = 1.0f;
    c->foc_fw_ramp_time = 0.0f;
    c->foc_fw_q_current_factor = 0.0f;
    c->foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
    /* VESC 6.00 defaults for the independent FOC rotor observer. The model
     * R/L/flux and main gain are populated by motor detection or MC config. */
    c->foc_observer_type = FOC_OBSERVER_ORTEGA_ORIGINAL;
    c->foc_observer_gain_slow = 0.05f;
    c->foc_observer_offset = 0.0f;
    c->foc_encoder_offset = MCCONF_ENCODER_OFFSET_DEFAULT;
    c->foc_encoder_inverted = false;
    c->foc_encoder_ratio = (float)default_motor_poles(second) * 0.5f;
    c->m_encoder_counts = (int32_t)MCCONF_ENCODER_COUNTS_DEFAULT;
    c->foc_openloop_rpm = (float)MCCONF_OPENLOOP_RPM_DEFAULT;
    c->foc_hall_interp_erpm = (float)MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT;
    c->m_hall_extra_samples = (int)MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT;
    c->s_pid_ramp_erpms_s = (float)MCCONF_SPEED_RAMP_ERPMS_S;
    c->s_pid_min_erpm = (float)MCCONF_SPEED_RELEASE_ERPM;
    c->s_pid_allow_braking = true;
    c->s_pid_kd_filter = MCCONF_SPEED_KD_FILTER_DEFAULT;
    c->s_pid_speed_source = S_PID_SPEED_SRC_FAST;
    c->s_pid_kp = (float)MCCONF_SPEED_KP_Q11 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->s_pid_ki = (float)MCCONF_SPEED_KI_Q16 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->s_pid_kd = (float)MCCONF_SPEED_KD_Q11 / (float)MCCONF_SPEED_GAIN_SCALE;
    c->p_pid_kp = (float)MCCONF_POSITION_KP_Q11 / 1000.0f;
    c->p_pid_ki = (float)MCCONF_POSITION_KI_Q16 / 1000.0f;
    c->p_pid_kd = (float)MCCONF_POSITION_KD_Q11 / 1000.0f;
    c->p_pid_kd_filter = (float)MCCONF_POSITION_KD_FILTER_Q16 / 65536.0f;
    c->p_pid_kd_proc = 0.00035f; /* upstream VESC default process-D damping */
    c->p_pid_ang_div = 1.0f;
    c->p_pid_gain_dec_angle = 0.0f;
    c->si_motor_poles = (uint8_t)default_motor_poles(second);
    c->si_gear_ratio = 1.0f; /* direct drive; set >0 in VESC Tool for a gearbox */
    c->si_wheel_diameter = MCCONF_SI_WHEEL_DIAMETER;
    c->si_battery_type = BATTERY_TYPE_LIION_3_0__4_2;
    c->si_battery_cells = BAT_CELLS;
    c->si_battery_ah = 0.0f; /* isi kapasitas aktual di VESC Tool bila ingin estimasi Wh tersisa */
    for (int i=0;i<8;i++) c->foc_hall_table[i] = (int8_t)s_default_foc_hall_table[i];
}

void mcpwm_foc_get_default_configuration(mc_configuration *conf, bool second) {
    if (conf) conf_defaults(conf, second);
}

static void hall_interp_recompute(mcpwm_foc_motor_t *m) {
    if (!m) return;
    float erpm_f=m->m_conf.foc_hall_interp_erpm;
    if (!(erpm_f >= 0.0f && erpm_f <= MCCONF_L_MAX_ERPM))
        erpm_f=(float)MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT;
    uint32_t erpm=(uint32_t)(erpm_f+0.5f);
    if (erpm>65535u) erpm=65535u;
    m->m_hall_interp_erpm=(uint16_t)erpm;
    {
        int extra=m->m_conf.m_hall_extra_samples;
        if(extra<0 || extra>20)extra=(int)MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT;
        const uint8_t window=(uint8_t)(1u+2u*(uint32_t)extra);
        m->m_hall_filter_delay_ticks=(uint8_t)extra; /* majority edge delay = N frames */
        if(m->m_hall_filter_window!=window){
            m->m_hall_filter_window=window;
            m->m_hall_sample_index=0u; m->m_hall_sample_count=0u;
            m->m_hall_sample_sum_u=0u; m->m_hall_sample_sum_v=0u; m->m_hall_sample_sum_w=0u;
        }
    }

    /* VESC foc_correct_hall(): interpolation is disabled when
     * 10*PWM_FREQ/max(edge_age,last_period) falls below foc_hall_interp_erpm.
     * Precompute the equivalent tick boundary so the 16-kHz ISR needs no divide. */
    if(erpm==0u){
        m->m_hall_interp_max_ticks=MCCONF_HALL_TIMEOUT_TICKS;
        m->m_hall_rate_min_step=1u;
    }else{
        uint32_t ticks=((uint32_t)PWM_FREQ*10u)/erpm;
        if(ticks<1u)ticks=1u;
        if(ticks>MCCONF_HALL_TIMEOUT_TICKS)ticks=MCCONF_HALL_TIMEOUT_TICKS;
        m->m_hall_interp_max_ticks=(uint16_t)ticks;
        /* Upstream rate limiter: max(hall_erpm, interp_erpm) * 1.5. */
        uint32_t step=((uint64_t)erpm*65536u*3u)/(60u*(uint32_t)PWM_FREQ*2u);
        if(step<1u)step=1u;
        if(step>32767u)step=32767u;
        m->m_hall_rate_min_step=(uint16_t)step;
    }
}

void mcpwm_foc_refresh_hall_interpolation(bool second) {
    hall_interp_recompute(mcpwm_foc_get_motor(second));
}

static bool encoder_port_active(const mcpwm_foc_motor_t *m, bool second) {
    return !second && m && m->m_conf.m_sensor_port_mode == SENSOR_PORT_MODE_ABI;
}

static bool encoder_feedback_selected(const mcpwm_foc_motor_t *m, bool second) {
    if (!encoder_port_active(m, second)) return false;
    return m->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER ||
           m->m_conf.foc_sensor_mode == FOC_SENSOR_MODE_ENCODER_AB;
}

static float encoder_norm_deg(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    return deg;
}

static void position_ang_div_recompute(mcpwm_foc_motor_t *m) {
    if(!m)return;
    float div=m->m_conf.p_pid_ang_div;
    if(!(div>=0.01f && div<=1000.0f))div=1.0f;
    m->m_conf.p_pid_ang_div=div;
    /* Float32 cukup: div minimum 0,01 membuat q maksimum 6,55e6, masih
     * jauh di bawah area presisi yang dapat memengaruhi satu count Q16. */
    float q=65536.0f/div;
    if(q<1.0f)q=1.0f;
    if(q>6553600.0f)q=6553600.0f;
    m->m_pos_pid_ang_div_inv_q16=(uint32_t)(q+0.5f);
    m->m_pos_pid_div_accum=0u;
    m->m_pos_pid_raw_last=0u;
    m->m_pos_pid_feedback_phase=0u;
}

static void position_feedback_update(mcpwm_foc_motor_t *m, uint16_t raw) {
    if(!m)return;
    /* p_pid_ang_div is converted to inverse Q16 whenever config changes. Do not
     * execute Cortex-M3 soft-float comparisons in the 16-kHz ADC ISR. The
     * integer interval below is exactly the rounded inverse of 0.98<div<1.02. */
    const uint32_t step=m->m_pos_pid_ang_div_inv_q16?m->m_pos_pid_ang_div_inv_q16:65536u;
    if(step>=64251u && step<=66873u){
        m->m_pos_pid_feedback_phase=raw;
        m->m_pos_pid_raw_last=raw;
        m->m_pos_pid_div_accum=0u;
        return;
    }
    const uint16_t last=m->m_pos_pid_raw_last;
    if(raw<16384u && last>49152u)m->m_pos_pid_div_accum=(uint16_t)(m->m_pos_pid_div_accum+step);
    else if(raw>49152u && last<16384u)m->m_pos_pid_div_accum=(uint16_t)(m->m_pos_pid_div_accum-step);
    m->m_pos_pid_raw_last=raw;
    const uint32_t scaled=(uint32_t)(((uint64_t)raw*step)>>16);
    m->m_pos_pid_feedback_phase=(uint16_t)(m->m_pos_pid_div_accum+scaled);
}

static void encoder_runtime_configure(mcpwm_foc_motor_t *m, bool second, bool reinitialize) {
    if (!m) return;
    if (!encoder_port_active(m, second)) {
        if (!second && encoder_is_configured() == ENCODER_TYPE_ABI) encoder_deinit();
        m->m_encoder_configured=0u; m->m_encoder_synced=0u;
        m->m_encoder_erpm_q16=0; m->m_encoder_mech_rpm_q16=0;
        return;
    }

    if (reinitialize || encoder_is_configured() != ENCODER_TYPE_ABI) {
        if (!encoder_init(&m->m_conf)) {
            m->m_encoder_configured=0u; m->m_encoder_synced=0u;
            return;
        }
        m->m_encoder_synced=0u;
        m->m_position_counts=0; m->m_position_abs_counts=0u;
    } else {
        encoder_update_config(&m->m_conf);
    }

    uint32_t counts=(uint32_t)m->m_conf.m_encoder_counts;
    if(counts<4u)counts=MCCONF_ENCODER_COUNTS_DEFAULT;
    if(counts>65536u)counts=65536u;
    m->m_encoder_counts=counts;
    m->m_encoder_count_to_phase_q16=(uint32_t)((1ULL<<32)/counts);
    m->m_encoder_mdeg_per_count_q12=(uint32_t)((((uint64_t)360000u<<12)+(counts/2u))/counts);
    m->m_encoder_mech_rpm_coeff_q3=(uint32_t)((60ULL*(uint64_t)PWM_FREQ*8ULL+counts/2u)/counts);
    {
        const uint64_t num=(uint64_t)counts*(uint64_t)MCCONF_ENCODER_FAULT_MAX_RPM;
        const uint32_t den=60u*(uint32_t)PWM_FREQ;
        uint32_t d=(uint32_t)((num+den-1u)/den)+MCCONF_ENCODER_FAULT_DELTA_MARGIN_COUNTS;
        if(d<1u)d=1u;
        if(d>UINT16_MAX)d=UINT16_MAX;
        m->m_encoder_max_delta_per_tick=(uint16_t)d;
    }
    float ratio=m->m_conf.foc_encoder_ratio;
    if(!(ratio>=0.01f && ratio<=MCCONF_ENCODER_RATIO_MAX)) ratio=(float)motor_pole_pairs(false);
    m->m_encoder_ratio_q16=(uint32_t)(ratio*65536.0f+0.5f);
    const float ofs=encoder_norm_deg(m->m_conf.foc_encoder_offset);
    m->m_encoder_offset_phase=(uint16_t)(ofs*(65536.0f/360.0f)+0.5f);
    m->m_encoder_raw_count=encoder_read_raw_count();
    m->m_encoder_prev_count=m->m_encoder_raw_count;
#ifdef STM32F103xE
    if(!second && reinitialize){
        const uint32_t idr=GPIOB->IDR;
        encoder_gpio_edge_a=0u; encoder_gpio_edge_b=0u; encoder_gpio_edge_pb5=0u; encoder_gpio_samples=0u;
        encoder_gpio_last_ab=(uint8_t)(((idr & GPIO_PIN_6)?1u:0u) | ((idr & GPIO_PIN_7)?2u:0u));
        encoder_gpio_last_pb5=(idr & GPIO_PIN_5)?1u:0u;
    }
#endif
    m->m_encoder_delta_accum=0; m->m_encoder_speed_ticks=0u; m->m_encoder_idle_ticks=0u;
    /* Rebase counter dan speed harus atomik secara semantik. Membawa RPM lama
     * ke config/sync baru akan masuk ke speed PID dan p_pid_kd_proc sebagai
     * derivative palsu selama satu timeout estimator. */
    m->m_encoder_erpm_q16=0; m->m_encoder_mech_rpm_q16=0; m->m_rpm=0;
    m->m_encoder_configured=1u;
}

void mcpwm_foc_refresh_encoder_configuration(bool second, bool reinitialize) {
    encoder_runtime_configure(mcpwm_foc_get_motor(second),second,reinitialize);
}

void mcpwm_foc_refresh_position_configuration(bool second) {
    position_ang_div_recompute(mcpwm_foc_get_motor(second));
}

static void encoder_runtime_set_deg(mcpwm_foc_motor_t *m, float deg) {
    if(!m || !m->m_encoder_configured)return;
    /* encoder_set_deg() menulis TIM4->CNT. ISR 16 kHz dapat membaca counter
     * pada saat yang sama, jadi sinkronkan seluruh delta/speed tracker secara
     * atomik agar software-zero tidak pernah terlihat sebagai gerakan fisik. */
    __disable_irq();
    encoder_set_deg(deg);
    const uint32_t cnt=encoder_read_raw_count();
    m->m_encoder_raw_count=cnt;
    m->m_encoder_prev_count=cnt;
    m->m_encoder_delta_accum=0;
    m->m_encoder_speed_ticks=0u;
    m->m_encoder_idle_ticks=0u;
    m->m_encoder_erpm_q16=0;
    m->m_encoder_mech_rpm_q16=0;
    m->m_rpm=0;
    __enable_irq();
}

static uint16_t position_feedback_phase_u16(const mcpwm_foc_motor_t *m, bool second) {
    if(!m)return 0u;
    /* Hall + default p_pid_ang_div=1 uses the live FOC/Hall phase directly,
     * matching upstream fallback state_now->phase and avoiding redundant 16-kHz
     * copies. Encoder or non-unity angular division uses the scaled cache. */
    const bool enc_active=!second && encoder_port_active(m,false) && m->m_encoder_configured;
    const uint32_t step=m->m_pos_pid_ang_div_inv_q16?m->m_pos_pid_ang_div_inv_q16:65536u;
    if(!enc_active && step>=64251u && step<=66873u)return m->m_phase;
    return m->m_pos_pid_feedback_phase;
}

static int32_t position_error_sign(const mcpwm_foc_motor_t *m, bool second) {
    return (!second && encoder_port_active(m,false) && m->m_encoder_configured &&
            m->m_conf.foc_encoder_inverted) ? -1 : 1;
}

static void current_pid_recompute_coeff(mcpwm_foc_motor_t *m) {
    if(!m)return;
    /* Legacy/custom integer tuning fields remain wire-compatible, but their
     * execution is now the upstream VESC physical PI equation.
     * Kp field: physical Kp = raw/1536 V/A.
     * Ki field: physical Ki = raw/4.608 V/(A*s). */
    m->m_current_kpq_v_q16=(uint32_t)(((uint64_t)m->m_kpq_q11*128u+1u)/3u);
    m->m_current_kpd_v_q16=(uint32_t)(((uint64_t)m->m_kpd_q11*128u+1u)/3u);
    m->m_current_kiq_dt_v_q16=(uint32_t)(((uint64_t)m->m_kiq_q16*MCCONF_FOC_CONTROL_DIV*8u+4u)/9u);
    m->m_current_kid_dt_v_q16=(uint32_t)(((uint64_t)m->m_kid_q16*MCCONF_FOC_CONTROL_DIV*8u+4u)/9u);
    /* p_v_q16 = err_q4 * gain_v_q16 / 800. Store gain*256/800
     * once here so current_pi_vesc_state() can use a signed 64-bit multiply
     * followed by >>8. Error versus the exact division is below 1 Q16-LSB per
     * current-count and is covered by the host current-loop regression. */
    m->m_current_kpq_err_q8=(uint32_t)(((uint64_t)m->m_current_kpq_v_q16*8u+12u)/25u);
    m->m_current_kpd_err_q8=(uint32_t)(((uint64_t)m->m_current_kpd_v_q16*8u+12u)/25u);
    m->m_current_kiq_err_q8=(uint32_t)(((uint64_t)m->m_current_kiq_dt_v_q16*8u+12u)/25u);
    m->m_current_kid_err_q8=(uint32_t)(((uint64_t)m->m_current_kid_dt_v_q16*8u+12u)/25u);
}

static void speed_pid_recompute_coeff(mcpwm_foc_motor_t *m) {
    if (!m) return;
    const uint32_t lim=(uint32_t)(m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4);
    /* Fixed-point exact terhadap persamaan VESC pada dt dasar 1 ms:
     * P = error * Kp / 20
     * I += error * Ki * dt / 20
     * D = d(error) * Kd / dt / 20
     * error_q2 = ERPM*4. Tidak ada float/divide runtime di outer PID. */
    uint64_t v=((uint64_t)m->m_kps_q11*268435456ULL + 500000ULL)/1000000ULL;
    m->m_speed_kp_coeff_q16=(v>UINT32_MAX)?UINT32_MAX:(uint32_t)v;
    /* 536870912 / 1e9 adalah faktor Q16 integrator untuk dt=1 ms. */
    v=((uint64_t)m->m_kis_q16*lim*536870912ULL + 500000000ULL)/1000000000ULL;
    m->m_speed_ki_coeff_q16=(v>UINT32_MAX)?UINT32_MAX:(uint32_t)v;
    /* D Q4 pada dt=1 ms: raw_gain/1e5 * current_limit * 3200. */
    v=((uint64_t)m->m_kds_q11*lim*32ULL + 500ULL)/1000ULL;
    m->m_speed_kd_coeff_q8=(v>UINT32_MAX)?UINT32_MAX:(uint32_t)v;
}

static void position_pid_recompute_coeff(mcpwm_foc_motor_t *m) {
    if (!m) return;
    /* Count-position process D uses signed mechanical RPM -> electrical deg/s. */
    float v=m->m_conf.p_pid_kd_proc*6.0f*32768.0f*65536.0f;
    if(v<0.0f)v=0.0f;
    m->m_position_kd_proc_coeff_q16=(v>=4294967040.0f)?UINT32_MAX:(uint32_t)(v+0.5f);
    /* Stock VESC SET_POS tracks electrical phase directly. Cache a coefficient
     * for -d(phase)/dt * kd_proc in normalized Q15, preserving the sign of the
     * actual phase delta instead of relying on Hall ERPM sign conventions. */
    v=m->m_conf.p_pid_kd_proc*(float)MCCONF_OUTER_PID_HZ*180.0f*16.0f;
    if(v<0.0f)v=0.0f;
    if(v>65535.0f)v=65535.0f;
    m->m_position_kd_proc_phase_coeff_q4=(uint16_t)(v+0.5f);
    if(m->m_conf.p_pid_gain_dec_angle>0.1f){
        float div=m->m_conf.p_pid_ang_div;
        if(!(div>=0.01f && div<=1000.0f))div=1.0f;
        float md=m->m_conf.p_pid_gain_dec_angle*1000.0f/div;
        if(md<1.0f)md=1.0f;
        m->m_position_gain_dec_mdeg=(md>=4294967040.0f)?UINT32_MAX:(uint32_t)(md+0.5f);
        m->m_position_gain_dec_inv_q31=(uint32_t)(((1ULL<<31)+(m->m_position_gain_dec_mdeg/2u))/m->m_position_gain_dec_mdeg);
    }else{
        m->m_position_gain_dec_mdeg=0u;
        m->m_position_gain_dec_inv_q31=0u;
    }
}


/* Terapkan gain duty PI terhadap tegangan bus aktual.
 * Fungsi ini hanya dipanggil dari jalur konfigurasi/housekeeping, sehingga
 * pembagian integer tidak pernah masuk ke ISR ADC 16 kHz. */
static void duty_pi_apply_vbus(mcpwm_foc_motor_t *m, uint32_t vin_cv) {
    if (!m) return;
    if (vin_cv < 500u) vin_cv = 500u;
    uint64_t v=(uint64_t)m->m_duty_kp_base_q12_x100 + vin_cv/2u;
    v/=vin_cv;
    if(v>UINT32_MAX)v=UINT32_MAX;
    m->m_duty_kp_q12_per_permille=(uint32_t)v;
    v=(uint64_t)m->m_duty_ki_base_q12_x100 + vin_cv/2u;
    v/=vin_cv;
    if(v>UINT32_MAX)v=UINT32_MAX;
    m->m_duty_ki_q12_per_permille=(uint32_t)v;
}

static uint16_t position_gain_scale_q15(const mcpwm_foc_motor_t *m,int32_t error_mdeg){
    if(!m || m->m_position_gain_dec_mdeg==0u)return 32768u;
    uint32_t ae=(uint32_t)(error_mdeg<0?-(int64_t)error_mdeg:error_mdeg);
    if(ae>=m->m_position_gain_dec_mdeg)return 32768u;
    uint32_t s=(uint32_t)(((uint64_t)ae*m->m_position_gain_dec_inv_q31)>>16);
    if(s>32768u)s=32768u;
    return (uint16_t)s;
}

static bool hall_table_runtime_sane(const uint8_t t[8]) {
    uint8_t u[8], sorted[6];
    for (uint8_t i=0u;i<8u;++i) u[i]=(uint8_t)t[i];
    if (u[0]!=255u || u[7]!=255u) return false;
    for (uint8_t h=1u;h<=6u;++h) { if (u[h]>=200u) return false; sorted[h-1u]=u[h]; }
    for (uint8_t i=0u;i<5u;++i) for(uint8_t j=(uint8_t)(i+1u);j<6u;++j)
        if(sorted[j]<sorted[i]){uint8_t x=sorted[i];sorted[i]=sorted[j];sorted[j]=x;}
    for (uint8_t i=0u;i<6u;++i) {
        const uint16_t a=sorted[i], b=(i==5u)?(uint16_t)sorted[0]+200u:sorted[i+1u];
        const uint16_t gap=b-a; if(gap<18u || gap>48u) return false;
    }
    return true;
}

static uint32_t fault_stop_ticks_from_ms(int32_t configured_ms) {
    uint32_t ms=configured_ms>0?(uint32_t)configured_ms:(uint32_t)MCCONF_FAULT_STOP_TIME_MS;
    if(ms<50u)ms=50u;
    uint64_t ticks=((uint64_t)ms*(uint64_t)PWM_FREQ+999u)/1000u;
    if(ticks==0u)ticks=1u;
    if(ticks>UINT32_MAX)ticks=UINT32_MAX;
    return (uint32_t)ticks;
}

static bool current_offset_pair_plausible(int16_t a, int16_t b) {
    const int32_t da=(int32_t)a-MCCONF_CURRENT_OFFSET_CENTER_ADC;
    const int32_t db=(int32_t)b-MCCONF_CURRENT_OFFSET_CENTER_ADC;
    const int32_t pair=(int32_t)a-(int32_t)b;
    return ABS(da)<=MCCONF_CURRENT_OFFSET_MAX_DEVIATION_ADC &&
           ABS(db)<=MCCONF_CURRENT_OFFSET_MAX_DEVIATION_ADC &&
           ABS(pair)<=MCCONF_CURRENT_OFFSET_MAX_PAIR_DELTA_ADC;
}

static bool driven_offset_pair_plausible(int16_t a, int16_t b) {
    /* Powered zero-vector is a different analog operating point from bridge-OFF.
     * Do not require ADC midscale here: on the real LEFT bridge the healthy
     * common-mode is about 3.3k counts. Still fail closed on rail proximity or
     * disagreement between the two independently sampled phase amplifiers. */
    const int32_t lo=MCCONF_DRIVEN_OFFSET_RAIL_MARGIN_ADC;
    const int32_t hi=4095-MCCONF_DRIVEN_OFFSET_RAIL_MARGIN_ADC;
    const int32_t pair=(int32_t)a-(int32_t)b;
    return (int32_t)a>=lo && (int32_t)a<=hi &&
           (int32_t)b>=lo && (int32_t)b<=hi &&
           ABS(pair)<=MCCONF_CURRENT_OFFSET_MAX_PAIR_DELTA_ADC;
}

static uint32_t motor_abs_erpm_for_fault(const mcpwm_foc_motor_t *m, bool second) {
    if(!m)return 0u;
    if(encoder_feedback_selected(m,second) && m->m_encoder_configured){
        int32_t e=m->m_encoder_erpm_q16;
        if(e<0)e=(e==INT32_MIN)?INT32_MAX:-e;
        return (uint32_t)e>>16;
    }
    if(m->m_hall_initialized && m->m_hall_direction!=0 &&
       m->m_hall_period>0u && m->m_hall_period<MCCONF_HALL_TIMEOUT_TICKS &&
       m->m_hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS){
        /* 1 Hall edge = 60 derajat elektrik, sehingga ERPM = 10*Fs/period.
         * Gunakan periode mentah/filtered Hall sebelum m_rpm di-clamp 1000 RPM. */
        return ((uint32_t)PWM_FREQ*10u)/(uint32_t)m->m_hall_period;
    }
    return 0u;
}

static void safety_thresholds_recompute(mcpwm_foc_motor_t *m) {
    if(!m)return;
    float lim=m->m_conf.l_max_erpm;
    if(-m->m_conf.l_min_erpm>lim)lim=-m->m_conf.l_min_erpm;
    if(!(lim>0.0f))lim=MCCONF_L_MAX_ERPM;
    float hard=lim*((float)MCCONF_ABS_OVERSPEED_MARGIN_PERCENT/100.0f);
    if(hard<lim+50.0f)hard=lim+50.0f;
    if(hard>100000.0f)hard=100000.0f;
    m->m_abs_erpm_fault=(uint32_t)(hard+0.5f);
}


static void pll_coeff_recompute(mcpwm_foc_motor_t *m) {
    if(!m)return;
    const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
    float kp=m->m_conf.foc_pll_kp;
    float ki=m->m_conf.foc_pll_ki;
    if(!(kp>=0.0f && kp<=10000.0f))kp=MCCONF_FOC_PLL_KP_DEFAULT;
    if(!(ki>=0.0f && ki<=200000.0f))ki=MCCONF_FOC_PLL_KI_DEFAULT;
    float a=kp*dt*65536.0f;
    float b=ki*dt*dt*65536.0f;
    if(a<0.0f)a=0.0f;
    if(b<0.0f)b=0.0f;
    m->m_pll_kp_dt_q16=(a>=4294967040.0f)?UINT32_MAX:(uint32_t)(a+0.5f);
    m->m_pll_ki_dt2_q16=(b>=4294967040.0f)?UINT32_MAX:(uint32_t)(b+0.5f);
    /* speed_step_q32 = ERPM/60 * dt * 2^32. Clamp memakai hard
     * overspeed threshold agar PLL tidak wind-up melewati envelope hardware. */
    float lim=(float)(m->m_abs_erpm_fault?m->m_abs_erpm_fault:1u)*dt*(4294967296.0f/60.0f);
    if(lim<1.0f)lim=1.0f;
    if(lim>=2147483520.0f)m->m_pll_speed_limit_step_q32=INT32_MAX;
    else m->m_pll_speed_limit_step_q32=(int32_t)(lim+0.5f);
    m->m_pll_valid=0u;
    m->m_pll_phase_acc_q32=(uint32_t)m->m_phase<<16;
    m->m_pll_speed_step_q32=0;
    m->m_pll_erpm_q16=0;
    m->m_pll_mech_rpm_q16=0;
}

static void decoupling_coeff_recompute(mcpwm_foc_motor_t *m) {
    if(!m)return;
    /* Satuan upstream: omega_e [rad/s], I [A], L [H], lambda [Wb].
     * Internal F103 menyimpan current Q4 dan voltage sebagai modulation-count.
     * Semua konversi float dilakukan di sini; control ISR hanya multiply/shift. */
    const float l=m->m_conf.foc_motor_l;
    const float diff=m->m_conf.foc_motor_ld_lq_diff;
    float ld=l-diff*0.5f;
    float lq=l+diff*0.5f;
    if(!(ld>0.0f && ld<=0.1f))ld=0.0f;
    if(!(lq>0.0f && lq<=0.1f))lq=0.0f;
    float flux=m->m_conf.foc_motor_flux_linkage;
    if(!(flux>0.0f && flux<=1.0f))flux=0.0f;
    const float counts_per_v=(float)s_mod_counts_per_volt_q16/65536.0f;
    const float omega_per_erpm=0.1047197551f; /* 2*pi/60 */
    const float q24=16777216.0f;
    float kd=omega_per_erpm*ld*counts_per_v*q24/(float)FOC_CURRENT_Q4_PER_A;
    float kq=omega_per_erpm*lq*counts_per_v*q24/(float)FOC_CURRENT_Q4_PER_A;
    float kf=omega_per_erpm*flux*counts_per_v*q24;
    if(kd<0.0f)kd=0.0f;
    if(kq<0.0f)kq=0.0f;
    if(kf<0.0f)kf=0.0f;
    m->m_dec_ld_coeff_q24=(kd>=2147483520.0f)?INT32_MAX:(int32_t)(kd+0.5f);
    m->m_dec_lq_coeff_q24=(kq>=2147483520.0f)?INT32_MAX:(int32_t)(kq+0.5f);
    m->m_dec_flux_coeff_q24=(kf>=2147483520.0f)?INT32_MAX:(int32_t)(kf+0.5f);
}

static void motor_fault_set(mcpwm_foc_motor_t *m, mc_fault_code code) {
    if(!m)return;
    m->m_fault=code;
    if(code!=FAULT_CODE_NONE && !s_foc_trace_frozen && !s_foc_trace_fault_pending){
        /* Single-writer trace contract: fault setters may run in main or ISR,
         * therefore they only latch a trigger. DMA1_Channel1 owns ring writes,
         * trigger capture and freeze at the next deterministic ISR boundary. */
        s_foc_trace_fault_motor=(m==&m_motor_2)?2u:1u;
        s_foc_trace_fault_code=(uint8_t)code;
        FOC_MEMORY_BARRIER(); s_foc_trace_fault_pending=1u;
    }
    /* Timeout telah dikonversi saat config berubah. Fault path ISR sekarang
     * O(1), tanpa software divide 64-bit pada kondisi yang justru kritis. */
    m->m_fault_recovery_ticks=m->m_fault_stop_ticks?m->m_fault_stop_ticks:1u;
    m->m_fault_safe_ticks=0u;
}


static void driven_offset_finalize_non_isr(void) {
    mcpwm_foc_motor_t *motors[2]={&m_motor_1,&m_motor_2};
    for(uint8_t i=0u;i<2u;++i){
        mcpwm_foc_motor_t *m=motors[i];
        if(!m->m_driven_offset_finalize_pending || !m->m_driven_offset_calibrating ||
           m->m_driven_offset_samples<MCCONF_BRIDGE_SETTLE_SAMPLES)continue;
        const int32_t den=(int32_t)MCCONF_BRIDGE_SETTLE_SAMPLES;
        m->m_driven_offset0=(int16_t)((m->m_driven_offset_sum0+den/2)/den);
        m->m_driven_offset1=(int16_t)((m->m_driven_offset_sum1+den/2)/den);
        m->m_driven_offsetdc=(int16_t)((m->m_driven_offset_sumdc+den/2)/den);
        const bool valid=driven_offset_pair_plausible(m->m_driven_offset0,m->m_driven_offset1);
        m->m_driven_offset_valid=valid?1u:0u;
        m->m_driven_offset_powered_valid=valid?1u:0u;
        m->m_current_offset_valid=valid?1u:0u;
        m->m_driven_offset_calibrating=0u;
        m->m_driven_offset_finalize_pending=0u;
        m->m_bridge_settle_ticks=0u;
        m->m_telem_current_lpf_q16[0]=m->m_telem_current_lpf_q16[1]=m->m_telem_current_lpf_q16[2]=0;
        m->m_id_telem_q4=0; m->m_iq_telem_q4=0; m->m_current_in_telem_counts=0;
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0; m->m_telem_avg_samples=0u;
        if(!valid && m->m_fault==FAULT_CODE_NONE)
            motor_fault_set(m,FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1);
    }
}

static void motor_reset(mcpwm_foc_motor_t *m, bool second) {
    memset(m, 0, sizeof(*m));
    conf_defaults(&m->m_conf, second);
    openloop_phase_coeff_recompute(m,second);
    m->m_state = MC_STATE_OFF;
    m->m_control_mode = CONTROL_MODE_NONE;
    m->m_fault = FAULT_CODE_NONE;
    m->m_sample_zero_window_counts=pwm_res/2u;
    m->m_sample_window_min_counts=0u;
    m->m_sample_guard_counts=FOC_CURRENT_SAMPLE_GUARD_COUNTS;
    m->m_sample_adc_phase_counts=pwm_res;
    m->m_sample_window_valid=1u;
    m->m_sample_sector=1u;
    m->m_kpq_q11=MCCONF_FOC_CURRENT_KP_Q11; m->m_kiq_q16=MCCONF_FOC_CURRENT_KI_Q16;
    m->m_kpd_q11=MCCONF_FOC_ID_KP_Q11; m->m_kid_q16=MCCONF_FOC_ID_KI_Q16;
    m->m_kps_q11=MCCONF_SPEED_KP_Q11; m->m_kis_q16=MCCONF_SPEED_KI_Q16; m->m_kds_q11=MCCONF_SPEED_KD_Q11;
    m->m_kpp_q11=MCCONF_POSITION_KP_Q11; m->m_kip_q16=MCCONF_POSITION_KI_Q16; m->m_kdp_q11=MCCONF_POSITION_KD_Q11;
    m->m_position_kd_filter_q16=MCCONF_POSITION_KD_FILTER_Q16;
    /* VESC position D accumulators start from zero elapsed time. The first
     * 1-kHz PID tick adds its real dt once; seeding with one tick would make
     * the first derivative interval 2 ms and halve D authority. */
    m->m_position_dt_ticks=0u;
    m->m_position_min_counts=INT32_MIN; m->m_position_max_counts=INT32_MAX;
    m->m_steering_span_counts=0; m->m_steering_calibrated=0u; m->m_steering_homed=0u;
    m->m_current_limit_q4=(int16_t)(MCCONF_L_CURRENT_MAX*MCCONF_L_CURRENT_MAX_SCALE*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_current_limit_neg_q4=(int16_t)(-MCCONF_L_CURRENT_MIN*MCCONF_L_CURRENT_MIN_SCALE*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_battery_cut_start_adc=(uint16_t)(MCCONF_L_BATTERY_CUT_START*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_battery_cut_end_adc=(uint16_t)(MCCONF_L_BATTERY_CUT_END*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_battery_regen_cut_start_adc=(uint16_t)(MCCONF_L_BATTERY_REGEN_CUT_START*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_battery_regen_cut_end_adc=(uint16_t)(MCCONF_L_BATTERY_REGEN_CUT_END*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_vin_min_adc=(uint16_t)(MCCONF_L_MIN_VIN*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_vin_max_adc=(uint16_t)(MCCONF_L_MAX_VIN*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
    m->m_watt_max_x10=(uint32_t)(MCCONF_L_WATT_MAX*10.0f+0.5f);
    m->m_watt_regen_x10=(uint32_t)(-MCCONF_L_WATT_MIN*10.0f+0.5f);
    m->m_fault_stop_ticks=fault_stop_ticks_from_ms(m->m_conf.m_fault_stop_time_ms);
    safety_thresholds_recompute(m);
    pll_coeff_recompute(m);
    decoupling_coeff_recompute(m);
    m->m_temp_fet_start_x10=(int16_t)(MCCONF_L_TEMP_FET_START*10.0f+0.5f);
    m->m_temp_fet_end_x10=(int16_t)(MCCONF_L_TEMP_FET_END*10.0f+0.5f);
    m->m_temp_fet_accel_start_x10=(int16_t)((MCCONF_L_TEMP_FET_START+MCCONF_L_TEMP_ACCEL_DEC*(25.0f-MCCONF_L_TEMP_FET_START))*10.0f+0.5f);
    m->m_temp_fet_accel_end_x10=(int16_t)((MCCONF_L_TEMP_FET_END+MCCONF_L_TEMP_ACCEL_DEC*(25.0f-MCCONF_L_TEMP_FET_END))*10.0f+0.5f);
    m->m_erpm_pos_end=(int32_t)MCCONF_L_MAX_ERPM;
    m->m_erpm_neg_end=(int32_t)MCCONF_L_MIN_ERPM;
    m->m_erpm_pos_start=(int32_t)(MCCONF_L_MAX_ERPM*MCCONF_L_ERPM_START);
    m->m_erpm_neg_start=(int32_t)(MCCONF_L_MIN_ERPM*MCCONF_L_ERPM_START);
    m->m_input_current_max_q4=(int16_t)(MCCONF_L_IN_CURRENT_MAX*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_input_current_regen_q4=(int16_t)(-MCCONF_L_IN_CURRENT_MIN*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_watt_current_max_q4=m->m_input_current_max_q4;
    m->m_watt_current_regen_q4=m->m_input_current_regen_q4;
    m->m_in_current_map_start_q15=(uint16_t)(MCCONF_L_IN_CURRENT_MAP_START*32768.0f+0.5f);
    m->m_in_current_map_filter_q16=(uint16_t)(MCCONF_L_IN_CURRENT_MAP_FILTER*65535.0f+0.5f);
    current_pid_recompute_coeff(m);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    hall_interp_recompute(m);
    m->m_abs_current_limit_counts=(int16_t)(MCCONF_L_ABS_CURRENT_MAX*(float)A2BIT_CONV+0.5f);
    m->m_duty_limit_permille=(int16_t)(MCCONF_L_MAX_DUTY*1000.0f+0.5f);
    m->m_voltage_limit_counts=(int16_t)CLAMP(((int32_t)m->m_duty_limit_permille*(int32_t)MCCONF_FOC_DUTY_VOLTAGE_MAX)/1000,1,MCCONF_FOC_DUTY_VOLTAGE_MAX);
    m->m_duty_start_permille=(int16_t)(MCCONF_L_MAX_DUTY*MCCONF_L_DUTY_START*1000.0f+0.5f);
    m->m_cc_min_current_q4=(int16_t)(MCCONF_CC_MIN_CURRENT*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_duty_end_current_q4=(int16_t)(MCCONF_CC_MIN_CURRENT*5.0f*FOC_CURRENT_Q4_PER_A+0.5f);
    m->m_speed_kd_filter_q16=(uint16_t)(MCCONF_SPEED_KD_FILTER_DEFAULT*65535.0f+0.5f);
    m->m_telem_current_filter_q16=(uint16_t)(MCCONF_FOC_TELEMETRY_FILTER_DEFAULT*65535.0f+0.5f);
    {
        const uint16_t pp = motor_pole_pairs(second);
        if (MCCONF_SPEED_RAMP_ERPMS_S == 0u) {
            m->m_speed_ramp_rpm_s = 0u;
        } else {
            m->m_speed_ramp_rpm_s = (uint16_t)(MCCONF_SPEED_RAMP_ERPMS_S / pp);
            if (m->m_speed_ramp_rpm_s == 0u) m->m_speed_ramp_rpm_s = 1u;
        }
        m->m_speed_release_erpm_q16 = (uint32_t)MCCONF_SPEED_RELEASE_ERPM << 16;
    }
    {
        const float base=13421772.8f;
        const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
        m->m_duty_kp_base_q12_x100=(uint32_t)(MCCONF_FOC_DUTY_DOWNRAMP_KP*base+0.5f);
        m->m_duty_ki_base_q12_x100=(uint32_t)(MCCONF_FOC_DUTY_DOWNRAMP_KI*dt*base+0.5f);
        const uint32_t vin_cv=((uint32_t)(batVoltage>0?batVoltage:1)*(uint32_t)BAT_CALIB_REAL_VOLTAGE)/(uint32_t)BAT_CALIB_ADC;
        duty_pi_apply_vbus(m,vin_cv?vin_cv:1u);
    }
    m->m_hall_pos_prev = 0;
    m->m_hall_reject_counted_state = 0xffu;
    m->m_hall_period = MCCONF_HALL_TIMEOUT_TICKS;
    for (int i=0;i<4;i++) m->m_hall_period_hist[i] = MCCONF_HALL_TIMEOUT_TICKS;
    m->m_phase_openloop = 0u;
    m->m_openloop_phase_acc_q32 = 0u;
    position_ang_div_recompute(m);
    /* VESC tacho bin untuk electrical phase 0 adalah step 3 pada peta
     * [-180,+180). Memulai dari 3 mencegah lonjakan tachometer +3 saat boot. */
    m->m_tacho_step_last = 3u;
}

void mcpwm_foc_init(void) {
    motor_reset(&m_motor_1, false);
    motor_reset(&m_motor_2, true);
    offsetcount = 0;
    offsetrlA = offsetrlB = offsetrrB = offsetrrC = offsetdcl = offsetdcr = 2000;
    /* Re-init harus mengembalikan state LPF Vbus seperti cold boot. Tanpa ini,
     * housekeeping dapat mewarisi tegangan filter sesi sebelumnya dan membuat
     * fault palsu walau motor/fault state sudah di-reset. */
    batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
    batVoltageFixdt = q16_from_i32_sat((int32_t)batVoltage);
    m_motor_1.m_driven_offset0=offsetrlA; m_motor_1.m_driven_offset1=offsetrlB; m_motor_1.m_driven_offsetdc=offsetdcl;
    m_motor_2.m_driven_offset0=offsetrrB; m_motor_2.m_driven_offset1=offsetrrC; m_motor_2.m_driven_offsetdc=offsetdcr;
    m_motor_1.m_off_offset0=offsetrlA; m_motor_1.m_off_offset1=offsetrlB; m_motor_1.m_off_offsetdc=offsetdcl;
    m_motor_2.m_off_offset0=offsetrrB; m_motor_2.m_off_offset1=offsetrrC; m_motor_2.m_off_offsetdc=offsetdcr;
    foc_isr_cycles = 0u;
    mcpwm_foc_reset_isr_profile();
    s_overrun = 0;
    s_voltage_scale_bat_adc = INT16_MIN;
    s_vesc_owned[0]=s_vesc_owned[1]=0u;
    s_vesc_timeout_braking[0]=s_vesc_timeout_braking[1]=0u;
    s_vesc_timeout_expired[0]=s_vesc_timeout_expired[1]=0u;
    s_vesc_timeout_ticks[0]=s_vesc_timeout_ticks[1]=0u;
    s_vesc_timeout_ms[0]=s_vesc_timeout_ms[1]=VESC_RUNTIME_TIMEOUT_DEFAULT_MS;
    s_vesc_timeout_brake_q4[0]=s_vesc_timeout_brake_q4[1]=0;
    s_energy_last_ms = 0u;
    s_foc_control_div = 0u;
    s_housekeeping_last_ms = 0u;
    s_outer_tick_remainder = 0u;
    s_outer_pid_last_ms = 0u;
    s_outer_pid_last_cycle = 0u;
    outer_control_max_cycles = 0u;
    outer_control_miss_count = 0u;
    outer_control_jitter_max_cycles = 0u;
    outer_control_period_min_cycles = UINT32_MAX;
    outer_control_period_max_cycles = 0u;
    s_left_abi_tacho_pos_last = 0;
    s_left_abi_tacho_remainder = 0;
    s_left_abi_tacho_tracking = 0u;
}

mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool second) { return second ? &m_motor_2 : &m_motor_1; }
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool second) { return second ? &m_motor_2 : &m_motor_1; }

static int16_t amp_to_q4(const mcpwm_foc_motor_t *m, float current);

void mcpwm_foc_set_configuration(const mc_configuration *conf, bool second) {
    if (!conf) return;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    /* Fail-safe shadow-publication gate. Main cannot be pre-empted by itself, but
     * the 16-kHz ADC ISR can interrupt the many cache calculations below. Mark
     * the endpoint unavailable and disable MOE before m_conf/caches change; the
     * ISR takes an explicit config-update fast exit until every derived value is
     * coherent again. This avoids a long global IRQ-off section. */
    m->m_config_update_active=1u;
    FOC_MEMORY_BARRIER();
    if(second) RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    else LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    mc_configuration next = *conf;
    f103_mcconf_canonicalize_unsupported(&next);
    /* VESC stores the number of motor poles (not pole-pairs). FOC speed math
     * uses this value at runtime, so changing Motor Poles in VESC Tool really
     * changes ERPM <-> mechanical RPM conversion without recompiling. */
    if (next.si_motor_poles < 2u || (next.si_motor_poles & 1u)) {
        next.si_motor_poles = m->m_conf.si_motor_poles;
        if (next.si_motor_poles < 2u || (next.si_motor_poles & 1u))
            next.si_motor_poles = (uint8_t)default_motor_poles(second);
    }
    if (!(next.si_gear_ratio >= 0.01f && next.si_gear_ratio <= 1000.0f)) next.si_gear_ratio = 1.0f;
    if (!(next.l_max_duty > 0.0f) || next.l_max_duty > MCCONF_L_MAX_DUTY) next.l_max_duty=MCCONF_L_MAX_DUTY;
    if (!(next.l_erpm_start >= 0.0f && next.l_erpm_start <= 1.0f)) next.l_erpm_start=MCCONF_L_ERPM_START;
    if (!(next.l_duty_start >= 0.0f && next.l_duty_start <= 1.0f)) next.l_duty_start=MCCONF_L_DUTY_START;
    if (!(next.l_temp_accel_dec >= 0.0f && next.l_temp_accel_dec <= 1.0f)) next.l_temp_accel_dec=MCCONF_L_TEMP_ACCEL_DEC;
    if (!(next.l_in_current_map_start >= 0.0f && next.l_in_current_map_start <= 1.0f)) next.l_in_current_map_start=MCCONF_L_IN_CURRENT_MAP_START;
    if (!(next.l_in_current_map_filter >= 0.00001f && next.l_in_current_map_filter <= 1.0f)) next.l_in_current_map_filter=MCCONF_L_IN_CURRENT_MAP_FILTER;
    if (!(next.s_pid_kd_filter >= 0.0f && next.s_pid_kd_filter <= 1.0f)) next.s_pid_kd_filter=MCCONF_SPEED_KD_FILTER_DEFAULT;
    if (!(next.l_current_max_scale >= 0.0f && next.l_current_max_scale <= 1.0f)) next.l_current_max_scale=MCCONF_L_CURRENT_MAX_SCALE;
    if (!(next.l_current_min_scale >= 0.0f && next.l_current_min_scale <= 1.0f)) next.l_current_min_scale=MCCONF_L_CURRENT_MIN_SCALE;
    if (!(next.l_battery_cut_start > next.l_battery_cut_end && next.l_battery_cut_end >= 0.0f)) {
        next.l_battery_cut_start=MCCONF_L_BATTERY_CUT_START;
        next.l_battery_cut_end=MCCONF_L_BATTERY_CUT_END;
    }
    if (!(next.l_min_vin >= 5.0f && next.l_max_vin > next.l_min_vin && next.l_max_vin <= 80.0f)) {
        next.l_min_vin=MCCONF_L_MIN_VIN; next.l_max_vin=MCCONF_L_MAX_VIN;
    }
    if (!(next.l_battery_regen_cut_end > next.l_battery_regen_cut_start &&
          next.l_battery_regen_cut_start >= 0.0f && next.l_battery_regen_cut_end <= next.l_max_vin)) {
        next.l_battery_regen_cut_start=MCCONF_L_BATTERY_REGEN_CUT_START;
        next.l_battery_regen_cut_end=MCCONF_L_BATTERY_REGEN_CUT_END;
    }
    if (!(next.l_watt_max > 0.0f && next.l_watt_max <= 200000000.0f)) next.l_watt_max=MCCONF_L_WATT_MAX;
    if (!(next.l_watt_min < 0.0f && next.l_watt_min >= -200000000.0f)) next.l_watt_min=MCCONF_L_WATT_MIN;
    if (!(next.l_temp_fet_end > next.l_temp_fet_start && next.l_temp_fet_start >= -40.0f && next.l_temp_fet_end <= 180.0f)) {
        next.l_temp_fet_start=MCCONF_L_TEMP_FET_START; next.l_temp_fet_end=MCCONF_L_TEMP_FET_END;
    }
    /* Hardware tidak mempunyai sensor temperatur motor eksternal. Jangan
     * mengarang temperatur motor; tandai port temperatur motor sebagai disabled. */
    next.m_motor_temp_sens_type=TEMP_SENSOR_DISABLED;
    if(!(next.foc_pll_kp>=0.0f && next.foc_pll_kp<=10000.0f))next.foc_pll_kp=MCCONF_FOC_PLL_KP_DEFAULT;
    if(!(next.foc_pll_ki>=0.0f && next.foc_pll_ki<=200000.0f))next.foc_pll_ki=MCCONF_FOC_PLL_KI_DEFAULT;
    /* F103 menyediakan source PLL dan FAST. FASTER belum mempunyai estimator
     * terpisah, sehingga canonicalize ke FAST agar readback tidak berbohong. */
    if(next.s_pid_speed_source!=S_PID_SPEED_SRC_PLL && next.s_pid_speed_source!=S_PID_SPEED_SRC_FAST)
        next.s_pid_speed_source=S_PID_SPEED_SRC_FAST;
    if((uint8_t)next.foc_cc_decoupling>(uint8_t)FOC_CC_DECOUPLING_CROSS_BEMF)
        next.foc_cc_decoupling=FOC_CC_DECOUPLING_DISABLED;
    {
        const bool l_ok=isfinite(next.foc_motor_l)&&next.foc_motor_l>0.000001f&&next.foc_motor_l<=0.1f;
        const bool flux_ok=isfinite(next.foc_motor_flux_linkage)&&next.foc_motor_flux_linkage>0.000001f&&next.foc_motor_flux_linkage<=1.0f;
        if((next.foc_cc_decoupling==FOC_CC_DECOUPLING_CROSS && !l_ok) ||
           (next.foc_cc_decoupling==FOC_CC_DECOUPLING_BEMF && !flux_ok) ||
           (next.foc_cc_decoupling==FOC_CC_DECOUPLING_CROSS_BEMF && (!l_ok || !flux_ok)))
            next.foc_cc_decoupling=FOC_CC_DECOUPLING_DISABLED;
    }
    /* Physical sensor contract for this board is deliberately narrower than a
     * generic VESC: LEFT is either ABI Encoder or Hall, RIGHT is Hall-only.
     * R/L/Flux commissioning is sensor-independent because OPENLOOP overrides
     * electrical phase; it must not require or expose a persistent Sensorless
     * feedback mode on the steering endpoint. */
    if (second) {
        next.sensor_mode=SENSOR_MODE_SENSORED;
        next.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
        next.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
    } else if (next.m_sensor_port_mode==SENSOR_PORT_MODE_ABI) {
        next.sensor_mode=SENSOR_MODE_SENSORED;
        if (next.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER &&
            next.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER_AB)
            next.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
    } else {
        next.sensor_mode=SENSOR_MODE_SENSORED;
        next.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
        next.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
    }
    if (next.m_encoder_counts<4 || next.m_encoder_counts>65536)
        next.m_encoder_counts=(int32_t)MCCONF_ENCODER_COUNTS_DEFAULT;
    if (!(next.foc_encoder_ratio>=0.01f && next.foc_encoder_ratio<=MCCONF_ENCODER_RATIO_MAX))
        next.foc_encoder_ratio=(float)(next.si_motor_poles/2u);
    next.foc_encoder_offset=encoder_norm_deg(next.foc_encoder_offset);
    if (!(next.l_min_erpm < 0.0f)) next.l_min_erpm=MCCONF_L_MIN_ERPM;
    if (next.l_min_erpm < MCCONF_L_MIN_ERPM) next.l_min_erpm=MCCONF_L_MIN_ERPM;
    if (!(next.l_max_erpm > 0.0f)) next.l_max_erpm=MCCONF_L_MAX_ERPM;
    if (next.l_max_erpm > MCCONF_L_MAX_ERPM) next.l_max_erpm=MCCONF_L_MAX_ERPM;
    if (!(next.si_wheel_diameter > 0.001f && next.si_wheel_diameter < 5.0f)) next.si_wheel_diameter=MCCONF_SI_WHEEL_DIAMETER;
    if ((uint8_t)next.si_battery_type > (uint8_t)BATTERY_TYPE_LEAD_ACID) next.si_battery_type=BATTERY_TYPE_LIION_3_0__4_2;
    if (next.si_battery_cells < 1 || next.si_battery_cells > 32) next.si_battery_cells=BAT_CELLS;
    if (!(next.si_battery_ah >= 0.0f && next.si_battery_ah <= 655.35f)) next.si_battery_ah=0.0f;
    if (!(next.l_in_current_max >= 0.1f) || next.l_in_current_max > (float)I_DC_MAX) next.l_in_current_max=MCCONF_L_IN_CURRENT_MAX;
    if (!(next.l_in_current_min <= -0.1f) || next.l_in_current_min < -(float)I_DC_MAX) next.l_in_current_min=MCCONF_L_IN_CURRENT_MIN;
    if (!(next.m_duty_ramp_step >= 0.0001f && next.m_duty_ramp_step <= 0.20f)) next.m_duty_ramp_step=MCCONF_DUTY_RAMP_STEP_DEFAULT;
    if (!(next.cc_min_current >= 0.001f && next.cc_min_current <= 1.0f)) next.cc_min_current=MCCONF_CC_MIN_CURRENT;
    /* Absolute phase-current fault must cover both motoring and regenerative
     * current ranges. Checking only l_current_max lets a large negative
     * l_current_min exceed the ABS threshold during braking. Keep the VESC Tool
     * field authoritative when valid, bounded by the board hard ceiling. */
    {
        float commanded_abs = next.l_current_max;
        if (-next.l_current_min > commanded_abs) commanded_abs = -next.l_current_min;
        if (!(next.l_abs_current_max >= commanded_abs) ||
            next.l_abs_current_max > MCCONF_L_ABS_CURRENT_MAX) {
            next.l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
        }
    }
    if (!(next.foc_duty_dowmramp_kp > 0.0f)) next.foc_duty_dowmramp_kp=MCCONF_FOC_DUTY_DOWNRAMP_KP;
    if (!(next.foc_duty_dowmramp_ki > 0.0f)) next.foc_duty_dowmramp_ki=MCCONF_FOC_DUTY_DOWNRAMP_KI;
    /* Upstream uses foc_current_filter_const for less time-critical filtered
     * currents. Keep the current-loop feedback filter independent and make this
     * standard MC-config field control monitoring smoothness only. */
    if (!(next.foc_current_filter_const >= 0.001f && next.foc_current_filter_const <= 1.0f))
        next.foc_current_filter_const=MCCONF_FOC_TELEMETRY_FILTER_DEFAULT;
    if (!(next.foc_hall_interp_erpm >= 0.0f && next.foc_hall_interp_erpm <= MCCONF_L_MAX_ERPM))
        next.foc_hall_interp_erpm=(float)MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT;
    if(next.m_hall_extra_samples < 0 || next.m_hall_extra_samples > 20)
        next.m_hall_extra_samples=(int)MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT;
    if (!(next.p_pid_kd_filter >= 0.0f && next.p_pid_kd_filter <= 1.0f))
        next.p_pid_kd_filter=(float)MCCONF_POSITION_KD_FILTER_Q16/65536.0f;
    if (!(next.p_pid_ang_div>=0.01f && next.p_pid_ang_div<=1000.0f))next.p_pid_ang_div=1.0f;
    if (!(next.p_pid_kd_proc>=0.0f && next.p_pid_kd_proc<=10.0f))next.p_pid_kd_proc=0.00035f;
    if (!(next.p_pid_gain_dec_angle>=0.0f && next.p_pid_gain_dec_angle<=3276.7f))next.p_pid_gain_dec_angle=0.0f;
    if (!(next.p_pid_offset>-100000.0f && next.p_pid_offset<100000.0f))next.p_pid_offset=0.0f;
    const bool poles_changed = m->m_conf.si_motor_poles != next.si_motor_poles;
    const bool encoder_reinit = (!second) &&
        (m->m_conf.m_sensor_port_mode != next.m_sensor_port_mode ||
         m->m_conf.m_encoder_counts != next.m_encoder_counts);
    const bool feedback_mode_changed = m->m_conf.foc_sensor_mode != next.foc_sensor_mode;
    const bool encoder_cal_changed = (!second) &&
        (m->m_conf.foc_encoder_inverted != next.foc_encoder_inverted ||
         m->m_conf.foc_encoder_offset != next.foc_encoder_offset ||
         m->m_conf.foc_encoder_ratio != next.foc_encoder_ratio);
    /* Never let a malformed VESC Tool/EEPROM Hall table become the live FOC
     * angle source. Preserve the last known-good table while still accepting
     * the other configuration fields. */
    if (!hall_table_runtime_sane(next.foc_hall_table)) {
        for (uint8_t h=0u;h<8u;++h) next.foc_hall_table[h]=m->m_conf.foc_hall_table[h];
    }
    const bool encoder_requires_resync = (!second) &&
        (encoder_reinit || feedback_mode_changed || encoder_cal_changed);
    /* Publish konfigurasi encoder secara fail-safe terhadap ISR 16 kHz. Clear
     * sync dan release bridge SEBELUM m_conf baru terlihat; kalau urutannya
     * dibalik, satu frame Park/SVPWM dapat memakai offset/ratio baru dengan
     * status sync lama dan menghasilkan lonjakan electrical phase. */
    if (poles_changed || encoder_requires_resync) mcpwm_foc_release_motor(second);
    if (encoder_requires_resync) m->m_encoder_synced=0u;
    m->m_conf = next;
    m->m_fault_stop_ticks=fault_stop_ticks_from_ms(next.m_fault_stop_time_ms);
    safety_thresholds_recompute(m);
    openloop_phase_coeff_recompute(m,second);
    pll_coeff_recompute(m);
    decoupling_coeff_recompute(m);
    hall_interp_recompute(m);
    position_ang_div_recompute(m);
    encoder_runtime_configure(m,second,encoder_reinit);
    if (poles_changed) {
        /* A live pole-count change would instantly rescale the speed loop.
         * Release first, matching upstream's stop-on-structural-config-change policy. */
        mcpwm_foc_release_motor(second);
        m->m_speed_set_rpm = 0;
        m->m_speed_target_rpm = 0;
        m->m_speed_target_rpm_q16 = 0;
        m->m_speed_set_ramp_q16 = 0;
    }

    /* VESC configuration uses electrical units. Convert once outside the ISR
     * and keep the actual speed-loop ramp integer/fixed-point. */
    const float pp = (float)motor_pole_pairs(second);
    float ramp_mech = next.s_pid_ramp_erpms_s / pp;
    if (next.s_pid_ramp_erpms_s <= 0.0f) {
        /* Upstream VESC: zero ramp means direct setpoint, not 1 RPM/s. */
        m->m_speed_ramp_rpm_s = 0u;
    } else {
        if (ramp_mech < 1.0f) ramp_mech = 1.0f;
        if (ramp_mech > 5000.0f) ramp_mech = 5000.0f;
        m->m_speed_ramp_rpm_s = (uint16_t)(ramp_mech + 0.5f);
    }

    /* Keep VESC s_pid_min_erpm in its native electrical domain. Converting
     * through integer mechanical RPM shifts thresholds whenever pole-pairs do
     * not divide the ERPM value exactly (75 ERPM @4pp used to become 72). */
    float release_erpm = next.s_pid_min_erpm;
    if (release_erpm < 0.0f) release_erpm = 0.0f;
    if (release_erpm > 65535.0f) release_erpm = 65535.0f;
    m->m_speed_release_erpm_q16=(uint32_t)(release_erpm*65536.0f+0.5f);
    int32_t kpc=(int32_t)(next.foc_current_kp*1536.0f+0.5f);
    int32_t kic=(int32_t)(next.foc_current_ki*4.608f+0.5f);
    kpc=CLAMP(kpc,0,65535); kic=CLAMP(kic,0,65535);
    m->m_kpq_q11=m->m_kpd_q11=(uint16_t)kpc;
    m->m_kiq_q16=m->m_kid_q16=(uint16_t)kic;
    m->m_kps_q11=(uint16_t)CLAMP((int32_t)(next.s_pid_kp*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kis_q16=(uint16_t)CLAMP((int32_t)(next.s_pid_ki*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kds_q11=(uint16_t)CLAMP((int32_t)(next.s_pid_kd*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kpp_q11=(uint16_t)CLAMP((int32_t)(next.p_pid_kp*1000.0f+0.5f),0,65535);
    m->m_kip_q16=(uint16_t)CLAMP((int32_t)(next.p_pid_ki*1000.0f+0.5f),0,65535);
    m->m_kdp_q11=(uint16_t)CLAMP((int32_t)(next.p_pid_kd*1000.0f+0.5f),0,65535);
    m->m_position_kd_filter_q16=(uint16_t)CLAMP((int32_t)(next.p_pid_kd_filter*65535.0f+0.5f),0,65535);
    m->m_current_limit_q4=(int16_t)CLAMP((int32_t)(next.l_current_max*next.l_current_max_scale*FOC_CURRENT_Q4_PER_A+0.5f),1,MCCONF_MOTOR_CURRENT_MAX_Q4);
    m->m_current_limit_neg_q4=(int16_t)CLAMP((int32_t)(-next.l_current_min*next.l_current_min_scale*FOC_CURRENT_Q4_PER_A+0.5f),1,MCCONF_MOTOR_CURRENT_MAX_Q4);
    m->m_battery_cut_start_adc=(uint16_t)CLAMP((int32_t)(next.l_battery_cut_start*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),1,4095);
    m->m_battery_cut_end_adc=(uint16_t)CLAMP((int32_t)(next.l_battery_cut_end*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),0,4094);
    m->m_battery_regen_cut_start_adc=(uint16_t)CLAMP((int32_t)(next.l_battery_regen_cut_start*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),0,4094);
    m->m_battery_regen_cut_end_adc=(uint16_t)CLAMP((int32_t)(next.l_battery_regen_cut_end*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),1,4095);
    m->m_vin_min_adc=(uint16_t)CLAMP((int32_t)(next.l_min_vin*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),1,4094);
    m->m_vin_max_adc=(uint16_t)CLAMP((int32_t)(next.l_max_vin*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f),2,4095);
    /* l_watt_* sudah divalidasi <= +/-200 MW, sehingga x10 tepat berada
     * dalam int32. Hindari cast float->int64 yang menarik helper soft-double. */
    m->m_watt_max_x10=(uint32_t)CLAMP((int32_t)(next.l_watt_max*10.0f+0.5f),1,2000000000);
    m->m_watt_regen_x10=(uint32_t)CLAMP((int32_t)(-next.l_watt_min*10.0f+0.5f),1,2000000000);
    m->m_temp_fet_start_x10=(int16_t)CLAMP((int32_t)(next.l_temp_fet_start*10.0f+0.5f),-400,1800);
    m->m_temp_fet_end_x10=(int16_t)CLAMP((int32_t)(next.l_temp_fet_end*10.0f+0.5f),-399,1800);
    {
        const float tas=next.l_temp_fet_start+next.l_temp_accel_dec*(25.0f-next.l_temp_fet_start);
        const float tae=next.l_temp_fet_end+next.l_temp_accel_dec*(25.0f-next.l_temp_fet_end);
        m->m_temp_fet_accel_start_x10=(int16_t)CLAMP((int32_t)(tas*10.0f+0.5f),-400,1800);
        m->m_temp_fet_accel_end_x10=(int16_t)CLAMP((int32_t)(tae*10.0f+0.5f),-399,1800);
    }
    m->m_erpm_pos_end=(int32_t)(next.l_max_erpm+0.5f);
    m->m_erpm_neg_end=(int32_t)(next.l_min_erpm-0.5f);
    m->m_erpm_pos_start=(int32_t)(next.l_max_erpm*next.l_erpm_start+0.5f);
    m->m_erpm_neg_start=(int32_t)(next.l_min_erpm*next.l_erpm_start-0.5f);
    m->m_wrong_voltage_integrator=0u;
    m->m_input_current_max_q4=(int16_t)CLAMP((int32_t)(next.l_in_current_max*FOC_CURRENT_Q4_PER_A+0.5f),1,I_DC_MAX*FOC_CURRENT_Q4_PER_A);
    m->m_input_current_regen_q4=(int16_t)CLAMP((int32_t)(-next.l_in_current_min*FOC_CURRENT_Q4_PER_A+0.5f),1,I_DC_MAX*FOC_CURRENT_Q4_PER_A);
    {
        const uint32_t cv=((uint32_t)(batVoltage>0?batVoltage:1)*(uint32_t)BAT_CALIB_REAL_VOLTAGE)/(uint32_t)BAT_CALIB_ADC;
        watt_current_limits_refresh(m,cv?cv:1u);
    }
    m->m_in_current_map_start_q15=(uint16_t)CLAMP((int32_t)(next.l_in_current_map_start*32768.0f+0.5f),0,32768);
    m->m_in_current_map_filter_q16=(uint16_t)CLAMP((int32_t)(next.l_in_current_map_filter*65535.0f+0.5f),1,65535);
    m->m_in_current_map_lpf_q20=0;
    {
        int32_t a=(int32_t)(next.foc_current_filter_const*65535.0f+0.5f);
        m->m_telem_current_filter_q16=(uint16_t)CLAMP(a,1,65535);
    }
    current_pid_recompute_coeff(m);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    m->m_abs_current_limit_counts=(int16_t)CLAMP((int32_t)(next.l_abs_current_max*(float)A2BIT_CONV+0.5f),1,32767);
    m->m_duty_limit_permille=(int16_t)CLAMP((int32_t)(next.l_max_duty*1000.0f+0.5f),1,1000);
    m->m_voltage_limit_counts=(int16_t)CLAMP(((int32_t)m->m_duty_limit_permille*(int32_t)MCCONF_FOC_DUTY_VOLTAGE_MAX)/1000,1,MCCONF_FOC_DUTY_VOLTAGE_MAX);
    m->m_duty_start_permille=(int16_t)CLAMP((int32_t)(next.l_max_duty*next.l_duty_start*1000.0f+0.5f),0,m->m_duty_limit_permille);
    m->m_cc_min_current_q4=(int16_t)CLAMP((int32_t)(next.cc_min_current*FOC_CURRENT_Q4_PER_A+0.5f),1,MCCONF_MOTOR_CURRENT_MAX_Q4);
    m->m_duty_end_current_q4=(int16_t)CLAMP((int32_t)(next.cc_min_current*5.0f*FOC_CURRENT_Q4_PER_A+0.5f),1,MCCONF_MOTOR_CURRENT_MAX_Q4);
    m->m_speed_kd_filter_q16=(uint16_t)CLAMP((int32_t)(next.s_pid_kd_filter*65535.0f+0.5f),0,65535);
    {
        const float base=13421772.8f; /* (32768*4096/1000)*100 */
        const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
        m->m_duty_kp_base_q12_x100=(uint32_t)(next.foc_duty_dowmramp_kp*base+0.5f);
        m->m_duty_ki_base_q12_x100=(uint32_t)(next.foc_duty_dowmramp_ki*dt*base+0.5f);
        const uint32_t vin_cv=((uint32_t)(batVoltage>0?batVoltage:1)*(uint32_t)BAT_CALIB_REAL_VOLTAGE)/(uint32_t)BAT_CALIB_ADC;
        duty_pi_apply_vbus(m,vin_cv?vin_cv:1u);
    }
    /* All configuration-derived caches are now coherent. The next ADC frame may
     * re-arm the bridge only after recomputing fresh CCRs through the normal gate. */
    FOC_MEMORY_BARRIER();
    m->m_config_update_active=0u;
}
void mcpwm_foc_sync_tuning_to_conf(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    current_pid_recompute_coeff(m);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
    m->m_conf.foc_current_kp=(float)m->m_kpq_q11/1536.0f;
    m->m_conf.foc_current_ki=(float)m->m_kiq_q16/4.608f;
    m->m_conf.s_pid_kp=(float)m->m_kps_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.s_pid_ki=(float)m->m_kis_q16/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.s_pid_kd=(float)m->m_kds_q11/(float)MCCONF_SPEED_GAIN_SCALE;
    m->m_conf.p_pid_kp=(float)m->m_kpp_q11/1000.0f; m->m_conf.p_pid_ki=(float)m->m_kip_q16/1000.0f; m->m_conf.p_pid_kd=(float)m->m_kdp_q11/1000.0f;
}

void mcpwm_foc_apply_tuning_from_conf(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    int32_t kpc=(int32_t)(m->m_conf.foc_current_kp*1536.0f+0.5f);
    int32_t kic=(int32_t)(m->m_conf.foc_current_ki*4.608f+0.5f);
    kpc=CLAMP(kpc,0,65535); kic=CLAMP(kic,0,65535);
    m->m_kpq_q11=m->m_kpd_q11=(uint16_t)kpc;
    m->m_kiq_q16=m->m_kid_q16=(uint16_t)kic;
    m->m_kps_q11=(uint16_t)CLAMP((int32_t)(m->m_conf.s_pid_kp*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kis_q16=(uint16_t)CLAMP((int32_t)(m->m_conf.s_pid_ki*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kds_q11=(uint16_t)CLAMP((int32_t)(m->m_conf.s_pid_kd*(float)MCCONF_SPEED_GAIN_SCALE+0.5f),0,65535);
    m->m_kpp_q11=(uint16_t)CLAMP((int32_t)(m->m_conf.p_pid_kp*1000.0f+0.5f),0,65535);
    m->m_kip_q16=(uint16_t)CLAMP((int32_t)(m->m_conf.p_pid_ki*1000.0f+0.5f),0,65535);
    m->m_kdp_q11=(uint16_t)CLAMP((int32_t)(m->m_conf.p_pid_kd*1000.0f+0.5f),0,65535);
    current_pid_recompute_coeff(m);
    speed_pid_recompute_coeff(m);
    position_pid_recompute_coeff(m);
}
const volatile mc_configuration *mcpwm_foc_get_configuration(bool second) { return &mcpwm_foc_get_motor(second)->m_conf; }

void mcpwm_foc_rl_capture_start(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    __disable_irq();
    m->m_rl_capture_active=0u;
    m->m_rl_capture_have_prev=0u;
    m->m_rl_capture_prev_id_q4=0;
    m->m_rl_capture_n=0u;
    m->m_rl_sum_di2=0; m->m_rl_sum_div=0; m->m_rl_sum_dii=0; m->m_rl_sum_di=0;
    m->m_rl_capture_active=1u;
    __enable_irq();
}
void mcpwm_foc_rl_capture_stop(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    __disable_irq(); m->m_rl_capture_active=0u; __enable_irq();
}
void mcpwm_foc_rl_capture_get(bool second, mcpwm_foc_rl_capture_t *out) {
    if(!out)return;
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    __disable_irq();
    out->samples=m->m_rl_capture_n; out->sum_di2=m->m_rl_sum_di2;
    out->sum_div=m->m_rl_sum_div; out->sum_dii=m->m_rl_sum_dii; out->sum_di=m->m_rl_sum_di;
    __enable_irq();
}

static int16_t amp_to_q4(const mcpwm_foc_motor_t *m, float current) {
    float max_a = m ? m->m_conf.l_current_max * m->m_conf.l_current_max_scale : (float)I_MOT_MAX;
    float min_a = m ? m->m_conf.l_current_min * m->m_conf.l_current_min_scale : -(float)I_MOT_MAX;
    if (max_a <= 0.0f || max_a > (float)I_MOT_MAX) max_a = (float)I_MOT_MAX;
    if (min_a >= 0.0f || min_a < -(float)I_MOT_MAX) min_a = -(float)I_MOT_MAX;
    if (current > max_a) current = max_a;
    if (current < min_a) current = min_a;
    float q = current * (float)FOC_CURRENT_Q4_PER_A;
    if (q > (float)MCCONF_MOTOR_CURRENT_MAX_Q4) q = (float)MCCONF_MOTOR_CURRENT_MAX_Q4;
    if (q < -(float)MCCONF_MOTOR_CURRENT_MAX_Q4) q = -(float)MCCONF_MOTOR_CURRENT_MAX_Q4;
    return (int16_t)q;
}

static void reset_position_pid(mcpwm_foc_motor_t *m){
    m->m_position_integrator=0;m->m_position_prev_error=0;m->m_position_prev_error_mdeg=0;m->m_position_sat_hold=0;
    m->m_position_dt_ticks=0u;m->m_position_d_filter_q15=0;m->m_position_d_proc_filter_q15=0;
    m->m_position_prev_proc_phase=position_feedback_phase_u16(m,m==&m_motor_2);m->m_position_prev_proc_count=m->m_position_counts;m->m_position_proc_dt_ticks=0u;
    m->m_position_breakaway_ticks=0u;m->m_position_no_motion_ticks=0u;m->m_position_motion_seen=0u;
    m->m_position_step_braking=0u;m->m_position_brake_direction=0;
    m->m_position_last_motion_count=m->m_position_counts;
    m->m_position_drive_direction=0;m->m_position_settle_ticks=0;
}

static void reset_current_pi(mcpwm_foc_motor_t *m) {
    m->m_iq_integrator = 0; m->m_id_integrator = 0;
    m->m_iq_sat_hold = 0; m->m_id_sat_hold = 0;
}

static void set_control_mode(mcpwm_foc_motor_t *m, mc_control_mode mode);

static void speed_setpoint_slew_step(mcpwm_foc_motor_t *m, uint32_t dt_ms) {
    const int32_t target_q16 = m->m_speed_target_rpm_q16;
    if (m->m_speed_ramp_rpm_s == 0u) {
        m->m_speed_set_ramp_q16 = target_q16;
        m->m_speed_set_rpm = (int16_t)(target_q16 >> 16);
        return;
    }
    if(dt_ms==0u)dt_ms=1u;
    int64_t step64=((int64_t)m->m_speed_ramp_rpm_s*65536LL*(int64_t)dt_ms+500LL)/1000LL;
    if(step64<1)step64=1;
    if(step64>INT32_MAX)step64=INT32_MAX;
    const int32_t step_q16=(int32_t)step64;

    if (m->m_speed_set_ramp_q16 < target_q16) {
        m->m_speed_set_ramp_q16 += step_q16;
        if (m->m_speed_set_ramp_q16 > target_q16) m->m_speed_set_ramp_q16 = target_q16;
    } else if (m->m_speed_set_ramp_q16 > target_q16) {
        m->m_speed_set_ramp_q16 -= step_q16;
        if (m->m_speed_set_ramp_q16 < target_q16) m->m_speed_set_ramp_q16 = target_q16;
    }
    m->m_speed_set_rpm = (int16_t)(m->m_speed_set_ramp_q16 >> 16);
}

static void speed_mode_enter(mcpwm_foc_motor_t *m) {
    if (m->m_control_mode != CONTROL_MODE_SPEED) {
        const bool second=(m==&m_motor_2);
        const int32_t measured_q16=measured_mech_rpm_q16(m,second);
        set_control_mode(m, CONTROL_MODE_SPEED);
        /* Upstream VESC seeds a ramped SPEED entry from live feedback. Keep the
         * Q16 estimator resolution here (Hall/ABI/PLL selectable) instead of
         * quantizing through legacy int16 m_rpm before the first PID tick. */
        m->m_speed_set_ramp_q16 = measured_q16;
        m->m_speed_set_rpm = (int16_t)CLAMP((measured_q16 >> 16),INT16_MIN,INT16_MAX);
    }
}

static void set_control_mode(mcpwm_foc_motor_t *m, mc_control_mode mode) {
    /* Saat E-stop aktif semua permintaan energize ditolak. CONTROL_MODE_NONE
     * tetap diizinkan agar release/fault path selalu dapat mematikan bridge. */
    if (s_estop_ticks != 0u && mode != CONTROL_MODE_NONE) return;
    if (m->m_control_mode != mode) {
        reset_current_pi(m);
        m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; m->m_speed_d_filter_q4=0; reset_position_pid(m);
        m->m_duty_i_q15=0; m->m_duty_pi_active=0u;
        m->m_brake_vq_prev=0; m->m_brake_speed_dir_prev=0; m->m_brake_zero_duty_samples=0u;
        /* Never seed a new torque reference from measured Iq. With low-side
         * shunts the phase current is not observable in high-impedance/coast
         * states, so OFF telemetry can legitimately be biased/noisy. Preserve
         * the previously commanded/slewed reference across active-mode changes;
         * mcpwm_foc_release_motor() already guarantees this is zero from NONE. */
        m->m_iq_set_ramp_q16 = (int32_t)((int64_t)m->m_iq_set_q4 * 65536LL);
        m->m_iq_target_q4 = m->m_iq_set_q4;
        m->m_control_mode = mode;
    }
}

void mcpwm_foc_set_duty(float duty, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    int32_t dpm=(int32_t)(duty>=0.0f?duty*1000.0f+0.5f:duty*1000.0f-0.5f);
    const int32_t lim=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
    dpm=CLAMP(dpm,-lim,lim);
    /* Upstream mcpwm_foc_set_duty() always enters CONTROL_MODE_DUTY, including
     * duty=0. Zero duty is therefore a commanded phase-short/zero vector, not
     * an implicit coast command. Release is a separate current=0/timeout action. */
    set_control_mode(m, CONTROL_MODE_DUTY);
    m->m_duty_set_permille=(int16_t)dpm;
}
void mcpwm_foc_set_pid_speed(float erpm, bool second) {
    /* VESC COMM_SET_RPM is ERPM. As in VESC foc_run_pid_control_speed, keep a
     * command setpoint and a separately ramped active setpoint. A zero command
     * therefore decelerates through the configured ramp instead of becoming an
     * abrupt zero-speed servo. When the ramp reaches the low-speed release
     * threshold the controller integrators are reset and the bridge is released. */
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    if (erpm > m->m_conf.l_max_erpm) erpm=m->m_conf.l_max_erpm;
    if (erpm < m->m_conf.l_min_erpm) erpm=m->m_conf.l_min_erpm;
    const int16_t mech_rpm = erpm_to_mech_rpm(erpm, second);
    const int32_t new_target_q16 = erpm_to_mech_rpm_q16(erpm, second);
    m->m_speed_target_rpm = mech_rpm;
    m->m_speed_target_rpm_q16 = new_target_q16;
    if (m->m_speed_target_rpm_q16 != 0 || m->m_control_mode == CONTROL_MODE_SPEED) {
        speed_mode_enter(m);
    } else {
        /* Zero ERPM while not already in a speed ramp is simply release. */
        mcpwm_foc_release_motor(second);
    }
}
void mcpwm_foc_set_pid_pos(float position_deg,bool second){
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    /* Incremental ABI cannot safely close the position loop until electrical
     * phase has been established for this boot. Reject torque rather than
     * guessing phase from a reset counter. */
    if(!second && encoder_feedback_selected(m,false) && !m->m_encoder_synced){
        mcpwm_foc_release_motor(false);
        return;
    }
    /* Match vedderb/bldc foc_run_pid_control_pos: without a dedicated encoder,
     * VESC uses the live FOC electrical rotor phase as m_pos_pid_now and closes
     * a shortest-path angular PID on that continuous phase. Do NOT quantize the
     * command to Hall transition counts. */
    while(position_deg>=360.0f)position_deg-=360.0f;
    while(position_deg<0.0f)position_deg+=360.0f;
    uint32_t ph=(uint32_t)(position_deg*(65536.0f/360.0f)+0.5f);
    if(ph>=65536u)ph=0u;
    const uint16_t new_phase=(uint16_t)ph;
    const bool branch_change=(m->m_control_mode==CONTROL_MODE_POS && m->m_pos_pid_phase_mode==0u);
    const int16_t target_delta=(int16_t)(new_phase-m->m_pos_pid_set_phase);
    const bool target_changed=(target_delta>64 || target_delta<-64);
    set_control_mode(m,CONTROL_MODE_POS);
    if(target_changed && !branch_change){
        m->m_position_breakaway_ticks=0u;m->m_position_no_motion_ticks=0u;m->m_position_motion_seen=0u;
        m->m_position_step_braking=0u;m->m_position_brake_direction=0;
        m->m_position_last_motion_count=m->m_position_counts;
        m->m_position_prev_proc_phase=position_feedback_phase_u16(m,second);m->m_position_proc_dt_ticks=0u;
    }
    m->m_pos_pid_phase_mode=1u;
    m->m_pos_pid_set_phase=new_phase;
}
void mcpwm_foc_set_position_counts(int32_t pc,bool second){
    mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);
    if(pc<m->m_position_min_counts)pc=m->m_position_min_counts;
    if(pc>m->m_position_max_counts)pc=m->m_position_max_counts;
    const bool count_mode_active=(m->m_control_mode==CONTROL_MODE_POS && m->m_pos_pid_phase_mode==0u);
    const bool branch_change=(m->m_control_mode==CONTROL_MODE_POS && m->m_pos_pid_phase_mode!=0u);
    const bool target_changed=(pc!=m->m_position_target_counts);
    set_control_mode(m,CONTROL_MODE_POS);
    if(!count_mode_active || branch_change){
        m->m_position_target_ramp_q16=(int64_t)m->m_position_counts * 65536LL;
        m->m_position_pid_target_counts=m->m_position_counts;
        reset_position_pid(m);
    }
    if(target_changed && !branch_change){
        m->m_position_breakaway_ticks=0u; m->m_position_no_motion_ticks=0u;
        m->m_position_last_motion_count=m->m_position_counts;
    }
    m->m_pos_pid_phase_mode=0u;
    m->m_position_target_counts=pc;
}
void mcpwm_foc_set_position_user_counts(int32_t pc,bool second){
    mcpwm_foc_set_position_counts(user_position_to_internal(pc,second),second);
}

void mcpwm_foc_steering_clear_calibration(void){
    mcpwm_foc_motor_t *m=&m_motor_1;
    mcpwm_foc_release_motor(false);
    m->m_steering_span_counts=0;
    m->m_steering_calibrated=0u;
    m->m_steering_homed=0u;
    m->m_position_min_counts=INT32_MIN;
    m->m_position_max_counts=INT32_MAX;
    m->m_position_target_counts=m->m_position_counts;
    m->m_position_pid_target_counts=m->m_position_counts;
    m->m_position_target_ramp_q16=(int64_t)m->m_position_counts * 65536LL;
    reset_position_pid(m);
}

static int32_t steering_safe_span_from_measured(int32_t measured_span){
    if(measured_span==0)return 0;
    int64_t a=measured_span<0?-(int64_t)measured_span:(int64_t)measured_span;
    int64_t safe=(a*(int64_t)MCCONF_STEERING_SAFE_SPAN_PERCENT)/100LL;
    if(safe<(int64_t)MCCONF_STEERING_MIN_SPAN_COUNTS)safe=MCCONF_STEERING_MIN_SPAN_COUNTS;
    /* Force an even magnitude so +/-half is exactly symmetric. */
    safe &= ~1LL;
    if(safe>(int64_t)INT32_MAX)safe=INT32_MAX-1;
    return measured_span<0?-(int32_t)safe:(int32_t)safe;
}

static int32_t steering_runtime_span_from_measured(int32_t measured_span){
    const int32_t safe=steering_safe_span_from_measured(measured_span);
    if(safe==0)return 0;
    const bool neg=safe<0;
    int64_t a=neg?-(int64_t)safe:(int64_t)safe;
    int64_t runtime=(a*(int64_t)MCCONF_STEERING_RUNTIME_SPAN_NUM)/
                    (int64_t)MCCONF_STEERING_RUNTIME_SPAN_DEN;
    if(runtime<(int64_t)MCCONF_STEERING_MIN_SPAN_COUNTS)runtime=MCCONF_STEERING_MIN_SPAN_COUNTS;
    runtime &= ~1LL;
    return neg?-(int32_t)runtime:(int32_t)runtime;
}

bool mcpwm_foc_steering_set_span(int32_t span_counts, bool homed){
    mcpwm_foc_motor_t *m=&m_motor_1;
    int32_t a=span_counts<0?-span_counts:span_counts;
    if(a<MCCONF_STEERING_MIN_SPAN_COUNTS)return false;
    const int32_t runtime_span=steering_runtime_span_from_measured(span_counts);
    const int32_t runtime_abs=runtime_span<0?-runtime_span:runtime_span;
    m->m_steering_span_counts=span_counts; /* persist/report measured hard-stop span */
    m->m_steering_calibrated=1u;
    m->m_steering_homed=homed?1u:0u;
    /* Runtime authority uses the calibrated safe span, then keeps only 8/9
     * of it. This corresponds to the previously measured physical 20..340
     * region while the external coordinate remains normalized 0..360. */
    const int32_t half=runtime_abs/2;
    m->m_position_min_counts=-half;
    m->m_position_max_counts= half;
    m->m_position_target_ramp_step_q16=0u; /* no hidden position slew; VESC PID uses requested target directly */
    m->m_position_target_ramp_q16=(int64_t)m->m_position_counts * 65536LL;
    m->m_position_pid_target_counts=m->m_position_counts;
    return true;
}

bool mcpwm_foc_steering_rebase_left(void){
    mcpwm_foc_motor_t *m=&m_motor_1;
    if(!m->m_steering_calibrated)return false;
    m->m_position_counts=0; m->m_position_abs_counts=0u;
    m->m_position_target_counts=0;
    m->m_position_pid_target_counts=0;
    m->m_position_target_ramp_q16=0;
    reset_position_pid(m);
    m->m_steering_homed=1u;
    return true;
}

bool mcpwm_foc_steering_rebase_center(void){
    mcpwm_foc_motor_t *m=&m_motor_1;
    if(!m->m_steering_calibrated || m->m_steering_span_counts==0 || !m->m_encoder_synced)
        return false;
    /* Only the calibrated span is persisted. The operator places the steering
     * physically at center before each power-on, so after ABI/electrical phase
     * synchronization this boot position is the absolute logical center. */
    m->m_position_counts=0;
    m->m_position_abs_counts=0u;
    m->m_position_target_counts=0;
    m->m_position_pid_target_counts=0;
    m->m_position_target_ramp_q16=0;
    reset_position_pid(m);
    m->m_steering_homed=1u;
    return true;
}

bool mcpwm_foc_steering_is_calibrated(void){return m_motor_1.m_steering_calibrated!=0u;}
bool mcpwm_foc_steering_is_homed(void){return m_motor_1.m_steering_calibrated&&m_motor_1.m_steering_homed;}
int32_t mcpwm_foc_steering_span_counts(void){return m_motor_1.m_steering_span_counts;}
int32_t mcpwm_foc_steering_safe_span_counts(void){return steering_runtime_span_from_measured(m_motor_1.m_steering_span_counts);}

float mcpwm_foc_get_steering_deg(void){
    const mcpwm_foc_motor_t *m=&m_motor_1;
    const int32_t safe_span=steering_runtime_span_from_measured(m->m_steering_span_counts);
    if(!m->m_steering_calibrated || safe_span==0)return 0.0f;
    /* Count 0 is center. Logical feedback saturates at the safe 0/360 endpoints;
     * raw TIM4 and accumulated count remain available separately for diagnostics. */
    float d=((float)m->m_position_counts/(float)safe_span) *
        (MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);
    if(d<MCCONF_STEERING_POS_MIN_DEG)d=MCCONF_STEERING_POS_MIN_DEG;
    if(d>MCCONF_STEERING_POS_MAX_DEG)d=MCCONF_STEERING_POS_MAX_DEG;
    return d;
}

bool mcpwm_foc_set_steering_deg(float deg){
    mcpwm_foc_motor_t *m=&m_motor_1;
    const int32_t safe_span=steering_runtime_span_from_measured(m->m_steering_span_counts);
    if(!m->m_steering_calibrated || !m->m_steering_homed || !m->m_encoder_synced ||
       safe_span==0){mcpwm_foc_release_motor(false);return false;}
    if(deg<MCCONF_STEERING_POS_MIN_DEG)deg=MCCONF_STEERING_POS_MIN_DEG;
    if(deg>MCCONF_STEERING_POS_MAX_DEG)deg=MCCONF_STEERING_POS_MAX_DEG;
    const float f=deg/(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);
    const int32_t target=(int32_t)lroundf(f*(float)safe_span);
    mcpwm_foc_set_position_counts(target,false);
    return true;
}
void mcpwm_foc_set_position_user_limits(int32_t minc,int32_t maxc,bool second){
    mcpwm_foc_motor_t*m=mcpwm_foc_get_motor(second);
    if(minc>maxc){int32_t t=minc;minc=maxc;maxc=t;}
    if(!second){
        m->m_position_min_counts=minc;
        m->m_position_max_counts=maxc;
    }else{
        /* user [min,max] maps to internal [-max,-min] */
        m->m_position_min_counts=user_position_to_internal(maxc,true);
        m->m_position_max_counts=user_position_to_internal(minc,true);
        if(m->m_position_min_counts>m->m_position_max_counts){
            int32_t t=m->m_position_min_counts;m->m_position_min_counts=m->m_position_max_counts;m->m_position_max_counts=t;
        }
    }
    if(m->m_position_target_counts<m->m_position_min_counts)m->m_position_target_counts=m->m_position_min_counts;
    if(m->m_position_target_counts>m->m_position_max_counts)m->m_position_target_counts=m->m_position_max_counts;
}
int32_t mcpwm_foc_get_position_user_counts(bool second){
    return internal_position_to_user(mcpwm_foc_get_motor_const(second)->m_position_counts,second);
}
int32_t mcpwm_foc_get_position_target_user_counts(bool second){
    return internal_position_to_user(mcpwm_foc_get_motor_const(second)->m_position_target_counts,second);
}
int32_t mcpwm_foc_get_position_min_user_counts(bool second){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(second);
    return second?internal_position_to_user(m->m_position_max_counts,true):m->m_position_min_counts;
}
int32_t mcpwm_foc_get_position_max_user_counts(bool second){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(second);
    return second?internal_position_to_user(m->m_position_min_counts,true):m->m_position_max_counts;
}
void mcpwm_foc_reset_position(bool second) {
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    m->m_position_counts = 0;
    m->m_position_abs_counts = 0u;
    m->m_position_target_counts = 0;
    m->m_position_pid_target_counts = 0;
    m->m_position_target_ramp_q16 = 0;
    if (second) odom_r = 0;
    else odom_l = 0;
    reset_position_pid(m);
}

void mcpwm_foc_set_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_CURRENT);
    m->m_iq_target_q4=amp_to_q4(m,current);
    m->m_iq_set_q4=m->m_iq_target_q4;
    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
    m->m_id_set_q4=0;
}
static void mcpwm_foc_set_brake_current_q4(int16_t current_q4, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    int32_t q=current_q4; if(q<0)q=-q;
    if(q==0){mcpwm_foc_release_motor(second);return;}
    if(q>MCCONF_MOTOR_CURRENT_MAX_Q4)q=MCCONF_MOTOR_CURRENT_MAX_Q4;
    set_control_mode(m,CONTROL_MODE_CURRENT_BRAKE);
    m->m_brake_current_q4=(int16_t)q;
    m->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_iq_set_ramp_q16=0; m->m_id_set_q4=0;
}
void mcpwm_foc_set_brake_current(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    mcpwm_foc_set_brake_current_q4(amp_to_q4(m,current<0?-current:current),second);
}
void mcpwm_foc_set_handbrake(float current, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const float min_i=(m->m_conf.cc_min_current>0.0f)?m->m_conf.cc_min_current:MCCONF_CC_MIN_CURRENT;
    if (current < min_i && current > -min_i) { mcpwm_foc_release_motor(second); return; }
    set_control_mode(m, CONTROL_MODE_HANDBRAKE);
    m->m_handbrake_current_q4=amp_to_q4(m,current<0?-current:current);
    if(m->m_handbrake_current_q4<0)m->m_handbrake_current_q4=(int16_t)-m->m_handbrake_current_q4;
    m->m_iq_target_q4=m->m_handbrake_current_q4;
    m->m_iq_set_q4=m->m_iq_target_q4;
    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
    m->m_id_set_q4=0;
}
void mcpwm_foc_set_openloop_current(float current, float rpm, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second); set_control_mode(m, CONTROL_MODE_OPENLOOP);
    m->m_openloop_id_target_q4=0; m->m_openloop_id_ramp_q16=0; m->m_id_set_q4=0; m->m_iq_target_q4=amp_to_q4(m,current); m->m_iq_set_q4=m->m_iq_target_q4; m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536; m->m_openloop_speed_q16=(int32_t)(rpm*65536.0f); m->m_phase_override=1;
}
void mcpwm_foc_set_openloop_phase(float current, float phase, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second); set_control_mode(m, CONTROL_MODE_OPENLOOP_PHASE);
    m->m_openloop_id_target_q4=amp_to_q4(m,current);
    m->m_openloop_id_ramp_q16=(int32_t)m->m_openloop_id_target_q4*65536;
    m->m_id_set_q4=m->m_openloop_id_target_q4;
    m->m_iq_target_q4=0; m->m_iq_set_q4=0; m->m_iq_set_ramp_q16=0;
    while (phase < 0.0f) phase += 360.0f;
    while (phase >= 360.0f) phase -= 360.0f;
    m->m_phase_openloop=(uint16_t)(phase*(65536.0f/360.0f)); m->m_phase_override=1;
}
bool mcpwm_foc_encoder_startup_align(bool second) {
    encoder_align_stage=1u;
    encoder_align_before_count=encoder_align_jog_count=encoder_align_back_count=0u;
    encoder_align_jog_delta=encoder_align_back_delta=0; encoder_align_current_ma=0u;
    if(second){encoder_align_stage=0xE1u;return false;}
    mcpwm_foc_motor_t *m=&m_motor_1;
    if(!encoder_port_active(m,false)){encoder_align_stage=9u;return true;}
    if(!m->m_encoder_configured) encoder_runtime_configure(m,false,true);
    if(!m->m_encoder_configured){encoder_align_stage=0xE2u;return false;}
    encoder_align_stage=2u;
#ifdef STM32F103xE
    /* Every alignment attempt gets fresh A/B evidence. If one quadrature line
     * is dead or disconnected, fail fast instead of escalating steering current
     * for several seconds and starving the USART main-context parser. */
    {
        const uint32_t idr=GPIOB->IDR;
        encoder_gpio_edge_a=0u; encoder_gpio_edge_b=0u; encoder_gpio_edge_pb5=0u; encoder_gpio_samples=0u;
        encoder_gpio_last_ab=(uint8_t)(((idr & GPIO_PIN_6)?1u:0u) | ((idr & GPIO_PIN_7)?2u:0u));
        encoder_gpio_last_pb5=(idr & GPIO_PIN_5)?1u:0u;
    }
#endif

    /* Incremental ABI has no absolute index. Lock the rotor to a known
     * electrical phase with D-axis current, but do not assume a fixed current
     * can overcome steering tyre/linkage stiction. Increase Id gradually and
     * probe only +/-60 electrical degrees. The first level that produces a
     * plausible ABI delta becomes the alignment current for this boot. */
    for(uint32_t t=0u;t<1000u && !mcpwm_foc_dc_cal_done();++t) foc_bounded_delay_ms(1u);
    if(!mcpwm_foc_dc_cal_done()){encoder_align_stage=0xE3u;return false;}
    encoder_align_stage=3u;

    float ceiling=m->m_conf.l_current_max*m->m_conf.l_current_max_scale;
    if(!(ceiling>0.0f))ceiling=m->m_conf.l_current_max;
    if(ceiling>MCCONF_ENCODER_STARTUP_ALIGN_MAX_A)ceiling=MCCONF_ENCODER_STARTUP_ALIGN_MAX_A;
    if(ceiling>MCCONF_STEERING_CAL_CURRENT_MAX_A)ceiling=MCCONF_STEERING_CAL_CURRENT_MAX_A;
    if(ceiling>(float)I_MOT_MAX)ceiling=(float)I_MOT_MAX;
    if(ceiling<0.10f)ceiling=0.10f;
    float current=MCCONF_ENCODER_STARTUP_ALIGN_CURRENT_A;
    if(current<0.10f)current=0.10f;
    float previous=0.0f;
    const float ratio=(float)MCCONF_POLE_PAIRS_LEFT;
    const uint32_t counts=m->m_encoder_counts>=4u?m->m_encoder_counts:MCCONF_ENCODER_COUNTS_DEFAULT;
    const int32_t half=(int32_t)(counts/2u);
    const float expected_f=(float)counts*60.0f/(360.0f*ratio);
    /* A tiny ABI twitch is not enough to establish electrical zero under steering
     * load. Hardware measurements gave ~18 counts at 1.5 A (borderline/no useful
     * torque afterwards) and ~38 counts at 2.5 A. Require >=40% of the ideal
     * +30 electrical-degree excursion before accepting phase lock. */
    int32_t min_move=(int32_t)(expected_f*0.40f); if(min_move<4)min_move=4;
    int32_t max_move=(int32_t)(expected_f*3.0f)+4;
    bool aligned=false;
    bool detected_inverted=false;
    /* On power-up the operator places the wheel at mechanical center. TIM4 is
     * incremental only, so remember how far the rotor moves while Id locks the
     * first electrical phase. After synchronization we can drive that exact
     * relative displacement back and call the original boot point 180 degrees. */
    int32_t boot_center_to_phase0_counts=0;

    m->m_encoder_synced=0u;
    mcpwm_foc_release_motor(false);
    while(current<=ceiling+0.001f){
        const uint32_t level_ref_raw=encoder_read_raw_count();
        encoder_align_current_ma=(uint16_t)(current*1000.0f+0.5f);
        encoder_align_stage=4u;
        /* Hold phase zero while ramping Id from the previous level. */
        for(uint32_t t=1u;t<=MCCONF_ENCODER_STARTUP_ALIGN_RAMP_MS;++t){
            const float f=(float)t/(float)MCCONF_ENCODER_STARTUP_ALIGN_RAMP_MS;
            const float i=previous+(current-previous)*f;
            mcpwm_foc_set_openloop_phase(i,0.0f,false);
            mcpwm_foc_vesc_override_touch(false);
            foc_bounded_delay_ms(1u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        for(uint32_t t=0u;t<MCCONF_ENCODER_STARTUP_ALIGN_HOLD_MS;++t){
            mcpwm_foc_set_openloop_phase(current,0.0f,false);
            mcpwm_foc_vesc_override_touch(false);
            foc_bounded_delay_ms(1u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        {
            const uint32_t held_raw=encoder_read_raw_count();
            int32_t dd=(int32_t)held_raw-(int32_t)level_ref_raw;
            if(dd>half)dd-=(int32_t)counts; else if(dd<-half)dd+=(int32_t)counts;
            boot_center_to_phase0_counts+=dd;
        }

        /* Phase zero is now the physical electrical reference. Rebase the
         * incremental counter and test +60 degrees first. */
        encoder_runtime_set_deg(m,0.0f);
        const uint32_t before=encoder_read_raw_count();
        encoder_align_before_count=before;
        for(uint32_t t=1u;t<=60u;++t){
            mcpwm_foc_set_openloop_phase(current,60.0f*(float)t/60.0f,false);
            mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(2u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        for(uint32_t t=0u;t<80u;++t){mcpwm_foc_vesc_override_touch(false);foc_bounded_delay_ms(1u);}
        uint32_t probe=encoder_read_raw_count();
        encoder_align_jog_count=probe;
        int32_t dp=(int32_t)probe-(int32_t)before;
        if(dp>half)dp-=(int32_t)counts; else if(dp<-half)dp+=(int32_t)counts;
        encoder_align_jog_delta=dp;
#ifdef STM32F103xE
        /* A valid quadrature move must exercise both A and B. Seeing repeated
         * edges on only one input means the position/direction feedback is not
         * trustworthy; never increase Id in that condition. */
        if((encoder_gpio_edge_a>=4u && encoder_gpio_edge_b==0u) ||
           (encoder_gpio_edge_b>=4u && encoder_gpio_edge_a==0u)){
            encoder_align_stage=0xA2u;
            goto align_fail;
        }
#endif
        for(int32_t t=59;t>=0;--t){
            mcpwm_foc_set_openloop_phase(current,60.0f*(float)t/60.0f,false);
            mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(2u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        for(uint32_t t=0u;t<80u;++t){mcpwm_foc_vesc_override_touch(false);foc_bounded_delay_ms(1u);}
        int32_t adp=dp<0?-dp:dp;
        if(adp>=min_move && adp<=max_move){
            detected_inverted=dp<0;
            aligned=true; encoder_align_stage=5u;
            break;
        }

        /* A mechanical stop can block the + direction. Retry the same bounded
         * probe in the negative direction before increasing current. */
        encoder_runtime_set_deg(m,0.0f);
        const uint32_t before_neg=encoder_read_raw_count();
        for(uint32_t t=1u;t<=60u;++t){
            mcpwm_foc_set_openloop_phase(current,-60.0f*(float)t/60.0f,false);
            mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(2u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        for(uint32_t t=0u;t<80u;++t){mcpwm_foc_vesc_override_touch(false);foc_bounded_delay_ms(1u);}
        probe=encoder_read_raw_count();
        encoder_align_back_count=probe;
        int32_t dm=(int32_t)probe-(int32_t)before_neg;
        if(dm>half)dm-=(int32_t)counts; else if(dm<-half)dm+=(int32_t)counts;
        encoder_align_back_delta=dm;
        for(int32_t t=59;t>=0;--t){
            mcpwm_foc_set_openloop_phase(current,-60.0f*(float)t/60.0f,false);
            mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(2u);
            if(m->m_fault!=FAULT_CODE_NONE)goto align_fail;
        }
        for(uint32_t t=0u;t<80u;++t){mcpwm_foc_vesc_override_touch(false);foc_bounded_delay_ms(1u);}
        const int32_t adm=dm<0?-dm:dm;
        if(adm>=min_move && adm<=max_move){
            /* Negative electrical phase producing positive count means the ABI
             * direction must be inverted for the VESC phase equation. */
            detected_inverted=dm>0;
            aligned=true; encoder_align_stage=6u;
            break;
        }

        previous=current;
        current+=MCCONF_ENCODER_STARTUP_ALIGN_STEP_A;
    }
    if(!aligned){encoder_align_stage=0xA1u;goto align_fail;}

    /* The motor pole count is known and ABI offset is meaningless across power
     * cycles. Learn only direction from the bounded phase probe, then define
     * electrical phase 0 as ABI software zero for this boot. */
    m->m_conf.foc_encoder_offset=0.0f;
    m->m_conf.foc_encoder_ratio=ratio;
    m->m_conf.foc_encoder_inverted=detected_inverted;
    encoder_runtime_configure(m,false,false);
    encoder_runtime_set_deg(m,0.0f);
    /* If a hard-stop span is already calibrated, the boot position is assumed
     * to be center (180 deg) as requested. We are currently at electrical
     * phase-zero, displaced by boot_center_to_phase0_counts from that point. */
    const int32_t boot_center=0;
    m->m_position_counts=boot_center_to_phase0_counts;
    m->m_position_target_counts=boot_center;
    m->m_position_abs_counts=(uint32_t)(boot_center_to_phase0_counts<0?
                                        -boot_center_to_phase0_counts:boot_center_to_phase0_counts);
    encoder_feedback_update(m,false,1u);
    m->m_encoder_synced=1u;
    /* A persisted hard-stop span plus the explicit boot-at-center policy gives
     * us an absolute logical reference for this power cycle. Mark homed only
     * after phase/ABI synchronization is valid; without this, SET_POS remains
     * correctly fail-closed forever after every reboot. */
    if(m->m_steering_calibrated && m->m_steering_span_counts!=0)
        m->m_steering_homed=1u;
    encoder_align_stage=9u;
    mcpwm_foc_release_motor(false);
    mcpwm_foc_vesc_override_clear(false);
    return true;

align_fail:
    if(encoder_align_stage<0x80u)encoder_align_stage=(uint8_t)(0xB0u | (encoder_align_stage&0x0Fu));
    m->m_encoder_synced=0u;
    mcpwm_foc_release_motor(false);
    mcpwm_foc_vesc_override_clear(false);
    return false;
}

bool mcpwm_foc_encoder_is_synced(bool second) {
    return !second && m_motor_1.m_encoder_configured && m_motor_1.m_encoder_synced;
}


static float encoder_detect_angle_diff(float a, float b) {
    float d=a-b;
    while(d>180.0f)d-=360.0f;
    while(d<-180.0f)d+=360.0f;
    return d;
}

static bool encoder_detect_move(mcpwm_foc_motor_t *m, float current,
                                float *phase_cont, float target_cont) {
    if(!m || !phase_cont)return false;
    const float dir=target_cont>=*phase_cont?1.0f:-1.0f;
    while((dir>0.0f && *phase_cont<target_cont) ||
          (dir<0.0f && *phase_cont>target_cont)){
        *phase_cont+=dir;
        if((dir>0.0f && *phase_cont>target_cont) ||
           (dir<0.0f && *phase_cont<target_cont))*phase_cont=target_cont;
        mcpwm_foc_set_openloop_phase(current,encoder_norm_deg(*phase_cont),false);
        mcpwm_foc_vesc_override_touch(false);
        foc_bounded_delay_ms(2u);
        if(m->m_fault!=FAULT_CODE_NONE)return false;
    }
    return true;
}

bool mcpwm_foc_encoder_detect(float current, bool second, float *offset, float *ratio, bool *inverted) {
    encoder_stage_set(1u);
    encoder_detect_plus_mdeg=0; encoder_detect_minus_mdeg=0;
    encoder_detect_plus_count=0u; encoder_detect_minus_count=0u; encoder_detect_origin_count=0u;
    encoder_gpio_edge_a=0u; encoder_gpio_edge_b=0u; encoder_gpio_edge_pb5=0u; encoder_gpio_samples=0u;
    encoder_gpio_last_ab=(uint8_t)(((GPIOB->IDR & GPIO_PIN_6)?1u:0u) | ((GPIOB->IDR & GPIO_PIN_7)?2u:0u));
    encoder_gpio_last_pb5=(GPIOB->IDR & GPIO_PIN_5)?1u:0u;
    encoder_detect_plus_id_q4=encoder_detect_plus_iq_q4=0;
    encoder_detect_minus_id_q4=encoder_detect_minus_iq_q4=0;
    uint8_t fail_code=0u;
    if(offset)*offset=1001.0f;
    if(ratio)*ratio=0.0f;
    if(inverted)*inverted=false;
    if(second){encoder_stage_set(0xE1u);return false;}
    mcpwm_foc_motor_t *m=&m_motor_1;
    if(!encoder_port_active(m,false) || !m->m_encoder_configured){encoder_stage_set(0xE2u);return false;}
    encoder_stage_set(2u);
    if(current<0.20f)current=0.20f;
    if(current>MCCONF_STEERING_CAL_CURRENT_MAX_A)current=MCCONF_STEERING_CAL_CURRENT_MAX_A;
    float enc_ceiling=m->m_conf.l_current_max*m->m_conf.l_current_max_scale;
    if(!(enc_ceiling>0.0f))enc_ceiling=m->m_conf.l_current_max;
    if(enc_ceiling>(float)I_MOT_MAX)enc_ceiling=(float)I_MOT_MAX;
    if(current>enc_ceiling)current=enc_ceiling;
    for(uint32_t t=0u;t<1000u && !mcpwm_foc_dc_cal_done();++t)foc_bounded_delay_ms(1u);
    if(!mcpwm_foc_dc_cal_done()){encoder_stage_set(0xE3u);return false;}
    encoder_stage_set(3u);

    /* Steering-safe VESC ABI detection.
     *
     * A generic VESC encoder detect can rotate through several electrical
     * revolutions. That is appropriate for a free motor, but the LEFT motor is
     * mechanically constrained steering. Align the rotor to electrical phase 0
     * first, define that physical point as the incremental-ABI software zero,
     * then probe only +/-60 electrical degrees around zero. With the expected
     * 4 pole-pairs this is about +/-15 mechanical degrees, safely inside the
     * calibrated +/-30 degree steering envelope.
     *
     * Incremental A/B has no absolute index, therefore a persistent absolute
     * encoder offset is meaningless across power cycles. The VESC-equivalent
     * reference is re-established by mcpwm_foc_encoder_startup_align() on every
     * boot: physical phase 0 <-> ABI software zero. Detection therefore returns
     * offset=0 and measures ratio/inversion from the bounded symmetric probes. */
    m->m_encoder_synced=0u;
    mcpwm_foc_release_motor(false);
    for(uint32_t t=1u;t<=250u;++t){
        mcpwm_foc_set_openloop_phase(current*(float)t/250.0f,0.0f,false);
        mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(1u);
        if(m->m_fault!=FAULT_CODE_NONE){fail_code=1u; goto detect_fail;}
    }
    for(uint32_t t=0u;t<400u;++t){
        mcpwm_foc_set_openloop_phase(current,0.0f,false);
        mcpwm_foc_vesc_override_touch(false); foc_bounded_delay_ms(1u);
        if(m->m_fault!=FAULT_CODE_NONE){fail_code=2u; goto detect_fail;}
    }

    encoder_stage_set(4u);
    encoder_runtime_set_deg(m,0.0f);
    encoder_detect_origin_count=encoder_read_raw_count();
    float phase_cont=0.0f;
    float plus_sum=0.0f, minus_sum=0.0f;
    const int samples=3;
    for(int pass=0;pass<samples;++pass){
        if(!encoder_detect_move(m,current,&phase_cont,60.0f)){fail_code=3u; goto detect_fail;}
        foc_bounded_delay_ms(150u);
        const float plus_sample=encoder_detect_angle_diff(encoder_read_deg(),0.0f);
        plus_sum+=plus_sample;
        encoder_detect_plus_mdeg=(int32_t)(plus_sample*1000.0f);
        encoder_detect_plus_count=encoder_read_raw_count();
        encoder_detect_plus_id_q4=m->m_id_q4; encoder_detect_plus_iq_q4=m->m_iq_q4;
        if(!encoder_detect_move(m,current,&phase_cont,0.0f)){fail_code=4u; goto detect_fail;}
        foc_bounded_delay_ms(200u);
        /* Steering gearbox backlash and tyre load can prevent an exact return
         * to the first incremental count even though electrical phase 0 is
         * actively held. ABI has no absolute index, so rebase the software
         * counter at this known electrical reference before the opposite probe. */
        encoder_runtime_set_deg(m,0.0f);

        if(!encoder_detect_move(m,current,&phase_cont,-60.0f)){fail_code=6u; goto detect_fail;}
        foc_bounded_delay_ms(150u);
        const float minus_sample=encoder_detect_angle_diff(encoder_read_deg(),0.0f);
        minus_sum+=minus_sample;
        encoder_detect_minus_mdeg=(int32_t)(minus_sample*1000.0f);
        encoder_detect_minus_count=encoder_read_raw_count();
        encoder_detect_minus_id_q4=m->m_id_q4; encoder_detect_minus_iq_q4=m->m_iq_q4;
        if(!encoder_detect_move(m,current,&phase_cont,0.0f)){fail_code=7u; goto detect_fail;}
        foc_bounded_delay_ms(200u);
        encoder_runtime_set_deg(m,0.0f);
    }

    encoder_stage_set(5u);
    const float plus=plus_sum/(float)samples;
    const float minus=minus_sum/(float)samples;
    /* A steering axis is not a free rotor: one probe can be shortened by a
     * hard-stop, tyre scrub or gearbox backlash. The motor pole count is known
     * independently, so use it as the VESC encoder ratio and use the probe only
     * to prove A/B motion and determine inversion. At least one direction must
     * move by >=25% of the ideal 60-electrical-degree mechanical excursion. */
    const float configured=(float)MCCONF_POLE_PAIRS_LEFT;
    if(configured<1.0f || configured>100.0f){fail_code=11u; goto detect_fail;}
    const float ideal=60.0f/configured;
    const float min_motion=ideal*0.25f;
    const float max_motion=ideal*2.0f;
    const float ap=fabsf(plus), am=fabsf(minus);
    const bool plus_ok=ap>=min_motion && ap<=max_motion;
    const bool minus_ok=am>=min_motion && am<=max_motion;
    if(!plus_ok && !minus_ok){fail_code=9u; goto detect_fail;}
    bool inv=false;
    if(plus_ok) inv=plus<0.0f;
    if(minus_ok){
        const bool inv_minus=minus>0.0f; /* negative electrical target */
        if(plus_ok && inv_minus!=inv){fail_code=10u; goto detect_fail;}
        inv=inv_minus;
    }
    const float rat=configured;

    encoder_runtime_set_deg(m,0.0f);
    if(offset)*offset=0.0f;
    if(ratio)*ratio=rat;
    if(inverted)*inverted=inv;
    m->m_encoder_synced=0u;
    mcpwm_foc_release_motor(false);
    mcpwm_foc_vesc_override_clear(false);
    encoder_stage_set(6u);
    return true;

detect_fail:
    m->m_encoder_synced=0u;
    mcpwm_foc_release_motor(false);
    mcpwm_foc_vesc_override_clear(false);
    encoder_stage_set((uint8_t)(0xA0u | (fail_code & 0x1Fu)));
    return false;
}

void mcpwm_foc_force_bridges_off(void){
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR&=~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;
}

void mcpwm_foc_release_motor(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    /* Fail-safe ordering: hardware output off FIRST. If the ADC ISR pre-empts
     * any of the software-state cleanup below, it cannot generate torque. */
    if (second) {
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
        RIGHT_TIM->RIGHT_TIM_U=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_V=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_W=pwm_res/2u;
    } else {
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        LEFT_TIM->LEFT_TIM_U=pwm_res/2u; LEFT_TIM->LEFT_TIM_V=pwm_res/2u; LEFT_TIM->LEFT_TIM_W=pwm_res/2u;
    }
    FOC_MEMORY_BARRIER();
    const bool was_active=(m->m_control_mode!=CONTROL_MODE_NONE);
    set_control_mode(m, CONTROL_MODE_NONE);
    if(was_active){
        /* The high-impedance amplifier common-mode point can move after a
         * driven interval. Re-acquire its telemetry-only zero once the rotor is
         * stationary; never reuse a stale OFF baseline from before motion. */
        m->m_off_offset_valid=0u; m->m_off_offset_samples=0u;
        m->m_off_settle_ticks=MCCONF_OFF_TELEM_SETTLE_SAMPLES;
        m->m_off_offset_sum0=0; m->m_off_offset_sum1=0; m->m_off_offset_sumdc=0;
        m->m_current_lpf_q16[0]=m->m_current_lpf_q16[1]=0;
        m->m_telem_current_lpf_q16[0]=m->m_telem_current_lpf_q16[1]=m->m_telem_current_lpf_q16[2]=0;
        m->m_id_telem_q4=0; m->m_iq_telem_q4=0; m->m_current_in_telem_counts=0;
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0;
        m->m_telem_avg_samples=0u;
    }
    m->m_id_telem_q4=0; m->m_iq_telem_q4=0; m->m_current_in_telem_counts=0;
    m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0; m->m_telem_avg_samples=0u;
    m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
    m->m_duty_set_permille=0;
    m->m_id_set_q4=0; m->m_openloop_id_target_q4=0;
    m->m_openloop_id_ramp_q16=0;
    m->m_speed_target_rpm=0; m->m_speed_target_rpm_q16=0; m->m_speed_set_rpm=0; m->m_speed_set_ramp_q16=0;
    m->m_speed_integrator=0; m->m_speed_prev_error=0; m->m_speed_sat_hold=0; reset_position_pid(m);
    m->m_state=MC_STATE_OFF;
}

void mcpwm_foc_clear_fault(bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    /* A manual reset must never re-arm a bridge. Release first, then clear the
     * transient fault latch and its qualification state. If the underlying
     * condition still exists, the normal safety checks will fault again. */
    mcpwm_foc_release_motor(second);
    m->m_fault_recovery_ticks=0u;
    m->m_fault_safe_ticks=0u;
    m->m_wrong_voltage_integrator=0u;
    m->m_overspeed_streak=0u;
    m->m_phase_overcurrent_streak=0u;
    FOC_MEMORY_BARRIER();
    m->m_fault=FAULT_CODE_NONE;
    m->m_state=MC_STATE_OFF;
}

void mcpwm_foc_clear_faults(void) {
    mcpwm_foc_clear_fault(false);
    mcpwm_foc_clear_fault(true);
}

void mcpwm_foc_report_watchdog_reset_fault(void) {
    /* VESC defines FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET as code 10. This is
     * a boot-cause report, not a live electrical failure: release both bridges
     * first, expose the fault long enough for the host to observe it, then let
     * the normal VESC fault-stop recovery timer clear it. Do not feed it into
     * the pre-fault flight recorder because there is no valid pre-reset trace. */
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    mcpwm_foc_motor_t *motors[2] = {&m_motor_1, &m_motor_2};
    for (uint8_t i = 0u; i < 2u; ++i) {
        mcpwm_foc_motor_t *m = motors[i];
        m->m_fault = FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET;
        m->m_fault_recovery_ticks = m->m_fault_stop_ticks ? m->m_fault_stop_ticks : 1u;
        m->m_fault_safe_ticks = 0u;
        m->m_state = MC_STATE_OFF;
    }
}

bool mcpwm_foc_estop_active(void) {
    return s_estop_ticks != 0u;
}

void mcpwm_foc_estop_both(uint16_t duration_ms) {
    /* 65535 ms * 16 kHz masih muat uint32. Pembulatan ke atas memastikan
     * durasi tidak pernah lebih pendek dari nilai wire VESC. */
    const uint32_t ticks=((uint32_t)duration_ms*(uint32_t)PWM_FREQ+999u)/1000u;
    s_estop_ticks=ticks;
    /* Urutan release kedua motor mengikuti mc_interface_release_motor_override_both
     * upstream: output dipadamkan sekarang, bukan menunggu ISR berikutnya. */
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
}


static int16_t trq_ca_to_q4(const mcpwm_foc_motor_t *m, int16_t ca) {
    (void)m;
    /* ISR-safe integer-only centiampere scaling: 50 -> 0.50 A,
     * 1500 -> 15.00 A. Runtime VESC commands are clamped separately by the
     * non-ISR amp_to_q4 API. */
    int32_t q4=((int32_t)ca*(int32_t)A2BIT_CONV*16)/100;
    if(q4>MCCONF_MOTOR_CURRENT_MAX_Q4)q4=MCCONF_MOTOR_CURRENT_MAX_Q4;
    else if(q4<-MCCONF_MOTOR_CURRENT_MAX_Q4)q4=-MCCONF_MOTOR_CURRENT_MAX_Q4;
    return (int16_t)q4;
}

void mcpwm_foc_set_mode_command(uint8_t mode, int16_t command, bool run_request,
                                uint16_t openloop_rpm, bool second) {
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    if (mode==TRQ_MODE) {
        if (!run_request) {
            /* Explicit STOP in torque mode is VESC-style release, not braking.
             * First slew Iq_ref to zero; once the active reference is zero,
             * release the bridge so the wheel is truly free-running. Once it
             * is released, keep it released until a new RUN request arrives. */
            if (m->m_control_mode != CONTROL_MODE_NONE) {
                if (m->m_control_mode != CONTROL_MODE_CURRENT) {
                    set_control_mode(m, CONTROL_MODE_CURRENT);
                }
                m->m_iq_target_q4 = 0;
                m->m_id_set_q4 = 0;
                if (m->m_iq_set_q4 == 0 && m->m_iq_set_ramp_q16 == 0) {
                    mcpwm_foc_release_motor(second);
                }
            }
        } else {
            set_control_mode(m, CONTROL_MODE_CURRENT);
            m->m_iq_target_q4 = trq_ca_to_q4(m, command);
            m->m_id_set_q4 = 0;
        }
    } else if (mode==SPD_MODE) {
        /* Speed mode follows the VESC concept of command_rpm -> ramped set_rpm.
         * STOP sets the target to zero but keeps SPEED active while the setpoint
         * ramps down. The low-speed release happens inside motor_control_step,
         * where the speed and current integrators are reset before free-running. */
        m->m_speed_target_rpm = run_request ? command : 0;
        m->m_speed_target_rpm_q16 = (int32_t)((int64_t)m->m_speed_target_rpm * 65536LL);
        if (m->m_speed_target_rpm_q16 != 0 || m->m_control_mode == CONTROL_MODE_SPEED) {
            speed_mode_enter(m);
        }
    } else if (mode==VLT_MODE) {
        /* Zero voltage on this hardware must mean high-impedance/free-run, not
         * a synchronously switched zero vector. After the legacy command ramp
         * reaches zero, release the bridge. */
        if (!run_request || command == 0) {
            if(m->m_control_mode!=CONTROL_MODE_NONE) mcpwm_foc_release_motor(second);
        } else {
            /* Integer equivalent of the VESC duty setter for the legacy ISR
             * source. Do not execute software floating-point at 16 kHz. */
            set_control_mode(m, CONTROL_MODE_DUTY);
            const int32_t lim=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
            m->m_duty_set_permille=(int16_t)CLAMP((int32_t)command,-lim,lim);
        }
    } else if(mode==5u){
        if (!run_request) {
            if(m->m_control_mode!=CONTROL_MODE_NONE) mcpwm_foc_release_motor(second);
        } else {
            const int32_t user_target = second ? positionCommandR : positionCommandL;
            mcpwm_foc_set_position_counts(user_position_to_internal(user_target, second), second);
        }
    } else if (mode==SVPWM_MODE) {
        set_control_mode(m, CONTROL_MODE_OPENLOOP);
        int32_t a=command<0?-(int32_t)command:(int32_t)command;
        if(a>(int32_t)SVPWM_MAX_ID_A)a=(int32_t)SVPWM_MAX_ID_A;
        m->m_openloop_id_target_q4=(int16_t)(a*FOC_CURRENT_Q4_PER_A); m->m_iq_target_q4=0; m->m_iq_set_q4=0;
        int32_t rpm=(int32_t)openloop_rpm; if(rpm>(int32_t)MCCONF_OPENLOOP_RPM_MAX)rpm=(int32_t)MCCONF_OPENLOOP_RPM_MAX;
        m->m_speed_set_rpm=(command<0)?-(int16_t)rpm:(command>0?(int16_t)rpm:0);
        m->m_phase_override=1;
    } else {
        if(m->m_control_mode!=CONTROL_MODE_NONE) mcpwm_foc_release_motor(second);
    }
}

static uint8_t hall_read(mcpwm_foc_motor_t *m, bool second) {
    /* Preserve EFeru master timing: exactly one GPIO snapshot per motor and
     * 16-kHz ADC frame. VESC m_hall_extra_samples is implemented as a rolling
     * majority over synchronized frames instead of repeated immediate reads;
     * this preserves the same odd 1+2N majority semantics without starving the
     * UART/main loop on the 72-MHz Cortex-M3. */
    const uint32_t idr=second?RIGHT_HALL_U_PORT->IDR:LEFT_HALL_U_PORT->IDR;
    uint8_t raw;
    if(!second){
        const uint8_t u=(idr&LEFT_HALL_U_PIN)?0u:1u;
        const uint8_t v=(idr&LEFT_HALL_V_PIN)?0u:1u;
        const uint8_t w=(idr&LEFT_HALL_W_PIN)?0u:1u;
        raw=(uint8_t)((u<<2)|(v<<1)|w);
    }else{
        const uint8_t u=(idr&RIGHT_HALL_U_PIN)?0u:1u;
        const uint8_t v=(idr&RIGHT_HALL_V_PIN)?0u:1u;
        const uint8_t w=(idr&RIGHT_HALL_W_PIN)?0u:1u;
        raw=(uint8_t)((u<<2)|(v<<1)|w);
    }
    m->m_hall_raw_state=raw;
    uint8_t window=m->m_hall_filter_window;
    if(window==0u || window>41u)window=1u;

    uint8_t idx=m->m_hall_sample_index;
    if(idx>=window)idx=0u;
    if(m->m_hall_sample_count>=window){
        const uint8_t old=m->m_hall_sample_history[idx];
        m->m_hall_sample_sum_u-=(uint8_t)((old>>2)&1u);
        m->m_hall_sample_sum_v-=(uint8_t)((old>>1)&1u);
        m->m_hall_sample_sum_w-=(uint8_t)(old&1u);
    }else{
        m->m_hall_sample_count++;
    }
    m->m_hall_sample_history[idx]=raw;
    m->m_hall_sample_sum_u+=(uint8_t)((raw>>2)&1u);
    m->m_hall_sample_sum_v+=(uint8_t)((raw>>1)&1u);
    m->m_hall_sample_sum_w+=(uint8_t)(raw&1u);
    idx++;
    if(idx>=window)idx=0u;
    m->m_hall_sample_index=idx;

    const uint8_t n=m->m_hall_sample_count?m->m_hall_sample_count:1u;
    const uint8_t th=(uint8_t)(n/2u);
    const uint8_t filtered=(uint8_t)(((m->m_hall_sample_sum_u>th)?4u:0u) |
                                     ((m->m_hall_sample_sum_v>th)?2u:0u) |
                                     ((m->m_hall_sample_sum_w>th)?1u:0u));
    m->m_hall_filtered_state=filtered;
    return filtered;
}

static uint8_t hall_table_angle(const mcpwm_foc_motor_t *m, uint8_t hall) {
    return (uint8_t)m->m_conf.foc_hall_table[hall & 7u];
}

static bool hall_feedback_valid(const mcpwm_foc_motor_t *m) {
    if (!m || !m->m_hall_debounce_initialized || !m->m_hall_initialized) return false;
    const uint8_t h=m->m_hall_state & 7u;
    if(h==0u || h==7u) return false;
    const uint8_t angle=hall_table_angle(m,h);
    /* m_hall_pos_prev is the last electrically accepted Hall center. A stable
     * but non-adjacent debounced code is intentionally kept out of the FOC
     * bridge until it returns to the accepted state or reaches a genuinely
     * adjacent state. This turns the sequence-reject counter into a real
     * fail-safe instead of merely diagnostic bookkeeping. */
    return angle<200u && angle==m->m_hall_pos_prev;
}

static bool fault_code_auto_recoverable(mc_fault_code code) {
    /* Configuration/flash corruption is not a transient electrical event and
     * must never be hidden by an automatic reset. Everything else still has to
     * pass fault_recovery_conditions_safe() continuously before clearing. */
    switch(code){
    case FAULT_CODE_FLASH_CORRUPTION:
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG:
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG:
        return false;
    default:
        return code!=FAULT_CODE_NONE;
    }
}

static bool fault_recovery_conditions_safe(const mcpwm_foc_motor_t *m, bool second) {
    if(!m || !fault_code_auto_recoverable(m->m_fault))return false;

    /* Hardware bridge is forced OFF by the ISR while m_fault is latched.  Do
     * not clear until measured D/Q and DC-link current are genuinely quiet. */
    const int32_t iq_lim=((int32_t)MCCONF_FAULT_RECOVERY_SAFE_CURRENT_MA*
                          (int32_t)FOC_CURRENT_Q4_PER_A)/1000;
    const int32_t dc_lim=((int32_t)MCCONF_FAULT_RECOVERY_SAFE_CURRENT_MA*
                          (int32_t)A2BIT_CONV)/1000;
    if(ABS((int32_t)m->m_id_q4)>iq_lim || ABS((int32_t)m->m_iq_q4)>iq_lim ||
       ABS((int32_t)m->m_id_telem_q4)>iq_lim || ABS((int32_t)m->m_iq_telem_q4)>iq_lim ||
       ABS((int32_t)m->m_current_in_counts)>dc_lim ||
       ABS((int32_t)m->m_current_in_telem_counts)>dc_lim)return false;

    if(motor_abs_erpm_for_fault(m,second)>MCCONF_FAULT_RECOVERY_SAFE_ERPM)return false;
    if(batVoltage<(int32_t)m->m_vin_min_adc || batVoltage>(int32_t)m->m_vin_max_adc)return false;
    if(m->m_temp_fet_end_x10>m->m_temp_fet_start_x10 &&
       s_board_temperature_x10>=m->m_temp_fet_start_x10)return false;
    if(!m->m_current_offset_valid)return false;

    /* Sensor faults may clear only when the configured feedback source is back
     * and coherent.  This also prevents a recovered fault from immediately
     * re-arming a bridge onto an invalid Hall/ABI state. */
    if(encoder_feedback_selected(m,second)){
        if(!m->m_encoder_configured || !m->m_encoder_synced)return false;
    }else if(!hall_feedback_valid(m)){
        return false;
    }
    return true;
}


static void foc_pll_update_fixed(mcpwm_foc_motor_t *m, bool second, bool control_update) {
    if(!m || !control_update)return;
    const bool sensor_ready=encoder_feedback_selected(m,second)?
        (m->m_encoder_configured&&m->m_encoder_synced):hall_feedback_valid(m);
    const bool synthetic_phase=m->m_control_mode==CONTROL_MODE_OPENLOOP ||
        m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE ||
        m->m_control_mode==CONTROL_MODE_HANDBRAKE;
    if(!sensor_ready || synthetic_phase){
        m->m_pll_valid=0u;
        m->m_pll_speed_step_q32=0;
        m->m_pll_erpm_q16=0;
        m->m_pll_mech_rpm_q16=0;
        return;
    }
    if(!m->m_pll_valid){
        m->m_pll_phase_acc_q32=(uint32_t)m->m_phase<<16;
        m->m_pll_speed_step_q32=0;
        m->m_pll_erpm_q16=0;
        m->m_pll_mech_rpm_q16=0;
        m->m_pll_valid=1u;
        return;
    }

    /* Bentuk persamaan sama dengan foc_pll_run() upstream VESC:
     * phase += (speed + Kp*error)*dt; speed += Ki*error*dt.
     * Setelah dt dan konversi radian->putaran dilipat ke koefisien slow-path,
     * ISR tinggal multiply integer. Wrap int16 memberi error sudut terpendek. */
    const uint16_t est_phase=(uint16_t)(m->m_pll_phase_acc_q32>>16);
    const int32_t err=(int32_t)(int16_t)(m->m_phase-est_phase);
    int64_t prop=(int64_t)err*(int64_t)m->m_pll_kp_dt_q16;
    if(prop>INT32_MAX)prop=INT32_MAX;
    if(prop<INT32_MIN)prop=INT32_MIN;
    const int64_t phase_step=(int64_t)m->m_pll_speed_step_q32+prop;
    m->m_pll_phase_acc_q32+=(uint32_t)(int32_t)phase_step;

    int64_t next_speed=(int64_t)m->m_pll_speed_step_q32+
        (int64_t)err*(int64_t)m->m_pll_ki_dt2_q16;
    const int64_t lim=m->m_pll_speed_limit_step_q32>0?m->m_pll_speed_limit_step_q32:INT32_MAX;
    if(next_speed>lim)next_speed=lim;
    if(next_speed<-lim)next_speed=-lim;
    m->m_pll_speed_step_q32=(int32_t)next_speed;

    /* ERPM_Q16 = phase_step_Q32 * (60/dt) / 2^16. Faktor 60*Fs/DIV
     * konstan 160000 pada 16 kHz/DIV6; SMULL+shift, tanpa software divide. */
    const int32_t rate=(int32_t)((60u*(uint32_t)PWM_FREQ)/(uint32_t)MCCONF_FOC_CONTROL_DIV);
    int64_t eq16=((int64_t)m->m_pll_speed_step_q32*(int64_t)rate)>>16;
    if(eq16>INT32_MAX)eq16=INT32_MAX;
    if(eq16<INT32_MIN)eq16=INT32_MIN;
    const int32_t eq32=(int32_t)eq16;
    m->m_pll_erpm_q16=eq32;
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    /* Setelah clamp nilai sudah int32; paksa pembagian 32-bit agar Cortex-M3
     * memakai SDIV hardware dan tidak menarik __aeabi_ldivmod ke ADC ISR. */
    m->m_pll_mech_rpm_q16=pp>0?(eq32/pp):eq32;
}

static int16_t hall_angle_diff(uint8_t now, uint8_t prev) {
    int16_t d = (int16_t)now - (int16_t)prev;
    if (d > 100) d -= 200;
    if (d < -100) d += 200;
    return d;
}

static int16_t phase_diff_u16(uint16_t target, uint16_t actual) {
    return (int16_t)(target - actual);
}

static uint16_t hall_angle200_to_phase(uint8_t a) {
    return (uint16_t)(((uint32_t)a * 65536u) / 200u);
}

static uint8_t hall_midpoint200(uint8_t previous_center, int16_t center_delta) {
    int16_t edge = (int16_t)previous_center + center_delta / 2;
    while (edge < 0) edge += 200;
    while (edge >= 200) edge -= 200;
    return (uint8_t)edge;
}

static uint8_t hall_sample_state(mcpwm_foc_motor_t *m, bool second) {
    const uint8_t raw_h = hall_read(m,second);
    if(raw_h==0u || raw_h==7u){
    }

    /* GPIO Hall inputs are asynchronous to the 16-kHz ADC ISR. One transient
     * sample during a switching edge must never become an electrical-sector
     * transition. Accept a new raw code only after consecutive agreement. */
    if (!m->m_hall_debounce_initialized) {
        m->m_hall_debounce_initialized = 1u;
        m->m_hall_state = raw_h;
        m->m_hall_candidate_state = raw_h;
        m->m_hall_candidate_count = 0u;
    } else if (raw_h == m->m_hall_state) {
        m->m_hall_candidate_state = raw_h;
        m->m_hall_candidate_count = 0u;
    } else {
        if (raw_h != m->m_hall_candidate_state) {
            m->m_hall_candidate_state = raw_h;
            m->m_hall_candidate_count = 1u;
        } else if (m->m_hall_candidate_count < 0xffu) {
            m->m_hall_candidate_count++;
        }
        if (m->m_hall_candidate_count >= MCCONF_HALL_DEBOUNCE_SAMPLES) {
            m->m_hall_state = m->m_hall_candidate_state;
            m->m_hall_candidate_count = 0u;
        }
    }
    return m->m_hall_state;
}

static void hall_process_state(mcpwm_foc_motor_t *m, bool second, uint8_t h) {
    const uint8_t angle = hall_table_angle(m, h);
    const bool valid = (h != 0u && h != 7u && angle < 200u);
    /* m_hall_ticks is aged once per 16-kHz ADC frame by hall_update() or the
     * released-motor path. Keeping the timebase outside this heavier estimator
     * lets unchanged Hall sectors skip expensive correction math safely. */

    /* Upstream foc_correct_hall() clears the previous Hall angle when the
     * Hall code is invalid. This target has no sensorless observer fallback,
     * so keeping a stale Hall phase while the bridge is powered is unsafe.
     * Drop estimator lock immediately after the debounced input becomes 0/7
     * (or maps to an invalid table entry). The bridge gate below releases the
     * motor, while the next valid Hall code re-initializes directly to its
     * calibrated sector center. Position/tachometer counters are preserved. */
    if (!valid) {
        const bool closed_loop_active=m->m_control_mode!=CONTROL_MODE_NONE &&
            m->m_control_mode!=CONTROL_MODE_OPENLOOP &&
            m->m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE;
        if(closed_loop_active)mcpwm_foc_release_motor(second);
        m->m_hall_initialized=0u;
        m->m_hall_direction=0;
        m->m_hall_direction_stable_edges=0u;
        m->m_hall_interp_active=0u;
        m->m_hall_period=MCCONF_HALL_TIMEOUT_TICKS;
        m->m_hall_interp_step_q16=0u;
        m->m_hall_rate_limit_step=m->m_hall_rate_min_step?m->m_hall_rate_min_step:1u;
        m->m_rpm=0;
    }

    if (valid) {
        if (!m->m_hall_initialized) {
            m->m_hall_initialized = 1u;
            m->m_hall_pos_prev = angle;          /* current sector center */
            m->m_hall_pos = angle;               /* edge/base until first valid transition */
            m->m_hall_ticks = 0u;
            m->m_hall_direction = 0;
            m->m_phase_hall = hall_angle200_to_phase(angle);
            m->m_phase_hall_target = m->m_phase_hall;
            m->m_rpm = 0;
        } else if (angle != m->m_hall_pos_prev) {
            const uint8_t previous_center = m->m_hall_pos_prev;
            const int16_t ad = hall_angle_diff(angle, previous_center);
            const int16_t aad = ad < 0 ? (int16_t)-ad : ad;
            int8_t dir = 0;
            /* One Hall edge should be about 200/6 = 33.3 VESC angle units.
             * 15..50 tolerates calibration spread while rejecting skipped states. */
            if (aad >= 15 && aad <= 50) dir = ad > 0 ? 1 : -1;

            if (dir != 0) {
                const int8_t motion_dir = hall_motion_direction(second, dir);
                uint16_t period = m->m_hall_ticks;
                if (period == 0u) period = 1u;
                const bool period_outlier =
                    (m->m_hall_direction != 0 &&
                     m->m_hall_direction == dir &&
                     m->m_hall_direction_stable_edges >= MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES &&
                     m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS &&
                     ((uint32_t)period * MCCONF_HALL_PERIOD_OUTLIER_RATIO) < m->m_hall_period);
                uint16_t period_for_filter = period;
                if (period_outlier) {
                    /* Never reject an electrically valid adjacent Hall state just
                     * because its timing changed quickly. During acceleration that
                     * creates a false skipped-state cascade on the next edge. VESC
                     * uses Hall state for phase correction; timing is an estimator
                     * input. Accept the state, but slew-limit the period estimate. */
                    if (m->m_hall_reject_counted_state != h) {
                        m->m_hall_period_reject_count++;
                        m->m_hall_last_reject_reason = 1u;
                        m->m_hall_last_reject_from = 0xffu;
                        for(uint8_t rh=1u;rh<=6u;++rh) if(hall_table_angle(m,rh)==m->m_hall_pos_prev){m->m_hall_last_reject_from=rh;break;}
                        m->m_hall_last_reject_to = h;
                    }
                    const uint16_t floor_period=(uint16_t)(m->m_hall_period/MCCONF_HALL_PERIOD_OUTLIER_RATIO);
                    if(floor_period>0u && period_for_filter<floor_period) period_for_filter=floor_period;
                }
                m->m_hall_reject_counted_state = 0xffu;
                const bool direction_reset =
                    (m->m_hall_direction == 0 || m->m_hall_direction != dir ||
                     m->m_hall_period_hist[0] == MCCONF_HALL_TIMEOUT_TICKS);
                if (direction_reset) {
                    for (int i = 0; i < 4; i++) m->m_hall_period_hist[i] = period_for_filter;
                    m->m_hall_hist_pos = 0u;
                    m->m_hall_direction_stable_edges = 0u;
                } else {
                    m->m_hall_period_hist[m->m_hall_hist_pos++ & 3u] = period_for_filter;
                }
                uint32_t sum = 0u;
                for (int i = 0; i < 4; i++) sum += m->m_hall_period_hist[i];
                m->m_hall_period = (uint16_t)(sum / 4u);
                if (!m->m_hall_period) m->m_hall_period = 1u;
                {
                    const uint32_t sector=65536u/6u;
                    m->m_hall_interp_step_q16=(uint32_t)(((sector<<16)+(m->m_hall_period/2u))/m->m_hall_period);
                    /* VESC 6.00 membatasi laju sudut Hall dari kecepatan edge
                     * TERBARU, bukan dari periode RPM yang sudah dirata-ratakan.
                     * Memakai m_hall_period di sini membuat limiter tertinggal
                     * saat akselerasi. Faktor 3/2 identik dengan angle_step
                     * upstream = hall_speed * dt * 1.5. */
                    const uint32_t raw_period = period > 0u ? period : 1u;
                    /* sector<=10922 and period<=65535, so this ratio fits
                     * entirely in uint32_t; avoid __aeabi_uldivmod on Hall edge. */
                    uint32_t rs = (sector * 3u + raw_period) / (raw_period * 2u);
                    if (rs == 0u) rs = 1u;
                    /* Minimum rate follows VESC foc_correct_hall exactly:
                     * max(hall_erpm, foc_hall_interp_erpm) * dt * 1.5.
                     * m_hall_rate_min_step is precomputed from Motor Config. */
                    const uint32_t min_step=m->m_hall_rate_min_step?m->m_hall_rate_min_step:1u;
                    if (rs < min_step) rs = min_step;
                    if (rs > 32767u) rs = 32767u;
                    m->m_hall_rate_limit_step=(uint16_t)rs;
                }
                m->m_hall_ticks = 0u;
                m->m_hall_direction = dir;
                /* Hall period only changes on an accepted edge. Calculate the
                 * mechanical RPM once here instead of dividing at 16 kHz. */
                {
                    const uint32_t pp=motor_pole_pairs(second);
                    const uint32_t den=(uint32_t)m->m_hall_period*(pp?pp:1u);
                    int32_t rpm=(int32_t)(((uint32_t)PWM_FREQ*10u)/den);
                    /* Telemetry mechanical RPM must not saturate at legacy
                     * N_MOT_MAX=1000, otherwise speed feedback becomes false
                     * above 4000 ERPM on LEFT. int16 is ample for this hardware. */
                    if (rpm > INT16_MAX) rpm = INT16_MAX;
                    m->m_rpm = (int16_t)(rpm * motion_dir);
                }
                if (m->m_hall_direction_stable_edges < 0xffu) m->m_hall_direction_stable_edges++;
                if (m->m_position_abs_counts < UINT32_MAX) m->m_position_abs_counts++;
                /* With Hall feedback one accepted edge is exactly one VESC
                 * tachometer step (6 steps/electrical revolution). Count it at
                 * the edge instead of re-quantizing electrical phase at 16 kHz. */
                if(motion_dir>0){
                    if(m->m_tachometer<INT32_MAX)m->m_tachometer++;
                    if(m->m_position_counts<INT32_MAX)m->m_position_counts++;
                    if(!second){odom_l=(odom_l>=8999)?0:(int16_t)(odom_l+1);}else{odom_r=(odom_r<=0)?8999:(int16_t)(odom_r-1);}
                }else{
                    if(m->m_tachometer>INT32_MIN)m->m_tachometer--;
                    if(m->m_position_counts>INT32_MIN)m->m_position_counts--;
                    if(!second){odom_l=(odom_l<=0)?8999:(int16_t)(odom_l-1);}else{odom_r=(odom_r>=8999)?0:(int16_t)(odom_r+1);}
                }
                if(m->m_tachometer_abs<UINT32_MAX)m->m_tachometer_abs++;
                m->m_hall_pos = hall_midpoint200(previous_center, ad);
                /* VESC 6.00 memulai estimator Hall tepat di midpoint saat edge
                 * baru diterima. Output FOC sendiri tetap melewati rate limiter. */
                m->m_phase_hall_target = hall_angle200_to_phase(m->m_hall_pos);
                m->m_hall_pos_prev = angle;
            } else {
                if (m->m_hall_reject_counted_state != h) {
                    m->m_hall_invalid_transition_count++;
                    m->m_hall_sequence_reject_count++;
                    m->m_hall_last_reject_reason = 2u;
                    m->m_hall_last_reject_from = 0xffu;
                    for(uint8_t rh=1u;rh<=6u;++rh) if(hall_table_angle(m,rh)==m->m_hall_pos_prev){m->m_hall_last_reject_from=rh;break;}
                    m->m_hall_last_reject_to = h;
                    m->m_hall_reject_counted_state = h;
                }
                /* At 16 kHz a stable non-adjacent Hall transition is not a
                 * normal skipped sector at this hardware's ERPM envelope. The
                 * local target has no observer fallback, so release closed-loop
                 * drive and keep the last accepted Hall center latched. A fresh
                 * VESC command can run again only after the Hall sequence has
                 * returned to a valid accepted/adjacent state. Open-loop Hall
                 * detection remains observation-only and is never aborted here. */
                if(m->m_control_mode!=CONTROL_MODE_NONE &&
                   m->m_control_mode!=CONTROL_MODE_OPENLOOP &&
                   m->m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE)
                    mcpwm_foc_release_motor(second);
            }
        } else {
            /* Returned to the last accepted Hall sector. A later departure is
             * a new transition attempt and may be counted once again. */
            m->m_hall_reject_counted_state = 0xffu;
        }
    }

    if (!valid || !m->m_hall_initialized || m->m_hall_ticks > MCCONF_HALL_TIMEOUT_TICKS ||
        !m->m_hall_period || m->m_hall_direction == 0) {
        m->m_rpm = 0;
    }

    /* VESC uses electrical RPM and max(time since edge,last edge period), not
     * mechanical RPM hysteresis. Compare ticks against a precomputed boundary
     * to avoid integer division in the 16-kHz ISR. This also disables
     * interpolation naturally when the wheel slows/stops between Hall edges. */
    if (m->m_hall_direction != 0 && m->m_hall_period > 0u &&
        m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS) {
        uint16_t hall_age=m->m_hall_ticks;
        if(hall_age<m->m_hall_period)hall_age=m->m_hall_period;
        m->m_hall_interp_active=(uint8_t)(m->m_hall_interp_erpm==0u ||
                                         hall_age<=m->m_hall_interp_max_ticks);
    } else {
        m->m_hall_interp_active=0u;
    }

    uint16_t desired = m->m_phase_hall;
    if (valid) {
        /* If this debounced code was rejected as a non-adjacent transition,
         * m_hall_pos_prev deliberately remains the previous accepted center.
         * Never let the rejected code leak into the Hall phase estimator. */
        const uint8_t phase_angle=(m->m_hall_initialized && angle!=m->m_hall_pos_prev) ?
                                  m->m_hall_pos_prev : angle;
        if (m->m_hall_interp_active && m->m_hall_direction != 0 &&
            m->m_hall_period > 0u && m->m_hall_period < MCCONF_HALL_TIMEOUT_TICKS) {
            const uint16_t edge_phase = hall_angle200_to_phase(m->m_hall_pos);
            uint32_t ticks = m->m_hall_ticks;
            if (ticks > m->m_hall_period) ticks = m->m_hall_period;
            const uint32_t frac = (uint32_t)((ticks * m->m_hall_interp_step_q16) >> 16);
            const uint32_t phase_delay_ticks=(uint32_t)MCCONF_HALL_PHASE_ADVANCE_TICKS+
                                             (uint32_t)m->m_hall_filter_delay_ticks;
            const uint32_t debounce_adv=(uint32_t)((phase_delay_ticks * m->m_hall_interp_step_q16) >> 16);
            const uint32_t phase_frac=frac+debounce_adv;
            const uint16_t interp_phase = (uint16_t)(m->m_hall_direction > 0 ?
                                         (uint32_t)edge_phase + phase_frac :
                                         (uint32_t)edge_phase - phase_frac);
            const uint16_t center_phase = hall_angle200_to_phase(phase_angle);
            const int16_t interp_center_err = phase_diff_u16(interp_phase, center_phase);
            const int16_t max_interp_err = (int16_t)(65536u / 12u); /* 30 deg */
            const int16_t abs_interp_err = interp_center_err < 0 ?
                                           (int16_t)-interp_center_err : interp_center_err;

            const bool err_same_direction=
                (interp_center_err>0 && m->m_hall_direction>0) ||
                (interp_center_err<0 && m->m_hall_direction<0);
            if (abs_interp_err < max_interp_err || !err_same_direction) {
                /* foc_correct_hall VESC: interpolate when error is <30 deg OR
                 * its sign differs from Hall speed. The second condition is
                 * essential after acceleration/reversal: an estimator that is
                 * behind the new center must be allowed to catch up instead of
                 * being pulled backwards by the 1%% correction branch. */
                desired = interp_phase;
            } else {
                /* Jika tabel Hall tidak seragam atau akselerasi membuat
                 * interpolator terlalu jauh, VESC menarik estimator ke center
                 * sebesar error/100 setiap ISR, bukan membiarkannya drift. */
                const int16_t center_error = phase_diff_u16(center_phase, m->m_phase_hall_target);
                int16_t correction = (int16_t)(center_error / 100);
                if (correction == 0 && center_error != 0) correction = center_error > 0 ? 1 : -1;
                desired = (uint16_t)(m->m_phase_hall_target + correction);
            }
        } else {
            /* At low speed use the calibrated Hall-sector center directly,
             * just like foc_correct_hall() in VESC 6.00. */
            desired = hall_angle200_to_phase(phase_angle);
        }
    }
    m->m_phase_hall_target = desired;

    /* VESC rate-limits corrected Hall phase to avoid current spikes when a
     * Hall edge or noisy sample moves the target abruptly. The actual Hall
     * sector rate is the primary limit; retain a small minimum around the
     * interpolation-on threshold so low-speed center corrections stay smooth. */
    uint32_t max_step=m->m_hall_rate_limit_step;
    if(max_step==0u)max_step=1u;
    const int16_t pd = phase_diff_u16(desired, m->m_phase_hall);
    if (pd > (int16_t)max_step) m->m_phase_hall = (uint16_t)(m->m_phase_hall + (uint16_t)max_step);
    else if (pd < -(int16_t)max_step) m->m_phase_hall = (uint16_t)(m->m_phase_hall - (uint16_t)max_step);
    else m->m_phase_hall = desired;
}

static void hall_update(mcpwm_foc_motor_t *m, bool second, bool control_update) {
    if (m->m_hall_ticks < 0xffffu) m->m_hall_ticks++;
    const uint8_t before = m->m_hall_state;
    const uint8_t h = hall_sample_state(m, second);
    /* GPIO/debounce remains 16 kHz. Full interpolation/correction is required
     * immediately on an accepted edge, otherwise only at this motor's regulator
     * cadence. This preserves edge timing/safety while freeing CPU for VESC IO. */
    if (!m->m_hall_initialized || h != before || control_update) {
        hall_process_state(m, second, h);
    } else {
        /* The heavy target estimator is decimated, but the VESC Hall phase rate
         * limiter itself must still run every PWM frame. Otherwise phase_hall
         * advances only 1/control_div as fast, lags the sector target by tens of
         * electrical degrees, and torque collapses at speed. This fast path is
         * only a signed compare/add and keeps Park/SVPWM phase at 16 kHz. */
        uint32_t max_step=m->m_hall_rate_limit_step;
        if(max_step==0u)max_step=1u;
        const int16_t pd=phase_diff_u16(m->m_phase_hall_target,m->m_phase_hall);
        if(pd>(int16_t)max_step)m->m_phase_hall=(uint16_t)(m->m_phase_hall+(uint16_t)max_step);
        else if(pd<-(int16_t)max_step)m->m_phase_hall=(uint16_t)(m->m_phase_hall-(uint16_t)max_step);
        else m->m_phase_hall=m->m_phase_hall_target;
    }
}


static void encoder_feedback_update(mcpwm_foc_motor_t *m, bool second, uint16_t elapsed_pwm_ticks) {
    if(elapsed_pwm_ticks==0u)elapsed_pwm_ticks=1u;
    if (!encoder_port_active(m,second) || !m->m_encoder_configured || m->m_encoder_counts<4u) return;
    const uint32_t cnt=encoder_read_raw_count();
    const uint32_t counts=m->m_encoder_counts;
    int32_t delta=(int32_t)cnt-(int32_t)m->m_encoder_prev_count;
    const int32_t half=(int32_t)(counts/2u);
    if(delta>half)delta-=(int32_t)counts;
    else if(delta<-half)delta+=(int32_t)counts;
    m->m_encoder_prev_count=cnt;
    m->m_encoder_raw_count=cnt;
    {
        const uint32_t ad=(uint32_t)(delta<0?-delta:delta);
        uint32_t allowed=(uint32_t)(m->m_encoder_max_delta_per_tick?m->m_encoder_max_delta_per_tick:1u)*
                         (uint32_t)elapsed_pwm_ticks;
        if(allowed>counts/2u)allowed=counts/2u;
        if(ad>allowed){
            /* Lompatan ABI yang secara fisik mustahil dianggap slip/counter
             * corruption. Putus sinkronisasi agar bridge tidak pernah memakai
             * phase elektrik hasil counter yang meragukan. */
            m->m_encoder_synced=0u;
            m->m_encoder_delta_accum=0;
            m->m_encoder_erpm_q16=0;
            m->m_encoder_mech_rpm_q16=0;
            m->m_rpm=0;
            motor_fault_set(m,FAULT_CODE_ENCODER_SLIP);
            return;
        }
    }

    if(delta!=0){
        int64_t pos=(int64_t)m->m_position_counts+(int64_t)delta;
        if(pos>INT32_MAX)pos=INT32_MAX; else if(pos<INT32_MIN)pos=INT32_MIN;
        m->m_position_counts=(int32_t)pos;
        uint32_t ad=(uint32_t)(delta<0?-delta:delta);
        if(UINT32_MAX-m->m_position_abs_counts<ad)m->m_position_abs_counts=UINT32_MAX;
        else m->m_position_abs_counts+=ad;
        m->m_encoder_delta_accum+=delta;
        m->m_encoder_idle_ticks=0u;
    }else if(m->m_encoder_idle_ticks<MCCONF_ENCODER_SPEED_TIMEOUT_TICKS){
        uint32_t idle=(uint32_t)m->m_encoder_idle_ticks+(uint32_t)elapsed_pwm_ticks;
        if(idle>MCCONF_ENCODER_SPEED_TIMEOUT_TICKS)idle=MCCONF_ENCODER_SPEED_TIMEOUT_TICKS;
        m->m_encoder_idle_ticks=(uint16_t)idle;
    }

    uint32_t mech_phase=(uint32_t)(((uint64_t)cnt*m->m_encoder_count_to_phase_q16)>>16);
    mech_phase&=0xffffu;
    /* VESC memisahkan encoder_read_deg() mechanical dari corrected electrical
     * phase. Position/PID memakai nilai mechanical; Park/SVPWM memakai hasil
     * inversion*ratio-offset di bawah. */
    m->m_encoder_mech_phase=(uint16_t)mech_phase;
    if(m->m_conf.foc_encoder_inverted && mech_phase!=0u)mech_phase=65536u-mech_phase;
    const uint32_t elec=(uint32_t)(((uint64_t)mech_phase*m->m_encoder_ratio_q16)>>16);
    m->m_phase_encoder=(uint16_t)(elec-(uint32_t)m->m_encoder_offset_phase);

    {
        uint32_t speed_ticks=(uint32_t)m->m_encoder_speed_ticks+(uint32_t)elapsed_pwm_ticks;
        if(speed_ticks>UINT16_MAX)speed_ticks=UINT16_MAX;
        m->m_encoder_speed_ticks=(uint16_t)speed_ticks;
    }
    /* Finalisasi RPM melibatkan pembagian integer dan tidak diperlukan oleh
     * electrical-phase/current loop. ISR hanya mengakumulasi delta+elapsed;
     * housekeeping 200 Hz mengonsumsi window ini untuk SPEED/telemetry. */
}

static void encoder_feedback_finalize_speed_non_isr(mcpwm_foc_motor_t *m, bool second) {
    if(!m || !encoder_feedback_selected(m,second) || !m->m_encoder_configured)return;

    /* Snapshot/consume seperti arsitektur SmartESC: producer ISR hanya menambah
     * delta dan elapsed tick. Reader slow-path mengambil satu window atomik, lalu
     * division/multiply RPM dilakukan setelah interrupt dibuka kembali. */
    int32_t dc=0;
    uint32_t ticks=0u;
    uint16_t idle_ticks=0u;
    /* Fast lock-free precheck: the 20-ms estimator window means the atomic
     * consume is needed only ~50 Hz, not on every 1-kHz outer-control tick. */
    if(m->m_encoder_speed_ticks<MCCONF_ENCODER_SPEED_WINDOW_TICKS)return;
    __disable_irq();
    if(m->m_encoder_speed_ticks>=MCCONF_ENCODER_SPEED_WINDOW_TICKS){
        dc=m->m_encoder_delta_accum;
        ticks=m->m_encoder_speed_ticks;
        idle_ticks=m->m_encoder_idle_ticks;
        m->m_encoder_delta_accum=0;
        m->m_encoder_speed_ticks=0u;
    }
    __enable_irq();
    if(ticks==0u)return;

    if(dc!=0){
        const int32_t dc_foc=m->m_conf.foc_encoder_inverted?-dc:dc;
        int64_t prod=(int64_t)dc_foc*(int32_t)m->m_encoder_mech_rpm_coeff_q3;
        if(prod>INT32_MAX)prod=INT32_MAX; else if(prod<INT32_MIN)prod=INT32_MIN;
        int64_t mq3=prod/(int32_t)ticks;
        /* ABI feedback keeps the real mechanical speed. The control authority
         * remains l_min/l_max_erpm; do not clip feedback at N_MOT_MAX=1000. */
        if(mq3>((int64_t)INT16_MAX*8LL))mq3=(int64_t)INT16_MAX*8LL;
        if(mq3<((int64_t)INT16_MIN*8LL))mq3=(int64_t)INT16_MIN*8LL;
        const int32_t mech_q3=(int32_t)mq3;
        m->m_encoder_mech_rpm_q16=(int32_t)((int64_t)mech_q3*8192LL);
        int64_t eq16=((int64_t)m->m_encoder_mech_rpm_q16*(int64_t)m->m_encoder_ratio_q16)>>16;
        if(eq16>INT32_MAX)eq16=INT32_MAX; else if(eq16<INT32_MIN)eq16=INT32_MIN;
        m->m_encoder_erpm_q16=(int32_t)eq16;
        m->m_rpm=(int16_t)(mech_q3/8);
    }else if(idle_ticks>=MCCONF_ENCODER_SPEED_TIMEOUT_TICKS){
        m->m_encoder_erpm_q16=0; m->m_encoder_mech_rpm_q16=0; m->m_rpm=0;
    }
}

static void encoder_tachometer_update_non_isr(mcpwm_foc_motor_t *m) {
    if(!m || !encoder_feedback_selected(m,false) || !m->m_encoder_configured || !m->m_encoder_synced){
        s_left_abi_tacho_tracking=0u;
        s_left_abi_tacho_remainder=0;
        if(m)s_left_abi_tacho_pos_last=m->m_position_counts;
        return;
    }

    int32_t pos;
    uint32_t counts,ratio_q16;
    uint32_t e0=0u,x0=0u,e1=0u,x1=0u;
    bool snapshot_ok=false;
    for(uint8_t retry=0u; retry<32u; ++retry) {
        mcpwm_foc_get_irq_epoch(&e0,&x0);
        if(e0!=x0)continue;
        pos=m->m_position_counts; counts=m->m_encoder_counts; ratio_q16=m->m_encoder_ratio_q16;
        mcpwm_foc_get_irq_epoch(&e1,&x1);
        if(e0==e1 && x0==x1 && e1==x1){snapshot_ok=true;break;}
    }
    if(!snapshot_ok || counts<4u || ratio_q16==0u)return;

    if(!s_left_abi_tacho_tracking){
        s_left_abi_tacho_pos_last=pos;
        s_left_abi_tacho_remainder=0;
        s_left_abi_tacho_tracking=1u;
        return;
    }

    const int64_t delta_raw=(int64_t)pos-(int64_t)s_left_abi_tacho_pos_last;
    s_left_abi_tacho_pos_last=pos;
    if(delta_raw==0)return;

    /* VESC tachometer = 6 count/electrical revolution. Correct sign follows
     * corrected electrical phase, sehingga ABI inverted membalik delta raw.
     * Remainder disimpan dalam domain numerator agar fractional sector tidak
     * pernah hilang walaupun housekeeping hanya 200 Hz. Semua 64-bit division
     * berada di slow path, bukan ADC ISR. */
    const int64_t delta_foc=m->m_conf.foc_encoder_inverted?-delta_raw:delta_raw;
    const int64_t den=(int64_t)counts*65536LL;
    int64_t num=s_left_abi_tacho_remainder + delta_foc*(int64_t)ratio_q16*6LL;
    const int64_t steps=num/den;
    s_left_abi_tacho_remainder=num-steps*den;
    if(steps==0)return;

    int64_t t=(int64_t)m->m_tachometer+steps;
    if(t>INT32_MAX)t=INT32_MAX; else if(t<INT32_MIN)t=INT32_MIN;
    m->m_tachometer=(int32_t)t;
    uint64_t ad=(uint64_t)(steps<0?-steps:steps);
    uint64_t ta=(uint64_t)m->m_tachometer_abs+ad;
    m->m_tachometer_abs=ta>UINT32_MAX?UINT32_MAX:(uint32_t)ta;
}


static void openloop_current_ramp_update(mcpwm_foc_motor_t *m) {
    /* Shared mode-4/Hall-detect Id slew. This helper deliberately never changes
     * m_phase_openloop: COMM_DETECT_HALL_FOC depends on the requested synthetic
     * electrical phase remaining exactly where the detector put it. */
    const int32_t target_id_q16=(int32_t)m->m_openloop_id_target_q4*65536;
    int32_t id_step_q16=((int32_t)MCCONF_OPENLOOP_ID_SLEW_A_S*FOC_CURRENT_Q4_PER_A*65536)/PWM_FREQ;
    if(m->m_openloop_id_ramp_q16<target_id_q16){
        m->m_openloop_id_ramp_q16+=id_step_q16;
        if(m->m_openloop_id_ramp_q16>target_id_q16)m->m_openloop_id_ramp_q16=target_id_q16;
    } else if(m->m_openloop_id_ramp_q16>target_id_q16){
        m->m_openloop_id_ramp_q16-=id_step_q16;
        if(m->m_openloop_id_ramp_q16<target_id_q16)m->m_openloop_id_ramp_q16=target_id_q16;
    }
    m->m_id_set_q4=(int16_t)(m->m_openloop_id_ramp_q16>>16);
    m->m_iq_target_q4=0;
    m->m_iq_set_q4=0;
}

static void openloop_update(mcpwm_foc_motor_t *m) {
    if(m->m_openloop_id_target_q4==0){
        /* Standard VESC OPENLOOP_CURRENT: m_openloop_speed_q16 is signed ERPM.
         * Integrate electrical phase directly, with no pole-pair multiplication,
         * acceleration ramp, or alignment delay. */
        const int64_t step=((int64_t)m->m_openloop_speed_q16*
                            (int64_t)OPENLOOP_ERPM_Q16_TO_PHASE_Q32_Q24)>>24;
        m->m_openloop_phase_acc_q32=(uint32_t)((int64_t)m->m_openloop_phase_acc_q32+step);
        m->m_phase_openloop=(uint16_t)(m->m_openloop_phase_acc_q32>>16);
        return;
    }
    /* Legacy hoverboard SVPWM utility below remains a separate rotating-Id
     * hardware mode; it must not redefine the VESC OPENLOOP_CURRENT API. */
    const int16_t target=m->m_speed_set_rpm;
    const int8_t requested_dir=(target>0)?1:(target<0?-1:0);
    const uint16_t absrpm=(uint16_t)(target<0?-target:target);

    openloop_current_ramp_update(m);

    if (requested_dir!=0 && m->m_openloop_direction!=requested_dir) {
        m->m_openloop_direction=requested_dir;
        m->m_openloop_speed_q16=0;
        m->m_openloop_phase_acc_q32=((uint32_t)SVPWM_ALIGN_PHASE)<<16;
        m->m_phase_openloop=SVPWM_ALIGN_PHASE;
        m->m_openloop_align_ticks=(uint16_t)(((uint32_t)PWM_FREQ*MCCONF_OPENLOOP_ALIGN_MS)/1000u);
        m->m_openloop_primed=0u;
        m->m_openloop_id_ramp_q16=0;
        m->m_id_set_q4=0;
        reset_current_pi(m);
    }

    if (!m->m_openloop_primed && requested_dir!=0) {
        reset_current_pi(m);
        m->m_openloop_primed=1u;
    }

    if (m->m_openloop_align_ticks && requested_dir!=0) {
        m->m_openloop_align_ticks--;
        m->m_phase_openloop=(uint16_t)(m->m_openloop_phase_acc_q32>>16);
        return;
    }

    const int32_t target_speed_q16=q16_from_i32_sat((int32_t)absrpm);
    int32_t speed_step=((int32_t)MCCONF_OPENLOOP_ACCEL_RPM_S*65536)/PWM_FREQ;
    if(m->m_openloop_speed_q16<target_speed_q16){
        m->m_openloop_speed_q16+=speed_step;
        if(m->m_openloop_speed_q16>target_speed_q16)m->m_openloop_speed_q16=target_speed_q16;
    } else if(m->m_openloop_speed_q16>target_speed_q16){
        m->m_openloop_speed_q16-=speed_step;
        if(m->m_openloop_speed_q16<target_speed_q16)m->m_openloop_speed_q16=target_speed_q16;
    }

    const int8_t phase_dir=(requested_dir!=0)?requested_dir:m->m_openloop_direction;
    const uint32_t phaseStep=(uint32_t)(((uint64_t)(uint32_t)m->m_openloop_speed_q16*
                                               m->m_openloop_step_per_rpm_q32)>>16);
    if(phase_dir>0)m->m_openloop_phase_acc_q32+=phaseStep;
    else if(phase_dir<0)m->m_openloop_phase_acc_q32-=phaseStep;
    m->m_phase_openloop=(uint16_t)(m->m_openloop_phase_acc_q32>>16);

    if(requested_dir==0 && m->m_openloop_speed_q16==0 && m->m_id_set_q4==0){
        m->m_openloop_direction=0;
        m->m_openloop_primed=0u;
        m->m_openloop_phase_acc_q32=((uint32_t)SVPWM_ALIGN_PHASE)<<16;
        m->m_phase_openloop=SVPWM_ALIGN_PHASE;
    }
}

static int16_t current_pi_vesc_state(int16_t err_q4, uint32_t kp_err_q8, uint32_t ki_err_q8,
                                    int16_t max_counts, int16_t min_counts,
                                    int32_t *integ_v_q16, uint8_t *sat) {
    /* Fixed-point equivalent of upstream VESC control_current():
     *   integrator += Ierr * Ki * dt
     *   output      = integrator + Ierr * Kp
     * followed by voltage-vector saturation/anti-windup. Integrator is stored
     * in physical volts Q16, therefore a Vin change does not rescale its state. */
    const int64_t pmul=(int64_t)err_q4*(int64_t)kp_err_q8;
    const int64_t imul=(int64_t)err_q4*(int64_t)ki_err_q8;
    const int64_t p=(pmul>=0)?((pmul+128)>>8):-(((-pmul)+128)>>8);
    const int64_t istep=(imul>=0)?((imul+128)>>8):-(((-imul)+128)>>8);
    int64_t isum=(int64_t)*integ_v_q16+istep;
    int32_t lim_counts=max_counts;
    if(lim_counts<0)lim_counts=-lim_counts;
    int32_t nlim=min_counts; if(nlim<0)nlim=-nlim;
    if(nlim<lim_counts)lim_counts=nlim;
    if(lim_counts<0)lim_counts=0;
    int64_t vmax=((int64_t)lim_counts*(int64_t)s_volt_q24_per_mod_count+128)>>8;
    if(isum>vmax)isum=vmax; else if(isum<-vmax)isum=-vmax;
    int64_t v=p+isum;
    uint8_t clipped=0u;
    if(v>vmax){v=vmax;clipped=1u;}else if(v<-vmax){v=-vmax;clipped=1u;}
    *integ_v_q16=(int32_t)isum;
    *sat=clipped;
    int64_t prod=v*(int64_t)s_mod_counts_per_volt_q16;
    int64_t counts=(prod>=0)?((prod+(1LL<<31))>>32):-(((-prod)+(1LL<<31))>>32);
    if(counts>max_counts)counts=max_counts;
    if(counts<min_counts)counts=min_counts;
    return (int16_t)counts;
}

/* Cortex-M3 position PID math: keep all expensive configuration work outside
 * the ADC ISR. 64-bit products are 32x32->64 (SMULL/UMULL); there is no 64-bit
 * software division in these helpers. */
#define POSITION_I_DT_Q15 ((uint32_t)((((1ULL<<46) + ((uint64_t)500000u*MCCONF_OUTER_PID_HZ))) / ((uint64_t)1000000u*MCCONF_OUTER_PID_HZ)))
#define POSITION_D_RATE_Q8 ((uint32_t)((((uint64_t)32768u*MCCONF_OUTER_PID_HZ*256u) + 500000u) / 1000000u))

static int32_t position_p_term_q15(int32_t error_mdeg, uint32_t kp_eff) {
    if(error_mdeg==0 || kp_eff==0u)return 0;
    const bool neg=error_mdeg<0;
    const uint32_t ae=(uint32_t)(neg?-(int64_t)error_mdeg:error_mdeg);
    const uint64_t prod=(uint64_t)ae*kp_eff;
    if(prod>=1000000ULL)return neg?-32768:32768;
    /* prod < 1e6 here, so prod*4096 fits uint32 exactly. 32768/1e6 = 4096/125000. */
    const uint32_t q=((uint32_t)prod*4096u)/125000u;
    return neg?-(int32_t)q:(int32_t)q;
}

static int32_t position_i_step_q16(int32_t error_mdeg, uint32_t ki_eff, uint32_t dt_ms) {
    const uint32_t coeff=ki_eff*POSITION_I_DT_Q15; /* <= 1.73e9 for uint16 gain */
    if(dt_ms==0u)dt_ms=1u;
    int64_t x=(int64_t)error_mdeg*(int32_t)coeff;
    x=(x>=0)?(x>>15):-(((-x)>>15));
    x*=dt_ms;
    if(x>INT32_MAX)x=INT32_MAX; else if(x<INT32_MIN)x=INT32_MIN;
    return (int32_t)x;
}

static int32_t position_d_error_q15(int32_t de_mdeg, uint32_t kd_eff, uint32_t dt_ticks) {
    if(kd_eff==0u)return 0;
    if(dt_ticks==0u)dt_ticks=1u;
    const uint32_t coeff=kd_eff*POSITION_D_RATE_Q8; /* <= 1.47e9 */
    int64_t num=(int64_t)de_mdeg*(int32_t)coeff;
    num=(num>=0)?(num>>8):-(((-num)>>8));
    const int64_t lim=(int64_t)32768*(int32_t)dt_ticks;
    if(num>=lim)return 32768;
    if(num<=-lim)return -32768;
    return (int32_t)num/(int32_t)dt_ticks; /* hardware SDIV, never __aeabi_ldivmod */
}

static int32_t position_d_process_phase_q15(int32_t signed_delta, uint16_t coeff_q4, uint32_t dt_ticks) {
    if(dt_ticks==0u)dt_ticks=1u;
    int64_t num=-(int64_t)signed_delta*(int32_t)coeff_q4;
    num=(num>=0)?(num>>4):-(((-num)>>4));
    const int64_t lim=(int64_t)32768*(int32_t)dt_ticks;
    if(num>=lim)return 32768;
    if(num<=-lim)return -32768;
    return (int32_t)num/(int32_t)dt_ticks;
}

static int16_t position_q15_to_iq_q4(int32_t out_q15, int32_t limit_q4) {
    int32_t prod=out_q15*limit_q4; /* bounded: 32768 * motor-current-q4 */
    return (int16_t)(prod/32768);  /* constant power-of-two; compiler emits shift */
}

static int16_t position_pid_iq_target_step(mcpwm_foc_motor_t *m, bool second, uint32_t dt_ms) {
    /* Stock COMM_SET_POS remains VESC-compatible in electrical degrees. On
     * this Hall-only hoverboard, direct position->Iq is under-damped because one
     * Hall sector is 60 electrical degrees. Use the hardware-safe cascade
     * position PID -> ERPM -> speed PI -> Iq -> current PI -> Vq for phase mode.
     * The custom long-range Hall-count API remains a separate branch. */
    int32_t error_mdeg;
    int32_t count_error=0;
    int32_t limit_q4=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    if(m->m_pos_pid_phase_mode && !second && encoder_feedback_selected(m,false)){
        /* Steering-specific torque ceiling. The wire command remains standard
         * COMM_SET_POS, but a 30-deg command must never inherit a 15-A traction
         * current limit. This ceiling is tuned from real steering tests. */
        const int32_t steering_cap=((int32_t)FOC_CURRENT_Q4_PER_A*
            (int32_t)MCCONF_STEERING_POSITION_CURRENT_MAX_MA)/1000;
        if(limit_q4>steering_cap)limit_q4=steering_cap;
    }
    if(m->m_pos_pid_phase_mode){
        /* VESC foc_run_pid_control_pos: shortest-path angular PID langsung
         * terhadap posisi rotor yang tersedia. Pada board hoverboard tanpa
         * encoder terpisah, m_phase adalah estimasi fase rotor FOC/Hall seperti
         * fallback state_now->phase pada base VESC. Keluaran PID dinormalisasi
         * [-1,1] lalu diskalakan ke batas arus posisi yang aman. */
        const uint16_t pos_now=position_feedback_phase_u16(m,second);
        const int16_t phase_err=(int16_t)(m->m_pos_pid_set_phase-pos_now);
        error_mdeg=(int32_t)(((int64_t)phase_err*360000LL)>>16);
        error_mdeg*=position_error_sign(m,second);
        const uint16_t gain_scale=position_gain_scale_q15(m,error_mdeg);
        const uint32_t kp_eff=((uint32_t)m->m_kpp_q11*gain_scale+16384u)>>15;
        const uint32_t ki_eff=((uint32_t)m->m_kip_q16*gain_scale+16384u)>>15;
        const uint32_t kd_eff=((uint32_t)m->m_kdp_q11*gain_scale+16384u)>>15;

        int32_t p_q15=position_p_term_q15(error_mdeg,kp_eff);

        if(ki_eff==0u){
            m->m_position_integrator=0;
        }else{
            const int32_t istep=position_i_step_q16(error_mdeg,ki_eff,dt_ms);
            int64_t isum=(int64_t)m->m_position_integrator+istep;
            int32_t i_lim_q15=32768-(p_q15<0?-p_q15:p_q15);
            if(i_lim_q15<0)i_lim_q15=0;
            int64_t ilim=(int64_t)i_lim_q15*65536LL;
            if(isum>ilim)isum=ilim; else if(isum<-ilim)isum=-ilim;
            m->m_position_integrator=(int32_t)isum;
        }

        int32_t d_raw_q15=0;
        if(kd_eff!=0u){
            uint32_t acc=(uint32_t)m->m_position_dt_ticks+(dt_ms?dt_ms:1u);
            m->m_position_dt_ticks=(uint16_t)(acc>65535u?65535u:acc);
            if(error_mdeg==m->m_position_prev_error_mdeg){
                d_raw_q15=0;
            }else{
                const uint32_t dt_ticks=m->m_position_dt_ticks?m->m_position_dt_ticks:1u;
                const int32_t de_mdeg=error_mdeg-m->m_position_prev_error_mdeg;
                d_raw_q15=position_d_error_q15(de_mdeg,kd_eff,dt_ticks);
                m->m_position_dt_ticks=0u;
            }
        }else{
            m->m_position_dt_ticks=0u;
        }
        m->m_position_prev_error_mdeg=error_mdeg;
        const int32_t dd=d_raw_q15-m->m_position_d_filter_q15;
        m->m_position_d_filter_q15 +=
            (int32_t)(((int64_t)dd*m->m_position_kd_filter_q16)>>16);

        /* VESC p_pid_kd_proc: derivative dari posisi terukur, bukan error.
         * Akumulasikan dt saat fase tidak berubah agar sensor resolusi rendah
         * tidak menghasilkan spike D pada edge berikutnya. */
        { uint32_t acc=(uint32_t)m->m_position_proc_dt_ticks+(dt_ms?dt_ms:1u);
          m->m_position_proc_dt_ticks=(uint16_t)(acc>65535u?65535u:acc); }
        int32_t dproc_raw_q15=0;
        const uint16_t proc_now=position_feedback_phase_u16(m,second);
        const int16_t proc_delta=(int16_t)(proc_now-m->m_position_prev_proc_phase);
        if(proc_delta!=0){
            const uint32_t dt_ticks=m->m_position_proc_dt_ticks?m->m_position_proc_dt_ticks:1u;
            const int32_t signed_delta=(int32_t)proc_delta*position_error_sign(m,second);
            const int32_t dp=position_d_process_phase_q15(signed_delta,m->m_position_kd_proc_phase_coeff_q4,dt_ticks);
            dproc_raw_q15=(int32_t)(((int64_t)dp*gain_scale)>>15);
            m->m_position_prev_proc_phase=proc_now;
            m->m_position_proc_dt_ticks=0u;
        }
        const int32_t dpdiff=dproc_raw_q15-m->m_position_d_proc_filter_q15;
        m->m_position_d_proc_filter_q15 +=
            (int32_t)(((int64_t)dpdiff*m->m_position_kd_filter_q16)>>16);

        int32_t out_q15=p_q15+(m->m_position_integrator>>16)+
                        m->m_position_d_filter_q15+m->m_position_d_proc_filter_q15;
        out_q15=CLAMP(out_q15,-32768,32768);

        /* Upstream foc_run_pid_control_pos: normalized output [-1,1] is
         * multiplied by l_current_max*l_current_max_scale. `limit_q4` is that
         * configured envelope in this fixed-point port. No Hall-only deadband
         * or hidden 0.6-A cap is allowed on standard COMM_SET_POS. */
        return position_q15_to_iq_q4(out_q15,limit_q4);
    }

    /* Count-position branch. Calibrated LEFT steering uses a slew-limited
     * internal PID target while preserving the final COMM_SET_POS target. */
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    const bool encoder_count_mode=encoder_port_active(m,second) && m->m_encoder_configured &&
                                  m->m_encoder_counts>=4u;
    const bool steering_count_mode=encoder_count_mode && !second && m->m_steering_calibrated;
    int32_t pid_target=m->m_position_target_counts;
    m->m_position_pid_target_counts=pid_target;
    m->m_position_target_ramp_q16=q16_from_i32_sat(pid_target);
    int64_t ec64=(int64_t)pid_target-(int64_t)m->m_position_counts;
    if(ec64>32767)ec64=32767; else if(ec64<-32768)ec64=-32768;
    count_error=(int32_t)ec64;
    if(encoder_count_mode){
        if(!second && m->m_steering_calibrated && m->m_steering_span_counts!=0){
            /* Steering user coordinates are calibrated independently from the
             * motor-shaft ABI CPR. The measured hard-stop span represents the
             * complete -30..+30 degree wheel envelope, so using 4096 CPR here
             * can understate steering error by the gearbox/linkage ratio and
             * leave the position loop below static breakaway torque. The signed
             * span already maps count direction to LEFT/RIGHT user direction. */
            const int32_t steering_span_abs=m->m_steering_span_counts<0?
                                            -m->m_steering_span_counts:m->m_steering_span_counts;
            /* Logical span sign only swaps the VESC 0/360 endpoint labels.
             * Closed-loop torque direction is an electrical/ABI property and
             * must follow foc_encoder_inverted, exactly like generic ABI mode.
             * Otherwise a target toward center can drive farther into a stop. */
            error_mdeg=(count_error*60000)/steering_span_abs;
            error_mdeg*=position_error_sign(m,second);
        }else{
            /* Generic ABI position: one custom count is one quadrature count. */
            error_mdeg=(int32_t)(((int64_t)count_error*(int32_t)m->m_encoder_mdeg_per_count_q12)>>12);
            error_mdeg*=position_error_sign(m,second);
        }
    }else{
        const int32_t mdeg_per_count=360000/(6*pp);
        error_mdeg=count_error*mdeg_per_count;
    }
    m->m_position_prev_error=(int16_t)count_error;
    const uint16_t gain_scale=position_gain_scale_q15(m,error_mdeg);
    const uint32_t kp_eff=((uint32_t)m->m_kpp_q11*gain_scale+16384u)>>15;
    const uint32_t ki_eff=((uint32_t)m->m_kip_q16*gain_scale+16384u)>>15;
    const uint32_t kd_eff=((uint32_t)m->m_kdp_q11*gain_scale+16384u)>>15;

    /* PID posisi ternormalisasi seperti VESC. Hasilnya berada pada Q15 dan
     * selanjutnya diskalakan ke batas arus Iq yang aman untuk hardware. */
    int32_t p_q15=position_p_term_q15(error_mdeg,kp_eff);

    if(ki_eff==0u){
        m->m_position_integrator=0;
    }else{
        const int32_t istep=position_i_step_q16(error_mdeg,ki_eff,dt_ms);
        int64_t isum=(int64_t)m->m_position_integrator+istep;
        int32_t i_lim_q15=32768-(p_q15<0?-p_q15:p_q15);
        if(i_lim_q15<0)i_lim_q15=0;
        int64_t ilim=(int64_t)i_lim_q15*65536LL;
        if(isum>ilim)isum=ilim; else if(isum<-ilim)isum=-ilim;
        m->m_position_integrator=(int32_t)isum;
    }

    if(kd_eff!=0u){
        int32_t d_raw_q15=0;
        uint32_t acc=(uint32_t)m->m_position_dt_ticks+(dt_ms?dt_ms:1u);
        m->m_position_dt_ticks=(uint16_t)(acc>65535u?65535u:acc);
        if(error_mdeg==m->m_position_prev_error_mdeg){
            d_raw_q15=0;
        }else{
            const uint32_t dt_ticks=m->m_position_dt_ticks?m->m_position_dt_ticks:1u;
            const int32_t de_mdeg=error_mdeg-m->m_position_prev_error_mdeg;
            d_raw_q15=position_d_error_q15(de_mdeg,kd_eff,dt_ticks);
            m->m_position_dt_ticks=0u;
        }
        const int32_t dd=d_raw_q15-m->m_position_d_filter_q15;
        m->m_position_d_filter_q15 += (int32_t)(((int64_t)dd*m->m_position_kd_filter_q16)>>16);
    }else{
        m->m_position_dt_ticks=0u;
        m->m_position_d_filter_q15=0;
    }
    m->m_position_prev_error_mdeg=error_mdeg;
    const int32_t d_q15=m->m_position_d_filter_q15;

    /* VESC p_pid_kd_proc: derivative of measured POSITION in the same
     * coordinate as the position error. For calibrated steering, convert the
     * encoder-count delta through the measured +/-30 deg steering span. */
    { uint32_t acc=(uint32_t)m->m_position_proc_dt_ticks+(dt_ms?dt_ms:1u);
      m->m_position_proc_dt_ticks=(uint16_t)(acc>65535u?65535u:acc); }
    int32_t dproc_raw_q15=0;
    const int32_t proc_count=m->m_position_counts;
    const int32_t proc_delta=proc_count-m->m_position_prev_proc_count;
    if(proc_delta!=0 && m->m_conf.p_pid_kd_proc>0.0f){
        const uint32_t dt_ticks=m->m_position_proc_dt_ticks?m->m_position_proc_dt_ticks:1u;
        int32_t delta_mdeg;
        if(steering_count_mode && m->m_steering_span_counts!=0){
            const int32_t span_abs=m->m_steering_span_counts<0?-m->m_steering_span_counts:m->m_steering_span_counts;
            delta_mdeg=(int32_t)(((int64_t)proc_delta*60000LL)/span_abs);
            delta_mdeg*=position_error_sign(m,second);
        }else if(encoder_count_mode){
            delta_mdeg=(int32_t)(((int64_t)proc_delta*(int32_t)m->m_encoder_mdeg_per_count_q12)>>12);
            delta_mdeg*=position_error_sign(m,second);
        }else{
            delta_mdeg=proc_delta*(360000/(6*pp));
        }
        /* Normalized process-D: -d(position_deg)/dt * kd_proc. */
        const int64_t num=-(int64_t)delta_mdeg*(int64_t)m->m_position_kd_proc_coeff_q16;
        int64_t q=(num>=0)?(num>>16):-(((-num)>>16));
        q/= (int32_t)dt_ticks;
        if(q>32768)q=32768; else if(q<-32768)q=-32768;
        dproc_raw_q15=(int32_t)((q*(int64_t)gain_scale)>>15);
        m->m_position_prev_proc_count=proc_count;
        m->m_position_proc_dt_ticks=0u;
    }
    const int32_t dpdiff=dproc_raw_q15-m->m_position_d_proc_filter_q15;
    m->m_position_d_proc_filter_q15 +=
        (int32_t)(((int64_t)dpdiff*m->m_position_kd_filter_q16)>>16);
    const int32_t dproc_q15=m->m_position_d_proc_filter_q15;

    int32_t out_q15=p_q15+(m->m_position_integrator>>16)+d_q15+dproc_q15;
    out_q15=CLAMP(out_q15,-32768,32768);

    /* Keep VESC's normalized position PID, but scale it by a hardware-safe
     * position current range instead of multiplying by the full 15 A motor
     * limit and clipping afterwards. Post-clipping turns small position errors
     * into bang-bang current; pre-scaling preserves proportional authority near
     * the target while still providing enough breakaway torque at large error. */
    const int32_t pos_current_ma=steering_count_mode ?
        (int32_t)MCCONF_STEERING_POSITION_CURRENT_MAX_MA : (int32_t)MCCONF_POSITION_CURRENT_MAX_MA;
    int32_t pos_lim_q4=((int32_t)FOC_CURRENT_Q4_PER_A*pos_current_ma)/1000;
    if(pos_lim_q4>limit_q4)pos_lim_q4=limit_q4;
    int32_t iq_cmd_q4=position_q15_to_iq_q4(out_q15,pos_lim_q4);

    if(encoder_count_mode){
        /* Encoder/steering position is PID-only. No anti-stiction or minimum
         * current pulse may modify the PID torque request. */
        m->m_position_breakaway_ticks=0u;
        m->m_position_no_motion_ticks=0u;
        if(steering_count_mode && m->m_conf.m_invert_direction)
            iq_cmd_q4=-iq_cmd_q4;
        return (int16_t)iq_cmd_q4;
    }

    /* Generic count-position follows the same PID-only rule. */
    m->m_position_breakaway_ticks=0u;
    m->m_position_no_motion_ticks=0u;
    m->m_position_drive_direction=0;
    m->m_position_step_braking=0u;
    return (int16_t)iq_cmd_q4;
}

static int16_t speed_pid_iq_target_erpm_step(mcpwm_foc_motor_t *m, bool second,
                                                   int32_t target_erpm_q16, int32_t output_limit_q4, uint32_t dt_ms) {
    /* VESC speed PID -> Iq. The normal speed mode uses the full configured
     * current range. Hall-position mode reuses the exact same regulator with a
     * smaller output ceiling, so position cannot wind the speed integrator into
     * multi-ampere torque while a wheel is mechanically blocked. */
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    const int32_t full_limit_q4=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    int32_t limit_q4=output_limit_q4;
    if(limit_q4<=0 || limit_q4>full_limit_q4)limit_q4=full_limit_q4;
    const int64_t target64=(int64_t)target_erpm_q16;
    int64_t measured64=(int64_t)measured_mech_rpm_q16(m,second)*pp;
    int64_t error64 = target64 - measured64;
    if (error64 > INT32_MAX) error64 = INT32_MAX;
    if (error64 < INT32_MIN) error64 = INT32_MIN;
    const int32_t error_q16 = (int32_t)error64;

    /* VESC foc_run_pid_control_speed zeros the torque request below
     * s_pid_min_erpm. With Hall sensing this also prevents a low-speed
     * boundary-hunting limit cycle where one Hall sector spans a large
     * fraction of the requested speed period. */
    const int64_t min_erpm_q16 = (int64_t)m->m_speed_release_erpm_q16;
    const int64_t target_abs_q16 = target64 < 0 ? -target64 : target64;
    if (target_abs_q16 < min_erpm_q16) {
        m->m_speed_integrator = 0;
        m->m_speed_sat_hold = 0u;
        m->m_speed_prev_error = error_q16;
        return 0;
    }

    /* Keep 0.25-ERPM resolution while avoiding three software 64-bit divides.
     * Coefficients are recomputed when VESC Tool/EEPROM tuning changes. */
    const int32_t error_q2=error_q16>>14;
    int64_t pnorm64=((int64_t)error_q2*(int64_t)m->m_speed_kp_coeff_q16)>>16;
    if(pnorm64>32768)pnorm64=32768; else if(pnorm64<-32768)pnorm64=-32768;
    const int32_t p_q4=(int32_t)((pnorm64*(int64_t)full_limit_q4)>>15);

    if(dt_ms==0u)dt_ms=1u;
    const int64_t i_step_base=((int64_t)error_q2*(int64_t)m->m_speed_ki_coeff_q16)>>16;
    const int64_t i_step=i_step_base*(int64_t)dt_ms;
    const int64_t i_lim = (int64_t)limit_q4 * 65536LL;
    const int32_t i_old=m->m_speed_integrator;

    int32_t d_q4 = 0;
    if (m->m_speed_kd_coeff_q8 != 0u) {
        int64_t de64=(int64_t)error_q16-(int64_t)m->m_speed_prev_error;
        if(de64>INT32_MAX)de64=INT32_MAX; else if(de64<INT32_MIN)de64=INT32_MIN;
        const uint32_t de_mag_q16=(uint32_t)(de64<0 ? -de64 : de64);
        const int32_t de_q2=de64<0 ? -(int32_t)(de_mag_q16>>14) : (int32_t)(de_mag_q16>>14);
        int64_t dq=((int64_t)de_q2*(int64_t)m->m_speed_kd_coeff_q8)>>8;
        dq/=(int32_t)dt_ms;
        if(dq>limit_q4)dq=limit_q4; else if(dq<-limit_q4)dq=-limit_q4;
        const int32_t d_raw_q4=(int32_t)dq;
        const int32_t dd=d_raw_q4-m->m_speed_d_filter_q4;
        m->m_speed_d_filter_q4 += (int32_t)(((int64_t)dd*m->m_speed_kd_filter_q16)>>16);
        d_q4=m->m_speed_d_filter_q4;
    } else {
        m->m_speed_d_filter_q4=0;
    }

    /* Sama seperti VESC: output tick ini memakai I-term yang sudah tersimpan,
     * kemudian integrator diperbarui untuk tick berikutnya. Anti-windup tetap
     * lebih konservatif: jangan mengintegrasikan lebih jauh ke arah saturasi. */
    int32_t out_q4 = p_q4 + (i_old >> 16) + d_q4;
    if(m->m_kis_q16==0u){
        m->m_speed_integrator=0;
    }else{
        int64_t i_candidate=(int64_t)i_old+i_step;
        if(i_candidate>i_lim)i_candidate=i_lim;
        if(i_candidate<-i_lim)i_candidate=-i_lim;
        m->m_speed_integrator=(int32_t)i_candidate;
    }
    m->m_speed_sat_hold=0u;
    m->m_speed_prev_error = error_q16;

    if (!m->m_conf.s_pid_allow_braking) {
        const int64_t erpm20_q16 = 20LL << 16;
        if (measured64 > erpm20_q16 && out_q4 < 0) out_q4 = 0;
        if (measured64 < -erpm20_q16 && out_q4 > 0) out_q4 = 0;
    }
    return (int16_t)CLAMP(out_q4,-limit_q4,limit_q4);
}

static int16_t speed_pid_iq_target_step(mcpwm_foc_motor_t *m,bool second,uint32_t dt_ms){
    const int32_t pp=(int32_t)motor_pole_pairs(second);
    int64_t target64=(int64_t)m->m_speed_set_ramp_q16*pp;
    if(target64>INT32_MAX)target64=INT32_MAX;
    if(target64<INT32_MIN)target64=INT32_MIN;
    const int32_t full_limit=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;

    /* COMM_SET_RPM authority is the persisted VESC speed PID only.  Do not
     * inject a project-specific fixed-current breakaway pulse here: on the
     * deployed Hall traction motor a 1 A kick can cross several Hall sectors
     * before the first reliable speed sample and overshoot a 100-200 eRPM
     * request by nearly an order of magnitude.  Stiction/startup behaviour is
     * therefore governed by the tuned Kp/Ki/Kd and normal current limits. */
    return speed_pid_iq_target_erpm_step(m,second,(int32_t)target64,full_limit,dt_ms);
}

static int16_t duty_control_iq_target_step(mcpwm_foc_motor_t *m) {
    int32_t set=m->m_duty_set_permille;
    const int32_t max_permille=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
    set=CLAMP(set,-max_permille,max_permille);
    if(set==0){m->m_duty_i_q15=0;m->m_duty_pi_active=0u;return 0;}
    const int32_t now=m->m_duty_now_permille;
    const int32_t aset=set<0?-set:set, anow=now<0?-now:now;
    const bool same_sign=(now==0)||((set>0)==(now>0));
    const int32_t limit=m->m_current_limit_q4>0?m->m_current_limit_q4:MCCONF_MOTOR_CURRENT_MAX_Q4;
    if(!(same_sign && anow>aset+10)){
        m->m_duty_i_q15=0; m->m_duty_pi_active=0u;
        return (int16_t)(set>0?limit:-limit);
    }
    /* Upstream VESC uses a duty down-ramp PI that commands Iq while the current
     * PI remains the inner loop. Fixed-point equivalent uses nominal bus scaling
     * precomputed outside the ISR, avoiding float/division in the 16-kHz path. */
    const int32_t err=set-now;
    const int32_t p_q15=(int32_t)(((int64_t)err*m->m_duty_kp_q12_per_permille)>>12);
    int32_t i=m->m_duty_i_q15+(int32_t)(((int64_t)err*m->m_duty_ki_q12_per_permille)>>12);
    i=CLAMP(i,-32768,32768); m->m_duty_i_q15=i; m->m_duty_pi_active=1u;
    int32_t out=CLAMP(p_q15+i,-32768,32768);
    return (int16_t)(((int64_t)out*limit)/32768LL);
}

static int16_t phase_current_counts_to_q4(const mcpwm_foc_motor_t *m, int16_t counts) {
    /* Keep a one/two-sample high-duty shunt glitch from kicking the current PI
     * far outside the absolute-current envelope while the qualified ABS fault
     * logic below decides whether it is persistent. */
    const int32_t max_counts=(m && m->m_abs_current_limit_counts>0)?
                             m->m_abs_current_limit_counts:
                             (int32_t)(MCCONF_L_ABS_CURRENT_MAX*(float)A2BIT_CONV);
    int32_t c=CLAMP((int32_t)counts,-max_counts,max_counts);
    int32_t q4=c*16;
    /* Preserve the generated-controller numeric saturation as a final guard. */
    if(q4>27200)q4=27200; else if(q4<-27200)q4=-27200;
    return (int16_t)q4;
}

static int16_t telemetry_lpf_step(int32_t *state_q16, uint16_t alpha_q16, int16_t sample) {
    const int32_t target=q16_from_i32_sat((int32_t)sample);
    const int32_t diff=target-*state_q16;
    *state_q16 += (int32_t)(((int64_t)diff*(int32_t)alpha_q16)>>16);
    return (int16_t)(*state_q16>>16);
}

static inline int16_t div8_toward_zero_i16(int32_t v) {
    /* Cortex-M3: exact C signed /8 semantics tanpa SDIV. Arithmetic right shift
     * membulatkan negatif ke -inf, jadi tambahkan 7 lebih dulu agar hasil tetap
     * truncation-toward-zero seperti pembagian C. Input caller dibatasi kecil. */
    if(v<0)v+=7;
    return (int16_t)(v>>3);
}

static int16_t off_telem_deadband_counts(int16_t sample) {
    /* High-impedance current amplifiers have a separate common-mode operating
     * point. After its stationary zero calibration, preserve changes around
     * that baseline for passive/back-drive telemetry, but strip ADC noise. */
    const int16_t db=(int16_t)MCCONF_OFF_TELEM_DEADBAND_COUNTS;
    if (sample > db) return (int16_t)(sample-db);
    if (sample < -db) return (int16_t)(sample+db);
    return 0;
}

static void telemetry_avg_push(mcpwm_foc_motor_t *m, int16_t id_q4, int16_t iq_q4, int16_t ibus_counts) {
    /* COMM_GET_VALUES normally consumes this every ~20 ms. Bound the window so
     * a disconnected VESC Tool can never overflow the 32-bit accumulators. */
    if (m->m_telem_avg_samples >= 1024u) {
        m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0;
        m->m_telem_avg_samples=0u;
    }
    m->m_telem_sum_id_q4 += id_q4;
    m->m_telem_sum_iq_q4 += iq_q4;
    m->m_telem_sum_ibus_counts += ibus_counts;
    m->m_telem_avg_samples++;
}

/* VESC-standard independent FOC observer for Rotor Position diagnostics.
 *
 * The actuator phase m_phase can be Encoder/Hall corrected and therefore must
 * never be used as mcpwm_foc_get_phase_observer(). This diagnostic observer is
 * an Ortega flux observer, matching the default VESC 6.00 observer equation.
 * It runs only on this motor's current-regulator slot to preserve the proven
 * F103 ISR budget; the 50/100-Hz diagnostic output remains fully realtime.
 * No atan2 is executed in the ADC ISR -- phase conversion happens on readout. */
static bool foc_observer_model_valid(const mcpwm_foc_motor_t *m) {
    if (!m) return false;
    const mc_configuration *c=&m->m_conf;
    return c->foc_observer_type==FOC_OBSERVER_ORTEGA_ORIGINAL &&
           isfinite(c->foc_motor_r) && c->foc_motor_r>0.0f && c->foc_motor_r<=2.0f &&
           isfinite(c->foc_motor_l) && c->foc_motor_l>0.0f && c->foc_motor_l<=0.1f &&
           isfinite(c->foc_motor_flux_linkage) && c->foc_motor_flux_linkage>0.000001f && c->foc_motor_flux_linkage<=1.0f &&
           isfinite(c->foc_observer_gain) && c->foc_observer_gain>0.0f;
}

static void foc_observer_bootstrap(mcpwm_foc_motor_t *m, float lia, float lib) {
    const float lambda=m->m_conf.foc_motor_flux_linkage;
    int16_t sn=0,cs=32767;
    foc_sin_cos_q15(m->m_phase,&sn,&cs);
    m->m_observer_x1=lambda*((float)cs/32767.0f)+lia;
    m->m_observer_x2=lambda*((float)sn/32767.0f)+lib;
    m->m_observer_l_ia=lia;
    m->m_observer_l_ib=lib;
    m->m_observer_valid=1u;
}

static void foc_observer_update_diag(mcpwm_foc_motor_t *m, float dt) {
    if (!foc_observer_model_valid(m)) { m->m_observer_valid=0u; return; }
    const mc_configuration *c=&m->m_conf;
    const float inv_i=1.0f/(float)FOC_CURRENT_Q4_PER_A;
    const float ia=(float)m->m_i_alpha_q4*inv_i;
    const float ib=(float)m->m_i_beta_q4*inv_i;
    const float id=(float)m->m_id_q4*inv_i;
    const float iq=(float)m->m_iq_q4*inv_i;
    float L=c->foc_motor_l;
    if (fabsf(id)>0.1f || fabsf(iq)>0.1f) {
        const float den=id*id+iq*iq;
        if (den>0.000001f) L=L-c->foc_motor_ld_lq_diff*0.5f+c->foc_motor_ld_lq_diff*(iq*iq/den);
    }
    const float lia=L*ia, lib=L*ib;
    if (!m->m_observer_valid || !isfinite(m->m_observer_x1) || !isfinite(m->m_observer_x2)) {
        foc_observer_bootstrap(m,lia,lib); return;
    }

    /* Use the voltage vector that produced this current sample (previous held
     * D/Q output), exactly as a model observer should. Internal 16000 modulation
     * maps to Vdq = mod*Vbus/1.5 in upstream VESC. */
    const foc_dq_t vprev={m->m_vd,m->m_vq};
    foc_ab_t vab={0,0};
    foc_inv_park(&vprev,m->m_phase,&vab);
    const float vin=bus_voltage_now();
    const float vscale=vin/(1.5f*(float)MCCONF_FOC_VOLTAGE_MAX);
    float va=(float)vab.alpha*vscale;
    float vb=(float)vab.beta*vscale;

    /* Dead-time compensation mengikuti update_valpha_vbeta() VESC. Upstream
     * mengurangi modulation alpha/beta berdasarkan tanda arus phase, lalu
     * memakai hasil itu hanya untuk model/observer dan estimasi input current;
     * nilai kompensasi TIDAK ditambahkan ke switching command. Board F103 ini
     * tidak memiliki ADC Va/Vb/Vc, sehingga cabang yang ekuivalen adalah
     * mengoreksi tegangan model dari command modulation dan Vbus. Gunakan D/Q
     * telemetry-filtered seperti id_filter/iq_filter upstream agar noise zero
     * crossing tidak membuat sign compensation bergetar. Jalur ini 200 Hz,
     * seluruh float tetap di luar ADC ISR 16 kHz. */
    if(m->m_state==MC_STATE_RUNNING && c->foc_dt_us>0.0f){
        const foc_dq_t ifilt={m->m_id_telem_q4,m->m_iq_telem_q4};
        foc_ab_t iab={0,0};
        foc_inv_park(&ifilt,m->m_phase,&iab);
        int32_t sign_alpha_q15=0,sign_beta_q15=0;
        foc_deadtime_sign_q15(iab.alpha,iab.beta,&sign_alpha_q15,&sign_beta_q15);
        const float mod_comp=c->foc_dt_us*1.0e-6f*(float)PWM_FREQ;
        const float volt_comp=mod_comp*(2.0f/3.0f)*vin;
        va-=((float)sign_alpha_q15/32768.0f)*volt_comp;
        vb-=((float)sign_beta_q15/32768.0f)*volt_comp;
    }
    const float lambda=c->foc_motor_flux_linkage;
    const float ex=m->m_observer_x1-lia;
    const float ey=m->m_observer_x2-lib;
    float err=lambda*lambda-(ex*ex+ey*ey);
    if (err>0.0f) err=0.0f; /* VESC Ortega convergence rule */

    float slow=c->foc_observer_gain_slow;
    if (!isfinite(slow) || slow<0.0f) slow=0.05f;
    if (slow>1.0f) slow=1.0f;
    float duty=(float)m->m_duty_now_permille/1000.0f;
    if (duty<0.0f) duty=-duty;
    float gain_scale=duty*vin/40.0f;
    if (gain_scale<slow) gain_scale=slow;
    const float gamma_half=(c->foc_observer_gain*gain_scale*4.0f)*0.5f;
    if(!(dt>0.0f && dt<=0.1f))dt=0.005f;
    m->m_observer_x1 += (va-c->foc_motor_r*ia + gamma_half*ex*err)*dt;
    m->m_observer_x2 += (vb-c->foc_motor_r*ib + gamma_half*ey*err)*dt;
    m->m_observer_l_ia=lia;
    m->m_observer_l_ib=lib;

    const float limit=lambda*8.0f;
    if (!isfinite(m->m_observer_x1) || !isfinite(m->m_observer_x2) ||
        fabsf(m->m_observer_x1)>limit || fabsf(m->m_observer_x2)>limit) {
        foc_observer_bootstrap(m,lia,lib); return;
    }
    /* Upstream prevents observer-vector magnitude from collapsing. */
    const float mag2=m->m_observer_x1*m->m_observer_x1+m->m_observer_x2*m->m_observer_x2;
    if (mag2 < lambda*lambda*0.25f) {
        m->m_observer_x1*=1.1f;
        m->m_observer_x2*=1.1f;
    }
}

static int32_t motor_erpm_for_decoupling(const mcpwm_foc_motor_t *m, bool second) {
    if(m->m_pll_valid)return m->m_pll_erpm_q16>>16;
    int64_t e=(int64_t)measured_mech_rpm_raw_q16(m,second)*(int64_t)motor_pole_pairs(second);
    e>>=16;
    if(e>INT32_MAX)e=INT32_MAX;
    if(e<INT32_MIN)e=INT32_MIN;
    return (int32_t)e;
}

static void current_decoupling_apply(mcpwm_foc_motor_t *m, bool second, foc_dq_t *v, int16_t vector_limit) {
    if(!m||!v||m->m_control_mode>=CONTROL_MODE_HANDBRAKE)return;
    const mc_foc_cc_decoupling_mode mode=m->m_conf.foc_cc_decoupling;
    if(mode==FOC_CC_DECOUPLING_DISABLED)return;
    const int32_t erpm=motor_erpm_for_decoupling(m,second);
    if(erpm==0)return;

    int32_t dec_vd=0, dec_vq=0, bemf=0;
    if(mode==FOC_CC_DECOUPLING_CROSS || mode==FOC_CC_DECOUPLING_CROSS_BEMF){
        int64_t x=(int64_t)m->m_iq_q4*(int64_t)erpm;
        x=(x*(int64_t)m->m_dec_lq_coeff_q24)>>24;
        if(x>vector_limit)x=vector_limit;
        if(x<-(int64_t)vector_limit)x=-(int64_t)vector_limit;
        dec_vd=(int32_t)x;
        x=(int64_t)m->m_id_q4*(int64_t)erpm;
        x=(x*(int64_t)m->m_dec_ld_coeff_q24)>>24;
        if(x>vector_limit)x=vector_limit;
        if(x<-(int64_t)vector_limit)x=-(int64_t)vector_limit;
        dec_vq=(int32_t)x;
    }
    if(mode==FOC_CC_DECOUPLING_BEMF || mode==FOC_CC_DECOUPLING_CROSS_BEMF){
        int64_t x=((int64_t)erpm*(int64_t)m->m_dec_flux_coeff_q24)>>24;
        if(x>vector_limit)x=vector_limit;
        if(x<-(int64_t)vector_limit)x=-(int64_t)vector_limit;
        bemf=(int32_t)x;
    }
    /* Tanda identik PMSM/VESC: vd -= omega*Lq*Iq;
     * vq += omega*Ld*Id + omega*lambda. */
    int32_t vd=(int32_t)v->d-dec_vd;
    int32_t vq=(int32_t)v->q+dec_vq+bemf;
    v->d=(int16_t)CLAMP(vd,-vector_limit,vector_limit);
    v->q=(int16_t)CLAMP(vq,-vector_limit,vector_limit);
}

static void motor_control_step(mcpwm_foc_motor_t *m, bool second, int16_t i0_counts,
                               int16_t i1_counts, int16_t idc_counts, bool control_update) {
    if(m->m_config_update_active){
        /* Configuration is being rebuilt in main context. Never inspect m_conf
         * while its struct copy/caches can be partial; output remains centered
         * and MOE is held off by the outer ADC gate. */
        m->m_pwm_a=m->m_pwm_b=m->m_pwm_c=0;
        m->m_ccr_a=m->m_ccr_b=m->m_ccr_c=pwm_res/2u;
        m->m_state=MC_STATE_OFF;
        return;
    }
    uint32_t profSensorStart=0u;
    if(foc_prof_detail_sample)profSensorStart=DWT->CYCCNT;
    /* PB6/PB7 are mutually exclusive: once LEFT ABI owns TIM4, never sample
     * those lines as Hall V/W. Encoder raw/count/phase remains live even OFF. */
    const bool encoder_port=encoder_port_active(m,second);
    const bool encoder_fast_sample=control_update;
    /* LEFT ABI steering is mechanically slow. TIM4 counts edges in hardware, so
     * reading/scaling the encoder on every 16-kHz PWM frame only burns Cortex-M3
     * time. Sample it on the regulator slot (or continuously during encoder
     * commissioning). Current/DC protection remains in the 16-kHz ADC ISR. */
    if (encoder_port) {
        if(encoder_fast_sample){
            const uint16_t elapsed_pwm_ticks=(m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE)?1u:(uint16_t)MCCONF_FOC_CONTROL_DIV;
            encoder_feedback_update(m,second,elapsed_pwm_ticks);
        }
    }
    else if (m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) {
        /* Fixed-phase Hall detection only needs the debounced physical code. */
        (void)hall_sample_state(m,second);
    } else if (m->m_control_mode==CONTROL_MODE_NONE) {
        /* Released motor: one Hall GPIO snapshot every ADC frame, but run the
         * heavier edge estimator only when the debounced sector actually changes.
         * Between edges only age the last period so passive/manual-spin telemetry
         * remains real without paying closed-loop interpolation cost at 16 kHz. */
        if(m->m_hall_ticks<0xffffu)m->m_hall_ticks++;
        const uint8_t before=m->m_hall_state;
        const uint8_t hs=hall_sample_state(m,second);
        if(!m->m_hall_initialized || hs!=before){
            hall_process_state(m,second,hs);
        }else{
            if(m->m_hall_ticks>MCCONF_HALL_TIMEOUT_TICKS){
                m->m_rpm=0; m->m_hall_direction=0; m->m_hall_interp_active=0u;
            }
        }
    } else hall_update(m, second, control_update);
    if (m->m_control_mode==CONTROL_MODE_OPENLOOP) {
        openloop_update(m);
    } else if (m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) {
        /* Fixed phase/current is set directly by the VESC setter. Detect routines
         * perform any desired current ramp explicitly before entering each step. */
    }

    /* Hall estimator needs the generated +30deg sector-center offset. Synthetic
     * measured/open-loop phase does NOT: the old generated measurement branch
     * subtracts 30deg before a +30deg sine table, giving a net zero offset. */
    if (m->m_control_mode==CONTROL_MODE_HANDBRAKE) {
        /* Upstream VESC fixes electrical phase at zero in handbrake mode. The
         * requested current then produces a stationary locking field rather
         * than a rotating torque command. */
        m->m_phase=0u;
    } else {
        const bool openloop_phase=m->m_phase_override &&
            (m->m_control_mode==CONTROL_MODE_OPENLOOP || m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
        if(openloop_phase) m->m_phase=m->m_phase_openloop;
        else if(encoder_feedback_selected(m,second) && m->m_encoder_synced) m->m_phase=m->m_phase_encoder;
        else if(!encoder_port_active(m,second)) m->m_phase=m->m_phase_hall;
        /* ABI yang belum sync tidak boleh menjadi active FOC phase. Raw
         * mechanical/electrical encoder tetap tersedia lewat diagnostic getter,
         * sedangkan m_phase mempertahankan phase aktif terakhir sampai alignment. */
        /* POSITION feedback untuk outer PID disnapshot pada scheduler 1 kHz
         * di main context. ISR hanya menjaga phase listrik yang dibutuhkan FOC. */
    }
    if(foc_prof_detail_sample){
        const uint32_t used=DWT->CYCCNT-profSensorStart;
        if(used>foc_prof_sensor_max_cycles)foc_prof_sensor_max_cycles=used;
    }

    /* Hall fast hold berlaku untuk RIGHT dan juga LEFT bila LEFT dipilih Hall.
     * Pada 5/6 frame tanpa regulator, Hall tetap disample/debounce/advance 16 kHz,
     * lalu hanya rotate held Vd/Vq menjadi SVPWM baru. Semua PID/current/telemetry
     * branch generic dilewati. LEFT ABI tidak masuk sini dan tetap menahan CCR
     * sampai regulator slot karena TIM4 menangkap encoder di hardware. */
    if(!control_update && !encoder_port &&
       m->m_control_mode!=CONTROL_MODE_NONE &&
       m->m_control_mode!=CONTROL_MODE_OPENLOOP &&
       m->m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE){
        const bool source_enabled_fast=(enable!=0u)||mcpwm_foc_vesc_command_live(second);
        if(source_enabled_fast && m->m_fault==FAULT_CODE_NONE && hall_feedback_valid(m) &&
           m->m_bridge_settle_ticks==0u){
            uint32_t profSvpwmStart=0u;
            if(foc_prof_detail_sample)profSvpwmStart=DWT->CYCCNT;
            foc_dq_t hv={m->m_vd,m->m_vq};
            foc_abc_t hpwm; foc_centered_svpwm(&hv,m->m_phase,&hpwm);
            m->m_pwm_a=hpwm.a;m->m_pwm_b=hpwm.b;m->m_pwm_c=hpwm.c;
            m->m_ccr_a=(uint16_t)CLAMP((int32_t)hpwm.a+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
            m->m_ccr_b=(uint16_t)CLAMP((int32_t)hpwm.b+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
            m->m_ccr_c=(uint16_t)CLAMP((int32_t)hpwm.c+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
            if(foc_prof_detail_sample){
                const uint32_t used=DWT->CYCCNT-profSvpwmStart;
                if(used>foc_prof_fast_hold_svpwm_max_cycles)foc_prof_fast_hold_svpwm_max_cycles=used;
            }
            m->m_state=MC_STATE_RUNNING;m->m_isr_count++;
            return;
        }
    }

    /* LEFT ABI frozen detection hanya aktif pada SPEED mode. Position hold,
     * hard-stop steering detect, dan fixed-phase commissioning sengaja dikecualikan
     * karena kondisi tersebut memang dapat menghasilkan arus tanpa gerakan. */
    if(control_update && !second && encoder_feedback_selected(m,false) && m->m_encoder_configured && m->m_encoder_synced &&
       m->m_control_mode==CONTROL_MODE_SPEED){
        int32_t tr=m->m_speed_target_rpm_q16;
        uint32_t atr=(uint32_t)(tr<0?-tr:tr);
        const uint32_t target_erpm_q16=atr*(uint32_t)motor_pole_pairs(false);
        const int32_t iq=m->m_iq_set_q4<0?-(int32_t)m->m_iq_set_q4:(int32_t)m->m_iq_set_q4;
        const int32_t iq_min=(int32_t)MCCONF_ENCODER_STUCK_CURRENT_MA*(int32_t)FOC_CURRENT_Q4_PER_A/1000;
        if(target_erpm_q16>((uint32_t)MCCONF_ENCODER_STUCK_MIN_ERPM<<16) && iq>=iq_min &&
           m->m_encoder_idle_ticks>=MCCONF_ENCODER_STUCK_TIMEOUT_TICKS){
            m->m_encoder_synced=0u;
            motor_fault_set(m,FAULT_CODE_ENCODER_FAULT);
        }
    }

    if(control_update){
        const bool pll_needed=(m->m_conf.s_pid_speed_source==S_PID_SPEED_SRC_PLL) ||
                              (m->m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_DISABLED);
        if(pll_needed){
            uint32_t profStart=0u;
            if(foc_prof_detail_sample)profStart=DWT->CYCCNT;
            foc_pll_update_fixed(m,second,control_update);
            if(foc_prof_detail_sample){
                const uint32_t used=DWT->CYCCNT-profStart;
                if(used>foc_prof_pll_max_cycles)foc_prof_pll_max_cycles=used;
            }
        }else if(m->m_pll_valid){
            /* Default FAST sensor source does not consume the PLL. Clear stale
             * state once instead of spending ~300 cycles on every regulator slot. */
            m->m_pll_valid=0u; m->m_pll_speed_step_q32=0;
            m->m_pll_erpm_q16=0; m->m_pll_mech_rpm_q16=0;
        }
    }
    const bool source_enabled = (enable != 0u) || mcpwm_foc_vesc_command_live(second);
    const bool openloop_feedback = m->m_control_mode==CONTROL_MODE_OPENLOOP ||
                                   m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE;
    const bool feedback_ready = openloop_feedback ||
        (encoder_feedback_selected(m,second) ? m->m_encoder_synced : hall_feedback_valid(m));
    const bool inactive = !source_enabled || !feedback_ready ||
        m->m_fault!=FAULT_CODE_NONE || m->m_control_mode==CONTROL_MODE_NONE;
    /* Released motors still need one Clarke/Park measurement on their normal
     * regulator slot. i0/i1/idc already come from the separately calibrated
     * high-impedance OFF baseline, so continue below until telemetry has been
     * measured. The inactive branch after the transform keeps PWM/control off. */
    /* On the two non-regulator slots an inactive non-NONE mode has no current
     * transform or control state to update. */
    if (inactive && !control_update) {
        m->m_state=MC_STATE_OFF;
        m->m_isr_count++;
        return;
    }
    if (!control_update && encoder_port && m->m_encoder_synced &&
        m->m_control_mode!=CONTROL_MODE_OPENLOOP &&
        m->m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE) {
        /* Encoder phase, Vd/Vq and therefore PWM vector are intentionally held
         * until this motor's next regulator slot. The advanced timer continues
         * switching at 16 kHz with the last safe CCRs. This removes redundant
         * Park/SVPWM work from five of six steering frames. */
        m->m_state=MC_STATE_RUNNING;
        m->m_isr_count++;
        return;
    }

    if (m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE && !control_update) {
        /* Fixed electrical phase + held Vd/Vq means the two non-regulator PWM
         * slots would reproduce identical SVPWM/CCRs. Keep hard DC protection
         * in DMA1_Channel1, but skip redundant Clarke/Park/PI/SVPWM work here.
         * The regulator slot remains PWM/6 = 2.667 kHz exactly as configured. */
        m->m_state=MC_STATE_RUNNING;
        m->m_isr_count++;
        return;
    }

    /* Control-current offset is calibrated once in the original EFeru
     * startup ADC window. Do not pause/reset a new command to re-learn offset:
     * doing that while a rotor is already moving can fold real BEMF/current into
     * the offset and adds a visible dead time at every OFF->RUN transition. */

    /* ADC/current estimator tetap berjalan walaupun bridge OFF. Pada hardware
     * low-side-shunt ini baseline amplifier berbeda antara driven dan high-Z,
     * sehingga ISR memilih baseline OFF khusus untuk telemetry. Nilai sensor
     * tersebut tetap dipublikasikan saat idle/coast; hanya state kontrol yang
     * dinolkan sebelum keluar agar tidak pernah menghasilkan torsi. */
    /* The generated PI regulators run at PWM/6 = 2.667 kHz. Clarke/Park is
     * therefore only needed on this motor's regulator slot. Between regulator
     * slots Vd/Vq are held while the 16-kHz protection/sensor path remains
     * active. This avoids redundant transforms on five of six PWM frames. */
    m->m_current_in_counts=idc_counts;
    if(control_update){
        uint32_t profCurrentStart=0u;
        if(foc_prof_detail_sample)profCurrentStart=DWT->CYCCNT;
        const int16_t i0_q4=phase_current_counts_to_q4(m,i0_counts);
        const int16_t i1_q4=phase_current_counts_to_q4(m,i1_counts);
        foc_ab_t ab; if(second)foc_clarke_bc_q4(i0_q4,i1_q4,&ab); else foc_clarke_ab_q4(i0_q4,i1_q4,&ab);
        m->m_i_alpha_q4=ab.alpha; m->m_i_beta_q4=ab.beta;
        foc_dq_t raw; foc_park_q4(&ab,m->m_phase,&raw);
        /* Upstream VESC control_current() closes the PI on the instantaneous
         * Park currents. foc_current_filter_const is monitoring-only and must
         * not add phase lag to Id/Iq feedback. Keep raw D/Q authoritative for
         * control and apply the configurable LPF only to telemetry below. */
        m->m_iq_q4=raw.q; m->m_id_q4=raw.d;
        m->m_dq_sample_fresh=1u;
        if(foc_prof_detail_sample){
            const uint32_t used=DWT->CYCCNT-profCurrentStart;
            if(used>foc_prof_current_max_cycles)foc_prof_current_max_cycles=used;
        }
    }

    if (inactive) {
        m->m_state=MC_STATE_OFF;
        reset_current_pi(m); m->m_speed_integrator=0; m->m_speed_prev_error=0;
        m->m_speed_sat_hold=0; reset_position_pid(m);
        m->m_iq_set_q4=0; m->m_iq_target_q4=0; m->m_iq_set_ramp_q16=0;
        m->m_id_set_q4=0; m->m_openloop_id_target_q4=0; m->m_openloop_id_ramp_q16=0;
        /* Keep the just-measured OFF-state alpha/beta, Id/Iq and DC-link
         * current available to telemetry. They are measurement-only because
         * CONTROL_MODE_NONE, MOE=0, all current targets/integrators are zero,
         * and protection explicitly requires a powered valid sample. */
        m->m_dq_sample_fresh=0u;
        m->m_vd=0; m->m_vq=0;
        m->m_pwm_a=0; m->m_pwm_b=0; m->m_pwm_c=0; m->m_duty_now_permille=0;
        m->m_ccr_a=pwm_res/2u; m->m_ccr_b=pwm_res/2u; m->m_ccr_c=pwm_res/2u;
        m->m_isr_count++;
        return;
    }

    /* A newly-enabled advanced-timer bridge changes the low-side current
     * amplifier common-mode operating point. Hold a true zero vector for only
     * a few synchronized ADC frames; do not recalibrate and do not touch the
     * requested current/speed/position setpoints. */
    if (m->m_bridge_settle_ticks != 0u) {
        m->m_state=MC_STATE_RUNNING;
        reset_current_pi(m);
        m->m_vd=0; m->m_vq=0;
        m->m_pwm_a=0; m->m_pwm_b=0; m->m_pwm_c=0; m->m_duty_now_permille=0;
        m->m_ccr_a=pwm_res/2u; m->m_ccr_b=pwm_res/2u; m->m_ccr_c=pwm_res/2u;
        m->m_isr_count++;
        return;
    }

    foc_dq_t v={m->m_vd,m->m_vq};
    m->m_state=MC_STATE_RUNNING;
    uint32_t profRegulatorStart=0u;
    if(foc_prof_detail_sample && control_update)profRegulatorStart=DWT->CYCCNT;
        if (control_update) {
            /* Plant-identification capture for Detect All. m_vd is the voltage
             * held over the interval that produced the current delta below.
             * Accumulate sufficient statistics only; no float/division in ISR. */
            if(m->m_rl_capture_active && m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE){
                const int16_t id_now=m->m_id_q4;
                if(m->m_rl_capture_have_prev){
                    const int32_t di=(int32_t)id_now-(int32_t)m->m_rl_capture_prev_id_q4;
                    const int32_t imid=((int32_t)id_now+(int32_t)m->m_rl_capture_prev_id_q4)/2;
                    const int32_t vv=(int32_t)m->m_vd;
                    /* Cortex-M3 has a fast 32x32 multiply but no native 64x64.
                     * Products are safely bounded by int32 here; widen only the
                     * accumulator so Detect-All cannot blow the 16-kHz deadline. */
                    if(m->m_rl_capture_n<2048u){
                        const int32_t p_di2=di*di;
                        const int32_t p_div=di*vv;
                        const int32_t p_dii=di*imid;
                        m->m_rl_sum_di2+=(int64_t)p_di2;
                        m->m_rl_sum_div+=(int64_t)p_div;
                        m->m_rl_sum_dii+=(int64_t)p_dii;
                        m->m_rl_sum_di+=(int64_t)di;
                        m->m_rl_capture_n++;
                    }
                }else m->m_rl_capture_have_prev=1u;
                m->m_rl_capture_prev_id_q4=id_now;
            }
            /* The proven generated controller updates its regulators once every
             * six 16-kHz ADC frames (2.667 kHz). */
            /* Current/speed/position modes may use the same configured physical
             * full-safe modulation ceiling as VESC duty mode. */
            /* VESC duty is normalized to [-1,1], but the physical hoverboard
             * FOC bridge must retain EFeru's 110-count sampling margin. Map
             * normalized duty to the board-scaled vector from config.h. The
             * separate FOC_SVPWM_VECTOR_FULL_SAFE constant retains EFeru's
             * exact 110..1890 electrical ceiling for reference/testing. */
            const int16_t v_closed=(m->m_voltage_limit_counts>0)?
                m->m_voltage_limit_counts:(int16_t)MCCONF_FOC_DUTY_VOLTAGE_MAX;
            int16_t vector_limit=v_closed;

            m->m_id_set_q4 = (m->m_control_mode==CONTROL_MODE_OPENLOOP ||
                              m->m_control_mode==CONTROL_MODE_OPENLOOP_PHASE) ?
                              m->m_id_set_q4 : 0;
            const int16_t ed=(int16_t)CLAMP((int32_t)m->m_id_set_q4-m->m_id_q4,-32768,32767);
            uint32_t profStageStart=0u;
            if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
            v.d=current_pi_vesc_state(ed,m->m_current_kpd_err_q8,m->m_current_kid_err_q8,
                             v_closed,(int16_t)-v_closed,&m->m_id_integrator,&m->m_id_sat_hold);
            if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_id_pi_max_cycles)foc_prof_id_pi_max_cycles=used;}
            int16_t q_lim=voltage_circle_q_limit(v.d,v_closed);
            if (m->m_control_mode==CONTROL_MODE_DUTY) {
                int32_t dp=m->m_duty_set_permille<0?-(int32_t)m->m_duty_set_permille:(int32_t)m->m_duty_set_permille;
                const int32_t cfgmax=m->m_duty_limit_permille>0?m->m_duty_limit_permille:1000;
                if(dp>cfgmax)dp=cfgmax;
                int32_t duty_v=(dp*MCCONF_FOC_DUTY_VOLTAGE_MAX)/1000;
                if(duty_v>MCCONF_FOC_DUTY_VOLTAGE_MAX)duty_v=MCCONF_FOC_DUTY_VOLTAGE_MAX;
                vector_limit=(int16_t)duty_v;
                q_lim=voltage_circle_q_limit(v.d,vector_limit);
            }
            if (m->m_control_mode==CONTROL_MODE_SPEED) {
                /* Outer SPEED PID berjalan 1 kHz di main context seperti thread
                 * FOC PID VESC. ISR hanya menutup current loop dengan cached Iq. */
                m->m_iq_set_q4=m->m_iq_target_q4;
                m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
                m->m_iq_set_q4=current_circle_iq_limit_q4(m,m->m_iq_set_q4);
                if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_current_circle_max_cycles)foc_prof_current_circle_max_cycles=used;}
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
                v.q=current_pi_vesc_state(eq,m->m_current_kpq_err_q8,m->m_current_kiq_err_q8,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
                if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_iq_pi_max_cycles)foc_prof_iq_pi_max_cycles=used;}
            } else {
                if(m->m_control_mode==CONTROL_MODE_DUTY){
                    m->m_iq_target_q4=duty_control_iq_target_step(m);
                    /* Upstream duty control applies the current limit directly
                     * when duty is below the target, and uses its down-ramp PI
                     * only when measured duty exceeds the target. */
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                } else if(m->m_control_mode==CONTROL_MODE_POS){
                    /* Outer POSITION PID 1 kHz menghasilkan cached Iq target. */
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                } else if (m->m_control_mode==CONTROL_MODE_CURRENT_BRAKE) {
                    /* VESC: iq = -SIGN(speed) * abs(brake_current). At zero
                     * speed the reference is zero and the centered zero vector
                     * keeps the phases shorted while the brake command remains
                     * active. */
                    const int8_t dir=feedback_motion_direction(m,second);
                    m->m_iq_target_q4=dir>0?(int16_t)-m->m_brake_current_q4:
                                          (dir<0?m->m_brake_current_q4:0);
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                } else if (m->m_control_mode==CONTROL_MODE_HANDBRAKE) {
                    m->m_iq_target_q4=m->m_handbrake_current_q4;
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                } else if (m->m_control_mode==CONTROL_MODE_CURRENT ||
                           m->m_control_mode==CONTROL_MODE_OPENLOOP) {
                    /* OPENLOOP_CURRENT uses exactly the signed Iq reference set
                     * by mcpwm_foc_set_openloop_current(), as upstream VESC. */
                    m->m_iq_set_q4=m->m_iq_target_q4;
                    m->m_iq_set_ramp_q16=(int32_t)m->m_iq_set_q4*65536;
                } else {
                    /* OPENLOOP_PHASE and all other non-torque modes use Iq=0. */
                    m->m_iq_target_q4=0;
                    m->m_iq_set_q4=0;
                    m->m_iq_set_ramp_q16=0;
                }
                if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
                m->m_iq_set_q4=current_circle_iq_limit_q4(m,m->m_iq_set_q4);
                if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_current_circle_max_cycles)foc_prof_current_circle_max_cycles=used;}
                const int16_t eq=(int16_t)CLAMP((int32_t)m->m_iq_set_q4-m->m_iq_q4,-32768,32767);
                if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
                v.q=current_pi_vesc_state(eq,m->m_current_kpq_err_q8,m->m_current_kiq_err_q8,
                                 q_lim,(int16_t)-q_lim,
                                 &m->m_iq_integrator,&m->m_iq_sat_hold);
                if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_iq_pi_max_cycles)foc_prof_iq_pi_max_cycles=used;}
            }
            /* Feed-forward D/Q decoupling VESC dijalankan sesudah kedua PI.
             * Default tetap DISABLED; CROSS/BEMF hanya lolos config bila model
             * L/flux valid. Voltage circle di bawah tetap menjadi safety akhir. */
            if(m->m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_DISABLED){
                if(foc_prof_detail_sample)profStageStart=DWT->CYCCNT;
                current_decoupling_apply(m,second,&v,vector_limit);
                /* Decoupling can move Vd/Vq after the PI circle limit, therefore
                 * only this optional mode needs a second final vector clamp. */
                foc_vector_limit(&v,vector_limit);
                if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profStageStart;if(used>foc_prof_decouple_limit_max_cycles)foc_prof_decouple_limit_max_cycles=used;}
            }
            if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profRegulatorStart;if(used>foc_prof_regulator_max_cycles)foc_prof_regulator_max_cycles=used;}
        }
    m->m_vd=v.d; m->m_vq=v.q;
    uint32_t profSvpwmStart=0u;
    if(foc_prof_detail_sample)profSvpwmStart=DWT->CYCCNT;
    foc_abc_t pwm; foc_centered_svpwm(&v,m->m_phase,&pwm);
    if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profSvpwmStart;if(used>foc_prof_svpwm_max_cycles)foc_prof_svpwm_max_cycles=used;}
    /* Keep EFeru's exact hardware clamp: ARR=2000, pwm_margin=110, therefore
     * every CCR remains in 110..1890. Normal VESC commands are additionally
     * scaled in config.h, leaving margin below this absolute electrical clamp. */
    m->m_pwm_a=pwm.a; m->m_pwm_b=pwm.b; m->m_pwm_c=pwm.c;
    /* VESC Tool still uses normalized duty [-1,1]. Report the physical full-safe
     * vector as 1.000, so command 1.0 and telemetry 1.0 both mean EFeru's real
     * PWM ceiling rather than an unsafe mathematical CCR rail. */
    if (control_update && m->m_control_mode==CONTROL_MODE_DUTY) {
        uint32_t profDutyStart=0u; if(foc_prof_detail_sample)profDutyStart=DWT->CYCCNT;
        m->m_duty_now_permille=duty_permille_from_vdq(v.d,v.q);
        if(foc_prof_detail_sample){const uint32_t used=DWT->CYCCNT-profDutyStart;if(used>foc_prof_duty_mag_max_cycles)foc_prof_duty_mag_max_cycles=used;}
    }
    m->m_ccr_a=(uint16_t)CLAMP((int32_t)pwm.a+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_b=(uint16_t)CLAMP((int32_t)pwm.b+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_ccr_c=(uint16_t)CLAMP((int32_t)pwm.c+pwm_res/2,pwm_margin,pwm_res-pwm_margin);
    m->m_isr_count++;
}

static int16_t duty_permille_from_vdq(int16_t vd, int16_t vq) {
    const uint32_t mag2=(uint32_t)((int32_t)vd*vd)+(uint32_t)((int32_t)vq*vq);
    const uint32_t mag=foc_isqrt_u32(mag2);
    int32_t dpm=(int32_t)((mag*1000u+(MCCONF_FOC_DUTY_VOLTAGE_MAX/2u))/MCCONF_FOC_DUTY_VOLTAGE_MAX);
    if(dpm>1000)dpm=1000;
    if(vq<0)dpm=-dpm;
    return (int16_t)dpm;
}


static void motor_telemetry_non_isr(mcpwm_foc_motor_t *m, bool second, uint32_t virtual_steps) {
    if(!m || virtual_steps==0u)return;
    int16_t td=m->m_id_q4, tq=m->m_iq_q4, ti=m->m_current_in_counts;
    const bool bridge_on=second ? ((RIGHT_TIM->BDTR&TIM_BDTR_MOE)!=0u) : ((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u);
    /* Low-side shunt samples are not physically meaningful during OFF->RUN.
     * Do not expose the pre-settle common-mode transient to VESC Tool/ROS as
     * a fictitious multi-amp current spike. Control protection still runs in
     * the hard realtime path; this only qualifies monitoring telemetry. */
    if(m->m_control_mode!=CONTROL_MODE_NONE &&
       (!bridge_on || m->m_driven_offset_calibrating || !m->m_driven_offset_valid || m->m_bridge_settle_ticks!=0u)){
        td=0; tq=0; ti=0;
    }else if(m->m_control_mode==CONTROL_MODE_NONE){
        /* Bridge-OFF current is a real ADC measurement using m_off_offset*.
         * Publish it only after the OFF baseline has settled; otherwise return
         * zero during calibration. This telemetry path never feeds protection. */
        if(!m->m_off_offset_valid || bridge_on){td=0; tq=0; ti=0;}
    }
    const uint16_t a=m->m_telem_current_filter_q16?m->m_telem_current_filter_q16:6553u;
    /* Jalankan virtual sample sebanyak cadence regulator yang berlalu agar
     * time-constant konfigurasi foc_current_filter_const tetap setara. */
    while(virtual_steps--){
        m->m_id_telem_q4=telemetry_lpf_step(&m->m_telem_current_lpf_q16[0],a,td);
        m->m_iq_telem_q4=telemetry_lpf_step(&m->m_telem_current_lpf_q16[1],a,tq);
        m->m_current_in_telem_counts=telemetry_lpf_step(&m->m_telem_current_lpf_q16[2],a,ti);
        telemetry_avg_push(m,m->m_id_telem_q4,m->m_iq_telem_q4,m->m_current_in_telem_counts);
    }
}

static int32_t relay_speed_erpm_now(const mcpwm_foc_motor_t *m, bool second) {
    int64_t e=(int64_t)measured_mech_rpm_q16(m,second)*(int64_t)motor_pole_pairs(second);
    e>>=16;
    if(e>INT32_MAX)e=INT32_MAX;
    if(e<INT32_MIN)e=INT32_MIN;
    return (int32_t)e;
}

static int32_t relay_steering_mdeg_now(const mcpwm_foc_motor_t *m) {
    const int32_t span=steering_runtime_span_from_measured(m->m_steering_span_counts);
    if(span==0)return 0;
    int64_t x=(int64_t)m->m_position_counts*60000LL;
    x/=span;
    if(x>30000)x=30000;
    if(x<-30000)x=-30000;
    return (int32_t)x;
}

static int32_t relay_steering_iq_sign(const mcpwm_foc_motor_t *m) {
    int32_t sign=steering_safe_span_from_measured(m->m_steering_span_counts)<0?-1:1;
    if(position_error_sign(m,false)<0)sign=-sign;
    if(m->m_conf.m_invert_direction)sign=-sign;
    return sign;
}

static bool relay_hall_position_active(const mcpwm_foc_motor_t *m) {
    return m && m->m_conf.m_sensor_port_mode==SENSOR_PORT_MODE_HALL &&
           m->m_conf.foc_sensor_mode==FOC_SENSOR_MODE_HALL;
}

static int32_t relay_hall_position_mdeg_now(const mcpwm_foc_motor_t *m, bool second) {
    const uint16_t ph=position_feedback_phase_u16(m,second);
    return (int32_t)(((uint32_t)ph*360000u+32768u)>>16);
}

static int32_t relay_angle_delta_mdeg(int32_t value,int32_t target) {
    int32_t d=value-target;
    while(d>180000)d-=360000;
    while(d<=-180000)d+=360000;
    return d;
}

static void relay_finish(uint8_t failed) {
    if(!s_foc_relay.active)return;
    const bool second=s_foc_relay.second!=0u;
    s_foc_relay.active=0u;
    s_foc_relay.done=1u;
    s_foc_relay.failed=failed;
    mcpwm_foc_release_motor(second);
    mcpwm_foc_vesc_override_clear(second);
}

static void relay_apply_current(mcpwm_foc_motor_t *m, bool second) {
    int32_t q=s_foc_relay.relay_positive?(int32_t)s_foc_relay.relay_q4:-(int32_t)s_foc_relay.relay_q4;
    if(s_foc_relay.mode==MCPWM_FOC_RELAY_POSITION && !relay_hall_position_active(m))q*=relay_steering_iq_sign(m);
    q=CLAMP(q,-MCCONF_MOTOR_CURRENT_MAX_Q4,MCCONF_MOTOR_CURRENT_MAX_Q4);
    m->m_iq_target_q4=(int16_t)q;
    m->m_iq_set_q4=(int16_t)q;
    m->m_iq_set_ramp_q16=q*65536;
    m->m_id_set_q4=0;
    mcpwm_foc_vesc_override_touch(second);
}

static void relay_crossing(uint32_t now_ms, bool upper) {
    const uint32_t last=upper?s_foc_relay.last_upper_ms:s_foc_relay.last_lower_ms;
    s_foc_relay.crossings++;
    if(last!=0u && s_foc_relay.crossings>=5u){
        const uint32_t period=now_ms-last;
        if(period>=20u && period<=5000u && s_foc_relay.period_count<UINT16_MAX){
            s_foc_relay.period_sum_ms+=period;
            s_foc_relay.period_count++;
        }
    }
    if(upper)s_foc_relay.last_upper_ms=now_ms;else s_foc_relay.last_lower_ms=now_ms;
    if(s_foc_relay.crossings==2u){
        s_foc_relay.minimum=s_foc_relay.measurement;
        s_foc_relay.maximum=s_foc_relay.measurement;
    }
}

static void relay_process(uint32_t now_ms) {
    if(!s_foc_relay.active)return;
    const bool second=s_foc_relay.second!=0u;
    mcpwm_foc_motor_t *m=second?&m_motor_2:&m_motor_1;
    if(m->m_fault!=FAULT_CODE_NONE){relay_finish(1u);return;}
    const uint32_t elapsed=now_ms-s_foc_relay.start_ms;
    if(elapsed>=s_foc_relay.timeout_ms){relay_finish(2u);return;}

    int32_t y;
    if(s_foc_relay.mode==MCPWM_FOC_RELAY_SPEED){
        y=relay_speed_erpm_now(m,second);
        int64_t guard=(int64_t)(s_foc_relay.target<0?-s_foc_relay.target:s_foc_relay.target)+2000LL;
        const int64_t h4=(int64_t)s_foc_relay.hysteresis*4LL;
        if(h4>guard)guard=h4;
        if(guard>(int64_t)MCCONF_L_MAX_ERPM)guard=(int64_t)MCCONF_L_MAX_ERPM;
        if((int64_t)y>guard || (int64_t)y<-guard){relay_finish(3u);return;}
    }else if(relay_hall_position_active(m)){
        const int32_t raw=relay_hall_position_mdeg_now(m,second);
        const int32_t d=relay_angle_delta_mdeg(raw,s_foc_relay.target);
        y=s_foc_relay.target+d; /* continuous local coordinate around captured center */
        int32_t ad=d<0?-d:d;
        int32_t guard=s_foc_relay.hysteresis*6;if(guard<30000)guard=30000;
        if(guard>120000)guard=120000;
        if(ad>guard){relay_finish(4u);return;}
    }else{
        y=relay_steering_mdeg_now(m);
        int32_t d=y-s_foc_relay.target;if(d<0)d=-d;
        int32_t guard=s_foc_relay.hysteresis*6;if(guard<10000)guard=10000;
        if(d>guard || y<-30000 || y>30000){relay_finish(4u);return;}
    }
    s_foc_relay.measurement=y;
    if(s_foc_relay.crossings>=2u){
        if(y<s_foc_relay.minimum)s_foc_relay.minimum=y;
        if(y>s_foc_relay.maximum)s_foc_relay.maximum=y;
    }

    if(s_foc_relay.relay_positive){
        if(y>=s_foc_relay.target+s_foc_relay.hysteresis){
            s_foc_relay.relay_positive=0u;
            relay_crossing(now_ms,true);
        }
    }else if(y<=s_foc_relay.target-s_foc_relay.hysteresis){
        s_foc_relay.relay_positive=1u;
        relay_crossing(now_ms,false);
    }
    if(s_foc_relay.crossings>=s_foc_relay.required_crossings && s_foc_relay.period_count>=4u){
        relay_finish(0u);return;
    }
    relay_apply_current(m,second);
}

bool mcpwm_foc_relay_start(mcpwm_foc_relay_mode_t mode,bool second,int32_t target,
                           int32_t hysteresis,uint16_t relay_current_ma,uint8_t required_crossings,
                           uint32_t timeout_ms){
    if(s_foc_relay.active || (mode!=MCPWM_FOC_RELAY_SPEED && mode!=MCPWM_FOC_RELAY_POSITION))return false;
    if(hysteresis<=0 || relay_current_ma<100u || relay_current_ma>3000u ||
       required_crossings<6u || required_crossings>14u || timeout_ms<2000u || timeout_ms>20000u)return false;
    mcpwm_foc_motor_t *m=second?&m_motor_2:&m_motor_1;
    if(m->m_fault!=FAULT_CODE_NONE)return false;
    if(mode==MCPWM_FOC_RELAY_POSITION){
        if(relay_hall_position_active(m)){
            if(!hall_feedback_valid(m) || hysteresis>30000)return false;
            /* Hall motors are free-rotating. Tune position around the current
             * single-turn electrical angle; no steering span/home is relevant. */
            target=relay_hall_position_mdeg_now(m,second);
        }else{
            if(second || !m->m_steering_calibrated || !m->m_steering_homed || !m->m_encoder_synced)return false;
            if(target<-20000 || target>20000 || hysteresis>5000)return false;
        }
    }else if(target<=hysteresis || target>(int32_t)MCCONF_L_MAX_ERPM-1000 || hysteresis>2000)return false;

    const uint32_t next_sequence=s_foc_relay.sequence+1u;
    memset(&s_foc_relay,0,sizeof(s_foc_relay));
    s_foc_relay.sequence=next_sequence;
    s_foc_relay.mode=(uint8_t)mode;s_foc_relay.second=second?1u:0u;
    s_foc_relay.target=target;s_foc_relay.hysteresis=hysteresis;
    s_foc_relay.relay_current_ma=relay_current_ma;
    s_foc_relay.relay_q4=amp_to_q4(m,(float)relay_current_ma*0.001f);
    if(s_foc_relay.relay_q4<0)s_foc_relay.relay_q4=(int16_t)-s_foc_relay.relay_q4;
    s_foc_relay.required_crossings=required_crossings;s_foc_relay.timeout_ms=timeout_ms;
    s_foc_relay.start_ms=HAL_GetTick();s_foc_relay.relay_positive=1u;
    s_foc_relay.measurement=(mode==MCPWM_FOC_RELAY_SPEED)?relay_speed_erpm_now(m,second):
        (relay_hall_position_active(m)?relay_hall_position_mdeg_now(m,second):relay_steering_mdeg_now(m));
    s_foc_relay.minimum=s_foc_relay.measurement;s_foc_relay.maximum=s_foc_relay.measurement;
    set_control_mode(m,CONTROL_MODE_CURRENT);
    relay_apply_current(m,second);
    s_foc_relay.active=1u;
    return true;
}

void mcpwm_foc_relay_abort(void){
    if(s_foc_relay.active)relay_finish(9u);
}

void mcpwm_foc_relay_get(mcpwm_foc_relay_status_t *out){
    if(!out)return;
    FOC_MEMORY_BARRIER();
    out->sequence=s_foc_relay.sequence;
    out->elapsed_ms=HAL_GetTick()-s_foc_relay.start_ms;
    out->period_sum_ms=s_foc_relay.period_sum_ms;
    out->target=s_foc_relay.target;out->hysteresis=s_foc_relay.hysteresis;
    out->measurement=s_foc_relay.measurement;out->minimum=s_foc_relay.minimum;out->maximum=s_foc_relay.maximum;
    out->relay_current_ma=s_foc_relay.relay_current_ma;out->period_count=s_foc_relay.period_count;
    out->active=s_foc_relay.active;out->done=s_foc_relay.done;out->failed=s_foc_relay.failed;
    out->mode=s_foc_relay.mode;out->second=s_foc_relay.second;out->relay_positive=s_foc_relay.relay_positive;
    out->crossings=s_foc_relay.crossings;out->required_crossings=s_foc_relay.required_crossings;
    FOC_MEMORY_BARRIER();
}

void mcpwm_foc_outer_control_non_isr(uint32_t now_ms) {
    const uint32_t cycle_start=DWT->CYCCNT;
    /* Finalisasi powered current-zero sesegera mungkin setelah jendela 80 ADC
     * sample selesai. Ini tetap non-ISR, tetapi tidak lagi menunggu gate
     * housekeeping 200 Hz sehingga OFF->RUN tidak mendapat dead-time ekstra
     * sampai 5 ms hanya karena finalisasi tiga pembagian integer. */
    driven_offset_finalize_non_isr();
    if(s_outer_pid_last_ms==0u){
        s_outer_pid_last_ms=now_ms;
        s_outer_pid_last_cycle=cycle_start;
        return;
    }
    uint32_t dt_ms=now_ms-s_outer_pid_last_ms;
    if(dt_ms==0u)return;
    const uint32_t period_cycles=cycle_start-s_outer_pid_last_cycle;
    s_outer_pid_last_cycle=cycle_start;
    if(period_cycles<outer_control_period_min_cycles)outer_control_period_min_cycles=period_cycles;
    if(period_cycles>outer_control_period_max_cycles)outer_control_period_max_cycles=period_cycles;
    const uint64_t expected64=(uint64_t)OUTER_PID_PERIOD_CYCLES*(uint64_t)dt_ms;
    const uint32_t expected=expected64>UINT32_MAX?UINT32_MAX:(uint32_t)expected64;
    const uint32_t jitter=(period_cycles>expected)?(period_cycles-expected):(expected-period_cycles);
    if(jitter>outer_control_jitter_max_cycles)outer_control_jitter_max_cycles=jitter;
    if(dt_ms>1u)outer_control_miss_count += dt_ms-1u;
    s_outer_pid_last_ms=now_ms;
    /* Normalnya tepat 1 ms. Gap panjang menandakan main sempat diblokir; jangan
     * melakukan stale catch-up. Satu evaluasi memakai dt aktual yang dibatasi
     * agar integral/derivative tidak memberi impulse berbahaya setelah stall. */
    if(dt_ms>3u)dt_ms=3u;

    /* Snapshot feedback lebih dulu sehingga kedua outer loop memakai data yang
     * konsisten dari tick 1-kHz yang sama. Semua konversi ini di luar ADC ISR. */
    encoder_feedback_finalize_speed_non_isr(&m_motor_1,false);
    if(encoder_feedback_selected(&m_motor_1,false) && m_motor_1.m_encoder_configured)
        position_feedback_update(&m_motor_1,m_motor_1.m_encoder_mech_phase);
    else if(m_motor_1.m_control_mode==CONTROL_MODE_POS ||
            m_motor_1.m_pos_pid_ang_div_inv_q16<64251u || m_motor_1.m_pos_pid_ang_div_inv_q16>66873u)
        position_feedback_update(&m_motor_1,m_motor_1.m_phase);
    if(m_motor_2.m_control_mode==CONTROL_MODE_POS ||
       m_motor_2.m_pos_pid_ang_div_inv_q16<64251u || m_motor_2.m_pos_pid_ang_div_inv_q16>66873u)
        position_feedback_update(&m_motor_2,m_motor_2.m_phase);

    /* Stage-2 relay identification is clocked by this same fresh-feedback 1-kHz
     * scheduler. UART only starts/observes the test; it never defines switching
     * instants, so Ku/Pu are not contaminated by serial latency. */
    relay_process(now_ms);

    mcpwm_foc_motor_t *motors[2]={&m_motor_1,&m_motor_2};
    for(uint8_t i=0u;i<2u;++i){
        mcpwm_foc_motor_t *m=motors[i];
        const bool second=i!=0u;
        if(m->m_fault!=FAULT_CODE_NONE)continue;
        if(m->m_control_mode==CONTROL_MODE_SPEED){
            const uint32_t profSpeedStart=DWT->CYCCNT;
            speed_setpoint_slew_step(m,dt_ms);
            const int32_t pp=(int32_t)motor_pole_pairs(second);
            const int64_t set_erpm_q16=(int64_t)m->m_speed_set_ramp_q16*pp;
            const int64_t abs_set=set_erpm_q16<0?-set_erpm_q16:set_erpm_q16;
            const int64_t min_set=(int64_t)m->m_speed_release_erpm_q16;
            if(m->m_speed_target_rpm_q16==0 && abs_set<min_set){
                m->m_speed_integrator=0; m->m_speed_prev_error=0;
                m->m_speed_sat_hold=0; m->m_speed_d_filter_q4=0;
                m->m_iq_target_q4=0;
            }else{
                m->m_iq_target_q4=speed_pid_iq_target_step(m,second,dt_ms);
            }
            const uint32_t profSpeedUsed=DWT->CYCCNT-profSpeedStart;
            if(profSpeedUsed>foc_prof_speed_pid_max_cycles)foc_prof_speed_pid_max_cycles=profSpeedUsed;
        }else if(m->m_control_mode==CONTROL_MODE_POS){
            const uint32_t profPosStart=DWT->CYCCNT;
            m->m_iq_target_q4=position_pid_iq_target_step(m,second,dt_ms);
            const uint32_t profPosUsed=DWT->CYCCNT-profPosStart;
            if(profPosUsed>foc_prof_position_pid_max_cycles)foc_prof_position_pid_max_cycles=profPosUsed;
        }
    }
    const uint32_t used=DWT->CYCCNT-cycle_start;
    if(used>outer_control_max_cycles)outer_control_max_cycles=used;
}

void mcpwm_foc_housekeeping_non_isr(uint32_t now_ms) {
    /* Semua pekerjaan berbasis waktu manusia/protokol dikeluarkan dari ADC ISR.
     * Main memanggil fungsi ini sekitar 200 Hz. Counter lama tetap memakai satuan
     * PWM-tick agar EEPROM/wire semantics tidak berubah; elapsed_ms dikonversi
     * menjadi jumlah tick ekuivalen dan dikurangi atomik 32-bit. */
    /* Outer loop integrates only time that has actually elapsed. Legacy source
     * commands are applied AFTER the previous interval is serviced, so a newly
     * received RPM/POS request never gets 5 ms worth of virtual PID steps before
     * its first real control interval. */
    if(s_housekeeping_last_ms==0u){
        s_housekeeping_last_ms=now_ms;
        if(!s_vesc_owned[0])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwml,motorRunReq!=0u,svpwmOpenloopRpm,false);
        if(!s_vesc_owned[1])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwmr,motorRunReq!=0u,svpwmOpenloopRpm,true);
        return;
    }
    uint32_t elapsed=now_ms-s_housekeeping_last_ms;
    if(elapsed<TELEMETRY_PERIOD_MS){
        if(!s_vesc_owned[0])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwml,motorRunReq!=0u,svpwmOpenloopRpm,false);
        if(!s_vesc_owned[1])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwmr,motorRunReq!=0u,svpwmOpenloopRpm,true);
        return;
    }
    if(elapsed>100u)elapsed=100u; /* jangan membuat satu stall main menghapus timeout panjang sekaligus */
    s_housekeeping_last_ms=now_ms;
    uint32_t pwm_ticks=(elapsed*(uint32_t)PWM_FREQ+999u)/1000u;
    if(pwm_ticks==0u)pwm_ticks=1u;

    /* Finalisasi powered baseline normal. Worker commissioning blocking juga
     * memanggil service yang sama setiap 1 ms melalui foc_bounded_delay_ms(). */
    driven_offset_finalize_non_isr();

    /* Telemetry LPF mempertahankan time-constant ekuivalen regulator DIV=6.
     * POSITION/SPEED control tetap pada scheduler outer 1 kHz terpisah. */
    uint32_t outer_acc=s_outer_tick_remainder+pwm_ticks;
    const uint32_t outer_steps=outer_acc/(uint32_t)MCCONF_FOC_CONTROL_DIV;
    s_outer_tick_remainder=outer_acc%(uint32_t)MCCONF_FOC_CONTROL_DIV;

    /* Position/speed feedback sudah disampling scheduler outer 1 kHz.
     * Housekeeping hanya menangani VESC tachometer/telemetry lambat. */
    encoder_tachometer_update_non_isr(&m_motor_1);

    motor_telemetry_non_isr(&m_motor_1,false,outer_steps);
    motor_telemetry_non_isr(&m_motor_2,true,outer_steps);

    /* Fault recovery, VESC command timeout, dan E-stop duration adalah timer
     * millisecond; tidak ada alasan menguranginya 16.000 kali/detik. Flag E-stop
     * sendiri tetap dibaca ADC ISR setiap frame sehingga aktivasi tetap instan. */
    mcpwm_foc_motor_t *motors[2]={&m_motor_1,&m_motor_2};
    for(uint8_t i=0u;i<2u;++i){
        mcpwm_foc_motor_t *m=motors[i];
        if(m->m_fault!=FAULT_CODE_NONE){
            uint32_t t=m->m_fault_recovery_ticks;
            if(t>0u){
                if(t<=pwm_ticks)t=0u; else t-=pwm_ticks;
                m->m_fault_recovery_ticks=t;
                m->m_fault_safe_ticks=0u;
            }else if(fault_recovery_conditions_safe(m,i!=0u)){
                const uint32_t dwell=((uint32_t)MCCONF_FAULT_RECOVERY_SAFE_DWELL_MS*(uint32_t)PWM_FREQ+999u)/1000u;
                uint32_t q=m->m_fault_safe_ticks+pwm_ticks;
                if(q<m->m_fault_safe_ticks || q>dwell)q=dwell;
                m->m_fault_safe_ticks=q;
                if(q>=dwell){
                    /* Keep bridge OFF while clearing all command/integrator
                     * state. A fresh post-recovery host command is required to
                     * arm again; clearing a latch itself never produces torque. */
                    mcpwm_foc_release_motor(i!=0u);
                    m->m_wrong_voltage_integrator=0u;
                    m->m_overspeed_streak=0u;
                    m->m_phase_overcurrent_streak=0u;
                    FOC_MEMORY_BARRIER();
                    m->m_fault=FAULT_CODE_NONE;
                    m->m_fault_safe_ticks=0u;
                    m->m_state=MC_STATE_OFF;
                }
            }else{
                m->m_fault_safe_ticks=0u;
            }
        }else{
            m->m_fault_safe_ticks=0u;
        }
        if(s_vesc_owned[i] && s_vesc_timeout_expired[i]){
            /* Deadline itself is enforced by the 16-kHz ADC ISR so a stuck main
             * loop cannot leave stale torque applied. Main only performs the
             * heavier state transition after the hardware has already coasted. */
            s_vesc_timeout_expired[i]=0u;
            const bool second=(i!=0u);
            const int16_t brake_q4=s_vesc_timeout_brake_q4[i];
            if(brake_q4>0){mcpwm_foc_set_brake_current_q4(brake_q4,second);s_vesc_timeout_braking[i]=1u;}
            else {mcpwm_foc_release_motor(second);s_vesc_timeout_braking[i]=0u;}
        }
    }
    if(s_estop_ticks!=0u){
        uint32_t t=s_estop_ticks;
        s_estop_ticks=(t<=pwm_ticks)?0u:(t-pwm_ticks);
    }

    /* Battery ADC dan board temperature berubah jauh lebih lambat dari PWM.
     * BatVoltage tetap cache yang dibaca proteksi/config lainnya. */
    filtLowPass32(adc_buffer.batt1,BAT_FILT_COEF,&batVoltageFixdt);
    batVoltage=(int16_t)(batVoltageFixdt>>16);

    /* Health checks lambat. Skala integrator memakai jumlah PWM frame ekuivalen
     * sehingga waktu qualification tetap setara implementasi lama 16 kHz. */
    if(offsetcount>=2000u){
        for(uint8_t i=0u;i<2u;++i){
            mcpwm_foc_motor_t *fm=motors[i];
            int32_t vdiff=0;
            if(batVoltage<(int32_t)fm->m_vin_min_adc)vdiff=(int32_t)fm->m_vin_min_adc-batVoltage;
            else if(batVoltage>(int32_t)fm->m_vin_max_adc)vdiff=batVoltage-(int32_t)fm->m_vin_max_adc;
            if(vdiff>0){
                uint64_t ni=(uint64_t)fm->m_wrong_voltage_integrator+(uint64_t)(uint32_t)vdiff*pwm_ticks;
                if(ni>UINT32_MAX)ni=UINT32_MAX;
                fm->m_wrong_voltage_integrator=(uint32_t)ni;
                const uint32_t threshold=((uint32_t)fm->m_vin_max_adc*5u)/100u;
                if(fm->m_fault==FAULT_CODE_NONE && fm->m_wrong_voltage_integrator>threshold){
                    motor_fault_set(fm,batVoltage<(int32_t)fm->m_vin_min_adc?FAULT_CODE_UNDER_VOLTAGE:FAULT_CODE_OVER_VOLTAGE);
                    fm->m_wrong_voltage_integrator=threshold*2u;
                }
            }else if(fm->m_wrong_voltage_integrator>0u){
                fm->m_wrong_voltage_integrator=(fm->m_wrong_voltage_integrator>pwm_ticks)?fm->m_wrong_voltage_integrator-pwm_ticks:0u;
            }
            const uint32_t ae=motor_abs_erpm_for_fault(fm,i!=0u);
            if(fm->m_abs_erpm_fault>0u && ae>fm->m_abs_erpm_fault){
                uint32_t q=(uint32_t)fm->m_overspeed_streak+pwm_ticks;
                if(q>MCCONF_ABS_OVERSPEED_QUAL_SAMPLES)q=MCCONF_ABS_OVERSPEED_QUAL_SAMPLES;
                fm->m_overspeed_streak=(uint8_t)q;
                if(fm->m_fault==FAULT_CODE_NONE && fm->m_overspeed_streak>=MCCONF_ABS_OVERSPEED_QUAL_SAMPLES)
                    motor_fault_set(fm,FAULT_CODE_ABS_OVERSPEED);
            }else fm->m_overspeed_streak=0u;
            /* Offset validity is meaningful only after the one-time 2000-frame
             * bridge-OFF calibration. During an intentional powered zero-vector
             * relearn, m_current_offset_valid is temporarily false by design. */
            if(offsetcount>=2000u && !fm->m_driven_offset_calibrating &&
               !fm->m_current_offset_valid && fm->m_fault==FAULT_CODE_NONE)
                motor_fault_set(fm,FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1);
            if(fm->m_fault==FAULT_CODE_NONE && fm->m_temp_fet_end_x10>fm->m_temp_fet_start_x10 &&
               s_board_temperature_x10>=fm->m_temp_fet_end_x10)
                motor_fault_set(fm,FAULT_CODE_OVER_TEMP_FET);
        }
    }

    const float dt=(float)elapsed*0.001f;
    foc_observer_update_diag(&m_motor_1,dt);
    foc_observer_update_diag(&m_motor_2,dt);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_DUTY)
        m_motor_1.m_duty_now_permille=duty_permille_from_vdq(m_motor_1.m_vd,m_motor_1.m_vq);
    if(m_motor_2.m_control_mode!=CONTROL_MODE_DUTY)
        m_motor_2.m_duty_now_permille=duty_permille_from_vdq(m_motor_2.m_vd,m_motor_2.m_vq);

    /* Apply legacy source for the NEXT interval. VESC binary setters are already
     * applied asynchronously by the protocol path and are never duplicated here. */
    if(!s_vesc_owned[0])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwml,motorRunReq!=0u,svpwmOpenloopRpm,false);
    if(!s_vesc_owned[1])mcpwm_foc_set_mode_command(ctrlModReq,(int16_t)pwmr,motorRunReq!=0u,svpwmOpenloopRpm,true);
}

void mcpwm_foc_adc_int_handler(void) {
    /* Hot path hanya menjadwalkan regulator. Timeout/fault recovery/source
     * arbitration sudah dilayani housekeeping non-ISR. */
    const uint8_t control_slot=s_foc_control_div;
    if(++s_foc_control_div>=MCCONF_FOC_CONTROL_DIV)s_foc_control_div=0u;
    const bool update_left=(control_slot==0u);
    const bool update_right=(MCCONF_FOC_CONTROL_DIV<=1u)?update_left:(control_slot==1u);
    if(update_left)foc_sample_window_update(&m_motor_1,false);
    if(update_right)foc_sample_window_update(&m_motor_2,true);

    const bool step_slot=s_foc_step_test.active &&
        ((s_foc_step_test.second==0u && update_left)||(s_foc_step_test.second!=0u && update_right));
    bool step_capture=false,step_failed=false;
    if(step_slot){
        mcpwm_foc_motor_t *sm=s_foc_step_test.second?&m_motor_2:&m_motor_1;
        const mc_control_mode step_mode=(s_foc_step_test.axis==MCPWM_FOC_STEP_AXIS_D)?
            CONTROL_MODE_OPENLOOP_PHASE:CONTROL_MODE_CURRENT;
        if(sm->m_fault!=FAULT_CODE_NONE || sm->m_control_mode!=step_mode){
            step_failed=true;
        }else if(s_foc_step_test.settling){
            const bool ready=sm->m_current_offset_valid && sm->m_driven_offset_valid &&
                !sm->m_driven_offset_calibrating && sm->m_bridge_settle_ticks==0u &&
                sm->m_sample_window_valid;
            if(ready){
                /* Start the authoritative trace only after powered-offset settle. */
                foc_trace_clear_isr_owned();
                s_foc_step_test.settling=0u;
            }else if(s_foc_step_test.settle_remaining>0u){
                s_foc_step_test.settle_remaining--;
                if(s_foc_step_test.settle_remaining==0u)step_failed=true;
            }else step_failed=true;
        }
        if(step_failed){
            s_foc_step_test.active=0u;s_foc_step_test.done=1u;s_foc_step_test.settling=0u;
            s_foc_trace_event_latch|=1u<<6;step_capture=true;
            mcpwm_foc_release_motor(s_foc_step_test.second!=0u);
            mcpwm_foc_vesc_override_clear(s_foc_step_test.second!=0u);
        }else if(!s_foc_step_test.settling){
            step_capture=true;s_foc_trace_event_latch|=1u<<2;
            if(s_foc_step_test.pre_remaining>0u){
                s_foc_step_test.pre_remaining--;
            }else if(!s_foc_step_test.step_fired){
                if(s_foc_step_test.axis==MCPWM_FOC_STEP_AXIS_D){
                    sm->m_openloop_id_target_q4=s_foc_step_test.step_q4;
                    sm->m_openloop_id_ramp_q16=(int32_t)s_foc_step_test.step_q4*65536;
                    sm->m_id_set_q4=s_foc_step_test.step_q4;
                    sm->m_iq_target_q4=0;sm->m_iq_set_q4=0;sm->m_iq_set_ramp_q16=0;
                }else{
                    sm->m_iq_target_q4=s_foc_step_test.step_q4;sm->m_iq_set_q4=s_foc_step_test.step_q4;
                    sm->m_iq_set_ramp_q16=(int32_t)s_foc_step_test.step_q4*65536;
                }
                s_foc_step_test.step_fired=1u;s_foc_trace_event_latch|=1u<<3;
            }
        }
    }
    uint32_t profMotorStart=0u;
    if(foc_prof_detail_sample)profMotorStart=DWT->CYCCNT;
    motor_control_step(&m_motor_1,false,curL_phaA,curL_phaB,curL_DC,update_left);
    if(foc_prof_detail_sample){
        const uint32_t used=DWT->CYCCNT-profMotorStart;
        if(used>foc_prof_motor_step_max_cycles[0])foc_prof_motor_step_max_cycles[0]=used;
        volatile uint32_t *const dst=update_left?&foc_prof_motor_control_max_cycles[0]:&foc_prof_motor_hold_max_cycles[0];
        if(used>*dst)*dst=used;
    }
    if(update_left)foc_motor_heartbeat[0]++;
    if(foc_prof_detail_sample)profMotorStart=DWT->CYCCNT;
    motor_control_step(&m_motor_2,true,curR_phaB,curR_phaC,curR_DC,update_right);
    if(foc_prof_detail_sample){
        const uint32_t used=DWT->CYCCNT-profMotorStart;
        if(used>foc_prof_motor_step_max_cycles[1])foc_prof_motor_step_max_cycles[1]=used;
        volatile uint32_t *const dst=update_right?&foc_prof_motor_control_max_cycles[1]:&foc_prof_motor_hold_max_cycles[1];
        if(used>*dst)*dst=used;
    }
    if(update_right)foc_motor_heartbeat[1]++;
    /* During a synchronized step, capture only after powered-offset settling;
     * otherwise retain the historical RIGHT-slot cadence (PWM/CONTROL_DIV). */
    if(step_capture || (!s_foc_step_test.active && !step_failed && update_right))foc_trace_capture_internal(control_slot);
    if(step_failed)s_foc_trace_frozen=1u;
    if(step_capture && s_foc_step_test.step_fired && s_foc_step_test.active){
        if(s_foc_step_test.post_remaining>0u)s_foc_step_test.post_remaining--;
        if(s_foc_step_test.post_remaining==0u){
            s_foc_step_test.active=0u;s_foc_step_test.done=1u;s_foc_trace_frozen=1u;
            mcpwm_foc_release_motor(s_foc_step_test.second!=0u);
            mcpwm_foc_vesc_override_clear(s_foc_step_test.second!=0u);
        }
    }
    foc_iqL_q4=m_motor_1.m_iq_q4;foc_idL_q4=m_motor_1.m_id_q4;
    foc_iqR_q4=m_motor_2.m_iq_q4;foc_idR_q4=m_motor_2.m_id_q4;
    LEFT_TIM->LEFT_TIM_U=m_motor_1.m_ccr_a;LEFT_TIM->LEFT_TIM_V=m_motor_1.m_ccr_b;LEFT_TIM->LEFT_TIM_W=m_motor_1.m_ccr_c;
    RIGHT_TIM->RIGHT_TIM_U=m_motor_2.m_ccr_a;RIGHT_TIM->RIGHT_TIM_V=m_motor_2.m_ccr_b;RIGHT_TIM->RIGHT_TIM_W=m_motor_2.m_ccr_c;
}

void mcpwm_foc_set_board_temperature_x10(int16_t temperature_x10) {
    s_board_temperature_x10=temperature_x10;
}

void mcpwm_foc_vesc_timeout_configure(bool second, uint32_t timeout_ms, float brake_current) {
    const uint8_t i=second?1u:0u;
    if (brake_current < 0.0f) brake_current=-brake_current;
    if (brake_current > (float)I_MOT_MAX) brake_current=(float)I_MOT_MAX;
    s_vesc_timeout_ms[i]=timeout_ms;
    {
        mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
        int16_t q=amp_to_q4(m,brake_current); if(q<0)q=(int16_t)-q;
        s_vesc_timeout_brake_q4[i]=q;
    }
    if (s_vesc_owned[i]) mcpwm_foc_vesc_override_touch(second);
}

void mcpwm_foc_vesc_override_touch(bool second) {
    const uint8_t i=second?1u:0u;
    const uint32_t ms=s_vesc_timeout_ms[i];
    s_vesc_owned[i]=1u;
    s_vesc_timeout_braking[i]=0u;
    s_vesc_timeout_expired[i]=0u;
    if (ms == 0u) {
        /* VESC App Config timeout_msec=0 explicitly disables the timeout. */
        s_vesc_timeout_ticks[i]=UINT32_MAX;
    } else {
        uint64_t ticks=((uint64_t)PWM_FREQ*(uint64_t)ms+999u)/1000u;
        if (ticks == 0u) ticks=1u;
        if (ticks >= (uint64_t)UINT32_MAX) ticks=(uint64_t)UINT32_MAX-1u;
        s_vesc_timeout_ticks[i]=(uint32_t)ticks;
    }
}
bool mcpwm_foc_vesc_override_active(bool second) { return s_vesc_owned[second?1u:0u] != 0u; }
bool mcpwm_foc_vesc_command_live(bool second) {
    const uint8_t i=second?1u:0u;
    return s_vesc_owned[i] && (s_vesc_timeout_ticks[i] != 0u || s_vesc_timeout_braking[i]);
}
void mcpwm_foc_vesc_override_clear(bool second) {
    const uint8_t i=second?1u:0u;
    s_vesc_owned[i]=0u; s_vesc_timeout_braking[i]=0u; s_vesc_timeout_expired[i]=0u; s_vesc_timeout_ticks[i]=0u;
}




/* ========================================================================== */
/* Original EFeru ADC DMA ISR timing/calibration path                          */
/* ========================================================================== */
void f103_DMA1_Channel1_IRQHandler_impl(void) {
    const uint32_t focIsrStartCycles = DWT->CYCCNT;
    /* Count actual CPU entry into DMA1 Channel1, independent of motor mode. */
    foc_irq_entry_count++;
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    if(foc_prof_reset_ack!=foc_prof_reset_request){foc_isr_profile_clear_isr_owned();FOC_MEMORY_BARRIER();foc_prof_reset_ack=foc_prof_reset_request;}
    foc_trace_service_requests_isr();

    if(offsetcount < 2000) {  // calibrate ADC offsets
        foc_isr_profile_slot=0xffu;
        offsetcount++;
        /* ADC F103 adalah unsigned 12-bit. Mean IIR /2 tidak perlu SDIV: semua
         * operand 0..4095, sehingga unsigned add + shift identik secara numerik. */
        offsetrlA = (int16_t)(((uint32_t)adc_buffer.rlA + (uint16_t)offsetrlA) >> 1);
        offsetrlB = (int16_t)(((uint32_t)adc_buffer.rlB + (uint16_t)offsetrlB) >> 1);
        offsetrrB = (int16_t)(((uint32_t)adc_buffer.rrB + (uint16_t)offsetrrB) >> 1);
        offsetrrC = (int16_t)(((uint32_t)adc_buffer.rrC + (uint16_t)offsetrrC) >> 1);
        offsetdcl = (int16_t)(((uint32_t)adc_buffer.dcl + (uint16_t)offsetdcl) >> 1);
        offsetdcr = (int16_t)(((uint32_t)adc_buffer.dcr + (uint16_t)offsetdcr) >> 1);
        if(offsetcount==2000u){
            m_motor_1.m_driven_offset0=offsetrlA; m_motor_1.m_driven_offset1=offsetrlB; m_motor_1.m_driven_offsetdc=offsetdcl;
            m_motor_2.m_driven_offset0=offsetrrB; m_motor_2.m_driven_offset1=offsetrrC; m_motor_2.m_driven_offsetdc=offsetdcr;
            /* Startup samples are bridge-OFF. They are a valid ADC baseline but
             * are not the upstream-style driven (50% PWM zero-vector) baseline. */
            m_motor_1.m_driven_offset_valid=1u; m_motor_2.m_driven_offset_valid=1u;
            m_motor_1.m_driven_offset_powered_valid=0u; m_motor_2.m_driven_offset_powered_valid=0u;
            m_motor_1.m_current_offset_valid=current_offset_pair_plausible(offsetrlA,offsetrlB)?1u:0u;
            m_motor_2.m_current_offset_valid=current_offset_pair_plausible(offsetrrB,offsetrrC)?1u:0u;
            if(!m_motor_1.m_current_offset_valid)motor_fault_set(&m_motor_1,FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1);
            if(!m_motor_2.m_current_offset_valid)motor_fault_set(&m_motor_2,FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1);
            m_motor_1.m_driven_offset_calibrating=0u; m_motor_2.m_driven_offset_calibrating=0u;
            m_motor_1.m_driven_offset_samples=2000u; m_motor_2.m_driven_offset_samples=2000u;
        }
        foc_isr_monitor_end(focIsrStartCycles,0u,0u);
        return;
    }

    /* Actuator watchdog deadline belongs to the hard realtime clock, not main
     * housekeeping. Two atomic 32-bit decrements are cheap and guarantee that a
     * deadlocked parser/main loop cannot hold the last VESC torque indefinitely.
     * On expiry hardware is disabled immediately; main later performs release or
     * configured timeout-brake state transitions. */
    for(uint8_t wi=0u;wi<2u;++wi){
        if(s_vesc_owned[wi] && !s_vesc_timeout_braking[wi]){
            uint32_t t=s_vesc_timeout_ticks[wi];
            if(t!=0u && t!=UINT32_MAX){
                --t; s_vesc_timeout_ticks[wi]=t;
                if(t==0u){
                    s_vesc_timeout_expired[wi]=1u;
                    if(wi==0u)LEFT_TIM->BDTR&=~TIM_BDTR_MOE;
                    else RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;
                }
            }
        }
    }

    /* Source ownership is exclusive. Once VESC owns an endpoint, a legacy
     * enable flag must never bypass an expired VESC deadline. */
    const bool estopActive=s_estop_ticks!=0u;
    const uint8_t leftSourceEnable=(!estopActive) &&
        (s_vesc_owned[0] ? mcpwm_foc_vesc_command_live(false) : (enable!=0u));
    const uint8_t rightSourceEnable=(!estopActive) &&
        (s_vesc_owned[1] ? mcpwm_foc_vesc_command_live(true) : (enable!=0u));
    const bool leftOpenloopRequest=(m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                    m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const bool rightOpenloopRequest=(m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                     m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const bool leftFeedbackReady=!m_motor_1.m_config_update_active && (leftOpenloopRequest ||
        (encoder_feedback_selected(&m_motor_1,false) ? m_motor_1.m_encoder_synced : hall_feedback_valid(&m_motor_1)));
    const bool rightFeedbackReady=!m_motor_2.m_config_update_active &&
        (rightOpenloopRequest || hall_feedback_valid(&m_motor_2));
    const uint8_t leftDriveRequest=(leftSourceEnable!=0u)&&(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)&&
        leftFeedbackReady&&m_motor_1.m_current_offset_valid;
    const uint8_t rightDriveRequest=(rightSourceEnable!=0u)&&(m_motor_2.m_control_mode!=CONTROL_MODE_NONE)&&
        rightFeedbackReady&&m_motor_2.m_current_offset_valid;
    const uint8_t isrSlot=s_foc_control_div;
    foc_isr_profile_slot=(isrSlot<MCCONF_FOC_CONTROL_DIV)?isrSlot:0xffu;
    if(isrSlot<MCCONF_FOC_CONTROL_DIV){
        if(foc_isr_expected_slot!=0xffu && isrSlot!=foc_isr_expected_slot)foc_isr_slot_sequence_error_count++;
        foc_isr_expected_slot=(uint8_t)((isrSlot+1u)>=MCCONF_FOC_CONTROL_DIV?0u:(isrSlot+1u));
        foc_isr_steady_count++;
    }
    if(foc_prof_sample_down==0u){
        foc_prof_detail_sample=1u; foc_prof_sample_down=(uint8_t)(FOC_PROF_SAMPLE_PERIOD-1u);
        if(isrSlot<MCCONF_FOC_CONTROL_DIV)foc_prof_detail_slot_count[isrSlot]++;
    }else {foc_prof_detail_sample=0u;foc_prof_sample_down--;}
    uint32_t focPreFaultEnd=0u, focPreOffsetEnd=0u;
    if(foc_prof_detail_sample){
        focPreFaultEnd=DWT->CYCCNT;
        const uint32_t v=focPreFaultEnd-focIsrStartCycles;
        if(v>foc_prof_pre_gate_max_cycles)foc_prof_pre_gate_max_cycles=v;
    }
    const bool offTelemetryLeft=(isrSlot==2u);
    const bool offTelemetryRight=(isrSlot==3u);
    const uint8_t leftBridgeWasOn=(LEFT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    const uint8_t rightBridgeWasOn=(RIGHT_TIM->BDTR&TIM_BDTR_MOE)?1u:0u;
    if(leftDriveRequest && !leftBridgeWasOn && m_motor_1.m_fault==FAULT_CODE_NONE){
        m_motor_1.m_bridge_settle_ticks=MCCONF_BRIDGE_SETTLE_SAMPLES;
        /* The powered low-side baseline is learned once per boot. Subsequent
         * OFF->RUN transitions still hold the zero-vector settle window, but
         * they reuse the validated powered baseline instead of making every
         * teleop start look like a fresh motor calibration. */
        if(!m_motor_1.m_driven_offset_powered_valid){
            m_motor_1.m_driven_offset_calibrating=1u;
            m_motor_1.m_driven_offset_samples=0u;
            m_motor_1.m_driven_offset_sum0=0; m_motor_1.m_driven_offset_sum1=0; m_motor_1.m_driven_offset_sumdc=0;
            m_motor_1.m_driven_offset_finalize_pending=0u;
        }else{
            m_motor_1.m_driven_offset_calibrating=0u;
            m_motor_1.m_driven_offset_finalize_pending=0u;
        }
        LEFT_TIM->LEFT_TIM_U=pwm_res/2u; LEFT_TIM->LEFT_TIM_V=pwm_res/2u; LEFT_TIM->LEFT_TIM_W=pwm_res/2u;
        reset_current_pi(&m_motor_1);
        m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
        m_motor_1.m_telem_current_lpf_q16[0]=m_motor_1.m_telem_current_lpf_q16[1]=m_motor_1.m_telem_current_lpf_q16[2]=0;
        m_motor_1.m_id_telem_q4=0; m_motor_1.m_iq_telem_q4=0; m_motor_1.m_current_in_telem_counts=0;
        m_motor_1.m_telem_sum_id_q4=0; m_motor_1.m_telem_sum_iq_q4=0; m_motor_1.m_telem_sum_ibus_counts=0; m_motor_1.m_telem_avg_samples=0u;
    }
    if(rightDriveRequest && !rightBridgeWasOn && m_motor_2.m_fault==FAULT_CODE_NONE){
        m_motor_2.m_bridge_settle_ticks=MCCONF_BRIDGE_SETTLE_SAMPLES;
        if(!m_motor_2.m_driven_offset_powered_valid){
            m_motor_2.m_driven_offset_calibrating=1u;
            m_motor_2.m_driven_offset_samples=0u;
            m_motor_2.m_driven_offset_sum0=0; m_motor_2.m_driven_offset_sum1=0; m_motor_2.m_driven_offset_sumdc=0;
            m_motor_2.m_driven_offset_finalize_pending=0u;
        }else{
            m_motor_2.m_driven_offset_calibrating=0u;
            m_motor_2.m_driven_offset_finalize_pending=0u;
        }
        RIGHT_TIM->RIGHT_TIM_U=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_V=pwm_res/2u; RIGHT_TIM->RIGHT_TIM_W=pwm_res/2u;
        reset_current_pi(&m_motor_2);
        m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
        m_motor_2.m_telem_current_lpf_q16[0]=m_motor_2.m_telem_current_lpf_q16[1]=m_motor_2.m_telem_current_lpf_q16[2]=0;
        m_motor_2.m_id_telem_q4=0; m_motor_2.m_iq_telem_q4=0; m_motor_2.m_current_in_telem_counts=0;
        m_motor_2.m_telem_sum_id_q4=0; m_motor_2.m_telem_sum_iq_q4=0; m_motor_2.m_telem_sum_ibus_counts=0; m_motor_2.m_telem_avg_samples=0u;
    }
    /* driven_offset_calibrating stays asserted for the complete 80-sample
     * zero-vector window and is cleared only by completion or abort below. */

    /* On the first ISR after a RUN->OFF transition the MOE state sampled at
     * entry is still ON, but it is disabled earlier in this same ISR. The
     * phase-current amplifier operating point can jump immediately to its OFF
     * bias. Reset measurement filters so that one mixed-offset sample cannot
     * appear as a 10 A telemetry spike. */
    if(!leftDriveRequest && leftBridgeWasOn){
        m_motor_1.m_driven_offset_calibrating=0u;
        /* Preserve the validated powered baseline across normal coast/stop. */
        m_motor_1.m_driven_offset_finalize_pending=0u;
        m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
        m_motor_1.m_telem_current_lpf_q16[0]=m_motor_1.m_telem_current_lpf_q16[1]=m_motor_1.m_telem_current_lpf_q16[2]=0;
        m_motor_1.m_id_telem_q4=0; m_motor_1.m_iq_telem_q4=0; m_motor_1.m_current_in_telem_counts=0;
        m_motor_1.m_telem_sum_id_q4=0; m_motor_1.m_telem_sum_iq_q4=0; m_motor_1.m_telem_sum_ibus_counts=0; m_motor_1.m_telem_avg_samples=0u;
    }
    if(!rightDriveRequest && rightBridgeWasOn){
        m_motor_2.m_driven_offset_calibrating=0u;
        /* Preserve the validated powered baseline across normal coast/stop. */
        m_motor_2.m_driven_offset_finalize_pending=0u;
        m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
        m_motor_2.m_telem_current_lpf_q16[0]=m_motor_2.m_telem_current_lpf_q16[1]=m_motor_2.m_telem_current_lpf_q16[2]=0;
        m_motor_2.m_id_telem_q4=0; m_motor_2.m_iq_telem_q4=0; m_motor_2.m_current_in_telem_counts=0;
        m_motor_2.m_telem_sum_id_q4=0; m_motor_2.m_telem_sum_iq_q4=0; m_motor_2.m_telem_sum_ibus_counts=0; m_motor_2.m_telem_avg_samples=0u;
    }

    /* Calibrate the high-impedance current-amplifier operating point only after
     * its common-mode transient has settled. After a driven interval, wait 50 ms
     * and (if the wheel moved) until the last Hall edge is stale before learning
     * 256 zero-current samples. This prevents post-release 5..20 A telemetry
     * artifacts while keeping manual back-drive current observable afterwards. */
    if(!leftBridgeWasOn && !leftDriveRequest && m_motor_1.m_off_settle_ticks>0u) m_motor_1.m_off_settle_ticks--;
    if(!rightBridgeWasOn && !rightDriveRequest && m_motor_2.m_off_settle_ticks>0u) m_motor_2.m_off_settle_ticks--;
    const bool leftOffStationary=(m_motor_1.m_rpm==0) &&
        (m_motor_1.m_hall_direction==0 || m_motor_1.m_hall_ticks>=MCCONF_HALL_TIMEOUT_TICKS);
    const bool rightOffStationary=(m_motor_2.m_rpm==0) &&
        (m_motor_2.m_hall_direction==0 || m_motor_2.m_hall_ticks>=MCCONF_HALL_TIMEOUT_TICKS);
    /* The low-side current amplifier high-Z operating point drifts for hundreds
     * of milliseconds after MOE is disabled. A one-shot OFF zero therefore
     * becomes stale and can show tens of amps while the bridge is open. Track
     * the OFF zero slowly only while Hall motion is stale/stationary. Once the
     * rotor moves, freeze the baseline so passive/manual-spin current remains
     * observable. This path is telemetry-only and never feeds protection. */
    if(offTelemetryLeft && m_motor_1.m_off_offset_valid && !leftBridgeWasOn && !leftDriveRequest && leftOffStationary){
        if((++m_motor_1.m_off_offset_samples & 7u)==0u){
            int32_t d=(int32_t)adc_buffer.rlA-m_motor_1.m_off_offset0; d=CLAMP(d,-64,64); m_motor_1.m_off_offset0+=div8_toward_zero_i16(d);
            d=(int32_t)adc_buffer.rlB-m_motor_1.m_off_offset1; d=CLAMP(d,-64,64); m_motor_1.m_off_offset1+=div8_toward_zero_i16(d);
            d=(int32_t)adc_buffer.dcl-m_motor_1.m_off_offsetdc; d=CLAMP(d,-32,32); m_motor_1.m_off_offsetdc+=div8_toward_zero_i16(d);
        }
    }
    if(offTelemetryRight && m_motor_2.m_off_offset_valid && !rightBridgeWasOn && !rightDriveRequest && rightOffStationary){
        if((++m_motor_2.m_off_offset_samples & 7u)==0u){
            int32_t d=(int32_t)adc_buffer.rrB-m_motor_2.m_off_offset0; d=CLAMP(d,-64,64); m_motor_2.m_off_offset0+=div8_toward_zero_i16(d);
            d=(int32_t)adc_buffer.rrC-m_motor_2.m_off_offset1; d=CLAMP(d,-64,64); m_motor_2.m_off_offset1+=div8_toward_zero_i16(d);
            d=(int32_t)adc_buffer.dcr-m_motor_2.m_off_offsetdc; d=CLAMP(d,-32,32); m_motor_2.m_off_offsetdc+=div8_toward_zero_i16(d);
        }
    }
    if(offTelemetryLeft && !m_motor_1.m_off_offset_valid && m_motor_1.m_off_settle_ticks==0u && !leftBridgeWasOn && !leftDriveRequest && leftOffStationary){
        m_motor_1.m_off_offset_sum0+=(int32_t)adc_buffer.rlA;
        m_motor_1.m_off_offset_sum1+=(int32_t)adc_buffer.rlB;
        m_motor_1.m_off_offset_sumdc+=(int32_t)adc_buffer.dcl;
        if(++m_motor_1.m_off_offset_samples>=256u){
            m_motor_1.m_off_offset0=(int16_t)(m_motor_1.m_off_offset_sum0>>8);
            m_motor_1.m_off_offset1=(int16_t)(m_motor_1.m_off_offset_sum1>>8);
            m_motor_1.m_off_offsetdc=(int16_t)(m_motor_1.m_off_offset_sumdc>>8);
            m_motor_1.m_off_offset_valid=1u;
            m_motor_1.m_current_lpf_q16[0]=m_motor_1.m_current_lpf_q16[1]=0;
            m_motor_1.m_telem_current_lpf_q16[0]=m_motor_1.m_telem_current_lpf_q16[1]=m_motor_1.m_telem_current_lpf_q16[2]=0;
        }
    }
    if(offTelemetryRight && !m_motor_2.m_off_offset_valid && m_motor_2.m_off_settle_ticks==0u && !rightBridgeWasOn && !rightDriveRequest && rightOffStationary){
        m_motor_2.m_off_offset_sum0+=(int32_t)adc_buffer.rrB;
        m_motor_2.m_off_offset_sum1+=(int32_t)adc_buffer.rrC;
        m_motor_2.m_off_offset_sumdc+=(int32_t)adc_buffer.dcr;
        if(++m_motor_2.m_off_offset_samples>=256u){
            m_motor_2.m_off_offset0=(int16_t)(m_motor_2.m_off_offset_sum0>>8);
            m_motor_2.m_off_offset1=(int16_t)(m_motor_2.m_off_offset_sum1>>8);
            m_motor_2.m_off_offsetdc=(int16_t)(m_motor_2.m_off_offset_sumdc>>8);
            m_motor_2.m_off_offset_valid=1u;
            m_motor_2.m_current_lpf_q16[0]=m_motor_2.m_current_lpf_q16[1]=0;
            m_motor_2.m_telem_current_lpf_q16[0]=m_motor_2.m_telem_current_lpf_q16[1]=m_motor_2.m_telem_current_lpf_q16[2]=0;
        }
    }

    /* Powered zero-vector calibration: ISR hanya mengakumulasi ADC. Pembagian
     * dan plausibility final dilakukan housekeeping 200 Hz. Ini menghilangkan
     * 3 SDIV per PWM frame dari jendela 80-sample. Saat 80 sampel lengkap,
     * settle_ticks sengaja ditahan di 1 sampai slow-path menandai baseline valid. */
    if(leftBridgeWasOn && leftDriveRequest && m_motor_1.m_bridge_settle_ticks>0u &&
       m_motor_1.m_driven_offset_calibrating && !m_motor_1.m_driven_offset_finalize_pending){
        m_motor_1.m_driven_offset_sum0+=(int32_t)adc_buffer.rlA;
        m_motor_1.m_driven_offset_sum1+=(int32_t)adc_buffer.rlB;
        m_motor_1.m_driven_offset_sumdc+=(int32_t)adc_buffer.dcl;
        if(m_motor_1.m_driven_offset_samples<MCCONF_BRIDGE_SETTLE_SAMPLES)
            m_motor_1.m_driven_offset_samples++;
        if(m_motor_1.m_driven_offset_samples>=MCCONF_BRIDGE_SETTLE_SAMPLES)
            m_motor_1.m_driven_offset_finalize_pending=1u;
    }
    if(rightBridgeWasOn && rightDriveRequest && m_motor_2.m_bridge_settle_ticks>0u &&
       m_motor_2.m_driven_offset_calibrating && !m_motor_2.m_driven_offset_finalize_pending){
        m_motor_2.m_driven_offset_sum0+=(int32_t)adc_buffer.rrB;
        m_motor_2.m_driven_offset_sum1+=(int32_t)adc_buffer.rrC;
        m_motor_2.m_driven_offset_sumdc+=(int32_t)adc_buffer.dcr;
        if(m_motor_2.m_driven_offset_samples<MCCONF_BRIDGE_SETTLE_SAMPLES)
            m_motor_2.m_driven_offset_samples++;
        if(m_motor_2.m_driven_offset_samples>=MCCONF_BRIDGE_SETTLE_SAMPLES)
            m_motor_2.m_driven_offset_finalize_pending=1u;
    }

    /* Three current-sampling states are kept deliberately separate:
     *  1) DRIVEN+settled: use the powered zero-vector baseline learned above.
     *  2) Stable bridge-OFF: use the high-impedance offset for telemetry only.
     *  3) OFF<->RUN transition/settling: publish zero and never trip current.
     * Only state (1) is ever allowed to feed over-current protection below. */
    const uint8_t leftCurrentSampleValid=leftBridgeWasOn&&leftDriveRequest&&(m_motor_1.m_bridge_settle_ticks==0u)&&m_motor_1.m_driven_offset_valid;
    const uint8_t rightCurrentSampleValid=rightBridgeWasOn&&rightDriveRequest&&(m_motor_2.m_bridge_settle_ticks==0u)&&m_motor_2.m_driven_offset_valid;
    const uint8_t leftOffTelemValid=(!leftBridgeWasOn)&&(!leftDriveRequest)&&m_motor_1.m_off_offset_valid;
    const uint8_t rightOffTelemValid=(!rightBridgeWasOn)&&(!rightDriveRequest)&&m_motor_2.m_off_offset_valid;
    if(leftCurrentSampleValid){
        curL_phaA=(int16_t)(m_motor_1.m_driven_offset0-(int16_t)adc_buffer.rlA);
        curL_phaB=(int16_t)(m_motor_1.m_driven_offset1-(int16_t)adc_buffer.rlB);
        curL_DC=(int16_t)(m_motor_1.m_driven_offsetdc-(int16_t)adc_buffer.dcl);
    }else if(leftOffTelemValid){
        curL_phaA=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offset0-(int16_t)adc_buffer.rlA));
        curL_phaB=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offset1-(int16_t)adc_buffer.rlB));
        curL_DC=off_telem_deadband_counts((int16_t)(m_motor_1.m_off_offsetdc-(int16_t)adc_buffer.dcl));
    }else{ curL_phaA=0; curL_phaB=0; curL_DC=0; }
    if(rightCurrentSampleValid){
        curR_phaB=(int16_t)(m_motor_2.m_driven_offset0-(int16_t)adc_buffer.rrB);
        curR_phaC=(int16_t)(m_motor_2.m_driven_offset1-(int16_t)adc_buffer.rrC);
        curR_DC=(int16_t)(m_motor_2.m_driven_offsetdc-(int16_t)adc_buffer.dcr);
    }else if(rightOffTelemValid){
        curR_phaB=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offset0-(int16_t)adc_buffer.rrB));
        curR_phaC=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offset1-(int16_t)adc_buffer.rrC));
        curR_DC=off_telem_deadband_counts((int16_t)(m_motor_2.m_off_offsetdc-(int16_t)adc_buffer.dcr));
    }else{ curR_phaB=0; curR_phaC=0; curR_DC=0; }
    if(foc_prof_detail_sample){
        focPreOffsetEnd=DWT->CYCCNT;
        const uint32_t v=focPreOffsetEnd-focPreFaultEnd;
        if(v>foc_prof_pre_offset_max_cycles)foc_prof_pre_offset_max_cycles=v;
    }

    const int32_t curL_phaC=-(int32_t)curL_phaA-(int32_t)curL_phaB;
    const int32_t curR_phaA=-(int32_t)curR_phaB-(int32_t)curR_phaC;
    const uint8_t leftOpenloop = (m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                  m_motor_1.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const uint8_t rightOpenloop = (m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP ||
                                   m_motor_2.m_control_mode==CONTROL_MODE_OPENLOOP_PHASE);
    const int32_t leftPhaseLimit=leftOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):m_motor_1.m_abs_current_limit_counts;
    const int32_t rightPhaseLimit=rightOpenloop?((int32_t)SVPWM_PHASE_LIMIT_A*A2BIT_CONV):m_motor_2.m_abs_current_limit_counts;
    /* Phase over-current protection must apply to every powered mode. Hall
     * detection uses the mode-4 current-control power path, so its fixed-phase
     * submode gets the same stricter phase/DC limits even when the legacy
     * ctrlModReq is not SVPWM_MODE. */
    /* VESC suppresses current-unbalance diagnostics when phase sampling loses
     * observability at high duty, but ABS over-current protection stays active.
     * On this fixed low-side-shunt board, qualify a >80% phase excursion for a
     * few consecutive ADC frames instead of disabling the fault outright. */
    /* ABS current is a VESC motor-current protection, not a raw two-shunt ADC
     * plausibility test. EFeru feeds the raw phase channels into the current
     * controller and uses DC-link current as the immediate hardware-level trip.
     * Near high duty one phase can be poorly observable; reconstructing C=-A-B
     * and faulting on that raw sample created false ABS trips at ~0.96 duty and
     * even on 1 A startup commands. Use D/Q current magnitude instead:
     *   slow_abs=false -> fast FOC feedback LPF (control current)
     *   slow_abs=true  -> additional monitoring LPF (slower VESC ABS option)
     * The raw DC-link trip below remains immediate. */
    const int32_t leftAbsQ4=leftPhaseLimit*16;
    const int32_t rightAbsQ4=rightPhaseLimit*16;
    const uint8_t leftDqFresh=m_motor_1.m_dq_sample_fresh;
    const uint8_t rightDqFresh=m_motor_2.m_dq_sample_fresh;
    uint8_t leftPhaseExceeded=0u, rightPhaseExceeded=0u;
    if(leftCurrentSampleValid && leftDqFresh){
        const int32_t fd=m_motor_1.m_id_q4, fq=m_motor_1.m_iq_q4;
        const int32_t sd=m_motor_1.m_id_telem_q4, sq=m_motor_1.m_iq_telem_q4;
        const uint32_t fast2=(uint32_t)(fd*fd)+(uint32_t)(fq*fq);
        const uint32_t slow2=(uint32_t)(sd*sd)+(uint32_t)(sq*sq);
        const uint32_t lim2=(uint32_t)(leftAbsQ4*leftAbsQ4);
        leftPhaseExceeded=m_motor_1.m_conf.l_slow_abs_current?(slow2>lim2):(fast2>lim2);
        if(!leftPhaseExceeded)m_motor_1.m_phase_overcurrent_streak=0u;
        else if(m_motor_1.m_phase_overcurrent_streak<MCCONF_ABS_CURRENT_QUAL_SAMPLES)m_motor_1.m_phase_overcurrent_streak++;
        m_motor_1.m_dq_sample_fresh=0u;
    }else if(!leftCurrentSampleValid){m_motor_1.m_phase_overcurrent_streak=0u;m_motor_1.m_dq_sample_fresh=0u;}
    if(rightCurrentSampleValid && rightDqFresh){
        const int32_t fd=m_motor_2.m_id_q4, fq=m_motor_2.m_iq_q4;
        const int32_t sd=m_motor_2.m_id_telem_q4, sq=m_motor_2.m_iq_telem_q4;
        const uint32_t fast2=(uint32_t)(fd*fd)+(uint32_t)(fq*fq);
        const uint32_t slow2=(uint32_t)(sd*sd)+(uint32_t)(sq*sq);
        const uint32_t lim2=(uint32_t)(rightAbsQ4*rightAbsQ4);
        rightPhaseExceeded=m_motor_2.m_conf.l_slow_abs_current?(slow2>lim2):(fast2>lim2);
        if(!rightPhaseExceeded)m_motor_2.m_phase_overcurrent_streak=0u;
        else if(m_motor_2.m_phase_overcurrent_streak<MCCONF_ABS_CURRENT_QUAL_SAMPLES)m_motor_2.m_phase_overcurrent_streak++;
        m_motor_2.m_dq_sample_fresh=0u;
    }else if(!rightCurrentSampleValid){m_motor_2.m_phase_overcurrent_streak=0u;m_motor_2.m_dq_sample_fresh=0u;}
    /* Three distinct per-motor D/Q regulator samples (16 kHz / DIV6 =
     * 2.667 kHz, about 1.125 ms total) reject turn-on/sampling transients.
     * Catastrophic raw DC-link overcurrent
     * remains an immediate 16-kHz shutdown path below. */
    const uint8_t leftPhaseTrip=leftDqFresh&&leftPhaseExceeded&&m_motor_1.m_phase_overcurrent_streak>=MCCONF_ABS_CURRENT_QUAL_SAMPLES;
    const uint8_t rightPhaseTrip=rightDqFresh&&rightPhaseExceeded&&m_motor_2.m_phase_overcurrent_streak>=MCCONF_ABS_CURRENT_QUAL_SAMPLES;
    const int32_t leftDcLimit=leftOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    const int32_t rightDcLimit=rightOpenloop?((int32_t)SVPWM_DC_LIMIT_A*A2BIT_CONV):curDC_max;
    const uint8_t leftDcTrip = leftCurrentSampleValid && (ABS(curL_DC) > leftDcLimit);
    const uint8_t rightDcTrip = rightCurrentSampleValid && (ABS(curR_DC) > rightDcLimit);
    const uint8_t leftCurrentTrip = leftPhaseTrip || leftDcTrip;
    const uint8_t rightCurrentTrip = rightPhaseTrip || rightDcTrip;
    /* Safety gate phase 1: trip/stop disables the bridge immediately. Do NOT
     * arm a previously-off bridge here. The FOC step below must first compute
     * and write CCRs for the new command; otherwise the first powered PWM frame
     * uses the stale OFF-state 50% zero vector and can create a current spike. */
    if(leftCurrentTrip || leftDriveRequest==0u || m_motor_1.m_fault!=FAULT_CODE_NONE){
        LEFT_TIM->BDTR&=~TIM_BDTR_MOE;
        if(leftCurrentTrip){
            m_motor_1.m_current_trip_count++; if(leftPhaseTrip)m_motor_1.m_phase_trip_count++; if(leftDcTrip)m_motor_1.m_dc_trip_count++;
            m_motor_1.m_last_trip_source=(leftPhaseTrip?1u:0u)|(leftDcTrip?2u:0u);
            m_motor_1.m_last_trip_phase0_counts=curL_phaA; m_motor_1.m_last_trip_phase1_counts=curL_phaB; m_motor_1.m_last_trip_phase2_counts=(int16_t)CLAMP(curL_phaC,-32768,32767);
            m_motor_1.m_last_trip_dc_counts=curL_DC;
            m_motor_1.m_last_trip_duty_permille=duty_permille_from_vdq(m_motor_1.m_vd,m_motor_1.m_vq);
            motor_fault_set(&m_motor_1,FAULT_CODE_ABS_OVER_CURRENT);
        }
    }
    if(rightCurrentTrip || rightDriveRequest==0u || m_motor_2.m_fault!=FAULT_CODE_NONE){
        RIGHT_TIM->BDTR&=~TIM_BDTR_MOE;
        if(rightCurrentTrip){
            m_motor_2.m_current_trip_count++; if(rightPhaseTrip)m_motor_2.m_phase_trip_count++; if(rightDcTrip)m_motor_2.m_dc_trip_count++;
            m_motor_2.m_last_trip_source=(rightPhaseTrip?1u:0u)|(rightDcTrip?2u:0u);
            m_motor_2.m_last_trip_phase0_counts=(int16_t)CLAMP(curR_phaA,-32768,32767); m_motor_2.m_last_trip_phase1_counts=curR_phaB; m_motor_2.m_last_trip_phase2_counts=curR_phaC;
            m_motor_2.m_last_trip_dc_counts=curR_DC;
            m_motor_2.m_last_trip_duty_permille=duty_permille_from_vdq(m_motor_2.m_vd,m_motor_2.m_vq);
            motor_fault_set(&m_motor_2,FAULT_CODE_ABS_OVER_CURRENT);
        }
    }

    /* buzzerTimer tetap monotonic 16-kHz clock untuk main-loop cadence. Audio
     * pattern diproses sebagai counter O(1), tanpa div/mod di ISR motor. */
    buzzerTimer++;
    static uint16_t buzzerBlockTicks=0u;
    static uint8_t buzzerBlock=0u;
    static uint8_t buzzerToggleTicks=0u;
    if(++buzzerBlockTicks>=5000u){
        buzzerBlockTicks=0u;
        const uint8_t blocks=(uint8_t)(buzzerPattern+1u);
        if(++buzzerBlock>=blocks)buzzerBlock=0u;
        if(buzzerBlock==0u){buzzerPrev=1u;if(++buzzerIdx>(uint8_t)(buzzerCount+2u))buzzerIdx=1u;}
        else {buzzerPrev=0u;buzzer_pin_low_fast();}
    }
    if(buzzerFreq!=0u && buzzerBlock==0u && (buzzerIdx<=buzzerCount || buzzerCount==0u)){
        if(++buzzerToggleTicks>=buzzerFreq){buzzerToggleTicks=0u;buzzer_pin_toggle_fast();}
    }else{buzzerToggleTicks=0u;if(buzzerFreq==0u)buzzer_pin_low_fast();}

    if (s_overrun) { m_motor_1.m_overrun_count++;m_motor_2.m_overrun_count++;foc_isr_monitor_end(focIsrStartCycles,0u,0u);return; }
    s_overrun=1;
    /* Detailed checkpoints are sampled; total/slot accounting remains continuous. */
    uint32_t focControlStart=0u, focPostStart=0u;
    if(foc_prof_detail_sample){
        focControlStart=DWT->CYCCNT;
        const uint32_t protect=focControlStart-focPreOffsetEnd;
        const uint32_t pre=focControlStart-focIsrStartCycles;
        if(protect>foc_prof_pre_protect_max_cycles)foc_prof_pre_protect_max_cycles=protect;
        if(pre>foc_prof_pre_max_cycles)foc_prof_pre_max_cycles=pre;
        foc_prof_detail_sample_count++;
    }
    mcpwm_foc_adc_int_handler();
    if(s_foc_trace_fault_pending && !s_foc_trace_frozen){
        s_foc_trace_event_latch|=1u<<6;foc_trace_capture_internal(0xfeu);
        s_foc_trace_trigger_motor=s_foc_trace_fault_motor;s_foc_trace_trigger_fault=s_foc_trace_fault_code;
        s_foc_trace_fault_pending=0u;s_foc_trace_frozen=1u;s_foc_step_test.active=0u;s_foc_step_test.done=1u;
    }
    if(foc_prof_detail_sample){
        focPostStart=DWT->CYCCNT;
        const uint32_t control=focPostStart-focControlStart;
        if(control>foc_prof_control_max_cycles)foc_prof_control_max_cycles=control;
    }

    /* odom_l/odom_r are wrapped incrementally on each accepted Hall edge.
     * This is exactly position_counts modulo 9000 without two software integer
     * divisions at 2.667 kHz inside the highest-priority ADC ISR. */
    if(leftBridgeWasOn && leftDriveRequest && m_motor_1.m_bridge_settle_ticks>0u && !m_motor_1.m_driven_offset_finalize_pending) m_motor_1.m_bridge_settle_ticks--;
    if(rightBridgeWasOn && rightDriveRequest && m_motor_2.m_bridge_settle_ticks>0u && !m_motor_2.m_driven_offset_finalize_pending) m_motor_2.m_bridge_settle_ticks--;

    /* Safety gate phase 2: only after motor_control_step has written the CCRs
     * may a previously-off bridge be armed. Re-evaluate sensor feedback here:
     * hall_update()/encoder update runs inside motor_control_step, so the
     * pre-control driveRequest can be stale on the exact frame where a debounced
     * Hall input becomes invalid or an encoder loses readiness. Never re-arm
     * MOE from that stale pre-control decision. */
    const bool leftFeedbackReadyPost=!m_motor_1.m_config_update_active && (leftOpenloopRequest ||
        (encoder_feedback_selected(&m_motor_1,false) ? m_motor_1.m_encoder_synced : hall_feedback_valid(&m_motor_1)));
    const bool rightFeedbackReadyPost=!m_motor_2.m_config_update_active &&
        (rightOpenloopRequest || hall_feedback_valid(&m_motor_2));
    if(leftDriveRequest && leftFeedbackReadyPost && !leftCurrentTrip && m_motor_1.m_fault==FAULT_CODE_NONE) LEFT_TIM->BDTR|=TIM_BDTR_MOE;
    if(rightDriveRequest && rightFeedbackReadyPost && !rightCurrentTrip && m_motor_2.m_fault==FAULT_CODE_NONE) RIGHT_TIM->BDTR|=TIM_BDTR_MOE;
    s_overrun=0;
    /* Only a complete ISR pass counts as healthy progress. Startup calibration
     * and overrun-abort paths intentionally do not increment this token. */
    foc_adc_heartbeat++;
    foc_isr_monitor_end(focIsrStartCycles,focPostStart,foc_prof_detail_sample);
}



#if !defined(__arm__) && !defined(__thumb__)
/* Host regression tidak memakai vector table STM32. Sediakan nama IRQ standar
 * sebagai wrapper tipis agar test menjalankan implementasi ISR yang sama. */
void DMA1_Channel1_IRQHandler(void) {
    f103_DMA1_Channel1_IRQHandler_impl();
}
#endif

uint8_t mcpwm_foc_hall_detect_angle200(int64_t sum_s, int64_t sum_c, uint16_t n) {
    if (n <= 30u) return 255u;
    /* Finalisasi tetap memakai atan2 dari vector akumulasi, tetapi dalam phase
     * integer sehingga libm atan2/atan tidak ikut ke image F103. */
    const int32_t ys=sum_s>INT32_MAX?INT32_MAX:(sum_s<INT32_MIN?INT32_MIN:(int32_t)sum_s);
    const int32_t xc=sum_c>INT32_MAX?INT32_MAX:(sum_c<INT32_MIN?INT32_MIN:(int32_t)sum_c);
    const uint16_t phase=foc_atan2_phase_u16(ys,xc);
    uint32_t v=((uint32_t)phase*200u)>>16;
    if(v>199u)v=199u;
    return (uint8_t)v;
}

#ifdef MCPWM_FOC_HOST_REFERENCE_DETECT
/* Host-only blocking reference for regression against upstream VESC.
 * Production STM32F103 uses the cooperative mcpwm_foc_hall_detect_* worker
 * in vesc_protocol.c, so carrying this second blocking implementation in flash
 * would be pure dead code. */
bool mcpwm_foc_hall_detect(float current, bool second, uint8_t table[8]) {
    /* Same detection method/name as upstream mcpwm_foc_hall_detect(). The
     * F103 dual-motor port adds the explicit `second` selector and returns bool
     * instead of ChibiOS fault+result because faults are stored per motor. */
    if (!table) return false;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    float max_i = m->m_conf.l_current_max;
    if (max_i <= 0.0f || max_i > (float)I_MOT_MAX) max_i = (float)I_MOT_MAX;
    if (current < 0.0f) current = -current;
    if (current > max_i) current = max_i;

    int64_t sum_s[8] = {0};
    int64_t sum_c[8] = {0};
    uint16_t samples[8] = {0};
    for (uint8_t i = 0u; i < 8u; ++i) table[i] = 255u;

    /* Upstream: ramp alignment current for 1000 x 1 ms. */
    for (uint16_t i = 0u; i < 1000u; ++i) {
        mcpwm_foc_set_openloop_phase((float)i * current / 1000.0f, 0.0f, second);
        mcpwm_foc_vesc_override_touch(second);
        if (m->m_fault != FAULT_CODE_NONE) goto exit_hall_detect;
        HAL_Delay(1u);
    }

    /* Upstream: three full forward sweeps, j=0..359. */
    for (uint8_t pass = 0u; pass < 3u; ++pass) {
        for (uint16_t deg = 0u; deg < 360u; ++deg) {
            mcpwm_foc_set_openloop_phase(current, (float)deg, second);
            mcpwm_foc_vesc_override_touch(second);
            if (m->m_fault != FAULT_CODE_NONE) goto exit_hall_detect;
            HAL_Delay(5u);
            const uint8_t h = m->m_hall_filtered_state & 7u;
            int16_t sn,cs;
            const uint16_t ph=(uint16_t)(((uint32_t)deg*65536u)/360u);
            foc_sin_cos_q15(ph,&sn,&cs);
            sum_s[h] += sn; sum_c[h] += cs;
            if (samples[h] < 0xffffu) samples[h]++;
        }
    }

    /* Upstream: three full reverse sweeps, j=360..0 inclusive. */
    for (uint8_t pass = 0u; pass < 3u; ++pass) {
        for (int16_t deg = 360; deg >= 0; --deg) {
            mcpwm_foc_set_openloop_phase(current, (float)deg, second);
            mcpwm_foc_vesc_override_touch(second);
            if (m->m_fault != FAULT_CODE_NONE) goto exit_hall_detect;
            HAL_Delay(5u);
            const uint8_t h = m->m_hall_filtered_state & 7u;
            const uint16_t dnorm = (deg == 360) ? 0u : (uint16_t)deg;
            int16_t sn,cs;
            const uint16_t ph=(uint16_t)(((uint32_t)dnorm*65536u)/360u);
            foc_sin_cos_q15(ph,&sn,&cs);
            sum_s[h] += sn; sum_c[h] += cs;
            if (samples[h] < 0xffffu) samples[h]++;
        }
    }

    {
        uint8_t fails = 0u;
        for (uint8_t h = 0u; h < 8u; ++h) {
            table[h] = mcpwm_foc_hall_detect_angle200(sum_s[h], sum_c[h], samples[h]);
            if (table[h] == 255u) fails++;
        }
        mcpwm_foc_release_motor(second);
        mcpwm_foc_vesc_override_clear(second);
        return fails == 2u;
    }

exit_hall_detect:
    mcpwm_foc_release_motor(second);
    mcpwm_foc_vesc_override_clear(second);
    return false;
}

#endif

bool mcpwm_foc_dc_cal_done(void){return offsetcount>=2000u;}
void mcpwm_foc_get_current_offsets(int16_t *p0,int16_t *p1,int16_t *dc,bool second){if(!second){if(p0)*p0=offsetrlA;if(p1)*p1=offsetrlB;if(dc)*dc=offsetdcl;}else{if(p0)*p0=offsetrrB;if(p1)*p1=offsetrrC;if(dc)*dc=offsetdcr;}}
uint32_t mcpwm_foc_get_isr_cycles(void){return foc_isr_cycles;}uint32_t mcpwm_foc_get_isr_cycles_max(void){return foc_isr_cycles_max;}
void mcpwm_foc_get_liveness(uint32_t *adc_heartbeat,uint32_t motor_heartbeat[2]){
    if(adc_heartbeat)*adc_heartbeat=foc_adc_heartbeat;
    if(motor_heartbeat){motor_heartbeat[0]=foc_motor_heartbeat[0];motor_heartbeat[1]=foc_motor_heartbeat[1];}
}
static void foc_trace_clear_isr_owned(void){
    s_foc_trace_head=0u;s_foc_trace_count=0u;s_foc_trace_frozen=0u;
    s_foc_trace_trigger_motor=0u;s_foc_trace_trigger_fault=0u;s_foc_trace_write_count=0u;
    s_foc_trace_fault_pending=0u;s_foc_trace_event_latch=0u;s_foc_trace_cycle_pending=0u;
}
static void foc_trace_service_requests_isr(void){
    if(s_foc_trace_clear_req){foc_trace_clear_isr_owned();s_foc_trace_clear_req=0u;FOC_MEMORY_BARRIER();s_foc_trace_ack_seq=s_foc_trace_req_seq;}
    if(s_foc_trace_freeze_req){s_foc_trace_frozen=1u;s_foc_trace_freeze_req=0u;FOC_MEMORY_BARRIER();s_foc_trace_ack_seq=s_foc_trace_req_seq;}
}
static bool foc_trace_request_wait(bool clear){
    const uint32_t req=s_foc_trace_req_seq+1u;s_foc_trace_req_seq=req;
    if(clear)s_foc_trace_clear_req=1u;else s_foc_trace_freeze_req=1u;
    FOC_MEMORY_BARRIER();
#if defined(__arm__) || defined(__thumb__)
    if((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)==0u){if(clear)foc_trace_clear_isr_owned();else s_foc_trace_frozen=1u;s_foc_trace_clear_req=s_foc_trace_freeze_req=0u;s_foc_trace_ack_seq=req;return true;}
    const uint32_t start=DWT->CYCCNT;
    while(s_foc_trace_ack_seq!=req){if((uint32_t)(DWT->CYCCNT-start)>(FOC_ISR_BUDGET_CYCLES*16u))return false;}
#else
    if(clear)foc_trace_clear_isr_owned();else s_foc_trace_frozen=1u;s_foc_trace_clear_req=s_foc_trace_freeze_req=0u;s_foc_trace_ack_seq=req;
#endif
    return true;
}
bool mcpwm_foc_trace_freeze(void){return foc_trace_request_wait(false);}
bool mcpwm_foc_trace_clear(void){return foc_trace_request_wait(true);}
void mcpwm_foc_trace_get_meta(mcpwm_foc_trace_meta_t *out){
    if(!out)return;
    out->write_count=s_foc_trace_write_count;
    out->frozen=s_foc_trace_frozen;
    out->trigger_motor=s_foc_trace_trigger_motor;out->trigger_fault=s_foc_trace_trigger_fault;
    out->count=s_foc_trace_count;out->head=s_foc_trace_head;out->capacity=MCPWM_FOC_TRACE_CAPACITY;
    out->sample_size=(uint16_t)sizeof(mcpwm_foc_trace_sample_t);
}
bool mcpwm_foc_trace_read(uint8_t chronological_index,mcpwm_foc_trace_sample_t *out){
    if(!out)return false;
    const uint8_t count=s_foc_trace_count;
    if(chronological_index>=count)return false;
    const uint8_t head=s_foc_trace_head; const uint8_t oldest=(uint8_t)((head+MCPWM_FOC_TRACE_CAPACITY-count)%MCPWM_FOC_TRACE_CAPACITY);
    const mcpwm_foc_trace_sample_t *src=&s_foc_trace[(oldest+chronological_index)%MCPWM_FOC_TRACE_CAPACITY];
    for(uint8_t retry=0u;retry<3u;++retry){
        const uint32_t a=src->guard;if(a&1u)continue;FOC_MEMORY_BARRIER();mcpwm_foc_trace_sample_t tmp=*src;FOC_MEMORY_BARRIER();
        const uint32_t b=src->guard;if(a==b && !(b&1u)){*out=tmp;return true;}
    }
    return false;
}

void mcpwm_foc_get_adc_sample_diag(bool second,mcpwm_foc_adc_sample_diag_t *out){
    if(!out)return;
    const mcpwm_foc_motor_t *m=second?&m_motor_2:&m_motor_1;
    uint32_t e0=0u,x0=0u,e1=0u,x1=0u;
    bool snapshot_ok=false;
    for(uint8_t retry=0u; retry<32u; ++retry){
        mcpwm_foc_get_irq_epoch(&e0,&x0);if(e0!=x0)continue;
        out->ccr_a=m->m_ccr_a;out->ccr_b=m->m_ccr_b;out->ccr_c=m->m_ccr_c;
        out->zero_window_counts=m->m_sample_zero_window_counts;out->min_window_counts=m->m_sample_window_min_counts;
        out->guard_counts=m->m_sample_guard_counts;out->adc_phase_counts=m->m_sample_adc_phase_counts;out->invalid_count=m->m_sample_invalid_count;
        out->sector=m->m_sample_sector;out->window_valid=m->m_sample_window_valid;out->offset_valid=m->m_current_offset_valid;
        out->driven_offset_valid=m->m_driven_offset_valid;out->bridge_settled=(m->m_bridge_settle_ticks==0u)?1u:0u;
        mcpwm_foc_get_irq_epoch(&e1,&x1);if(e0==e1&&x0==x1&&e1==x1){snapshot_ok=true;break;}
    }
    if(!snapshot_ok){ memset(out,0,sizeof(*out)); out->window_valid=0u; }
}

bool mcpwm_foc_step_test_arm_axis(float pre_a,float step_a,uint8_t pre_samples,uint8_t post_samples,
                                  bool second,mcpwm_foc_step_axis_t axis){
    mcpwm_foc_motor_t *m=second?&m_motor_2:&m_motor_1;
    const uint16_t requested=(uint16_t)pre_samples+(uint16_t)post_samples;
    if(s_foc_step_test.active || m->m_fault!=FAULT_CODE_NONE || pre_samples<2u || post_samples<2u ||
       requested>MCPWM_FOC_TRACE_CAPACITY || (axis!=MCPWM_FOC_STEP_AXIS_Q && axis!=MCPWM_FOC_STEP_AXIS_D))return false;
    const int16_t pre=amp_to_q4(m,pre_a), step=amp_to_q4(m,step_a);
    if(!mcpwm_foc_trace_clear())return false;
    /* Q-axis uses the normal VESC current command. D-axis commissioning locks
     * the present electrical phase and steps Id with Iq=0, so the two PI axes
     * can be validated independently without changing the runtime control law. */
    if(axis==MCPWM_FOC_STEP_AXIS_D){
        set_control_mode(m,CONTROL_MODE_OPENLOOP_PHASE);
        m->m_phase_openloop=m->m_phase;m->m_phase_override=1u;
        m->m_openloop_id_target_q4=pre;m->m_openloop_id_ramp_q16=(int32_t)pre*65536;m->m_id_set_q4=pre;
        m->m_iq_target_q4=0;m->m_iq_set_q4=0;m->m_iq_set_ramp_q16=0;
    }else{
        mcpwm_foc_set_current((float)pre/(float)FOC_CURRENT_Q4_PER_A,second);
    }
    mcpwm_foc_vesc_override_touch(second);
    s_foc_step_test.sequence++;s_foc_step_test.pre_q4=pre;s_foc_step_test.step_q4=step;s_foc_step_test.second=second?1u:0u;
    s_foc_step_test.axis=(uint8_t)axis;s_foc_step_test.pre_remaining=pre_samples;s_foc_step_test.post_remaining=post_samples;
    s_foc_step_test.step_fired=0u;s_foc_step_test.done=0u;s_foc_step_test.settling=1u;s_foc_step_test.settle_remaining=255u;
    FOC_MEMORY_BARRIER();s_foc_step_test.active=1u;return true;
}

bool mcpwm_foc_step_test_arm(float pre_a,float step_a,uint8_t pre_samples,uint8_t post_samples,bool second){
    return mcpwm_foc_step_test_arm_axis(pre_a,step_a,pre_samples,post_samples,second,MCPWM_FOC_STEP_AXIS_Q);
}
void mcpwm_foc_step_test_get(mcpwm_foc_step_test_status_t *out){
    if(!out)return;
    FOC_MEMORY_BARRIER();
    out->sequence=s_foc_step_test.sequence;out->pre_q4=s_foc_step_test.pre_q4;out->step_q4=s_foc_step_test.step_q4;
    out->active=s_foc_step_test.active;out->second=s_foc_step_test.second;out->pre_remaining=s_foc_step_test.pre_remaining;out->post_remaining=s_foc_step_test.post_remaining;
    out->step_fired=s_foc_step_test.step_fired;out->done=s_foc_step_test.done;FOC_MEMORY_BARRIER();
}

static float q4_to_amp(int16_t q){return (float)q/(float)FOC_CURRENT_Q4_PER_A;}
static float motor_current_vesc_a(const mcpwm_foc_motor_t *m){
    const int32_t id=m->m_id_telem_q4;
    const int32_t iq=m->m_iq_telem_q4;
    const uint32_t mag2=(uint32_t)(id*id)+(uint32_t)(iq*iq);
    int32_t mag=(int32_t)foc_isqrt_u32(mag2);
    /* Monitoring API follows the same filtered DQ/Ibus snapshot used by
     * COMM_GET_VALUES. The fast current controller still uses m_id_q4/m_iq_q4. */
    if(m->m_current_in_telem_counts>0)mag=-mag;
    return (float)mag/(float)FOC_CURRENT_Q4_PER_A;
}
float mcpwm_foc_get_tot_current_motor(bool s){return motor_current_vesc_a(mcpwm_foc_get_motor_const(s));}
float mcpwm_foc_get_tot_current_in_motor(bool s){return -(float)mcpwm_foc_get_motor_const(s)->m_current_in_telem_counts/(float)A2BIT_CONV;}
float mcpwm_foc_get_motor_mechanical_rpm(bool s){
    const float pp=(float)motor_pole_pairs(s);
    return pp>0.0f ? mcpwm_foc_get_erpm_motor(s)/pp : 0.0f;
}
float mcpwm_foc_get_output_rpm(bool s){return mcpwm_foc_get_motor_mechanical_rpm(s)/motor_gear_ratio(s);}
uint16_t mcpwm_foc_get_pole_pairs(bool s){return motor_pole_pairs(s);}
float mcpwm_foc_get_gear_ratio(bool s){return motor_gear_ratio(s);}
float mcpwm_foc_get_erpm_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    if(encoder_feedback_selected(m,s) && m->m_encoder_configured)
        return (float)m->m_encoder_erpm_q16/65536.0f;
    /* One Hall edge is 60 electrical degrees. Derive ERPM directly from edge
     * period so low-speed values such as 50 ERPM are not quantized through an
     * integer mechanical-RPM intermediate (50 ERPM @15pp used to display 45). */
    if(m->m_hall_initialized && m->m_hall_direction!=0 &&
       m->m_hall_period>0u && m->m_hall_period<MCCONF_HALL_TIMEOUT_TICKS &&
       m->m_hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS){
        const float erpm=((float)PWM_FREQ*10.0f)/(float)m->m_hall_period;
        return erpm*(float)hall_motion_direction(s,m->m_hall_direction);
    }
    return (float)m->m_rpm*(float)motor_pole_pairs(s);
}
float mcpwm_foc_get_duty_cycle_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_duty_now_permille/1000.0f;}
float mcpwm_foc_get_id_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_id_telem_q4);}float mcpwm_foc_get_iq_motor(bool s){return q4_to_amp(mcpwm_foc_get_motor_const(s)->m_iq_telem_q4);}
static float bus_voltage_now(void){return ((float)batVoltage*(float)BAT_CALIB_REAL_VOLTAGE)/(float)BAT_CALIB_ADC/100.0f;}

void mcpwm_foc_energy_update(uint32_t now_ms) {
    /* Keep the proven hoverFOC ADC ISR free of VESC reciprocal/division work.
     * Battery ADC/filtering remains in DMA1_Channel1 exactly as the master;
     * only the derived VESC voltage scale is refreshed here in main context. */
    if (s_voltage_scale_bat_adc != batVoltage) current_voltage_scale_refresh();
    if (s_energy_last_ms == 0u) { s_energy_last_ms = now_ms; return; }
    uint32_t dt_ms = (uint32_t)(now_ms - s_energy_last_ms);
    if (dt_ms == 0u) return;
    s_energy_last_ms = now_ms;
    /* Do not integrate a long debugger/power-stall gap as real energy. */
    if (dt_ms > 100u) dt_ms = 100u;
    const float dt_s = (float)dt_ms * 0.001f;
    const float vin = bus_voltage_now();
    mcpwm_foc_motor_t *motors[2] = {&m_motor_1, &m_motor_2};
    for (uint8_t k = 0u; k < 2u; ++k) {
        mcpwm_foc_motor_t *m = motors[k];
        const float current_in = -(float)m->m_current_in_counts / (float)A2BIT_CONV;
        const float amp_s = current_in * dt_s;
        const float watt_s = current_in * vin * dt_s;
        if (current_in >= 0.0f) {
            m->m_amp_seconds += amp_s;
            m->m_watt_seconds += watt_s;
        } else {
            m->m_amp_seconds_charged -= amp_s;
            m->m_watt_seconds_charged -= watt_s;
        }
    }
}
/* Internal D/Q is the VESC modulation normalized to 16000 == mod 1.0.
 * Upstream uses mod = Vdq * 1.5 / Vbus, hence Vdq = mod*Vbus/1.5. */
float mcpwm_foc_get_vd_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vd*(bus_voltage_now()/(1.5f*(float)MCCONF_FOC_VOLTAGE_MAX));}
float mcpwm_foc_get_vq_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_vq*(bus_voltage_now()/(1.5f*(float)MCCONF_FOC_VOLTAGE_MAX));}
float mcpwm_foc_get_phase_motor(bool s){return (float)mcpwm_foc_get_motor_const(s)->m_phase*(360.0f/65536.0f);}
bool mcpwm_foc_observer_valid(bool s){return mcpwm_foc_get_motor_const(s)->m_observer_valid!=0u && foc_observer_model_valid(mcpwm_foc_get_motor_const(s));}
float mcpwm_foc_get_phase_observer_motor(bool s){
    const mcpwm_foc_motor_t *m=mcpwm_foc_get_motor_const(s);
    if(!mcpwm_foc_observer_valid(s))return 0.0f;
    const float ox=m->m_observer_x1-m->m_observer_l_ia;
    const float oy=m->m_observer_x2-m->m_observer_l_ib;
    const float norm=fmaxf(fabsf(ox),fabsf(oy));
    int32_t xi=0,yi=0;
    if(norm>1e-12f){
        xi=(int32_t)(ox*(32767.0f/norm));
        yi=(int32_t)(oy*(32767.0f/norm));
    }
    float deg=(float)foc_atan2_phase_u16(yi,xi)*(360.0f/65536.0f);
    /* Same switching-lag compensation concept as upstream m_phase_now_observer.
     * This port updates the diagnostic observer at the current-regulator cadence. */
    const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
    deg += mcpwm_foc_get_erpm_motor(s)*6.0f*dt*(0.5f+m->m_conf.foc_observer_offset);
    while(deg>=360.0f)deg-=360.0f;
    while(deg<0.0f)deg+=360.0f;
    return deg;
}
float mcpwm_foc_get_phase_encoder_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    return (!s && m->m_encoder_configured)?(float)m->m_phase_encoder*(360.0f/65536.0f):0.0f;
}
float mcpwm_foc_get_encoder_position_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    return (!s && m->m_encoder_configured)?(float)m->m_encoder_mech_phase*(360.0f/65536.0f):0.0f;
}
float mcpwm_foc_get_pid_pos_now_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    return (float)position_feedback_phase_u16(m,s)*(360.0f/65536.0f);
}
float mcpwm_foc_get_pid_pos_set_motor(bool s){
    const mcpwm_foc_motor_t*m=mcpwm_foc_get_motor_const(s);
    return (float)m->m_pos_pid_set_phase*(360.0f/65536.0f);
}
mc_state mcpwm_foc_get_state_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_state;}mc_fault_code mcpwm_foc_get_fault_motor(bool s){return mcpwm_foc_get_motor_const(s)->m_fault;}

typedef struct {
    int16_t rpm,duty,vd,vq;
    int32_t encoder_erpm_q16,tachometer;
    uint32_t tachometer_abs;
    uint16_t phase,encoder_mech_phase,hall_period,hall_ticks,bridge_settle_ticks;
    uint8_t hall_initialized,encoder_configured,driven_offset_valid,driven_offset_calibrating;
    int8_t hall_direction;
    mc_control_mode control_mode;
    mc_fault_code fault;
} foc_telem_isr_snapshot_t;

static void foc_telem_isr_snapshot(const mcpwm_foc_motor_t *m, foc_telem_isr_snapshot_t *o) {
    uint32_t e0=0u,x0=0u,e1=0u,x1=0u;
    if(!m||!o)return;
    bool snapshot_ok=false;
    for(uint8_t retry=0u; retry<32u; ++retry) {
        mcpwm_foc_get_irq_epoch(&e0,&x0);
        if(e0!=x0)continue;
        o->rpm=m->m_rpm; o->duty=m->m_duty_now_permille; o->vd=m->m_vd; o->vq=m->m_vq;
        o->encoder_erpm_q16=m->m_encoder_erpm_q16; o->encoder_configured=m->m_encoder_configured;
        o->tachometer=m->m_tachometer; o->tachometer_abs=m->m_tachometer_abs;
        o->phase=m->m_phase; o->encoder_mech_phase=m->m_encoder_mech_phase;
        o->hall_initialized=m->m_hall_initialized; o->hall_direction=m->m_hall_direction;
        o->hall_period=m->m_hall_period; o->hall_ticks=m->m_hall_ticks; o->fault=m->m_fault;
        o->control_mode=m->m_control_mode; o->driven_offset_valid=m->m_driven_offset_valid;
        o->driven_offset_calibrating=m->m_driven_offset_calibrating; o->bridge_settle_ticks=m->m_bridge_settle_ticks;
        mcpwm_foc_get_irq_epoch(&e1,&x1);
        if(e0==e1 && x0==x1 && e1==x1){snapshot_ok=true;break;}
    }
    if(!snapshot_ok) memset(o,0,sizeof(*o));
}

static void foc_telem_consume_main(mcpwm_foc_motor_t *m, int16_t *id_q4, int16_t *iq_q4, int16_t *ibus_counts,
                                   int32_t *sum_id_q4, int32_t *sum_iq_q4, int32_t *sum_ibus_counts, uint16_t *avg_n) {
    *id_q4=m->m_id_telem_q4; *iq_q4=m->m_iq_telem_q4; *ibus_counts=m->m_current_in_telem_counts;
    *sum_id_q4=m->m_telem_sum_id_q4; *sum_iq_q4=m->m_telem_sum_iq_q4; *sum_ibus_counts=m->m_telem_sum_ibus_counts; *avg_n=m->m_telem_avg_samples;
    m->m_telem_sum_id_q4=0; m->m_telem_sum_iq_q4=0; m->m_telem_sum_ibus_counts=0; m->m_telem_avg_samples=0u;
}

void mcpwm_foc_get_values_scaled(mcpwm_foc_values_scaled_t *v,bool second){
    if(!v)return;
    const mcpwm_foc_motor_t *m=mcpwm_foc_get_motor_const(second);
    int16_t id_q4,iq_q4,ibus_counts;
    int32_t sum_id_q4,sum_iq_q4,sum_ibus_counts; uint16_t avg_n;
    foc_telem_isr_snapshot_t is;
    foc_telem_consume_main((mcpwm_foc_motor_t *)m,&id_q4,&iq_q4,&ibus_counts,&sum_id_q4,&sum_iq_q4,&sum_ibus_counts,&avg_n);
    foc_telem_isr_snapshot(m,&is);
    if(avg_n>0u){
        id_q4=(int16_t)(sum_id_q4/(int32_t)avg_n);
        iq_q4=(int16_t)(sum_iq_q4/(int32_t)avg_n);
        ibus_counts=(int16_t)(sum_ibus_counts/(int32_t)avg_n);
    }else{
        /* Never reuse a previous GET_VALUES current cache when no fresh
         * telemetry sample exists in this request window. This is especially
         * important across OFF<->RUN current-offset transitions. */
        id_q4=0; iq_q4=0; ibus_counts=0;
    }
    /* Acquisition already qualifies powered and OFF-state current separately.
     * Do not erase a valid bridge-OFF sample here. */
    const int32_t q4pa=(int32_t)FOC_CURRENT_Q4_PER_A;
    v->id_x100=((int32_t)id_q4*100)/q4pa;
    v->iq_x100=((int32_t)iq_q4*100)/q4pa;
    {const uint32_t mag2=(uint32_t)((int32_t)id_q4*id_q4)+(uint32_t)((int32_t)iq_q4*iq_q4);int32_t mag=(int32_t)foc_isqrt_u32(mag2);if(ibus_counts>0)mag=-mag;v->current_motor_x100=(mag*100)/q4pa;}
    v->current_in_x100=(-(int32_t)ibus_counts*100)/(int32_t)A2BIT_CONV;
    v->duty_x1000=is.duty;
    if(encoder_feedback_selected(m,second)&&is.encoder_configured)v->erpm=is.encoder_erpm_q16/65536;
    else if(is.hall_initialized&&is.hall_direction!=0&&is.hall_period>0u&&is.hall_period<MCCONF_HALL_TIMEOUT_TICKS&&is.hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS)
        v->erpm=((int32_t)PWM_FREQ*10/(int32_t)is.hall_period)*(int32_t)hall_motion_direction(second,is.hall_direction);
    else v->erpm=(int32_t)is.rpm*(int32_t)motor_pole_pairs(second);
    const int32_t vin_cv=((int32_t)(batVoltage>0?batVoltage:1)*(int32_t)BAT_CALIB_REAL_VOLTAGE)/(int32_t)BAT_CALIB_ADC;
    v->vin_x10=(int16_t)(vin_cv/10);
    /* Energy counter tetap float karena integrator housekeeping memang float, tetapi
     * hanya empat perkalian konstan; semua current/RPM/Vdq realtime sudah integer. */
    v->ah_x10000=(int32_t)(m->m_amp_seconds*2.7777778f);
    v->ah_charged_x10000=(int32_t)(m->m_amp_seconds_charged*2.7777778f);
    v->wh_x10000=(int32_t)(m->m_watt_seconds*2.7777778f);
    v->wh_charged_x10000=(int32_t)(m->m_watt_seconds_charged*2.7777778f);
    v->tachometer=is.tachometer;
    v->tachometer_abs=is.tachometer_abs>(uint32_t)INT32_MAX?INT32_MAX:(int32_t)is.tachometer_abs;
    v->fault=(uint8_t)is.fault;
    const int64_t den=(int64_t)3*(int64_t)MCCONF_FOC_VOLTAGE_MAX;
    v->vd_x1000=(int32_t)(((int64_t)is.vd*(int64_t)vin_cv*20LL)/den);
    v->vq_x1000=(int32_t)(((int64_t)is.vq*(int64_t)vin_cv*20LL)/den);
}

void mcpwm_foc_get_values(mc_values *v,bool second){
    if (!v) return;
    memset(v, 0, sizeof(*v));
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    /* Snapshot every ISR-owned field in one very short critical section. Do
     * all floating-point conversion afterwards, so one VESC telemetry packet
     * can never mix Id/Iq/RPM/phase values from different 16-kHz frames. */
    int16_t id_q4,iq_q4,ibus_counts;
    int32_t sum_id_q4,sum_iq_q4,sum_ibus_counts; uint16_t avg_n;
    foc_telem_isr_snapshot_t is;
    foc_telem_consume_main((mcpwm_foc_motor_t *)m,&id_q4,&iq_q4,&ibus_counts,&sum_id_q4,&sum_iq_q4,&sum_ibus_counts,&avg_n);
    foc_telem_isr_snapshot(m,&is);
    if(avg_n>0u){
        id_q4=(int16_t)(sum_id_q4/(int32_t)avg_n);
        iq_q4=(int16_t)(sum_iq_q4/(int32_t)avg_n);
        ibus_counts=(int16_t)(sum_ibus_counts/(int32_t)avg_n);
    }else{
        id_q4=0; iq_q4=0; ibus_counts=0;
    }
    /* Same acquisition-qualified current as the fixed-point GET_VALUES path.
     * OFF telemetry uses its own high-impedance baseline and is safe to expose. */

    const float vin=bus_voltage_now();
    v->v_in=vin;
    v->amp_hours=m->m_amp_seconds/3600.0f;
    v->amp_hours_charged=m->m_amp_seconds_charged/3600.0f;
    v->watt_hours=m->m_watt_seconds/3600.0f;
    v->watt_hours_charged=m->m_watt_seconds_charged/3600.0f;
    v->id=q4_to_amp(id_q4); v->iq=q4_to_amp(iq_q4);
    { uint32_t mag2=(uint32_t)((int32_t)id_q4*id_q4)+(uint32_t)((int32_t)iq_q4*iq_q4);
      int32_t mag=(int32_t)foc_isqrt_u32(mag2); if(ibus_counts>0)mag=-mag;
      v->current_motor=(float)mag/(float)FOC_CURRENT_Q4_PER_A; }
    v->current_in=-(float)ibus_counts/(float)A2BIT_CONV;
    if(encoder_feedback_selected(m,second) && is.encoder_configured)
        v->rpm=(float)is.encoder_erpm_q16/65536.0f;
    else if(is.hall_initialized && is.hall_direction!=0 && is.hall_period>0u && is.hall_period<MCCONF_HALL_TIMEOUT_TICKS && is.hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS)
        v->rpm=((float)PWM_FREQ*10.0f/(float)is.hall_period)*(float)hall_motion_direction(second,is.hall_direction);
    else v->rpm=(float)is.rpm*(float)motor_pole_pairs(second);
    /* Wire VESC: tachometer tetap 60 electrical degree per count, bukan raw
     * ABI quadrature count. Position memakai mechanical encoder angle ketika
     * ABI dikonfigurasi, sama seperti m_pos_pid_now upstream. */
    v->tachometer=is.tachometer;
    v->tachometer_abs=is.tachometer_abs>(uint32_t)INT32_MAX?INT32_MAX:(int32_t)is.tachometer_abs;
    const uint16_t pos_phase=(!second && is.encoder_configured)?is.encoder_mech_phase:is.phase;
    v->position=(float)pos_phase*(360.0f/65536.0f);
    v->duty_now=(float)is.duty/1000.0f; v->fault_code=is.fault; v->vesc_id=second?2:1;
    /* Upstream VESC: modulation = Vdq * 1.5 / Vbus. Internal D/Q uses the
     * modulation-count scale, so physical Vdq is mod * Vbus / 1.5. Keep
     * COMM_GET_VALUES consistent with the individual Vd/Vq getters. */
    v->vd=(float)is.vd*(vin/(1.5f*(float)MCCONF_FOC_VOLTAGE_MAX));
    v->vq=(float)is.vq*(vin/(1.5f*(float)MCCONF_FOC_VOLTAGE_MAX));
}
