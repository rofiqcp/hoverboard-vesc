#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "stm32f1xx_hal.h"
#include "vesc/datatypes.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include "vesc/vesc_protocol.h"
#include "vesc/app_vesc.h"
#include "vesc/mcconf_serial.h"
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "platform_watchdog.h"
#include "motor/mcconf_default.h"
#include "defines.h"

UART_HandleTypeDef huart3 = {0};
volatile adc_buf_t adc_buffer = {0};
volatile uint8_t steering_detect_stage=0u;
volatile uint8_t encoder_detect_stage=0u;
volatile uint8_t encoder_align_stage=0u;
volatile uint32_t encoder_align_before_count=0u, encoder_align_jog_count=0u, encoder_align_back_count=0u;
volatile int32_t encoder_align_jog_delta=0, encoder_align_back_delta=0;
volatile uint16_t encoder_align_current_ma=0u;
volatile int32_t encoder_detect_plus_mdeg=0, encoder_detect_minus_mdeg=0;
volatile uint32_t encoder_gpio_edge_a=0u, encoder_gpio_edge_b=0u, encoder_gpio_edge_pb5=0u, encoder_gpio_samples=0u;
int16_t board_temp_deg_c = 31;
static uint32_t tick_ms = 1000u;
static uint8_t tx_capture[1024];
static uint16_t tx_capture_len = 0u;
static int selected_motor = 1;
static float set_current[2];
static float set_rpm[2];
static float set_duty[2];
static float set_pos[2];
static unsigned touch_count[2];
static unsigned store_count[2];
static unsigned clear_faults_count=0u;
static bool store_ok=true, load_ok=true;
static mc_configuration confs[2];
static mcpwm_foc_motor_t diag_motors[2];
/* Deterministic motor plant used to prove Detect-All actually identifies
 * parameters rather than returning success after Hall-only setup. */
static const float plant_r[2]={0.180f,0.220f};
static const float plant_l[2]={0.000350f,0.000420f};
static const float plant_flux[2]={0.0180f,0.0200f};
static float plant_vd[2]={0.0f,0.0f}, plant_vq[2]={0.0f,0.0f};
static bool rl_capture_on[2]={false,false};
static bool plant_fail_rl=false;
static int32_t pos_user[2] = {0,0};
static int32_t pos_target_user[2] = {0,0};
static int32_t pos_min_user[2] = {INT32_MIN,INT32_MIN};
static int32_t pos_max_user[2] = {INT32_MAX,INT32_MAX};
static uint32_t fw_erase_size=0u, fw_write_offset=0u, fw_write_len=0u;
static unsigned fw_reset_count=0u;
bool f103_fw_erase_staging(uint32_t fw_size){fw_erase_size=fw_size;return fw_size>0u;}
bool f103_fw_write_staging(uint32_t offset,const uint8_t *data,uint32_t len){(void)data;fw_write_offset=offset;fw_write_len=len;return len>0u;}
void f103_fw_reset_to_bootloader(void){fw_reset_count++;}

uint32_t HAL_GetTick(void) { return tick_ms; }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *d, uint16_t n, uint32_t t) {
    (void)h; (void)t;
    if (n > sizeof(tx_capture)) return 1;
    memcpy(tx_capture,d,n); tx_capture_len=n; return HAL_OK;
}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h, uint8_t *d, uint16_t n) {
    (void)h;
    if (n > sizeof(tx_capture)) return 1;
    memcpy(tx_capture,d,n); tx_capture_len=n; return HAL_OK;
}
void __disable_irq(void) {}
void __enable_irq(void) {}
void foc_sin_cos_q15(uint16_t phase, int16_t *sn, int16_t *cs) {
    const double a=(double)phase*(6.28318530717958647692/65536.0);
    if(sn)*sn=(int16_t)lround(sin(a)*32767.0);
    if(cs)*cs=(int16_t)lround(cos(a)*32767.0);
}
float foc_sqrtf_slow(float x) { return x>0.0f?sqrtf(x):0.0f; }

void mc_interface_select_motor_thread(int motor) { selected_motor=motor; }
bool mc_interface_dccal_done(void){return true;}
const volatile mc_configuration *mc_interface_get_configuration_motor(bool second) { return &confs[second?1:0]; }
void mc_interface_set_configuration(mc_configuration *configuration) { confs[selected_motor==2?1:0]=*configuration; }
void mc_interface_get_values_motor(mc_values *v, bool second) {
    memset(v,0,sizeof(*v));
    v->v_in=48.1f; v->current_motor=second?2.5f:1.5f; v->current_in=second?1.1f:0.8f;
    v->id=second?0.2f:0.1f; v->iq=second?-2.5f:1.5f; v->rpm=second?-321.0f:123.0f;
    v->duty_now=second?-0.22f:0.11f; v->fault_code=FAULT_CODE_NONE; v->vd=1.2f; v->vq=second?-5.0f:4.0f;
    v->position=second?342.0f:12.5f; v->tachometer=second?-45:31; v->tachometer_abs=second?45:31;
}
float mc_interface_get_pid_pos_now_motor(bool second) { return second?342.0f:12.5f; }
float mc_interface_get_pid_pos_set_motor(bool second) { return set_pos[second?1:0]; }
void mc_interface_set_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_current_rel(float val) {
    const int j=selected_motor==2?1:0;
    const float duty=set_duty[j];
    const float max_i=confs[j].l_current_max*confs[j].l_current_max_scale;
    float min_i=confs[j].l_current_min*confs[j].l_current_min_scale;
    if(min_i<0.0f)min_i=-min_i;
    const bool same=(fabsf(duty)<0.02f)||((val>=0.0f)==(duty>=0.0f));
    set_current[j]=val*(same?max_i:min_i);
}
void mc_interface_set_brake_current(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_brake_current_rel(float val) {
    const int j=selected_motor==2?1:0;
    float min_i=confs[j].l_current_min*confs[j].l_current_min_scale;
    if(min_i<0.0f)min_i=-min_i;
    set_current[j]=val*min_i;
}
void mc_interface_set_handbrake(float c) { set_current[selected_motor==2?1:0]=c; }
void mc_interface_set_pid_speed(float r) { set_rpm[selected_motor==2?1:0]=r; }
void mc_interface_set_pid_pos(float p) { set_pos[selected_motor==2?1:0]=p; }
static int32_t mock_steering_span=683;
static bool mock_steering_cal=true;
static bool mock_steering_homed=true;
static unsigned steering_span_hw_calls=0u;
static unsigned encoder_detect_calls=0u;
static bool encoder_detect_saw_motor_model=false;
bool mc_interface_steering_calibration_valid(void){return mock_steering_cal;}
bool mc_interface_reset_steering_calibration(void){return true;}
static bool mock_steering_logical_inv=false;
bool mc_interface_steering_logical_inverted(void){return mock_steering_logical_inv;}
bool mc_interface_set_steering_logical_inverted(bool v){mock_steering_logical_inv=v;return true;}
bool mc_interface_store_steering_calibration(void){return true;}
bool mc_interface_steering_boot_home(void){diag_motors[0].m_encoder_synced=1u;return true;}
bool mc_interface_steering_set_current_as_center(void){set_pos[0]=0.0f;return true;}
float mc_interface_get_steering_deg(void){return 12.5f;}
bool mc_interface_set_steering_deg(float p){set_pos[0]=p;return true;}
bool mc_interface_steering_detect_calibrate(float current,float *offset,float *ratio,bool *inverted,
                                            int32_t *raw_left,int32_t *raw_right,int32_t *span){
    (void)current; if(offset)*offset=0.0f; if(ratio)*ratio=4.0f; if(inverted)*inverted=false;
    if(raw_left)*raw_left=0;
    if(raw_right)*raw_right=683;
    if(span)*span=683;
    mock_steering_span=683; mock_steering_cal=true; mock_steering_homed=true; steering_span_hw_calls++;
    return true;
}
void mc_interface_get_steering_span_diag(int32_t *neg1,int32_t *pos1,int32_t *neg2,int32_t *pos2,
                                         int32_t *span1,int32_t *span2,int32_t *tolerance){
    if(neg1) *neg1=0;
    if(pos1) *pos1=683;
    if(neg2) *neg2=1;
    if(pos2) *pos2=684;
    if(span1) *span1=683;
    if(span2) *span2=683;
    if(tolerance) *tolerance=32;
}
void mcpwm_foc_sync_tuning_to_conf(bool second) { (void)second; }
void mcpwm_foc_get_default_configuration(mc_configuration *c, bool second) {
    (void)second; memset(c,0,sizeof(*c)); c->motor_type=MOTOR_TYPE_FOC; c->l_current_max=15.0f; c->l_current_min=-15.0f;
    c->foc_sensor_mode=FOC_SENSOR_MODE_HALL; c->si_motor_poles=30u;
    { const uint8_t t[8]={255u,83u,17u,50u,150u,117u,183u,255u}; for(int i=0;i<8;i++) c->foc_hall_table[i]=(int8_t)t[i]; }
}
void mc_interface_set_duty(float d) { set_duty[selected_motor==2?1:0]=d; }
void mc_interface_set_openloop_phase(float current,float phase){mcpwm_foc_set_openloop_phase(current,phase,selected_motor==2);}
void mc_interface_set_openloop_current(float current,float rpm){mcpwm_foc_set_openloop_current(current,rpm,selected_motor==2);}
mc_state mc_interface_get_state_motor(bool second){return diag_motors[second?1:0].m_state;}
mc_fault_code mc_interface_get_fault_motor(bool second) { (void)second; return FAULT_CODE_NONE; }
void mcpwm_foc_vesc_timeout_configure(bool second, uint32_t timeout_ms, float brake_current) {
    (void)second; (void)timeout_ms; (void)brake_current;
}
static uint16_t estop_hold_ms=0u;
void mcpwm_foc_estop_both(uint16_t duration_ms) {
    estop_hold_ms=duration_ms;
    diag_motors[0].m_control_mode=CONTROL_MODE_NONE;
    diag_motors[1].m_control_mode=CONTROL_MODE_NONE;
}
bool mcpwm_foc_estop_active(void) { return estop_hold_ms!=0u; }
void mcpwm_foc_vesc_override_touch(bool second) { touch_count[second?1:0]++; }
void mcpwm_foc_vesc_override_clear(bool second) { touch_count[second?1:0]=0u; }
bool mcpwm_foc_vesc_override_active(bool second) { return touch_count[second?1:0] != 0u; }
mcpwm_foc_motor_t *mcpwm_foc_get_motor(bool second) { return &diag_motors[second?1:0]; }
const mcpwm_foc_motor_t *mcpwm_foc_get_motor_const(bool second) { return &diag_motors[second?1:0]; }
void mcpwm_foc_set_openloop_phase(float current, float phase, bool second) {
    const int j=second?1:0;
    const float ia=plant_fail_rl?0.0f:fabsf(current);
    diag_motors[j].m_openloop_id_target_q4=(int16_t)lroundf(ia*800.0f);
    diag_motors[j].m_id_q4=diag_motors[j].m_openloop_id_target_q4;
    diag_motors[j].m_id_telem_q4=diag_motors[j].m_id_q4;
    plant_vd[j]=plant_r[j]*ia;
    diag_motors[j].m_vd=(int16_t)lroundf(plant_vd[j]/0.001f);
    diag_motors[j].m_control_mode=CONTROL_MODE_OPENLOOP_PHASE;
    while(phase>=360.0f) phase-=360.0f;
    while(phase<0.0f) phase+=360.0f;
    /* Different raw Hall permutations model swapped phase/Hall wiring. */
    static const uint8_t seq0[6]={1u,2u,3u,4u,5u,6u};
    static const uint8_t seq1[6]={6u,4u,5u,1u,3u,2u};
    int sec=(int)(phase/60.0f); if(sec<0)sec=0; if(sec>5)sec=5;
    diag_motors[j].m_hall_state=j?seq1[sec]:seq0[sec];
    diag_motors[j].m_hall_filtered_state=diag_motors[j].m_hall_state;
    diag_motors[j].m_hall_raw_state=diag_motors[j].m_hall_state;
}
void mcpwm_foc_set_openloop_current(float current,float rpm,bool second){
    const int j=second?1:0; const float ia=fabsf(current);
    diag_motors[j].m_control_mode=CONTROL_MODE_OPENLOOP;
    diag_motors[j].m_id_q4=0;
    diag_motors[j].m_id_telem_q4=0;
    plant_vd[j]=0.0f;
    diag_motors[j].m_vd=0;
    diag_motors[j].m_iq_q4=(int16_t)lroundf(ia*800.0f);
    diag_motors[j].m_iq_telem_q4=diag_motors[j].m_iq_q4;
    diag_motors[j].m_rpm=(int16_t)lroundf(rpm);
    const float omega=fabsf(rpm)*6.28318530717958647692f/60.0f;
    /* Match the VESC open-loop flux model used by the detector:
     * Vmag = R*I + omega*(lambda + L*I). */
    plant_vq[j]=plant_r[j]*ia+omega*(plant_flux[j]+plant_l[j]*ia);
    diag_motors[j].m_vq=(int16_t)lroundf(plant_vq[j]/0.001f);
}
float mcpwm_foc_get_id_motor(bool second){return (float)diag_motors[second?1:0].m_id_telem_q4/800.0f;}
float mcpwm_foc_get_iq_motor(bool second){return (float)diag_motors[second?1:0].m_iq_telem_q4/800.0f;}
float mcpwm_foc_get_vd_motor(bool second){return plant_vd[second?1:0];}
float mcpwm_foc_get_vq_motor(bool second){return plant_vq[second?1:0];}
void mcpwm_foc_rl_capture_start(bool second){rl_capture_on[second?1:0]=true;}
void mcpwm_foc_rl_capture_stop(bool second){rl_capture_on[second?1:0]=false;}
void mcpwm_foc_rl_capture_get(bool second,mcpwm_foc_rl_capture_t *o){
    const int j=second?1:0; if(!o)return; memset(o,0,sizeof(*o));
    o->samples=200u; o->sum_di2=1000000LL;
    /* vscale=1 mV/internal count, q4_per_A=800, dt=control_div/PWM_FREQ. */
    const double b=(double)plant_l[j]/(0.001*800.0*((double)MCCONF_FOC_CONTROL_DIV/(double)PWM_FREQ));
    o->sum_div=(int64_t)llround(b*(double)o->sum_di2);
}
void mc_interface_release_motor(void) { diag_motors[selected_motor==2?1:0].m_control_mode=CONTROL_MODE_NONE; }
void mcpwm_foc_release_motor(bool second) { diag_motors[second?1:0].m_control_mode=CONTROL_MODE_NONE; }
void mcpwm_foc_clear_faults(void) { clear_faults_count++; diag_motors[0].m_fault=FAULT_CODE_NONE; diag_motors[1].m_fault=FAULT_CODE_NONE; diag_motors[0].m_control_mode=CONTROL_MODE_NONE; diag_motors[1].m_control_mode=CONTROL_MODE_NONE; }
void mcpwm_foc_force_bridges_off(void) {
    diag_motors[0].m_control_mode=CONTROL_MODE_NONE;
    diag_motors[1].m_control_mode=CONTROL_MODE_NONE;
}
float mcpwm_foc_get_phase_motor(bool second) { return (float)diag_motors[second?1:0].m_phase * (360.0f / 65536.0f); }
bool mcpwm_foc_observer_valid(bool second) { (void)second; return true; }
float mcpwm_foc_get_phase_observer_motor(bool second) { return second?210.0f:100.0f; }
float mcpwm_foc_get_phase_encoder_motor(bool second) { return second?0.0f:12.5f; }
float mcpwm_foc_get_encoder_position_motor(bool second) { return second?0.0f:12.5f; }
bool mcpwm_foc_encoder_is_synced(bool second) { return !second && diag_motors[0].m_encoder_synced!=0u; }
bool mcpwm_foc_steering_is_calibrated(void){return mock_steering_cal;}
bool mcpwm_foc_steering_is_homed(void){return mock_steering_homed;}
void mcpwm_foc_steering_clear_calibration(void){mock_steering_cal=false;mock_steering_homed=false;mock_steering_span=0;}
bool mcpwm_foc_steering_set_span(int32_t span,bool homed){if(span==0)return false;mock_steering_span=span;mock_steering_cal=true;mock_steering_homed=homed;return true;}
int32_t mcpwm_foc_steering_span_counts(void){return mock_steering_span;}
int32_t mcpwm_foc_steering_safe_span_counts(void){return 648;}
bool mcpwm_foc_encoder_startup_align(bool second) { if(second)return false; diag_motors[0].m_encoder_synced=1u; return true; }
bool mcpwm_foc_encoder_detect(float current,bool second,float *offset,float *ratio,bool *inverted) {
    (void)current; if(second)return false;
    encoder_detect_calls++;
    encoder_detect_saw_motor_model=confs[0].foc_motor_r>0.0f && confs[0].foc_motor_l>0.0f && confs[0].foc_motor_flux_linkage>0.0f;
    if(offset)*offset=0.0f;
    if(ratio)*ratio=4.0f;
    if(inverted)*inverted=false;
    return true;
}
uint32_t mcpwm_foc_get_isr_cycles(void) { return 1234u; }
uint32_t mcpwm_foc_get_isr_cycles_max(void) { return 2345u; }
bool mcpwm_foc_reset_isr_profile(void) { return true; }
void platform_watchdog_get_status(platform_watchdog_status_t *out) { if(out) memset(out,0,sizeof(*out)); }
bool platform_watchdog_boot_was_iwdg(void) { return false; }
void mcpwm_foc_get_irq_epoch(uint32_t *entry,uint32_t *exit){if(entry)*entry=0u;if(exit)*exit=0u;}
uint32_t usart3_rx_error_count(void){return 3u;}
uint32_t usart3_rx_restart_count(void){return 4u;}
uint32_t usart3_forced_recovery_count(void){return 5u;}
void mcpwm_foc_get_isr_profile(mcpwm_foc_isr_profile_t *out) {
    if(!out)return;
    memset(out,0,sizeof(*out));
    out->total_max_cycles=2345u;
    out->detail_sample_count=31u;
    for(uint8_t i=0u;i<6u;++i)out->detail_slot_count[i]=(uint32_t)(i+1u);
    out->steady_isr_count=186u;
    out->slot_sequence_error_count=0u;
    out->fast_hold_svpwm_max_cycles=77u;
    out->profile_revision=0x00030000u;
    out->active_slot_count=6u; out->reset_epoch=7u;
    out->dma_tc_pending_exit_count=9u;
}
bool mcpwm_foc_trace_clear(void) {return true;}
bool mcpwm_foc_trace_freeze(void) {return true;}
void mcpwm_foc_trace_get_meta(mcpwm_foc_trace_meta_t *out){if(out){memset(out,0,sizeof(*out));out->capacity=MCPWM_FOC_TRACE_CAPACITY;out->sample_size=sizeof(mcpwm_foc_trace_sample_t);}}
bool mcpwm_foc_trace_read(uint8_t index,mcpwm_foc_trace_sample_t *out){if(!out||index!=0u)return false;memset(out,0,sizeof(*out));out->pwm_tick=123u;return true;}
void mcpwm_foc_get_adc_sample_diag(bool second,mcpwm_foc_adc_sample_diag_t *out){if(!out)return;memset(out,0,sizeof(*out));out->ccr_a=1000u;out->ccr_b=1200u;out->ccr_c=800u;out->zero_window_counts=800u;out->min_window_counts=700u;out->guard_counts=184u;out->adc_phase_counts=2000u;out->invalid_count=second?2u:1u;out->sector=second?4u:2u;out->window_valid=1u;out->offset_valid=1u;out->driven_offset_valid=1u;out->bridge_settled=1u;}
static mcpwm_foc_step_test_status_t step_mock;
bool mcpwm_foc_step_test_arm(float pre,float step,uint8_t pre_n,uint8_t post_n,bool second){step_mock.sequence++;step_mock.pre_q4=(int16_t)(pre*800.0f);step_mock.step_q4=(int16_t)(step*800.0f);step_mock.active=1u;step_mock.second=second?1u:0u;step_mock.pre_remaining=pre_n;step_mock.post_remaining=post_n;step_mock.step_fired=0u;step_mock.done=0u;return true;}
bool mcpwm_foc_step_test_arm_axis(float pre,float step,uint8_t pre_n,uint8_t post_n,bool second,mcpwm_foc_step_axis_t axis){(void)axis;return mcpwm_foc_step_test_arm(pre,step,pre_n,post_n,second);}
void mcpwm_foc_step_test_get(mcpwm_foc_step_test_status_t *out){if(out)*out=step_mock;}
static mcpwm_foc_relay_status_t relay_mock;
bool mcpwm_foc_relay_start(mcpwm_foc_relay_mode_t mode,bool second,int32_t target,int32_t hyst,uint16_t relay_ma,uint8_t crossings,uint32_t timeout_ms){(void)timeout_ms;memset(&relay_mock,0,sizeof(relay_mock));relay_mock.sequence++;relay_mock.mode=(uint8_t)mode;relay_mock.second=second?1u:0u;relay_mock.target=target;relay_mock.hysteresis=hyst;relay_mock.relay_current_ma=relay_ma;relay_mock.required_crossings=crossings;relay_mock.active=1u;return true;}
void mcpwm_foc_relay_abort(void){relay_mock.active=0u;relay_mock.done=1u;relay_mock.failed=9u;}
void mcpwm_foc_relay_get(mcpwm_foc_relay_status_t *out){if(out)*out=relay_mock;}
float mcpwm_foc_get_erpm_motor(bool second) { return (float)diag_motors[second?1:0].m_rpm; }
void mcpwm_foc_get_current_offsets(int16_t *p0,int16_t *p1,int16_t *dc,bool second){if(p0)*p0=second?2003:1998;if(p1)*p1=second?1997:2001;if(dc)*dc=second?2002:1999;}
uint16_t mcpwm_foc_get_pole_pairs(bool second){return (uint16_t)((confs[second?1:0].si_motor_poles>=2?confs[second?1:0].si_motor_poles:30u)/2u);}
float mcpwm_foc_get_gear_ratio(bool second){float g=confs[second?1:0].si_gear_ratio;return g>0.0f?g:1.0f;}
float mcpwm_foc_get_motor_mechanical_rpm(bool second){return mcpwm_foc_get_erpm_motor(second)/(float)mcpwm_foc_get_pole_pairs(second);}
float mcpwm_foc_get_output_rpm(bool second){return mcpwm_foc_get_motor_mechanical_rpm(second)/mcpwm_foc_get_gear_ratio(second);}
void mcpwm_foc_set_position_user_counts(int32_t v, bool second) {
    const int j=second?1:0;
    if(v<pos_min_user[j])v=pos_min_user[j];
    if(v>pos_max_user[j])v=pos_max_user[j];
    pos_target_user[j]=v;
}
void mcpwm_foc_set_position_user_limits(int32_t lo,int32_t hi,bool second) {
    const int j=second?1:0; pos_min_user[j]=lo;pos_max_user[j]=hi;
    if(pos_target_user[j]<lo)pos_target_user[j]=lo;
    if(pos_target_user[j]>hi)pos_target_user[j]=hi;
}
int32_t mcpwm_foc_get_position_user_counts(bool second){return pos_user[second?1:0];}
int32_t mcpwm_foc_get_position_target_user_counts(bool second){return pos_target_user[second?1:0];}
int32_t mcpwm_foc_get_position_min_user_counts(bool second){return pos_min_user[second?1:0];}
int32_t mcpwm_foc_get_position_max_user_counts(bool second){return pos_max_user[second?1:0];}
void mcpwm_foc_reset_position(bool second){const int j=second?1:0;pos_user[j]=0;pos_target_user[j]=0;}
bool mc_interface_store_configuration_motor(bool second) { store_count[second?1:0]++; return store_ok; }
bool mc_interface_load_configuration_motor(bool second) { (void)second; return load_ok; }
void mc_interface_restore_default_motor(bool second,bool store){mc_configuration c; mcpwm_foc_get_default_configuration(&c,second); confs[second?1:0]=c; if(store)store_count[second?1:0]++;}
uint8_t mcpwm_foc_hall_detect_angle200(int64_t sum_s,int64_t sum_c,uint16_t n){
    if(n<=30u)return 255u;
    float ang=atan2f((float)sum_s,(float)sum_c)*(180.0f/3.14159265358979323846f);
    while(ang<0.0f)ang+=360.0f;
    while(ang>=360.0f)ang-=360.0f;
    uint32_t v=(uint32_t)(ang*(200.0f/360.0f)); if(v>199u)v=199u; return (uint8_t)v;
}
bool mcpwm_foc_hall_detect(float current, bool second, uint8_t table[8]) {
    (void)current;
    static const uint8_t t[8]={255u,83u,17u,50u,150u,117u,183u,255u};
    for (int i=0;i<8;i++) table[i]=t[i];
    for (int i=0;i<8;i++) confs[second?1:0].foc_hall_table[i]=(int8_t)t[i];
    return true;
}

static uint16_t make_frame(const uint8_t *payload, uint16_t len, uint8_t *out) {
    uint16_t i=0u;
    if (len<=255u) { out[i++]=2u; out[i++]=(uint8_t)len; }
    else { out[i++]=3u; out[i++]=(uint8_t)(len>>8); out[i++]=(uint8_t)len; }
    memcpy(out+i,payload,len); i=(uint16_t)(i+len);
    uint16_t crc=vesc_crc16(payload,len); out[i++]=(uint8_t)(crc>>8); out[i++]=(uint8_t)crc; out[i++]=3u;
    return i;
}
static bool decode_tx(uint8_t *payload, uint16_t *len) {
    if (tx_capture_len<5u) return false;
    uint16_t h=0u,n=0u;
    if(tx_capture[0]==2u){h=2u;n=tx_capture[1];}
    else if(tx_capture[0]==3u){h=3u;n=(uint16_t)(((uint16_t)tx_capture[1]<<8)|tx_capture[2]);}
    else return false;
    if(tx_capture_len!=(uint16_t)(h+n+3u) || tx_capture[h+n+2u]!=3u) return false;
    uint16_t crc=(uint16_t)(((uint16_t)tx_capture[h+n]<<8)|tx_capture[h+n+1u]);
    if(crc!=vesc_crc16(tx_capture+h,n)) return false;
    memcpy(payload,tx_capture+h,n);*len=n;return true;
}
static bool transact(const uint8_t *p,uint16_t n,uint8_t *reply,uint16_t *rn){
    uint8_t f[800];uint16_t fn=make_frame(p,n,f);tx_capture_len=0u;
    for(uint16_t i=0;i<fn;i++) (void)vesc_protocol_rx_byte(f[i]);
    vesc_protocol_process_pending();
    if(tx_capture_len==0u){*rn=0u;return true;}
    return decode_tx(reply,rn);
}
static bool pump_until_cmd(uint32_t max_ms,uint8_t cmd,uint8_t *reply,uint16_t *rn){
    tx_capture_len=0u;
    for(uint32_t n=0;n<max_ms;n++){
        tick_ms++;
        vesc_protocol_process_pending();
        vesc_protocol_periodic(tick_ms);
        if(tx_capture_len!=0u && decode_tx(reply,rn) && *rn>0u && reply[0]==cmd)return true;
    }
    *rn=0u;return false;
}
static bool pump_until_reply(uint32_t max_ms,uint8_t *reply,uint16_t *rn){
    return pump_until_cmd(max_ms,COMM_DETECT_HALL_FOC,reply,rn);
}
static void enqueue_only(const uint8_t *p,uint16_t n){
    uint8_t f[800];const uint16_t fn=make_frame(p,n,f);
    for(uint16_t i=0;i<fn;i++) (void)vesc_protocol_rx_byte(f[i]);
}
static int fail(const char *s){fprintf(stderr,"FAIL: %s\n",s);return 1;}
static int nearf32(float a,float b,float eps){return fabsf(a-b)<=eps;}
static int check_values_reply(const uint8_t *r,uint16_t rn,bool second){
    if(rn<60u || r[0]!=COMM_GET_VALUES) return fail(second?"right values header":"local values header");
    int32_t i=1;
    (void)buffer_get_float16(r,1e1f,&i); /* temp mos */
    (void)buffer_get_float16(r,1e1f,&i); /* temp motor */
    const float im=buffer_get_float32(r,1e2f,&i);
    const float iin=buffer_get_float32(r,1e2f,&i);
    const float id=buffer_get_float32(r,1e2f,&i);
    const float iq=buffer_get_float32(r,1e2f,&i);
    const float duty=buffer_get_float16(r,1e3f,&i);
    const float erpm=buffer_get_float32(r,1e0f,&i);
    const float vin=buffer_get_float16(r,1e1f,&i);
    (void)buffer_get_float32(r,1e4f,&i); (void)buffer_get_float32(r,1e4f,&i);
    (void)buffer_get_float32(r,1e4f,&i); (void)buffer_get_float32(r,1e4f,&i);
    (void)buffer_get_int32(r,&i); (void)buffer_get_int32(r,&i);
    const uint8_t fault=r[i++];
    const float pos=buffer_get_float32(r,1e6f,&i);
    const uint8_t idvesc=r[i++];
    (void)buffer_get_float16(r,1e1f,&i); (void)buffer_get_float16(r,1e1f,&i); (void)buffer_get_float16(r,1e1f,&i);
    const float vd=buffer_get_float32(r,1e3f,&i);
    const float vq=buffer_get_float32(r,1e3f,&i);
    if (i < rn) (void)r[i++]; /* timeout/kill */
    if(!nearf32(im,second?2.5f:1.5f,0.011f)) return fail(second?"right Imotor":"local Imotor");
    if(!nearf32(iin,second?1.1f:0.8f,0.011f)) return fail(second?"right Iin":"local Iin");
    if(!nearf32(id,second?0.2f:0.1f,0.011f)) return fail(second?"right Id":"local Id");
    if(!nearf32(iq,second?2.5f:1.5f,0.011f)) return fail(second?"right Iq normalize":"local Iq");
    if(!nearf32(duty,second?0.22f:0.11f,0.002f)) return fail(second?"right duty normalize":"local duty");
    if(!nearf32(erpm,second?321.0f:123.0f,0.5f)) return fail(second?"right ERPM normalize":"local ERPM");
    if(!nearf32(vin,48.1f,0.11f)) return fail("Vin");
    if(fault!=FAULT_CODE_NONE || idvesc!=(second?2u:1u)) return fail(second?"right fault/id":"local fault/id");
    if(!nearf32(pos,second?342.0f:255.0f,0.001f)) return fail(second?"right position normalize":"local VESC position 180-center");
    if(!nearf32(vd,1.2f,0.002f)) return fail(second?"right Vd":"local Vd");
    if(!nearf32(vq,second?5.0f:4.0f,0.002f)) return fail(second?"right Vq normalize":"local Vq");
    return 0;
}
int main(void){
    memset(confs,0,sizeof(confs)); memset(diag_motors,0,sizeof(diag_motors));
    confs[0].motor_type=confs[1].motor_type=MOTOR_TYPE_FOC;
    /* Real hardware default: right motor is mirrored through standard VESC DIR_MULT. */
    confs[1].m_invert_direction=true;
    diag_motors[0].m_iq_target_q4=2400; diag_motors[0].m_iq_set_q4=1600; diag_motors[0].m_iq_q4=800;
    diag_motors[0].m_hall_state=5u; diag_motors[0].m_control_mode=CONTROL_MODE_CURRENT; diag_motors[0].m_state=MC_STATE_RUNNING;
    diag_motors[1].m_iq_target_q4=-2400; diag_motors[1].m_iq_set_q4=-1600; diag_motors[1].m_iq_q4=-800;
    diag_motors[0].m_rpm=50; diag_motors[1].m_rpm=-50;
    diag_motors[1].m_hall_state=3u; diag_motors[1].m_control_mode=CONTROL_MODE_CURRENT; diag_motors[1].m_state=MC_STATE_RUNNING;
    confs[0].l_current_max=confs[1].l_current_max=15.0f;
    confs[0].l_current_min=confs[1].l_current_min=-15.0f;
    confs[0].l_current_max_scale=confs[1].l_current_max_scale=1.0f;
    confs[0].l_current_min_scale=confs[1].l_current_min_scale=1.0f;
    confs[0].l_battery_cut_start=confs[1].l_battery_cut_start=35.0f;
    confs[0].l_battery_cut_end=confs[1].l_battery_cut_end=33.7f;
    confs[0].l_min_erpm=confs[1].l_min_erpm=-15000.0f;
    confs[0].l_max_erpm=confs[1].l_max_erpm=15000.0f;
    confs[0].si_motor_poles=confs[1].si_motor_poles=30u;
    confs[0].si_gear_ratio=confs[1].si_gear_ratio=1.0f;
    confs[0].si_wheel_diameter=confs[1].si_wheel_diameter=0.083f;
    confs[0].si_battery_type=confs[1].si_battery_type=BATTERY_TYPE_LIION_3_0__4_2;
    confs[0].si_battery_cells=confs[1].si_battery_cells=10;
    confs[0].si_battery_ah=confs[1].si_battery_ah=0.0f;
    vesc_protocol_init();
    uint8_t r[800];uint16_t rn=0u; int32_t k=0;
    uint8_t fw[]={COMM_FW_VERSION}; if(!transact(fw,sizeof(fw),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION||r[1]!=6u||r[2]!=0u)return fail("local fw");
    if(strcmp((const char *)&r[3],"motor_left")!=0)return fail("local hardware name");
    uint8_t ping[]={COMM_PING_CAN}; if(!transact(ping,sizeof(ping),r,&rn)||rn!=2u||r[0]!=COMM_PING_CAN||r[1]!=2u)return fail("ping id2");
    {
        const uint8_t magic0=0x48u, magic1=0x42u, ver=2u, op=17u;
        uint8_t gp[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,op,0u};
        if(!transact(gp,sizeof(gp),r,&rn)||rn!=294u||tx_capture[0]!=3u||r[0]!=COMM_CUSTOM_APP_DATA||r[4]!=op||r[5]!=0u)
            return fail("stage1 ISR profile long-frame");
        int32_t pi=6; uint32_t vals[72];
        for(uint8_t z=0u;z<72u;++z)vals[z]=buffer_get_uint32(r,&pi);
        if(vals[0]!=2345u||vals[45]!=31u||vals[46]!=1u||vals[51]!=6u||vals[52]!=186u||
           vals[53]!=0u||vals[54]!=77u||vals[55]!=0x00030000u||vals[56]!=6u||vals[57]!=7u||vals[71]!=9u)
            return fail("stage1 ISR profile ABI values");
    }
    {
        const uint8_t magic0=0x48u, magic1=0x42u, ver=2u, op=24u;
        uint8_t gh[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,op};
        if(!transact(gh,sizeof(gh),r,&rn)||rn!=86u||r[0]!=COMM_CUSTOM_APP_DATA||r[4]!=op||r[5]!=0u)
            return fail("stage2 comms health packet");
        int32_t hi=6; uint32_t hv[20]; for(uint8_t z=0u;z<20u;++z)hv[z]=buffer_get_uint32(r,&hi);
        if(hv[14]!=0u||hv[15]!=0u||hv[16]!=0u) return fail("stage2 host comms health hardware placeholders");
    }
    {
        const uint8_t magic0=0x48u, magic1=0x42u, ver=2u, op=25u;
        uint8_t gp[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,op};
        if(!transact(gp,sizeof(gp),r,&rn)||rn!=46u||r[0]!=COMM_CUSTOM_APP_DATA||r[4]!=op||r[5]!=0u)
            return fail("platform info packet");
        int32_t pi=6;
        if(buffer_get_uint16(r,&pi)!=2u||buffer_get_uint16(r,&pi)!=4u||buffer_get_uint16(r,&pi)!=3u||buffer_get_uint16(r,&pi)!=3u)
            return fail("platform schema values");
    }
    {
        const uint8_t magic0=0x48u,magic1=0x42u,ver=2u;
        uint8_t qa[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,26u};
        if(!transact(qa,sizeof(qa),r,&rn)||rn!=29u||r[4]!=26u||r[5]!=0u)return fail("adc validity packet");
        int32_t ai=6; if(buffer_get_uint16(r,&ai)!=1000u||buffer_get_uint16(r,&ai)!=1200u||buffer_get_uint16(r,&ai)!=800u)return fail("adc ccr values");
        if(buffer_get_uint16(r,&ai)!=800u||buffer_get_uint16(r,&ai)!=700u||buffer_get_uint16(r,&ai)!=184u||buffer_get_uint16(r,&ai)!=2000u)return fail("adc window values");
        if(buffer_get_uint32(r,&ai)!=1u||r[ai++]!=2u||r[ai++]!=1u)return fail("adc validity state");
        uint8_t st[16]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,27u};int32_t si=5;buffer_append_int32(st,500,&si);buffer_append_int32(st,1500,&si);st[si++]=6u;st[si++]=20u;
        if(!transact(st,(uint16_t)si,r,&rn)||rn!=10u||r[4]!=27u||r[5]!=0u)return fail("step arm packet");
        uint8_t ss[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,28u};
        if(!transact(ss,sizeof(ss),r,&rn)||rn!=20u||r[4]!=28u||r[5]!=0u)return fail("step status packet");
        pos_user[0]=1234;pos_target_user[0]=2345;diag_motors[0].m_position_d_proc_filter_q15=-321;
        diag_motors[0].m_control_mode=CONTROL_MODE_POS;diag_motors[0].m_pos_pid_phase_mode=0u;
        confs[0].foc_encoder_inverted=true;confs[0].m_invert_direction=false;diag_motors[0].m_conf=confs[0];
        uint8_t pd[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,30u};
        if(!transact(pd,sizeof(pd),r,&rn)||rn!=22u||r[4]!=30u||r[5]!=0u)return fail("position D state packet");
        int32_t pdi=6;if(buffer_get_int32(r,&pdi)!=1234||buffer_get_int32(r,&pdi)!=2345||buffer_get_int32(r,&pdi)!=-321)return fail("position D state values");
        if(r[pdi++]!=(uint8_t)CONTROL_MODE_POS||r[pdi++]!=0u||r[pdi++]!=1u||r[pdi++]!=0u)return fail("position D state flags");
    }
    uint8_t fwr[]={COMM_FORWARD_CAN,2u,COMM_FW_VERSION}; if(!transact(fwr,sizeof(fwr),r,&rn)||rn<4u||r[0]!=COMM_FW_VERSION)return fail("right fw");
    if(strcmp((const char *)&r[3],"motor_right")!=0)return fail("right hardware name");

    /* Realtime SET_* commands are intentionally coalesced per motor. If several
     * arrive before the main loop runs, only the newest setpoint is meaningful;
     * stale duty/current/rpm commands must not consume the generic request FIFO.
     * Read/config requests remain FIFO elsewhere in this test. */
    {
        uint8_t qd[5]={COMM_SET_DUTY,0,0,0,0}; int32_t qi=1; buffer_append_int32(qd,5000,&qi);
        uint8_t qc[5]={COMM_SET_CURRENT,0,0,0,0}; qi=1; buffer_append_int32(qc,3000,&qi);
        uint8_t qr[5]={COMM_SET_RPM,0,0,0,0}; qi=1; buffer_append_int32(qr,50,&qi);
        uint8_t qp[5]={COMM_SET_POS,0,0,0,0}; qi=1; buffer_append_int32(qp,15000000,&qi);
        enqueue_only(qd,sizeof(qd)); enqueue_only(qc,sizeof(qc)); enqueue_only(qr,sizeof(qr)); enqueue_only(qp,sizeof(qp));
        vesc_protocol_process_pending();
        if(!nearf32(set_pos[0],-27.5f,0.001f))return fail("realtime mailbox latest VESC-tool mapped setpoint");
        if(!nearf32(set_duty[0],0.0f,0.0001f)||!nearf32(set_current[0],0.0f,0.001f)||
           !nearf32(set_rpm[0],0.0f,0.001f))return fail("realtime mailbox stale command applied");
    }

    uint8_t gv[]={COMM_GET_VALUES}; if(!transact(gv,sizeof(gv),r,&rn) || check_values_reply(r,rn,false)) return 1;
    /* Upstream VESC GET_VALUES is request/reply only. The host owns the polling
     * cadence; firmware must not inject unsolicited packets that can be
     * mistaken for replies from another virtual motor. */

    /* VESC Tool can use the SETUP flavor for its dashboard; it is likewise
     * exactly one request -> one reply. */
    {
        uint8_t gvs[]={COMM_GET_VALUES_SETUP};
        if(!transact(gvs,sizeof(gvs),r,&rn)||rn<50u||r[0]!=COMM_GET_VALUES_SETUP)return fail("setup values immediate");
        int32_t si=1;
        (void)buffer_get_float16(r,1e1f,&si); /* temp mos */
        (void)buffer_get_float16(r,1e1f,&si); /* temp motor */
        const float setup_imotor=buffer_get_float32(r,1e2f,&si);
        const float setup_iin=buffer_get_float32(r,1e2f,&si);
        (void)buffer_get_float16(r,1e3f,&si); /* duty */
        (void)buffer_get_float32(r,1e0f,&si); /* ERPM */
        const float speed=buffer_get_float32(r,1e3f,&si);
        (void)buffer_get_float16(r,1e1f,&si); /* Vin */
        const float battery=buffer_get_float16(r,1e3f,&si);
        (void)buffer_get_float32(r,1e4f,&si); (void)buffer_get_float32(r,1e4f,&si);
        (void)buffer_get_float32(r,1e4f,&si); (void)buffer_get_float32(r,1e4f,&si);
        const float distance=buffer_get_float32(r,1e3f,&si);
        const float distance_abs=buffer_get_float32(r,1e3f,&si);
        const float expected_speed=(123.0f/15.0f/60.0f)*0.083f*3.14159265358979323846f;
        const float expected_distance=31.0f*0.083f*3.14159265358979323846f/(3.0f*30.0f);
        if(!nearf32(setup_imotor,4.0f,0.011f)||!nearf32(setup_iin,1.9f,0.011f))
            return fail("setup dual current aggregation");
        if(!nearf32(speed,expected_speed,0.0011f)||!nearf32(distance,expected_distance,0.0011f)||
           !nearf32(distance_abs,expected_distance,0.0011f))return fail("setup speed/distance physical scaling");
        if(battery<0.995f||battery>1.001f)return fail("setup Li-ion battery level");
    }
    /* VESC Tool Set Odometer has no ACK. Verify the selective Setup Values
     * reader returns the requested runtime odometer value afterwards. */
    {
        uint8_t so[5]={COMM_SET_ODOMETER,0,0,0,0}; int32_t oi=1;
        buffer_append_uint32(so,1234u,&oi);
        if(!transact(so,sizeof(so),r,&rn)||rn!=0u)return fail("set odometer no-ack");
        uint8_t gos[5]={COMM_GET_VALUES_SETUP_SELECTIVE,0,0,0,0}; oi=1;
        buffer_append_uint32(gos,(1u<<20),&oi);
        if(!transact(gos,sizeof(gos),r,&rn)||rn!=9u||r[0]!=COMM_GET_VALUES_SETUP_SELECTIVE)return fail("get selective odometer");
        oi=1;
        if(buffer_get_uint32(r,&oi)!=(1u<<20)||buffer_get_uint32(r,&oi)!=1234u)return fail("odometer value");
    }
    /* Power latch sengaja tidak digunakan pada board ini. COMM_SHUTDOWN tetap
     * harus menghasilkan keadaan aman dengan melepas bridge tanpa ACK. */
    {
        diag_motors[0].m_control_mode=CONTROL_MODE_CURRENT;
        uint8_t sh[3]={COMM_SHUTDOWN,0u,0u};
        if(!transact(sh,sizeof(sh),r,&rn)||rn!=0u)return fail("shutdown no-ack");
        if(diag_motors[0].m_control_mode!=CONTROL_MODE_NONE)return fail("shutdown release");
    }

    uint8_t gvr[]={COMM_FORWARD_CAN,2u,COMM_GET_VALUES}; if(!transact(gvr,sizeof(gvr),r,&rn) || check_values_reply(r,rn,true)) return 1;
    /* Upstream VESC 6.00 rotor-position contract. Exercise all seven buttons:
     * Inductance is BLDC detection-only; the six FOC/position traces must be
     * sourced independently and encoded as COMM_ROTOR_POSITION deg*100000. */
    {
        int32_t pi=1;
        diag_motors[0].m_encoder_configured=1u;
        uint8_t ind[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_INDUCTANCE};
        if(!transact(ind,sizeof(ind),r,&rn)||rn!=0u)return fail("set detect inductance");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(tx_capture_len!=0u)return fail("FOC inductance must be detection-only");

        uint8_t enc[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_ENCODER};
        if(!transact(enc,sizeof(enc),r,&rn)||rn!=0u)return fail("set detect encoder");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("encoder rotor packet");
        pi=1; if(buffer_get_int32(r,&pi)!=1250000)return fail("encoder must be mechanical 12.5deg");

        uint8_t obs[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_OBSERVER};
        if(!transact(obs,sizeof(obs),r,&rn)||rn!=0u)return fail("set detect observer");
        diag_motors[0].m_phase=16384u; /* active phase intentionally differs: 90deg */
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("observer rotor packet");
        pi=1; if(buffer_get_int32(r,&pi)!=10000000)return fail("observer must not alias active phase");

        uint8_t pid[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_PID_POS};
        if(!transact(pid,sizeof(pid),r,&rn)||rn!=0u)return fail("set detect pid pos");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("pid position packet");
        pi=1; { const int32_t got=buffer_get_int32(r,&pi); const float exp=(12.5f-MCCONF_STEERING_POS_MIN_DEG)*360.0f/(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG); int32_t d=got-(int32_t)lroundf(exp*100000.0f); if(d<0)d=-d; if(d>2)return fail("pid position steering feedback"); }

        diag_motors[0].m_pos_pid_phase_mode=1u; set_pos[0]=22.5f;
        uint8_t pe[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_PID_POS_ERROR};
        if(!transact(pe,sizeof(pe),r,&rn)||rn!=0u)return fail("set detect pid error");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("pid error packet");
        pi=1; if(buffer_get_int32(r,&pi)!=1000000)return fail("pid error setpoint-now");

        uint8_t oe[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_ENCODER_OBSERVER_ERROR};
        if(!transact(oe,sizeof(oe),r,&rn)||rn!=0u)return fail("set detect obs enc");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("obs-vs-enc packet");
        pi=1; if(buffer_get_int32(r,&pi)!=8750000)return fail("obs-vs-enc must be observer-encoder");

        diag_motors[1].m_phase_hall=32768u; /* 180deg */
        uint8_t oh[]={COMM_FORWARD_CAN,2u,COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_HALL_OBSERVER_ERROR};
        if(!transact(oh,sizeof(oh),r,&rn)||rn!=0u)return fail("set detect obs hall");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("obs-vs-hall packet");
        pi=1; if(buffer_get_int32(r,&pi)!=3000000)return fail("obs-vs-hall must be observer-hall");

        uint8_t sdr[]={COMM_FORWARD_CAN,2u,COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_OBSERVER};
        if(!transact(sdr,sizeof(sdr),r,&rn)||rn!=0u)return fail("set detect right observer");
        tick_ms+=11u; tx_capture_len=0u; vesc_protocol_periodic(tick_ms);
        if(!decode_tx(r,&rn)||rn!=5u||r[0]!=COMM_ROTOR_POSITION)return fail("right observer packet");
        pi=1; if(buffer_get_int32(r,&pi)!=21000000)return fail("right observer independent value");
        uint8_t off[]={COMM_SET_DETECT,(uint8_t)DISP_POS_MODE_NONE};
        if(!transact(off,sizeof(off),r,&rn))return fail("set detect off");
    }
    uint8_t duty[5]={COMM_SET_DUTY,0,0,0,0}; k=1; buffer_append_int32(duty,12500,&k);
    if(!transact(duty,sizeof(duty),r,&rn)||rn!=0u||!nearf32(set_duty[0],0.125f,0.0001f)) return fail("local duty");
    uint8_t dutyr[7]={COMM_FORWARD_CAN,2u,COMM_SET_DUTY,0,0,0,0}; k=3; buffer_append_int32(dutyr,12500,&k);
    if(!transact(dutyr,sizeof(dutyr),r,&rn)||rn!=0u||!nearf32(set_duty[1],0.125f,0.0001f)) return fail("right duty forward unchanged");
    uint8_t cur[7]={COMM_FORWARD_CAN,2u,COMM_SET_CURRENT,0,0,0,0}; k=3; buffer_append_int32(cur,2500,&k);
    if(!transact(cur,sizeof(cur),r,&rn)||rn!=0u)return fail("right current reply");
    if(fabsf(set_current[1]-2.5f)>0.001f||touch_count[1]==0u)return fail("right current forward/ownership");

    /* VESC Tool Commands::setCurrentRel uses int32 x1e5. Verify both the
     * percentage scaling and unchanged virtual-right VESC coordinate semantics. */
    {
        uint8_t rel[5]={COMM_SET_CURRENT_REL,0,0,0,0}; k=1; buffer_append_int32(rel,50000,&k);
        if(!transact(rel,sizeof(rel),r,&rn)||rn!=0u||!nearf32(set_current[0],7.5f,0.002f))
            return fail("set current relative local");
        uint8_t relr[7]={COMM_FORWARD_CAN,2u,COMM_SET_CURRENT_REL,0,0,0,0}; k=3; buffer_append_int32(relr,50000,&k);
        if(!transact(relr,sizeof(relr),r,&rn)||rn!=0u||!nearf32(set_current[1],7.5f,0.002f))
            return fail("set current relative right");
    }

    /* Battery-cut read/write is a standard VESC 6.00 control. First apply a
     * volatile local value, then a stored+forwarded value to both endpoints. */
    {
        uint8_t gbc[]={COMM_GET_BATTERY_CUT};
        if(!transact(gbc,sizeof(gbc),r,&rn)||rn!=9u||r[0]!=COMM_GET_BATTERY_CUT)return fail("get battery cut");
        int32_t bi=1;
        if(!nearf32(buffer_get_float32(r,1e3f,&bi),35.0f,0.002f)||
           !nearf32(buffer_get_float32(r,1e3f,&bi),33.7f,0.002f))return fail("battery cut defaults");

        uint8_t sbc[11]={COMM_SET_BATTERY_CUT}; bi=1;
        buffer_append_float32(sbc,36.5f,1e3f,&bi); buffer_append_float32(sbc,33.0f,1e3f,&bi);
        sbc[bi++]=0u; sbc[bi++]=0u;
        const unsigned b0=store_count[0], b1=store_count[1];
        if(!transact(sbc,(uint16_t)bi,r,&rn)||rn!=1u||r[0]!=COMM_SET_BATTERY_CUT)return fail("set battery cut volatile ack");
        if(store_count[0]!=b0||store_count[1]!=b1||!nearf32(confs[0].l_battery_cut_start,36.5f,0.002f))
            return fail("battery cut volatile semantics");

        bi=1; buffer_append_float32(sbc,37.0f,1e3f,&bi); buffer_append_float32(sbc,33.2f,1e3f,&bi);
        sbc[bi++]=1u; sbc[bi++]=1u;
        if(!transact(sbc,(uint16_t)bi,r,&rn)||rn!=1u||r[0]!=COMM_SET_BATTERY_CUT)return fail("set battery cut stored ack");
        if(store_count[0]!=b0+1u||store_count[1]!=b1+1u||
           !nearf32(confs[0].l_battery_cut_start,37.0f,0.002f)||!nearf32(confs[1].l_battery_cut_end,33.2f,0.002f))
            return fail("battery cut store/forward semantics");
    }

    /* Temporary MC limits are emitted by VESC Tool Setup. The ACK flag must
     * be honored and store=false must not touch EEPROM. */
    {
        uint8_t mt[64]={0}; int32_t ti=0; mt[ti++]=COMM_SET_MCCONF_TEMP;
        mt[ti++]=0u; mt[ti++]=0u; mt[ti++]=1u; mt[ti++]=0u;
        buffer_append_float32_auto(mt,0.55f,&ti); buffer_append_float32_auto(mt,0.75f,&ti);
        buffer_append_float32_auto(mt,-8000.0f,&ti); buffer_append_float32_auto(mt,9000.0f,&ti);
        buffer_append_float32_auto(mt,0.01f,&ti); buffer_append_float32_auto(mt,0.80f,&ti);
        buffer_append_float32_auto(mt,-50.0f,&ti); buffer_append_float32_auto(mt,75.0f,&ti);
        const unsigned before=store_count[0];
        if(!transact(mt,(uint16_t)ti,r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF_TEMP)return fail("mcconf temp ack");
        if(store_count[0]!=before||!nearf32(confs[0].l_current_min_scale,0.55f,0.001f)||
           !nearf32(confs[0].l_current_max_scale,0.75f,0.001f)||!nearf32(confs[0].l_max_erpm,9000.0f,0.5f))
            return fail("mcconf temp apply no-store");

        uint8_t gmt[]={COMM_GET_MCCONF_TEMP};
        if(!transact(gmt,sizeof(gmt),r,&rn)||rn<40u||r[0]!=COMM_GET_MCCONF_TEMP)return fail("get mcconf temp");
        ti=1;
        if(!nearf32(buffer_get_float32_auto(r,&ti),0.55f,0.001f)||!nearf32(buffer_get_float32_auto(r,&ti),0.75f,0.001f))
            return fail("get mcconf temp scales");
        if(!nearf32(buffer_get_float32_auto(r,&ti),-8000.0f,0.5f)||!nearf32(buffer_get_float32_auto(r,&ti),9000.0f,0.5f))
            return fail("get mcconf temp erpm");
    }

    /* NO_STORE must still apply the App Config live and return its own command
     * byte, matching Commands::setAppConfNoStore in VESC Tool. */
    /* Hardware app support must be honest: only UART, ADC, and ADC+UART
     * are accepted. Unsupported PPM/PAS modes canonicalize to UART. */
    {
        app_configuration ac;
        app_vesc_defaults(&ac,1u);
        ac.app_to_use=APP_PPM;
        if(!app_vesc_set_configuration(false,&ac) || app_vesc_get_configuration(false)->app_to_use!=APP_UART)
            return fail("unsupported app mode must canonicalize to UART");
        ac=*app_vesc_get_configuration(false); ac.app_to_use=APP_ADC;
        if(!app_vesc_set_configuration(false,&ac) || app_vesc_get_configuration(false)->app_to_use!=APP_ADC)
            return fail("APP_ADC support");
        ac=*app_vesc_get_configuration(false); ac.app_to_use=APP_ADC_UART;
        if(!app_vesc_set_configuration(false,&ac) || app_vesc_get_configuration(false)->app_to_use!=APP_ADC_UART)
            return fail("APP_ADC_UART support");
        ac=*app_vesc_get_configuration(false); ac.app_to_use=APP_UART;
        if(!app_vesc_set_configuration(false,&ac) || app_vesc_get_configuration(false)->app_to_use!=APP_UART ||
           app_vesc_get_configuration(false)->app_uart_baudrate!=921600u ||
           !app_vesc_get_configuration(false)->permanent_uart_enabled)
            return fail("APP_UART permanent VESC-standard 921600 support");
    }

    {
        app_configuration ac=*app_vesc_get_configuration(false);
        ac.timeout_msec=432u;
        uint8_t ap[700]; ap[0]=COMM_SET_APPCONF_NO_STORE;
        const int32_t an=confgenerator_serialize_appconf(ap+1,&ac);
        if(an<=0||!transact(ap,(uint16_t)(an+1),r,&rn)||rn!=1u||r[0]!=COMM_SET_APPCONF_NO_STORE)
            return fail("appconf no-store ack");
        if(app_vesc_get_configuration(false)->timeout_msec!=432u)return fail("appconf no-store live apply");
    }
    uint8_t rpm[5]={COMM_SET_RPM,0,0,0,0}; k=1;buffer_append_int32(rpm,300,&k);if(!transact(rpm,sizeof(rpm),r,&rn))return fail("local rpm frame");
    if(fabsf(set_rpm[0]-300.0f)>0.001f||touch_count[0]==0u)return fail("local rpm");
    uint8_t posl[5]={COMM_SET_POS,0,0,0,0}; k=1;buffer_append_int32(posl,45000000,&k);
    if(!transact(posl,sizeof(posl),r,&rn)||fabsf(set_pos[0]-(-22.5f))>0.001f)return fail("local VESC Tool position map");
    uint8_t posr[7]={COMM_FORWARD_CAN,2u,COMM_SET_POS,0,0,0,0}; k=3;buffer_append_int32(posr,30000000,&k);
    if(!transact(posr,sizeof(posr),r,&rn)||fabsf(set_pos[1]-30.0f)>0.001f)return fail("right position forward unchanged");
    uint8_t gmr[]={COMM_FORWARD_CAN,2u,COMM_GET_MCCONF};
    if(!transact(gmr,sizeof(gmr),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF) return fail("get right mcconf");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("right mcconf signature");}
    uint8_t gm[]={COMM_GET_MCCONF};
    if(!transact(gm,sizeof(gm),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF) return fail("get mcconf");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("mcconf signature");}
    uint8_t gmd[]={COMM_GET_MCCONF_DEFAULT};
    if(!transact(gmd,sizeof(gmd),r,&rn)||rn<10u||r[0]!=COMM_GET_MCCONF_DEFAULT) return fail("get mcconf default");
    {int32_t mi=1; if(buffer_get_uint32(r,&mi)!=MCCONF_SIGNATURE) return fail("mcconf default signature");}
    {
        uint8_t sm[700]; mc_configuration c=confs[0];
        c.l_current_max=9.0f; c.l_current_min=-11.0f; c.l_abs_current_max=10.0f; c.foc_sensor_mode=FOC_SENSOR_MODE_HALL; c.si_motor_poles=20u; c.si_gear_ratio=5.5f;
        const uint8_t ht[8]={255u,80u,14u,47u,147u,114u,180u,255u};
        for(int q=0;q<8;q++) c.foc_hall_table[q]=(int8_t)ht[q];
        sm[0]=COMM_SET_MCCONF; const int32_t sn=confgenerator_serialize_mcconf(sm+1,&c);
        const unsigned before=store_count[0];
        if(sn<=0 || !transact(sm,(uint16_t)(sn+1),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF) return fail("set mcconf ack");
        if(store_count[0]!=before+1u || !nearf32(confs[0].l_current_max,9.0f,0.01f)) return fail("set mcconf apply/store");
        if(!nearf32(confs[0].l_abs_current_max,MCCONF_L_ABS_CURRENT_MAX,0.01f)) return fail("SET_MCCONF ABS must cover negative current magnitude");
        if(confs[0].si_motor_poles!=20u || !nearf32(confs[0].si_gear_ratio,5.5f,0.01f)) return fail("set mcconf runtime poles/gear");
        for(int q=0;q<8;q++) if((uint8_t)confs[0].foc_hall_table[q]!=ht[q]) return fail("set mcconf hall table");

        /* Standard VESC reset-default workflow is GET_MCCONF_DEFAULT followed
         * by SET_MCCONF. Verify the default reply is independent of active
         * config and can be written/persisted normally. */
        if(!transact(gmd,sizeof(gmd),r,&rn)||r[0]!=COMM_GET_MCCONF_DEFAULT)return fail("get defaults after custom write");
        mc_configuration defc; memset(&defc,0,sizeof(defc));
        if(!confgenerator_deserialize_mcconf(r+1,&defc) || !nearf32(defc.l_current_max,15.0f,0.01f))return fail("default mcconf content");
        sm[0]=COMM_SET_MCCONF; const int32_t dn=confgenerator_serialize_mcconf(sm+1,&defc);
        const unsigned before_def=store_count[0];
        if(dn<=0 || !transact(sm,(uint16_t)(dn+1),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF)return fail("write defaults ack");
        if(store_count[0]!=before_def+1u || !nearf32(confs[0].l_current_max,15.0f,0.01f))return fail("write defaults apply/store");
    }
    {
        uint8_t sm[702]; mc_configuration c=confs[1];
        c.l_current_max=8.0f; c.l_current_min=-8.0f; c.foc_sensor_mode=FOC_SENSOR_MODE_HALL; c.si_motor_poles=14u; c.si_gear_ratio=2.0f;
        const uint8_t ht[8]={255u,82u,16u,49u,149u,116u,182u,255u};
        for(int q=0;q<8;q++) c.foc_hall_table[q]=(int8_t)ht[q];
        sm[0]=COMM_FORWARD_CAN; sm[1]=2u; sm[2]=COMM_SET_MCCONF;
        const int32_t sn=confgenerator_serialize_mcconf(sm+3,&c); const unsigned before=store_count[1];
        if(sn<=0 || !transact(sm,(uint16_t)(sn+3),r,&rn)||rn!=1u||r[0]!=COMM_SET_MCCONF) return fail("set right mcconf ack");
        if(store_count[1]!=before+1u || !nearf32(confs[1].l_current_max,8.0f,0.01f)) return fail("set right mcconf apply/store");
        if(confs[1].si_motor_poles!=14u || !nearf32(confs[1].si_gear_ratio,2.0f,0.01f)) return fail("set right mcconf runtime poles/gear");
        for(int q=0;q<8;q++) if((uint8_t)confs[1].foc_hall_table[q]!=ht[q]) return fail("set right mcconf hall table");
    }
    {
        /* Full signed-int32 long-range position is project-specific and rides
         * inside standard COMM_CUSTOM_APP_DATA. Standard COMM_SET_POS remains
         * VESC single-turn degrees. */
        const uint8_t magic0=0x48u, magic1=0x42u, ver=2u;
        uint8_t cp[16]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,3u};
        k=5; buffer_append_int32(cp,-1000000,&k); buffer_append_int32(cp,1000000,&k);
        if(!transact(cp,(uint16_t)k,r,&rn)||rn!=22u||r[0]!=COMM_CUSTOM_APP_DATA||r[5]!=0u) return fail("custom set limits");
        int32_t ci=6; (void)buffer_get_int32(r,&ci); (void)buffer_get_int32(r,&ci);
        if(buffer_get_int32(r,&ci)!=-1000000 || buffer_get_int32(r,&ci)!=1000000) return fail("custom limits values");

        uint8_t ct[12]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,4u};
        k=5; buffer_append_int32(ct,-345678,&k);
        if(!transact(ct,(uint16_t)k,r,&rn)||r[5]!=0u||pos_target_user[0]!=-345678) return fail("custom left target");
        if(touch_count[0]==0u) return fail("custom target ownership");

        uint8_t ctr[14]={COMM_FORWARD_CAN,2u,COMM_CUSTOM_APP_DATA,magic0,magic1,ver,4u};
        k=7; buffer_append_int32(ctr,456789,&k);
        if(!transact(ctr,(uint16_t)k,r,&rn)||r[5]!=0u||pos_target_user[1]!=456789) return fail("custom right target");

        uint8_t dg[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,1u};
        if(!transact(dg,sizeof(dg),r,&rn)||rn<78u||r[0]!=COMM_CUSTOM_APP_DATA||r[4]!=1u||r[5]!=0u) return fail("custom diag");

        if(r[6]!=1u || r[10]!=5u) return fail("custom diag id/hall");
        int32_t di=14;
        if(buffer_get_int32(r,&di)!=3000) return fail("custom diag iq target 3A");

        /* Project tuning extension remains inside COMM_CUSTOM_APP_DATA. */
        {
            uint8_t gt[]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,6u};
            if(!transact(gt,sizeof(gt),r,&rn)||rn!=30u||r[4]!=6u||r[5]!=0u) return fail("custom get tuning");
            uint8_t st[40]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,7u}; int32_t si=5;
            const uint16_t tv[10]={1300,1400,900,1000,950,2000,3,60,1,2};
            for(int z=0;z<10;z++)buffer_append_uint16(st,tv[z],&si);
            buffer_append_uint16(st,3277u,&si); st[si++]=1u;
            unsigned before_store=store_count[0];
            if(!transact(st,(uint16_t)si,r,&rn)||rn!=30u||r[4]!=7u||r[5]!=0u) return fail("custom set tuning");
            if(diag_motors[0].m_kpq_q11!=1300u||diag_motors[0].m_kid_q16!=1000u||diag_motors[0].m_kps_q11!=950u||diag_motors[0].m_kpp_q11!=60u) return fail("custom tuning apply");
            if(diag_motors[0].m_telem_current_filter_q16!=3277u||store_count[0]!=(before_store+1u)) return fail("custom tuning filter/store");
            uint8_t idt[20]={COMM_CUSTOM_APP_DATA,magic0,magic1,ver,8u}; si=5; buffer_append_int32(idt,300,&si); buffer_append_int32(idt,0,&si);
            if(!transact(idt,(uint16_t)si,r,&rn)||rn!=6u||diag_motors[0].m_openloop_id_target_q4!=240||diag_motors[0].m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE) return fail("custom id test");
            si=5; buffer_append_int32(idt,0,&si); buffer_append_int32(idt,0,&si);
            if(!transact(idt,(uint16_t)si,r,&rn)||diag_motors[0].m_control_mode!=CONTROL_MODE_NONE) return fail("custom id release");
        }
    }
    /* VESC Tool exact Hall-detect contract: command 28 is a blocking worker
     * operation, so request handling returns immediately with no response while
     * ordinary request/reply traffic remains alive. Detect does not apply/store. */
    { uint8_t sd[2]={COMM_SET_DETECT,0u}; (void)transact(sd,sizeof(sd),r,&rn); }
    uint8_t dh[5]={COMM_DETECT_HALL_FOC,0,0,0,0}; k=1; buffer_append_int32(dh,1000,&k);
    const unsigned st0=store_count[0]; int8_t oldhall0[8]; memcpy(oldhall0,confs[0].foc_hall_table,8);
    if(!transact(dh,sizeof(dh),r,&rn)||rn!=0u)return fail("local hall detect must start asynchronously");
    for(int z=0;z<100;z++){tick_ms++;vesc_protocol_periodic(tick_ms);} /* worker active */
    { uint8_t gv[1]={COMM_GET_VALUES}; if(!transact(gv,1u,r,&rn)||rn==0u||r[0]!=COMM_GET_VALUES)return fail("request/reply stalled during hall detect"); }
    if(!pump_until_reply(14000u,r,&rn)||rn!=10u||r[0]!=COMM_DETECT_HALL_FOC||r[9]!=0u)return fail("local VESC Tool hall reply");
    if(store_count[0]!=st0||memcmp(oldhall0,confs[0].foc_hall_table,8)!=0)return fail("detect must not apply/store mcconf");
    {
        const uint8_t expect[8]={255u,17u,50u,83u,117u,150u,183u,255u};
        for(int hi=0;hi<8;hi++){
            if(hi==0||hi==7){if(r[1+hi]!=255u)return fail("local hall invalid states");}
            else {int d=(int)r[1+hi]-(int)expect[hi]; if(d<0)d=-d; if(d>100)d=200-d; if(d>1)return fail("local hall table wire order");}
        }
    }

    uint8_t dhr[7]={COMM_FORWARD_CAN,2u,COMM_DETECT_HALL_FOC,0,0,0,0}; k=3; buffer_append_int32(dhr,1000,&k);
    const unsigned st1=store_count[1]; int8_t oldhall1[8]; memcpy(oldhall1,confs[1].foc_hall_table,8);
    if(!transact(dhr,sizeof(dhr),r,&rn)||rn!=0u)return fail("right hall detect must start asynchronously");
    if(!pump_until_reply(14000u,r,&rn)||rn!=10u||r[0]!=COMM_DETECT_HALL_FOC||r[9]!=0u)return fail("right VESC Tool hall reply");
    if(store_count[1]!=st1||memcmp(oldhall1,confs[1].foc_hall_table,8)!=0)return fail("right detect must not apply/store mcconf");

    /* Stock VESC Tool motor-model commands must also be functional, not only
     * Detect-All. They are cooperative on bare metal and restore MC config. */
    {
        for(int m=0;m<2;m++){
            confs[m].foc_motor_r=plant_r[m];
            confs[m].foc_motor_l=plant_l[m];
            confs[m].foc_motor_flux_linkage=plant_flux[m];
        }
        const unsigned sb0=store_count[0], sb1=store_count[1];
        const mc_configuration keep0=confs[0], keep1=confs[1];

        uint8_t rl[1]={COMM_DETECT_MOTOR_R_L};
        if(!transact(rl,sizeof(rl),r,&rn)||rn!=0u)return fail("local measure R/L must start asynchronously");
        if(!pump_until_cmd(6000u,COMM_DETECT_MOTOR_R_L,r,&rn)||rn!=13u)return fail("local measure R/L reply timeout");
        { int32_t ri=1; const float rr=buffer_get_float32(r,1e6f,&ri); const float lu=buffer_get_float32(r,1e3f,&ri); const float du=buffer_get_float32(r,1e3f,&ri);
          if(fabsf(rr-plant_r[0])>0.003f||fabsf(lu-plant_l[0]*1e6f)>30.0f||fabsf(du)>1.0f)return fail("local measure R/L values/units"); }
        if(store_count[0]!=sb0||memcmp(&confs[0],&keep0,sizeof(keep0))!=0)return fail("local measure R/L must restore/no-store");

        uint8_t rlr[3]={COMM_FORWARD_CAN,2u,COMM_DETECT_MOTOR_R_L};
        if(!transact(rlr,sizeof(rlr),r,&rn)||rn!=0u)return fail("right measure R/L start");
        if(!pump_until_cmd(6000u,COMM_DETECT_MOTOR_R_L,r,&rn)||rn!=13u)return fail("right measure R/L reply timeout");
        { int32_t ri=1; const float rr=buffer_get_float32(r,1e6f,&ri); const float lu=buffer_get_float32(r,1e3f,&ri);
          if(fabsf(rr-plant_r[1])>0.003f||fabsf(lu-plant_l[1]*1e6f)>30.0f)return fail("right measure R/L values/units"); }
        if(store_count[1]!=sb1||memcmp(&confs[1],&keep1,sizeof(keep1))!=0)return fail("right measure R/L must restore/no-store");

        uint8_t fl[17]={0}; int32_t fi=0; fl[fi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE;
        buffer_append_float32(fl,3.0f,1e3f,&fi); buffer_append_float32(fl,600.0f,1e3f,&fi);
        buffer_append_float32(fl,0.30f,1e3f,&fi); buffer_append_float32(fl,plant_r[0],1e6f,&fi);
        if(fi!=17||!transact(fl,(uint16_t)fi,r,&rn)||rn!=0u)return fail("local measure flux26 start");
        if(!pump_until_cmd(7000u,COMM_DETECT_MOTOR_FLUX_LINKAGE,r,&rn)||rn!=5u)return fail("local measure flux26 reply timeout");
        { int32_t ri=1; const float lam=buffer_get_float32(r,1e7f,&ri); if(fabsf(lam-plant_flux[0])>0.0015f)return fail("local flux26 FOC compatibility value"); }
        if(store_count[0]!=sb0||memcmp(&confs[0],&keep0,sizeof(keep0))!=0)return fail("flux26 must restore/no-store");

        uint8_t fo[23]={0}; fi=0; fo[fi++]=COMM_FORWARD_CAN; fo[fi++]=2u; fo[fi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
        buffer_append_float32(fo,3.0f,1e3f,&fi); buffer_append_float32(fo,1800.0f,1e3f,&fi);
        buffer_append_float32(fo,0.30f,1e3f,&fi); buffer_append_float32(fo,plant_r[1],1e6f,&fi);
        buffer_append_float32(fo,plant_l[1],1e8f,&fi);
        if(fi!=23||!transact(fo,(uint16_t)fi,r,&rn)||rn!=0u)return fail("right flux57 start");
        if(!pump_until_cmd(6000u,COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,r,&rn)||rn!=14u)return fail("right flux57 reply timeout");
        { int32_t ri=1; const float lam=buffer_get_float32(r,1e7f,&ri); const float eo=buffer_get_float32(r,1e6f,&ri); const float er=buffer_get_float32(r,1e6f,&ri); const uint8_t ei=r[ri++];
          if(fabsf(lam-plant_flux[1])>0.0015f||fabsf(eo+1.0f)>0.001f||fabsf(er+1.0f)>0.001f||ei!=0u)return fail("right flux57 value/wire encoder tuple"); }
        if(store_count[1]!=sb1||memcmp(&confs[1],&keep1,sizeof(keep1))!=0)return fail("flux57 must restore/no-store");
    }

    /* Project contract: standalone VESC Tool Detect Encoder is translated to
     * the real steering hard-stop span calibration. Detect-All must not call
     * that hard-stop sweep. */
    confs[0].m_sensor_port_mode=SENSOR_PORT_MODE_ABI;
    confs[0].foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
    confs[0].m_encoder_counts=1024; confs[0].si_motor_poles=8u; confs[0].foc_encoder_ratio=4.0f;
    {
        const unsigned hw_before=steering_span_hw_calls;
        const unsigned enc_before=encoder_detect_calls;
        uint8_t de[5]={COMM_DETECT_ENCODER,0,0,0,0}; int32_t ei=1; buffer_append_int32(de,3000,&ei);
        if(!transact(de,sizeof(de),r,&rn)||rn!=10u||r[0]!=COMM_DETECT_ENCODER)return fail("standalone encoder detect reply");
        if(encoder_detect_calls!=enc_before+1u)return fail("standalone encoder must run electrical ABI detect first");
        if(steering_span_hw_calls!=hw_before+1u || mock_steering_span!=683)return fail("standalone encoder must measure hardware span after electrical detect");
    }

    /* VESC Tool Detect All (COMM 58): non-blocking on this bare-metal target,
     * app outputs gated, dual Hall detection applied/stored, and an int16
     * non-negative result is returned after both virtual motors complete. */
    {
        uint8_t da[22]={0}; int32_t ai=0;
        da[ai++]=COMM_DETECT_APPLY_ALL_FOC;
        da[ai++]=1u;
        buffer_append_float32(da,50.0f,1e3f,&ai);
        buffer_append_float32(da,-8.0f,1e3f,&ai);
        buffer_append_float32(da,8.0f,1e3f,&ai);
        buffer_append_float32(da,250.0f,1e3f,&ai);
        buffer_append_float32(da,2500.0f,1e3f,&ai);
        const unsigned b0=store_count[0], b1=store_count[1];
        const unsigned hw_before_all=steering_span_hw_calls;
        if(ai!=22 || !transact(da,(uint16_t)ai,r,&rn) || rn!=0u)return fail("detect all must start asynchronously");
        for(int z=0;z<100;z++){tick_ms++;vesc_protocol_periodic(tick_ms);}
        { uint8_t qv[1]={COMM_GET_VALUES}; if(!transact(qv,1u,r,&rn)||rn==0u||r[0]!=COMM_GET_VALUES)return fail("request/reply stalled during detect all"); }
        if(!pump_until_cmd(45000u,COMM_DETECT_APPLY_ALL_FOC,r,&rn)||rn!=3u)return fail("detect all result timeout");
        { int32_t ri=1; if(buffer_get_int16(r,&ri)<0)return fail("detect all returned failure"); }
        if(store_count[0]!=(b0+1u)||store_count[1]!=(b1+1u))return fail("detect all must persist both motor configs");
        if(steering_span_hw_calls!=hw_before_all)return fail("detect all encoder must not sweep hardware span");
        if(!encoder_detect_saw_motor_model)return fail("detect all must identify R/L/flux before encoder sensor stage");
        if(mock_steering_span!=683 || !mock_steering_cal || !mock_steering_homed)return fail("detect all must preserve existing steering span");
        if(confs[0].m_sensor_port_mode!=SENSOR_PORT_MODE_ABI ||
           confs[0].foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER ||
           fabsf(confs[0].foc_encoder_offset-0.0f)>0.01f ||
           fabsf(confs[0].foc_encoder_ratio-4.0f)>0.01f ||
           confs[0].foc_encoder_inverted || confs[0].si_motor_poles!=8u)
            return fail("detect all left encoder apply");
        {
            int valid=0;
            for(int h=0;h<8;h++)if((uint8_t)confs[1].foc_hall_table[h]!=255u)valid++;
            if(valid!=6 || confs[1].m_sensor_port_mode!=SENSOR_PORT_MODE_HALL ||
               confs[1].foc_sensor_mode!=FOC_SENSOR_MODE_HALL)
                return fail("detect all right hall apply");
        }
        if(!nearf32(confs[0].l_in_current_min,-8.0f,0.01f)||!nearf32(confs[1].l_in_current_max,8.0f,0.01f))return fail("detect all input-current apply");
        if(!nearf32(confs[0].foc_openloop_rpm,250.0f,0.01f)||!nearf32(confs[1].foc_sl_erpm,2500.0f,0.01f))return fail("detect all FOC setup fields");
        for(int m=0;m<2;m++){
            if(fabsf(confs[m].foc_motor_r-plant_r[m])>0.003f)return fail("detect all R identification");
            if(fabsf(confs[m].foc_motor_l-plant_l[m])>0.00003f)return fail("detect all L identification");
            if(fabsf(confs[m].foc_motor_flux_linkage-plant_flux[m])>0.0015f){printf("flux m=%d got=%f exp=%f R=%f L=%f\n",m,confs[m].foc_motor_flux_linkage,plant_flux[m],confs[m].foc_motor_r,confs[m].foc_motor_l);return fail("detect all flux identification");}
            if(fabsf(confs[m].foc_current_kp-confs[m].foc_motor_l*1000.0f)>0.002f)return fail("detect all VESC Kp=L/tc");
            if(fabsf(confs[m].foc_current_ki-confs[m].foc_motor_r*1000.0f)>0.2f)return fail("detect all VESC Ki=R/tc");
        }
        uint8_t ena[6]={COMM_APP_DISABLE_OUTPUT,0u,0u,0u,0u,0u};
        if(!transact(ena,sizeof(ena),r,&rn))return fail("re-enable app output");
    }
    /* LEFT can also be commissioned as Hall. Detect-All must keep the motor
     * model-first order and must not touch the steering-span calibration. */
    {
        confs[0].m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
        confs[0].foc_sensor_mode=FOC_SENSOR_MODE_HALL;
        const unsigned hw_before=steering_span_hw_calls; const int32_t span_before=mock_steering_span;
        uint8_t da[22]={0}; int32_t ai=0;
        da[ai++]=COMM_DETECT_APPLY_ALL_FOC; da[ai++]=1u;
        buffer_append_float32(da,50.0f,1e3f,&ai);
        buffer_append_float32(da,-8.0f,1e3f,&ai); buffer_append_float32(da,8.0f,1e3f,&ai);
        buffer_append_float32(da,250.0f,1e3f,&ai); buffer_append_float32(da,2500.0f,1e3f,&ai);
        if(!transact(da,(uint16_t)ai,r,&rn)||rn!=0u)return fail("detect all left Hall start");
        if(!pump_until_cmd(50000u,COMM_DETECT_APPLY_ALL_FOC,r,&rn)||rn!=3u)return fail("detect all left Hall timeout");
        {int32_t ri=1;if(buffer_get_int16(r,&ri)<0)return fail("detect all left Hall result");}
        if(confs[0].m_sensor_port_mode!=SENSOR_PORT_MODE_HALL || confs[0].foc_sensor_mode!=FOC_SENSOR_MODE_HALL)return fail("detect all left Hall apply");
        {int valid=0;for(int h=0;h<8;h++)if((uint8_t)confs[0].foc_hall_table[h]!=255u)valid++;if(valid!=6)return fail("detect all left Hall table");}
        if(steering_span_hw_calls!=hw_before || mock_steering_span!=span_before)return fail("left Hall detect all must not touch steering span");
    }

    /* Detect-All failure must be transactional. A motor that does not build
     * d-axis current still allows Hall sweep visibility, but R/L identification
     * must reject it and preserve the last-known-good stored configuration. */
    {
        const mc_configuration keep0=confs[0], keep1=confs[1];
        const unsigned sb0=store_count[0], sb1=store_count[1];
        plant_fail_rl=true;
        uint8_t da[22]={0}; int32_t ai=0;
        da[ai++]=COMM_DETECT_APPLY_ALL_FOC; da[ai++]=1u;
        buffer_append_float32(da,50.0f,1e3f,&ai);
        buffer_append_float32(da,-8.0f,1e3f,&ai); buffer_append_float32(da,8.0f,1e3f,&ai);
        buffer_append_float32(da,250.0f,1e3f,&ai); buffer_append_float32(da,2500.0f,1e3f,&ai);
        if(!transact(da,(uint16_t)ai,r,&rn)||rn!=0u)return fail("detect all failure start");
        if(!pump_until_cmd(40000u,COMM_DETECT_APPLY_ALL_FOC,r,&rn)||rn!=3u)return fail("detect all failure result timeout");
        {int32_t ri=1;if(buffer_get_int16(r,&ri)>=0)return fail("detect all false success on dead current plant");}
        plant_fail_rl=false;
        if(store_count[0]!=sb0||store_count[1]!=sb1)return fail("detect all failure must not store");
        if(fabsf(confs[0].foc_motor_r-keep0.foc_motor_r)>1e-7f||
           fabsf(confs[1].foc_motor_flux_linkage-keep1.foc_motor_flux_linkage)>1e-7f||
           memcmp(confs[0].foc_hall_table,keep0.foc_hall_table,8)!=0||
           memcmp(confs[1].foc_hall_table,keep1.foc_hall_table,8)!=0)
            return fail("detect all failure rollback");
    }
    {
        uint8_t th[]={COMM_TERMINAL_CMD,'h','e','l','p'};
        if(!transact(th,sizeof(th),r,&rn)||rn<8u||r[0]!=COMM_PRINT||memcmp(r+1,"Commands:",9u)!=0)
            return fail("terminal help framing");
        uint8_t tf[]={COMM_FORWARD_CAN,2u,COMM_TERMINAL_CMD_SYNC,'f','w'};
        if(!transact(tf,sizeof(tf),r,&rn)||r[0]!=COMM_PRINT||rn<12u||memcmp(r+1,"motor_right",11u)!=0)
            return fail("terminal sync/right framing");
        diag_motors[0].m_fault=FAULT_CODE_ABS_OVER_CURRENT;
        diag_motors[1].m_fault=FAULT_CODE_UNDER_VOLTAGE;
        diag_motors[0].m_control_mode=CONTROL_MODE_CURRENT;
        diag_motors[1].m_control_mode=CONTROL_MODE_SPEED;
        const unsigned cf0=clear_faults_count;
        uint8_t tc[]={COMM_TERMINAL_CMD,'f','a','u','l','t','s','_','c','l','e','a','r'};
        if(!transact(tc,sizeof(tc),r,&rn)||r[0]!=COMM_PRINT||clear_faults_count!=cf0+1u||
           diag_motors[0].m_fault!=FAULT_CODE_NONE||diag_motors[1].m_fault!=FAULT_CODE_NONE||
           diag_motors[0].m_control_mode!=CONTROL_MODE_NONE||diag_motors[1].m_control_mode!=CONTROL_MODE_NONE)
            return fail("terminal all-fault reset/release contract");
    }
    {
        diag_motors[0].m_control_mode=CONTROL_MODE_CURRENT;
        uint8_t rb[]={COMM_REBOOT};
        if(!transact(rb,sizeof(rb),r,&rn)||rn!=0u||diag_motors[0].m_control_mode!=CONTROL_MODE_NONE)
            return fail("reboot release-before-reset contract");
    }
    printf("VESC_PROTOCOL_HOST_PASS fw=6.00 can=2 current_rel=ok battery_cut=rw mcconf_temp=rw app_nostore=ok odometer=ok shutdown=release reboot=ok hall=ok values=ok mcconf=rw\n");
    return 0;
}
