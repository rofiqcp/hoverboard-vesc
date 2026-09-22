#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/foc_math.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
extern uint8_t enable;
extern volatile uint8_t motorRunReq;
extern volatile int pwml;
extern volatile int pwmr;
extern volatile int32_t positionCommandL;
extern volatile int32_t positionCommandR;
extern int16_t curL_phaA, curL_phaB, curL_DC;
extern int16_t curR_phaB, curR_phaC, curR_DC;
extern int16_t batVoltage;

void DMA1_Channel1_IRQHandler(void);

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static int fail(const char *s){fprintf(stderr,"FAIL %s\n",s);return 1;}
static uint32_t legacy_ms=1u;
static uint32_t outer_ms=1u;
static uint32_t sim_pwm_frames=0u;
static uint32_t sim_outer_frames=0u;
static void legacy_sync(void){mcpwm_foc_housekeeping_non_isr(legacy_ms);legacy_ms+=5u;}
static void sim_isr_step(void){
    mcpwm_foc_adc_int_handler();
    if(++sim_outer_frames>=16u){
        sim_outer_frames=0u;
        mcpwm_foc_outer_control_non_isr(outer_ms++);
    }
    if(++sim_pwm_frames>=80u){sim_pwm_frames=0u;legacy_sync();}
}
static void set_hall(GPIO_TypeDef *port,uint16_t pu,uint16_t pv,uint16_t pw,uint8_t h){
    port->IDR|=(uint32_t)(pu|pv|pw);
    if(h&4u)port->IDR&=~(uint32_t)pu;
    if(h&2u)port->IDR&=~(uint32_t)pv;
    if(h&1u)port->IDR&=~(uint32_t)pw;
}
static void set_halls(uint8_t l,uint8_t r){
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,l);
    set_hall(GPIOC,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,r);
}
static uint32_t ramp_frames_for_delta(uint32_t delta_rpm,uint16_t ramp_rpm_s){
    if(ramp_rpm_s==0u)return 32u;
    const uint64_t frames=((uint64_t)delta_rpm*(uint64_t)PWM_FREQ+
                           (uint64_t)ramp_rpm_s-1u)/(uint64_t)ramp_rpm_s;
    return (uint32_t)(frames+(uint64_t)PWM_FREQ/20u+32u);
}
static void use_legacy_hall_fixture(void){
    /* These regression cases predate the production LEFT encoder. Override only
     * the feedback-selection fields directly so PI defaults remain untouched. */
    m_motor_1.m_conf.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    m_motor_1.m_conf.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
    m_motor_1.m_encoder_synced=0u;
    m_motor_2.m_conf.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    m_motor_2.m_conf.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
}

int main(void){
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();

    /* MODE 1 follows VESC FOC duty architecture: duty limits modulation while
     * the inner current PI remains active. It must never bypass l_current_max. */
    ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    legacy_sync();
    sim_isr_step(); sim_isr_step();
    if(m_motor_1.m_iq_target_q4!=m_motor_1.m_current_limit_q4)return fail("mode1 left current-limit target");
    if(m_motor_2.m_iq_target_q4!=-m_motor_2.m_current_limit_q4)return fail("mode1 right current-limit target");
    if(abs(m_motor_1.m_vq)>1440 || abs(m_motor_2.m_vq)>1440)return fail("mode1 10pct modulation ceiling");
    if(m_motor_1.m_vd!=0 || m_motor_2.m_vd!=0)return fail("mode1 Id target must stay zero");
    mc_configuration duty_conf=m_motor_1.m_conf; duty_conf.l_current_max=2.0f; duty_conf.l_current_min=-2.0f;
    mcpwm_foc_set_configuration(&duty_conf,false);
    mcpwm_foc_set_duty(0.20f,false);
    for(unsigned i=0;i<MCCONF_FOC_CONTROL_DIV;i++)sim_isr_step();
    if(abs(m_motor_1.m_iq_target_q4)>1600)return fail("mode1 must honor 2A mcconf current limit");
    if(abs(m_motor_1.m_vq)>2880)return fail("mode1 20pct modulation ceiling");
    mcpwm_foc_set_mode_command(VLT_MODE,0,false,SVPWM_OPENLOOP_RPM_DEFAULT,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("mode1 STOP must release/free-run");

    /* Regression: production profile is 15 A motor / 15 A DC-link with a
     * VESC-style 0.02 duty ramp. A 95%% command must ramp to 950 permille while
     * retaining the 15 A current authority that was accidentally left at 1 A
     * during bench-safe tests. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();
    mc_configuration duty15=m_motor_1.m_conf;
    duty15.l_current_max=15.0f; duty15.l_current_min=-15.0f;
    duty15.l_in_current_max=15.0f; duty15.l_in_current_min=-15.0f;
    duty15.l_max_duty=MCCONF_L_MAX_DUTY; duty15.m_duty_ramp_step=0.02f;
    mcpwm_foc_set_configuration(&duty15,false);
    mcpwm_foc_set_duty(1.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    curL_phaA=curL_phaB=curL_DC=0;
    for(int i=0;i<180;i++)sim_isr_step();
    const int16_t production_duty_limit=(int16_t)(MCCONF_L_MAX_DUTY*1000.0f+0.5f);
    if(m_motor_1.m_duty_set_permille!=production_duty_limit)
        return fail("15A duty command must clamp to production duty ceiling");
    if(m_motor_1.m_current_limit_q4!=15*FOC_CURRENT_Q4_PER_A)return fail("production duty current authority must be 15A");
    if(m_motor_1.m_input_current_max_q4!=15*FOC_CURRENT_Q4_PER_A)return fail("production input current limit must be 15A");
    if(abs(m_motor_1.m_iq_set_q4)>15*FOC_CURRENT_Q4_PER_A)return fail("95pct duty exceeds 15A motor current envelope");
    if(abs(m_motor_1.m_vq)>MCCONF_FOC_DUTY_VOLTAGE_MAX)return fail("95pct duty exceeds voltage ceiling");

    /* Verify free-run is electrical high impedance, not merely Vq=0: after the
     * control mode is released the corresponding timer MOE must turn off. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    legacy_sync();
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2004;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR & TIM_BDTR_MOE)==0u || (RIGHT_TIM->BDTR & TIM_BDTR_MOE)==0u)
        return fail("mode1 RUN must enable both bridges");
    pwml=0; pwmr=0; motorRunReq=0u;
    legacy_sync();
    DMA1_Channel1_IRQHandler();
    DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR & TIM_BDTR_MOE)!=0u || (RIGHT_TIM->BDTR & TIM_BDTR_MOE)!=0u)
        return fail("free-run release must disable MOE/high impedance");

    /* MODE 2: legacy command is mechanical RPM. The active setpoint follows
     * the configured electrical-RPM ramp converted through the active pole-pair
     * count; regression timing is derived from the runtime ramp. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();
    ctrlModReq=SPD_MODE; pwml=50; pwmr=-50;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_speed_target_rpm!=50)return fail("mode2 target must be 50 mechanical RPM");
    if(m_motor_1.m_speed_set_rpm!=0)return fail("mode2 active speed must start ramped from measured speed");
    if(m_motor_1.m_iq_set_q4!=0)return fail("mode2 must not create Iq reference");
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++)sim_isr_step();
    if(m_motor_1.m_speed_set_rpm!=50)return fail("mode2 speed ramp must reach 50 RPM");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 must remain speed control while running");

    /* Other speed commands use the same physical mechanical-RPM scale and the
     * same ramp. Check 100 RPM and a direction reversal to -50 RPM. */
    pwml=100; pwmr=-100;
    legacy_sync();
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++)sim_isr_step();
    if(m_motor_1.m_speed_target_rpm!=100 || m_motor_1.m_speed_set_rpm!=100)
        return fail("mode2 100 RPM scaling/ramp");
    pwml=-50; pwmr=50;
    legacy_sync();
    for(uint32_t i=0;i<ramp_frames_for_delta(150u,m_motor_1.m_speed_ramp_rpm_s);i++)sim_isr_step();
    if(m_motor_1.m_speed_target_rpm!=-50 || m_motor_1.m_speed_set_rpm!=-50)
        return fail("mode2 reverse -50 RPM ramp");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 reverse must stay speed mode");

    /* STOP: target becomes zero, active setpoint ramps down gradually. It must
     * NOT release on the first zero command. Stop Vq is gently limited. */
    pwml=0; pwmr=0;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_speed_target_rpm!=0)return fail("mode2 STOP target zero");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP must ramp before release");
    if(abs(m_motor_1.m_speed_set_rpm)<=5)return fail("mode2 STOP ramp must not jump to zero");
    for(uint32_t i=0;i<(uint32_t)PWM_FREQ/5u;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP released too early");
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("mode2 STOP must release in low-speed zone");
    if(m_motor_1.m_speed_integrator!=0 || m_motor_1.m_iq_target_q4!=0)
        return fail("mode2 release must reset speed PID and command zero Iq");
    if(m_motor_1.m_speed_set_rpm!=0 || m_motor_1.m_speed_target_rpm!=0 ||
       m_motor_1.m_iq_target_q4!=0 || m_motor_1.m_iq_set_q4!=0 ||
       (LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)
        return fail("mode2 released speed state");

    /* VESC boundary uses electrical RPM. Derive every mechanical expectation
     * from the canonical configured pole-pair count rather than legacy fixtures. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; set_halls(3u,3u);
    mcpwm_foc_set_pid_speed(15000.0f,false);
    if(m_motor_1.m_speed_target_rpm!=(15000/(int)MCCONF_POLE_PAIRS_LEFT))
        return fail("LEFT 15000 ERPM conversion incorrect");
    mcpwm_foc_set_pid_speed(15000.0f,true);
    if(m_motor_2.m_speed_target_rpm!=(15000/(int)MCCONF_POLE_PAIRS_RIGHT))
        return fail("RIGHT 15000 ERPM conversion incorrect");
    mcpwm_foc_release_motor(false); mcpwm_foc_release_motor(true);

    const float vesc_erpm_50=50.0f*(float)MCCONF_POLE_PAIRS_LEFT;
    mcpwm_foc_set_pid_speed(vesc_erpm_50,false);
    mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_speed_target_rpm!=50)return fail("VESC ERPM target conversion");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("VESC speed mode entry");
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++){ if((i%1000u)==0u)mcpwm_foc_vesc_override_touch(false); sim_isr_step(); }
    if(m_motor_1.m_speed_set_rpm!=50)return fail("VESC speed ramp reach target");
    mcpwm_foc_set_pid_speed(0.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("VESC zero ERPM must ramp before release");
    if(m_motor_1.m_speed_target_rpm!=0)return fail("VESC zero ERPM target");
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++){ if((i%1000u)==0u)mcpwm_foc_vesc_override_touch(false); sim_isr_step(); }
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE || m_motor_1.m_iq_target_q4!=0 || m_motor_1.m_iq_set_q4!=0)
        return fail("VESC zero ERPM must release below configured threshold");
    mcpwm_foc_set_pid_speed(vesc_erpm_50,false);
    mcpwm_foc_vesc_override_touch(false);
    m_motor_1.m_hall_initialized=1u;
    m_motor_1.m_hall_ref_rpm=50;
    m_motor_1.m_hall_ticks=0u;
    mc_values vals;
    mcpwm_foc_get_values(&vals,false);
    if(fabsf(vals.rpm-vesc_erpm_50)>0.1f)return fail("mc_values.rpm must be ERPM");

    /* Low-speed VESC regression: preserve fractional mechanical RPM after
     * converting 50 ERPM through the configured pole-pair count. */
    mcpwm_foc_init(); use_legacy_hall_fixture();
    mcpwm_foc_set_pid_speed(50.0f,false);
    mcpwm_foc_vesc_override_touch(false);
    const int32_t q16_50_erpm=(int32_t)((50.0f/(float)MCCONF_POLE_PAIRS_LEFT)*65536.0f+0.5f);
    if(abs(m_motor_1.m_speed_target_rpm_q16-q16_50_erpm)>2)return fail("50 ERPM fractional Q16 target");
    m_motor_1.m_hall_initialized=1u; m_motor_1.m_hall_direction=1;
    m_motor_1.m_hall_ref_rpm=3; m_motor_1.m_hall_ticks=0u;
    /* Hall telemetry is intentionally the integer mechanical reference snapshot,
     * so 3 RPM at 15 pole-pairs reports 45 ERPM. The command path above retains
     * the requested 50 ERPM fraction in Q16 without inventing Hall precision. */
    if(fabsf(mcpwm_foc_get_erpm_motor(false)-
             (3.0f*(float)MCCONF_POLE_PAIRS_LEFT))>0.05f)
        return fail("low-speed Hall reference telemetry");

    /* VESC speed-ramp zero means no ramp. It must not silently become 1 RPM/s,
     * and position derivative timers must start with no phantom elapsed tick. */
    mcpwm_foc_init(); use_legacy_hall_fixture();
    if(m_motor_1.m_position_dt_ticks!=0u || m_motor_1.m_position_proc_dt_ticks!=0u)
        return fail("position D timer reset must start at zero");
    mc_configuration noramp=m_motor_1.m_conf;
    noramp.s_pid_ramp_erpms_s=0.0f;
    noramp.s_pid_min_erpm=75.0f;
    mcpwm_foc_set_configuration(&noramp,false);
    if(m_motor_1.m_speed_release_erpm_q16!=(75u<<16))
        return fail("s_pid_min_erpm must stay exact electrical ERPM");
    mcpwm_foc_set_pid_speed(200.0f,false);
    mcpwm_foc_outer_control_non_isr(outer_ms++);
    mcpwm_foc_outer_control_non_isr(outer_ms++);
    if(m_motor_1.m_speed_ramp_rpm_s!=0u || m_motor_1.m_speed_set_ramp_q16!=m_motor_1.m_speed_target_rpm_q16)
        return fail("VESC zero speed ramp must apply target directly");

    /* Stock VESC SET_CURRENT scaling: 3.00 A must become the exact Iq target. */
    mcpwm_foc_init(); use_legacy_hall_fixture();
    mcpwm_foc_set_current(3.0f,false);
    if(m_motor_1.m_iq_target_q4!=2400)return fail("VESC 3A Iq target scaling");

    /* MODE 3 scaling remains exact: 50 cA = 0.50 A, 1500 cA = 15 A. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    legacy_sync();
    ctrlModReq=TRQ_MODE; pwml=1500; pwmr=-1500;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_iq_target_q4!=12000)return fail("TRQ 1500cA = 15A scaling");
    if(m_motor_2.m_iq_target_q4!=-12000)return fail("TRQ right 15A internal mirror scaling");

    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    legacy_sync();
    ctrlModReq=TRQ_MODE; pwml=50; pwmr=-50;
    legacy_sync();
    for(int i=0;i<1000;i++)sim_isr_step();
    if(m_motor_1.m_iq_target_q4!=400)return fail("TRQ 50cA target scaling");
    if(m_motor_1.m_iq_set_q4!=400)return fail("TRQ slew must reach 0.50A");
    if(m_motor_1.m_id_set_q4!=0)return fail("TRQ Id must remain zero");
    if(m_motor_1.m_vq<=0)return fail("TRQ PI must generate positive Vq");

    /* MODE 3 STOP: no current-brake state. Ramp Iq reference to zero and then
     * release/high-impedance so the wheel keeps free-running. */
    motorRunReq=0u; pwml=0; pwmr=0;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_control_mode==CONTROL_MODE_CURRENT_BRAKE)return fail("TRQ STOP must not brake");
    if(m_motor_1.m_iq_target_q4!=0)return fail("TRQ STOP target must be zero");
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT)return fail("TRQ STOP must slew current before release");
    for(int i=0;i<1200;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("TRQ STOP must release after Iq ramp");
    if(m_motor_1.m_iq_set_q4!=0 || m_motor_1.m_iq_target_q4!=0)return fail("TRQ release current state");
    for(int i=0;i<20;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("TRQ STOP must stay released");

    /* MODE 4 unchanged: sensorless Id current, Iq=0, 2 -> 2 A and 6 A safety clamp. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    legacy_sync();
    ctrlModReq=SVPWM_MODE; pwml=2; pwmr=-2;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_id_set_q4>=1600)return fail("mode4 Id must slew, not step");
    for(uint32_t i=0u;i<((uint32_t)PWM_FREQ*SVPWM_ALIGN_MS/1000u)+32u;i++)sim_isr_step();
    if(m_motor_1.m_openloop_id_target_q4!=1600)return fail("mode4 2A target");
    if(m_motor_1.m_id_set_q4!=1600)return fail("mode4 Id ramp reaches 2A");
    if(m_motor_1.m_iq_set_q4!=0)return fail("mode4 Iq target zero");
    if(m_motor_1.m_phase!=m_motor_1.m_phase_openloop)return fail("mode4 synthetic phase offset");
    ctrlModReq=SVPWM_MODE; pwml=10; pwmr=-10;
    legacy_sync();
    sim_isr_step();
    if(m_motor_1.m_openloop_id_target_q4!=SVPWM_MAX_ID_A*FOC_CURRENT_Q4_PER_A)return fail("mode4 Id safety clamp");

    /* Standard VESC open-loop APIs are distinct from the legacy SVPWM utility:
     * OPENLOOP_CURRENT rotates signed Iq at electrical RPM; OPENLOOP_PHASE applies
     * signed Id at a fixed electrical phase. No pole-pair multiplication here. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; set_halls(3u,3u);
    m_motor_1.m_openloop_phase_acc_q32=0u; m_motor_1.m_phase_openloop=0u;
    mcpwm_foc_set_openloop_current(1.0f,600.0f,false); mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_OPENLOOP ||
       m_motor_1.m_iq_target_q4!=FOC_CURRENT_Q4_PER_A || m_motor_1.m_id_set_q4!=0 ||
       m_motor_1.m_openloop_speed_q16!=(600<<16)) return fail("VESC openloop current set semantics");
    for(int i=0;i<160;i++)sim_isr_step();
    if(m_motor_1.m_phase_openloop<6500u || m_motor_1.m_phase_openloop>6610u)
        return fail("VESC openloop electrical RPM integration");
    if(m_motor_1.m_iq_set_q4!=FOC_CURRENT_Q4_PER_A || m_motor_1.m_id_set_q4!=0)
        return fail("VESC openloop current axis changed in control loop");
    mcpwm_foc_set_openloop_phase(-1.0f,90.0f,false); mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_OPENLOOP_PHASE ||
       m_motor_1.m_id_set_q4!=-FOC_CURRENT_Q4_PER_A || m_motor_1.m_iq_set_q4!=0 ||
       m_motor_1.m_phase_openloop<16380u || m_motor_1.m_phase_openloop>16388u)
        return fail("VESC openloop fixed-phase signed Id semantics");

    /* RIGHT Hall direction regression. Honor the configured VESC Hall majority
     * filter before applying the edge debounce assertion. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; ctrlModReq=VLT_MODE; pwml=1;pwmr=-1;set_halls(3u,3u);
    legacy_sync();
    if(!m_motor_2.m_conf.m_invert_direction)return fail("right default direction mirror config");
    for(int i=0;i<100;i++)sim_isr_step();
    set_halls(2u,2u);
    for(uint16_t i=0u;i<(uint16_t)m_motor_2.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;i++)sim_isr_step();
    if(m_motor_2.m_hall_direction!=-1)return fail("right reverse Hall direction");
    for(uint32_t i=0u;i<MCCONF_HALL_TIMEOUT_TICKS+100u;i++)sim_isr_step();
    if(m_motor_2.m_hall_interp_active!=0u)return fail("Hall interpolation low-speed disable");

    /* V13 runtime tunables start from the proven V12 fixed-point values. */
    mcpwm_foc_init(); use_legacy_hall_fixture();
    if(m_motor_1.m_kpq_q11!=MCCONF_FOC_CURRENT_KP_Q11 || m_motor_1.m_kiq_q16!=MCCONF_FOC_CURRENT_KI_Q16)
        return fail("V13 Q current PI defaults");
    if(m_motor_1.m_kpd_q11!=MCCONF_FOC_ID_KP_Q11 || m_motor_1.m_kid_q16!=MCCONF_FOC_ID_KI_Q16)
        return fail("V13 D current PI defaults");
    if(m_motor_1.m_kps_q11!=MCCONF_SPEED_KP_Q11 || m_motor_1.m_kis_q16!=MCCONF_SPEED_KI_Q16 || m_motor_1.m_kds_q11!=0u)
        return fail("V13 speed PID defaults");
    if(m_motor_1.m_kpp_q11!=MCCONF_POSITION_KP_Q11 || m_motor_1.m_kip_q16!=MCCONF_POSITION_KI_Q16 || m_motor_1.m_kdp_q11!=MCCONF_POSITION_KD_Q11)
        return fail("V13 position PID defaults");

    /* Requested 2026-09 current/telemetry/position regression. Standard VESC
     * COMM_SET_CURRENT scaling is mA/1000, so 1.000 A must become exactly 800
     * Q4 units with A2BIT_CONV=50. Closed-loop current must also be allowed to
     * use the configured full-safe EFeru modulation rather than the historical 80% cap. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; motorRunReq=0u; set_halls(3u,3u);
    legacy_sync();
    mc_configuration oneamp=m_motor_1.m_conf;
    oneamp.l_current_max=1.0f; oneamp.l_current_min=-1.0f; oneamp.l_max_duty=1.0f;
    oneamp.foc_current_filter_const=0.10f;
    mcpwm_foc_set_configuration(&oneamp,false);
    if(abs((int)m_motor_1.m_telem_current_filter_q16-6554)>2)return fail("VESC foc_current_filter_const mapping");
    mcpwm_foc_vesc_timeout_configure(false,0u,0.0f);
    /* Low-side phase current is not observable while the bridge is released.
     * A bogus OFF measurement must never seed the next torque reference. */
    m_motor_1.m_iq_q4=(int16_t)(-5*FOC_CURRENT_Q4_PER_A);
    mcpwm_foc_set_current(1.0f,false); mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_iq_set_q4!=FOC_CURRENT_Q4_PER_A ||
       m_motor_1.m_iq_set_ramp_q16!=((int32_t)FOC_CURRENT_Q4_PER_A<<16))
        return fail("SET_CURRENT must use command directly, never OFF measured Iq");
    if(m_motor_1.m_iq_target_q4!=FOC_CURRENT_Q4_PER_A)return fail("VESC SET_CURRENT 1A exact Q4 scaling");
    curL_phaA=curL_phaB=curL_DC=0;
    for(int i=0;i<18000;i++)sim_isr_step();
    if(abs(m_motor_1.m_vq)<=((MCCONF_FOC_VOLTAGE_MAX*8)/10))return fail("1A current PI still hard-capped at 80pct modulation");
    if(abs(m_motor_1.m_vq)>MCCONF_FOC_DUTY_VOLTAGE_MAX)return fail("1A current PI exceeds EFeru full-safe modulation");

    /* EFeru hoverboard current-sense contract: one ampere is exactly 50 ADC
     * counts and the driven polarity is offset-ADC. Prove the real DMA/ISR path,
     * not only conversion helpers: LEFT uses rlA/rlB/dcl and RIGHT rrB/rrC/dcr. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; motorRunReq=0u; set_halls(3u,3u);
    legacy_sync();
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2000;i++)DMA1_Channel1_IRQHandler();
    mcpwm_foc_vesc_timeout_configure(false,0u,0.0f); mcpwm_foc_vesc_timeout_configure(true,0u,0.0f);
    mcpwm_foc_set_current(0.10f,false); mcpwm_foc_vesc_override_touch(false);
    mcpwm_foc_set_current(0.10f,true);  mcpwm_foc_vesc_override_touch(true);
    /* First powered transition now has an explicit common-mode pre-settle
     * window before the 80-sample zero-vector baseline. Exercise the complete
     * production sequence instead of assuming the historical 80-frame settle. */
    const int bridge_cal_frames =
        (int)MCCONF_BRIDGE_PRESETTLE_SAMPLES +
        (int)MCCONF_BRIDGE_SETTLE_SAMPLES + 2;
    for(int i=0;i<bridge_cal_frames;i++)DMA1_Channel1_IRQHandler();
    /* Mean/validity finalization intentionally stays in the 5-ms housekeeping. */
    legacy_sync();
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u || (RIGHT_TIM->BDTR&TIM_BDTR_MOE)==0u)return fail("current-scale bridge setup");
    if(m_motor_1.m_bridge_settle_ticks!=0u || m_motor_2.m_bridge_settle_ticks!=0u)return fail("current-scale bridge settle");
    adc_buffer.rlA=1950; adc_buffer.rlB=2050; adc_buffer.dcl=1950;
    adc_buffer.rrB=1950; adc_buffer.rrC=2050; adc_buffer.dcr=1950;
    /* Phase shunts are intentionally consumed only in each motor's scheduler
     * slot (LEFT=0, RIGHT=1). Observe one complete control supercycle instead
     * of assuming both motors publish phase samples in the same PWM frame. */
    int left_current_slot_seen=0, right_current_slot_seen=0;
    for(int i=0;i<(int)MCCONF_FOC_CONTROL_DIV+2;i++){
        DMA1_Channel1_IRQHandler();
        if(curL_phaA!=0 || curL_phaB!=0){
            if(curL_phaA!=50 || curL_phaB!=-50 || curL_DC!=50)
                return fail("EFeru left offset-ADC 50count/A mapping");
            left_current_slot_seen=1;
        }
        if(curR_phaB!=0 || curR_phaC!=0){
            if(curR_phaB!=50 || curR_phaC!=-50 || curR_DC!=50)
                return fail("EFeru right offset-ADC 50count/A mapping");
            right_current_slot_seen=1;
        }
    }
    if(!left_current_slot_seen || !right_current_slot_seen)
        return fail("EFeru current scheduler slots not observed");
    if(A2BIT_CONV!=50 || FOC_CURRENT_Q4_PER_A!=800)return fail("EFeru current unit 50count/A Q4=800/A");
    mcpwm_foc_release_motor(false); mcpwm_foc_release_motor(true);

    /* Hard realtime VESC deadline: even if legacy enable remains high and main
     * housekeeping is never called, the ADC ISR must remove MOE when the VESC
     * command deadline expires. This is the fail-safe for a deadlocked parser or
     * main loop while the 16-kHz current ISR is still alive. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2000;i++)DMA1_Channel1_IRQHandler();
    mcpwm_foc_vesc_timeout_configure(false,0u,0.0f);
    mcpwm_foc_set_current(0.10f,false); mcpwm_foc_vesc_override_touch(false);
    for(int i=0;i<100;i++)DMA1_Channel1_IRQHandler();
    mcpwm_foc_outer_control_non_isr(outer_ms++);
    for(int i=0;i<8;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u)return fail("watchdog setup bridge must be active");
    mcpwm_foc_vesc_timeout_configure(false,2u,0.0f);
    for(int i=0;i<40;i++)DMA1_Channel1_IRQHandler(); /* no main/housekeeping */
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)return fail("ISR VESC timeout must clear MOE with main stalled");
    if(mcpwm_foc_vesc_command_live(false))return fail("expired VESC deadline must not remain source-live");
    for(int i=0;i<20;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)return fail("legacy enable must not bypass owned VESC timeout");
    mcpwm_foc_vesc_override_clear(false); enable=0u; motorRunReq=0u;

    /* Stable bridge-OFF telemetry uses the qualified high-impedance baseline so
     * VESC Tool can still observe real near-zero current while stopped. It must
     * stay bounded and must not be replaced by a synthetic all-zero packet. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; motorRunReq=0u; set_halls(3u,3u);
    legacy_sync();
    /* OFF baseline is intentionally qualified only after the Hall estimator has
     * independently declared the rotor stationary for at least one second. */
    m_motor_1.m_rpm=0; m_motor_2.m_rpm=0;
    m_motor_1.m_hall_ticks=PWM_FREQ; m_motor_2.m_hall_ticks=PWM_FREQ;
    adc_buffer.rlA=2263; adc_buffer.rlB=2281; adc_buffer.dcl=1925;
    /* Cold boot spends the first 2000 DMA frames on startup ADC offset
     * calibration. OFF telemetry then requires one full stationary second and
     * 256 LEFT diagnostic samples, which arrive only in slot 2 of the six-slot
     * FOC supercycle. Exercise those production timings explicitly. */
    for(int i=0;i<2000;i++)DMA1_Channel1_IRQHandler();
    const int off_diag_frames =
        (int)MCCONF_OFF_TELEM_SETTLE_SAMPLES +
        256*(int)MCCONF_FOC_CONTROL_DIV + 12;
    for(int i=0;i<off_diag_frames;i++)DMA1_Channel1_IRQHandler();
    legacy_sync();
    if(!m_motor_1.m_off_offset_valid)return fail("OFF diagnostic offset did not calibrate");
    adc_buffer.rlA=2251; adc_buffer.rlB=2293; adc_buffer.dcl=1915;
    for(int i=0;i<120;i++)DMA1_Channel1_IRQHandler();
    legacy_sync();
    mc_values offv; mcpwm_foc_get_values(&offv,false);
    if(m_motor_1.m_state!=MC_STATE_OFF || (LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)return fail("released bridge state regression");
    if(!isfinite(offv.current_in)||!isfinite(offv.id)||!isfinite(offv.iq)||!isfinite(offv.current_motor))
        return fail("standard OFF current telemetry must stay finite");
    if(fabsf(offv.current_in)>0.20f || fabsf(offv.id)>0.20f || fabsf(offv.iq)>0.20f || fabsf(offv.current_motor)>0.20f)
        return fail("standard OFF current telemetry exceeds qualified near-zero bound");
    /* Upstream VESC FOC semantics force phase-derived motor/Id/Iq telemetry
     * to zero while CONTROL_MODE_NONE. Only the independent DC-link current
     * remains observable with the bridge released. */
    if(fabsf(offv.id)>0.001f || fabsf(offv.iq)>0.001f || fabsf(offv.current_motor)>0.001f)
        return fail("standard OFF phase-current telemetry must be zero");

    /* One Hall count is four mechanical degrees at 15 pole-pairs. Kp=0.060
     * would request about 0.24 A with a 1 A motor-current limit, but the
     * hardware-safe position-output cap must clamp that request deterministically. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; motorRunReq=0u; set_halls(3u,3u);
    legacy_sync();
    mc_configuration pos1=m_motor_1.m_conf; pos1.l_current_max=1.0f; pos1.l_current_min=-1.0f;
    pos1.p_pid_kp=0.060f; pos1.p_pid_ki=0.0f; pos1.p_pid_kd=0.0f; pos1.p_pid_kd_filter=0.20f;
    mcpwm_foc_set_configuration(&pos1,false);
    mcpwm_foc_vesc_timeout_configure(false,0u,0.0f);
    mcpwm_foc_set_position_counts(1,false); mcpwm_foc_vesc_override_touch(false);
    curL_phaA=curL_phaB=curL_DC=0;
    for(int i=0;i<100;i++)sim_isr_step();
    {
        const int32_t cap_q4=((int32_t)FOC_CURRENT_Q4_PER_A*
                              (int32_t)MCCONF_POSITION_CURRENT_MAX_MA)/1000;
        const int32_t effective_cap=cap_q4<m_motor_1.m_current_limit_q4?
                                    cap_q4:m_motor_1.m_current_limit_q4;
        if(m_motor_1.m_iq_target_q4<=0)
            return fail("position PID must request positive current for positive count error");
        if(m_motor_1.m_iq_target_q4>effective_cap)
            return fail("position PID/current safety cap");
    }
    if(m_motor_1.m_position_kd_filter_q16<13000u || m_motor_1.m_position_kd_filter_q16>13200u)return fail("position D filter config mapping");

    /* A stock VESC mc_configuration has one common current Kp/Ki. Writing it
     * through VESC Tool intentionally applies that pair to both D and Q, while
     * the custom KPQ/KIQ/KPD/KID terminal parameters may separate them later. */
    mc_configuration tune=m_motor_1.m_conf;
    tune.foc_current_kp=1.0f; tune.foc_current_ki=100.0f;
    tune.s_pid_kp=0.01234f; tune.s_pid_ki=0.02345f; tune.s_pid_kd=0.00067f;
    tune.p_pid_kp=3.0f; tune.p_pid_ki=0.1f; tune.p_pid_kd=0.05f;
    mcpwm_foc_set_configuration(&tune,false);
    if(m_motor_1.m_kpq_q11!=1536u || m_motor_1.m_kpd_q11!=1536u)return fail("VESC current Kp maps D/Q");
    if(abs((int)m_motor_1.m_kiq_q16-461)>1 || m_motor_1.m_kiq_q16!=m_motor_1.m_kid_q16)return fail("VESC current Ki maps D/Q");
    if(m_motor_1.m_kps_q11!=1234u || m_motor_1.m_kis_q16!=2345u || m_motor_1.m_kds_q11!=67u)return fail("VESC speed PID mapping");
    mcpwm_foc_sync_tuning_to_conf(false);
    if(fabsf(m_motor_1.m_conf.s_pid_kp-0.01234f)>0.000006f || fabsf(m_motor_1.m_conf.s_pid_ki-0.02345f)>0.000006f)return fail("VESC speed PID 1e-5 readback precision");
    if(m_motor_1.m_kpp_q11!=3000u || m_motor_1.m_kip_q16!=100u || m_motor_1.m_kdp_q11!=50u)return fail("VESC position PID mapping");

    /* Legacy mode 5 is multi-turn Hall position: 15 pole pairs * 6 sectors =
     * 90 counts/revolution (4 mechanical degrees/count). User-facing positive
     * targets are mirrored internally for motor 2. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();
    ctrlModReq=5u; positionCommandL=10; positionCommandR=10; pwml=0; pwmr=0;
    legacy_sync();
    for(int i=0;i<120;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_POS || m_motor_2.m_control_mode!=CONTROL_MODE_POS)return fail("mode5 position control entry");
    if(m_motor_1.m_position_target_counts!=10 || m_motor_2.m_position_target_counts!=-10)return fail("mode5 right user sign normalization");
    if(m_motor_1.m_iq_target_q4<=0 || m_motor_2.m_iq_target_q4>=0)return fail("position PID must command signed Iq");

    m_motor_1.m_position_counts=12345; m_motor_2.m_position_counts=-12345;
    mcpwm_foc_reset_position(false); mcpwm_foc_reset_position(true);
    if(m_motor_1.m_position_counts!=0 || m_motor_2.m_position_counts!=0)return fail("reset position count");
    if(m_motor_1.m_position_integrator!=0 || m_motor_2.m_position_integrator!=0)return fail("reset position PID");

    m_motor_1.m_position_min_counts=-5; m_motor_1.m_position_max_counts=5;
    mcpwm_foc_set_position_counts(100,false);
    if(m_motor_1.m_position_target_counts!=5)return fail("position PMAX clamp");
    mcpwm_foc_set_position_counts(-100,false);
    if(m_motor_1.m_position_target_counts!=-5)return fail("position PMIN clamp");

    /* Standard VESC COMM_SET_POS stays single-turn electrical degrees and never
     * aliases the project's long-range count coordinate. Its normalized PID
     * output scales to the configured motor-current envelope exactly upstream. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=0u; motorRunReq=0u; set_halls(3u,3u);
    legacy_sync();
    for(int i=0;i<6;i++)sim_isr_step();
    {
        const float now=mcpwm_foc_get_phase_motor(false);
        float target=now+60.0f; if(target>=360.0f)target-=360.0f;
        mcpwm_foc_set_pid_pos(target,false); mcpwm_foc_vesc_override_touch(false);
        /* POSITION outer PID is serviced outside the ADC ISR at 1 kHz. */
        for(int i=0;i<80;i++)sim_isr_step();
        if(!m_motor_1.m_pos_pid_phase_mode || m_motor_1.m_control_mode!=CONTROL_MODE_POS)return fail("VESC position Hall phase mode");
        if(m_motor_1.m_iq_target_q4<=0)return fail("VESC position positive phase error must request positive Iq");
        { const int32_t abs_iq=m_motor_1.m_iq_target_q4<0?-m_motor_1.m_iq_target_q4:m_motor_1.m_iq_target_q4;
          const int32_t abs_ref=m_motor_1.m_iq_set_q4<0?-m_motor_1.m_iq_set_q4:m_motor_1.m_iq_set_q4;
          const int32_t count_cap=((int32_t)FOC_CURRENT_Q4_PER_A*(int32_t)MCCONF_POSITION_CURRENT_MAX_MA)/1000;
          if(abs_iq>m_motor_1.m_current_limit_q4 || abs_ref>m_motor_1.m_current_limit_q4)
              return fail("VESC position configured current envelope");
          if(abs_iq<=count_cap)return fail("standard VESC position must not use custom count cap"); }
        mcpwm_foc_set_pid_pos(mcpwm_foc_get_phase_motor(false),false);
        for(int i=0;i<80;i++)sim_isr_step();
        if(m_motor_1.m_iq_target_q4!=0)return fail("VESC position zero error must produce zero torque");
    }
    mcpwm_foc_release_motor(false);
    m_motor_1.m_phase=0u; m_motor_1.m_position_counts=0;
    mcpwm_foc_set_pid_pos(40.0f,false);
    if(!m_motor_1.m_pos_pid_phase_mode)return fail("VESC position must use live phase mode");
    { const float setdeg=(float)m_motor_1.m_pos_pid_set_phase*(360.0f/65536.0f);
      if(fabsf(setdeg-40.0f)>0.02f)return fail("VESC position electrical target precision"); }
    if(m_motor_1.m_position_target_counts!=0)return fail("VESC SET_POS must not alter custom Hall-count target");
    mc_values pvals;
    m_motor_1.m_phase=(uint16_t)lroundf(40.0f*(65536.0f/360.0f));
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.position-40.0f)>0.02f)return fail("VESC rotor position telemetry electrical degrees");

    /* VESC current semantics: current_motor is SIGN(Ibus)*sqrt(Id^2+Iq^2),
     * current_in is battery/DC-bus current, and Id/Iq remain separate. */
    m_motor_1.m_control_mode=CONTROL_MODE_CURRENT; m_motor_1.m_driven_offset_valid=1u;
    m_motor_1.m_driven_offset_calibrating=0u; m_motor_1.m_bridge_settle_ticks=0u; LEFT_TIM->BDTR|=TIM_BDTR_MOE;
    m_motor_1.m_telem_sum_id_q4=600; m_motor_1.m_telem_sum_iq_q4=800; m_motor_1.m_telem_sum_imotor_q4=1000; m_motor_1.m_telem_sum_ibus_counts=-50; m_motor_1.m_telem_avg_samples=1u;
    m_motor_1.m_id_q4=600;   /* raw/control 0.75 A */
    m_motor_1.m_iq_q4=800;   /* raw/control 1.00 A */
    m_motor_1.m_current_in_counts=-50;
    m_motor_1.m_id_telem_q4=600;   /* monitoring LPF output */
    m_motor_1.m_iq_telem_q4=800;
    m_motor_1.m_current_in_telem_counts=-50; /* EFeru polarity => +1.00 A Ibus */
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.current_motor-1.25f)>0.01f)return fail("VESC Imotor signed DQ magnitude positive");
    if(fabsf(pvals.current_in-1.00f)>0.01f)return fail("VESC Ibattery positive scaling");
    if(fabsf(pvals.id-0.75f)>0.01f || fabsf(pvals.iq-1.00f)>0.01f)return fail("VESC Id/Iq scaling");
    m_motor_1.m_current_in_counts=50;
    m_motor_1.m_current_in_telem_counts=50;
    m_motor_1.m_telem_sum_id_q4=600; m_motor_1.m_telem_sum_iq_q4=800; m_motor_1.m_telem_sum_imotor_q4=1000; m_motor_1.m_telem_sum_ibus_counts=50; m_motor_1.m_telem_avg_samples=1u;
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.current_motor+1.25f)>0.01f || fabsf(pvals.current_in+1.00f)>0.01f)return fail("VESC regenerative current signs");

    /* Standard VESC Ah/Wh telemetry must be live counters, not constant zero.
     * Integrate exactly 100 ms at +1 A and then 100 ms at -1 A. The default
     * test bus is 40.0 V, so expected draw/charge are 0.1 As and 4.0 Ws. */
    mcpwm_foc_init(); use_legacy_hall_fixture();
    m_motor_1.m_current_in_counts=-50;
    mcpwm_foc_energy_update(1000u);
    mcpwm_foc_energy_update(1100u);
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.amp_hours-(0.1f/3600.0f))>1e-7f)return fail("VESC Ah drawn integration");
    if(fabsf(pvals.watt_hours-(pvals.amp_hours*pvals.v_in))>2e-6f)return fail("VESC Wh drawn integration");
    if(pvals.amp_hours_charged!=0.0f || pvals.watt_hours_charged!=0.0f)return fail("VESC charged counters must start zero");
    m_motor_1.m_current_in_counts=50;
    mcpwm_foc_energy_update(1200u);
    mcpwm_foc_get_values(&pvals,false);
    if(fabsf(pvals.amp_hours_charged-(0.1f/3600.0f))>1e-7f)return fail("VESC Ah charged integration");
    if(fabsf(pvals.watt_hours_charged-(pvals.amp_hours_charged*pvals.v_in))>2e-6f)return fail("VESC Wh charged integration");

    /* Standard VESC drivetrain setup is runtime configuration: motor poles
     * change electrical<->mechanical conversion; gear ratio changes output
     * shaft speed only. COMM_SET_RPM and mc_values.rpm stay ERPM. */
    {
        mc_configuration dyn=m_motor_1.m_conf;
        dyn.si_motor_poles=20u; dyn.si_gear_ratio=5.0f;
        mcpwm_foc_set_configuration(&dyn,false);
        mcpwm_foc_set_pid_speed(300.0f,false);
        if(mcpwm_foc_get_pole_pairs(false)!=10u)return fail("runtime pole-pair 20 poles -> 10pp");
        if(labs((long)m_motor_1.m_speed_target_rpm_q16-(long)(30*65536))>2)return fail("300 ERPM -> 30 motor RPM @10pp");
        m_motor_1.m_hall_initialized=1u; m_motor_1.m_hall_ref_rpm=100; m_motor_1.m_hall_ticks=0u;
        if(fabsf(mcpwm_foc_get_motor_mechanical_rpm(false)-100.0f)>0.01f)return fail("runtime motor mechanical RPM");
        if(fabsf(mcpwm_foc_get_output_rpm(false)-20.0f)>0.01f)return fail("gearbox output RPM");
        dyn.si_gear_ratio=1.0f; mcpwm_foc_set_configuration(&dyn,false);
        if(fabsf(mcpwm_foc_get_output_rpm(false)-100.0f)>0.01f)return fail("direct drive output RPM");
    }

    /* Batas safety VESC 6.00 harus benar-benar memengaruhi runtime, bukan
     * sekadar terserialisasi di MC Config. Upstream FOC membatasi Iq melalui
     * Ibus ~= mod_q*Iq, jadi uji watt motoring/regen pada 40 V dan mod_q=0,5,
     * lalu uji derating/fault temperatur dan Vin. */
    {
        mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; set_halls(3u,3u);
        mc_configuration saf=m_motor_1.m_conf;
        saf.l_current_max=15.0f; saf.l_current_min=-15.0f;
        saf.l_watt_max=100.0f; saf.l_watt_min=-80.0f;
        saf.l_min_vin=30.0f; saf.l_max_vin=50.0f;
        saf.l_temp_fet_start=60.0f; saf.l_temp_fet_end=65.0f;
        mcpwm_foc_set_configuration(&saf,false);
        /* Isolate the VESC runtime-limit path from whatever legacy ctrlModReq
         * a previous test case left behind. A real VESC SET_* command owns the
         * motor before these limits are evaluated. */
        mcpwm_foc_vesc_timeout_configure(false,0u,0.0f);
        mcpwm_foc_vesc_override_touch(false);
        batVoltage=(int16_t)((4000L*BAT_CALIB_ADC)/BAT_CALIB_REAL_VOLTAGE); /* 40,00 V */
        m_motor_1.m_control_mode=CONTROL_MODE_CURRENT;
        m_motor_1.m_vq=MCCONF_FOC_VOLTAGE_MAX/2; /* mod_q = +0.5 */
        m_motor_1.m_iq_target_q4=10*FOC_CURRENT_Q4_PER_A;
        m_motor_1.m_iq_set_q4=m_motor_1.m_iq_target_q4;
        m_motor_1.m_iq_set_ramp_q16=(int32_t)m_motor_1.m_iq_set_q4<<16;
        mcpwm_foc_set_board_temperature_x10(250);
        for(uint32_t wi=0u;wi<MCCONF_FOC_CONTROL_DIV;wi++)sim_isr_step();
        if(m_motor_1.m_iq_set_q4>5*FOC_CURRENT_Q4_PER_A+4)return fail("VESC watt max 100W @40V mod_q0.5");
        m_motor_1.m_vq=MCCONF_FOC_VOLTAGE_MAX/2; /* positive voltage, negative Iq = regen */
        m_motor_1.m_iq_target_q4=-10*FOC_CURRENT_Q4_PER_A;
        m_motor_1.m_iq_set_q4=m_motor_1.m_iq_target_q4;
        m_motor_1.m_iq_set_ramp_q16=(int32_t)m_motor_1.m_iq_set_q4<<16;
        for(uint32_t wi=0u;wi<MCCONF_FOC_CONTROL_DIV;wi++)sim_isr_step();
        if(m_motor_1.m_iq_set_q4<-(4*FOC_CURRENT_Q4_PER_A+4))return fail("VESC watt min -80W @40V mod_q0.5");

        /* Pada 62,5 C (tengah 60..65 C) batas arus harus sekitar 50%. */
        m_motor_1.m_duty_now_permille=0;
        m_motor_1.m_iq_target_q4=15*FOC_CURRENT_Q4_PER_A;
        m_motor_1.m_iq_set_q4=m_motor_1.m_iq_target_q4;
        m_motor_1.m_iq_set_ramp_q16=(int32_t)m_motor_1.m_iq_set_q4<<16;
        mcpwm_foc_set_board_temperature_x10(625);
        for(uint32_t wi=0u;wi<MCCONF_FOC_CONTROL_DIV;wi++)sim_isr_step();
        if(m_motor_1.m_iq_set_q4>7500*FOC_CURRENT_Q4_PER_A/1000+8)return fail("VESC FET temperature current derating");

        /* Temperatur dan Vin adalah slow-health checks seperti timeout thread VESC;
         * current/DC trip tetap berada di DMA. Startup ADC 2000 frame harus selesai
         * sebelum health fault boleh aktif. */
        adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
        adc_buffer.dcl=adc_buffer.dcr=2000;
        adc_buffer.batt1=(uint16_t)(((uint32_t)m_motor_1.m_vin_min_adc+(uint32_t)m_motor_1.m_vin_max_adc)/2u);
        for(int i=0;i<2001;i++)DMA1_Channel1_IRQHandler();
        /* Battery LPF starts from compiled nominal voltage. Settle it to the
         * fixture midpoint before testing an unrelated temperature fault. */
        for(int i=0;i<250;i++)legacy_sync();
        m_motor_1.m_fault=FAULT_CODE_NONE; m_motor_1.m_fault_recovery_ticks=0u;
        m_motor_1.m_wrong_voltage_integrator=0u;
        mcpwm_foc_set_board_temperature_x10(650);
        legacy_sync();
        if(m_motor_1.m_fault!=FAULT_CODE_OVER_TEMP_FET)return fail("VESC FET over-temperature fault");

        /* Vin fault memakai battery LPF slow-path. Tahan ADC pada kondisi fault
         * sampai filter + qualification melampaui threshold. */
        mcpwm_foc_set_board_temperature_x10(250);
        m_motor_1.m_fault=FAULT_CODE_NONE; m_motor_1.m_fault_recovery_ticks=0u; m_motor_1.m_wrong_voltage_integrator=0u;
        adc_buffer.batt1=(uint16_t)(m_motor_1.m_vin_min_adc>120u?m_motor_1.m_vin_min_adc-120u:0u);
        for(int i=0;i<400 && m_motor_1.m_fault==FAULT_CODE_NONE;i++)legacy_sync();
        if(m_motor_1.m_fault!=FAULT_CODE_UNDER_VOLTAGE)return fail("VESC under-voltage fault");
        m_motor_1.m_fault=FAULT_CODE_NONE; m_motor_1.m_fault_recovery_ticks=0u; m_motor_1.m_wrong_voltage_integrator=0u;
        adc_buffer.batt1=(uint16_t)(m_motor_1.m_vin_max_adc+120u);
        for(int i=0;i<400 && m_motor_1.m_fault==FAULT_CODE_NONE;i++)legacy_sync();
        if(m_motor_1.m_fault!=FAULT_CODE_OVER_VOLTAGE)return fail("VESC over-voltage fault");
    }

    printf("MOTOR_CONTROL_V13_PASS speed_ramp=%uRPM/s trq50=0.50A posCPR=90 VESCcmd40_phase=%u VESCpos=%.1f runtime_poles_gear=1\n",
           m_motor_1.m_speed_ramp_rpm_s,(unsigned)m_motor_1.m_pos_pid_set_phase,pvals.position);
    return 0;
}
