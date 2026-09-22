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

    /* MODE 1: VESC-style duty limits modulation while inner current PI stays active. */
    ctrlModReq=VLT_MODE; pwml=100; pwmr=-100;
    legacy_sync();
    sim_isr_step(); sim_isr_step();
    if(m_motor_1.m_iq_target_q4!=m_motor_1.m_current_limit_q4)return fail("mode1 left current target");
    if(m_motor_2.m_iq_target_q4!=-m_motor_2.m_current_limit_q4)return fail("mode1 right current target");
    if(abs(m_motor_1.m_vq)>1440 || abs(m_motor_2.m_vq)>1440)return fail("mode1 duty voltage ceiling");
    if(m_motor_1.m_vd!=0 || m_motor_2.m_vd!=0)return fail("mode1 Vd must be zero");
    mcpwm_foc_set_mode_command(VLT_MODE,0,false,SVPWM_OPENLOOP_RPM_DEFAULT,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)return fail("mode1 STOP must release/free-run");

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

    /* Startup offset remains a one-time 2000-frame bridge-OFF calibration.
     * OFF->RUN waits the configured pre-settle window, then learns the powered zero-vector common-mode for 80 frames.
     * The real LEFT bridge can sit near 3.3k ADC counts when MOE is enabled, so
     * validity is rail/pair based rather than incorrectly forcing midscale. */
    mcpwm_foc_init(); LEFT_TIM->BDTR=0u; RIGHT_TIM->BDTR=0u; use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();
    ctrlModReq=VLT_MODE; pwml=100; pwmr=0;
    legacy_sync();
    adc_buffer.rlA=2254; adc_buffer.rlB=2275; adc_buffer.dcl=1923;
    adc_buffer.rrB=1963; adc_buffer.rrC=1938; adc_buffer.dcr=1861; adc_buffer.batt1=2000;
    for(int i=0;i<2000;i++)DMA1_Channel1_IRQHandler();
    if(!m_motor_1.m_current_offset_valid || !m_motor_2.m_current_offset_valid)
        return fail("startup current offsets must be valid");
    /* Hall debounce can need a few frames before the first safe arm. */
    for(unsigned i=0;i<8u && (LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u || !m_motor_1.m_driven_offset_calibrating)
        return fail("OFF-to-RUN must enter powered zero-vector calibration");
    adc_buffer.rlA=3304; adc_buffer.rlB=3436; adc_buffer.dcl=1922;
    adc_buffer.rrB=1963; adc_buffer.rrC=1938; adc_buffer.dcr=1861;
    for(unsigned i=0;i<MCCONF_BRIDGE_PRESETTLE_SAMPLES+MCCONF_BRIDGE_SETTLE_SAMPLES+8u && !m_motor_1.m_driven_offset_finalize_pending;i++)
        DMA1_Channel1_IRQHandler();
    /* Mean/validity powered baseline sekarang sengaja difinalisasi housekeeping
     * (<=5 ms), sementara bridge tetap ditahan zero-vector. */
    legacy_sync();
    if(m_motor_1.m_driven_offset_calibrating || !m_motor_1.m_driven_offset_valid ||
       m_motor_1.m_driven_offset_samples<MCCONF_BRIDGE_SETTLE_SAMPLES)
        return fail("powered zero-vector calibration must complete full window");
    if(abs((int)m_motor_1.m_driven_offset0-3304)>2 || abs((int)m_motor_1.m_driven_offset1-3436)>2)
        return fail("powered common-mode baseline must track real LEFT zero vector");
    if(m_motor_1.m_fault!=FAULT_CODE_NONE || (LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u)
        return fail("healthy powered common-mode must not create offset fault");

    /* A rail-stuck powered current channel must still fail closed after the
     * complete settle window. Use a fresh instance so this safety test cannot
     * contaminate later motor-control cases. */
    mcpwm_foc_init(); LEFT_TIM->BDTR=0u; RIGHT_TIM->BDTR=0u; use_legacy_hall_fixture(); enable=1u; motorRunReq=1u; set_halls(3u,3u);
    legacy_sync();
    ctrlModReq=VLT_MODE; pwml=100; pwmr=0;
    legacy_sync();
    adc_buffer.rlA=2254; adc_buffer.rlB=2275; adc_buffer.dcl=1923;
    adc_buffer.rrB=1963; adc_buffer.rrC=1938; adc_buffer.dcr=1861; adc_buffer.batt1=2000;
    for(int i=0;i<2000;i++)DMA1_Channel1_IRQHandler();
    for(unsigned i=0;i<8u && (LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u;i++)DMA1_Channel1_IRQHandler();
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u)return fail("rail-test bridge did not arm");
    adc_buffer.rlA=4090; adc_buffer.rlB=4088; adc_buffer.dcl=1922;
    for(unsigned i=0;i<MCCONF_BRIDGE_PRESETTLE_SAMPLES+MCCONF_BRIDGE_SETTLE_SAMPLES+8u && !m_motor_1.m_driven_offset_finalize_pending;i++)
        DMA1_Channel1_IRQHandler();
    legacy_sync();
    if(m_motor_1.m_fault!=FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1 || m_motor_1.m_driven_offset_valid)
        return fail("powered offset near ADC rail must fault and stay invalid");

    /* MODE 2: legacy command is mechanical RPM. Active speed setpoint must
     * follow the configured ERPM/s ramp converted through the active pole-pair
     * count; the test duration is derived from that runtime value, not a stale
     * hard-coded mechanical ramp. */
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
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP must ramp toward zero vector");
    if(abs(m_motor_1.m_speed_set_rpm)<=5)return fail("mode2 STOP ramp must not jump to zero");
    for(uint32_t i=0;i<(uint32_t)PWM_FREQ/5u;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_SPEED)return fail("mode2 STOP left SPEED mode too early");
    for(uint32_t i=0;i<ramp_frames_for_delta(50u,m_motor_1.m_speed_ramp_rpm_s);i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)
        return fail("mode2 STOP must release after entering low-speed release zone");
    if(m_motor_1.m_speed_integrator!=0 || m_motor_1.m_iq_target_q4!=0)
        return fail("mode2 release must reset speed PID and command zero Iq");
    if(m_motor_1.m_speed_set_rpm!=0 || m_motor_1.m_speed_target_rpm!=0)
        return fail("mode2 release must zero speed states");
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)
        return fail("mode2 release must disable bridge MOE");

    /* VESC boundary: COMM_SET_RPM is electrical RPM. Convert the 50 mechanical
     * RPM test target through the canonical configured pole-pair count. */
    const float vesc_erpm_50=50.0f*(float)MCCONF_POLE_PAIRS_LEFT;
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; set_halls(3u,3u);
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
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE || m_motor_1.m_iq_set_q4!=0)
        return fail("VESC zero ERPM must release below configured low-speed threshold");
    mcpwm_foc_set_pid_speed(vesc_erpm_50,false);
    mcpwm_foc_vesc_override_touch(false);
    m_motor_1.m_hall_initialized=1u;
    m_motor_1.m_hall_ref_rpm=50;
    m_motor_1.m_hall_ticks=0u;
    mc_values vals;
    mcpwm_foc_get_values(&vals,false);
    if(fabsf(vals.rpm-vesc_erpm_50)>0.1f)return fail("mc_values.rpm must be ERPM");

    /* VESC speed STOP: once the ramp enters s_pid_min_erpm, release the bridge
     * to high impedance after clearing the speed/current state. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u;
    /* Production main initializes housekeeping before a VESC command arrives.
     * Prime the slow clock here so the following 80 PWM frames represent one
     * actual 5-ms outer-loop interval, not scheduler initialization itself. */
    legacy_sync();
    mcpwm_foc_set_pid_speed(300.0f,false); mcpwm_foc_vesc_override_touch(false);
    m_motor_1.m_speed_set_ramp_q16=(int32_t)4<<16;
    m_motor_1.m_speed_target_rpm_q16=0; m_motor_1.m_speed_target_rpm=0;
    m_motor_1.m_iq_set_q4=800; m_motor_1.m_iq_target_q4=800; m_motor_1.m_iq_set_ramp_q16=(int32_t)800<<16;
    /* SPEED outer PID/zero-zone berjalan di scheduler 1 kHz. 80 PWM frame
     * mewakili 5 ms sehingga menyediakan beberapa outer tick fresh-feedback. */
    for(unsigned i=0;i<80u;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE)
        return fail("speed STOP low-speed zone must release bridge");
    if(m_motor_1.m_iq_set_q4!=0 || m_motor_1.m_iq_target_q4!=0)
        return fail("speed STOP release must clear Iq");
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)
        return fail("speed STOP release must disable bridge MOE");

    /* VESC current brake: recompute torque sign from live rotor speed. At zero/stale
     * speed the zero-current zero-vector remains active until another command. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u;
    m_motor_1.m_hall_initialized=1u; m_motor_1.m_hall_direction=1; m_motor_1.m_hall_period=200u; m_motor_1.m_hall_ticks=20u; m_motor_1.m_rpm=53;
    mcpwm_foc_set_brake_current(1.0f,false); mcpwm_foc_vesc_override_touch(false);
    for(unsigned i=0;i<MCCONF_FOC_CONTROL_DIV;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT_BRAKE || m_motor_1.m_iq_target_q4>=0)return fail("brake must oppose positive speed");
    m_motor_1.m_hall_ticks=1200u;
    for(unsigned i=0;i<MCCONF_FOC_CONTROL_DIV;i++)sim_isr_step();
    if(m_motor_1.m_iq_target_q4!=0)return fail("stale brake speed must command zero torque");
    for(int i=0;i<1200;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT_BRAKE || m_motor_1.m_iq_set_q4!=0)return fail("brake must remain active at zero speed");

    /* VESC handbrake is not current-brake. It creates a stationary electrical
     * field at phase zero so the rotor is held rather than continuously driven. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u;
    mcpwm_foc_set_handbrake(1.0f,false); mcpwm_foc_vesc_override_touch(false);
    for(unsigned i=0;i<MCCONF_FOC_CONTROL_DIV;i++)sim_isr_step();
    if(m_motor_1.m_control_mode!=CONTROL_MODE_HANDBRAKE)return fail("handbrake control mode");
    if(m_motor_1.m_phase!=0u)return fail("handbrake must lock phase zero");
    if(m_motor_1.m_iq_target_q4<=0)return fail("handbrake current missing");

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

    /* VESC normalized +/-1.0 is clamped by the configured l_max_duty safety
     * ceiling before the EFeru hardware modulation limit is applied. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u;
    const int16_t duty_limit=m_motor_1.m_duty_limit_permille;
    legacy_sync();
    ctrlModReq=VLT_MODE; pwml=1000; pwmr=0;
    legacy_sync();
    for(int i=0;i<32000;i++)sim_isr_step();
    if(m_motor_1.m_duty_set_permille!=duty_limit)return fail("duty +1.0 command clamp");
    if(m_motor_1.m_duty_now_permille!=duty_limit)return fail("duty +1.0 telemetry clamp");
    if(m_motor_1.m_iq_target_q4!=m_motor_1.m_current_limit_q4)return fail("duty target must stay current-limited");
    if(m_motor_1.m_ccr_a<110 || m_motor_1.m_ccr_a>1890 ||
       m_motor_1.m_ccr_b<110 || m_motor_1.m_ccr_b>1890 ||
       m_motor_1.m_ccr_c<110 || m_motor_1.m_ccr_c>1890)
        return fail("duty1 CCR violates EFeru 110..1890 margin");
    mcpwm_foc_set_duty(-1.0f,false); mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_duty_set_permille!=-duty_limit)return fail("duty -1.0 command clamp");
    for(int i=0;i<32000;i++){if((i%1000)==0)mcpwm_foc_vesc_override_touch(false);sim_isr_step();}
    if(m_motor_1.m_duty_now_permille!=-duty_limit)return fail("duty -1.0 telemetry clamp");
    mcpwm_foc_set_duty(0.0f,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE || m_motor_1.m_duty_set_permille!=0 ||
       (LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)
        return fail("VESC SET_DUTY zero must release high impedance");
    mcpwm_foc_set_current(1.0f,false); mcpwm_foc_vesc_override_touch(false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT)return fail("VESC SET_CURRENT 1A entry");
    mcpwm_foc_set_current(0.0f,false);
    if(m_motor_1.m_control_mode!=CONTROL_MODE_NONE || (LEFT_TIM->BDTR&TIM_BDTR_MOE)!=0u)
        return fail("VESC SET_CURRENT zero must immediate free-run");

    /* Two-shunt regression: a raw phase sample can be unobservable near a PWM
     * boundary and must NOT directly cause VESC ABS_OVER_CURRENT. The ABS source
     * is D/Q motor-current magnitude; three consecutive over-limit D/Q samples
     * still must fault. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u; ctrlModReq=VLT_MODE; pwml=950; pwmr=0;
    adc_buffer.batt1=(uint16_t)(((uint32_t)m_motor_1.m_vin_min_adc+(uint32_t)m_motor_1.m_vin_max_adc)/2u);
    legacy_sync();
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2005;i++)DMA1_Channel1_IRQHandler();
    adc_buffer.batt1=(uint16_t)(((uint32_t)m_motor_1.m_vin_min_adc+(uint32_t)m_motor_1.m_vin_max_adc)/2u);
    if((LEFT_TIM->BDTR&TIM_BDTR_MOE)==0u)return fail("duty95 bridge did not arm");
    for(unsigned i=0u;i<MCCONF_BRIDGE_PRESETTLE_SAMPLES+MCCONF_BRIDGE_SETTLE_SAMPLES+8u && !m_motor_1.m_driven_offset_finalize_pending;i++){
        DMA1_Channel1_IRQHandler();
    }
    legacy_sync();
    adc_buffer.rlA=400; /* raw reconstructed phase >30 A; intentionally ignored as direct ABS source */
    m_motor_1.m_id_q4=0; m_motor_1.m_iq_q4=0; m_motor_1.m_dq_sample_fresh=1u; DMA1_Channel1_IRQHandler();
    if(m_motor_1.m_fault!=FAULT_CODE_NONE || m_motor_1.m_phase_overcurrent_streak!=0u)
        return fail("raw two-shunt phase glitch must not trip ABS");
    adc_buffer.rlA=adc_buffer.rlB=2000;
    for(int k=1;k<=3;k++){
        m_motor_1.m_id_q4=(int16_t)((MCCONF_L_ABS_CURRENT_MAX+1.0f)*FOC_CURRENT_Q4_PER_A); m_motor_1.m_iq_q4=0; m_motor_1.m_dq_sample_fresh=1u;
        DMA1_Channel1_IRQHandler();
        if(k<3 && m_motor_1.m_fault!=FAULT_CODE_NONE)return fail("DQ ABS faulted before qualifier");
    }
    if(m_motor_1.m_fault!=FAULT_CODE_ABS_OVER_CURRENT)return fail("persistent DQ motor over-current must fault");
    if(m_motor_1.m_phase_trip_count!=1u || m_motor_1.m_dc_trip_count!=0u || m_motor_1.m_last_trip_source!=1u)
        return fail("DQ ABS diagnostic split");

    /* DC-link level-2 protection remains immediate even at high duty. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); set_halls(3u,3u); enable=1u; motorRunReq=1u; ctrlModReq=VLT_MODE; pwml=950; pwmr=0;
    adc_buffer.batt1=(uint16_t)(((uint32_t)m_motor_1.m_vin_min_adc+(uint32_t)m_motor_1.m_vin_max_adc)/2u);
    legacy_sync();
    adc_buffer.rlA=adc_buffer.rlB=adc_buffer.rrB=adc_buffer.rrC=2000;
    adc_buffer.dcl=adc_buffer.dcr=2000; adc_buffer.batt1=2000;
    for(int i=0;i<2005;i++)DMA1_Channel1_IRQHandler();
    adc_buffer.batt1=(uint16_t)(((uint32_t)m_motor_1.m_vin_min_adc+(uint32_t)m_motor_1.m_vin_max_adc)/2u);
    for(unsigned i=0u;i<MCCONF_BRIDGE_PRESETTLE_SAMPLES+MCCONF_BRIDGE_SETTLE_SAMPLES+8u && !m_motor_1.m_driven_offset_finalize_pending;i++){
        DMA1_Channel1_IRQHandler();
    }
    legacy_sync();
    adc_buffer.dcl=1100; /* +900 counts = 18 A > 17 A DC hard limit */
    m_motor_1.m_duty_now_permille=900; DMA1_Channel1_IRQHandler();
    if(m_motor_1.m_fault!=FAULT_CODE_ABS_OVER_CURRENT)return fail("DC-link OC must fault immediately");
    if(m_motor_1.m_dc_trip_count!=1u || m_motor_1.m_last_trip_source!=2u)return fail("DC OC diagnostic split");

    /* RIGHT Hall direction regression. Hall filtering now honors VESC
     * m_hall_extra_samples (default window 7), so allow the majority filter
     * plus debounce to settle before checking the accepted 3->2 edge. */
    mcpwm_foc_init(); use_legacy_hall_fixture(); enable=1u; ctrlModReq=VLT_MODE; pwml=1;pwmr=-1;set_halls(3u,3u);
    legacy_sync();
    if(!m_motor_2.m_conf.m_invert_direction)return fail("right default direction mirror config");
    for(int i=0;i<100;i++)sim_isr_step();
    set_halls(2u,2u);
    for(uint16_t i=0u;i<(uint16_t)m_motor_2.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;i++)sim_isr_step();
    if(m_motor_2.m_hall_direction!=-1)return fail("right reverse Hall direction");
    for(uint32_t i=0u;i<MCCONF_HALL_TIMEOUT_TICKS+100u;i++)sim_isr_step();
    if(m_motor_2.m_hall_interp_active!=0u)return fail("Hall interpolation low-speed disable");

    printf("MOTOR_CONTROL_V12_PASS speed_ramp=%uRPM/s release=%uERPM trq50=0.50A erpm=%.0f mode4=2A\n",
           m_motor_1.m_speed_ramp_rpm_s,(unsigned)(m_motor_1.m_speed_release_erpm_q16>>16),vals.rpm);
    return 0;
}
