#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;
uint16_t VirtAddVarTab[NB_OF_VAR];

static uint16_t ee_value[NB_OF_VAR];
static uint8_t ee_valid[NB_OF_VAR];
/* Permanent write failure after N successful EEPROM writes, used to model a
 * brownout/flash fault in the middle of a transaction. -1 disables injection. */
static int ee_fail_after=-1;
static int ee_write_count=0;

static int slot_from_addr(uint16_t addr){
    for(unsigned i=0;i<NB_OF_VAR;i++) if(VirtAddVarTab[i]==addr) return (int)i;
    return -1;
}
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data){
    int i=slot_from_addr(VirtAddress);
    if(i<0 || !ee_valid[i]) return 1u;
    *Data=ee_value[i];
    return 0u;
}
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data){
    int i=slot_from_addr(VirtAddress);
    if(i<0) return 1u;
    if(ee_fail_after>=0 && ee_write_count++>=ee_fail_after)return 1u;
    ee_value[i]=Data; ee_valid[i]=1u;
    return HAL_OK;
}
uint16_t EE_Init(void){return 0u;}
uint16_t EE_Format(void){memset(ee_valid,0,sizeof(ee_valid));return 0u;}

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y){
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static int fail(const char *s){fprintf(stderr,"FAIL %s\n",s);return 1;}
static void fill_table(uint8_t t[8], uint8_t base){
    t[0]=255u; t[7]=255u;
    for(int i=1;i<=6;i++) t[i]=(uint8_t)(base+(i-1)*33u);
}
static int same8(const uint8_t a[8],const uint8_t b[8]){return memcmp(a,b,8)==0;}

int main(void){
    for(unsigned i=0;i<NB_OF_VAR;i++) VirtAddVarTab[i]=(uint16_t)(1000u+i);
    memset(ee_valid,0,sizeof(ee_valid));
    mcpwm_foc_init();

    /* ABS current must cover the magnitude of BOTH current directions. A
     * braking limit larger than the positive motoring limit must not slip past
     * validation and cause a guaranteed ABS fault during regen. */
    {
        mc_configuration bad=m_motor_1.m_conf;
        bad.l_current_max=1.0f; bad.l_current_min=-12.0f; bad.l_abs_current_max=10.0f;
        mcpwm_foc_set_configuration(&bad,false);
        if(fabsf(m_motor_1.m_conf.l_abs_current_max-MCCONF_L_ABS_CURRENT_MAX)>0.001f)
            return fail("ABS current must cover negative current magnitude");
        if(m_motor_1.m_abs_current_limit_counts!=(int16_t)(MCCONF_L_ABS_CURRENT_MAX*A2BIT_CONV+0.5f))
            return fail("ABS runtime count scaling after bidirectional validation");
        bad.l_abs_current_max=13.0f;
        mcpwm_foc_set_configuration(&bad,false);
        if(fabsf(m_motor_1.m_conf.l_abs_current_max-13.0f)>0.001f ||
           m_motor_1.m_abs_current_limit_counts!=13*A2BIT_CONV)
            return fail("valid ABS current setting must stay authoritative");
    }

    uint8_t hl[8],hr[8]; fill_table(hl,5u); fill_table(hr,11u);
    mc_configuration cl=m_motor_1.m_conf, cr=m_motor_2.m_conf;
    memcpy(cl.foc_hall_table,hl,8); memcpy(cr.foc_hall_table,hr,8);
    cl.l_current_max=12.34f; cl.l_current_min=-7.65f; cl.l_abs_current_max=18.25f; cl.l_max_duty=0.9134f; cl.l_slow_abs_current=true;
    cr.l_current_max=9.87f; cr.l_current_min=-6.54f; cr.l_abs_current_max=17.75f; cr.l_max_duty=0.8765f; cr.l_slow_abs_current=false;
    cl.l_in_current_max=14.50f; cl.l_in_current_min=-13.50f; cl.m_duty_ramp_step=0.0312f; cl.cc_min_current=0.17f;
    cr.l_in_current_max=12.25f; cr.l_in_current_min=-11.75f; cr.m_duty_ramp_step=0.0175f; cr.cc_min_current=0.23f;
    cl.l_current_max_scale=0.80f; cl.l_current_min_scale=0.60f;
    cr.l_current_max_scale=0.70f; cr.l_current_min_scale=0.50f;
    cl.l_battery_cut_start=37.20f; cl.l_battery_cut_end=33.10f;
    cr.l_battery_cut_start=36.80f; cr.l_battery_cut_end=32.90f;
    cl.l_min_vin=28.50f; cl.l_max_vin=52.30f; cl.l_watt_max=1234.5f; cl.l_watt_min=-987.6f; cl.l_temp_fet_start=61.2f; cl.l_temp_fet_end=72.3f;
    cr.l_min_vin=29.20f; cr.l_max_vin=51.70f; cr.l_watt_max=1111.1f; cr.l_watt_min=-888.8f; cr.l_temp_fet_start=59.4f; cr.l_temp_fet_end=70.5f;
    cl.l_min_erpm=-12000.0f; cl.l_max_erpm=10000.0f; cl.si_wheel_diameter=0.2450f;
    cr.l_min_erpm=-9000.0f; cr.l_max_erpm=11000.0f; cr.si_wheel_diameter=0.3100f;
    cl.p_pid_kd_filter=0.37f; cr.p_pid_kd_filter=0.63f;
    cl.foc_duty_dowmramp_kp=23.4f; cl.foc_duty_dowmramp_ki=456.7f;
    cr.foc_duty_dowmramp_kp=17.8f; cr.foc_duty_dowmramp_ki=321.2f;
    cl.si_motor_poles=20u; cl.si_gear_ratio=5.25f; cl.foc_current_filter_const=0.0731f; cl.foc_hall_interp_erpm=620.0f; cl.m_hall_extra_samples=5;
    cl.foc_motor_r=0.180123f; cl.foc_motor_l=0.00035123f; cl.foc_motor_ld_lq_diff=0.0000123f; cl.foc_motor_flux_linkage=0.018765f;
    cl.m_sensor_port_mode=SENSOR_PORT_MODE_ABI; cl.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
    cl.m_encoder_counts=4096; cl.foc_encoder_offset=17.251234f; cl.foc_encoder_ratio=10.123456f; cl.foc_encoder_inverted=true;
    cl.p_pid_offset=13.125f; cl.p_pid_ang_div=2.0f; cl.m_invert_direction=true;
    cr.p_pid_offset=-7.25f; cr.p_pid_ang_div=0.5f; cr.m_invert_direction=false;
    cl.p_pid_kd_proc=0.0012345f; cl.p_pid_gain_dec_angle=12.3f;
    cr.p_pid_kd_proc=0.0023456f; cr.p_pid_gain_dec_angle=45.6f;
    cr.si_motor_poles=14u; cr.si_gear_ratio=1.0f; cr.foc_current_filter_const=0.2197f; cr.foc_hall_interp_erpm=900.0f; cr.m_hall_extra_samples=2;
    cr.foc_motor_r=0.220456f; cr.foc_motor_l=0.00042156f; cr.foc_motor_ld_lq_diff=-0.0000098f; cr.foc_motor_flux_linkage=0.020876f;
    cl.foc_pll_kp=2100.125f; cl.foc_pll_ki=31000.5f; cl.foc_dt_us=0.1234f;
    cl.s_pid_speed_source=S_PID_SPEED_SRC_PLL; cl.foc_cc_decoupling=FOC_CC_DECOUPLING_CROSS_BEMF;
    cr.foc_pll_kp=1800.25f; cr.foc_pll_ki=28000.75f; cr.foc_dt_us=0.4564f;
    cr.s_pid_speed_source=S_PID_SPEED_SRC_FAST; cr.foc_cc_decoupling=FOC_CC_DECOUPLING_BEMF;
    mcpwm_foc_set_configuration(&cl,false);
    mcpwm_foc_set_configuration(&cr,true);
    m_motor_1.m_kpq_q11=1111u; m_motor_1.m_kiq_q16=2222u;
    m_motor_1.m_kpd_q11=333u;  m_motor_1.m_kid_q16=444u;
    m_motor_1.m_kps_q11=555u;  m_motor_1.m_kis_q16=666u; m_motor_1.m_kds_q11=77u;
    m_motor_1.m_kpp_q11=888u;  m_motor_1.m_kip_q16=999u; m_motor_1.m_kdp_q11=111u;
    m_motor_1.m_speed_ramp_rpm_s=123u; m_motor_1.m_speed_release_erpm_q16=28u<<16;
    m_motor_2.m_kpq_q11=1212u; m_motor_2.m_kiq_q16=2323u;
    m_motor_2.m_kpd_q11=343u;  m_motor_2.m_kid_q16=454u;
    m_motor_2.m_kps_q11=565u;  m_motor_2.m_kis_q16=676u; m_motor_2.m_kds_q11=87u;
    m_motor_2.m_kpp_q11=898u;  m_motor_2.m_kip_q16=909u; m_motor_2.m_kdp_q11=121u;
    m_motor_2.m_speed_ramp_rpm_s=234u; m_motor_2.m_speed_release_erpm_q16=120u<<16;
    /* Raw/custom tuning is an explicit fixed-point API. Mirror it into the
     * standard MC config before persistence, exactly like HB_CUSTOM_SET_TUNING
     * and terminal SET callbacks do in production. */
    mcpwm_foc_sync_tuning_to_conf(false);
    mcpwm_foc_sync_tuning_to_conf(true);
    if(!mc_interface_store_configuration_motor(false)) return fail("store left");
    if(!mc_interface_store_configuration_motor(true)) return fail("store right");

    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false)) return fail("load left");
    if(!mc_interface_load_configuration_motor(true)) return fail("load right");
    if(!same8(m_motor_1.m_conf.foc_hall_table,hl)) return fail("left Hall persistence");
    if(!same8(m_motor_2.m_conf.foc_hall_table,hr)) return fail("right Hall persistence");
    if(fabsf(m_motor_1.m_conf.l_current_max-12.34f)>0.011f) return fail("left current persistence");
    if(fabsf(m_motor_2.m_conf.l_current_max-9.87f)>0.011f) return fail("right current persistence");
    if(fabsf(m_motor_1.m_conf.l_current_min+7.65f)>0.011f || fabsf(m_motor_2.m_conf.l_current_min+6.54f)>0.011f) return fail("current min persistence");
    if(fabsf(m_motor_1.m_conf.l_in_current_max-14.50f)>0.011f || fabsf(m_motor_2.m_conf.l_in_current_max-12.25f)>0.011f) return fail("input current max persistence");
    if(fabsf(m_motor_1.m_conf.l_in_current_min+13.50f)>0.011f || fabsf(m_motor_2.m_conf.l_in_current_min+11.75f)>0.011f) return fail("input current min persistence");
    if(fabsf(m_motor_1.m_conf.m_duty_ramp_step-0.0312f)>0.00011f || fabsf(m_motor_2.m_conf.m_duty_ramp_step-0.0175f)>0.00011f) return fail("duty ramp persistence");
    if(fabsf(m_motor_1.m_conf.cc_min_current-0.17f)>0.011f || fabsf(m_motor_2.m_conf.cc_min_current-0.23f)>0.011f) return fail("cc_min_current persistence");
    if(fabsf(m_motor_1.m_conf.l_current_max_scale-0.80f)>0.00011f ||
       fabsf(m_motor_1.m_conf.l_current_min_scale-0.60f)>0.00011f ||
       fabsf(m_motor_2.m_conf.l_current_max_scale-0.70f)>0.00011f ||
       fabsf(m_motor_2.m_conf.l_current_min_scale-0.50f)>0.00011f) return fail("current scale persistence");
    if(fabsf(m_motor_1.m_conf.l_battery_cut_start-1.0f*37.20f)>0.011f ||
       fabsf(m_motor_1.m_conf.l_battery_cut_end-33.10f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_start-36.80f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_end-32.90f)>0.011f) return fail("battery cut persistence");
    if(fabsf(m_motor_1.m_conf.l_min_vin-28.50f)>0.011f || fabsf(m_motor_1.m_conf.l_max_vin-52.30f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_min_vin-29.20f)>0.011f || fabsf(m_motor_2.m_conf.l_max_vin-51.70f)>0.011f) return fail("Vin safety persistence");
    if(fabsf(m_motor_1.m_conf.l_watt_max-1234.5f)>0.11f || fabsf(m_motor_1.m_conf.l_watt_min+987.6f)>0.11f ||
       fabsf(m_motor_2.m_conf.l_watt_max-1111.1f)>0.11f || fabsf(m_motor_2.m_conf.l_watt_min+888.8f)>0.11f) return fail("watt safety persistence");
    if(fabsf(m_motor_1.m_conf.l_temp_fet_start-61.2f)>0.051f || fabsf(m_motor_1.m_conf.l_temp_fet_end-72.3f)>0.051f ||
       fabsf(m_motor_2.m_conf.l_temp_fet_start-59.4f)>0.051f || fabsf(m_motor_2.m_conf.l_temp_fet_end-70.5f)>0.051f) return fail("FET temperature persistence");
    if(m_motor_1.m_watt_max_x10!=12345u || m_motor_1.m_watt_regen_x10!=9876u ||
       m_motor_2.m_watt_max_x10!=11111u || m_motor_2.m_watt_regen_x10!=8888u) return fail("watt runtime restore");
    if(m_motor_1.m_temp_fet_start_x10!=612 || m_motor_1.m_temp_fet_end_x10!=723 ||
       m_motor_2.m_temp_fet_start_x10!=594 || m_motor_2.m_temp_fet_end_x10!=705) return fail("temperature runtime restore");
    if(abs((int)m_motor_1.m_vin_min_adc-(int)(28.50f*100.0f*BAT_CALIB_ADC/BAT_CALIB_REAL_VOLTAGE+0.5f))>1 ||
       abs((int)m_motor_2.m_vin_max_adc-(int)(51.70f*100.0f*BAT_CALIB_ADC/BAT_CALIB_REAL_VOLTAGE+0.5f))>1) return fail("Vin ADC runtime restore");
    if(fabsf(m_motor_1.m_conf.l_min_erpm+12000.0f)>0.5f || fabsf(m_motor_1.m_conf.l_max_erpm-10000.0f)>0.5f ||
       fabsf(m_motor_2.m_conf.l_min_erpm+9000.0f)>0.5f || fabsf(m_motor_2.m_conf.l_max_erpm-11000.0f)>0.5f) return fail("ERPM persistence/clamp");
    if(fabsf(m_motor_1.m_conf.si_wheel_diameter-0.2450f)>0.00011f ||
       fabsf(m_motor_2.m_conf.si_wheel_diameter-0.3100f)>0.00011f) return fail("wheel diameter persistence");
    if(abs((int)m_motor_1.m_current_limit_q4-7898)>1 || abs((int)m_motor_1.m_current_limit_neg_q4-3672)>1 ||
       abs((int)m_motor_2.m_current_limit_q4-5527)>1 || abs((int)m_motor_2.m_current_limit_neg_q4-2616)>1)
        return fail("scaled bidirectional current runtime restore");
    {
        const int left_start=(int)(37.20f*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        const int left_end=(int)(33.10f*100.0f*(float)BAT_CALIB_ADC/(float)BAT_CALIB_REAL_VOLTAGE+0.5f);
        if(abs((int)m_motor_1.m_battery_cut_start_adc-left_start)>1 ||
           abs((int)m_motor_1.m_battery_cut_end_adc-left_end)>1)
            return fail("battery cut ADC runtime restore");
    }
    if(m_motor_1.m_input_current_max_q4!=11600 || m_motor_1.m_input_current_regen_q4!=10800 ||
       m_motor_2.m_input_current_max_q4!=9800 || m_motor_2.m_input_current_regen_q4!=9400) return fail("input current runtime restore");
    if(fabsf(m_motor_1.m_conf.l_abs_current_max-18.25f)>0.011f || fabsf(m_motor_2.m_conf.l_abs_current_max-17.75f)>0.011f) return fail("abs current persistence");
    if(!m_motor_1.m_conf.l_slow_abs_current || m_motor_2.m_conf.l_slow_abs_current) return fail("slow ABS current persistence");
    if(fabsf(m_motor_1.m_conf.l_max_duty-0.9134f)>0.00011f || fabsf(m_motor_2.m_conf.l_max_duty-0.8765f)>0.00011f) return fail("max duty persistence");
    if(fabsf(m_motor_1.m_conf.p_pid_kd_filter-0.37f)>0.00011f || fabsf(m_motor_2.m_conf.p_pid_kd_filter-0.63f)>0.00011f) return fail("position D filter persistence");
    if(fabsf(m_motor_1.m_conf.foc_duty_dowmramp_kp-23.4f)>0.051f || fabsf(m_motor_2.m_conf.foc_duty_dowmramp_kp-17.8f)>0.051f) return fail("duty downramp Kp persistence");
    if(fabsf(m_motor_1.m_conf.foc_duty_dowmramp_ki-456.7f)>0.051f || fabsf(m_motor_2.m_conf.foc_duty_dowmramp_ki-321.2f)>0.051f) return fail("duty downramp Ki persistence");
    if(m_motor_1.m_abs_current_limit_counts<912 || m_motor_1.m_abs_current_limit_counts>914 ||
       m_motor_2.m_abs_current_limit_counts<887 || m_motor_2.m_abs_current_limit_counts>889) return fail("absolute current runtime restore");
    if(m_motor_1.m_duty_limit_permille!=913 || m_motor_2.m_duty_limit_permille!=877) return fail("max duty runtime restore");
    if(abs((int)m_motor_1.m_position_kd_filter_q16-(int)(0.37f*65535.0f+0.5f))>2 ||
       abs((int)m_motor_2.m_position_kd_filter_q16-(int)(0.63f*65535.0f+0.5f))>2) return fail("position D filter runtime restore");
    if(m_motor_1.m_duty_kp_q12_per_permille==0u || m_motor_1.m_duty_ki_q12_per_permille==0u ||
       m_motor_2.m_duty_kp_q12_per_permille==0u || m_motor_2.m_duty_ki_q12_per_permille==0u) return fail("duty PI runtime restore");
    /* Motor poles are hardware identity, not EEPROM identity: stale/cross-motor
     * slots must canonicalize to LEFT 4 pole-pairs and RIGHT 15 pole-pairs.
     * Gear ratio remains a legitimate persisted user configuration. */
    if(m_motor_1.m_conf.si_motor_poles!=8u || fabsf(m_motor_1.m_conf.si_gear_ratio-5.25f)>0.02f) return fail("left fixed poles/gear persistence");
    if(m_motor_2.m_conf.si_motor_poles!=30u || fabsf(m_motor_2.m_conf.si_gear_ratio-1.0f)>0.02f) return fail("right fixed poles/gear persistence");
    if(fabsf(m_motor_1.m_conf.foc_current_filter_const-0.0731f)>0.00011f) return fail("left telemetry filter persistence");
    if(fabsf(m_motor_2.m_conf.foc_current_filter_const-0.2197f)>0.00011f) return fail("right telemetry filter persistence");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-620.0f)>0.5f || fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-900.0f)>0.5f)
        return fail("Hall interpolation ERPM persistence");
    if(m_motor_1.m_conf.m_hall_extra_samples!=5 || m_motor_2.m_conf.m_hall_extra_samples!=2) return fail("Hall extra-samples persistence");
    if(fabsf(m_motor_1.m_conf.foc_motor_r-0.180123f)>1e-7f || fabsf(m_motor_2.m_conf.foc_motor_r-0.220456f)>1e-7f ||
       fabsf(m_motor_1.m_conf.foc_motor_l-0.00035123f)>1e-9f || fabsf(m_motor_2.m_conf.foc_motor_l-0.00042156f)>1e-9f ||
       fabsf(m_motor_1.m_conf.foc_motor_flux_linkage-0.018765f)>1e-7f || fabsf(m_motor_2.m_conf.foc_motor_flux_linkage-0.020876f)>1e-7f)
        return fail("Detect-All motor model exact persistence");
    if(fabsf(m_motor_1.m_conf.foc_pll_kp-2100.125f)>1e-6f ||
       fabsf(m_motor_1.m_conf.foc_pll_ki-31000.5f)>1e-4f ||
       m_motor_1.m_conf.s_pid_speed_source!=S_PID_SPEED_SRC_PLL ||
       m_motor_1.m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_CROSS_BEMF ||
       fabsf(m_motor_2.m_conf.foc_pll_kp-1800.25f)>1e-6f ||
       fabsf(m_motor_2.m_conf.foc_pll_ki-28000.75f)>1e-4f ||
       m_motor_2.m_conf.s_pid_speed_source!=S_PID_SPEED_SRC_FAST ||
       m_motor_2.m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_BEMF ||
       fabsf(m_motor_1.m_conf.foc_dt_us-0.123f)>0.00051f ||
       fabsf(m_motor_2.m_conf.foc_dt_us-0.456f)>0.00051f)
        return fail("PLL/decoupling/deadtime exact persistence");
    if(m_motor_1.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_ABI || m_motor_1.m_conf.foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER ||
       m_motor_1.m_conf.m_encoder_counts!=4096 || !m_motor_1.m_conf.foc_encoder_inverted ||
       fabsf(m_motor_1.m_conf.foc_encoder_offset-17.251234f)>0.000001f ||
       fabsf(m_motor_1.m_conf.foc_encoder_ratio-10.123456f)>0.000001f)
        return fail("LEFT ABI exact encoder persistence");
    if(!m_motor_1.m_conf.m_invert_direction || m_motor_2.m_conf.m_invert_direction ||
       fabsf(m_motor_1.m_conf.p_pid_offset-13.125f)>0.000001f ||
       fabsf(m_motor_2.m_conf.p_pid_offset+7.25f)>0.000001f ||
       fabsf(m_motor_1.m_conf.p_pid_ang_div-2.0f)>0.000001f ||
       fabsf(m_motor_2.m_conf.p_pid_ang_div-0.5f)>0.000001f ||
       fabsf(m_motor_1.m_conf.p_pid_kd_proc-0.0012345f)>0.0000001f ||
       fabsf(m_motor_2.m_conf.p_pid_kd_proc-0.0023456f)>0.0000001f ||
       fabsf(m_motor_1.m_conf.p_pid_gain_dec_angle-12.3f)>0.051f ||
       fabsf(m_motor_2.m_conf.p_pid_gain_dec_angle-45.6f)>0.051f)
        return fail("V32 exact position/direction persistence");
    if(m_motor_1.m_pos_pid_ang_div_inv_q16<32767u || m_motor_1.m_pos_pid_ang_div_inv_q16>32769u ||
       m_motor_2.m_pos_pid_ang_div_inv_q16<131071u || m_motor_2.m_pos_pid_ang_div_inv_q16>131073u)
        return fail("V32 p_pid_ang_div runtime cache restore");
    if(!m_motor_1.m_encoder_configured || m_motor_1.m_encoder_synced) return fail("LEFT ABI runtime init/sync lifecycle");
    if(m_motor_2.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_HALL || m_motor_2.m_conf.foc_sensor_mode!=FOC_SENSOR_MODE_HALL)
        return fail("RIGHT must remain Hall-only");
    if(m_motor_1.m_hall_interp_erpm!=620u || m_motor_2.m_hall_interp_erpm!=900u ||
       m_motor_1.m_hall_interp_max_ticks==0u || m_motor_2.m_hall_interp_max_ticks==0u ||
       m_motor_1.m_hall_rate_min_step==0u || m_motor_2.m_hall_rate_min_step==0u)
        return fail("Hall interpolation runtime coefficient restore");
    if(m_motor_1.m_kpq_q11!=1111u || m_motor_1.m_kiq_q16!=2222u || m_motor_1.m_kdp_q11!=111u) return fail("left gains persistence");
    if(m_motor_2.m_kpq_q11!=1212u || m_motor_2.m_kiq_q16!=2323u || m_motor_2.m_kdp_q11!=121u) return fail("right gains persistence");
    if(m_motor_1.m_speed_ramp_rpm_s!=123u || m_motor_1.m_speed_release_erpm_q16!=(28u<<16)) return fail("left speed persistence");
    if(m_motor_2.m_speed_ramp_rpm_s!=234u || m_motor_2.m_speed_release_erpm_q16!=(120u<<16)) return fail("right speed persistence");

    /* Standard VESC SET_MCCONF keeps the requested float values authoritative.
     * Runtime coefficients are quantized for the ISR, but GET_MCCONF and EEPROM
     * must return these exact float32 shadows rather than reverse-converting the
     * fixed-point coefficients. */
    {
        mc_configuration xl=m_motor_1.m_conf, xr=m_motor_2.m_conf;
        xl.foc_current_kp=0.812345f; xl.foc_current_ki=267.12345f;
        xl.s_pid_kp=0.0123456f; xl.s_pid_ki=0.0234567f; xl.s_pid_kd=0.0006789f;
        xl.p_pid_kp=0.123456f; xl.p_pid_ki=0.023456f; xl.p_pid_kd=0.0034567f;
        xr.foc_current_kp=0.923456f; xr.foc_current_ki=312.34567f;
        xr.s_pid_kp=0.0135791f; xr.s_pid_ki=0.0246802f; xr.s_pid_kd=0.0007891f;
        xr.p_pid_kp=0.234567f; xr.p_pid_ki=0.034567f; xr.p_pid_kd=0.0045678f;
        mcpwm_foc_set_configuration(&xl,false); mcpwm_foc_set_configuration(&xr,true);
        if(!mc_interface_store_configuration_motor(false) || !mc_interface_store_configuration_motor(true))
            return fail("exact PID shadow store");
        mcpwm_foc_init();
        if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
            return fail("exact PID shadow load");
        if(fabsf(m_motor_1.m_conf.foc_current_kp-xl.foc_current_kp)>1e-7f ||
           fabsf(m_motor_1.m_conf.foc_current_ki-xl.foc_current_ki)>1e-5f ||
           fabsf(m_motor_1.m_conf.s_pid_kp-xl.s_pid_kp)>1e-8f ||
           fabsf(m_motor_1.m_conf.s_pid_ki-xl.s_pid_ki)>1e-8f ||
           fabsf(m_motor_1.m_conf.s_pid_kd-xl.s_pid_kd)>1e-9f ||
           fabsf(m_motor_1.m_conf.p_pid_kp-xl.p_pid_kp)>1e-7f ||
           fabsf(m_motor_1.m_conf.p_pid_ki-xl.p_pid_ki)>1e-7f ||
           fabsf(m_motor_1.m_conf.p_pid_kd-xl.p_pid_kd)>1e-8f)
            return fail("left exact PID float32 persistence");
        if(fabsf(m_motor_2.m_conf.foc_current_kp-xr.foc_current_kp)>1e-7f ||
           fabsf(m_motor_2.m_conf.foc_current_ki-xr.foc_current_ki)>1e-5f ||
           fabsf(m_motor_2.m_conf.s_pid_kp-xr.s_pid_kp)>1e-8f ||
           fabsf(m_motor_2.m_conf.s_pid_ki-xr.s_pid_ki)>1e-8f ||
           fabsf(m_motor_2.m_conf.s_pid_kd-xr.s_pid_kd)>1e-9f ||
           fabsf(m_motor_2.m_conf.p_pid_kp-xr.p_pid_kp)>1e-7f ||
           fabsf(m_motor_2.m_conf.p_pid_ki-xr.p_pid_ki)>1e-7f ||
           fabsf(m_motor_2.m_conf.p_pid_kd-xr.p_pid_kd)>1e-8f)
            return fail("right exact PID float32 persistence");
    }

    /* V35 (0x6021) sudah memiliki motor model dan exact PID tetapi belum
     * menyimpan PLL/decoupling. Migrasi wajib memakai default aman lalu append
     * EXT11 dan menulis signature V36 tanpa merusak field lama. */
    ee_value[43]=0x6021u; ee_value[44]=0x6021u;
    for(unsigned i=260u;i<270u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V35 PLL/decoupling migration load");
    if(fabsf(m_motor_1.m_conf.foc_pll_kp-MCCONF_FOC_PLL_KP_DEFAULT)>0.001f ||
       fabsf(m_motor_2.m_conf.foc_pll_ki-MCCONF_FOC_PLL_KI_DEFAULT)>0.001f ||
       m_motor_1.m_conf.foc_cc_decoupling!=FOC_CC_DECOUPLING_DISABLED ||
       m_motor_1.m_conf.s_pid_speed_source!=S_PID_SPEED_SRC_FAST ||
       fabsf(m_motor_1.m_conf.foc_dt_us-MCCONF_FOC_DT_US_DEFAULT)>0.000001f ||
       fabsf(m_motor_2.m_conf.foc_dt_us-MCCONF_FOC_DT_US_DEFAULT)>0.000001f)
        return fail("V35 PLL/decoupling/deadtime migration defaults");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u)
        return fail("V35 migration signature");
    for(unsigned i=260u;i<270u;i++)if(!ee_valid[i])return fail("V35 EXT11 rewrite");

    /* V34 (0x6020) already has Hall-extra but not the R/L/flux model slots.
     * Migration must preserve every old field and append zero "not detected yet"
     * values without treating the absence as corruption. */
    ee_value[43]=0x6020u; ee_value[44]=0x6020u;
    for(unsigned i=207u;i<223u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V34 motor-model migration load");
    if(m_motor_1.m_conf.m_hall_extra_samples!=5 || m_motor_2.m_conf.m_hall_extra_samples!=2)
        return fail("V34 Hall-extra preservation");
    if(fabsf(m_motor_1.m_conf.foc_motor_r)>1e-9f || fabsf(m_motor_2.m_conf.foc_motor_l)>1e-9f ||
       fabsf(m_motor_1.m_conf.foc_motor_flux_linkage)>1e-9f)
        return fail("V34 motor-model migration defaults");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u)
        return fail("V34 migration signature");
    for(unsigned i=207u;i<223u;i++)if(!ee_valid[i])return fail("V34 motor-model slot rewrite");

    /* V33 (0x601F) is the firmware image deployed before Hall-extra support.
     * It already contains all other current MC fields, but slots 205/206 did
     * not exist. Migration must preserve the configuration, seed the upstream
     * VESC default of 3 extra Hall samples, then rewrite the new signature. */
    ee_value[43]=0x601Fu; ee_value[44]=0x601Fu;
    ee_valid[205]=0u; ee_valid[206]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V33 Hall-extra migration load");
    if(m_motor_1.m_conf.m_hall_extra_samples!=3 || m_motor_2.m_conf.m_hall_extra_samples!=3)
        return fail("V33 Hall-extra migration default");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u || !ee_valid[205] || !ee_valid[206] ||
       ee_value[205]!=3u || ee_value[206]!=3u)
        return fail("V33 Hall-extra migration rewrite");

    /* V31 (0x601D) already persisted ABI on quantized x100/x10000 slots.
     * V32 must preserve those legacy values, initialize new user-position fields
     * to upstream defaults, then rewrite the exact append-only schema. */
    ee_value[43]=0x601Du; ee_value[44]=0x601Du;
    for(unsigned i=186u;i<223u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V31 migration load");
    if(m_motor_1.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_ABI ||
       fabsf(m_motor_1.m_conf.foc_encoder_offset-17.25f)>0.011f ||
       fabsf(m_motor_1.m_conf.foc_encoder_ratio-10.1235f)>0.00011f)
        return fail("V31 quantized encoder preservation");
    if(m_motor_1.m_conf.m_invert_direction || !m_motor_2.m_conf.m_invert_direction ||
       fabsf(m_motor_1.m_conf.p_pid_offset)>0.000001f ||
       fabsf(m_motor_1.m_conf.p_pid_ang_div-1.0f)>0.000001f ||
       fabsf(m_motor_1.m_conf.p_pid_kd_proc-0.00035f)>0.0000001f ||
       fabsf(m_motor_1.m_conf.p_pid_gain_dec_angle)>0.000001f)
        return fail("V31 user-field/direction migration defaults");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u)
        return fail("V31 migration signature");
    for(unsigned i=186u;i<223u;i++)if(!ee_valid[i])return fail("V31 V34-slot rewrite");

    /* V30 (0x601C) sudah punya Hall interpolation, tetapi belum menyimpan ABI.
     * Migrasi tidak boleh merusak nilai Hall interpolation lama. */
    ee_value[43]=0x601Cu; ee_value[44]=0x601Cu;
    for(unsigned i=186u;i<223u;i++)ee_valid[i]=0u;
    for(unsigned i=181u;i<=185u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V30 migration load");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-620.0f)>0.5f || fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-900.0f)>0.5f)
        return fail("V30 Hall interpolation preservation");
    if(m_motor_1.m_conf.m_sensor_port_mode!=SENSOR_PORT_MODE_HALL || m_motor_1.m_conf.m_encoder_counts!=4096)
        return fail("V30 encoder default migration");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u || !ee_valid[181] || !ee_valid[185])
        return fail("V30 encoder schema rewrite");

    /* V29 (0x601B) sudah punya seluruh safety persistence tetapi belum punya
     * slot foc_hall_interp_erpm. Migrasi harus memakai default upstream 500
     * ERPM, membangun koefisien runtime, lalu menulis schema/slot baru. */
    ee_value[43]=0x601Bu; ee_value[44]=0x601Bu;
    for(unsigned i=186u;i<223u;i++)ee_valid[i]=0u;
    ee_valid[179]=ee_valid[180]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V29 migration load");
    if(fabsf(m_motor_1.m_conf.foc_hall_interp_erpm-500.0f)>0.5f ||
       fabsf(m_motor_2.m_conf.foc_hall_interp_erpm-500.0f)>0.5f ||
       m_motor_1.m_hall_interp_erpm!=500u || m_motor_2.m_hall_interp_erpm!=500u)
        return fail("V29 Hall interpolation migration default/runtime");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u || !ee_valid[179] || !ee_valid[180] ||
       ee_value[179]!=500u || ee_value[180]!=500u || !ee_valid[181] || !ee_valid[182] ||
       !ee_valid[183] || !ee_valid[184] || !ee_valid[185])
        return fail("V29 migration Hall/encoder slots/signature");

    /* Simulasikan upgrade image V28 (0x601A), yaitu firmware yang sudah punya
     * filter x10000 tetapi belum punya slot Vin/watt/temperatur. Field lama
     * harus tetap utuh, safety baru memakai default aman, lalu schema ditulis
     * ulang atomik menjadi 0x6020. */
    ee_value[43]=0x601Au; ee_value[44]=0x601Au;
    for(unsigned i=186u;i<223u;i++)ee_valid[i]=0u;
    for(unsigned i=163u;i<179u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V28 migration load");
    if(fabsf(m_motor_1.m_conf.l_battery_cut_start-37.20f)>0.011f ||
       fabsf(m_motor_2.m_conf.l_battery_cut_end-32.90f)>0.011f ||
       fabsf(m_motor_1.m_conf.foc_current_filter_const-0.0731f)>0.00011f ||
       fabsf(m_motor_2.m_conf.si_wheel_diameter-0.3100f)>0.00011f)
        return fail("V28 old-field preservation");
    if(fabsf(m_motor_1.m_conf.l_min_vin-MCCONF_L_MIN_VIN)>0.011f ||
       fabsf(m_motor_2.m_conf.l_temp_fet_end-MCCONF_L_TEMP_FET_END)>0.051f)
        return fail("V28 safety default migration");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u)
        return fail("V28 migration signature");
    for(unsigned i=163u;i<179u;i++)if(!ee_valid[i])return fail("V28 migration safety slots");
    if(!ee_valid[179] || !ee_valid[180] || ee_value[179]!=500u || ee_value[180]!=500u ||
       !ee_valid[181] || !ee_valid[185]) return fail("V28 migration Hall/encoder slots");

    /* V27 (0x6019) juga harus tetap diterima. Hilangkan slot filter dan safety
     * untuk meniru image lama, lalu pastikan keduanya dibuat kembali. */
    ee_value[43]=0x6019u; ee_value[44]=0x6019u;
    for(unsigned i=186u;i<223u;i++)ee_valid[i]=0u;
    ee_valid[161]=ee_valid[162]=0u;
    for(unsigned i=163u;i<179u;i++)ee_valid[i]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false) || !mc_interface_load_configuration_motor(true))
        return fail("V27 migration load");
    if(ee_value[43]!=0x6022u || ee_value[44]!=0x6022u || !ee_valid[161] || !ee_valid[162] ||
       !ee_valid[179] || !ee_valid[180] || !ee_valid[181] || !ee_valid[185])
        return fail("V27 migration rewrite/signature");

    /* Power-loss transaction regression. Start from two valid endpoints, then
     * make every EEPROM write fail after a few successful words. The LEFT
     * signature must already have been invalidated, while RIGHT remains valid.
     * Restore the emulated EEPROM afterwards so the legacy corruption test below
     * remains independent. */
    {
        uint16_t saved_value[NB_OF_VAR]; uint8_t saved_valid[NB_OF_VAR];
        memcpy(saved_value,ee_value,sizeof(saved_value));
        memcpy(saved_valid,ee_valid,sizeof(saved_valid));
        mc_configuration interrupted=m_motor_1.m_conf; interrupted.l_current_max=3.21f;
        mcpwm_foc_set_configuration(&interrupted,false);
        ee_write_count=0; ee_fail_after=6;
        if(mc_interface_store_configuration_motor(false))return fail("brownout store must report failure");
        ee_fail_after=-1;
        if(!ee_valid[43] || ee_value[43]!=0u)return fail("brownout must leave LEFT signature invalid");
        if(!ee_valid[44] || ee_value[44]!=0x6022u)return fail("brownout LEFT store must not invalidate RIGHT");
        mcpwm_foc_init();
        if(mc_interface_load_configuration_motor(false))return fail("partial LEFT config must fail closed");
        if(!mc_interface_load_configuration_motor(true))return fail("RIGHT config must survive LEFT brownout");
        memcpy(ee_value,saved_value,sizeof(saved_value));
        memcpy(ee_valid,saved_valid,sizeof(saved_valid));
        ee_write_count=0;
    }

    /* Independent signatures: corrupt only right signature; left remains valid. */
    ee_value[44]=0u;
    mcpwm_foc_init();
    if(!mc_interface_load_configuration_motor(false)) return fail("left must survive right signature corruption");
    if(mc_interface_load_configuration_motor(true)) return fail("right corrupted signature must reject");

    printf("EEPROM_DUAL_PERSISTENCE_PASS leftI=%.2f rightI=%.2f leftHall=%u rightHall=%u leftRamp=%u rightRamp=%u poles=%u/%u gear=%.2f/%.2f\n",
           m_motor_1.m_conf.l_current_max,9.87f,m_motor_1.m_conf.foc_hall_table[1],hr[1],
           m_motor_1.m_speed_ramp_rpm_s,234u,m_motor_1.m_conf.si_motor_poles,m_motor_2.m_conf.si_motor_poles,
           m_motor_1.m_conf.si_gear_ratio,m_motor_2.m_conf.si_gear_ratio);
    return 0;
}
