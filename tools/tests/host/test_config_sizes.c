#include <stdio.h>
#include <string.h>
#include <math.h>
#include "vesc/mcconf_serial.h"

static int nearf(float a,float b,float e){ return fabsf(a-b)<=e; }
int main(void){
  static uint8_t mb[2048],ab[2048]; mc_configuration m,r; app_configuration a;
  memset(&m,0,sizeof(m)); memset(&r,0,sizeof(r)); memset(&a,0,sizeof(a));
  m.foc_hfi_amb_mode=FOC_AMB_MODE_D_SINGLE_PULSE; m.foc_hfi_amb_current=12.3f; m.foc_hfi_amb_tres=7;
  m.foc_hfi_max_err=0.27f; m.foc_hfi_reset_erpm=456.0f; m.foc_fw_backoff=1.234f;
  m.foc_short_ls_on_zero_duty=true; m.foc_overmod_factor=0.95f; m.foc_mag_vd_max=0.91f;
  m.s_pid_kp=0.123f; m.s_pid_ki=0.456f; m.s_pid_kd=0.007f; m.s_pid_ramp_erpms_s=4321.0f;
  m.foc_sl_erpm_start=2345.0f;
  m.foc_control_sample_mode=FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL;
  m.foc_current_sample_mode=FOC_CURRENT_SAMPLE_MODE_BEST_SENSOR;
  m.s_pid_speed_source=S_PID_SPEED_SRC_FASTER;
  int32_t mn=confgenerator_serialize_mcconf(mb,&m), an=confgenerator_serialize_appconf(ab,&a);
  if(!confgenerator_deserialize_mcconf(mb,&r)) return 2;
  if(r.s_pid_speed_source!=S_PID_SPEED_SRC_FASTER || !nearf(r.foc_fw_backoff,1.234f,0.002f)) return 3;
  if(!nearf(r.foc_sl_erpm_start,2345.0f,0.1f) ||
     r.foc_control_sample_mode!=FOC_CONTROL_SAMPLE_MODE_V0_V7_INTERPOL ||
     r.foc_current_sample_mode!=FOC_CURRENT_SAMPLE_MODE_BEST_SENSOR) return 4;
  if(r.foc_hfi_amb_mode!=FOC_AMB_MODE_D_SINGLE_PULSE || r.foc_hfi_amb_tres!=7) return 4;
  if(!nearf(r.foc_hfi_amb_current,12.3f,0.11f) || !nearf(r.foc_hfi_max_err,0.27f,0.002f)) return 5;
  if(!nearf(r.foc_hfi_reset_erpm,456.0f,0.1f) || !r.foc_short_ls_on_zero_duty) return 6;
  if(!nearf(r.foc_overmod_factor,0.95f,0.0002f) || !nearf(r.foc_mag_vd_max,0.91f,0.0002f)) return 7;
  if(!nearf(r.s_pid_ramp_erpms_s,4321.0f,0.1f)) return 8;
  printf("mcconf=%ld appconf=%ld speed_src=%u\n",(long)mn,(long)an,(unsigned)r.s_pid_speed_source);
  return (mn>0 && mn<=700 && an>0 && an<=700)?0:1;
}
