#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motor/foc_math.h"
static int fail(const char *m){fprintf(stderr,"FAIL %s\n",m);return 1;}

int main(void){
  int16_t s=0,c=0; foc_sin_cos_q15(0,&s,&c); if(s!=0||c!=32767)return fail("sin/cos cardinal 0");
  foc_sin_cos_q15(16384u,&s,&c); if(s!=32767||c!=0)return fail("sin/cos cardinal 90");
  foc_sin_cos_q15(32768u,&s,&c); if(s!=0||c!=-32767)return fail("sin/cos cardinal 180");
  foc_sin_cos_q15(49152u,&s,&c); if(s!=-32767||c!=0)return fail("sin/cos cardinal 270");
  int max_trig_err=0;
  for(uint32_t ph=0;ph<65536u;ph++){
    foc_sin_cos_q15((uint16_t)ph,&s,&c);
    const double a=6.28318530717958647692*(double)ph/65536.0;
    const int sr=(int)lround(sin(a)*32767.0), cr=(int)lround(cos(a)*32767.0);
    const int es=abs((int)s-sr), ec=abs((int)c-cr);
    if(es>max_trig_err)max_trig_err=es;
    if(ec>max_trig_err)max_trig_err=ec;
    if(es>4||ec>4)return fail("sin/cos LUT interpolation error");
  }
  double max_sqrt_rel=0.0;
  for(int e=-20;e<=20;e++){
    for(int m=1;m<=97;m+=3){
      const double x=ldexp((double)m/97.0,e);
      const double ref=sqrt(x);
      const double got=(double)foc_sqrtf_slow((float)x);
      const double rel=fabs(got-ref)/(ref>0.0?ref:1.0);
      if(rel>max_sqrt_rel)max_sqrt_rel=rel;
      if(rel>2.0e-6)return fail("slow sqrt relative error");
    }
  }
  if(foc_sqrtf_slow(0.0f)!=0.0f||foc_sqrtf_slow(-1.0f)!=0.0f)return fail("slow sqrt nonpositive");
  double max_atan_err=0.0;
  for(int k=0;k<3600;k++){
    const double a=6.28318530717958647692*(double)k/3600.0;
    const int32_t x=(int32_t)lround(cos(a)*1000000.0);
    const int32_t y=(int32_t)lround(sin(a)*1000000.0);
    const uint16_t ph=foc_atan2_phase_u16(y,x);
    const int32_t ref=(int32_t)lround((double)k*65536.0/3600.0)&0xffff;
    int32_t de=(int32_t)ph-ref; if(de>32767)de-=65536; if(de<-32768)de+=65536;
    const double err=fabs((double)de*360.0/65536.0);
    if(err>max_atan_err)max_atan_err=err;
    if(err>0.40)return fail("fixed atan2 phase error");
  }
  /* Dead-time phase-sign helper harus identik dengan rumus upstream. Hindari
   * titik tepat pada zero crossing karena SIGN(0) memang diskontinu. */
  int32_t dtsa=0,dtsb=0;
  foc_deadtime_sign_q15(1000,0,&dtsa,&dtsb);
  if(abs(dtsa-43690)>1||dtsb!=0)return fail("deadtime sign cardinal alpha+");
  foc_deadtime_sign_q15(-1000,0,&dtsa,&dtsb);
  if(abs(dtsa+43690)>1||dtsb!=0)return fail("deadtime sign cardinal alpha-");
  foc_deadtime_sign_q15(0,1000,&dtsa,&dtsb);
  if(dtsa!=0||dtsb!=2*FOC_INV_SQRT3_Q15)return fail("deadtime sign cardinal beta+");
  for(int32_t aa=-3000;aa<=3000;aa+=137){
    for(int32_t bb=-3000;bb<=3000;bb+=149){
      const double pa=(double)aa;
      const double pb=-0.5*(double)aa+0.8660254037844386*(double)bb;
      const double pc=-0.5*(double)aa-0.8660254037844386*(double)bb;
      if(fabs(pa)<1.0||fabs(pb)<1.0||fabs(pc)<1.0)continue;
      const int sa=(pa>0)-(pa<0), sb=(pb>0)-(pb<0), sc=(pc>0)-(pc<0);
      const int32_t ra=(int32_t)lround(((2.0*sa-sb-sc)/3.0)*32768.0);
      const int32_t rb=(int32_t)lround(((sb-sc)/sqrt(3.0))*32768.0);
      foc_deadtime_sign_q15((int16_t)aa,(int16_t)bb,&dtsa,&dtsb);
      if(abs(dtsa-ra)>1||abs(dtsb-rb)>2)return fail("deadtime sign differential");
    }
  }
  foc_ab_t ab={1200,-700},ab2={0,0}; foc_dq_t dq={0,0};
  foc_park_q4(&ab,10000,&dq); foc_inv_park(&dq,10000,&ab2);
  if(abs(ab.alpha-ab2.alpha)>3||abs(ab.beta-ab2.beta)>3)return fail("park roundtrip");
  /* ISR sqrt harus exact integer sqrt tetapi berbasis Flash LUT, bukan loop
   * restoring bit-by-bit. Uji seluruh rentang kecil dan sampel uint32 penuh. */
  for(uint32_t x=0u;x<1000000u;++x){
    uint32_t r=foc_isqrt_u32(x);
    uint32_t ref=(uint32_t)sqrt((double)x);
    if(r!=ref)return fail("sqrt LUT exact small range");
  }
  uint32_t prng=0x13579bdfu;
  for(uint32_t k=0u;k<200000u;++k){
    prng=prng*1664525u+1013904223u;
    uint32_t r=foc_isqrt_u32(prng);
    uint32_t ref=(uint32_t)sqrt((double)prng);
    if(r!=ref)return fail("sqrt LUT exact uint32 sample");
  }
  foc_dq_t v={20000,20000};foc_vector_limit(&v,14400);
  uint32_t mag=(uint32_t)((int32_t)v.d*v.d+(int32_t)v.q*v.q); if(mag>(uint32_t)14420u*14420u)return fail("vector limit");
  foc_abc_t pwm;foc_centered_svpwm(&v,12345,&pwm);
  if(abs(pwm.a)>1000||abs(pwm.b)>1000||abs(pwm.c)>1000)return fail("svpwm range");
  /* Prove both limits separately: the EFeru electrical ceiling itself and the
   * board-specific VESC normalization chosen in config.h. */
  int max_abs=0, max_span=0;
  foc_dq_t vmax={FOC_SVPWM_VECTOR_FULL_SAFE,0};
  for(int deg=0;deg<360;deg++){
    foc_abc_t x;
    uint16_t ph=(uint16_t)(((uint32_t)deg*65536u)/360u);
    foc_centered_svpwm(&vmax,ph,&x);
    int vals[3]={x.a,x.b,x.c};
    int mx=vals[0],mn=vals[0];
    for(int j=0;j<3;j++){int av=abs(vals[j]);if(av>max_abs)max_abs=av;if(vals[j]>mx)mx=vals[j];if(vals[j]<mn)mn=vals[j];}
    if(mx-mn>max_span)max_span=mx-mn;
    if(mx>890||mn<-890)return fail("EFeru full-safe PWM exceeds +/-890");
  }
  if(max_abs<887||max_abs>890)return fail("EFeru full-safe PWM does not reach physical ceiling");
  int scaled_abs=0;
  foc_dq_t vscaled={FOC_SVPWM_VECTOR_MAX,0};
  for(int deg=0;deg<360;deg++){
    foc_abc_t x; uint16_t ph=(uint16_t)(((uint32_t)deg*65536u)/360u);
    foc_centered_svpwm(&vscaled,ph,&x);
    int vals[3]={x.a,x.b,x.c};
    for(int j=0;j<3;j++){int av=abs(vals[j]);if(av>scaled_abs)scaled_abs=av;}
  }
  if(FOC_SVPWM_VECTOR_MAX!=(FOC_SVPWM_VECTOR_FULL_SAFE*VESC_DUTY_PHYSICAL_SCALE_PERMILLE)/1000)
    return fail("config duty scale arithmetic");
  if(scaled_abs<850||scaled_abs>856)return fail("0.960 physical duty scale PWM range");
  printf("FOC_FIXEDPOINT_RUNTIME_PASS lut_max_err=%d atan_max_deg=%.3f sqrt_rel=%.3g pwm=%d,%d,%d eferu_max_abs=%d scaled960_abs=%d span=%d\n",max_trig_err,max_atan_err,max_sqrt_rel,pwm.a,pwm.b,pwm.c,max_abs,scaled_abs,max_span);
  return 0;
}
