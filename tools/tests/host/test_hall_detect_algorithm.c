#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"

GPIO_TypeDef _GPIOA={0},_GPIOB={0},_GPIOC={0};
TIM_TypeDef _TIM1={0},_TIM8={0};
DMA_TypeDef _DMA1={0};
DWT_Type _DWT={0};
CoreDebug_Type _CoreDebug={0};
volatile adc_buf_t adc_buffer={0};
uint8_t ctrlModReq=VLT_MODE;

void filtLowPass32(int16_t u, uint16_t coef, int32_t *y){
    int32_t err=(int32_t)u-(*y>>16);
    if(err>32767)err=32767; else if(err<-32768)err=-32768;
    *y+=(int32_t)coef*err;
}

static void set_hall(GPIO_TypeDef *port,uint16_t pu,uint16_t pv,uint16_t pw,uint8_t h){
    port->IDR|=(uint32_t)(pu|pv|pw);
    if(h&4u)port->IDR&=~(uint32_t)pu;
    if(h&2u)port->IDR&=~(uint32_t)pv;
    if(h&1u)port->IDR&=~(uint32_t)pw;
}
static void hold_left_hall(uint8_t h,uint16_t ticks){
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h);
    for(uint16_t i=0u;i<ticks;++i)mcpwm_foc_adc_int_handler();
}
static void force_hall_fixture(void){
    /* This file deliberately validates the Hall detector on both bridges.
     * Production LEFT defaults to ABI encoder, so opt into Hall explicitly. */
    m_motor_1.m_conf.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    m_motor_1.m_conf.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
    m_motor_1.m_encoder_synced=0u;
    m_motor_2.m_conf.m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    m_motor_2.m_conf.foc_sensor_mode=FOC_SENSOR_MODE_HALL;
}

/* Deliberately non-default Hall/phase permutations. Index is 60-degree
 * electrical sector in applied board phase coordinates. */
static uint8_t left_raw_for_sector[6]  ={5u,1u,3u,2u,6u,4u};
static uint8_t right_raw_for_sector[6] ={2u,6u,4u,5u,1u,3u};

static uint8_t permute_hall_bits(uint8_t h,const uint8_t p[3]){
    const uint8_t bits[3]={(uint8_t)((h>>2)&1u),(uint8_t)((h>>1)&1u),(uint8_t)(h&1u)};
    return (uint8_t)((bits[p[0]]<<2)|(bits[p[1]]<<1)|bits[p[2]]);
}

static uint8_t raw_for_phase(uint16_t phase,const uint8_t map[6]){
    /* sector center 0,60,...; boundaries +/-30 deg */
    uint32_t a200=((uint32_t)phase*200u+32768u)/65536u;
    if(a200>=200u)a200-=200u;
    uint32_t sector=((a200+17u)/33u)%6u;
    return map[sector];
}

void HAL_Delay(uint32_t ms){
    /* Hardware keeps the 16-kHz FOC ISR running during the blocking detector.
     * Simulate that here. This regression specifically catches the V14 bug
     * where CONTROL_MODE_OPENLOOP_PHASE was overwritten by openloop_update(). */
    for(uint32_t t=0;t<ms;t++){
        for(uint8_t isr=0;isr<16u;isr++){
            uint8_t hl=raw_for_phase(m_motor_1.m_phase_openloop,left_raw_for_sector);
            uint8_t hr=raw_for_phase(m_motor_2.m_phase_openloop,right_raw_for_sector);
            set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,hl);
            set_hall(GPIOC,RIGHT_HALL_U_PIN,RIGHT_HALL_V_PIN,RIGHT_HALL_W_PIN,hr);
            mcpwm_foc_adc_int_handler();
        }
    }
}

static int circular_diff200(uint8_t a,uint8_t b){
    int d=(int)a-(int)b;
    while(d>100)d-=200;
    while(d<-100)d+=200;
    return d<0?-d:d;
}
static int fail(const char*s){fprintf(stderr,"FAIL %s\n",s);return 1;}

static int validate(const uint8_t t[8],const uint8_t map[6],const char *which){
    if(t[0]!=255u||t[7]!=255u)return fail("invalid-state markers");
    for(uint8_t sec=0;sec<6u;sec++){
        const uint8_t raw=map[sec];
        const uint8_t expected=(uint8_t)((sec*200u+3u)/6u);
        if(t[raw]>=200u || circular_diff200(t[raw],expected)>4){
            fprintf(stderr,"FAIL %s raw=%u got=%u expected~%u\n",which,raw,t[raw],expected);
            return 1;
        }
    }
    return 0;
}

int main(void){
    uint8_t tl[8],tr[8];

    /* Wiring robustness: any permutation of the three Hall signal wires, plus
     * a reversed motor phase sequence, must still be learnable by Hall detect.
     * This models the practical phase/Hall swaps users make at the connector. */
    {
        static const uint8_t perms[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
        static const uint8_t base[6]={5u,1u,3u,2u,6u,4u};
        uint8_t testtab[8]; unsigned cases=0u;
        for(unsigned pi=0;pi<6u;pi++) for(unsigned rev=0;rev<2u;rev++){
            for(unsigned sec=0;sec<6u;sec++){
                const unsigned src=rev?((6u-sec)%6u):sec;
                left_raw_for_sector[sec]=permute_hall_bits(base[src],perms[pi]);
            }
            mcpwm_foc_init(); force_hall_fixture();
            if(!mcpwm_foc_hall_detect(1.0f,false,testtab))return fail("permuted Hall/phase detector returned false");
            if(validate(testtab,left_raw_for_sector,"permuted"))return 1;
            cases++;
        }
        { const uint8_t l0[6]={5u,1u,3u,2u,6u,4u}; const uint8_t r0[6]={2u,6u,4u,5u,1u,3u};
          memcpy(left_raw_for_sector,l0,6); memcpy(right_raw_for_sector,r0,6); }
        printf("HALL_WIRING_PERMUTATION_PASS cases=%u\n",cases);
    }

    mcpwm_foc_init(); force_hall_fixture();
    HAL_Delay(1u);
    if(!mcpwm_foc_hall_detect(1.0f,false,tl))return fail("left detector returned false");
    if(validate(tl,left_raw_for_sector,"left"))return 1;
    { uint8_t t2[8],t3[8];
      if(!mcpwm_foc_hall_detect(1.0f,false,t2) || !mcpwm_foc_hall_detect(1.0f,false,t3))return fail("left repeated detector false");
      if(memcmp(tl,t2,8)!=0 || memcmp(tl,t3,8)!=0)return fail("left 3x detector table repeatability"); }
    if(!mcpwm_foc_hall_detect(1.0f,true,tr))return fail("right detector returned false");
    if(validate(tr,right_raw_for_sector,"right"))return 1;
    { uint8_t t2[8],t3[8];
      if(!mcpwm_foc_hall_detect(1.0f,true,t2) || !mcpwm_foc_hall_detect(1.0f,true,t3))return fail("right repeated detector false");
      if(memcmp(tr,t2,8)!=0 || memcmp(tr,t3,8)!=0)return fail("right 3x detector table repeatability"); }
    /* Upstream mcpwm_foc_hall_detect only returns the table. Applying/storing
     * it belongs to conf_general/command handling. Apply explicitly here before
     * exercising closed-loop Hall interpolation. */
    mc_configuration cl=m_motor_1.m_conf, cr=m_motor_2.m_conf;
    for(int i=0;i<8;i++){cl.foc_hall_table[i]=(int8_t)tl[i];cr.foc_hall_table[i]=(int8_t)tr[i];}
    mcpwm_foc_init(); force_hall_fixture();
    mcpwm_foc_set_configuration(&cl,false);
    mcpwm_foc_set_configuration(&cr,true);

    /* VESC Hall FOC format is 0..199 -> 0..360 electrical degrees; 255 is
     * invalid. Prove the detected center is the phase used by closed-loop FOC. */
    const uint8_t h0=left_raw_for_sector[2];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h0);
    for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
    const uint16_t expected_phase=(uint16_t)(((uint32_t)tl[h0]*65536u)/200u);
    if(m_motor_1.m_phase_hall!=expected_phase)return fail("Hall table 0..199 electrical-angle mapping");
    mcpwm_foc_set_current(0.2f,false); mcpwm_foc_vesc_override_touch(false);
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_phase!=m_motor_1.m_phase_hall)return fail("closed-loop FOC must use corrected Hall phase");

    /* One asynchronous wrong sample must not alter the debounced state/count. */
    const int32_t pos_before=m_motor_1.m_position_counts;
    const uint8_t glitch=left_raw_for_sector[4];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,glitch);
    mcpwm_foc_adc_int_handler();
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h0);
    mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_hall_state!=h0 || m_motor_1.m_position_counts!=pos_before)return fail("single-sample Hall glitch debounce");

    /* A real adjacent code held for the debounce window must be accepted. */
    const uint8_t h1=left_raw_for_sector[3];
    set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,h1);
    for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
    if(m_motor_1.m_hall_state!=h1)return fail("stable Hall transition acceptance");
    if(abs((int)(m_motor_1.m_position_counts-pos_before))!=1)return fail("stable Hall transition position count");

    /* Hall-only fail-safe: unlike full VESC this board has no sensorless
     * observer fallback. A debounced 000/111 reading must therefore drop Hall
     * estimator lock and clear the live torque reference. When a valid Hall
     * code returns, re-sync directly to that calibrated sector without
     * inventing a skipped edge/tachometer count from the stale pre-fault state. */
    {
        const int32_t pos_valid=m_motor_1.m_position_counts;
        const int32_t tach_valid=m_motor_1.m_tachometer;
        const uint32_t reject_valid=m_motor_1.m_hall_sequence_reject_count;
        set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,0u);
        for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
        if(m_motor_1.m_hall_state!=0u || m_motor_1.m_hall_initialized || m_motor_1.m_hall_direction!=0)
            return fail("invalid Hall must drop estimator lock");
        if(m_motor_1.m_state!=MC_STATE_OFF || m_motor_1.m_iq_target_q4!=0 || m_motor_1.m_iq_set_q4!=0)
            return fail("invalid Hall must remove closed-loop torque");
        if(m_motor_1.m_position_counts!=pos_valid || m_motor_1.m_tachometer!=tach_valid)
            return fail("invalid Hall must not create position/tachometer edge");

        const uint8_t reconnect=left_raw_for_sector[0]; /* deliberately non-adjacent to h1 */
        set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,reconnect);
        for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
        if(!m_motor_1.m_hall_initialized || m_motor_1.m_hall_state!=reconnect)
            return fail("valid Hall reconnect must re-sync estimator");
        if(m_motor_1.m_position_counts!=pos_valid || m_motor_1.m_tachometer!=tach_valid)
            return fail("Hall reconnect must not synthesize skipped edge");
        if(m_motor_1.m_hall_sequence_reject_count!=reject_valid)
            return fail("Hall reconnect must not count stale-state sequence reject");
        const uint16_t reconnect_phase=(uint16_t)(((uint32_t)tl[reconnect]*65536u)/200u);
        if(m_motor_1.m_phase_hall!=reconnect_phase)
            return fail("Hall reconnect must use calibrated sector center");
        mcpwm_foc_set_current(0.2f,false); mcpwm_foc_vesc_override_touch(false);
        mcpwm_foc_adc_int_handler();
        if(m_motor_1.m_phase!=m_motor_1.m_phase_hall)
            return fail("closed-loop Hall phase after reconnect");

        /* A stable non-adjacent code is electrically impossible at this
         * sampled speed envelope. Match upstream foc_correct_hall semantics:
         * reject that angle and keep the last accepted electrical phase without
         * collapsing a live current command because of one transient. */
        const uint8_t skipped=left_raw_for_sector[3];
        const uint16_t phase_before_reject=m_motor_1.m_phase_hall;
        const int32_t pos_before_reject=m_motor_1.m_position_counts;
        const int32_t tach_before_reject=m_motor_1.m_tachometer;
        const int16_t iq_before_reject=m_motor_1.m_iq_target_q4;
        const uint32_t rejects_before=m_motor_1.m_hall_sequence_reject_count;
        set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,skipped);
        for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
        if(m_motor_1.m_hall_sequence_reject_count!=rejects_before+1u)
            return fail("stable skipped Hall state must count one sequence reject");
        if(m_motor_1.m_phase_hall!=phase_before_reject)
            return fail("rejected Hall state must not leak into FOC phase");
        if(m_motor_1.m_position_counts!=pos_before_reject || m_motor_1.m_tachometer!=tach_before_reject)
            return fail("rejected Hall state must not create position/tachometer edge");
        if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT ||
           m_motor_1.m_iq_target_q4!=iq_before_reject)
            return fail("single skipped Hall state must preserve the live current command");

        /* Returning to the last accepted Hall state clears the feedback mismatch
         * without synthesizing motion or requiring command re-arming. */
        set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,reconnect);
        for(uint16_t n=0u;n<(uint16_t)m_motor_1.m_hall_filter_window+MCCONF_HALL_DEBOUNCE_SAMPLES+2u;++n)mcpwm_foc_adc_int_handler();
        if(m_motor_1.m_position_counts!=pos_before_reject || m_motor_1.m_tachometer!=tach_before_reject)
            return fail("Hall sequence recovery must not synthesize motion");
        if(m_motor_1.m_control_mode!=CONTROL_MODE_CURRENT ||
           m_motor_1.m_iq_target_q4!=iq_before_reject)
            return fail("Hall sequence recovery must preserve the live command");
        mcpwm_foc_release_motor(false); mcpwm_foc_vesc_override_clear(false);
    }

    /* A malformed Hall table arriving through SET_MCCONF must never replace
     * the last-known-good angle map used by active FOC. */
    uint8_t good_table[8]; memcpy(good_table,m_motor_1.m_conf.foc_hall_table,8);
    mc_configuration bad=m_motor_1.m_conf;
    bad.foc_hall_table[0]=255u; bad.foc_hall_table[7]=255u;
    bad.foc_hall_table[1]=10u; bad.foc_hall_table[2]=11u;
    bad.foc_hall_table[3]=12u; bad.foc_hall_table[4]=13u;
    bad.foc_hall_table[5]=14u; bad.foc_hall_table[6]=15u;
    mcpwm_foc_set_configuration(&bad,false);
    if(memcmp(good_table,m_motor_1.m_conf.foc_hall_table,8)!=0)return fail("malformed runtime Hall table must be rejected");

    /* Reversal/noise regression at ~700-750 ERPM-equivalent edge timing.
     * Sequence follows detected table phase order. One deliberate one-ISR
     * skipped-state glitch must be invisible after debounce. A real reversal
     * must reset period history and cannot be counted as a bad Hall sequence. */
    mcpwm_foc_init(); force_hall_fixture(); mcpwm_foc_set_configuration(&cl,false);
    const uint8_t seq[6]={5u,1u,3u,2u,6u,4u};
    hold_left_hall(seq[0],240u);
    for(uint8_t lap=0u;lap<2u;++lap){
        for(uint8_t k=1u;k<=6u;++k){
            const uint8_t h=seq[k%6u];
            if(lap==1u && k==3u){
                const uint8_t glitch=seq[(k+2u)%6u];
                set_hall(GPIOB,LEFT_HALL_U_PIN,LEFT_HALL_V_PIN,LEFT_HALL_W_PIN,glitch);
                mcpwm_foc_adc_int_handler();
            }
            hold_left_hall(h,220u);
        }
    }
    if(m_motor_1.m_hall_direction!=1)return fail("forward Hall direction after debounce");
    if(m_motor_1.m_hall_sequence_reject_count!=0u)return fail("forward/glitch sequence reject");
    if(m_motor_1.m_hall_period_reject_count!=0u)return fail("forward/glitch period reject");
    /* Reverse from current seq[0] by walking the same six states backward. */
    for(uint8_t k=1u;k<=12u;++k){
        const uint8_t idx=(uint8_t)((6u-(k%6u))%6u);
        hold_left_hall(seq[idx],220u);
    }
    if(m_motor_1.m_hall_direction!=-1)return fail("reverse Hall direction after warmup");
    if(m_motor_1.m_hall_sequence_reject_count!=0u)return fail("reversal sequence reject");
    if(m_motor_1.m_hall_period_reject_count!=0u)return fail("reversal period reject");
    printf("HALL_DETECT_ALGORITHM_PASS repeats=3x(each detect=3F+3R) left=[%u,%u,%u,%u,%u,%u] right=[%u,%u,%u,%u,%u,%u]\n",
           tl[1],tl[2],tl[3],tl[4],tl[5],tl[6],tr[1],tr[2],tr[3],tr[4],tr[5],tr[6]);
    return 0;
}
