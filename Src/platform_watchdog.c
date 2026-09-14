#include <string.h>
#include "stm32f1xx_hal.h"
#include "platform_watchdog.h"
#include "motor/mcpwm_foc.h"

extern volatile uint32_t boot_reset_csr;
extern volatile uint32_t boot_reset_reason;
extern volatile uint32_t boot_reset_stage;

#define PLATFORM_IWDG_SERVICE_MS 100u
#define PLATFORM_IWDG_RELOAD     313u /* ~2.0 s at nominal 40-kHz LSI, /256 */

static volatile uint32_t s_feed_count=0u;
static volatile uint32_t s_reject_count=0u;
static volatile uint32_t s_last_feed_ms=0u;
static volatile uint32_t s_last_adc_heartbeat=0u;
static volatile uint32_t s_last_motor_heartbeat[2]={0u,0u};
static volatile uint8_t s_enabled=0u;
static volatile uint8_t s_last_health_ok=0u;
static volatile uint8_t s_init_failed=0u;
static volatile uint8_t s_init_fail_stage=0u;

static inline void platform_iwdg_reload(void) {
#ifdef STM32F103xE
    IWDG->KR=0xAAAAu;
#endif
}

void platform_watchdog_maintenance_kick(void) {
#ifdef STM32F103xE
    if (s_enabled && !s_init_failed) {
        platform_iwdg_reload();
        s_last_feed_ms=HAL_GetTick();
        s_feed_count++;
    }
#endif
}

void platform_watchdog_init(void) {
    uint32_t hb=0u, mh[2]={0u,0u};
    mcpwm_foc_get_liveness(&hb,mh);
    s_last_adc_heartbeat=hb;
    s_last_motor_heartbeat[0]=mh[0];
    s_last_motor_heartbeat[1]=mh[1];
    s_last_feed_ms=HAL_GetTick();
#ifdef STM32F103xE
    s_init_failed=0u;
    s_init_fail_stage=0u;
    /* STM32F1 requires the watchdog to be STARTED before PR/RLR update flags
     * can complete. Use ST's HAL sequence exactly: START -> write access ->
     * PR/RLR -> wait update -> reload. The previous code waited on SR before
     * START and therefore failed closed on real F103 hardware. */
    IWDG_HandleTypeDef hiwdg={0};
    hiwdg.Instance=IWDG;
    hiwdg.Init.Prescaler=IWDG_PRESCALER_256;
    hiwdg.Init.Reload=PLATFORM_IWDG_RELOAD;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        s_init_failed=1u; s_init_fail_stage=1u; goto fail_closed;
    }
    if ((IWDG->SR & (IWDG_SR_PVU|IWDG_SR_RVU)) != 0u) {
        s_init_failed=1u; s_init_fail_stage=2u; goto fail_closed;
    }
#endif
    s_enabled=1u; s_last_health_ok=1u; return;
fail_closed:
    s_enabled=0u; s_last_health_ok=0u;
    mcpwm_foc_release_motor(false); mcpwm_foc_release_motor(true);
}

void platform_watchdog_service(void) {
    if (s_init_failed) {
        mcpwm_foc_release_motor(false); mcpwm_foc_release_motor(true);
        return;
    }
    if(!s_enabled)return;
    const uint32_t now=HAL_GetTick();
    if((uint32_t)(now-s_last_feed_ms)<PLATFORM_IWDG_SERVICE_MS)return;
    uint32_t hb=0u, mh[2]={0u,0u};
    mcpwm_foc_get_liveness(&hb,mh);
    const bool healthy=(hb!=s_last_adc_heartbeat) &&
                       (mh[0]!=s_last_motor_heartbeat[0]) &&
                       (mh[1]!=s_last_motor_heartbeat[1]);
    s_last_health_ok=healthy?1u:0u;
    if(healthy){
        s_last_adc_heartbeat=hb;
        s_last_motor_heartbeat[0]=mh[0];
        s_last_motor_heartbeat[1]=mh[1];
        s_last_feed_ms=now;
        s_feed_count++;
        platform_iwdg_reload();
    }else{
        /* Do not move last_feed_ms. If hard-realtime progress stays stale the
         * hardware watchdog is intentionally allowed to reset the MCU. */
        s_reject_count++;
    }
}

bool platform_watchdog_boot_was_iwdg(void) {
#ifdef RCC_CSR_IWDGRSTF
    return (boot_reset_csr & RCC_CSR_IWDGRSTF)!=0u;
#else
    return false;
#endif
}

void platform_watchdog_get_status(platform_watchdog_status_t *out) {
    if(!out)return;
    memset(out,0,sizeof(*out));
    out->boot_reset_csr=boot_reset_csr;
    out->boot_reset_reason=boot_reset_reason;
    out->boot_reset_stage=boot_reset_stage;
    out->feed_count=s_feed_count;
    out->reject_count=s_reject_count;
    out->last_adc_heartbeat=s_last_adc_heartbeat;
    out->last_motor_heartbeat[0]=s_last_motor_heartbeat[0];
    out->last_motor_heartbeat[1]=s_last_motor_heartbeat[1];
    out->last_feed_ms=s_last_feed_ms;
    out->enabled=s_enabled;
    out->last_health_ok=s_last_health_ok;
    out->boot_was_iwdg=platform_watchdog_boot_was_iwdg()?1u:0u;
    out->init_failed=s_init_failed;
    out->init_fail_stage=s_init_fail_stage;
#ifdef STM32F103xE
    out->iwdg_sr=IWDG->SR;
    out->iwdg_pr=IWDG->PR;
    out->iwdg_rlr=IWDG->RLR;
#endif
}
