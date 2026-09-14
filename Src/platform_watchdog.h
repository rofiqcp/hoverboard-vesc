#ifndef PLATFORM_WATCHDOG_H_
#define PLATFORM_WATCHDOG_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t boot_reset_csr;
    uint32_t boot_reset_reason;
    uint32_t boot_reset_stage;
    uint32_t feed_count;
    uint32_t reject_count;
    uint32_t last_adc_heartbeat;
    uint32_t last_motor_heartbeat[2];
    uint32_t last_feed_ms;
    uint8_t enabled;
    uint8_t last_health_ok;
    uint8_t boot_was_iwdg;
    uint8_t init_failed;
    uint8_t init_fail_stage;
    uint32_t iwdg_sr;
    uint32_t iwdg_pr;
    uint32_t iwdg_rlr;
} platform_watchdog_status_t;

void platform_watchdog_init(void);
void platform_watchdog_maintenance_kick(void);
#ifdef STM32F103xE
void platform_watchdog_service(void);
#else
static inline void platform_watchdog_service(void) {}
#endif
void platform_watchdog_get_status(platform_watchdog_status_t *out);
bool platform_watchdog_boot_was_iwdg(void);

#endif
