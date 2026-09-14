#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;

static uint16_t image_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc=0u;
    for(uint32_t i=0u;i<len;++i){
        crc^=(uint16_t)data[i]<<8;
        for(uint8_t b=0u;b<8u;++b) crc=(crc&0x8000u)?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1);
    }
    return crc;
}

static bool running_vector_valid(void) {
    const uint32_t sp=*(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv=*(const uint32_t *)(F103_APP_BASE_ADDR+4u);
    if(sp<0x20000000u || sp>F103_BOOT_REQUEST_ADDR || (sp&3u)) return false;
    if((rv&1u)==0u) return false;
    const uint32_t pc=rv&~1u;
    return pc>=F103_APP_BASE_ADDR && pc<(F103_APP_BASE_ADDR+F103_APP_REGION_SIZE);
}

static void release_both(void) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
}


bool f103_fw_erase_staging(uint32_t fw_size) {
    /* The 120-KiB internal staging slot no longer exists. Field upload must
     * enter the resident bootloader first; the candidate is retained by the
     * PC/NUC host and streamed into the 240-KiB active region. Never erase
     * application flash while the motor-control application is executing. */
    (void)fw_size;
    stage_session_active = false;
    stage_session_total = 0u;
    return false;
}

bool f103_fw_write_staging(uint32_t offset, const uint8_t *data, uint32_t len) {
    (void)offset;
    (void)data;
    (void)len;
    return false;
}

bool f103_fw_test_pending(void) {
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (m->magic != F103_UPDATE_META_MAGIC || m->state != F103_UPDATE_STATE_TEST) return false;
    if (m->version != F103_UPDATE_META_VERSION ||
        (uint16_t)(m->version ^ m->version_inv) != 0xFFFFu) return false;
    if (m->size == 0u || m->size > F103_APP_REGION_SIZE || m->size != ~m->size_inv) return false;
    return (uint16_t)(m->crc16 ^ m->crc16_inv) == 0xFFFFu;
}

bool f103_fw_confirm_running_image(void) {
    if (!f103_fw_test_pending()) return true;
    release_both();
    const f103_update_meta_t *cur = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!running_vector_valid()) {
        *(volatile uint32_t *)F103_RESET_STAGE_ADDR = F103_STAGE_PROBATION_VECTOR_FAIL;
        return false;
    }
    if (image_crc16((const uint8_t *)F103_APP_BASE_ADDR, cur->size) != cur->crc16) {
        *(volatile uint32_t *)F103_RESET_STAGE_ADDR = F103_STAGE_PROBATION_CRC_FAIL;
        return false;
    }
    /* Never erase/program internal flash from the running motor application.
     * STM32F1 is single-bank; an IRQ fetch during metadata erase can strand a
     * healthy TEST image. Hand the verified probation result to the resident
     * bootloader through reset-persistent SRAM instead. */
    *(volatile uint32_t *)F103_RESET_STAGE_ADDR = F103_STAGE_PROBATION_CONFIRM_OK;
    *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_TEST_OK;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}

void f103_fw_reset_to_bootloader(void) {
    release_both();
    volatile uint32_t *const request = (volatile uint32_t *)F103_BOOT_REQUEST_ADDR;
    request[0] = F103_BOOT_REQUEST_MAGIC;
    request[1] = F103_BOOT_REQUEST_MAGIC_INV;
    *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_FW_UPDATE;
    __DSB();
    __ISB();
    NVIC_SystemReset();
    for (;;) { }
}
