#include "flash_update_f103.h"
#include "f103_boot_layout.h"
#include "motor/mcpwm_foc.h"
#include "defines.h"
#include "stm32f1xx_hal.h"
#include <string.h>

static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u)) return false;
    HAL_FLASH_Unlock();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t hw = data[i];
        if (i + 1u < len) hw |= (uint16_t)((uint16_t)data[i + 1u] << 8);
        else hw |= 0xFF00u;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, base + i, hw) != HAL_OK) {
            HAL_FLASH_Lock(); return false;
        }
    }
    HAL_FLASH_Lock();
    return memcmp((const void *)base, data, len) == 0;
}

static void release_both(void) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
}


static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    FLASH_EraseInitTypeDef e = {0};
    uint32_t page_error = 0u;
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    e.PageAddress = base;
    e.NbPages = (bytes + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &page_error);
    HAL_FLASH_Lock();
    return st == HAL_OK && page_error == 0xFFFFFFFFu;
}

bool f103_fw_erase_staging(uint32_t fw_size) {
    /* The 120-KiB internal staging slot no longer exists. Field upload must
     * enter the resident bootloader first; the candidate is retained by the
     * NUC/F411 host and streamed into the 240-KiB active region. Never erase
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
    f103_update_meta_t m = *cur;
    m.state = F103_UPDATE_STATE_CONFIRMED;
    m.test_attempt = 0xFFFFu;
    m.test_attempt_inv = 0xFFFFu;
    /* Fail-closed confirmation: power loss during erase/program leaves invalid
     * metadata, and resident bootloader stays in recovery rather than booting unknown bytes. */
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    return program_halfwords(F103_META_BASE_ADDR, (const uint8_t *)&m, sizeof(m));
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
