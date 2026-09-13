#include "stm32f1xx_hal.h"
#include "vesc/f103_boot_layout.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

#define COMM_FW_VERSION          0u
#define COMM_JUMP_TO_BOOTLOADER  1u
#define COMM_ERASE_NEW_APP       2u
#define COMM_WRITE_NEW_APP_DATA  3u
#define COMM_REBOOT             29u
#define COMM_ALIVE              30u
#define COMM_CUSTOM_APP_DATA    36u

#define HB_MAGIC0               0x48u
#define HB_MAGIC1               0x42u
#define HB_VERSION              1u
#define HB_BOOT_GET_INFO        0xF0u
#define HB_BOOT_READ_APP        0xF1u

#define RX_MAX_PAYLOAD 512u
#define RECOVERY_IDLE_BLINK_MS  250u

static UART_HandleTypeDef huart3;

static inline void f103_debug_keepalive(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    uint32_t mapr=AFIO->MAPR;
    mapr&=~AFIO_MAPR_SWJ_CFG_Msk;
    mapr|=AFIO_MAPR_SWJ_CFG_RESET;
    AFIO->MAPR=mapr;
    DBGMCU->CR|=DBGMCU_CR_DBG_IWDG_STOP;
    __DSB();
    __ISB();
}

void SysTick_Handler(void) { HAL_IncTick(); }
static uint8_t rx_payload[RX_MAX_PAYLOAD];

/* Candidate storage is external to the F103 (PC/NUC host). The resident
 * bootloader writes only one active page at a time and journals completed
 * pages in the 2-KiB metadata page. */
static bool stage_session_active = false;
static uint32_t stage_session_total = 0u;
static uint32_t stream_expected_app_offset = 0u;

/* SWD-readable recovery diagnostics; no protocol or motor-side effect. */
volatile uint32_t boot_diag_rx_bytes = 0u;
volatile uint32_t boot_diag_start_frames = 0u;
volatile uint32_t boot_diag_packets_ok = 0u;
volatile uint32_t boot_diag_crc_errors = 0u;
volatile uint32_t boot_diag_tx_replies = 0u;
volatile uint32_t boot_diag_uart_errors = 0u;
volatile uint32_t boot_diag_copy_code = 0u;
volatile uint32_t boot_diag_copy_page = 0u;
volatile uint32_t boot_diag_copy_addr = 0u;
volatile uint32_t boot_diag_copy_size = 0u;
volatile uint32_t boot_diag_copy_crc_stage = 0u;
volatile uint32_t boot_diag_copy_crc_app = 0u;

static __attribute__((noreturn)) void boot_fault_hold(uint32_t code) {
    __disable_irq();
    boot_diag_copy_code = code;
    for (;;) { f103_debug_keepalive(); __NOP(); }
}
void HardFault_Handler(void) { boot_fault_hold(0x48415244u); }
void MemManage_Handler(void) { boot_fault_hold(0x4D454D46u); }
void BusFault_Handler(void) { boot_fault_hold(0x42555346u); }
void UsageFault_Handler(void) { boot_fault_hold(0x55534147u); }

static uint16_t crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0u;
    for (uint32_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0u; b < 8u; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint16_t be16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0] << 8) | p[1]); }

static void safe_gpio_init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();

    /* Hoverboard half-bridges: high-side pins LOW, complementary low-side pins HIGH. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); /* keep power latch on */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); /* buzzer off */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); /* LED off */

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8; HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_4; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15; HAL_GPIO_Init(GPIOB, &g);

    g.Mode = GPIO_MODE_AF_PP; g.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOB, &g);
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_NOPULL; g.Pin = GPIO_PIN_11; HAL_GPIO_Init(GPIOB, &g);
}


static bool boot_clock_init(void) {
    /* Match the application clock tree so USART3 can run the project-wide
     * high-speed VESC transport: HSI/2 * 16 = 64 MHz SYSCLK, APB1 = 32 MHz.
     * HAL_Init() is called first with a correct 8-MHz SystemCoreClock model,
     * therefore the oscillator-switch timeouts and SysTick are valid. */
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    osc.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return false;
    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) == HAL_OK;
}

static bool uart_init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = F103_VESC_UART_BAUD;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) { ++boot_diag_uart_errors; return false; }
    return true;
}

#define RAMFUNC __attribute__((section(".ramfunc"), noinline, long_call))
#define FLASH_ERROR_MASK (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)

static RAMFUNC bool ram_flash_wait_ready(uint32_t guard) {
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
        if (guard-- == 0u) return false;
    }
    return true;
}

static RAMFUNC bool ram_flash_unlock(void) {
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
    return (FLASH->CR & FLASH_CR_LOCK) == 0u;
}

static RAMFUNC void ram_flash_clear_status(void) {
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
}

static RAMFUNC bool ram_flash_erase_page(uint32_t address) {
    if (!ram_flash_wait_ready(8000000u) || !ram_flash_unlock()) return false;
    ram_flash_clear_status();
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR |= FLASH_CR_STRT;
    const bool ready = ram_flash_wait_ready(8000000u);
    const uint32_t sr = FLASH->SR;
    FLASH->CR &= ~FLASH_CR_PER;
    FLASH->CR |= FLASH_CR_LOCK;
    ram_flash_clear_status();
    return ready && (sr & FLASH_ERROR_MASK) == 0u;
}

static RAMFUNC bool ram_flash_program_block(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u) != 0u || !ram_flash_wait_ready(8000000u) || !ram_flash_unlock()) return false;
    ram_flash_clear_status();
    for (uint32_t i = 0u; i < len; i += 2u) {
        uint16_t wanted = data[i];
        wanted |= (uint16_t)((i + 1u < len ? data[i + 1u] : 0xFFu) << 8);
        volatile uint16_t *dst = (volatile uint16_t *)(base + i);
        const uint16_t current = *dst;
        if (current == wanted) continue;
        if (current != 0xFFFFu) {
            FLASH->CR |= FLASH_CR_LOCK;
            return false;
        }
        FLASH->CR |= FLASH_CR_PG;
        *dst = wanted;
        const bool ready = ram_flash_wait_ready(1000000u);
        const uint32_t sr = FLASH->SR;
        FLASH->CR &= ~FLASH_CR_PG;
        ram_flash_clear_status();
        if (!ready || (sr & FLASH_ERROR_MASK) != 0u || *dst != wanted) {
            FLASH->CR |= FLASH_CR_LOCK;
            return false;
        }
    }
    FLASH->CR |= FLASH_CR_LOCK;
    return true;
}

static bool erase_pages(uint32_t base, uint32_t bytes) {
    if ((base & (F103_FLASH_PAGE_SIZE - 1u)) != 0u || bytes == 0u) return false;
    const uint32_t pages = (bytes + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    for (uint32_t page = 0u; page < pages; ++page) {
        if (!ram_flash_erase_page(base + page * F103_FLASH_PAGE_SIZE)) return false;
    }
    return true;
}

static bool program_halfwords(uint32_t base, const uint8_t *data, uint32_t len) {
    if (!data || (base & 1u) != 0u) return false;
    if (!ram_flash_program_block(base, data, len)) return false;
    return memcmp((const void *)base, data, len) == 0;
}

static bool erase_one_page(uint32_t address) {
    return erase_pages(address, F103_FLASH_PAGE_SIZE);
}

static bool app_vector_valid(void) {
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    if (sp < 0x20000000u || sp > F103_BOOT_REQUEST_ADDR || (sp & 3u)) return false;
    if ((rv & 1u) == 0u) return false;
    const uint32_t pc = rv & ~1u;
    return pc >= F103_APP_BASE_ADDR && pc < (F103_APP_BASE_ADDR + F103_APP_REGION_SIZE);
}

static bool meta_common_valid(const f103_update_meta_t *m) {
    if (!m || m->magic != F103_UPDATE_META_MAGIC) return false;
    if (m->size == 0u || m->size > F103_APP_REGION_SIZE || m->size != ~m->size_inv) return false;
    if (m->version != F103_UPDATE_META_VERSION ||
        (uint16_t)(m->version ^ m->version_inv) != 0xFFFFu) return false;
    return m->state == F103_UPDATE_STATE_STREAM || m->state == F103_UPDATE_STATE_TEST ||
           m->state == F103_UPDATE_STATE_RECOVERY || m->state == F103_UPDATE_STATE_CONFIRMED;
}

static bool meta_crc_valid(const f103_update_meta_t *m) {
    return meta_common_valid(m) && (uint16_t)(m->crc16 ^ m->crc16_inv) == 0xFFFFu;
}

static bool meta_crc_blank(const f103_update_meta_t *m) {
    return meta_common_valid(m) && m->crc16 == 0xFFFFu && m->crc16_inv == 0xFFFFu;
}

static bool confirmed_app_valid(const f103_update_meta_t *m) {
    return meta_crc_valid(m) && m->state == F103_UPDATE_STATE_CONFIRMED &&
           app_vector_valid() && crc16((const uint8_t *)F103_APP_BASE_ADDR, m->size) == m->crc16;
}

static uint16_t page_marker(uint32_t page) {
    return (uint16_t)(F103_UPDATE_JOURNAL_MARK_BASE | (page & 0xFFu));
}

static bool journal_page_complete(uint32_t page) {
    if (page >= (F103_APP_REGION_SIZE / F103_FLASH_PAGE_SIZE)) return false;
    const volatile uint16_t *p = (const volatile uint16_t *)(F103_META_BASE_ADDR +
                                  F103_UPDATE_JOURNAL_OFFSET + page * 2u);
    return *p == page_marker(page);
}

static uint32_t journal_completed_bytes(const f103_update_meta_t *m) {
    if (!meta_common_valid(m) || m->state != F103_UPDATE_STATE_STREAM) return 0u;
    const uint32_t pages = (m->size + F103_FLASH_PAGE_SIZE - 1u) / F103_FLASH_PAGE_SIZE;
    uint32_t done = 0u;
    for (uint32_t page = 0u; page < pages; ++page) {
        if (!journal_page_complete(page)) break;
        const uint32_t next = (page + 1u) * F103_FLASH_PAGE_SIZE;
        done = next < m->size ? next : m->size;
    }
    return done;
}

static bool journal_mark_page(uint32_t page) {
    if (journal_page_complete(page)) return true;
    const uint16_t marker = page_marker(page);
    return program_halfwords(F103_META_BASE_ADDR + F103_UPDATE_JOURNAL_OFFSET + page * 2u,
                             (const uint8_t *)&marker, sizeof(marker));
}

static bool write_meta(uint32_t state, uint32_t size, uint16_t crc, bool crc_known) {
    f103_update_meta_t m;
    memset(&m, 0xFF, sizeof(m));
    m.magic = F103_UPDATE_META_MAGIC;
    m.state = state;
    m.size = size;
    m.size_inv = ~size;
    if (crc_known) {
        m.crc16 = crc;
        m.crc16_inv = (uint16_t)~crc;
    }
    m.version = F103_UPDATE_META_VERSION;
    m.version_inv = (uint16_t)~F103_UPDATE_META_VERSION;
    if (!erase_pages(F103_META_BASE_ADDR, F103_META_REGION_SIZE)) return false;
    return program_halfwords(F103_META_BASE_ADDR, (const uint8_t *)&m, sizeof(m));
}

static bool set_stream_crc(uint16_t crc) {
    f103_update_meta_t *m = (f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!meta_common_valid(m) || m->state != F103_UPDATE_STATE_STREAM) return false;
    if (meta_crc_valid(m)) return m->crc16 == crc;
    if (!meta_crc_blank(m)) return false;
    uint16_t pair[2] = {crc, (uint16_t)~crc};
    return program_halfwords(F103_META_BASE_ADDR + offsetof(f103_update_meta_t, crc16),
                             (const uint8_t *)pair, sizeof(pair));
}

static bool test_attempt_blank(const f103_update_meta_t *m) {
    return m->test_attempt == 0xFFFFu && m->test_attempt_inv == 0xFFFFu;
}

static bool mark_test_attempt(void) {
    uint16_t pair[2] = {F103_TEST_ATTEMPT_MAGIC, (uint16_t)~F103_TEST_ATTEMPT_MAGIC};
    return program_halfwords(F103_META_BASE_ADDR + offsetof(f103_update_meta_t, test_attempt),
                             (const uint8_t *)pair, sizeof(pair));
}

static bool stream_begin(uint32_t size, bool crc_known, uint16_t crc, uint32_t *resume_stream_off) {
    if (size == 0u || size > F103_APP_REGION_SIZE) return false;
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    bool reusable = meta_common_valid(m) && m->state == F103_UPDATE_STATE_STREAM && m->size == size;
    if (reusable && crc_known && meta_crc_valid(m) && m->crc16 != crc) reusable = false;
    if (!reusable) {
        if (!write_meta(F103_UPDATE_STATE_STREAM, size, crc, crc_known)) return false;
    } else if (crc_known && meta_crc_blank(m)) {
        if (!set_stream_crc(crc)) return false;
    }
    m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    stream_expected_app_offset = journal_completed_bytes(m);
    stage_session_active = true;
    stage_session_total = size + F103_VESC_IMAGE_HEADER_SIZE;
    if (resume_stream_off) {
        *resume_stream_off = stream_expected_app_offset == 0u ? 0u :
                             stream_expected_app_offset + F103_VESC_IMAGE_HEADER_SIZE;
    }
    return true;
}

static bool stream_write(uint32_t staged_off, const uint8_t *data, uint32_t len) {
    if (!stage_session_active || !data || len == 0u || staged_off > stage_session_total ||
        len > stage_session_total - staged_off) return false;
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!meta_common_valid(m) || m->state != F103_UPDATE_STATE_STREAM) return false;

    uint32_t idx = 0u;
    uint32_t app_off;
    if (staged_off == 0u) {
        if (len < F103_VESC_IMAGE_HEADER_SIZE) return false;
        const uint32_t header_size = be32(data);
        const uint16_t header_crc = be16(data + 4u);
        if (header_size != m->size || !set_stream_crc(header_crc)) return false;
        idx = F103_VESC_IMAGE_HEADER_SIZE;
        app_off = 0u;
    } else {
        if (staged_off < F103_VESC_IMAGE_HEADER_SIZE) return false;
        app_off = staged_off - F103_VESC_IMAGE_HEADER_SIZE;
    }
    const uint32_t payload_len = len - idx;
    if (app_off > m->size || payload_len > m->size - app_off) return false;

    uint32_t pos = app_off;
    if (pos < stream_expected_app_offset) {
        const uint32_t dup = (stream_expected_app_offset - pos) < payload_len ?
                             (stream_expected_app_offset - pos) : payload_len;
        if (memcmp((const void *)(F103_APP_BASE_ADDR + pos), data + idx, dup) != 0) return false;
        pos += dup;
        idx += dup;
    }
    if (pos > stream_expected_app_offset) return false;

    while (idx < len) {
        const uint32_t page = pos / F103_FLASH_PAGE_SIZE;
        const uint32_t in_page = pos % F103_FLASH_PAGE_SIZE;
        uint32_t chunk = F103_FLASH_PAGE_SIZE - in_page;
        if (chunk > len - idx) chunk = len - idx;
        if (chunk > m->size - pos) chunk = m->size - pos;
        if (chunk == 0u) break;
        if (in_page == 0u && !journal_page_complete(page)) {
            if (!erase_one_page(F103_APP_BASE_ADDR + page * F103_FLASH_PAGE_SIZE)) return false;
        }
        if (!program_halfwords(F103_APP_BASE_ADDR + pos, data + idx, chunk)) return false;
        pos += chunk;
        idx += chunk;
        stream_expected_app_offset = pos;
        if ((pos % F103_FLASH_PAGE_SIZE) == 0u || pos == m->size) {
            if (!journal_mark_page(page)) return false;
        }
    }
    return true;
}

static bool stream_finalize(void) {
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (!meta_crc_valid(m) || m->state != F103_UPDATE_STATE_STREAM) return false;
    const uint32_t done = journal_completed_bytes(m);
    if (done != m->size) return false;
    boot_diag_copy_size = m->size;
    boot_diag_copy_crc_app = crc16((const uint8_t *)F103_APP_BASE_ADDR, m->size);
    if (boot_diag_copy_crc_app != m->crc16 || !app_vector_valid()) return false;
    return write_meta(F103_UPDATE_STATE_TEST, m->size, m->crc16, true);
}

__attribute__((naked, noreturn)) static void branch_to_app(uint32_t sp, uint32_t rv) {
    (void)sp; (void)rv;
    __asm volatile (
        "msr msp, r0\n"
        "bx r1\n"
    );
}

static void jump_app(void) {
    const uint32_t sp = *(const uint32_t *)F103_APP_BASE_ADDR;
    const uint32_t rv = *(const uint32_t *)(F103_APP_BASE_ADDR + 4u);
    (void)HAL_UART_DeInit(&huart3);
    *(volatile uint32_t *)F103_RESET_STAGE_ADDR = 0xB0070001u;
    HAL_SuspendTick();
    __disable_irq();
    SysTick->CTRL = 0u; SysTick->LOAD = 0u; SysTick->VAL = 0u;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    for (uint32_t i = 0u; i < 8u; ++i) { NVIC->ICER[i] = 0xFFFFFFFFu; NVIC->ICPR[i] = 0xFFFFFFFFu; }
    SCB->SHCSR &= ~(SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk);
    SCB->CFSR = 0xFFFFFFFFu; SCB->HFSR = 0xFFFFFFFFu; SCB->DFSR = 0xFFFFFFFFu;
    __set_BASEPRI(0u); __set_FAULTMASK(0u); __set_PSP(0u); __set_CONTROL(0u);
    SCB->VTOR = F103_APP_BASE_ADDR;
    __DSB();
    __ISB();
    *(volatile uint32_t *)F103_RESET_STAGE_ADDR = 0xB0070002u;
    /* Never execute C code after MSP changes. branch_to_app is naked and
     * transfers directly to the application's Reset_Handler. */
    branch_to_app(sp, rv);
}


static bool uart_recv_byte(uint8_t *out, uint32_t timeout_ms) {
    const uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < timeout_ms) {
        const uint32_t sr = USART3->SR;
        if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
            volatile uint32_t discard = USART3->DR;
            (void)discard;
            ++boot_diag_uart_errors;
            continue;
        }
        if ((sr & USART_SR_RXNE) != 0u) {
            *out = (uint8_t)USART3->DR;
            ++boot_diag_rx_bytes;
            return true;
        }
    }
    return false;
}

static bool uart_send_bytes(const uint8_t *data, uint16_t len, uint32_t timeout_ms) {
    if (!data || len == 0u) return false;
    for (uint16_t i = 0u; i < len; ++i) {
        const uint32_t start = HAL_GetTick();
        while ((USART3->SR & USART_SR_TXE) == 0u) {
            if ((HAL_GetTick() - start) >= timeout_ms) { ++boot_diag_uart_errors; return false; }
        }
        USART3->DR = data[i];
    }
    const uint32_t start = HAL_GetTick();
    while ((USART3->SR & USART_SR_TC) == 0u) {
        if ((HAL_GetTick() - start) >= timeout_ms) { ++boot_diag_uart_errors; return false; }
    }
    return true;
}

static void send_payload(const uint8_t *p, uint16_t len) {
    uint8_t tx[RX_MAX_PAYLOAD + 7u]; uint16_t i = 0u;
    if (!p || len == 0u || len > RX_MAX_PAYLOAD) return;
    if (len <= 255u) { tx[i++] = 2u; tx[i++] = (uint8_t)len; }
    else { tx[i++] = 3u; tx[i++] = (uint8_t)(len >> 8); tx[i++] = (uint8_t)len; }
    memcpy(&tx[i], p, len); i = (uint16_t)(i + len);
    uint16_t c = crc16(p, len); tx[i++] = (uint8_t)(c >> 8); tx[i++] = (uint8_t)c; tx[i++] = 3u;
    if (uart_send_bytes(tx, i, 1000u)) ++boot_diag_tx_replies;
}

static bool recv_payload(uint32_t timeout_ms, uint16_t *len_out) {
    const uint32_t start_ms = HAL_GetTick();
    uint8_t b = 0u;
    while ((HAL_GetTick() - start_ms) < timeout_ms) {
        if (!uart_recv_byte(&b, 10u)) continue;
        if (b != 2u && b != 3u) continue;
        ++boot_diag_start_frames;
        uint16_t len = 0u;
        if (b == 2u) {
            if (!uart_recv_byte(&b, 50u)) continue;
            len = b;
        } else {
            uint8_t l0 = 0u, l1 = 0u;
            if (!uart_recv_byte(&l0, 50u) || !uart_recv_byte(&l1, 50u)) continue;
            len = (uint16_t)(((uint16_t)l0 << 8) | l1);
        }
        if (len == 0u || len > RX_MAX_PAYLOAD) continue;
        bool ok = true;
        for (uint16_t i = 0u; i < len; ++i) {
            if (!uart_recv_byte(&rx_payload[i], 1000u)) { ok = false; break; }
        }
        if (!ok) continue;
        uint8_t c0 = 0u, c1 = 0u, tail = 0u;
        if (!uart_recv_byte(&c0, 100u) || !uart_recv_byte(&c1, 100u) || !uart_recv_byte(&tail, 100u)) continue;
        const uint16_t got = (uint16_t)(((uint16_t)c0 << 8) | c1);
        if (tail != 3u || got != crc16(rx_payload, len)) { ++boot_diag_crc_errors; continue; }
        ++boot_diag_packets_ok;
        if (len_out) *len_out = len;
        return true;
    }
    return false;
}

static void reply_fw_version(void) {
    uint8_t b[72]; uint16_t i = 0u;
    const char hw[] = "f103rc_bootloader"; const char fw[] = "f103rc_bootloader";
    b[i++] = COMM_FW_VERSION; b[i++] = 6u; b[i++] = 0u;
    memcpy(&b[i], hw, sizeof(hw)); i += sizeof(hw);
    memcpy(&b[i], (const void *)0x1FFFF7E8u, 12u); i += 12u;
    b[i++] = 1u; b[i++] = 0u; b[i++] = 0u; b[i++] = 0u;
    b[i++] = 0u; b[i++] = 0u; b[i++] = 0u; b[i++] = 0u;
    memcpy(&b[i], fw, sizeof(fw)); i += sizeof(fw);
    send_payload(b, i);
}

static void put_be32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v;
}

static void put_be16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v;
}

static void recovery_custom(const uint8_t *d, uint16_t n) {
    if (!d || n < 4u || d[0] != HB_MAGIC0 || d[1] != HB_MAGIC1 || d[2] != HB_VERSION) return;
    const uint8_t op = d[3];
    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    if (op == HB_BOOT_GET_INFO) {
        uint8_t r[28]; uint16_t i = 0u;
        r[i++] = COMM_CUSTOM_APP_DATA; r[i++] = HB_MAGIC0; r[i++] = HB_MAGIC1;
        r[i++] = HB_VERSION; r[i++] = op; r[i++] = 0u;
        put_be32(&r[i], F103_APP_REGION_SIZE); i += 4u;
        put_be32(&r[i], meta_common_valid(m) ? m->state : 0u); i += 4u;
        put_be32(&r[i], meta_common_valid(m) ? m->size : 0u); i += 4u;
        put_be16(&r[i], meta_crc_valid(m) ? m->crc16 : 0u); i += 2u;
        const uint32_t done = (meta_common_valid(m) && m->state == F103_UPDATE_STATE_STREAM) ?
                              journal_completed_bytes(m) : 0u;
        put_be32(&r[i], done == 0u ? 0u : done + F103_VESC_IMAGE_HEADER_SIZE); i += 4u;
        put_be16(&r[i], meta_common_valid(m) ? m->test_attempt : 0xFFFFu); i += 2u;
        r[i++] = app_vector_valid() ? 1u : 0u;
        send_payload(r, i);
        return;
    }
    if (op == HB_BOOT_READ_APP) {
        uint8_t r[256]; uint16_t i = 0u;
        r[i++] = COMM_CUSTOM_APP_DATA; r[i++] = HB_MAGIC0; r[i++] = HB_MAGIC1;
        r[i++] = HB_VERSION; r[i++] = op; r[i++] = 1u;
        if (n < 10u) { send_payload(r, i); return; }
        const uint32_t off = be32(d + 4u);
        uint16_t want = be16(d + 8u);
        if (want > 240u) want = 240u;
        if (off > F103_APP_REGION_SIZE || want > F103_APP_REGION_SIZE - off) {
            send_payload(r, i); return;
        }
        r[5] = 0u;
        put_be32(&r[i], off); i += 4u;
        put_be16(&r[i], want); i += 2u;
        memcpy(&r[i], (const void *)(F103_APP_BASE_ADDR + off), want); i = (uint16_t)(i + want);
        send_payload(r, i);
    }
}

static bool recovery_command(uint16_t len) {
    if (len == 0u) return true;
    const uint8_t id = rx_payload[0]; const uint8_t *d = rx_payload + 1u; uint16_t n = len - 1u;
    if (id == COMM_FW_VERSION) { reply_fw_version(); return true; }
    if (id == COMM_ALIVE) return true;
    if (id == COMM_CUSTOM_APP_DATA) { recovery_custom(d, n); return true; }
    if (id == COMM_ERASE_NEW_APP) {
        uint8_t r[6] = {COMM_ERASE_NEW_APP, 0u, 0u, 0u, 0u, 0u};
        if (n >= 4u) {
            const uint32_t size = be32(d);
            const bool crc_known = n >= 6u;
            const uint16_t crc = crc_known ? be16(d + 4u) : 0u;
            uint32_t resume = 0u;
            if (stream_begin(size, crc_known, crc, &resume)) {
                r[1] = 1u; put_be32(&r[2], resume);
            }
        }
        send_payload(r, sizeof(r)); return true;
    }
    if (id == COMM_WRITE_NEW_APP_DATA) {
        uint8_t r[6] = {COMM_WRITE_NEW_APP_DATA,0u,0u,0u,0u,0u};
        if (n >= 4u) {
            const uint32_t off = be32(d); const uint32_t dl = n - 4u;
            put_be32(&r[2], off);
            if (dl > 0u) r[1] = stream_write(off, d + 4u, dl) ? 1u : 0u;
        }
        send_payload(r, sizeof(r)); return true;
    }
    if (id == COMM_JUMP_TO_BOOTLOADER) {
        if (stream_finalize()) NVIC_SystemReset();
        return true;
    }
    if (id == COMM_REBOOT) {
        const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
        /* Leaving recovery is allowed only for a cryptographically-equivalent
         * condition to normal boot: CONFIRMED metadata, matching whole-image
         * CRC, and a sane vector table. STREAM/TEST/RECOVERY stay resident. */
        if (confirmed_app_valid(m)) NVIC_SystemReset();
        return true;
    }
    return true;
}

int main(void) {
    /* Earliest resident action: recover SW-DP and freeze IWDG while halted. */
    f103_debug_keepalive();
    /* Startup SystemInit() leaves the MCU on HSI=8 MHz but the CMSIS variable
     * defaults to 72 MHz. Fix the software model first, then raise the actual
     * clock to the same 64/32-MHz tree as the application before USART3 init. */
    SystemCoreClockUpdate();
    HAL_Init();
    /* Reassert after HAL_Init as well; no peripheral init may strand SWD. */
    f103_debug_keepalive();
    if (!boot_clock_init()) {
        for (;;) { f103_debug_keepalive(); __NOP(); }
    }
    safe_gpio_init();
    bool uart_ready = uart_init();

    volatile uint32_t *const boot_request = (volatile uint32_t *)F103_BOOT_REQUEST_ADDR;
    const bool force_recovery = boot_request[0] == F103_BOOT_REQUEST_MAGIC &&
                                boot_request[1] == F103_BOOT_REQUEST_MAGIC_INV;
    /* Consume immediately so any later reset boots normally unless another
     * explicit request or persistent PENDING/RECOVERY metadata exists. */
    boot_request[0] = 0u;
    boot_request[1] = 0u;
    __DSB();

    const f103_update_meta_t *m = (const f103_update_meta_t *)F103_META_BASE_ADDR;
    bool recovery = force_recovery;

    if (meta_common_valid(m)) {
        if (m->state == F103_UPDATE_STATE_STREAM || m->state == F103_UPDATE_STATE_RECOVERY) {
            recovery = true;
        } else if (m->state == F103_UPDATE_STATE_TEST) {
            if (force_recovery) {
                recovery = true;
            } else if (!meta_crc_valid(m) || !app_vector_valid() ||
                       crc16((const uint8_t *)F103_APP_BASE_ADDR, m->size) != m->crc16) {
                (void)write_meta(F103_UPDATE_STATE_RECOVERY, m->size, m->crc16, meta_crc_valid(m));
                recovery = true;
            } else if (!test_attempt_blank(m)) {
                /* TEST was already attempted and reset before CONFIRMED. */
                (void)write_meta(F103_UPDATE_STATE_RECOVERY, m->size, m->crc16, true);
                recovery = true;
            } else if (mark_test_attempt()) {
                jump_app();
            } else {
                recovery = true;
            }
        } else if (m->state == F103_UPDATE_STATE_CONFIRMED) {
            /* Normal boot is authorized only by persistent CONFIRMED metadata.
             * This prevents stale bytes at APP_BASE from being mistaken for an
             * application after interrupted programming or metadata corruption. */
            if (!force_recovery && confirmed_app_valid(m)) jump_app();
            recovery = true;
        }
    }

    /* Invalid/blank metadata is intentionally recovery-only. */
    if (!recovery) recovery = true;

    uint32_t blink = HAL_GetTick();
    uint32_t debug_keepalive = blink;
    uint32_t uart_retry = blink;
    while (recovery) {
        const uint32_t now=HAL_GetTick();
        if (!uart_ready) {
            if ((uint32_t)(now-uart_retry) >= 100u) {
                uart_retry=now; uart_ready=uart_init();
            }
        } else {
            uint16_t len = 0u;
            if (recv_payload(50u, &len)) recovery_command(len);
        }
        if ((uint32_t)(now-debug_keepalive) >= 25u) {
            debug_keepalive=now; f103_debug_keepalive();
        }
        if ((uint32_t)(now-blink) >= RECOVERY_IDLE_BLINK_MS) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); blink = now;
        }
    }
    for (;;) { f103_debug_keepalive(); __NOP(); }
}
