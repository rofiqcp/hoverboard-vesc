#include <string.h>
#include "encoder/enc_abi.h"

static uint32_t abi_counts_sane(uint32_t counts) {
    if (counts < 4u) counts = 4u;
    if (counts > 65536u) counts = 65536u;
    return counts;
}

bool enc_abi_init(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer || !cfg->A_gpio || !cfg->B_gpio) return false;
    cfg->counts = abi_counts_sane(cfg->counts);
    memset(&cfg->state, 0, sizeof(cfg->state));

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};
    /* Keep the ABI inputs biased high even if one external pull-up or connector
     * contact is marginal. The STM32 weak pull-up is harmless in parallel with
     * the board 2.2 kOhm pull-ups and prevents a floating channel from looking
     * permanently low during startup alignment. */
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin = cfg->A_pin;
    HAL_GPIO_Init(cfg->A_gpio, &io);
    io.Pin = cfg->B_pin;
    HAL_GPIO_Init(cfg->B_gpio, &io);

    cfg->timer->CR1 = 0u;
    cfg->timer->CR2 = 0u;
    cfg->timer->SMCR = 0u;
    cfg->timer->DIER = 0u;
    cfg->timer->CCER = 0u;
    cfg->timer->CCMR1 = 0u;
    cfg->timer->CCMR2 = 0u;
    cfg->timer->PSC = 0u;
    /* Legacy working firmware used the full 16-bit TIM4 counter. Keep that
     * hardware behaviour and normalize to configured CPR in the read path. */
    cfg->timer->ARR = 0xffffu;
    cfg->timer->CNT = 0u;

    /* Match the proven pre-VESC TIM4 setup: TI12, rising/rising, filter=3.
     * VESC semantics are preserved above this hardware capture layer. */
    cfg->timer->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 |
                        (3u << TIM_CCMR1_IC1F_Pos) |
                        (3u << TIM_CCMR1_IC2F_Pos);
    cfg->timer->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
    cfg->timer->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    cfg->timer->EGR = TIM_EGR_UG;
    cfg->timer->CR1 |= TIM_CR1_CEN;

    /* Sama seperti VESC: state ABI mulai belum tersinkron. Karena hardware ini
     * hanya A/B tanpa index I, encoder_set_deg() setelah electrical alignment
     * yang mengubah index_found menjadi true. */
    return true;
}

void enc_abi_deinit(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer) return;
    cfg->timer->CR1 &= ~TIM_CR1_CEN;
    GPIO_InitTypeDef io = {0};
    /* Restore the same floating-input electrical state used by the known-good
     * steering firmware. LEFT ABI owns PB6/PB7 in this project; Hall mode is
     * not allowed to share those two lines while ABI is selected. */
    io.Mode = GPIO_MODE_INPUT;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin = cfg->A_pin;
    HAL_GPIO_Init(cfg->A_gpio, &io);
    io.Pin = cfg->B_pin;
    HAL_GPIO_Init(cfg->B_gpio, &io);
}

uint32_t enc_abi_read_cnt(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer || cfg->counts < 1u) return 0u;
    return cfg->timer->CNT % cfg->counts;
}

float enc_abi_read_deg(ABI_config_t *cfg) {
    if (!cfg || !cfg->timer || cfg->counts < 1u) return 0.0f;
    return ((float)enc_abi_read_cnt(cfg) * 360.0f) / (float)cfg->counts;
}

void enc_abi_set_deg(ABI_config_t *cfg, float deg) {
    if (!cfg || !cfg->timer || cfg->counts < 1u) return;
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f) deg += 360.0f;
    /* VESC encoder_set_deg() memakai truncation, bukan rounding. */
    uint32_t cnt = (uint32_t)(deg * (float)cfg->counts / 360.0f);
    if (cnt >= cfg->counts) cnt = 0u;
    cfg->timer->CNT = cnt;
    cfg->state.index_found = true;
}
