#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H

/* Native-host regression shim only.
 * Target firmware uses the real STM32Cube HAL supplied by PlatformIO.
 */
#include <stdint.h>

typedef struct {
  volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GPIO_TypeDef;

typedef struct {
  volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR;
  volatile uint32_t CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
  volatile uint32_t CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR;
} TIM_TypeDef;

typedef struct {
  volatile uint32_t ISR, IFCR;
} DMA_TypeDef;

typedef struct {
  volatile uint32_t CCR, CNDTR, CPAR, CMAR;
} DMA_Channel_TypeDef;

typedef struct {
  volatile uint32_t CTRL, CYCCNT;
} DWT_Type;

typedef struct {
  volatile uint32_t DHCSR, DCRSR, DCRDR, DEMCR;
} CoreDebug_Type;

typedef struct {
  uint32_t Pin;
  uint32_t Mode;
  uint32_t Pull;
  uint32_t Speed;
} GPIO_InitTypeDef;

typedef struct {
  void *Instance;
} UART_HandleTypeDef;

extern GPIO_TypeDef _GPIOA;
extern GPIO_TypeDef _GPIOB;
extern GPIO_TypeDef _GPIOC;
extern TIM_TypeDef _TIM1;
extern TIM_TypeDef _TIM8;
extern DMA_TypeDef _DMA1;
extern DWT_Type _DWT;
extern CoreDebug_Type _CoreDebug;

#define GPIOA (&_GPIOA)
#define GPIOB (&_GPIOB)
#define GPIOC (&_GPIOC)
#define TIM1 (&_TIM1)
#define TIM8 (&_TIM8)
#define DMA1 (&_DMA1)
#define DWT (&_DWT)
#define CoreDebug (&_CoreDebug)

#define GPIO_PIN_0  (1u << 0)
#define GPIO_PIN_1  (1u << 1)
#define GPIO_PIN_2  (1u << 2)
#define GPIO_PIN_3  (1u << 3)
#define GPIO_PIN_4  (1u << 4)
#define GPIO_PIN_5  (1u << 5)
#define GPIO_PIN_6  (1u << 6)
#define GPIO_PIN_7  (1u << 7)
#define GPIO_PIN_8  (1u << 8)
#define GPIO_PIN_9  (1u << 9)
#define GPIO_PIN_10 (1u << 10)
#define GPIO_PIN_11 (1u << 11)
#define GPIO_PIN_12 (1u << 12)
#define GPIO_PIN_13 (1u << 13)
#define GPIO_PIN_14 (1u << 14)
#define GPIO_PIN_15 (1u << 15)

#define GPIO_MODE_INPUT       0u
#define GPIO_NOPULL           0u
#define GPIO_PULLUP           1u
#define GPIO_SPEED_FREQ_LOW   0u

#define TIM_BDTR_MOE          (1u << 15)
#define TIM_CCMR1_CC1S_0      (1u << 0)
#define TIM_CCMR1_CC2S_0      (1u << 8)
#define TIM_CCMR1_IC1F_Pos    4u
#define TIM_CCMR1_IC2F_Pos    12u
#define TIM_SMCR_SMS_0        (1u << 0)
#define TIM_SMCR_SMS_1        (1u << 1)
#define TIM_CCER_CC1P         (1u << 1)
#define TIM_CCER_CC2P         (1u << 5)
#define TIM_EGR_UG            (1u << 0)
#define TIM_CR1_CEN           (1u << 0)

#define DMA_ISR_TCIF1         (1u << 1)
#define DMA_IFCR_CTCIF1       (1u << 1)

#define DWT_CTRL_CYCCNTENA_Msk (1u << 0)
#define UART_WORDLENGTH_8B      0u
#define HAL_OK                  0u
#define FLASH_PAGE_SIZE         0x800u

#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE()  ((void)0)
#define __disable_irq()              ((void)0)
#define __enable_irq()               ((void)0)

static inline void HAL_GPIO_Init(GPIO_TypeDef *port, const GPIO_InitTypeDef *init) {
  (void)port;
  (void)init;
}

static inline uint32_t *host_hal_tick_storage(void) {
  static uint32_t tick = 0u;
  return &tick;
}

static inline uint32_t HAL_GetTick(void) {
  return *host_hal_tick_storage();
}

static inline void HAL_Delay(uint32_t ms) {
  *host_hal_tick_storage() += ms;
}

static inline uint32_t HAL_FLASH_Unlock(void) {
  return HAL_OK;
}

static inline uint32_t HAL_FLASH_Lock(void) {
  return HAL_OK;
}

#endif
