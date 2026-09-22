#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef struct { volatile uint32_t IDR, ODR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CR1,CR2,SMCR,DIER,SR,EGR,CCMR1,CCMR2,CCER,CNT,PSC,ARR,RCR,CCR1,CCR2,CCR3,CCR4,BDTR,DCR,DMAR; } TIM_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CTRL,CYCCNT,CPICNT,EXCCNT,SLEEPCNT,LSUCNT,FOLDCNT,PCSR; } DWT_Type;
typedef struct { volatile uint32_t CR1,CR2,CR3,BRR,DR,SR; } USART_TypeDef;
typedef struct { volatile uint32_t CCR,CNDTR; } DMA_Channel_TypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { DMA_HandleTypeDef *hdmatx; DMA_HandleTypeDef *hdmarx; USART_TypeDef *Instance; uint32_t gState,ErrorCode,RxState; } UART_HandleTypeDef;
typedef struct { int dummy; } ADC_HandleTypeDef;
typedef int GPIO_PinState;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
extern GPIO_TypeDef _GPIOA,_GPIOB,_GPIOC; extern TIM_TypeDef _TIM1,_TIM8; extern DMA_TypeDef _DMA1; extern DWT_Type _DWT; extern USART_TypeDef _USART3;
#define GPIOA (&_GPIOA)
#define GPIOB (&_GPIOB)
#define GPIOC (&_GPIOC)
#define TIM1 (&_TIM1)
#define TIM8 (&_TIM8)
#define DMA1 (&_DMA1)
#define DWT (&_DWT)
#define USART3 (&_USART3)
#define DMA_IFCR_CTCIF1 (1u<<1)
#define TIM_BDTR_MOE (1u<<15)
#define GPIO_PIN_0 (1u<<0)
#define GPIO_PIN_1 (1u<<1)
#define GPIO_PIN_2 (1u<<2)
#define GPIO_PIN_3 (1u<<3)
#define GPIO_PIN_4 (1u<<4)
#define GPIO_PIN_5 (1u<<5)
#define GPIO_PIN_6 (1u<<6)
#define GPIO_PIN_7 (1u<<7)
#define GPIO_PIN_8 (1u<<8)
#define GPIO_PIN_9 (1u<<9)
#define GPIO_PIN_10 (1u<<10)
#define GPIO_PIN_11 (1u<<11)
#define GPIO_PIN_12 (1u<<12)
#define GPIO_PIN_13 (1u<<13)
#define GPIO_PIN_14 (1u<<14)
#define GPIO_PIN_15 (1u<<15)
#define UART_WORDLENGTH_8B 0
#define HAL_OK 0
#define HAL_UART_STATE_READY 0u
#define HAL_UART_ERROR_NONE 0u
#define UART_IT_IDLE 0u
#define USART_CR1_PEIE (1u<<8)
#define USART_CR3_EIE (1u<<0)
#define USART_CR3_DMAR (1u<<6)
#define DMA_CCR_EN (1u<<0)
#define CLEAR_BIT(REG,BIT) ((REG)&=~(BIT))
#define SET_BIT(REG,BIT) ((REG)|=(BIT))
#define __HAL_UART_ENABLE_IT(H,I) ((void)(H),(void)(I))
#define __HAL_DMA_GET_COUNTER(H) ((H) && (H)->Instance ? (H)->Instance->CNDTR : 0u)
#define FLASH_TYPEPROGRAM_HALFWORD 0u
#define FLASH_PAGE_SIZE 2048u
#define HAL_MAX_DELAY 0xffffffffu

typedef struct { uint32_t PLLState, PLLSource, PLLMUL; } RCC_PLLInitTypeDef;
typedef struct { uint32_t OscillatorType, HSIState, HSICalibrationValue; RCC_PLLInitTypeDef PLL; } RCC_OscInitTypeDef;
typedef struct { uint32_t ClockType, SYSCLKSource, AHBCLKDivider, APB1CLKDivider, APB2CLKDivider; } RCC_ClkInitTypeDef;
typedef struct { uint32_t PeriphClockSelection, AdcClockSelection; } RCC_PeriphCLKInitTypeDef;
typedef struct { volatile uint32_t DHCSR,DCRSR,DCRDR,DEMCR; } CoreDebug_Type;
extern CoreDebug_Type _CoreDebug;
#define CoreDebug (&_CoreDebug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u<<24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define NVIC_PRIORITYGROUP_4 0u
#define MemoryManagement_IRQn 0
#define BusFault_IRQn 1
#define UsageFault_IRQn 2
#define SVCall_IRQn 3
#define DebugMonitor_IRQn 4
#define PendSV_IRQn 5
#define SysTick_IRQn 6
#define RCC_OSCILLATORTYPE_HSI 1u
#define RCC_HSI_ON 1u
#define RCC_PLL_ON 1u
#define RCC_PLLSOURCE_HSI_DIV2 1u
#define RCC_PLL_MUL16 16u
#define RCC_CLOCKTYPE_HCLK 1u
#define RCC_CLOCKTYPE_SYSCLK 2u
#define RCC_CLOCKTYPE_PCLK1 4u
#define RCC_CLOCKTYPE_PCLK2 8u
#define RCC_SYSCLKSOURCE_PLLCLK 1u
#define RCC_SYSCLK_DIV1 1u
#define RCC_HCLK_DIV1 1u
#define RCC_HCLK_DIV2 2u
#define RCC_PERIPHCLK_ADC 1u
#define RCC_ADCPCLK2_DIV4 4u
#define RCC_ADCPCLK2_DIV6 6u
#define RCC_ADCPCLK2_DIV8 8u
#define FLASH_LATENCY_2 2u
#define SYSTICK_CLKSOURCE_HCLK 1u
#define __HAL_RCC_AFIO_CLK_ENABLE() ((void)0)
#define __HAL_RCC_DMA1_CLK_DISABLE() ((void)0)
static inline void HAL_Init(void){}
static inline void HAL_NVIC_SetPriorityGrouping(uint32_t g){(void)g;}
static inline void HAL_NVIC_SetPriority(int i,uint32_t p,uint32_t s){(void)i;(void)p;(void)s;}
static inline int HAL_RCC_OscConfig(RCC_OscInitTypeDef *x){(void)x;return HAL_OK;}
static inline int HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *x,uint32_t f){(void)x;(void)f;return HAL_OK;}
static inline int HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *x){(void)x;return HAL_OK;}
static inline uint32_t HAL_RCC_GetHCLKFreq(void){return 64000000u;}
static inline int HAL_SYSTICK_Config(uint32_t x){(void)x;return HAL_OK;}
static inline void HAL_SYSTICK_CLKSourceConfig(uint32_t x){(void)x;}
static inline int HAL_ADC_Start(ADC_HandleTypeDef *h){(void)h;return HAL_OK;}

static inline uint32_t HAL_GetTick(void){return 0u;}
static inline void __disable_irq(void){}
static inline void __enable_irq(void){}
static inline void HAL_GPIO_TogglePin(GPIO_TypeDef *p, uint16_t pin){(void)p;(void)pin;}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s){(void)p;(void)pin;(void)s;}
static inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin){(void)p;(void)pin;return GPIO_PIN_RESET;}
static inline int HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t*d,uint16_t n,uint32_t t){(void)h;(void)d;(void)n;(void)t;return HAL_OK;}
static inline int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t*d,uint16_t n){(void)h;(void)d;(void)n;return HAL_OK;}
static inline int HAL_UART_Receive_DMA(UART_HandleTypeDef *h,uint8_t*d,uint16_t n){(void)h;(void)d;(void)n;return HAL_OK;}
static inline int HAL_UART_DMAStop(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static inline int HAL_UART_DeInit(UART_HandleTypeDef *h){(void)h;return HAL_OK;}
static inline void NVIC_SystemReset(void){}
static inline int HAL_FLASH_Unlock(void){return HAL_OK;}
static inline int HAL_FLASH_Lock(void){return HAL_OK;}
static inline int HAL_FLASH_Program(uint32_t type,uint32_t addr,uint64_t data){(void)type;(void)addr;(void)data;return HAL_OK;}
void HAL_Delay(uint32_t ms);

/* Host regression extensions used by encoder ABI tests. */
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
#define GPIO_MODE_INPUT 0u
#define GPIO_NOPULL 0u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_LOW 0u
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *cfg){(void)p;(void)cfg;}
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM4_CLK_ENABLE() ((void)0)
#define TIM_CCMR1_CC1S_0 (1u<<0)
#define TIM_CCMR1_CC2S_0 (1u<<8)
#define TIM_CCMR1_IC1F_Pos 4u
#define TIM_CCMR1_IC2F_Pos 12u
#define TIM_SMCR_SMS_0 (1u<<0)
#define TIM_SMCR_SMS_1 (1u<<1)
#define TIM_CCER_CC1P (1u<<1)
#define TIM_CCER_CC2P (1u<<5)
#define TIM_EGR_UG (1u<<0)
#define TIM_CR1_CEN (1u<<0)
static TIM_TypeDef _host_TIM4 __attribute__((unused))={0};
#define TIM4 (&_host_TIM4)
