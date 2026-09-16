#ifndef CONTROL_UART_H_
#define CONTROL_UART_H_

#include "stm32f1xx_hal.h"
#include "vesc/f103_boot_layout.h"

#if defined(F103_CONTROL_USART2) && defined(F103_CONTROL_USART3)
#error "Select exactly one control UART: F103_CONTROL_USART2 or F103_CONTROL_USART3"
#elif !defined(F103_CONTROL_USART2) && !defined(F103_CONTROL_USART3)
#if defined(USE_HAL_DRIVER)
#error "No control UART selected. Use a USART2 or USART3 PlatformIO environment."
#else
/* Host-only regression builds do not carry PlatformIO flags; retain their
 * historical USART3 model without weakening the target-build fail-closed gate. */
#define F103_CONTROL_USART3 1
#define F103_CONTROL_UART_HOST_FALLBACK 1
#endif
#endif

#define CONTROL_UART_BAUD             F103_VESC_UART_BAUD
#define CONTROL_UART_BOOT_BAUD        F103_BOOT_UART_BAUD
#define CONTROL_UART_WORDLENGTH       UART_WORDLENGTH_8B

#if defined(F103_CONTROL_USART2)
#define CONTROL_UART_INSTANCE          USART2
#define CONTROL_UART_IRQn              USART2_IRQn
#define CONTROL_UART_RX_DMA_CHANNEL    DMA1_Channel6
#define CONTROL_UART_TX_DMA_CHANNEL    DMA1_Channel7
#define CONTROL_UART_RX_DMA_IRQn       DMA1_Channel6_IRQn
#define CONTROL_UART_TX_DMA_IRQn       DMA1_Channel7_IRQn
#define CONTROL_UART_APP_ADC_AVAILABLE 0u
#define CONTROL_UART_PA2_PA3_ARE_UART  1u
#define CONTROL_UART_IS_USART2         1u
#define CONTROL_UART_IS_USART3         0u
#elif defined(F103_CONTROL_USART3)
#define CONTROL_UART_INSTANCE          USART3
#define CONTROL_UART_IRQn              USART3_IRQn
#define CONTROL_UART_RX_DMA_CHANNEL    DMA1_Channel3
#define CONTROL_UART_TX_DMA_CHANNEL    DMA1_Channel2
#define CONTROL_UART_RX_DMA_IRQn       DMA1_Channel3_IRQn
#define CONTROL_UART_TX_DMA_IRQn       DMA1_Channel2_IRQn
#define CONTROL_UART_APP_ADC_AVAILABLE 1u
#define CONTROL_UART_PA2_PA3_ARE_UART  0u
#define CONTROL_UART_IS_USART2         0u
#define CONTROL_UART_IS_USART3         1u
#else
#error "Internal error: control UART selection escaped the fail-closed gate"
#endif

/* PA2/PA3 are physically multiplexed between USART2 and the VESC App-ADC.
 * Keep this relationship compile-time and fail closed if a future edit makes
 * the two feature definitions inconsistent. */
#if defined(F103_CONTROL_USART2) && CONTROL_UART_APP_ADC_AVAILABLE
#error "USART2 owns PA2/PA3: App-ADC must be disabled"
#endif
#if defined(F103_CONTROL_USART2) && !CONTROL_UART_PA2_PA3_ARE_UART
#error "USART2 build must reserve PA2/PA3 for UART"
#endif
#if defined(F103_CONTROL_USART3) && !CONTROL_UART_APP_ADC_AVAILABLE
#error "USART3 build must retain PA2/PA3 App-ADC support"
#endif
#if defined(F103_CONTROL_USART3) && CONTROL_UART_PA2_PA3_ARE_UART
#error "USART3 build must not reserve PA2/PA3 for UART"
#endif

#endif /* CONTROL_UART_H_ */
