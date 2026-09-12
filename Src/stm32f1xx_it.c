/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

#include "defines.h"
#include "config.h"
#include "util.h"
#include "vesc/f103_boot_layout.h"

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN 0 */
extern UART_HandleTypeDef huart3;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */
/******************************************************************************/

static inline void f103_emergency_pwm_off(void);

/**
* @brief This function handles Non maskable interrupt.
*/
void f103_NMI_Handler_impl(void) {
  f103_emergency_pwm_off();
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}


static inline void f103_emergency_pwm_off(void)
{
  /* Exception path: keep this register-only and bounded. */
  TIM1->BDTR &= ~TIM_BDTR_MOE;
  TIM8->BDTR &= ~TIM_BDTR_MOE;
  TIM1->CCR1 = 0; TIM1->CCR2 = 0; TIM1->CCR3 = 0;
  TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0;
}

static __attribute__((noreturn)) void f103_fault_to_recovery(uint32_t reason)
{
  /* Keep the power stage inert, preserve SWD, and make a synchronous reset
   * into the resident bootloader. This prevents a persistent application fault
   * from becoming fault -> IWDG -> reboot -> fault forever. */
  f103_emergency_pwm_off();
  DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;
  volatile uint32_t *const boot_request = (volatile uint32_t *)F103_BOOT_REQUEST_ADDR;
  boot_request[0] = F103_BOOT_REQUEST_MAGIC;
  boot_request[1] = F103_BOOT_REQUEST_MAGIC_INV;
  *(volatile uint32_t *)F103_RESET_REASON_ADDR = reason;
  __DSB();
  __ISB();
  NVIC_SystemReset();
  for (;;) { __NOP(); }
}

/**
* @brief This function handles Hard fault interrupt.
*/
void f103_HardFault_Handler_impl(void) {
  f103_fault_to_recovery(0x48415244u); /* HARD */
}

/**
* @brief This function handles Memory management fault.
*/
void f103_MemManage_Handler_impl(void) {
  f103_fault_to_recovery(0x4D454D46u); /* MEMF */
}

/**
* @brief This function handles Prefetch fault, memory access fault.
*/
void f103_BusFault_Handler_impl(void) {
  f103_fault_to_recovery(0x42555346u); /* BUSF */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void f103_UsageFault_Handler_impl(void) {
  f103_fault_to_recovery(0x55534746u); /* USGF */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void f103_SVC_Handler_impl(void) {
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void f103_DebugMon_Handler_impl(void) {
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void f103_PendSV_Handler_impl(void) {
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/


void f103_SysTick_Handler_impl(void) {
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}





/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void f103_DMA1_Channel2_IRQHandler_impl(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void f103_DMA1_Channel3_IRQHandler_impl(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}


/**
  * @brief This function handles USART3 global interrupt.
  */
void f103_USART3_IRQHandler_impl(void)
{
  if ((__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET) &&
      (__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET)) {
    /* RX DMA is drained only from the main loop. Calling usart3_rx_check()
     * here races its static oldPos against the main-loop caller and can feed
     * the same DMA bytes twice into the VESC parser, producing false CRC
     * errors on otherwise valid frames. IDLE only wakes/acknowledges UART. */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
  }
  HAL_UART_IRQHandler(&huart3);
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
