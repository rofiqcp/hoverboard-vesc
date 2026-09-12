#include <stdio.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "motor/mcpwm_foc.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"
#include "vesc/vesc_protocol.h"
#include "vesc/f103_boot_layout.h"
#include "vesc/flash_update_f103.h"
#include "vesc/app_vesc.h"
#include "comms.h"
#include "platform_watchdog.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern InputStruct input1[];
extern InputStruct input2[];
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern uint8_t timeoutFlgSerial;
extern uint8_t ctrlModReq;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int16_t batVoltage;
extern volatile uint32_t buzzerTimer;
extern volatile uint32_t foc_isr_cycles;
extern volatile int16_t foc_iqL_q4;
extern volatile int16_t foc_iqR_q4;
extern volatile int16_t foc_idL_q4;
extern volatile int16_t foc_idR_q4;

volatile uint32_t main_loop_counter = 0;
volatile uint32_t boot_reset_csr = 0u;
volatile uint32_t boot_reset_reason = 0u;
volatile uint32_t boot_reset_stage = 0u;
volatile uint32_t main_prof_vesc_max_cycles = 0u;
volatile uint32_t main_prof_house_max_cycles = 0u;
volatile uint32_t main_prof_tail_max_cycles = 0u;
int16_t batVoltageCalib = 0;
int16_t board_temp_deg_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;
int16_t cmdL = 0;
int16_t cmdR = 0;

static int16_t cmdLRateFixdt = 0;
static int16_t cmdRRateFixdt = 0;
static int32_t cmdLFixdt = 0;
static int32_t cmdRFixdt = 0;
static uint32_t buzzerTimerPrev = 0;

static void cycleCounterInit(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t controllerFaultActive(void) {
  return (m_motor_1.m_fault != FAULT_CODE_NONE) || (m_motor_2.m_fault != FAULT_CODE_NONE);
}

static inline void f103_debug_keepalive(void) {
#if defined(__arm__) || defined(__thumb__)
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_ENABLE();
  __HAL_DBGMCU_FREEZE_IWDG();
  __DSB();
  __ISB();
#endif
}

int main(void) {
  /* Capture the reset source before HAL/application code can obscure it, then
   * clear sticky RCC reset flags so the next reboot has an unambiguous cause. */
#ifdef STM32F103xE
  boot_reset_csr = RCC->CSR;
  boot_reset_reason = *(volatile uint32_t *)F103_RESET_REASON_ADDR;
  boot_reset_stage = *(volatile uint32_t *)F103_RESET_STAGE_ADDR;
  *(volatile uint32_t *)F103_RESET_REASON_ADDR = 0u;
  RCC->CSR |= RCC_CSR_RMVF;
#endif
  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  /* Keep the F103 debug port recoverable in every runtime build. */
  f103_debug_keepalive();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);

  SystemClock_Config();
  cycleCounterInit();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();

  /* Defensive post-init restore. Any future AFIO remap added by peripheral
   * setup must not be allowed to strand the target from SWD. */
  f103_debug_keepalive();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
  Input_Lim_Init();
  Input_Init();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  /* Bootloader hands off with PRIMASK set so no peripheral IRQ can run before
   * C runtime and all HAL handles are initialized. Enable globally only here,
   * after GPIO/TIM/ADC/UART/DMA are configured and their state is valid. */
  __enable_irq();

  /* Incremental ABI has no absolute index. If a steering hard-stop span has
   * already been calibrated, normal power-up assumes the wheel was placed at
   * center by the operator. Wait for the ADC zero calibration, perform only the
   * adaptive Id electrical-phase/ABI synchronization, then return to the exact
   * boot point and define it as logical VESC position 180 degrees. No hard-stop
   * sweep is performed on an ordinary power cycle. */
  mcpwm_foc_release_motor(false);
  if(mc_interface_steering_calibration_valid()){
    for(uint32_t t=0u;t<2500u && !mcpwm_foc_dc_cal_done();++t)HAL_Delay(1u);
    if(mcpwm_foc_dc_cal_done())(void)mc_interface_steering_boot_home();
  }

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int32_t boardTempAdcFixdt = (int32_t)adc_buffer.temp * 65536;
  int16_t boardTempAdcFilt = adc_buffer.temp;
#if !POWER_BUTTON_BYPASS
  while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) HAL_Delay(10);
#endif

  /* Start IWDG only after potentially long boot/home/button waits. From here on
   * every intentional blocking commissioning helper services the same health gate. */
  platform_watchdog_init();

  /* A freshly streamed image gets one guarded test boot. Both bridges remain
   * torque-off while real ADC/FOC liveness services the hardware watchdog.
   * If this image resets before confirmation, TEST metadata persists and the
   * resident bootloader stays in recovery for host-LKG restore. */
  if (f103_fw_test_pending()) {
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    const uint32_t fw_test_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - fw_test_start) < 3000u) {
      platform_watchdog_service();
      HAL_Delay(1u);
    }
    if (!f103_fw_confirm_running_image()) {
      *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_FW_UPDATE;
      NVIC_SystemReset();
    }
  }

  while (1) {
    uint32_t prof0=DWT->CYCCNT;
    /* Deadline control didahulukan dari parser komunikasi. Upstream VESC
     * menjalankan speed/position pada PID thread terpisah; pada bare-metal F103
     * padanan paling deterministik adalah mengeksekusi scheduler 1 kHz sebelum
     * packet/config work. Paket yang baru tiba menjadi setpoint tick berikutnya
     * (latensi <=1 ms), tetapi burst VESC tidak boleh menambah jitter kontrol. */
    uint32_t vesc_now_ms = HAL_GetTick();
    mcpwm_foc_outer_control_non_isr(vesc_now_ms);
    platform_watchdog_service();

    /* Drain USART3 circular DMA every main-loop pass as a deterministic
     * fallback to the IDLE-line IRQ, then process protocol in the remaining
     * main-context budget. FOC current regulation remains interrupt-driven. */
    usart3_rx_check();
    vesc_protocol_process_pending();
    vesc_now_ms = HAL_GetTick();
    vesc_protocol_periodic(vesc_now_ms);
    usart3_recovery_tick(vesc_now_ms);
    uint32_t profd=DWT->CYCCNT-prof0;
    // cppcheck-suppress unsignedLessThanZero -- CYCCNT dan maksimum profiler sama-sama uint32_t.
    if(profd>main_prof_vesc_max_cycles)main_prof_vesc_max_cycles=profd;

    if ((buzzerTimer - buzzerTimerPrev) <= (16u * DELAY_IN_MAIN_LOOP)) continue;

    prof0=DWT->CYCCNT;
    readCommand();
    const bool vescLinkActive = vesc_protocol_link_active();
    calcAvgSpeed();
    app_vesc_process(HAL_GetTick());
    mcpwm_foc_energy_update(HAL_GetTick());
    mcpwm_foc_housekeeping_non_isr(HAL_GetTick());

    /* Legacy serial has its own enable/beep handshake. A live VESC binary link
     * is armed by valid VESC traffic and must never enter this blocking ~300-ms
     * legacy sequence; otherwise VESC Tool/Python telemetry can time out exactly
     * when a motor command is first issued. */
    if (!vescLinkActive && !timeoutFlgSerial && enable == 0 && !controllerFaultActive() &&
        input1[0].cmd > -50 && input1[0].cmd < 50 && input2[0].cmd > -50 && input2[0].cmd < 50) {
      beepShort(6);
      beepShort(4);
      HAL_Delay(100);
      cmdLFixdt = 0;
      cmdRFixdt = 0;
      enable = 1;
      printf("-- Motors enabled --\r\n");
    }

    profd=DWT->CYCCNT-prof0;
    if(profd>main_prof_house_max_cycles)main_prof_house_max_cycles=profd;
    prof0=DWT->CYCCNT;

    /* Left and right motor commands are independent. STOP also follows the
     * same rate limiter so the motors decelerate instead of dropping torque
     * abruptly. Snap only the final <=1 command-count residual to exact zero
     * after the rate-limiter target has already reached zero. */
    rateLimiter16(input1[0].cmd, RATE, &cmdLRateFixdt);
    rateLimiter16(input2[0].cmd, RATE, &cmdRRateFixdt);
    filtLowPass32(cmdLRateFixdt >> 4, FILTER, &cmdLFixdt);
    filtLowPass32(cmdRRateFixdt >> 4, FILTER, &cmdRFixdt);
    cmdL = (int16_t)(cmdLFixdt >> 16);
    cmdR = (int16_t)(cmdRFixdt >> 16);

    if (input1[0].cmd == 0 && cmdLRateFixdt == 0 && abs(cmdL) <= 1) {
      cmdLFixdt = 0;
      cmdL = 0;
    }
    if (input2[0].cmd == 0 && cmdRRateFixdt == 0 && abs(cmdR) <= 1) {
      cmdRFixdt = 0;
      cmdR = 0;
    }
    pwml = cmdL;
    pwmr = -cmdR;  // positive cmdR means forward wheel direction, matching positive cmdL


    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &boardTempAdcFixdt);
    boardTempAdcFilt = (int16_t)(boardTempAdcFixdt >> 16);
    board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                       (boardTempAdcFilt - TEMP_CAL_LOW_ADC) /
                       (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
    mcpwm_foc_set_board_temperature_x10(board_temp_deg_c);
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
    left_dc_curr = -(m_motor_1.m_current_in_counts * 100) / A2BIT_CONV;
    right_dc_curr = -(m_motor_2.m_current_in_counts * 100) / A2BIT_CONV;
    dc_curr = left_dc_curr + right_dc_curr;

    /* USART3 is VESC-exclusive: no raw debug output on the motor link. */

    /* USART3 hanya membawa protokol VESC. Telemetri legacy 72-byte telah
     * dihapus agar tidak ada jalur mati/duplikat yang dapat mencemari link motor. */

    poweroffPressCheck();

    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {
      poweroff();
    } else if (controllerFaultActive()) {
      enable = 0;
      beepCount(1, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {
      beepCount(0, 10, 30);
    } else {
      beepCount(0, 0, 0);
    }

    // if (abs(cmdL) > 50 || abs(cmdR) > 50) inactivityTimeoutCounter = 0;
    // else ++inactivityTimeoutCounter;
    // if (inactivityTimeoutCounter > (INACTIVITY_TIMEOUT * 60u * 1000u) / (DELAY_IN_MAIN_LOOP + 1u)) poweroff();

    profd=DWT->CYCCNT-prof0;
    if(profd>main_prof_tail_max_cycles)main_prof_tail_max_cycles=profd;
    buzzerTimerPrev = buzzerTimer;
    ++main_loop_counter;
  }
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  /* PCLK2 runtime adalah 64 MHz. STM32F103xC/D/E membatasi ADCCLK sampai
   * 14 MHz, jadi DIV6 memberi 10,667 MHz dan tetap menyelesaikan sequence
   * dual-ADC jauh di dalam frame PWM 62,5 us. Nilai ADC_CLOCK_DIV di config.h
   * wajib sama karena dipakai untuk offset sinkronisasi TIM8 terhadap ADC. */
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV6;  // 10,667 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}
