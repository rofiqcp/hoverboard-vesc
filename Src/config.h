#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"
#include "vesc/f103_boot_layout.h"
#include "control_uart.h"

/* UART transport is selected by the PlatformIO environment. */
#ifndef VARIANT_USART
#define VARIANT_USART
#endif

#define CPU_CLOCK_HZ             64000000u
#define PWM_FREQ_HZ              16000u
#define PWM_FREQ                 PWM_FREQ_HZ
#define DEAD_TIME                48
#define DELAY_IN_MAIN_LOOP       5
#define A2BIT_CONV               50  /* EFeru ADC current scaling: 50 count/A */

/* Board-specific VESC duty normalization. VESC Tool keeps the standard
 * [-1.000,+1.000] command range, while 1.000 is mapped to the highest physical
 * modulation verified stable on this two-shunt hoverboard power stage. Keep
 * these hardware scaling constants in config.h so they are never confused with
 * user MC Configuration limits. */
#define VESC_DUTY_PHYSICAL_SCALE_PERMILLE   960
#define FOC_PWM_MARGIN_COUNTS               110
#define FOC_SVPWM_VECTOR_FULL_SAFE        14238
#define FOC_SVPWM_VECTOR_MAX     ((FOC_SVPWM_VECTOR_FULL_SAFE * VESC_DUTY_PHYSICAL_SCALE_PERMILLE) / 1000)
#define ADC_CONV_TIME_7C5        20
#define ADC_CONV_CLOCK_CYCLES    ADC_CONV_TIME_7C5
/* PCLK2=64 MHz; DIV6 => ADCCLK=10,667 MHz (<=14 MHz batas STM32F103).
 * Konstanta ini juga mengubah offset TIM8 agar alignment sampling tetap sama
 * dalam satuan cycle timer 64 MHz. */
#define ADC_CLOCK_DIV            6
#define ADC_TOTAL_CONV_TIME      (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)
/* ADC trigger phase relative to the synchronized TIM1/TIM8 PWM pair.
 * Tuned from real driven-zero current measurements; keep independent from ADC conversion time. */
#define FOC_ADC_PHASE_OFFSET_COUNTS 120u
/* Conservative fixed-trigger current-sample qualification. The sample is only
 * declared window-valid when the centered low-side zero-vector half-window
 * exceeds dead-time + one ADC conversion alignment interval + settling margin. */
#define FOC_CURRENT_SAMPLE_SETTLE_COUNTS  16u
#define FOC_CURRENT_SAMPLE_GUARD_COUNTS   ((uint16_t)(DEAD_TIME + ADC_TOTAL_CONV_TIME + FOC_CURRENT_SAMPLE_SETTLE_COUNTS))

#define BAT_FILT_COEF            655
#define BAT_CALIB_REAL_VOLTAGE   3970
#define BAT_CALIB_ADC            1492
#define BAT_CELLS                10
#define BAT_LVL2_ENABLE          0
#define BAT_LVL1_ENABLE          1
#define BAT_LVL2                 (360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_LVL1                 (350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE
#define BAT_DEAD                 (337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE

#define TEMP_FILT_COEF           655
#define TEMP_CAL_LOW_ADC         1655
#define TEMP_CAL_LOW_DEG_C       358
#define TEMP_CAL_HIGH_ADC        1588
#define TEMP_CAL_HIGH_DEG_C      489
#define TEMP_WARNING_ENABLE      0
#define TEMP_WARNING             600
#define TEMP_POWEROFF_ENABLE     0
#define TEMP_POWEROFF            650

/* Development profile: keep board latched on and ignore the momentary power
 * button while VESC Tool/UART integration is under test. Safety faults still
 * disable the PWM bridge; only the physical OFF latch is bypassed. */
#define POWER_OFF_ENABLE          0
#define POWER_BUTTON_BYPASS       1

#define COM_CTRL                 0
#define SIN_CTRL                 1
#define FOC_CTRL                 2
#define OPEN_MODE                0
#define VLT_MODE                 1
#define SPD_MODE                 2
#define TRQ_MODE                 3
#define SVPWM_MODE               4
#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA
#define CTRL_TYP_SEL             FOC_CTRL
#define CTRL_MOD_REQ             SPD_MODE

/* Mode 4: VESC-style sensorless open-loop PHASE with closed current PI.
 *
 * Host command semantics are intentionally different from modes 1/2/3:
 *   mode 4: |cmd| = Id target in ampere, sign = rotation direction.
 *           Example: start 2,2 -> Id_ref = +2 A on both motors, Iq_ref = 0 A.
 * The electrical angle is generated internally (no Hall/encoder feedback) and
 * fed to the SAME generated Clarke/Park + Id/Iq PI + centered SVPWM path used
 * by normal FOC. Hall remains sampled only for telemetry/RPM diagnostics.
 *
 * This follows the VESC open-loop-phase convention: Id=current, Iq=0, with a
 * phase override. We add a slow phase rotation after alignment so the motor can
 * spin sensorlessly while the current loop regulates the requested Id. */
#define SVPWM_POLE_PAIRS                 15u
#define SVPWM_ALIGN_MS                  600u
#define SVPWM_ALIGN_PHASE             49152u   /* 3*pi/2 on uint16 electrical angle */
#define SVPWM_OPENLOOP_RPM_DEFAULT       10u   /* mechanical RPM, sign comes from command */
#define SVPWM_OPENLOOP_RPM_MAX          300u
#define SVPWM_ACCEL_RPM_PER_S            20u
#define SVPWM_ID_SLEW_A_PER_S             4u
#define SVPWM_MAX_ID_A                   6u   /* sensorless detect/open-loop command ceiling */

/* Torque/current mode uses direct centiampere command semantics:
 *   cmd 50 = 0.50 A, cmd 100 = 1.00 A, cmd 3000 = 30.00 A.
 * STOP braking is therefore also specified in centiamperes.
 * 10 mechanical RPM prevents Hall-boundary brake hunting near zero speed. */
#define TRQ_STOP_RPM_DEADBAND            10
#define DIAG_ENA                 1
#define I_MOT_MAX                30
#define I_DC_MAX                 17
#define N_MOT_MAX                1000  /* legacy mechanical display range; bukan authority COMM_SET_RPM */
#define FIELD_WEAK_ENA           0
#define FIELD_WEAK_MAX           5
#define PHASE_ADV_MAX            25
#define FIELD_WEAK_HI            1000
#define FIELD_WEAK_LO            750

#define INACTIVITY_TIMEOUT       30
#define BEEPS_BACKWARD           0
#define RATE                     480
#define FILTER                   6553

/* Independent signed motor commands over the selected control UART. Mode 3 supports +/-3000 cA
 * to represent the full +/-30 A hard motor-current range; modes 1/2 are still saturated by their
 * own generated-controller limits, while mode 4 clamps to +/-I_MOT_MAX A. */
#define PRI_INPUT1               2, -3000, 0, 3000, 0
#define PRI_INPUT2               2, -3000, 0, 3000, 0
#define INPUTS_NR                1
#define FLASH_WRITE_KEY          0x1002

/* One physical VESC communication interface, selected at build time. */
#if defined(F103_CONTROL_USART2)
#define CONTROL_SERIAL_USART2    1
#define FEEDBACK_SERIAL_USART2
#define DEBUG_SERIAL_USART2
#else
#define CONTROL_SERIAL_USART3    1
#define FEEDBACK_SERIAL_USART3
#define DEBUG_SERIAL_USART3
#endif
#define DEBUG_SERIAL_PROTOCOL
#define SERIAL_START_FRAME       0xABCD
#define SERIAL_BUFFER_SIZE       768
#define SERIAL_DEBUG_LINE_SIZE   96
#define SERIAL_TIMEOUT           160

/* Batas watchdog aktuator lokal. host command layer boleh lebih ketat, tetapi F103 adalah
 * otoritas terakhir yang benar-benar mematikan PWM bila command stream hilang. */
/* Upstream VESC appconf_default.h uses 1000 ms. Keep the same default so the
 * standard 200-ms COMM_ALIVE cadence has ample scheduling margin on F103.
 * The hard realtime watchdog still disables MOE on a genuinely lost link. */
#define VESC_RUNTIME_TIMEOUT_DEFAULT_MS 1000u
#define VESC_RUNTIME_TIMEOUT_MIN_MS       50u
#define VESC_RUNTIME_TIMEOUT_MAX_MS     1000u

#define SERIAL_STATUS_ENABLED    (1u << 0)
#define SERIAL_STATUS_TIMEOUT    (1u << 1)
#define SERIAL_STATUS_LEFT_FAULT (1u << 2)
#define SERIAL_STATUS_RIGHT_FAULT (1u << 3)

#endif
