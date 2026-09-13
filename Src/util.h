#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef struct {
  uint16_t start;
  int16_t cmdL;
  int16_t cmdR;
  uint16_t checksum;
} SerialCommand;

typedef struct {
  int16_t raw;
  int16_t cmd;
  uint8_t typ;
  uint8_t typDef;
  int16_t min;
  int16_t mid;
  int16_t max;
  int16_t dband;
} InputStruct;

void BLDC_Init(void);
void Input_Lim_Init(void);
bool Input_Init(void);
bool eeprom_persistence_healthy(void);
void UART_EnableRxErrorRecovery(UART_HandleTypeDef *huart);
uint32_t usart3_rx_error_count(void);
uint32_t usart3_rx_restart_count(void);
uint32_t usart3_forced_recovery_count(void);
void usart3_recovery_tick(uint32_t now_ms);

void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepShort(uint8_t freq);
void calcAvgSpeed(void);

void readCommand(void);
void usart3_rx_check(void);

void poweroff(void);
void poweroffPressCheck(void);

void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);

#endif
