#ifndef APP_VESC_H_
#define APP_VESC_H_

#include <stdbool.h>
#include <stdint.h>
#include "vesc/datatypes.h"

void app_vesc_init(void);
void app_vesc_defaults(app_configuration *conf, uint8_t controller_id);
const app_configuration *app_vesc_get_configuration(bool second);
bool app_vesc_set_configuration(bool second, const app_configuration *conf);
bool app_vesc_store_configuration(bool second);
bool app_vesc_load_configuration(bool second);
bool app_vesc_read_persisted_configuration(bool second, app_configuration *out);
void app_vesc_process(uint32_t now_ms);
void app_vesc_disable_output(int32_t time_ms);
bool app_vesc_output_disabled(uint32_t now_ms);

float app_vesc_adc_decoded(bool second_channel);
float app_vesc_adc_voltage(bool second_channel);

#endif
