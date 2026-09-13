#ifndef VESC_PROTOCOL_H_
#define VESC_PROTOCOL_H_
#include <stdint.h>
#include <stdbool.h>
void vesc_protocol_init(void);
void vesc_protocol_transport_reset(void);
bool vesc_protocol_rx_byte(uint8_t byte);
bool vesc_protocol_rx_in_progress(void);
void vesc_protocol_process_pending(void);
void vesc_protocol_periodic(uint32_t now_ms);
bool vesc_protocol_link_active(void);
uint32_t vesc_protocol_rx_ok_count(void);
void vesc_protocol_set_probation(bool enabled);
uint32_t vesc_protocol_fw_version_count(void);
uint32_t vesc_protocol_rx_crc_error_count(void);
#endif
