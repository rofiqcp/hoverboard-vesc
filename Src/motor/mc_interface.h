#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

#include <stdbool.h>
#include "vesc/datatypes.h"

#ifdef __cplusplus
extern "C" {
#endif

void mc_interface_init(bool reset_conf);
int mc_interface_motor_now(void);
void mc_interface_select_motor_thread(int motor);
const volatile mc_configuration *mc_interface_get_configuration(void);
void mc_interface_set_configuration(mc_configuration *configuration);
bool mc_interface_dccal_done(void);
mc_fault_code mc_interface_get_fault(void);
mc_state mc_interface_get_state(void);
mc_control_mode mc_interface_get_control_mode(void);
void mc_interface_set_duty(float dutyCycle);
void mc_interface_set_pid_speed(float erpm); /* VESC standard: electrical RPM */
void mc_interface_set_pid_pos(float position_deg);
void mc_interface_set_current(float current);
void mc_interface_set_current_rel(float current_rel);
void mc_interface_set_brake_current(float current);
void mc_interface_set_brake_current_rel(float current_rel);
void mc_interface_set_handbrake(float current);
void mc_interface_set_openloop_current(float current, float rpm);
void mc_interface_set_openloop_phase(float current, float phase);
void mc_interface_release_motor(void);
float mc_interface_get_duty_cycle_now(void);
float mc_interface_get_rpm(void); /* VESC standard: electrical RPM */
float mc_interface_get_pid_pos_now(void);
float mc_interface_get_pid_pos_set(void);
float mc_interface_get_tot_current(void);
float mc_interface_get_tot_current_in(void);
float mc_interface_get_id(void);
float mc_interface_get_iq(void);
float mc_interface_get_vd(void);
float mc_interface_get_vq(void);
float mc_interface_get_phase(void);
void mc_interface_get_values(mc_values *values);

/* Explicit dual-motor helpers for bare-metal ISR/application code. */
void mc_interface_set_mode_command_motor(uint8_t mode, int16_t command,
                                         bool run_request, uint16_t openloop_rpm,
                                         bool is_second_motor);
void mc_interface_get_values_motor(mc_values *values, bool is_second_motor);
float mc_interface_get_pid_pos_now_motor(bool is_second_motor);
float mc_interface_get_pid_pos_set_motor(bool is_second_motor);
const volatile mc_configuration *mc_interface_get_configuration_motor(bool is_second_motor);
mc_fault_code mc_interface_get_fault_motor(bool is_second_motor);
mc_state mc_interface_get_state_motor(bool is_second_motor);

/* EEPROM persistence for the VESC-visible subset that affects this board. */
bool mc_interface_store_configuration_motor(bool is_second_motor);
bool mc_interface_load_configuration_motor(bool is_second_motor);
bool mc_interface_read_persisted_configuration_motor(bool is_second_motor,
                                                      mc_configuration *out);
void mc_interface_restore_default_motor(bool is_second_motor, bool store_to_eeprom);

#ifdef __cplusplus
}
#endif


/* LEFT ABI steering calibration/homing. */
bool mc_interface_store_steering_calibration(void);
bool mc_interface_load_steering_calibration(void);
bool mc_interface_steering_calibration_valid(void);
bool mc_interface_reset_steering_calibration(void);
bool mc_interface_steering_logical_inverted(void);
bool mc_interface_set_steering_logical_inverted(bool inverted);
bool mc_interface_steering_boot_home(void);
bool mc_interface_steering_set_current_as_center(void);
bool mc_interface_steering_detect_calibrate(float current, float *offset, float *ratio, bool *inverted,
                                            int32_t *raw_left, int32_t *raw_right, int32_t *span);
void mc_interface_get_steering_span_diag(int32_t *neg1,int32_t *pos1,int32_t *neg2,int32_t *pos2,
                                         int32_t *span1,int32_t *span2,int32_t *tolerance);
bool mc_interface_set_steering_deg(float deg);
float mc_interface_get_steering_deg(void);
#endif
