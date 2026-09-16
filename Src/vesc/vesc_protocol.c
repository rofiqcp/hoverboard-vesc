#include <string.h>
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "defines.h"
#include "util.h"
#include "motor/mcpwm_foc.h"

#ifndef VESC_EXTENDED_TERMINAL
#define VESC_EXTENDED_TERMINAL 0
#endif
#include "motor/foc_math.h"
#include "motor/mcconf_default.h"
#include "motor/mc_interface.h"
#include "vesc/datatypes.h"
#include "vesc/buffer.h"
#include "vesc/crc.h"
#include "vesc/flash_update_f103.h"
#include "vesc/f103_boot_layout.h"
#include "vesc/mcconf_serial.h"
#include "vesc/vesc_protocol.h"
#include "vesc/app_vesc.h"
#include "platform_watchdog.h"

#define VESC_FW_MAJOR               6u
#define VESC_FW_MINOR               0u
#define VESC_LOCAL_ID               1u
#define VESC_SECOND_MOTOR_ID        2u
#define VESC_LINK_HOLD_MS        2000u
#define VESC_MAX_PAYLOAD          700u
#define VESC_MAX_FRAME      (VESC_MAX_PAYLOAD + 7u)
#define VESC_RX_INTERBYTE_TIMEOUT_MS 100u
#define VESC_RX_QUEUE_DEPTH          16u
#define VESC_TX_QUEUE_DEPTH          8u

/* Project-specific extensions are transported inside standard
 * COMM_CUSTOM_APP_DATA, so stock VESC commands remain wire-compatible. */
#define HB_CUSTOM_MAGIC0              0x48u /* 'H' */
#define HB_CUSTOM_MAGIC1              0x42u /* 'B' */
#define HB_CUSTOM_VERSION                2u
#define HB_CUSTOM_GET_DIAG               1u
#define HB_CUSTOM_GET_POS_STATE          2u
#define HB_CUSTOM_SET_POS_LIMITS         3u
#define HB_CUSTOM_SET_POS_TARGET         4u
#define HB_CUSTOM_RESET_POSITION         5u
#define HB_CUSTOM_GET_TUNING             6u
#define HB_CUSTOM_SET_TUNING             7u
#define HB_CUSTOM_SET_ID_TEST            8u
#define HB_CUSTOM_SET_STEERING_DEG        9u /* ROS/Web signed mechanical millidegree */
#define HB_CUSTOM_GET_STEERING_CAL       10u
#define HB_CUSTOM_SET_OPENLOOP_TEST       11u /* bounded commissioning, either motor */
#define HB_CUSTOM_HALL_PIN_TEST            12u /* passive RIGHT Hall electrical test */
#define HB_CUSTOM_STEERING_HOME             13u /* bounded LEFT ABI startup alignment + home */
#define HB_CUSTOM_ENCODER_DEBUG             14u /* read-only LEFT ABI alignment/detect black box */
#define HB_CUSTOM_STEERING_SET_CENTER        15u /* redefine current LEFT ABI position as logical POS180 */
#define HB_CUSTOM_GET_ROTOR_SNAPSHOT          16u /* simultaneous VESC-standard rotor/position diagnostics */
#define HB_CUSTOM_GET_ISR_PROFILE             17u /* read/reset cycle profiler; diagnostic only */
#define HB_CUSTOM_GET_TRACE_META               18u /* compact pre-fault flight-recorder metadata */
#define HB_CUSTOM_GET_TRACE_SAMPLE             19u /* chronological trace sample */
#define HB_CUSTOM_CLEAR_TRACE                  20u /* re-arm recorder after diagnosis */
#define HB_CUSTOM_GET_PLATFORM_HEALTH           21u /* reset cause + IWDG/liveness supervisory state */
#define HB_CUSTOM_FREEZE_TRACE                   22u /* diagnostic-only manual flight-recorder freeze */
#define HB_CUSTOM_GET_FW_UPDATE_STATE             23u /* persistent boot metadata state/size/CRC */
#define HB_CUSTOM_GET_COMMS_HEALTH                 24u /* bounded RX/TX/main-loop transport health */
#define HB_CUSTOM_GET_PLATFORM_INFO                 25u /* fail-closed source/schema/build identity */
#define HB_CUSTOM_GET_ADC_VALIDITY                  26u /* fixed-trigger CCR/zero-window evidence */
#define HB_CUSTOM_ARM_CURRENT_STEP                  27u /* exact control-slot trace step */
#define HB_CUSTOM_GET_STEP_STATUS                   28u
#define HB_CUSTOM_BOOT_HANDOFF                      29u /* ACK first, reset only after UART drains */
#define HB_CUSTOM_GET_POSITION_D_STATE              30u /* read-only process-D sign observability */
#define HB_CUSTOM_START_RELAY_AUTOTUNE              31u /* stage-2 firmware-clocked speed/position relay */
#define HB_CUSTOM_GET_RELAY_AUTOTUNE                32u
#define HB_CUSTOM_ABORT_RELAY_AUTOTUNE              33u
#define HB_PLATFORM_SCHEMA 2u
#define HB_DIAG_SCHEMA 4u
#define HB_ISR_SCHEMA 3u
#define HB_TRACE_SCHEMA 3u
#define HB_FEATURE_BITMAP 0x000003FFu
#if defined(__arm__) || defined(__thumb__)
#define HB_MEMORY_BARRIER() __DMB()
#else
#define HB_MEMORY_BARRIER() __asm__ volatile("" ::: "memory")
#endif
#ifndef F103_BUILD_ID32
#define F103_BUILD_ID32 0u
#define F103_GIT_SHA_HI32 0u
#define F103_GIT_SHA_LO16 0u
#endif

extern UART_HandleTypeDef huart3;
extern int16_t board_temp_deg_c;
extern volatile adc_buf_t adc_buffer;
extern volatile uint32_t buzzerTimer;
extern volatile uint8_t steering_detect_stage;
extern volatile uint8_t encoder_detect_stage;
extern volatile uint8_t encoder_align_stage;
extern volatile uint32_t encoder_align_before_count;
extern volatile uint32_t encoder_align_jog_count;
extern volatile uint32_t encoder_align_back_count;
extern volatile int32_t encoder_align_jog_delta;
extern volatile int32_t encoder_align_back_delta;
extern volatile uint16_t encoder_align_current_ma;
extern volatile int32_t encoder_detect_plus_mdeg;
extern volatile int32_t encoder_detect_minus_mdeg;
extern volatile uint32_t encoder_gpio_edge_a;
extern volatile uint32_t encoder_gpio_edge_b;
extern volatile uint32_t encoder_gpio_edge_pb5;
extern volatile uint32_t encoder_gpio_samples;
#ifdef STM32F103xE
extern volatile uint32_t main_prof_vesc_max_cycles;
extern volatile uint32_t main_prof_house_max_cycles;
extern volatile uint32_t main_prof_tail_max_cycles;
extern volatile uint32_t foc_prof_sensor_max_cycles;
extern volatile uint32_t foc_prof_current_max_cycles;
extern volatile uint32_t foc_prof_regulator_max_cycles;
extern volatile uint32_t foc_prof_svpwm_max_cycles;
extern volatile uint32_t foc_isr_deadline_miss_count;
extern volatile uint32_t foc_prof_pre_max_cycles;
extern volatile uint32_t foc_prof_control_max_cycles;
extern volatile uint32_t foc_prof_post_max_cycles;
extern volatile uint32_t foc_isr_cycles_max;
#else
volatile uint32_t main_prof_vesc_max_cycles = 0u;
volatile uint32_t main_prof_house_max_cycles = 0u;
volatile uint32_t main_prof_tail_max_cycles = 0u;
volatile uint32_t foc_prof_sensor_max_cycles = 0u;
volatile uint32_t foc_prof_current_max_cycles = 0u;
volatile uint32_t foc_prof_regulator_max_cycles = 0u;
volatile uint32_t foc_prof_svpwm_max_cycles = 0u;
volatile uint32_t foc_isr_deadline_miss_count = 0u;
volatile uint32_t foc_prof_pre_max_cycles = 0u;
volatile uint32_t foc_prof_control_max_cycles = 0u;
volatile uint32_t foc_prof_post_max_cycles = 0u;
volatile uint32_t foc_isr_cycles_max = 0u;
#endif

static volatile uint8_t s_rx_active = 0u;
static volatile uint16_t s_rx_index = 0u;
static volatile uint16_t s_rx_expected = 0u;
static volatile uint16_t s_payload_start = 0u;
static volatile uint16_t s_payload_len = 0u;
static uint8_t s_rx_frame[VESC_MAX_FRAME];
static uint8_t s_pending_payload[VESC_RX_QUEUE_DEPTH][VESC_MAX_PAYLOAD];
static uint8_t s_process_payload[VESC_MAX_PAYLOAD];
static uint8_t s_config_payload[VESC_MAX_PAYLOAD];
/* Protocol handlers run serialized in main context. Reuse one transactional
 * MC-config workspace instead of allocating duplicate 724-byte static copies
 * in every setter; this preserves rollback semantics while protecting stack/RAM
 * margin on the 48-KiB F103. */
static mc_configuration s_mc_txn_backup[2];
static mc_configuration s_mc_txn_next[2];
static volatile uint16_t s_pending_len[VESC_RX_QUEUE_DEPTH];
static volatile uint8_t s_pending_head = 0u;
static volatile uint8_t s_pending_tail = 0u;
static volatile uint8_t s_pending_count = 0u;
static volatile uint32_t s_rx_queue_drop = 0u;
static volatile uint32_t s_rx_queue_highwater = 0u;
static uint32_t s_process_last_ms = 0u;
static uint32_t s_process_gap_max_ms = 0u;
static volatile uint32_t s_link_last_ms = 0u;
/* Offset odometer per endpoint. VESC menghitung odometer sebagai nilai dasar
 * ditambah trip distance. Hardware ini tidak memiliki backup-domain RTC, jadi
 * offset bersifat runtime; jarak trip tetap berasal dari edge Hall nyata. */
static int64_t s_odometer_offset_m[2] = {0, 0};
static volatile uint32_t s_rx_ok = 0u;
static volatile uint32_t s_rx_crc_err = 0u;
static volatile uint8_t s_probation = 0u;
static volatile uint32_t s_fw_version_count = 0u;
static volatile uint32_t s_rx_last_byte_ms = 0u;
static volatile uint32_t s_rx_timeout_reset = 0u;
/* Realtime setpoint mailbox. SET_* packets have no reply and repeated packets
 * supersede older setpoints. Coalescing them here prevents stale RPM/current/
 * position commands from filling the generic request FIFO while VESC Tool is
 * simultaneously polling GET_VALUES. The main loop applies one latest command
 * per motor; configuration/detection/read requests remain strictly FIFO. */
typedef struct { uint8_t pending; uint8_t len; uint8_t payload[5]; } rt_cmd_mailbox_t;
static volatile rt_cmd_mailbox_t s_rt_cmd[2];
static volatile uint32_t s_rt_cmd_coalesced = 0u;
/* Framed reply FIFO. The in-flight slot remains owned by DMA until UART gState
 * returns READY, so no response buffer can be overwritten mid-transmission. */
static uint8_t s_tx_frame[VESC_TX_QUEUE_DEPTH][VESC_MAX_FRAME];
static uint16_t s_tx_len[VESC_TX_QUEUE_DEPTH];
static uint8_t s_tx_head = 0u;
static uint8_t s_tx_tail = 0u;
static uint8_t s_tx_active = 0u;
static uint32_t s_tx_queue_drop = 0u;
static uint32_t s_tx_queue_highwater = 0u;
static uint32_t s_tx_start_fail = 0u;
static volatile uint8_t s_boot_handoff_pending=0u;
static uint32_t s_boot_handoff_deadline_ms=0u;
#define HB_BOOT_HANDOFF_TIMEOUT_MS 250u
#ifdef STM32F103xE
/* Wall-cycle profiler COMM_GET_VALUES. DWT elapsed sengaja termasuk preemption
 * FOC karena yang harus dipenuhi VESC Tool adalah latency end-to-end <20 ms. */
static uint32_t s_prof_values_snapshot_max_cycles = 0u;
static uint32_t s_prof_values_position_max_cycles = 0u;
static uint32_t s_prof_values_serialize_max_cycles = 0u;
static uint32_t s_prof_values_tx_max_cycles = 0u;
#endif
static volatile uint8_t s_last_hall_store_ok[2] = {0u, 0u};

/* Bounded commissioning-only open-loop spin. This is deliberately separate
 * from normal VESC position/current control: it may run while the incremental
 * steering encoder is unsynchronised, but it can never stay armed indefinitely.
 * Production steering commands still fail closed until encoder sync/homing. */
static uint8_t s_openloop_test_active = 0u;
static uint8_t s_openloop_test_second = 0u;
static uint32_t s_openloop_test_deadline = 0u;
#define HB_OPENLOOP_TEST_MAX_MA       2000
#define HB_OPENLOOP_TEST_MAX_MERPM   20000 /* 20.000 electrical RPM */
#define HB_OPENLOOP_TEST_MAX_MS       1000u
#define HB_OPENLOOP_TEST_MIN_MS         50u

/* Stock VESC handles COMM_DETECT_HALL_FOC as a blocking command in a dedicated
 * worker thread. This F103 target is bare-metal, so reproduce the same external
 * behavior with a cooperative state machine: UART request/reply remains alive
 * during the ~12 s detection, while only the selected motor is locked. Detect
 * itself never applies or stores the table; VESC Tool's Apply + SET_MCCONF does. */
typedef enum {
    HALL_DETECT_IDLE = 0,
    HALL_DETECT_ALIGN,
    HALL_DETECT_SWEEP
} hall_detect_stage_t;

typedef struct {
    uint8_t active;
    uint8_t second;
    hall_detect_stage_t stage;
    uint8_t pass;
    uint8_t waiting_sample;
    uint16_t align_step;
    int16_t degree;
    uint32_t align_start_time;
    uint32_t next_time;
    float current_a;
    uint8_t restore_backup;
    mc_configuration backup;
    int64_t sum_s[8];
    int64_t sum_c[8];
    uint16_t samples[8];
} hall_detect_job_t;

static hall_detect_job_t s_hall_detect;

typedef enum {
    DETECT_ALL_IDLE = 0,
    DETECT_ALL_ENCODER,
    DETECT_ALL_HALL,
    DETECT_ALL_RL_ALIGN,
    DETECT_ALL_RL_LOW,
    DETECT_ALL_RL_STEP,
    DETECT_ALL_RL_HIGH,
    DETECT_ALL_FLUX_RAMP,
    DETECT_ALL_FLUX_SAMPLE,
    DETECT_ALL_FLUX_RETURN
} detect_all_stage_t;

typedef struct {
    uint8_t active;
    uint8_t motor_index;
    detect_all_stage_t stage;
    uint32_t stage_start_time;
    uint32_t next_sample_time;
    uint32_t sample_n;
    float max_power_loss;
    float min_current_in;
    float max_current_in;
    float openloop_rpm;
    float sl_erpm;
    float current_low;
    float current_high;
    float flux_current;
    float flux_target_erpm;
    float sum_i;
    float sum_v;
    float sum_i_raw;
    float sum_v_raw;
    float sum_erpm;
    mc_configuration backup[2];
    mc_configuration result[2];
    float r[2], l[2], ld_lq[2], flux[2], imax[2];
    float encoder_offset;
    float encoder_ratio;
    uint8_t encoder_inverted;
    float low_i[2], low_v[2], high_i[2], high_v[2];
    float low_i_raw[2], low_v_raw[2], high_i_raw[2], high_v_raw[2];
    uint8_t hall[2][8];
    uint8_t left_sensor_encoder;
    /* 0 = Detect-All. Otherwise this same cooperative motor-model worker is
     * serving a stock VESC measurement command (25/26/57) for one endpoint. */
    uint8_t standalone_cmd;
    uint8_t standalone_second;
    float standalone_flux_current;
    float standalone_flux_ramp_erpm_s;
    uint8_t flux_bounded_left;
    float flux_base_phase_deg;
    float flux_return_phase_deg;
    int32_t flux_guard_min_counts;
    int32_t flux_guard_max_counts;
} detect_all_job_t;

static detect_all_job_t s_detect_all;
static int16_t s_detect_all_last_detail = 0;
static uint32_t detect_time_now(void);
static bool detect_time_due(uint32_t now, uint32_t deadline);
static void mcpwm_foc_hall_detect_process(uint32_t now_ms);
static void conf_general_detect_apply_all_foc_process(uint32_t now_ms);
static void measure_r_l_imax_f103_start(uint8_t mi, uint32_t now_time);
static void reply_mcconf(bool second, COMM_PACKET_ID id);
/* VESC Tool rotor-position display stream. Upstream commands.c stores one
 * display_position_mode per VESC instance; this dual virtual target stores the
 * last selected endpoint so COMM_FORWARD_CAN ID2 behaves like a second VESC. */
static volatile disp_pos_mode s_display_pos_mode = DISP_POS_MODE_NONE;
static volatile uint8_t s_display_second = 0u;
static uint32_t s_display_prev_ms = 0u;

static void rx_reset(void) {
    s_rx_active = 0u;
    s_rx_index = 0u;
    s_rx_expected = 0u;
    s_payload_start = 0u;
    s_payload_len = 0u;
}

static void app_defaults(app_configuration *a, uint8_t id) {
    app_vesc_defaults(a, id);
}

void vesc_protocol_init(void) {
    rx_reset();
    s_pending_head = s_pending_tail = s_pending_count = 0u;
    memset((void *)s_pending_len, 0, sizeof(s_pending_len));
    s_rx_queue_drop = 0u;
    s_rx_queue_highwater = 0u;
    s_process_last_ms = 0u;
    s_process_gap_max_ms = 0u;
    s_link_last_ms = 0u;
    s_rx_ok = 0u;
    s_rx_crc_err = 0u;
    s_probation = 0u;
    s_fw_version_count = 0u;
    memset((void *)s_rt_cmd, 0, sizeof(s_rt_cmd));
    s_rt_cmd_coalesced = 0u;
    s_tx_head = s_tx_tail = s_tx_active = 0u;
    memset(s_tx_len, 0, sizeof(s_tx_len));
    s_tx_queue_drop = 0u;
    s_tx_queue_highwater = 0u;
    s_tx_start_fail = 0u;
    s_last_hall_store_ok[0] = s_last_hall_store_ok[1] = 0u;
    memset(&s_hall_detect, 0, sizeof(s_hall_detect));
    memset(&s_detect_all, 0, sizeof(s_detect_all));
    s_display_pos_mode = DISP_POS_MODE_NONE;
    s_display_second = 0u;
    s_display_prev_ms = 0u;
    app_vesc_init();
}

void vesc_protocol_transport_reset(void) {
    /* Communication recovery only. Preserve MC/App configuration and EEPROM,
     * but discard every in-flight command/reply so a corrupt frame can never
     * execute after the UART is restarted. */
    rx_reset();
    s_pending_head = s_pending_tail = s_pending_count = 0u;
    memset((void *)s_pending_len, 0, sizeof(s_pending_len));
    memset((void *)s_rt_cmd, 0, sizeof(s_rt_cmd));
    s_tx_head = s_tx_tail = s_tx_active = 0u;
    memset(s_tx_len, 0, sizeof(s_tx_len));
    s_link_last_ms = 0u;
    s_openloop_test_active = 0u;
    memset(&s_hall_detect, 0, sizeof(s_hall_detect));
    memset(&s_detect_all, 0, sizeof(s_detect_all));
}

bool vesc_protocol_rx_in_progress(void) { return s_rx_active != 0u; }
void vesc_protocol_set_probation(bool enabled) {
    s_probation = enabled ? 1u : 0u;
    if (enabled) {
        mcpwm_foc_release_motor(false); mcpwm_foc_release_motor(true);
        mcpwm_foc_force_bridges_off();
    }
}
uint32_t vesc_protocol_fw_version_count(void) { return s_fw_version_count; }

static bool rt_command_extract(const uint8_t *vp, uint16_t n, uint8_t *motor, const uint8_t **cmdp) {
    if (!vp || !motor || !cmdp) return false;
    const uint8_t *c = vp; uint16_t cn = n; uint8_t mi = 0u;
    if (n >= 3u && vp[0] == COMM_FORWARD_CAN && vp[1] == VESC_SECOND_MOTOR_ID) {
        c = vp + 2u; cn = (uint16_t)(n - 2u); mi = 1u;
    }
    if (cn != 5u) return false;
    switch ((COMM_PACKET_ID)c[0]) {
    case COMM_SET_DUTY: case COMM_SET_CURRENT: case COMM_SET_CURRENT_BRAKE:
    case COMM_SET_HANDBRAKE: case COMM_SET_RPM: case COMM_SET_POS:
    /* COMM_SET_CURRENT_REL is the same kind of no-reply realtime setpoint as
     * COMM_SET_CURRENT (1 cmd byte + int32 = 5 bytes either way) and was
     * missing from this list, so it fell through to the plain FIFO instead
     * of getting "latest setpoint wins" coalescing like its sibling. */
    case COMM_SET_CURRENT_REL:
        *motor = mi; *cmdp = c; return true;
    default: return false;
    }
}

static bool probation_packet_allowed(const uint8_t *p, uint16_t len);

static void complete_frame(void) {
    const uint16_t p = s_payload_start;
    const uint16_t n = s_payload_len;
    bool valid = false;
    if (n > 0u && n <= VESC_MAX_PAYLOAD && s_rx_expected >= (uint16_t)(p + n + 3u)) {
        const uint16_t rx_crc = (uint16_t)(((uint16_t)s_rx_frame[p + n] << 8) |
                                           (uint16_t)s_rx_frame[p + n + 1u]);
        const uint16_t calc = vesc_crc16(&s_rx_frame[p], n);
        valid = (s_rx_frame[p + n + 2u] == 3u) && (rx_crc == calc);
    }
    if (valid) {
        /* COMM_ALIVE is the VESC motor-command watchdog heartbeat and has no
         * response payload. Handle it immediately after CRC validation instead
         * of putting it behind realtime/config requests. This mirrors
         * timeout_reset() semantics and guarantees a full RX FIFO cannot make a
         * one-click VESC Tool setpoint expire while Send Alive is active. */
        const uint8_t *vp=&s_rx_frame[p];
        /* TEST images are transport-probed before they are trusted. Drop every
         * valid-but-disallowed packet here, before ALIVE or realtime mailboxes
         * can touch motor ownership/watchdogs. Count it as valid wire traffic
         * so UART recovery does not fight a healthy host, but create no side
         * effect and do not mark the motor link active. */
        if (s_probation && !probation_packet_allowed(vp,n)) {
            s_rx_ok++;
            rx_reset();
            return;
        }
        const bool alive_local=(n==1u && vp[0]==COMM_ALIVE);
        const bool alive_right=(n==3u && vp[0]==COMM_FORWARD_CAN &&
                                vp[1]==VESC_SECOND_MOTOR_ID && vp[2]==COMM_ALIVE);
        if (alive_local || alive_right) {
            mcpwm_foc_vesc_override_touch(alive_right);
            s_link_last_ms=HAL_GetTick();
            s_rx_ok++;
            rx_reset();
            return;
        }
        {
            uint8_t mi=0u; const uint8_t *cp=0;
            if (rt_command_extract(vp,n,&mi,&cp)) {
                rt_cmd_mailbox_t *mb=(rt_cmd_mailbox_t *)&s_rt_cmd[mi];
                if (mb->pending) s_rt_cmd_coalesced++;
                mb->len=5u;
                memcpy((void *)mb->payload,cp,5u);
                mb->pending=1u;
                /* Reset VESC timeout at wire-receive time. Even if the main loop
                 * is busy for a few milliseconds, a valid fresh setpoint must
                 * count as alive exactly like upstream timeout_reset(). */
                mcpwm_foc_vesc_override_touch(mi!=0u);
                s_link_last_ms=HAL_GetTick(); s_rx_ok++; rx_reset(); return;
            }
        }

        /* UART DMA/ISR can deliver several VESC Tool requests before the 5-ms
         * main loop processes them. V15 had one pending slot and silently
         * discarded every additional valid packet; that is especially harmful
         * to realtime polling. Keep a small bounded FIFO instead. */
        if (s_pending_count < VESC_RX_QUEUE_DEPTH) {
            const uint8_t slot = s_pending_head;
            memcpy(s_pending_payload[slot], &s_rx_frame[p], n);
            s_pending_len[slot] = n;
            s_pending_head = (uint8_t)((slot + 1u) % VESC_RX_QUEUE_DEPTH);
            s_pending_count++;
            if ((uint32_t)s_pending_count > s_rx_queue_highwater) s_rx_queue_highwater = s_pending_count;
        } else {
            s_rx_queue_drop++;
        }
        s_link_last_ms = HAL_GetTick();
        s_rx_ok++;
    } else {
        s_rx_crc_err++;
    }
    rx_reset();
}

bool vesc_protocol_rx_byte(uint8_t byte) {
    const uint32_t now_ms = HAL_GetTick();
    if (s_rx_active && (uint32_t)(now_ms - s_rx_last_byte_ms) > VESC_RX_INTERBYTE_TIMEOUT_MS) {
        /* A truncated/corrupt long frame must never poison all later traffic.
         * Direct USB-UART upload chunks are explicitly paced on the validated direct F103 UART link, so 12 ms leaves margin while preventing a false start byte from swallowing later RT frames. */
        rx_reset();
        s_rx_timeout_reset++;
    }
    s_rx_last_byte_ms = now_ms;
    if (!s_rx_active) {
        if (byte != 2u && byte != 3u && byte != 4u) return false;
        s_rx_active = 1u;
        s_rx_frame[0] = byte;
        s_rx_index = 1u;
        s_link_last_ms = now_ms; /* suppress unsolicited terminal telemetry immediately */
        return true;
    }

    if (s_rx_index >= VESC_MAX_FRAME) {
        rx_reset();
        return true;
    }
    s_rx_frame[s_rx_index++] = byte;

    const uint8_t start = s_rx_frame[0];
    if (start == 2u && s_rx_index == 2u) {
        s_payload_start = 2u;
        s_payload_len = s_rx_frame[1];
    } else if (start == 3u && s_rx_index == 3u) {
        s_payload_start = 3u;
        s_payload_len = (uint16_t)(((uint16_t)s_rx_frame[1] << 8) | s_rx_frame[2]);
    } else if (start == 4u && s_rx_index == 4u) {
        /* STM32F103 implementation deliberately caps packets below 64 KiB. */
        const uint32_t n = ((uint32_t)s_rx_frame[1] << 16) |
                           ((uint32_t)s_rx_frame[2] << 8) | s_rx_frame[3];
        if (n > VESC_MAX_PAYLOAD) { rx_reset(); return true; }
        s_payload_start = 4u;
        s_payload_len = (uint16_t)n;
    }

    if (s_payload_start != 0u && s_rx_expected == 0u) {
        if (s_payload_len == 0u || s_payload_len > VESC_MAX_PAYLOAD) {
            rx_reset();
            return true;
        }
        s_rx_expected = (uint16_t)(s_payload_start + s_payload_len + 3u);
    }
    if (s_rx_expected != 0u && s_rx_index == s_rx_expected) complete_frame();
    return true;
}

bool vesc_protocol_link_active(void) {
    return (uint32_t)(HAL_GetTick() - s_link_last_ms) < VESC_LINK_HOLD_MS;
}
uint32_t vesc_protocol_rx_ok_count(void) { return s_rx_ok; }
uint32_t vesc_protocol_rx_crc_error_count(void) { return s_rx_crc_err; }

static uint8_t vesc_tx_queue_count(void) {
    const uint8_t head=s_tx_head, tail=s_tx_tail;
    return head>=tail ? (uint8_t)(head-tail) :
        (uint8_t)(VESC_TX_QUEUE_DEPTH-(uint8_t)(tail-head));
}

static void vesc_tx_service(void) {
    /* Single-producer (main) / single-consumer (UART-TC ISR) ring. The slot at
     * s_tx_tail remains owned by DMA until gState returns READY. Advancing the
     * consumer from HAL_UART_TxCpltCallback starts the next queued VESC reply
     * immediately instead of waiting up to one main-loop period. */
    if (s_tx_active) {
        if (huart3.gState != HAL_UART_STATE_READY) return;
        s_tx_active = 0u;
        if (s_tx_head != s_tx_tail) {
            s_tx_len[s_tx_tail] = 0u;
            s_tx_tail = (uint8_t)((s_tx_tail + 1u) % VESC_TX_QUEUE_DEPTH);
        }
    }
    while (s_tx_head != s_tx_tail && huart3.gState == HAL_UART_STATE_READY) {
        const uint8_t slot = s_tx_tail;
        const uint16_t n = s_tx_len[slot];
        if (n == 0u || n > VESC_MAX_FRAME) {
            s_tx_len[slot]=0u;
            s_tx_tail = (uint8_t)((s_tx_tail + 1u) % VESC_TX_QUEUE_DEPTH);
            continue;
        }
        if (HAL_UART_Transmit_DMA(&huart3, s_tx_frame[slot], n) == HAL_OK) {
            s_tx_active = 1u;
        } else {
            s_tx_start_fail++;
        }
        return;
    }
}

void vesc_protocol_tx_complete_isr(void) {
    vesc_tx_service();
}

static void uart_send_payload(const uint8_t *payload, uint16_t len) {
    if (!payload || len == 0u || len > VESC_MAX_PAYLOAD) return;
    vesc_tx_service();
    const uint8_t next=(uint8_t)((s_tx_head+1u)%VESC_TX_QUEUE_DEPTH);
    if (next == s_tx_tail) {
        s_tx_queue_drop++;
        return;
    }
    const uint8_t slot = s_tx_head;
    uint8_t *tx = s_tx_frame[slot];
    uint16_t i = 0u;
    if (len <= 255u) {
        tx[i++] = 2u;
        tx[i++] = (uint8_t)len;
    } else {
        tx[i++] = 3u;
        tx[i++] = (uint8_t)(len >> 8);
        tx[i++] = (uint8_t)len;
    }
    memcpy(&tx[i], payload, len);
    i = (uint16_t)(i + len);
    const uint16_t crc = vesc_crc16(payload, len);
    tx[i++] = (uint8_t)(crc >> 8);
    tx[i++] = (uint8_t)crc;
    tx[i++] = 3u;
    s_tx_len[slot] = i;
    s_tx_head = next;
    const uint8_t queued=vesc_tx_queue_count();
    if ((uint32_t)queued > s_tx_queue_highwater) s_tx_queue_highwater=queued;
    vesc_tx_service();
}

static float wrap_angle_diff_deg(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static float steering_vesc_position_deg(void) {
    /* VESC Tool-compatible public position for LEFT steering. This is a raw
     * normalized actuator coordinate 0..360 derived from the calibrated encoder
     * count span. Vehicle wheel-angle calibration belongs to ROS/ROS Web. */
    const float mech=mc_interface_get_steering_deg();
    float pos=(mech-MCCONF_STEERING_POS_MIN_DEG) * 360.0f /
        (MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);
    if(pos<0.0f)pos=0.0f;
    if(pos>360.0f)pos=360.0f;
    return pos;
}

static bool display_rotor_pos(bool second, disp_pos_mode mode, float *out) {
    if (!out) return false;
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    switch (mode) {
    case DISP_POS_MODE_OBSERVER:
        /* Observer and Hall share the same 0..360 electrical coordinate. At
         * standstill the flux observer has no BEMF authority, so Hall axes fall
         * back to the live Hall phase learned by Detect Hall. While observer
         * state is valid, keep publishing the independent observer angle. */
        if (mcpwm_foc_observer_valid(second)) *out=mcpwm_foc_get_phase_observer_motor(second);
        else *out=(float)m->m_phase_hall*(360.0f/65536.0f);
        return true;
    case DISP_POS_MODE_ENCODER:
        /* Diagnostic convention for this dual controller: ABI shows mechanical
         * encoder 0..360. A Hall-configured axis uses this same VESC Tool button
         * to show m_phase_hall 0..360 from the detected/interpolated Hall table. */
        if (!second && m->m_encoder_configured) *out=mcpwm_foc_get_encoder_position_motor(false);
        else *out=(float)m->m_phase_hall*(360.0f/65536.0f);
        return true;
    case DISP_POS_MODE_PID_POS:
        /* VESC Tool position remains standard 0..360. LEFT maps that public
         * actuator coordinate onto the calibrated steering travel. */
        *out = !second ? steering_vesc_position_deg() : mc_interface_get_pid_pos_now_motor(second);
        return true;
    case DISP_POS_MODE_PID_POS_ERROR: {
        if(m->m_pos_pid_phase_mode){
            const float target=mc_interface_get_pid_pos_set_motor(second);
            const float now=mc_interface_get_pid_pos_now_motor(second);
            float err=wrap_angle_diff_deg(target,now);
            if(second)err=-err; /* normalisasi virtual motor kanan */
            *out=err;
        }else{
            const int32_t dc=m->m_position_target_counts-m->m_position_counts;
            float err;
            if(!second && m->m_encoder_configured && m->m_encoder_counts>=4u){
                err=(float)dc*360.0f/(float)m->m_encoder_counts;
                if(m->m_conf.foc_encoder_inverted)err=-err;
            }else{
                const float pp=(float)mcpwm_foc_get_pole_pairs(second);
                err=(pp>0.0f)?((float)dc*60.0f/pp):0.0f;
                if(second)err=-err;
            }
            *out=err;
        }
        return true;
    }
    case DISP_POS_MODE_ENCODER_OBSERVER_ERROR:
        /* Upstream VESC: angle_difference(observer electrical, encoder electrical). */
        if(!second && m->m_encoder_configured && mcpwm_foc_observer_valid(false))
            *out = wrap_angle_diff_deg(mcpwm_foc_get_phase_observer_motor(false),
                                       mcpwm_foc_get_phase_encoder_motor(false));
        else *out = 0.0f;
        return true;
    case DISP_POS_MODE_HALL_OBSERVER_ERROR: {
        /* Upstream VESC: angle_difference(observer electrical, Hall electrical). */
        const float hall = (float)m->m_phase_hall * (360.0f / 65536.0f);
        *out = mcpwm_foc_observer_valid(second) ?
            wrap_angle_diff_deg(mcpwm_foc_get_phase_observer_motor(second), hall) : 0.0f;
        return true;
    }
    default:
        return false;
    }
}

void vesc_protocol_periodic(uint32_t now_ms) {
    vesc_tx_service();
    if(s_boot_handoff_pending){
#if defined(__arm__) || defined(__thumb__)
        const bool uart_tc=(huart3.Instance->SR & USART_SR_TC)!=0u;
#else
        const bool uart_tc=true;
#endif
        const bool tx_drained=(vesc_tx_queue_count()==0u)&&(s_tx_active==0u)&&(huart3.gState==HAL_UART_STATE_READY)&&uart_tc;
        if(tx_drained){s_boot_handoff_pending=0u;f103_fw_reset_to_bootloader();}
        if((int32_t)(now_ms-s_boot_handoff_deadline_ms)>=0){s_boot_handoff_pending=0u;}
    }
    if (s_rx_active && (uint32_t)(now_ms - s_rx_last_byte_ms) > VESC_RX_INTERBYTE_TIMEOUT_MS) {
        rx_reset();
        s_rx_timeout_reset++;
    }
    const uint32_t detector_now = detect_time_now();
    mcpwm_foc_hall_detect_process(detector_now);
    conf_general_detect_apply_all_foc_process(detector_now);
    if (s_openloop_test_active) {
        const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(s_openloop_test_second != 0u);
        const bool expired = detect_time_due(detector_now, s_openloop_test_deadline);
        if (expired || !m || m->m_fault != FAULT_CODE_NONE) {
            mc_interface_select_motor_thread(s_openloop_test_second ? 2 : 1);
            mc_interface_release_motor();
            mc_interface_select_motor_thread(1);
            s_openloop_test_active = 0u;
        }
    }
    const disp_pos_mode mode = s_display_pos_mode;
    if (mode == DISP_POS_MODE_NONE) return;
    if ((uint32_t)(now_ms - s_display_prev_ms) < 10u) return;
    /* COMM_SET_DETECT enables the same unsolicited rotor-position stream used
     * by VESC Tool. Unlike the old implementation, do not require an idle UART
     * before sampling: that reduced a nominal 100-Hz stream to ~60 Hz whenever
     * 50-Hz RT values for two motors were also being requested.
     *
     * Rotor telemetry is low priority and latest-sample by construction. Service
     * the DMA queue first, then enqueue only while at least one slot remains
     * reserved for solicited GET_VALUES/config replies. If the link is truly
     * saturated we simply retry the newest rotor sample on the next main-loop
     * pass instead of queueing stale positions. */
    vesc_tx_service();
    if (vesc_tx_queue_count() >= (VESC_TX_QUEUE_DEPTH - 2u)) return;
    s_display_prev_ms = now_ms;
    const bool second = s_display_second != 0u;
    float pos = 0.0f;
    if (!display_rotor_pos(second, mode, &pos)) return;
    /* COMM_FORWARD_CAN on dual-motor VESC only selects motor thread 2.
     * It never changes packet coordinates; direction is handled by the normal
     * Motor Configuration DIR_MULT path in mc_interface. */
    uint8_t b[5];
    int32_t i = 0;
    b[i++] = COMM_ROTOR_POSITION;
    buffer_append_int32(b, (int32_t)(pos * 100000.0f), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void read_uuid(uint8_t out[12], bool second) {
#ifdef STM32F103xE
    const volatile uint8_t *uid = (const volatile uint8_t *)0x1FFFF7E8u;
    for (uint8_t i = 0u; i < 12u; ++i) out[i] = uid[i];
#else
    static const uint8_t host_uid[12] = {0x48,0x4f,0x56,0x45,0x52,0x46,0x31,0x30,0x33,0x46,0x4f,0x43};
    memcpy(out, host_uid, sizeof(host_uid));
#endif
    if (second) out[11]++;
}

static void reply_fw_version(bool second) {
    uint8_t b[80];
    int32_t i = 0;
    const char *hw = second ? "motor_right" : "motor_left";
    const char *fw = second ? "motor_right" : "motor_left";
    uint8_t uid[12];
    read_uuid(uid, second);
    b[i++] = COMM_FW_VERSION;
    b[i++] = VESC_FW_MAJOR;
    b[i++] = VESC_FW_MINOR;
    strcpy((char *)&b[i], hw); i += (int32_t)strlen(hw) + 1;
    memcpy(&b[i], uid, 12u); i += 12;
    b[i++] = 1u;                 /* pairing_done */
    b[i++] = 0u;                 /* FW_TEST_VERSION_NUMBER */
    b[i++] = HW_TYPE_VESC;
    b[i++] = 0u;                 /* custom config count */
    b[i++] = 0u;                 /* phase filters */
    b[i++] = 0u;                 /* QML HW */
    b[i++] = 0u;                 /* QML app */
    b[i++] = 0u;                 /* NRF flags */
    strcpy((char *)&b[i], fw); i += (int32_t)strlen(fw) + 1;
    /* VESC firmware 6.00 ends COMM_FW_VERSION after FW_NAME. Do not append
     * post-6.00 fields; older VESC Tool parsers otherwise see trailing bytes. */
    uart_send_payload(b, (uint16_t)i);
}

static void get_values_normalized(bool second, mc_values *v) {
    mc_interface_get_values_motor(v, second);
    const mc_configuration *conf=(const mc_configuration *)mc_interface_get_configuration_motor(second);
    /* COMM_GET_VALUES upstream melewati mc_interface: direction inversion
     * berlaku pada RPM/duty/Iq/Vq/tachometer, sedangkan motor/input current dan
     * d-axis tetap tidak dibalik. Position adalah user PID position. */
    if(conf && conf->m_invert_direction){
        v->rpm=-v->rpm; v->iq=-v->iq; v->duty_now=-v->duty_now; v->vq=-v->vq;
        v->tachometer=-v->tachometer;
    }
    v->position=!second ? steering_vesc_position_deg() : mc_interface_get_pid_pos_now_motor(second);
    /* Hoverboard temperature calibration is deci-degC (358 = 35.8C). */
    v->temp_mos = (float)board_temp_deg_c * 0.1f;
    v->temp_mos_1 = v->temp_mos;
    v->temp_mos_2 = v->temp_mos;
    v->temp_mos_3 = v->temp_mos;
    v->temp_motor = 0.0f;
    v->vesc_id = second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID;
}

static void append_values_fields(uint8_t *b, int32_t *i, const mc_values *v, uint32_t mask) {
    if (mask & (1u << 0)) buffer_append_float16(b, v->temp_mos, 1e1f, i);
    if (mask & (1u << 1)) buffer_append_float16(b, v->temp_motor, 1e1f, i);
    if (mask & (1u << 2)) buffer_append_float32(b, v->current_motor, 1e2f, i);
    if (mask & (1u << 3)) buffer_append_float32(b, v->current_in, 1e2f, i);
    if (mask & (1u << 4)) buffer_append_float32(b, v->id, 1e2f, i);
    if (mask & (1u << 5)) buffer_append_float32(b, v->iq, 1e2f, i);
    if (mask & (1u << 6)) buffer_append_float16(b, v->duty_now, 1e3f, i);
    if (mask & (1u << 7)) buffer_append_float32(b, v->rpm, 1e0f, i);
    if (mask & (1u << 8)) buffer_append_float16(b, v->v_in, 1e1f, i);
    if (mask & (1u << 9)) buffer_append_float32(b, v->amp_hours, 1e4f, i);
    if (mask & (1u << 10)) buffer_append_float32(b, v->amp_hours_charged, 1e4f, i);
    if (mask & (1u << 11)) buffer_append_float32(b, v->watt_hours, 1e4f, i);
    if (mask & (1u << 12)) buffer_append_float32(b, v->watt_hours_charged, 1e4f, i);
    if (mask & (1u << 13)) buffer_append_int32(b, v->tachometer, i);
    if (mask & (1u << 14)) buffer_append_int32(b, v->tachometer_abs, i);
    if (mask & (1u << 15)) b[(*i)++] = (uint8_t)v->fault_code;
    if (mask & (1u << 16)) buffer_append_float32(b, v->position, 1e6f, i);
    if (mask & (1u << 17)) b[(*i)++] = (uint8_t)v->vesc_id;
    if (mask & (1u << 18)) {
        buffer_append_float16(b, v->temp_mos_1, 1e1f, i);
        buffer_append_float16(b, v->temp_mos_2, 1e1f, i);
        buffer_append_float16(b, v->temp_mos_3, 1e1f, i);
    }
    if (mask & (1u << 19)) buffer_append_float32(b, v->vd, 1e3f, i);
    if (mask & (1u << 20)) buffer_append_float32(b, v->vq, 1e3f, i);
    if (mask & (1u << 21)) b[(*i)++] = 0u; /* timeout/kill status */
}

static void send_values_packet(bool second, bool selective, uint32_t mask) {
    uint8_t b[128];
    int32_t i = 0;
    b[i++] = selective ? COMM_GET_VALUES_SELECTIVE : COMM_GET_VALUES;
    if (selective) buffer_append_uint32(b, mask, &i);

#ifdef STM32F103xE
    {
        /* STM32F103 has no FPU. Use the integer-scaled snapshot for both full
         * and selective GET_VALUES so telemetry latency stays bounded while
         * the 16-kHz FOC ISR is active. Selective polling previously fell back
         * to the soft-float path and could exceed the host timeout under load. */
        mcpwm_foc_values_scaled_t v;
        uint32_t pv0=DWT->CYCCNT;
        mcpwm_foc_get_values_scaled(&v, second);
        uint32_t pv1=DWT->CYCCNT;
        uint32_t dt=(uint32_t)(pv1-pv0);
        if(dt>s_prof_values_snapshot_max_cycles)s_prof_values_snapshot_max_cycles=dt;
        const mc_configuration *conf=(const mc_configuration *)mc_interface_get_configuration_motor(second);
        const int32_t dir=(conf && conf->m_invert_direction)?-1:1;
        if(mask&(1u<<0)) buffer_append_int16(b, board_temp_deg_c, &i);
        if(mask&(1u<<1)) buffer_append_int16(b, 0, &i);
        if(mask&(1u<<2)) buffer_append_int32(b, v.current_motor_x100, &i);
        if(mask&(1u<<3)) buffer_append_int32(b, v.current_in_x100, &i);
        if(mask&(1u<<4)) buffer_append_int32(b, v.id_x100, &i);
        if(mask&(1u<<5)) buffer_append_int32(b, dir*v.iq_x100, &i);
        if(mask&(1u<<6)) buffer_append_int16(b, (int16_t)(dir*(int32_t)v.duty_x1000), &i);
        if(mask&(1u<<7)) buffer_append_int32(b, dir*v.erpm, &i);
        if(mask&(1u<<8)) buffer_append_int16(b, v.vin_x10, &i);
        if(mask&(1u<<9)) buffer_append_int32(b, v.ah_x10000, &i);
        if(mask&(1u<<10)) buffer_append_int32(b, v.ah_charged_x10000, &i);
        if(mask&(1u<<11)) buffer_append_int32(b, v.wh_x10000, &i);
        if(mask&(1u<<12)) buffer_append_int32(b, v.wh_charged_x10000, &i);
        if(mask&(1u<<13)) buffer_append_int32(b, dir*v.tachometer, &i);
        if(mask&(1u<<14)) buffer_append_int32(b, v.tachometer_abs, &i);
        if(mask&(1u<<15)) b[i++]=v.fault;
        if(mask&(1u<<16)) {
            uint32_t pv2=DWT->CYCCNT;
            /* Keep the standard VESC position field in its public 0..360 actuator
             * coordinate. LEFT steering adapts physical travel into that domain. */
            const float pos = !second ? steering_vesc_position_deg() :
                mc_interface_get_pid_pos_now_motor(second);
            buffer_append_float32(b, pos, 1e6f, &i);
            uint32_t pv3=DWT->CYCCNT;
            dt=(uint32_t)(pv3-pv2);
            if(dt>s_prof_values_position_max_cycles)s_prof_values_position_max_cycles=dt;
        }
        if(mask&(1u<<17)) b[i++]=second?VESC_SECOND_MOTOR_ID:VESC_LOCAL_ID;
        if(mask&(1u<<18)) {
            buffer_append_int16(b, board_temp_deg_c, &i);
            buffer_append_int16(b, board_temp_deg_c, &i);
            buffer_append_int16(b, board_temp_deg_c, &i);
        }
        if(mask&(1u<<19)) buffer_append_int32(b, v.vd_x1000, &i);
        if(mask&(1u<<20)) buffer_append_int32(b, dir*v.vq_x1000, &i);
        if(mask&(1u<<21)) b[i++]=0u;
        uint32_t pv4=DWT->CYCCNT;
        dt=(uint32_t)(pv4-pv1);
        if(dt>s_prof_values_serialize_max_cycles)s_prof_values_serialize_max_cycles=dt;
        uart_send_payload(b,(uint16_t)i);
        uint32_t pv5=DWT->CYCCNT;
        dt=(uint32_t)(pv5-pv4);
        if(dt>s_prof_values_tx_max_cycles)s_prof_values_tx_max_cycles=dt;
        return;
    }
#endif

    mc_values v;
    get_values_normalized(second, &v);
    append_values_fields(b, &i, &v, mask);
    uart_send_payload(b, (uint16_t)i);
}

static void reply_values(bool second, bool selective, const uint8_t *data, uint16_t len) {
    uint32_t mask = 0xffffffffu;
    if (selective) {
        if (len < 4u) return;
        int32_t r = 0;
        mask = buffer_get_uint32(data, &r);
    }
    send_values_packet(second, selective, mask);

    /* Stock VESC semantics are strict request/reply: the host (VESC Tool)
     * chooses the polling rate. Never arm an unsolicited values stream here. */
}

/** Batasi nilai float tanpa menarik dependensi utilitas VESC yang tidak dipakai. */
static float vesc_clampf(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * Hitung level baterai seperti VESC 6.00 dari tipe baterai, jumlah sel, dan Vin.
 * Estimasi Wh hanya diaktifkan bila kapasitas Ah memang diisi pengguna; firmware
 * tidak mengarang kapasitas baterai yang tidak diketahui.
 */
static float setup_battery_level(const mc_configuration *conf, float vin, float *wh_left) {
    if (wh_left) *wh_left = 0.0f;
    if (!conf || conf->si_battery_cells < 1) return 0.0f;

    const float cells = (float)conf->si_battery_cells;
    const float cell_v = vin / cells;
    float level = 0.0f;
    float ah_left = 0.0f;
    float ah_total = conf->si_battery_ah;
    float avg_voltage_left = 0.0f;

    if (conf->si_battery_type == BATTERY_TYPE_LIION_3_0__4_2) {
        float x = vesc_clampf((cell_v - 3.2f) / (4.2f - 3.2f), 0.0f, 1.0f);
        const float x2 = x * x;
        const float x3 = x2 * x;
        const float x4 = x3 * x;
        const float x5 = x4 * x;
        /* Polynomial resmi utils_batt_liion_norm_v_to_capacity VESC 6.00. */
        level = -2.979767f * x5 + 5.487810f * x4 - 3.501286f * x3 +
                1.675683f * x2 + 0.317147f * x;
        level = vesc_clampf(level, 0.0f, 1.0f);
        ah_total *= 0.85f;
        ah_left = level * ah_total;
        avg_voltage_left = (3.2f * cells + vin) * 0.5f;
    } else if (conf->si_battery_type == BATTERY_TYPE_LIIRON_2_6__3_6) {
        level = vesc_clampf((cell_v - 2.6f) / (3.6f - 2.6f), 0.0f, 1.0f);
        ah_left = level * ah_total;
        avg_voltage_left = (2.8f * cells + vin) * 0.5f;
    } else {
        level = vesc_clampf((cell_v - 2.1f) / (2.36f - 2.1f), 0.0f, 1.0f);
        ah_left = level * ah_total;
        avg_voltage_left = (2.1f * cells + vin) * 0.5f;
    }

    if (wh_left && ah_total > 0.0f) *wh_left = ah_left * avg_voltage_left;
    return level;
}

/**
 * Turunkan speed dan distance dari ERPM/tachometer dengan rumus VESC 6.00.
 * Tachometer firmware ini bertambah sekali per edge Hall, tepat enam edge per
 * revolusi elektrik, sehingga skala wheel*pi/(3*poles*gear) tetap identik.
 */
static void setup_motion_values(bool second, const mc_values *values,
                                float *speed_mps, float *distance_m,
                                float *distance_abs_m) {
    const mc_configuration *conf =
        (const mc_configuration *)mc_interface_get_configuration_motor(second);
    float wheel = conf->si_wheel_diameter;
    float gear = conf->si_gear_ratio;
    int poles = conf->si_motor_poles;
    if (!(wheel > 0.001f && wheel < 5.0f)) wheel = MCCONF_SI_WHEEL_DIAMETER;
    if (!(gear > 0.0f)) gear = 1.0f;
    if (poles < 2 || (poles & 1)) poles = 2;
    const float pi = 3.14159265358979323846f;
    const float pole_pairs = (float)poles * 0.5f;
    if (speed_mps) *speed_mps = (values->rpm / pole_pairs / 60.0f) * wheel * pi / gear;
    const float tacho_scale = (wheel * pi) / (3.0f * (float)poles * gear);
    if (distance_m) *distance_m = (float)values->tachometer * tacho_scale;
    if (distance_abs_m) *distance_abs_m = (float)values->tachometer_abs * tacho_scale;
}

/** Hitung odometer VESC dari offset SET_ODOMETER dan distance_abs Hall. */
static uint32_t setup_odometer_m(bool second, float distance_abs_m) {
    /* Odometer packet berukuran uint32 meter; clamp sebelum cast agar tidak
     * membutuhkan float->int64 helper pada Cortex-M3. */
    float trip_f=distance_abs_m>=0.0f?distance_abs_m:0.0f;
    uint32_t trip_u=trip_f>4294967040.0f?UINT32_MAX:(uint32_t)trip_f;
    int64_t trip=(int64_t)trip_u;
    int64_t value = s_odometer_offset_m[second ? 1u : 0u] + trip;
    if (value < 0) value = 0;
    if (value > (int64_t)UINT32_MAX) value = (int64_t)UINT32_MAX;
    return (uint32_t)value;
}

static void send_values_setup_packet(bool second, bool selective, uint32_t mask) {
    uint8_t b[128];
    int32_t i = 0;
    const COMM_PACKET_ID id = selective ? COMM_GET_VALUES_SETUP_SELECTIVE : COMM_GET_VALUES_SETUP;
    b[i++] = (uint8_t)id;
    if (selective) buffer_append_uint32(b, mask, &i);

    mc_values v;
    get_values_normalized(second, &v);
    mc_values totals = v;
    if (!second) {
        /* COMM_GET_VALUES_SETUP upstream menjumlahkan controller lokal dan CAN
         * yang aktif. Motor-2 board ini adalah endpoint virtual CAN ID2, jadi
         * agregasikan arus dan counter energi yang sama seperti VESC dual. */
        mc_values right_values;
        get_values_normalized(true, &right_values);
        totals.current_motor += right_values.current_motor;
        totals.current_in += right_values.current_in;
        totals.amp_hours += right_values.amp_hours;
        totals.amp_hours_charged += right_values.amp_hours_charged;
        totals.watt_hours += right_values.watt_hours;
        totals.watt_hours_charged += right_values.watt_hours_charged;
    }
    float speed_mps = 0.0f, distance_m = 0.0f, distance_abs_m = 0.0f, wh_left = 0.0f;
    setup_motion_values(second, &v, &speed_mps, &distance_m, &distance_abs_m);
    const mc_configuration *conf =
        (const mc_configuration *)mc_interface_get_configuration_motor(second);
    const float battery_level = setup_battery_level(conf, v.v_in, &wh_left);
    if (mask & (1u << 0)) buffer_append_float16(b, v.temp_mos, 1e1f, &i);
    if (mask & (1u << 1)) buffer_append_float16(b, v.temp_motor, 1e1f, &i);
    if (mask & (1u << 2)) buffer_append_float32(b, totals.current_motor, 1e2f, &i);
    if (mask & (1u << 3)) buffer_append_float32(b, totals.current_in, 1e2f, &i);
    if (mask & (1u << 4)) buffer_append_float16(b, v.duty_now, 1e3f, &i);
    if (mask & (1u << 5)) buffer_append_float32(b, v.rpm, 1e0f, &i);
    if (mask & (1u << 6)) buffer_append_float32(b, speed_mps, 1e3f, &i);
    if (mask & (1u << 7)) buffer_append_float16(b, v.v_in, 1e1f, &i);
    if (mask & (1u << 8)) buffer_append_float16(b, battery_level, 1e3f, &i);
    if (mask & (1u << 9)) buffer_append_float32(b, totals.amp_hours, 1e4f, &i);
    if (mask & (1u << 10)) buffer_append_float32(b, totals.amp_hours_charged, 1e4f, &i);
    if (mask & (1u << 11)) buffer_append_float32(b, totals.watt_hours, 1e4f, &i);
    if (mask & (1u << 12)) buffer_append_float32(b, totals.watt_hours_charged, 1e4f, &i);
    if (mask & (1u << 13)) buffer_append_float32(b, distance_m, 1e3f, &i);
    if (mask & (1u << 14)) buffer_append_float32(b, distance_abs_m, 1e3f, &i);
    if (mask & (1u << 15)) buffer_append_float32(b, v.position, 1e6f, &i);
    if (mask & (1u << 16)) b[i++] = (uint8_t)v.fault_code;
    if (mask & (1u << 17)) b[i++] = (uint8_t)v.vesc_id;
    if (mask & (1u << 18)) b[i++] = second ? 1u : 2u;
    if (mask & (1u << 19)) buffer_append_float32(b, wh_left, 1e3f, &i);
    if (mask & (1u << 20)) buffer_append_uint32(b, setup_odometer_m(second, distance_abs_m), &i);
    if (mask & (1u << 21)) buffer_append_uint32(b, HAL_GetTick(), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void reply_values_setup(bool second, bool selective, const uint8_t *data, uint16_t len) {
    uint32_t mask = 0xffffffffu;
    if (selective) {
        if (len < 4u) return;
        int32_t r = 0;
        mask = buffer_get_uint32(data, &r);
    }
    send_values_setup_packet(second, selective, mask);
    /* GET_VALUES_SETUP is also one request -> one reply, matching upstream VESC. */
}

/* Forward declaration: dipakai command config/current sebelum implementasi detector. */
static bool hall_detect_motor_locked(bool second);

static void touch_motor(bool second) { mcpwm_foc_vesc_override_touch(second); }

static void reply_mcconf(bool second, COMM_PACKET_ID id) {
    mc_configuration *const c=&s_mc_txn_next[0];
    int32_t i = 0;
    s_config_payload[i++] = (uint8_t)id;
    if (id == COMM_GET_MCCONF_DEFAULT) {
        mcpwm_foc_get_default_configuration(c, second);
    } else {
        *c = *mc_interface_get_configuration_motor(second);
    }
    const int32_t n = confgenerator_serialize_mcconf(&s_config_payload[i], c);
    if (n > 0 && (uint32_t)(i + n) <= sizeof(s_config_payload)) uart_send_payload(s_config_payload, (uint16_t)(i + n));
}

static void set_mcconf(bool second, const uint8_t *data, uint16_t len) {
    mc_configuration *const backup=&s_mc_txn_backup[0];
    mc_configuration *const c=&s_mc_txn_next[0];
    bool committed = false;
    *backup = *mc_interface_get_configuration_motor(second);
    *c = *backup;
    const int32_t expected = confgenerator_serialize_mcconf(s_config_payload, c);
    if (expected > 0 && len >= (uint16_t)expected && confgenerator_deserialize_mcconf(data, c)) {
        c->motor_type = MOTOR_TYPE_FOC;
        if (c->l_current_max < 0.1f) c->l_current_max = 0.1f;
        if (c->l_current_max > (float)I_MOT_MAX) c->l_current_max = (float)I_MOT_MAX;
        if (c->l_current_min > -0.1f) c->l_current_min = -0.1f;
        if (c->l_current_min < -(float)I_MOT_MAX) c->l_current_min = -(float)I_MOT_MAX;
        {
            float commanded_abs = c->l_current_max;
            if (-c->l_current_min > commanded_abs) commanded_abs = -c->l_current_min;
            if (!(c->l_abs_current_max >= commanded_abs) ||
                c->l_abs_current_max > MCCONF_L_ABS_CURRENT_MAX) {
                c->l_abs_current_max = MCCONF_L_ABS_CURRENT_MAX;
            }
        }
        if (!(c->l_max_duty > 0.0f) || c->l_max_duty > MCCONF_L_MAX_DUTY) c->l_max_duty=MCCONF_L_MAX_DUTY;
        if (!(c->l_in_current_max >= 0.1f) || c->l_in_current_max > (float)I_DC_MAX) c->l_in_current_max=MCCONF_L_IN_CURRENT_MAX;
        if (!(c->l_in_current_min <= -0.1f) || c->l_in_current_min < -(float)I_DC_MAX) c->l_in_current_min=MCCONF_L_IN_CURRENT_MIN;
        if (!(c->m_duty_ramp_step >= 0.0001f && c->m_duty_ramp_step <= 0.20f)) c->m_duty_ramp_step=MCCONF_DUTY_RAMP_STEP_DEFAULT;
        /* LEFT supports the standard VESC ABI sensor-port mode on PB6/PB7.
         * RIGHT has no ABI timer route and is intentionally Hall-only. */
        if(second){
            c->m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
            c->foc_sensor_mode=FOC_SENSOR_MODE_HALL;
        }else if(c->m_sensor_port_mode==SENSOR_PORT_MODE_ABI){
            if(c->foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER && c->foc_sensor_mode!=FOC_SENSOR_MODE_ENCODER_AB)
                c->foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
            if(c->m_encoder_counts<4 || c->m_encoder_counts>65536)c->m_encoder_counts=(int32_t)MCCONF_ENCODER_COUNTS_DEFAULT;
            if(!(c->foc_encoder_ratio>=0.01f && c->foc_encoder_ratio<=MCCONF_ENCODER_RATIO_MAX))c->foc_encoder_ratio=(float)MCCONF_POLE_PAIRS_LEFT;
            while(c->foc_encoder_offset>=360.0f)c->foc_encoder_offset-=360.0f;
            while(c->foc_encoder_offset<0.0f)c->foc_encoder_offset+=360.0f;
        }else{
            c->m_sensor_port_mode=SENSOR_PORT_MODE_HALL; c->foc_sensor_mode=FOC_SENSOR_MODE_HALL;
        }
        if (c->si_motor_poles < 2u || (c->si_motor_poles & 1u)) c->si_motor_poles = 30u;
        if (!(c->si_gear_ratio >= 0.01f && c->si_gear_ratio <= 1000.0f)) c->si_gear_ratio = 1.0f;
        mc_interface_select_motor_thread(second ? 2 : 1);
        mc_interface_set_configuration(c);
        /* VESC Tool SET_MCCONF is a configuration write, not a motor-detect
         * command. ABI without index is intentionally left UNSYNCED here; the
         * FOC bridge gate already prevents closed-loop drive until an explicit
         * encoder alignment/detect procedure establishes electrical zero. This
         * keeps SET_MCCONF deterministic and prevents unexpected rotor motion. */
        if(mc_interface_store_configuration_motor(second)){
            committed=true;
        }else{
            /* Never leave RAM newer than EEPROM while reporting failure. */
            mc_interface_set_configuration(backup);
            (void)mc_interface_store_configuration_motor(second);
        }
    }
    /* Persistent VESC writes ACK only after the EEPROM commit barrier succeeds. */
    if (committed) { uint8_t ack = COMM_SET_MCCONF; uart_send_payload(&ack, 1u); }
}

static void reply_appconf(bool second, COMM_PACKET_ID id) {
    int32_t i = 0;
    app_configuration defaults;
    const app_configuration *a;
    s_config_payload[i++] = (uint8_t)id;
    if (id == COMM_GET_APPCONF_DEFAULT) {
        app_defaults(&defaults, second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID);
        a = &defaults;
    } else {
        a = app_vesc_get_configuration(second);
    }
    const int32_t n = confgenerator_serialize_appconf(&s_config_payload[i], a);
    if (n > 0 && (uint32_t)(i + n) <= sizeof(s_config_payload)) uart_send_payload(s_config_payload, (uint16_t)(i + n));
}

/**
 * Terapkan App Configuration VESC 6.00.
 *
 * `store_to_eeprom=false` dipakai COMM_SET_APPCONF_NO_STORE sehingga tombol
 * pengaturan sementara di VESC Tool benar-benar tidak mengubah flash.
 */
static void set_appconf(bool second, const uint8_t *data, uint16_t len,
                        bool store_to_eeprom, COMM_PACKET_ID ack_id) {
    const app_configuration backup=*app_vesc_get_configuration(second);
    app_configuration tmp=backup;
    const int32_t expected = confgenerator_serialize_appconf(s_config_payload, &tmp);
    if (expected > 0 && len >= (uint16_t)expected && confgenerator_deserialize_appconf(data, &tmp)) {
        bool ok=app_vesc_set_configuration(second,&tmp);
        if(ok && store_to_eeprom)ok=app_vesc_store_configuration(second);
        if(!ok){
            (void)app_vesc_set_configuration(second,&backup);
            if(store_to_eeprom)(void)app_vesc_store_configuration(second);
            return;
        }
        uint8_t ack = (uint8_t)ack_id;
        uart_send_payload(&ack, 1u);
    }
}

/** COMM_SET_CURRENT_REL: persis Commands -> mc_interface_set_current_rel VESC. */
static void set_current_relative(bool second, const uint8_t *data, uint16_t len) {
    if (len < 4u || hall_detect_motor_locked(second)) return;
    int32_t ind = 0;
    const float rel = (float)buffer_get_int32(data, &ind) / 100000.0f;
    mc_interface_select_motor_thread(second ? 2 : 1);
    touch_motor(second);
    mc_interface_set_current_rel(rel);
}

/** Kirim dua ambang battery-cut sesuai format wire VESC 6.00. */
static void reply_battery_cut(bool second) {
    const mc_configuration *c = (const mc_configuration *)mc_interface_get_configuration_motor(second);
    uint8_t b[12]; int32_t i = 0;
    b[i++] = COMM_GET_BATTERY_CUT;
    buffer_append_float32(b, c->l_battery_cut_start, 1e3f, &i);
    buffer_append_float32(b, c->l_battery_cut_end, 1e3f, &i);
    uart_send_payload(b, (uint16_t)i);
}

/**
 * Terapkan battery-cut secara live. Derating arusnya dilakukan oleh loop FOC
 * menggunakan threshold ADC yang sudah diprekomputasi saat config diterapkan.
 */
static void set_battery_cut(bool second, const uint8_t *data, uint16_t len) {
    if (len < 10u || hall_detect_motor_locked(second)) return;
    int32_t i = 0;
    const float start = buffer_get_float32(data, 1e3f, &i);
    const float end = buffer_get_float32(data, 1e3f, &i);
    const bool store = data[i++] != 0u;
    const bool forward = data[i++] != 0u;
    if (!(start > end && end >= 0.0f && start <= 80.0f)) return;

    bool target[2]={false,false};
    for(uint8_t motor=0u;motor<2u;++motor){
        const bool target_second=motor!=0u;
        target[motor]=(target_second==second)||(forward&&!second);
        if(!target[motor])continue;
        s_mc_txn_backup[motor]=*mc_interface_get_configuration_motor(target_second);
        s_mc_txn_next[motor]=s_mc_txn_backup[motor];
        s_mc_txn_next[motor].l_battery_cut_start=start;s_mc_txn_next[motor].l_battery_cut_end=end;
        mc_interface_select_motor_thread(target_second?2:1);
        mc_interface_set_configuration(&s_mc_txn_next[motor]);
    }
    bool ok=true;
    if(store){
        for(uint8_t motor=0u;motor<2u && ok;++motor)if(target[motor])
            ok=mc_interface_store_configuration_motor(motor!=0u);
        if(!ok){
            /* Roll RAM and any already-committed endpoint back to the old image. */
            for(uint8_t motor=0u;motor<2u;++motor)if(target[motor]){
                mc_interface_select_motor_thread(motor?2:1);
                mc_interface_set_configuration(&s_mc_txn_backup[motor]);
                (void)mc_interface_store_configuration_motor(motor!=0u);
            }
        }
    }
    mc_interface_select_motor_thread(second?2:1);
    if(ok){uint8_t ack=COMM_SET_BATTERY_CUT;uart_send_payload(&ack,1u);}
}

/** Kirim konfigurasi limit sementara yang dipakai halaman Setup VESC Tool. */
static void reply_mcconf_temp(bool second) {
    const mc_configuration *c = (const mc_configuration *)mc_interface_get_configuration_motor(second);
    uint8_t b[64]; int32_t i = 0;
    b[i++] = COMM_GET_MCCONF_TEMP;
    buffer_append_float32_auto(b, c->l_current_min_scale, &i);
    buffer_append_float32_auto(b, c->l_current_max_scale, &i);
    buffer_append_float32_auto(b, c->l_min_erpm, &i);
    buffer_append_float32_auto(b, c->l_max_erpm, &i);
    buffer_append_float32_auto(b, c->l_min_duty, &i);
    buffer_append_float32_auto(b, c->l_max_duty, &i);
    buffer_append_float32_auto(b, c->l_watt_min, &i);
    buffer_append_float32_auto(b, c->l_watt_max, &i);
    buffer_append_float32_auto(b, c->l_in_current_min, &i);
    buffer_append_float32_auto(b, c->l_in_current_max, &i);
    b[i++] = c->si_motor_poles;
    buffer_append_float32_auto(b, c->si_gear_ratio, &i);
    buffer_append_float32_auto(b, c->si_wheel_diameter, &i);
    uart_send_payload(b, (uint16_t)i);
}

/**
 * Terapkan COMM_SET_MCCONF_TEMP/SETUP sesuai layout VESC 6.00. Pada variant
 * SETUP, batas kecepatan masuk dalam m/s dan dikonversi ke ERPM menggunakan
 * pole, gear ratio, dan diameter roda dari MC Config. `forward_can` diterapkan
 * ke endpoint virtual motor-2 tanpa membuat CAN fisik palsu.
 */
static void set_mcconf_temp(bool second, COMM_PACKET_ID packet_id,
                            const uint8_t *data, uint16_t len) {
    if (len < 36u || hall_detect_motor_locked(second)) return;
    int32_t i = 0;
    const bool store = data[i++] != 0u;
    const bool forward = data[i++] != 0u;
    const bool ack = data[i++] != 0u;
    const bool divide = data[i++] != 0u;
    const float current_min_scale = buffer_get_float32_auto(data, &i);
    const float current_max_scale = buffer_get_float32_auto(data, &i);
    const float limit_min_in = buffer_get_float32_auto(data, &i);
    const float limit_max_in = buffer_get_float32_auto(data, &i);
    const float duty_min = buffer_get_float32_auto(data, &i);
    const float duty_max = buffer_get_float32_auto(data, &i);
    float watt_min = buffer_get_float32_auto(data, &i);
    float watt_max = buffer_get_float32_auto(data, &i);
    float input_min = 0.0f, input_max = 0.0f;
    const bool has_input_limits = len >= (uint16_t)(i + 8);
    if (has_input_limits) {
        input_min = buffer_get_float32_auto(data, &i);
        input_max = buffer_get_float32_auto(data, &i);
    }
    const float controllers = (divide && forward && !second) ? 2.0f : 1.0f;
    watt_min /= controllers; watt_max /= controllers;

    bool target[2]={false,false};
    for (uint8_t motor = 0u; motor < 2u; ++motor) {
        const bool target_second = motor != 0u;
        target[motor]=(target_second==second)||(forward&&!second);
        if(!target[motor])continue;
        s_mc_txn_backup[motor]=*mc_interface_get_configuration_motor(target_second);
        s_mc_txn_next[motor]=s_mc_txn_backup[motor];
        mc_configuration *c=&s_mc_txn_next[motor];
        c->l_current_min_scale = current_min_scale;
        c->l_current_max_scale = current_max_scale;
        if (packet_id == COMM_SET_MCCONF_TEMP_SETUP) {
            const float wheel = c->si_wheel_diameter > 0.001f ? c->si_wheel_diameter : MCCONF_SI_WHEEL_DIAMETER;
            const float gear = c->si_gear_ratio > 0.0f ? c->si_gear_ratio : 1.0f;
            const float fact = (((float)c->si_motor_poles * 0.5f) * 60.0f * gear) /
                               (wheel * 3.14159265358979323846f);
            c->l_min_erpm = limit_min_in * fact; c->l_max_erpm = limit_max_in * fact;
        } else {
            c->l_min_erpm = limit_min_in; c->l_max_erpm = limit_max_in;
        }
        c->l_min_duty = duty_min; c->l_max_duty = duty_max;
        c->l_watt_min = watt_min; c->l_watt_max = watt_max;
        if (has_input_limits) { c->l_in_current_min = input_min; c->l_in_current_max = input_max; }
        mc_interface_select_motor_thread(target_second ? 2 : 1);
        mc_interface_set_configuration(c);
    }
    bool ok=true;
    if(store){
        for(uint8_t motor=0u;motor<2u && ok;++motor)if(target[motor])
            ok=mc_interface_store_configuration_motor(motor!=0u);
        if(!ok){
            for(uint8_t motor=0u;motor<2u;++motor)if(target[motor]){
                mc_interface_select_motor_thread(motor?2:1);
                mc_interface_set_configuration(&s_mc_txn_backup[motor]);
                (void)mc_interface_store_configuration_motor(motor!=0u);
            }
        }
    }
    mc_interface_select_motor_thread(second ? 2 : 1);
    if (ack && ok) { uint8_t id = (uint8_t)packet_id; uart_send_payload(&id, 1u); }
}

static void reply_decoded_adc(void) {
    uint8_t b[20];
    int32_t i = 0;
    b[i++] = COMM_GET_DECODED_ADC;
    buffer_append_int32(b, (int32_t)(app_vesc_adc_decoded(false) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_voltage(false) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_decoded(true) * 1000000.0f), &i);
    buffer_append_int32(b, (int32_t)(app_vesc_adc_voltage(true) * 1000000.0f), &i);
    uart_send_payload(b, (uint16_t)i);
}

/* VESC Tool app-realtime polling always rotates PPM, ADC and Nunchuk.
 * This board has no physical PPM/Nunchuk input, but the commands still need a
 * standards-compatible response so the host-side per-command timeout state is
 * cleared instead of generating a false transport timeout. */
static void reply_decoded_ppm(void) {
    uint8_t b[9];
    int32_t i = 0;
    b[i++] = COMM_GET_DECODED_PPM;
    buffer_append_int32(b, 0, &i); /* decoded input */
    buffer_append_int32(b, 0, &i); /* last pulse length */
    uart_send_payload(b, (uint16_t)i);
}

static void reply_decoded_chuk(void) {
    uint8_t b[5];
    int32_t i = 0;
    b[i++] = COMM_GET_DECODED_CHUK;
    buffer_append_int32(b, 0, &i);
    uart_send_payload(b, (uint16_t)i);
}

/* Firmware does not maintain the upstream accumulated STAT_VALUES structure.
 * A zero returned mask is the protocol-safe way to report no optional stats
 * fields while still acknowledging COMM_GET_STATS immediately. */
static void reply_stats(void) {
    uint8_t b[5];
    int32_t i = 0;
    b[i++] = COMM_GET_STATS;
    buffer_append_uint32(b, 0u, &i);
    uart_send_payload(b, (uint16_t)i);
}

/* Detect timing is derived from the same 16-kHz ADC/PWM interrupt that
 * drives FOC. This counter cannot lose ticks when the high-priority FOC ISR
 * saturates the CPU, unlike SysTick/HAL_GetTick. Using PWM ticks also preserves
 * the proven master hardware sampling cadence and adds no extra interrupt. */
static uint32_t detect_time_now(void) {
#ifdef STM32F103xE
    return buzzerTimer;
#else
    return HAL_GetTick();
#endif
}

static uint32_t detect_time_after_ms(uint32_t now, uint32_t ms) {
#ifdef STM32F103xE
    const uint32_t ticks_per_ms = (uint32_t)PWM_FREQ / 1000u;
    return now + ticks_per_ms * ms;
#else
    return now + ms;
#endif
}

static uint32_t detect_time_elapsed_ms(uint32_t start, uint32_t now) {
#ifdef STM32F103xE
    const uint32_t ticks_per_ms = (uint32_t)PWM_FREQ / 1000u;
    return ticks_per_ms ? (uint32_t)(now - start) / ticks_per_ms : 0u;
#else
    return (uint32_t)(now - start);
#endif
}

static bool detect_time_due(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static bool hall_detect_motor_locked(bool second) {
    return s_hall_detect.active && (s_hall_detect.second != 0u) == second;
}

static void mcpwm_foc_hall_detect_start(bool second, float current) {
    const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
    float max_i = m->m_conf.l_current_max;
    if (max_i <= 0.0f || max_i > (float)I_MOT_MAX) max_i = (float)I_MOT_MAX;
    if(current<0.0f)current=-current;
    if(current<0.10f)current=0.10f;
    if(current>max_i)current=max_i;

    memset(&s_hall_detect, 0, sizeof(s_hall_detect));
    s_hall_detect.active = 1u;
    s_hall_detect.second = second ? 1u : 0u;
    s_hall_detect.stage = HALL_DETECT_ALIGN;
    s_hall_detect.current_a = current;
    s_hall_detect.align_start_time = detect_time_now();
    s_hall_detect.next_time = s_hall_detect.align_start_time;
    mc_interface_select_motor_thread(second ? 2 : 1);
    mcpwm_foc_set_openloop_phase(0.0f, 0.0f, second);
    mcpwm_foc_vesc_override_touch(second);
    mc_interface_select_motor_thread(1);
}

static void detect_all_reply(int16_t result) {
    uint8_t b[3];
    int32_t i=0;
    b[i++]=COMM_DETECT_APPLY_ALL_FOC;
    buffer_append_int16(b,result,&i);
    uart_send_payload(b,(uint16_t)i);
}

static int16_t detect_all_fault_result(bool second) {
    const mc_fault_code f=mc_interface_get_fault_motor(second);
    return f==FAULT_CODE_NONE ? -10 : (int16_t)((int)f-100);
}

static void detect_all_release_all(void) {
    mcpwm_foc_rl_capture_stop(false); mcpwm_foc_rl_capture_stop(true);
    mc_interface_select_motor_thread(1); mc_interface_release_motor(); mcpwm_foc_vesc_override_clear(false);
    mc_interface_select_motor_thread(2); mc_interface_release_motor(); mcpwm_foc_vesc_override_clear(true);
    mc_interface_select_motor_thread(1);
}

static void detect_all_restore_backups(void) {
    for(uint8_t mi=0u;mi<2u;++mi){
        mc_interface_select_motor_thread(mi?2:1);
        mc_configuration c=s_detect_all.backup[mi];
        mc_interface_set_configuration(&c);
    }
    mc_interface_select_motor_thread(1);
}

static void detect_all_reset_sample(void) {
    s_detect_all.sample_n=0u;
    s_detect_all.sum_i=0.0; s_detect_all.sum_v=0.0;
    s_detect_all.sum_i_raw=0.0; s_detect_all.sum_v_raw=0.0;
    s_detect_all.sum_erpm=0.0;
}

static void detect_all_apply_runtime(uint8_t mi) {
    mc_interface_select_motor_thread(mi?2:1);
    mc_configuration c=s_detect_all.result[mi];
    mc_interface_set_configuration(&c);
    mc_interface_select_motor_thread(1);
}

static void detect_all_apply_common_limits(mc_configuration *c) {
    if(!c)return;
    if(s_detect_all.min_current_in < -0.001f && s_detect_all.min_current_in >= -(float)I_DC_MAX)
        c->l_in_current_min=s_detect_all.min_current_in;
    if(s_detect_all.max_current_in > 0.001f && s_detect_all.max_current_in <= (float)I_DC_MAX)
        c->l_in_current_max=s_detect_all.max_current_in;
    if(s_detect_all.openloop_rpm > 0.001f && s_detect_all.openloop_rpm <= (float)MCCONF_OPENLOOP_RPM_MAX)
        c->foc_openloop_rpm=s_detect_all.openloop_rpm;
    if(s_detect_all.sl_erpm > 0.001f && s_detect_all.sl_erpm <= MCCONF_L_MAX_ERPM)
        c->foc_sl_erpm=s_detect_all.sl_erpm;
}

static void detect_all_prepare_hall(uint8_t mi) {
    mc_configuration *c=&s_detect_all.result[mi];
    detect_all_apply_common_limits(c);
    memcpy(c->foc_hall_table,s_detect_all.hall[mi],8u);
    c->motor_type=MOTOR_TYPE_FOC;
    c->sensor_mode=SENSOR_MODE_SENSORED;
    c->m_sensor_port_mode=SENSOR_PORT_MODE_HALL;
    c->foc_sensor_mode=FOC_SENSOR_MODE_HALL;
}

static bool detect_all_prepare_encoder_left(void) {
    float off=1001.0f, ratio=0.0f; bool inv=false;
    if(!mcpwm_foc_encoder_detect(MCCONF_STEERING_DETECT_CURRENT_START_A,false,&off,&ratio,&inv))return false;
    mc_configuration *c=&s_detect_all.result[0];
    detect_all_apply_common_limits(c);
    c->motor_type=MOTOR_TYPE_FOC;
    c->sensor_mode=SENSOR_MODE_SENSORED;
    c->m_sensor_port_mode=SENSOR_PORT_MODE_ABI;
    c->foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
    c->m_encoder_counts=(int32_t)MCCONF_ENCODER_COUNTS_DEFAULT;
    c->si_motor_poles=(uint8_t)(2u*MCCONF_POLE_PAIRS_LEFT);
    c->foc_encoder_offset=off; c->foc_encoder_ratio=ratio; c->foc_encoder_inverted=inv;
    s_detect_all.encoder_offset=off; s_detect_all.encoder_ratio=ratio; s_detect_all.encoder_inverted=inv?1u:0u;
    /* Detect-All mengikuti semantik VESC: komisioning sensor hanya mencari
     * parameter elektrik encoder. Hard-stop dan span steering LEFT adalah
     * kalibrasi mekanik proyek dan hanya boleh disentuh oleh Detect Encoder. */
    return true;
}

static bool detect_all_commit(void) {
    bool ok=true;
    for(uint8_t mi=0u;mi<2u;++mi){
        mc_interface_select_motor_thread(mi?2:1);
        mc_configuration c=s_detect_all.result[mi];
        mc_interface_set_configuration(&c);
        if(!mc_interface_store_configuration_motor(mi!=0u)){ok=false;break;}
        s_last_hall_store_ok[mi]=(mi==1u)?1u:0u;
    }
    if(!ok){
        /* Best-effort atomic rollback: restore RAM and rewrite both old configs.
         * The per-motor EEPROM signature is written last by store_configuration. */
        detect_all_restore_backups();
        for(uint8_t mi=0u;mi<2u;++mi){
            mc_interface_select_motor_thread(mi?2:1);
            (void)mc_interface_store_configuration_motor(mi!=0u);
            s_last_hall_store_ok[mi]=0u;
        }
    }
    mc_interface_select_motor_thread(1);
    return ok;
}

static void detect_all_finish(int16_t result) {
    detect_all_release_all();
    memset(&s_hall_detect,0,sizeof(s_hall_detect));
    if(result>=0){
        if(!detect_all_commit()) result=-1;
    }else{
        detect_all_restore_backups();
    }
    s_detect_all.active=0u;
    s_detect_all.stage=DETECT_ALL_IDLE;
    /* Utility::detectAllFoc brackets the command with APP_DISABLE_OUTPUT. We
     * also clear our internal safety gate here so raw protocol clients cannot
     * leave application output suppressed for the full 180-s guard window. */
    app_vesc_disable_output(0);
    /* Stock VESC sends fresh MC configuration before the final result. */
    if(result>=0) reply_mcconf(false,COMM_GET_MCCONF);
    detect_all_reply(result);
}

static void standalone_measure_finish(bool success) {
    const bool second=s_detect_all.standalone_second!=0u;
    const uint8_t cmd=s_detect_all.standalone_cmd;
    mcpwm_foc_rl_capture_stop(second);
    mc_interface_select_motor_thread(second?2:1);
    mc_interface_release_motor();
    mcpwm_foc_vesc_override_clear(second);
    mc_configuration restore=s_detect_all.backup[second?1u:0u];
    mc_interface_set_configuration(&restore);
    mc_interface_select_motor_thread(1);
    app_vesc_disable_output(0);

    uint8_t b[20]; int32_t i=0;
    b[i++]=cmd;
    const uint8_t mi=second?1u:0u;
    if(cmd==COMM_DETECT_MOTOR_R_L){
        const float r=success?s_detect_all.r[mi]:0.0f;
        const float l_uH=success?s_detect_all.l[mi]*1.0e6f:0.0f;
        const float ld_uH=success?s_detect_all.ld_lq[mi]*1.0e6f:0.0f;
        buffer_append_float32(b,r,1e6f,&i);
        buffer_append_float32(b,l_uH,1e3f,&i);
        buffer_append_float32(b,ld_uH,1e3f,&i);
    }else{
        const float flux=success?s_detect_all.flux[mi]:0.0f;
        buffer_append_float32(b,flux,1e7f,&i);
        if(cmd==COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP){
            /* Upstream optionally estimates encoder parameters while coasting.
             * Steering-safe F103 commissioning deliberately does not add an
             * unbounded encoder travel phase to a flux-only command. */
            buffer_append_float32(b,-1.0f,1e6f,&i);
            buffer_append_float32(b,-1.0f,1e6f,&i);
            b[i++]=0u;
        }
    }
    s_detect_all.active=0u;
    s_detect_all.stage=DETECT_ALL_IDLE;
    s_detect_all.standalone_cmd=0u;
    uart_send_payload(b,(uint16_t)i);
}

static void conf_general_detect_worker_finish(int16_t result) {
    if(s_detect_all.standalone_cmd!=0u) standalone_measure_finish(result>=0);
    else detect_all_finish(result);
}

static bool mcpwm_foc_measure_res_ind_f103_start(bool second) {
    if(s_hall_detect.active || s_detect_all.active)return false;
    memset(&s_detect_all,0,sizeof(s_detect_all));
    s_detect_all.active=1u;
    s_detect_all.standalone_cmd=COMM_DETECT_MOTOR_R_L;
    s_detect_all.standalone_second=second?1u:0u;
    s_detect_all.max_power_loss=50.0f; /* Imax is not returned by command 25. */
    for(uint8_t mi=0u;mi<2u;++mi){
        s_detect_all.backup[mi]=*mc_interface_get_configuration_motor(mi!=0u);
        s_detect_all.result[mi]=s_detect_all.backup[mi];
        s_detect_all.result[mi].motor_type=MOTOR_TYPE_FOC;
    }
    app_vesc_disable_output(60000);
    measure_r_l_imax_f103_start(second?1u:0u,detect_time_now());
    return true;
}

static bool conf_general_measure_flux_linkage_f103_worker_start(bool second, COMM_PACKET_ID cmd,
                                          float current,float rpm_or_ramp,
                                          float resistance,float inductance) {
    if(s_hall_detect.active || s_detect_all.active)return false;
    if(!(resistance>0.0f && resistance<=2.0f))return false;
    if(cmd==COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP && !(inductance>0.0f && inductance<=0.1f))return false;
    memset(&s_detect_all,0,sizeof(s_detect_all));
    s_detect_all.active=1u;
    s_detect_all.standalone_cmd=(uint8_t)cmd;
    s_detect_all.standalone_second=second?1u:0u;
    if(cmd==COMM_DETECT_MOTOR_FLUX_LINKAGE){
        const mc_configuration *live=(const mc_configuration *)mc_interface_get_configuration_motor(second);
        inductance=(live && live->foc_motor_l>0.0f)?live->foc_motor_l:0.0f;
    }
    s_detect_all.standalone_flux_current=fabsf(current);
    s_detect_all.standalone_flux_ramp_erpm_s=(cmd==COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP)?fabsf(rpm_or_ramp):600.0f;
    for(uint8_t mi=0u;mi<2u;++mi){
        s_detect_all.backup[mi]=*mc_interface_get_configuration_motor(mi!=0u);
        s_detect_all.result[mi]=s_detect_all.backup[mi];
        s_detect_all.result[mi].motor_type=MOTOR_TYPE_FOC;
    }
    const uint8_t mi=second?1u:0u;
    s_detect_all.motor_index=mi;
    s_detect_all.r[mi]=resistance;
    s_detect_all.l[mi]=inductance;
    float fmax=s_detect_all.result[mi].l_current_max*s_detect_all.result[mi].l_current_max_scale;
    if(!(fmax>0.0f))fmax=s_detect_all.result[mi].l_current_max;
    if(fmax>(float)I_MOT_MAX)fmax=(float)I_MOT_MAX;
    s_detect_all.flux_current=fabsf(current);
    if(s_detect_all.flux_current<0.50f)s_detect_all.flux_current=0.50f;
    if(s_detect_all.flux_current>fmax)s_detect_all.flux_current=fmax;
    s_detect_all.flux_bounded_left=0u;
    if(!second){
        const mc_configuration *lc=&s_detect_all.backup[0];
        const bool abi=lc->m_sensor_port_mode==SENSOR_PORT_MODE_ABI &&
            (lc->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER || lc->foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB);
        const bool bounded_ready=abi && mc_interface_steering_calibration_valid() &&
            mcpwm_foc_steering_is_homed() && mcpwm_foc_encoder_is_synced(false) &&
            mcpwm_foc_get_position_min_user_counts(false)<mcpwm_foc_get_position_max_user_counts(false);
        if(bounded_ready){
            s_detect_all.flux_bounded_left=1u;
            s_detect_all.flux_base_phase_deg=mcpwm_foc_get_phase_motor(false);
            s_detect_all.flux_guard_min_counts=mcpwm_foc_get_position_min_user_counts(false);
            s_detect_all.flux_guard_max_counts=mcpwm_foc_get_position_max_user_counts(false);
            if(s_detect_all.flux_current<MCCONF_STEERING_DETECT_CURRENT_START_A)
                s_detect_all.flux_current=MCCONF_STEERING_DETECT_CURRENT_START_A;
            if(s_detect_all.flux_current>5.0f)s_detect_all.flux_current=5.0f;
            if(s_detect_all.flux_current>fmax)s_detect_all.flux_current=fmax;
            s_detect_all.flux_target_erpm=400.0f;
        }else{
            /* Preserve protocol compatibility for non-steering/host models. On
             * the deployed LEFT steering unit bounded_ready is mandatory and
             * true after encoder detect/home, so continuous rotation is never
             * selected there. */
            s_detect_all.flux_target_erpm=600.0f;
        }
    }else{
        s_detect_all.flux_target_erpm=600.0f;
        if(cmd==COMM_DETECT_MOTOR_FLUX_LINKAGE && rpm_or_ramp>600.0f && rpm_or_ramp<1200.0f)
            s_detect_all.flux_target_erpm=rpm_or_ramp;
    }
    if(s_detect_all.standalone_flux_ramp_erpm_s<50.0f)s_detect_all.standalone_flux_ramp_erpm_s=50.0f;
    if(s_detect_all.standalone_flux_ramp_erpm_s>20000.0f)s_detect_all.standalone_flux_ramp_erpm_s=20000.0f;
    detect_all_apply_runtime(mi);
    app_vesc_disable_output(60000);
    detect_all_reset_sample();
    s_detect_all.stage=DETECT_ALL_FLUX_RAMP;
    s_detect_all.stage_start_time=detect_time_now();
    s_detect_all.next_sample_time=s_detect_all.stage_start_time;
    return true;
}

static inline bool conf_general_measure_flux_linkage_start(bool second, float current,
                                                        float duty, float min_erpm,
                                                        float resistance) {
    (void)duty;
    /* COMM_DETECT_MOTOR_FLUX_LINKAGE (26) in upstream uses the legacy BLDC
     * sensorless commutator. This F103 firmware is FOC-only, so this wrapper is
     * a protocol-compatible adapter onto the FOC commissioning worker. */
    return conf_general_measure_flux_linkage_f103_worker_start(second,
        COMM_DETECT_MOTOR_FLUX_LINKAGE,current,min_erpm,resistance,0.0f);
}

static inline bool conf_general_measure_flux_linkage_openloop_start(bool second, float current,
                                                                 float duty, float erpm_per_sec,
                                                                 float resistance, float inductance) {
    (void)duty;
    /* Same command/method family as upstream conf_general_measure_flux_linkage_openloop;
     * cooperative suffix is required because this bare-metal target has no worker thread. */
    return conf_general_measure_flux_linkage_f103_worker_start(second,
        COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,current,erpm_per_sec,resistance,inductance);
}

static void measure_r_l_imax_f103_start(uint8_t mi, uint32_t now_time) {
    s_detect_all.motor_index=mi;
    detect_all_apply_runtime(mi);
    const bool second=mi!=0u;
    const float max_i=s_detect_all.result[mi].l_current_max>0.5f ? s_detect_all.result[mi].l_current_max : (float)I_MOT_MAX;
    /* R/L identification also starts at a useful 3-A Id level on this
     * low-side-shunt board. The old 0.6-A point was below reliable driven
     * current observability and produced lowI=0 even though the detector later
     * surfaced the generic flux-linkage error in VESC Tool. */
    float lo=MCCONF_STEERING_DETECT_CURRENT_START_A;
    float hi=lo+2.0f;
    if(lo>max_i*0.60f)lo=max_i*0.60f;
    if(hi>max_i*0.85f)hi=max_i*0.85f;
    if(hi>(float)I_MOT_MAX)hi=(float)I_MOT_MAX;
    if(lo<0.50f)lo=0.50f;
    if(hi<lo+0.50f)hi=lo+0.50f;
    if(hi>max_i)hi=max_i;
    s_detect_all.current_low=lo; s_detect_all.current_high=hi;
    mcpwm_foc_set_openloop_phase(lo,0.0f,second);
    mcpwm_foc_vesc_override_touch(second);
    s_detect_all.stage=DETECT_ALL_RL_ALIGN;
    s_detect_all.stage_start_time=now_time;
    s_detect_all.next_sample_time=now_time;
    detect_all_reset_sample();
}

static float detect_all_sensor_current(uint8_t mi) {
    float i=s_detect_all.imax[mi]/3.0f;
    if(!(i>0.0f))i=MCCONF_STEERING_DETECT_CURRENT_START_A;
    if(i<0.10f)i=0.10f;
    if(i>(float)I_MOT_MAX)i=(float)I_MOT_MAX;
    return i;
}

static void conf_general_autodetect_apply_sensors_foc_start(uint32_t now_time) {
    detect_all_release_all();
    s_detect_all.motor_index=0u;
    s_detect_all.stage_start_time=now_time;
    s_detect_all.next_sample_time=now_time;
    detect_all_apply_runtime(0u);
    if(s_detect_all.left_sensor_encoder){
        s_detect_all.stage=DETECT_ALL_ENCODER;
    }else{
        s_detect_all.stage=DETECT_ALL_HALL;
        mcpwm_foc_hall_detect_start(false,detect_all_sensor_current(0u));
    }
}

static void mcpwm_foc_hall_detect_finish(bool success) {
    uint8_t table[8]; uint8_t fails=0u;
    for(uint8_t h=0u;h<8u;++h){const uint8_t a=mcpwm_foc_hall_detect_angle200(s_hall_detect.sum_s[h],s_hall_detect.sum_c[h],s_hall_detect.samples[h]);table[h]=a;if(a==255u)fails++;}
    if(fails!=2u)success=false;
    const bool second=s_hall_detect.second!=0u;
    const uint8_t restore_backup=s_hall_detect.restore_backup;
    const mc_configuration backup=s_hall_detect.backup;
    mc_interface_select_motor_thread(second?2:1); mc_interface_release_motor(); mcpwm_foc_vesc_override_clear(second); mc_interface_select_motor_thread(1);
    if(s_detect_all.active){
        if(!success){s_detect_all_last_detail=second?10:11;detect_all_finish(detect_all_fault_result(second));return;}
        const uint8_t mi=second?1u:0u; memcpy(s_detect_all.hall[mi],table,8u); detect_all_prepare_hall(mi); detect_all_apply_runtime(mi);
        memset(&s_hall_detect,0,sizeof(s_hall_detect));
        if(!second){s_detect_all.stage=DETECT_ALL_HALL;s_detect_all.motor_index=1u;mcpwm_foc_hall_detect_start(true,detect_all_sensor_current(1u));}
        else detect_all_finish(2);
        return;
    }
    uint8_t reply[10]; reply[0]=COMM_DETECT_HALL_FOC; memcpy(&reply[1],table,8u); reply[9]=success?0u:1u;
    s_last_hall_store_ok[second?1u:0u]=0u; /* standalone detect is not a store */
    if(restore_backup){
        mc_interface_select_motor_thread(second?2:1);
        mc_configuration c=backup;
        mc_interface_set_configuration(&c);
        mc_interface_select_motor_thread(1);
    }
    memset(&s_hall_detect,0,sizeof(s_hall_detect)); uart_send_payload(reply,sizeof(reply));
}

static void mcpwm_foc_hall_detect_command_start(bool second, const uint8_t *data, uint16_t len) {
    if (s_hall_detect.active || s_detect_all.active) return;
    if (len < 4u) {
        uint8_t reply[10] = {COMM_DETECT_HALL_FOC,255,255,255,255,255,255,255,255,1};
        uart_send_payload(reply, sizeof(reply));
        return;
    }

    mc_interface_select_motor_thread(second?2:1);
    const mc_configuration *live=(const mc_configuration *)mc_interface_get_configuration_motor(second);
    if(!live || live->m_sensor_port_mode!=SENSOR_PORT_MODE_HALL){
        mc_interface_select_motor_thread(1);
        uint8_t reply[10] = {COMM_DETECT_HALL_FOC,255,255,255,255,255,255,255,255,1};
        uart_send_payload(reply,sizeof(reply));
        return;
    }

    mc_configuration backup=*live;
    mc_configuration temp=backup;
    /* Upstream COMM_DETECT_HALL_FOC wrapper uses a temporary FOC setup, runs
     * mcpwm_foc_hall_detect(), then restores the complete previous MC config. */
    temp.motor_type=MOTOR_TYPE_FOC;
    /* Timer F103 fixed 16 kHz; Hall detect tidak boleh membuat MC config
     * sementara mengaku berjalan pada switching frequency lain. */
    temp.foc_f_zv=(float)PWM_FREQ;
    temp.foc_current_kp=0.01f;
    temp.foc_current_ki=10.0f;
    mc_interface_set_configuration(&temp);
    mc_interface_select_motor_thread(1);

    int32_t ind = 0;
    float current = (float)buffer_get_int32(data, &ind) / 1000.0f;
    mcpwm_foc_hall_detect_start(second,current);
    s_hall_detect.backup=backup;
    s_hall_detect.restore_backup=1u;
}

static void conf_general_detect_apply_all_foc_can_start(const uint8_t *data,uint16_t len) {
    if(s_hall_detect.active || s_detect_all.active) return;
    if(len<21u){ detect_all_reply(-1); return; }
    int32_t k=0;
    const uint8_t detect_can=data[k++]; (void)detect_can;
    const float max_power_loss=buffer_get_float32(data,1e3f,&k);
    const float min_current_in=buffer_get_float32(data,1e3f,&k);
    const float max_current_in=buffer_get_float32(data,1e3f,&k);
    const float openloop_rpm=buffer_get_float32(data,1e3f,&k);
    const float sl_erpm=buffer_get_float32(data,1e3f,&k);
    if(!(max_power_loss>=0.5f && max_power_loss<=5000.0f)){detect_all_reply(-1);return;}
    memset(&s_detect_all,0,sizeof(s_detect_all));
    s_detect_all_last_detail=0; s_detect_all.active=1u;
    s_detect_all.max_power_loss=max_power_loss; s_detect_all.min_current_in=min_current_in;
    s_detect_all.max_current_in=max_current_in; s_detect_all.openloop_rpm=openloop_rpm; s_detect_all.sl_erpm=sl_erpm;
    for(uint8_t mi=0u;mi<2u;++mi){
        s_detect_all.backup[mi]=*mc_interface_get_configuration_motor(mi!=0u);
        s_detect_all.result[mi]=s_detect_all.backup[mi];
        detect_all_apply_common_limits(&s_detect_all.result[mi]);
        s_detect_all.result[mi].motor_type=MOTOR_TYPE_FOC;
        /* Model identification itself is sensor-independent because the
         * commissioning control modes override electrical phase explicitly.
         * Keep the user's physical/FOC sensor selection intact here. This is
         * safer on the steering LEFT endpoint than exposing a temporary
         * SENSORLESS configuration that is valid only while OPENLOOP is active. */
    }
    s_detect_all.left_sensor_encoder=(s_detect_all.backup[0].m_sensor_port_mode==SENSOR_PORT_MODE_ABI &&
        (s_detect_all.backup[0].foc_sensor_mode==FOC_SENSOR_MODE_ENCODER ||
         s_detect_all.backup[0].foc_sensor_mode==FOC_SENSOR_MODE_ENCODER_AB))?1u:0u;
    app_vesc_disable_output(180000);
    detect_all_release_all();
    /* Upstream VESC Detect-All order: motor model first (R/L/Imax -> flux),
     * then sensor commissioning. Rotor sensors are not used for these model
     * measurements. */
    measure_r_l_imax_f103_start(0u,detect_time_now());
}

static void mcpwm_foc_hall_detect_process(uint32_t now_time) {
    if (!s_hall_detect.active) return;
    if (!detect_time_due(now_time, s_hall_detect.next_time)) return;
    const bool second = s_hall_detect.second != 0u;
    mcpwm_foc_motor_t *m = mcpwm_foc_get_motor(second);
    mcpwm_foc_vesc_override_touch(second);
    if (m->m_fault != FAULT_CODE_NONE) { mcpwm_foc_hall_detect_finish(false); return; }

    if (s_hall_detect.stage == HALL_DETECT_ALIGN) {
        /* The upstream detector ramps alignment current for about one second.
         * On this bare-metal target the main loop is preempted by the 16-kHz FOC
         * ISR, so counting 1000 scheduler visits can stretch one second into
         * tens of seconds (especially on motor 2). Drive the ramp from wall time
         * instead; skipped visits simply advance to the correct current. */
        const uint32_t elapsed_ms=detect_time_elapsed_ms(s_hall_detect.align_start_time, now_time);
        if (elapsed_ms < 1000u) {
            const uint16_t step=(uint16_t)(elapsed_ms+1u);
            s_hall_detect.align_step=step;
            const float i=s_hall_detect.current_a*(float)step/1000.0f;
            mcpwm_foc_set_openloop_phase(i,0.0f,second);
            mcpwm_foc_vesc_override_touch(second);
            s_hall_detect.next_time=detect_time_after_ms(now_time,1u);
            return;
        }
        s_hall_detect.align_step=1000u;
        mcpwm_foc_set_openloop_phase(s_hall_detect.current_a,0.0f,second);
        s_hall_detect.stage = HALL_DETECT_SWEEP;
        s_hall_detect.pass = 0u;
        s_hall_detect.degree = 0;
        s_hall_detect.waiting_sample = 0u;
    }

    if (s_hall_detect.stage != HALL_DETECT_SWEEP) return;
    if (!s_hall_detect.waiting_sample) {
        mcpwm_foc_set_openloop_phase(s_hall_detect.current_a, (float)s_hall_detect.degree, second);
        mcpwm_foc_vesc_override_touch(second);
        s_hall_detect.waiting_sample = 1u;
        s_hall_detect.next_time = detect_time_after_ms(now_time,5u);
        return;
    }

    /* Upstream utils_read_hall() samples the physical Hall code with its
     * configured majority filter. Use the equivalent majority state here, not
     * the closed-loop/debounced Hall estimator state. */
    const uint8_t h = m->m_hall_filtered_state & 7u;
    const int32_t dnorm = (s_hall_detect.degree >= 360) ? 0 : s_hall_detect.degree;
    /* F103 adaptation: keep the upstream circular-mean method, but source
     * sine/cosine from the existing Q15 LUT. This avoids pulling full sinf/cosf
     * into the 120-KiB application image; finalization still uses atan2f and
     * VESC truncate semantics, so this is not the old nearest-bin best_dot path. */
    int16_t sn,cs;
    const uint16_t ph=(uint16_t)(((uint32_t)dnorm*65536u)/360u);
    foc_sin_cos_q15(ph,&sn,&cs);
    s_hall_detect.sum_s[h] += sn;
    s_hall_detect.sum_c[h] += cs;
    if (s_hall_detect.samples[h] < 0xffffu) s_hall_detect.samples[h]++;
    s_hall_detect.waiting_sample = 0u;

    if (s_hall_detect.pass < 3u) {
        if (s_hall_detect.degree >= 359) {
            s_hall_detect.pass++;
            s_hall_detect.degree = (s_hall_detect.pass < 3u) ? 0 : 360;
        } else {
            s_hall_detect.degree++;
        }
    } else {
        if (s_hall_detect.degree <= 0) {
            s_hall_detect.pass++;
            if (s_hall_detect.pass >= 6u) {
                mcpwm_foc_hall_detect_finish(true);
                return;
            }
            s_hall_detect.degree = 360;
        } else {
            s_hall_detect.degree--;
        }
    }

    /* Upstream blocking mcpwm_foc_hall_detect() sets the next 1-degree phase
     * immediately after sampling the previous one, then sleeps 5 ms. Mirror
     * that exactly in the cooperative worker: do not spend a second main-loop
     * visit just to arm the next phase, which doubles sweep time under the
     * high-load 16-kHz FOC ISR. */
    mcpwm_foc_set_openloop_phase(s_hall_detect.current_a,
                                 (float)s_hall_detect.degree, second);
    mcpwm_foc_vesc_override_touch(second);
    s_hall_detect.waiting_sample = 1u;
    s_hall_detect.next_time = detect_time_after_ms(now_time,5u);
}

static bool measure_r_l_imax_f103_finish(uint8_t mi) {
    const float di_phys=(float)s_detect_all.high_i[mi]-(float)s_detect_all.low_i[mi];
    const float dv_phys=(float)s_detect_all.high_v[mi]-(float)s_detect_all.low_v[mi];
    const float di_raw=(float)s_detect_all.high_i_raw[mi]-(float)s_detect_all.low_i_raw[mi];
    const float dv_raw=(float)s_detect_all.high_v_raw[mi]-(float)s_detect_all.low_v_raw[mi];
    if(fabsf(di_phys)<0.25f || fabsf(di_raw)<100.0f || fabsf(dv_raw)<20.0f) return false;
    const float r=fabsf(dv_phys/di_phys);
    if(!(r>=0.005f && r<=2.0f)) return false;

    mcpwm_foc_rl_capture_t cap;
    mcpwm_foc_rl_capture_get(mi!=0u,&cap);
    if(cap.samples<20u || cap.sum_di2==0) return false;
    const float a=dv_raw/di_raw;
    const float c=(float)s_detect_all.low_v_raw[mi]-a*(float)s_detect_all.low_i_raw[mi];
    const float b=((float)cap.sum_div-a*(float)cap.sum_dii-c*(float)cap.sum_di)/(float)cap.sum_di2;
    const float vscale=fabsf(dv_phys/dv_raw);
    const float dt=(float)MCCONF_FOC_CONTROL_DIV/(float)PWM_FREQ;
    const float l=fabsf(b)*vscale*(float)FOC_CURRENT_Q4_PER_A*dt;
    if(!(l>=0.000005f && l<=0.020f)) return false;

    float im=foc_sqrtf_slow((float)s_detect_all.max_power_loss/(r*1.5f));
    if(im<1.0f)im=1.0f;
    if(im>(float)I_MOT_MAX)im=(float)I_MOT_MAX;
    s_detect_all.r[mi]=(float)r;
    s_detect_all.l[mi]=(float)l;
    s_detect_all.ld_lq[mi]=0.0f; /* safe d-axis identification; non-salient default */
    s_detect_all.imax[mi]=(float)im;
    return true;
}

/* Same equations and function name as upstream conf_general.c. This helper is
 * commissioning/main-loop code, so float math is intentional and never enters
 * the 16-kHz ADC ISR. */
static inline void conf_general_calc_apply_foc_cc_kp_ki_gain(mc_configuration *mcconf, float tc) {
    if(!mcconf || !(tc>0.0f)) return;
    const float bw=1.0f/(tc*1.0e-6f);
    mcconf->foc_current_kp=mcconf->foc_motor_l*bw;
    mcconf->foc_current_ki=mcconf->foc_motor_r*bw;
    if(mcconf->foc_motor_flux_linkage>0.000001f)
        mcconf->foc_observer_gain=1000.0f/(mcconf->foc_motor_flux_linkage*mcconf->foc_motor_flux_linkage);
}

static void conf_general_detect_apply_all_foc_finalize_motor(uint8_t mi) {
    mc_configuration *c=&s_detect_all.result[mi];
    c->foc_motor_r=s_detect_all.r[mi];
    c->foc_motor_l=s_detect_all.l[mi];
    c->foc_motor_ld_lq_diff=s_detect_all.ld_lq[mi];
    c->foc_motor_flux_linkage=s_detect_all.flux[mi];
    conf_general_calc_apply_foc_cc_kp_ki_gain(c,1000.0f);
    c->l_current_max=s_detect_all.imax[mi];
    c->l_current_min=-s_detect_all.imax[mi];
    float absmax=s_detect_all.imax[mi]*1.5f;
    if(absmax>MCCONF_L_ABS_CURRENT_MAX)absmax=MCCONF_L_ABS_CURRENT_MAX;
    if(absmax<s_detect_all.imax[mi])absmax=s_detect_all.imax[mi];
    c->l_abs_current_max=absmax;
    c->motor_type=MOTOR_TYPE_FOC;
    /* Do not overwrite sensor selection while finalizing R/L/flux. The next
     * commissioning stage explicitly applies Encoder or Hall. */
}

static void conf_general_detect_apply_all_foc_process(uint32_t now_time) {
    if(!s_detect_all.active || s_detect_all.stage==DETECT_ALL_IDLE) return;
    if(s_detect_all.stage==DETECT_ALL_ENCODER){
        /* Encoder detect is intentionally blocking like upstream VESC's motor
         * detection worker, while the bridge remains output-gated. */
        if(!detect_all_prepare_encoder_left()){
            s_detect_all_last_detail=9;
            conf_general_detect_worker_finish(-10);
            return;
        }
        detect_all_apply_runtime(0u);
        s_detect_all.stage=DETECT_ALL_HALL;
        s_detect_all.motor_index=1u;
        /* The RIGHT traction motor needs enough d-axis alignment torque for
         * all six Hall sectors to cross cleanly. 1 A reproducibly left missing
         * / compressed sectors on the real motor; 2 A gives a sane six-state
         * table with no fault or over-current trip. mcpwm_foc_hall_detect_start()
         * still clamps this request to the configured motor-current limit. */
        mcpwm_foc_hall_detect_start(true,detect_all_sensor_current(1u));
        return;
    }
    if(s_detect_all.stage==DETECT_ALL_HALL) return;
    if(!detect_time_due(now_time,s_detect_all.next_sample_time)) return;
    s_detect_all.next_sample_time=detect_time_after_ms(now_time,1u);
    const uint8_t mi=s_detect_all.motor_index;
    const bool second=mi!=0u;
    mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
    const mc_fault_code fault=mc_interface_get_fault_motor(second);
    if(fault!=FAULT_CODE_NONE){conf_general_detect_worker_finish((int16_t)((int)fault-100));return;}
    mcpwm_foc_vesc_override_touch(second);
    const uint32_t elapsed=detect_time_elapsed_ms(s_detect_all.stage_start_time,now_time);

    switch(s_detect_all.stage){
    case DETECT_ALL_RL_ALIGN:
        if(elapsed>=400u){
            detect_all_reset_sample();
            s_detect_all.stage=DETECT_ALL_RL_LOW;
            s_detect_all.stage_start_time=now_time;
        }
        break;
    case DETECT_ALL_RL_LOW:
        /* Commissioning must use the regulator-owned instantaneous current.
         * GET_VALUES telemetry is deliberately filtered/qualified and can be
         * zero while the powered-offset path is settling, which made a real
         * 3 A locked-rotor Id measurement fail as low_i=0. */
        s_detect_all.sum_i+=(float)m->m_id_q4/(float)FOC_CURRENT_Q4_PER_A;
        s_detect_all.sum_v+=mcpwm_foc_get_vd_motor(second);
        s_detect_all.sum_i_raw+=m->m_id_q4;
        s_detect_all.sum_v_raw+=m->m_vd;
        s_detect_all.sample_n++;
        if(elapsed>=250u && s_detect_all.sample_n>=20u){
            const float n=(float)s_detect_all.sample_n;
            s_detect_all.low_i[mi]=(float)(s_detect_all.sum_i/n);
            s_detect_all.low_v[mi]=(float)(s_detect_all.sum_v/n);
            s_detect_all.low_i_raw[mi]=(float)(s_detect_all.sum_i_raw/n);
            s_detect_all.low_v_raw[mi]=(float)(s_detect_all.sum_v_raw/n);
            if(fabsf(s_detect_all.low_i[mi])<0.20f){s_detect_all_last_detail=2;conf_general_detect_worker_finish(-10);return;}
            mcpwm_foc_rl_capture_start(second);
            mcpwm_foc_set_openloop_phase(s_detect_all.current_high,0.0f,second);
            mcpwm_foc_vesc_override_touch(second);
            s_detect_all.stage=DETECT_ALL_RL_STEP;
            s_detect_all.stage_start_time=now_time;
        } else if(elapsed>=2000u && s_detect_all.sample_n<20u){
            s_detect_all_last_detail=1; conf_general_detect_worker_finish(-10); return;
        }
        break;
    case DETECT_ALL_RL_STEP: {
        /* L comes from the dI transient, so do not integrate hundreds of ms of
         * steady-state ADC noise after a 3->5 A step has already settled. At
         * 16 kHz/DIV6, 32 regulator samples are about 12 ms and retain the
         * informative transient. Keep a 100-ms/20-sample fallback for a busy
         * main loop, then fail closed rather than fitting noise. */
        mcpwm_foc_rl_capture_t live_cap;
        mcpwm_foc_rl_capture_get(second,&live_cap);
        const bool enough=(live_cap.samples>=32u) ||
                          (elapsed>=100u && live_cap.samples>=20u);
        if(enough){
            mcpwm_foc_rl_capture_stop(second);
            detect_all_reset_sample();
            s_detect_all.stage=DETECT_ALL_RL_HIGH;
            s_detect_all.stage_start_time=now_time;
        }else if(elapsed>=500u){
            s_detect_all_last_detail=12; conf_general_detect_worker_finish(-10); return;
        }
        break;
    }
    case DETECT_ALL_RL_HIGH:
        /* Commissioning must use the regulator-owned instantaneous current.
         * GET_VALUES telemetry is deliberately filtered/qualified and can be
         * zero while the powered-offset path is settling, which made a real
         * 3 A locked-rotor Id measurement fail as low_i=0. */
        s_detect_all.sum_i+=(float)m->m_id_q4/(float)FOC_CURRENT_Q4_PER_A;
        s_detect_all.sum_v+=mcpwm_foc_get_vd_motor(second);
        s_detect_all.sum_i_raw+=m->m_id_q4;
        s_detect_all.sum_v_raw+=m->m_vd;
        s_detect_all.sample_n++;
        if(elapsed>=250u && s_detect_all.sample_n>=20u){
            const float n=(float)s_detect_all.sample_n;
            s_detect_all.high_i[mi]=(float)(s_detect_all.sum_i/n);
            s_detect_all.high_v[mi]=(float)(s_detect_all.sum_v/n);
            s_detect_all.high_i_raw[mi]=(float)(s_detect_all.sum_i_raw/n);
            s_detect_all.high_v_raw[mi]=(float)(s_detect_all.sum_v_raw/n);
            if(!measure_r_l_imax_f103_finish(mi)){s_detect_all_last_detail=4;conf_general_detect_worker_finish(-10);return;}
            if(s_detect_all.standalone_cmd==COMM_DETECT_MOTOR_R_L){
                conf_general_detect_worker_finish(0);
                return;
            }
            /* Match VESC Detect-All: flux open-loop current is Imax / 2.5.
             * Keep only the board safety clamps around that upstream choice. */
            s_detect_all.flux_current=s_detect_all.standalone_cmd ?
                s_detect_all.standalone_flux_current : s_detect_all.imax[mi]/2.5f;
            {
                float fmax=s_detect_all.result[mi].l_current_max*s_detect_all.result[mi].l_current_max_scale;
                if(!(fmax>0.0f))fmax=s_detect_all.result[mi].l_current_max;
                if(fmax>(float)I_MOT_MAX)fmax=(float)I_MOT_MAX;
                if(s_detect_all.flux_current>fmax)s_detect_all.flux_current=fmax;
            }
            if(s_detect_all.flux_current<0.50f)s_detect_all.flux_current=0.50f;
            s_detect_all.flux_target_erpm=600.0f;
            detect_all_reset_sample();
            s_detect_all.stage=DETECT_ALL_FLUX_RAMP;
            s_detect_all.stage_start_time=now_time;
        } else if(elapsed>=2000u && s_detect_all.sample_n<20u){
            s_detect_all_last_detail=3; conf_general_detect_worker_finish(-10); return;
        }
        break;
    case DETECT_ALL_FLUX_RAMP: {
        if(s_detect_all.flux_bounded_left && !second){
            const int32_t pos=mcpwm_foc_get_position_user_counts(false);
            const int32_t margin=128;
            if(pos<=s_detect_all.flux_guard_min_counts+margin || pos>=s_detect_all.flux_guard_max_counts-margin){
                s_detect_all_last_detail=12; conf_general_detect_worker_finish(-10); return;
            }
            const float ramp_ms=300.0f;
            float f=(float)elapsed/ramp_ms; if(f>1.0f)f=1.0f;
            float ph=s_detect_all.flux_base_phase_deg-60.0f*f;
            while(ph<0.0f)ph+=360.0f;
            while(ph>=360.0f)ph-=360.0f;
            float ia=s_detect_all.flux_current*f; if(ia<0.50f)ia=0.50f;
            mcpwm_foc_set_openloop_phase(ia,ph,false);
            mcpwm_foc_vesc_override_touch(false);
            if((float)elapsed>=ramp_ms){
                detect_all_reset_sample();
                s_detect_all.stage=DETECT_ALL_FLUX_SAMPLE;
                s_detect_all.stage_start_time=now_time;
            }
            break;
        }
        float ramp_rate=s_detect_all.standalone_cmd ? s_detect_all.standalone_flux_ramp_erpm_s : 1800.0f;
        if(ramp_rate<50.0f)ramp_rate=50.0f;
        float ramp_ms=(s_detect_all.flux_target_erpm-80.0f)*1000.0f/ramp_rate;
        if(ramp_ms<100.0f)ramp_ms=100.0f;
        if(ramp_ms>5000.0f)ramp_ms=5000.0f;
        float f=(float)elapsed/ramp_ms; if(f>1.0f)f=1.0f;
        const float erpm=80.0f+(s_detect_all.flux_target_erpm-80.0f)*f;
        mcpwm_foc_set_openloop_current(s_detect_all.flux_current,erpm,second);
        mcpwm_foc_vesc_override_touch(second);
        if((float)elapsed>=ramp_ms){
            const float actual_erpm=fabsf(mcpwm_foc_get_erpm_motor(second));
            float fmax=s_detect_all.result[mi].l_current_max*s_detect_all.result[mi].l_current_max_scale;
            if(!(fmax>0.0f))fmax=s_detect_all.result[mi].l_current_max;
            if(fmax>(float)I_MOT_MAX)fmax=(float)I_MOT_MAX;
            /* If the rotor still has not broken away, increase torque current in
             * 1-A steps and retry the speed ramp. Stop increasing immediately
             * once motion is established. */
            if(actual_erpm<s_detect_all.flux_target_erpm*0.20f &&
               s_detect_all.flux_current<fmax-0.01f){
                s_detect_all.flux_current+=MCCONF_STEERING_DETECT_CURRENT_STEP_A;
                if(s_detect_all.flux_current>fmax)s_detect_all.flux_current=fmax;
                s_detect_all.stage_start_time=now_time;
                break;
            }
            detect_all_reset_sample();
            s_detect_all.stage=DETECT_ALL_FLUX_SAMPLE;
            s_detect_all.stage_start_time=now_time;
        }
        break;
    }
    case DETECT_ALL_FLUX_SAMPLE: {
        bool take_sample=true;
        uint32_t bounded_done_ms=0u;
        if(s_detect_all.flux_bounded_left && !second){
            const int32_t pos=mcpwm_foc_get_position_user_counts(false);
            const int32_t margin=128;
            if(pos<=s_detect_all.flux_guard_min_counts+margin || pos>=s_detect_all.flux_guard_max_counts-margin){
                s_detect_all_last_detail=12; conf_general_detect_worker_finish(-10); return;
            }
            uint32_t leg_ms=(uint32_t)(20000.0f/s_detect_all.flux_target_erpm+0.5f);
            if(leg_ms<40u)leg_ms=40u;
            if(leg_ms>250u)leg_ms=250u;
            const uint32_t cyc=leg_ms*2u;
            bounded_done_ms=cyc*10u; /* exactly ten symmetric triangle cycles */
            const uint32_t ce=elapsed%cyc;
            const bool forward=ce<leg_ms;
            const uint32_t le=forward?ce:(ce-leg_ms);
            const float ff=(float)le/(float)leg_ms;
            const float off=forward?(-60.0f+120.0f*ff):(60.0f-120.0f*ff);
            float ph=s_detect_all.flux_base_phase_deg+off;
            while(ph<0.0f)ph+=360.0f;
            while(ph>=360.0f)ph-=360.0f;
            mcpwm_foc_set_openloop_phase(s_detect_all.flux_current,ph,false);
            s_detect_all.flux_return_phase_deg=ph;
            mcpwm_foc_vesc_override_touch(false);
            if(le<8u || le+8u>=leg_ms)take_sample=false;
            const float ae=fabsf(mcpwm_foc_get_erpm_motor(false));
            if(ae<s_detect_all.flux_target_erpm*0.35f || ae>s_detect_all.flux_target_erpm*1.80f)take_sample=false;
        }else{
            mcpwm_foc_set_openloop_current(s_detect_all.flux_current,s_detect_all.flux_target_erpm,second);
            mcpwm_foc_vesc_override_touch(second);
        }
        /* Flux identification is commissioning, not user telemetry. Sample the
         * direct FOC state so telemetry LPF/GET_VALUES consumption cannot bias
         * |Idq| toward zero while the bridge is intentionally energized. */
        const float id=(float)m->m_id_q4/(float)FOC_CURRENT_Q4_PER_A;
        const float iq=(float)m->m_iq_q4/(float)FOC_CURRENT_Q4_PER_A;
        const float vd=mcpwm_foc_get_vd_motor(second);
        const float vq=mcpwm_foc_get_vq_motor(second);
        if(take_sample){
            s_detect_all.sum_i+=foc_sqrtf_slow(id*id+iq*iq);
            s_detect_all.sum_v+=foc_sqrtf_slow(vd*vd+vq*vq);
            s_detect_all.sum_erpm+=fabsf(mcpwm_foc_get_erpm_motor(second));
            s_detect_all.sample_n++;
        }
        const uint32_t min_elapsed=s_detect_all.flux_bounded_left?bounded_done_ms:800u;
        const uint32_t max_elapsed=s_detect_all.flux_bounded_left?(bounded_done_ms+800u):3000u;
        const uint32_t min_samples=s_detect_all.flux_bounded_left?200u:40u;
        if(elapsed>=min_elapsed && s_detect_all.sample_n>=min_samples){
            const float n=(float)s_detect_all.sample_n;
            const float i_mag=s_detect_all.sum_i/n;
            const float v_mag=s_detect_all.sum_v/n;
            const float erpm=s_detect_all.sum_erpm/n;
            if(erpm<s_detect_all.flux_target_erpm*0.50f ||
               erpm>s_detect_all.flux_target_erpm*1.60f){s_detect_all_last_detail=6;conf_general_detect_worker_finish(-10);return;}
            const float omega=erpm*6.28318530717958647692f/60.0f;
            const float bemf=v_mag-s_detect_all.r[mi]*i_mag;
            if(bemf<=0.02f || omega<=1.0f){s_detect_all_last_detail=7;conf_general_detect_worker_finish(-10);return;}
            const float ldrop=s_detect_all.l[mi]*i_mag;
            const float flux=bemf/omega-ldrop;
            if(!(flux>=0.0001f && flux<=1.0f)){s_detect_all_last_detail=8;conf_general_detect_worker_finish(-10);return;}
            s_detect_all.flux[mi]=flux;
            if(s_detect_all.flux_bounded_left && !second){
                /* Return to the captured electrical phase before release. This
                 * removes the cumulative steering drift that otherwise occurs
                 * when a triangle measurement ends at an arbitrary phase. */
                s_detect_all.flux_return_phase_deg=mcpwm_foc_get_phase_motor(false);
                s_detect_all.stage=DETECT_ALL_FLUX_RETURN;
                s_detect_all.stage_start_time=now_time;
                break;
            }
            if(s_detect_all.standalone_cmd){
                conf_general_detect_worker_finish(0);
                return;
            }
            conf_general_detect_apply_all_foc_finalize_motor(mi);
            mc_interface_select_motor_thread(second?2:1); mc_interface_release_motor();
            mcpwm_foc_vesc_override_clear(second); mc_interface_select_motor_thread(1);
            if(mi==0u){measure_r_l_imax_f103_start(1u,now_time);}
            else conf_general_autodetect_apply_sensors_foc_start(now_time);
        } else if(elapsed>=max_elapsed && s_detect_all.sample_n<min_samples){
            s_detect_all_last_detail=s_detect_all.flux_bounded_left?13:5;
            conf_general_detect_worker_finish(-10); return;
        }
        break;
    }
    case DETECT_ALL_FLUX_RETURN: {
        if(!s_detect_all.flux_bounded_left || second){s_detect_all_last_detail=14;conf_general_detect_worker_finish(-10);return;}
        const int32_t pos=mcpwm_foc_get_position_user_counts(false);
        const int32_t margin=128;
        if(pos<=s_detect_all.flux_guard_min_counts+margin || pos>=s_detect_all.flux_guard_max_counts-margin){
            s_detect_all_last_detail=12; conf_general_detect_worker_finish(-10); return;
        }
        const float return_ms=300.0f;
        float f=(float)elapsed/return_ms; if(f>1.0f)f=1.0f;
        float d=s_detect_all.flux_base_phase_deg-s_detect_all.flux_return_phase_deg;
        while(d>180.0f)d-=360.0f;
        while(d<-180.0f)d+=360.0f;
        float ph=s_detect_all.flux_return_phase_deg+d*f;
        while(ph<0.0f)ph+=360.0f;
        while(ph>=360.0f)ph-=360.0f;
        float ia=s_detect_all.flux_current*(1.0f-0.75f*f);
        if(ia<0.50f)ia=0.50f;
        mcpwm_foc_set_openloop_phase(ia,ph,false);
        mcpwm_foc_vesc_override_touch(false);
        if((float)elapsed>=return_ms){
            if(s_detect_all.standalone_cmd){conf_general_detect_worker_finish(0);return;}
            conf_general_detect_apply_all_foc_finalize_motor(mi);
            mc_interface_select_motor_thread(1); mc_interface_release_motor();
            mcpwm_foc_vesc_override_clear(false); mc_interface_select_motor_thread(1);
            measure_r_l_imax_f103_start(1u,now_time);
        }
        break;
    }
    default:
        break;
    }
}

static int32_t q4_to_milliamps_normalized(int16_t q4, bool second) {
    int32_t ma = ((int32_t)q4 * 1000) / ((int32_t)A2BIT_CONV * 16);
    return second ? -ma : ma;
}

static void reply_custom_pos_state(bool second, uint8_t op, uint8_t status) {
    uint8_t b[32];
    int32_t i = 0;
    b[i++] = COMM_CUSTOM_APP_DATA;
    b[i++] = HB_CUSTOM_MAGIC0;
    b[i++] = HB_CUSTOM_MAGIC1;
    b[i++] = HB_CUSTOM_VERSION;
    b[i++] = op;
    b[i++] = status;
    buffer_append_int32(b, mcpwm_foc_get_position_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_target_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_min_user_counts(second), &i);
    buffer_append_int32(b, mcpwm_foc_get_position_max_user_counts(second), &i);
    uart_send_payload(b, (uint16_t)i);
}

static void process_custom_app(bool second, const uint8_t *data, uint16_t len) {
    if (!data || len < 4u || data[0] != HB_CUSTOM_MAGIC0 ||
        data[1] != HB_CUSTOM_MAGIC1 || data[2] != HB_CUSTOM_VERSION) {
        return;
    }
    const uint8_t op = data[3];
    const uint8_t *d = data + 4;
    const uint16_t n = (uint16_t)(len - 4u);
    int32_t k = 0;

    if (op == HB_CUSTOM_GET_POS_STATE) {
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_SET_POS_LIMITS) {
        if (n < 8u) { reply_custom_pos_state(second, op, 1u); return; }
        const int32_t minc = buffer_get_int32(d, &k);
        const int32_t maxc = buffer_get_int32(d, &k);
        if (minc > maxc) { reply_custom_pos_state(second, op, 2u); return; }
        mcpwm_foc_set_position_user_limits(minc, maxc, second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_SET_POS_TARGET) {
        if (n < 4u) { reply_custom_pos_state(second, op, 1u); return; }
        const int32_t target = buffer_get_int32(d, &k);
        touch_motor(second);
        mcpwm_foc_set_position_user_counts(target, second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_RESET_POSITION) {
        mcpwm_foc_reset_position(second);
        reply_custom_pos_state(second, op, 0u);
        return;
    }
    if (op == HB_CUSTOM_GET_STEERING_CAL) {
        uint8_t b[48]; int32_t j=0; uint8_t flags=0u;
        if(mc_interface_steering_calibration_valid())flags|=0x01u;
        if(mcpwm_foc_steering_is_homed())flags|=0x02u;
        if(mcpwm_foc_encoder_is_synced(false))flags|=0x04u;
        if(mc_interface_steering_logical_inverted())flags|=0x08u;
        const int32_t sp=mcpwm_foc_steering_span_counts();
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u; b[j++]=flags;
        buffer_append_int32(b,sp,&j);
        buffer_append_int32(b,mcpwm_foc_get_position_user_counts(false),&j);
        buffer_append_int32(b,mcpwm_foc_get_position_target_user_counts(false),&j);
        buffer_append_int32(b,(int32_t)lroundf(mc_interface_get_steering_deg()*1000.0f),&j);
        /* Project diagnostic extension after the stable steering prefix. */
        b[j++]=(uint8_t)mcpwm_foc_get_motor_const(false)->m_conf.m_sensor_port_mode;
        b[j++]=(uint8_t)mcpwm_foc_get_motor_const(false)->m_conf.foc_sensor_mode;
        b[j++]=mcpwm_foc_get_motor_const(false)->m_encoder_configured?1u:0u;
        b[j++]=(uint8_t)mcpwm_foc_get_motor_const(false)->m_fault;
        buffer_append_uint32(b,mcpwm_foc_get_motor_const(false)->m_encoder_raw_count,&j);
        buffer_append_int32(b,mcpwm_foc_steering_safe_span_counts(),&j);
        float pos360=(mc_interface_get_steering_deg()-MCCONF_STEERING_POS_MIN_DEG)*360.0f/
            (MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);
        if(pos360<0.0f){pos360=0.0f;}
        if(pos360>360.0f){pos360=360.0f;}
        buffer_append_int32(b,(int32_t)lroundf(pos360*1000.0f),&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_STEERING_HOME) {
        uint8_t status=0u;
        if(second) status=1u;
        else if(!mc_interface_steering_calibration_valid()) status=2u;
        else if(!mc_interface_steering_boot_home()) status=3u;
        uint8_t b[16]; int32_t j=0; uint8_t flags=0u;
        if(mc_interface_steering_calibration_valid())flags|=0x01u;
        if(mcpwm_foc_steering_is_homed())flags|=0x02u;
        if(mcpwm_foc_encoder_is_synced(false))flags|=0x04u;
        if(mc_interface_steering_logical_inverted())flags|=0x08u;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=status; b[j++]=flags;
        buffer_append_int32(b,mcpwm_foc_steering_span_counts(),&j);
        uart_send_payload(b,(uint16_t)j);
        return;
    }
    if (op == HB_CUSTOM_STEERING_SET_CENTER) {
        uint8_t status=0u;
        if(second)status=1u;
        else if(!mc_interface_steering_calibration_valid())status=2u;
        else if(!mcpwm_foc_encoder_is_synced(false))status=3u;
        else if(!mc_interface_steering_set_current_as_center())status=4u;
        uint8_t b[24]; int32_t j=0; uint8_t flags=0u;
        if(mc_interface_steering_calibration_valid())flags|=0x01u;
        if(mcpwm_foc_steering_is_homed())flags|=0x02u;
        if(mcpwm_foc_encoder_is_synced(false))flags|=0x04u;
        if(mc_interface_steering_logical_inverted())flags|=0x08u;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=status; b[j++]=flags;
        buffer_append_int32(b,mcpwm_foc_steering_span_counts(),&j);
        buffer_append_int32(b,mcpwm_foc_steering_safe_span_counts(),&j);
        buffer_append_int32(b,mcpwm_foc_get_position_user_counts(false),&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_ROTOR_SNAPSHOT) {
        /* The stock VESC Tool exposes one DISP_POS_MODE at a time. ROS/Web needs
         * all engineering signals simultaneously, so return one compact snapshot
         * while reusing display_rotor_pos() for the exact same semantics.
         *
         * Inductance is intentionally NOT synthesized: upstream VESC only sends
         * mcpwm_get_detect_pos() while MC_STATE_DETECTING. This F103 target has no
         * equivalent public detector-position getter, therefore bit6 stays clear
         * and the field is N/A outside a future true detection signal. */
        const mcpwm_foc_motor_t *m=mcpwm_foc_get_motor_const(second);
        float encoder=0.0f, observer=0.0f, pid_pos=0.0f;
        float obs_enc=0.0f, obs_hall=0.0f, pid_error=0.0f;
        const bool encoder_valid=!second && m->m_encoder_configured;
        const bool observer_valid=mcpwm_foc_observer_valid(second);
        const bool hall_valid=(m->m_conf.foc_sensor_mode==FOC_SENSOR_MODE_HALL) && m->m_hall_initialized;
        (void)display_rotor_pos(second,DISP_POS_MODE_OBSERVER,&observer);
        (void)display_rotor_pos(second,DISP_POS_MODE_PID_POS,&pid_pos);
        (void)display_rotor_pos(second,DISP_POS_MODE_PID_POS_ERROR,&pid_error);
        if(encoder_valid){
            (void)display_rotor_pos(second,DISP_POS_MODE_ENCODER,&encoder);
            (void)display_rotor_pos(second,DISP_POS_MODE_ENCODER_OBSERVER_ERROR,&obs_enc);
        }
        if(hall_valid)(void)display_rotor_pos(second,DISP_POS_MODE_HALL_OBSERVER_ERROR,&obs_hall);
        uint8_t flags=0u;
        if(encoder_valid)flags|=0x01u; /* mechanical encoder */
        if(observer_valid)flags|=0x02u;/* independent FOC observer */
        flags|=0x04u;                 /* PID position */
        if(encoder_valid && observer_valid)flags|=0x08u;/* observer - encoder */
        if(hall_valid && observer_valid)flags|=0x10u;   /* observer - Hall */
        flags|=0x20u;                 /* PID setpoint - position */
        /* bit6 = inductance/detect signal valid; deliberately clear for now. */
        uint8_t b[48]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
        b[j++]=second?VESC_SECOND_MOTOR_ID:VESC_LOCAL_ID;
        b[j++]=flags; b[j++]=(uint8_t)m->m_conf.foc_sensor_mode; b[j++]=(uint8_t)m->m_state;
        buffer_append_int32(b,(int32_t)lroundf(encoder*100000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(observer*100000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(pid_pos*100000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(obs_enc*100000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(obs_hall*100000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(pid_error*100000.0f),&j);
        buffer_append_int32(b,0,&j); /* inductance/detect signal: invalid unless bit6 is set */
        uart_send_payload(b,(uint16_t)j);
        return;
    }
    if (op == HB_CUSTOM_GET_ISR_PROFILE) {
        const uint8_t reset_ok=(n<1u || d[0]==0u || mcpwm_foc_reset_isr_profile())?1u:0u;
        mcpwm_foc_isr_profile_t p; mcpwm_foc_get_isr_profile(&p);
        /* Stage-1 profiler v0x00020002 exceeds the 255-byte short-frame limit.
         * uart_send_payload() automatically emits the normal VESC long frame. */
        uint8_t b[320]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=reset_ok?0u:2u;
#define APPP(v) buffer_append_uint32(b,(v),&j)
        APPP(p.total_max_cycles); APPP(p.deadline_miss_count);
        APPP(p.pre_max_cycles); APPP(p.control_max_cycles); APPP(p.post_max_cycles);
        APPP(p.pre_gate_max_cycles); APPP(p.pre_offset_max_cycles); APPP(p.pre_protect_max_cycles);
        APPP(p.motor_step_max_cycles[0]); APPP(p.motor_step_max_cycles[1]);
        APPP(p.motor_control_max_cycles[0]); APPP(p.motor_control_max_cycles[1]);
        APPP(p.motor_hold_max_cycles[0]); APPP(p.motor_hold_max_cycles[1]);
        APPP(p.sensor_max_cycles); APPP(p.pll_max_cycles); APPP(p.current_max_cycles); APPP(p.regulator_max_cycles);
        APPP(p.position_pid_max_cycles); APPP(p.speed_pid_max_cycles); APPP(p.current_circle_max_cycles);
        APPP(p.id_pi_max_cycles); APPP(p.iq_pi_max_cycles); APPP(p.decouple_limit_max_cycles);
        APPP(p.svpwm_max_cycles); APPP(p.duty_mag_max_cycles); APPP(p.reentry_guard_total);
        for(uint8_t si=0u;si<6u;++si)APPP(p.slot_max_cycles[si]);
        for(uint8_t si=0u;si<6u;++si)APPP(p.slot_miss_count[si]);
        for(uint8_t si=0u;si<6u;++si)APPP(p.slot_count[si]);
        APPP(p.detail_sample_count);
        for(uint8_t si=0u;si<6u;++si)APPP(p.detail_slot_count[si]);
        APPP(p.steady_isr_count); APPP(p.slot_sequence_error_count);
        APPP(p.fast_hold_svpwm_max_cycles); APPP(p.profile_revision); APPP(p.active_slot_count); APPP(p.reset_epoch);
        APPP(p.outer_max_cycles); APPP(p.outer_miss_count); APPP(p.outer_jitter_max_cycles);
        APPP(p.outer_period_min_cycles); APPP(p.outer_period_max_cycles);
        APPP(p.adc_heartbeat); APPP(p.motor_heartbeat[0]); APPP(p.motor_heartbeat[1]);
        APPP(p.snapshot_dwt); APPP(p.irq_entry_count); APPP(p.irq_exit_count);
        APPP(p.motor_step_count[0]); APPP(p.motor_step_count[1]);
        APPP(p.dma_tc_pending_exit_count);
#undef APPP
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_PLATFORM_HEALTH) {
        platform_watchdog_status_t w; platform_watchdog_get_status(&w);
        uint8_t b[64]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
        b[j++]=w.enabled; b[j++]=w.last_health_ok; b[j++]=w.boot_was_iwdg; b[j++]=w.init_failed;
        b[j++]=w.init_fail_stage; b[j++]=0u;
        buffer_append_uint32(b,w.boot_reset_csr,&j); buffer_append_uint32(b,w.boot_reset_reason,&j);
        buffer_append_uint32(b,w.boot_reset_stage,&j); buffer_append_uint32(b,w.feed_count,&j);
        buffer_append_uint32(b,w.reject_count,&j); buffer_append_uint32(b,w.last_adc_heartbeat,&j);
        buffer_append_uint32(b,w.last_motor_heartbeat[0],&j); buffer_append_uint32(b,w.last_motor_heartbeat[1],&j);
        buffer_append_uint32(b,w.last_feed_ms,&j); buffer_append_uint32(b,w.iwdg_sr,&j);
        buffer_append_uint32(b,w.iwdg_pr,&j); buffer_append_uint32(b,w.iwdg_rlr,&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_COMMS_HEALTH) {
        uint8_t b[96]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
#define APPCH(v) buffer_append_uint32(b,(uint32_t)(v),&j)
        APPCH(s_rx_ok); APPCH(s_rx_crc_err); APPCH(s_rx_timeout_reset);
        APPCH(s_rx_queue_drop); APPCH(s_rx_queue_highwater); APPCH(s_rt_cmd_coalesced);
        APPCH(s_tx_queue_drop); APPCH(s_tx_start_fail); APPCH(s_tx_queue_highwater);
        APPCH(s_process_gap_max_ms); APPCH(s_pending_count); APPCH(vesc_tx_queue_count());
        APPCH(s_tx_active); APPCH(s_rx_active);
#ifdef STM32F103xE
        APPCH(usart3_rx_error_count()); APPCH(usart3_rx_restart_count()); APPCH(usart3_forced_recovery_count());
        APPCH(main_prof_vesc_max_cycles); APPCH(main_prof_house_max_cycles); APPCH(main_prof_tail_max_cycles);
#else
        for(uint8_t z=0u;z<6u;++z)APPCH(0u);
#endif
#undef APPCH
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_PLATFORM_INFO) {
        uint8_t b[64]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1; b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
        buffer_append_uint16(b,HB_PLATFORM_SCHEMA,&j); buffer_append_uint16(b,HB_DIAG_SCHEMA,&j);
        buffer_append_uint16(b,HB_ISR_SCHEMA,&j); buffer_append_uint16(b,HB_TRACE_SCHEMA,&j);
        buffer_append_uint32(b,HB_FEATURE_BITMAP,&j); buffer_append_uint32(b,F103_BUILD_ID32,&j);
        buffer_append_uint32(b,F103_GIT_SHA_HI32,&j); buffer_append_uint16(b,F103_GIT_SHA_LO16,&j);
        buffer_append_uint16(b,MCCONF_FOC_CONTROL_DIV,&j); buffer_append_uint32(b,CPU_CLOCK_HZ,&j); buffer_append_uint32(b,PWM_FREQ_HZ,&j);
        buffer_append_uint32(b,MCPWM_FOC_ISR_PROFILE_REVISION,&j); buffer_append_uint16(b,MCPWM_FOC_TRACE_CAPACITY,&j);
        buffer_append_uint16(b,MCPWM_FOC_PROFILE_SLOT_CAPACITY,&j); uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_ADC_VALIDITY) {
        mcpwm_foc_adc_sample_diag_t q;mcpwm_foc_get_adc_sample_diag(second,&q);
        uint8_t b[40];int32_t j=0;b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=0u;
        buffer_append_uint16(b,q.ccr_a,&j);buffer_append_uint16(b,q.ccr_b,&j);buffer_append_uint16(b,q.ccr_c,&j);
        buffer_append_uint16(b,q.zero_window_counts,&j);buffer_append_uint16(b,q.min_window_counts,&j);buffer_append_uint16(b,q.guard_counts,&j);buffer_append_uint16(b,q.adc_phase_counts,&j);
        buffer_append_uint32(b,q.invalid_count,&j);b[j++]=q.sector;b[j++]=q.window_valid;b[j++]=q.offset_valid;b[j++]=q.driven_offset_valid;b[j++]=q.bridge_settled;
        uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_ARM_CURRENT_STEP) {
        uint8_t status=1u;uint32_t seq=0u;
        if(n>=10u){
            int32_t si=0;const int32_t pre_ma=buffer_get_int32(d,&si);const int32_t step_ma=buffer_get_int32(d,&si);
            const uint8_t pre_n=d[si++],post_n=d[si++];
            const uint8_t axis_raw=(n>=11u)?d[si++]:0u;
            const mcpwm_foc_step_axis_t axis=(axis_raw==1u)?MCPWM_FOC_STEP_AXIS_D:MCPWM_FOC_STEP_AXIS_Q;
            if(axis_raw>1u)status=3u;
            else status=mcpwm_foc_step_test_arm_axis((float)pre_ma*0.001f,(float)step_ma*0.001f,pre_n,post_n,second,axis)?0u:2u;
            mcpwm_foc_step_test_status_t st;mcpwm_foc_step_test_get(&st);seq=st.sequence;
        }
        uint8_t b[10]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,status,0,0,0,0};int32_t j=6;buffer_append_uint32(b,seq,&j);uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_GET_STEP_STATUS) {
        mcpwm_foc_step_test_status_t st;mcpwm_foc_step_test_get(&st);uint8_t b[24];int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=0u;
        buffer_append_uint32(b,st.sequence,&j);buffer_append_int16(b,st.pre_q4,&j);buffer_append_int16(b,st.step_q4,&j);
        b[j++]=st.active;b[j++]=st.second;b[j++]=st.pre_remaining;b[j++]=st.post_remaining;b[j++]=st.step_fired;b[j++]=st.done;uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_GET_POSITION_D_STATE) {
        const mcpwm_foc_motor_t *pm=mcpwm_foc_get_motor_const(second);
        uint8_t b[24];int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=0u;
        buffer_append_int32(b,mcpwm_foc_get_position_user_counts(second),&j);
        buffer_append_int32(b,mcpwm_foc_get_position_target_user_counts(second),&j);
        buffer_append_int32(b,pm?pm->m_position_d_proc_filter_q15:0,&j);
        b[j++]=pm?(uint8_t)pm->m_control_mode:0u;b[j++]=pm?pm->m_pos_pid_phase_mode:0u;
        b[j++]=pm?(pm->m_conf.foc_encoder_inverted?1u:0u):0u;b[j++]=pm?(pm->m_conf.m_invert_direction?1u:0u):0u;
        uart_send_payload(b,(uint16_t)j);return;
    }

    if (op == HB_CUSTOM_START_RELAY_AUTOTUNE) {
        uint8_t status=1u;uint32_t seq=0u;
        if(n>=16u){
            int32_t si=0;const uint8_t mode=d[si++];
            const int32_t target=buffer_get_int32(d,&si);const int32_t hyst=buffer_get_int32(d,&si);
            const uint16_t relay_ma=buffer_get_uint16(d,&si);const uint8_t crossings=d[si++];
            const uint32_t timeout_ms=buffer_get_uint32(d,&si);
            if(mode<1u || mode>2u)status=3u;
            else status=mcpwm_foc_relay_start((mcpwm_foc_relay_mode_t)mode,second,target,hyst,relay_ma,crossings,timeout_ms)?0u:2u;
            mcpwm_foc_relay_status_t st;mcpwm_foc_relay_get(&st);seq=st.sequence;
        }
        uint8_t b[10]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,status,0,0,0,0};
        int32_t j=6;buffer_append_uint32(b,seq,&j);uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_GET_RELAY_AUTOTUNE) {
        mcpwm_foc_relay_status_t st;mcpwm_foc_relay_get(&st);uint8_t b[56];int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=0u;
        buffer_append_uint32(b,st.sequence,&j);buffer_append_uint32(b,st.elapsed_ms,&j);buffer_append_uint32(b,st.period_sum_ms,&j);
        buffer_append_int32(b,st.target,&j);buffer_append_int32(b,st.hysteresis,&j);buffer_append_int32(b,st.measurement,&j);
        buffer_append_int32(b,st.minimum,&j);buffer_append_int32(b,st.maximum,&j);
        buffer_append_uint16(b,st.relay_current_ma,&j);buffer_append_uint16(b,st.period_count,&j);
        b[j++]=st.active;b[j++]=st.done;b[j++]=st.failed;b[j++]=st.mode;b[j++]=st.second;b[j++]=st.relay_positive;
        b[j++]=st.crossings;b[j++]=st.required_crossings;uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_ABORT_RELAY_AUTOTUNE) {
        mcpwm_foc_relay_abort();uint8_t b[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,0u};
        uart_send_payload(b,6u);return;
    }

    if (op == HB_CUSTOM_BOOT_HANDOFF) {
        uint8_t b[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,0u};
        if(second||s_boot_handoff_pending){b[5]=1u;uart_send_payload(b,6u);return;}
        mcpwm_foc_force_bridges_off();
        uart_send_payload(b,6u);s_boot_handoff_deadline_ms=HAL_GetTick()+HB_BOOT_HANDOFF_TIMEOUT_MS;HB_MEMORY_BARRIER();s_boot_handoff_pending=1u;return;
    }

    if (op == HB_CUSTOM_GET_FW_UPDATE_STATE) {
        const f103_update_meta_t *m=(const f103_update_meta_t *)F103_META_BASE_ADDR;
        const bool common=(m->magic==F103_UPDATE_META_MAGIC) &&
                          (m->size>0u && m->size<=F103_APP_REGION_SIZE) &&
                          (m->size==~m->size_inv) &&
                          (m->version==F103_UPDATE_META_VERSION) &&
                          ((uint16_t)(m->version ^ m->version_inv)==0xFFFFu);
        const bool crc_ok=common && ((uint16_t)(m->crc16 ^ m->crc16_inv)==0xFFFFu);
        uint8_t b[20]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=(common && crc_ok)?0u:1u;
        buffer_append_uint32(b,common?m->state:0u,&j);
        buffer_append_uint32(b,common?m->size:0u,&j);
        buffer_append_uint16(b,crc_ok?m->crc16:0u,&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_TRACE_META) {
        mcpwm_foc_trace_meta_t t; mcpwm_foc_trace_get_meta(&t);
        uint8_t b[32]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=0u;
        buffer_append_uint32(b,t.write_count,&j);b[j++]=t.frozen;b[j++]=t.trigger_motor;b[j++]=t.trigger_fault;
        b[j++]=t.count;b[j++]=t.head;b[j++]=t.capacity;buffer_append_uint16(b,t.sample_size,&j);
        uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_GET_TRACE_SAMPLE) {
        mcpwm_foc_trace_sample_t t; const uint8_t idx=n?d[0]:0xffu;
        const uint8_t status=(n>=1u && mcpwm_foc_trace_read(idx,&t))?0u:1u;
        uint8_t b[96];int32_t j=0;b[j++]=COMM_CUSTOM_APP_DATA;b[j++]=HB_CUSTOM_MAGIC0;b[j++]=HB_CUSTOM_MAGIC1;b[j++]=HB_CUSTOM_VERSION;b[j++]=op;b[j++]=status;
        if(!status){
            buffer_append_uint32(b,t.pwm_tick,&j);buffer_append_uint16(b,t.isr_cycles,&j);b[j++]=t.control_slot;b[j++]=t.event_bits;
#define APPS(v) buffer_append_int16(b,(v),&j)
            APPS(t.left_id_q4);APPS(t.left_iq_q4);APPS(t.left_id_set_q4);APPS(t.left_iq_set_q4);APPS(t.left_vd);APPS(t.left_vq);APPS(t.left_erpm);
            APPS(t.right_id_q4);APPS(t.right_iq_q4);APPS(t.right_id_set_q4);APPS(t.right_iq_set_q4);APPS(t.right_vd);APPS(t.right_vq);APPS(t.right_erpm);
#undef APPS
            buffer_append_int32(b,t.left_id_integrator,&j);buffer_append_int32(b,t.left_iq_integrator,&j);
            buffer_append_int32(b,t.right_id_integrator,&j);buffer_append_int32(b,t.right_iq_integrator,&j);
            buffer_append_uint16(b,t.left_sample_window,&j);buffer_append_uint16(b,t.right_sample_window,&j);
            buffer_append_uint16(b,t.vin_adc,&j);b[j++]=t.left_fault;b[j++]=t.right_fault;b[j++]=t.left_quality;b[j++]=t.right_quality;
        }
        uart_send_payload(b,(uint16_t)j);return;
    }
    if (op == HB_CUSTOM_CLEAR_TRACE) {
        const bool ok=mcpwm_foc_trace_clear();uint8_t b[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,ok?0u:2u};uart_send_payload(b,6u);return;
    }
    if (op == HB_CUSTOM_FREEZE_TRACE) {
        const bool ok=mcpwm_foc_trace_freeze();uint8_t b[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,ok?0u:2u};uart_send_payload(b,6u);return;
    }

    if (op == HB_CUSTOM_ENCODER_DEBUG) {
        if(second)return;
        const mcpwm_foc_motor_t *m=mcpwm_foc_get_motor_const(false);
        uint8_t b[96]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1;
        b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=0u;
        b[j++]=encoder_align_stage; b[j++]=steering_detect_stage; b[j++]=encoder_detect_stage;
        b[j++]=m->m_conf.foc_encoder_inverted?1u:0u;
        b[j++]=m->m_encoder_configured?1u:0u; b[j++]=m->m_encoder_synced?1u:0u;
        buffer_append_int32(b,(int32_t)lroundf(m->m_conf.foc_encoder_offset*1000.0f),&j);
        buffer_append_int32(b,(int32_t)lroundf(m->m_conf.foc_encoder_ratio*1000.0f),&j);
        buffer_append_uint32(b,m->m_encoder_raw_count,&j);
        buffer_append_uint32(b,encoder_align_before_count,&j);
        buffer_append_uint32(b,encoder_align_jog_count,&j);
        buffer_append_uint32(b,encoder_align_back_count,&j);
        buffer_append_int32(b,encoder_align_jog_delta,&j);
        buffer_append_int32(b,encoder_align_back_delta,&j);
        buffer_append_int32(b,encoder_detect_plus_mdeg,&j);
        buffer_append_int32(b,encoder_detect_minus_mdeg,&j);
        buffer_append_uint32(b,encoder_gpio_edge_a,&j);
        buffer_append_uint32(b,encoder_gpio_edge_b,&j);
        buffer_append_uint32(b,encoder_gpio_edge_pb5,&j);
        buffer_append_uint32(b,encoder_gpio_samples,&j);
        buffer_append_uint16(b,encoder_align_current_ma,&j);
        buffer_append_int32(b,mcpwm_foc_steering_span_counts(),&j);
        buffer_append_int32(b,m->m_position_counts,&j);
        buffer_append_int32(b,m->m_position_target_counts,&j);
        buffer_append_int32(b,m->m_position_pid_target_counts,&j);
        uart_send_payload(b,(uint16_t)j);
        return;
    }
    if (op == HB_CUSTOM_SET_STEERING_DEG) {
        /* ROS/Web runtime path: signed mechanical steering degrees with 0=center.
         * Keep this separate from stock COMM_SET_POS, which remains the upstream
         * single-turn PID-position command. No per-cycle ACK is emitted; custom
         * steering calibration telemetry is the mechanical feedback channel. */
        if (second || n < 4u) return;
        int32_t mdeg=buffer_get_int32(d,&k);
        if(mdeg>30000)mdeg=30000;
        if(mdeg< -30000)mdeg=-30000;
        touch_motor(false);
        (void)mc_interface_set_steering_deg((float)mdeg/1000.0f);
        return;
    }
    if (op == HB_CUSTOM_GET_TUNING || op == HB_CUSTOM_SET_TUNING) {
        mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);
        uint8_t tuning_status=0u;
        if (op == HB_CUSTOM_SET_TUNING) {
            if (n < 20u) { uint8_t e[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,1u}; uart_send_payload(e,6u); return; }
            mc_configuration *const backup=&s_mc_txn_backup[0];
            *backup=m->m_conf;
            m->m_kpq_q11=buffer_get_uint16(d,&k); m->m_kiq_q16=buffer_get_uint16(d,&k);
            m->m_kpd_q11=buffer_get_uint16(d,&k); m->m_kid_q16=buffer_get_uint16(d,&k);
            m->m_kps_q11=buffer_get_uint16(d,&k); m->m_kis_q16=buffer_get_uint16(d,&k); m->m_kds_q11=buffer_get_uint16(d,&k);
            m->m_kpp_q11=buffer_get_uint16(d,&k); m->m_kip_q16=buffer_get_uint16(d,&k); m->m_kdp_q11=buffer_get_uint16(d,&k);
            mcpwm_foc_sync_tuning_to_conf(second);
            bool store=false;
            if (n >= 22u) {
                uint16_t fa=buffer_get_uint16(d,&k);
                if(fa<1u)fa=1u;
                m->m_telem_current_filter_q16=fa;
                uint32_t fx10000=((uint32_t)fa*10000u+32767u)/65535u;
                if(fx10000<10u)fx10000=10u;
                if(fx10000>10000u)fx10000=10000u;
                m->m_conf.foc_current_filter_const=(float)fx10000/10000.0f;
                if(n>=23u)store=d[22]!=0u;
            } else if (n >= 21u) store=d[20]!=0u;
            if(store && !mc_interface_store_configuration_motor(second)){
                /* Persistent tuning is transactional: restore runtime to the
                 * pre-command MC image and report a nonzero custom status. */
                mc_interface_set_configuration(backup);
                (void)mc_interface_store_configuration_motor(second);
                m=mcpwm_foc_get_motor(second);
                tuning_status=3u;
            }
        }
        uint8_t b[40]; int32_t j=0;
        b[j++]=COMM_CUSTOM_APP_DATA; b[j++]=HB_CUSTOM_MAGIC0; b[j++]=HB_CUSTOM_MAGIC1; b[j++]=HB_CUSTOM_VERSION; b[j++]=op; b[j++]=tuning_status;
        buffer_append_uint16(b,m->m_kpq_q11,&j); buffer_append_uint16(b,m->m_kiq_q16,&j);
        buffer_append_uint16(b,m->m_kpd_q11,&j); buffer_append_uint16(b,m->m_kid_q16,&j);
        buffer_append_uint16(b,m->m_kps_q11,&j); buffer_append_uint16(b,m->m_kis_q16,&j); buffer_append_uint16(b,m->m_kds_q11,&j);
        buffer_append_uint16(b,m->m_kpp_q11,&j); buffer_append_uint16(b,m->m_kip_q16,&j); buffer_append_uint16(b,m->m_kdp_q11,&j);
        buffer_append_uint16(b,m->m_telem_current_filter_q16,&j); buffer_append_int16(b,m->m_current_limit_q4,&j);
        uart_send_payload(b,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_SET_ID_TEST) {
        uint8_t status=0u;
        if (n < 8u) status=1u;
        int32_t ma=0,mdeg=0;
        if(status==0u){
            ma=buffer_get_int32(d,&k); mdeg=buffer_get_int32(d,&k);
            if(ma<0)ma=-ma;
            if(ma>5000 || mdeg>360000 || mdeg<-360000)status=2u;
        }
        touch_motor(second);
        if(status!=0u || ma==0)mc_interface_release_motor();
        else mcpwm_foc_set_openloop_phase((float)ma/1000.0f,(float)mdeg/1000.0f,second);
        { uint8_t a[6]={COMM_CUSTOM_APP_DATA,HB_CUSTOM_MAGIC0,HB_CUSTOM_MAGIC1,HB_CUSTOM_VERSION,op,status}; uart_send_payload(a,6u); }
        return;
    }
    if (op == HB_CUSTOM_SET_OPENLOOP_TEST) {
        uint8_t status = 0u;
        int32_t ma = 0, merpm = 0;
        uint16_t duration_ms = 0u;
        if (n < 10u) status = 1u;
        if (status == 0u) {
            ma = buffer_get_int32(d,&k);
            merpm = buffer_get_int32(d,&k);
            duration_ms = buffer_get_uint16(d,&k);
            if (ma < 0) ma = -ma;
            if (ma > HB_OPENLOOP_TEST_MAX_MA || merpm > HB_OPENLOOP_TEST_MAX_MERPM ||
                merpm < -HB_OPENLOOP_TEST_MAX_MERPM || duration_ms > HB_OPENLOOP_TEST_MAX_MS ||
                (ma != 0 && duration_ms < HB_OPENLOOP_TEST_MIN_MS)) status = 2u;
        }
        if (status == 0u) {
            if (s_openloop_test_active) {
                mc_interface_select_motor_thread(s_openloop_test_second ? 2 : 1);
                mc_interface_release_motor();
                s_openloop_test_active = 0u;
            }
            mc_interface_select_motor_thread(second ? 2 : 1);
            touch_motor(second);
            if (ma == 0 || merpm == 0 || duration_ms == 0u) {
                mc_interface_release_motor();
                mc_interface_select_motor_thread(1);
                s_openloop_test_active = 0u;
            } else {
                const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
                if (!m || m->m_fault != FAULT_CODE_NONE) status = 3u;
                else {
                    float applied_erpm=(float)merpm / 1000.0f;
                    if(m->m_conf.m_invert_direction)applied_erpm=-applied_erpm;
                    mcpwm_foc_set_openloop_current((float)ma / 1000.0f, applied_erpm, second);
                    s_openloop_test_second = second ? 1u : 0u;
                    s_openloop_test_deadline = detect_time_after_ms(detect_time_now(), (uint32_t)duration_ms);
                    s_openloop_test_active = 1u;
                }
                mc_interface_select_motor_thread(1);
            }
        }
        if (status != 0u) {
            mc_interface_select_motor_thread(second ? 2 : 1);
            mc_interface_release_motor();
            mc_interface_select_motor_thread(1);
            s_openloop_test_active = 0u;
        }
        uint8_t a[18]; int32_t j=0;
        a[j++]=COMM_CUSTOM_APP_DATA; a[j++]=HB_CUSTOM_MAGIC0; a[j++]=HB_CUSTOM_MAGIC1; a[j++]=HB_CUSTOM_VERSION; a[j++]=op; a[j++]=status;
        buffer_append_int32(a,ma,&j); buffer_append_int32(a,merpm,&j); buffer_append_uint16(a,duration_ms,&j);
        uart_send_payload(a,(uint16_t)j);
        return;
    }
    if (op == HB_CUSTOM_HALL_PIN_TEST) {
        uint8_t status=0u, raw0=0u, raw_dn=0u, raw_up=0u;
        if(!second) status=1u;
        const mcpwm_foc_motor_t *m=mcpwm_foc_get_motor_const(true);
        if(status==0u && (!m || m->m_state!=MC_STATE_OFF))status=2u;
        if(status==0u){
#ifdef STM32F103xE
            volatile uint32_t *const crh=(volatile uint32_t *)0x40011004u;
            volatile uint32_t *const idr=(volatile uint32_t *)0x40011008u;
            volatile uint32_t *const odr=(volatile uint32_t *)0x4001100Cu;
            const uint32_t saved_crh=*crh, saved_odr=*odr;
            raw0=(uint8_t)((*idr>>10)&7u);
            *crh=(saved_crh & ~0x000FFF00u) | 0x00088800u; /* PC10..12 input pull */
            *odr=saved_odr & ~(7u<<10);                   /* internal pull-down */
            for(volatile uint32_t z=0;z<10000u;++z){}
            raw_dn=(uint8_t)((*idr>>10)&7u);
            *odr=saved_odr | (7u<<10);                    /* internal pull-up */
            for(volatile uint32_t z=0;z<10000u;++z){}
            raw_up=(uint8_t)((*idr>>10)&7u);
            *crh=saved_crh; *odr=saved_odr;
#else
            status=3u;
#endif
        }
        uint8_t a[10]; int32_t j=0;
        a[j++]=COMM_CUSTOM_APP_DATA; a[j++]=HB_CUSTOM_MAGIC0; a[j++]=HB_CUSTOM_MAGIC1; a[j++]=HB_CUSTOM_VERSION; a[j++]=op; a[j++]=status;
        a[j++]=raw0; a[j++]=raw_dn; a[j++]=raw_up;
        uart_send_payload(a,(uint16_t)j); return;
    }
    if (op == HB_CUSTOM_GET_DIAG) {
        /* Payload diagnostic bertambah lintas revisi. Sisakan headroom besar dan
         * jangan lagi mengandalkan ukuran historis 208 byte yang sudah overflow. */
        uint8_t b[256];
        int32_t i = 0;
        const mcpwm_foc_motor_t *m = mcpwm_foc_get_motor_const(second);
        struct {
            mc_control_mode mode; mc_state state; mc_fault_code fault;
            uint8_t hall,hall_pos,hall_prev,interp,rej_reason,rej_from,rej_to,hall_init; int8_t hall_dir;
            int16_t iq_target,iq_set,iq,id,duty,rpm;
            int32_t pos,pos_target,pos_min,pos_max;
            uint16_t phase,phase_hall,phase_target,hall_period,hall_ticks;
            uint32_t hall_invalid,current_trips,period_rejects,sequence_rejects;
            uint32_t phase_trips,dc_trips;
            uint8_t phase_streak,last_trip_source;
            int16_t last_trip_p0,last_trip_p1,last_trip_p2,last_trip_dc,last_trip_duty;
            int16_t driven_off0,driven_off1,driven_offdc;
            uint16_t driven_samples; uint8_t driven_valid,driven_cal;
            int16_t off0,off1,offdc; uint16_t off_samples,off_settle; uint8_t off_valid;
        } ds = {0};
        uint32_t snap_e0,snap_x0,snap_e1,snap_x1;
        for(;;) {
        mcpwm_foc_get_irq_epoch(&snap_e0,&snap_x0);
        if(snap_e0!=snap_x0)continue;
        ds.mode=m->m_control_mode; ds.state=m->m_state; ds.fault=m->m_fault;
        ds.hall=m->m_hall_state; ds.hall_pos=m->m_hall_pos; ds.hall_prev=m->m_hall_pos_prev;
        ds.hall_dir=m->m_hall_direction; ds.interp=m->m_hall_interp_active; ds.rej_reason=m->m_hall_last_reject_reason;
        ds.rej_from=m->m_hall_last_reject_from; ds.rej_to=m->m_hall_last_reject_to; ds.hall_init=m->m_hall_initialized;
        ds.iq_target=m->m_iq_target_q4; ds.iq_set=m->m_iq_set_q4; ds.iq=m->m_iq_q4; ds.id=m->m_id_q4;
        ds.duty=m->m_duty_now_permille; ds.rpm=m->m_rpm; ds.pos=m->m_position_counts;
        ds.pos_target=m->m_position_target_counts; ds.pos_min=m->m_position_min_counts; ds.pos_max=m->m_position_max_counts;
        ds.phase=m->m_phase; ds.phase_hall=m->m_phase_hall; ds.phase_target=m->m_phase_hall_target;
        ds.hall_period=m->m_hall_period; ds.hall_ticks=m->m_hall_ticks;
        ds.hall_invalid=m->m_hall_invalid_transition_count; ds.current_trips=m->m_current_trip_count;
        ds.period_rejects=m->m_hall_period_reject_count; ds.sequence_rejects=m->m_hall_sequence_reject_count;
        ds.phase_trips=m->m_phase_trip_count; ds.dc_trips=m->m_dc_trip_count; ds.phase_streak=m->m_phase_overcurrent_streak;
        ds.last_trip_source=m->m_last_trip_source; ds.last_trip_p0=m->m_last_trip_phase0_counts; ds.last_trip_p1=m->m_last_trip_phase1_counts;
        ds.last_trip_p2=m->m_last_trip_phase2_counts; ds.last_trip_dc=m->m_last_trip_dc_counts; ds.last_trip_duty=m->m_last_trip_duty_permille;
        ds.driven_off0=m->m_driven_offset0; ds.driven_off1=m->m_driven_offset1; ds.driven_offdc=m->m_driven_offsetdc;
        ds.driven_samples=m->m_driven_offset_samples; ds.driven_valid=m->m_driven_offset_valid; ds.driven_cal=m->m_driven_offset_calibrating;
        ds.off0=m->m_off_offset0; ds.off1=m->m_off_offset1; ds.offdc=m->m_off_offsetdc;
        ds.off_samples=m->m_off_offset_samples; ds.off_settle=m->m_off_settle_ticks; ds.off_valid=m->m_off_offset_valid;
        mcpwm_foc_get_irq_epoch(&snap_e1,&snap_x1);
        if(snap_e0==snap_e1 && snap_x0==snap_x1 && snap_e1==snap_x1)break;
        }
        const uint16_t pp=mcpwm_foc_get_pole_pairs(second);
        float erpm_f=(float)ds.rpm*(float)pp;
        if(ds.hall_init && ds.hall_dir!=0 && ds.hall_period>0u && ds.hall_period<MCCONF_HALL_TIMEOUT_TICKS && ds.hall_ticks<=MCCONF_HALL_TIMEOUT_TICKS)
            erpm_f=((float)PWM_FREQ*10.0f/(float)ds.hall_period)*(float)ds.hall_dir;
        if (second) erpm_f = -erpm_f;
        int32_t erpm = (int32_t)(erpm_f >= 0.0f ? erpm_f + 0.5f : erpm_f - 0.5f);
        int32_t duty = (int32_t)ds.duty * 100;
        if (second) duty = -duty;
        const int32_t pos_user = second ? (ds.pos==INT32_MIN?INT32_MAX:-ds.pos) : ds.pos;
        const int32_t target_user = second ? (ds.pos_target==INT32_MIN?INT32_MAX:-ds.pos_target) : ds.pos_target;
        const int32_t min_user = second ? (ds.pos_max==INT32_MIN?INT32_MAX:-ds.pos_max) : ds.pos_min;
        const int32_t max_user = second ? (ds.pos_min==INT32_MIN?INT32_MAX:-ds.pos_min) : ds.pos_max;

        b[i++] = COMM_CUSTOM_APP_DATA;
        b[i++] = HB_CUSTOM_MAGIC0;
        b[i++] = HB_CUSTOM_MAGIC1;
        b[i++] = HB_CUSTOM_VERSION;
        b[i++] = op;
        b[i++] = 0u;
        b[i++] = second ? VESC_SECOND_MOTOR_ID : VESC_LOCAL_ID;
        b[i++] = (uint8_t)ds.mode;
        b[i++] = (uint8_t)ds.state;
        b[i++] = (uint8_t)ds.fault;
        b[i++] = ds.hall;
        b[i++] = mcpwm_foc_vesc_override_active(second) ? 1u : 0u;
        b[i++] = s_last_hall_store_ok[second ? 1u : 0u];
        /* Logical ARM follows a live VESC binary link (VESC Tool or Python).
         * This is intentionally distinct from motor-command ownership above:
         * telemetry alone arms the control link, but never energizes the bridge. */
        b[i++] = vesc_protocol_link_active() ? 1u : 0u;
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq_target, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq_set, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.iq, second), &i);
        buffer_append_int32(b, q4_to_milliamps_normalized(ds.id, false), &i);
        buffer_append_int32(b, erpm, &i);
        buffer_append_int32(b, duty, &i);
        buffer_append_int32(b, pos_user, &i);
        buffer_append_int32(b, target_user, &i);
        buffer_append_int32(b, min_user, &i);
        buffer_append_int32(b, max_user, &i);
        buffer_append_uint32(b, ds.hall_invalid, &i);
        buffer_append_uint32(b, ds.current_trips, &i);
        buffer_append_uint32(b, s_rx_ok, &i);
        buffer_append_uint32(b, s_rx_crc_err, &i);
        for (uint8_t h = 0u; h < 8u; ++h) b[i++] = (uint8_t)m->m_conf.foc_hall_table[h];
        /* Extended Hall/phase diagnostics. Values are raw fixed-point so the
         * host can prove table->electrical-angle->FOC-phase mapping while the
         * bridge is active. Backward-compatible: legacy parsers may stop at 78. */
        b[i++] = (uint8_t)m->m_conf.foc_hall_table[ds.hall & 7u];
        b[i++] = ds.hall_pos;
        b[i++] = ds.hall_prev;
        b[i++] = (uint8_t)ds.hall_dir;
        b[i++] = ds.interp;
        b[i++] = ds.rej_reason;
        b[i++] = ds.rej_from;
        b[i++] = ds.rej_to;
        buffer_append_uint16(b, ds.phase, &i);
        buffer_append_uint16(b, ds.phase_hall, &i);
        buffer_append_uint16(b, ds.phase_target, &i);
        buffer_append_uint16(b, ds.hall_period, &i);
        buffer_append_uint16(b, ds.hall_ticks, &i);
        buffer_append_uint32(b, ds.period_rejects, &i);
        buffer_append_uint32(b, ds.sequence_rejects, &i);
        /* Drivetrain/current-calibration diagnostics are project extensions
         * appended after the stable V16 payload. VESC standard telemetry remains
         * unchanged; older host parsers can stop at byte 104. */
        {
            int16_t po0=0,po1=0,dco=0;
            mcpwm_foc_get_current_offsets(&po0,&po1,&dco,second);
            buffer_append_int16(b,po0,&i); buffer_append_int16(b,po1,&i); buffer_append_int16(b,dco,&i);
            b[i++]=(uint8_t)m->m_conf.si_motor_poles;
            b[i++]=(uint8_t)pp;
            const float gear=mcpwm_foc_get_gear_ratio(second);
            buffer_append_int32(b,(int32_t)(gear*1000.0f+0.5f),&i);
            { const float mr=(pp>0u?erpm_f/(float)pp:0.0f);
              const float orpm=mr/gear;
              buffer_append_int32(b,(int32_t)(mr>=0.0f?mr*1000.0f+0.5f:mr*1000.0f-0.5f),&i);
              buffer_append_int32(b,(int32_t)(orpm>=0.0f?orpm*1000.0f+0.5f:orpm*1000.0f-0.5f),&i); }
            buffer_append_uint32(b,s_rx_queue_drop,&i);
            buffer_append_uint32(b,mcpwm_foc_get_isr_cycles(),&i);
            buffer_append_uint32(b,mcpwm_foc_get_isr_cycles_max(),&i);
            /* High-duty ABS-overcurrent diagnostics. Keep legacy fields first so
             * older tools remain compatible; these bytes identify whether the
             * last FAULT_CODE_ABS_OVER_CURRENT came from phase or DC sensing. */
            buffer_append_uint32(b,ds.phase_trips,&i); buffer_append_uint32(b,ds.dc_trips,&i);
            b[i++]=ds.phase_streak; b[i++]=ds.last_trip_source;
            buffer_append_int16(b,ds.last_trip_p0,&i); buffer_append_int16(b,ds.last_trip_p1,&i);
            buffer_append_int16(b,ds.last_trip_p2,&i); buffer_append_int16(b,ds.last_trip_dc,&i);
            buffer_append_int16(b,ds.last_trip_duty,&i);
            buffer_append_int16(b,ds.driven_off0,&i); buffer_append_int16(b,ds.driven_off1,&i); buffer_append_int16(b,ds.driven_offdc,&i);
            buffer_append_uint16(b,ds.driven_samples,&i); b[i++]=ds.driven_valid; b[i++]=ds.driven_cal;
            /* Raw ADC snapshot for calibration/telemetry validation. This is
             * diagnostic-only and never participates in VESC standard packets. */
            buffer_append_uint16(b,adc_buffer.rlA,&i); buffer_append_uint16(b,adc_buffer.rlB,&i);
            buffer_append_uint16(b,adc_buffer.dcl,&i); buffer_append_uint16(b,adc_buffer.rrB,&i);
            buffer_append_uint16(b,adc_buffer.rrC,&i); buffer_append_uint16(b,adc_buffer.dcr,&i);
            buffer_append_int16(b,ds.off0,&i); buffer_append_int16(b,ds.off1,&i); buffer_append_int16(b,ds.offdc,&i);
            buffer_append_uint16(b,ds.off_samples,&i); buffer_append_uint16(b,ds.off_settle,&i); b[i++]=ds.off_valid; /* off_offset_valid */
            buffer_append_uint32(b,s_tx_queue_drop,&i);
            buffer_append_uint32(b,s_tx_start_fail,&i);
            buffer_append_uint32(b,s_rx_queue_highwater,&i);
            buffer_append_uint32(b,s_process_gap_max_ms,&i);
            /* Diagnostic slot: actual ADC/FOC ISR invocation counter. This is
             * more actionable than a nested-DWT main-loop maximum while timing
             * the interrupt cadence on real hardware. */
            buffer_append_uint32(b,m->m_isr_count,&i);
            /* Reuse two diagnostic-only profiler slots for USART3 recovery. */
#ifdef STM32F103xE
            buffer_append_uint32(b,usart3_rx_error_count(),&i);
            buffer_append_uint32(b,usart3_rx_restart_count(),&i);
#else
            buffer_append_uint32(b,0u,&i);
            buffer_append_uint32(b,0u,&i);
#endif
#ifdef STM32F103xE
            /* Temporary steering commissioning diagnostics. Reuse profiler slots
             * without growing the already-near-256-byte packet. */
            buffer_append_uint32(b,(uint32_t)m->m_position_no_motion_ticks,&i);
            buffer_append_uint32(b,(uint32_t)m->m_position_breakaway_ticks,&i);
            buffer_append_int32(b,m->m_position_last_motion_count,&i);
            buffer_append_int32(b,m->m_position_target_counts-m->m_position_counts,&i);
            /* Keep diagnostic reply below the 256-byte local payload buffer.
             * Pre/post were measured separately during profiling; retain the
             * actionable control/step maxima plus explicit re-entry guard diagnostics. */
            buffer_append_uint32(b,foc_prof_sensor_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_current_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_regulator_max_cycles,&i);
            buffer_append_uint32(b,foc_prof_svpwm_max_cycles,&i);
            buffer_append_uint32(b,m_motor_1.m_overrun_count+m_motor_2.m_overrun_count,&i);
#else
            for(uint8_t pi=0u;pi<9u;++pi)buffer_append_uint32(b,0u,&i);
#endif
            b[i++]=m->m_current_offset_valid; /* boot/current offset validity is distinct from off_offset_valid */
        }
        if ((uint32_t)i <= sizeof(b)) uart_send_payload(b, (uint16_t)i);
    }
}

/** VESC Tool Terminal output. Keep one print below the normal payload limit. */
static void terminal_send_text(const char *text) {
    static uint8_t b[VESC_MAX_PAYLOAD];
    if(!text)return;
    size_t n=strlen(text); if(n>sizeof(b)-1u)n=sizeof(b)-1u;
    b[0]=COMM_PRINT; memcpy(&b[1],text,n); uart_send_payload(b,(uint16_t)(n+1u));
}

static const char *vesc_fault_name(mc_fault_code fault) {
    switch (fault) {
    case FAULT_CODE_NONE: return "NONE";
    case FAULT_CODE_OVER_VOLTAGE: return "OVER_VOLTAGE";
    case FAULT_CODE_UNDER_VOLTAGE: return "UNDER_VOLTAGE";
    case FAULT_CODE_DRV: return "DRV";
    case FAULT_CODE_ABS_OVER_CURRENT: return "ABS_OVER_CURRENT";
    case FAULT_CODE_OVER_TEMP_FET: return "OVER_TEMP_FET";
    case FAULT_CODE_OVER_TEMP_MOTOR: return "OVER_TEMP_MOTOR";
    case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "GATE_DRIVER_OVER_VOLTAGE";
    case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "GATE_DRIVER_UNDER_VOLTAGE";
    case FAULT_CODE_MCU_UNDER_VOLTAGE: return "MCU_UNDER_VOLTAGE";
    case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "BOOTING_FROM_WATCHDOG_RESET";
    case FAULT_CODE_ENCODER_SPI: return "ENCODER_SPI";
    case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
    case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
    case FAULT_CODE_FLASH_CORRUPTION: return "FLASH_CORRUPTION";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "HIGH_OFFSET_CURRENT_SENSOR_1";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "HIGH_OFFSET_CURRENT_SENSOR_2";
    case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "HIGH_OFFSET_CURRENT_SENSOR_3";
    case FAULT_CODE_UNBALANCED_CURRENTS: return "UNBALANCED_CURRENTS";
    case FAULT_CODE_BRK: return "BRK";
    case FAULT_CODE_RESOLVER_LOT: return "RESOLVER_LOT";
    case FAULT_CODE_RESOLVER_DOS: return "RESOLVER_DOS";
    case FAULT_CODE_RESOLVER_LOS: return "RESOLVER_LOS";
    case FAULT_CODE_FLASH_CORRUPTION_APP_CFG: return "FLASH_CORRUPTION_APP_CFG";
    case FAULT_CODE_FLASH_CORRUPTION_MC_CFG: return "FLASH_CORRUPTION_MC_CFG";
    case FAULT_CODE_ENCODER_NO_MAGNET: return "ENCODER_NO_MAGNET";
    case FAULT_CODE_ENCODER_MAGNET_TOO_STRONG: return "ENCODER_MAGNET_TOO_STRONG";
    case FAULT_CODE_PHASE_FILTER: return "PHASE_FILTER";
    case FAULT_CODE_ENCODER_FAULT: return "ENCODER_FAULT";
    case FAULT_CODE_LV_OUTPUT_FAULT: return "LV_OUTPUT_FAULT";
    case FAULT_CODE_ENCODER_SLIP: return "ENCODER_SLIP";
    case FAULT_CODE_OVERSPEED: return "OVERSPEED";
    case FAULT_CODE_UNDERSPEED: return "UNDERSPEED";
    case FAULT_CODE_ABS_OVERSPEED: return "ABS_OVERSPEED";
    default: return "UNKNOWN";
    }
}

/* Tiny decimal parsers avoid pulling strtof/strtol into the 120-KiB app image. */
static bool terminal_float(const char *s,float *out){
    if(!s||!out||!*s)return false;
    bool neg=false;
    if(*s=='-'||*s=='+'){neg=*s=='-';s++;}
    uint32_t ip=0u,fp=0u,fs=1u; bool any=false;
    while(*s>='0'&&*s<='9'){any=true;ip=ip*10u+(uint32_t)(*s-'0');s++;}
    if(*s=='.'){s++;while(*s>='0'&&*s<='9'){any=true;if(fs<1000000u){fp=fp*10u+(uint32_t)(*s-'0');fs*=10u;}s++;}}
    if(!any||*s!='\0')return false;
    float v=(float)ip+(float)fp/(float)fs;
    *out=neg?-v:v;
    return true;
}
static void terminal_lower(char *s){for(;s&&*s;s++)if(*s>='A'&&*s<='Z')*s=(char)(*s-'A'+'a');}

static void terminal_help(void){
    terminal_send_text("Commands:\nREAD help fw status values encoder|enc config|mcconf tuning faults perf detect\nCTRL set duty X | current A | current_rel X | brake A | handbrake A | rpm ERPM | pos 0..360 | steer -30..30 | id A PHASE | openloop A ERPM | stop [all]\n");
    terminal_send_text("Commands: CFG: set sensor encoder|hall | invert 0|1 | current_limit A | input_current MIN MAX | erpm_limit MIN MAX | poles N | gear R | encoder_counts N | encoder_ratio R | encoder_offset DEG | encoder_invert 0|1 | pos_kp/pos_ki/pos_kd/pos_kd_proc V | speed_kp/speed_ki/speed_kd V | speed_ramp ERPM_S | speed_src 0PLL|1FAST | decoupling 0OFF|1CROSS|2BEMF|3BOTH | current_kp/current_ki V. FAULT: faults | faults clear|reset | faults_clear | faults_reset. SAVE: save mcconf|steering | load mcconf | defaults [save]\n");
    terminal_send_text("Commands: DETECT hall [A] | encoder [START_A] | all [LOSS MIN_IN MAX_IN OPENRPM SLERPM] | status|cancel | home; alias foc_encoder_detect. Detect Encoder LEFT: electrical ABI detect + 2x sweep hard-stop kiri/kanan + simpan span. Detect All: R/L/flux kedua motor + sensor commissioning; tidak mengubah hard-stop/span steering. RIGHT Hall-only. rpm=ERPM, A=amp, rel=-1..1.\n");
}

static void terminal_values(bool second){
    mc_values v;get_values_normalized(second,&v);char o[260];
    snprintf(o,sizeof(o),"id=%u fault=%u(%s) Vin=%.2f erpm=%.0f duty=%.3f Im=%.2f Iin=%.2f Id=%.2f Iq=%.2f Vd=%.2f Vq=%.2f pos=%.2f hall=%u state=%u mode=%u\n",
        (unsigned)v.vesc_id,(unsigned)v.fault_code,vesc_fault_name(v.fault_code),(double)v.v_in,(double)v.rpm,(double)v.duty_now,
        (double)v.current_motor,(double)v.current_in,(double)v.id,(double)v.iq,(double)v.vd,(double)v.vq,
        (double)v.position,(unsigned)mcpwm_foc_get_motor_const(second)->m_hall_state,(unsigned)mc_interface_get_state_motor(second),(unsigned)mcpwm_foc_get_motor_const(second)->m_control_mode);
    terminal_send_text(o);
}
static void terminal_detect_status(void){
    char o[280];uint8_t mi=s_detect_all.motor_index<2u?s_detect_all.motor_index:0u;
    snprintf(o,sizeof(o),"detect active=%u stage=%u motor=%u n=%lu detail=%d hall=%u:%u pass=%u deg=%d R=%ldmOhm L=%lduH flux=%ldmWb fault=%u/%u\n",
        (unsigned)s_detect_all.active,(unsigned)s_detect_all.stage,(unsigned)s_detect_all.motor_index,(unsigned long)s_detect_all.sample_n,
        (int)s_detect_all_last_detail,(unsigned)s_hall_detect.active,(unsigned)s_hall_detect.second,(unsigned)s_hall_detect.pass,(int)s_hall_detect.degree,
        (long)(s_detect_all.r[mi]*1000.0f),(long)(s_detect_all.l[mi]*1000000.0f),(long)(s_detect_all.flux[mi]*1000.0f),
        (unsigned)mcpwm_foc_get_motor_const(false)->m_fault,(unsigned)mcpwm_foc_get_motor_const(true)->m_fault);terminal_send_text(o);
}
static void terminal_cancel_detect(void){
    if(s_detect_all.active){detect_all_release_all();detect_all_restore_backups();memset(&s_detect_all,0,sizeof(s_detect_all));}
    if(s_hall_detect.active){bool r=s_hall_detect.second!=0u;mc_interface_select_motor_thread(r?2:1);mc_interface_release_motor();mcpwm_foc_vesc_override_clear(r);memset(&s_hall_detect,0,sizeof(s_hall_detect));}
    app_vesc_disable_output(0);mc_interface_select_motor_thread(1);
}

/* Return 1=changed, 0=unknown, -1=bad value. Numeric limits match the fixed
 * point storage so Terminal never silently truncates a PID/config value. */
static int terminal_cfg_one(mc_configuration *c,bool second,const char *k,const char *sv){
    float v;if(!c||!k||!sv)return -1;
    if(!strcmp(k,"sensor")){
        if(!strcmp(sv,"hall")){c->m_sensor_port_mode=SENSOR_PORT_MODE_HALL;c->sensor_mode=SENSOR_MODE_SENSORED;c->foc_sensor_mode=FOC_SENSOR_MODE_HALL;return 1;}
        if(!second&&!strcmp(sv,"encoder")){c->m_sensor_port_mode=SENSOR_PORT_MODE_ABI;c->sensor_mode=SENSOR_MODE_SENSORED;c->foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;c->m_encoder_counts=MCCONF_ENCODER_COUNTS_DEFAULT;c->si_motor_poles=2u*MCCONF_POLE_PAIRS_LEFT;c->foc_encoder_ratio=MCCONF_POLE_PAIRS_LEFT;return 1;}return -1;
    }
    if(!terminal_float(sv,&v))return -1;
    long i=(long)v;
    if(!strcmp(k,"invert")){if(v!=(float)i||(i!=0&&i!=1))return -1;c->m_invert_direction=i!=0;return 1;}
    if(!strcmp(k,"encoder_invert")){if(second||v!=(float)i||(i!=0&&i!=1))return -1;c->foc_encoder_inverted=i!=0;return 1;}
    if(!strcmp(k,"poles")){if(v!=(float)i||i<2||i>254||(i&1))return -1;c->si_motor_poles=(uint8_t)i;return 1;}
    if(!strcmp(k,"encoder_counts")){if(second||v!=(float)i||i<4||i>65536)return -1;c->m_encoder_counts=(uint32_t)i;return 1;}
    if(!strcmp(k,"current_limit")){if(v<0.1f||v>I_MOT_MAX)return -1;c->l_current_max=v;c->l_current_min=-v;return 1;}
    if(!strcmp(k,"gear")){if(v<0.01f||v>1000.0f)return -1;c->si_gear_ratio=v;return 1;}
    if(!strcmp(k,"encoder_ratio")){if(second||v<0.01f||v>MCCONF_ENCODER_RATIO_MAX)return -1;c->foc_encoder_ratio=v;return 1;}
    if(!strcmp(k,"encoder_offset")){if(second||fabsf(v)>100000.0f)return -1;c->foc_encoder_offset=v;return 1;}
    if(!strncmp(k,"pos_k",5)){
        if(!strcmp(k+5,"d_proc")){if(v<0||v>10.0f)return -1;c->p_pid_kd_proc=v;return 1;}
        if(v<0||v>65.535f||k[6])return -1;
        if(k[5]=='p')c->p_pid_kp=v;else if(k[5]=='i')c->p_pid_ki=v;else if(k[5]=='d')c->p_pid_kd=v;else return -1;return 1;
    }
    if(!strncmp(k,"speed_k",7)){if(v<0||v>65535.0f/MCCONF_SPEED_GAIN_SCALE||k[8])return -1;if(k[7]=='p')c->s_pid_kp=v;else if(k[7]=='i')c->s_pid_ki=v;else if(k[7]=='d')c->s_pid_kd=v;else return -1;return 1;}
    if(!strcmp(k,"speed_ramp")){if(v<100.0f||v>75000.0f)return -1;c->s_pid_ramp_erpms_s=v;return 1;}
    if(!strcmp(k,"speed_src")){if(v!=(float)i||i<0||i>1)return -1;c->s_pid_speed_source=(S_PID_SPEED_SRC)i;return 1;}
    if(!strcmp(k,"decoupling")){if(v!=(float)i||i<0||i>3)return -1;c->foc_cc_decoupling=(mc_foc_cc_decoupling_mode)i;return 1;}
    if(!strncmp(k,"current_k",9)){if(k[10]||v<0)return -1;if(k[9]=='p'){if(v>65535.0f/1536.0f)return -1;c->foc_current_kp=v;}else if(k[9]=='i'){if(v>65535.0f/4.608f)return -1;c->foc_current_ki=v;}else return -1;return 1;}
    return 0;
}

static void process_terminal_command(bool second,const uint8_t *data,uint16_t len){
    char line[112];uint16_t n=len>=sizeof(line)?sizeof(line)-1u:len;if(n)memcpy(line,data,n);line[n]='\0';
    while(n&& (line[n-1]=='\r'||line[n-1]=='\n'||line[n-1]==' '||line[n-1]=='\t'))line[--n]='\0';
    char *a[8];int ac=0;char *t=strtok(line," \t");while(t&&ac<8){a[ac++]=t;t=strtok(NULL," \t");}if(!ac)return;terminal_lower(a[0]);
    mc_interface_select_motor_thread(second?2:1);mcpwm_foc_motor_t *m=mcpwm_foc_get_motor(second);const mc_configuration *cc=(const mc_configuration *)mc_interface_get_configuration_motor(second);char o[420];

    if(!strcmp(a[0],"help")||!strcmp(a[0],"?")){terminal_help();return;}
    if(!strcmp(a[0],"fw")){snprintf(o,sizeof(o),"%s FW6.00 id=%u role=%s sensor=%s\n",second?"motor_right":"motor_left",second?2u:1u,second?"drive":"steer",second?"Hall":(cc->m_sensor_port_mode==SENSOR_PORT_MODE_ABI?"ABI":"Hall"));terminal_send_text(o);return;}
    if(!strcmp(a[0],"faults_clear")||!strcmp(a[0],"faults_reset")||!strcmp(a[0],"reset_faults")||
       (!strcmp(a[0],"reset")&&ac>1&&!strcmp(a[1],"faults"))||
       (!strcmp(a[0],"faults")&&ac>1&&(!strcmp(a[1],"clear")||!strcmp(a[1],"reset")))){
        mcpwm_foc_clear_faults();
        terminal_send_text("OK all motor faults reset; bridges remain released\n");
        return;
    }
    if(!strcmp(a[0],"status")||!strcmp(a[0],"values")||!strcmp(a[0],"faults")){terminal_values(second);return;}
    if(!strcmp(a[0],"encoder")||!strcmp(a[0],"enc")){
        if(second){terminal_send_text("RIGHT Hall-only\n");return;}int32_t sp=mcpwm_foc_steering_span_counts(),safe=mcpwm_foc_steering_safe_span_counts();
        float p360=(mc_interface_get_steering_deg()-MCCONF_STEERING_POS_MIN_DEG)*360.0f/(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);if(p360<0.0f){p360=0.0f;}
        if(p360>360.0f){p360=360.0f;}
        snprintf(o,sizeof(o),"raw=%lu pos=%ld target=%ld measured_span=%ld safe_span=%ld pos360=%.1f center=%ld deg=%ldm sync=%u cfg=%u cal=%u homed=%u foc_enc_inv=%u logical_inv=%u counts=%lu ratio=%.3f off=%.2f\n",
            (unsigned long)m->m_encoder_raw_count,(long)mcpwm_foc_get_position_user_counts(false),(long)mcpwm_foc_get_position_target_user_counts(false),(long)sp,(long)safe,(double)p360,0L,(long)(mc_interface_get_steering_deg()*1000.0f),
            (unsigned)mcpwm_foc_encoder_is_synced(false),(unsigned)m->m_encoder_configured,(unsigned)mc_interface_steering_calibration_valid(),(unsigned)mcpwm_foc_steering_is_homed(),(unsigned)m->m_conf.foc_encoder_inverted,(unsigned)mc_interface_steering_logical_inverted(),(unsigned long)m->m_conf.m_encoder_counts,(double)m->m_conf.foc_encoder_ratio,(double)m->m_conf.foc_encoder_offset);terminal_send_text(o);return;
    }
    if(!strcmp(a[0],"steering")){
        if(second){terminal_send_text("ERR LEFT steering only\n");return;}
        const char *sub=ac>1?a[1]:"status"; terminal_lower((char *)sub);
        if(!strcmp(sub,"status")){int32_t n1=0,p1=0,n2=0,p2=0,s1=0,s2=0,tol=0;mc_interface_get_steering_span_diag(&n1,&p1,&n2,&p2,&s1,&s2,&tol);float p360=(mc_interface_get_steering_deg()-MCCONF_STEERING_POS_MIN_DEG)*360.0f/(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);if(p360<0.0f){p360=0.0f;}
        if(p360>360.0f){p360=360.0f;}snprintf(o,sizeof(o),"steering cal=%u home=%u sync=%u logical_inv=%u foc_enc_inv=%u measured_span=%ld safe_span=%ld deg=%.3f pos360=%.1f raw=%lu count=%ld sweep1=%ld/%ld span1=%ld sweep2=%ld/%ld span2=%ld tol=%ld\n",(unsigned)mc_interface_steering_calibration_valid(),(unsigned)mcpwm_foc_steering_is_homed(),(unsigned)mcpwm_foc_encoder_is_synced(false),(unsigned)mc_interface_steering_logical_inverted(),(unsigned)m->m_conf.foc_encoder_inverted,(long)mcpwm_foc_steering_span_counts(),(long)mcpwm_foc_steering_safe_span_counts(),(double)mc_interface_get_steering_deg(),(double)p360,(unsigned long)m->m_encoder_raw_count,(long)mcpwm_foc_get_position_user_counts(false),(long)n1,(long)p1,(long)s1,(long)n2,(long)p2,(long)s2,(long)tol);terminal_send_text(o);return;}
        if(!strcmp(sub,"center")||!strcmp(sub,"zero")){terminal_send_text(mc_interface_steering_set_current_as_center()?"OK current steering position is now POS180/0deg\n":"ERR steering center requires valid span + synced encoder\n");return;}
        if(!strcmp(sub,"reset")){terminal_send_text(mc_interface_reset_steering_calibration()?"OK steering span reset; electrical encoder config preserved\n":"ERR steering reset\n");return;}
        if(!strcmp(sub,"invert")&&ac>2){float x=0.0f;if(!terminal_float(a[2],&x)||(x!=0.0f&&x!=1.0f)){terminal_send_text("ERR steering invert 0|1\n");return;}terminal_send_text(mc_interface_set_steering_logical_inverted(x>0.5f)?"OK steering logical mapping saved\n":"ERR steering invert requires valid span\n");return;}
        terminal_send_text("ERR steering status|center|zero|reset|invert 0|1\n");return;
    }
    if(!strcmp(a[0],"config")||!strcmp(a[0],"mcconf")){snprintf(o,sizeof(o),"sensor=%u/%u inv=%u poles=%u gear=%.2f I=%.1f/%.1f Iin=%.1f/%.1f erpm=%.0f/%.0f R=%.4f L=%.0fuH flux=%.2fmWb dec=%u speed_src=%u\n",(unsigned)cc->m_sensor_port_mode,(unsigned)cc->foc_sensor_mode,(unsigned)cc->m_invert_direction,(unsigned)cc->si_motor_poles,(double)cc->si_gear_ratio,(double)cc->l_current_min,(double)cc->l_current_max,(double)cc->l_in_current_min,(double)cc->l_in_current_max,(double)cc->l_min_erpm,(double)cc->l_max_erpm,(double)cc->foc_motor_r,(double)(cc->foc_motor_l*1e6f),(double)(cc->foc_motor_flux_linkage*1e3f),(unsigned)cc->foc_cc_decoupling,(unsigned)cc->s_pid_speed_source);terminal_send_text(o);return;}
    if(!strcmp(a[0],"tuning")){snprintf(o,sizeof(o),"current %.6f %.3f | speed %.6f %.6f %.6f ramp=%.0fERPM/s | pos %.4f %.4f %.4f kdproc %.6f\n",(double)cc->foc_current_kp,(double)cc->foc_current_ki,(double)cc->s_pid_kp,(double)cc->s_pid_ki,(double)cc->s_pid_kd,(double)cc->s_pid_ramp_erpms_s,(double)cc->p_pid_kp,(double)cc->p_pid_ki,(double)cc->p_pid_kd,(double)cc->p_pid_kd_proc);terminal_send_text(o);return;}
    if(!strcmp(a[0],"perf")){
        if(ac>1&&!strcmp(a[1],"reset")){
            foc_isr_cycles_max=0u; foc_isr_deadline_miss_count=0u;
            foc_prof_pre_max_cycles=foc_prof_control_max_cycles=foc_prof_post_max_cycles=0u;
            foc_prof_sensor_max_cycles=foc_prof_current_max_cycles=foc_prof_regulator_max_cycles=foc_prof_svpwm_max_cycles=0u;
        }
        snprintf(o,sizeof(o),"isr=%lu/%lu miss=%lu pre=%lu ctrl=%lu post=%lu sensor=%lu current=%lu reg=%lu svpwm=%lu overrun=%lu rxdrop=%lu txdrop=%lu gap=%lums\n",
            (unsigned long)mcpwm_foc_get_isr_cycles(),(unsigned long)mcpwm_foc_get_isr_cycles_max(),
            (unsigned long)foc_isr_deadline_miss_count,(unsigned long)foc_prof_pre_max_cycles,
            (unsigned long)foc_prof_control_max_cycles,(unsigned long)foc_prof_post_max_cycles,
            (unsigned long)foc_prof_sensor_max_cycles,(unsigned long)foc_prof_current_max_cycles,
            (unsigned long)foc_prof_regulator_max_cycles,(unsigned long)foc_prof_svpwm_max_cycles,
            (unsigned long)m->m_overrun_count,(unsigned long)s_rx_queue_drop,(unsigned long)s_tx_queue_drop,
            (unsigned long)s_process_gap_max_ms);terminal_send_text(o);return;}


    if(!strcmp(a[0],"trace")){
        const char *sub=ac>1?a[1]:"meta"; terminal_lower((char *)sub);
        if(!strcmp(sub,"clear")){mcpwm_foc_trace_clear();terminal_send_text("OK trace cleared\n");return;}
        if(!strcmp(sub,"freeze")){mcpwm_foc_trace_freeze();terminal_send_text("OK trace frozen\n");return;}
        if(!strcmp(sub,"meta")){
            mcpwm_foc_trace_meta_t tm;mcpwm_foc_trace_get_meta(&tm);
            snprintf(o,sizeof(o),"trace write=%lu frozen=%u trig_motor=%u trig_fault=%u count=%u head=%u cap=%u sample=%u\n",
                (unsigned long)tm.write_count,(unsigned)tm.frozen,(unsigned)tm.trigger_motor,(unsigned)tm.trigger_fault,
                (unsigned)tm.count,(unsigned)tm.head,(unsigned)tm.capacity,(unsigned)tm.sample_size);
            terminal_send_text(o);return;
        }
        if(!strcmp(sub,"sample")&&ac>2){
            float xf=0.0f;if(!terminal_float(a[2],&xf)||xf<0.0f||xf>255.0f){terminal_send_text("ERR trace sample 0..255\n");return;}
            const uint8_t ix=(uint8_t)xf;mcpwm_foc_trace_sample_t ts;
            if(!mcpwm_foc_trace_read(ix,&ts)){terminal_send_text("ERR trace sample unavailable\n");return;}
            snprintf(o,sizeof(o),"trace %u tick=%lu cyc=%u slot=%u ev=%u L=id:%d iq:%d idt:%d iqt:%d vd:%d vq:%d e:%d f:%u q:%u R=id:%d iq:%d idt:%d iqt:%d vd:%d vq:%d e:%d f:%u q:%u vin=%u\n",
                (unsigned)ix,(unsigned long)ts.pwm_tick,(unsigned)ts.isr_cycles,(unsigned)ts.control_slot,(unsigned)ts.event_bits,
                (int)ts.left_id_q4,(int)ts.left_iq_q4,(int)ts.left_id_set_q4,(int)ts.left_iq_set_q4,(int)ts.left_vd,(int)ts.left_vq,(int)ts.left_erpm,(unsigned)ts.left_fault,(unsigned)ts.left_quality,
                (int)ts.right_id_q4,(int)ts.right_iq_q4,(int)ts.right_id_set_q4,(int)ts.right_iq_set_q4,(int)ts.right_vd,(int)ts.right_vq,(int)ts.right_erpm,(unsigned)ts.right_fault,(unsigned)ts.right_quality,(unsigned)ts.vin_adc);
            terminal_send_text(o);return;
        }
        terminal_send_text("ERR trace clear|freeze|meta|sample N\n");return;
    }

    bool alias_enc=!strcmp(a[0],"foc_encoder_detect");
    if(!strcmp(a[0],"detect")||alias_enc){
        const char *sub=alias_enc?"encoder":(ac>1?a[1]:"status");int base=alias_enc?1:2;
        if(!strcmp(sub,"status")){terminal_detect_status();return;}
        if(!strcmp(sub,"cancel")){terminal_cancel_detect();terminal_send_text("OK detect cancelled\n");return;}
        if(s_detect_all.active||s_hall_detect.active){terminal_send_text("ERR detect busy\n");return;}
        if(!strcmp(sub,"hall")){float x=3.0f;if(ac>base&&!terminal_float(a[base],&x)){terminal_send_text("ERR current\n");return;}if(x<0.3f||x>I_MOT_MAX){snprintf(o,sizeof(o),"ERR 0.3..%dA\n",I_MOT_MAX);terminal_send_text(o);return;}mcpwm_foc_hall_detect_start(second,x);terminal_send_text("OK Hall detect started; poll 'detect'\n");return;}
        if(!strcmp(sub,"encoder")){if(second){terminal_send_text("ERR RIGHT Hall-only\n");return;}float x=MCCONF_STEERING_DETECT_CURRENT_START_A;if(ac>base&&!terminal_float(a[base],&x)){terminal_send_text("ERR current\n");return;}if(x<0.3f||x>I_MOT_MAX){snprintf(o,sizeof(o),"ERR 0.3..%dA\n",I_MOT_MAX);terminal_send_text(o);return;}float off=1001,rat=0;bool inv=false;int32_t p0=0,p1=0,sp=0;terminal_send_text("Steering span-only detect running...\n");bool ok=mc_interface_steering_detect_calibrate(x,&off,&rat,&inv,&p0,&p1,&sp);snprintf(o,sizeof(o),"%s off=%.2f ratio=%.3f logical_inv=%u stops=%ld/%ld span=%ld center=180\n",ok?"PASS":"FAIL",(double)off,(double)rat,(unsigned)inv,(long)p0,(long)p1,(long)sp);terminal_send_text(o);return;}
        if(!strcmp(sub,"all")){if(second){terminal_send_text("ERR start from LEFT/ID1\n");return;}float loss=50,minin=cc->l_in_current_min,maxin=cc->l_in_current_max,ol=cc->foc_openloop_rpm,sl=cc->foc_sl_erpm;if(ac-base==5){if(!terminal_float(a[base],&loss)||!terminal_float(a[base+1],&minin)||!terminal_float(a[base+2],&maxin)||!terminal_float(a[base+3],&ol)||!terminal_float(a[base+4],&sl)){terminal_send_text("ERR args\n");return;}}else if(ac!=base){terminal_send_text("ERR detect all [LOSS MIN MAX OPENRPM SLERPM]\n");return;}uint8_t d[21];int32_t k=0;d[k++]=1;buffer_append_float32(d,loss,1e3f,&k);buffer_append_float32(d,minin,1e3f,&k);buffer_append_float32(d,maxin,1e3f,&k);buffer_append_float32(d,ol,1e3f,&k);buffer_append_float32(d,sl,1e3f,&k);conf_general_detect_apply_all_foc_can_start(d,sizeof(d));terminal_send_text("OK Detect-All started; poll 'detect'\n");return;}
        terminal_send_text("ERR detect status|hall|encoder|all|cancel\n");return;
    }
    if(!strcmp(a[0],"home")){if(second||cc->m_sensor_port_mode!=SENSOR_PORT_MODE_ABI){terminal_send_text("ERR LEFT encoder only\n");return;}terminal_send_text(mc_interface_steering_boot_home()?"PASS home\n":"FAIL home\n");return;}
    if(!strcmp(a[0],"stop")){if(ac>1&&!strcmp(a[1],"all")){mc_interface_select_motor_thread(1);mc_interface_release_motor();mcpwm_foc_vesc_override_clear(false);mc_interface_select_motor_thread(2);mc_interface_release_motor();mcpwm_foc_vesc_override_clear(true);mc_interface_select_motor_thread(second?2:1);}else{mc_interface_release_motor();mcpwm_foc_vesc_override_clear(second);}terminal_send_text("OK stopped\n");return;}
    if(!strcmp(a[0],"save")&&ac>1){if(!strcmp(a[1],"mcconf")){terminal_send_text(mc_interface_store_configuration_motor(second)?"OK saved\n":"ERR save\n");return;}if(!second&&!strcmp(a[1],"steering")){terminal_send_text(mc_interface_store_steering_calibration()?"OK saved\n":"ERR steering\n");return;}}
    if(!strcmp(a[0],"load")&&ac>1&&!strcmp(a[1],"mcconf")){terminal_send_text(mc_interface_load_configuration_motor(second)?"OK loaded\n":"ERR load\n");return;}
    if(!strcmp(a[0],"defaults")){bool sv=ac>1&&!strcmp(a[1],"save");mc_interface_restore_default_motor(second,sv);terminal_send_text(sv?"OK defaults saved\n":"OK defaults RAM\n");return;}
    if(!strcmp(a[0],"set")){
        if(ac<3){terminal_send_text("ERR set key value\n");return;}if(s_detect_all.active||hall_detect_motor_locked(second)){terminal_send_text("ERR detect busy\n");return;}terminal_lower(a[1]);float x,y;
        if(!strcmp(a[1],"duty")&&terminal_float(a[2],&x)&&x>=-1&&x<=1){touch_motor(second);mc_interface_set_duty(x);goto setok;}
        if(!strcmp(a[1],"current")&&terminal_float(a[2],&x)&&fabsf(x)<=I_MOT_MAX){touch_motor(second);mc_interface_set_current(x);goto setok;}
        if(!strcmp(a[1],"current_rel")&&terminal_float(a[2],&x)&&x>=-1&&x<=1){touch_motor(second);mc_interface_set_current_rel(x);goto setok;}
        if(!strcmp(a[1],"brake")&&terminal_float(a[2],&x)&&x>=0&&x<=I_MOT_MAX){touch_motor(second);mc_interface_set_brake_current(x);goto setok;}
        if(!strcmp(a[1],"handbrake")&&terminal_float(a[2],&x)&&x>=0&&x<=I_MOT_MAX){touch_motor(second);mc_interface_set_handbrake(x);goto setok;}
        if(!strcmp(a[1],"rpm")&&terminal_float(a[2],&x)&&x>=cc->l_min_erpm&&x<=cc->l_max_erpm){touch_motor(second);mc_interface_set_pid_speed(x);goto setok;}
        if(!strcmp(a[1],"pos")&&terminal_float(a[2],&x)){if(!second){if(x<0||x>360)goto setbad;y=MCCONF_STEERING_POS_MIN_DEG+(x/360.0f)*(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);touch_motor(false);if(!mc_interface_set_steering_deg(y))goto setbad;}else{touch_motor(true);mc_interface_set_pid_pos(x);}goto setok;}
        if(!strcmp(a[1],"steer")&&!second&&terminal_float(a[2],&x)&&x>=MCCONF_STEERING_POS_MIN_DEG&&x<=MCCONF_STEERING_POS_MAX_DEG){touch_motor(false);if(!mc_interface_set_steering_deg(x))goto setbad;goto setok;}
        if(!strcmp(a[1],"id")&&ac>3&&terminal_float(a[2],&x)&&terminal_float(a[3],&y)&&x>=0&&x<=I_MOT_MAX){touch_motor(second);mc_interface_set_openloop_phase(x,y);goto setok;}
        if(!strcmp(a[1],"openloop")&&ac>3&&terminal_float(a[2],&x)&&terminal_float(a[3],&y)&&fabsf(x)<=I_MOT_MAX&&fabsf(y)<=MCCONF_L_MAX_ERPM){touch_motor(second);mc_interface_set_openloop_current(x,y);goto setok;}
        if(!strcmp(a[1],"input_current")&&ac>3&&terminal_float(a[2],&x)&&terminal_float(a[3],&y)&&x<=-0.1f&&x>=-I_DC_MAX&&y>=0.1f&&y<=I_DC_MAX){mc_configuration c=*cc;c.l_in_current_min=x;c.l_in_current_max=y;mc_interface_release_motor();mc_interface_set_configuration(&c);terminal_send_text("OK RAM; save mcconf\n");return;}
        if(!strcmp(a[1],"erpm_limit")&&ac>3&&terminal_float(a[2],&x)&&terminal_float(a[3],&y)&&x<0&&y>0&&x>=MCCONF_L_MIN_ERPM&&y<=MCCONF_L_MAX_ERPM){mc_configuration c=*cc;c.l_min_erpm=x;c.l_max_erpm=y;mc_interface_release_motor();mc_interface_set_configuration(&c);terminal_send_text("OK RAM; save mcconf\n");return;}
        {mc_configuration c=*cc;int r=terminal_cfg_one(&c,second,a[1],a[2]);if(r==1){mc_interface_release_motor();mc_interface_set_configuration(&c);terminal_send_text("OK RAM; save mcconf\n");return;}if(r<0)goto setbad;}
setbad: terminal_send_text("ERR set value/syntax; type help\n");return;
setok: terminal_send_text("OK set\n");return;
    }
    terminal_send_text("Unknown. Type help.\n");
}

static void process_command(const uint8_t *p, uint16_t len, bool second) {
    if (!p || len == 0u) return;
    const COMM_PACKET_ID id = (COMM_PACKET_ID)p[0];
    const uint8_t *d = p + 1;
    const uint16_t n = (uint16_t)(len - 1u);
    int32_t k = 0;
    mc_interface_select_motor_thread(second ? 2 : 1);
#ifdef STM32F103xE
    *(volatile uint32_t *)F103_RESET_STAGE_ADDR = 0xC0000000u | ((uint32_t)(second ? 1u : 0u) << 8) | ((uint32_t)id & 0xFFu);
#endif

    switch (id) {
    case COMM_FW_VERSION:
        ++s_fw_version_count;
        reply_fw_version(second);
        break;
    case COMM_ERASE_NEW_APP: {
        uint8_t reply[2] = {COMM_ERASE_NEW_APP, 0u};
        if (!second && n >= 4u) {
            int32_t ui = 0;
            const uint32_t fw_size = buffer_get_uint32(d, &ui);
            reply[1] = f103_fw_erase_staging(fw_size) ? 1u : 0u;
        }
        uart_send_payload(reply, sizeof(reply));
        break;
    }
    case COMM_WRITE_NEW_APP_DATA: {
        uint8_t reply[6] = {COMM_WRITE_NEW_APP_DATA, 0u, 0u, 0u, 0u, 0u};
        if (!second && n >= 4u) {
            int32_t ui = 0;
            const uint32_t offset = buffer_get_uint32(d, &ui);
            const uint32_t data_len = (uint32_t)n - 4u;
            reply[1] = f103_fw_write_staging(offset, d + 4u, data_len) ? 1u : 0u;
            reply[2] = (uint8_t)(offset >> 24);
            reply[3] = (uint8_t)(offset >> 16);
            reply[4] = (uint8_t)(offset >> 8);
            reply[5] = (uint8_t)offset;
        }
        uart_send_payload(reply, sizeof(reply));
        break;
    }
    case COMM_JUMP_TO_BOOTLOADER:
        /* Production field updates use HB_CUSTOM_BOOT_HANDOFF (magic + version + ACK).
         * Do not let an unsolicited/legacy VESC Tool probe turn a healthy motor APP into
         * an indefinitely resident bootloader. The resident bootloader still accepts
         * COMM_JUMP_TO_BOOTLOADER to finalize an already-authenticated stream. */
        (void)second;
        break;
    case COMM_GET_VALUES:
        reply_values(second, false, d, n);
        break;
    case COMM_GET_VALUES_SELECTIVE:
        reply_values(second, true, d, n);
        break;
    case COMM_GET_VALUES_SETUP:
        reply_values_setup(second, false, d, n);
        break;
    case COMM_GET_VALUES_SETUP_SELECTIVE:
        reply_values_setup(second, true, d, n);
        break;
    case COMM_GET_STATS:
        reply_stats();
        break;
    case COMM_SET_DETECT:
        if (n >= 1u) {
            const uint8_t raw_mode = d[0];
            if (raw_mode <= (uint8_t)DISP_POS_MODE_HALL_OBSERVER_ERROR) {
                s_display_second = second ? 1u : 0u;
                s_display_pos_mode = (disp_pos_mode)raw_mode;
                s_display_prev_ms = HAL_GetTick();
            } else {
                s_display_pos_mode = DISP_POS_MODE_NONE;
            }
        }
        break;
    case COMM_SET_DUTY:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float duty = (float)buffer_get_int32(d, &k) / 100000.0f;
            touch_motor(second); mc_interface_set_duty(duty);
        }
        break;
    case COMM_SET_CURRENT:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_current(current);
        }
        break;
    case COMM_SET_CURRENT_REL:
        set_current_relative(second, d, n);
        break;
    case COMM_SET_CURRENT_BRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_brake_current(current);
        }
        break;
    case COMM_SET_HANDBRAKE:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float current = (float)buffer_get_int32(d, &k) / 1000.0f;
            touch_motor(second); mc_interface_set_handbrake(current);
        }
        break;
    case COMM_SET_RPM:
        if (hall_detect_motor_locked(second)) break;
        if (n >= 4u) {
            const float rpm = (float)buffer_get_int32(d, &k);
            touch_motor(second); mc_interface_set_pid_speed(rpm);
        }
        break;
    case COMM_SET_POS:
        if (hall_detect_motor_locked(second)) break;
        if(n>=4u){
            float pos=(float)buffer_get_int32(d,&k)/1000000.0f;
            /* Wire-compatible VESC Tool position command. LEFT exposes a raw
             * normalized 0..360 steering-actuator coordinate and maps it to the
             * calibrated encoder-count span. Vehicle physical degrees are not
             * defined here; ROS/ROS Web owns that calibration. RIGHT retains the
             * stock single-turn PID-position path. */
            if(!second){
                if(pos<0.0f)pos=0.0f;
                if(pos>360.0f)pos=360.0f;
                pos=MCCONF_STEERING_POS_MIN_DEG +
                    (pos/360.0f)*(MCCONF_STEERING_POS_MAX_DEG-MCCONF_STEERING_POS_MIN_DEG);
            }
            touch_motor(second);
            if(!second) (void)mc_interface_set_steering_deg(pos);
            else mc_interface_set_pid_pos(pos);
        }
        break;
    case COMM_ALIVE:
        touch_motor(second);
        break;
    case COMM_MOTOR_ESTOP: {
        /* Wire VESC 6.00: payload uint16 adalah lama ignore-input dalam ms.
         * E-stop selalu berlaku untuk kedua motor, termasuk paket forwarded. */
        uint16_t hold_ms=0u;
        if(n>=2u) hold_ms=buffer_get_uint16(d,&k);
        mcpwm_foc_estop_both(hold_ms);
        break;
    }
    case COMM_GET_MCCONF:
    case COMM_GET_MCCONF_DEFAULT:
        reply_mcconf(second, id);
        break;
    case COMM_SET_MCCONF:
        if (hall_detect_motor_locked(second)) break;
        set_mcconf(second, d, n);
        break;
    case COMM_SET_MCCONF_TEMP:
    case COMM_SET_MCCONF_TEMP_SETUP:
        set_mcconf_temp(second, id, d, n);
        break;
    case COMM_GET_MCCONF_TEMP:
        reply_mcconf_temp(second);
        break;
    case COMM_GET_BATTERY_CUT:
        reply_battery_cut(second);
        break;
    case COMM_SET_BATTERY_CUT:
        set_battery_cut(second, d, n);
        break;
    case COMM_GET_APPCONF:
    case COMM_GET_APPCONF_DEFAULT:
        reply_appconf(second, id);
        break;
    case COMM_GET_DECODED_PPM:
        reply_decoded_ppm();
        break;
    case COMM_GET_DECODED_ADC:
        reply_decoded_adc();
        break;
    case COMM_GET_DECODED_CHUK:
        reply_decoded_chuk();
        break;
    case COMM_SET_ODOMETER:
        /* Tombol Set Odometer VESC Tool tidak punya ACK. Simpan offset terhadap
         * trip Hall saat ini supaya pembacaan berikutnya terus bertambah. */
        if (n >= 4u) {
            const uint32_t requested = buffer_get_uint32(d, &k);
            mc_values ov;
            float speed_unused = 0.0f, dist_unused = 0.0f, dist_abs = 0.0f;
            get_values_normalized(second, &ov);
            setup_motion_values(second, &ov, &speed_unused, &dist_unused, &dist_abs);
            const float trip_f=dist_abs>=0.0f?dist_abs:0.0f;
            const uint32_t trip_u=trip_f>4294967040.0f?UINT32_MAX:(uint32_t)trip_f;
            s_odometer_offset_m[second ? 1u : 0u] = (int64_t)requested - (int64_t)trip_u;
        }
        break;
    case COMM_SET_APPCONF:
        set_appconf(second, d, n, true, COMM_SET_APPCONF);
        break;
    case COMM_SET_APPCONF_NO_STORE:
        set_appconf(second, d, n, false, COMM_SET_APPCONF_NO_STORE);
        break;
    case COMM_APP_DISABLE_OUTPUT:
        /* VESC Tool sends [forward_can][time_ms] before Detect All. This dual
         * board has no physical CAN bus, so one shared app-output gate covers
         * the local and virtual motor endpoints. */
        if(n>=5u){
            const uint8_t fwd_can=d[k++];
            const int32_t time_ms=buffer_get_int32(d,&k);
            (void)fwd_can;
            app_vesc_disable_output(time_ms);
        }
        break;
    case COMM_DETECT_MOTOR_R_L:
        if(!mcpwm_foc_measure_res_ind_f103_start(second)){
            uint8_t b[13]; int32_t bi=0; b[bi++]=COMM_DETECT_MOTOR_R_L;
            buffer_append_float32(b,0.0f,1e6f,&bi);
            buffer_append_float32(b,0.0f,1e3f,&bi);
            buffer_append_float32(b,0.0f,1e3f,&bi);
            uart_send_payload(b,(uint16_t)bi);
        }
        break;
    case COMM_DETECT_MOTOR_FLUX_LINKAGE: {
        if(n<16u){
            uint8_t b[5]; int32_t bi=0; b[bi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE;
            buffer_append_float32(b,0.0f,1e7f,&bi); uart_send_payload(b,(uint16_t)bi); break;
        }
        const float current=buffer_get_float32(d,1e3f,&k);
        const float min_erpm=buffer_get_float32(d,1e3f,&k);
        const float duty=buffer_get_float32(d,1e3f,&k);
        const float resistance=buffer_get_float32(d,1e6f,&k);
        if(!conf_general_measure_flux_linkage_start(second,current,duty,min_erpm,resistance)){
            uint8_t b[5]; int32_t bi=0; b[bi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE;
            buffer_append_float32(b,0.0f,1e7f,&bi); uart_send_payload(b,(uint16_t)bi);
        }
        break;
    }
    case COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP: {
        if(n<16u){
            uint8_t b[14]; int32_t bi=0; b[bi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
            buffer_append_float32(b,0.0f,1e7f,&bi); buffer_append_float32(b,-1.0f,1e6f,&bi);
            buffer_append_float32(b,-1.0f,1e6f,&bi); b[bi++]=0u; uart_send_payload(b,(uint16_t)bi); break;
        }
        const float current=buffer_get_float32(d,1e3f,&k);
        const float erpm_per_sec=buffer_get_float32(d,1e3f,&k);
        const float duty=buffer_get_float32(d,1e3f,&k);
        const float resistance=buffer_get_float32(d,1e6f,&k);
        float inductance=0.0f;
        if(n>=(uint16_t)(k+4)) inductance=buffer_get_float32(d,1e8f,&k);
        if(!(inductance>0.0f)){
            const mc_configuration *cc=(const mc_configuration *)mc_interface_get_configuration_motor(second);
            if(cc)inductance=cc->foc_motor_l;
        }
        if(!conf_general_measure_flux_linkage_openloop_start(second,current,duty,erpm_per_sec,resistance,inductance)){
            uint8_t b[14]; int32_t bi=0; b[bi++]=COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP;
            buffer_append_float32(b,0.0f,1e7f,&bi); buffer_append_float32(b,-1.0f,1e6f,&bi);
            buffer_append_float32(b,-1.0f,1e6f,&bi); b[bi++]=0u; uart_send_payload(b,(uint16_t)bi);
        }
        break;
    }
    case COMM_DETECT_ENCODER: {
#ifdef STM32F103xE
        *(volatile uint32_t *)F103_RESET_STAGE_ADDR = 0xC100001Bu;
#endif
        /* Full LEFT encoder/span detection is a commissioning-only operation.
         * Never allow a drive/joystick session to turn a malformed or stale
         * packet into a mechanical hard-stop sweep. */
        const mcpwm_foc_motor_t *drive=mcpwm_foc_get_motor_const(true);
        float drive_erpm=mcpwm_foc_get_erpm_motor(true);
        if(drive_erpm<0.0f)drive_erpm=-drive_erpm;
        const bool drive_busy=(drive && drive->m_control_mode!=CONTROL_MODE_NONE) ||
                              drive_erpm>(float)MCCONF_FAULT_RECOVERY_SAFE_ERPM;
        if(second || drive_busy){
            uint8_t reply[10]; int32_t ri=0; reply[ri++]=COMM_DETECT_ENCODER;
            buffer_append_float32(reply,1001.0f,1e6f,&ri);
            buffer_append_float32(reply,0.0f,1e6f,&ri);
            reply[ri++]=0u;
            uart_send_payload(reply,(uint16_t)ri);
            break;
        }
        float off=1001.0f, ratio=0.0f; bool inv=false;
        float current=MCCONF_STEERING_HOME_CURRENT_A;
        if(n>=4u) current=(float)buffer_get_int32(d,&k)/1000.0f;
        if(current<0.30f)current=0.30f;
        if(current>MCCONF_STEERING_CAL_CURRENT_MAX_A)current=MCCONF_STEERING_CAL_CURRENT_MAX_A;
        app_vesc_disable_output(60000);
#ifdef STM32F103xE
        *(volatile uint32_t *)F103_RESET_STAGE_ADDR = 0xC200001Bu;
#endif
        bool ok=false;
        int32_t raw_left=0,raw_right=0,span=0;
        if(!second){
            const mc_configuration backup=*mc_interface_get_configuration_motor(false);
            const int32_t old_span=mcpwm_foc_steering_span_counts();
            const bool old_cal=mcpwm_foc_steering_is_calibrated();
            const bool old_homed=mcpwm_foc_steering_is_homed();
            const bool old_logical_inv=mc_interface_steering_logical_inverted();
            float eoff=1001.0f, eratio=0.0f; bool einv=false;
            /* Project-specific standalone Detect Encoder is a superset of the
             * VESC command: first perform the electrical ABI phase/direction
             * probe, then use those detected electrical parameters while the
             * steering commissioning routine measures the real hard-stop span. */
            if(backup.m_sensor_port_mode==SENSOR_PORT_MODE_ABI &&
               mcpwm_foc_encoder_detect(current,false,&eoff,&eratio,&einv)){
                mc_configuration detected=backup;
                detected.motor_type=MOTOR_TYPE_FOC;
                detected.sensor_mode=SENSOR_MODE_SENSORED;
                detected.m_sensor_port_mode=SENSOR_PORT_MODE_ABI;
                detected.foc_sensor_mode=FOC_SENSOR_MODE_ENCODER;
                detected.foc_encoder_offset=eoff;
                detected.foc_encoder_ratio=eratio;
                detected.foc_encoder_inverted=einv;
                mc_interface_select_motor_thread(1);
                mc_interface_set_configuration(&detected);
                float span_off=0.0f,span_ratio=0.0f; bool logical_inv=false;
                ok=mc_interface_steering_detect_calibrate(current,&span_off,&span_ratio,&logical_inv,
                                                           &raw_left,&raw_right,&span);
                if(ok){
                    /* Standalone Detect Encoder changes real electrical ABI
                     * parameters. Persist them together with the already stored
                     * steering span before returning success; otherwise a reboot
                     * can silently restore the old inversion/ratio and make the
                     * geometric calibration unsafe. */
                    ok=mc_interface_store_configuration_motor(false);
                }
                if(ok){
                    /* Wire reply remains stock VESC electrical semantics. */
                    off=eoff; ratio=eratio; inv=einv;
                }else{
                    mc_configuration restore=backup;
                    mc_interface_set_configuration(&restore);
                    (void)mc_interface_store_configuration_motor(false);
                    (void)mc_interface_set_steering_logical_inverted(old_logical_inv);
                    if(old_cal && old_span!=0)
                        (void)mcpwm_foc_steering_set_span(old_span,old_homed);
                    else
                        mcpwm_foc_steering_clear_calibration();
                }
            }
        }
        app_vesc_disable_output(0);
        if(!ok){off=1001.0f;ratio=0.0f;inv=false;}
        uint8_t reply[10]; int32_t ri=0; reply[ri++]=COMM_DETECT_ENCODER;
        buffer_append_float32(reply,off,1e6f,&ri);
        buffer_append_float32(reply,ratio,1e6f,&ri);
        reply[ri++]=inv?1u:0u;
        uart_send_payload(reply,(uint16_t)ri);
        (void)raw_left; (void)raw_right; (void)span;
        break;
    }
    case COMM_DETECT_HALL_FOC:
        mcpwm_foc_hall_detect_command_start(second, d, n);
        break;
    case COMM_DETECT_APPLY_ALL_FOC:
        if(!second) conf_general_detect_apply_all_foc_can_start(d,n);
        break;
    case COMM_TERMINAL_CMD:
    case COMM_TERMINAL_CMD_SYNC:
        process_terminal_command(second, d, n);
        break;
    case COMM_SHUTDOWN:
        /* Board hoverboard tidak mempunyai power-latch VESC. Pertahankan
         * command tetap aman: lepaskan bridge/motor tanpa mematikan MCU/UART. */
        mc_interface_release_motor();
        break;
    case COMM_CUSTOM_APP_DATA:
        process_custom_app(second, d, n);
        break;
    case COMM_REBOOT:
        /* Sama seperti VESC: release bridge dahulu, lalu reset MCU. Host build
         * sengaja tidak mengeksekusi register Cortex-M agar unit test aman. */
        mc_interface_release_motor();
#ifdef STM32F103xE
        *(volatile uint32_t *)F103_RESET_REASON_ADDR = F103_RESET_REASON_REBOOT;
        __DSB();
        NVIC_SystemReset();
#endif
        break;
    default:
        /* Unsupported commands are intentionally ignored, as stock VESC commands.c does
         * for command IDs without a hardware implementation. */
        break;
    }
    mc_interface_select_motor_thread(1);
}

static bool probation_packet_allowed(const uint8_t *p, uint16_t len) {
    if (!s_probation) return true;
    if (!p || len == 0u) return false;
    const COMM_PACKET_ID id=(COMM_PACKET_ID)p[0];
    if (id==COMM_FW_VERSION) return true;
    if (id==COMM_CUSTOM_APP_DATA && len>=5u && p[1]==HB_CUSTOM_MAGIC0 && p[2]==HB_CUSTOM_MAGIC1 && p[3]==HB_CUSTOM_VERSION) {
        const uint8_t op=p[4];
        return op==HB_CUSTOM_GET_FW_UPDATE_STATE || op==HB_CUSTOM_GET_PLATFORM_INFO || op==HB_CUSTOM_BOOT_HANDOFF;
    }
    return false;
}

static void process_top_packet(const uint8_t *p, uint16_t len) {
    if (!p || len == 0u || !probation_packet_allowed(p,len)) return;
    const COMM_PACKET_ID id = (COMM_PACKET_ID)p[0];
    if (id == COMM_FORWARD_CAN) {
        if (len >= 3u && p[1] == VESC_SECOND_MOTOR_ID) {
            /* Exact dual-motor VESC semantic: forwarding to the virtual second CAN ID
             * selects motor thread 2 and executes the nested packet locally. */
            process_command(p + 2, (uint16_t)(len - 2u), true);
        }
        return;
    }
    if (id == COMM_PING_CAN) {
        uint8_t b[2] = {COMM_PING_CAN, VESC_SECOND_MOTOR_ID};
        uart_send_payload(b, sizeof(b));
        return;
    }
    process_command(p, len, false);
}

static void process_rt_mailboxes(void) {
    if (s_probation) {
        for (uint8_t mi=0u; mi<2u; ++mi) s_rt_cmd[mi].pending=0u;
        return;
    }
    /* RX DMA bytes are parsed only by usart3_rx_check() in main context. The
     * realtime mailbox therefore has one producer and one consumer in the same
     * context; masking the priority-0 FOC IRQ here only adds avoidable jitter. */
    for (uint8_t mi=0u; mi<2u; ++mi) {
        uint8_t p[5]; uint8_t have=0u;
        if (s_rt_cmd[mi].pending && s_rt_cmd[mi].len==5u) {
            memcpy(p,(const void *)s_rt_cmd[mi].payload,5u);
            s_rt_cmd[mi].pending=0u; have=1u;
        }
        if (have) process_command(p,5u,mi!=0u);
    }
}

void vesc_protocol_process_pending(void) {
    const uint32_t process_now_ms = HAL_GetTick();
    if (s_process_last_ms != 0u) {
        const uint32_t gap = process_now_ms - s_process_last_ms;
        if (gap > s_process_gap_max_ms) s_process_gap_max_ms = gap;
    }
    s_process_last_ms = process_now_ms;
    vesc_tx_service();
    process_rt_mailboxes();
    /* Process only a small bounded batch before returning to main(). The main
     * loop immediately calls usart3_rx_check() again, so DMA RX is drained
     * between batches instead of being ignored while a large telemetry backlog
     * is serialized. Two packets/pass keeps latency low without starving RX. */
    for (uint8_t processed = 0u; processed < 2u; ++processed) {
        uint16_t n = 0u;
        uint8_t slot = 0u;
        /* complete_frame() and this consumer both execute in main context.
         * Never hold off the 16-kHz DMA1_Channel1 IRQ while copying up to a
         * 700-byte configuration packet. */
        if (s_pending_count == 0u) break;
        slot = s_pending_tail;
        n = s_pending_len[slot];
        if (n > VESC_MAX_PAYLOAD) n = VESC_MAX_PAYLOAD;
        memcpy(s_process_payload, s_pending_payload[slot], n);
        s_pending_len[slot] = 0u;
        s_pending_tail = (uint8_t)((slot + 1u) % VESC_RX_QUEUE_DEPTH);
        s_pending_count--;
        s_link_last_ms = HAL_GetTick();
        process_top_packet(s_process_payload, n);
        process_rt_mailboxes();
        vesc_tx_service();
    }
    vesc_tx_service();
}
