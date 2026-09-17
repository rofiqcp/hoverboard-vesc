#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

#include "config.h"
#include "vesc/datatypes.h"

/* VESC-style configuration names, values inherited from the proven fixed-point
 * EFeru/hoverboard controller. Runtime FOC math remains integer/fixed-point. */
#define MCCONF_L_CURRENT_MAX                 15.0f
#define MCCONF_L_CURRENT_MIN                -15.0f
#define MCCONF_L_IN_CURRENT_MAX              15.0f
#define MCCONF_L_IN_CURRENT_MIN             -15.0f
/* Batas dinamis standar VESC. Nilai disiapkan di luar ISR lalu runtime hanya
 * memakai perbandingan, perkalian, dan shift integer agar ringan di F103. */
#define MCCONF_L_ERPM_START                    0.80f
#define MCCONF_L_DUTY_START                    1.00f
#define MCCONF_L_TEMP_ACCEL_DEC                0.15f
#define MCCONF_L_IN_CURRENT_MAP_START           0.90f
#define MCCONF_L_IN_CURRENT_MAP_FILTER          0.002f
/* Nilai konfigurasi VESC yang sebelumnya nol akibat memset. Battery cut
 * mengikuti batas baterai 10S pada firmware hardware masteran (3.50/3.37 V/cell). */
#define MCCONF_L_CURRENT_MAX_SCALE             1.0f
#define MCCONF_L_CURRENT_MIN_SCALE             1.0f
#define MCCONF_L_BATTERY_CUT_START            35.0f
#define MCCONF_L_BATTERY_CUT_END              33.7f
/* Derating regen dibuat sebelum hard over-voltage. Nilai ini masih aman untuk
 * bus 50 V dan dapat diubah dari VESC Tool sesuai pack baterai yang digunakan. */
#define MCCONF_L_BATTERY_REGEN_CUT_START       48.0f
#define MCCONF_L_BATTERY_REGEN_CUT_END         49.5f
#define MCCONF_L_MIN_VIN                      30.0f
#define MCCONF_L_MAX_VIN                      50.0f
#define MCCONF_L_TEMP_FET_START                70.0f
#define MCCONF_L_TEMP_FET_END                  80.0f
#define MCCONF_L_TEMP_MOTOR_START              80.0f
#define MCCONF_L_TEMP_MOTOR_END               100.0f
#define MCCONF_L_WATT_MAX                1500000.0f
#define MCCONF_L_WATT_MIN               -1500000.0f
#define MCCONF_SI_WHEEL_DIAMETER              0.083f
#define MCCONF_L_MAX_ERPM                 15000.0f
#define MCCONF_L_MIN_ERPM                -15000.0f
#define MCCONF_L_MIN_DUTY                     0.0f
#define MCCONF_L_MAX_DUTY                    1.00f
#define MCCONF_FAULT_STOP_TIME_MS             500u
/* Automatic fault recovery is deliberately stricter than the fault-stop timer.
 * A transient fault may clear only after the bridge has remained electrically
 * quiet and all basic health inputs are valid for this additional dwell. */
#define MCCONF_FAULT_RECOVERY_SAFE_CURRENT_MA     1000u
#define MCCONF_FAULT_RECOVERY_SAFE_ERPM             75u
#define MCCONF_FAULT_RECOVERY_SAFE_DWELL_MS        300u
#define MCCONF_FOC_DUTY_DOWNRAMP_KP            20.0f
#define MCCONF_FOC_DUTY_DOWNRAMP_KI           400.0f
#define MCCONF_DUTY_RAMP_STEP_DEFAULT            0.02f /* VESC m_duty_ramp_step */
#define MCCONF_CC_MIN_CURRENT                     0.05f /* VESC-style release threshold */
#define MCCONF_DUTY_PI_BUS_NOMINAL_V            42.5f
/* ABI incremental encoder LEFT, VESC m_encoder_counts semantics.
 * 1024 PPR quadrature = 4096 counts/rev. PB6/PB7 are shared with LEFT Hall V/W,
 * therefore Hall and ABI are mutually exclusive sensor-port modes. */
#define MCCONF_ENCODER_COUNTS_DEFAULT             4096u
#define MCCONF_ENCODER_RATIO_MAX                  10000.0f
#define MCCONF_ENCODER_OFFSET_DEFAULT             0.0f
#define MCCONF_ENCODER_STARTUP_ALIGN_CURRENT_A    3.00f /* Detect-All/boot Id starts at 3 A */
#define MCCONF_ENCODER_STARTUP_ALIGN_STEP_A       1.00f /* VESC-style adaptive rise until motion */
#define MCCONF_ENCODER_STARTUP_ALIGN_MAX_A       15.00f /* hard board/config ceiling; never exceeded */
#define MCCONF_ENCODER_STARTUP_ALIGN_RAMP_MS       120u
#define MCCONF_ENCODER_STARTUP_ALIGN_HOLD_MS       120u
/* Internal LEFT steering normalization envelope. External owners always use
 * standard VESC COMM_SET_POS 0..360; this internal -30..+30 coordinate only maps
 * that raw actuator position onto the calibrated encoder-count span. Vehicle
 * physical wheel-angle calibration is owned by ROS/ROS Web. */
#define MCCONF_STEERING_POS_MIN_DEG             (-30.0f)
#define MCCONF_STEERING_POS_MAX_DEG               30.0f
/* Runtime steering deliberately avoids the high-friction end regions found in
 * hardware tests. The measured/calibrated hard-stop span is preserved, then
 * the old logical 20..340 physical window is normalized back to 0..360 for
 * VESC Tool / ROS / Web commands and feedback. 320/360 = 8/9 runtime span. */
#define MCCONF_STEERING_RUNTIME_SPAN_NUM             8u
#define MCCONF_STEERING_RUNTIME_SPAN_DEN             9u
#define MCCONF_STEERING_POSITION_CURRENT_MAX_MA  10000u
/* Calibrated LEFT steering static-friction assist. Hardware floor test on
 * 2026-09-17 showed that ~10 A Iq target produces the ~0.20 duty needed to
 * break tyre/linkage stiction on the floor. This is a short position-only kick;
 * the normal motor hard limit remains independently configured (12 A deployed). */
#define MCCONF_STEERING_BREAKAWAY_CURRENT_MA     12000u
#define MCCONF_STEERING_BREAKAWAY_DUTY_PERMILLE   200u
#define MCCONF_STEERING_BREAKAWAY_ENTER_MDEG       800u
#define MCCONF_STEERING_BREAKAWAY_REARM_MDEG       500u
#define MCCONF_STEERING_BREAKAWAY_SETTLE_MDEG      350u
#define MCCONF_STEERING_BREAKAWAY_MIN_MS            10u
#define MCCONF_STEERING_BREAKAWAY_MAX_MS            18u
#define MCCONF_STEERING_BREAKAWAY_REARM_MS         150u
#define MCCONF_STEERING_PROGRESS_MDEG                200u
#define MCCONF_STEERING_STALL_VEL_MDEG_S          1500u
#define MCCONF_STEERING_BREAKAWAY_MOTION_COUNTS     16u
/* LEFT steering cascaded servo: position -> steering velocity -> Iq -> FOC.
 * Values are fixed-point friendly and intentionally independent from RIGHT Hall
 * speed PID. Position P=12 1/s reaches the 35 deg/s velocity ceiling at ~2.9 deg,
 * creating a natural approach/braking region before the target. */
#define MCCONF_STEERING_HOLD_ENTER_MDEG              400u
#define MCCONF_STEERING_HOLD_EXIT_MDEG               650u
#define MCCONF_STEERING_HOLD_VEL_MDEG_S             2000u
#define MCCONF_STEERING_APPROACH_MDEG                3000u
#define MCCONF_STEERING_POS_TO_VEL_KP                  12u
#define MCCONF_STEERING_VEL_MAX_MDEG_S              35000u
/* velocity Kp = 0.4 A/(deg/s): q4 = vel_error[mdeg/s] * 8 / 25. */
#define MCCONF_STEERING_VEL_KP_NUM                      1
#define MCCONF_STEERING_VEL_KP_DEN                     50
/* Start with velocity integral disabled; add only after P+friction is stable. */
#define MCCONF_STEERING_VEL_KI_Q16                      0
#define MCCONF_STEERING_VEL_I_LIMIT_MA               1500u
#define MCCONF_STEERING_FRICTION_CURRENT_MA          3000u
#define MCCONF_STEERING_TRACK_CURRENT_MAX_MA          5000u
#define MCCONF_STEERING_APPROACH_CURRENT_MAX_MA       3500u
#define MCCONF_STEERING_VEL_FILTER_SHIFT                 2u
#define MCCONF_STEERING_MOTION_PROGRESS_COUNTS       32u /* commissioning motion threshold; not a torque assist */
#define MCCONF_STEERING_CENTER_CURRENT_A           2.00f /* commissioning return-to-midpoint */
#define MCCONF_STEERING_CENTER_TOL_COUNTS           24u /* ~0.32 deg on measured ~4500-count span */
#define MCCONF_STEERING_CENTER_PID_MS             2500u
#define MCCONF_STEERING_CENTER_TRIM_TIMEOUT_MS    2500u
#define MCCONF_STEERING_CENTER_PULSE_MS             15u
#define MCCONF_STEERING_CENTER_REST_MS              70u
#define MCCONF_STEERING_HOME_CURRENT_A             4.00f
#define MCCONF_STEERING_CAL_CURRENT_MAX_A         15.00f /* commissioning only; runtime steering stays capped separately */
#define MCCONF_STEERING_DETECT_CURRENT_START_A      3.00f
#define MCCONF_STEERING_DETECT_CURRENT_STEP_A       1.00f
#define MCCONF_STEERING_MOVE_PROBE_MS                450u
#define MCCONF_STEERING_STOP_CONFIRM_MS              300u
#define MCCONF_STEERING_STALL_MS                     350u
#define MCCONF_STEERING_SEEK_TIMEOUT_MS            20000u
#define MCCONF_STEERING_MIN_SPAN_COUNTS              32
#define MCCONF_STEERING_SAFE_SPAN_PERCENT             95u /* measured hard-stop span is preserved; runtime uses 95% for 2.5% margin each side */
#define MCCONF_STEERING_SETTLE_COUNTS                 6
#define MCCONF_ENCODER_SPEED_WINDOW_TICKS           320u /* 20 ms @16 kHz, 50-Hz speed estimator */
#define MCCONF_ENCODER_SPEED_TIMEOUT_TICKS         8000u /* 0.5 s -> zero */
#define MCCONF_FOC_CURRENT_KP_Q11           1229u
#define MCCONF_FOC_CURRENT_KI_Q16           1229u
#define MCCONF_FOC_ID_KP_Q11                 819u
#define MCCONF_FOC_ID_KI_Q16                 737u
/* VESC default foc_current_filter_const is 0.1. The current PI uses raw Park
 * feedback; this standard field controls monitoring/telemetry filtering only. */
#define MCCONF_FOC_TELEMETRY_FILTER_DEFAULT     0.10f
/* Upstream VESC 6.x PLL defaults. Runtime F103 mengubahnya menjadi koefisien
 * fixed-point pada saat konfigurasi berubah; tidak ada float di ADC ISR. */
#define MCCONF_FOC_PLL_KP_DEFAULT              2000.0f
#define MCCONF_FOC_PLL_KI_DEFAULT             30000.0f
/* Dead-time compensation mengikuti field VESC foc_dt_us, tetapi hanya
 * mengoreksi model tegangan/observer; tidak mengubah switching command SVPWM.
 * Default OFF sampai karakterisasi hardware dilakukan. EEPROM EXT11 menyimpan
 * nilai dengan resolusi 1 ns pada 12 bit (0..4,095 us). */
#define MCCONF_FOC_DT_US_DEFAULT                  0.0f
#define MCCONF_FOC_DT_US_MAX                      4.095f
#define MCCONF_FOC_DT_NS_MAX                      4095u
/* VESC speed PID uses normalized output/current scaling. Hardware step tests at
 * +/-750 ERPM selected Kp=0.002, Ki=0.002, Kd=0 for this Hall hoverboard: the
 * doubled Ki removed ~3.3% steady error without excessive current; Kd stays 0
 * because Hall-speed quantization makes a derivative term noisy. These integer
 * fields are persisted gain*1000, not direct Vq-controller coefficients. */
#define MCCONF_SPEED_GAIN_SCALE             100000u /* 1e-5 resolution; fits standard VESC speed gains in uint16 */
#define MCCONF_SPEED_KP_Q11                   200u /* 0.00200 */
#define MCCONF_SPEED_KI_Q16                   200u /* 0.00200 */
#define MCCONF_SPEED_KD_Q11                     0u
#define MCCONF_SPEED_KD_FILTER_DEFAULT         0.20f
#define MCCONF_POSITION_KP_Q11                  25u /* 0.025: upstream VESC default position Kp */
#define MCCONF_POSITION_KI_Q16                   0u
#define MCCONF_POSITION_KD_Q11                   0u
#define MCCONF_POSITION_KD_FILTER_Q16         13107u /* 0.20, VESC default D filter */
/* Project-only multi-turn count-position safety. Standard COMM_SET_POS does
 * not use these limits; it follows VESC normalized PID and motor-current limits. */
#define MCCONF_POSITION_CURRENT_MAX_MA          600u /* custom count-position ceiling */
#define MCCONF_POSITION_DAMP_CURRENT_MA         400u /* kinetic brake; below measured 0.6 A static breakaway */
/* VESC-style speed-command ramp. VESC exposes this in ERPM/s; the ISR keeps
 * mechanical RPM fixed-point. 20000 ERPM/s was selected from repeated 8000-ERPM
 * hardware steps as the best response/stability compromise; still configurable. */
#define MCCONF_SPEED_RAMP_ERPMS_S            20000u
#define MCCONF_SPEED_RELEASE_ERPM               75u  /* 5 mechanical RPM @ 15 pole-pairs */
/* Zero-speed active-brake quiet zone. Hall feedback is quantized near standstill:
 * one reverse edge after crossing zero can still report ~150 eRPM. Enter quiet
 * below 100 eRPM and only re-arm above 250 eRPM so a stop cannot chatter between
 * adjacent Hall sectors, while a genuine external roll still re-enables braking. */
#define MCCONF_ZERO_HOLD_QUIET_ENTER_ERPM      100u
#define MCCONF_ZERO_HOLD_QUIET_EXIT_ERPM       250u
#define MCCONF_FOC_VOLTAGE_MAX              16000
#define MCCONF_FOC_DUTY_VOLTAGE_MAX          FOC_SVPWM_VECTOR_MAX
#define MCCONF_L_ABS_CURRENT_MAX               30.0f /* absolute hard phase-current ceiling; VESC Tool motor limit <=30A */
#define MCCONF_PWM_MARGIN_COUNTS          FOC_PWM_MARGIN_COUNTS
#define MCCONF_ABS_CURRENT_QUAL_SAMPLES           3u /* ~0.19 ms @16 kHz: reject transient D/Q spikes */
/* Safety tambahan yang tetap ringan untuk Cortex-M3. Overspeed memakai Hall
 * period mentah agar fault tidak tertutup clamp telemetry 1000 mechanical RPM. */
#define MCCONF_ABS_OVERSPEED_MARGIN_PERCENT      110u /* hard fault 10% di atas soft ERPM limit */
#define MCCONF_ABS_OVERSPEED_QUAL_SAMPLES          8u /* 0,5 ms @16 kHz, menolak satu glitch timing */
/* Dua shunt fase FOC normalnya berpusat dekat ADC midscale. Toleransi sengaja
 * lebar agar pergeseran common-mode board hoverboard tidak memicu false fault. */
#define MCCONF_CURRENT_OFFSET_CENTER_ADC         2048
#define MCCONF_CURRENT_OFFSET_MAX_DEVIATION_ADC   900
#define MCCONF_CURRENT_OFFSET_MAX_PAIR_DELTA_ADC  900
/* Saat MOE aktif, low-side current amplifier LEFT dapat bergeser common-mode
 * lebih dari 900 count dari midscale walaupun kedua channel masih sehat.
 * Powered zero-vector memakai validator terpisah: jauh dari rail + pair delta. */
#define MCCONF_DRIVEN_OFFSET_RAIL_MARGIN_ADC      128
/* LEFT ABI: batas delta dibuat jauh di atas kecepatan steering normal. Nilai
 * ini hanya menangkap loncatan counter/glitch; hard-stop calibration OPENLOOP
 * tetap diizinkan karena gerak aktualnya sangat lambat. */
#define MCCONF_ENCODER_FAULT_MAX_RPM             1500u
#define MCCONF_ENCODER_FAULT_DELTA_MARGIN_COUNTS    2u
#define MCCONF_ENCODER_STUCK_MIN_ERPM             100u
#define MCCONF_ENCODER_STUCK_CURRENT_MA           1000u
#define MCCONF_ENCODER_STUCK_TIMEOUT_TICKS       8000u /* 0,5 s @16 kHz */
/* OFF->RUN powered-current baseline. The low-side current amplifiers move to
 * a different common-mode when MOE turns on. Real hardware needs a short
 * discard window before the baseline is stationary; only then average 80 PWM
 * frames. 160 discard + 80 average = 15 ms total @16 kHz. */
#define MCCONF_BRIDGE_PRESETTLE_SAMPLES            160u
#define MCCONF_BRIDGE_SETTLE_SAMPLES                80u
/* OFF/high-impedance telemetry uses its own frozen zero-current ADC baseline.
 * Remove a few ADC counts of amplifier noise without hiding real passive/regen
 * current changes when the wheel is back-driven manually. 4 counts = 0.08 A. */
#define MCCONF_OFF_TELEM_DEADBAND_COUNTS              4
#define MCCONF_OFF_TELEM_SETTLE_SAMPLES           16000u /* 1 s @16 kHz: high-Z shunt common-mode benar-benar stabil */
#define MCCONF_MOTOR_CURRENT_MAX_Q4  (I_MOT_MAX * A2BIT_CONV * 16)
#define MCCONF_MOTOR_RPM_MAX                 N_MOT_MAX
#define MCCONF_POLE_PAIRS_LEFT               4u
#define MCCONF_POLE_PAIRS_RIGHT              15u
/* VESC mcconf_default.h: foc_hall_interp_erpm default = 500 ERPM.
 * Nilai runtime tetap berasal dari Motor Config dan diprecompute ke integer
 * sebelum masuk ISR; jangan ubah menjadi mechanical RPM karena pole-pair bisa
 * berbeda antar motor dan dapat diubah dari VESC Tool. */
#define MCCONF_FOC_HALL_INTERP_ERPM_DEFAULT    500u
/* Upstream VESC default: 3 extra samples => 7 instantaneous GPIO reads with majority vote. */
#define MCCONF_M_HALL_EXTRA_SAMPLES_DEFAULT       3u
#ifndef MCCONF_FOC_CONTROL_DIV
#define MCCONF_FOC_CONTROL_DIV                  6u
#endif
#if (MCCONF_FOC_CONTROL_DIV < 1u) || (MCCONF_FOC_CONTROL_DIV > 6u)
#error "MCCONF_FOC_CONTROL_DIV must be 1..6"
#endif
#define MCCONF_OUTER_PID_HZ                  1000u /* VESC FOC speed/position PID thread equivalent */
#define MCCONF_TELEMETRY_HZ                   200u /* housekeeping/telemetry slow path */
/* Hall timeout must be longer than one Hall sector at low VESC ERPM.
 * At 50 ERPM: 60/(50*6)=0.2 s/edge => 3200 ISR ticks @16 kHz.
 * 8000 ticks (0.5 s) keeps valid low-speed Hall feedback down to ~20 ERPM. */
#define MCCONF_HALL_TIMEOUT_TICKS            8000u
/* Reject an impossible Hall edge that is >4x faster than the previous valid
 * sector period. This suppresses contact/boundary chatter near zero speed. */
#define MCCONF_HALL_PERIOD_OUTLIER_RATIO         4u
/* Require a new GPIO Hall code to persist for three 16-kHz samples (~125 us
 * from first to third sample). This filters switching-edge/metastability
 * glitches without materially shifting a 60-deg sector at steering speeds. */
#define MCCONF_HALL_DEBOUNCE_SAMPLES              3u
#define MCCONF_HALL_PHASE_ADVANCE_TICKS       (MCCONF_HALL_DEBOUNCE_SAMPLES - 1u)
/* After start or a real direction reversal, accept five valid adjacent edges
 * before enabling the period-outlier test. This fully refreshes the four-edge
 * period history so acceleration from near-zero is not mistaken for chatter. */
#define MCCONF_HALL_PERIOD_FILTER_WARMUP_EDGES     5u
#define MCCONF_TRQ_STOP_RPM_DEADBAND  TRQ_STOP_RPM_DEADBAND
#define MCCONF_OPENLOOP_RPM_DEFAULT   SVPWM_OPENLOOP_RPM_DEFAULT
#define MCCONF_OPENLOOP_RPM_MAX       SVPWM_OPENLOOP_RPM_MAX
#define MCCONF_OPENLOOP_ACCEL_RPM_S   SVPWM_ACCEL_RPM_PER_S
#define MCCONF_OPENLOOP_ALIGN_MS       SVPWM_ALIGN_MS
#define MCCONF_OPENLOOP_ID_SLEW_A_S    SVPWM_ID_SLEW_A_PER_S

#endif
