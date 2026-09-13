#include <math.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "config.h"
#include "defines.h"
#ifdef STM32F103xE
#include "eeprom.h"
#endif
#include "motor/mc_interface.h"
#include "motor/mcpwm_foc.h"
#include "vesc/app_vesc.h"

extern volatile adc_buf_t adc_buffer;

typedef struct {
    uint32_t last_ms;
    uint32_t zero_ms;
    float filt_v1;
    float filt_v2;
    float ramp;
    uint8_t initialized;
} app_adc_state_t;

static app_configuration s_conf[2];
static app_adc_state_t s_state[2];
static volatile int8_t s_output_disable_mode = 0; /* 0 enabled, 1 timed, 2 forever */
static volatile uint32_t s_output_disable_until_ms = 0u;
static volatile float s_v1 = 0.0f;
static volatile float s_v2 = 0.0f;
static volatile float s_dec1 = 0.0f;
static volatile float s_dec2 = 0.0f;
static volatile bool s_range_ok = true;

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float mapf(float x, float in0, float in1, float out0, float out1) {
    if (fabsf(in1 - in0) < 1e-9f) return out0;
    return out0 + (x - in0) * (out1 - out0) / (in1 - in0);
}

static void deadband(float *v, float tres) {
    tres = clampf(tres, 0.0f, 0.99f);
    if (fabsf(*v) < tres) {
        *v = 0.0f;
    } else {
        const float k = 1.0f / (1.0f - tres);
        *v = (*v > 0.0f) ? (k * *v + 1.0f - k) : -(k * -*v + 1.0f - k);
    }
}

/* F103 hanya menjalankan keluarga POLY standar VESC. EXPO/NATURAL memerlukan
 * powf/expf yang menarik beberapa KiB libm/soft-double pada Cortex-M3. App
 * config tetap wire-compatible dan mode unsupported dicanonicalize ke POLY,
 * sehingga VESC Tool tidak pernah membaca kembali opsi yang diam-diam diabaikan. */
static float throttle_curve(float val, float curve_acc, float curve_brake, int mode) {
    (void)mode;
    val = clampf(val, -1.0f, 1.0f);
    const float a = fabsf(val);
    const float curve = val >= 0.0f ? curve_acc : curve_brake;
    const float ret = curve >= 0.0f ? 1.0f - ((1.0f - a) / (1.0f + curve * a)) :
                                      a / (1.0f - curve * (1.0f - a));
    return val < 0.0f ? -ret : ret;
}


#ifdef STM32F103xE
/* App Config persistence remains permanently at EEPROM-emulation slots
 * 49..122. MC extension slots were appended at 123+, so legacy App Config
 * addresses never move. Each motor gets 37 16-bit App slots. */
#define APP_EE_BASE              49u
#define APP_EE_STRIDE            37u
#define APP_EE_SIGNATURE         0xA601u
#define APP_EE_KEY_SLOT          0u
#define APP_EE_KEY_VALUE         FLASH_WRITE_KEY
_Static_assert(APP_EE_BASE == 49u, "App Config base must stay after legacy MC slot 48");
_Static_assert(APP_EE_STRIDE == 37u, "App Config stride changed");
_Static_assert((APP_EE_BASE + (2u * APP_EE_STRIDE)) == 123u, "App Config must occupy exactly slots 49..122");
extern uint16_t VirtAddVarTab[NB_OF_VAR];

static bool app_ee_read(uint16_t slot, uint16_t *v) {
    return slot < NB_OF_VAR && EE_ReadVariable(VirtAddVarTab[slot], v) == 0u;
}
static bool app_ee_write_if_changed(uint16_t slot, uint16_t v) {
    uint16_t old = 0u;
    if (app_ee_read(slot, &old) && old == v) return true;
    return slot < NB_OF_VAR && EE_WriteVariable(VirtAddVarTab[slot], v) == HAL_OK;
}
static uint32_t float_bits(float f) { uint32_t u=0u; memcpy(&u,&f,sizeof(u)); return u; }
static float bits_float(uint32_t u) { float f=0.0f; memcpy(&f,&u,sizeof(f)); return f; }
static bool app_ee_write_u32(uint16_t slot, uint32_t v) {
    return app_ee_write_if_changed(slot,(uint16_t)(v&0xffffu)) &&
           app_ee_write_if_changed((uint16_t)(slot+1u),(uint16_t)(v>>16));
}
static bool app_ee_read_u32(uint16_t slot, uint32_t *v) {
    uint16_t lo=0u,hi=0u;
    if(!app_ee_read(slot,&lo)||!app_ee_read((uint16_t)(slot+1u),&hi))return false;
    *v=(uint32_t)lo|((uint32_t)hi<<16); return true;
}
static uint16_t app_flags_pack(const adc_config *c) {
    return (uint16_t)((c->use_filter?1u:0u) |
        (((uint16_t)c->safe_start&3u)<<1) | ((uint16_t)c->buttons<<3) |
        ((c->voltage_inverted?1u:0u)<<11) | ((c->voltage2_inverted?1u:0u)<<12) |
        ((c->multi_esc?1u:0u)<<13) | ((c->tc?1u:0u)<<14));
}
static void app_flags_unpack(adc_config *c, uint16_t f) {
    c->use_filter=(f&1u)!=0u; c->safe_start=(SAFE_START_MODE)((f>>1)&3u);
    c->buttons=(uint8_t)((f>>3)&0xffu); c->voltage_inverted=(f&(1u<<11))!=0u;
    c->voltage2_inverted=(f&(1u<<12))!=0u; c->multi_esc=(f&(1u<<13))!=0u;
    c->tc=(f&(1u<<14))!=0u;
}

bool app_vesc_store_configuration(bool second) {
    if (!EE_IsHealthy()) return false;
    const app_configuration *a=&s_conf[second?1u:0u];
    const adc_config *c=&a->app_adc_conf; const uint8_t b=(uint8_t)(APP_EE_BASE+(second?APP_EE_STRIDE:0u));
    /* EEPROM emulation executes from the same STM32F1 flash as the FOC ISR.
     * Release both bridges before writes, then invalidate this endpoint's commit
     * marker so interrupted updates never boot as a partially valid App Config. */
    mcpwm_foc_release_motor(false);
    mcpwm_foc_release_motor(true);
    bool ok=true; if (HAL_FLASH_Unlock() != HAL_OK) return false;
    if (!app_ee_write_if_changed(b,0u)) {
        HAL_FLASH_Lock();
        return false;
    }
    ok &= app_ee_write_if_changed((uint8_t)(b+1u),(uint16_t)a->app_to_use);
    ok &= app_ee_write_u32((uint8_t)(b+2u),a->timeout_msec);
    ok &= app_ee_write_u32((uint8_t)(b+4u),float_bits(a->timeout_brake_current));
    ok &= app_ee_write_if_changed((uint8_t)(b+6u),(uint16_t)c->ctrl_type);
    ok &= app_ee_write_u32((uint8_t)(b+7u),float_bits(c->hyst));
    ok &= app_ee_write_u32((uint8_t)(b+9u),float_bits(c->voltage_start));
    ok &= app_ee_write_u32((uint8_t)(b+11u),float_bits(c->voltage_end));
    ok &= app_ee_write_u32((uint8_t)(b+13u),float_bits(c->voltage_min));
    ok &= app_ee_write_u32((uint8_t)(b+15u),float_bits(c->voltage_max));
    ok &= app_ee_write_u32((uint8_t)(b+17u),float_bits(c->voltage_center));
    ok &= app_ee_write_u32((uint8_t)(b+19u),float_bits(c->voltage2_start));
    ok &= app_ee_write_u32((uint8_t)(b+21u),float_bits(c->voltage2_end));
    ok &= app_ee_write_if_changed((uint8_t)(b+23u),app_flags_pack(c));
    ok &= app_ee_write_u32((uint8_t)(b+24u),float_bits(c->throttle_exp));
    ok &= app_ee_write_u32((uint8_t)(b+26u),float_bits(c->throttle_exp_brake));
    ok &= app_ee_write_if_changed((uint8_t)(b+28u),(uint16_t)c->throttle_exp_mode);
    ok &= app_ee_write_u32((uint8_t)(b+29u),float_bits(c->ramp_time_pos));
    ok &= app_ee_write_u32((uint8_t)(b+31u),float_bits(c->ramp_time_neg));
    ok &= app_ee_write_u32((uint8_t)(b+33u),float_bits(c->tc_max_diff));
    ok &= app_ee_write_u32((uint8_t)(b+35u),c->update_rate_hz);
    /* Commit only a complete payload. Any failed EEPROM write leaves sig=0 and
     * therefore cannot turn a mixed old/new App Config into a valid image. */
    if (ok) ok = app_ee_write_if_changed(b,APP_EE_SIGNATURE);
    if (ok) ok = app_ee_write_if_changed(APP_EE_KEY_SLOT,(uint16_t)APP_EE_KEY_VALUE);
    HAL_FLASH_Lock(); return ok;
}

bool app_vesc_load_configuration(bool second) {
    if (!EE_IsHealthy()) return false;
    const uint8_t b=(uint8_t)(APP_EE_BASE+(second?APP_EE_STRIDE:0u)); uint16_t key=0u,sig=0u,v16=0u,flags=0u; uint32_t u=0u;
    if(!app_ee_read(APP_EE_KEY_SLOT,&key)||key!=(uint16_t)APP_EE_KEY_VALUE||!app_ee_read(b,&sig)||sig!=APP_EE_SIGNATURE)return false;
    app_configuration a=s_conf[second?1u:0u]; adc_config *c=&a.app_adc_conf;
    if(!app_ee_read((uint8_t)(b+1u),&v16)) return false;
    a.app_to_use=(app_use)v16;
    if(!app_ee_read_u32((uint8_t)(b+2u),&a.timeout_msec))return false;
    if(!app_ee_read_u32((uint8_t)(b+4u),&u)) return false;
    a.timeout_brake_current=bits_float(u);
    if(!app_ee_read((uint8_t)(b+6u),&v16)) return false;
    c->ctrl_type=(adc_control_type)v16;
#define READ_F32(off,field) do{if(!app_ee_read_u32((uint8_t)(b+(off)),&u))return false;(field)=bits_float(u);}while(0)
    READ_F32(7u,c->hyst); READ_F32(9u,c->voltage_start); READ_F32(11u,c->voltage_end);
    READ_F32(13u,c->voltage_min); READ_F32(15u,c->voltage_max); READ_F32(17u,c->voltage_center);
    READ_F32(19u,c->voltage2_start); READ_F32(21u,c->voltage2_end);
    if(!app_ee_read((uint8_t)(b+23u),&flags)) return false;
    app_flags_unpack(c,flags);
    READ_F32(24u,c->throttle_exp); READ_F32(26u,c->throttle_exp_brake);
    if(!app_ee_read((uint8_t)(b+28u),&v16)) return false;
    c->throttle_exp_mode=(thr_exp_mode)v16;
    READ_F32(29u,c->ramp_time_pos); READ_F32(31u,c->ramp_time_neg); READ_F32(33u,c->tc_max_diff);
#undef READ_F32
    if(!app_ee_read_u32((uint8_t)(b+35u),&c->update_rate_hz))return false;
    return app_vesc_set_configuration(second,&a);
}

#else
bool app_vesc_store_configuration(bool second) { (void)second; return true; }
bool app_vesc_load_configuration(bool second) { (void)second; return false; }
#endif

void app_vesc_defaults(app_configuration *a, uint8_t id) {
    if (!a) return;
    memset(a, 0, sizeof(*a));
    a->controller_id = id;
    a->timeout_msec = VESC_RUNTIME_TIMEOUT_DEFAULT_MS;
    a->timeout_brake_current = 0.0f;
    a->can_baud_rate = CAN_BAUD_500K;
    a->pairing_done = true;
    a->permanent_uart_enabled = true;
    a->can_mode = CAN_MODE_VESC;
    a->app_to_use = APP_UART;
    a->app_uart_baudrate = USART3_BAUD;
    a->app_adc_conf.ctrl_type = ADC_CTRL_TYPE_NONE;
    a->app_adc_conf.hyst = 0.15f;
    a->app_adc_conf.voltage_start = 0.9f;
    a->app_adc_conf.voltage_end = 3.0f;
    a->app_adc_conf.voltage_min = 0.0f;
    a->app_adc_conf.voltage_max = 3.3f;
    a->app_adc_conf.voltage_center = 1.65f;
    a->app_adc_conf.voltage2_start = 0.9f;
    a->app_adc_conf.voltage2_end = 3.0f;
    a->app_adc_conf.use_filter = true;
    a->app_adc_conf.safe_start = SAFE_START_REGULAR;
    /* POLY adalah satu-satunya curve yang sengaja didukung pada F103. Curve
     * default 0 tetap identitas sama seperti default EXPO VESC. */
    a->app_adc_conf.throttle_exp_mode = THR_EXP_POLY;
    a->app_adc_conf.ramp_time_pos = 0.3f;
    a->app_adc_conf.ramp_time_neg = 0.1f;
    a->app_adc_conf.update_rate_hz = 200u;
}

void app_vesc_init(void) {
    app_vesc_defaults(&s_conf[0], 1u);
    app_vesc_defaults(&s_conf[1], 2u);
    mcpwm_foc_vesc_timeout_configure(false, s_conf[0].timeout_msec, s_conf[0].timeout_brake_current);
    mcpwm_foc_vesc_timeout_configure(true, s_conf[1].timeout_msec, s_conf[1].timeout_brake_current);
    memset(s_state, 0, sizeof(s_state));
    s_output_disable_mode = 0;
    s_output_disable_until_ms = 0u;
}

static bool adc_app_enabled(const app_configuration *a);

const app_configuration *app_vesc_get_configuration(bool second) {
    return &s_conf[second ? 1 : 0];
}

bool app_vesc_set_configuration(bool second, const app_configuration *conf) {
    if (!conf) return false;
    const app_configuration previous = s_conf[second ? 1u : 0u];
    app_configuration c = *conf;
    c.controller_id = second ? 2u : 1u;
    c.can_mode = CAN_MODE_VESC;
    c.permanent_uart_enabled = true; /* USART3 selalu harus dapat diakses VESC Tool. */
    /* Hardware ini hanya mempunyai jalur aplikasi UART dan dua ADC PA2/PA3.
     * Mode PPM/Nunchuk/NRF/PAS tidak memiliki input fisik, sehingga jangan
     * menerima konfigurasi yang tampak valid tetapi tidak mungkin bekerja. */
    if (c.app_to_use != APP_UART && c.app_to_use != APP_ADC && c.app_to_use != APP_ADC_UART) {
        c.app_to_use = APP_UART;
    }
    if (c.app_adc_conf.ctrl_type > ADC_CTRL_TYPE_PID_REV_BUTTON) c.app_adc_conf.ctrl_type = ADC_CTRL_TYPE_NONE;
    if (c.app_adc_conf.throttle_exp_mode != THR_EXP_POLY) c.app_adc_conf.throttle_exp_mode = THR_EXP_POLY;
    if (c.app_adc_conf.update_rate_hz == 0u) c.app_adc_conf.update_rate_hz = 1u;
    /* Fail-closed production policy: App Config tidak boleh menonaktifkan atau
     * memperpanjang timeout aktuator lokal melewati 500 ms. Nilai 0 dari VESC
     * Tool dikembalikan ke default aman 300 ms. */
    if (c.timeout_msec == 0u) c.timeout_msec = VESC_RUNTIME_TIMEOUT_DEFAULT_MS;
    else if (c.timeout_msec < VESC_RUNTIME_TIMEOUT_MIN_MS) c.timeout_msec = VESC_RUNTIME_TIMEOUT_MIN_MS;
    else if (c.timeout_msec > VESC_RUNTIME_TIMEOUT_MAX_MS) c.timeout_msec = VESC_RUNTIME_TIMEOUT_MAX_MS;
    /* This board has one physical VESC UART. Keep its electrical link fixed at
     * F103_VESC_UART_BAUD so writing App Config cannot strand VESC Tool on an unknown baud. */
    c.app_uart_baudrate = USART3_BAUD;
    /* Leaving an active ADC controller must release its last motor command once.
     * In APP_ADC_UART with ctrl_type NONE, subsequent UART SET_* commands remain
     * authoritative because the ADC app no longer touches the motor at all. */
#ifdef STM32F103xE
    if (adc_app_enabled(&previous) && previous.app_adc_conf.ctrl_type != ADC_CTRL_TYPE_NONE &&
        (!adc_app_enabled(&c) || c.app_adc_conf.ctrl_type == ADC_CTRL_TYPE_NONE)) {
        mc_interface_select_motor_thread(second ? 2 : 1);
        mc_interface_release_motor();
    }
#else
    (void)previous;
#endif
    s_conf[second ? 1 : 0] = c;
    mcpwm_foc_vesc_timeout_configure(second, c.timeout_msec, c.timeout_brake_current);
    memset(&s_state[second ? 1 : 0], 0, sizeof(s_state[0]));
    return true;
}


void app_vesc_disable_output(int32_t time_ms) {
    if(time_ms==0){
        s_output_disable_mode=0; s_output_disable_until_ms=0u;
    }else if(time_ms<0){
        s_output_disable_mode=2; s_output_disable_until_ms=0u;
    }else{
        s_output_disable_mode=1; s_output_disable_until_ms=HAL_GetTick()+(uint32_t)time_ms;
    }
}

bool app_vesc_output_disabled(uint32_t now_ms) {
    if(s_output_disable_mode==2) return true;
    if(s_output_disable_mode==1){
        if((int32_t)(now_ms-s_output_disable_until_ms)<0) return true;
        s_output_disable_mode=0; s_output_disable_until_ms=0u;
    }
    return false;
}

static bool adc_app_enabled(const app_configuration *a) {
    return a->app_to_use == APP_ADC || a->app_to_use == APP_ADC_UART;
}

static void touch(bool second) {
    mcpwm_foc_vesc_override_touch(second);
}

static void select_motor(bool second) {
    mc_interface_select_motor_thread(second ? 2 : 1);
}

static void set_current_user(bool second, float amp) {
    select_motor(second);
    touch(second);
    mc_interface_set_current(amp);
}

static void set_current_rel_user(bool second, float rel) {
    select_motor(second);
    touch(second);
    mc_interface_set_current_rel(rel);
}

static void set_brake_rel_user(bool second, float rel) {
    select_motor(second);
    touch(second);
    mc_interface_set_brake_current_rel(rel);
}

static void set_brake_user(bool second, float amp) {
    select_motor(second);
    if (amp < 0.0f) amp = -amp;
    touch(second);
    mc_interface_set_brake_current(amp);
}

static void set_duty_user(bool second, float duty) {
    select_motor(second);
    touch(second);
    mc_interface_set_duty(duty);
}

static void set_rpm_user(bool second, float erpm) {
    select_motor(second);
    touch(second);
    mc_interface_set_pid_speed(erpm);
}

static void apply_adc(bool second, const app_configuration *a, uint32_t now_ms, float raw_v1, float raw_v2) {
    app_adc_state_t *st = &s_state[second ? 1 : 0];
    const adc_config *c = &a->app_adc_conf;
    uint32_t hz = c->update_rate_hz;
    if (hz < 1u) hz = 1u;
    if (hz > 200u) hz = 200u; /* main housekeeping cadence is 200 Hz */
    const uint32_t period = (1000u + hz - 1u) / hz;
    if (st->initialized && (uint32_t)(now_ms - st->last_ms) < period) return;
    const uint32_t dt = st->initialized ? (uint32_t)(now_ms - st->last_ms) : period;
    st->last_ms = now_ms;
    if (!st->initialized) {
        st->filt_v1 = raw_v1;
        st->filt_v2 = raw_v2;
        st->initialized = 1u;
    } else {
        st->filt_v1 += (raw_v1 - st->filt_v1) * 0.2f;
        st->filt_v2 += (raw_v2 - st->filt_v2) * 0.2f;
    }

    float v1 = c->use_filter ? st->filt_v1 : raw_v1;
    float v2 = c->use_filter ? st->filt_v2 : raw_v2;
    bool range_ok = v1 >= c->voltage_min && v1 <= c->voltage_max;
    float pwr;
    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
        pwr = v1 < c->voltage_center ? mapf(v1, c->voltage_start, c->voltage_center, 0.0f, 0.5f) :
                                      mapf(v1, c->voltage_center, c->voltage_end, 0.5f, 1.0f);
        break;
    default:
        pwr = mapf(v1, c->voltage_start, c->voltage_end, 0.0f, 1.0f);
        break;
    }
    pwr = clampf(pwr, 0.0f, 1.0f);
    if (c->voltage_inverted) pwr = 1.0f - pwr;

    float brake = clampf(mapf(v2, c->voltage2_start, c->voltage2_end, 0.0f, 1.0f), 0.0f, 1.0f);
    if (c->voltage2_inverted) brake = 1.0f - brake;

    s_v1 = v1; s_v2 = v2; s_dec1 = pwr; s_dec2 = brake; s_range_ok = range_ok;

    /* ADC_CTRL_TYPE_NONE is telemetry-only. In particular APP_ADC_UART must not
     * repeatedly issue zero-current/brake commands, because that steals control
     * from UART and keeps the power bridge switching at a nonzero modulation. */
    if (c->ctrl_type == ADC_CTRL_TYPE_NONE) {
        st->ramp = 0.0f;
        st->zero_ms = 0u;
        return;
    }

    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
        pwr = pwr * 2.0f - 1.0f;
        break;
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
        pwr -= brake;
        break;
    default:
        break;
    }
    deadband(&pwr, c->hyst);
    pwr = throttle_curve(pwr, c->throttle_exp, c->throttle_exp_brake, c->throttle_exp_mode);

    const float ramp_t = fabsf(pwr) > fabsf(st->ramp) ? c->ramp_time_pos : c->ramp_time_neg;
    if (ramp_t > 0.01f) {
        const float step = (float)dt / (ramp_t * 1000.0f);
        if (st->ramp < pwr) st->ramp = fminf(st->ramp + step, pwr);
        else if (st->ramp > pwr) st->ramp = fmaxf(st->ramp - step, pwr);
        pwr = st->ramp;
    } else {
        st->ramp = pwr;
    }

    if (fabsf(pwr) < 0.001f) st->zero_ms += dt; else st->zero_ms = 0u;
    if ((c->safe_start != SAFE_START_DISABLED && st->zero_ms < 500u) || !range_ok ||
        (mc_interface_get_fault_motor(second) != FAULT_CODE_NONE && c->safe_start != SAFE_START_NO_FAULT)) {
        set_brake_user(second, a->timeout_brake_current);
        return;
    }

    const volatile mc_configuration *mc = mc_interface_get_configuration_motor(second);
    switch (c->ctrl_type) {
    case ADC_CTRL_TYPE_NONE:
        set_current_user(second, 0.0f);
        break;
    case ADC_CTRL_TYPE_CURRENT:
    case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
        set_current_rel_user(second, pwr);
        break;
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
    case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
    case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
        if (pwr >= 0.0f) set_current_rel_user(second, pwr);
        else set_brake_rel_user(second, -pwr);
        break;
    case ADC_CTRL_TYPE_DUTY:
    case ADC_CTRL_TYPE_DUTY_REV_CENTER:
    case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
        set_duty_user(second, pwr * mc->l_max_duty);
        break;
    case ADC_CTRL_TYPE_PID:
    case ADC_CTRL_TYPE_PID_REV_CENTER:
    case ADC_CTRL_TYPE_PID_REV_BUTTON:
        set_rpm_user(second, pwr >= 0.0f ? pwr * mc->l_max_erpm : (-pwr) * mc->l_min_erpm);
        break;
    default:
        set_current_user(second, 0.0f);
        break;
    }
}

void app_vesc_process(uint32_t now_ms) {
    if(app_vesc_output_disabled(now_ms)) { mc_interface_select_motor_thread(1); return; }
    const float v1 = (float)adc_buffer.adc2_spare4 * (3.3f / 4095.0f); /* PA2 / ADC2 CH2 */
    const float v2 = (float)adc_buffer.adc2_spare5 * (3.3f / 4095.0f); /* PA3 / ADC2 CH3 */
    /* Realtime ADC VESC Tool harus tetap menampilkan tegangan pin walaupun
     * App ADC tidak sedang dipilih. apply_adc() akan menimpa cache ini dengan
     * nilai terfilter bila App ADC memang aktif. */
    s_v1 = v1;
    s_v2 = v2;
    const bool local_adc = adc_app_enabled(&s_conf[0]);
    if (local_adc) apply_adc(false, &s_conf[0], now_ms, v1, v2);
    if (local_adc && s_conf[0].app_adc_conf.multi_esc) {
        apply_adc(true, &s_conf[0], now_ms, v1, v2);
    } else if (adc_app_enabled(&s_conf[1])) {
        apply_adc(true, &s_conf[1], now_ms, v1, v2);
    }
    mc_interface_select_motor_thread(1);
}

float app_vesc_adc_decoded(bool second_channel) { return second_channel ? s_dec2 : s_dec1; }
float app_vesc_adc_voltage(bool second_channel) { return second_channel ? s_v2 : s_v1; }
