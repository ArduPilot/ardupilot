#include "AP_Brush.h"

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

AP_Brush *AP_Brush::_singleton;

AP_Brush::AP_Brush()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Brush must be singleton");
    }
    _singleton = this;

    _active = false;
    _front_on = false;
    _rear_on = false;
    _power_pct = 0;
    _last_front_pwm = PWM_STOP_US;
    _last_rear_pwm = PWM_STOP_US;
    _last_front_on = false;
    _last_rear_on = false;
    _last_power_pct = 0;
    _last_log_ms = 0;
}

void AP_Brush::set_active(bool active)
{
    _active = active;
    if (!_active) {
        write_outputs(false, false, 0);
    } else {
        write_outputs(_front_on, _rear_on, _power_pct);
    }
}

void AP_Brush::update(bool front_on, bool rear_on, uint8_t power_pct)
{
    _front_on = front_on;
    _rear_on = rear_on;
    _power_pct = power_pct;

    if (!_active) {
        return;
    }

    write_outputs(_front_on, _rear_on, _power_pct);
}

void AP_Brush::stop_all()
{
    _front_on = false;
    _rear_on = false;
    _power_pct = 0;
    write_outputs(false, false, 0);
}

uint16_t AP_Brush::calc_pwm_us(bool on, uint8_t power_pct) const
{
    if (!on || power_pct == 0) {
        return PWM_STOP_US;
    }

    const float scaled = constrain_float(float(power_pct) / 100.0f, 0.0f, 1.0f);
    return uint16_t(PWM_STOP_US + scaled * float(PWM_MAX_US - PWM_STOP_US));
}

void AP_Brush::write_outputs(bool front_on, bool rear_on, uint8_t power_pct)
{
    const uint16_t front_pwm = calc_pwm_us(front_on, power_pct);
    const uint16_t rear_pwm = calc_pwm_us(rear_on, power_pct);

    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_brush_front, front_pwm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_vgsolar_brush_rear, rear_pwm);

    log_brush_status(front_on, rear_on, power_pct, front_pwm, rear_pwm);

    _last_front_pwm = front_pwm;
    _last_rear_pwm = rear_pwm;
}

void AP_Brush::log_brush_status(bool front_on, bool rear_on, uint8_t power_pct,
                                uint16_t front_pwm, uint16_t rear_pwm)
{
    const uint32_t now = AP_HAL::millis();
    const bool state_changed =
        front_on != _last_front_on ||
        rear_on != _last_rear_on ||
        power_pct != _last_power_pct;
    const bool pwm_changed =
        front_pwm != _last_front_pwm ||
        rear_pwm != _last_rear_pwm;

    if (!state_changed && !pwm_changed) {
        return;
    }
    if (!state_changed && (now - _last_log_ms) < LOG_INTERVAL_MS) {
        return;
    }

    _last_front_on = front_on;
    _last_rear_on = rear_on;
    _last_power_pct = power_pct;
    _last_log_ms = now;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "VG_BRUSH: front=%u rear=%u pwr=%u pwm=%u/%u",
                    unsigned(front_on), unsigned(rear_on), unsigned(power_pct),
                    unsigned(front_pwm), unsigned(rear_pwm));
}

namespace AP {

AP_Brush &brush()
{
    return *AP_Brush::get_singleton();
}

}

static AP_Brush ap_brush_singleton;
