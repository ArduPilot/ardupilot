#pragma once

#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>

/*
  VGSolar front/rear roll brush PWM output.
  NCU runtime params (0x0101~0x0104) are applied via AP_CompanionComputer;
  PWM is only driven while VGSL mode is active.
 */
class AP_Brush {
public:
    AP_Brush();

    /* Do not allow copies */
    AP_Brush(const AP_Brush &other) = delete;
    AP_Brush &operator=(const AP_Brush&) = delete;

    static AP_Brush *get_singleton() { return _singleton; }

    // VGSL mode enter/leave
    void set_active(bool active);

    // Apply desired brush state; outputs PWM only when active
    void update(bool front_on, bool rear_on, uint8_t power_pct);

    // Force neutral PWM and clear desired state
    void stop_all();

private:
    static AP_Brush *_singleton;

    bool _active;
    bool _front_on;
    bool _rear_on;
    uint8_t _power_pct;

    uint16_t _last_front_pwm;
    uint16_t _last_rear_pwm;
    bool _last_front_on;
    bool _last_rear_on;
    uint8_t _last_power_pct;
    uint32_t _last_log_ms;

    static constexpr uint32_t LOG_INTERVAL_MS = 2000;

    uint16_t calc_pwm_us(bool on, uint8_t power_pct, SRV_Channel::Aux_servo_function_t function) const;
    void write_outputs(bool front_on, bool rear_on, uint8_t power_pct);
    void log_brush_status(bool front_on, bool rear_on, uint8_t power_pct, uint16_t front_pwm, uint16_t rear_pwm);
};

namespace AP {
    AP_Brush &brush();
}
