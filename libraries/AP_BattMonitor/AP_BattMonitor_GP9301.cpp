#include "AP_BattMonitor_GP9301.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

/*
  "battery" monitor for liquid fuel level systems that give a PWM value indicating quantity of remaining fuel.

  Output is:

    - mAh remaining is fuel level in millilitres
    - consumed mAh is in consumed millilitres
    - fixed 1.0v voltage
 */
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_GP9301::AP_BattMonitor_GP9301(
    AP_BattMonitor& mon, AP_BattMonitor::BattMonitor_State& mon_state, AP_BattMonitor_Params& params)
    : AP_BattMonitor_Analog(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v

    // need to add check
    _state.healthy = false;
}

/*
  read - read the "voltage" and "current"
*/
void AP_BattMonitor_GP9301::read()
{
    if (!volt_pin_pwm_source.set_pin(_volt_pin, "Volt_PWM")) {
        _state.healthy = false;
        return;
    }

    uint16_t volt_pulse_width = volt_pin_pwm_source.get_pwm_us();

    // hal.console->printf("volt_pulse_width=%d\r\n", (int)volt_pulse_width);

    /*
      this driver assumes that CAPACITY is set to tank volume in millilitres.
     */
    const uint16_t volt_pwm_empty = 100;
    const uint16_t volt_pwm_full = 20000;

    uint32_t now_us = AP_HAL::micros();

    float dt = now_us - _state.last_time_micros;

    // check for invalid pulse
    if (volt_pulse_width <= volt_pwm_empty || volt_pulse_width >= volt_pwm_full) {
        _state.healthy = (now_us - _state.last_time_micros) < 250000U;
        return;
    }
    volt_pulse_width = constrain_int16(volt_pulse_width, volt_pwm_empty, volt_pwm_full);

    _state.voltage = (float)volt_pulse_width * 0.001f * 0.1f * 5.0f * 10.0f * _volt_multiplier;

    // read current
    if (has_current()) {
        if (!curr_pin_pwm_source.set_pin(_volt_pin, "Curr_PWM")) {
            _state.healthy = false;
            return;
        }

        uint16_t curr_pulse_width = curr_pin_pwm_source.get_pwm_us();

        // read current
        _state.current_amps = (curr_pulse_width - _curr_amp_offset) * _curr_amp_per_volt;

        // update total current drawn since startup
        float mah = _state.current_amps * dt * 0.0000002778f;
        _state.consumed_mah += mah;
        _state.consumed_wh += 0.001f * mah * _state.voltage;
    }

    _state.last_time_micros = now_us;

    _state.healthy = true;
}

/// return true if battery provides current info
bool AP_BattMonitor_GP9301::has_current() const
{
    return ((AP_BattMonitor::Type)_params._type.get() == AP_BattMonitor::Type::PWM_VOLTAGE_AND_CURRENT);
}
