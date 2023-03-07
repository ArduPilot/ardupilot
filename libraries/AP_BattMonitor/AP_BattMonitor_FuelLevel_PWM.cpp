#include "AP_BattMonitor_config.h"

#if AP_BATTERY_FUELLEVEL_PWM_ENABLED

#include <AP_HAL/AP_HAL.h>

#include "AP_BattMonitor_FuelLevel_PWM.h"

/*
  "battery" monitor for liquid fuel level systems that give a PWM value indicating quantity of remaining fuel.

  Output is:

    - mAh remaining is fuel level in millilitres
    - consumed mAh is in consumed millilitres
    - fixed 1.0v voltage
 */
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_FuelLevel_PWM::AP_BattMonitor_FuelLevel_PWM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Analog(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v

    // need to add check
    _state.healthy = false;
}

/*
  read - read the "voltage" and "current"
*/
void AP_BattMonitor_FuelLevel_PWM::read()
{
    if (!pwm_source.set_pin(_curr_pin, "FuelLevelPWM")) {
        _state.healthy = false;
        return;
    }

    uint16_t pulse_width = pwm_source.get_pwm_us();

    /*
      this driver assumes that CAPACITY is set to tank volume in millilitres.
     */
    const uint16_t pwm_empty = 1100;
    const uint16_t pwm_full = 1900;
    const uint16_t pwm_buffer = 20;

    uint32_t now_us = AP_HAL::micros();

    // check for invalid pulse
    if (pulse_width <= (pwm_empty - pwm_buffer)|| pulse_width >= (pwm_full + pwm_buffer)) {
        _state.healthy = (now_us - _state.last_time_micros) < 250000U;
        return;
    }
    pulse_width = constrain_int16(pulse_width, pwm_empty, pwm_full);
    float proportion_full = (pulse_width - pwm_empty) / float(pwm_full - pwm_empty);
    float proportion_used = 1.0 - proportion_full;

    _state.last_time_micros = now_us;
    _state.healthy = true;

    // map consumed_mah to consumed millilitres
    _state.consumed_mah = proportion_used * _params._pack_capacity;

    // map consumed_wh using fixed voltage of 1
    _state.consumed_wh = _state.consumed_mah;
}

#endif  // AP_BATTERY_FUELLEVEL_PWM_ENABLED
