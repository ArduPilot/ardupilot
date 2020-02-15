#include <AP_HAL/AP_HAL.h>
#include "AP_BattMonitor_FuelLevel_PWM.h"
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
AP_BattMonitor_FuelLevel_PWM::AP_BattMonitor_FuelLevel_PWM(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    _state.voltage = 1.0; // show a fixed voltage of 1v

    // need to add check
    _state.healthy = false;
}

/*
  handle interrupt on an instance
 */
void AP_BattMonitor_FuelLevel_PWM::irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin_state == 1) {
        irq_state.last_pulse_us = timestamp;
    } else if (irq_state.last_pulse_us != 0) {
        irq_state.pulse_width_us = timestamp - irq_state.last_pulse_us;
        irq_state.pulse_count1 ++;
    }
}

/*
  read - read the "voltage" and "current"
*/
void AP_BattMonitor_FuelLevel_PWM::read()
{
    int8_t pin = _params._curr_pin;
    if (last_pin != pin) {
        // detach from last pin
        if (last_pin != -1) {
            hal.gpio->detach_interrupt(last_pin);
        }
        // attach to new pin
        last_pin = pin;
        if (last_pin > 0) {
            hal.gpio->pinMode(last_pin, HAL_GPIO_INPUT);
            if (!hal.gpio->attach_interrupt(
                    last_pin,
                    FUNCTOR_BIND_MEMBER(&AP_BattMonitor_FuelLevel_PWM::irq_handler, void, uint8_t, bool, uint32_t),
                    AP_HAL::GPIO::INTERRUPT_BOTH)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "FuelLevelPWM: Failed to attach to pin %u", unsigned(last_pin));
            }
        }
    }
    uint32_t now_us = AP_HAL::micros();
    if (pulse_count2 == irq_state.pulse_count1) {
        _state.healthy = (now_us - _state.last_time_micros) < 250000U;
        return;
    }
    uint32_t pulse_width = irq_state.pulse_width_us;
    pulse_count2 = irq_state.pulse_count1;

    /*
      this driver assumes that CAPACITY is set to tank volume in millilitres.
     */
    const uint16_t pwm_empty = 1100;
    const uint16_t pwm_full = 1900;
    const uint16_t pwm_buffer = 20;


    // check for invalid pulse
    if (pulse_width <= (pwm_empty - pwm_buffer)|| pulse_width >= (pwm_full + pwm_buffer)) {
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
