/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state) :
    AP_BattMonitor_Backend(mon, instance, mon_state)
{
    _volt_pin_analog_source = hal.analogin->channel(mon._volt_pin[instance]);
    _curr_pin_analog_source = hal.analogin->channel(mon._curr_pin[instance]);

    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // this copes with changing the pin at runtime
    _volt_pin_analog_source->set_pin(_mon._volt_pin[_state.instance]);

    // get voltage
    _state.voltage = _volt_pin_analog_source->voltage_average() * _mon._volt_multiplier[_state.instance];

    // read current
    if (_mon.has_current(_state.instance)) {
        // calculate time since last current read
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _curr_pin_analog_source->set_pin(_mon._curr_pin[_state.instance]);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average()-_mon._curr_amp_offset[_state.instance])*_mon._curr_amp_per_volt[_state.instance];

        // update total current drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            _state.current_total_mah += _state.current_amps * dt * 0.0000002778f;
        }

        // record time
        _state.last_time_micros = tnow;
    }
}
