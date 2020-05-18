#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Math.h"

/*
  battery monitor that is the sum of other battery monitors after this one

  This can be used to combined other current/voltage sensors into a
  single backend
 */
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Math::AP_BattMonitor_Math(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       uint8_t instance) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{
}

// read - read the voltage and current
void AP_BattMonitor_Math::read()
{
    // this means we're targeting BATT1 vs BATT2.
    const uint8_t targets[2] = {0,1};

    if ((_instance == targets[0]) || (_instance == targets[1]) ||
        _instance >= _mon.num_instances() ||
        !_mon.healthy(targets[0]) || !_mon.healthy(targets[1]) )
    {
        _state.healthy = false;
        return;
    }

    const float voltage[2] = {_mon.voltage(targets[0]), _mon.voltage(targets[1])};

    float current[2] = {0.0f, 0.0f};
     _has_current = _mon.current_amps(current[0], targets[0]) && _mon.current_amps(current[1], targets[1]);

    switch (_params.type()) {
    case AP_BattMonitor_Params::BattMonitor_TYPE_Math_1_Plus_2:
        _state.voltage = voltage[0] + voltage[1];
        _state.current_amps = _has_current ? (current[0] + current[1]) : 0;
        break;
    case AP_BattMonitor_Params::BattMonitor_TYPE_Math_1_Minus_2:
        _state.voltage = voltage[0] - voltage[1];
        _state.current_amps = _has_current ? (current[0] - current[1]) : 0;
        break;
    case AP_BattMonitor_Params::BattMonitor_TYPE_Math_2_Minus_1:
        _state.voltage = voltage[1] - voltage[0];
        _state.current_amps = _has_current ? (current[1] - current[0]) : 0;
        break;

    default:
        _state.healthy = false;
        return;
    }

    const uint32_t tnow = AP_HAL::micros();
    const double dt = tnow - _state.last_time_micros;
    // update total current drawn since startup
    if (_has_current && (_state.last_time_micros != 0) && (dt < 2000000.0)) {
        // .0002778 is 1/3600 (conversion to hours)
        const double mah = _state.current_amps * (dt * 0.0000002778);
        _state.consumed_mah += mah;
        _state.consumed_wh  += 0.001 * mah * _state.voltage;
    }

    // record time
    _state.last_time_micros = tnow;
    _state.healthy = true;
}
