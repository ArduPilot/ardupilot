#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Sum.h"

/*
  battery monitor that is the sum of other battery monitors after this one

  This can be used to combined other current/voltage sensors into a
  single backend
 */
extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_Sum::AP_BattMonitor_Sum(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       uint8_t instance) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{
}

// read - read the voltage and current
void
AP_BattMonitor_Sum::read()
{
    float voltage_sum = 0;
    uint8_t voltage_count = 0;
    float current_sum = 0;
    uint8_t current_count = 0;

    for (uint8_t i=_instance+1; i<_mon.num_instances(); i++) {
        if (!_mon.healthy(i)) {
            continue;
        }
        voltage_sum += _mon.voltage(i);
        voltage_count++;
        float current;
        if (_mon.current_amps(current, i)) {
            current_sum += current;
            current_count++;
        }
    }
    if (voltage_count > 0) {
        _state.voltage = voltage_sum / voltage_count;
        _state.last_time_micros = AP_HAL::micros();
    }
    if (current_count > 0) {
        _state.current_amps = current_sum;
    }
    _has_current = (current_count > 0);
    _state.healthy = (voltage_count > 0);
}
