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

const AP_Param::GroupInfo AP_BattMonitor_Sum::var_info[] = {

    // Param indexes must be between 20 and 29 to avoid conflict with other battery monitor param tables loaded by pointer

    // @Param: SUM_MASK
    // @DisplayName: Battery Sum mask
    // @Description: 0: sum of remaining battery monitors, If none 0 sum of specified monitors. Current will be summed and voltages averaged.
    // @Bitmask: 0:monitor 1, 1:monitor 2, 2:monitor 3, 3:monitor 4, 4:monitor 5, 5:monitor 6, 6:monitor 7, 7:monitor 8, 8:monitor 9
    // @User: Standard
    AP_GROUPINFO("SUM_MASK", 20, AP_BattMonitor_Sum, _sum_mask, 0),

    // Param indexes must be between 20 and 29 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Sum::AP_BattMonitor_Sum(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params,
                                       uint8_t instance) :
    AP_BattMonitor_Backend(mon, mon_state, params),
    _instance(instance)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
}

// read - read the voltage and current
void
AP_BattMonitor_Sum::read()
{
    float voltage_sum = 0;
    uint8_t voltage_count = 0;
    float current_sum = 0;
    uint8_t current_count = 0;

    for (uint8_t i=0; i<_mon.num_instances(); i++) {
        if (i == _instance) {
            // never include self
            continue;
        }
        if ((_sum_mask == 0) && (i <= _instance)) {
            // sum of remaining, skip lower instances
            continue;
        }
        if ((_sum_mask != 0) && ((_sum_mask & 1U<<i) == 0)) {
            // mask param, skip if mask bit not set
            continue;
        }
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
    const uint32_t tnow_us = AP_HAL::micros();
    const uint32_t dt_us = tnow_us - _state.last_time_micros;

    if (voltage_count > 0) {
        _state.voltage = voltage_sum / voltage_count;
    }
    if (current_count > 0) {
        _state.current_amps = current_sum;
    }

    update_consumed(_state, dt_us);

    _has_current = (current_count > 0);
    _state.healthy = (voltage_count > 0);

    if (_state.healthy) {
        _state.last_time_micros = tnow_us;
    }
}
