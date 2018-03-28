/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_BattMonitor_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params, uint8_t instance) :
        _mon(mon),
        _state(mon_state),
        _params(params),
        _instance(instance)
{
}

/// capacity_remaining_pct - returns the % battery capacity remaining (0 ~ 100)
uint8_t AP_BattMonitor_Backend::capacity_remaining_pct() const
{
    float mah_remaining = _params._pack_capacity - _state.consumed_mah;
    if ( _params._pack_capacity > 10 ) { // a very very small battery
        return (100 * (mah_remaining) / _params._pack_capacity);
    } else {
        return 0;
    }
}

// update battery resistance estimate
// faster rates of change of the current and voltage readings cause faster updates to the resistance estimate
// the battery resistance is calculated by comparing the latest current and voltage readings to a low-pass filtered current and voltage
// high current steps are integrated into the resistance estimate by varying the time constant of the resistance filter
void AP_BattMonitor_Backend::update_resistance_estimate()
{
    // return immediately if no current
    if (!has_current() || !is_positive(_state.current_amps)) {
        return;
    }

    // update maximum current seen since startup and protect against divide by zero
    _current_max_amps = MAX(_current_max_amps, _state.current_amps);
    float current_delta = _state.current_amps - _current_filt_amps;
    if (is_zero(current_delta)) {
        return;
    }

    // update reference voltage and current
    if (_state.voltage > _resistance_voltage_ref) {
        _resistance_voltage_ref = _state.voltage;
        _resistance_current_ref = _state.current_amps;
    }

    // calculate time since last update
    uint32_t now = AP_HAL::millis();
    float loop_interval = (now - _resistance_timer_ms) / 1000.0f;
    _resistance_timer_ms = now;

    // estimate short-term resistance
    float filt_alpha = constrain_float(loop_interval/(loop_interval + AP_BATT_MONITOR_RES_EST_TC_1), 0.0f, 0.5f);
    float resistance_alpha = MIN(1, AP_BATT_MONITOR_RES_EST_TC_2*fabsf((_state.current_amps-_current_filt_amps)/_current_max_amps));
    float resistance_estimate = -(_state.voltage-_voltage_filt)/current_delta;
    if (is_positive(resistance_estimate)) {
        _state.resistance = _state.resistance*(1-resistance_alpha) + resistance_estimate*resistance_alpha;
    }

    // calculate maximum resistance
    if ((_resistance_voltage_ref > _state.voltage) && (_state.current_amps > _resistance_current_ref)) {
        float resistance_max = (_resistance_voltage_ref - _state.voltage) / (_state.current_amps - _resistance_current_ref);
        _state.resistance = MIN(_state.resistance, resistance_max);
    }

    // update the filtered voltage and currents
    _voltage_filt = _voltage_filt*(1-filt_alpha) + _state.voltage*filt_alpha;
    _current_filt_amps = _current_filt_amps*(1-filt_alpha) + _state.current_amps*filt_alpha;

    // update estimated voltage without sag
    _state.voltage_resting_estimate = _state.voltage + _state.current_amps * _state.resistance;
}

bool AP_BattMonitor_Backend::extended_arming_checks(bool report) const
{
    char failure_str[41]; // only 50 characters fit in statustext, reserve 9 for "Battery X"
    float voltage;

    switch (_params.failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage = _state.voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage = _mon.voltage_resting_estimate(_instance);
            break;
    }

    if (_params._critical_voltage > 0 && voltage <= _params._critical_voltage) {
        hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "critical failsafe (%.2fV)", voltage);
        goto failed;
    }

    if (_params._low_voltage > 0 && voltage <= _params._low_voltage) {
        hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "low failsafe (%.2fV)", voltage);
        goto failed;
    }

    if (has_current()) {
        float remaining_mah = _params._pack_capacity - _state.consumed_mah;
        if (_params._critical_capacity > 0 && remaining_mah < _params._critical_capacity) {
            hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "critical failsafe (%.0f mAh)", remaining_mah);
            goto failed;
        }

        if (_params._low_capacity > 0 && remaining_mah < _params._low_capacity) {
            hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "low failsafe (%.0f mAh)", remaining_mah);
            goto failed;
        }
    }

    if (!_state.healthy) {
        hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "is unhealthy");
        goto failed;
    }

    if (_state.failsafe != AP_BattMonitor::BatteryFailsafe_None) {
        hal.util->snprintf(failure_str, ARRAY_SIZE(failure_str), "is in a %s failsafe",
                           _state.failsafe == AP_BattMonitor::BatteryFailsafe_Low ? "low" : "critical");
        goto failed;
    }

    return true;

failed:
    if (report) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "PreArm: Battery %d %s", _instance, failure_str);
    }

    return false;
}
