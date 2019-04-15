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
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

/*
  base class constructor.
  This incorporates initialisation as well.
*/
AP_BattMonitor_Backend::AP_BattMonitor_Backend(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state,
                                               AP_BattMonitor_Params &params) :
        _mon(mon),
        _state(mon_state),
        _params(params)
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

float AP_BattMonitor_Backend::voltage_resting_estimate() const
{
    // resting voltage should always be greater than or equal to the raw voltage
    return MAX(_state.voltage, _state.voltage_resting_estimate);
}

AP_BattMonitor::BatteryFailsafe AP_BattMonitor_Backend::update_failsafes(void)
{
    const uint32_t now = AP_HAL::millis();

    bool low_voltage, low_capacity, critical_voltage, critical_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity);

    if (critical_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.critical_voltage_start_ms == 0) {
            _state.critical_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.critical_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::BatteryFailsafe_Critical;
        }
    } else {
        // acceptable voltage so reset timer
        _state.critical_voltage_start_ms = 0;
    }

    if (critical_capacity) {
        return AP_BattMonitor::BatteryFailsafe_Critical;
    }

    if (low_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.low_voltage_start_ms == 0) {
            _state.low_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.low_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::BatteryFailsafe_Low;
        }
    } else {
        // acceptable voltage so reset timer
        _state.low_voltage_start_ms = 0;
    }

    if (low_capacity) {
        return AP_BattMonitor::BatteryFailsafe_Low;
    }

    // if we've gotten this far then battery is ok
    return AP_BattMonitor::BatteryFailsafe_None;
}

static bool update_check(size_t buflen, char *buffer, bool failed, const char *message)
{
    if (failed) {
        strncpy(buffer, message, buflen);
        return false;
    }
    return true;
}

bool AP_BattMonitor_Backend::arming_checks(char * buffer, size_t buflen) const
{
    bool low_voltage, low_capacity, critical_voltage, critical_capacity;
    check_failsafe_types(low_voltage, low_capacity, critical_voltage, critical_capacity);

    bool below_arming_voltage = is_positive(_params._arming_minimum_voltage) &&
                                (_state.voltage < _params._arming_minimum_voltage);
    bool below_arming_capacity = (_params._arming_minimum_capacity > 0) &&
                                 ((_params._pack_capacity - _state.consumed_mah) < _params._arming_minimum_capacity);

    bool result = update_check(buflen, buffer, low_voltage,  "low voltage failsafe");
    result = result && update_check(buflen, buffer, low_capacity, "low capacity failsafe");
    result = result && update_check(buflen, buffer, critical_voltage, "critical voltage failsafe");
    result = result && update_check(buflen, buffer, critical_capacity, "critical capacity failsafe");
    result = result && update_check(buflen, buffer, below_arming_voltage, "below minimum arming voltage");
    result = result && update_check(buflen, buffer, below_arming_capacity, "below minimum arming capacity");

    return result;
}

void AP_BattMonitor_Backend::check_failsafe_types(bool &low_voltage, bool &low_capacity, bool &critical_voltage, bool &critical_capacity) const
{
    // use voltage or sag compensated voltage
    float voltage_used;
    switch (_params.failsafe_voltage_source()) {
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_Raw:
        default:
            voltage_used = _state.voltage;
            break;
        case AP_BattMonitor_Params::BattMonitor_LowVoltageSource_SagCompensated:
            voltage_used = voltage_resting_estimate();
            break;
    }

    // check critical battery levels
    if ((voltage_used > 0) && (_params._critical_voltage > 0) && (voltage_used < _params._critical_voltage)) {
        critical_voltage = true;
    } else {
        critical_voltage = false;
    }

    // check capacity failsafe if current monitoring is enabled
    if (has_current() && (_params._critical_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._critical_capacity)) {
        critical_capacity = true;
    } else {
        critical_capacity = false;
    }

    if ((voltage_used > 0) && (_params._low_voltage > 0) && (voltage_used < _params._low_voltage)) {
        low_voltage = true;
    } else {
        low_voltage = false;
    }

    // check capacity if current monitoring is enabled
    if (has_current() && (_params._low_capacity > 0) &&
        ((_params._pack_capacity - _state.consumed_mah) < _params._low_capacity)) {
        low_capacity = true;
    } else {
        low_capacity = false;
    }
}
