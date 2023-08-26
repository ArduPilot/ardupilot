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

#if AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
#include "AP_ESC_Telem/AP_ESC_Telem.h"
#endif

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

// capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
// return false if the battery is unhealthy, does not have current monitoring, or the pack_capacity is too small
bool AP_BattMonitor_Backend::capacity_remaining_pct(uint8_t &percentage) const
{
    // we consider anything under 10 mAh as being an invalid capacity and so will be our measurement of remaining capacity
    if ( _params._pack_capacity <= 10) {
        return false;
    }

    // the monitor must have current readings in order to estimate consumed_mah and be healthy
    if (!has_current() || !_state.healthy) {
        return false;
    }

    const float mah_remaining = _params._pack_capacity - _state.consumed_mah;
    percentage = constrain_float(100 * mah_remaining / _params._pack_capacity, 0, UINT8_MAX);
    return true;
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
    float loop_interval = (now - _resistance_timer_ms) * 0.001f;
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

AP_BattMonitor::Failsafe AP_BattMonitor_Backend::update_failsafes(void)
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
            return AP_BattMonitor::Failsafe::Critical;
        }
    } else {
        // acceptable voltage so reset timer
        _state.critical_voltage_start_ms = 0;
    }

    if (critical_capacity) {
        return AP_BattMonitor::Failsafe::Critical;
    }

    if (low_voltage) {
        // this is the first time our voltage has dropped below minimum so start timer
        if (_state.low_voltage_start_ms == 0) {
            _state.low_voltage_start_ms = now;
        } else if (_params._low_voltage_timeout > 0 &&
                   now - _state.low_voltage_start_ms > uint32_t(_params._low_voltage_timeout)*1000U) {
            return AP_BattMonitor::Failsafe::Low;
        }
    } else {
        // acceptable voltage so reset timer
        _state.low_voltage_start_ms = 0;
    }

    if (low_capacity) {
        return AP_BattMonitor::Failsafe::Low;
    }

    // if we've gotten this far then battery is ok
    return AP_BattMonitor::Failsafe::None;
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
    bool fs_capacity_inversion = is_positive(_params._critical_capacity) &&
                                 is_positive(_params._low_capacity) &&
                                 !(_params._low_capacity > _params._critical_capacity);
    bool fs_voltage_inversion = is_positive(_params._critical_voltage) &&
                                is_positive(_params._low_voltage) &&
                                !(_params._low_voltage > _params._critical_voltage);

    bool result = update_check(buflen, buffer, !_state.healthy, "unhealthy");
    result = result && update_check(buflen, buffer, below_arming_voltage, "below minimum arming voltage");
    result = result && update_check(buflen, buffer, below_arming_capacity, "below minimum arming capacity");
    result = result && update_check(buflen, buffer, low_voltage,  "low voltage failsafe");
    result = result && update_check(buflen, buffer, low_capacity, "low capacity failsafe");
    result = result && update_check(buflen, buffer, critical_voltage, "critical voltage failsafe");
    result = result && update_check(buflen, buffer, critical_capacity, "critical capacity failsafe");
    result = result && update_check(buflen, buffer, fs_capacity_inversion, "capacity failsafe critical >= low");
    result = result && update_check(buflen, buffer, fs_voltage_inversion, "voltage failsafe critical >= low");

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

#if AP_BATTERY_ESC_TELEM_OUTBOUND_ENABLED
void AP_BattMonitor_Backend::update_esc_telem_outbound()
{
    const uint8_t esc_index = _params._esc_telem_outbound_index;
    if (esc_index == 0 || !_state.healthy) {
        // Disabled if there's no ESC identified to route the data to or if the battery is unhealthy
        return;
    }

    AP_ESC_Telem_Backend::TelemetryData telem {};

    uint16_t type = AP_ESC_Telem_Backend::TelemetryType::VOLTAGE;
    telem.voltage = _state.voltage; // all battery backends have voltage

    if (has_current()) {
        telem.current = _state.current_amps;
        type |= AP_ESC_Telem_Backend::TelemetryType::CURRENT;
    }

    if (has_consumed_energy()) {
        telem.consumption_mah = _state.consumed_mah;
        type |= AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION;
    }

    float temperature_c;
    if (_mon.get_temperature(temperature_c, _state.instance)) {
        // get the temperature from the frontend so we check for external temperature
        telem.temperature_cdeg = temperature_c * 100;
        type |= AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE;
    }

    AP::esc_telem().update_telem_data(esc_index-1, telem, type);
}
#endif


/*
  default implementation for reset_remaining(). This sets consumed_wh
  and consumed_mah based on the given percentage. Use percentage=100
  for a full battery
*/
bool AP_BattMonitor_Backend::reset_remaining(float percentage)
{
    percentage = constrain_float(percentage, 0, 100);
    const float used_proportion = (100.0f - percentage) * 0.01f;
    _state.consumed_mah = used_proportion * _params._pack_capacity;
    // without knowing the history we can't do consumed_wh
    // accurately. Best estimate is based on current voltage. This
    // will be good when resetting the battery to a value close to
    // full charge
    _state.consumed_wh = _state.consumed_mah * 0.001f * _state.voltage;

    // reset failsafe state for this backend
    _state.failsafe = update_failsafes();

    return true;
}

/*
  update consumed mAh and Wh
 */
void AP_BattMonitor_Backend::update_consumed(AP_BattMonitor::BattMonitor_State &state, uint32_t dt_us)
{
    // update total current drawn since startup
    if (state.last_time_micros != 0 && dt_us < 2000000) {
        const float mah = calculate_mah(state.current_amps, dt_us);
        state.consumed_mah += mah;
        state.consumed_wh  += 0.001 * mah * state.voltage;
    }
}
