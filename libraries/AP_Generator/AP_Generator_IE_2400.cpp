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

#include "AP_Generator_IE_2400.h"

#if AP_GENERATOR_IE2400_ENABLED

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

void AP_Generator_IE_2400::init()
{
    // Call init from base class to do common setup
    AP_Generator_IE_FuelCell::init();

    // Tell frontend what measurements are available for this generator
    _frontend._has_current = true;
    _frontend._has_consumed_energy = true;
    _frontend._has_fuel_remaining = true;
}

// Update fuel cell, expected to be called at 20hz
void AP_Generator_IE_2400::assign_measurements(const uint32_t now)
{
    // Successfully decoded a new valid sentence
    // Update internal fuel cell state
    _pwr_out = _parsed.pwr_out;
    _spm_pwr = _parsed.spm_pwr;

    _state = (State)_parsed.state;
    _err_code = _parsed.err_code;

    // Scale tank pressure linearly to a value between 0 and 1
    // Min = 5 bar, max = 300 bar, PRESS_GRAD = 1/295.
    const float PRESS_GRAD = 0.003389830508f;
    _fuel_remaining = constrain_float((_parsed.tank_bar-5)*PRESS_GRAD,0,1);

    // Update battery voltage
    _voltage = _parsed.battery_volt;

    /* Calculate battery current. Convention: +current battery is discharging, -current
    battery is charging.  This is aligned with normal AP behaviour.  This is the opposite
    of IE's convention hence *-1 */
    if (_parsed.battery_volt > 0) {
        _current = -1 * _parsed.battery_pwr / _parsed.battery_volt;
    } else {
        _current = 0;
    }

    // Calculate consumed current
    _consumed_mah += _current * (now - _last_time_ms) * AMS_TO_MAH;

    _last_time_ms = now;
}

// Process characters received and extract terms for IE 2.4kW
void AP_Generator_IE_2400::decode_latest_term()
{
    // Null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;

    switch (_term_number) {
        case 1:
            // Float
            _parsed.tank_bar = strtof(_term, NULL);
            break;

        case 2:
            // Float
            _parsed.battery_volt = strtof(_term, NULL);
            break;

        case 3:
            // Signed int base 10
            _parsed.pwr_out = strtol(_term, nullptr, 10);
            break;

        case 4:
            // Unsigned int base 10
            _parsed.spm_pwr = strtoul(_term, nullptr, 10);
            break;

        case 5:
            // Signed int base 10
            _parsed.battery_pwr = strtol(_term, nullptr, 10);
            break;

        case 6:
            // Unsigned int base 10
            _parsed.state = strtoul(_term, nullptr, 10);
            break;

        case 7:
            // Unsigned int base 10
            _parsed.err_code = strtoul(_term, nullptr, 10);
            // Sentence only declared valid when we have the expected number of terms
            _sentence_valid = true;
            break;

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }
}

// Check for failsafes
AP_BattMonitor::Failsafe AP_Generator_IE_2400::update_failsafes() const
{
    // Check for error codes that lead to critical action battery monitor failsafe
    if (is_critical_error(_err_code)) {
        return AP_BattMonitor::Failsafe::Critical;
    }

    // Check for error codes that lead to low action battery monitor failsafe
    if (is_low_error(_err_code)) {
        return AP_BattMonitor::Failsafe::Low;
    }

    return AP_BattMonitor::Failsafe::None;
}

// Check for error codes that are deemed critical
bool AP_Generator_IE_2400::is_critical_error(const uint32_t err_in) const
{
    switch ((ErrorCode)err_in) {
        // Error codes that lead to critical action battery monitor failsafe
        case ErrorCode::BATTERY_CRITICAL:
        case ErrorCode::PRESSURE_CRITICAL:
        case ErrorCode::SYSTEM_CRITICAL:
            return true;

        default:
            // Minor internal error is always ignored and caught by the default
            return false;
    }
}

// Check for error codes that are deemed severe and would be cause to trigger a battery monitor low failsafe action
bool AP_Generator_IE_2400::is_low_error(const uint32_t err_in) const
{
    switch ((ErrorCode)err_in) {
        // Error codes that lead to critical action battery monitor failsafe
        case ErrorCode::START_DENIED:
        case ErrorCode::PRESSURE_ALERT:
        case ErrorCode::BATTERY_LOW:
        case ErrorCode::PRESSURE_LOW:
        case ErrorCode::SPM_LOST:
        case ErrorCode::REDUCED_POWER:
            return true;

        default:
            // Minor internal error is always ignored and caught by the default
            return false;
    }
}

// Check error codes and populate message with error code
bool AP_Generator_IE_2400::check_for_err_code(char* msg_txt, uint8_t msg_len) const
{
    // Check if we have received an error code
    if (!is_critical_error(_err_code) && !is_low_error(_err_code)) {
        return false;
    }

    hal.util->snprintf(msg_txt, msg_len, "Fuel cell err code <%u>", (unsigned)_err_code);
    return true;
}

// log generator status to the onboard log
void AP_Generator_IE_2400::log_write()
{
#define MASK_LOG_ANY    0xFFFF
    if (!AP::logger().should_log(MASK_LOG_ANY)) {
        return;
    }

    AP::logger().WriteStreaming(
        "IE24",
        "TimeUS,FUEL,SPMPWR,POUT,ERR",
        "s%WW-",
        "F2---",
        "Qfiii",
        AP_HAL::micros64(),
        _fuel_remaining,
        _spm_pwr,
        _pwr_out,
        _err_code
        );
}
#endif  // AP_GENERATOR_IE2400_ENABLED
