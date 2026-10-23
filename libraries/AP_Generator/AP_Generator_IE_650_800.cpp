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

#pragma GCC optimize("Os")

#include "AP_Generator_IE_650_800.h"

#if AP_GENERATOR_IE_650_800_ENABLED

extern const AP_HAL::HAL& hal;

void AP_Generator_IE_650_800::init()
{
    // Call init from base class to do common setup
    AP_Generator_IE_FuelCell::init();

    // Tell frontend what measurements are available for this generator
    // This unit does not have current but this needs to be true to make use of consumed_mah in BattMonitor
    _frontend._has_current = true;
    _frontend._has_consumed_energy = true;
    _frontend._has_fuel_remaining = true;
}

// Update fuel cell, expected to be called at 20hz
void AP_Generator_IE_650_800::assign_measurements(const uint32_t now)
{
    // Successfully decoded a new valid sentence
    // Update internal fuel cell state
    _state = (State)_parsed.state;
    _err_code = _parsed.err_code;

    // Update variables to be returned to front end
    _fuel_remaining = _parsed.tank_pct * 0.01;

    // Invert bat remaining percent to match AP_BattMonitor convention
    _consumed_mah = 100.0f - _parsed.battery_pct;

    // This unit does not report voltage, always report 1 volt
    _voltage = 1;

    _last_time_ms = now;
}

// Process characters received and extract terms for IE 650/800 W
void AP_Generator_IE_650_800::decode_latest_term()
{
    // Null terminate and move onto the next term
    _term[_term_offset] = 0;
    _term_offset = 0;
    _term_number++;

    if (_start_char != '<') {
        _sentence_valid = false;
        return;
    }

    switch (_term_number) {
        case 1:
            _parsed.tank_pct = strtof(_term, NULL);
            // Out of range values
            if (_parsed.tank_pct > 100.0f || _parsed.tank_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 2:
            _parsed.battery_pct = strtof(_term, NULL);
            // Out of range values
            if (_parsed.battery_pct > 100.0f || _parsed.battery_pct < 0.0f) {
                _data_valid = false;
            }
            break;

        case 3:
            _parsed.state = strtoul(_term, nullptr, 10);
            break;
        
        case 4:
            _parsed.err_code = strtoul(_term, nullptr, 16);
            // Sentence only declared valid when we have the expected number of terms
            _sentence_valid = _data_valid;
            break;

        default:
            // We have received more terms than, something has gone wrong with telemetry data, mark invalid sentence
            _sentence_valid = false;
            break;
    }
}

// Check error codes and populate message with error code
bool AP_Generator_IE_650_800::check_for_err_code(char* msg_txt, uint8_t msg_len) const
{
    // Check for any valid error codes
    if ((_err_code & (fs_crit_mask | fs_low_mask)) == 0) {
        return false;
    }

    // Error codes are converted to hex to make it easier to compare to user manual for these units
    hal.util->snprintf(msg_txt, msg_len, "Fuel cell err code <0x%x>", (unsigned)_err_code);
    return true;
}

// Check for failsafes
AP_BattMonitor::Failsafe AP_Generator_IE_650_800::update_failsafes() const
{
    // Check if we are in a critical failsafe
    if ((_err_code & fs_crit_mask) != 0) {
        return  AP_BattMonitor::Failsafe::Critical;
    }

    // Check if we are in a low failsafe
    if ((_err_code & fs_low_mask) != 0) {
        return  AP_BattMonitor::Failsafe::Low;
    }

    return AP_BattMonitor::Failsafe::None;
}

#endif  // AP_GENERATOR_IE_650_800_ENABLED

