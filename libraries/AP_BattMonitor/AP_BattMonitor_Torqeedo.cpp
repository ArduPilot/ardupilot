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


#include "AP_BattMonitor_config.h"

#if AP_BATTERY_TORQEEDO_ENABLED

#include "AP_BattMonitor_Torqeedo.h"

#include <AP_HAL/AP_HAL.h>

#define AP_BATTMON_TORQEEDO_TIMEOUT_US 5000000

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_Torqeedo::read(void)
{
    AP_Torqeedo *torqeedo = AP_Torqeedo::get_singleton();
    if (torqeedo == nullptr) {
        _state.healthy = false;
        return;
    }

    // get voltage, current, temp and remaining capacity percentage
    float volts;
    float current_amps;
    float temp_C;
    if (torqeedo->get_batt_info(volts, current_amps, temp_C, remaining_pct)) {
        have_info = true;
        _state.voltage = volts;
        _state.current_amps = current_amps;
        _state.temperature = temp_C;
        _state.temperature_time = AP_HAL::millis();

        // update total current draw
        const uint32_t tnow_us = AP_HAL::micros();
        const uint32_t diff_us = tnow_us - _state.last_time_micros;
        if (diff_us < AP_BATTMON_TORQEEDO_TIMEOUT_US) {
            _state.consumed_mah += _state.current_amps * diff_us * 1e-6f / 3600.0 * 1000.0;
        }
        _state.last_time_micros = tnow_us;
        _state.healthy = true;
    }

    // read battery pack capacity
    if (!have_capacity) {
        uint16_t batt_capacity_ah;
        if (torqeedo->get_batt_capacity_Ah(batt_capacity_ah)) {
            have_capacity = true;
            if (batt_capacity_ah * 1000 != _params._pack_capacity) {
                _params._pack_capacity.set_and_notify(batt_capacity_ah * 1000);
            }
        }
    }
}

// capacity_remaining_pct - returns true if the battery % is available and writes to the percentage argument
bool AP_BattMonitor_Torqeedo::capacity_remaining_pct(uint8_t &percentage) const
{
    if (have_info) {
        percentage = remaining_pct;
    }
    return have_info;
}

#endif // AP_BATTERY_TORQEEDO_ENABLED
