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


#include <AP_HAL/AP_HAL.h>
#include <AP_BLHeli/AP_BLHeli.h>

#ifdef HAVE_AP_BLHELI_SUPPORT

#include "AP_BattMonitor_BLHeliESC.h"

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_BLHeliESC::init(void)
{
}

void AP_BattMonitor_BLHeliESC::read(void)
{
    AP_BLHeli *blheli = AP_BLHeli::get_singleton();
    if (!blheli) {
        return;
    }

    uint8_t num_escs = 0;
    float voltage_sum=0;
    float current_sum=0;
    float consumed_sum=0;
    float temperature_sum=0;
    uint32_t now = AP_HAL::millis();
    uint32_t highest_ms=0;
    
    for (uint8_t i=0; i<AP_BLHELI_MAX_ESCS; i++) {
        AP_BLHeli::telem_data td;
        if (!blheli->get_telem_data(i, td)) {
            continue;
        }

        // accumulate consumed_sum regardless of age, to cope with ESC
        // dropping out
        consumed_sum += td.consumption;
        
        if (now - td.timestamp_ms > 1000) {
            // don't use old data
            continue;
        }
        
        num_escs++;
        voltage_sum += td.voltage;
        current_sum += td.current;
        temperature_sum += td.temperature;
        if (td.timestamp_ms > highest_ms) {
            highest_ms = td.timestamp_ms;
        }
    }

    if (num_escs > 0) {
        _state.voltage = (voltage_sum / num_escs) * 0.01;
        _state.temperature = temperature_sum / num_escs;
        _state.healthy = true;
    } else {
        _state.voltage = 0;
        _state.temperature = 0;
        _state.healthy = false;
    }
    _state.current_amps = current_sum * 0.01;
    _state.consumed_mah = consumed_sum;
    _state.last_time_micros = highest_ms * 1000;
    _state.temperature_time = highest_ms;

    if (current_sum > 0) {
        // if we have ever got a current value then we know we have a
        // current sensor
        have_current = true;
    }
}

#endif // HAVE_AP_BLHELI_SUPPORT
