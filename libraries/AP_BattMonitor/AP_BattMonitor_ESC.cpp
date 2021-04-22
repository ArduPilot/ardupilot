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
#include "AP_BattMonitor_ESC.h"

#if HAL_WITH_ESC_TELEM

extern const AP_HAL::HAL &hal;

void AP_BattMonitor_ESC::init(void)
{
}

void AP_BattMonitor_ESC::read(void)
{
    AP_ESC_Telem& telem = AP::esc_telem();

    uint8_t voltage_escs = 0;     // number of ESCs with valid voltage
    uint8_t temperature_escs = 0; // number of ESCs with valid temperature
    float voltage_sum = 0;
    float current_sum = 0;
    float temperature_sum = 0;
    uint32_t highest_ms = 0;

    for (uint8_t i=0; i<ESC_TELEM_MAX_ESCS; i++) {
        int16_t  temperature_deg;
        float voltage;
        float current;
        float consumption_mah;

        if (telem.get_consumption_mah(i, consumption_mah)) {
            // accumulate consumed_sum regardless of age, to cope with ESC
            // dropping out
            _state.consumed_mah += consumption_mah;
        }

        if (telem.get_voltage(i, voltage)) {
            voltage_sum += voltage;
            voltage_escs++;
        }

        if (telem.get_current(i, current)) {
            current_sum += current;
        }

        if (telem.get_temperature(i, temperature_deg)) {
            temperature_sum += temperature_deg;
            temperature_escs++;
        }

        if (telem.get_last_telem_data_ms(i) > highest_ms) {
            highest_ms = telem.get_last_telem_data_ms(i);
        }
    }

    if (voltage_escs > 0) {
        _state.voltage = voltage_sum / voltage_escs;
        _state.healthy = true;
    } else {
        _state.voltage = 0;
        _state.healthy = false;
    }
    if (temperature_escs > 0) {
        _state.temperature = temperature_sum / temperature_escs;
    } else {
        _state.temperature = 0;
    }

    _state.current_amps = current_sum;
    _state.last_time_micros = highest_ms * 1000;
    _state.temperature_time = highest_ms;

    if (current_sum > 0) {
        // if we have ever got a current value then we know we have a
        // current sensor
        have_current = true;
    }
}

#endif // HAL_WITH_ESC_TELEM
