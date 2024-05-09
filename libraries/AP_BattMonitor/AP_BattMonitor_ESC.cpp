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

#if AP_BATTERY_ESC_ENABLED

#include "AP_BattMonitor_ESC.h"

const AP_Param::GroupInfo AP_BattMonitor_ESC::var_info[] = {

    // Param indexes must be between 36 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    // @Param: ESC_MASK
    // @DisplayName: ESC mask
    // @Description: If 0 all connected ESCs will be used. If non-zero, only those selected in will be used.
    // @Bitmask: 0: ESC 1, 1: ESC 2, 2: ESC 3, 3: ESC 4, 4: ESC 5, 5: ESC 6, 6: ESC 7, 7: ESC 8, 8: ESC 9, 9: ESC 10, 10: ESC 11, 11: ESC 12, 12: ESC 13, 13: ESC 14, 14: ESC 15, 15: ESC 16, 16: ESC 17, 17: ESC 18, 18: ESC 19, 19: ESC 20, 20: ESC 21, 21: ESC 22, 22: ESC 23, 23: ESC 24, 24: ESC 25, 25: ESC 26, 26: ESC 27, 27: ESC 28, 28: ESC 29, 29: ESC 30, 30: ESC 31, 31: ESC 32
    // @User: Standard
    AP_GROUPINFO("ESC_MASK", 36, AP_BattMonitor_ESC, _mask, 0),

    // Param indexes must be between 36 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

// constructor. This incorporates initialisation as well.
AP_BattMonitor_ESC::AP_BattMonitor_ESC(AP_BattMonitor &mon,
                                       AP_BattMonitor::BattMonitor_State &mon_state,
                                       AP_BattMonitor_Params &params):
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;
};

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
    _state.consumed_mah = delta_mah;

    const bool all_enabled = _mask == 0;
    for (uint8_t i=0; i<ESC_TELEM_MAX_ESCS; i++) {
        if (!all_enabled && ((_mask & (1U<<i)) == 0)) {
            // Only include ESCs set in mask
            continue;
        }

        int16_t  temperature_cdeg;
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

        if (telem.get_temperature(i, temperature_cdeg)) {
            temperature_sum += float(temperature_cdeg) * 0.01f;
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
        have_temperature = true;
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

bool AP_BattMonitor_ESC::reset_remaining(float percentage)
{
    delta_mah = 0.0f;
    read();
    const float current_mah = _state.consumed_mah;
    if (AP_BattMonitor_Backend::reset_remaining(percentage)) {
        delta_mah = _state.consumed_mah - current_mah;
        return true;
    }

    return false;
}

#endif // AP_BATTERY_ESC_ENABLED
