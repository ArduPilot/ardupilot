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
/*
  battery model for electric aircraft
*/

#pragma once

#include <Filter/LowPassFilter.h>

namespace SITL {

class Battery {
public:
    void setup(float _capacity_Ah, float _resistance_ohm, float _max_voltage);

    // Resets the battery state if the configuration (e.g. from SIM_BATT_* parameters) has changed.
    void maybe_reset(float desired_voltage, float desired_capacity_Ah);

    void init_voltage(float voltage);
    void init_capacity(float capacity);

    // Call this periodically to "step" the battery forward in time
    void consume_energy(float current_amp, uint64_t now_us);

    float get_voltage(void) const { return voltage_filter.get(); }
    float get_capacity(void) const { return capacity_Ah; }

    // return battery temperature in Kelvin:
    float get_temperature(void) const { return temperature.kelvin; }

private:
    float capacity_Ah;
    float resistance_ohm;
    float max_voltage;
    float voltage_set;
    float remaining_Ah;
    uint64_t last_us;

    struct {
        float kelvin = 273;
        uint64_t last_update_us;
    } temperature;
    void update_temperature(float current_amp, uint64_t now_us);

    // 10Hz filter for battery voltage
    LowPassFilterFloat voltage_filter{10};

    float get_resting_voltage(float charge_pct) const;
    void set_initial_SoC(float voltage);
};
}
