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
    // Note: capacity<=0 means **unlimited** capacity.
    void setup(float _capacity_Ah, float _resistance_ohm, float _max_voltage, float _ambient_temperature_degC);

    // Resets the battery state if the configuration (e.g. from SIM_BATT_* parameters) has changed.
    void maybe_reset(float desired_voltage, float desired_capacity_Ah);

    // Call this periodically to "step" the battery forward in time
    void consume_energy(float current_amp, uint64_t now_us);

    float get_voltage(void) const { return voltage_filter.get(); }
    float get_capacity(void) const { return capacity_Ah; }
    float get_temperature_degC(void) const { return temperature_degC; }
    bool capacity_is_unlimited(void) const { return !(is_positive(capacity_Ah)); } // for readability

private:
    float capacity_Ah; // set <= 0 for unlmited capacity
    float resistance_ohm;
    float max_voltage;
    float ambient_temperature_degC;
    float voltage_set;
    float remaining_Ah; // if capacity is unlimited, this is FLT_MAX
    uint64_t last_us;

    float temperature_degC = 0.0f;
    void update_temperature(float current_amp, float dt);

    // 10Hz filter for battery voltage
    LowPassFilterFloat voltage_filter{10};

    float get_resting_voltage(void) const;
    float compute_remaining_Ah(float voltage) const;
};
}
