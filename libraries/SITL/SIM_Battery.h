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

constexpr float ambient_temperature = 0.0f;  // degC

class Battery {
public:
    // Note: capacity<=0 means **unlimited** capacity.
    void setup(float _capacity_Ah, float _resistance_ohm, float _max_voltage);

    // Implements the (re)setting behavior described by SIM_BATT_* parameters
    // (reminder: if the desired values are different than previous choices, reset)
    void maybe_reset(float desired_voltage, float desired_capacity_Ah);

    // Call this periodically to "step" the battery forward in time
    void consume_energy(float current_amps);

    float get_voltage(void) const { return voltage_filter.get(); }

    // return battery temperature in Kelvin:
    float get_temperature(void) const { return temperature.kelvin; }

private:
    float capacity_Ah; // set <= 0 for unlmited capacity
    float resistance_ohm;
    float max_voltage;
    float voltage_set;
    float remaining_Ah; // if capacity is unlimited, set this to FLT_MAX
    uint64_t last_us;

    struct {
        float kelvin = ambient_temperature + 273.15f;
        uint64_t last_update_micros;
    } temperature;
    void update_temperature(float current_amps, uint64_t now);

    // 10Hz filter for battery voltage
    LowPassFilterFloat voltage_filter{10};

    float get_resting_voltage(void) const;
    float compute_remaining_ah(float voltage) const; // const shows this has no side effects
    void set_remaining_ah(void);
    bool capacity_is_unlimited(void) const { return !(is_positive(capacity_Ah)); } // for readability
};
}
