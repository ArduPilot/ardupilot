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

/*
  class to describe a motor position
 */
class Battery {
public:
    void setup(float _capacity_Ah, float _resistance, float _max_voltage);

    void init_voltage(float voltage);

    void set_current(float current_amps);
    float get_voltage(void) const;

    // return battery temperature in Kelvin:
    float get_temperature(void) const { return temperature.kelvin; }

private:
    float capacity_Ah;
    float resistance;
    float max_voltage;
    float voltage_set;
    float remaining_Ah;
    uint64_t last_us;

    struct {
        float kelvin = 273;
        uint64_t last_update_micros;
    } temperature;

    // 10Hz filter for battery voltage
    LowPassFilterFloat voltage_filter{10};

    float get_resting_voltage(float charge_pct) const;
    void set_initial_SoC(float voltage);
};
}
