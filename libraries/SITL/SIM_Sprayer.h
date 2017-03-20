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
  simple sprayer simulation class
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

class Sprayer {
public:
    const uint8_t pump_servo;
    const int8_t spinner_servo;

    Sprayer(const uint8_t _pump_servo, int8_t _spinner_servo) :
        pump_servo(_pump_servo),
        spinner_servo(_spinner_servo)
    {}

    // update sprayer state
    void update(const struct Aircraft::sitl_input &input);

    float payload_mass() const { return capacity; }; // kg; water, so kg=l

private:

    const uint32_t report_interval = 1000000; // microseconds
    uint64_t last_report_us;

    const float pump_max_rate = 0.01; // litres/second
    const float pump_slew_rate = 20; // percent/scond
    float last_pump_output; // percentage

    const float spinner_max_rate = 3600; // degrees/second
    const float spinner_slew_rate = 20; // percent/second
    float last_spinner_output; // percentage

    double capacity = 0.25; // litres

    uint64_t start_time_us;
    uint64_t last_update_us;

    bool should_report();
    bool zero_report_done = false;
};

}
