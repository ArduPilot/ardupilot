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
  simple sprayer simulator class
*/

#include "SIM_Sprayer.h"
#include <stdio.h>

using namespace SITL;

/*
  update sprayer state
 */
void Sprayer::update(const Aircraft::sitl_input &input)
{
    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;

    // update remaining payload
    if (capacity > 0) {
        const double delta = last_pump_output * pump_max_rate * dt;
        capacity -= delta;
        if (capacity < 0) {
            capacity = 0.0f;
        }
    }

    // update pump
    float pump_demand = (input.servos[pump_servo]-1000) * 0.001f;
    // ::fprintf(stderr, "pump_demand=%f\n", pump_demand);
    if (pump_demand < 0) { // never updated
        pump_demand = 0;
    }
    const float pump_max_change = pump_slew_rate/100.0f * dt;
    last_pump_output = constrain_float(pump_demand, last_pump_output-pump_max_change, last_pump_output+pump_max_change);
    last_pump_output = constrain_float(last_pump_output, 0, 1);

    // update spinner (if any)
    if (spinner_servo >= 0) {
        const float spinner_demand = (input.servos[spinner_servo]-1000) * 0.001f;
        const float spinner_max_change = spinner_slew_rate * 0.01f * dt;
        last_spinner_output = constrain_float(spinner_demand, last_spinner_output-spinner_max_change, last_spinner_output+spinner_max_change);
        last_spinner_output = constrain_float(last_spinner_output, 0, 1);
    }

    if (should_report()) {
        printf("Remaining: %f litres\n", capacity);
        printf("Pump: %f l/s\n", last_pump_output * pump_max_rate);
        if (spinner_servo >= 0) {
            printf("Spinner: %f rev/s\n", (last_spinner_output * spinner_max_rate)/360.0f);
        }
        last_report_us = now;
    }

    last_update_us = now;
    return;
}

bool Sprayer::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    if (!is_zero(last_pump_output) || !is_zero(last_spinner_output)) {
        zero_report_done = false;
        return true;
    }

    if (!zero_report_done) {
        zero_report_done = true;
        return true;
    }

    return false;
}

