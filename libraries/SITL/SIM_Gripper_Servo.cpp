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
  simple Gripper (Servo) simulation class
*/

#include "SIM_Gripper_Servo.h"
#include <stdio.h>

using namespace SITL;

/*
  update gripper state
 */
void Gripper_Servo::update(const Aircraft::sitl_input &input)
{
    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;

    // update gripper position

    float position_demand = (input.servos[gripper_servo]-1000) * 0.001f;
    if (position_demand < 0) { // never updated
        position_demand = 0;
    }

    const float position_max_change = position_slew_rate/100.0f * dt;
    position = constrain_float(position_demand, position-position_max_change, position+position_max_change);

    const float jaw_gap = gap*(1.0f-position);
    if (should_report()) {
        ::fprintf(stderr, "position_demand=%f jaw_gap=%f load=%f\n", position_demand, jaw_gap, load_mass);
        last_report_us = now;
        reported_position = position;
    }

    if (jaw_gap < 5) {
        if (aircraft->on_ground()) {
            load_mass = 1.0f; // attach the load
        }
    } else if (jaw_gap > 10) {
        load_mass = 0.0f; // detach the load
    }

    last_update_us = now;
    return;
}

bool Gripper_Servo::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    if (reported_position != position) {
        return true;
    }

    return false;
}


float Gripper_Servo::payload_mass() const
{
    if (aircraft->hagl() < string_length) {
        return 0.0f;
    }
    return load_mass;
}
