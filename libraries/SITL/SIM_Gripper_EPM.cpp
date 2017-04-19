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
  simple Gripper (OpenGrab EPM) simulation class
*/

#include "SIM_Gripper_EPM.h"
#include <stdio.h>
#include <AP_Common/AP_Common.h>

using namespace SITL;

/*
  update gripper state
 */
void Gripper_EPM::update_servobased(const Aircraft::sitl_input &input)
{
    if (! servo_based) {
        return;
    }
    demand = (input.servos[gripper_servo]-1000) * 0.001f; // 0.0 - 1.0
    if (demand < 0) { // never updated
        demand = 0;
    }
}


void Gripper_EPM::update_from_demand(const Aircraft::sitl_input &input)
{
    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;

    // decay the field
    field_strength = field_strength * (100-field_decay_rate * dt)/100.0f;

    // note that "demand" here is just an on/off switch; we only care
    // about which range it falls into
    if (demand > 0.6) {
        // we are instructed to grip harder
        field_strength = field_strength + (100.0f-field_strength) * field_strength_slew_rate/100.0f * dt;
    } else if (demand < 0.4) {
        // we are instructed to loosen grip
        field_strength = field_strength * (100-field_degauss_rate * dt)/100.0f;
    } else {
        // neutral; no demanded change
    }

    if (should_report()) {
        ::fprintf(stderr, "demand=%f\n", demand);
        printf("Field strength: %f%%\n", field_strength);
        printf("Field strength: %f Tesla\n", tesla());
        last_report_us = now;
        reported_field_strength = field_strength;
    }

    last_update_us = now;
    return;
}

void Gripper_EPM::update(const Aircraft::sitl_input &input)
{
    update_servobased(input);

    update_from_demand(input);
}


bool Gripper_EPM::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    if (fabs(reported_field_strength - field_strength) > 10.0f) {
        return true;
    }

    return false;
}

float Gripper_EPM::tesla()
{
    // https://en.wikipedia.org/wiki/Orders_of_magnitude_(magnetic_field)
    // 200N lifting capacity ~= 2.5T
    const float percentage_to_tesla = 0.25;
    return percentage_to_tesla * field_strength/100.0f;
}
