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
  simple Gripper (EPM) simulation class
*/

#pragma once

#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class Gripper_EPM {
public:
    Gripper_EPM() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    float payload_mass() const { return 0.0f; } // kg

    // update field stength
    void update(const struct sitl_input &input);

    static const struct AP_Param::GroupInfo var_info[];
    bool is_enabled() const {return static_cast<bool>(gripper_emp_enable);}

private:

    AP_Int8  gripper_emp_enable;  // enable gripper sim
    AP_Int8  gripper_emp_servo_pin;

    const uint32_t report_interval = 100000; // microseconds
    uint64_t last_report_us;

    bool servo_based = true;

    float field_strength;   // percentage
    float reported_field_strength = -1; // unlikely

    // I've a feeling these are probably a higher order than this:
    const float field_strength_slew_rate = 400; // (percentage of delta between field strength and 100)/second
    const float field_decay_rate = 2; // percent of field strength/second
    const float field_degauss_rate = 300; // percent of field strength/second

    uint64_t last_update_us;

    bool should_report();

    void update_from_demand();
    void update_servobased(int16_t gripper_pwm);

    float tesla();

    float demand;
};

}
