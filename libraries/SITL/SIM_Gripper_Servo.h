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

#pragma once
#include "stdint.h"
#include <AP_Param/AP_Param.h>
#include "SITL_Input.h"

namespace SITL {

class Gripper_Servo {
public:
    Gripper_Servo() {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // update Gripper state
    void update(const struct sitl_input &input);

    float payload_mass() const; // kg

    void set_alt(float alt) {altitude = alt;};
    static const struct AP_Param::GroupInfo var_info[];
    bool is_enabled() const {return static_cast<bool>(gripper_enable);}
    bool is_jaw_open() const {return jaw_open;}

private:
    static constexpr int16_t SIM_GRIPPER_GRAB_PWM_DEFAULT = 2000;
    static constexpr int16_t SIM_GRIPPER_RELEASE_PWM_DEFAULT = 1000;
    AP_Int8  gripper_enable;  // enable gripper sim
    AP_Int8  gripper_servo_pin;
    AP_Int16 grab_pwm;              // PWM value sent to Gripper to initiate grabbing the cargo
    AP_Int16 release_pwm;           // PWM value sent to Gripper to release the cargo
    AP_Int8 reverse;                // reverse closing direction
    const uint32_t report_interval = 1000000; // microseconds
    uint64_t last_report_us;
    bool jaw_open = false;
    const float gap = 30; // mm
    float altitude;
    float position; // percentage
    float position_slew_rate = 35; // percentage
    float reported_position = -1; // unlikely

    uint64_t last_update_us;

    bool should_report() const;

    // dangle load from a string:
    const float string_length = 2.0f; // metres
    float load_mass = 0.0f; // kilograms
};

}
