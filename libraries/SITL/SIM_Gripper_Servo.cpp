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
#include "AP_HAL/AP_HAL.h"
#include "AP_Math/AP_Math.h"
#include <stdio.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Gripper_Servo::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Gripper servo Sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the gripper servo simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, Gripper_Servo, gripper_enable, 0),

    // @Param: PIN
    // @DisplayName: Gripper servo pin
    // @Description: The pin number that the gripper servo is connected to. (start at 1)
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("PIN", 1, Gripper_Servo, gripper_servo_pin, -1),

    // @Param: GRAB
    // @DisplayName: Gripper Grab PWM
    // @Description: PWM value in microseconds sent to Gripper to initiate grabbing the cargo
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("GRAB", 2, Gripper_Servo, grab_pwm, SIM_GRIPPER_GRAB_PWM_DEFAULT),

    // @Param: RELEASE
    // @DisplayName: Gripper Release PWM
    // @Description: PWM value in microseconds sent to Gripper to release the cargo
    // @User: Advanced
    // @Range: 1000 2000
    // @Units: PWM
    AP_GROUPINFO("RELEASE", 3, Gripper_Servo, release_pwm, SIM_GRIPPER_RELEASE_PWM_DEFAULT),

    // @Param: REVERSE
    // @DisplayName: Gripper close direction
    // @Description: Reverse the closing direction.
    // @User: Advanced
    // @Values: 0:Normal,1:Reverse
    AP_GROUPINFO("REVERSE", 4, Gripper_Servo, reverse, 0),

    AP_GROUPEND
};

/*
  update gripper state
 */
void Gripper_Servo::update(const struct sitl_input &input)
{
    const int16_t gripper_pwm = gripper_servo_pin >= 1 ? input.servos[gripper_servo_pin-1] : -1;

    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;

    // update gripper position
    if (gripper_pwm < 0) {
        last_update_us = now;
        return;
    }
    const int16_t diff_pwm = abs(grab_pwm - release_pwm);
    float position_demand = (gripper_pwm - diff_pwm) * 0.001f;
    if (gripper_pwm < MIN(grab_pwm, release_pwm) || position_demand > 1.0f) { // never updated
        position_demand = position;
    }

    const float position_max_change = position_slew_rate / 100.0f * dt;
    position = constrain_float(position_demand, position - position_max_change, position + position_max_change);
    float jaw_gap;
    if ((release_pwm < grab_pwm && reverse) || (release_pwm > grab_pwm && !reverse)) {
        jaw_gap = gap * position;
    } else {
        jaw_gap = gap * (1.0f - position);
    }
    if (should_report()) {
        ::fprintf(stderr, "position_demand=%f jaw_gap=%f load=%f\n", position_demand, jaw_gap, load_mass);
        last_report_us = now;
        reported_position = position;
    }

    if (jaw_gap < 5) {
        if (altitude <= 0.0f) {
            load_mass = 1.0f; // attach the load
            jaw_open = false;
        }
    } else if (jaw_gap > 10) {
        load_mass = 0.0f; // detach the load
        jaw_open = true;
    }

    last_update_us = now;
}

bool Gripper_Servo::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    if (!is_equal(reported_position, position)) {
        return true;
    }

    return false;
}


float Gripper_Servo::payload_mass() const
{
    if (altitude < string_length) {
        return 0.0f;
    }
    return load_mass;
}
