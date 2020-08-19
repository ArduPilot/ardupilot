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
#include "AP_HAL/AP_HAL.h"
#include <stdio.h>
#include <AP_Math/AP_Math.h>

using namespace SITL;

// table of user settable parameters
const AP_Param::GroupInfo Gripper_EPM::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Gripper servo Sim enable/disable
    // @Description: Allows you to enable (1) or disable (0) the gripper servo simulation
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 0, Gripper_EPM, gripper_emp_enable, 0),

    // @Param: PIN
    // @DisplayName: Gripper emp pin
    // @Description: The pin number that the gripper emp is connected to. (start at 1)
    // @Range: 0 15
    // @User: Advanced
    AP_GROUPINFO("PIN", 1, Gripper_EPM, gripper_emp_servo_pin, -1),

    AP_GROUPEND
};

/*
  update gripper state
 */
void Gripper_EPM::update_servobased(int16_t gripper_pwm)
{
    if (!servo_based) {
        return;
    }
    if (gripper_pwm >= 0) {
        demand = (gripper_pwm - 1000) * 0.001f; // 0.0 - 1.0
        if (is_negative(demand)) { // never updated
            demand = 0.0f;
        }
    }
}


void Gripper_EPM::update_from_demand()
{
    const uint64_t now = AP_HAL::micros64();
    const float dt = (now - last_update_us) * 1.0e-6f;

    // decay the field
    field_strength = field_strength * (100.0f - field_decay_rate * dt) / 100.0f;

    // note that "demand" here is just an on/off switch; we only care
    // about which range it falls into
    if (demand > 0.6f) {
        // we are instructed to grip harder
        field_strength = field_strength + (100.0f - field_strength) * field_strength_slew_rate / 100.0f * dt;
    } else if (demand < 0.4f) {
        // we are instructed to loosen grip
        field_strength = field_strength * (100.0f - field_degauss_rate * dt) / 100.0f;
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
}

void Gripper_EPM::update(const struct sitl_input &input)
{
    const int16_t gripper_pwm = gripper_emp_servo_pin >= 1 ? input.servos[gripper_emp_servo_pin-1] : -1;

    update_servobased(gripper_pwm);

    update_from_demand();
}


bool Gripper_EPM::should_report()
{
    if (AP_HAL::micros64() - last_report_us < report_interval) {
        return false;
    }

    if (fabsf(reported_field_strength - field_strength) > 10.0) {
        return true;
    }

    return false;
}

float Gripper_EPM::tesla()
{
    // https://en.wikipedia.org/wiki/Orders_of_magnitude_(magnetic_field)
    // 200N lifting capacity ~= 2.5T
    const float percentage_to_tesla = 0.25f;
    return static_cast<float>(percentage_to_tesla * field_strength / 100.0f);
}
