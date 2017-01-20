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

#pragma once

#include <AP_Gripper/AP_Gripper_Backend.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_Gripper_Servo : public AP_Gripper_Backend {
public:

    AP_Gripper_Servo(struct AP_Gripper::Backend_Config &_config) :
        AP_Gripper_Backend(_config) { }

    // grab - move the servo to the grab position
    void grab() override;

    // release - move the servo output to the release position
    void release() override;

    // grabbed - returns true if gripper in grabbed state
    bool grabbed() const override;

    // released - returns true if gripper in released state
    bool released() const override;

    // valid - returns true if the backend should be working
    bool valid() const;

protected:

    // type-specific intiailisations:
    void init_gripper() override;

    // type-specific periodic updates:
    void update_gripper() override;

private:

    uint32_t action_timestamp; // ms; time grab or release happened
    const uint16_t action_time = 3000; // ms; time to grab or release

    bool has_state_pwm(const uint16_t pwm) const;
};
