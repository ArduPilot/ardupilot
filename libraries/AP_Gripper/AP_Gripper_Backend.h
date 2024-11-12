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

#include <AP_Gripper/AP_Gripper.h>

#if AP_GRIPPER_ENABLED

class AP_Gripper_Backend {
public:
    AP_Gripper_Backend(struct AP_Gripper::Backend_Config &_config) :
        config(_config) { }

    // initialise the gripper backend
    void init();

    // update - should be called at at least 10hz
    void update();

    // grab - move the servo to the grab position
    virtual void grab() = 0;

    // release - move the servo output to the release position
    virtual void release() = 0;

    // valid - returns true if the backend should be working
    virtual bool valid() const { return true; };

    // released - returns true if currently in released position
    virtual bool released() const = 0;

    // grabbed - returns true if currently in grabbed position
    virtual bool grabbed() const = 0;

    // type-specific initialisations:
    virtual void init_gripper() = 0;

    // type-specific periodic updates:
    virtual void update_gripper() { };

protected:

    uint32_t _last_grab_or_release; // ms; time last grab or release happened

    struct AP_Gripper::Backend_Config &config;
};

#endif  // AP_GRIPPER_ENABLED
