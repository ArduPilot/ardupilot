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

#include <AP_Winch/AP_Winch_Backend.h>
#include <SRV_Channel/SRV_Channel.h>

class AP_Winch_Servo : public AP_Winch_Backend {
public:

    AP_Winch_Servo(struct AP_Winch::Backend_Config &_config) :
        AP_Winch_Backend(_config) { }

    // initialise the winch
    void init(const AP_WheelEncoder* wheel_encoder) override;

    // control the winch
    void update() override;

private:
    // external reference
    const AP_WheelEncoder* _wheel_encoder;

    uint32_t last_update_ms;    // last time update was called
    bool limit_high;            // output hit limit on last iteration
    bool limit_low;             // output hit lower limit on last iteration
};
