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

#include <AP_Winch/AP_Winch.h>

class AP_Winch_Backend {
public:
    AP_Winch_Backend(struct AP_Winch::Backend_Config &_config) :
        config(_config) { }

    // initialise the backend
    virtual void init(const AP_WheelEncoder* wheel_encoder) = 0;

    // update - should be called at at least 10hz
    virtual void update() = 0;

protected:

    struct AP_Winch::Backend_Config &config;
};
