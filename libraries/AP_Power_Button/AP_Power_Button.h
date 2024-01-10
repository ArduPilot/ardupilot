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

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// allow buttons for up to 4 pins
#define MS_PWR_BUTTON_DOWN 1500

class AP_Power_Button {
public:
    // constructor
    AP_Power_Button(void);
    ~AP_Power_Button() {};
    // update button state and send messages, called periodically by main loop
    void update(void);

private:

    bool pwr_button_down();
    uint64_t last_power_down_time_ms;

};
