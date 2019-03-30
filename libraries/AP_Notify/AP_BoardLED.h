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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

#include "NotifyDevice.h"

#define HIGH 1
#define LOW 0

class AP_BoardLED: public NotifyDevice
{
public:
    // initialise the LED driver
    bool init(void) override;

    // should be called at 50Hz
    void update(void) override;

private:
    // counter incremented at 50Hz
    uint8_t _counter;
};
