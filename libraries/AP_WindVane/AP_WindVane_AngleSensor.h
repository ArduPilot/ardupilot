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

#if AP_WINDVANE_ANGLESENSOR_ENABLED

#include "AP_WindVane_Backend.h"

#include <AP_AngleSensor/AP_AngleSensor.h>

class AP_WindVane_AngleSensor : public AP_WindVane_Backend
{
public:
    // constructor
    AP_WindVane_AngleSensor(AP_WindVane &frontend);

    // update state
    void update_direction() override;

private:
    uint8_t _encoder_instance = 0;
};

#endif  //AP_WINDVANE_ANGLESENSOR_ENABLED