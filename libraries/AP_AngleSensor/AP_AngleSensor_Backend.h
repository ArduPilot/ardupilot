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

#include "AP_AngleSensor.h"
#include <AP_HAL/Semaphores.h>

#if AP_ANGLESENSOR_ENABLED

class AP_AngleSensor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
    AP_AngleSensor_Backend(AP_AngleSensor &frontend, uint8_t instance, AP_AngleSensor::AngleSensor_State &state);

    // we declare a virtual destructor so that AngleSensor drivers can
    // override with a custom destructor if need be
    virtual ~AP_AngleSensor_Backend(void) {}

    // update the state structure. All backends must implement this.
    virtual void update() = 0;

protected:

    // copy state to front end helper function
    void copy_state_to_frontend(float angle_radians, uint8_t quality, uint32_t last_reading_ms);

    AP_AngleSensor &_frontend;
    AP_AngleSensor::AngleSensor_State &_state;

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;
};

#endif  // AP_ANGLESENSOR_ENABLED
