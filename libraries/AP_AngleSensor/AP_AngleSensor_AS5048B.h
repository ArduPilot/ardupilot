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

#if AP_ANGLESENSOR_ENABLED

#include "AP_AngleSensor.h"
#include "AP_AngleSensor_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_AngleSensor_AS5048B : public AP_AngleSensor_Backend
{
public:
    // constructor
    AP_AngleSensor_AS5048B(AP_AngleSensor &frontend, uint8_t instance, AP_AngleSensor::AngleSensor_State &state);

    // update state
    void update(void) override;

    bool init(void);

private:

    // pointer to I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;

    void timer(void);

};

#endif  // AP_ANGLESENSOR_ENABLED
