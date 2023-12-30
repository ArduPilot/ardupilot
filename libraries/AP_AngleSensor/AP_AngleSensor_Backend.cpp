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

#include "AP_AngleSensor_Backend.h"

#if AP_ANGLESENSOR_ENABLED

#include "AP_AngleSensor.h"

// base class constructor.
AP_AngleSensor_Backend::AP_AngleSensor_Backend(AP_AngleSensor &frontend, uint8_t instance, AP_AngleSensor::AngleSensor_State &state) :
        _frontend(frontend),
        _state(state) 
{
    state.instance = instance;
}

// copy state to front end helper function
void AP_AngleSensor_Backend::copy_state_to_frontend(float angle_radians, uint8_t quality, uint32_t last_reading_ms)
{
    _state.angle_radians = angle_radians;
    _state.quality = quality;
    _state.last_reading_ms = last_reading_ms;
}

#endif  // AP_ANGLESENSOR_ENABLED