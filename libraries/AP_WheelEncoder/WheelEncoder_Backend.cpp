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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_WheelEncoder.h"
#include "WheelEncoder_Backend.h"

// base class constructor.
AP_WheelEncoder_Backend::AP_WheelEncoder_Backend(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
        _frontend(frontend),
        _state(state) 
{
}

// return pin.  returns -1 if pin is not defined for this instance
int8_t AP_WheelEncoder_Backend::get_pin_a() const
{
    if (_state.instance > 1) {
        return -1;
    }
    return _frontend._pina[_state.instance].get();
}

// return pin.  returns -1 if pin is not defined for this instance
int8_t AP_WheelEncoder_Backend::get_pin_b() const
{
    if (_state.instance > 1) {
        return -1;
    }
    return _frontend._pinb[_state.instance].get();
}
