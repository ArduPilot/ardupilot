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
#include "AP_WheelEncoder.h"
#include "WheelEncoder_Backend.h"

// base class constructor.
AP_WheelEncoder_Backend::AP_WheelEncoder_Backend(AP_WheelEncoder &frontend, uint8_t instance, AP_WheelEncoder::WheelEncoder_State &state) :
        _frontend(frontend),
        _state(state) 
{
    state.instance = instance;
}

// return pin.  returns -1 if pin is not defined for this instance
int8_t AP_WheelEncoder_Backend::get_pin_a() const
{
    if (_state.instance >= WHEELENCODER_MAX_INSTANCES) {
        return -1;
    }
    return _frontend._pina[_state.instance].get();
}

// return pin.  returns -1 if pin is not defined for this instance
int8_t AP_WheelEncoder_Backend::get_pin_b() const
{
    if (_state.instance >= WHEELENCODER_MAX_INSTANCES) {
        return -1;
    }
    return _frontend._pinb[_state.instance].get();
}

// copy state to front end helper function
void AP_WheelEncoder_Backend::copy_state_to_frontend(int32_t distance_count, uint32_t total_count, uint32_t error_count, uint32_t last_reading_ms)
{
    // record distance and time change for calculating rate before previous state is overwritten
    _state.dt_ms = last_reading_ms - _state.last_reading_ms;
    _state.dist_count_change = distance_count - _state.distance_count;

    // copy distance and error count so it is accessible to front end
    _state.distance_count = distance_count;
    _state.total_count = total_count;
    _state.error_count = error_count;
    _state.last_reading_ms = last_reading_ms;
}
