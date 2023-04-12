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

#include "AP_RangeFinder_Lua.h"
#include <AP_HAL/AP_HAL.h>

#if AP_SCRIPTING_ENABLED

// constructor
AP_RangeFinder_Lua::AP_RangeFinder_Lua(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    set_status(RangeFinder::Status::NoData);
}


// Set the distance based on a Lua Script
bool AP_RangeFinder_Lua::handle_script_msg(float dist_m)
{
    state.last_reading_ms = AP_HAL::millis();
    _distance_m = dist_m;
    return true;
}


// update the state of the sensor
void AP_RangeFinder_Lua::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_LUA_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        state.distance_m = 0.0f;
    } else {
        state.distance_m = _distance_m;
        update_status();
    }
}

#endif
