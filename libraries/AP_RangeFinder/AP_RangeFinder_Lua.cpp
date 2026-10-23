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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LUA_ENABLED

#include "AP_RangeFinder_Lua.h"
#include <AP_HAL/AP_HAL.h>

// constructor
AP_RangeFinder_Lua::AP_RangeFinder_Lua(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
}


// Process range finder data from a lua driver. The state structure needs to be completely
// filled in by the lua script. The data passed to this method is copied to a pending_state
// structure. The update() method periodically copies data from pending_state to state. The get_state()
// method returns data from state.
bool AP_RangeFinder_Lua::handle_script_msg(const RangeFinder::RangeFinder_State &state_arg)
{
    WITH_SEMAPHORE(_sem);
    _state_pending = state_arg;
    return true;
}

// Process range finder data from a lua driver - legacy interface. This method takes
// a distance measurement and fills in the pending state structure. In this legacy mode
// the lua script only passes in a distance measurement and does not manage the rest
// of the fields in the state structure.
bool AP_RangeFinder_Lua::handle_script_msg(float dist_m) {

    const uint32_t now = AP_HAL::millis();

    WITH_SEMAPHORE(_sem);

    _state_pending.last_reading_ms = now;
    _state_pending.distance_m = dist_m;
    _state_pending.signal_quality_pct = RangeFinder::SIGNAL_QUALITY_UNKNOWN;
    _state_pending.voltage_mv = 0;
    update_status(_state_pending);

    return true;
}

// Update the state of the sensor
void AP_RangeFinder_Lua::update(void)
{
    WITH_SEMAPHORE(_sem);

    // Time out on incoming data
    if (_state_pending.status != RangeFinder::Status::NotConnected &&
            AP_HAL::millis() - _state_pending.last_reading_ms > AP_RANGEFINDER_LUA_TIMEOUT_MS) {
        set_status(_state_pending, RangeFinder::Status::NoData);
    }
    state = _state_pending;
}

#endif  // AP_RANGEFINDER_LUA_ENABLED
