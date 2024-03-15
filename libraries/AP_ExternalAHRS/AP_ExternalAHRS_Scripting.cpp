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
/*
  support for scripting based external AHRS
 */

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SCRIPTING_ENABLED

#include "AP_ExternalAHRS_Scripting.h"

extern const AP_HAL::HAL &hal;

// accessors for AP_AHRS
bool AP_ExternalAHRS_Scripting::healthy(void) const
{
    WITH_SEMAPHORE(state.sem);
    return AP_HAL::millis() - last_update_ms < 100;
}

bool AP_ExternalAHRS_Scripting::initialised(void) const
{
    WITH_SEMAPHORE(state.sem);
    return last_update_ms != 0;
}

// handle input from scripting backend
bool AP_ExternalAHRS_Scripting::handle_scripting(const AP_ExternalAHRS::state_t &_state, nav_filter_status_flags_t &_filter_status)
{
    WITH_SEMAPHORE(state.sem);

    state.accel = _state.accel;
    state.gyro = _state.gyro;

    if (_state.have_quaternion) {
        state.quat = _state.quat;
    }

    if (_state.have_location) {
        state.location = _state.location;
    }

    if (_state.have_velocity) {
        state.velocity = _state.velocity;
    }

    if (_state.have_origin && !state.have_origin) {
        state.origin = _state.origin;
        state.have_origin = _state.have_origin;
    }

    state.have_quaternion = _state.have_quaternion;
    state.have_location = _state.have_location;
    state.have_velocity = _state.have_velocity;

    filter_status.flags = _filter_status;

    last_update_ms = AP_HAL::millis();
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS_Scripting::get_filter_status(nav_filter_status &status) const
{
    WITH_SEMAPHORE(state.sem);
    if (healthy()) {
        status = filter_status;
    } else {
        memset(&status, 0, sizeof(status));
    }
}

#endif  // AP_EXTERNAL_AHRS_SCRIPTING_ENABLED

