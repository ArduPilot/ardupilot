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

#include "AP_Torqeedo.h"

#if HAL_TORQEEDO_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_HAL/Semaphores.h>

class AP_Torqeedo_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Torqeedo_Backend(AP_Torqeedo &_frontend, AP_Torqeedo::Torqeedo_State &_state, AP_Torqeedo_Params &_params);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Torqeedo_Backend(void) {}

    // initialize drivers
    virtual void init(void) {};

    // update the state structure
    virtual void update(void) = 0;

    // clear error
    virtual void clear_motor_error(void) {}

    // get latest battery status info.  returns true on success and populates arguments
    virtual bool get_batt_info(float &voltage, float &current_amps, float &temp_C, uint8_t &pct_remaining) const WARN_IF_UNUSED { return false; }

    virtual bool get_batt_capacity_Ah(uint16_t &amp_hours) const { return false; }

protected:

    // set status and update valid_count
    void set_status(AP_Torqeedo::Status status);

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    AP_Torqeedo::Type _backend_type;

    AP_Torqeedo &frontend;
    AP_Torqeedo::Torqeedo_State &state;   // reference to this instances state
    AP_Torqeedo_Params &params;            // parameters for this backend
};

#endif // HAL_PROXIMITY_ENABLED
