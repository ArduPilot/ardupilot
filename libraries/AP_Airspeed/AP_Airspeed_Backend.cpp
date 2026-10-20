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
  backend driver class for airspeed
 */

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

AP_Airspeed_Backend::AP_Airspeed_Backend(AP_Airspeed &_frontend, uint8_t _instance) :
    frontend(_frontend),
    instance(_instance)
{
}

int8_t AP_Airspeed_Backend::get_pin(void) const
{
#ifndef HAL_BUILD_AP_PERIPH
    return frontend.param[instance].pin;
#else
    return 0;
#endif
}

float AP_Airspeed_Backend::get_psi_range(void) const
{
    return frontend.param[instance].psi_range;
}

uint8_t AP_Airspeed_Backend::get_bus(void) const
{
    return frontend.param[instance].bus;
}

bool AP_Airspeed_Backend::bus_is_configured(void) const
{
    return frontend.param[instance].bus.configured();
}

void AP_Airspeed_Backend::set_bus_id(uint32_t id)
{
    frontend.param[instance].bus_id.set_and_save(id);
}

// some sensors use zero offsets, so need no calibration
void AP_Airspeed_Backend::set_use_zero_offset(void)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (frontend.state[instance].cal.start_ms != 0) {
        // cancel a calibration started before we knew it was not needed
        frontend.state[instance].cal.start_ms = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Airspeed %u calibration not required", instance + 1);
    }
    frontend.param[instance].offset.set(0.0);
#endif
    frontend.state[instance].cal.state = AP_Airspeed::CalibrationState::NOT_REQUIRED_ZERO_OFFSET;
}

#endif  // AP_AIRSPEED_ENABLED
