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

//
//  UAVCAN libcanard GPS driver
//
#include <AP_HAL/AP_HAL.h>

#if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_GPS_IN)

#include "AP_GPS_UAVCANARD.h"

#ifndef ARDUPILOT_GNSS_STATUS_STATUS_LOGGING
#define ARDUPILOT_GNSS_STATUS_STATUS_LOGGING 1
#endif
#ifndef ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE
#define ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE 2
#endif

extern const AP_HAL::HAL& hal;

bool AP_GPS_UAVCANARD::_new_data;
AP_GPS::GPS_State AP_GPS_UAVCANARD::interim_state;
HAL_Semaphore AP_GPS_UAVCANARD::sem;
bool AP_GPS_UAVCANARD::seen_message;
bool AP_GPS_UAVCANARD::seen_status;
bool AP_GPS_UAVCANARD::healthy;
uint32_t AP_GPS_UAVCANARD::status_flags;


// Member Methods
AP_GPS_UAVCANARD::AP_GPS_UAVCANARD(AP_GPS &_gps, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _state, nullptr)
{
}


// Consume new data and mark it received
bool AP_GPS_UAVCANARD::read(void)
{
    WITH_SEMAPHORE(sem);
    if (_new_data) {
        _new_data = false;

        // the encoding of accuracies in UAVCAN can result in infinite
        // values. These cause problems with blending. Use 1000m and 1000m/s instead
        interim_state.horizontal_accuracy = MIN(interim_state.horizontal_accuracy, 1000.0);
        interim_state.vertical_accuracy = MIN(interim_state.vertical_accuracy, 1000.0);
        interim_state.speed_accuracy = MIN(interim_state.speed_accuracy, 1000.0);

        state = interim_state;
        return true;
    }
    if (!seen_message) {
        // start with NO_GPS until we get first packet
        state.status = AP_GPS::GPS_Status::NO_GPS;
    }

    return false;
}

bool AP_GPS_UAVCANARD::is_healthy(void) const
{
    // if we don't have any health reports, assume it's healthy
    if (!seen_status) {
        return true;
    }
    return healthy;
}

bool AP_GPS_UAVCANARD::logging_healthy(void) const
{
    // if we don't have status, assume it's valid
    if (!seen_status) {
        return true;
    }

    return (status_flags & ARDUPILOT_GNSS_STATUS_STATUS_LOGGING) != 0;
}

bool AP_GPS_UAVCANARD::is_configured(void) const
{
    // if we don't have status assume it's configured
    if (!seen_status) {
        return true;
    }

    return (status_flags & ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE) != 0;
}

#endif // #if defined(HAL_BUILD_AP_PERIPH) && defined(HAL_PERIPH_ENABLE_GPS_IN)
