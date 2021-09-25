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
//  UAVCAN GPS driver
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include "GPS_Backend.h"

//#include <../modules/libcanard/canard.h>
// --------------------
// THIS WORKS but it can't find "canard.h" inside it.
//#include <modules/libcanard/dsdlc_generated/ardupilot/gnss/Heading.h>
// --------------------

class AP_GPS_UAVCANARD : public AP_GPS_Backend {
public:
    AP_GPS_UAVCANARD(AP_GPS &_gps, AP_GPS::GPS_State &_state);

    bool read() override;

    bool is_healthy(void) const override;

    bool logging_healthy(void) const override;

    bool is_configured(void) const override;

    const char *name() const override { return "UAVCANard"; }

    static void set_GPS_State(const AP_GPS::GPS_State new_state) {
        WITH_SEMAPHORE(sem);
        interim_state = new_state;
        seen_message = true;
        _new_data = true;
    }

    static void set_status(const uint32_t new_status_flags, const bool new_healthy) {
        WITH_SEMAPHORE(sem);
        seen_status = true;
        healthy = new_healthy;
        status_flags = new_status_flags;
    }

private:
    static bool _new_data;
    static AP_GPS::GPS_State interim_state;
    static HAL_Semaphore sem;
    static bool seen_message;
    static bool seen_status;
    static bool healthy;
    static uint32_t status_flags;
};
