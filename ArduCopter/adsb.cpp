/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * adsb.cpp
 * Copyright (C) Tom Pittenger 2015
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Copter.h"

#if ADSB_ENABLED == ENABLED

/*
 *  this module deals with ADS-B handling for ArduPilot
 *  ADS-B is an RF based collision avoidance protocol to tell nearby aircraft your location
 *  https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
 *
 */

/*
  handle periodic adsb database maintenance and handle threats
 */
void Copter::adsb_update(void)
{
    adsb.update();
    adsb_handle_vehicle_threats();
}

/*
 * Handle ADS-B based threats which are platform dependent
 */
void Copter::adsb_handle_vehicle_threats(void)
{
    // handle clearing of threat
    if (adsb.get_is_evading_threat() && !adsb.get_possible_threat()) {
        adsb.set_is_evading_threat(false);
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_ADSB, ERROR_CODE_FAILSAFE_RESOLVED);
        gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat cleared");
        return;
    }

    // handle new threat
    if (!adsb.get_is_evading_threat() && adsb.get_possible_threat()) {
        adsb.set_is_evading_threat(true);
        Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_ADSB, ERROR_CODE_FAILSAFE_OCCURRED);
        gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat!");
        return;
    }
}

#endif // #ADSB_ENABLED
