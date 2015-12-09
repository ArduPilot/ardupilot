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

#include "Plane.h"

/*
 *  this module deals with ADS-B handling for ArduPlane
 *  ADS-B is an RF based collision avoidance protocol to tell nearby aircraft your location
 *  https://en.wikipedia.org/wiki/Automatic_dependent_surveillance_%E2%80%93_broadcast
 *
 */

/*
  handle periodic adsb database maintenance and handle threats
 */
void Plane::adsb_update(void)
{
    adsb.update();

    if (!adsb.get_is_evading_threat()) {
        if (adsb.get_possible_threat()) {
            bool evasion_start_success = adsb_evasion_start();
            adsb.set_is_evading_threat(evasion_start_success);
        }
    } else if (adsb_state.is_evading) {
        // there has been no manual override, handle an auto-evasion
        if (!adsb.get_possible_threat()) {
                adsb.set_is_evading_threat(false);
                adsb_evasion_stop();
                adsb_state.is_evading = false;
        } else {
            adsb_evasion_ongoing();
        }
    }
}

/*
 * This fires once at the moment we realize there is a threat
 */
bool Plane::adsb_evasion_start(void)
{
    if (control_mode != AUTO) {
        // evasion is only supported while in AUTO mode
        return false;
    }
    if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF ||
        mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
        // evasion during a plane take-off or land can result in a crash. Ignore ADS-B traffic
        return false;
    }

    adsb_state.timestamp_ms =  millis();
    adsb_state.prev_wp = prev_WP_loc; // must be done BEFORE set_mode()

    switch(adsb.get_behavior()) {
    case AP_ADSB::ADSB_BEHAVIOR_NONE:
    default:
        gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat found, no action taken");
        break;

    case AP_ADSB::ADSB_BEHAVIOR_GUIDED:
        gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat found, switching to GUIDED mode");
        set_mode(GUIDED);
        adsb_state.is_evading = true; // must be done AFTER set_mode()
        break;

    case AP_ADSB::ADSB_BEHAVIOR_LOITER:
    case AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND:
        gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat found, performing LOITER");
        set_mode(LOITER);
        adsb_state.is_evading = true; // must be done after set_mode()
        break;
    } // switch behavior

    return true;
}

/*
 * This fires once at the moment we realize there is no longer a threat
 */
void Plane::adsb_evasion_stop(void)
{
    gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat gone, continuing mission");

    FlightMode prev_control_mode = control_mode;
    set_mode(AUTO);
    if (prev_control_mode == LOITER)
    {
        // if resuming from loiter, smoothly get back on track
        prev_WP_loc = adsb_state.prev_wp;
        auto_state.no_crosstrack = false;
    }
}

/*
 * This fires once per second while evading a threat
 */
void Plane::adsb_evasion_ongoing(void)
{
    uint32_t now = millis();

    if (control_mode == LOITER &&
        adsb.get_behavior() == AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND &&
        now - adsb_state.timestamp_ms >= 1000)
    {
        // slowly reduce altitude 100 cm/s while loitering. Drive into the ground if threat persists
         adsb_state.timestamp_ms = now;
         next_WP_loc.alt -= 100;
    }
}
