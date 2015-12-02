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
    adsb_handle_vehicle_threats();
}

/*
 * Handle ADS-B based threats which are platform dependent
 */
void Plane::adsb_handle_vehicle_threats(void)
{
    uint32_t now = millis();
    AP_ADSB::ADSB_BEHAVIOR behavior = adsb.get_behavior();

    switch (control_mode) {
    case AUTO:
        if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_TAKEOFF) {
            // for testing purposes ignore ADS-B traffic until we get into the air so we don't screw up the sim takeoff
            break;
        }
        switch(behavior) {
        case AP_ADSB::ADSB_BEHAVIOR_NONE:
        default:
            break;

        case AP_ADSB::ADSB_BEHAVIOR_LOITER:
        case AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND:
            if (adsb.get_another_vehicle_within_radius() && !adsb.get_is_evading_threat()) {
                adsb.set_is_evading_threat(true);
                gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat found, performing LOITER");
                adsb_state.prev_wp = prev_WP_loc;
                set_mode(LOITER);
                if (behavior == AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND) {
                    adsb_state.time_last_alt_change_ms = now;
                }
            }
        } // switch behavior
        break; // case auto

    case LOITER:
        switch(behavior) {
        case AP_ADSB::ADSB_BEHAVIOR_NONE:
            // TODO: recover from this
        default:
            break;

        case AP_ADSB::ADSB_BEHAVIOR_LOITER:
        case AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND:
            if (adsb.get_is_evading_threat()) {
                if (!adsb.get_another_vehicle_within_radius()) {
                    adsb.set_is_evading_threat(false);
                    gcs_send_text(MAV_SEVERITY_CRITICAL, "ADS-B threat gone, continuing mission");
                    set_mode(AUTO);
                    prev_WP_loc = adsb_state.prev_wp;
                    auto_state.no_crosstrack = false;
                } else if (behavior == AP_ADSB::ADSB_BEHAVIOR_LOITER_AND_DESCEND &&
                            now - adsb_state.time_last_alt_change_ms >= 1000) {
                   // slowly reduce altitude 1m/s while loitering. Drive into the ground if threat persists
                    adsb_state.time_last_alt_change_ms = now;
                    next_WP_loc.alt -= 100;
                } // get_another_vehicle_within_radius
            } // get_is_evading_threat
        } // switch behavior
        break; // case LOITER

    default:
        break;
    } // switch control_mode
}

