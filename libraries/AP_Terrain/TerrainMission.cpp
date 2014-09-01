// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  handle checking mission points for terrain data
 */

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <GCS_MAVLink.h>
#include <GCS.h>
#include "AP_Terrain.h"

#if AP_TERRAIN_AVAILABLE

extern const AP_HAL::HAL& hal;

/*
  check that we have fetched all mission terrain data
 */
void AP_Terrain::update_mission_data(void)
{
    if (last_mission_change_ms != mission.last_change_time_ms() ||
        last_mission_spacing != grid_spacing) {
        // the mission has changed - start again
        next_mission_index = 1;
        next_mission_pos = 0;
        last_mission_change_ms = mission.last_change_time_ms();
        last_mission_spacing = grid_spacing;
    }
    if (next_mission_index == 0) {
        // nothing to do
        return;
    }

    uint16_t pending, loaded;
    get_statistics(pending, loaded);
    if (pending && ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        // wait till we have fully filled the current set of grids
        return;
    }

    // don't do more than 20 waypoints at a time, to prevent too much
    // CPU usage
    for (uint8_t i=0; i<20; i++) {
        // get next mission command
        AP_Mission::Mission_Command cmd;
        if (!mission.read_cmd_from_storage(next_mission_index, cmd)) {
            // nothing more to do
            next_mission_index = 0;
            return;
        }

        // we only want nav waypoint commands. That should be enough to
        // prefill the terrain data and makes many things much simpler
        while ((cmd.id != MAV_CMD_NAV_WAYPOINT &&
                cmd.id != MAV_CMD_NAV_SPLINE_WAYPOINT) ||
               (cmd.content.location.lat == 0 && cmd.content.location.lng == 0)) {
            next_mission_index++;
            if (!mission.read_cmd_from_storage(next_mission_index, cmd)) {
                // nothing more to do
                next_mission_index = 0;
                next_mission_pos = 0;
                return;
            }
        }

        // we will fetch 5 points around the waypoint. Four at 10 grid
        // spacings away at 45, 135, 225 and 315 degrees, and the
        // point itself
        if (next_mission_pos != 4) {
            location_update(cmd.content.location, 45+90*next_mission_pos, grid_spacing.get() * 10);
        }

        // we have a mission command to check
        float height;
        if (!height_amsl(cmd.content.location, height)) {
            // if we can't get data for a mission item then return and
            // check again next time
            return;
        }

        next_mission_pos++;
        if (next_mission_pos == 5) {
#if TERRAIN_DEBUG
            hal.console->printf("checked waypoint %u\n", (unsigned)next_mission_index);
#endif

            // move to next waypoint
            next_mission_index++;
            next_mission_pos = 0;
        }
    }
}

/*
  check that we have fetched all rally terrain data
 */
void AP_Terrain::update_rally_data(void)
{
    if (last_rally_change_ms != rally.last_change_time_ms() ||
        last_rally_spacing != grid_spacing) {
        // a rally point has changed - start again
        next_rally_index = 1;
        last_rally_change_ms = rally.last_change_time_ms();
        last_rally_spacing = grid_spacing;
    }
    if (next_rally_index == 0) {
        // nothing to do
        return;
    }

    uint16_t pending, loaded;
    get_statistics(pending, loaded);
    if (pending && ahrs.get_gps().status() >= AP_GPS::GPS_OK_FIX_3D) {
        // wait till we have fully filled the current set of grids
        return;
    }

    while (true) {
        // get next rally point
        struct RallyLocation rp;
        if (!rally.get_rally_point_with_index(next_rally_index, rp)) {
            // nothing more to do
            next_rally_index = 0;
            return;
        }

        Location loc;
        loc.lat = rp.lat;
        loc.lng = rp.lng;
        float height;
        if (!height_amsl(loc, height)) {
            // if we can't get data for a rally item then return and
            // check again next time
            return;
        }

#if TERRAIN_DEBUG
        hal.console->printf("checked rally point %u\n", (unsigned)next_rally_index);
#endif

        // move to next rally point
        next_rally_index++;
    }
}

#endif // AP_TERRAIN_AVAILABLE
