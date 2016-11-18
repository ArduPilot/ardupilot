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
 *   AP_Landing.cpp - Landing logic handler for ArduPlane
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AP_Landing::var_info[] = {

    AP_GROUPEND
};



/*
     Restart a landing by first checking for a DO_LAND_START and
     jump there. Otherwise decrement waypoint so we would re-start
     from the top with same glide slope. Return true if successful.
 */
bool AP_Landing::restart_landing_sequence()
{
    if (mission.get_current_nav_cmd().id != MAV_CMD_NAV_LAND) {
        return false;
    }

    uint16_t do_land_start_index = mission.get_landing_sequence_start();
    uint16_t prev_cmd_with_wp_index = mission.get_prev_nav_cmd_with_wp_index();
    bool success = false;
    uint16_t current_index = mission.get_current_nav_index();
    AP_Mission::Mission_Command cmd;

    if (mission.read_cmd_from_storage(current_index+1,cmd) &&
            cmd.id == MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT &&
            (cmd.p1 == 0 || cmd.p1 == 1) &&
            mission.set_current_cmd(current_index+1))
    {
        // if the next immediate command is MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT to climb, do it
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_NOTICE, "Restarted landing sequence. Climbing to %dm", cmd.content.location.alt/100);
        success =  true;
    }
    else if (do_land_start_index != 0 &&
            mission.set_current_cmd(do_land_start_index))
    {
        // look for a DO_LAND_START and use that index
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_NOTICE, "Restarted landing via DO_LAND_START: %d",do_land_start_index);
        success =  true;
    }
    else if (prev_cmd_with_wp_index != AP_MISSION_CMD_INDEX_NONE &&
               mission.set_current_cmd(prev_cmd_with_wp_index))
    {
        // if a suitable navigation waypoint was just executed, one that contains lat/lng/alt, then
        // repeat that cmd to restart the landing from the top of approach to repeat intended glide slope
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_NOTICE, "Restarted landing sequence at waypoint %d", prev_cmd_with_wp_index);
        success =  true;
    } else {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Unable to restart landing sequence");
        success =  false;
    }

    return success;
}


/*
   find the nearest landing sequence starting point (DO_LAND_START) and
   switch to that mission item.  Returns false if no DO_LAND_START
   available.
 */
bool AP_Landing::jump_to_landing_sequence(void)
{
    uint16_t land_idx = mission.get_landing_sequence_start();
    if (land_idx != 0) {
        if (mission.set_current_cmd(land_idx)) {

            //if the mission has ended it has to be restarted
            if (mission.state() == AP_Mission::MISSION_STOPPED) {
                mission.resume();
            }

            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Landing sequence start");
            return true;
        }
    }

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_WARNING, "Unable to start landing sequence");
    return false;
}

/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */
Location AP_Landing::setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc)
{
    float total_distance = get_distance(prev_WP_loc, next_WP_loc);

    // If someone mistakenly puts all 0's in their LAND command then total_distance
    // will be calculated as 0 and cause a divide by 0 error below.  Lets avoid that.
    if (total_distance < 1) {
        total_distance = 1;
    }

    // height we need to sink for this WP
    float sink_height = (prev_WP_loc.alt - next_WP_loc.alt)*0.01f;

    // current ground speed
    float groundspeed = ahrs.groundspeed();
    if (groundspeed < 0.5f) {
        groundspeed = 0.5f;
    }

    // calculate time to lose the needed altitude
    float sink_time = total_distance / groundspeed;
    if (sink_time < 0.5f) {
        sink_time = 0.5f;
    }

    // find the sink rate needed for the target location
    float sink_rate = sink_height / sink_time;

    // the height we aim for is the one to give us the right flare point
    float aim_height = aparm.land_flare_sec * sink_rate;
    if (aim_height <= 0) {
        aim_height = aparm.land_flare_alt;
    }

    // don't allow the aim height to be too far above LAND_FLARE_ALT
    if (aparm.land_flare_alt > 0 && aim_height > aparm.land_flare_alt*2) {
        aim_height = aparm.land_flare_alt*2;
    }

    // calculate slope to landing point
    bool is_first_calc = is_zero(slope);
    slope = (sink_height - aim_height) / total_distance;
    if (is_first_calc) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Landing glide slope %.1f degrees", (double)degrees(atanf(slope)));
    }


    // time before landing that we will flare
    float flare_time = aim_height / SpdHgt_Controller->get_land_sinkrate();

    // distance to flare is based on ground speed, adjusted as we
    // get closer. This takes into account the wind
    float flare_distance = groundspeed * flare_time;

    // don't allow the flare before half way along the final leg
    if (flare_distance > total_distance/2) {
        flare_distance = total_distance/2;
    }

    // project a point 500 meters past the landing point, passing
    // through the landing point
    const float land_projection = 500;
    int32_t land_bearing_cd = get_bearing_cd(prev_WP_loc, next_WP_loc);

    // now calculate our aim point, which is before the landing
    // point and above it
    Location loc = next_WP_loc;
    location_update(loc, land_bearing_cd*0.01f, -flare_distance);
    loc.alt += aim_height*100;

    // calculate point along that slope 500m ahead
    location_update(loc, land_bearing_cd*0.01f, land_projection);
    loc.alt -= slope * land_projection * 100;

    return loc;
}


