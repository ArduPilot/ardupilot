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

#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/AP_Common.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Vehicle/AP_Vehicle.h>

/// @class  AP_Landing
/// @brief  Class managing ArduPlane landing methods
class AP_Landing
{
public:

    // constructor
    AP_Landing(AP_Mission &_mission, AP_AHRS &_ahrs, AP_SpdHgtControl *_SpdHgt_Controller, const AP_Vehicle::FixedWing &_aparm) :
            mission(_mission),
            ahrs(_ahrs),
            SpdHgt_Controller(_SpdHgt_Controller),
            aparm(_aparm) {
        AP_Param::setup_object_defaults(this, var_info);
    }

    bool restart_landing_sequence();
    bool jump_to_landing_sequence(void);

    Location setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc);
    void check_if_need_to_abort(const AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state);

    static const struct AP_Param::GroupInfo var_info[];

    // Flag to indicate if we have landed.
    // Set land_complete if we are within 2 seconds distance or within 3 meters altitude of touchdown
    bool complete;

    // Flag to indicate if we have triggered pre-flare. This occurs when we have reached LAND_PF_ALT
    bool pre_flare;

    // are we in auto and flight mode is approach || pre-flare || final (flare)
    bool in_progress;

    // calculated approach slope during auto-landing: ((prev_WP_loc.alt - next_WP_loc.alt)*0.01f - aparm.land_flare_sec * sink_rate) / get_distance(prev_WP_loc, next_WP_loc)
    float slope;

    // same as land_slope but sampled once before a rangefinder changes the slope. This should be the original mission planned slope
    float initial_slope;

    // landing altitude offset (meters)
    float alt_offset;

    // once landed, post some landing statistics to the GCS
    bool post_stats;

    // have we checked for an auto-land?
    bool checked_for_autoland;

    // denotes if a go-around has been commanded for landing
    bool commanded_go_around;


private:

    bool has_aborted_due_to_slope_recalc;

    AP_Mission &mission;
    AP_AHRS &ahrs;
    AP_SpdHgtControl *SpdHgt_Controller;

    const AP_Vehicle::FixedWing &aparm;

};
