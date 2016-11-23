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
#include <AP_Navigation/AP_Navigation.h>

/// @class  AP_Landing
/// @brief  Class managing ArduPlane landing methods
class AP_Landing
{
public:

    FUNCTOR_TYPEDEF(set_target_altitude_proportion_fn_t, void, const Location&, float);
    set_target_altitude_proportion_fn_t set_target_altitude_proportion_fn;

    FUNCTOR_TYPEDEF(constrain_target_altitude_location_fn_t, void, const Location&, const Location&);
    constrain_target_altitude_location_fn_t constrain_target_altitude_location_fn;

    FUNCTOR_TYPEDEF(adjusted_altitude_cm_fn_t, int32_t);
    adjusted_altitude_cm_fn_t adjusted_altitude_cm_fn;

    FUNCTOR_TYPEDEF(adjusted_relative_altitude_cm_fn_t, int32_t);
    adjusted_relative_altitude_cm_fn_t adjusted_relative_altitude_cm_fn;

    FUNCTOR_TYPEDEF(disarm_if_autoland_complete_fn_t, void);
    disarm_if_autoland_complete_fn_t disarm_if_autoland_complete_fn;

    FUNCTOR_TYPEDEF(update_flight_stage_fn_t, void);
    update_flight_stage_fn_t update_flight_stage_fn;

    // constructor
    AP_Landing(AP_Mission &_mission, AP_AHRS &_ahrs, AP_SpdHgtControl *_SpdHgt_Controller, AP_Navigation *_nav_controller, AP_Vehicle::FixedWing &_aparm,
            set_target_altitude_proportion_fn_t _set_target_altitude_proportion_fn,
            constrain_target_altitude_location_fn_t _constrain_target_altitude_location_fn,
            adjusted_altitude_cm_fn_t _adjusted_altitude_cm_fn,
            adjusted_relative_altitude_cm_fn_t _adjusted_relative_altitude_cm_fn,
            disarm_if_autoland_complete_fn_t _disarm_if_autoland_complete_fn,
            update_flight_stage_fn_t _update_flight_stage_fn):
            mission(_mission)
            ,ahrs(_ahrs)
            ,SpdHgt_Controller(_SpdHgt_Controller)
            ,nav_controller(_nav_controller)
            ,aparm(_aparm)
            ,set_target_altitude_proportion_fn(_set_target_altitude_proportion_fn)
            ,constrain_target_altitude_location_fn(_constrain_target_altitude_location_fn)
            ,adjusted_altitude_cm_fn(_adjusted_altitude_cm_fn)
            ,adjusted_relative_altitude_cm_fn(_adjusted_relative_altitude_cm_fn)
            ,disarm_if_autoland_complete_fn(_disarm_if_autoland_complete_fn)
            ,update_flight_stage_fn(_update_flight_stage_fn) {
        AP_Param::setup_object_defaults(this, var_info);
    }

    bool restart_landing_sequence();
    bool jump_to_landing_sequence(void);
    bool verify_land(AP_SpdHgtControl::FlightStage flight_stage, const Location &prev_WP_loc, Location &next_WP_loc,const  Location &current_loc,
            int32_t auto_state_takeoff_altitude_rel_cm, float height, float sink_rate, float wp_proportion, uint32_t last_flying_ms, bool is_armed, bool is_flying, bool rangefinder_state_in_range, bool &throttle_suppressed);

    void adjust_landing_slope_for_rangefinder_bump(AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, float wp_distance, int32_t &target_altitude_offset_cm);

    void setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm);
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

    // denotes if a go-around has been commanded for landing
    bool commanded_go_around;


private:

    bool has_aborted_due_to_slope_recalc;

    AP_Mission &mission;
    AP_AHRS &ahrs;
    AP_SpdHgtControl *SpdHgt_Controller;
    AP_Navigation *nav_controller;

    AP_Vehicle::FixedWing &aparm;
};
