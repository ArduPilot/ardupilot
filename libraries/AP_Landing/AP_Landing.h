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
#include <AP_Navigation/AP_Navigation.h>

/// @class  AP_Landing
/// @brief  Class managing ArduPlane landing methods
class AP_Landing
{
public:

    FUNCTOR_TYPEDEF(set_target_altitude_proportion_fn_t, void, const Location&, float);
    FUNCTOR_TYPEDEF(constrain_target_altitude_location_fn_t, void, const Location&, const Location&);
    FUNCTOR_TYPEDEF(adjusted_altitude_cm_fn_t, int32_t);
    FUNCTOR_TYPEDEF(adjusted_relative_altitude_cm_fn_t, int32_t);
    FUNCTOR_TYPEDEF(disarm_if_autoland_complete_fn_t, void);
    FUNCTOR_TYPEDEF(update_flight_stage_fn_t, void);

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
            ,update_flight_stage_fn(_update_flight_stage_fn)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // stages of flight
    enum LandingStage {
        STAGE_UNKNOWN       = 0,
        STAGE_APPROACH      = 4,
        STAGE_PREFLARE      = 5,
        STAGE_FINAL         = 6,
    };

    enum Landing_Type {
        TYPE_STANDARD_GLIDE_SLOPE = 0,
//      TODO: TYPE_DEEPSTALL,
//      TODO: TYPE_PARACHUTE,
//      TODO: TYPE_HELICAL,
    };

    bool verify_land(const bool is_aborting, const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const int32_t auto_state_takeoff_altitude_rel_cm, const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range, bool &throttle_suppressed);
    void adjust_landing_slope_for_rangefinder_bump(AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm);
    void setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm);
    void check_if_need_to_abort(const AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state);
    bool request_go_around(void);


    // helper functions
    void reset(void);
    bool restart_landing_sequence(void);
    float wind_alignment(const float heading_deg);
    float head_wind(void);
    int32_t get_target_airspeed_cm(void);

    // accessor functions for the params and states
    static const struct AP_Param::GroupInfo var_info[];
    int16_t get_pitch_cd(void) const { return pitch_cd; }
    float get_flare_sec(void) const { return flare_sec; }
    float get_pre_flare_airspeed(void) const { return pre_flare_airspeed; }
    int8_t get_disarm_delay(void) const { return disarm_delay; }
    int8_t get_then_servos_neutral(void) const { return then_servos_neutral; }
    int8_t get_abort_throttle_enable(void) const { return abort_throttle_enable; }
    int8_t get_flap_percent(void) const { return flap_percent; }
    int8_t get_throttle_slewrate(void) const { return throttle_slewrate; }
    bool is_commanded_go_around(void) const { return commanded_go_around; }
    bool is_complete(void) const { return complete; }
    void set_initial_slope() { initial_slope = slope; }
    void set_stage(LandingStage _stage) { stage = _stage; }
    LandingStage get_stage() const { return stage; }



    // Flag to indicate if we have landed.
    // Set land_complete if we are within 2 seconds distance or within 3 meters altitude of touchdown
    bool complete;

    // Flag to indicate if we have triggered pre-flare. This occurs when we have reached LAND_PF_ALT
    bool pre_flare;

    // are we in auto and flight mode is approach || pre-flare || final (flare)
    bool in_progress;

    // landing altitude offset (meters)
    float alt_offset;

private:

    // once landed, post some landing statistics to the GCS
    bool post_stats;

    bool has_aborted_due_to_slope_recalc;

    // denotes if a go-around has been commanded for landing
    bool commanded_go_around;

    // same as land_slope but sampled once before a rangefinder changes the slope. This should be the original mission planned slope
    float initial_slope;

    // calculated approach slope during auto-landing: ((prev_WP_loc.alt - next_WP_loc.alt)*0.01f - flare_sec * sink_rate) / get_distance(prev_WP_loc, next_WP_loc)
    float slope;

    // flight stages for landing
    LandingStage stage;

    AP_Mission &mission;
    AP_AHRS &ahrs;
    AP_SpdHgtControl *SpdHgt_Controller;
    AP_Navigation *nav_controller;

    AP_Vehicle::FixedWing &aparm;

    set_target_altitude_proportion_fn_t set_target_altitude_proportion_fn;
    constrain_target_altitude_location_fn_t constrain_target_altitude_location_fn;
    adjusted_altitude_cm_fn_t adjusted_altitude_cm_fn;
    adjusted_relative_altitude_cm_fn_t adjusted_relative_altitude_cm_fn;
    disarm_if_autoland_complete_fn_t disarm_if_autoland_complete_fn;
    update_flight_stage_fn_t update_flight_stage_fn;

    AP_Int16 pitch_cd;
    AP_Float flare_alt;
    AP_Float flare_sec;
    AP_Float pre_flare_airspeed;
    AP_Float pre_flare_alt;
    AP_Float pre_flare_sec;
    AP_Float slope_recalc_shallow_threshold;
    AP_Float slope_recalc_steep_threshold_to_abort;
    AP_Int8 disarm_delay;
    AP_Int8 then_servos_neutral;
    AP_Int8 abort_throttle_enable;
    AP_Int8 flap_percent;
    AP_Int8 throttle_slewrate;
    AP_Int8 type;

    // Land Type STANDARD GLIDE SLOPE
    bool type_slope_verify_land(const bool is_aborting, const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const int32_t auto_state_takeoff_altitude_rel_cm, const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range, bool &throttle_suppressed);

    void type_slope_adjust_landing_slope_for_rangefinder_bump(AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm);

    void type_slope_setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm);
    void type_slope_check_if_need_to_abort(const AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state);
    bool type_slope_request_go_around(void);

};
