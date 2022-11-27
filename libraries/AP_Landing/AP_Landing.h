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
#include <AP_TECS/AP_TECS.h>
#include <AP_Navigation/AP_Navigation.h>
#include "AP_Landing_Deepstall.h"
#include <AP_Common/Location.h>

/// @class  AP_Landing
/// @brief  Class managing ArduPlane landing methods
class AP_Landing {
    friend class AP_Landing_Deepstall;

public:
    FUNCTOR_TYPEDEF(set_target_altitude_proportion_fn_t, void, const Location&, float);
    FUNCTOR_TYPEDEF(constrain_target_altitude_location_fn_t, void, const Location&, const Location&);
    FUNCTOR_TYPEDEF(adjusted_altitude_cm_fn_t, int32_t);
    FUNCTOR_TYPEDEF(adjusted_relative_altitude_cm_fn_t, int32_t);
    FUNCTOR_TYPEDEF(disarm_if_autoland_complete_fn_t, void);
    FUNCTOR_TYPEDEF(update_flight_stage_fn_t, void);

    AP_Landing(AP_Mission &_mission, AP_AHRS &_ahrs, AP_TECS *_tecs_Controller, AP_Navigation *_nav_controller, AP_FixedWing &_aparm,
               set_target_altitude_proportion_fn_t _set_target_altitude_proportion_fn,
               constrain_target_altitude_location_fn_t _constrain_target_altitude_location_fn,
               adjusted_altitude_cm_fn_t _adjusted_altitude_cm_fn,
               adjusted_relative_altitude_cm_fn_t _adjusted_relative_altitude_cm_fn,
               disarm_if_autoland_complete_fn_t _disarm_if_autoland_complete_fn,
               update_flight_stage_fn_t _update_flight_stage_fn);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Landing);


    // NOTE: make sure to update is_type_valid()
    enum Landing_Type {
        TYPE_STANDARD_GLIDE_SLOPE = 0,
#if HAL_LANDING_DEEPSTALL_ENABLED
        TYPE_DEEPSTALL = 1,
#endif
//      TODO: TYPE_PARACHUTE,
//      TODO: TYPE_HELICAL,
    };

    // we support upto 32 boolean bits for users wanting to change landing behaviour.
    enum OptionsMask {
        ON_LANDING_FLARE_USE_THR_MIN                   = (1<<0),   // If set then set trottle to thr_min instead of zero on final flare
    };

    void do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude);
    bool verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const int32_t auto_state_takeoff_altitude_rel_cm, bool &throttle_suppressed);
    bool verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range);
    void adjust_landing_slope_for_rangefinder_bump(AP_FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm);
    void setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm);
    bool override_servos(void);
    void check_if_need_to_abort(const AP_FixedWing::Rangefinder_State &rangefinder_state);
    bool request_go_around(void);
    bool is_flaring(void) const;
    bool is_on_approach(void) const;
    bool is_ground_steering_allowed(void) const;
    bool is_throttle_suppressed(void) const;
    bool is_flying_forward(void) const;
    bool use_thr_min_during_flare(void) const; //defaults to false, but _options bit zero enables it.
    void handle_flight_stage_change(const bool _in_landing_stage);
    int32_t constrain_roll(const int32_t desired_roll_cd, const int32_t level_roll_limit_cd);
    bool get_target_altitude_location(Location &location);
    bool send_landing_message(mavlink_channel_t chan);

    // terminate the flight with an immediate landing, returns false if unable to be used for termination
    bool terminate(void);

    // helper functions
    bool restart_landing_sequence(void);
    float wind_alignment(const float heading_deg);
    float head_wind(void);
    int32_t get_target_airspeed_cm(void);

    // accessor functions for the params and states
    static const struct AP_Param::GroupInfo var_info[];
    
    int16_t get_pitch_cd(void) const { return pitch_cd; }
    float get_flare_sec(void) const { return flare_sec; }
    int8_t get_disarm_delay(void) const { return disarm_delay; }
    int8_t get_then_servos_neutral(void) const { return then_servos_neutral; }
    int8_t get_abort_throttle_enable(void) const { return abort_throttle_enable; }
    int8_t get_flap_percent(void) const { return flap_percent; }
    int8_t get_throttle_slewrate(void) const { return throttle_slewrate; }
    bool is_commanded_go_around(void) const { return flags.commanded_go_around; }
    bool is_complete(void) const;
    void set_initial_slope(void) { initial_slope = slope; }
    bool is_expecting_impact(void) const;
    void Log(void) const;
    const AP_PIDInfo * get_pid_info(void) const;

    // landing altitude offset (meters)
    float alt_offset;

private:
    struct {
        // denotes if a go-around has been commanded for landing
        bool commanded_go_around:1;

        // are we in auto and flight_stage is LAND
        bool in_progress:1;
    } flags;

    AP_Int16 _options;    // user-configurable bitmask options, via a parameter, for landing

    // same as land_slope but sampled once before a rangefinder changes the slope. This should be the original mission planned slope
    float initial_slope;

    // calculated approach slope during auto-landing: ((prev_WP_loc.alt - next_WP_loc.alt)*0.01f - flare_sec * sink_rate) / prev_WP_loc.get_distance(next_WP_loc)
    float slope;

    float height_flare_log;

    AP_Mission &mission;
    AP_AHRS &ahrs;
    AP_TECS *tecs_Controller;
    AP_Navigation *nav_controller;
    
    AP_FixedWing &aparm;

    set_target_altitude_proportion_fn_t set_target_altitude_proportion_fn;
    constrain_target_altitude_location_fn_t constrain_target_altitude_location_fn;
    adjusted_altitude_cm_fn_t adjusted_altitude_cm_fn;
    adjusted_relative_altitude_cm_fn_t adjusted_relative_altitude_cm_fn;
    disarm_if_autoland_complete_fn_t disarm_if_autoland_complete_fn;
    update_flight_stage_fn_t update_flight_stage_fn;

#if HAL_LANDING_DEEPSTALL_ENABLED
    // support for deepstall landings
    AP_Landing_Deepstall deepstall;
#endif

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
    AP_Int8 flare_effectivness_pct;

    // Land Type STANDARD GLIDE SLOPE

    enum  {
        SLOPE_STAGE_NORMAL,
        SLOPE_STAGE_APPROACH,
        SLOPE_STAGE_PREFLARE,
        SLOPE_STAGE_FINAL
    } type_slope_stage;

    struct {
        // once landed, post some landing statistics to the GCS
        bool post_stats:1;

        bool has_aborted_due_to_slope_recalc:1;
    } type_slope_flags;

    void type_slope_do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude);
    void type_slope_verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed);
    bool type_slope_verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range);

    void type_slope_adjust_landing_slope_for_rangefinder_bump(AP_FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm);

    void type_slope_setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm);
    int32_t type_slope_get_target_airspeed_cm(void);
    void type_slope_check_if_need_to_abort(const AP_FixedWing::Rangefinder_State &rangefinder_state);
    int32_t type_slope_constrain_roll(const int32_t desired_roll_cd, const int32_t level_roll_limit_cd);
    bool type_slope_request_go_around(void);
    void type_slope_log(void) const;
    bool type_slope_is_complete(void) const;
    bool type_slope_is_flaring(void) const;
    bool type_slope_is_on_approach(void) const;
    bool type_slope_is_expecting_impact(void) const;
    bool type_slope_is_throttle_suppressed(void) const;
};
