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
#include <AP_Navigation/AP_Navigation.h>
#include <PID/PID.h>

#ifndef HAL_LANDING_DEEPSTALL_ENABLED
#define HAL_LANDING_DEEPSTALL_ENABLED (BOARD_FLASH_SIZE > 1024)
#endif

class AP_Landing;

/// @class  AP_Landing_Deepstall
/// @brief  Class managing Plane Deepstall landing methods
class AP_Landing_Deepstall
{
private:
    friend class AP_Landing;

    // constructor
    AP_Landing_Deepstall(AP_Landing &_landing) :
        landing(_landing)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    AP_Landing &landing;

    static const struct AP_Param::GroupInfo var_info[];

    AP_Float forward_speed;
    AP_Float slope_a;
    AP_Float slope_b;
    AP_Float approach_extension;
    AP_Float down_speed;
    AP_Float slew_speed;
    AP_Int16 elevator_pwm;
    AP_Float handoff_airspeed;
    AP_Float handoff_lower_limit_airspeed;
    AP_Float L1_period;
    AP_Float L1_i;
    AP_Float yaw_rate_limit;
    AP_Float time_constant;
    AP_Float min_abort_alt;
    AP_Float aileron_scalar;
    int32_t loiter_sum_cd;         // used for tracking the progress on loitering
    DEEPSTALL_STAGE stage;
    Location landing_point;
    Location extended_approach;
    Location breakout_location;
    Location arc;
    Location arc_entry;
    Location arc_exit;
    float target_heading_deg;      // target heading for the deepstall in degrees
    uint32_t stall_entry_time;     // time when the aircrafted enter the stall (in millis)
    uint16_t initial_elevator_pwm; // PWM to start slewing the elevator up from
    uint32_t last_time;            // last time the controller ran
    float L1_xtrack_i;             // L1 integrator for navigation
    PID ds_PID;
    int32_t last_target_bearing;   // used for tracking the progress on loitering
    float crosstrack_error; // current crosstrack error
    float predicted_travel_distance; // distance the aircraft is perdicted to travel during deepstall
    bool hold_level; // locks the target yaw rate of the aircraft to 0, serves to hold the aircraft level at all times
    float approach_alt_offset;     // approach altitude offset

    //public AP_Landing interface
    void do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude);
    void verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed);
    bool verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms,
            const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range);
    void setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc,
            const Location &current_loc, int32_t &target_altitude_offset_cm);
    bool override_servos(void);
    bool request_go_around(void);
    bool get_target_altitude_location(Location &location);
    int32_t get_target_airspeed_cm(void) const;
    bool is_throttle_suppressed(void) const;
    bool is_flying_forward(void) const;
    bool is_on_approach(void) const;
    bool terminate(void);
    void Log(void) const;

    bool send_deepstall_message(mavlink_channel_t chan) const;

    const AP_PIDInfo& get_pid_info(void) const;

    //private helpers
    void build_approach_path(bool use_current_heading);
    float predict_travel_distance(const Vector3f wind, const float height, const bool print);
    bool verify_breakout(const Location &current_loc, const Location &target_loc, const float height_error) const;
    float update_steering(void);

    #define DEEPSTALL_LOITER_ALT_TOLERANCE 5.0f
};
