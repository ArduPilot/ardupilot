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

    // @Param: SLOPE_RCALC
    // @DisplayName: Landing slope re-calc threshold
    // @Description: This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your altitude is lower than the intended slope path. This value is the threshold of the correction to re-calculate the landing approach slope. Set to zero to keep the original slope all the way down and any detected baro drift will be corrected by pitching/throttling up to snap back to resume the original slope path. Otherwise, when a rangefinder altitude correction exceeds this threshold it will trigger a slope re-calculate to give a shallower slope. This also smoothes out the approach when flying over objects such as trees. Recommend a value of 2m.
    // @Range: 0 5
    // @Units: m
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO("SLOPE_RCALC", 1, AP_Landing, slope_recalc_shallow_threshold, 2.0f),

    // @Param: ABORT_DEG
    // @DisplayName: Landing auto-abort slope threshold
    // @Description: This parameter is used when using a rangefinder during landing for altitude correction from baro drift (RNGFND_LANDING=1) and the altitude correction indicates your actual altitude is higher than the intended slope path. Normally it would pitch down steeply but that can result in a crash with high airspeed so this allows remembering the baro offset and self-abort the landing and come around for another landing with the correct baro offset applied for a perfect slope. An auto-abort go-around will only happen once, next attempt will not auto-abort again. This operation happens entirely automatically in AUTO mode. This value is the delta degrees threshold to trigger the go-around compared to the original slope. Example: if set to 5 deg and the mission planned slope is 15 deg then if the new slope is 21 then it will go-around. Set to 0 to disable. Requires LAND_SLOPE_RCALC > 0.
    // @Range: 0 90
    // @Units: deg
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ABORT_DEG", 2, AP_Landing, slope_recalc_steep_threshold_to_abort, 0),

    // @Param: PITCH_CD
    // @DisplayName: Landing Pitch
    // @Description: Used in autoland to give the minimum pitch in the final stage of landing (after the flare). This parameter can be used to ensure that the final landing attitude is appropriate for the type of undercarriage on the aircraft. Note that it is a minimum pitch only - the landing code will control pitch above this value to try to achieve the configured landing sink rate.
    // @Units: cdeg
    // @User: Advanced
    AP_GROUPINFO("PITCH_CD", 3, AP_Landing, pitch_cd, 0),

    // @Param: FLARE_ALT
    // @DisplayName: Landing flare altitude
    // @Description: Altitude in autoland at which to lock heading and flare to the LAND_PITCH_CD pitch. Note that this option is secondary to LAND_FLARE_SEC. For a good landing it preferable that the flare is triggered by LAND_FLARE_SEC.
    // @Units: m
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_ALT", 4, AP_Landing, flare_alt, 3.0f),

    // @Param: FLARE_SEC
    // @DisplayName: Landing flare time
    // @Description: Vertical time before landing point at which to lock heading and flare with the motor stopped. This is vertical time, and is calculated based solely on the current height above the ground and the current descent rate.  Set to 0 if you only wish to flare based on altitude (see LAND_FLARE_ALT).
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("FLARE_SEC", 5, AP_Landing, flare_sec, 2.0f),

    // @Param: PF_ALT
    // @DisplayName: Landing pre-flare altitude
    // @Description: Altitude to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. The pre-flare flight stage trigger works just like LAND_FLARE_ALT but higher. Disabled when LAND_PF_ARSPD is 0.
    // @Units: m
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_ALT", 6, AP_Landing, pre_flare_alt, 10.0f),

    // @Param: PF_SEC
    // @DisplayName: Landing pre-flare time
    // @Description: Vertical time to ground to trigger pre-flare flight stage where LAND_PF_ARSPD controls airspeed. This pre-flare flight stage trigger works just like LAND_FLARE_SEC but earlier. Disabled when LAND_PF_ARSPD is 0.
    // @Units: s
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_SEC", 7, AP_Landing, pre_flare_sec, 6.0f),

    // @Param: PF_ARSPD
    // @DisplayName: Landing pre-flare airspeed
    // @Description: Desired airspeed during pre-flare flight stage. This is useful to reduce airspeed just before the flare. Use 0 to disable.
    // @Units: m/s
    // @Range: 0 30
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("PF_ARSPD", 8, AP_Landing, pre_flare_airspeed, 0),

    // @Param: THR_SLEW
    // @DisplayName: Landing throttle slew rate
    // @Description: This parameter sets the slew rate for the throttle during auto landing. When this is zero the THR_SLEWRATE parameter is used during landing. The value is a percentage throttle change per second, so a value of 20 means to advance the throttle over 5 seconds on landing. Values below 50 are not recommended as it may cause a stall when airspeed is low and you can not throttle up fast enough.
    // @Units: %
    // @Range: 0 127
    // @Increment: 1
    // @User: User
    AP_GROUPINFO("THR_SLEW", 9, AP_Landing, throttle_slewrate, 0),

    // @Param: DISARMDELAY
    // @DisplayName: Landing disarm delay
    // @Description: After a landing has completed using a LAND waypoint, automatically disarm after this many seconds have passed. Use 0 to not disarm.
    // @Units: s
    // @Increment: 1
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("DISARMDELAY", 10, AP_Landing, disarm_delay, 20),

    // @Param: THEN_NEUTRL
    // @DisplayName: Set servos to neutral after landing
    // @Description: When enabled, after an autoland and auto-disarm via LAND_DISARMDELAY happens then set all servos to neutral. This is helpful when an aircraft has a rough landing upside down or a crazy angle causing the servos to strain.
    // @Values: 0:Disabled, 1:Servos to Neutral, 2:Servos to Zero PWM
    // @User: Advanced
    AP_GROUPINFO("THEN_NEUTRL", 11, AP_Landing, then_servos_neutral, 0),

    // @Param: ABORT_THR
    // @DisplayName: Landing abort using throttle
    // @Description: Allow a landing abort to trigger with a throttle > 95%
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("ABORT_THR", 12, AP_Landing, abort_throttle_enable, 0),

    // @Param: FLAP_PERCNT
    // @DisplayName: Landing flap percentage
    // @Description: The amount of flaps (as a percentage) to apply in the landing approach and flare of an automatic landing
    // @Range: 0 100
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO("FLAP_PERCNT", 13, AP_Landing, flap_percent, 0),

    // @Param: TYPE
    // @DisplayName: Auto-landing type
    // @Description: Specifies the auto-landing type to use
    // @Values: 0:Standard Glide Slope, 1:Deepstall
    // @User: Standard
    AP_GROUPINFO("TYPE",    14, AP_Landing, type, TYPE_STANDARD_GLIDE_SLOPE),

    // @Group: DS_
    // @Path: AP_Landing_Deepstall.cpp
    AP_SUBGROUPINFO(deepstall, "DS_", 15, AP_Landing, AP_Landing_Deepstall),
    
    AP_GROUPEND
};

    // constructor
AP_Landing::AP_Landing(AP_Mission &_mission, AP_AHRS &_ahrs, AP_SpdHgtControl *_SpdHgt_Controller, AP_Navigation *_nav_controller, AP_Vehicle::FixedWing &_aparm,
                       set_target_altitude_proportion_fn_t _set_target_altitude_proportion_fn,
                       constrain_target_altitude_location_fn_t _constrain_target_altitude_location_fn,
                       adjusted_altitude_cm_fn_t _adjusted_altitude_cm_fn,
                       adjusted_relative_altitude_cm_fn_t _adjusted_relative_altitude_cm_fn,
                       disarm_if_autoland_complete_fn_t _disarm_if_autoland_complete_fn,
                       update_flight_stage_fn_t _update_flight_stage_fn) :
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
    ,deepstall(*this)
{
    AP_Param::setup_object_defaults(this, var_info);
}


void AP_Landing::do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    Log(); // log old state so we get a nice transition from old to new here

    flags.commanded_go_around = false;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_do_land(cmd, relative_altitude);
        break;
    case TYPE_DEEPSTALL:
        deepstall.do_land(cmd, relative_altitude);
        break;
    default:
        // a incorrect type is handled in the verify_land
        break;
    }

    Log();
}

/*
  update navigation for landing. Called when on landing approach or
  final flare
 */
bool AP_Landing::verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms, const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
    bool success = true;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        success = type_slope_verify_land(prev_WP_loc, next_WP_loc, current_loc,
                height, sink_rate, wp_proportion, last_flying_ms, is_armed, is_flying, rangefinder_state_in_range);
        break;
    case TYPE_DEEPSTALL:
        success = deepstall.verify_land(prev_WP_loc, next_WP_loc, current_loc,
                                             height, sink_rate, wp_proportion, last_flying_ms, is_armed, is_flying, rangefinder_state_in_range);
        break;
    default:
        // returning TRUE while executing verify_land() will increment the
        // mission index which in many cases will trigger an RTL for end-of-mission
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Landing configuration error, invalid LAND_TYPE");
        success = true;
        break;
    }
    Log();
    return success;
}


bool AP_Landing::verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
    const int32_t auto_state_takeoff_altitude_rel_cm, bool &throttle_suppressed)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_verify_abort_landing(prev_WP_loc, next_WP_loc, throttle_suppressed);
        break;
    case TYPE_DEEPSTALL:
        deepstall.verify_abort_landing(prev_WP_loc, next_WP_loc, throttle_suppressed);
        break;
    default:
        break;
    }

    // see if we have reached abort altitude
     if (adjusted_relative_altitude_cm_fn() > auto_state_takeoff_altitude_rel_cm) {
         next_WP_loc = current_loc;
         mission.stop();
         if (restart_landing_sequence()) {
             mission.resume();
         }
         // else we're in AUTO with a stopped mission and handle_auto_mode() will set RTL
     }

     Log();

     // make sure to always return false so it leaves the mission index alone
     return false;
}

void AP_Landing::adjust_landing_slope_for_rangefinder_bump(AP_Vehicle::FixedWing::Rangefinder_State &rangefinder_state, Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc, const float wp_distance, int32_t &target_altitude_offset_cm)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_adjust_landing_slope_for_rangefinder_bump(rangefinder_state, prev_WP_loc, next_WP_loc, current_loc, wp_distance, target_altitude_offset_cm);
        break;
    case TYPE_DEEPSTALL:
    default:
        break;
    }
}

// send out any required mavlink messages
bool AP_Landing::send_landing_message(mavlink_channel_t chan) {
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_DEEPSTALL:
        return deepstall.send_deepstall_message(chan);
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

bool AP_Landing::is_flaring(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_flaring();
    case TYPE_DEEPSTALL:
    default:
        return false;
    }
}

// return true while the aircraft is performing a landing approach
// when true the vehicle will:
//   - disable ground steering
//   - call setup_landing_glide_slope() and adjust_landing_slope_for_rangefinder_bump()
//   - will be considered flying if sink rate > 0.2, and can trigger crash detection
bool AP_Landing::is_on_approach(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_on_approach();
    case TYPE_DEEPSTALL:
        return deepstall.is_on_approach();
    default:
        return false;
    }
}

// return true while the aircraft is allowed to perform ground steering
bool AP_Landing::is_ground_steering_allowed(void) const
{
    if (!flags.in_progress) {
        return true;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_on_approach();
    case TYPE_DEEPSTALL:
        return false;
    default:
        return true;
    }
}

// return true when at the last stages of a land when an impact with the ground is expected soon
// when true is_flying knows that the vehicle was expecting to stop flying, possibly because of a hard impact
bool AP_Landing::is_expecting_impact(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_expecting_impact();
    case TYPE_DEEPSTALL:
    default:
        return false;
    }
}

// returns true when the landing library has overriden any servos
bool AP_Landing::override_servos(void) {
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_DEEPSTALL:
        return deepstall.override_servos();
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

// returns a PID_Info object if there is one available for the selected landing
// type, otherwise returns a nullptr, indicating no data to be logged/sent
const DataFlash_Class::PID_Info* AP_Landing::get_pid_info(void) const
{
    switch (type) {
    case TYPE_DEEPSTALL:
        return &deepstall.get_pid_info();
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return nullptr;
    }
}

/*
  a special glide slope calculation for the landing approach

  During the land approach use a linear glide slope to a point
  projected through the landing point. We don't use the landing point
  itself as that leads to discontinuities close to the landing point,
  which can lead to erratic pitch control
 */

void AP_Landing::setup_landing_glide_slope(const Location &prev_WP_loc, const Location &next_WP_loc, const Location &current_loc, int32_t &target_altitude_offset_cm)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_setup_landing_glide_slope(prev_WP_loc, next_WP_loc, current_loc, target_altitude_offset_cm);
        break;
    case TYPE_DEEPSTALL:
    default:
        break;
    }
}

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
        gcs().send_text(MAV_SEVERITY_NOTICE, "Restarted landing sequence. Climbing to %dm", cmd.content.location.alt/100);
        success =  true;
    }
    else if (do_land_start_index != 0 &&
            mission.set_current_cmd(do_land_start_index))
    {
        // look for a DO_LAND_START and use that index
        gcs().send_text(MAV_SEVERITY_NOTICE, "Restarted landing via DO_LAND_START: %d",do_land_start_index);
        success =  true;
    }
    else if (prev_cmd_with_wp_index != AP_MISSION_CMD_INDEX_NONE &&
               mission.set_current_cmd(prev_cmd_with_wp_index))
    {
        // if a suitable navigation waypoint was just executed, one that contains lat/lng/alt, then
        // repeat that cmd to restart the landing from the top of approach to repeat intended glide slope
        gcs().send_text(MAV_SEVERITY_NOTICE, "Restarted landing sequence at waypoint %d", prev_cmd_with_wp_index);
        success =  true;
    } else {
        gcs().send_text(MAV_SEVERITY_WARNING, "Unable to restart landing sequence");
        success =  false;
    }

    if (success) {
        // exit landing stages if we're no longer executing NAV_LAND
        update_flight_stage_fn();
    }

    Log();
    return success;
}

int32_t AP_Landing::constrain_roll(const int32_t desired_roll_cd, const int32_t level_roll_limit_cd)
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_constrain_roll(desired_roll_cd, level_roll_limit_cd);
    case TYPE_DEEPSTALL:
    default:
        return desired_roll_cd;
    }
}

// returns true if landing provided a Location structure with the current target altitude
bool AP_Landing::get_target_altitude_location(Location &location)
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_DEEPSTALL:
        return deepstall.get_target_altitude_location(location);
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}

/*
 * Determine how aligned heading_deg is with the wind. Return result
 * is 1.0 when perfectly aligned heading into wind, -1 when perfectly
 * aligned with-wind, and zero when perfect cross-wind. There is no
 * distinction between a left or right cross-wind. Wind speed is ignored
 */
float AP_Landing::wind_alignment(const float heading_deg)
{
    const Vector3f wind = ahrs.wind_estimate();
    const float wind_heading_rad = atan2f(-wind.y, -wind.x);
    return cosf(wind_heading_rad - radians(heading_deg));
}

/*
 * returns head-wind in m/s, 0 for tail-wind.
 */
float AP_Landing::head_wind(void)
{
    const float alignment = wind_alignment(ahrs.yaw_sensor*0.01f);

    if (alignment <= 0) {
        return 0;
    }

    return alignment * ahrs.wind_estimate().length();
}

/*
 * returns target airspeed in cm/s depending on flight stage
 */
int32_t AP_Landing::get_target_airspeed_cm(void)
{
    if (!flags.in_progress) {
        // not landing, use regular cruise airspeed
        return aparm.airspeed_cruise_cm;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_get_target_airspeed_cm();
    case TYPE_DEEPSTALL:
        return deepstall.get_target_airspeed_cm();
    default:
        // don't return the landing airspeed, because if type is invalid we have
        // no postive indication that the land airspeed has been configured or
        // how it was meant to be utilized
        return SpdHgt_Controller->get_target_airspeed();
    }
}

/*
 * request a landing abort given the landing type
 * return true on success
 */
bool AP_Landing::request_go_around(void)
{
    bool success = false;

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        success = type_slope_request_go_around();
        break;
    case TYPE_DEEPSTALL:
        success = deepstall.request_go_around();
        break;
    default:
        break;
    }

    Log();
    return success;
}

void AP_Landing::handle_flight_stage_change(const bool _in_landing_stage)
{
    Log(); // log old value to plot discrete transitions
    flags.in_progress = _in_landing_stage;
    flags.commanded_go_around = false;
    Log();
}

/*
 * returns true when a landing is complete, usually used to disable throttle
 */
bool AP_Landing::is_complete(void) const
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_complete();
    case TYPE_DEEPSTALL:
        return false;
    default:
        return true;
    }
}

void AP_Landing::Log(void) const
{
    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        type_slope_log();
        break;
    case TYPE_DEEPSTALL:
        deepstall.Log();
        break;
    default:
        break;
    }
}

/*
 * returns true when throttle should be suppressed while landing
 */
bool AP_Landing::is_throttle_suppressed(void) const
{
    if (!flags.in_progress) {
        return false;
    }

    switch (type) {
    case TYPE_STANDARD_GLIDE_SLOPE:
        return type_slope_is_throttle_suppressed();
    case TYPE_DEEPSTALL:
        return deepstall.is_throttle_suppressed();
    default:
        return false;
    }
}

/*
 * returns false when the vehicle might not be flying forward while landing
 */
bool AP_Landing::is_flying_forward(void) const
{
    if (!flags.in_progress) {
        return true;
    }

    switch (type) {
    case TYPE_DEEPSTALL:
        return deepstall.is_flying_forward();
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return true;
    }
}

/*
 * attempt to terminate flight with an immediate landing
 * returns true if the landing library can and is terminating the landing
 */
bool AP_Landing::terminate(void) {
    switch (type) {
    case TYPE_DEEPSTALL:
        return deepstall.terminate();
    case TYPE_STANDARD_GLIDE_SLOPE:
    default:
        return false;
    }
}
