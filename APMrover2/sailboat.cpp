#include "Rover.h"

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 5000   // tacks in auto mode timeout if not successfully completed within this many milliseconds
#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed paramiter and controller, for mapping you may not want to go too fast
 - mavlink sailing messages
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse...
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - wing type sails with 'elevator' control
 - tack on depth sounder info to stop sailing into shallow water on indirect sailing routes
 - add option to do proper tacks, ie tacking on flat spot in the waves, or only try once at a certain speed, or some better method than just changing the desired heading suddenly
*/

const AP_Param::GroupInfo Sailboat::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable Sailboat
    // @Description: This enables Sailboat functionality
    // @Values: 0:Disable,1:Enable sail assist only,2:Enable
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("ENABLE", 1, Sailboat, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ANGLE_MIN
    // @DisplayName: Sail min angle
    // @Description: Mainsheet tight, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MIN", 2, Sailboat, sail_angle_min, 0),

    // @Param: ANGLE_MAX
    // @DisplayName: Sail max angle
    // @Description: Mainsheet loose, angle between centerline and boom
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_MAX", 3, Sailboat, sail_angle_max, 90),

    // @Param: ANGLE_IDEAL
    // @DisplayName: Sail ideal angle
    // @Description: Ideal angle between sail and apparent wind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ANGLE_IDEAL", 4, Sailboat, sail_angle_ideal, 25),

    // @Param: HEEL_MAX
    // @DisplayName: Sailing maximum heel angle
    // @Description: When in auto sail trim modes the heel will be limited to this value using PID control
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("HEEL_MAX", 5, Sailboat, sail_heel_angle_max, 15),

    // @Param: SAIL_NO_GO_ANGLE
    // @DisplayName: Sailing no go zone angle
    // @Description: The typical closest angle to the wind the vehicle will sail at. the vehicle will sail at this angle when going upwind
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("NO_GO_ANGLE", 6, Sailboat, sail_no_go, 45),

    // @Param: WNDSPD_MIN
    // @DisplayName: Sailboat minimum wind speed to sail in
    // @Description: Sailboat minimum wind speed to continue sail in, at lower wind speeds the sailboat will motor if one is fitted
    // @Units: m/s
    // @Range: 0 5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WNDSPD_MIN", 7, Sailboat, sail_assist_windspeed, 0),

    AP_GROUPEND
};


/*
  constructor
 */
Sailboat::Sailboat()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// Should we use sailboat navigation?
bool Sailboat::nav_enabled() const
{
    return (enable == 2) &&
           (throttle_state != Sailboat_Throttle::FORCE_MOTOR) &&
           (!throttle_assist() || tack_assist);
}

void Sailboat::init()
{
    // sailboat defaults
    if (sail_enabled()) {
        rover.g2.crash_angle.set_default(0);
    }

    if (nav_enabled()) {
        rover.g2.loit_type.set_default(1);
        rover.g2.loit_radius.set_default(5);
        rover.g2.wp_nav.set_default_overshoot(10);
    }

    // if we have a throttle of some sort allow to use it
    if (rover.g2.motors.have_skid_steering() ||
        SRV_Channels::function_assigned(SRV_Channel::k_throttle) ||
        rover.get_frame_type() != rover.g2.motors.frame_type::FRAME_TYPE_UNDEFINED) {
        throttle_state = Sailboat_Throttle::ASSIST;
    }
}

// initialise rc input (channel_mainsail), may be called intermittently
void Sailboat::init_rc_in()
{
    // get auxiliary throttle value
    RC_Channel *rc_ptr = rc().find_channel_for_option(RC_Channel::AUX_FUNC::MAINSAIL);
    if (rc_ptr != nullptr) {
        // use aux as sail input if defined
        channel_mainsail = rc_ptr;
        channel_mainsail->set_angle(100);
        channel_mainsail->set_default_dead_zone(30);
    } else {
        // use throttle channel
        channel_mainsail = rover.channel_throttle;
    }
}

// decode pilot mainsail input and return in steer_out and throttle_out arguments
// mainsail_out is in the range 0 to 100, defaults to 100 (fully relaxed) if no input configured
void Sailboat::get_pilot_desired_mainsail(float &mainsail_out)
{
    // no RC input means mainsail is moved to trim
    if ((rover.failsafe.bits & FAILSAFE_EVENT_THROTTLE) || (channel_mainsail == nullptr)) {
        mainsail_out = 100.0f;
        return;
    }
    mainsail_out = constrain_float(channel_mainsail->get_control_in(), 0.0f, 100.0f);
}

// calculate throttle and mainsail angle required to attain desired speed (in m/s)
// returns true if successful, false if sailboats not enabled
void Sailboat::get_throttle_and_mainsail_out(float desired_speed, float &throttle_out, float &mainsail_out)
{
    if (!sail_enabled()) {
        throttle_out = 0.0f;
        mainsail_out = 0.0f;
        return;
    }

    // run speed controller if motor is forced on or if assistance is enabled for low speeds or tacking
    if ((throttle_state == Sailboat_Throttle::FORCE_MOTOR) || throttle_assist()) {
        // run speed controller - duplicate of calls found in mode::calc_throttle();
        throttle_out = 100.0f * rover.g2.attitude_control.get_throttle_out_speed(desired_speed,
                                                                        rover.g2.motors.limit.throttle_lower,
                                                                        rover.g2.motors.limit.throttle_upper,
                                                                        rover.g.speed_cruise,
                                                                        rover.g.throttle_cruise * 0.01f,
                                                                        rover.G_Dt);
    } else {
        throttle_out = 0.0f;
    }

    //
    // mainsail control
    //
    // if we are motoring or attempting to reverse relax the sail
    if (throttle_state == Sailboat_Throttle::FORCE_MOTOR || !is_positive(desired_speed)) {
        mainsail_out = 100.0f;
    } else {
        // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no difference so take abs
        float wind_dir_apparent = fabsf(rover.g2.windvane.get_apparent_wind_direction_rad());
        wind_dir_apparent = degrees(wind_dir_apparent);

        // set the main sail to the ideal angle to the wind
        float mainsail_angle = wind_dir_apparent -sail_angle_ideal;

        // make sure between allowable range
        mainsail_angle = constrain_float(mainsail_angle,sail_angle_min, sail_angle_max);

        // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
        float mainsail_base = linear_interpolate(0.0f, 100.0f, mainsail_angle,sail_angle_min,sail_angle_max);

        // use PID controller to sheet out
        const float pid_offset = rover.g2.attitude_control.get_sail_out_from_heel(radians(sail_heel_angle_max), rover.G_Dt) * 100.0f;

        mainsail_out = constrain_float((mainsail_base + pid_offset), 0.0f ,100.0f);
    }
}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Sailboat::get_VMG() const
{
    // return 0 if not heading towards waypoint
    if (!rover.control_mode->is_autopilot_mode()) {
        return 0.0f;
    }

    float speed;
    if (!rover.g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }

    return (speed * cosf(wrap_PI(radians(rover.g2.wp_nav.wp_bearing_cd() * 0.01f) - rover.ahrs.yaw)));
}

// handle user initiated tack while in acro mode
void Sailboat::handle_tack_request_acro()
{
    if (!nav_enabled() || currently_tacking) {
        return;
    }
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    currently_tacking = true;
    tack_heading_rad = wrap_2PI(rover.ahrs.yaw + 2.0f * wrap_PI((rover.g2.windvane.get_absolute_wind_direction_rad() - rover.ahrs.yaw)));

    auto_tack_request_ms = AP_HAL::millis();
}

// return target heading in radians when tacking (only used in acro)
float Sailboat::get_tack_heading_rad() const
{
    return tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Sailboat::handle_tack_request_auto()
{
    if (!nav_enabled() || currently_tacking) {
        return;
    }

    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    auto_tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Sailboat::clear_tack()
{
    currently_tacking = false;
    tack_assist = false;
    auto_tack_request_ms = 0;
}

// returns true if boat is currently tacking
bool Sailboat::tacking() const
{
    return nav_enabled() && currently_tacking;
}

// returns true if sailboat should take a indirect navigation route to go upwind
// desired_heading should be in centi-degrees
bool Sailboat::use_indirect_route(float desired_heading_cd) const
{
    if (!nav_enabled()) {
        return false;
    }

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    return fabsf(wrap_PI(rover.g2.windvane.get_absolute_wind_direction_rad() - desired_heading_rad)) <= radians(sail_no_go);
}

// if we can't sail on the desired heading then we should pick the best heading that we can sail on
// this function assumes the caller has already checked sailboat_use_indirect_route(desired_heading_cd) returned true
float Sailboat::calc_heading(float desired_heading_cd)
{
    if (!nav_enabled()) {
        return desired_heading_cd;
    }

    bool should_tack = false;

    // check for user requested tack
    uint32_t now = AP_HAL::millis();
    if (auto_tack_request_ms != 0) {
        // set should_tack flag is user requested tack within last 0.5 sec
        should_tack = ((now - auto_tack_request_ms) < 500);
        auto_tack_request_ms = 0;
    }

    // calculate left and right no go headings looking upwind
    const float left_no_go_heading_rad = wrap_2PI(rover.g2.windvane.get_absolute_wind_direction_rad() + radians(sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(rover.g2.windvane.get_absolute_wind_direction_rad() - radians(sail_no_go));

    // calculate current tack, Port if heading is left of no-go, STBD if right of no-go
    Sailboat_Tack current_tack;
    if (is_negative(rover.g2.windvane.get_apparent_wind_direction_rad())) {
        current_tack = TACK_PORT;
    } else {
        current_tack = TACK_STARBOARD;
    }

    // trigger tack if cross track error larger than waypoint_overshoot parameter
    // this effectively defines a 'corridor' of width 2*waypoint_overshoot that the boat will stay within
    const float cross_track_error = rover.g2.wp_nav.crosstrack_error();
    if ((fabsf(cross_track_error) >= rover.g2.wp_nav.get_overshoot()) && !is_zero(rover.g2.wp_nav.get_overshoot()) && !currently_tacking) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(cross_track_error) && (current_tack == TACK_STARBOARD)) {
            should_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(cross_track_error) && (current_tack == TACK_PORT)) {
            should_tack = true;
        }
    }

    // if tack triggered, calculate target heading
    if (should_tack) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        // calculate target heading for the new tack
        switch (current_tack) {
            case TACK_PORT:
                tack_heading_rad = right_no_go_heading_rad;
                break;
            case TACK_STARBOARD:
                tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        currently_tacking = true;
        auto_tack_start_ms = AP_HAL::millis();
    }

    // if we are tacking we maintain the target heading until the tack completes or times out
    if (currently_tacking) {
        // check if we have reached target
        if (fabsf(wrap_PI(tack_heading_rad - rover.ahrs.yaw)) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            clear_tack();
        } else if ((now - auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            // tack has taken too long
            if ((throttle_state == Sailboat_Throttle::ASSIST) && (now - auto_tack_start_ms) < (3.0f * SAILBOAT_AUTO_TACKING_TIMEOUT_MS)) {
                // if we have throttle available use it for another two time periods to get the tack done
                tack_assist = true;
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
                clear_tack();
            }
        }
        // return tack target heading
        return degrees(tack_heading_rad) * 100.0f;
    }

    // return closest possible heading to wind
    if (current_tack == TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}

// should we use the throttle?
bool Sailboat::throttle_assist() const
{
    // throttle is assist is disabled
    if (throttle_state != Sailboat_Throttle::ASSIST) {
        return false;
    }

    // assist with a tack
    if (tack_assist) {
        return true;
    }

    // wind speed is less than sailing cut-off
    if (!is_zero(sail_assist_windspeed) &&
        rover.g2.windvane.wind_speed_enabled() &&
        rover.g2.windvane.get_true_wind_speed() < sail_assist_windspeed) {
        return true;
    }

    return false;
}
