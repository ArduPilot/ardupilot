#include "Rover.h"

#define SAILBOAT_AUTO_TACKING_TIMEOUT_MS 50000  // tacks in auto mode timeout if not successfully completed within this many milliseconds
#define SAILBOAT_TACKING_ACCURACY_DEG 10        // tack is considered complete when vehicle is within this many degrees of target tack angle
/*
To Do List
 - Improve tacking in light winds and bearing away in strong wings
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - max speed paramiter and contoller, for maping you may not want to go too fast
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

// update mainsail's desired angle based on wind speed and direction and desired speed (in m/s)
void Rover::sailboat_update_mainsail(float desired_speed)
{
    if (!g2.motors.has_sail()) {
        return;
    }

    // relax sail if desired speed is zero
    if (!is_positive(desired_speed)) {
        g2.motors.set_mainsail(100.0f);
        return;
    }

    // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no difference so take abs
    float wind_dir_apparent = fabsf(g2.windvane.get_apparent_wind_direction_rad());
    wind_dir_apparent = degrees(wind_dir_apparent);

    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent - g2.sail_angle_ideal;

    // make sure between allowable range
    mainsail_angle = constrain_float(mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = linear_interpolate(0.0f, 100.0f, mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // use PID controller to sheet out
    const float pid_offset = g2.attitude_control.get_sail_out_from_heel(radians(g2.sail_heel_angle_max), G_Dt) * 100.0f;

    mainsail = constrain_float((mainsail+pid_offset), 0.0f ,100.0f);
    g2.motors.set_mainsail(mainsail);
}

// Velocity Made Good, this is the speed we are traveling towards the desired destination
// only for logging at this stage
// https://en.wikipedia.org/wiki/Velocity_made_good
float Rover::sailboat_get_VMG() const
{
    // return 0 if not heading towards waypoint
    if (!control_mode->is_autopilot_mode()) {
        return 0.0f;
    }

    float speed;
    if (!g2.attitude_control.get_forward_speed(speed)) {
        return 0.0f;
    }
    return (speed * cosf(wrap_PI(radians(nav_controller->target_bearing_cd()) - ahrs.yaw)));
}

// handle user initiated tack while in acro mode
void Rover::sailboat_handle_tack_request_acro()
{
    // set tacking heading target to the current angle relative to the true wind but on the new tack
    sailboat.tacking = true;
    sailboat.tack_heading_rad = wrap_2PI(ahrs.yaw + 2.0f * wrap_PI((g2.windvane.get_absolute_wind_direction_rad() - ahrs.yaw)));
}

// return target heading in radians when tacking (only used in acro)
float Rover::sailboat_get_tack_heading_rad() const
{
    return sailboat.tack_heading_rad;
}

// handle user initiated tack while in autonomous modes (Auto, Guided, RTL, SmartRTL, etc)
void Rover::sailboat_handle_tack_request_auto()
{
    // record time of request for tack.  This will be processed asynchronously by sailboat_calc_heading
    sailboat.auto_tack_request_ms = AP_HAL::millis();
}

// clear tacking state variables
void Rover::sailboat_clear_tack()
{
    sailboat.tacking = false;
    sailboat.auto_tack_request_ms = 0;
}

// returns true if boat is currently tacking
bool Rover::sailboat_tacking() const
{
    return sailboat.tacking;
}

// returns true if sailboat should take a indirect navigation route to go upwind
// desired_heading should be in centi-degrees
bool Rover::sailboat_use_indirect_route(float desired_heading_cd) const
{
    if (!g2.motors.has_sail()) {
        return false;
    }

    // convert desired heading to radians
    const float desired_heading_rad = radians(desired_heading_cd * 0.01f);

    // check if desired heading is in the no go zone, if it is we can't go direct
    return fabsf(wrap_PI(g2.windvane.get_absolute_wind_direction_rad() - desired_heading_rad)) <= radians(g2.sail_no_go);
}

// if we can't sail on the desired heading then we should pick the best heading that we can sail on
// this function assumes the caller has already checked sailboat_use_indirect_route(desired_heading_cd) returned true
float Rover::sailboat_calc_heading(float desired_heading_cd)
{
    if (!g2.motors.has_sail()) {
        return desired_heading_cd;
    }

    bool should_tack = false;

    // check for user requested tack
    uint32_t now = AP_HAL::millis();
    if (sailboat.auto_tack_request_ms != 0) {
        // set should_tack flag is user requested tack within last 0.5 sec
        should_tack = ((now - sailboat.auto_tack_request_ms) < 500);
        sailboat.auto_tack_request_ms = 0;
    }

    // calculate left and right no go headings looking upwind
    const float left_no_go_heading_rad = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() + radians(g2.sail_no_go));
    const float right_no_go_heading_rad = wrap_2PI(g2.windvane.get_absolute_wind_direction_rad() - radians(g2.sail_no_go));

    // calculate current tack, Port if heading is left of no-go, STBD if right of no-go
    Sailboat_Tack current_tack;
    if (is_negative(g2.windvane.get_apparent_wind_direction_rad())) {
        current_tack = TACK_PORT;
    } else {
        current_tack = TACK_STARBOARD;
    }

    // trigger tack if cross track error larger than waypoint_overshoot parameter
    // this effectively defines a 'corridor' of width 2*waypoint_overshoot that the boat will stay within
    if ((fabsf(rover.nav_controller->crosstrack_error()) >= g.waypoint_overshoot) && !is_zero(g.waypoint_overshoot) && !sailboat_tacking()) {
        // make sure the new tack will reduce the cross track error
        // if were on starboard tack we are traveling towards the left hand boundary
        if (is_positive(rover.nav_controller->crosstrack_error()) && (current_tack == TACK_STARBOARD)) {
            should_tack = true;
        }
        // if were on port tack we are traveling towards the right hand boundary
        if (is_negative(rover.nav_controller->crosstrack_error()) && (current_tack == TACK_PORT)) {
            should_tack = true;
        }
    }

    // if tack triggered, calculate target heading
    if (should_tack) {
        gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking");
        // calculate target heading for the new tack
        switch (current_tack) {
            case TACK_PORT:
                sailboat.tack_heading_rad = right_no_go_heading_rad;
                break;
            case TACK_STARBOARD:
                sailboat.tack_heading_rad = left_no_go_heading_rad;
                break;
        }
        sailboat.tacking = true;
        sailboat.auto_tack_start_ms = AP_HAL::millis();
    }

    // if we are tacking we maintain the target heading until the tack completes or times out
    if (sailboat.tacking) {
        // check if we have reached target
        if (fabsf(wrap_PI(sailboat.tack_heading_rad - ahrs.yaw)) <= radians(SAILBOAT_TACKING_ACCURACY_DEG)) {
            sailboat_clear_tack();
        } else if ((now - sailboat.auto_tack_start_ms) > SAILBOAT_AUTO_TACKING_TIMEOUT_MS) {
            // tack has taken too long
            gcs().send_text(MAV_SEVERITY_INFO, "Sailboat: Tacking timed out");
            sailboat_clear_tack();
        }
        // return tack target heading
        return degrees(sailboat.tack_heading_rad) * 100.0f;
    }

    // return closest possible heading to wind
    if (current_tack == TACK_PORT) {
        return degrees(left_no_go_heading_rad) * 100.0f;
    } else {
        return degrees(right_no_go_heading_rad) * 100.0f;
    }
}
