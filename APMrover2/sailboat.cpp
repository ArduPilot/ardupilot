#include "Rover.h"

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
