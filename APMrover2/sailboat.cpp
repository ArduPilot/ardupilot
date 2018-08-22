#include "Rover.h"

/*
To Do List
 ----- short term
 - add calibration code for analog wind Vane
 - setup log of relevant sailing variables
 - add sailboat parameters, ie no go zone, sheeting range, ideal sail aoa
 - auto sheet mode, not sure if acro will work well, may be very tricky to go at a defined speed, depends on wind speed and heading. dead down wind it can be bad to try and slowdown
 suggest instead a auto sheet mode, where the sheet is trimmed for the heading the boat is on automatically this would work with acro steering, we could then have a try and sail or sheet out and stop user input
 this would also work for acro mode motor sailing
 - consider drag vs lift sailing differences, ie upwind sail is like wing, dead down wind sail is like parachute
 - auto mode
 - Set up PID for the mainsail, So that it will hold at a maximum heal value, by sheeting out and/or changing heading (possibly some sort of ratio parameter here, ie only sheet out or only bear away or combination)
 - allow tack on geofence for auto mode use on restricted water

 ----- Long term
 - add wind speed sensor
 - mavlink sailing messages
 - sailing loiter, boat stays inside loiter circle, more like plane than rover, will carry on sailing but stay close to a point, cant slow down too much or will lose steering
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse esc
 - smart decision making, ie tack on windshifts, what to do if stuck head to wind
 - sailing 'mode' switching ie in light wind you may have larger vmg by pointing lower and going faster, where as it may be better to go high and slow in heavy wind, tricky because very boat dependent
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitoring to prevent nose diving in heavy weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independent sheeting for main and jib
 - wing type sails with 'elevator' control
*/

// directly set a mainsail value (used for manual modes)
void Rover::sailboat_set_mainsail(float mainsail)
{
    if (!g2.motors.has_sail()) {
        return;
    }
    g2.motors.set_mainsail(mainsail);
}

// update mainsail's desired angle based on wind speed and direction
void Rover::sailboat_update_mainsail()
{
    if (!g2.motors.has_sail()) {
        return;
    }

    // + is wind over starboard side, - is wind over port side, but as the sails are sheeted the same on each side it makes no dirence so take abs
    float wind_dir_apparent = fabsf(windvane.get_apparent_wind_direction_rad());

    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent - g2.sail_angle_ideal;

    // make sure between allowable range
    mainsail_angle = constrain_float(mainsail_angle, g2.sail_angle_min, g2.sail_angle_max);

    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = ((mainsail_angle - g2.sail_angle_min)/(g2.sail_angle_max - g2.sail_angle_min)) * 100.0f;

    g2.motors.set_mainsail(mainsail);
}

// Velocity Made Good, this is the speed we are travling towards the desired destination
// Not sure how usefull this is at this stage, but would be usefull to log
float Rover::sailboat_VMG(float target_heading)
{
    // https://en.wikipedia.org/wiki/Velocity_made_good

    float speed;
    g2.attitude_control.get_forward_speed(speed);

    float vmg = speed * cosf(wrap_PI(target_heading - ahrs.yaw));

    return vmg;
}
