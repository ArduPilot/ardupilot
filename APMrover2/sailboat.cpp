#include "Rover.h"

/*
To Do List
 ----- short term
 - add calabration code for analog wind Vane
 - setup log of relevent sailing varables
 - add sailboat paramiters, ie no go zone, sheeting range, ideal sail aoa
 - auto sheet mode, not sure if acro will work well, may be very tricky to go at a defined speed, depends on wind speed and heading. dead down wind it can be bad to try and slowdown
 suggest insted a auto sheet mode, where the sheet is trimed for the heading the boat is on automticaly this would work with acro sterring, we could then have a try and sail or sheet out and stop user imput
 this would also work for acro mode motor sailing 
 - consider drag vs lift sailing diffrences, ie upwind sail is like wing, dead down wind sail is like prachute
 - auto mode
 - Set up PID for the mainsail, So that it will hold at a maximum heal value, by sheeting out and/or changing heading (posibly some sort of rato paramiter here, ie olny sheet out or only bear away or combination)
 - allow tack on geofence for auto mode use on restricted water

 
 ----- Long term
 - add wind speed sensor
 - mavlink sailing messages 
 - sailing loiter, boat stays inside loiter circle, more like plane than rover, will carry on sailing but stay close to a point, cant slow down too muc or will lose sterring
 - motor sailing, some boats may also have motor, we need to decide at what point we would be better of just motoring in low wind, or for a tight loiter, or to hit waypoint exactly, or if stuck head to wind, or to reverse ect
 - smart decition making, ie tack on windshifts, what to do if stuck head to wind 
 - sailing 'mode' swithcing ie in light wind you may have larger vmg by pointing lower and going faster, where as it may be beter to go high and slow in heavy wind, tricky because very boat dipendent
 - some sailing codes track waves to try and 'surf' and to allow tacking on a flat bit, not sure if there is much gain to be had here
 - add some sort of pitch monitering to prevent nose diving in hevery weather
 - pitch PID for hydrofoils
 - more advanced sail control, ie twist
 - independednt sheeting for main and jib
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
    
    // + is wind over stbd side, - is wind over port side, but as the sails are sheeted the same on each side it makes no dirence so take abs
    float wind_dir_apparent = abs(windvane.get_apparent_wind_direction_rad());

    // set the main sail to the ideal angle to the wind
    float mainsail_angle = wind_dir_apparent - g2.sail_ideal_angle;
    
    // make sure between alowable range
    mainsail_angle = constrain_float(mainsail_angle, g2.sail_min_angle, g2.sail_max_angle);
    
    // linear interpolate mainsail value (0 to 100) from wind angle mainsail_angle
    float mainsail = ((mainsail_angle - g2.sail_min_angle)/(g2.sail_max_angle - g2.sail_min_angle)) * 100.0f;

    g2.motors.set_mainsail(mainsail);
}

// Velocity Made Good, this is the speed we are travling towards the desired destination 
// Not sure how usefull this is at this stage, but would be usefull to log
float Rover::Sailboat_VMG(float Target_heading)
{
    // https://en.wikipedia.org/wiki/Velocity_made_good
    
    float speed;
    g2.attitude_control.get_forward_speed(speed);
       
    float VMG = speed * cosf(wrap_PI(Target_heading - ahrs.yaw));
    
    return VMG;
}