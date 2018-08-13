#include "Rover.h"

#define SAILBOAT_WIND_ANGLE_MIN     radians(20) // sailboats can sail as close as 20degrees from wind
#define SAILBOAT_MAINSAIL_ANGLE_MAX radians(90) // sailboats mainsail can be up to 90degrees

// update mainsail's desired angle based on wind speed and direction
void Rover::sailboat_update_mainsail(bool move_forward)
{
    if (!g2.motors.has_sail()) {
        return;
    }

    // if not attempting to move forward, relax sail
    // To-Do: pull in sail to make it point into the wind?
    if (!move_forward) {
        g2.motors.set_mainsail(SAILBOAT_MAINSAIL_ANGLE_MAX);
        return;
    }

    // get wind direction from wind vane (direction wind is blowing towards)
    // temporarily take wind direction from channel6
    float wind_dir_abs = rc().channel(CH_6)->norm_input() * radians(360);

    // convert wind direction to relative angle (in radians)
    float wind_dir_rel = fabsf(wrap_PI(ahrs.yaw - wind_dir_abs));

    // linear interpolate mainsail angle from wind angle (20deg ~ 180deg)
    // to mainsail range (0deg ~ 90deg)
    float mainsail_angle_rad = 0.0f;
    if (wind_dir_rel > SAILBOAT_WIND_ANGLE_MIN) {
        mainsail_angle_rad = (wind_dir_rel - SAILBOAT_WIND_ANGLE_MIN) * (radians(90) / (radians(180)-SAILBOAT_WIND_ANGLE_MIN));
    }

    g2.motors.set_mainsail(mainsail_angle_rad);
}

