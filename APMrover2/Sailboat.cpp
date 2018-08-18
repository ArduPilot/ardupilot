#include "Rover.h"

#define SAILBOAT_WIND_ANGLE_MIN     radians(20) // sailboats can sail as close as 20degrees from wind
#define SAILBOAT_MAINSAIL_ANGLE_MAX radians(90) // sailboats mainsail can be up to 90degrees

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

    // get wind direction from wind vane (direction wind is blowing towards)
    // temporarily take wind direction from channel6
    //const float wind_dir_abs = rc().channel(CH_6)->norm_input() * radians(360);

    // convert wind direction to relative angle (in radians)
    //const float wind_dir_rel = fabsf(wrap_PI(ahrs.yaw - wind_dir_abs));
    
    const float wind_dir_rel = ap_windvane->get_apparent_wind_direction_rad();

    // linear interpolate mainsail value (0 to 100) from wind angle (20deg ~ 180deg)
    const float mainsail = MAX(0.0f, wind_dir_rel - SAILBOAT_WIND_ANGLE_MIN) / (radians(180) - SAILBOAT_WIND_ANGLE_MIN) * 100.0f;

    g2.motors.set_mainsail(mainsail);
}

