#include "mode.h"
#include "Rover.h"

void ModeSteering::update() {
    /*
        in steering mode we control lateral acceleration
        directly. We first calculate the maximum lateral
        acceleration at full steering lock for this speed. That is
        V^2/R where R is the radius of turn. We get the radius of
        turn from half the STEER2SRV_P.
    */
    const float ground_speed = rover.ground_speed;
    float max_g_force = ground_speed * ground_speed / rover.steerController.get_turn_radius();

    // constrain to user set TURN_MAX_G
    max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

    lateral_acceleration = max_g_force * (channel_steer->get_control_in() / 4500.0f);
    calc_nav_steer();

    // and throttle gives speed in proportion to cruise speed, up
    // to 50% throttle, then uses nudging above that.
    float target_speed = channel_throttle->get_control_in() * 0.01f * 2.0f * g.speed_cruise;
    rover.set_reverse(is_negative(target_speed));
    if (rover.in_reverse) {
        target_speed = constrain_float(target_speed, -g.speed_cruise, 0.0f);
    } else {
        target_speed = constrain_float(target_speed, 0.0f, g.speed_cruise);
    }
    calc_throttle(target_speed);
}
