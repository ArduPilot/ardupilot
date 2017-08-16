#include "mode.h"
#include "Rover.h"

void ModeSteering::update()
{
    // convert pilot throttle input to desired speed (up to twice the cruise speed)
    float target_speed = channel_throttle->get_control_in() * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        lateral_acceleration = 0.0f;
        return;
    }

    // in steering mode we control lateral acceleration directly. We first calculate the maximum lateral
    // acceleration at full steering lock for this speed. That is V^2/R where R is the radius of turn.
    float max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);

    // constrain to user set TURN_MAX_G
    max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

    // convert pilot steering input to desired lateral acceleration
    lateral_acceleration = max_g_force * (channel_steer->get_control_in() / 4500.0f);

    // reverse target lateral acceleration if backing up
    bool reversed = false;
    if (is_negative(target_speed)) {
        reversed = true;
        lateral_acceleration = -lateral_acceleration;
    }

    // mark us as in_reverse when using a negative throttle
    rover.set_reverse(reversed);

    // run speed to throttle output controller
    if (is_zero(target_speed)) {
        stop_vehicle();
    } else {
        // run steering controller
        calc_nav_steer(reversed);
        calc_throttle(target_speed, false);
    }
}
