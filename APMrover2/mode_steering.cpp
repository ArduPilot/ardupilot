#include "mode.h"
#include "Rover.h"

void ModeSteering::update()
{
    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);

    // determine if pilot is requesting pivot turn
    bool is_pivot_turning = g2.motors.have_skid_steering() && is_zero(desired_speed) && (!is_zero(desired_steering));

    // In steering mode we control lateral acceleration directly.
    // For pivot steering vehicles we use the TURN_MAX_G parameter
    // For regular steering vehicles we use the maximum lateral acceleration at full steering lock for this speed: V^2/R where R is the radius of turn.
    float max_g_force;
    if (is_pivot_turning) {
        max_g_force = g.turn_max_g * GRAVITY_MSS;
    } else {
        max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);
    }

    // constrain to user set TURN_MAX_G
    max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

    // convert pilot steering input to desired lateral acceleration
    float desired_lat_accel = max_g_force * (desired_steering / 4500.0f);

    // reverse target lateral acceleration if backing up
    bool reversed = false;
    if (is_negative(desired_speed)) {
        reversed = true;
        desired_lat_accel = -desired_lat_accel;
    }

    // mark us as in_reverse when using a negative throttle
    rover.set_reverse(reversed);

    // run lateral acceleration to steering controller
    calc_steering_from_lateral_acceleration(desired_lat_accel, reversed);

    // run speed to throttle controller
    calc_throttle(desired_speed, false, true);
}
