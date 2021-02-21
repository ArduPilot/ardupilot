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
        _desired_lat_accel = 0.0f;
        return;
    }

    float desired_steering, desired_speed;
    get_pilot_desired_steering_and_speed(desired_steering, desired_speed);

    bool reversed = is_negative(desired_speed);

    // determine if pilot is requesting pivot turn
    if (g2.motors.have_skid_steering() && is_zero(desired_speed)) {
        // pivot turning using turn rate controller
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);
        _desired_lat_accel = 0.0f;

        // run steering turn rate controller and throttle controller
        const float steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                                          g2.motors.limit.steer_left,
                                                                          g2.motors.limit.steer_right,
                                                                          rover.G_Dt);
        set_steering(steering_out * 4500.0f);
    } else {
        // In steering mode we control lateral acceleration directly.
        // For regular steering vehicles we use the maximum lateral acceleration
        //  at full steering lock for this speed: V^2/R where R is the radius of turn.
        float max_g_force = speed * speed / MAX(g2.turn_radius, 0.1f);
        max_g_force = constrain_float(max_g_force, 0.1f, g.turn_max_g * GRAVITY_MSS);

        // convert pilot steering input to desired lateral acceleration
        _desired_lat_accel = max_g_force * (desired_steering / 4500.0f);

        // reverse target lateral acceleration if backing up
        if (reversed) {
            _desired_lat_accel = -_desired_lat_accel;
        }

        // run lateral acceleration to steering controller
        calc_steering_from_lateral_acceleration(_desired_lat_accel, reversed);
    }

    // run speed to throttle controller
    calc_throttle(desired_speed, true);
}
