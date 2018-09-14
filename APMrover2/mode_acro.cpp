#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{
    // get speed forward
    float speed, desired_steering;
    if (!attitude_control.get_forward_speed(speed)) {
        float desired_throttle;
        // convert pilot stick input into desired steering and throttle
        get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
    } else {
        float desired_speed;
        // convert pilot stick input into desired steering and speed
        get_pilot_desired_steering_and_speed(desired_steering, desired_speed);
        calc_throttle(desired_speed, false, true);
    }

    float steering_out;

    // handle sailboats
    if (!is_zero(desired_steering)) {
        // steering input return control to user
        rover.sailboat_clear_tack();
    }
    if (g2.motors.has_sail() && rover.sailboat_tacking()) {
        // call heading controller during tacking
        steering_out = attitude_control.get_steering_out_heading(rover.sailboat_get_tack_heading_rad(),
                                                                 g2.pivot_turn_rate,
                                                                 g2.motors.limit.steer_left,
                                                                 g2.motors.limit.steer_right,
                                                                 rover.G_Dt);
    } else {
        // convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

        // run steering turn rate controller and throttle controller
        steering_out = attitude_control.get_steering_out_rate(target_turn_rate,
                                                              g2.motors.limit.steer_left,
                                                              g2.motors.limit.steer_right,
                                                              rover.G_Dt);
    }

    g2.motors.set_steering(steering_out * 4500.0f);
}

bool ModeAcro::requires_velocity() const
{
    return g2.motors.have_skid_steering()? false: true;
}

// sailboats in acro mode support user manually initiating tacking from transmitter
void ModeAcro::handle_tack_request()
{
    rover.sailboat_handle_tack_request_acro();
}
