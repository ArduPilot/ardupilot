#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{

    // convert pilot stick input into desired steering and throttle
    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    // set reverse flag backing up
    const bool reversed = is_negative(desired_throttle);
    rover.set_reverse(reversed);

    // convert pilot steering input to desired turn rate in radians/sec
    const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

    // run steering turn rate controller and throttle controller
    const float steering_out = attitude_control.get_steering_out_rate(
                                                                    target_turn_rate,
                                                                    g2.motors.limit.steer_left,
                                                                    g2.motors.limit.steer_right);

    g2.motors.set_steering(steering_out * 4500.0f);

    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed, just use the provided throttle
        g2.motors.set_throttle(desired_throttle);
    } else {

        // convert pilot throttle input to desired speed
        float target_speed = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

        calc_throttle(target_speed, false, true);
    }
}

bool ModeAcro::requires_velocity() const
{
    return g2.motors.have_skid_steering()? false: true;
}
