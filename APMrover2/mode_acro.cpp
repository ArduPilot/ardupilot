#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{
    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    // convert pilot throttle input to desired speed (up to twice the cruise speed)
    float target_speed = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // determine if pilot is requesting pivot turn
    bool is_pivot_turning = g2.motors.have_skid_steering() && is_zero(target_speed) && (!is_zero(desired_steering));

    // convert steering request to turn rate in radians/sec
    float turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

    // set reverse flag backing up
    bool reversed = is_negative(target_speed);
    rover.set_reverse(reversed);

    // run speed to throttle output controller
    if (is_zero(target_speed) && !is_pivot_turning) {
        stop_vehicle();
    } else {
        // run steering turn rate controller and throttle controller
        float steering_out = attitude_control.get_steering_out_rate(turn_rate, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right, reversed);
        g2.motors.set_steering(steering_out * 4500.0f);
        calc_throttle(target_speed, false);
    }
}
