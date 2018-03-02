#include "mode.h"
#include "Rover.h"

void ModeAcro::update()
{
    // get speed forward
    float speed;
    if (!attitude_control.get_forward_speed(speed)) {
        // no valid speed so stop
        g2.motors.set_throttle(0.0f);
        g2.motors.set_steering(0.0f);
        return;
    }

    // convert pilot stick input into desired steering and throttle
    float desired_steering, desired_throttle;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);

    // convert pilot throttle input to desired speed (up to twice the cruise speed)
    float target_speed = desired_throttle * 0.01f * calc_speed_max(g.speed_cruise, g.throttle_cruise * 0.01f);

    // convert pilot steering input to desired turn rate in radians/sec
    const float target_turn_rate = (desired_steering / 4500.0f) * radians(g2.acro_turn_rate);

    // determine if pilot is requesting pivot turn
    const bool is_pivot_turning = g2.motors.have_skid_steering() && is_zero(target_speed) && (!is_zero(desired_steering));

    // stop vehicle if target speed is zero and not pivot turning
    if (is_zero(target_speed) && !is_pivot_turning) {
        stop_vehicle();
        return;
    }

    // set reverse flag backing up
    const bool reversed = is_negative(target_speed);
    rover.set_reverse(reversed);

    // apply object avoidance to desired speed using half vehicle's maximum acceleration/deceleration
    rover.g2.avoid.adjust_speed(0.0f, 0.5f * attitude_control.get_accel_max(), ahrs.yaw, target_speed, rover.G_Dt);

    // run steering turn rate controller and throttle controller
    const float steering_out = attitude_control.get_steering_out_rate(target_turn_rate, g2.motors.have_skid_steering(), g2.motors.limit.steer_left, g2.motors.limit.steer_right, reversed);
    g2.motors.set_steering(steering_out * 4500.0f);
    calc_throttle(target_speed, false);
}
