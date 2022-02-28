#include "Rover.h"

// initialise guided_nogps controller
bool ModeGuidedNoGPS::_enter()
{
    submode_yaw_rate_and_throttle.enter(0, 0); // if this fails we're in trouble
    submode = &submode_yaw_rate_and_throttle;
    return true;
}

// update - runs the guided controller
// should be called at 100hz or more
void ModeGuidedNoGPS::update()
{
    submode->update();
}

bool ModeGuidedNoGPS::set_desired_turn_rate_and_throttle(float turn_rate_degs, float throttle)
{
    if (submode_yaw_rate_and_throttle.enter(radians(turn_rate_degs), throttle)) {
        submode = &submode_yaw_rate_and_throttle;
        return true;
    }
    return false;
}

void ModeGuidedNoGPS::YawRateAndThrottle::update()
{
    const float steering_out = rover.g2.attitude_control.get_steering_out_rate(
        desired_yaw_rate_rads,
        rover.g2.motors.limit.steer_left,
        rover.g2.motors.limit.steer_right,
        rover.G_Dt);

    rover.g2.motors.set_steering(steering_out * 4500.0f, false);
    rover.g2.motors.set_throttle(throttle * 100.0f);
}
