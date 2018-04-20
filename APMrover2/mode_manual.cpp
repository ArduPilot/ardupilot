#include "mode.h"
#include "Rover.h"

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_yaw;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle, desired_yaw);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering);
    g2.motors.set_yaw(desired_yaw);


    // mark us as in_reverse when using a negative throttle to stop AHRS getting off
    //rover.set_reverse(is_negative(g2.motors.get_throttle()));

}
