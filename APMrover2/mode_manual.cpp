#include "mode.h"
#include "Rover.h"

void ModeManual::update()
{
    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(channel_throttle->get_control_in());
    g2.motors.set_steering(channel_steer->get_control_in());

    // mark us as in_reverse when using a negative throttle to stop AHRS getting off
    rover.set_reverse(is_negative(g2.motors.get_throttle()));
}
