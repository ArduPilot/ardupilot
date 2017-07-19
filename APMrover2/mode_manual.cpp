#include "mode.h"
#include "Rover.h"

void ModeManual::update()
{
    // mark us as in_reverse when using a negative throttle to
    // stop AHRS getting off
    rover.set_reverse(is_negative(g2.motors.get_throttle()));
}
