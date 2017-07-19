#include "mode.h"
#include "Rover.h"

void ModeHold::update()
{
    // hold position - stop motors and center steering
    g2.motors.set_throttle(0.0f);
    g2.motors.set_steering(0.0f);
    if (!rover.in_auto_reverse) {
        rover.set_reverse(false);
    }
}
