#include "mode.h"
#include "Rover.h"

bool ModeHold::_enter()
{
    if (rover.is_boat()) {
        _can_loiter = rover.mode_loiter.enter();
    }

    return true;
}

void ModeHold::update()
{
    float throttle = 0.0f;

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // boat should hold position
    if (rover.is_boat() && _can_loiter) {
        rover.mode_loiter.update();
        return;
    }

    // relax mainsail
    g2.motors.set_mainsail(100.0f);

    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(0.0f);
}

void ModeHold::_exit()
{
    _can_loiter = false;
}
