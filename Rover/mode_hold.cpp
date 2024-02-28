#include "Rover.h"

void ModeHold::update()
{
    float throttle = 0.0f;

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(throttle);
    }

    // relax mainsail
    g2.sailboat.relax_sails();

    // hold position - stop motors and center steering
    g2.motors.set_throttle(throttle);
    g2.motors.set_steering(0.0f);
}
