#include "mode.h"
#include "Rover.h"

void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
}

void ModeManual::update()
{
    float desired_steering, desired_throttle, desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }

    // walking robots support roll, pitch and walking_height
    float desired_roll, desired_pitch, desired_walking_height;
    get_pilot_desired_roll_and_pitch(desired_roll, desired_pitch);
    get_pilot_desired_walking_height(desired_walking_height);
    g2.motors.set_roll(desired_roll);
    g2.motors.set_pitch(desired_pitch);
    g2.motors.set_walking_height(desired_walking_height);

    // set sailboat sails
    float desired_mainsail;
    float desired_wingsail;
    float desired_mast_rotation;
    g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail, desired_mast_rotation);
    g2.motors.set_mainsail(desired_mainsail);
    g2.motors.set_wingsail(desired_wingsail);
    g2.motors.set_mast_rotation(desired_wingsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, (g2.manual_options & ManualOptions::SPEED_SCALING));
    g2.motors.set_lateral(desired_lateral);
}
