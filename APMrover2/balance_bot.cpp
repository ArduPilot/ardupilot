#include<stdio.h>
#include "Rover.h"

// Function to set a desired pitch angle according to throttle
void Rover::balancebot_pitch_control(float &throttle, bool armed)
{
    // calculate desired pitch angle
    const float demanded_pitch = radians(-throttle * 0.01f * g2.bal_pitch_max);

    // calculate required throttle using PID controller
    const float balance_throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch, armed, G_Dt) * 100.0f;

    // constrain throttle between -100 and 100
    throttle = constrain_float(balance_throttle, -100.0f, 100.0f);
}

// returns true if vehicle is a balance bot
// called in AP_MotorsUGV::output()
// this affects whether the vehicle tries to control its pitch with throttle output
bool Rover::is_balancebot() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
}
