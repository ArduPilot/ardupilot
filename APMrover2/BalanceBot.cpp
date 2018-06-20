#include<stdio.h>
#include "Rover.h"

// Function to set a desired pitch angle according to throttle
void Rover::balance_pitch(float &throttle, bool armed)
{
    // calculate desired pitch angle
    float demanded_pitch = radians(-(throttle/100) * g2.bal_pitch_max);

    // calculate required throttle using PID controller
    float balance_throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch, armed)*100;

    throttle = constrain_float(balance_throttle, -100, 100);
}

// returns true if vehicle is a balance bot
// called in AP_MotorsUGV::output()
// this affects whether the vehicle tries to control its pitch with throttle output
bool Rover::is_BalanceBot() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
}
