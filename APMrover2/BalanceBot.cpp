#include<stdio.h>
#include "Rover.h"


void Rover::balance_pitch(float &throttle, bool armed)
{
    float balance_throttle = 0;
    float throttle_new = 0;
    float demanded_pitch = 0;
    balance_throttle = g2.attitude_control.get_throttle_out_from_pitch(demanded_pitch, armed)*100;
    throttle_new = constrain_float(throttle + balance_throttle, -100, 100);
    throttle = throttle_new;
}

// returns true if vehicle is a balance bot
// called in AP_MotorsUGV::output()
// this affects whether the vehicle tries to control its pitch with throttle output
bool Rover::is_BalanceBot() const
{
    return ((enum frame_class)g2.frame_class.get() == FRAME_BALANCEBOT);
}
