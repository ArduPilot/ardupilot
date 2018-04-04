#include "Tracker.h"

// Functions to read the RC radio input

void Tracker::read_radio()
{
    if (RC_Channels::has_new_input()) {
        RC_Channels::set_pwm_all();
    }
}
