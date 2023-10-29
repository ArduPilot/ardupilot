#include "Blimp.h"
/*
 * Init and run calls for land flight mode
 */

// Runs the main land controller
void ModeLand::run()
{
    //Stop moving
    motors->right_out = 0;
    motors->front_out = 0;
    motors->yaw_out = 0;
    motors->down_out = 0;
}

// set_mode_land_failsafe - sets mode to LAND
// this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_land_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}