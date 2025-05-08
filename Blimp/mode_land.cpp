#include "Blimp.h"
/*
 * Init and run calls for land flight mode
 */

bool ModeLand::init(bool ignore_checks)
{
    targ_vel = {0.0,0.0,0.5*blimp.loiter->max_vel_z};
    return true;
}

// Runs the main land controller
void ModeLand::run()
{
    if (blimp.position_ok()) {
        blimp.loiter->run_vel(targ_vel, targ_vel_yaw, Vector4b{false,false,false,false}, true);
    } else {
        //No position/velocity, so all we can do is go down slowly.
        motors->right_out = 0;
        motors->front_out = 0;
        motors->yaw_out = 0;
        motors->down_out = 0.5;
    }
}

// set_mode_land_failsafe - sets mode to LAND
// this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_land_failsafe(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}