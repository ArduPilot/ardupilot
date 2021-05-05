#include "Blimp.h"

/*
 * Init and run calls for stabilize flight mode
 */

// manual_run - runs the main manual controller
// should be called at 100hz or more
void ModeLand::run()
{
    //stop moving

}

// set_mode_land_with_pause - sets mode to LAND and triggers 4 second delay before descent starts
//  this is always called from a failsafe so we trigger notification to pilot
void Blimp::set_mode_land_with_pause(ModeReason reason)
{
    set_mode(Mode::Number::LAND, reason);
    //TODO: Add pause

    // alert pilot to mode change
    AP_Notify::events.failsafe_mode_change = 1;
}