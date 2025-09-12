#include "Blimp.h"
/*
 * Init and run calls for hold flight mode
 */

// Runs the main hold controller
void ModeHold::run()
{
    //Stop moving
    motors->right_out = 0;
    motors->front_out = 0;
    motors->yaw_out = 0;
    motors->down_out = 0;
}
