#include "Blimp.h"
/*
 * Init and run calls for manual flight mode
 */

// Runs the main manual controller
void ModeManual::run()
{
    Vector3f pilot;
    float pilot_yaw;
    get_pilot_input(pilot, pilot_yaw);
    motors->right_out = pilot.y;
    motors->front_out = pilot.x;
    motors->yaw_out = pilot_yaw;
    motors->down_out = pilot.z;
}
