#include "Blimp.h"
/*
 * Init and run calls for rtl flight mode
 */

//Number of seconds of movement that the target position can be ahead of actual position.
#define POS_LAG 1

bool ModeRTL::init(bool ignore_checks)
{
    return true;
}

//Runs the main rtl controller
void ModeRTL::run()
{
    Vector3f target_pos = {0,0,0};
    float target_yaw = 0;
    blimp.loiter->run(target_pos, target_yaw, Vector4b{false,false,false,false});
}
