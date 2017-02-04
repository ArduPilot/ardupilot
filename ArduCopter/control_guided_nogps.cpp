#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool Copter::guided_nogps_init(bool ignore_checks)
{
    // start in angle control mode
    guided_angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_nogps_run()
{
    // run angle controller
    guided_angle_control_run();
}

