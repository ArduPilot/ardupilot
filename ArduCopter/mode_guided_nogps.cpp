#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
void Copter::ModeGuidedNoGPS::enter()
{
    // start in angle control mode
    Copter::ModeGuided::angle_control_start();
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeGuidedNoGPS::run()
{
    // run angle controller
    Copter::ModeGuided::angle_control_run();
}
