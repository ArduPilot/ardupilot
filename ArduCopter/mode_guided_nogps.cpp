#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool Copter::ModeGuidedNoGPS::init(bool ignore_checks)
{
    // start in angle control mode
    Copter::ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeGuidedNoGPS::run()
{
    // initialize smoothing gain
    attitude_control->set_smoothing_gain(get_smoothing_gain());

    // run angle controller
    Copter::ModeGuided::angle_control_run();
}
