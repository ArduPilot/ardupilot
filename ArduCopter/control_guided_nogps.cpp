#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool Copter::FlightMode_Guided_NoGPS::init(bool ignore_checks)
{
    // start in angle control mode
    Copter::FlightMode_Guided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::FlightMode_Guided_NoGPS::run()
{
    // run angle controller
    Copter::FlightMode_Guided::angle_control_run();
}
