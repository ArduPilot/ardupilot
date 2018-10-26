#include "Copter.h"

/*
 * Init and run calls for Initialise flight mode
 */

bool Copter::ModeInitialising::init(bool ignore_checks)
{
    // Return True initially
    return true;
}

void Copter::ModeInitialising::run()
{
    // Never used
    return;
}