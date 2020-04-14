#include "Copter.h"
#include "mode.h"

bool ModePayloadRelease::init(bool ignore_checks)
{
    gcs().send_text(MAV_SEVERITY_INFO, "Initialise payload release");

    if (!ignore_checks) {   //if not ignore checks
        return true;
    }
    //if ignore checks
    return true;
}

void ModePayloadRelease::run()
{
    gcs().send_text(MAV_SEVERITY_INFO, "payload release run");
}