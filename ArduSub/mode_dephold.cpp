#include "Sub.h"


bool ModeDephold::init(bool ignore_checks) {




    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeDephold::run()
{
    gcs().send_text(MAV_SEVERITY_CRITICAL, "DEPTH_HOLD mode active. Holding depth at %.f cm", (float)g2.rov_depth_cm);
}

