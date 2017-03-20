#include "Copter.h"

/*
 * control_avoid.cpp - init and run calls for AP_Avoidance's AVOID flight mode
 *
 * This re-uses GUIDED mode functions but does not interfere with the GCS or companion computer's
 * use of guided mode because the velocity requests arrive from different sources (i.e MAVLink messages
 * for GCS and Companion Computers vs the AP_Avoidance_Copter class for adsb avoidance) and inputs from
 * each source are only accepted and processed in the appropriate flight mode.
 */

// initialise avoid_adsb controller
bool Copter::avoid_adsb_init(const bool ignore_checks)
{
    // re-use guided mode
    return guided_init(ignore_checks);
}

bool Copter::avoid_adsb_set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (control_mode != AVOID_ADSB) {
        return false;
    }

    // re-use guided mode's velocity controller
    guided_set_velocity(velocity_neu);
    return true;
}

// runs the AVOID_ADSB controller
void Copter::avoid_adsb_run()
{
    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    guided_run();
}
