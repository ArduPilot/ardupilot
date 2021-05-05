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
bool ModeAvoidADSB::init(const bool ignore_checks)
{
    // re-use guided mode
    return ModeGuided::init(ignore_checks);
}

bool ModeAvoidADSB::set_velocity(const Vector3f& velocity_neu)
{
    // check flight mode
    if (copter.flightmode->mode_number() != Mode::Number::AVOID_ADSB) {
        return false;
    }

    // re-use guided mode's velocity controller
    ModeGuided::set_velocity(velocity_neu);
    return true;
}

// runs the AVOID_ADSB controller
void ModeAvoidADSB::run()
{
    // re-use guided mode's velocity controller
    // Note: this is safe from interference from GCSs and companion computer's whose guided mode
    //       position and velocity requests will be ignored while the vehicle is not in guided mode
    ModeGuided::run();
}
