#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position from interial nav library
    current_loc.lng = inertial_nav.get_longitude();
    current_loc.lat = inertial_nav.get_latitude();

    // exit immediately if we do not have an altitude estimate
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    if (ahrs.home_is_set()) {
        current_loc.set_alt_cm(inertial_nav.get_altitude(),
                               Location::AltFrame::ABOVE_HOME);
    } else {
        // without home use alt above the EKF origin
        current_loc.set_alt_cm(inertial_nav.get_altitude(),
                               Location::AltFrame::ABOVE_ORIGIN);
    }
}
