#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    inertial_nav.update(vibration_check.high_vibes);

    // pull position from ahrs
    Location loc;
    ahrs.get_position(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

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
