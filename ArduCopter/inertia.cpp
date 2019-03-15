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

    Location::AltFrame frame;
    if (ahrs.home_is_set()) {
        frame = Location::AltFrame::ABOVE_HOME;
    } else {
        // without home use alt above the EKF origin
        frame = Location::AltFrame::ABOVE_ORIGIN;
    }
    current_loc.set_alt_cm(inertial_nav.get_altitude(), frame);
    current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);

    // set flags and get velocity
    climb_rate = inertial_nav.get_velocity_z();
}
