#include "Sub.h"

// read_inertia - read inertia in from accelerometers
void Sub::read_inertia()
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

    current_loc.alt = inertial_nav.get_altitude();

    // get velocity, altitude is always absolute frame, referenced from
    // water's surface
    climb_rate = inertial_nav.get_velocity_z();
}
