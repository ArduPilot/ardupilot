#include "Sub.h"

// read_inertia - read inertia in from accelerometers
void Sub::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update();

    // pull position from ahrs
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    if (!inertial_nav.get_filter_status().flags.vert_pos) {
        return;
    }

    current_loc.alt = inertial_nav.get_position_z_up_cm();

    // get velocity, altitude is always absolute frame, referenced from
    // water's surface
    climb_rate = inertial_nav.get_velocity_z_up_cms();
}
