#include "Sub.h"

// read_inertia - read inertia in from accelerometers
void Sub::read_inertia()
{
    // inertial altitude estimates
    sub.pos_control.update_estimates();

    // pull position from ahrs
    Location loc;
    // AHRS provides a best-guess in case of failure
    UNUSED_RESULT(ahrs.get_location(loc));
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    if (!AP::ahrs().has_status(AP_AHRS::Status::VERT_POS)) {
        return;
    }

    current_loc.alt = pos_control.get_pos_estimate_U_m() * 100.0f;

    // get velocity, altitude is always absolute frame, referenced from
    // water's surface
    climb_rate = pos_control.get_vel_estimate_U_ms() * 100.0f;
}
