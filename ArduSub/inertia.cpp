#include "Sub.h"

// read_inertia - read inertia in from accelerometers
void Sub::read_inertia()
{
    // inertial altitude estimates
    inertial_nav.update(G_Dt);

    // pull position from interial nav library
    if (!ahrs.get_origin(ekf_origin)) {
        return;
    }
    Location loc{};
    if (!ahrs.get_position(loc)) {
        return;
    }
    current_loc.lng = loc.lng;
    current_loc.lat = loc.lat;

    // Get XYZ position and velocity in NEU and cm
    if (!ahrs.get_relative_position_NED_origin(current_pos)) {
        return;
    }
    current_pos = current_pos * 100.0f;  // m to cm
    current_pos.z = -current_pos.z;  // NED to NEU
    if (!ahrs.get_velocity_NED(current_vel)) {
        return;
    }
    current_vel = current_vel * 100.0f;  // m to cm
    current_vel.z = -current_vel.z;  // NED to NEU

    // exit immediately if we do not have an altitude estimate
    nav_filter_status filt_status;
    if (!ahrs.get_filter_status(filt_status)) {
        return;
    }
    if (!filt_status.flags.vert_pos) {
        return;
    }

    current_loc.alt = current_pos.z;

    // get velocity, altitude is always absolute frame, referenced from
    // water's surface
    climb_rate = current_vel.z;
}
