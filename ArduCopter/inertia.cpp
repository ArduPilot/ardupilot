#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // pull position
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

    // without home return alt above the EKF origin
    if (ap.home_state == HOME_UNSET) {
        // with inertial nav we can update the altitude and climb rate at 50hz
        // get the D position relative to the local earth frame origin
        float posD;
        if (ahrs.get_relative_position_D_origin(posD)) {
            current_loc.alt = -posD * 100;  // convert from m in NED to cm in NEU
        }
    } else {
        // with inertial nav we can update the altitude and climb rate at 50hz
        current_loc.alt = pv_alt_above_home(current_pos.z);
    }

    // set flags and get velocity
    current_loc.flags.relative_alt = true;
    climb_rate = current_vel.z;
}
