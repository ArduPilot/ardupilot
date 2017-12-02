#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // reset ahrs flags
    ahrs_state.has_ekf_origin = false;
    ahrs_state.has_current_loc = false;
    ahrs_state.has_relative_pos = false;
    ahrs_state.has_current_vel = false;

    // pull position
    ahrs_state.has_ekf_origin = ahrs.get_origin(ekf_origin);

    Location loc{};
    if (ahrs.get_position(loc)) {
        ahrs_state.has_current_loc = true;
        current_loc.lng = loc.lng;
        current_loc.lat = loc.lat;
    }

    // Get XYZ position and velocity in NEU and cm
    if (ahrs.get_relative_position_NED_origin(current_pos)) {
        ahrs_state.has_relative_pos = true;
        current_pos = current_pos * 100.0f;  // m to cm
        current_pos.z = -current_pos.z;  // NED to NEU
    }

    if (ahrs.get_velocity_NED(current_vel)) {
        ahrs_state.has_current_vel = true;
        current_vel = current_vel * 100.0f;  // m to cm
        current_vel.z = -current_vel.z;  // NED to NEU
    }

    ahrs.get_filter_status(filt_status);
    ahrs_state.has_filt_status = filt_status.flags.vert_pos;

    if (ahrs_state.has_relative_pos ) {
        if (ap.home_state == HOME_UNSET) {
            current_loc.set_alt_cm(current_pos.z, Location_Class::ALT_FRAME_ABOVE_ORIGIN);
        } else {
            current_loc.set_alt_cm(pv_alt_above_home(current_pos.z), Location_Class::ALT_FRAME_ABOVE_HOME);
        }
    }
    if (ahrs_state.has_current_vel) {
        climb_rate = current_vel.z;
    }

}
