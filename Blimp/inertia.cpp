#include "Blimp.h"

// read_inertia - read inertia in from accelerometers
void Blimp::read_inertia()
{
    // pull position from ahrs
    Location loc;
    ahrs.get_location(loc);
    current_loc.lat = loc.lat;
    current_loc.lng = loc.lng;

    // exit immediately if we do not have an altitude estimate
    float pos_d_m;
    if (!AP::ahrs().get_relative_position_D_origin_float(pos_d_m)) {
        return;
    }

    // current_loc.alt is alt-above-home, converted from AHRS's alt-above-ekf-origin
    const int32_t alt_above_origin_cm = -pos_d_m * 100.0;
    current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_cm(alt_above_origin_cm, Location::AltFrame::ABOVE_HOME);
    }
}
