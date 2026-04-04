#include "Copter.h"

// read_inertia - read inertia in from accelerometers
void Copter::read_inertia()
{
    // inertial altitude estimates. Use barometer climb rate during high vibrations
    pos_control->update_estimates(vibration_check.high_vibes);
#if MODE_FOLLOW_ENABLED
    g2.follow.update_estimates();
#endif

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
    const float alt_above_origin_m = -pos_d_m;
    current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_ORIGIN);
    if (!ahrs.home_is_set() || !current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME)) {
        // if home has not been set yet we treat alt-above-origin as alt-above-home
        current_loc.set_alt_m(alt_above_origin_m, Location::AltFrame::ABOVE_HOME);
    }
}
