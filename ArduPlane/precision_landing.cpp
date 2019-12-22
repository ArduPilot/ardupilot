#include "Plane.h"

#if PRECISION_LANDING == ENABLED
void Plane::init_precland()
{
    plane.precland.init(400);
}

void Plane::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;

#if AP_TERRAIN_AVAILABLE
    // use range finder altitude if it is valid, else try to get terrain alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.height_estimate;
    } else if (g.terrain_follow) {
        if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, height_above_ground_cm)) {
            height_above_ground_cm = current_loc.alt;
        }
    }
#endif

    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.height_estimate;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
}
#endif