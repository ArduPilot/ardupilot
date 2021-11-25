//
// functions to support precision landing
//

#include "Plane.h"

#if PRECISION_LANDING == ENABLED

void Plane::init_precland()
{
    plane.g2.precland.init(scheduler.get_loop_rate_hz());  // this must be the same number as in the scheduler table
}

void Plane::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;
    bool rangefinder_alt_ok = rangefinder_state.in_range;

    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok) {
        height_above_ground_cm = rangefinder_state.height_estimate;
    }

    g2.precland.update(height_above_ground_cm, rangefinder_alt_ok);
}
#endif
