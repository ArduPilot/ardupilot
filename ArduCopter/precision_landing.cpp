//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    int32_t height_above_ground_cm = current_loc.alt;
    // use range finder altitude if it is valid, otherwise use home alt
    if (rangefinder_alt_ok()) {
        height_above_ground_cm = rangefinder_state.alt_cm_glitch_protected;
    }

    precland.update(height_above_ground_cm, rangefinder_alt_ok());
    g2.precland_sm->set_alt_above_ground_cm(height_above_ground_cm);
    if (battery.has_failsafed()) {
        g2.precland_sm->set_emergency_flag(true);
    }
    g2.precland_sm->run();
}
#endif
