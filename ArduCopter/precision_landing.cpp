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
    if (!rangefinder_alt_ok() ||
        rangefinder_state.alt_cm_glitch_protected == 0) {
        return precland.update(0, false);
    }
    return precland.update(rangefinder_state.alt_cm_glitch_protected,
                           true);
}
#endif
