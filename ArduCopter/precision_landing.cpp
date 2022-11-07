//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(rangefinder_state.alt_cm_glitch_protected,
                           rangefinder_alt_ok());
}
#endif
