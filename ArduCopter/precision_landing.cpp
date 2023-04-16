//
// functions to support precision landing
//

#include "Copter.h"

#if AC_PRECLAND_ENABLED

void Copter::init_precland()
{
    copter.precland.init(400);
}

void Copter::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(rangefinder_state.alt_cm_glitch_protected,
                           rangefinder_alt_ok());
}
#endif
