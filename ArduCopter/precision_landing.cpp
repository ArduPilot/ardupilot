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
    // alt will be unused if we pass false through as the second parameter: you can change backend type by sending 1 or 2 as per condition
    return precland.update(rangefinder_state.alt_cm_glitch_protected,
                           rangefinder_alt_ok(), precland_source);
}
#endif
