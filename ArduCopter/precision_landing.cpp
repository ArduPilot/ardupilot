//
// functions to support precision landing
//

#include "Copter.h"

#if AC_PRECLAND_ENABLED

void Copter::init_precland()
{
    // scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate:
    copter.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
}

void Copter::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(rangefinder_state.alt_glitch_protected_m * 100.0, rangefinder_alt_ok());
}
#endif
