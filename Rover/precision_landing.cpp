//
// functions to support precision landing
//

#include "Rover.h"

#if AC_PRECLAND_ENABLED

void Rover::init_precland()
{
    // scheduler table specifies 400Hz, but we can call it no faster
    // than the scheduler loop rate:
    rover.precland.init(MIN(400, scheduler.get_loop_rate_hz()));
}

void Rover::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(0, false);
}
#endif
