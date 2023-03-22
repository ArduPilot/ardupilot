//
// functions to support precision landing
//

#include "Rover.h"

#if AC_PRECLAND_ENABLED

void Rover::init_precland()
{
    rover.precland.init(400);
}

void Rover::update_precland()
{
    // alt will be unused if we pass false through as the second parameter:
    return precland.update(0, false);
}
#endif
