//
// functions to support precision landing
//

#include "Plane.h"

#if PRECISION_LANDING == ENABLED
void Plane::init_precland()
{
    plane.g2.precland.init(400); // this must match the number on the scheduler table
}

void Plane::update_precland()
{
    if (plane.g.rangefinder_landing &&
        rangefinder_state.in_range) {
        return g2.precland.update(rangefinder_state.height_estimate,
                                  true);
    }
    return g2.precland.update(0, false);
}
#endif
