/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    copter.precland.init();
}

void Copter::update_precland()
{
    float final_alt = current_loc.alt;

    // use range finder altitude if it is valid
    if (rangefinder_enabled && (rangefinder_alt_health >= RANGEFINDER_HEALTH_MAX)) {
        final_alt = rangefinder_alt;
    }

    copter.precland.update(final_alt);

    // log output
    Log_Write_Precland();
}
#endif
