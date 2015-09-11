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
    if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        final_alt = sonar_alt;
    }

    copter.precland.update(final_alt);

    // log output
    Log_Write_Precland();
}
#endif
