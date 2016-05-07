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
    if (rangefinder_alt_ok()) {
        final_alt = rangefinder_state.alt_cm;
    }

    copter.precland.update(final_alt);

    // log output
    Log_Write_Precland();
}
#endif
