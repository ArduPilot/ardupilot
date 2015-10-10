/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// functions to support precision landing
//

#include "Copter.h"

#if PRECISION_LANDING == ENABLED

void Copter::init_precland()
{
    precland.init();
}

void Copter::update_precland()
{
    precland.update(current_loc.alt);

    // log output
    Log_Write_Precland();
}

#endif
