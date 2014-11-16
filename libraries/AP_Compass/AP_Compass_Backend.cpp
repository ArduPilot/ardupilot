/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL.h>
#include "Compass.h"
#include "AP_Compass_Backend.h"

AP_Compass_Backend::AP_Compass_Backend(Compass &compass) :
    _compass(compass),
    product_id(AP_PRODUCT_ID_NONE)
{}