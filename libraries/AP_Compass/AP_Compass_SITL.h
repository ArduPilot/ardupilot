#pragma once

#include "AP_Compass.h"

#if AP_COMPASS_SITL_ENABLED

#include "AP_Compass_Backend.h"

#include <AP_Math/AP_Math.h>
#include <AP_Declination/AP_Declination.h>
#include <SITL/SITL.h>
#include <SITL/SIM_Compass.h>

class AP_Compass_SITL : public AP_Compass_Backend {
public:
    AP_Compass_SITL(uint8_t sitl_instance);

private:
    SITL::SIM *_sitl;

    void _timer();

    uint8_t sitl_instance;  // offset into SITL state structure arrays

    // the simulation of what this compass reports:
    SITL::CompassSim _compass_sim{sitl_instance};
};
#endif // AP_COMPASS_SITL_ENABLED
