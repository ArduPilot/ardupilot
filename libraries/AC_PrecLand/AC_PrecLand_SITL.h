#pragma once

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_Math/AP_Math.h>
#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <SITL/SITL.h>

/*
 * AC_PrecLand_SITL - supplies vectors to a fake landing target
 */

class AC_PrecLand_SITL : public AC_PrecLand_Backend
{
public:

    // Constructor
    using AC_PrecLand_Backend::AC_PrecLand_Backend;

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

private:
    SITL::SITL          *_sitl;                 // sitl instance pointer

};

#endif
