#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_IRLOCK_ENABLED

#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #include <AP_IRLock/AP_IRLock_SITL.h>
#else
 #include <AP_IRLock/AP_IRLock.h>
#endif

/*
 * AC_PrecLand_IRLock - implements precision landing using target vectors provided
 *                         by an IRLock
 */

class AC_PrecLand_IRLock : public AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_IRLock(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state);

    // perform any required initialisation of backend
    void init() override;

    // retrieve updates from sensor
    void update() override;

private:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    AP_IRLock_SITL irlock;
#else
    AP_IRLock_I2C irlock;
#endif
};

#endif // AC_PRECLAND_IRLOCK_ENABLED
