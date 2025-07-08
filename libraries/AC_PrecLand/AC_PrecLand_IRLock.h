#pragma once

#include "AC_PrecLand_config.h"

#if AC_PRECLAND_IRLOCK_ENABLED

#include <AC_PrecLand/AC_PrecLand_Backend.h>
#include <AP_Math/AP_Math.h>
#include <AP_IRLock/AP_IRLock_config.h>

#if AP_IRLOCK_SITL_ENABLED
#include <AP_IRLock/AP_IRLock_SITL.h>
#elif AP_IRLOCK_I2C_ENABLED
#include <AP_IRLock/AP_IRLock_I2C.h>
#endif  // AP_IRLOCK_I2C_ENABLED

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
#if AP_IRLOCK_SITL_ENABLED
    AP_IRLock_SITL irlock;
#elif AP_IRLOCK_I2C_ENABLED
    AP_IRLock_I2C irlock;
#endif
};

#endif // AC_PRECLAND_IRLOCK_ENABLED
