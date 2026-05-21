/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_LIS2MDL_ENABLED

#include "AP_Compass_IIS2MDC.h"

#ifndef HAL_COMPASS_LIS2MDL_I2C_ADDR
#define HAL_COMPASS_LIS2MDL_I2C_ADDR HAL_COMPASS_IIS2MDC_I2C_ADDR
#endif

// LIS2MDL is the consumer-grade variant of the IIS2MDC: same ST silicon,
// identical WHO_AM_I, register map, I2C address and sensitivity. Inherit
// the IIS2MDC implementation and only override device_type() so boards
// declared as LIS2MDL keep their DEVTYPE_LIS2MDL device IDs.
class AP_Compass_LIS2MDL : public AP_Compass_IIS2MDC
{
public:
    using AP_Compass_IIS2MDC::AP_Compass_IIS2MDC;

    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

protected:
    DevTypes device_type() const override { return DEVTYPE_LIS2MDL; }
};

#endif  // AP_COMPASS_LIS2MDL_ENABLED
