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
 *
 */
#pragma once

#include "AP_Compass_config.h"

#if AP_COMPASS_IIS2MDC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_IIS2MDC_I2C_ADDR
#define HAL_COMPASS_IIS2MDC_I2C_ADDR 0x1E
#endif

#ifndef HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL
#define HAL_COMPASS_IIS2MDC_ORIENTATION_EXTERNAL ROTATION_NONE
#endif

#ifndef HAL_COMPASS_IIS2MDC_ORIENTATION_INTERNAL
#define HAL_COMPASS_IIS2MDC_ORIENTATION_INTERNAL ROTATION_NONE
#endif

class AP_Compass_IIS2MDC : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "IIS2MDC";

private:
    AP_Compass_IIS2MDC(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                        bool force_external,
                        enum Rotation rotation);

    bool check_whoami();
    void timer();
    bool init();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    enum Rotation _rotation;
    bool _force_external;
};

#endif  // AP_COMPASS_IIS2MDC_ENABLED
