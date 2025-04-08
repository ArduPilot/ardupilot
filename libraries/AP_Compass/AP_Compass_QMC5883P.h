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
 * Driver by Lokesh Ramina, Jan 2022
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass_config.h"

#ifdef AP_COMPASS_QMC5883P_ENABLED

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_QMC5883P_I2C_ADDR
#define HAL_COMPASS_QMC5883P_I2C_ADDR 0x2C
#endif

/*
  setup default orientations
 */
#ifndef HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL
#define HAL_COMPASS_QMC5883P_ORIENTATION_EXTERNAL ROTATION_ROLL_180
#endif

#ifndef HAL_COMPASS_QMC5883P_ORIENTATION_INTERNAL
#define HAL_COMPASS_QMC5883P_ORIENTATION_INTERNAL ROTATION_ROLL_180_YAW_270
#endif

class AP_Compass_QMC5883P : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "QMC5883P";

private:
    AP_Compass_QMC5883P(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                        bool force_external,
                        enum Rotation rotation);

    void _dump_registers();
    bool _check_whoami();
    void timer();
    bool init();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    enum Rotation _rotation;
    uint8_t _instance;
    bool _force_external:1;
};

#endif  // AP_COMPASS_QMC5883P_ENABLED
