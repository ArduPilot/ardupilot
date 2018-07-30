/*
 * Copyright (C) 2016  Emlid Ltd. All rights reserved.
 *
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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_QMC5883L_I2C_ADDR
#define HAL_COMPASS_QMC5883L_I2C_ADDR 0x0D
#endif

/*
  setup default orientations
 */
#ifndef HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL
#define HAL_COMPASS_QMC5883L_ORIENTATION_EXTERNAL ROTATION_ROLL_180
#endif

#ifndef HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL
#define HAL_COMPASS_QMC5883L_ORIENTATION_INTERNAL ROTATION_ROLL_180_YAW_270
#endif

class AP_Compass_QMC5883L : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
									 bool force_external,
                                     enum Rotation rotation = ROTATION_NONE);

    void read() override;

    static constexpr const char *name = "QMC5883L";

private:
    AP_Compass_QMC5883L(Compass &compass,
                       AP_HAL::OwnPtr<AP_HAL::Device> dev,
					   bool force_external,
                       enum Rotation rotation);

    void _dump_registers();
    bool _check_whoami();
    void timer();
    bool init();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    Vector3f _accum = Vector3f();
    uint32_t _accum_count = 0;

    enum Rotation _rotation;
    uint8_t _instance;
    bool _force_external:1;
};
