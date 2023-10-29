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

#include "AP_Compass_config.h"

#if AP_COMPASS_IST8310_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass_Backend.h"

#ifndef HAL_COMPASS_IST8310_I2C_ADDR
#define HAL_COMPASS_IST8310_I2C_ADDR 0x0E
#endif

#ifndef AP_COMPASS_IST8310_DEFAULT_ROTATION
#define AP_COMPASS_IST8310_DEFAULT_ROTATION ROTATION_PITCH_180
#endif

class AP_Compass_IST8310 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                     bool force_external,
                                     enum Rotation rotation);

    void read() override;

    static constexpr const char *name = "IST8310";

private:
    AP_Compass_IST8310(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                       bool force_external,
                       enum Rotation rotation);

    void timer();
    bool init();
    void start_conversion();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    AP_HAL::Device::PeriodicHandle _periodic_handle;

    enum Rotation _rotation;
    uint8_t _instance;
    bool _ignore_next_sample;
    bool _force_external;
};

#endif  // AP_COMPASS_IST8310_ENABLED
