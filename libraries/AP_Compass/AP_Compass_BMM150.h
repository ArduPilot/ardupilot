/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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

class AP_Compass_BMM150 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(Compass &compass,
                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void read() override;

    static constexpr const char *name = "BMM150";

private:
    AP_Compass_BMM150(Compass &compass, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /**
     * Device periodic callback to read data from the sensor.
     */
    bool init();
    void _update();
    bool _load_trim_values();
    int16_t _compensate_xy(int16_t xy, uint32_t rhall, int32_t txy1, int32_t txy2);
    int16_t _compensate_z(int16_t z, uint32_t rhall);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    Vector3f _mag_accum;
    uint32_t _accum_count;

    uint8_t _compass_instance;

    struct {
        int8_t x1;
        int8_t y1;
        int8_t x2;
        int8_t y2;
        uint16_t z1;
        int16_t z2;
        int16_t z3;
        int16_t z4;
        uint8_t xy1;
        int8_t xy2;
        uint16_t xyz1;
    } _dig;

    uint32_t _last_read_ms;
    AP_HAL::Util::perf_counter_t _perf_err;
};
