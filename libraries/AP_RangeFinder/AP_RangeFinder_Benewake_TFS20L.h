/*
 * Copyright (C) 2025 Shubham Jolapara <shubhamjolapara256@gmail.com>
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
 * 
 * 
 * 
 * Benewake TFS20L I2C Rangefinder Driver
 * 
 * Range: 0.01m to 20m
 * Datasheet: https://altronics.cl/uploads/benewake/TFS20-L-User-Manual.pdf
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFS20L_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#define TFS20L_ADDR_DEFAULT              0x10        // TFS20L default device id

// Forward declaration
namespace AP_HAL {
    class I2CDevice;
}

class AP_RangeFinder_Benewake_TFS20L : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice *dev);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    AP_RangeFinder_Benewake_TFS20L(RangeFinder::RangeFinder_State &_state,
                                   AP_RangeFinder_Params &_params,
                                   AP_HAL::I2CDevice &dev);

    bool init();
    void timer();

    AP_HAL::I2CDevice &_dev;

    struct {
        uint32_t sum;
        uint32_t count;
        uint32_t out_of_range_count;
    } accum;

};

#endif
