/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_GP2Y0E03 : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    AP_RangeFinder_GP2Y0E03(RangeFinder::RangeFinder_State &_state,
                                    AP_RangeFinder_Params &_params,
                                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    void timer();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    struct {
        uint32_t sum;
        uint32_t count;
    } accum;
};