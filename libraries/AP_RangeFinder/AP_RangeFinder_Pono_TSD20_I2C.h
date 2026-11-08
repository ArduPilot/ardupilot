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
 *
 * PONO TSD20 single-point LiDAR, I2C mode
 *
 * Range: 0.05m to 20m (90% reflective target), 0.05m to 10m (10% target)
 */
#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_PONO_TSD20_I2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"

#define TSD20_ADDR_DEFAULT              0x52        // TSD20 default 7-bit device address

// Forward declaration
namespace AP_HAL {
    class I2CDevice;
}

class AP_RangeFinder_Pono_TSD20_I2C : public AP_RangeFinder_Backend_I2C
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::I2CDevice &dev) {
        // this will free the object if configuration fails:
        return configure(NEW_NOTHROW AP_RangeFinder_Pono_TSD20_I2C(_state, _params, dev));
    }

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    using AP_RangeFinder_Backend_I2C::AP_RangeFinder_Backend_I2C;

    bool init() override;
    void timer();

    struct {
        uint32_t sum_mm;
        uint32_t count;
        uint32_t out_of_range_count;
    } accum;

};

#endif  // AP_RANGEFINDER_PONO_TSD20_I2C_ENABLED
