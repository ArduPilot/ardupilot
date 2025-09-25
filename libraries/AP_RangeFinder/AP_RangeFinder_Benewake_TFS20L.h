/*
 * Copyright (C) 2024
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

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_BENEWAKE_TFS20L_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#define TFS20L_ADDR_DEFAULT              0x10        // TFS20L default device id

#include <AP_HAL/utility/sparse-endian.h>
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_Benewake_TFS20L : public AP_RangeFinder_Backend
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
    AP_RangeFinder_Benewake_TFS20L(RangeFinder::RangeFinder_State &_state,
                                   AP_RangeFinder_Params &_params,
                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    void timer();

    bool read_registers(uint8_t reg_addr, uint8_t* data, uint8_t len);
    void process_raw_measure(uint16_t distance_raw, uint16_t strength_raw,
                             uint16_t &output_distance_cm);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    struct {
        uint32_t sum;
        uint32_t count;
    } accum;

    // TFS20L register addresses
    static constexpr uint8_t TFS20L_DIST_LOW = 0x00;         // Distance low byte
    static constexpr uint8_t TFS20L_DIST_HIGH = 0x01;        // Distance high byte  
    static constexpr uint8_t TFS20L_AMP_LOW = 0x02;          // Signal strength/amplitude low byte
    static constexpr uint8_t TFS20L_AMP_HIGH = 0x03;         // Signal strength/amplitude high byte
    static constexpr uint8_t TFS20L_TEMP_LOW = 0x04;         // Temperature low byte
    static constexpr uint8_t TFS20L_TEMP_HIGH = 0x05;        // Temperature high byte
    static constexpr uint8_t TFS20L_VERSION_REVISION = 0x0A; // Version revision byte
    static constexpr uint8_t TFS20L_VERSION_MINOR = 0x0B;    // Version minor byte
    static constexpr uint8_t TFS20L_VERSION_MAJOR = 0x0C;    // Version major byte
    static constexpr uint8_t TFS20L_ENABLE = 0x25;           // Enable register
    
    // Distance and strength limits
    static constexpr uint16_t MAX_DIST_CM = 2000;
    static constexpr uint16_t MIN_DIST_CM = 1;
    static constexpr uint16_t MIN_STRENGTH = 100;
};

#endif
