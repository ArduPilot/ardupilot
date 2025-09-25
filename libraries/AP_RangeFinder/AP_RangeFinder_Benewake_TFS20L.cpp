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
#include "AP_RangeFinder_Benewake_TFS20L.h"

#if AP_RANGEFINDER_BENEWAKE_TFS20L_ENABLED

#include <utility>

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define DRIVER "TFS20L"
#define BENEWAKE_OUT_OF_RANGE_ADD_CM 100

AP_RangeFinder_Benewake_TFS20L::AP_RangeFinder_Benewake_TFS20L(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(std::move(dev))
{
}

AP_RangeFinder_Backend *AP_RangeFinder_Benewake_TFS20L::detect(
        RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params,
        AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_RangeFinder_Benewake_TFS20L *sensor
        = NEW_NOTHROW AP_RangeFinder_Benewake_TFS20L(_state, _params, std::move(dev));

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_RangeFinder_Benewake_TFS20L::init()
{
    uint8_t version_data[3];

    _dev->get_semaphore()->take_blocking();
    _dev->set_retries(3);

    

    // Read firmware version to detect if sensor is present
    if (!read_registers(TFS20L_VERSION_MAJOR, version_data, 3)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TFS20L: No response from sensor");
        goto fail;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TFS20L: found fw version %u.%u.%u",
                  version_data[0], version_data[1], version_data[2]);

    _dev->get_semaphore()->give();

    // Register periodic callback to read sensor data
    _dev->register_periodic_callback(50000,  // 20Hz sampling rate
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Benewake_TFS20L::timer, void));

    return true;

fail:
    _dev->get_semaphore()->give();
    return false;
}

bool AP_RangeFinder_Benewake_TFS20L::read_registers(uint8_t reg_addr, uint8_t* data, uint8_t len)
{
    /*
     * Register-based I2C read for TFS20L
     * Use transfer method to write register address and read data
     */
    return _dev->transfer(&reg_addr, 1, data, len);
}

void AP_RangeFinder_Benewake_TFS20L::update()
{
    WITH_SEMAPHORE(_sem);

    if (accum.count > 0) {
        state.distance_m = (accum.sum * 0.01f) / accum.count;
        state.last_reading_ms = AP_HAL::millis();
        accum.sum = 0;
        accum.count = 0;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_Benewake_TFS20L::process_raw_measure(uint16_t distance_raw, uint16_t strength_raw,
                                                         uint16_t &output_distance_cm)
{
    output_distance_cm = distance_raw;

    if (strength_raw < MIN_STRENGTH || strength_raw == 0xFFFF || output_distance_cm > MAX_DIST_CM) {
        /*
         * When signal strength is too low or invalid, set distance to max + offset
         * This forces status to OutOfRangeHigh rather than NoData
         */
        output_distance_cm = MAX(MAX_DIST_CM, max_distance()*100 + BENEWAKE_OUT_OF_RANGE_ADD_CM);
    } else {
        output_distance_cm = constrain_int16(output_distance_cm, MIN_DIST_CM, MAX_DIST_CM);
    }
}

void AP_RangeFinder_Benewake_TFS20L::timer()
{
    uint8_t raw_data[6]; // Buffer for distance + strength + temperature data

    /*
     * Read the first 6 registers at once to get all data
     * TFS20L_DIST_LOW (0x00) through TFS20L_TEMP_HIGH (0x05)
     */
    if (!read_registers(TFS20L_DIST_LOW, raw_data, sizeof(raw_data))) {
        return;
    }

    // Combine bytes to form distance and strength (little-endian)
    uint16_t distance_cm = (uint16_t(raw_data[1]) << 8) | raw_data[0];
    uint16_t strength = (uint16_t(raw_data[3]) << 8) | raw_data[2];
    // Temperature data is also available in raw_data[4-5] if needed

    uint16_t processed_distance;
    process_raw_measure(distance_cm, strength, processed_distance);

    {
        WITH_SEMAPHORE(_sem);
        accum.sum += processed_distance;
        accum.count++;
    }
}

#endif  // AP_RANGEFINDER_BENEWAKE_TFS20L_ENABLED
