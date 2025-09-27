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
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

#define DRIVER "TFS20L"
#define BENEWAKE_OUT_OF_RANGE_ADD_CM 100


// TFS20L register addresses
static const uint8_t TFS20L_DIST_LOW = 0x00;         // Distance low byte
static const uint8_t TFS20L_DIST_HIGH = 0x01;        // Distance high byte  
static const uint8_t TFS20L_AMP_LOW = 0x02;          // Signal strength/amplitude low byte
static const uint8_t TFS20L_AMP_HIGH = 0x03;         // Signal strength/amplitude high byte
static const uint8_t TFS20L_TEMP_LOW = 0x04;         // Temperature low byte
static const uint8_t TFS20L_TEMP_HIGH = 0x05;        // Temperature high byte
static const uint8_t TFS20L_VERSION_REVISION = 0x0A; // Version revision byte
static const uint8_t TFS20L_VERSION_MINOR = 0x0B;    // Version minor byte
static const uint8_t TFS20L_VERSION_MAJOR = 0x0C;    // Version major byte
static const uint8_t TFS20L_ENABLE = 0x25;           // Enable register

// Distance and strength limits
static const uint16_t MAX_DIST_CM = 2000;
static const uint16_t MIN_DIST_CM = 1;
static const uint16_t MIN_STRENGTH = 100;


AP_RangeFinder_Benewake_TFS20L::AP_RangeFinder_Benewake_TFS20L(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::I2CDevice *dev)
    : AP_RangeFinder_Backend(_state, _params)
    , _dev(dev)
{
}

AP_RangeFinder_Backend *AP_RangeFinder_Benewake_TFS20L::detect(
        RangeFinder::RangeFinder_State &_state,
        AP_RangeFinder_Params &_params,
        AP_HAL::I2CDevice *dev)
{
    if (dev == nullptr) {
        return nullptr;
    }
    AP_RangeFinder_Benewake_TFS20L *sensor = NEW_NOTHROW AP_RangeFinder_Benewake_TFS20L(_state, _params, dev);
    if (sensor == nullptr || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_RangeFinder_Benewake_TFS20L::init()
{
    uint8_t version_data[3];

    {
        WITH_SEMAPHORE(_dev->get_semaphore());
        _dev->set_retries(3);

        // Read firmware version to detect if sensor is present
        if (!_dev->read_registers(TFS20L_VERSION_MAJOR, version_data, 3)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TFS20L: No response from sensor");
            return false;
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TFS20L: found fw version %u.%u.%u",
                      version_data[0], version_data[1], version_data[2]);
    }

    // Register periodic callback to read sensor data
    _dev->register_periodic_callback(50000,  // 20Hz sampling rate
                                     FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Benewake_TFS20L::timer, void));

    return true;
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

void AP_RangeFinder_Benewake_TFS20L::timer()
{
    uint8_t raw_data[6]; // Buffer for distance + strength + temperature data

    /*
     * Read the first 6 registers at once to get all data
     * TFS20L_DIST_LOW (0x00) through TFS20L_TEMP_HIGH (0x05)
     */
    if (!_dev->read_registers(TFS20L_DIST_LOW, raw_data, sizeof(raw_data))) {
        return;
    }

    // Combine bytes to form distance and strength (little-endian)
    uint16_t distance_cm = (uint16_t(raw_data[1]) << 8) | raw_data[0];
    uint16_t strength = (uint16_t(raw_data[3]) << 8) | raw_data[2];
    // Temperature data is also available in raw_data[4-5] if needed

    // Validate reading and handle invalid cases
    if (strength < MIN_STRENGTH || strength == 0xFFFF || 
        distance_cm > MAX_DIST_CM || distance_cm < MIN_DIST_CM) {
        /*
         * When signal strength is too low or invalid, or distance is outside
         * sensor's specified range, set distance to max + offset.
         * This forces status to OutOfRangeHigh rather than NoData
         */
        distance_cm = MAX(MAX_DIST_CM, max_distance()*100 + BENEWAKE_OUT_OF_RANGE_ADD_CM);
    }

    {
        WITH_SEMAPHORE(_sem);
        accum.sum += distance_cm;
        accum.count++;
    }
}

#endif  // AP_RANGEFINDER_BENEWAKE_TFS20L_ENABLED
