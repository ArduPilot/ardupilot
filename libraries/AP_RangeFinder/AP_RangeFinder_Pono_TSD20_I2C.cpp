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
 */
#include "AP_RangeFinder_Pono_TSD20_I2C.h"

#if AP_RANGEFINDER_PONO_TSD20_I2C_ENABLED

#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

static constexpr float TSD20_OUT_OF_RANGE_ADD_M = 1.0f;

// TSD20 register addresses
static constexpr uint8_t TSD20_REG_DIST_HIGH = 0x00;     // Distance high byte, low byte follows at 0x01
static constexpr uint8_t TSD20_REG_LASER = 0x02;         // Laser control: 1 = on, 0 = off
static constexpr uint8_t TSD20_REG_ID = 0x03;            // LiDAR ID, always 0x4A

static constexpr uint8_t TSD20_ID = 0x4A;
static constexpr uint8_t TSD20_LASER_ON = 0x01;

// Distance limits in millimetres. The sensor has a 50mm blind zone and a
// 20m maximum range on a bright target; readings outside this window are
// marked as out of range rather than averaged in.
static constexpr uint16_t MIN_DIST_MM = 50;
static constexpr uint16_t MAX_DIST_MM = 20000;

bool AP_RangeFinder_Pono_TSD20_I2C::init()
{
    {
        WITH_SEMAPHORE(dev.get_semaphore());
        dev.set_retries(3);

        // Read the ID register to detect if the sensor is present
        uint8_t id;
        if (!dev.read_registers(TSD20_REG_ID, &id, 1)) {
            return false;
        }
        if (id != TSD20_ID) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "TSD20: unexpected ID 0x%02X", unsigned(id));
            return false;
        }

        // The sensor only measures with the laser enabled
        if (!dev.write_register(TSD20_REG_LASER, TSD20_LASER_ON)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "TSD20: failed to enable laser");
            return false;
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TSD20: found on bus %u", unsigned(dev.bus_num()));
    }

    // The sensor updates at 200Hz; 50Hz sampling is plenty
    dev.register_periodic_callback(20000,
                                   FUNCTOR_BIND_MEMBER(&AP_RangeFinder_Pono_TSD20_I2C::timer, void));

    return true;
}

void AP_RangeFinder_Pono_TSD20_I2C::update()
{
    WITH_SEMAPHORE(_sem);

    const uint32_t total_count = accum.count + accum.out_of_range_count;
    if (total_count > 0) {
        if (accum.out_of_range_count == total_count) {
            // All readings were out of range
            state.distance_m = max_distance() + TSD20_OUT_OF_RANGE_ADD_M;
            set_status(RangeFinder::Status::OutOfRangeHigh);
        } else if (accum.count > 0) {
            // Some readings were valid
            state.distance_m = (accum.sum_mm * 0.001f) / accum.count;
            update_status();
        }
        state.last_reading_ms = AP_HAL::millis();
        accum.sum_mm = 0;
        accum.count = 0;
        accum.out_of_range_count = 0;
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::Status::NoData);
    }
}

void AP_RangeFinder_Pono_TSD20_I2C::timer()
{
    /*
     * Read both distance bytes in a single transaction so the sensor
     * cannot update the value between the high and low byte.
     */
    uint8_t raw_data[2];
    if (!dev.read_registers(TSD20_REG_DIST_HIGH, raw_data, sizeof(raw_data))) {
        return;
    }

    // Combine bytes to form the distance (big-endian, millimetres)
    uint16_t distance_mm = (uint16_t(raw_data[0]) << 8) | raw_data[1];

    /*
     * TODO: confirm the value reported with no target in I2C mode. The
     * UART protocol reports 50000, which falls outside MAX_DIST_MM and is
     * treated as out of range here. Readings inside the 50mm blind zone
     * are unreliable and rejected the same way.
     */
    if (distance_mm < MIN_DIST_MM || distance_mm > MAX_DIST_MM) {
        WITH_SEMAPHORE(_sem);
        accum.out_of_range_count++;
        return;
    }

    {
        WITH_SEMAPHORE(_sem);
        accum.sum_mm += distance_mm;
        accum.count++;
    }
}

#endif  // AP_RANGEFINDER_PONO_TSD20_I2C_ENABLED
