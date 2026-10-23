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

/*
  Driver for the TWTOF240UI I2C time-of-flight rangefinder.

  The module is a short-range (about 2.4 m) ToF sensor. A failed or
  out-of-range sample is reported as a distance above 3 m and is ignored.
  Init only checks that the sensor answers on I2C, so a bad first sample
  does not prevent detection.

  Protocol (I2C register writes, then a 5-byte read of register 0x81):
    0x80 = 0x01  start
    0x81 = 0x01  measure, millimetres
    distance_mm = buf[2] << 8 | buf[3]
 */
#include "AP_RangeFinder_TWTOF240UI.h"

#if AP_RANGEFINDER_TWTOF240UI_ENABLED

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Readings above this are the module's failure code, not a distance.
#define TOFM_DIST_FAIL_MM 3000

#define TOFM_CMD_START_FLAG 0x80
#define TOFM_CMD_ST_MM      0x81

bool AP_RangeFinder_TWTOF240UI::init(void)
{
    WITH_SEMAPHORE(dev.get_semaphore());

    if (!read_sample(nullptr)) {
        return false;
    }

    dev.register_periodic_callback(100000,
                                   FUNCTOR_BIND_MEMBER(&AP_RangeFinder_TWTOF240UI::timer, void));
    return true;
}

bool AP_RangeFinder_TWTOF240UI::read_sample(uint16_t *reading_mm)
{
    uint8_t buf[5];

    // Match the module sequence used by the original driver: two
    // command writes, then a read. A failed sample is rejected later
    // from the distance field; the writes themselves are not checked.
    dev.write_register(TOFM_CMD_START_FLAG, 0x01);
    hal.scheduler->delay_microseconds(10);

    dev.write_register(TOFM_CMD_ST_MM, 0x01);
    hal.scheduler->delay_microseconds(10);

    if (!dev.read_registers(TOFM_CMD_ST_MM, buf, sizeof(buf))) {
        return false;
    }

    if (reading_mm == nullptr) {
        return true;
    }

    const uint16_t dist_mm = uint16_t(buf[2] * 256 + buf[3]);
    if (dist_mm > TOFM_DIST_FAIL_MM) {
        return false;
    }

    *reading_mm = dist_mm;
    return true;
}

void AP_RangeFinder_TWTOF240UI::timer(void)
{
    uint16_t dist_mm;
    if (read_sample(&dist_mm)) {
        WITH_SEMAPHORE(_sem);
        distance_mm = dist_mm;
        new_distance = true;
        state.last_reading_ms = AP_HAL::millis();
    }
}

void AP_RangeFinder_TWTOF240UI::update(void)
{
    WITH_SEMAPHORE(_sem);
    if (new_distance) {
        state.distance_m = distance_mm * 0.001f;
        new_distance = false;
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 300) {
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_TWTOF240UI_ENABLED
