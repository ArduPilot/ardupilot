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
  driver for Lanbao PSK-CM8JL65-CC5 Lidar
 */
#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_Lanbao.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

/*
  this sensor has no way of reporting "out of range", it will keep
  reporting distances at of around 7 to 8 meters even when pointed at
  the sky. For this reason we limit the max range to 6 meters as
  otherwise we may be giving false data
 */
#define LANBAO_MAX_RANGE_CM 600

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_Lanbao::AP_RangeFinder_Lanbao(RangeFinder::RangeFinder_State &_state,
                                             AP_RangeFinder_Params &_params,
                                             uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        // always 115200
        uart->begin(115200);
    }
}

/* 
   detect if a rangefinder is connected. We'll detect by trying to
   take a reading on Serial. If we get a result the sensor is there.
*/
bool AP_RangeFinder_Lanbao::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}


// read - return last value measured by sensor
bool AP_RangeFinder_Lanbao::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_range = 0;
    uint32_t count = 0;

    // format is: [ 0xA5 | 0x5A | distance-MSB-mm | distance-LSB-mm | crc16 ]

    // read any available lines from the lidar
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        int16_t b = uart->read();
        if (b == -1) {
            break;
        }
        if (buf_len == 0 && b != 0xA5) {
            // discard
            continue;
        }
        if (buf_len == 1 && b != 0x5A) {
            // discard
            if (b == 0xA5) {
                buf[0] = b;
            } else {
                buf_len = 0;
            }
            continue;
        }
        buf[buf_len++] = b;
        if (buf_len == sizeof(buf)) {
            buf_len = 0;
            uint16_t crc = (buf[5]<<8) | buf[4];
            if (crc != calc_crc_modbus(buf, 4)) {
                // bad CRC, discard
                continue;
            }
            sum_range += float((buf[2]<<8) | buf[3]) * 0.001;
            count++;
        }
    }
    if (count > 0) {
        reading_cm = (sum_range / count) * 100;
        return reading_cm <= LANBAO_MAX_RANGE_CM?true:false;
    }
    return false;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_Lanbao::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
