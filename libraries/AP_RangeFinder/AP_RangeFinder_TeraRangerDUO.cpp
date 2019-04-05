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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_TeraRangerDUO.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/crc.h>
#include <GCS_MAVLink/GCS.h>
#include <ctype.h>
#include <stdio.h>

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_TeraRangerDUO::AP_RangeFinder_TeraRangerDUO(RangeFinder::RangeFinder_State &_state,
                                                           AP_RangeFinder_Params &_params,
                                                           AP_SerialManager &serial_manager,
                                                           uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

// detect if a TeraRanger DUO rangefinder sensor is connected by looking for a configured serial port
bool AP_RangeFinder_TeraRangerDUO::detect(AP_SerialManager &serial_manager, uint8_t serial_instance)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// update the state of the sensor
void AP_RangeFinder_TeraRangerDUO::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // process incoming messages
    if (get_reading(state.distance_cm)) {
        _last_reading_ms = AP_HAL::millis();        
    }
    update_status();
}

// update status
void AP_RangeFinder_TeraRangerDUO::update_status()
{
    if (AP_HAL::millis() - _last_reading_ms > TRDUO_TIMEOUT_MS) {
        set_status(RangeFinder::RangeFinder_NoData);
    } else {
        if ((int16_t)state.distance_cm > TRDUO_MAX_DISTANCE_TOF) {
            set_status(RangeFinder::RangeFinder_OutOfRangeHigh);
        } else if ((int16_t)state.distance_cm < TRDUO_MIN_DISTANCE_SOUND) {
            set_status(RangeFinder::RangeFinder_OutOfRangeLow);
        } else {
            set_status(RangeFinder::RangeFinder_Good);
        }
    }
}

// read - return last value measured by sensor
bool AP_RangeFinder_TeraRangerDUO::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    float sum_tof = 0;
    float sum_sound = 0;
    uint16_t tof_count = 0;
    uint16_t sound_count = 0;
    int16_t nbytes = uart->available();

    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == 'T') {
            _buffer_count = 0;
            _found_start = true;
        }

        if (!_found_start) {
            continue;
        }

        _buffer[_buffer_count++] = c;
        
        if (_buffer_count >= TRDUO_BUFFER_SIZE_FULL) {
            // check if message has right CRC
            if (crc_crc8(_buffer, TRDUO_BUFFER_SIZE_FULL - 1) == _buffer[TRDUO_BUFFER_SIZE_FULL - 1]){
                uint16_t t_distance = process_distance(_buffer[1], _buffer[2]);
                uint16_t s_distance = process_distance(_buffer[4], _buffer[5]);

                // check ToF distance range
                if (t_distance > TRDUO_MIN_DISTANCE_TOF && t_distance < TRDUO_MAX_DISTANCE_TOF) {
                    sum_tof += t_distance;
                    tof_count++;
                } else {
                    // check sound distance range
                    if (s_distance > TRDUO_MIN_DISTANCE_SOUND && s_distance < TRDUO_MAX_DISTANCE_SOUND) {
                        sum_sound += s_distance;
                        sound_count++;
                    }
                }
            }
            _buffer_count = 0;
            _found_start = false;
        }
    }

    // we always use ToF(precise mode) distance
    if (sum_tof > 0) {
        reading_cm = sum_tof / tof_count;
        return (tof_count > 0);
    } else if (sum_sound > 0) {
        reading_cm = sum_sound / sound_count;
        return (sound_count > 0);
    }

    return false;
}

uint16_t AP_RangeFinder_TeraRangerDUO::process_distance(uint8_t buf1, uint8_t buf2)
{
    uint16_t val = buf1 << 8;
    val |= buf2;

    return val / TRDUO_VALUE_TO_CM_FACTOR;
}
